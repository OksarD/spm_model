#include "nrf_rtc.h"
#include "stepper_driver.hpp"

StepperMotor::StepperMotor(uint8_t _stepPin, uint8_t _dirPin, uint8_t _sleepPin)
    : stepPin(_stepPin), dirPin(_dirPin), sleepPin(_sleepPin), direction(false), 
        position(0), step_state(false), halted(true) 
    {
        // Initialise pins
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(sleepPin, OUTPUT);
        setSpeed(0);
        set_dir(true);
        step();
    }

void StepperMotor::setSpeed(float speed) {
    if (abs(speed) < 1e-5) { 
        if (!halted) { // set flag when speed is close to zero
            halted = true; 
            timer->CC[ccIndex] = timer_current() - 1; // set compare reg behind timer so it takes max time to step
        }
    } else {
        intervalTicks = round(abs(speed_scale/speed));
        if (halted) {
            halted = false;
            prev_cc = timer->CC[ccIndex];
            timer->CC[ccIndex] = timer_current() + 1; // initialise CC so the interupt fires on the next tick
        } else {
            // if the new speed value is slow enough to be set immediately, then set the corresponding cc value,
            // if it is too fast, set the cc to the next tick
            uint32_t new_cc = prev_cc + intervalTicks;
            uint32_t current_cc = timer_current();
            if (new_cc > current_cc) {
                prev_cc = timer->CC[ccIndex];
                timer->CC[ccIndex] = new_cc;
            } else {
                prev_cc = timer->CC[ccIndex];
                timer->CC[ccIndex] = current_cc + 1;
            }
        }
        // set direction
        if (speed < 0) { set_dir(false); }
        else { set_dir(true); }
    }
}

void StepperMotor::enable() {
    digitalWrite(sleepPin, HIGH);
    enable_interrupt(ccIndex);
    timer->CC[ccIndex] = timer_current() - 1;
    halted = true;
}

void StepperMotor::disable() {
    digitalWrite(sleepPin, LOW);
    disable_interrupt(ccIndex);
}

void StepperMotor::debug_print() {
    Serial.print("ind: ");
    Serial.print(ccIndex);
    Serial.print(" timer: ");
    Serial.print(timer_current());
    Serial.print(" cc: ");
    Serial.print(timer->CC[ccIndex]);
    Serial.print(" step_state: ");
    Serial.print(step_state);
    Serial.print(" halt: ");
    Serial.print(halted);
    Serial.print(" dir: ");
    Serial.print(direction);
}

void StepperMotor::set_dir(bool dir) {
    if (direction != dir) {
        direction = dir;
        digitalWrite(dirPin, direction);
    }
}

void StepperMotor::step() {
    step_state = !step_state;
    digitalWrite(stepPin, step_state);
    if (direction) position++;
    else position--;
    prev_cc = timer->CC[ccIndex];
    timer->CC[ccIndex] += intervalTicks;
}

StepperDriver::StepperDriver(NRF_TIMER_Type* _timer, IRQn_Type _irq, uint8_t _reg_max, uint8_t _prescaler)
    : timer(_timer), irq(_irq), motorCount(0), reg_max(_reg_max), 
    prescaler(_prescaler), speed_scale(pow(2,(4-_prescaler))*1e6) {}

void StepperDriver::begin() {
    timer->TASKS_STOP  = 1;  // just in case it's running
    timer->TASKS_CLEAR = 1;
    timer->BITMODE = 3UL; // 32 bit
    timer->MODE = 0UL;    // timer, not counter
    timer->PRESCALER = prescaler; // freq = 16Mhz / 2^prescaler = 1Mhz 
    timer->INTENSET = 0; // NRF_RTC_INT_COMPARE0_MASK | NRF_RTC_INT_COMPARE1_MASK | NRF_RTC_INT_COMPARE2_MASK;
    NVIC_EnableIRQ(irq);
    timer->TASKS_START = 1;
}

void StepperDriver::add_motor(StepperMotor& m) {
    m.ccIndex = motorCount;
    motorCount ++;
    motors.push_back(&m);
    if (motors.size() >= reg_max) {
        Serial.println("Too many motors for this timer!");
    }
    m.timer = timer;
    m.reg_max = reg_max;
    timer->CC[m.ccIndex] = 0;
    m.speed_scale = speed_scale;
}

void StepperDriver::irq_handler() {
    for (uint8_t i=0; i<motorCount; i++) {
        if (timer->EVENTS_COMPARE[i] == 1) {   
            timer->EVENTS_COMPARE[i] = 0;
            motors[i]->step();
        }
    }
}

uint32_t StepperMotor::timer_current() {
    timer->TASKS_CAPTURE[5] = 1;
    return timer->CC[5];
}

void StepperMotor::enable_interrupt(uint8_t index) {
    timer->EVENTS_COMPARE[index] = 0;
    timer->INTENSET = (1UL << 16 + index);
}

void StepperMotor::disable_interrupt(uint8_t index) {
    timer->INTENCLR = (1UL << 16 + index);
}