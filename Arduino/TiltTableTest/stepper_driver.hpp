#pragma once
#include <Arduino.h>
#include <nrf_timer.h>
#include <vector>

using namespace std;

class StepperMotor {
public:
    friend class StepperDriver;
    volatile long position;

    StepperMotor(uint8_t _stepPin, uint8_t _dirPin, uint8_t _sleepPin);

    void setSpeed(float speed);
    void enable();
    void disable();
    void debug_print();

private:
    uint32_t int_masks[4];
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t sleepPin;
    uint32_t intervalTicks;   // ticks between steps
    uint32_t cc;     
    uint8_t ccIndex;          // which CC register to use
    bool direction;           // true = forward
    volatile bool step_state;
    bool halted;
    uint8_t reg_max;
    NRF_TIMER_Type* timer;
    float speed_scale;
    uint32_t prev_cc;

    void set_dir(bool dir);
    void step();

    uint32_t timer_current();
    void enable_interrupt(uint8_t index);
    void disable_interrupt(uint8_t index);
};

class StepperDriver {
public:
    friend class StepperMotor;
    StepperDriver(NRF_TIMER_Type* _timer, IRQn_Type _irq, uint8_t _reg_max, uint8_t _prescaler);

    void begin();
    void add_motor(StepperMotor& m);
    void irq_handler();

private:
    uint8_t reg_max;
    NRF_TIMER_Type* timer;
    IRQn_Type irq;
    uint8_t motorCount;
    vector<StepperMotor*> motors; // reserve the last register for capture
    float speed_scale;
    uint8_t prescaler;

};