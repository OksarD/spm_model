#include <Arduino_LSM9DS1.h>
#include <math.h>
#include <AccelStepper.h>
#include "CoaxialSPM.hpp"
#include <queue>
#include <vector>

//--- Adjust Debugging levels Here (#define TRACE/DEBUG/INFO)
#define INFO


#ifdef TRACE
  #define DEBUG
#endif
#ifdef DEBUG
  #define INFO
#endif

// Pins
#define DIR_1 2 
#define STEP_1 3 //PWM
#define SLEEP_1 4
#define DIR_2 5
#define STEP_2 6 //PWM
#define SLEEP_2 7
#define DIR_3 8
#define STEP_3 9 //PWM
#define SLEEP_3 10
#define MICROSTEP 8 // Microstepping is hardwired to 8 (M1/M0 set HIGH)
#define MOTOR_STEPS 200 // Motor Steps per revolution
#define MAX_SPEED 5000
#define ROT_SCALE 5.2 // temporary pulley with 20 teeth
#define RAD_TO_DEG 57.29577951308232  // Constant to convert radians to degrees

// Fifo buffer parameters
#define COMMAND_BUFFER_FULL_SIZE 512 // approx 16 commands at roughly 32 bytes per command
#define COMMAND_BUFFER_EMPTY_SIZE 256 // approx 16 commands at roughly 32 bytes per command
#define COMMAND_MAX_SIZE 128 // usually around 32 bytes
#define XON 0x11
#define XOFF 0x13
#define LOOP_TIMING_INTERVAL 20000 // microseconds 
#define DELIM '\n'

// States
#define DEFAULT_STATE 0
#define HOME_STATE 1
#define POSITION_STATE 2
#define TRAJECTORY_OPEN_STATE 3
#define TRAJECTORY_CLOSED_STATE 4

uint8_t state = DEFAULT_STATE;
uint8_t next_state;

// Global settings
bool flow_control_halt = false;
bool loop_timing_enabled = false;


// Coaxial SPM object
float a1 = radians(50);
float a2 = radians(75);
float b  = radians(100);
Coaxial_SPM spm(a1, a2, b);

// Stepper motors
AccelStepper stepper_1(AccelStepper::DRIVER, STEP_1, DIR_1);
AccelStepper stepper_2(AccelStepper::DRIVER, STEP_2, DIR_2);
AccelStepper stepper_3(AccelStepper::DRIVER, STEP_3, DIR_3);

std::queue<char> command_buffer;
unsigned long loop_start_time;
unsigned long loop_time_elapsed = 0;
unsigned long loop_time_proc;

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino is ready...");
  initializeMotors();
  Serial.println("\nPlease enter a command: ");
}

void loop() {
  // Read serial data and place into buffer
  if(Serial.available()) {
    char command[COMMAND_MAX_SIZE];
    clearCharArray(command,sizeof(command));
    int command_length = Serial.readBytesUntil(DELIM, command, sizeof(command));
    buffer_push(command_length, command);
  }
  
  // flow control
  if (command_buffer.size() >= COMMAND_BUFFER_FULL_SIZE && flow_control_halt == false) {
    Serial.write(XOFF);
    flow_control_halt = true;
    #ifdef TRACE
    Serial.println("XOFF sent");
    #endif
  } else if (command_buffer.size() < COMMAND_BUFFER_EMPTY_SIZE && flow_control_halt == true) {
    Serial.write(XON);
    flow_control_halt = false;
    #ifdef TRACE
    Serial.println("XON sent");
    #endif
  }
  
  bool movement_command = false;
  char* command;
  if(loop_timing_proc()) {
    // code to run every loop timing proc
    if (!command_buffer.empty()) {
      #ifdef TRACE
      Serial.print("Buffer length ");
      Serial.println(command_buffer.size());
      #endif
      command = buffer_pop();
      #ifdef TRACE
      Serial.print("Popped Command ");
      Serial.println(command);
      #endif
      // Select states or settings from command prefix (first character of command)
      switch(command[0]) {
        case 'M': {
          movement_command = true;
        }
        case 'H': {
          #ifdef INFO
          Serial.println("Device Homing.");
          #endif
          break;
          next_state = HOME_STATE;
          break;
        }
        case 'P': {
          next_state = POSITION_STATE;
          #ifdef INFO
          Serial.println("Position Control Enabled. Command (MY<>P<>R<>)");
          #endif
          break;
        }
        case 'T': {
          #ifdef INFO
          Serial.println("Closed-Loop Trajectory Control Enabled. Command (MY<>P<>R<>y<>p<>r<>)");
          #endif
          next_state = TRAJECTORY_CLOSED_STATE;
          break;
        }
        case 'O': {
          #ifdef INFO
          Serial.println("Open-Loop Trajectory Control Enabled. Command (MY<>P<>R<>y<>p<>r<>)");
          #endif
          next_state = TRAJECTORY_OPEN_STATE;
          break;
        }
        case 'E': {
          enable_motors();
          break;
        }
        case 'D': {
          disable_motors();
          break;
        }
        case 'L': {
          loop_timing_enabled = true;
          loop_start_time = micros();
          loop_time_proc = 0;
          #ifdef INFO
          Serial.println("Loop timing enabled");
          #endif
          break;
        }
        case 'X': {
          loop_timing_enabled = false;
          #ifdef INFO
          Serial.println("Loop timing disabled");
          #endif
          break;
        }
        default: {
          #ifdef DEBUG
          Serial.print("Unknown Command ");
          Serial.println(command);
          #endif
        }
      }
    }

    // Execute control actions based on state
    switch(state) {
      case DEFAULT_STATE: {
        // do nothing
        break;
      }
      case HOME_STATE: {
        Vector3f ypr_home(0,0,0);
        position_control(ypr_home);
        break;
      }
      case TRAJECTORY_OPEN_STATE: {
        int traj[6];
        extract_trajectory_command(traj, command); //get trajectory input in milliradians(/s)
        Vector3f ypr(traj[0]*0.001, traj[1]*0.001, traj[2]*0.001); // ypr reference in radians
        Vector3f ypr_velocity(traj[3]*0.001, traj[4]*0.001, traj[5]*0.001); // ypr derivative reference in rad/s
        open_trajectory_control(ypr, ypr_velocity);
      }
    }
  }
  // Outside loop timing (run every loop cycle)
  // Polling for stepper motors
  stepper_1.runSpeed(); 
  stepper_2.runSpeed();
  stepper_3.runSpeed();
  yield();
  // update state machine
  update_state();
}
