#include <Arduino.h>
#include <queue>
#include "SparkFunLSM6DSO.h"
#include <math.h>
#include <AccelStepper.h>
#include "CoaxialSPM.hpp"
#include "sensor_fusion.hpp"

//--- Adjust Debugging levels Here (#define TRACE/DEBUG/INFO)
#define INFO

#ifdef TRACE
  #define DEBUG
#endif
#ifdef DEBUG
  #define INFO
#endif

// Pins
#define CS 1
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
#define IDLE_STATE 0
#define HOME_STATE 1
#define POSITION_STATE 2
#define TRAJECTORY_OPEN_STATE 3
#define TRAJECTORY_CLOSED_STATE 4
#define TEST_STATE 5

using namespace std;
using namespace Eigen;

extern uint8_t state;
extern uint8_t next_state;

// Global settings
extern bool flow_control_halt;
extern bool loop_timing_enabled;

extern Coaxial_SPM spm;

// Stepper motors
extern AccelStepper stepper_0;
extern AccelStepper stepper_1;
extern AccelStepper stepper_2;

extern std::queue<char> command_buffer;
extern unsigned long loop_start_time;
extern unsigned long loop_time_elapsed;
extern unsigned long loop_time_proc;

// IMU
extern LSM6DSO platformIMU;
extern Vector3f gyro_bias;

extern KalmanFilter<float, 4, 4> kalman;