#include "project.hpp"
#include "helpers.hpp"

uint8_t state = IDLE_STATE;
uint8_t next_state;

// Global settings
bool flow_control_halt = false;
bool loop_timing_enabled = false;

// Coaxial SPM object
float a1 = radians(50);
float a2 = radians(75);
float b  = radians(100);
Coaxial_SPM spm(a1, a2, b);

// Stepper motors should be arranged around the Z axis (anti-clockwise from birds-eye view), 
// starting with the motor connected to the +Y platform axis
AccelStepper stepper_0(AccelStepper::DRIVER, STEP_1, DIR_1); 
AccelStepper stepper_1(AccelStepper::DRIVER, STEP_2, DIR_2);
AccelStepper stepper_2(AccelStepper::DRIVER, STEP_3, DIR_3);

std::queue<char> command_buffer;
unsigned long loop_start_time;
unsigned long loop_time_elapsed = 0;
unsigned long loop_time_proc;

// IMU
LSM6DSO platformIMU;
Vector3f gyro_bias(0,0,0);
Vector3f ypr_zero(0,0,0);

// Kalman Filter
Matrix4f Q = 1e-2 * Matrix4f::Identity();
Matrix4f R = 1 * Matrix4f::Identity();
Vector3f origin(0.0f,0.0f,0.0f);
Vector4f x0 = ypr_to_q(origin);
Matrix4f P0 = Matrix4f::Identity();
Matrix4f F0 = Matrix4f::Identity();
Matrix4f H = Matrix4f::Identity();

KalmanFilter<float, 4, 4> kalman(x0, P0, F0, H, Q, R);

// Lookup Table forward-kinematics
float fpk_xy0 = spm.actuator_origin - PI; // table is centered around actuator origin, and extends to +/- pi
float fpk_dxy = 2*PI/LOOKUP_TABLE_DIM;
LookupTable2D fpk_yaw_table(fpk_xy0, fpk_xy0, fpk_dxy, fpk_dxy,
                  LOOKUP_TABLE_DIM, LOOKUP_TABLE_DIM, yaw_table);

void setup() {
  Serial.begin(115200);
  delay(1000); 
  initializeMotors();
  Serial.println("Init IMU");
  Wire.begin();
  delay(10);
  if(platformIMU.begin())
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( platformIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");

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
  
  // Reference signals
  static Vector3f ypr_ref = ypr_zero;
  static Vector3f ypr_velocity_ref = ypr_zero;

  static bool movement_ongoing = false;
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
      // Read first character of command to perform function or set states
      switch(command[0]) {
        // set the control reference signals
        case 'M': {
          movement_ongoing = true;
          if (state == TRAJECTORY_OPEN_STATE || state == TRAJECTORY_CLOSED_STATE ) { 
            ypr_ref = extract_position(command);
            ypr_velocity_ref = extract_velocity(command);
          }
          else if (state == POSITION_STATE) {
            ypr_ref = extract_position(command);
          }
          break;
        }
        // enable/disable motors
        case 'E': {
          enable_motors();
          break;
        }
        case 'D': {
          disable_motors();
          break;
        }
        // State selection
        case 'H': {
          next_state = HOME_STATE;
          halt_motors();
          #ifdef INFO
          Serial.println("Device Homing... ");
          #endif
          delay(1000); // let the device settle before measuring gyro drift
          gyro_bias = gyro_xyz(100); // average a lot of samples to accurately measure drift
          kalman = KalmanFilter<float, 4, 4>(x0, P0, F0, H, Q, R); // reset kalman filter
          delay(2000); // let kalman filter converge
          enable_loop_timing();
          #ifdef INFO
          Serial.print("Gyro drift: ");
          Serial.print(gyro_bias[0], 3);
          Serial.print(", ");
          Serial.print(gyro_bias[1], 3);
          Serial.print(", ");
          Serial.print(gyro_bias[2], 3);
          Serial.println();
          #endif
          break;
        }
        case 'P': {
          next_state = POSITION_STATE;
          enable_loop_timing();
          #ifdef INFO
          Serial.println("Position Control Enabled. Command (MY<>P<>R<>)");
          #endif
          break;
        }
        case 'C': {
          next_state = TRAJECTORY_CLOSED_STATE;
          enable_loop_timing();
          #ifdef INFO
          Serial.println("Closed-Loop Trajectory Control Enabled. Command (MY<>P<>R<>y<>p<>r<>)");
          #endif
          break;
        }
        case 'O': {
          next_state = TRAJECTORY_OPEN_STATE;
          enable_loop_timing();
          #ifdef INFO
          Serial.println("Open-Loop Trajectory Control Enabled. Command (MY<>P<>R<>y<>p<>r<>)");
          #endif
          break;
        }
        case 'T': {
          next_state = TEST_STATE;
          #ifdef INFO
          Serial.println("Test Mode Enabled");
          #endif
          break;
        }
        case 'I': {
          next_state = IDLE_STATE;
          #ifdef INFO
          Serial.println("Device Idle.");
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
      case IDLE_STATE: {
        halt_motors();
        movement_ongoing = false;
        break;
      }
      case HOME_STATE: {
        Vector3f ypr_meas = ypr_estimate(false);
        Vector3f error = subtract_angles(ypr_zero, ypr_meas);
        position_control(error, ypr_meas);
        float tolerance = radians(0.25);
        // reset actuator position after homed
        if (abs(error[1]) < tolerance && abs(error[2]) < tolerance) {
          next_state = IDLE_STATE;
          reset_actuator_position();
          #ifdef INFO
          Serial.println("Homing done.");
          #endif
        }
        break;
      }
      case POSITION_STATE: {
        if (movement_ongoing) {
          Vector3f ypr_meas = ypr_estimate();
          Vector3f error = subtract_angles(ypr_ref, ypr_meas);
          position_control(error, ypr_estimate());
          float tolerance = radians(0.25);
          // move to idle state when done
          if (abs(error[0]) < tolerance && abs(error[1]) < tolerance && abs(error[2]) < tolerance) {
            #ifdef INFO
            movement_ongoing = false;
            halt_motors();
            Serial.print("Moved to ");
            Serial.print(ypr_ref[0]);
            Serial.print(", ");
            Serial.print(ypr_ref[1]);
            Serial.print(", ");
            Serial.print(ypr_ref[2]);
            Serial.println();
            #endif
          }
          break;
        }
      }
      case TRAJECTORY_OPEN_STATE: {
        if (movement_ongoing) {
          open_trajectory_control(ypr_ref, ypr_velocity_ref);
        } else {
          halt_motors();
        }
        break;
      }
      case TEST_STATE: {
        static bool print_accel = false;
        static bool print_gyro = false;
        static bool print_kalman = false;

        if(hasChar(command, '0')) {
          stepper_0.setSpeed(actuator_to_motor_speed(radians(20)));
        }
        if(hasChar(command, '1')) {
          stepper_1.setSpeed(actuator_to_motor_speed(radians(20)));
        }
        if(hasChar(command, '2')) {
          stepper_2.setSpeed(actuator_to_motor_speed(radians(20)));
        }
        if(hasChar(command, 'A')) {
          print_accel = !print_accel;
        }
        if(hasChar(command, 'G')) {
          print_gyro = !print_gyro;
        }
        if(hasChar(command, 'K')) {
          print_kalman = !print_kalman;
        }

        // Printing
        if(print_accel) {
          Vector3f accel = accel_ypr();
          Serial.print("Accel_ypr: ");
          Serial.print(accel[0], 3);
          Serial.print(",");
          Serial.print(accel[1], 3);
          Serial.print(",");
          Serial.print(accel[2], 3);
          Serial.println();
        }
        if(print_gyro) {
          Vector3f gyro = gyro_xyz();
          Serial.print("gyro_xyz: ");
          Serial.print(gyro[0], 3);
          Serial.print(",");
          Serial.print(gyro[1], 3);
          Serial.print(",");
          Serial.print(gyro[2], 3);
          Serial.println();
        }
        if(print_kalman) {
          Vector3f kal = ypr_estimate();
          Serial.print("kalman_ypr: ");
          Serial.print(kal[0], 3);
          Serial.print(",");
          Serial.print(kal[1], 3);
          Serial.print(",");
          Serial.print(kal[2], 3);
          Serial.println();
        }
        break;
      }
    }
  }
  // Outside loop timing (run every loop cycle)
  // Polling for stepper motors
  stepper_0.runSpeed(); 
  stepper_1.runSpeed();
  stepper_2.runSpeed();
  yield();
  // update state machine
  state = next_state;
}
