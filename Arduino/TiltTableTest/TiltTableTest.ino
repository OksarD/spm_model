#include "project.hpp"
#include "helpers.hpp"

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

// IMU
LSM6DSO platformIMU;
Vector3f gyro_bias;

// Kalman Filter
Matrix4f Q = 1e-2 * Matrix4f::Identity();
Matrix4f R = 1 * Matrix4f::Identity();
Vector3f origin(0.0f,0.0f,0.0f);
Vector4f x0 = ypr_to_q(origin);
Matrix4f P0 = Matrix4f::Identity();
Matrix4f F0 = Matrix4f::Identity();
Matrix4f H = Matrix4f::Identity();

KalmanFilter<float, 4, 4> kalman(x0, P0, F0, H, Q, R);

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
      // Select states and set/reset loop timing from command prefix (first character of command)
      switch(command[0]) {
        case 'M': {
          movement_command = true;
        }
        case 'H': {
          next_state = HOME_STATE;
          disable_loop_timing();
          #ifdef INFO
          Serial.println("Device Homing.");
          #endif
          break;
        }
        case 'P': {
          next_state = POSITION_STATE;
          disable_loop_timing();
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
        case 'E': {
          enable_motors();
          break;
        }
        case 'D': {
          disable_motors();
          break;
        }
        case 'T': {
          next_state = TEST_STATE;
          disable_loop_timing();
          #ifdef INFO
          Serial.println("Test Mode Enabled");
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
        Vector3f ypr_meas = accel_ypr();
        position_control(ypr_home, ypr_meas);
        break;
      }
      case TRAJECTORY_OPEN_STATE: {
        int traj[6];
        extract_trajectory_command(traj, command); //get trajectory input in milliradians(/s)
        Vector3f ypr_ref(traj[0]*0.001, traj[1]*0.001, traj[2]*0.001); // ypr reference in radians
        Vector3f ypr_velocity_ref(traj[3]*0.001, traj[4]*0.001, traj[5]*0.001); // ypr derivative reference in rad/s
        open_trajectory_control(ypr_ref, ypr_velocity_ref);
      }
      case TEST_STATE: {
        stepper_1.setSpeed()
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
  state = next_state;
}
