#include "project.hpp"
#include "helpers.hpp"
#include "compensator.hpp"

#define SETTLE_UPDATES 50

uint8_t state = IDLE_STATE;
uint8_t next_state;

// Global settings
bool flow_control_halt = false;
bool loop_timing_enabled = false;

// Coaxial SPM object
float a1 = radians(50);
float a2 = radians(75);
float b = radians(100);
Coaxial_SPM spm(a1, a2, b);

// Stepper motors should be arranged around the Z axis (anti-clockwise from birds-eye view),
// starting with the motor connected to the +Y platform axis
StepperDriver driver(NRF_TIMER4, TIMER4_IRQn, 6, 7);
StepperMotor stepper_0(STEP_1, DIR_1, SLEEP_1);
StepperMotor stepper_1(STEP_2, DIR_2, SLEEP_2);
StepperMotor stepper_2(STEP_3, DIR_3, SLEEP_3);

extern "C" void TIMER4_IRQHandler_v() {
    driver.irq_handler();
}

std::queue<char> command_buffer;
unsigned long loop_start_time;
unsigned long loop_time_elapsed = 0;
unsigned long loop_time_proc;

// IMU
LSM6DSO platformIMU;
Vector3f gyro_bias(0, 0, 0);

// Kalman Filter
Matrix4f Q = 1e-4 * Matrix4f::Identity();
Matrix4f R = 1 * Matrix4f::Identity();
Vector3f origin(0.0f, 0.0f, 0.0f);
Quaternionf x0_q = ypr_to_q(origin);
Vector4f x0 = Vector4f(x0_q.w(), x0_q.x(), x0_q.y(), x0_q.z());
Matrix4f P0 = Matrix4f::Identity();
Matrix4f F0 = Matrix4f::Identity();
Matrix4f H = Matrix4f::Identity();

KalmanFilter<float, 4, 4> kalman(x0, P0, F0, H, Q, R);
KalmanFilter<float, 4, 4> kalman_predict(x0, P0, F0, H, Q, R);
// Lookup Table forward-kinematics
float fpk_xy0 = spm.actuator_origin - PI;  // table is centered around actuator origin, and extends to +/- pi
float fpk_dxy = 2 * PI / LOOKUP_TABLE_DIM;
LookupTable2D fpk_yaw_table(fpk_xy0, fpk_xy0, fpk_dxy, fpk_dxy,
                            LOOKUP_TABLE_DIM, LOOKUP_TABLE_DIM, yaw_table);

// controller
PID position_compensator(2, 0, 0, 0.1, 1);

void setup() {
  Serial.begin(115200);
  delay(1000);
  initializeMotors();
  Serial.println("Init IMU");
  Wire.begin();
  delay(10);
  if (platformIMU.begin())
    Serial.println("Ready.");
  else {
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if (platformIMU.initialize(BASIC_SETTINGS))
    Serial.println("Loaded Settings.");

  Serial.println("\nPlease enter a command: ");
}

void loop() {
  // Read serial data and place into buffer
  if (Serial.available()) {
    char command[COMMAND_MAX_SIZE];
    clearCharArray(command, sizeof(command));
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
  static Vector3f ypr_ref = Vector3f::Constant(0);
  static Vector3f ypr_velocity_ref = Vector3f::Constant(0);

  static bool movement_ongoing = false;
  char* command;
  if (loop_timing_proc()) {
#ifdef TRACE
    if (state != IDLE_STATE) {
      Serial.println("============================ DEBUG ============================");
    }
#endif
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
      switch (command[0]) {
        // set the control reference signals
        case 'M':
          {
            movement_ongoing = true;
            if (state == TRAJECTORY_OPEN_STATE || state == TRAJECTORY_CLOSED_STATE) {
              ypr_ref = extract_position(command);
              ypr_velocity_ref = extract_velocity(command);
            } else if (state == POSITION_STATE) {
              ypr_ref = extract_position(command);
            }
            break;
          }
        // enable/disable motors
        case 'E':
          {
            enable_motors();
            break;
          }
        case 'D':
          {
            disable_motors();
            break;
          }
        // State selection
        case 'H':
          {
            next_state = HOME_STATE;
            position_compensator.reset();
            halt_motors();
#ifdef INFO
            Serial.println("Device Homing... ");
#endif
            delay(500); // let robot settle
            gyro_bias = gyro_xyz(200);                                // average a lot of samples to accurately measure drift
            kalman = KalmanFilter<float, 4, 4>(x0, P0, F0, H, Q, R);
            kalman_predict = KalmanFilter<float, 4, 4>(x0, P0, F0, H, Q, R);  // reset kalman filter
            Quaternionf est;
            for (uint32_t i=0; i<SETTLE_UPDATES; i++) { // let kalman filter converge
              est = estimate(false);
              delayMicroseconds(LOOP_TIMING_INTERVAL);
            }                 
            enable_loop_timing();
#ifdef DEBUG
            Vector3f est_ypr = q_to_ypr(est);
            Serial.print("Gyro drift: ");
            Serial.print(gyro_bias[0], 3);
            Serial.print(", ");
            Serial.print(gyro_bias[1], 3);
            Serial.print(", ");
            Serial.print(gyro_bias[2], 3);
            Serial.print("\nConverged estimate: ");
            Serial.print(est_ypr[0], 3);
            Serial.print(", ");
            Serial.print(est_ypr[1], 3);
            Serial.print(", ");
            Serial.print(est_ypr[2], 3);
            Serial.println();
#endif
            break;
          }
        case 'P':
          {
            next_state = POSITION_STATE;
            enable_loop_timing();
            position_compensator.reset();
#ifdef INFO
            Serial.println("Position Control Enabled. Command (MY<>P<>R<>)");
#endif
            break;
          }
        case 'C':
          {
            next_state = TRAJECTORY_CLOSED_STATE;
            enable_loop_timing();
#ifdef INFO
            Serial.println("Closed-Loop Trajectory Control Enabled. Command (MY<>P<>R<>y<>p<>r<>)");
#endif
            break;
          }
        case 'O':
          {
            next_state = TRAJECTORY_OPEN_STATE;
            enable_loop_timing();
#ifdef INFO
            Serial.println("Open-Loop Trajectory Control Enabled. Command (MY<>P<>R<>y<>p<>r<>)");
#endif
            break;
          }
        case 'T':
          {
            next_state = TEST_STATE;
#ifdef INFO
            Serial.println("Test Mode Enabled");
#endif
            break;
          }
        case 'I':
          {
            next_state = IDLE_STATE;
#ifdef INFO
            Serial.println("Device Idle.");
#endif
            break;
          }
        default:
          {
#ifdef DEBUG
            Serial.print("Unknown Command ");
            Serial.println(command);
#endif
          }
      }
    }

    // Execute control actions based on state
    switch (state) {
      case IDLE_STATE:
        {
          halt_motors();
          movement_ongoing = false;
          break;
        }
      case HOME_STATE:
        {
          Quaternionf meas = estimate(false);
          Quaternionf ref_q = ypr_to_q(Vector3f::Constant(0));
          Quaternionf error = (ref_q * meas.conjugate()).normalized();
          position_control(error, meas, position_compensator);
          // reset actuator position after homed

          if (abs(q_to_aa(error)[0]) < POSITION_ANGLE_TOLERANCE) {
            next_state = IDLE_STATE;
            reset_actuator_position();
            Serial.println("#F"); // send hash command to python script
#ifdef INFO
            Serial.println("Homing done.");
#endif
          }
          break;
        }
      case POSITION_STATE:
        {
          if (movement_ongoing) {
            Quaternionf meas = estimate();
            Quaternionf ref_q = ypr_to_q(ypr_ref);
            Quaternionf error = (ref_q * meas.conjugate()).normalized();
            #ifdef DEBUG
            Vector3f ypr_meas = q_to_ypr(meas);
            Serial.print("meas: ");
            Serial.print(ypr_meas[0], 3);
            Serial.print(",");
            Serial.print(ypr_meas[1], 3);
            Serial.print(",");
            Serial.print(ypr_meas[2], 3);
            Serial.println();
            #endif
            position_control(error, meas, position_compensator);
            if (abs(q_to_aa(error)[0]) < POSITION_ANGLE_TOLERANCE) {
              Serial.println("#F"); // send hash command to python script
              halt_motors();
              movement_ongoing = false;
#ifdef INFO
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
      case TRAJECTORY_OPEN_STATE:
        {
          if (movement_ongoing) {
            //Vector3f ypr_meas = ypr_estimate(); // does the measurement delay make a difference?
            open_trajectory_control(ypr_ref, ypr_velocity_ref);
          } else {
            halt_motors();
          }
          break;
        }
      case TEST_STATE:
        {
          static bool print_accel = false;
          static bool print_gyro = false;
          static bool print_kalman = false;

          if (hasChar(command, ')')) { // shift+0
            test_motor(stepper_0, 0);
          }
          if (hasChar(command, '!')) { // shift+1
            test_motor(stepper_1, 1);
          }
          if (hasChar(command, '@')) { // shift+2
            test_motor(stepper_2, 2);
          }
          if (hasChar(command, 'A')) {
            print_accel = !print_accel;
          }
          if (hasChar(command, 'G')) {
            print_gyro = !print_gyro;
          }
          if (hasChar(command, 'K')) {
            print_kalman = !print_kalman;
          }
          if (hasChar(command, 'F')) {
            // test actuator values for fpk table
            float fpk_yaw = interp_yaw_fpk();
            Serial.print("act_pos: ");
            Serial.print(actuator_position()[0]);
            Serial.print(", ");
            Serial.print(actuator_position()[1]);
            Serial.print(", ");
            Serial.print(actuator_position()[2]);
            Serial.print("\nfpk_yaw test ");
            Serial.println(fpk_yaw, 3);
          }
          if (hasChar(command, 'f')) {
            uint32_t i = uint32_t(ExtractValue(command, 'f'));
            uint32_t j = uint32_t(ExtractValue(command, ','));
            // test actuator values for fpk table
            float fpk_yaw = fpk_yaw_table.at(i,j);
            Serial.print("i, j: ");
            Serial.print(i);
            Serial.print(", ");
            Serial.print(j);
            Serial.print("\nfpk_yaw test (index) ");
            Serial.println(fpk_yaw, 3);
          }

          // Printing
          if (print_accel) {
            Vector3f accel = accel_ypr();
            Serial.print("Accel_ypr: ");
            Serial.print(accel[0], 3);
            Serial.print(",");
            Serial.print(accel[1], 3);
            Serial.print(",");
            Serial.print(accel[2], 3);
            Serial.println();
          }
          if (print_gyro) {
            Vector3f gyro = gyro_xyz();
            Serial.print("gyro_xyz: ");
            Serial.print(gyro[0], 3);
            Serial.print(",");
            Serial.print(gyro[1], 3);
            Serial.print(",");
            Serial.print(gyro[2], 3);
            Serial.println();
          }
          if (print_kalman) {
            Vector3f kal = q_to_ypr(estimate());
            Serial.print("kal: ");
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
  yield();
  // update state machine
  state = next_state;
}
