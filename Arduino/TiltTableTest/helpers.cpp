#include "helpers.hpp"

void initializeMotors() {
  // You could configure the motor drivers here, set microstepping, etc.
    stepper_0.setMaxSpeed(MAX_SPEED);
    stepper_1.setMaxSpeed(MAX_SPEED);
    stepper_2.setMaxSpeed(MAX_SPEED);
    pinMode(SLEEP_1, OUTPUT);
    pinMode(SLEEP_2, OUTPUT);
    pinMode(SLEEP_3, OUTPUT);
    Serial.println("Stepper Motors initialised.");
}

void poll_steppers() {
  stepper_0.runSpeed();
  stepper_1.runSpeed();
  stepper_2.runSpeed();
}

float ExtractValue(const char* linea, char eje) {
  const char* index = strchr(linea, eje);
  if (index == NULL) {
    return 0;  
  }
  return atof(index + 1);
}

bool hasChar(const char* linea, char eje) {
  const char* index = strchr(linea, eje);
  if (index == NULL) {
    return 0;  
  }
  return 1;
}

void clearCharArray(char* charArray, size_t size) {
  for (size_t i = 0; i < size; i++) {
    charArray[i] = '\0';  // Fill each element with null character
  }
}

Vector3f extract_position(char* command) {
  Vector3f pos(ExtractValue(command, 'Y'),
              ExtractValue(command, 'P'),
              ExtractValue(command, 'R')
  );
  return pos * 0.001; // convert to rad/s
}

Vector3f extract_velocity(char* command) {
  Vector3f vel(ExtractValue(command, 'y'),
              ExtractValue(command, 'p'),
              ExtractValue(command, 'r')
  );
  return vel * 0.001; // convert to rad/s
}

void disable_motors(){
  digitalWrite(SLEEP_1, LOW);
  digitalWrite(SLEEP_2, LOW);
  digitalWrite(SLEEP_3, LOW);
  halt_motors();
  #ifdef DEBUG
  Serial.println("Motors Disabled.");
  #endif
}

void halt_motors() {
  stepper_0.setSpeed(0);
  stepper_1.setSpeed(0);
  stepper_2.setSpeed(0);
}

void enable_motors() {
  digitalWrite(SLEEP_1, HIGH);
  digitalWrite(SLEEP_2, HIGH);
  digitalWrite(SLEEP_3, HIGH);
  halt_motors();
  #ifdef DEBUG
  Serial.println("Motors Enabled.");
  #endif
}

void buffer_push(unsigned int length, char* items) {
  for (unsigned int i=0; i<length; i++) {
    command_buffer.push(items[i]);
  }
  command_buffer.push(DELIM); //add back the delimiter
}

char* buffer_pop() {
  static char command[COMMAND_MAX_SIZE];
  byte index = 0;
  clearCharArray(command,sizeof(command));
  // pop until the start character is reached
  while (!command_buffer.empty() && command_buffer.front() != DELIM) {
      command[index] = command_buffer.front();
      command_buffer.pop();
      index++;
  }
  if (!command_buffer.empty()) {
    command[index] = command_buffer.front();
    command_buffer.pop(); // pop the delimiter character as well
  }
  return command;
}

void set_actuator_velocity(Vector3f actuator_velocity) {
  stepper_0.setSpeed(actuator_to_motor_speed(actuator_velocity[0]));
  stepper_1.setSpeed(actuator_to_motor_speed(actuator_velocity[1]));
  stepper_2.setSpeed(actuator_to_motor_speed(actuator_velocity[2]));
}

void reset_actuator_position() {
  stepper_0.setCurrentPosition(actuator_to_motor_position(spm.actuator_origin));
  stepper_1.setCurrentPosition(actuator_to_motor_position(spm.actuator_origin));
  stepper_2.setCurrentPosition(actuator_to_motor_position(spm.actuator_origin));
  #ifdef INFO
  Serial.print("Actuators reset to origin position: ");
  Serial.println(spm.actuator_origin, 3);
  #endif
}

Vector3f actuator_position() {
  Vector3f pos(motor_to_actuator_position(stepper_0.currentPosition()),
          motor_to_actuator_position(stepper_1.currentPosition()),
          motor_to_actuator_position(stepper_2.currentPosition())
  );
  return pos;
}

void enable_loop_timing() {
  loop_timing_enabled = true;
  loop_start_time = micros();
  loop_time_proc = 0;
  #ifdef DEBUG
  Serial.println("Loop timing enabled.");
  #endif
}

void disable_loop_timing() {
  if(loop_timing_enabled) {
    loop_timing_enabled = false;
    #ifdef DEBUG
    Serial.println("Loop timing disabled.");
    #endif
  }
}

bool loop_timing_proc() {
  if (!loop_timing_enabled) {
    return true; // always allow loop function when loop timing is disabled
  } else {
    loop_time_elapsed = micros() - loop_start_time; // elapsed time since session start
    if (loop_time_elapsed - loop_time_proc > LOOP_TIMING_INTERVAL) {
      loop_time_proc += LOOP_TIMING_INTERVAL;
      #ifdef TRACE
      Serial.println("Loop timing proc");
      #endif
      return true; // enable buffer read when loop timing has started
    } else {
      return false; // prevent buffer read
    }
  }
}

// Estimation functions

Vector3f accel_ypr(unsigned int samples) {
  Vector3f accel(0,0,0);
  // Get average accel reading
  for(uint8_t i=0; i<samples; i++) {
    accel[0] += platformIMU.readFloatAccelX();
    poll_steppers();
    accel[1] += platformIMU.readFloatAccelY();
    poll_steppers();
    accel[2] += platformIMU.readFloatAccelZ();
    poll_steppers();
  }
  if(samples > 1) {
    accel /= samples;
  }
  // unwind roll and pitch in the right-hand ENU frame
  Vector3f ypr;
  ypr[0] = 0; // do not set yaw
  ypr[1] = -atan2(accel[0], accel[2]);
  ypr[2] = atan2(accel[1], sqrt(pow(accel[0],2) + pow(accel[2],2)));
  return ypr;
}

Vector3f gyro_xyz(unsigned int samples) {
  Vector3f gyro(0,0,0);
  for(uint8_t i=0; i<samples; i++) {
    gyro[0] += radians(platformIMU.readFloatGyroX());
    poll_steppers();
    gyro[1] += radians(platformIMU.readFloatGyroY());
    poll_steppers(); 
    gyro[2] += radians(platformIMU.readFloatGyroZ());
    poll_steppers();
  }
  if(samples > 1) {
    gyro /= samples;
  }
  return gyro;
}

Vector3f ypr_estimate(bool include_yaw_fpk) {
  Vector3f ypr_meas = accel_ypr();
  if (include_yaw_fpk) { 
    ypr_meas[0] = interp_yaw_fpk();
  }
  Vector3f gyro = gyro_xyz() - gyro_bias;
  kalman.F = gyro_transition_matrix(gyro, LOOP_TIMING_INTERVAL/1e6);
  kalman.predict();
  #ifdef DEBUG
  Serial.print("ypr_meas ");
  Serial.print(ypr_meas[0], 3);
  Serial.print(", ");
  Serial.print(ypr_meas[1], 3);
  Serial.print(", ");
  Serial.print(ypr_meas[2], 3);
  Serial.print(" predict: ");
  Serial.print(kalman.state()[0], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[1], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[2], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[3], 3);
  #endif
  kalman.correct(ypr_to_q(ypr_meas));
  kalman.x = unit_q(kalman.x);
  Vector3f k_state = q_to_ypr(kalman.state());
  #ifdef DEBUG
  Serial.print(" correct: ");
  Serial.print(kalman.state()[0], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[1], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[2], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[3], 3);
  Serial.print(" ypr_correct: ");
  Serial.print(k_state[0], 3);
  Serial.print(", ");
  Serial.print(k_state[1], 3);
  Serial.print(", ");
  Serial.print(k_state[2], 3);
  Serial.println();
  #endif
  return k_state;
}

float interp_yaw_fpk() {
  Vector3f act_pos = actuator_position();
  float offset = spm.actuator_direction*(act_pos[0] - spm.actuator_origin);
  Vector3f actuator_offset = act_pos + Vector3f::Constant(offset); // offset the actuator position such that m0 is placed at the origin
  float yaw_offset = fpk_yaw_table.interp(actuator_offset[1], actuator_offset[2]); // fpk table is in the m1/m2 domain
  if (yaw_offset < -PI || yaw_offset > PI) { // if yaw value out-of-bounds (NAN)
    disable_motors();
    next_state = IDLE_STATE;
    Serial.println("Platform Out of Bounds! Ensure that position commands have less than a 40 degree slope.");
    return FPK_NAN_CODE;
  }
  #ifdef DEBUG
  Serial.print("act_pos: ");
  Serial.print(act_pos[0], 3);
  Serial.print(", ");
  Serial.print(act_pos[1], 3);
  Serial.print(", ");
  Serial.print(act_pos[2], 3);
  Serial.print(" offset: ");
  Serial.print(offset, 3);
  Serial.print(" act_offset: ");
  Serial.print(actuator_offset[0], 3);
  Serial.print(",");
  Serial.print(actuator_offset[1], 3);
  Serial.print(",");
  Serial.print(actuator_offset[2], 3);
  Serial.print(" yaw_offset: ");
  Serial.print(yaw_offset, 3);
  Serial.print(" yaw_interp: ");
  Serial.print(wrap_rad(yaw_offset + offset), 3);
  Serial.println();
  #endif
  return wrap_rad(yaw_offset + offset); // add the offset back to obtain the true yaw value
}

// Control functions
void position_control(Vector3f ypr_error, Vector3f ypr_meas) {
  Vector3f control_signal = 1 * ypr_error; // proportional controller
  Matrix3f R_mat = spm.R_ypr(ypr_meas);
  Vector3f xyz_platform_velocity = spm.ypr_to_xyz_velocity(control_signal, ypr_meas);
  Vector3f actuator_velocity = spm.solve_ivk(R_mat, xyz_platform_velocity);
  set_actuator_velocity(actuator_velocity);
  #ifdef DEBUG
  Serial.print("Error: ");
  Serial.print(ypr_error[0], 3);
  Serial.print(", ");
  Serial.print(ypr_error[1], 3);
  Serial.print(", ");
  Serial.print(ypr_error[2], 3);
  Serial.print(" Control: ");
  Serial.print(control_signal[0], 3);
  Serial.print(", ");
  Serial.print(control_signal[1], 3);
  Serial.print(", ");
  Serial.print(control_signal[2], 3);
  Serial.print(" Act Vel: ");
  Serial.print(actuator_velocity[0], 3);
  Serial.print(",");
  Serial.print(actuator_velocity[1], 3);
  Serial.print(",");
  Serial.print(actuator_velocity[2], 3);
  Serial.println();
  #endif
}

void open_trajectory_control(Vector3f ypr_ref, Vector3f ypr_velocity_ref) {
  Matrix3f R_mat = spm.R_ypr(ypr_ref);
  Vector3f xyz_platform_velocity = spm.ypr_to_xyz_velocity(ypr_velocity_ref, ypr_ref);
  Vector3f actuator_velocity = spm.solve_ivk(R_mat, xyz_platform_velocity);
  set_actuator_velocity(actuator_velocity);
  #ifdef INFO
  Serial.print(loop_time_elapsed);
  Serial.print(",");
  Serial.print(actuator_velocity[0], 3);
  Serial.print(",");
  Serial.print(actuator_velocity[1], 3);
  Serial.print(",");
  Serial.print(actuator_velocity[2], 3);
  Serial.println();
  #endif
}

void test_motor(AccelStepper& m, uint8_t ind) {
  //float start_pos = actuator_position()[ind];
  long start_pos = m.currentPosition();
  #ifdef INFO
  Serial.print("Stepper ");
  Serial.print(ind);
  Serial.print(" Start pos: ");
  Serial.print(start_pos);
  #endif
  static unsigned long time_start = millis();
  m.setSpeed(actuator_to_motor_speed(radians(30))); // move +30 degrees/s for 1 second around control axis (-Z)
  while(millis() - time_start < 1e3) {
    m.runSpeed();
    yield();
  }
  m.setSpeed(0);
  //float end_pos = actuator_position()[ind];
  long end_pos = m.currentPosition();
  #ifdef INFO
  Serial.print(" End pos: ");
  Serial.print(end_pos);
  Serial.print(" Delta: ");
  Serial.print(end_pos - start_pos);
  Serial.println();
  #endif
}

// Conversions
long actuator_to_motor_position(float act) {
  return long(spm.actuator_direction*MICROSTEP*ROT_SCALE*MOTOR_STEPS*act/(2*PI));
}

float actuator_to_motor_speed(float act) {
  return spm.actuator_direction*MICROSTEP*ROT_SCALE*MOTOR_STEPS*act/(2*PI);
}

float motor_to_actuator_position(long mot) {
  return 2*PI*float(mot)/(spm.actuator_direction*MICROSTEP*ROT_SCALE*MOTOR_STEPS);
}

float motor_to_actuator_speed(float mot) {
  return 2*PI*mot/(spm.actuator_direction*MICROSTEP*ROT_SCALE*MOTOR_STEPS);
}

