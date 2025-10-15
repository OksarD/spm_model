#include "helpers.hpp"

void initializeMotors() {
  // You could configure the motor drivers here, set microstepping, etc.
    // stepper_0.setMaxSpeed(MAX_SPEED);
    // stepper_1.setMaxSpeed(MAX_SPEED);
    // stepper_2.setMaxSpeed(MAX_SPEED);
    driver.begin();
    driver.add_motor(stepper_0);
    driver.add_motor(stepper_1);
    driver.add_motor(stepper_2);
    Serial.println("Stepper Motors initialised.");
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
  stepper_0.disable();
  stepper_1.disable();
  stepper_2.disable();
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
  stepper_0.enable();
  stepper_1.enable();
  stepper_2.enable();
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

void set_actuator_velocity(Vector3f& actuator_velocity) {
  stepper_0.setSpeed(actuator_to_motor_speed(actuator_velocity[0]));
  stepper_1.setSpeed(actuator_to_motor_speed(actuator_velocity[1]));
  stepper_2.setSpeed(actuator_to_motor_speed(actuator_velocity[2]));
}

void reset_actuator_position() {
  stepper_0.position = actuator_to_motor_position(spm.actuator_origin);
  stepper_1.position = actuator_to_motor_position(spm.actuator_origin);
  stepper_2.position = actuator_to_motor_position(spm.actuator_origin);
  #ifdef INFO
  Serial.print("Actuators reset to origin position: ");
  Serial.println(spm.actuator_origin, 3);
  #endif
}

Vector3f actuator_position() {
  Vector3f pos(motor_to_actuator_position(stepper_0.position),
          motor_to_actuator_position(stepper_1.position),
          motor_to_actuator_position(stepper_2.position)
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
    accel[1] += platformIMU.readFloatAccelY();
    accel[2] += platformIMU.readFloatAccelZ();
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
    gyro[1] += radians(platformIMU.readFloatGyroY());
    gyro[2] += radians(platformIMU.readFloatGyroZ());
  }
  if(samples > 1) {
    gyro /= samples;
  }
  return gyro;
}

Quaternionf estimate(bool include_yaw_fpk) {
  Vector3f ypr_meas = accel_ypr(1);
  if (include_yaw_fpk) { 
    ypr_meas[0] = interp_yaw_fpk();
  }
  Quaternionf q_meas = ypr_to_q(ypr_meas);
  Vector3f gyro = gyro_xyz() - gyro_bias;
  kalman.F = xyz_velocity_transition_matrix(gyro, LOOP_TIMING_INTERVAL/1e6);
  kalman.predict();
  #ifdef TRACE
  Serial.print("0, ");
  Serial.print(gyro[0]);
  Serial.print(", ");
  Serial.print(gyro[1]);
  Serial.print(", ");
  Serial.print(gyro[2]);
  Serial.print(", 1, ");
  Serial.print(ypr_meas[0]);
  Serial.print(", ");
  Serial.print(ypr_meas[1]);
  Serial.print(", ");
  Serial.print(ypr_meas[2]);
  Serial.print(", 2, ");
  Serial.print(q_meas.w(), 3);
  Serial.print(", ");
  Serial.print(q_meas.x(), 3);
  Serial.print(", ");
  Serial.print(q_meas.y(), 3);
  Serial.print(", ");
  Serial.print(q_meas.z(), 3);
  Serial.print(", 3, ");
  Serial.print(kalman.state()[0], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[1], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[2], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[3], 3);
  #endif
  kalman.correct(Vector4f(q_meas.w(),q_meas.x(), q_meas.y(), q_meas.z()));
  kalman.x /= kalman.x.norm();
  Quaternionf est = Quaternionf(kalman.x[0], kalman.x[1], kalman.x[2], kalman.x[3]);
  #ifdef TRACE
  Serial.print(", 4, ");
  Serial.print(kalman.state()[0], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[1], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[2], 3);
  Serial.print(", ");
  Serial.print(kalman.state()[3], 3);
  Serial.println();
  #endif
  #ifdef INFO
  // evaluate kalman filter with these prints
  // second kalman filter to only predict to evaluate accumulated error
  kalman_predict.F = xyz_velocity_transition_matrix(gyro, LOOP_TIMING_INTERVAL/1e6);
  kalman_predict.predict();
  Vector3f pred = q_to_ypr(Quaternionf(kalman_predict.x[0], kalman_predict.x[1], kalman_predict.x[2], kalman_predict.x[3])); // predict step for debugging
  Vector3f est_ypr = q_to_ypr(est);
  Serial.print(loop_time_elapsed);
  Serial.print(",");
  Serial.print(est_ypr[0], 4);
  Serial.print(",");
  Serial.print(est_ypr[1], 4);
  Serial.print(",");
  Serial.print(est_ypr[2], 4);
  Serial.print(",");
  Serial.print(ypr_meas[0], 4);
  Serial.print(",");
  Serial.print(ypr_meas[1], 4);
  Serial.print(",");
  Serial.print(ypr_meas[2], 4);
  Serial.print(",");
  Serial.print(pred[0], 4);
  Serial.print(",");
  Serial.print(pred[1], 4);
  Serial.print(",");
  Serial.print(pred[2], 4);
  Serial.println();
  #endif
  return est;
}

float interp_yaw_fpk() {
  Vector3f act_pos = actuator_position();
  float offset = act_pos[0] - spm.actuator_origin;
  Vector3f actuator_offset = act_pos - Vector3f::Constant(offset); // offset the actuator position such that m0 is placed at the origin
  float yaw_offset = fpk_yaw_table.interp(actuator_offset[1], actuator_offset[2]); // fpk table is in the m1/m2 domain
  if (yaw_offset < -2*PI || yaw_offset > 2*PI) { // if yaw value drastically out-of-bounds (NAN)
    disable_motors();
    next_state = IDLE_STATE;
    Serial.println("Platform Out of Bounds! Ensure that position commands have less than a 40 degree slope.");
    return FPK_NAN_CODE;
  }
  #ifdef TRACE
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
void position_control(Quaternionf& error, Quaternionf& meas, PID& comp) {
  Vector4f error_aa = q_to_aa(error);
  if (error_aa[0] < 0) {
    error_aa *= -1;  // ensure positive angular errors only
  }
  float compensated_error = comp.update(error_aa[0], LOOP_TIMING_INTERVAL/1e6); // compensate for the angle and not direction
  error_aa[0] = compensated_error;
  Vector3f control = aa_to_xyz(error_aa);
  Matrix3f R = aa_to_R(q_to_aa(meas));
  Vector3f actuator_velocity = spm.solve_ivk(R, control);
  set_actuator_velocity(actuator_velocity);
  #ifdef TRACE
  //Serial.print("Error: ");
  Serial.print(error.w(), 3);
  Serial.print(", ");
  Serial.print(error.x(), 3);
  Serial.print(", ");
  Serial.print(error.y(), 3);
  Serial.print(", ");
  Serial.print(error.z(), 3);
  //Serial.print(", ");
  //Serial.print(" Control: ");
  // Serial.print(control[0], 3);
  // Serial.print(", ");
  // Serial.print(control[1], 3);
  // Serial.print(", ");
  // Serial.print(control[2], 3);
  // Serial.print(", ");
  //Serial.print(" Act Vel: ");
  // Serial.print(actuator_velocity[0], 3);
  // Serial.print(",");
  // Serial.print(actuator_velocity[1], 3);
  // Serial.print(",");
  // Serial.print(actuator_velocity[2], 3);
  // Serial.println();
  #endif
}

void open_trajectory_control(Vector3f& ypr_ref, Vector3f& ypr_velocity_ref) {
  Matrix3f R_mat = spm.R_ypr(ypr_ref);
  Vector3f xyz_platform_velocity = spm.ypr_to_xyz_velocity(ypr_velocity_ref, ypr_ref);
  Vector3f actuator_velocity = spm.solve_ivk(R_mat, xyz_platform_velocity);
  set_actuator_velocity(actuator_velocity);
  #ifdef INFO
  estimate();
  #endif
}

void test_motor(StepperMotor& m, uint8_t ind) {
  long start_pos = m.position;
  #ifdef INFO
  Serial.print("Stepper ");
  Serial.print(ind);
  Serial.print(" Start pos: ");
  Serial.print(start_pos);
  #endif
  unsigned long start_time = millis();
  m.setSpeed(actuator_to_motor_speed(radians(30))); // move +30 degrees/s for 1 second around control axis (-Z)
  while (start_time + 1e3 > millis()) {
    m.debug_print();
    Serial.println();
  }
  m.setSpeed(0);
  long end_pos = m.position;
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

// Euler angles (yaw-pitch-roll) to quaternion
Quaternionf ypr_to_q(const Vector3f& ypr) {
    static Quaternionf q_prev = Quaternionf::Identity();
    float sin_y_2 = sin(ypr[0]*0.5);
    float cos_y_2 = cos(ypr[0]*0.5);
    float sin_p_2 = sin(ypr[1]*0.5);
    float cos_p_2 = cos(ypr[1]*0.5);
    float sin_r_2 = sin(ypr[2]*0.5);
    float cos_r_2 = cos(ypr[2]*0.5); 
    Quaternionf q;
    q.w() = cos_r_2 * cos_p_2 * cos_y_2 + sin_r_2 * sin_p_2 * sin_y_2;
    q.x() = sin_r_2 * cos_p_2 * cos_y_2 - cos_r_2 * sin_p_2 * sin_y_2;
    q.y() = cos_r_2 * sin_p_2 * cos_y_2 + sin_r_2 * cos_p_2 * sin_y_2;
    q.z() = cos_r_2 * cos_p_2 * sin_y_2 - sin_r_2 * sin_p_2 * cos_y_2;
    if (q.dot(q_prev) < 0.0f) { // account for double-covering to maintain sign consistency
      q.coeffs() *= -1.0f;
    }
    q_prev = q;
    return q.normalized();
}

// Quaternion to euler angles (yaw-pitch-roll)
Vector3f q_to_ypr(const Quaternionf& q) {
    Quaternionf qn = q.normalized();
    float w = qn.w(), x = qn.x(), y = qn.y(), z = qn.z();
    float yaw = atan2(2.0f * (w*z + x*y),
                           1.0f - 2.0f * (y*y + z*z));
    float sinp = 2.0f * (w*y - z*x);
    sinp = clamp(sinp, -1.0f, 1.0f);
    float pitch = asin(sinp);
    float roll = atan2(2.0f * (w*x + y*z),
                            1.0f - 2.0f * (x*x + y*y));
    return Vector3f(yaw, pitch, roll);
}

// Quaternion to axis-angle [a, x, y, z]
Vector4f q_to_aa(const Quaternionf& q_in) {
    Quaternionf q = q_in.normalized();
    // Ensure numerical stability
    float angle = 2.0f * acos(clamp(q.w(), -1.0f, 1.0f));
    float s = sqrt(1.0f - q.w() * q.w());
    Vector3f axis = q.vec() / s;  // q.x, q.y, q.z normalized
    Vector4f aa;
    aa << angle, axis.x(), axis.y(), axis.z();
    return aa;
}

// Axis-Angle [a, x, y, z] to Quaternion
Quaternionf aa_to_q(const Vector4f& aa) {
    float angle = aa(0);
    Vector3f axis = aa.tail<3>();
    float half_angle = 0.5f * angle;
    float s = sin(half_angle);
    float c = cos(half_angle);
    if (axis.norm() < 1e-6f)
        axis = Vector3f(1.0f, 0.0f, 0.0f);
    else
        axis.normalize();
    return Quaternionf(c, axis.x() * s, axis.y() * s, axis.z() * s);
}

// Axis-angle [a, x, y, z] to rotation matrix
Matrix3f aa_to_R(const Vector4f& aa)
{
    float angle = aa(0);
    Vector3f axis = aa.tail<3>();
    // Handle degenerate/small-angle case
    if (axis.norm() < 1e-6f)
        return Matrix3f::Identity();
    axis.normalize();
    float c = cos(angle);
    float s = sin(angle);
    float one_c = 1.0f - c;
    // Rodrigues’ rotation formula
    Matrix3f R;
    R << c + axis.x() * axis.x() * one_c,
         axis.x() * axis.y() * one_c - axis.z() * s,
         axis.x() * axis.z() * one_c + axis.y() * s,

         axis.y() * axis.x() * one_c + axis.z() * s,
         c + axis.y() * axis.y() * one_c,
         axis.y() * axis.z() * one_c - axis.x() * s,

         axis.z() * axis.x() * one_c - axis.y() * s,
         axis.z() * axis.y() * one_c + axis.x() * s,
         c + axis.z() * axis.z() * one_c;
    return R;
}

// Axis angle [a, x, y, z] to rotation vector
Vector3f aa_to_xyz(const Vector4f& aa) {
  return Vector3f(aa[0]*aa[1], aa[0]*aa[2], aa[0]*aa[3]);
}
