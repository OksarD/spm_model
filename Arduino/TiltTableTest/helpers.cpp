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

float actuator_to_motor(float act) {
  return spm.actuator_direction*MICROSTEP*ROT_SCALE*MOTOR_STEPS*act/(2*PI);
}

float motor_to_actuator(float mot) {
  return 2*PI*mot/(spm.actuator_direction*MICROSTEP*ROT_SCALE*MOTOR_STEPS);
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
  stepper_0.setSpeed(actuator_to_motor(actuator_velocity[0]));
  stepper_1.setSpeed(actuator_to_motor(actuator_velocity[1]));
  stepper_2.setSpeed(actuator_to_motor(actuator_velocity[2]));
}

void reset_actuator_position() {
  stepper_0.setCurrentPosition(actuator_to_motor(spm.actuator_origin));
  stepper_1.setCurrentPosition(actuator_to_motor(spm.actuator_origin));
  stepper_2.setCurrentPosition(actuator_to_motor(spm.actuator_origin));
}

Vector3f actuator_position() {
  Vector3f pos(motor_to_actuator(stepper_0.currentPosition()),
          motor_to_actuator(stepper_1.currentPosition()),
          motor_to_actuator(stepper_2.currentPosition())
  );
  return pos;
}

void enable_loop_timing() {
  loop_timing_enabled = true;
  loop_start_time = micros();
  loop_time_proc = 0;
  #ifdef INFO
  Serial.println("Loop timing enabled.");
  #endif
}

void disable_loop_timing() {
  if(loop_timing_enabled) {
    loop_timing_enabled = false;
    #ifdef INFO
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
      #ifdef DEBUG
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

Vector3f ypr_estimate() {
  Vector3f ypr_accel = accel_ypr();
  Vector3f gyro = gyro_xyz() - gyro_bias;
  kalman.F = gyro_transition_matrix(gyro, LOOP_TIMING_INTERVAL/1e6);
  kalman.predict();
  kalman.correct(ypr_to_q(ypr_accel));
  return q_to_ypr(kalman.state());
}

// Control functions
void position_control(Vector3f ypr_ref, Vector3f ypr_meas) {
  Vector3f error = ypr_ref - ypr_meas;
  Vector3f control_signal = 0.5 * error; // proportional controller
  Matrix3f R_mat = spm.R_ypr(ypr_meas);
  Vector3f xyz_platform_velocity = spm.ypr_to_xyz_velocity(control_signal, ypr_meas);
  Vector3f actuator_velocity = spm.solve_ivk(R_mat, xyz_platform_velocity);
  set_actuator_velocity(actuator_velocity);
  #ifdef INFO
  Serial.print("Error: ");
  Serial.print(error[0], 3);
  Serial.print(", ");
  Serial.print(error[1], 3);
  Serial.print(", ");
  Serial.print(error[2], 3);
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
  Serial.print(" M_speed: ");
  Serial.print(stepper_0.speed());
  Serial.print(",");
  Serial.print(stepper_1.speed());
  Serial.print(",");
  Serial.print(stepper_2.speed());
  Serial.println();
  #endif
}

void open_trajectory_control(Vector3f ypr_ref, Vector3f ypr_velocity_ref) {
  Matrix3f R_mat = spm.R_ypr(ypr_ref);
  Vector3f xyz_platform_velocity = spm.ypr_to_xyz_velocity(ypr_velocity_ref, ypr_ref);
  Vector3f actuator_velocity = spm.solve_ivk(R_mat, xyz_platform_velocity);
  set_actuator_velocity(actuator_velocity);
  #ifdef INFO
  Serial.print(actuator_velocity[0], 3);
  Serial.print(",");
  Serial.print(actuator_velocity[1], 3);
  Serial.print(",");
  Serial.print(actuator_velocity[2], 3);
  Serial.print(" M_speed: ");
  Serial.print(stepper_0.speed());
  Serial.print(",");
  Serial.print(stepper_1.speed());
  Serial.print(",");
  Serial.print(stepper_2.speed());
  Serial.println();
  #endif
}

void test_motor(AccelStepper m) {
  unsigned long time_start = millis();
  m.setSpeed(actuator_to_motor(radians(30))); // move +30 degrees around control axis (-Z)
  while(millis() - time_start < 1e3) {
    m.runSpeed();
    yield();
  }
  m.setSpeed(0);
}

