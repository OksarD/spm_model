// Functions used in main sketch

void update_state() {
  state = next_state;
}

void initializeMotors() {
  // You could configure the motor drivers here, set microstepping, etc.
    stepper_1.setMaxSpeed(MAX_SPEED);
    stepper_2.setMaxSpeed(MAX_SPEED);
    stepper_3.setMaxSpeed(MAX_SPEED);
    pinMode(SLEEP_1, OUTPUT);
    pinMode(SLEEP_2, OUTPUT);
    pinMode(SLEEP_3, OUTPUT);
    Serial.println("Stepper Motors initialization succeeded!");
}

float ExtractValue(const char* linea, char eje) {
  const char* index = strchr(linea, eje);
  if (index == NULL) {
    return 0;  
  }
  return atof(index + 1);
}

void clearCharArray(char* charArray, size_t size) {
  for (size_t i = 0; i < size; i++) {
    charArray[i] = '\0';  // Fill each element with null character
  }
}

void disable_motors(){
  digitalWrite(SLEEP_1, LOW);
  digitalWrite(SLEEP_2, LOW);
  digitalWrite(SLEEP_3, LOW);
  #ifdef DEBUG
  Serial.println("Motors Disabled!");
  #endif
}

void enable_motors(){
  digitalWrite(SLEEP_1, HIGH);
  digitalWrite(SLEEP_2, HIGH);
  digitalWrite(SLEEP_3, HIGH);
  #ifdef DEBUG
  Serial.println("Motors Enabled!");
  #endif
}

void extract_trajectory_command(int traj[], const char* command){
  // trajectory command in order (Yaw, Pitch, Roll, Yaw-Deriv, Pitch-Deriv, Roll-Deriv)
  const char letters[] = {'Y','P','R','y','p','r'};
  for(uint8_t i=0; i<sizeof(letters); i++) {
    traj[i] = ExtractValue(command, letters[i]);
  }
}

float actuator_to_motor_speed(float input) {
  return MICROSTEP*ROT_SCALE*MOTOR_STEPS*input/(2*PI);
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

// Control functions
void position_control(Vector3f ypr) {
  
}

void open_trajectory_control(Vector3f ypr, Vector3f ypr_velocity) {
  Matrix3f R_mat = spm.R_ypr(ypr);
  Vector3f xyz_platform_velocity = spm.ypr_to_xyz_velocity(ypr_velocity, ypr);
  Vector3f actuator_velocity = spm.solve_ivk(R_mat, xyz_platform_velocity);
  #ifdef DEBUG
  Serial.print(loop_time_elapsed);
  Serial.print(",");
  Serial.print(actuator_velocity[0], 4);
  Serial.print(",");
  Serial.print(actuator_velocity[1], 4);
  Serial.print(",");
  Serial.print(actuator_velocity[2], 4);
  Serial.println();
  #endif
  // convert actuator velocity (rad/s) to stepper velocity (steps/s) and set the motor speed
  stepper_1.setSpeed(actuator_to_motor_speed(actuator_velocity[0]));
  stepper_2.setSpeed(actuator_to_motor_speed(actuator_velocity[1]));
  stepper_3.setSpeed(actuator_to_motor_speed(actuator_velocity[2]));
}


