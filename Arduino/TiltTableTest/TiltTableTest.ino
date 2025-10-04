#include <Arduino_LSM9DS1.h>
#include <math.h>
#include <AccelStepper.h>
#include "CoaxialSPM.hpp"
#include <queue>
#include <vector>
//--- Adjust Debugging levels Here (#define TRACE/DEBUG/INFO/WARNING)

#define INFO

#ifdef DEBUG
  #define INFO
#endif

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

#define COMMAND_BUFFER_FULL_SIZE 512 // approx 16 commands at roughly 32 bytes per command
#define COMMAND_BUFFER_EMPTY_SIZE 256 // approx 16 commands at roughly 32 bytes per command
#define COMMAND_MAX_SIZE 128 // usually around 32 bytes
#define XON 0x11
#define XOFF 0x13
#define LOOP_TIMING_INTERVAL 20000 // microseconds 
#define DELIM '\n'

// Coaxial SPM object
float a1 = radians(50);
float a2 = radians(75);
float b  = radians(100);
Coaxial_SPM spm(a1, a2, b);

AccelStepper stepper_1(AccelStepper::DRIVER, STEP_1, DIR_1);
AccelStepper stepper_2(AccelStepper::DRIVER, STEP_2, DIR_2);
AccelStepper stepper_3(AccelStepper::DRIVER, STEP_3, DIR_3);

long roll_angle = 0;
long pitch_angle = 0;
long yaw_angle = 0;
int r_input;
int p_input;
int y_input;
int dr_input;
int dp_input;
int dy_input;

std::queue<char> command_buffer;
bool flow_control_halt = false;
bool loop_timing_enabled = false;
unsigned long loop_start_time;
unsigned long loop_time_elapsed = 0;
unsigned long loop_time_proc;

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

long input_angles(const char* command){
  r_input = ExtractValue(command, 'R');
  p_input = ExtractValue(command, 'P');
  y_input = ExtractValue(command, 'Y');
  dr_input = ExtractValue(command, 'r');
  dp_input = ExtractValue(command, 'p');
  dy_input = ExtractValue(command, 'y');
}

void print_input_angles() {
  Serial.print("R input: ");
  Serial.println(r_input);
  Serial.print("P input: ");
  Serial.println(r_input);
  Serial.print("Y input: ");
  Serial.println(r_input);
  Serial.print("dR input: ");
  Serial.println(r_input);
  Serial.print("dP input: ");
  Serial.println(r_input);
  Serial.print("dY input: ");
  Serial.println(r_input);
}

void show_command(){
  Serial.println("----------------------------------------------------");
  Serial.println("Press D to disable the motors.");
  Serial.println("Press E to able the motors.");
  Serial.println("Enter the command in the following format to tilt the table in the Roll, Pitch and Yaw directions: MR0P0Y0");
  Serial.println("where the number after R is the roll value, the number after P is the pitch value and the number after Y is the yaw value.");
}

float actuator_to_motor_speed(float input) {
  return MICROSTEP*ROT_SCALE*MOTOR_STEPS*input/(2*PI);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino is ready...");
  initializeMotors();
  show_command();
  Serial.println("\nPlease enter a command: ");
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

bool loop_timing() {
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
    #ifdef DEBUG
    Serial.println("XOFF sent");
    #endif
  } else if (command_buffer.size() < COMMAND_BUFFER_EMPTY_SIZE && flow_control_halt == true) {
    Serial.write(XON);
    flow_control_halt = false;
    #ifdef DEBUG
    Serial.println("XON sent");
    #endif
  }
  
  // Read commands from buffer, always if loop timing has not started, and only when the timer procs if it is allowed.
  if (!command_buffer.empty() && loop_timing()) {
    #ifdef DEBUG
    Serial.print("Buffer length ");
    Serial.println(command_buffer.size());
    #endif
    char* command = buffer_pop();
    #ifdef DEBUG
    Serial.print("Popped Command ");
    Serial.println(command);
    #endif
    switch(command[0]) {
      case 'M': {
        // Movement command
        input_angles(command); //get trajectory input in milliradians(/s)
        #ifdef DEBUG
        print_input_angles();
        #endif
        // compute open-loop actuator velocity from reference trajectory
        Vector3f ypr(y_input*0.001, p_input*0.001, r_input*0.001); // ypr reference in radians
        Matrix3f R_mat = spm.R_ypr(ypr);
        Vector3f ypr_platform_velocity(dy_input*0.001, dp_input*0.001, dr_input*0.001); // ypr velocity reference in rad/s
        Vector3f xyz_platform_velocity = spm.ypr_to_xyz_velocity(ypr_platform_velocity, ypr);
        Vector3f actuator_velocity = spm.solve_ivk(R_mat, xyz_platform_velocity);
        #ifdef INFO
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
      case 'G': {
        print_current_position();
        break;
      }
      case 'S': {
        loop_timing_enabled = true;
        loop_start_time = micros();
        loop_time_proc = 0;
        #ifdef DEBUG
        Serial.println("Loop timing enabled");
        #endif
        break;
      }
      case 'X': {
        loop_timing_enabled = false;
        #ifdef DEBUG
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
  // Polling for stepper motors
  stepper_1.runSpeed(); 
  stepper_2.runSpeed();
  stepper_3.runSpeed();
  yield();
}

void print_current_position(){
  Serial.print("Current Roll: ");
  Serial.println(roll_angle);
  Serial.print("Current Pitch: ");
  Serial.println(pitch_angle);
  Serial.print("Current Yaw: ");
  Serial.println(yaw_angle);
}
