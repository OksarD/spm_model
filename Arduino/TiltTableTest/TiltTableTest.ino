#include <Arduino_LSM9DS1.h>
#include <math.h>
#include <AccelStepper.h>
#include "CoaxialSPM.hpp"

//#define DEBUG

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
#define ROT_SCALE 3.08333
#define RAD_TO_DEG 57.29577951308232  // Constant to convert radians to degrees

// Coaxial SPM object
float a1 = PI / 4;   // 45 deg
float a2 = PI / 2;   // 90 deg
float b  = PI / 2;   // 90 deg
Coaxial_SPM spm(a1, a2, b);

AccelStepper stepper_1(AccelStepper::DRIVER, STEP_1, DIR_1);
AccelStepper stepper_2(AccelStepper::DRIVER, STEP_2, DIR_2);
AccelStepper stepper_3(AccelStepper::DRIVER, STEP_3, DIR_3);

long stepper_1_pos = 0;
long stepper_2_pos = 0;
long stepper_3_pos = 0;
long roll_angle = 0;
long pitch_angle = 0;
long yaw_angle = 0;
int r_input;
int p_input;
int y_input;
int dr_input;
int dp_input;
int dy_input;
long test_start_time;
bool test_running = false;
int test_duration = 5000;

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
  Serial.println("Disabling Motors...");
  digitalWrite(SLEEP_1, LOW);
  digitalWrite(SLEEP_2, LOW);
  digitalWrite(SLEEP_3, LOW);
  delay(1000);
  Serial.println("Motors Disabled!");
}

void enable_motors(){
  Serial.println("Enabling Motors...");
  digitalWrite(SLEEP_1, HIGH);
  digitalWrite(SLEEP_2, HIGH);
  digitalWrite(SLEEP_3, HIGH);
  delay(1000);
  Serial.println("Motors Enabled!");
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

void loop() {
  // float angle = 0.00;
  if(Serial.available()) {
    char command[100]; // Assuming a maximum command length of 700 characters
    clearCharArray(command,sizeof(command));
    Serial.readBytesUntil('\n', command, sizeof(command));
    #ifdef DEBUG
    Serial.println(command);
    #endif
    if (command[0] == 'E' || command[0] == 'e'){
      enable_motors();
    }
    if (command[0] == 'D' || command[0] == 'd'){
      disable_motors();
    }
    if (command[0] == 'G' || command[0] == 'g'){
      print_current_position();
    }
    if (command[0] == 'M' || command[0] == 'm') {
      input_angles(command); //get trajectory input in milliradians(/s)
      #ifdef DEBUG
      print_input_angles();
      #endif
      // compute open-loop actuator velocity from reference trajectory
      Vector3f ypr(y_input*0.001, p_input*0.001, r_input*0.001); // ypr reference in radians
      Matrix3f R_mat = spm.R_ypr(ypr);
      Vector3f ypr_platform_velocity(dy_input*0.001, dp_input*0.001, dr_input*0.001); // ypr velocity reference in rad/s
      Vector3f xyz_platform_velocity = spm.ypr_to_xyz_velocity(ypr_platform_velocity, ypr);
      Vector3f input_velocity = spm.solve_ivk(R_mat, xyz_platform_velocity);
      #ifdef DEBUG
      Serial.println("Actuator Velocity");
      Serial.println(input_velocity[0]);
      Serial.println(input_velocity[1]);
      Serial.println(input_velocity[2]);
      #endif
      // convert actuator velocity (rad/s) to stepper velocity (steps/s)
      float m1_speed = actuator_to_motor_speed(input_velocity[0]);
      float m2_speed = actuator_to_motor_speed(input_velocity[1]);
      float m3_speed = actuator_to_motor_speed(input_velocity[2]);

      stepper_1.setSpeed(m1_speed);
      stepper_2.setSpeed(m2_speed);
      stepper_3.setSpeed(m3_speed);
    }
    if ((command[0] == 'T' || command[0] == 't') && !test_running) {
        Serial.println("Testing continous velocity control...");
        test_start_time = millis();
        test_running = true;
    }
  }
  if(test_running) {
    int t = millis() - test_start_time;
    // generate wave
    float actuator_speed = 2*sin(t*0.001);
    stepper_1.setSpeed(actuator_to_motor_speed(actuator_speed));
    stepper_2.setSpeed(actuator_to_motor_speed(actuator_speed));
    stepper_3.setSpeed(actuator_to_motor_speed(actuator_speed));
    if (t > test_start_time + test_duration) { // execute at end of test
      Serial.println("Test finished");
      stepper_1.setSpeed(0);
      stepper_2.setSpeed(0);
      stepper_3.setSpeed(0);
      test_running = false;
    }
  } 
  stepper_1.runSpeed(); // Polling for stepper motors
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
