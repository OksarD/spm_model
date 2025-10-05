#pragma once
#include <Arduino.h>
#include "project.hpp"
#include <ArduinoEigen.h>
#include <cmath>
#include <complex>
#include <vector>

using namespace std;
using namespace Eigen;

// Motor helpers
void initializeMotors();
void disable_motors();
void enable_motors();
float actuator_to_motor_speed(float input);
void test_motor(AccelStepper m);
void halt_motors();
void set_actuator_velocity();
void reset_actuator_position();
Vector3f actuator_position();

// buffer helpers
void buffer_push(unsigned int length, char* items);
char* buffer_pop();
void clearCharArray(char* charArray, size_t size);
float ExtractValue(const char* linea, char eje);
bool hasChar(const char* linea, char eje);
Vector3f extract_position(char* command);
Vector3f extract_velocity(char* command);

// Loop Timing
void enable_loop_timing();
void disable_loop_timing();
bool loop_timing_proc();

// Estimation functions
Vector3f accel_ypr(unsigned int samples = 1);
Vector3f gyro_xyz(unsigned int samples = 1);
Vector3f ypr_estimate();
// Control functions
void position_control(Vector3f ypr_ref, Vector3f ypr_meas);
void open_trajectory_control(Vector3f ypr_ref, Vector3f ypr_velocity_ref);


