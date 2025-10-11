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
void test_motor(StepperMotor& m, uint8_t ind);
void halt_motors();
void set_actuator_velocity(Vector3f& actuator_velocity);
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
Quaternionf estimate(bool include_yaw_fpk = true);
float interp_yaw_fpk();

// Control functions
void position_control(Quaternionf& error, Quaternionf& meas);
void open_trajectory_control(Vector3f& ypr_ref, Vector3f& ypr_velocity_ref);

// Conversions
long actuator_to_motor_position(float act);
float actuator_to_motor_speed(float act);
float motor_to_actuator_position(long mot);
float motor_to_actuator_speed(float mot);

// Angle conversions
template <typename T>
inline T clamp(T value, T low, T high) {
    if (value < low)  return low;
    if (value > high) return high;
    return value;
}

Quaternionf ypr_to_q(const Vector3f& ypr);
Vector3f q_to_ypr(const Quaternionf& q);
Vector4f q_to_aa(const Quaternionf& q_in);
Quaternionf aa_to_q(const Vector4f& aa);
Matrix3f aa_to_R(const Vector4f& aa);
Vector3f aa_to_xyz(const Vector4f& aa);
