#pragma once
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <deque>

//#define DEBUG

template <typename T>
inline T clamp(T value, T low, T high) {
    if (value < low)  return low;
    if (value > high) return high;
    return value;
}

class PID {
public:
    // Construct with gains. output limits and integrator limits are optional.
    PID(float Kp = 1.0, float Ki = 0.0, float Kd = 0.0,
        float out_min = -1e9, float out_max = 1e9,
        float int_min = -1e9, float int_max = 1e9, 
        uint8_t deriv_average = 1)
      : Kp_(Kp), Ki_(Ki), Kd_(Kd),
        out_min_(out_min), out_max_(out_max),
        int_min_(int_min), int_max_(int_max),
        deriv_average_(deriv_average)
    {
        for (uint8_t i=0;i<deriv_average_;i++) {
            prev_derivs.push_back(0); // fill deriv buffer with zeroes
        }
    }

    // Reset internal state
    void reset() {
        prev_error_ = 0.0;
        //prev_measurement_ = 0.0;
        integrator_ = 0.0;
        last_output_ = 0.0;
        initialized_ = false;
    }

    // Set tunings at runtime
    void setTunings(float Kp, float Ki, float Kd) {
        Kp_ = Kp; Ki_ = Ki; Kd_ = Kd;
    }

    void setOutputLimits(float min, float max) {
        out_min_ = min; out_max_ = max;
        if (out_min_ > out_max_) std::swap(out_min_, out_max_);
    }

    void setIntegratorLimits(float min, float max) {
        int_min_ = min; int_max_ = max;
        if (int_min_ > int_max_) std::swap(int_min_, int_max_);
    }

    // Main update: provide setpoint, measurement, and dt (seconds).
    // Returns controller output (clamped to output limits).
    float update(float error, float dt) {
        if (dt <= 0.0) {
            // invalid dt -> return last output unchanged
            return last_output_;
        }

        //const float error = setpoint - measurement;

        // initialize prev_measurement on first call to avoid large derivative spike
        if (!initialized_) {
            //prev_measurement_ = measurement;
            prev_error_ = error;
            initialized_ = true;
        }

        const float derivative = (error - prev_error_) / dt;

        // Low-pass fiter (average) derivative to prevent jittery behaviour
        prev_derivs.push_back(derivative);
        prev_derivs.pop_front();
        float average_derivative = 0;
        for (uint8_t i=0;i<prev_derivs.size();i++) {
            average_derivative += prev_derivs[i];
        }
        average_derivative /= prev_derivs.size();

        // Tentatively compute integrator increment
        const float integrator_increment = Ki_ * error * dt;

        // Compute tentative integrator using simple clamp/backcalc rules
        float tentative_integrator = integrator_ + integrator_increment;

        // If back-calculation is enabled, we'll compute unsaturated output and then correct integrator
        float unsat_output = Kp_ * error + tentative_integrator + Kd_ * average_derivative;

        // Saturate output to limits
        float sat_output = clamp(unsat_output, out_min_, out_max_);

        // conditional integration (simple integrator clamping):
        // If output is saturated and error would make it worse, do NOT apply the integrator increment.
        bool saturating_high = (unsat_output > out_max_) && (error > 0.0);
        bool saturating_low  = (unsat_output < out_min_) && (error < 0.0);

        if (saturating_high || saturating_low) {
            // do not integrate (prevent further windup)
            tentative_integrator = integrator_;
        } else {
            // allow integrator with clamping
            tentative_integrator = clamp(tentative_integrator, int_min_, int_max_);
        }

        // Final output (with new integrator)
        float output = Kp_ * error + tentative_integrator + Kd_ * average_derivative;
        output = clamp(output, out_min_, out_max_);
        // Save state
        integrator_ = tentative_integrator;
        prev_error_ = error;
        //prev_measurement_ = measurement;
        last_output_ = output;
        last_derivative_ = average_derivative;
        #ifdef DEBUG
        Serial.print("E: ");
        Serial.print(error, 3);
        Serial.print("\tP: ");
        Serial.print(Kp_ * error, 3);
        Serial.print("\tI: ");
        Serial.print(tentative_integrator, 3);
        Serial.print("\tD: ");
        Serial.print(Kd_ * average_derivative, 3);
        Serial.print("\tderiv: ");
        Serial.print(derivative, 3);
        Serial.print("\tOut: ");
        Serial.print(output, 3);
        Serial.println();
        #endif
        return output;
    }

    // Accessors
    float lastOutput() const { return last_output_; }
    float integrator() const { return integrator_; }
    float lastDerivative() const { return last_derivative_; }

private:
    float Kp_, Ki_, Kd_;
    float out_min_, out_max_;
    float int_min_, int_max_;
    float integrator_ = 0.0;
    float prev_error_ = 0.0;
    //float prev_measurement_ = 0.0;
    float last_output_ = 0.0;
    bool initialized_ = false;
    float last_derivative_ = 0;
    deque<float> prev_derivs;
    float deriv_average_;
};