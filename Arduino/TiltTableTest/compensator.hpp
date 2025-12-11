#pragma once
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

template <typename T>
inline T clamp(T value, T low, T high) {
    if (value < low)  return low;
    if (value > high) return high;
    return value;
}

inline bool isFinite(float x) {
    return !(isnan(x) || isinf(x));
}

class PID {
public:
    PID(float Kp = 1.0, float Ki = 0.0, float Kd = 0.0,
        float out_min = -1e9, float out_max = 1e9,
        float int_min = -1e9, float int_max = 1e9, 
        uint8_t deriv_average = 1)
      : Kp_(Kp), Ki_(Ki), Kd_(Kd),
        out_min_(out_min), out_max_(out_max),
        int_min_(int_min), int_max_(int_max),
        deriv_average_(std::max<uint8_t>(1, deriv_average))
    {
        prev_derivs.assign(deriv_average_, 0.0f);
    }

    void reset() {
        prev_error_ = 0.0;
        integrator_ = 0.0;
        last_output_ = 0.0;
        initialized_ = false;
    }

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

    float update(float error, float dt) {
        // --- Sanity checks ---
        if (!isFinite(error) || !isFinite(dt) || dt <= 1e-9f) {
            return last_output_; // reject bad or zero dt
        }

        if (!initialized_) {
            prev_error_ = error;
            initialized_ = true;
        }

        // --- Derivative computation with guard ---
        float derivative = (error - prev_error_) / dt;
        derivative = clamp(derivative, -1e6f, 1e6f); // limit large spikes

        // --- Low-pass average of derivative ---
        prev_derivs.push_back(derivative);
        if (prev_derivs.size() > deriv_average_) prev_derivs.pop_front();

        float avg_deriv = 0.0f;
        for (float d : prev_derivs) avg_deriv += d;
        avg_deriv /= prev_derivs.size();

        // --- Integrator ---
        float integrator_increment = Ki_ * error * dt;
        if (!isFinite(integrator_increment)) integrator_increment = 0.0f;

        float tentative_integrator = integrator_ + integrator_increment;

        // --- Provisional output ---
        float unsat_output = Kp_ * error + tentative_integrator + Kd_ * avg_deriv;
        if (!isFinite(unsat_output)) unsat_output = last_output_;

        // --- Saturation checks ---
        float sat_output = clamp(unsat_output, out_min_, out_max_);
        bool saturating_high = (unsat_output > out_max_) && (error > 0.0);
        bool saturating_low  = (unsat_output < out_min_) && (error < 0.0);

        // --- Anti-windup ---
        if (saturating_high || saturating_low) {
            tentative_integrator = integrator_; // freeze integrator
        } else {
            tentative_integrator = clamp(tentative_integrator, int_min_, int_max_);
        }

        // --- Final output ---
        float output = Kp_ * error + tentative_integrator + Kd_ * avg_deriv;
        output = clamp(output, out_min_, out_max_);

        // Sanitize final output
        if (!isFinite(output)) output = 0.0f;

        // --- Save state ---
        integrator_ = tentative_integrator;
        prev_error_ = error;
        last_output_ = output;
        last_derivative_ = avg_deriv;

        return output;
    }

    float lastOutput() const { return last_output_; }
    float integrator() const { return integrator_; }
    float lastDerivative() const { return last_derivative_; }

private:
    float Kp_, Ki_, Kd_;
    float out_min_, out_max_;
    float int_min_, int_max_;
    float integrator_ = 0.0;
    float prev_error_ = 0.0;
    float last_output_ = 0.0;
    bool initialized_ = false;
    float last_derivative_ = 0;
    std::deque<float> prev_derivs;
    uint8_t deriv_average_;
};
