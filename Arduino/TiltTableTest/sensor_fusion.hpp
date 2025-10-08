#pragma once
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <cmath>
#include <complex>
#include <vector>
#include <algorithm>

using namespace std;
using namespace Eigen;

// Kalman filter class and related functions

template <typename T>
inline T clamp(T value, T low, T high) {
    if (value < low)  return low;
    if (value > high) return high;
    return value;
}

Matrix4f gyro_transition_matrix(Vector3f gyro_xyz, float dt);

Quaternionf ypr_to_q(Vector3f ypr);
Vector3f q_to_ypr(Quaternionf q);

template <typename T, int N, int M>
class KalmanFilter {
public:
    using StateVector = Matrix<T, N, 1>;
    using StateMatrix = Matrix<T, N, N>;
    using MeasurementVector = Matrix<T, M, 1>;
    using MeasurementMatrix = Matrix<T, M, N>;
    using MeasurementCov = Matrix<T, M, M>;
    using GainMatrix = Matrix<T, N, M>;
    StateMatrix F; // state transition matrix
    MeasurementMatrix H; // state to measurement transformation
    StateVector x; // state

private:
    StateMatrix P; // state covaraince
    StateMatrix Q; // state transition coveriance
    MeasurementCov R; // meaurement covariance

public:
    KalmanFilter(const StateVector& x0,
                 const StateMatrix& P0,
                 const StateMatrix& F_,
                 const MeasurementMatrix& H_,
                 const StateMatrix& Q_,
                 const MeasurementCov& R_);

    void predict();

    void correct(const MeasurementVector& z);

    StateVector& state();
};

template <typename T, int N, int M>
KalmanFilter<T, N, M>::KalmanFilter(const StateVector& x0,
                 const StateMatrix& P0,
                 const StateMatrix& F_,
                 const MeasurementMatrix& H_,
                 const StateMatrix& Q_,
                 const MeasurementCov& R_)
        : x(x0), P(P0), F(F_), Q(Q_), H(H_), R(R_) {}

template <typename T, int N, int M>
void KalmanFilter<T, N, M>::predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
}

template <typename T, int N, int M>
void KalmanFilter<T, N, M>::correct(const MeasurementVector& z) {
    MeasurementVector y = z - H * x;                // innovation
    MeasurementCov S = H * P * H.transpose() + R;   // innovation covariance
    GainMatrix K = P * H.transpose() * S.inverse(); // Kalman gain
    x = x + K * y;
    P = (StateMatrix::Identity() - K * H) * P;
}

template <typename T, int N, int M>
Matrix<T, N, 1>& KalmanFilter<T, N, M>::state() { 
    return x; 
}

class LookupTable2D {
public:
    LookupTable2D(float x0, float y0, float dx, float dy,
                  size_t nx, size_t ny, const float* data)
        : x0_(x0), y0_(y0), dx_(dx), dy_(dy),
          nx_(nx), ny_(ny), data_(data) {}

    float interp(float x, float y) const {
        // Compute grid coordinates
        float gx = (x - x0_) / dx_;
        float gy = (y - y0_) / dy_;

        // Find lower-left cell corner
        int i = static_cast<int>(floor(gx));
        int j = static_cast<int>(floor(gy));

        // Clamp to valid range
        if (i < 0) i = 0;
        if (j < 0) j = 0;
        if (i >= static_cast<int>(nx_) - 1) i = nx_ - 2;
        if (j >= static_cast<int>(ny_) - 1) j = ny_ - 2;

        // Local fractional position
        float tx = gx - i;
        float ty = gy - j;

        // Sample the 4 corner values
        float f00 = at(i, j);
        float f10 = at(i + 1, j);
        float f01 = at(i, j + 1);
        float f11 = at(i + 1, j + 1);

        // Bilinear interpolation
        return f00 * (1 - tx) * (1 - ty)
             + f10 * tx       * (1 - ty)
             + f01 * (1 - tx) * ty
             + f11 * tx       * ty;
    }

private:
    inline float at(size_t i, size_t j) const {
        return data_[j * nx_ + i];  // Row-major
    }

    const float* data_;
    float x0_, y0_;
    float dx_, dy_;
    size_t nx_, ny_;
};
