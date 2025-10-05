#pragma once
#include <ArduinoEigen.h>
#include <cmath>
#include <complex>
#include <vector>

using namespace std;
using namespace Eigen;

Matrix4f gyro_transition_matrix(Vector3f gyro_xyz, float dt);

Vector4f ypr_to_q(Vector3f ypr);
Vector3f q_to_ypr(Vector4f quaternion);

Vector4f unit_q(Vector4f q);

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

private:
    StateVector x; // state
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