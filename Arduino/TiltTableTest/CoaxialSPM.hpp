#pragma once
#include <ArduinoEigen.h>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <vector>
#include <float.h>

using namespace Eigen;
using namespace std;

// ================== Utility Functions ================== //

// Quadratic solver
pair<complex<float>, complex<float>> solve_quadratic(float a, float b, float c);

// Normalize vector
Vector3f unit_vector(const Vector3f& v);

// Angle between vectors
float angle_between(const Vector3f& a, const Vector3f& b);

// Wrap radians [-pi, pi)
float wrap_rad(float angle);

// Subtracts angles a1-a2 accounting for discontinuity
float subtract_angles(float a1, float a2);
Vector3f subtract_angles(Vector3f a1, Vector3f a2);

// Rotation matrices
Matrix3f R_x(float a);
Matrix3f R_y(float a);
Matrix3f R_z(float a);
Matrix3f R_axis(const Vector3f& axis, float angle);

bool isclose(float a, float b, float tol=1e-6);

// ================== Coaxial_SPM Class ================== //

class Coaxial_SPM {
public:
    float a1, a2, b;
    std::vector<Vector3f> v, w;
    Vector3f n;
    Vector3f u;
    std::vector<Vector3f> v_origin;
    Vector3f n_origin;
    std::vector<float> eta;
    Matrix3f J;
    std::vector<float> sin_eta;
    std::vector<float> cos_eta;
    float sin_a1, sin_a2, sin_b;
    float cos_a1, cos_a2, cos_b;
    float actuator_origin, actuator_direction;

    // Constructor
    Coaxial_SPM(float a1_, float a2_, float b_);

    // YPR to rotation matrix
    Matrix3f R_ypr(const Vector3f& angle);

    // Limb offset
    float eta_i(int i);

    // Intermediate joint vector
    Vector3f w_i(float in_i, int i);

    // Platform joint vector at home
    Vector3f v_i_origin(int i);

    // Inverse position kinematics terms
    float A_i_ipk(const Vector3f& vi, int i);
    float B_i_ipk(const Vector3f& vi, int i);
    float C_i_ipk(const Vector3f& vi, int i);

    // Inverse position kinematics
    Vector3f solve_ipk(const Matrix3f& r);

    // Verify constraints
    void verify_position();

    // Inverse velocity kinematics
    Vector3f solve_ivk(const Matrix3f& platform_angle, const Vector3f& platform_velocity);

    // YPR velocity to angular velocity
    Vector3f ypr_to_xyz_velocity(const Vector3f& ypr_velocity, const Vector3f& ypr_point);
};
