#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <vector>

using namespace Eigen;
using namespace std;

constexpr double PI = 3.1415926535897932384;

// ================== Utility Functions ================== //

// Quadratic solver
pair<complex<double>, complex<double>> solve_quadratic(double a, double b, double c);

// Normalize vector
Vector3d unit_vector(const Vector3d& v);

// Angle between vectors
double angle_between(const Vector3d& a, const Vector3d& b);

// Wrap radians [-pi, pi)
double wrap_rad(double angle);

// Rotation matrices
Matrix3d R_x(double a);
Matrix3d R_y(double a);
Matrix3d R_z(double a);
Matrix3d R_axis(const Vector3d& axis, double angle);

// ================== Coaxial_SPM Class ================== //

class Coaxial_SPM {
public:
    double a1, a2, b;
    std::vector<Vector3d> v, w;
    Vector3d n;
    Vector3d u;
    std::vector<Vector3d> v_origin;
    Vector3d n_origin;
    std::vector<double> eta;
    Matrix3d J;
    double actuator_origin, actuator_direction;

    // Constructor
    Coaxial_SPM(double a1_, double a2_, double b_);

    // YPR to rotation matrix
    Matrix3d R_ypr(const Vector3d& angle);

    // Limb offset
    double eta_i(int i);

    // Intermediate joint vector
    Vector3d w_i(double in_i, int i);

    // Platform joint vector at home
    Vector3d v_i_origin(int i);

    // Inverse position kinematics terms
    double A_i_ipk(const Vector3d& vi, int i);
    double B_i_ipk(const Vector3d& vi, int i);
    double C_i_ipk(const Vector3d& vi, int i);

    // Inverse position kinematics
    Vector3d solve_ipk(const Matrix3d& r);

    // Verify constraints
    void verify_position();

    // Inverse velocity kinematics
    Vector3d solve_ivk(const Matrix3d& platform_angle, const Vector3d& platform_velocity);

    // YPR velocity to angular velocity
    Vector3d ypr_to_xyz_velocity(const Vector3d& ypr_velocity, const Vector3d& ypr_point);

private:
    bool isclose(double a, double b, double tol=1e-6);
};
