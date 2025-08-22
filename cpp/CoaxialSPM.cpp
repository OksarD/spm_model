#include "CoaxialSPM.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;

// ================== Utility Functions ================== //

pair<complex<double>, complex<double>> solve_quadratic(double a, double b, double c) {
    complex<double> discriminant = b*b - 4*a*c;
    complex<double> sqrt_disc = sqrt(discriminant);
    complex<double> root1 = (-b + sqrt_disc) / (2.0*a);
    complex<double> root2 = (-b - sqrt_disc) / (2.0*a);
    return {root1, root2};
}

Vector3d unit_vector(const Vector3d& v) {
    return v / v.norm();
}

double angle_between(const Vector3d& a, const Vector3d& b) {
    return acos(a.dot(b) / (a.norm() * b.norm()));
}

double wrap_rad(double angle) {
    if (angle >= PI) return angle - 2*PI;
    if (angle < -PI) return angle + 2*PI;
    return angle;
}

Matrix3d R_x(double a) {
    Matrix3d R;
    R << 1, 0, 0,
         0, cos(a), -sin(a),
         0, sin(a), cos(a);
    return R;
}

Matrix3d R_y(double a) {
    Matrix3d R;
    R << cos(a), 0, sin(a),
         0, 1, 0,
        -sin(a), 0, cos(a);
    return R;
}

Matrix3d R_z(double a) {
    Matrix3d R;
    R << cos(a), -sin(a), 0,
         sin(a), cos(a), 0,
         0, 0, 1;
    return R;
}

Matrix3d R_axis(const Vector3d& axis, double angle) {
    double ux = axis(0), uy = axis(1), uz = axis(2);
    double c = cos(angle), s = sin(angle), r = 1 - c;

    Matrix3d R;
    R << ux*ux*r + c,      ux*uy*r - uz*s,  ux*uz*r + uy*s,
         ux*uy*r + uz*s,   uy*uy*r + c,     uy*uz*r - ux*s,
         ux*uz*r - uy*s,   uy*uz*r + ux*s,  uz*uz*r + c;
    return R;
}

// ================== Coaxial_SPM Class ================== //

Coaxial_SPM::Coaxial_SPM(double a1_, double a2_, double b_)
    : a1(a1_), a2(a2_), b(b_), u(0,0,-1), n_origin(0,0,1),
      actuator_origin(PI), actuator_direction(-1)
{
    for (int i=0; i<3; i++) {
        v_origin.push_back(v_i_origin(i));
        eta.push_back(eta_i(i));
    }
}

Matrix3d Coaxial_SPM::R_ypr(const Vector3d& angle) {
    return R_z(angle(0)) * R_y(angle(1)) * R_x(angle(2));
}

double Coaxial_SPM::eta_i(int i) {
    return 2*i*PI/3.0;
}

Vector3d Coaxial_SPM::w_i(double in_i, int i) {
    double e = eta_i(i);
    return Vector3d(cos(e-in_i)*sin(a1),
                    sin(e-in_i)*sin(a1),
                    -cos(a1));
}

Vector3d Coaxial_SPM::v_i_origin(int i) {
    double e = eta_i(i);
    return Vector3d(-sin(e)*sin(b),
                     cos(e)*sin(b),
                     cos(b));
}

double Coaxial_SPM::A_i_ipk(const Vector3d& vi, int i) {
    double e = eta_i(i);
    return -vi(0)*cos(e)*sin(a1) - vi(1)*sin(e)*sin(a1) - vi(2)*cos(a1) - cos(a2);
}

double Coaxial_SPM::B_i_ipk(const Vector3d& vi, int i) {
    double e = eta_i(i);
    return vi(0)*sin(e)*sin(a1) - vi(1)*cos(e)*sin(a1);
}

double Coaxial_SPM::C_i_ipk(const Vector3d& vi, int i) {
    double e = eta_i(i);
    return vi(0)*cos(e)*sin(a1) + vi(1)*sin(e)*sin(a1) - vi(2)*cos(a1) - cos(a2);
}

Vector3d Coaxial_SPM::solve_ipk(const Matrix3d& r) {
    v.clear(); w.clear();
    for (int i=0; i<3; i++) {
        v.push_back(r * v_origin[i]);
    }
    n = r * n_origin;

    vector<double> A(3), B(3), C(3);
    for (int i=0; i<3; i++) {
        A[i] = A_i_ipk(v[i], i);
        B[i] = B_i_ipk(v[i], i);
        C[i] = C_i_ipk(v[i], i);
    }

    Vector3d input_angle;
    for (int i=0; i<3; i++) {
        auto roots = solve_quadratic(A[i], 2*B[i], C[i]);
        double T = roots.first.real();
        input_angle(i) = 2 * atan(T);
        w.push_back(w_i(input_angle(i), i));
    }

    verify_position();
    return input_angle;
}

void Coaxial_SPM::verify_position() {
    for (int i=0; i<3; i++) {
        if (!isclose(angle_between(u, w[i]), a1))
            throw runtime_error("Rotation invalid! param a1 incorrect");
        if (!isclose(angle_between(w[i], v[i]), a2))
            throw runtime_error("Rotation invalid! param a2 incorrect");
        if (!isclose(angle_between(v[i], n), b))
            throw runtime_error("Rotation invalid! param b incorrect");
    }
}

Vector3d Coaxial_SPM::solve_ivk(const Matrix3d& platform_angle, const Vector3d& platform_velocity) {
    solve_ipk(platform_angle);
    // print V and W vectors
    cout << "v0\n" << v[0] << endl;
    cout << "v1\n" << v[1] << endl;
    cout << "v2\n" << v[2] << endl;
    cout << "w0\n" << w[0] << endl;
    cout << "w1\n" << w[1] << endl;
    cout << "w2\n" << w[2] << endl;

    Matrix3d A;
    for (int i=0; i<3; i++) {
        A.row(i) = w[i].cross(v[i]);
    }

    Matrix3d B = Matrix3d::Zero();
    for (int i=0; i<3; i++) {
        B(i,i) = u.cross(w[i]).dot(v[i]);
    }

    J = B.inverse() * A;

    // print A B and J Matrices
    cout << "A\n" << A << endl;
    cout << "B\n" << B << endl;
    cout << "J\n" << J << endl;

    return J * platform_velocity;
}

Vector3d Coaxial_SPM::ypr_to_xyz_velocity(const Vector3d& ypr_velocity, const Vector3d& ypr_point) {
    double yaw_v = ypr_velocity(0);
    double pitch_v = ypr_velocity(1);
    double roll_v = ypr_velocity(2);
    double yaw = ypr_point(0);
    double pitch = ypr_point(1);

    Vector3d x(1,0,0), y(0,1,0), z(0,0,1);
    Matrix3d E_yaw = R_z(yaw);
    Matrix3d E_pitch = E_yaw * R_y(pitch);

    return yaw_v*z + pitch_v*(E_yaw*y) + roll_v*(E_pitch*x);
}

bool Coaxial_SPM::isclose(double a, double b, double tol) {
    return fabs(a - b) < tol;
}