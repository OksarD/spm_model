#include "CoaxialSPM.hpp"
#include <iostream>

#define VERIFY_KINEMATICS

using namespace std;
using namespace Eigen;

#define PI 3.1415926535897932384

// ================== Utility Functions ================== //

pair<complex<float>, complex<float>> solve_quadratic(float a, float b, float c) {
    complex<float> discriminant = b*b - 4*a*c;
    complex<float> sqrt_disc = sqrt(discriminant);
    complex<float> root1 = (-b + sqrt_disc) / (2.0f*a);
    complex<float> root2 = (-b - sqrt_disc) / (2.0f*a);
    return {root1, root2};
}

Vector3f unit_vector(const Vector3f& v) {
    return v / v.norm();
}

float angle_between(const Vector3f& a, const Vector3f& b) {
    return acos(a.dot(b) / (a.norm() * b.norm()));
}

float wrap_rad(float angle) {
    if (angle >= PI) return angle - 2*PI;
    if (angle < -PI) return angle + 2*PI;
    return angle;
}

Matrix3f R_x(float a) {
    float sin_a = sin(a);
    float cos_a = cos(a);
    Matrix3f R;
    R << 1, 0, 0,
         0, cos_a, -sin_a,
         0, sin_a, cos_a;
    return R;
}

Matrix3f R_y(float a) {
    float sin_a = sin(a);
    float cos_a = cos(a);
    Matrix3f R;
    R << cos_a, 0, sin_a,
         0, 1, 0,
        -sin_a, 0, cos_a;
    return R;
}

Matrix3f R_z(float a) {
    float sin_a = sin(a);
    float cos_a = cos(a);
    Matrix3f R;
    R << cos_a, -sin_a, 0,
         sin_a, cos_a, 0,
         0, 0, 1;
    return R;
}

Matrix3f R_axis(const Vector3f& axis, float angle) {
    float ux = axis(0), uy = axis(1), uz = axis(2);
    float c = cos(angle), s = sin(angle), r = 1 - c;

    Matrix3f R;
    R << ux*ux*r + c,      ux*uy*r - uz*s,  ux*uz*r + uy*s,
         ux*uy*r + uz*s,   uy*uy*r + c,     uy*uz*r - ux*s,
         ux*uz*r - uy*s,   uy*uz*r + ux*s,  uz*uz*r + c;
    return R;
}

// ================== Coaxial_SPM Class ================== //

Coaxial_SPM::Coaxial_SPM(float a1_, float a2_, float b_)
    : a1(a1_), a2(a2_), b(b_), u(0,0,-1), n_origin(0,0,1),
      actuator_origin(PI), actuator_direction(-1),
      sin_a1(sin(a1_)), cos_a1(cos(a1_)),
      sin_a2(sin(a2_)), cos_a2(cos(a2_)),
      sin_b(sin(b_)), cos_b(cos(b_))
{
    for (int i=0; i<3; i++) {
        eta.push_back(eta_i(i));
        v_origin.push_back(v_i_origin(i));
        sin_eta.push_back(sin(eta_i(i)));
        cos_eta.push_back(cos(eta_i(i)));
    }
}

Vector3f Coaxial_SPM::v_i_origin(int i) {
    float e = eta_i(i);
    return Vector3f(-sin(e)*sin_b,
                     cos(e)*sin_b,
                     cos_b);
}

Matrix3f Coaxial_SPM::R_ypr(const Vector3f& angle) {
    return R_z(angle(0)) * R_y(angle(1)) * R_x(angle(2));
}

float Coaxial_SPM::eta_i(int i) {
    return 2*i*PI/3.0;
}

Vector3f Coaxial_SPM::w_i(float in_i, int i) {
    return Vector3f(cos(eta[i]-in_i)*sin_a1,
                    sin(eta[i]-in_i)*sin_a1,
                    -cos_a1);
}

float Coaxial_SPM::A_i_ipk(const Vector3f& vi, int i) {
    return -vi(0)*cos_eta[i]*sin_a1 - vi(1)*sin_eta[i]*sin_a1 - vi(2)*cos_a1 - cos_a2;
}

float Coaxial_SPM::B_i_ipk(const Vector3f& vi, int i) {
    return vi(0)*sin_eta[i]*sin_a1 - vi(1)*cos_eta[i]*sin_a1;
}

float Coaxial_SPM::C_i_ipk(const Vector3f& vi, int i) {
    return vi(0)*cos_eta[i]*sin_a1 + vi(1)*sin_eta[i]*sin_a1 - vi(2)*cos_a1 - cos_a2;
}

Vector3f Coaxial_SPM::solve_ipk(const Matrix3f& r) {
    v.clear(); w.clear();
    for (int i=0; i<3; i++) {
        v.push_back(r * v_origin[i]);
    }
    n = r * n_origin;

    vector<float> A(3), B(3), C(3);
    for (int i=0; i<3; i++) {
        A[i] = A_i_ipk(v[i], i);
        B[i] = B_i_ipk(v[i], i);
        C[i] = C_i_ipk(v[i], i);
    }

    Vector3f input_angle;
    for (int i=0; i<3; i++) {
        auto roots = solve_quadratic(A[i], 2*B[i], C[i]);
        float T = roots.first.real();
        input_angle(i) = 2 * atan(T);
        w.push_back(w_i(input_angle(i), i));
    }
    #ifdef VERIFY_KINEMATICS
        verify_position();
    #endif
    return input_angle;
}

void Coaxial_SPM::verify_position() {
    for (int i=0; i<3; i++) {
        if (!isclose(angle_between(u, w[i]), a1))
            cout <<"Rotation invalid! param a1 incorrect" << endl;
        if (!isclose(angle_between(w[i], v[i]), a2))
            cout << "Rotation invalid! param a2 incorrect" << endl;
        if (!isclose(angle_between(v[i], n), b))
            cout << "Rotation invalid! param b incorrect" << endl;
    }
}

Vector3f Coaxial_SPM::solve_ivk(const Matrix3f& platform_angle, const Vector3f& platform_velocity) {
    solve_ipk(platform_angle);

    Matrix3f A;
    for (int i=0; i<3; i++) {
        A.row(i) = w[i].cross(v[i]);
    }

    Matrix3f B = Matrix3f::Zero();
    for (int i=0; i<3; i++) {
        B(i,i) = u.cross(w[i]).dot(v[i]);
    }

    J = B.inverse() * A;

    return J * platform_velocity;
}

Vector3f Coaxial_SPM::ypr_to_xyz_velocity(const Vector3f& ypr_velocity, const Vector3f& ypr_point) {
    float yaw_v = ypr_velocity(0);
    float pitch_v = ypr_velocity(1);
    float roll_v = ypr_velocity(2);
    float yaw = ypr_point(0);
    float pitch = ypr_point(1);

    Vector3f x(1,0,0), y(0,1,0), z(0,0,1);
    Matrix3f E_yaw = R_z(yaw);
    Matrix3f E_pitch = E_yaw * R_y(pitch);

    return yaw_v*z + pitch_v*(E_yaw*y) + roll_v*(E_pitch*x);
}

bool Coaxial_SPM::isclose(float a, float b, float tol) {
    return fabs(a - b) < tol;
}