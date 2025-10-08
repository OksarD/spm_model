#include "sensor_fusion.hpp"

using namespace std;
using namespace Eigen;

// ========== Utility Functions ==========

Matrix4f gyro_transition_matrix(Vector3f gyro_xyz, float dt) {
    Matrix4f f;
    f << 0, -gyro_xyz[0], -gyro_xyz[1], -gyro_xyz[2],
            gyro_xyz[0], 0, gyro_xyz[2], -gyro_xyz[1],
            gyro_xyz[1], -gyro_xyz[2], 0, gyro_xyz[0],
            gyro_xyz[2], gyro_xyz[1], -gyro_xyz[0], 0;
    return Matrix4f::Identity() + (dt/2) * f;
}

Quaternionf ypr_to_q(Vector3f ypr) {
    float sin_y_2 = sin(ypr[0]*0.5);
    float cos_y_2 = cos(ypr[0]*0.5);
    float sin_p_2 = sin(ypr[1]*0.5);
    float cos_p_2 = cos(ypr[1]*0.5);
    float sin_r_2 = sin(ypr[2]*0.5);
    float cos_r_2 = cos(ypr[2]*0.5); 
    // Quaternion in [w,x,y,z] form
    Vector4f q;
    q << cos_r_2 * cos_p_2 * cos_y_2 + sin_r_2 * sin_p_2 * sin_y_2,
        sin_r_2 * cos_p_2 * cos_y_2 - cos_r_2 * sin_p_2 * sin_y_2,
        cos_r_2 * sin_p_2 * cos_y_2 + sin_r_2 * cos_p_2 * sin_y_2,
        cos_r_2 * cos_p_2 * sin_y_2 - sin_r_2 * sin_p_2 * cos_y_2;
    return Quaternionf(q[0], q[1], q[2], q[3]).normalized();
}

Vector3f q_to_ypr(Quaternionf q) {
    Quaternionf qn = q.normalized();
    float w = qn.w(), x = qn.x(), y = qn.y(), z = qn.z();
    float yaw = atan2(2.0f * (w*z + x*y),
                           1.0f - 2.0f * (y*y + z*z));
    float sinp = 2.0f * (w*y - z*x);
    sinp = clamp(sinp, -1.0f, 1.0f);
    float pitch = asin(sinp);
    float roll = atan2(2.0f * (w*x + y*z),
                            1.0f - 2.0f * (x*x + y*y));
    return Vector3f(yaw, pitch, roll);
}