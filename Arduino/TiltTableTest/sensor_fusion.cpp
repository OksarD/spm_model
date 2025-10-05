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

Vector4f ypr_to_q(Vector3f ypr) {
    float sin_y_2 = sin(ypr[0]*0.5);
    float cos_y_2 = cos(ypr[0]*0.5);
    float sin_p_2 = sin(ypr[1]*0.5);
    float cos_p_2 = cos(ypr[1]*0.5);
    float sin_r_2 = sin(ypr[2]*0.5);
    float cos_r_2 = cos(ypr[2]*0.5); 
    // quarternion in [w,x,y,z] form
    Vector4f q;
    q << cos_r_2 * cos_p_2 * cos_y_2 + sin_r_2 * sin_p_2 * sin_y_2,
                sin_r_2 * cos_p_2 * cos_y_2 - cos_r_2 * sin_p_2 * sin_y_2,
                cos_r_2 * sin_p_2 * cos_y_2 + sin_r_2 * cos_p_2 * sin_y_2,
                cos_r_2 * cos_p_2 * sin_y_2 - sin_r_2 * sin_p_2 * cos_y_2;
    return q;
}

Vector3f q_to_ypr(Vector4f q) {
    // quarternion in [w,x,y,z] form
    float q_w = q[0];
    float q_x = q[1];
    float q_y = q[2];
    float q_z = q[3];
    Vector3f ypr;
    ypr << atan2(2*(q_w*q_z + q_x*q_y), 1-2*(pow(q_y,2) + pow(q_z,2))),
            -PI/2 + 2*atan2(sqrt(1+2*(q_w*q_y - q_x*q_z)), sqrt(1-2*(q_w*q_y - q_x*q_z))),
            atan2(2*(q_w*q_x + q_y*q_z), 1-2*(pow(q_x,2) + pow(q_y,2)));
    return ypr;
}

Vector4f unit_q(Vector4f q) {
    float norm = sqrt(pow(q[0],2) + pow(q[1],2) + pow(q[2],2) + pow(q[3],2));
    return q / norm;
}