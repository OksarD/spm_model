#include "sensor_fusion.hpp"

using namespace std;
using namespace Eigen;

// ========== Utility Functions ==========

Matrix4f gyro_transition_matrix(Vector3f& gyro_xyz, float dt) {
    Matrix4f f;
    f << 0, -gyro_xyz[0], -gyro_xyz[1], -gyro_xyz[2],
            gyro_xyz[0], 0, gyro_xyz[2], -gyro_xyz[1],
            gyro_xyz[1], -gyro_xyz[2], 0, gyro_xyz[0],
            gyro_xyz[2], gyro_xyz[1], -gyro_xyz[0], 0;
    return Matrix4f::Identity() + (dt/2) * f;
}