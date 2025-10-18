#include "sensor_fusion.hpp"

// sensor fusion functions

Matrix4f omega(Vector3f& gyro) {
    Matrix4f om;
    om << 0, -gyro[0], -gyro[1], -gyro[2],
             gyro[0], 0,  gyro[2], -gyro[1],
             gyro[1], -gyro[2], 0,  gyro[0],
             gyro[2],  gyro[1], -gyro[0], 0;
    return om;
}

Vector4f gyro_predict_f(Vector4f& x, Matrix4f& omega, float dt) {
    Vector4f x_new = x + 0.5 * omega * x * dt;
    return x_new.normalized();
};

Matrix4f gyro_jacobian_F(Matrix4f& omega, float dt) {
    return Matrix4f::Identity() + 0.5 * omega * dt;
};