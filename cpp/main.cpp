#include <iostream>
#include "CoaxialSPM.hpp"

using namespace std;
using namespace Eigen;

// g++ main.cpp CoaxialSPM.cpp -I libs/eigen-3.4.0

int main() {
    // Define manipulator parameters (angles in radians)
    double a1 = PI / 4;   // 45 deg
    double a2 = PI / 2;   // 90 deg
    double b  = PI / 2;   // 90 deg

    // Create manipulator
    Coaxial_SPM spm(a1, a2, b);

    // Define a platform orientation (yaw, pitch, roll)
    Vector3d ypr(3, -0.1, 0.1); // radians
    Matrix3d R = spm.R_ypr(ypr);
    
    // Define a platform angular velocity in body frame
    Vector3d ypr_platform_velocity(0, 0.1, 0.5); // rad/s
    Vector3d xyz_platform_velocity = spm.ypr_to_xyz_velocity(ypr_platform_velocity, ypr);

    cout << "xyz_velocity\n" << xyz_platform_velocity << endl;
    cout << "R matrix\n" << R << endl;
    try {
        // Solve inverse velocity kinematics
        Vector3d input_velocity = spm.solve_ivk(R, xyz_platform_velocity);

        cout << "Platform angular velocity: " << ypr_platform_velocity << endl;
        cout << "Joint input velocities:    " << input_velocity << endl;
    } 
    catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
    }

    return 0;
}
