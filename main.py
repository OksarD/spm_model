import numpy as np
from numpy import pi, cos, sin, sqrt, acos
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cmath
from math import degrees, radians

import manipulator
from utils import *

# https://link.springer.com/chapter/10.1007/978-3-030-75271-2_4

# Parameters
a1 = pi/4; # alpha 1
a2 = pi/2; # alpha 2
b = pi/2; # beta

spm = manipulator.Coaxial_SPM(a1, a2, b)

def main():
    input_offset = pi # add input offset to ensure joint angles at home position are zero (required when using first solution)
    # Desired euler angle
    x_rot = 0
    y_rot = 0
    z_rot = 0
    # Desired angular velocity (euler)
    x_vel = pi/6
    y_vel = 0
    z_vel = 0

    desired_angle = np.array([x_rot, y_rot, z_rot])
    actuator_angles = spm.set_platform_position(desired_angle)
    desired_velocity = np.array([x_vel, y_vel, z_vel])
    actuator_velocity = spm.set_platform_velocity(desired_velocity)


    v_colours = ['r', 'g', 'b']
    w_colours = ['darkred', 'darkgreen', 'darkblue']

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plot_vector(spm.u, 'black', ax)

    for i in spm.i_range:
        print("Joint %i" % i)
        print("Actuator angle (deg): %.2f" % degrees(wrap_rad(actuator_angles[i] + input_offset)))
        print("Actuator velocity (deg/s): %.2f" % degrees(actuator_velocity[i]))

        plot_vector(spm.v[i], v_colours[i], ax)
        plot_vector(spm.w[i], w_colours[i], ax)

    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)
    ax.set_box_aspect((1, 1, 1))
    plt.show()

main()
