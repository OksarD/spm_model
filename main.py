import numpy as np
from numpy import pi, cos, sin, sqrt, acos, atan2, degrees, radians
import matplotlib.pyplot as plt
import manipulator
from utils import *

# Methods used to solve kinematics are derived from this paper:
# https://link.springer.com/chapter/10.1007/978-3-030-75271-2_4

# Parameters
a1 = pi/4; # alpha 1
a2 = pi/2; # alpha 2
b = pi/2; # beta

spm = manipulator.Coaxial_SPM(a1, a2, b)

def plot_vector(p, colour, ax):
    origin = np.array([0,0,0])
    ax.quiver(*origin, p[0], p[1], p[2], color=colour, linewidths = 3)  # 'c' for color, 'marker' for shape

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Desired angle

    # yaw = pi/4
    # pitch = pi/6
    # roll = -pi/12
    x = 1
    y = 1
    z = 0
    a = pi/6
    axis = unit_vector(np.array([x,y,z]))
    desired_rotation = R_axis(axis, a)

    # Desired angular velocity w = (w_x,w_y,w_z)
    w_x = 0
    w_y = 0
    w_z = 0
    desired_velocity = np.array([w_x, w_y, w_z])

    # solve kinematics
    actuator_angles = spm.solve_ipk(desired_rotation)
    actuator_velocity = spm.solve_ivk(desired_rotation, desired_velocity)
    fpk_rotation = spm.solve_fpk(actuator_angles)

    # Display resulting vectors
    v_colours = ['r', 'g', 'b']
    w_colours = ['darkred', 'darkgreen', 'darkblue']
    fpk_v_colours = ['c', 'm', 'y']
    plot_vector(axis, 'gray', ax)
    plot_vector(spm.u, 'black', ax)
   
    for i in spm.i_range:
        # print actuator state
        print("Joint %i" % i)
        print("Actuator angle (deg): %.2f" % degrees(actuator_angles[i]))
        print("Actuator velocity (deg/s): %.2f" % degrees(actuator_velocity[i]))
        plot_vector(spm.v[i], v_colours[i], ax)
        plot_vector(spm.w[i], w_colours[i], ax)
        plot_vector(spm.v_fpk[i], fpk_v_colours[i], ax)

    # verify positional kinematics with matching rotation matrices
    print("desired_r_matrix:")
    print(desired_rotation)
    print("fpk_r_matrix:")
    print(fpk_rotation)

    # plot graph
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)
    ax.set_box_aspect((1, 1, 1))
    plt.show()

main()
