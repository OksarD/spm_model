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

    # Desired angle in ypr
    yaw = pi/4
    pitch = pi/6
    roll = -pi/12
    ypr = np.array([yaw,pitch,roll])
    R_ypr = spm.R_ypr(ypr)

    # Desired angular velocity in ypr
    w_yaw = 1
    w_pitch = 1
    w_roll = 1
    w_ypr = np.array([w_yaw, w_pitch, w_roll])
    w_xyz = spm.ypr_to_xyz_velocity(w_ypr, ypr)

    # solve kinematics
    actuator_angles = spm.solve_ipk(R_ypr)
    actuator_velocity = spm.solve_ivk(R_ypr, w_xyz)
    ypr_fpk = spm.solve_fpk(actuator_angles)

    # verify positional kinematics with matching rotation matrices
    print("desired_ypr:", ypr)
    print("fpk_ypr:", ypr_fpk)

    # verify Jacobian and ypr_to_xyz transformation functions with matching actuator angles
    epsilon = 1e-3 # arbitrarily small angular positional change
    dt = np.array([epsilon/w_ypr[i] for i in spm.i_range])
    a_i = spm.solve_ipk(spm.R_ypr(ypr))
    a_f = spm.solve_ipk(spm.R_ypr(ypr + epsilon))
    actuator_velocity_epsilon = (a_f - a_i)/dt
    print("discrete actuator velocity", actuator_velocity_epsilon)
    print("Jacobian actuator velocity", actuator_velocity)

    # Display resulting vectors
    v_colours = ['r', 'g', 'b']
    w_colours = ['darkred', 'darkgreen', 'darkblue']
    fpk_v_colours = ['c', 'm', 'y']
    plot_vector(spm.u, 'black', ax)
   
    for i in spm.i_range:
        # print actuator state
        print("Joint %i" % i)
        print("Actuator angle (deg): %.2f" % degrees(actuator_angles[i]))
        print("Actuator velocity (deg/s): %.2f" % degrees(actuator_velocity[i]))
        plot_vector(spm.v[i], v_colours[i], ax)
        plot_vector(spm.w[i], w_colours[i], ax)
        plot_vector(spm.v_fpk[i], fpk_v_colours[i], ax)

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
