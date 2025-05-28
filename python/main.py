import numpy as np
from numpy import pi, cos, sin, sqrt, acos, atan2, degrees, radians
import matplotlib.pyplot as plt
import manipulator
from utils import *

np.set_printoptions(formatter={'float': '{:.3f}'.format})

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

    # Desired angle in ypr (rad/s)
    yaw = pi/4
    pitch = pi/6
    roll = pi/12
    ypr = np.array([wrap_rad(yaw),wrap_rad(pitch),wrap_rad(roll)])
    R_ypr = spm.R_ypr(ypr)

    # Desired angular velocity in ypr (rad/s)
    w_yaw = 1
    w_pitch = 1
    w_roll = 1
    w_ypr = np.array([w_yaw, w_pitch, w_roll])
    w_xyz = spm.ypr_to_xyz_velocity(w_ypr, ypr)

    print("w_xyz", w_xyz)

    # solve kinematics
    actuator_angles = spm.solve_ipk(R_ypr)
    actuator_velocity = spm.solve_ivk(R_ypr, w_xyz)
    ypr_fpk = spm.solve_fpk(actuator_angles)

    # verify positional kinematics with matching rotation matrices
    print("desired_ypr:", ypr)
    print("fpk_ypr:", ypr_fpk)

    # verify Jacobian and ypr_to_xyz transformation functions with a 
    # discrete approximation based on positional kinematics
    dt_epsilon = 1e-6 # arbitrarily small time change
    ypr_f = [ypr[i] + w_ypr[i]*dt_epsilon for i in spm.i_range]
    a_i = spm.solve_ipk(spm.R_ypr(ypr))
    a_f = spm.solve_ipk(spm.R_ypr(ypr_f))
    a_delta = np.array([closest_angular_delta(a_f[i],a_i[i]) for i in spm.i_range])
    actuator_velocity_approx = a_delta/dt_epsilon
    print("approx actuator velocity", actuator_velocity_approx)
    print("Jacobian actuator velocity", actuator_velocity)

    # Display resulting vectors
    v_colours = ['r', 'g', 'b']
    w_colours = ['darkred', 'darkgreen', 'darkblue']
    fpk_v_colours = ['c', 'm', 'y']
    plot_vector(spm.u, 'black', ax)
   
    for i in spm.i_range:
        # print actuator state
        print("Joint %i" % i)
        print("Actuator angle: %.2f" % (actuator_angles[i]))
        print("Actuator velocity: %.2f" % (actuator_velocity[i]))
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
