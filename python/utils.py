import numpy as np
from numpy import pi, cos, sin, sqrt, acos, degrees, radians
import cmath

# solve quadratic equation returning two complex numbers (even is the root is repreated)
def solve_quadratic(a, b, c):
    discriminant = b**2 - 4*a*c
    sqrt_discriminant = cmath.sqrt(discriminant)
    root1 = (-b + sqrt_discriminant) / (2*a)
    root2 = (-b - sqrt_discriminant) / (2*a)
    return (root1, root2)

# normalise a vector to a magnitude of 1
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

# calculate minimum angle between two vectors
def angle_between(a, b):
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    angle = acos(np.dot(a, b) / (a_norm * b_norm))
    return angle

# wraps between -pi <= angle < pi
def wrap_rad(angle): 
    if angle >= pi:
        return angle - pi*2
    elif angle < -pi:
        return angle + pi*2
    else:
        return angle

# get rotation matrix of angle around the x axis
def R_x(angle):
    return np.array([[1, 0, 0],
                    [0, cos(angle), -sin(angle)],
                    [0, sin(angle), cos(angle)]])

# get rotation matrix of angle around the y axis
def R_y(angle):
    return np.array([[cos(angle), 0, sin(angle)],
                    [0, 1, 0],
                    [-sin(angle), 0, cos(angle)]])

# get rotation matrix of angle around the z axis
def R_z(angle):
    return np.array([[cos(angle), -sin(angle), 0],
                    [sin(angle), cos(angle), 0],
                    [0, 0, 1]])

# get the rotation matrix from axis-angle co-ordinates
def R_axis(axis, angle):
    u_x = axis[0]
    u_y = axis[1]
    u_z = axis[2]
    r = 1-cos(angle)
    s = sin(angle)
    c = cos(angle)
    return np.array([[(u_x**2)*r + c, u_x*u_y*r - u_z*s, u_x*u_z*r + u_y*s],
                     [u_x*u_y*r + u_z*s, (u_y**2)*r + c, u_y*u_z*r - u_x*s],
                     [u_x*u_z*r - u_y*s, u_y*u_z*r + u_x*s, (u_z**2)*r + c]])

# get the projected vector of a vector to a plane, given the normal vector of that plane
def project_to_plane(normal_vector, projected_vector):
    n = normal_vector
    k = projected_vector
    proj_k = (np.dot(k,n)/(np.linalg.norm(n)**2))*n
    return k - proj_k

# get rotation matrix based on a 3-element list of orthonormal body vectors
def R_orthonormal(o_v):
    return np.array([o_v[0], o_v[1], o_v[2]]).T

# calculates a2 - a1 and accounts for angular discontiuity by choosing the value closest to zero
def angle_subtract(a_2, a_1):
    opt = [a_2 - a_1 + 2*pi, 
            a_2 - a_1 - 2*pi, 
            a_2 - a_1]
    return min(opt, key=abs)

def ypr_to_xyz_velocity(ypr_velocity, ypr_point):
        # https://physics.stackexchange.com/questions/492906/application-of-angular-velocity-to-euler-angles
        yaw_v = ypr_velocity[0]
        pitch_v = ypr_velocity[1]
        roll_v = ypr_velocity[2]
        yaw = ypr_point[0]
        pitch = ypr_point[1]
        x = np.array([1,0,0])
        y = np.array([0,1,0])
        z = np.array([0,0,1])
        E_yaw = R_z(yaw)
        E_pitch = E_yaw@R_y(pitch)
        w = yaw_v*z + pitch_v*(E_yaw@y) + roll_v*(E_pitch@x)
        return w