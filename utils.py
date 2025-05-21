import numpy as np
from numpy import pi, cos, sin, sqrt, acos
import cmath

def solve_quadratic(a, b, c):
    discriminant = b**2 - 4*a*c
    sqrt_discriminant = cmath.sqrt(discriminant)
    root1 = (-b + sqrt_discriminant) / (2*a)
    root2 = (-b - sqrt_discriminant) / (2*a)
    return (root1, root2)

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(a, b):
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    angle = acos(np.dot(a, b) / (a_norm * b_norm))
    return angle
    
def wrap_rad(angle): # wraps between +/-pi
    if angle > pi:
        return angle - pi*2
    elif angle <= -pi:
        return angle + pi*2
    else:
        return angle
    
def R_x(angle):
    return np.array([[1, 0, 0],
                    [0, cos(angle), -sin(angle)],
                    [0, sin(angle), cos(angle)]])

def R_y(angle):
    return np.array([[cos(angle), 0, sin(angle)],
                    [0, 1, 0],
                    [-sin(angle), 0, cos(angle)]])

def R_z(angle):
    return np.array([[cos(angle), -sin(angle), 0],
                    [sin(angle), cos(angle), 0],
                    [0, 0, 1]])

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

def project_to_plane(normal_vector, projected_vector):
    n = normal_vector
    k = projected_vector
    proj_k = (np.dot(k,n)/(np.linalg.norm(n)**2))*n
    return k - proj_k
