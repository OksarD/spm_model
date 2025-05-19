import numpy as np
from numpy import pi, cos, sin, sqrt, acos
import cmath
from math import degrees, radians

def solve_quadratic(a, b, c):
    discriminant = b**2 - 4*a*c
    sqrt_discriminant = cmath.sqrt(discriminant)
    root1 = (-b + sqrt_discriminant) / (2*a)
    root2 = (-b - sqrt_discriminant) / (2*a)
    return (root1, root2)

def plot_vector(p, colour, ax):
    origin = np.array([0,0,0])
    ax.quiver(*origin, p[0], p[1], p[2], color=colour, linewidths = 3)  # 'c' for color, 'marker' for shape

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
    
def R_x(a):
    return np.array([[1, 0, 0],
                    [0, cos(a), -sin(a)],
                    [0, sin(a), cos(a)]])

def R_y(a):
    return np.array([[cos(a), 0, sin(a)],
                    [0, 1, 0],
                    [-sin(a), 0, cos(a)]])

def R_z(a):
    return np.array([[cos(a), -sin(a), 0],
                    [sin(a), cos(a), 0],
                    [0, 0, 1]])