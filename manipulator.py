import numpy as np
from numpy import pi, cos, sin, sqrt, acos
import cmath
from utils import *
from math import degrees, radians, isclose

class Coaxial_SPM:
    def __init__(self, a1, a2, b): # gamma assumed to be zero for coaxial spm
        self.a1 = a1 # alpha 1
        self.a2 = a2 # alpha 2
        self.b = b # beta
        self.v = None
        self.w = None
        self.n = None
        self.u = np.array([0, 0, -1]).T # u vector face down for all joints
        self.i_range = range(0,3) # joint values are i = (0,1,2)
        self.v_origin = [self.v_i_origin(v_i) for v_i in self.i_range]
        self.n_origin = np.array([0, 0, 1]).T
        self.eta = [self.eta_i(i) for i in self.i_range]
        self.J = None


    def R_euler(self, angle): # for euler angles
        x_r = angle[0] # x rotation
        y_r = angle[1] # y rotation
        z_r = angle[2] # z rotation
        return np.array([[cos(y_r)*cos(z_r), -cos(y_r)*sin(z_r), sin(y_r)],
                        [cos(x_r)*sin(z_r)+sin(x_r)*sin(y_r)*cos(z_r), cos(x_r)*cos(z_r)-sin(x_r)*sin(y_r)*sin(z_r), -sin(x_r)*cos(y_r)],
                        [sin(x_r)*sin(z_r)-cos(x_r)*sin(y_r)*cos(z_r), sin(x_r)*cos(z_r)+cos(x_r)*sin(y_r)*sin(z_r), cos(x_r)*cos(y_r)]])

    def eta_i(self, i):
        return 2*i*pi/3 # assume i = (0,1,2)

    def w_i(self, in_i, i): 
        e_i = self.eta_i(i)
        return np.array([cos(e_i-in_i)*sin(self.a1), sin(e_i-in_i)*sin(self.a1), -cos(self.a1)]).T

    def v_i_origin(self, i):
        e_i = self.eta_i(i)
        return np.array([-sin(e_i)*sin(self.b), cos(e_i)*sin(self.b), cos(self.b)]).T

    def A_i(self, v_i, i):
        e_i = self.eta_i(i)
        v_ix = v_i[0]
        v_iy = v_i[1]
        v_iz = v_i[2]
        return -v_ix*cos(e_i)*sin(self.a1) - v_iy*sin(e_i)*sin(self.a1) - v_iz*cos(self.a1) - cos(self.a2)

    def B_i(self, v_i, i):
        e_i = self.eta_i(i)
        v_ix = v_i[0]
        v_iy = v_i[1]
        return v_ix*sin(e_i)*sin(self.a1) - v_iy*cos(e_i)*sin(self.a1)

    def C_i(self, v_i, i):
        e_i = self.eta_i(i)
        v_ix = v_i[0]
        v_iy = v_i[1]
        v_iz = v_i[2]
        return v_ix*cos(e_i)*sin(self.a1) + v_iy*sin(e_i)*sin(self.a1) - v_iz*cos(self.a1) - cos(self.a2)
    
    def set_platform_position(self, platform_angle):
        R = self.R_euler(platform_angle)
        self.v = [R@self.v_origin[i] for i in self.i_range]
        self.n = R@self.n_origin
        A = [self.A_i(self.v[i], i) for i in self.i_range]
        B = [self.B_i(self.v[i], i) for i in self.i_range]
        C = [self.C_i(self.v[i], i) for i in self.i_range]
        T = [solve_quadratic(A[i], B[i]*2, C[i])[0] for i in self.i_range] # we choose the first solution [0] because it lines up with the orientation of our manipulator
        input_angle = [cmath.atan(T[i]).real*2 for i in self.i_range]  
        self.w = [self.w_i(input_angle[i], i) for i in self.i_range] # intermediate joint unit vector
        self.verify_position()
        return input_angle
    
    def verify_position(self):
        invalid_param = None
        for i in self.i_range:
            if isclose(angle_between(self.u, self.w[i]), self.a1) == False:
                invalid_param = "a1"
            elif isclose(angle_between(self.w[i], self.v[i]), self.a2) == False:
                invalid_param = "a2"
            elif isclose(angle_between(self.v[i], self.n), self.b) == False:
                invalid_param = "b"
            if invalid_param != None:
                raise BaseException("Rotation invalid! paramater %s is incorrect" % invalid_param)

    def set_platform_velocity(self, platform_velocity):
        A = np.array([np.cross(self.w[i], self.v[i]).T for i in self.i_range])
        B = np.diag([np.dot(np.cross(self.u, self.w[i]), self.v[i]) for i in self.i_range])
        self.J = np.linalg.inv(B)@A  
        input_velocity = self.J@platform_velocity.T
        print(platform_velocity.shape)
        return input_velocity



        


