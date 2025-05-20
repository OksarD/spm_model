import numpy as np
from numpy import pi, cos, sin, sqrt, acos
import cmath
from utils import *
from math import degrees, radians, isclose
from scipy import optimize
from scipy.spatial.transform import Rotation

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
        self.w_fpk = None
        self.v_fpk = None
        self.v_fpk = None
        self.actuator_origin = pi
        self.actuator_direction = -1

    def R_ypr(self, angle): # for euler angles
        y = angle[0] # z rotation
        p = angle[1] # y rotation
        r = angle[2] # x rotation
        return R_z(y) @ R_y(p) @ R_x(r)

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
    
    def solve_ipk(self, platform_angle):
        R = self.R_ypr(platform_angle)
        self.v = [R@self.v_origin[i] for i in self.i_range]
        self.n = R@self.n_origin
        A = [self.A_i(self.v[i], i) for i in self.i_range]
        B = [self.B_i(self.v[i], i) for i in self.i_range]
        C = [self.C_i(self.v[i], i) for i in self.i_range]
        T = [solve_quadratic(A[i], B[i]*2, C[i])[0] for i in self.i_range] # we choose the first solution [0] because it lines up with the orientation of our manipulator
        input_angle = np.array([cmath.atan(T[i]).real*2 for i in self.i_range])
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

    def solve_ivk(self, platform_angle, platform_velocity):
        self.solve_ipk(platform_angle) # solve for w and v unit vectors at operting point
        A = np.array([np.cross(self.w[i], self.v[i]).T for i in self.i_range])
        B = np.diag([np.dot(np.cross(self.u, self.w[i]), self.v[i]) for i in self.i_range])
        self.J = np.linalg.inv(B)@A  
        input_velocity = self.J@platform_velocity.T
        print(platform_velocity.shape)
        return input_velocity

    def fpk_system(self, v_out):
        # Initialize residuals
        r = np.zeros(9)
        # rename variables for ease of use
        v0_x = v_out[0]
        v0_y = v_out[1]
        v0_z = v_out[2]
        v1_x = v_out[3]
        v1_y = v_out[4]
        v1_z = v_out[5]
        v2_x = v_out[6]
        v2_y = v_out[7]
        v2_z = v_out[8]
        # w[0][2] is w_0_y (first number is ith w vector, second number is xyz selection)
        w = self.w_fpk
        # angle to intermediate joint constraint
        r[0] = w[0,0]*v0_x + w[0,1]*v0_y + w[0,2]*v0_z
        r[1] = w[1,0]*v1_x + w[1,1]*v1_y + w[1,2]*v1_z
        r[2] = w[2,0]*v2_x + w[2,1]*v2_y + w[2,2]*v2_z
        # vector angular constraint
        r[3] = v0_x*v1_x + v0_y*v1_y + v0_z*v1_z - cos(2*pi/3)
        r[4] = v1_x*v2_x + v1_y*v2_y + v1_z*v2_z - cos(2*pi/3)
        r[5] = v2_x*v0_x + v2_y*v0_y + v2_z*v0_z - cos(2*pi/3)
        # unit vector magnitude constraint
        r[6] = v0_x**2 + v0_y**2 + v0_z**2 - 1
        r[7] = v1_x**2 + v1_y**2 + v1_z**2 - 1
        r[8] = v2_x**2 + v2_y**2 + v2_z**2 - 1
        return r

    def solve_fpk(self, input_angles):
        # decouple yaw by subtracting first angle
        yaw_offset = wrap_rad(self.actuator_direction*(input_angles[0] - self.actuator_origin))
        print("In_0", degrees(yaw_offset))
        offset_input_angles = input_angles + yaw_offset
        init_v = [1,1,1,-1,-1,1,1,-1,1] # initial vector guess which corresponds to l-l-l configuration
        # nonlinear canonical system of equations
        self.w_fpk = np.array([self.w_i(offset_input_angles[i], i) for i in self.i_range])
        v_out = optimize.fsolve(func=self.fpk_system, x0=init_v)
        self.v_fpk = np.array([v_out[i*3:i*3+3] for i in self.i_range])
        self.v_fpk = [R_z(yaw_offset) @ self.v_fpk[i] for i in self.i_range]
        return None
