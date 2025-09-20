import numpy as np
from numpy import pi, cos, sin, sqrt, acos, atan2, isclose
import cmath
from utils import *
from scipy import optimize
import scipy.spatial.transform as transform
import warnings
warnings.filterwarnings("error")

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

    # calculate rotation amtrix from yaw-pitch-roll input
    def R_ypr(self, angle):
        yaw = angle[0]
        pitch = angle[1]
        roll = angle[2]
        return R_z(yaw) @ R_y(pitch) @ R_x(roll)

    # eta defines how a function changes for the three limbs
    def eta_i(self, i):
        return 2*i*pi/3 # assume i = (0,1,2)

    # calculate the vector of an ith itermediary joint vector
    def w_i(self, in_i, i): 
        e_i = self.eta_i(i)
        return np.array([cos(e_i-in_i)*sin(self.a1), sin(e_i-in_i)*sin(self.a1), -cos(self.a1)]).T

    # caluclate the home position of the ith platform joint vector
    def v_i_origin(self, i):
        e_i = self.eta_i(i)
        return np.array([-sin(e_i)*sin(self.b), cos(e_i)*sin(self.b), cos(self.b)]).T

    # calculate the ith A term in the inverse positional kinematic problem
    def A_i_ipk(self, v_i, i):
        e_i = self.eta_i(i)
        v_ix = v_i[0]
        v_iy = v_i[1]
        v_iz = v_i[2]
        return -v_ix*cos(e_i)*sin(self.a1) - v_iy*sin(e_i)*sin(self.a1) - v_iz*cos(self.a1) - cos(self.a2)
    
    # calculate the ith A term in the inverse positional kinematic problem
    def B_i_ipk(self, v_i, i):
        e_i = self.eta_i(i)
        v_ix = v_i[0]
        v_iy = v_i[1]
        return v_ix*sin(e_i)*sin(self.a1) - v_iy*cos(e_i)*sin(self.a1)
    
    # calculate the ith A term in the inverse positional kinematic problem
    def C_i_ipk(self, v_i, i):
        e_i = self.eta_i(i)
        v_ix = v_i[0]
        v_iy = v_i[1]
        v_iz = v_i[2]
        return v_ix*cos(e_i)*sin(self.a1) + v_iy*sin(e_i)*sin(self.a1) - v_iz*cos(self.a1) - cos(self.a2)
    
    # algebraically solves positional inverse kinematic problem
    def solve_ipk(self, r, exception=True):
        self.v = [r@self.v_origin[i] for i in self.i_range]
        self.n = r@self.n_origin
        A = [self.A_i_ipk(self.v[i], i) for i in self.i_range]
        B = [self.B_i_ipk(self.v[i], i) for i in self.i_range]
        C = [self.C_i_ipk(self.v[i], i) for i in self.i_range]
        T = [solve_quadratic(A[i], B[i]*2, C[i])[0] for i in self.i_range] # we choose the first solution [0] because it lines up with the orientation of our manipulator
        input_angle = np.array([cmath.atan(T[i]).real*2 for i in self.i_range])
        self.w = [self.w_i(input_angle[i], i) for i in self.i_range] # intermediate joint unit vector
        if self.verify_position(exception) == True:
            return input_angle
        else:
            return [np.nan, np.nan, np.nan]
    
    # use system paramaters to verify if the current state is valid
    def verify_position(self, exception=True):
        invalid_param = None
        for i in self.i_range:
            if isclose(angle_between(self.u, self.w[i]), self.a1) == False:
                invalid_param = "a1"
            elif isclose(angle_between(self.w[i], self.v[i]), self.a2) == False:
                invalid_param = "a2"
            elif isclose(angle_between(self.v[i], self.n), self.b) == False:
                invalid_param = "b"
            if invalid_param != None:
                if exception == True:
                    raise BaseException("Rotation invalid! paramater %s is incorrect" % invalid_param)
                return False
            else:
                return True
    
    # algebraically solves inverse velocity kinematic problem using Jacobian matrix
    def solve_ivk(self, platform_angle, platform_velocity):
        self.solve_ipk(platform_angle) # solve for w and v unit vectors at operting point
        # print("v\n", self.v)
        # print("w\n", self.w)
        A = np.array([np.cross(self.w[i], self.v[i]).T for i in self.i_range])
        # print("A")
        # print(A)
        B = np.diag([np.dot(np.cross(self.u, self.w[i]), self.v[i]) for i in self.i_range])
        # print("B:")
        # print(B)
        self.J = np.linalg.inv(B)@A  
        # print("J:")
        # print(self.J)
        input_velocity = self.J@platform_velocity.T
        return input_velocity



    # quadratic system to solve fpk problem. Has eight solutions so the initial guess must be chosen carefully to ensure the correct solution is found
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

    # numerically solves fpk problem and returns the rotation matrix
    def solve_fpk(self, input_angles, ignore_error=False):
        # decouple yaw by subtracting first angle
        # also tirn actuator angle/velocity positive (around the up Z axis instead of the default down)
        yaw_offset = wrap_rad(self.actuator_direction*(input_angles[0] - self.actuator_origin))
        # print("yaw_offset", yaw_offset)
        offset_input_angles = input_angles + yaw_offset
        init_v = [1,1,1,-1,-1,1,1,-1,1] # initial vector guess which corresponds to l-l-l configuration
        # nonlinear canonical system of equations
        self.w_fpk = np.array([self.w_i(offset_input_angles[i], i) for i in self.i_range])
        try:
            v_out = optimize.fsolve(func=self.fpk_system, x0=init_v)
            self.v_fpk = np.array([v_out[i*3:i*3+3] for i in self.i_range])
            self.v_fpk = [R_z(yaw_offset) @ self.v_fpk[i] for i in self.i_range]
            # print("v_fpk")
            # print(self.v_fpk)
            ypr = self.unwind_ypr(self.v_fpk)
        except RuntimeWarning as warn:
            if ignore_error == True:
                ypr_invalid = [np.nan, np.nan, np.nan]
                return ypr_invalid
            else:
                raise(warn)
        return ypr
    
    # unwinds ypr angles from three-vector (v) platform representation. Angular range is -180 <= a < 180
    def unwind_ypr(self, v):
        # normal, pitch and roll axes in the body frame
        roll_ax, pitch_ax, normal = self.get_orthonormals(v)
        #print("r_ax, p_ax", roll_ax, pitch_ax)
        normal_z = np.array([0,0,1])
        # using angle_between will not provide angles above 180, so the roll/pitch unwind range is limited to +/- 90 degrees
        roll_projected_vector = unit_vector(project_to_plane(roll_ax, normal_z))
        #print('roll_proj_v', roll_projected_vector)
        roll = pi/2 - angle_between(roll_projected_vector, pitch_ax)
        #print("roll", roll) 
        pitch_ax = R_axis(roll_ax, -roll) @ pitch_ax
        #print("pitch_ax", pitch_ax)
        pitch = -(pi/2 - angle_between(normal_z, roll_ax))
        #print("pitch", pitch)
        roll_ax = R_axis(pitch_ax, -pitch) @ roll_ax
        #print("roll_ax", roll_ax)
        # use trig values to convert unit vector to angle to avoid 180 degree constraint
        yaw = atan2(roll_ax[1], roll_ax[0])
        return np.array([yaw, pitch, roll])
    
    # get orthonormal vectors (xyz) relative to the body frame
    def get_orthonormals(self, v):
        y_v = v[0]
        z_v = unit_vector(np.cross(v[0], v[1]))
        # minus puts the x axis in the positive x direction
        x_v = -unit_vector(np.cross(z_v, y_v))
        return x_v, y_v, z_v
    
    def ypr_to_xyz_velocity(self, ypr_velocity, ypr_point):
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