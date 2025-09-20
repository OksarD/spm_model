import numpy as np
from numpy import pi, cos, sin, sqrt, acos, atan2, degrees, radians
from math import floor
import matplotlib.pyplot as plt
import manipulator
from utils import *
from datetime import datetime
from scipy.interpolate import RegularGridInterpolator
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata

np.set_printoptions(formatter={'float': '{:.3f}'.format})

# config
SAVE_TABLE = False
TEST_INTERP_ACCURACY = False
PLOT_INTERP = True

# SPM Parameters
a1 = pi/4; # alpha 1
a2 = pi/2; # alpha 2
b = pi/2; # beta

spm = manipulator.Coaxial_SPM(a1, a2, b)

# Table parameters
TABLE_DIV = 180
NAN_CODE = -999
ANGLE_MIN = -pi
ANGLE_MAX = pi
M0_ANGLE = 0

def format_table(arr):
    f_table = []
    f_table_str = ""
    for row in arr:
        f_row = []
        for point in row:
            if type(point) == list or type(point) == np.ndarray:
            # will format if the point is a vector or scalar
                for i in point:
                    f_point = []
                    if np.isnan(i):
                        f_point.append(NAN_CODE) # error value if the fpk function doesnt solve
                    else:
                        f_point.append(float(point))
                    f_row.append(f_point)
            else:
                if np.isnan(point):
                    f_row.append(NAN_CODE) # error value if the fpk function doesnt solve
                else:
                    f_row.append(float(point))
        f_table.append(f_row)
    for i in f_table[:-1]:
        f_table_str += format(i) + ',\n'
    f_table_str += format(f_table[-1])
    return f_table_str

def bilinear_interp_square(xq, yq, min, max, div, Z):
    # find indices of grid square
    interval = (max-min)/div
    i = int(floor((xq-min)/interval))
    j = int(floor((yq-min)/interval))
    # print("q:", xq, yq, "ij:" , i, j)
    # print("interval:", interval)
    # out of bounds check
    if i < 0 or j < 0 or i > div or j > div:
        return np.nan
    # grid cell corners
    x1, x2 = min + i*interval, min + (i+1)*interval
    y1, y2 = min + j*interval, min + (j+1)*interval

    Q11 = Z[i, j]
    Q21 = Z[i+1, j]
    Q12 = Z[i, j+1]
    Q22 = Z[i+1, j+1]

    # propogate nan value from corner values
    # if Q11 == np.nan or Q21 == np.nan or Q12 == np.nan or Q22 == np.nan:
    #     return np.nan
    # bilinear formula
    return (
        Q11 * (x2 - xq) * (y2 - yq) +
        Q21 * (xq - x1) * (y2 - yq) +
        Q12 * (x2 - xq) * (yq - y1) +
        Q22 * (xq - x1) * (yq - y1)
    ) / ((x2 - x1) * (y2 - y1))

def fill_griddata(Z, method="linear"):
    x = np.arange(Z.shape[0])
    y = np.arange(Z.shape[1])
    X, Y = np.meshgrid(x, y, indexing="ij")

    # valid points
    mask = ~np.isnan(Z)
    points = np.column_stack((X[mask], Y[mask]))
    values = Z[mask]

    # interpolate all points
    Zi = griddata(points, values, (X, Y), method=method)
    return Zi

def main():
    print("Generating yaw position forward kinematic table (assimung first motor is at zero)")
    
    # generate fpk table
    table = np.zeros((TABLE_DIV+1, TABLE_DIV+1)) # add 1 for endpoint
    motor_angles = np.linspace(ANGLE_MIN,ANGLE_MAX, num=TABLE_DIV+1, endpoint=True) 
    print("Motor angles:", motor_angles)
    #print(motor_angle_linspace)
    for i in range(TABLE_DIV+1):
        for j in range(TABLE_DIV+1):   
            m1_angle = motor_angles[i]
            m2_angle = motor_angles[j]   
            actuator_angle = np.array([M0_ANGLE,m1_angle,m2_angle])
            # solve kinematics
            ypr_fpk = spm.solve_fpk(actuator_angle, ignore_error=True)
            table[i, j] = ypr_fpk[0]

    filled_table = fill_griddata(table)
    # lookup_table_unwrapped = np.unwrap(lookup_table, axis=0)
    # lookup_table_unwrapped_2 = np.unwrap(lookup_table, axis=1)

    print("Lookup Table")
    print(table)
    table_formatted = format_table(table)
    #print(table_formatted)

    # test table accuracy
    #interp = RegularGridInterpolator((motor_angles, motor_angles), np.nan_to_num(lookup_table, nan=0))
    if TEST_INTERP_ACCURACY:
        TEST_DIV = 10
        motor_angles_test = np.linspace(ANGLE_MIN,ANGLE_MAX, num=TEST_DIV, endpoint=False)
        interp_values = np.zeros((TEST_DIV, TEST_DIV))
        real_values = np.zeros((TEST_DIV, TEST_DIV))
        inaccuracy = np.zeros((TEST_DIV, TEST_DIV))

        for i in range(TEST_DIV):
            for j in range(TEST_DIV):
                m1_angle = motor_angles_test[i]
                m2_angle = motor_angles_test[j]
                actuator_angle = np.array([M0_ANGLE,m1_angle,m2_angle])
                yaw_interp = bilinear_interp_square(m1_angle, m2_angle, ANGLE_MIN, ANGLE_MAX, TABLE_DIV, table)
                yaw_real = spm.solve_fpk(actuator_angle, ignore_error=True)[0]
                if np.isnan(yaw_real) or np.isnan(yaw_interp): # propogate nan values
                    inaccuracy[i,j] = np.nan
                else:
                    inaccuracy[i,j] = angle_subtract(yaw_interp, yaw_real)

                interp_values[i,j] = yaw_interp
                real_values[i,j] = yaw_real
        print("Interpolated Test Values:")
        print(interp_values)
        print("Real Test Values:")
        print(real_values)

        print("Inaccuracies:")
        print(inaccuracy)
        print("Max inaccuracy", np.nanmax(inaccuracy))

    if PLOT_INTERP:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        M1_AX, M2_AX = np.meshgrid(motor_angles, motor_angles, indexing="ij")
        ax.plot_surface(M1_AX, M2_AX, filled_table, cmap="viridis")

        ax.set_xlabel("M1 axis")
        ax.set_ylabel("M2 axis")
        ax.set_zlabel("Yaw value")
        plt.show()

    if SAVE_TABLE:
        timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        filename = "yaw_table_" + timestamp + ".txt"
        print("Written to", filename)
        with open(filename, "w") as f:
            f.write(table_formatted)

main()