import numpy as np
from numpy import pi, cos, sin, sqrt, acos, atan2, degrees, radians
import matplotlib.pyplot as plt
import manipulator
from utils import *
from datetime import datetime

np.set_printoptions(formatter={'float': '{:.3f}'.format})

# SPM Parameters
a1 = pi/4; # alpha 1
a2 = pi/2; # alpha 2
b = pi/2; # beta

spm = manipulator.Coaxial_SPM(a1, a2, b)

# Table parameters
motor_angle_division = 18
nan_val = -999

def format_array(arr):
    formatted = []
    formatted_str = ""
    for row in arr:
        new_row = []
        for val in row:
            if np.isnan(val):
                new_row.append(nan_val) # error value if the fpk function doesnt solve
            else:
                new_row.append(float(val))  # format floats to 3 decimals
        formatted.append(new_row)
    for i in formatted[:-1]:
        formatted_str += format(i) + ',\n'
    formatted_str += format(formatted[-1])
    return formatted_str

def main():
    print("Generating yaw position forward kinematic table (assimung first motor is at zero)")
    yaw_table = []
    
    motor_angle_linspace = np.linspace(-pi,pi, num=motor_angle_division, endpoint=False)
    #print(motor_angle_linspace)
    m0_angle = 0 # assume first motor at zero
    for m1_angle in motor_angle_linspace:
        table_row = []
        for m2_angle in motor_angle_linspace:            
            actuator_angles = np.array([m0_angle,m1_angle,m2_angle])
            # solve kinematics
            ypr_fpk = spm.solve_fpk(actuator_angles, ignore_error=True)
            yaw_val = ypr_fpk[0]
            table_row.append(yaw_val)
        yaw_table.append(table_row)
    
    #print(yaw_table)
    table_formatted = format_array(yaw_table)
    #print(table_formatted)

    timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    filename = "yaw_table_" + timestamp + ".txt"
    print("Written to", filename)
    with open(filename, "w") as f:
        f.write(table_formatted)

main()