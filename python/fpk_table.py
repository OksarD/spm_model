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
PLOT_TABLE = True
LOAD_TABLE = False
FILENAME_TABLE = "ypr_table.npy"
FILENAME_TABLE_TXT = "table_formatted.txt"

# SPM Parameters
a1 = radians(50); # alpha 1
a2 = radians(75); # alpha 2
b = radians(100); # beta

spm = manipulator.Coaxial_SPM(a1, a2, b)

# Sampling Params
SLOPE_LIMIT = radians(45)

# Table Params
TABLE_DIM = 15
NAN_CODE = -999

class sample_point:
    def __init__(self, _act, _ypr):
        self.act = _act
        self.ypr = _ypr
        self.act_dist = None

def format_table(arr):
    f_table_str = ""
    for row in arr:
        for point in row:
                if np.isnan(point):
                    f_table_str += str(NAN_CODE) + ", " # error value if the fpk function doesnt solve
                else:
                    f_table_str += "{:.6f}, ".format(float(point))
        f_table_str += "\n"
    return f_table_str

# Main Functions

ypr_table = np.zeros((TABLE_DIM, TABLE_DIM, 3))
act_lin = np.linspace(spm.actuator_origin-pi, spm.actuator_origin+pi, TABLE_DIM, endpoint=False)

if LOAD_TABLE == False:
    total_entries = len(act_lin)**2
    entries_complete = 0
    print("Table Dimension:", TABLE_DIM)
    print("Table Size:", total_entries)
    print("Adding YPR entries...")
    for ind_i, i in enumerate(act_lin):
        for ind_j, j in enumerate(act_lin):
            ypr_fpk = spm.solve_fpk([wrap_rad(spm.actuator_origin), wrap_rad(i), wrap_rad(j)], ignore_error=True)
            ypr_table[ind_i, ind_j, :] = ypr_fpk
            entries_complete += 1
            print(f"\r{100*entries_complete/total_entries:.2f}% ", end="", flush=True)
    print("\nDone.")
else:
    print("Loading Saved Table...")
    ypr_table = np.load(FILENAME_TABLE)

if PLOT_TABLE:
    # Yaw
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    M1_AX, M2_AX = np.meshgrid(act_lin, act_lin, indexing="ij")
    ax.plot_surface(M1_AX, M2_AX, ypr_table[:,:,0], cmap="viridis")
    ax.set_xlabel("M1 axis")
    ax.set_ylabel("M2 axis")
    ax.set_zlabel("Yaw value")
    # Pitch
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(M1_AX, M2_AX, ypr_table[:,:,1], cmap="viridis")
    ax.plot_surface(M1_AX, M2_AX, ypr_table[:,:,2], cmap="plasma")
    ax.set_xlabel("M1 axis")
    ax.set_ylabel("M2 axis")
    ax.set_zlabel("Pitch/Roll value")
    plt.show()

if SAVE_TABLE:
    np.save(FILENAME_TABLE, ypr_table)
    print("Written table to", FILENAME_TABLE)
    # format only the yaw component of the table and save it as text for copying
    yaw_table_formatted = format_table(ypr_table[:,:,0])  
    pitch_table_formatted = format_table(ypr_table[:,:,1]) 
    roll_table_formatted = format_table(ypr_table[:,:,2])
    with open("yaw_" + FILENAME_TABLE_TXT, "w") as f:
        f.write(yaw_table_formatted)
    with open("pitch_" + FILENAME_TABLE_TXT, "w") as f:
        f.write(pitch_table_formatted) 
    with open("roll_" + FILENAME_TABLE_TXT, "w") as f:
        f.write(roll_table_formatted) 
    print("Formatted as txt and written to yaw/pitch/roll_" + FILENAME_TABLE_TXT)
