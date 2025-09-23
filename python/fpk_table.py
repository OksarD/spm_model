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
SAVE_TABLE = True
PLOT_TABLE = True
LOAD_TABLE = False
FILENAME_TABLE = "ypr_table.npy"
FILENAME_TABLE_TXT = "table_formatted.txt"

# SPM Parameters
a1 = pi/4; # alpha 1
a2 = pi/2; # alpha 2
b = pi/2; # beta

spm = manipulator.Coaxial_SPM(a1, a2, b)

# Sampling Params
PITCH_LIMIT = radians(45)
ROLL_LIMIT = radians(45)
YAW_LIMIT = 2*pi
SAMPLE_INTERVAL = radians(0.5)
YAW_ORIGIN = pi

# Table Params
TABLE_DIM = 180
INCLUSION_TOLERANCE = SAMPLE_INTERVAL*2
NAN_CODE = -999

ipk_samples = []
yaw_range = np.arange(YAW_LIMIT-2*pi, YAW_LIMIT, SAMPLE_INTERVAL)
pitch_range = np.arange(-PITCH_LIMIT, PITCH_LIMIT, SAMPLE_INTERVAL)
roll_range = np.arange(-PITCH_LIMIT, PITCH_LIMIT, SAMPLE_INTERVAL)

total_samples = len(yaw_range)*len(pitch_range)*len(roll_range)
samples_complete = 0

class sample_point:
    def __init__(self, _act, _ypr):
        self.act = _act
        self.ypr = _ypr
        self.act_dist = None

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
                        f_point.append(float(i))
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

# Main Functions
if LOAD_TABLE == False:
    print("Sample Interval:", SAMPLE_INTERVAL)
    print("Sampling Operating Region for near-zero M0 positions...   ")
    for y in yaw_range:
        for p in pitch_range:
            for r in roll_range:
                #print("ypr:", y,p,r)
                ypr = np.array([wrap_rad(y),wrap_rad(p),wrap_rad(r)])
                R_pyr = spm.R_ypr(ypr)
                actuator_pos = spm.solve_ipk(R_pyr, exception=False)
                if actuator_pos[0] != np.nan and (-INCLUSION_TOLERANCE <= actuator_pos[0] and actuator_pos[0] <= INCLUSION_TOLERANCE):
                    sample = sample_point(actuator_pos, [y,p,r])
                    ipk_samples.append(sample)
                samples_complete += 1
        print(f"\r{floor(100*samples_complete/total_samples):02}% ", end="", flush=True)
    print("\nIPK Samples:", len(ipk_samples))

    ypr_table = np.zeros((TABLE_DIM, TABLE_DIM, 3))
    act_lin = np.linspace(-pi, pi, TABLE_DIM, endpoint=False)

    total_entries = len(act_lin)**2
    entries_complete = 0

    print("Table Dimension:", TABLE_DIM)
    print("Table Size:", total_entries)
    print("Adding Yaw Entries for nearest actuator-domain point in sample pool...   ")
    for ind_i, i in enumerate(act_lin):
        for ind_j, j in enumerate(act_lin):
            # find nearest sample point in actuator domain
            closest_sample = None
            for s in ipk_samples:
                s.act_dist = np.linalg.norm(np.array([s.act[0], angle_subtract(s.act[1], i), angle_subtract(s.act[2], j)]))
                if closest_sample == None:
                    closest_sample = s
                closest_sample = min(closest_sample, s, key=lambda x:x.act_dist)
            # add corresponding ypr point to table
            if closest_sample.act_dist > INCLUSION_TOLERANCE:
                ypr_table[ind_i, ind_j, :] = [np.nan, np.nan, np.nan]
            else:
                ypr_table[ind_i, ind_j, :] = closest_sample.ypr
            entries_complete += 1
        print(f"\r{floor(100*entries_complete/total_entries):02}% ", end="", flush=True)
    print("\nDone.")
else:
    print("Loading Saved Table...")
    ypr_table = np.load(FILENAME_TABLE)
    act_lin = np.linspace(-pi, pi, TABLE_DIM, endpoint=False)

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
    ax.set_xlabel("M1 axis")
    ax.set_ylabel("M2 axis")
    ax.set_zlabel("Pitch value")
    # Roll
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(M1_AX, M2_AX, ypr_table[:,:,2], cmap="viridis")
    ax.set_xlabel("M1 axis")
    ax.set_ylabel("M2 axis")
    ax.set_zlabel("Roll value")
    plt.show()

if SAVE_TABLE:
    np.save(FILENAME_TABLE, ypr_table)
    print("Written table to", FILENAME_TABLE)

    # format only the yaw component of the table and save it as text for copying
    table_formatted = format_table(ypr_table[:,:,0])   
    timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    with open(FILENAME_TABLE_TXT, "w") as f:
        f.write(table_formatted) 
    print("Formatted as txt and written to", FILENAME_TABLE_TXT)
