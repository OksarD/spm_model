import numpy as np
from numpy import pi, cos, sin, sqrt, acos, atan2, degrees, radians
from math import floor
import matplotlib.pyplot as plt
import manipulator
from utils import *
from datetime import datetime
from scipy.interpolate import RegularGridInterpolator
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage import median_filter
from scipy.ndimage import label, binary_fill_holes
import os
from scipy.interpolate import interp2d

np.set_printoptions(formatter={'float': '{:.3f}'.format})

# config
SAVE_TABLE = False
PLOT_TABLE = False
LOAD_TABLE = True
POST_PROCESS_TABLE = True

DATA_FOLDER = "fpk_table_data"
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
TABLE_DIM = 180
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

def remove_local_outliers(data, window=3, threshold=2):
    data = data.copy()
    local_med = median_filter(data, size=window, mode='reflect')
    abs_dev = np.abs(data - local_med)
    mad = median_filter(abs_dev, size=window, mode='reflect')

    mask = abs_dev > threshold * mad
    data[mask] = local_med[mask]
    return data

def keep_center_region(surface, jump_threshold=0.2, connectivity=4, fill_holes=False):
    """
    Keeps only the continuous region around the center of the surface,
    where continuity is defined by both:
      - no NaNs
      - no value jumps above jump_threshold between adjacent cells.
    
    Parameters
    ----------
    surface : np.ndarray
        2D array (surface map)
    jump_threshold : float
        Maximum allowed difference between adjacent cells to be considered connected.
    connectivity : int
        4 or 8 — defines how diagonals are treated as connected.
    fill_holes : bool
        Fill small NaN holes inside the main region.
    """
    data = surface.copy()
    valid = ~np.isnan(data)

    # Step 1: Build connectivity mask that breaks where jumps exceed threshold
    mask = valid.copy()

    # Compare with 4 or 8 neighbours
    dy = [0, 1, -1, 0]
    dx = [1, 0, 0, -1]
    if connectivity == 8:
        dy += [1, 1, -1, -1]
        dx += [1, -1, 1, -1]

    for y_shift, x_shift in zip(dy, dx):
        shifted = np.roll(data, shift=(y_shift, x_shift), axis=(0,1))
        diff = np.abs(data - shifted)
        broken = diff > jump_threshold
        broken |= np.isnan(diff)
        # mark both sides of large jumps as invalid connections
        mask[broken & valid] = False

    # Step 2: Label connected regions of remaining mask
    structure = np.ones((3,3)) if connectivity == 8 else np.array([[0,1,0],[1,1,1],[0,1,0]])
    labeled, n = label(mask, structure=structure)

    if n == 0:
        return data

    # Step 3: Keep the patch that includes the center pixel
    center = (data.shape[0]//2, data.shape[1]//2)
    center_label = labeled[center]

    if center_label == 0:
        # Center might be NaN or isolated; pick nearest valid region
        ys, xs = np.nonzero(labeled)
        if len(ys) == 0:
            return data
        dist = (ys-center[0])**2 + (xs-center[1])**2
        nearest = np.argmin(dist)
        center_label = labeled[ys[nearest], xs[nearest]]

    keep_mask = labeled == center_label

    # Step 4: Optionally fill internal NaN holes
    if fill_holes:
        keep_mask = binary_fill_holes(keep_mask)

    # Step 5: Mask out everything else
    data[~keep_mask] = np.nan
    return data

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
    ypr_table = np.load(os.path.join(DATA_FOLDER, FILENAME_TABLE))

if POST_PROCESS_TABLE:
    yaw_table = keep_center_region(remove_local_outliers(ypr_table[:,:,0]), True)
    pitch_table = keep_center_region(remove_local_outliers(ypr_table[:,:,1]), True)
    roll_table = keep_center_region(remove_local_outliers(ypr_table[:,:,2]), True)
else:
    yaw_table = ypr_table[:,:,0]
    pitch_table = ypr_table[:,:,1]
    roll_table = ypr_table[:,:,2]



y = RegularGridInterpolator((act_lin, act_lin), yaw_table)
p = RegularGridInterpolator((act_lin, act_lin), pitch_table)
r = RegularGridInterpolator((act_lin, act_lin), roll_table)

# test table
pt = [0.7, -0.4]
pt[0] = pt[0]+spm.actuator_origin
pt[1] = pt[1]+spm.actuator_origin
ypr = [y(pt), p(pt), r(pt)]
ypr_real = spm.solve_fpk([wrap_rad(spm.actuator_origin), wrap_rad(pt[0]), wrap_rad(pt[1])])
print("pt m1_m2", pt)
print("ypr fpk table: ", ypr)
print("ypr fpk real:", ypr_real)

# test table by index
i = 40
j = 90
ypr_ind = [yaw_table[i,j], pitch_table[i,j], roll_table[i,j]]
print("ij: ", i, j )
print("ypr fpk table indexed: ", ypr_ind)


if PLOT_TABLE:
    # Yaw
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    M1_AX, M2_AX = np.meshgrid(act_lin, act_lin, indexing="ij")
    ax.plot_surface(M1_AX, M2_AX, yaw_table, cmap="viridis")
    ax.set_xlabel("M1 axis")
    ax.set_ylabel("M2 axis")
    ax.set_zlabel("Yaw value")
    # Pitch
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(M1_AX, M2_AX, pitch_table, cmap="viridis")
    ax.plot_surface(M1_AX, M2_AX, roll_table, cmap="plasma")
    ax.set_xlabel("M1 axis")
    ax.set_ylabel("M2 axis")
    ax.set_zlabel("Pitch/Roll value")
    plt.show()

if SAVE_TABLE:
    if not LOAD_TABLE:
        np.save(os.path.join(DATA_FOLDER, FILENAME_TABLE), ypr_table)
        print("Written table to", FILENAME_TABLE)
    # format only the yaw component of the table and save it as text for copying
    yaw_table_formatted = format_table(yaw_table)  
    pitch_table_formatted = format_table(pitch_table) 
    roll_table_formatted = format_table(roll_table)

    with open(DATA_FOLDER + "/yaw_" + FILENAME_TABLE_TXT, "w") as f:
        f.write(yaw_table_formatted)
    with open(DATA_FOLDER + "/pitch_" + FILENAME_TABLE_TXT, "w") as f:
        f.write(pitch_table_formatted) 
    with open(DATA_FOLDER + "/roll_" + FILENAME_TABLE_TXT, "w") as f:
        f.write(roll_table_formatted) 
    print("Formatted as txt and written to yaw/pitch/roll_" + FILENAME_TABLE_TXT)
