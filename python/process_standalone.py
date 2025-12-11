import numpy as np
import pandas as pd
import os
from io import StringIO
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # required for 3D plots
np.set_printoptions(threshold=100)
from math import atan2, pi, radians
from generator import pathGenerator
from scipy.interpolate import interp1d
from scipy.signal import correlate
from scipy import stats
# Script options


# Vicon Config
DATA_FOLDER = "standalone_data"
FILE = "open_YPRT.csv"

# Path Generator Config
PATH_SAMPLE_FREQUENCY = 50
FILTER_FREQUENCY = 10
generator = pathGenerator(PATH_SAMPLE_FREQUENCY, FILTER_FREQUENCY)

line = FILE.split("_")[1].split(".csv")[0]
print("Test Function", line)

# convert csv into arrays of data
df = pd.read_csv(os.path.join(DATA_FOLDER, FILE)).dropna(axis=1, how="all")
ref = np.array([df[" ref_y"], df[" ref_p"], df[" ref_r"]]).T
results = np.array([df[" meas_y"], df[" meas_p"], df[" meas_r"]]).T
t = np.array(df["timestamp"]) / 1000000
error = results - ref

# plot reference and result in euler angles
plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(t, results, label=["y", "p", "r"])
#plt.plot(resampled_t, resampled_results)
plt.plot(t, ref, label=["y", "p", "r"], linestyle =':')
plt.legend(); plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Standalone Test " + FILE[:-4] + " in Euler angles (YPR)")

#plot error in euler angles
plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(t, error, label=["y", "p", "r"])
plt.legend(); plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Standalone Test Error " + FILE[:-4] + (" in Euler angles (YPR)"))

# calculate confifence interval
ci_upper = np.percentile(abs(error), 95, axis=0)
ci_lower = np.percentile(abs(error), 5, axis=0)

print("95% CI:", ci_upper)
print("5% CI:", ci_lower)

print("error max absolute (ypr)", np.max(abs(error), axis=0))
print("error average absolute (ypr)", np.mean(abs(error), axis=0))

plt.show()