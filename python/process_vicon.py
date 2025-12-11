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

# Script options


# Vicon Config
DATA_FOLDER = "vicon_data"
FILE = "closed_YPRT01.csv"
SKELETON_NAME = "Platform_Renamed"
VICON_SAMPLE_FREQUENCY = 100
DISCARD_POINTS = ["-X", "+X", "-Y"]

# Path Generator Config
PATH_SAMPLE_FREQUENCY = 50
FILTER_FREQUENCY = 10
generator = pathGenerator(PATH_SAMPLE_FREQUENCY, FILTER_FREQUENCY)

line = FILE.split("_")[1].split(".csv")[0]
line = ''.join(c for c in line if not c.isdigit())

print("Test Function", line)

# generate test trajectory
traj_y = []
traj_p = []
traj_r = []
test_duration = 20
if line == "YT": # yaw triangle
    # period of 4 so that it spends 2 seconds per stride, therefore travels at the speed of the amplitude
    traj_y = generator.generate_triangle_trajectory(radians(45),4,test_duration, filter=True)
    traj_p = generator.generate_zero_trajectory(test_duration)
    traj_r = generator.generate_zero_trajectory(test_duration)
if line == "YPT": # yaw-pitch synchronous triangle
    # period of 4 so that it spends 2 seconds per stride, therefore travels at the speed of the amplitude
    traj_y = generator.generate_triangle_trajectory(radians(45),4,test_duration, filter=True)
    traj_p = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
    traj_r = generator.generate_zero_trajectory(test_duration)
if line == "YRT": # yaw-roll synchronous triangle
    traj_y = generator.generate_triangle_trajectory(radians(45),4,test_duration, filter=True)
    traj_p = generator.generate_zero_trajectory(test_duration)
    traj_r = generator.generate_triangle_trajectory(radians(-30),4,test_duration, filter=True)
if line == "PRT": # pitch_roll synchronous triangle
    traj_y = generator.generate_zero_trajectory(test_duration)
    traj_p = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
    traj_r = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
if line == "YPRT": # yaw-pitch-roll synchronous triangle
    traj_y = generator.generate_triangle_trajectory(radians(45),4,test_duration, filter=True)
    traj_p = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
    traj_r = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
if line == "YS": # yaw sinusoid
    traj_y = generator.generate_sin_trajectory(radians(45),4,test_duration, filter=True)
    traj_p = generator.generate_zero_trajectory(test_duration)
    traj_r = generator.generate_zero_trajectory(test_duration)
if line == "YPS": # yaw-pitch off-sync sinusoid
    traj_y = generator.generate_sin_trajectory(radians(45),4,test_duration, filter=True)
    traj_p = generator.generate_cos_trajectory(radians(30),4,test_duration, filter=True)
    traj_r = generator.generate_zero_trajectory(test_duration)
if line == "YRS": # yaw-roll off-sync sinusoid
    traj_y = generator.generate_sin_trajectory(radians(45),4,test_duration, filter=True)
    traj_p = generator.generate_zero_trajectory(test_duration)
    traj_r = generator.generate_cos_trajectory(radians(30),4,test_duration, filter=True)
if line == "PRS": # yaw-pitch off-sync sinusoid
    traj_y = generator.generate_zero_trajectory(test_duration)
    traj_p = generator.generate_sin_trajectory(radians(30),4,test_duration, filter=True)
    traj_r = generator.generate_cos_trajectory(radians(30),4,test_duration, filter=True)
if line == "YPRS": # yaw-pitch off-sync sinusoid
    traj_y = generator.generate_sin_trajectory(radians(45),4,test_duration, filter=True)
    traj_p = generator.generate_sin_trajectory(radians(30),4,test_duration, filter=True)
    traj_r = generator.generate_cos_trajectory(radians(30),4,test_duration, filter=True)

# rotation object to ypr array
def r_to_ypr(q: R) -> np.ndarray:
    # Ensure quaternion is normalized
    qn = q.as_quat()  # returns [x, y, z, w]
    x, y, z, w = qn
    # Yaw (around z)
    yaw = np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    # Pitch (around y)
    sinp = 2 * (w*y - z*x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    # Roll (around x)
    roll = np.arctan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    return np.array([yaw, pitch, roll])

# plot the vicon ball pointcloud for debugging purposes
def plot_pointcloud(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o')
    # Set equal aspect ratio for data
    xlim = [np.min(points[:, 0]), np.max(points[:, 0])]
    ylim = [np.min(points[:, 1]), np.max(points[:, 1])]
    zlim = [np.min(points[:, 2]), np.max(points[:, 2])]
    # Find overall range
    max_range = np.ptp([*xlim, *ylim]) / 2.0
    mid_x = np.mean(xlim)
    mid_y = np.mean(ylim)
    mid_z = np.mean(zlim)
    # Apply equal limits
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    ax.set_box_aspect((1, 1, 1))  # Keep cube box shape
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show(block=False)

# Main Script

# process vicon csv
with open(os.path.join(DATA_FOLDER, FILE), "r") as f:
    df_lines = f.readlines()
# Extract dataframe
marker_names = df_lines[2].strip("\n").strip(",").split(",,,")
marker_names = [n.split(SKELETON_NAME + ":")[1] for n in marker_names]
csv_text = "".join(df_lines[4:])
df = pd.read_csv(StringIO(csv_text))
# Duplicate column names for x,y,z
marker_names_3 = ["F", "S"]
for n in marker_names:
    for _ in range(3):
        marker_names_3.append(n)
df.columns = marker_names_3
# remove discarded points
df = df.drop(columns=DISCARD_POINTS)
marker_names = [x for x in marker_names if x not in DISCARD_POINTS]
#print(marker_names)
# drop frame and sample columns
xyz_cols = ["F", "S"] + [f"{c}" for _ in marker_names for c in ["X", "Y", "Z"]]
df.columns = xyz_cols
df = df.drop(columns=["F", "S"])
# convert data to numpy array
data = df.to_numpy()
data_3d = data.reshape(data.shape[0], int(data.shape[1]/3), 3)
# rotate data around z axis to align with vicon frame using the center and +Y point
y_point = data_3d[0][marker_names.index("+Y")]
c_point = data_3d[0][marker_names.index("Center")]
origin_angle = atan2(y_point[1] - c_point[1], y_point[0] - c_point[0]) - pi/2
z_axis = np.array([0,0,1])
origin_r = R.from_rotvec(-origin_angle * z_axis)
data_3d = origin_r.apply(data_3d.reshape(-1, 3)).reshape(data_3d.shape)
# take first element of data as home position
p = data_3d[0]
#plot_pointcloud(p)
#print(data_3d)

# caluclate angle of all pointclouds relative to the first one, generating the path
results_traj = []
results_t = np.arange(0, data.shape[0]/VICON_SAMPLE_FREQUENCY, 1/VICON_SAMPLE_FREQUENCY)
for ind, q in enumerate(data_3d):
    valid_mask = ~np.isnan(p).any(axis=1) & ~np.isnan(q).any(axis=1)
    if valid_mask.sum() < 3:
        # Not enough points to compute rotation
        raise ValueError("Not enough valid points to compute transform")
    p_valid = p[valid_mask]
    q_valid = q[valid_mask]
    # Compute centroids
    p_centroid = p_valid.mean(axis=0)
    q_centroid = q_valid.mean(axis=0)
    # Center points
    P_centered = p_valid - p_centroid
    Q_centered = q_valid - q_centroid
    # Covariance
    H = Q_centered.T @ P_centered
    # SVD
    U, S, Vt = np.linalg.svd(H)
    R_mat = Vt.T @ U.T
    # Rotation object
    rot = R.from_matrix(R_mat).inv()
    #quat = rot.as_quat()
    ypr = r_to_ypr(rot)
    results_traj.append(ypr)
    print(f"\r{100*ind/data_3d.shape[0]:.2f}% ", end="", flush=True)

# shift the vicondata in time so it is in line with the reference data
ref_t = np.array(traj_y._t)
ref_traj = np.array([[traj_y._func[i], traj_p._func[i], traj_r._func[i]] for i in range(len(traj_y._t))])
results_traj = np.array(results_traj)
time_shift_ypr = []
time_shift_average = 0
resampled_ref = []
resampled_results = []
resampled_t = None
shifted_t = None

for i in range(3):
    # Inputs: t1, y1, t2, y2 (1D numpy arrays, irregular times)
    t_min = max(np.min(ref_t), np.min(results_t))
    t_max = min(np.max(ref_t), np.max(results_t))
    t_min_full = min(np.min(ref_t), np.min(results_t))
    t_max_full = max(np.max(ref_t), np.max(results_t))
    # Build a fine step based on data
    dt_est = np.median(np.diff(np.unique(np.sort(np.r_[ref_t, results_t]))))
    dt = dt_est / 2.0  # finer than typical spacing
    tg = np.arange(t_min, t_max, dt)  # common time grid
    tg_full = np.arange(t_min_full, t_max_full, dt)
    # Interpolate both signals onto tg (linear is robust; cubic if smoother data)
    f_ref = interp1d(ref_t, ref_traj[:,i], kind='linear', bounds_error=False, fill_value=np.nan)
    f_res = interp1d(results_t, results_traj[:,i], kind='linear', bounds_error=False, fill_value=np.nan)
    y_ref_g = f_ref(tg)
    y_res_g = f_res(tg)
    y_ref_g_full = f_ref(tg_full)
    y_res_g_full = f_res(tg_full)
    # compute cross corelation
    corr = correlate(y_ref_g, y_res_g, mode='full')                     # cross-correlation
    lags_samples = np.arange(-len(y_res_g) + 1, len(y_ref_g))        # sample lags
    k = np.argmax(corr)                                         # peak index
    lag_samples = lags_samples[k]                            # best lag (in samples)
    time_shift = lag_samples * (tg[1] - tg[0])               # seconds
    time_shift_ypr.append(time_shift)
    # store resampled functions
    resampled_ref.append(y_ref_g_full)
    resampled_results.append(y_res_g_full)
    resampled_t = tg_full
    shifted_t = tg

# apply time shift to original datasets for plotting
print("Time shift ypr (s)", time_shift_ypr)
shifted_results = []
error = []
for i in range(3):
    f_interp = interp1d(resampled_t, resampled_results[i], fill_value="extrapolate")
    shift_res = f_interp(resampled_t - time_shift_ypr[0])
    resampled_ref[i] = resampled_ref[i][:len(shifted_t)]
    shift_res = shift_res[:len(shifted_t)]
    shifted_results.append(shift_res)
    error.append(shifted_results[i] - resampled_ref[i])

# transform array for easy plotting
shifted_results = np.array(shifted_results).T
resampled_ref = np.array(resampled_ref).T
resampled_results = np.array(resampled_results).T
error = np.array(error).T

# plot reference and result in euler angles
plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(shifted_t, shifted_results, label=["y", "p", "r"])
#plt.plot(resampled_t, resampled_results)
plt.plot(shifted_t, resampled_ref, label=["y", "p", "r"], linestyle =':')
plt.legend(); plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Vicon Capture Test " + FILE[:-4] + " in Euler angles (YPR)")

#plot error in euler angles
plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(shifted_t, error, label=["y", "p", "r"])
plt.legend(); plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Vicon Capture Test Error " + FILE[:-4] + (" in Euler angles (YPR)"))

# calculate confifence interval
ci_upper = np.percentile(abs(error), 95, axis=0)
ci_lower = np.percentile(abs(error), 5, axis=0)

print("95% CI:", ci_upper)
print("5% CI:", ci_lower)

print("error max absolute (ypr)", np.max(abs(error), axis=0))
print("error average absolute (ypr)", np.mean(abs(error), axis=0))

plt.show()