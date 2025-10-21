import numpy as np
from numpy import cos, sin

# --- Helper functions ---
def clamp(x, min_val, max_val):
    return max(min_val, min(max_val, x))

# --- 1. Euler (yaw, pitch, roll) → Quaternion ---
_q_prev = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z] global memory like static var

def ypr_to_q(ypr):
    global _q_prev
    y, p, r = ypr  # yaw, pitch, roll

    sin_y_2, cos_y_2 = np.sin(y * 0.5), np.cos(y * 0.5)
    sin_p_2, cos_p_2 = np.sin(p * 0.5), np.cos(p * 0.5)
    sin_r_2, cos_r_2 = np.sin(r * 0.5), np.cos(r * 0.5)

    w = cos_r_2 * cos_p_2 * cos_y_2 + sin_r_2 * sin_p_2 * sin_y_2
    x = sin_r_2 * cos_p_2 * cos_y_2 - cos_r_2 * sin_p_2 * sin_y_2
    y = cos_r_2 * sin_p_2 * cos_y_2 + sin_r_2 * cos_p_2 * sin_y_2
    z = cos_r_2 * cos_p_2 * sin_y_2 - sin_r_2 * sin_p_2 * cos_y_2

    q = np.array([w, x, y, z])
    if np.dot(q, _q_prev) < 0.0:
        q *= -1.0
    _q_prev = q
    return q / np.linalg.norm(q)


# --- 2. Quaternion → Euler (yaw, pitch, roll) ---
def q_to_ypr(q):
    q = q / np.linalg.norm(q)
    w, x, y, z = q

    yaw = np.arctan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    sinp = 2.0 * (w*y - z*x)
    sinp = clamp(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    roll = np.arctan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))

    return np.array([yaw, pitch, roll])


# --- 3. Quaternion → Axis-Angle [a, x, y, z] ---
def q_to_aa(q):
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    angle = 2.0 * np.arccos(clamp(w, -1.0, 1.0))
    s = np.sqrt(1.0 - w * w)
    if s < 1e-8:
        axis = np.array([1.0, 0.0, 0.0])
    else:
        axis = np.array([x, y, z]) / s
    return np.concatenate(([angle], axis))


# --- 4. Axis-Angle [a, x, y, z] → Quaternion ---
def aa_to_q(aa):
    angle = aa[0]
    axis = aa[1:4]
    if np.linalg.norm(axis) < 1e-6:
        axis = np.array([1.0, 0.0, 0.0])
    else:
        axis = axis / np.linalg.norm(axis)
    half_angle = 0.5 * angle
    s = np.sin(half_angle)
    c = np.cos(half_angle)
    return np.array([c, axis[0] * s, axis[1] * s, axis[2] * s])


# --- 5. Axis-Angle [a, x, y, z] → Rotation Matrix ---
def aa_to_R(aa):
    angle = aa[0]
    axis = aa[1:4]
    if np.linalg.norm(axis) < 1e-6:
        return np.eye(3)
    axis = axis / np.linalg.norm(axis)
    c = np.cos(angle)
    s = np.sin(angle)
    one_c = 1.0 - c

    x, y, z = axis
    R = np.array([
        [c + x*x*one_c,     x*y*one_c - z*s, x*z*one_c + y*s],
        [y*x*one_c + z*s,   c + y*y*one_c,   y*z*one_c - x*s],
        [z*x*one_c - y*s,   z*y*one_c + x*s, c + z*z*one_c]
    ])
    return R


# --- 6. Axis-Angle [a, x, y, z] → Rotation Vector (scaled axis) ---
def aa_to_xyz(aa):
    a, x, y, z = aa
    return np.array([a * x, a * y, a * z])

# get rotation matrix of angle around the x axis
def R_x(angle):
    return np.array([[1, 0, 0],
                    [0, cos(angle), -sin(angle)],
                    [0, sin(angle), cos(angle)]])

# get rotation matrix of angle around the y axis
def R_y(angle):
    return np.array([[cos(angle), 0, sin(angle)],
                    [0, 1, 0],
                    [-sin(angle), 0, cos(angle)]])

# get rotation matrix of angle around the z axis
def R_z(angle):
    return np.array([[cos(angle), -sin(angle), 0],
                    [sin(angle), cos(angle), 0],
                    [0, 0, 1]])

def ypr_to_xyz_velocity(ypr_velocity, ypr_point):
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