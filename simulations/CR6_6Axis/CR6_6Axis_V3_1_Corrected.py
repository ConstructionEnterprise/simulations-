"""
CHAPPELL ROBOTICS
CR6 Six-Axis Robot — Version 3.1 (Corrected Pick Logic)

VERSION : 6Axis_V3.1_Pick_Validated
PURPOSE : Fixes the DH-to-IK mismatch, corrects tool orientation, 
          and validates the static part pick/place handshake.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from collections import deque

# ── DH Parameters (Pristine V3 Geometry) ──────────────────────────────────
D1 = 1.5
A2 = 2.5
A3 = 2.0
D6 = 0.5
MAX_REACH = A2 + A3 + D6  # Total reach capability

# ── Conveyor Layout ───────────────────────────────────────────────────────
CONV_X_START  = -4.0   
CONV_X_END    =  4.0   
CONV_Y_CENTER = -2.8   # Pushed out to allow links space
CONV_WIDTH    =  1.0   
CONV_Z        =  1.0   
CONV_LEG_Z    =  0.0   

PICK_ZONE_X   =  1.0   

# ── Part Geometry ─────────────────────────────────────────────────────────
PART_HEIGHT   =  0.2   
PART_X_STATIC =  PICK_ZONE_X
PART_Y_STATIC =  CONV_Y_CENTER
PART_Z_STATIC =  CONV_Z + (PART_HEIGHT / 2.0)  # Core center sits at 1.10

# Dynamic State Registers
part_position = [PART_X_STATIC, PART_Y_STATIC, PART_Z_STATIC]
is_attached   = False
dwell_frames  = 0
DWELL_LIMIT   = 15  

# ── DH Transform Matrix Generator ─────────────────────────────────────────
def dh_transform(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,  -st*ca,  st*sa,  a*ct],
        [st,   ct*ca, -ct*sa,  a*st],
        [0,    sa,     ca,     d   ],
        [0,    0,      0,      1   ]
    ])

# ── Rotation Matrix Utilities ─────────────────────────────────────────────
def rpy_to_matrix(rpy):
    r, p, y = rpy
    Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

def mat_to_quat(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])

def quat_to_mat(q):
    norm = np.linalg.norm(q)
    if norm < 1e-6: return np.eye(3)
    w, x, y, z = q / norm
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z),   2*(y*z-x*w)],
        [2*(x*z-y*w),     2*(y*z+x*w), 1-2*(x*x+y*y)]
    ])

def slerp(q0, q1, t):
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = np.clip(np.dot(q0, q1), -1.0, 1.0)
    if dot < 0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return q0 + t * (q1 - q0)
    theta0 = np.arccos(dot)
    theta  = theta0 * t
    sin0   = np.sin(theta0)
    return (np.sin(theta0 - theta) * q0 + np.sin(theta) * q1) / sin0

# ── Analytical Forward Kinematics ──────────────────────────────────────────
def forward_kinematics(q):
    q1, q2, q3, q4, q5, q6 = q
    # CORRECTED DH TABLE: Joints 2 and 3 are now parallel (Shoulder and Elbow)
    dh = [
        [0,    np.pi/2,  D1,  q1], # J1: Rotation about Z
        [A2,   0,        0,   q2], # J2: Shoulder
        [A3,   0,        0,   q3], # J3: Elbow
        [0,   -np.pi/2,  0,   q4], # J4: Wrist Rotate
        [0,    np.pi/2,  0,   q5], # J5: Wrist Tilt
        [0,    0,        D6,  q6], # J6: Tool Plate
    ]
    T = np.eye(4)
    positions = [T[:3, 3].copy()]
    for row in dh:
        T = T @ dh_transform(*row)
        positions.append(T[:3, 3].copy())
    return positions

# ── Analytical Inverse Kinematics (Validated 6-Axis) ───────────────────────
def inverse_kinematics(target_pos, R06):
    px, py, pz = target_pos
    approach   = R06[:, 2]

    # Calculate Wrist Center (WC)
    wx = px - D6 * approach[0]
    wy = py - D6 * approach[1]
    wz = pz - D6 * approach[2]

    # J1: Base rotation
    q1 = np.arctan2(wy, wx)
    
    # J2, J3: 2-Link planar solution
    r     = np.hypot(wx, wy)
    s     = wz - D1
    dist2 = r*r + s*s
    dist  = np.sqrt(dist2)

    if dist > (A2 + A3) * 0.99 or dist < np.abs(A2 - A3):
        return None

    cos3 = (dist2 - A2**2 - A3**2) / (2 * A2 * A3)
    cos3 = np.clip(cos3, -1.0, 1.0)
    
    # Elbow-down configuration (standard for this reach)
    q3   = np.arctan2(-np.sqrt(1 - cos3**2), cos3)
    q2   = np.arctan2(s, r) - np.arctan2(A3 * np.sin(q3), A2 + A3 * np.cos(q3))

    # Wrist Solution (J4, J5, J6)
    T1  = dh_transform(0,    np.pi/2,  D1,  q1)
    T2  = dh_transform(A2,   0,        0,   q2)
    T3  = dh_transform(A3,   0,        0,   q3)
    R03 = (T1 @ T2 @ T3)[:3, :3]
    R36 = R03.T @ R06

    # Euler ZYZ extraction for the wrist
    q5 = np.arctan2(np.sqrt(R36[0,2]**2 + R36[1,2]**2), R36[2,2])
    if np.abs(np.sin(q5)) > 1e-6:
        q4 = np.arctan2( R36[1,2],  R36[0,2])
        q6 = np.arctan2( R36[2,1], -R36[2,0])
    else:
        q4 = 0.0
        q6 = np.arctan2(-R36[0,1], R36[1,1])

    return np.array([q1, q2, q3, q4, q5, q6])

# ── Scene Drawing Layer ────────────────────────────────────────────────────
def draw_conveyor(ax):
    y0 = CONV_Y_CENTER - CONV_WIDTH / 2
    y1 = CONV_Y_CENTER + CONV_WIDTH / 2
    x0 = CONV_X_START
    x1 = CONV_X_END
    z  = CONV_Z

    belt_x = np.array([[x0, x1], [x0, x1]])
    belt_y = np.array([[y0, y0], [y1, y1]])
    belt_z = np.array([[z,  z ], [z,  z ]])
    ax.plot_surface(belt_x, belt_y, belt_z, color="steelblue", alpha=0.35, zorder=1)

    for y_rail in [y0, y1]:
        ax.plot([x0, x1], [y_rail, y_rail], [z, z], color="dimgray", linewidth=3, zorder=2)
    for x_end in [x0, x1]:
        ax.plot([x_end, x_end], [y0, y1], [z, z], color="dimgray", linewidth=3, zorder=2)
    for lx in [x0 + 0.5, x1 - 0.5]:
        for ly in [y0, y1]:
            ax.plot([lx, lx], [ly, ly], [CONV_LEG_Z, z], color="dimgray", linewidth=2, zorder=2)

    ax.scatter(PICK_ZONE_X, CONV_Y_CENTER, z + 0.02, color="yellow", s=120, marker="D", edgecolors="orange", linewidths=1.5, zorder=5)

# ── Waypoint Configuration (Corrected Tool Face) ──────────────────────────
# Tool points straight down at the belt (-Z direction)
DOWN_ORIENTATION = np.array([0.0, np.pi, 0.0])  

PICK_APPROACH_Z = CONV_Z + 1.2   
PICK_TARGET_POS = np.array([PART_X_STATIC, PART_Y_STATIC, PART_Z_STATIC])

WAYPOINTS = {
    "HOME":           (np.array([ 1.5,  0.5,   3.5]),  DOWN_ORIENTATION.copy()),
    "PICK_APPROACH":  (np.array([ PICK_ZONE_X, CONV_Y_CENTER, PICK_APPROACH_Z]), DOWN_ORIENTATION.copy()),
    "PICK":           (PICK_TARGET_POS.copy(),                                   DOWN_ORIENTATION.copy()), 
    "LIFT":           (np.array([ PICK_ZONE_X, CONV_Y_CENTER, PICK_APPROACH_Z]), DOWN_ORIENTATION.copy()),
    "PLACE":          (np.array([-1.5,  1.5,   1.5]),   DOWN_ORIENTATION.copy()), 
}

STATE_SEQUENCE = ["HOME", "PICK_APPROACH", "PICK", "LIFT", "PLACE", "HOME"]

# Linear Interpolation State Engine
seg_idx   = 0
t_seg     = 0.0

def get_waypoint_pose(name):
    pos, rpy = WAYPOINTS[name]
    return pos.copy(), rpy_to_matrix(rpy)

start_pos, start_R = get_waypoint_pose(STATE_SEQUENCE[0])
end_pos,   end_R   = get_waypoint_pose(STATE_SEQUENCE[1])
start_q_quat = mat_to_quat(start_R)
end_q_quat   = mat_to_quat(end_R)

current_q = np.array([0.0, 0.2, -0.5, 0.0, 0.0, 0.0])
trace = deque(maxlen=600)

# ── Window Rendering Setup ─────────────────────────────────────────────────
fig = plt.figure(figsize=(12, 9))
ax  = fig.add_subplot(111, projection="3d")
plt.subplots_adjust(bottom=0.12)

slider_ax    = plt.axes([0.2, 0.03, 0.6, 0.03])
speed_slider = Slider(slider_ax, "Motion Speed", 0.002, 0.04, valinit=0.012)

def update(frame):
    global seg_idx, t_seg, current_q
    global start_pos, end_pos, start_q_quat, end_q_quat, start_R, end_R
    global part_position, is_attached, dwell_frames

    ax.clear()
    speed = speed_slider.val
    current_state = STATE_SEQUENCE[seg_idx]

    # Timeline State Transitions
    t_seg += speed
    if t_seg >= 1.0:
        if current_state in ["PICK", "PLACE"] and dwell_frames < DWELL_LIMIT:
            dwell_frames += 1
            t_seg = 1.0  
        else:
            dwell_frames = 0
            t_seg   = 0.0
            seg_idx = (seg_idx + 1) % (len(STATE_SEQUENCE) - 1)

            start_pos, start_R = get_waypoint_pose(STATE_SEQUENCE[seg_idx])
            end_pos,   end_R   = get_waypoint_pose(STATE_SEQUENCE[seg_idx + 1])
            start_q_quat = mat_to_quat(start_R)
            end_q_quat   = mat_to_quat(end_R)

    current_state = STATE_SEQUENCE[seg_idx]
    
    # Core Spatial Path Generation
    t     = t_seg
    pos_t = start_pos + t * (end_pos - start_pos)
    q_t   = slerp(start_q_quat, end_q_quat, t)
    R_t   = quat_to_mat(q_t)

    # Solve Joint Parameters
    solved = inverse_kinematics(pos_t, R_t)
    if solved is not None:
        current_q = solved

    pts = forward_kinematics(current_q)
    tcp = pts[-1]

    # VALIDATED PICK LOGIC
    if current_state == "PICK":
        # Check proximity to part center
        dist = np.linalg.norm(np.array(tcp) - np.array(part_position))
        if dist < 0.25:  
            is_attached = True
    elif current_state == "PLACE":
        is_attached = False
        # Part stays at the place location
        part_position = [WAYPOINTS["PLACE"][0][0], WAYPOINTS["PLACE"][0][1], WAYPOINTS["PLACE"][0][2]]
    elif current_state == "HOME" and not is_attached:
        # Reset part to conveyor for next cycle
        part_position = [PART_X_STATIC, PART_Y_STATIC, PART_Z_STATIC]

    if is_attached:
        part_position = list(tcp)

    # Render Visual Layer
    draw_conveyor(ax)
    ax.scatter(part_position[0], part_position[1], part_position[2], color="darkorange", s=140, marker="s", edgecolors="black", zorder=6)

    colors  = ["#FF3333", "#FF8800", "#FFD700", "#44DD44", "#00CCFF", "#4466FF"]
    lwidths = [8, 7, 7, 5, 5, 4]
    for i in range(len(pts) - 1):
        ax.plot([pts[i][0], pts[i+1][0]], [pts[i][1], pts[i+1][1]], [pts[i][2], pts[i+1][2]], color=colors[i], linewidth=lwidths[i], solid_capstyle="round")

    for pt in pts:
        ax.scatter(*pt, color="white", s=40, zorder=5, edgecolors="gray")
    ax.scatter(*tcp, color="magenta", s=130, marker="*", zorder=7)

    trace.append(tcp.copy())
    if len(trace) > 2:
        tr = np.array(trace)
        ax.plot(tr[:,0], tr[:,1], tr[:,2], color="purple", linewidth=1.2, alpha=0.4)

    # Telemetry HUD
    q_deg = np.degrees(current_q)
    hud = (
        f"STATE : {current_state}\n"
        f"T     : {t:.2f}\n"
        f"DWELL : {dwell_frames}/{DWELL_LIMIT}\n"
        f"---------------------\n"
        f"J1: {q_deg[0]:+7.1f}°\n"
        f"J2: {q_deg[1]:+7.1f}°\n"
        f"J3: {q_deg[2]:+7.1f}°\n"
        f"J4: {q_deg[3]:+7.1f}°\n"
        f"J5: {q_deg[4]:+7.1f}°\n"
        f"J6: {q_deg[5]:+7.1f}°\n"
        f"---------------------\n"
        f"TCP : [{tcp[0]:+.2f},{tcp[1]:+.2f},{tcp[2]:+.2f}]\n"
        f"PART: [{part_position[0]:+.2f},{part_position[1]:+.2f},{part_position[2]:+.2f}]\n"
        f"ATTACHED: {is_attached}\n"
    )
    ax.text2D(0.02, 0.97, hud, transform=ax.transAxes, fontsize=8, family="monospace", verticalalignment="top")

    ax.set_title("CHAPPELL ROBOTICS  —  CR6  V3.1  |  PICK VALIDATION", fontsize=10, fontweight="bold")
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(0, 6)
    ax.grid(True, alpha=0.3)

ani = FuncAnimation(fig, update, interval=40)
plt.show()
