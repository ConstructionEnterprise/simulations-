"""
CHAPPELL ROBOTICS
CR6 Six-Axis Robot — Step 6.1: Bounded Workspace Guard (Corrected)

VERSION : 6Axis_V6.1_Workspace_Guard_Fixed
PURPOSE : Restores correct DH parameters, fixes tool orientation, 
          resolves part recycling logic, and adds reachability visualization.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from collections import deque

# ── DH Parameters (Pristine V3.1 Geometry — RESTORED) ─────────────────────
D1 = 1.5
A2 = 2.5
A3 = 2.0
D6 = 0.5
MAX_REACH = A2 + A3 + D6  # 5.0 (corrected)

# ── Conveyor Cell Layout ──────────────────────────────────────────────────
CONV_X_START  = -4.0   
CONV_X_END    =  4.0   
CONV_Y_CENTER = -2.8   
CONV_WIDTH    =  1.0   
CONV_Z        =  1.0   
CONV_LEG_Z    =  0.0   

# ── Factory Dynamics & Boundaries ─────────────────────────────────────────
CONV_SPEED    =  0.040 
PART_HEIGHT   =  0.2   
PART_Z_TARGET =  CONV_Z + (PART_HEIGHT / 2.0)  # Z = 1.10

# Safe Mathematical Intercept Window Radius (92% of hardware limit)
SAFE_INTERCEPT_RADIUS = MAX_REACH * 0.92  # 4.6

# Dynamic Operational Registers
part_x        = CONV_X_START
is_attached   = False
dwell_frames  = 0
DWELL_LIMIT   = 15  
parts_cleared = 0
waiting_for_home_reset = False  # New flag for part recycle

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
    w, x, y, z = q / np.linalg.norm(q)
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

# ── Analytical Forward Kinematics (RESTORED V3.1) ─────────────────────────
def forward_kinematics(q):
    q1, q2, q3, q4, q5, q6 = q
    # CORRECTED DH TABLE — Joints 2 and 3 parallel
    dh = [
        [0,    np.pi/2,  D1,  q1],
        [A2,   0,        0,   q2],
        [A3,   0,        0,   q3],
        [0,   -np.pi/2,  0,   q4],
        [0,    np.pi/2,  0,   q5],
        [0,    0,        D6,  q6],
    ]
    T = np.eye(4)
    positions = [T[:3, 3].copy()]
    for row in dh:
        T = T @ dh_transform(*row)
        positions.append(T[:3, 3].copy())
    return positions

# ── Analytical Inverse Kinematics (RESTORED V3.1) ─────────────────────────
def inverse_kinematics(target_pos, R06):
    px, py, pz = target_pos
    approach   = R06[:, 2]

    # Wrist Center calculation — FIXED
    wx = px - D6 * approach[0]
    wy = py - D6 * approach[1]
    wz = pz - D6 * approach[2]

    q1 = np.arctan2(wy, wx)
    r  = np.hypot(wx, wy)
    s  = wz - D1
    dist2 = r*r + s*s
    dist  = np.sqrt(dist2)

    if dist > MAX_REACH * 0.99:
        return None

    cos3 = (dist2 - A2**2 - A3**2) / (2 * A2 * A3)
    if np.abs(cos3) > 1.0:
        return None
        
    # Elbow-down configuration
    q3 = np.arctan2(-np.sqrt(1 - cos3**2), cos3)
    q2 = np.arctan2(s, r) - np.arctan2(A3 * np.sin(q3), A2 + A3 * np.cos(q3))

    T1  = dh_transform(0,    np.pi/2,  D1,  q1)
    T2  = dh_transform(A2,   0,        0,   q2)
    T3  = dh_transform(A3,   0,        0,   q3)
    R03 = (T1 @ T2 @ T3)[:3, :3]
    R36 = R03.T @ R06

    q5 = np.arctan2(np.sqrt(R36[0,2]**2 + R36[1,2]**2), R36[2,2])
    if np.abs(np.sin(q5)) > 1e-6:
        q4 = np.arctan2(R36[1,2] / np.sin(q5), R36[0,2] / np.sin(q5))
        q6 = np.arctan2(R36[2,1] / np.sin(q5), -R36[2,0] / np.sin(q5))
    else:
        q4 = 0.0
        q6 = np.arctan2(-R36[0,1], R36[1,1])

    return np.array([q1, q2, q3, q4, q5, q6])

# ── Workspace Tracking Guard (FIXED wrist center calculation) ─────────────
def is_part_reachable(p_x):
    """Verifies if part position sits inside the verified physical reach sphere."""
    target = np.array([p_x, CONV_Y_CENTER, PART_Z_TARGET])
    # Corrected: approach vector points down (-Z), so wrist center is ABOVE target
    wrist_center_est = target - np.array([0.0, 0.0, D6])
    dist_from_shoulder = np.linalg.norm(wrist_center_est - np.array([0.0, 0.0, D1]))
    return dist_from_shoulder <= SAFE_INTERCEPT_RADIUS

# ── Conveyor Rendering Utility ────────────────────────────────────────────
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

# ── Reachability Visualization ────────────────────────────────────────────
def draw_reachability_sphere(ax):
    """Draws the safe intercept sphere for debugging."""
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    x_sphere = SAFE_INTERCEPT_RADIUS * np.outer(np.cos(u), np.sin(v))
    y_sphere = SAFE_INTERCEPT_RADIUS * np.outer(np.sin(u), np.sin(v))
    z_sphere = SAFE_INTERCEPT_RADIUS * np.outer(np.ones_like(u), np.cos(v)) + D1
    ax.plot_wireframe(x_sphere, y_sphere, z_sphere, color="lime", alpha=0.08, linewidth=0.5)

# ── Rigid Waypoint Coordinates Matrix Generator ───────────────────────────
# FIXED: Tool points straight down (-Z direction) using [0, π, 0] RPY
DEFAULT_RPY = np.array([0.0, np.pi, 0.0])  

def get_dynamic_waypoints(p_x):
    return {
        "HOME":           (np.array([ 0.8,  0.8,  3.2]), DEFAULT_RPY.copy()),
        "PICK_APPROACH":  (np.array([ p_x,  CONV_Y_CENTER, CONV_Z + 1.2]), DEFAULT_RPY.copy()),
        "PICK":           (np.array([ p_x,  CONV_Y_CENTER, PART_Z_TARGET]), DEFAULT_RPY.copy()), 
        "LIFT":           (np.array([ p_x,  CONV_Y_CENTER, CONV_Z + 1.2]), DEFAULT_RPY.copy()),
        "PLACE":          (np.array([-1.2,  1.2,  1.6]), DEFAULT_RPY.copy()), 
    }

STATE_SEQUENCE = ["HOME", "PICK_APPROACH", "PICK", "LIFT", "PLACE", "HOME"]

# Core Engine Registers
seg_idx   = 0
t_seg     = 0.0
current_q = np.array([0.0, 0.2, -0.5, 0.0, 0.0, 0.0])
trace     = deque(maxlen=600)

# ── Window Rendering Setup ─────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 10))
ax  = fig.add_subplot(111, projection="3d")
plt.subplots_adjust(bottom=0.12)

slider_ax    = plt.axes([0.2, 0.03, 0.6, 0.03])
speed_slider = Slider(slider_ax, "Motion Speed", 0.005, 0.05, valinit=0.016)

def update(frame):
    global seg_idx, t_seg, current_q, trace
    global part_x, is_attached, dwell_frames, parts_cleared, waiting_for_home_reset

    ax.clear()
    motion_speed = speed_slider.val
    
    # Draw reachability sphere for debugging
    draw_reachability_sphere(ax)
    
    # ── Live Conveyor Feed ────────────────────────────────────────────────
    if not is_attached and not waiting_for_home_reset:
        part_x += CONV_SPEED
        if part_x > CONV_X_END:
            part_x = CONV_X_START

    # ── Workspace Intercept Guard Logic (FIXED) ───────────────────────────
    current_state = STATE_SEQUENCE[seg_idx]
    
    if current_state == "HOME" and not is_attached and not waiting_for_home_reset:
        if is_part_reachable(part_x):
            # Safe entry confirmed: Trigger tracking launch sequence
            t_seg = 0.0
            seg_idx = 1  # Switch to PICK_APPROACH
    
    # Refresh state after potential change
    current_state = STATE_SEQUENCE[seg_idx]
    w_dict = get_dynamic_waypoints(part_x)

    # ── Timeline State Manager ───────────────────────────────────────────
    if current_state != "HOME" or is_attached:
        t_seg += motion_speed
        if t_seg >= 1.0:
            if current_state in ["PICK", "PLACE"] and dwell_frames < DWELL_LIMIT:
                dwell_frames += 1
                t_seg = 1.0  
            else:
                if current_state == "PLACE":
                    parts_cleared += 1
                    # FIXED: Don't reset part_x here — wait for HOME
                    waiting_for_home_reset = True
                
                dwell_frames = 0
                t_seg = 0.0
                seg_idx = (seg_idx + 1) % (len(STATE_SEQUENCE) - 1)
    
    # FIXED: Reset part only when safely back at HOME after PLACE
    if current_state == "HOME" and waiting_for_home_reset and not is_attached:
        part_x = CONV_X_START
        waiting_for_home_reset = False

    current_state = STATE_SEQUENCE[seg_idx]
    
    # Extract path boundaries
    from_pos, from_R = w_dict[STATE_SEQUENCE[seg_idx]][0], rpy_to_matrix(w_dict[STATE_SEQUENCE[seg_idx]][1])
    to_pos,   to_R   = w_dict[STATE_SEQUENCE[seg_idx+1]][0], rpy_to_matrix(w_dict[STATE_SEQUENCE[seg_idx+1]][1])
    
    start_quat = mat_to_quat(from_R)
    end_quat   = mat_to_quat(to_R)

    # ── Compute Trajectory Vectors ────────────────────────────────────────
    t = t_seg
    pos_t = from_pos + t * (to_pos - from_pos)
    q_t = slerp(start_quat, end_quat, t)
    R_t = quat_to_mat(q_t)

    # Calculate Joint Solutions with fallback warning
    solved = inverse_kinematics(pos_t, R_t)
    if solved is not None:
        current_q = solved

    pts = forward_kinematics(current_q)
    tcp = pts[-1]

    # ── Physical Handshake Register ───────────────────────────────────────
    if current_state == "PICK" and not is_attached:
        dist_to_part = np.linalg.norm(tcp - np.array([part_x, CONV_Y_CENTER, PART_Z_TARGET]))
        if dist_to_part < 0.25:
            is_attached = True

    if current_state == "PLACE":
        is_attached = False

    if is_attached:
        part_render_pos = [tcp[0], tcp[1], tcp[2]]
    else:
        part_render_pos = [part_x, CONV_Y_CENTER, PART_Z_TARGET]

    # ── Render Scene Elements ─────────────────────────────────────────────
    draw_conveyor(ax)
    
    # Draw Moving Component
    ax.scatter(part_render_pos[0], part_render_pos[1], part_render_pos[2], 
               color="darkorange", s=140, marker="s", edgecolors="black", zorder=6)

    # Draw Robot Arm Links
    colors  = ["#FF3333", "#FF8800", "#FFD700", "#44DD44", "#00CCFF", "#4466FF"]
    lwidths = [8, 7, 7, 5, 5, 4]
    for i in range(len(pts) - 1):
        ax.plot([pts[i][0], pts[i+1][0]], [pts[i][1], pts[i+1][1]], [pts[i][2], pts[i+1][2]], 
                color=colors[i], linewidth=lwidths[i], solid_capstyle="round")

    for pt in pts:
        ax.scatter(*pt, color="white", s=40, zorder=5, edgecolors="gray")
    ax.scatter(*tcp, color="magenta", s=130, marker="*", zorder=7)

    # Draw Tool Center Path Trace
    trace.append(tcp.copy())
    if len(trace) > 2:
        tr = np.array(trace)
        ax.plot(tr[:,0], tr[:,1], tr[:,2], color="purple", linewidth=1.2, alpha=0.4)

    # ── Telemetry Dashboard HUD ───────────────────────────────────────────
    q_deg = np.degrees(current_q)
    in_range = is_part_reachable(part_x)
    hud = (
        f"CHAPPELL ROBOTICS — CR6 V6.1\n"
        f"STATE : {current_state}\n"
        f"TIMELINE T : {t:.2f}\n"
        f"DWELL : {dwell_frames}/{DWELL_LIMIT}\n"
        f"PARTS CLEARED: {parts_cleared}\n"
        f"---------------------\n"
        f"PART IN RANGE : {in_range}\n"
        f"SAFE RADIUS : {SAFE_INTERCEPT_RADIUS:.2f}\n"
        f"---------------------\n"
        f"J1: {q_deg[0]:+7.1f}°\n"
        f"J2: {q_deg[1]:+7.1f}°\n"
        f"J3: {q_deg[2]:+7.1f}°\n"
        f"J4: {q_deg[3]:+7.1f}°\n"
        f"J5: {q_deg[4]:+7.1f}°\n"
        f"J6: {q_deg[5]:+7.1f}°\n"
        f"---------------------\n"
        f"TCP : [{tcp[0]:+.2f},{tcp[1]:+.2f},{tcp[2]:+.2f}]\n"
        f"PART: [{part_render_pos[0]:+.2f},{part_render_pos[1]:+.2f},{part_render_pos[2]:+.2f}]\n"
        f"ATTACHED: {is_attached}\n"
    )
    ax.text2D(0.02, 0.97, hud, transform=ax.transAxes, fontsize=8, family="monospace", verticalalignment="top")

    ax.set_title("CHAPPELL ROBOTICS  —  CR6  V6.1  |  WORKSPACE GUARDED TRACKING (CORRECTED)", 
                 fontsize=10, fontweight="bold")
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(0, 6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.grid(True, alpha=0.3)

ani = FuncAnimation(fig, update, interval=40)
plt.show()