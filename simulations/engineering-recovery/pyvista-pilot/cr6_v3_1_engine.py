"""
Phase 3B, Class C migration: CR6_6Axis_V3_1_Corrected.py has no classes --
all simulation state (part_position, is_attached, dwell_frames, seg_idx,
t_seg, current_q) lives as module-level globals, mutated directly inside
the single matplotlib `update(frame)` callback alongside the drawing calls.
That's the real architectural separation this migration class needs
(per PHASE_3A_CLASSIFICATION.md): there is no existing class boundary to
extract, so this file reimplements the exact same computation -- every DH
constant, every formula, every state transition -- inside a CR6Engine
class with a step()/snapshot() split, instead of copying an existing
class verbatim the way the Class A migrations did. The original source
file (simulations/CR6_6Axis/CR6_6Axis_V3_1_Corrected.py) was read, not
modified, to build this.
"""
import numpy as np
from collections import deque

# ── DH Parameters (Pristine V3 Geometry) -- identical to the source ────────
D1 = 1.5
A2 = 2.5
A3 = 2.0
D6 = 0.5
MAX_REACH = A2 + A3 + D6

# ── Conveyor Layout -- identical to the source ──────────────────────────────
CONV_X_START = -4.0
CONV_X_END = 4.0
CONV_Y_CENTER = -2.8
CONV_WIDTH = 1.0
CONV_Z = 1.0
CONV_LEG_Z = 0.0

PICK_ZONE_X = 1.0

# ── Part Geometry -- identical to the source ────────────────────────────────
PART_HEIGHT = 0.2
PART_X_STATIC = PICK_ZONE_X
PART_Y_STATIC = CONV_Y_CENTER
PART_Z_STATIC = CONV_Z + (PART_HEIGHT / 2.0)

DOWN_ORIENTATION = np.array([0.0, np.pi, 0.0])
PICK_APPROACH_Z = CONV_Z + 1.2
PICK_TARGET_POS = np.array([PART_X_STATIC, PART_Y_STATIC, PART_Z_STATIC])

WAYPOINTS = {
    "HOME": (np.array([1.5, 0.5, 3.5]), DOWN_ORIENTATION.copy()),
    "PICK_APPROACH": (np.array([PICK_ZONE_X, CONV_Y_CENTER, PICK_APPROACH_Z]), DOWN_ORIENTATION.copy()),
    "PICK": (PICK_TARGET_POS.copy(), DOWN_ORIENTATION.copy()),
    "LIFT": (np.array([PICK_ZONE_X, CONV_Y_CENTER, PICK_APPROACH_Z]), DOWN_ORIENTATION.copy()),
    "PLACE": (np.array([-1.5, 1.5, 1.5]), DOWN_ORIENTATION.copy()),
}
STATE_SEQUENCE = ["HOME", "PICK_APPROACH", "PICK", "LIFT", "PLACE", "HOME"]
DWELL_LIMIT = 15


def dh_transform(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])


def rpy_to_matrix(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def mat_to_quat(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


def quat_to_mat(q):
    norm = np.linalg.norm(q)
    if norm < 1e-6:
        return np.eye(3)
    w, x, y, z = q / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
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
    theta = theta0 * t
    sin0 = np.sin(theta0)
    return (np.sin(theta0 - theta) * q0 + np.sin(theta) * q1) / sin0


def forward_kinematics(q):
    q1, q2, q3, q4, q5, q6 = q
    dh = [
        [0, np.pi / 2, D1, q1],
        [A2, 0, 0, q2],
        [A3, 0, 0, q3],
        [0, -np.pi / 2, 0, q4],
        [0, np.pi / 2, 0, q5],
        [0, 0, D6, q6],
    ]
    T = np.eye(4)
    positions = [T[:3, 3].copy()]
    for row in dh:
        T = T @ dh_transform(*row)
        positions.append(T[:3, 3].copy())
    return positions


def inverse_kinematics(target_pos, R06):
    px, py, pz = target_pos
    approach = R06[:, 2]
    wx = px - D6 * approach[0]
    wy = py - D6 * approach[1]
    wz = pz - D6 * approach[2]
    q1 = np.arctan2(wy, wx)
    r = np.hypot(wx, wy)
    s = wz - D1
    dist2 = r * r + s * s
    dist = np.sqrt(dist2)
    if dist > (A2 + A3) * 0.99 or dist < np.abs(A2 - A3):
        return None
    cos3 = (dist2 - A2 ** 2 - A3 ** 2) / (2 * A2 * A3)
    cos3 = np.clip(cos3, -1.0, 1.0)
    q3 = np.arctan2(-np.sqrt(1 - cos3 ** 2), cos3)
    q2 = np.arctan2(s, r) - np.arctan2(A3 * np.sin(q3), A2 + A3 * np.cos(q3))
    T1 = dh_transform(0, np.pi / 2, D1, q1)
    T2 = dh_transform(A2, 0, 0, q2)
    T3 = dh_transform(A3, 0, 0, q3)
    R03 = (T1 @ T2 @ T3)[:3, :3]
    R36 = R03.T @ R06
    q5 = np.arctan2(np.sqrt(R36[0, 2] ** 2 + R36[1, 2] ** 2), R36[2, 2])
    if np.abs(np.sin(q5)) > 1e-6:
        q4 = np.arctan2(R36[1, 2], R36[0, 2])
        q6 = np.arctan2(R36[2, 1], -R36[2, 0])
    else:
        q4 = 0.0
        q6 = np.arctan2(-R36[0, 1], R36[1, 1])
    return np.array([q1, q2, q3, q4, q5, q6])


def get_waypoint_pose(name):
    pos, rpy = WAYPOINTS[name]
    return pos.copy(), rpy_to_matrix(rpy)


class CR6Engine:
    """
    Everything that was module-level global state in the original file,
    now owned by an instance -- same computation, same transition logic,
    reorganized so a renderer can read it without also being the thing
    that mutates it.
    """

    def __init__(self):
        self.part_position = [PART_X_STATIC, PART_Y_STATIC, PART_Z_STATIC]
        self.is_attached = False
        self.dwell_frames = 0

        self.seg_idx = 0
        self.t_seg = 0.0
        start_pos, start_R = get_waypoint_pose(STATE_SEQUENCE[0])
        end_pos, end_R = get_waypoint_pose(STATE_SEQUENCE[1])
        self.start_pos, self.start_R = start_pos, start_R
        self.end_pos, self.end_R = end_pos, end_R
        self.start_q_quat = mat_to_quat(start_R)
        self.end_q_quat = mat_to_quat(end_R)

        self.current_q = np.array([0.0, 0.2, -0.5, 0.0, 0.0, 0.0])
        self.trace = deque(maxlen=600)
        self.frame = 0

    def step(self, speed=0.012):
        self.frame += 1
        current_state = STATE_SEQUENCE[self.seg_idx]

        self.t_seg += speed
        if self.t_seg >= 1.0:
            if current_state in ("PICK", "PLACE") and self.dwell_frames < DWELL_LIMIT:
                self.dwell_frames += 1
                self.t_seg = 1.0
            else:
                self.dwell_frames = 0
                self.t_seg = 0.0
                self.seg_idx = (self.seg_idx + 1) % (len(STATE_SEQUENCE) - 1)
                self.start_pos, self.start_R = get_waypoint_pose(STATE_SEQUENCE[self.seg_idx])
                self.end_pos, self.end_R = get_waypoint_pose(STATE_SEQUENCE[self.seg_idx + 1])
                self.start_q_quat = mat_to_quat(self.start_R)
                self.end_q_quat = mat_to_quat(self.end_R)

        current_state = STATE_SEQUENCE[self.seg_idx]
        t = self.t_seg
        pos_t = self.start_pos + t * (self.end_pos - self.start_pos)
        q_t = slerp(self.start_q_quat, self.end_q_quat, t)
        R_t = quat_to_mat(q_t)

        solved = inverse_kinematics(pos_t, R_t)
        if solved is not None:
            self.current_q = solved

        pts = forward_kinematics(self.current_q)
        tcp = pts[-1]

        if current_state == "PICK":
            dist = np.linalg.norm(np.array(tcp) - np.array(self.part_position))
            if dist < 0.25:
                self.is_attached = True
        elif current_state == "PLACE":
            self.is_attached = False
            self.part_position = [WAYPOINTS["PLACE"][0][0], WAYPOINTS["PLACE"][0][1], WAYPOINTS["PLACE"][0][2]]
        elif current_state == "HOME" and not self.is_attached:
            self.part_position = [PART_X_STATIC, PART_Y_STATIC, PART_Z_STATIC]

        if self.is_attached:
            self.part_position = list(tcp)

        self.trace.append(tcp.copy())
        self._last_state = current_state
        self._last_pts = pts

    def snapshot(self):
        """Read-only render view -- no sim logic here, matches the Class A files' own convention."""
        return {
            "frame": self.frame,
            "state": self._last_state,
            "pts": self._last_pts,
            "tcp": self._last_pts[-1],
            "part_position": list(self.part_position),
            "is_attached": self.is_attached,
            "trace": list(self.trace),
        }
