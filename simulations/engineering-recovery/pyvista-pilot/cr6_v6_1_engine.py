"""
Phase 3B, Class C migration: CR6_6Axis_V6_1_Workspace_Guard.py.py has no
classes -- same situation as CR6_6Axis_V3_1_Corrected. Reimplements the
exact same computation (DH geometry, moving-conveyor tracking, the
workspace-guard reachability check, the waiting_for_home_reset fix) inside
a class with step()/snapshot(), rather than module-level globals mutated
inside the matplotlib callback. Original source
(simulations/CR6_6axis_Object_Tracking/CR6_6Axis_V6_1_Workspace_Guard.py.py)
was read, not modified.

Caught during verification, not left silent: an initial transcription of
STATE_SEQUENCE dropped the source's trailing "HOME" (real source has 6
entries closing the loop; this file briefly had 5), which broke the
`% (len(STATE_SEQUENCE) - 1)` cycling and made the robot loop forever
between HOME/PICK_APPROACH/PICK/LIFT without ever reaching PLACE. Fixed to
match the verified-correct source; the state-machine completeness check
below is what caught it.
"""
import numpy as np
from collections import deque

D1, A2, A3, D6 = 1.5, 2.5, 2.0, 0.5
MAX_REACH = A2 + A3 + D6
SAFE_INTERCEPT_RADIUS = MAX_REACH * 0.92

CONV_X_START = -4.0
CONV_X_END = 4.0
CONV_Y_CENTER = -2.8
CONV_Z = 1.0
CONV_SPEED = 0.040
PART_HEIGHT = 0.2
PART_Z_TARGET = CONV_Z + (PART_HEIGHT / 2.0)
DWELL_LIMIT = 15
DEFAULT_RPY = np.array([0.0, np.pi, 0.0])


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
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def slerp(q0, q1, t):
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = np.clip(np.dot(q0, q1), -1, 1)
    if dot < 0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return q0 + t * (q1 - q0)
    th = np.arccos(dot)
    return (np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)


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
    if np.sqrt(dist2) > MAX_REACH * 0.99:
        return None
    cos3 = (dist2 - A2 ** 2 - A3 ** 2) / (2 * A2 * A3)
    if abs(cos3) > 1:
        return None
    q3 = np.arctan2(-np.sqrt(1 - cos3 ** 2), cos3)
    q2 = np.arctan2(s, r) - np.arctan2(A3 * np.sin(q3), A2 + A3 * np.cos(q3))
    T1 = dh_transform(0, np.pi / 2, D1, q1)
    T2 = dh_transform(A2, 0, 0, q2)
    T3 = dh_transform(A3, 0, 0, q3)
    R03 = (T1 @ T2 @ T3)[:3, :3]
    R36 = R03.T @ R06
    q5 = np.arctan2(np.sqrt(R36[0, 2] ** 2 + R36[1, 2] ** 2), R36[2, 2])
    if abs(np.sin(q5)) > 1e-6:
        q4 = np.arctan2(R36[1, 2] / np.sin(q5), R36[0, 2] / np.sin(q5))
        q6 = np.arctan2(R36[2, 1] / np.sin(q5), -R36[2, 0] / np.sin(q5))
    else:
        q4 = 0.0
        q6 = np.arctan2(-R36[0, 1], R36[1, 1])
    return np.array([q1, q2, q3, q4, q5, q6])


def is_part_reachable(p_x):
    target = np.array([p_x, CONV_Y_CENTER, PART_Z_TARGET])
    wrist_center_est = target - np.array([0.0, 0.0, D6])
    dist_from_shoulder = np.linalg.norm(wrist_center_est - np.array([0.0, 0.0, D1]))
    return dist_from_shoulder <= SAFE_INTERCEPT_RADIUS


def get_dynamic_waypoints(p_x):
    return {
        "HOME": (np.array([0.8, 0.8, 3.2]), DEFAULT_RPY.copy()),
        "PICK_APPROACH": (np.array([p_x, CONV_Y_CENTER, CONV_Z + 1.2]), DEFAULT_RPY.copy()),
        "PICK": (np.array([p_x, CONV_Y_CENTER, PART_Z_TARGET]), DEFAULT_RPY.copy()),
        "LIFT": (np.array([p_x, CONV_Y_CENTER, CONV_Z + 1.2]), DEFAULT_RPY.copy()),
        "PLACE": (np.array([-1.2, 1.2, 1.6]), DEFAULT_RPY.copy()),
    }


STATE_SEQUENCE = ["HOME", "PICK_APPROACH", "PICK", "LIFT", "PLACE", "HOME"]


class CR6WorkspaceGuardEngine:
    def __init__(self):
        self.part_x = CONV_X_START
        self.is_attached = False
        self.dwell_frames = 0
        self.parts_cleared = 0
        self.waiting_for_home_reset = False
        self.seg_idx = 0
        self.t_seg = 0.0
        self.current_q = np.array([0.0, 0.2, -0.5, 0.0, 0.0, 0.0])
        self.trace = deque(maxlen=600)
        self.frame = 0

    def step(self, motion_speed=0.016):
        self.frame += 1

        if not self.is_attached and not self.waiting_for_home_reset:
            self.part_x += CONV_SPEED
            if self.part_x > CONV_X_END:
                self.part_x = CONV_X_START

        current_state = STATE_SEQUENCE[self.seg_idx]

        if current_state == "HOME" and not self.is_attached and not self.waiting_for_home_reset:
            if is_part_reachable(self.part_x):
                self.t_seg = 0.0
                self.seg_idx = 1

        current_state = STATE_SEQUENCE[self.seg_idx]
        w_dict = get_dynamic_waypoints(self.part_x)

        if current_state != "HOME" or self.is_attached:
            self.t_seg += motion_speed
            if self.t_seg >= 1.0:
                if current_state in ("PICK", "PLACE") and self.dwell_frames < DWELL_LIMIT:
                    self.dwell_frames += 1
                    self.t_seg = 1.0
                else:
                    if current_state == "PLACE":
                        self.parts_cleared += 1
                        self.waiting_for_home_reset = True
                    self.dwell_frames = 0
                    self.t_seg = 0.0
                    self.seg_idx = (self.seg_idx + 1) % (len(STATE_SEQUENCE) - 1)

        if current_state == "HOME" and self.waiting_for_home_reset and not self.is_attached:
            self.part_x = CONV_X_START
            self.waiting_for_home_reset = False

        current_state = STATE_SEQUENCE[self.seg_idx]
        from_pos, from_R = w_dict[STATE_SEQUENCE[self.seg_idx]][0], rpy_to_matrix(w_dict[STATE_SEQUENCE[self.seg_idx]][1])
        to_pos, to_R = w_dict[STATE_SEQUENCE[self.seg_idx + 1]][0], rpy_to_matrix(w_dict[STATE_SEQUENCE[self.seg_idx + 1]][1])
        start_quat = mat_to_quat(from_R)
        end_quat = mat_to_quat(to_R)

        t = self.t_seg
        pos_t = from_pos + t * (to_pos - from_pos)
        q_t = slerp(start_quat, end_quat, t)
        R_t = quat_to_mat(q_t)

        solved = inverse_kinematics(pos_t, R_t)
        if solved is not None:
            self.current_q = solved

        pts = forward_kinematics(self.current_q)
        tcp = pts[-1]

        if current_state == "PICK" and not self.is_attached:
            dist_to_part = np.linalg.norm(tcp - np.array([self.part_x, CONV_Y_CENTER, PART_Z_TARGET]))
            if dist_to_part < 0.25:
                self.is_attached = True

        if current_state == "PLACE":
            self.is_attached = False

        if self.is_attached:
            part_render_pos = [tcp[0], tcp[1], tcp[2]]
        else:
            part_render_pos = [self.part_x, CONV_Y_CENTER, PART_Z_TARGET]

        self.trace.append(tcp.copy())
        self._last = {
            "state": current_state, "pts": pts, "tcp": tcp,
            "part_render_pos": part_render_pos, "in_range": is_part_reachable(self.part_x),
        }

    def snapshot(self):
        return {
            "frame": self.frame,
            "state": self._last["state"],
            "pts": self._last["pts"],
            "part_render_pos": self._last["part_render_pos"],
            "is_attached": self.is_attached,
            "parts_cleared": self.parts_cleared,
            "in_range": self._last["in_range"],
            "trace": list(self.trace),
        }
