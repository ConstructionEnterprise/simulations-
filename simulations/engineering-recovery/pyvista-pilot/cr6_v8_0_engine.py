"""
CHAPPELL ROBOTICS
CR6 V7.5 — Dual Robot Manufacturing Cell

Clean rewrite from V6.1 validated foundation.

DESIGN PRINCIPLES
  1. DH convention from V6.1 — alpha=+pi/2 Joint1, self-consistent, unchanged
  2. Part ownership is the ONLY communication between robots
  3. Ownership transfers at physical events only (PICK/PLACE actions)
  4. COMMITTED_TO_A: part reserved but still renders on conveyor
  5. LIFT height = 2.0 (workspace verified, not SAFE_Z)
  6. Launch only when LIFT is reachable, not just PICK
  7. State set at top of step(), consistent throughout frame
  8. Waypoints locked at launch, no per-frame rebuilding

OWNERSHIP CHAIN
  ON_CONVEYOR → COMMITTED_TO_A → HELD_BY_A → IN_FIXTURE → HELD_BY_B → COMPLETE

LAYOUT
  Robot A base: [0, 0, 0]   picks from conveyor (Y=-2.8)
  Robot B base: [0, 3.5, 0] picks from fixture, places to output
  Fixture:      [-1, 1, 1.2]
  Output:       [-2.5, 4.5, 1.2]
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from collections import deque

# ═══════════════════════════════════════════════════════════════════════
# SECTION 1 — DH PARAMETERS  (V6.1 validated — do not modify)
# ═══════════════════════════════════════════════════════════════════════

D1 = 1.5
A2 = 2.5
A3 = 2.0
D6 = 0.5
MAX_REACH = A2 + A3 + D6        # 5.0

# Launch radius: 90% of MAX_REACH
# Verified: part_x >= -3.5 gives dist=4.59 for LIFT — within 4.5 limit
SAFE_REACH = MAX_REACH * 0.90   # 4.5

# ═══════════════════════════════════════════════════════════════════════
# SECTION 2 — WORLD CONSTANTS
# ═══════════════════════════════════════════════════════════════════════

CONV_X_START   = -4.0
CONV_X_END     =  4.0
CONV_Y_CENTER  = -2.8
CONV_WIDTH     =  1.0
CONV_Z         =  1.0
CONV_SPEED     =  0.04

PART_Z         =  CONV_Z + 0.1   # 1.10 — part centre on belt

RA_BASE        = np.array([0.0,  0.0,  0.0])
RB_BASE        = np.array([0.0,  3.5,  0.0])

FIXTURE_POS    = np.array([-1.0,  1.0,  1.2])
OUTPUT_POS     = np.array([-2.5,  4.5,  1.2])

LIFT_Z         =  2.0     # verified reachable from far conveyor positions
SAFE_Z         =  3.2     # used for fixture/output approach only
DWELL_LIMIT    =  15
ARRIVE_DIST    =  0.25

# Tool points straight down — RPY [0, pi, 0] (V6.1 validated)
DEFAULT_RPY    = np.array([0.0, np.pi, 0.0])

# Part renders slightly below TCP
PART_OFFSET    = np.array([0.0, 0.0, -0.15])

# ═══════════════════════════════════════════════════════════════════════
# SECTION 3 — MATH  (copied exactly from V6.1)
# ═══════════════════════════════════════════════════════════════════════

def dh_transform(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

def rpy_to_R(rpy):
    r, p, y = rpy
    Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

def R_to_quat(R):
    t = R[0,0]+R[1,1]+R[2,2]
    if t > 0:
        s = 0.5/np.sqrt(t+1)
        return np.array([0.25/s,(R[2,1]-R[1,2])*s,(R[0,2]-R[2,0])*s,(R[1,0]-R[0,1])*s])
    elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s = 2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2])
        return np.array([(R[2,1]-R[1,2])/s,0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s])
    elif R[1,1]>R[2,2]:
        s = 2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2])
        return np.array([(R[0,2]-R[2,0])/s,(R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s])
    else:
        s = 2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1])
        return np.array([(R[1,0]-R[0,1])/s,(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s])

def quat_to_R(q):
    q = q/np.linalg.norm(q)
    w,x,y,z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)  ],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)  ],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ])

def slerp(q0, q1, t):
    q0=q0/np.linalg.norm(q0); q1=q1/np.linalg.norm(q1)
    d = np.clip(np.dot(q0,q1),-1,1)
    if d < 0: q1=-q1; d=-d
    if d > 0.9995: return q0+t*(q1-q0)
    th = np.arccos(d)
    return (np.sin((1-t)*th)*q0 + np.sin(t*th)*q1)/np.sin(th)

# ═══════════════════════════════════════════════════════════════════════
# SECTION 4 — KINEMATICS  (copied exactly from V6.1)
# ═══════════════════════════════════════════════════════════════════════

def fk(q):
    """Return 7 world-local joint positions. Last = TCP."""
    q1,q2,q3,q4,q5,q6 = q
    dh = [
        [0,  np.pi/2, D1, q1],
        [A2, 0,       0,  q2],
        [A3, 0,       0,  q3],
        [0, -np.pi/2, 0,  q4],
        [0,  np.pi/2, 0,  q5],
        [0,  0,       D6, q6],
    ]
    T = np.eye(4)
    pts = [T[:3,3].copy()]
    for row in dh:
        T = T @ dh_transform(*row)
        pts.append(T[:3,3].copy())
    return pts

def ik(target_local, R06):
    """Analytical IK. target_local in robot-local space. Returns q or None."""
    px,py,pz = target_local
    ap = R06[:,2]
    wx,wy,wz = px-D6*ap[0], py-D6*ap[1], pz-D6*ap[2]
    q1 = np.arctan2(wy,wx)
    r  = np.hypot(wx,wy)
    s  = wz-D1
    d2 = r*r+s*s
    if np.sqrt(d2) > MAX_REACH*0.99: return None
    c3 = (d2-A2**2-A3**2)/(2*A2*A3)
    if abs(c3)>1: return None
    q3 = np.arctan2(-np.sqrt(1-c3**2), c3)
    q2 = np.arctan2(s,r) - np.arctan2(A3*np.sin(q3), A2+A3*np.cos(q3))
    T1 = dh_transform(0,  np.pi/2, D1, q1)
    T2 = dh_transform(A2, 0,       0,  q2)
    T3 = dh_transform(A3, 0,       0,  q3)
    R03= (T1@T2@T3)[:3,:3]
    R36= R03.T@R06
    q5 = np.arctan2(np.sqrt(R36[0,2]**2+R36[1,2]**2), R36[2,2])
    if abs(np.sin(q5))>1e-6:
        q4 = np.arctan2(R36[1,2]/np.sin(q5), R36[0,2]/np.sin(q5))
        q6 = np.arctan2(R36[2,1]/np.sin(q5),-R36[2,0]/np.sin(q5))
    else:
        q4=0.0; q6=np.arctan2(-R36[0,1],R36[1,1])
    return np.array([q1,q2,q3,q4,q5,q6])

def ik_world(world_pos, base, rpy=None):
    """IK from world position. Returns q or None."""
    R = rpy_to_R(rpy if rpy is not None else DEFAULT_RPY)
    return ik(np.array(world_pos)-base, R)

def tcp_world(q, base):
    return fk(q)[-1] + base

# ═══════════════════════════════════════════════════════════════════════
# SECTION 5 — REACHABILITY  (checks LIFT, the tighter constraint)
# ═══════════════════════════════════════════════════════════════════════

def lift_reachable(part_x, base):
    """True if LIFT at (part_x, CONV_Y, LIFT_Z) is within SAFE_REACH from base."""
    local = np.array([part_x, CONV_Y_CENTER, LIFT_Z]) - base
    # approach vector for DEFAULT_RPY [0,pi,0] is [0,0,-1]
    wrist = local - np.array([0,0,-D6])  # = local + [0,0,D6]
    dist  = np.linalg.norm(wrist - np.array([0,0,D1]))
    return dist <= SAFE_REACH

# ═══════════════════════════════════════════════════════════════════════
# SECTION 6 — ROBOT
# ═══════════════════════════════════════════════════════════════════════

class Robot:
    """
    Single CR6 robot. Executes a fixed waypoint sequence via Cartesian
    interpolation + SLERP. Completely self-contained.

    Waypoints are a list of (world_pos, rpy) tuples.
    Robot interpolates segment by segment, dwells at dwell_states,
    returns action string when a segment completes.
    """

    def __init__(self, base, name, colors):
        self.base   = np.array(base)
        self.name   = name
        self.colors = colors
        self.active = False
        self.state  = "IDLE"
        self.trace  = deque(maxlen=400)

        # Solve a safe park pose above base so robot renders correctly from frame 1
        # Park at [base_x+0, base_y+0, 3.5] — straight up, within reach for any base
        park_world = np.array([self.base[0], self.base[1], 3.5])
        R_default  = rpy_to_R(DEFAULT_RPY)
        q_park     = ik(park_world - self.base, R_default)
        self.q     = q_park if q_park is not None else np.array([0.0, 0.8, -0.5, 0.0, 0.5, 0.0])

        # Interpolation state
        self._wps      = []      # list of (world_pos, rpy)
        self._names    = []      # state name per waypoint
        self._dwell_at = set()   # state names that require dwell
        self._idx      = 0
        self._t        = 0.0
        self._dwell    = 0

    def launch(self, waypoints, names, dwell_at):
        """
        Start a motion sequence.
        Prepends current TCP position as waypoint 0 so robot moves
        smoothly from its current pose — no snap on launch.
        """
        # Current TCP world position becomes the starting waypoint
        current_tcp = self.tcp()
        current_rpy = waypoints[0][1].copy()   # same orientation as first waypoint

        start_wp    = (current_tcp, current_rpy)

        self._wps      = [start_wp] + list(waypoints)
        self._names    = ["CURRENT"] + list(names)
        self._dwell_at = dwell_at
        self._idx      = 0
        self._t        = 0.0
        self._dwell    = 0
        self.active    = True
        self.state     = names[0]

    def step(self, speed):
        """
        Advance one frame. Returns completed state name or None.
        State is always set BEFORE return so callers read current state.
        """
        if not self.active or len(self._wps) < 2:
            return None

        self._t += speed
        action = None

        if self._t >= 1.0:
            cur_name = self._names[self._idx]
            if cur_name in self._dwell_at and self._dwell < DWELL_LIMIT:
                # Dwell — hold position
                self._dwell += 1
                self._t = 1.0
            else:
                # Advance to next waypoint
                self._dwell = 0
                self._t     = 0.0
                action      = cur_name
                self._idx  += 1

                if self._idx >= len(self._wps) - 1:
                    # Sequence complete
                    self.state  = "IDLE"
                    self.active = False
                    return action

                self.state = self._names[self._idx]

        # Interpolate between current and next waypoint
        i     = self._idx
        p0, r0 = self._wps[i]
        p1, r1 = self._wps[i+1]
        t      = self._t

        pos_t = np.array(p0) + t*(np.array(p1)-np.array(p0))
        q_t   = slerp(R_to_quat(rpy_to_R(r0)), R_to_quat(rpy_to_R(r1)), t)
        R_t   = quat_to_R(q_t)

        q_sol = ik(pos_t - self.base, R_t)
        if q_sol is not None:
            self.q = q_sol

        return action

    def tcp(self):
        return tcp_world(self.q, self.base)

    def joint_pts(self):
        return [p + self.base for p in fk(self.q)]

# ═══════════════════════════════════════════════════════════════════════
# SECTION 7 — WAYPOINT BUILDERS
# ═══════════════════════════════════════════════════════════════════════

R_DEFAULT = DEFAULT_RPY

def wp(pos, rpy=None):
    return (np.array(pos), rpy if rpy is not None else R_DEFAULT.copy())

def build_A(part_x):
    """Robot A: conveyor pick → fixture place."""
    px, py = part_x, CONV_Y_CENTER
    return [
        wp([px,       py,       SAFE_Z  ]),   # HOME
        wp([px,       py,       CONV_Z+1.2]), # PICK_APPROACH
        wp([px,       py,       PART_Z  ]),   # PICK — TCP descends to part level
        wp([px,       py,       LIFT_Z  ]),   # LIFT
        wp(FIXTURE_POS + [0,0,SAFE_Z-FIXTURE_POS[2]]), # FIXTURE_APPROACH
        wp(FIXTURE_POS),                       # FIXTURE_PLACE
        wp([px,       py,       SAFE_Z  ]),   # RETURN_HOME
    ], ["HOME","PICK_APPROACH","PICK","LIFT","FIXTURE_APPROACH","FIXTURE_PLACE","RETURN_HOME"], \
       {"PICK","FIXTURE_PLACE"}

def build_B():
    """Robot B: fixture pick → output place."""
    return [
        wp(FIXTURE_POS + [0,0,SAFE_Z-FIXTURE_POS[2]]), # HOME
        wp(FIXTURE_POS + [0,0,0.8]),                    # FIXTURE_APPROACH
        wp(FIXTURE_POS),                                 # FIXTURE_PICK
        wp(FIXTURE_POS + [0,0,SAFE_Z-FIXTURE_POS[2]]), # LIFT
        wp(OUTPUT_POS  + [0,0,SAFE_Z-OUTPUT_POS[2]]),  # OUTPUT_APPROACH
        wp(OUTPUT_POS),                                  # OUTPUT_PLACE
        wp(FIXTURE_POS + [0,0,SAFE_Z-FIXTURE_POS[2]]), # RETURN_HOME
    ], ["HOME","FIXTURE_APPROACH","FIXTURE_PICK","LIFT","OUTPUT_APPROACH","OUTPUT_PLACE","RETURN_HOME"], \
       {"FIXTURE_PICK","OUTPUT_PLACE"}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 8 — PART
# ═══════════════════════════════════════════════════════════════════════

class Part:
    def __init__(self):
        self.reset()

    def reset(self):
        self.x         = CONV_X_START
        self.ownership = "ON_CONVEYOR"
        self.pos       = np.array([CONV_X_START, CONV_Y_CENTER, PART_Z])

# ═══════════════════════════════════════════════════════════════════════
# SECTION 9 — WORLD ORCHESTRATOR
# ═══════════════════════════════════════════════════════════════════════

class World:
    def __init__(self):
        self.part    = Part()
        self.ra      = Robot(RA_BASE, "A",
                             ["#FF3333","#FF8800","#FFD700","#44DD44","#00CCFF","#4466FF"])
        self.rb      = Robot(RB_BASE, "B",
                             ["#CC00FF","#FF66CC","#FFFFFF","#00FFCC","#FF9900","#AAAAAA"])
        self.cleared    = 0
        self.frame      = 0
        self._a_offset  = np.zeros(3)   # frozen part-to-TCP offset when A picks
        self._b_offset  = np.zeros(3)   # frozen part-to-TCP offset when B picks

    def step(self, speed):
        self.frame += 1
        p  = self.part
        ra = self.ra
        rb = self.rb

        # ── Conveyor ──────────────────────────────────────────────────
        if p.ownership == "ON_CONVEYOR":
            p.x += CONV_SPEED
            if p.x > CONV_X_END:
                p.x = CONV_X_START
            p.pos = np.array([p.x, CONV_Y_CENTER, PART_Z])

        # Part stays at conveyor pos while committed (visual — no teleport)
        if p.ownership == "COMMITTED_TO_A":
            p.pos = np.array([p.x, CONV_Y_CENTER, PART_Z])

        # ── Robot A launch ────────────────────────────────────────────
        if not ra.active and p.ownership == "ON_CONVEYOR":
            if lift_reachable(p.x, RA_BASE):
                wps, names, dwell = build_A(p.x)
                ra.launch(wps, names, dwell)
                p.ownership = "COMMITTED_TO_A"

        # ── Robot A tick ──────────────────────────────────────────────
        if ra.active:
            ra.step(speed)

            # Pick: attach when TCP reaches part
            if p.ownership == "COMMITTED_TO_A":
                if np.linalg.norm(ra.tcp() - p.pos) < 0.35:
                    p.ownership = "HELD_BY_A"

            # Place: release when TCP reaches fixture AND in place state
            if p.ownership == "HELD_BY_A" and ra.state == "FIXTURE_PLACE":
                if np.linalg.norm(ra.tcp() - FIXTURE_POS) < 0.35:
                    p.ownership = "IN_FIXTURE"
                    p.pos       = FIXTURE_POS.copy()

        # Part follows A TCP while held
        if p.ownership == "HELD_BY_A":
            p.pos = ra.tcp().copy()

        # Safety: if A finishes without picking, return part to conveyor
        if not ra.active and p.ownership == "COMMITTED_TO_A":
            p.ownership = "ON_CONVEYOR"

        # ── Robot B launch ────────────────────────────────────────────
        if not rb.active and p.ownership == "IN_FIXTURE":
            wps, names, dwell = build_B()
            rb.launch(wps, names, dwell)

        # ── Robot B tick ──────────────────────────────────────────────
        if rb.active:
            rb.step(speed)

            # Pick from fixture: attach when TCP reaches fixture
            if p.ownership == "IN_FIXTURE" and rb.state == "FIXTURE_PICK":
                if np.linalg.norm(rb.tcp() - FIXTURE_POS) < 0.35:
                    p.ownership = "HELD_BY_B"

            # Place at output: release when TCP reaches output
            if p.ownership == "HELD_BY_B" and rb.state == "OUTPUT_PLACE":
                if np.linalg.norm(rb.tcp() - OUTPUT_POS) < 0.35:
                    p.ownership = "COMPLETE"
                    p.pos       = OUTPUT_POS.copy()

        # Part follows B TCP while held
        if p.ownership == "HELD_BY_B":
            p.pos = rb.tcp().copy()

        # ── Cycle complete ────────────────────────────────────────────
        if p.ownership == "COMPLETE" and not rb.active:
            self.cleared += 1
            p.reset()

    def snapshot(self):
        """Added for the Phase 3B pyvista migration -- read-only render view, no sim logic."""
        return {
            "frame": self.frame,
            "cleared": self.cleared,
            "ra": {"state": self.ra.state, "active": self.ra.active, "joints": self.ra.joint_pts()},
            "rb": {"state": self.rb.state, "active": self.rb.active, "joints": self.rb.joint_pts()},
            "part": {"pos": self.part.pos.copy(), "ownership": self.part.ownership},
        }
