"""
CONSTRUCTION ENTERPRISES — CHAPPELL ROBOTICS
CE_Integrated_Cell  V1.0

INTEGRATED MODULAR WALL MANUFACTURING CELL
  Station 1 — LGS Framing Cell   (Dual CR6: material handler + assembler)
  Station 2 — Sheathing Cell     (Dual CR6: sheet pickup + fastening)
  Station 3 — Factory Rail       (CR6 on linear rail: inspection / rework / outfeed)

MANUFACTURING FLOW
  Raw studs loaded into material rack (far left)
  → CR6-F1 picks studs/tracks, places on framing table
  → CR6-F2 assembles (fastens) members on table
  → Completed wall frame conveys to sheathing station
  → CR6-S1 picks sheathing sheet from magazine
  → CR6-S2 fastens sheathing to frame on sheathing table
  → Factory Rail robot performs inspection sweep over finished panel
  → Finished wall exits on outfeed conveyor (far right)

LAYOUT  (left → right along X-axis, front of line Y≈0)
  X:  -10   Raw material rack
  X:  -6    Framing station  (CR6-F1 base Y=-2, CR6-F2 base Y=+2)
  X:   0    Transfer conveyor between stations
  X:  +5    Sheathing station  (CR6-S1 base Y=-2, CR6-S2 base Y=+2)
  X:  +12   Outfeed / stacking
  Rail runs along Y=0, X=-2 to +13 (front of both stations)

DH PARAMETERS  (V6.1 validated — do not modify)
  D1=1.5  A2=2.5  A3=2.0  D6=0.5  MAX_REACH=5.0

PART OWNERSHIP CHAIN
  FRAMING:
    STUD_ON_RACK → STUD_HELD_F1 → STUD_ON_TABLE → FRAME_ASSEMBLING → FRAME_DONE
  TRANSFER:
    FRAME_DONE → FRAME_CONVEYING → FRAME_AT_SHEATHING
  SHEATHING:
    SHEET_ON_MAG → SHEET_HELD_S1 → SHEET_ON_FRAME → WALL_FASTENING → WALL_DONE
  OUTFEED:
    WALL_DONE → RAIL_INSPECTING → WALL_COMPLETE
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from collections import deque

# ═══════════════════════════════════════════════════════════════════════
# SECTION 1 — DH PARAMETERS  (V6.1 validated — do not modify)
# ═══════════════════════════════════════════════════════════════════════

D1, A2, A3, D6 = 1.5, 2.5, 2.0, 0.5
MAX_REACH  = A2 + A3 + D6       # 5.0
SAFE_REACH = MAX_REACH * 0.90   # 4.5

# ═══════════════════════════════════════════════════════════════════════
# SECTION 2 — WORLD CONSTANTS
# ═══════════════════════════════════════════════════════════════════════

DEFAULT_RPY = np.array([0.0, np.pi, 0.0])
DWELL_LIMIT = 14
ARRIVE_DIST = 0.30

# ── Station 1: Framing ──────────────────────────────────────────────
FRAME_TABLE_X  = -6.0
FRAME_TABLE_Y  =  0.0
FRAME_TABLE_Z  =  0.9
FRAME_TABLE_W  =  3.5
FRAME_TABLE_D  =  2.5

RACK_X  = -10.0
RACK_Y  =  -2.0
RACK_Z  =   0.9
RACK_SLOT_ZS = [RACK_Z + 0.15 * i for i in range(5)]

# CR6-F1: material handler (front)
RF1_BASE = np.array([FRAME_TABLE_X, -2.6, 0.0])
# CR6-F2: assembler/fastener (back)
RF2_BASE = np.array([FRAME_TABLE_X, +2.6, 0.0])

STUD_PLACE_POSITIONS = [
    np.array([FRAME_TABLE_X - 0.9,  FRAME_TABLE_Y, FRAME_TABLE_Z + 0.05]),
    np.array([FRAME_TABLE_X,        FRAME_TABLE_Y, FRAME_TABLE_Z + 0.05]),
    np.array([FRAME_TABLE_X + 0.9,  FRAME_TABLE_Y, FRAME_TABLE_Z + 0.05]),
]
TRACK_PLACE_POSITIONS = [
    np.array([FRAME_TABLE_X, FRAME_TABLE_Y - 0.9, FRAME_TABLE_Z + 0.05]),
    np.array([FRAME_TABLE_X, FRAME_TABLE_Y + 0.9, FRAME_TABLE_Z + 0.05]),
]

# Assembly sequence — (type, position_index)
ASSEMBLY_SEQ = [
    ("TRACK", 0), ("STUD", 0), ("STUD", 1), ("STUD", 2), ("TRACK", 1)
]

# ── Inter-station conveyor ──────────────────────────────────────────
CONV_X_START  = -4.5
CONV_X_END    =  3.5
CONV_Y        =   0.0
CONV_Z        =   0.85
CONV_SPEED    =   0.06  # units/frame when active

# ── Station 2: Sheathing ────────────────────────────────────────────
SHEATH_TABLE_X = 5.5
SHEATH_TABLE_Y = 0.0
SHEATH_TABLE_Z = 0.9
SHEATH_TABLE_W = 3.5
SHEATH_TABLE_D = 2.5

MAG_X  =  3.0
MAG_Y  =  -2.5
MAG_Z  =   0.9

RS1_BASE = np.array([SHEATH_TABLE_X, -2.8, 0.0])
RS2_BASE = np.array([SHEATH_TABLE_X, +2.8, 0.0])

SHEET_PLACE_POS = np.array([SHEATH_TABLE_X, SHEATH_TABLE_Y, SHEATH_TABLE_Z + 0.06])

# ── Station 3: Factory Rail ─────────────────────────────────────────
RAIL_X_START =  -2.0
RAIL_X_END   =  13.0
RAIL_Y       =   -1.5
RAIL_Z       =   0.0
RAIL_SPEED   =   0.07

INSPECT_PASSES = [
    np.array([SHEATH_TABLE_X - 1.5, SHEATH_TABLE_Y, 2.5]),
    np.array([SHEATH_TABLE_X,       SHEATH_TABLE_Y, 2.5]),
    np.array([SHEATH_TABLE_X + 1.5, SHEATH_TABLE_Y, 2.5]),
]

# ── Outfeed ─────────────────────────────────────────────────────────
OUTFEED_X_START = 10.0
OUTFEED_X_END   = 13.5
OUTFEED_Y       =  0.0
OUTFEED_Z       =  0.85

# Heights
LIFT_Z     = 2.2
SAFE_Z     = 3.0
PARK_Z     = 3.0

# ═══════════════════════════════════════════════════════════════════════
# SECTION 3 — MATH  (V6.1 validated)
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
        s=0.5/np.sqrt(t+1)
        return np.array([0.25/s,(R[2,1]-R[1,2])*s,(R[0,2]-R[2,0])*s,(R[1,0]-R[0,1])*s])
    elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s=2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2])
        return np.array([(R[2,1]-R[1,2])/s,0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s])
    elif R[1,1]>R[2,2]:
        s=2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2])
        return np.array([(R[0,2]-R[2,0])/s,(R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s])
    else:
        s=2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1])
        return np.array([(R[1,0]-R[0,1])/s,(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s])

def quat_to_R(q):
    q=q/np.linalg.norm(q); w,x,y,z=q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)  ],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)  ],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ])

def slerp(q0, q1, t):
    q0=q0/np.linalg.norm(q0); q1=q1/np.linalg.norm(q1)
    d=np.clip(np.dot(q0,q1),-1,1)
    if d<0: q1=-q1; d=-d
    if d>0.9995: return q0+t*(q1-q0)
    th=np.arccos(d)
    return (np.sin((1-t)*th)*q0 + np.sin(t*th)*q1)/np.sin(th)

# ═══════════════════════════════════════════════════════════════════════
# SECTION 4 — KINEMATICS  (V6.1 validated)
# ═══════════════════════════════════════════════════════════════════════

def fk(q):
    q1,q2,q3,q4,q5,q6 = q
    dh = [
        [0,  np.pi/2, D1, q1],
        [A2, 0,       0,  q2],
        [A3, 0,       0,  q3],
        [0, -np.pi/2, 0,  q4],
        [0,  np.pi/2, 0,  q5],
        [0,  0,       D6, q6],
    ]
    T=np.eye(4); pts=[T[:3,3].copy()]
    for row in dh:
        T=T@dh_transform(*row); pts.append(T[:3,3].copy())
    return pts

def ik(local_pos, R06):
    px,py,pz=local_pos; ap=R06[:,2]
    wx,wy,wz=px-D6*ap[0], py-D6*ap[1], pz-D6*ap[2]
    q1=np.arctan2(wy,wx); r=np.hypot(wx,wy); s=wz-D1
    d2=r*r+s*s
    if np.sqrt(d2)>MAX_REACH*0.99: return None
    c3=(d2-A2**2-A3**2)/(2*A2*A3)
    if abs(c3)>1: return None
    q3=np.arctan2(-np.sqrt(1-c3**2),c3)
    q2=np.arctan2(s,r)-np.arctan2(A3*np.sin(q3),A2+A3*np.cos(q3))
    T1=dh_transform(0,np.pi/2,D1,q1)
    T2=dh_transform(A2,0,0,q2)
    T3=dh_transform(A3,0,0,q3)
    R03=(T1@T2@T3)[:3,:3]; R36=R03.T@R06
    q5=np.arctan2(np.sqrt(R36[0,2]**2+R36[1,2]**2),R36[2,2])
    if abs(np.sin(q5))>1e-6:
        q4=np.arctan2(R36[1,2]/np.sin(q5),R36[0,2]/np.sin(q5))
        q6=np.arctan2(R36[2,1]/np.sin(q5),-R36[2,0]/np.sin(q5))
    else:
        q4=0.0; q6=np.arctan2(-R36[0,1],R36[1,1])
    return np.array([q1,q2,q3,q4,q5,q6])

def ik_world(world_pos, base, rpy=None):
    R=rpy_to_R(rpy if rpy is not None else DEFAULT_RPY)
    return ik(np.array(world_pos)-base, R)

def tcp_world(q, base):
    return fk(q)[-1] + base

def park_pose(base):
    R=rpy_to_R(DEFAULT_RPY)
    q=ik(np.array([0.0, 0.0, PARK_Z]), R)
    return q if q is not None else np.array([0.0, 0.8, -0.5, 0.0, 0.5, 0.0])

# ═══════════════════════════════════════════════════════════════════════
# SECTION 5 — ROBOT CLASS
# ═══════════════════════════════════════════════════════════════════════

def wp(pos, rpy=None):
    return (np.array(pos), (rpy if rpy is not None else DEFAULT_RPY).copy())

class Robot:
    """
    CR6 6-axis robot. Fixed base or rail-mounted (base provided externally).
    Executes waypoint sequences via Cartesian interp + SLERP.
    """
    def __init__(self, base, name, colors, rail_mounted=False):
        self.base         = np.array(base)
        self.name         = name
        self.colors       = colors
        self.rail_mounted = rail_mounted
        self.active       = False
        self.state        = "IDLE"
        self.trace        = deque(maxlen=300)
        self.q            = park_pose(base)
        self._wps         = []
        self._names       = []
        self._dwell_at    = set()
        self._idx         = 0
        self._t           = 0.0
        self._dwell       = 0

    def launch(self, waypoints, names, dwell_at, base_override=None):
        b = base_override if base_override is not None else self.base
        cur_tcp = tcp_world(self.q, b)
        start   = (cur_tcp, DEFAULT_RPY.copy())
        self._wps      = [start] + list(waypoints)
        self._names    = ["CURRENT"] + list(names)
        self._dwell_at = dwell_at
        self._idx=0; self._t=0.0; self._dwell=0
        self.active=True; self.state=names[0]

    def step(self, speed, base_override=None):
        b = base_override if base_override is not None else self.base
        if not self.active or len(self._wps)<2: return None
        self._t += speed; action=None
        if self._t >= 1.0:
            cur=self._names[self._idx]
            if cur in self._dwell_at and self._dwell < DWELL_LIMIT:
                self._dwell+=1; self._t=1.0
            else:
                self._dwell=0; self._t=0.0; action=cur
                self._idx+=1
                if self._idx >= len(self._wps)-1:
                    self.state="IDLE"; self.active=False; return action
                self.state=self._names[self._idx]
        i=self._idx
        p0,r0=self._wps[i]; p1,r1=self._wps[i+1]; t=self._t
        pos_t=np.array(p0)+t*(np.array(p1)-np.array(p0))
        q_t=slerp(R_to_quat(rpy_to_R(r0)),R_to_quat(rpy_to_R(r1)),t)
        R_t=quat_to_R(q_t)
        q_sol=ik(pos_t-b, R_t)
        if q_sol is not None: self.q=q_sol
        return action

    def tcp(self, base_override=None):
        b = base_override if base_override is not None else self.base
        return tcp_world(self.q, b)

    def joint_pts(self, base_override=None):
        b = base_override if base_override is not None else self.base
        return [p+b for p in fk(self.q)]

# ═══════════════════════════════════════════════════════════════════════
# SECTION 6 — FACTORY RAIL
# ═══════════════════════════════════════════════════════════════════════

class FactoryRail:
    def __init__(self):
        self.current_x   = SHEATH_TABLE_X  # start at sheathing station
        self.target_x    = SHEATH_TABLE_X
        self._direction  = 0
        self.state       = "IDLE"

    @property
    def robot_base(self):
        return np.array([self.current_x, RAIL_Y, RAIL_Z])

    def move_to(self, x):
        x=np.clip(x, RAIL_X_START, RAIL_X_END)
        if abs(self.current_x-x)<0.02: return
        self.target_x=x
        self._direction = 1.0 if x>self.current_x else -1.0
        self.state="TRAVELING"

    def update(self):
        if self.state=="IDLE": return True
        self.current_x += self._direction*RAIL_SPEED
        if self._direction>0 and self.current_x>=self.target_x:
            self.current_x=self.target_x; self.state="IDLE"; return True
        if self._direction<0 and self.current_x<=self.target_x:
            self.current_x=self.target_x; self.state="IDLE"; return True
        return False

    def at(self, x, tol=0.08):
        return abs(self.current_x-x)<tol

# ═══════════════════════════════════════════════════════════════════════
# SECTION 7 — WAYPOINT BUILDERS
# ═══════════════════════════════════════════════════════════════════════

# ── Station 1: Framing ──────────────────────────────────────────────

def build_f1_place_member(rack_pos, table_pos):
    """CR6-F1: rack pick → table place."""
    rp, tp = np.array(rack_pos), np.array(table_pos)
    wps = [
        wp([rp[0], rp[1], SAFE_Z]),
        wp([rp[0], rp[1], rp[2]+0.5]),
        wp(rp),
        wp([rp[0], rp[1], LIFT_Z]),
        wp([tp[0], tp[1], SAFE_Z]),
        wp([tp[0], tp[1], tp[2]+0.5]),
        wp(tp),
        wp([tp[0], tp[1], SAFE_Z]),
    ]
    names = ["RACK_APPROACH","RACK_DESCEND","RACK_PICK",
             "LIFT","TABLE_APPROACH","TABLE_DESCEND","TABLE_PLACE","RETURN"]
    return wps, names, {"RACK_PICK","TABLE_PLACE"}

def build_f2_fasten(pos):
    """CR6-F2: approach and fasten member at given position."""
    p = np.array(pos)
    wps = [
        wp([p[0], p[1], SAFE_Z]),
        wp([p[0], p[1], p[2]+0.5]),
        wp(p),
        wp([p[0], p[1], p[2]+0.5]),
        wp([p[0], p[1], SAFE_Z]),
    ]
    names = ["APPROACH","DESCEND","FASTEN","RETRACT","CLEAR"]
    return wps, names, {"FASTEN"}

# ── Station 2: Sheathing ────────────────────────────────────────────

def build_s1_pickup():
    """CR6-S1: sheet magazine → sheathing table."""
    mp = np.array([MAG_X, MAG_Y, MAG_Z])
    sp = SHEET_PLACE_POS.copy()
    wps = [
        wp([mp[0], mp[1], SAFE_Z]),
        wp([mp[0], mp[1], mp[2]+0.5]),
        wp(mp),
        wp([mp[0], mp[1], LIFT_Z]),
        wp([sp[0], sp[1], SAFE_Z]),
        wp([sp[0], sp[1], sp[2]+0.5]),
        wp(sp),
        wp([sp[0], sp[1], SAFE_Z]),
    ]
    names = ["MAG_APPROACH","MAG_DESCEND","MAG_PICK",
             "LIFT","TABLE_APPROACH","TABLE_DESCEND","TABLE_PLACE","RETURN"]
    return wps, names, {"MAG_PICK","TABLE_PLACE"}

def build_s2_fasten():
    """CR6-S2: fasten sheathing panel at multiple points."""
    pts = [
        np.array([SHEATH_TABLE_X-1.0, SHEATH_TABLE_Y-0.8, SHEATH_TABLE_Z+0.1]),
        np.array([SHEATH_TABLE_X+1.0, SHEATH_TABLE_Y-0.8, SHEATH_TABLE_Z+0.1]),
        np.array([SHEATH_TABLE_X,     SHEATH_TABLE_Y,     SHEATH_TABLE_Z+0.1]),
        np.array([SHEATH_TABLE_X-1.0, SHEATH_TABLE_Y+0.8, SHEATH_TABLE_Z+0.1]),
        np.array([SHEATH_TABLE_X+1.0, SHEATH_TABLE_Y+0.8, SHEATH_TABLE_Z+0.1]),
    ]
    wps=[]; names=[]
    wps.append(wp([pts[0][0], pts[0][1], SAFE_Z])); names.append("APPROACH")
    for i,p in enumerate(pts):
        wps.append(wp(p))
        names.append(f"FASTEN_{i+1}")
    wps.append(wp([pts[-1][0], pts[-1][1], SAFE_Z]))
    names.append("CLEAR")
    dwell_at = {f"FASTEN_{i+1}" for i in range(len(pts))}
    return wps, names, dwell_at

# ── Station 3: Rail inspection ──────────────────────────────────────

def build_rail_inspect(base_x):
    """Rail robot: sweep across finished panel for inspection."""
    b = np.array([base_x, RAIL_Y, 0.0])
    wps=[]
    names=[]
    for i, p in enumerate(INSPECT_PASSES):
        wps.append(wp([p[0], p[1], p[2]]))
        names.append(f"INSPECT_{i+1}")
    wps.append(wp([INSPECT_PASSES[-1][0], INSPECT_PASSES[-1][1], SAFE_Z]))
    names.append("INSPECT_DONE")
    return wps, names, {f"INSPECT_{i+1}" for i in range(len(INSPECT_PASSES))}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 8 — WORLD STATE MACHINE
# ═══════════════════════════════════════════════════════════════════════

class CellState:
    """Tracks production state across all three stations."""
    def __init__(self):
        # Framing station
        self.framing_seq_idx   = 0      # which member in ASSEMBLY_SEQ
        self.members_placed    = 0
        self.framing_complete  = False
        self.placed_positions  = []     # list of (mtype, pos) placed so far
        # Transfer conveyor
        self.frame_x           = FRAME_TABLE_X
        self.frame_conveying   = False
        self.frame_at_sheath   = False
        # Sheathing station
        self.sheet_placed      = False
        self.sheathing_active  = False
        self.sheathing_done    = False
        # Rail / outfeed
        self.rail_inspecting   = False
        self.wall_complete     = False
        self.outfeed_x         = OUTFEED_X_START
        self.outfeed_moving    = False
        # Cycle counter
        self.cycles            = 0


class World:
    def __init__(self):
        COLORS_A = ["#FF3333","#FF8800","#FFD700","#44DD44","#00CCFF","#4466FF"]
        COLORS_B = ["#CC00FF","#FF66CC","#FFFFFF","#00FFCC","#FF9900","#AAAAAA"]
        COLORS_S = ["#FF5500","#FFAA00","#FFEE44","#55FF55","#22DDFF","#8888FF"]
        COLORS_R = ["#FF8C00","#FFD700","#FFFF88","#88FF88","#88FFFF","#FF88FF"]

        self.rf1  = Robot(RF1_BASE,  "F1", COLORS_A)
        self.rf2  = Robot(RF2_BASE,  "F2", COLORS_B)
        self.rs1  = Robot(RS1_BASE,  "S1", COLORS_S)
        self.rs2  = Robot(RS2_BASE,  "S2", COLORS_B)
        self.rail = FactoryRail()
        self.rr   = Robot(self.rail.robot_base, "RR", COLORS_R, rail_mounted=True)

        self.state   = CellState()
        self.tick    = 0
        self._f1_launched = False
        self._f2_launched = False
        self._f1_placed   = False
        self._s1_launched = False
        self._s2_launched = False
        self._rr_launched = False
        self._rail_moved  = False

    # ── helpers ────────────────────────────────────────────────────

    def _get_rack_pos(self, seq_idx):
        """Alternating rack slots for studs/tracks."""
        return np.array([RACK_X, RACK_Y, RACK_SLOT_ZS[seq_idx % len(RACK_SLOT_ZS)]])

    def _get_table_pos(self, member_type, pos_idx):
        if member_type == "STUD":
            return STUD_PLACE_POSITIONS[pos_idx]
        else:
            return TRACK_PLACE_POSITIONS[pos_idx]

    # ── step ───────────────────────────────────────────────────────

    def step(self, speed):
        self.tick += 1
        s     = self.state
        rf1   = self.rf1
        rf2   = self.rf2
        rs1   = self.rs1
        rs2   = self.rs2
        rail  = self.rail
        rr    = self.rr
        base  = rail.robot_base

        # ── STATION 1: FRAMING ──────────────────────────────────────

        if not s.framing_complete:
            seq_idx = s.framing_seq_idx
            if seq_idx < len(ASSEMBLY_SEQ):
                mtype, pidx = ASSEMBLY_SEQ[seq_idx]
                rack_pos  = self._get_rack_pos(seq_idx)
                table_pos = self._get_table_pos(mtype, pidx)

                # CR6-F1: pick & place current member
                if not rf1.active and not self._f1_launched:
                    wps, names, dwell = build_f1_place_member(rack_pos, table_pos)
                    rf1.launch(wps, names, dwell)
                    self._f1_launched = True
                    self._f1_placed   = False   # reset place flag for this member

                if rf1.active:
                    rf1.step(speed)
                    # Proximity trigger: member appears the moment TCP descends
                    # within 0.4 units of the table position — during the approach,
                    # not on waypoint exit, so it snaps on while the robot is still there
                    if not self._f1_placed:
                        if np.linalg.norm(rf1.tcp() - table_pos) < 0.4:
                            s.placed_positions.append((mtype, table_pos.copy()))
                            self._f1_placed = True

                # CR6-F2: fasten after F1 places
                if not rf1.active and self._f1_launched and not rf2.active and not self._f2_launched:
                    wps, names, dwell = build_f2_fasten(table_pos)
                    rf2.launch(wps, names, dwell)
                    self._f2_launched = True

                if rf2.active:
                    rf2.step(speed)

                # Advance sequence after both complete
                if not rf1.active and not rf2.active and self._f1_launched and self._f2_launched:
                    s.framing_seq_idx += 1
                    s.members_placed  += 1
                    self._f1_launched  = False
                    self._f2_launched  = False
                    self._f1_placed    = False

                    if s.framing_seq_idx >= len(ASSEMBLY_SEQ):
                        s.framing_complete = True

        # ── TRANSFER CONVEYOR ───────────────────────────────────────

        if s.framing_complete and not s.frame_conveying and not s.frame_at_sheath:
            s.frame_conveying = True
            s.frame_x = FRAME_TABLE_X

        if s.frame_conveying:
            s.frame_x += CONV_SPEED
            if s.frame_x >= SHEATH_TABLE_X - 0.3:
                s.frame_x = SHEATH_TABLE_X - 0.3
                s.frame_conveying = False
                s.frame_at_sheath = True

        # ── STATION 2: SHEATHING ────────────────────────────────────

        if s.frame_at_sheath and not s.sheathing_done:

            # S1: pick sheet from magazine
            if not rs1.active and not self._s1_launched and not s.sheet_placed:
                wps, names, dwell = build_s1_pickup()
                rs1.launch(wps, names, dwell)
                self._s1_launched = True

            if rs1.active:
                rs1.step(speed)

            if not rs1.active and self._s1_launched and not s.sheet_placed:
                s.sheet_placed     = True
                s.sheathing_active = True

            # S2: fasten once sheet is placed
            if s.sheet_placed and not rs2.active and not self._s2_launched:
                wps, names, dwell = build_s2_fasten()
                rs2.launch(wps, names, dwell)
                self._s2_launched = True

            if rs2.active:
                rs2.step(speed)

            if not rs2.active and self._s2_launched and s.sheet_placed and not s.sheathing_done:
                s.sheathing_done = True

        # ── STATION 3: RAIL INSPECTION ──────────────────────────────

        if s.sheathing_done and not s.wall_complete:

            # Move rail to sheathing station
            if not self._rail_moved and rail.state == "IDLE" and not rail.at(SHEATH_TABLE_X):
                rail.move_to(SHEATH_TABLE_X)
                self._rail_moved = True

            rail.update()

            # Hold park pose during transit
            if rail.state == "TRAVELING":
                rr.q = park_pose(base)

            # Launch inspection when rail arrives
            if rail.state == "IDLE" and rail.at(SHEATH_TABLE_X):
                if not rr.active and not self._rr_launched:
                    wps, names, dwell = build_rail_inspect(base[0])
                    rr.launch(wps, names, dwell, base_override=base)
                    self._rr_launched = True

            if rr.active:
                rr.step(speed, base_override=base)

            if not rr.active and self._rr_launched:
                s.wall_complete = True
                s.outfeed_moving = True

        # ── OUTFEED ─────────────────────────────────────────────────

        if s.outfeed_moving:
            s.outfeed_x += CONV_SPEED * 0.8
            if s.outfeed_x >= OUTFEED_X_END:
                s.outfeed_x = OUTFEED_X_END
                s.outfeed_moving = False
                # Cycle reset
                s.cycles += 1
                self._reset_cycle()

    def _reset_cycle(self):
        s = self.state
        s.framing_seq_idx  = 0
        s.members_placed   = 0
        s.framing_complete = False
        s.placed_positions = []
        s.frame_x          = FRAME_TABLE_X
        s.frame_conveying  = False
        s.frame_at_sheath  = False
        s.sheet_placed     = False
        s.sheathing_active = False
        s.sheathing_done   = False
        s.rail_inspecting  = False
        s.wall_complete    = False
        s.outfeed_x        = OUTFEED_X_START
        s.outfeed_moving   = False
        self._f1_launched  = False
        self._f2_launched  = False
        self._f1_placed    = False
        self._s1_launched  = False
        self._s2_launched  = False
        self._rr_launched  = False
        self._rail_moved   = False

# ═══════════════════════════════════════════════════════════════════════

    def snapshot(self):
        """Added for the Phase 3B pyvista migration -- read-only render view, no sim logic."""
        s = self.state
        rail_base = self.rail.robot_base
        robots = {name: {"state": r.state, "pts": r.joint_pts()} for name, r in
                  [("F1", self.rf1), ("F2", self.rf2), ("S1", self.rs1), ("S2", self.rs2)]}
        robots["RR"] = {"state": self.rr.state, "pts": self.rr.joint_pts(base_override=rail_base)}
        return {
            "tick": self.tick,
            "cycles": s.cycles,
            "framing_complete": s.framing_complete,
            "sheathing_done": s.sheathing_done,
            "wall_complete": s.wall_complete,
            "rail_x": self.rail.current_x if hasattr(self.rail, "current_x") else rail_base[0],
            "robots": robots,
            "placed_positions": list(s.placed_positions),
        }
