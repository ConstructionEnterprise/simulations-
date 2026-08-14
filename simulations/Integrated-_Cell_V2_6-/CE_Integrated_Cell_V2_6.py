"""
CONSTRUCTION ENTERPRISES — CHAPPELL ROBOTICS
CE_Integrated_Cell  V2.0

INTEGRATED MODULAR WALL MANUFACTURING CELL — TABLE_JIG ARCHITECTURE
  TABLE_JIG_FIXED   — LGS framing assembly (Dual CR6: F1 material handler + F2 fastener)
  TABLE_JIG_ROLLER  — Servo roller transfer zone (no robots, no assembly)
  TABLE_JIG_TILT    — Sheathing + inspection + tilt to vertical + overhead crane outfeed

MANUFACTURING FLOW
  Raw studs/tracks loaded into material rack
  → CR6-F1 picks members, places on TABLE_JIG_FIXED (proximity trigger)
  → CR6-F2 fastens each member immediately on placement
  → Completed frame indexes through TABLE_JIG_ROLLER to TABLE_JIG_TILT
  → CR6-S1 picks sheet from magazine, places on frame
  → CR6-S2 fastens sheathing (5-point pattern)
  → TABLE_JIG_TILT rotates 0° → 90° (wall goes vertical)
  → Overhead crane travels, lowers, hooks, lifts finished wall
  → Cycle resets

LAYOUT  (left → right along X-axis)
  X: -10   Raw material rack
  X:  -6   TABLE_JIG_FIXED  (CR6-F1 @ Y=-2.6, CR6-F2 @ Y=+2.6)
  X:  -1   TABLE_JIG_ROLLER (servo roller transfer zone)
  X:  +5   TABLE_JIG_TILT   (CR6-S1 @ Y=-2.8, CR6-S2 @ Y=+2.8)
  X: +10   Overhead crane park position

DH PARAMETERS  (V6.1 validated — do not modify)
  D1=1.5  A2=2.5  A3=2.0  D6=0.5  MAX_REACH=5.0

SHARED DATUM
  X-axis centerline at Y=0 runs through all three table zones
  Locating pins at fixed X positions on FIXED and TILT tables
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from collections import deque

# ═══════════════════════════════════════════════════════════════════════
# SECTION 1 — DH PARAMETERS  (V6.1 validated — do not modify)
# ═══════════════════════════════════════════════════════════════════════

D1, A2, A3, D6 = 1.5, 2.5, 2.0, 0.5
MAX_REACH  = A2 + A3 + D6      # 5.0
SAFE_REACH = MAX_REACH * 0.90  # 4.5

# ═══════════════════════════════════════════════════════════════════════
# SECTION 2 — WORLD CONSTANTS
# ═══════════════════════════════════════════════════════════════════════

DEFAULT_RPY = np.array([0.0, np.pi, 0.0])
DWELL_LIMIT = 14

# ── Raw material rack ────────────────────────────────────────────────
RACK_X       = -10.0
RACK_Y       =  -2.0
RACK_Z       =   0.9
RACK_SLOT_ZS = [RACK_Z + 0.15 * i for i in range(5)]

# ── TABLE_JIG_FIXED ──────────────────────────────────────────────────
FIXED_CX   = -6.0
FIXED_CY   =  0.0
FIXED_Z    =  0.9
FIXED_W    =  4.0   # length along X
FIXED_D    =  2.8   # width along Y

RF1_BASE = np.array([FIXED_CX, -2.6, 0.0])   # material handler
RF2_BASE = np.array([FIXED_CX, +2.6, 0.0])   # fastener

STUD_PLACE_POSITIONS = [
    np.array([FIXED_CX - 1.0, FIXED_CY, FIXED_Z + 0.05]),
    np.array([FIXED_CX,       FIXED_CY, FIXED_Z + 0.05]),
    np.array([FIXED_CX + 1.0, FIXED_CY, FIXED_Z + 0.05]),
]
TRACK_PLACE_POSITIONS = [
    np.array([FIXED_CX, FIXED_CY - 1.0, FIXED_Z + 0.05]),
    np.array([FIXED_CX, FIXED_CY + 1.0, FIXED_Z + 0.05]),
]
ASSEMBLY_SEQ = [("TRACK", 0), ("STUD", 0), ("STUD", 1), ("STUD", 2), ("TRACK", 1)]

# ── TABLE_JIG_ROLLER ─────────────────────────────────────────────────
ROLLER_X_START = -3.8
ROLLER_X_END   =  2.8
ROLLER_Y       =  0.0
ROLLER_Z       =  0.85
ROLLER_SPEED   =  0.07   # units/frame

# ── TABLE_JIG_TILT ───────────────────────────────────────────────────
TILT_CX    =  5.5
TILT_CY    =  0.0
TILT_Z     =  0.9
TILT_W     =  4.0
TILT_D     =  2.8
TILT_PIVOT_X = 3.2   # pivot edge (left/infeed side of tilt table)
TILT_SPEED   = 0.8   # degrees per frame at speed=1.0

RS1_BASE = np.array([TILT_CX, -2.8, 0.0])
RS2_BASE = np.array([TILT_CX, +2.8, 0.0])

MAG_X  =  3.2
MAG_Y  = -2.8
MAG_Z  =  0.9

SHEET_PLACE_POS = np.array([TILT_CX, TILT_CY, TILT_Z + 0.06])

FASTEN_PTS = [
    np.array([TILT_CX - 1.2, TILT_CY - 1.0, TILT_Z + 0.12]),
    np.array([TILT_CX + 1.2, TILT_CY - 1.0, TILT_Z + 0.12]),
    np.array([TILT_CX,       TILT_CY,        TILT_Z + 0.12]),
    np.array([TILT_CX - 1.2, TILT_CY + 1.0, TILT_Z + 0.12]),
    np.array([TILT_CX + 1.2, TILT_CY + 1.0, TILT_Z + 0.12]),
]

# ── Overhead crane ───────────────────────────────────────────────────
CRANE_PARK_X   = 11.0
CRANE_ACTIVE_X =  5.5
CRANE_BEAM_Y   =  0.0
CRANE_BEAM_Z   =  5.5
CRANE_SPEED    =  0.06   # travel speed
CRANE_LOWER_Z  =  2.2   # hook lowered position
CRANE_LIFT_Z   =  5.2   # hook fully raised
CRANE_HOOK_SPEED = 0.04

# Heights
LIFT_Z  = 2.2
SAFE_Z  = 3.0
PARK_Z  = 3.0

# ── CE aesthetics ────────────────────────────────────────────────────
CE_GOLD  = "#CC6600"
CE_BLACK = "#1A1A1A"

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
    def __init__(self, base, name, colors):
        self.base      = np.array(base)
        self.name      = name
        self.colors    = colors
        self.active    = False
        self.state     = "IDLE"
        self.trace     = deque(maxlen=200)
        self.q         = park_pose(base)
        self._wps      = []
        self._names    = []
        self._dwell_at = set()
        self._idx      = 0
        self._t        = 0.0
        self._dwell    = 0

    def launch(self, waypoints, names, dwell_at):
        cur_tcp = tcp_world(self.q, self.base)
        self._wps      = [(cur_tcp, DEFAULT_RPY.copy())] + list(waypoints)
        self._names    = ["CURRENT"] + list(names)
        self._dwell_at = dwell_at
        self._idx=0; self._t=0.0; self._dwell=0
        self.active=True; self.state=names[0]

    def step(self, speed):
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
        qt=slerp(R_to_quat(rpy_to_R(r0)),R_to_quat(rpy_to_R(r1)),t)
        Rt=quat_to_R(qt)
        qs=ik(pos_t-self.base, Rt)
        if qs is not None: self.q=qs
        return action

    def tcp(self):
        return tcp_world(self.q, self.base)

    def joint_pts(self):
        return [p+self.base for p in fk(self.q)]

# ═══════════════════════════════════════════════════════════════════════
# SECTION 6 — OVERHEAD CRANE
# ═══════════════════════════════════════════════════════════════════════

class OverheadCrane:
    """
    Crane beam travels along X-axis at fixed Y=0, Z=CRANE_BEAM_Z.
    Hook lowers/raises along Z from beam.
    States: PARKED → TRAVELING → LOWERING → HOOKED → LIFTING → CLEARING → PARKED
    """
    def __init__(self):
        self.beam_x    = CRANE_PARK_X
        self.hook_z    = CRANE_BEAM_Z
        self.state     = "PARKED"
        self._target_x = CRANE_PARK_X
        self._target_z = CRANE_BEAM_Z

    def activate(self):
        if self.state == "PARKED":
            self._target_x = CRANE_ACTIVE_X
            self.state = "TRAVELING"

    def update(self):
        if self.state == "TRAVELING":
            dx = self._target_x - self.beam_x
            if abs(dx) < CRANE_SPEED:
                self.beam_x = self._target_x
                self.state = "LOWERING"
                self._target_z = CRANE_LOWER_Z
            else:
                self.beam_x += np.sign(dx) * CRANE_SPEED

        elif self.state == "LOWERING":
            dz = self._target_z - self.hook_z
            if abs(dz) < CRANE_HOOK_SPEED:
                self.hook_z = self._target_z
                self.state = "HOOKED"
            else:
                self.hook_z += np.sign(dz) * CRANE_HOOK_SPEED

        elif self.state == "HOOKED":
            # Brief dwell then lift
            self._dwell = getattr(self, '_dwell', 0) + 1
            if self._dwell > 20:
                self._dwell = 0
                self.state = "LIFTING"
                self._target_z = CRANE_LIFT_Z

        elif self.state == "LIFTING":
            dz = self._target_z - self.hook_z
            if abs(dz) < CRANE_HOOK_SPEED:
                self.hook_z = self._target_z
                self.state = "CLEARING"
                self._target_x = CRANE_PARK_X
            else:
                self.hook_z += np.sign(dz) * CRANE_HOOK_SPEED

        elif self.state == "CLEARING":
            dx = self._target_x - self.beam_x
            if abs(dx) < CRANE_SPEED:
                self.beam_x = self._target_x
                self.hook_z = CRANE_BEAM_Z
                self.state = "PARKED"
                return True   # cycle complete signal
            else:
                self.beam_x += np.sign(dx) * CRANE_SPEED
        return False

    @property
    def hook_pos(self):
        return np.array([self.beam_x, CRANE_BEAM_Y, self.hook_z])

# ═══════════════════════════════════════════════════════════════════════
# SECTION 7 — WAYPOINT BUILDERS
# ═══════════════════════════════════════════════════════════════════════

def build_f1_place(rack_pos, table_pos):
    rp, tp = np.array(rack_pos), np.array(table_pos)
    wps = [
        wp([rp[0], rp[1], SAFE_Z]),
        wp([rp[0], rp[1], rp[2]+0.45]),
        wp(rp),
        wp([rp[0], rp[1], LIFT_Z]),
        wp([tp[0], tp[1], SAFE_Z]),
        wp([tp[0], tp[1], tp[2]+0.45]),
        wp(tp),
        wp([tp[0], tp[1], SAFE_Z]),
    ]
    names = ["RACK_APPROACH","RACK_DESCEND","RACK_PICK",
             "LIFT","TABLE_APPROACH","TABLE_DESCEND","TABLE_PLACE","RETURN"]
    return wps, names, {"RACK_PICK","TABLE_PLACE"}

def build_f2_fasten(pos):
    p = np.array(pos)
    wps = [
        wp([p[0], p[1], SAFE_Z]),
        wp([p[0], p[1], p[2]+0.4]),
        wp(p),
        wp([p[0], p[1], p[2]+0.4]),
        wp([p[0], p[1], SAFE_Z]),
    ]
    names = ["APPROACH","DESCEND","FASTEN","RETRACT","CLEAR"]
    return wps, names, {"FASTEN"}

def build_s1_pickup():
    mp = np.array([MAG_X, MAG_Y, MAG_Z])
    sp = SHEET_PLACE_POS.copy()
    wps = [
        wp([mp[0], mp[1], SAFE_Z]),
        wp([mp[0], mp[1], mp[2]+0.4]),
        wp(mp),
        wp([mp[0], mp[1], LIFT_Z]),
        wp([sp[0], sp[1], SAFE_Z]),
        wp([sp[0], sp[1], sp[2]+0.4]),
        wp(sp),
        wp([sp[0], sp[1], SAFE_Z]),
    ]
    names = ["MAG_APPROACH","MAG_DESCEND","MAG_PICK",
             "LIFT","TABLE_APPROACH","TABLE_DESCEND","TABLE_PLACE","RETURN"]
    return wps, names, {"MAG_PICK","TABLE_PLACE"}

def build_s2_fasten():
    wps = [wp([FASTEN_PTS[0][0], FASTEN_PTS[0][1], SAFE_Z])]
    names = ["APPROACH"]
    for i, p in enumerate(FASTEN_PTS):
        wps.append(wp(p))
        names.append(f"FASTEN_{i+1}")
    wps.append(wp([FASTEN_PTS[-1][0], FASTEN_PTS[-1][1], SAFE_Z]))
    names.append("CLEAR")
    return wps, names, {f"FASTEN_{i+1}" for i in range(len(FASTEN_PTS))}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 8 — CELL STATE MACHINE
# ═══════════════════════════════════════════════════════════════════════

class CellState:
    def __init__(self):
        # ── Station 1: TABLE_JIG_FIXED ──
        self.framing_seq_idx  = 0
        self.framing_complete = False
        self.placed_positions = []      # (mtype, pos) — drawn each frame

        # ── TABLE_JIG_ROLLER transfer ──
        self.frame_x          = FIXED_CX
        self.frame_conveying  = False
        self.frame_at_tilt    = False

        # ── Station 2: TABLE_JIG_TILT ──
        self.sheet_placed     = False
        self.sheathing_done   = False

        # ── Tilt sequence ──
        self.tilt_angle       = 0.0     # degrees: 0=flat, 90=vertical
        self.tilt_state       = "FLAT"  # FLAT|TILTING|VERTICAL|CRANE|RESET
        self.fixture_locked   = False

        # ── Crane ──
        self.crane_active     = False
        self.wall_lifted      = False

        # ── Counters ──
        self.cycles           = 0


class World:
    def __init__(self):
        CA = ["#FF3333","#FF8800","#FFD700","#44DD44","#00CCFF","#4466FF"]
        CB = ["#CC00FF","#FF66CC","#FFFFFF","#00FFCC","#FF9900","#AAAAAA"]
        CS = ["#FF5500","#FFAA00","#FFEE44","#55FF55","#22DDFF","#8888FF"]
        CT = ["#FF8C00","#FFD700","#FFFF88","#88FF88","#88FFFF","#FF88FF"]

        self.rf1   = Robot(RF1_BASE,  "F1", CA)
        self.rf2   = Robot(RF2_BASE,  "F2", CB)
        self.rs1   = Robot(RS1_BASE,  "S1", CS)
        self.rs2   = Robot(RS2_BASE,  "S2", CT)
        self.crane = OverheadCrane()
        self.state = CellState()
        self.tick  = 0

        self._f1_launched = False
        self._f2_launched = False
        self._f1_placed   = False
        self._s1_launched = False
        self._s2_launched = False

    def _rack_pos(self, idx):
        return np.array([RACK_X, RACK_Y, RACK_SLOT_ZS[idx % len(RACK_SLOT_ZS)]])

    def _table_pos(self, mtype, pidx):
        return STUD_PLACE_POSITIONS[pidx] if mtype == "STUD" else TRACK_PLACE_POSITIONS[pidx]

    def step(self, speed):
        self.tick += 1
        s = self.state

        # ── STATION 1: TABLE_JIG_FIXED ──────────────────────────────
        if not s.framing_complete:
            idx = s.framing_seq_idx
            if idx < len(ASSEMBLY_SEQ):
                mtype, pidx = ASSEMBLY_SEQ[idx]
                rack_pos  = self._rack_pos(idx)
                table_pos = self._table_pos(mtype, pidx)

                if not self.rf1.active and not self._f1_launched:
                    wps, names, dwell = build_f1_place(rack_pos, table_pos)
                    self.rf1.launch(wps, names, dwell)
                    self._f1_launched = True
                    self._f1_placed   = False

                if self.rf1.active:
                    self.rf1.step(speed)
                    # Proximity trigger — member appears when F1 TCP near table
                    if not self._f1_placed:
                        if np.linalg.norm(self.rf1.tcp() - table_pos) < 0.4:
                            s.placed_positions.append((mtype, table_pos.copy()))
                            self._f1_placed = True

                if not self.rf1.active and self._f1_launched and \
                   not self.rf2.active and not self._f2_launched:
                    wps, names, dwell = build_f2_fasten(table_pos)
                    self.rf2.launch(wps, names, dwell)
                    self._f2_launched = True

                if self.rf2.active:
                    self.rf2.step(speed)

                if not self.rf1.active and not self.rf2.active and \
                   self._f1_launched and self._f2_launched:
                    s.framing_seq_idx += 1
                    self._f1_launched  = False
                    self._f2_launched  = False
                    self._f1_placed    = False
                    if s.framing_seq_idx >= len(ASSEMBLY_SEQ):
                        s.framing_complete = True

        # ── TABLE_JIG_ROLLER TRANSFER ────────────────────────────────
        if s.framing_complete and not s.frame_conveying and not s.frame_at_tilt:
            s.frame_conveying = True
            s.frame_x = FIXED_CX

        if s.frame_conveying:
            s.frame_x += ROLLER_SPEED
            if s.frame_x >= TILT_CX - 0.3:
                s.frame_x = TILT_CX - 0.3
                s.frame_conveying = False
                s.frame_at_tilt   = True

        # ── STATION 2: TABLE_JIG_TILT — SHEATHING ───────────────────
        if s.frame_at_tilt and not s.sheathing_done:

            if not self.rs1.active and not self._s1_launched and not s.sheet_placed:
                wps, names, dwell = build_s1_pickup()
                self.rs1.launch(wps, names, dwell)
                self._s1_launched = True

            if self.rs1.active:
                self.rs1.step(speed)

            if not self.rs1.active and self._s1_launched and not s.sheet_placed:
                s.sheet_placed = True

            if s.sheet_placed and not self.rs2.active and not self._s2_launched:
                wps, names, dwell = build_s2_fasten()
                self.rs2.launch(wps, names, dwell)
                self._s2_launched = True

            if self.rs2.active:
                self.rs2.step(speed)

            if not self.rs2.active and self._s2_launched and s.sheet_placed:
                s.sheathing_done  = True
                s.fixture_locked  = True
                s.tilt_state      = "TILTING"

        # ── TILT SEQUENCE ────────────────────────────────────────────
        if s.tilt_state == "TILTING":
            s.tilt_angle += TILT_SPEED * speed * 60.0
            if s.tilt_angle >= 60.0:
                s.tilt_angle = 60.0
                s.tilt_state = "VERTICAL"

        if s.tilt_state == "VERTICAL" and not s.crane_active:
            self.crane.activate()
            s.crane_active = True

        # ── OVERHEAD CRANE ───────────────────────────────────────────
        if s.crane_active:
            done = self.crane.update()
            if done:
                s.wall_lifted = True
                s.crane_active = False
                s.cycles += 1
                self._reset_cycle()

    def _reset_cycle(self):
        s = self.state
        s.framing_seq_idx  = 0
        s.framing_complete = False
        s.placed_positions = []
        s.frame_x          = FIXED_CX
        s.frame_conveying  = False
        s.frame_at_tilt    = False
        s.sheet_placed     = False
        s.sheathing_done   = False
        s.tilt_angle       = 0.0
        s.tilt_state       = "FLAT"
        s.fixture_locked   = False
        s.crane_active     = False
        s.wall_lifted      = False
        self._f1_launched  = False
        self._f2_launched  = False
        self._f1_placed    = False
        self._s1_launched  = False
        self._s2_launched  = False

# ═══════════════════════════════════════════════════════════════════════
# SECTION 9 — RENDERING
# ═══════════════════════════════════════════════════════════════════════

def draw_floor(ax):
    xs = np.array([[-13, 14], [-13, 14]])
    ys = np.array([[-7,  -7], [ 7,   7]])
    ax.plot_surface(xs, ys, np.zeros_like(xs), color="#ECECEC", alpha=0.5)

def draw_table(ax, cx, cy, w, d, z, color="#5A5A5A", label=""):
    hw, hd = w/2, d/2
    xs = np.array([[cx-hw, cx+hw], [cx-hw, cx+hw]])
    ys = np.array([[cy-hd, cy-hd], [cy+hd, cy+hd]])
    zs = np.full_like(xs, z)
    ax.plot_surface(xs, ys, zs, color=color, alpha=0.55)
    ax.plot([cx-hw,cx+hw,cx+hw,cx-hw,cx-hw],
            [cy-hd,cy-hd,cy+hd,cy+hd,cy-hd],
            [z,z,z,z,z], color=CE_GOLD, lw=1.5, alpha=0.9)
    for lx in [cx-hw, cx+hw]:
        for ly in [cy-hd, cy+hd]:
            ax.plot([lx,lx],[ly,ly],[0,z], color="#333333", lw=2)
    if label:
        ax.text(cx, cy, z+0.14, label, fontsize=6, color="white",
                family="monospace", ha="center", fontweight="bold")

def draw_roller_zone(ax):
    y0, y1 = -1.2, 1.2
    z = ROLLER_Z
    ax.plot([ROLLER_X_START, ROLLER_X_END],[y0,y0],[z,z], color=CE_GOLD, lw=2)
    ax.plot([ROLLER_X_START, ROLLER_X_END],[y1,y1],[z,z], color=CE_GOLD, lw=2)
    for rx in np.linspace(ROLLER_X_START+0.25, ROLLER_X_END-0.25,
                          int((ROLLER_X_END-ROLLER_X_START)*2.2)):
        ax.plot([rx,rx],[y0,y1],[z+0.02,z+0.02], color="#888888", lw=2.5, alpha=0.7)
    ax.text((ROLLER_X_START+ROLLER_X_END)/2, 0, z+0.18,
            "TABLE_JIG_ROLLER", fontsize=5.5, color=CE_GOLD,
            family="monospace", ha="center")

def draw_datum_line(ax):
    """Shared X-axis centerline — the datum that runs through all three tables."""
    ax.plot([-8.5, 9.0], [0, 0], [FIXED_Z+0.01]*2,
            color="#0088FF", lw=1.5, linestyle="--", alpha=0.6)
    ax.text(9.2, 0, FIXED_Z+0.15, "X DATUM", fontsize=5,
            color="#0088FF", family="monospace", alpha=0.7)

def draw_locating_pins(ax, cx, z):
    """Datum locating pins on fixed and tilt tables."""
    for dx, dy in [(-1.6, -1.1), (-1.6, 1.1), (1.6, -1.1), (1.6, 1.1)]:
        ax.plot([cx+dx, cx+dx], [dy, dy], [z, z+0.25],
                color="#FFDD00", lw=3, solid_capstyle="round")

def draw_tilt_table_assembly(ax, tilt_angle):
    """
    TABLE_JIG_TILT drawn as a single rigid body.
    Every element — surface, frame edges, legs, locating pins —
    is transformed through the same rot() as the wall panel.
    Pivot axis: Y-direction line at (TILT_PIVOT_X, *, TILT_Z).
    At tilt_angle=0: table is flat (identical to draw_table).
    At tilt_angle=60: far edge has lifted, near edge stays at pivot.
    """
    ang = np.radians(tilt_angle)
    PL  = TILT_W          # table length  = 4.0
    PW  = TILT_D / 2.0    # table half-width = 1.4
    TH  = TILT_Z          # table top surface Z when flat

    def rot(x_local, cy, z_local=0.0):
        """
        Rigid rotation about hinge at (TILT_PIVOT_X, cy, Z=0) — floor level.
        x_local: distance along table surface from hinge (0=hinge, PL=far edge).
        z_local: perpendicular offset from hinge plane.
                 TH   = table surface (default)
                 TH+h = above surface by h
                 0    = at hinge plane
                 -ve  = below hinge (leg foot, into floor)
        Formula: pure 2D rotation in X-Z, pivot at world origin of hinge.
        wx = TILT_PIVOT_X + x_local*cos - (TH+z_local)*sin
        wz =                x_local*sin + (TH+z_local)*cos
        At ang=0: wx = TILT_PIVOT_X + x_local, wz = TH+z_local  (correct flat position)
        At ang>0: entire assembly rotates as rigid body about floor-level hinge.
        """
        offset = TH + z_local   # total perpendicular distance from hinge
        wx = TILT_PIVOT_X + x_local * np.cos(ang) - offset * np.sin(ang)
        wz =                x_local * np.sin(ang) + offset * np.cos(ang)
        return (wx, cy, wz)

    # ── Table top surface — 4 corners as Poly3DCollection ──
    s0 = rot(0.0, -PW)
    s1 = rot(PL,  -PW)
    s2 = rot(PL,  +PW)
    s3 = rot(0.0, +PW)
    surf = Poly3DCollection([[s0, s1, s2, s3]],
                             alpha=0.60, facecolor="#4A2A1A", edgecolor=CE_GOLD,
                             linewidth=1.5)
    ax.add_collection3d(surf)

    # ── Table frame edges (perimeter lines) ──
    for y in [-PW, +PW]:
        p0 = rot(0.0, y); p1 = rot(PL, y)
        ax.plot([p0[0],p1[0]], [p0[1],p1[1]], [p0[2],p1[2]],
                color=CE_GOLD, lw=1.5, alpha=0.9)
    for x in [0.0, PL]:
        p0 = rot(x, -PW); p1 = rot(x, +PW)
        ax.plot([p0[0],p1[0]], [p0[1],p1[1]], [p0[2],p1[2]],
                color=CE_GOLD, lw=1.5, alpha=0.9)

    # ── Legs — four corners, extend downward from table surface ──
    # z_local goes from 0 (surface) to -TH (floor)
    for xl, yl in [(0.0,-PW),(PL,-PW),(PL,+PW),(0.0,+PW)]:
        top = rot(xl, yl, 0.0)
        bot = rot(xl, yl, -TH)
        ax.plot([top[0],bot[0]], [top[1],bot[1]], [top[2],bot[2]],
                color="#333333", lw=2)

    # ── Locating pins — stand proud of surface ──
    for px_frac, py in [(0.4,-PW*0.78),(0.4,+PW*0.78),(0.6,-PW*0.78),(0.6,+PW*0.78)]:
        base = rot(px_frac * PL, py, 0.0)
        tip  = rot(px_frac * PL, py, 0.25)
        ax.plot([base[0],tip[0]], [base[1],tip[1]], [base[2],tip[2]],
                color="#FFDD00", lw=3, solid_capstyle="round")

    # ── Label — only show when mostly flat ──
    if tilt_angle < 40.0:
        mid = rot(PL / 2.0, 0.0, 0.12)
        ax.text(mid[0], mid[1], mid[2], "TABLE_JIG_TILT",
                fontsize=6, color="white", family="monospace",
                ha="center", fontweight="bold")


def draw_rack(ax):
    rx, ry = RACK_X, RACK_Y
    hw, hd = 1.3, 0.45
    for x in [rx-hw, rx+hw]:
        ax.plot([x,x],[ry-hd,ry-hd],[0,1.9], color="#333333", lw=3)
        ax.plot([x,x],[ry+hd,ry+hd],[0,1.9], color="#333333", lw=3)
    for z in RACK_SLOT_ZS:
        xs = np.array([[rx-hw,rx+hw],[rx-hw,rx+hw]])
        ys = np.array([[ry-hd,ry-hd],[ry+hd,ry+hd]])
        ax.plot_surface(xs, ys, np.full_like(xs, z), color="#8B7355", alpha=0.55)
    ax.text(rx, ry, 2.05, "RAW\nMATERIAL\nRACK", fontsize=5.5,
            color=CE_GOLD, family="monospace", ha="center", fontweight="bold")

def draw_magazine(ax):
    mx, my = MAG_X, MAG_Y
    hw, hd = 1.4, 0.5
    for z in [0.35, 0.6, 0.82, 0.98]:
        xs = np.array([[mx-hw,mx+hw],[mx-hw,mx+hw]])
        ys = np.array([[my-hd,my-hd],[my+hd,my+hd]])
        ax.plot_surface(xs, ys, np.full_like(xs, z), color="#D2A679", alpha=0.6)
    for x in [mx-hw, mx+hw]:
        ax.plot([x,x],[my-hd,my-hd],[0,1.2], color="#444", lw=2.5)
        ax.plot([x,x],[my+hd,my+hd],[0,1.2], color="#444", lw=2.5)
    ax.text(mx, my, 1.38, "SHEET\nMAGAZINE", fontsize=5.5,
            color=CE_GOLD, family="monospace", ha="center", fontweight="bold")

def draw_crane(ax, crane, wall_lifted, tilt_angle):
    """Overhead crane beam + hook. Wall travels with hook when lifting."""
    bx = crane.beam_x
    hz = crane.hook_z

    # Beam
    ax.plot([ROLLER_X_START, CRANE_PARK_X+1.5],
            [CRANE_BEAM_Y, CRANE_BEAM_Y],
            [CRANE_BEAM_Z+0.12]*2, color=CE_GOLD, lw=10,
            solid_capstyle="butt", alpha=0.9)

    # Trolley on beam
    for dy in [-0.3, 0.3]:
        ax.plot([bx-0.4, bx+0.4],[dy,dy],
                [CRANE_BEAM_Z+0.05]*2, color=CE_BLACK, lw=5)
    ax.plot([bx-0.4, bx+0.4],[CRANE_BEAM_Y-0.3, CRANE_BEAM_Y+0.3],
            [CRANE_BEAM_Z+0.05]*2, color=CE_BLACK, lw=4)

    # Hook wire
    ax.plot([bx, bx],[CRANE_BEAM_Y, CRANE_BEAM_Y],
            [CRANE_BEAM_Z, hz], color="#888888", lw=1.5, linestyle="-")

    # Hook
    ax.scatter(bx, CRANE_BEAM_Y, hz,
               color=CE_GOLD, s=80, marker="v", zorder=8)

    # ── Wall panel: animates from tilt angle (60°) to vertical (90°) ──
    # HOOKED:   panel at 60°  (just left the tilt table)
    # LIFTING:  panel sweeps 60° → 90° as hook rises  (t = 0→1)
    # CLEARING: panel at 90° (fully vertical, travelling to park)
    if crane.state in ("HOOKED", "LIFTING") or        (crane.state == "CLEARING" and hz > TILT_Z + 0.5):

        # ── Constants — never change ──
        PL  = TILT_W        # panel length = 4.0
        PW  = TILT_D / 2.0  # panel half-width = 1.4
        THK = 0.10          # half thickness

        # ── Compute panel angle this frame ──
        if crane.state == "HOOKED":
            panel_ang_deg = 60.0
        elif crane.state == "LIFTING":
            lift_range = CRANE_LIFT_Z - CRANE_LOWER_Z   # total lift distance
            t = np.clip((hz - CRANE_LOWER_Z) / lift_range, 0.0, 1.0)
            # Ease: sinusoidal so rotation feels smooth not mechanical
            t_eased = 0.5 - 0.5 * np.cos(t * np.pi)
            panel_ang_deg = 60.0 + t_eased * 30.0      # 60° → 90°
        else:
            panel_ang_deg = 90.0

        panel_ang = np.radians(panel_ang_deg)

        # ── Hook attachment: top-pivot corner of panel ──
        # Panel pivot point hangs from hook at hz
        hook_x = bx
        hook_z = hz - 0.15

        # ── Panel corners via rigid rotation about hook point ──
        # Local frame: pivot at (0,0), panel extends downward (negative local Z)
        # At 90°: panel is fully vertical — extends straight down
        # At 60°: panel is tilted — far end is out and down
        #
        # local_x along panel length (0=pivot end, PL=far end)
        # Rotation: world_x = hook_x + local_x * cos(panel_ang - pi/2 + pi/2)
        # Simpler: treat panel as rotating from vertical
        # offset_from_hook along panel:
        #   wx = hook_x - offset * sin(panel_ang)   (X component)
        #   wz = hook_z - offset * cos(panel_ang)   (Z component, downward)

        def panel_pt(offset, dy):
            """
            offset: distance along panel from hook attachment (0=top, PL=bottom)
            dy: Y offset (±PW for edges, 0 for center)
            Returns world (x, y, z)
            """
            wx = hook_x - offset * np.sin(panel_ang - np.pi/2)
            wz = hook_z - offset * np.cos(panel_ang - np.pi/2)
            return (wx, dy, wz)

        # Four face corners: top-near, top-far, bot-far, bot-near
        # "near/far" = ±THK in X (thickness), "top/bot" = 0/PL along panel
        def corner(offset, dy, thk_sign):
            base = panel_pt(offset, dy)
            # Thickness direction is perpendicular to panel in X-Z plane
            perp_x = thk_sign * THK * np.cos(panel_ang - np.pi/2)
            perp_z = thk_sign * THK * (-np.sin(panel_ang - np.pi/2))
            return (base[0] + perp_x, base[1], base[2] + perp_z)

        # Panel faces — OSB front/back, edge sides
        for dy_pair, fc in [((-PW, +PW), "#C8A068")]:
            for thk_s in [-1, +1]:
                face = [corner(0,  dy_pair[0], thk_s),
                        corner(0,  dy_pair[1], thk_s),
                        corner(PL, dy_pair[1], thk_s),
                        corner(PL, dy_pair[0], thk_s)]
                poly = Poly3DCollection([face], alpha=0.82,
                                        facecolor=fc, edgecolor=CE_GOLD, linewidth=1.2)
                ax.add_collection3d(poly)

        # Side edges (Y faces)
        for dy in [-PW, +PW]:
            face = [corner(0,  dy, -1), corner(0,  dy, +1),
                    corner(PL, dy, +1), corner(PL, dy, -1)]
            poly = Poly3DCollection([face], alpha=0.55,
                                    facecolor="#AA8855", edgecolor=CE_GOLD, linewidth=0.8)
            ax.add_collection3d(poly)

        # Hook attachment marker
        ax.scatter(hook_x, CRANE_BEAM_Y, hook_z+0.08,
                   color=CE_GOLD, s=55, marker="^", zorder=7)

        # Angle readout during rotation
        mid = panel_pt(PL/2, 0.0)
        ax.text(mid[0]-0.3, mid[1], mid[2],
                f"{panel_ang_deg:.0f}°", fontsize=6.5, color="#FF8800",
                family="monospace", ha="center", fontweight="bold")

        ax.text(hook_x, CRANE_BEAM_Y, hook_z+0.45, "COMPLETE",
                fontsize=5.5, color="#44FF44", family="monospace", ha="center")

    state_color = {"PARKED":"#44FF44","TRAVELING":CE_GOLD,
                   "LOWERING":"#FFFF00","HOOKED":"#FF8800",
                   "LIFTING":"#FF4400","CLEARING":"#FF8800"}.get(crane.state, "white")
    ax.text(bx, CRANE_BEAM_Y-0.5, CRANE_BEAM_Z+0.35,
            f"CRANE\n{crane.state}", fontsize=5.5,
            color=state_color, family="monospace", ha="center")

def draw_wall_panel(ax, corners_3d, color, edge_color, alpha=0.72, label=None):
    """Draw a wall panel as a Poly3DCollection quad — handles any orientation."""
    verts = [list(corners_3d)]
    poly = Poly3DCollection(verts, alpha=alpha, facecolor=color, edgecolor=edge_color, linewidth=1.5)
    ax.add_collection3d(poly)
    if label:
        cx = np.mean([c[0] for c in corners_3d])
        cy = np.mean([c[1] for c in corners_3d])
        cz = np.mean([c[2] for c in corners_3d])
        ax.text(cx, cy, cz+0.25, label, fontsize=5.5,
                color="#44FF44", family="monospace", ha="center")


def draw_tilt_table_wall(ax, s):
    """
    Draw wall panel rotating about pivot edge at TILT_PIVOT_X.
    Uses Poly3DCollection — works at all angles including 90° vertical.
    Suppressed when crane has taken the wall (crane draws it instead).
    """
    # Only show if wall is on tilt table and crane hasn't taken it yet
    if not (s.frame_at_tilt or s.sheathing_done or s.tilt_state != "FLAT"):
        return
    # Suppress tilt panel once crane has engaged — crane render takes over.
    # HOOKED, LIFTING, CLEARING all draw the panel via draw_crane().
    # Keeping both active caused double-render visual expansion.
    if s.crane_active and world.crane.state in ("HOOKED", "LIFTING", "CLEARING"):
        return

    # ── Panel geometry: computed ONCE per frame from constants only ──
    # TILT_W and TILT_D are module-level constants — never modified.
    # ang is derived purely from s.tilt_angle (scalar, no accumulation).
    # rot() maps local panel coords to world coords via pure trig.
    # No frame-to-frame state carries into this geometry block.
    ang = np.radians(s.tilt_angle)
    PL  = TILT_W        # panel length = 4.0  (constant)
    PW  = TILT_D / 2.0  # panel half-width = 1.4  (constant — was wrong at 1.1)

    def rot(x_local, cy):
        """
        Rigid-body rotation about hinge at (TILT_PIVOT_X, cy, Z=0) — floor level.
        x_local: distance along panel surface from hinge (0=hinge edge, PL=far edge).
        Panel surface sits at perpendicular offset TILT_Z from the hinge plane.
        Formula: wx = TILT_PIVOT_X + x_local*cos - TILT_Z*sin
                 wz =                x_local*sin + TILT_Z*cos
        At ang=0: wx = TILT_PIVOT_X + x_local, wz = TILT_Z  (correct flat position)
        Dimensions INVARIANT: |rot(PL) - rot(0)| == PL at every angle.
        """
        wx = TILT_PIVOT_X + x_local * np.cos(ang) - TILT_Z * np.sin(ang)
        wz =                x_local * np.sin(ang) + TILT_Z * np.cos(ang)
        return (wx, cy, wz)

    # Four panel corners — derived from PL and PW constants only
    c0 = rot(0.0, -PW)   # pivot edge, front
    c1 = rot(PL,  -PW)   # far edge,   front
    c2 = rot(PL,  +PW)   # far edge,   back
    c3 = rot(0.0, +PW)   # pivot edge, back
    corners = [c0, c1, c2, c3]

    color      = "#C8A068" if s.sheet_placed else "#88AACC"
    edge_color = CE_GOLD   if s.sheet_placed else "#4488FF"
    draw_wall_panel(ax, corners, color, edge_color)

    # LGS stud lines — positions also derived from PL/PW constants only
    if not s.sheet_placed:
        for frac in [0.3, 0.5, 0.7]:
            p0 = rot(frac * PL, -PW)
            p1 = rot(frac * PL, +PW)
            ax.plot([p0[0],p1[0]],[p0[1],p1[1]],[p0[2],p1[2]],
                    color="#5588CC", lw=2, alpha=0.8)

    # Angle readout
    if s.tilt_angle > 2.0:
        mid = rot(PL / 2.0, 0.0)
        ax.text(mid[0], mid[1], mid[2]+0.3,
                f"{s.tilt_angle:.0f}°", fontsize=7, color="#FF8800",
                family="monospace", ha="center", fontweight="bold")

def draw_placed_members(ax, s):
    """Draw each LGS member as placed on TABLE_JIG_FIXED (or moving with frame)."""
    if not s.placed_positions:
        return
    dx = 0.0
    if s.frame_conveying:
        dx = s.frame_x - FIXED_CX
    elif s.frame_at_tilt and not s.sheet_placed:
        # On tilt table (flat) — only if not yet shown in tilt render
        dx = (TILT_CX - 0.3) - FIXED_CX

    for mtype, pos in s.placed_positions:
        px = pos[0] + dx
        py = pos[1]
        pz = pos[2]
        if mtype == "STUD":
            ax.plot([px,px],[py-0.9,py+0.9],[pz,pz],
                    color="#5588CC", lw=3.5, solid_capstyle="round", alpha=0.9)
        else:
            ax.plot([px-1.3,px+1.3],[py,py],[pz,pz],
                    color="#3366AA", lw=2.5, solid_capstyle="round", alpha=0.9)

def draw_robot(ax, robot):
    pts = robot.joint_pts()
    lws = [8, 7, 7, 5, 5, 4]
    for i in range(len(pts)-1):
        p1, p2 = pts[i], pts[i+1]
        ax.plot([p1[0],p2[0]],[p1[1],p2[1]],[p1[2],p2[2]],
                color=robot.colors[i], lw=lws[i], solid_capstyle="round")
    for pt in pts:
        ax.scatter(*pt, color="white", s=20, zorder=5,
                   edgecolors="gray", linewidths=0.4)
    tcp = pts[-1]
    star_c = {"F1":"magenta","F2":"cyan","S1":"#FF8800","S2":"#00FFCC"}.get(robot.name,"white")
    ax.scatter(*tcp, color=star_c, s=90, marker="*", zorder=7)
    robot.trace.append(tcp.copy())
    if len(robot.trace) > 2:
        tr = np.array(robot.trace)
        tc = {"F1":"purple","F2":"teal","S1":"#884400","S2":"#007744"}.get(robot.name,"gray")
        ax.plot(tr[:,0],tr[:,1],tr[:,2], color=tc, lw=0.6, alpha=0.18)

def draw_base_ring(ax, base, color=CE_GOLD):
    th = np.linspace(0, 2*np.pi, 32)
    ax.plot(0.5*np.cos(th)+base[0], 0.5*np.sin(th)+base[1],
            np.zeros(32), color=color, lw=2)

def draw_station_labels(ax, s):
    tilt_color = {"FLAT":"#888888","TILTING":"#FF8800",
                  "VERTICAL":"#FF6600","CRANE":"#FF4400",
                  "RESET":"#44FF44"}.get(s.tilt_state, "#888888")
    labels = [
        (FIXED_CX, 3.4, 3.2, "① TABLE_JIG\n   FIXED",    CE_GOLD),
        (0.0,      3.4, 3.2, "② TABLE_JIG\n   ROLLER",   "#888888"),
        (TILT_CX,  3.4, 3.2, "③ TABLE_JIG\n   TILT",     tilt_color),
    ]
    for lx, ly, lz, txt, col in labels:
        ax.text(lx, ly, lz, txt, fontsize=7, color=col,
                family="monospace", ha="center", fontweight="bold")

# ═══════════════════════════════════════════════════════════════════════
# SECTION 10 — MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════

world = World()

fig = plt.figure(figsize=(18, 10))
fig.patch.set_facecolor("white")
ax  = fig.add_subplot(111, projection="3d")
ax.set_facecolor("white")
plt.subplots_adjust(bottom=0.10, left=0.02, right=0.98)

sax = plt.axes([0.15, 0.03, 0.55, 0.025])
sax.set_facecolor("#EEEEEE")
spd = Slider(sax, "Speed", 0.005, 0.25, valinit=0.06, color=CE_GOLD)
spd.label.set_color("#111111")
spd.valtext.set_color("#111111")

rax = plt.axes([0.75, 0.02, 0.08, 0.04])
btn = Button(rax, "RESET", color="#DDDDDD", hovercolor="#BBBBBB")
btn.label.set_color(CE_BLACK)

def reset_sim(event):
    global world
    world = World()

btn.on_clicked(reset_sim)


def update(frame_num):
    ax.clear()
    ax.set_facecolor("white")
    world.step(spd.val)
    s     = world.state
    crane = world.crane

    # ── Scene ──────────────────────────────────────────────────────
    draw_floor(ax)
    draw_rack(ax)
    draw_magazine(ax)

    # TABLE_JIG_FIXED
    draw_table(ax, FIXED_CX, FIXED_CY, FIXED_W, FIXED_D, FIXED_Z,
               color="#2A4A2A", label="TABLE_JIG_FIXED")
    draw_locating_pins(ax, FIXED_CX, FIXED_Z)

    # TABLE_JIG_ROLLER
    draw_roller_zone(ax)

    # TABLE_JIG_TILT — full rigid-body assembly, rotates with tilt_angle
    draw_tilt_table_assembly(ax, s.tilt_angle)

    # Shared datum line
    draw_datum_line(ax)

    # Crane
    draw_crane(ax, crane, s.wall_lifted, s.tilt_angle)

    # Base rings
    for base in [RF1_BASE, RF2_BASE, RS1_BASE, RS2_BASE]:
        draw_base_ring(ax, base)

    # Payloads
    draw_placed_members(ax, s)
    draw_tilt_table_wall(ax, s)

    # Robots
    draw_robot(ax, world.rf1)
    draw_robot(ax, world.rf2)
    draw_robot(ax, world.rs1)
    draw_robot(ax, world.rs2)

    # Station labels
    draw_station_labels(ax, s)

    # ── HUD ────────────────────────────────────────────────────────
    def fmt(robot):
        q  = np.degrees(robot.q)
        tc = robot.tcp()
        return (
            f"  CR6-{robot.name}  {'ACTIVE' if robot.active else 'IDLE  '}  {robot.state[:12]}\n"
            f"   J:{q[0]:+5.0f} {q[1]:+5.0f} {q[2]:+5.0f} {q[3]:+5.0f} {q[4]:+5.0f} {q[5]:+5.0f}\n"
            f"   TCP[{tc[0]:+.1f},{tc[1]:+.1f},{tc[2]:+.1f}]\n"
        )

    hud = (
        f"CE INTEGRATED CELL  V2.0  —  TABLE_JIG ARCHITECTURE\n"
        f"{'─'*44}\n"
        f"① FIXED   seq {s.framing_seq_idx}/{len(ASSEMBLY_SEQ)}"
        f"  {'DONE' if s.framing_complete else 'ACTIVE'}\n"
        + fmt(world.rf1)
        + fmt(world.rf2)
        + f"{'─'*44}\n"
        f"② ROLLER  {'TRANSFERRING' if s.frame_conveying else 'CLEAR':12}"
        f"  X={s.frame_x:+.1f}\n"
        f"{'─'*44}\n"
        f"③ TILT    {s.tilt_state:10}  {s.tilt_angle:5.1f}°"
        f"  fixture={'LOCKED' if s.fixture_locked else 'OPEN  '}\n"
        + fmt(world.rs1)
        + fmt(world.rs2)
        + f"{'─'*44}\n"
        f"CRANE  {crane.state:10}  X={crane.beam_x:+.1f}  Z={crane.hook_z:+.1f}\n"
        f"CYCLES COMPLETE : {s.cycles}\n"
        f"FRAME           : {world.tick}\n"
    )

    ax.text2D(0.01, 0.99, hud, transform=ax.transAxes,
              fontsize=5.8, family="monospace", va="top", color="#111111",
              bbox=dict(boxstyle="round,pad=0.3",
                        facecolor="#FFFDF0", alpha=0.88, edgecolor=CE_GOLD))

    ax.set_title(
        "CONSTRUCTION ENTERPRISES  —  INTEGRATED MODULAR WALL MANUFACTURING CELL  V2.0\n"
        "TABLE_JIG_FIXED  ►  TABLE_JIG_ROLLER  ►  TABLE_JIG_TILT  ►  OVERHEAD CRANE",
        fontsize=9, fontweight="bold", color=CE_BLACK)

    ax.set_xlim(-13, 14)
    ax.set_ylim(-8,   8)
    ax.set_zlim( 0,   7)
    ax.set_xlabel("X  (←  INFEED  |  OUTFEED  →)", color="#333333", fontsize=7)
    ax.set_ylabel("Y", color="#333333", fontsize=7)
    ax.set_zlabel("Z", color="#333333", fontsize=7)
    ax.tick_params(colors="#333333", labelsize=6)
    ax.xaxis.pane.fill = True; ax.yaxis.pane.fill = True; ax.zaxis.pane.fill = True
    ax.xaxis.pane.set_facecolor("#F5F5F5")
    ax.yaxis.pane.set_facecolor("#F5F5F5")
    ax.zaxis.pane.set_facecolor("#F5F5F5")
    ax.xaxis.pane.set_edgecolor("#CCCCCC")
    ax.yaxis.pane.set_edgecolor("#CCCCCC")
    ax.zaxis.pane.set_edgecolor("#CCCCCC")
    ax.grid(True, alpha=0.3, color="#AAAAAA")
    ax.set_xlim(-13, 14)
    ax.set_ylim(-8,   8)
    ax.set_zlim( 0,   7)


ani = FuncAnimation(fig, update, interval=1, cache_frame_data=False)
plt.show()
