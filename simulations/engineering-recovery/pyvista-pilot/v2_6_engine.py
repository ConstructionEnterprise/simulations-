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

    def snapshot(self):
        """Added for the Phase 3B pyvista migration -- read-only render view, no sim logic."""
        s = self.state
        return {
            "tick": self.tick,
            "cycles": s.cycles,
            "tilt_angle": s.tilt_angle,
            "tilt_state": s.tilt_state,
            "framing_complete": s.framing_complete,
            "sheathing_done": s.sheathing_done,
            "wall_lifted": s.wall_lifted,
            "crane": {"state": self.crane.state, "beam_x": self.crane.beam_x, "hook_z": self.crane.hook_z},
            "robots": {name: {"state": r.state, "pts": r.joint_pts()} for name, r in
                       [("F1", self.rf1), ("F2", self.rf2), ("S1", self.rs1), ("S2", self.rs2)]},
            "placed_positions": list(s.placed_positions),
        }
