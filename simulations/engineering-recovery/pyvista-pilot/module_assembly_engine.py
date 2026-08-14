"""
CONSTRUCTION ENTERPRISES — CHAPPELL ROBOTICS
CE_Module_Assembly  V1.0

MODULAR CONSTRUCTION DIGITAL TWIN — WALL PANEL → MODULE ASSEMBLY
  STAGE 1 — Wall Manufacturing Cell  (from V2.5: TABLE_JIG architecture)
  STAGE 2 — Module Assembly Jig      (overhead crane delivers panels to module)

MODULE ASSEMBLY SEQUENCE
  4 wall panels manufactured sequentially by the wall cell
  Crane delivers each panel to MODULE_JIG in sequence:
    Panel 1 → NORTH wall  (Y+ face, facing inward)
    Panel 2 → SOUTH wall  (Y− face, facing inward)
    Panel 3 → EAST wall   (X+ face, connecting N+S)
    Panel 4 → WEST wall   (X− face, connecting N+S, closing the module)
  Module complete → crane parks → cycle resets

MODULE_JIG LAYOUT
  Center: X=22, Y=0
  Module interior footprint: 4.0 × 4.0 (world units)
  Wall panel height: 4.0 (TILT_W)
  Module height: 4.0

CRANE EXTENDED
  Same beam, extended range: X=5.5 (tilt pickup) to X=22 (module jig)
  Hook positions above each module wall slot

DH PARAMETERS  (V6.1 validated — do not modify)
  D1=1.5  A2=2.5  A3=2.0  D6=0.5  MAX_REACH=5.0
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from collections import deque

# ═══════════════════════════════════════════════════════════════════════
# SECTION 1 — DH PARAMETERS  (V6.1 validated — do not modify)
# ═══════════════════════════════════════════════════════════════════════

D1, A2, A3, D6 = 1.5, 2.5, 2.0, 0.5
MAX_REACH  = A2 + A3 + D6
SAFE_REACH = MAX_REACH * 0.90

# ═══════════════════════════════════════════════════════════════════════
# SECTION 2 — WORLD CONSTANTS
# ═══════════════════════════════════════════════════════════════════════

DEFAULT_RPY = np.array([0.0, np.pi, 0.0])
DWELL_LIMIT = 14

# ── Wall manufacturing (unchanged from V2.5) ─────────────────────────
RACK_X       = -10.0;  RACK_Y = -2.0;  RACK_Z = 0.9
RACK_SLOT_ZS = [RACK_Z + 0.15*i for i in range(5)]

FIXED_CX = -6.0;  FIXED_CY = 0.0;  FIXED_Z = 0.9
FIXED_W  =  4.0;  FIXED_D  = 2.8
RF1_BASE = np.array([FIXED_CX, -2.6, 0.0])
RF2_BASE = np.array([FIXED_CX, +2.6, 0.0])

STUD_PLACE_POSITIONS = [
    np.array([FIXED_CX-1.0, FIXED_CY, FIXED_Z+0.05]),
    np.array([FIXED_CX,     FIXED_CY, FIXED_Z+0.05]),
    np.array([FIXED_CX+1.0, FIXED_CY, FIXED_Z+0.05]),
]
TRACK_PLACE_POSITIONS = [
    np.array([FIXED_CX, FIXED_CY-1.0, FIXED_Z+0.05]),
    np.array([FIXED_CX, FIXED_CY+1.0, FIXED_Z+0.05]),
]
ASSEMBLY_SEQ = [("TRACK",0),("STUD",0),("STUD",1),("STUD",2),("TRACK",1)]

ROLLER_X_START = -3.8;  ROLLER_X_END = 2.8
ROLLER_Y = 0.0;  ROLLER_Z = 0.85;  ROLLER_SPEED = 0.07

TILT_CX = 5.5;  TILT_CY = 0.0;  TILT_Z = 0.9
TILT_W  = 4.0;  TILT_D  = 2.8
TILT_PIVOT_X = 3.2;  TILT_SPEED = 0.8
RS1_BASE = np.array([TILT_CX, -2.8, 0.0])
RS2_BASE = np.array([TILT_CX, +2.8, 0.0])
MAG_X = 3.2;  MAG_Y = -2.8;  MAG_Z = 0.9
SHEET_PLACE_POS = np.array([TILT_CX, TILT_CY, TILT_Z+0.06])
FASTEN_PTS = [
    np.array([TILT_CX-1.2, TILT_CY-1.0, TILT_Z+0.12]),
    np.array([TILT_CX+1.2, TILT_CY-1.0, TILT_Z+0.12]),
    np.array([TILT_CX,     TILT_CY,      TILT_Z+0.12]),
    np.array([TILT_CX-1.2, TILT_CY+1.0, TILT_Z+0.12]),
    np.array([TILT_CX+1.2, TILT_CY+1.0, TILT_Z+0.12]),
]

# ── Crane (extended range to reach module jig) ───────────────────────
CRANE_BEAM_Y     =  0.0
CRANE_BEAM_Z     =  5.5
CRANE_SPEED      =  0.07
CRANE_LOWER_Z    =  2.2
CRANE_LIFT_Z     =  5.2
CRANE_HOOK_SPEED =  0.04
CRANE_PARK_X     = 11.0
CRANE_PICKUP_X   =  5.5   # above tilt table

# ── MODULE_JIG ───────────────────────────────────────────────────────
MOD_CX   = 22.0    # module jig center X
MOD_CY   =  0.0    # module jig center Y
MOD_SIZE =  4.0    # interior footprint (X and Y)
MOD_H    =  4.0    # wall height = TILT_W
MOD_THK  =  0.20   # wall panel thickness

# Four wall slots: name, crane delivery X, crane delivery Y, orientation
# Each panel delivered vertically, crane positions above slot
MODULE_SLOTS = [
    ("NORTH", MOD_CX,            MOD_CY + MOD_SIZE/2 + MOD_THK/2, "NS"),
    ("SOUTH", MOD_CX,            MOD_CY - MOD_SIZE/2 - MOD_THK/2, "NS"),
    ("EAST",  MOD_CX + MOD_SIZE/2 + MOD_THK/2, MOD_CY,            "EW"),
    ("WEST",  MOD_CX - MOD_SIZE/2 - MOD_THK/2, MOD_CY,            "EW"),
]

# Heights
LIFT_Z = 2.2;  SAFE_Z = 3.0;  PARK_Z = 3.0

CE_GOLD  = "#CC6600"
CE_BLACK = "#1A1A1A"

# ═══════════════════════════════════════════════════════════════════════
# SECTION 3 — MATH  (V6.1 validated)
# ═══════════════════════════════════════════════════════════════════════

def dh_transform(a, alpha, d, theta):
    ct,st = np.cos(theta),np.sin(theta)
    ca,sa = np.cos(alpha),np.sin(alpha)
    return np.array([
        [ct,-st*ca, st*sa,a*ct],
        [st, ct*ca,-ct*sa,a*st],
        [0,  sa,    ca,   d   ],
        [0,  0,     0,    1   ]
    ])

def rpy_to_R(rpy):
    r,p,y = rpy
    Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz@Ry@Rx

def R_to_quat(R):
    t=R[0,0]+R[1,1]+R[2,2]
    if t>0:
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
        [1-2*(y*y+z*z),2*(x*y-z*w),  2*(x*z+y*w)  ],
        [2*(x*y+z*w),  1-2*(x*x+z*z),2*(y*z-x*w)  ],
        [2*(x*z-y*w),  2*(y*z+x*w),  1-2*(x*x+y*y)]
    ])

def slerp(q0,q1,t):
    q0=q0/np.linalg.norm(q0); q1=q1/np.linalg.norm(q1)
    d=np.clip(np.dot(q0,q1),-1,1)
    if d<0: q1=-q1; d=-d
    if d>0.9995: return q0+t*(q1-q0)
    th=np.arccos(d)
    return (np.sin((1-t)*th)*q0+np.sin(t*th)*q1)/np.sin(th)

def fk(q):
    q1,q2,q3,q4,q5,q6=q
    dh=[[0,np.pi/2,D1,q1],[A2,0,0,q2],[A3,0,0,q3],
        [0,-np.pi/2,0,q4],[0,np.pi/2,0,q5],[0,0,D6,q6]]
    T=np.eye(4); pts=[T[:3,3].copy()]
    for row in dh:
        T=T@dh_transform(*row); pts.append(T[:3,3].copy())
    return pts

def ik(local_pos,R06):
    px,py,pz=local_pos; ap=R06[:,2]
    wx,wy,wz=px-D6*ap[0],py-D6*ap[1],pz-D6*ap[2]
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

def tcp_world(q,base): return fk(q)[-1]+base
def park_pose(base):
    R=rpy_to_R(DEFAULT_RPY)
    q=ik(np.array([0.0,0.0,PARK_Z]),R)
    return q if q is not None else np.array([0.0,0.8,-0.5,0.0,0.5,0.0])

# ═══════════════════════════════════════════════════════════════════════
# SECTION 4 — ROBOT CLASS
# ═══════════════════════════════════════════════════════════════════════

def wp(pos,rpy=None):
    return (np.array(pos),(rpy if rpy is not None else DEFAULT_RPY).copy())

class Robot:
    def __init__(self,base,name,colors):
        self.base=np.array(base); self.name=name; self.colors=colors
        self.active=False; self.state="IDLE"
        self.trace=deque(maxlen=200)
        self.q=park_pose(base)
        self._wps=[]; self._names=[]; self._dwell_at=set()
        self._idx=0; self._t=0.0; self._dwell=0

    def launch(self,waypoints,names,dwell_at):
        cur=tcp_world(self.q,self.base)
        self._wps=[(cur,DEFAULT_RPY.copy())]+list(waypoints)
        self._names=["CURRENT"]+list(names)
        self._dwell_at=dwell_at; self._idx=0; self._t=0.0; self._dwell=0
        self.active=True; self.state=names[0]

    def step(self,speed):
        if not self.active or len(self._wps)<2: return None
        self._t+=speed; action=None
        if self._t>=1.0:
            cur=self._names[self._idx]
            if cur in self._dwell_at and self._dwell<DWELL_LIMIT:
                self._dwell+=1; self._t=1.0
            else:
                self._dwell=0; self._t=0.0; action=cur
                self._idx+=1
                if self._idx>=len(self._wps)-1:
                    self.state="IDLE"; self.active=False; return action
                self.state=self._names[self._idx]
        i=self._idx
        p0,r0=self._wps[i]; p1,r1=self._wps[i+1]; t=self._t
        pos_t=np.array(p0)+t*(np.array(p1)-np.array(p0))
        qt=slerp(R_to_quat(rpy_to_R(r0)),R_to_quat(rpy_to_R(r1)),t)
        qs=ik(pos_t-self.base,quat_to_R(qt))
        if qs is not None: self.q=qs
        return action

    def tcp(self): return tcp_world(self.q,self.base)
    def joint_pts(self): return [p+self.base for p in fk(self.q)]

# ═══════════════════════════════════════════════════════════════════════
# SECTION 5 — OVERHEAD CRANE  (extended range)
# ═══════════════════════════════════════════════════════════════════════

class OverheadCrane:
    """
    Extended crane: picks up wall panel from tilt table,
    travels to module jig slot, lowers, releases, returns to pickup.
    States: PARKED→TRAVELING_PICKUP→LOWERING→HOOKED→LIFTING→
            ROTATING→TRAVELING_DELIVER→LOWERING_DELIVER→PLACING→
            RISING→TRAVELING_PARK→PARKED
    """
    def __init__(self):
        self.beam_x       = CRANE_PARK_X
        self.hook_z       = CRANE_BEAM_Z
        self.panel_ang    = 60.0   # degrees — matches tilt table
        self.state        = "PARKED"
        self._target_x    = CRANE_PARK_X
        self._target_z    = CRANE_BEAM_Z
        self._deliver_x   = CRANE_PARK_X
        self._deliver_y   = 0.0
        self._dwell       = 0

    def set_delivery(self, slot_x, slot_y):
        self._deliver_x = slot_x
        self._deliver_y = slot_y

    def activate(self):
        if self.state == "PARKED":
            self._target_x = CRANE_PICKUP_X
            self.state = "TRAVELING_PICKUP"

    def update(self):
        """Returns True when delivery complete (panel placed in slot)."""
        spd = CRANE_SPEED

        if self.state == "TRAVELING_PICKUP":
            if self._move_x(CRANE_PICKUP_X, spd):
                self.state = "LOWERING"

        elif self.state == "LOWERING":
            if self._move_z(CRANE_LOWER_Z, CRANE_HOOK_SPEED):
                self.state = "HOOKED"; self._dwell = 0

        elif self.state == "HOOKED":
            self._dwell += 1
            if self._dwell > 18:
                self.state = "LIFTING"

        elif self.state == "LIFTING":
            lift_range = CRANE_LIFT_Z - CRANE_LOWER_Z
            t = np.clip((self.hook_z - CRANE_LOWER_Z) / lift_range, 0.0, 1.0)
            t_e = 0.5 - 0.5*np.cos(t*np.pi)
            self.panel_ang = 60.0 + t_e * 30.0
            if self._move_z(CRANE_LIFT_Z, CRANE_HOOK_SPEED):
                self.panel_ang = 90.0
                self.state = "TRAVELING_DELIVER"

        elif self.state == "TRAVELING_DELIVER":
            if self._move_x(self._deliver_x, spd):
                self.state = "LOWERING_DELIVER"; self._dwell = 0

        elif self.state == "LOWERING_DELIVER":
            # Lower panel into slot — hook comes down to just above floor
            deliver_z = 1.5
            if self._move_z(deliver_z, CRANE_HOOK_SPEED):
                self.state = "PLACING"; self._dwell = 0

        elif self.state == "PLACING":
            self._dwell += 1
            if self._dwell > 22:
                self.state = "RISING"

        elif self.state == "RISING":
            if self._move_z(CRANE_LIFT_Z, CRANE_HOOK_SPEED):
                self.state = "TRAVELING_PARK"

        elif self.state == "TRAVELING_PARK":
            if self._move_x(CRANE_PARK_X, spd):
                self.hook_z = CRANE_BEAM_Z
                self.panel_ang = 60.0
                self.state = "PARKED"
                return True   # delivery complete

        return False

    def _move_x(self, target, speed):
        dx = target - self.beam_x
        if abs(dx) <= speed:
            self.beam_x = target; return True
        self.beam_x += np.sign(dx)*speed; return False

    def _move_z(self, target, speed):
        dz = target - self.hook_z
        if abs(dz) <= speed:
            self.hook_z = target; return True
        self.hook_z += np.sign(dz)*speed; return False

    @property
    def carrying(self):
        return self.state in ("HOOKED","LIFTING","TRAVELING_DELIVER",
                              "LOWERING_DELIVER","PLACING","RISING")

# ═══════════════════════════════════════════════════════════════════════
# SECTION 6 — WAYPOINT BUILDERS
# ═══════════════════════════════════════════════════════════════════════

def build_f1_place(rack_pos,table_pos):
    rp,tp=np.array(rack_pos),np.array(table_pos)
    wps=[wp([rp[0],rp[1],SAFE_Z]),wp([rp[0],rp[1],rp[2]+0.45]),wp(rp),
         wp([rp[0],rp[1],LIFT_Z]),wp([tp[0],tp[1],SAFE_Z]),
         wp([tp[0],tp[1],tp[2]+0.45]),wp(tp),wp([tp[0],tp[1],SAFE_Z])]
    names=["RACK_APPROACH","RACK_DESCEND","RACK_PICK","LIFT",
           "TABLE_APPROACH","TABLE_DESCEND","TABLE_PLACE","RETURN"]
    return wps,names,{"RACK_PICK","TABLE_PLACE"}

def build_f2_fasten(pos):
    p=np.array(pos)
    wps=[wp([p[0],p[1],SAFE_Z]),wp([p[0],p[1],p[2]+0.4]),wp(p),
         wp([p[0],p[1],p[2]+0.4]),wp([p[0],p[1],SAFE_Z])]
    names=["APPROACH","DESCEND","FASTEN","RETRACT","CLEAR"]
    return wps,names,{"FASTEN"}

def build_s1_pickup():
    mp=np.array([MAG_X,MAG_Y,MAG_Z]); sp=SHEET_PLACE_POS.copy()
    wps=[wp([mp[0],mp[1],SAFE_Z]),wp([mp[0],mp[1],mp[2]+0.4]),wp(mp),
         wp([mp[0],mp[1],LIFT_Z]),wp([sp[0],sp[1],SAFE_Z]),
         wp([sp[0],sp[1],sp[2]+0.4]),wp(sp),wp([sp[0],sp[1],SAFE_Z])]
    names=["MAG_APPROACH","MAG_DESCEND","MAG_PICK","LIFT",
           "TABLE_APPROACH","TABLE_DESCEND","TABLE_PLACE","RETURN"]
    return wps,names,{"MAG_PICK","TABLE_PLACE"}

def build_s2_fasten():
    wps=[wp([FASTEN_PTS[0][0],FASTEN_PTS[0][1],SAFE_Z])]; names=["APPROACH"]
    for i,p in enumerate(FASTEN_PTS):
        wps.append(wp(p)); names.append(f"FASTEN_{i+1}")
    wps.append(wp([FASTEN_PTS[-1][0],FASTEN_PTS[-1][1],SAFE_Z])); names.append("CLEAR")
    return wps,names,{f"FASTEN_{i+1}" for i in range(len(FASTEN_PTS))}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 7 — STATE MACHINES
# ═══════════════════════════════════════════════════════════════════════

class WallCellState:
    """State for one wall panel production cycle."""
    def __init__(self):
        self.framing_seq_idx  = 0
        self.framing_complete = False
        self.placed_positions = []
        self.frame_x          = FIXED_CX
        self.frame_conveying  = False
        self.frame_at_tilt    = False
        self.sheet_placed     = False
        self.sheathing_done   = False
        self.tilt_angle       = 0.0
        self.tilt_state       = "FLAT"
        self.fixture_locked   = False
        self.crane_active     = False


class ModuleState:
    """Tracks module assembly — 4 panels placed in jig."""
    def __init__(self):
        self.panels_placed   = 0      # 0→4
        self.slot_idx        = 0      # which slot next
        self.placed_walls    = []     # list of (slot_name, orientation)
        self.module_complete = False
        self.modules_built   = 0


class World:
    def __init__(self):
        CA=["#FF3333","#FF8800","#FFD700","#44DD44","#00CCFF","#4466FF"]
        CB=["#CC00FF","#FF66CC","#FFFFFF","#00FFCC","#FF9900","#AAAAAA"]
        CS=["#FF5500","#FFAA00","#FFEE44","#55FF55","#22DDFF","#8888FF"]
        CT=["#FF8C00","#FFD700","#FFFF88","#88FF88","#88FFFF","#FF88FF"]
        self.rf1=Robot(RF1_BASE,"F1",CA); self.rf2=Robot(RF2_BASE,"F2",CB)
        self.rs1=Robot(RS1_BASE,"S1",CS); self.rs2=Robot(RS2_BASE,"S2",CT)
        self.crane=OverheadCrane()
        self.ws=WallCellState()   # wall cell
        self.ms=ModuleState()     # module assembly
        self.tick=0
        self._f1_launched=False; self._f2_launched=False; self._f1_placed=False
        self._s1_launched=False; self._s2_launched=False

    def _rack_pos(self,idx):
        return np.array([RACK_X,RACK_Y,RACK_SLOT_ZS[idx%len(RACK_SLOT_ZS)]])

    def _table_pos(self,mtype,pidx):
        return STUD_PLACE_POSITIONS[pidx] if mtype=="STUD" else TRACK_PLACE_POSITIONS[pidx]

    def step(self,speed):
        self.tick+=1
        ws=self.ws; ms=self.ms

        if ms.module_complete:
            return

        # ── WALL CELL: framing ───────────────────────────────────────
        if not ws.framing_complete:
            idx=ws.framing_seq_idx
            if idx<len(ASSEMBLY_SEQ):
                mtype,pidx=ASSEMBLY_SEQ[idx]
                rack_pos=self._rack_pos(idx); table_pos=self._table_pos(mtype,pidx)
                if not self.rf1.active and not self._f1_launched:
                    wps,names,dwell=build_f1_place(rack_pos,table_pos)
                    self.rf1.launch(wps,names,dwell)
                    self._f1_launched=True; self._f1_placed=False
                if self.rf1.active:
                    self.rf1.step(speed)
                    if not self._f1_placed and np.linalg.norm(self.rf1.tcp()-table_pos)<0.4:
                        ws.placed_positions.append((mtype,table_pos.copy()))
                        self._f1_placed=True
                if not self.rf1.active and self._f1_launched and \
                   not self.rf2.active and not self._f2_launched:
                    wps,names,dwell=build_f2_fasten(table_pos)
                    self.rf2.launch(wps,names,dwell); self._f2_launched=True
                if self.rf2.active: self.rf2.step(speed)
                if not self.rf1.active and not self.rf2.active and \
                   self._f1_launched and self._f2_launched:
                    ws.framing_seq_idx+=1
                    self._f1_launched=False; self._f2_launched=False; self._f1_placed=False
                    if ws.framing_seq_idx>=len(ASSEMBLY_SEQ):
                        ws.framing_complete=True

        # ── WALL CELL: roller transfer ───────────────────────────────
        if ws.framing_complete and not ws.frame_conveying and not ws.frame_at_tilt:
            ws.frame_conveying=True; ws.frame_x=FIXED_CX
        if ws.frame_conveying:
            ws.frame_x+=ROLLER_SPEED
            if ws.frame_x>=TILT_CX-0.3:
                ws.frame_x=TILT_CX-0.3; ws.frame_conveying=False; ws.frame_at_tilt=True

        # ── WALL CELL: sheathing ─────────────────────────────────────
        if ws.frame_at_tilt and not ws.sheathing_done:
            if not self.rs1.active and not self._s1_launched and not ws.sheet_placed:
                wps,names,dwell=build_s1_pickup()
                self.rs1.launch(wps,names,dwell); self._s1_launched=True
            if self.rs1.active: self.rs1.step(speed)
            if not self.rs1.active and self._s1_launched and not ws.sheet_placed:
                ws.sheet_placed=True
            if ws.sheet_placed and not self.rs2.active and not self._s2_launched:
                wps,names,dwell=build_s2_fasten()
                self.rs2.launch(wps,names,dwell); self._s2_launched=True
            if self.rs2.active: self.rs2.step(speed)
            if not self.rs2.active and self._s2_launched and ws.sheet_placed:
                ws.sheathing_done=True; ws.fixture_locked=True; ws.tilt_state="TILTING"

        # ── WALL CELL: tilt ──────────────────────────────────────────
        if ws.tilt_state=="TILTING":
            ws.tilt_angle+=0.8*speed*60.0
            if ws.tilt_angle>=60.0:
                ws.tilt_angle=60.0; ws.tilt_state="VERTICAL"

        # ── CRANE: pick up and deliver to module jig ─────────────────
        if ws.tilt_state=="VERTICAL" and not ws.crane_active:
            slot_name,sx,sy,orient=MODULE_SLOTS[ms.slot_idx]
            self.crane.set_delivery(sx,sy)
            self.crane.activate()
            ws.crane_active=True

        if ws.crane_active:
            done=self.crane.update()
            if done:
                # Panel placed in slot
                slot_name,sx,sy,orient=MODULE_SLOTS[ms.slot_idx]
                ms.placed_walls.append((slot_name,sx,sy,orient))
                ms.panels_placed+=1
                ms.slot_idx+=1
                if ms.panels_placed>=4:
                    ms.module_complete=True
                    ms.modules_built+=1
                else:
                    self._reset_wall_cycle()

    def _reset_wall_cycle(self):
        ws=self.ws
        ws.framing_seq_idx=0; ws.framing_complete=False; ws.placed_positions=[]
        ws.frame_x=FIXED_CX; ws.frame_conveying=False; ws.frame_at_tilt=False
        ws.sheet_placed=False; ws.sheathing_done=False
        ws.tilt_angle=0.0; ws.tilt_state="FLAT"; ws.fixture_locked=False
        ws.crane_active=False
        self._f1_launched=False; self._f2_launched=False; self._f1_placed=False
        self._s1_launched=False; self._s2_launched=False

# ═══════════════════════════════════════════════════════════════════════

    def snapshot(self):
        """Added for the Phase 3B pyvista migration -- read-only render view, no sim logic."""
        ws, ms = self.ws, self.ms
        return {
            "tick": self.tick,
            "tilt_angle": ws.tilt_angle, "tilt_state": ws.tilt_state,
            "framing_complete": ws.framing_complete, "sheathing_done": ws.sheathing_done,
            "panels_placed": ms.panels_placed, "placed_walls": list(ms.placed_walls), "module_complete": ms.module_complete,
            "crane": {"state": self.crane.state, "beam_x": self.crane.beam_x, "hook_z": self.crane.hook_z},
            "robots": {name: {"state": r.state, "pts": r.joint_pts()} for name, r in
                       [("F1", self.rf1), ("F2", self.rf2), ("S1", self.rs1), ("S2", self.rs2)]},
            "placed_positions": list(ws.placed_positions),
        }
