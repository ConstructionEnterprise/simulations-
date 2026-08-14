"""
CHAPPELL ROBOTICS — CONSTRUCTION ENTERPRISES
Factory_Rail  V2.0

CONCEPT
  A standard CR6 robot is mounted on a FactoryRail linear track.
  The rail moves the robot base along the X-axis between stations.
  Robot kinematics are unchanged — the base simply relocates.

LAYOUT (West → East)
  Table_Jig_1   X=0.0  WEST  — destination (receives wall frame)
  Table_Jig_2   X=8.0  EAST  — source      (holds completed wall frame)

DEMONSTRATION SEQUENCE
  1. CR6 starts at Table_Jig_1 (west/home)
  2. FactoryRail travels EAST to Table_Jig_2
  3. CR6 picks completed LGS WallFrame from Table_Jig_2
  4. CR6 lifts WallFrame
  5. FactoryRail returns WEST to Table_Jig_1
  6. CR6 places WallFrame on Table_Jig_1
  7. Cycle repeats

FIXES V1 → V2
  - INIT_PARK stall fixed: skip park motion, go directly to RAIL_TO_JIG2
  - Frame Z corrected: frame sits ON TOP of jig surface (not inside it)
  - Station naming: Jig_1 (west/dest), Jig_2 (east/source)
  - Frame rendered flat/horizontal on table surface

GEOMETRY (verified reachable within 97% of MAX_REACH)
  Rail:       X = 0.0 to 8.0
  Jig_1:      X=0.0, Y=3.0, Z=0.8  (west — destination)
  Jig_2:      X=8.0, Y=3.0, Z=0.8  (east  — source, has wall frame)
  LIFT_Z:     2.5  (transit hold height)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from collections import deque

# ═══════════════════════════════════════════════════════════════════════
# SECTION 1 — DH PARAMETERS  (V6.1 validated — do not modify)
# ═══════════════════════════════════════════════════════════════════════

D1, A2, A3, D6 = 1.5, 2.5, 2.0, 0.5
MAX_REACH  = A2 + A3 + D6        # 5.0
SAFE_REACH = MAX_REACH * 0.97    # 4.85

# ═══════════════════════════════════════════════════════════════════════
# SECTION 2 — WORLD CONSTANTS
# ═══════════════════════════════════════════════════════════════════════

# Rail
RAIL_X_START   =  0.0
RAIL_X_END     =  8.0
RAIL_Y         =  0.0
RAIL_Z         =  0.0
RAIL_SPEED     =  0.05   # units/frame during transit

# Stations — West to East
STATION_JIG1_X =  0.0   # Table_Jig_1 — WEST — destination
STATION_JIG2_X =  8.0   # Table_Jig_2 — EAST — source (has wall frame)

# Robot base (Y, Z fixed — only X moves with rail)
ROBOT_BASE_Y   =  0.0
ROBOT_BASE_Z   =  0.0

# Table_Jig_2 (east — source, has frame)
JIG2_X         =  8.0
JIG2_Y         =  3.0
JIG2_Z         =  0.8
JIG2_W         =  3.0
JIG2_D         =  2.0

# Table_Jig_1 (west — destination, receives frame)
JIG1_X         =  0.0
JIG1_Y         =  3.0
JIG1_Z         =  0.8
JIG1_W         =  3.0
JIG1_D         =  2.0

# WallFrame dimensions — lays FLAT on jig (horizontal panel)
FRAME_W        =  2.4   # width along X
FRAME_D        =  1.6   # depth along Y
FRAME_THICK    =  0.10  # thickness

# Frame rests ON TOP of jig surface
FRAME_ON_JIG_Z =  JIG2_Z + FRAME_THICK / 2.0  # 0.85 — centre of flat frame

# Motion heights
LIFT_Z         =  2.5   # transit hold height
APPROACH_Z     =  1.8   # pre-descend
PICK_Z         =  JIG2_Z + FRAME_THICK + 0.05  # 0.95 — TCP just above frame
PLACE_Z        =  JIG1_Z + FRAME_THICK + 0.05  # 0.95 — TCP just above jig_1
PARK_Z         =  3.0   # robot parked upright

DWELL          =  15

DEFAULT_RPY    = np.array([0.0, np.pi, 0.0])

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
    r,p,y = rpy
    Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

def R_to_quat(R):
    t = R[0,0]+R[1,1]+R[2,2]
    if t>0:
        s=0.5/np.sqrt(t+1); return np.array([0.25/s,(R[2,1]-R[1,2])*s,(R[0,2]-R[2,0])*s,(R[1,0]-R[0,1])*s])
    elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s=2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2]); return np.array([(R[2,1]-R[1,2])/s,0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s])
    elif R[1,1]>R[2,2]:
        s=2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2]); return np.array([(R[0,2]-R[2,0])/s,(R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s])
    else:
        s=2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1]); return np.array([(R[1,0]-R[0,1])/s,(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s])

def quat_to_R(q):
    q=q/np.linalg.norm(q); w,x,y,z=q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)  ],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)  ],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ])

def slerp(q0,q1,t):
    q0=q0/np.linalg.norm(q0); q1=q1/np.linalg.norm(q1)
    d=np.clip(np.dot(q0,q1),-1,1)
    if d<0: q1=-q1; d=-d
    if d>0.9995: return q0+t*(q1-q0)
    th=np.arccos(d)
    return (np.sin((1-t)*th)*q0+np.sin(t*th)*q1)/np.sin(th)

# ═══════════════════════════════════════════════════════════════════════
# SECTION 4 — KINEMATICS  (V6.1 validated)
# ═══════════════════════════════════════════════════════════════════════

def fk(q):
    """Full FK — returns 7 joint positions in robot-local space."""
    q1,q2,q3,q4,q5,q6 = q
    dh = [[0,np.pi/2,D1,q1],[A2,0,0,q2],[A3,0,0,q3],
          [0,-np.pi/2,0,q4],[0,np.pi/2,0,q5],[0,0,D6,q6]]
    T=np.eye(4); pts=[T[:3,3].copy()]
    for row in dh:
        T=T@dh_transform(*row); pts.append(T[:3,3].copy())
    return pts

def ik(local_pos, R06):
    """Analytical IK. local_pos in robot-local space."""
    px,py,pz=local_pos; ap=R06[:,2]
    wx,wy,wz=px-D6*ap[0],py-D6*ap[1],pz-D6*ap[2]
    q1=np.arctan2(wy,wx); r=np.hypot(wx,wy); s=wz-D1
    d2=r*r+s*s
    if np.sqrt(d2)>MAX_REACH*0.99: return None
    c3=(d2-A2**2-A3**2)/(2*A2*A3)
    if abs(c3)>1: return None
    q3=np.arctan2(-np.sqrt(1-c3**2),c3)
    q2=np.arctan2(s,r)-np.arctan2(A3*np.sin(q3),A2+A3*np.cos(q3))
    T1=dh_transform(0,np.pi/2,D1,q1); T2=dh_transform(A2,0,0,q2); T3=dh_transform(A3,0,0,q3)
    R03=(T1@T2@T3)[:3,:3]; R36=R03.T@R06
    q5=np.arctan2(np.sqrt(R36[0,2]**2+R36[1,2]**2),R36[2,2])
    if abs(np.sin(q5))>1e-6:
        q4=np.arctan2(R36[1,2]/np.sin(q5),R36[0,2]/np.sin(q5))
        q6=np.arctan2(R36[2,1]/np.sin(q5),-R36[2,0]/np.sin(q5))
    else:
        q4=0.0; q6=np.arctan2(-R36[0,1],R36[1,1])
    return np.array([q1,q2,q3,q4,q5,q6])

def wp(pos, rpy=None):
    return (np.array(pos), (rpy if rpy is not None else DEFAULT_RPY).copy())

# ═══════════════════════════════════════════════════════════════════════
# SECTION 5 — FACTORY RAIL
# ═══════════════════════════════════════════════════════════════════════

class FactoryRail:
    """
    Linear transport asset.
    Moves the robot base along the X-axis between stations.
    The rail is a factory infrastructure component — not a robot.
    """

    def __init__(self):
        self.current_x   = RAIL_X_START
        self.target_x    = RAIL_X_START
        self.travel_speed= RAIL_SPEED
        self.state       = "IDLE"   # IDLE / TRAVELING
        self._direction  = 0

    def move_to(self, x):
        """Command rail to travel to target X position."""
        x = np.clip(x, RAIL_X_START, RAIL_X_END)
        if abs(self.current_x - x) < 0.01:
            return
        self.target_x   = x
        self._direction = 1.0 if x > self.current_x else -1.0
        self.state      = "TRAVELING"

    def update(self):
        """Advance rail position one frame. Returns True when target reached."""
        if self.state == "IDLE":
            return True
        self.current_x += self._direction * self.travel_speed
        # Check arrival
        if self._direction > 0 and self.current_x >= self.target_x:
            self.current_x = self.target_x
            self.state     = "IDLE"
            return True
        if self._direction < 0 and self.current_x <= self.target_x:
            self.current_x = self.target_x
            self.state     = "IDLE"
            return True
        return False

    def stop(self):
        self.state = "IDLE"

    def at_station(self, station_x, tol=0.05):
        return abs(self.current_x - station_x) < tol

    @property
    def robot_base(self):
        """Current world-space robot base position."""
        return np.array([self.current_x, ROBOT_BASE_Y, ROBOT_BASE_Z])

# ═══════════════════════════════════════════════════════════════════════
# SECTION 6 — CR6 ROBOT
# ═══════════════════════════════════════════════════════════════════════

class CR6:
    """
    Standard CR6 6-axis robot.
    Base position provided externally by FactoryRail.
    Kinematics are unchanged — rail moves the base, nothing else.
    """

    def __init__(self):
        self.state  = "IDLE"
        self.active = False
        self.trace  = deque(maxlen=400)
        self._wps   = []; self._names = []; self._dwell_at = set()
        self._idx   = 0;  self._t     = 0.0; self._dwell  = 0
        # Init pose — solved at startup, updated with rail position
        R = rpy_to_R(DEFAULT_RPY)
        q0 = ik(np.array([0.0, 0.0, PARK_Z]), R)
        self.q = q0 if q0 is not None else np.zeros(6)

    def launch(self, waypoints, names, dwell_at=None, base=None):
        """Start motion sequence. base = current rail robot_base."""
        if base is not None:
            cur_tcp = self.tcp(base)
        else:
            cur_tcp = self.tcp(np.zeros(3))
        start = (cur_tcp, DEFAULT_RPY.copy())
        self._wps      = [start] + list(waypoints)
        self._names    = ["CURRENT"] + list(names)
        self._dwell_at = dwell_at or set()
        self._idx=0; self._t=0.0; self._dwell=0
        self.active=True; self.state=names[0]

    def step(self, speed, base):
        """
        Advance one frame.
        base: current world-space base position from FactoryRail.
        """
        if not self.active or len(self._wps)<2: return None
        self._t += speed; action=None
        if self._t>=1.0:
            cur=self._names[self._idx]
            if cur in self._dwell_at and self._dwell<DWELL:
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
        q_t=slerp(R_to_quat(rpy_to_R(r0)),R_to_quat(rpy_to_R(r1)),t)
        R_t=quat_to_R(q_t)
        # IK uses robot-local coordinates
        q_sol=ik(pos_t-base, R_t)
        if q_sol is not None: self.q=q_sol
        return action

    def tcp(self, base):
        """TCP in world space."""
        return fk(self.q)[-1] + base

    def joint_pts(self, base):
        """All joint positions in world space."""
        return [p+base for p in fk(self.q)]

# ═══════════════════════════════════════════════════════════════════════
# SECTION 7 — WALL FRAME PAYLOAD
# ═══════════════════════════════════════════════════════════════════════

class WallFrame:
    """
    Completed LGS wall frame — the payload.
    Lays FLAT (horizontal) on jig surface.
    Ownership: ON_JIG2 / HELD / ON_JIG1
    """
    def __init__(self):
        self.reset()

    def reset(self):
        # Frame centre on top of Jig_2 surface
        self.pos       = np.array([JIG2_X, JIG2_Y, FRAME_ON_JIG_Z])
        self.ownership = "ON_JIG2"

# ═══════════════════════════════════════════════════════════════════════
# SECTION 8 — WAYPOINT BUILDERS
# ═══════════════════════════════════════════════════════════════════════

def build_pickup_sequence(base_x):
    """
    Robot at Jig_2 (east) — pick WallFrame.
    Descends to PICK_Z which is above the flat frame surface.
    """
    bx = base_x
    return [
        wp([bx, JIG2_Y, APPROACH_Z]),
        wp([bx, JIG2_Y, PICK_Z + 0.3]),
        wp([bx, JIG2_Y, PICK_Z]),
        wp([bx, JIG2_Y, LIFT_Z]),
    ], ["APPROACH","DESCEND","PICK","LIFT"], {"PICK"}

def build_place_sequence(base_x):
    """
    Robot at Jig_1 (west) — place WallFrame flat on table.
    """
    bx = base_x
    return [
        wp([bx, JIG1_Y, LIFT_Z]),
        wp([bx, JIG1_Y, PLACE_Z + 0.3]),
        wp([bx, JIG1_Y, PLACE_Z]),
        wp([bx, JIG1_Y, LIFT_Z]),
        wp([bx, ROBOT_BASE_Y, PARK_Z]),
    ], ["APPROACH_JIG1","DESCEND","PLACE","RETRACT","PARK"], {"PLACE"}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 9 — WORLD ORCHESTRATOR
# ═══════════════════════════════════════════════════════════════════════

class World:
    """
    Orchestrates FactoryRail, CR6, and WallFrame through
    the complete transport cycle.
    """

    # Top-level simulation states
    STATES = [
        "INIT_PARK",        # Robot parks at Station A ready
        "RAIL_TO_B",        # Rail travels to Station B
        "PICKUP",           # Robot picks wall frame at Station B
        "RAIL_TO_A",        # Rail returns to Station A (robot holds frame)
        "PLACE",            # Robot places frame at outfeed
        "RESET",            # Frame resets on jig, repeat
    ]

    def __init__(self):
        self.rail      = FactoryRail()
        self.robot     = CR6()
        self.frame     = WallFrame()
        self.sim_state = "RAIL_TO_JIG2"   # start immediately — no park stall
        self.cycles    = 0
        self.tick      = 0
        self._launched = False

    def step(self, speed):
        self.tick += 1
        rail  = self.rail
        robot = self.robot
        frame = self.frame
        base  = rail.robot_base

        if self.sim_state == "RAIL_TO_JIG2":
            # Travel EAST to Jig_2
            if not self._launched:
                rail.move_to(STATION_JIG2_X)
                self._launched = True
            # Robot holds upright during transit
            R = rpy_to_R(DEFAULT_RPY)
            q = ik(np.array([0.0, 0.0, PARK_Z]), R)
            if q is not None: robot.q = q
            if rail.state == "IDLE" and rail.at_station(STATION_JIG2_X):
                self.sim_state = "PICKUP"
                self._launched = False

        elif self.sim_state == "PICKUP":
            if not self._launched:
                wps, names, dwells = build_pickup_sequence(base[0])
                robot.launch(wps, names, dwells, base=base)
                self._launched = True

            robot.step(speed, base)

            # Proximity pick — frame on Jig_2
            if frame.ownership == "ON_JIG2":
                if np.linalg.norm(robot.tcp(base) - frame.pos) < 0.35:
                    frame.ownership = "HELD"

            if frame.ownership == "HELD":
                frame.pos = robot.tcp(base).copy()

            if not robot.active:
                self.sim_state = "RAIL_TO_JIG1"
                self._launched = False

        elif self.sim_state == "RAIL_TO_JIG1":
            # Travel WEST back to Jig_1
            if not self._launched:
                rail.move_to(STATION_JIG1_X)
                self._launched = True

            # Robot holds frame at LIFT_Z during transit
            hold_world = np.array([base[0], JIG2_Y, LIFT_Z])
            R = rpy_to_R(DEFAULT_RPY)
            q = ik(hold_world - base, R)
            if q is not None: robot.q = q

            if frame.ownership == "HELD":
                frame.pos = robot.tcp(base).copy()

            if rail.state == "IDLE" and rail.at_station(STATION_JIG1_X):
                self.sim_state = "PLACE"
                self._launched = False

        elif self.sim_state == "PLACE":
            if not self._launched:
                wps, names, dwells = build_place_sequence(base[0])
                robot.launch(wps, names, dwells, base=base)
                self._launched = True

            robot.step(speed, base)

            if frame.ownership == "HELD":
                frame.pos = robot.tcp(base).copy()

            # Proximity place — lay flat on Jig_1
            jig1_world = np.array([JIG1_X, JIG1_Y, FRAME_ON_JIG_Z])
            if frame.ownership == "HELD":
                if np.linalg.norm(robot.tcp(base) - np.array([JIG1_X, JIG1_Y, PLACE_Z])) < 0.35:
                    frame.ownership = "ON_JIG1"
                    frame.pos       = jig1_world.copy()

            if not robot.active:
                self.sim_state = "RESET"
                self._launched = False

        elif self.sim_state == "RESET":
            self.cycles += 1
            frame.reset()
            self.sim_state = "RAIL_TO_JIG2"
            self._launched = False

        # Always update rail
        rail.update()

# ═══════════════════════════════════════════════════════════════════════

    def snapshot(self):
        """Added for the Phase 3B pyvista migration -- read-only render view, no sim logic."""
        base = self.rail.robot_base
        return {
            "tick": self.tick,
            "cycles": self.cycles,
            "sim_state": self.sim_state,
            "rail_x": self.rail.current_x,
            "robot": {"state": self.robot.state, "pts": self.robot.joint_pts(base)},
            "frame": {"pos": self.frame.pos.copy(), "ownership": self.frame.ownership},
        }
