"""
CHAPPELL ROBOTICS — CONSTRUCTION ENTERPRISES
Dual_Robot_Jig_Frame  V1.0

LGS WALL FRAME ASSEMBLY CELL

ROBOTS
  CR6-1  Material handler — picks studs/tracks from rack, places on jig
  CR6-2  Welder          — welds all members on jig

WALL PANEL (V1)
  bottom_track  — placed first along Y = -0.8
  stud_L        — left stud    at X = -1.0
  stud_C        — center stud  at X =  0.0
  stud_R        — right stud   at X = +1.0
  top_track     — placed last  along Y = +0.8

ASSEMBLY SEQUENCE
  CR6-1 places:   bottom_track → stud_L → stud_C → stud_R → top_track
  CR6-2 welds:    each member at all joint intersections after CR6-1 places it

GEOMETRY (all verified reachable within 97% of MAX_REACH)
  CR6-1 base:  [0.0, -2.8,  0.0]
  CR6-2 base:  [0.0, +2.8,  0.0]
  Table jig:   X±1.5, Y±1.2, Z=0.8
  Material rack: [-3.5, -2.8, 0.8]

FUTURE (V2+)
  - 5-stud panel (24"oc) via robot rail travel
  - Automated outfeed conveyor
  - Production counter + cycle time
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

# Robot bases
RA_BASE = np.array([0.0, -2.8,  0.0])   # CR6-1 front
RB_BASE = np.array([0.0,  2.8,  0.0])   # CR6-2 back

# Table jig
TABLE_Z   =  0.8
TABLE_W   =  3.0   # X span
TABLE_D   =  2.8   # Y span
JIGSURF_Z =  TABLE_Z + 0.05   # part rests just above table surface

# Wall panel geometry
BOTTOM_Y  = -0.8
TOP_Y     =  0.8
STUD_XS   = [-1.0, 0.0, 1.0]
TRACK_LEN =  2.4   # X extent of tracks (±1.2)
STUD_H    =  1.6   # stud length (Y span of panel)
MEMBER_Z  =  JIGSURF_Z

# Material rack
RACK_BASE    = np.array([-3.5, -2.8, 0.0])
RACK_TOP     = np.array([-3.5, -2.8, TABLE_Z])
RACK_LEVELS  = [TABLE_Z + 0.15 * i for i in range(5)]

# Motion parameters
DWELL      = 12    # frames at pick/place/weld
SAFE_Z     =  3.2  # approach height
LIFT_Z     =  2.0  # lift after pick
APPROACH_Z =  1.6  # pre-descend height above table

DEFAULT_RPY = np.array([0.0, np.pi, 0.0])

# Weld sparks
SPARK_LIFE = 18

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
    q1,q2,q3,q4,q5,q6=q
    dh=[[0,np.pi/2,D1,q1],[A2,0,0,q2],[A3,0,0,q3],
        [0,-np.pi/2,0,q4],[0,np.pi/2,0,q5],[0,0,D6,q6]]
    T=np.eye(4); pts=[T[:3,3].copy()]
    for row in dh:
        T=T@dh_transform(*row); pts.append(T[:3,3].copy())
    return pts

def ik(local_pos, R06):
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

def tcp_world(q, base):
    return fk(q)[-1] + base

def wp(pos, rpy=None):
    return (np.array(pos), (rpy if rpy is not None else DEFAULT_RPY).copy())

# ═══════════════════════════════════════════════════════════════════════
# SECTION 5 — ROBOT CLASS
# ═══════════════════════════════════════════════════════════════════════

class Robot:
    def __init__(self, base, name, colors):
        self.base   = np.array(base)
        self.name   = name
        self.colors = colors
        self.state  = "IDLE"
        self.active = False
        self.trace  = deque(maxlen=300)
        self._wps=[];  self._names=[];  self._dwell_at=set()
        self._idx=0;   self._t=0.0;    self._dwell=0
        # Init pose
        R=rpy_to_R(DEFAULT_RPY)
        q0=ik(np.array([0,0,3.5])-self.base, R)
        self.q = q0 if q0 is not None else np.zeros(6)

    def launch(self, waypoints, names, dwell_at=None):
        cur = (self.tcp(), DEFAULT_RPY.copy())
        self._wps      = [cur] + list(waypoints)
        self._names    = ["CURRENT"] + list(names)
        self._dwell_at = dwell_at or set()
        self._idx=0; self._t=0.0; self._dwell=0
        self.active=True; self.state=names[0]

    def step(self, speed):
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
        q_sol=ik(pos_t-self.base, R_t)
        if q_sol is not None: self.q=q_sol
        return action

    def tcp(self):
        return fk(self.q)[-1]+self.base

    def pts(self):
        return [p+self.base for p in fk(self.q)]

# ═══════════════════════════════════════════════════════════════════════
# SECTION 6 — PANEL MEMBER
# ═══════════════════════════════════════════════════════════════════════

class Member:
    """A single LGS panel member (track or stud)."""
    def __init__(self, member_id, member_type, place_pos):
        self.id         = member_id
        self.type       = member_type   # "bottom_track","top_track","stud"
        self.place_pos  = np.array(place_pos)
        self.status     = "IN_RACK"     # IN_RACK / HELD / PLACED / WELDED
        self.pos        = RACK_TOP.copy()

    def rack_pick_pos(self, index):
        """Staggered rack positions."""
        z = RACK_LEVELS[index % len(RACK_LEVELS)]
        return np.array([RACK_BASE[0], RACK_BASE[1], z])

# ═══════════════════════════════════════════════════════════════════════
# SECTION 7 — ASSEMBLY SEQUENCE BUILDER
# ═══════════════════════════════════════════════════════════════════════

def build_place_sequence(pick_pos, place_pos):
    """
    CR6-1 waypoints: park → rack approach → pick → lift → travel → descend → place → retract
    """
    px,py,pz = pick_pos
    dx,dy,dz = place_pos
    return [
        wp([px,  py,  pz+1.5]),   # RACK_APPROACH
        wp([px,  py,  pz     ]),   # PICK
        wp([px,  py,  LIFT_Z ]),   # LIFT
        wp([dx,  dy,  LIFT_Z ]),   # TRAVEL
        wp([dx,  dy,  dz+0.4 ]),   # DESCEND
        wp([dx,  dy,  dz     ]),   # PLACE
        wp([dx,  dy,  LIFT_Z ]),   # RETRACT
    ], ["RACK_APPROACH","PICK","LIFT","TRAVEL","DESCEND","PLACE","RETRACT"], \
       {"PICK","PLACE"}

def build_weld_sequence(weld_points):
    """
    CR6-2 waypoints: visits each weld point in sequence.
    weld_points: list of (x,y,z) positions
    """
    wps, names = [], []
    for i, pt in enumerate(weld_points):
        x,y,z = pt
        wps  += [wp([x,y,z+0.8]), wp([x,y,z+0.05])]
        names+= [f"WELD_APPROACH_{i}", f"WELD_{i}"]
    return wps, names, {n for n in names if n.startswith("WELD_") and "APPROACH" not in n}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 8 — WORLD ORCHESTRATOR
# ═══════════════════════════════════════════════════════════════════════

class World:
    def __init__(self):
        self.cr6_1 = Robot(RA_BASE, "CR6-1",
                           ["#FF3333","#FF8800","#FFD700","#44DD44","#00CCFF","#4466FF"])
        self.cr6_2 = Robot(RB_BASE, "CR6-2",
                           ["#CC44FF","#FF66CC","#FFFFFF","#00FFCC","#FF9900","#AAAAAA"])

        # Build member sequence
        self.members = [
            Member(0, "bottom_track", [0.0,  BOTTOM_Y, MEMBER_Z]),
            Member(1, "stud",         [STUD_XS[0], 0.0, MEMBER_Z]),
            Member(2, "stud",         [STUD_XS[1], 0.0, MEMBER_Z]),
            Member(3, "stud",         [STUD_XS[2], 0.0, MEMBER_Z]),
            Member(4, "top_track",    [0.0,  TOP_Y,    MEMBER_Z]),
        ]
        self.place_idx   = 0     # next member for CR6-1 to place
        self.held_member = None
        self.weld_queue  = []    # members waiting for CR6-2 to weld
        self.sparks      = []
        self.panels_done = 0
        self.frame       = 0

    def _launch_place(self):
        """Launch CR6-1 to pick and place next member."""
        if self.place_idx >= len(self.members): return
        m = self.members[self.place_idx]
        pick_pos  = m.rack_pick_pos(self.place_idx)
        place_pos = m.place_pos
        wps, names, dwells = build_place_sequence(pick_pos, place_pos)
        self.cr6_1.launch(wps, names, dwells)
        self.held_member = m
        # Keep member at rack until TCP is close — do not pre-attach

    def _launch_weld(self, member):
        """Launch CR6-2 to weld a placed member."""
        weld_pts = self._weld_points(member)
        wps, names, dwells = build_weld_sequence(weld_pts)
        self.cr6_2.launch(wps, names, dwells)

    def _weld_points(self, member):
        """Return list of weld positions for a member."""
        if member.type == "bottom_track":
            return [[sx, BOTTOM_Y, MEMBER_Z] for sx in STUD_XS]
        elif member.type == "top_track":
            return [[sx, TOP_Y, MEMBER_Z] for sx in STUD_XS]
        elif member.type == "stud":
            x = member.place_pos[0]
            return [[x, BOTTOM_Y, MEMBER_Z], [x, TOP_Y, MEMBER_Z]]
        return []

    def step(self, speed):
        self.frame += 1
        r1, r2 = self.cr6_1, self.cr6_2

        # ── CR6-1 launch trigger ──────────────────────────────────────
        if not r1.active and self.place_idx < len(self.members):
            if not self.cr6_2.active or self.place_idx == 0:
                self._launch_place()

        # ── CR6-1 tick ────────────────────────────────────────────────
        if r1.active:
            action = r1.step(speed)
            m = self.held_member

            # Proximity pick
            if m and m.status == "IN_RACK":
                if np.linalg.norm(r1.tcp() - m.rack_pick_pos(self.place_idx)) < 0.3:
                    m.status = "HELD"

            # Part follows TCP
            if m and m.status == "HELD":
                m.pos = r1.tcp().copy()

            # Proximity place
            if m and m.status == "HELD" and r1.state in ("PLACE","RETRACT"):
                if np.linalg.norm(r1.tcp() - m.place_pos) < 0.3:
                    m.status = "PLACED"
                    m.pos    = m.place_pos.copy()
                    self.weld_queue.append(m)
                    self.held_member = None
                    self.place_idx  += 1

        # ── CR6-2 launch trigger ──────────────────────────────────────
        if not r2.active and self.weld_queue:
            next_m = self.weld_queue.pop(0)
            self._launch_weld(next_m)

        # ── CR6-2 tick ────────────────────────────────────────────────
        if r2.active:
            action = r2.step(speed)
            if action and action.startswith("WELD_") and "APPROACH" not in action:
                # Spawn weld sparks
                tcp = r2.tcp()
                for _ in range(8):
                    angle = np.random.uniform(0, 2*np.pi)
                    speed_s = np.random.uniform(0.02, 0.08)
                    self.sparks.append({
                        "pos": tcp.copy(),
                        "vel": np.array([np.cos(angle)*speed_s,
                                         np.sin(angle)*speed_s,
                                         np.random.uniform(0.01,0.06)]),
                        "life": SPARK_LIFE,
                        "max_life": SPARK_LIFE
                    })
                # Mark welded
                idx = int(action.split("_")[-1])

        # ── Check panel complete ──────────────────────────────────────
        all_welded = all(m.status in ("WELDED","PLACED") for m in self.members)
        cr6_2_done = not r2.active and not self.weld_queue
        if all_welded or (self.place_idx >= len(self.members) and cr6_2_done and not r2.active):
            if not r1.active and not r2.active and self.weld_queue == []:
                placed = sum(1 for m in self.members if m.status in ("PLACED","WELDED","HELD"))
                if placed == len(self.members):
                    self.panels_done += 1
                    self._reset_panel()

        # ── Update sparks ─────────────────────────────────────────────
        live = []
        for s in self.sparks:
            s["pos"] += s["vel"]
            s["vel"][2] -= 0.003
            s["life"]   -= 1
            if s["life"] > 0: live.append(s)
        self.sparks = live

    def _reset_panel(self):
        for i, m in enumerate(self.members):
            m.status = "IN_RACK"
            m.pos    = m.rack_pick_pos(i)
        self.place_idx   = 0
        self.held_member = None
        self.weld_queue  = []

# ═══════════════════════════════════════════════════════════════════════
# SECTION 9 — RENDERING
# ═══════════════════════════════════════════════════════════════════════

def draw_table_jig(ax):
    """Draw the assembly jig table."""
    hw, hd = TABLE_W/2, TABLE_D/2
    xs = np.array([[-hw,hw],[-hw,hw]])
    ys = np.array([[-hd,-hd],[hd,hd]])
    zs = np.full_like(xs, TABLE_Z)
    ax.plot_surface(xs, ys, zs, color="#555555", alpha=0.4)
    for x in [-hw, hw]:
        for y in [-hd, hd]:
            ax.plot([x,x],[y,y],[0,TABLE_Z], color="#444444", lw=2)
    # Jig pins
    for sx in STUD_XS:
        for py in [BOTTOM_Y, TOP_Y]:
            ax.scatter(sx, py, TABLE_Z+0.06, color="gold", s=25,
                       edgecolors="orange", linewidths=0.5, zorder=4)

def draw_rack(ax):
    """Draw material rack."""
    rx, ry = RACK_BASE[0], RACK_BASE[1]
    for z in RACK_LEVELS:
        ax.plot([rx-0.6, rx+0.1],[ry,ry],[z,z], color="#888888", lw=4, alpha=0.7)
    for x in [rx-0.6, rx+0.1]:
        ax.plot([x,x],[ry,ry],[0, RACK_LEVELS[-1]+0.2], color="#666666", lw=3)
    ax.text(rx-0.3, ry-0.3, 0.1, "RACK", fontsize=6,
            color="#AAAAAA", family="monospace")

def draw_member(ax, member):
    """Draw a placed LGS member. Only render once HELD or beyond."""
    if member.status == "IN_RACK": return   # not yet picked — don't render at rack
    pos = member.pos
    color_map = {
        "bottom_track": "#4488FF",
        "top_track":    "#4488FF",
        "stud":         "#88BBFF",
    }
    weld_color = "#00FF88"
    c = weld_color if member.status == "WELDED" else color_map[member.type]

    if member.type in ("bottom_track", "top_track"):
        if member.status == "HELD":
            ax.scatter(pos[0], pos[1], pos[2],
                       color=c, s=150, marker="s",
                       edgecolors="white", linewidths=1, zorder=8)
        else:
            ax.plot([pos[0]-1.2, pos[0]+1.2],
                    [pos[1], pos[1]],
                    [pos[2], pos[2]],
                    color=c, lw=5, solid_capstyle="round")
    else:  # stud — render as small square marker when held, full stud when placed
        if member.status == "HELD":
            # Render as compact box following TCP
            ax.scatter(pos[0], pos[1], pos[2],
                       color=c, s=120, marker="s",
                       edgecolors="white", linewidths=1, zorder=8)
        else:
            ax.plot([pos[0], pos[0]],
                    [BOTTOM_Y, TOP_Y],
                    [pos[2], pos[2]],
                    color=c, lw=4, solid_capstyle="round")

def draw_robot(ax, robot):
    pts = robot.pts()
    tcp = pts[-1]
    lws = [8,7,7,5,5,4]
    for i in range(len(pts)-1):
        p1,p2=pts[i],pts[i+1]
        ax.plot([p1[0],p2[0]],[p1[1],p2[1]],[p1[2],p2[2]],
                color=robot.colors[i], lw=lws[i], solid_capstyle="round")
    for pt in pts:
        ax.scatter(*pt, color="white", s=28, zorder=5,
                   edgecolors="gray", linewidths=0.5)
    star = "magenta" if robot.name=="CR6-1" else "cyan"
    ax.scatter(*tcp, color=star, s=120, marker="*", zorder=7)
    robot.trace.append(tcp.copy())
    if len(robot.trace)>2:
        tr=np.array(robot.trace)
        tc="purple" if robot.name=="CR6-1" else "teal"
        ax.plot(tr[:,0],tr[:,1],tr[:,2], color=tc, lw=0.8, alpha=0.3)

def draw_sparks(ax, sparks):
    for s in sparks:
        alpha = s["life"] / s["max_life"]
        ax.scatter(*s["pos"], color="yellow", s=12,
                   alpha=alpha, zorder=8, edgecolors="none")

def draw_base_ring(ax, base, color):
    th=np.linspace(0,2*np.pi,32)
    ax.plot(0.5*np.cos(th)+base[0], 0.5*np.sin(th)+base[1],
            np.zeros(32), color=color, lw=2)

# ═══════════════════════════════════════════════════════════════════════
# SECTION 10 — MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════

world = World()

fig = plt.figure(figsize=(14, 10))
ax  = fig.add_subplot(111, projection="3d")
plt.subplots_adjust(bottom=0.12)

sax = plt.axes([0.2, 0.03, 0.6, 0.03])
spd = Slider(sax, "Motion Speed", 0.005, 0.06, valinit=0.020)

def update(frame):
    ax.clear()
    world.step(spd.val)

    r1, r2 = world.cr6_1, world.cr6_2

    # Scene
    draw_table_jig(ax)
    draw_rack(ax)
    draw_base_ring(ax, RA_BASE, "#FF6666")
    draw_base_ring(ax, RB_BASE, "#AA66FF")

    # Members
    for m in world.members:
        draw_member(ax, m)

    # Robots
    draw_robot(ax, r1)
    draw_robot(ax, r2)

    # Weld sparks
    draw_sparks(ax, world.sparks)

    # Panel outline (ghost)
    if world.place_idx > 0:
        for sx in STUD_XS:
            ax.plot([sx,sx],[BOTTOM_Y,TOP_Y],[MEMBER_Z,MEMBER_Z],
                    color="gray", lw=0.5, alpha=0.25, linestyle="--")
        ax.plot([-1.2,1.2],[BOTTOM_Y,BOTTOM_Y],[MEMBER_Z,MEMBER_Z],
                color="gray", lw=0.5, alpha=0.25, linestyle="--")
        ax.plot([-1.2,1.2],[TOP_Y,TOP_Y],[MEMBER_Z,MEMBER_Z],
                color="gray", lw=0.5, alpha=0.25, linestyle="--")

    # HUD
    q1 = np.degrees(r1.q)
    q2 = np.degrees(r2.q)
    placed   = sum(1 for m in world.members if m.status in ("PLACED","WELDED"))
    welded   = sum(1 for m in world.members if m.status == "WELDED")
    m_status = " | ".join(f"{m.type[:3].upper()}:{m.status[:3]}"
                          for m in world.members)

    hud = (
        f"CHAPPELL ROBOTICS  —  LGS WALL FRAME  V1.0\n"
        f"{'─'*44}\n"
        f"CR6-1  [{('ACTIVE' if r1.active else 'IDLE  '):6}]  {r1.state}\n"
        f" J1:{q1[0]:+6.1f} J2:{q1[1]:+6.1f} J3:{q1[2]:+6.1f}\n"
        f" TCP:[{r1.tcp()[0]:+.2f},{r1.tcp()[1]:+.2f},{r1.tcp()[2]:+.2f}]\n"
        f"{'─'*44}\n"
        f"CR6-2  [{('ACTIVE' if r2.active else 'IDLE  '):6}]  {r2.state}\n"
        f" J1:{q2[0]:+6.1f} J2:{q2[1]:+6.1f} J3:{q2[2]:+6.1f}\n"
        f" TCP:[{r2.tcp()[0]:+.2f},{r2.tcp()[1]:+.2f},{r2.tcp()[2]:+.2f}]\n"
        f"{'─'*44}\n"
        f"PLACED  : {placed}/{len(world.members)}\n"
        f"WELDED  : {welded}/{len(world.members)}\n"
        f"PANELS  : {world.panels_done}\n"
        f"FRAME   : {world.frame}\n"
        f"{'─'*44}\n"
        f"{m_status}\n"
    )
    ax.text2D(0.02, 0.97, hud, transform=ax.transAxes,
              fontsize=7, family="monospace", va="top", color="black")

    ax.set_title("CHAPPELL ROBOTICS  —  CR6  |  LGS WALL FRAME ASSEMBLY  V1.1",
                 fontsize=10, fontweight="bold")
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim( 0, 5)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.view_init(elev=28, azim=-55)
    ax.grid(True, alpha=0.25)

ani = FuncAnimation(fig, update, interval=40)
plt.show()
