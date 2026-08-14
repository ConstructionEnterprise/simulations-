"""
CONSTRUCTION ENTERPRISES — CHAPPELL ROBOTICS
CE_Rail_System  V1.0

CR6 RAIL SYSTEM — DUAL RAIL CONFIGURATION — PHASE 2 SUBSYSTEM
  Standalone digital twin of the CR6 dual-rail robot system.
  Reference: Chappell Robotics CR6 Rail System finalized render (June 2026).

ARCHITECTURE
  Rail A     — Left rail  (Y = RAIL_A_Y), 2 robots, ATC rack at each end
  Rail B     — Right rail (Y = RAIL_B_Y), 2 robots, ATC rack at each end
  TABLE_JIG  — Black steel frame / wood deck, centered between rails
  4 ATC Racks — Near + far end of each rail, symmetric dual-rack layout

ROBOTS
  Robot A1  — Rail A, near end  (starts near ATC_A_NEAR)
  Robot A2  — Rail A, far end   (starts near ATC_A_FAR)
  Robot B1  — Rail B, near end  (starts near ATC_B_NEAR)
  Robot B2  — Rail B, far end   (starts near ATC_B_FAR)

ROBOT STATE MACHINE (per robot)
  PARKED_AT_ATC → TRAVELING_TO_WORK → WORKING →
  TRAVELING_TO_ATC → AT_ATC → TOOL_CHANGE → PARKED_AT_ATC

WORLD COORDINATES  (consistent with CE_Integrated_Cell_V2.6)
  TABLE_JIG center  X = 5.5,  Y = 0
  Rail A            Y = -3.8  (left / front)
  Rail B            Y = +3.8  (right / back)
  Rail X extent     X = 0.5  →  X = 10.5  (full jig length + ATC clearance)

PLATFORM  Android · Pydroid 3 · Python 3 · NumPy · Matplotlib
VALIDATED  Headless cycle test before display.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ═══════════════════════════════════════════════════════════════════════
# SECTION 1 — GEOMETRY CONSTANTS
# ═══════════════════════════════════════════════════════════════════════

# ── Rail dimensions ───────────────────────────────────────────────────
RAIL_X_MIN     =  0.5    # rail start (near end)
RAIL_X_MAX     = 10.5    # rail end   (far end)
RAIL_A_Y       = -3.8    # Rail A centerline Y
RAIL_B_Y       = +3.8    # Rail B centerline Y
RAIL_W         =  0.55   # rail body width (Y-axis)
RAIL_H         =  0.28   # rail body height
RAIL_BASE_H    =  0.12   # base extrusion height below rail
RAIL_BASE_W    =  0.80   # base width

# ── ATC Rack dimensions ───────────────────────────────────────────────
ATC_W          =  1.10   # rack width (X-axis)
ATC_D          =  0.75   # rack depth (Y-axis, extends outward from rail)
ATC_H          =  1.60   # rack total height
ATC_TIER_H     =  0.45   # height per tool tier
ATC_TIERS      =  3      # number of tool tiers
TOOLS_PER_TIER =  5      # tools per tier per rack
TOOL_H         =  0.28   # tool holder cone height
TOOL_R         =  0.07   # tool holder cone base radius

# ATC rack X centres (flush to rail ends, extend inward slightly)
ATC_NEAR_X     = RAIL_X_MIN + ATC_W / 2 + 0.05
ATC_FAR_X      = RAIL_X_MAX - ATC_W / 2 - 0.05

# ── Robot carriage ────────────────────────────────────────────────────
CARRIAGE_W     =  0.50   # carriage footprint along X
CARRIAGE_H     =  0.22   # carriage height above rail
CARRIAGE_D     =  RAIL_W + 0.10

# ── Robot arm (simplified 6-axis silhouette) ──────────────────────────
ROBOT_BASE_R   =  0.18   # base cylinder radius
ROBOT_BASE_H   =  0.28   # base height
LINK1_H        =  0.55   # upper arm length
LINK2_H        =  0.50   # forearm length
ROBOT_COLOR    = "#DDDDDD"   # white robot body
ROBOT_JOINT    = "#CC5500"   # orange joints

# ── TABLE_JIG ─────────────────────────────────────────────────────────
JIG_CX         =  5.50   # jig center X  (matches V2.6 TILT_CX)
JIG_CY         =  0.0    # jig center Y
JIG_LEN        =  6.80   # jig length  (X-axis)
JIG_WID        =  3.20   # jig width   (Y-axis, fits between rails)
JIG_FRAME_H    =  0.55   # steel frame height
JIG_LEG_H      =  0.30   # leveling leg height
JIG_DECK_H     =  0.08   # wood deck thickness
GRID_DIV       =  5      # grid divisions on deck surface

# ── Work zones on jig ─────────────────────────────────────────────────
# Each robot has a nominal work zone along the jig
# A1/B1 work near end, A2/B2 work far end
WORK_NEAR_X    =  JIG_CX - JIG_LEN * 0.25
WORK_FAR_X     =  JIG_CX + JIG_LEN * 0.25

# ── CE aesthetics ─────────────────────────────────────────────────────
CE_GOLD        = "#CC6600"
CE_AMBER       = "#E8A020"
CE_BLACK       = "#1A1A1A"
CE_DARK        = "#222222"
CE_RAIL_BODY   = "#1C1C1C"    # near-black rail extrusion
CE_RACK_BODY   = "#1A1A1A"    # ATC rack steel
CE_DECK        = "#D4A96A"    # wood deck
CE_FRAME       = "#1A1A1A"    # jig steel frame
CE_FENCE_POST  = "#E8A020"    # safety fence posts
CE_FENCE_MESH  = "#AAAAAA"

# ── Motion speeds ─────────────────────────────────────────────────────
ROBOT_SPEED    =  0.06   # X travel per frame
DWELL_WORKING  =  60     # frames working on jig
DWELL_ATC      =  30     # frames at ATC for tool change

# ═══════════════════════════════════════════════════════════════════════
# SECTION 2 — ROBOT STATE MACHINE
# ═══════════════════════════════════════════════════════════════════════

class CR6Robot:
    """
    Single CR6 robot on a linear rail carriage.
    Travels between ATC rack and work zone on the TABLE_JIG.
    """

    def __init__(self, name, rail_y, home_x, work_x, atc_x, side):
        self.name      = name
        self.rail_y    = rail_y
        self.x         = home_x
        self.work_x    = work_x
        self.atc_x     = atc_x
        self.side      = side      # +1 = Rail B (back), -1 = Rail A (front)
        self.state     = "PARKED_AT_ATC"
        self._dwell    = 0
        self.cycles    = 0
        self.tool_idx  = 0         # current tool (0-4)
        self._active   = False

    def activate(self):
        if self.state == "PARKED_AT_ATC":
            self.state  = "TRAVELING_TO_WORK"
            self._active = True

    def update(self):
        if not self._active:
            return False

        if self.state == "TRAVELING_TO_WORK":
            if self._move(self.work_x):
                self.state  = "WORKING"
                self._dwell = 0

        elif self.state == "WORKING":
            self._dwell += 1
            if self._dwell >= DWELL_WORKING:
                self.state = "TRAVELING_TO_ATC"

        elif self.state == "TRAVELING_TO_ATC":
            if self._move(self.atc_x):
                self.state  = "AT_ATC"
                self._dwell = 0

        elif self.state == "AT_ATC":
            self._dwell += 1
            if self._dwell >= DWELL_ATC // 2:
                self.state = "TOOL_CHANGE"
                self._dwell = 0

        elif self.state == "TOOL_CHANGE":
            self._dwell += 1
            if self._dwell >= DWELL_ATC // 2:
                self.tool_idx = (self.tool_idx + 1) % TOOLS_PER_TIER
                self.state    = "PARKED_AT_ATC"
                self.cycles  += 1
                return True

        return False

    def _move(self, target):
        dx = target - self.x
        if abs(dx) <= ROBOT_SPEED:
            self.x = target
            return True
        self.x += np.sign(dx) * ROBOT_SPEED
        return False

    @property
    def working(self):
        return self.state == "WORKING"

    @property
    def changing_tool(self):
        return self.state in ("AT_ATC", "TOOL_CHANGE")

    def arm_tip_pos(self):
        """Returns approximate world position of tool tip for visualisation."""
        reach = 0.9
        ry = self.rail_y + self.side * reach * 0.6
        rz = RAIL_H + CARRIAGE_H + ROBOT_BASE_H + LINK1_H + LINK2_H * 0.5
        if self.working:
            rz = RAIL_H + CARRIAGE_H + ROBOT_BASE_H + 0.35   # reaching down
        return np.array([self.x, ry, rz])


# ═══════════════════════════════════════════════════════════════════════
# SECTION 3 — WORLD
# ═══════════════════════════════════════════════════════════════════════

class RailWorld:
    def __init__(self):
        # Rail A — Y negative (left)
        self.A1 = CR6Robot("A1", RAIL_A_Y, ATC_NEAR_X, WORK_NEAR_X,
                           ATC_NEAR_X, side=+1)
        self.A2 = CR6Robot("A2", RAIL_A_Y, ATC_FAR_X,  WORK_FAR_X,
                           ATC_FAR_X,  side=+1)
        # Rail B — Y positive (right)
        self.B1 = CR6Robot("B1", RAIL_B_Y, ATC_NEAR_X, WORK_NEAR_X,
                           ATC_NEAR_X, side=-1)
        self.B2 = CR6Robot("B2", RAIL_B_Y, ATC_FAR_X,  WORK_FAR_X,
                           ATC_FAR_X,  side=-1)

        self.robots = [self.A1, self.A2, self.B1, self.B2]
        self.tick   = 0

        # Stagger activation so robots don't all start simultaneously
        self._start_delays = {
            "A1": 5, "A2": 20, "B1": 12, "B2": 30
        }

    def step(self):
        self.tick += 1
        for r in self.robots:
            delay = self._start_delays.get(r.name, 0)
            if self.tick >= delay and r.state == "PARKED_AT_ATC":
                r.activate()
            done = r.update()
            if done:
                # Auto-restart cycle
                r.activate()

    def reset(self):
        self.__init__()


# ═══════════════════════════════════════════════════════════════════════
# SECTION 4 — HEADLESS VALIDATION
# ═══════════════════════════════════════════════════════════════════════

def run_headless_validation():
    w = RailWorld()
    MAX_FRAMES = 10000
    target_cycles = 3   # each robot completes 3 cycles

    for _ in range(MAX_FRAMES):
        w.step()
        if all(r.cycles >= target_cycles for r in w.robots):
            print(f"[VALIDATION] PASS — all 4 robots completed "
                  f"{target_cycles} cycles at frame {w.tick}")
            # Position bounds check
            for r in w.robots:
                assert RAIL_X_MIN <= r.x <= RAIL_X_MAX, \
                    f"{r.name} out of rail bounds: x={r.x:.2f}"
            print("[VALIDATION] All position assertions passed.")
            return True

    print(f"[VALIDATION] FAIL — did not complete within {MAX_FRAMES} frames")
    return False

run_headless_validation()


# ═══════════════════════════════════════════════════════════════════════
# SECTION 5 — RENDERING HELPERS
# ═══════════════════════════════════════════════════════════════════════

def quad(ax, corners, fc, ec="#222222", alpha=0.88, lw=1.0):
    poly = Poly3DCollection([list(corners)], alpha=alpha,
                            facecolor=fc, edgecolor=ec, linewidth=lw)
    ax.add_collection3d(poly)


def box(ax, cx, cy, cz, hw, hd, hh, fc, ec="#1A1A1A", alpha=0.92):
    x0,x1 = cx-hw, cx+hw
    y0,y1 = cy-hd, cy+hd
    z0,z1 = cz-hh, cz+hh
    faces = [
        [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0)],
        [(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)],
        [(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)],
        [(x0,y1,z0),(x1,y1,z0),(x1,y1,z1),(x0,y1,z1)],
        [(x0,y0,z0),(x0,y1,z0),(x0,y1,z1),(x0,y0,z1)],
        [(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)],
    ]
    for f in faces:
        quad(ax, f, fc, ec, alpha)


def draw_floor(ax):
    xs = np.array([[-1, 12], [-1, 12]])
    ys = np.array([[-7, -7], [ 7,  7]])
    ax.plot_surface(xs, ys, np.zeros_like(xs),
                    color="#F0F0F0", alpha=0.50, zorder=0)


def draw_rail(ax, rail_y, label):
    """
    Black extruded rail body with CE gold accent stripe and base.
    Matches the heavy black extrusion from the reference render.
    """
    rx0, rx1 = RAIL_X_MIN, RAIL_X_MAX
    rw = RAIL_W / 2
    rb = RAIL_BASE_W / 2

    # ── Base plate (wider than rail body) ────────────────────────────
    base_faces = [
        [(rx0-0.1, rail_y-rb, 0),   (rx1+0.1, rail_y-rb, 0),
         (rx1+0.1, rail_y+rb, 0),   (rx0-0.1, rail_y+rb, 0)],
        [(rx0-0.1, rail_y-rb, RAIL_BASE_H),(rx1+0.1, rail_y-rb, RAIL_BASE_H),
         (rx1+0.1, rail_y+rb, RAIL_BASE_H),(rx0-0.1, rail_y+rb, RAIL_BASE_H)],
    ]
    for f in base_faces:
        quad(ax, f, "#111111", "#333333", alpha=0.95)

    # Base sides
    for y_s in [rail_y-rb, rail_y+rb]:
        quad(ax,
             [(rx0-0.1,y_s,0),(rx1+0.1,y_s,0),
              (rx1+0.1,y_s,RAIL_BASE_H),(rx0-0.1,y_s,RAIL_BASE_H)],
             "#161616", "#333333", alpha=0.95)

    # Leveling feet
    for fx in np.linspace(rx0+0.3, rx1-0.3, 8):
        for fy_off in [-rb+0.1, rb-0.1]:
            box(ax, fx, rail_y+fy_off, -0.06,
                0.06, 0.06, 0.06, "#333333", "#555555", 0.9)

    # ── Main rail body ────────────────────────────────────────────────
    rz0 = RAIL_BASE_H
    rz1 = rz0 + RAIL_H
    rail_faces = [
        [(rx0,rail_y-rw,rz0),(rx1,rail_y-rw,rz0),
         (rx1,rail_y+rw,rz0),(rx0,rail_y+rw,rz0)],
        [(rx0,rail_y-rw,rz1),(rx1,rail_y-rw,rz1),
         (rx1,rail_y+rw,rz1),(rx0,rail_y+rw,rz1)],
    ]
    for f in rail_faces:
        quad(ax, f, CE_RAIL_BODY, "#333333", alpha=0.97)
    for y_s in [rail_y-rw, rail_y+rw]:
        quad(ax,
             [(rx0,y_s,rz0),(rx1,y_s,rz0),(rx1,y_s,rz1),(rx0,y_s,rz1)],
             "#252525", "#333333", alpha=0.97)

    # ── CE Gold accent stripe along rail top ─────────────────────────
    stripe_z = rz1 + 0.02
    ax.plot([rx0, rx1], [rail_y-rw+0.05, rail_y-rw+0.05],
            [stripe_z]*2, color=CE_AMBER, lw=2.5, alpha=0.9)
    ax.plot([rx0, rx1], [rail_y+rw-0.05, rail_y+rw-0.05],
            [stripe_z]*2, color=CE_AMBER, lw=2.5, alpha=0.9)

    # ── Linear guide channels (two parallel slots) ────────────────────
    for gy in [rail_y-0.12, rail_y+0.12]:
        ax.plot([rx0, rx1], [gy, gy], [rz1+0.01]*2,
                color="#444444", lw=4, alpha=0.8, solid_capstyle="butt")
        ax.plot([rx0, rx1], [gy, gy], [rz1+0.03]*2,
                color="#666666", lw=1.5, alpha=0.6, solid_capstyle="butt")

    # ── CE Chappell Robotics label ────────────────────────────────────
    ax.text((rx0+rx1)/2, rail_y, RAIL_BASE_H + RAIL_H*0.4,
            "CE  CHAPPELL ROBOTICS",
            fontsize=5, color=CE_AMBER, family="monospace",
            ha="center", va="center", fontweight="bold")

    # ── Rail label ────────────────────────────────────────────────────
    ax.text(rx0 - 0.3, rail_y, RAIL_BASE_H + RAIL_H*0.5, label,
            fontsize=8, color=CE_AMBER, family="monospace",
            ha="right", fontweight="bold")


def draw_atc_rack(ax, cx, rail_y, side):
    """
    ATC rack — black steel frame, 3 tiers of cone-style BT/CAT tool holders.
    side: +1 rack extends toward Y+ (Rail A outward), -1 toward Y- (Rail B outward)
    Matches the reference render geometry.
    """
    rack_y_center = rail_y + side * (RAIL_BASE_W/2 + ATC_D/2 + 0.05)
    rz0 = 0.0

    # ── Rack frame ────────────────────────────────────────────────────
    box(ax, cx, rack_y_center, ATC_H/2,
        ATC_W/2, ATC_D/2, ATC_H/2,
        CE_RACK_BODY, "#333333", alpha=0.93)

    # ── Tier shelves ──────────────────────────────────────────────────
    for t in range(ATC_TIERS):
        tz = rz0 + 0.25 + t * ATC_TIER_H
        shelf = [
            (cx-ATC_W/2+0.05, rack_y_center-ATC_D/2+0.05, tz),
            (cx+ATC_W/2-0.05, rack_y_center-ATC_D/2+0.05, tz),
            (cx+ATC_W/2-0.05, rack_y_center+ATC_D/2-0.05, tz),
            (cx-ATC_W/2+0.05, rack_y_center+ATC_D/2-0.05, tz),
        ]
        quad(ax, shelf, "#282828", "#444444", alpha=0.9)

        # Tool holders on this tier
        tool_xs = np.linspace(cx - ATC_W/2 + 0.15,
                              cx + ATC_W/2 - 0.15, TOOLS_PER_TIER)
        for tx in tool_xs:
            # Cone body
            n = 8
            theta = np.linspace(0, 2*np.pi, n, endpoint=False)
            base_pts = [(tx + TOOL_R*np.cos(a),
                         rack_y_center + TOOL_R*np.sin(a),
                         tz + 0.03) for a in theta]
            tip_pt   = (tx, rack_y_center, tz + TOOL_H)
            for i in range(n):
                tri = [base_pts[i], base_pts[(i+1)%n], tip_pt]
                quad(ax, tri, "#5A5A5A", "#333333", alpha=0.85, lw=0.5)
            # Cone base ring
            base_face = base_pts
            quad(ax, base_face, "#3A3A3A", "#444444", alpha=0.8, lw=0.5)

    # ── CE badge on rack face ─────────────────────────────────────────
    face_y = rack_y_center - side * ATC_D/2
    ax.text(cx, face_y, ATC_H * 0.45,
            "CE\nCHAPPELL\nROBOTICS",
            fontsize=4.5, color=CE_AMBER,
            family="monospace", ha="center", va="center",
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.15",
                      facecolor=CE_BLACK, alpha=0.7,
                      edgecolor=CE_AMBER, linewidth=0.8))


def draw_carriage(ax, robot_x, rail_y):
    """Robot carriage riding on the rail."""
    cz = RAIL_BASE_H + RAIL_H
    box(ax, robot_x, rail_y, cz + CARRIAGE_H/2,
        CARRIAGE_W/2, CARRIAGE_D/2, CARRIAGE_H/2,
        "#2A2A2A", "#444444", alpha=0.95)
    # Carriage guide pads
    for gy in [rail_y-0.12, rail_y+0.12]:
        box(ax, robot_x, gy, cz + 0.04,
            CARRIAGE_W/2 * 0.7, 0.04, 0.04,
            "#3A3A3A", "#555555", 0.9)


def draw_robot_arm(ax, robot):
    """
    Simplified 6-axis robot arm.
    Pose changes based on state: working (reaching down) vs travelling.
    """
    rx   = robot.x
    ry   = robot.rail_y
    side = robot.side
    base_z = RAIL_BASE_H + RAIL_H + CARRIAGE_H

    # ── Base ──────────────────────────────────────────────────────────
    box(ax, rx, ry, base_z + ROBOT_BASE_H/2,
        ROBOT_BASE_R, ROBOT_BASE_R, ROBOT_BASE_H/2,
        "#CCCCCC", "#888888", 0.92)

    # ── Shoulder joint ────────────────────────────────────────────────
    j1z = base_z + ROBOT_BASE_H
    box(ax, rx, ry, j1z + 0.06,
        0.10, 0.10, 0.06, ROBOT_JOINT, "#AA4400", 0.95)

    if robot.working:
        # Arm reaching down toward jig — elbow bent forward
        el_x = rx + side * 0.18
        el_y = ry + side * 0.55
        el_z = j1z + LINK1_H * 0.5
        tip_x = rx + side * 0.12
        tip_y = ry + side * 0.75
        tip_z = j1z + 0.15
    elif robot.changing_tool:
        # Arm reaching sideways outward to ATC rack
        el_x = rx
        el_y = ry - side * 0.30
        el_z = j1z + LINK1_H * 0.7
        tip_x = rx
        tip_y = ry - side * ATC_D * 1.2
        tip_z = j1z + ATC_TIER_H * (robot.tool_idx % ATC_TIERS)
    else:
        # Travelling — arm raised, tucked
        el_x = rx
        el_y = ry + side * 0.25
        el_z = j1z + LINK1_H
        tip_x = rx
        tip_y = ry + side * 0.40
        tip_z = j1z + LINK1_H + LINK2_H * 0.6

    # ── Upper arm ─────────────────────────────────────────────────────
    ax.plot([rx, el_x], [ry, el_y], [j1z+ROBOT_BASE_H*0.5, el_z],
            color="#DDDDDD", lw=6, solid_capstyle="round", alpha=0.92)

    # ── Elbow joint ───────────────────────────────────────────────────
    ax.scatter([el_x], [el_y], [el_z],
               color=ROBOT_JOINT, s=38, zorder=10, depthshade=False)

    # ── Forearm ───────────────────────────────────────────────────────
    ax.plot([el_x, tip_x], [el_y, tip_y], [el_z, tip_z],
            color="#CCCCCC", lw=4, solid_capstyle="round", alpha=0.90)

    # ── Wrist joint ───────────────────────────────────────────────────
    ax.scatter([tip_x], [tip_y], [tip_z],
               color=ROBOT_JOINT, s=22, zorder=10, depthshade=False)

    # ── Tool (end effector) ───────────────────────────────────────────
    tool_colors = ["#FF4400", "#44AAFF", "#44FF44", "#FFAA00", "#FF44AA"]
    tc = tool_colors[robot.tool_idx % len(tool_colors)]
    tool_tip = np.array([tip_x, tip_y, tip_z])
    tool_end = tool_tip + np.array([0, side*0.04, -0.18])
    ax.plot([tool_tip[0], tool_end[0]],
            [tool_tip[1], tool_end[1]],
            [tool_tip[2], tool_end[2]],
            color=tc, lw=3, solid_capstyle="round", alpha=0.95)

    # ── Robot label ───────────────────────────────────────────────────
    ax.text(rx, ry - side*0.1, base_z + ROBOT_BASE_H + 0.08,
            robot.name, fontsize=5.5, color=CE_AMBER,
            family="monospace", ha="center", fontweight="bold")


def draw_table_jig(ax):
    """
    TABLE_JIG — black steel frame with wood deck surface.
    Matches reference render: grid frame, wood slat deck, leveling feet.
    """
    cx, cy = JIG_CX, JIG_CY
    lh = JIG_LEN / 2
    lw = JIG_WID / 2
    fh = JIG_FRAME_H
    leg_z = JIG_LEG_H

    # ── Leveling legs ─────────────────────────────────────────────────
    for lx in [cx-lh+0.3, cx+lh-0.3]:
        for ly in [cy-lw+0.2, cy+lw-0.2]:
            box(ax, lx, ly, leg_z/2,
                0.07, 0.07, leg_z/2, "#333333", "#555555", 0.9)
            # Foot plate
            box(ax, lx, ly, 0.04,
                0.12, 0.12, 0.04, "#2A2A2A", "#444444", 0.9)

    # ── Frame base cross-members ──────────────────────────────────────
    frame_z0 = leg_z
    # Perimeter frame
    for (x0,x1,yc) in [
        (cx-lh, cx+lh, cy-lw),
        (cx-lh, cx+lh, cy+lw),
    ]:
        quad(ax,
             [(x0,yc,frame_z0),(x1,yc,frame_z0),
              (x1,yc,frame_z0+fh),(x0,yc,frame_z0+fh)],
             CE_FRAME, "#333333", 0.95)
    for (xc,y0,y1) in [
        (cx-lh, cy-lw, cy+lw),
        (cx+lh, cy-lw, cy+lw),
    ]:
        quad(ax,
             [(xc,y0,frame_z0),(xc,y1,frame_z0),
              (xc,y1,frame_z0+fh),(xc,y0,frame_z0+fh)],
             CE_FRAME, "#333333", 0.95)

    # ── Interior grid cross-members ───────────────────────────────────
    for xi in np.linspace(cx-lh+JIG_LEN/GRID_DIV,
                          cx+lh-JIG_LEN/GRID_DIV, GRID_DIV-1):
        quad(ax,
             [(xi-0.03, cy-lw, frame_z0+0.05),
              (xi+0.03, cy-lw, frame_z0+0.05),
              (xi+0.03, cy+lw, frame_z0+0.05),
              (xi-0.03, cy+lw, frame_z0+0.05)],
             "#1A1A1A", "#333333", 0.8)
    for yi in np.linspace(cy-lw+JIG_WID/GRID_DIV,
                          cy+lw-JIG_WID/GRID_DIV, GRID_DIV-1):
        quad(ax,
             [(cx-lh, yi-0.03, frame_z0+0.05),
              (cx+lh, yi-0.03, frame_z0+0.05),
              (cx+lh, yi+0.03, frame_z0+0.05),
              (cx-lh, yi+0.03, frame_z0+0.05)],
             "#1A1A1A", "#333333", 0.8)

    # ── Wood deck surface ─────────────────────────────────────────────
    deck_z = frame_z0 + fh
    slat_w = JIG_WID / 10
    for i, sx in enumerate(np.linspace(cx-lh+0.05, cx+lh-0.05, 14)):
        col = CE_DECK if i % 2 == 0 else "#C89858"
        quad(ax,
             [(sx-JIG_LEN/28, cy-lw+0.05, deck_z),
              (sx+JIG_LEN/28, cy-lw+0.05, deck_z),
              (sx+JIG_LEN/28, cy+lw-0.05, deck_z),
              (sx-JIG_LEN/28, cy+lw-0.05, deck_z)],
             col, "#A07840", 0.88)

    # ── Central access hatch (black square, from render) ──────────────
    hatch_size = 0.55
    quad(ax,
         [(cx-hatch_size, cy-hatch_size, deck_z+0.005),
          (cx+hatch_size, cy-hatch_size, deck_z+0.005),
          (cx+hatch_size, cy+hatch_size, deck_z+0.005),
          (cx-hatch_size, cy+hatch_size, deck_z+0.005)],
         "#111111", "#333333", 0.97)

    # ── CE gold accent markers on frame edge ──────────────────────────
    for mx in np.linspace(cx-lh+0.5, cx+lh-0.5, 6):
        ax.plot([mx-0.08, mx+0.08], [cy-lw, cy-lw],
                [frame_z0+fh*0.5]*2, color=CE_AMBER, lw=2, alpha=0.8)

    ax.text(cx, cy, deck_z + 0.12, "TABLE JIG",
            fontsize=6, color="#888888", family="monospace",
            ha="center", fontweight="bold")


def draw_safety_fence(ax):
    """Perimeter fence behind each rail (back side only, matching render)."""
    FENCE_H = 2.0
    POST_SPACING = 0.9

    for rail_y, side in [(RAIL_A_Y, -1), (RAIL_B_Y, +1)]:
        fy = rail_y + side * (RAIL_BASE_W/2 + 0.55)
        post_xs = np.arange(RAIL_X_MIN - 0.2, RAIL_X_MAX + 0.4, POST_SPACING)
        for px in post_xs:
            ax.plot([px,px],[fy,fy],[0,FENCE_H+0.12],
                    color=CE_FENCE_POST, lw=2.5,
                    solid_capstyle="round", alpha=0.88)
        # Mesh panels
        for i in range(len(post_xs)-1):
            x0, x1 = post_xs[i], post_xs[i+1]
            panel = [(x0,fy,0),(x1,fy,0),(x1,fy,FENCE_H),(x0,fy,FENCE_H)]
            quad(ax, panel, CE_FENCE_MESH, CE_FENCE_POST,
                 alpha=0.15, lw=0.5)
        # Horizontal rails
        for hz in [FENCE_H*0.35, FENCE_H*0.70, FENCE_H]:
            ax.plot([post_xs[0], post_xs[-1]], [fy, fy],
                    [hz, hz], color=CE_FENCE_POST, lw=1.2, alpha=0.6)


def draw_work_indicators(ax, robots):
    """Highlight active work positions on the jig."""
    deck_z = JIG_LEG_H + JIG_FRAME_H + JIG_DECK_H + 0.02
    for r in robots:
        if r.working:
            tip = r.arm_tip_pos()
            # Work point flash on jig surface
            ax.scatter([r.x], [JIG_CY], [deck_z],
                       color=ROBOT_JOINT, s=55, alpha=0.7,
                       marker="x", zorder=12)
            # Vertical line from tool tip to work point
            ax.plot([tip[0], r.x], [tip[1], JIG_CY],
                    [tip[2], deck_z],
                    color=ROBOT_JOINT, lw=1, linestyle="--", alpha=0.5)


# ═══════════════════════════════════════════════════════════════════════
# SECTION 6 — MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════

world = RailWorld()

fig = plt.figure(figsize=(20, 11))
fig.patch.set_facecolor("white")
ax  = fig.add_subplot(111, projection="3d")
ax.set_facecolor("white")
plt.subplots_adjust(bottom=0.10, left=0.01, right=0.99)

sax = plt.axes([0.15, 0.03, 0.55, 0.025])
sax.set_facecolor("#EEEEEE")
spd = Slider(sax, "Speed", 0.5, 8.0, valinit=1.0, color=CE_AMBER)
spd.label.set_color("#111111")
spd.valtext.set_color("#111111")

rax = plt.axes([0.75, 0.02, 0.08, 0.04])
btn = Button(rax, "RESET", color="#DDDDDD", hovercolor="#BBBBBB")
btn.label.set_color(CE_BLACK)

def reset_sim(event):
    global world
    world = RailWorld()
btn.on_clicked(reset_sim)


def update(frame_num):
    ax.clear()
    ax.set_facecolor("white")

    steps = max(1, int(spd.val))
    for _ in range(steps):
        world.step()

    robots = world.robots

    # ── Scene ────────────────────────────────────────────────────────
    draw_floor(ax)

    # Rails
    draw_rail(ax, RAIL_A_Y, "RAIL A")
    draw_rail(ax, RAIL_B_Y, "RAIL B")

    # ATC Racks — 4 total, one at each end of each rail
    # Rail A racks extend outward toward Y- (side = -1)
    draw_atc_rack(ax, ATC_NEAR_X, RAIL_A_Y, side=-1)
    draw_atc_rack(ax, ATC_FAR_X,  RAIL_A_Y, side=-1)
    # Rail B racks extend outward toward Y+ (side = +1)
    draw_atc_rack(ax, ATC_NEAR_X, RAIL_B_Y, side=+1)
    draw_atc_rack(ax, ATC_FAR_X,  RAIL_B_Y, side=+1)

    # TABLE_JIG
    draw_table_jig(ax)

    # Safety fence
    draw_safety_fence(ax)

    # Robot carriages + arms
    for r in robots:
        draw_carriage(ax, r.x, r.rail_y)
        draw_robot_arm(ax, r)

    # Work indicators
    draw_work_indicators(ax, robots)

    # ── State colour map ──────────────────────────────────────────────
    state_colors = {
        "PARKED_AT_ATC":      "#44FF44",
        "TRAVELING_TO_WORK":  CE_AMBER,
        "WORKING":            "#FF4400",
        "TRAVELING_TO_ATC":   "#AADDFF",
        "AT_ATC":             "#FFDD00",
        "TOOL_CHANGE":        "#FF8800",
    }

    # ── HUD ───────────────────────────────────────────────────────────
    hud_lines = [
        "CE CR6 RAIL SYSTEM  V1.0",
        "─" * 36,
        f"{'ROBOT':<6} {'STATE':<20} {'X':>5}  {'TOOL':>4}  {'CYC':>3}",
        "─" * 36,
    ]
    for r in robots:
        sc = state_colors.get(r.state, "white")
        hud_lines.append(
            f"{r.name:<6} {r.state:<20} {r.x:>5.2f}  T{r.tool_idx:>3}  {r.cycles:>3}"
        )
    hud_lines += [
        "─" * 36,
        f"ATC NEAR  X={ATC_NEAR_X:.2f}",
        f"ATC FAR   X={ATC_FAR_X:.2f}",
        f"RAIL A    Y={RAIL_A_Y:.1f}",
        f"RAIL B    Y={RAIL_B_Y:.1f}",
        "─" * 36,
        f"FRAME     {world.tick}",
    ]
    hud = "\n".join(hud_lines)

    ax.text2D(0.01, 0.99, hud, transform=ax.transAxes,
              fontsize=5.8, family="monospace", va="top", color="#111111",
              bbox=dict(boxstyle="round,pad=0.3",
                        facecolor="#FFFDF0", alpha=0.90,
                        edgecolor=CE_AMBER))

    # Per-robot state badges
    for i, r in enumerate(robots):
        sc = state_colors.get(r.state, "white")
        ax.text2D(0.99, 0.99 - i*0.055,
                  f"◉ {r.name}  {r.state}",
                  transform=ax.transAxes, fontsize=6.5,
                  family="monospace", va="top", ha="right",
                  color=sc, fontweight="bold",
                  bbox=dict(boxstyle="round,pad=0.2",
                            facecolor=CE_BLACK, alpha=0.82,
                            edgecolor=sc))

    ax.set_title(
        "CONSTRUCTION ENTERPRISES  —  CHAPPELL ROBOTICS\n"
        "CR6 RAIL SYSTEM V1.0  —  DUAL RAIL  │  4 ROBOTS  │  4 ATC RACKS",
        fontsize=9, fontweight="bold", color=CE_BLACK)

    ax.set_xlim(-1, 12)
    ax.set_ylim(-7,  7)
    ax.set_zlim( 0,  5)
    ax.set_xlabel("X  (RAIL TRAVEL →)", color="#333333", fontsize=7)
    ax.set_ylabel("Y  (CROSS-AXIS)",     color="#333333", fontsize=7)
    ax.set_zlabel("Z  (HEIGHT)",         color="#333333", fontsize=7)
    ax.tick_params(colors="#444444", labelsize=6)

    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = True
        pane.set_facecolor("#F5F5F5")
        pane.set_edgecolor("#CCCCCC")
    ax.grid(True, alpha=0.3, color="#AAAAAA")

    ax.set_xlim(-1, 12)
    ax.set_ylim(-7,  7)
    ax.set_zlim( 0,  5)


ani = FuncAnimation(fig, update, interval=1, cache_frame_data=False)
plt.show()
