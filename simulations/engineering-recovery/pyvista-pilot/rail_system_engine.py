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

    def snapshot(self):
        """Added for the Phase 3B pyvista migration -- read-only render view, no sim logic."""
        return {
            "tick": self.tick,
            "robots": {r.name: {"state": r.state, "x": r.x, "rail_y": r.rail_y,
                                 "tool_idx": r.tool_idx, "cycles": r.cycles,
                                 "tip": r.arm_tip_pos()} for r in self.robots},
        }
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
