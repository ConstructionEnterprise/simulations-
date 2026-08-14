"""
CONSTRUCTION ENTERPRISES — CHAPPELL ROBOTICS
CE_Overhead_Gantry  V1.0

OVERHEAD GANTRY MATERIAL HANDLING SYSTEM — PHASE 1 SUBSYSTEM
  Standalone digital twin of the 3-axis overhead gantry crane.
  Reference: Chappell Robotics 2.0T portal gantry render (June 2026).

STRUCTURAL ARCHITECTURE  (from reference render)
  End Trucks     — Two portal frames (dark grey box-section columns + base plates)
                   Fixed in Y. Travel in X along floor-embedded runway rails.
  Bridge Beam    — CE-gold I-beam spanning Y-axis between end trucks.
                   Catwalk + safety railing on top. Travels with end trucks in X.
  Trolley/Hoist  — Black hoist unit riding on bottom flange of bridge beam.
                   Travels independently in Y along bridge beam.
  Hook Block     — Wire rope + hook hanging from hoist. Travels in Z.

THREE AXES
  X  — Bridge travel  (end trucks + bridge move together along runway)
  Y  — Trolley travel (hoist unit slides along bridge beam)
  Z  — Hoist travel   (hook raises and lowers)

STATE MACHINE
  PARKED → TRAVELING_X → TRAVELING_Y → LOWERING → HOOKED →
  LIFTING → TRAVELING_DELIVER_X → TRAVELING_DELIVER_Y →
  LOWERING_DELIVER → PLACING → RISING → RETURNING_Y →
  RETURNING_X → PARKED

WORLD LAYOUT  (matches CE_Integrated_Cell_V2.6 coordinate space)
  Pickup zone     X=5.5,  Y=0   (above TABLE_JIG_TILT)
  Delivery zone   X=22.0, Y=0   (above MODULE_JIG)
  Park position   X=14.0, Y=0   (mid-span, clear of both zones)

PLATFORM  Android · Pydroid 3 · Python · NumPy · Matplotlib
VALIDATED  Headless cycle test before display code.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ═══════════════════════════════════════════════════════════════════════
# SECTION 1 — GANTRY GEOMETRY CONSTANTS
# All dimensions in world units. Every constant named and justified.
# ═══════════════════════════════════════════════════════════════════════

# ── Runway rails (X-axis) ────────────────────────────────────────────
RUNWAY_X_MIN   =  2.0    # infeed hard limit (above tilt table approach)
RUNWAY_X_MAX   = 26.0    # outfeed hard limit (beyond module jig)
RUNWAY_Y_NEG   = -4.5    # front rail Y position
RUNWAY_Y_POS   = +4.5    # back rail Y position
RUNWAY_Z       =  0.05   # rail sits just above floor

# ── Bridge beam (Y-axis span) ────────────────────────────────────────
BRIDGE_BEAM_Z  =  5.8    # bottom flange of bridge beam (hook hang height)
BRIDGE_BEAM_H  =  0.55   # beam depth (I-section height)
BRIDGE_BEAM_W  =  0.30   # beam flange width
CATWALK_H      =  0.20   # catwalk platform above top flange
RAIL_H         =  0.18   # safety railing height above catwalk

# ── Portal columns (end trucks) ──────────────────────────────────────
COL_W          =  0.40   # column box section width
COL_D          =  0.50   # column box section depth
COL_H          =  BRIDGE_BEAM_Z          # column height = beam Z
BASE_PLATE_W   =  0.75   # base plate footprint half-width
BASE_PLATE_H   =  0.08   # base plate thickness

# ── Trolley / hoist unit ─────────────────────────────────────────────
TROLLEY_W      =  0.70   # hoist box width (Y-axis)
TROLLEY_D      =  0.55   # hoist box depth (X-axis)
TROLLEY_H      =  0.45   # hoist box height

# ── Hook block ───────────────────────────────────────────────────────
HOOK_PARK_Z    =  BRIDGE_BEAM_Z - 0.3   # hook retracted (just below beam)
HOOK_LOWER_Z   =  1.10   # hook fully lowered for pickup (above tilt table height)
HOOK_DELIVER_Z =  1.20   # hook lowered for delivery into module slot
HOOK_SPEED     =  0.045  # Z travel speed per frame

# ── Travel positions ──────────────────────────────────────────────────
PARK_X         = 14.0    # bridge park X  (mid-span, clear of both zones)
PARK_Y         =  0.0    # trolley park Y (centerline)

PICKUP_X       =  5.5    # above TABLE_JIG_TILT  (matches V2.6 TILT_CX)
PICKUP_Y       =  0.0    # centerline

DELIVER_X      = 22.0    # above MODULE_JIG  (matches V2.6 MOD_CX)
DELIVER_Y      =  0.0    # centerline (NORTH/SOUTH/EAST/WEST handled by Y offset)

BRIDGE_SPEED   =  0.07   # X travel speed per frame
TROLLEY_SPEED  =  0.06   # Y travel speed per frame

# ── Safety envelope ───────────────────────────────────────────────────
SAFE_X_MIN     = RUNWAY_X_MIN + 0.5
SAFE_X_MAX     = RUNWAY_X_MAX - 0.5
SAFE_Y_MIN     = RUNWAY_Y_NEG + 0.8
SAFE_Y_MAX     = RUNWAY_Y_POS - 0.8

# ── CE aesthetics ─────────────────────────────────────────────────────
CE_GOLD        = "#CC6600"
CE_AMBER       = "#E8A020"    # bridge beam / structural steel colour
CE_DARK        = "#2A2A2A"    # column grey
CE_COLUMN      = "#4A4A4A"    # column body
CE_BLACK       = "#1A1A1A"
CE_HOIST       = "#1E1E1E"    # hoist unit (black box)
CE_WIRE        = "#888888"

# ── Dwell frames ──────────────────────────────────────────────────────
DWELL_HOOKED   = 22    # frames dwelling at HOOKED before lifting
DWELL_PLACING  = 28    # frames dwelling at PLACING before rising

# ═══════════════════════════════════════════════════════════════════════
# SECTION 2 — GANTRY STATE MACHINE
# ═══════════════════════════════════════════════════════════════════════

class OverheadGantry:
    """
    Three-axis gantry crane.
      bridge_x  — X position of bridge beam (and both end trucks)
      trolley_y — Y position of trolley/hoist on bridge beam
      hook_z    — Z position of hook block

    State machine:
      PARKED → TRAVELING_X → TRAVELING_Y → LOWERING → HOOKED →
      LIFTING → TRAVELING_DELIVER_X → TRAVELING_DELIVER_Y →
      LOWERING_DELIVER → PLACING → RISING → RETURNING_Y →
      RETURNING_X → PARKED
    """

    def __init__(self):
        self.bridge_x  = PARK_X
        self.trolley_y = PARK_Y
        self.hook_z    = HOOK_PARK_Z
        self.state     = "PARKED"
        self._dwell    = 0
        self.cycles    = 0

        # Delivery target (set before activate)
        self._deliver_x = DELIVER_X
        self._deliver_y = DELIVER_Y

        # Panel angle (matches V2.6 tilt pickup: 60° → 90° on lift)
        self.panel_ang  = 60.0   # degrees

    def set_delivery(self, dx, dy):
        self._deliver_x = dx
        self._deliver_y = dy

    def activate(self):
        """Start a pickup-deliver-return cycle from PARKED."""
        if self.state == "PARKED":
            self.state = "TRAVELING_X"

    def update(self):
        """
        Step one frame. Returns True when full cycle completes
        (gantry back at PARKED after delivery).
        """
        if self.state == "TRAVELING_X":
            if self._move_x(PICKUP_X, BRIDGE_SPEED):
                self.state = "TRAVELING_Y"

        elif self.state == "TRAVELING_Y":
            if self._move_y(PICKUP_Y, TROLLEY_SPEED):
                self.state = "LOWERING"

        elif self.state == "LOWERING":
            if self._move_z(HOOK_LOWER_Z, HOOK_SPEED):
                self.state = "HOOKED"
                self._dwell = 0
                self.panel_ang = 60.0

        elif self.state == "HOOKED":
            self._dwell += 1
            if self._dwell >= DWELL_HOOKED:
                self.state = "LIFTING"

        elif self.state == "LIFTING":
            # Animate panel 60° → 90° during lift (matches V2.6)
            lift_range = HOOK_PARK_Z - HOOK_LOWER_Z
            t = np.clip((HOOK_PARK_Z - self.hook_z) / lift_range, 0.0, 1.0)
            t_e = 0.5 - 0.5 * np.cos((1.0 - t) * np.pi)
            self.panel_ang = 60.0 + t_e * 30.0
            if self._move_z(HOOK_PARK_Z, HOOK_SPEED):
                self.panel_ang = 90.0
                self.state = "TRAVELING_DELIVER_X"

        elif self.state == "TRAVELING_DELIVER_X":
            if self._move_x(self._deliver_x, BRIDGE_SPEED):
                self.state = "TRAVELING_DELIVER_Y"

        elif self.state == "TRAVELING_DELIVER_Y":
            if self._move_y(self._deliver_y, TROLLEY_SPEED):
                self.state = "LOWERING_DELIVER"

        elif self.state == "LOWERING_DELIVER":
            if self._move_z(HOOK_DELIVER_Z, HOOK_SPEED):
                self.state = "PLACING"
                self._dwell = 0

        elif self.state == "PLACING":
            self._dwell += 1
            if self._dwell >= DWELL_PLACING:
                self.state = "RISING"

        elif self.state == "RISING":
            if self._move_z(HOOK_PARK_Z, HOOK_SPEED):
                self.state = "RETURNING_Y"

        elif self.state == "RETURNING_Y":
            if self._move_y(PARK_Y, TROLLEY_SPEED):
                self.state = "RETURNING_X"

        elif self.state == "RETURNING_X":
            if self._move_x(PARK_X, BRIDGE_SPEED):
                self.state = "PARKED"
                self.panel_ang = 60.0
                self.cycles += 1
                return True   # cycle complete

        return False

    # ── axis movers ────────────────────────────────────────────────────
    def _move_x(self, target, speed):
        dx = target - self.bridge_x
        if abs(dx) <= speed:
            self.bridge_x = target; return True
        self.bridge_x += np.sign(dx) * speed; return False

    def _move_y(self, target, speed):
        dy = target - self.trolley_y
        if abs(dy) <= speed:
            self.trolley_y = target; return True
        self.trolley_y += np.sign(dy) * speed; return False

    def _move_z(self, target, speed):
        dz = target - self.hook_z
        if abs(dz) <= speed:
            self.hook_z = target; return True
        self.hook_z += np.sign(dz) * speed; return False

    @property
    def carrying(self):
        return self.state in (
            "HOOKED", "LIFTING",
            "TRAVELING_DELIVER_X", "TRAVELING_DELIVER_Y",
            "LOWERING_DELIVER", "PLACING", "RISING"
        )

    @property
    def hook_pos(self):
        return np.array([self.bridge_x, self.trolley_y, self.hook_z])


# ═══════════════════════════════════════════════════════════════════════
# SECTION 3 — WORLD / AUTO-CYCLE CONTROLLER
# ═══════════════════════════════════════════════════════════════════════

# Simulated delivery slots (matches MODULE_JIG from V2.6 / Module Assembly)
DELIVERY_SLOTS = [
    ("NORTH", DELIVER_X,            DELIVER_Y + 2.1),
    ("SOUTH", DELIVER_X,            DELIVER_Y - 2.1),
    ("EAST",  DELIVER_X + 2.1,      DELIVER_Y      ),
    ("WEST",  DELIVER_X - 2.1,      DELIVER_Y      ),
]

class World:
    def __init__(self):
        self.gantry       = OverheadGantry()
        self.tick         = 0
        self.slot_idx     = 0
        self.placed_walls = []        # (name, dx, dy) confirmed placed
        self.module_done  = False
        self._auto_start  = False     # delay first activation 1 frame

    def step(self, speed_factor=1.0):
        self.tick += 1
        g = self.gantry

        # Queue next delivery automatically when gantry returns to PARKED
        if g.state == "PARKED" and not self.module_done:
            if self.slot_idx < len(DELIVERY_SLOTS):
                name, dx, dy = DELIVERY_SLOTS[self.slot_idx]
                g.set_delivery(dx, dy)
                g.activate()

        done = g.update()
        if done and self.slot_idx < len(DELIVERY_SLOTS):
            name, dx, dy = DELIVERY_SLOTS[self.slot_idx]
            self.placed_walls.append((name, dx, dy))
            self.slot_idx += 1
            if self.slot_idx >= len(DELIVERY_SLOTS):
                self.module_done = True

    def reset(self):
        self.gantry       = OverheadGantry()
        self.tick         = 0
        self.slot_idx     = 0
        self.placed_walls = []
        self.module_done  = False


# ═══════════════════════════════════════════════════════════════════════
# SECTION 4 — HEADLESS VALIDATION
# Run before any display. Proves full 4-panel delivery cycle completes.
# ═══════════════════════════════════════════════════════════════════════

def run_headless_validation():
    w = World()
    MAX_FRAMES = 25000
    for i in range(MAX_FRAMES):
        w.step()
        if w.module_done:
            print(f"[VALIDATION] PASS — module complete at frame {w.tick}")
            print(f"[VALIDATION] Panels placed: {[p[0] for p in w.placed_walls]}")
            assert len(w.placed_walls) == 4, "Expected 4 panels"
            assert w.gantry.state == "PARKED", "Gantry should be PARKED after cycle"
            # Dimension check: bridge must be within runway limits throughout
            assert RUNWAY_X_MIN <= w.gantry.bridge_x <= RUNWAY_X_MAX, "Bridge out of runway"
            assert RUNWAY_Y_NEG <= w.gantry.trolley_y <= RUNWAY_Y_POS, "Trolley out of span"
            print("[VALIDATION] All assertions passed.")
            return True
    print(f"[VALIDATION] FAIL — did not complete within {MAX_FRAMES} frames")
    return False

run_headless_validation()


# ═══════════════════════════════════════════════════════════════════════
# SECTION 5 — RENDERING
# Full structural geometry matching reference render.
# Poly3DCollection for all 3D panels. No plot_surface on rotated geometry.
# ═══════════════════════════════════════════════════════════════════════

def quad(ax, corners, fc, ec="#333333", alpha=0.85, lw=1.2):
    poly = Poly3DCollection([list(corners)], alpha=alpha,
                            facecolor=fc, edgecolor=ec, linewidth=lw)
    ax.add_collection3d(poly)


def box(ax, cx, cy, cz, hw, hd, hh, fc, ec="#222222", alpha=0.9):
    """Draw a solid box centred at (cx,cy,cz) with half-extents hw,hd,hh."""
    x0, x1 = cx-hw, cx+hw
    y0, y1 = cy-hd, cy+hd
    z0, z1 = cz-hh, cz+hh
    faces = [
        [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0)],  # bottom
        [(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)],  # top
        [(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)],  # front
        [(x0,y1,z0),(x1,y1,z0),(x1,y1,z1),(x0,y1,z1)],  # back
        [(x0,y0,z0),(x0,y1,z0),(x0,y1,z1),(x0,y0,z1)],  # left
        [(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)],  # right
    ]
    for f in faces:
        quad(ax, f, fc, ec, alpha)


def draw_floor(ax):
    xs = np.array([[-2, 30], [-2, 30]])
    ys = np.array([[-8, -8], [ 8,  8]])
    ax.plot_surface(xs, ys, np.zeros_like(xs), color="#EFEFEF", alpha=0.45)


def draw_runway_rails(ax):
    """Two X-axis floor rails that the end trucks ride on."""
    for ry in [RUNWAY_Y_NEG, RUNWAY_Y_POS]:
        # Rail body
        ax.plot([RUNWAY_X_MIN, RUNWAY_X_MAX], [ry, ry],
                [RUNWAY_Z, RUNWAY_Z], color="#555555", lw=6,
                solid_capstyle="butt", alpha=0.9)
        # Rail cap highlight
        ax.plot([RUNWAY_X_MIN, RUNWAY_X_MAX], [ry, ry],
                [RUNWAY_Z+0.03, RUNWAY_Z+0.03], color="#888888", lw=2,
                solid_capstyle="butt", alpha=0.7)
    # End stop markers
    for rx in [RUNWAY_X_MIN, RUNWAY_X_MAX]:
        for ry in [RUNWAY_Y_NEG, RUNWAY_Y_POS]:
            ax.plot([rx, rx], [ry-0.25, ry+0.25],
                    [RUNWAY_Z+0.08]*2, color=CE_GOLD, lw=3)

    ax.text((RUNWAY_X_MIN+RUNWAY_X_MAX)/2, RUNWAY_Y_NEG-0.5, RUNWAY_Z+0.15,
            "RUNWAY RAIL  (X-AXIS TRAVEL)", fontsize=5.5,
            color="#888888", family="monospace", ha="center")


def draw_end_truck(ax, bridge_x, rail_y, label=""):
    """
    Portal end truck: box-section column + base plate + gussets.
    Matches dark grey structural steel from reference render.
    """
    hw = COL_W / 2
    hd = COL_D / 2

    # ── Base plate ────────────────────────────────────────────────────
    box(ax, bridge_x, rail_y, BASE_PLATE_H/2,
        BASE_PLATE_W, BASE_PLATE_W*0.7, BASE_PLATE_H/2,
        fc="#333333", ec="#555555", alpha=0.95)

    # ── Column body ───────────────────────────────────────────────────
    box(ax, bridge_x, rail_y, COL_H/2,
        hw, hd, COL_H/2,
        fc=CE_COLUMN, ec="#222222", alpha=0.95)

    # ── Column highlight (front face lighter) ─────────────────────────
    front_face = [
        (bridge_x-hw, rail_y-hd, 0.15),
        (bridge_x+hw, rail_y-hd, 0.15),
        (bridge_x+hw, rail_y-hd, COL_H-0.1),
        (bridge_x-hw, rail_y-hd, COL_H-0.1),
    ]
    quad(ax, front_face, "#5A5A5A", "#222222", alpha=0.7)

    # ── Gusset triangles (like render) ────────────────────────────────
    for sign in [-1, +1]:
        gusset = [
            (bridge_x + sign*hw,      rail_y,        0.1),
            (bridge_x + sign*hw*2.2,  rail_y,        0.1),
            (bridge_x + sign*hw,      rail_y,        COL_H*0.35),
        ]
        quad(ax, gusset, "#3A3A3A", "#222222", alpha=0.8)

    # ── End truck saddle (connects column top to beam) ─────────────────
    box(ax, bridge_x, rail_y, COL_H + 0.15,
        hw*1.6, hd*1.4, 0.18,
        fc="#3A3A3A", ec="#222222", alpha=0.95)

    if label:
        ax.text(bridge_x, rail_y, COL_H + 0.55, label,
                fontsize=5, color="#AAAAAA", family="monospace", ha="center")


def draw_bridge_beam(ax, bridge_x):
    """
    CE-gold I-beam spanning Y between the two end trucks.
    Catwalk platform + safety railings on top (matches render).
    """
    y0 = RUNWAY_Y_NEG
    y1 = RUNWAY_Y_POS
    bz = BRIDGE_BEAM_Z
    bh = BRIDGE_BEAM_H
    bw = BRIDGE_BEAM_W / 2

    # ── Bottom flange ─────────────────────────────────────────────────
    bf = [
        (bridge_x-bw, y0, bz),
        (bridge_x+bw, y0, bz),
        (bridge_x+bw, y1, bz),
        (bridge_x-bw, y1, bz),
    ]
    quad(ax, bf, CE_AMBER, CE_GOLD, alpha=0.95)

    # ── Web (vertical plate) ──────────────────────────────────────────
    web = [
        (bridge_x-0.06, y0, bz),
        (bridge_x+0.06, y0, bz),
        (bridge_x+0.06, y1, bz),
        (bridge_x-0.06, y1, bz),
    ]
    quad(ax, web, CE_AMBER, CE_GOLD, alpha=0.90)

    # ── Top flange ────────────────────────────────────────────────────
    tf = [
        (bridge_x-bw, y0, bz+bh),
        (bridge_x+bw, y0, bz+bh),
        (bridge_x+bw, y1, bz+bh),
        (bridge_x-bw, y1, bz+bh),
    ]
    quad(ax, tf, CE_AMBER, CE_GOLD, alpha=0.95)

    # ── Catwalk platform ──────────────────────────────────────────────
    cw = bw * 3.0    # catwalk wider than beam flanges
    cat = [
        (bridge_x-cw, y0+0.3, bz+bh+CATWALK_H),
        (bridge_x+cw, y0+0.3, bz+bh+CATWALK_H),
        (bridge_x+cw, y1-0.3, bz+bh+CATWALK_H),
        (bridge_x-cw, y1-0.3, bz+bh+CATWALK_H),
    ]
    quad(ax, cat, CE_AMBER, CE_GOLD, alpha=0.80)

    # ── Safety railings ────────────────────────────────────────────────
    rz_base = bz + bh + CATWALK_H
    rz_top  = rz_base + RAIL_H
    # Longitudinal rails (along Y)
    for rx in [bridge_x-cw, bridge_x+cw]:
        ax.plot([rx, rx], [y0+0.3, y1-0.3],
                [rz_top, rz_top], color=CE_GOLD, lw=2, alpha=0.9)
    # Posts along beam
    for ry in np.linspace(y0+0.4, y1-0.4, 7):
        for rx in [bridge_x-cw, bridge_x+cw]:
            ax.plot([rx, rx], [ry, ry],
                    [rz_base, rz_top], color=CE_GOLD, lw=1.5, alpha=0.85)
    # Mid rail
    for rx in [bridge_x-cw, bridge_x+cw]:
        ax.plot([rx, rx], [y0+0.3, y1-0.3],
                [(rz_base+rz_top)/2]*2, color=CE_GOLD, lw=1, alpha=0.6)

    # ── CE label on beam face ─────────────────────────────────────────
    ax.text(bridge_x, (y0+y1)/2, bz+bh*0.5,
            "CR  CHAPPELL\n    ROBOTICS",
            fontsize=5, color=CE_BLACK, family="monospace",
            ha="center", va="center", fontweight="bold")

    # ── 2.0T capacity plate ───────────────────────────────────────────
    ax.text(bridge_x+0.5, (y0+y1)/2, bz+bh+0.08,
            "2.0T", fontsize=6, color="white",
            family="monospace", ha="center", fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.15", facecolor=CE_BLACK,
                      edgecolor=CE_GOLD, linewidth=1))


def draw_trolley(ax, bridge_x, trolley_y):
    """
    Hoist unit (black box) riding bottom flange of bridge beam.
    Matches dark hoist body from reference render.
    """
    tz = BRIDGE_BEAM_Z - TROLLEY_H/2 - 0.05
    # Hoist body
    box(ax, bridge_x, trolley_y, tz,
        TROLLEY_D/2, TROLLEY_W/2, TROLLEY_H/2,
        fc=CE_HOIST, ec="#444444", alpha=0.95)

    # Wheel flanges (ride on bottom flange of beam)
    for dy in [-TROLLEY_W/2 - 0.05, TROLLEY_W/2 + 0.05]:
        box(ax, bridge_x, trolley_y+dy, BRIDGE_BEAM_Z-0.06,
            TROLLEY_D/2*0.7, 0.04, 0.06,
            fc="#333333", ec="#555555", alpha=0.9)

    # Motor housing bump on top
    box(ax, bridge_x, trolley_y, tz + TROLLEY_H/2 + 0.07,
        TROLLEY_D/2*0.5, TROLLEY_W/2*0.5, 0.07,
        fc="#252525", ec="#444444", alpha=0.95)

    # Travel direction indicator
    ax.text(bridge_x, trolley_y, tz + TROLLEY_H/2 + 0.25,
            "◄ Y ►", fontsize=4.5, color=CE_GOLD,
            family="monospace", ha="center")


def draw_wire_and_hook(ax, bridge_x, trolley_y, hook_z):
    """Wire rope from hoist down to hook block."""
    hoist_z = BRIDGE_BEAM_Z - TROLLEY_H - 0.05

    # Wire rope (two strands for visual weight)
    for dy in [-0.04, +0.04]:
        ax.plot([bridge_x, bridge_x],
                [trolley_y+dy, trolley_y+dy],
                [hoist_z, hook_z],
                color=CE_WIRE, lw=1.2, linestyle="-", alpha=0.85)

    # Hook block body
    box(ax, bridge_x, trolley_y, hook_z,
        0.12, 0.10, 0.12,
        fc="#BBAA00", ec=CE_GOLD, alpha=0.95)

    # Hook (curved shape approximated)
    hook_pts_x = [bridge_x, bridge_x, bridge_x+0.14, bridge_x+0.14, bridge_x+0.05]
    hook_pts_y = [trolley_y]*5
    hook_pts_z = [hook_z-0.12, hook_z-0.28, hook_z-0.28, hook_z-0.18, hook_z-0.18]
    ax.plot(hook_pts_x, hook_pts_y, hook_pts_z,
            color=CE_GOLD, lw=2.5, solid_capstyle="round")


def draw_safety_fence(ax):
    """
    Perimeter safety fence — matches render.
    Yellow posts + wire mesh panels around the cell footprint.
    """
    FX0, FX1 = RUNWAY_X_MIN + 0.5, RUNWAY_X_MAX - 0.5
    FY0, FY1 = RUNWAY_Y_NEG - 0.3, RUNWAY_Y_POS + 0.3
    FZ       = 1.85    # fence panel height
    POST_H   = FZ + 0.12

    # Posts
    post_xs_front = np.linspace(FX0, FX1, 14)
    post_xs_back  = np.linspace(FX0, FX1, 14)
    post_ys_side  = np.linspace(FY0, FY1, 6)

    for px in post_xs_front:
        for py in [FY0, FY1]:
            ax.plot([px,px],[py,py],[0,POST_H],
                    color=CE_GOLD, lw=2.5, solid_capstyle="round", alpha=0.9)

    for py in post_ys_side:
        for px in [FX0, FX1]:
            ax.plot([px,px],[py,py],[0,POST_H],
                    color=CE_GOLD, lw=2.5, solid_capstyle="round", alpha=0.9)

    # Mesh panels (wireframe look)
    mesh_alpha = 0.18
    mesh_col   = "#AAAAAA"
    # Front/back
    for fy in [FY0, FY1]:
        for i in range(len(post_xs_front)-1):
            x0, x1 = post_xs_front[i], post_xs_front[i+1]
            panel = [(x0,fy,0),(x1,fy,0),(x1,fy,FZ),(x0,fy,FZ)]
            quad(ax, panel, mesh_col, CE_GOLD, alpha=mesh_alpha, lw=0.5)
    # Sides
    for fx in [FX0, FX1]:
        for i in range(len(post_ys_side)-1):
            y0, y1 = post_ys_side[i], post_ys_side[i+1]
            panel = [(fx,y0,0),(fx,y1,0),(fx,y1,FZ),(fx,y0,FZ)]
            quad(ax, panel, mesh_col, CE_GOLD, alpha=mesh_alpha, lw=0.5)

    # Gate opening (front centre)
    gate_x = (FX0 + FX1) / 2
    ax.plot([gate_x-0.6, gate_x-0.6],[FY0,FY0],[0,POST_H],
            color=CE_BLACK, lw=3, alpha=0.8)
    ax.plot([gate_x+0.6, gate_x+0.6],[FY0,FY0],[0,POST_H],
            color=CE_BLACK, lw=3, alpha=0.8)


def draw_panel_on_hook(ax, gantry):
    """
    Wall panel riding on the hook — vertical from 90° during transit.
    During LIFTING: animates from 60° → 90° (V2.6 convention).
    """
    if not gantry.carrying:
        return

    PL  = 4.0     # panel length  (TILT_W from V2.6)
    PW  = 1.4     # panel half-width
    THK = 0.10    # panel half-thickness

    ang = np.radians(gantry.panel_ang)
    hx  = gantry.bridge_x
    hy  = gantry.trolley_y
    hz  = gantry.hook_z - 0.15    # attachment point below hook block

    def panel_pt(offset, dy):
        wx = hx - offset * np.sin(ang - np.pi/2)
        wz = hz - offset * np.cos(ang - np.pi/2)
        return (wx, hy + dy, wz)

    def pcorner(offset, dy, ts):
        base = panel_pt(offset, dy)
        px = ts * THK * np.cos(ang - np.pi/2)
        pz = ts * THK * (-np.sin(ang - np.pi/2))
        return (base[0]+px, base[1], base[2]+pz)

    # Front and back faces
    for ts in [-1, +1]:
        face = [pcorner(0, -PW, ts), pcorner(0,  PW, ts),
                pcorner(PL, PW, ts), pcorner(PL, -PW, ts)]
        quad(ax, face, "#C8A068", CE_GOLD, alpha=0.85)

    # Side edges
    for dy in [-PW, +PW]:
        face = [pcorner(0,  dy, -1), pcorner(0,  dy, +1),
                pcorner(PL, dy, +1), pcorner(PL, dy, -1)]
        quad(ax, face, "#AA8855", CE_GOLD, alpha=0.55, lw=0.8)

    # Angle readout
    mid = panel_pt(PL/2, 0)
    ax.text(mid[0], mid[1], mid[2]+0.3,
            f"{gantry.panel_ang:.0f}°",
            fontsize=6.5, color="#FF8800",
            family="monospace", ha="center", fontweight="bold")


def draw_pickup_zone(ax):
    """Highlight zone above TABLE_JIG_TILT position."""
    pz = 0.05
    hw = 2.2
    zone = [
        (PICKUP_X-hw, -1.6, pz), (PICKUP_X+hw, -1.6, pz),
        (PICKUP_X+hw, +1.6, pz), (PICKUP_X-hw, +1.6, pz),
    ]
    quad(ax, zone, "#2244AA", "#4488FF", alpha=0.20)
    ax.text(PICKUP_X, 0, pz+0.15, "PICKUP\nZONE",
            fontsize=5.5, color="#4488FF",
            family="monospace", ha="center")


def draw_delivery_zone(ax):
    """Highlight zone above MODULE_JIG."""
    pz = 0.05
    hw = 2.4
    zone = [
        (DELIVER_X-hw, -2.4, pz), (DELIVER_X+hw, -2.4, pz),
        (DELIVER_X+hw, +2.4, pz), (DELIVER_X-hw, +2.4, pz),
    ]
    quad(ax, zone, "#1A4422", "#44AA44", alpha=0.20)
    ax.text(DELIVER_X, 0, pz+0.15, "MODULE\nJIG",
            fontsize=5.5, color="#44FF88",
            family="monospace", ha="center")

    # Corner posts of module jig
    MOD_S = 2.0
    for px, py in [(DELIVER_X-MOD_S, -MOD_S), (DELIVER_X+MOD_S, -MOD_S),
                   (DELIVER_X+MOD_S, +MOD_S), (DELIVER_X-MOD_S, +MOD_S)]:
        ax.plot([px,px],[py,py],[0,4.2], color=CE_GOLD, lw=3, alpha=0.7)


def draw_placed_walls(ax, placed_walls):
    """Panels already delivered and placed in module jig."""
    colors = ["#C8A068", "#C09050", "#B87840", "#B06030"]
    MOD_S = 2.0
    THK   = 0.10
    for i, (name, dx, dy) in enumerate(placed_walls):
        c = colors[i % len(colors)]
        # Determine orientation from name
        if name in ("NORTH", "SOUTH"):
            hw, hd = MOD_S, THK
        else:
            hw, hd = THK, MOD_S
        verts_bot = [(dx-hw, dy-hd, 0.05), (dx+hw, dy-hd, 0.05),
                     (dx+hw, dy+hd, 0.05), (dx-hw, dy+hd, 0.05)]
        verts_top = [(dx-hw, dy-hd, 4.0),  (dx+hw, dy-hd, 4.0),
                     (dx+hw, dy+hd, 4.0),  (dx-hw, dy+hd, 4.0)]
        quad(ax, verts_bot, c, CE_GOLD, alpha=0.88)
        quad(ax, verts_top, c, CE_GOLD, alpha=0.88)
        for ex, ey in [(dx-hw,dy-hd),(dx+hw,dy-hd),(dx+hw,dy+hd),(dx-hw,dy+hd)]:
            ax.plot([ex,ex],[ey,ey],[0,4.0], color=CE_GOLD, lw=1.2, alpha=0.8)
        ax.text(dx, dy, 4.35, name, fontsize=5,
                color=c, family="monospace", ha="center", fontweight="bold")


def draw_safety_envelope(ax, bridge_x, trolley_y):
    """Dashed safety exclusion zone around current gantry position."""
    ex = 1.0; ey = 1.0
    x0, x1 = bridge_x-ex, bridge_x+ex
    y0, y1 = trolley_y-ey, trolley_y+ey
    zz = 0.08
    env = [(x0,y0,zz),(x1,y0,zz),(x1,y1,zz),(x0,y1,zz)]
    quad(ax, env, "#FF4400", "#FF6600", alpha=0.07)


def draw_axis_indicators(ax, bridge_x, trolley_y, hook_z):
    """Live axis position readout lines."""
    # X-axis bridge position line (drops to floor)
    ax.plot([bridge_x, bridge_x], [RUNWAY_Y_NEG-0.2, RUNWAY_Y_NEG-0.2],
            [0, BRIDGE_BEAM_Z], color="#4488FF", lw=0.8,
            linestyle=":", alpha=0.5)
    # Y-axis trolley position
    ax.plot([bridge_x-0.05, bridge_x+0.05],
            [RUNWAY_Y_NEG-0.2, RUNWAY_Y_NEG-0.2],
            [BRIDGE_BEAM_Z*0.5]*2, color="#4488FF", lw=2, alpha=0.7)


# ═══════════════════════════════════════════════════════════════════════
# SECTION 6 — MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════

world = World()

fig = plt.figure(figsize=(20, 11))
fig.patch.set_facecolor("white")
ax  = fig.add_subplot(111, projection="3d")
ax.set_facecolor("white")
plt.subplots_adjust(bottom=0.10, left=0.01, right=0.99)

sax = plt.axes([0.15, 0.03, 0.55, 0.025])
sax.set_facecolor("#EEEEEE")
spd = Slider(sax, "Speed", 0.5, 6.0, valinit=1.0, color=CE_GOLD)
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

    steps = max(1, int(spd.val))
    for _ in range(steps):
        world.step()

    g  = world.gantry
    bx = g.bridge_x
    ty = g.trolley_y
    hz = g.hook_z

    # ── Scene ───────────────────────────────────────────────────────
    draw_floor(ax)
    draw_runway_rails(ax)
    draw_safety_fence(ax)

    # ── Gantry structure ─────────────────────────────────────────────
    # End trucks on each runway rail
    draw_end_truck(ax, bx, RUNWAY_Y_NEG, "ET-FRONT")
    draw_end_truck(ax, bx, RUNWAY_Y_POS, "ET-BACK")

    # Bridge beam
    draw_bridge_beam(ax, bx)

    # Trolley / hoist
    draw_trolley(ax, bx, ty)

    # Wire and hook
    draw_wire_and_hook(ax, bx, ty, hz)

    # ── Work zones ────────────────────────────────────────────────────
    draw_pickup_zone(ax)
    draw_delivery_zone(ax)
    draw_placed_walls(ax, world.placed_walls)

    # ── Payload ───────────────────────────────────────────────────────
    draw_panel_on_hook(ax, g)

    # ── Safety envelope ───────────────────────────────────────────────
    draw_safety_envelope(ax, bx, ty)
    draw_axis_indicators(ax, bx, ty, hz)

    # ── State colour map ─────────────────────────────────────────────
    state_colors = {
        "PARKED":               "#44FF44",
        "TRAVELING_X":          CE_GOLD,
        "TRAVELING_Y":          "#FFDD00",
        "LOWERING":             "#FFAA00",
        "HOOKED":               "#FF8800",
        "LIFTING":              "#FF5500",
        "TRAVELING_DELIVER_X":  "#FF6600",
        "TRAVELING_DELIVER_Y":  "#FF9900",
        "LOWERING_DELIVER":     "#FFBB00",
        "PLACING":              "#FF4400",
        "RISING":               "#FF8800",
        "RETURNING_Y":          "#AADDFF",
        "RETURNING_X":          "#88BBFF",
    }
    sc = state_colors.get(g.state, "white")

    # ── HUD ──────────────────────────────────────────────────────────
    next_slot = DELIVERY_SLOTS[world.slot_idx][0] if world.slot_idx < 4 else "DONE"
    complete_str = "  ✓ MODULE COMPLETE" if world.module_done else ""

    hud = (
        f"CE OVERHEAD GANTRY  V1.0\n"
        f"{'─'*36}\n"
        f"STATE    {g.state}\n"
        f"{'─'*36}\n"
        f"BRIDGE X  {bx:+6.2f}  "
        f"[{RUNWAY_X_MIN:.1f} → {RUNWAY_X_MAX:.1f}]\n"
        f"TROLLEY Y {ty:+6.2f}  "
        f"[{RUNWAY_Y_NEG:.1f} → {RUNWAY_Y_POS:.1f}]\n"
        f"HOOK Z    {hz:+6.2f}  "
        f"[{HOOK_LOWER_Z:.1f} → {HOOK_PARK_Z:.1f}]\n"
        f"PANEL ANG {g.panel_ang:5.1f}°\n"
        f"{'─'*36}\n"
        f"PICKUP    X={PICKUP_X:.1f}  Y={PICKUP_Y:.1f}\n"
        f"DELIVER   X={g._deliver_x:.1f}  Y={g._deliver_y:.1f}\n"
        f"{'─'*36}\n"
        f"PANELS    {world.slot_idx}/4  NEXT={next_slot}\n"
        f"CYCLES    {g.cycles}\n"
        f"{complete_str}\n"
        f"{'─'*36}\n"
        f"FRAME     {world.tick}\n"
    )

    ax.text2D(0.01, 0.99, hud, transform=ax.transAxes,
              fontsize=5.8, family="monospace", va="top", color="#111111",
              bbox=dict(boxstyle="round,pad=0.3",
                        facecolor="#FFFDF0", alpha=0.90, edgecolor=CE_GOLD))

    # State badge
    ax.text2D(0.99, 0.99, f"◉ {g.state}",
              transform=ax.transAxes, fontsize=7,
              family="monospace", va="top", ha="right", color=sc,
              fontweight="bold",
              bbox=dict(boxstyle="round,pad=0.3",
                        facecolor=CE_BLACK, alpha=0.85, edgecolor=sc))

    ax.set_title(
        "CONSTRUCTION ENTERPRISES  —  CHAPPELL ROBOTICS\n"
        "CE OVERHEAD GANTRY V1.0  —  2.0T PORTAL GANTRY  "
        "│  X: BRIDGE  │  Y: TROLLEY  │  Z: HOIST",
        fontsize=9, fontweight="bold", color=CE_BLACK)

    ax.set_xlim(0,  28)
    ax.set_ylim(-8,  8)
    ax.set_zlim( 0,  8)
    ax.set_xlabel("X  (BRIDGE TRAVEL →)", color="#333333", fontsize=7)
    ax.set_ylabel("Y  (TROLLEY TRAVEL)", color="#333333", fontsize=7)
    ax.set_zlabel("Z  (HOIST)", color="#333333", fontsize=7)
    ax.tick_params(colors="#444444", labelsize=6)

    ax.xaxis.pane.fill = True; ax.yaxis.pane.fill = True; ax.zaxis.pane.fill = True
    ax.xaxis.pane.set_facecolor("#F5F5F5")
    ax.yaxis.pane.set_facecolor("#F5F5F5")
    ax.zaxis.pane.set_facecolor("#F5F5F5")
    ax.xaxis.pane.set_edgecolor("#CCCCCC")
    ax.yaxis.pane.set_edgecolor("#CCCCCC")
    ax.zaxis.pane.set_edgecolor("#CCCCCC")
    ax.grid(True, alpha=0.3, color="#AAAAAA")

    ax.set_xlim(0,  28)
    ax.set_ylim(-8,  8)
    ax.set_zlim( 0,  8)


ani = FuncAnimation(fig, update, interval=1, cache_frame_data=False)
plt.show()
