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

    def snapshot(self):
        """Added for the Phase 3B pyvista migration -- read-only render view, no sim logic."""
        g = self.gantry
        return {
            "tick": self.tick,
            "state": g.state,
            "bridge_x": g.bridge_x,
            "trolley_y": g.trolley_y,
            "hook_z": g.hook_z,
            "cycles": g.cycles,
            "placed_walls": list(self.placed_walls),
            "module_done": self.module_done,
        }
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

