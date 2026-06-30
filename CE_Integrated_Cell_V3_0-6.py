"""
CONSTRUCTION ENTERPRISES — CHAPPELL ROBOTICS
CE_Integrated_Cell  V3.0

INTEGRATED MODULAR WALL MANUFACTURING CELL
  Complete digital twin combining all 4 validated subsystems.
  Reference: Chappell Robotics integrated cell render (June 2026).

MANUFACTURING FLOW
  1. CR6 Rail System    — 4 robots assemble LGS panel on TABLE_JIG_FIXED
  2. Roller Transfer    — Panel moves from FIXED → TILT (X=-1.0)
  3. Tilt Table         — Panel raised 0°→60° for gantry pickup (X=5.5)
  4. Overhead Gantry    — 2.0T crane lifts panel, delivers to MODULE_JIG (X=22.0)

SUBSYSTEMS INTEGRATED
  CE_Overhead_Gantry_V1.py   — Phase 1  (frame 3792)
  CE_Rail_System_V1.py       — Phase 2  (frame 641)
  CE_TableJig_Tilt_V1.py     — Phase 3  (frame 792)
  CE_TableJig_Roller_V1.py   — Phase 4  (frame 1037)

WORLD COORDINATE SPACE
  TABLE_JIG_FIXED   X=-6.0, Y=0, Z=0.9
  TABLE_JIG_ROLLER  X=-1.0, Y=0, Z=0.9
  TABLE_JIG_TILT    X=+5.5, Y=0, Z=0.9  (PIVOT_X=8.8)
  MODULE_JIG        X=+22.0, Y=0
  Gantry Park       X=+14.0, Y=0
  Runway Rails      Y=±4.5
  CR6 Rails         Y=±1.55

INTER-SUBSYSTEM TRIGGERS
  Rail WORKING complete (cycles≥1)  → Roller RECEIVING
  Roller DELIVERED                  → Tilt TILTING_UP
  Tilt HELD_AT_60                   → Gantry activate (TRAVELING_X)
  Gantry PARKED (cycle complete)    → Rail next cycle

PLATFORM  Android · Pydroid 3 · Python 3 · NumPy · Matplotlib
VALIDATED  Full end-to-end cycle headless before display.
"""

import os
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ═══════════════════════════════════════════════════════════════════════
# SECTION 1 — SHARED CONSTANTS
# ═══════════════════════════════════════════════════════════════════════

# ── CE aesthetics ─────────────────────────────────────────────────────
CE_GOLD    = "#CC6600"
CE_AMBER   = "#E8A020"
CE_BLACK   = "#1A1A1A"
CE_DARK    = "#2A2A2A"
CE_COLUMN  = "#4A4A4A"
CE_HOIST   = "#1E1E1E"
CE_WIRE    = "#888888"
CE_BASE    = "#2A2A2A"
CE_ROLLER_C= "#AAAAAA"
CE_CAP     = "#E8A020"
CE_MOTOR   = "#1A1A1A"
CE_CHAIN   = "#444444"
CE_PANEL_T = "#C8D8E8"
CE_FENCE_P = "#E8A020"
CE_FENCE_M = "#888888"
CE_LEAF    = "#333333"
CE_HYD_B   = "#3A3A3A"
CE_HYD_R   = "#AAAAAA"
CE_CLAMP   = "#E8A020"
CE_DECK    = "#D4A96A"
CE_PANEL_C = "#C8A44A"

# ── World positions ───────────────────────────────────────────────────
# Jig assembly shifted +5 units toward MODULE_JIG (50% of 10 unit gap)

FIXED_CX   =  8.3;  FIXED_CY  = 0.0;  FIXED_Z  = 0.9
FIXED_W    =  6.6;  FIXED_D   = 3.8

ROLLER_CX  = 14.9;  ROLLER_CY = 0.0;  ROLLER_Z = 0.9
ROLLER_W   =  6.6;  ROLLER_D  = 3.8
ROLLER_COUNT = 20;  ROLLER_R  = 0.05; ROLLER_SPACING = 0.35

TILT_CX    = 21.5;  TILT_CY   = 0.0;  TILT_Z   = 0.9
TILT_W     =  6.6;  TILT_D    = 3.8
TILT_BASE_W=  6.6;  TILT_BASE_D=3.8;  TILT_BASE_H=0.9
TILT_THICK =  0.18
PIVOT_X    =  TILT_CX + TILT_BASE_W / 2   # = 24.8
PIVOT_Z    =  TILT_Z
MAX_TILT   = 60.0

MOD_CX     = 32.0;  MOD_CY    = 0.0
MOD_S      =  2.2

RUNWAY_X_MIN = 1.0;  RUNWAY_X_MAX = 36.0
RUNWAY_Y_NEG =-4.5;  RUNWAY_Y_POS = 4.5
BRIDGE_BEAM_Z= 7.8;  BRIDGE_BEAM_H= 0.55  # raised to clear tilt leaf at 60°
COL_H        = BRIDGE_BEAM_Z
HOOK_PARK_Z  = BRIDGE_BEAM_Z - 0.3   # = 7.5

# ── Pickup height on tilted leaf ──────────────────────────────────────
# Panel sits on leaf at 60°. Hook attaches near the free end of the leaf.
# Free end position at 60°:
#   X = PIVOT_X - TILT_W*cos(60°) = 24.8 - 6.6*0.5 = 21.5  (= TILT_CX ✓)
#   Z = PIVOT_Z + TILT_W*sin(60°) = 0.9 + 6.6*0.866 = 6.62
# Hook attaches at ~70% up the leaf (panel attachment rigging point):
#   u = TILT_W * 0.70 = 4.62
#   Z_attach = PIVOT_Z + u*sin(60°) = 0.9 + 4.0 = 4.9
# Add panel thickness and rigging: +0.35
HOOK_LOWER_Z  = 5.25   # hook stops here to pick up panel on 60° leaf
HOOK_DELIVER_Z= 1.20   # delivery into MODULE_JIG at floor level

RAIL_A_Y   = -1.55;  RAIL_B_Y  = 1.55
RAIL_X_MIN =  5.0;   RAIL_X_MAX= 11.6   # spans FIXED table (FIXED_CX ± FIXED_W/2)
ATC_NEAR_X =  5.5;   ATC_FAR_X = 11.1

PANEL_SPEED = 0.025
TILT_SPEED  = 0.8
BRIDGE_SPEED= 0.07
TROLLEY_SPEED=0.06
HOOK_SPEED  = 0.045

# ── Rendering helpers ─────────────────────────────────────────────────
def quad(ax, corners, fc, ec="#222222", alpha=0.88, lw=0.8):
    poly = Poly3DCollection([list(corners)], alpha=alpha,
                            facecolor=fc, edgecolor=ec, linewidth=lw)
    ax.add_collection3d(poly)

def box(ax, cx, cy, cz, hw, hd, hh, fc, ec="#1A1A1A", alpha=0.92):
    x0,x1=cx-hw,cx+hw; y0,y1=cy-hd,cy+hd; z0,z1=cz-hh,cz+hh
    for f in [
        [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0)],
        [(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)],
        [(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)],
        [(x0,y1,z0),(x1,y1,z0),(x1,y1,z1),(x0,y1,z1)],
        [(x0,y0,z0),(x0,y1,z0),(x0,y1,z1),(x0,y0,z1)],
        [(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)],
    ]: quad(ax,f,fc,ec,alpha)

# ═══════════════════════════════════════════════════════════════════════
# SECTION 2 — STATE MACHINES
# ═══════════════════════════════════════════════════════════════════════

class RailSubsystem:
    def __init__(self, y, x_min, x_max):
        self.y     = y
        self.x_min = x_min
        self.x_max = x_max

class OverheadGantry:
    def __init__(self):
        self.bridge_x=28.0; self.trolley_y=0.0; self.hook_z=HOOK_PARK_Z
        self.state="PARKED"; self._dwell=0; self.cycles=0; self.panel_ang=60.0
        self._deliver_x=MOD_CX; self._deliver_y=MOD_CY
        # FIX 2: pickup attachment point on leaf — stored at HOOKED state
        PICKUP_U=TILT_W*0.70   # 70% up leaf from pivot = 4.62m
        self.pickup_x=PIVOT_X-PICKUP_U*np.cos(np.radians(60))  # = 22.49
        self.pickup_z=PIVOT_Z+PICKUP_U*np.sin(np.radians(60))  # = 4.90
        self.pickup_ang=60.0

    def activate(self):
        if self.state=="PARKED": self.state="TRAVELING_X"

    def update(self):
        if self.state=="TRAVELING_X":
            if self._move_x(21.5,BRIDGE_SPEED): self.state="TRAVELING_Y"  # TILT_CX
        elif self.state=="TRAVELING_Y":
            if self._move_y(0.0,TROLLEY_SPEED): self.state="LOWERING"
        elif self.state=="LOWERING":
            if self._move_z(HOOK_LOWER_Z,HOOK_SPEED): self.state="HOOKED"; self._dwell=0; self.panel_ang=60.0
        elif self.state=="HOOKED":
            self._dwell+=1
            if self._dwell>=18: self.state="LIFTING"
        elif self.state=="LIFTING":
            lr=HOOK_PARK_Z-HOOK_LOWER_Z; t=np.clip((HOOK_PARK_Z-self.hook_z)/lr,0,1)
            t_e=0.5-0.5*np.cos((1.0-t)*np.pi); self.panel_ang=60.0+t_e*30.0
            if self._move_z(HOOK_PARK_Z,HOOK_SPEED): self.panel_ang=90.0; self.state="TRAVELING_DELIVER_X"
        elif self.state=="TRAVELING_DELIVER_X":
            if self._move_x(self._deliver_x,BRIDGE_SPEED): self.state="TRAVELING_DELIVER_Y"
        elif self.state=="TRAVELING_DELIVER_Y":
            if self._move_y(self._deliver_y,TROLLEY_SPEED): self.state="LOWERING_DELIVER"
        elif self.state=="LOWERING_DELIVER":
            if self._move_z(HOOK_DELIVER_Z,HOOK_SPEED): self.state="PLACING"; self._dwell=0
        elif self.state=="PLACING":
            self._dwell+=1
            if self._dwell>=24: self.state="RISING"
        elif self.state=="RISING":
            if self._move_z(HOOK_PARK_Z,HOOK_SPEED): self.state="RETURNING_Y"
        elif self.state=="RETURNING_Y":
            if self._move_y(0.0,TROLLEY_SPEED): self.state="RETURNING_X"
        elif self.state=="RETURNING_X":
            if self._move_x(28.0,BRIDGE_SPEED):  # park mid-span between TILT and MOD
                self.state="PARKED"; self.panel_ang=60.0; self.cycles+=1; return True
        return False

    def _move_x(self,t,s):
        d=t-self.bridge_x
        if abs(d)<=s: self.bridge_x=t; return True
        self.bridge_x+=np.sign(d)*s; return False
    def _move_y(self,t,s):
        d=t-self.trolley_y
        if abs(d)<=s: self.trolley_y=t; return True
        self.trolley_y+=np.sign(d)*s; return False
    def _move_z(self,t,s):
        d=t-self.hook_z
        if abs(d)<=s: self.hook_z=t; return True
        self.hook_z+=np.sign(d)*s; return False

    @property
    def carrying(self):
        # Panel released at PLACING — not carried during RISING/RETURNING
        return self.state in ("HOOKED","LIFTING","TRAVELING_DELIVER_X",
                              "TRAVELING_DELIVER_Y","LOWERING_DELIVER")


class CR6Robot:
    def __init__(self,name,rail,home_x,work_x,atc_x,side):
        self.name=name; self.rail=rail; self.x=home_x
        self.work_x=work_x; self.atc_x=atc_x; self.side=side
        self.state="PARKED_AT_ATC"; self._dwell=0; self.cycles=0
        self.tool_idx=0; self._active=False

    @property
    def rail_y(self): return self.rail.y

    def activate(self):
        if self.state=="PARKED_AT_ATC": self.state="TRAVELING_TO_WORK"; self._active=True

    def update(self):
        if not self._active: return False
        if self.state=="TRAVELING_TO_WORK":
            if self._move(self.work_x): self.state="WORKING"; self._dwell=0
        elif self.state=="WORKING":
            self._dwell+=1
            if self._dwell>=50: self.state="TRAVELING_TO_ATC"
        elif self.state=="TRAVELING_TO_ATC":
            if self._move(self.atc_x): self.state="AT_ATC"; self._dwell=0
        elif self.state=="AT_ATC":
            self._dwell+=1
            if self._dwell>=15: self.state="TOOL_CHANGE"; self._dwell=0
        elif self.state=="TOOL_CHANGE":
            self._dwell+=1
            if self._dwell>=15:
                self.tool_idx=(self.tool_idx+1)%5; self.state="PARKED_AT_ATC"
                self.cycles+=1; return True
        return False

    def _move(self,t):
        d=t-self.x
        if abs(d)<=0.06: self.x=t; return True
        self.x+=np.sign(d)*0.06; return False

    @property
    def working(self): return self.state=="WORKING"
    @property
    def changing_tool(self): return self.state in ("AT_ATC","TOOL_CHANGE")


class RollerTable:
    def __init__(self):
        self.state="IDLE"; self.panel_x=ROLLER_CX-ROLLER_W/2+0.3
        self.roller_angle=0.0; self._dwell=0; self.cycles=0; self.speed=0.0
        self.travel_pct=0.0; self._active=False
        self.panel_x_start=ROLLER_CX-ROLLER_W/2+0.3
        self.panel_x_end  =ROLLER_CX+ROLLER_W/2-0.3

    def activate(self):
        if self.state=="IDLE":
            self.state="RECEIVING"; self._active=True; self._dwell=0
            self.panel_x=self.panel_x_start

    def update(self):
        if not self._active: return False
        rng=self.panel_x_end-self.panel_x_start
        if self.state=="RECEIVING":
            self._dwell+=1
            if self._dwell>=30: self.state="TRANSFER_FORWARD"; self._dwell=0
        elif self.state=="TRANSFER_FORWARD":
            self.speed=PANEL_SPEED; self.panel_x+=PANEL_SPEED
            self.roller_angle=(self.roller_angle+3.5)%360
            self.travel_pct=np.clip((self.panel_x-self.panel_x_start)/rng*100,0,100)
            if self.panel_x>=self.panel_x_end:
                self.panel_x=self.panel_x_end; self.travel_pct=100.0
                self.state="DELIVERED"; self._dwell=0; self.speed=0.0
        elif self.state=="DELIVERED":
            self._dwell+=1
            if self._dwell>=25: self.state="RETURNING"; self._dwell=0
        elif self.state=="RETURNING":
            self.panel_x=self.panel_x_start; self.travel_pct=0.0
            self.state="IDLE"; self.cycles+=1; self._active=False; return True
        return False

    @property
    def rollers_spinning(self): return self.state=="TRANSFER_FORWARD"
    @property
    def panel_visible(self): return self.state in ("RECEIVING","TRANSFER_FORWARD","DELIVERED")


class TiltTable:
    def __init__(self):
        self.angle_deg=0.0; self.state="FLAT"; self._dwell=0
        self.cycles=0; self.pin_extended=True; self._active=False

    def activate(self):
        if self.state=="FLAT": self._active=True; self.state="TILTING_UP"

    def update(self):
        if not self._active: return False
        if self.state=="TILTING_UP":
            self.angle_deg+=TILT_SPEED
            if self.angle_deg>=MAX_TILT:
                self.angle_deg=MAX_TILT; self.state="HELD_AT_60"; self._dwell=0
        elif self.state=="HELD_AT_60":
            pass   # holds until release_panel() called externally
        elif self.state=="CRANE_PICKUP":
            pass   # holds until return_home() called externally
        elif self.state=="RETURNING_DOWN":
            self.angle_deg -= TILT_SPEED
            if self.angle_deg <= 0.0:
                self.angle_deg=0.0; self.state="FLAT"; self.pin_extended=True
                self.cycles+=1; self._active=False; return True
        return False

    def release_panel(self):
        """Called when gantry enters LIFTING — panel secured, pins retract."""
        if self.state == "HELD_AT_60":
            self.state = "CRANE_PICKUP"
            self.pin_extended = False

    def return_home(self):
        """Called when gantry clears (TRAVELING_DELIVER_X) — tilt can return."""
        if self.state == "CRANE_PICKUP":
            self.state = "RETURNING_DOWN"

    @property
    def angle_rad(self): return np.radians(self.angle_deg)

    def leaf_corners(self):
        ang=self.angle_rad; hw=TILT_D/2
        def lp(u,dv):
            return (PIVOT_X-u*np.cos(ang), TILT_CY+dv, PIVOT_Z+u*np.sin(ang))
        return lp(0,-hw),lp(0,+hw),lp(TILT_W,+hw),lp(TILT_W,-hw)

    def cyl_tip(self,y_off):
        ang=self.angle_rad; u=TILT_BASE_W*0.55
        return (PIVOT_X-u*np.cos(ang), TILT_CY+y_off, PIVOT_Z+u*np.sin(ang)-TILT_THICK*0.5)

    def pin_pos(self,u_off,v_off):
        ang=self.angle_rad; u=TILT_W/2-u_off
        wx=PIVOT_X-u*np.cos(ang); wz=PIVOT_Z+u*np.sin(ang)
        ph=0.28 if self.pin_extended else 0.08
        return (wx,TILT_CY+v_off,wz),(wx+ph*np.sin(ang),TILT_CY+v_off,wz+ph*np.cos(ang))


# ═══════════════════════════════════════════════════════════════════════
# SECTION 3 — INTEGRATED WORLD CONTROLLER
# ═══════════════════════════════════════════════════════════════════════

class IntegratedCell:
    """
    Master controller — coordinates all 4 subsystems.
    Inter-subsystem trigger logic lives here.
    """
    def __init__(self):
        # Subsystem state machines
        self.gantry = OverheadGantry()
        self.roller  = RollerTable()
        self.tilt    = TiltTable()

        # CR6 robots — Rail System
        self.rail_A = RailSubsystem(RAIL_A_Y, RAIL_X_MIN, RAIL_X_MAX)
        self.rail_B = RailSubsystem(RAIL_B_Y, RAIL_X_MIN, RAIL_X_MAX)
        self.A1 = CR6Robot("A1", self.rail_A, ATC_NEAR_X, FIXED_CX-1.2, ATC_NEAR_X, +1)
        self.A2 = CR6Robot("A2", self.rail_A, ATC_FAR_X,  FIXED_CX+1.2, ATC_FAR_X,  +1)
        self.B1 = CR6Robot("B1", self.rail_B, ATC_NEAR_X, FIXED_CX-1.2, ATC_NEAR_X, -1)
        self.B2 = CR6Robot("B2", self.rail_B, ATC_FAR_X,  FIXED_CX+1.2, ATC_FAR_X,  -1)
        self.robots = [self.A1, self.A2, self.B1, self.B2]

        self.tick          = 0
        self.placed_walls  = []
        self.module_done   = False
        self._phase        = "RAIL_WORKING"   # master phase tracker
        self._start_delays = {"A1":5,"A2":18,"B1":11,"B2":25}
        self._rail_cycles_needed = 2   # robot cycles before roller starts

    def step(self):
        self.tick += 1

        # ── Phase: RAIL_WORKING ──────────────────────────────────────
        if self._phase == "RAIL_WORKING":
            for r in self.robots:
                delay = self._start_delays.get(r.name, 0)
                if self.tick >= delay and r.state == "PARKED_AT_ATC":
                    r.activate()
                r.update()
            # Transition: enough robot cycles done → start roller
            if all(r.cycles >= self._rail_cycles_needed for r in self.robots):
                self._phase = "ROLLER_TRANSFER"
                self.roller.activate()

        # ── Phase: ROLLER_TRANSFER ────────────────────────────────────
        elif self._phase == "ROLLER_TRANSFER":
            for r in self.robots:
                if r.state == "PARKED_AT_ATC": r.activate()
                r.update()
            done = self.roller.update()
            # FIX Issue 3: wait for roller RETURNING complete (panel fully on tilt)
            # not DELIVERED — panel is still on roller at that point
            if done:   # done = True only when state returns to IDLE after RETURNING
                self._phase = "TILT_RAISING"
                if self.tilt.state == "FLAT":
                    self.tilt.activate()

        # ── Phase: TILT_RAISING ───────────────────────────────────────
        elif self._phase == "TILT_RAISING":
            for r in self.robots:
                if r.state == "PARKED_AT_ATC": r.activate()
                r.update()
            self.roller.update()
            self.tilt.update()
            if self.tilt.state == "HELD_AT_60":
                self._phase = "GANTRY_PICKUP"
                self.gantry.activate()

        # ── Phase: GANTRY_PICKUP ──────────────────────────────────────
        elif self._phase == "GANTRY_PICKUP":
            for r in self.robots:
                if r.state == "PARKED_AT_ATC": r.activate()
                r.update()
            self.tilt.update()
            done = self.gantry.update()

            # FIX Issue 2: signal tilt based on gantry state
            # Panel secured = gantry in LIFTING → release pins, tilt can start returning
            if self.gantry.state == "LIFTING":
                self.tilt.release_panel()   # pins retract, tilt enters CRANE_PICKUP

            # Gantry clear = traveling to deliver → tilt can return home
            if self.gantry.state == "TRAVELING_DELIVER_X":
                self.tilt.return_home()     # tilt starts RETURNING_DOWN

            if done:
                self.placed_walls.append(f"PANEL_{self.gantry.cycles}")
                self._phase = "RETURNING"

        # ── Phase: RETURNING ─────────────────────────────────────────
        elif self._phase == "RETURNING":
            for r in self.robots:
                if r.state == "PARKED_AT_ATC": r.activate()
                r.update()
            self.tilt.update()
            if self.tilt.state == "FLAT" and self.gantry.state == "PARKED":
                if len(self.placed_walls) >= 4:
                    self.module_done = True
                    self._phase = "COMPLETE"
                else:
                    # Reset for next panel
                    self._rail_cycles_needed += 2
                    self._phase = "RAIL_WORKING"

        # ── Phase: COMPLETE ───────────────────────────────────────────
        elif self._phase == "COMPLETE":
            pass  # Cell holds — module done

    def reset(self):
        self.__init__()


# ═══════════════════════════════════════════════════════════════════════
# SECTION 4 — HEADLESS VALIDATION
# ═══════════════════════════════════════════════════════════════════════

def run_headless_validation():
    cell = IntegratedCell()
    MAX_FRAMES = 40000
    for _ in range(MAX_FRAMES):
        cell.step()
        if cell.gantry.cycles >= 1:
            print(f"[VALIDATION] PASS — first full cell cycle at frame {cell.tick}")
            print(f"[VALIDATION] Gantry cycles: {cell.gantry.cycles}")
            print(f"[VALIDATION] Roller cycles: {cell.roller.cycles}")
            print(f"[VALIDATION] Tilt cycles:   {cell.tilt.cycles}")
            print(f"[VALIDATION] Walls placed:  {len(cell.placed_walls)}")
            assert cell.gantry.bridge_x >= RUNWAY_X_MIN, "Bridge out of runway"
            assert cell.gantry.bridge_x <= RUNWAY_X_MAX, "Bridge out of runway"
            assert cell.tilt.angle_deg  <= MAX_TILT,     "Tilt cap violated"
            print("[VALIDATION] All assertions passed.")
            return True
    print(f"[VALIDATION] FAIL — did not complete within {MAX_FRAMES} frames")
    return False

run_headless_validation()


# ═══════════════════════════════════════════════════════════════════════
# SECTION 5 — RENDERING
# ═══════════════════════════════════════════════════════════════════════

def draw_floor(ax):
    xs=np.array([[-2,38],[-2,38]]); ys=np.array([[-7,-7],[7,7]])
    ax.plot_surface(xs,ys,np.zeros_like(xs),color="#EFEFEF",alpha=0.40)

def draw_safety_fence(ax):
    """Single perimeter fence at gantry runway Y=±4.5."""
    FX0,FX1=RUNWAY_X_MIN-0.5,RUNWAY_X_MAX+0.5; FH=2.0; SP=0.9
    for fy in [RUNWAY_Y_NEG,RUNWAY_Y_POS]:
        pxs=np.arange(FX0,FX1+0.1,SP)
        for px in pxs:
            ax.plot([px,px],[fy,fy],[0,FH+0.1],color=CE_FENCE_P,lw=2.2,
                    solid_capstyle="round",alpha=0.88)
        for i in range(len(pxs)-1):
            quad(ax,[(pxs[i],fy,0),(pxs[i+1],fy,0),(pxs[i+1],fy,FH),(pxs[i],fy,FH)],
                 CE_FENCE_M,CE_FENCE_P,alpha=0.14,lw=0.4)
        for hz in [FH*0.35,FH*0.70,FH]:
            ax.plot([FX0,FX1],[fy,fy],[hz,hz],color=CE_FENCE_P,lw=1.0,alpha=0.55)
    # End fences
    for fx in [FX0,FX1]:
        pys=np.linspace(RUNWAY_Y_NEG,RUNWAY_Y_POS,6)
        for py in pys:
            ax.plot([fx,fx],[py,py],[0,FH+0.1],color=CE_FENCE_P,lw=2.2,alpha=0.80)
        for i in range(len(pys)-1):
            quad(ax,[(fx,pys[i],0),(fx,pys[i+1],0),(fx,pys[i+1],FH),(fx,pys[i],FH)],
                 CE_FENCE_M,CE_FENCE_P,alpha=0.12,lw=0.4)

def draw_runway_rails(ax):
    for ry in [RUNWAY_Y_NEG,RUNWAY_Y_POS]:
        ax.plot([RUNWAY_X_MIN,RUNWAY_X_MAX],[ry,ry],[0.05,0.05],
                color="#555555",lw=5,solid_capstyle="butt",alpha=0.9)
        ax.plot([RUNWAY_X_MIN,RUNWAY_X_MAX],[ry,ry],[0.08,0.08],
                color="#888888",lw=1.5,alpha=0.6)

def draw_gantry(ax,gantry):
    bx=gantry.bridge_x; ty=gantry.trolley_y; hz=gantry.hook_z
    # End trucks
    for ry in [RUNWAY_Y_NEG,RUNWAY_Y_POS]:
        hw=0.20; hd=0.25
        box(ax,bx,ry,COL_H/2,hw,hd,COL_H/2,CE_COLUMN,"#222222",0.95)
        box(ax,bx,ry,0.04,0.37,0.37*0.7,0.04,"#333333","#555555",0.90)
        box(ax,bx,ry,COL_H+0.15,hw*1.6,hd*1.4,0.18,"#3A3A3A","#222222",0.95)
    # Bridge beam
    y0,y1=RUNWAY_Y_NEG,RUNWAY_Y_POS; bz=BRIDGE_BEAM_Z; bw=0.15
    quad(ax,[(bx-bw,y0,bz),(bx+bw,y0,bz),(bx+bw,y1,bz),(bx-bw,y1,bz)],
         "#E8A020",CE_GOLD,0.95)
    quad(ax,[(bx-bw,y0,bz+BRIDGE_BEAM_H),(bx+bw,y0,bz+BRIDGE_BEAM_H),
             (bx+bw,y1,bz+BRIDGE_BEAM_H),(bx-bw,y1,bz+BRIDGE_BEAM_H)],
         "#E8A020",CE_GOLD,0.95)
    # Catwalk railings
    cw=bw*3.0; rz=bz+BRIDGE_BEAM_H+0.20+0.18
    for rx in [bx-cw,bx+cw]:
        ax.plot([rx,rx],[y0+0.3,y1-0.3],[rz,rz],color=CE_GOLD,lw=1.8,alpha=0.85)
    # CE label
    ax.text(bx,(y0+y1)/2,bz+BRIDGE_BEAM_H*0.5,"CR  CHAPPELL\n    ROBOTICS",
            fontsize=4,color=CE_BLACK,family="monospace",ha="center",va="center",fontweight="bold")
    ax.text(bx+0.5,(y0+y1)/2,bz+BRIDGE_BEAM_H+0.08,"2.0T",fontsize=5.5,
            color="white",family="monospace",ha="center",fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.15",facecolor=CE_BLACK,edgecolor=CE_GOLD,linewidth=1))
    # Trolley
    tz2=BRIDGE_BEAM_Z-0.45/2-0.05
    box(ax,bx,ty,tz2,0.27,0.35,0.22,CE_HOIST,"#444444",0.95)
    # Wire and hook
    for dy in [-0.04,+0.04]:
        ax.plot([bx,bx],[ty+dy,ty+dy],[BRIDGE_BEAM_Z-0.5,hz],
                color=CE_WIRE,lw=1.2,alpha=0.85)
    box(ax,bx,ty,hz,0.12,0.10,0.12,"#BBAA00",CE_GOLD,0.95)
    hook_pts_x=[bx,bx,bx+0.14,bx+0.14,bx+0.05]
    hook_pts_z=[hz-0.12,hz-0.28,hz-0.28,hz-0.18,hz-0.18]
    ax.plot(hook_pts_x,[ty]*5,hook_pts_z,color=CE_GOLD,lw=2.5,solid_capstyle="round")

def draw_cr6_rails(ax):
    for rail_y,label in [(RAIL_A_Y,"RAIL A"),(RAIL_B_Y,"RAIL B")]:
        rw=0.11; rb=0.15; rz0=0.06; rz1=rz0+0.14
        quad(ax,[(RAIL_X_MIN,rail_y-rb,rz0),(RAIL_X_MAX,rail_y-rb,rz0),
                 (RAIL_X_MAX,rail_y+rb,rz0),(RAIL_X_MIN,rail_y+rb,rz0)],
             "#1C1C1C","#333333",0.95)
        quad(ax,[(RAIL_X_MIN,rail_y-rw,rz1),(RAIL_X_MAX,rail_y-rw,rz1),
                 (RAIL_X_MAX,rail_y+rw,rz1),(RAIL_X_MIN,rail_y+rw,rz1)],
             "#1C1C1C","#333333",0.97)
        for gy in [rail_y-0.05,rail_y+0.05]:
            ax.plot([RAIL_X_MIN,RAIL_X_MAX],[gy,gy],[rz1+0.01,rz1+0.01],
                    color="#444444",lw=2,alpha=0.8,solid_capstyle="butt")
        ax.text((RAIL_X_MIN+RAIL_X_MAX)/2,rail_y,rz0+0.07,label,
                fontsize=5,color=CE_AMBER,family="monospace",ha="center",fontweight="bold")

def draw_atc_racks(ax):
    for rail_y,side in [(RAIL_A_Y,-1),(RAIL_B_Y,+1)]:
        for cx in [ATC_NEAR_X,ATC_FAR_X]:
            ry=rail_y+side*(0.15+0.37)
            box(ax,cx,ry,0.80/2,0.55,0.37,0.80/2,"#1A1A1A","#333333",0.93)
            for t in range(3):
                tz=0.25+t*0.22
                for tx in np.linspace(cx-0.40,cx+0.40,5):
                    n=6; theta=np.linspace(0,2*np.pi,n,endpoint=False)
                    bpts=[(tx+0.07*np.cos(a),ry+0.07*np.sin(a),tz+0.03) for a in theta]
                    tip=(tx,ry,tz+0.28)
                    for i in range(n):
                        quad(ax,[bpts[i],bpts[(i+1)%n],tip],"#5A5A5A","#333333",0.80,0.5)

def draw_robot(ax,robot):
    rx=robot.x; ry=robot.rail_y; side=robot.side
    base_z=0.06+0.14+0.14
    box(ax,rx,ry,base_z+0.14,0.18,0.18,0.14,"#CCCCCC","#888888",0.92)
    j1z=base_z+0.28
    box(ax,rx,ry,j1z+0.06,0.10,0.10,0.06,"#CC5500","#AA4400",0.95)
    if robot.working:
        el_x,el_y,el_z=rx,ry+side*0.80,j1z+0.55*0.7
        tip_x,tip_y,tip_z=rx,ry+side*1.45,0.9+0.15
    elif robot.changing_tool:
        el_x,el_y,el_z=rx,ry-side*0.30,j1z+0.55*0.7
        el_x,tip_x=rx,rx; tip_y=ry-side*0.7; tip_z=j1z+0.22*robot.tool_idx
    else:
        el_x,el_y,el_z=rx,ry+side*0.25,j1z+0.55
        tip_x,tip_y,tip_z=rx,ry+side*0.40,j1z+0.55+0.50*0.6
    ax.plot([rx,el_x],[ry,el_y],[j1z+0.14,el_z],color="#DDDDDD",lw=5,
            solid_capstyle="round",alpha=0.92)
    ax.scatter([el_x],[el_y],[el_z],color="#CC5500",s=30,zorder=10,depthshade=False)
    ax.plot([el_x,tip_x],[el_y,tip_y],[el_z,tip_z],color="#CCCCCC",lw=3.5,
            solid_capstyle="round",alpha=0.90)
    ax.scatter([tip_x],[tip_y],[tip_z],color="#CC5500",s=18,zorder=10,depthshade=False)
    tc=["#FF4400","#44AAFF","#44FF44","#FFAA00","#FF44AA"][robot.tool_idx%5]
    ax.plot([tip_x,tip_x],[tip_y+side*0.04,tip_y],[tip_z-0.18,tip_z],
            color=tc,lw=2.5,solid_capstyle="round",alpha=0.95)

def draw_table_jig_fixed(ax):
    cx,cy=FIXED_CX,FIXED_CY; hw,hd=FIXED_W/2,FIXED_D/2; z=FIXED_Z
    xs=np.array([[cx-hw,cx+hw],[cx-hw,cx+hw]])
    ys=np.array([[cy-hd,cy-hd],[cy+hd,cy+hd]])
    ax.plot_surface(xs,ys,np.full_like(xs,z),color="#2A4A2A",alpha=0.60)
    ax.plot([cx-hw,cx+hw,cx+hw,cx-hw,cx-hw],[cy-hd,cy-hd,cy+hd,cy+hd,cy-hd],[z]*5,
            color=CE_GOLD,lw=1.8,alpha=0.95)
    for lx in [cx-hw,cx+hw]:
        for ly in [cy-hd,cy+hd]:
            ax.plot([lx,lx],[ly,ly],[0,z],color="#333333",lw=2.5)
    for dx,dy in [(-1.4,-1.0),(-1.4,1.0),(1.4,-1.0),(1.4,1.0)]:
        ax.plot([cx+dx,cx+dx],[dy,dy],[z,z+0.22],color="#FFDD00",lw=2.5,
                solid_capstyle="round")
    ax.text(cx,cy,z+0.12,"TABLE_JIG_FIXED",fontsize=5,color="white",
            family="monospace",ha="center",fontweight="bold")

def draw_roller_table(ax,roller):
    cx,cy=ROLLER_CX,ROLLER_CY; hw,hd=ROLLER_W/2,ROLLER_D/2
    bz=ROLLER_Z-0.32; tz=ROLLER_Z
    # Base frame top and sides
    for sy,ey in [(-hd,-hd+0.12),(hd-0.12,hd)]:
        quad(ax,[(cx-hw,sy,tz),(cx+hw,sy,tz),(cx+hw,ey,tz),(cx-hw,ey,tz)],
             CE_BASE,CE_GOLD,0.95)
    for sy in [-hd,hd]:
        quad(ax,[(cx-hw,sy,bz),(cx+hw,sy,bz),(cx+hw,sy,tz),(cx-hw,sy,tz)],
             "#282828","#333333",0.90)
    ax.plot([cx-hw,cx+hw,cx+hw,cx-hw,cx-hw],[-hd,-hd,hd,hd,-hd],[tz]*5,
            color=CE_GOLD,lw=2.0,alpha=0.95)
    # Rollers
    roller_xs=np.linspace(cx-hw+0.20,cx+hw-0.20,ROLLER_COUNT)
    for rx in roller_xs:
        n=6; angles=np.linspace(0,2*np.pi,n,endpoint=False)
        for i in range(n):
            a0=angles[i]+np.radians(roller.roller_angle)
            a1=angles[(i+1)%n]+np.radians(roller.roller_angle)
            br=0.5+0.5*np.sin(a0+np.pi/2); val=int(100+br*100)
            col=f"#{val:02X}{val:02X}{val:02X}"
            face=[(rx,cy-hd+0.12,tz+ROLLER_R*np.sin(a0)),
                  (rx,cy+hd-0.12,tz+ROLLER_R*np.sin(a0)),
                  (rx,cy+hd-0.12,tz+ROLLER_R*np.sin(a1)),
                  (rx,cy-hd+0.12,tz+ROLLER_R*np.sin(a1))]
            quad(ax,face,col,"#888888",0.80,0.2)
    # Panel on rollers
    if roller.panel_visible:
        px=roller.panel_x; ptz=tz+ROLLER_R*2+0.01; hw_p=3.0; hd_p=1.65
        quad(ax,[(px-hw_p,cy-hd_p,ptz),(px+hw_p,cy-hd_p,ptz),
                 (px+hw_p,cy+hd_p,ptz),(px-hw_p,cy+hd_p,ptz)],
             CE_PANEL_T,CE_GOLD,0.55)
        ax.plot([px-hw_p,px+hw_p,px+hw_p,px-hw_p,px-hw_p],
                [cy-hd_p,cy-hd_p,cy+hd_p,cy+hd_p,cy-hd_p],[ptz]*5,
                color=CE_GOLD,lw=1.5,alpha=0.9)

def draw_tilt_table(ax,tilt,gantry_lifting=False):
    _gantry_lifting=gantry_lifting
    ang=tilt.angle_rad
    # Base frame
    bx_rear=PIVOT_X; bx_front=PIVOT_X-TILT_BASE_W; by=TILT_D/2; bz=TILT_BASE_H
    quad(ax,[(bx_front,-by,bz),(bx_rear,-by,bz),(bx_rear,+by,bz),(bx_front,+by,bz)],
         CE_BASE,CE_GOLD,0.95)
    for sy in [-by,by]:
        quad(ax,[(bx_front,sy,0),(bx_rear,sy,0),(bx_rear,sy,bz),(bx_front,sy,bz)],
             "#282828","#333333",0.90)
    ax.plot([bx_front,bx_rear,bx_rear,bx_front,bx_front],[-by,-by,by,by,-by],[bz]*5,
            color=CE_GOLD,lw=2.0,alpha=0.95)
    # Trunnion blocks
    for py in [TILT_CY-TILT_D*0.35,TILT_CY+TILT_D*0.35]:
        box(ax,bx_rear,py,bz+0.12,0.12,0.10,0.12,"#3A3A3A",CE_GOLD,0.95)
    # Tilt leaf — fade to ghost during LIFTING so panel clears without visual collision
    leaf_alpha = 0.12 if gantry_lifting else 0.92
    edge_alpha_col = "#888888" if gantry_lifting else CE_GOLD
    c0,c1,c2,c3=tilt.leaf_corners()
    quad(ax,[c0,c1,c2,c3],CE_LEAF,edge_alpha_col,leaf_alpha)
    def off(pt):
        return (pt[0]-TILT_THICK*np.sin(ang),pt[1],pt[2]-TILT_THICK*np.cos(ang))
    b0,b1,b2,b3=off(c0),off(c1),off(c2),off(c3)
    quad(ax,[b0,b1,b2,b3],"#282828","#333333",leaf_alpha*0.87)
    quad(ax,[c0,c3,b3,b0],"#303030","#333333",leaf_alpha*0.87)
    quad(ax,[c1,c2,b2,b1],"#303030","#333333",leaf_alpha*0.87)
    quad(ax,[c0,c1,b1,b0],"#2A2A2A","#333333",leaf_alpha*0.87)
    quad(ax,[c3,c2,b2,b3],"#252525","#333333",leaf_alpha*0.92)
    # Hydraulic cylinders
    cyl_base_x=PIVOT_X-1.80
    for y_off in [-0.55,+0.55]:
        foot=(cyl_base_x,TILT_CY+y_off,0.30); rod_tip=tilt.cyl_tip(y_off)
        dx=rod_tip[0]-foot[0]; dy=rod_tip[1]-foot[1]; dz=rod_tip[2]-foot[2]
        mid=(foot[0]+0.55*dx,foot[1]+0.55*dy,foot[2]+0.55*dz)
        ax.plot([foot[0],mid[0]],[foot[1],mid[1]],[foot[2],mid[2]],
                color=CE_HYD_B,lw=7,solid_capstyle="round",alpha=0.95)
        ax.plot([mid[0],rod_tip[0]],[mid[1],rod_tip[1]],[mid[2],rod_tip[2]],
                color=CE_HYD_R,lw=4,solid_capstyle="round",alpha=0.90)
    # Locating pins
    for u_off,v_off in [(-2.5,-1.1),(-2.5,+1.1),(2.5,-1.1),(2.5,+1.1)]:
        bp,tp=tilt.pin_pos(u_off,v_off)
        ax.plot([bp[0],tp[0]],[bp[1],tp[1]],[bp[2],tp[2]],
                color=CE_GOLD,lw=3.5,solid_capstyle="round",alpha=0.95)
    # Panel on leaf — hidden once gantry takes ownership (LIFTING or beyond)
    # FIX 3: pass gantry state in; hide panel the moment gantry enters LIFTING
    if tilt.state not in ("CRANE_PICKUP",) and tilt.angle_deg > 0.1 and not _gantry_lifting:
        hw_p=TILT_W/2-0.2; hd_p=TILT_D/2-0.15; LIFT=0.06
        def lp(u,dv):
            # FIX 1: outward normal — subtract sin from X, add cos to Z
            wx=PIVOT_X-u*np.cos(ang)-LIFT*np.sin(ang)
            wz=PIVOT_Z+u*np.sin(ang)+LIFT*np.cos(ang)
            return (wx,TILT_CY+dv,wz)
        pc0=lp(0.2,-hd_p); pc1=lp(0.2,+hd_p)
        pc2=lp(TILT_W-0.2,+hd_p); pc3=lp(TILT_W-0.2,-hd_p)
        quad(ax,[pc0,pc1,pc2,pc3],CE_PANEL_C,CE_GOLD,0.85)

def draw_panel_on_hook(ax,gantry):
    if not gantry.carrying: return
    PL=4.0; PW=1.4; THK=0.10
    ang=np.radians(gantry.panel_ang)
    hy=gantry.trolley_y

    # FIX 2: use stored pickup attachment point as rotation origin.
    # During HOOKED/LIFTING the panel pivots about pickup_x/pickup_z on the leaf.
    # Once vertical (90°) and traveling, track with the hook position.
    if gantry.state in ("HOOKED","LIFTING"):
        ox=gantry.pickup_x; oz=gantry.pickup_z
    else:
        # FIX 1: vertical carry — anchor panel TOP to hook, clamp bottom above floor
        # panel_ang=90 so sin(90-90)=0, cos(90-90)=1 → panel hangs straight down
        panel_top_z = gantry.hook_z - 0.15       # top of panel just below hook
        panel_bot_z = panel_top_z - PL            # bottom if fully hanging
        floor_clear = 0.12                        # keep bottom this far above floor
        if panel_bot_z < floor_clear:
            # panel bottom would go through floor — shift origin up
            panel_top_z = floor_clear + PL
        ox=gantry.bridge_x; oz=panel_top_z

    def pp(offset,dy,ts):
        # offset runs along the panel length (away from attachment point)
        wx=ox - offset*np.sin(ang - np.pi/2)
        wz=oz - offset*np.cos(ang - np.pi/2)
        # thickness offset perpendicular to panel face
        px= ts*THK*np.cos(ang - np.pi/2)
        pz= ts*THK*(-np.sin(ang - np.pi/2))
        return (wx+px, hy+dy, wz+pz)

    for ts in [-1,+1]:
        face=[pp(0,-PW,ts),pp(0,PW,ts),pp(PL,PW,ts),pp(PL,-PW,ts)]
        quad(ax,face,"#C8A068",CE_GOLD,0.85)

def draw_module_jig(ax,placed_walls):
    cx,cy=MOD_CX,MOD_CY; z=0.05
    # Jig frame
    for (x0,x1,yc) in [(cx-MOD_S,cx+MOD_S,cy-MOD_S),(cx-MOD_S,cx+MOD_S,cy+MOD_S)]:
        ax.plot([x0,x1],[yc,yc],[z,z],color=CE_GOLD,lw=2.5,alpha=0.8)
    for (xc,y0,y1) in [(cx-MOD_S,cy-MOD_S,cy+MOD_S),(cx+MOD_S,cy-MOD_S,cy+MOD_S)]:
        ax.plot([xc,xc],[y0,y1],[z,z],color=CE_GOLD,lw=2.5,alpha=0.8)
    ax.text(cx,cy,z+0.15,"MODULE_JIG",fontsize=5.5,color="#44FF88",
            family="monospace",ha="center",fontweight="bold")
    # Placed panels
    cols=["#C8A068","#C09050","#B87840","#B06030"]
    for i,name in enumerate(placed_walls):
        c=cols[i%4]
        if "PANEL_1" in name or i==0: xo,yo,hw_p,hd_p=-MOD_S,0,MOD_S,0.10
        elif i==1: xo,yo,hw_p,hd_p=MOD_S,0,MOD_S,0.10
        elif i==2: xo,yo,hw_p,hd_p=0,-MOD_S,0.10,MOD_S
        else: xo,yo,hw_p,hd_p=0,MOD_S,0.10,MOD_S
        quad(ax,[(cx+xo-hw_p,cy+yo-hd_p,0.1),(cx+xo+hw_p,cy+yo-hd_p,0.1),
                 (cx+xo+hw_p,cy+yo+hd_p,0.1),(cx+xo-hw_p,cy+yo+hd_p,0.1)],
             c,CE_GOLD,0.85)

def draw_material_stacks(ax):
    """LGS steel framing bundles flanking the FIXED table."""
    for sx,sy in [(FIXED_CX-FIXED_W/2-1.5, -2.0),(FIXED_CX-FIXED_W/2-1.5, 2.0)]:
        for zl in [0.1,0.35,0.60]:
            box(ax,sx,sy,zl+0.12,1.2,0.35,0.12,"#8899AA","#667788",0.85)
            for bx2 in [sx-0.6,sx,sx+0.6]:
                ax.plot([bx2,bx2],[sy-0.35,sy+0.35],[zl,zl+0.24],
                        color=CE_GOLD,lw=1.0,alpha=0.7)
    ax.text(FIXED_CX-FIXED_W/2-1.5,0,0.85,"LGS\nSTEEL",fontsize=5,
            color="#AABBCC",family="monospace",ha="center",alpha=0.8)


# ═══════════════════════════════════════════════════════════════════════
# SECTION 6 — MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════

# ── IO infrastructure — file-based control and state reporting ─────────
_STATE_DIR  = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "state")
_STATE_FILE = os.path.join(_STATE_DIR, "state.json")
_STATE_TMP  = os.path.join(_STATE_DIR, "state.tmp")
_CMD_FILE   = os.path.join(_STATE_DIR, "command_queue.json")

os.makedirs(_STATE_DIR, exist_ok=True)
if not os.path.exists(_CMD_FILE):
    try:
        with open(_CMD_FILE, "w", encoding="utf-8") as _f:
            json.dump({"command": None, "params": {}}, _f)
    except Exception:
        pass

_paused         = False   # True when "pause" command received
_force_one_step = False   # True after "step" command; clears after one tick fires
_WRITE_EVERY    = 10      # write state.json every N animation frames
_write_counter  = 0

cell = IntegratedCell()

fig = plt.figure(figsize=(22, 12))
fig.patch.set_facecolor("white")
ax  = fig.add_subplot(111, projection="3d")
ax.set_facecolor("white")
plt.subplots_adjust(bottom=0.10, left=0.01, right=0.99)

sax = plt.axes([0.15, 0.03, 0.55, 0.025])
sax.set_facecolor("#EEEEEE")
spd = Slider(sax, "Speed", 0.5, 10.0, valinit=1.0, color=CE_AMBER)
spd.label.set_color("#111111"); spd.valtext.set_color("#111111")

rax = plt.axes([0.75, 0.02, 0.08, 0.04])
btn = Button(rax, "RESET", color="#DDDDDD", hovercolor="#BBBBBB")
btn.label.set_color(CE_BLACK)

def reset_sim(event):
    global cell
    cell = IntegratedCell()
btn.on_clicked(reset_sim)


def update(frame_num):
    global _paused, _force_one_step, _write_counter, cell

    ax.clear(); ax.set_facecolor("white")

    # ── HOOK A: read command_queue.json ──────────────────────────────
    try:
        with open(_CMD_FILE, "r", encoding="utf-8") as _f:
            _cmd_data = json.load(_f)
        _cmd = _cmd_data.get("command")
        if _cmd is not None:
            _params = _cmd_data.get("params") or {}
            if _cmd == "pause":
                _paused = True
            elif _cmd == "resume":
                _paused = False
            elif _cmd == "reset":
                cell = IntegratedCell()
                _paused = False
                _write_counter = 0
            elif _cmd == "step":
                _force_one_step = True
            elif _cmd == "set_speed":
                _v = float(_params.get("speed", spd.val))
                spd.set_val(max(0.5, min(10.0, _v)))
            with open(_CMD_FILE, "w", encoding="utf-8") as _f:
                json.dump({"command": None, "params": {}}, _f)
    except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError, ValueError):
        pass

    # ── Advance simulation ────────────────────────────────────────────
    if _force_one_step:
        cell.step()
        _force_one_step = False
    elif not _paused:
        steps = max(1, int(spd.val))
        for _ in range(steps):
            cell.step()

    # ── HOOK B: write state.json (throttled: every _WRITE_EVERY frames) ─
    _write_counter += 1
    if _write_counter >= _WRITE_EVERY:
        _write_counter = 0
        try:
            _snap = {
                "frame":        cell.tick,
                "master_phase": cell._phase,
                "module_done":  cell.module_done,
                "placed_walls": list(cell.placed_walls),
                "gantry": {
                    "state":     cell.gantry.state,
                    "cycles":    cell.gantry.cycles,
                    "bridge_x":  round(cell.gantry.bridge_x, 3),
                    "trolley_y": round(cell.gantry.trolley_y, 3),
                    "hook_z":    round(cell.gantry.hook_z, 3),
                    "carrying":  cell.gantry.carrying,
                    "panel_ang": round(cell.gantry.panel_ang, 1),
                },
                "roller": {
                    "state":      cell.roller.state,
                    "cycles":     cell.roller.cycles,
                    "panel_x":    round(cell.roller.panel_x, 3),
                    "travel_pct": round(cell.roller.travel_pct, 1),
                    "speed":      round(cell.roller.speed, 4),
                },
                "tilt": {
                    "state":        cell.tilt.state,
                    "cycles":       cell.tilt.cycles,
                    "angle_deg":    round(cell.tilt.angle_deg, 2),
                    "pin_extended": cell.tilt.pin_extended,
                },
                "robots": {
                    _rb.name: {
                        "state":    _rb.state,
                        "x":        round(_rb.x, 3),
                        "cycles":   _rb.cycles,
                        "tool_idx": _rb.tool_idx,
                    }
                    for _rb in cell.robots
                },
            }
            with open(_STATE_TMP, "w", encoding="utf-8") as _f:
                json.dump(_snap, _f, indent=2)
            os.replace(_STATE_TMP, _STATE_FILE)
        except Exception:
            pass

    g=cell.gantry; r=cell.roller; t=cell.tilt

    # ── Draw scene in order ──────────────────────────────────────────
    draw_floor(ax)
    draw_safety_fence(ax)
    draw_runway_rails(ax)
    draw_material_stacks(ax)
    draw_table_jig_fixed(ax)
    draw_roller_table(ax, r)
    draw_tilt_table(ax, t, gantry_lifting=(g.state=="LIFTING"))
    draw_module_jig(ax, cell.placed_walls)
    draw_cr6_rails(ax)
    draw_atc_racks(ax)
    for robot in cell.robots:
        draw_robot(ax, robot)
    draw_gantry(ax, g)
    draw_panel_on_hook(ax, g)

    # ── State colours ─────────────────────────────────────────────────
    phase_colors = {
        "RAIL_WORKING":    "#44FF44",
        "ROLLER_TRANSFER": CE_AMBER,
        "TILT_RAISING":    "#FF8800",
        "GANTRY_PICKUP":   "#FF4400",
        "RETURNING":       "#AADDFF",
        "COMPLETE":        "#44FFAA",
    }
    pc = phase_colors.get(cell._phase, "white")

    # ── HUD ───────────────────────────────────────────────────────────
    hud = (
        f"CE INTEGRATED CELL  V3.0\n"
        f"{'─'*38}\n"
        f"PHASE     {cell._phase}\n"
        f"{'─'*38}\n"
        f"GANTRY    {g.state:<22} CYC:{g.cycles}\n"
        f"  X={g.bridge_x:+5.1f}  Y={g.trolley_y:+4.1f}  Z={g.hook_z:+4.1f}\n"
        f"ROLLER    {r.state:<22} CYC:{r.cycles}\n"
        f"  POS={r.panel_x:+5.2f}  TRAVEL:{r.travel_pct:.0f}%\n"
        f"TILT      {t.state:<22} CYC:{t.cycles}\n"
        f"  ANG={t.angle_deg:5.1f}°/{MAX_TILT:.0f}°\n"
        f"{'─'*38}\n"
        f"ROBOTS    "
        f"A1:{cell.A1.state[:4]}  A2:{cell.A2.state[:4]}\n"
        f"          "
        f"B1:{cell.B1.state[:4]}  B2:{cell.B2.state[:4]}\n"
        f"{'─'*38}\n"
        f"PANELS    {len(cell.placed_walls)}/4\n"
        f"MODULE    {'✓ COMPLETE' if cell.module_done else 'IN PROGRESS'}\n"
        f"{'─'*38}\n"
        f"FRAME     {cell.tick:05d}\n"
    )

    ax.text2D(0.01, 0.99, hud, transform=ax.transAxes,
              fontsize=5.5, family="monospace", va="top", color="#111111",
              bbox=dict(boxstyle="round,pad=0.3", facecolor="#FFFDF0",
                        alpha=0.92, edgecolor=CE_GOLD))

    # Phase badge
    ax.text2D(0.99, 0.99, f"◉ {cell._phase}",
              transform=ax.transAxes, fontsize=7, family="monospace",
              va="top", ha="right", color=pc, fontweight="bold",
              bbox=dict(boxstyle="round,pad=0.3", facecolor=CE_BLACK,
                        alpha=0.85, edgecolor=pc))

    ax.set_title(
        "CONSTRUCTION ENTERPRISES  —  CHAPPELL ROBOTICS\n"
        "CE INTEGRATED CELL V3.0  │  CR6 RAIL  │  ROLLER TRANSFER  "
        "│  TILT TABLE  │  2.0T GANTRY",
        fontsize=9, fontweight="bold", color=CE_BLACK)

    ax.set_xlim(-2, 38)
    ax.set_ylim(-20, 20)
    ax.set_zlim(-12, 28)
    ax.set_xlabel("X  (CELL AXIS →)", color="#333333", fontsize=7)
    ax.set_ylabel("Y  (CROSS AXIS)",  color="#333333", fontsize=7)
    ax.set_zlabel("Z  (HEIGHT)",      color="#333333", fontsize=7)
    ax.tick_params(colors="#444444", labelsize=6)

    for pane in [ax.xaxis.pane,ax.yaxis.pane,ax.zaxis.pane]:
        pane.fill=True; pane.set_facecolor("#F5F5F5"); pane.set_edgecolor("#CCCCCC")
    ax.grid(True, alpha=0.3, color="#AAAAAA")
    ax.set_box_aspect([1, 1, 1])

    ax.set_xlim(-2, 38)
    ax.set_ylim(-20, 20)
    ax.set_zlim(-12, 28)


ani = FuncAnimation(fig, update, interval=1, cache_frame_data=False)
plt.show()
