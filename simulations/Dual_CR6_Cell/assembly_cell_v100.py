"""
CHAPPELL ROBOTICS
Assembly Cell v29  —  Pygame Renderer  |  Architecture: Sim / Renderer Split
=============================================================================

MIGRATION FROM v28
------------------
Renderer  : Matplotlib → Pygame (pure 2D draw calls, no OpenGL)
Camera    : Custom perspective projection (3D→2D), same angle as v28
            Can be swapped for OpenGL / Panda3D / Ursina with no sim changes

ARCHITECTURE
------------
  SimulationEngine   — all logic, state machines, kinematics, part tracking
  Camera             — 3D→2D projection (perspective, rotate, zoom)
  Renderer           — takes sim snapshot, calls pygame.draw.*
  HUD                — metrics overlay, pure pygame text
  Main loop          — event → sim.step() → renderer.draw() → flip

SIMULATION PRESERVED FROM v28 (unchanged)
------------------------------------------
  • Robot 1 & 2 state machines (IDLE/PICKING_A/B/PRE_PLACE/PLACING/PICKING/
    PLACING/WELDING/FRAME_COMPLETE)
  • Zone-based accumulation conveyor with station_busy gate
  • load_zone_clear() centre-point check (v28 fix)
  • advance_waypoints() re-entry guard (v28 fix)
  • Stall detector fires only when both robots IDLE (v28 fix)
  • IK_Z_FLOOR naming, held-stud z = tcp - h/2 (v28 fixes)
  • Spark system, outfeed conveyor, weld points
  • Part conservation tracking

CONTROLS
--------
  Arrow keys / WASD   : orbit camera
  +  /  -             : zoom
  R                   : reset camera
  SPACE               : pause / resume
  ESC / Q             : quit
"""

import sys, math, time
import numpy as np

# ── dependency check ──────────────────────────────────────────────────────────
try:
    import pygame
except ImportError:
    print("MISSING: pygame.  In Pydroid: pip install pygame")
    sys.exit(1)


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 1 — SIMULATION CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════

# Stud geometry
STUD_H  = 0.35
RAIL_W  = 3.0      # type-A: long in X
RAIL_D  = 0.35
POST_W  = 0.35     # type-B: long in Y
POST_D  = 3.0

STUD_ON_BELT_Z = STUD_H          # = 0.35  bottom face on belt (no shade artefact)

# Conveyor
BELT_TOP_Z     = 0.0
BELT_HALF_W    = 1.0
BELT_THICK     = 0.15
CONVEYOR_Y     = 0.0
CONVEYOR_SPEED = 0.09

# Zones
ZONE_LENGTH = 3.0
ZONE_LOAD   =  3.5
ZONE_1      =  6.5
ZONE_2      =  9.5
ZONE_PICKUP = 13.0
ZONES       = [ZONE_LOAD, ZONE_1, ZONE_2, ZONE_PICKUP]
LOAD_X      = ZONE_LOAD
PICKUP_X    = ZONE_PICKUP

# Kinematics
L1          = 4.0
L2          = 4.0
SAFE_Z      = 3.0
ARRIVE_DIST = 0.28
IK_Z_FLOOR  = STUD_H / 2   # 0.175 — IK clamp only, not a render value

# Fixture table
TABLE_X     = 18.0
TABLE_Y     =  4.5
TABLE_W     =  5.5
TABLE_D     =  4.0
TABLE_THICK =  0.12
FRAME_HALF  =  1.3
PANEL_SIZE  =  4

FRAME_SLOTS = [
    dict(type="A", x=TABLE_X,              y=TABLE_Y+FRAME_HALF, z=STUD_ON_BELT_Z),
    dict(type="A", x=TABLE_X,              y=TABLE_Y-FRAME_HALF, z=STUD_ON_BELT_Z),
    dict(type="B", x=TABLE_X-FRAME_HALF,   y=TABLE_Y,            z=STUD_ON_BELT_Z),
    dict(type="B", x=TABLE_X+FRAME_HALF,   y=TABLE_Y,            z=STUD_ON_BELT_Z),
]

R1_HOME = [0.5, 0.0, SAFE_Z]
R2_HOME = [TABLE_X, TABLE_Y, SAFE_Z + 1.0]


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 2 — SIMULATION ENGINE
# ═══════════════════════════════════════════════════════════════════════════════

class SimulationEngine:
    """
    Pure simulation. No rendering. No pygame.
    All state is on self.  Can be stepped independently of display.
    To migrate to a different renderer: keep this class, replace Renderer.
    """

    def __init__(self):
        self._stud_counter = 0
        self._reset_state()

    # ── part factory ──────────────────────────────────────────────────────────

    def _make_stud(self, ptype, x, y, z):
        self._stud_counter += 1
        sid = f"S{self._stud_counter:02d}_{ptype}"
        if ptype == "A":
            return dict(id=sid, type="A", x=x, y=y, z=z,
                        w=RAIL_W, d=RAIL_D, h=STUD_H, state="QUEUE")
        else:
            return dict(id=sid, type="B", x=x, y=y, z=z,
                        w=POST_W, d=POST_D, h=STUD_H, state="QUEUE")

    # ── state init ────────────────────────────────────────────────────────────

    def _reset_state(self):
        self.part_a_queue = [
            self._make_stud("A", 1.5, -3.2, 0.8),
            self._make_stud("A", 2.5, -3.2, 1.4),
            self._make_stud("A", 3.5, -3.2, 2.0),
            self._make_stud("A", 4.5, -3.2, 2.6),
        ]
        self.part_b_queue = [
            self._make_stud("B", 1.5,  3.2, 0.8),
            self._make_stud("B", 2.5,  3.2, 1.4),
            self._make_stud("B", 3.5,  3.2, 2.0),
            self._make_stud("B", 4.5,  3.2, 2.6),
        ]
        self.TOTAL_PARTS = len(self.part_a_queue) + len(self.part_b_queue)

        self.conveyor_parts = []
        self.frame_parts    = []
        self.filled_slots   = set()
        self.outfeed_panels = []
        self.spark_frames   = []
        self.weld_points    = []
        self.weld_targets   = []

        self.produced_count = 0
        self.station_busy   = False
        self.paused         = False
        self._stall_counter = 0
        self.frame_num      = 0
        self.cycle_times    = []
        self._cycle_start   = time.time()

        self.robot1 = {
            "base": np.array([0.0, 0.0, 0.0]),
            "theta1": math.pi/4, "theta2": math.pi/3, "theta3": -math.pi/2,
            "target": np.array([1.5, -3.2, 0.8]),
            "waypoints": [], "wp_index": 0,
            "state": "IDLE", "held": None,
            "label": "R1",
        }
        self.robot2 = {
            "base": np.array([16.0, 3.5, 0.0]),
            "theta1": -math.pi*0.5, "theta2": math.pi/3, "theta3": -math.pi/2,
            "target": np.array([PICKUP_X, CONVEYOR_Y, SAFE_Z]),
            "waypoints": [], "wp_index": 0,
            "state": "IDLE", "held": None,
            "weld_index": 0,
            "_slot_idx": None, "_stud": None, "_slot": None,
            "label": "R2",
        }

    # ── kinematics ────────────────────────────────────────────────────────────

    def fk3(self, robot):
        b  = robot["base"]
        t1, t2, t3 = robot["theta1"], robot["theta2"], robot["theta3"]
        ad = np.array([math.cos(t1), math.sin(t1), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        j1  = b.copy()
        j2  = j1  + L1 * (math.cos(t2) * ad + math.sin(t2) * up)
        tcp = j2  + L2 * (math.cos(t2+t3) * ad + math.sin(t2+t3) * up)
        return j1, j2, tcp

    def _ik3(self, robot, tx, ty, tz):
        tz    = max(tz, IK_Z_FLOOR)
        b     = robot["base"]
        dx, dy, dz = tx-b[0], ty-b[1], tz-b[2]
        t1    = math.atan2(dy, dx)
        r     = math.hypot(dx, dy)
        reach = min(math.hypot(r, dz), L1+L2-0.001)
        c3    = max(-1.0, min(1.0, (reach**2-L1**2-L2**2)/(2*L1*L2)))
        t3    = math.acos(c3)
        alpha = math.atan2(dz, r)
        beta  = math.atan2(L2*math.sin(t3), L1+L2*math.cos(t3))
        return t1, alpha+beta, -t3

    def _move_toward(self, robot, speed=0.14):
        tx, ty, tz = robot["target"]
        g1, g2, g3 = self._ik3(robot, tx, ty, tz)
        robot["theta1"] += (g1-robot["theta1"])*speed
        robot["theta2"] += (g2-robot["theta2"])*speed
        robot["theta3"] += (g3-robot["theta3"])*speed

    # ── waypoint system ───────────────────────────────────────────────────────

    def _set_wp(self, robot, wps):
        robot["waypoints"] = list(wps)
        robot["wp_index"]  = 0
        robot["target"]    = np.array(wps[0], dtype=float)

    def _advance_wp(self, robot):
        # v28 FIX 1: guard against re-entry after completion
        if robot["wp_index"] >= len(robot["waypoints"]):
            return True
        _, _, tcp = self.fk3(robot)
        tgt = np.array(robot["waypoints"][robot["wp_index"]], dtype=float)
        if np.linalg.norm(tcp - tgt) < ARRIVE_DIST:
            robot["wp_index"] += 1
            if robot["wp_index"] >= len(robot["waypoints"]):
                return True
            robot["target"] = np.array(robot["waypoints"][robot["wp_index"]], dtype=float)
        return False

    # ── waypoint builders ─────────────────────────────────────────────────────

    def _r1_pick_wps(self, sx, sy, sz):
        return [R1_HOME, [sx, sy, SAFE_Z], [sx, sy, sz], [sx, sy, SAFE_Z]]
    def _r1_hover_wps(self):
        return [[LOAD_X, CONVEYOR_Y, SAFE_Z]]
    def _r1_descend_wps(self):
        dz = STUD_ON_BELT_Z + STUD_H/2
        return [[LOAD_X, CONVEYOR_Y, dz], [LOAD_X, CONVEYOR_Y, SAFE_Z], R1_HOME]
    def _r2_pick_wps(self, s):
        pz = STUD_ON_BELT_Z + STUD_H/2
        return [R2_HOME, [s["x"],s["y"],SAFE_Z], [s["x"],s["y"],pz], [s["x"],s["y"],SAFE_Z]]
    def _r2_place_wps(self, slot):
        px, py, pz = slot["x"], slot["y"], slot["z"]
        plz = pz + STUD_H/2
        return [[px,py,SAFE_Z], [px,py,plz+0.5], [px,py,plz], [px,py,SAFE_Z], R2_HOME]
    def _r2_weld_wps(self, t):
        tx, ty, tz = t
        return [[tx,ty,tz+1.0], [tx,ty,tz], [tx,ty,tz+1.0]]

    # ── conveyor helpers ──────────────────────────────────────────────────────

    def _part_bbox(self, p):
        hw = p["w"]/2
        return p["x"]-hw, p["x"]+hw

    def _zone_range(self, zx):
        return zx-ZONE_LENGTH/2, zx+ZONE_LENGTH/2

    def _zone_occupied_by(self, zx, parts, exclude=None):
        zm, zM = self._zone_range(zx)
        for p in parts:
            if p is exclude: continue
            pm, pM = self._part_bbox(p)
            if pm < zM and pM > zm: return True
        return False

    def _zone_of(self, p):
        for i in range(len(ZONES)-1, -1, -1):
            if p["x"] >= ZONES[i]-ZONE_LENGTH/2: return i
        return 0

    def _advance_conveyor(self):
        for p in self.conveyor_parts:
            z  = self._zone_of(p); nz = z+1
            if z >= len(ZONES)-1: continue
            if self.station_busy and z == len(ZONES)-2:
                t = ZONES[z]
                if p["x"] < t: p["x"] = min(p["x"]+CONVEYOR_SPEED, t)
                continue
            nzx = ZONES[nz]
            if not self._zone_occupied_by(nzx, self.conveyor_parts, exclude=p):
                p["x"] = min(p["x"]+CONVEYOR_SPEED, nzx)
            else:
                t = ZONES[z]
                if p["x"] < t: p["x"] = min(p["x"]+CONVEYOR_SPEED, t)

    def _load_zone_clear(self):
        for p in self.conveyor_parts:
            if abs(p["x"]-LOAD_X) < 1.5: return False
        return True

    def _find_slot_for(self, ptype):
        for i, s in enumerate(FRAME_SLOTS):
            if s["type"] == ptype and i not in self.filled_slots:
                return i, s
        return None, None

    def _make_weld_targets(self):
        return [
            np.array([TABLE_X-FRAME_HALF, TABLE_Y+FRAME_HALF, STUD_ON_BELT_Z+STUD_H]),
            np.array([TABLE_X+FRAME_HALF, TABLE_Y+FRAME_HALF, STUD_ON_BELT_Z+STUD_H]),
            np.array([TABLE_X-FRAME_HALF, TABLE_Y-FRAME_HALF, STUD_ON_BELT_Z+STUD_H]),
            np.array([TABLE_X+FRAME_HALF, TABLE_Y-FRAME_HALF, STUD_ON_BELT_Z+STUD_H]),
        ]

    def _restock_queues(self):
        """Refill part queues for continuous production. Preserves produced_count."""
        self.part_a_queue = [
            self._make_stud("A", 1.5, -3.2, 0.8),
            self._make_stud("A", 2.5, -3.2, 1.4),
            self._make_stud("A", 3.5, -3.2, 2.0),
            self._make_stud("A", 4.5, -3.2, 2.6),
        ]
        self.part_b_queue = [
            self._make_stud("B", 1.5,  3.2, 0.8),
            self._make_stud("B", 2.5,  3.2, 1.4),
            self._make_stud("B", 3.5,  3.2, 2.0),
            self._make_stud("B", 4.5,  3.2, 2.6),
        ]
        self._cycle_start = time.time()

    # ── main step ─────────────────────────────────────────────────────────────

    def step(self):
        if self.paused:
            return

        self.frame_num += 1
        r1, r2 = self.robot1, self.robot2
        _, _, tcp1 = self.fk3(r1)
        _, _, tcp2 = self.fk3(r2)

        # Part conservation
        total_parts = (
            len(self.part_a_queue) + len(self.part_b_queue)
            + len(self.conveyor_parts) + len(self.frame_parts)
            + (1 if r1["held"] else 0)
            + (1 if r2["held"] else 0)
        )

        # Stall detector — v28 FIX 2: only when both IDLE
        both_idle = r1["state"] == "IDLE" and r2["state"] == "IDLE"
        if both_idle and total_parts == self.TOTAL_PARTS and len(self.frame_parts) < PANEL_SIZE:
            self._stall_counter += 1
            if self._stall_counter == 300:
                print(f"[F{self.frame_num}] *** STALL: R1={r1['state']} "
                      f"R2={r2['state']} belt={len(self.conveyor_parts)} "
                      f"frame={len(self.frame_parts)}/{PANEL_SIZE}")
        else:
            self._stall_counter = 0

        # ── Robot 1 ───────────────────────────────────────────────────────────

        if r1["state"] == "IDLE":
            if self.part_a_queue:
                p = self.part_a_queue[0]
                self._set_wp(r1, self._r1_pick_wps(p["x"], p["y"], p["z"]))
                r1["state"] = "PICKING_A"

        elif r1["state"] == "PICKING_A":
            done = self._advance_wp(r1)
            if r1["wp_index"] in (2,3) and r1["held"] is None and self.part_a_queue:
                p = self.part_a_queue[0]
                if np.linalg.norm(tcp1 - np.array([p["x"],p["y"],p["z"]])) < ARRIVE_DIST+0.2:
                    r1["held"] = dict(**p)
                    self.part_a_queue.pop(0)
                    r1["held"]["state"] = "ROBOT1"
            if r1["held"]:
                r1["held"]["x"] = tcp1[0]; r1["held"]["y"] = tcp1[1]
                r1["held"]["z"] = max(tcp1[2]-r1["held"]["h"]/2, IK_Z_FLOOR)
            if done:
                if r1["held"] is None and self.part_a_queue:
                    p = self.part_a_queue[0]
                    self._set_wp(r1, self._r1_pick_wps(p["x"],p["y"],p["z"]))
                else:
                    self._set_wp(r1, self._r1_hover_wps())
                    r1["state"] = "PRE_PLACE_A"

        elif r1["state"] == "PRE_PLACE_A":
            if r1["held"]:
                r1["held"]["x"] = tcp1[0]; r1["held"]["y"] = tcp1[1]
                r1["held"]["z"] = max(tcp1[2]-r1["held"]["h"]/2, IK_Z_FLOOR)
            self._advance_wp(r1)
            if self._load_zone_clear():
                self._set_wp(r1, self._r1_descend_wps())
                r1["state"] = "PLACING_A"

        elif r1["state"] == "PLACING_A":
            drop_z = STUD_ON_BELT_Z
            if r1["held"]:
                r1["held"]["x"] = tcp1[0]; r1["held"]["y"] = tcp1[1]
                r1["held"]["z"] = max(tcp1[2]-r1["held"]["h"]/2, IK_Z_FLOOR)
            if r1["wp_index"] <= 1 and r1["held"] is not None:
                dtgt = np.array([LOAD_X, CONVEYOR_Y, drop_z+STUD_H/2])
                if np.linalg.norm(tcp1-dtgt) < ARRIVE_DIST+0.2:
                    part = dict(**r1["held"])
                    part.update(x=LOAD_X, y=CONVEYOR_Y, z=drop_z, state="CONVEYOR")
                    self.conveyor_parts.append(part)
                    r1["held"] = None
            done = self._advance_wp(r1)
            if done and r1["held"] is not None:
                part = dict(**r1["held"])
                part.update(x=LOAD_X, y=CONVEYOR_Y, z=drop_z, state="CONVEYOR")
                self.conveyor_parts.append(part)
                r1["held"] = None
            if done:
                if self.part_b_queue:
                    p = self.part_b_queue[0]
                    self._set_wp(r1, self._r1_pick_wps(p["x"],p["y"],p["z"]))
                    r1["state"] = "PICKING_B"
                else:
                    r1["state"] = "IDLE"

        elif r1["state"] == "PICKING_B":
            done = self._advance_wp(r1)
            if r1["wp_index"] in (2,3) and r1["held"] is None and self.part_b_queue:
                p = self.part_b_queue[0]
                if np.linalg.norm(tcp1 - np.array([p["x"],p["y"],p["z"]])) < ARRIVE_DIST+0.2:
                    r1["held"] = dict(**p)
                    self.part_b_queue.pop(0)
                    r1["held"]["state"] = "ROBOT1"
            if r1["held"]:
                r1["held"]["x"] = tcp1[0]; r1["held"]["y"] = tcp1[1]
                r1["held"]["z"] = max(tcp1[2]-r1["held"]["h"]/2, IK_Z_FLOOR)
            if done:
                if r1["held"] is None and self.part_b_queue:
                    p = self.part_b_queue[0]
                    self._set_wp(r1, self._r1_pick_wps(p["x"],p["y"],p["z"]))
                else:
                    self._set_wp(r1, self._r1_hover_wps())
                    r1["state"] = "PRE_PLACE_B"

        elif r1["state"] == "PRE_PLACE_B":
            if r1["held"]:
                r1["held"]["x"] = tcp1[0]; r1["held"]["y"] = tcp1[1]
                r1["held"]["z"] = max(tcp1[2]-r1["held"]["h"]/2, IK_Z_FLOOR)
            self._advance_wp(r1)
            if self._load_zone_clear():
                self._set_wp(r1, self._r1_descend_wps())
                r1["state"] = "PLACING_B"

        elif r1["state"] == "PLACING_B":
            drop_z = STUD_ON_BELT_Z
            if r1["held"]:
                r1["held"]["x"] = tcp1[0]; r1["held"]["y"] = tcp1[1]
                r1["held"]["z"] = max(tcp1[2]-r1["held"]["h"]/2, IK_Z_FLOOR)
            if r1["wp_index"] <= 1 and r1["held"] is not None:
                dtgt = np.array([LOAD_X, CONVEYOR_Y, drop_z+STUD_H/2])
                if np.linalg.norm(tcp1-dtgt) < ARRIVE_DIST+0.2:
                    part = dict(**r1["held"])
                    part.update(x=LOAD_X, y=CONVEYOR_Y, z=drop_z, state="CONVEYOR")
                    self.conveyor_parts.append(part)
                    r1["held"] = None
            done = self._advance_wp(r1)
            if done and r1["held"] is not None:
                part = dict(**r1["held"])
                part.update(x=LOAD_X, y=CONVEYOR_Y, z=drop_z, state="CONVEYOR")
                self.conveyor_parts.append(part)
                r1["held"] = None
            if done:
                r1["state"] = "IDLE"

        # ── conveyor ──────────────────────────────────────────────────────────
        self._advance_conveyor()

        # ── Robot 2 ───────────────────────────────────────────────────────────

        if r2["state"] == "IDLE":
            if (len(self.filled_slots) == PANEL_SIZE and
                    len(self.frame_parts) == PANEL_SIZE and
                    r2["held"] is None):
                self.weld_targets   = self._make_weld_targets()
                self.weld_points    = []
                r2["weld_index"]    = 0
                self._set_wp(r2, self._r2_weld_wps(self.weld_targets[0]))
                r2["state"] = "WELDING"
            else:
                if r2["held"] is None:
                    self.station_busy = False
                ready = [p for p in self.conveyor_parts
                         if abs(p["x"]-PICKUP_X) < ZONE_LENGTH*0.5]
                if ready:
                    for c in ready:
                        si, sl = self._find_slot_for(c["type"])
                        if sl is not None:
                            r2["_stud"] = c; r2["_slot"] = sl; r2["_slot_idx"] = si
                            self._set_wp(r2, self._r2_pick_wps(c))
                            r2["state"] = "PICKING"
                            self.station_busy = True
                            break

        elif r2["state"] == "PICKING":
            if r2["_stud"] not in self.conveyor_parts and r2["held"] is None:
                r2["state"] = "IDLE"; self.station_busy = False
            else:
                done = self._advance_wp(r2)
                pz   = STUD_ON_BELT_Z + STUD_H/2
                if r2["wp_index"] in (2,3) and r2["held"] is None:
                    s = r2["_stud"]
                    if s in self.conveyor_parts:
                        tgt = np.array([s["x"], s["y"], pz])
                        if np.linalg.norm(tcp2-tgt) < ARRIVE_DIST+0.2:
                            r2["held"] = dict(**s)
                            self.conveyor_parts.remove(s)
                            r2["held"]["state"] = "ROBOT2"
                            self.station_busy = False
                if done and r2["held"] is None:
                    r2["state"] = "IDLE"; self.station_busy = False
                elif done:
                    self._set_wp(r2, self._r2_place_wps(r2["_slot"]))
                    r2["state"] = "PLACING"
                if r2["held"]:
                    r2["held"]["x"] = tcp2[0]
                    r2["held"]["y"] = tcp2[1]
                    r2["held"]["z"] = tcp2[2]

        elif r2["state"] == "PLACING":
            if r2["held"]:
                r2["held"]["x"] = tcp2[0]
                r2["held"]["y"] = tcp2[1]
                r2["held"]["z"] = tcp2[2]
            if r2["wp_index"] <= 3 and r2["held"] is not None:
                sl    = r2["_slot"]
                plz   = sl["z"] + STUD_H/2
                tgt   = np.array([sl["x"], sl["y"], plz])
                if np.linalg.norm(tcp2-tgt) < ARRIVE_DIST+0.2:
                    placed = dict(**r2["held"])
                    placed.update(x=sl["x"], y=sl["y"], z=sl["z"], state="FRAME")
                    self.frame_parts.append(placed)
                    self.filled_slots.add(r2["_slot_idx"])
                    r2["held"] = None
            done = self._advance_wp(r2)
            if done and r2["held"] is not None:
                sl = r2["_slot"]
                placed = dict(**r2["held"])
                placed.update(x=sl["x"], y=sl["y"], z=sl["z"], state="FRAME")
                self.frame_parts.append(placed)
                self.filled_slots.add(r2["_slot_idx"])
                r2["held"] = None
            if done:
                if len(self.filled_slots) >= PANEL_SIZE:
                    self.weld_targets   = self._make_weld_targets()
                    self.weld_points    = []
                    r2["weld_index"]    = 0
                    self._set_wp(r2, self._r2_weld_wps(self.weld_targets[0]))
                    r2["state"] = "WELDING"
                else:
                    r2["state"] = "IDLE"

        elif r2["state"] == "WELDING":
            done = self._advance_wp(r2)
            if r2["wp_index"] == 1:
                tgt = self.weld_targets[r2["weld_index"]]
                if np.linalg.norm(tcp2-tgt) < ARRIVE_DIST+0.05:
                    if len(self.weld_points) == r2["weld_index"]:
                        self.weld_points.append(tgt.copy())
                        for _ in range(28):
                            self.spark_frames.append(dict(
                                x=tgt[0]+np.random.uniform(-0.30,0.30),
                                y=tgt[1]+np.random.uniform(-0.20,0.20),
                                z=tgt[2]+np.random.uniform(0.0, 0.55),
                                life=20))
            if done:
                r2["weld_index"] += 1
                if r2["weld_index"] < len(self.weld_targets):
                    self._set_wp(r2, self._r2_weld_wps(self.weld_targets[r2["weld_index"]]))
                else:
                    r2["state"] = "FRAME_COMPLETE"

        elif r2["state"] == "FRAME_COMPLETE":
            fw = FRAME_HALF*2 + max(RAIL_D, POST_W)
            self.outfeed_panels.append(
                dict(x=TABLE_X, y=TABLE_Y, z=STUD_ON_BELT_Z, w=fw, d=fw, h=STUD_H))
            self.produced_count += 1
            t_now = time.time()
            self.cycle_times.append(t_now - self._cycle_start)
            self._cycle_start = t_now
            print(f"  *** PANEL #{self.produced_count} COMPLETE "
                  f"(cycle {self.cycle_times[-1]:.1f}s) ***")
            self.frame_parts.clear()
            self.filled_slots.clear()
            self.weld_points = []
            r2["state"] = "IDLE"

        # ── sparks ────────────────────────────────────────────────────────────
        live = []
        for s in self.spark_frames:
            s["life"] -= 1
            s["x"] += np.random.uniform(-0.06, 0.06)
            s["y"] += np.random.uniform(-0.06, 0.06)
            s["z"] += np.random.uniform(0.02,  0.10)
            if s["life"] > 0: live.append(s)
        self.spark_frames[:] = live

        # ── outfeed ───────────────────────────────────────────────────────────
        for f in self.outfeed_panels:
            f["y"] += CONVEYOR_SPEED * 1.5
        self.outfeed_panels[:] = [f for f in self.outfeed_panels if f["y"] < 15]

        # ── continuous production loop ────────────────────────────────────────
        # When all queues empty and both robots idle, restart a new batch
        all_empty = (
            not self.part_a_queue and not self.part_b_queue
            and not self.conveyor_parts and not self.frame_parts
            and r1["held"] is None and r2["held"] is None
            and r1["state"] == "IDLE" and r2["state"] == "IDLE"
        )
        if all_empty:
            self._restock_queues()

        # ── joint angles ──────────────────────────────────────────────────────
        self._move_toward(r1)
        self._move_toward(r2)

    # ── public snapshot (for renderer) ────────────────────────────────────────

    def snapshot(self):
        """Return read-only view of sim state for renderer. No sim logic here."""
        r1, r2 = self.robot1, self.robot2
        _, j2_1, tcp1 = self.fk3(r1)
        _, j2_2, tcp2 = self.fk3(r2)
        return {
            "frame_num":      self.frame_num,
            "produced_count": self.produced_count,
            "station_busy":   self.station_busy,
            "paused":         self.paused,
            "cycle_times":    list(self.cycle_times),
            "r1": {
                "state": r1["state"], "held": r1["held"],
                "base": r1["base"].copy(), "j2": j2_1.copy(), "tcp": tcp1.copy(),
                "theta1": r1["theta1"], "theta2": r1["theta2"], "theta3": r1["theta3"],
            },
            "r2": {
                "state": r2["state"], "held": r2["held"],
                "base": r2["base"].copy(), "j2": j2_2.copy(), "tcp": tcp2.copy(),
                "theta1": r2["theta1"], "theta2": r2["theta2"], "theta3": r2["theta3"],
                "weld_index": r2["weld_index"],
            },
            "conveyor_parts": list(self.conveyor_parts),
            "frame_parts":    list(self.frame_parts),
            "filled_slots":   set(self.filled_slots),
            "outfeed_panels": list(self.outfeed_panels),
            "spark_frames":   list(self.spark_frames),
            "weld_points":    list(self.weld_points),
            "weld_targets":   list(self.weld_targets),
            "part_a_queue":   list(self.part_a_queue),
            "part_b_queue":   list(self.part_b_queue),
            "zones_occupied": [
                self._zone_occupied_by(zx, self.conveyor_parts) for zx in ZONES
            ],
        }


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 3 — CAMERA  (3D → 2D perspective projection)
# ═══════════════════════════════════════════════════════════════════════════════

class Camera:
    """
    Software perspective camera.
    project(x,y,z) → (sx, sy) screen pixels, or None if behind camera.

    To replace with OpenGL: keep SimulationEngine, replace this class
    with a glPerspective / gluLookAt setup.
    """

    def __init__(self, screen_w, screen_h):
        self.W = screen_w
        self.H = screen_h
        # Orbit parameters — same initial angle as v28 view_init(elev=28, azim=-48)
        self.elev   =  22.0     # degrees
        self.azim   = -52.0     # degrees
        self.dist   =  22.0     # closer — fills the phone screen
        self.cx     =  11.0     # look-at X  (centre of scene, belt mid)
        self.cy     =   1.5     # look-at Y
        self.cz     =   1.8     # look-at Z
        self.fov    =  72.0     # wider FOV fills vertical phone screen
        self._build()

    def _build(self):
        """Recompute camera matrix from orbit params."""
        az  = math.radians(self.azim)
        el  = math.radians(self.elev)
        # Eye position in world
        self.eye = np.array([
            self.cx + self.dist * math.cos(el) * math.cos(az),
            self.cy + self.dist * math.cos(el) * math.sin(az),
            self.cz + self.dist * math.sin(el),
        ])
        # Camera basis vectors
        fwd   = np.array([self.cx,self.cy,self.cz]) - self.eye
        fwd  /= np.linalg.norm(fwd)
        world_up = np.array([0.0,0.0,1.0])
        right = np.cross(fwd, world_up); right /= np.linalg.norm(right)
        up    = np.cross(right, fwd)
        self._fwd   = fwd
        self._right = right
        self._up    = up
        self._tan_half_fov = math.tan(math.radians(self.fov)/2)

    def project(self, x, y, z):
        """World → screen. Returns (px, py) or None if behind camera."""
        p   = np.array([x,y,z]) - self.eye
        d   = np.dot(p, self._fwd)
        if d < 0.1:
            return None                        # behind camera
        r   = np.dot(p, self._right)
        u   = np.dot(p, self._up)
        aspect = self.W / self.H
        sx  = int(self.W/2 + (r/d) / (self._tan_half_fov * aspect) * self.W/2)
        sy  = int(self.H/2 - (u/d) /  self._tan_half_fov             * self.H/2)
        return sx, sy

    def project_pt(self, pt):
        return self.project(pt[0], pt[1], pt[2])

    def depth(self, x, y, z):
        p = np.array([x,y,z]) - self.eye
        return np.dot(p, self._fwd)

    def reset(self):
        self.__init__(self.W, self.H)

    def orbit(self, d_azim, d_elev):
        self.azim  += d_azim
        self.elev   = max(-85, min(85, self.elev + d_elev))
        self._build()

    def zoom(self, delta):
        self.dist = max(8, min(100, self.dist + delta))
        self._build()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 4 — RENDERER
# ═══════════════════════════════════════════════════════════════════════════════

# Palette
C_RAIL      = (58,  127, 193)
C_RAIL_DK   = (26,  95,  160)
C_POST      = (224, 123,  32)
C_POST_DK   = (176,  90,   0)
C_BELT      = (130, 130, 130)
C_BELT_LN   = (170, 170, 255)
C_BELT_PICK = (218, 178,   0)
C_TABLE     = (160, 160, 160)
C_SHELF     = (190, 190, 190)
C_R1_A      = ( 26, 112, 196)
C_R1_B      = ( 10,  47, 110)
C_R2_A      = ( 46, 168,  74)
C_R2_B      = ( 10,  74,  30)
C_JOINT     = (200, 200, 200)
C_TCP       = (255,  40,  40)
C_SPARK     = (255, 165,  30)
C_WELD      = (140, 255,  60)
C_OUTFEED   = (  0, 180,  90)
C_GRID      = (220, 220, 220)
C_ZONE_MRK  = (120, 120, 220)
C_WIRE      = ( 40,  40,  40)
C_BG        = (245, 245, 245)


class Renderer:
    """
    Receives a camera and a sim snapshot.
    Calls only pygame.draw.* — no simulation logic.
    Swap this class to migrate to OpenGL / Panda3D / Ursina.
    """

    def __init__(self, surface, camera):
        self.surf = surface
        self.cam  = camera

    # ── primitives ────────────────────────────────────────────────────────────

    def _line(self, p3a, p3b, color, width=1):
        a = self.cam.project_pt(p3a)
        b = self.cam.project_pt(p3b)
        if a and b:
            pygame.draw.line(self.surf, color, a, b, width)

    def _dot(self, p3, color, r=4):
        s = self.cam.project_pt(p3)
        if s:
            pygame.draw.circle(self.surf, color, s, r)

    def _poly(self, pts3, color, wire_color=None, width=0):
        pts2 = [self.cam.project_pt(p) for p in pts3]
        if any(p is None for p in pts2):
            return
        if width == 0:
            pygame.draw.polygon(self.surf, color, pts2)
        if wire_color:
            pygame.draw.polygon(self.surf, wire_color, pts2, 1)

    # ── box faces sorted back-to-front ────────────────────────────────────────

    def _box(self, cx, cy, z_bottom, w, d, h, face_col, wire_col=C_WIRE, alpha=None):
        """
        Draw a 3D axis-aligned box projected to 2D.
        Faces are depth-sorted so visible faces render on top.
        cx,cy = world centre X,Y;  z_bottom = world bottom face Z.
        """
        x0, x1 = cx-w/2, cx+w/2
        y0, y1 = cy-d/2, cy+d/2
        z0, z1 = z_bottom, z_bottom+h

        # 6 faces: (centre_point, corner_list, normal_hint)
        faces = [
            # bottom
            ([(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0)],
             ((x0+x1)/2,(y0+y1)/2,z0)),
            # top
            ([(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)],
             ((x0+x1)/2,(y0+y1)/2,z1)),
            # front y0
            ([(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)],
             ((x0+x1)/2,y0,(z0+z1)/2)),
            # back y1
            ([(x0,y1,z0),(x1,y1,z0),(x1,y1,z1),(x0,y1,z1)],
             ((x0+x1)/2,y1,(z0+z1)/2)),
            # left x0
            ([(x0,y0,z0),(x0,y1,z0),(x0,y1,z1),(x0,y0,z1)],
             (x0,(y0+y1)/2,(z0+z1)/2)),
            # right x1
            ([(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)],
             (x1,(y0+y1)/2,(z0+z1)/2)),
        ]

        # Depth-sort faces: furthest first
        def face_depth(f):
            return self.cam.depth(*f[1])

        faces.sort(key=face_depth)

        for corners, centre in faces:
            # Back-face cull: skip faces whose centre is behind the eye
            if self.cam.depth(*centre) < 0.1:
                continue
            # Simple shading: darken faces by depth relative to top face
            dz = centre[2] - z_bottom
            shade = max(0.55, min(1.0, 0.65 + 0.35 * (dz / (h + 0.001))))
            shaded = tuple(int(c * shade) for c in face_col)
            self._poly(corners, shaded, wire_col)

    # ── scene elements ────────────────────────────────────────────────────────

    def _draw_grid(self):
        z = -0.01
        for gx in range(0, 27, 2):
            self._line((gx,-5,z),(gx,11,z), C_GRID, 1)
        for gy in range(-5,12,2):
            self._line((0,gy,z),(26,gy,z), C_GRID, 1)

    def _draw_conveyor(self, snap):
        # Belt slab
        self._box(12.0, CONVEYOR_Y, -BELT_THICK,
                  24.0, BELT_HALF_W*2, BELT_THICK, C_BELT)
        # Zone lines on belt surface
        for zx in ZONES:
            self._line((zx,-BELT_HALF_W,0.01),(zx,BELT_HALF_W,0.01), C_BELT_LN, 1)
        # Pickup highlight
        self._line((PICKUP_X,-BELT_HALF_W,0.015),(PICKUP_X,BELT_HALF_W,0.015),
                   C_BELT_PICK, 2)

    def _draw_fixture_table(self, snap):
        self._box(TABLE_X, TABLE_Y, -TABLE_THICK,
                  TABLE_W, TABLE_D, TABLE_THICK, C_TABLE)
        # Frame slot guides
        hw = FRAME_HALF + 0.2
        for a, b in [
            ((TABLE_X-hw,TABLE_Y+FRAME_HALF,0.01),(TABLE_X+hw,TABLE_Y+FRAME_HALF,0.01)),
            ((TABLE_X-hw,TABLE_Y-FRAME_HALF,0.01),(TABLE_X+hw,TABLE_Y-FRAME_HALF,0.01)),
            ((TABLE_X-FRAME_HALF,TABLE_Y-hw,0.01),(TABLE_X-FRAME_HALF,TABLE_Y+hw,0.01)),
            ((TABLE_X+FRAME_HALF,TABLE_Y-hw,0.01),(TABLE_X+FRAME_HALF,TABLE_Y+hw,0.01)),
        ]:
            self._line(a, b, (140,140,140), 1)

    def _draw_queue_shelves(self):
        sx, sw, shw, sth = 3.0, 5.0, 0.6, 0.06
        for z in [0.8, 1.4, 2.0, 2.6]:
            self._box(sx, -3.2, z-sth, sw, shw*2, sth, C_SHELF)
            self._box(sx,  3.2, z-sth, sw, shw*2, sth, C_SHELF)

    def _draw_stud(self, part, alpha_dim=False):
        cx, cy   = part["x"], part["y"]
        zb       = part["z"]
        w, d, h  = part["w"], part["d"], part["h"]
        fc = C_RAIL if part["type"] == "A" else C_POST
        dk = C_RAIL_DK if part["type"] == "A" else C_POST_DK
        if alpha_dim:
            fc = tuple(int(c*0.35) for c in fc)
            dk = tuple(int(c*0.35) for c in dk)
        self._box(cx, cy, zb, w, d, h, fc)
        # top cap stripe
        self._box(cx, cy, zb+h-0.04, w, d, 0.04, dk)

    def _draw_robot(self, rsnap, col_upper, col_lower):
        base = rsnap["base"]
        j2   = rsnap["j2"]
        tcp  = rsnap["tcp"]

        # Upper arm
        self._line(base, j2, col_upper, 5)
        # Lower arm
        self._line(j2, tcp, col_lower, 4)
        # Joints
        self._dot(base, C_JOINT, 7)
        self._dot(j2,   C_JOINT, 6)
        self._dot(tcp,  C_JOINT, 4)
        # TCP marker
        self._dot(tcp, C_TCP, 5)
        # Gripper fingers
        tcp_arr = np.array(tcp)
        for sign in (-1, 1):
            tip = tcp_arr + np.array([0.0, sign*0.20, 0.22])
            self._line(tcp, tip, C_TCP, 2)

    def _draw_sparks(self, snap):
        for s in snap["spark_frames"]:
            pt = self.cam.project(s["x"], s["y"], s["z"])
            if pt:
                r = max(1, int(s["life"]/5))
                pygame.draw.circle(self.surf, C_SPARK, pt, r)

    def _draw_weld_stars(self, snap):
        for wp in snap["weld_points"]:
            self._dot(wp, C_WELD, 6)

    # ── depth-sorted full scene draw ──────────────────────────────────────────

    def draw(self, snap):
        self.surf.fill(C_BG)
        self._draw_grid()
        self._draw_conveyor(snap)
        self._draw_fixture_table(snap)
        self._draw_queue_shelves()

        # Collect all parts for depth sort
        render_items = []

        # Queue parts
        for i, p in enumerate(snap["part_a_queue"]):
            render_items.append(("stud", p, i > 0))
        for i, p in enumerate(snap["part_b_queue"]):
            render_items.append(("stud", p, i > 0))

        # Conveyor parts
        for p in snap["conveyor_parts"]:
            render_items.append(("stud", p, False))

        # Held parts
        if snap["r1"]["held"]:
            render_items.append(("stud", snap["r1"]["held"], False))
        if snap["r2"]["held"]:
            render_items.append(("stud", snap["r2"]["held"], False))

        # Frame parts
        for p in snap["frame_parts"]:
            render_items.append(("stud", p, False))

        # Outfeed panels
        for f in snap["outfeed_panels"]:
            render_items.append(("outfeed", f, False))

        # Robots
        render_items.append(("robot1", snap["r1"], False))
        render_items.append(("robot2", snap["r2"], False))

        # Depth sort all items (furthest first)
        def item_depth(item):
            kind, obj, _ = item
            if kind == "stud" or kind == "outfeed":
                return self.cam.depth(obj["x"], obj["y"],
                                      obj["z"] + obj.get("h",0)/2)
            elif kind in ("robot1","robot2"):
                return self.cam.depth(*obj["base"])
            return 0

        render_items.sort(key=item_depth)

        for kind, obj, dimmed in render_items:
            if kind == "stud":
                self._draw_stud(obj, alpha_dim=dimmed)
            elif kind == "outfeed":
                self._box(obj["x"], obj["y"], obj["z"],
                          obj["w"], obj["d"], obj["h"], C_OUTFEED)
            elif kind == "robot1":
                self._draw_robot(obj, C_R1_A, C_R1_B)
            elif kind == "robot2":
                self._draw_robot(obj, C_R2_A, C_R2_B)

        self._draw_sparks(snap)
        self._draw_weld_stars(snap)


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 5 — HUD
# ═══════════════════════════════════════════════════════════════════════════════

class HUD:
    def __init__(self, surface):
        self.surf = surface
        pygame.font.init()
        self.fn  = pygame.font.SysFont("monospace", 13, bold=True)
        self.fsm = pygame.font.SysFont("monospace", 11)
        self.ftl = pygame.font.SysFont("monospace", 15, bold=True)

    def _txt(self, text, font, color, x, y):
        surf = font.render(text, True, color)
        self.surf.blit(surf, (x, y))
        return surf.get_height()

    def draw(self, snap, fps):
        r1, r2 = snap["r1"], snap["r2"]

        def deg(r): return math.degrees(r) % 360

        total_parts = (
            len(snap["part_a_queue"]) + len(snap["part_b_queue"])
            + len(snap["conveyor_parts"]) + len(snap["frame_parts"])
            + (1 if r1["held"] else 0)
            + (1 if r2["held"] else 0)
        )

        z_occ = ["■" if o else "□" for o in snap["zones_occupied"]]
        slots  = " ".join(
            f"{FRAME_SLOTS[i]['type']}{i}:{'✓' if i in snap['filled_slots'] else '·'}"
            for i in range(len(FRAME_SLOTS))
        )
        avg_cycle = (sum(snap["cycle_times"])/len(snap["cycle_times"])
                     if snap["cycle_times"] else 0.0)

        lines = [
            ("CHAPPELL ROBOTICS  v29",            (20,  60, 160), self.fn),
            ("─"*28,                              (100,100,100), self.fsm),
            (f"R1: {r1['state']:<16}",            (30, 100, 200), self.fsm),
            (f"R2: {r2['state']:<16}",            (20, 160,  60), self.fsm),
            ("─"*28,                              (100,100,100), self.fsm),
            (f"Panels produced : {snap['produced_count']}",   (40,40,40), self.fsm),
            (f"Avg cycle time  : {avg_cycle:.1f}s",(40,40,40), self.fsm),
            (f"Belt parts      : {len(snap['conveyor_parts'])}",(40,40,40),self.fsm),
            (f"Frame studs     : {len(snap['frame_parts'])}/{PANEL_SIZE}", (40,40,40),self.fsm),
            (f"Total parts     : {total_parts}/{snap['produced_count']*PANEL_SIZE + len(snap['frame_parts']) + len(snap['conveyor_parts']) + len(snap['part_a_queue']) + len(snap['part_b_queue']) + (1 if r1['held'] else 0) + (1 if r2['held'] else 0)}",(40,40,40),self.fsm),
            (f"Belt status     : {'HOLD' if snap['station_busy'] else 'RUN '}",(40,40,40),self.fsm),
            ("─"*28,                              (100,100,100), self.fsm),
            (f"Zones LOAD Z1 Z2 PICK",            (80, 80, 160), self.fsm),
            (f"      {z_occ[0]}    {z_occ[1]}   {z_occ[2]}   {z_occ[3]}", (80,80,160),self.fsm),
            (f"Slots: {slots}",                   (80, 80, 160), self.fsm),
            ("─"*28,                              (100,100,100), self.fsm),
            (f"R1 Yaw:{deg(r1['theta1']):6.1f}°  Shldr:{math.degrees(r1['theta2']):6.1f}°", (30,100,200),self.fsm),
            (f"R2 Yaw:{deg(r2['theta1']):6.1f}°  Shldr:{math.degrees(r2['theta2']):6.1f}°", (20,160, 60),self.fsm),
            ("─"*28,                              (100,100,100), self.fsm),
            (f"FPS: {fps:.0f}   F:{snap['frame_num']}",  (80,80,80),self.fsm),
            ("SPACE:pause  R:reset cam",           (120,120,120),self.fsm),
            ("Arrows:orbit  +/-:zoom  Q:quit",     (120,120,120),self.fsm),
            ("PAUSED" if snap["paused"] else "",   (220,  0,  0), self.fn),
        ]

        lh  = self.fsm.get_linesize() + 2
        pad = 7
        box_w = 222
        box_h = len(lines) * lh + pad * 2 + 4

        bg = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
        bg.fill((255, 255, 255, 210))
        self.surf.blit(bg, (5, 5))
        pygame.draw.rect(self.surf, (80,80,80), (5, 5, box_w, box_h), 1)

        y = 5 + pad
        for text, color, font in lines:
            if text:
                s = font.render(text, True, color)
                self.surf.blit(s, (5 + pad, y))
            y += lh

        # Title bar
        title = self.ftl.render(
            "CHAPPELL ROBOTICS  |  Assembly Cell v29  |  3D Sim / Pygame Renderer",
            True, (30, 30, 30))
        tw = title.get_width()
        W  = self.surf.get_width()
        bg2 = pygame.Surface((tw+16, title.get_height()+8), pygame.SRCALPHA)
        bg2.fill((255,255,255,200))
        self.surf.blit(bg2, (W//2 - tw//2 - 8, 6))
        self.surf.blit(title, (W//2 - tw//2, 10))


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 6 — MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    pygame.init()
    pygame.font.init()

    info = pygame.display.Info()
    W, H = info.current_w, info.current_h

    try:
        screen = pygame.display.set_mode((W, H), pygame.FULLSCREEN | pygame.DOUBLEBUF)
    except Exception:
        W, H = 1280, 720
        screen = pygame.display.set_mode((W, H), pygame.DOUBLEBUF)

    pygame.display.set_caption("CHAPPELL ROBOTICS | Assembly Cell v29")

    sim      = SimulationEngine()
    camera   = Camera(W, H)
    renderer = Renderer(screen, camera)
    hud      = HUD(screen)
    clock    = pygame.time.Clock()

    # Keyboard orbit state
    orbit_speed = 1.8   # degrees per frame
    zoom_speed  = 0.8

    running = True
    fps     = 35.0
    _last_touch = None   # for touch drag orbit on Android

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_r:
                    camera.reset()
                elif event.key == pygame.K_SPACE:
                    sim.paused = not sim.paused
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    camera.zoom(-zoom_speed * 2)
                elif event.button == 5:
                    camera.zoom( zoom_speed * 2)
                elif event.button == 1:
                    _last_touch = event.pos
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    _last_touch = None
            elif event.type == pygame.MOUSEMOTION:
                if _last_touch and pygame.mouse.get_pressed()[0]:
                    dx = event.pos[0] - _last_touch[0]
                    dy = event.pos[1] - _last_touch[1]
                    camera.orbit(-dx * 0.35, dy * 0.25)
                    _last_touch = event.pos
            # Android touch events (finger drag)
            elif event.type == pygame.FINGERDOWN:
                _last_touch = (int(event.x * W), int(event.y * H))
            elif event.type == pygame.FINGERUP:
                _last_touch = None
            elif event.type == pygame.FINGERMOTION:
                if _last_touch:
                    nx = int(event.x * W)
                    ny = int(event.y * H)
                    dx = nx - _last_touch[0]
                    dy = ny - _last_touch[1]
                    camera.orbit(-dx * 0.35, dy * 0.25)
                    _last_touch = (nx, ny)

        # Continuous key hold for orbit/zoom
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]  or keys[pygame.K_a]: camera.orbit(-orbit_speed, 0)
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]: camera.orbit( orbit_speed, 0)
        if keys[pygame.K_UP]    or keys[pygame.K_w]: camera.orbit(0,  orbit_speed*0.6)
        if keys[pygame.K_DOWN]  or keys[pygame.K_s]: camera.orbit(0, -orbit_speed*0.6)
        if keys[pygame.K_EQUALS] or keys[pygame.K_PLUS]:  camera.zoom(-zoom_speed)
        if keys[pygame.K_MINUS]:                          camera.zoom( zoom_speed)

        # Step simulation
        sim.step()

        # Render
        snap = sim.snapshot()
        renderer.draw(snap)
        hud.draw(snap, fps)

        pygame.display.flip()
        clock.tick(35)
        fps = clock.get_fps()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
