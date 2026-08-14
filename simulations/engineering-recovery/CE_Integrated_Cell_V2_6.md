# CE_Integrated_Cell_V2_6.py — Engineering Recovery

**Source:** `simulations/Integrated-_Cell_V2_6-/CE_Integrated_Cell_V2_6.py` (1,165 lines, matplotlib)
**Status:** reference / not-assessed

## What it simulates: a complete four-station modular wall manufacturing cell

The most complete single-file factory cell of the 11 — four coordinated stations along one X-axis centerline, four robots, a servo roller transfer, a tilting table, and an overhead crane, all in one simulation:

1. **Raw material rack** (`X=-10.0`) — stud/track stock, 5 stacked slots.
2. **TABLE_JIG_FIXED** (`X=-6.0`) — LGS framing assembly. Two robots: `CR6-F1` (material handler, base `Y=-2.6`) picks members from the rack and places them per an explicit assembly order (`ASSEMBLY_SEQ = [TRACK, STUD, STUD, STUD, TRACK]` — matches the same bottom-track/3-stud/top-track pattern seen in `Dual_Robot_Jig_Frame_V1_1.py`); `CR6-F2` (fastener, base `Y=+2.6`) fastens each member immediately after placement.
3. **TABLE_JIG_ROLLER** (`X=-1.0`) — a powered servo roller transfer zone with no robots; indexes the completed frame from the fixed table to the tilt table (`ROLLER_SPEED=0.07`/frame).
4. **TABLE_JIG_TILT** (`X=+5.5`) — sheathing, inspection, and tilt-to-vertical. Two more robots: `CR6-S1` (sheet pickup from a magazine at `X=3.2`), `CR6-S2` (fastens the sheathing in a specific 5-point pattern — 4 corner points plus center, `FASTEN_PTS`). The table then rotates about a pivot edge (`TILT_PIVOT_X=3.2`) to bring the finished wall from flat to vertical.
5. **Overhead crane** (park `X=11.0`) — travels to the tilt table, lowers, hooks, lifts the finished vertical wall, completing the cycle.

All four robots share the same CR6 DH geometry as every other CR6 file (`D1=1.5, A2=2.5, A3=2.0, D6=0.5`, `SAFE_REACH = MAX_REACH*0.90`).

## ★ Confirmed cross-file coordinate match with CE_Overhead_Gantry_V1.py

`TILT_CX = 5.5` here is **exactly** `CE_Overhead_Gantry_V1.py`'s `PICKUP_X = 5.5` — confirming, from this file's own side, that the two simulations were deliberately built to share one coordinate space, not just similar by coincidence. The standalone gantry file appears to be a separate, far more structurally-detailed extraction of just this cell's crane subsystem (this file's own `OverheadCrane` class, defined at line 302, is a much simpler embedded model than the dedicated gantry file's full portal/bridge/trolley/hook anatomy) — two different fidelity levels of the same conceptual machine, not duplicates of the same code.

## Discrepancy worth flagging (not silently resolved)

The docstring states the tilt table "rotates 0° → 90° (wall goes vertical)," but the actual code caps `tilt_angle` at **60.0 degrees**, not 90 (`if s.tilt_angle >= 60.0: s.tilt_angle = 60.0`). This is either an unfinished feature, a deliberate later change not reflected in the docstring, or a bug — not determined here, recorded as an open discrepancy between stated intent and actual behavior rather than assumed to be either.

## Architecture

`Robot` (shared across all four arms), `OverheadCrane`, `CellState` (tracks `tilt_angle` and per-station state names), `World` orchestrator. Waypoint builders are per-role (`build_f1_place`, `build_f2_fasten`, `build_s1_pickup`, `build_s2_fasten`) rather than one generic builder — each robot's job is encoded as its own function.

## Notes

- `Poly3DCollection`-based solid rendering (like the gantry file), not just line/scatter — `draw_tilt_table_assembly()` explicitly models the tilt table as a rigid body that rotates as one assembly about its pivot edge, correctly re-deriving geometry at `tilt_angle=0` vs. mid-rotation vs. `60°`, not just repositioning a flat sprite.
- Units unstated, consistent with the rest of the CR6-lineage files.
