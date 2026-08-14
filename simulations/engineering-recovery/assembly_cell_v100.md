# assembly_cell_v100.py — Engineering Recovery

**Source:** `simulations/Dual_CR6_Cell/assembly_cell_v100.py` (1,199 lines — largest of the 11)
**Status:** reference / not-assessed
**Current renderer: pygame, not matplotlib.** The automated matplotlib-reference count for this file (1) is misleading — that single hit is the docstring's own historical note ("Renderer: Matplotlib → PyVista Pygame"), not live code. This file already completed one migration (matplotlib → pygame, "v28 → v29" per its own header) before landing in this state.

## Architecture — already renderer-agnostic by design

Explicitly split into four layers, stated directly in the docstring as intentional:
- `SimulationEngine` — pure logic/state machines/kinematics/part-tracking, zero rendering dependency
- `Camera` — custom 3D→2D perspective projection (rotate/zoom), independent of the simulation
- `Renderer` — consumes a simulation snapshot, issues `pygame.draw.*` calls
- `HUD` — metrics overlay, pure pygame text

The docstring states this split was deliberate specifically so the renderer "can be swapped for OpenGL / Panda3D / Ursina with no sim changes." **Of all 11 files, this is the one already structured for a clean pyvista swap** — only `Renderer`/`Camera`/`HUD` would need replacing; `SimulationEngine` should port unchanged.

## What it simulates: a two-robot light-gauge-steel frame assembly cell

Real, specific manufacturing engineering, not a generic pick-and-place demo:

**Parts**: "studs" of two types — type A (`RAIL_W=3.0` long in X, `RAIL_D=0.35`) and type B (`POST_W=0.35`, `POST_D=3.0` long in Y), both `STUD_H=0.35` tall. This is a rail/post framing pattern consistent with light-gauge-steel (LGS) wall-panel construction.

**Process flow**:
1. Studs queue in two supply stacks (4 of each type), fed onto an accumulation conveyor with 4 named zones (`ZONE_LOAD=3.5, ZONE_1=6.5, ZONE_2=9.5, ZONE_PICKUP=13.0`), each zone `ZONE_LENGTH=3.0` — parts advance zone-to-zone only when the next zone is unoccupied (a real queueing/blocking model, not free continuous motion).
2. Robot 1 (base `[0,0,0]`) picks studs from the supply stacks onto the conveyor's load zone.
3. Robot 2 (base `[16.0, 3.5, 0]`) picks studs from the conveyor pickup zone and places them into a 4-slot fixture table (`TABLE_X=18.0, TABLE_Y=4.5`) — slots alternate type A/B around a frame center (`FRAME_HALF=1.3`), i.e., two rails + two posts forming a rectangular frame.
4. Once all 4 slots are filled, Robot 2 performs a `WELDING` pass at 4 explicit weld-target points (frame corners), then transitions to `FRAME_COMPLETE` and the finished panel exits to an outfeed.
5. Queues auto-restock (`_restock_queues()`) for continuous production, preserving the running `produced_count`.

**Robot kinematics here are different from the CR6 files**: a simpler 3-DOF planar arm (`fk3`/`_ik3`, angles `theta1/theta2/theta3`, link lengths `L1=L2=4.0`), not the CR6's 6-DOF DH chain — this cell's arms are a distinct, simpler kinematic model, not a reuse of the CR6 robot.

**Real engineering safeguards present in the code, explicitly labeled as bug fixes in the docstring** ("v28 fixes," preserved unchanged into v29):
- `load_zone_clear()` — a center-point occupancy check before allowing the load zone to accept a new part.
- `_advance_wp()` — an explicit re-entry guard against double-advancing a completed waypoint sequence.
- Stall detector — fires only when *both* robots are simultaneously `IDLE` with the full part count accounted for but the frame not yet complete (a real deadlock-detection heuristic, printed as a diagnostic, not silently ignored).
- **Part conservation tracking**: every `step()` computes `total_parts` across every possible location (both queues + conveyor + frame + held-by-either-robot) and this is used as the invariant the stall detector checks against — a genuine "nothing was lost or duplicated" sanity check built into the simulation itself.

## State machines (both robots, from the docstring's own summary)

`IDLE → PICKING_A/B → PRE_PLACE → PLACING → PICKING → PLACING → WELDING → FRAME_COMPLETE`

## Notes / unknowns

- Units unstated (same gap as the CR6 files).
- The interaction between this cell's simpler 3-DOF arms and whether they're meant to represent the same physical CR6 robots or a different machine type is not stated anywhere in the file — flagged as an open question, not resolved here.
- Controls (arrow/WASD orbit, +/- zoom, R reset, SPACE pause, ESC/Q quit) confirm this was built as an interactively-explorable simulation, not just a fixed-camera animation loop like the matplotlib files.
