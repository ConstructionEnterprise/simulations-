# Factory_Rail_V2.py — Engineering Recovery

**Source:** `simulations/Factory_Rail_v2/Factory_Rail_V2.py` (700 lines, matplotlib)
**Status:** reference / not-assessed

## What it simulates: a rail-mounted robot transporting a finished panel between two jigs

Real, specific concept per the docstring: "A standard CR6 robot is mounted on a FactoryRail linear track. The rail moves the robot base along the X-axis between stations. Robot kinematics are unchanged — the base simply relocates." This is the clearest example among the 11 of the exact "rail-system"/gantry-transport category — the robot itself is unmodified (same CR6 DH geometry as every other file: `D1=1.5, A2=2.5, A3=2.0, D6=0.5`), only its *base position* becomes a moving variable driven by a separate `FactoryRail` object.

**Layout**: rail spans `X = 0.0` (Table_Jig_1, west, destination) to `X = 8.0` (Table_Jig_2, east, source — holds a completed wall frame at start). `LIFT_Z = 2.5` transit hold height.

**Demonstration sequence (from the docstring, matches the code)**: robot starts parked at Jig_1 → rail travels east to Jig_2 → robot picks the completed `WallFrame` off Jig_2 → lifts it → rail travels west back to Jig_1 → robot places the frame on Jig_1 → cycle repeats. This directly continues the LGS wall-panel narrative from `Dual_Robot_Jig_Frame_V1_1.py` — that file assembles a panel; this one models moving a *finished* panel between stations.

**WallFrame payload**: `FRAME_W=2.4` (X), `FRAME_D=1.6` (Y), `FRAME_THICK=0.10` — modeled lying flat/horizontal on the jig surface, not standing on edge.

**Explicit V1→V2 fixes, stated in the docstring** (recorded as the author's own account, not independently verified): an `INIT_PARK` stall was removed (sim now starts directly in `RAIL_TO_JIG2` rather than an initial park motion that apparently caused a hang), a frame Z-height correction (frame previously rendered partially inside the jig surface rather than resting on top of it), and a station-naming cleanup (`Jig_1`/`Jig_2` replacing an earlier, presumably confusing naming scheme).

## Architecture

Three explicit classes: `FactoryRail` (the track itself as a first-class object — position, travel state), `CR6` (the robot, base position supplied externally by the rail rather than fixed), `WallFrame` (the payload). `World` orchestrates a rail-transit state machine (`RAIL_TO_JIG2` ⟷ `RAIL_TO_JIG1`) alongside the robot's own pick/place waypoint sequences (`build_pickup_sequence`, `build_place_sequence`).

## Notes

- Reachability margin here is `SAFE_REACH = MAX_REACH * 0.97` — a tighter (less conservative) margin than the 90–92% used in the CR6 dual-cell and workspace-guard files; not explained further in comments, just stated as "verified reachable."
- Units unstated, consistent with the rest of the CR6 lineage.
