# CR6_V8_0_Dual_Robot_Cell.py — Engineering Recovery

**Source:** `simulations/CR6_V08_Dual_Robot_Cell/CR6_V8_0_Dual_Robot_Cell.py` (577 lines, matplotlib)
**Status:** reference / not-assessed

## Relationship to the earlier CR6 files

Same robot geometry/kinematics as `CR6_6Axis_V3_1_Corrected.py` / `CR6_6Axis_V6_1_Workspace_Guard.py.py` (identical DH constants, FK/IK math — the file's own header states "copied exactly from V6.1"). This is a real architectural step up: a proper OOP rewrite (`Robot`, `Part`, `World` classes) supporting **two robots handing off one part**, not a single robot's isolated cycle.

## Cell layout (explicit constants)

- Robot A base: `[0, 0, 0]` — picks from conveyor (`Y = -2.8`)
- Robot B base: `[0, 3.5, 0]` — picks from fixture, places to output
- Fixture position: `[-1.0, 1.0, 1.2]`
- Output position: `[-2.5, 4.5, 1.2]`
- `LIFT_Z = 2.0` (verified-reachable transit height from the conveyor), `SAFE_Z = 3.2` (approach height for fixture/output, a taller clearance than the conveyor lift height)
- `SAFE_REACH = MAX_REACH * 0.90` (= 4.5) — same reachability-guard pattern as the Workspace Guard file, tuned to a slightly different margin (90% vs. that file's 92%)

## Part ownership state machine — the real coordination mechanism

Explicitly documented in the file's own header as the **only** communication channel between the two robots (design principle #2/#3 in the docstring):

```
ON_CONVEYOR → COMMITTED_TO_A → HELD_BY_A → IN_FIXTURE → HELD_BY_B → COMPLETE
```

- `COMMITTED_TO_A`: Robot A has claimed the part and begun its approach, but the part still visually renders on the moving conveyor (no teleport) until physically grasped.
- Ownership transitions only at physical proximity events (TCP within `0.35` of the part/fixture/output), not at abstract timer events — e.g. `HELD_BY_A` only triggers once `‖ra.tcp() - p.pos‖ < 0.35`.
- Explicit safety fallback: if Robot A's motion sequence finishes without ever having picked up the part (a failure case), ownership reverts to `ON_CONVEYOR` rather than leaving the part in a stuck/ambiguous state.
- Robot B only launches once ownership reaches `IN_FIXTURE` — the two robots' motion sequences are decoupled and reactive to shared state, not scripted against each other's timing directly.

## `Robot` class — reusable per-arm engine (worth noting for the smart-geometry/composability direction)

Each `Robot` instance is self-contained: holds its own base position, joint state, color scheme, and a waypoint-sequence interpolator (`launch()`/`step()`) that Cartesian-interpolates position and SLERPs orientation between named waypoints, with per-waypoint dwell support. `launch()` explicitly prepends the robot's *current* TCP as the first waypoint so a newly-launched sequence starts smoothly from wherever the arm currently is, rather than snapping. This is the closest thing in any of the 11 files to a genuinely reusable, parameterized "smart component" pattern — two independently-instantiated robots sharing one class, each given a different base position and waypoint program.

## Visualization (matplotlib)

Same animated 3D + slider pattern as the earlier CR6 files, extended to render two robots simultaneously (each with its own 6-segment color scheme and motion trace), fixture/output "station" markers (flat colored pads + marker glyph), and a dual-robot HUD (both arms' joint angles, part ownership state, cycle-complete counter).

## Notes / unknowns

- No stated units (same gap as `CR6_6Axis_V3_1_Corrected.py`); axis labels here are unitless (`"X"`, `"Y"`, `"Z"`, not `"X (m)"`).
- "V7.5" appears in the docstring title but the filename and on-screen title both say "V8.0" — an unresolved version-label inconsistency in the source itself, reproduced here as observed, not resolved.
