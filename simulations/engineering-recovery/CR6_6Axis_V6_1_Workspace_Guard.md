# CR6_6Axis_V6_1_Workspace_Guard.py.py — Engineering Recovery

**Source:** `simulations/CR6_6axis_Object_Tracking/CR6_6Axis_V6_1_Workspace_Guard.py.py` (395 lines, matplotlib) — note the double `.py.py` extension in the actual filename, reproduced here as observed, not corrected.
**Status:** reference / not-assessed

## Relationship to CR6_6Axis_V3_1_Corrected.py

Same robot: identical DH parameters (`D1=1.5, A2=2.5, A3=2.0, D6=0.5`, `MAX_REACH=5.0`), identical FK/IK/quaternion math, identical conveyor cell layout. This is a direct evolution, not a different robot — the directory name (`CR6_6axis_Object_Tracking`) and version bump (V3.1 → V6.1) indicate this is the "moving target interception" generation of the same cell.

## What's new here (the real engineering content of this version)

- **Continuous conveyor feed**: the part now moves continuously along X (`CONV_SPEED = 0.040`/frame) instead of sitting static at a fixed pick point — this file simulates intercepting a moving part, not picking a stationary one.
- **Workspace/reachability guard**: `SAFE_INTERCEPT_RADIUS = MAX_REACH * 0.92` (= 4.6) — an explicit safety margin below the robot's true kinematic limit. `is_part_reachable(p_x)` checks whether the part's current position is within this safe radius from the shoulder before the robot commits to a pick attempt — the robot waits at `HOME` until the moving part enters this validated window, then launches the intercept sequence.
- **Reachability sphere visualization**: `draw_reachability_sphere()` renders the safe-intercept boundary as a translucent wireframe sphere centered at the shoulder height (`z = D1`) — a real debugging/verification aid for confirming the guard logic visually, not just numerically.
- **State-handling bug fix, explicit in the docstring**: a `waiting_for_home_reset` flag was added specifically to fix a part-recycling bug — the previous version's part reset logic (visible in V3.1) was apparently unsafe/premature; this version only resets the part to conveyor-start once the arm has fully returned `HOME` *after* a completed `PLACE`, not immediately at `PLACE`.
- **`parts_cleared` counter**: running count of completed pick-place cycles, shown on the HUD.

## Units note

Unlike `CR6_6Axis_V3_1_Corrected.py`, this file explicitly labels its axes `"X (m)"`, `"Y (m)"`, `"Z (m)"` — confirming the scene units here are meters. This does not necessarily confirm the same for V3.1 (a separate file with no axis labels), but given both share identical geometry constants, meters is the reasonable working assumption for the whole CR6 lineage unless a later file contradicts it.

## Visualization migration note

Same matplotlib stack as V3.1 (`FuncAnimation`, `Slider`), plus a wireframe sphere render (`plot_wireframe`) for the reachability guard — this is an additional geometry type a pyvista port needs to reproduce (translucent sphere shell), not just line/scatter primitives.
