# CR6_6Axis_V3_1_Corrected.py — Engineering Recovery

**Source:** `simulations/CR6_6Axis/CR6_6Axis_V3_1_Corrected.py` (349 lines, matplotlib)
**Status:** reference / not-assessed (per `SIMULATION_METADATA.yaml`)

## Observed facts

**Robot kinematic model** — analytical 6-DOF DH (Denavit-Hartenberg) chain, fully implemented both directions:
- DH constants: `D1=1.5` (base height), `A2=2.5` (upper-arm/shoulder link), `A3=2.0` (forearm/elbow link), `D6=0.5` (wrist-to-tool offset). `MAX_REACH = A2+A3+D6 = 5.0` (units not specified anywhere in the file — could be meters, feet, or arbitrary sim units; not stated).
- `forward_kinematics(q)`: standard DH transform chain for 6 joints (J1 base rotation, J2 shoulder, J3 elbow, J4 wrist rotate, J5 wrist tilt, J6 tool plate), returns the position of every joint along the chain.
- `inverse_kinematics(target_pos, R06)`: full analytical (closed-form, not iterative) geometric IK — wrist-center decomposition, 2-link planar solve for J2/J3 (elbow-down configuration specifically chosen), Euler-ZYZ extraction for the wrist joints J4/J5/J6. Returns `None` if the target is outside `[|A2-A3|, 0.99*(A2+A3)]` reach (an explicit, intentional reachability guard).
- Rotation utilities: RPY→matrix, matrix→quaternion, quaternion→matrix, and SLERP — used to interpolate tool orientation smoothly between waypoints, not just position.

**Cell/conveyor geometry:**
- Conveyor spans `x: [-4.0, 4.0]`, centered at `y = -2.8`, belt width `1.0`, belt surface at `z = 1.0`, legs to floor (`z = 0`).
- A fixed "pick zone" marker at `x = 1.0` on the conveyor.
- Part: `PART_HEIGHT = 0.2`, sits on the conveyor with its center at `z = 1.10` (belt height + half part height).

**Motion/process state machine:**
- Sequence: `HOME → PICK_APPROACH → PICK → LIFT → PLACE → HOME`, looping.
- Linear position interpolation + quaternion SLERP orientation interpolation between named waypoints.
- Dwell logic at `PICK`/`PLACE` states: holds for `DWELL_LIMIT = 15` frames before advancing.
- Pick/place attach logic: at the `PICK` state, if the tool-center-point (TCP) comes within `0.25` distance units of the part, `is_attached` becomes `True` and the part is thereafter rigidly carried at the TCP position; released at `PLACE`; part resets to its conveyor origin only if the arm returns `HOME` without ever having attached (i.e., a failed-pick cycle resets cleanly rather than leaving the part in an ambiguous state).

**Visualization (matplotlib):** 3D animated (`FuncAnimation`, 40ms interval), interactive "Motion Speed" slider widget, per-segment colored arm rendering (6 distinct colors/widths per link), a fading motion trace of the last 600 TCP positions, and a live text HUD (joint angles in degrees, TCP position, part position, attach state, current state-machine phase).

## Interpretation / notes (not verified against a real spec)

- "CR6" strongly suggests this models a real commercial 6-axis collaborative robot arm (naming matches Dobot's CR6 line), but the DH parameters here are round, simplified numbers (`1.5, 2.5, 2.0, 0.5`) — I have not confirmed these against any actual CR6 datasheet. Treat as an idealized/simplified kinematic model unless corroborated elsewhere.
- No units are declared anywhere in the file (no comment, docstring, or constant naming indicates meters vs. feet vs. arbitrary scene units).
- This is a single-robot, single-part pick-and-place cycle — no multi-robot coordination, no collision checking against other geometry, no gripper/tool model beyond a point TCP.

## Visualization migration note

Uses `matplotlib.pyplot`, `mpl_toolkits.mplot3d` (via `projection="3d"`), `FuncAnimation`, and `matplotlib.widgets.Slider`. A pyvista port would need an equivalent for: animated redraw loop, an interactive scalar/slider control, and text HUD overlay — not just static mesh rendering.
