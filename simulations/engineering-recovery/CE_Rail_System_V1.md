# CE_Rail_System_V1.py — Engineering Recovery

**Source:** `simulations/Rail_System_V1/CE_Rail_System_V1.py` (795 lines, matplotlib)
**Status:** reference / not-assessed

## ★★ Important: this file's robot names match the LIVE production Digital Twin exactly

Robots here are named **A1, A2, B1, B2** — Rail A near/far and Rail B near/far. This is not a coincidence: the *live* Digital Twin (`CE_Integrated_Cell_V3_0-6.py`, the protected file — see `CE_Integrated_Cell_V3_0-6.md`) constructs its real robots with these exact same four names (`IntegratedCell.__init__: self.A1/A2/B1/B2`), and FF's own frontend (`frontend/src/features/robotics/roboticsData.ts`) hardcodes `ROBOT_NAMES = ["A1", "A2", "B1", "B2"]` to match. **This archived file appears to be a standalone, structurally-detailed geometric model of the same physical dual-rail/four-robot hardware architecture that the live twin represents more abstractly** — real value for engineering recovery: this is the closest thing among the 11 archived files to primary-source documentation of the live twin's actual physical rail/ATC layout. Worth a deliberate side-by-side comparison against the live file's own robot/rail constants in a future pass (not done here — this file was read, the live file was not touched or diffed against, per the immutability boundary).

## Real, specific equipment (like the gantry file, has an explicit hardware reference)

Docstring: **"Reference: Chappell Robotics CR6 Rail System finalized render (June 2026)."** Same coordinate-space cross-reference pattern as the gantry file: `JIG_CX = 5.50 # jig center X (matches V2.6 TILT_CX)` — a third file confirmed to share `CE_Integrated_Cell_V2_6`'s coordinate space, alongside `CE_Overhead_Gantry_V1.py`.

## Architecture

- **Rail A** (`Y=-3.8`, "left/front") and **Rail B** (`Y=+3.8`, "right/back") — two independent linear rails, each carrying two robots (near-end and far-end), each rail spanning `X = 0.5` to `X = 10.5`.
- **TABLE_JIG** — a shared steel-frame/wood-deck worktable centered between the rails (`6.80 × 3.20`), with robots from both rails working on it from opposite sides.
- **4 ATC (Automatic Tool Changer) racks** — one at each end of each rail (near+far × A+B), symmetric layout. Each rack: 3 tool tiers × 5 tools per tier = **15 tool positions per rack, 60 total across the system** — real automation-engineering capacity data, not arbitrary.
- Robot arm is modeled here as a simplified 6-axis silhouette (base cylinder + two links), not the full DH chain used in the CR6 pick-and-place files — this file prioritizes structural/visual fidelity of the rail-and-ATC hardware over kinematic accuracy of the arm itself.

## Robot state machine (per robot, explicit in the docstring)

```
PARKED_AT_ATC → TRAVELING_TO_WORK → WORKING →
TRAVELING_TO_ATC → AT_ATC → TOOL_CHANGE → PARKED_AT_ATC
```

Notably, `PARKED_AT_ATC` as the sole named idle/rest state matches `roboticsData.ts`'s own real comment in the live frontend: *"the twin's real robot state machine: PARKED_AT_ATC is the only at-rest state; every other real state string is active motion/work."* This is direct corroborating evidence that this archived file's state model and the live twin's real state model are the same underlying design, not just similarly named by coincidence.

## Work zone split

`WORK_NEAR_X`/`WORK_FAR_X` split the jig into near/far halves at ±25% of jig length from center — A1/B1 work the near half, A2/B2 work the far half, matching the "near/far" naming already built into each robot's identity.

## Notes

- Same "Android · Pydroid 3 · NumPy · Matplotlib" platform note and "headless cycle test before display" validation discipline as `CE_Overhead_Gantry_V1.py` — likely built by the same author, same session/era, same practice.
- `Poly3DCollection` solid-body rendering, consistent with the gantry and integrated-cell files.
