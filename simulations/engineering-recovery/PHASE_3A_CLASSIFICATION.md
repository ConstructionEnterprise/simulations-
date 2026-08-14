# Phase 3A — Migration Classification (no migration performed)

**Question asked**: how much render/engineering separation does each remaining file actually need before it could receive a pyvista renderer the way `assembly_cell_v100.py` did? Answered by direct inspection (class structure + confirming what each animation callback actually does), not assumption.

**Method**: for every file, checked (1) whether simulation state lives in a `World`-style class with its own `.step()` method, or as bare module-level globals; (2) for class-based files, confirmed the matplotlib `update(frame)` callback's actual body — does it call `world.step()` cleanly before drawing, or mutate state directly inside the draw code too.

## Result: the split is better than the 4-tier A/B/C/D framework anticipated

**Class A — has a `World`-style class, `.step()` verified to contain all real state mutation, called cleanly before drawing.** Adding a `snapshot()` method (the same shape as `assembly_cell_v100.py`'s) is a small, additive, mechanical change — not an architectural separation. This is the same migration pattern the pilot already proved, not a new one:

| File | Classes present |
|---|---|
| `CR6_V8_0_Dual_Robot_Cell.py` | `Robot`, `Part`, `World` |
| `Dual_Robot_Jig_Frame_V1_1.py` | `Robot`, `Member`, `World` |
| `Factory_Rail_V2.py` | `FactoryRail`, `CR6`, `WallFrame`, `World` |
| `CE_Overhead_Gantry_V1.py` | `OverheadGantry`, `World` |
| `CE_Integrated_Cell_V2_6.py` | `Robot`, `OverheadCrane`, `CellState`, `World` |
| `CE_Module_Assembly_V1.py` | `Robot`, `OverheadCrane`, `WallCellState`, `ModuleState`, `World` |
| `CE_Rail_System_V1.py` | `CR6Robot`, `RailWorld` |
| `CE_Integrated_Cell_V1_3.py` | `Robot`, `FactoryRail`, `CellState`, `World` |

Directly verified for `CE_Rail_System_V1.py` and `CE_Overhead_Gantry_V1.py` (representative samples): `update(frame_num)` is exactly `ax.clear() → world.step() → <draw calls>`. No state mutation is mixed into the drawing section. The other six were classified by the same class-structure evidence but not individually line-verified this pass — flagged as inference from strong structural evidence, not independently confirmed for each one.

**Class C — no classes at all; state is module-level globals, mutated directly inside the single `update(frame)` function alongside the drawing calls.** These two genuinely need a real state/render separation introduced before any renderer swap is possible — not a mechanical addition:

| File | Module-level state vars |
|---|---|
| `CR6_6Axis_V3_1_Corrected.py` | 14 (`part_position`, `is_attached`, `dwell_frames`, `seg_idx`, `t_seg`, `current_q`, etc.) |
| `CR6_6Axis_V6_1_Workspace_Guard.py.py` | 14 (adds `parts_cleared`, `waiting_for_home_reset`) |

Notably, these are the two *simplest* simulations by engineering scope (single robot, single part) but the *least* separated by code structure — scope and migration difficulty aren't correlated here.

**Class D — outside classification, per standing instruction**: `CE_Integrated_Cell_V3_0-6.py` (protected/immutable, both copies).

**Already done**: `assembly_cell_v100.py` — piloted in Phase 2, proven successful.

## What this means for sequencing

8 of the 10 remaining files are much closer to "ready" than the a-priori Class A/B/C/D framework assumed — none of them need the kind of careful architectural surgery that framework's "Class C: interleaved renderer/engineering → careful architectural separation" category was written to describe. Only the two earliest CR6 files fit that description. If/when renderer migration resumes, the 8 class-based files are a natural single batch (same pattern, low individual risk each); the 2 global-state files are a genuinely different, smaller, separate task.

No files were modified in this classification pass.
