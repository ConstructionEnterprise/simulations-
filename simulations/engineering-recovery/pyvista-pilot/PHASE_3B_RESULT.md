# Phase 3B — Remaining Migrations, Result

All 10 remaining simulations migrated to PyVista (the 11th, `assembly_cell_v100`, was Phase 2's pilot; the 12th, `CE_Integrated_Cell_V3_0-6`, remains outside classification and untouched, per its protection record). **Every original source file was read, never modified** -- confirmed via `git status` on all 12 source directories, empty diff.

## 8 Class-A migrations (extract verbatim, add `snapshot()`)

Same pattern Phase 2 proved: the `World`-style class's existing `.step()` already cleanly separates state mutation from drawing, so the migration is extraction (programmatic `sed`, not retyped) plus one small additive method.

| Simulation | Engine/renderer files | Verified by |
|---|---|---|
| `CR6_V8_0_Dual_Robot_Cell` | `cr6_v8_0_engine.py` / `_render.py` | Real part-ownership handoff rendered (Robot A holding part at LIFT) |
| `Dual_Robot_Jig_Frame_V1_1` | `jig_frame_engine.py` / `_render.py` | Real member placement states (HELD/PLACED) rendered |
| `Factory_Rail_V2` | `factory_rail_engine.py` / `_render.py` | Rail travel + frame pickup/return rendered |
| `CE_Overhead_Gantry_V1` | `gantry_engine.py` / `_render.py` | File's own built-in `run_headless_validation()` independently passed at frame 3792 on import -- matches the docstring's own cited validation frame exactly |
| `CE_Integrated_Cell_V2_6` | `v2_6_engine.py` / `_render.py` | 4-station layout with real placed members rendered |
| `CE_Module_Assembly_V1` | `module_assembly_engine.py` / `_render.py` | Real module panel placement (1/4 NORTH) rendered |
| `CE_Rail_System_V1` | `rail_system_engine.py` / `_render.py` | All 4 robots (A1/A2/B1/B2) at correct working positions rendered |
| `CE_Integrated_Cell_V1_3` | `v1_3_engine.py` / `_render.py` | 5 robots including the rail-mounted inspection robot rendered correctly |

## 2 Class-C migrations (real separation introduced, not just extracted)

No `World`-style class existed in either source file -- state was module-level globals mutated directly inside the matplotlib callback (per `PHASE_3A_CLASSIFICATION.md`). These required reimplementing the same computation inside a new engine class, not copying an existing boundary.

**`CR6_6Axis_V3_1_Corrected`** (`cr6_v3_1_engine.py` / `_render.py`): verified two ways beyond "it runs" -- an independent FK(IK(target)) round-trip check (error at machine epsilon, ~4e-16) confirming the kinematics were transcribed correctly, and a full 2000-frame run confirming all 5 documented states were reached with real pick/attach events.

**`CR6_6Axis_V6_1_Workspace_Guard`** (`cr6_v6_1_engine.py` / `_render.py`): same round-trip check passed clean, but the first full-cycle verification failed -- `PLACE` was never reached, the robot looped forever between HOME/PICK_APPROACH/PICK/LIFT while holding the part. Traced to a real transcription error: `STATE_SEQUENCE` was written with 5 entries, dropping the source's trailing `"HOME"` (confirmed 6 entries via `grep` against the actual source file), which broke the `% (len(STATE_SEQUENCE) - 1)` cycling math. Fixed to match the verified source; re-run confirmed 10 complete pick-place cycles over 3000 frames. **This is exactly the kind of error the verification step exists to catch** -- documented in the file's own docstring rather than silently corrected.

## What this establishes

The Phase 3A classification held up under actual execution, not just inspection: all 8 Class-A files migrated with only mechanical, additive changes; both Class-C files genuinely needed new engine code, and one of those two surfaced a real transcription bug that only independent state-machine verification caught -- the same discipline (verify against source, verify against physics, don't assume "it ran" means "it's correct") that made Phase 2's pilot trustworthy carried through all 10 of these.

## Explicitly not done in this phase

No solid-body/mesh geometry beyond simple boxes/spheres/lines for any migration. No interactive controls. No visual comparison against the original matplotlib output (matplotlib itself is available in this environment, unlike pygame, but no original-vs-new pixel diffing was performed -- verification here is state/behavior correctness, same standard as Phase 2). No changes to the Engineering Asset Catalog's `visualization` blocks yet to reflect these migrations (`migrationClass: "A"`/`"C"` entries still describe pre-migration status) -- that update is a natural next step, not done as part of this phase.
