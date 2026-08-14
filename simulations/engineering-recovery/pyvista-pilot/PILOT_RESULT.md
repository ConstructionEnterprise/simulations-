# PyVista Migration Pilot — Result

**Target:** `assembly_cell_v100.py` (`Dual_CR6_Cell/`), chosen because Phase 1 found it already had a renderer-agnostic `SimulationEngine` / `Camera` / `Renderer` / `HUD` split, with pygame as its current renderer.

**Success criterion (per plan): "the engineering model survives separation from its original renderer" — not merely "PyVista renders something."**

## What was done

1. **Source preserved exactly.** `Dual_CR6_Cell/assembly_cell_v100.py` was never opened for writing — confirmed via `git status` (empty diff) both before and after this pilot.
2. **`simulation_engine.py`** — a programmatic `sed` extraction (not retyped) of the original file's docstring, imports (minus the `try/except import pygame` guard block, which is the only thing excluded), constants, and the complete `SimulationEngine` class including its `snapshot()` method. Byte-for-byte from the source for everything it contains.
3. **`pyvista_renderer.py`** — new code, written for this pilot. Contains zero simulation logic; every visual element is driven by reading `engine.snapshot()`, the same read-only contract the original pygame `Renderer` class consumes.

## Verification

- `simulation_engine.py` imports and runs standalone with **zero pygame dependency** — proves the engineering logic (kinematics, part-conservation tracking, state machines, weld sequencing) has no actual coupling to its original renderer, exactly as Phase 1's recovery record predicted from reading the source.
- Ran 600 simulation steps. Real, correct behavior observed via `snapshot()` at intervals: robot states cycling through the documented state machine (`PICKING_A/B → PRE_PLACE → PLACING_A/B → PICKING → PLACING → WELDING → FRAME_COMPLETE`), stud parts moving along conveyor zones, a real assembled frame corner (track + stud at a right angle, matching `ASSEMBLY_SEQ`) appearing on the fixture table by frame 600.
- Rendered via PyVista, off-screen, to PNG snapshots at 100-frame intervals — visually confirmed the geometry is structurally sensible and evolves correctly frame to frame (parts appear, move, and get placed; robot arms track their targets; no static/frozen/broken rendering).

## What this establishes

The migration pattern that worked: **extract/reuse the engineering class unchanged, write a new renderer against its existing `snapshot()` contract.** No engineering logic needed to change at all for this file, because it was already structured for exactly this separation. This is not necessarily the pattern for the other 10 files — none of them have a `snapshot()`-style renderer boundary; their rendering and simulation logic are interleaved in the `update()`/animation-callback function itself. Porting any of those would require first *introducing* a clean state/render separation (a real, non-trivial step this file didn't need), not just swapping the rendering backend.

## Explicitly not done in this pilot

- No solid-body/mesh geometry (boxes were kept simple; no `Poly3DCollection`-equivalent detailed structural rendering).
- No interactive camera controls, HUD styling, or spark-particle visual fidelity matching the original pygame version.
- No comparison run of the original pygame version side-by-side (pygame failed to install in this environment — no prebuilt wheel yet for the installed Python 3.14; not investigated further, noted as an environment gap).
- No changes to any of the other 10 simulation files.

## Files in this directory

- `simulation_engine.py` — extracted engineering logic (see provenance note in its own docstring)
- `pyvista_renderer.py` — new PyVista renderer against `snapshot()`
- `PILOT_RESULT.md` — this file
