# Engineering Recovery — Index

**Phase 1 of the Robot Library / engineering-recovery workstream.** Read-only reconnaissance and documentation of all 11 unique Python simulation files in this repository's 12 governed directories. No file has been modified, formatted, refactored, converted, renamed, or moved. No rendering migration has been performed. This index and the per-file records it links are the recovered engineering knowledge; the source files themselves are untouched.

**Protected exception**: `CE_Integrated_Cell_V3_0-6.py` — both the archived copy here and the live production copy in `Construction_Enterprises/Chappell_Robotics/` — is documented but was read-only throughout, per [`CE_Integrated_Cell_V3_0-6.md`](CE_Integrated_Cell_V3_0-6.md)'s explicit protection record.

## Summary table

| Simulation | Lines | Engineering systems (observed) | Visualization (observed) | Dependencies (observed) | Notable unknowns / interpretation flags |
|---|---:|---|---|---|---|
| [`CR6_6Axis_V3_1_Corrected`](CR6_6Axis_V3_1_Corrected.md) | 349 | Single CR6 6-DOF arm, analytical FK/IK, static-part pick/place | matplotlib (animated 3D + slider) | numpy, matplotlib | No stated units; "CR6" not confirmed against a real datasheet |
| [`CR6_6Axis_V6_1_Workspace_Guard`](CR6_6Axis_V6_1_Workspace_Guard.md) | 395 | Same arm + moving-conveyor object tracking + reachability-guard safety logic | matplotlib (+ wireframe sphere) | numpy, matplotlib | Units confirmed as meters (axis labels) — the one file that states this |
| [`CR6_V8_0_Dual_Robot_Cell`](CR6_V8_0_Dual_Robot_Cell.md) | 577 | Two CR6 arms, explicit part-ownership state machine, reusable `Robot` class | matplotlib | numpy, matplotlib | Docstring says "V7.5," title/filename say "V8.0" — unresolved |
| [`assembly_cell_v100`](assembly_cell_v100.md) | 1,199 | LGS stud framing + weld cell, zone-based conveyor, part-conservation invariant, 3-DOF planar arms | **pygame** (already migrated off matplotlib once; sim/render architecture explicitly decoupled) | numpy, pygame | Relationship between this cell's simpler arms and the CR6 robots elsewhere not stated |
| [`Dual_Robot_Jig_Frame_V1_1`](Dual_Robot_Jig_Frame_V1_1.md) | 596 (same file, 2 directories) | Real 5-member LGS wall-panel framing + welding sequence, most explicit docstring of the 11 | matplotlib (+ weld sparks) | numpy, matplotlib | Documented V2+ roadmap (rail travel, outfeed, cycle counter) was never implemented |
| [`Factory_Rail_V2`](Factory_Rail_V2.md) | 700 | Rail-mounted robot base, transports finished panel between 2 jigs | matplotlib | numpy, matplotlib | None significant |
| [`CE_Overhead_Gantry_V1`](CE_Overhead_Gantry_V1.md) | 900 | Real 2.0T portal gantry crane, 3-axis (bridge/trolley/hook), full structural anatomy | matplotlib (solid `Poly3DCollection` geometry) | numpy, matplotlib | Shares coordinate space with `CE_Integrated_Cell_V2_6` (confirmed both directions) |
| [`CE_Integrated_Cell_V2_6`](CE_Integrated_Cell_V2_6.md) | 1,165 | 4-station full wall cell (framing/roller/tilt/crane), 4 robots | matplotlib (solid geometry) | numpy, matplotlib | Docstring says tilt goes to 90°; code caps at 60° — resolved by V3_0-6 (60° was correct) |
| [`CE_Module_Assembly_V1`](CE_Module_Assembly_V1.md) | 930 | Same wall cell as V2.6, crane extended to assemble 4 panels into 1 closed module | matplotlib (solid geometry) | numpy, matplotlib | None significant — cleanest direct extension in the set |
| [`CE_Rail_System_V1`](CE_Rail_System_V1.md) | 795 | Dual-rail, 4-robot (A1/A2/B1/B2) system + 4 ATC tool-changer racks (60 tool positions) | matplotlib (solid geometry) | numpy, matplotlib | **Robot names match the live production Digital Twin exactly** — likely primary-source hardware documentation for it |
| [`CE_Integrated_Cell_V1_3`](CE_Integrated_Cell_V1_3.md) | 1,066 | Earliest 3-station version (framing/sheathing/rail-inspect), conveyor transfer, no tilt/crane | matplotlib | numpy, matplotlib | Directory says "V1_3," file docstring says "V1.0" — unresolved |
| [`CE_Integrated_Cell_V3_0-6`](CE_Integrated_Cell_V3_0-6.md) **(protected)** | 1,081 | The culmination: all 4 subsystems integrated, explicit inter-subsystem trigger chain, real rigging-geometry math | matplotlib (solid geometry) | numpy, matplotlib, **os, json** (unique to this file) | 2 of its 4 named subsystem sources (`CE_TableJig_Tilt_V1.py`, `CE_TableJig_Roller_V1.py`) don't exist as separate archived files anywhere in this repo |

**Total: ~10,300 lines of legacy simulation engineering across 12 distinct files** (12 governed directories; `Dual_Robot_Jig_Frame_V1_1` and `Dual_Robot_Cell_Jig_Frame` share identical file content, documented once). 11 of 12 currently render via matplotlib; 1 (`assembly_cell_v100.py`) already migrated to pygame with a renderer-agnostic architecture -- proven in Phase 2 (see `pyvista-pilot/PILOT_RESULT.md`) to survive a full swap to PyVista with zero engineering-logic changes.

## The real lineage (recovered by cross-referencing constants and stated facts across files, not from any single source document)

```
CE_Integrated_Cell_V1_3    (3 stations: framing → sheathing → rail-inspect, conveyor transfer)
        ↓
CE_Integrated_Cell_V2_6    (4 stations: + roller transfer + tilt-to-vertical + crane, coordinate-shares with the standalone Gantry and Rail System files)
        ↓
CE_Module_Assembly_V1      (same cell, crane range extended to assemble 4 panels → 1 closed module)
        ↓
CE_Integrated_Cell_V3_0-6  (protected — culmination: explicit 4-subsystem integration + trigger-chain orchestration; this is the live Digital Twin's lineage)
```

`Dual_Robot_Jig_Frame_V1_1` documents the single-wall-panel framing sequence that all of the above build on top of. `CR6_6Axis_V3_1_Corrected` → `CR6_6Axis_V6_1_Workspace_Guard` → `CR6_V8_0_Dual_Robot_Cell` is a separate, parallel lineage — validating single-robot, then object-tracking, then dual-robot-handoff kinematics/coordination patterns independently of the station-based cell lineage above.

## Cross-references worth carrying forward

1. **`CE_Rail_System_V1.py`'s robot names (A1/A2/B1/B2) match the live production Digital Twin exactly** — this file is likely the closest existing primary-source documentation of the live twin's actual dual-rail/ATC hardware architecture. Worth a dedicated, still read-only comparison pass against the live file in a future session.
2. **`CE_Overhead_Gantry_V1.py` and `CE_Rail_System_V1.py` both explicitly share coordinate space with `CE_Integrated_Cell_V2_6.py`** — real, working precedent for building "smart geometry" as independently-simulatable components that compose into one shared world, which is the direction the Robot Library is meant to grow into.
3. **`CE_Integrated_Cell_V3_0-6.py`'s inter-subsystem trigger model** (Rail → Roller → Tilt → Gantry → next cycle, each a distinct stateful subsystem) is the clearest existing precedent for input/output-driven component composability anywhere in the archived material.

## What Phase 1 deliberately did not do

No matplotlib→pyvista conversion. No refactoring. No file modification of any kind, including the protected file (read only, for cataloging). No frontend/catalog code. Those are later phases, not started here.
