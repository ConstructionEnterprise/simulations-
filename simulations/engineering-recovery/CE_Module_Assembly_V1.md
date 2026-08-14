# CE_Module_Assembly_V1.py — Engineering Recovery

**Source:** `simulations/Module_Assembly_V1/CE_Module_Assembly_V1.py` (930 lines, matplotlib)
**Status:** reference / not-assessed

## Direct successor to CE_Integrated_Cell_V2_6 — confirmed by reused constants

The docstring states this directly: "STAGE 1 — Wall Manufacturing Cell (from V2.5: TABLE_JIG architecture)." Every wall-cell constant (`RACK_*`, `FIXED_*`, `ROLLER_*`, `TILT_*`, `RF1_BASE/RF2_BASE/RS1_BASE/RS2_BASE`, `ASSEMBLY_SEQ`, `FASTEN_PTS`) is copied verbatim, value-for-value, from `CE_Integrated_Cell_V2_6.py`. This file adds **Stage 2**: the crane's range is extended from the tilt-table pickup point (`X=5.5`) out to a new **Module Assembly Jig** at `X=22`, and completed wall panels are delivered there one at a time to build a full four-sided modular unit.

## Real module-assembly sequence (the new content in this file)

The wall-manufacturing cell (Stage 1, unchanged) produces one finished wall panel per cycle. The crane then delivers four sequential panels to specific faces of a `4.0 × 4.0` module footprint centered at `X=22, Y=0`:

```
Panel 1 → NORTH wall (Y+ face, facing inward)
Panel 2 → SOUTH wall (Y− face, facing inward)
Panel 3 → EAST wall  (X+ face, connects N+S)
Panel 4 → WEST wall  (X− face, connects N+S, closes the module)
```

Module height and wall panel height both `4.0` (matches `TILT_W`, the panel's manufactured width, now standing vertical after the Stage-1 tilt). Once all four walls are placed, the crane parks and the whole cycle resets — this is a real, complete "manufacture 4 walls → assemble into a closed box module" process, i.e. the actual panelized/modular building production sequence this whole simulation lineage has been building toward.

## Lineage across the 11 files (recovered by comparing shared constants, not stated as a single roadmap anywhere)

```
Dual_Robot_Jig_Frame_V1_1  (single wall: bottom/3 studs/top, 2 robots)
        ↓
CE_Integrated_Cell_V2_6    (adds roller transfer + tilt-to-vertical + 2 more robots + crane)
        ↓
CE_Module_Assembly_V1      (same cell, crane range extended to assemble 4 panels into one module)
```

`CE_Overhead_Gantry_V1.py` sits alongside this lineage as a detailed standalone model of just the crane subsystem (coordinate-matched to `CE_Integrated_Cell_V2_6`'s tilt-table position, per that file's own recovery record). `CE_Integrated_Cell_V1_3.py` and the protected `CE_Integrated_Cell_V3_0-6.py` (see their own recovery records) appear to be further version steps beyond this point — not fully cross-referenced here; their own records should be read alongside this one for the complete picture.

## Notes

- Same DH robot geometry, same `Poly3DCollection` solid-body rendering approach as `CE_Integrated_Cell_V2_6.py`.
- Crane pickup point explicitly commented `# above tilt table` at `X=5.5`, delivery presumably at `X=22` per the module jig center — confirms this file's crane constants are a strict superset/extension of the V2.6 file's, not an independent redesign.
