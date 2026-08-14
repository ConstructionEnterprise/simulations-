# CE_Integrated_Cell_V1_3.py — Engineering Recovery

**Source:** `simulations/CE_Intergrated_Cell_V1_3/CE_Integrated_Cell_V1_3.py` (1,066 lines, matplotlib)
**Status:** reference / not-assessed
**Naming note**: the containing directory is named `V1_3`, but the file's own docstring header says `V1.0` — an unresolved inconsistency in the source itself, reproduced as observed, not corrected. (This directory also holds the separately-protected `CE_Integrated_Cell_V3_0-6.py` — two different version labels coexist in one directory; see that file's own recovery record.)

## Earliest integrated-cell version in this lineage — predecessor to CE_Integrated_Cell_V2_6

Comparing constants and process flow directly against `CE_Integrated_Cell_V2_6.py` and `CE_Module_Assembly_V1.py` (see their own recovery records), this file is an earlier design:

- **Three stations, not four**: Framing (`X=-6`) → Sheathing (`X=+5.5`) → Factory Rail inspection (no dedicated tilt table, no crane).
- **Transfer is a straight conveyor** (`CONV_X_START=-4.5` to `CONV_X_END=3.5`), not the later roller-index + tilt-table mechanism.
- **No overhead crane** — instead, a rail-mounted single robot performs an inspection sweep (`INSPECT_PASSES`, 3 fixed viewpoints over the sheathing table) and the wall exits directly via outfeed/stacking, rather than being lifted and delivered to a module jig.
- Same DH robot geometry as every other CR6-lineage file.

## Process flow (from the docstring, matches the code)

```
Raw studs (rack, X=-10) → CR6-F1 places on framing table (X=-6) → CR6-F2 fastens
    → conveyor → CR6-S1 places sheathing sheet (from magazine, X=3.0) on sheathing table (X=5.5)
    → CR6-S2 fastens sheathing → Factory Rail robot inspects (3-pass sweep)
    → finished wall exits via outfeed (X=+12)
```

## Part-ownership state chains (explicit in the docstring, four separate chains for four sub-processes)

```
FRAMING:   STUD_ON_RACK → STUD_HELD_F1 → STUD_ON_TABLE → FRAME_ASSEMBLING → FRAME_DONE
TRANSFER:  FRAME_DONE → FRAME_CONVEYING → FRAME_AT_SHEATHING
SHEATHING: SHEET_ON_MAG → SHEET_HELD_S1 → SHEET_ON_FRAME → WALL_FASTENING → WALL_DONE
OUTFEED:   WALL_DONE → RAIL_INSPECTING → WALL_COMPLETE
```

This is the most granular ownership-chain documentation of any file in the set — explicitly modeling each sub-stage as its own tracked state rather than one flat robot-state enum.

## Lineage position (updates the map from `CE_Module_Assembly_V1.md`)

```
CE_Integrated_Cell_V1_3   (3 stations, conveyor transfer, rail-inspection outfeed — this file)
        ↓
CE_Integrated_Cell_V2_6   (4 stations, roller transfer, tilt-to-vertical, crane outfeed)
        ↓
CE_Module_Assembly_V1     (same as V2.6, crane extended to assemble 4 panels into 1 module)
        ↓
CE_Integrated_Cell_V3_0-6 (live, protected — see its own recovery record; not diffed against this file)
```

This ordering is inferred from comparing each file's own stated constants and process complexity, not from any single document that states the roadmap explicitly — flagged as interpretation, not a confirmed fact from the source material itself.
