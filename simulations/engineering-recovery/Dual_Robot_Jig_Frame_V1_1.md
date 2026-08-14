# Dual_Robot_Jig_Frame_V1_1.py — Engineering Recovery

**Source:** appears identically in both `simulations/Dual_Robot_Jig_Frame_V1_1/` and `simulations/Dual_Robot_Cell_Jig_Frame/` (596 lines each, matplotlib) — the same file content lives under two different governed directory names in the consolidated repo. Documented once here; both directories' `INDEX.md` entries point back to this record.
**Status:** reference / not-assessed (both directories)

## What it simulates: LGS wall-frame assembly, real and specific (this file has the clearest docstring of all 11)

Two CR6 robots (same DH geometry as the other CR6 files — `D1=1.5, A2=2.5, A3=2.0, D6=0.5`, `MAX_REACH=5.0`, here run at a tighter `SAFE_REACH = MAX_REACH*0.97`) with explicitly different roles, not identical twins:
- **CR6-1** (base `[0, -2.8, 0]`) — material handler, picks studs/tracks from a rack and places them on the jig.
- **CR6-2** (base `[0, +2.8, 0]`) — welder, welds every member at all its joint intersections once CR6-1 has placed it.

**Wall panel definition (V1), explicit in the docstring**: a standard 5-member stick-framed wall section —
- `bottom_track` placed first at `Y = -0.8`
- 3 studs at `X = -1.0, 0.0, +1.0` (left/center/right), each `STUD_H = 1.6` long
- `top_track` placed last at `Y = +0.8`
- Assembly order: `bottom_track → stud_L → stud_C → stud_R → top_track`, welded by CR6-2 immediately after each placement — this is a real, standard light-gauge-steel wall-panel framing sequence.

**Fixture/rack geometry**: table jig `X±1.5, Y±1.2` at `Z=0.8`; material rack at `[-3.5, -2.8]`, 5 stacked levels (`RACK_LEVELS`, `0.15` spacing) for holding stock members before pick.

**Documented future roadmap (from the file's own docstring, not yet implemented — recorded as stated intent, not fact)**: V2+ was planned to add a 5-stud panel at 24" on-center spacing via robot rail travel, an automated outfeed conveyor, and a production counter with cycle-time tracking. None of that exists in this file as written — it's the original author's own stated next-steps.

## Architecture

Same `Robot`/`World` class pattern as `CR6_V8_0_Dual_Robot_Cell.py`, plus a `Member` class specific to this cell: each wall-panel member (track/stud) carries a `status` field with lifecycle `IN_RACK → HELD → PLACED → WELDED`. `build_weld_sequence()` generates paired `WELD_APPROACH_i`/`WELD_i` waypoints per joint intersection. Visualization includes weld-spark particle effects (`draw_sparks`, `SPARK_LIFE=18` frames) and a live progress HUD tracking placed/welded counts against total member count.

## Notes

- This is the most explicitly-documented file of the 11 in terms of stated engineering intent (the docstring reads like a real design note, not just code comments).
- Units unstated, consistent with the other CR6-lineage files.
