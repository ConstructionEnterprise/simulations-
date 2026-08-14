# CE_Integrated_Cell_V3_0-6.py — Engineering Recovery (content)

**This is the engineering-content record. See `CE_Integrated_Cell_V3_0-6.md`'s companion protection record (same filename, written first) for the immutability rules — both copies (archived and live) are strictly read-only for this and all future work.** This record was produced by reading the **archived reference copy** (`simulations-/simulations/CE_Intergrated_Cell_V1_3/CE_Integrated_Cell_V3_0-6.py`), which the protection record already established has a different hash than, and is not identical to, the live production twin. Content here describes the archived snapshot; it is not a guarantee of the live file's current exact behavior.

## The culmination of the whole simulation lineage — confirmed by the file's own docstring

"**Complete digital twin combining all 4 validated subsystems**. Reference: Chappell Robotics integrated cell render (June 2026)." This resolves the version lineage inferred across the other recovery records into a stated fact from the source itself:

```
1. CR6 Rail System   — 4 robots assemble LGS panel on TABLE_JIG_FIXED
2. Roller Transfer   — Panel moves FIXED → TILT
3. Tilt Table        — Panel raised 0° → 60° for gantry pickup
4. Overhead Gantry   — 2.0T crane lifts panel, delivers to MODULE_JIG
```

**Explicitly named subsystem sources**, with development phase and validation frame number for each:
- `CE_Overhead_Gantry_V1.py` — Phase 1 (frame 3792)
- `CE_Rail_System_V1.py` — Phase 2 (frame 641)
- `CE_TableJig_Tilt_V1.py` — Phase 3 (frame 792)
- `CE_TableJig_Roller_V1.py` — Phase 4 (frame 1037)

**Gap worth flagging**: the first two subsystem files are present as their own archived directories in this repository (`CE_Overhead_Gantry_V1.py` → `Overhead_Gantry_V1/`, `CE_Rail_System_V1.py` → `Rail_System_V1/`, both separately documented in their own recovery records). **`CE_TableJig_Tilt_V1.py` and `CE_TableJig_Roller_V1.py` do not exist anywhere in the 12 archived directories** — their logic only survives embedded inside this integrated file. If either was ever a standalone source file, it was not preserved in this consolidation; if it was only ever a design label rather than a real separate file, that isn't determinable from what's archived here either. Not resolved — recorded as an open gap.

## Resolves an earlier discrepancy

`CE_Integrated_Cell_V2_6.py`'s docstring claimed the tilt table goes "0° → 90°," but its actual code capped at 60°. This file's docstring states the tilt explicitly and consistently as **"0°→60°"** and defines `MAX_TILT = 60.0` directly as a named constant — confirming 60° was the real, deliberate, final design value, and the V2.6 docstring's "90°" was simply an outdated/incorrect description carried over from an earlier design intent, not a bug in either file's actual behavior.

## World coordinate space (final, integrated version — larger scale than earlier files)

```
TABLE_JIG_FIXED   X=8.3   (was -6.0 in V2.6 — whole layout shifted/rescaled)
TABLE_JIG_ROLLER  X=14.9
TABLE_JIG_TILT    X=21.5  (PIVOT_X=24.8)
MODULE_JIG        X=32.0
Gantry park       X=14.0 area / runway X=1.0–36.0
Runway rails      Y=±4.5      CR6 rails  Y=±1.55
```

This is a real rescale/relayout relative to `CE_Integrated_Cell_V2_6.py` and `CE_Module_Assembly_V1.py` (which used `X=-10` to `X=+22`) — same conceptual stations, larger and shifted coordinate space, consistent with the docstring's own note: "Jig assembly shifted +5 units toward MODULE_JIG."

## ★ Inter-subsystem trigger model — the real precedent for "smart geometry" composability

The docstring states an explicit handoff chain between subsystems, each one a distinct class/state machine that triggers the next on a physical completion event, not on a shared global timer:

```
Rail WORKING complete (cycles≥1)  → Roller RECEIVING
Roller DELIVERED                  → Tilt TILTING_UP
Tilt HELD_AT_60                   → Gantry activate (TRAVELING_X)
Gantry PARKED (cycle complete)    → Rail next cycle
```

This is the clearest existing precedent anywhere in the archived material for the "smart geometry with input/output relationships connecting to adjacent geometry" composability concept — the live/final twin design already treats Rail, Roller, Tilt, and Gantry as separate, individually-stateful subsystems wired together by explicit trigger conditions, not as one monolithic script. A future component-based Robot Library architecture would be formalizing and generalizing a pattern that this file's own design already establishes, not inventing something unrelated to how the real system is actually built.

## Real hooked-panel rigging geometry (worked example of real engineering math present in the comments)

The file works out, in comments, the exact hook-attachment point on the tilted panel: free end position at 60° tilt (`X = PIVOT_X - TILT_W·cos(60°)`, `Z = PIVOT_Z + TILT_W·sin(60°)`), then the actual rigging point at 70% up the panel's leaf length plus a thickness/rigging allowance, arriving at `HOOK_LOWER_Z = 5.25`. This is real trigonometric engineering work, shown and preserved in the comments, not just a hardcoded magic number.

## Notes

- Uses the CE aesthetic color palette (gold/amber/gray) consistent across the gantry, rail, and integrated-cell files — a real, consistent visual identity system used across the whole lineage, not per-file arbitrary colors.
- `import os, json` appear at the top (unlike any other file in the set) — suggests this version does some file I/O (config loading, state persistence, or logging) that the earlier, simpler files don't. Not investigated further here — reading past line 150 of this specific file was intentionally limited given its protected status; this note flags it as something worth a closer, still read-only look in a future pass rather than something resolved now.
