# CE_Overhead_Gantry_V1.py — Engineering Recovery

**Source:** `simulations/Overhead_Gantry_V1/CE_Overhead_Gantry_V1.py` (900 lines, matplotlib)
**Status:** reference / not-assessed

## Real, specific equipment — not a generic crane

The most detailed, professionally-documented file of the 11. Docstring explicitly states this is a digital twin of a real physical machine: **"Reference: Chappell Robotics 2.0T portal gantry render (June 2026)"** — a real 2-ton-capacity overhead bridge/portal gantry crane, not an invented prop. Every geometric constant is individually commented with its physical justification (e.g. `BRIDGE_BEAM_Z = 5.8 # bottom flange of bridge beam (hook hang height)`).

## ★ Important cross-file relationship — this simulation shares a coordinate space with CE_Integrated_Cell_V2_6

The docstring states directly: **"WORLD LAYOUT (matches CE_Integrated_Cell_V2.6 coordinate space)"** — pickup zone at `X=5.5` is explicitly noted as matching that other file's `TILT_CX` (tilt-table center-X), and delivery zone at `X=22.0` matches its `MOD_CX` (module-jig center-X). **This means the 11 archived simulations are not all independent, isolated experiments** — at least this pair was deliberately designed to represent two different subsystems of one shared factory layout, coordinated by a common coordinate system, even though they exist as separate files/simulations. This is directly relevant to the "smart geometry with input/output connections to adjacent geometry" composability goal — this gantry-to-cell relationship is the clearest existing precedent for it anywhere in the 11 files. Should be cross-checked against `CE_Integrated_Cell_V2_6.py` directly (see that file's own recovery record) to confirm the coordinate match from both sides.

## Structural architecture (real overhead-crane anatomy, from the docstring)

- **End trucks** — two portal-frame columns (dark grey box-section, `COL_W=0.40 × COL_D=0.50`), fixed in Y, travel in X along floor-embedded runway rails (`RUNWAY_X_MIN=2.0` to `RUNWAY_X_MAX=26.0`).
- **Bridge beam** — CE-gold I-beam spanning the Y-axis between end trucks, with a catwalk (`CATWALK_H=0.20`) and safety railing (`RAIL_H=0.18`) on top; travels with the end trucks in X.
- **Trolley/hoist** — black hoist unit riding the bridge beam's bottom flange, travels independently in Y along the beam (`TROLLEY_SPEED=0.06`/frame).
- **Hook block** — wire rope + hook hanging from the hoist, travels in Z (`HOOK_SPEED=0.045`/frame; `HOOK_PARK_Z`, `HOOK_LOWER_Z=1.10` for pickup, `HOOK_DELIVER_Z=1.20` for delivery — deliberately different heights for the two zones).

**Three independent axes**: X = bridge travel, Y = trolley travel, Z = hoist travel — a true 3-axis overhead crane kinematic model, structurally distinct from every robot-arm file in this set (no joint angles/DH chain at all — this is a gantry, not an articulated arm).

## Process state machine (12 states, explicit in the docstring)

```
PARKED → TRAVELING_X → TRAVELING_Y → LOWERING → HOOKED →
LIFTING → TRAVELING_DELIVER_X → TRAVELING_DELIVER_Y →
LOWERING_DELIVER → PLACING → RISING → RETURNING_Y →
RETURNING_X → PARKED
```

Pickup zone `X=5.5` (above a "TABLE_JIG_TILT"), delivery zone `X=22.0` (above a "MODULE_JIG"), park position `X=14.0` (mid-span, deliberately clear of both zones so the parked gantry doesn't obstruct either station).

## Safety envelope

Explicit safe-travel bounds inset from the physical runway limits: `SAFE_X_MIN/MAX` = runway limits ±0.5, `SAFE_Y_MIN/MAX` = runway rail positions ±0.8 — a real operational safety margin distinct from the hard mechanical limits.

## Notes

- Platform note in the docstring: "Android · Pydroid 3 · Python · NumPy · Matplotlib" — this was developed/run on a mobile device (Pydroid 3, an Android Python IDE), and explicitly "VALIDATED — Headless cycle test before display code" — meaning the author ran the simulation logic without a display first to validate the cycle before adding matplotlib rendering. A real, disciplined development practice worth preserving as a fact, not just code trivia.
- Uses `Poly3DCollection` (solid 3D polygon faces), not just line/scatter primitives like the CR6 files — this file renders actual solid box/beam geometry, not wireframe stick figures. A pyvista port has real solid-mesh geometry to work from here, more directly translatable than the line-based robot-arm renders.
