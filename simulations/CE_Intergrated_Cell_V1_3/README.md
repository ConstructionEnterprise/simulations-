# CE Integrated Cell — Controllable Digital Twin

**Canonical twin:** `CE_Integrated_Cell_V3_0-6.py`

The integrated manufacturing cell digital twin. Combines four validated
subsystems (CR6 Rail System, Roller Transfer, Tilt Table, 2.0T Overhead
Gantry) under a master phase tracker. It is controllable via a file-based
contract: it writes status to `state.json` and obeys commands from
`command_queue.json`, with the validated simulation logic untouched.

---

## Current State: Pass 1 Complete

The rail has been promoted to a first-class `RailSubsystem` object.
`cell.rail_A` and `cell.rail_B` are now real named objects, consistent with
`cell.gantry`, `cell.roller`, and `cell.tilt`.

**Validation: identical to baseline** — frame 1395, all four cycle counts
(gantry, roller, tilt, walls) = 1. Behavior unchanged; this was a refactor of
where data lives, not how the cell behaves.

---

## Checkpoints in Commit History

Because every upload shares the same filename, use the **commit history** to
identify and restore versions:

| Commit  | Date   | State |
|---------|--------|-------|
| `6295dda` | Jun 30 | **Pass 1 complete — rail as real object (CURRENT)** |
| `6d9a9b4` | Jun 29 | Milestone #1 complete — controllable twin, PRE-rail restructuring |
| `b278eae` | Jun 7  | Earlier V1_3 file |

To restore an earlier version: open the commit, view the file at that commit,
and download it. The current file is always recoverable to any prior checkpoint.

---

## Next Steps

1. **Targeting layer** — wire `rail_A`/`rail_B` and the other subsystems into
   the command contract with a `target` field. Line-stop semantics (pausing any
   subsystem holds the whole line). Add `paused_by` / `paused_at` reporting to
   `state.json`.
2. **Pass 2 — ATC + tools** — promote the ATC to a first-class subsystem:
   four instances (near/far on each rail), each owning its tools (gripper,
   screwdriver, suction; welder is the default-fitted tool on the CR6 arm).

---

## Do Not Modify

The validated simulation logic — `IntegratedCell`, `cell.step()`, and the
state-machine classes (`OverheadGantry`, `RollerTable`, `TiltTable`,
`CR6Robot`) — is do-not-modify. All extensions wrap it; they never alter it.
Any change must keep the validation gate passing identically (frame 1395,
all cycles = 1).
