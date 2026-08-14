"""
PyVista renderer for assembly_cell_v100's SimulationEngine.

Migration pilot for the §31 engineering-recovery workstream: proves the
simulation/engineering model survives separation from its original renderer
(pygame) by driving an entirely different rendering backend (PyVista) off
the exact same `snapshot()` contract the original Renderer class consumes.
No simulation logic lives here -- this file only draws what `snapshot()`
reports. `simulation_engine.py` is a byte-for-byte extraction of the
original file's SimulationEngine class and its supporting constants
(programmatic sed extraction, not retyped) -- the source file itself,
`Dual_CR6_Cell/assembly_cell_v100.py`, was never modified.
"""

import numpy as np
import pyvista as pv

from simulation_engine import (
    SimulationEngine,
    L1, L2,
    STUD_H,
    TABLE_D, TABLE_THICK, TABLE_W, TABLE_X, TABLE_Y,
    ZONES,
)

STUD_COLOR = {"A": "#4488FF", "B": "#FF8844"}
FRAME_COLOR = "#22CC66"
ROBOT_COLOR = ["#FF3333", "#FFD700"]


def part_box(p):
    """A stud/track part as an axis-aligned box, matching the sim's own w/d/h fields."""
    return pv.Box(bounds=(
        p["x"] - p["w"] / 2, p["x"] + p["w"] / 2,
        p["y"] - p["d"] / 2, p["y"] + p["d"] / 2,
        p["z"] - p["h"] / 2, p["z"] + p["h"] / 2,
    ))


def robot_arm_lines(snap_robot):
    """Base -> elbow -> TCP as a 2-segment polyline, matching fk3()'s own 3 joint points."""
    base, j2, tcp = snap_robot["base"], snap_robot["j2"], snap_robot["tcp"]
    return pv.lines_from_points(np.array([base, j2, tcp]))


def build_scene(plotter, snap):
    plotter.clear()

    # Fixture table
    plotter.add_mesh(
        pv.Box(bounds=(
            TABLE_X - TABLE_W / 2, TABLE_X + TABLE_W / 2,
            TABLE_Y - TABLE_D / 2, TABLE_Y + TABLE_D / 2,
            0.0, TABLE_THICK,
        )),
        color="#999999", opacity=0.6,
    )

    # Conveyor zone markers (matches the sim's own ZONES constant)
    for zx in ZONES:
        plotter.add_mesh(pv.Line((zx, -1.5, 0), (zx, 1.5, 0)), color="#555555", line_width=2)

    # Robots
    for i, key in enumerate(("r1", "r2")):
        r = snap[key]
        plotter.add_mesh(robot_arm_lines(r), color=ROBOT_COLOR[i], line_width=10, label=f"Robot {key.upper()} ({r['state']})")
        plotter.add_mesh(pv.Sphere(radius=0.15, center=r["tcp"]), color=ROBOT_COLOR[i])
        if r["held"]:
            plotter.add_mesh(part_box(r["held"]), color=STUD_COLOR.get(r["held"]["type"], "white"))

    # Parts in flight
    for p in snap["conveyor_parts"]:
        plotter.add_mesh(part_box(p), color=STUD_COLOR.get(p["type"], "white"))
    for p in snap["frame_parts"]:
        plotter.add_mesh(part_box(p), color=FRAME_COLOR)

    # Weld sparks
    for s in snap["spark_frames"]:
        plotter.add_mesh(pv.Sphere(radius=0.03, center=(s["x"], s["y"], s["z"])), color="yellow")

    plotter.add_text(
        f"frame {snap['frame_num']}  |  produced {snap['produced_count']}  |  "
        f"R1 {snap['r1']['state']}  |  R2 {snap['r2']['state']}",
        position="upper_left", font_size=10, color="black",
    )


def run_pilot(n_frames: int, snapshot_every: int, out_dir: str):
    engine = SimulationEngine()
    plotter = pv.Plotter(off_screen=True, window_size=(1000, 700))
    plotter.set_background("white")
    plotter.camera_position = [(30, -25, 25), (9, 0, 2), (0, 0, 1)]

    saved = []
    for frame in range(1, n_frames + 1):
        engine.step()
        if frame % snapshot_every == 0 or frame == n_frames:
            snap = engine.snapshot()
            build_scene(plotter, snap)
            path = f"{out_dir}/frame_{frame:04d}.png"
            plotter.screenshot(path)
            saved.append((frame, path, snap["produced_count"], snap["r1"]["state"], snap["r2"]["state"]))
            print(f"frame {frame:4d}  produced={snap['produced_count']}  r1={snap['r1']['state']:14s} r2={snap['r2']['state']}")

    plotter.close()
    return saved


if __name__ == "__main__":
    import sys
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    run_pilot(n_frames=600, snapshot_every=100, out_dir=out)
