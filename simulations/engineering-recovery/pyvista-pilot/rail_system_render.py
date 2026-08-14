"""PyVista renderer for CE_Rail_System_V1 (Phase 3B, Class A migration).

Special note: this simulation's robot identities (A1/A2/B1/B2) match the
live production Digital Twin exactly -- see the recovery record's
"HIGH correlation" flag. Rendering it is not a claim of equivalence to the
live twin, only a visualization of this archived file's own real behavior.
"""
import sys
import pyvista as pv

from rail_system_engine import RailWorld
from pv_toolkit import new_offscreen_plotter, run_and_snapshot

ROBOT_COLOR = {"A1": "#FF3333", "A2": "#FF8800", "B1": "#00CCFF", "B2": "#4466FF"}


def build_scene(plotter, snap):
    plotter.clear()
    plotter.add_mesh(pv.Line((0.5, -3.8, 0), (10.5, -3.8, 0)), color="gray", line_width=3)
    plotter.add_mesh(pv.Line((0.5, 3.8, 0), (10.5, 3.8, 0)), color="gray", line_width=3)
    plotter.add_mesh(pv.Box(bounds=(2.1, 8.9, -1.6, 1.6, 0.85, 0.90)), color="#D4A96A", opacity=0.6)  # table jig
    for name, r in snap["robots"].items():
        plotter.add_mesh(pv.Sphere(radius=0.2, center=(r["x"], r["rail_y"], 0.4)), color=ROBOT_COLOR[name])
        plotter.add_mesh(pv.Line((r["x"], r["rail_y"], 0.4), tuple(r["tip"])), color=ROBOT_COLOR[name], line_width=5)
    status = "  ".join(f"{n}:{r['state']}" for n, r in snap["robots"].items())
    plotter.add_text(f"tick {snap['tick']}  {status}", position="upper_left", font_size=9, color="black")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = RailWorld()
    plotter = new_offscreen_plotter(camera_position=[(5.5, -14, 8), (5.5, 0, 0.5), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=600, snapshot_every=150, out_dir=out, label="railsys_")
