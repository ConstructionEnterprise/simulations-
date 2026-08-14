"""PyVista renderer for Factory_Rail_V2 (Phase 3B, Class A migration)."""
import sys
import pyvista as pv

from factory_rail_engine import World
from pv_toolkit import new_offscreen_plotter, robot_arm_mesh, run_and_snapshot

FRAME_COLOR = {"ON_JIG2": "gold", "HELD": "tomato", "ON_JIG1": "#22CC66"}


def build_scene(plotter, snap):
    plotter.clear()
    plotter.add_mesh(pv.Line((0, 0, 0), (8, 0, 0)), color="gray", line_width=3)
    plotter.add_mesh(robot_arm_mesh(snap["robot"]["pts"]), color="#FF3333", line_width=8)
    f = snap["frame"]
    plotter.add_mesh(pv.Cube(center=f["pos"], x_length=1.2, y_length=0.8, z_length=0.1),
                      color=FRAME_COLOR.get(f["ownership"], "white"))
    plotter.add_text(
        f"tick {snap['tick']}  cycles {snap['cycles']}  state:{snap['sim_state']}  rail_x:{snap['rail_x']:.2f}",
        position="upper_left", font_size=10, color="black",
    )


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = World()
    plotter = new_offscreen_plotter(camera_position=[(12, -10, 8), (4, 1.5, 1), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=800, snapshot_every=200, out_dir=out, label="rail_",
                      step_fn=lambda e: e.step(0.02))
