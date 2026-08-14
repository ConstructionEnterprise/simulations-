"""PyVista renderer for CR6_V8_0_Dual_Robot_Cell (Phase 3B, Class A migration)."""
import sys
import pyvista as pv

from cr6_v8_0_engine import World
from pv_toolkit import new_offscreen_plotter, robot_arm_mesh, run_and_snapshot

ROBOT_COLOR = {"A": "#FF3333", "B": "#CC00FF"}
PART_COLOR = {
    "ON_CONVEYOR": "darkorange", "COMMITTED_TO_A": "darkorange",
    "HELD_BY_A": "tomato", "IN_FIXTURE": "gold", "HELD_BY_B": "cyan", "COMPLETE": "lime",
}


def build_scene(plotter, snap):
    plotter.clear()
    for key in ("ra", "rb"):
        r = snap[key]
        plotter.add_mesh(robot_arm_mesh(r["joints"]), color=ROBOT_COLOR["A" if key == "ra" else "B"], line_width=8)
        plotter.add_mesh(pv.Sphere(radius=0.15, center=r["joints"][-1]), color="black")
    p = snap["part"]
    plotter.add_mesh(pv.Cube(center=p["pos"], x_length=0.3, y_length=0.3, z_length=0.2), color=PART_COLOR.get(p["ownership"], "white"))
    plotter.add_text(f"frame {snap['frame']}  cleared {snap['cleared']}  A:{snap['ra']['state']}  B:{snap['rb']['state']}  part:{p['ownership']}",
                      position="upper_left", font_size=10, color="black")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = World()
    plotter = new_offscreen_plotter(camera_position=[(15, -10, 10), (0, 1.75, 1), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=500, snapshot_every=100, out_dir=out, label="cr6v80_",
                      step_fn=lambda e: e.step(0.018))
