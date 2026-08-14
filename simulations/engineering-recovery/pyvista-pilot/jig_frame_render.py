"""PyVista renderer for Dual_Robot_Jig_Frame_V1_1 (Phase 3B, Class A migration)."""
import sys
import pyvista as pv

from jig_frame_engine import World
from pv_toolkit import new_offscreen_plotter, robot_arm_mesh, run_and_snapshot

MEMBER_COLOR = {"IN_RACK": "gray", "HELD": "tomato", "PLACED": "gold", "WELDED": "#22CC66"}


def build_scene(plotter, snap):
    plotter.clear()
    plotter.add_mesh(robot_arm_mesh(snap["cr6_1"]["pts"]), color="#FF3333", line_width=8)
    plotter.add_mesh(robot_arm_mesh(snap["cr6_2"]["pts"]), color="#CC44FF", line_width=8)
    for m in snap["members"]:
        plotter.add_mesh(pv.Cube(center=m["pos"], x_length=0.35, y_length=0.35, z_length=0.15),
                          color=MEMBER_COLOR.get(m["status"], "white"))
    plotter.add_text(
        f"frame {snap['frame']}  panels {snap['panels_done']}  "
        f"CR6-1:{snap['cr6_1']['state']}  CR6-2:{snap['cr6_2']['state']}",
        position="upper_left", font_size=10, color="black",
    )


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = World()
    plotter = new_offscreen_plotter(camera_position=[(8, -8, 6), (0, 0, 1), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=800, snapshot_every=200, out_dir=out, label="jig_",
                      step_fn=lambda e: e.step(0.02))
