"""PyVista renderer for CR6_6Axis_V3_1_Corrected (Phase 3B, Class C migration)."""
import sys
import pyvista as pv

from cr6_v3_1_engine import CR6Engine
from pv_toolkit import new_offscreen_plotter, robot_arm_mesh, run_and_snapshot


def build_scene(plotter, snap):
    plotter.clear()
    plotter.add_mesh(pv.Box(bounds=(-4, 4, -3.3, -2.3, 0.98, 1.02)), color="steelblue", opacity=0.4)  # conveyor
    plotter.add_mesh(robot_arm_mesh(snap["pts"]), color="#FF3333", line_width=8)
    color = "tomato" if snap["is_attached"] else "darkorange"
    plotter.add_mesh(pv.Cube(center=snap["part_position"], x_length=0.25, y_length=0.25, z_length=0.2), color=color)
    if len(snap["trace"]) > 2:
        plotter.add_mesh(pv.lines_from_points(snap["trace"]), color="purple", line_width=1)
    plotter.add_text(f"frame {snap['frame']}  state:{snap['state']}  attached:{snap['is_attached']}",
                      position="upper_left", font_size=10, color="black")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = CR6Engine()
    plotter = new_offscreen_plotter(camera_position=[(8, -8, 6), (0, 0, 1.5), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=600, snapshot_every=150, out_dir=out, label="cr6v31_")
