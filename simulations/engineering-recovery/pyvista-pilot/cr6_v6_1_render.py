"""PyVista renderer for CR6_6Axis_V6_1_Workspace_Guard (Phase 3B, Class C migration)."""
import sys
import pyvista as pv

from cr6_v6_1_engine import CR6WorkspaceGuardEngine, SAFE_INTERCEPT_RADIUS, D1
from pv_toolkit import new_offscreen_plotter, robot_arm_mesh, run_and_snapshot


def build_scene(plotter, snap):
    plotter.clear()
    plotter.add_mesh(pv.Box(bounds=(-4, 4, -3.3, -2.3, 0.98, 1.02)), color="steelblue", opacity=0.35)
    plotter.add_mesh(pv.Sphere(radius=SAFE_INTERCEPT_RADIUS, center=(0, 0, D1)), color="lime", opacity=0.06, style="wireframe")
    plotter.add_mesh(robot_arm_mesh(snap["pts"]), color="#FF3333", line_width=8)
    color = "tomato" if snap["is_attached"] else ("gold" if snap["in_range"] else "darkorange")
    plotter.add_mesh(pv.Cube(center=snap["part_render_pos"], x_length=0.25, y_length=0.25, z_length=0.2), color=color)
    if len(snap["trace"]) > 2:
        plotter.add_mesh(pv.lines_from_points(snap["trace"]), color="purple", line_width=1)
    plotter.add_text(
        f"frame {snap['frame']}  state:{snap['state']}  attached:{snap['is_attached']}  "
        f"in_range:{snap['in_range']}  cleared:{snap['parts_cleared']}",
        position="upper_left", font_size=10, color="black",
    )


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = CR6WorkspaceGuardEngine()
    plotter = new_offscreen_plotter(camera_position=[(8, -8, 6), (0, 0, 1.5), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=700, snapshot_every=175, out_dir=out, label="cr6v61_")
