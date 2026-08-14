"""PyVista renderer for CE_Module_Assembly_V1 (Phase 3B, Class A migration)."""
import sys
import pyvista as pv

from module_assembly_engine import World
from pv_toolkit import box_at, new_offscreen_plotter, robot_arm_mesh, run_and_snapshot

ROBOT_COLOR = {"F1": "#FF3333", "F2": "#CC00FF", "S1": "#FF5500", "S2": "#FF8C00"}


def build_scene(plotter, snap):
    plotter.clear()
    plotter.add_mesh(box_at(-6.0, 0, 0.9, 2.0, 1.4, 0.05), color="#999999", opacity=0.6)
    plotter.add_mesh(box_at(5.5, 0, 0.9, 2.0, 1.4, 0.05), color="#999999", opacity=0.6)
    plotter.add_mesh(box_at(22.0, 0, 0.9, 2.0, 2.0, 0.05), color="#777777", opacity=0.5)  # module jig
    for name, r in snap["robots"].items():
        plotter.add_mesh(robot_arm_mesh(r["pts"]), color=ROBOT_COLOR[name], line_width=7)
    c = snap["crane"]
    plotter.add_mesh(pv.Sphere(radius=0.15, center=(c["beam_x"], 0, c["hook_z"])), color="#E8A020")
    for (mtype, pos) in snap["placed_positions"]:
        plotter.add_mesh(pv.Cube(center=pos, x_length=0.3, y_length=0.3, z_length=0.15), color="#22CC66")
    plotter.add_text(
        f"tick {snap['tick']}  panels {snap['panels_placed']}/4 {snap['placed_walls']}  module_done:{snap['module_complete']}",
        position="upper_left", font_size=10, color="black",
    )


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = World()
    plotter = new_offscreen_plotter(camera_position=[(8, -30, 22), (8, 0, 1), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=6000, snapshot_every=1000, out_dir=out, label="modasm_",
                      step_fn=lambda e: e.step(0.02))
