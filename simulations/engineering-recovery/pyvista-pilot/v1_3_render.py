"""PyVista renderer for CE_Integrated_Cell_V1_3 (Phase 3B, Class A migration)."""
import sys
import pyvista as pv

from v1_3_engine import World
from pv_toolkit import box_at, new_offscreen_plotter, robot_arm_mesh, run_and_snapshot

ROBOT_COLOR = {"F1": "#FF3333", "F2": "#CC00FF", "S1": "#FF5500", "S2": "#FF66CC", "RR": "#FF8C00"}


def build_scene(plotter, snap):
    plotter.clear()
    plotter.add_mesh(box_at(-6.0, 0, 0.9, 1.75, 1.25, 0.05), color="#999999", opacity=0.6)   # framing
    plotter.add_mesh(box_at(5.5, 0, 0.9, 1.75, 1.25, 0.05), color="#999999", opacity=0.6)    # sheathing
    plotter.add_mesh(pv.Line((-2, -1.5, 0), (13, -1.5, 0)), color="gray", line_width=2)      # inspection rail
    for name, r in snap["robots"].items():
        plotter.add_mesh(robot_arm_mesh(r["pts"]), color=ROBOT_COLOR[name], line_width=7)
    for (mtype, pos) in snap["placed_positions"]:
        plotter.add_mesh(pv.Cube(center=pos, x_length=0.3, y_length=0.3, z_length=0.15), color="#22CC66")
    plotter.add_text(
        f"tick {snap['tick']}  framing_done:{snap['framing_complete']}  "
        f"sheathing_done:{snap['sheathing_done']}  wall_complete:{snap['wall_complete']}  rail_x:{snap['rail_x']:.1f}",
        position="upper_left", font_size=9, color="black",
    )


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = World()
    plotter = new_offscreen_plotter(camera_position=[(0, -18, 14), (0, 0, 1), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=3000, snapshot_every=500, out_dir=out, label="v13_",
                      step_fn=lambda e: e.step(0.02))
