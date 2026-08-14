"""PyVista renderer for CE_Integrated_Cell_V2_6 (Phase 3B, Class A migration)."""
import sys
import pyvista as pv

from v2_6_engine import World
from pv_toolkit import box_at, new_offscreen_plotter, robot_arm_mesh, run_and_snapshot

ROBOT_COLOR = {"F1": "#FF3333", "F2": "#CC00FF", "S1": "#FF5500", "S2": "#FF8C00"}


def build_scene(plotter, snap):
    plotter.clear()
    # Stations
    plotter.add_mesh(box_at(-6.0, 0, 0.9, 2.0, 1.4, 0.05), color="#999999", opacity=0.6)   # FIXED
    plotter.add_mesh(box_at(-1.0, 0, 0.85, 3.4, 1.4, 0.03), color="#AAAAAA", opacity=0.4)  # ROLLER
    plotter.add_mesh(box_at(5.5, 0, 0.9, 2.0, 1.4, 0.05), color="#999999", opacity=0.6)    # TILT
    for name, r in snap["robots"].items():
        plotter.add_mesh(robot_arm_mesh(r["pts"]), color=ROBOT_COLOR[name], line_width=7)
    c = snap["crane"]
    plotter.add_mesh(pv.Sphere(radius=0.15, center=(c["beam_x"], 0, c["hook_z"])), color="#E8A020")
    for (mtype, pos) in snap["placed_positions"]:
        plotter.add_mesh(pv.Cube(center=pos, x_length=0.3, y_length=0.3, z_length=0.15), color="#22CC66")
    plotter.add_text(
        f"tick {snap['tick']}  tilt:{snap['tilt_angle']:.0f}deg({snap['tilt_state']})  "
        f"framing_done:{snap['framing_complete']}  sheathing_done:{snap['sheathing_done']}",
        position="upper_left", font_size=10, color="black",
    )


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = World()
    plotter = new_offscreen_plotter(camera_position=[(0, -20, 15), (0, 0, 1), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=3000, snapshot_every=500, out_dir=out, label="v26_",
                      step_fn=lambda e: e.step(0.02))
