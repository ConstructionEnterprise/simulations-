"""PyVista renderer for CE_Overhead_Gantry_V1 (Phase 3B, Class A migration)."""
import sys
import pyvista as pv

from gantry_engine import World
from pv_toolkit import box_at, new_offscreen_plotter, run_and_snapshot


def build_scene(plotter, snap):
    plotter.clear()
    bx, ty, hz = snap["bridge_x"], snap["trolley_y"], snap["hook_z"]
    # Runway rails
    plotter.add_mesh(pv.Line((2, -4.5, 0), (26, -4.5, 0)), color="gray", line_width=3)
    plotter.add_mesh(pv.Line((2, 4.5, 0), (26, 4.5, 0)), color="gray", line_width=3)
    # End trucks (columns) at bridge_x, both rails
    for rail_y in (-4.5, 4.5):
        plotter.add_mesh(box_at(bx, rail_y, 2.9, 0.2, 0.25, 2.9), color="#4A4A4A")
    # Bridge beam spans between the rails at bridge_x
    plotter.add_mesh(box_at(bx, 0, 5.8, 0.15, 4.5, 0.28), color="#CC6600")
    # Trolley/hoist
    plotter.add_mesh(box_at(bx, ty, 5.8, 0.35, 0.28, 0.23), color="#1E1E1E")
    # Hook wire + hook
    plotter.add_mesh(pv.Line((bx, ty, 5.8), (bx, ty, hz)), color="#888888", line_width=2)
    plotter.add_mesh(pv.Sphere(radius=0.15, center=(bx, ty, hz)), color="#E8A020")
    plotter.add_text(
        f"tick {snap['tick']}  state:{snap['state']}  placed:{[p[0] for p in snap['placed_walls']]}  done:{snap['module_done']}",
        position="upper_left", font_size=10, color="black",
    )


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    engine = World()
    plotter = new_offscreen_plotter(camera_position=[(20, -25, 20), (17, 0, 3), (0, 0, 1)])
    run_and_snapshot(engine, plotter, build_scene, n_frames=4000, snapshot_every=500, out_dir=out, label="gantry_")
