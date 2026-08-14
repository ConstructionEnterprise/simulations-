"""
Shared PyVista rendering helpers for the Phase 3B migrations. No simulation
logic lives here -- every function takes already-computed state (points,
positions, sizes) and returns/adds a mesh. Kept generic across the CR6-arm
family and the structural (gantry/rail/tilt) family rather than duplicating
boilerplate in every per-simulation renderer script.
"""
import numpy as np
import pyvista as pv


def robot_arm_mesh(points):
    """A polyline through N joint points (base -> ... -> TCP)."""
    return pv.lines_from_points(np.asarray(points))


def box_at(cx, cy, cz, hw, hd, hh):
    return pv.Box(bounds=(cx - hw, cx + hw, cy - hd, cy + hd, cz - hh, cz + hh))


def part_box(p, default_w=0.3, default_d=0.3, default_h=0.2):
    """A part dict with x/y/z and optional w/d/h -- matches every sim's own part-dict shape."""
    hw = p.get("w", default_w) / 2
    hd = p.get("d", default_d) / 2
    hh = p.get("h", default_h) / 2
    return box_at(p["x"], p["y"], p["z"], hw, hd, hh)


def new_offscreen_plotter(window_size=(1000, 700), camera_position=None, background="white"):
    plotter = pv.Plotter(off_screen=True, window_size=window_size)
    plotter.set_background(background)
    if camera_position:
        plotter.camera_position = camera_position
    return plotter


def run_and_snapshot(engine, plotter, build_scene_fn, n_frames, snapshot_every, out_dir, label="", step_fn=None):
    """
    Drives the engine for n_frames (via `step_fn(engine)` if given, else
    `engine.step()` -- different sims' step() signatures vary, e.g. some
    take a speed argument), calling build_scene_fn(plotter, snap) at each
    snapshot interval and saving a PNG. build_scene_fn must call
    plotter.clear() itself (mirrors each sim's own ax.clear() convention).
    Returns the list of (frame, path) saved.
    """
    saved = []
    for frame in range(1, n_frames + 1):
        if step_fn:
            step_fn(engine)
        else:
            engine.step()
        if frame % snapshot_every == 0 or frame == n_frames:
            snap = engine.snapshot()
            build_scene_fn(plotter, snap)
            path = f"{out_dir}/{label}frame_{frame:04d}.png"
            plotter.screenshot(path)
            saved.append((frame, path))
            print(f"[{label}] frame {frame:5d} saved -> {path}")
    plotter.close()
    return saved
