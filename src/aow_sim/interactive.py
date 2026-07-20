"""Shared interactive-viewer loop for teleop harnesses (run_pivot, run_drive).

macOS requires the passive viewer to run under mjpython; the RuntimeError from
launch_passive is converted into that hint."""

from __future__ import annotations

import time

import mujoco
import mujoco.viewer


def teleop_loop(model, data, step, on_key, intro: str, module: str,
                draw=None) -> None:
    """Run `step(model, data)` every physics step inside a real-time-paced
    passive viewer with `on_key(keycode)` handling. If `draw` is given, it is
    called as `draw(viewer.user_scn, model, data)` each rendered frame to add
    overlay geometry."""
    print(intro)
    try:
        viewer = mujoco.viewer.launch_passive(model, data, key_callback=on_key)
    except RuntimeError as e:
        raise SystemExit(
            f"could not start the interactive viewer ({e}).\n"
            "On macOS the passive viewer must run under mjpython:\n"
            f"    mjpython -m {module} --teleop"
        ) from e
    sync_every = max(1, int(1 / 60 / model.opt.timestep))
    with viewer as v:
        t_wall = time.perf_counter()
        while v.is_running():
            for _ in range(sync_every):
                step(model, data)
                mujoco.mj_step(model, data)
            if draw is not None:
                draw(v.user_scn, model, data)
            v.sync()
            t_wall += sync_every * model.opt.timestep
            lag = t_wall - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            else:
                t_wall = time.perf_counter()
