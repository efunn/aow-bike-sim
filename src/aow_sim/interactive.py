"""Shared interactive-viewer loop for teleop harnesses (run_pivot, run_drive).

macOS requires the passive viewer to run under mjpython; the RuntimeError from
launch_passive is converted into that hint."""

from __future__ import annotations

import time

import mujoco
import mujoco.viewer


# NOTE: do not resize the viewer window from here. An earlier attempt reached
# the backing NSWindow via AppKit and set its frame; under mjpython that is a
# hard crash ("trace trap"), because mjpython runs the viewer's UI on the MAIN
# thread and this script on a secondary one, and AppKit window calls off the
# main thread abort the process. Doing it properly needs the call marshalled
# onto the main queue; until that is written AND tested on a real display,
# F5 (MuJoCo's own fullscreen toggle) is the only safe route.


def teleop_loop(model, data, step, on_key, intro: str, module: str,
                draw=None, show_ui: bool = False, on_start=None) -> None:
    """Run `step(model, data)` every physics step inside a real-time-paced
    passive viewer with `on_key(keycode)` handling. If `draw` is given, it is
    called as `draw(viewer.user_scn, model, data)` each rendered frame to add
    overlay geometry.

    `show_ui` restores the viewer's two side panels. They are off by default:
    teleop is driven from the keyboard, the panels eat a third of the window,
    and the only button worth having (Reset) is already bound to Backspace.
    The viewer has no API to *start* fullscreen — F5 toggles it."""
    print(intro)
    print("  F5 fullscreen · Backspace reset the sim"
          + ("" if show_ui else " · --ui brings the side panels back"))
    try:
        viewer = mujoco.viewer.launch_passive(model, data, key_callback=on_key,
                                              show_left_ui=show_ui,
                                              show_right_ui=show_ui)
    except RuntimeError as e:
        raise SystemExit(
            f"could not start the interactive viewer ({e}).\n"
            "On macOS the passive viewer must run under mjpython:\n"
            f"    mjpython -m {module} --teleop"
        ) from e
    sync_every = max(1, int(1 / 60 / model.opt.timestep))
    with viewer as v:
        # Hand the caller the live handle once, so it can drive the camera.
        # Camera mutations belong on THIS thread (the loop below), not in the
        # key callback, which the viewer runs on its own.
        if on_start is not None:
            on_start(v)
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
