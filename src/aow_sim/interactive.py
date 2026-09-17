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


def steps_per_frame(timestep: float, slowmo: float = 1.0, fps: float = 60.0) -> int:
    """Physics steps to advance between rendered frames.

    Slow motion cuts steps-per-frame rather than lengthening the frame, so the
    RENDER rate stays at `fps` and the motion stays smooth; spending the extra
    wall time inside a frame instead would render at fps/slowmo -- 6 fps at
    10x, a stutter rather than slow motion.

    It bottoms out at 1: past `1/fps/timestep` (about 42x at the shipped 60 fps
    and 4e-4 timestep) every frame is already a single physics step, and
    further slowdown can only come from longer frames, so the frame rate starts
    falling instead. That is the ceiling worth asking for, the same one
    analysis/wheel_slowmo.py documents for its offline renders.
    """
    return max(1, int(round(1 / fps / timestep / max(1e-3, slowmo))))


def teleop_loop(model, data, step, on_key, intro: str, module: str,
                draw=None, show_ui: bool = False, on_start=None,
                slowmo=None, paused=None, pre_step=None) -> None:
    """Run `step(model, data)` every physics step inside a real-time-paced
    passive viewer with `on_key(keycode)` handling. If `draw` is given, it is
    called as `draw(viewer.user_scn, model, data)` each rendered frame to add
    overlay geometry.

    `slowmo` is an optional one-element list holding the playback divisor, read
    fresh every frame so it can be changed from a key callback while running.
    2.0 means one second of sim takes two of wall clock. It buys wall time per
    physics step, NOT extra resolution: the trajectory is bit-identical, and
    `sim.timestep` is untouched.

    `paused` is an optional one-element list read fresh every frame, like
    `slowmo`. While it holds True the physics does NOT advance -- but `step`
    is still called once per frame, so keys are processed and the scene keeps
    redrawing. That is what lets a modal overlay be adjusted against a frozen
    bike. Anything `step` does off `data.time` stalls while paused, so a
    caller that needs a clock there must use a wall one.

    `pre_step(data)`, if given, runs immediately before every `mj_step` and
    never while paused -- the hook the detailed drivetrain needs
    (drivetrain_model.DrivetrainSim), which integrates per physics step.

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
    with viewer as v:
        # Hand the caller the live handle once, so it can drive the camera.
        # Camera mutations belong on THIS thread (the loop below), not in the
        # key callback, which the viewer runs on its own.
        if on_start is not None:
            on_start(v)
        t_wall = time.perf_counter()
        while v.is_running():
            # Recomputed per frame, because `slowmo` is live.
            f = max(1e-3, float(slowmo[0])) if slowmo else 1.0
            n = steps_per_frame(model.opt.timestep, f)
            if paused is not None and paused[0]:
                step(model, data)          # keys and drawing, no physics
                n = 0
            else:
                for _ in range(n):
                    step(model, data)
                    if pre_step is not None:
                        pre_step(data)
                    mujoco.mj_step(model, data)
            if draw is not None:
                draw(v.user_scn, model, data)
            v.sync()
            t_wall += n * model.opt.timestep * f
            lag = t_wall - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            else:
                t_wall = time.perf_counter()


def mirror_loop(model, data, frame, on_key, intro: str, module: str,
                draw=None, show_ui: bool = False, on_start=None,
                fps: float = 60.0) -> None:
    """Like `teleop_loop`, but the STATE COMES FROM OUTSIDE and there is no
    physics. `frame(model, data)` is called once per rendered frame and is
    expected to write `qpos`/`qvel` and call `mj_forward` itself.

    A separate function rather than a flag on `teleop_loop`, deliberately. The
    two differ in the one line that matters -- `mj_step` -- and everything
    around it (slow motion, pause, `pre_step`, steps-per-frame) is meaningless
    when the trajectory is somebody else's. Threading a `mirror=True` through
    would put four dead branches inside the loop that drives the simulator,
    which is the loop least worth making conditional.

    PACED ON THE WALL CLOCK, not on `data.time`: the bike's clock is the bike's
    and arrives in the packets. Frames are rendered at `fps` whatever the
    telemetry rate, so a dropped packet redraws the last pose rather than
    freezing the window, and a link running faster than the display simply
    loses intermediate frames -- which for a mirror is correct. `frame` should
    therefore be cheap and non-blocking; if it waits on a socket, the viewer
    stops.
    """
    print(intro)
    print("  F5 fullscreen"
          + ("" if show_ui else " · --ui brings the side panels back"))
    try:
        viewer = mujoco.viewer.launch_passive(model, data, key_callback=on_key,
                                              show_left_ui=show_ui,
                                              show_right_ui=show_ui)
    except RuntimeError as e:
        raise SystemExit(
            f"could not start the interactive viewer ({e}).\n"
            "On macOS the passive viewer must run under mjpython:\n"
            f"    mjpython -m {module} --mirror <host>"
        ) from e
    with viewer as v:
        if on_start is not None:
            on_start(v)
        period = 1.0 / max(1.0, fps)
        t_wall = time.perf_counter()
        while v.is_running():
            frame(model, data)
            if draw is not None:
                draw(v.user_scn, model, data)
            v.sync()
            t_wall += period
            lag = t_wall - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            else:
                t_wall = time.perf_counter()
