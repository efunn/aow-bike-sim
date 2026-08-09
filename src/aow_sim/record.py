"""Offscreen recorder: drive a scripted command sequence and render it to
video + a contact sheet, with the teleop ground dial drawn in.

No viewer, no mjpython, no window — so it runs anywhere (CI, ssh, a headless
box) and, unlike teleop, the result is a file you can look at afterwards.

  python -m aow_sim.record --script crab            # -> traces/crab.mp4 + .png
  python -m aow_sim.record --script drive --general general_rl_1k
  python -m aow_sim.record --script crab --analytic # the LQR path instead
  python -m aow_sim.record --script right           # self-righting, rear view

The dial is the same `run_drive._overlay` teleop draws (key `2`), rendered
onto the offscreen scene rather than the viewer's, so what you see here is
what you would see driving:

  green tick = commanded heading   cyan tick  = actual heading
  orange ray = commanded velocity  yellow ray = actual velocity

The rear wheel's roller detail is deliberately not worth watching here — the
AOW spins too fast to read at video rates. Body pose, steer angle and the dial
carry the information.

`--script right` is a different animal: it starts the bike ALREADY FALLEN and
runs the self-righting mechanism (see docs/plans/self-righting.md) through
deploy → hand-off → retract, so there is no gamepad, no trail worth drawing,
and the default plan view is exactly the wrong camera. It defaults to `rear`
and draws the phase instead of the stick gates.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

try:
    import imageio.v2 as imageio
except ImportError as e:            # fail at import, not 15 s into a render
    raise SystemExit(
        "aow_sim.record needs imageio + the ffmpeg plugin:\n"
        "    pip install -e '.[viz]'   (or: pip install imageio imageio-ffmpeg)"
    ) from e

from .build_model import build_model, load_params
from .control import DriveController
from .control import gamepad as gp
from .control.linearize import design_all, settle_upright
from .control.righting import (RightingSequencer, roll_pitch,
                               settle_fallen, settle_inverted)
from .run_drive import (_LEAD_MAX, _TRAIL, _TURN_RATE, _command_ref,
                        _fresh, _overlay)

# Scripts that own their own controller and start from a non-upright pose, so
# the gamepad/command machinery below does not apply to them.
_SEQUENCES = {"right"}

# (t [s], label, fn(controller, psi0)) -- applied once when the clock passes t.
_SCRIPTS = {
    "drive": [
        (0.5, "forward 0.8", lambda c, p: c.set_command_polar(0.8, 0.0, psi_cmd=p)),
        (4.0, "turn +90", lambda c, p: c.set_command_polar(0.8, 0.0, psi_cmd=p + np.pi / 2)),
        (8.0, "stop", lambda c, p: c.set_command_polar(0.0, 0.0, psi_cmd=p + np.pi / 2)),
        (10.5, "reverse 0.5", lambda c, p: c.set_command_polar(-0.5, 0.0, psi_cmd=p + np.pi / 2)),
        (14.0, "about-face", lambda c, p: c.set_command_polar(0.0, 0.0, psi_cmd=p - np.pi / 2)),
    ],
    # The command that motivated this tool: does the bike translate sideways
    # while holding heading, or does it yaw away and slide?
    "crab": [
        (1.0, "crab LEFT 0.4", lambda c, p: c.set_command_polar(0.4, np.pi / 2, psi_cmd=p)),
        (5.0, "hold", lambda c, p: c.set_command_polar(0.0, 0.0, psi_cmd=p)),
        (7.0, "crab RIGHT 0.4", lambda c, p: c.set_command_polar(0.4, -np.pi / 2, psi_cmd=p)),
        (11.0, "hold", lambda c, p: c.set_command_polar(0.0, 0.0, psi_cmd=p)),
        (13.0, "fwd+crab", lambda c, p: c.set_command_polar(
            float(np.hypot(0.5, 0.3)), float(np.arctan2(0.3, 0.5)), psi_cmd=p)),
    ],
}


# Drawing programs. Each segment is (seconds, Pad, label, pen) and the Pad is
# held for its whole duration -- these are stick DEFLECTIONS, not one-shot
# commands, so every shape here is reachable by hand or by controller.
#
#   ly = forward/back    rx = heading rate    pen "up" stops laying trail
#
# Geometry: at speed v with heading rate psi_dot the bike arcs at radius
# R = v / psi_dot, and psi_dot = rx * _TURN_RATE. So rx sets curvature and ly
# sets how fast the pen moves along it -- which means scaling ly and rx
# TOGETHER draws the SAME shape faster. These are 1.5x the first pass, with
# durations divided to match; that keeps rx off the stops (0.90 of full
# deflection on the S, the tightest of the three) and leaves the 35 deg lead
# clamp some headroom. Verified by eye: all three letters still close.
_LY = 0.63                      # ~0.76 m/s at v_max 1.2
_HALF_TURN_S = 2.93             # seconds for 180 deg at rx = 0.90


def _pad(ly=0.0, rx=0.0, lx=0.0):
    return gp.Pad(ly=ly, rx=rx, lx=lx)


_DRAWINGS = {
    # One closed circle: constant speed, constant curvature.
    "o": [
        (0.8, _pad(), "settle", "up"),
        (7.7, _pad(ly=_LY, rx=0.78), "circle", "down"),
        (0.6, _pad(), "stop", "down"),
    ],
    # Two mirrored half-circles. This is the shape that needs a symmetric
    # policy -- general_rl_1k has the lowest handedness of the three (58%).
    "s": [
        (0.8, _pad(), "settle", "up"),
        (_HALF_TURN_S, _pad(ly=_LY, rx=0.90), "upper arc (left)", "down"),
        (_HALF_TURN_S, _pad(ly=_LY, rx=-0.90), "lower arc (right)", "down"),
        (0.6, _pad(), "stop", "down"),
    ],
    # The one that needs the pen: crossbar, lift, reposition, drop, stem.
    "t": [
        (0.8, _pad(), "settle", "up"),
        (2.0, _pad(ly=_LY), "crossbar", "down"),
        (0.5, _pad(), "PEN UP", "up"),
        (0.8, _pad(ly=-_LY), "reverse to midpoint", "up"),
        # Absolute snap, not a timed turn: a held rx is open-loop and overshot
        # by ~30 deg, which put the stem on a diagonal. This is teleop's 6/7
        # (and the pad's snap button) -- exact, and clamp-exempt by design.
        (2.2, _pad(), "snap -90 deg", "up", -90.0),
        (2.0, _pad(ly=_LY), "stem (PEN DOWN)", "down"),
        (0.6, _pad(), "stop", "down"),
    ],
}


def _disc(frame, cx, cy, r, rgb, width=None):
    """Filled disc, or a ring when `width` is given. Pure numpy: the HUD must
    not depend on PIL, which is only ever optional here."""
    h, w, _ = frame.shape
    y0, y1 = max(0, cy - r - 1), min(h, cy + r + 2)
    x0, x1 = max(0, cx - r - 1), min(w, cx + r + 2)
    if y1 <= y0 or x1 <= x0:
        return
    yy, xx = np.ogrid[y0:y1, x0:x1]
    d2 = (yy - cy) ** 2 + (xx - cx) ** 2
    m = d2 <= r * r if width is None else \
        (d2 <= r * r) & (d2 >= (r - width) ** 2)
    frame[y0:y1, x0:x1][m] = rgb


def _hud(frame, seg, pen_down, v_max, crab_max):
    """Gamepad input overlay: what the sticks are doing, burnt into the frame.

    Drawn the way a controller reads, not the way the axes are signed --
    `lx`/`rx` are POSITIVE to the left (body +Y is left, and heading rate is
    CCW-positive), so the dot is mirrored to sit where the thumb would be.
    """
    if seg is None:
        return frame
    pad = seg[1]
    snap = seg[4] if len(seg) > 4 else None
    h, w, _ = frame.shape
    r = max(18, h // 11)                 # stick gate radius
    # Leave room BELOW the gates for their captions; at h - r - 14 the labels
    # ran off the bottom of the frame.
    pad_px, cy = r + 10, h - r - 34
    ink, face = (35, 35, 40), (238, 238, 240)

    for k, (label_cx, dx, dy) in enumerate((
            (pad_px, -pad.lx, -pad.ly),                      # LEFT stick
            (pad_px + 2 * r + 26, -pad.rx, 0.0))):           # RIGHT stick X
        _disc(frame, label_cx, cy, r, face)
        _disc(frame, label_cx, cy, r, ink, width=2)
        _disc(frame, label_cx, cy, 2, (170, 170, 175))       # centre pip
        dot = (int(label_cx + dx * (r - 6)), int(cy + dy * (r - 6)))
        _disc(frame, dot[0], dot[1], max(4, r // 4), (30, 90, 210))

    # Pen: filled = laying trail, hollow = pen up. Same red as the trail.
    px = pad_px + 4 * r + 52
    red = tuple(int(255 * v) for v in _TRAIL)
    _disc(frame, px, cy, r // 2, red if pen_down else face)
    _disc(frame, px, cy, r // 2, ink, width=2)

    # A heading SNAP is a button, not a stick, so the gates stay centred while
    # it runs -- light a button so the frame doesn't read as "no input".
    bx = px + r + 30
    if snap is not None:
        _disc(frame, bx, cy, r // 2, (250, 190, 40))
        _disc(frame, bx, cy, r // 2, ink, width=2)

    try:                                  # text is a bonus, never a dependency
        from PIL import Image, ImageDraw
        im = Image.fromarray(frame)
        dr = ImageDraw.Draw(im)
        dr.text((pad_px - 8, cy + r + 4), "L stick", fill=ink)
        dr.text((pad_px + 2 * r + 16, cy + r + 4), "R stick", fill=ink)
        dr.text((px - 14, cy + r + 4), "PEN " + ("DOWN" if pen_down else "UP"),
                fill=ink)
        if snap is not None:
            dr.text((bx - 20, cy + r + 4), f"SNAP {snap:+.0f}\u00b0", fill=ink)
        dr.text((pad_px - 8, cy - r - 16),
                f"v {pad.ly * v_max:+.2f}  crab {pad.lx * crab_max:+.2f}"
                f"  turn {pad.rx:+.2f}", fill=ink)
        frame = np.asarray(im)
    except Exception:
        pass
    return frame


# Tracking camera presets: (elevation, azimuth). `rear` looks up the bike's own
# +X from behind, which is the roll plane -- the only view a righting stroke
# reads in at all. -22 rather than level: a near-level camera spends the top
# third of the frame on empty sky, and this all happens near the floor.
_CAM_PRESETS = {"rear": (-22.0, 180.0), "front": (-22.0, 0.0)}


def _camera(model, mode: str, distance: float, elevation: float,
            azimuth: float, lookat):
    """`chase`/`rear`/`front` track the chassis; `top` is a FIXED world view.

    Default to fixed: a tracking camera pins the bike to the centre of frame,
    which is exactly what hides translation -- and translation is the whole
    question when judging a crab. Fixed + the world trail makes "moved
    sideways" and "spun on the spot" impossible to confuse.

    A righting stroke is the opposite case: it is all attitude and no
    translation, and a plan view of it shows nothing at all."""
    cam = mujoco.MjvCamera()
    if mode == "top":
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = lookat
    else:
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = model.body("chassis").id
    if mode in _CAM_PRESETS:
        elevation, azimuth = _CAM_PRESETS[mode]
    cam.distance, cam.elevation, cam.azimuth = distance, elevation, azimuth
    return cam


_INVERT_SETTLE_S = 3.0     # let it roll off the roof before deploying
_PHASE_LABEL = {"lift": "DEPLOY", "balance": "HAND-OFF", "retract": "RETRACT",
                "inverted": "ON ITS BACK", "fallen": "FALLEN"}
_PHASE_RGB = {"fallen": (200, 60, 50), "inverted": (150, 40, 130),
              "lift": (230, 150, 40),
              "balance": (40, 150, 70), "retract": (60, 110, 200)}


def _phase_hud(frame, info):
    """Righting-run overlay: which phase, how far the mechanism has swung, and
    how many turns that is at the servo.

    The stick gates `_hud` draws are meaningless here -- there is no gamepad in
    this run -- and the servo-turns readout is the point: past 1.00 the real
    XC330 has to be in extended-position (multi-turn) mode."""
    if info is None:
        return frame
    phase, roll, mech_deg, turns = info
    h = frame.shape[0]
    r = max(14, h // 16)
    cx, cy = r + 12, h - r - 30
    _disc(frame, cx, cy, r, _PHASE_RGB.get(phase, (120, 120, 120)))
    _disc(frame, cx, cy, r, (35, 35, 40), width=2)
    try:                                  # text is a bonus, never a dependency
        from PIL import Image, ImageDraw
        im = Image.fromarray(frame)
        dr = ImageDraw.Draw(im)
        ink = (20, 20, 20)
        dr.text((cx + r + 12, cy - 14),
                _PHASE_LABEL.get(phase, phase.upper()), fill=ink)
        dr.text((cx + r + 12, cy - 1), f"roll {roll:+6.1f}°", fill=ink)
        dr.text((cx + r + 12, cy + 12),
                f"mech {mech_deg:+6.1f}°   servo {turns:4.2f} turns"
                + ("  MULTI-TURN" if turns > 1.0 else ""), fill=ink)
        frame = np.asarray(im)
    except Exception:
        pass
    return frame


def _trail(scn, pts, rgba=(*_TRAIL, 1.0)):
    """Breadcrumbs of where the chassis has actually been, in WORLD frame.
    The dial is drawn under the bike and travels with it, so it cannot show
    displacement; this can."""
    for q in pts:
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([0.012, 0.012, 0.012]),
                            np.array([q[0], q[1], 0.004]), np.eye(3).ravel(),
                            np.asarray(rgba, np.float32))
        scn.ngeom += 1


def _record_righting(params, general: str | None, wings: bool, fps: int,
                     seconds: float, retract_after: float,
                     inverted: bool = False):
    """PASS 1 for `--script right`: start fallen, run the mechanism through
    deploy -> hand-off -> retract, and keep one snapshot per output frame.

    Returns (model, data, controller, states, marks) in the same shape the
    drive scripts produce, so PASS 2 renders both without branching."""
    model = build_model(params, variant="full", righting=True, wings=wings)
    design = design_all(params, build_model(params))
    data = mujoco.MjData(model)
    # `inverted` starts the run UPSIDE DOWN instead of on its side, so the
    # recording covers the part no still frame shows: the bike rolling off the
    # roof ridge onto its side before the mechanism has anything to push on.
    # settle=0 for the inverted start: the ROLL-OFF is the thing worth
    # filming, so the run has to begin at the drop rather than after it. The
    # mechanism is held at stow for `_INVERT_SETTLE_S` below so the bike gets
    # to find its own rest first, the way it would on the floor.
    data.qpos[:] = (settle_inverted(params, wings=wings, settle=0.0) if inverted
                    else settle_fallen(params, wings=wings))
    mujoco.mj_forward(model, data)
    move = general or params["control"].get("general_move", "general_rl")
    seq = RightingSequencer(params, model, wings=wings,
                            # The wing pair deploys the same way whichever side
                            # it fell on; the single arm has to reach for it.
                            direction=1.0 if wings else None,
                            retract_after=retract_after, move=move,
                            design=design)
    seq.reset(model, data)

    every = max(1, int(round(1.0 / fps / model.opt.timestep)))
    states, marks, phase = [], [], "inverted" if inverted else "fallen"
    if inverted:
        marks.append((0, _PHASE_LABEL.get(phase, phase)))
    hold = int(round((_INVERT_SETTLE_S if inverted else 0.0)
                     / model.opt.timestep))
    for i in range(int(round(seconds / model.opt.timestep))):
        roll, _ = roll_pitch(data.qpos[3:7])
        if i < hold:
            # Stowed and passive: let the roof do its half of the job before
            # the mechanism is allowed to do its own.
            data.ctrl[seq.aid] = seq.cmd
        else:
            seq.step(model, data)
        mujoco.mj_step(model, data)
        # During the hold the sequencer is not stepping, so its phase is still
        # its constructed one -- don't let that register as a transition.
        if i >= hold and seq.phase != phase:
            phase = seq.phase
            marks.append((len(states), _PHASE_LABEL.get(phase, phase)))
            print(f"  t={data.time:5.2f}s  {_PHASE_LABEL.get(phase, phase)}"
                  f"  roll {roll:+.1f} deg  servo {seq.servo_turns:.2f} turns")
        if len(states) * every <= int(data.time / model.opt.timestep):
            # The mechanism has no drive command until hand-off, so the ground
            # dial is suppressed (cmd=None) until there is one to draw.
            cmd = _command_ref(seq.ctrl, data) if seq.ctrl is not None else None
            states.append((data.qpos.copy(), data.qvel.copy(), cmd, False,
                           (phase, roll, np.degrees(seq.cmd), seq.servo_turns)))
    return model, data, seq.ctrl, states, marks


def record(script: str, general: str | None, analytic: bool, out: Path,
           width: int, height: int, fps: int, distance: float,
           elevation: float, azimuth: float, hockey: bool,
           camera: str = 'top', wings: bool = True, seconds: float = 12.0,
           retract_after: float = 1.0, inverted: bool = False) -> dict:
    params = load_params()
    righting = script in _SEQUENCES
    if righting:
        model, data, c, states, marks = _record_righting(
            params, general, wings, fps, seconds, retract_after, inverted)
        mode = ("wings" if wings else "arm") + ":" + (
            general or params["control"].get("general_move", "general_rl"))
        fell, v_max, crab_max, drawing = False, 1.2, 0.0, None
        return _render(model, data, c, states, marks, out, width, height, fps,
                       distance, elevation, azimuth, camera, v_max, crab_max,
                       {"mode": mode, "script": script, "fell": fell})

    model = build_model(params, variant="full", hockey=hockey)
    data = _fresh(model, settle_upright(model).qpos)
    c = DriveController(params, model)
    c.reset(model, data)

    mode = "analytic"
    if not analytic:
        name = general or params["control"].get("general_move", "general_rl")
        c.engage_general(data, name=name)
        mode = f"general:{name}"
    psi0 = c._psi

    drawing = _DRAWINGS.get(script)
    events = _SCRIPTS.get(script, [])
    duration = (sum(seg[0] for seg in drawing) if drawing
                else events[-1][0] + 4.0)
    every = max(1, int(round(1.0 / fps / model.opt.timestep)))
    v_max = c.profile.v_max

    # PASS 1 -- simulate, keeping one state snapshot per output frame. Nothing
    # is rendered yet, because the camera cannot be framed until the path is
    # known, and a fixed camera the bike drives out of is useless.
    marks, fired, states, fell = [], 0, [], False
    # Pen defaults DOWN for the diagnostic scripts. Only a drawing program has
    # pen choreography; a `crab` or `drive` run wants the trail throughout,
    # and starting it "up" silently produced no trail at all -- which is the
    # one thing those recordings exist to show.
    psi_cmd, seg_i, seg_end = psi0, -1, 0.0
    pen = "up" if drawing else "down"
    crab_max = float(getattr(c._gen, "v_lat_frac", 0.4)) * v_max if not analytic else 0.0
    while data.time < duration:
        if drawing:
            while seg_i + 1 < len(drawing) and data.time >= seg_end:
                seg_i += 1
                seg_end += drawing[seg_i][0]
                pen = drawing[seg_i][3]
                marks.append((len(states), drawing[seg_i][2]))
                print(f"  t={data.time:5.2f}s  {drawing[seg_i][2]}  [pen {pen}]")
            seg = drawing[max(seg_i, 0)]
            pad = seg[1]
            snap = seg[4] if len(seg) > 4 else None
            v_lon, v_lat, psi_cmd = gp.apply(
                pad, psi_cmd, model.opt.timestep, v_max, crab_max, _TURN_RATE)
            if snap is None:
                # Same 35 deg lead clamp teleop applies, so a shape drawn here
                # is a shape the arrows can draw: the policy is never handed a
                # heading command it has no chance of catching.
                psi_cmd = c._psi + float(np.clip(psi_cmd - c._psi,
                                                 -_LEAD_MAX, _LEAD_MAX))
            else:
                psi_cmd = psi0 + np.deg2rad(snap)
            speed, course = gp.to_polar(v_lon, v_lat)
            c.set_command_polar(speed, course, psi_cmd=psi_cmd)
        else:
            while fired < len(events) and data.time >= events[fired][0]:
                events[fired][2](c, psi0)
                marks.append((len(states), events[fired][1]))
                print(f"  t={data.time:5.2f}s  {events[fired][1]}")
                fired += 1
        c.step(model, data)
        mujoco.mj_step(model, data)
        if len(states) * every <= int(data.time / model.opt.timestep):
            states.append((data.qpos.copy(), data.qvel.copy(),
                           _command_ref(c, data), pen == "down",
                           drawing[max(seg_i, 0)] if drawing else None))
        R = data.body("chassis").xmat.reshape(3, 3)
        if abs(np.arctan2(R[2, 1], R[2, 2])) > np.deg2rad(60):
            fell = True
            print(f"  t={data.time:5.2f}s  FELL")
            break

    return _render(model, data, c, states, marks, out, width, height, fps,
                   distance, elevation, azimuth, camera, v_max, crab_max,
                   {"mode": mode, "script": script, "fell": fell})


def _render(model, data, c, states, marks, out: Path, width: int, height: int,
            fps: int, distance: float, elevation: float, azimuth: float,
            camera: str, v_max: float, crab_max: float, meta: dict) -> dict:
    """PASS 2 -- frame the camera to the path actually taken, then render the
    stored states. Replaying snapshots keeps this exact: no re-simulation,
    so the video cannot drift from the run that produced the numbers."""
    xy = np.array([q[:2] for q, *_ in states])
    lookat = np.array([xy[:, 0].mean(), xy[:, 1].mean(), 0.0])
    span = float(max(np.ptp(xy[:, 0]), np.ptp(xy[:, 1]), 0.6))
    if camera == "top":
        distance = span * 1.35 + 0.9      # bike + trail + margin, always in frame
    # Modest lift only: the earlier +0.6/+0.7 washed the floor to pure white.
    model.vis.headlight.ambient[:] = 0.45
    model.vis.headlight.diffuse[:] = 0.35
    model.vis.headlight.specular[:] = 0.0   # kill the blown-out floor hotspot
    # The offscreen framebuffer is declared in the MJCF (default 640x480) and
    # Renderer refuses anything larger, so raise it to whatever was asked for.
    model.vis.global_.offwidth = max(width, model.vis.global_.offwidth)
    model.vis.global_.offheight = max(height, model.vis.global_.offheight)
    renderer = mujoco.Renderer(model, height, width)
    cam = _camera(model, camera, distance, elevation, azimuth, lookat)

    # Stream frames straight to the encoder. Accumulating them was a silent
    # killer: 500 frames at 900x640x3 is ~880 MB, and the process just died
    # mid-render with no traceback. Only the contact-sheet tiles are kept.
    out.parent.mkdir(parents=True, exist_ok=True)
    picks = [0] + [min(i + int(fps * 1.5), len(states) - 1) for i, _ in marks]
    picks = sorted(set(p for p in picks if 0 <= p < len(states)))[:8]
    tiles = {}
    writer = imageio.get_writer(out, fps=fps, macro_block_size=1)
    for i, (qpos, qvel, cmd, pen_down, seg) in enumerate(states):
        data.qpos[:], data.qvel[:] = qpos, qvel
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        # reset=False: append the dial onto the model geoms already in the
        # scene instead of clearing them (the viewer owns a private scene;
        # this one does not). `cmd is None` before a righting run hands off:
        # there is no drive command yet, so there is no dial to draw.
        if cmd is not None:
            _overlay(renderer.scene, model, data, c, [True], v_max, reset=False,
                     command=cmd)
        # Only points laid down with the pen DOWN. Spheres, not a
        # polyline, so a pen-up gap is simply absent rather than
        # bridged by a stroke that was never driven.
        drawn = np.array([q[:2] for q, _, _, pen, _ in states[:i + 1] if pen])
        if len(drawn):
            _trail(renderer.scene, drawn)
        frame = (_phase_hud(renderer.render(), seg)
                 if meta["script"] in _SEQUENCES
                 else _hud(renderer.render(), seg, pen_down, v_max, crab_max))
        writer.append_data(frame)
        if i in picks:
            tiles[i] = frame.copy()
    writer.close()

    # Contact sheet: one tile per scripted event, so the whole run is readable
    # as a single still.
    sheet = out.with_suffix(".png")
    tiles = [tiles[p] for p in picks if p in tiles]
    if tiles:
        cols = min(4, len(tiles))
        rows = int(np.ceil(len(tiles) / cols))
        h, w, _ = tiles[0].shape
        grid = np.zeros((rows * h, cols * w, 3), np.uint8)
        for k, t in enumerate(tiles):
            r, col = divmod(k, cols)
            grid[r * h:(r + 1) * h, col * w:(col + 1) * w] = t
        # Burn the event labels in, so the sheet reads without the console log.
        try:
            from PIL import Image, ImageDraw
            im = Image.fromarray(grid)
            dr = ImageDraw.Draw(im)
            for k, pick in enumerate(picks):
                lab = "start"
                for i, name in marks:
                    if i <= pick:
                        lab = name
                r, col = divmod(k, cols)
                dr.text((col * w + 8, r * h + 8),
                        f"t={pick / fps:4.1f}s  {lab}", fill=(20, 20, 20))
            grid = np.asarray(im)
        except Exception:
            pass
        imageio.imwrite(sheet, grid)

    return {**meta, "frames": len(states),
            "seconds": round(float(data.time), 2),
            "video": str(out), "sheet": str(sheet)}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--script", default="crab",
                    choices=sorted(set(_SCRIPTS) | set(_DRAWINGS) | _SEQUENCES),
                    help="crab/drive = diagnostics; o/s/t = pen drawings; "
                         "right = self-righting from a fallen start")
    ap.add_argument("--general", default=None, metavar="NAME",
                    help="policy to drive with (moves/NAME.{yaml,npz})")
    ap.add_argument("--analytic", action="store_true",
                    help="drive with the LQR/analytic controller instead")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--camera", default=None,
                    choices=("top", "chase", "rear", "front"),
                    help="top = fixed world view (shows translation); "
                         "chase/rear/front = locked to the bike (shows "
                         "attitude). Default: top, or rear for --script right")
    ap.add_argument("--distance", type=float, default=3.0)
    # -89, not -65: a true plan view makes image-up = world +Y, so "moved
    # sideways" can be read off the frame instead of inferred from a tilt.
    ap.add_argument("--elevation", type=float, default=-89.0)
    ap.add_argument("--azimuth", type=float, default=90.0)
    ap.add_argument("--hockey", action="store_true")
    # --script right only
    ap.add_argument("--arm", action="store_true",
                    help="right: use the single righting arm instead of the "
                         "mirrored wing pair")
    ap.add_argument("--seconds", type=float, default=12.0,
                    help="right: run length")
    ap.add_argument("--inverted", action="store_true",
                    help="start the `right` script UPSIDE DOWN, so the "
                         "recording covers the roof rolling it onto its side "
                         "first (see self_righting.py invert)")
    ap.add_argument("--retract-after", type=float, default=1.0,
                    help="right: seconds the policy must hold it before the "
                         "mechanism is pulled back to stow")
    a = ap.parse_args()
    righting = a.script in _SEQUENCES
    # A righting stroke is all attitude and no translation, so the plan view
    # that the drive scripts want shows nothing at all here.
    camera = a.camera or ("rear" if righting else "top")
    distance = a.distance if a.camera or not righting else 0.9
    tag = (("wings" if not a.arm else "arm") if righting else
           a.general or ("analytic" if a.analytic else "general"))
    out = a.out or Path("traces") / f"{a.script}_{tag}.mp4"
    print(f"recording {a.script} ({tag})")
    print(record(a.script, a.general, a.analytic, out, a.width, a.height,
                 a.fps, distance, a.elevation, a.azimuth, a.hockey,
                 camera, not a.arm, a.seconds, a.retract_after,
                 a.inverted))


if __name__ == "__main__":
    main()
