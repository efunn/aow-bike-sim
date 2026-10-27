"""The mirror, re-rendered from an onboard record: a video to lay over a real one.

    python -m aow_sim.mirror_replay traces/bike/<stamp>_run --info
    python -m aow_sim.mirror_replay traces/bike/<stamp>_run \\
        --from 12 --to 95 --face 14.0 --view 30 --fps 60

`bike.sh on` / `run` writes every tick to `traces/bike/<stamp>_run` (run_bike's
RunRecorder, 100 Hz). Each row here goes through `telemetry.apply_pose` -- the
function the live mirror calls on every packet -- so this draws what the
mirror would have drawn, without a station having been open. Same caveats as
the mirror (hw/telemetry.py): attitude, steer, wheel spin and the righting
crank are measured; ride height and the front wheel are drawn; the POSITION
is the bike's own dead reckoning, left in on purpose -- the drift is the
odometry, made visible. `--pin` holds the bike at the origin instead.

THE HEADING. The bike's yaw has no fixed relation to where a real camera
stands, so the virtual camera is placed RELATIVE TO THE BIKE at one moment:
`--face T` names a record time at which you know how the bike sat in your
shot, and `--view DEG` is where the camera stood, measured around the bike
from its nose that moment, counter-clockwise seen from above: 0 looks at the
nose, 90 at its left side, 180 from behind. Everything after T then turns
the way it turned in the room. Wrong by a few degrees? Change `--view`.

THE CLOCK. Output frame k shows record time `from + k / fps`. Sync by hand
in the editor against an event visible in both -- an engage, a crank lift.
`--info` prints the record's start (wall clock, to the second) and its
state changes with their record times, which is where to look for one.

Output: <run>/mirror.mp4 by default (traces/ is gitignored, synced), or --out.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

try:
    import imageio.v2 as imageio
except ImportError as e:      # pragma: no cover
    raise SystemExit("aow_sim.mirror_replay needs imageio + the ffmpeg plugin:\n"
                     "    pip install imageio imageio-ffmpeg") from e

from .build_model import (SwingLinkageSolver, build_model, strip_visuals,
                          tune_lighting)
from .control.linearize import settle_upright
from .drivetrain_model import base_params
from .hw import telemetry as T
from .control.steer import XC330_COUNTS_PER_RAD
from .hw.dynamixel import POS_WRAP
from .params import load_params

TRAIL_RGBA = (0.95, 0.75, 0.15, 0.9)


def load_run(run: Path) -> tuple[dict, dict]:
    """-> (columns, meta). Columns are the record's own keys: `bike_*` the
    controller's view, `raw_*` the registers as read."""
    z = np.load(run / "capture.npz")
    meta = json.loads((run / "meta.json").read_text())
    return {k: z[k] for k in z.files}, meta


def _col(cols: dict, key: str) -> np.ndarray:
    a = cols[key]
    return a.reshape(len(a), -1)


def turned(counts: np.ndarray) -> np.ndarray:
    """Accumulated servo-shaft angle [rad] from single-turn counts, the way
    BikeBus sums it (pos_delta, then a running sum), from 0 at row 0."""
    c = counts.astype(np.int64)
    d = ((np.diff(c) + POS_WRAP // 2) % POS_WRAP) - POS_WRAP // 2
    return np.concatenate([[0.0], np.cumsum(d) / XC330_COUNTS_PER_RAD])


def packets(cols: dict, meta: dict, params: dict) -> dict:
    """The record -> per-row arrays of the telemetry fields apply_pose reads."""
    ids = list(meta["ids"])
    role = {v: int(k) for k, v in meta["roles"].items()}
    onb = (params.get("control") or {}).get("onboard") or {}
    sign = onb.get("servo_sign", (1.0, 1.0))
    belt = float(params["drivetrain"]["belt_ratio"])
    pos = cols["raw_present_position"]
    out = {
        "t": _col(cols, "bike_t")[:, 0],
        "state": _col(cols, "bike_state")[:, 0].astype(int),
        "quat": _col(cols, "bike_quat"),
        "gyro": _col(cols, "bike_gyro"),
        "v_world": _col(cols, "bike_v_world"),
        "pos": _col(cols, "bike_pos"),
        "psi": _col(cols, "bike_psi")[:, 0],
        "steer": _col(cols, "bike_steer")[:, 0],
    }
    out["shaft"] = np.stack(
        [turned(pos[:, ids.index(role[r])]) * belt * float(s)
         for r, s in (("drive_a", sign[0]), ("drive_b", sign[1]))], axis=1)
    if "righting" in role:
        # Current-based position mode: the winding is real, so the counts
        # are taken as read (signed 32-bit), as BikeBus does.
        c = pos[:, ids.index(role["righting"])].astype(np.int64)
        c = np.where(c >= 2 ** 31, c - 2 ** 32, c)
        out["righting_pos"] = c / XC330_COUNTS_PER_RAD
    return out


def state_changes(pk: dict, meta: dict) -> list[tuple[float, str]]:
    names = meta.get("states", [])
    s, t = pk["state"], pk["t"]
    at = [0] + list(np.where(np.diff(s) != 0)[0] + 1)
    return [(float(t[i]), names[s[i]] if s[i] < len(names) else str(s[i]))
            for i in at]


def mirror_model(params: dict):
    """The live mirror's bike: default build (the righting module), bare look,
    teleop lighting. -> (model, rest qpos, swing_pose or None)."""
    model = build_model(params, variant="full")
    from .run_drive import TELEOP_FLOOR_M, _enlarge_drawn_floor
    _enlarge_drawn_floor(model, TELEOP_FLOOR_M)
    tune_lighting(model)
    strip_visuals(model)
    eq = settle_upright(model)
    swing_pose = None
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                         "swing_crank_joint") >= 0:
        solver = SwingLinkageSolver(params["righting"]["module"]["linkage"],
                                    params["omni_wheel"]["outer_radius"])
        onb = params["control"].get("onboard", {})
        stow = np.deg2rad(float(onb.get("righting_stow_deg", 180.0)))
        sgn = float(onb.get("righting_sign", 1))
        swing_pose = lambda servo, _s=solver: _s.pose((float(servo) - stow) * sgn)
    return model, eq.qpos.copy(), swing_pose


def _trail(scn, pts) -> None:
    for a, b in zip(pts[:-1], pts[1:]):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                            np.zeros(3), np.eye(3).ravel(),
                            np.array(TRAIL_RGBA, np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.004,
                             np.array([a[0], a[1], 0.002]),
                             np.array([b[0], b[1], 0.002]))
        scn.ngeom += 1


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path, help="traces/bike/<stamp>_run")
    ap.add_argument("--info", action="store_true",
                    help="print the record's start and its state changes, render nothing")
    ap.add_argument("--from", dest="t0", type=float, default=None,
                    help="record time [s] of the first frame (default: start)")
    ap.add_argument("--to", dest="t1", type=float, default=None,
                    help="record time [s] of the last frame (default: end)")
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--face", type=float, default=None,
                    help="record time [s] the camera is placed against "
                         "(default: --from)")
    ap.add_argument("--view", type=float, default=30.0,
                    help="camera bearing from the bike's nose at --face [deg], "
                         "CCW from above: 0 front, 90 left, 180 behind")
    ap.add_argument("--elevation", type=float, default=-25.0,
                    help="camera pitch [deg], negative looks down")
    ap.add_argument("--distance", type=float, default=0.9, help="[m]")
    ap.add_argument("--fov", type=float, default=None,
                    help="vertical field of view [deg] (default: the model's)")
    ap.add_argument("--track", action="store_true",
                    help="camera follows the bike's position (default: fixed, "
                         "like the real one)")
    ap.add_argument("--pin", action="store_true",
                    help="hold the bike at the origin: attitude only, no odometry")
    ap.add_argument("--no-trail", action="store_true",
                    help="omit the dead-reckoned path drawn on the floor")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)

    cols, meta = load_run(args.run)
    params = base_params(load_params())
    pk = packets(cols, meta, params)
    t = pk["t"]
    if args.info:
        print(f"{args.run}: started {meta.get('created')}, "
              f"{t[-1] - t[0]:.1f} s, policy {meta.get('policy')}")
        for ts, name in state_changes(pk, meta):
            print(f"  {ts:8.2f} s  {name}")
        return

    t0 = t[0] if args.t0 is None else args.t0
    t1 = t[-1] if args.t1 is None else args.t1
    t_face = t0 if args.face is None else args.face
    frames = np.arange(t0, t1, 1.0 / args.fps)
    rows = np.clip(np.searchsorted(t, frames), 0, len(t) - 1)
    i_face = int(np.clip(np.searchsorted(t, t_face), 0, len(t) - 1))

    model, rest, swing_pose = mirror_model(params)
    data = mujoco.MjData(model)
    data.qpos[:] = rest
    adr = T.PoseAdr(model)
    rest_z = float(rest[2])
    if args.fov is not None:
        model.vis.global_.fovy = args.fov
    model.vis.global_.offwidth = max(args.width, model.vis.global_.offwidth)
    model.vis.global_.offheight = max(args.height, model.vis.global_.offheight)
    renderer = mujoco.Renderer(model, args.height, args.width)

    origin = pk["pos"][i_face].copy()
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    # A free camera at azimuth `az` LOOKS along az, so it stands on the
    # opposite side: in front of a nose at psi means az = psi + 180.
    cam.azimuth = float(np.degrees(pk["psi"][i_face]) + args.view + 180.0)
    cam.elevation, cam.distance = args.elevation, args.distance
    cam.lookat[:] = [0.0, 0.0, 0.06] if args.pin else [*origin, 0.06]

    out = args.out or args.run / "mirror.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(out, fps=args.fps, macro_block_size=1)
    keys = ("quat", "gyro", "v_world", "pos", "steer", "shaft", "righting_pos")
    shaft, path = (0.0, 0.0), []
    try:
        for k, i in enumerate(rows):
            tel = {key: pk[key][i] for key in keys if key in pk}
            if args.pin:
                tel["pos"] = np.zeros(2)
            shaft = T.apply_pose(model, data, tel, adr, params,
                                 shaft_angle=shaft, rest_z=rest_z,
                                 dt=1.0 / args.fps, swing_pose=swing_pose)
            mujoco.mj_forward(model, data)
            if args.track and not args.pin:
                cam.lookat[:2] = tel["pos"]
            renderer.update_scene(data, camera=cam)
            if not (args.no_trail or args.pin):
                path.append(tuple(tel["pos"]))
                _trail(renderer.scene, path[::max(1, len(path) // 400)])
            writer.append_data(renderer.render())
    finally:
        writer.close()
    print(f"{out}: {len(rows)} frames at {args.fps:g} fps, record "
          f"{t0:.2f}-{t1:.2f} s; camera {args.view:+.0f} deg from the nose "
          f"at {t_face:.2f} s")


if __name__ == "__main__":
    main()
