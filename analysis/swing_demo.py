"""Render the co-rotating swing-wing mechanism so it can be SEEN.

The kinematics are hard to read from numbers: the pair shares one shaft and one
actuator, and the whole behaviour is that a single sign in the joint equality
(+1 instead of the geared pair's -1) makes one wing swing down and out while the
other tucks up and in. A table of foot heights does not show that. A video does.

Three shots, concatenated:
  orbit    the bike parked at centre, camera circling, so the symmetric rest V
           and the shared pivot line are legible
  sweep    left -> centre -> right -> centre -> left from a fixed side camera:
           the co-rotation, and the far wing stopping clear of the chassis
  ball     with --hockey, the same sweep next to a ball at the strike pose

  python analysis/swing_demo.py
  python analysis/swing_demo.py --hockey --out traces/swing_ball.mp4

Writes an mp4 under traces/ (gitignored, Dropbox-synced). Read-only otherwise.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np
import yaml

from aow_sim.build_model import (SWING_CFG, build_model, load_params,
                                 tune_lighting)

REPO = Path(__file__).resolve().parents[1]


def _writer(out: Path, fps: int):
    try:
        import imageio.v2 as imageio
    except ImportError as e:                        # pragma: no cover
        raise SystemExit("needs imageio + ffmpeg: pip install -e '.[viz]'") from e
    out.parent.mkdir(parents=True, exist_ok=True)
    return imageio.get_writer(out, fps=fps, macro_block_size=1)


def _hold(model, data, aid, target, seconds, dt):
    """Drive the actuator to `target` and step, yielding nothing -- the caller
    renders. Kept separate so every shot advances physics identically."""
    for _ in range(int(seconds / dt)):
        data.ctrl[aid] = target
        mujoco.mj_step(model, data)
        yield


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hockey", action="store_true", help="include the ball scene")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    a = ap.parse_args()

    p = load_params()
    cfg = yaml.safe_load(SWING_CFG.read_text())
    model = build_model(p, swing=True, hockey=a.hockey)
    tune_lighting(model)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    aid = model.actuator("swing").id
    dep = np.deg2rad(float(cfg["deploy_deg"]))
    dt = model.opt.timestep
    every = max(1, int(round(1.0 / (a.fps * dt))))

    # The bike is NOT balancing here -- there is no controller in this script.
    # Freeze the chassis so the mechanism is the only thing moving; a toppling
    # bike would bury the very motion the video exists to show.
    model.body_gravcomp[model.body("chassis").id] = 1.0
    model.opt.gravity[:] = 0.0

    out = a.out or REPO / "traces" / ("swing_ball.mp4" if a.hockey else "swing_wings.mp4")
    # Renderer refuses anything larger than the model's offscreen buffer, and
    # the MJCF default is small -- same fix as aow_sim.record.
    model.vis.global_.offwidth = max(a.width, model.vis.global_.offwidth)
    model.vis.global_.offheight = max(a.height, model.vis.global_.offheight)
    renderer = mujoco.Renderer(model, a.height, a.width)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.09, 0.0, 0.08]
    cam.distance = 0.62
    cam.elevation = -14

    plan = [("orbit", 0.0, 3.0), ("centre", 0.0, 0.4),
            ("left", -dep, 1.1), ("centre", 0.0, 0.9),
            ("right", dep, 1.1), ("centre", 0.0, 0.9),
            ("left", -dep, 1.1), ("centre", 0.0, 0.9)]

    n = 0
    with _writer(out, a.fps) as w:
        for label, target, secs in plan:
            for k, _ in enumerate(_hold(model, data, aid, target, secs, dt)):
                n += 1
                if n % every:
                    continue
                if label == "orbit":
                    cam.azimuth = 90 + 360 * (n * dt) / 3.0
                else:
                    cam.azimuth = 90            # fixed side view for the sweep
                renderer.update_scene(data, cam)
                w.append_data(renderer.render())
    print(f"wrote {out}  ({out.stat().st_size / 1e6:.1f} MB)")
    print(f"  rest V at +-{cfg['rest_deg']:.0f} deg, stroke +-{cfg['deploy_deg']:.0f} deg, "
          f"{cfg['gear_ratio']:g}:1")


if __name__ == "__main__":
    main()
