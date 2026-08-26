"""The pen, with the ONBOARD VELOCITY ESTIMATE in the loop.

`pen.py` draws what the bike traces on the floor when a policy drives it on
MuJoCo ground truth. This asks the question that matters for the hardware: what
does it draw when the policy is driving on `hw/odometry.py`'s estimate instead,
which is what the Pi will actually hand it?

Stationary drift is the sharpest version of that. A hold command should draw a
dot. Anything larger is the balance gait wandering, and the estimate feeding
the policy decides how far it wanders.

Modes come from `aow_sim.sim_odometry.MODES`, plus `truth` for the baseline.
Measured 2026-08-26, only `truth` and `lon_only` survive a hold at all: `front`
and `lat_only` fall within a couple of seconds, so their traces stop where the
bike does.

  python analysis/pen_odometry.py
  python analysis/pen_odometry.py --seconds 20 --tag long

Writes analysis/plots/pen_odometry_<tag>.png and prints the drift table.
Read-only: loads moves/*.npz and changes nothing else.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import extract_state
from aow_sim.control.drive import DriveController
from aow_sim.control.linearize import settle_upright
from aow_sim.sim_odometry import MODES, SimOdometry

FALL_DEG = 60.0


def _plots_dir() -> Path:
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(parents=True, exist_ok=True)
    return d


def episode(model, params, eq, mode, vcmd, seconds, name):
    dt = model.opt.timestep
    data = mujoco.MjData(model)
    data.qpos[:] = eq
    mujoco.mj_forward(model, data)
    ctl = DriveController(params, model)
    ctl.reset(model, data)
    ctl.engage_general(data, name=name)
    odo = None if mode == "truth" else SimOdometry(model, params, mode=mode)
    xy, yaw, fell = [], [], None
    for k in range(int(seconds / dt)):
        ctl.set_command(v_cmd_world=vcmd)
        if odo is None:
            ctl.step(model, data)
        else:
            with odo.estimated(data, dt):
                ctl.step(model, data)
        mujoco.mj_step(model, data)
        s = extract_state(data, np.zeros(3))
        if abs(s.roll) > np.radians(FALL_DEG):
            fell = k * dt
            break
        if k % 25 == 0:
            xy.append(data.qpos[:2].copy())
            yaw.append(s.yaw)
    return np.array(xy), np.array(yaw), fell


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--modes", nargs="+",
                    default=["truth", "lon_only", "front", "lat_only"])
    ap.add_argument("--tag", default="hold",
                    help="figure suffix; every tracked figure must be "
                         "reproducible at the name it is tracked under")
    args = ap.parse_args()

    params = load_params()
    model = build_model(params, variant="full")
    eq = settle_upright(model).qpos.copy()
    name = params["control"].get("general_move", "general_rl")
    for m in args.modes:
        if m != "truth" and m not in MODES:
            raise SystemExit(f"unknown mode {m!r}; have truth + {tuple(MODES)}")

    fig, axes = plt.subplots(1, len(args.modes),
                             figsize=(4.2 * len(args.modes), 4.4), squeeze=False)
    print(f"  holding station for {args.seconds:g} s, policy {name}\n")
    print(f"  {'mode':10} {'final drift':>12} {'max excursion':>14} "
          f"{'path length':>12}   outcome")
    for ax, mode in zip(axes[0], args.modes):
        xy, yaw, fell = episode(model, params, eq, mode, (0.0, 0.0),
                                args.seconds, name)
        r = np.linalg.norm(xy - xy[0], axis=1) if len(xy) else np.array([0.0])
        step = np.linalg.norm(np.diff(xy, axis=0), axis=1).sum() if len(xy) > 1 else 0.0
        out = f"FELL at {fell:.2f}s" if fell else "held"
        print(f"  {mode:10} {r[-1]*1000:9.1f} mm {r.max()*1000:11.1f} mm "
              f"{step*1000:9.1f} mm   {out}")
        ax.plot(xy[:, 0] * 1000, xy[:, 1] * 1000, lw=1.0)
        ax.plot([0], [0], "k+", ms=9)
        ax.set_title(f"{mode}\n{out}", fontsize=10)
        ax.set_xlabel("x [mm]")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
    axes[0][0].set_ylabel("y [mm]")
    fig.suptitle(f"stationary drift with odometry in the loop — {name}", y=0.99)
    fig.tight_layout()
    out = _plots_dir() / f"pen_odometry_{args.tag}.png"
    fig.savefig(out, dpi=130)
    print(f"\n  wrote {out}")


if __name__ == "__main__":
    main()
