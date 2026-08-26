"""The pen, with the ONBOARD VELOCITY ESTIMATE in the loop.

`pen.py` draws what the bike traces on the floor when a policy drives it on
MuJoCo ground truth. This asks the question that matters for the hardware: what
does it draw when the policy is driving on `hw/odometry.py`'s estimate instead,
which is what the Pi will actually hand it?

Stationary drift is the sharpest version of that. A hold command should draw a
dot. Anything larger is the balance gait wandering, and the estimate feeding
the policy decides how far it wanders.

Modes come from `aow_sim.sim_odometry.MODES`, plus `truth` for the baseline.
`--encoder` selects the encoder model, and more than one draws a row each.

WHICH MODES SURVIVE IS A PROPERTY OF THE POLICY, NOT OF THIS SCRIPT. An earlier
version of this docstring said flatly that "only `truth` and `lon_only` survive
a hold": that was measured with `general_rl_smooth_diff_pi`, which is trained on
MuJoCo truth. `general_rl_odo` holds in EVERY mode and on both encoders.

AND READ THE PATH LENGTH NEXT TO THE DRIFT. They say different things, and the
policy that looks worse on one can look better on the other. Holding 12 s on
truth, 2026-08-27:

    policy                      final drift   path length
    general_rl_odo                 764.0 mm       770.3 mm   creeps in a line
    general_rl_smooth_diff_pi      341.8 mm      1073.9 mm   jitters, stays put
    general_rl_nolat               273.1 mm       329.8 mm   quiet AND still

`general_rl_odo` has nearly the SHORTEST path and the LARGEST drift -- it is
not wandering, it is driving away at a steady -64 mm/s on a zero command. That
is a steady-state velocity error in the policy and it is visible on TRUTH, with
no estimator in the loop, so it is not an odometry artifact. Confirmed by
reading the two side by side: on truth the policy OBSERVES -64.2 mm/s and does
not correct it.

ONE HOLD IS NOT AN EVAL. Every policy survives this command; the truth-trained
one still scores 0.010 with survival 0.05 on the 20-command grid
(`analysis/chatter.py --force-odometry`). Use this to see the SHAPE of a path,
and the grid to decide anything.

  python analysis/pen_odometry.py
  python analysis/pen_odometry.py --policy general_rl_odo \
      --encoder ideal counts --tag odo_encoder

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
from aow_sim.sim_odometry import ENCODERS, MODES, SimOdometry

FALL_DEG = 60.0


def _plots_dir() -> Path:
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(parents=True, exist_ok=True)
    return d


def episode(model, params, eq, mode, vcmd, seconds, name, encoder="ideal"):
    dt = model.opt.timestep
    data = mujoco.MjData(model)
    data.qpos[:] = eq
    mujoco.mj_forward(model, data)
    ctl = DriveController(params, model)
    ctl.reset(model, data)
    ctl.engage_general(data, name=name)
    odo = (None if mode == "truth"
           else SimOdometry(model, params, mode=mode, encoder=encoder))
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
    ap.add_argument("--encoder", nargs="+", default=["ideal"],
                    choices=list(ENCODERS),
                    help="encoder model(s) to draw. Given more than one, every "
                         "mode is drawn once per encoder -- which is how you "
                         "see whether quantisation and the RateFilter lag move "
                         "the PATH, a thing no scalar metric can show.")
    ap.add_argument("--policy", default=None,
                    help="move to drive (default: control.general_move). Use "
                         "general_rl_odo to ask the question of a policy that "
                         "was actually TRAINED on the estimate.")
    ap.add_argument("--tag", default="hold",
                    help="figure suffix; every tracked figure must be "
                         "reproducible at the name it is tracked under")
    args = ap.parse_args()

    params = load_params()
    model = build_model(params, variant="full")
    eq = settle_upright(model).qpos.copy()
    name = args.policy or params["control"].get("general_move", "general_rl")
    for m in args.modes:
        if m != "truth" and m not in MODES:
            raise SystemExit(f"unknown mode {m!r}; have truth + {tuple(MODES)}")

    encs = list(args.encoder)
    fig, axes = plt.subplots(len(encs), len(args.modes),
                             figsize=(4.2 * len(args.modes), 4.4 * len(encs)),
                             squeeze=False)
    print(f"  holding station for {args.seconds:g} s, policy {name}\n")
    print(f"  {'encoder':9} {'mode':10} {'final drift':>12} "
          f"{'max excursion':>14} {'path length':>12}   outcome")
    for row, enc in zip(axes, encs):
        for ax, mode in zip(row, args.modes):
            xy, yaw, fell = episode(model, params, eq, mode, (0.0, 0.0),
                                    args.seconds, name, encoder=enc)
            r = (np.linalg.norm(xy - xy[0], axis=1) if len(xy)
                 else np.array([0.0]))
            step = (np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()
                    if len(xy) > 1 else 0.0)
            out = f"FELL at {fell:.2f}s" if fell else "held"
            # `truth` has no encoder -- say so rather than printing a model it
            # did not use.
            shown = "-" if mode == "truth" else enc
            print(f"  {shown:9} {mode:10} {r[-1]*1000:9.1f} mm "
                  f"{r.max()*1000:11.1f} mm {step*1000:9.1f} mm   {out}")
            ax.plot(xy[:, 0] * 1000, xy[:, 1] * 1000, lw=1.0)
            ax.plot([0], [0], "k+", ms=9)
            ax.set_title(f"{mode} · {shown}\n{out}", fontsize=10)
            ax.set_xlabel("x [mm]")
            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
        row[0].set_ylabel("y [mm]")
    fig.suptitle(f"stationary drift with odometry in the loop — {name}", y=0.99)
    fig.tight_layout()
    out = _plots_dir() / f"pen_odometry_{args.tag}.png"
    fig.savefig(out, dpi=130)
    print(f"\n  wrote {out}")


if __name__ == "__main__":
    main()
