"""Estimated velocity against truth against command, as a time trace.

`pen_odometry.py` showed that driving on an estimated v_lon holds the bike up
but drifts ~50% further than truth over a 12 s hold. A drift is an integral, so
the question it raises is about the SHAPE of the velocity error: a constant
bias integrates linearly and would be trivially removable, while a zero-mean
error that merely lags does not integrate at all and would need something else
entirely. RMS cannot tell those apart. This can.

Both channels are drawn because they fail differently: v_lat is what drops the
bike (measured 2026-08-26), v_lon is what moves it off the mark.

  python analysis/odometry_trace.py
  python analysis/odometry_trace.py --command 0.6 --tag fwd

Writes analysis/plots/odometry_trace_<tag>.png and prints the error stats.
Read-only.
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
from aow_sim.sim_odometry import SimOdometry


def _plots_dir() -> Path:
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(parents=True, exist_ok=True)
    return d


def lag_samples(err, ref, max_lag=200):
    """Cross-correlation peak: does the estimate merely LAG the truth?"""
    a = ref - ref.mean()
    best, arg = -np.inf, 0
    for L in range(-max_lag, max_lag + 1):
        b = np.roll(err - err.mean(), L)
        c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
        if c > best:
            best, arg = c, L
    return arg, best


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--command", type=float, default=0.0, help="v_lon command")
    ap.add_argument("--lat-command", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--mode", default="lon_only")
    ap.add_argument("--tag", default="hold")
    args = ap.parse_args()

    params = load_params()
    model = build_model(params, variant="full")
    eq = settle_upright(model).qpos.copy()
    dt = model.opt.timestep
    name = params["control"].get("general_move", "general_rl")

    data = mujoco.MjData(model)
    data.qpos[:] = eq
    mujoco.mj_forward(model, data)
    ctl = DriveController(params, model)
    ctl.reset(model, data)
    ctl.engage_general(data, name=name)
    odo = SimOdometry(model, params, mode=args.mode)

    vcmd = (args.command, args.lat_command)
    t, est, tru, cmd = [], [], [], []
    for k in range(int(args.seconds / dt)):
        ctl.set_command(v_cmd_world=vcmd)
        s = extract_state(data, np.zeros(3))
        e_lon, e_lat = odo.update(data, dt)
        with odo.estimated(data, dt):
            ctl.step(model, data)
        mujoco.mj_step(model, data)
        if abs(s.roll) > np.radians(60):
            print(f"  fell at {k*dt:.2f}s — trace truncated")
            break
        if k % 10 == 0:
            t.append(k * dt)
            est.append((e_lon, e_lat))
            tru.append((s.v_lon, s.v_lat))
            cmd.append(vcmd)
    t = np.array(t); est = np.array(est); tru = np.array(tru); cmd = np.array(cmd)

    print(f"\n  policy {name}, mode {args.mode}, command {vcmd}\n")
    print(f"  {'channel':8} {'bias mm/s':>10} {'RMS mm/s':>10} {'corr':>7} "
          f"{'lag ms':>8}  (lag>0: estimate TRAILS truth)")
    for i, ch in enumerate(("v_lon", "v_lat")):
        err = est[:, i] - tru[:, i]
        L, c = lag_samples(est[:, i], tru[:, i])
        print(f"  {ch:8} {err.mean()*1000:10.2f} "
              f"{np.sqrt(np.mean(err**2))*1000:10.2f} "
              f"{np.corrcoef(est[:, i], tru[:, i])[0,1]:7.3f} "
              f"{L*10*dt*1000:8.1f}")

    fig, axes = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True)
    for ax, i, ch in zip(axes, (0, 1), ("v_lon", "v_lat")):
        ax.plot(t, tru[:, i] * 1000, lw=1.4, label=f"{ch} true")
        ax.plot(t, est[:, i] * 1000, lw=1.0, alpha=0.85, label=f"{ch} estimated")
        ax.plot(t, cmd[:, i] * 1000, "k--", lw=0.9, label=f"{ch} commanded")
        ax.axhline(0, color="0.6", lw=0.6)
        ax.set_ylabel(f"{ch} [mm/s]")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(alpha=0.3)
    axes[1].set_xlabel("t [s]")
    fig.suptitle(f"onboard estimate vs truth vs command — {name}, "
                 f"mode {args.mode}, cmd {vcmd}")
    fig.tight_layout()
    out = _plots_dir() / f"odometry_trace_{args.tag}.png"
    fig.savefig(out, dpi=130)
    print(f"\n  wrote {out}")


if __name__ == "__main__":
    main()
