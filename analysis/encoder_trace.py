"""SHOW the encoder model working, sample by sample.

`SimOdometry(encoder=...)` claims to reproduce the hardware path: quantise the
shaft to 4096 counts/rev, difference it over the tick, filter. Tests assert
that; this DRAWS it, because a staircase either looks like a staircase or the
model is not doing what it says.

Four rows, all against the same trajectory:

  1. POSITION -- the true input-shaft angle against the quantised count, in
     wheel-travel millimetres. The staircase step is the whole claim: one count
     is 0.236 mm at the wheel (2*pi/4096 at the servo, times belt_ratio 3.0,
     times the 0.0512 m rolling radius).
  2. RAW DIFFERENCE -- that staircase differenced over one tick, unfiltered.
     This is the quantisation noise before anything smooths it, and at a 10 ms
     tick one count is worth 23.6 mm/s.
  3. FILTERED -- what each encoder model actually hands the estimator, against
     truth. `counts` is our 25 ms / taper 0.5; `reported` is the XC430's own
     Present Velocity, a ~50 ms boxcar.
  4. ERROR against truth, so lag and noise can be told apart: lag shows up as
     a shape that follows the signal late, noise as spread around zero.

  python analysis/encoder_trace.py
  python analysis/encoder_trace.py --v 0.0 --tag rest

Writes analysis/plots/encoder_trace_<tag>.png. Read-only.
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
from aow_sim.control.steer import XC330_COUNTS_PER_RAD
from aow_sim.sim_odometry import ENCODER_FILTER, SimOdometry

ENCS = ("ideal", "counts", "reported")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--v", type=float, default=0.6, help="hold speed [m/s]")
    ap.add_argument("--seconds", type=float, default=2.5)
    ap.add_argument("--policy", default="general_rl_odo")
    ap.add_argument("--tag", default="drive")
    args = ap.parse_args()

    params = load_params()
    model = build_model(params, variant="full")
    dt = model.opt.timestep
    r_wheel = params["omni_wheel"]["outer_radius"]

    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    mujoco.mj_forward(model, data)
    ctl = DriveController(params, model)
    ctl.reset(model, data)
    ctl.engage_general(data, name=args.policy)
    ctl.set_command(v_cmd_world=(args.v, 0.0), psi_cmd=0.0)

    # ONE trajectory, every encoder ticked on it. Separate runs would diverge
    # the moment the estimate reaches the controller.
    odos = {e: SimOdometry(model, params, encoder=e) for e in ENCS}
    drive = odos["counts"]          # the one whose estimate closes the loop
    belt = drive.est.belt_ratio
    arc = (2 * np.pi / 4096) * belt * r_wheel        # metres per count

    t, true_pos, q_pos, raw, filt, truth_v = [], [], [], {e: [] for e in ENCS}, \
        {e: [] for e in ENCS}, []
    prev_c, prev_t = None, None
    for k in range(int(args.seconds / dt)):
        with drive.estimated(data, dt):
            ctl.step(model, data)
        mujoco.mj_step(model, data)
        for o in odos.values():
            if o is not drive:
                o.update(data, dt)
        # Sample on the ESTIMATOR's clock, which is what it actually sees.
        if k % int(round((1.0 / drive.odo_hz) / dt)) == 0:
            t.append(k * dt)
            rad_in = float(drive._read(data, "input_a_pos")[0])
            c = int(round(rad_in / belt * XC330_COUNTS_PER_RAD))
            true_pos.append(rad_in / belt * XC330_COUNTS_PER_RAD * arc)
            q_pos.append(c * arc)
            tick = 1.0 / drive.odo_hz
            raw["counts"].append(
                ((c - prev_c) * arc / tick) if prev_c is not None else 0.0)
            prev_c = c
            # BODY v_lon, not world x. They agree only at zero heading, and
            # the bike yaws while it balances.
            truth_v.append(float(extract_state(data, np.zeros(3)).v_lon))
            for e, o in odos.items():
                filt[e].append(o._last[0])

    t = np.array(t)
    fig, ax = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    col = {"ideal": "tab:green", "counts": "tab:blue", "reported": "tab:red"}

    n = min(len(t), 60)                     # first 0.6 s: the staircase is visible
    ax[0].step(t[:n], np.array(q_pos[:n]) * 1000, where="post",
               color="tab:blue", lw=1.4, label="quantised counts")
    ax[0].plot(t[:n], np.array(true_pos[:n]) * 1000, "k--", lw=1.0,
               label="true shaft angle")
    ax[0].set_ylabel("travel at wheel [mm]")
    ax[0].set_title(f"1. POSITION — one count = {arc*1000:.4f} mm at the wheel "
                    f"(first {n} ticks)", fontsize=10)
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)

    ax[1].plot(t, np.array(raw["counts"]) * 1000, color="tab:gray", lw=0.8)
    ax[1].plot(t, np.array(truth_v) * 1000, "k--", lw=1.2, label="truth")
    ax[1].set_ylabel("v_lon [mm/s]")
    ax[1].set_title(f"2. RAW DIFFERENCE, unfiltered — one count over a "
                    f"{1000/odos['counts'].odo_hz:.0f} ms tick is "
                    f"{arc*odos['counts'].odo_hz*1000:.1f} mm/s", fontsize=10)
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)

    for e in ENCS:
        w = ENCODER_FILTER.get(e)
        lab = (f"{e} ({w[0]:.0f} ms, taper {w[1]})" if w
               else f"{e} (instantaneous)")
        ax[2].plot(t, np.array(filt[e]) * 1000, color=col[e], lw=1.1, label=lab)
    ax[2].plot(t, np.array(truth_v) * 1000, "k--", lw=1.2, label="truth")
    ax[2].set_ylabel("v_lon [mm/s]")
    ax[2].set_title("3. FILTERED — what each model hands the estimator", fontsize=10)
    ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)

    for e in ENCS:
        err = (np.array(filt[e]) - np.array(truth_v)) * 1000
        ax[3].plot(t, err, color=col[e], lw=1.0,
                   label=f"{e}: RMS {np.sqrt(np.mean(err**2)):.1f} mm/s")
    ax[3].axhline(0, color="k", lw=0.8, ls="--")
    ax[3].set_ylabel("error [mm/s]"); ax[3].set_xlabel("time [s]")
    ax[3].set_title("4. ERROR vs truth — lag follows the signal late, "
                    "noise spreads about zero", fontsize=10)
    ax[3].legend(fontsize=8); ax[3].grid(alpha=0.3)

    fig.suptitle(f"encoder models against one trajectory — {args.policy}, "
                 f"v={args.v} m/s", y=0.995)
    fig.tight_layout()
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(parents=True, exist_ok=True)
    out = d / f"encoder_trace_{args.tag}.png"
    fig.savefig(out, dpi=130)

    print(f"  one count = {arc*1000:.4f} mm at the wheel "
          f"= {arc*odos['counts'].odo_hz*1000:.1f} mm/s over one tick\n")
    print(f"  {'encoder':10}{'filter':>18}{'lag':>8}{'RMS err':>10}{'spread':>9}")
    for e in ENCS:
        err = (np.array(filt[e]) - np.array(truth_v)) * 1000
        w = ENCODER_FILTER.get(e)
        f0 = list(odos[e]._filt.values())[0]
        print(f"  {e:10}{(f'{w[0]:.0f} ms taper {w[1]}' if w else 'none'):>18}"
              f"{f0.group_delay_ms if w else 0.0:>7.1f}ms"
              f"{np.sqrt(np.mean(err**2)):>9.1f}{np.std(err):>9.1f}")
    print(f"\n  wrote {out}")


if __name__ == "__main__":
    main()
