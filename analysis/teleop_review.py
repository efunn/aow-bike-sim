"""Review a recorded teleop session: what you asked for vs what the bike did.

Written for the question "what was my control strategy", not "did it track" --
so the KEY EVENTS are drawn on every panel. A heading trace that never settles
looks like a broken controller until you can see that another 180 snap arrived
before the last one converged, which is an operator strategy rather than a
plant failure.

  python -m aow_sim.run_drive --teleop --record        # produces the trace
  python analysis/teleop_review.py traces/teleop/teleop_<stamp>.npz
  python analysis/teleop_review.py <path> --from 12 --to 25   # zoom a window

Writes <trace>.png beside the trace and prints a per-key tally. Read-only
otherwise: it loads one npz and touches nothing else.
"""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Keys worth a marker. Everything else is tallied but not drawn, so a session
# full of throttle taps does not bury the three snaps that shaped it.
_MARKED = {"snap 180": "tab:red", "snap +90": "tab:orange",
           "snap -90": "tab:purple", "stop": "tab:gray",
           "crab left": "tab:green", "crab right": "tab:olive"}


def load(path):
    z = np.load(path, allow_pickle=True)
    cols = {n: i for i, n in enumerate(z["columns"].tolist())}
    return z, z["rows"], cols


def unwrap_deg(a):
    return np.degrees(np.unwrap(a))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trace")
    ap.add_argument("--from", dest="t0", type=float, default=None)
    ap.add_argument("--to", dest="t1", type=float, default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    z, rows, C = load(args.trace)
    t = rows[:, C["t"]]
    keep = np.ones_like(t, bool)
    if args.t0 is not None:
        keep &= t >= args.t0
    if args.t1 is not None:
        keep &= t <= args.t1
    rows, t = rows[keep], t[keep]
    ev_t, ev_lab = z["event_t"], z["event_label"]
    ev_keep = (ev_t >= t[0]) & (ev_t <= t[-1])
    ev_t, ev_lab = ev_t[ev_keep], ev_lab[ev_keep]

    print(f"{Path(args.trace).name}")
    print(f"  policy   {z['policy']}")
    print(f"  digest   {z['params_digest']}  (matches config? check load warnings)")
    print(f"  {t[-1] - t[0]:.1f} s, {len(rows)} rows at {float(z['rate_hz']):.0f} Hz, "
          f"{len(ev_t)} key events")
    tally = Counter(ev_lab.tolist())
    if tally:
        print("  keys: " + "  ".join(f"{k}x{v}" for k, v in tally.most_common()))

    # How hard were the heading commands stacked? This is the number that
    # describes "180s faster than it can turn".
    snaps = ev_t[np.isin(ev_lab, ["snap 180", "snap +90", "snap -90"])]
    if len(snaps) > 1:
        gaps = np.diff(snaps)
        print(f"  heading snaps: {len(snaps)}, gap min {gaps.min():.2f} s / "
              f"median {np.median(gaps):.2f} s")
        err = np.abs(np.degrees(np.arctan2(
            np.sin(rows[:, C["cmd_psi"]] - rows[:, C["yaw"]]),
            np.cos(rows[:, C["cmd_psi"]] - rows[:, C["yaw"]]))))
        settled = err < 5.0
        print(f"  heading within 5 deg of command {100 * settled.mean():.0f}% of the time")

    fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True,
                           gridspec_kw={"height_ratios": [2, 1.4, 1.4, 1.2]})

    ax[0].plot(t, unwrap_deg(rows[:, C["cmd_psi"]]), color="tab:red", lw=1.6,
               label="commanded heading")
    ax[0].plot(t, unwrap_deg(rows[:, C["yaw"]]), color="tab:blue", lw=1.4,
               label="actual heading")
    ax[0].set_ylabel("heading [deg]\n(unwrapped)")
    ax[0].legend(loc="upper left", fontsize=8)
    ax[0].set_title(f"{Path(args.trace).stem}   ·   {z['policy']}", fontsize=10)

    ax[1].plot(t, rows[:, C["cmd_v"]], color="tab:red", lw=1.4, label="cmd v_lon")
    ax[1].plot(t, rows[:, C["v_lon"]], color="tab:blue", lw=1.2, label="actual v_lon")
    ax[1].plot(t, rows[:, C["cmd_v_lat"]], color="tab:orange", lw=1.2, ls="--",
               label="cmd v_lat")
    ax[1].plot(t, rows[:, C["v_lat"]], color="tab:cyan", lw=1.0, ls="--",
               label="actual v_lat")
    ax[1].set_ylabel("body velocity\n[m/s]")
    ax[1].legend(loc="upper left", fontsize=8, ncol=2)

    ax[2].plot(t, np.degrees(rows[:, C["roll"]]), color="tab:blue", lw=1.2, label="roll")
    ax[2].plot(t, np.degrees(rows[:, C["pitch"]]), color="tab:green", lw=1.0, label="pitch")
    ax[2].plot(t, np.degrees(rows[:, C["steer"]]), color="tab:brown", lw=1.0, label="steer")
    ax[2].set_ylabel("attitude\n[deg]")
    ax[2].legend(loc="upper left", fontsize=8, ncol=3)

    ax[3].plot(t, rows[:, C["ctrl_a"]], lw=0.9, label="drive_a cmd")
    ax[3].plot(t, rows[:, C["ctrl_b"]], lw=0.9, label="drive_b cmd")
    ax[3].set_ylabel("input shaft\n[rad/s]")
    ax[3].set_xlabel("time [s]")
    ax[3].legend(loc="upper left", fontsize=8)

    seen = set()
    for a in ax:
        for te, lab in zip(ev_t, ev_lab):
            col = _MARKED.get(str(lab))
            if col is None:
                continue
            a.axvline(te, color=col, lw=1.0, alpha=0.55,
                      label=lab if lab not in seen else None)
            seen.add(lab)
        a.grid(alpha=0.25)
    if seen:
        ax[0].legend(loc="upper left", fontsize=8, ncol=2)

    out = Path(args.out) if args.out else Path(args.trace).with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
