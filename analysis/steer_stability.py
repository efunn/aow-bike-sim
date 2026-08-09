"""Does parking the steer near 90 deg actually buy balance authority?

`general_rl_smooth_bouncy_lat` (12M steps) parks its steer at +86.5 deg and
spends 62% of a rollout within 30 deg of +90, while its forward speed ratio
sits at 0.06. Two readings of that, and they imply opposite fixes:

  A. The bike really IS more stable with the rear omni turned across the
     travel direction -- lateral ground force is then available directly,
     instead of via lean-and-steer -- and the policy found real physics. The
     fix is then in the OBJECTIVE (`_score = survive_rate x track` pays for
     survival and never charges for refusing a direction).
  B. Steer angle at rest is simply unpenalised (general-rl-improvements.md
     2.2), the objective is flat in it, and 90 deg is drift into that flat
     region rather than a discovery. The fix is the same term, for a
     different reason -- but nothing about the physics is interesting.

This tells them apart by measuring the RECOVERABLE SET as a function of where
the steer is parked: the largest initial roll angle the controller still
recovers from, at each steer offset. If A, the boundary widens toward +-90.

SYMMETRY, AND WHY THE SWEEP IS TWO-SIDED. The plant is mirror-symmetric
(`axle_cant_deg` 0, measured), so steer +x and -x are physically equivalent
and any left/right difference here is the POLICY's handedness, not the bike's
-- which is exactly why both sides are swept rather than one being assumed.
Note also that the observation encodes steer as (sin 2t, cos 2t), so +90 and
-90 are the SAME observation and the policy cannot distinguish them; a
difference between those two columns is therefore a difference in the plant
state the policy is reacting to, not a preference it can express directly.

    python analysis/steer_stability.py                  # both damping ratios
    python analysis/steer_stability.py --dampratio 0.5  # just the current one
    python analysis/steer_stability.py --move general_rl_smooth_stiff

Read-only with respect to the repo: loads moves/*.npz and config, writes one
PNG next to itself. It does NOT modify bike_params.yaml -- the damping ratio
is overridden in the loaded dict, in memory only.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from aow_sim.build_model import load_params

import no_return as nr


def critical_roll(v0: float, steer_deg: float, side: int,
                  ceiling: float = 60.0) -> float:
    """Largest initial roll [deg] on `side` still recovered, at this steer.

    Coarse scan up to the first failure, then bisect -- the same shape as
    no_return._cold_point, deliberately, so the numbers are comparable to the
    ones that tool reports at steer 0. Returns the contiguous boundary only
    (no recovery-island reporting); islands are noted in that tool and are not
    what this sweep is asking about.
    """
    steer = np.deg2rad(steer_deg)
    lo, hi = 0.0, None
    theta = nr.COARSE_DEG
    while theta <= ceiling:
        ok, _ = nr._rollout(v0, side * np.deg2rad(theta), 0.0, steer=steer)
        if not ok:
            hi = theta
            break
        lo = theta
        theta += nr.COARSE_DEG
    if hi is None:
        return ceiling
    while hi - lo > nr.BISECT_DEG:
        mid = 0.5 * (lo + hi)
        ok, _ = nr._rollout(v0, side * np.deg2rad(mid), 0.0, steer=steer)
        if ok:
            lo = mid
        else:
            hi = mid
    return lo


def sweep(params, move, steers, speed, dampratio):
    params = {**params, "sim": {**params["sim"],
                                "contact_solref": [params["sim"]["contact_solref"][0],
                                                   dampratio]}}
    nr._init(params, "general", move, [speed])
    out = {}
    for s in steers:
        out[s] = (critical_roll(speed, s, +1), critical_roll(speed, s, -1))
        print(f"    steer {s:+6.1f} deg   roll_crit  right {out[s][0]:5.2f}   "
              f"left {out[s][1]:5.2f}   (deg)")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--move", default="general_rl_smooth_bouncy_lat")
    ap.add_argument("--speed", type=float, default=0.0,
                    help="forward speed [m/s]; 0 = standstill, the hardest case")
    ap.add_argument("--dampratio", type=float, nargs="+", default=[0.5, 1.0],
                    help="contact_solref[1] values to compare")
    ap.add_argument("--steers", type=float, nargs="+",
                    default=[-90, -60, -30, 0, 30, 60, 90])
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).with_suffix(".png"))
    args = ap.parse_args()

    params = load_params()
    results = {}
    for dr in args.dampratio:
        print(f"\ndampratio {dr}  (move {args.move}, v={args.speed} m/s)")
        results[dr] = sweep(params, args.move, args.steers, args.speed, dr)

    _plot(args.out, results, args.steers, args.move, args.speed)
    print(f"\nwrote {args.out}")


def _plot(out: Path, results, steers, move, speed) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 4.5))
    for dr, marker in zip(sorted(results), ("o", "s")):
        r = results[dr]
        for side, idx, ls in ((" right", 0, "-"), ("left", 1, "--")):
            ax.plot(steers, [r[s][idx] for s in steers], marker=marker, ls=ls,
                    label=f"dampratio {dr}, {side.strip()}")
    ax.set_xlabel("steer angle parked at [deg]")
    ax.set_ylabel("largest recoverable initial roll [deg]")
    ax.set_title(f"Recoverable set vs steer offset\n{move}, v={speed} m/s")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=130)


if __name__ == "__main__":
    main()
