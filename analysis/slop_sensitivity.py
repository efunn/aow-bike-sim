"""How much do drivetrain-trained policies care about the roller slop's two
GUESSes, `drivetrain_model.roller_slop.centring_stiffness` and `.damping`?

The eval grid of analysis/drivetrain_eval.py (20 commands, fixed seeds,
`--encoder counts --ahrs tm151 --ahrs-tau 0.19`), each value x0.1 and x10,
the slop removed, and x0.99 / x1.01 on the stiffness as the noise floor: a
1% change cannot matter physically, so what it moves is the rollouts' chaos.

    python analysis/slop_sensitivity.py
    python analysis/slop_sensitivity.py --policies general_rl_smooth_temporal

Read-only: prints, writes nothing.
"""
from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drivetrain_eval as de  # noqa: E402
from aow_sim.params import load_params  # noqa: E402

_R = load_params()["drivetrain_model"]["roller_slop"]
K, C = float(_R["centring_stiffness"]), float(_R["damping"])
SLOP = {
    "nominal": dict(),
    "k_x0.99": dict(overrides={"roller_slop": {"centring_stiffness": K * 0.99}}),
    "k_x1.01": dict(overrides={"roller_slop": {"centring_stiffness": K * 1.01}}),
    "k_x0.1": dict(overrides={"roller_slop": {"centring_stiffness": K * 0.1}}),
    "k_x10": dict(overrides={"roller_slop": {"centring_stiffness": K * 10}}),
    "c_x0.1": dict(overrides={"roller_slop": {"damping": C * 0.1}}),
    "c_x10": dict(overrides={"roller_slop": {"damping": C * 10}}),
    "no_slop": dict(without=("roller_slop",)),
}
# At import, so the worker processes (spawned, re-importing this module) see
# them too.
de.VARIANTS.update(SLOP)


def one(job):
    return de.one(job)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policies", nargs="+", default=[
        "general_rl_smooth_temporal", "general_rl_drivetrain_p100_1"])
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    args = ap.parse_args()
    jobs = [(n, v, "counts", "tm151", 0.19) for n in args.policies for v in SLOP]
    out = {}
    with ProcessPoolExecutor(max_workers=min(len(jobs), args.workers)) as ex:
        for name, variant, m, rows, _rate in ex.map(one, jobs):
            out[(name, variant)] = (m, rows)
    print(f"slop nominal: centring_stiffness {K:g} N m/rad, damping {C:g} N m s/rad")
    print(f"{'policy':<30s} {'variant':<9s} {'score':>6s} {'surv':>5s} "
          f"{'trk_geo':>7s} {'head':>6s}  fell on")
    for n in args.policies:
        for v in SLOP:
            m, rows = out[(n, v)]
            fell = [f"{c[0]:+.2f},{c[1]:+.2f},{c[2]:+d}" for c in
                    (r["cmd"] for r in rows if r["fell"])]
            print(f"{n:<30s} {v:<9s} {de._score(m):6.3f} {m['survive_rate']:5.2f} "
                  f"{m['track_geo']:7.3f} {m['head_err_deg']:6.1f}  "
                  f"{' '.join(fell) or '-'}")
        print()


if __name__ == "__main__":
    main()
