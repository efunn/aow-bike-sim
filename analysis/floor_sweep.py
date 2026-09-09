"""Which floor should the bike drive on?

Plan and rationale: docs/plans/floors-and-the-contact-model.md

Sweeps the two contact properties a real floor actually varies -- sliding
friction and contact compliance -- and reports what the eval grid does at each
point, for whichever policies you name. It answers "how much does the choice of
surface matter at all", which is the question that decides whether the surface
is worth agonising over or whether any hard floor will do.

WHY BOTH AXES, AND WHY THEY ARE NOT SYMMETRIC.

  friction_sliding   is measured by `contact-protocol.md` P0b in minutes -- tilt
                     a board until the wheel slides, mu = tan(theta). Every
                     trained policy HAS seen variation here: `friction_frac 0.2`
                     spans 0.72-1.08 around the shipped 0.9.

  contact_solref     is the compliance, and it is the dangerous axis. NO trained
                     policy has ever seen it vary -- `solref_frac` and
                     `dampratio_range` sit commented out in rl_general.yaml --
                     and eval survival is known to fall off a cliff just below
                     nominal. A carpet or a foam mat moves THIS, not just mu.

So a friction column that looks flat is a genuine "pick whichever floor is
convenient"; a dampratio column that looks flat is only ever "this policy
happened to cope", because nothing trained it to.

The sweep is NOT a substitute for measuring. It tells you the SHAPE of the
sensitivity; P0b and P1 tell you where your actual candidate floors sit on it.
Run this first to find out how carefully you need to measure, then measure.

    python analysis/floor_sweep.py                       # default policy, both axes
    python analysis/floor_sweep.py --policies general_rl_odo_ahrs general_rl_odo_ahrs_rand2
    python analysis/floor_sweep.py --mu 0.4 0.9 1.4 --dampratio 0.3 0.5 1.0

Read-only: it never writes a config and never touches moves/.
"""
from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aow_sim.build_model import load_params
from aow_sim.control.general_env import _load_rl_config
from aow_sim.train_general_rl import _eval_episodes, eval_cmds
from rsa_policies import REPO, env_for, load_general

# Plausible floors, as sliding-friction coefficients. Bracketing values, NOT
# claims about any particular surface -- P0b is what turns a material into a
# number. The shipped GUESS is 0.9.
MU_DEFAULT = [0.5, 0.7, 0.9, 1.1, 1.4]
# Contact damping ratio. 1.0 ships. **LOWER IS STIFFER, NOT SOFTER** -- this is
# the single easiest thing to get backwards in this file, and getting it
# backwards inverts every conclusion drawn from the sweep.
#
#   b = 2 / (d_width * timeconst)                       <- damping
#   k = d(r) / (d_width^2 * timeconst^2 * dampratio^2)  <- stiffness
#
# `dampratio` appears ONLY in k, as 1/dampratio^2. So dropping it 1.0 -> 0.3
# does not reduce damping, it makes the contact ~11x STIFFER -- which is why
# the drop test bounces there. Measured sink at bike weight, via
# analysis/contact_calibration.static_curve:
#
#     dampratio  0.30    0.50    1.00 (ships)   2.00
#     sink       0.039   0.108   0.391 mm       1.075
#
# A SOFT FLOOR -- a mat, carpet -- is therefore HIGHER dampratio and/or higher
# timeconst, not lower. Spanning only <= 1.0 tests the stiff half and nothing
# else, so the default spans both sides of the shipped value.
DR_DEFAULT = [0.3, 0.5, 1.0, 2.0]

# Contact time constant [s]. The honest "softness" knob: it sets damping AND
# stiffness, and sink goes as timeconst^2, so raising it is unambiguously a
# softer, more damped contact. 0.005 ships; MuJoCo's stock default is 0.02.
TC_DEFAULT = [0.005]


def _cfg():
    """Eval config: randomization OFF, no ball. Matches per_command.py so the
    numbers here sit on the same scale as every other table in analysis/."""
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    cfg = {**cfg, "env": {**cfg["env"], "ball_prob": 0.0}}
    return cfg


def _sink_mm(timeconst, dampratio, weight_n=10.0):
    """Sink at bike weight [mm] -- the physically legible label for a solref.

    Reported beside every cell because `dampratio 0.3` reads as "softer" to
    almost everyone and is in fact 10x stiffer. A sink in millimetres cannot be
    misread the way a dimensionless ratio can.
    """
    from contact_calibration import static_curve, deflection_at
    # 12 mm, not 4: a soft contact ([0.020, 2.0]) sinks past 4 mm and
    # `deflection_at` returns nan off the end of its curve, which then sorts
    # the softest row to the BOTTOM of a table whose whole point is the order.
    return deflection_at(static_curve(timeconst, dampratio,
                                      np.linspace(0.0, 12.0, 120)), weight_n)


def _one_point(job):
    """One (policy, mu, timeconst, dampratio) cell. Own process -- a MuJoCo
    model does not pickle, so everything is rebuilt from strings and floats."""
    name, mu, tc, dr = job
    params = load_params()
    # `load_params` has already resolved every {value, source} node to a bare
    # float, so these are plain assignments -- writing a dict back here puts one
    # into `geom_friction` and MuJoCo rejects it at add_geom, several frames away
    # from the mistake.
    params["sim"]["friction_sliding"] = float(mu)
    params["sim"]["contact_solref"] = [float(tc), float(dr)]

    cfg = _cfg()
    pol = load_general(name)
    env = env_for(pol, params, cfg)
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n_act = env.action_space.shape[0]

    def act(o):
        return (np.asarray(pol.action(o), float) / scale)[:n_act]

    m, _rows = _eval_episodes(env, act, eval_cmds(cfg["env"]["v_max"]))
    track = m.get("track_geo", m.get("track", float("nan")))
    return {
        "policy": name, "mu": mu, "timeconst": tc, "dampratio": dr,
        "sink_mm": _sink_mm(tc, dr),
        "survive": m["survive_rate"],
        "track": track,
        "score": m["survive_rate"] * track,
        "drift_m": m.get("drift_m", float("nan")),
        "fwd": m.get("speed_ratio_fwd", float("nan")),
        "rev": m.get("speed_ratio_rev", float("nan")),
        # PER FAMILY, because the whole-grid number is a geometric mean over 20
        # commands and friction is expected to bite on ONE of them. `hold` is a
        # single episode and `crab` is two, so a friction effect confined there
        # moves the aggregate by almost nothing while being the entire story.
        # This is `_by_family`'s reason for existing, applied to the contact.
        "by_family": m.get("by_family", {}),
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policies", nargs="+", default=["general_rl_odo_ahrs"])
    ap.add_argument("--mu", nargs="+", type=float, default=MU_DEFAULT)
    ap.add_argument("--dampratio", nargs="+", type=float, default=DR_DEFAULT)
    ap.add_argument("--timeconst", nargs="+", type=float, default=TC_DEFAULT,
                    help="contact time constant [s]; sets damping AND stiffness")
    ap.add_argument("--by-family", action="store_true",
                    help="also break drift out per command family -- the only "
                         "way to see an effect confined to station-keeping")
    args = ap.parse_args()

    jobs = [(n, mu, tc, dr) for n in args.policies
            for tc in args.timeconst for dr in args.dampratio for mu in args.mu]
    print(f"{len(jobs)} eval grids "
          f"({len(args.policies)} policies x {len(args.mu)} mu x "
          f"{len(args.timeconst)} timeconst x {len(args.dampratio)} dampratio)\n")

    out = []
    with ProcessPoolExecutor(max_workers=min(len(jobs), os.cpu_count() or 1)) as ex:
        for r in ex.map(_one_point, jobs):
            out.append(r)
            print(f"  done  {r['policy']:<28} mu {r['mu']:.2f}  "
                  f"tc {r['timeconst']:.4f}  dr {r['dampratio']:.2f}  "
                  f"(sink {r['sink_mm']:.3f} mm)  score {r['score']:.3f}")

    for name in args.policies:
        rows = [r for r in out if r["policy"] == name]
        print(f"\n=== {name}")
        print("  score = survive x track_geo, over the 20-command grid, "
              "randomization off\n")
        head = ("     sink  solref            |"
                + "".join(f"  mu {mu:<8.2f}" for mu in args.mu))
        print(head); print("  " + "-" * (len(head) - 2))
        # Softest first, so the column reads the way a person expects: DOWN the
        # page is stiffer. Sorting by sink rather than by dampratio is the whole
        # point -- dampratio alone sorts backwards.
        keys = sorted({(x["timeconst"], x["dampratio"]) for x in rows},
                      key=lambda k: -_sink_mm(*k))
        for tc, dr in keys:
            cells = []
            for mu in args.mu:
                r = next(x for x in rows if x["mu"] == mu
                         and x["timeconst"] == tc and x["dampratio"] == dr)
                cells.append(f"  {r['score']:.3f}/{r['survive']:.2f}")
            sink = _sink_mm(tc, dr)
            note = "  <- ships" if (tc, dr) == (0.005, 1.0) else ""
            print(f"  {sink:7.3f}  [{tc:.4f}, {dr:.2f}] |"
                  + "".join(f"{c:<12}" for c in cells) + note)
        print("\n  rows are sorted SOFTEST FIRST (largest sink at the top).")
        print("\n  cells are score/survival. The mu row the policies were "
              "TRAINED over is 0.72-1.08;")
        print("  nothing has ever trained over the dampratio axis at all "
              "(see rl_general.yaml).")

        if not args.by_family:
            continue
        fams = ["hold", "spin", "cruise", "crab", "turn_big"]
        for dr in args.dampratio:
            print(f"\n  --- per family at dampratio {dr:.2f}  "
                  f"(drift_m; hold is ONE episode, crab is two)")
            print("  family    |" + "".join(f"  mu {mu:<7.2f}" for mu in args.mu))
            for fam in fams:
                cells = []
                for mu in args.mu:
                    r = next(x for x in rows if x["mu"] == mu
                             and x["dampratio"] == dr
                             and x["timeconst"] == args.timeconst[0])
                    f = r["by_family"].get(fam, {})
                    d = f.get("drift_m")
                    cells.append(f"  {d:.3f}" if isinstance(d, (int, float))
                                 else "      -")
                print(f"  {fam:<9} |" + "".join(f"{c:<12}" for c in cells))


if __name__ == "__main__":
    main()
