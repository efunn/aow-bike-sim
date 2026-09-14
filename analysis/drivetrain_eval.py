"""Can the existing policies drive the DETAILED drivetrain?

The eval grid (the same 20 commands, seeds and scoring as every table in
docs/status.md) over policies x plant variants, where a variant is the ideal
drivetrain or some subset of config/drivetrain_model.yaml switched on. One
process per (policy, variant), so every row ran the same code on the same
grid and the only thing that differs along a policy's rows is the plant.

    python analysis/drivetrain_eval.py
    python analysis/drivetrain_eval.py --policies general_rl_odo_ahrs \
        --variants ideal servo full --by-family
    python analysis/drivetrain_eval.py --bench 2000      # step cost, serial

VARIANTS
    ideal           no overlay: the plant every policy trained on
    servo           firmware velocity loop + motor line + measured inertia and
                    input-shaft friction, factory gains P 100 / I 1920
    servo+detent    ... plus the differential's tooth-pass friction
    full            ... plus per-roller slop
    slop            roller slop alone, on the ideal servo
    full_p400       full, at firmware P 400 / I 1920
    full_p400_i3840 full, at P 400 / I 3840 -- the gains that cut detent
                    stalls most on the bench, and ring most
    full_p200       full, at P 200 / I 1920 -- no peak on the bench
    full_p400_nolag full_p400 with the loop lag removed (diagnostic only)
    full_every2     full, firmware loop at 1.25 kHz instead of every step

SENSORS default to `--encoder counts --ahrs tm151 --ahrs-tau 0.19`, the mode
docs/status.md's policy table is measured in, so the `ideal` rows reproduce it.
The flags set the mode ON THE POLICY, as analysis/per_command.py does,
overriding what the move yaml declares.

`--bench` runs one policy SERIALLY through each variant and reports env steps
per second against `ideal`: the number to watch before any of this goes into
training. The parallel eval's timings share cores and are not a benchmark.

Read-only: loads moves/*.npz, prints, writes nothing.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from aow_sim import drivetrain_model as dm  # noqa: E402
from aow_sim.build_model import load_params  # noqa: E402
from aow_sim.control.general_env import _load_rl_config  # noqa: E402
from aow_sim.train_general_rl import (FAMILIES, _eval_episodes,  # noqa: E402
                                      _score, eval_cmds)
from rsa_policies import REPO, env_for, load_general  # noqa: E402

VARIANTS = {
    "ideal": None,
    "servo": dict(without=("detent", "roller_slop")),
    "servo+detent": dict(without=("roller_slop",)),
    "full": dict(),
    "slop": dict(without=("servo", "detent")),
    "full_p400": dict(gains=(400, 1920)),
    "full_p400_i3840": dict(gains=(400, 3840)),
    "full_p200": dict(gains=(200, 1920)),
    # P 400 with the loop's measurement lag and velocity filter removed: the
    # same gains without the ~20 Hz peak. Not a plant anyone can buy -- it is
    # there to say whether a collapse at P 400 is the resonance.
    "full_p400_nolag": dict(gains=(400, 1920),
                            overrides={"servo": {"measurement_delay_s": 0.0,
                                                 "velocity_filter_s": 0.0}}),
    # The firmware loop and frictions every second physics step (1.25 kHz).
    "full_every2": dict(overrides={"update_every": 2}),
}
DEFAULT_VARIANTS = ("ideal", "servo", "servo+detent", "full", "slop")
DEFAULT_POLICIES = ("general_rl_odo_ahrs", "general_rl_odo_ahrs_rand2")


def params_for(variant: str) -> dict:
    params = load_params()
    spec = VARIANTS[variant]
    if spec is None:
        return params
    return dm.with_drivetrain(params, **spec)


def cfg_for() -> dict:
    """Randomization off and no ball, as train_general_rl._eval_cfg."""
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    return {**cfg,
            "randomization": {**cfg["randomization"], "enabled": False},
            "env": {**cfg["env"], "ball_prob": 0.0}}


def build(name, variant, encoder, ahrs, tau):
    params, cfg = params_for(variant), cfg_for()
    pol = load_general(name)
    if encoder:
        pol.odometry_encoder = encoder
    pol.ahrs_level, pol.ahrs_tau_s, pol.ahrs_channels = ahrs, tau, "both"
    # The VARIANT decides the plant here, not the policy's own record.
    pol.drivetrain_model = None
    env = env_for(pol, params, cfg)
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n_act = env.action_space.shape[0]

    def act(o):
        return (np.asarray(pol.action(o), float) / scale)[:n_act]

    return env, act, cfg


def one(job):
    name, variant, encoder, ahrs, tau = job
    env, act, cfg = build(name, variant, encoder, ahrs, tau)
    t0 = time.perf_counter()
    m, rows = _eval_episodes(env, act, eval_cmds(cfg["env"]["v_max"]))
    wall = time.perf_counter() - t0
    return name, variant, m, rows, sum(r["steps"] for r in rows) / wall


def bench(name, variants, steps, encoder, ahrs, tau):
    print(f"\nstep cost, {name}, {steps} env steps per variant, serial "
          f"(hold command, episodes restarted on a fall)")
    base = None
    for v in variants:
        env, act, _cfg = build(name, v, encoder, ahrs, tau)
        obs, _ = env.reset(seed=0, options={"v_cmd": (0.0, 0.0), "difficulty": 1.0})
        for _ in range(50):                 # warm up caches and the policy
            obs, *_ = env.step(act(obs))
        t0 = time.perf_counter()
        for _ in range(steps):
            obs, _r, term, trunc, _i = env.step(act(obs))
            if term or trunc:
                obs, _ = env.reset(seed=0, options={"v_cmd": (0.0, 0.0),
                                                    "difficulty": 1.0})
        rate = steps / (time.perf_counter() - t0)
        base = base or rate
        print(f"  {v:<16s} {rate:7.0f} env steps/s   x{rate / base:.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policies", nargs="+", default=list(DEFAULT_POLICIES))
    ap.add_argument("--variants", nargs="+", default=list(DEFAULT_VARIANTS),
                    choices=list(VARIANTS))
    ap.add_argument("--encoder", default="counts")
    ap.add_argument("--ahrs", default="tm151")
    ap.add_argument("--ahrs-tau", type=float, default=0.19)
    ap.add_argument("--by-family", action="store_true",
                    help="survival and track per command family as well")
    ap.add_argument("--bench", type=int, default=0, metavar="STEPS",
                    help="instead of the eval, time STEPS env steps of the first "
                         "policy through each variant, serially")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    args = ap.parse_args()

    if args.bench:
        bench(args.policies[0], args.variants, args.bench, args.encoder,
              args.ahrs, args.ahrs_tau)
        return

    jobs = [(n, v, args.encoder, args.ahrs, args.ahrs_tau)
            for n in args.policies for v in args.variants]
    print(f"{len(jobs)} evals, sensors: encoder {args.encoder}, ahrs {args.ahrs} "
          f"tau {args.ahrs_tau:g}")
    out = {}
    with ProcessPoolExecutor(max_workers=min(len(jobs), args.workers)) as ex:
        for name, variant, m, rows, _rate in ex.map(one, jobs):
            out[(name, variant)] = (m, rows)
    print(f"\n{'policy':<28s} {'variant':<16s} {'score':>6s} {'surv':>5s} "
          f"{'trk_geo':>7s} {'vel_err':>7s} {'head':>6s}  fell on")
    for n in args.policies:
        for v in args.variants:
            m, rows = out[(n, v)]
            fell = [f"{c[0]:+.2f},{c[1]:+.2f},{c[2]:+d}" for c in
                    (r["cmd"] for r in rows if r["fell"])]
            print(f"{n:<28s} {v:<16s} {_score(m):6.3f} {m['survive_rate']:5.2f} "
                  f"{m['track_geo']:7.3f} {m['vel_err']:7.3f} "
                  f"{m['head_err_deg']:6.1f}  {' '.join(fell) or '-'}")
            if args.by_family:
                fam = m["by_family"]
                print("    survival  " + "  ".join(
                    f"{k} {fam[k]['survive_rate']:.2f}" for k in FAMILIES if k in fam))
        print()


if __name__ == "__main__":
    main()
