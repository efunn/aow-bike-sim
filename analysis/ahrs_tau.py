"""Does the AHRS orientation-error CORRELATION TIME matter? Mostly no.

`sim_ahrs.TAU_ORIENT_S` was a guess of 2.0 s; a real TM151 measures 0.19 s
(`analysis/tm151_check.py`, 300 s at rest, exponential fit r2 0.999). That is
10x, and `general_rl_odo_ahrs` trained against the guess -- so the worry was
that it had specialised to a timescale the hardware does not have. This script
is what settled it; the constant moved to 0.19 on the strength of it, and
`--taus` still defaults to comparing the two.

WHAT THIS SCRIPT FOUND, and the reason the worry is smaller than it looked:

  1. tau cannot change the SIZE of the error. `_gm_step` is a stationary
     Gauss-Markov process -- the innovation carries a sqrt(1-a^2) factor
     precisely so the standing deviation stays at the level's RMS for every
     tau. tau moves only how fast the error wanders. Section `error` measures
     this: RMS flat at ~1.45 deg from tau 1e-5 to 60, while the mean
     step-to-step change goes 1.67 -> 0.03 deg.

  2. Both ends of the tau range are BENIGN, which is the module header's own
     prediction: white noise the loop averages away at one end, a constant
     offset it trims out at the other. So the damage is a shallow interior
     bump, not a monotone cost.

  3. Retraining at the measured tau produced a policy that is not better.
     Section `grid` is the 2 x N table. `odo_ahrs` (trained at 2.0) loses
     survival 1.00 -> 0.95 when flown at 0.19, which is the whole documented
     effect and is ONE episode in twenty.

  4. The per-command detail kills the specialisation story outright.
     `odo_ahrs` disagrees with itself across tau on ONE command of twenty --
     while `general_rl_odo`, which never trained against an AHRS and so cannot
     have learned a timescale, flips FOUR, and `odo_ahrs_rand` flips SIX. That
     ordering is impossible if the flips measure learned specialisation; they
     measure marginal episodes moving under a different noise draw. `odo_ahrs`'s
     one flip is a `dpsi 180` command, already recorded in status.md as an edge
     case it fails regardless of tau.

  5. The heading-error gap is not a broad degradation. Section `detail`
     breaks it down: the 7.8 -> 17.3 deg grid mean is two episodes out of
     twenty, one of which simply never turns around (171.7 deg final error).
     The MEDIAN is 5.5 vs 5.3 deg -- unchanged.

CONCLUSION. The AHRS noise MAGNITUDE matters a great deal (see the
`odo_ahrs` vs `odo` rows: training against the error is worth ~0.15 of score
and 27 deg of heading). Its AUTOCORRELATION does not, over the range anyone
has reason to believe. Keep the Gauss-Markov model -- an error that jumps
independently every control step is not what a fusion filter does, and the
model costs nothing -- but stop treating tau as a live risk.

WHAT THIS DOES NOT SHOW. There is no hand-flown demonstration of tau, and
this script is not one either: every number here comes from fixed-seed
episodes on the eval grid. Attempts to find a teleop case failed -- the
commands that separate the taus are the same ones the policy drops at either
tau. Treat a single flipped episode as an anecdote; the seed-noise floor on
`score` is about +-0.02.

  python analysis/ahrs_tau.py                      # all three sections
  python analysis/ahrs_tau.py --section error      # just the error model
  python analysis/ahrs_tau.py --taus 0.19 2.0 --policies general_rl_odo_ahrs

Read-only apart from stdout: loads moves/*.npz and writes nothing.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aow_sim.build_model import build_model, load_params
from aow_sim.control.general_env import _load_rl_config
from aow_sim.control.linearize import settle_upright
from aow_sim.sim_ahrs import SimAhrs, rpy_from_quat
from aow_sim.train_general_rl import _eval_episodes, _score, eval_cmds
from rsa_policies import REPO, env_for, load_general

# The policies the tau question is actually about: one trained without any
# AHRS, one at the guess, one at the measurement, and the two that randomise
# it. `odo` is the control -- it never trained against an AHRS, so any tau
# sensitivity it shows is the plant's, not a learned specialisation.
DEFAULT_POLICIES = ("general_rl_odo",
                    "general_rl_odo_ahrs",
                    "general_rl_odo_ahrs_tau019",
                    "general_rl_odo_ahrs_rand",
                    "general_rl_odo_ahrs_rand2")

# The guess and the measurement. Everything below is a 2-column comparison of
# these unless --taus says otherwise.
DEFAULT_TAUS = (2.0, 0.19)

# Sweep for the error-model section. Spans four decades so the two benign
# ends and the interior are all visible in one table.
ERROR_TAUS = (1e-5, 0.19, 2.0, 10.0, 60.0, 1e7)


def base_cfg():
    """The eval config chatter.py uses: rl_general.yaml, randomization off."""
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    return {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}


def ahrs_cfg(cfg, tau, level="tm151"):
    return {**cfg, "env": {**cfg["env"], "ahrs_level": level,
                           "ahrs_tau_s": float(tau), "ahrs_channels": "both"}}


def policy_for(name, encoder):
    """Load a move, and FORCE its encoder so every row is comparable.

    `general_rl_odo` predates the `odometry_encoder` field, so
    `policy_env_overrides` defaults it to `ideal` -- instantaneous joint
    velocity, no quantisation, no RateFilter. Left alone it would be scored on
    an easier plant than every `counts` policy beside it, which reads as the
    no-AHRS policy winning. It is not a fair row; it is a different bike.
    """
    pol = load_general(name)
    if encoder:
        pol.odometry_encoder = encoder
    return pol


def act_fn(pol, env):
    """`env.step` wants the NORMALIZED action (fraction of bound) and applies
    scale_action itself -- the same convention chatter.py's rollout_grid uses.
    Feeding it `pol.action()` raw double-scales, and every episode falls in
    under 1.3 s, which reads like a broken policy rather than a broken
    harness."""
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n = env.action_space.shape[0]
    return lambda obs: (np.asarray(pol.action(obs), float) / scale)[:n]


def run_grid(pol, env, cmds):
    """The eval grid, through the TRAINER's own episode loop.

    Deliberately `_eval_episodes` rather than a loop of our own: it is what
    `analysis/chatter.py` and every `moves/*.yaml` metrics block ran, so a
    number here and a number there cannot drift apart. It also owns the
    action convention -- `env.step` wants the NORMALIZED action (fraction of
    bound) and applies scale_action itself, and feeding it `pol.action()` raw
    double-scales and drops every episode inside 1.3 s.
    """
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n = env.action_space.shape[0]
    act = lambda obs: (np.asarray(pol.action(obs), float) / scale)[:n]
    return _eval_episodes(env, act, cmds)


def section_error(args):
    """What tau does to the ERROR ITSELF, with no policy in the loop."""
    p = load_params()
    m = build_model(p)
    d = mujoco.MjData(m)
    d.qpos[:] = settle_upright(m).qpos
    mujoco.mj_forward(m, d)
    adr = m.sensor("ahrs_quat").adr[0]
    truth = rpy_from_quat(np.array(d.sensordata[adr:adr + 4]))[0]

    dt, T = 1.0 / args.hz, args.seconds
    n = int(T / dt)
    print(f"THE ERROR ITSELF -- tm151, {T:g} s at {args.hz:g} Hz, bike held "
          f"still.\nRMS is flat by construction; only the wander rate moves.\n")
    print(f"{'tau [s]':>10}{'roll err RMS':>14}{'mean |step|':>13}"
          f"{'first 5 s':>11}{'last 5 s':>10}")
    for tau in ERROR_TAUS:
        a = SimAhrs(m, p, level="tm151", seed=0, tau_orient_s=tau)
        err = np.empty(n)
        for i in range(n):
            err[i] = np.rad2deg(rpy_from_quat(a.sample(d, dt)["quat"])[0]
                                - truth)
        k = int(5.0 / dt)
        print(f"{tau:>10g}{err.std():>13.3f}°{np.abs(np.diff(err)).mean():>12.3f}°"
              f"{err[:k].std():>10.3f}°{err[-k:].std():>9.3f}°")
    print("\nThe two ends are the benign ones sim_ahrs.py:236 predicts. Note "
          "the last\nrow: the process starts at zero and needs ~tau to reach "
          "its RMS, so a very\nlong tau is not a frozen 1.5° offset -- it is "
          "no error at all. Same reason\nthe first 5 s of any episode "
          "under-errors at tau 2.0 (0.75° of 1.42°).")


def section_grid(args):
    """Score, survival and heading for every policy at every tau."""
    params = load_params()
    cfg = base_cfg()
    cmds = eval_cmds(cfg["env"]["v_max"])
    out = {}
    for tau in args.taus:
        for name in args.policies:
            pol = policy_for(name, args.encoder)
            out[(name, tau)] = run_grid(pol, env_for(pol, params,
                                                     ahrs_cfg(cfg, tau)), cmds)

    w = max(len(n) for n in args.policies) + 2
    enc = args.encoder or "each policy's own (NOT COMPARABLE)"
    print(f"THE POLICY GRID -- {len(cmds)} commands, identical seeds, "
          f"randomization off,\n--ahrs tm151, encoder {enc}.\n")
    print(f"{'policy':{w}}" + "".join(f"{'tau ' + str(t):>30}"
                                      for t in args.taus))
    print(f"{'':{w}}" + "".join(f"{'score':>8}{'surv':>7}{'head':>8}{'med':>7}"
                                for _ in args.taus))
    for name in args.policies:
        line = f"{name:{w}}"
        for tau in args.taus:
            m, rows = out[(name, tau)]
            head = np.array([r["head_err_deg"] for r in rows])
            line += (f"{_score(m):>8.3f}{m['survive_rate']:>7.2f}"
                     f"{head.mean():>7.1f}°{np.median(head):>6.1f}°")
        print(line)
    print("\n`med` is the MEDIAN final heading error and is the honest column. "
          "`head_err_deg`\nis the error at the LAST step of an episode, not a "
          "time average, so one command\nthe policy gave up on moves a "
          "20-episode mean by several degrees.")

    spread = {n: abs(_score(out[(n, args.taus[0])][0])
                     - _score(out[(n, args.taus[-1])][0]))
              for n in args.policies}
    print("\nscore spread across tau (the tau-sensitivity of each policy):")
    for n, v in sorted(spread.items(), key=lambda kv: kv[1]):
        print(f"  {n:34}{v:.3f}")
    return out


def section_detail(args, out=None):
    """Which command disagrees, and what the heading mean is really made of."""
    if out is None:
        out = section_grid(args)
    cmds = eval_cmds(base_cfg()["env"]["v_max"])

    print("\nPER-COMMAND DISAGREEMENTS -- same policy, same seed, one tau "
          "falls and the\nother does not:")
    found = False
    for name in args.policies:
        for k, (a, b, c) in enumerate(cmds):
            fell = [out[(name, t)][1][k]["fell"] for t in args.taus]
            if len(set(fell)) > 1:
                found = True
                which = ", ".join(f"tau {t}: {'FELL' if f else 'survived'}"
                                  for t, f in zip(args.taus, fell))
                print(f"  {name}  cmd {k}  v_lon {a:+.2f} v_lat {b:+.2f} "
                      f"dpsi {np.rad2deg(c):+.0f}° -- {which}")
    if not found:
        print("  none -- every policy agrees with itself across taus")
    else:
        print("\n  A dpsi 180 command here is WEAK evidence: status.md "
              "already records\n  'forward travel plus a 180° heading flip "
              "can still drop it' as an edge\n  case of odo_ahrs independent "
              "of tau.")

    print("\nWHERE EACH HEADING MEAN COMES FROM -- the worst episodes in each "
          "cell:")
    for name in args.policies:
        for tau in args.taus:
            rows = out[(name, tau)][1]
            head = np.array([r["head_err_deg"] for r in rows])
            worst = np.argsort(head)[::-1][:2]
            print(f"  {name:30} tau {tau:<5} mean {head.mean():>5.1f}° "
                  f"median {np.median(head):>5.1f}°  worst: "
                  + ", ".join(f"cmd {i} {head[i]:.0f}°" for i in worst)
                  + f"  without the worst: "
                    f"{np.delete(head, worst[0]).mean():.1f}°")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--section", choices=("all", "error", "grid", "detail"),
                    default="all", help="which part to run; `error` needs no "
                                        "policies and is seconds rather than "
                                        "minutes")
    ap.add_argument("--policies", nargs="+", metavar="NAME",
                    default=list(DEFAULT_POLICIES),
                    help="move names to evaluate")
    ap.add_argument("--taus", nargs="+", type=float, default=list(DEFAULT_TAUS),
                    metavar="S", help="correlation times to compare [s]")
    ap.add_argument("--encoder", default="counts",
                    choices=("counts", "ideal", "reported", ""),
                    help="force every policy onto ONE encoder model so the "
                         "rows are comparable; \"\" leaves each policy on its "
                         "own declaration, which is NOT comparable across a "
                         "set that mixes them")
    ap.add_argument("--hz", type=float, default=50.0,
                    help="sample rate for the error-model section")
    ap.add_argument("--seconds", type=float, default=60.0,
                    help="duration for the error-model section")
    args = ap.parse_args()

    rows = None
    if args.section in ("all", "error"):
        section_error(args)
    if args.section in ("all", "grid"):
        print()
        rows = section_grid(args)
    if args.section in ("all", "detail"):
        section_detail(args, rows)


if __name__ == "__main__":
    main()
