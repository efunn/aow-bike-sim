"""Disturbance-recovery envelope: kick the bike sideways at hold, does it survive.

WHY THIS FILE EXISTS RATHER THAN AN INLINE LOOP. An earlier ad-hoc version of
this measurement gave DIFFERENT ANSWERS FOR THE SAME POLICY AND SEEDS depending
on how the harness was written -- 6/6, 4/6, 6/6, 1/6, 4/6 one run and
6/6, 5/6, 4/6, 2/6, 0/6 the next. The difference was that one built a fresh
GeneralEnv per trial and the other reused one across trials, so state that
`reset()` does not clear (the velocity filter's `_vbar`, `_prev_a`, the
unwrapped heading `_psi`) leaked from trial to trial. A recovery is chaotic
enough that this is the difference between falling and not.

THE LEAK IS `data.qacc_warmstart`. `reset()` does clear the obvious state --
qpos, qvel, `_prev_a`, `_v_bar_w`, `_psi`, `_steer` -- but MuJoCo warm-starts
its constraint solver from the previous solution, and nothing resets that. So
the solver begins each episode from wherever the last one ended. Demonstrated:
one trial run twice in FRESH envs terminates at step 163 both times; the same
trial in an env that had already run a hard fall terminates at 169; clearing
`qacc_warmstart` after reset brings it back to 163, matching fresh exactly.

So the env IS reusable -- which matters, because a fresh one costs a model
build plus an LQR design (~2 s, and this sweep is ~90 trials) -- but only with
that one extra line. `_reset` below is the only sanctioned way to start a
trial here.

THE KICK is a lateral velocity step on the chassis freejoint, matching what
tests/test_hw_odometry.py does (`data.qvel[1] += shove`). NOT `xfrc_applied`:
GeneralEnv.step zeroes that at the top of every step, so a force written from
outside the env is wiped before any physics runs and the disturbance silently
does nothing -- which looks like a miraculously robust policy.

  python analysis/kick_recovery.py --policies general_rl_glide_pitch_hub
  python analysis/kick_recovery.py --dv 0.2 0.3 0.4 --seeds 8

Read the HUB column as well as survival: it says whether the policy still
reaches for the wheel when disturbed, which is the mechanism a hold-time hub
penalty can accidentally train out. Survival counts on 6-8 seeds are noisy;
the hub number is a direct measurement and moves much less.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.general_env import _load_rl_config
from rsa_policies import env_for, load_general, REPO

CONFIG_FOR = {                      # a policy must be evaluated in ITS OWN env
    "general_rl_glide_pitch_dt4e4": "rl_general_glide_pitch.yaml",
    "general_rl_glide_pitch_og":    "rl_general_glide_pitch.yaml",
    "general_rl_glide_pitch_smooth": "rl_general_glide_pitch_smooth.yaml",
    "general_rl_glide_pitch_hub":   "rl_general_glide_pitch_hub.yaml",
    "general_rl_glide_pitch_hub2":  "rl_general_glide_pitch_hub2.yaml",
    "general_rl_glide_pitch_hub3":  "rl_general_glide_pitch_hub3.yaml",
    "general_rl_smooth_diff_og":    "rl_general_smooth_diff.yaml",
    # Hold-only diagnostics: v_max 0, so they have NEVER seen a drive command.
    # Fine to kick (the kick is not a command), meaningless to drive.
    "general_rl_hold":              "rl_general_hold.yaml",
    "general_rl_hold_nohub_2":      "rl_general_hold_nohub.yaml",
}


def make(name):
    """(policy, env, action scale) for one export, built once and reused."""
    cfg = _load_rl_config(REPO / "config" / CONFIG_FOR.get(name, "rl_general.yaml"))
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    pol = load_general(name)
    env = env_for(pol, load_params(), cfg)
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    return pol, env, np.where(scale > 0, scale, 1.0)


def _reset(env, seed):
    obs, _ = env.reset(seed=seed, options={"v_cmd": (0.0, 0.0),
                                           "psi_cmd_rel": 0.0, "difficulty": 1.0})
    env.data.qacc_warmstart[:] = 0.0     # see the module docstring; not optional
    return obs


def trial(pol, env, scale, dv, seed, seconds=8.0, t_kick=3.0, window=1.0):
    """One kick. Returns (survived, mean |hub| over `window` after the kick)."""
    obs = _reset(env, seed)
    k0, kw = int(t_kick / env.ctrl_dt), int(window / env.ctrl_dt)
    hub = []
    for k in range(int(seconds / env.ctrl_dt)):
        if k == k0:
            env.data.qvel[1] += dv
        a = (np.asarray(pol.action(obs), float) / scale)[:env.action_space.shape[0]]
        obs, _r, term, _tr, _i = env.step(a)
        if k0 <= k < k0 + kw:
            hub.append(abs(a[1]))
        if term:
            return False, float(np.mean(hub)) if hub else 0.0
    return True, float(np.mean(hub))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policies", nargs="*", default=list(CONFIG_FOR))
    ap.add_argument("--dv", nargs="*", type=float,
                    default=[0.20, 0.25, 0.30, 0.35, 0.40])
    ap.add_argument("--seeds", type=int, default=6)
    args = ap.parse_args()
    seeds = [11 + 7 * i for i in range(args.seeds)]

    print(f"lateral velocity kick at hold, {len(seeds)} seeds, warm-start cleared per trial\n")
    print(f"{'policy':32}" + "".join(f"{d:>8.2f}" for d in args.dv)
          + f"{'hub used':>10}")
    for name in args.policies:
        if not (Path(REPO / "moves" / f"{name}.npz").exists()):
            print(f"{name:32}  (no export)")
            continue
        pol, env, scale = make(name)
        row, hubs = [], []
        for dv in args.dv:
            res = [trial(pol, env, scale, dv, s) for s in seeds]
            row.append(f"{sum(r[0] for r in res)}/{len(seeds)}".rjust(8))
            hubs += [r[1] for r in res]
        print(f"{name:32}" + "".join(row) + f"{np.mean(hubs):10.2f}", flush=True)


if __name__ == "__main__":
    main()
