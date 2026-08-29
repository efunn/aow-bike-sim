"""What the `pitch` observation is actually for, and what breaks without it.

`general_rl_pitch_smooth_diff_pi` completes the 180-degree reversal; the
policies without `obs_pitch` do not, and backflip if they try. That is a
correlation across four policies. This is the causal test: take the ONE policy
that can do it and perturb ONLY its pitch pair, leaving every other channel at
MuJoCo truth.

  normal   the policy as exported
  blank    pitch and pitch_rate held at 0 -- the value they take when level
  noisy    +1.42 deg RMS on pitch, the MEASURED TM151 pitch error

WHAT IT SHOWS.

  * BLANKING BRINGS THE BACKFLIP BACK, at the same reverse speeds and the same
    84-86 deg as the policies that never had pitch. Same weights, same
    sensors, one channel zeroed. The eval grid barely notices (0.866 -> 0.816,
    one extra fall in twenty), which is the signature of a channel with a
    NARROW purpose rather than general regulation -- blanking a microcontrol
    input would wreck ordinary driving.
  * NOISE DOES NOT. At the measured sensor error the manoeuvre still
    completes at every reverse speed. It costs general performance
    (0.866 -> 0.676) and not the flip protection.
  * THE OPERATIVE BAND IS 2-8 DEG, not the 80-90 of the flip itself. The
    second table asks the policy for its action twice at every step, once with
    the real pitch pair and once with it blanked, and bins the difference by
    the pitch at that step: 0.145 of a bound below 1 deg, 1.06 by 2-3 deg, and
    saturated at the full range by 5-8. The policy is catching the front wheel
    on the way up, not reacting to a bike already over. 97% of steps sit in
    the idle band, which is why removing pitch costs so little on the grid.

WHY THAT MATTERS FOR THE SENSOR. Signal 2-8 deg against a 1.42 deg noise floor
is an SNR of 1.4-5.6, not the 60 the 80-90 framing suggests. The measured
outcome above says it survives anyway -- detection is a repeated-sample
decision over the ~0.1-0.5 s the onset takes, and the response saturates by
5-8 deg, so the policy needs to resolve "above ~3 deg", not measure an angle.
But the margin is thin, and a LOW-PASS FILTER ON PITCH WOULD BE ACTIVELY
HARMFUL: it would clean up the idle band at the cost of lagging the onset,
which is the one thing the channel is for.

  python analysis/pitch_ablation.py
  python analysis/pitch_ablation.py --policy general_rl_odo_ahrs_pitch

Read-only apart from stdout.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aow_sim.build_model import load_params
from aow_sim.control.balance import extract_state
from aow_sim.control.general_env import _load_rl_config
from aow_sim.train_general_rl import _eval_episodes, _score, eval_cmds
from rsa_policies import REPO, env_for, load_general

# Measured TM151 pitch error, this repo's own sim_ahrs against truth.
NOISE_DEG = 1.418
# The gyro's real sigma is 0.083 deg/s (GYRO_NOISE_PP_DPS * PP_TO_SIGMA), so
# this is ~85x the hardware figure -- deliberately punitive, because a null
# result under too MUCH noise is the useful direction to be wrong in.
RATE_NOISE_DEG_S = NOISE_DEG * 5.0
FRACS = (0.42, 0.70, 0.85, 1.0)


def make_act(pol, env, mode, rng):
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n = env.action_space.shape[0]
    lay = list(env.obs_layout)
    ip, ir = lay.index("pitch"), lay.index("pitch_rate")

    def act(obs):
        o = np.asarray(obs, float).copy()
        if mode == "blank":
            o[ip] = o[ir] = 0.0
        elif mode == "noisy":
            o[ip] += np.deg2rad(NOISE_DEG) * rng.standard_normal()
            o[ir] += np.deg2rad(RATE_NOISE_DEG_S) * rng.standard_normal()
        return (np.asarray(pol.action(o), float) / scale)[:n]
    return act


def probe(pol, env, mode, v, rng, rev_s=4.0, turn_s=8.0):
    """Reverse until the speed is established, then snap the heading 180."""
    act = make_act(pol, env, mode, rng)
    obs, _ = env.reset(seed=7, options={"v_cmd": (v, 0.0), "psi_cmd_rel": 0.0,
                                        "difficulty": 1.0})
    n_rev = int(rev_s / env.ctrl_dt)
    pit, when = [], None
    for i in range(n_rev + int(turn_s / env.ctrl_dt)):
        if i == n_rev:
            env._psi_cmd += np.pi
            c, s = np.cos(env._psi_cmd), np.sin(env._psi_cmd)
            env._v_cmd_w = np.array([c * v, s * v])
        obs, _r, term, trunc, _i = env.step(act(obs))
        if i >= n_rev:
            pit.append(np.degrees(extract_state(env.data, env._p0).pitch))
        if term:
            when = "after" if i >= n_rev else "BEFORE"
            break
        if trunc:
            break
    return (max(pit, key=abs) if pit else float("nan"), when)


def onset(pol, env):
    """|action difference| when pitch is blanked, binned by |pitch|."""
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n = env.action_space.shape[0]
    lay = list(env.obs_layout)
    ip, ir = lay.index("pitch"), lay.index("pitch_rate")
    rows = []
    for f in FRACS[1:]:
        v = -f * env.p["control"]["drive"]["v_max"] if False else -f * 1.2
        obs, _ = env.reset(seed=7, options={"v_cmd": (v, 0.0),
                                            "psi_cmd_rel": 0.0,
                                            "difficulty": 1.0})
        n_rev = int(4.0 / env.ctrl_dt)
        for i in range(n_rev + int(8.0 / env.ctrl_dt)):
            if i == n_rev:
                env._psi_cmd += np.pi
                c, s = np.cos(env._psi_cmd), np.sin(env._psi_cmd)
                env._v_cmd_w = np.array([c * v, s * v])
            a_real = np.asarray(pol.action(obs), float) / scale
            o = np.asarray(obs, float).copy()
            o[ip] = o[ir] = 0.0
            a_blank = np.asarray(pol.action(o), float) / scale
            rows.append((abs(np.degrees(extract_state(env.data, env._p0).pitch)),
                         float(np.abs(a_real - a_blank).max())))
            obs, _r, term, trunc, _i = env.step(a_real[:n])
            if term or trunc:
                break
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", default="general_rl_pitch_smooth_diff_pi")
    args = ap.parse_args()

    params = load_params()
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    v_max = cfg["env"]["v_max"]
    pol = load_general(args.policy)
    env = env_for(pol, params, cfg)
    if "pitch" not in env.obs_layout:
        raise SystemExit(f"{args.policy} has no pitch in its observation")

    print(f"{args.policy}, MuJoCo truth sensors, ONLY the pitch pair "
          f"perturbed.\npeak |pitch| over a reverse-then-180, and the eval "
          f"grid:\n")
    print(f"{'':10}" + "".join(f"{'-' + format(f * v_max, '.2f') + ' m/s':>17}"
                               for f in FRACS) + "      eval grid")
    for mode in ("normal", "blank", "noisy"):
        rng = np.random.default_rng(0)
        cells = ""
        for f in FRACS:
            pk, when = probe(pol, env, mode, -f * v_max, rng)
            cells += (f"{pk:>12.1f}°"
                      + f"{('FELL' if when == 'after' else when or ''):>5}")
        rng = np.random.default_rng(0)
        m, _ = _eval_episodes(env, make_act(pol, env, mode, rng),
                              eval_cmds(v_max))
        print(f"{mode:10}{cells}   {_score(m):.3f} / {m['survive_rate']:.2f}")

    A = onset(pol, env)
    print(f"\n\nWHERE THE CHANNEL EARNS ITS PLACE. {len(A)} steps; the policy "
          f"asked twice at\nevery step, once with the real pitch pair and once "
          f"with it blanked:\n")
    print(f"{'|pitch| band':>16}{'steps':>8}{'mean |da|':>12}{'p90 |da|':>11}")
    edges = [0, 1, 2, 3, 5, 8, 12, 20, 90]
    for lo, hi in zip(edges, edges[1:]):
        sel = (A[:, 0] >= lo) & (A[:, 0] < hi)
        if sel.sum() < 5:
            continue
        print(f"{f'{lo}-{hi} deg':>16}{int(sel.sum()):>8}"
              f"{A[sel, 1].mean():>12.3f}{np.percentile(A[sel, 1], 90):>11.3f}")
    print(f"\n|da| is a fraction of the action bound; 2.0 is the full range.")
    print(f"peak |pitch| reached with the channel intact: {A[:, 0].max():.1f} deg "
          f"-- it never lets the bike get near the 84-86 the blanked run hits.")


if __name__ == "__main__":
    main()
