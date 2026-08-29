"""Why the pitchless policies never turn around: THEY BACKFLIP IF THEY TRY.

The eval grid cannot show this. Its 180-degree commands start from REST, and a
policy without `obs_pitch` responds by accelerating backwards and never
turning at all -- so the failure never happens, and `head_err_tail` just
records that the heading is wrong. Nothing in the grid asks the bike to turn
around while ALREADY reversing at speed, which is the case that kills it.

This script sets that case up directly, the way teleop does: hold a reverse
command until the speed is established, then snap the heading command 180
degrees (the `8` key), and record the pitch. Above ~0.84 m/s of reverse every
policy WITHOUT obs_pitch pitches up 80-88 degrees and goes over backwards.
`general_rl_pitch_smooth_diff_pi`, which observes pitch, never exceeds ~23
degrees at any reverse speed up to v_max and never falls.

That closes the causal chain behind the per-family finding: it is not that
pitch makes the manoeuvre nicer, it is that WITHOUT PITCH THE MANOEUVRE ENDS
THE EPISODE, so the policy correctly learns never to attempt it. What the eval
sees -- reversing at 0.8 m/s with a 175-degree heading error and near-perfect
velocity tracking -- is the learned avoidance, not the failure.

FELL BEFORE vs FELL AFTER the snap is reported separately and it matters: at
v_max some policies cannot hold a full-speed reverse at all, and counting that
as a flip would be wrong.

  python analysis/reverse_flip.py
  python analysis/reverse_flip.py --policies general_rl_odo_ahrs --fracs 0.7 1.0

Read-only apart from stdout: loads moves/*.npz and writes nothing.
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
from rsa_policies import REPO, env_for, load_general

# (policy, encoder, ahrs) -- the truth-trained pair has to run on truth, which
# is what it was trained for; forcing sensors on it would confound the answer.
DEFAULT_ARMS = [("general_rl_odo_ahrs", "counts", "tm151"),
                ("general_rl_odo_ahrs_rand2", "counts", "tm151"),
                ("general_rl_smooth_diff_pi", "ideal", "none"),
                ("general_rl_pitch_smooth_diff_pi", "ideal", "none")]


def probe(pol, env, v, rev_s, turn_s):
    """Reverse at `v` for `rev_s`, snap the heading 180, watch the pitch."""
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n_act = env.action_space.shape[0]
    obs, _ = env.reset(seed=7, options={"v_cmd": (v, 0.0), "psi_cmd_rel": 0.0,
                                        "difficulty": 1.0})
    n_rev = int(rev_s / env.ctrl_dt)
    pit, rol, when = [], [], None
    for i in range(n_rev + int(turn_s / env.ctrl_dt)):
        if i == n_rev:                      # <- the `8` key
            env._psi_cmd += np.pi
            c, s = np.cos(env._psi_cmd), np.sin(env._psi_cmd)
            env._v_cmd_w = np.array([c * v, s * v])
        a = (np.asarray(pol.action(obs), float) / scale)[:n_act]
        obs, _r, term, trunc, _info = env.step(a)
        st = extract_state(env.data, env._p0)
        if i >= n_rev:                      # only the manoeuvre itself
            pit.append(np.degrees(st.pitch))
            rol.append(abs(np.degrees(st.roll)))
        if term:
            when = "after" if i >= n_rev else "BEFORE"
            break
        if trunc:
            break
    return (max(pit, key=abs) if pit else float("nan"),
            max(rol) if rol else float("nan"), when)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policies", nargs="+", metavar="NAME",
                    help="override the default arms; each runs on its own "
                         "declared encoder and no AHRS")
    ap.add_argument("--fracs", nargs="+", type=float,
                    default=[0.42, 0.70, 0.85, 1.0],
                    help="reverse speeds as a fraction of v_max")
    ap.add_argument("--reverse-s", type=float, default=4.0,
                    help="seconds of reverse before the heading snap")
    ap.add_argument("--turn-s", type=float, default=8.0)
    args = ap.parse_args()

    arms = ([(n, "", "none") for n in args.policies] if args.policies
            else DEFAULT_ARMS)
    params = load_params()
    base = _load_rl_config(REPO / "config" / "rl_general.yaml")
    base = {**base, "randomization": {**base["randomization"], "enabled": False}}
    v_max = base["env"]["v_max"]

    print(f"{args.reverse_s:g} s of reverse, then the heading command snaps "
          f"180 deg. Peak PITCH\nover the manoeuvre; 80-90 deg is the bike "
          f"over backwards. `BEFORE` means it fell\nduring the reverse, before "
          f"the snap -- not a flip.\n")
    print(f"{'policy':32}" + "".join(f"{'-' + format(f * v_max, '.2f'):>19}"
                                     for f in args.fracs))
    print(f"{'':32}" + "".join(f"{'pitch':>9}{'roll':>6}{'fell':>4}"
                               for _ in args.fracs))
    for name, enc, ahrs in arms:
        cfg = base if ahrs == "none" else {
            **base, "env": {**base["env"], "ahrs_level": ahrs,
                            "ahrs_tau_s": 0.19, "ahrs_channels": "both"}}
        pol = load_general(name)
        if enc:
            pol.odometry_encoder = enc
        env = env_for(pol, params, cfg)
        cells = ""
        for f in args.fracs:
            p, r, when = probe(pol, env, -f * v_max, args.reverse_s, args.turn_s)
            cells += (f"{p:>8.1f}°{r:>5.0f}°"
                      + (f"{'YES' if when == 'after' else when:>4}"
                         if when else f"{' -':>4}"))
        print(f"{name:32}{cells}")


if __name__ == "__main__":
    main()
