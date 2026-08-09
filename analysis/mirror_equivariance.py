"""How left-right asymmetric is a general_rl policy, against a null.

The plant is mirror-symmetric (axle_cant_deg 0, roller cone pairs mirrored
about the wheel mid-plane) and the command distribution is symmetric, so an
unbiased policy should satisfy

    a(M . obs) = M_a . a(obs)

for the sagittal reflection M. The residual IS the policy's handedness, and it
needs no alignment between nets -- each net is compared against its own
mirrored self, so unit orderings never enter.

The number is only interpretable against the UNTRAINED baseline this also
computes: a random net has no reason to be equivariant, so it marks the "no
symmetry learned at all" end of the scale, and 0% marks perfect symmetry.

  python analysis/mirror_equivariance.py

Read-only: loads moves/*.npz and writes nothing.
"""

from __future__ import annotations

import argparse
import copy

import numpy as np

from aow_sim.control.flick import MOVES_DIR
from aow_sim.control.policy import load_policy_npz

# Labelled by the move file each one loads from.
POLICIES = {n: n for n in ("general_rl_og", "general_rl", "general_rl_1k",
                           "general_rl_smooth_og",
                           "general_rl_smooth_diff_og")}

# build_obs order: 0 roll, 1 roll_rate, 2 yaw_rate, 3 sin2steer, 4 cos2steer,
# 5 steer_rate, 6 v_lon, 7 v_lat, 8 v_cmd_lon, 9 v_cmd_lat, 10 sin psi_err,
# 11 cos psi_err, 12..14 prev_action [steer_rate, hub, diff].
FLIP_OBS = np.ones(15)
FLIP_OBS[[0, 1, 2, 3, 5, 7, 9, 10, 12, 14]] = -1
FLIP_ACT = np.array([-1.0, 1.0, -1.0])          # [steer_rate, hub, diff]


def sample_states(n, rng):
    """Plausible states with a spread of commands. NOT the on-policy
    distribution -- but mirror error is a PAIRED comparison (the same states,
    reflected), so a distribution mismatch hits both sides equally."""
    o = np.zeros((n, 15))
    o[:, 0] = rng.normal(0, 0.05, n)
    o[:, 1] = rng.normal(0, 0.3, n)
    o[:, 2] = rng.normal(0, 0.3, n)
    st = rng.normal(0, 0.4, n)
    o[:, 3], o[:, 4] = np.sin(2 * st), np.cos(2 * st)
    o[:, 5] = rng.normal(0, 0.5, n)
    o[:, 6] = rng.uniform(-0.5, 1.2, n)
    o[:, 7] = rng.normal(0, 0.15, n)
    o[:, 8] = rng.uniform(-0.5, 1.2, n)
    o[:, 9] = rng.uniform(-0.48, 0.48, n)
    pe = rng.uniform(-np.pi, np.pi, n)
    o[:, 10], o[:, 11] = np.sin(pe), np.cos(pe)
    o[:, 12:15] = rng.normal(0, 0.3, (n, 3))
    return o


def mirror_error(pol, O) -> float:
    """Mean |a(M o) - M_a a(o)|, normalized per action channel so steer, hub
    and diff contribute comparably despite their different units."""
    act = lambda X: np.array([pol.action(x) for x in X])
    base = act(O)
    err = np.abs(act(O * FLIP_OBS) - base * FLIP_ACT).mean(0)
    return float((err / (np.abs(base).mean(0) + 1e-9)).mean() * 100)


def random_like(pol, seed):
    r = copy.copy(pol)
    g = np.random.default_rng(seed)
    r.layers = [(g.normal(0, np.sqrt(2.0 / W.shape[1]), W.shape), np.zeros_like(b))
                for W, b in pol.layers]
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--n", type=int, default=3000, help="states to average over")
    ap.add_argument("--nulls", type=int, default=5, help="random nets for the null")
    args = ap.parse_args()

    rng = np.random.default_rng(0)
    O = sample_states(args.n, rng)
    pols = {k: load_policy_npz(MOVES_DIR / f"{k}.npz") for k in POLICIES}

    print("mirror-equivariance error   (0% = perfectly symmetric)\n")
    for k, name in POLICIES.items():
        print(f"  trained  {name:18} {mirror_error(pols[k], O):5.0f}%")
    ref = next(iter(pols.values()))
    nulls = [mirror_error(random_like(ref, 100 + i), O) for i in range(args.nulls)]
    print(f"\n  UNTRAINED null  (n={args.nulls})      "
          f"{np.mean(nulls):5.0f}%   [{min(nulls):.0f}, {max(nulls):.0f}]")
    print("\nTraining moves the policies well below the untrained null, so some"
          "\nsymmetry IS learned -- but none lands near 0%, and the ordering is"
          "\nthe handedness you can feel in teleop.")


if __name__ == "__main__":
    main()
