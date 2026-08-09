"""Representational similarity analysis across trained general_rl policies.

Motivating question: independently trained policies cannot be compared unit by
unit -- hidden layers are permutation-symmetric, so "unit 37" means nothing
across nets. Representational geometry *is* comparable: build a dissimilarity
matrix (RDM) over command conditions for each net and correlate the RDMs.

The point of this script is the CONTROLS. A cross-net RDM correlation of ~0.95
is meaningless on its own, because every model here reads the same observation
vector, and conditions that differ in their inputs will produce similar RDMs in
any function of those inputs. So each run reports, alongside the trained-net
correlations:

  input       RDM computed on the normalized observations themselves -- the
              geometry that is free, before any network does anything.
  random      untrained nets of the same shape on the same stimuli. If random
              nets correlate as highly as trained ones, the number says nothing
              about learning.
  tr-rnd      trained vs random. The gap between (trained,trained) and
              (trained,random) is the part attributable to training.

Stimuli are shared: states are pooled from every policy's own rollouts and then
pushed through all models, so representation is isolated from behaviour (a net
is not judged on a state distribution only it visits).

  python analysis/rsa_policies.py
  python analysis/rsa_policies.py --steps 200 --sets crab speed heading

Read-only: loads moves/*.npz and writes nothing.
"""

from __future__ import annotations

import argparse
from itertools import combinations
from pathlib import Path

import numpy as np
from scipy.stats import spearmanr

from aow_sim.build_model import load_params
from aow_sim.control.flick import MOVES_DIR
from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.policy import load_policy_npz

# Label each policy by the move file it loads from, so a plot can never be
# mistaken for a different export.
POLICIES = {n: n for n in ("general_rl_og", "general_rl", "general_rl_1k",
                           "general_rl_smooth_og",
                           "general_rl_smooth_diff")}
REPO = Path(__file__).resolve().parents[1]


def condition_sets(v_max: float) -> dict:
    """Several 'evals'. If the cross-net correlation is the same for all of
    them, it is a property of the observation encoding, not of any particular
    behaviour."""
    return {
        # The five teleop primitives that are SUSTAINED: the command is still
        # being asked for at the end of the trace, so the state is stationary
        # and a single mean pattern represents the whole condition. This is
        # the set to use for any static-pattern analysis.
        "sustained": [("hold", 0, 0, 0), ("fwd", .67 * v_max, 0, 0),
                      ("rev", -.42 * v_max, 0, 0), ("crabL", 0, .33 * v_max, 0),
                      ("crabR", 0, -.33 * v_max, 0)],
        # As `sustained`, but with fwd/rev at EQUAL magnitude. The sustained
        # set uses +0.80 / -0.50 m/s, so the centroid of the four directional
        # commands sits at +0.075 m/s rather than at the origin -- which
        # displaces `hold` away from centre and makes it look anti-correlated
        # with `fwd` even for a perfect code. Use this set whenever the
        # comparison is against the ideal matrix.
        "balanced": [("hold", 0, 0, 0), ("fwd", .42 * v_max, 0, 0),
                     ("rev", -.42 * v_max, 0, 0), ("crabL", 0, .33 * v_max, 0),
                     ("crabR", 0, -.33 * v_max, 0)],
        # KEPT AS A COUNTEREXAMPLE, not for drawing conclusions from. Adding
        # the two turns breaks the stationarity assumption -- see the note in
        # move_confusion.py. Retained so the failure mode stays visible rather
        # than being rediscovered.
        "with_turns": [("hold", 0, 0, 0), ("fwd", .67 * v_max, 0, 0),
                       ("rev", -.42 * v_max, 0, 0), ("crabL", 0, .33 * v_max, 0),
                       ("crabR", 0, -.33 * v_max, 0), ("turnL", 0, 0, 90),
                       ("turnR", 0, 0, -90)],
        # speed only -- conditions differ along ONE observation axis
        "speed": [(f"v{v:+.2f}", v * v_max, 0, 0)
                  for v in (-0.42, -0.2, 0.0, 0.2, 0.42, 0.67, 1.0)],
        # heading only -- differ along psi_err, velocity command fixed at zero
        "heading": [(f"h{d:+.0f}", 0, 0, d)
                    for d in (-170, -90, -45, 0, 45, 90, 170)],
        # crab only, both signs and magnitudes
        "crab": [(f"c{c:+.2f}", 0, c * v_max, 0)
                 for c in (-0.4, -0.25, -0.1, 0.0, 0.1, 0.25, 0.4)],
    }


def hidden(pol, obs_batch):
    """Penultimate-layer activations for a batch of raw observations."""
    x = (np.asarray(obs_batch, float) - pol.obs_mean) / np.sqrt(pol.obs_var + 1e-8)
    x = np.clip(x, -pol.obs_clip, pol.obs_clip)
    for W, b in pol.layers[:-1]:
        x = pol.activation(x @ W.T + b)
    return x


def normalized_obs(pol, obs_batch):
    x = (np.asarray(obs_batch, float) - pol.obs_mean) / np.sqrt(pol.obs_var + 1e-8)
    return np.clip(x, -pol.obs_clip, pol.obs_clip)


def random_like(pol, seed):
    """An untrained net of the same shape, sharing the trained net's
    observation statistics so the input distribution is matched."""
    rng = np.random.default_rng(seed)
    layers = [(rng.normal(0, np.sqrt(2.0 / W.shape[1]), W.shape), np.zeros_like(b))
              for W, b in pol.layers]
    import copy
    r = copy.copy(pol)
    r.layers = layers
    return r


def trace(pol, env, cond, steps):
    """Closed-loop rollout; returns the observations visited."""
    _, v_lon, v_lat, dpsi = cond
    obs, _ = env.reset(seed=7, options={"v_cmd": (v_lon, v_lat),
                                        "psi_cmd_rel": np.deg2rad(dpsi),
                                        "difficulty": 1.0})
    out = []
    for _ in range(steps):
        out.append(np.asarray(obs, float))
        a = pol.action(obs)
        na = np.array([a[0] / pol.bounds.steer_rate_max,
                       a[1] / pol.bounds.hub_max,
                       a[2] / pol.bounds.diff_max])[:env.action_space.shape[0]]
        obs, _r, term, trunc, _i = env.step(na)
        if term or trunc:
            break
    return np.array(out)


def crossnobis(patterns: dict) -> tuple[list, np.ndarray]:
    """Cross-validated Mahalanobis RDM. Splitting each condition in half and
    crossing the halves makes the estimate unbiased: truly identical
    conditions sit at 0 instead of at a positive floor set by noise."""
    labels = list(patterns)
    resid = np.vstack([p - p.mean(0) for p in patterns.values()])
    C = np.cov(resid.T)
    C += np.trace(C) / C.shape[0] * 0.1 * np.eye(C.shape[0])   # shrinkage
    P = np.linalg.inv(C)
    A = {k: v[:len(v) // 2].mean(0) for k, v in patterns.items()}
    B = {k: v[len(v) // 2:].mean(0) for k, v in patterns.items()}
    n = len(labels)
    D = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            D[i, j] = ((A[labels[i]] - A[labels[j]])
                       @ P @ (B[labels[i]] - B[labels[j]]))
    return labels, D


def run_set(name, conds, env, pols, steps):
    # Shared stimuli: pool each policy's own visited states per condition, so
    # every model is scored on the same inputs.
    stim = {}
    for label, *_ in conds:
        stim[label] = []
    for pol in pols.values():
        for cond in conds:
            stim[cond[0]].append(trace(pol, env, cond, steps))
    n_min = min(min(len(t) for t in v) for v in stim.values())
    stim = {k: np.vstack([t[:n_min] for t in v]) for k, v in stim.items()}

    ref = next(iter(pols.values()))
    models = {f"tr:{s}": p for s, p in
              zip([POLICIES[k] for k in POLICIES], pols.values())}
    models.update({f"rnd:{i}": random_like(ref, 100 + i) for i in range(3)})

    rdms = {}
    for mname, m in models.items():
        rdms[mname] = crossnobis({k: hidden(m, v) for k, v in stim.items()})[1]
    labels, rdms["input"] = crossnobis(
        {k: normalized_obs(ref, v) for k, v in stim.items()})

    iu = np.triu_indices(len(labels), 1)
    def corr(a, b):
        return spearmanr(rdms[a][iu], rdms[b][iu]).statistic

    tr = [k for k in rdms if k.startswith("tr:")]
    rnd = [k for k in rdms if k.startswith("rnd:")]
    out = {
        "trained-trained": [corr(a, b) for a, b in combinations(tr, 2)],
        "random-random": [corr(a, b) for a, b in combinations(rnd, 2)],
        "trained-random": [corr(a, b) for a in tr for b in rnd],
        "trained-input": [corr(a, "input") for a in tr],
        "random-input": [corr(a, "input") for a in rnd],
    }
    print(f"\n=== {name}  ({len(labels)} conditions, {n_min} steps/trace) ===")
    for k, v in out.items():
        print(f"  {k:16} r = {np.mean(v):+.3f}   "
              f"[{min(v):+.2f}, {max(v):+.2f}]  n={len(v)}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--steps", type=int, default=100,
                    help="control steps per trace (50 Hz; 100 = 2 s)")
    ap.add_argument("--sets", nargs="*", default=None)
    args = ap.parse_args()

    params = load_params()
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = GeneralEnv(params, cfg)
    pols = {k: load_policy_npz(MOVES_DIR / f"{k}.npz") for k in POLICIES}

    sets = condition_sets(cfg["env"]["v_max"])
    for name in (args.sets or sets):
        run_set(name, sets[name], env, pols, args.steps)

    print("\nHow to read this: the trained-trained number is only evidence of a"
          "\nshared learned geometry if it clearly exceeds random-random and"
          "\ntrained-input. If all four are alike, the RDM is inherited from the"
          "\nobservation encoding and says nothing about what was learned.")


if __name__ == "__main__":
    main()
