"""Fly a trained general policy in the assembled GPU env (GpuGeneralEnv) and,
for comparison, in GeneralEnv: falls, return, tracking. A smoke test of the
whole port, not a parity check -- the engines' contacts differ (phase 1).

  python analysis/gpu/phase2/fly_policy.py --nworld 8 --steps 250
"""
import argparse
import time

import numpy as np
import warp as wp

from aow_sim.control.policy import load_policy_npz
from aow_sim.gpu.env import GpuGeneralEnv

CONFIG = "config/rl_general_v3.yaml"


def _act(pol, obs):
    return np.stack([np.clip(pol.mean(o), -1, 1) for o in obs]).astype(np.float32)


def fly_gpu(policy, nworld, steps, seed):
    g = GpuGeneralEnv(CONFIG, nworld=nworld, seed=seed)
    pol = load_policy_npz(policy)
    obs = g.reset().numpy()
    act = wp.zeros((nworld, 3), dtype=float)
    rets, falls, ends, rews = np.zeros(nworld), 0, 0, []
    t = time.perf_counter()
    for k in range(steps):
        act.assign(_act(pol, obs))
        o, r, term, trunc = g.step(act)
        obs, r = o.numpy(), r.numpy()
        term, trunc = term.numpy(), trunc.numpy()
        if not np.all(np.isfinite(obs)):
            print(f"  non-finite obs at step {k}")
            break
        rets += r
        rews.append(r)
        falls += int(term.sum())
        ends += int((term | trunc).sum())
    dt = time.perf_counter() - t
    return falls, ends, np.mean(rews), dt


def fly_cpu(policy, nworld, steps, seed):
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    pol = load_policy_npz(policy)
    falls, rews = 0, []
    for w in range(nworld):
        env = GeneralEnv(None, _load_rl_config(CONFIG), seed=seed + w)
        obs, _ = env.reset(seed=seed + w)
        for _ in range(steps):
            obs, r, term, trunc, _ = env.step(_act(pol, [obs])[0])
            rews.append(r)
            if term or trunc:
                falls += int(term)
                obs, _ = env.reset()
    return falls, np.mean(rews)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="moves/general_rl_v3_s0.npz")
    ap.add_argument("--nworld", type=int, default=8)
    ap.add_argument("--steps", type=int, default=250)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    wp.config.quiet = True
    wp.init()
    f, e, mr, dt = fly_gpu(a.policy, a.nworld, a.steps, a.seed)
    print(f"GPU env ({wp.get_device()}): {a.nworld} worlds x {a.steps} steps: falls {f}, "
          f"episode ends {e}, mean reward/step {mr:.3f}; {dt:.1f} s "
          f"({a.nworld * a.steps / dt:.0f} policy steps/s)")
    f2, mr2 = fly_cpu(a.policy, a.nworld, a.steps, a.seed)
    print(f"GeneralEnv (C MuJoCo): same: falls {f2}, mean reward/step {mr2:.3f}")
