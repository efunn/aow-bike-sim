"""The command sampler (aow_sim.gpu.commands) against GeneralEnv's family
sampler, by DISTRIBUTION: Warp's generator cannot reproduce numpy's stream.
At each difficulty, N draws each side: family shares, |v| and |step|
quantiles, the resample interval. A difference well under the sampling noise
(~1/sqrt(N) on a share) is a pass.

  python analysis/gpu/phase2/check_commands.py
"""
import argparse

import numpy as np
import warp as wp

from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.gpu import commands as gc

FAM = ("hold", "turn_in_place", "straight", "moving_turn")


def _classify(v, step):
    return np.where((v == 0) & (step == 0), 0, np.where(v == 0, 1, np.where(step == 0, 2, 3)))


def python_draws(env, d, n, seed):
    rng = np.random.default_rng(seed)
    env._diff = d
    v, st, nr = np.zeros(n), np.zeros(n), np.zeros(n)
    for i in range(n):
        env._psi = 0.0
        env._psi_cmd = 0.0
        env._step = 0
        env._sample_family_command(rng)
        env._schedule_resample(rng)
        pc = env._psi_cmd
        v[i] = env._v_cmd_w @ np.array([np.cos(pc), np.sin(pc)])
        st[i] = pc
        nr[i] = env._next_resample
    return v, st, nr


def warp_draws(c, d, n, seed):
    rng = wp.zeros(n, dtype=wp.uint32)
    wp.launch(gc.seed_rng, n, [seed, rng])
    vw = wp.zeros((n, 2), dtype=float)
    pc = wp.zeros(n, dtype=float)
    nr = wp.zeros(n, dtype=int)
    wp.launch(gc.draw_many, n, [c, float(d), rng, vw, pc, nr])
    pc = pc.numpy().astype(float)
    v = vw.numpy() @ np.eye(2)
    v = v[:, 0] * np.cos(pc) + v[:, 1] * np.sin(pc)
    return v, pc, nr.numpy().astype(float)


def main(n):
    wp.config.quiet = True
    wp.init()
    env = GeneralEnv(None, _load_rl_config("config/rl_general_v3.yaml"))
    c = gc.const_from(env)
    worst = 0.0
    for d in (0.0, 0.3, 0.6, 1.0):
        pv, ps, pn = python_draws(env, d, n, 1)
        wv, ws, wn = warp_draws(c, d, n, 1)
        pf, wf = _classify(pv, ps), _classify(wv, ws)
        print(f"difficulty {d}:")
        for k, name in enumerate(FAM):
            a, b = np.mean(pf == k), np.mean(wf == k)
            worst = max(worst, abs(a - b))
            print(f"  {name:14s} share  python {a:.4f}  warp {b:.4f}")
        for label, p, w in (("|v| m/s", np.abs(pv[pv != 0]), np.abs(wv[wv != 0])),
                            ("|step| deg", np.degrees(np.abs(ps[ps != 0])),
                             np.degrees(np.abs(ws[ws != 0]))),
                            ("resample steps", pn, wn)):
            if len(p) and len(w):
                q = (0.1, 0.5, 0.9)
                print(f"  {label:14s} q10/50/90  python {np.round(np.quantile(p, q), 3)}"
                      f"  warp {np.round(np.quantile(w, q), 3)}")
        sp = np.mean(pv[pv != 0] > 0) if np.any(pv) else np.nan
        sw = np.mean(wv[wv != 0] > 0) if np.any(wv) else np.nan
        print(f"  forward share of v != 0  python {sp:.3f}  warp {sw:.3f}")
    print(f"\nworst family-share gap {worst:.4f} (sampling noise ~{1 / np.sqrt(n):.4f}); {n} draws a side")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=100_000)
    main(ap.parse_args().n)
