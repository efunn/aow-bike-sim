"""What the one-step-stale prev_action (GeneralEnv before 2026-10-14) did to
a policy trained on it, flown both ways in today's env.

  fresh  the obs carries the action just applied -- the env now, and what
         drive._general_compute feeds on the bike
  stale  the obs carries the action before it -- the env every general
         policy up to 10-14 trained on (emulated: the previous action is
         written back into obs[12:15]; nothing else differs)

Same commands, seeds and 15 s episodes both ways; randomization off.

  python analysis/prev_action_stale.py --policy moves/general_rl_v3_s0.npz
"""
import argparse
from multiprocessing import Pool

import numpy as np

CMDS = {   # (v_lon, psi_rel deg)
    "hold": (0.0, 0.0), "straight 0.5": (0.5, 0.0), "reverse 0.4": (-0.4, 0.0),
    "spin 90": (0.0, 90.0), "spin 180": (0.0, 180.0), "moving 0.5 +90": (0.5, 90.0),
}


def _episode(job):
    policy, config, cmd, seed, mode = job
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    from aow_sim.control.policy import load_policy_npz
    cfg = _load_rl_config(config)
    cfg["randomization"] = {**cfg["randomization"], "enabled": False}
    env = GeneralEnv(None, cfg, seed=seed)
    pol = load_policy_npz(policy)
    v, psi = CMDS[cmd]
    obs, _ = env.reset(seed=seed, options={"v_cmd": (v, 0.0),
                                           "psi_cmd_rel": np.deg2rad(psi),
                                           "difficulty": 1.0})
    pol.reset()
    prev = np.zeros(3)
    head, vel, fell = [], [], False
    for _ in range(env.max_steps):
        a = np.clip(pol.mean(obs), -1, 1).astype(np.float32)
        obs, r, term, trunc, info = env.step(a)
        if mode == "stale":
            obs = obs.copy()
            obs[12:15] = prev
        prev = a
        head.append(info["head_err_deg"])
        vel.append(info["vel_err"])
        if term or trunc:
            fell = bool(info["fell"])
            break
    n = len(head)
    return cmd, mode, fell, float(np.mean(head[n // 2:])), float(np.median(vel))


def main(policy, config, seeds):
    jobs = [(policy, config, c, s, m) for c in CMDS for s in range(seeds)
            for m in ("stale", "fresh")]
    with Pool(10) as p:
        rows = p.map(_episode, jobs)
    print(f"{policy}, {seeds} seeds per cell, 15 s\n")
    print(f"{'command':16s} {'mode':6s} {'falls':>6s} {'head err 2nd half':>18s} {'vel err med':>12s}")
    for c in CMDS:
        for m in ("stale", "fresh"):
            rr = [r for r in rows if r[0] == c and r[1] == m]
            print(f"{c:16s} {m:6s} {sum(r[2] for r in rr):>3d}/{len(rr):<2d} "
                  f"{np.mean([r[3] for r in rr]):>15.1f} deg {np.mean([r[4] for r in rr]):>9.3f} m/s")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="moves/general_rl_v3_s0.npz")
    ap.add_argument("--config", default="config/rl/rl_general_v3.yaml")
    ap.add_argument("--seeds", type=int, default=5)
    a = ap.parse_args()
    main(a.policy, a.config, a.seeds)
