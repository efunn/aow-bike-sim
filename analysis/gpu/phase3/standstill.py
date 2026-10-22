"""Does a policy STAND STILL when told to? Zero command from a fresh reset in
GeneralEnv (the eval config, 5 s episodes), several seeds: max planar speed,
when it first passes 0.3 m/s, and where it ends up. Torch-free, no warp.

    python analysis/gpu/phase3/standstill.py general_rl_v3_s0 general_puffer_1h_26M

Why it exists (2026-10-15): PufferLib policies whose training curves looked
fine drove off at 0.4-0.7 m/s on a zero command (one in circles, steer
parked at 24 deg) -- the eval grid's single hold episode does not show it.
v3_s0 reference: max |v| 0.10-0.17 m/s, never above 0.3, 0.05-0.10 m.
"""
from __future__ import annotations

import argparse

import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.policy import load_policy_npz
from aow_sim.drivetrain_model import from_env_config
from aow_sim.eval_general import _eval_cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("moves", nargs="+", help="names under moves/")
    ap.add_argument("--config", default="config/rl/rl_general_v3.yaml")
    ap.add_argument("--seeds", type=int, default=6)
    a = ap.parse_args()
    cfg = _load_rl_config(a.config)
    params = from_env_config(load_params(), cfg["env"])
    for name in a.moves:
        pol = load_policy_npz(f"moves/{name}.npz")
        env = GeneralEnv(params, _eval_cfg(cfg))
        sc = np.array([pol.bounds.steer_rate_max, pol.bounds.hub_max, pol.bounds.diff_max])
        sc = np.where(sc == 0, 1, sc)
        print(name)
        for seed in range(a.seeds):
            obs, _ = env.reset(seed=seed, options={"v_cmd": (0.0, 0.0), "psi_cmd_rel": 0.0,
                                                   "difficulty": 1.0})
            pol.reset()
            p0 = env.data.qpos[:2].copy()
            vmax, t_move, k, done, term, hub = 0.0, None, 0, False, False, []
            while not done:
                act = np.asarray(pol.action(obs), float)[:3] / sc
                obs, _r, term, trunc, _ = env.step(act)
                v = float(np.hypot(*env.data.qvel[:2]))
                vmax = max(vmax, v)
                if t_move is None and v > 0.3:
                    t_move = k * env.ctrl_dt
                hub.append(abs(act[1]))
                done = term or trunc
                k += 1
            d = float(np.hypot(*(env.data.qpos[:2] - p0)))
            moved = "never" if t_move is None else f"{t_move:.1f} s"
            print(f"  seed {seed}: {'FELL' if term else 'ok  '} {k * env.ctrl_dt:4.1f} s  "
                  f"max |v| {vmax:.2f} m/s  >0.3 m/s at {moved:>6}  "
                  f"end {d:.2f} m from start  mean |hub| {np.mean(hub):.2f}")


if __name__ == "__main__":
    main()
