"""The action kernel (aow_sim.gpu.action) against GeneralEnv.step's action
path, on the recorded reference: each episode a world, the recorded action
and measured steer in, the drive and steer commands the first substep saw out.

  python analysis/gpu/phase2/check_action.py
"""
import argparse
import pathlib

import numpy as np
import warp as wp

from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.gpu import action as ga


def main(ref_path):
    wp.config.quiet = True
    wp.init()
    r = np.load(ref_path, allow_pickle=True)
    cfg = pathlib.Path(str(r["config"]))
    if not cfg.exists():          # recorded before the configs moved (2026-10-21)
        cfg = pathlib.Path("config/rl") / cfg.name
    env = GeneralEnv(None, _load_rl_config(str(cfg)))
    c = ga.const_from(env)
    starts = list(r["starts"])
    S = env.substeps
    ep = r["pol_episode"].astype(int)
    W = len(starts)
    T = min(int((ep == w).sum()) for w in range(W))
    first = [s["step"] for s in starts]

    target = wp.array(np.array([s["env"]["steer"] for s in starts], np.float32), dtype=float)
    ctrl = wp.zeros((W, env.model.nu), dtype=float)
    errs = []
    for k in range(T):
        idx = [first[w] + k for w in range(W)]
        act = wp.array(r["pol_action"][idx].astype(np.float32), dtype=float)
        qpos = wp.array(r["sub_pre_qpos"][[i * S for i in idx]].astype(np.float32), dtype=float)
        ga.launch_apply_action(c, act, qpos, target, ctrl)
        got = ctrl.numpy()[:, [c.ctrl_a, c.ctrl_b, c.ctrl_steer]]
        want = r["sub_pre_ctrl"][[i * S for i in idx]][:, [c.ctrl_a, c.ctrl_b, c.ctrl_steer]]
        errs.append(np.abs(got - want))
    e = np.max(errs, axis=(0, 1))
    print(f"drive_a {e[0]:.3g}  drive_b {e[1]:.3g}  steer target {e[2]:.3g} "
          f"(max |err| over {T} steps x {W} worlds)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="traces/gpu_phase2/reference.npz")
    main(ap.parse_args().ref)
