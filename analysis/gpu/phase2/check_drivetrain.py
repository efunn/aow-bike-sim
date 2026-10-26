"""The drivetrain kernel (aow_sim.gpu.drivetrain) against GeneralEnv's
DrivetrainSim, on the recorded reference: each episode is one world, fed the
recorded `pre` qpos/qvel/ctrl every substep, carrying its own state; its
motor torques and applied forces are compared with the recorded `hooked` ones.

  python analysis/gpu/phase2/check_drivetrain.py [--ref traces/gpu_phase2/reference.npz]
"""
import argparse
import pathlib

import numpy as np
import warp as wp

from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.gpu import drivetrain as gd


def main(ref_path):
    wp.config.quiet = True
    wp.init()
    r = np.load(ref_path, allow_pickle=True)
    starts = list(r["starts"])
    cfg = pathlib.Path(str(r["config"]))
    if not cfg.exists():          # recorded before the configs moved (2026-10-21)
        cfg = pathlib.Path("config/rl") / cfg.name
    env = GeneralEnv(None, _load_rl_config(str(cfg)))
    c = gd.const_from(env._drive)
    n_sub = len(r["sub_pre_time"])
    bounds = [s["substep"] for s in starts] + [n_sub]
    lens = [bounds[i + 1] - bounds[i] for i in range(len(starts))]
    T, W = min(lens), len(starts)          # equal-length episodes as worlds

    st = gd.DriveState(W, c.nc, c.nm)
    for w, s in enumerate(starts):
        gd.load_state(st, w, s["drive"])

    def stack(name):
        return np.stack([r[name][bounds[w]:bounds[w] + T] for w in range(W)], 1)
    qpos, qvel, ctrl = stack("sub_pre_qpos"), stack("sub_pre_qvel"), stack("sub_pre_ctrl")
    want_ctrl, want_frc = stack("sub_hooked_ctrl"), stack("sub_hooked_qfrc_applied")

    d_qpos = wp.zeros(qpos.shape[1:], dtype=float)
    d_qvel = wp.zeros(qvel.shape[1:], dtype=float)
    d_ctrl = wp.zeros(ctrl.shape[1:], dtype=float)
    d_frc = wp.zeros(want_frc.shape[1:], dtype=float)
    got_m, got_f = np.zeros((T, W, 2)), np.zeros((T, W, 2))
    for t in range(T):
        d_qpos.assign(qpos[t].astype(np.float32))
        d_qvel.assign(qvel[t].astype(np.float32))
        d_ctrl.assign(ctrl[t].astype(np.float32))
        gd.launch_pre_step(c, st, d_qpos, d_qvel, d_ctrl, d_frc)
        got_m[t] = d_ctrl.numpy()[:, [c.motor_a, c.motor_b]]
        got_f[t] = d_frc.numpy()[:, [c.da, c.db]]
    ref_m = want_ctrl[:, :, [c.motor_a, c.motor_b]]
    ref_f = want_frc[:, :, [c.da, c.db]]
    for name, got, ref in (("motor torque", got_m, ref_m), ("applied force", got_f, ref_f)):
        err = np.abs(got - ref)
        scale = np.abs(ref).max()
        worst = np.unravel_index(err.argmax(), err.shape)
        print(f"{name:14s} max |err| {err.max():.3g} (ref scale {scale:.3g}, "
              f"rel {err.max() / scale:.2g}) at substep {worst[0]} world {worst[1]}; "
              f"p99 {np.percentile(err, 99):.3g}")
    return got_m, ref_m, got_f, ref_f


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="traces/gpu_phase2/reference.npz")
    main(ap.parse_args().ref)
