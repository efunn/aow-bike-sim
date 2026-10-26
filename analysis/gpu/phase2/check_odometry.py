"""The odometry kernel (aow_sim.gpu.odometry) against SimOdometry (counts,
front), on the recorded reference: each episode a world, every 100 Hz tick
fed the sensors and the AHRS reading that tick produced, the estimate
compared with the one the Python estimator held after that tick.

  python analysis/gpu/phase2/check_odometry.py
"""
import argparse
import pathlib

import numpy as np
import warp as wp

from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.gpu import odometry as go


def main(ref_path):
    wp.config.quiet = True
    wp.init()
    r = np.load(ref_path, allow_pickle=True)
    cfg = pathlib.Path(str(r["config"]))
    if not cfg.exists():          # recorded before the configs moved (2026-10-21)
        cfg = pathlib.Path("config/rl") / cfg.name
    env = GeneralEnv(None, _load_rl_config(str(cfg)))
    c = go.const_from(env._odo)
    starts = list(r["starts"])
    W = len(starts)
    n_all = len(r["sub_ahrs_at"])
    lo = [s["ahrs_k"] for s in starts]           # first tick after the reset prime
    hi = [s["ahrs_k"] - 1 for s in starts[1:]] + [n_all]
    T = min(h - l for l, h in zip(lo, hi))
    at = r["sub_ahrs_at"]                         # the substep each tick saw
    odo_out = r["sub_odo_out"]                    # held estimate, per substep

    st = go.OdoState(W)
    f32 = lambda x: wp.array(np.asarray(x, np.float32), dtype=float)
    err = []
    for t in range(T):
        idx = [lo[w] + t for w in range(W)]
        go.launch_advance(c, st, f32(r["sub_ahrs_sd"][idx]),
                          f32(r["sub_ahrs_out_quat"][idx]), f32(r["sub_ahrs_out_gyro"][idx]),
                          f32(r["sub_ahrs_out_accel"][idx]))
        got = np.stack([st.v_lon.numpy(), st.v_lat.numpy()], 1)
        want = odo_out[at[idx]]
        err.append(np.abs(got - want))
    e = np.array(err)                              # (T, W, 2)
    scale = np.abs(odo_out).max(0)
    print(f"v_lon max |err| {e[..., 0].max():.3g} m/s (scale {scale[0]:.3g}), "
          f"p99 {np.percentile(e[..., 0], 99):.3g}")
    print(f"v_lat max |err| {e[..., 1].max():.3g} m/s (scale {scale[1]:.3g}), "
          f"p99 {np.percentile(e[..., 1], 99):.3g}")
    big = np.argwhere(e.max(2) > 1e-3)
    print(f"ticks with any error > 1 mm/s: {len(big)} of {T * W}")
    print(f"{T} ticks x {W} worlds")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="traces/gpu_phase2/reference.npz")
    main(ap.parse_args().ref)
