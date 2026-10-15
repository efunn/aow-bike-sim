"""The AHRS kernel (aow_sim.gpu.ahrs) against SimAhrs at tm151_filter, on the
recorded reference: each episode a world, every 100 Hz tick fed the sensor
reading and the 12 normal draws numpy used, outputs compared.

  python analysis/gpu_phase2/check_ahrs.py
"""
import argparse

import numpy as np
import warp as wp

from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.gpu import ahrs as gah


def main(ref_path):
    wp.config.quiet = True
    wp.init()
    r = np.load(ref_path, allow_pickle=True)
    env = GeneralEnv(None, _load_rl_config(str(r["config"])))
    c = gah.const_from(env._ahrs)
    starts = list(r["starts"])
    W = len(starts)
    n_all = len(r["sub_ahrs_at"])
    # an episode's ticks: from its start (after the reset's priming sample)
    # to the next episode's priming sample
    lo = [s["ahrs_k"] for s in starts]
    hi = [s["ahrs_k"] - 1 for s in starts[1:]] + [n_all]
    T = min(h - l for l, h in zip(lo, hi))

    st = gah.AhrsState(W)
    for w, s in enumerate(starts):
        gah.load_state(st, w, s["ahrs"])
    sd, z, dts = r["sub_ahrs_sd"], r["sub_ahrs_z"], r["sub_ahrs_dt"]
    want = {k: r[f"sub_ahrs_out_{k}"] for k in ("quat", "gyro", "accel")}
    err = {k: [] for k in want}
    for t in range(T):
        idx = [lo[w] + t for w in range(W)]
        gah.launch_sample(c, st, float(dts[idx[0]]),
                          wp.array(sd[idx].astype(np.float32), dtype=float),
                          wp.array(z[idx].astype(np.float32), dtype=float))
        for k, arr in (("quat", st.quat), ("gyro", st.gyro), ("accel", st.accel)):
            got = arr.numpy()
            ref = want[k][idx]
            if k == "quat":                      # q and -q are one rotation
                sign = np.sign(np.sum(got * ref, axis=1, keepdims=True))
                got = got * sign
            err[k].append(np.abs(got - ref).max())
    for k, e in err.items():
        e = np.array(e)
        print(f"{k:5s} max |err| {e.max():.3g} (first half {e[:T // 2].max():.3g}, "
              f"second half {e[T // 2:].max():.3g})")
    # the same in degrees of roll, what the policy reads
    from aow_sim.sim_ahrs import rpy_from_quat
    idx = [lo[w] + T - 1 for w in range(W)]
    got = st.quat.numpy()
    for w in range(W):
        gr = np.degrees(rpy_from_quat(got[w]))
        rr = np.degrees(rpy_from_quat(want["quat"][idx[w]]))
        print(f"world {w}: final roll/pitch/yaw diff [deg] {np.round(gr - rr, 5)}")
    print(f"{T} ticks x {W} worlds")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="traces/gpu_phase2/reference.npz")
    main(ap.parse_args().ref)
