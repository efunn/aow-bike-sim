"""The obs/reward kernel (aow_sim.gpu.obs_reward) against GeneralEnv.step's
second half, on the recorded reference. Every policy step is one world (the
kernel's state comes in recorded), so the whole run is one launch.

  python analysis/gpu_phase2/check_obs_reward.py
"""
import argparse

import numpy as np
import warp as wp

from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.gpu import obs_reward as gor


def main(ref_path):
    wp.config.quiet = True
    wp.init()
    r = np.load(ref_path, allow_pickle=True)
    env = GeneralEnv(None, _load_rl_config(str(r["config"])))
    c = gor.const_from(env)
    starts = list(r["starts"])
    S = env.substeps
    ep = r["pol_episode"].astype(int)
    N = len(ep)
    first = {w: s["step"] for w, s in enumerate(starts)}
    local = np.array([k - first[ep[k]] for k in range(N)])
    last_sub = np.arange(N) * S + S - 1

    def before(name, start_key):
        """The env scalar as it stood when step k began."""
        out = []
        for k in range(N):
            if local[k] == 0:
                out.append(np.asarray(starts[ep[k]]["env"][start_key], float))
            else:
                out.append(r[f"pol_{name}"][k - 1])
        return np.array(out)

    f = lambda x: wp.array(np.asarray(x, np.float32), dtype=float)
    i32 = lambda x: wp.array(np.asarray(x, np.int32), dtype=int)
    psi = f(before("psi", "psi"))
    raw_prev = f(before("raw_prev", "raw_prev"))
    prev_a = f(np.array([np.zeros(3) if local[k] == 0 else r["pol_action"][k - 1]
                         for k in range(N)]))
    step = i32(local)
    track_sum, track_n = f(np.zeros(N)), i32(np.zeros(N))
    obs = wp.zeros((N, gor.OBS_DIM), dtype=float)
    rew = wp.zeros(N, dtype=float)
    term, trunc = i32(np.zeros(N)), i32(np.zeros(N))
    gor.launch(c, f(r["sub_post_qpos"][last_sub]), f(r["sub_post_qvel"][last_sub]),
               f(r["sub_ahrs_quat"][last_sub]), f(r["sub_ahrs_gyro"][last_sub]),
               f(r["sub_odo_out"][last_sub, 0]), f(r["sub_odo_out"][last_sub, 1]),
               f(before("v_cmd_w", "v_cmd_w")), f(before("psi_cmd", "psi_cmd")),
               psi, raw_prev, f(r["pol_action"]), prev_a, step, track_sum, track_n,
               obs, rew, term, trunc)

    e_psi = np.abs(psi.numpy() - r["pol_psi"])
    e_rew = np.abs(rew.numpy() - r["pol_reward"])
    nxt = [k for k in range(N - 1) if ep[k + 1] == ep[k]]
    e_obs = np.abs(obs.numpy()[nxt] - r["pol_obs"][[k + 1 for k in nxt]])
    print(f"heading psi  max |err| {e_psi.max():.3g} rad")
    print(f"reward       max |err| {e_rew.max():.3g} (range {r['pol_reward'].min():.3f}"
          f"-{r['pol_reward'].max():.3f})")
    print("obs          max |err| per entry:", np.array2string(e_obs.max(0), precision=2))
    print(f"terminated/truncated agree: "
          f"{np.array_equal(term.numpy(), r['pol_terminated'].astype(int))} / "
          f"{np.array_equal(trunc.numpy(), r['pol_truncated'].astype(int))}; {N} steps")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="traces/gpu_phase2/reference.npz")
    main(ap.parse_args().ref)
