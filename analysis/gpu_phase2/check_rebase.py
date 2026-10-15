"""The drivetrain angle rebase (aow_sim.gpu.rebase) leaves the plant as it was.

Two single-world GPU envs, same seed and actions (v3_s0's, computed on A),
rebase OFF -- bit-identical. After `--warm` steps env B is changed and its
rebase switched on, then both run on:
  consistent  angles moved UP by i = 12, j = 2 (i - j a multiple of 10, the
              rebase's own rule) with the hook state that lives in them; B's
              rebase brings them back at the end of the next step
  nudge       input a moved 1e-6 rad, nothing else: the YARDSTICK. The bike
              is chaotic, and any float32 rounding the shift costs grows like
              this; "exact" means no worse than a nudge
  broken      input a alone by 6*pi, the hub not following: the control
              that shows the check can fail

  python analysis/gpu_phase2/check_rebase.py
"""
import argparse

import numpy as np
import warp as wp

from aow_sim.control.policy import load_policy_npz
from aow_sim.gpu.env import GpuGeneralEnv

CONFIG = "config/rl_general_v3.yaml"


def _fixed(env, v):
    """Hold one command for the whole episode, as the eval options do."""
    env.v_cmd_w.assign(np.tile([v, 0.0], (env.nworld, 1)).astype(np.float32))
    env.psi_cmd.assign(np.zeros(env.nworld, np.float32))
    env.next_resample.assign(np.full(env.nworld, 10 ** 9, np.int32))


def _shift_up(env, i, j, consistent=True):
    c = env.c_rb
    unit = 6 * np.pi
    da, db = unit * i, unit * j
    q = env.d.qpos.numpy()
    if consistent:
        dhub, dring = 0.5 * (da + db), 0.5 * (da - db)
        q[:, c.qa] += da
        q[:, c.qb] += db
        q[:, c.hub] += dhub
        q[:, c.ring] += dring
        q[:, c.roller0:c.roller0 + 8] += 2.4 * dring
    else:
        q[:, c.qa] += da                       # hub, ring, b left behind
        dring = 0.0
    env.d.qpos.assign(q)
    s = env.s_drv
    for arr, add in ((s.ref, np.array([da, db]) / c.belt), (s.fa, np.array([da, db]))):
        arr.assign(arr.numpy() + (add if consistent else np.array([add[0], 0.0])))
    th = s.th.numpy()
    th[:, 0] += da / c.belt
    if consistent:
        th[:, 1] += db / c.belt
    s.th.assign(th)
    s.anchor.assign(s.anchor.numpy() + (0.5 * (da - db) if consistent else 0.5 * da))
    prev = env.s_odo.prev.numpy()
    prev[:, 0] += 4096 * i
    if consistent:
        prev[:, 1] += 4096 * j
    env.s_odo.prev.assign(prev)


def pair(policy, warm, after, v, mode):
    pol = load_policy_npz(policy)
    A = GpuGeneralEnv(CONFIG, nworld=1, seed=5, rebase=False)
    B = GpuGeneralEnv(CONFIG, nworld=1, seed=5, rebase=False)
    oa, ob = A.reset().numpy(), B.reset().numpy()
    _fixed(A, v)
    _fixed(B, v)
    act = wp.zeros((1, 3), dtype=float)
    diffs = []
    for k in range(warm + after):
        if k == warm:
            if mode == "consistent":
                _shift_up(B, 12, 2, True)
            elif mode == "broken":
                _shift_up(B, 1, 0, False)
            else:
                q = B.d.qpos.numpy()
                q[:, B.c_rb.qa] += 1e-6
                B.d.qpos.assign(q)
            B.rebase = True
        a = np.clip(pol.mean(oa[0]), -1, 1).astype(np.float32)[None]
        act.assign(a)
        oa, ra, *_ = (x.numpy() for x in A.step(act))
        ob, rb, *_ = (x.numpy() for x in B.step(act))
        if k >= warm:
            diffs.append((np.abs(oa - ob).max(), abs(float(ra[0] - rb[0]))))
    qa = A.d.qpos.numpy()[0, A.c_rb.qa]
    qb = B.d.qpos.numpy()[0, B.c_rb.qa]
    return np.array(diffs), qa, qb


def main(policy, warm, after, v):
    for mode in ("nudge", "consistent", "broken"):
        d, qa, qb = pair(policy, warm, after, v, mode)
        print(f"{mode:10s}: max |obs diff| after 1 step {d[0, 0]:.2g}, 2 {d[1, 0]:.2g}, "
              f"5 {d[:5, 0].max():.2g}, {after} {d[:, 0].max():.2g}; reward "
              f"{d[:5, 1].max():.2g} (5 steps).  input a: A {qa:.1f}, B {qb:.1f} rad")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="moves/general_rl_v3_s0.npz")
    ap.add_argument("--warm", type=int, default=100)
    ap.add_argument("--after", type=int, default=25)
    ap.add_argument("--v", type=float, default=1.0)
    a = ap.parse_args()
    wp.config.quiet = True
    wp.init()
    main(a.policy, a.warm, a.after, a.v)
