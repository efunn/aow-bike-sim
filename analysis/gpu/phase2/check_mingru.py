"""bench_env's Warp MinGRU against numpy MinGRUPolicy, EVERY step, eager,
4 worlds: the memory carried across steps, and cleared on each auto-reset.
One numpy policy per world, fed the same obs; from step 10 world 0 is
driven with a fixed full action until it falls, to exercise _forget.

    PYTHONPATH=src python analysis/gpu/phase2/check_mingru.py moves/general_puffer_fixes_52M.npz [steps]

Runs on Warp's CPU backend (no graph capture; bench_env.py --check does the
eager-vs-graph comparison, on CUDA). 2026-10-15, Mac CPU, 52M, 60 steps:
action err 4.6e-7, memory err 4.2e-5 (float32 sums vs float64, h ~0.4),
4 resets each leaving the world's memory at exactly 0."""
import sys
import numpy as np
import warp as wp
sys.path.insert(0, "analysis/gpu/phase2")
from bench_env import Runner
from aow_sim.control.policy import load_policy_npz

wp.config.quiet = True
wp.init()
path, n, steps = sys.argv[1], 4, int(sys.argv[2]) if len(sys.argv) > 2 else 40
r = Runner(n, path, 3, "cuda:0" if wp.is_cuda_available() else "cpu")
pols = [load_policy_npz(path) for _ in range(n)]
worst_a = worst_h = 0.0
resets = 0
for k in range(steps):
    obs = r.env.obs.numpy().copy()
    r.mlp(r.env.obs, r.act)
    a_w = r.act.numpy().copy()
    if k >= 10:                     # shove world 0: full steer one way
        a_w[0] = [1.0, 1.0, -1.0]
        r.act.assign(a_w)
    a_np = np.stack([np.clip(p.mean(o), -1, 1) for p, o in zip(pols, obs)])
    h_np = np.stack([p.h for p in pols])
    if k < 10:
        worst_a = max(worst_a, float(np.abs(a_w - a_np).max()))
    else:
        worst_a = max(worst_a, float(np.abs(a_w[1:] - a_np[1:]).max()))
    worst_h = max(worst_h, float(np.abs(r.mlp.state.numpy() - h_np).max()))
    _, rew, term, trunc = r.env.step(r.act)
    r.mlp.after_step(term, trunc)
    done = (term.numpy() | trunc.numpy()) != 0
    for w in np.flatnonzero(done):
        pols[w].reset(); resets += 1
    if done.any():
        print(f"step {k}: reset worlds {np.flatnonzero(done).tolist()}, warp h there now "
              f"{np.abs(r.mlp.state.numpy()[done]).max():.2e}")
print(f"{steps} steps x {n} worlds: max |action| err {worst_a:.2e}, max |h| err {worst_h:.2e}, "
      f"resets {resets}, mean |h| {np.abs(h_np).mean():.3f}")
