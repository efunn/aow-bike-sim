"""Does an exported env graph (`python -m aow_sim.gpu.export_graph`) still
fly, loaded the way the PufferLib env loads it? Phase 3 of
docs/plans/gpu-training.md.

Loads the `.wrp` with `wp.capture_load` (the same `wp_apic_load_graph` the
C side calls), reaches every buffer by NAME through its device pointer, and
flies v3_s0 for --steps policy steps: the MLP (Warp kernels, bench_env's)
reads `obs`, writes `act_in`, the graph launches. Reports falls, reward per
step, and the episode summaries the C side turns into PufferLib's logs
(ep_end, last_*), so a wrong buffer or a stale reset shows here first.

  python analysis/gpu/phase3/check_graph.py graphs/aow_bike_4096 --steps 300
  python analysis/gpu/phase3/check_graph.py graphs/aow_bike_4096 --steps 100 --random
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import warp as wp

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "phase2"))
from bench_env import WarpMLP                                   # noqa: E402

DT = {"float32": wp.float32, "int32": wp.int32}


def bind(graph, meta, device):
    """Every bound buffer as a Warp array over the loaded graph's memory."""
    out = {}
    for name, b in meta["buffers"].items():
        dt = DT[b["dtype"]]
        ptr = graph.get_param_ptr(name)
        n = int(np.prod(b["shape"]))
        if graph._params[name]["size"] < n * 4:
            raise RuntimeError(f"{name}: bound {graph._params[name]['size']} B < {n * 4} B")
        out[name] = wp.array(ptr=ptr, shape=tuple(b["shape"]), dtype=dt, device=device)
    return out


def main(a):
    wp.config.quiet = True
    wp.init()
    meta = json.loads(Path(a.stem + ".json").read_text())
    dev = "cuda:0" if wp.is_cuda_available() else "cpu"
    g = wp.capture_load(a.stem, device=dev)
    b = bind(g, meta, dev)
    n = meta["nworld"]
    mlp = None if a.random else WarpMLP(a.policy, n, dev)
    rng = np.random.default_rng(0)
    falls = ends = 0
    rsum = 0.0
    lens, rets, tracks, diffs = [], [], [], []
    for k in range(a.steps):
        if mlp is None:              # random actions: falls within ~20 steps, so the
            b["act_in"].assign(rng.uniform(-1, 1, (n, 3)).astype(np.float32))  # reset path runs
        else:
            mlp(b["obs"], b["act_in"])
        wp.capture_launch(g)
        r = b["reward"].numpy()
        if not np.all(np.isfinite(b["obs"].numpy())):
            print(f"non-finite obs at step {k}")
            break
        rsum += float(r.sum())
        e = b["ep_end"].numpy().astype(bool)
        if e.any():
            ends += int(e.sum())
            falls += int(b["last_fell"].numpy()[e].sum())
            lens += b["last_len"].numpy()[e].tolist()
            rets += b["last_ret"].numpy()[e].tolist()
            tracks += b["last_track"].numpy()[e].tolist()
            diffs += b["last_diff"].numpy()[e].tolist()
    rec = {"stem": a.stem, "actions": "random" if a.random else a.policy,
           "nworld": n, "steps": a.steps, "device": str(wp.get_device(dev).name),
           "falls": falls, "episode_ends": ends,
           "reward_per_step": round(rsum / (n * a.steps), 4),
           "ended_len_mean": round(float(np.mean(lens)), 1) if lens else None,
           "ended_return_mean": round(float(np.mean(rets)), 2) if rets else None,
           "ended_track_mean": round(float(np.mean(tracks)), 3) if tracks else None,
           "ended_difficulty_mean": round(float(np.mean(diffs)), 3) if diffs else None,
           "max_steps": meta["max_steps"]}
    print(json.dumps(rec))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("stem")
    ap.add_argument("--policy", default="moves/general_rl_v3_s0.npz")
    ap.add_argument("--steps", type=int, default=300)
    ap.add_argument("--random", action="store_true", help="random actions, not the policy")
    main(ap.parse_args())
