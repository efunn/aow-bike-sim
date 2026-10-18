"""Where a GpuGeneralEnv policy step spends its GPU time (plan, phase 2:
the whole env ran ~35 % fewer physics steps/s than bare MJWarp -- which
part costs it?).

Every call the env's `step` makes is wrapped FROM OUTSIDE (module
attributes, the env module's `wp` / `mjw`, the guard's methods), so this
profiles the env's own code, not a copy. Each wrapped call is timed on the
GPU with Warp's activity timing (`wp.timing_begin(TIMING_ALL)`: kernels,
memcpy, memset), summed per section over a few policy steps flown eagerly by
v3_s0 after a warm-up as a graph. The same steps' wall time AS A GRAPH is
measured too: graph wall minus summed GPU time is what the graph's gaps and
launches cost.

  python analysis/gpu/phase2/profile_env.py --nworld 4096 32768
  python analysis/gpu/phase2/profile_env.py --nworld 16 --steps 2   # Mac: CPU wall clock

On the CPU device the per-call time is the wall clock of a synchronous
launch (a smoke test of the wrapping; the split means little there).
One JSON line per size on stdout and appended to --out, then a table.
"""
import argparse
import json
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import warp as wp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench_env import Runner                                   # noqa: E402

import aow_sim.gpu.env as genv                                 # noqa: E402
from aow_sim.gpu import action as gact                         # noqa: E402
from aow_sim.gpu import ahrs as gahrs                          # noqa: E402
from aow_sim.gpu import drivetrain as gdrv                     # noqa: E402
from aow_sim.gpu import gearbox as ggb                         # noqa: E402
from aow_sim.gpu import obs_reward as gobs                     # noqa: E402
from aow_sim.gpu import odometry as godo                       # noqa: E402

# Which part of the step a section belongs to, for the summary.
GROUP = {
    "mjw.step": "physics (MJWarp step)",
    "drivetrain.pre_step": "our hooks, per substep",
    "gearbox.pre_step": "our hooks, per substep",
    "action.apply": "our per-policy-step work",
    "ahrs.draw_z": "our sense tick (100 Hz)",
    "ahrs.sample": "our sense tick (100 Hz)",
    "odometry.advance": "our sense tick (100 Hz)",
    "rebase": "our per-policy-step work",
    "obs_reward": "our per-policy-step work",
    "after_step": "our per-policy-step work",
    "_done": "our per-policy-step work",
    "copy": "our per-policy-step work",
    "reset_worlds": "reset path (every step)",
    "guard.save": "reset path (every step)",
    "mjw.forward": "reset path (every step)",
    "guard.restore": "reset path (every step)",
    "reset_obs": "reset path (every step)",
    "mlp": "policy net",
}


class Prof:
    def __init__(self, cuda):
        self.cuda = cuda
        self.on = False
        self.t = defaultdict(float)          # section -> ms
        self.n = defaultdict(int)            # section -> calls
        self.k = defaultdict(float)          # (section, kernel) -> ms
        self.depth = 0

    def wrap(self, name, fn):
        def inner(*a, **kw):
            if not self.on or self.depth:
                return fn(*a, **kw)
            self.depth += 1
            try:
                if self.cuda:
                    wp.timing_begin(cuda_filter=wp.TIMING_ALL)
                    out = fn(*a, **kw)
                    for r in wp.timing_end():
                        self.t[name] += r.elapsed
                        self.k[(name, r.name)] += r.elapsed
                else:
                    t = time.perf_counter()
                    out = fn(*a, **kw)
                    self.t[name] += 1e3 * (time.perf_counter() - t)
            finally:
                self.depth -= 1
            self.n[name] += 1
            return out
        return inner


class _Proxy:
    """Stands in for a module inside env.py: named attributes wrapped, the
    rest passed through."""

    def __init__(self, mod, wrapped):
        self._mod, self._w = mod, wrapped

    def __getattr__(self, a):
        return self._w.get(a) or getattr(self._mod, a)


def instrument(prof, r):
    """Wrap everything GpuGeneralEnv.step / _reset_done / _tick call."""
    e = r.env
    kname = {"rebase": "rebase", "after_step": "after_step", "_done": "_done",
             "reset_worlds": "reset_worlds", "draw_z": "ahrs.draw_z",
             "reset_obs": "reset_obs"}

    def launch(kernel, *a, **kw):
        name = kname.get(kernel.key.split(".")[-1], "launch:" + kernel.key)
        return prof.wrap(name, wp.launch)(kernel, *a, **kw)

    genv.wp = _Proxy(wp, {"launch": launch, "copy": prof.wrap("copy", wp.copy)})
    genv.mjw = _Proxy(genv.mjw, {"step": prof.wrap("mjw.step", genv.mjw.step),
                                 "forward": prof.wrap("mjw.forward", genv.mjw.forward)})
    gact.launch_apply_action = prof.wrap("action.apply", gact.launch_apply_action)
    gdrv.launch_pre_step = prof.wrap("drivetrain.pre_step", gdrv.launch_pre_step)
    ggb.launch_pre_step = prof.wrap("gearbox.pre_step", ggb.launch_pre_step)
    gahrs.launch_sample = prof.wrap("ahrs.sample", gahrs.launch_sample)
    godo.launch_advance = prof.wrap("odometry.advance", godo.launch_advance)
    gobs.launch = prof.wrap("obs_reward", gobs.launch)
    e.guard.save = prof.wrap("guard.save", e.guard.save)
    e.guard.restore = prof.wrap("guard.restore", e.guard.restore)
    if r.mlp is not None:
        r.mlp = prof.wrap("mlp", r.mlp)


def profile(n, policy, warm, steps, graph_steps, seed, device):
    cuda = wp.get_device(device).is_cuda
    r = Runner(n, policy, seed, device)
    # warm-up as a graph: realistic contact state, kernels compiled
    r.step()
    if cuda:
        r.capture()
    r.run(warm)
    wp.synchronize_device(device)
    # the graph's own wall per policy step, same state region
    t = time.perf_counter()
    r.run(graph_steps)
    wp.synchronize_device(device)
    graph_ms = 1e3 * (time.perf_counter() - t) / graph_steps
    # eager, instrumented
    prof = Prof(cuda)
    instrument(prof, r)
    prof.on = True
    r.graph = None
    t = time.perf_counter()
    r.run(steps)
    wp.synchronize_device(device)
    eager_wall_ms = 1e3 * (time.perf_counter() - t) / steps
    prof.on = False
    tally = r.tally()
    # Bare physics on THIS model, as a graph of the same 50 mjw.step calls
    # and nothing else, from the state just reached (it then drifts without
    # the hooks, so few launches). Phase 1's 2.36M physics steps/s was a
    # different (rigid, export) model: this is the like-for-like baseline.
    e = r.env
    phys_ms = None
    if cuda:
        with wp.ScopedCapture(device=device) as cap:
            for _ in range(e.substeps):
                genv.mjw.step(e.m, e.d)
        wp.capture_launch(cap.graph)
        wp.synchronize_device(device)
        t = time.perf_counter()
        for _ in range(5):
            wp.capture_launch(cap.graph)
        wp.synchronize_device(device)
        phys_ms = 1e3 * (time.perf_counter() - t) / 5
    sec = {k: v / steps for k, v in prof.t.items()}
    total = sum(sec.values())
    groups = defaultdict(float)
    for k, v in sec.items():
        groups[GROUP.get(k, "other")] += v
    top_mjw = sorted(((kn, v / steps) for (s, kn), v in prof.k.items() if s == "mjw.step"),
                     key=lambda x: -x[1])[:12]
    return {
        "nworld": n, "device": wp.get_device(device).name, "policy": policy or "random",
        "steps_profiled": steps, "falls_during": tally["falls"],
        "graph_ms_per_policy_step": round(graph_ms, 3),
        "gpu_ms_per_policy_step": round(total, 3),
        "graph_overhead_ms": round(graph_ms - total, 3) if cuda else None,
        "physics_only_graph_ms": round(phys_ms, 3) if phys_ms else None,
        "physics_only_steps_per_s": round(n * e.substeps / (phys_ms / 1e3)) if phys_ms else None,
        "eager_wall_ms_per_policy_step": round(eager_wall_ms, 1),
        "sections_ms": {k: round(v, 4) for k, v in sorted(sec.items(), key=lambda x: -x[1])},
        "calls_per_step": {k: v // steps for k, v in prof.n.items()},
        "groups_ms": {k: round(v, 4) for k, v in sorted(groups.items(), key=lambda x: -x[1])},
        "mjw_step_top_kernels_ms": [(k, round(v, 4)) for k, v in top_mjw],
    }


def show(rec):
    tot = rec["gpu_ms_per_policy_step"]
    print(f"\n{rec['nworld']} worlds on {rec['device']}: graph {rec['graph_ms_per_policy_step']:.2f} ms "
          f"per policy step; summed GPU time {tot:.2f} ms"
          + (f"; gaps/launch {rec['graph_overhead_ms']:.2f} ms" if rec["graph_overhead_ms"] is not None else ""))
    if rec["physics_only_graph_ms"]:
        print(f"  bare physics, same model, as a graph: {rec['physics_only_graph_ms']:.2f} ms "
              f"({rec['physics_only_steps_per_s'] / 1e6:.2f}M physics steps/s); the whole env "
              f"takes x{rec['graph_ms_per_policy_step'] / rec['physics_only_graph_ms']:.2f}")
    phys = rec["groups_ms"].get("physics (MJWarp step)", 0.0)
    for g, v in rec["groups_ms"].items():
        print(f"  {g:<32} {v:8.3f} ms  {100 * v / tot:5.1f} %  "
              f"(x{v / phys:.2f} of physics)" if phys else "")
    print("  sections:")
    for s, v in rec["sections_ms"].items():
        print(f"    {s:<22} {v:8.4f} ms  {100 * v / tot:5.1f} %  x{rec['calls_per_step'][s]}")
    if rec["mjw_step_top_kernels_ms"]:
        print("  inside mjw.step, top kernels:")
        for k, v in rec["mjw_step_top_kernels_ms"]:
            print(f"    {k[:60]:<60} {v:8.4f} ms")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--nworld", type=int, nargs="+", default=[4096, 32768])
    ap.add_argument("--policy", default="moves/archive/general_rl_v3_s0.npz",
                    help="'' for random actions")
    ap.add_argument("--warm", type=int, default=50, help="graph steps before profiling")
    ap.add_argument("--steps", type=int, default=5, help="profiled policy steps")
    ap.add_argument("--graph-steps", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="results/profile.jsonl")
    a = ap.parse_args()
    wp.config.quiet = True
    wp.init()
    dev = "cuda:0" if wp.is_cuda_available() else "cpu"
    for n in a.nworld:
        rec = profile(n, a.policy or None, a.warm, a.steps, a.graph_steps, a.seed, dev)
        print(json.dumps(rec), flush=True)
        show(rec)
        try:
            Path(a.out).parent.mkdir(parents=True, exist_ok=True)
            with open(a.out, "a") as f:
                f.write(json.dumps(rec) + "\n")
        except OSError:
            pass
