"""Whole-env speed of GpuGeneralEnv on one GPU (plan, phase 2 pod run, steps
3-4): policy steps/s with the hooks, auto-reset and obs all in, eager and as
one captured CUDA graph per policy step, random actions or a trained MLP.

The MLP runs as Warp kernels (normalize -> dense layers -> clip), so the
policy and the env capture into the SAME graph: one launch per policy step,
no host round trip. Falls and reward are summed on the device inside the
graph, so the timed loop never syncs.

  python analysis/gpu_phase2/bench_env.py --check            # capture == eager?
  python analysis/gpu_phase2/bench_env.py --nworld 1024 4096 16384 32768
  python analysis/gpu_phase2/bench_env.py --nworld 4096 --policy moves/general_rl_v3_s0.npz

Each result is one JSON line on stdout and appended to --out.
"""
import argparse
import json
import subprocess
import time

import numpy as np
import warp as wp

from aow_sim.control.policy import load_policy_npz
from aow_sim.gpu.env import GpuGeneralEnv

CONFIG = "config/rl_general_v3.yaml"
ACT = {"none": 0, "tanh": 1, "relu": 2}


@wp.kernel
def _rand_act(rng: wp.array(dtype=wp.uint32), act: wp.array2d(dtype=float)):
    w = wp.tid()
    s = rng[w]
    for j in range(3):
        act[w, j] = 2.0 * wp.randf(s) - 1.0
    rng[w] = s


@wp.kernel
def _normalize(obs: wp.array2d(dtype=float), mean: wp.array(dtype=float),
               inv_std: wp.array(dtype=float), clip: float, x: wp.array2d(dtype=float)):
    w, j = wp.tid()
    x[w, j] = wp.clamp((obs[w, j] - mean[j]) * inv_std[j], -clip, clip)


@wp.kernel
def _dense(x: wp.array2d(dtype=float), W: wp.array2d(dtype=float), b: wp.array(dtype=float),
           act: int, y: wp.array2d(dtype=float)):
    w, j = wp.tid()
    s = b[j]
    for k in range(W.shape[1]):
        s += W[j, k] * x[w, k]
    if act == 1:
        s = wp.tanh(s)
    elif act == 2:
        s = wp.max(s, 0.0)
    y[w, j] = s


@wp.kernel
def _clip_act(y: wp.array2d(dtype=float), act: wp.array2d(dtype=float)):
    w, j = wp.tid()
    act[w, j] = wp.clamp(y[w, j], -1.0, 1.0)


@wp.kernel
def _tally(term: wp.array(dtype=int), trunc: wp.array(dtype=int), rew: wp.array(dtype=float),
           falls: wp.array(dtype=int), ends: wp.array(dtype=int), rsum: wp.array(dtype=float)):
    w = wp.tid()
    wp.atomic_add(falls, 0, term[w])
    wp.atomic_add(ends, 0, wp.max(term[w], trunc[w]))
    wp.atomic_add(rsum, 0, rew[w])


class WarpMLP:
    """The exported policy's deterministic mean, clipped to [-1, 1]: what
    fly_policy.py feeds the env (`np.clip(pol.mean(o), -1, 1)`)."""

    def __init__(self, path, n, device):
        pol = load_policy_npz(path)
        f = lambda a: wp.array(np.asarray(a, np.float32), dtype=float, device=device)
        self.pol = pol
        self.mean = f(pol.obs_mean)
        self.inv_std = f(1.0 / np.sqrt(pol.obs_var + 1e-8))
        self.clip = float(pol.obs_clip)
        d = np.load(path)
        if "arch" in d:
            raise NotImplementedError("memory policies (MinGRU)")
        self.act = ACT[str(d["act"]) if "act" in d else "tanh"]     # as load_policy_npz
        self.layers = [(wp.array2d(np.asarray(W, np.float32), dtype=float, device=device), f(b))
                       for W, b in pol.layers]
        self.x = wp.zeros((n, pol.obs_dim), dtype=float, device=device)
        self.h = [wp.zeros((n, W.shape[0]), dtype=float, device=device) for W, _ in self.layers]
        self.n, self.device = n, device

    def __call__(self, obs, out):
        n, dev = self.n, self.device
        wp.launch(_normalize, (n, self.x.shape[1]), [obs, self.mean, self.inv_std, self.clip, self.x],
                  device=dev)
        x = self.x
        for i, ((W, b), y) in enumerate(zip(self.layers, self.h)):
            last = i == len(self.layers) - 1
            wp.launch(_dense, (n, W.shape[0]), [x, W, b, 0 if last else self.act, y], device=dev)
            x = y
        wp.launch(_clip_act, (n, 3), [x, out], device=dev)


def _gpu_mem_mb():
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=10)
        return int(r.stdout.split()[0])
    except Exception:
        return None


class Runner:
    def __init__(self, n, policy, seed, device):
        t = time.perf_counter()
        self.env = GpuGeneralEnv(CONFIG, nworld=n, seed=seed, device=device)
        self.build_s = time.perf_counter() - t
        self.n, self.device = n, device
        self.act = wp.zeros((n, 3), dtype=float, device=device)
        self.mlp = WarpMLP(policy, n, device) if policy else None
        self.rng = wp.array(np.random.default_rng(seed).integers(1, 2**31, n, dtype=np.uint32),
                            dtype=wp.uint32, device=device)
        self.falls = wp.zeros(1, dtype=int, device=device)
        self.ends = wp.zeros(1, dtype=int, device=device)
        self.rsum = wp.zeros(1, dtype=float, device=device)
        t = time.perf_counter()
        self.env.reset()
        wp.synchronize_device(device)
        self.reset_s = time.perf_counter() - t
        self.graph = None

    def step(self):
        e = self.env
        if self.mlp is not None:
            self.mlp(e.obs, self.act)
        else:
            wp.launch(_rand_act, self.n, [self.rng, self.act], device=self.device)
        _, r, term, trunc = e.step(self.act)
        wp.launch(_tally, self.n, [term, trunc, r, self.falls, self.ends, self.rsum], device=self.device)

    def capture(self):
        t = time.perf_counter()
        with wp.ScopedCapture(device=self.device) as cap:
            self.step()
        self.graph = cap.graph
        return time.perf_counter() - t

    def run(self, k):
        for _ in range(k):
            if self.graph is not None:
                wp.capture_launch(self.graph)
            else:
                self.step()

    def tally(self, reset=True):
        out = {"falls": int(self.falls.numpy()[0]), "ends": int(self.ends.numpy()[0]),
               "rsum": float(self.rsum.numpy()[0])}
        if reset:
            self.falls.zero_(), self.ends.zero_(), self.rsum.zero_()
        return out


def timed(run, k):
    wp.synchronize_device(run.device)
    t = time.perf_counter()
    run.run(k)
    wp.synchronize_device(run.device)
    return time.perf_counter() - t


def bench(n, policy, steps, eager_steps, seed, device):
    r = Runner(n, policy, seed, device)
    rec = {"nworld": n, "policy": policy or "random", "build_s": round(r.build_s, 2),
           "reset_s": round(r.reset_s, 2)}
    rec["first_step_s"] = round(timed(r, 1), 2)              # kernel compile / module load
    dt = timed(r, eager_steps)
    rec["eager_policy_sps"] = round(n * eager_steps / dt)
    rec["capture_s"] = round(r.capture(), 2)
    timed(r, 2)                                               # first graph launches
    r.tally()
    dt = timed(r, steps)
    t = r.tally()
    rec.update(graph_policy_sps=round(n * steps / dt), graph_physics_sps=round(n * steps * r.env.substeps / dt),
               steps=steps, wall_s=round(dt, 2), falls=t["falls"], episode_ends=t["ends"],
               reward_per_step=round(t["rsum"] / (n * steps), 4), gpu_mem_mb=_gpu_mem_mb())
    return rec


def check(policy, n, steps, seed, device):
    """Capture changes nothing: the same seed flown eager and as a graph.
    MJWarp's atomics may reorder sums, so equality is not expected bit for
    bit; falls and reward per step are the test, as in fly_policy.py."""
    out = {}
    for mode in ("eager", "graph"):
        r = Runner(n, policy, seed, device)
        if mode == "graph":
            r.step()                                          # compile before capture
            r.capture()
            r.run(steps - 1)
        else:
            r.run(steps)
        wp.synchronize_device(device)
        t = r.tally()
        out[mode] = {"falls": t["falls"], "episode_ends": t["ends"],
                     "reward_per_step": round(t["rsum"] / (n * steps), 4),
                     "obs_mean_abs": float(np.abs(r.env.obs.numpy()).mean())}
    # the Warp MLP against the numpy policy on the final observations
    if policy:
        pol = load_policy_npz(policy)
        r.mlp(r.env.obs, r.act)
        a_w = r.act.numpy()
        a_np = np.stack([np.clip(pol.mean(o), -1, 1) for o in r.env.obs.numpy()])
        out["mlp_max_err"] = float(np.abs(a_w - a_np).max())
    return {"check": True, "nworld": n, "steps": steps, **out}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--nworld", type=int, nargs="+", default=[1024, 4096, 16384, 32768])
    ap.add_argument("--policy", default=None, help="an exported .npz; random actions if omitted")
    ap.add_argument("--steps", type=int, default=200, help="timed graph launches")
    ap.add_argument("--eager-steps", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--check", action="store_true", help="eager vs captured, v3_s0, 64 worlds")
    ap.add_argument("--out", default="results/phase2.jsonl")
    a = ap.parse_args()
    wp.config.quiet = True
    wp.init()
    dev = "cuda:0" if wp.is_cuda_available() else "cpu"

    def emit(rec):
        rec["device"] = str(wp.get_device(dev).name)
        line = json.dumps(rec)
        print(line, flush=True)
        try:
            with open(a.out, "a") as f:
                f.write(line + "\n")
        except OSError:
            pass

    if a.check:
        emit(check(a.policy or "moves/general_rl_v3_s0.npz", 64, 250, a.seed, dev))
    else:
        for n in a.nworld:
            emit(bench(n, a.policy, a.steps, a.eager_steps, a.seed, dev))
