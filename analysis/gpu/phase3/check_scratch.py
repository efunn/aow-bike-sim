"""Does sharing MJWarp's working memory (gpu/scratch.py) change the env?
And what does it save? CUDA only (Warp's custom allocators are).

    PYTHONPATH=src python analysis/gpu/phase3/check_scratch.py --nworld 512 --steps 200
    PYTHONPATH=src python analysis/gpu/phase3/check_scratch.py --mem 16384 65536

Default: export THREE graphs of the same seed -- two without the reuse (the
pre-2026-10-16 export) and one with -- and fly each, in its own process,
through the same seeded random actions (random, so worlds fall and the
reset path, with its mjw.forward, runs too). The two without set the noise
floor: MJWarp numbers its constraint rows with atomics and the curriculum
sums with float atomics, so two runs of the SAME graph need not agree to the
bit. The verdict: with-reuse against without must sit inside that floor.

--poison: THE correctness test. Two exports of one seed already differ at
step 1 (0.15-0.20 in obs, 4090, 2026-10-16), so comparing exports cannot
see a reuse bug through the noise. Instead: fly the env eagerly with the
reuse on and, after EVERY MJWarp call returns, fill every scratch slot with
NaN (0x7fc00000). A call that read anything an earlier call left in scratch
would read NaN, and it would reach the state; the safety argument in
scratch.py is exactly "nothing does". Same run unpoisoned alongside, for
the falls and reward.

--mem N...: export with the reuse at each N, load it in a fresh process as
PufferLib does (wp.capture_load), launch, and print the process's GPU memory
-- against 12.3 GB at 16k without (A40, 2026-10-16).
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()


def gpu_mib():
    r = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory",
                        "--format=csv,noheader,nounits"], capture_output=True, text=True)
    for line in r.stdout.splitlines():
        pid, mib = (x.strip() for x in line.split(","))
        if pid == str(os.getpid()):
            return int(mib)
    return None


def fly(stem, out, steps):
    """Child: load the graph, fly seeded random actions, save the trajectory."""
    import warp as wp
    sys.path.insert(0, str(HERE.parent))
    from check_graph import bind
    wp.config.quiet = True
    wp.init()
    meta = json.loads(Path(stem + ".json").read_text())
    g = wp.capture_load(stem, device="cuda:0")
    b = bind(g, meta, "cuda:0")
    n = meta["nworld"]
    rng = np.random.default_rng(0)
    obs, rew, ends = [], [], []
    for _ in range(steps):
        b["act_in"].assign(rng.uniform(-1, 1, (n, 3)).astype(np.float32))
        wp.capture_launch(g)
        obs.append(b["obs"].numpy())
        rew.append(b["reward"].numpy())
        ends.append(b["ep_end"].numpy())
    wp.synchronize()
    np.savez(out, obs=np.stack(obs), rew=np.stack(rew), ends=np.stack(ends))
    print(json.dumps({"stem": stem, "gpu_mib": gpu_mib(), "scratch": meta.get("scratch_reuse")}))


def poison(a):
    import warp as wp
    from contextlib import contextmanager
    from aow_sim.gpu.env import GpuGeneralEnv
    from aow_sim.gpu.scratch import MjwScratch
    wp.config.quiet = True
    wp.init()
    dev = wp.get_device("cuda:0")
    n = a.nworld

    class Poisoned(MjwScratch):
        on = False

        @contextmanager
        def call(self):
            with super().call():
                yield
            if self.on:
                for ptr, cap in self.slots:
                    wp.array(ptr=ptr, dtype=wp.uint32, shape=(cap // 4,),
                             device=dev).fill_(0x7FC00000)

    out = {}
    for poisoned in (False, True):
        env = GpuGeneralEnv("config/rl_general_v3.yaml", nworld=n, seed=0, device=dev,
                            shared_curriculum_envs=32, curriculum_lead=0.1)
        sc = Poisoned(dev)
        env.scratch = sc
        act = wp.zeros((n, 3), dtype=float, device=dev)
        rng = np.random.default_rng(0)
        nonfinite_step, falls, rsum = None, 0, 0.0
        with wp.ScopedAllocator(dev, sc):
            env.reset()
            env.step(act)
            sc.freeze()
            sc.on = poisoned
            for k in range(a.steps):
                act.assign(rng.uniform(-1, 1, (n, 3)).astype(np.float32))
                obs, rew, term, _ = env.step(act)
                o = obs.numpy()
                falls += int(term.numpy().sum())
                rsum += float(rew.numpy().sum())
                if nonfinite_step is None and not (np.all(np.isfinite(o))
                                                   and np.all(np.isfinite(env.d.qpos.numpy()))):
                    nonfinite_step = k
        out[poisoned] = (nonfinite_step, falls, rsum / (n * a.steps))
        print(f"poisoned={poisoned!s:5s}: first non-finite step {nonfinite_step}, "
              f"falls {falls}, reward/step {rsum / (n * a.steps):.4f}, "
              f"slots {len(sc.slots)} ({sc.nbytes / 1e6:.1f} MB)", flush=True)
    ok = out[True][0] is None and out[False][0] is None
    print(f"{a.steps} steps x {n} worlds, scratch poisoned after every MJWarp call: "
          f"{'PASS -- no call reads what an earlier one left' if ok else 'FAIL'}")


def export(n, stem, reuse, seed=0):
    cmd = [sys.executable, "-m", "aow_sim.gpu.export_graph", "--nworld", str(n),
           "--out", stem, "--seed", str(seed)] + ([] if reuse else ["--no-reuse-scratch"])
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)


def child(*args):
    r = subprocess.run([sys.executable, str(HERE), *args], check=True,
                       capture_output=True, text=True)
    return json.loads(r.stdout.strip().splitlines()[-1])


def compare(a):
    tmp = Path(tempfile.mkdtemp(prefix="check_scratch_"))
    runs = {"off_1": False, "off_2": False, "on": True}
    traj = {}
    for name, reuse in runs.items():
        stem = str(tmp / name)
        export(a.nworld, stem, reuse)
        info = child("--fly", stem, str(tmp / f"{name}.npz"), str(a.steps))
        traj[name] = np.load(tmp / f"{name}.npz")
        print(f"{name:6s} reuse={reuse!s:5s} gpu {info['gpu_mib']} MiB  scratch {info['scratch']}")

    def diff(x, y):
        d = {k: float(np.abs(traj[x][k].astype(float) - traj[y][k].astype(float)).max())
             for k in ("obs", "rew")}
        d["step1_obs"] = float(np.abs(traj[x]["obs"][0].astype(float) - traj[y]["obs"][0]).max())
        d["ends"] = int((traj[x]["ends"] != traj[y]["ends"]).sum())
        same = np.all(traj[x]["obs"] == traj[y]["obs"], axis=(1, 2))
        d["identical_steps"] = int(np.argmin(same)) if not same.all() else len(same)
        return d

    floor, test = diff("off_1", "off_2"), diff("on", "off_1")
    print(f"noise floor (off vs off): {floor}")
    print(f"reuse       (on  vs off): {test}")
    # A scratch bug (a later call reading a slot an earlier one left) shows
    # in the FIRST step: substep 2 already runs on reused memory. Later
    # steps may drift apart chaotically even off-vs-off, so the whole
    # trajectory is judged only against the floor.
    ok = (test["step1_obs"] <= floor["step1_obs"] + 1e-6
          and test["identical_steps"] >= min(floor["identical_steps"], a.steps)
          and test["obs"] <= floor["obs"] + 1e-6)
    tot_ends = int(traj["off_1"]["ends"].sum())
    print(f"{a.steps} steps x {a.nworld} worlds, {tot_ends} episode ends: "
          f"{'PASS' if ok else 'CHECK BY HAND'} (reuse no further from off than off is from itself)")


def mem(a):
    tmp = Path(tempfile.mkdtemp(prefix="check_scratch_mem_"))
    for n in a.mem:
        stem = str(tmp / f"g{n}")
        export(n, stem, True)
        info = child("--fly", stem, str(tmp / f"g{n}.npz"), "3")
        size = sum(f.stat().st_size for f in tmp.iterdir() if f.name.startswith(f"g{n}"))
        print(f"{n:6d} worlds, reuse on: {info['gpu_mib']} MiB on load+3 launches, "
              f"scratch {info['scratch']}, files {size / 1e6:.0f} MB")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--fly":
        fly(sys.argv[2], sys.argv[3], int(sys.argv[4]))
        sys.exit(0)
    ap = argparse.ArgumentParser()
    ap.add_argument("--nworld", type=int, default=512)
    ap.add_argument("--steps", type=int, default=200)
    ap.add_argument("--mem", type=int, nargs="*", help="world counts to measure memory at")
    ap.add_argument("--poison", action="store_true", help="the NaN-poison correctness test")
    a = ap.parse_args()
    poison(a) if a.poison else mem(a) if a.mem else compare(a)
