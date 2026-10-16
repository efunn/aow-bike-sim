"""Every Warp allocation made INSIDE one env.step (after a warm-up step), with
the caller that made it: what an APIC capture would have to give its own
fixed memory. PYTHONPATH=src python alloc_probe.py NWORLD"""
import collections, sys, traceback
import warp as wp
from warp._src.context import CudaMempoolAllocator, CudaDefaultAllocator

wp.config.quiet = True
wp.init()
n = int(sys.argv[1])
from aow_sim.gpu.env import GpuGeneralEnv
env = GpuGeneralEnv("config/rl_general_v3.yaml", nworld=n, seed=0, device="cuda:0",
                    shared_curriculum_envs=32, curriculum_lead=0.1)
env.reset()
act = wp.zeros((n, 3), dtype=float, device="cuda:0")
env.step(act)
wp.synchronize()

rec = collections.Counter(); cnt = collections.Counter()
def where():
    for f in reversed(traceback.extract_stack()[:-2]):
        if "/warp/" not in f.filename:
            return f"{f.filename.split('site-packages/')[-1].split('/aow/')[-1]}:{f.lineno} {f.name}"
    return "?"
def wrap(cls):
    orig = cls.allocate
    def patched(self, size_in_bytes, *a, **k):
        w = where(); rec[w] += size_in_bytes; cnt[w] += 1
        return orig(self, size_in_bytes, *a, **k)
    cls.allocate = patched
for cls in (CudaMempoolAllocator, CudaDefaultAllocator):
    wrap(cls)
env.step(act)
wp.synchronize()
tot = sum(rec.values())
print(f"one step, {n} worlds: {sum(cnt.values())} allocations, {tot/2**20:.0f} MiB total, "
      f"{tot/n/1024:.0f} KiB per world")
for k, v in rec.most_common(15):
    print(f"  {v/2**20:9.1f} MiB  x{cnt[k]:<5d} {k}")
