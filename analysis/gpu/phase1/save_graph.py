"""Bridge probe, Python half: capture K MJWarp steps for N worlds as one
Warp graph and save it (APIC, `.wrp`) for bridge_probe.cu to load with no
Python. Writes, beside the graph:

  <out>_ref.npz  qpos after ONE launch from the saved state (the C side must
                 reproduce it), plus the saved state itself, so the C side
                 can restore it through the bound params and launch again.

Bound names (what C can reach by `wp_apic_get_param_ptr`): the state a reset
writes (the steer delay line `history` included -- leaving it out was the
CPU env's reset bug of 2026-10-14), and ctrl. 50 steps x 0.4 ms = one 20 ms
control step.

  python save_graph.py --nworld 4096 --steps 50 --out wrp/bike
"""
import argparse
import json
import os
import time

import numpy as np
import warp as wp
import mujoco_warp as mjw

from probe_physics import NCONMAX, NJMAX, load

BOUND = ("qpos", "qvel", "act", "ctrl", "qacc_warmstart", "time", "history")


def main(a):
    wp.config.quiet = True
    wp.init()
    if a.no_mempool:      # mempool alloc/free nodes cannot sit in a child graph
        wp.set_mempool_enabled(wp.get_device(), False)
    mjm, mjd = load()
    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd, nworld=a.nworld, nconmax=NCONMAX, njmax=NJMAX)
    mjw.step(m, d)                                    # JIT, and a non-trivial state
    wp.synchronize()
    saved = {k: getattr(d, k).numpy().copy() for k in BOUND}

    with wp.ScopedCapture(apic=True) as cap:
        for _ in range(a.steps):
            mjw.step(m, d)
    if not wp.get_device().is_cuda:                   # CPU capture executes eagerly:
        for k in BOUND:                               # put the state back
            getattr(d, k).assign(saved[k])
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    arrays = {k: getattr(d, k) for k in BOUND}
    wp.capture_save(cap.graph, a.out, inputs=arrays, outputs=arrays)

    # the reference: one launch of the live graph from the saved state
    wp.capture_launch(cap.graph)
    wp.synchronize()
    ref = d.qpos.numpy().copy()

    # Python-side launch rate, for comparison with the C side
    reps = a.reps
    wp.synchronize()
    t = time.perf_counter()
    for _ in range(reps):
        wp.capture_launch(cap.graph)
    wp.synchronize()
    dt = time.perf_counter() - t

    np.savez(a.out + "_ref.npz", ref_qpos=ref, **{"saved_" + k: v for k, v in saved.items()})
    # the same as raw float32 for the C side: <out>_ref_qpos.bin, <out>_saved_<k>.bin
    ref.astype(np.float32).tofile(a.out + "_ref_qpos.bin")
    for k, v in saved.items():
        v.astype(np.float32).tofile(f"{a.out}_saved_{k}.bin")
    print(json.dumps(dict(
        probe="save_graph", device=str(wp.get_device()), nworld=a.nworld, steps=a.steps,
        bound={k: [str(getattr(d, k).dtype.__name__), list(getattr(d, k).shape),
                   int(getattr(d, k).capacity)] for k in BOUND},
        wrp_mb=round(sum(os.path.getsize(os.path.join(r, f))
                         for r, _, fs in os.walk(os.path.dirname(a.out) or ".")
                         for f in fs) / 1e6, 1),
        python_launch_physics_steps_per_s=round(a.nworld * a.steps * reps / dt),
        nan_worlds=int((~np.isfinite(ref).all(1)).sum()))), flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--nworld", type=int, default=4096)
    ap.add_argument("--steps", type=int, default=50)
    ap.add_argument("--reps", type=int, default=20)
    ap.add_argument("--out", default="wrp/bike")
    ap.add_argument("--no-mempool", action="store_true")
    main(ap.parse_args())
