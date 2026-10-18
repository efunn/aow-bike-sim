"""Save one GpuGeneralEnv policy step as a Warp APIC graph (`.wrp`) for a
trainer that runs no Python (PufferLib, phase 3 of docs/plans/gpu-training.md).

The env is built for N worlds, reset (so the saved state is every world at
the start of an episode, training randomization drawn), and one `step` is
captured: action in -> 50 substeps with the hooks -> obs, reward, flags,
episode summaries, auto-reset. `capture_save` snapshots every buffer, so
the file IS the env: loading it from C (`wp_apic_load_graph`) and launching
its exec steps all N worlds, the state living on in the loaded buffers.

Bound buffers, reachable by name from C (`wp_apic_get_param_ptr`):

  act_in      (N, 3) float32  in: the action, normalized [-1, 1] (clipped inside)
  obs         (N, 17) float32 out: the observation (general_spec, v3 layout;
              20 wide with obs_style: the style bit, sin/cos wheel error)
  reward      (N,) float32
  terminated  (N,) int32      a fall
  truncated   (N,) int32      the episode's time limit
  ep_end      (N,) int32      1 on the step an episode ended (either way)
  last_ret / last_len / last_track / last_fell / last_diff
              the finished episode's return, length [policy steps], tracking
              score, fall (int32) and curriculum difficulty; valid where ep_end

After a step, obs is already the NEXT episode's first observation where
ep_end is set (auto-reset inside the step), as PufferLib expects.

  python -m aow_sim.gpu.export_graph --nworld 4096 --out graphs/aow_bike_4096

GPU memory: every MJWarp call in the step shares one set of working memory
(scratch.py; --no-reuse-scratch turns it off). Without it the APIC graph
keeps each of the ~50 calls' own: 0.72 MB a world, 12.3 GB at 16k worlds.
analysis/gpu/phase3/check_scratch.py compares the two.

Writes <out>.wrp, <out>_modules/ (the compiled kernels) and <out>.json (the
buffers, the config and its hash, versions). On the CPU device a capture
executes as it records, so the saved state is one step past the reset;
only CUDA exports are for training.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from contextlib import nullcontext
from pathlib import Path

import warp as wp

OUTPUTS = ("obs", "reward", "terminated", "truncated", "ep_end", "last_ret",
           "last_len", "last_track", "last_fell", "last_diff", "diff_shared")


def export(config: str, nworld: int, out: str, seed: int = 0, device=None,
           curriculum_envs: int = 32, curriculum_lead: float = 0.1,
           reuse_scratch: bool = True) -> dict:
    import mujoco
    import mujoco_warp

    from .env import GpuGeneralEnv
    from .scratch import MjwScratch

    # Training graphs share one curriculum difficulty, paced as v3's 32 SB3
    # envs (env.py, shared_curriculum_envs); 0 = GeneralEnv's per-world rule.
    # Credit only from episodes played within curriculum_lead of it (env.py).
    env = GpuGeneralEnv(config, nworld=nworld, seed=seed, device=device,
                        shared_curriculum_envs=curriculum_envs,
                        curriculum_lead=curriculum_lead if curriculum_envs > 0 else 0.0)
    act_in = wp.zeros((nworld, 3), dtype=float, device=device)
    # reuse_scratch: every MJWarp call in the step shares ONE set of working
    # memory (scratch.py) -- the APIC capture otherwise keeps all ~50 calls'
    # own, 0.72 MB a world (12.3 GB at 16k worlds, measured on an A40). The
    # warm-up step below sizes the slots, outside the capture; freeze() makes
    # a captured allocation they cannot serve an error, not a leak.
    dev = wp.get_device(device)
    scratch = MjwScratch(dev) if reuse_scratch and dev.is_cuda else None
    env.scratch = scratch
    with wp.ScopedAllocator(dev, scratch) if scratch is not None else nullcontext():
        env.reset()
        env.step(act_in)            # compile every kernel and module before the capture
        env.reset()                 # back to fresh episodes everywhere
        wp.synchronize_device(device)
        if scratch is not None:
            scratch.freeze()
        with wp.ScopedCapture(device=device, apic=True) as cap:
            env.step(act_in)
    outs = {k: getattr(env, k) for k in OUTPUTS}
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    wp.capture_save(cap.graph, out, inputs={"act_in": act_in}, outputs=outs)
    bufs = {k: {"dtype": a.dtype.__name__, "shape": list(a.shape)}
            for k, a in {"act_in": act_in, **outs}.items()}
    meta = {
        "nworld": nworld, "obs_dim": int(env.obs.shape[1]), "act_dim": 3,
        "config": config, "config_sha256": hashlib.sha256(Path(config).read_bytes()).hexdigest(),
        "seed": seed, "device": str(wp.get_device(device)),
        "substeps": env.substeps, "ctrl_dt": env.cpu.ctrl_dt,
        "max_steps": int(env.cpu.max_steps), "buffers": bufs,
        "scratch_reuse": ({"slots": len(scratch.slots), "mb": round(scratch.nbytes / 1e6, 1)}
                          if scratch is not None else None),
        "curriculum": ({"shared": True, "envs": curriculum_envs, "lead": env.cur_lead}
                       if curriculum_envs > 0
                       else {"shared": False}),
        "versions": {"warp": wp.__version__, "mujoco": mujoco.__version__,
                     "mujoco_warp": getattr(mujoco_warp, "__version__", "?")},
    }
    Path(out + ".json").write_text(json.dumps(meta, indent=1))
    return meta


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/rl_general_v3.yaml")
    ap.add_argument("--nworld", type=int, default=4096)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="graphs/aow_bike")
    ap.add_argument("--curriculum-envs", type=int, default=32,
                    help="shared difficulty paced as this many per-env curricula; 0 = per world")
    ap.add_argument("--curriculum-lead", type=float, default=0.1,
                    help="credit only episodes played within this of the shared value; 0 = off")
    ap.add_argument("--no-reuse-scratch", action="store_true",
                    help="give every MJWarp call its own working memory, as before "
                         "2026-10-16 (~12x the GPU memory; for the A/B check)")
    a = ap.parse_args()
    wp.config.quiet = True
    wp.init()
    meta = export(a.config, a.nworld, a.out, a.seed, curriculum_envs=a.curriculum_envs,
                  curriculum_lead=a.curriculum_lead, reuse_scratch=not a.no_reuse_scratch)
    size = sum(f.stat().st_size for f in Path(a.out).parent.rglob("*")
               if f.is_file() and f.name.startswith(Path(a.out).name))
    print(json.dumps({"saved": a.out, "nworld": meta["nworld"], "device": meta["device"],
                      "mb": round(size / 1e6, 1)}))
