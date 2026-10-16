"""Where does the trainer's ~0.7 MB a world go? One mode per process:
  build    GpuGeneralEnv + ordinary ScopedCapture of one step, then launch
  apic     the same with ScopedCapture(apic=True), as export_graph does
  load     wp.capture_load of the saved .wrp (the loader PufferLib's side uses)
Prints this process's GPU memory (nvidia-smi) after each stage.
  PYTHONPATH=src python mem_probe.py MODE NWORLD
"""
import os
import subprocess
import sys

import numpy as np
import warp as wp


def mem(tag):
    wp.synchronize()
    r = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory",
                        "--format=csv,noheader,nounits"], capture_output=True, text=True)
    mine = [l for l in r.stdout.splitlines() if l.split(",")[0].strip() == str(os.getpid())]
    print(f"{tag:28s} {mine[0].split(',')[1].strip() if mine else '?':>7s} MiB", flush=True)


mode, n = sys.argv[1], int(sys.argv[2])
wp.config.quiet = True
wp.init()
dev = "cuda:0"
mem("init")
if mode in ("build", "apic"):
    from aow_sim.gpu.env import GpuGeneralEnv
    env = GpuGeneralEnv("config/rl_general_v3.yaml", nworld=n, seed=0, device=dev,
                        shared_curriculum_envs=32, curriculum_lead=0.1)
    env.reset()
    mem("env built + reset")
    act = wp.zeros((n, 3), dtype=float, device=dev)
    env.step(act)
    mem("one eager step")
    with wp.ScopedCapture(device=dev, apic=(mode == "apic")) as cap:
        env.step(act)
    mem("captured")
    for _ in range(3):
        wp.capture_launch(cap.graph)
    mem("3 graph launches")
else:
    g = wp.capture_load(f"graphs/aow_bike_{n}", device=dev)
    mem("loaded")
    for _ in range(3):
        wp.capture_launch(g)
    mem("3 graph launches")
