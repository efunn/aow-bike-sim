"""Time the PPO update phase alone -- the part of a rollout that runs after
collection, single-process, while all n_envs workers sit idle.

  python scripts/bench_update.py                 # uses config/rl_general.yaml
  OMP_NUM_THREADS=8 python scripts/bench_update.py --batch 256,1024

Two [128,128] nets (pi + vf) and Adam over an n_steps x n_envs buffer, which
is the shape of what SB3 does; it is not SB3's actual loss, so read it as an
upper bound on how fast that phase could possibly go on this box. Touches no
files and no env, so it is safe to run beside a training run.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import torch

from aow_sim.control.general_env import _load_rl_config
from aow_sim.control.general_spec import ACT_DIM, OBS_DIM


def net(arch: list[int], out: int) -> torch.nn.Sequential:
    layers, prev = [], OBS_DIM
    for h in arch:
        layers += [torch.nn.Linear(prev, h), torch.nn.Tanh()]
        prev = h
    return torch.nn.Sequential(*layers, torch.nn.Linear(prev, out))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--batch", default="", help="override batch sizes, comma-separated")
    ap.add_argument("--config", default="config/rl_general.yaml")
    args = ap.parse_args()

    a = _load_rl_config(Path(args.config))["algo"]
    buf = a["n_steps"] * a["n_envs"]
    arch = list(a["net_arch"])
    epochs = a["n_epochs"]
    batches = ([int(x) for x in args.batch.split(",")] if args.batch
               else [a["batch_size"], a["batch_size"] * 4])

    print(f"buffer {buf} ({a['n_envs']} envs x {a['n_steps']} steps), "
          f"net {arch}, {epochs} epochs, torch threads {torch.get_num_threads()}")
    print(f"{'batch':>7} {'grad steps':>11} {'total':>8} {'per step':>9}")
    for bs in batches:
        pi, vf = net(arch, ACT_DIM), net(arch, 1)
        opt = torch.optim.Adam([*pi.parameters(), *vf.parameters()],
                               lr=a["learning_rate"])
        x = torch.randn(buf, OBS_DIM)

        def one(mb: torch.Tensor) -> None:
            loss = pi(mb).square().mean() + vf(mb).square().mean()
            opt.zero_grad()
            loss.backward()
            opt.step()

        for _ in range(3):
            one(x[:bs])                                  # warm up
        t = time.perf_counter()
        for _ in range(epochs):
            for i in range(0, buf, bs):
                one(x[i:i + bs])
        dt = time.perf_counter() - t
        n = epochs * -(-buf // bs)
        print(f"{bs:>7} {n:>11} {dt:>7.2f}s {dt / n * 1000:>8.2f}ms")


if __name__ == "__main__":
    main()
