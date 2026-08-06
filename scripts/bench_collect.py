"""Measure raw SubprocVecEnv collection throughput vs n_envs -- no PPO, no
policy, no checkpoints. Isolates "how well does env stepping parallelize on
this box" from the update and eval phases that also sit in a training rollout.

  python scripts/bench_collect.py                    # sweep 1,2,4,8,16,24,32
  python scripts/bench_collect.py --envs 8,16,32 --steps 32768

Writes nothing: it builds envs the same way train_general_rl._make_vecenv does,
steps them with random actions, and reports steps/s.

RUN THIS ON AN IDLE BOX. It cannot corrupt a training run -- but a training
run will corrupt it. Every row here is a claim about how many cores N workers
can keep busy, which is meaningless while n_envs other workers are already
holding those cores; the numbers come back low, noisy, and non-monotonic.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv

from aow_sim.build_model import load_params
from aow_sim.control.general_env import GeneralEnv, _load_rl_config

CGROUP_CPU = Path("/sys/fs/cgroup/cpu.stat")


def cpu_usec() -> int | None:
    """CPU-microseconds this cgroup has burned. None where unavailable.

    Deliberately not ps/vmstat: a container with a bogus /proc/stat btime
    reports garbage %CPU (elapsed comes out as ~441e6 days), while this
    counter is a plain monotonic total and stays honest.
    """
    try:
        for line in CGROUP_CPU.read_text().splitlines():
            if line.startswith("usage_usec"):
                return int(line.split()[1])
    except OSError:
        pass
    return None


def bench(params, cfg, n_envs: int, steps: int) -> tuple[float, float | None]:
    venv = SubprocVecEnv([
        (lambda i=i: Monitor(GeneralEnv(params, cfg, seed=i)))
        for i in range(n_envs)
    ])
    try:
        venv.reset()
        for _ in range(20):                      # warm up: JIT, page faults
            venv.step([venv.action_space.sample() for _ in range(n_envs)])

        iters = max(1, steps // n_envs)
        c0, t0 = cpu_usec(), time.perf_counter()
        for _ in range(iters):
            venv.step([venv.action_space.sample() for _ in range(n_envs)])
        wall = time.perf_counter() - t0
        c1 = cpu_usec()
        cores = (c1 - c0) / 1e6 / wall if c0 is not None and c1 is not None else None
        return iters * n_envs / wall, cores
    finally:
        venv.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--envs", default="1,2,4,8,16,24,32",
                    help="comma-separated n_envs values to sweep")
    ap.add_argument("--steps", type=int, default=16384,
                    help="env steps to time per row (default: one rollout)")
    ap.add_argument("--config", default="config/rl_general.yaml")
    args = ap.parse_args()

    params = load_params()
    cfg = _load_rl_config(Path(args.config))
    counts = [int(x) for x in args.envs.split(",")]

    print(f"{args.steps} steps per row, {args.config}")
    print(f"{'n_envs':>7} {'steps/s':>9} {'per env':>8} {'scaling':>8} {'cores':>7}")
    base = None
    for n in counts:
        fps, cores = bench(params, cfg, n, args.steps)
        base = base if base is not None else fps
        cores_s = f"{cores:7.1f}" if cores is not None else "      -"
        print(f"{n:>7} {fps:>9.0f} {fps / n:>8.1f} {fps / base:>7.2f}x {cores_s}")


if __name__ == "__main__":
    main()
