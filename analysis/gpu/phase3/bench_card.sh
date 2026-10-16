#!/bin/bash
# Which GPU? One card's speed, the same way on every card
# (docs/plans/gpu-training.md). Launched ONCE, detached, from ~/aow on a pod:
#
#   ssh ... 'cd ~/aow && (nohup bash analysis/gpu/phase3/bench_card.sh \
#            > bench.log 2>&1 < /dev/null &)'
#
#   PARTS="env 4096 16384 32768 65536"   (default) -- any subset, in order:
#     env     bench_env.py, env only (v3_s0's MLP inside the graph) at
#             1k-64k worlds (BENCH_NW): the A40's phase 2 rows stop at 32k,
#             and the fixed-latency vs per-world split
#     <N>     a real `puffer train` at N worlds for TRAIN_S (default 240) s:
#             end to end, the learner and the PufferLib path included
#
# Each part's results land in results/bench_<part>/; results/ALL_DONE is
# written LAST, success or not. Setup and the Warp cache as pod_launch.sh;
# a card of another architecture (sm_89 Ada, sm_120 Blackwell) recompiles
# the kernels once (~1 min), which the timings record.
TRAIN_S=${TRAIN_S:-240}
PARTS=${PARTS:-env 4096 16384 32768 65536}
export BENCH_NW=${BENCH_NW:-1024 4096 16384 32768 65536}
cd ~/aow
mkdir -p results
{
  bash analysis/gpu/phase1/pod_setup.sh > results/setup.log 2>&1 && . ~/venv/bin/activate && {
    nvidia-smi --query-gpu=name,clocks.max.sm,memory.total,driver_version --format=csv > results/gpu.txt
    if [ -f ~/warp-cache.tgz ]; then
      mkdir -p ~/.cache && tar xzf ~/warp-cache.tgz -C ~/.cache
    fi
    for part in $PARTS; do
      if [ "$part" = env ]; then
        # run_pod.sh (phase 2) writes into ./results; give it its own
        ( mv results results_keep && mkdir results && \
          bash analysis/gpu/phase2/run_pod.sh bench > results/run.log 2>&1 ; \
          mv results "results_keep/bench_env" ; mv results_keep results )
      else
        ( mv results results_keep && mkdir results && \
          NW=$part TRAIN_S=$TRAIN_S SKIP_CHECKS=1 bash analysis/gpu/phase3/run_pod.sh > results/phase3.log 2>&1 ; \
          mv results "results_keep/bench_$part" ; mv results_keep results )
      fi
    done
  }
} ; echo ALL > results/ALL_DONE
