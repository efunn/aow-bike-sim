#!/bin/bash
# The whole phase 3 pod job, launched ONCE, detached, from ~/aow on the pod
# (docs/plans/gpu-training.md, phase 3, "The next run"):
#
#   ssh ... 'cd ~/aow && (nohup bash analysis/gpu/phase3/pod_launch.sh \
#            > launch.log 2>&1 < /dev/null &)'
#
# setup (fails fast if CUDA is unusable) -> the Warp kernel cache from
# ~/warp-cache.tgz if one was copied up -> run_pod.sh (export, checks,
# build, TRAIN_S of training) -> PufferLib's checkpoints into results/.
# results/ALL_DONE is written LAST, success or not: the local waiter
# blocks on it. (run_pod.sh's own trap writes results/DONE earlier --
# never wait on that one here.)
TRAIN_S=${TRAIN_S:-3600}
cd ~/aow
mkdir -p results
{
  bash analysis/gpu/phase1/pod_setup.sh > results/setup.log 2>&1 && . ~/venv/bin/activate && {
    if [ -f ~/warp-cache.tgz ]; then
      t0=$(date +%s.%N)
      mkdir -p ~/.cache && tar xzf ~/warp-cache.tgz -C ~/.cache
      echo "{\"step\":\"warp_cache_restore\",\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $t0}"),\"files\":$(find ~/.cache/warp -type f | wc -l)}" >> results/timings.jsonl
    fi
    TRAIN_S=$TRAIN_S bash analysis/gpu/phase3/run_pod.sh > results/phase3.log 2>&1
    cp -r ~/puffer/checkpoints results/puffer_checkpoints 2>/dev/null
  }
} ; echo ALL > results/ALL_DONE
