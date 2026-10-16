#!/bin/bash
# The scratch-reuse checks on a pod (gpu/scratch.py), from ~/aow, detached:
#
#   ssh ... 'cd ~/aow && (nohup bash analysis/gpu/phase3/scratch_pod.sh \
#            > scratch.log 2>&1 < /dev/null &)'
#
# setup -> check_scratch.py (reuse vs none, 512 worlds) -> its --mem at
# MEM_NW -> results/scratch_DONE, success or not. Read results/check_scratch.log
# BEFORE trusting a bench on the new export.
MEM_NW=${MEM_NW:-16384 32768 65536}
cd ~/aow
mkdir -p results
{
  bash analysis/gpu/phase1/pod_setup.sh > results/setup.log 2>&1 && . ~/venv/bin/activate && {
    export PATH=/usr/local/cuda/bin:$PATH PYTHONPATH=src
    pip install -q gymnasium==1.3.0 scipy pyyaml > results/pip_phase3.log 2>&1
    if [ -f ~/warp-cache.tgz ]; then mkdir -p ~/.cache && tar xzf ~/warp-cache.tgz -C ~/.cache; fi
    python analysis/gpu/phase3/check_scratch.py --nworld 512 --steps 200 > results/check_scratch.log 2>&1
    python analysis/gpu/phase3/check_scratch.py --mem $MEM_NW >> results/check_scratch.log 2>&1
  }
} ; echo DONE > results/scratch_DONE
