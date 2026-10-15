#!/bin/bash
# Phase 2 on a pod (docs/plans/gpu-training.md, "Next: the pod run"). Run from
# the copied tree's root AFTER analysis/gpu_phase1/pod_setup.sh, inside its
# venv. No git, no tokens. Every step is timed into results/timings.jsonl;
# results go to results/phase2.jsonl.
export PATH=/usr/local/cuda/bin:$PATH
export PYTHONPATH=src
mkdir -p results
step() {  # step <name> <cmd...>; keeps going on failure, records the status
    local name=$1; shift
    local t0=$(date +%s.%N)
    "$@" > "results/$name.log" 2>&1
    local rc=$?
    echo "{\"step\":\"$name\",\"rc\":$rc,\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $t0}")}" | tee -a results/timings.jsonl
    tail -3 "results/$name.log"
}
P=analysis/gpu_phase2
step pip_phase2 pip install -q gymnasium==1.3.0 scipy pyyaml
# 2. still flies on CUDA (first run also compiles every kernel: the cold cost)
step fly_cold   python $P/fly_policy.py --nworld 8 --steps 250
step fly_warm   python $P/fly_policy.py --nworld 8 --steps 250
# 3. the whole step as one graph changes nothing
step check      python $P/bench_env.py --check
# 4. speed, random actions then v3_s0's MLP (both inside the graph)
step bench_rand python $P/bench_env.py --nworld 1024 4096 16384 32768
step bench_mlp  python $P/bench_env.py --nworld 1024 4096 16384 32768 --policy moves/general_rl_v3_s0.npz
# for the Dockerfile: the compiled kernels
step warp_cache tar czf results/warp-cache.tgz -C ~/.cache warp
