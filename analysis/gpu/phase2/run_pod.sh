#!/bin/bash
# Phase 2 on a pod (docs/plans/gpu-training.md, "The pod run"). Run from
# the copied tree's root AFTER analysis/gpu/phase1/pod_setup.sh, inside its
# venv. No git, no tokens. Every step is timed into results/timings.jsonl;
# results go to results/phase2.jsonl and results/eval_grid.json.
#
#   bash run_pod.sh                  # everything
#   bash run_pod.sh eval profile     # only the named parts: fly check bench eval profile
export PATH=/usr/local/cuda/bin:$PATH
export PYTHONPATH=src
mkdir -p results
trap 'echo DONE > results/DONE' EXIT   # the done-file wait must fire on a failure too
step() {  # step <name> <cmd...>; keeps going on failure, records the status
    local name=$1; shift
    local t0=$(date +%s.%N)
    "$@" > "results/$name.log" 2>&1
    local rc=$?
    echo "{\"step\":\"$name\",\"rc\":$rc,\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $t0}")}" | tee -a results/timings.jsonl
    tail -3 "results/$name.log"
}
P=analysis/gpu/phase2
WHAT=${*:-fly check bench eval profile}
want() { [[ " $WHAT " == *" $1 "* ]]; }
step pip_phase2 pip install -q gymnasium==1.3.0 scipy pyyaml
if want fly; then   # still flies on CUDA (the first run also compiles every kernel)
    step fly_cold   python $P/fly_policy.py --nworld 8 --steps 250
    step fly_warm   python $P/fly_policy.py --nworld 8 --steps 250
fi
if want check; then # the whole step as one graph changes nothing
    step check      python $P/bench_env.py --check
fi
if want bench; then # speed, random actions then v3_s0's MLP (both inside the graph)
    # BENCH_POLICIES: the drivers whose LOAD is timed (an MLP or a MinGRU,
    # moves/<name>.npz; must be in the bundle). The load follows the plant,
    # so keep the current best policy in the list when the physics changes.
    BENCH_NW=${BENCH_NW:-1024 4096 16384 32768}
    BENCH_POLICIES=${BENCH_POLICIES:-general_rl_v3_s0 general_puffer_fixes_52M}
    step bench_rand python $P/bench_env.py --nworld $BENCH_NW
    for pol in $BENCH_POLICIES; do
        [ -f moves/$pol.npz ] || { echo "no moves/$pol.npz in the bundle"; continue; }
        step check_$pol python $P/bench_env.py --check --policy moves/$pol.npz
        step bench_$pol python $P/bench_env.py --nworld $BENCH_NW --policy moves/$pol.npz
    done
fi
if want eval; then  # the eval grid, GPU (32 seeds a command) against GeneralEnv's _eval_episodes
    step eval_grid  python $P/eval_grid.py --seeds 32
fi
if want profile; then  # where a policy step's GPU time goes, and bare physics on this model
    step profile    python $P/profile_env.py --nworld 1024 4096 16384 32768
    # the whole-timeline view, if the image carries Nsight Systems
    if command -v nsys > /dev/null; then
        step nsys nsys profile -o results/nsys_env --force-overwrite true \
            python $P/bench_env.py --nworld 4096 --steps 20 --policy moves/general_rl_v3_s0.npz --out /dev/null
    fi
fi
# for the Dockerfile: the compiled kernels
step warp_cache tar czf results/warp-cache.tgz -C ~/.cache warp
