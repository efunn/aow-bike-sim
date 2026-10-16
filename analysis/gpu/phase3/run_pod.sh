#!/bin/bash
# Phase 3 on a pod (docs/plans/gpu-training.md): the PufferLib bridge. Run
# from the copied tree's root AFTER analysis/gpu/phase1/pod_setup.sh, inside
# its venv. No git push, no tokens (PufferLib is cloned, public). Every step
# is timed into results/timings.jsonl, logs in results/<step>.log.
#
#   bash analysis/gpu/phase3/run_pod.sh            # export, check, build, train
#   NW=16384 bash analysis/gpu/phase3/run_pod.sh   # another world count
#   ENV_SEED=1 PUFFER_SEED=43 bash ...              # another seed: BOTH halves
#
# Two seeds: ENV_SEED is baked into the exported graph (commands, domain
# draws; export_graph --seed, default 0) and PUFFER_SEED is aow_bike.ini's
# [base] seed (policy init, action sampling; 42 as shipped). Neither makes a
# run repeatable -- float atomics in the shared curriculum and MJWarp's
# atomically numbered constraint rows vary with scheduling -- but a new
# seed should change both.
#
# Order: export the env graph -> fly v3_s0 through the RELOADED graph (and
# random actions, for the reset path) -> build PufferLib with the env ->
# train headless for TRAIN_S seconds (default 600) -> the warp cache.
export PATH=/usr/local/cuda/bin:$PATH
export PYTHONPATH=src
mkdir -p results graphs
trap 'echo DONE > results/DONE' EXIT   # the done-file wait must fire on a failure too
step() {  # step <name> <cmd...>; keeps going on failure, records the status
    local name=$1; shift
    local t0=$(date +%s.%N)
    "$@" > "results/$name.log" 2>&1
    local rc=$?
    echo "{\"step\":\"$name\",\"rc\":$rc,\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $t0}")}" | tee -a results/timings.jsonl
    tail -3 "results/$name.log"
    return $rc
}
NW=${NW:-4096}
TRAIN_S=${TRAIN_S:-600}
ENV_SEED=${ENV_SEED:-0}
G=graphs/aow_bike_$NW
step pip_phase3 pip install -q gymnasium==1.3.0 scipy pyyaml
step export     python -m aow_sim.gpu.export_graph --nworld $NW --out $G --seed $ENV_SEED || exit 1
du -sh $G.wrp ${G}_modules | tee -a results/export.log
# SKIP_CHECKS=1 (the card benchmark, bench_card.sh): the checks fly every
# world, ~75 s at 4096 and far longer at 64k, and judge the graph, not the card
# The build (CPU, ~10 s once PufferLib is fetched) runs WHILE the checks fly
# (GPU, ~75 s at 4096): neither needs the other.
step build      bash src/aow_sim/gpu/puffer/install.sh ~/puffer $G &
BUILD_PID=$!
if [ -z "${SKIP_CHECKS:-}" ]; then
step check_v3s0   python analysis/gpu/phase3/check_graph.py $G --steps 300
step check_random python analysis/gpu/phase3/check_graph.py $G --steps 100 --random
fi
wait $BUILD_PID || { echo "build failed: results/build.log"; exit 1; }
if [ -n "${PUFFER_SEED:-}" ]; then
    sed -i "s/^seed = .*/seed = $PUFFER_SEED/" ~/puffer/config/aow_bike.ini
fi
grep -n '^seed' ~/puffer/config/aow_bike.ini | tee -a results/build.log
# the trainer, headless, for a fixed wall time; its own log is the result
( cd ~/puffer && step_t0=$(date +%s.%N) && \
  timeout $TRAIN_S ./puffer train --headless > $OLDPWD/results/train.log 2>&1; \
  echo "{\"step\":\"train\",\"rc\":$?,\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $step_t0}")}" \
      | tee -a $OLDPWD/results/timings.jsonl )
cp -r ~/puffer/logs results/puffer_logs 2>/dev/null
tail -40 results/train.log
step warp_cache tar czf results/warp-cache.tgz -C ~/.cache warp
