#!/bin/bash
# Phase 3 on a pod (docs/plans/gpu-training.md): the PufferLib bridge. Run
# from the copied tree's root AFTER analysis/gpu/phase1/pod_setup.sh, inside
# its venv. No git push, no tokens (PufferLib is cloned, public). Every step
# is timed into results/timings.jsonl, logs in results/<step>.log.
#
#   bash analysis/gpu/phase3/run_pod.sh            # export, check, build, train
#   NW=16384 bash analysis/gpu/phase3/run_pod.sh   # another world count
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
G=graphs/aow_bike_$NW
step pip_phase3 pip install -q gymnasium==1.3.0 scipy pyyaml
step export     python -m aow_sim.gpu.export_graph --nworld $NW --out $G || exit 1
du -sh $G.wrp ${G}_modules | tee -a results/export.log
step check_v3s0   python analysis/gpu/phase3/check_graph.py $G --steps 300
step check_random python analysis/gpu/phase3/check_graph.py $G --steps 100 --random
step build      bash src/aow_sim/gpu/puffer/install.sh ~/puffer $G || exit 1
# the trainer, headless, for a fixed wall time; its own log is the result
( cd ~/puffer && step_t0=$(date +%s.%N) && \
  timeout $TRAIN_S ./puffer train --headless > $OLDPWD/results/train.log 2>&1; \
  echo "{\"step\":\"train\",\"rc\":$?,\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $step_t0}")}" \
      | tee -a $OLDPWD/results/timings.jsonl )
cp -r ~/puffer/logs results/puffer_logs 2>/dev/null
tail -40 results/train.log
step warp_cache tar czf results/warp-cache.tgz -C ~/.cache warp
