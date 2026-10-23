#!/bin/bash
# SEVERAL full training runs at once on one GPU, under NVIDIA MPS -- what
# pod_launch.sh runs when SEEDS > 1 (docs/plans/gpu-training.md, "Several
# runs on one card"). Run from the copied tree's root after
# analysis/gpu/phase1/pod_setup.sh, inside its venv. The one-run path is
# run_pod.sh and is untouched by this file.
#
#   SEEDS=4 bash analysis/gpu/phase3/run_seeds.sh
#
#   SEEDS=4        runs; run k (1..SEEDS) has env seed ENV_SEED+k-1 and
#                  PufferLib seed PUFFER_SEED+k-1, so run 1 is the seed pair
#                  a one-run launch uses (0, 42)
#   MPS=1          start the MPS daemon (0: plain processes, which time-slice
#                  the card and gain nothing -- measured 2026-10-23)
#   TRAIN_S=7200   a wall-time cap per run; the ini's total_timesteps (80M)
#                  normally ends each run first. 4 x 80M ~46 min on a 5090
#   NW, HORIZON, CONFIG, CURRICULUM_LEAD, SKIP_CHECKS: as run_pod.sh, the
#                  same for every run
#
# Every run gets its own graph (graphs/aow_bike_<NW>_s<k>), its own copy of
# ~/puffer (~/puffer_s<k>; one build, so every run must share the obs
# width), and results/seed_<k>/ with train.log, watch.log, the ini, the
# graph json and puffer_checkpoints/. results/seed_<k>/DONE when it ends.
export PATH=/usr/local/cuda/bin:$PATH
export PYTHONPATH=src
mkdir -p results graphs
trap 'echo DONE > results/DONE' EXIT
step() {  # as run_pod.sh
    local name=$1; shift
    local t0=$(date +%s.%N)
    "$@" > "results/$name.log" 2>&1
    local rc=$?
    echo "{\"step\":\"$name\",\"rc\":$rc,\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $t0}")}" | tee -a results/timings.jsonl
    tail -3 "results/$name.log"
    return $rc
}
SEEDS=${SEEDS:-4}
MPS=${MPS:-1}
NW=${NW:-4096}
TRAIN_S=${TRAIN_S:-7200}
ENV_SEED=${ENV_SEED:-0}
PUFFER_SEED=${PUFFER_SEED:-42}
CONFIG=${CONFIG:-config/rl/rl_general_v3.yaml}
CURRICULUM_LEAD=${CURRICULUM_LEAD:-$(awk "BEGIN{print 0.1 * $NW / 4096}")}
step pip_phase3 pip install -q gymnasium==1.3.0 scipy pyyaml
for k in $(seq 1 $SEEDS); do
    step export_s$k python -m aow_sim.gpu.export_graph --config $CONFIG --nworld $NW \
        --out graphs/aow_bike_${NW}_s$k --seed $((ENV_SEED + k - 1)) \
        --curriculum-lead $CURRICULUM_LEAD || exit 1
done
G1=graphs/aow_bike_${NW}_s1
step build bash src/aow_sim/gpu/puffer/install.sh ~/puffer $G1 &
BUILD_PID=$!
if [ -z "${SKIP_CHECKS:-}" ]; then
step check_v3s0   python analysis/gpu/phase3/check_graph.py $G1 --steps 300
step check_random python analysis/gpu/phase3/check_graph.py $G1 --steps 100 --random
fi
wait $BUILD_PID || { echo "build failed: results/build.log"; exit 1; }
if [ -n "${HORIZON:-}" ]; then
    sed -i "s/^horizon = .*/horizon = $HORIZON/" ~/puffer/config/aow_bike.ini
fi
for k in $(seq 1 $SEEDS); do
    rm -rf ~/puffer_s$k && cp -r ~/puffer ~/puffer_s$k && rm -rf ~/puffer_s$k/checkpoints ~/puffer_s$k/logs
    sed -i -e "s#^graph = .*#graph = $(realpath graphs/aow_bike_${NW}_s$k)#" \
        -e "s/^seed = .*/seed = $((PUFFER_SEED + k - 1))/" ~/puffer_s$k/config/aow_bike.ini
    mkdir -p results/seed_$k
    grep -n '^seed\|^horizon\|^total_agents\|^graph' ~/puffer_s$k/config/aow_bike.ini \
        | tee results/seed_$k/ini.txt | tr '\n' ' ' >> results/build.log; echo >> results/build.log
done
if [ "$MPS" = 1 ] && [ "$SEEDS" -gt 1 ]; then
    export CUDA_MPS_PIPE_DIRECTORY=/tmp/mps_pipe CUDA_MPS_LOG_DIRECTORY=/tmp/mps_log
    mkdir -p $CUDA_MPS_PIPE_DIRECTORY $CUDA_MPS_LOG_DIRECTORY
    nvidia-cuda-mps-control -d && echo "MPS on" | tee -a results/build.log \
        || echo "MPS daemon did not start: runs time-slice" | tee -a results/build.log
fi
t0=$(date +%s.%N)
for k in $(seq 1 $SEEDS); do
    python3 -u analysis/gpu/phase3/watch.py results/seed_$k/train.log --every 25 \
        --until results/seed_$k/DONE > results/seed_$k/watch.log 2>&1 < /dev/null &
    ( cd ~/puffer_s$k && timeout $TRAIN_S ./puffer train --headless \
          > ~/aow/results/seed_$k/train.log 2>&1
      echo "{\"step\":\"train_s$k\",\"rc\":$?,\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $t0}")}" \
          | tee -a ~/aow/results/timings.jsonl
      sleep 5; echo DONE > ~/aow/results/seed_$k/DONE ) &
done
wait
[ -n "${CUDA_MPS_PIPE_DIRECTORY:-}" ] && echo quit | nvidia-cuda-mps-control
for k in $(seq 1 $SEEDS); do
    cp -r ~/puffer_s$k/checkpoints results/seed_$k/puffer_checkpoints 2>/dev/null
    cp -r ~/puffer_s$k/logs results/seed_$k/puffer_logs 2>/dev/null
    cp ~/puffer_s$k/config/aow_bike.ini results/seed_$k/
    cp graphs/aow_bike_${NW}_s$k.json results/seed_$k/
done
step warp_cache tar czf results/warp-cache.tgz -C ~/.cache warp
