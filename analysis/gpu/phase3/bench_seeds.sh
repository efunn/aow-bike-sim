#!/bin/bash
# Several independent trainings on ONE card at once: does the card's spare
# throughput at 4096 worlds come back as more runs per hour? (The 5090
# steps 16k worlds 1.87x faster than 4k, results_5090_bench, so one 4k run
# leaves it partly idle; 16k in ONE run learned no faster per epoch.)
#
# Run on a pod AFTER pod_launch.sh has finished (it needs ~/puffer built,
# the venv and the Warp cache), from ~/aow, detached:
#
#   ssh ... 'cd ~/aow && (nohup bash analysis/gpu/phase3/bench_seeds.sh \
#            > bench_seeds.log 2>&1 < /dev/null &)'
#
#   COUNTS="2 4 8"   how many runs at once, one count after the other
#   MPS_COUNTS="4 8" the same again under NVIDIA MPS ("" skips it)
#   TRAIN_S=120      seconds each count trains (SPS is read from 45 s on)
#   NW=4096          worlds per run
#   CONFIG=config/rl/rl_general_v3.yaml
#
# WHY MPS. Without it, processes on one GPU TIME-SLICE: each context gets
# the whole card in turn, and kernels from two processes never run at the
# same moment. So plain concurrency only fills the gaps one run leaves (its
# CPU side, the learner, launch latency). MPS funnels every process through
# one context, so small kernels from different runs can share the SMs --
# which is what a run at 4096 worlds leaves free. The daemon may not start
# in a container; the summary then has no mps rows and says why.
#
# Run k (1..max) gets its own graph (export_graph --seed k), its own copy
# of ~/puffer (~/puffer_s<k>, its ini's graph and [base] seed 42+k), so
# no two runs share a seed or a checkpoint dir. Logs:
# results/bench_seeds/n<N>/s<k>.log, the summary in
# results/bench_seeds/summary.txt, results/bench_seeds/ALL_DONE last.
COUNTS=${COUNTS:-2 4 8}
MPS_COUNTS=${MPS_COUNTS-4 8}
TRAIN_S=${TRAIN_S:-120}
NW=${NW:-4096}
CONFIG=${CONFIG:-config/rl/rl_general_v3.yaml}
cd ~/aow
. ~/venv/bin/activate
export PATH=/usr/local/cuda/bin:$PATH PYTHONPATH=src
OUT=results/bench_seeds
mkdir -p $OUT graphs
MAX=$(echo $COUNTS $MPS_COUNTS | tr ' ' '\n' | sort -n | tail -1)
{
  nvidia-smi --query-gpu=name,clocks.max.sm,memory.total --format=csv,noheader
  for k in $(seq 1 $MAX); do
    G=$(pwd)/graphs/seedbench_s$k
    python -m aow_sim.gpu.export_graph --config $CONFIG --nworld $NW --out $G \
        --seed $k --curriculum-lead $(awk "BEGIN{print 0.1 * $NW / 4096}") > $OUT/export_s$k.log 2>&1 \
        || { echo "export s$k failed"; tail -5 $OUT/export_s$k.log; }
    rm -rf ~/puffer_s$k && cp -r ~/puffer ~/puffer_s$k
    sed -i -e "s#^graph = .*#graph = $G#" -e "s/^seed = .*/seed = $((42 + k))/" \
        ~/puffer_s$k/config/aow_bike.ini
    rm -rf ~/puffer_s$k/checkpoints
    echo "s$k: $(grep -E '^(graph|seed|total_agents) ' ~/puffer_s$k/config/aow_bike.ini | tr '\n' ' ')"
  done
  bench() {  # bench <tag> <n>
    mkdir -p $OUT/$1
    for k in $(seq 1 $2); do
      ( cd ~/puffer_s$k && timeout $TRAIN_S ./puffer train --headless > ~/aow/$OUT/$1/s$k.log 2>&1 ) &
    done
    sleep $((TRAIN_S / 2))
    echo "$1 mid-run: $(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader)"
    wait
  }
  for n in $COUNTS; do bench n$n $n; done
  if [ -n "$MPS_COUNTS" ]; then
    export CUDA_MPS_PIPE_DIRECTORY=/tmp/mps_pipe CUDA_MPS_LOG_DIRECTORY=/tmp/mps_log
    mkdir -p $CUDA_MPS_PIPE_DIRECTORY $CUDA_MPS_LOG_DIRECTORY
    if nvidia-cuda-mps-control -d; then
      sleep 2
      for n in $MPS_COUNTS; do bench mps$n $n; done
      echo quit | nvidia-cuda-mps-control
    else
      echo "MPS daemon did not start (rc $?): no mps rows"
    fi
    cat $CUDA_MPS_LOG_DIRECTORY/control.log 2>/dev/null | tail -5
  fi
} > $OUT/setup.txt 2>&1
TAGS="$(for n in $COUNTS; do echo -n "n$n "; done)$(for n in $MPS_COUNTS; do echo -n "mps$n "; done)"
python3 - "$OUT" $TAGS > $OUT/summary.txt 2>&1 <<'EOF'
import re, sys, glob
out, tags = sys.argv[1], sys.argv[2:]
def num(p, key):
    m = re.search(re.escape(key) + r"\s+(-?[\d.]+)([KM]?)", p)
    return float(m.group(1)) * {"": 1, "K": 1e3, "M": 1e6}[m.group(2)] if m else None
def up(p):
    m = re.search(r"Uptime\s+((?:\d+h\s*)?(?:\d+m\s*)?(?:\d+s\s*)?(?:\d+ms)?)", p)
    return sum(float(v) * {"h": 3600, "m": 60, "s": 1, "ms": 1e-3}[u]
               for v, u in re.findall(r"(\d+)(h|ms|m|s)", m.group(1))) if m else None
print("arm    per-run steps/s (from 45 s on)            total     per-run mean")
for tag in tags:
    rates = []
    for f in sorted(glob.glob(f"{out}/{tag}/s*.log")):
        rows = [(up(p), num(p, "Steps")) for p in open(f, errors="replace").read().split("PufferLib 5.0")[1:]]
        rows = [r for r in rows if r[0] is not None and r[1] is not None and r[0] >= 45]
        rates.append((rows[-1][1] - rows[0][1]) / (rows[-1][0] - rows[0][0]) if len(rows) > 1 else float("nan"))
    if not rates:
        print(f"{tag:<6} (no logs)"); continue
    tot = sum(rates)
    print(f"{tag:<6} {' '.join(f'{r/1e3:5.1f}k' for r in rates):<40} {tot/1e3:6.1f}k  {tot/len(rates)/1e3:6.1f}k")
EOF
cat $OUT/setup.txt $OUT/summary.txt
echo ALL > $OUT/ALL_DONE
