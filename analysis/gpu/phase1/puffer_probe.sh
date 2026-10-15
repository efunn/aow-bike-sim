#!/bin/bash
# Bridge probes 1 and 5 (docs/plans/gpu-training.md): does PufferLib 5.0 build
# on this CUDA/driver, what does its own CUDA env (robot_arm) train at end to
# end, and does `policy.num_layers = 0` (MLP-only, no MinGRU) run without
# editing algo.cu. Public repo at the commit the plan read; no token.
export PATH=/usr/local/cuda/bin:$PATH   # runpod/pytorch: toolkit present, not on PATH
set -u
OUT=${OUT:-$PWD/results}
mkdir -p "$OUT"
cd ~
t0=$(date +%s)
[ -d puffer ] || git clone -q https://github.com/PufferAI/PufferLib.git puffer
echo "{\"step\":\"puffer_clone\",\"wall_s\":$(( $(date +%s) - t0 ))}" | tee -a "$OUT/timings.jsonl"
cd puffer && git checkout -q ea7f989 && git log --oneline -1 | tee "$OUT/puffer_commit.txt"

# libomp: build.sh links -lomp5 on Linux
if ! ldconfig -p | grep -q 'libomp5\.so '; then
    so=$(ldconfig -p | awk '/libomp\.so[ .]/{print $NF; exit}')
    [ -n "$so" ] && ln -sf "$so" /usr/lib/x86_64-linux-gnu/libomp5.so
fi

# At ea7f989 build.sh greps robot_arm.cu for `typedef ... obs_t`, but the
# typedef is in robot_arm.h (found on the pod, 2026-10-14): the build refuses
# before compiling. Let the check see the .h too; nothing else changes.
sed -i 's|grep -q '"'"'typedef\[\[:space:\]\].\*obs_t'"'"' "\$ENV_HEADER"|cat "$ENV_HEADER" "${ENV_HEADER%.cu}.h" 2>/dev/null \| grep -q '"'"'typedef[[:space:]].*obs_t'"'"'|' build.sh
grep -n 'obs_t' build.sh | head -3

t0=$(date +%s)
./build.sh robot_arm > "$OUT/puffer_build.log" 2>&1
echo "{\"probe\":\"puffer_build\",\"ok\":$([ -x ./puffer ] && echo true || echo false),\"wall_s\":$(( $(date +%s) - t0 ))}" | tee -a "$OUT/results.jsonl" "$OUT/timings.jsonl"
[ -x ./puffer ] || { tail -30 "$OUT/puffer_build.log"; exit 1; }

# 5 min of its default robot_arm training, then 2 min with no MinGRU layers
timeout 300 ./puffer train --headless > "$OUT/puffer_robot_arm.log" 2>&1
echo "{\"probe\":\"puffer_robot_arm\",\"exit\":$?}" | tee -a "$OUT/results.jsonl"
timeout 120 ./puffer train --headless --policy.num_layers=0 > "$OUT/puffer_layers0.log" 2>&1
echo "{\"probe\":\"puffer_layers0\",\"exit\":$?}" | tee -a "$OUT/results.jsonl"
cp -r logs "$OUT/puffer_logs" 2>/dev/null
tail -5 "$OUT/puffer_robot_arm.log" "$OUT/puffer_layers0.log"
