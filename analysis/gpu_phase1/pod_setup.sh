#!/bin/bash
# On the pod, from the directory holding this folder's files. Pins as in
# docs/plans/gpu-training.md: MuJoCo 3.10.0 (the .mjb is tied to it) and the
# MJWarp release built against it; Warp is whatever that release resolves
# (1.18.0 on the Mac, 2026-10-14). No repo, no tokens.
# Every step is timed into results/timings.jsonl: what a Docker image would
# save (user, 2026-10-14).
export PATH=/usr/local/cuda/bin:$PATH   # runpod/pytorch: toolkit present, not on PATH
set -e
mkdir -p results
step() {  # step <name> <cmd...>
    local name=$1; shift
    local t0=$(date +%s.%N)
    "$@"
    echo "{\"step\":\"$name\",\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $t0}")}" | tee -a results/timings.jsonl
}
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
nvcc --version | tail -2
# PufferLib's build: clang + OpenMP, ccache (build.sh calls it), git, libGL (raylib links -lGL)
step apt_update apt-get update -qq
step apt_install sh -c "apt-get install -y -qq clang libomp-dev ccache git libgl-dev > /dev/null"
step venv python -m venv --system-site-packages ~/venv
. ~/venv/bin/activate
step pip_mujoco_warp pip install -q "mujoco==3.10.0" "mujoco-warp==3.10.0.3"
step pip_mjx_jax pip install -q "mujoco-mjx==3.10.0" "jax[cuda12]"
step pip_nccl pip install -q nvidia-nccl-cu12
# FAIL here if CUDA is unusable: Warp otherwise falls back to the CPU without
# an error. 2026-10-14, a CA-MTL-1 A40: nvidia-smi listed the card, but cuInit
# gave error 999 (torch too), and the export ran on the CPU (2.8 GB graph)
# for 6 min before anyone noticed the GPU sat at 0 %.
step warp_init python -c "import warp as wp; wp.init(); print(wp.__version__, wp.get_device()); assert wp.is_cuda_available(), 'CUDA unusable on this pod (Warp fell back to the CPU) -- replace the pod'"
pip freeze | grep -iE "mujoco|warp|jax|nccl" > results/versions.txt
cat results/versions.txt
