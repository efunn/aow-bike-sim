#!/bin/bash
# All of phase 1 in order, on the pod, after pod_setup.sh. Every probe
# appends JSON lines to results/results.jsonl; logs beside it. Copy results/
# back before terminating the pod.
export PATH=/usr/local/cuda/bin:$PATH   # runpod/pytorch: toolkit present, not on PATH
set -u
. ~/venv/bin/activate
mkdir -p results
R=results/results.jsonl
T=results/timings.jsonl
run() {   # every probe's wall time too (user: what would go in a Docker image)
    echo "== $*" >&2
    local t0=$(date +%s.%N)
    "$@" 2>> results/stderr.log | grep '^{' | tee -a "$R"
    echo "{\"step\":\"$*\",\"wall_s\":$(awk "BEGIN{print $(date +%s.%N) - $t0}")}" >> "$T"
}

nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | sed 's/^/# /' | tee -a "$R"

# physics (gate A); compile first, from the fresh pod's empty kernel cache
run python probe_physics.py compile --nworld 1024
run python probe_physics.py track --steps 1250 --every 250
run python probe_physics.py perworld --steps 250
run python probe_physics.py sweep --nworld 1024 4096 16384 32768 --steps 1000 --chunk 50
run python probe_physics.py sweep_mjx --nworld 1024 4096 --steps 200 --chunk 50

# bridge: graph out of Python, into C
WV=$(python -c "import warp; print(warp.__version__)")
LIB=$(python -c "import warp, os; print(os.path.join(os.path.dirname(warp.__file__), 'bin', 'warp.so'))")   # libwarp.dylib on macOS
run nvcc -O2 -std=c++17 bridge_probe.cu -o bridge_probe -lcuda -ldl
for NW in 1024 4096; do      # the .wrp snapshots every buffer: ~0.7 MB per world
    run python save_graph.py --nworld $NW --steps 50 --out wrp/bike$NW
    run ./bridge_probe "$LIB" wrp/bike$NW "$WV" $NW 50 100
    rm -rf wrp/bike$NW*      # GBs; the numbers are in results/
done

# PufferLib
OUT=$PWD/results bash puffer_probe.sh
# what a Docker image could carry: Warp's compiled kernels, JAX's cache, pip's
du -sh ~/.cache/warp ~/.cache/jax ~/.cache/pip ~/.ccache 2>/dev/null | sed 's/^/# cache /' | tee -a "$T"
# Warp's compiled kernels, for the Dockerfile's warp-cache/ (copy results/ back)
tar czf results/warp-cache.tgz -C ~/.cache/warp . 2>/dev/null && ls -la results/warp-cache.tgz >&2
echo "done; results in results/" >&2
