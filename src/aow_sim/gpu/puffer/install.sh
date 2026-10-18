#!/bin/bash
# Put the AoW bike env into a PufferLib 5.0 checkout and build the trainer.
#
#   bash src/aow_sim/gpu/puffer/install.sh <puffer_dir> <graph_stem> [--float]
#
# Run inside the Python env that exported the graph (it asks Warp where its
# warp.so is, and which version it is). Fetches PufferLib (public, no token)
# at ea7f989 -- the commit docs/plans/gpu-training.md read and phase 1 built
# -- shallow, through fetch.sh, if <puffer_dir> is not a checkout yet, and
# patches its trainer (puffer_aow.patch).
# Copies aow_bike.cu to ocean/aow_bike/ and
# writes config/aow_bike.ini with the graph and Warp paths filled in. The
# graph's world count must equal vec.total_agents there (4096 as shipped).
# Then: cd <puffer_dir> && ./puffer train --headless
set -euo pipefail
PUF=$1
STEM=$(realpath "$2")
FLOAT=${3:-}
HERE=$(cd "$(dirname "$0")" && pwd)
export PATH=/usr/local/cuda/bin:$PATH        # runpod/pytorch: toolkit not on PATH

[ -f "$STEM.wrp" ] || { echo "no graph at $STEM.wrp (python -m aow_sim.gpu.export_graph)"; exit 1; }
WARP_LIB=$(python -c "import warp, os; print(os.path.join(os.path.dirname(warp.__file__), 'bin', 'warp.so'))")
WARP_VER=$(python -c "import warp; print(warp.__version__)")
[ -f "$WARP_LIB" ] || { echo "no $WARP_LIB"; exit 1; }
NW=$(python -c "import json; print(json.load(open('$STEM.json'))['nworld'])")
OBS=$(python -c "import json; print(json.load(open('$STEM.json'))['obs_dim'])")

bash "$HERE/fetch.sh" "$PUF"          # the pinned commit, shallow
cd "$PUF"
# the temporal smoothness term, advantage normalisation and the gradient log
# (each on only when aow_bike.cu defines its AOW_* switch). Reset first: a
# rerun must not patch twice.
git checkout -q -- src/algo.cu src/pufferl.cu
git apply "$HERE/puffer_aow.patch"
cp "$HERE/smooth_temporal.h" src/smooth_temporal.h
# libomp: build.sh links -lomp5 on Linux (phase 1)
if ! ldconfig -p | grep -q 'libomp5\.so '; then
    so=$(ldconfig -p | awk '/libomp\.so[ .]/{print $NF; exit}')
    [ -n "$so" ] && ln -sf "$so" /usr/lib/x86_64-linux-gnu/libomp5.so
fi
mkdir -p ocean/aow_bike
# the obs width is the graph's (17, or 20 with the style bit)
sed -e "s/^#define OBS_SIZE .*/#define OBS_SIZE $OBS/" "$HERE/aow_bike.cu" \
    > ocean/aow_bike/aow_bike.cu
grep -q "^#define OBS_SIZE $OBS\$" ocean/aow_bike/aow_bike.cu || { echo "OBS_SIZE not set"; exit 1; }
sed -e "s#GRAPH_STEM#$STEM#" -e "s#WARP_LIB#$WARP_LIB#" -e "s#WARP_VERSION#$WARP_VER#" \
    -e "s#^total_agents = .*#total_agents = $NW#" \
    "$HERE/aow_bike.ini" > config/aow_bike.ini
./build.sh aow_bike --cu $FLOAT
[ -x ./puffer ] && echo "built $PUF/puffer for aow_bike ($NW worlds, obs $OBS, graph $STEM)"
