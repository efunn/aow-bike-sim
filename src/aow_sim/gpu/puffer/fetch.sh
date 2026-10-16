#!/bin/bash
# PufferLib at the pinned commit into <dir>, ONE commit not the history: a
# full clone is 928 MB and took 46.7 s on a 5090 pod (2026-10-16), this
# 8.0 s. A no-op if <dir> is already a checkout. install.sh calls it;
# pod_setup.sh calls it early, in the background, to overlap pip.
#   bash src/aow_sim/gpu/puffer/fetch.sh <dir>
set -euo pipefail
PUF=$1
PUF_SHA=ea7f989371270d2948f96728bda8aec5e7634a4c
if [ ! -d "$PUF/.git" ]; then
    mkdir -p "$PUF" && git -C "$PUF" init -q
    git -C "$PUF" remote add origin https://github.com/PufferAI/PufferLib.git
    git -C "$PUF" fetch -q --depth 1 origin "$PUF_SHA"
fi
git -C "$PUF" checkout -q "$PUF_SHA"
