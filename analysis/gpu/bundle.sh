#!/bin/bash
# The tarball a GPU pod needs, from the repo root here:  bash analysis/gpu/bundle.sh OUT.tgz
# Code, configs, v3_s0 (the checks fly it), general_puffer_fixes_52M
# (bench_env's MinGRU driver) and the GPU scripts; never results,
# caches or anything git-ignored. ~1.1 MB. On the pod: mkdir ~/aow && tar xzf
# into it. Copy files, never a git remote: no token goes on a rented machine.
set -e
OUT=${1:?usage: bash analysis/gpu/bundle.sh OUT.tgz}
COPYFILE_DISABLE=1 tar czf "$OUT" --no-xattrs \
    --exclude __pycache__ --exclude '*.pyc' --exclude 'results*' \
    --exclude 'v3_rigid.mjb' --exclude 'v3_init_state.npz' \
    src config pyproject.toml \
    moves/general_rl_v3_s0.npz moves/general_rl_v3_s0.yaml \
    moves/general_puffer_fixes_52M.npz moves/general_puffer_fixes_52M.yaml \
    analysis/gpu
ls -la "$OUT" | awk '{print $5, $NF}'
