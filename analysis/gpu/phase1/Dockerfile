# TENTATIVE -- never built (2026-10-14). What pod_setup.sh does by hand on
# every rental, done once: ~4 min a pod on the A40 run (apt 25 s, pip 38 s,
# Warp kernel compile 45 s, PufferLib compile 125 s).
#
# Where it would live: build on the box (x86_64, Docker 29; the Mac is arm64),
# push to a registry RunPod can pull from -- Docker Hub public is simplest
# (nothing here is secret); GHCR private needs a token on the box and a
# registry credential in RunPod. Then create pods with `image:` set to it.
#
# Open before building:
#  - warp-cache/: copy back ~/.cache/warp from a pod (run_all.sh tars it into
#    results/). Warp emits PTX for min(device arch, 75) by default [read,
#    warp/_src/context.py get_cuda_compile_arch], so a cache from the A40
#    (sm_86) should serve any GPU >= sm_75 -- untested. It is keyed by Warp
#    and mujoco_warp versions and the model: re-copy when any of them moves.
#    The driver still JITs that PTX to the exact GPU on first load; how much
#    of the 45 s that is, is not known.
#  - PufferLib is compiled with nvcc -arch=native by default, which needs a
#    GPU at build time; NVCC_ARCH=all-major builds for every major arch
#    instead (slower build, bigger binary). Untested.
#  - robot_arm is only the probe env; our own env replaces it in phase 3.
FROM runpod/pytorch:1.0.2-cu1281-torch280-ubuntu2404

ENV PATH=/opt/venv/bin:/usr/local/cuda/bin:$PATH \
    WARP_CACHE_PATH=/opt/warp-cache

# PufferLib's build: clang + OpenMP, ccache, git, libGL (raylib links -lGL)
RUN apt-get update -qq \
 && apt-get install -y -qq clang libomp-dev ccache git libgl-dev \
 && rm -rf /var/lib/apt/lists/*

# versions as measured on the A40 run (results_a40/versions.txt)
RUN python -m venv --system-site-packages /opt/venv \
 && pip install --no-cache-dir \
      mujoco==3.10.0 mujoco-warp==3.10.0.3 warp-lang==1.18.0 \
      mujoco-mjx==3.10.0 jax==0.11.2 jax-cuda12-plugin==0.11.2 \
      jax-cuda12-pjrt==0.11.2 nvidia-nccl-cu12==2.27.3

# PufferLib at the commit the plan read, with the two build fixes
# (puffer_probe.sh: obs_t lives in robot_arm.h; -lomp5 symlink)
RUN git clone -q https://github.com/PufferAI/PufferLib.git /opt/puffer \
 && cd /opt/puffer && git checkout -q ea7f989 \
 && sed -i 's|grep -q '"'"'typedef\[\[:space:\]\].\*obs_t'"'"' "\$ENV_HEADER"|cat "$ENV_HEADER" "${ENV_HEADER%.cu}.h" 2>/dev/null \| grep -q '"'"'typedef[[:space:]].*obs_t'"'"'|' build.sh \
 && so=$(ldconfig -p | awk '/libomp\.so[ .]/{print $NF; exit}') \
 && ln -sf "$so" /usr/lib/x86_64-linux-gnu/libomp5.so \
 && NVCC_ARCH=all-major ./build.sh robot_arm

COPY warp-cache/ /opt/warp-cache/
