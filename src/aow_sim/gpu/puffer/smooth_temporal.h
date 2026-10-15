// CAPS's temporal term on the policy MEAN, for PufferLib's fused PPO loss --
// SmoothPPO's `temporal` (src/aow_sim/smooth_ppo.py) on the GPU stack:
//
//     lam * sum_h (mu_{t+1,h} - mu_{t,h})^2      per pair inside one episode
//
// mu in units of each action's bound (what the network emits, before
// aow_bike.cu's clip), as SmoothPPO's. PufferLib's train minibatch is rows of
// `horizon` CONSECUTIVE steps of one agent, so a pair is (t, t+1) in one row;
// a pair is skipped when either step carries a terminal, which is right
// whichever step PufferLib stamps the terminal on (at the cost of one extra
// pair per episode end). Used by puffer_aow.patch inside ppo_loss_compute,
// one thread per (row, t): each thread owns the loss of its pair (t, t+1) and
// writes d/d mu_t of BOTH pairs it is in, so nothing is shared between
// threads. `scale` is the kernel's 1/(N*T), as every other term there.
//
// Plain C++ when not under nvcc, so analysis/gpu/phase3/check_smooth.py can
// check the gradient against finite differences without a GPU.
#pragma once

#ifndef __CUDACC__
#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif
#endif

__host__ __device__ inline float aow_smooth_temporal(
        const float* mu_prev, const float* mu, const float* mu_next,
        int has_prev, int has_next, int A, float lam, float scale, float* grad) {
    float loss = 0.0f;
    for (int h = 0; h < A; ++h) {
        float g = 0.0f;
        if (has_prev) {
            g += mu[h] - mu_prev[h];
        }
        if (has_next) {
            float d = mu_next[h] - mu[h];
            loss += d * d;
            g -= d;
        }
        grad[h] = 2.0f * lam * scale * g;
    }
    return lam * scale * loss;
}
