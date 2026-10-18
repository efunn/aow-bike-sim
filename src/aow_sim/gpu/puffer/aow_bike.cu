// PufferLib 5.0 GPU env for the AoW bike (docs/plans/gpu-training.md, phase 3).
//
// The env is a saved Warp graph: `python -m aow_sim.gpu.export_graph` builds
// GpuGeneralEnv for N worlds, resets it, captures ONE policy step (action ->
// 50 MuJoCo Warp substeps with the drivetrain / gearbox hooks -> AHRS and
// odometry -> obs, reward, flags -> auto-reset) and saves it with Warp's APIC
// (`.wrp` + `_modules/`). This file loads it through libwarp's C API -- no
// Python in the trainer, at startup or per step -- and each step:
//
//   ab_in kernel: PufferLib's actions, CLIPPED to [-1, 1] -> the graph's act_in
//   cudaGraphLaunch(the Warp graph's exec)              (one launch, 50 substeps)
//   ab_out kernel: obs -> obs_t, reward, terminal = fall OR time limit,
//                  finished episodes -> this world's Log
//
// all on the stream PufferLib binds. The state lives in the loaded graph's
// buffers. Run with base.cudagraphs = -1: PufferLib's per-horizon capture
// would record our cudaGraphLaunch inside its own capture, which phase 1
// measured as unsupported (T5a/T5b); eager rollout launches the two
// back to back instead.
//
// [env] keys (config/aow_bike.ini): graph = stem of the exported graph,
// warp_lib = path to Warp's warp.so (python -c "import warp,os;
// print(os.path.dirname(warp.__file__))")/bin/warp.so, warp_version.
//
// THE CLIP IS SB3's CONTRACT, not a safety net. PufferLib hands over the raw
// Gaussian sample (mean + std * noise, clamped only at +-1e6). SB3 clips to the
// action box before env.step, and GeneralEnv relies on that: the physics clips
// again, but the effort and smoothness penalties and obs[12:15] (the action
// just applied) read the action as given. Unclipped, a wandering mean made
// rewards of -1e8 and inputs of the same size, and the 2026-10-15 1 h run went
// to NaN at ~33M steps (return -91M, kl 1.1e6). PPO's log-prob still uses the
// unclipped sample, as SB3's does. A NaN action clips to -1 (fmaxf).
//
// Time limits are TERMINALS here: PufferLib 5.0 has no truncation signal, so
// a timed-out episode is not bootstrapped (SB3, the CPU stack, does). Phase 4.
#ifndef PUFFER_AOW_BIKE_GPU_CU
#define PUFFER_AOW_BIKE_GPU_CU

#define PUF_BACKEND PUF_GPU

// Three changes to PufferLib's trainer, made by puffer_aow.patch (install.sh
// applies it) and switched on here, so they reach this env's build only:
//
// AOW_SMOOTH_TEMPORAL: SmoothPPO's temporal term on the policy mean
// (smooth_temporal.h), in PPO's loss. The CPU stack's weight (config/rl_general_v3.yaml, algo.smooth.temporal),
// CHOSEN, not swept: one gradient-ratio point on SB3's trained MLP actor put
// the term at ~1/4 of PPO's policy gradient (policy-smoothness-losses.md §4).
// Not re-measured on this net. Remove the line to train without it.
#define AOW_SMOOTH_TEMPORAL 0.04f
// AOW_NORM_ADV: advantages normalised per minibatch, as SB3 does. PufferLib
// uses them raw, so ent_coef and the temporal weight above would mean
// whatever the reward's scale made them.
#define AOW_NORM_ADV
// AOW_GRAD_LOG: one "AOW_GRAD pg .. smooth .. ratio .." line per epoch in
// train.log -- the temporal term's gradient against PPO's, at the policy
// mean (watch.py shows the ratio).
#define AOW_GRAD_LOG
// The reward PufferLib TRAINS on, scaled: it clamps every training reward to
// [-1, 1] (pufferl.cu:1487 @ ea7f989). Unscaled, a step reaches 3.0 (w_vel
// 1.5 + w_head 1.0 + w_alive 0.5), so once the heading error is under ~21 deg
// heading + alive alone hit the clamp and VELOCITY TRACKING EARNED NOTHING;
// a fall's -50 was -1. Every PufferLib run to 2026-10-15 trained that way --
// and the next run's checkpoints tracked heading and ignored speed. x 0.02
// puts a fall at ~-1 and keeps every ratio. The dashboard's episode_return
// is the env's own (last_ret), unscaled.
#define AOW_REWARD_SCALE 0.02f

#include <cuda.h>
#include <cuda_bf16.h>
#include <cuda_runtime.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// bf16 trainer builds store obs in precision_t, float builds in float (as
// robot_arm does; build.sh greps this file for the typedef).
#if defined(from_float) && !defined(PRECISION_FLOAT)
typedef precision_t obs_t;
#define AB_OBS(x) __float2bfloat16(x)
#else
typedef float obs_t;
#define AB_OBS(x) (x)
#endif
#include "pufferenv.h"

// general_spec's v3 layout (20 with the style bit); ACT: steer rate, hub,
// diff, continuous. install.sh rewrites OBS_SIZE from the graph's json
// obs_dim: ab_param below only checks "at least", so a 20-wide graph under
// a 17 here would load and silently never show the policy the bit.
#define OBS_SIZE 17
#define NUM_ATNS 3
#define ACT_SIZES {1, 1, 1}

// PufferLib divides every field by n (episodes) before puf_log.
struct Log {
    float score;            // = track: GeneralEnv's per-episode tracking score
    float episode_return;
    float episode_length;   // policy steps (50 Hz)
    float fall_rate;
    float track;
    float difficulty;       // the world's curriculum difficulty at episode end
    float n;
};

typedef struct Env {
    Log log;
    Agent agents[1];
    int num_agents;
    int tag;
    int boundary_reached;
    unsigned int rng;
} Env;

// libwarp's APIC C API (warp/native/apic.h), resolved at runtime.
typedef int (*ab_wp_init_t)(const char*);
typedef void* (*ab_load_t)(void*, const char*, int);
typedef void* (*ab_ptr_t)(void*, const char*);
typedef size_t (*ab_size_t)(void*, const char*);
typedef void* (*ab_graph_t)(void*);
typedef const char* (*ab_err_t)();
typedef CUresult (*ab_ctx_t)(CUcontext*);

static struct {
    Env* envs;
    int n;
    obs_t* observations;
    float* actions;
    float* rewards;
    float* terminals;
    cudaStream_t stream;
    void* graph;                 // the loaded APIC graph
    cudaGraphExec_t exec;
    float* act_in;               // (n, 3)
    float* obs;                  // (n, OBS_SIZE)
    float* reward;
    int* terminated;
    int* truncated;
    int* ep_end;
    float* last_ret;
    int* last_len;
    float* last_track;
    int* last_fell;
    float* last_diff;
} g_ab;

#define AB_CK(x) do { cudaError_t e_ = (x); if (e_ != cudaSuccess) { \
    fprintf(stderr, "aow_bike: %s failed: %s (%s:%d)\n", #x, cudaGetErrorString(e_), \
            __FILE__, __LINE__); exit(1); } } while (0)

static const char* ab_str(Dict* kw, const char* key, const char* fallback) {
    DictItem* it = dict_find(kw, key);
    if (it != NULL && it->str != NULL && it->str[0] != '\0' && strcmp(it->str, "None") != 0) {
        return it->str;
    }
    if (fallback == NULL) {
        fprintf(stderr, "aow_bike: [env] %s is required\n", key);
        exit(1);
    }
    return fallback;
}

// A bound buffer by name, checked against the size this env expects.
static void* ab_param(ab_ptr_t ptr, ab_size_t size, const char* name, size_t want) {
    void* p = ptr(g_ab.graph, name);
    size_t have = size(g_ab.graph, name);
    if (p == NULL || have < want) {
        fprintf(stderr, "aow_bike: graph buffer '%s' is %zu B, need %zu: graph exported for "
                "a different vec.total_agents?\n", name, have, want);
        exit(1);
    }
    return p;
}

__global__ void ab_out(int n, const float* obs, const float* reward, const int* terminated,
        const int* truncated, const int* ep_end, const float* last_ret, const int* last_len,
        const float* last_track, const int* last_fell, const float* last_diff,
        obs_t* observations, float* rewards, float* terminals, Env* envs) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) {
        return;
    }
    for (int j = 0; j < OBS_SIZE; j++) {
        observations[i * OBS_SIZE + j] = AB_OBS(obs[i * OBS_SIZE + j]);
    }
    rewards[i] = AOW_REWARD_SCALE * reward[i];
    terminals[i] = (terminated[i] | truncated[i]) ? 1.0f : 0.0f;
    if (ep_end[i]) {
        Log* log = &envs[i].log;
        log->score += last_track[i];
        log->episode_return += last_ret[i];
        log->episode_length += (float)last_len[i];
        log->fall_rate += (float)last_fell[i];
        log->track += last_track[i];
        log->difficulty += last_diff[i];
        log->n += 1.0f;
    }
}

__global__ void ab_in(int n, const float* actions, float* act_in) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n * NUM_ATNS) {
        return;
    }
    act_in[i] = fminf(fmaxf(actions[i], -1.0f), 1.0f);
}

// The reset observation: the graph was saved right after a reset, so its obs
// buffer already holds every world's first observation.
__global__ void ab_first_obs(int n, const float* obs, obs_t* observations, float* rewards,
        float* terminals) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) {
        return;
    }
    for (int j = 0; j < OBS_SIZE; j++) {
        observations[i * OBS_SIZE + j] = AB_OBS(obs[i * OBS_SIZE + j]);
    }
    rewards[i] = 0.0f;
    terminals[i] = 0.0f;
}

Env* puf_vec_create(int n, Dict* env_kwargs,
        obs_t* observations, float* actions, float* rewards, float* terminals) {
    const char* stem = ab_str(env_kwargs, "graph", NULL);
    const char* lib = ab_str(env_kwargs, "warp_lib", NULL);
    const char* ver = ab_str(env_kwargs, "warp_version", "1.18.0");

    AB_CK(cudaFree(0));                          // the runtime's primary context, current
    void* hcuda = dlopen("libcuda.so.1", RTLD_NOW | RTLD_GLOBAL);
    ab_ctx_t ctx_get = hcuda ? (ab_ctx_t)dlsym(hcuda, "cuCtxGetCurrent") : NULL;
    CUcontext ctx = NULL;
    if (ctx_get == NULL || ctx_get(&ctx) != CUDA_SUCCESS || ctx == NULL) {
        fprintf(stderr, "aow_bike: no current CUDA context\n");
        exit(1);
    }
    void* h = dlopen(lib, RTLD_NOW | RTLD_GLOBAL);
    if (h == NULL) {
        fprintf(stderr, "aow_bike: dlopen %s: %s\n", lib, dlerror());
        exit(1);
    }
    ab_wp_init_t wp_init = (ab_wp_init_t)dlsym(h, "wp_init");
    ab_load_t load = (ab_load_t)dlsym(h, "wp_apic_load_graph");
    ab_ptr_t ptr = (ab_ptr_t)dlsym(h, "wp_apic_get_param_ptr");
    ab_size_t size = (ab_size_t)dlsym(h, "wp_apic_get_param_size");
    ab_graph_t get_exec = (ab_graph_t)dlsym(h, "wp_apic_get_cuda_graph_exec");
    ab_err_t errstr = (ab_err_t)dlsym(h, "wp_get_error_string");
    if (!wp_init || !load || !ptr || !size || !get_exec) {
        fprintf(stderr, "aow_bike: %s lacks the APIC C API\n", lib);
        exit(1);
    }
    wp_init(ver);
    char wrp[1024];
    snprintf(wrp, sizeof wrp, "%s.wrp", stem);
    g_ab.graph = load(ctx, wrp, 0);              // 0 = CUDA
    if (g_ab.graph == NULL) {
        fprintf(stderr, "aow_bike: loading %s failed: %s\n", wrp, errstr ? errstr() : "?");
        exit(1);
    }
    g_ab.exec = (cudaGraphExec_t)get_exec(g_ab.graph);
    size_t f = sizeof(float);
    g_ab.act_in = (float*)ab_param(ptr, size, "act_in", (size_t)n * NUM_ATNS * f);
    g_ab.obs = (float*)ab_param(ptr, size, "obs", (size_t)n * OBS_SIZE * f);
    g_ab.reward = (float*)ab_param(ptr, size, "reward", n * f);
    g_ab.terminated = (int*)ab_param(ptr, size, "terminated", n * f);
    g_ab.truncated = (int*)ab_param(ptr, size, "truncated", n * f);
    g_ab.ep_end = (int*)ab_param(ptr, size, "ep_end", n * f);
    g_ab.last_ret = (float*)ab_param(ptr, size, "last_ret", n * f);
    g_ab.last_len = (int*)ab_param(ptr, size, "last_len", n * f);
    g_ab.last_track = (float*)ab_param(ptr, size, "last_track", n * f);
    g_ab.last_fell = (int*)ab_param(ptr, size, "last_fell", n * f);
    g_ab.last_diff = (float*)ab_param(ptr, size, "last_diff", n * f);
    // an exact size match too: a graph for MORE worlds would load and run,
    // silently stepping worlds PufferLib never sees
    if (size(g_ab.graph, "obs") != (size_t)n * OBS_SIZE * f) {
        fprintf(stderr, "aow_bike: graph obs is %zu wide, OBS_SIZE is %d (install.sh "
                "sets it from the graph's json)\n", size(g_ab.graph, "obs") / (n * f), OBS_SIZE);
        exit(1);
    }
    if (size(g_ab.graph, "reward") != n * f) {
        fprintf(stderr, "aow_bike: graph has %zu worlds, vec.total_agents is %d\n",
                size(g_ab.graph, "reward") / f, n);
        exit(1);
    }

    Env* host = (Env*)calloc(n, sizeof(Env));
    for (int i = 0; i < n; i++) {
        host[i].num_agents = 1;
        host[i].rng = i + 1;
    }
    AB_CK(cudaMalloc((void**)&g_ab.envs, n * sizeof(Env)));
    AB_CK(cudaMemcpy(g_ab.envs, host, n * sizeof(Env), cudaMemcpyHostToDevice));
    free(host);
    g_ab.n = n;
    g_ab.observations = observations;
    g_ab.actions = actions;
    g_ab.rewards = rewards;
    g_ab.terminals = terminals;
    g_ab.stream = 0;
    fprintf(stderr, "aow_bike: loaded %s, %d worlds\n", wrp, n);
    return g_ab.envs;
}

void puf_bind_stream(cudaStream_t stream) {
    g_ab.stream = stream;
}

void puf_init(Env* env, Dict*) {
    env->num_agents = 1;
}

// Called once at the start of training. Worlds are already at fresh episodes
// (exported after a reset); later resets happen inside the graph.
void puf_reset(Env*) {
    int b = 256;
    ab_first_obs<<<(g_ab.n + b - 1) / b, b, 0, g_ab.stream>>>(
        g_ab.n, g_ab.obs, g_ab.observations, g_ab.rewards, g_ab.terminals);
    AB_CK(cudaGetLastError());
}

void puf_step(Env*) {
    int b = 256;
    ab_in<<<(g_ab.n * NUM_ATNS + b - 1) / b, b, 0, g_ab.stream>>>(
        g_ab.n, g_ab.actions, g_ab.act_in);
    AB_CK(cudaGraphLaunch(g_ab.exec, g_ab.stream));
    ab_out<<<(g_ab.n + b - 1) / b, b, 0, g_ab.stream>>>(
        g_ab.n, g_ab.obs, g_ab.reward, g_ab.terminated, g_ab.truncated, g_ab.ep_end,
        g_ab.last_ret, g_ab.last_len, g_ab.last_track, g_ab.last_fell, g_ab.last_diff,
        g_ab.observations, g_ab.rewards, g_ab.terminals, g_ab.envs);
    AB_CK(cudaGetLastError());
}

void puf_close(Env*) {
    if (g_ab.envs) {
        cudaFree(g_ab.envs);
        g_ab.envs = NULL;
    }
}

void puf_render(Env*) {
}

void puf_log(Log* log, Dict* out) {
    dict_set(out, "score", log->score);
    dict_set(out, "perf", log->score);
    dict_set(out, "episode_return", log->episode_return);
    dict_set(out, "episode_length", log->episode_length);
    dict_set(out, "fall_rate", log->fall_rate);
    dict_set(out, "track", log->track);
    dict_set(out, "difficulty", log->difficulty);
}

#endif
