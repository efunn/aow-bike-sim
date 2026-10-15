// Bridge probe, C half (docs/plans/gpu-training.md, phase 1). No Python.
//
// Loads the .wrp that save_graph.py wrote (50 MJWarp steps x N worlds, one
// Warp graph) through libwarp's exported APIC C API, then asks the bridge's
// questions in order, one JSON line each:
//
//   T1 load      wp_init + wp_apic_load_graph from C; the bound params
//   T2 replay    launch Warp's graph exec on a stream C created (foreign to
//                Warp); qpos == save_graph.py's reference?
//   T3 restore   write the saved state into the bound param pointers with
//                plain cudaMemcpy (what an env reset from C would do), launch
//                again; == reference?
//   T4 rate      R launches on the foreign stream, CUDA-event timed
//   T5 nest      PufferLib captures its rollout with cudaStreamBeginCapture.
//                Inside such a capture: D2D copy "actions" -> ctrl, the Warp
//                graph, D2D copy qpos -> "obs". Two ways in:
//                  a) cudaGraphLaunch(exec) while capturing
//                  b) cudaGraphAddChildGraphNode on the capture's graph
//                Instantiate the outer graph, launch it from the restored
//                state; == reference? And its launch rate.
//
// Build + run (on the pod; libwarp.so is in the warp wheel):
//   nvcc -O2 -std=c++17 bridge_probe.cu -o bridge_probe -lcuda -ldl
//   ./bridge_probe $(python -c "import warp,os;print(os.path.dirname(warp.__file__))")/bin/warp.so \
//       wrp/bike 1.18.0 4096 50 200
#include <cuda.h>
#include <cuda_runtime.h>
#include <dlfcn.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

typedef int (*wp_init_t)(const char*);
typedef void* (*load_t)(void*, const char*, int);
typedef void* (*ptr_t)(void*, const char*);
typedef size_t (*size_t_t)(void*, const char*);
typedef int (*num_t)(void*);
typedef const char* (*name_t)(void*, int);
typedef void* (*graph_t)(void*);
typedef const char* (*err_t)();

static const char* BOUND[] = {"qpos", "qvel", "act", "ctrl", "qacc_warmstart", "time", "history"};
static const int NBOUND = 7;

#define CK(x) do { cudaError_t e_ = (x); if (e_ != cudaSuccess) { \
    printf("{\"probe\":\"error\",\"at\":\"%s\",\"line\":%d,\"cuda\":\"%s\"}\n", #x, __LINE__, \
           cudaGetErrorString(e_)); exit(1); } } while (0)

static std::vector<float> read_bin(const char* path) {
    std::vector<float> v;
    FILE* f = fopen(path, "rb");
    if (!f) { printf("{\"probe\":\"error\",\"missing\":\"%s\"}\n", path); exit(1); }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    v.resize(n / sizeof(float));
    if (fread(v.data(), 1, n, f) != (size_t)n) exit(1);
    fclose(f);
    return v;
}

struct Ctx {
    ptr_t param_ptr; size_t_t param_size;
    void* g; const char* stem;
};

static double max_diff_qpos(Ctx& c, const std::vector<float>& ref) {
    std::vector<float> q(ref.size());
    CK(cudaDeviceSynchronize());
    CK(cudaMemcpy(q.data(), c.param_ptr(c.g, "qpos"), q.size() * 4, cudaMemcpyDeviceToHost));
    double m = 0;
    for (size_t i = 0; i < q.size(); i++) {
        double d = fabs((double)q[i] - ref[i]);
        if (!(d == d)) return INFINITY;      // NaN
        if (d > m) m = d;
    }
    return m;
}

static void restore(Ctx& c) {
    char p[1024];
    for (int i = 0; i < NBOUND; i++) {
        snprintf(p, sizeof p, "%s_saved_%s.bin", c.stem, BOUND[i]);
        std::vector<float> v = read_bin(p);
        size_t sz = c.param_size(c.g, BOUND[i]);
        // a bound name maps the array's whole base allocation; copy what we have
        CK(cudaMemcpy(c.param_ptr(c.g, BOUND[i]), v.data(), v.size() * 4 < sz ? v.size() * 4 : sz,
                      cudaMemcpyHostToDevice));
    }
    CK(cudaDeviceSynchronize());
}

static double time_launches(cudaGraphExec_t ex, cudaStream_t s, int reps) {
    cudaEvent_t a, b;
    CK(cudaEventCreate(&a)); CK(cudaEventCreate(&b));
    CK(cudaGraphLaunch(ex, s)); CK(cudaStreamSynchronize(s));     // warm
    CK(cudaEventRecord(a, s));
    for (int i = 0; i < reps; i++) CK(cudaGraphLaunch(ex, s));
    CK(cudaEventRecord(b, s)); CK(cudaEventSynchronize(b));
    float ms; CK(cudaEventElapsedTime(&ms, a, b));
    return ms / 1e3;
}

// Add `child` as a child-graph node at the current point of stream s's capture.
static cudaError_t add_child_in_capture(cudaStream_t s, cudaGraph_t child) {
    cudaStreamCaptureStatus st; unsigned long long id; cudaGraph_t cg;
    const cudaGraphNode_t* deps; size_t nd;
#if CUDART_VERSION >= 13000
    const cudaGraphEdgeData* ed;
    cudaError_t e = cudaStreamGetCaptureInfo(s, &st, &id, &cg, &deps, &ed, &nd);
#else
    cudaError_t e = cudaStreamGetCaptureInfo(s, &st, &id, &cg, &deps, &nd);
#endif
    if (e != cudaSuccess) return e;
    if (st != cudaStreamCaptureStatusActive) return cudaErrorStreamCaptureInvalidated;
    cudaGraphNode_t node;
    e = cudaGraphAddChildGraphNode(&node, cg, deps, nd, child);
    if (e != cudaSuccess) return e;
#if CUDART_VERSION >= 13000
    return cudaStreamUpdateCaptureDependencies(s, &node, nullptr, 1, cudaStreamSetCaptureDependencies);
#else
    return cudaStreamUpdateCaptureDependencies(s, &node, 1, cudaStreamSetCaptureDependencies);
#endif
}

int main(int argc, char** argv) {
    if (argc < 7) { fprintf(stderr, "usage: libwarp.so stem warp_version nworld steps reps\n"); return 2; }
    const char *lib = argv[1], *stem = argv[2], *ver = argv[3];
    int nworld = atoi(argv[4]), steps = atoi(argv[5]), reps = atoi(argv[6]);

    CK(cudaSetDevice(0)); CK(cudaFree(0));
    CUcontext cuctx; cuCtxGetCurrent(&cuctx);
    cudaDeviceProp prop; CK(cudaGetDeviceProperties(&prop, 0));
    int rt, drv; cudaRuntimeGetVersion(&rt); cudaDriverGetVersion(&drv);

    void* h = dlopen(lib, RTLD_NOW | RTLD_GLOBAL);
    if (!h) { printf("{\"probe\":\"T1\",\"ok\":false,\"dlopen\":\"%s\"}\n", dlerror()); return 1; }
    wp_init_t wp_init = (wp_init_t)dlsym(h, "wp_init");
    load_t load = (load_t)dlsym(h, "wp_apic_load_graph");
    Ctx c; c.stem = stem;
    c.param_ptr = (ptr_t)dlsym(h, "wp_apic_get_param_ptr");
    c.param_size = (size_t_t)dlsym(h, "wp_apic_get_param_size");
    num_t nparams = (num_t)dlsym(h, "wp_apic_get_num_params");
    name_t pname = (name_t)dlsym(h, "wp_apic_get_param_name");
    graph_t get_graph = (graph_t)dlsym(h, "wp_apic_get_cuda_graph");
    graph_t get_exec = (graph_t)dlsym(h, "wp_apic_get_cuda_graph_exec");
    err_t errstr = (err_t)dlsym(h, "wp_get_error_string");

    int ie = wp_init(ver);
    char wrp[1024]; snprintf(wrp, sizeof wrp, "%s.wrp", stem);
    c.g = load(cuctx, wrp, 0);
    printf("{\"probe\":\"T1\",\"ok\":%s,\"gpu\":\"%s\",\"cudart\":%d,\"driver\":%d,\"wp_init\":%d,"
           "\"err\":\"%s\",\"params\":[", c.g ? "true" : "false", prop.name, rt, drv, ie,
           (!c.g && errstr) ? errstr() : "");
    if (!c.g) { printf("]}\n"); return 1; }
    for (int i = 0; i < nparams(c.g); i++)
        printf("%s[\"%s\",%zu]", i ? "," : "", pname(c.g, i), c.param_size(c.g, pname(c.g, i)));
    printf("]}\n"); fflush(stdout);

    char p[1024]; snprintf(p, sizeof p, "%s_ref_qpos.bin", stem);
    std::vector<float> ref = read_bin(p);
    cudaGraph_t wg = (cudaGraph_t)get_graph(c.g);
    {   // what the Warp graph is made of: child graphs refuse some node types
        size_t n = 0; cudaGraphGetNodes(wg, nullptr, &n);
        std::vector<cudaGraphNode_t> nodes(n); cudaGraphGetNodes(wg, nodes.data(), &n);
        int counts[32] = {0};
        for (size_t i = 0; i < n; i++) { cudaGraphNodeType t; cudaGraphNodeGetType(nodes[i], &t); if ((int)t < 32) counts[(int)t]++; }
        printf("{\"probe\":\"node_types\",\"total\":%zu,\"by_type\":{", n);
        for (int t = 0, k = 0; t < 32; t++) if (counts[t]) printf("%s\"%d\":%d", k++ ? "," : "", t, counts[t]);
        printf("}}\n"); fflush(stdout);
    }
    cudaGraphExec_t wx = (cudaGraphExec_t)get_exec(c.g);
    cudaStream_t s; CK(cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking));   // C's own stream

    // T2: the graph as loaded holds the saved state
    CK(cudaGraphLaunch(wx, s)); CK(cudaStreamSynchronize(s));
    printf("{\"probe\":\"T2\",\"qpos_maxdiff_vs_ref\":%g}\n", max_diff_qpos(c, ref)); fflush(stdout);

    // T3
    restore(c);
    CK(cudaGraphLaunch(wx, s)); CK(cudaStreamSynchronize(s));
    printf("{\"probe\":\"T3\",\"qpos_maxdiff_vs_ref\":%g}\n", max_diff_qpos(c, ref)); fflush(stdout);

    // T4
    double sec = time_launches(wx, s, reps);
    printf("{\"probe\":\"T4\",\"nworld\":%d,\"steps_per_launch\":%d,\"launches\":%d,\"wall_s\":%.4f,"
           "\"physics_steps_per_s\":%.0f}\n", nworld, steps, reps, sec,
           (double)nworld * steps * reps / sec); fflush(stdout);

    // T5: buffers standing in for PufferLib's (C-allocated, not Warp's)
    size_t ctrl_b = c.param_size(c.g, "ctrl"), qpos_b = c.param_size(c.g, "qpos");
    void *actions, *obs;
    CK(cudaMalloc(&actions, ctrl_b)); CK(cudaMalloc(&obs, qpos_b));
    snprintf(p, sizeof p, "%s_saved_ctrl.bin", stem);
    std::vector<float> sc = read_bin(p);
    CK(cudaMemcpy(actions, sc.data(), sc.size() * 4 < ctrl_b ? sc.size() * 4 : ctrl_b,
                  cudaMemcpyHostToDevice));

    for (int way = 0; way < 2; way++) {
        const char* label = way == 0 ? "T5a_graphlaunch_in_capture" : "T5b_child_node_in_capture";
        cudaStream_t cs; CK(cudaStreamCreateWithFlags(&cs, cudaStreamNonBlocking));
        CK(cudaStreamBeginCapture(cs, cudaStreamCaptureModeThreadLocal));
        cudaMemcpyAsync(c.param_ptr(c.g, "ctrl"), actions, ctrl_b, cudaMemcpyDeviceToDevice, cs);
        cudaError_t e = way == 0 ? cudaGraphLaunch(wx, cs) : add_child_in_capture(cs, wg);
        cudaMemcpyAsync(obs, c.param_ptr(c.g, "qpos"), qpos_b, cudaMemcpyDeviceToDevice, cs);
        cudaGraph_t outer = nullptr;
        cudaError_t ee = cudaStreamEndCapture(cs, &outer);
        cudaGetLastError();
        if (e != cudaSuccess || ee != cudaSuccess || !outer) {
            printf("{\"probe\":\"%s\",\"ok\":false,\"add\":\"%s\",\"end_capture\":\"%s\"}\n", label,
                   cudaGetErrorString(e), cudaGetErrorString(ee));
            fflush(stdout);
            continue;
        }
        cudaGraphExec_t ox;
        cudaError_t ie2 = cudaGraphInstantiate(&ox, outer, 0);
        if (ie2 != cudaSuccess) {
            printf("{\"probe\":\"%s\",\"ok\":false,\"instantiate\":\"%s\"}\n", label, cudaGetErrorString(ie2));
            cudaGetLastError(); fflush(stdout);
            continue;
        }
        restore(c);
        CK(cudaGraphLaunch(ox, s)); CK(cudaStreamSynchronize(s));
        double dq = max_diff_qpos(c, ref);
        std::vector<float> o(qpos_b / 4), q(qpos_b / 4);
        CK(cudaMemcpy(o.data(), obs, qpos_b, cudaMemcpyDeviceToHost));
        CK(cudaMemcpy(q.data(), c.param_ptr(c.g, "qpos"), qpos_b, cudaMemcpyDeviceToHost));
        double obs_eq = memcmp(o.data(), q.data(), qpos_b) == 0;
        double osec = time_launches(ox, s, reps);
        printf("{\"probe\":\"%s\",\"ok\":true,\"qpos_maxdiff_vs_ref\":%g,\"obs_copy_exact\":%s,"
               "\"physics_steps_per_s\":%.0f,\"vs_T4\":%.3f}\n", label, dq, obs_eq ? "true" : "false",
               (double)nworld * steps * reps / osec, sec / osec);
        fflush(stdout);
    }
    // T6: no stream capture at all -- a hand-built graph with the Warp graph
    // as a child. Splits "this graph cannot be a child" from "a child cannot
    // be added during capture". Then the memcpy nodes alone and the kernels.
    {
        cudaGraph_t outer; CK(cudaGraphCreate(&outer, 0));
        cudaGraphNode_t node;
        cudaError_t e = cudaGraphAddChildGraphNode(&node, outer, nullptr, 0, wg);
        cudaGetLastError();
        printf("{\"probe\":\"T6_child_node_no_capture\",\"add\":\"%s\"", cudaGetErrorString(e));
        if (e == cudaSuccess) {
            cudaGraphExec_t ox; cudaError_t ie3 = cudaGraphInstantiate(&ox, outer, 0);
            printf(",\"instantiate\":\"%s\"", cudaGetErrorString(ie3));
            if (ie3 == cudaSuccess) {
                restore(c);
                CK(cudaGraphLaunch(ox, s)); CK(cudaStreamSynchronize(s));
                printf(",\"qpos_maxdiff_vs_ref\":%g", max_diff_qpos(c, ref));
            }
        }
        // which memcpy kinds does the Warp graph hold? (host memory in a node
        // is one thing a child graph refuses)
        size_t n = 0; cudaGraphGetNodes(wg, nullptr, &n);
        std::vector<cudaGraphNode_t> nodes(n); cudaGraphGetNodes(wg, nodes.data(), &n);
        int kinds[8] = {0};
        for (size_t i = 0; i < n; i++) {
            cudaGraphNodeType t; cudaGraphNodeGetType(nodes[i], &t);
            if (t == cudaGraphNodeTypeMemcpy) {
                cudaMemcpy3DParms p; cudaGraphMemcpyNodeGetParams(nodes[i], &p);
                if ((int)p.kind < 8) kinds[(int)p.kind]++;
            }
        }
        printf(",\"memcpy_kinds\":{\"h2h\":%d,\"h2d\":%d,\"d2h\":%d,\"d2d\":%d,\"default\":%d}}\n",
               kinds[0], kinds[1], kinds[2], kinds[3], kinds[4]);
        fflush(stdout);
    }
    return 0;
}
