"""Check src/aow_sim/gpu/puffer/smooth_temporal.h without a GPU: compile it as
plain C++, drive it the way puffer_aow.patch does (one call per (row, t) of
an (N, T, A) minibatch with terminals), and compare

  - the summed loss against a numpy reference of SmoothPPO's temporal term
    (pairs inside one episode, lam * sum (mu_{t+1} - mu_t)^2 / (N*T)), and
  - the gradient against central finite differences of that loss.

    python analysis/gpu/phase3/check_smooth.py

Needs a C++ compiler (c++ on PATH). The kernel's indexing (t = nt % T, the
neighbour rows at +-(A+1) in the fused logits) is mirrored here, not run.
"""
import ctypes
import subprocess
import tempfile
from pathlib import Path

import numpy as np

HDR = Path(__file__).resolve().parents[3] / "src/aow_sim/gpu/puffer/smooth_temporal.h"
SHIM = """
#include "%s"
extern "C" float batch(const float* mu, const float* term, int N, int T, int A,
                       float lam, float* grad) {
    float scale = 1.0f / float(N * T), loss = 0.0f;
    for (int nt = 0; nt < N * T; ++nt) {
        int t = nt %% T;
        bool th = term[nt] != 0.0f;
        int hp = t > 0 && !th && term[nt - 1] == 0.0f;
        int hn = t + 1 < T && !th && term[nt + 1] == 0.0f;
        const float* m = mu + nt * A;
        loss += aow_smooth_temporal(hp ? m - A : m, m, hn ? m + A : m, hp, hn, A,
                                    lam, scale, grad + nt * A);
    }
    return loss;
}
"""


def reference(mu, term, lam):
    N, T, A = mu.shape
    ok = (term[:, :-1] == 0) & (term[:, 1:] == 0)
    d = mu[:, 1:] - mu[:, :-1]
    return lam * float((d ** 2 * ok[..., None]).sum()) / (N * T)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        src, so = Path(tmp) / "shim.cpp", Path(tmp) / "shim.so"
        src.write_text(SHIM % HDR)
        subprocess.run(["c++", "-O1", "-shared", "-fPIC", "-o", str(so), str(src)], check=True)
        lib = ctypes.CDLL(str(so))
        lib.batch.restype = ctypes.c_float
        f32p = ctypes.POINTER(ctypes.c_float)

        def run(mu, term, lam):
            mu = np.ascontiguousarray(mu, np.float32)
            g = np.zeros_like(mu)
            loss = lib.batch(mu.ctypes.data_as(f32p), term.ctypes.data_as(f32p),
                             mu.shape[0], mu.shape[1], mu.shape[2], ctypes.c_float(lam),
                             g.ctypes.data_as(f32p))
            return float(loss), g

        rng = np.random.default_rng(0)
        N, T, A, lam = 6, 16, 3, 0.04
        mu = rng.normal(0, 1, (N, T, A)).astype(np.float32)
        term = (rng.random((N, T)) < 0.08).astype(np.float32)
        term[0, 0] = term[1, -1] = term[2, 5] = 1
        loss, g = run(mu, term, lam)
        ref = reference(mu.astype(np.float64), term, lam)
        print(f"loss: kernel {loss:.6e}  reference {ref:.6e}  rel {abs(loss - ref) / ref:.1e}"
              f"  ({int(term.sum())} terminals in {N}x{T})")
        eps, worst = 1e-3, 0.0
        for idx in np.ndindex(mu.shape):
            mp, mm = mu.astype(np.float64), mu.astype(np.float64)
            mp[idx] += eps
            mm[idx] -= eps
            fd = (reference(mp, term, lam) - reference(mm, term, lam)) / (2 * eps)
            worst = max(worst, abs(fd - g[idx]))
        print(f"grad: max |analytic - finite difference| {worst:.1e} "
              f"(grad scale {np.abs(g).max():.1e})")
        loss0, g0 = run(mu, term, 0.0)
        print(f"lam 0: loss {loss0}, grad max {np.abs(g0).max()}")
        ok = abs(loss - ref) / ref < 1e-5 and worst < 1e-6 and loss0 == 0 and not g0.any()
        print("PASS" if ok else "FAIL")
        return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
