"""Is the detent in the FLOOR runs' PWM?

    python analysis/servo/floor_detent.py      # -> analysis/servo/plots/floor_detent.png

 Average the diff-mode duty by notch
phase (the encoder frame the hung creep located the detent in), split by the
diff's direction of motion. Loads that do not know the notch (balance, ground)
average out; the detent does not. Out (+) reads angle-only + friction, back (-)
angle-only - friction, as in the bench map."""
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "policy")); sys.path.insert(0, str(HERE))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import drivetrain_trace as D
import drivetrain_fit as F
from aow_sim.hw.bench_log import Capture

b = D.bench_detent("261024-221037_creep_film")
ids, signs = b["ids"], b["signs"]
NB = 15
edges = np.linspace(-3.75, 3.75, NB + 1)
mid = 0.5 * (edges[1:] + edges[:-1])


def binned(x, y):
    m = np.full(NB, np.nan); se = np.full(NB, np.nan); n = np.zeros(NB, int)
    k = np.clip(np.digitize(x, edges) - 1, 0, NB - 1)
    for i in range(NB):
        v = y[k == i]
        n[i] = len(v)
        if len(v) > 20:
            m[i], se[i] = v.mean(), v.std() / np.sqrt(len(v))
    return m, se, n


def diff_duty(c):
    (A, B), (sa, sb) = ids, signs
    dA, dB = (F._fill(c.value("Present PWM", i)) for i in (A, B))
    return (sa * dA - sb * dB) / 2          # + pushes ring-vs-hub +


fig, axs = plt.subplots(1, 3, figsize=(17, 4.6), sharey=True)
for ax, run in zip(axs, ("261024-174349_run", "261024-191558_run")):
    c = Capture.load(f"traces/bike/{run}")
    t = c.bike("t"); eng = c.bike("state").astype(int) == 0
    ph = D._phi_enc(c, ids, signs)
    w = np.gradient(ph, t)                        # deg/s of ring-vs-hub
    x = (ph - b["flip"] + 3.75) % 7.5 - 3.75      # 0 = the flip
    d = diff_duty(c)
    for lab, sel, col in (("moving +", eng & (w > 30), "#d0532a"), ("moving -", eng & (w < -30), "#2a6fc9"),
                          ("still", eng & (np.abs(w) < 20), "#646d73")):
        m, se, n = binned(x[sel], d[sel])
        ax.errorbar(mid, m, yerr=2 * se, color=col, marker="o", ms=3, lw=1.4, capsize=2,
                    label=f"{lab} ({sel.sum()} samples)")
    mp, _, _ = binned(x[eng & (w > 30)], d[eng & (w > 30)])
    mm, _, _ = binned(x[eng & (w < -30)], d[eng & (w < -30)])
    ang, fr = (mp + mm) / 2, (mp - mm) / 2
    print(f"{run}: diff duty sd {d[eng].std():.3f}; angle-only (out+back)/2 p-p {np.nanmax(ang) - np.nanmin(ang):.3f}, "
          f"friction (out-back)/2 mean {np.nanmean(fr):.3f}")
    print("   angle-only by bin: " + " ".join(f"{v:+.3f}" for v in ang - np.nanmean(ang)))
    ax.axvline(0, color="k", lw=0.8); ax.axhline(0, color="#bbb", lw=0.8)
    ax.set_title(f"floor {run} (policy engaged)", loc="left", fontsize=10)
    ax.set_xlabel("ring-vs-hub from the hung creep's flip, deg (+-3.75 = midway)")
    ax.legend(fontsize=8)
axs[0].set_ylabel("diff-mode duty, mean +- 2 SE (+ pushes ring-vs-hub +)")

# the hung P 4000 map in the same frame, for scale
from drivetrain_bench import map_profile
cap = F._load("261024-211612_map")
prof = map_profile(cap, "diff")
ax = axs[2]
for dsgn, col, lab in ((1, "#d0532a", "out (+)"), (-1, "#2a6fc9", "back (-)")):
    pr = prof[dsgn]
    # map_profile's angle is ring-vs-hub deg in the capture's own frame: same formula, same encoders
    xx = (pr[:, 0] - b["flip"] + 3.75) % 7.5 - 3.75
    o = np.argsort(xx)
    ax.plot(xx[o], pr[o, 1], ".", color=col, ms=3, label=f"hung map {lab}")
ax.axvline(0, color="k", lw=0.8); ax.axhline(0, color="#bbb", lw=0.8)
ax.set_title("hung static map 261024-211612 (P 4000), same frame", loc="left", fontsize=10)
ax.set_xlabel("ring-vs-hub from the flip, deg"); ax.legend(fontsize=8)
fig.tight_layout()
out = str(Path(__file__).resolve().parent / "plots" / "floor_detent.png")
fig.savefig(out, dpi=110)
print("wrote", out)
