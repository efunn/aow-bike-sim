"""Where the steer XC330 works on its motor curve, from the bike's records.

    python analysis/servo/steer_load.py      # every capture under traces/bike/

Mode 4: Present PWM is the duty. A DC motor at duty u and supply s gives
tau = (u s) g(n / (u s)), g the 12 V full-duty curve. With the line,
g(n) = S (1 - n/n0); with the traced graph, the rotated cubic. Output
torque = (tau - 0.0057) / 1.15 while driving. Engaged samples only
(state 0), the pre-swap runs excluded."""
import glob, os, numpy as np
from aow_sim.params import load_params
from aow_sim.righting_servo import GraphLaw, LineLaw
from aow_sim.gearbox_friction import constants
p = load_params(); srv = p["servos"]["xc330_t181"]; fc = constants(p)
G = GraphLaw(srv, fc["running_nm"], fc["running_frac"]); L = LineLaw(srv)
SKIP = ("261024-163351", "261024-163836")
U, N, S, RUNS = [], [], [], 0
for d in sorted(glob.glob("traces/bike/26*")):
    f = d + "/capture.npz"
    if not os.path.exists(f) or any(s in d for s in SKIP):
        continue
    try:
        z = np.load(f, allow_pickle=True)
        ids = list(z["ids"])
        if 103 not in ids or "bike_state" not in z.files:
            continue
    except Exception:
        continue
    j = ids.index(103)
    m = (z["bike_state"] == 0) & z["ok"]
    if m.sum() < 50:
        continue
    pw = z["raw_present_pwm"][:, j].astype(np.int64); pw = np.where(pw > 32767, pw - 65536, pw)
    ve = z["raw_present_velocity"][:, j].astype(np.int64); ve = np.where(ve > 2**31 - 1, ve - 2**32, ve)
    v = z["raw_present_input_voltage"][:, j] * 0.1
    U.append(pw[m] / 885.0); N.append(ve[m] * 0.229); S.append(v[m] / 12.0); RUNS += 1
u, n, s = map(np.concatenate, (U, N, S))
print(f"{RUNS} runs, {len(u)} engaged samples")
au = np.abs(u)
print("duty |u| percentiles 50/90/99/99.9/max:", np.round(np.percentile(au, [50, 90, 99, 99.9, 100]), 3))
print(f"  at the PWM limit (|u| >= 0.99): {np.mean(au >= 0.99) * 100:.2f} %")
sg = np.sign(u); nd = sg * n            # rpm in the direction the duty drives
v = au * s
def out(law):
    t = np.array([vi * law.full(ni / vi, 1.0) if vi > 1e-3 else 0.0 for vi, ni in zip(v, nd)])
    return (t - 0.0057) / 1.15, t
To_g, tm_g = out(G); To_l, tm_l = out(L)
drv = (nd >= 0) & (au > 0.02)
print(f"driving samples {drv.mean() * 100:.0f} %, braking/back-driven {((nd < 0) & (au > 0.02)).mean() * 100:.0f} %")
T = To_g[drv]
print("output torque while driving, graph, percentiles 50/90/99/99.9/max N m:",
      np.round(np.percentile(T, [50, 90, 99, 99.9, 100]), 3))
for lim in (0.08, 0.20, 0.28, 0.35):
    print(f"  above {lim:.2f} N m: {np.mean(T > lim) * 100:.2f} % of driving samples")
diff = (To_l - To_g)[drv]
print("line minus graph, driving: percentiles 50/99/max |diff| mN m",
      np.round(np.percentile(np.abs(diff) * 1e3, [50, 99, 100]), 1))
hi = drv & (To_g > 0.28)
if hi.any():
    print(f"  where graph > 0.28: line - graph median {np.median((To_l - To_g)[hi]) * 1e3:+.0f} mN m, max {np.max(np.abs((To_l - To_g)[hi])) * 1e3:.0f}")
print("speed |n| rpm percentiles 50/90/99/max:", np.round(np.percentile(np.abs(n), [50, 90, 99, 100]), 1))

# Where on the 12 V curve: n_eq = n / (u s). The graph traced 16-92 rpm.
act = au > 0.02
neq = np.where(act, nd / np.maximum(v, 1e-9), np.nan)
bins = [(-1e9, 0, "back-driven (n_eq < 0)"), (0, 16, "0-16 (past the traced stall end)"),
        (16, 49.3, "16-49 (graph T > 0.28)"), (49.3, 92, "49-92 (T 0.08-0.28: the line's fit)"),
        (92, 106.2, "92-106 (T < 0.08, up to no load)"), (106.2, 1e9, "> 106 (back-EMF over the supply: braking)")]
print("\nsamples with |duty| > 0.02, by where they sit on the 12 V curve:")
for lo, hi, lab in bins:
    k = act & (neq >= lo) & (neq < hi)
    if not k.any():
        print(f"  {lab:44s}   0.00 %"); continue
    dt = (To_l - To_g)[k] * 1.15            # motor-torque difference, N m
    print(f"  {lab:44s} {k.sum() / act.sum() * 100:6.2f} %   |torque| median {np.median(np.abs(tm_g[k])):.3f} N m,"
          f"  line - graph median {np.median(dt) * 1e3:+5.1f}, 99% {np.percentile(np.abs(dt), 99) * 1e3:5.1f} mN m")
