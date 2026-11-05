"""The XC330's Present Current against MOTOR torque: one cubic through every
measurement of it, and the figure that shows why the traced graph alone is
not enough.

ROBOTIS's graph (perf_graph_fit.py) is drawn at FULL duty, so along it low
torque comes with high speed. Its current reads 135 mA at no load, where the
bike's own XC330 on a bare shaft, full duty, 113 rpm, reads 27 mA
(docs/measurements/servo-measurements.yaml, xc330_no_load); and at 35-189 mA
it needs 60-110 mA more than the X330 fixture's lever did for the same torque
(docs/plans/righting-servo-model.md). Near stall it is the only measurement
there is. So the law is one cubic in motor torque fitted to:

  * the lever's five lifts from rest (stall; static gearbox friction,
    0.51 x load + 5.1 mN m, takes the load to motor torque);
  * the bare shaft at full duty, no load (running friction, 5.7 mN m);
  * the graph's traced current from 0.30 N m of OUTPUT torque to the end of
    its trace (0.379), taken to motor torque through the running gearbox
    loss, 0.15 x load + 5.7 mN m;

weighted so the six bench points and the graph's stretch count equally.
Current, not speed, is what it depends on: the no-load point at full speed
sits on the lever's line.

The bike's lifts (full duty, crank 0-40) are drawn but NOT fitted: their
torque is not measured, only placed by the graph's speed curve at the speed
they ran (median 11.9 V, so the 12 V curve), and they sit 35-40 mA under
both the graph and this law. Either the law reads ~35 mA high there or this
motor gives ~5 % less torque at those speeds than the graph says; the bike's
data cannot tell which.

    python analysis/servo/current_law_fit.py
    -> coefficients printed (servos.xc330_t181.current_law),
       analysis/servo/plots/xc330_current_law.png
"""
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from aow_sim.build_model import load_params  # noqa: E402
from aow_sim.gearbox_friction import constants  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "plots" / "xc330_current_law.png"

# The X330 fixture's lever, lifting from rest: load [N m at the output] and
# the Goal Current it lifted above [mA] (righting-servo-model.md, 2026-09-26).
LEVER_LOAD = np.array([8.3, 14.7, 33.6, 59.1, 101.9]) / 1000.0
LEVER_MA = np.array([35.0, 49.0, 79.0, 129.0, 189.0])
# The graph's stretch the fit uses, OUTPUT torque [N m]
GRAPH_FROM = 0.30
# The bike's lifts at full PWM, crank 0-40 deg, runs 10-26..11-03: crank
# speed bin centre [deg/s] -> median Present Current [mA]
# (traces/bike, the lift_speed table of 2026-11-05)
BIKE_LIFTS = [(35, 675), (65, 664), (95, 636), (125, 605), (170, 574)]


def v(x):
    return x["value"] if isinstance(x, dict) and "value" in x else x


def fit():
    p = load_params()
    fc = constants(p)
    pg = p["servos"]["xc330_t181"]["performance_graph"]
    ct = np.array(v(pg["torque_of_rpm"]), float)
    ci = np.array(v(pg["current_a"]), float)
    lo_t, hi_t = v(pg["torque_traced"])
    r0, rc, s0, sc = fc["running_nm"], fc["running_frac"], fc["static_nm"], fc["static_frac"]
    meas = yaml.safe_load(open(ROOT / "docs/measurements/servo-measurements.yaml"))
    nl = [r for r in meas["xc330_no_load"]["speed_vs_duty"] if r["duty"] == 1.0][0]
    no_load_ma = 1000.0 * nl["current_A"]

    n = np.linspace(0.0, 106.2, 400)
    T = np.polyval(ct[::-1], n)
    gx, gy = (1 + rc) * T + r0, 1000.0 * np.polyval(ci[::-1], T)
    traced = (T >= lo_t) & (T <= hi_t)
    use = traced & (T >= GRAPH_FROM)
    lever_x = (1 + sc) * LEVER_LOAD + s0
    X = np.r_[lever_x, r0, gx[use]]
    Y = np.r_[LEVER_MA, no_load_ma, gy[use]]
    W = np.r_[np.full(6, 3.0), np.full(use.sum(), 18.0 * 3.0 / use.sum())]
    c = np.polyfit(X, Y, 3, w=W)[::-1] / 1000.0          # A, ascending powers
    return dict(c=c, ct=ct, ci=ci, r0=r0, rc=rc, n=n, T=T, gx=gx, gy=gy,
                traced=traced, use=use, lever_x=lever_x, no_load_ma=no_load_ma,
                lo_t=lo_t, hi_t=hi_t, pg=pg)


def law_ma(c, tau):
    return 1000.0 * np.polyval(np.asarray(c)[::-1], tau)


def main():
    f = fit()
    c, r0, rc = f["c"], f["r0"], f["rc"]
    xs = np.linspace(0.0, 0.6, 400)
    assert (np.diff(law_ma(c, xs)) > 0).all(), "not monotonic to 0.6 N m"
    print("servos.xc330_t181.current_law (A, motor torque N m, ascending):")
    print("  [" + ", ".join(f"{x:.6g}" for x in c) + "]")
    eb = law_ma(c, np.r_[f["lever_x"], r0]) - np.r_[LEVER_MA, f["no_load_ma"]]
    eg = law_ma(c, f["gx"][f["use"]]) - f["gy"][f["use"]]
    print(f"  bench points {eb.min():+.0f}..{eb.max():+.0f} mA; graph from {GRAPH_FROM} N m out "
          f"{eg.min():+.0f}..{eg.max():+.0f} mA")
    bx = [(1 + rc) * np.polyval(f["ct"][::-1], s / 6.0) + r0 for s, _ in BIKE_LIFTS]
    for x, (s, i) in zip(bx, BIKE_LIFTS):
        print(f"  bike lift {s:3d} deg/s: {i} mA measured, law {law_ma(c, x):.0f} at the graph's torque {x:.3f}")

    C = {"graph": "#2a78d6", "lever": "#eb6834", "noload": "#1baf7a", "bike": "#eda100", "law": "#e87ba4"}
    ink, mute = "#2b2b2a", "#8a897f"
    fig, ax = plt.subplots(figsize=(9, 6), dpi=150)
    tr = f["traced"]
    ax.plot(f["gx"][tr], f["gy"][tr], color=C["graph"], lw=2, label="ROBOTIS graph, full duty, 12 V (traced)")
    lo = f["gx"] < f["gx"][tr].min()
    ax.plot(f["gx"][lo], f["gy"][lo], color=C["graph"], lw=1.2, ls=":", label="graph, its quadratic past the trace")
    for rpm in (0, 17, 50, 90, 106):
        t = np.polyval(f["ct"][::-1], rpm)
        x, y = (1 + rc) * t + r0, 1000.0 * np.polyval(f["ci"][::-1], t)
        ax.plot(x, y, "o", ms=4, color=C["graph"])
        ax.annotate(f"{rpm} rpm", (x, y), xytext=(6, -12), textcoords="offset points", fontsize=8, color=mute)
    ax.plot(xs, xs / 0.83 * 1000 + 16.5, color=C["lever"], lw=1.2, ls="--",
            label="lever line 0.83 (I - 16.5 mA), fitted to 0.1 N m")
    ax.plot(f["lever_x"], LEVER_MA, "s", ms=8, color=C["lever"], mec="white", mew=1.5,
            label="X330 fixture lever, lifting from rest (5 loads)")
    ax.plot([r0], [f["no_load_ma"]], "D", ms=9, color=C["noload"], mec="white", mew=1.5,
            label="bare shaft, no load, full duty, 113 rpm")
    ax.plot(bx, [i for _, i in BIKE_LIFTS], "o", ms=8, color=C["bike"], mec="white", mew=1.5,
            label="bike lifts, full duty (torque placed by the graph's speed; not fitted)")
    ax.plot(xs, law_ma(c, xs), color=C["law"], lw=2, label="current_law: cubic, bench + graph from 0.30 N m out")
    ax.set_xlim(0, 0.5)
    ax.set_ylim(0, 760)
    ax.set_xlabel("motor torque [N m]  (the graph through the running gearbox, the lever through the static)", color=ink)
    ax.set_ylabel("Present Current [mA]", color=ink)
    ax.set_title("XC330-T181: Present Current against motor torque", color=ink, loc="left")
    ax.grid(color="#e6e5df", lw=0.6)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(mute)
    ax.tick_params(colors=mute)
    ax.legend(fontsize=7.5, frameon=False, loc="upper left")
    fig.tight_layout()
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT)
    print("wrote", OUT.relative_to(ROOT))


if __name__ == "__main__":
    main()
