"""Trace ROBOTIS's performance graphs (docs/robotis/) and fit polynomials.

The black curve is speed (rpm) and the pink one current (A), both against
output torque. Each is traced column by column by colour, mapped through the
grid's pixel bounds, and fitted (`DEGREE`): output torque as a cubic in
speed, current as a quadratic in output torque. The figure is the
graph's grid alone (no axes artwork) with the fits drawn over it.

    python analysis/servo/perf_graph_fit.py            # both servos
    -> analysis/servo/plots/{xc330,xc430}_graph_fit.png, coefficients printed

The traced points are the graph as drawn: a line 2-3 px wide, so about
+-0.002 N m and +-0.5 rpm per point at these scales.
"""
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs" / "robotis"
OUT = Path(__file__).resolve().parent / "plots"

# Grid bounds in pixels (the outer gridlines, found from the image's gray
# lines) and the data range they span, read off the axis labels.
GRAPHS = {
    "xc330": dict(
        file="xc330-t181 _ ROBOTIS Docs_files/xc330-t181_performance_graph-927cf239ea80602ee0980350ce1bab01.webp",
        px=(102.0, 507.5), py=(441.0, 34.3), t=(0.0, 0.40), rpm=(0.0, 100.0), amp=(0.0, 1.0)),
    "xc430": dict(
        file="xc430-w150 _ ROBOTIS Docs_files/xc430_w150_performance_graph-842569cdea046c195d11471bf2e2d0b6.webp",
        px=(102.0, 509.0), py=(442.0, 35.3), t=(0.0, 1.00), rpm=(0.0, 110.0), amp=(0.0, 2.0)),
}


# Speed bends down faster than a quadratic over the last ~0.04 N m on both
# graphs (2.4 / 2.9 rpm off there); a cubic follows it. Current is quadratic.
# The speed curve is fitted ROTATED -- torque as a cubic in rpm -- because
# that is the way the servo model reads it (the most torque full duty gives
# at a speed); the curve is one-to-one, so nothing is lost.
DEGREE = {"rpm": 3, "amp": 2}

# The straight motor line the steer keeps (bike_params stall_torque): fitted
# over the output torques the steer works in, with no-load held at the bare
# shaft's measured 113 rpm, through the gearbox's running loss.
LINE_RANGE = (0.08, 0.28)
LINE_NO_LOAD = 113.0
RUN_FRAC, RUN_NM = 0.15, 0.0057     # servos.xc330_t181.friction_running_*


def poly_str(c, fmt, pow_sym="²³", var="T"):
    terms = [f"{c[0]:{fmt}}"]
    for k, ck in enumerate(c[1:], 1):
        p = "" if k == 1 else (f"^{k}" if pow_sym == "^" else pow_sym[k - 2])
        terms.append(f"{ck:+{fmt}} {var}{p}")
    return " ".join(terms)


def line_fit(T, n):
    """Motor stall torque [N m] of ts (1 - n/113) over LINE_RANGE, and the
    worst output-torque miss there."""
    m = (T >= LINE_RANGE[0]) & (T <= LINE_RANGE[1])
    tm = (1 + RUN_FRAC) * T[m] + RUN_NM
    x = 1 - n[m] / LINE_NO_LOAD
    S = float(x @ tm / (x @ x))
    return S, float(np.abs(S * x - tm).max() / (1 + RUN_FRAC))


def trace(img, mask, g):
    """Per pixel column inside the grid: the mean row of `mask`'s pixels,
    mapped to (torque, fraction of the y range). Columns with a thin or
    split trace (another curve's text or a crossing) are dropped."""
    x0, x1 = (int(round(v)) for v in g["px"])
    yb, yt = g["py"]
    T, F = [], []
    for c in range(x0 + 2, x1 - 1):
        lo, hi = int(yt) + 2, int(yb) - 2   # inside the frame: the axis lines are dark too
        rows = np.flatnonzero(mask[lo:hi + 1, c]) + lo
        # one contiguous run (a steep stretch spans many rows; its mean row is
        # still the line's centre), not two pieces
        if len(rows) < 2 or np.diff(rows).max() > 2:
            continue
        T.append(g["t"][0] + (c - x0) / (x1 - x0) * (g["t"][1] - g["t"][0]))
        F.append((yb - rows.mean()) / (yb - yt))
    return np.array(T), np.array(F)


def fit(name):
    g = GRAPHS[name]
    a = np.asarray(Image.open(DOCS / g["file"]).convert("RGB")).astype(int)
    r, gr, b = a[..., 0], a[..., 1], a[..., 2]
    black = (a.max(axis=2) < 90) & (np.ptp(a, axis=2) < 12)   # dark and gray, not the green
    pink = (r > 150) & (gr < 90) & (b > 50) & (b < 170)
    out = {}
    for key, mask, rng in (("rpm", black, g["rpm"]), ("amp", pink, g["amp"])):
        T, F = trace(a, mask, g)
        y = rng[0] + F * (rng[1] - rng[0])
        if key == "rpm":                                 # T = c[0] + c[1] n + ...
            c = np.polyfit(y, T, DEGREE[key])[::-1]
            res = np.polynomial.polynomial.polyval(y, c) - T
        else:                                            # I = c[0] + c[1] T + ...
            c = np.polyfit(T, y, DEGREE[key])[::-1]
            res = np.polynomial.polynomial.polyval(T, c) - y
        out[key] = (T, y, c, float(np.sqrt(np.mean(res ** 2))), float(np.abs(res).max()))
    return a, g, out


def plot(name, a, g, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    x0, x1 = g["px"]
    yb, yt = g["py"]
    crop = a[int(round(yt)):int(round(yb)) + 1, int(round(x0)):int(round(x1)) + 1].astype(np.uint8)
    fig, ax = plt.subplots(figsize=(6, 6 * crop.shape[0] / crop.shape[1]), dpi=150)
    ax.imshow(crop, extent=(g["t"][0], g["t"][1], 0.0, 1.0), aspect="auto")
    for key, rng, col, lab in (("rpm", g["rpm"], "tab:cyan", "speed"), ("amp", g["amp"], "tab:orange", "current")):
        T, y, c, rms, mx = out[key]
        if key == "rpm":
            nn = np.linspace(y.min(), y.max(), 200)
            tt, f = np.polynomial.polynomial.polyval(nn, c), (nn - rng[0]) / (rng[1] - rng[0])
            text = f"T = {poly_str(c, '.4g', var='n')} N m (n rpm; rms {rms * 1e3:.2g}, max {mx * 1e3:.2g} mN m)"
        else:
            tt = np.linspace(T.min(), T.max(), 200)
            f = (np.polynomial.polynomial.polyval(tt, c) - rng[0]) / (rng[1] - rng[0])
            text = f"I = {poly_str(c, '.4g')} A (rms {rms * 1e3:.2g}, max {mx * 1e3:.2g} mA)"
        ax.plot(tt, f, "--", color=col, lw=1.6, label=f"{lab}: {text}")
    ax.set_xlim(*g["t"]); ax.set_ylim(0, 1)
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title(f"{name}: grid {g['t'][1]:g} N m x {g['rpm'][1]:g} rpm / {g['amp'][1]:g} A, fits dashed",
                 fontsize=8)
    ax.legend(loc="upper center", fontsize=6.5, framealpha=0.9)
    OUT.mkdir(exist_ok=True)
    p = OUT / f"{name}_graph_fit.png"
    fig.savefig(p, bbox_inches="tight"); plt.close(fig)
    return p


def main():
    for name in GRAPHS:
        a, g, out = fit(name)
        p = plot(name, a, g, out)
        print(f"{name}  -> {p.relative_to(ROOT)}")
        T, y, c, rms, mx = out["rpm"]
        print(f"  torque of rpm: {poly_str(c, '.6g', '^', 'n')}   n {y.min():.1f}..{y.max():.1f}, "
              f"{len(T)} cols, rms {rms * 1e3:.3g}, max {mx * 1e3:.3g} mN m")
        if name == "xc330":                  # the steer's line: an XC330 only
            S, miss = line_fit(T, y)
            print(f"  line over {LINE_RANGE[0]:g}-{LINE_RANGE[1]:g} N m, no-load {LINE_NO_LOAD:g} rpm: "
                f"stall {S:.4f} N m (motor), worst miss {miss * 1e3:.1f} mN m")
        T, y, c, rms, mx = out["amp"]
        print(f"  current of T:  {poly_str(c, '.6g', '^')}   T {T.min():.3f}..{T.max():.3f}, "
              f"{len(T)} cols, rms {rms * 1e3:.3g}, max {mx * 1e3:.3g} mA")


if __name__ == "__main__":
    main()
