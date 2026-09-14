"""Drivetrain chirp at several firmware velocity-loop gains: where the peaks are.

    python analysis/drivetrain_chirp_plot.py        # -> analysis/plots/drivetrain_chirp_gain.png
    python analysis/drivetrain_chirp_plot.py --tag alt --captures <c1> <c2> ...

Reads chirp captures from traces/drivetrain/ (gitignored, Dropbox-synced) and
changes nothing. Each capture is identified by its Velocity P AND I gains, read
from the capture itself (factory 100 / 1920 where it set none), because two
captures at the same P are otherwise indistinguishable. Both velocities come
from the two servo ENCODERS, not the firmware's filtered Present Velocity:

    common = belt * (sa*wA + sb*wB) / 2     the hub
    diff   = belt * (sa*wA - sb*wB) / 2     ring-vs-hub, i.e. the rollers

in input-shaft rad/s, with the sign convention and belt ratio each capture
stores in its own meta.json.

Top row: tracking (achieved / commanded fundamental, sine fit in windows of 3
periods, +-1 frame difference corrected for its sinc attenuation) against
frequency. Below: each sweep against its instantaneous frequency -- the band
is the achieved velocity in the commanded mode, the lines are +-the commanded
amplitude, and the gray trace underneath is the OTHER mode, i.e. leakage.
Traces use a +-2 frame difference (8 ms span), which under-reads by at most
16 % at 40 Hz; the tracking curves are corrected, the traces are not.

Phase is measured against the time a command was computed, so it includes the
~4 ms before the servo's velocity reference moves (drivetrain-measurements.yaml,
`latency`); the tracking magnitude does not depend on that.

Numbers behind it: docs/measurements/drivetrain-measurements.yaml, `chirp_vs_gain`.
"""

from __future__ import annotations

import argparse
import sys
import warnings
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.legend_handler import HandlerTuple  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from aow_sim.hw.bench_log import Capture  # noqa: E402

DEFAULT_CAPTURES = ("260913-001455_chirp",     # P 100 / I 1920 (factory)
                    "260913-014930_chirp",     # P 200 / I 1920
                    "260913-013646_chirp",     # P 400 / I 1920
                    "260913-134255_chirp")     # P 400 / I 3840
FRAME_S = 0.002
FACTORY_I = 1920

# Reference data-viz palette: text inks and chrome, and categorical slots 1-4 in
# order, validated for adjacent pairs (lines). Colour follows the GAIN SETTING,
# never its position in the list; slots 3 and 4 sit under 3:1 contrast, so every
# trace panel's title names its gains.
SURFACE, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
GAIN_COLOR = {(100, 1920): "#2a78d6", (200, 1920): "#eb6834",
              (400, 1920): "#1baf7a", (400, 3840): "#eda100"}
EXTRA = ("#e87ba4", "#008300")        # slots 5-6, only if another setting is added


def _fund(x, t, f):
    g = np.isfinite(x) & np.isfinite(t)
    X = np.column_stack([np.sin(2 * np.pi * f * t[g]), np.cos(2 * np.pi * f * t[g]),
                         np.ones(g.sum())])
    a = np.linalg.lstsq(X, x[g], rcond=None)[0]
    return complex(a[0], a[1])


def _modes(cap: Capture, half_span: int):
    sa, sb = cap.meta["signs"]
    belt = cap.meta["params"]["belt_ratio"]["value"]
    ia, ib = cap.ids
    wa, wb = cap.diff_velocity(ia, half_span), cap.diff_velocity(ib, half_span)
    ca, cb = cap.command(ia), cap.command(ib)
    return ({"common": belt * (sa * wa + sb * wb) / 2, "diff": belt * (sa * wa - sb * wb) / 2},
            {"common": belt * (sa * ca + sb * cb) / 2, "diff": belt * (sa * ca - sb * cb) / 2})


def sweep(cap: Capture) -> dict:
    """{mode: freq, v, v_other, amp, fc, gain, gains} for each chirp segment."""
    th = cap.arrays["t_host"]
    v1, cmd = _modes(cap, 1)
    v2, _ = _modes(cap, 2)
    out = {}
    for k, s in enumerate(cap.segments):
        info = s["info"]
        if info.get("kind") != "chirp":
            continue
        sl, mode = cap.segment_slice(k), info["mode"]
        other = "diff" if mode == "common" else "common"
        t = th[sl] - th[sl][0]
        T, f0, f1 = s["seconds"], info["f0"], info["f1"]
        kk = f1 / f0
        fcs = np.geomspace(f0 * 1.15, f1 / 1.1, 45)
        gain = []
        for fc in fcs:
            tc, w = T * np.log(fc / f0) / np.log(kk), 1.5 / fc
            m = (t > tc - w) & (t < tc + w)
            if m.sum() < 12:
                gain.append(np.nan)
                continue
            G = (_fund(v1[mode][sl][m], t[m], fc) / _fund(cmd[mode][sl][m], t[m], fc)
                 / np.sinc(fc * 2 * FRAME_S))
            gain.append(abs(G))
        out[mode] = {"freq": f0 * kk ** (t / T), "v": v2[mode][sl], "v_other": v2[other][sl],
                     "amp": float(np.nanmax(np.abs(cmd[mode][sl]))), "fc": fcs,
                     "gain": np.array(gain),
                     "gains": (int(info.get("kvp", 100)), int(info.get("kvi", FACTORY_I)))}
    return out


def _label(gains) -> str:
    return f"P {gains[0]} / I {gains[1]}"


def _style(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(1)
    ax.grid(True, color=GRID, linewidth=0.8, linestyle="-")
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelcolor=INK2, labelsize=9, length=3)
    ax.set_xscale("log")
    ax.set_xlim(0.5, 40)
    ticks = [0.5, 1, 2, 5, 10, 20, 40]
    ax.set_xticks(ticks)
    ax.set_xticklabels([f"{x:g}" for x in ticks])
    ax.minorticks_off()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--captures", nargs="+", default=list(DEFAULT_CAPTURES),
                    help="capture directory names under traces/drivetrain/")
    ap.add_argument("--tag", help="variant suffix for the output name")
    args = ap.parse_args()

    warnings.simplefilter("ignore", RuntimeWarning)
    data = sorted((sweep(Capture.load(ROOT / "traces" / "drivetrain" / c)) for c in args.captures),
                  key=lambda d: next(iter(d.values()))["gains"])
    colors, spare = {}, iter(EXTRA)
    for d in data:
        gains = next(iter(d.values()))["gains"]
        colors[gains] = GAIN_COLOR.get(gains) or next(spare)

    plt.rcParams["font.family"] = ["Helvetica Neue", "Arial", "DejaVu Sans"]
    fig = plt.figure(figsize=(12, 3.8 + 2.35 * len(data)), facecolor=SURFACE)
    # Row 1 is a spacer holding the trace legend, so it belongs to every trace
    # row equally instead of sitting inside (and spilling out of) the first.
    gs = fig.add_gridspec(2 + len(data), 2, height_ratios=[1.45, 0.16] + [1] * len(data),
                          hspace=0.55, wspace=0.14, top=0.90, bottom=0.06, left=0.075, right=0.985)
    fig.text(0.075, 0.965, f"Drivetrain chirp at {len(colors)} firmware velocity-loop gain settings",
             color=INK, fontsize=14, weight="semibold")
    fig.text(0.075, 0.94, "15 % of no-load, wheel in the air, hand held. Velocities from the "
             "servo encoders, at the input shafts. common = hub, diff = rollers.",
             color=INK2, fontsize=10)

    for col, mode in enumerate(("common", "diff")):
        ax = fig.add_subplot(gs[0, col])
        _style(ax)
        ax.axhline(1.0, color=AXIS, linewidth=1)
        for d in data:
            s = d[mode]
            ax.plot(s["fc"], s["gain"], color=colors[s["gains"]], linewidth=2,
                    solid_capstyle="round", solid_joinstyle="round", label=_label(s["gains"]))
        top = max(data, key=lambda d: np.nanmax(d[mode]["gain"]))[mode]
        j = int(np.nanargmax(top["gain"]))
        if top["gain"][j] > 1.0:
            ax.plot(top["fc"][j], top["gain"][j], "o", markersize=8, color=colors[top["gains"]],
                    markeredgecolor=SURFACE, markeredgewidth=2, zorder=5)
            ax.annotate(f"{_label(top['gains'])} peak {top['gain'][j]:.2f} at {top['fc'][j]:.0f} Hz",
                        (top["fc"][j], top["gain"][j]), xytext=(-12, 8),
                        textcoords="offset points", ha="right", color=INK2, fontsize=9)
        ax.set_ylim(0, 1.35)
        ax.set_title(f"{mode}: tracking against frequency", loc="left", color=INK, fontsize=11)
        if col == 0:
            ax.set_ylabel("achieved / commanded", color=INK2, fontsize=10)
        else:
            # In the DIFF panel, lower left, one column: below its P 100 curve at
            # 0.5-1.7 Hz nothing is drawn. Both panels' tops carry the peak
            # label, and the common panel's P 100 roll-off crosses any corner.
            ax.legend(frameon=False, labelcolor=INK2, fontsize=9, loc="lower left", ncol=1)
        ax.set_xlabel("frequency, Hz", color=INK2, fontsize=9)

    lax = fig.add_subplot(gs[1, :])
    lax.axis("off")
    achieved = tuple(Line2D([], [], color=colors[k], linewidth=2) for k in colors)
    lax.legend(handles=[Line2D([], [], color=INK2, linewidth=1), achieved,
                        Line2D([], [], color=MUTED, linewidth=2)],
               labels=["± commanded amplitude", "achieved in the commanded mode, by gain setting",
                       "achieved in the other mode (leakage)"],
               handler_map={tuple: HandlerTuple(ndivide=None, pad=0.3)}, handlelength=5,
               loc="center", ncol=3, frameon=False, labelcolor=INK2, fontsize=9)

    for r, d in enumerate(data, start=2):
        for col, mode in enumerate(("common", "diff")):
            s = d[mode]
            ax = fig.add_subplot(gs[r, col])
            _style(ax)
            ax.plot(s["freq"], s["v_other"], color=MUTED, linewidth=0.6, alpha=0.9)
            ax.plot(s["freq"], s["v"], color=colors[s["gains"]], linewidth=0.7)
            for sign in (1, -1):
                ax.axhline(sign * s["amp"], color=INK2, linewidth=1)
            lim = 1.7 * s["amp"]
            ax.set_ylim(-lim, lim)
            ax.set_title(f"{_label(s['gains'])}, {mode} commanded", loc="left", color=INK,
                         fontsize=10)
            if col == 0:
                ax.set_ylabel("rad/s", color=INK2, fontsize=10)
            if r == len(data) + 1:
                ax.set_xlabel("chirp frequency at that moment, Hz", color=INK2, fontsize=9)

    out = ROOT / "analysis" / "plots" / ("drivetrain_chirp_gain" + (f"_{args.tag}" if args.tag else "") + ".png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f"wrote {out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
