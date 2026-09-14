"""Creep: where the drivetrain sticks, in diff and in common.

    python analysis/drivetrain_creep_plot.py    # -> analysis/plots/drivetrain_creep_stickiness.png
    python analysis/drivetrain_creep_plot.py --tag alt --series <capture> --fold <c1> <c2> ...

Reads `creep` captures from traces/drivetrain/ (gitignored, Dropbox-synced)
and changes nothing. Velocity is from the two servo ENCODERS, combined into the
two drivetrain modes at the input shafts:

    common = belt * (sa*A + sb*B) / 2     the hub
    diff   = belt * (sa*A - sb*B) / 2     ring-vs-hub, i.e. the rollers

Top: the time series through the +1 % and +3 % creep against the commanded
speed, differenced over +-5 frames (20 ms). Not tighter: one encoder count on
one servo is 0.13 deg of mode angle, which over 8 ms reads as 16 deg/s -- as
large as the whole 1 % command. The stalls last 100-400 ms.

Bottom: the same motion FOLDED onto one 7.5 deg cycle of its own angle -- every
tooth of the differential's mesh repeats every 7.5 deg of ring-vs-hub angle.
Plotted is the AVERAGE SPEED THROUGH each part of the tooth, i.e. travel per
pass divided by the time spent there, as a multiple of the command. That, not
a per-bin median of frame speeds: at 1 % each bin holds a mix of stuck frames
and burst frames, and a median of a two-valued mix jumps between them. The
angle track is resampled to 0.2 ms first, so a burst that crosses a bin between
two frames still counts its time there. Line = both runs pooled, band = the
range between the runs. Common mode is folded onto its OWN best period,
8.44 deg (`COMMON_PERIOD_DEG`, `--common-period`), a real but soft ripple whose
phase does not hold across every segment. Folded onto 7.5 deg it is flat at 1,
which is the evidence the hub has no share in the diff detent; re-run with
`--common-period 7.5 --tag common75` to see that.

The phase reference is the raw angle modulo 7.5 deg, comparable across
captures only while the servos keep their absolute single-turn positions. The
two default runs agree to ~0.5 deg (docs/measurements/drivetrain-measurements.yaml,
`diff_detent`).
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
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

from aow_sim.hw.bench_log import Capture  # noqa: E402

DEFAULT_SERIES = "260913-013834_creep"
DEFAULT_FOLD = ("260913-012157_creep", "260913-013834_creep")
PERIOD_DEG = 7.5             # the differential's mesh, from the tooth counts
# Common mode's own ripple. NOT a tooth count: the best pooled fold of the 1 %
# common creeps, 8.438 deg in BOTH runs (scan in 0.002 deg steps, contrast =
# std of log through-speed). 3 % prefers 8.31-8.44 and is weaker, and the
# ripple's phase does not hold across every segment, so this is a real but soft
# line -- nothing like the 7.5 deg diff detent. Folded on 7.5 deg instead,
# common is flat at 1.0 (the control this row used to show).
COMMON_PERIOD_DEG = 8.44
NBINS = 40
HALF_SPAN = 5
FINE_S = 0.0002
SKIP_S = 0.5                 # the start-up from rest is not creep
# "Sticky" = averaging slower than this multiple of the command. Half, not a
# quarter: averaged over every pass, no part of the tooth is ever fully stopped
# (the stall position wanders a little pass to pass), so a quarter reported
# ~0 deg for a band plainly visible in the curve.
STALL_FRAC = 0.5
YMAX_FOLDED = 8.0

SURFACE, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
SPEED_COLOR = {0.01: "#2a78d6", 0.03: "#eb6834"}    # reference categorical slots 1-2


def mode_data(cap: Capture, mode: str):
    """(angle deg, velocity deg/s, belt) of one drivetrain mode, whole capture."""
    sa, sb = cap.meta["signs"]
    belt = cap.meta["params"]["belt_ratio"]["value"]
    ia, ib = cap.ids
    k = 1.0 if mode == "common" else -1.0
    pa, pb = cap.position_rad(ia), cap.position_rad(ib)
    wa, wb = cap.diff_velocity(ia, HALF_SPAN), cap.diff_velocity(ib, HALF_SPAN)
    return (np.degrees(belt * (sa * pa + k * sb * pb) / 2),
            np.degrees(belt * (sa * wa + k * sb * wb) / 2), belt)


def creep_segments(cap: Capture, mode: str, frac: float, sign: int):
    for k, s in enumerate(cap.segments):
        info = s["info"]
        if (info.get("kind") == "creep" and info["mode"] == mode
                and abs(abs(info["frac"]) - frac) < 1e-9 and np.sign(info["frac"]) == sign):
            yield k, s


def _occupancy(cap: Capture, mode: str, frac: float, sign: int, period: float):
    """(time spent per bin [s], degrees travelled, commanded deg/s) for one capture."""
    ang, _, belt = mode_data(cap, mode)
    tick = cap.time_s(cap.ids[0])
    th = cap.arrays["t_host"]
    edges = np.linspace(0, period, NBINS + 1)
    occ, travel, cmd = np.zeros(NBINS), 0.0, None
    for k, s in creep_segments(cap, mode, frac, sign):
        sl = cap.segment_slice(k)
        cmd = np.degrees(belt * abs(s["info"]["v"]))
        m = ((th[sl] - th[sl][0]) > SKIP_S) & np.isfinite(ang[sl]) & np.isfinite(tick[sl])
        ts, xs = tick[sl][m], ang[sl][m]
        if len(ts) < 50:
            continue
        fine = np.arange(ts[0], ts[-1], FINE_S)
        xf = np.interp(fine, ts + np.arange(len(ts)) * 1e-9, xs)
        occ += np.histogram(xf % period, bins=edges)[0] * FINE_S
        travel += abs(xs[-1] - xs[0])
    return occ, travel, cmd


def _through_speed(occ, travel, cmd, period):
    """Average speed through each bin, over the command: travel per pass / time there."""
    passes = travel / period
    with np.errstate(divide="ignore", invalid="ignore"):
        return (period / NBINS) * passes / occ / cmd


def folded(caps, mode: str, frac: float, sign: int, period: float):
    """(bin centres, pooled ratio, per-run min, per-run max), or None."""
    runs = [r for r in (_occupancy(c, mode, frac, sign, period) for c in caps)
            if r[2] is not None]
    if not runs:
        return None
    cmd = runs[0][2]
    pooled = _through_speed(sum(r[0] for r in runs), sum(r[1] for r in runs), cmd, period)
    each = np.array([_through_speed(r[0], r[1], cmd, period) for r in runs])
    edges = np.linspace(0, period, NBINS + 1)
    return (edges[:-1] + edges[1:]) / 2, pooled, each.min(axis=0), each.max(axis=0)


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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--series", default=DEFAULT_SERIES, help="capture for the time series")
    ap.add_argument("--fold", nargs="+", default=list(DEFAULT_FOLD),
                    help="captures pooled for the folded panels")
    ap.add_argument("--common-period", type=float, default=COMMON_PERIOD_DEG,
                    help="degrees to fold common mode onto (default: its best fold, 8.44)")
    ap.add_argument("--tag", help="variant suffix for the output name")
    args = ap.parse_args()
    warnings.simplefilter("ignore", RuntimeWarning)

    base = ROOT / "traces" / "drivetrain"
    series = Capture.load(base / args.series)
    fold_caps = [Capture.load(base / c) for c in args.fold]
    th = series.arrays["t_host"]

    plt.rcParams["font.family"] = ["Helvetica Neue", "Arial", "DejaVu Sans"]
    fig = plt.figure(figsize=(12, 14), facecolor=SURFACE)
    gs = fig.add_gridspec(4, 2, height_ratios=[1, 1, 1.15, 1.15], hspace=0.55,
                          wspace=0.14, top=0.885, bottom=0.045, left=0.075, right=0.985)
    fig.text(0.075, 0.968, "Where the drivetrain sticks: creep in diff and in common",
             color=INK, fontsize=14, weight="semibold")
    fig.text(0.075, 0.948, "Velocity from the servo encoders, at the input shafts. diff = ring "
             "vs hub (the rollers), common = the hub. Wheel in the air, hand held, firmware P 100.",
             color=INK2, fontsize=10)
    fig.legend(handles=[Line2D([], [], color=SPEED_COLOR[0.01], linewidth=2),
                        Line2D([], [], color=SPEED_COLOR[0.03], linewidth=2),
                        Line2D([], [], color=INK2, linewidth=1),
                        Patch(facecolor=MUTED, alpha=0.25, linewidth=0)],
               labels=["1 % of no-load", "3 % of no-load", "commanded speed",
                       "range between the two runs (folded panels)"],
               loc="center left", bbox_to_anchor=(0.068, 0.922), ncol=4, frameon=False,
               labelcolor=INK2, fontsize=9)

    # --- time series ------------------------------------------------------
    for row, mode in ((0, "diff"), (1, "common")):
        _, spd, belt = mode_data(series, mode)
        axes, peaks = [], []
        for col, frac in enumerate((0.01, 0.03)):
            ax = fig.add_subplot(gs[row, col])
            _style(ax)
            for k, s in creep_segments(series, mode, frac, +1):
                sl = series.segment_slice(k)
                t = th[sl] - th[sl][0]
                cmd = np.degrees(belt * abs(s["info"]["v"]))
                ax.axhline(0, color=AXIS, linewidth=1)
                ax.plot(t, spd[sl], color=SPEED_COLOR[frac], linewidth=0.9)
                ax.axhline(cmd, color=INK2, linewidth=1)
                ax.set_xlim(0, s["seconds"])
                peaks.append(np.nanpercentile(spd[sl], 99.8))
                ax.set_title(f"{mode}, +{frac:.0%} ({cmd:.0f} °/s commanded)", loc="left",
                             color=INK, fontsize=10)
            if col == 0:
                ax.set_ylabel("°/s, 20 ms difference", color=INK2, fontsize=10)
            ax.set_xlabel("seconds into the creep", color=INK2, fontsize=9)
            axes.append(ax)
        top = 1.08 * max(peaks)
        for ax in axes:
            ax.set_ylim(-0.12 * top, top)

    # --- folded ------------------------------------------------------------
    for row, mode in ((2, "diff"), (3, "common")):
        period = PERIOD_DEG if mode == "diff" else args.common_period
        for col, sign in enumerate((1, -1)):
            ax = fig.add_subplot(gs[row, col])
            _style(ax)
            ax.axhline(1.0, color=INK2, linewidth=1)
            notes = []
            for frac in (0.01, 0.03):
                c = folded(fold_caps, mode, frac, sign, period)
                if c is None:
                    continue
                x, pooled, lo, hi = c
                clip = lambda y: np.minimum(y, YMAX_FOLDED)   # noqa: E731
                ax.fill_between(x, clip(lo), clip(hi), color=SPEED_COLOR[frac], alpha=0.14,
                                linewidth=0)
                ax.plot(x, clip(pooled), color=SPEED_COLOR[frac], linewidth=2,
                        solid_capstyle="round", solid_joinstyle="round")
                stalled = np.sum(pooled < STALL_FRAC) * period / NBINS
                notes.append(f"{frac:.0%}: {stalled:.1f}°")
            ax.set_xlim(0, period)
            ax.set_xticks(np.arange(0, period + 0.01, 1.5))
            # Common's ripple is a few tens of percent; on diff's 0-8 scale it
            # is a flat line. Its own scale, said in the title.
            ax.set_ylim(0, YMAX_FOLDED if mode == "diff" else 3.0)
            what = ("one 7.5° tooth" if mode == "diff"
                    else f"{period:g}° (its best fold; NOTE the 0-3 scale)")
            ax.set_title(f"{mode} folded onto {what}, pushing {'+' if sign > 0 else '−'}",
                         loc="left", color=INK, fontsize=10)
            ax.text(0.99, 0.95, f"slower than {STALL_FRAC:.0%} of command over: "
                    + ",  ".join(notes), transform=ax.transAxes, ha="right", va="top",
                    color=INK2, fontsize=9)
            if col == 0:
                ax.set_ylabel("average speed through / commanded", color=INK2, fontsize=10)
            ax.set_xlabel(("ring-vs-hub" if mode == "diff" else "hub")
                          + " angle within the cycle, degrees", color=INK2, fontsize=9)

    out = ROOT / "analysis" / "plots" / ("drivetrain_creep_stickiness"
                                         + (f"_{args.tag}" if args.tag else "") + ".png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f"wrote {out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
