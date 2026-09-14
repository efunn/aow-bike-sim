"""The creep test, rebuilt in the sim -- and the roller "flick" the bench cannot do.

Top rows: `analysis/drivetrain_creep_plot.py`'s time series (the same capture,
the same 20 ms differencing, speed as a multiple of the command) for the bench
and for the MuJoCo testbed -- the wheel on a stand, in the air, the Station A
rig -- replaying the capture's own recorded commands under each plant:

    bench              the capture
    full               config/drivetrain_model.yaml, every part on
    without slop       --drivetrain-without roller_slop
    without detent     --drivetrain-without detent
    without servo      the old native velocity-PI, plus detent and slop
    ideal              no overlay: the plant every policy trained on

THE CREEP TEST CANNOT SEE THE SLOP. Only the two servo shafts are encoded, and
with the wheel in the air nothing loads a roller, so the play between a roller
and the drive never reaches an encoder. `without slop` draws on top of `full`,
and that is the result, not a bug. The bottom row is the test that CAN see it,
done in the sim because on the real wheel it means a finger on a roller: the
drive held at goal 0, one roller pushed through a slow torque ramp and back
(angle against torque), then flicked at 20 rad/s (angle against time). Angles
are the roller relative to where the drive puts it, k_roller x ring-vs-hub.

The detent phase is not measured, so the sim's stalls land at different
absolute angles from the bench's; compare how often and how long, not where.

    python analysis/drivetrain_creep_replay.py
    python analysis/drivetrain_creep_replay.py --capture 260913-012157_creep --tag run1

Writes analysis/plots/drivetrain_creep_replay[_<tag>].png. Reads traces/drivetrain/.
"""

from __future__ import annotations

import argparse
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

BELT = 3.0
TRACES = ROOT / "traces" / "drivetrain"
VARIANTS = {                     # label -> with_drivetrain kwargs, None = ideal
    "full": dict(),
    "without slop": dict(without=("roller_slop",)),
    "without detent": dict(without=("detent",)),
    "without servo": dict(without=("servo",)),
    "ideal": None,
}
COLORS = {"bench": "#222222", "full": "#4477aa", "without slop": "#66ccee",
          "without detent": "#ee6677", "without servo": "#228833",
          "ideal": "#999999"}
SEGMENTS = (("diff", 0.01), ("diff", 0.03), ("common", 0.01))


def _params(variant):
    from aow_sim import drivetrain_model as dm
    from aow_sim.build_model import load_params
    base = load_params()
    spec = VARIANTS[variant]
    return base if spec is None else dm.with_drivetrain(base, **spec)


def _hold(raw):
    out, last = np.empty_like(raw), 0.0
    for i, v in enumerate(raw):
        if np.isfinite(v):
            last = v
        out[i] = last
    return out


def _fill(x):
    g = np.isfinite(x)
    return np.interp(np.arange(len(x)), np.flatnonzero(g), x[g])


def replay(job):
    """Servo-frame shaft angles at the capture's frame times, one plant."""
    import mujoco

    from aow_sim import drivetrain_model as dm
    from aow_sim.build_model import build_model
    from aow_sim.hw.bench_log import Capture
    capture, variant = job
    cap = Capture.load(TRACES / capture)
    p = _params(variant)
    m = build_model(p, variant="testbed")
    d = mujoco.MjData(m)
    sim = dm.DrivetrainSim.attach(m, p)
    sa, sb = cap.meta["signs"]
    A, B = cap.ids
    t = _fill(cap.time_s(A))
    ca, cb = _hold(cap.command(A)), _hold(cap.command(B))
    ia, ib = m.actuator("drive_a").id, m.actuator("drive_b").id
    qa, qb = m.joint("input_a_spin").qposadr[0], m.joint("input_b_spin").qposadr[0]
    pa, pb = np.empty(len(t)), np.empty(len(t))
    for k in range(len(t)):
        j = max(k - 1, 0)
        while d.time < t[k] - t[0] - 1e-9:
            d.ctrl[ia], d.ctrl[ib] = BELT * sa * ca[j], BELT * sb * cb[j]
            if sim is not None:
                sim.pre_step(d)
            mujoco.mj_step(m, d)
        pa[k], pb[k] = d.qpos[qa] / (BELT * sa), d.qpos[qb] / (BELT * sb)
    return variant, pa, pb


def roller_tests(variant):
    """(torque ramp, deviation) and (time, deviation) for one roller, drive held."""
    import mujoco

    from aow_sim import drivetrain_model as dm
    from aow_sim.build_model import build_model
    p = _params(variant)
    m = build_model(p, variant="testbed")
    k = p["drivetrain"]["k_roller"]
    ia, ib = m.actuator("drive_a").id, m.actuator("drive_b").id
    rq, rd = m.joint("roller_spin_0").qposadr[0], m.joint("roller_spin_0").dofadr[0]
    gq = m.joint("ring_spin").qposadr[0]
    h = m.opt.timestep

    def dev(d):
        return math.degrees(d.qpos[rq] - k * d.qpos[gq])

    out = []
    for test in ("push", "flick"):
        d = mujoco.MjData(m)
        sim = dm.DrivetrainSim.attach(m, p)
        d.ctrl[ia] = d.ctrl[ib] = 0.0
        if test == "flick":
            d.qvel[rd] = 20.0
        n = int((6.0 if test == "push" else 0.4) / h)
        xs, ys = [], []
        for i in range(n):
            if test == "push":
                s = i / n                     # 0 -> +T -> -T -> 0
                tq = 2e-3 * (4 * s if s < 0.25 else 2 - 4 * s if s < 0.75 else 4 * s - 4)
                d.qfrc_applied[rd] = tq
            if sim is not None:
                sim.pre_step(d)
            mujoco.mj_step(m, d)
            if i % 10 == 0:
                xs.append(d.qfrc_applied[rd] * 1e3 if test == "push" else d.time * 1e3)
                ys.append(dev(d))
        out.append((np.array(xs), np.array(ys)))
    return variant, out


def mode(pa, pb, sa, sb, which):
    return BELT * (sa * pa - sb * pb) / 2 if which == "diff" else BELT * (sa * pa + sb * pb) / 2


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from aow_sim.hw.bench_log import Capture
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", default="260913-013834_creep")
    ap.add_argument("--window", type=float, nargs=2, default=(1.0, 6.0),
                    metavar=("T0", "T1"), help="seconds into each segment to draw")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    cap = Capture.load(TRACES / args.capture)
    sa, sb = cap.meta["signs"]
    A, B = cap.ids
    t = _fill(cap.time_s(A))
    runs = {"bench": (_fill(cap.position_rad(A)), _fill(cap.position_rad(B)))}
    with ProcessPoolExecutor(max_workers=len(VARIANTS)) as ex:
        for v, pa, pb in ex.map(replay, [(args.capture, v) for v in VARIANTS]):
            runs[v] = (pa, pb)
        rollers = dict(ex.map(roller_tests, ["full", "without slop"]))

    rows = ["bench"] + list(VARIANTS)
    fig, axes = plt.subplots(len(rows) + 1, len(SEGMENTS),
                             figsize=(14, 2.05 * (len(rows) + 1)), squeeze=False)
    h = 5
    table = {row: [] for row in rows}
    for j, (which, frac) in enumerate(SEGMENTS):
        ks = [k for k, s in enumerate(cap.meta["segments"])
              if s["info"].get("kind") == "creep" and s["info"]["mode"] == which
              and np.isclose(s["info"]["frac"], frac)]
        sl = cap.segment_slice(ks[0])
        info = cap.meta["segments"][ks[0]]["info"]
        cmd = BELT * abs(info["v"])
        ts = t[sl] - t[sl][0]
        win = (ts >= args.window[0]) & (ts <= args.window[1])
        for i, row in enumerate(rows):
            q = mode(*runs[row], sa, sb, which)
            v = np.full(len(q), np.nan)
            v[h:-h] = (q[2 * h:] - q[:-2 * h]) / (t[2 * h:] - t[:-2 * h])
            r = v[sl] / cmd
            steady = r[len(r) // 6:]
            stuck = 100 * np.nanmean(steady < 0.25)
            burst = np.nanpercentile(steady, 99)
            ax = axes[i][j]
            ax.axhline(1.0, color="0.75", lw=0.8)
            ax.axhline(0.25, color="0.85", lw=0.6, ls=":")
            ax.plot(ts[win], r[win], color=COLORS[row], lw=0.9)
            ax.set_ylim(-1.0, 9.5 if frac < 0.02 else 4.5)
            ax.set_xlim(*args.window)
            ax.text(0.99, 0.93, f"stuck {stuck:.0f} %   burst {burst:.1f}x",
                    transform=ax.transAxes, ha="right", va="top", fontsize=8.5,
                    color=COLORS[row])
            if j == 0:
                ax.set_ylabel(row, fontsize=10, color=COLORS[row])
            if i == 0:
                ax.set_title(f"{which} {frac:+.0%} creep: speed / command", fontsize=10)
            if i < len(rows) - 1:
                ax.set_xticklabels([])
            else:
                ax.set_xlabel("s into segment", fontsize=8)
            ax.tick_params(labelsize=7)
            table[row].append(f"stuck {stuck:3.0f}% burst {burst:4.1f}x")
    print(f"{'plant':<16s}" + "".join(f"{w} {f:+.0%}".rjust(24) for w, f in SEGMENTS))
    for row in rows:
        print(f"{row:<16s}" + "".join(c.rjust(24) for c in table[row]))

    ax_push, ax_flick, ax_note = axes[-1]
    for v, style in (("full", "-"), ("without slop", "--")):
        (tq, dp), (tf, df) = rollers[v]
        ax_push.plot(tq, dp, style, color=COLORS[v], lw=1.2, label=v)
        ax_flick.plot(tf, df, style, color=COLORS[v], lw=1.2, label=v)
    ax_push.set_xlabel("torque pushed on one roller [mN.m]   (drive held at goal 0)", fontsize=8)
    ax_push.set_ylabel("roller vs drive [deg]", fontsize=9)
    ax_push.set_title("sim: push one roller slowly there and back", fontsize=10)
    ax_flick.set_xlabel("ms after the flick", fontsize=8)
    ax_flick.set_title("sim: flick one roller at 20 rad/s", fontsize=10)
    for ax in (ax_push, ax_flick):
        ax.axhline(0, color="0.8", lw=0.6)
        ax.legend(fontsize=8, loc="lower right")
        ax.tick_params(labelsize=7)
    ax_note.axis("off")
    ax_note.text(0.0, 0.95,
                 "The creep rows cannot see the slop:\nonly the servo shafts are "
                 "encoded, and in\nthe air nothing loads a roller, so\n"
                 "'without slop' draws on top of 'full'.\n\nThe detent phase is "
                 "unmeasured, so sim\nstalls land at other absolute angles.\n"
                 "Compare how often and how long.\n\n"
                 f"capture {args.capture}, factory P100 / I1920", fontsize=8.5,
                 va="top", color="0.25")
    fig.tight_layout()
    tag = f"_{args.tag}" if args.tag else ""
    out = ROOT / "analysis" / "plots" / f"drivetrain_creep_replay{tag}.png"
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
