"""Fit, and check, the detailed drivetrain against the Station A captures.

Every `measured` number in config/drivetrain_model.yaml comes out of one of
these sections, and `replay` checks the result the only way that means
anything: it runs the recorded command streams through the MuJoCo TESTBED
(the wheel on a stand, in the air -- the bench rig) with the overlay's
per-step loop, and computes the same tracking numbers on the capture and on
the sim with the same code. It exercises drivetrain_model.py itself, not a
copy of its equations.

    python analysis/servo/drivetrain_fit.py                    # every section
    python analysis/servo/drivetrain_fit.py controller coast detent
    python analysis/servo/drivetrain_fit.py replay --caps 260913-013646_chirp
    python analysis/servo/drivetrain_fit.py --session 261025   # a later session's captures

`--session PREFIX` swaps every section's captures for the ones under
traces/drivetrain/ whose names start with PREFIX, by kind (the name's
suffix): reversal/chirp/step for `controller`, the last coast for `coast`,
creep for `detent`, chirp/reversal/creep for `replay`. Without it the
sections read the 2026-09-13 bench set the config was fitted to.

  controller  Present PWM regressed on (v_ref - LPF(w), integral(v_ref) - theta)
              per firmware gain setting, over a grid of command delay and
              velocity filter. Reports Kp and Ki PER TABLE UNIT, which is what
              makes the model predictive for gains nobody has run.
  coast       PWM-mode spin-up and coast: rotor inertia, Coulomb and viscous
              friction per servo and per mode, by simulating each decay.
  detent      diff creep folded onto the 7.5 deg cycle, SPLIT BY PUSH
              DIRECTION: the conservative part is what the two directions share
              in sign, the dissipative part what flips with it.
  replay      the closed-loop check described above.
  map         the static push against angle from `drivetrain_bench.py map`:
              each held step's duty at rest, out and back, split into friction
              ((out - back) / 2) and the part that depends on angle alone
              ((out + back) / 2), with each one's strongest period. Taken at
              rest, so the creep's stick-slip sawtooth is not in it.

Reads traces/drivetrain/ (gitignored, Dropbox-synced). Writes nothing.
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from aow_sim.hw.bench_log import Capture  # noqa: E402

TRACES = ROOT / "traces" / "drivetrain"
BELT = 3.0
TS, W0 = 1.6, 106 * 2 * np.pi / 60           # XC430-W150, 12 V datasheet

CONTROLLER_CAPS = ("260913-001336_reversal", "260913-011952_reversal",
                   "260913-001455_chirp", "260913-014930_chirp",
                   "260913-013646_chirp", "260913-134255_chirp",
                   "260913-003338_step", "260913-000853_step")
REPLAY_CAPS = ("260913-001455_chirp", "260913-014930_chirp",
               "260913-013646_chirp", "260913-134255_chirp",
               "260913-011952_reversal", "260913-133208_creep")


def _session(prefix, *kinds):
    """Capture names under TRACES starting with `prefix`, of these kinds."""
    names = sorted(p.name for p in TRACES.iterdir()
                   if p.name.startswith(prefix) and p.name.rsplit("_", 1)[-1] in kinds)
    if not names:
        raise SystemExit(f"--session {prefix}: no {'/'.join(kinds)} capture in {TRACES}")
    return tuple(names)


def _caps(args, default, *kinds):
    return _session(args.session, *kinds) if getattr(args, "session", None) else default


def _load(name):
    return Capture.load(TRACES / name)


def _fill(x):
    g = np.isfinite(x)
    return np.interp(np.arange(len(x)), np.flatnonzero(g), x[g])


def _hold(raw):
    """Commands are nan where nothing was sent; the servo keeps the last."""
    out, last = np.empty_like(raw), 0.0
    for i, v in enumerate(raw):
        if np.isfinite(v):
            last = v
        out[i] = last
    return out


def _gains(cap, sid):
    snap = cap.meta["snapshot_before"][str(sid)]
    return np.array([(s["info"].get("kvp", snap["Velocity P Gain"]),
                      s["info"].get("kvi", snap["Velocity I Gain"]))
                     for s in cap.meta["segments"]])[cap.arrays["segment"]]


def _lp(x, dt, tau):
    if tau <= 0:
        return x.copy()
    from scipy.signal import lfilter
    a = dt / (tau + dt)
    return lfilter([a], [1, -(1 - a)], x)


# -- controller --------------------------------------------------------------

def controller(args):
    data = []
    for name in _caps(args, CONTROLLER_CAPS, "reversal", "chirp", "step"):
        cap = _load(name)
        kinds = np.array([s["info"].get("kind", "") for s in cap.meta["segments"]])
        for sid in cap.ids:
            t = _fill(cap.time_s(sid))
            data.append(dict(name=name, sid=sid, t=t, th=_fill(cap.position_rad(sid)),
                             duty=cap.value("Present PWM", sid),
                             cmd=_hold(cap.command(sid)), sg=cap.arrays["segment"],
                             gains=_gains(cap, sid),
                             kinds=kinds[cap.arrays["segment"]]))

    def fit(delay, tauf, report=False):
        sse, rows = 0.0, []
        for D in data:
            dt = np.median(np.diff(D["t"]))
            ref = np.roll(D["cmd"], delay)
            ref[:delay] = 0.0
            wf = _lp(np.gradient(D["th"], D["t"]), dt, tauf)
            refpos = np.cumsum(ref * np.gradient(D["t"]))
            for g in {tuple(x) for x in D["gains"]}:
                m = ((D["gains"] == g).all(axis=1)
                     & np.isin(D["kinds"], ["step", "return", "chirp", "square", "rest"])
                     & np.isfinite(D["duty"]) & (np.abs(D["duty"]) < 0.95))
                m[:50] = False
                if m.sum() < 300:
                    continue
                ks = np.unique(D["sg"][m])
                X = np.column_stack([ref - wf, refpos - D["th"]]
                                    + [(D["sg"] == k).astype(float) for k in ks])[m]
                y = D["duty"][m]
                c, *_ = np.linalg.lstsq(X, y, rcond=None)
                e = y - X @ c
                sse += float(e @ e)
                rows.append((D["name"], D["sid"], g, c[0], c[1],
                             1 - np.var(e) / np.var(y)))
        return sse, rows

    grid = sorted((fit(d, tf)[0], d, tf) for d in (0, 1, 2, 3)
                  for tf in (0.0, 0.004, 0.008, 0.012, 0.016, 0.024))
    print("controller: shared command delay x velocity filter, by total squared "
          "duty error (lower is better)")
    for sse, d, tf in grid[:5]:
        print(f"  delay {d} frames ({2 * d} ms)  filter {tf * 1e3:4.0f} ms  SSE {sse:8.1f}")
    _sse, rows = fit(grid[0][1], grid[0][2])
    print(f"\n  at the best ({2 * grid[0][1]} ms, {grid[0][2] * 1e3:.0f} ms), per capture,"
          " servo and gain setting:")
    print(f"  {'capture':<24s} {'id':>4s} {'P:I':>9s} {'Kp':>7s} {'Kp/unit':>9s} "
          f"{'Ki':>6s} {'Ki/unit':>9s} {'R2':>6s}")
    p_sens, i_sens = [], []
    for name, sid, g, kp, ki, r2 in rows:
        print(f"  {name:<24s} {sid:4d} {g[0]:4d}:{g[1]:<4d} {kp:7.4f} {kp / g[0]:9.2e} "
              f"{ki:6.3f} {ki / g[1]:9.2e} {r2:6.3f}")
        (i_sens if name.endswith("_step") else p_sens).append((kp / g[0], ki / g[1]))
    print("\n  Kp/unit, median over reversals + chirps (they load the P term): "
          f"{np.median([x[0] for x in p_sens]):.2e}   ROBOTIS conversion 3.68e-04")
    print("  Ki/unit, median over steps (they load the I term):              "
          f"{np.median([x[1] for x in i_sens]):.2e}")


# -- coast -------------------------------------------------------------------

def coast(args):
    from scipy.optimize import least_squares
    cap = _load(_caps(args, ("260913-001639_coast",), "coast")[-1])
    segs, sg = cap.meta["segments"], cap.arrays["segment"]

    def sim(p, duty, on, dt):
        J, tc, b = p
        w, x = np.empty(len(duty)), 0.0
        for i in range(len(duty)):
            drive = (TS * (duty[i] - x / W0) if on[i] else 0.0) - b * x
            if x == 0.0 and abs(drive) <= tc:
                x = 0.0
            else:
                s = np.sign(x) if x != 0.0 else np.sign(drive)
                xn = x + dt * (drive - tc * s) / J
                x = 0.0 if (x != 0.0 and np.sign(xn) != np.sign(x)) else xn
            w[i] = x
        return w

    print("coast: J [kg m2], Coulomb tau_c [N.m], viscous b [N.m s/rad], all at the "
          "SERVO shaft, per servo")
    for mode in ("common", "diff"):
        for sid in cap.ids:
            vel, duty, t = cap.diff_velocity(sid, 2), cap.value("Present PWM", sid), cap.time_s(sid)
            chunks = []
            for k, s in enumerate(segs):
                if s["info"].get("mode") != mode or s["info"]["kind"] != "spin":
                    continue
                idx = np.flatnonzero((sg == k) | (sg == k + 1))
                on = np.array([segs[sg[i]]["info"]["kind"] != "coast_off" for i in idx])
                chunks.append((np.nan_to_num(duty[idx]), on, vel[idx],
                               float(np.nanmedian(np.diff(t[idx])))))

            def resid(p):
                return np.concatenate([(sim(p, d, on, dt) - w)[np.isfinite(w)]
                                       for d, on, w, dt in chunks])

            r = least_squares(resid, [1e-3, 0.2, 0.005],
                              bounds=([1e-5, 0, 0], [1e-2, 1.0, 0.2]))
            rms = np.sqrt(np.mean(resid(r.x) ** 2))
            print(f"  {mode:<6s} id{sid}: J {r.x[0]:.2e}  tau_c {r.x[1]:.3f} "
                  f"({r.x[1] / TS:.3f} duty)  b {r.x[2]:.4f}  rms {rms:.2f} rad/s")


# -- detent ------------------------------------------------------------------

def detent(args):
    period, nb = 7.5, 15
    print(f"detent: diff push (duty per servo, in the push direction) folded onto "
          f"{period} deg of ring-vs-hub, {nb} bins")
    for name in _caps(args, ("260913-012157_creep", "260913-013834_creep"), "creep"):
        cap = _load(name)
        sa, sb = cap.meta["signs"]
        A, B = cap.ids
        segs, sg = cap.meta["segments"], cap.arrays["segment"]
        r = np.degrees(BELT * (sa * cap.position_rad(A) - sb * cap.position_rad(B)) / 2)
        e = (sa * cap.value("Present PWM", A) - sb * cap.value("Present PWM", B)) / 2
        for frac in (0.01, 0.03):
            prof = {}
            for sgn in (+1, -1):
                ks = [k for k, s in enumerate(segs) if s["info"].get("kind") == "creep"
                      and s["info"]["mode"] == "diff"
                      and np.isclose(s["info"]["frac"], sgn * frac)]
                m = np.isin(sg, ks) & cap.ok & np.isfinite(e)
                b = np.minimum((np.mod(r[m], period) / period * nb).astype(int), nb - 1)
                prof[sgn] = np.array([np.median(sgn * e[m][b == i]) for i in range(nb)])
            push = (prof[1] + prof[-1]) / 2       # opposes both: friction-like
            hill = (prof[1] - prof[-1]) / 2       # same sign both ways: conservative
            print(f"  {name} {frac:.0%}:")
            print("    friction-like  " + " ".join(f"{x:5.2f}" for x in push))
            print("    conservative   " + " ".join(f"{x:+5.2f}" for x in hill))
            print(f"    friction bump above its floor {push.max() - np.percentile(push, 20):.3f} "
                  f"duty; conservative swing p-p {hill.max() - hill.min():.3f}")


# -- replay ------------------------------------------------------------------

def _vel(pos, t, h=1):
    v = np.full(len(pos), np.nan)
    v[h:-h] = (pos[2 * h:] - pos[:-2 * h]) / (t[2 * h:] - t[:-2 * h])
    return v


def _chirp_gain(v, c, t, f0, f1, T):
    tt = t - t[0]
    fi = f0 * (f1 / f0) ** (tt / T)
    fc, g, i = [], [], 0
    while True:
        j = np.searchsorted(tt, tt[i] + 3 / fi[i])
        if j >= len(tt):
            break
        w = slice(i, j)
        X = np.column_stack([c[w], np.gradient(c[w], tt[w]) / (2 * np.pi * fi[i]),
                             np.ones(j - i)])
        ok = np.isfinite(v[w])
        if ok.sum() > 10:
            co, *_ = np.linalg.lstsq(X[ok], v[w][ok], rcond=None)
            g.append(np.hypot(co[0], co[1]))
            fc.append(fi[(i + j) // 2])
        i = j
    return np.array(fc), np.array(g)


def _simulate(cap, params_overlay):
    """Replay the capture's per-servo commands through the testbed model."""
    import mujoco

    from aow_sim import drivetrain_model as dm
    from aow_sim.build_model import build_model, load_params

    base = load_params()
    sa, sb = cap.meta["signs"]
    A, B = cap.ids
    t = _fill(cap.time_s(A))
    cmdA, cmdB = _hold(cap.command(A)), _hold(cap.command(B))
    segs, sg = cap.meta["segments"], cap.arrays["segment"]
    gains = _gains(cap, A)
    # Gains change per block: rebuild the loop whenever they do.
    out = {A: np.full(len(t), np.nan), B: np.full(len(t), np.nan)}
    blocks, start = [], 0
    for i in range(1, len(t) + 1):
        if i == len(t) or (gains[i] != gains[start]).any():
            blocks.append((start, i, tuple(int(x) for x in gains[start])))
            start = i
    params = dm.with_drivetrain(base, **params_overlay)
    m = build_model(dm.with_drivetrain(base, **params_overlay), variant="testbed")
    d = mujoco.MjData(m)
    ia, ib = m.actuator("drive_a").id, m.actuator("drive_b").id
    qa, qb = m.joint("input_a_spin").qposadr[0], m.joint("input_b_spin").qposadr[0]
    h = m.opt.timestep
    for s0, s1, g in blocks:
        p = dm.with_drivetrain(base, **{**params_overlay, "gains": g})
        sim = dm.DrivetrainSim(m, p)
        sim.reset(d)
        tn = t[s0]
        for k in range(s0, s1):
            while d.time < t[k] - t[0] - 1e-9 or tn is None:
                d.ctrl[ia] = BELT * sa * cmdA[k - 1 if k > s0 else k]
                d.ctrl[ib] = BELT * sb * cmdB[k - 1 if k > s0 else k]
                sim.pre_step(d)
                mujoco.mj_step(m, d)
                tn = 0
            out[A][k] = d.qpos[qa] / (BELT * sa)
            out[B][k] = d.qpos[qb] / (BELT * sb)
    del segs, params
    return t, out, {A: cmdA, B: cmdB}


def replay(args):
    overrides = {}
    if args.update_every:
        overrides["update_every"] = args.update_every
    if args.friction:
        overrides["servo"] = {"coulomb_friction": (args.friction * 2)[:2]}
    overlay = {"overrides": overrides}
    if args.no_slop:
        overlay["without"] = ("roller_slop",)
    print("replay: capture (R) against the MuJoCo testbed with the overlay (S), same "
          "metric code.\n  chirp: tracking plateau over 3-10 Hz and the peak above "
          "10 Hz, per servo.\n  square: p97-p3 swing over the commanded 2A.\n  creep: "
          "time under 25 % of command, p99 burst over command.")
    for name in args.caps:
        cap = _load(name)
        A, B = cap.ids
        t, sim, cmd = _simulate(cap, overlay)
        real = {sid: _fill(cap.position_rad(sid)) for sid in (A, B)}
        print(f"\n  {name}")
        for k, s in enumerate(cap.meta["segments"]):
            inf, sl = s["info"], cap.segment_slice(k)
            gain = f"P{inf.get('kvp', 100)}:I{inf.get('kvi', 1920)}"
            if inf.get("kind") == "chirp":
                cells = []
                for sid in (A, B):
                    for lab, pos in (("R", real[sid]), ("S", sim[sid])):
                        fc, g = _chirp_gain(_vel(pos, t)[sl], cmd[sid][sl], t[sl],
                                            inf["f0"], inf["f1"], s["seconds"])
                        hi = fc > 10
                        cells.append(f"{lab}{sid % 100} {np.nanmean(g[(fc > 3) & (fc < 10)]):.2f}"
                                     f" pk {np.nanmax(g[hi]):.2f}@{fc[hi][np.nanargmax(g[hi])]:4.1f}")
                print(f"    chirp {inf['mode']:<6s} {gain:<10s} " + " | ".join(cells))
            elif inf.get("kind") == "square":
                cells = []
                for sid in (A, B):
                    amp = np.nanmax(np.abs(cmd[sid][sl]))
                    if amp == 0:
                        continue
                    for lab, pos in (("R", real[sid]), ("S", sim[sid])):
                        v = _vel(pos, t)[sl][len(range(*sl.indices(len(t)))) // 4:]
                        cells.append(f"{lab}{sid % 100} "
                                     f"{(np.nanpercentile(v, 97) - np.nanpercentile(v, 3)) / (2 * amp):.2f}")
                print(f"    square {inf['mode']:<5s} {inf['freq']:4.0f} Hz {gain:<10s} "
                      + " ".join(cells))
            elif inf.get("kind") == "creep":
                sa, sb = cap.meta["signs"]
                cells = []
                for lab, P in (("R", real), ("S", sim)):
                    q = ((sa * P[A] - sb * P[B]) / 2 if inf["mode"] == "diff"
                         else (sa * P[A] + sb * P[B]) / 2)
                    v = _vel(q, t, 5)[sl] * np.sign(inf["v"])
                    v = v[len(v) // 6:]
                    cv = abs(inf["v"])
                    cells.append(f"{lab} stuck {100 * np.nanmean(v < 0.25 * cv):3.0f}% "
                                 f"burst {np.nanpercentile(v, 99) / cv:4.1f}x")
                print(f"    creep {inf['mode']} {inf['frac']:+.2f} {gain:<10s} " + " | ".join(cells))


def static_map(args):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from drivetrain_bench import map_profile
    for name in _caps(args, (), "map"):
        cap = _load(name)
        print(f"map {name}: duty at rest, + = the + direction of each mode")
        for mode in ("diff", "common"):
            prof = map_profile(cap, mode)
            if not len(prof[1]) or not len(prof[-1]):
                continue
            o_, b_ = (prof[d][np.argsort(prof[d][:, 0])] for d in (1, -1))
            lo, hi = max(o_[0, 0], b_[0, 0]), min(o_[-1, 0], b_[-1, 0])
            g = np.arange(lo, hi, 0.25)
            o, b = np.interp(g, *o_.T), np.interp(g, *b_.T)
            fr, an = (o - b) / 2, (o + b) / 2

            def period(x):
                x = x - x.mean()
                X = np.abs(np.fft.rfft(x * np.hanning(len(x))))
                c = np.fft.rfftfreq(len(x), 0.25)
                m = (c > 1 / max(hi - lo, 1.0)) & (c < 1 / 1.5)
                return 1 / c[m][np.argmax(X[m])] if m.any() else float("nan")
            print(f"  {mode:6s} {hi - lo:5.1f} deg covered | friction mean {fr.mean():.3f} "
                  f"(p-p {np.ptp(fr):.3f}, period {period(fr):4.1f} deg) | angle-only "
                  f"p-p {np.ptp(an):.3f} sd {an.std():.3f} (period {period(an):4.1f} deg)")
            step = max(1, len(g) // 30)
            print("    angle " + " ".join(f"{x:6.1f}" for x in g[::step]))
            print("    fric  " + " ".join(f"{x:6.3f}" for x in fr[::step]))
            print("    angle-only " + " ".join(f"{x:+6.3f}" for x in (an - an.mean())[::step]))


SECTIONS = {"controller": controller, "coast": coast, "detent": detent, "replay": replay,
            "map": static_map}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sections", nargs="*", choices=list(SECTIONS) + [[]],
                    default=["controller", "coast", "detent", "replay"])
    ap.add_argument("--caps", nargs="+", default=None,
                    help="captures for `replay` (default: the 09-13 set, or "
                         "--session's)")
    ap.add_argument("--session", metavar="PREFIX",
                    help="fit the captures whose names start with PREFIX "
                         "(e.g. a date, 261025) instead of the 09-13 bench set")
    ap.add_argument("--no-slop", action="store_true",
                    help="replay without roller slop")
    ap.add_argument("--update-every", type=int, default=0, metavar="N",
                    help="replay with the firmware loop every N physics steps")
    ap.add_argument("--friction", type=float, nargs="+", default=None,
                    metavar="N.M", help="coulomb_friction per servo, one value "
                    "or servo 101 then 102")
    args = ap.parse_args()
    sections = args.sections or ("controller", "coast", "detent", "replay")
    if args.caps is None and "replay" in sections:
        args.caps = list(_caps(args, REPLAY_CAPS, "chirp", "reversal", "creep"))
    for s in sections:
        SECTIONS[s](args)
        print()


if __name__ == "__main__":
    main()
