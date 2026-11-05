"""The righting linkage's friction: the unloaded preload from the bench
sweeps, and the pin-load coefficient from the bike's own lifts and lowers.

  unloaded   the 10-17 sweeps (stubs only, 15-120 deg/s, traces/righting_sweep):
             Present Current -> motor torque by servos.xc330_t181.current_law,
             less running gearbox friction, half the difference of the two
             directions per 10 deg bin over the 117 deg stroke, fitted
             base + q2 theta^2 -> righting.module.linkage_friction_nm / _q2.
  bike       the bike's lifts against its lowers at the same crank angle,
             crank moving > 20 deg/s the right way (traces/bike, 10-26 on;
             the two pre-swap runs left out): half their difference is the
             friction, half their sum the gravity, per 10 deg of crank.
  loaded MU [MU ...]
             the sim flying run_bike's own lie-down and get-up (its LIE_MAP,
             120 deg/s, PROP_CURRENT 0; 785 counts, 11.6 V) at each
             righting.module.linkage_load_mu, against the bike. FITTED TO
             THE WHOLE LIFT, crank -20 to 90: its time and charge (what the
             lift costs, at full duty, where the current reading is surest).
             The split is the check.

MEASURED 2026-11-05 (current_law, unloaded 0.0554 + 0.0137 theta^2):
  bike R 0.97 s 550 mA s, L 1.00 s 595 (131 lifts); sim mu 0.22 R 0.95 s
  558, L 0.95 s 570 (0.15: 539 / 548; 0.25: 590 / 606; 0.40: 1.4-1.6 s,
  930-1050, slower than any bike lift; 0.45 does not lift). The split at
  0.22 leaves the friction half ~30 mA short at crank 0-40, all of it in
  the lowers (sim ~120 mA, bike ~65); it closes at 0.40.

THE SPLIT (the check)'S ASSUMPTION: one current law in both directions, so a lift's
current is gravity plus friction and a lower's gravity less friction. The
lifts run at full duty; the lowers at ~10-13 % Present PWM, where the bike's
lower current barely moves with speed (51 -> 80 mA from 20 to 140 deg/s at
crank 0-40), which fits dry friction and not a duty-scaled reading.

    python analysis/mechanism/linkage_friction_fit.py unloaded
    python analysis/mechanism/linkage_friction_fit.py bike
    python analysis/mechanism/linkage_friction_fit.py loaded 0 0.3 0.38 0.45
"""
from __future__ import annotations

import contextlib
import copy
import glob
import io
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "analysis" / "moves"))

from aow_sim.build_model import load_params  # noqa: E402
from aow_sim.gearbox_friction import constants  # noqa: E402

BINS = np.arange(0, 101, 10)
SWEEPS = ["261017-140824_righting_sweep", "261017-141122_righting_sweep",
          "261017-141348_righting_sweep", "261017-141451_righting_sweep",
          "261017-141812_righting_sweep_free", "261017-141951_righting_sweep_free_full",
          "261017-145903_righting_sweep_speed30", "261017-145951_righting_sweep_speed30",
          "261017-150021_righting_sweep_speed60", "261017-150302_righting_sweep_speed120"]
STROKE_DEG = 117.0
PRE_SWAP = {"261024-163351", "261024-163836"}
CRANK_ID = 104
#: the whole lift's window, crank deg (lying rests at ~-25; props at ~98)
WINDOW = (-20.0, 90.0)


def unloaded(params=None):
    from aow_sim.hw.bench_log import Capture
    from aow_sim.righting_servo import GraphLaw
    p = params or load_params()
    fc = constants(p)
    r0, rc = fc["running_nm"], fc["running_frac"]
    law = GraphLaw(p["servos"]["xc330_t181"], r0, rc)

    def crank_tau(i):
        return math.copysign(max(law.torque(abs(i)) - r0, 0.0) / (1 + rc), i)
    rows = []
    for run in SWEEPS:
        cap = Capture.load(ROOT / "traces" / "righting_sweep" / run)
        stow = math.radians(cap.meta["stow_deg"])
        i = int(cap.meta["ids"][0])
        pos, cur = cap.position_rad(i), cap.value("Present Current", i)
        for s in cap.segments:
            if s["info"].get("kind") != "leg":
                continue
            sl = slice(s["start"], s["stop"])
            ok = cap.ok[sl]
            rows.append((s["info"]["dir"], pos[sl][ok] - stow, cur[sl][ok]))
    edges = np.radians(np.arange(-STROKE_DEG, STROKE_DEG + 1e-9, 10.0))
    th, f = [], []
    for a0, a1 in zip(edges[:-1], edges[1:]):
        med = {}
        for dd in (1, -1):
            A = np.concatenate([a for d, a, _ in rows if d == dd])
            C = np.concatenate([c for d, _, c in rows if d == dd])
            m = (A >= a0) & (A < a1)
            if m.sum() >= 5:
                med[dd] = crank_tau(float(np.median(C[m])))
        if len(med) == 2:
            th.append(0.5 * (a0 + a1))
            f.append(0.5 * (med[1] - med[-1]))
    th, f = np.array(th), np.array(f)
    X = np.c_[np.ones_like(th), th ** 2]
    (b, q2), *_ = np.linalg.lstsq(X, f, rcond=None)
    rms = float(np.sqrt(np.mean((X @ [b, q2] - f) ** 2)))
    print(f"unloaded linkage friction: {b:.4f} + {q2:.4f} theta^2 N m, rms {1000 * rms:.1f} mN m "
          f"({len(th)} bins over +-{STROKE_DEG:.0f} deg)")
    print("  by crank, mN m: " + " ".join(f"{math.degrees(t):+.0f}:{1000 * v:.0f}" for t, v in zip(th, f)))
    return b, q2


def _bike_segments(run_dir):
    z = np.load(os.path.join(run_dir, "capture.npz"), allow_pickle=True)
    st = z["bike_state"].astype(int)
    if len(st) < 10 or CRANK_ID not in list(z["ids"]):
        return []
    j = list(z["ids"]).index(CRANK_ID)
    roll = np.degrees(z["bike_roll"])
    crank = np.degrees(z["raw_present_position"][:, j] * 2 * np.pi / 4096 - np.pi)
    cur = z["raw_effort"][:, j].astype(np.int64)
    cur = np.where(cur > 32767, cur - 65536, cur)
    t = z["bike_t"]
    vel = np.gradient(crank) / np.maximum(np.gradient(t), 1e-3)
    out = []
    s3 = (st == 3).astype(int)                     # 3: the sit sequence
    for a in np.flatnonzero(np.diff(s3) == 1) + 1:
        b = a + int(np.argmax(st[a:] != 3)) if (st[a:] != 3).any() else len(st)
        if b - a < 50:
            continue
        if abs(roll[a]) > 50:
            side, kind = np.sign(roll[a]), "lift"
        elif b < len(st) and st[b] == 4 and abs(roll[b - 1]) > 60:   # 4: lying
            side, kind = np.sign(roll[b - 1]), "lower"
        else:
            continue
        u, i, w = side * crank[a:b], side * cur[a:b], side * vel[a:b]
        mv = (w > 20) if kind == "lift" else (w < -20)
        out.append((kind, int(side), u[mv], i[mv], w[mv], _charge(t[a:b], u, np.abs(cur[a:b]) / 1000.0)
                    if kind == "lift" else None))
    return out


def _charge(t, u, amps):
    """(time [s], charge [A s]) from crank WINDOW[0] to WINDOW[1], or None."""
    i0, i1 = np.flatnonzero(u >= WINDOW[0]), np.flatnonzero(u >= WINDOW[1])
    if not len(i0) or not len(i1) or i1[0] <= i0[0]:
        return None
    sl = slice(i0[0], i1[0] + 1)
    return float(t[sl][-1] - t[sl][0]), float(np.trapezoid(amps[sl], t[sl]))


def _binned(segs, kind, side, col=3):
    row = []
    for c in BINS:
        v = [np.median(x[col][np.abs(x[2] - c) < 5]) for x in segs
             if x[0] == kind and x[1] == side and (np.abs(x[2] - c) < 5).sum() > 2]
        row.append(float(np.median(v)) if v else np.nan)
    return np.array(row)


def bike(pattern="261[01]"):
    segs = []
    for d in sorted(glob.glob(str(ROOT / "traces" / "bike" / f"{pattern}*"))):
        if os.path.basename(d)[:13] in PRE_SWAP or not os.path.exists(d + "/capture.npz"):
            continue
        try:
            segs += _bike_segments(d)
        except (KeyError, ValueError, OSError, EOFError):
            continue
    res = {}
    for s in (1, -1):
        li, lo = _binned(segs, "lift", s), _binned(segs, "lower", s)
        res[s] = dict(lift=li, lower=lo, fric=(li - lo) / 2, grav=(li + lo) / 2,
                      lift_speed=np.abs(_binned(segs, "lift", s, 4)),
                      n=(sum(1 for x in segs if x[0] == "lift" and x[1] == s),
                         sum(1 for x in segs if x[0] == "lower" and x[1] == s)),
                      charge=np.array([x[5] for x in segs if x[0] == "lift" and x[1] == s
                                       and x[5] is not None]))
    return res


def _sim_split(params, policy="general_puffer_seed1_72M"):
    import swing_sit as H
    from aow_sim.control.swing_sit import SwingSit
    from aow_sim.hw import run_bike as RB
    res, ends = {}, []
    for side in (1.0, -1.0):
        with contextlib.redirect_stdout(io.StringIO()):
            sim = H.Sim(params, policy)
        sim.srv.set_goal_current(785)
        sim.gc = 785
        sim.srv.set_supply(11.6 / 12)
        travel = np.degrees(float(sim.m.actuator_ctrlrange[sim.aid, 1]))
        seq = SwingSit(np.radians(travel), hand_off=False, rest_deg=RB.LIE_REST_DEG,
                       lift_dps=RB.SIT_DPS, lower_dps=RB.SIT_DPS, lower_map=RB.LIE_MAP,
                       lower_roll_dps=RB.LIE_ROLL_DPS, lower_roll_acc=RB.LIE_ROLL_ACC,
                       retry_s=None, prop_current=RB.PROP_CURRENT,
                       lift_to_deg=tuple(travel if v == "stroke" else v for v in
                                         (RB.LIFT_TO_DEG["left"], RB.LIFT_TO_DEG["right"])))
        H.hold(sim, 0.5)
        d, cd = sim.d, sim.m.joint("swing_crank_joint").dofadr[0]
        rec, up = [], []
        state = {"on": True}

        def run(until, T, trace=None):
            t0 = d.time
            while d.time - t0 < T:
                roll, rate, crank = sim.state()
                cmd = seq.step(d.time, roll, rate, crank)
                state["on"] = sim.step(cmd, state["on"])
                if trace is not None:
                    trace.append((d.time, side * math.degrees(crank), abs(sim.srv.current)))
                k = seq.phase if seq.phase in ("lift", "lower") else None
                w = side * math.degrees(d.qvel[cd])
                if k and ((k == "lift" and w > 20) or (k == "lower" and w < -20)):
                    rec.append((k, int(side), side * math.degrees(crank), side * sim.srv.current * 1000, w))
                if seq.phase == until:
                    return True
            return False
        seq.lie_down(d.time, side=side)
        ok = run("lying", 12.0)
        H.hold(sim, 1.0, seq.step(d.time, *sim.state()))
        seq.get_up(d.time, sim.state()[0])
        run("propped", 5.0, up)
        up = np.array(up)
        r_, _, c_ = sim.state()
        ends.append(f"{'R' if side > 0 else 'L'} lay {ok}, {seq.phase} crank {math.degrees(c_):+.1f} "
                    f"roll {math.degrees(r_):+.1f}")
        s = int(side)

        def binned(kind, col):
            u = np.array([r[2] for r in rec if r[0] == kind])
            v = np.array([r[col] for r in rec if r[0] == kind])
            return np.array([np.median(v[np.abs(u - c) < 5]) if (np.abs(u - c) < 5).sum() > 2
                             else np.nan for c in BINS])
        li, lo, sp = binned("lift", 3), binned("lower", 3), binned("lift", 4)
        res[s] = dict(lift=li, lower=lo, fric=(li - lo) / 2, grav=(li + lo) / 2, lift_speed=sp,
                      charge=_charge(up[:, 0], up[:, 1], up[:, 2]) if len(up) else None)
    return res, ends


def _row(name, v, fmt="{:5.0f}"):
    return f"  {name:16s}" + " ".join(fmt.format(x) if np.isfinite(x) else "    -" for x in v)


def loaded(mus, bk=None):
    bk = bk or bike()
    base = load_params()
    print("crank             " + " ".join(f"{c:5d}" for c in BINS))
    summary = []
    for mu in mus:
        p = copy.deepcopy(base)
        p["righting"]["module"]["linkage_load_mu"] = float(mu)
        sm, ends = _sim_split(p)
        print(f"\nmu {mu}: " + "; ".join(ends))
        gaps = []
        for s, nm in ((1, "R"), (-1, "L")):
            b, m = bk[s], sm[s]
            print(_row(f"{nm} lift mA", m["lift"]) + "   bike " + " ".join(f"{x:.0f}" for x in b["lift"][:5]))
            print(_row(f"{nm} lower mA", m["lower"]) + "   bike " + " ".join(f"{x:.0f}" for x in b["lower"][:5]))
            print(_row(f"{nm} friction half", m["fric"]))
            print(_row(f"{nm}  bike - sim", b["fric"] - m["fric"]))
            print(_row(f"{nm} gravity half", m["grav"]) + "   bike " + " ".join(f"{x:.0f}" for x in b["grav"][:5]))
            print(_row(f"{nm} lift deg/s", m["lift_speed"]) + "   bike " + " ".join(f"{x:.0f}" for x in b["lift_speed"][:5]))
            gaps.append((b["fric"] - m["fric"])[:5])
        g = np.concatenate(gaps)
        ch = [sm[s]["charge"] for s in (1, -1)]
        summary.append((mu, float(np.nanmean(g)), float(np.nanmin(g)), float(np.nanmax(g)), ch))
    print(f"\nTHE FIT -- the whole lift, crank {WINDOW[0]:+.0f} to {WINDOW[1]:+.0f} deg: time and charge "
          f"(integral of |Present Current|), R / L")
    for s, nm in ((1, "R"), (-1, "L")):
        c = bk[s]["charge"]
        print(f"  bike {nm}: {len(c)} lifts, time {np.median(c[:, 0]):.2f} s, charge {1000 * np.median(c[:, 1]):.0f} mA s "
              f"({1000 * np.percentile(c[:, 1], 25):.0f}-{1000 * np.percentile(c[:, 1], 75):.0f})")
    for mu, mean, lo, hi, ch in summary:
        txt = " / ".join("did not reach" if c is None else f"{c[0]:.2f} s {1000 * c[1]:.0f} mA s" for c in ch)
        print(f"  sim mu {mu:5.2f}: {txt}")
    print("\nTHE CHECK -- crank 0-40, both sides: bike friction half minus sim's, mA (the lowers run at "
          "~10 % PWM, the least sure reading here)")
    for mu, mean, lo, hi, _ in summary:
        print(f"  mu {mu:5.2f}: mean {mean:+6.1f}  range {lo:+5.0f}..{hi:+4.0f}")


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("unloaded", "bike", "loaded"):
        print(__doc__)
        return 2
    if sys.argv[1] == "unloaded":
        unloaded()
    elif sys.argv[1] == "bike":
        bk = bike()
        print("crank             " + " ".join(f"{c:5d}" for c in BINS))
        for s, nm in ((1, "R"), (-1, "L")):
            b = bk[s]
            print(f" {nm}: {b['n'][0]} lifts, {b['n'][1]} lowers")
            for k in ("lift", "lower", "fric", "grav", "lift_speed"):
                print(_row(f"{nm} {k}", b[k]))
    else:
        loaded([float(x) for x in sys.argv[2:]] or [0.0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
