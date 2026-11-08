"""The get-up's lift over time, bike against sim, before and with the ballast.

Every bike lift from lying (run_bike's sit sequence starting at |roll| > 50,
traces/bike 10-26 on, the pre-swap runs left out) is cut at the moment its
crank passes -20 deg (the whole-lift window's start in
linkage_friction_fit.py) and drawn on one time axis with the sim flying the
same sequence (run_bike's SwingSit settings, 785 counts, 11.6 V, mu 0.22)
on the plant before the ballast and with it. Signs are folded onto the
lifting side, so both sides read lying ~+74 deg and propped ~+12.

  plots/lift_traces.png        the lifts that propped: roll, crank, crank speed,
                               Present Current, Present PWM / the sim's duty; R and L
  plots/lift_failures.png      the same for every lift that did not prop
  plots/lift_charge_temp.png   each lift's charge (crank -20..90) against the
                               crank servo's Present Temperature at its start
  printed                      the prop rate by side against the case temperature,
                               the I^2 t of the 60 s before each lift (the
                               winding's recent heating) and the supply

Present PWM in mode 5 is not the duty (status.md: ~2x the model at stall);
it is drawn beside the sim's duty for its shape only.

BALLAST_RUNS names the runs flown with the 72 g ballast, by name, because
the Pi's clock names some runs for the wrong day.

    python analysis/mechanism/lift_traces.py
"""
from __future__ import annotations

import contextlib
import copy
import glob
import io
import math
import os
import sys
import warnings
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "analysis" / "moves"))
sys.path.insert(0, str(ROOT / "analysis" / "mechanism"))

import linkage_friction_fit as F  # noqa: E402
from aow_sim.build_model import load_params  # noqa: E402

BALLAST_RUNS = {"261105-143426"}          # recorded 11-07, the ballast's first
PLOTS = ROOT / "analysis" / "mechanism" / "plots"
T = np.arange(-0.3, 2.6, 0.01)            # s from the crank passing -20 deg
STATES = ["engaged", "cut", "standby", "sit", "lying", "propped"]


def bike_lifts():
    """[{run, ballast, side, ok, temp, charge, t, roll, crank, speed, cur, pwm}]"""
    out = []
    for d in sorted(glob.glob(str(ROOT / "traces" / "bike" / "261[01]*"))):
        name = os.path.basename(d)[:13]
        if name in F.PRE_SWAP or not os.path.exists(d + "/capture.npz"):
            continue
        try:
            z = np.load(d + "/capture.npz", allow_pickle=True)
            st = z["bike_state"].astype(int)
            ids = list(z["ids"])
            j = ids.index(F.CRANK_ID)
        except (KeyError, ValueError, OSError, EOFError):
            continue
        if len(st) < 10:
            continue
        s16 = lambda a: np.where(a > 32767, a - 65536, a).astype(float)
        t = z["bike_t"]
        roll = np.degrees(z["bike_roll"])
        crank = np.degrees(z["raw_present_position"][:, j] * 2 * np.pi / 4096 - np.pi)
        cur = s16(z["raw_effort"][:, j].astype(np.int64))
        pwm = s16(z["raw_present_pwm"][:, j].astype(np.int64)) / 885.0
        temp = z["raw_present_temperature"][:, j]
        volt = z["raw_present_input_voltage"][:, j] * 0.1
        # I^2 t over the 60 s before each lift [A^2 s]: the winding's recent
        # heating, which the case's Present Temperature lags.
        i2t = np.concatenate(([0.0], np.cumsum((cur[1:] / 1000.0) ** 2 * np.diff(t))))
        speed = np.gradient(crank) / np.maximum(np.gradient(t), 1e-3)
        s3 = (st == 3).astype(int)
        for a in np.flatnonzero(np.diff(s3) == 1) + 1:
            b = a + int(np.argmax(st[a:] != 3)) if (st[a:] != 3).any() else len(st)
            if b - a < 50 or abs(roll[a]) <= 50:
                continue
            side = float(np.sign(roll[a]))
            u = side * crank[a:b]
            k = np.flatnonzero(u >= F.WINDOW[0])
            if not len(k):
                continue
            t0 = t[a + k[0]]
            sl = slice(max(a - 50, 0), min(b + 100, len(t)))
            tt = t[sl] - t0
            ch = F._charge(t[a:b], u, np.abs(cur[a:b]) / 1000.0)
            out.append(dict(
                run=name, ballast=name in BALLAST_RUNS, side=int(side),
                ok=b < len(st) and STATES[st[b]] == "propped", temp=int(temp[a]),
                volt=float(volt[a]), i2t60=float(i2t[a] - np.interp(t[a] - 60.0, t, i2t)),
                charge=ch, t0=float(t0),
                **{key: np.interp(T, tt, side * arr[sl], left=np.nan, right=np.nan)
                   for key, arr in (("roll", roll), ("crank", crank), ("speed", speed),
                                    ("cur", cur), ("pwm", pwm))}))
    return out


def sim_lifts(params, policy="general_puffer_seed1_72M"):
    """{side: dict of T-sampled series + charge} flying run_bike's get-up."""
    import swing_sit as H
    from aow_sim.control.swing_sit import SwingSit
    from aow_sim.hw import run_bike as RB
    res = {}
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
        on = [True]

        def run(until, secs, rec=None):
            t0 = d.time
            while d.time - t0 < secs:
                roll, rate, crank = sim.state()
                cmd = seq.step(d.time, roll, rate, crank)
                on[0] = sim.step(cmd, on[0])
                if rec is not None:
                    rec.append((d.time, side * math.degrees(roll), side * math.degrees(crank),
                                side * math.degrees(d.qvel[cd]), side * sim.srv.current * 1000,
                                side * sim.srv.duty))
                if seq.phase == until and (rec is None or d.time - t0 > 2.6):
                    return True
            return False
        seq.lie_down(d.time, side=side)
        run("lying", 12.0)
        H.hold(sim, 1.0, seq.step(d.time, *sim.state()))
        seq.get_up(d.time, sim.state()[0])
        rec = []
        run("propped", 4.0, rec)
        r = np.array(rec)
        k = np.flatnonzero(r[:, 2] >= F.WINDOW[0])[0]
        tt = r[:, 0] - r[k, 0]
        res[int(side)] = dict(
            charge=F._charge(r[:, 0], r[:, 2], np.abs(r[:, 4]) / 1000.0),
            **{key: np.interp(T, tt, r[:, i], left=np.nan, right=np.nan)
               for i, key in enumerate(("roll", "crank", "speed", "cur", "pwm"), start=1)})
    return res


def unballasted(p):
    """`p` as it was before 11-07: no ballast, the old upper case."""
    q = copy.deepcopy(p)
    q["righting"]["module"]["ballast"]["mass"] = 0.0
    q["righting"]["module"]["fixed"]["upper_case"].update(
        mass=0.0078 * float(p["bike"]["printed_mass_scale"]),
        at=[0.0, 19.0, -51.2], size=[0.0223, 0.0281, 0.051])
    return q


ROWS = (("roll", "roll, deg"), ("crank", "crank, deg"), ("speed", "crank speed, deg/s"),
        ("cur", "Present Current, mA"), ("pwm", "Present PWM (bike) / duty (sim)"))
GREY, ORANGE, BLUE = "#8a8a8a", "#d9711c", "#2b6cb0"


def _legend(fig, axes):
    """One legend for the figure, in a row above the panels, from every
    panel's labels (a label drawn in one panel only still shows)."""
    seen = {}
    for ax in axes.flat:
        for h, lab in zip(*ax.get_legend_handles_labels()):
            seen.setdefault(lab, h)
    order = sorted(seen, key=lambda lab: (not lab.startswith("bike"), "with" in lab, lab))
    fig.legend([seen[k] for k in order], order, loc="upper center", ncol=2,
               fontsize=9.5, frameon=False, bbox_to_anchor=(0.5, 1.0))


def plot_traces(bike, sims, out, ok):
    """`ok` True: the lifts that propped, before as median + IQR and with
    the ballast one by one. False: every lift that did not prop, one by one
    (before grey, with the ballast orange), the sim's (propping) lifts for
    reference."""
    fig, axes = plt.subplots(len(ROWS), 2, figsize=(12, 14), sharex=True)
    for col, (side, nm) in enumerate(((1, "right"), (-1, "left"))):
        pre = [x for x in bike if x["side"] == side and not x["ballast"] and x["ok"] == ok]
        new = sorted((x for x in bike if x["side"] == side and x["ballast"] and x["ok"] == ok),
                     key=lambda x: x["t0"])
        for row, (key, label) in enumerate(ROWS):
            ax = axes[row, col]
            if ok:
                A = np.array([x[key] for x in pre])
                with np.errstate(all="ignore"), warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    med = np.nanmedian(A, 0)
                    lo, hi = np.nanpercentile(A, 25, 0), np.nanpercentile(A, 75, 0)
                ax.fill_between(T, lo, hi, color=GREY, alpha=0.25, lw=0)
                ax.plot(T, med, color=GREY, lw=2, label="bike before the ballast: median, band 25-75 %")
            else:
                for i, x in enumerate(pre):
                    ax.plot(T, x[key], color=GREY, alpha=0.35, lw=0.8,
                            label="bike before the ballast, one line a lift" if i == 0 else None)
            for i, x in enumerate(new):
                shade = 0.35 + 0.65 * (i / max(len(new) - 1, 1))
                ax.plot(T, x[key], color=ORANGE, alpha=shade, lw=1.3,
                        label="bike with the ballast, light to dark in run order" if i == 0 else None)
            ax.plot(T, sims["before"][side][key], color=BLUE, lw=1.6, ls=":",
                    label="sim before the ballast" + ("" if ok else " (propped, for reference)"))
            ax.plot(T, sims["ballast"][side][key], color=BLUE, lw=1.8,
                    label="sim with the ballast" + ("" if ok else " (propped, for reference)"))
            ax.grid(color="#dddddd", lw=0.5)
            ax.set_ylabel(label)
            if row == 0:
                what = "propped" if ok else "did not prop"
                ax.set_title(f"lifting from lying on the {nm}: {what}\n"
                             f"{len(pre)} before the ballast, {len(new)} with it", fontsize=10.5)
        axes[-1, col].set_xlabel("s from the crank passing -20 deg")
    _legend(fig, axes)
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    fig.savefig(out, dpi=110)
    plt.close(fig)


def plot_charge_temp(bike, sims, out):
    fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharey=True)
    for ax, (side, nm) in zip(axes, ((1, "right"), (-1, "left"))):
        for ballast, colour, lab in ((False, GREY, "bike before"), (True, ORANGE, "bike with ballast")):
            xs = [x for x in bike if x["side"] == side and x["ballast"] == ballast
                  and x["charge"] is not None]
            ax.scatter([x["temp"] for x in xs], [1000 * x["charge"][1] for x in xs], s=22,
                       color=colour, alpha=0.8 if ballast else 0.45, label=f"{lab} ({len(xs)})",
                       edgecolor="white", lw=0.5)
            nf = sum(1 for x in bike if x["side"] == side and x["ballast"] == ballast
                     and x["charge"] is None)
            if nf:
                ax.text(0.02, 0.95 - 0.07 * ballast, f"{lab}: {nf} never reached crank 90 (not drawn)",
                        transform=ax.transAxes, fontsize=9, color="#444444")
        for k, ls in (("before", ":"), ("ballast", "-")):
            c = sims[k][side]["charge"]
            if c is not None:
                ax.axhline(1000 * c[1], color=BLUE, ls=ls, lw=1.4, label=f"sim {k}")
        ax.set_title(f"lifting from lying on the {nm}")
        ax.set_xlabel("crank servo Present Temperature at the lift's start, C")
        ax.grid(color="#dddddd", lw=0.5)
    axes[0].set_ylabel("charge, crank -20..90, mA s")
    _legend(fig, axes)
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    fig.savefig(out, dpi=110)
    plt.close(fig)


def heat_tables(bike):
    """Prop rate before the ballast by side against the case temperature,
    the I^2 t of the 60 s before, and the supply; then the ballast run in
    order."""
    pre = [x for x in bike if not x["ballast"]]

    def rate(xs):
        return (f"{sum(x['ok'] for x in xs):3d}/{len(xs):3d} ({100 * np.mean([x['ok'] for x in xs]):3.0f} %)"
                if xs else "      -")
    q = np.percentile([x["i2t60"] for x in pre], [100 / 3, 200 / 3])
    print("\nPROPPED / LIFTS before the ballast")
    for side, nm in ((1, "R"), (-1, "L")):
        xs = [x for x in pre if x["side"] == side]
        print(f"  {nm} case temp  <29 C {rate([x for x in xs if x['temp'] < 29])}"
              f"   29-32 {rate([x for x in xs if 29 <= x['temp'] <= 32])}"
              f"   >32 {rate([x for x in xs if x['temp'] > 32])}")
        print(f"  {nm} I2t, 60 s  <{q[0]:.2f} A2s {rate([x for x in xs if x['i2t60'] < q[0]])}"
              f"   mid {rate([x for x in xs if q[0] <= x['i2t60'] < q[1]])}"
              f"   >={q[1]:.2f} {rate([x for x in xs if x['i2t60'] >= q[1]])}")
        print(f"  {nm} supply     <11.7 V {rate([x for x in xs if x['volt'] < 11.7])}"
              f"   >=11.7 {rate([x for x in xs if x['volt'] >= 11.7])}")
    print("\nTHE BALLAST RUN, in order")
    for x in sorted((x for x in bike if x["ballast"]), key=lambda x: x["t0"]):
        print(f"  {'R' if x['side'] > 0 else 'L'} {x['t0']:6.1f} s  {x['temp']} C  I2t60 {x['i2t60']:4.2f} A2s  "
              f"{x['volt']:.1f} V  " + ("propped " if x["ok"] else "FAILED  ")
              + ("" if x["charge"] is None else f"{1000 * x['charge'][1]:.0f} mA s"))


def main():
    PLOTS.mkdir(exist_ok=True)
    bike = bike_lifts()
    p = load_params()
    sims = {"before": sim_lifts(unballasted(p)), "ballast": sim_lifts(p)}
    plot_traces(bike, sims, PLOTS / "lift_traces.png", ok=True)
    plot_traces(bike, sims, PLOTS / "lift_failures.png", ok=False)
    plot_charge_temp(bike, sims, PLOTS / "lift_charge_temp.png")
    for side, nm in ((1, "R"), (-1, "L")):
        for ballast in (False, True):
            xs = [x for x in bike if x["side"] == side and x["ballast"] == ballast]
            ok = [x for x in xs if x["charge"] is not None]
            print(f"{nm} {'ballast' if ballast else 'before '}: {len(xs)} lifts, {sum(x['ok'] for x in xs)} propped, "
                  f"{len(ok)} reached 90; temp {min(x['temp'] for x in xs)}-{max(x['temp'] for x in xs)} C")
        for k in sims:
            c = sims[k][side]["charge"]
            print(f"  sim {k}: " + ("did not reach 90" if c is None else f"{c[0]:.2f} s {1000 * c[1]:.0f} mA s"))
    heat_tables(bike)
    print(f"wrote lift_traces.png, lift_failures.png, lift_charge_temp.png in {PLOTS}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
