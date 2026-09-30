"""Steer friction map: the torque it takes to turn the built steering, by angle.

The first print of the front steer (docs/plans/steering-design.md, "First
print") turns in a printed Φ14 bushing with one tight spot where the two
parts' seams meet. This measures that: the steer XC330 drives the headset
round at a slow constant speed in current-based position mode (5), and the
current it draws, both directions, is the friction at every angle.

    python analysis/steer_friction.py --port /dev/ttyUSB0 --dry-run
    python analysis/steer_friction.py --port /dev/ttyUSB0 --note "upside down, no load"
    python analysis/steer_friction.py summary traces/steer_friction/<capture dir> [--plot]

Its companion is `servo_breakaway.py --ids 103`, unchanged: the current at
which the steer STARTS to move, against the bare XC330's 20-25 mA of
2026-09-25. This one is the RUNNING friction and where round the turn it sits.

THE SCHEDULE. 1 s torque off, 2 s holding (the resting current reading).
Then per `--speeds` [deg/s] and per rep: `--turns` turns +, then the same -,
so the net winding stays zero. The goal is a ramp from wherever the output
is when the segment starts; `--cap-ma` caps the current, so a jam stalls at
the cap instead of loading the printed pins with the whole servo.
`--max-lag-deg` aborts (torque off) if the output falls that far behind the
ramp -- a jam, not friction.

READING IT. Per sample the motor torque is k * sign(I) * max(|I| - I0, 0),
with k and I0 the X330 fixture's mode-5 fit (servos.xc330_t181
current_torque_gain / current_deadband). In 10 deg bins of STEER angle
(servo angle - control.onboard.steer_zero_deg; steering.gear_ratio is 1):

    friction(θ) = (τ+(θ) - τ-(θ)) / 2      Coulomb, both directions
    bias(θ)     = (τ+(θ) + τ-(θ)) / 2      anything that pushes one way:
                                           gravity on an off-axis mass, a
                                           constant offset in the reading

The half-difference cancels a constant current offset and the bias, so the
friction column needs no calibration beyond k and I0. It is the friction AT
THE OUTPUT: the gearbox's own plus the headset's. The gearbox part is
`gearbox_friction`'s running line, 0.15 x load + 5.7 mN m, so the headset
alone is estimated as (friction - 5.7) / 1.15 -- MODEL-DERIVED, from a
different unit's gearbox. Running the same schedule on a bare XC330 (the
other one, horn only) measures that baseline instead of assuming it.

Resolution: Present Current is 1 mA a count, 0.83 mN m. k was fitted over
8-59 mN m (35-129 mA); outside that it is an extrapolation.

THERMAL. <= 150 mA by default and moving the whole time; temperature is read
every frame and the run stops at `--max-temp`.
"""

from __future__ import annotations

import argparse
import math
import os
import platform
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))     # this checkout's source, even from a worktree
sys.path.insert(0, str(Path(__file__).resolve().parent))

import yaml  # noqa: E402

from aow_sim.hw.bench_log import (Capture, Segment, git_state,  # noqa: E402
                                  new_capture_dir, record)
from aow_sim.hw.dynamixel import (MODE_CURRENT_POSITION,  # noqa: E402
                                  DynamixelBus, IndirectMap,
                                  describe_hardware_error)
from aow_sim.params import DEFAULT_PARAMS  # noqa: E402
from servo_breakaway import (_FAILED, MODEL, READ_BLOCK, WATCHDOG_MS,  # noqa: E402
                             Tee, _try, righting_gains, set_current, torque)

TEST = "steer_friction"
BIN_DEG = 10.0
SETTLE_S = 0.5          # dropped from each ramp: the lag building up
RUNNING_NM, RUNNING_FRAC = 0.0057, 0.15     # fallbacks; read from params below


def model() -> dict:
    """k, I0, the gearbox's running line and the steer zero, from bike_params."""
    raw = yaml.safe_load(Path(DEFAULT_PARAMS).read_text())
    sv = raw["servos"]["xc330_t181"]
    v = lambda x: x["value"] if isinstance(x, dict) else x    # noqa: E731
    return {"k": v(sv["current_torque_gain"]), "i0": v(sv["current_deadband"]),
            "run_nm": v(sv.get("friction_running_nm", RUNNING_NM)),
            "run_frac": v(sv.get("friction_running_fraction", RUNNING_FRAC)),
            "zero_deg": float(raw["control"]["onboard"]["steer_zero_deg"])}


def build_plan(args, base: dict) -> list:
    hold_here = lambda t, s: dict(base)     # noqa: E731
    segs = [Segment("offset: torque OFF", 1.0, None, enter=torque(False),
                    info={"kind": "offset", "torque": False}),
            Segment(f"offset: torque on, holding, {args.cap_ma:g} mA", 2.0, hold_here,
                    enter=lambda bus: (set_current(args.cap_ma)(bus), bus.torque(True)),
                    info={"kind": "offset", "torque": True})]
    for rep in range(args.reps):
        for speed in args.speeds:
            w = math.radians(speed)
            secs = SETTLE_S + args.turns * 360.0 / speed
            for sgn in (+1, -1):
                anchor = {}

                def ramp(t, s, sgn=sgn, w=w, anchor=anchor):
                    if not anchor and s is not None:
                        anchor.update({i: s[i]["Present Position"] for i in s})
                    return {i: anchor[i] + sgn * w * t for i in anchor} if anchor else None
                segs.append(Segment(f"ramp {'+' if sgn > 0 else '-'}{speed:g} deg/s rep {rep}",
                                    secs, ramp, enter=set_current(args.cap_ma),
                                    info={"kind": "ramp", "dir": sgn, "speed_deg_s": speed,
                                          "rep": rep}))

                def stop_here(t, s):
                    return {i: s[i]["Present Position"] for i in s} if s is not None else None
                segs.append(Segment("stop where it is", 0.5, stop_here, info={"kind": "stop"}))
    return segs


# --------------------------------------------------------------------------
# analysis
# --------------------------------------------------------------------------

def _rows(cap: Capture, k: int) -> slice:
    s = cap.segments[k]
    return slice(s["start"], s["stop"])


def friction_table(cap: Capture, dxl_id: int, m: dict) -> dict:
    """{speed: {"edges", "fric", "bias", "n", "w_mean", "w_sd"}} in N m / rad/s."""
    nb = int(round(360 / BIN_DEG))
    acc = {}
    for k, s in enumerate(cap.segments):
        info = s["info"]
        if info.get("kind") != "ramp":
            continue
        sl = _rows(cap, k)
        ok = cap.ok[sl]
        t = cap.time_s(dxl_id)[sl][ok]
        pos = cap.position_rad(dxl_id)[sl][ok]
        cur = cap.value("Present Current", dxl_id)[sl][ok]
        keep = t - t[0] >= SETTLE_S if len(t) else t
        t, pos, cur = t[keep], pos[keep], cur[keep]
        if len(t) < 10:
            continue
        tau = m["k"] * np.sign(cur) * np.clip(np.abs(cur) - m["i0"], 0, None)
        steer = (np.degrees(pos) - m["zero_deg"] + 180.0) % 360.0 - 180.0
        b = np.clip(((steer + 180.0) // BIN_DEG).astype(int), 0, nb - 1)
        a = acc.setdefault(info["speed_deg_s"], {+1: [np.zeros(nb), np.zeros(nb)],
                                                 -1: [np.zeros(nb), np.zeros(nb)], "w": []})
        np.add.at(a[info["dir"]][0], b, tau)
        np.add.at(a[info["dir"]][1], b, 1)
        w = np.diff(pos) / np.maximum(np.diff(t), 1e-6) * info["dir"]
        a["w"].append(w)
    out = {}
    for speed, a in sorted(acc.items()):
        with np.errstate(invalid="ignore", divide="ignore"):
            tp = a[+1][0] / a[+1][1]
            tm = a[-1][0] / a[-1][1]
        w = np.concatenate(a["w"])
        fric = (tp - tm) / 2
        # + current turns the output +, as far as anyone has checked; if not,
        # every bin comes out negative, and the magnitude is still the answer
        fric *= np.sign(np.nanmean(fric)) or 1.0
        out[speed] = {"edges": -180.0 + BIN_DEG * np.arange(nb), "fric": fric,
                      "bias": (tp + tm) / 2, "n": np.minimum(a[+1][1], a[-1][1]),
                      "w_mean": float(np.mean(w)), "w_sd": float(np.std(w))}
    return out


def summarize(cap: Capture, plot: bool = False, out: Path | None = None) -> None:
    m = model()
    ids = [int(i) for i in cap.meta["ids"]]
    print(f"\nk {m['k']} N m/A, I0 {m['i0'] * 1000:.1f} mA, steer zero {m['zero_deg']:g} deg; "
          f"headset = (friction - {m['run_nm'] * 1000:.1f} mN m) / {1 + m['run_frac']:.2f} "
          f"(model, a different unit's gearbox)")
    for k, s in enumerate(cap.segments):
        if s["info"].get("kind") == "offset":
            for i in ids:
                cur = cap.value("Present Current", i)[_rows(cap, k)]
                cur = cur[np.isfinite(cur)] * 1000
                if len(cur):
                    print(f"  id {i} {s['label']:38s} mean {cur.mean():+5.1f} mA  sd {cur.std():.1f}")
    tables = {i: friction_table(cap, i, m) for i in ids}
    for i, tab in tables.items():
        for speed, r in tab.items():
            good = r["n"] > 0
            f = r["fric"][good] * 1000
            e = r["edges"][good]
            hs = (f - m["run_nm"] * 1000) / (1 + m["run_frac"])
            slip = r["w_sd"] / max(abs(r["w_mean"]), 1e-9)
            print(f"\nid {i}, {speed:g} deg/s (ran {math.degrees(r['w_mean']):.1f} deg/s, "
                  f"sd {math.degrees(r['w_sd']):.1f}{'  STICK-SLIP?' if slip > 0.5 else ''}):")
            print(f"  friction at the output  mean {f.mean():5.1f}  min {f.min():5.1f} "
                  f"@{e[f.argmin()] + BIN_DEG / 2:+4.0f}  max {f.max():5.1f} "
                  f"@{e[f.argmax()] + BIN_DEG / 2:+4.0f} deg steer   [mN m]")
            print(f"  headset alone (model)   mean {hs.mean():5.1f}  max {hs.max():5.1f}")
            print(f"  bias (one-way push)     {np.nanmin(r['bias'][good]) * 1000:+5.1f} .. "
                  f"{np.nanmax(r['bias'][good]) * 1000:+5.1f}")
            print("  by angle (deg: mN m)   " + "  ".join(
                f"{a + BIN_DEG / 2:+.0f}:{v:.1f}" for a, v in zip(e, f)))
    if plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 4))
        for i, tab in tables.items():
            for speed, r in tab.items():
                c = r["edges"] + BIN_DEG / 2
                ax.plot(c, r["fric"] * 1000, marker="o", ms=3, label=f"id {i}, {speed:g} deg/s")
        ax.set_xlabel("steer angle [deg] (0 = straight ahead)")
        ax.set_ylabel("friction at the output [mN m]")
        ax.set_xlim(-180, 180)
        ax.set_ylim(bottom=0)
        ax.grid(alpha=0.3)
        ax.legend()
        ax.set_title(cap.meta.get("note") or TEST)
        fig.tight_layout()
        path = (out or Path(".")) / "friction.png"
        fig.savefig(path, dpi=120)
        print(f"\nplot -> {path}")


# --------------------------------------------------------------------------
# the runner (the same shape as servo_breakaway's)
# --------------------------------------------------------------------------

def run(args) -> int:
    tee = Tee(sys.stdout)
    sys.stdout = tee
    bus = cap = orig = before = after = None
    torque_on, notes = False, {}
    try:
        if not args.port:
            raise SystemExit("no --port given and AOW_DXL_PORT is unset")
        bus = DynamixelBus(args.port, baud=args.baud, ids=tuple(args.ids)).open()
        wrong = {i: ct.name for i, ct in bus.tables.items() if ct.name != MODEL}
        if wrong:
            raise SystemExit(f"this test is {MODEL} only; found {wrong}")
        errs = bus.hardware_errors()
        if errs:
            raise SystemExit(f"latched hardware error "
                             f"{ {i: describe_hardware_error(e) for i, e in errs.items()} }")
        orig = bus.snapshot()
        for i in bus.ids:
            if args.cap_ma > orig[i]["Current Limit"]:
                raise SystemExit(f"id {i}: Current Limit {orig[i]['Current Limit']} mA "
                                 f"< --cap-ma {args.cap_ma:g}")
        bus.prepare()                           # torque off, Return Delay 0, profiles 0
        for i in bus.ids:
            bus.write_raw(i, "Operating Mode", MODE_CURRENT_POSITION)
        for i in bus.ids:                       # AFTER the mode write, which resets them
            bus.write_raw(i, "Position P Gain", args.p_gain)
            bus.write_raw(i, "Position D Gain", args.d_gain)
            bus.write_raw(i, "Position I Gain", 0)
            got = (bus.read_raw(i, "Position P Gain"), bus.read_raw(i, "Position D Gain"))
            if got != (args.p_gain, args.d_gain):
                raise RuntimeError(f"id {i}: wrote P/D {args.p_gain}/{args.d_gain}, read {got}")
        imap = IndirectMap(bus.tables)
        for name in READ_BLOCK:
            imap.read(name)
        imap.read("Goal Position", label="goal_readback")
        imap.write("Goal Position", label="goal")
        bus.apply_map(imap, verify=True)
        before = bus.snapshot()
        base = {i: bus.read(i, "Present Position") for i in bus.ids}
        bus.write_frame(dict(base))             # torque-on drives to the RAM goal
        segs = build_plan(args, base)
        total = sum(s.seconds for s in segs)
        v = {i: before[i]["Present Input Voltage"] * 0.1 for i in bus.ids}
        print(f"\n{TEST}: ids {list(bus.ids)}, supply {v} V, P/D {args.p_gain}/{args.d_gain}, "
              f"cap {args.cap_ma:g} mA; speeds {args.speeds} deg/s x {args.turns:g} turns "
              f"each way x {args.reps} reps, {total / 60:.1f} min at {args.rate:g} Hz.")
        if args.dry_run:
            for s in segs[:8]:
                print(f"  {s.label:38s} {s.seconds:5.1f} s  {s.info}")
            print(f"  ... {len(segs) - 8} more.  --dry-run: torque never enabled.")
            return 0
        if not args.yes:
            try:
                input(f"\nTorque ON for ids {list(bus.ids)}: the steering turns "
                      f"{args.turns:g} full turns each way. Clear the wheel's sweep. "
                      f"Enter to start (Ctrl-C aborts): ")
            except EOFError:
                raise SystemExit("no terminal to confirm on; pass --yes") from None
        bus.bus_watchdog(WATCHDOG_MS)
        torque_on = True
        stop = {}

        def guard(seg):
            inner = seg.command

            def cmd(t, s):
                goal = inner(t, s) if inner else None
                if s is not None:
                    tmax = max(s[i]["Present Temperature"] for i in s)
                    if tmax >= args.max_temp:
                        stop["hot_c"] = tmax
                        raise RuntimeError(f"{tmax:.0f} C >= --max-temp {args.max_temp}")
                    if goal and seg.info.get("kind") == "ramp":
                        lag = max(abs(math.degrees(goal[i] - s[i]["Present Position"]))
                                  for i in goal)
                        if lag > args.max_lag_deg:
                            stop["lag_deg"] = lag
                            raise RuntimeError(f"output {lag:.0f} deg behind the ramp "
                                               f"(> --max-lag-deg {args.max_lag_deg:g}): jammed?")
                return goal
            seg.command = cmd
            return seg

        def finish(b):
            notes["watchdog_tripped"] = b.watchdog_tripped()
            b.write_frame({i: b.read(i, "Present Position") for i in b.ids})
            b.bus_watchdog(0)

        cap = record(bus, imap, [guard(s) for s in segs], args.rate, finish=finish)
        notes.update(stop)
    finally:
        if bus is not None and bus._port is not None:
            if _try(bus.torque, False) is _FAILED:
                print("!! TORQUE OFF FAILED -- cut power to the servos")
            if torque_on:
                _try(bus.bus_watchdog, 0)
            if orig is not None:
                _try(lambda: [bus.write_raw(i, "Operating Mode", orig[i]["Operating Mode"])
                              for i in bus.ids])
                _try(lambda: [(bus.write_raw(i, "Position P Gain", orig[i]["Position P Gain"]),
                               bus.write_raw(i, "Position D Gain", orig[i]["Position D Gain"]),
                               bus.write_raw(i, "Position I Gain", orig[i]["Position I Gain"]))
                              for i in bus.ids])
                notes["restored"] = True
            after = _try(bus.snapshot)
            bus.close()
        sys.stdout = tee.stream
        if cap is not None:
            cap.meta.update(
                test=TEST, note=args.note, argv=sys.argv, port=args.port, baud=args.baud,
                gains={"p": args.p_gain, "d": args.d_gain}, cap_ma=args.cap_ma,
                git=git_state(ROOT), python=sys.version.split()[0],
                platform=platform.platform(), watchdog_ms=WATCHDOG_MS,
                snapshot_initial=orig, snapshot_before=before,
                snapshot_after=after if isinstance(after, dict) else None, shutdown=notes)
            out = new_capture_dir(Path(args.out), TEST, args.tag)
            cap.save(out)
            sys.stdout = tee
            print(f"\nsaved -> {out}")
            try:
                summarize(cap)
            except Exception as e:              # noqa: BLE001 -- the capture is saved
                print(f"\n(summary failed: {type(e).__name__}: {e}; the capture is intact)")
            sys.stdout = tee.stream
            (out / "log.txt").write_text("".join(tee.lines))
    return 0 if cap is not None and cap.meta["complete"] else 1


def main() -> int:
    if len(sys.argv) > 2 and sys.argv[1] == "summary":
        d = Path(sys.argv[2])
        summarize(Capture.load(d), plot="--plot" in sys.argv, out=d)
        return 0
    g = righting_gains()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=os.environ.get("AOW_DXL_PORT"))
    ap.add_argument("--baud", type=int, default=3_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[103])
    ap.add_argument("--speeds", type=float, nargs="+", default=[20.0, 60.0],
                    help="ramp speeds [deg/s]; two, to see whether it is Coulomb or viscous")
    ap.add_argument("--turns", type=float, default=2.0, help="turns each way per ramp")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--cap-ma", type=float, default=150.0,
                    help="Goal Current cap: ~0.11 N m, the most the printed pins ever see")
    ap.add_argument("--max-lag-deg", type=float, default=30.0)
    ap.add_argument("--p-gain", type=int, default=g["p"])
    ap.add_argument("--d-gain", type=int, default=g["d"])
    ap.add_argument("--rate", type=float, default=200.0)
    ap.add_argument("--max-temp", type=int, default=45)
    ap.add_argument("--out", default=str(ROOT / "traces" / TEST))
    ap.add_argument("--tag", default=None)
    ap.add_argument("--note", default="")
    ap.add_argument("--yes", action="store_true", help="skip the Enter prompt")
    ap.add_argument("--dry-run", action="store_true",
                    help="open the bus, print the schedule, never enable torque")
    return run(ap.parse_args())


if __name__ == "__main__":
    sys.exit(main())
