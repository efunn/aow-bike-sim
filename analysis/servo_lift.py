"""Loaded lever: the Goal Current at which an XC330 lifts a known torque, and
the one below which it lets it fall.

The X330 fixture (`aow_sim.cad_x330_fixture`, IDLER) puts a two-armed lever
on the horn, shaft horizontal, with a mass slot in each arm. A mass m at r
from the shaft is a gravity torque m g r cos(angle) on the output, KNOWN
without any current law. So this asks the question the breakaway sweep
(`servo_breakaway.py`) could not: how much output torque a given bus current
makes at the bottom of the range, and how much of the ~22 mA breakaway is
deadband versus gearbox friction.

    P=/dev/ttyUSB0                              # on the Pi; or AOW_DXL_PORT
    python analysis/servo_lift.py --port $P --dry-run
    python analysis/servo_lift.py --port $P --mass-g 117 --radius-mm 44 --note "..."
    python analysis/servo_lift.py summary traces/servo_lift/<capture dir>

ONE CONFIGURATION PER RUN: move the mass between runs and say where it is on
the command line; it goes into meta.json. The lever starts LEVEL (--level-deg,
the fixture's 180) and every goal stays within --step-deg of it, well inside
the fixture's +-45 deg stops.

  SETTLE    hold level at --home-ma.
  HOLD      1 s more: Present Current while carrying the load, no motion.
  TRIES     per trial: HOME to level at --home-ma, then set Goal Current I and
            step the goal --step-deg UP or DOWN. Currents shuffled, both
            directions, --reps repeats.

READING IT. Each try is classified by its extremes, not its end point:
`toward` = the output went > --moved-deg toward the goal, `against` = it went
> --moved-deg the other way (the load won), `held` = neither. With the load
pulling one way, the direction AGAINST the load gives two thresholds -- the
current that lifts it (static friction + load) and the current below which it
falls (load - friction). Their midpoint is the current that balances the
load alone; half their gap is the friction. With several loads the midpoints
against torque are the current -> torque law at low current, and its
intercept is the deadband.

SAFETY. Every frame, if the output is more than --guard-deg from level, or
has moved --fall-deg against its goal, the script sets Goal Current back to
--home-ma (torque back on, if off) and commands level: the load is caught
rather than dropped onto the stop. Such a trial is marked `rescued`.
Temperature stops the run at --max-temp.

TORQUE NEVER GOES OFF WITH THE LOAD RAISED. A loaded lever back-drives the
gearbox (34 g at 15-25 mm does), so there is no torque-off phase; the start
refuses if torque is already on (the mode change would drop it); and EVERY
exit -- normal end, error, Ctrl-C, a failed lift -- ramps the lever back to
where it was found (`lower_to_start`) before torque goes off. 2026-09-25: an
earlier abort path did not, dropped 117 g at 44 mm 45 deg onto the stop, and
the horn pins sheared.
"""

from __future__ import annotations

import argparse
import math
import os
import platform
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))     # this checkout's source, even from a worktree

from aow_sim.hw.bench_log import (Capture, Segment, git_state,  # noqa: E402
                                  new_capture_dir, record)
from aow_sim.hw.dynamixel import (MODE_CURRENT_POSITION,  # noqa: E402
                                  DynamixelBus, IndirectMap,
                                  describe_hardware_error)
from servo_breakaway import (_FAILED, MODEL, READ_BLOCK, WATCHDOG_MS,  # noqa: E402
                             Tee, _try, righting_gains, set_current)

TEST = "servo_lift"
G = 9.81


def build_plan(args, level: dict, ctx: dict) -> list:
    """Every segment of the run. `level` is each servo's level angle [rad];
    `ctx` carries the bus (set once it is open) and the rescue log."""
    rng = np.random.default_rng(args.seed)
    step = math.radians(args.step_deg)
    guard = math.radians(args.guard_deg)
    fall = math.radians(args.fall_deg)

    def rescue(k_seg, why):
        bus = ctx["bus"]
        for i in bus.ids:
            bus.write(i, "Goal Current", args.home_ma / 1000)
        bus.torque(True)
        ctx["rescues"].append({"segment": k_seg, "why": why})
        ctx["caught"] = k_seg

    def guarded(k_seg, goal, sgn=None):
        """command(t, s): `goal` until a guard trips, then level. `sgn` +-1:
        catch a fall against a goal that way; 0: catch --fall-deg of motion
        either way (torque off); None: the level guard only."""
        def cmd(t, s):
            if ctx.get("caught") == k_seg:
                return dict(level)
            if s is not None:
                for i in s:
                    p = s[i]["Present Position"]
                    off = p - level[i]
                    if abs(off) > guard:
                        rescue(k_seg, f"id {i} {math.degrees(off):+.1f} deg from level")
                        return dict(level)
                    d = p - ctx["start"][k_seg][i] if k_seg in ctx["start"] else 0.0
                    if sgn is not None and (abs(d) if sgn == 0 else -d * sgn) > fall:
                        rescue(k_seg, f"id {i} fell {math.degrees(p - ctx['start'][k_seg][i]):+.1f} deg")
                        return dict(level)
                if k_seg not in ctx["start"]:
                    ctx["start"][k_seg] = {i: s[i]["Present Position"] for i in s}
            return goal
        return cmd

    def home_enter(bus):
        set_current(args.home_ma)(bus)
        bus.torque(True)

    segs = []

    def add(label, seconds, goal, enter, info, sgn=None):
        k = len(segs)
        segs.append(Segment(label, seconds, guarded(k, goal, sgn), enter=enter, info=info))

    add("settle at level", 1.5, dict(level), home_enter, {"kind": "settle"})
    add(f"hold level, {args.home_ma:g} mA", 1.0, dict(level), None,
        {"kind": "hold", "ma": args.home_ma})
    for rep in range(args.reps):
        for sgn in (+1, -1):
            order = list(args.currents)
            rng.shuffle(order)
            goal = {i: level[i] + sgn * step for i in level}
            for ma in order:
                info = {"rep": rep, "dir": sgn, "ma": ma}
                add("home", args.home_s, dict(level), set_current(args.home_ma),
                    {"kind": "home", **info})
                add(f"try {ma:g} mA {'+' if sgn > 0 else '-'} rep {rep}", args.try_s,
                    goal, set_current(ma), {"kind": "try", **info}, sgn=sgn)
    add("end: level", 1.0, dict(level), set_current(args.home_ma), {"kind": "home"})
    return segs


# --------------------------------------------------------------------------
# analysis
# --------------------------------------------------------------------------

def _rows(cap: Capture, k: int) -> slice:
    s = cap.segments[k]
    return slice(s["start"], s["stop"])


def load_torque(meta: dict) -> float | None:
    m, r = meta.get("mass_g"), meta.get("radius_mm")
    return None if not m or r is None else m / 1000 * G * r / 1000


def summarize(cap: Capture) -> dict:
    meta = cap.meta
    moved = math.radians(meta.get("moved_deg", 1.0))
    ids = [int(i) for i in meta["ids"]]
    rescued = {r["segment"] for r in meta.get("rescues", [])}
    tau = load_torque(meta)
    print(f"\nLOAD: {meta.get('mass_g') or 0:g} g at {meta.get('radius_mm') or 0:g} mm "
          f"({meta.get('side') or '-'} arm) = "
          f"{'no load' if tau is None else f'{tau * 1000:.1f} mN m'} at level; "
          f"{meta.get('note', '')}")
    out = {"tau_nm": tau, "ids": {}}
    for i in ids:
        res = out["ids"][i] = {}
        for k, s in enumerate(cap.segments):
            kind = s["info"].get("kind")
            if kind not in ("hold", "off"):
                continue
            sl = _rows(cap, k)
            ok = cap.ok[sl]
            cur = cap.value("Present Current", i)[sl][ok] * 1000
            pos = cap.position_rad(i)[sl][ok]
            if not len(pos):
                continue
            dp = math.degrees(pos[-1] - pos[0])
            span = math.degrees(pos.max() - pos.min())
            flag = " RESCUED" if k in rescued else ""
            print(f"  id {i} {s['label']:22s} |I| mean {np.abs(cur).mean():5.1f} mA  "
                  f"moved {dp:+5.1f} deg (span {span:4.1f}){flag}")
            res[kind] = {"i_ma": float(np.abs(cur).mean()), "moved_deg": dp,
                         "rescued": k in rescued}

        trials = []
        for k, s in enumerate(cap.segments):
            info = s["info"]
            if info.get("kind") != "try":
                continue
            sl = _rows(cap, k)
            ok = cap.ok[sl]
            pos = cap.position_rad(i)[sl][ok]
            cur = cap.value("Present Current", i)[sl][ok] * 1000
            if len(pos) < 5:
                continue
            d = (pos - pos[0]) * info["dir"]
            toward, against = d.max(), -d.min()
            cls = ("against" if against > moved else "toward" if toward > moved else "held")
            trials.append({"dir": info["dir"], "ma": info["ma"], "cls": cls,
                           "rescued": k in rescued,
                           "i": float(np.median(cur * info["dir"]))})
        for sgn in (+1, -1):
            sub = [t for t in trials if t["dir"] == sgn]
            if not sub:
                continue
            print(f"  id {i}, goal {'+' if sgn > 0 else '-'}:  current  toward/held/against  "
                  f"median signed Present Current")
            lift = fall = None
            for ma in sorted({t["ma"] for t in sub}):
                at = [t for t in sub if t["ma"] == ma]
                n = {c: sum(t["cls"] == c for t in at) for c in ("toward", "held", "against")}
                nr = sum(t["rescued"] for t in at)
                if lift is None and n["toward"] == len(at):
                    lift = ma
                if n["against"]:
                    fall = ma
                print(f"      {ma:5.0f} mA   {n['toward']}/{n['held']}/{n['against']}"
                      f"{f'  ({nr} caught)' if nr else '':14s}  {np.median([t['i'] for t in at]):+6.1f} mA")
            print(f"    -> every try moved toward the goal from {lift} mA; "
                  f"the load won at up to {fall} mA")
            res[f"dir{sgn:+d}"] = {"lift_ma": lift, "fall_max_ma": fall}
    if meta.get("rescues"):
        print(f"\n  {len(meta['rescues'])} rescue(s): "
              + "; ".join(r["why"] for r in meta["rescues"][:6])
              + (" ..." if len(meta["rescues"]) > 6 else ""))
    return out


# --------------------------------------------------------------------------
# the runner
# --------------------------------------------------------------------------

def nearest_level(p: float, level_deg: float) -> float:
    """The multi-turn angle [rad] nearest `p` that reads `level_deg` mod 360."""
    lv, turn = math.radians(level_deg), math.tau
    return lv + round((p - lv) / turn) * turn


def lower_to_start(bus, args, ctx: dict) -> None:
    """Ramp the lever back to where the run found it -- usually resting on a
    stop -- at --home-ma, before ANY torque-off. On 2026-09-25 an abort path
    turned torque off with 117 g at 44 mm held level; it fell 45 deg onto the
    stop and the fixture sheared. Every exit goes through here."""
    start = ctx.get("present")
    if not start:
        return
    for i in bus.ids:
        bus.write(i, "Goal Current", args.home_ma / 1000)
    now = {i: bus.read(i, "Present Position") for i in bus.ids}
    n = 40
    for k in range(1, n + 1):
        bus.write_frame({i: now[i] + (start[i] - now[i]) * k / n for i in bus.ids})
        time.sleep(args.lift_s / n)
    time.sleep(0.3)


def run(args) -> int:
    tee = Tee(sys.stdout)
    sys.stdout = tee
    bus = cap = orig = before = after = None
    torque_on, notes = False, {}
    ctx = {"bus": None, "rescues": [], "start": {}}
    try:
        if not args.port:
            raise SystemExit("no --port given and AOW_DXL_PORT is unset")
        bus = DynamixelBus(args.port, baud=args.baud, ids=tuple(args.ids)).open()
        ctx["bus"] = bus
        wrong = {i: ct.name for i, ct in bus.tables.items() if ct.name != MODEL}
        if wrong:
            raise SystemExit(f"this test is {MODEL} only; found {wrong}")
        errs = bus.hardware_errors()
        if errs:
            raise SystemExit(f"latched hardware error "
                             f"{ {i: describe_hardware_error(e) for i, e in errs.items()} }")
        orig = bus.snapshot()
        # prepare() turns torque off to change mode. If something is already
        # holding the lever up, that drops it: refuse instead.
        held = [i for i in bus.ids if orig[i]["Torque Enable"]]
        if held:
            raise SystemExit(f"torque is ON for {held}: something may be holding a load up. "
                             f"Lower it and turn torque off by hand first")
        top = max([args.home_ma, *args.currents])
        for i in bus.ids:
            if top > orig[i]["Current Limit"]:
                raise SystemExit(f"id {i}: Current Limit {orig[i]['Current Limit']} mA "
                                 f"< the {top:.0f} this run asks for")

        bus.prepare()                           # torque off, Return Delay 0, profiles 0
        for i in bus.ids:
            bus.write_raw(i, "Operating Mode", MODE_CURRENT_POSITION)
        for i in bus.ids:                       # AFTER the mode write, which resets them
            bus.write_raw(i, "Position P Gain", args.p_gain)
            bus.write_raw(i, "Position D Gain", args.d_gain)
            bus.write_raw(i, "Position I Gain", 0)
        imap = IndirectMap(bus.tables)
        for name in READ_BLOCK:
            imap.read(name)
        imap.read("Goal Position", label="goal_readback")
        imap.write("Goal Position", label="goal")
        bus.apply_map(imap, verify=True)
        before = bus.snapshot()
        present = {i: bus.read(i, "Present Position") for i in bus.ids}
        level = {i: nearest_level(present[i], args.level_deg) for i in bus.ids}
        for i in bus.ids:
            off = math.degrees(present[i] - level[i])
            print(f"id {i}: {math.degrees(present[i]):.1f} deg, {off:+.1f} from level")
            if abs(off) > args.start_tol_deg:
                raise SystemExit(f"id {i} is {off:+.1f} deg from level ({args.level_deg:g}), "
                                 f"past the fixture's stops: check --level-deg")
        bus.write_frame(dict(present))          # torque-on holds where it is, then settles
        segs = build_plan(args, level, ctx)
        total = sum(s.seconds for s in segs)
        n_try = sum(s.info.get("kind") == "try" for s in segs)
        v = {i: before[i]["Present Input Voltage"] * 0.1 for i in bus.ids}
        tau = load_torque(vars(args))
        print(f"\n{TEST}: ids {list(bus.ids)}, supply {v} V, P/D {args.p_gain}/{args.d_gain}; "
              f"load {args.mass_g or 0:g} g at {args.radius_mm or 0:g} mm "
              f"({'none' if tau is None else f'{tau * 1000:.1f} mN m'}); "
              f"{n_try} tries, goal +-{args.step_deg:g} deg, {total / 60:.1f} min "
              f"at {args.rate:g} Hz. Currents {sorted(args.currents)} mA.")
        if args.dry_run:
            for s in segs[:10]:
                print(f"  {s.label:28s} {s.seconds:4.1f} s  {s.info}")
            print(f"  ... {len(segs) - 10} more.  --dry-run: torque never enabled.")
            return 0
        if not args.yes:
            try:
                input(f"\nTorque ON for ids {list(bus.ids)}, the lever swinging up to "
                      f"{args.step_deg:g} deg from level. Enter to start (Ctrl-C aborts): ")
            except EOFError:
                raise SystemExit("no terminal to confirm on; pass --yes") from None
        # A load back-drives the gearbox with torque off (34 g at 15-25 mm
        # does: user, 2026-09-25), so the lever usually starts on a stop.
        # Raise it to level along a ramp before the capture starts.
        set_current(args.home_ma)(bus)
        bus.torque(True)
        torque_on = True
        n = 40
        ctx["present"] = present
        for k in range(1, n + 1):
            bus.write_frame({i: present[i] + (level[i] - present[i]) * k / n for i in bus.ids})
            time.sleep(args.lift_s / n)
        time.sleep(0.5)
        for i in bus.ids:
            off = math.degrees(bus.read(i, "Present Position") - level[i])
            if abs(off) > 8:            # P droop under load: 3.3 deg at 50 mN m, P 700
                raise SystemExit(f"id {i} did not reach level at {args.home_ma:g} mA "
                                 f"({off:+.1f} deg off)")
        bus.bus_watchdog(WATCHDOG_MS)

        hot = {"t": None}

        def temp_guard(seg):
            inner = seg.command

            def cmd(t, s):
                if s is not None:
                    tmax = max(s[i]["Present Temperature"] for i in s)
                    if tmax >= args.max_temp:
                        hot["t"] = tmax
                        raise RuntimeError(f"{tmax:.0f} C >= --max-temp {args.max_temp}")
                return inner(t, s) if inner else None
            seg.command = cmd
            return seg

        def finish(b):
            notes["watchdog_tripped"] = b.watchdog_tripped()
            b.bus_watchdog(0)
            lower_to_start(b, args, ctx)
            notes["lowered"] = True

        cap = record(bus, imap, [temp_guard(s) for s in segs], args.rate, finish=finish)
        if hot["t"] is not None:
            notes["stopped_hot_c"] = hot["t"]
    finally:
        if bus is not None and bus._port is not None:
            if torque_on and not notes.get("lowered"):
                _try(bus.bus_watchdog, 0)
                _try(lower_to_start, bus, args, ctx)
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
                mass_g=args.mass_g, radius_mm=args.radius_mm, side=args.side,
                level_deg=args.level_deg, step_deg=args.step_deg,
                moved_deg=args.moved_deg, rescues=ctx["rescues"],
                gains={"p": args.p_gain, "d": args.d_gain},
                git=git_state(ROOT), python=sys.version.split()[0],
                platform=platform.platform(), watchdog_ms=WATCHDOG_MS,
                snapshot_initial=orig, snapshot_before=before,
                snapshot_after=None if after is _FAILED else after, shutdown=notes)
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
        for d in sys.argv[2:]:
            summarize(Capture.load(d))
        return 0
    g = righting_gains()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=os.environ.get("AOW_DXL_PORT"))
    ap.add_argument("--baud", type=int, default=3_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[103],
                    help="the servo carrying the lever (104 is the bare one)")
    ap.add_argument("--mass-g", type=float, default=0.0,
                    help="hung mass incl. its screw and washers (0: lever alone)")
    ap.add_argument("--radius-mm", type=float, default=None,
                    help="its distance from the shaft axis (the arm's ticks)")
    ap.add_argument("--side", default=None, help="which arm, for the record")
    ap.add_argument("--currents", type=float, nargs="+",
                    default=[0, 5, 10, *range(15, 46), 50, 60, 70, 80, 100, 120, 150,
                             200, 250],
                    help="Goal Current per try [mA]. 1 mA steps over 15-45: under the "
                         "sqrt law the whole 15-71 mN m load range sits within ~7 mA "
                         "of the ~20 mA deadband, so 5 mA steps could not tell it from "
                         "the linear law, which spreads it over ~15-80")
    ap.add_argument("--level-deg", type=float, default=180.0,
                    help="Present Position with the lever level (the fixture's zero)")
    ap.add_argument("--start-tol-deg", type=float, default=50.0)
    ap.add_argument("--lift-s", type=float, default=1.5,
                    help="ramp from the starting angle to level before the capture")
    ap.add_argument("--step-deg", type=float, default=20.0,
                    help="goal distance from level per try; stops are at 45")
    ap.add_argument("--guard-deg", type=float, default=32.0,
                    help="catch the load this far from level")
    ap.add_argument("--fall-deg", type=float, default=6.0,
                    help="catch the load once it has gone this far against the goal")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--home-ma", type=float, default=300)
    ap.add_argument("--home-s", type=float, default=0.8)
    ap.add_argument("--try-s", type=float, default=1.2)
    ap.add_argument("--p-gain", type=int, default=g["p"])
    ap.add_argument("--d-gain", type=int, default=g["d"])
    ap.add_argument("--moved-deg", type=float, default=1.0)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--seed", type=int, default=0)
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
