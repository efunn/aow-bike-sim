"""Breakaway sweep: at what Goal Current does a bare XC330 move its own output?

Current-based position mode (5), the mode the righting and rig servos run in.
Written to replace a GUESS of 100 mA (from rough experience that turned out
to be the XL330's). Its first run, 2026-09-25, both bare XC330s: nothing
moves at <= 20 mA, everything at >= 25, and running they stop between 22 and
20 -- `config/floor_rig.yaml` now has `servo_friction_ma: 22` from it. Plan
and interpretation: docs/plans/pre-assembly-bench-checklist.md, "Breakaway
sweep RUN".

    P=/dev/cu.usbserial-FTB8HNE3            # or export AOW_DXL_PORT
    python analysis/servo_breakaway.py --port $P --dry-run      # schedule only
    python analysis/servo_breakaway.py --port $P --note "bare, horn only"
    python analysis/servo_breakaway.py summary traces/servo_breakaway/<capture dir>

Both servos (ids 103 and 104 by default) run the same schedule at once, each
relative to its own starting angle. Nothing is attached, so nothing needs
holding. Three phases, one capture:

  OFFSET   1 s torque OFF, then 2 s torque on holding its goal at 50 mA:
           what Present Current reads when the motor does no work.
  BREAKAWAY per trial: HOME at `--home-ma` to a start angle, then set Goal
           Current I and step the goal `--step-deg` away for `--try-s`.
           The step is far enough that P x error is far past I, so the loop
           sits AT the cap -- the question is only whether I moves the
           output. Currents in shuffled order within each series; both
           directions; three start angles 120 deg apart (the gear mesh
           differs); `--reps` repeats.
  KINETIC  goal far away (`--kinetic-turns`), current stepped DOWN from
           `--kinetic-from-ma` every `--kinetic-step-s` until the output
           stops: the current that KEEPS it moving, against the breakaway
           current that STARTS it. Both directions.

READING IT. `moved` = the output turned more than `--moved-deg` during the
try; `reached` = it got within `--reach-deg` of the goal. Breakaway is the
lowest current at which every trial of that servo and direction moved. If
that comes out ~100 mA but the horn back-drives easily by hand, the threshold
is not gearbox friction but current the motor never turns into torque, and
the model belongs in righting_servo as a deadband (see the plan).

THERMAL. Bare shaft, <= 300 mA, mostly moving: well under the stall script's
worst point. Temperature is read every frame and the run stops at
`--max-temp` (45 C; the servo's own limit is 70).

Position gains are set to `--p-gain` / `--d-gain` (default: the bike's own
righting gains, control.onboard.gains.righting) AFTER the mode write, which
resets them, and everything is restored on exit.
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

import yaml  # noqa: E402

from aow_sim.hw.bench_log import (Capture, Segment, git_state,  # noqa: E402
                                  new_capture_dir, record)
from aow_sim.hw.dynamixel import (MODE_CURRENT_POSITION,  # noqa: E402
                                  DynamixelBus, IndirectMap,
                                  describe_hardware_error)
from aow_sim.params import DEFAULT_PARAMS  # noqa: E402

MODEL = "xc330_t181"
TEST = "servo_breakaway"
READ_BLOCK = ("Realtime Tick", "Present Position", "Present Velocity",
              "Present Current", "Present PWM", "Present Temperature",
              "Hardware Error Status")
WATCHDOG_MS = 100
OFFSET_MA = 50


def righting_gains() -> dict:
    raw = yaml.safe_load(Path(DEFAULT_PARAMS).read_text())
    g = raw["control"]["onboard"]["gains"]["righting"]
    return {"p": int(g["Position P Gain"]), "d": int(g["Position D Gain"])}


def set_current(ma: float):
    """A Segment.enter that writes Goal Current (RAM, fine with torque on)
    on every servo and reads it back -- a SyncWrite reports nothing."""
    def enter(bus):
        for i in bus.ids:
            bus.write(i, "Goal Current", ma / 1000)
            got = bus.read_raw(i, "Goal Current")
            if got != round(ma):
                raise RuntimeError(f"id {i}: wrote Goal Current {ma:.0f} mA, read {got}")
    return enter


def torque(on: bool):
    return lambda bus: bus.torque(on)


def build_plan(args, base: dict) -> list:
    """Every segment of the run. `base` is each servo's starting angle [rad]."""
    rng = np.random.default_rng(args.seed)
    step = math.radians(args.step_deg)
    hold_here = lambda t, s: {i: base[i] for i in base}
    segs = [Segment("offset: torque OFF", 1.0, None, enter=torque(False),
                    info={"kind": "offset", "torque": False}),
            Segment(f"offset: torque on, holding, {OFFSET_MA} mA", 2.0, hold_here,
                    enter=lambda bus: (set_current(OFFSET_MA)(bus), bus.torque(True)),
                    info={"kind": "offset", "torque": True, "ma": OFFSET_MA})]
    for rep in range(args.reps):
        for a_deg in args.angles:
            for sgn in (+1, -1):
                start = {i: base[i] + math.radians(a_deg) for i in base}
                goal = {i: start[i] + sgn * step for i in base}
                order = list(args.currents)
                rng.shuffle(order)
                # up to 240 deg to the series' start angle: longer than a home
                segs.append(Segment(f"to start angle {a_deg:g} deg", 1.5,
                                    lambda t, s, g=start: g, enter=set_current(args.home_ma),
                                    info={"kind": "move"}))
                for ma in order:
                    info = {"rep": rep, "angle_deg": a_deg, "dir": sgn, "ma": ma}
                    segs += [
                        Segment(f"home {a_deg:g} deg", args.home_s,
                                lambda t, s, g=start: g, enter=set_current(args.home_ma),
                                info={"kind": "home", **info}),
                        Segment(f"try {ma:g} mA {'+' if sgn > 0 else '-'} "
                                f"@{a_deg:g} rep {rep}", args.try_s,
                                lambda t, s, g=goal: g, enter=set_current(ma),
                                info={"kind": "try", "goal": {str(i): g for i, g in goal.items()},
                                      "start": {str(i): v for i, v in start.items()}, **info})]
    far = math.tau * args.kinetic_turns
    levels = np.arange(args.kinetic_from_ma, args.kinetic_to_ma - 1e-9, -args.kinetic_dec_ma)
    for sgn in (+1, -1):
        # The goal is set relative to wherever the output IS when the phase
        # starts (read from the first frame), so it never unwinds earlier turns.
        anchor = {}

        def far_goal(t, s, sgn=sgn, anchor=anchor):
            if not anchor and s is not None:
                anchor.update({i: s[i]["Present Position"] for i in s})
            return {i: anchor[i] + sgn * far for i in anchor} if anchor else None
        for k, ma in enumerate(levels):
            segs.append(Segment(f"kinetic {'+' if sgn > 0 else '-'} {ma:g} mA",
                                args.kinetic_step_s, far_goal, enter=set_current(ma),
                                info={"kind": "kinetic", "dir": sgn, "ma": float(ma),
                                      "level": k}))

        def stop_here(t, s):
            return {i: s[i]["Present Position"] for i in s} if s is not None else None
        segs.append(Segment("kinetic: stop where it is", 0.5, stop_here,
                            enter=set_current(args.home_ma), info={"kind": "stop"}))
    return segs


# --------------------------------------------------------------------------
# analysis
# --------------------------------------------------------------------------

def _seg_rows(cap: Capture, k: int) -> slice:
    s = cap.segments[k]
    return slice(s["start"], s["stop"])


def summarize(cap: Capture, args=None) -> None:
    moved_deg = cap.meta.get("moved_deg", 1.0)
    reach_deg = cap.meta.get("reach_deg", 2.0)
    ids = [int(i) for i in cap.meta["ids"]]
    segs = cap.segments

    print("\nOFFSET -- Present Current with no work being done:")
    for k, s in enumerate(segs):
        if s["info"].get("kind") != "offset":
            continue
        sl = _seg_rows(cap, k)
        for i in ids:
            cur = cap.value("Present Current", i)[sl][cap.ok[sl]] * 1000
            if len(cur):
                print(f"  id {i} {s['label']:34s} mean {cur.mean():+6.1f} mA  "
                      f"sd {cur.std():4.1f}  (n {len(cur)})")

    trials = []
    for k, s in enumerate(segs):
        info = s["info"]
        if info.get("kind") != "try":
            continue
        sl = _seg_rows(cap, k)
        ok = cap.ok[sl]
        for i in ids:
            pos = cap.position_rad(i)[sl][ok]
            cur = cap.value("Present Current", i)[sl][ok] * 1000
            if len(pos) < 5:
                continue
            p0 = pos[0]
            goal = info["goal"][str(i)]
            disp = math.degrees(pos[-1] - p0) * info["dir"]
            miss = abs(math.degrees(pos[-1] - goal))
            trials.append({"id": i, "dir": info["dir"], "angle": info["angle_deg"],
                           "rep": info["rep"], "ma": info["ma"], "disp": disp,
                           "moved": disp > moved_deg, "reached": miss < reach_deg,
                           "i_rep": float(np.median(np.abs(cur)))})

    if trials:
        print(f"\nBREAKAWAY -- per current: trials that moved (> {moved_deg:g} deg) / "
              f"reached (< {reach_deg:g} deg off) / all; median |Present Current|")
        for i in ids:
            for sgn in (+1, -1):
                sub = [t for t in trials if t["id"] == i and t["dir"] == sgn]
                if not sub:
                    continue
                print(f"  id {i}, direction {'+' if sgn > 0 else '-'}:")
                currents = sorted({t["ma"] for t in sub})
                all_moved = None
                for ma in currents:
                    at = [t for t in sub if t["ma"] == ma]
                    nm = sum(t["moved"] for t in at)
                    nr = sum(t["reached"] for t in at)
                    if all_moved is None and nm == len(at):
                        all_moved = ma
                    print(f"    {ma:5.0f} mA  moved {nm}/{len(at)}  reached {nr}/{len(at)}  "
                          f"median disp {np.median([t['disp'] for t in at]):6.1f} deg  "
                          f"|I| {np.median([t['i_rep'] for t in at]):5.0f} mA")
                print(f"    -> breakaway (every trial moved): "
                      f"{'none in the sweep' if all_moved is None else f'{all_moved:.0f} mA'}")

    print("\nKINETIC -- speed while the current is stepped down:")
    for i in ids:
        for sgn in (+1, -1):
            rows = []
            for k, s in enumerate(segs):
                info = s["info"]
                if info.get("kind") != "kinetic" or info["dir"] != sgn:
                    continue
                sl = _seg_rows(cap, k)
                ok = cap.ok[sl]
                pos = cap.position_rad(i)[sl][ok]
                t = cap.time_s(i)[sl][ok]
                if len(pos) < 5:
                    continue
                half = len(pos) // 2           # second half: past the transient
                w = (pos[-1] - pos[half]) / max(t[-1] - t[half], 1e-6) * sgn
                rows.append((info["ma"], w))
            if not rows:
                continue
            stalled = next((ma for ma, w in rows if w < 0.05), None)
            print(f"  id {i} {'+' if sgn > 0 else '-'}: " + "  ".join(
                f"{ma:.0f}:{w:.2f}" for ma, w in rows) + " (mA:rad/s)")
            print(f"    -> stops at {'-- (never)' if stalled is None else f'{stalled:.0f} mA'}")


# --------------------------------------------------------------------------
# the runner
# --------------------------------------------------------------------------

class Tee:
    """stdout to the terminal AND into the capture's log.txt."""

    def __init__(self, stream):
        self.stream, self.lines = stream, []

    def write(self, s):
        self.lines.append(s)
        return self.stream.write(s)

    def flush(self):
        self.stream.flush()


_FAILED = object()


def _try(fn, *a):
    try:
        return fn(*a)
    except Exception as e:                      # noqa: BLE001 -- shutdown must continue
        print(f"  shutdown step {getattr(fn, '__name__', fn)} failed: {e}")
        return _FAILED


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
        if errs and args.reboot:
            print(f"rebooting {sorted(errs)}: "
                  f"{ {i: describe_hardware_error(e) for i, e in errs.items()} }")
            bus.reboot(list(errs))
            errs = bus.hardware_errors()
        if errs:
            raise SystemExit(f"latched hardware error "
                             f"{ {i: describe_hardware_error(e) for i, e in errs.items()} }; "
                             f"find the cause, then pass --reboot")
        orig = bus.snapshot()
        top = max([args.home_ma, args.kinetic_from_ma, *args.currents])
        for i in bus.ids:
            lim = orig[i]["Current Limit"]
            if top > lim:
                raise SystemExit(f"id {i}: Current Limit is {lim} mA, the schedule "
                                 f"asks for {top:.0f}")

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
        # RAM keeps the last goal; torque-on drives to it. Make it here.
        bus.write_frame(dict(base))
        segs = build_plan(args, base)
        total = sum(s.seconds for s in segs)
        n_try = sum(s.info.get("kind") == "try" for s in segs)
        v = {i: before[i]["Present Input Voltage"] * 0.1 for i in bus.ids}
        print(f"\n{TEST}: ids {list(bus.ids)}, supply {v} V, P/D {args.p_gain}/{args.d_gain}; "
              f"{n_try} tries of {args.try_s:g} s, {len(segs)} segments, {total / 60:.1f} min "
              f"at {args.rate:g} Hz. Currents {sorted(args.currents)} mA, step "
              f"{args.step_deg:g} deg, start angles {args.angles}, {args.reps} reps.")
        if args.dry_run:
            for s in segs[:12]:
                print(f"  {s.label:34s} {s.seconds:4.1f} s  {s.info}")
            print(f"  ... {len(segs) - 12} more.  --dry-run: torque never enabled.")
            return 0
        if not args.yes:
            try:
                input(f"\nTorque ON for ids {list(bus.ids)}, bare horns turning up to "
                      f"{args.kinetic_turns:g} turns. Enter to start (Ctrl-C aborts): ")
            except EOFError:
                raise SystemExit("no terminal to confirm on; pass --yes") from None
        bus.bus_watchdog(WATCHDOG_MS)
        torque_on = True                        # the second offset segment enables it

        hot = {"t": None}

        def guard(seg):
            """Wrap each command with the temperature check."""
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
            b.write_frame({i: b.read(i, "Present Position") for i in b.ids})
            b.bus_watchdog(0)

        cap = record(bus, imap, [guard(s) for s in segs], args.rate, finish=finish)
        if hot["t"] is not None:
            notes["stopped_hot_c"] = hot["t"]
    finally:
        if bus is not None and bus._port is not None:
            if _try(bus.torque, False) is _FAILED:
                print("!! TORQUE OFF FAILED -- cut power to the servos")
            if torque_on:
                _try(bus.bus_watchdog, 0)
            if orig is not None:
                _try(lambda: [bus.write_raw(i, "Operating Mode", orig[i]["Operating Mode"])
                              for i in bus.ids])
                # after the mode, never before: a mode write resets gains
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
                moved_deg=args.moved_deg, reach_deg=args.reach_deg,
                gains={"p": args.p_gain, "d": args.d_gain},
                git=git_state(ROOT), python=sys.version.split()[0],
                platform=platform.platform(), watchdog_ms=WATCHDOG_MS,
                snapshot_initial=orig, snapshot_before=before,
                snapshot_after=None if after is _FAILED else after, shutdown=notes)
            out = new_capture_dir(Path(args.out), TEST, args.tag)
            cap.save(out)
            sys.stdout = tee
            print(f"\nsaved -> {out.relative_to(ROOT) if out.is_relative_to(ROOT) else out}")
            try:
                summarize(cap)
            except Exception as e:              # noqa: BLE001 -- the capture is saved
                print(f"\n(summary failed: {type(e).__name__}: {e}; the capture is intact)")
            sys.stdout = tee.stream
            (out / "log.txt").write_text("".join(tee.lines))
    return 0 if cap is not None and cap.meta["complete"] else 1


def main() -> int:
    if len(sys.argv) > 2 and sys.argv[1] == "summary":
        summarize(Capture.load(sys.argv[2]))
        return 0
    g = righting_gains()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=os.environ.get("AOW_DXL_PORT"))
    ap.add_argument("--baud", type=int, default=3_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[103, 104])
    ap.add_argument("--currents", type=float, nargs="+",
                    default=[2, 4, 6, 8, 10, 12, 15, 20, 25, 30, 40, 50, 70, 100, 150],
                    help="Goal Current per try [mA]. Low-heavy: a bare XC330 reached a "
                         "45 deg goal at 50 mA and ran at its no-load speed there "
                         "(shakedown, 2026-09-25)")
    ap.add_argument("--step-deg", type=float, default=45.0,
                    help="goal step per try; at P 700 that asks ~1.4 A, far past any cap")
    ap.add_argument("--angles", type=float, nargs="+", default=[0, 120, 240],
                    help="start angles [deg] from where each servo begins")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--home-ma", type=float, default=300,
                    help="Goal Current for the move back to the start angle")
    ap.add_argument("--home-s", type=float, default=0.6)
    ap.add_argument("--try-s", type=float, default=1.2)
    ap.add_argument("--kinetic-from-ma", type=float, default=60)
    ap.add_argument("--kinetic-to-ma", type=float, default=2)
    ap.add_argument("--kinetic-dec-ma", type=float, default=2)
    ap.add_argument("--kinetic-step-s", type=float, default=0.5)
    ap.add_argument("--kinetic-turns", type=float, default=60,
                    help="goal distance for the kinetic phase; must outrun it")
    ap.add_argument("--p-gain", type=int, default=g["p"],
                    help=f"Position P Gain (default {g['p']}, the bike's righting gain)")
    ap.add_argument("--d-gain", type=int, default=g["d"])
    ap.add_argument("--moved-deg", type=float, default=1.0)
    ap.add_argument("--reach-deg", type=float, default=2.0)
    ap.add_argument("--rate", type=float, default=200.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-temp", type=int, default=45)
    ap.add_argument("--out", default=str(ROOT / "traces" / TEST))
    ap.add_argument("--tag", default=None)
    ap.add_argument("--note", default="")
    ap.add_argument("--reboot", action="store_true",
                    help="reboot a servo with a latched hardware error first")
    ap.add_argument("--yes", action="store_true", help="skip the Enter prompt")
    ap.add_argument("--dry-run", action="store_true",
                    help="open the bus, print the schedule, never enable torque")
    return run(ap.parse_args())


if __name__ == "__main__":
    sys.exit(main())
