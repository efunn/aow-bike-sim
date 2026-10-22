"""Slow sweep of the righting module through its stroke: does the mechanism
move freely, and what current does it take to move it?

The first powered run of the V2 module as built (crank, couplers, wing hubs,
stubs; no blades). Nothing heavy rides on it, so the current it draws while
moving slowly is the MECHANISM'S FRICTION, plus the stubs' small gravity
torque, as a function of crank angle.

    P=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8HNE3-if00-port0
    python analysis/mechanism/righting_sweep.py --port $P --dry-run      # schedule, no torque
    python analysis/mechanism/righting_sweep.py --port $P --amps 30 --note "first motion"
    python analysis/mechanism/righting_sweep.py --port $P --breakaway -100 -45 0 45 100 --cap-ma 300
    python analysis/mechanism/righting_sweep.py summary traces/righting_sweep/<capture dir>

--breakaway replaces the sweep with a static-friction test at each angle: a
goal CREEP at --creep-dps from rest, never a step (see build_breakaway_plan).

THE SCHEDULE, all in crank degrees from --stow-deg (servo degrees: the servo
drives the crank directly, gear ratio 1):

  HOLD     1 s torque on, goal = wherever it already is. Present Current with
           no motion: the offset.
  TO STOW  a ramp at --speed-dps from there to stow.
  SWEEPS   per amplitude A in --amps: stow -> +A -> stow -> -A -> stow, each
           leg a constant-speed ramp of the GOAL, --dwell-s at each end.
  HOLD     1 s at stow.

The goal moves smoothly, so the servo tracks it with a small lag and draws
only what the mechanism resists. 129.3 is the module's commanded stroke
(`righting-design.md`); with BLADES fitted their toes stop it at ~130. With
STUBS ONLY there is no toe stop: the next contact is the knuckle on the other
wing's coupler, touching at ~138 (planar CAD sweep, not measured). Start with
a small --amps and grow it.

SAFETY. Goal Current is capped at --cap-ma, so a bind stalls the servo rather
than forcing the linkage. Every frame:
  * on a moving leg, |goal - position| > --lag-deg AND speed < --stall-dps
    for --lag-s -> stop (stalled: a bind, or friction above the cap). Lag
    alone is not a stall: with Position I 0 it grows ~30 mA/deg;
  * |goal - position| > --runaway-deg anywhere -> stop;
  * |position - stow| > max(--amps) + 3 deg    -> stop (outside the plan);
  * temperature >= --max-temp                  -> stop.
A stop holds the position it reached, then torque goes off. Ctrl-C does the
same. The capture is saved either way.

READING IT. `summary` bins each constant-speed leg by crank angle and prints
the median Present Current per bin and direction. Against the hold offset:
  friction ~ (I_up - I_down) / 2   (it opposes motion, so it flips sign)
  bias     ~ (I_up + I_down) / 2   (gravity on the stubs, spring, offset)
The bare XC330's own kinetic friction is ~20-22 mA (`servo_breakaway.py`,
2026-09-25), so anything well above that is the linkage.
"""

from __future__ import annotations

import argparse
import math
import os
import platform
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))     # this checkout's source, even from a worktree
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "servo"))  # sibling category

from servo_breakaway import _FAILED, Tee, _try, righting_gains  # noqa: E402

from aow_sim.hw.bench_log import (Capture, Segment, git_state,  # noqa: E402
                                  new_capture_dir, record)
from aow_sim.hw.dynamixel import (MODE_CURRENT_POSITION,  # noqa: E402
                                  DynamixelBus, IndirectMap,
                                  describe_hardware_error)

MODEL = "xc330_t181"
TEST = "righting_sweep"
READ_BLOCK = ("Realtime Tick", "Present Position", "Present Velocity",
              "Present Current", "Present PWM", "Present Temperature",
              "Hardware Error Status")
WATCHDOG_MS = 100


def set_current(ma: float):
    def enter(bus):
        for i in bus.ids:
            bus.write(i, "Goal Current", ma / 1000)
            got = bus.read_raw(i, "Goal Current")
            if got != round(ma):
                raise RuntimeError(f"id {i}: wrote Goal Current {ma:.0f} mA, read {got}")
    return enter


def ramp(a: float, b: float, speed: float):
    """Goal from a to b [rad] at `speed` [rad/s]; returns (command, seconds)."""
    T = abs(b - a) / speed

    def cmd(t, s, a=a, b=b, T=T):
        x = b if T <= 0 else a + (b - a) * min(t / T, 1.0)
        return {i: x for i in s} if s is not None else None
    return cmd, T


def build_plan(args, start: float, stow: float) -> list:
    v = math.radians(args.speed_dps)
    hold = lambda g: (lambda t, s: {i: g for i in s} if s is not None else None)
    segs = [Segment("hold where it is", 1.0, hold(start),
                    enter=lambda bus: (set_current(args.cap_ma)(bus), bus.torque(True)),
                    info={"kind": "hold"})]
    cmd, T = ramp(start, stow, v)
    segs.append(Segment("to stow", T + 0.3, cmd, info={"kind": "move"}))
    for A in args.amps:
        a = math.radians(A)
        for k, (p, q) in enumerate(((0, +a), (+a, 0), (0, -a), (-a, 0))):
            if args.step:
                # THE BIKE'S OWN COMMAND: hw/ground.py jumps the goal to
                # stow +- travel and the current cap sets the speed.
                segs.append(Segment(f"A {A:g}: STEP {math.degrees(p):+.0f} -> "
                                    f"{math.degrees(q):+.0f}", args.step_s,
                                    hold(stow + q),
                                    info={"kind": "step", "amp_deg": A,
                                          "dir": 1 if q > p else -1,
                                          "from_deg": math.degrees(p),
                                          "to_deg": math.degrees(q)}))
                continue
            cmd, T = ramp(stow + p, stow + q, v)
            sgn = 1 if q > p else -1
            segs.append(Segment(f"A {A:g}: {math.degrees(p):+.0f} -> {math.degrees(q):+.0f}",
                                T, cmd, info={"kind": "leg", "amp_deg": A, "dir": sgn,
                                              "from_deg": math.degrees(p),
                                              "to_deg": math.degrees(q)}))
            segs.append(Segment(f"A {A:g}: dwell at {math.degrees(q):+.0f}", args.dwell_s,
                                hold(stow + q), info={"kind": "dwell", "amp_deg": A,
                                                      "at_deg": math.degrees(q)}))
    segs.append(Segment("hold at stow", 1.0, hold(stow), info={"kind": "hold"}))
    return segs


def build_breakaway_plan(args, start: float, stow: float) -> list:
    """Static friction at each crank angle, with NO step anywhere.

    Per angle a and direction d: ramp to a - d*approach, then ramp to a, so
    the servo arrives moving AGAINST d and parks with its residual current
    opposing the test. Rest. Then CREEP the goal in d at --creep-dps: the
    current grows with the lag (Position I is 0) until the mechanism breaks
    free, and the current at the first motion is the breakaway. The servo
    then follows the goal at the creep rate, never faster than it slips.

    Stepping the goal instead is what servo_breakaway.py does, and on this
    module (2026-10-17) its 300 mA homing threw the crank at 630 deg/s.
    """
    v = math.radians(args.speed_dps)
    creep = math.radians(args.creep_dps)
    hold = lambda g: (lambda t, s: {i: g for i in s} if s is not None else None)
    segs = [Segment("hold where it is", 1.0, hold(start),
                    enter=lambda bus: (set_current(args.cap_ma)(bus), bus.torque(True)),
                    info={"kind": "hold"})]
    here = start
    for rep in range(args.reps):
        for a_deg in args.breakaway:
            a = stow + math.radians(a_deg)
            for d in (+1, -1):
                pre = a - d * math.radians(args.approach_deg)
                for label, to in (("approach", pre), ("to", a)):
                    cmd, T = ramp(here, to, v)
                    segs.append(Segment(f"{a_deg:+g} {'+-'[d < 0]}: {label}", T, cmd,
                                        info={"kind": "move"}))
                    here = to
                segs.append(Segment(f"{a_deg:+g} {'+-'[d < 0]}: rest", args.rest_s,
                                    hold(a), info={"kind": "rest"}))
                end = a + d * math.radians(args.creep_deg)
                cmd, T = ramp(a, end, creep)
                segs.append(Segment(f"{a_deg:+g} {'+-'[d < 0]}: creep rep {rep}", T, cmd,
                                    info={"kind": "creep", "angle_deg": a_deg, "dir": d,
                                          "rep": rep}))
                here = end
    cmd, T = ramp(here, stow, v)
    segs.append(Segment("to stow", T + 0.5, cmd, info={"kind": "move"}))
    segs.append(Segment("hold at stow", 1.0, hold(stow), info={"kind": "hold"}))
    return segs


# --------------------------------------------------------------------------
# analysis
# --------------------------------------------------------------------------

def summarize_breakaway(cap: Capture, moved_deg: float = 0.5) -> None:
    stow = math.radians(cap.meta["stow_deg"])
    i = int(cap.meta["ids"][0])
    pos = cap.position_rad(i)
    cur = cap.value("Present Current", i) * 1000
    vel = np.degrees(cap.value("Present Velocity", i))
    res = {}
    for s in cap.segments:
        info = s["info"]
        if info.get("kind") != "creep":
            continue
        sl = slice(s["start"], s["stop"])
        ok = cap.ok[sl]
        p, c, w = pos[sl][ok], cur[sl][ok], vel[sl][ok]
        if len(p) < 10:
            continue
        d = info["dir"]
        disp = np.degrees(p - p[0]) * d
        k = int(np.argmax(disp > moved_deg)) if (disp > moved_deg).any() else None
        # the current the moment before it went: max over the last 5 frames
        brk = None if k is None else float(np.max(c[max(0, k - 5):k + 1] * d))
        res.setdefault((info["angle_deg"], d), []).append(
            (brk, math.degrees(p[0] - stow), float(np.abs(w).max())))
    if not res:
        print("\nno creep trials recorded")
        return
    print(f"\nBREAKAWAY -- current at the first {moved_deg:g} deg of motion [mA], "
          f"per trial; peak speed in the trial")
    print(f"  {'angle':>6s} {'dir':>3s}  {'from crank':>10s}  breakaway  peak deg/s")
    for (a, d), trials in sorted(res.items()):
        for brk, p0, vmax in trials:
            b = f"{brk:6.0f}" if brk is not None else "  none"
            print(f"  {a:+6.0f} {'+-'[d < 0]:>3s}  {p0:+10.1f}  {b:>9s}  {vmax:6.0f}")
    print("\n  compare: kinetic (sweep) ~90 mA near stow, ~140 at 75-90, ~170-200 at 90-120;"
          "\n  bare XC330 breakaway ~25 mA")


def summarize_steps(cap: Capture) -> None:
    stow = math.radians(cap.meta["stow_deg"])
    i = int(cap.meta["ids"][0])
    pos = np.degrees(cap.position_rad(i) - stow)
    cur = cap.value("Present Current", i) * 1000
    vel = np.degrees(cap.value("Present Velocity", i))
    t_all = cap.time_s(i)
    print("\nSTEPS -- goal jumped, current cap sets the speed")
    print(f"  {'step':>16s} {'peak deg/s':>10s} {'peak mA':>8s} {'t90 s':>6s} "
          f"{'overshoot':>9s} {'final':>7s}")
    for s in cap.segments:
        info = s["info"]
        if info.get("kind") != "step":
            continue
        sl = slice(s["start"], s["stop"])
        ok = cap.ok[sl]
        p, c, w, t = pos[sl][ok], cur[sl][ok], vel[sl][ok], t_all[sl][ok]
        if len(p) < 5:
            continue
        a, b, d = info["from_deg"], info["to_deg"], info["dir"]
        prog = (p - p[0]) * d
        span = abs(b - p[0])
        k90 = np.argmax(prog >= 0.9 * span) if (prog >= 0.9 * span).any() else None
        t90 = f"{t[k90] - t[0]:6.3f}" if k90 is not None else "    --"
        over = max(0.0, float(np.max((p - b) * d)))
        print(f"  {a:+5.0f} -> {b:+5.0f} {np.abs(w).max():10.0f} {np.abs(c).max():8.0f} "
              f"{t90} {over:9.1f} {p[-1]:+7.1f}")
    print("\n  overshoot = furthest past the goal [deg]; final = crank at the segment's end")
    if cap.meta.get("shutdown", {}).get("stopped"):
        print(f"\nSTOPPED EARLY: {cap.meta['shutdown']['stopped']}")


def summarize(cap: Capture, bin_deg: float = 15.0) -> None:
    if cap.meta.get("breakaway"):
        summarize_breakaway(cap)
        return
    if cap.meta.get("step"):
        summarize_steps(cap)
        return
    stow = math.radians(cap.meta["stow_deg"])
    i = int(cap.meta["ids"][0])
    segs = cap.segments
    pos_all = cap.position_rad(i)
    cur_all = cap.value("Present Current", i) * 1000

    print("\nHOLD -- Present Current with no motion:")
    for s in segs:
        if s["info"].get("kind") == "hold":
            sl = slice(s["start"], s["stop"])
            c = cur_all[sl][cap.ok[sl]]
            if len(c):
                print(f"  {s['label']:22s} mean {c.mean():+6.1f} mA  sd {c.std():4.1f}  (n {len(c)})")

    rows = {+1: [], -1: []}
    reach = {+1: 0.0, -1: 0.0}
    for s in segs:
        info = s["info"]
        if info.get("kind") != "leg":
            continue
        sl = slice(s["start"], s["stop"])
        ok = cap.ok[sl]
        ang = np.degrees(pos_all[sl][ok] - stow)
        if len(ang):
            reach[+1] = max(reach[+1], ang.max())
            reach[-1] = min(reach[-1], ang.min())
        rows[info["dir"]].append((ang, cur_all[sl][ok]))
    if not rows[+1] and not rows[-1]:
        print("\nno sweep legs recorded")
        return
    print(f"\nREACHED: crank {reach[-1]:+.1f} .. {reach[+1]:+.1f} deg from stow")

    lo = math.floor(reach[-1] / bin_deg) * bin_deg
    hi = math.ceil(reach[+1] / bin_deg) * bin_deg
    edges = np.arange(lo, hi + bin_deg, bin_deg)
    cat = {d: (np.concatenate([a for a, _ in r]) if r else np.array([]),
               np.concatenate([c for _, c in r]) if r else np.array([]))
           for d, r in rows.items()}
    print(f"\nSWEEP -- median Present Current [mA] by crank angle, moving + / moving -")
    print(f"  {'crank deg':>13s} {'I +':>7s} {'I -':>7s} {'friction':>9s} {'bias':>7s}")
    for a0, a1 in zip(edges[:-1], edges[1:]):
        med = {}
        for d in (+1, -1):
            ang, cur = cat[d]
            m = (ang >= a0) & (ang < a1)
            med[d] = float(np.median(cur[m])) if m.sum() >= 5 else None
        up, dn = med[+1], med[-1]
        fr = f"{(up - dn) / 2:+9.1f}" if up is not None and dn is not None else f"{'':9s}"
        bi = f"{(up + dn) / 2:+7.1f}" if up is not None and dn is not None else f"{'':7s}"
        f = lambda x: f"{x:+7.1f}" if x is not None else f"{'--':>7s}"
        print(f"  {a0:+6.0f}..{a1:+5.0f} {f(up)} {f(dn)} {fr} {bi}")
    print("\n  friction = (I+ - I-)/2, bias = (I+ + I-)/2. Bare XC330 kinetic: ~20-22 mA.")
    if cap.meta.get("shutdown", {}).get("stopped"):
        print(f"\nSTOPPED EARLY: {cap.meta['shutdown']['stopped']}")


# --------------------------------------------------------------------------
# the runner
# --------------------------------------------------------------------------

def run(args) -> int:
    tee = Tee(sys.stdout)
    sys.stdout = tee
    bus = cap = orig = before = after = None
    torque_on, notes = False, {}
    try:
        if not args.port:
            raise SystemExit("no --port given and AOW_DXL_PORT is unset")
        bus = DynamixelBus(args.port, baud=args.baud, ids=(args.id,)).open()
        name = bus.tables[args.id].name
        if name != MODEL:
            raise SystemExit(f"id {args.id} is a {name}; this test is {MODEL} only")
        errs = bus.hardware_errors()
        if errs:
            raise SystemExit(f"latched hardware error "
                             f"{ {i: describe_hardware_error(e) for i, e in errs.items()} }; "
                             f"find the cause and reboot it first")
        orig = bus.snapshot()
        mode = orig[args.id]["Operating Mode"]
        if mode != MODE_CURRENT_POSITION:
            raise SystemExit(f"id {args.id} is in mode {mode}, not current-based "
                             f"position ({MODE_CURRENT_POSITION}); this script does not "
                             f"write the mode (EEPROM)")
        if args.cap_ma > orig[args.id]["Current Limit"]:
            raise SystemExit(f"--cap-ma {args.cap_ma} > Current Limit "
                             f"{orig[args.id]['Current Limit']}")

        bus.prepare()                           # torque off, Return Delay 0, profiles 0
        bus.write_raw(args.id, "Position P Gain", args.p_gain)
        bus.write_raw(args.id, "Position D Gain", args.d_gain)
        bus.write_raw(args.id, "Position I Gain", 0)
        imap = IndirectMap(bus.tables)
        for n in READ_BLOCK:
            imap.read(n)
        imap.read("Goal Position", label="goal_readback")
        imap.write("Goal Position", label="goal")
        bus.apply_map(imap, verify=True)
        before = bus.snapshot()
        start = bus.read(args.id, "Present Position")
        stow = math.radians(args.stow_deg)
        bus.write_frame({args.id: start})     # torque-on drives to RAM's last goal
        if args.breakaway:
            segs = build_breakaway_plan(args, start, stow)
            reach = max(abs(a) for a in args.breakaway) + max(args.creep_deg, args.approach_deg)
        else:
            segs = build_plan(args, start, stow)
            reach = max(args.amps)
        total = sum(s.seconds for s in segs)
        volts = before[args.id]["Present Input Voltage"] * 0.1
        print(f"\n{TEST}: id {args.id}, supply {volts:.1f} V, P/D {args.p_gain}/{args.d_gain}, "
              f"cap {args.cap_ma:g} mA, {args.speed_dps:g} deg/s.\n"
              f"  now at servo {math.degrees(start):.1f} deg = crank "
              f"{math.degrees(start - stow):+.1f} from stow {args.stow_deg:g}; "
              f"{'breakaway at ' + str(args.breakaway) if args.breakaway else 'amplitudes ' + str(args.amps)}"
              f" deg; reach +-{reach:g}; {len(segs)} segments, {total:.0f} s; "
              f"speed guard {args.max_dps:g} deg/s.")
        if abs(math.degrees(start - stow)) > reach + 3:
            raise SystemExit("it starts outside the planned range; check --stow-deg")
        if args.dry_run:
            for s in segs:
                print(f"  {s.label:30s} {s.seconds:5.1f} s")
            print("  --dry-run: torque never enabled.")
            return 0
        if not args.yes:
            try:
                input(f"\nTorque ON for id {args.id}: the crank moves within "
                      f"+-{reach:g} deg of stow. Enter to start (Ctrl-C aborts): ")
            except EOFError:
                raise SystemExit("no terminal to confirm on; pass --yes") from None
        bus.bus_watchdog(WATCHDOG_MS)
        torque_on = True

        lim = math.radians(reach + 3)
        vmax = math.radians(args.max_dps)
        lag = math.radians(args.lag_deg)
        runaway = math.radians(args.runaway_deg)
        stall_v = math.radians(args.stall_dps)
        state = {"since": None}

        def guard(seg):
            inner = seg.command

            def cmd(t, s):
                goal = inner(t, s) if inner else None
                if s is None:
                    return goal
                st = s[args.id]
                if st["Present Temperature"] >= args.max_temp:
                    notes["stopped"] = f"{st['Present Temperature']:.0f} C"
                    raise RuntimeError(notes["stopped"])
                p = st["Present Position"]
                if abs(st["Present Velocity"]) > vmax:
                    notes["stopped"] = (f"too fast in '{seg.label}': "
                                        f"{math.degrees(st['Present Velocity']):+.0f} deg/s at "
                                        f"crank {math.degrees(p - stow):+.1f}")
                    raise RuntimeError(notes["stopped"])
                if abs(p - stow) > lim:
                    notes["stopped"] = (f"out of range: crank "
                                        f"{math.degrees(p - stow):+.1f} deg")
                    raise RuntimeError(notes["stopped"])
                if goal:
                    e = abs(goal[args.id] - p)
                    if e > runaway and seg.info.get("kind") != "step":
                        notes["stopped"] = (f"{math.degrees(e):.1f} deg behind in "
                                            f"'{seg.label}' at crank {math.degrees(p - stow):+.1f}")
                        raise RuntimeError(notes["stopped"])
                    # LAG ALONE IS NOT A STALL. Position I is 0, so the lag
                    # grows with the current the mechanism takes (~30 mA/deg at
                    # P 700, 2026-10-17): a moving servo sat 6.2 deg behind at
                    # 180 mA and tripped a lag-only guard. A stall is lag WITH
                    # the output stopped, and only while the goal is moving --
                    # at a dwell it legitimately parks short against friction.
                    moving = seg.info.get("kind") in ("leg", "move")
                    if moving and e > lag and abs(st["Present Velocity"]) < stall_v:
                        state["since"] = state["since"] or (seg.label, t)
                        if state["since"][0] == seg.label and t - state["since"][1] > args.lag_s:
                            notes["stopped"] = (f"stalled in '{seg.label}': "
                                                f"{math.degrees(e):.1f} deg behind at crank "
                                                f"{math.degrees(p - stow):+.1f}")
                            raise RuntimeError(notes["stopped"])
                    else:
                        state["since"] = None
                return goal
            seg.command = cmd
            return seg

        def finish(b):
            notes["watchdog_tripped"] = b.watchdog_tripped()
            b.write_frame({i: b.read(i, "Present Position") for i in b.ids})
            b.bus_watchdog(0)

        cap = record(bus, imap, [guard(s) for s in segs], args.rate, finish=finish)
    finally:
        if bus is not None and bus._port is not None:
            if _try(bus.torque, False) is _FAILED:
                print("!! TORQUE OFF FAILED -- cut power to the servos")
            if torque_on:
                _try(bus.bus_watchdog, 0)
            if orig is not None:
                o = orig[args.id]
                _try(lambda: (bus.write_raw(args.id, "Position P Gain", o["Position P Gain"]),
                              bus.write_raw(args.id, "Position D Gain", o["Position D Gain"]),
                              bus.write_raw(args.id, "Position I Gain", o["Position I Gain"])))
                notes["restored"] = True
            after = _try(bus.snapshot)
            bus.close()
        sys.stdout = tee.stream
        if cap is not None:
            cap.meta.update(
                test=TEST, note=args.note, argv=sys.argv, port=args.port, baud=args.baud,
                stow_deg=args.stow_deg, amps=args.amps, speed_dps=args.speed_dps,
                breakaway=args.breakaway, creep_dps=args.creep_dps, max_dps=args.max_dps,
                step=args.step, step_s=args.step_s,
                cap_ma=args.cap_ma, gains={"p": args.p_gain, "d": args.d_gain},
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
    ap.add_argument("--id", type=int, default=104)
    ap.add_argument("--stow-deg", type=float, default=180.0,
                    help="servo angle of the crank's rest pose "
                         "(control.onboard.righting_stow_deg)")
    ap.add_argument("--amps", type=float, nargs="+", default=[30.0],
                    help="sweep amplitudes, crank deg from stow, in order. "
                         "The commanded stroke is 129.3")
    ap.add_argument("--speed-dps", type=float, default=15.0)
    ap.add_argument("--breakaway", type=float, nargs="+", default=None, metavar="DEG",
                    help="instead of sweeping: breakaway current at these crank angles, "
                         "both directions, by a goal CREEP (no steps)")
    ap.add_argument("--step", action="store_true",
                    help="JUMP the goal instead of ramping it, as hw/ground.py does; "
                         "the current cap sets the speed")
    ap.add_argument("--step-s", type=float, default=1.0, help="time per step")
    ap.add_argument("--creep-dps", type=float, default=4.0)
    ap.add_argument("--creep-deg", type=float, default=20.0)
    ap.add_argument("--approach-deg", type=float, default=8.0)
    ap.add_argument("--rest-s", type=float, default=1.0)
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--max-dps", type=float, default=60.0,
                    help="stop if the crank ever turns faster than this")
    ap.add_argument("--dwell-s", type=float, default=0.5)
    ap.add_argument("--cap-ma", type=float, default=150.0,
                    help="Goal Current cap. The bare servo moves at ~25 mA")
    ap.add_argument("--lag-deg", type=float, default=6.0)
    ap.add_argument("--lag-s", type=float, default=0.3)
    ap.add_argument("--stall-dps", type=float, default=3.0,
                    help="below this speed, lag > --lag-deg on a moving leg is a stall")
    ap.add_argument("--runaway-deg", type=float, default=20.0,
                    help="stop on this much lag anywhere, moving or not")
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
    raise SystemExit(main())
