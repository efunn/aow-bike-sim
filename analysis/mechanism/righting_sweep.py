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

--launch replaces it with full-current starts from rest, both ways (see
build_launch_plan): the crank parks at +A and at -A, then its goal jumps
--launch-span further out at --launch-ma. Run with the bike held in the
air, wings clear of everything but the bike itself:

    python analysis/mechanism/righting_sweep.py --port $P --launch 0 10 20 30 \
        --cap-ma 400 --tag launch_air_1 --note "held in the air"

--give prop|lie --side right|left replaces it with THE CRANK'S GIVE under
load (see build_give_plan): the bike sits still -- propped, or lying on
--side -- and the servo pushes UP ONLY (the get-up's way) with a staircase of
Goal Current, 0 -> --give-peak-ma -> 0, the goal parked --give-span ahead.
The crank winds into the play and spring between servo and blade while the
bike stays put; Present Position against current, up and down, is that
curve. Run with run_bike stopped and the bike already in the pose:

    python analysis/mechanism/righting_sweep.py --port $P --give prop --side right \
        --tag give_prop_R --note "propped after a right get-up, run_bike quit"
    python analysis/mechanism/righting_sweep.py --port $P --give lie --side right \
        --tag give_lie_R --note "lying on the right, soft lowered"

--wiggle DEG parks the crank at DEG and wiggles its goal +- each of
--wiggle-amp (1 and 2 deg by default), holding each side --wiggle-dwell-s, to
LOOK at what moves (see build_wiggle_plan):

    python analysis/mechanism/righting_sweep.py --wiggle -15 --wiggle-amp 1 2 3 \
        --cap-ma 300 --tag wiggle_m15 --note "bike in the air"

--wiggle here wiggles about wherever the crank IS when you press Enter: set the
mechanism by hand (torque is off until then), press Enter, and it reads Present
Position, takes that as the centre and holds it, with no move before or after.
Two poses compared this way have no approach direction between them (the
11-08 -66 / +66 pair came in from opposite sides). The crank is left torqued
at the centre at the end, then released.

    python analysis/mechanism/righting_sweep.py --wiggle here --tag wiggle_here_m66 \
        --note "set by hand to ~-66, from stow"

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


def _onboard() -> dict:
    """config/onboard.yaml (the Pi's devices), or {} if it will not load."""
    try:
        import yaml
        return yaml.safe_load((ROOT / "config" / "onboard.yaml").read_text()) or {}
    except Exception:                       # noqa: BLE001 -- only a default
        return {}


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


def build_launch_plan(args, start: float, stow: float) -> list:
    """Full-current starts from rest at each crank angle, both ways.

    THE QUESTION (2026-11-07): the left get-up (crank -) loads up past crank
    -10 where the right does not, and the slow sweeps cannot see it: they
    stopped at 10, at 30 deg/s. Here the servo starts from rest at +A or -A
    the way a lift does: Goal Current at --launch-ma (785, the get-up's)
    and the goal jumped --launch-span OUTWARD (away from stow, the lift's
    direction; from 0, both ways). In the air nothing loads the crank but
    the linkage and the wings' inertia, so at matched A a slower start, or
    more current for the same speed, on one side is that side's mechanism
    -- or the motor, if it is weaker one way. Sides alternate within each
    angle so heat drifts both sides alike.

    Between launches: --cap-ma and a ramp at --speed-dps, as the sweep's.
    """
    v = math.radians(args.speed_dps)
    span = math.radians(args.launch_span)
    hold = lambda g: (lambda t, s: {i: g for i in s} if s is not None else None)
    segs = [Segment("hold where it is", 1.0, hold(start),
                    enter=lambda bus: (set_current(args.cap_ma)(bus), bus.torque(True)),
                    info={"kind": "hold"})]
    here = start
    for rep in range(args.reps):
        for a_deg in args.launch:
            for d in ((+1, -1) if rep % 2 == 0 else (-1, +1)):
                a = stow + d * math.radians(a_deg)
                cmd, T = ramp(here, a, v)
                segs.append(Segment(f"{d * a_deg:+g}: to", T + 0.2, cmd,
                                    info={"kind": "move"}))
                # the warning before the throw: this label prints as the rest begins
                segs.append(Segment(f"{d * a_deg:+g}: rest -- LAUNCH "
                                    f"{'+-'[d < 0]} in {args.rest_s:g} s", args.rest_s,
                                    hold(a), info={"kind": "rest"}))
                end = a + d * span
                segs.append(Segment(
                    f"{d * a_deg:+g}: LAUNCH -> {d * (a_deg + args.launch_span):+g} "
                    f"at {args.launch_ma:g} mA, rep {rep}", args.launch_s, hold(end),
                    enter=set_current(args.launch_ma),
                    info={"kind": "launch", "from_deg": d * a_deg, "dir": d, "rep": rep,
                          "to_deg": d * (a_deg + args.launch_span)}))
                segs.append(Segment(f"{d * a_deg:+g}: settle", 0.3, hold(end),
                                    enter=set_current(args.cap_ma),
                                    info={"kind": "settle"}))
                here = end
    cmd, T = ramp(here, stow, v)
    segs.append(Segment("to stow", T + 0.5, cmd, info={"kind": "move"}))
    segs.append(Segment("hold at stow", 1.0, hold(stow), info={"kind": "hold"}))
    return segs


#: THE OVERLOAD TRIP LAW (B2, 2026-11-03, analysis/servo/righting_overload.py;
#: config/rl/rl_righting.yaml `overload`): held above `floor` the servo trips
#: after charge / (I - floor) s -- 730 mA 2.0 s, 600 3.6, 500 7.0 -- and
#: below it a full charge drains in `cool` s. Replayed over every bike run it
#: called all 7 real overloads 0.04-0.14 s early. Untested under 500 mA.
TRIP_FLOOR_A, TRIP_CHARGE_AS, TRIP_COOL_S = 0.413, 0.645, 0.5


#: Position P Gain -> mA per degree of error (the firmware's K5, params.XC330_K5)
MA_PER_DEG_PER_P = 0.0025464790894703256 * 1000 * math.pi / 180
#: --wiggle runs on the plant's gains (P 700 / D 1400), the only ones tried
#: that never oscillated. Stiffer ones on the bike in the air, 11-08:
#:   P 3000 / D 1400  +-1 deg tracked to 0.1, but an ~18 Hz limit cycle grew
#:                    from the first step (261108-014840)
#:   P 2000 / D 4000  quiet at +-1 deg for 5 poses, then a 23 Hz limit cycle,
#:                    4.6 deg p2p, current cap to cap (261108-015053)
#: Both over the user's ceiling for these servos, P 1500 / D 3000. So P 700
#: sits ~1-3 deg short against the friction: ask more amplitude.
#: No I gain ever: see run().


def heat_step(h: float, amps: float, dt: float) -> float:
    """The trip law's heat (1 = overload) after `dt` s at |`amps`|."""
    a = abs(amps)
    if a > TRIP_FLOOR_A:
        return h + (a - TRIP_FLOOR_A) / TRIP_CHARGE_AS * dt
    return max(0.0, h - dt / TRIP_COOL_S)


def give_currents(args) -> list:
    """The staircase, mA: 0 -> peak -> 0 in --give-step-ma, peak once."""
    n = max(1, int(math.ceil(args.give_peak_ma / args.give_step_ma)))
    up = [min(args.give_peak_ma, k * args.give_step_ma) for k in range(n + 1)]
    return up + up[-2::-1]


def build_give_plan(args, start: float, stow: float, side: int) -> list:
    """The crank's give under load, pushing UP ONLY.

    THE QUESTION (2026-11-08): when a hard-held prop let go (261027-143131,
    ~400 mA ramped to 0), the crank came back ~3 deg with the roll
    unchanged; at ~100 mA, 1-2. And a lift from lying moves the crank 2-3 deg
    on ~100-190 mA before it takes the load. So something between the servo
    and the blade gives: play, a spring, or both.

    Here the bike sits still and the servo pushes the get-up's way only. The
    goal is parked --give-span ahead of the start, UP (+ for a right get-up,
    - for a left; lying on the right the crank rests near -24 and up is
    still +), so Present Current is capped by Goal Current at every step
    (P 700 asks ~31 mA/deg). Goal Current climbs 0 -> --give-peak-ma in
    --give-step-ma and back, --give-step-s a step; the goal never moves
    and never points down. At 0 mA the drive is off, not braking.
    Between reps --rest-s at 0 mA lets the trip law's heat drain.

    The crank's position against current, up then down, is the curve: the
    gap between the legs at low current is the play, the slope the spring.
    It STOPS (holds, then torque off) as soon as the bike itself moves:
    the crank more than --give-max-deg up from its start, or the AHRS
    orientation more than --max-tilt-deg from its start.
    """
    hold = lambda g: (lambda t, s: {i: g for i in s} if s is not None else None)
    goal = start + side * math.radians(args.give_span)
    assert side * (goal - start) > 0, "the goal must point up"
    segs = [Segment("hold where it is, 0 mA", 1.0, hold(start),
                    enter=lambda bus: (set_current(0)(bus), bus.torque(True)),
                    info={"kind": "hold"})]
    cur = give_currents(args)
    peak = cur.index(max(cur))
    for rep in range(args.reps):
        if rep:
            segs.append(Segment(f"rest at 0 mA before rep {rep}", args.rest_s, hold(goal),
                                enter=set_current(0), info={"kind": "rest"}))
        for k, ma in enumerate(cur):
            leg = "up" if k <= peak else "down"
            segs.append(Segment(f"rep {rep} {leg} {ma:4.0f} mA", args.give_step_s, hold(goal),
                                enter=set_current(ma),
                                info={"kind": "give", "ma": ma, "leg": leg, "rep": rep,
                                      "side": side}))
    segs.append(Segment("release, 0 mA", 0.5, hold(goal), enter=set_current(0),
                        info={"kind": "rest"}))
    return segs


def _wiggle_arg(v: str):
    """--wiggle: a crank angle [deg], or `here`."""
    return "here" if v.strip().lower() == "here" else float(v)


def build_wiggle_plan(args, start: float, stow: float) -> list:
    """Park at --wiggle DEG, then wiggle the goal +-A about it, to WATCH.

    THE QUESTION (2026-11-08): --give found ~1.8 deg of the crank's travel
    taken on almost no current and a soft spring after it, and every pin
    hole is reamed to a slip fit -- so where does it bend? Ramp to the angle
    at --speed-dps, rest, then per amplitude in --wiggle-amp, --wiggle-cycles
    cycles: `step` holds +A then -A for --wiggle-dwell-s each (a still pose
    to look at; the cap sets the speed between them), `sine` sweeps
    smoothly at --wiggle-hz. Back to the angle, then to where it started.
    Goal Current is --cap-ma throughout: the load you can push against by
    hand, or the bike's weight, shows as Present Position falling short.
    """
    v = math.radians(args.speed_dps)
    hold = lambda g: (lambda t, s: {i: g for i in s} if s is not None else None)
    here = getattr(args, "wiggle_here", False)
    # --wiggle here: the centre IS where it was set by hand (start), and the
    # plan never moves the crank anywhere else
    at = start if here else stow + math.radians(args.wiggle)
    segs = [Segment("hold where it is", 1.0, hold(start),
                    enter=lambda bus: (set_current(args.cap_ma)(bus), bus.torque(True)),
                    info={"kind": "hold"})]
    if not here:
        # the moves get their own current: from a prop's end of stroke the left
        # would not start back at 300 mA in the air (261108-013838)
        cmd, T = ramp(start, at, v)
        segs.append(Segment(f"to {args.wiggle:+g} at {args.wiggle_move_ma:g} mA", T + 0.3, cmd,
                            enter=set_current(args.wiggle_move_ma), info={"kind": "move"}))
    segs.append(Segment(f"rest at {args.wiggle:+g}, {args.cap_ma:g} mA", args.rest_s, hold(at),
                        enter=set_current(args.cap_ma),
                        info={"kind": "wiggle_rest", "amp_deg": 0.0}))
    for A in args.wiggle_amp:
        a = math.radians(A)
        if args.wiggle_shape == "step":
            for c in range(args.wiggle_cycles):
                for sgn in (+1, -1):
                    segs.append(Segment(f"{args.wiggle:+g} {'+-'[sgn < 0]}{A:g} (cycle {c})",
                                        args.wiggle_dwell_s, hold(at + sgn * a),
                                        info={"kind": "wiggle", "amp_deg": A, "dir": sgn,
                                              "cycle": c}))
        else:
            w = 2.0 * math.pi * args.wiggle_hz

            def sine(t, s, a=a, w=w):
                return {i: at + a * math.sin(w * t) for i in s} if s is not None else None
            segs.append(Segment(f"{args.wiggle:+g} sine +-{A:g} at {args.wiggle_hz:g} Hz",
                                args.wiggle_cycles / args.wiggle_hz, sine,
                                info={"kind": "wiggle_sine", "amp_deg": A}))
        segs.append(Segment(f"back to {args.wiggle:+g}", 0.5, hold(at),
                            info={"kind": "wiggle_rest", "amp_deg": A}))
    if here:
        segs.append(Segment("hold", 0.5, hold(at), info={"kind": "hold"}))
        return segs
    cmd, T = ramp(at, start, v)
    segs.append(Segment("back to the start", T + 0.3, cmd,
                        enter=set_current(args.wiggle_move_ma), info={"kind": "move"}))
    segs.append(Segment("hold", 0.5, hold(start), info={"kind": "hold"}))
    return segs


def summarize_wiggle(cap: Capture) -> None:
    stow = math.radians(cap.meta["stow_deg"])
    i = int(cap.meta["ids"][0])
    pos = np.degrees(cap.position_rad(i) - stow)
    cur = cap.value("Present Current", i) * 1000
    at = cap.meta["wiggle"]
    print(f"\nWIGGLE about crank {at:+g} at Goal Current {cap.meta['cap_ma']:g} mA"
          f"\n  step: per held pose, the second half's median; sine: the extremes reached")
    print(f"  {'pose':>14s} {'goal':>7s} {'crank':>8s} {'short by':>8s} {'I mA':>6s}")
    for s in cap.segments:
        info = s["info"]
        sl = slice(s["start"], s["stop"])
        ok = cap.ok[sl]
        p, c = pos[sl][ok], cur[sl][ok]
        if len(p) < 4:
            continue
        if info.get("kind") in ("wiggle", "wiggle_rest"):
            g = at + info.get("dir", 0) * info["amp_deg"] if info["kind"] == "wiggle" else at
            h = len(p) // 2
            pm, cm = float(np.median(p[h:])), float(np.median(c[h:]))
            tag = (f"{'+-'[info['dir'] < 0]}{info['amp_deg']:g} c{info['cycle']}"
                   if info["kind"] == "wiggle" else "rest")
            print(f"  {tag:>14s} {g:+7.2f} {pm:+8.2f} {abs(g - pm):8.2f} {cm:+6.0f}")
        elif info.get("kind") == "wiggle_sine":
            A = info["amp_deg"]
            print(f"  {'sine +-' + format(A, 'g'):>14s} {'':7s} {p.min() - at:+8.2f}.."
                  f"{p.max() - at:+.2f} about {at:+g} (asked +-{A:g}); I {c.min():+.0f}..{c.max():+.0f}")
    gp = cap.meta.get("gains", {})
    print(f"\n  short by = goal - crank: where a load holds the crank back "
          f"(P {gp.get('p')}: ~{gp.get('p', 0) * MA_PER_DEG_PER_P:.0f} mA/deg)")
    if cap.meta.get("give_heat_peak") is not None:
        print(f"  trip-law heat peak {cap.meta['give_heat_peak']:.2f} (1 = overload)")
    if cap.meta.get("shutdown", {}).get("stopped"):
        print(f"\nSTOPPED EARLY: {cap.meta['shutdown']['stopped']}")


def give_heat_plan(segs) -> float:
    """Worst-case peak heat: every give step drawing its full Goal Current."""
    h = peak = 0.0
    for s in segs:
        ma = s.info.get("ma", 0.0) if s.info.get("kind") == "give" else 0.0
        h = heat_step(h, ma / 1000.0, s.seconds)
        peak = max(peak, h)
    return peak


# --------------------------------------------------------------------------
# analysis
# --------------------------------------------------------------------------

def summarize_give(cap: Capture, out_dir: Path | None = None) -> None:
    stow = math.radians(cap.meta["stow_deg"])
    i = int(cap.meta["ids"][0])
    side = int(cap.meta.get("give_side", 1))
    pos = np.degrees(cap.position_rad(i) - stow)
    cur = cap.value("Present Current", i) * 1000
    tilt = None
    if out_dir is not None and (Path(out_dir) / "ahrs.npz").exists():
        z = np.load(Path(out_dir) / "ahrs.npz")
        tilt = (z["frame"], z["tilt_deg"])
    p0 = None
    rows = {}
    for s in cap.segments:
        info = s["info"]
        sl = slice(s["start"], s["stop"])
        ok = cap.ok[sl]
        if info.get("kind") == "hold" and p0 is None and ok.any():
            p0 = float(np.median(pos[sl][ok]))
        if info.get("kind") != "give":
            continue
        idx = np.arange(s["start"], s["stop"])[ok]
        if len(idx) < 4:
            continue
        last = idx[len(idx) // 2:]            # the step's second half: settled
        tl = None
        if tilt is not None:
            m = np.isin(tilt[0], last)
            tl = float(np.median(tilt[1][m])) if m.any() else None
        rows[(info["rep"], info["leg"], info["ma"])] = (
            float(np.median(pos[last])), float(np.median(cur[last] * side)), tl)
    if not rows or p0 is None:
        print("\nno give steps recorded")
        return
    print(f"\nGIVE -- {cap.meta.get('give')}, {'right' if side > 0 else 'left'} side; "
          f"crank at the start {p0:+.1f} deg from stow.\n"
          f"  per step, the second half's median: crank UP from the start [deg], "
          f"Present Current [mA], AHRS tilt from the start [deg]")
    for rep in sorted({r for r, _, _ in rows}):
        steps = sorted({ma for r, _, ma in rows if r == rep})
        print(f"\n  rep {rep}   {'Goal mA':>7s}  {'up: crank':>9s} {'I':>5s} {'tilt':>5s}"
              f"   {'down: crank':>11s} {'I':>5s} {'tilt':>5s}")
        f = lambda v, fmt: fmt % v if v is not None else " " * (len(fmt % 0.0))
        for ma in steps:
            u, d = rows.get((rep, "up", ma)), rows.get((rep, "down", ma))
            cell = lambda x: ((f"{side * (x[0] - p0):+9.2f}", f"{x[1]:5.0f}", f(x[2], "%5.2f"))
                              if x else (" " * 9, " " * 5, " " * 5))
            a, b = cell(u), cell(d)
            print(f"          {ma:7.0f}  {a[0]} {a[1]} {a[2]}   {b[0]:>11s} {b[1]} {b[2]}")
    print("\n  play ~ the gap between up and down at the same Goal Current near 0;"
          "\n  spring ~ the up leg's slope, deg per 100 mA. Present Current is the"
          "\n  servo's own (bus) reading; Goal Current is its cap.")
    if cap.meta.get("shutdown", {}).get("stopped"):
        print(f"\nSTOPPED EARLY: {cap.meta['shutdown']['stopped']}")
    if cap.meta.get("give_heat_peak") is not None:
        print(f"\ntrip-law heat: planned worst case {cap.meta.get('give_heat_plan', 0):.2f}, "
              f"measured peak {cap.meta['give_heat_peak']:.2f} (1 = overload)")


LAUNCH_MARKS = (5.0, 10.0, 15.0, 20.0)


def launch_trials(cap: Capture) -> list:
    """[{from, dir, rep, t_at: {deg: s}, v_at: {deg: deg/s}, peak, i_mean, pwm}]
    per launch, travel measured outward from where it started."""
    stow = math.radians(cap.meta["stow_deg"])
    i = int(cap.meta["ids"][0])
    pos = np.degrees(cap.position_rad(i) - stow)
    cur = cap.value("Present Current", i) * 1000
    vel = np.degrees(cap.value("Present Velocity", i))
    pwm = cap.value("Present PWM", i)
    t_all = cap.time_s(i)
    out = []
    for s in cap.segments:
        info = s["info"]
        if info.get("kind") != "launch":
            continue
        sl = slice(s["start"], s["stop"])
        ok = cap.ok[sl]
        p, c, w, u, t = pos[sl][ok], cur[sl][ok], vel[sl][ok], pwm[sl][ok], t_all[sl][ok]
        if len(p) < 5:
            continue
        d = info["dir"]
        trav, t = (p - p[0]) * d, t - t[0]
        # the crank parks short of a ramp's goal (P only, Position I 0: ~3.5
        # deg at 400 mA in the air, 11-07), so a launch starts where it parked
        row = dict(frm=info["from_deg"], dir=d, rep=info["rep"], t_at={}, v_at={},
                   start=float(p[0]), dist=abs(info["to_deg"] - float(p[0])),
                   peak=float(np.max(w * d)), temp=None)
        for mk in LAUNCH_MARKS:
            k = np.flatnonzero(trav >= mk)
            row["t_at"][mk] = float(t[k[0]]) if len(k) else None
            row["v_at"][mk] = float(w[k[0]] * d) if len(k) else None
        k15 = np.flatnonzero(trav >= 15.0)
        n = k15[0] + 1 if len(k15) else len(p)
        row["i_mean"] = float(np.mean(c[:n] * d))
        row["pwm"] = float(np.mean(np.abs(u[:n])))
        out.append(row)
    return out


def summarize_launch(cap: Capture) -> None:
    rows = launch_trials(cap)
    if not rows:
        print("\nno launches recorded")
        return
    span = cap.meta.get("launch_span")
    print(f"\nLAUNCHES at {cap.meta.get('launch_ma'):g} mA from rest, {span:g} deg outward"
          f"\n  (+ = right wing down / right get-up; - = left). Travel from the start.")
    hdr = "  ".join(f"t{m:g}" for m in LAUNCH_MARKS)
    print(f"  {'from':>5s} {'rep':>3s} {'start':>6s} {'to go':>5s}  {hdr}   v@10   v@20  peak  I(0-15)  PWM")
    f = lambda x, fmt: (fmt % x) if x is not None else "   --"
    for r in sorted(rows, key=lambda r: (abs(r["frm"]), -r["dir"], r["rep"])):
        ts = "  ".join(f(None if r["t_at"][m] is None else r["t_at"][m] * 1000, "%3.0f")
                       for m in LAUNCH_MARKS)
        print(f"  {r['frm']:+5.0f} {r['rep']:3d} {r['start']:+6.1f} {r['dist']:5.1f}  {ts}  {f(r['v_at'][10], '%5.0f')}  "
              f"{f(r['v_at'][20], '%5.0f')} {r['peak']:5.0f}  {r['i_mean']:+7.0f}  {r['pwm']:.2f}")
    print("\n  t5..t20 = ms to 5..20 deg of travel; v@ = deg/s there; I = mean Present Current"
          "\n  over the first 15 deg, in the direction of travel")
    print("\nRIGHT - LEFT at matched start (median over reps; + = the left is slower / draws more)")
    print(f"  {'|from|':>6s}  {'dt20 ms':>7s}  {'dv@10':>6s}  {'dv@20':>6s}  {'dI mA':>6s}")
    for a in sorted({abs(r["frm"]) for r in rows}):
        side = {d: [r for r in rows if abs(r["frm"]) == a and r["dir"] == d] for d in (1, -1)}
        if not side[1] or not side[-1]:
            continue
        med = lambda rs, g: (float(np.median([g(r) for r in rs if g(r) is not None]))
                             if any(g(r) is not None for r in rs) else None)
        diff = []
        for g, scale, sign in ((lambda r: r["t_at"][20.0], 1000, 1), (lambda r: r["v_at"][10.0], 1, -1),
                               (lambda r: r["v_at"][20.0], 1, -1), (lambda r: r["i_mean"], 1, 1)):
            R, L = med(side[1], g), med(side[-1], g)
            diff.append(None if R is None or L is None else sign * (L - R) * scale)
        print(f"  {a:6.0f}  " + "  ".join(f(x, "%+6.0f") for x in diff))
    if cap.meta.get("shutdown", {}).get("stopped"):
        print(f"\nSTOPPED EARLY: {cap.meta['shutdown']['stopped']}")

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


def summarize(cap: Capture, bin_deg: float = 15.0, out_dir: Path | None = None) -> None:
    if cap.meta.get("give"):
        summarize_give(cap, out_dir)
        return
    if cap.meta.get("wiggle") is not None:
        summarize_wiggle(cap)
        return
    if cap.meta.get("launch"):
        summarize_launch(cap)
        return
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
    bus = cap = orig = before = after = ahrs = None
    torque_on, notes = False, {}
    side = {"right": 1, "left": -1}.get(args.side or "", 0)
    tilt_log = {"frame": [], "tilt_deg": [], "t": []}
    heat = {"h": 0.0, "peak": 0.0, "plan": None}
    try:
        if args.give and not side:
            raise SystemExit("--give needs --side right|left: the side the bike is "
                             "propped up from, or lying on")
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
        if args.launch and args.launch_ma > orig[args.id]["Current Limit"]:
            raise SystemExit(f"--launch-ma {args.launch_ma} > Current Limit "
                             f"{orig[args.id]['Current Limit']}")
        if args.give and args.give_peak_ma > orig[args.id]["Current Limit"]:
            raise SystemExit(f"--give-peak-ma {args.give_peak_ma} > Current Limit "
                             f"{orig[args.id]['Current Limit']}")
        if args.cap_ma > orig[args.id]["Current Limit"]:
            raise SystemExit(f"--cap-ma {args.cap_ma} > Current Limit "
                             f"{orig[args.id]['Current Limit']}")

        bus.prepare()                           # torque off, Return Delay 0, profiles 0
        bus.write_raw(args.id, "Position P Gain", args.p_gain)
        bus.write_raw(args.id, "Position D Gain", args.d_gain)
        # NEVER anything but 0: on this firmware writing I back to 0 does not
        # clear the accumulated error (ROBOTIS; their V54 fix is XL330 only,
        # unconfirmed for the XC330), so a nonzero I leaves windup until reboot
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
        def plan_wiggle(start):
            """The wiggle's segments and reach from `start`; for --wiggle here
            the centre is `start` itself."""
            if args.wiggle_here:
                args.wiggle = round(math.degrees(start - stow), 2)
            segs = build_wiggle_plan(args, start, stow)
            reach = max(abs(args.wiggle) + max(args.wiggle_amp), abs(math.degrees(start - stow)))
            # The wiggle can sit against a load by design: refuse a --cap-ma
            # whose whole wiggle, stalled, would trip. The moves at
            # --wiggle-move-ma are guarded live instead -- the stall guard
            # (0.3 s stuck) and the trip law on Present Current (--heat-stop).
            wig = sum(s.seconds for s in segs if s.info.get("kind", "").startswith("wiggle"))
            worst = max(0.0, args.cap_ma / 1000 - TRIP_FLOOR_A) / TRIP_CHARGE_AS * wig
            heat["plan"] = worst
            if worst > args.heat_max:
                raise SystemExit(f"--cap-ma {args.cap_ma:g} held stalled through the wiggle "
                                 f"could reach {worst:.2f} of an overload trip (limit "
                                 f"--heat-max {args.heat_max:g}): keep it under ~410 mA")
            return segs, reach

        if args.give:
            # the pose must match: propped, the crank is well up on its side;
            # lying, it rests past centre the OTHER way (right: ~ -24)
            up = side * math.degrees(start - stow)
            lo, hi = (55.0, 125.0) if args.give == "prop" else (-60.0, 5.0)
            if not lo <= up <= hi:
                raise SystemExit(
                    f"--give {args.give} --side {args.side}: the crank is at "
                    f"{math.degrees(start - stow):+.1f} from stow ({up:+.1f} the get-up's "
                    f"way), outside {lo:+.0f}..{hi:+.0f} for that pose. Wrong --side, "
                    f"or the bike is not in the pose.")
            segs = build_give_plan(args, start, stow, side)
            reach = abs(math.degrees(start - stow)) + args.give_span
            heat["plan"] = give_heat_plan(segs)
            if heat["plan"] > args.heat_max:
                raise SystemExit(
                    f"the staircase could reach {heat['plan']:.2f} of an overload trip if "
                    f"every step stalled at its Goal Current (limit --heat-max "
                    f"{args.heat_max:g}): lower --give-peak-ma or --give-step-s")
        elif args.wiggle is not None:
            segs, reach = plan_wiggle(start)
            if args.wiggle_move_ma > orig[args.id]["Current Limit"]:
                raise SystemExit(f"--wiggle-move-ma {args.wiggle_move_ma} > Current Limit")
        elif args.launch:
            segs = build_launch_plan(args, start, stow)
            reach = max(args.launch) + args.launch_span
        elif args.breakaway:
            segs = build_breakaway_plan(args, start, stow)
            reach = max(abs(a) for a in args.breakaway) + max(args.creep_deg, args.approach_deg)
        else:
            segs = build_plan(args, start, stow)
            reach = max(args.amps)
        total = sum(s.seconds for s in segs)
        volts = before[args.id]["Present Input Voltage"] * 0.1
        print(f"\n{TEST}: id {args.id}, supply {volts:.1f} V, P/D {args.p_gain}/{args.d_gain} ({args.p_gain * MA_PER_DEG_PER_P:.0f} mA/deg), "
              f"cap {args.cap_ma:g} mA, {args.speed_dps:g} deg/s.\n"
              f"  now at servo {math.degrees(start):.1f} deg = crank "
              f"{math.degrees(start - stow):+.1f} from stow {args.stow_deg:g}; "
              f"{('wiggle about where the crank is at the Enter by +-' + str(args.wiggle_amp)) if args.wiggle_here else ('wiggle about ' + format(args.wiggle, '+g') + ' by +-' + str(args.wiggle_amp)) if args.wiggle is not None else 'give, ' + args.give + ' on the ' + args.side + f', 0 -> {args.give_peak_ma:g} -> 0 mA, goal +{args.give_span:g}' if args.give else 'launches from +-' + str(args.launch) + f' at {args.launch_ma:g} mA' if args.launch else 'breakaway at ' + str(args.breakaway) if args.breakaway else 'amplitudes ' + str(args.amps)}"
              f" deg; reach +-{reach:g}; {len(segs)} segments, {total:.0f} s; "
              f"speed guard {args.max_dps:g} deg/s.")
        if args.give:
            print(f"  give: goal parked {args.give_span:g} deg UP; steps of "
                  f"{args.give_step_ma:g} mA, {args.give_step_s:g} s each, {args.reps} rep(s); "
                  f"stops if the crank goes {args.give_max_deg:g} deg up or the bike tilts "
                  f"{args.max_tilt_deg:g} deg. Overload: worst case {heat['plan']:.2f} of a trip "
                  f"(stops at {args.heat_stop:g}).")
        if abs(math.degrees(start - stow)) > reach + 3:
            raise SystemExit("it starts outside the planned range; check --stow-deg")
        if args.dry_run:
            for s in segs:
                print(f"  {s.label:30s} {s.seconds:5.1f} s")
            print("  --dry-run: torque never enabled.")
            return 0
        if args.give and not args.no_ahrs:
            from aow_sim.hw.ahrs import AhrsReader
            ahrs = AhrsReader(args.ahrs_port)
            ahrs.start()
            ahrs.wait_ready(2.0)
            heat["q0"] = np.asarray(ahrs.latest(0.1).quat, float)
        if not args.yes:
            try:
                input(f"\nTorque ON for id {args.id}: " + (
                      f"set the crank by hand now; it holds where it is at the Enter and "
                      f"moves +-{max(args.wiggle_amp):g} deg about it" if args.wiggle_here else
                      f"the crank moves within +-{reach:g} deg of stow") +
                      ". Enter to start (Ctrl-C aborts): ")
            except EOFError:
                raise SystemExit("no terminal to confirm on; pass --yes") from None
            if ahrs is not None:           # the reference is the pose at the go
                heat["q0"] = np.asarray(ahrs.latest(0.1).quat, float)
        if args.wiggle_here:
            # set by hand while the prompt waited: the centre is where it is NOW
            start = bus.read(args.id, "Present Position")
            bus.write_frame({args.id: start})
            segs, reach = plan_wiggle(start)
            print(f"  wiggle here: centre crank {args.wiggle:+.2f} (servo "
                  f"{math.degrees(start):.2f}), read at the Enter")
        bus.bus_watchdog(WATCHDOG_MS)
        torque_on = True

        # a launch arrives at full speed and overshoots its goal by a few deg
        lim = math.radians(reach + (args.launch_margin if args.launch else 3))
        vmax = math.radians(args.max_dps)
        lag = math.radians(args.lag_deg)
        runaway = math.radians(args.runaway_deg)
        stall_v = math.radians(args.stall_dps)
        state = {"since": None, "frame": -1, "clock": None, "stale": 0}
        give_max = math.radians(args.give_max_deg)
        import time as _time

        def heat_check(seg, st):
            """The trip law run on Present Current; stop before an overload."""
            now = _time.perf_counter()
            dt = 0.0 if state["clock"] is None else now - state["clock"]
            state["clock"] = now
            heat["h"] = heat_step(heat["h"], st["Present Current"], dt)
            heat["peak"] = max(heat["peak"], heat["h"])
            if heat["h"] > args.heat_stop:
                notes["stopped"] = (f"overload risk: trip-law heat {heat['h']:.2f} in "
                                    f"'{seg.label}'")
                raise RuntimeError(notes["stopped"])

        def give_checks(seg, st):
            """--give only: the crank's travel up, the tilt."""
            now = _time.perf_counter()
            moved = side * (st["Present Position"] - start)
            if moved > give_max:
                notes["stopped"] = (f"the crank went {math.degrees(moved):.1f} deg up in "
                                    f"'{seg.label}' (--give-max-deg {args.give_max_deg:g}): "
                                    f"the bike is moving")
                raise RuntimeError(notes["stopped"])
            if ahrs is None:
                return
            try:
                q = np.asarray(ahrs.latest(0.1).quat, float)
                state["stale"] = 0
            except RuntimeError:
                state["stale"] += 1
                if state["stale"] > 40:
                    notes["stopped"] = "AHRS stale for 0.2 s"
                    raise RuntimeError(notes["stopped"]) from None
                return
            c = abs(float(np.dot(q, heat["q0"]))) / (np.linalg.norm(q) * np.linalg.norm(heat["q0"]))
            tilt = math.degrees(2.0 * math.acos(min(1.0, c)))
            tilt_log["frame"].append(state["frame"])
            tilt_log["tilt_deg"].append(tilt)
            tilt_log["t"].append(now)
            if tilt > args.max_tilt_deg:
                notes["stopped"] = (f"the bike tilted {tilt:.1f} deg in '{seg.label}' "
                                    f"(--max-tilt-deg {args.max_tilt_deg:g}): it is moving")
                raise RuntimeError(notes["stopped"])

        def guard(seg):
            inner = seg.command

            def cmd(t, s):
                state["frame"] += 1           # one call per captured frame
                goal = inner(t, s) if inner else None
                if s is None:
                    return goal
                st = s[args.id]
                if args.give or args.wiggle is not None:
                    heat_check(seg, st)
                if args.give:
                    give_checks(seg, st)
                if st["Present Temperature"] >= args.max_temp:
                    notes["stopped"] = f"{st['Present Temperature']:.0f} C"
                    raise RuntimeError(notes["stopped"])
                p = st["Present Position"]
                # a launch and a wiggle step move fast by design
                free = seg.info.get("kind") in ("launch", "settle", "wiggle", "wiggle_sine")
                if abs(st["Present Velocity"]) > vmax and not free:
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
                    # a give parks its goal --give-span ahead on purpose
                    parked = ("give", "rest") if args.give else ()
                    if e > runaway and seg.info.get("kind") not in ("step", "launch", "settle",
                                                                   *parked):
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
        if ahrs is not None:
            _try(ahrs.stop)
        sys.stdout = tee.stream
        if cap is not None:
            cap.meta.update(
                test=TEST, note=args.note, argv=sys.argv, port=args.port, baud=args.baud,
                stow_deg=args.stow_deg, amps=args.amps, speed_dps=args.speed_dps,
                breakaway=args.breakaway, creep_dps=args.creep_dps, max_dps=args.max_dps,
                launch=args.launch, launch_ma=args.launch_ma, launch_span=args.launch_span,
                launch_s=args.launch_s,
                wiggle=args.wiggle, wiggle_amp=args.wiggle_amp,
                wiggle_shape=args.wiggle_shape, wiggle_cycles=args.wiggle_cycles,
                wiggle_dwell_s=args.wiggle_dwell_s, wiggle_hz=args.wiggle_hz,
                wiggle_move_ma=args.wiggle_move_ma, wiggle_here=args.wiggle_here,
                give=args.give, give_side=side, give_span=args.give_span,
                give_peak_ma=args.give_peak_ma, give_step_ma=args.give_step_ma,
                give_step_s=args.give_step_s, give_max_deg=args.give_max_deg,
                max_tilt_deg=args.max_tilt_deg, give_ahrs=ahrs is not None,
                give_heat_plan=heat["plan"],
                give_heat_peak=heat["peak"] if (args.give or args.wiggle is not None) else None,
                step=args.step, step_s=args.step_s,
                cap_ma=args.cap_ma, gains={"p": args.p_gain, "d": args.d_gain},
                git=git_state(ROOT), python=sys.version.split()[0],
                platform=platform.platform(), watchdog_ms=WATCHDOG_MS,
                snapshot_initial=orig, snapshot_before=before,
                snapshot_after=None if after is _FAILED else after, shutdown=notes)
            out = new_capture_dir(Path(args.out), TEST, args.tag)
            cap.save(out)
            if tilt_log["frame"]:
                # AHRS orientation change from the start, by captured frame
                np.savez(out / "ahrs.npz", **{k: np.asarray(v) for k, v in tilt_log.items()})
            sys.stdout = tee
            print(f"\nsaved -> {out.relative_to(ROOT) if out.is_relative_to(ROOT) else out}")
            try:
                summarize(cap, out_dir=out)
            except Exception as e:              # noqa: BLE001 -- the capture is saved
                print(f"\n(summary failed: {type(e).__name__}: {e}; the capture is intact)")
            sys.stdout = tee.stream
            (out / "log.txt").write_text("".join(tee.lines))
    return 0 if cap is not None and cap.meta["complete"] else 1


def main() -> int:
    if len(sys.argv) > 2 and sys.argv[1] == "summary":
        summarize(Capture.load(sys.argv[2]), out_dir=Path(sys.argv[2]))
        return 0
    g = righting_gains()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # AOW_DXL_PORT (a bench adapter), else the bike's own (onboard.yaml dxl_port)
    ap.add_argument("--port", default=os.environ.get("AOW_DXL_PORT") or _onboard().get("dxl_port"))
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
    ap.add_argument("--launch", type=float, nargs="+", default=None, metavar="DEG",
                    help="instead of sweeping: from rest at +DEG and -DEG, jump the goal "
                         "--launch-span outward at --launch-ma (bike held in the air)")
    ap.add_argument("--launch-ma", type=float, default=785.0,
                    help="Goal Current for a launch: the get-up's 785 by default")
    ap.add_argument("--launch-span", type=float, default=25.0,
                    help="how far past its start a launch's goal is [crank deg]")
    ap.add_argument("--launch-s", type=float, default=0.6, help="time per launch")
    ap.add_argument("--launch-margin", type=float, default=10.0,
                    help="overshoot allowed past the furthest launch goal before a stop")
    ap.add_argument("--wiggle", type=_wiggle_arg, default=None, metavar="DEG|here",
                    help="instead of sweeping: park at crank DEG and wiggle +- "
                         "--wiggle-amp about it, to watch what moves. `here`: about "
                         "wherever the crank is at the Enter (set it by hand), no moves")
    ap.add_argument("--wiggle-amp", type=float, nargs="+", default=[2.0, 4.0],
                    help="wiggle amplitudes [crank deg], in order. At P 700 the crank "
                         "sits ~1-3 deg short of each pose against the friction")
    ap.add_argument("--wiggle-shape", choices=("step", "sine"), default="step",
                    help="step: hold +A then -A (a still pose to look at); sine: smooth")
    ap.add_argument("--wiggle-cycles", type=int, default=3)
    ap.add_argument("--wiggle-dwell-s", type=float, default=1.5,
                    help="step: how long each pose is held")
    ap.add_argument("--wiggle-hz", type=float, default=0.5, help="sine: frequency")
    ap.add_argument("--wiggle-move-ma", type=float, default=785.0,
                    help="Goal Current for the ramps to the angle and back: the get-up's "
                         "785, so the goal ramp sets the motion, not the cap (the wiggle "
                         "itself runs at --cap-ma). Guarded live by the stall guard and "
                         "the trip law")
    ap.add_argument("--give", choices=("prop", "lie"), default=None,
                    help="instead of sweeping: the crank's give with the bike propped or "
                         "lying on --side, pushing UP only (see build_give_plan)")
    ap.add_argument("--side", choices=("right", "left"), default=None,
                    help="--give: the side propped up from / lying on")
    ap.add_argument("--give-span", type=float, default=20.0,
                    help="how far UP the goal is parked [crank deg]; ~31 mA/deg, so 20 "
                         "keeps Goal Current the cap up to ~620 mA")
    ap.add_argument("--give-peak-ma", type=float, default=400.0,
                    help="top of the staircase. 261027-143131 held props at ~400-430 mA "
                         "without moving; a lift from lying moved at ~400-460")
    ap.add_argument("--give-step-ma", type=float, default=25.0)
    ap.add_argument("--give-step-s", type=float, default=0.4)
    ap.add_argument("--give-max-deg", type=float, default=8.0,
                    help="stop if the crank goes this far up from its start")
    ap.add_argument("--max-tilt-deg", type=float, default=2.0,
                    help="stop if the AHRS orientation moves this far from the start")
    ap.add_argument("--ahrs-port", default=_onboard().get("ahrs_port", "/dev/serial0"))
    ap.add_argument("--no-ahrs", action="store_true",
                    help="--give without the AHRS: no tilt log or tilt stop")
    ap.add_argument("--heat-max", type=float, default=0.5,
                    help="refuse a staircase whose worst case (every step stalled at its "
                         "Goal Current) reaches this fraction of an overload trip")
    ap.add_argument("--heat-stop", type=float, default=0.8,
                    help="stop when the trip law, run on Present Current, reaches this")
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
    ap.add_argument("--lag-deg", type=float, default=None,
                    help="stall: this far behind a moving goal and not moving "
                         "(default 6; --wiggle 15, so its move can build the current to "
                         "break free of a prop's end at P 700)")
    ap.add_argument("--lag-s", type=float, default=0.3)
    ap.add_argument("--stall-dps", type=float, default=3.0,
                    help="below this speed, lag > --lag-deg on a moving leg is a stall")
    ap.add_argument("--runaway-deg", type=float, default=20.0,
                    help="stop on this much lag anywhere, moving or not")
    ap.add_argument("--p-gain", type=int, default=None,
                    help="Position P Gain (default: the plant's, 700 = ~31 mA/deg). "
                         "Stiffer oscillated on the bike: see the note by MA_PER_DEG_PER_P")
    ap.add_argument("--d-gain", type=int, default=None,
                    help="Position D Gain (default: the plant's, 1400)")
    ap.add_argument("--rate", type=float, default=200.0)
    ap.add_argument("--max-temp", type=int, default=45)
    ap.add_argument("--out", default=str(ROOT / "traces" / TEST))
    ap.add_argument("--tag", default=None)
    ap.add_argument("--note", default="")
    ap.add_argument("--yes", action="store_true", help="skip the Enter prompt")
    ap.add_argument("--dry-run", action="store_true",
                    help="open the bus, print the schedule, never enable torque")
    args = ap.parse_args()
    args.wiggle_here = args.wiggle == "here"
    if args.wiggle_here:
        args.wiggle = 0.0                    # replaced by the crank's angle at setup
    base = {"p": g["p"], "d": g["d"]}
    for k in ("p", "d"):
        if getattr(args, f"{k}_gain") is None:
            setattr(args, f"{k}_gain", base[k])
    if args.lag_deg is None:
        # --wiggle's move off a prop's end needs ~10-13 deg of lag at P 700
        # to break free (261108-013838); 15 is ~470 mA, under --runaway-deg
        args.lag_deg = 15.0 if args.wiggle is not None else 6.0
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
