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
    python analysis/servo_lift.py --port $P --mass-g 137 --radius-mm 44 --note "..."
    python analysis/servo_lift.py summary traces/servo_lift/<capture dir>
    python analysis/servo_lift.py fit traces/servo_lift/<dir> <dir> ...  # loaded runs

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
                                  MODE_POSITION, MODE_PWM, DynamixelBus,
                                  IndirectMap, describe_hardware_error)
from aow_sim.params import DEFAULT_PARAMS  # noqa: E402
from servo_breakaway import (_FAILED, MODEL, READ_BLOCK, Tee, _try)  # noqa: E402
import yaml  # noqa: E402

TEST = "servo_lift"
# The Bus Watchdog here. servo_breakaway's 100 ms tripped twice on the Pi 3,
# ~215 s and ~239 s into a capture (2026-09-26): one ~167 ms host stall each,
# which bench_log.record now prevents by turning the cyclic GC off for the
# capture (see there). This is the margin if something else stalls.
LIFT_WATCHDOG_MS = 300
G = 9.81

# WHAT CAPS THE DRIVE, per --mode. `current` is the righting servo's mode (5):
# Goal Current [mA] caps a current loop. `position` is the steering's (3; the
# bike's 4 is the same loop, multi-turn): the PID's output IS the duty, and
# Goal PWM [LSB, 885 = 100 %] caps it -- RAM, writable with torque on, so a
# catch is still "raise the cap". `pwm` (16) has no position loop at all:
# Goal PWM is the drive, for the free-spin sweep only.
MODES = {
    "current":  {"op": MODE_CURRENT_POSITION, "reg": "Goal Current", "scale": 1e-3,
                 "unit": "mA", "home": 300, "gains": "righting",
                 "levels": [*range(0, 202, 2), 225, 250]},
    "position": {"op": MODE_POSITION, "reg": "Goal PWM", "scale": 1 / 885, "unit": "LSB",
                 "home": 885, "gains": "steer", "levels": list(range(0, 402, 4))},
    "pwm":      {"op": MODE_PWM, "reg": "Goal PWM", "scale": 1 / 885, "unit": "LSB",
                 "home": 0, "gains": None,
                 "levels": [0, 10, 20, 30, 40, 60, 80, 100, 150, 200, 300, 450, 600, 885]},
}
CAP = MODES["current"]


def config_gains(role: str) -> dict:
    raw = yaml.safe_load(Path(DEFAULT_PARAMS).read_text())
    g = raw["control"]["onboard"]["gains"][role]
    return {"p": int(g["Position P Gain"]), "i": int(g["Position I Gain"]),
            "d": int(g["Position D Gain"])}


def write_cap(bus, v: float) -> None:
    for i in bus.ids:
        bus.write(i, CAP["reg"], v * CAP["scale"])


def set_cap(v: float):
    """A Segment.enter: write the mode's cap register on every servo and read
    it back -- a SyncWrite reports nothing."""
    def enter(bus):
        for i in bus.ids:
            bus.write(i, CAP["reg"], v * CAP["scale"])
            got = bus.read_raw(i, CAP["reg"])
            if got != (round(v) & 0xFFFF):
                raise RuntimeError(f"id {i}: wrote {CAP['reg']} {v:g}, read {got}")
    return enter


def build_plan(args, level: dict, ctx: dict, dirs=(+1, -1)) -> list:
    """Every segment of the run. `level` is each servo's level angle [rad];
    `ctx` carries the bus (set once it is open) and the rescue log."""
    rng = np.random.default_rng(args.seed)
    step = math.radians(args.step_deg)
    guard = math.radians(args.guard_deg)
    fall = math.radians(args.fall_deg)

    def rescue(k_seg, why):
        bus = ctx["bus"]
        try:
            write_cap(bus, args.home_ma)
        except RuntimeError as e:
            # 2026-09-26: a catch's Goal Current write came back err 7
            # (access error) -- what a goal write gets once the Bus Watchdog
            # has tripped -- with no gap in traffic to explain a trip. Clear
            # it, catch the load, re-arm, and say so rather than die.
            bus.bus_watchdog(0)
            write_cap(bus, args.home_ma)
            bus.bus_watchdog(LIFT_WATCHDOG_MS)
            why += f"; first write refused ({e}), watchdog cleared and re-armed"
        bus.torque(True)
        ctx["rescues"].append({"segment": k_seg, "why": why})
        ctx["caught"] = k_seg

    def guarded(k_seg, goal, sgn=None, fall_rad=None):
        """command(t, s): `goal` until a guard trips, then level. `sgn` +-1:
        catch a fall against a goal that way; 0: catch --fall-deg of motion
        either way; None: the level guard only. `fall_rad` overrides
        --fall-deg for this segment."""
        lim = fall if fall_rad is None else fall_rad

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
                    if sgn is not None and (abs(d) if sgn == 0 else -d * sgn) > lim:
                        rescue(k_seg, f"id {i} fell {math.degrees(p - ctx['start'][k_seg][i]):+.1f} deg")
                        return dict(level)
                if k_seg not in ctx["start"]:
                    ctx["start"][k_seg] = {i: s[i]["Present Position"] for i in s}
            return goal
        return cmd

    def home_enter(bus):
        set_cap(args.home_ma)(bus)
        bus.torque(True)

    segs = []

    def add(label, seconds, goal, enter, info, sgn=None, fall_rad=None):
        k = len(segs)
        segs.append(Segment(label, seconds, guarded(k, goal, sgn, fall_rad),
                            enter=enter, info=info))

    add("settle at level", 1.5, dict(level), home_enter, {"kind": "settle"})
    add(f"hold level, {args.home_ma:g} {CAP['unit']}", 1.0, dict(level), None,
        {"kind": "hold", "ma": args.home_ma})
    if args.freespin:
        # FREE SPIN (mode 16, a bare servo): Goal PWM IS the drive; step it up
        # then down, each way, and log the steady speed. No goals, no guard.
        del segs[:]
        first = True
        for sgn in (+1, -1):
            lv = [v for v in args.currents if v > 0]
            for leg, order in (("up", lv), ("down", lv[::-1])):
                for v in order:
                    e = set_cap(sgn * v)
                    enter = (lambda bus, e=e: (e(bus), bus.torque(True))) if first else e
                    first = False
                    segs.append(Segment(f"spin {'+' if sgn > 0 else '-'}{v:g} {leg}",
                                        args.spin_s, None, enter=enter,
                                        info={"kind": "spin", "leg": leg, "pwm": sgn * v}))
            segs.append(Segment("spin stop", 1.5, None, enter=set_cap(0),
                                info={"kind": "spin_stop"}))
        return segs
    if args.steps:
        # STEP RESPONSE: from level, goal +-size at the home cap, hold, back.
        # Both directions -- with a load on, one lifts and one lowers.
        for rep in range(args.reps):
            for sgn in (+1, -1):
                for size in args.step_sizes:
                    goal = {i: level[i] + sgn * math.radians(size) for i in level}
                    info = {"rep": rep, "dir": sgn, "size_deg": size}
                    add("home", 1.0, dict(level), home_enter, {"kind": "home", **info})
                    add(f"step {'+' if sgn > 0 else '-'}{size:g} deg rep {rep}", args.step_s,
                        goal, None, {"kind": "step", **info})
        add("end: level", 1.0, dict(level), home_enter, {"kind": "home"})
        return segs
    if args.pwm_sweep:
        # PWM SWEEP: the lever is on its stop with the load pulling it there;
        # push into the stop (goal 10 deg past it) at each Goal Current, up
        # then down, and read the drive the firmware actually applies. No
        # level guard: the lever never leaves the stop.
        del segs[:]                             # not the settle at level: stay on the stop
        into = {i: ctx["present"][i] + ctx["stop_side"] * math.radians(10) for i in level}
        up = list(args.pwm_currents)
        first = True
        for leg, order in (("up", up), ("down", up[::-1])):
            for ma in order:
                enter = (lambda bus, m=ma: (set_cap(m)(bus), bus.torque(True))) if first \
                    else set_cap(ma)
                first = False
                segs.append(Segment(f"pwm {leg} {ma:g} mA", args.pwm_hold_s,
                                    lambda t, s, g=into: g, enter=enter,
                                    info={"kind": "pwm", "leg": leg, "ma": ma}))
        return segs
    if args.release:
        # RELEASE: hold still at an angle, then let go -- torque OFF, or
        # torque ON at Goal Current 0 -- and log the slide. Catches once the
        # load has taken it --release-catch-deg, so a fall is measured, not
        # stopped at the first degree. Angles move the gear mesh.
        lift = dirs[0]
        release = {"off": lambda bus: bus.torque(False), "zero": set_cap(0)}
        for rep in range(args.reps):
            order = [(a, m) for a in args.release_angles for m in args.release_modes]
            rng.shuffle(order)
            for a, m in order:
                at = {i: level[i] + math.radians(a) for i in level}
                info = {"rep": rep, "angle_deg": a, "mode": m, "dir": lift,
                        "approach": args.approach}
                if args.approach != "any":
                    # arrive from a fixed side: lifted into the hold (from the
                    # load's side) or lowered into it. Which one decides the
                    # slip: 0 of 72 lowered-into holds slipped, 27 of 82
                    # lifted-into ones did (2026-09-26, shuffled order).
                    side = -lift if args.approach == "lift" else lift
                    pre = {i: at[i] + side * math.radians(args.approach_deg) for i in at}
                    add(f"pre {a:+g} deg", args.home_s, pre, home_enter, {"kind": "home", **info})
                add(f"home {a:+g} deg", args.home_s, at, home_enter, {"kind": "home", **info})
                add(f"release {m} at {a:+g} deg rep {rep}", args.release_s, at, release[m],
                    {"kind": "release", **info}, sgn=lift,
                    fall_rad=math.radians(args.release_catch_deg))
        add("end: level", 1.0, dict(level), home_enter, {"kind": "home"})
        return segs
    for rep in range(args.reps):
        for sgn in dirs:
            order = list(args.currents)
            rng.shuffle(order)
            goal = {i: level[i] + sgn * step for i in level}
            for ma in order:
                info = {"rep": rep, "dir": sgn, "ma": ma}
                add("home", args.home_s, dict(level), set_cap(args.home_ma),
                    {"kind": "home", **info})
                add(f"try {ma:g} mA {'+' if sgn > 0 else '-'} rep {rep}", args.try_s,
                    goal, set_cap(ma), {"kind": "try", **info}, sgn=sgn)
    add("end: level", 1.0, dict(level), set_cap(args.home_ma), {"kind": "home"})
    return segs


# --------------------------------------------------------------------------
# analysis
# --------------------------------------------------------------------------

# WHAT A CAPTURE'S OWN meta.json GOT WRONG, found after the fact. Applied on
# load by `fit` and `summary` (and printed), so the traces stay as recorded.
#   mass_g   -- the heavy weight is 137 g (marked on it, re-weighed 2026-09-26,
#               screw and nuts included); it was typed as 117 for every run.
#   until_s  -- the lever was not on the shaft after this: the hub's pins
#               sheared ~122 s into the 236 g run (homes settle -1 deg, not -4,
#               from 123.5 s; a 100 mA try reaches the goal at 129 s).
CORRECTIONS = {
    **{name: {"mass_g": 137.0} for name in (
        "260925-231112_servo_lift_117g44L", "260926-144419_servo_lift_117g44L_hub15",
        "260926-145550_servo_lift_117g44L_lift", "260926-150415_servo_lift_117g44L_lift2",
        "260926-151744_servo_lift_117g25L", "260926-181733_servo_lift_pos_117g44L",
        "260926-181855_servo_lift_steps_117g44L", "260926-190801_servo_lift_pos_117g44L_lower")},
    "260926-203643_servo_lift_236g44R": {"until_s": 122.0},
}


def load_capture(d) -> Capture:
    cap = Capture.load(d)
    fix = CORRECTIONS.get(Path(d).name)
    if fix:
        print(f"  {Path(d).name}: correcting {fix} (see CORRECTIONS)")
        cap.meta.update(fix)
    return cap


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
            print(f"  id {i}, goal {'+' if sgn > 0 else '-'}:  {meta.get('cap_unit', 'mA')}  "
                  f"toward/held/against  median signed Present Current")
            lift = fall = None
            for ma in sorted({t["ma"] for t in sub}):
                at = [t for t in sub if t["ma"] == ma]
                n = {c: sum(t["cls"] == c for t in at) for c in ("toward", "held", "against")}
                nr = sum(t["rescued"] for t in at)
                if lift is None and n["toward"] == len(at):
                    lift = ma
                if n["against"]:
                    fall = ma
                print(f"      {ma:5.0f} {meta.get('cap_unit', 'mA'):3s}  {n['toward']}/{n['held']}/{n['against']}"
                      f"{f'  ({nr} caught)' if nr else '':14s}  {np.median([t['i'] for t in at]):+6.1f} mA")
            print(f"    -> every try moved toward the goal from {lift} mA; "
                  f"the load won at up to {fall} mA")
            res[f"dir{sgn:+d}"] = {"lift_ma": lift, "fall_max_ma": fall}
    for i in ids:
        try:
            wd = cap.raw("Bus Watchdog", i)
        except KeyError:
            continue
        hit = np.flatnonzero(cap.ok & (wd == 255))
        if len(hit):
            k = next(k for k, s in enumerate(cap.segments) if s["start"] <= hit[0] < s["stop"])
            print(f"\n  id {i}: Bus Watchdog TRIPPED from frame {hit[0]} "
                  f"(segment {k}, {cap.segments[k]['label']}), {len(hit)} frames")
    if any(s["info"].get("kind") == "spin" for s in cap.segments):
        summarize_spin(cap)
    if any(s["info"].get("kind") == "step" for s in cap.segments):
        summarize_steps(cap)
    if any(s["info"].get("kind") == "pwm" for s in cap.segments):
        summarize_pwm(cap)
    if any(s["info"].get("kind") == "release" for s in cap.segments):
        summarize_release(cap)
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
    if CAP is MODES["pwm"]:                     # no position loop: just stop driving
        write_cap(bus, 0)
        time.sleep(1.0)
        return
    write_cap(bus, args.home_ma)
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
        top = max([args.home_ma, *[abs(c) for c in args.currents]])
        limit = "Current Limit" if CAP["reg"] == "Goal Current" else "PWM Limit"
        for i in bus.ids:
            lim = orig[i][limit]                # raw: mA, or PWM LSB
            if top > lim:
                raise SystemExit(f"id {i}: {limit} {lim} < the {top:g} {CAP['unit']} "
                                 f"this run asks for")

        bus.prepare()                           # torque off, Return Delay 0, profiles 0
        for i in bus.ids:
            bus.write_raw(i, "Operating Mode", CAP["op"])
        if CAP["gains"]:
            for i in bus.ids:                   # AFTER the mode write, which resets them
                bus.write_raw(i, "Position P Gain", args.p_gain)
                bus.write_raw(i, "Position D Gain", args.d_gain)
                bus.write_raw(i, "Position I Gain", args.i_gain)
        imap = IndirectMap(bus.tables)
        for name in READ_BLOCK:
            imap.read(name)
        imap.read("Goal Position", label="goal_readback")
        imap.read("Bus Watchdog")               # 255 once tripped: see rescue()
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
        # With a load the lever starts resting on the stop on the load's side,
        # and only the LIFTING direction is worth stepping: toward the load, a
        # low current cannot stop it at the goal and every try ends in a catch
        # at the guard (96 of them, 2026-09-26) that measures nothing new.
        i0 = bus.ids[0]
        off0 = present[i0] - level[i0]
        if args.dirs:
            dirs = [1 if d == "+" else -1 for d in args.dirs]
        elif args.mass_g:
            if abs(math.degrees(off0)) < 20:
                raise SystemExit(f"a mass is on but the lever is {math.degrees(off0):+.1f} deg "
                                 f"from level, not on a stop: which way lifts? pass --dirs")
            dirs = [-1 if off0 > 0 else 1]
        else:
            if args.release:
                raise SystemExit("--release needs --mass-g (or --dirs): which way is the load?")
            dirs = [1, -1]
        print(f"stepping goal {' and '.join('+' if d > 0 else '-' for d in dirs)}"
              f"{' (lifting)' if args.mass_g and not args.dirs else ''}")
        bus.write_frame(dict(present))          # torque-on holds where it is, then settles
        ctx["present"] = present
        if args.pwm_sweep:
            if abs(math.degrees(off0)) < 40:
                raise SystemExit(f"--pwm-sweep wants the lever resting on a stop; it is "
                                 f"{math.degrees(off0):+.1f} deg from level")
            ctx["stop_side"] = 1 if off0 > 0 else -1
        segs = build_plan(args, level, ctx, dirs)
        total = sum(s.seconds for s in segs)
        n_try = sum(s.info.get("kind") == "try" for s in segs)
        v = {i: before[i]["Present Input Voltage"] * 0.1 for i in bus.ids}
        tau = load_torque(vars(args))
        print(f"\n{TEST}: ids {list(bus.ids)}, mode {args.mode} ({CAP['reg']}), supply {v} V, "
              f"P/I/D {args.p_gain}/{args.i_gain}/{args.d_gain}; "
              f"load {args.mass_g or 0:g} g at {args.radius_mm or 0:g} mm "
              f"({'none' if tau is None else f'{tau * 1000:.1f} mN m'}); "
              f"{n_try} tries, goal +-{args.step_deg:g} deg, {total / 60:.1f} min "
              f"at {args.rate:g} Hz. Levels {sorted(args.currents)} {CAP['unit']}.")
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
        stay = args.pwm_sweep or args.freespin      # no lift to level first
        set_cap(args.home_ma if not stay else 0)(bus)
        if not args.freespin:
            bus.torque(True)
        torque_on = True
        n = 40
        for k in range(1, n + 1 if not stay else 1):
            bus.write_frame({i: present[i] + (level[i] - present[i]) * k / n for i in bus.ids})
            time.sleep(args.lift_s / n)
        time.sleep(0.5)
        for i in bus.ids if not stay else ():
            off = math.degrees(bus.read(i, "Present Position") - level[i])
            if abs(off) > 8:            # P droop under load: 3.3 deg at 50 mN m, P 700
                raise SystemExit(f"id {i} did not reach level at {args.home_ma:g} {CAP['unit']} "
                                 f"({off:+.1f} deg off)")
        bus.bus_watchdog(LIFT_WATCHDOG_MS)

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
                gains={"p": args.p_gain, "i": args.i_gain, "d": args.d_gain},
                mode=args.mode, cap_register=CAP["reg"], cap_unit=CAP["unit"],
                git=git_state(ROOT), python=sys.version.split()[0],
                platform=platform.platform(), watchdog_ms=LIFT_WATCHDOG_MS,
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


def summarize_spin(cap: Capture) -> None:
    """Free spin: steady speed over each step's second half, per signed PWM."""
    i = int(cap.meta["ids"][0])
    vel = cap.value("Present Velocity", i)
    cur = cap.value("Present Current", i) * 1000
    rows = {}
    for k, s in enumerate(cap.segments):
        info = s["info"]
        if info.get("kind") != "spin":
            continue
        sl = _rows(cap, k)
        ok = cap.ok[sl]
        n = int(ok.sum())
        h = slice(n // 2, n)
        rows.setdefault(info["pwm"], {})[info["leg"]] = (
            float(np.median(vel[sl][ok][h])), float(np.median(cur[sl][ok][h])))
    print("\nFREE SPIN -- Goal PWM [LSB, 885 = 100 %] -> steady speed [rad/s], Present Current [mA]")
    for pwm in sorted(rows, key=lambda x: (x < 0, abs(x))):
        r = rows[pwm]
        print(f"  {pwm:+5.0f} ({abs(pwm) / 885:5.1%})" + "".join(
            f"   {leg:4s} {r[leg][0]:+7.2f} rad/s  I {r[leg][1]:+6.1f}" for leg in ("up", "down")
            if leg in r))


def summarize_steps(cap: Capture) -> None:
    """Step response: 10-90 % rise, overshoot, the error it settles to, peak speed."""
    i = int(cap.meta["ids"][0])
    pos = np.degrees(cap.position_rad(i))
    vel = np.degrees(cap.value("Present Velocity", i))
    t = cap.time_s(i)
    lv = cap.meta.get("level_deg", 180.0)
    print("\nSTEP RESPONSE -- from level: 10-90 % rise, overshoot, final error (last 0.3 s), peak speed")
    for k, s in enumerate(cap.segments):
        info = s["info"]
        if info.get("kind") != "step":
            continue
        sl = _rows(cap, k)
        ok = cap.ok[sl]
        p, tt = pos[sl][ok], t[sl][ok] - t[sl][ok][0]
        if len(p) < 10:
            continue
        x = (p - p[0]) * info["dir"]                       # progress toward the goal
        size = info["size_deg"]
        i10 = np.argmax(x >= 0.1 * size) if (x >= 0.1 * size).any() else None
        i90 = np.argmax(x >= 0.9 * size) if (x >= 0.9 * size).any() else None
        rise = (tt[i90] - tt[i10]) * 1000 if i10 is not None and i90 is not None else float("nan")
        over = (x.max() - size) / size * 100
        fin = size - x[tt > tt[-1] - 0.3].mean()
        print(f"  {'+' if info['dir'] > 0 else '-'}{size:4.0f} deg rep {info['rep']}: rise "
              f"{rise:5.0f} ms  overshoot {over:+5.1f} %  final error {fin:+5.2f} deg  "
              f"peak {np.abs(vel[sl][ok]).max():5.0f} deg/s")


def summarize_pwm(cap: Capture) -> None:
    """Stalled on the stop: Present PWM (and Present Current) per Goal
    Current, up and down, over each step's second half."""
    i = int(cap.meta["ids"][0])
    pwm = cap.value("Present PWM", i)
    raw = cap.raw("Present PWM", i)
    cur = cap.value("Present Current", i) * 1000
    pos = np.degrees(cap.position_rad(i))
    out = {}
    for k, s in enumerate(cap.segments):
        info = s["info"]
        if info.get("kind") != "pwm":
            continue
        sl = _rows(cap, k)
        ok = cap.ok[sl]
        n = int(ok.sum())
        h = slice(n // 2, n)
        raws = raw[sl][ok][h].astype(np.int64)
        raws = np.where(raws > 32767, raws - 65536, raws)
        out.setdefault(info["ma"], {})[info["leg"]] = (
            float(np.median(np.abs(pwm[sl][ok][h]))), int(np.median(np.abs(raws))),
            float(np.median(np.abs(cur[sl][ok][h]))), float(np.ptp(pos[sl][ok])))
    print("\nPWM SWEEP -- pushed into the stop: Goal Current -> Present PWM (fraction, raw LSB), "
          "Present Current, position span")
    for ma in sorted(out):
        row = f"  {ma:5.0f} mA"
        for leg in ("up", "down"):
            if leg in out[ma]:
                f, r, c, sp = out[ma][leg]
                row += f"   {leg:4s} {f:6.3f} ({r:3d})  I {c:5.1f}  {sp:4.2f} deg"
        print(row)


def summarize_release(cap: Capture) -> None:
    """Per release: how far the load took the lever [deg], how fast once it
    went, and whether it had to be caught."""
    ids = [int(i) for i in cap.meta["ids"]]
    rescued = {r["segment"] for r in cap.meta.get("rescues", [])}
    rows = []
    for k, s in enumerate(cap.segments):
        info = s["info"]
        if info.get("kind") != "release":
            continue
        sl = _rows(cap, k)
        ok = cap.ok[sl]
        for i in ids:
            pos = cap.position_rad(i)[sl][ok]
            t = cap.time_s(i)[sl][ok]
            if len(pos) < 5:
                continue
            d = np.degrees(-(pos - pos[0]) * info["dir"])      # + = the load's way
            if k in rescued:                                     # up to the catch
                d = d[:int(np.argmax(d)) + 1]
            moved = np.flatnonzero(d > 1.0)
            t1 = t[moved[0]] - t[0] if len(moved) else None
            span = d[-1] - d[moved[0]] if len(moved) else 0.0
            w = span / (t[len(d) - 1] - t[moved[0]]) if len(moved) and t[len(d) - 1] > t[moved[0]] else 0.0
            rows.append((info["mode"], info["angle_deg"], info["rep"], float(d.max()), t1, w,
                         k in rescued))
    print("\nRELEASE -- held still, then let go: slide toward the load")
    print("  mode  angle  rep  max slide  to 1 deg   speed after   caught")
    for m, a, r, dm, t1, w, c in sorted(rows):
        print(f"  {m:5s} {a:+5.0f}  {r:3d}  {dm:7.1f} deg  "
              + (f"{t1 * 1000:6.0f} ms" if t1 is not None else "   never ")
              + f"  {w:7.1f} deg/s   {'yes' if c else ''}")
    for m in sorted({r[0] for r in rows}):
        sub = [r for r in rows if r[0] == m]
        n = sum(r[3] > 1.0 for r in sub)
        print(f"  {m}: slid in {n} of {len(sub)}")


def thresholds(cap: Capture) -> dict:
    """The lifting direction's two thresholds [mA], each the midpoint between
    neighbouring levels: LIFT, above which every try moved toward the goal;
    FALL, above which no try lost to the load (below the lift). Below the
    deadband the current does nothing and falling is up to the gearbox's
    static friction, which varies with gear phase -- so trials there can
    fall, hold, fall as the current rises; FALL is where that ends."""
    moved = math.radians(cap.meta.get("moved_deg", 1.0))
    by = {}
    for k, s in enumerate(cap.segments):
        info = s["info"]
        if info.get("kind") != "try":
            continue
        if s.get("t_host", 0.0) > cap.meta.get("until_s", math.inf):
            continue
        sl = _rows(cap, k)
        pos = cap.position_rad(int(cap.meta["ids"][0]))[sl][cap.ok[sl]]
        if len(pos) < 5:
            continue
        d = (pos - pos[0]) * info["dir"]
        by.setdefault(info["ma"], []).append(
            "against" if -d.min() > moved else "toward" if d.max() > moved else "held")
    lv = sorted(by)
    up = next((i for i in range(len(lv)) if all(c == "toward" for m in lv[i:] for c in by[m])),
              None)
    if up is None:
        return {"lift": None, "fall": None, "tau": load_torque(cap.meta)}
    lost = [i for i in range(up) if "against" in by[lv[i]]]
    fall = (lv[lost[-1]] + lv[lost[-1] + 1]) / 2 if lost else None
    return {"lift": (lv[up - 1] + lv[up]) / 2 if up else lv[0], "fall": fall,
            "tau": load_torque(cap.meta)}


def fit(dirs: list) -> None:
    """Model-free lines through the loaded captures: the LIFT current and the
    FALL current (load wins) against the load torque. Splitting their slopes
    into a torque constant and friction needs an assumption these static
    tests cannot check -- that forward and back-driving friction are equal --
    so that split is printed as conditional."""
    lift, fall = [], []
    unit = None
    for d in dirs:
        cap = load_capture(d)
        u = cap.meta.get("cap_unit", "mA")
        if unit is None:
            unit = u
        elif u != unit:
            raise SystemExit(f"{Path(d).name} is in {u}, the others in {unit}: one mode per fit")
        t = thresholds(cap)
        if t["tau"] is None or t["lift"] is None:
            print(f"  {Path(d).name}: no load, or never lifted in every try -- skipped")
            continue
        tau = t["tau"] * 1000
        lift.append((tau, t["lift"]))
        if t["fall"] is not None:
            fall.append((tau, t["fall"]))
        print(f"  {Path(d).name}: {tau:5.1f} mN m  lift {t['lift']:5.1f}  fall "
              + (f"{t['fall']:5.1f}" if t["fall"] is not None else "never") + f" {unit}")
    out = {}
    for name, pts in (("lift", lift), ("fall", fall)):
        if len(pts) < 2:
            continue
        x, y = (np.array(c) for c in zip(*pts))
        b, a = np.polyfit(x, y, 1)
        out[name] = (a, b)
        print(f"\n  {name}: {a:5.1f} {unit} + {b:.3f} {unit} per mN m   "
              f"(residuals {np.round(y - (a + b * x), 1)} {unit}, {len(pts)} loads)")
    if "lift" in out and "fall" in out:
        (al, bl), (af, bf) = out["lift"], out["fall"]
        kt = 2 / (bl + bf)                          # mN m per unit of the cap
        per = (f"kt {kt:.3f} N m/A" if unit == "mA" else
               f"{kt:.3f} mN m per LSB = {kt * 885 / 1000:.2f} N m at 100 % duty")
        print(f"  IF forward and back-drive friction were equal: {per}, "
              f"friction {kt * (bl - bf) / 2:.2f} x load + {kt * (al - af) / 2:.1f} mN m")


def main() -> int:
    if len(sys.argv) > 2 and sys.argv[1] == "fit":
        fit(sys.argv[2:])
        return 0
    if len(sys.argv) > 2 and sys.argv[1] == "summary":
        for d in sys.argv[2:]:
            summarize(load_capture(d))
        return 0
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
    ap.add_argument("--mode", choices=sorted(MODES), default="current",
                    help="current: Goal Current caps a current loop (mode 5, the "
                         "righting servo's); position: Goal PWM caps the duty of a "
                         "position loop (mode 3, the steering's); pwm: Goal PWM is "
                         "the drive (mode 16, --freespin only)")
    ap.add_argument("--currents", type=float, nargs="+", default=None,
                    help="cap levels per try, in the mode's unit (mA, or PWM LSB with "
                         "885 = 100 %%). Default per mode: current 0-200 by 2 mA "
                         "(137 g at 44 mm fell to 43 and lifted at ~129); position "
                         "0-400 by 4 LSB")
    ap.add_argument("--freespin", action="store_true",
                    help="mode pwm, a BARE servo: step Goal PWM up then down each way "
                         "and log the steady speed")
    ap.add_argument("--spin-s", type=float, default=2.0)
    ap.add_argument("--steps", action="store_true",
                    help="step response: goal +-each --step-sizes from level at the "
                         "home cap, both directions")
    ap.add_argument("--step-sizes", type=float, nargs="+", default=[5, 10, 20])
    ap.add_argument("--step-s", type=float, default=1.5)
    ap.add_argument("--release", action="store_true",
                    help="the RELEASE test instead of the current sweep: hold, then "
                         "let go with torque off or at 0 mA, and log the slide")
    ap.add_argument("--pwm-sweep", action="store_true",
                    help="with the lever resting on a stop: push into it at each "
                         "--pwm-currents, up then down, and log Present PWM -- where "
                         "the drive actually turns on")
    ap.add_argument("--pwm-currents", type=float, nargs="+", default=list(range(0, 41)))
    ap.add_argument("--pwm-hold-s", type=float, default=1.0)
    ap.add_argument("--release-modes", nargs="+", choices=["off", "zero"],
                    default=["off", "zero"])
    ap.add_argument("--release-angles", type=float, nargs="+", default=[-10, -5, 0, 5, 10],
                    help="hold angles from level [deg]; each is a different gear mesh")
    ap.add_argument("--release-s", type=float, default=2.0)
    ap.add_argument("--approach", choices=["any", "lift", "lower"], default="any",
                    help="arrive at each hold by lifting into it or lowering into it, "
                         "from --approach-deg away (any: from wherever the last one left it)")
    ap.add_argument("--approach-deg", type=float, default=5.0)
    ap.add_argument("--release-catch-deg", type=float, default=20.0,
                    help="let the load slide this far before catching it")
    ap.add_argument("--dirs", nargs="+", choices=["+", "-"], default=None,
                    help="goal directions to step (default: both with no mass, only "
                         "the lifting one with a mass on)")
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
    ap.add_argument("--home-ma", type=float, default=None,
                    help="the cap for homing and catches, in the mode's unit "
                         "(default: 300 mA, or 885 LSB = 100 %%)")
    ap.add_argument("--home-s", type=float, default=0.8)
    ap.add_argument("--try-s", type=float, default=1.2)
    ap.add_argument("--p-gain", type=int, default=None,
                    help="default: the config's gains for the mode's role (righting "
                         "for current, steer for position)")
    ap.add_argument("--i-gain", type=int, default=None)
    ap.add_argument("--d-gain", type=int, default=None)
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
    args = ap.parse_args()
    global CAP
    if args.freespin:
        args.mode = "pwm"
    if args.mode == "pwm" and not args.freespin:
        ap.error("--mode pwm is for --freespin only: it has no position loop to catch with")
    CAP = MODES[args.mode]
    if args.currents is None:
        args.currents = list(CAP["levels"])
    if args.home_ma is None:
        args.home_ma = CAP["home"]
    g = config_gains(CAP["gains"]) if CAP["gains"] else {"p": 0, "i": 0, "d": 0}
    for k in ("p", "i", "d"):
        if getattr(args, f"{k}_gain") is None:
            setattr(args, f"{k}_gain", g[k])
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
