"""Drivetrain testbed: the rear assembly on the bench, every frame recorded.

Station A of `docs/plans/first-physical-test.md`: two XC430-W150 (ids 101/102)
belted onto the two ring-gear shafts of the omni wheel's differential. Each
subcommand runs ONE test and writes ONE capture directory under
`traces/drivetrain/` (gitignored, Dropbox-synced) through
`aow_sim.hw.bench_log`, whose docstring is the file format. The printed tables
are a quick look; the capture is the product.

    P=/dev/cu.usbserial-FTB8HNE3            # or export AOW_DXL_PORT
    python analysis/drivetrain_bench.py idle     --port $P --note "wheel on, hand held"
    python analysis/drivetrain_bench.py jog      --port $P      # CHECK THE SIGNS FIRST
    python analysis/drivetrain_bench.py belt     --port $P --revs 2
    python analysis/drivetrain_bench.py belt     --port $P --mode diff --revs 1   # count a ROLLER
    python analysis/drivetrain_bench.py step     --port $P                  # D1
    python analysis/drivetrain_bench.py reversal --port $P                  # D2
    python analysis/drivetrain_bench.py chirp    --port $P
    python analysis/drivetrain_bench.py coast    --port $P                  # D4
    python analysis/drivetrain_bench.py hold     --port $P --seconds 20     # D3, by hand
    python analysis/drivetrain_bench.py creep    --port $P                  # gear detents
    python analysis/drivetrain_bench.py summary  traces/drivetrain/<capture dir>

    ... step --gains 100:1920 400:1920     # repeat the schedule per Velocity P:I

| test | torque | what it records | for |
|---|---|---|---|
| `idle` | off | nothing commanded; spin the wheel by hand | frame rate; hand spin-down; sanity |
| `jog` | on | slow a / b / common / diff | the sign convention, by eye |
| `belt` | on | exactly N servo revs, common or `--mode diff` | `belt_ratio` from the tyre; `k_roller` and the detent count from a roller |
| `step` | on | Goal Velocity steps, both signs, 2-100% | D1: bandwidth (small), slew (large) |
| `reversal` | on | square waves, amplitude x frequency | D2: the reversal envelope, LOADED this time |
| `chirp` | on | log sine sweep | the frequency response in one pass |
| `coast` | on/off | PWM spin-up, then torque OFF vs PWM 0 | D4: friction vs back-EMF damping |
| `hold` | on | goal 0 (or `--frac`) while you push the wheel | D3 / protocol §5: integral action |
| `creep` | on | very slow constant diff / common, two speeds | detents in the differential's mesh |

COMMON AND DIFFERENTIAL, BECAUSE THE WHEEL IS A DIFFERENTIAL. The hub is the
MEAN of the two ring-gear shafts and the rollers turn with their DIFFERENCE
(`drivetrain.mix_*`, `k_roller`). So `common` drives the hub with the rollers
still and `diff` spins the rollers with the hub still: two different inertias
and two different sets of friction (`hub_joint_*` vs `roller_joint_*`), which a
one-servo test would mix together. `a` / `b` drive one servo while the other
holds zero velocity.

THE SIGN IS A READING OF THE CAD, NOT OF THE RIG. Both servos face their horns
outboard (`analysis/servo_mount.py`), so one Goal Velocity turns them opposite
ways about the axle; `--signs 1 -1` is the default on that basis and is
unverified. Run `jog` and watch: `common` must turn the WHEEL, `diff` must spin
the ROLLERS. If it is the other way round, pass `--signs 1 1`. Captures store
per-SERVO commands, so a wrong sign mislabels segments but corrupts no data.

EVERY FRAME, ONE FastSyncRead through indirect block 1 (19 read bytes, plus the
goal read back and written: 27 of 28 for Goal Velocity, 23 for Goal PWM). The
evidence for each, and what the onboard loop should read instead, is
`docs/measurements/servo-logging.md`:

    Realtime Tick           2  the servo's own clock -- the only timing truth
    Present Position        4  unwrapped, differenced over any span you like
    Present Velocity        4  the firmware's own filtered estimate (lag measured 09-01)
    Present PWM             2  the duty actually spent: saturation shows here first
    Present Load            2  inferred torque, 0.1 % of max. The XC430 has NO
                               current register; calibrate against a known torque
    Present Input Voltage   2  sag at the servo terminals under load; no-load
                               speed and stall torque scale with it
    Present Temperature     1  winding resistance rises ~0.4 %/C, so a long
                               session drifts; this says how much
    Hardware Error Status   1  an overload shutdown drops torque MID-capture with
                               no error anywhere else
    goal, read back       4/2  the register the test writes. SyncWrite returns no
                               status, so this is the only proof each write landed.
                               (Velocity Trajectory held this slot until 2026-09-13.
                               It lags the goal register by a further ~2-3 ms, so it
                               is the servo's reference, not a copy of the goal.)
    Bus Watchdog            1  255 once tripped. Read per frame because the creep
                               capture of 2026-09-13 read TRIPPED at shutdown with
                               no traffic gap over 18 ms and every goal obeyed, and
                               a single read at the end cannot say when it fired

Host side, also per frame: read start, read and write durations, the command
sent to each servo, and a dropped-frame flag. Before and after, in meta.json:
every configuration register (`CONFIG_REGISTERS`), the drivetrain params with
their `source:`, the git revision and dirty paths, argv and `--note`.

Not recorded because the rig cannot see it: hub and roller angle (only the
servo shafts are encoded -- hence `belt`), applied torque (hence `--note`
whatever you hang or push), supply current (a bench-supply reading belongs in
`--note`).

HAND-HELD SAFETY. Torque goes on only after Enter (`--yes` skips). Bus Watchdog
is armed at 100 ms while torque is on, so a hung or killed script stops the
servos instead of leaving the wheel spinning. Every exit path, Ctrl-C included,
zeroes the goal, drops torque, disarms the watchdog, restores any operating
mode and gains it changed, and SAVES what was captured (marked incomplete).
Velocity goals clamp to the servo's own Velocity Limit.
"""

from __future__ import annotations

import argparse
import math
import os
import platform
import sys
import time
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))     # this checkout's source, even from a worktree

import yaml  # noqa: E402

from aow_sim.hw.bench_log import (Capture, Segment, git_state,  # noqa: E402
                                  new_capture_dir, record)
from aow_sim.hw.dynamixel import (MODE_PWM, MODE_VELOCITY,  # noqa: E402
                                  DynamixelBus, IndirectMap,
                                  describe_hardware_error)
from aow_sim.params import DEFAULT_PARAMS  # noqa: E402

MODEL = "xc430_w150"
READ_BLOCK = ("Realtime Tick", "Present Position", "Present Velocity",
              "Present PWM", "Present Load", "Present Input Voltage",
              "Present Temperature", "Hardware Error Status", "Bus Watchdog")
# Plus, whenever a test writes, the register it writes -- read back as
# "goal_readback". A SyncWrite returns no status packet, so reading the goal
# back is the only per-frame proof it landed. It replaced Velocity Trajectory,
# which is NOT the same number: the goal register reads back the next frame,
# and Velocity Trajectory (the servo's own reference) follows ~4 ms after the
# frame that computed the command, ramping over ~2 frames even with profiles
# at 0 (measured 2026-09-13; drivetrain-measurements.yaml, `latency`).
WATCHDOG_MS = 100
MODES = ("common", "diff", "a", "b")
HINT = {"a": "servo A turns, B holds",
        "b": "servo B turns, A holds",
        "common": "the WHEEL should turn, rollers still",
        "diff": "the ROLLERS should spin, hub still"}


# --------------------------------------------------------------------------
# context and schedule building blocks
# --------------------------------------------------------------------------

@dataclass
class Ctx:
    bus: DynamixelBus
    args: argparse.Namespace
    no_load: float           # rad/s at the servo, 12 V datasheet
    v_limit: float           # rad/s, the lower of the two servos' Velocity Limit
    _clamped: set = None

    @property
    def ids(self) -> tuple:
        return self.bus.ids

    def vec(self, mode: str) -> tuple:
        sa, sb = self.args.signs
        return {"common": (sa, sb), "diff": (sa, -sb),
                "a": (sa, 0.0), "b": (0.0, sb)}[mode]

    def goal(self, mode: str, value: float) -> dict:
        return {i: k * value for i, k in zip(self.ids, self.vec(mode))}

    def speed(self, frac: float) -> float:
        """Fraction of no-load -> servo rad/s, clamped under Velocity Limit."""
        v, cap = frac * self.no_load, 0.98 * self.v_limit
        if abs(v) > cap:
            if self._clamped is None:
                self._clamped = set()
            if abs(frac) not in self._clamped:
                print(f"  note: {abs(frac):g} x no-load = {abs(v):.2f} rad/s clamps "
                      f"to {cap:.2f} (Velocity Limit)")
                self._clamped.add(abs(frac))
            v = math.copysign(cap, v)
        return v


@dataclass
class Plan:
    segments: list
    write: str | None = "Goal Velocity"
    mode: int | None = MODE_VELOCITY
    torque: bool = True
    brief: str = ""


def const(ctx: Ctx, mode: str, value: float) -> Callable:
    g = ctx.goal(mode, value)
    return lambda t, s: g


def rest(ctx: Ctx, seconds: float, **info) -> Segment:
    return Segment("rest", seconds, const(ctx, "common", 0.0),
                   info={"kind": "rest", **info})


def gain_blocks(ctx: Ctx):
    """Yield ``(enter, info)`` once per ``--gains P:I`` (once, untouched, if none).

    Gains are RAM and writable with torque on, so a sweep needs no mode dance.
    Read back after writing: `control_tables/README.md` records a mode write
    silently resetting gains, and a sweep whose gains did not stick measures
    nothing.
    """
    if not ctx.args.gains:
        yield None, {}
        return
    for p, i in ctx.args.gains:
        def enter(bus, p=p, i=i):
            for d in bus.ids:
                bus.write_raw(d, "Velocity P Gain", p)
                bus.write_raw(d, "Velocity I Gain", i)
                got = (bus.read_raw(d, "Velocity P Gain"),
                       bus.read_raw(d, "Velocity I Gain"))
                if got != (p, i):
                    raise RuntimeError(f"id {d}: wrote gains {(p, i)}, read back {got}")
        yield enter, {"kvp": p, "kvi": i}


def with_enter(block: list, enter) -> list:
    if block and enter is not None:
        block[0].enter = enter
    return block


# --------------------------------------------------------------------------
# the tests
# --------------------------------------------------------------------------

# What `idle --write` writes. NOT Goal Velocity: with torque off the servo
# acknowledges a Goal Velocity write (rc=0 err=0) and keeps it at 0, and keeps
# Goal Position unchanged the same way -- measured 2026-09-13 on both XC430s,
# after the first `idle --write` read back 0 on all 1602 frames. Goal PWM,
# Profile Velocity and LED hold. Profile Velocity is 4 bytes like the goal,
# unused in velocity mode, and reset to 0 by `DynamixelBus.prepare` at the
# start of every run. See hw/control_tables/README.md.
IDLE_WRITE_REGISTER = "Profile Velocity"


def plan_idle(ctx: Ctx) -> Plan:
    """Torque off throughout. `--write` adds a SyncWrite every frame.

    With `--write` this is the bus-timing check: the frame rate and the read and
    write call times with a write in every frame, against a plain idle, and
    whether every write LANDED. Each frame writes a fresh RANDOM value, so a
    read-back matches exactly one frame's write and the lag is unambiguous; a
    slow sine changes by under one LSB per frame and would match several. It
    writes `IDLE_WRITE_REGISTER`, not a goal, because a goal written with torque
    off is silently discarded. Nothing moves; the register is zeroed on exit.
    """
    a = ctx.args
    if not a.write:
        return Plan([Segment("torque OFF -- spin the wheel by hand if you like",
                             a.seconds, None, info={"kind": "idle", "torque": False})],
                    write=None, mode=None, torque=False)
    rng = np.random.default_rng(0)
    zero = {i: 0.0 for i in ctx.ids}

    def random_value(t, s):
        return {i: float(rng.integers(0, 32768)) for i in ctx.ids}   # its full range

    return Plan([Segment(f"torque OFF, random {IDLE_WRITE_REGISTER} every frame",
                         a.seconds, random_value,
                         info={"kind": "idle_write", "torque": False,
                               "register": IDLE_WRITE_REGISTER}),
                 Segment(f"torque OFF, {IDLE_WRITE_REGISTER} zero", 0.2, lambda t, s: zero,
                         info={"kind": "rest", "torque": False})],
                write=IDLE_WRITE_REGISTER, mode=MODE_VELOCITY, torque=False)


def plan_jog(ctx: Ctx) -> Plan:
    a = ctx.args
    v = ctx.speed(a.frac)
    segs = [rest(ctx, 1.0)]
    for mode in ("a", "b", "common", "diff"):
        segs += [Segment(f"jog {mode}: {HINT[mode]}", a.seconds,
                         const(ctx, mode, v),
                         info={"kind": "jog", "mode": mode, "v": v}),
                 rest(ctx, 1.0)]
    return Plan(segs, brief=f"jog at {v:.2f} rad/s. WATCH: `common` should turn "
                            f"the wheel, `diff` should spin the rollers.")


def plan_belt(ctx: Ctx) -> Plan:
    """Exactly `--revs` servo revs, then stop, so a hand count can be compared.

    common: both ring shafts together, so the HUB turns belt_ratio x revs --
    count a mark on the tyre. diff: the shafts oppose, the hub stays, and the
    ring-vs-hub angle is belt_ratio x revs x 360 deg, so the ROLLERS turn
    k_roller x that -- count a mark on one roller. Diff defaults to 3 % of
    no-load, the creep speed at which every 7.5 deg detent shows as a stop, so
    the encoder data counts the detents while you count the turns.
    """
    a = ctx.args
    mode = a.mode
    frac = a.frac if a.frac is not None else (0.08 if mode == "common" else 0.03)
    v = ctx.speed(frac)
    target = a.revs * 2 * np.pi
    id_a = ctx.ids[0]
    run, stop = ctx.goal(mode, v), ctx.goal(mode, 0.0)
    st = {"prev": None, "acc": 0.0, "done": False}

    def cmd(t, s):
        if s is not None:
            p = s[id_a]["Present Position"]
            if st["prev"] is not None:
                st["acc"] += (p - st["prev"] + np.pi) % (2 * np.pi) - np.pi
            st["prev"] = p
        if st["done"] or abs(st["acc"]) >= target:
            st["done"] = True
            return stop
        return run

    seconds = target / abs(v) * 1.3 + 1.0
    belt = ctx.args.params["belt_ratio"]
    if mode == "common":
        brief = (f"Mark the tyre against something fixed. At belt_ratio "
                 f"{belt['value']} (source: {belt['source']}) the wheel "
                 f"should turn {a.revs * belt['value']:g} times.")
    else:
        kr = ctx.args.params["drivetrain"]["k_roller"]
        ring_deg = a.revs * belt["value"] * 360.0
        brief = (f"Mark ONE ROLLER against something fixed. {a.revs:g} servo revs in diff "
                 f"is {ring_deg:g} deg of ring-vs-hub angle: at k_roller {kr['value']} "
                 f"(source: {kr['source']}) the roller should turn "
                 f"{a.revs * belt['value'] * kr['value']:g} times, and "
                 f"{ring_deg / 7.5:.0f} detents should pass. This takes "
                 f"~{target / abs(v):.0f} s. Count the roller's turns.")
    return Plan([rest(ctx, 1.0),
                 Segment(f"belt: {a.revs:g} servo revs in {mode} mode", seconds,
                         cmd, info={"kind": "belt", "mode": mode, "revs": a.revs,
                                    "v": v, "frac": frac}),
                 rest(ctx, 1.5)],
                brief=brief)


def plan_step(ctx: Ctx) -> Plan:
    a = ctx.args
    signs = (1,) if a.one_sided else (1, -1)
    segs = []
    for enter, ginfo in gain_blocks(ctx):
        block = [rest(ctx, a.settle, **ginfo)]
        for mode in a.modes:
            for frac in a.fracs:
                for sg in signs:
                    f = sg * frac
                    v = ctx.speed(f)
                    block += [
                        Segment(f"step {mode} {f:+.2f}", a.hold, const(ctx, mode, v),
                                info={"kind": "step", "mode": mode, "frac": f,
                                      "v": v, **ginfo}),
                        Segment(f"return {mode} from {f:+.2f}", a.rest,
                                const(ctx, mode, 0.0),
                                info={"kind": "return", "mode": mode, "frac": f,
                                      "v_from": v, **ginfo})]
        segs += with_enter(block, enter)
    return Plan(segs)


def plan_reversal(ctx: Ctx) -> Plan:
    a = ctx.args
    segs = []
    for enter, ginfo in gain_blocks(ctx):
        block = [rest(ctx, 0.5, **ginfo)]
        for mode in a.modes:
            for frac in a.amps:
                amp = ctx.speed(frac)
                for f in a.freqs:
                    up, dn = ctx.goal(mode, amp), ctx.goal(mode, -amp)
                    block += [
                        Segment(f"square {mode} {frac:.2f} @ {f:g} Hz", a.cycles / f,
                                lambda t, s, up=up, dn=dn, f=f:
                                    up if (t * f) % 1.0 < 0.5 else dn,
                                info={"kind": "square", "mode": mode, "frac": frac,
                                      "amp": amp, "freq": f, **ginfo}),
                        rest(ctx, 0.3, **ginfo)]
        segs += with_enter(block, enter)
    return Plan(segs)


def plan_chirp(ctx: Ctx) -> Plan:
    a = ctx.args
    if not a.f1 > a.f0 > 0:
        raise SystemExit("chirp needs 0 < --f0 < --f1")
    if a.rate and a.f1 > a.rate / 10:
        print(f"  note: --f1 {a.f1:g} Hz is above a tenth of --rate {a.rate:g} Hz")
    k, T = a.f1 / a.f0, a.seconds
    c = 2 * np.pi * a.f0 * T / np.log(k)      # log sweep: f(t) = f0 * k**(t/T)
    segs = []
    for enter, ginfo in gain_blocks(ctx):
        block = [rest(ctx, 0.5, **ginfo)]
        for mode in a.modes:
            amp = ctx.speed(a.amp)
            vec = ctx.vec(mode)

            def cmd(t, s, amp=amp, vec=vec):
                v = amp * math.sin(c * (k ** (t / T) - 1.0))
                return {i: g * v for i, g in zip(ctx.ids, vec)}

            block += [Segment(f"chirp {mode} {a.amp:.2f} {a.f0:g}-{a.f1:g} Hz", T, cmd,
                              info={"kind": "chirp", "mode": mode, "amp": amp,
                                    "f0": a.f0, "f1": a.f1, "law": "log", **ginfo}),
                      rest(ctx, 0.5, **ginfo)]
        segs += with_enter(block, enter)
    return Plan(segs)


def plan_coast(ctx: Ctx) -> Plan:
    """D4. Spin up on PWM, then decay twice: torque OFF, and PWM 0 torque ON.

    PWM mode rather than velocity, so the second decay has no velocity loop
    braking it -- whatever slows it beyond the torque-off decay is the drive
    electronics at zero duty. Whether zero duty SHORTS the windings (back-EMF
    braking) or floats them (a second coast) is not documented; the two decays
    side by side answer it.
    """
    a = ctx.args
    d = float(np.clip(a.duty, 0.0, 1.0))
    zero = const(ctx, "common", 0.0)

    def torque(on):
        return lambda bus: bus.torque(on)

    segs = [Segment("rest", 0.5, zero, info={"kind": "rest"})]
    for mode in a.modes:
        for r in range(a.repeats):
            spin = const(ctx, mode, d)
            base = {"mode": mode, "duty": d, "repeat": r}
            segs += [
                Segment(f"spin {mode} duty {d:g}", a.spin, spin,
                        info={"kind": "spin", **base}),
                Segment(f"coast {mode}: torque OFF", a.decay, zero,
                        enter=torque(False),
                        info={"kind": "coast_off", "torque": False, **base}),
                Segment("torque back on", 0.3, zero, enter=torque(True),
                        info={"kind": "rest", **base}),
                Segment(f"spin {mode} duty {d:g}", a.spin, spin,
                        info={"kind": "spin", **base}),
                Segment(f"coast {mode}: PWM 0, torque on", a.decay, zero,
                        info={"kind": "coast_pwm0", "torque": True, **base}),
                Segment("rest", 0.3, zero, info={"kind": "rest", **base})]
    return Plan(segs, write="Goal PWM", mode=MODE_PWM)


def plan_hold(ctx: Ctx) -> Plan:
    a = ctx.args
    v = ctx.speed(a.frac)
    segs = []
    for enter, ginfo in gain_blocks(ctx):
        info = {"mode": a.mode, "v": v, **ginfo}
        segs += with_enter([
            Segment("baseline -- hands off", a.baseline, const(ctx, a.mode, v),
                    info={"kind": "baseline", **info}),
            Segment(f"hold {a.mode}: push or twist the wheel, let go, repeat",
                    a.seconds, const(ctx, a.mode, v), info={"kind": "hold", **info}),
            Segment("released -- hands off", a.baseline, const(ctx, a.mode, v),
                    info={"kind": "released", **info})], enter)
    return Plan(segs, brief="Say in --note what you pushed with, if anything "
                            "was known (a hung mass on a lever, a scale).")


def plan_creep(ctx: Ctx) -> Plan:
    """Slow constant velocity: does the motion stop at repeating ANGLES?

    Diff motion through the differential feels notchy by hand; common motion
    does not. Every mesh in it repeats every 7.5 deg of ring-vs-hub angle (48
    ring teeth; 12-tooth planet onto a 20-tooth roller gear at k_roller 2.4
    lands on the same 7.5), so a detent should space its stops 7.5 deg apart
    -- or 7.5/8 if the eight rollers are staggered.

    TWO SPEEDS ARE THE POINT. A detent is fixed in ANGLE, so its spacing is
    the same at both. The velocity loop's own stick-slip cycle is fixed in
    TIME, so its spacing in angle grows with speed. One speed cannot tell the
    two apart, and at small commands both happen at once. Common mode is the
    control: same loop, no notching.

    `--gains P:I ...` repeats the whole schedule per Velocity P/I pair: can
    firmware gains smooth the stick-slip through the detents? More P damps the
    burst; the I term is the spring that stores the twist released at
    breakaway, so lower I should mean longer stalls and bigger bursts.
    """
    a = ctx.args
    segs = []
    for enter, ginfo in gain_blocks(ctx):
        tag = f" P{ginfo['kvp']}:I{ginfo['kvi']}" if ginfo else ""
        block = [rest(ctx, 1.0, **ginfo)]
        for mode in a.modes:
            for frac in a.fracs:
                for sg in (1, -1):
                    v = ctx.speed(sg * frac)
                    block += [Segment(f"creep {mode} {sg * frac:+.3f}{tag}", a.seconds,
                                      const(ctx, mode, v),
                                      info={"kind": "creep", "mode": mode,
                                            "frac": sg * frac, "v": v, **ginfo}),
                              rest(ctx, 1.0, mode=mode, **ginfo)]
        segs += with_enter(block, enter)
    return Plan(segs)


# --------------------------------------------------------------------------
# quick-look reports: every one reads only the capture and its meta
# --------------------------------------------------------------------------

def _q(fn, x, *a):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return fn(x, *a)


def _of_kind(cap: Capture, *kinds):
    return [(k, s) for k, s in enumerate(cap.segments)
            if s["info"].get("kind") in kinds and s["stop"] > s["start"]]


def print_summary(cap: Capture) -> None:
    m, ok, ids = cap.meta, cap.ok, cap.ids
    th = cap.arrays["t_host"]
    span = th[-1] - th[0] if len(th) > 1 else float("nan")
    print(f"\n== {m.get('test', '?')}: {len(cap)} frames in {span:.1f} s, "
          f"{m['stats']['dropped']} dropped, {m['stats']['overruns']} over the "
          f"{m['rate_hz']:g} Hz pacing, complete={m['complete']}"
          + (f" ({m['stopped_by']})" if m["stopped_by"] else ""))
    print(f"   read {1e3 * _q(np.nanmedian, cap.arrays['read_s']):.2f} ms median "
          f"(p99 {1e3 * _q(np.nanpercentile, cap.arrays['read_s'], 99):.2f}); write "
          f"{1e3 * _q(np.nanmedian, cap.arrays['write_s']):.2f} ms")
    for i in ids:
        t = cap.time_s(i)
        d = np.diff(t[np.isfinite(t)]) * 1e3
        errs = np.bitwise_or.reduce(cap.raw("Hardware Error Status", i)[ok]) if ok.any() else 0
        print(f"   id {i}: tick dt median {_q(np.median, d):.1f} ms, max {_q(np.max, d):.0f} ms; "
              f"{_q(np.nanmin, cap.value('Present Input Voltage', i)):.1f}-"
              f"{_q(np.nanmax, cap.value('Present Input Voltage', i)):.1f} V; "
              f"{_q(np.nanmin, cap.value('Present Temperature', i)):.0f}-"
              f"{_q(np.nanmax, cap.value('Present Temperature', i)):.0f} C; "
              f"hardware errors: {describe_hardware_error(int(errs))}")
    sd = m.get("shutdown", {})
    if any((sd.get("watchdog_tripped") or {}).values()):
        print(f"   !! Bus Watchdog read TRIPPED at shutdown on {sd['watchdog_tripped']}. "
              f"Goals after a trip are ignored: check Velocity Trajectory against cmd")
    if "Bus Watchdog" in cap.labels:
        for i in ids:
            hit = np.flatnonzero(ok & (cap.raw("Bus Watchdog", i) == 255))
            if len(hit):
                k = int(cap.arrays["segment"][hit[0]])
                print(f"   !! id {i}: Bus Watchdog TRIPPED from frame {hit[0]} "
                      f"(t_host {th[hit[0]]:.2f} s, segment {k} '{cap.segments[k]['label']}')")

    vel = {i: cap.diff_velocity(i, 5) for i in ids}
    print(f"\n{'#':>3s}  {'segment':<46s} {'id':>4s} {'n':>5s} {'cmd':>7s} "
          f"{'vel':>7s} {'v_rep':>7s} {'|pwm|95':>7s} {'|load|95':>8s} {'Vmin':>5s}")
    for k, s in enumerate(cap.segments):
        sl = slice(s["start"], s["stop"])
        for c, i in enumerate(ids):
            name = s["label"][:46] if c == 0 else ""
            print(f"{k if c == 0 else '':>3}  {name:<46s} {i:4d} {sl.stop - sl.start:5d} "
                  f"{_q(np.nanmean, cap.command(i)[sl]):7.2f} "
                  f"{_q(np.nanmean, vel[i][sl]):7.2f} "
                  f"{_q(np.nanmean, cap.value('Present Velocity', i)[sl]):7.2f} "
                  f"{_q(np.nanpercentile, np.abs(cap.value('Present PWM', i)[sl]), 95):7.3f} "
                  f"{_q(np.nanpercentile, np.abs(cap.value('Present Load', i)[sl]), 95):8.3f} "
                  f"{_q(np.nanmin, cap.value('Present Input Voltage', i)[sl]):5.1f}")


def _drive_split(cap: Capture, i_a: int, i_b: int, sl: slice):
    """Net servo revs, and what they imply at the hub and across the rollers."""
    sa, sb = cap.meta["signs"]
    belt = cap.meta["params"]["belt_ratio"]["value"]
    pa, pb = cap.position_rad(i_a)[sl], cap.position_rad(i_b)[sl]
    ga, gb = np.flatnonzero(np.isfinite(pa)), np.flatnonzero(np.isfinite(pb))
    if not len(ga) or not len(gb):
        return None
    ra = (pa[ga[-1]] - pa[ga[0]]) / (2 * np.pi)
    rb = (pb[gb[-1]] - pb[gb[0]]) / (2 * np.pi)
    return ra, rb, belt * (sa * ra + sb * rb) / 2, belt * (sa * ra - sb * rb) / 2


def after_jog(cap: Capture) -> None:
    i_a, i_b = cap.ids
    print("\nnet revs per segment; hub/diff at the input shafts assume --signs "
          f"{cap.meta['signs']} and belt_ratio {cap.meta['params']['belt_ratio']['value']}")
    print(f"  {'segment':<44s} {'A rev':>7s} {'B rev':>7s} {'hub':>7s} {'diff':>7s}")
    for k, s in _of_kind(cap, "jog"):
        r = _drive_split(cap, i_a, i_b, cap.segment_slice(k))
        if r:
            print(f"  {s['label']:<44s} {r[0]:7.3f} {r[1]:7.3f} {r[2]:7.3f} {r[3]:7.3f}")
    print("\nThe numbers only restate the commands. The CHECK is what you saw: did "
          "`common`\nturn the wheel and `diff` spin the rollers? If reversed, re-run "
          "everything with --signs 1 1.")


def after_belt(cap: Capture) -> None:
    i_a, i_b = cap.ids
    k = _of_kind(cap, "belt")
    if not k:
        return
    seg = cap.segments[k[0][0]]
    mode = seg["info"].get("mode", "common")        # captures before --mode were common
    start = cap.segment_slice(k[0][0]).start
    r = _drive_split(cap, i_a, i_b, slice(start, len(cap)))
    if not r:
        return
    belt = cap.meta["params"]["belt_ratio"]
    print(f"\nservo A {r[0]:+.3f} rev, B {r[1]:+.3f} rev (encoder, through the settle).")
    if mode == "common":
        print(f"At belt_ratio {belt['value']} (source: {belt['source']}) the hub turned "
              f"{r[2]:+.3f} rev and the roller drive {r[3]:+.3f}.")
        print("Your count of the tyre mark divided by the servo revs IS the belt ratio.")
        return

    kr = cap.meta["params"]["drivetrain"]["k_roller"]
    ring_deg = 360.0 * r[3]
    print(f"Ring-vs-hub angle {ring_deg:+.1f} deg (hub {360.0 * r[2]:+.1f} deg, which should "
          f"be ~0). At k_roller {kr['value']} (source: {kr['source']}) the rollers turned "
          f"{kr['value'] * r[3]:+.2f} times.")
    print(f"k_roller = your roller count x 360 / {abs(ring_deg):.1f}   "
          f"(so {kr['value'] * abs(r[3]):.1f} counted turns confirms {kr['value']})")

    # The detents, from the encoders, over the moving part of the segment only.
    sa, sb = cap.meta["signs"]
    pa, pb = cap.position_rad(i_a), cap.position_rad(i_b)
    q = np.degrees(belt["value"] * (sa * pa - sb * pb) / 2)
    t = cap.time_s(i_a)
    sl = cap.segment_slice(k[0][0])
    qs, ts = q[sl], t[sl]
    g = np.isfinite(qs) & np.isfinite(ts)
    if g.sum() < 50:
        return
    cmd = np.degrees(belt["value"] * abs(seg["info"]["v"]))
    moving = np.flatnonzero(np.abs(qs[g] - qs[g][-1]) > 1.0)    # drop the final hold
    end = moving[-1] + 1 if len(moving) else g.sum()
    qm, tm = qs[g][:end], ts[g][:end]
    travel = abs(qm[-1] - qm[0])
    stops = _stops(qm, tm, 0.25 * cmd)
    pos = np.array([p for p, _ in stops])
    spacing = _q(np.median, np.abs(np.diff(pos))) if len(pos) > 2 else float("nan")
    per, peak, _ = _angle_ripple(qm, tm)
    # The SPECTRUM counts detents; stops only bound them from below. At 3 % the
    # first real run (260913-123607_belt) stopped fully at 75 of 144 -- the rest
    # were driven through -- while the angle spectrum put 144.0 cycles in the
    # same travel.
    cycles = travel / per if np.isfinite(per) else float("nan")
    print(f"\ndetents in the encoder data, over {travel:.0f} deg of ring-vs-hub travel "
          f"({travel / 7.5:.1f} expected at 7.5 deg):")
    print(f"  angle spectrum: period {per:.2f} deg ({peak:.0f}x the band median), i.e. "
          f"{cycles:.1f} detents")
    print(f"  full stops: {len(stops)}, median spacing {spacing:.2f} deg. Fewer stops than "
          f"detents is normal above ~1 %; the motion runs through some of them.")


def after_step(cap: Capture) -> None:
    vel = {i: cap.diff_velocity(i, 2) for i in cap.ids}
    print("\nsteps, position differenced over +-2 frames. t63 is the first sample past "
          "63 %:\ncoarse at 2 ms/frame and noisy on small steps. Fit from the capture.")
    print(f"  {'segment':<28s} {'id':>4s} {'target':>7s} {'steady':>7s} {'ratio':>6s} "
          f"{'t63 ms':>7s} {'|pwm|95':>7s}")
    for k, s in _of_kind(cap, "step", "return"):
        sl = cap.segment_slice(k)
        for i in cap.ids:
            t, v = cap.time_s(i)[sl], vel[i][sl]
            g = np.isfinite(t) & np.isfinite(v)
            target = _q(np.nanmedian, cap.command(i)[sl])
            if g.sum() < 10 or not np.isfinite(target):
                continue
            t, v = t[g] - t[g][0], v[g]
            v0, steady = v[:2].mean(), v[-max(5, len(v) // 4):].mean()
            span = target - v0
            t63 = float("nan")
            if abs(span) > 0.05:
                hit = np.flatnonzero((v - v0) / span >= 0.632)
                if len(hit):
                    t63 = 1e3 * t[hit[0]]
            ratio = steady / target if abs(target) > 1e-9 else float("nan")
            pwm = _q(np.nanpercentile, np.abs(cap.value("Present PWM", i)[sl]), 95)
            print(f"  {s['label']:<28s} {i:4d} {target:7.2f} {steady:7.2f} {ratio:6.2f} "
                  f"{t63:7.1f} {pwm:7.3f}")


def after_reversal(cap: Capture) -> None:
    vel = {i: cap.diff_velocity(i, 2) for i in cap.ids}
    print("\nswing = p97-p3 of differenced velocity after the first quarter period, "
          "over commanded 2A")
    print(f"  {'segment':<34s} {'id':>4s} {'swing':>7s} {'ratio':>6s} {'rep ratio':>9s} "
          f"{'|pwm|95':>7s} {'|load|95':>8s}")
    for k, s in _of_kind(cap, "square"):
        sl, f = cap.segment_slice(k), s["info"]["freq"]
        for i in cap.ids:
            cmd = cap.command(i)[sl]
            amp = _q(np.nanmax, np.abs(cmd))
            t = cap.time_s(i)[sl]
            settle = np.isfinite(t) & (t - _q(np.nanmin, t) > 0.25 / f)
            v, vr = vel[i][sl][settle], cap.value("Present Velocity", i)[sl][settle]
            if settle.sum() < 10 or not amp:
                continue
            sw = _q(np.nanpercentile, v, 97) - _q(np.nanpercentile, v, 3)
            swr = _q(np.nanpercentile, vr, 97) - _q(np.nanpercentile, vr, 3)
            print(f"  {s['label']:<34s} {i:4d} {sw:7.2f} {sw / (2 * amp):6.2f} "
                  f"{swr / (2 * amp):9.2f} "
                  f"{_q(np.nanpercentile, np.abs(cap.value('Present PWM', i)[sl]), 95):7.3f} "
                  f"{_q(np.nanpercentile, np.abs(cap.value('Present Load', i)[sl]), 95):8.3f}")


def after_coast(cap: Capture) -> None:
    vel = {i: cap.diff_velocity(i, 5) for i in cap.ids}
    print("\ndecays. t10/t50 separates the friction form: ~3.3 for a viscous "
          "(exponential)\ndecay, ~1.8 for Coulomb (linear). Quick look; fit from the capture.")
    print(f"  {'segment':<34s} {'id':>4s} {'v0':>6s} {'t50 ms':>7s} {'t10 ms':>7s} {'t10/t50':>7s}")
    for k, s in _of_kind(cap, "coast_off", "coast_pwm0"):
        sl = cap.segment_slice(k)
        for i in cap.ids:
            t, v = cap.time_s(i)[sl], vel[i][sl]
            g = np.isfinite(t) & np.isfinite(v)
            if g.sum() < 10:
                continue
            t, v = t[g] - t[g][0], v[g]
            v0 = v[:3].mean()
            if abs(v0) < 0.3:
                print(f"  {s['label']:<34s} {i:4d} {v0:6.2f}   (not spinning at the start)")
                continue
            rel = v / v0

            def cross(x):
                h = np.flatnonzero(rel <= x)
                return 1e3 * t[h[0]] if len(h) else float("nan")

            t50, t10 = cross(0.5), cross(0.1)
            print(f"  {s['label']:<34s} {i:4d} {v0:6.2f} {t50:7.0f} {t10:7.0f} "
                  f"{t10 / t50 if t50 else float('nan'):7.2f}")


def after_hold(cap: Capture) -> None:
    print("\nshaft angle against the baseline mean, in servo degrees. Returning to ~0 "
          "after\nrelease is integral action; a standing offset is a P-only loop.")
    print(f"  {'id':>4s} {'max excursion':>13s} {'after release':>13s} {'|load|95 hold':>13s}")
    base, hold, rel = (_of_kind(cap, k) for k in ("baseline", "hold", "released"))
    for n in range(min(len(base), len(hold), len(rel))):
        if cap.segments[hold[n][0]]["info"].get("kvp") is not None:
            print(f"  gains {cap.segments[hold[n][0]]['info']['kvp']}:"
                  f"{cap.segments[hold[n][0]]['info']['kvi']}")
        for i in cap.ids:
            p = np.degrees(cap.position_rad(i))
            b0 = _q(np.nanmean, p[cap.segment_slice(base[n][0])])
            ph = p[cap.segment_slice(hold[n][0])] - b0
            pr = p[cap.segment_slice(rel[n][0])] - b0
            load = _q(np.nanpercentile, np.abs(cap.value("Present Load", i)[
                cap.segment_slice(hold[n][0])]), 95)
            print(f"  {i:4d} {_q(np.nanmax, np.abs(ph)):13.1f} "
                  f"{_q(np.nanmean, pr[len(pr) // 2:]):13.2f} {load:13.3f}")


def after_idle(cap: Capture) -> None:
    i_a, i_b = cap.ids
    r = _drive_split(cap, i_a, i_b, slice(0, len(cap)))
    if r:
        print(f"\nnet servo revs A {r[0]:+.3f}, B {r[1]:+.3f}; hub {r[2]:+.3f}, "
              f"diff {r[3]:+.3f} at the input shafts (if --signs is right)")
    if "goal_readback" not in cap.labels:
        return
    print("\ndid each write land? The read-back compared, in raw counts, with the goal "
          "sent\nk frames earlier. 'lost' = a sent goal never seen in the 4 frames after it.")
    # Only the random-goal frames: a constant goal (the closing zero) matches
    # itself at every lag and would read as ~100 % everywhere.
    rand = np.zeros(len(cap), bool)
    for k, s in _of_kind(cap, "idle_write"):
        rand[cap.segment_slice(k)] = True
    for i in cap.ids:
        reg = cap.register("goal_readback", i)
        back = cap.raw("goal_readback", i)
        sent = cap.command(i)
        n = len(back)
        enc = np.array([reg.encode(v) if np.isfinite(v) else -1 for v in sent])
        seen = np.zeros(n, bool)
        shares = []
        for k in range(4):
            m = cap.ok[k:] & (enc[:n - k] >= 0) & rand[:n - k] & rand[k:]
            hit = m & (back[k:] == enc[:n - k])
            seen[:n - k] |= hit
            shares.append(f"k={k}: {100 * hit.sum() / max(m.sum(), 1):.1f} %")
        judged = (enc >= 0) & rand & (np.arange(n) < n - 4) & np.concatenate(
            [rand[4:], np.zeros(min(4, n), bool)])
        print(f"  id {i}: " + "  ".join(shares) +
              f"   lost {int((judged & ~seen).sum())} of {int(judged.sum())}")


def _stops(x: np.ndarray, t: np.ndarray, thr: float, minlen: int = 10) -> list:
    """``(angle, frame)`` per stop: >= minlen frames moving slower than thr deg/s.

    The angle is median-filtered over 11 frames first. A one-count encoder
    flicker is 0.26 deg at the input shaft, which differenced over a few frames
    reads as ~30 deg/s -- faster than a 1 % creep -- so without the filter a
    genuinely stopped shaft never counts as stopped.
    """
    g = np.flatnonzero(np.isfinite(x) & np.isfinite(t))
    if len(g) < 40:
        return []
    xs, ts, h, k = x[g], t[g], 5, 5
    xs = np.concatenate([xs[:k], np.median(
        np.lib.stride_tricks.sliding_window_view(xs, 2 * k + 1), axis=1), xs[-k:]])
    speed = np.full(len(xs), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        speed[h:-h] = np.abs((xs[2 * h:] - xs[:-2 * h]) / (ts[2 * h:] - ts[:-2 * h]))
    still = np.nan_to_num(speed, nan=np.inf) < thr
    out, start = [], None
    for j in range(len(still) + 1):
        s = still[j] if j < len(still) else False
        if s and start is None:
            start = j
        elif not s and start is not None:
            if j - start >= minlen:
                pos, last = float(np.median(xs[start:j])), int(g[j - 1])
                # A long dwell broken by a flicker is ONE stop, not several
                # 0 deg apart; keep its last frame, which is the breakaway.
                if out and abs(pos - out[-1][0]) < 1.0:
                    out[-1] = (out[-1][0], last)
                else:
                    out.append((pos, last))
            start = None
    return out


def _angle_ripple(x: np.ndarray, t: np.ndarray, lo: float = 1.0, hi: float = 30.0,
                  step: float = 0.1) -> tuple:
    """Velocity ripple as a spectrum over ANGLE -> (peak period deg, peak/median, 7.5 deg/median).

    Resampling onto a uniform angle grid is what makes a detent a sharp line:
    it repeats per degree turned, however unevenly in time the degrees came.
    Needs at least 4 x `hi` degrees of travel.
    """
    nan = (float("nan"),) * 3
    sgn = 1.0 if x[-1] >= x[0] else -1.0
    xm = np.maximum.accumulate(sgn * x)
    if xm[-1] - xm[0] < 4 * hi:
        return nan
    h = 3
    v = np.zeros(len(xm))
    dt = t[2 * h:] - t[:-2 * h]
    v[h:-h] = np.where(dt > 0, (xm[2 * h:] - xm[:-2 * h]) / np.where(dt > 0, dt, 1.0), 0.0)
    grid = np.arange(xm[0], xm[-1], step)
    vg = np.interp(grid, xm + np.arange(len(xm)) * 1e-9, v)
    vg = (vg - vg.mean()) * np.hanning(len(vg))
    power = np.abs(np.fft.rfft(vg)) ** 2
    f = np.fft.rfftfreq(len(vg), step)                  # cycles per degree
    band = (f >= 1.0 / hi) & (f <= 1.0 / lo)
    pb, fb = power[band], f[band]
    med = np.median(pb)
    j, j75 = int(np.argmax(pb)), int(np.argmin(np.abs(fb - 1 / 7.5)))
    return 1.0 / fb[j], pb[j] / med, pb[j75] / med


def after_creep(cap: Capture) -> None:
    sa, sb = cap.meta["signs"]
    belt = cap.meta["params"]["belt_ratio"]["value"]
    ia, ib = cap.ids
    pa, pb = cap.position_rad(ia), cap.position_rad(ib)
    t = cap.time_s(ia)
    la, lb = cap.value("Present Load", ia), cap.value("Present Load", ib)
    coord = {"common": np.degrees(belt * (sa * pa + sb * pb) / 2),
             "diff": np.degrees(belt * (sa * pa - sb * pb) / 2),
             "a": np.degrees(belt * sa * pa), "b": np.degrees(belt * sb * pb)}
    print("\ncreep, degrees at the input shafts: ring-vs-hub for diff, hub for common.")
    print("A DETENT keeps the same spacing and ripple period at both speeds; the loop's "
          "stick-slip\nspacing grows with speed. Mesh prediction 7.5 deg. Ripple columns "
          "are peak power over the\nmedian of the 1-30 deg band: ~1 is nothing, tens is a line.")
    w = {i: cap.diff_velocity(i, 5) for i in cap.ids}
    speed = {"common": np.degrees(belt * (sa * w[ia] + sb * w[ib]) / 2),
             "diff": np.degrees(belt * (sa * w[ia] - sb * w[ib]) / 2),
             "a": np.degrees(belt * sa * w[ia]), "b": np.degrees(belt * sb * w[ib])}
    th = cap.arrays["t_host"]
    by_gain = {}
    print("  stopped = time under 25 % of command; burst = p99 speed over command "
          "(both after the first 0.5 s).")
    print(f"  {'segment':<30s} {'rate/cmd':>8s} {'stops':>5s} {'stopped':>7s} {'burst':>5s} "
          f"{'spacing':>7s} {'R@7.5':>5s} {'period':>6s} {'peak':>6s} {'@7.5':>6s} "
          f"{'|load| at break':>15s}")
    for k, s in _of_kind(cap, "creep"):
        sl, mode = cap.segment_slice(k), s["info"]["mode"]
        x, ts = coord[mode][sl], t[sl]
        g = np.isfinite(x) & np.isfinite(ts)
        if g.sum() < 50:
            continue
        cmd = np.degrees(belt * abs(s["info"]["v"]))
        rate = abs(x[g][-1] - x[g][0]) / (ts[g][-1] - ts[g][0])
        stops = _stops(x, ts, 0.25 * cmd)
        pos = np.array([p for p, _ in stops])
        spacing = _q(np.median, np.abs(np.diff(pos))) if len(pos) > 2 else float("nan")
        R = abs(np.mean(np.exp(2j * np.pi * pos / 7.5))) if len(pos) > 2 else float("nan")
        brk = [0.5 * (abs(la[sl][j]) + abs(lb[sl][j])) for _, j in stops]
        per, peak, at75 = _angle_ripple(x[g], ts[g])
        sgn = 1.0 if s["info"]["frac"] > 0 else -1.0
        late = (th[sl] - th[sl][0]) > 0.5
        v = sgn * speed[mode][sl][late] / cmd
        v = v[np.isfinite(v)]
        stopped = float(np.mean(v < 0.25)) if len(v) else float("nan")
        burst = float(np.percentile(v, 99)) if len(v) else float("nan")
        brk_med = _q(np.median, brk) if brk else float("nan")
        print(f"  {s['label']:<30s} {rate / cmd:8.2f} {len(stops):5d} {100 * stopped:6.0f}% "
              f"{burst:5.1f} {spacing:7.2f} {R:5.2f} {per:6.2f} {peak:6.1f} {at75:6.1f} "
              f"{brk_med:15.3f}")
        key = (s["info"].get("kvp"), s["info"].get("kvi"), mode, abs(s["info"]["frac"]))
        by_gain.setdefault(key, []).append((stopped, burst, len(stops), rate / cmd, at75, brk_med))
    print("\nR@7.5 is how tightly the stops sit on a 7.5 deg grid (1 = exactly; chance ~1/sqrt(stops)).")
    if any(key[0] is not None for key in by_gain):
        print("\nper gain pair, both directions averaged:")
        print(f"  {'P':>4s} {'I':>5s} {'mode':6s} {'speed':>5s} {'stopped':>7s} {'burst':>5s} "
              f"{'stops':>5s} {'rate/cmd':>8s} {'@7.5':>6s} {'|load| break':>12s}")
        for key in sorted(by_gain, key=lambda kk: (kk[2], kk[3], kk[0] or 0, kk[1] or 0)):
            m = np.nanmean(np.array(by_gain[key], float), axis=0)
            print(f"  {key[0] or '-':>4} {key[1] or '-':>5} {key[2]:6s} {key[3]:5.0%} "
                  f"{100 * m[0]:6.0f}% {m[1]:5.1f} {m[2]:5.0f} {m[3]:8.2f} {m[4]:6.1f} {m[5]:12.3f}")


AFTER = {"idle": after_idle, "jog": after_jog, "belt": after_belt,
         "step": after_step, "reversal": after_reversal, "chirp": None,
         "coast": after_coast, "hold": after_hold, "creep": after_creep}
PLANS = {"idle": plan_idle, "jog": plan_jog, "belt": plan_belt,
         "step": plan_step, "reversal": plan_reversal, "chirp": plan_chirp,
         "coast": plan_coast, "hold": plan_hold, "creep": plan_creep}


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


def report(cap: Capture) -> None:
    for fn in (print_summary, AFTER.get(cap.meta.get("test"))):
        if fn is None:
            continue
        try:
            fn(cap)
        except Exception as e:                 # noqa: BLE001 -- the capture is saved
            print(f"\n({fn.__name__} failed: {type(e).__name__}: {e}; the capture "
                  f"is intact)")


def drivetrain_params() -> dict:
    """`drivetrain` + the XC430 block as WRITTEN, with their `source:` fields."""
    raw = yaml.safe_load(Path(DEFAULT_PARAMS).read_text())
    return {"belt_ratio": raw["drivetrain"]["belt_ratio"],
            "drivetrain": raw["drivetrain"],
            "xc430_w150": raw["servos"]["xc430_w150"]}


def run(args) -> int:
    test = args.test
    tee = Tee(sys.stdout)
    sys.stdout = tee
    args.params = drivetrain_params()
    bus = plan = cap = None
    orig = before = after = None
    torque_on, mode_changed, notes = False, False, {}
    try:
        if not args.port:
            raise SystemExit("no --port given and AOW_DXL_PORT is unset")
        if not args.note:
            print("note: no --note, so the capture will not say what was on the rig "
                  "(wheel? disc? belt? held how?)")
        bus = DynamixelBus(args.port, baud=args.baud, ids=tuple(args.ids)).open()
        wrong = {i: ct.name for i, ct in bus.tables.items() if ct.name != MODEL}
        if wrong:
            raise SystemExit(f"this station is {MODEL} only; found {wrong}")
        errs = bus.hardware_errors()
        if errs and args.reboot:
            print(f"rebooting {sorted(errs)} to clear: "
                  f"{ {i: describe_hardware_error(e) for i, e in errs.items()} }")
            bus.reboot(list(errs))
            errs = bus.hardware_errors()
        if errs:
            raise SystemExit(
                f"latched hardware error "
                f"{ {i: describe_hardware_error(e) for i, e in errs.items()} }: "
                f"torque would be refused. Find the cause, then pass --reboot.")

        orig = bus.snapshot()
        lsb = bus.tables[bus.ids[0]]["Velocity Limit"]
        ctx = Ctx(bus, args,
                  no_load=args.params["xc430_w150"]["no_load_rpm"]["value"] * 2 * np.pi / 60,
                  v_limit=min(lsb.decode(orig[i]["Velocity Limit"]) for i in bus.ids))
        plan = PLANS[test](ctx)

        bus.prepare()                   # torque off, Return Delay 0, profiles 0
        if plan.mode is not None and any(orig[i]["Operating Mode"] != plan.mode
                                         for i in bus.ids):
            for i in bus.ids:
                bus.write_raw(i, "Operating Mode", plan.mode)
            mode_changed = True
        imap = IndirectMap(bus.tables)
        for name in READ_BLOCK:
            imap.read(name)
        if plan.write:
            imap.read(plan.write, label="goal_readback")
            imap.write(plan.write, label="goal")
        bus.apply_map(imap, verify=True)
        before = bus.snapshot()

        total = sum(s.seconds for s in plan.segments)
        v = {i: before[i]["Present Input Voltage"] * 0.1 for i in bus.ids}
        print(f"\n{test}: {len(plan.segments)} segments, {total:.1f} s at "
              f"{args.rate:g} Hz; ids {list(bus.ids)}, signs {args.signs}; "
              f"supply {v} V; Velocity Limit {ctx.v_limit:.2f} rad/s")
        if plan.brief:
            print(plan.brief)
        if plan.torque:
            if not args.yes:
                try:
                    input(f"\nTorque ON for ids {list(bus.ids)}. Hands and hair clear "
                          f"of the belts, rig held, then Enter (Ctrl-C aborts): ")
                except EOFError:
                    raise SystemExit("no terminal to confirm on; pass --yes") from None
            if plan.write:
                # RAM keeps whatever goal the last run left; torque on drives
                # to it until the first frame overwrites it.
                bus.write_frame({i: 0.0 for i in bus.ids})
            bus.bus_watchdog(WATCHDOG_MS)
            bus.torque(True)
            torque_on = True
        cap = record(bus, imap, plan.segments, args.rate,
                     finish=_stop_traffic(plan, notes) if torque_on else None)
    finally:
        if bus is not None and bus._port is not None:
            if torque_on and "watchdog_tripped" not in notes:   # finish did not get to it
                r = _try(bus.watchdog_tripped)
                notes["watchdog_tripped"] = None if r is _FAILED else r
            if plan is not None and plan.write:
                # Torque on or not: a goal left in RAM is what the servo drives
                # to the moment anything next enables torque.
                _try(bus.write_frame, {i: 0.0 for i in bus.ids})
                if torque_on:
                    time.sleep(0.3)             # brake to rest before letting go
            if _try(bus.torque, False) is _FAILED:
                print("!! TORQUE OFF FAILED -- cut power to the servos")
            if torque_on:
                _try(bus.bus_watchdog, 0)
            if mode_changed:
                _try(lambda: [bus.write_raw(i, "Operating Mode", orig[i]["Operating Mode"])
                              for i in bus.ids])
                notes["restored_operating_mode"] = True
            if orig is not None and (mode_changed or args.gains):
                # after the mode, never before: a mode write resets gains
                _try(lambda: [(bus.write_raw(i, "Velocity P Gain", orig[i]["Velocity P Gain"]),
                               bus.write_raw(i, "Velocity I Gain", orig[i]["Velocity I Gain"]))
                              for i in bus.ids])
                notes["restored_gains"] = True
            after = _try(bus.snapshot)
            bus.close()
        sys.stdout = tee.stream
        if cap is not None:
            cap.meta.update(
                test=test, note=args.note, signs=list(args.signs), argv=sys.argv,
                port=args.port, baud=args.baud, params=args.params,
                git=git_state(ROOT), python=sys.version.split()[0],
                platform=platform.platform(), watchdog_ms=WATCHDOG_MS if torque_on else 0,
                plan={"write": plan.write, "operating_mode": plan.mode,
                      "torque": plan.torque, "read_block": list(READ_BLOCK)},
                snapshot_initial=orig, snapshot_before=before,
                snapshot_after=None if after is _FAILED else after, shutdown=notes)
            out = new_capture_dir(Path(args.out), test, args.tag)
            cap.save(out)
            sys.stdout = tee
            print(f"\nsaved -> {out.relative_to(ROOT) if out.is_relative_to(ROOT) else out}")
            report(cap)
            sys.stdout = tee.stream
            (out / "log.txt").write_text("".join(tee.lines))
    return 0 if cap is not None and cap.meta["complete"] else 1


_FAILED = object()


def _stop_traffic(plan: Plan, notes: dict) -> Callable:
    """`record`'s finish hook: read the watchdog, zero the goal, disarm.

    Runs before the frames are converted, because that conversion is the one
    pause in traffic this script makes. Both 89 s creep captures of 2026-09-13
    read Bus Watchdog clear on every frame and TRIPPED at shutdown; converting
    44k frames is tens of ms before GC and metadata, against a 100 ms watchdog.
    Harmless there (the goal was already zero), but it made the shutdown report
    cry wolf, and a tripped servo silently ignores goals.
    """
    def finish(bus):
        notes["watchdog_tripped"] = bus.watchdog_tripped()
        if plan.write:
            bus.write_frame({i: 0.0 for i in bus.ids})
        bus.bus_watchdog(0)
    return finish


def _try(fn, *a):
    try:
        return fn(*a)
    except Exception as e:                      # noqa: BLE001 -- shutdown must continue
        print(f"  shutdown step {getattr(fn, '__name__', fn)} failed: {e}")
        return _FAILED


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _gain(s: str) -> tuple:
    try:
        p, i = s.split(":")
        return int(p), int(i)
    except ValueError:
        raise argparse.ArgumentTypeError(f"--gains wants P:I, e.g. 100:1920; got {s!r}")


def main() -> int:
    common = argparse.ArgumentParser(add_help=False)
    g = common.add_argument_group("bus and output")
    g.add_argument("--port", default=os.environ.get("AOW_DXL_PORT"),
                   help="serial port (default $AOW_DXL_PORT)")
    g.add_argument("--baud", type=int, default=3_000_000)
    g.add_argument("--ids", type=int, nargs=2, default=[101, 102], metavar=("A", "B"))
    g.add_argument("--signs", type=float, nargs=2, default=[1.0, -1.0], metavar=("SA", "SB"),
                   help="servo sign that turns the hub forward (default 1 -1, from "
                        "the CAD, UNVERIFIED: run `jog`)")
    g.add_argument("--rate", type=float, default=500.0, help="frame rate [Hz], 0 = unpaced")
    g.add_argument("--out", default=str(ROOT / "traces" / "drivetrain"))
    g.add_argument("--tag", help="appended to the capture directory name")
    g.add_argument("--note", default="", help="what is on the rig and how it is held")
    g.add_argument("--yes", action="store_true", help="skip the Enter before torque on")
    g.add_argument("--reboot", action="store_true",
                   help="reboot a servo with a latched hardware error")

    gains = argparse.ArgumentParser(add_help=False)
    gains.add_argument("--gains", type=_gain, nargs="+", metavar="P:I",
                       help="repeat the schedule per Velocity P:I gain pair; "
                            "originals restored afterwards (factory 100:1920)")
    modes = argparse.ArgumentParser(add_help=False)
    modes.add_argument("--modes", nargs="+", choices=MODES, default=["common", "diff"])

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="test", required=True)

    p = sub.add_parser("idle", parents=[common],
                       help="torque off; frame rate, hand spin, or --write for bus timing")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--write", action="store_true",
                   help="SyncWrite a random Profile Velocity every frame, torque still off "
                        "(a Goal Velocity written with torque off is discarded)")

    p = sub.add_parser("jog", parents=[common], help="slow a/b/common/diff; check signs")
    p.add_argument("--frac", type=float, default=0.1, help="fraction of no-load")
    p.add_argument("--seconds", type=float, default=2.0)

    p = sub.add_parser("belt", parents=[common],
                       help="exactly N servo revs; count the tyre (common) or a roller (diff)")
    p.add_argument("--mode", choices=("common", "diff"), default="common")
    p.add_argument("--revs", type=float, default=2.0)
    p.add_argument("--frac", type=float, default=None,
                   help="fraction of no-load (default 0.08 common, 0.03 diff: slow "
                        "enough that every detent shows as a stop)")

    p = sub.add_parser("step", parents=[common, gains, modes], help="D1 velocity steps")
    p.add_argument("--fracs", type=float, nargs="+", default=[0.02, 0.05, 0.1, 0.2, 0.5, 1.0])
    p.add_argument("--hold", type=float, default=1.0)
    p.add_argument("--rest", type=float, default=1.0)
    p.add_argument("--settle", type=float, default=1.0)
    p.add_argument("--one-sided", action="store_true", help="positive steps only")

    p = sub.add_parser("reversal", parents=[common, gains, modes], help="D2 square waves")
    p.add_argument("--amps", type=float, nargs="+", default=[0.1, 0.25, 0.5])
    p.add_argument("--freqs", type=float, nargs="+", default=[5, 10, 15, 25])
    p.add_argument("--cycles", type=float, default=8.0)

    p = sub.add_parser("chirp", parents=[common, gains, modes], help="log sine sweep")
    p.add_argument("--amp", type=float, default=0.15, help="fraction of no-load")
    p.add_argument("--f0", type=float, default=0.5)
    p.add_argument("--f1", type=float, default=30.0)
    p.add_argument("--seconds", type=float, default=20.0)

    p = sub.add_parser("coast", parents=[common, modes], help="D4 torque-off vs PWM-0 decay")
    p.add_argument("--duty", type=float, default=0.4)
    p.add_argument("--spin", type=float, default=1.5)
    p.add_argument("--decay", type=float, default=3.0)
    p.add_argument("--repeats", type=int, default=2)

    p = sub.add_parser("hold", parents=[common, gains], help="D3 hold while you push")
    p.add_argument("--mode", choices=MODES, default="common")
    p.add_argument("--frac", type=float, default=0.0)
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--baseline", type=float, default=2.0)

    p = sub.add_parser("creep", parents=[common, gains, modes],
                       help="slow constant diff/common at two speeds; gear detents")
    p.add_argument("--fracs", type=float, nargs="+", default=[0.01, 0.03],
                   help="speeds as fractions of no-load; use at least two")
    p.add_argument("--seconds", type=float, default=10.0, help="per speed, per direction")

    p = sub.add_parser("summary", help="re-print a saved capture's quick look")
    p.add_argument("capture", type=Path)

    args = ap.parse_args()
    if args.test == "summary":
        report(Capture.load(args.capture))
        return 0
    for attr in ("gains",):
        if not hasattr(args, attr):
            setattr(args, attr, None)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
