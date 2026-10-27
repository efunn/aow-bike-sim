"""The onboard process: three threads, four failsafes, one control loop.

    python -m aow_sim.hw.run_bike --bundle deploy/bundle.npz

Structure (docs/plans/untethered-setup.md):
  control thread  SCHED_FIFO, 100 Hz — SyncRead -> DriveController -> SyncWrite
  ahrs thread     200 Hz             — UART -> latest-value slot
  link thread      50 Hz             — UDP command in, telemetry out

100 Hz, not the simulator's 200: `DriveController._gen_every` already retimes
the 50 Hz general policy against whatever the controller rate is, so 100 Hz
ticks driving a 50 Hz policy is the designed path and needs no code change.
Raise it only once measured jitter says you can.

WHY THE COMMAND LINK IS WiFi: the operator sends a whole command struct —
velocity vector, heading, controller mode, move triggers, re-zero — not two
analog axes. That is the surface run_drive.py's teleop already exposes, and it
is why an RC receiver was rejected. The cost is that WiFi has no failsafe of
its own, so all four below are mandatory, not optional.

FAILSAFES, and the one that is not a failsafe
  1. command age  >150 ms -> zero the velocity command (the policy keeps
                   balancing, which IS the safe state); >1 s -> torque off.
                   Armed only once the ground station has been heard from --
                   see `wait_for_link` -- and switched off entirely by
                   --no-link for a bench session with no operator.
  2. pack voltage: warn below 10.5 V, CUT AND HOLD below 9.9 V (3.5 / 3.3
                   V/cell), on the highest servo's Present Input Voltage
                   low-passed over 2 s -- `PackMonitor`. No extra hardware.
  3. AHRS stale   -> torque off (raised by AhrsReader.latest).
  *. |roll| past the cut angle is NOT in that list any more. It is the one
     condition that is expected, survivable and self-clearing, so it is a
     state transition and not a reason to stop: `FallGuard` drops torque on
     the three servos the policy drives, leaves the righting servo live, and
     lets the policy back in once the bike has settled. A bike that has to be
     power-cycled after every tip-over cannot be tested.
A physical switch cutting servo power independently of the Pi is the last one,
and the only one that still works if this process is the thing that failed.
"""

from __future__ import annotations

import argparse
import json
import errno
import select
import signal
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

from collections import deque

from ..params import load_params
from ..control.drive import DriveController
from ..control.flick import move_path
from ..control.balance import mix
from ..control.recovery import HANDOFF_RATE, RECOVER_DEG, ready_for_policy
from ..control.swing_sit import SwingSit
from . import telemetry
from .bench_log import RunRecorder, new_capture_dir
from .ahrs import (QOS_LIMITED, QOS_MIN_SERVICE, QOS_NAMES, QOS_UNKNOWN,
                   AhrsReader, MountCalibration)
from .bike_bus import CONTROL_HZ_DEFAULT, BikeBus, resolve_gains
from .dynamixel import BusReadError
from .odometry import VelocityEstimator, body_to_world
from .state import HardwareData, load_ahrs_mount, load_bundle

CONTROL_HZ = CONTROL_HZ_DEFAULT   # see hw/bike_bus.py
CMD_STALE_S = 0.15        # -> zero the command. PROVISIONAL, see note below
CMD_DEAD_S = 1.0          # -> torque off
PACK_WARN_V = 10.5        # 3.5 V/cell on 3S -> say so, once
PACK_CUT_V = 9.9          # 3.3 V/cell -> cut and hold. Pack total — see note
PACK_TAU_S = 2.0          # low-pass on the per-tick max servo reading
LINK_WAIT_S = 30.0        # how long to wait for the ground station before giving up
# Consecutive failed servo reads ridden out by holding the last state. One
# corrupt status packet used to end the session (rc=-3002 on the assembled
# bike, 2026-10-24); three in a row (30 ms at 100 Hz, well inside
# CMD_STALE_S) is a failing bus rather than a bad packet, and stops it.
BAD_READS_MAX = 3
# Where the onboard record goes: traces/ is gitignored and Dropbox-synced on
# the laptop, and the drivetrain bench captures already live beside it.
RECORD_ROOT = Path(__file__).resolve().parents[3] / "traces" / "bike"

# What the record keeps per tick besides the raw registers: name -> width.
# Everything the controller had, so a post-hoc question about a run is a load
# rather than a re-run. `state` is an index into FallGuard's state names,
# stored in meta as `states`.
BIKE_COLS = {
    "t": 1, "state": 1, "slip_ms": 1, "dt_ms": 1,
    "quat": 4, "gyro": 3, "accel": 3, "qos": 1, "roll": 1, "roll_rate": 1,
    "v_world": 2, "pos": 2, "psi": 1, "vlat_conf": 1, "steer": 1,
    "ctrl": 3,                      # drive_a, drive_b [input-shaft rad/s], steer [rad]
    "cmd_v_world": 2, "cmd_psi": 1, "link_age": 1,
    "righting_goal": 1, "righting_current": 1, "volts": 1,
    "controller": 1,                # index into telemetry.CONTROLLERS
}
# FallGuard.state. Appended, never reordered: records store the index.
# A crank this far out from stow, with the bike leaning onto it, is a bike
# resting on that blade: lie-down starts at `lower` from there, no policy.
REST_CRANK_DEG = 45.0
# Where the lie-down's `lower` ENDS: through centre to this far the other
# way, where the bike lying on its cage comes to rest (SwingSit `rest_deg`).
# MEASURED on the bike, 2026-10-26, lying on its left: a lower that stopped
# at centre back-drove from 2 to 26.5-28 deg when the current went (runs
# 261026-034200, -034517, -040340) and the bike dropped -68.7 -> -74.5 ->
# -73.7 deg at 32 deg/s -- the CLUNK. Lowered to 75 (run -040425), roll was
# already flat at -73.6 by ~23 deg and the blade only pressed past it.
# 25 stops just short of the rest, so the blade never props the bike up.
# The SIM puts this rest at ~77 deg (and the lie-down's sweep agreed with
# itself there); the bike is the authority.
LIE_REST_DEG = 25.0
# The crank's speed both ways, lift and lower. The lower was the sim's 30
# deg/s (tuned against a 211 deg/s drop at 120 there): on the bike, run
# 261026-041350, down took 5.3 s against 1.25 s up, roll following the
# crank at ~15 deg/s with a 37-60 deg/s peak; the lift peaks at 137-144.
# Matched at the lift's 120 (user, 2026-10-26).
SIT_DPS = 120.0
# The lie-down's ROLL against its crank, quasi-static, in the side's frame
# (SwingSit `lower_map`): the lower is ramped in roll along this, at
# LIE_ROLL_DPS, rather than at a constant crank rate. MEASURED on the bike,
# 2026-10-26: medians of six lowers at 30 deg/s, both sides (runs
# 261026-040425 left x2, -041350 right x4), which agree to ~1 deg sd; the
# 110 point is the settle pose (-162523: crank stalled at 107, roll 8.1).
# At -10 the cage has the bike (74 deg): past it the blade only lifts off,
# at SIT_DPS. At 120 deg/s crank the roll ran ~5 deg ahead of this from 60
# down, waited on the blade from 50 to 30 (the PAUSE, run -042118), then
# overshot the cage to 78 and bounced back to 74.
LIE_MAP = ((110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0, -10),
           (8.0, 10.7, 17.3, 23.8, 29.4, 34.4, 39.2, 44.0, 48.9, 54.1, 61.9,
            69.2, 73.4))
# Roll's peak rate and acceleration along that: 1.44 s from a prop at 102
# (the get-up's lift is 1.25 s), crank peaking at 125 deg/s (lift: 137-144).
LIE_ROLL_DPS = 60.0
LIE_ROLL_ACC = 240.0
# THE STEER while the sequence holds it: straight ahead, or SIT_STEER_DEG
# toward the side the bike leans (into the lean; + steer is left, + side
# right), whichever of the two straight poses (delta and delta + 180: the
# policy sees sin/cos 2 delta) is nearer -- never a half turn. Ramped at
# SIT_STEER_DPS, at ANY roll: a bike lying with its wheel turned rests on
# the wheel, and the short way back to straight only lifts it off (user,
# 2026-10-26; a first version held it until 30 deg of upright). Held through
# `propped` (only the drives go limp), so the policy is handed a straight
# wheel: get-ups with it turned were failing.
SIT_STEER_DEG = 0.0
SIT_STEER_DPS = 180.0
# PUMPING A STALLED GET-UP (SwingSit `pump`: push while the bike swings up
# on its wing, limp while it swings back) is OFF: one attempt, and a stall
# lowers and waits for the next press. Flown 10-26 (run 261026-195319, power
# brick, Goal Current turned down to 485 / 565): neither pumped lift got up,
# and the second ended in the righting servo's OVERLOAD -- 5.8 s above 400
# mA. In the sim (46-48 % Goal Current) it got 4 of 6 stalls up, on a bike
# that barely rocks and a crank that back-drives far too freely. Set > 0 to
# try it again; keep it under ~3 s, and see the overload above.
PUMP_S = 0.0
# THE CRANK WHILE PROPPED: let go (Goal Current ramped to 0 over the
# sequence's limp_s). The linkage holds a prop unpowered (user, 2026-10-27),
# and held, the servo drew 0.2-0.35 A against the bike doing nothing (runs
# 261026-041350 to -175020) -- once for 29.6 s -- with the overload a risk
# the longer it sits. 1.0 holds it as before.
PROP_CURRENT = 0.0
# WHERE THE GET-UP'S LIFT AIMS, per side (SwingSit `lift_to_deg`): "stroke"
# is the full stroke, a number a crank angle, None the 110 lean. HANDED
# because the bike is. LEFT to the stroke: at 110 its push eased off over
# the last 10-15 deg (PWM 577-695 of 885) and it propped at 14-16 deg roll,
# outside Enter's window; at the stroke it props at 12 (run 261027-145541).
# RIGHT at 110: at the stroke it reached its prop at -67 to -97 deg/s of
# roll and went over the top twice in four; at 110, -21 / -35 deg/s and it
# stayed (run -143131).
LIFT_TO_DEG = {"left": "stroke", "right": None}
# ENTER FROM A PROP: its own window, wider than the re-arm after a fall
# (REARM_ROLL_DEG, 12). The blade stays planted through `balance` -- a
# kickstand until the bike is upright -- and the props the bike rests on
# reach past 12: by hand, with the crank unpowered (user, 2026-10-27),
# left blade roll +1 at crank -116 and -13.6 at -96.9, right +4.8 at +109
# and +14.8 at +94. 15 takes all four. The sim's recoverable set says 12;
# this is the bike's call, and it is untested from past 12.
PROP_ENGAGE_DEG = 15.0
# ENTER FROM A PROP PAST THAT WINDOW (SwingSit `nudge`): the crank powered
# again and ramped out, then the hand-off once the roll is 0.5 deg inside.
# What it buys is the LET-GO prop's sag back: unpowered, the left-side
# get-ups settled at -15.3 to -15.7 deg roll, the crank back-driving ~3 deg
# (-97.5 -> -94.4, run 261027-143131), where held at full current the same
# side propped at -12.3 to -13.0 (run -135627). It cannot go further: there,
# against a 12 deg target, the crank moved under 2 deg at the stroke end.
# Only from a bike this still (NUDGE_STILL_DPS), leaning onto its blade.
# OFF: it has never got a prop inside the window on the bike -- at 100 %
# PWM from a standstill the crank moved ~3 deg (run 261027-144242) -- and
# the handed lift (LIFT_TO_DEG) now props the left side at ~12. Kept, like
# PUMP_S; Enter past the window is refused and says so.
NUDGE = False
NUDGE_DPS = 10.0
NUDGE_STILL_DPS = 10.0
# "sit": the swing-sit sequence is running (BikeRunner._sit_step); "lying":
# it has finished lying down and holds the bike there until a get-up;
# "propped": a get-up has stood the bike on a blade, drives limp (steer held
# straight), and waits for Enter to hand it to the policy.
STATES = ("engaged", "cut", "standby", "sit", "lying", "propped")

# Defaults for the fall guard, overridable from control.onboard in
# bike_params.yaml and from the command line.
#
# CUT: 60 deg is `fall_roll_deg` from rl_general.yaml -- the angle beyond which
# the policy never trained -- NOT a claim about recoverability.
#
# RE-ARM: NOT defined here. `control/recovery.py` owns the criterion and
# `control/righting.py`'s sequencer ends its lift phase on the same call, so
# there is one definition of "ready for the policy" and not two that drift.
# The numbers come from analysis/policy/no_return.py. An earlier version of this file
# had 30 deg / 60 deg/s of its own invention; 30 is outside the policy's
# recoverable set on its weak side by a factor of three.
CUT_ROLL_DEG = 60.0
REARM_ROLL_DEG = RECOVER_DEG
REARM_RATE_DPS = np.degrees(HANDOFF_RATE)
# The sequencer has no dwell -- it hands off the instant both hold, because the
# mechanism is propping the bike and waiting buys nothing. This keeps a short
# one anyway: onboard the roll comes from an AHRS rather than from mjData, and
# a single noisy sample should not re-arm a bike.
REARM_DWELL_S = 0.2

GRAVITY = 9.80665
PREFLIGHT_ACCEL_TOL = 0.15               # fraction of g
PREFLIGHT_GYRO_MAX = np.deg2rad(2.0)     # at rest

# CMD_STALE_S is a round number, not a measurement. Two things can put the real
# command-age distribution's tail past it:
#
#   * `brcmfmac` enables WiFi power save by DEFAULT, which spikes latency by
#     tens to hundreds of ms — squarely inside this window. The bike then zeroes
#     its velocity command with no external cause, i.e. the failsafe firing
#     correctly on a fault that does not exist. Fix it at the OS level
#     (`iw dev wlan0 set power_save off`, made persistent) and assert it at
#     startup the same way BikeBus asserts latency_timer.
#   * Client reconnect after an AP hiccup takes seconds, which is past
#     CMD_DEAD_S as well — that surfaces as an unexplained torque-off.
#
# Set this from a measured p99 before the first untethered run. See
# docs/plans/untethered-setup.md, "The radio, and what actually goes wrong with
# it" and Verification step 2b.
#
# The pack limits read the pack total via the servos' address 144, which cannot see
# individual cells. A pack with one weak cell can sit at 3.6/3.6/3.0 V and still
# report 10.2 V, so the weak cell is over-discharged with nothing onboard to
# notice. That is an accepted limitation — the free voltage read is still the
# right call — but it assumes regular balance charging keeps the cells matched
# enough for the average to mean something.


class FallGuard:
    """Cut the policy out when the bike is on its side; let it back in.

    WHAT THIS IS NOT. It is not a detector for "unrecoverable". Unrecoverability
    is not a roll angle at all: `analysis/policy/no_return.py` measures the point of no
    return at roll -3.2...13.6 deg with the bike still looking upright, because
    what has run out there is crawl authority, not lean margin. The recoverable
    set is a curve in (roll, roll rate) and it moves with forward speed. A
    single threshold cannot express it, and one placed at 60 deg is a long way
    past it. See `docs/plans/self-righting.md` section 1.

    What it IS: the safety cut. Past `cut_deg` the bike is down, the policy is
    extrapolating outside anything it trained on (60 deg is `fall_roll_deg`,
    the training termination), and the useful behaviour is to stop driving so
    it does not thrash on its side. CUT_ROLL_DEG is a default, not a constant:
    a bike that comes to rest on a wing or a wheel at 30-45 deg wants the
    threshold moved, so it is configurable from `control.onboard` and from the
    command line.

    ONE CRITERION, SHARED WITH THE SEQUENCER. `control/righting.py`'s
    `RightingSequencer` has run lift -> balance -> retract since 2026-08-14,
    and its lift phase ends on exactly the question this class asks to come
    back: is the bike inside the set the policy can recover from? That
    predicate now lives in `control/recovery.py` and BOTH call it, so there is
    no second copy to drift. This class is not a reimplementation of that one;
    it is the onboard wrapper -- a cut (which the sequencer has no notion of,
    because its premise is a bike that is already down), a dwell, and consent
    -- around the same test.

    The sequencer's CODE is not shared and cannot be: it imports mujoco, drives
    a MuJoCo actuator and schedules a stroke against a model. When the
    mechanism is physically built, the intended end state is one sequence
    onboard, of which this guard is the `balance` phase and `keep_policy`
    becomes the "never cut at all" option. Until the mechanism exists there is
    no lift and no retract to port, which is why this is a guard and not a
    sequencer yet.

    Coming back needs three things beyond the angle, each for its own reason:

      * HYSTERESIS (`rearm_deg` < `cut_deg`), or the guard chatters against
        the threshold on every wobble through it.
      * A DWELL on angle AND rate together. A bike swinging through upright at
        5 rad/s is momentarily inside the angle window and is not ready to
        drive; requiring both to stay small for `dwell_s` is what distinguishes
        "settled" from "passing through".
      * CONSENT, unless `auto`. Re-engaging by itself while someone has their
        hands on the bike is worse than waiting to be told.

    Pure -- no clock of its own, no I/O -- so the whole state machine is
    testable without a bike. `update` returns the EVENT ("cut", "rearm", or
    ""), not the state; the state is on the object.
    """

    def __init__(self, cut_deg: float = CUT_ROLL_DEG,
                 rearm_deg: float = REARM_ROLL_DEG,
                 rearm_rate_dps: float = REARM_RATE_DPS,
                 dwell_s: float = REARM_DWELL_S, auto: bool = False):
        if rearm_deg >= cut_deg:
            raise ValueError(
                f"rearm_deg {rearm_deg} must be below cut_deg {cut_deg}, or the "
                "guard has no hysteresis and will chatter at the threshold")
        self.cut = np.deg2rad(cut_deg)
        self.rearm_deg = float(rearm_deg)
        self.rearm = np.deg2rad(rearm_deg)
        self.rearm_rate = np.deg2rad(rearm_rate_dps)
        self.dwell_s = float(dwell_s)
        self.auto = bool(auto)
        self.state = "engaged"
        self.consent = False
        self._settled_since = None
        self.cuts = 0
        # True once the OPERATOR put the bike in standby (`standby()`): at
        # connect, or by disengaging. Unlike a cut it is never left
        # automatically -- --auto-rearm is about falls, and a bike someone
        # deliberately stood down must not re-engage because it sat still.
        # While True, nothing re-arms -- not consent, not --auto-rearm. Set by
        # the runner while the link is down: there is nobody to consent, and
        # an auto re-arm with no operator would drive a bike nobody can stop.
        self.hold = False

    def cut_now(self) -> bool:
        """Cut for a reason that is not a fall (a lost link). Clears consent
        -- a go-ahead given before the link died is not one for after it --
        and does NOT count toward `cuts`, which counts falls. -> whether it
        was engaged, i.e. whether there was torque to take off."""
        was = self.state == "engaged"
        self.state = "cut"
        self.consent = False
        self._settled_since = None
        return was

    def standby(self) -> bool:
        """The operator's disengage (and the state at connect): policy out,
        like a cut, but left only on consent -- see `__init__`. Clears any
        consent given before it. -> whether it was engaged."""
        was = self.state == "engaged"
        self.state = "standby"
        self.consent = False
        self._settled_since = None
        return was

    def request_rearm(self) -> None:
        """The operator's go-ahead. Ignored while engaged; consumed on rearm."""
        self.consent = True

    def update(self, t: float, roll: float, roll_rate: float) -> str:
        if self.state == "engaged":
            if abs(roll) > self.cut:
                self.state = "cut"
                self.consent = False
                self._settled_since = None
                self.cuts += 1
                return "cut"
            return ""
        # THE SAME CALL the righting sequencer ends its lift phase on
        # (control/recovery.py). Everything else in this method -- the dwell,
        # the consent, the cut itself -- is the onboard wrapper around it.
        settled = ready_for_policy(roll, roll_rate, self.rearm_deg,
                                   self.rearm_rate)
        # Held: the dwell does not accrue either. Stillness nobody was there
        # to see is not a reason to re-arm the instant someone arrives.
        if not settled or self.hold:
            self._settled_since = None
            return ""
        if self._settled_since is None:
            self._settled_since = t
        if t - self._settled_since < self.dwell_s:
            return ""
        if not (self.consent or (self.auto and self.state == "cut")):
            return ""
        self.state = "engaged"
        self.consent = False
        self._settled_since = None
        return "rearm"


class PackMonitor:
    """Low-voltage cutoff on the servos' own Present Input Voltage.

    THE HIGHEST READING, NOT ANY ONE SERVO. Every servo measures the rail at
    its own connector, so a loaded servo reads its own wiring drop on top of
    the pack's sag. Measured on the 12 V brick, 2026-09-18 (torque1 runs):
    all four agree to 0.1 V at rest; with the drives pinned the spread was
    0.2-0.3 V typical and 0.7 V worst, drive A lowest (11.0 V against the
    righting servo's 11.5-12.0). The least-loaded servo is the one closest to
    the pack. The old LVC read drive A alone, as a raw 1 Hz sample -- the
    worst choice on both counts.

    LOW-PASSED, tau 2 s, updated every tick. A TX burst or a hard balancing
    move sags the rail for tens of ms; a flat pack stays low. The filter is
    the whole debounce -- there is no separate dwell.

    BOTH EVENTS LATCH for the session. A pack under load recovers a few
    tenths when the load stops, so an unlatched cut would re-arm on the
    rebound and cut again; the fix for a flat pack is a new pack, which means
    a restart anyway. Pure -- no clock, no I/O.
    """

    def __init__(self, warn_v: float = PACK_WARN_V, cut_v: float = PACK_CUT_V,
                 tau_s: float = PACK_TAU_S):
        if cut_v >= warn_v:
            raise ValueError(f"cut {cut_v} V must be below warn {warn_v} V")
        self.warn_v, self.cut_v, self.tau_s = float(warn_v), float(cut_v), float(tau_s)
        self.v: float | None = None       # filtered pack voltage [V]
        self.warned = False
        self.low = False

    def update(self, dt: float, readings) -> str:
        """One tick. -> "warn", "cut" or "" (the EVENT, as FallGuard does)."""
        vals = [float(r) for r in readings if r is not None and r > 0]
        if not vals:
            return ""
        top = max(vals)
        if self.v is None:
            self.v = top
        else:
            self.v += min(dt / self.tau_s, 1.0) * (top - self.v)
        if not self.low and self.v < self.cut_v:
            self.low = self.warned = True
            return "cut"
        if not self.warned and self.v < self.warn_v:
            self.warned = True
            return "warn"
        return ""


class CommandLink:
    """UDP command receive + telemetry transmit.

    The command arrives as 16 packed bytes (`telemetry.decode_command`) and is
    decoded straight back into the dict this class always held, so nothing
    downstream of the socket knows the wire format changed. A datagram that
    will not decode -- an old station still sending JSON -- is said ONCE and
    otherwise ignored: an unreadable command is not a heartbeat.
    """

    def __init__(self, listen=("0.0.0.0", 9910), telemetry_hz=CONTROL_HZ):
        self.addr = listen
        self.telemetry_hz = telemetry_hz
        self.cmd = {"v_cmd_world": [0.0, 0.0], "psi_cmd": 0.0}
        self.cmd_t = 0.0
        self.peer = None
        self._stop = threading.Event()
        self._thread = None
        self.telemetry = {}
        self._said: set = set()

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(self.addr)
        # select() owns the waiting; keep a short timeout only so a
        # spurious readable never parks the thread.
        self._sock.settimeout(0.01)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="link")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self):
        # ACCUMULATED DEADLINE, not "long enough since the last send". This
        # loop only wakes when a datagram arrives or the socket times out, so
        # its clock is the OPERATOR's send rate -- and `now - last_tx > period`
        # against that clock aliases: with commands at 50 Hz and the period at
        # 50 Hz, jitter makes roughly every other wake fail the comparison.
        # Measured 2026-09-16 against a station sending at 50 Hz: 27.8 Hz of
        # telemetry, which a mirrored viewer would render as judder and which
        # reads as "the radio is struggling" rather than as an arithmetic bug.
        # AND WAIT ON THE DEADLINE, not only on the socket. `recvfrom` with a
        # 0.1 s timeout wakes on a datagram or after 100 ms, so telemetry could
        # never go out faster than commands came in: at a 50 Hz station the
        # accumulated deadline still only got 34.7 Hz out, because it can only
        # fire on a wake. `select` with the time-to-deadline as its timeout
        # wakes for BOTH reasons, which is the same shape hw/ground.py's loop
        # already has on the other end of the wire.
        next_tx = 0.0
        while not self._stop.is_set():
            timeout = min(0.1, max(0.0, next_tx - time.monotonic()))
            select.select([self._sock], [], [], timeout)
            try:
                data, peer = self._sock.recvfrom(4096)
                try:
                    self.cmd = telemetry.decode_command(data)
                except telemetry.CommandFormatError as e:
                    if str(e) not in self._said:
                        self._said.add(str(e))
                        _say(f"ignoring a command from {peer[0]}: {e}")
                    continue
                self.cmd_t = time.monotonic()
                self.peer = peer
            except socket.timeout:
                pass
            except Exception:
                pass          # a malformed datagram must never stop the link
            now = time.monotonic()
            # `and self.telemetry`: the control loop fills this dict, and the
            # link thread starts first. Sending the empty one is not harmless
            # -- the station version-checks the first packet it sees, and an
            # empty dict reads as "schema vNone", i.e. a stale deploy. Silence
            # until there is something to say.
            if self.peer and self.telemetry and now >= next_tx:
                try:
                    self._sock.sendto(json.dumps(self.telemetry).encode(), self.peer)
                except OSError:
                    pass
                # Advance by whole periods, and never bank credit for a gap we
                # slept through -- otherwise a reconnect fires a burst.
                next_tx = max(now, next_tx + 1.0 / self.telemetry_hz)

    def age(self) -> float:
        return time.monotonic() - self.cmd_t if self.cmd_t else float("inf")


class BikeRunner:
    def __init__(self, bundle_path: str, port: str, ahrs_port: str,
                 control_hz: float = CONTROL_HZ, servo_gains=None,
                 require_link: bool = True, auto_rearm: bool = False,
                 ids=None, torque: bool = True, seconds: float = 0.0,
                 steer_zero=None, allow_guess_mount: bool = False,
                 record: bool = True, tag: str | None = None,
                 link_wait: float = 0.0, pack_warn_v: float = PACK_WARN_V,
                 pack_cut_v: float = PACK_CUT_V, move: str | None = None,
                 hold: bool = False):
        self.params = load_params()
        # HOLD MODE (--hold): every servo torqued, drives at zero velocity,
        # steer held where it is, and the policy never engages. For the tests
        # that want the bike locked rather than balancing (a traction pull, a
        # drop with the drives stiff). Enter / Y re-captures the pose and
        # holds it; Backspace goes limp.
        self.hold = bool(hold)
        self._holding = False
        self._hold_steer = 0.0
        self.pack = PackMonitor(pack_warn_v, pack_cut_v)
        self.bundle_path = bundle_path
        self.record, self.tag = bool(record), tag
        self.link_wait = float(link_wait)
        self.rec: RunRecorder | None = None
        self.run_id: str | None = None
        self._stopped_by: str | None = None
        self.design, self.model = load_bundle(bundle_path, self.params,
                                              rate_hz=control_hz)
        # WHICH CONTROLLER: the general policy unless the operator asks for
        # the LQR (station key `l`). The LQR only flies gains designed at THIS
        # loop's rate -- a discrete design is only itself at its own rate --
        # and only the 10-state (crawl-state) design; anything else refuses,
        # by name, rather than flying a controller nobody tested.
        self.controller = "policy"
        self.lqr_why = lqr_unavailable(self.design, control_hz)
        self._lqr_refused: str | None = None   # say a refusal once, not per tick
        self.dt = 1.0 / control_hz
        self.ctl = DriveController(controller_params(self.params, control_hz),
                                   self.model, self.design)
        self.aid = self.ctl.aid
        self.data = HardwareData(self.model.nq, self.model.nv, self.model.nu)
        cfg = ((self.params.get("control") or {}).get("onboard") or {})
        self.require_link = bool(require_link)
        # WHICH policy, resolved HERE rather than at engage time, because the
        # firmware gains have to follow it and they are written before torque.
        # `move` (--move) overrides control.general_move for one session, so
        # flying another policy is not an edit to bike_params on the Pi.
        self.gen_name = move or self.params["control"].get("general_move",
                                                           "general_rl")
        # Checked here, before torque: a typo would otherwise fail only at
        # the first engage, after the firmware gains fell back to factory.
        if move and not move_path(move, suffix=".npz").exists():
            raise SystemExit(f"--move {move}: no {move_path(move, suffix='.npz')}")
        gains, self.gain_note = resolve_gains(self.params, self.gen_name,
                                              override=servo_gains)
        # control_hz reaches the bus, and that is not cosmetic: RateFilter
        # quantises its 25 ms window to whole ticks of the NOMINAL period, so
        # without this every rate other than 100 Hz silently filters over a
        # different span than it reports. A bring-up is a rate sweep, so this
        # was wrong exactly when it was being used.
        # IDs from the config (or --ids), because the BENCH servos are 101-104
        # while the bike's are 1-3: renumbering them to run a timing test would
        # be a destructive EEPROM write to get a read-only measurement.
        ids = tuple(ids or cfg.get("servo_ids") or (1, 2, 3))
        # A FOURTH id means the righting servo, so one flag covers a bench
        # whose ids are 101-104 rather than 1-4. Three ids and the config's
        # righting_id is used; three ids and no config entry means no righting
        # servo at all, which is the common bench case.
        righting = ids[3] if len(ids) > 3 else cfg.get("righting_id")
        self.bus = BikeBus(self.params, port=port, control_hz=control_hz,
                            ids=ids[:3], righting_id=righting, gains=gains,
                            righting_current=cfg.get("righting_current"),
                            servo_sign=cfg.get("servo_sign", (1.0, 1.0)),
                            steer_sign=cfg.get("steer_sign", 1.0))
        # A PERSISTED CONSTANT BEATS A CAPTURE, and this is the bike's default.
        # The policy observes the steer as sin/cos(2*delta) -- pi-periodic,
        # because the wheel is front-back symmetric -- the servo's encoder is
        # absolute within one turn, and bike.steering.gear_ratio is 1.0. So the
        # power-up reading already fixes the steer angle mod pi, and nothing
        # needs homing: what is needed is ONE number, the encoder reading with
        # the wheel physically straight, measured once at assembly.
        #
        # Left unset, BikeBus captures the current pose as zero instead, which
        # is right for a bare bench shaft and WRONG on a chassis -- it would
        # define whatever angle the wheel happened to be at as straight ahead.
        # `steer_zero` overrides the config for one session: a number pins it,
        # the string "capture" forces the startup capture. THE BENCH NEEDS THE
        # SECOND ONE. The config now pins 180 deg, which is where the servo
        # will be clamped on the assembled bike -- so until that assembly
        # happens, a bare shaft sitting at some other angle would be told it is
        # 107 deg off straight and every bench number would inherit the offset.
        zero = cfg.get("steer_zero_deg") if steer_zero is None else steer_zero
        if isinstance(zero, str):
            if zero != "capture":
                raise ValueError("--steer-zero takes a number or 'capture'")
            zero = None
        if zero is not None:
            self.bus.steer_zero = np.deg2rad(float(zero))
        self.torque = bool(torque)
        # A BENCH RUN MUST END ITSELF. `timeout` sends SIGTERM, which kills the
        # process without running `finally: shutdown()` -- so the servos stay
        # ENERGISED after the program is gone, holding whatever they last had.
        # 0 means run until a failsafe or ctrl-C.
        self.seconds = float(seconds)
        q_mount, self.mount_source = load_ahrs_mount(bundle_path)
        self.ahrs = AhrsReader(ahrs_port,
                               calibration=MountCalibration(q_mount),
                               poll=bool(cfg.get("ahrs_poll", False)))
        self.est = VelocityEstimator(self.params)
        self.link = CommandLink()
        self.guard = FallGuard(
            cut_deg=float(cfg.get("cut_roll_deg", CUT_ROLL_DEG)),
            rearm_deg=float(cfg.get("rearm_roll_deg", REARM_ROLL_DEG)),
            rearm_rate_dps=float(cfg.get("rearm_roll_rate_dps", REARM_RATE_DPS)),
            dwell_s=float(cfg.get("rearm_dwell_s", REARM_DWELL_S)),
            auto=auto_rearm)
        # SWING-SIT (control/swing_sit.py, the sequence teleop runs on PageDown
        # / PageUp): lie down onto a blade and get back up, at the operator's
        # press (`lie_n` / `up_n`). Built only with a righting servo and the
        # module's stroke, never in --hold. While it owns the bike the fall
        # guard is not consulted -- lying down IS a roll past the cut -- and
        # the guard's `engaged` means only "policy servos torqued"; anything
        # that takes them off (disengage, lost link, pack, fault) ends it.
        # GET-UP STOPS PROPPED (hand_off=False): on a blade, drives limp,
        # until Enter -- the policy never takes the bike by itself. NO
        # AUTO-RETRY (retry_s=None): a failed get-up lies there until the
        # operator presses again. Retried every 1-4 s, a crank against a
        # wall and then a dead one kept lifting for a minute (run
        # 261026-173521).
        mod = (self.params.get("righting") or {}).get("module") or {}
        travel = ((mod.get("linkage") or {}).get("stroke") or {}).get(
            "crank_travel_deg")
        self.sit = (SwingSit(np.radians(float(travel)), hand_off=False,
                             rest_deg=LIE_REST_DEG, lift_dps=SIT_DPS,
                             lower_dps=SIT_DPS, lower_map=LIE_MAP,
                             lower_roll_dps=LIE_ROLL_DPS,
                             lower_roll_acc=LIE_ROLL_ACC, retry_s=None,
                             pump_s=PUMP_S, prop_current=PROP_CURRENT,
                             nudge_dps=NUDGE_DPS,
                             lift_to_deg=tuple(
                                 float(travel) if v == "stroke" else v
                                 for v in (LIFT_TO_DEG["left"],
                                           LIFT_TO_DEG["right"]))
                             if travel else None,
                             nudge_to_deg=max(PROP_ENGAGE_DEG,
                                              self.guard.rearm_deg) - 0.5)
                    if self.bus.id_right and travel and not self.hold else None)
        self._r_stow = np.radians(float(cfg.get("righting_stow_deg", 180.0)))
        self._r_sign = float(cfg.get("righting_sign", 1))
        self._sit_gc = None               # the operator's Goal Current, restored after
        self._sit_steer = None            # steer held while the policy is off
        self._sit_steer_t = None          # ... and when it was last moved
        self._sit_off = False             # policy off last tick: re-engage on return
        self._sit_psi = 0.0               # heading the policy holds meanwhile
        self._sit_said = "riding"
        self._lie_seen = self._up_seen = self._park_seen = None
        self._sit_engage_pending = False  # Enter started a nudge: engage after
        # BOUNDED, and not a list. It used to grow without limit while every
        # tick sliced the last 500 out of it and built a fresh numpy array for
        # the telemetry -- 1.13 ms typical and 49.5 ms worst case on a Pi 3B+,
        # measured, against a 10 ms budget. The jitter number was the jitter.
        self.jitter = deque(maxlen=2000)
        self._jitter_ms = 0.0
        # Ticks that missed badly, with their index. A p99 tells you there is a
        # problem; this tells you WHERE, which is what separates "the bus
        # stalled" from "something on the control thread blocked".
        self._late: list = []
        self._roll = self._roll_rate = self._yaw = 0.0
        self._dt_meas = self.dt
        self._last_servos = None          # the last good read, for _read_servos
        self._bad_reads = 0               # in a row
        self._bad_reads_total = 0
        self._read_ok = True              # this tick's read; the record's `ok`
        self._righting_current = None
        # Torque on the righting servo. NOT at connect: in standby it stays
        # unpowered, free to back-drive by hand, until something uses it --
        # the station's wing goal, a sit press, the stow at an engage
        # (`_arm_righting`). Only shutdown ever takes it off again.
        self._right_armed = False
        # Preflight findings whose KIND may be accepted without disarming the
        # rest of preflight. See _report_preflight.
        self._preflight_allow = ((MOUNT_UNCALIBRATED,) if allow_guess_mount
                                 else ())
        self._w_shaft = (0.0, 0.0)
        self._shaft = (0.0, 0.0)
        self._qos = None
        self._righting_pos = None
        self._accel = (0.0, 0.0, 0.0)
        self._health: dict = {}
        self._link_lost = False
        self._rearm_seen = None           # (peer, rearm_n) last heard
        self._stop_seen = None            # (peer, stop_n) last heard
        # What the bike tells the station, in words (telemetry.EventLog).
        self.events = telemetry.EventLog()
        self._errs: dict = {}             # role -> Hardware Error Status seen
        self._servo_fault = False         # a policy servo has a latched error
        self._righting_fault = False      # ... the righting servo has; no sequence
        self._pack_low = False            # PackMonitor cut; latched

    # -- one tick ----------------------------------------------------------

    def _sense(self) -> None:
        """Servos + AHRS -> HardwareData. Everything the controllers read.

        Time advances by the servos' OWN Realtime Tick delta, not by the rate
        we asked for. The tick is stamped when the servo sampled its encoder,
        so integrating over it keeps the estimator and the controller's
        zero-order hold honest when the loop jitters or drops a tick. Falls
        back to the nominal period on the first tick (no previous stamp) or if
        the delta looks implausible.
        """
        s = self._read_servos()
        a = self.ahrs.latest()

        dt = s["dt"] if s["dt"] and 0.2 * self.dt < s["dt"] < 5 * self.dt else self.dt
        self._dt_meas = dt
        self.data.time += dt
        self.data.set_orientation(a.quat, a.gyro)
        self.data.qpos[self.ctl._sj] = s["steer_pos"]
        self.data.qvel[self.ctl._sd] = s["steer_vel"]
        # The LQR's crawl states come from the drive servos' own position
        # counts (balance.CrawlSensor): these are the same filtered rates
        # odometry reads below. Untested on hardware as of 2026-09-22.
        self.ctl.crawl.feed(s["w_servo_a"], s["w_servo_b"])

        roll, pitch, yaw = _rpy(a.quat)
        self._sensed = (s, a, dt, roll, pitch, yaw)
        self._estimate()
        self.data.integrate_position(dt)
        self._roll = roll
        self._yaw = yaw
        self._roll_rate = float(a.gyro[0])
        # For telemetry only -- no controller reads either. The shaft speeds
        # are what the mirror integrates into a wheel angle (the drive servos
        # run in velocity mode and their single-turn position wraps, so there
        # is no absolute angle to send); the QoS is the AHRS grading itself,
        # already decoded out of the frame this tick used.
        self._w_shaft = (s["w_servo_a"] * self.bus.belt_ratio,
                         s["w_servo_b"] * self.bus.belt_ratio)
        self._shaft = (s["turned_a"], s["turned_b"])
        # WHERE THE RIGHTING SERVO ACTUALLY IS, alongside the goal. Read every
        # tick because it is already in the sync-read window -- the righting
        # servo is the fourth id on the same chain -- so this costs no bus time.
        self._righting_pos = s["righting_pos"]
        self._qos = a.qos
        self._accel = a.accel
        self._health = s["health"]

    def _read_servos(self) -> dict:
        """One servo read in controller units, riding out a bad frame.

        A failed read (`BusReadError`) repeats the last good state with the
        nominal period -- velocities held, positions frozen for a tick, which
        the next good read's Realtime Tick delta spans -- and says so. More
        than BAD_READS_MAX in a row, or a failure before any good read,
        stops the session as before."""
        try:
            s = self.bus.to_controller_units(self.bus.read_state())
        except BusReadError as e:
            self._bad_reads += 1
            self._bad_reads_total += 1
            self._read_ok = False
            if self._last_servos is None or self._bad_reads > BAD_READS_MAX:
                raise BusReadError(f"{self._bad_reads} bad servo read(s) in a "
                                   f"row, last: {e}") from e
            self._event("bus", f"bad servo read ({e}): holding the last state "
                               f"({self._bad_reads} in a row, "
                               f"{self._bad_reads_total} this run)")
            return {**self._last_servos, "dt": None}
        self._bad_reads = 0
        self._read_ok = True
        self._last_servos = s
        return s

    def _estimate(self) -> None:
        """This tick's sensor sample -> `self.est` -> the velocity in `data`.
        No bus I/O and no clock: `_engage` re-runs it on a fresh estimator."""
        s, a, dt, roll, pitch, yaw = self._sensed
        v_lon, v_lat = self.est.update(
            dt, s["w_servo_a"], s["w_servo_b"],
            steer_joint=s["steer_pos"], yaw_rate=a.gyro[2],
            accel_body=a.accel, roll=roll, pitch=pitch)
        self.data.set_velocity(body_to_world(v_lon, v_lat, yaw))

    def _apply_command(self) -> None:
        age = self.link.age()
        if age > CMD_STALE_S:
            # Hold heading, zero velocity: keep balancing, stop travelling.
            self.ctl.set_command(v_cmd_world=[0.0, 0.0])
            if self.controller == "lqr":
                self.ctl.set_speed(0.0)
            return
        c = self.link.cmd
        want = c.get("controller")
        if want is not None and want != self.controller:
            self._switch_controller(want)
        self.ctl.set_command(v_cmd_world=c.get("v_cmd_world", [0.0, 0.0]),
                             psi_cmd=c.get("psi_cmd"))
        if self._sit_owns():
            # The sequence's `stop` needs the bike at rest, and the balance
            # after a lift wants it still: the operator's command waits.
            self.ctl.set_command(v_cmd_world=[0.0, 0.0], psi_cmd=self._sit_psi)
        if self.controller == "lqr":
            self.ctl.follow_command(self.data, c.get("v_cmd_world", [0.0, 0.0]),
                                    c.get("psi_cmd"))
        self._apply_operator(c)

    def _switch_controller(self, want: str) -> None:
        """Policy <-> LQR, on the operator's request. Refused, and SAID,
        when the bundle cannot fly the LQR at this loop rate; the station
        shows what the bike is actually flying, so a refusal is visible."""
        if want == "lqr" and self.lqr_why is not None:
            if self._lqr_refused != self.lqr_why:
                self._event("mode", f"LQR refused: {self.lqr_why}")
                self._lqr_refused = self.lqr_why
            return
        self.controller = want
        self._lqr_refused = None
        if want == "lqr":
            self.ctl.follow_reset(self.data)
            self._event("mode", "LQR engaged -- position is DEAD-RECKONED and "
                                "re-anchors on each heading key")
        else:
            self.ctl.engage_general(self.data, name=self.gen_name, reuse=True)
            self.ctl.set_command(v_cmd_world=[0.0, 0.0])
            self._event("mode", f"policy {self.gen_name} engaged")

    def _apply_operator(self, c: dict) -> None:
        """The parts of the command struct the POLICY does not own.

        The self-righting servo is driven from here and nowhere else: four
        training runs established the general policy should not drive the wings
        (`docs/plans/self-righting.md`), so it is an operator control that
        happens to share a daisy chain. Goal Current is applied on change only
        -- it is a separate bus transaction, so sending it every tick would put
        an operator-scale knob on the control path.
        """
        if self.hold:
            self._hold_from(c.get("rearm_n"), c.get("stop_n"))
        else:
            self._rearm_from(c.get("rearm_n"))
            self._stop_from(c.get("stop_n"))
            self._sit_from(c.get("lie_n"), c.get("up_n"), c.get("park_n"))
        if not self.bus.id_right:
            return
        if self._sit_owns():
            # The station drops its goal on a lie/up press, so a goal here
            # while LYING is the operator taking the wings back (9/4, the
            # triggers): teleop's pad_crank does the same. Mid-sequence the
            # sequence keeps them. PROPPED likewise: taking them back there
            # leaves an unbalanced bike limp, which is the operator's call.
            # THE CURRENT is still the operator's: [ / ] move the Goal
            # Current the sequence scales from (and restores at the end).
            # It used to be dropped here, so the keys did nothing from a
            # lie-down to the next riding (user, 2026-10-26).
            want = c.get("righting_current")
            if want is not None and self._sit_gc is not None \
                    and int(want) != self._sit_gc:
                self._sit_gc = int(want)
                self._event("sit", f"Goal Current {self._sit_gc} for the sequence")
            if self.sit.phase not in ("lying", "propped") \
                    or "righting_rad" not in c:
                return
            self._sit_release("wings taken back by the operator")
        if "righting_rad" in c:
            self._arm_righting()
            self.bus.set_righting(c["righting_rad"])
        want = c.get("righting_current")
        if want is not None and want != self._righting_current:
            self._righting_current = self.bus.set_righting_current(int(want))

    def _arm_righting(self) -> None:
        """Torque on the righting servo, the first time anything drives it.
        Its goal is set to where the crank IS first, so arming holds it
        there rather than sending it to a stale goal from before it was
        moved by hand."""
        if self._right_armed or not (self.torque and self.bus.id_right):
            return
        if self.bus._righting_goal is None and self._righting_pos is not None:
            self.bus.set_righting(self._righting_pos)
        self.bus.arm((self.bus.id_right,))
        self._right_armed = True
        self._event("righting", "righting servo powered")

    def _stow_if_unset(self) -> None:
        """At an engage, send the wings to stow if nobody has commanded them.

        Without this the righting servo holds wherever torque found it, and
        the bike is started lying on a wing: run 261024-191558 flew 84 s with
        the crank 21.8 deg off stow and no goal ever sent. At the ENGAGE, not
        at startup, because at startup the bike is lying on that wing and
        retracting it there would shove the bike over. Once the station has
        sent a goal it owns the wings and this does nothing."""
        if not self.bus.id_right or self.bus._righting_goal is not None:
            return
        cfg = ((self.params.get("control") or {}).get("onboard") or {})
        stow = np.radians(float(cfg.get("righting_stow_deg", 180.0)))
        self._arm_righting()
        self.bus.set_righting(stow)
        self._event("righting", f"wings to stow ({np.degrees(stow):.0f} deg): "
                                "none commanded yet")

    def _hold_on(self) -> None:
        """Lock all three policy servos at the pose they are in now."""
        if self._pack_low or self._servo_fault:
            self._event("hold", "hold refused: "
                        + ("pack cut" if self._pack_low else "servo fault"))
            return
        self._hold_steer = float(self.data.qpos[self.ctl._sj])
        if self.torque:
            self.bus.arm(self.bus.policy_ids)
        self._holding = True
        self._event("hold", f"HOLD: drives at 0, steer held at "
                    f"{np.degrees(self._hold_steer):+.1f} deg"
                    + ("" if self.torque else " (no torque: still limp)"))

    def _hold_cut(self) -> None:
        """A pack cut or a servo fault ends a hold, as it ends an engage.
        Needed because `guard.cut_now()` only drops torque from ENGAGED, and
        a hold is never engaged."""
        if self._holding and (self._pack_low or self._servo_fault):
            if self.torque:
                self.bus.torque(False, self.bus.policy_ids)
            self._holding = False
            self._event("cut", "HOLD released: "
                        + ("pack cut" if self._pack_low else "servo fault"))

    def _hold_from(self, rearm_n, stop_n) -> None:
        """--hold's keys, press counts judged as `_rearm_from` judges them:
        engage re-captures and holds, disengage goes limp."""
        peer = self.link.peer
        for n, attr, act in ((rearm_n, "_rearm_seen", "hold"),
                             (stop_n, "_stop_seen", "limp")):
            if n is None:
                continue
            seen = getattr(self, attr)
            if seen is not None and seen[0] == peer and n != seen[1] \
                    and not self._link_lost:
                if act == "hold":
                    self._hold_on()
                else:
                    if self.torque:
                        self.bus.torque(False, self.bus.policy_ids)
                    self._holding = False
                    self._event("standby", "LIMP: drives and steer off "
                                "(--hold; Enter / Y holds again)")
            setattr(self, attr, (peer, n))

    def _rearm_from(self, n) -> None:
        """Consent arrives as a COUNT of presses (`OperatorState.rearms`); a
        change is a press. Not acted on -- only recorded -- when the count
        comes from a station this bike has not heard before (a fresh station
        starts at 0, a restarted one too, and neither is a press), or while
        the link is down: a press the operator made during a dropout was made
        blind, and must not turn into consent the moment packets resume."""
        if n is None:
            return
        peer = self.link.peer
        seen = self._rearm_seen
        if seen is not None and seen[0] == peer and n != seen[1] \
                and not self._link_lost:
            reason = self._hold_reason()
            if reason is not None and self.guard.state != "engaged":
                # SAID, not swallowed: a press the bike will not act on
                # used to vanish without a word.
                self._event("hold", f"engage ignored -- held: {reason}")
            elif self.guard.state != "engaged":
                self._prop_if_resting()
            self.guard.request_rearm()
        self._rearm_seen = (peer, n)

    def _resting_crank(self) -> float | None:
        """The crank [rad, from stow, the sequence's frame] when the bike is
        RESTING ON A BLADE -- out past REST_CRANK_DEG, the bike leaning onto
        it -- else None. The lie-down's test for a bike it need not stop."""
        if self._righting_pos is None:
            return None
        crank = (self._righting_pos - self._r_stow) * self._r_sign
        if abs(crank) < np.radians(REST_CRANK_DEG) \
                or np.sign(self._roll) != np.sign(crank):
            return None
        return float(crank)

    def _prop_if_resting(self) -> None:
        """Enter on a bike standing on a blade the sequence did not put it
        on (set there by hand, or powered up that way): make it `propped`,
        so the hand-off is the prop's -- PROP_ENGAGE_DEG, the blade held as
        a kickstand from where it is -- and not the fall guard's 12 deg on
        a bike with nothing under it."""
        if self.sit is None or self._sit_owns() or self._righting_fault \
                or self._righting_current is None:
            return
        crank = self._resting_crank()
        if crank is None:
            return
        self._sit_gc = int(self._righting_current)
        self.sit.prop_from(self.data.time, crank)
        self._sit_said = "propped"
        self._event("sit", f"resting on the blade (crank "
                           f"{np.degrees(crank):+.0f}, roll "
                           f"{np.degrees(self._roll):+.1f} deg): engaging as "
                           "from a prop")

    def _stop_from(self, n) -> None:
        """The operator's disengage, a press COUNT like `rearm_n` and judged
        the same way (`_rearm_from`): a change from a known station is a
        press. Unlike consent it is acted on even mid-dropout's recovery --
        but a press needs a live link to arrive at all. Policy servos limp,
        righting servo still holding, state STANDBY until the next engage."""
        if n is None:
            return
        peer = self.link.peer
        seen = self._stop_seen
        if seen is not None and seen[0] == peer and n != seen[1]:
            if self.guard.standby() and self.torque:
                self.bus.torque(False, self.bus.policy_ids)
            self._event("standby", "STANDBY: disengaged by the operator -- "
                                   "policy out, righting servo live. Enter / "
                                   "Y to engage")
        self._stop_seen = (peer, n)

    # -- swing-sit ----------------------------------------------------------

    def _sit_owns(self) -> bool:
        """True while the sequence drives the bike, lying and propped
        included."""
        return self.sit is not None and (
            self.sit.active or self.sit.phase in ("lying", "propped"))

    def _state(self) -> str:
        """The state the station and the record see: the guard's, or the
        sequence's ("sit" running, "lying" done and holding)."""
        if not self._sit_owns():
            return self.guard.state
        return (self.sit.phase if self.sit.phase in ("lying", "propped")
                else "sit")

    def _sit_from(self, lie_n, up_n, park_n=None) -> None:
        """Lie-down / get-up / park presses, COUNTS judged as `_rearm_from`
        judges them: a change from the known station, with the link up."""
        peer = self.link.peer
        for n, attr, what in ((lie_n, "_lie_seen", "down"),
                              (up_n, "_up_seen", "up"),
                              (park_n, "_park_seen", "park")):
            if n is None:
                continue
            seen = getattr(self, attr)
            if seen is not None and seen[0] == peer and n != seen[1] \
                    and not self._link_lost:
                self._sit_start(what)
            setattr(self, attr, (peer, n))

    def _sit_start(self, what: str) -> None:
        """Begin lying down or getting up. Refused, and SAID, otherwise. The
        press is the consent: from a cut or standby it arms the policy
        servos (held, never the policy) without Enter.

        DOWN: the policy riding (the full sequence), or the bike already
        RESTING ON A BLADE -- `propped`, or powered up that way, the crank
        out past `REST_CRANK_DEG` and the bike leaning onto it -- which
        goes straight to `lower` and never runs the policy.
        UP: from lying, or from any cut / standby, at any roll: the lift
        ends propped either way. The side is the blade the crank is out
        on when the bike leans onto it, else the way the bike leans.
        PARK: the policy riding, as DOWN; it stops `propped` on the blade."""
        name = {"down": "lie down", "up": "get up", "park": "park"}[what]
        crank = (None if self._righting_pos is None
                 else (self._righting_pos - self._r_stow) * self._r_sign)
        out = crank is not None and abs(crank) >= np.radians(REST_CRANK_DEG)
        resting = out and (self.sit is not None and self.sit.phase == "propped"
                           or (self.guard.state != "engaged"
                               and self._resting_crank() is not None))
        riding = self.guard.state == "engaged" and self.controller == "policy"
        why = None
        if self.sit is None:
            why = ("no righting servo / module stroke" if not self.hold
                   else "--hold")
        elif self.sit.active:
            why = f"busy ({self.sit.phase})"
        elif self._hold_reason() is not None:
            why = f"held: {self._hold_reason()}"
        elif self._righting_fault:
            why = ("the righting servo has a latched hardware error -- "
                   "restart run_bike to reboot it")
        elif self._righting_current is None:
            why = "the righting servo's Goal Current is unknown"
        elif what == "down" and self.sit.phase == "lying":
            why = "already lying"
        elif what == "park" and self.sit.phase == "propped":
            why = "already propped -- Enter hands it to the policy"
        elif what == "park" and self.sit.phase == "lying":
            why = "lying -- get up, which ends propped"
        elif what == "park" and not riding:
            why = "the policy is not driving"
        elif what == "down" and not (riding or resting):
            why = ("the policy is not driving and the bike is not resting "
                   "on a blade" + ("" if crank is None else
                                   f" (crank {np.degrees(crank):+.0f}, roll "
                                   f"{np.degrees(self._roll):+.0f} deg)"))
        elif what == "up" and self.sit.phase == "propped":
            why = "already propped -- Enter hands it to the policy"
        elif what == "up" and self.sit.phase != "lying" \
                and self.guard.state == "engaged":
            why = "the policy is driving (Backspace, then get up)"
        if why is not None:
            self._event("sit", f"{name} ignored -- {why}")
            return
        if self._sit_gc is None:
            self._sit_gc = int(self._righting_current)
        self._arm_righting()
        t = self.data.time
        if self.guard.state != "engaged":
            # From a cut, standby or propped: the drives and steer hold
            # through what follows, so they need torque now.
            if self.torque:
                self.bus.arm(self.bus.policy_ids)
            self.guard.state = "engaged"
            self.guard.consent = False
            self._sit_off = True
        if what == "park":
            self._sit_psi = self._yaw
            self.sit.park(t, self._roll)              # the policy stops it first
        elif what == "down" and not self._sit_off:
            self._sit_psi = self._yaw
            self.sit.lie_down(t, self._roll)          # the policy stops it first
        elif what == "down":
            self.sit.lie_from_prop(t, crank)
        else:
            # The crank's side only when the bike leans onto that blade: a
            # bike lying on its cage has the crank LIE_REST_DEG the other
            # way, and lifting that blade would push at the air.
            onto = out and np.sign(crank) == np.sign(self._roll)
            self.sit.get_up(t, self._roll,
                            side=np.sign(crank) if onto else None)
        self._sit_said = self.sit.phase
        self._event("sit", f"{name}: {self.sit.phase} (Goal Current "
                           f"{self._sit_gc})")

    def _sit_release(self, why: str, cut: bool = False) -> None:
        """Hand everything back: the operator's Goal Current, the crank held
        where it IS, the sequence at `riding`. Policy servos: left as the
        caller found them, except out of `lying` or `propped` -- or with
        `cut` -- where they go limp (a cut, as a fall would leave it).
        Propped has the steer still on (SIT_STEER_DEG), so this is not
        keyed on the guard's `engaged`."""
        if self.sit is None:
            return
        lying = self.sit.phase in ("lying", "propped")
        if self._righting_pos is not None:
            self.bus.set_righting(self._righting_pos)
        if self._sit_gc is not None:
            self._righting_current = self.bus.set_righting_current(self._sit_gc)
        if lying or cut:
            if self.torque:
                self.bus.torque(False, self.bus.policy_ids)
            if self.guard.state == "engaged":
                self.guard.state = "cut"
            self.guard.consent = False
        self.sit.phase = "riding"
        self.sit._down_since = None
        self._sit_gc = self._sit_steer = self._sit_steer_t = None
        self._sit_off = False
        self._sit_engage_pending = False
        self._sit_said = "riding"
        self._event("sit", f"sequence ended: {why}")

    def _sit_step(self):
        """This tick of the sequence, or None when it does not own the bike.
        Runs where the fall guard would; the caller writes the drives."""
        if not self._sit_owns():
            return None
        if self.sit.phase in ("propped", "nudge"):
            # Limp already, so a disengage changes nothing here; a lost link,
            # a pack cut or a fault still ends it. Enter (the guard's consent)
            # is the hand-off, through the re-arm window like any engage.
            # A bike settled OUTSIDE the window but leaning onto its blade
            # -- a prop sagging on servo droop -- is nudged in first
            # (SwingSit.nudge), and the hand-off follows by itself: the
            # Enter was the consent for both.
            if self._hold_reason() is not None:
                self._sit_release(self._hold_reason())
                return None
            ready = ready_for_policy(self._roll, self._roll_rate,
                                     max(PROP_ENGAGE_DEG, self.guard.rearm_deg),
                                     self.guard.rearm_rate)
            if self.sit.phase == "nudge":
                self.guard.consent = False      # a press mid-nudge adds nothing
            elif self._sit_engage_pending:
                self._sit_engage_pending = False
                self.guard.consent = False
                if self.sit.nudged and ready:
                    self._sit_engage()
                else:
                    self._event("sit", f"nudge did not get it in -- roll "
                                f"{np.degrees(self._roll):+.0f} deg; still "
                                "propped, Enter to try again")
            elif self.guard.consent:
                self.guard.consent = False
                if ready:
                    self._sit_engage()
                elif (NUDGE and abs(self._roll_rate) < np.radians(NUDGE_STILL_DPS)
                      and self.sit.onto_blade(self._roll)):
                    crank = (0.0 if self._righting_pos is None else
                             (self._righting_pos - self._r_stow) * self._r_sign)
                    self.sit.nudge(self.data.time, crank)
                    self._sit_engage_pending = True
                    self._event("sit", f"roll {np.degrees(self._roll):+.0f} "
                                "deg, outside the re-arm window: nudging the "
                                "blade out, then engaging")
                else:
                    self._event("sit", f"engage ignored -- roll "
                                f"{np.degrees(self._roll):+.1f} deg, not "
                                f"settled inside {PROP_ENGAGE_DEG:.0f} deg")
        elif self.guard.state != "engaged":
            # Disengaged, link lost, pack cut, servo fault: all of them took
            # the policy servos off. The crank holds where it is.
            self._sit_release(self._hold_reason() or "disengaged")
            return None
        crank = (0.0 if self._righting_pos is None
                 else (self._righting_pos - self._r_stow) * self._r_sign)
        speed = float(np.hypot(self.data.qvel[0], self.data.qvel[1]))
        cmd = self.sit.step(self.data.time, self._roll, self._roll_rate,
                            crank, speed)
        if self.sit.phase != self._sit_said:
            self._sit_said = self.sit.phase
            self._event("sit", self.sit.phase + (
                " -- lie-down ABORTED: it went over the other way"
                if self.sit.phase == "retract" and self.sit.aborted else
                f" -- get-up {self.sit.failed.upper()} at crank "
                f"{np.degrees(crank):+.0f} deg: lowering back, press again"
                if self.sit.phase == "lower" and self.sit.failed else
                f" -- get-up stalled at crank {np.degrees(crank):+.0f} deg: "
                "pumping with the rock" if self.sit.phase == "pump" else ""))
        self.bus.set_righting(self._r_stow + self._r_sign * cmd.crank)
        # Quantised to 5 %: Goal Current is its own bus transaction, and the
        # limp ramp would otherwise write one every tick for a second.
        want = int(round(self._sit_gc * round(cmd.current * 20) / 20))
        if want != self._righting_current:
            self._righting_current = self.bus.set_righting_current(want)
        if cmd.policy and self._sit_off:
            # Back from holding: a clean slate, as any engage (its memory
            # and estimator ran on a bike lying on its side).
            self._engage(fresh_read=False)
            self._sit_psi = self._yaw
        self._sit_off = not cmd.policy
        if cmd.limp and self.guard.state == "engaged":
            # Propped: the bike stands on wheels + blade; the drives have
            # nothing to hold. The steer stays on, straight (SIT_STEER_DEG).
            if self.torque:
                self.bus.torque(False, (self.bus.id_a, self.bus.id_b))
            self.guard.state = "standby"
            self.guard.consent = False      # an Enter from before is not one for now
            self._event("sit", "PROPPED: drives limp, steer holding, wings "
                        + ("let go" if self.sit.prop_current == 0.0
                           else "holding")
                        + " -- Enter hands it to the policy")
        if cmd.policy:
            self._sit_steer = self._sit_steer_t = None
        else:
            self._sit_steer_step()
        if not self._sit_owns():                  # just finished: riding
            if self._sit_gc is not None:
                self._righting_current = self.bus.set_righting_current(
                    self._sit_gc)
            self._sit_gc = None
            return None
        return cmd

    def _sit_engage(self) -> None:
        """Propped -> the policy: drives armed, the sequence into `balance`."""
        self._arm_righting()                 # unpowered if set by hand
        if self.torque:
            self.bus.arm(self.bus.policy_ids)
        self.guard.state = "engaged"
        self._sit_off = True
        crank = (None if self._righting_pos is None else
                 (self._righting_pos - self._r_stow) * self._r_sign)
        self.sit.engage(self.data.time, crank)

    def _sit_steer_step(self) -> None:
        """Move the held steer one tick toward straight (SIT_STEER_DEG).

        It starts where the steer IS, not at the last ctrl: in a fresh
        process ctrl is 0 whatever the wheel reads, and holding it swung the
        wheel ~200 deg at the start of a get-up (run 261026-163421). The
        target is the nearer of the two straight poses, so the most it ever
        turns is 90 deg."""
        t = self.data.time
        if self._sit_steer is None:
            self._sit_steer = float(self.data.qpos[self.ctl._sj])
            self._sit_steer_t = t
        dt, self._sit_steer_t = t - self._sit_steer_t, t
        off = -self.sit.side * np.radians(SIT_STEER_DEG)
        to = off + np.pi * round((self._sit_steer - off) / np.pi)
        step = np.radians(SIT_STEER_DPS) * max(dt, 0.0)
        self._sit_steer += float(np.clip(to - self._sit_steer, -step, step))

    def _warning(self) -> str | None:
        """A standing warning, or None: the pack below warn but not yet cut
        (once cut, `hold` says it). The level behind the one-off PACK LOW
        message, which the station shows for 5 s and then drops."""
        if getattr(self, "hold", False):
            return ("HOLD MODE: policy disabled; servos "
                    + ("locked" if self._holding else "limp"))
        if self.pack.warned and not self._pack_low:
            return (f"pack low: {self.pack.v:.1f} V < {self.pack.warn_v:.1f} V"
                    f" ({self.pack.cut_v:.1f} V min)")
        return None

    def _hold_reason(self) -> str | None:
        """Why nothing will re-arm, in words, or None. The level behind
        `guard.hold`, sent every tick as telemetry `hold`. Worst first."""
        if self._pack_low:
            # A full power cycle, not just run_bike: a swap takes the Pi down
            # with the pack.
            return (f"pack at {self.pack.v:.1f} V ({self.pack.cut_v:.1f} V "
                    "min) POWER OFF AND SWAP BATTERY")
        if self._servo_fault:
            bad = [r for r in ("drive_a", "drive_b", "steer")
                   if self._errs.get(r)]
            return (f"{', '.join(bad) or 'a policy servo'} faulted -- restart "
                    "run_bike to reboot it")
        if self._link_lost:
            return "link lost"
        return None

    def preflight_ahrs(self, seconds: float = 0.5, strict: bool = True) -> list[str]:
        """Sanity-check the AHRS before engaging. Bike must be STATIONARY.

        Deliberately POSE-INDEPENDENT — it does not assume the bike is upright,
        because at startup it is usually on a stand or lying on its wings. So it
        checks only what holds in any orientation at rest:

          * frames are arriving at all, and are fresh;
          * |accel| == g, which catches a dead, mis-scaled or mis-parsed
            accelerometer regardless of which way is down;
          * |gyro| ~ 0, which catches a runaway bias — the failure that would
            otherwise show up as the bike calmly driving itself over;
          * the sensor's OWN QoS grade, which is the only one of the four that
            catches a unit whose numbers are all plausible and all wrong.
            Ep_Combo carries it, so it costs nothing to read. The WORST grade
            seen in the window is the one judged, not the mean: a sensor that
            dips into fault for 50 ms has faulted.

        What it CANNOT check is the mounting calibration, because that needs a
        known reference pose. See docs/plans/untethered-setup.md for the
        wings-down self-check that would close that gap once the wing geometry
        is built and its expected attitude is known.
        """
        problems: list = []
        # `design` passes: since the v2 bike (10-19) the TM151 is located on
        # its own four holes on the drive's tabs, so the CAD fixes the angle,
        # and nothing on the bike levels more finely than the AHRS itself.
        # What still blocks is a mount nobody located: `GUESS`, or `absent`
        # (no config/ahrs_mount.yaml, identity assumed).
        if self.mount_source not in MOUNT_LOCATED:
            problems.append((MOUNT_UNCALIBRATED,
                f"AHRS mount is '{self.mount_source}', not one of "
                f"{'/'.join(MOUNT_LOCATED)} — the rotation into the chassis "
                "frame is unlocated, and any error in it is a permanent roll "
                "bias."))

        t_end = time.monotonic() + seconds
        accels, gyros, qoses, last_err = [], [], [], None
        while time.monotonic() < t_end:
            try:
                s = self.ahrs.latest()
            except Exception as e:                      # stale, mid-run
                # Do NOT return on the first failure: a single late sample is
                # not a dead sensor, and bailing here is what made the startup
                # race invisible. Keep sampling for the whole window and judge
                # on what was collected.
                last_err = e
                time.sleep(0.01)
                continue
            accels.append(np.linalg.norm(s.accel))
            gyros.append(np.linalg.norm(s.gyro))
            qoses.append(s.qos)
            time.sleep(0.01)

        if not accels:
            problems.append((None, f"AHRS produced no samples in {seconds:.1f} s"
                             + (f": {last_err}" if last_err else "")))
            problems.append((None, self._ahrs_diagnosis()))
            return _report_preflight(problems, strict, self._preflight_allow)

        a, g = float(np.mean(accels)), float(np.max(gyros))
        qos = min(qoses)
        if qos == QOS_UNKNOWN:
            problems.append((None,
                "AHRS frames carry no QoS — the sample came from a path that "
                "does not decode Ep_Combo's sysState. Check the parser, not "
                "the sensor."))
        elif qos == QOS_LIMITED:
            # A WARNING, not a fault (user, 2026-10-20): it is the TM151
            # still settling after power-up. Measured on the bench Pi after a
            # reboot: first sample at 14.4 s of uptime already at 2, 3 at
            # 16.8 s, 4 at 33.9 s -- so `bench` started the moment the Pi
            # answers lands in it, and failing there only made the operator
            # retype the command. Lower grades (booting, fault) still block.
            problems.append((AHRS_QOS_LIMITED,
                f"AHRS reports QoS {qos} ({QOS_NAMES[QOS_LIMITED]}), below "
                f"{QOS_MIN_SERVICE} ({QOS_NAMES[QOS_MIN_SERVICE]}): normally "
                "the sensor settling after power-up (on the bench it reached 3 "
                "~2.5 s after its first sample). Keep the bike still; "
                "telemetry's qos shows it rise."))
        elif qos < QOS_MIN_SERVICE:
            problems.append((None,
                f"AHRS reports QoS {qos} ({QOS_NAMES.get(qos, '?')}); the bike "
                f"needs at least {QOS_MIN_SERVICE} "
                f"({QOS_NAMES[QOS_MIN_SERVICE]})."))
        if abs(a - GRAVITY) > PREFLIGHT_ACCEL_TOL * GRAVITY:
            problems.append((None,
                f"|accel| is {a:.2f} m/s^2, expected ~{GRAVITY:.2f} "
                f"(+-{PREFLIGHT_ACCEL_TOL:.0%}) — check units, parsing, or the sensor"))
        if g > PREFLIGHT_GYRO_MAX:
            problems.append((None,
                f"|gyro| peaks at {np.degrees(g):.1f} deg/s while supposedly at rest "
                f"(limit {np.degrees(PREFLIGHT_GYRO_MAX):.1f}) — bias, vibration, "
                "or the bike is moving"))
        print(f"preflight: |accel| {a:.2f} m/s^2, |gyro| max {np.degrees(g):.2f} deg/s, "
              f"qos {qos} ({QOS_NAMES.get(qos, '?')}), mount '{self.mount_source}'")
        return _report_preflight(problems, strict,
                                 (*self._preflight_allow, AHRS_QOS_LIMITED))

    def _ahrs_diagnosis(self) -> str:
        """Why is there no attitude? The three cases have different fixes.

        `frames == 0` alone is ambiguous, and the ambiguity is not academic:
        measured on the bench 2026-09-16, a TM151 fresh out of the box streams
        rpy(35), raw_gyro_acc_mag(41) and status(22) at 50 Hz and no Ep_Combo
        at all. `parse_frame` skips those silently -- they have good CRCs, they
        are simply not the message this reads -- so the reader reports
        frames=0, errors=0 and looks precisely like a sensor that is not
        plugged in. It cost a round of wiring checks before anyone counted the
        bytes.
        """
        r = self.ahrs
        if r.bytes_in == 0:
            return (f"nothing at all on {r.port}: 0 bytes. Check the cable and "
                    f"the port -- over USB the TM151 is a CDC device "
                    f"(/dev/ttyACM*), and its baud is ignored")
        if r.frames == 0:
            return (f"{r.bytes_in} bytes arrived but NO Ep_Combo frames "
                    f"({r.errors} parse errors). The sensor is alive and "
                    f"streaming the wrong message. Enable Ep_Combo at 200 Hz "
                    f"in ImuAssistant; `python analysis/sensing/tm151_serial.py` will "
                    f"name what it is sending instead")
        return (f"{r.frames} Ep_Combo frames decoded but the latest is stale -- "
                f"the reader is falling behind, not the sensor")

    def _watch_pack(self) -> None:
        """Low pack -> warn once; flat pack -> cut and hold, like a faulted
        servo. See `PackMonitor` for why the max and why filtered.

        IT USED TO END THE PROCESS (`_check_failsafes`, until 2026-09-18),
        on a raw 1 Hz sample of drive A alone. Now the loop, the telemetry and
        the record carry on, so the station SEES why the bike stopped and the
        record keeps the discharge curve. Nothing re-arms: the fix is a new
        pack and a restart. The righting servo keeps its last goal, as on
        every other cut.
        """
        readings = [h.get("Present Input Voltage")
                    for h in self._health.values()]
        event = self.pack.update(self._dt_meas, readings)
        if event == "warn":
            self._event("pack", f"PACK LOW: {self.pack.v:.1f} V < "
                                f"{self.pack.warn_v:.1f} V "
                                f"({self.pack.cut_v:.1f} V min)")
        elif event == "cut":
            self._pack_low = True
            if self.guard.cut_now() and self.torque:
                self.bus.torque(False, self.bus.policy_ids)
            self._event("cut", f"CUT: pack at {self.pack.v:.1f} V "
                               f"({self.pack.cut_v:.1f} V min) POWER OFF AND "
                               "SWAP BATTERY")
        self.guard.hold = self._link_lost or self._servo_fault or self._pack_low

    def _event(self, kind: str, text: str) -> None:
        """Say it here AND to the station. The one place a message is made."""
        _say(text)
        self.events.add(kind, text)

    def _watch_errors(self) -> None:
        """A servo's Hardware Error Status CHANGING is an event the bike
        announces; the level itself is in `servos` for display. Decided here
        so the station does not keep its own copy of the rule.

        A LATCHED ERROR ON A POLICY SERVO IS A CUT, and holds. The servo has
        already turned its own torque off -- the bike cannot balance on a dead
        drive or a limp steer -- and until 2026-09-18 the policy stayed
        engaged and kept commanding it: drive A tripped overload at 88.6 s of
        a bench run and was driven, dead, for 5 s. The latch only clears on a
        reboot, which restarting run_bike now does (`BikeBus.open`). The
        righting servo's fault is not a cut: it ends a lie-down / get-up
        (policy servos cut unless the policy is riding) and refuses the
        next one.
        """
        from .dynamixel import describe_hardware_error
        for role, h in self._health.items():
            err = int(h.get("Hardware Error Status", 0) or 0)
            if err != self._errs.get(role, 0):
                self._event("error", f"{role} HARDWARE ERROR: "
                                     f"{describe_hardware_error(err)}"
                            if err else f"{role} hardware error cleared")
            self._errs[role] = err
        # The righting servo's own fault ends the sequence (it can neither
        # lift nor hold) and refuses the next one. It is NOT a cut while the
        # policy rides: run 261026-173521 tripped overload against a wall,
        # and the sequence went on commanding a dead crank for a minute.
        rfault = bool(self._errs.get("righting"))
        if rfault and not self._righting_fault \
                and getattr(self, "sit", None) is not None and self._sit_owns():
            self._sit_release("the righting servo faulted -- restart run_bike "
                              "to reboot it", cut=self._sit_off)
        self._righting_fault = rfault
        fault = any(self._errs.get(r) for r in ("drive_a", "drive_b", "steer"))
        if fault and not self._servo_fault:
            if self.guard.cut_now() and self.torque:
                self.bus.torque(False, self.bus.policy_ids)
            self._event("cut", "CUT: a policy servo faulted -- restart "
                               "run_bike to reboot it (a latched error "
                               "survives until then)")
        self._servo_fault = fault
        self.guard.hold = self._link_lost or self._servo_fault or self._pack_low

    def _on_guard_event(self, event: str) -> None:
        if event == "cut":
            # Torque off the three the policy drives, and ONLY those: the
            # righting servo has to stay alive here, since lying down is the
            # one moment it has a job.
            self.bus.torque(False, self.bus.policy_ids)
            self._event("cut", f"CUT: roll {np.degrees(self._roll):.0f} deg -- "
                               "policy out, righting servo live. Enter / Y "
                               "once upright and still")
        elif event == "rearm":
            self._engage(fresh_read=False)   # this tick already sensed
            self._stow_if_unset()
            # ONLY WITH --torque. This armed unconditionally until 2026-09-18,
            # so `r` after a cut ENERGISED a --no-torque session -- the one
            # mode whose whole promise is that nothing can move.
            if self.torque:
                self.bus.arm(self.bus.policy_ids)
            self._event("rearm", "ENGAGED: policy driving, command zeroed"
                        + ("" if self.torque else " (--no-torque: still limp)"))

    def _watch_link(self) -> None:
        """Dead link -> cut and hold; link back -> say so and wait for `r`.

        Only the three policy servos lose torque, as in a fall. The righting
        servo keeps holding its last goal: it is current-limited, it is what
        it does on every normal tick, and dropping it would let a deployed
        wing fall under load with nobody there to see why.
        """
        if not self.require_link:
            return
        dead = self.link.age() > CMD_DEAD_S
        if dead and not self._link_lost:
            self._link_lost = True
            self.guard.hold = True
            if self.guard.cut_now() and self.torque:
                self.bus.torque(False, self.bus.policy_ids)
            self._event("link", f"LINK LOST ({self.link.age():.1f} s): policy "
                                "out, torque off -- waiting for a station")
        elif not dead and self._link_lost:
            self._link_lost = False
            self.guard.hold = self._servo_fault or self._pack_low
            reason = self._hold_reason()
            self._event("link", f"LINK BACK from {self.link.peer[0]}: " + (
                f"still HELD -- {reason}" if reason
                else "Enter / Y once the bike is upright and still"))

    def wait_for_link(self, timeout: float = LINK_WAIT_S) -> None:
        """Block until the ground station says hello. Before any torque.

        `run_bike` USED TO DIE ON TICK 1 with no ground station: `cmd_t` starts
        at 0.0, `age()` reports inf while it is falsy, and the command-dead
        failsafe is checked on the first pass through the loop -- so with
        nothing sending to port 9910 the process engaged the policy, tripped
        `command link dead (inf s)` and shut down, every time.

        Waiting is the right shape rather than starting deaf: the watchdog
        should mean "the operator went away", which is only meaningful once the
        operator has been there. `--no-link` is the bench escape, and it turns
        the link failsafes off rather than pretending a command arrived.
        """
        print(("waiting for a command" if not timeout else
               f"waiting up to {timeout:.0f} s for a command")
              + f" on {self.link.addr[0]}:{self.link.addr[1]} ...", flush=True)
        t_end = time.monotonic() + timeout if timeout else float("inf")
        while time.monotonic() < t_end:
            if self.link.age() < CMD_DEAD_S:
                print(f"ground station at {self.link.peer[0]}")
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"no command in {timeout:.0f} s. Start the ground station, or run "
            f"with --no-link for a bench session with no operator.")

    def _engage(self, fresh_read: bool = True) -> None:
        """Hand the actuators to the policy, from a clean slate.

        Called at startup AND on every re-arm after a fall, and it has to be
        the full reset both times. The controller carries state across ticks --
        `prev_action` in the policy observation, the 50 Hz zero-order-hold
        schedule, the command anchor -- and the estimator carries its rate
        filters and an integrated position. Resuming a policy with the
        prev_action it was using as it went over, and a velocity filter full of
        samples from the fall, is feeding it an observation from a bike that no
        longer exists.

        `fresh_read=False` is the re-arm, which runs inside a tick that has
        already sensed: the new estimator takes THAT tick's sample instead of
        a second bus read. The second read cost ~2-3 ms of a tick already
        over budget, advanced `data.time` a few ms twice in one tick, and fed
        the servos' rate filters a sample a few ms after the last.
        """
        self.est = VelocityEstimator(self.params)
        if fresh_read:
            self._sense()
        else:
            self._estimate()
        self.ctl.reset(self.model, self.data)
        if self.controller == "lqr":
            # reset() already dropped a fresh line anchor where the bike
            # stands; this also zeroes the speed and re-adopts the heading.
            self.ctl.follow_reset(self.data)
        else:
            self.ctl.engage_general(self.data, name=self.gen_name, reuse=True)
        # The operator's last command predates the fall. Re-anchor to standing
        # still and let them ask again.
        self.ctl.set_command(v_cmd_world=[0.0, 0.0])

    def run(self, preflight: bool = True) -> None:
        """The session, with ONE try around all of it, so every way out --
        ctrl-C, SIGTERM, SIGHUP, a failsafe, an exception in preflight --
        runs `shutdown()`, which is what takes torque off.

        SIGTERM AND SIGHUP USED TO SKIP IT. Python's default for both is to
        die on the spot, so `kill`, `timeout`, or a dropped `ssh -t` session
        left the servos energised holding their last goal -- a drive in
        velocity mode keeps spinning -- because the servos' own Bus Watchdog
        is not armed by this runner.

        AND NOT MID-TRANSACTION. Raising from the handler wherever the main
        thread happens to be lands, most of the time, inside a bus read or
        write -- which leaves the SDK's port marked busy, so the torque-off in
        `shutdown` was refused (measured: rc -1000 on a SIGTERM'd run). So
        inside the control loop a signal only REQUESTS the stop, and the loop
        honours it at the top of the next tick, between transactions -- at
        most 10 ms later. Outside the loop (preflight, waiting for a station)
        there is no transaction to interrupt and it raises at once. ctrl-C is
        routed the same way, since it had the same flaw.
        """
        for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
            signal.signal(sig, _on_stop_signal)
        _STOP.update(sig=None, defer=False)
        try:
            self._run(preflight)
        except KeyboardInterrupt:
            self._stopped_by = None          # ctrl-C is the normal way out
            _say("\nctrl-C")
        except Stopped as e:
            self._stopped_by = str(e)
            _say(f"\n{e}")
        except BaseException as e:
            self._stopped_by = f"{type(e).__name__}: {e}"
            raise
        finally:
            for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
                signal.signal(sig, signal.SIG_IGN)   # one shutdown, not two
            self.shutdown()

    def _run(self, preflight: bool) -> None:  # noqa: C901 -- the session
        # THE PORT FIRST: it is the single-instance lock. A second run_bike
        # used to open the bus before discovering 9910 was taken -- and
        # opening the bus turns torque OFF and rewrites the indirect map on
        # every servo, i.e. dropped the running instance's bike mid-session,
        # then crashed. Binding first means the second one touches nothing.
        try:
            self.link.start()
        except OSError as e:
            if e.errno == errno.EADDRINUSE:
                raise SystemExit(
                    f"port {self.link.addr[1]} is taken -- another run_bike is "
                    "already running (pgrep -af run_bike). Not touching the "
                    "bus.") from None
            raise
        self.bus.open()                      # torque still OFF; see BikeBus.open
        from .dynamixel import describe_hardware_error
        for i, bits in (self.bus.rebooted or {}).items():
            self._event("error", f"id {i} had a latched hardware error "
                                 f"({describe_hardware_error(bits)}) -- "
                                 "rebooted, clear now")
        # Per role, the unit the effort slot decodes in -- A or fraction of
        # max torque, by model (hw/bike_bus.HEALTH_BLOCK).
        self._effort_unit = {role: self.bus._map.register(i, "effort").unit_name
                             for i, role in self.bus.roles.items()}
        if self.record:
            self._start_record()
        # What open() just wrote, so the operator's first [ or ] is a CHANGE
        # rather than a redundant write of the value already on the servo.
        self._righting_current = self.bus.righting_current_applied
        self.ahrs.start()
        _try_realtime()

        # Before any torque: the bike is stationary here and never again.
        # True as written now -- `BikeBus.open` no longer energises the servos.
        # Wait for the sensor before judging it. Without this, preflight ran
        # microseconds after the reader thread was spawned and reported a dead
        # AHRS on a perfectly good port -- and poll mode could never win that
        # race, since its first sample costs a request round trip.
        try:
            waited = self.ahrs.wait_ready()
            print(f"AHRS first sample after {waited*1e3:.0f} ms "
                  f"({'polled' if self.ahrs.poll else 'pushed'})")
        except RuntimeError as e:
            print(f"PREFLIGHT: {e}")
            print(f"PREFLIGHT: {self._ahrs_diagnosis()}")
            if preflight:
                raise
        self.preflight_ahrs(strict=preflight)
        if self.require_link:
            self.wait_for_link(self.link_wait)

        voltage = self.bus.pack_voltage()
        # Seed the filter; the first tick decides. Set, not `update`d, so a
        # pack that is already flat still produces its "cut" event there.
        self.pack.v = voltage
        # WHICH policy: `self.gen_name`, resolved in __init__ because the
        # firmware gains follow it and are written before torque. It used to be
        # engage_general with no name at all, which silently took the hardcoded
        # "general_rl" default -- so the BIKE always drove general_rl no matter
        # what control.general_move said, while teleop honoured it. The two
        # disagreed, and the one that mattered was the one nobody could see.
        # STANDBY AT CONNECT (2026-10-20). A station connecting used to BE
        # the engage: the first packet put the policy on the bike and, with
        # torque, energised the drives and steer, with nothing pressed. Now
        # the policy waits for the operator's engage (Enter / Y), through the
        # same gate as a re-arm (upright, still, consent). --no-link has no
        # operator to ask, so it engages at once as before.
        standby = self.require_link or self.hold
        print(f"pack {voltage:.1f} V — general policy {self.gen_name} at "
              f"{1/self.dt:.0f} Hz, "
              + ("HOLD MODE: policy never engages" if self.hold
                 else "STANDBY: Enter / Y on the station engages" if standby
                 else "engaging (--no-link)"))
        print(f"  {self.gain_note}")
        print(f"  pack: warn below {self.pack.warn_v:.1f} V, cut below "
              f"{self.pack.cut_v:.1f} V (max servo reading, "
              f"{self.pack.tau_s:g} s low-pass)")
        print(f"  cut at {np.degrees(self.guard.cut):.0f} deg, re-arm below "
              f"{np.degrees(self.guard.rearm):.0f} deg"
              + ("" if self.guard.auto else " on request"))

        self.data.time = 0.0
        self._engage()
        if standby:
            self.guard.standby()
        if self.torque and voltage < self.pack.cut_v:
            print(f"pack {voltage:.1f} V is below the cut -- NOT arming")
        elif self.hold:
            self._arm_righting()
            self._hold_on()
        elif self.torque and standby:
            # NOTHING yet, the righting servo included: in standby it is
            # free to move by hand (to find a prop) until the station
            # drives it -- 9/4, the triggers, a sit press -- or an engage
            # stows it (`_arm_righting`). The policy's three wait for the
            # engage.
            if self.bus.id_right:
                print("  righting servo UNPOWERED until 9/4, the triggers, "
                      "a lie-down / get-up / park, or an engage")
        elif self.torque:
            self.bus.arm()                   # <- the first torque of the run
            self._right_armed = True
            self._stow_if_unset()            # --no-link: this IS the engage
        else:
            # EVERY OTHER PART OF THE TICK STILL RUNS. Goal writes are simply
            # discarded by a servo with torque off (measured, see
            # hw/control_tables/README.md), so the SyncWrite still goes on the
            # wire and the loop timing is the real thing -- while nothing can
            # move. This is the honest way to measure a control loop on a
            # bench where the steer is bare and the drives are geared to a
            # wheel.
            print("--no-torque: full loop, nothing energised")

        # PAY NUMPY'S FIRST-CALL COST BEFORE THE CLOCK STARTS. `np.percentile`
        # drags in sorting and function_base machinery on first use, which on a
        # Pi 3B+ measured as a 41 ms stall -- landing on tick 50, the first time
        # the 2 Hz jitter statistic ran, and taking five further ticks to catch
        # up. The loop was blameless; the instrumentation was late to its own
        # measurement, twice now (see the telemetry note above).
        np.percentile(np.zeros(8), 99)

        _STOP["defer"] = True                # from here, stop between ticks
        t0 = time.monotonic()
        next_tick = t0
        k = 0
        while True:
            if _STOP["sig"] is not None:
                raise _stopped(_STOP["sig"])
            next_tick += self.dt
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            slip = time.monotonic() - next_tick
            self.jitter.append(slip)
            if slip > 0.01 and len(self._late) < 40:
                self._late.append((k, round(slip * 1e3, 1)))

            self._sense()
            self._apply_command()

            k += 1
            # EVERY TICK, filtered (PackMonitor). It was a raw 1 Hz sample
            # precisely because a raw per-tick one would trip on every sag;
            # the low-pass is what makes per-tick safe.
            self._watch_pack()
            voltage = self.pack.v
            if k % 50 == 0 and self.jitter:   # 2 Hz; a percentile is not free
                self._jitter_ms = round(
                    float(np.percentile(np.fromiter(self.jitter, float), 99))
                    * 1e3, 2)
            if self.seconds and self.data.time >= self.seconds:
                print(f"reached --seconds {self.seconds:g}")
                break
            self._watch_link()
            self._watch_errors()
            self._hold_cut()

            # The fall guard runs on the SENSED state and before the
            # controller, so a tick that starts on the bike's side never
            # reaches the policy at all. Not while the swing-sit sequence
            # owns the bike: lying down is a roll past the cut.
            sit = self._sit_step()
            if sit is None:
                event = self.guard.update(self.data.time, self._roll,
                                          self._roll_rate)
                self._on_guard_event(event)

            write_s = float("nan")
            self.bus.last_goal = {}
            if sit is not None and not sit.policy:
                # The policy is off: drives at zero speed plus the sequence's
                # differential (unpowered, propped), the steer held and eased
                # straight (_sit_steer_step).
                c, a = self.data.ctrl, self.aid
                c[a["drive_a"]], c[a["drive_b"]] = mix(0.0, sit.diff)
                c[a["steer"]] = self._sit_steer
                t_w = time.perf_counter()
                self.bus.write_commands(c, a)
                write_s = time.perf_counter() - t_w
            elif self.guard.state == "engaged":
                self.ctl.tick(self.model, self.data)
                t_w = time.perf_counter()
                self.bus.write_commands(self.data.ctrl, self.aid)
                write_s = time.perf_counter() - t_w
            elif self._holding:
                c, a = self.data.ctrl, self.aid
                c[a["drive_a"]] = c[a["drive_b"]] = 0.0
                c[a["steer"]] = self._hold_steer
                t_w = time.perf_counter()
                self.bus.write_commands(c, a)
                write_s = time.perf_counter() - t_w
            else:
                self.bus.write_righting()

            # EVERY TICK since 2026-09-16, was every other one. The old
            # comment here said building a dict the link would not send was
            # waste, which was true while `CommandLink` transmitted at
            # 50 Hz -- but that halving is also 0-20 ms of extra staleness
            # on top of the transmit period, and the mirror shows it as
            # lag. Both now run at the control rate.
            #
            # THE COST IS MEASURED, not assumed: `telemetry.build` is
            # 48.8 us on a Pi 3B+ (155 us with a full pose and four servos
            # of registers, which is Phase 2's packet), so every tick is
            # ~0.5% of a 10 ms budget. The packet is 438 B, so 100 Hz is
            # 43.8 kB/s against a link measured to carry 1.5 MB/s with
            # zero loss. Neither end of that is close.
            if k % 100 == 0 and self.link.peer is None:
                # No ground station to send telemetry to, so say it here.
                print(f"  t {self.data.time:5.1f}  roll {np.degrees(self._roll):+6.1f} "
                      f"deg  drive {self.data.ctrl[self.aid['drive_a']]:+6.2f}"
                      f"/{self.data.ctrl[self.aid['drive_b']]:+6.2f} rad/s  "
                      f"steer {np.degrees(self.data.ctrl[self.aid['steer']]):+6.1f} deg  "
                      f"v {self.data.qvel[0]:+.2f},{self.data.qvel[1]:+.2f}  "
                      f"{self._jitter_ms:.2f} ms")
            self.link.telemetry = telemetry.build(
                t=self.data.time,
                state=self._state(),
                quat=self.data.qpos[3:7],
                gyro=self.data.qvel[3:6],
                v_world=self.data.qvel[:2],
                pos=self.data.qpos[:2],
                steer=self.data.qpos[self.ctl._sj],
                w_shaft=self._w_shaft,
                shaft=self._shaft,
                # The bike's OWN heading, so the station's `/` can re-aim
                # the command at where the bike actually points -- which is
                # what teleop's `/` does, and what "stop" means once the
                # commanded heading has drifted from the real one.
                # THE MEASURED YAW, NOT THE CONTROLLER'S. `ctl._psi` is only
                # updated while the policy runs, so it FROZE through every cut
                # -- 101 deg stale in one bench session -- and the station
                # re-aims and clamps against this number.
                psi=self._yaw,
                controller=self.controller,
                cmd_v_world=self.ctl._gen_v_cmd,
                cmd_psi=self.ctl._gen_psi_cmd,
                roll=self._roll,
                roll_rate=self._roll_rate,
                volts=voltage,
                # <1 means the front wheel is near perpendicular and v_lat
                # is coasting on the accelerometer -- worth seeing live.
                vlat_conf=self.est.confidence,
                qos=self._qos,
                jitter_ms=self._jitter_ms,
                dt_ms=self._dt_meas * 1e3,
                cuts=self.guard.cuts,
                righting=self.bus._righting_goal,
                # The GOAL above, the READING here. The station renders the
                # reading, so a wing held back by its current limit is
                # drawn held back instead of drawn as commanded.
                righting_pos=self._righting_pos,
                # So `[` and `]` step from the value that is ON the servo
                # (control.onboard.righting_current at startup) rather than
                # from zero. The station cannot know it otherwise.
                # The OPERATOR's value while the swing-sit scales it (limp
                # reads 0 on the servo, and a station that adopted that
                # would step [ / ] from zero). The record keeps the live one.
                righting_current=(self._sit_gc if self._sit_gc is not None
                                  else self._righting_current),
                servos=telemetry.servo_health(self._health,
                                              self._effort_unit),
                run=self.run_id,
                log=self.events.recent(),
                hold=self._hold_reason(),
                warn=self._warning(),
            )
            if self.rec is not None:
                self._record_tick(next_tick + slip - t0, slip, write_s,
                                  voltage)

    # -- the onboard record ------------------------------------------------

    def _start_record(self) -> None:
        from ..params import design_digest, plant_digest
        d = new_capture_dir(RECORD_ROOT, "run", self.tag)
        self.run_id = d.name
        self.rec = RunRecorder(d, self.bus._map, BIKE_COLS, rate_hz=1 / self.dt,
                               meta={
            "argv": sys.argv,
            "bundle": str(self.bundle_path),
            "policy": self.gen_name,
            "plant_digest": plant_digest(self.params),
            "design_digest": design_digest(self.params),
            "roles": {str(i): r for i, r in self.bus.roles.items()},
            "models": {str(i): m for i, m in self.bus.models.items()},
            "gains_applied": self.bus.gains_applied,
            "steer_zero_rad": self.bus.steer_zero,
            "torque": self.torque,
            "hold": self.hold,
            "states": list(STATES),
            # The whole config the run was taken under, so a param that moved
            # afterwards cannot quietly reinterpret it.
            "params": self.params,
        })
        print(f"recording to {d}")

    def _record_tick(self, t_host: float, slip: float, write_s: float,
                     voltage: float) -> None:
        c, a = self.data.ctrl, self.aid
        st = self._state()
        # A ridden-out bad read is recorded as one (`ok` False, raw blank),
        # not as the stale registers it held.
        raw = dict(self.bus.last_raw) if self._read_ok else None
        self.rec.add(t_host, raw, cmd=self.bus.last_goal,
                     read_s=self.bus.last_read_s, write_s=write_s, bike={
            "t": self.data.time,
            "state": STATES.index(st) if st in STATES else -1,
            "slip_ms": slip * 1e3, "dt_ms": self._dt_meas * 1e3,
            "quat": self.data.qpos[3:7], "gyro": self.data.qvel[3:6],
            "accel": self._accel,
            "qos": np.nan if self._qos is None else self._qos,
            "roll": self._roll, "roll_rate": self._roll_rate,
            "v_world": self.data.qvel[:2], "pos": self.data.qpos[:2],
            "psi": self.ctl._psi, "vlat_conf": self.est.confidence,
            "steer": self.data.qpos[self.ctl._sj],
            "ctrl": (c[a["drive_a"]], c[a["drive_b"]], c[a["steer"]]),
            "cmd_v_world": self.ctl._gen_v_cmd, "cmd_psi": self.ctl._gen_psi_cmd,
            "link_age": min(self.link.age(), 1e6),
            "righting_goal": (np.nan if self.bus._righting_goal is None
                              else self.bus._righting_goal),
            "righting_current": (np.nan if self._righting_current is None
                                 else self._righting_current),
            "volts": voltage,
            "controller": telemetry.CONTROLLERS.index(self.controller),
        })

    def shutdown(self) -> None:
        """Torque off FIRST, then everything else. Every print goes through
        `_say`, because after a SIGHUP the terminal is gone and a plain print
        raises -- which inside a `finally` would skip the steps after it."""
        try:
            self.bus.close()
        finally:
            if self.rec is not None:
                # SAID, AND NOT INTERRUPTIBLE. Torque is already off; the save
                # takes ~0.5 s per minute recorded on the Pi (a 973 s record:
                # 7.0 s to compress, 0.6 s to sync, 2026-10-27) and printed
                # nothing, so it read as a hang and got Ctrl-C'd -- which
                # raised inside the compress and left a partial capture.npz.
                # Further stop signals are only recorded until it is done.
                _STOP["defer"] = True
                _say(f"saving the record ({self.rec.n / CONTROL_HZ / 60:.1f} min, "
                     f"~{self.rec.n / CONTROL_HZ / 120:.0f} s) -- torque is "
                     "off; Ctrl-C waits for this")
                try:
                    d = self.rec.close(stopped_by=self._stopped_by,
                                       bad_reads=self._bad_reads_total)
                    _say(f"record: {self.rec.n} ticks -> {d} (on the card: "
                         "safe to power off)")
                except Exception as e:       # noqa: BLE001 -- never mask the stop
                    _say(f"record: close failed ({e}); rows.bin left in "
                         f"{self.rec.dir}, Capture.load reads it")
            self.ahrs.stop()
            self.link.stop()
            if self.jitter:
                j = np.fromiter(self.jitter, float) * 1e3
                _say(f"tick jitter: mean {j.mean():.2f} ms  "
                     f"p99 {np.percentile(j, 99):.2f} ms  max {j.max():.2f} ms")
                if self._late:
                    _say(f"  ticks over 10 ms late: {self._late}")


def lqr_unavailable(design, loop_hz: float) -> str | None:
    """Why this bundle cannot fly the LQR at `loop_hz`, or None if it can."""
    from ..control.lqr_design import STATE_NAMES
    if design.rate_hz is None or abs(design.rate_hz - loop_hz) > 1e-6:
        got = "an unstated rate" if design.rate_hz is None else f"{design.rate_hz:g} Hz"
        return (f"the bundle's gains are designed at {got}, this loop runs at "
                f"{loop_hz:g} Hz -- re-run `python -m aow_sim.export_deploy` "
                f"and copy deploy/bundle.npz to the bike")
    if design.Ks.shape[-1] != len(STATE_NAMES):
        return (f"the bundle's gains are {design.Ks.shape[-1]}-state, this code "
                f"flies {len(STATE_NAMES)} -- re-export and copy the bundle")
    return None


def controller_params(params: dict, loop_hz: float) -> dict:
    """`params` with `control.rate_hz` set to the rate this loop ACTUALLY
    ticks the controller at.

    THE CONTROLLER WAS RUNNING AT HALF SPEED ON THE BIKE until 2026-09-18.
    `control.rate_hz` is 200 -- the simulator's controller rate, and what the
    LQR schedule was designed at -- and DriveController took its `dt` from
    it. The bike ticks at 100 Hz. Every integrator and hold count inside the
    controller runs on `dt`, so on hardware:

        the policy was queried every 4 ticks = 25 Hz, trained at 50
        the steer target integrated at HALF the commanded rate
        the velocity low-pass had twice its time constant

    Found by replaying a bench record: the policy asked for -458 deg/s and
    the target moved at -229. Nothing had balanced on hardware yet, so no
    result is affected. Only the controller's copy changes: load_bundle still
    checks the digests against the real params, and the general policy does
    not use the 200 Hz LQR gains.
    """
    import copy
    out = copy.deepcopy(params)
    out["control"]["rate_hz"] = float(loop_hz)
    return out


class Stopped(BaseException):
    """A stop signal, raised in the main thread so the run unwinds through
    `finally`. BaseException, like KeyboardInterrupt, so no `except
    Exception` on the way can swallow it."""


# The signal handler's mailbox. `defer` is True while the control loop runs:
# the handler then only records the signal, and the loop acts on it between
# ticks (see BikeRunner.run for why).
_STOP = {"sig": None, "defer": False}


def _stopped(signum) -> BaseException:
    if signum == signal.SIGINT:
        return KeyboardInterrupt()
    return Stopped(f"stopped by {signal.Signals(signum).name}"
                   + (" (the ssh session closed)" if signum == signal.SIGHUP
                      else ""))


def _on_stop_signal(signum, _frame) -> None:
    if _STOP["defer"]:
        _STOP["sig"] = signum
        return
    raise _stopped(signum)


def _say(msg: str) -> None:
    """print, but a vanished terminal is not an error."""
    try:
        print(msg, flush=True)
    except OSError:
        pass


def _rpy(quat) -> tuple[float, float, float]:
    """(w,x,y,z) -> (roll, pitch, yaw), the ZYX convention extract_state uses."""
    from .ahrs import quat_to_mat
    R = quat_to_mat(quat)
    return (float(np.arctan2(R[2, 1], R[2, 2])),
            float(-np.arcsin(np.clip(R[2, 0], -1, 1))),
            float(np.arctan2(R[1, 0], R[0, 0])))


# A finding that is a STANDING CONDITION, not a fault: known, accepted, and
# true on every run until a separate piece of work clears it. Tagged rather
# than worded specially, so an escape hatch can be aimed at exactly this one.
MOUNT_UNCALIBRATED = "ahrs-mount-uncalibrated"
# config/ahrs_mount.yaml `source:` values preflight accepts as a located mount.
MOUNT_LOCATED = ("measured", "design")
# QoS 2 right after power-up: always a warning (preflight_ahrs).
AHRS_QOS_LIMITED = "ahrs-qos-limited"


def _report_preflight(problems, strict: bool, allow=()) -> list[str]:
    """Print preflight findings; raise on them only when strict.

    `problems` is a list of `(kind, text)` -- `kind` is None for an ordinary
    fault, or a tag like MOUNT_UNCALIBRATED for a standing condition. Anything
    whose kind is in `allow` prints as an accepted warning and does not block.

    WHY THE DISTINCTION EXISTS. The AHRS mount calibration is `GUESS` and stays
    `GUESS` until the bike can be jigged level on a level floor -- so it fired
    on EVERY run, and the only way past it was `--no-preflight`, which also
    disarms the |accel|, |gyro| and QoS checks. A gate that fires every single
    time is not a gate: it trains the operator to reach for the flag that turns
    off the gates which do catch things. Giving the standing condition its own
    narrow escape keeps the real ones armed.

    `strict` still defaults on, and `--no-preflight` remains for bench work
    where the bike is deliberately being moved or a sensor deliberately absent.
    """
    blocking = []
    for kind, text in problems:
        if kind is not None and kind in allow:
            print(f"PREFLIGHT WARNING (accepted): {text}")
        else:
            print(f"PREFLIGHT: {text}")
            blocking.append(text)
    if blocking and strict:
        only_mount = all(k == MOUNT_UNCALIBRATED for k, _ in problems)
        raise RuntimeError(
            f"{len(blocking)} preflight problem(s); "
            + ("pass --allow-guess-mount to accept it and keep the other "
               "checks armed" if only_mount else
               "fix them, or --no-preflight to disarm preflight entirely"))
    return [t for _, t in problems]


# NO gc.disable() HERE, and that is a measured decision rather than an
# oversight. Disabling the cyclic collector DID help once -- p99 28.5 -> 0.93 ms
# on a Pi 3B+ -- but only while the telemetry dict was being rebuilt every tick
# with an `np.percentile` over an unbounded list. With that fixed (bounded
# deque, percentile at 2 Hz, dict at 50 Hz to match the link), re-measured over
# 3000 ticks:
#
#     gc on : mean 0.14  p99 0.73  max 1.65 ms   (1638, 10, 1) collections
#     gc off: mean 0.16  p99 0.82  max 1.18 ms
#
# Indistinguishable, and `on` is marginally better on mean and p99. So the loop
# no longer generates enough garbage to matter, and disabling collection would
# be carrying a leak risk on a 1 GB board for nothing. Re-measure before
# reaching for it again.


def _try_realtime() -> None:
    """SCHED_FIFO for the control thread. Best effort — without CAP_SYS_NICE
    this fails, and a warning beats refusing to run."""
    try:
        import os
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
    except (AttributeError, PermissionError, OSError) as e:
        print(f"warning: no SCHED_FIFO ({e}); expect worse tick jitter")


def _gains(text: str) -> tuple:
    """`400:1920` -> (400, 1920). Same spelling as run_drive's --servo-gains."""
    try:
        p_gain, i_gain = text.split(":")
        return int(p_gain), int(i_gain)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"--servo-gains wants P:I, e.g. 400:1920; got {text!r}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bundle", default="deploy/bundle.npz")
    # Defaults come from control.onboard.{dxl_port,ahrs_port} when they are
    # set; the literals below are the generic fallbacks for a machine whose
    # config does not name them.
    ap.add_argument("--port", default=None,
                    help="U2D2 serial port (default: control.onboard.dxl_port)")
    ap.add_argument("--ahrs-port", default=None,
                    help="TM151 port (default: control.onboard.ahrs_port)")
    ap.add_argument("--rate", type=float, default=CONTROL_HZ)
    ap.add_argument("--no-preflight", action="store_true",
                    help="report AHRS preflight problems but engage anyway")
    ap.add_argument("--no-link", action="store_true",
                    help="bench mode: do not wait for a ground station and do "
                         "not treat its absence as a failure. The velocity "
                         "command is held at zero, so the policy balances and "
                         "does not travel")
    ap.add_argument("--servo-gains", type=_gains, metavar="P:I",
                    help="pin the drive servos' firmware Velocity P:I gains "
                         "(e.g. 400:1920) instead of taking the policy's own "
                         "training gains")
    ap.add_argument("--ids", type=lambda t: tuple(int(i) for i in t.split(",")),
                    help="drive_a,drive_b,steer -- e.g. 101,102,103 on the "
                         "bench. Defaults to control.onboard.servo_ids")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop cleanly after N seconds of loop time, torque "
                         "off on the way out. Use this rather than `timeout`, "
                         "whose SIGTERM skips the shutdown handler")
    ap.add_argument("--allow-guess-mount", action="store_true",
                    help="accept an unlocated AHRS mount (source GUESS, or no "
                         "config/ahrs_mount.yaml) and KEEP the rest of "
                         "preflight armed, rather than --no-preflight, which "
                         "also disarms the |accel|, |gyro| and QoS checks. "
                         "A 'design' or 'measured' mount needs no flag")
    ap.add_argument("--steer-zero", metavar="DEG|capture",
                    type=lambda t: t if t == "capture" else float(t),
                    help="servo-shaft angle that is STRAIGHT AHEAD, for one "
                         "session. `capture` calls the current pose zero, "
                         "which is what a bare bench shaft wants and what the "
                         "assembled bike must never do. Default: "
                         "control.onboard.steer_zero_deg")
    ap.add_argument("--no-torque", action="store_true",
                    help="run the whole loop but never enable torque. The "
                         "timing is real; nothing can move")
    ap.add_argument("--bench", action="store_true",
                    help="bench preset: --no-torque --allow-guess-mount. Add "
                         "--torque to energise anyway. The steer zero stays "
                         "the config's (operator's call, 2026-09-18: a bench "
                         "offset is visible in the mirror and harmless); pass "
                         "--steer-zero capture to call the current pose "
                         "straight")
    ap.add_argument("--torque", action="store_true",
                    help="with --bench: enable torque after all")
    ap.add_argument("--link-wait", type=float, default=0.0, metavar="S",
                    help="give up if no ground station within S seconds. "
                         "0 (default) waits forever -- start the bike, then "
                         "the mirror, whenever")
    ap.add_argument("--no-record", action="store_true",
                    help="do not write the onboard record. By default every "
                         "tick goes to traces/bike/<stamp>_run[_tag]/ on the "
                         "Pi -- a bench_log capture, see RunRecorder")
    ap.add_argument("--tag", help="suffix for the record's directory name")
    ap.add_argument("--hold", action="store_true",
                    help="lock the servos instead of balancing: drives hold "
                         "zero velocity, steer holds its angle, the policy "
                         "never engages. Enter / Y re-captures and holds, "
                         "Backspace goes limp. With --bench, add --torque")
    ap.add_argument("--move", metavar="NAME",
                    help="the general policy to fly this session, a moves/ "
                         "name (e.g. general_puffer_rake20_80M). Default: "
                         "control.general_move. Its firmware gains follow it")
    ap.add_argument("--pack-warn", type=float, default=PACK_WARN_V,
                    metavar="V", help=f"warn below V (default {PACK_WARN_V})")
    ap.add_argument("--pack-cut", type=float, default=PACK_CUT_V, metavar="V",
                    help=f"cut and hold below V (default {PACK_CUT_V}). The 12 V "
                         "brick rests at ~12.0 V, so --pack-warn 12.5 "
                         "--pack-cut 12.2 cuts on the first tick")
    ap.add_argument("--auto-rearm", action="store_true",
                    help="let the policy re-engage after a fall without the "
                         "operator asking. OFF by default -- a bike that "
                         "re-engages in your hands is worse than one that waits")
    args = ap.parse_args()
    if args.torque and not args.bench:
        ap.error("--torque only means something with --bench")
    torque = args.torque if args.bench else not args.no_torque
    steer_zero = args.steer_zero
    dxl_port, ahrs_port = _resolve_ports(args, load_params())
    print(f"bus  {dxl_port}\nahrs {ahrs_port}")
    BikeRunner(args.bundle, dxl_port, ahrs_port, args.rate,
               servo_gains=args.servo_gains,
               require_link=not args.no_link,
               auto_rearm=args.auto_rearm,
               ids=args.ids, torque=torque, seconds=args.seconds,
               steer_zero=steer_zero,
               allow_guess_mount=args.allow_guess_mount or args.bench,
               record=not args.no_record, tag=args.tag,
               link_wait=args.link_wait,
               pack_warn_v=args.pack_warn, pack_cut_v=args.pack_cut,
               move=args.move, hold=args.hold,
    ).run(preflight=not args.no_preflight)


def _resolve_ports(args, params) -> tuple:
    """(dxl, ahrs), from --port/--ahrs-port, else the config, else the generic
    device names. Separate and pure so a test can walk the precedence."""
    cfg = ((params.get("control") or {}).get("onboard") or {})
    return (args.port or cfg.get("dxl_port") or "/dev/ttyUSB0",
            args.ahrs_port or cfg.get("ahrs_port") or "/dev/serial0")


if __name__ == "__main__":
    main()
