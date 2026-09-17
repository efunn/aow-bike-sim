"""The onboard process: three threads, four failsafes, one control loop.

    python -m aow_sim.hw.run_bike --bundle deploy/bundle.npz

Structure (docs/plans/untethered-setup.md):
  control thread  SCHED_FIFO, 100 Hz — SyncRead -> DriveController -> BulkWrite
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
  2. pack voltage <10.2 V (3.4 V/cell) -> torque off. Read from the servos'
                   own Present Input Voltage register; no extra hardware.
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
import select
import socket
import threading
import time

import numpy as np

from collections import deque

from ..params import load_params
from ..control.drive import DriveController
from ..control.recovery import HANDOFF_RATE, RECOVER_DEG, ready_for_policy
from . import telemetry
from .ahrs import (QOS_MIN_SERVICE, QOS_NAMES, QOS_UNKNOWN,
                   AhrsReader, MountCalibration)
from .dynamixel import CONTROL_HZ_DEFAULT, ServoBus, resolve_gains
from .odometry import VelocityEstimator, body_to_world
from .state import HardwareData, load_ahrs_mount, load_bundle

CONTROL_HZ = CONTROL_HZ_DEFAULT   # see hw/dynamixel.py
CMD_STALE_S = 0.15        # -> zero the command. PROVISIONAL, see note below
CMD_DEAD_S = 1.0          # -> torque off
VOLTAGE_MIN = 10.2        # 3.4 V/cell on 3S. Pack total, not per-cell — see note
LINK_WAIT_S = 30.0        # how long to wait for the ground station before giving up

# Defaults for the fall guard, overridable from control.onboard in
# bike_params.yaml and from the command line.
#
# CUT: 60 deg is `fall_roll_deg` from rl_general.yaml -- the angle beyond which
# the policy never trained -- NOT a claim about recoverability.
#
# RE-ARM: NOT defined here. `control/recovery.py` owns the criterion and
# `control/righting.py`'s sequencer ends its lift phase on the same call, so
# there is one definition of "ready for the policy" and not two that drift.
# The numbers come from analysis/no_return.py. An earlier version of this file
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
#     startup the same way ServoBus asserts latency_timer.
#   * Client reconnect after an AP hiccup takes seconds, which is past
#     CMD_DEAD_S as well — that surfaces as an unexplained torque-off.
#
# Set this from a measured p99 before the first untethered run. See
# docs/plans/untethered-setup.md, "The radio, and what actually goes wrong with
# it" and Verification step 2b.
#
# VOLTAGE_MIN reads the pack total via the servos' address 144, which cannot see
# individual cells. A pack with one weak cell can sit at 3.6/3.6/3.0 V and still
# report 10.2 V, so the weak cell is over-discharged with nothing onboard to
# notice. That is an accepted limitation — the free voltage read is still the
# right call — but it assumes regular balance charging keeps the cells matched
# enough for the average to mean something.


class FallGuard:
    """Cut the policy out when the bike is on its side; let it back in.

    WHAT THIS IS NOT. It is not a detector for "unrecoverable". Unrecoverability
    is not a roll angle at all: `analysis/no_return.py` measures the point of no
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
        if not settled:
            self._settled_since = None
            return ""
        if self._settled_since is None:
            self._settled_since = t
        if t - self._settled_since < self.dwell_s:
            return ""
        if not (self.auto or self.consent):
            return ""
        self.state = "engaged"
        self.consent = False
        self._settled_since = None
        return "rearm"


class CommandLink:
    """UDP command receive + telemetry transmit.

    The command is a JSON object rather than a packed struct: it is 50 Hz of
    tiny datagrams on a private link, the readability is worth more than the
    bytes, and it keeps the ground station trivial to write and extend.
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
                self.cmd = json.loads(data.decode())
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
                 steer_zero=None, allow_guess_mount: bool = False):
        self.params = load_params()
        self.design, self.model = load_bundle(bundle_path, self.params)
        self.dt = 1.0 / control_hz
        self.ctl = DriveController(self.params, self.model, self.design)
        self.aid = self.ctl.aid
        self.data = HardwareData(self.model.nq, self.model.nv, self.model.nu)
        cfg = ((self.params.get("control") or {}).get("onboard") or {})
        self.require_link = bool(require_link)
        # WHICH policy, resolved HERE rather than at engage time, because the
        # firmware gains have to follow it and they are written before torque.
        self.gen_name = self.params["control"].get("general_move", "general_rl")
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
        self.bus = ServoBus(self.params, port=port, control_hz=control_hz,
                            ids=ids[:3], righting_id=righting, gains=gains,
                            righting_current=cfg.get("righting_current"),
                            servo_sign=cfg.get("servo_sign", (1.0, 1.0)))
        # A PERSISTED CONSTANT BEATS A CAPTURE, and this is the bike's default.
        # The policy observes the steer as sin/cos(2*delta) -- pi-periodic,
        # because the wheel is front-back symmetric -- the servo's encoder is
        # absolute within one turn, and bike.steering.gear_ratio is 1.0. So the
        # power-up reading already fixes the steer angle mod pi, and nothing
        # needs homing: what is needed is ONE number, the encoder reading with
        # the wheel physically straight, measured once at assembly.
        #
        # Left unset, ServoBus captures the current pose as zero instead, which
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
        self._roll = self._roll_rate = 0.0
        self._dt_meas = self.dt
        self._righting_current = None
        # Preflight findings whose KIND may be accepted without disarming the
        # rest of preflight. See _report_preflight.
        self._preflight_allow = ((MOUNT_UNCALIBRATED,) if allow_guess_mount
                                 else ())
        self._w_shaft = (0.0, 0.0)
        self._shaft = (0.0, 0.0)
        self._qos = None
        self._righting_pos = None

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
        s = self.bus.to_controller_units(self.bus.read_state())
        a = self.ahrs.latest()

        dt = s["dt"] if s["dt"] and 0.2 * self.dt < s["dt"] < 5 * self.dt else self.dt
        self._dt_meas = dt
        self.data.time += dt
        self.data.set_orientation(a.quat, a.gyro)
        self.data.qpos[self.ctl._sj] = s["steer_pos"]
        self.data.qvel[self.ctl._sd] = s["steer_vel"]

        roll, pitch, yaw = _rpy(a.quat)
        v_lon, v_lat = self.est.update(
            dt, s["w_servo_a"], s["w_servo_b"],
            steer_joint=s["steer_pos"], yaw_rate=a.gyro[2],
            accel_body=a.accel, roll=roll, pitch=pitch)
        self.data.set_velocity(body_to_world(v_lon, v_lat, yaw))
        self.data.integrate_position(dt)
        self._roll = roll
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

    def _apply_command(self) -> None:
        age = self.link.age()
        if age > CMD_STALE_S:
            # Hold heading, zero velocity: keep balancing, stop travelling.
            self.ctl.set_command(v_cmd_world=[0.0, 0.0])
            return
        c = self.link.cmd
        self.ctl.set_command(v_cmd_world=c.get("v_cmd_world", [0.0, 0.0]),
                             psi_cmd=c.get("psi_cmd"))
        self._apply_operator(c)

    def _apply_operator(self, c: dict) -> None:
        """The parts of the command struct the POLICY does not own.

        The self-righting servo is driven from here and nowhere else: four
        training runs established the general policy should not drive the wings
        (`docs/plans/self-righting.md`), so it is an operator control that
        happens to share a daisy chain. Goal Current is applied on change only
        -- it is a separate bus transaction, so sending it every tick would put
        an operator-scale knob on the control path.
        """
        if not self.bus.id_right:
            return
        if "righting_rad" in c:
            self.bus.set_righting(c["righting_rad"])
        want = c.get("righting_current")
        if want is not None and want != self._righting_current:
            self._righting_current = self.bus.set_righting_current(int(want))
        if c.get("rearm"):
            self.guard.request_rearm()

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
        if self.mount_source != "measured":
            problems.append((MOUNT_UNCALIBRATED,
                f"AHRS mount calibration is '{self.mount_source}', not 'measured' — "
                "assuming the sensor is perfectly aligned with the chassis. "
                "Any mounting tilt is a permanent roll bias."))

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
        elif qos < QOS_MIN_SERVICE:
            problems.append((None,
                f"AHRS reports QoS {qos} ({QOS_NAMES.get(qos, '?')}); the bike "
                f"needs at least {QOS_MIN_SERVICE} "
                f"({QOS_NAMES[QOS_MIN_SERVICE]}). Low grades right after "
                "power-on usually clear on their own — wait ~30 s for the "
                "gyro calibration and preflight again."))
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
        return _report_preflight(problems, strict, self._preflight_allow)

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
                    f"in ImuAssistant; `python analysis/tm151_serial.py` will "
                    f"name what it is sending instead")
        return (f"{r.frames} Ep_Combo frames decoded but the latest is stale -- "
                f"the reader is falling behind, not the sensor")

    def _check_failsafes(self, voltage: float) -> str | None:
        """The LATCHING failsafes -- the ones there is no coming back from.

        The fall is deliberately NOT here any more. It is the one condition
        that is expected, survivable and self-clearing, so it is a state
        transition (`FallGuard`) rather than a reason to stop the process; a
        bike that has to be power-cycled after every tip-over cannot be tested.
        The other three stay fatal because none of them clears itself: a dead
        link, a flat pack and a silent AHRS are all still true one second later.
        """
        if self.require_link and self.link.age() > CMD_DEAD_S:
            return f"command link dead ({self.link.age():.1f} s)"
        if voltage < VOLTAGE_MIN:
            return f"pack at {voltage:.1f} V (min {VOLTAGE_MIN})"
        return None

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
        print(f"waiting up to {timeout:.0f} s for a command on "
              f"{self.link.addr[0]}:{self.link.addr[1]} ...")
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if self.link.age() < CMD_DEAD_S:
                print(f"ground station at {self.link.peer[0]}")
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"no command in {timeout:.0f} s. Start the ground station, or run "
            f"with --no-link for a bench session with no operator.")

    def _engage(self) -> None:
        """Hand the actuators to the policy, from a clean slate.

        Called at startup AND on every re-arm after a fall, and it has to be
        the full reset both times. The controller carries state across ticks --
        `prev_action` in the policy observation, the 50 Hz zero-order-hold
        schedule, the command anchor -- and the estimator carries its rate
        filters and an integrated position. Resuming a policy with the
        prev_action it was using as it went over, and a velocity filter full of
        samples from the fall, is feeding it an observation from a bike that no
        longer exists.
        """
        self.est = VelocityEstimator(self.params)
        self._sense()
        self.ctl.reset(self.model, self.data)
        self.ctl.engage_general(self.data, name=self.gen_name)
        # The operator's last command predates the fall. Re-anchor to standing
        # still and let them ask again.
        self.ctl.set_command(v_cmd_world=[0.0, 0.0])

    def run(self, preflight: bool = True) -> None:
        self.bus.open()                      # torque still OFF; see ServoBus.open
        # What open() just wrote, so the operator's first [ or ] is a CHANGE
        # rather than a redundant write of the value already on the servo.
        self._righting_current = self.bus.righting_current_applied
        self.ahrs.start()
        self.link.start()
        _try_realtime()

        # Before any torque: the bike is stationary here and never again.
        # True as written now -- `ServoBus.open` no longer energises the servos.
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
            self.wait_for_link()

        voltage = self.bus.pack_voltage()
        # WHICH policy: `self.gen_name`, resolved in __init__ because the
        # firmware gains follow it and are written before torque. It used to be
        # engage_general with no name at all, which silently took the hardcoded
        # "general_rl" default -- so the BIKE always drove general_rl no matter
        # what control.general_move said, while teleop honoured it. The two
        # disagreed, and the one that mattered was the one nobody could see.
        print(f"pack {voltage:.1f} V — engaging general policy {self.gen_name} "
              f"at {1/self.dt:.0f} Hz")
        print(f"  {self.gain_note}")
        print(f"  cut at {np.degrees(self.guard.cut):.0f} deg, re-arm below "
              f"{np.degrees(self.guard.rearm):.0f} deg"
              + ("" if self.guard.auto else " on request"))

        self.data.time = 0.0
        self._engage()
        if self.torque:
            self.bus.arm()                   # <- the first torque of the run
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

        t0 = time.monotonic()
        next_tick = t0
        k = 0
        try:
            while True:
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
                if k % 100 == 0:            # 1 Hz; a bus read is not free
                    voltage = self.bus.pack_voltage()
                if k % 50 == 0 and self.jitter:   # 2 Hz; a percentile is not free
                    self._jitter_ms = round(
                        float(np.percentile(np.fromiter(self.jitter, float), 99))
                        * 1e3, 2)
                if self.seconds and self.data.time >= self.seconds:
                    print(f"reached --seconds {self.seconds:g}")
                    break
                reason = self._check_failsafes(voltage)
                if reason is not None:
                    print(f"FAILSAFE: {reason}")
                    break

                # The fall guard runs on the SENSED state and before the
                # controller, so a tick that starts on the bike's side never
                # reaches the policy at all.
                event = self.guard.update(self.data.time, self._roll,
                                          self._roll_rate)
                if event == "cut":
                    # Torque off the three the policy drives, and ONLY those:
                    # the righting servo has to stay alive here, since lying
                    # down is the one moment it has a job.
                    self.bus.torque(False, self.bus.policy_ids)
                    print(f"CUT: roll {np.degrees(self._roll):.0f} deg — policy "
                          f"out, righting servo still live")
                elif event == "rearm":
                    self._engage()
                    self.bus.arm(self.bus.policy_ids)
                    print("RE-ARMED: policy engaged, command zeroed")

                if self.guard.state == "engaged":
                    self.ctl.step(self.model, self.data)
                    self.bus.write_commands(self.data.ctrl, self.aid)

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
                    state=self.guard.state,
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
                    psi=self.ctl._psi,
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
                    righting_current=self._righting_current,
                )
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        try:
            self.bus.close()
        finally:
            self.ahrs.stop()
            self.link.stop()
            if self.jitter:
                j = np.fromiter(self.jitter, float) * 1e3
                print(f"tick jitter: mean {j.mean():.2f} ms  "
                      f"p99 {np.percentile(j, 99):.2f} ms  max {j.max():.2f} ms")
                if self._late:
                    print(f"  ticks over 10 ms late: {self._late}")


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
                    help="accept an uncalibrated AHRS mount and KEEP the rest "
                         "of preflight armed. This is the flag a bench session "
                         "wants: the mount stays 'GUESS' until the bike can be "
                         "jigged level, so without it preflight blocks every "
                         "run and the habit becomes --no-preflight, which also "
                         "disarms the |accel|, |gyro| and QoS checks")
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
    ap.add_argument("--auto-rearm", action="store_true",
                    help="let the policy re-engage after a fall without the "
                         "operator asking. OFF by default -- a bike that "
                         "re-engages in your hands is worse than one that waits")
    args = ap.parse_args()
    dxl_port, ahrs_port = _resolve_ports(args, load_params())
    print(f"bus  {dxl_port}\nahrs {ahrs_port}")
    BikeRunner(args.bundle, dxl_port, ahrs_port, args.rate,
               servo_gains=args.servo_gains,
               require_link=not args.no_link,
               auto_rearm=args.auto_rearm,
               ids=args.ids, torque=not args.no_torque, seconds=args.seconds,
               steer_zero=args.steer_zero,
               allow_guess_mount=args.allow_guess_mount,
    ).run(preflight=not args.no_preflight)


def _resolve_ports(args, params) -> tuple:
    """(dxl, ahrs), from --port/--ahrs-port, else the config, else the generic
    device names. Separate and pure so a test can walk the precedence."""
    cfg = ((params.get("control") or {}).get("onboard") or {})
    return (args.port or cfg.get("dxl_port") or "/dev/ttyUSB0",
            args.ahrs_port or cfg.get("ahrs_port") or "/dev/serial0")


if __name__ == "__main__":
    main()
