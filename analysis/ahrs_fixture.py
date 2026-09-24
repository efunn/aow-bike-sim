"""AHRS fixture: two XL330s move the TM151 through known angles, and their
encoders are the truth its orientation is scored against.

WHY. `docs/status.md` traces the standing falls to the ROLL orientation error,
and `sim_ahrs.ORIENT_RMS_DEG` carries the datasheet's "<1.5 deg" dynamic bound
as if it were an RMS. The resting error is measured (tau 0.19 s,
`tm151_check.py`); the DYNAMIC one is not, and a bike on the floor cannot give
it because nothing on the bike knows the true attitude. Two servos can: a 12-bit
absolute encoder on each output shaft is 0.088 deg per count, ~20x finer than
the error being measured.

AND WHERE THE AHRS MAY GO. Its place on the bike is not settled, and the
datasheet's own footnote [3] says a part without "vibration resistance" (the
TM151) is "susceptible to low frequency linear acceleration". Every mounting
position turns the bike's rotation into a different acceleration at the
sensor, so the second product of this rig is ERROR AGAINST NON-GRAVITY
ACCELERATION, measured at several heights above the roll axis. Candidate
positions on the bike are then priced by the acceleration the sim puts there.
Each segment's row carries `a_perp` for that reason: the RMS of the measured
acceleration PERPENDICULAR TO GRAVITY, which is the part that tilts the
sensor's gravity reference.

THE RIG. A serial chain, clamped to a roughly flat surface:

    base -- XL330 id 151 (YAW) -- bracket -- XL330 id 152 (ROLL) -- TM151

Both in Position Control Mode (3), 3 Mbps, centred at 2048 counts (180 deg on
the horn). The cables U2D2->151 and 151->152 cross the joints, so travel is
+-45 deg, held in the servo's own EEPROM (Min/Max Position Limit, written by
`setup`) and clamped to +-40 in software before every write. The Pi runs it
off the same two USB ports as the bike: U2D2 and the AHRS's STM32 CDC.

    python analysis/ahrs_fixture.py check                 # read-only
    python analysis/ahrs_fixture.py setup                 # EEPROM: mode 3, +-45 deg
    python analysis/ahrs_fixture.py jog                   # the ident moves only
    python analysis/ahrs_fixture.py tune                  # servo P/I/D, no AHRS needed
    python analysis/ahrs_fixture.py session --note "h=180 mm spacer"   # the lot, ~16 min
    python analysis/ahrs_fixture.py analyse traces/ahrs_fixture/<capture dir> [--plot]

    # one test at a time
    python analysis/ahrs_fixture.py rest   --seconds 300  # servos holding, no motion
    python analysis/ahrs_fixture.py step                  # fast roll steps, 20 s holds
    python analysis/ahrs_fixture.py chirp                 # log sweep 0.1 -> 8 Hz, 2 amps
    python analysis/ahrs_fixture.py replay                # the sim's standing flights
    python analysis/ahrs_fixture.py sway   --seconds 180  # band-limited random roll
    python analysis/ahrs_fixture.py sine                  # amplitude x frequency grid

    # on the Mac, once: the sim's motion, for `replay`
    python analysis/ahrs_fixture.py export-replay
    rsync -av traces/ahrs_fixture/replay_standing.npz \\
        efun@aowbike.local:'~/aow-bike-sim/traces/ahrs_fixture/'

THE TESTS, and what each is for. Motion statistics are from 60 simulated
standing flights on the bike's own sensors (`ahrs_fall_cause.py motion`):
roll 2.3 deg RMS, roll rate 35 deg/s RMS (103 p99), roll acceleration
906 deg/s^2 RMS (2707 p99), yaw rate 41 deg/s RMS; half the roll power below
1.3 Hz, 90% below 3.9 Hz.

    rest    the fixture's own floor: does servo dither add error?
    step    fast roll steps at the bike's p99 rate, then 20 s holds: how long
            the fusion takes to recover, and its static error at a lean
    chirp   log sweep, acceleration-capped at the bike's p99: error against
            FREQUENCY, reported per octave -- the curve behind every other
            number
    replay  the sim's own roll and yaw -- THE HEADLINE: dynamic RMS and tau
            under the motion that makes the bike fall. Yaw is high-passed to
            fit the +-40 deg travel (the cutoff is recorded); rates survive,
            slow heading wander does not. The bike's pitch and translation are
            not reproduced -- a two-joint fixture can make neither.
    yaw     the same flights, yaw only: does heading motion leak into tilt?
    sway    random roll with the sim's band, so the answer is not tied to the
            sim's exact trajectories

`session` runs them all behind one prologue, as one capture.

Every motion capture opens with the same PROLOGUE: home, 5 s held at centre (the
reference pose), then a yaw-only and a roll-only sine. The analysis fits the
two joint axes in the SENSOR frame from those, so neither the mount on the horn
nor the bracket has to be square, or even known. The capture is therefore
self-contained: nothing measured on one day is needed to analyse another.

TRUTH. With R0 the AHRS attitude at the reference pose and u_y, u_r the fitted
yaw and roll axes (sensor frame, at the reference pose),

    R_true(t) = R0 . Rot(u_y, psi(t) - psi0) . Rot(u_r, phi(t) - phi0)

which is exact for a serial yaw-then-roll chain (derivation in `truth_mats`).
R0 comes from the AHRS itself, so its static error AT THE REFERENCE POSE is
subtracted by construction: everything reported is error RELATIVE TO REST,
which is the dynamic part the datasheet's dynamic row describes. The static
error at other poses (accel misalignment, the 0.5 deg row) is still in it --
`step` reports it.

THE LEVER ARM IS MEASURED, NOT ASSUMED. The accelerometer reads gravity plus
alpha x r + omega x (omega x r), with r from the joint axes to the SENSING
POINT. Truth gives gravity, the gyro gives omega and alpha, so r comes out of
a linear least-squares fit (`fit_lever_arm`). The datasheet marks its axes at
the centre of the footprint but not the element's height in the 12.6 mm case,
and this makes that moot: the report gives the sensing point's distance from
each axis. Exact for axes that INTERSECT; build the bracket so they do.

WHAT IS REPORTED, per segment:
  * roll / pitch / heading error RMS, in the sensor's HEADING frame -- roll is
    about the horizontal projection of the sensor's x axis. Mount the unit
    x-along-the-roll-axis and "roll" means what the bike means by it.
  * twice: LAG-ALIGNED (the fitted sensor delay removed) and AS-RECEIVED (the
    frame against the truth at the moment it arrived on the Pi, which is what a
    controller sees).
  * `a_perp`: non-gravity acceleration perpendicular to gravity, RMS, measured.
  * the correlation time of the error, on segments >= 30 s.
  * chirps broken down per octave, each with the AHRS's GAIN AND PHASE about
    the roll joint (delay removed; the delay is reported separately). Gain < 1
    means it under-reads the lean -- the pendulum case, sensor below the roll
    axis -- and > 1 over-reads it, above. RMS alone cannot tell those apart,
    and a controller can. Roll chirps only.
  * steps by recovery time and final error.
  * gyro scale factor and the angle between the fitted axes, from the ident
    moves. NOT a gyro noise figure: the encoder rate is several deg/s noisy.

TIMING. Servos: `bench_log.record`, host clock at mid-read. AHRS: every Combo
frame, stamped with the same `perf_counter` on arrival, plus the sensor's own
microsecond clock -- which runs 0.3% FAST (200.00 Hz by its own stamps, 200.61
by the Pi's, 2026-09-23). The device clock is mapped to the host by a line
fitted under the arrival times (the fastest arrivals), which removes the drift
and the USB and scheduler jitter; what remains is a constant offset, and the
lag fit measures that.

RAW OUTPUT (41). Not needed: at rest Combo's gyro and accel are the raw
packet's to within noise (2026-09-23, 10 s each, gyro std 0.25/0.27/0.14 deg/s
on both, means within one standard error). `--raw-hz` polls it alongside Combo
with an Ep_Request -- no change to the sensor's flash. Polling at 100 Hz
returned 67 Hz of raw frames and left Combo at 200.0 Hz.

NOTHING HERE IMPORTS MUJOCO, so it runs on the Pi. Captures go to
`traces/ahrs_fixture/` (gitignored, Dropbox-synced); pull them back with

    rsync -av efun@aowbike.local:'~/aow-bike-sim/traces/ahrs_fixture/' traces/ahrs_fixture/
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import tm151_serial  # noqa: E402
from tm151_serial import G_VENDOR, Decoder, request  # noqa: E402

YAW, ROLL = 151, 152
IDS = (YAW, ROLL)
MODEL = "xl330_m288"
CENTER = np.pi                  # 2048 counts, the horn's 180 deg mark
LIMIT_DEG = 45.0                # EEPROM Min/Max Position Limit, from `setup`
SOFT_DEG = 40.0                 # every goal is clamped here first
COUNTS = 4096
MODE_POSITION = 3
READ_BLOCK = ("Realtime Tick", "Present Position", "Present Velocity",
              "Present Current")
EP_RAW = 41
OUT = ROOT / "traces" / "ahrs_fixture"
REPLAY_FILE = OUT / "replay_standing.npz"
FALL_CAUSE_MOTION = ROOT / "traces" / "ahrs_fall_cause_motion.pkl"
# "Settled" after a step: the error within this of its final value. 15x the
# 0.013 deg RMS of the fused output sitting on the desk (2026-09-23), so noise
# alone does not hold it open.
SETTLE_DEG = 0.2
PARTS = ("rest", "step", "chirp", "replay", "yaw", "sway")


def onboard_ports() -> tuple:
    """`control.onboard.{dxl,ahrs}_port` -- the bike's by-id paths, same USB."""
    try:
        import yaml
        cfg = yaml.safe_load((ROOT / "config" / "bike_params.yaml").read_text())
        ob = cfg["control"]["onboard"]
        return ob.get("dxl_port"), ob.get("ahrs_port")
    except Exception:                        # noqa: BLE001 -- a default, not a rule
        return None, None


# --------------------------------------------------------------------------
# AHRS logging: EVERY frame, not the latest-value slot `hw.ahrs` keeps
# --------------------------------------------------------------------------

class AhrsLog:
    """Background reader keeping every Combo (and polled raw) frame.

    `hw.ahrs.AhrsReader` keeps only the newest sample -- right for a control
    loop, wrong here, where a dropped frame is a hole in the error series. So
    this reuses the analysis decoder, which yields every packet in a chunk.

    The first Combo after opening the port is the last one of the PREVIOUS
    session (device clock 2-21 s behind, measured 2026-09-22). It lands before
    the capture starts and the clock fit drops it as an outlier.
    """

    def __init__(self, port: str, baud: int = 460800, raw_hz: float = 0.0):
        self.port, self.baud, self.raw_hz = port, baud, float(raw_hz)
        self.combo: list = []
        self.raw: list = []
        self.other: dict = {}
        self.requests = 0
        self._stop = threading.Event()
        self._thread = None
        self.dec = Decoder()

    def start(self) -> "AhrsLog":
        import serial

        # The table-driven CRC, not the reference bit loop: identical output
        # (`tests/test_hw_ahrs.py`), ~8x less time holding the GIL per frame,
        # which is time taken out of the servo loop on the same interpreter.
        from aow_sim.hw.ahrs import crc16_modbus
        tm151_serial.crc16 = crc16_modbus

        self._ser = serial.Serial(self.port, self.baud, timeout=0.005)
        self._ser.reset_input_buffer()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="ahrs-log")
        self._thread.start()
        return self

    def _run(self) -> None:
        clock, ser, dec = time.perf_counter, self._ser, self.dec
        period = 1.0 / self.raw_hz if self.raw_hz > 0 else 0.0
        due = 0.0
        while not self._stop.is_set():
            if period and clock() >= due:
                ser.write(request(EP_RAW))
                self.requests += 1
                due = clock() + period
            n = ser.in_waiting
            chunk = ser.read(n if n else 1)
            if not chunk:
                continue
            t = clock()
            for p in dec.feed(chunk):
                f = p.fields
                if p.kind == "combo":
                    self.combo.append((t, p.t_us, *f["quat"], *f["gyro"],
                                       *f["acc"], *f["rpy_deg"], f["qos"],
                                       f["temp_c"]))
                elif p.kind == "raw_gyro_acc_mag":
                    self.raw.append((t, p.t_us, *f["gyro"], *f["acc"], *f["mag"]))
                else:
                    self.other[p.kind] = self.other.get(p.kind, 0) + 1

    def wait(self, n: int = 20, timeout: float = 2.0) -> None:
        t0 = time.monotonic()
        while len(self.combo) < n:
            if time.monotonic() - t0 > timeout:
                raise SystemExit(
                    f"AHRS: {len(self.combo)} Combo frames in {timeout:g} s on "
                    f"{self.port} ({self.dec.n_ok} packets, other {self.other}, "
                    f"{self.dec.n_crc_bad} CRC failures). Is it plugged in and "
                    f"streaming Combo?")
            time.sleep(0.02)

    def qos(self) -> int:
        return int(self.combo[-1][15]) if self.combo else -1

    def wait_qos(self, min_qos: int, timeout: float) -> None:
        """Hold until the sensor grades itself `min_qos` or better.

        EasyObjectDictionary.h: 4 is "fine service", typically once
        DynamicGyroCalib has succeeded ~30 s after boot, and 5 "very good",
        after two successes ~5 min in. Measuring a unit that is still
        calibrating would put its warm-up into the dynamic figure.
        """
        t0 = last = time.monotonic()
        while self.qos() < min_qos:
            now = time.monotonic()
            if now - t0 > timeout:
                raise SystemExit(f"AHRS qos still {self.qos()} after {timeout:.0f} s "
                                 f"(want >= {min_qos}); pass --min-qos 0 to go anyway")
            if now - last > 10:
                print(f"  waiting for AHRS qos >= {min_qos}: {self.qos()} "
                      f"after {now - t0:.0f} s")
                last = now
            time.sleep(0.2)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._ser.close()

    def arrays(self, t0: float) -> dict:
        """Everything, on the servo capture's timebase (`t0_perf_counter`)."""
        c = np.asarray(self.combo, float).reshape(-1, 17)
        out = {"t_host": c[:, 0] - t0, "t_us": c[:, 1].astype(np.int64),
               "quat": c[:, 2:6], "gyro": c[:, 6:9], "acc_g": c[:, 9:12],
               "rpy_deg": c[:, 12:15], "qos": c[:, 15].astype(np.int8),
               "temp_c": c[:, 16],
               "crc_bad": np.array(self.dec.n_crc_bad),
               "resync_bytes": np.array(self.dec.n_resync),
               "requests": np.array(self.requests)}
        if self.raw:
            r = np.asarray(self.raw, float)
            out.update(raw_t_host=r[:, 0] - t0, raw_t_us=r[:, 1].astype(np.int64),
                       raw_gyro=r[:, 2:5], raw_acc_g=r[:, 5:8], raw_mag=r[:, 8:11])
        return out


# --------------------------------------------------------------------------
# Motion: every command is a function of segment time, clamped at SOFT_DEG
# --------------------------------------------------------------------------

def _smooth(x):
    """0 -> 1 with zero slope at both ends (smoothstep). Peak slope 1.5."""
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def _env(t, seconds, ramp=1.0):
    return _smooth(t / ramp) * _smooth((seconds - t) / ramp)


def goal(yaw_deg: float, roll_deg: float) -> dict:
    y = float(np.clip(yaw_deg, -SOFT_DEG, SOFT_DEG))
    r = float(np.clip(roll_deg, -SOFT_DEG, SOFT_DEG))
    return {YAW: CENTER + np.radians(y), ROLL: CENTER + np.radians(r)}


def _on(axis: str, x: float) -> dict:
    return goal(x, 0.0) if axis == "yaw" else goal(0.0, x)


def _seg(label, seconds, command, **info):
    from aow_sim.hw.bench_log import Segment
    return Segment(label, float(seconds), command=command, info=info)


def hold(seconds: float, label: str = "hold"):
    return _seg(label, seconds, lambda t, s: goal(0.0, 0.0), kind="hold")


def home(seconds: float = 2.0):
    """Ease from wherever the joints are to centre. Torque-on never jumps:
    `run` writes the present position as the goal before enabling torque."""
    start = {}

    def cmd(t, s):
        if not start:
            if s is None:
                return None
            start.update({i: np.degrees(s[i]["Present Position"] - CENTER)
                          for i in IDS})
        k = 1.0 - _smooth(t / seconds)
        return goal(start[YAW] * k, start[ROLL] * k)
    return _seg("home", seconds, cmd, kind="home")


def sine(axis: str, amp_deg: float, f_hz: float, seconds: float, label=None):
    ramp = min(1.0 / f_hz, 1.0)

    def cmd(t, s):
        return _on(axis, amp_deg * _env(t, seconds, ramp) * np.sin(2 * np.pi * f_hz * t))
    return _seg(label or f"sine_{axis}_{amp_deg:g}deg_{f_hz:g}Hz", seconds, cmd,
                kind="sine", axis=axis, amp_deg=amp_deg, f_hz=f_hz)


def step(axis: str, to_deg: float, rate_dps: float, hold_s: float,
         from_deg: float = 0.0, label=None):
    """Smoothstep from `from_deg` to `to_deg` at a peak rate of `rate_dps`,
    then hold. The move is timed so the PEAK rate is the one asked for
    (smoothstep's peak slope is 1.5x its mean)."""
    move_s = max(1.5 * abs(to_deg - from_deg) / rate_dps, 0.02)

    def cmd(t, s):
        return _on(axis, from_deg + (to_deg - from_deg) * _smooth(t / move_s))
    return _seg(label or f"step_{axis}_{from_deg:+g}_to_{to_deg:+g}", move_s + hold_s,
                cmd, kind="step", axis=axis, from_deg=from_deg, to_deg=to_deg,
                move_s=move_s, rate_dps=rate_dps)


def sweep_profile(amp_deg: float, rate_dps: float, a_max: float, cycles: int,
                  dt: float = 0.001):
    """Position [deg] for constant-rate sweeps between about +-amp: start at
    rest at 0, accelerate, cruise, reverse, ..., end at rest at 0. Every change
    of speed is a half-cosine with peak acceleration a_max, so in the CRUISE the
    joint turns at `rate_dps` with zero angular acceleration: rotation with
    (almost) no acceleration at the sensor, which is the point."""
    Ta = np.pi * rate_dps / (2 * a_max)          # 0 -> rate
    Tr = np.pi * rate_dps / a_max                # +rate -> -rate
    e = rate_dps * Tr / np.pi                    # a reversal's overshoot
    da = rate_dps * Ta / 2                       # distance while accelerating
    P = amp_deg - e                              # where reversals start
    if P <= da:
        raise ValueError(f"sweep at {rate_dps:g} deg/s needs more than +-{amp_deg:g} deg "
                         f"at {a_max:g} deg/s^2")
    pieces = []                                  # (kind, seconds)
    pieces += [("up", Ta), ("cruise", (P - da) / rate_dps), ("rev", Tr)]
    for c in range(cycles):
        pieces += [("cruise", 2 * P / rate_dps), ("rev", Tr)]
        pieces += [("cruise", 2 * P / rate_dps) if c < cycles - 1 else
                   ("cruise", (P - da) / rate_dps), ("rev", Tr) if c < cycles - 1 else
                   ("down", Ta)]
    v, sign = [], 1.0
    for kind, T in pieces:
        tau = np.arange(0.0, T, dt)
        if kind == "up":
            v.append(sign * rate_dps * (1 - np.cos(np.pi * tau / T)) / 2)
        elif kind == "down":
            v.append(sign * rate_dps * (1 + np.cos(np.pi * tau / T)) / 2)
        elif kind == "cruise":
            v.append(np.full(len(tau), sign * rate_dps))
        else:
            v.append(sign * rate_dps * np.cos(np.pi * tau / T))
            sign = -sign
    v = np.concatenate(v)
    t = np.arange(len(v) + 1) * dt
    return t, np.concatenate([[0.0], np.cumsum(v) * dt])


def sweep(axis: str, amp_deg: float, rate_dps: float, a_max: float, cycles: int):
    t_p, p = sweep_profile(amp_deg, rate_dps, a_max, cycles)

    def cmd(t, s):
        return _on(axis, float(np.interp(t, t_p, p)))
    return _seg(f"sweep_{axis}_{rate_dps:g}", float(t_p[-1]), cmd, kind="sweep",
                axis=axis, rate_dps=rate_dps, amp_deg=amp_deg, a_max=a_max)


def chirp_amp(f, amp_deg, a_max, rate_max):
    """Amplitude [deg] at frequency f: the asked amplitude, cut so neither the
    peak acceleration nor the peak rate exceeds its cap."""
    w = 2 * np.pi * np.asarray(f, float)
    return np.minimum(amp_deg, np.minimum(a_max / w ** 2, rate_max / w))


def chirp(axis: str, amp_deg: float, f0: float, f1: float, seconds: float,
          a_max: float, rate_max: float, label=None):
    """Exponential sweep f0 -> f1: equal time per octave. The amplitude follows
    `chirp_amp` of the INSTANTANEOUS frequency, so the high end is capped at the
    bike's own acceleration rather than at whatever the servo will do."""
    k = f1 / f0
    L = np.log(k)

    def cmd(t, s):
        f = f0 * k ** (t / seconds)
        ph = 2 * np.pi * f0 * seconds / L * (k ** (t / seconds) - 1)
        return _on(axis, _env(t, seconds) * chirp_amp(f, amp_deg, a_max, rate_max)
                   * np.sin(ph))
    return _seg(label or f"chirp_{axis}_{amp_deg:g}deg", seconds, cmd, kind="chirp",
                axis=axis, amp_deg=amp_deg, f0=f0, f1=f1, a_max=a_max,
                rate_max=rate_max)


def band_noise(seconds: float, rate: float, rms: float, lo: float, hi: float,
               rng) -> np.ndarray:
    """Gaussian noise band-limited to [lo, hi] Hz by FFT mask, scaled to `rms`."""
    n = int(seconds * rate) + 2
    X = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / rate)
    X[(f < lo) | (f > hi)] = 0.0
    y = np.fft.irfft(X, n)
    return y * (rms / y.std()) if rms > 0 else np.zeros(n)


def track(yaw_deg, roll_deg, rate: float, label: str, kind: str, **info):
    """Follow a sampled trajectory, faded in and out over 1 s."""
    yaw_deg, roll_deg = np.asarray(yaw_deg, float), np.asarray(roll_deg, float)
    seconds = (len(roll_deg) - 1) / rate
    tt = np.arange(len(roll_deg)) / rate

    def cmd(t, s):
        e = _env(t, seconds)
        return goal(e * np.interp(t, tt, yaw_deg), e * np.interp(t, tt, roll_deg))
    return _seg(label, seconds, cmd, kind=kind, **info)


def sway(seconds: float, roll_rms: float, yaw_rms: float, lo: float, hi: float,
         seed: int, label: str = "sway"):
    rate = 1000.0
    rng = np.random.default_rng(seed)
    r = band_noise(seconds, rate, roll_rms, lo, hi, rng)
    y = band_noise(seconds, rate, yaw_rms, lo, hi, rng)
    return track(y, r, rate, label, "sway", roll_rms_deg=roll_rms,
                 yaw_rms_deg=yaw_rms, band_hz=[lo, hi], seed=seed)


# -- replay: the sim's own standing flights ---------------------------------

def highpass(x, rate: float, fc: float, order: int = 2) -> np.ndarray:
    """Zero-phase Butterworth-magnitude high-pass by FFT, after removing the
    line through the end points so the circular FFT sees no step at the wrap."""
    x = np.asarray(x, float)
    x = x - np.linspace(x[0], x[-1], len(x))
    f = np.fft.rfftfreq(len(x), 1.0 / rate)
    r = (f / fc) ** order
    return np.fft.irfft(np.fft.rfft(x) * r / np.sqrt(1 + r * r), len(x))


def fit_travel(yaw_deg, rate: float, limit: float, fc0: float = 0.02):
    """High-pass yaw at the LOWEST cutoff that fits it inside +-limit.
    -> (yaw_hp, fc). The rates the sensor feels survive; the heading wander a
    +-45 deg joint cannot follow is what goes."""
    fc = fc0
    while True:
        y = highpass(yaw_deg, rate, fc)
        if np.abs(y).max() <= limit or fc > 2.0:
            return y, fc
        fc *= 1.25


def load_replay(path) -> dict:
    with np.load(path) as z:
        return {k: z[k] for k in z.files}


def replay_flights(rep: dict, which, n_default: int) -> list:
    """Flight indices: as given, or half that fell and half that did not."""
    if which:
        return list(which)
    fell = np.flatnonzero(rep["fell"])
    kept = np.flatnonzero(~rep["fell"].astype(bool))
    h = n_default // 2
    return sorted(list(fell[:n_default - h]) + list(kept[:h]))


def replay_segments(a, yaw_only: bool = False) -> list:
    rep = load_replay(a.replay_file)
    rate = float(rep["rate"])
    start = rep["start"]
    idx = replay_flights(rep, a.yaw_flights if yaw_only else a.flights,
                         2 if yaw_only else 4)
    segs = []
    for k in idx:
        roll = rep["roll_deg"][start[k]:start[k + 1]].astype(float)
        yaw = rep["yaw_deg"][start[k]:start[k + 1]].astype(float)
        yaw_hp, fc = fit_travel(yaw, rate, a.yaw_limit)
        if yaw_only:
            roll = np.zeros_like(roll)
        seed = int(rep["seed"][k])
        segs += [track(yaw_hp, np.clip(roll, -SOFT_DEG, SOFT_DEG), rate,
                       f"{'yaw' if yaw_only else 'replay'}_seed{seed}",
                       "yaw" if yaw_only else "replay",
                       flight=int(k), seed=seed, fell=bool(rep["fell"][k]),
                       yaw_highpass_hz=fc, source=str(a.replay_file),
                       target_roll_rms_deg=float(np.std(roll)),
                       target_roll_rate_rms_dps=float(np.std(np.gradient(roll, 1 / rate))),
                       target_yaw_rate_rms_dps=float(np.std(np.gradient(yaw_hp, 1 / rate)))),
                 hold(2.0)]
    return segs


# -- plans -------------------------------------------------------------------

def prologue(a) -> list:
    return [home(), hold(5.0, "ref"),
            sine("yaw", a.ident_deg, 0.5, 6.0, "ident_yaw"), hold(2.0, "settle"),
            sine("roll", a.ident_deg, 0.5, 6.0, "ident_roll"), hold(3.0, "settle")]


def step_segments(a) -> list:
    segs = []
    for amp in a.step_amps:
        segs += [step(a.axis, amp, a.step_rate, a.step_hold),
                 step(a.axis, 0.0, a.step_rate, a.step_hold, from_deg=amp)]
    return segs


def chirp_segments(a) -> list:
    segs = []
    for amp in a.chirp_amps:
        segs += [chirp(a.axis, amp, a.chirp_band[0], a.chirp_band[1],
                       a.chirp_seconds, a.a_max, a.max_rate), hold(2.0)]
    return segs


def sway_segments(a) -> list:
    return [sway(a.sway_seconds, a.roll_rms, a.yaw_rms, a.band[0], a.band[1],
                 a.seed), hold(2.0)]


@dataclass
class Plan:
    segments: list
    torque: bool = True
    brief: str = ""
    ahrs: bool = True           # False: servo-only (`tune`), no AHRS needed


def _framed(a, body: list, brief: str) -> Plan:
    end = getattr(a, "end_hold", 5.0)
    return Plan(prologue(a) + body + [hold(end, "ref_end")], brief=brief)


def plan_jog(a) -> Plan:
    return _framed(a, [], f"home, hold, yaw then roll +-{a.ident_deg:g} deg at 0.5 Hz")


def plan_rest(a) -> Plan:
    if a.limp:
        return Plan([_seg("ref", a.rest_seconds, None, kind="hold")], torque=False,
                    brief="torque OFF: the fixture as a plain stand")
    return Plan([home(), hold(a.rest_seconds, "ref")],
                brief="servos holding centre: the static figure WITH servo "
                      "dither and the fixture's own vibration")


def _led(on: bool):
    def enter(bus):
        for dxl in IDS:
            bus.write_raw(dxl, "LED", int(on))
    return enter


def plan_wiggle(a) -> Plan:
    """Hold centre with torque on while someone works the joints by hand.

    The LEDs light for exactly the hand-on window. The encoders are on the
    output shafts, so what they see is the gear train giving way against the
    position loop; what the AHRS sees beyond that is play between the horn
    and the sensor -- the two halves of the lost motion, separately.
    """
    hands = _seg("wiggle", a.wiggle_seconds, lambda t, s: goal(0.0, 0.0), kind="hold")
    hands.enter = _led(True)
    after = hold(3.0, "ref_end")
    after.enter = _led(False)
    return Plan([home(), hold(3.0, "ref"), hands, after],
                brief=f"hold centre; LEDs on for {a.wiggle_seconds:g} s: wiggle the "
                      f"joints by hand while they are lit")


def plan_sweep(a) -> Plan:
    """Constant-rate sweeps, one per rate, a still hold between. Built to ask
    whether ROTATION alone raises the TM151's tau: in the cruise the sensor
    turns with ~no acceleration (yaw at 40 deg/s, 12 mm off the axis: ~0.6 mg)."""
    segs = []
    for r in a.sweep_rates:
        segs += [sweep(a.sweep_axis, a.sweep_amp, r, a.sweep_accel, a.sweep_cycles), hold(5.0)]
    return _framed(a, segs, f"{a.sweep_axis} constant-rate sweeps +-{a.sweep_amp:g} deg at "
                            f"{a.sweep_rates} deg/s, reversals at {a.sweep_accel:g} deg/s^2")


def plan_sine(a) -> Plan:
    segs = []
    for amp in a.amps:
        for f in a.freqs:
            if amp * 2 * np.pi * f > a.max_rate:
                continue
            segs += [sine(a.axis, amp, f, max(4.0 / f, 6.0)), hold(1.5)]
    return _framed(a, segs, f"{a.axis} sines, amps {a.amps} deg x freqs {a.freqs} Hz, "
                            f"skipping peak rate > {a.max_rate:g} deg/s")


def plan_step(a) -> Plan:
    return _framed(a, step_segments(a),
                   f"{a.axis} steps to {a.step_amps} deg and back at "
                   f"{a.step_rate:g} deg/s peak, {a.step_hold:g} s holds")


def plan_chirp(a) -> Plan:
    return _framed(a, chirp_segments(a),
                   f"{a.axis} log sweeps {a.chirp_band[0]:g}->{a.chirp_band[1]:g} Hz "
                   f"in {a.chirp_seconds:g} s, amps {a.chirp_amps} deg, capped at "
                   f"{a.a_max:g} deg/s^2 and {a.max_rate:g} deg/s")


def plan_replay(a) -> Plan:
    return _framed(a, replay_segments(a), f"sim standing flights from {a.replay_file}")


def plan_sway(a) -> Plan:
    return _framed(a, sway_segments(a),
                   f"band-limited random: roll {a.roll_rms:g} deg RMS, yaw "
                   f"{a.yaw_rms:g}, {a.band[0]:g}-{a.band[1]:g} Hz, seed {a.seed}")


def plan_session(a) -> Plan:
    build = {"rest": lambda: [hold(a.rest_seconds, "rest")],
             "step": lambda: step_segments(a),
             "chirp": lambda: chirp_segments(a),
             "replay": lambda: replay_segments(a),
             "yaw": lambda: replay_segments(a, yaw_only=True),
             "sway": lambda: sway_segments(a)}
    body = [s for p in a.parts for s in build[p]()]
    return _framed(a, body, f"session: {' -> '.join(a.parts)}")


def _set_gains(p: int, i: int, d: int):
    def enter(bus):
        for dxl in IDS:
            bus.write_raw(dxl, "Position P Gain", p)
            bus.write_raw(dxl, "Position I Gain", i)
            bus.write_raw(dxl, "Position D Gain", d)
    return enter


def plan_tune(a) -> Plan:
    """The same stretch of one sim flight, once per gain setting.

    Servo-only: what is scored is how faithfully the joints reproduce the
    bike's motion, which needs no AHRS -- the encoder against the command
    (`track_score`). Each setting's gains go in on its segment's `enter`, and a
    held pause after it measures how still the servo sits at those gains.
    Re-run it once the bracket and AHRS are on: load changes the answer.
    """
    rep = load_replay(a.replay_file)
    rate = float(rep["rate"])
    k = (a.flights or [0])[0]
    s0, s1 = rep["start"][k], rep["start"][k + 1]
    yaw_hp, fc = fit_travel(rep["yaw_deg"][s0:s1].astype(float), rate, a.yaw_limit)
    n = min(int(a.tune_seconds * rate) + 1, s1 - s0)
    roll = np.clip(rep["roll_deg"][s0:s0 + n].astype(float), -SOFT_DEG, SOFT_DEG)
    segs = [home()]
    for kp in a.tune_kp:
        for ki in a.tune_ki:
            for kd in a.tune_kd:
                tag = f"p{kp}_i{ki}_d{kd}"
                seg = track(yaw_hp[:n], roll, rate, f"tune_{tag}", "tune",
                            kp=kp, ki=ki, kd=kd, flight=int(k), yaw_highpass_hz=fc)
                seg.enter = _set_gains(kp, ki, kd)
                segs += [seg, hold(1.5, f"still_{tag}")]
    n_set = len(a.tune_kp) * len(a.tune_ki) * len(a.tune_kd)
    return Plan(segs, ahrs=False,
                brief=f"{n_set} gain settings x {n / rate:.0f} s of sim flight {k}: "
                      f"P {a.tune_kp} x I {a.tune_ki} x D {a.tune_kd}")


PLANS = {"wiggle": plan_wiggle, "sweep": plan_sweep, "jog": plan_jog, "rest": plan_rest, "sine": plan_sine, "step": plan_step,
         "chirp": plan_chirp, "replay": plan_replay, "sway": plan_sway,
         "session": plan_session, "tune": plan_tune}


# --------------------------------------------------------------------------
# Bus: check, setup, run
# --------------------------------------------------------------------------

def open_bus(a):
    from aow_sim.hw.dynamixel import DynamixelBus

    bus = DynamixelBus(a.port, baud=a.baud, ids=IDS).open()
    wrong = {i: ct.name for i, ct in bus.tables.items() if ct.name != MODEL}
    if wrong:
        bus.close()
        raise SystemExit(f"this fixture is {MODEL} only; found {wrong}")
    return bus


def _limits_counts() -> tuple:
    half = int(round(LIMIT_DEG / 360.0 * COUNTS))
    return COUNTS // 2 - half, COUNTS // 2 + half


def cmd_check(a) -> int:
    from aow_sim.hw.dynamixel import describe_hardware_error

    bus = open_bus(a)
    lo, hi = _limits_counts()
    try:
        names = ("Firmware Version", "Operating Mode", "Min Position Limit",
                 "Max Position Limit", "Return Delay Time", "Position P Gain",
                 "Position I Gain", "Position D Gain", "Profile Velocity",
                 "Torque Enable", "Present Position", "Present Input Voltage",
                 "Present Temperature", "Hardware Error Status")
        snap = bus.snapshot(names)
        print(f"{'':24}" + "".join(f"{f'{i} ' + ('yaw' if i == YAW else 'roll'):>12}"
                                   for i in IDS))
        for n in names:
            print(f"{n:24}" + "".join(f"{snap[i][n]:>12}" for i in IDS))
        print(f"{'angle from centre [deg]':24}" + "".join(
            f"{(snap[i]['Present Position'] - COUNTS // 2) * 360 / COUNTS:>12.1f}"
            for i in IDS))
        ready = all(snap[i]["Operating Mode"] == MODE_POSITION
                    and snap[i]["Min Position Limit"] == lo
                    and snap[i]["Max Position Limit"] == hi for i in IDS)
        errs = bus.hardware_errors()
        if errs:
            print(f"LATCHED ERRORS: { {i: describe_hardware_error(e) for i, e in errs.items()} }")
        print("setup: " + ("done" if ready else
                           f"NOT DONE -- want mode {MODE_POSITION}, limits {lo}..{hi}; "
                           f"run `setup`"))
    finally:
        bus.close()
    log = AhrsLog(a.ahrs_port).start()
    try:
        log.wait()
        time.sleep(2.0)
    finally:
        log.stop()
    c = np.asarray(log.combo[1:], float)
    dt = np.diff(c[:, 1]) * 1e-6
    print(f"AHRS: {len(c)} Combo frames, device rate {1 / np.median(dt):.1f} Hz, "
          f"qos {int(c[-1, 15])}, rpy {np.round(c[-1, 12:15], 2)} deg, "
          f"|acc| {np.linalg.norm(c[:, 9:12], axis=1).mean():.4f} g, "
          f"other {log.other}, CRC failures {log.dec.n_crc_bad}")
    return 0


def cmd_setup(a) -> int:
    """EEPROM: Position Control Mode and the +-45 deg travel, torque off.

    EEPROM, so it survives a power cycle, and the servo's own firmware then
    refuses a goal outside the travel -- the limit lives where the cable is
    protected even if this script has a bug. Changing Operating Mode resets the
    PID gains to the mode's defaults; `run` writes its own every session.
    """
    bus = open_bus(a)
    lo, hi = _limits_counts()
    try:
        before = bus.snapshot(("Operating Mode", "Min Position Limit",
                               "Max Position Limit", "Drive Mode"))
        bus.torque(False)
        for i in IDS:
            bus.write_raw(i, "Operating Mode", MODE_POSITION)
            bus.write_raw(i, "Drive Mode", 0)
            bus.write_raw(i, "Min Position Limit", lo)
            bus.write_raw(i, "Max Position Limit", hi)
            bus.write_raw(i, "Return Delay Time", 0)
        after = bus.snapshot(("Operating Mode", "Min Position Limit",
                              "Max Position Limit", "Drive Mode"))
        for i in IDS:
            print(f"id {i}: {before[i]} -> {after[i]}")
    finally:
        bus.close()
    return 0


def cmd_capture(a) -> int:
    from aow_sim.hw.bench_log import git_state, new_capture_dir, record
    from aow_sim.hw.dynamixel import IndirectMap, describe_hardware_error

    plan = PLANS[a.test](a)             # before the bus: a bad plan costs nothing
    total = sum(s.seconds for s in plan.segments)
    lo, hi = _limits_counts()
    bus = open_bus(a)
    log = cap = None
    torque_on = False
    try:
        snap = bus.snapshot()
        bad = {i: (snap[i]["Operating Mode"], snap[i]["Min Position Limit"],
                   snap[i]["Max Position Limit"]) for i in IDS
               if (snap[i]["Operating Mode"], snap[i]["Min Position Limit"],
                   snap[i]["Max Position Limit"]) != (MODE_POSITION, lo, hi)}
        if bad:
            raise SystemExit(f"not set up (mode, min, max): {bad}; run `setup`")
        errs = bus.hardware_errors()
        if errs:
            raise SystemExit(f"latched hardware error "
                             f"{ {i: describe_hardware_error(e) for i, e in errs.items()} }")

        if plan.ahrs:
            log = AhrsLog(a.ahrs_port, raw_hz=a.raw_hz).start()
            log.wait()
            log.wait_qos(a.min_qos, a.qos_wait)

        bus.prepare()                         # torque off, Return Delay 0, profiles 0
        for i in IDS:
            bus.write_raw(i, "Position P Gain", a.kp)
            bus.write_raw(i, "Position I Gain", 0)
            bus.write_raw(i, "Position D Gain", a.kd)
        imap = IndirectMap(bus.tables)
        for name in READ_BLOCK:
            imap.read(name)
        imap.write("Goal Position", label="goal")
        bus.apply_map(imap, verify=True)
        before = bus.snapshot()

        v = {i: before[i]["Present Input Voltage"] * 0.1 for i in IDS}
        print(f"\n{a.test}: {len(plan.segments)} segments, {total / 60:.1f} min at "
              f"{a.rate:g} Hz; supply {v} V"
              + (f"; AHRS qos {log.qos()}" if log else ""))
        print(plan.brief)
        if plan.torque:
            if not a.yes:
                try:
                    input("\nTorque ON for 151 (yaw) and 152 (roll). Cables free, "
                          "fixture clamped, then Enter (Ctrl-C aborts): ")
                except EOFError:
                    raise SystemExit("no terminal to confirm on; pass --yes") from None
            here = bus.read_frame()
            bus.write_frame({i: here[i]["Present Position"] for i in IDS})
            bus.torque(True)
            torque_on = True
        cap = record(bus, imap, plan.segments, a.rate)
    finally:
        if torque_on:
            try:
                bus.torque(False)
            except Exception as e:           # noqa: BLE001
                print(f"!! TORQUE OFF FAILED ({e}) -- cut power to the servos")
        after = None
        try:
            after = bus.snapshot()
        except Exception:                    # noqa: BLE001
            pass
        bus.close()
        if log is not None:
            log.stop()

    if cap is None:
        return 1
    out = new_capture_dir(Path(a.out), a.test, a.tag)
    cap.meta.update(
        test=a.test, note=a.note, argv=sys.argv, port=a.port, ahrs_port=a.ahrs_port,
        fixture={"yaw_id": YAW, "roll_id": ROLL, "center_counts": COUNTS // 2,
                 "limit_deg": LIMIT_DEG, "soft_deg": SOFT_DEG,
                 "position_p": a.kp, "position_d": a.kd},
        git=git_state(ROOT), python=sys.version.split()[0],
        platform=platform.platform(), snapshot_before=before, snapshot_after=after,
        ahrs=None if log is None else {
            "frames": len(log.combo), "raw_frames": len(log.raw),
            "raw_hz": a.raw_hz, "other": log.other, "min_qos": a.min_qos})
    cap.save(out)
    if log is not None:
        np.savez_compressed(out / "ahrs.npz", **log.arrays(cap.meta["t0_perf_counter"]))
    st = cap.meta["stats"]
    print(f"\nsaved -> {out}\n  servo frames {st['frames']} "
          f"(dropped {st['dropped']}, overruns {st['overruns']})"
          + (f", AHRS frames {len(log.combo)}, raw {len(log.raw)}" if log else ""))
    lines = report_any(out)
    print("\n".join(lines))
    (out / "report.txt").write_text("\n".join(lines) + "\n")
    return 0 if cap.meta["complete"] else 1


def cmd_export_replay(a) -> int:
    """The sim's standing flights -> the small npz `replay` reads on the Pi.

    Source: `analysis/ahrs_fall_cause.py motion`, 64 seeds x 60 s of the
    default policy standing on estimate + TM151, logged at 100 Hz. The first
    2 s are dropped (the start transient) and, for a flight that fell, the last
    2 s (the fall itself, which leaves the fixture's travel). A flight left
    with under 20 s is skipped, so the earliest falls are not in the file.
    """
    import pickle

    M = pickle.load(open(a.src, "rb"))
    roll, yaw, start, seed, fell = [], [], [0], [], []
    rate = None
    for r in M:
        L = r["log"]
        dt = float(np.median(np.diff(L[:, 0])))
        rate = rate or 1.0 / dt
        k = int(round(2.0 / dt))
        L = L[k:(-k if r["tf"] is not None else None)]
        if len(L) < 10 * k:
            continue
        roll.append(np.degrees(L[:, 8]))                 # MOTION_COLS "roll"
        yaw.append(np.degrees(np.unwrap(L[:, 3])))       # MOTION_COLS "yaw"
        start.append(start[-1] + len(L))
        seed.append(int(r["seed"]))
        fell.append(r["tf"] is not None)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, roll_deg=np.concatenate(roll).astype(np.float32),
                        yaw_deg=np.concatenate(yaw).astype(np.float32),
                        start=np.array(start), seed=np.array(seed),
                        fell=np.array(fell), rate=np.array(round(rate, 6)),
                        source=np.array(str(a.src)))
    r_all = np.concatenate(roll)
    print(f"{len(seed)} flights ({sum(fell)} fell), {start[-1] / rate / 60:.1f} min at "
          f"{rate:g} Hz; roll RMS {np.std(r_all):.2f} deg -> {out}")
    return 0


# --------------------------------------------------------------------------
# Analysis: numpy only, so it runs on the Pi straight after a capture
# --------------------------------------------------------------------------

def _skew(v):
    """(..., 3) -> (..., 3, 3)."""
    z = np.zeros(v.shape[:-1])
    return np.stack([np.stack([z, -v[..., 2], v[..., 1]], -1),
                     np.stack([v[..., 2], z, -v[..., 0]], -1),
                     np.stack([-v[..., 1], v[..., 0], z], -1)], -2)


def axis_angle(u, theta):
    """Rotation matrices about unit axis `u` by each angle in `theta` -> (N, 3, 3)."""
    th = np.asarray(theta, float)[:, None, None]
    K = _skew(np.asarray(u, float))[None]
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def quats_to_mats(q):
    """(N, 4) w,x,y,z -> (N, 3, 3), sensor -> earth (the TM151's convention)."""
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def mats_to_rotvec(R):
    """(N, 3, 3) -> (N, 3) rotation vectors. Fine for any angle short of pi."""
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1)
    th = np.arccos(c)
    v = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0],
                  R[:, 1, 0] - R[:, 0, 1]], -1) / 2
    s = np.sin(th)
    k = np.where(s > 1e-9, th / np.where(s > 1e-9, s, 1), 1.0)
    return v * k[:, None]


def mean_quat(q):
    q = q * np.sign(q @ q[0])[:, None]
    m = q.mean(0)
    return m / np.linalg.norm(m)


def truth_mats(R0, u_y, u_r, dpsi, dphi):
    """R_true = R0 . Rot(u_y, dpsi) . Rot(u_r, dphi), all body-frame.

    For R = Rb Rz(psi) L Rx(phi) M (base, yaw joint, bracket, roll joint,
    mount), R0^-1 R factors into [M' Rx(-phi0) L' Rz(dpsi) L Rx(phi0) M] .
    [M' Rx(dphi) M], which are rotations about u_y = M' Rx(-phi0) L' z and
    u_r = M' x -- the two joint axes as the sensor sees them at the reference
    pose. So the fitted axes are all the geometry there is.
    """
    return R0[None] @ axis_angle(u_y, dpsi) @ axis_angle(u_r, dphi)


def tilt_errors(R_a, R_t):
    """AHRS vs truth -> (N, 3) degrees: roll, pitch about the sensor's heading
    frame (x's horizontal projection), and heading."""
    e_w = mats_to_rotvec(R_a @ np.transpose(R_t, (0, 2, 1)))   # earth frame
    h = np.arctan2(R_t[:, 1, 0], R_t[:, 0, 0])
    c, s = np.cos(h), np.sin(h)
    return np.degrees(np.stack([c * e_w[:, 0] + s * e_w[:, 1],
                                -s * e_w[:, 0] + c * e_w[:, 1], e_w[:, 2]], 1))


def gravity_sensor(R_t):
    """What the TM151's accelerometer reads at rest, in g: the earth's -z in
    the sensor frame. (Face up it reads (0, 0, -1) -- datasheet footnote [1] --
    i.e. the NEGATIVE of the specific force.)"""
    return -R_t[:, 2, :]


def nongravity_accel(R_t, acc_g):
    """Acceleration of the sensing point [m/s^2, sensor frame] from the
    accelerometer and the TRUE attitude. Not the AHRS's own attitude: a 1 deg
    tilt error is 0.17 m/s^2 of fake acceleration."""
    return G_VENDOR * (gravity_sensor(R_t) - acc_g)


def fit_lever_arm(R_t, acc_g, gyro, t, mask):
    """Least squares a = (skew(alpha) + skew(w)^2) r + b over `mask`.

    r: sensing point from the joint axes' intersection, sensor frame [m].
    b: accelerometer bias [m/s^2]. omega from the gyro, alpha its derivative:
    both far cleaner than the encoders differentiated once or twice.
    """
    a = nongravity_accel(R_t, acc_g)
    w = _ma(gyro, 5)
    al = np.gradient(_ma(gyro, 9), t, axis=0)
    K = _skew(al) + _skew(w) @ _skew(w)
    K, a = K[mask], a[mask]
    A = np.concatenate([K, np.broadcast_to(np.eye(3), K.shape)], 2).reshape(-1, 6)
    x, *_ = np.linalg.lstsq(A, a.reshape(-1), rcond=None)
    resid = (a.reshape(-1) - A @ x).reshape(-1, 3)
    sv = np.linalg.svd(A, compute_uv=False)
    return {"r_m": x[:3], "bias_ms2": x[3:],
            "resid_mg": 1e3 * np.sqrt((resid ** 2).mean(0)) / G_VENDOR,
            "condition": float(sv[0] / sv[-1])}


def _unwrap_us(t_us):
    d = np.diff(np.asarray(t_us, np.int64)) % (1 << 32)
    return np.concatenate([[0], np.cumsum(d)]) * 1e-6


def ahrs_clock(t_host, t_us):
    """Device clock -> host clock, along the FASTEST arrivals.

    Returns (t_sample_on_host, keep, info). A line fitted to (device, host)
    then shifted down to its 1st-percentile residual: arrivals only ever come
    late, never early, so the lower envelope is the transport's floor and the
    spread above it is jitter. The stale first frame (previous session) and any
    other clock outlier are dropped by `keep`.
    """
    t_dev = _unwrap_us(t_us)
    # Outliers against the FITTED LINE, not the median offset: the device
    # clock runs 0.3% fast, so over a 6 min session the raw offset itself
    # walks 1.1 s. A median-offset gate once cut both ends off a session,
    # reference hold included.
    keep = np.ones(len(t_dev), bool)
    keep[0] = False                          # always the previous session's
    for _ in range(3):
        b, a = np.polyfit(t_dev[keep], t_host[keep], 1)
        r = t_host - (a + b * t_dev)
        keep = np.abs(r - np.median(r[keep])) < 0.05
        keep[0] = False
    b, a = np.polyfit(t_dev[keep], t_host[keep], 1)
    r = t_host - (a + b * t_dev)
    floor = np.percentile(r[keep], 1)
    t_s = a + floor + b * t_dev
    jit = (t_host - t_s)[keep]
    return t_s, keep, {"drift_ppm": (b - 1) * 1e6,
                       "arrival_jitter_ms": {"p50": 1e3 * np.percentile(jit, 50),
                                             "p99": 1e3 * np.percentile(jit, 99)}}


def _ma(x, w):
    """Centred moving average over `w` samples along axis 0, edges shortened."""
    if w <= 1:
        return x
    k = np.ones(w) / w
    if x.ndim == 1:
        return np.convolve(x, k, "same") / np.convolve(np.ones(len(x)), k, "same")
    return np.stack([_ma(x[:, j], w) for j in range(x.shape[1])], 1)


def _in(t, span, trim=0.0):
    return (t >= span[0] + trim) & (t <= span[1] - trim)


def _rms(x):
    return np.sqrt((np.asarray(x) ** 2).mean(0))


def acf_tau(x, dt):
    """1/e crossing of the autocorrelation [s], or nan."""
    x = x - x.mean()
    n = len(x)
    f = np.fft.rfft(x, 2 * n)
    acf = np.fft.irfft(f * np.conj(f))[:n]
    acf /= acf[0]
    below = np.flatnonzero(acf < np.exp(-1))
    if not len(below):
        return float("nan")
    k = below[0]
    y0, y1 = acf[k - 1], acf[k]
    return float(dt * (k - 1 + (y0 - np.exp(-1)) / (y0 - y1)))


def joint_gain(x, err_along, omega):
    """The AHRS's gain and phase about a joint: fit what it reports,
    x + err_along, as g_i x + g_q (dx/dt)/omega -- in phase and in quadrature
    with the true motion x. -> (gain, phase_deg), phase positive = LEAD.

    This is what an RMS cannot show. Below the roll centre the sensor is a
    pendulum and its accelerometer under-reads the lean (to nothing at
    sqrt(g/h)); above it, an inverted pendulum, it over-reads. Equal RMS,
    opposite sign -- and to a controller, gain < 1 and gain > 1 are different
    feedback gains. `x` must be sampled uniformly enough for a gradient.
    """
    xd = np.gradient(_ma(x, 5))            # per sample, so omega is rad/sample
    A = np.stack([x, xd / omega], 1)
    (gi, gq), *_ = np.linalg.lstsq(A, x + err_along, rcond=None)
    return float(np.hypot(gi, gq)), float(np.degrees(np.arctan2(gq, gi)))


def chirp_bands(info, tr, err, a_perp, motion, err_along=None):
    """Per-octave rows for one chirp: the error's RMS where the sweep was in
    that octave, and -- given `err_along`, the error about the swept joint's
    own axis -- the AHRS's gain and phase there (`joint_gain`). `tr` is time
    since the chirp began; `err_along` is deg, `motion` deg from reference."""
    f0, f1, T = info["f0"], info["f1"], info["seconds"]
    k = f1 / f0
    f = f0 * k ** (tr / T)
    dt = float(np.median(np.diff(tr)))
    rows = []
    lo = f0
    while lo < f1 * 0.999:
        hi = min(lo * 2, f1)
        m = (f >= lo) & (f < hi) & (tr > 1.0) & (tr < T - 1.0)
        if m.sum() >= 20:
            row = {"f_lo": lo, "f_hi": hi, "rms": _rms(err[m]),
                   "a_perp": float(_rms(a_perp[m])),
                   "motion_amp_deg": float(np.sqrt(2) * np.std(motion[m]))}
            if err_along is not None:
                idx = np.flatnonzero(m)
                row["gain"], row["phase_deg"] = joint_gain(
                    motion[idx] - motion[idx].mean(), err_along[idx],
                    2 * np.pi * f[idx] * dt)
            rows.append(row)
        lo = hi
    return rows


def step_response(info, tr, err):
    """Final error at the new pose, the peak departure from it, and the time
    after the move until it stays within SETTLE_DEG."""
    T = float(tr[-1])
    final_m = tr > max(T - 5.0, info["move_s"] + (T - info["move_s"]) / 2)
    final = err[final_m, :2].mean(0)
    dev = np.hypot(*(err[:, :2] - final).T)
    after = tr >= info["move_s"]
    out_idx = np.flatnonzero(after & (dev > SETTLE_DEG))
    settle = float(tr[out_idx[-1]] - info["move_s"]) if len(out_idx) else 0.0
    return {"final_deg": final, "peak_dev_deg": float(dev[after].max()),
            "settle_s": settle}


# --------------------------------------------------------------------------
# The encoder-free reference: the TM151's own gyro, pinned by its accelerometer
# --------------------------------------------------------------------------
#
# The encoders sit on the servo OUTPUT SHAFTS. Between a shaft and the sensor
# are the horn, its screws and a printed bracket, and on the first mounted
# captures (2026-09-23) that chain lost 0.13-0.54 deg in the step holds and
# up to ~2 deg under a hand wiggle -- as large as the error being measured.
# So the headline number does not use them. In a still hold the accelerometer
# reads gravity and nothing else, which is the plate's true tilt; between two
# holds the raw gyro, integrated, carries it. Pinning the integration to the
# accelerometer at BOTH ends removes its drift. What is left measures the
# fusion filter, not the mechanism. It cannot see an error the gyro and the
# fusion share -- the step holds check the gyro's scale against the
# accelerometer for exactly that reason.

STILL = ("ref", "settle", "hold", "rest", "ref_end", "still")
SELFREF_PRE_S = 1.0             # hold before a segment the start is read from
SELFREF_POST_S = (0.5, 1.5)     # window in the hold after it


def _is_still(label: str) -> bool:
    return label.startswith(STILL)


def _turn(v, w, dt):
    """A world-fixed vector in a frame turning at w [rad/s] for dt."""
    th = np.linalg.norm(w) * dt
    if th < 1e-12:
        return v
    k = -w / np.linalg.norm(w)
    return v * np.cos(th) + np.cross(k, v) * np.sin(th) + k * (k @ v) * (1 - np.cos(th))


def _integrate_down(t, w, v0):
    v = np.empty((len(t), 3))
    v[0] = v0
    for i in range(1, len(t)):
        v[i] = _turn(v[i - 1], 0.5 * (w[i] + w[i - 1]), t[i] - t[i - 1])
    return v


def _about(axis, a, b):
    """Signed angle [deg] turning a onto b about `axis` (projected)."""
    pa = a - (a @ axis)[..., None] * axis if a.ndim > 1 else a - (a @ axis) * axis
    pb = b - (b @ axis)[..., None] * axis
    return np.degrees(np.arctan2((np.cross(pa, pb) @ axis), (pb * pa).sum(-1)))


def self_reference(t, gyro, acc, R_a, segments, u_r=None) -> dict:
    """The fused tilt against the gyro+accelerometer reference, per motion
    segment that has a still hold either side. `t` on the device clock (any
    linear map of it), gyro in rad/s, acc in the vendor's g, R_a the fused
    attitude. u_r: the roll axis in the sensor frame, for the error ABOUT it
    and the gyro's roll rate; without it only the total tilt is reported."""
    # Bias from EVERY still hold pooled: 1 s of hold leaves ~0.009 deg/s of
    # noise in the mean, which a 60 s segment integrates to 0.5 deg.
    kb = np.zeros(len(t), bool)
    for s in segments:
        if _is_still(s["label"]):
            kb |= _in(t, (s["t0"], s["t1"]), trim=0.5)
    if kb.sum() < 200 or acc is None:
        return {}
    bias = gyro[kb].mean(0)
    down_f = gravity_sensor(R_a)
    out = {"bias_from_s": float(kb.sum() * np.median(np.diff(t))),
           "bias_dps": np.degrees(bias),
           "segments": [], "steps": [], "series": {}}

    def span(s, i):
        pre = (s["t0"] - SELFREF_PRE_S, s["t0"])
        post = (s["t1"] + SELFREF_POST_S[0], s["t1"] + SELFREF_POST_S[1])
        return pre, post

    def run(pre, end, post):
        k = (t >= pre[0]) & (t <= post[1])
        kp, kq = _in(t, pre), _in(t, post)
        if kp.sum() < 50 or kq.sum() < 50:
            return None
        tt = t[k]
        a0 = acc[kp].mean(0); a0 /= np.linalg.norm(a0)
        a1 = acc[kq].mean(0); a1 /= np.linalg.norm(a1)
        v = _integrate_down(tt, gyro[k] - bias, a0)
        ve = v[tt >= post[0]].mean(0); ve /= np.linalg.norm(ve)
        e = np.cross(ve, a1)
        ang = float(np.arcsin(min(np.linalg.norm(e), 1.0)))
        raw = v.copy()
        if ang > 0:                  # pin the end, the correction growing linearly
            ax = e / np.linalg.norm(e)
            f = np.clip((tt - pre[1]) / (post[0] - pre[1]), 0, 1)
            for i in np.flatnonzero(f > 0):
                v[i] = _turn(v[i], -ax * f[i] * ang, 1.0)
        return k, tt, v, raw, a0, a1, float(np.degrees(ang))

    for i, s in enumerate(segments):
        before = segments[i - 1]["label"] if i else ""
        after = segments[i + 1]["label"] if i + 1 < len(segments) else ""
        if s["label"].startswith("step_") and u_r is not None:
            # the step segment ends in its own long hold: accelerometer vs gyro
            k_end = (s["t1"] - 5.0, s["t1"])
            r = run((s["t0"] - SELFREF_PRE_S, s["t0"]), None, k_end)
            if r is None:
                continue
            k, tt, v, raw, a0, a1, _ = r
            ke = tt >= k_end[0]
            rg = raw[ke].mean(0)
            out["steps"].append({"label": s["label"],
                                 "accel_deg": float(_about(u_r, a0, a1[None])[0]),
                                 "gyro_deg": float(_about(u_r, a0, rg[None])[0])})
            continue
        if _is_still(s["label"]) or s["label"] == "home" or not _is_still(before) \
                or not _is_still(after):
            continue
        pre, post = span(s, i)
        r = run(pre, None, post)
        if r is None:
            continue
        k, tt, v, _, a0, a1, mis = r
        f = down_f[k]
        m = (tt >= s["t0"]) & (tt <= s["t1"])
        # tilt error: the rotation taking the reference's down onto the fused
        # one, less what it was in the hold before (a fixed offset between the
        # filter and the raw accelerometer is not a dynamic error)
        ev = np.cross(v, f)
        pre_m = tt <= pre[1]
        ev = ev - ev[pre_m].mean(0)
        row = {"label": s["label"], "seconds": s["t1"] - s["t0"],
               "tilt_rms_deg": float(np.degrees(np.sqrt((ev[m] ** 2).sum(1).mean()))),
               "end_mismatch_deg": mis}
        if u_r is not None:
            er = np.degrees(ev @ u_r)
            rate = np.degrees((gyro[k] - bias) @ u_r)
            X = np.c_[rate[m], np.ones(m.sum())]
            c, *_ = np.linalg.lstsq(X, er[m], rcond=None)
            res = er[m] - X @ c
            dt = float(np.median(np.diff(tt)))
            row.update({"roll_rms_deg": float(_rms(er[m])),
                        "roll_p99_deg": float(np.percentile(np.abs(er[m]), 99)),
                        "roll_rate_rms_dps": float(_rms(rate[m])),
                        # a pure delay makes error = lag * rate in this sign
                        # (the synthetic rig's 20 ms comes back +20)
                        "lag_ms": float(1e3 * c[0]),
                        "after_lag_rms_deg": float(_rms(res)),
                        "tau_s": acf_tau(er[m], dt)})
            out["series"][s["label"]] = {
                "t": tt[m] - s["t0"],
                "ref_roll_deg": _about(u_r, a0, v[m]),
                "fused_roll_deg": _about(u_r, f[pre_m].mean(0) / np.linalg.norm(
                    f[pre_m].mean(0)), f[m])}
        out["segments"].append(row)
    st = [x for x in out["steps"] if abs(x["accel_deg"]) > 1]
    if st:
        a = np.array([x["accel_deg"] for x in st]); g_ = np.array([x["gyro_deg"] for x in st])
        out["gyro_scale_roll"] = float((a * g_).sum() / (a * a).sum())
    return out


def repeat_difference(sa: dict, sb: dict) -> list:
    """Two runs of the same seeded input, per segment: what does NOT repeat.
    Errors the mechanism or the filter make deterministically cancel; the
    reference's own difference is how far the PLATE itself moved differently."""
    rows = []
    for lab in sa:
        if lab not in sb:
            continue
        a, b = sa[lab], sb[lab]
        g = np.arange(0.0, min(a["t"][-1], b["t"][-1]), 0.0005)
        ra = np.interp(g, a["t"], a["ref_roll_deg"])
        rb = np.interp(g, b["t"], b["ref_roll_deg"])
        n = 250
        lags = np.arange(-200, 201)
        cc = [ra[n:-n] @ np.roll(rb, L)[n:-n] for L in lags]
        sh = lags[int(np.argmax(cc))] * 0.0005
        fa = np.interp(g, a["t"], a["fused_roll_deg"])
        fb = np.interp(g, b["t"] + sh, b["fused_roll_deg"])
        rb = np.interp(g, b["t"] + sh, b["ref_roll_deg"])
        sl = slice(400, -400)
        rows.append({"label": lab, "shift_ms": 1e3 * sh,
                     "fused_rms_deg": float(_rms((fa - fb)[sl]) / np.sqrt(2)),
                     "ref_rms_deg": float(_rms((ra - rb)[sl]) / np.sqrt(2))})
    return rows


# --------------------------------------------------------------------------
# The TM151 as a complementary filter
# --------------------------------------------------------------------------
#
# Measured 2026-09-23. AT REST the fused tilt is the accelerometer's tilt
# low-passed at tau ~0.19 s plus the integrated gyro high-passed at the same
# tau: r 0.92 / 0.93 in roll / pitch over 10 min torque-off, no magnetometer
# needed. IN MOTION the same filter fits best at tau 0.5-1 s -- the sensor
# trusts its accelerometer less while moving -- and at tau 0.7, with the
# FITTED lever arm and no free gain, it predicts the fused error's lever-arm
# part at k 0.9-1.06: most of the dynamic error on the d = 30 mount is the
# plate's own acceleration read as tilt. The rule that switches tau is unknown.

TAU_GRID_S = (0.1, 0.15, 0.19, 0.25, 0.35, 0.5, 0.7, 1.0, 2.0)


def comp_filter(t, gyro, acc, v0, tau, bias=0.0):
    """Down-vector (the vendor accelerometer's sign) through a first-order
    complementary filter: propagate with the gyro [rad/s], pull toward the
    normalised accelerometer at 1/tau."""
    an = acc / np.linalg.norm(acc, axis=1)[:, None]
    w = gyro - bias
    v = np.empty((len(t), 3))
    v[0] = v0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        p = _turn(v[i - 1], 0.5 * (w[i] + w[i - 1]), dt)
        p = p + (dt / tau) * (an[i] - p)
        v[i] = p / np.linalg.norm(p)
    return v


TAU_WINDOW_S = 2.0
TAU_WINDOW_GRID = np.array([0.1, 0.19, 0.3, 0.5, 0.7, 1.0, 2.0, 5.0])


def tau_windows(t, gyro, acc, fused, bias, win=TAU_WINDOW_S, taus=TAU_WINDOW_GRID):
    """tau fitted per window, each started from the fused state, beside what
    the sensor itself could see there: mean rotation rate and mean deviation of
    |acc| from 1 g. Measured 2026-09-23: a CONTINUUM -- 0.19 s below ~2 mg and
    ~1 deg/s, ~1 s past ~10 mg or ~15 deg/s. On this rig rotation and
    acceleration come together, so which one drives it is open."""
    an = np.linalg.norm(acc, axis=1)
    an = an / np.median(an)             # the sensor's own 1 g: its bias is a few mg
    rows = []
    for t0 in np.arange(t[0] + 1.0, t[-1] - win, win):
        k = (t >= t0) & (t < t0 + win)
        if k.sum() < 0.75 * win * 200:
            continue
        i0 = np.flatnonzero(k)[0]
        cost = np.array([np.degrees(np.sqrt((np.cross(
            comp_filter(t[k], gyro[k], acc[k], fused[i0], tau, bias), fused[k]) ** 2
        ).sum(1).mean())) for tau in taus])
        j = int(np.argmin(cost))
        tau = float(taus[j])
        if 0 < j < len(taus) - 1:            # parabola in log tau through the best three
            x, y = np.log(taus[j - 1:j + 2]), cost[j - 1:j + 2]
            p = np.polyfit(x, y, 2)
            if p[0] > 0:
                tau = float(np.exp(np.clip(-p[1] / (2 * p[0]), x[0], x[2])))
        rows.append({"t0": float(t0), "tau_s": tau, "contrast": float(cost.max() / max(cost.min(), 1e-9)),
                     "rate_dps": float(np.degrees(np.linalg.norm(gyro[k] - bias, axis=1)).mean()),
                     "acc_dev_mg": float(1e3 * np.abs(an[k] - 1.0).mean())})
    return rows


def _lowpass1(x, dt, tau):
    a = dt / (tau + dt)
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def _tilt_dev(v, m):
    """Small tilt deviations of down-vectors v from their mean m [rad], on two
    axes perpendicular to m: minus the body rotation that produced them."""
    e1 = np.cross(m, [1.0, 0, 0] if abs(m[0]) < 0.9 else [0, 1.0, 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(m, e1)
    c = np.cross(m, v)
    return np.c_[c @ e1, c @ e2], (e1, e2)


def rest_noise_model(t, gyro, acc, R_a, span, taus=TAU_GRID_S) -> dict:
    """The resting fused tilt against accelerometer-through-a-low-pass plus
    integrated-gyro-through-the-matching-high-pass, per tilt axis."""
    k = _in(t, span, trim=1.0)
    if k.sum() < 2000:
        return {}
    t, g, a, f = t[k], gyro[k], acc[k], gravity_sensor(R_a[k])
    dt = float(np.median(np.diff(t)))
    m = f.mean(0) / np.linalg.norm(f.mean(0))
    F, (e1, e2) = _tilt_dev(f, m)
    A, _ = _tilt_dev(a / np.linalg.norm(a, axis=1)[:, None], m)
    th = -np.cumsum((g - g.mean(0)) * dt, axis=0)       # the down-vector turns the other way
    Gi = np.c_[th @ e1, th @ e2]
    rows = []
    for tau in taus:
        lp = np.c_[[_lowpass1(A[:, j], dt, tau) for j in (0, 1)]].T
        hp = Gi - np.c_[[_lowpass1(Gi[:, j], dt, tau) for j in (0, 1)]].T
        s = slice(int(5 * tau / dt) + 1, None)           # the filters' start-up
        rows.append((tau, [float(np.corrcoef(F[s, j], lp[s, j] + hp[s, j])[0, 1])
                           for j in (0, 1)],
                     [float(np.corrcoef(F[s, j], lp[s, j])[0, 1]) for j in (0, 1)]))
    best = max(rows, key=lambda r: sum(r[1]))
    return {"seconds": float(t[-1] - t[0]), "fused_std_deg": np.degrees(F.std(0)),
            "accel_std_deg": np.degrees(A.std(0)), "tau_s": best[0],
            "r": best[1], "r_accel_only": best[2],
            "grid": [(r[0], r[1]) for r in rows]}


def analyse(servo: dict, ahrs: dict, segments: list) -> dict:
    """The whole analysis on plain arrays, so a synthetic rig can test it.

    `servo`: t [s], yaw, roll [rad from centre], all per servo frame.
    `ahrs`:  t_host [s] arrival, t_us device clock, quat (N,4), gyro (N,3),
             optionally acc_g (N,3) in the vendor's g.
    `segments`: [{"label", "t0", "t1", "info"?}] on the same timebase.
    """
    t_e, psi, phi = servo["t"], servo["yaw"], servo["roll"]
    t_s, keep, clock = ahrs_clock(ahrs["t_host"], ahrs["t_us"])
    t_arr = ahrs["t_host"][keep]
    t_s, q, g = t_s[keep], ahrs["quat"][keep], ahrs["gyro"][keep]
    acc = ahrs["acc_g"][keep] if "acc_g" in ahrs else None
    R_a = quats_to_mats(q)
    spans = {}
    for s in segments:
        spans.setdefault(s["label"], (s["t0"], s["t1"]))
    out = {"clock": clock, "segments": []}

    ref = spans.get("ref")
    if ref is None:
        raise ValueError("no 'ref' segment: nothing to reference the attitude to")
    m_ref = _in(t_s, ref, trim=0.5)
    R0 = quats_to_mats(mean_quat(q[m_ref])[None])[0]
    e_ref = _in(t_e, ref, trim=0.5)
    psi0, phi0 = psi[e_ref].mean(), phi[e_ref].mean()

    # Encoder rates, smoothed the same as the gyro, for the ident fit.
    w = 5
    dpsi_dt = np.gradient(_ma(psi, w), t_e)
    dphi_dt = np.gradient(_ma(phi, w), t_e)
    g_s = _ma(g, w)

    u_y = u_r = None
    lag = 0.0
    if "ident_yaw" in spans and "ident_roll" in spans:
        m = _in(t_s, spans["ident_yaw"], 0.3) | _in(t_s, spans["ident_roll"], 0.3)
        lags = np.arange(-0.020, 0.080, 0.0005)
        cost = []
        for d in lags:
            A = np.stack([np.interp(t_s[m] - d, t_e, dpsi_dt),
                          np.interp(t_s[m] - d, t_e, dphi_dt)], 1)
            V, *_ = np.linalg.lstsq(A, g_s[m], rcond=None)
            cost.append(float(np.sum((g_s[m] - A @ V) ** 2)))
        k = int(np.argmin(cost))
        lag = float(lags[k])
        if 0 < k < len(lags) - 1:            # parabolic refinement
            c0, c1, c2 = cost[k - 1], cost[k], cost[k + 1]
            den = c0 - 2 * c1 + c2
            if den > 0:
                lag += 0.5 * (c0 - c2) / den * (lags[1] - lags[0])
        A = np.stack([np.interp(t_s[m] - lag, t_e, dpsi_dt),
                      np.interp(t_s[m] - lag, t_e, dphi_dt)], 1)
        V, *_ = np.linalg.lstsq(A, g_s[m], rcond=None)
        v_y, v_r = V
        u_y, u_r = v_y / np.linalg.norm(v_y), v_r / np.linalg.norm(v_r)
        resid = g_s[m] - A @ V
        out["ident"] = {
            "lag_ms": 1e3 * lag,
            "u_yaw": u_y, "u_roll": u_r,
            "scale_yaw": float(np.linalg.norm(v_y)),
            "scale_roll": float(np.linalg.norm(v_r)),
            "axis_angle_deg": float(np.degrees(np.arccos(np.clip(u_y @ u_r, -1, 1)))),
            "resid_dps": np.degrees(np.sqrt((resid ** 2).mean(0))),
            "lag_at_edge": k in (0, len(lags) - 1),
        }

    def truth_at(tt):
        dpsi = np.interp(tt, t_e, psi) - psi0
        dphi = np.interp(tt, t_e, phi) - phi0
        if u_y is None:
            return np.broadcast_to(R0, (len(tt), 3, 3)).copy(), dpsi, dphi
        return truth_mats(R0, u_y, u_r, dpsi, dphi), dpsi, dphi

    R_lag, dpsi_l, dphi_l = truth_at(t_s - lag)
    R_rx, *_ = truth_at(t_arr)
    err_lag = tilt_errors(R_a, R_lag)
    err_rx = tilt_errors(R_a, R_rx)
    dpsi_d, dphi_d = np.degrees(dpsi_l), np.degrees(dphi_l)
    roll_rate = np.degrees(np.interp(t_s - lag, t_e, dphi_dt))

    # The error about the ROLL JOINT'S own axis, for the chirp gain/phase: the
    # error rotation projected on that axis's horizontal part in the heading
    # frame, divided by that part's squared length so a joint rotation of d
    # reported as (1 + k) d reads k d whatever the axis's small tilt.
    err_roll_axis = None
    if u_r is not None:
        ax = R_lag @ u_r
        hh = np.arctan2(R_lag[:, 1, 0], R_lag[:, 0, 0])
        c, s_ = np.cos(hh), np.sin(hh)
        d = np.stack([c * ax[:, 0] + s_ * ax[:, 1], -s_ * ax[:, 0] + c * ax[:, 1]], 1)
        err_roll_axis = (err_lag[:, :2] * d).sum(1) / (d * d).sum(1)

    a_perp = np.zeros(len(t_s))
    if acc is not None:
        a_ng = nongravity_accel(R_lag, acc)
        gh = gravity_sensor(R_lag)
        a_perp = np.linalg.norm(a_ng - (a_ng * gh).sum(1)[:, None] * gh, axis=1)
        riding = u_y is not None and min(out["ident"]["scale_yaw"],
                                         out["ident"]["scale_roll"]) > 0.5
        if riding:                  # off the joints, r would be fitted to nothing
            inseg = np.zeros(len(t_s), bool)
            for s in segments:
                inseg |= _in(t_s, (s["t0"], s["t1"]), 0.25)
            lv = fit_lever_arm(R_lag, acc, g, t_s, inseg)
            r = lv["r_m"]
            lv["from_roll_axis_mm"] = 1e3 * float(np.linalg.norm(r - (r @ u_r) * u_r))
            lv["from_yaw_axis_mm"] = 1e3 * float(np.linalg.norm(r - (r @ u_y) * u_y))
            out["lever"] = lv

    if u_y is None:
        moved = max(np.ptp(np.degrees(psi)), np.ptp(np.degrees(phi)))
        out["warning"] = (None if moved < 0.5 else
                          f"joints moved {moved:.1f} deg with no ident segments: "
                          f"the truth assumes the pose never changed")
    dt = float(np.median(np.diff(t_s)))
    for s in segments:
        mm = _in(t_s, (s["t0"], s["t1"]), trim=0.25)
        if mm.sum() < 20:
            continue
        info = s.get("info") or {}
        el, er = err_lag[mm], err_rx[mm]
        row = {"label": s["label"], "kind": info.get("kind"),
               "seconds": s["t1"] - s["t0"], "n": int(mm.sum()),
               "motion_rms_deg": [float(np.std(dpsi_d[mm])), float(np.std(dphi_d[mm]))],
               "roll_rate_rms_dps": float(_rms(roll_rate[mm])),
               "a_perp_rms": float(_rms(a_perp[mm])),
               "rms_lag": _rms(el), "rms_rx": _rms(er), "mean_lag": el.mean(0),
               "max_tilt_lag": float(np.max(np.hypot(el[:, 0], el[:, 1])))}
        if "target_roll_rate_rms_dps" in info:
            row["target_roll_rate_rms_dps"] = info["target_roll_rate_rms_dps"]
        if s["t1"] - s["t0"] >= 30:
            row["tau_s"] = [acf_tau(el[:, j], dt) for j in range(3)]
        mseg = (t_s >= s["t0"]) & (t_s <= s["t1"])
        tr = t_s[mseg] - lag - s["t0"]
        if info.get("kind") == "chirp":
            roll = info.get("axis") == "roll"
            motion = dphi_d if roll else dpsi_d
            along = err_roll_axis[mseg] if roll and err_roll_axis is not None else None
            row["bands"] = chirp_bands({**info, "seconds": s["t1"] - s["t0"]}, tr,
                                       err_lag[mseg], a_perp[mseg], motion[mseg], along)
        if info.get("kind") == "step":
            row["step"] = step_response(info, tr, err_lag[mseg])
        out["segments"].append(row)
    if acc is not None:
        out["selfref"] = self_reference(t_s, g, acc, R_a, segments, u_r)
        still = [s for s in segments if s["label"] in ("rest", "ref")]
        if still:
            longest = max(still, key=lambda s: s["t1"] - s["t0"])
            rm = rest_noise_model(t_s, g, acc, R_a, (longest["t0"], longest["t1"]))
            if rm:
                out["rest_model"] = {**rm, "segment": longest["label"]}
    out["series"] = {"t": t_s, "err_lag_deg": err_lag, "err_rx_deg": err_rx,
                     "yaw_deg": dpsi_d, "roll_deg": dphi_d, "a_perp": a_perp}
    return out


def analyse_dir(d) -> dict:
    from aow_sim.hw.bench_log import Capture

    d = Path(d)
    cap = Capture.load(d)
    with np.load(d / "ahrs.npz") as z:
        ahrs = {k: z[k] for k in z.files}
    ok = cap.ok
    t = (cap.arrays["t_host"] + cap.arrays["read_s"] / 2)[ok]
    servo = {"t": t, "yaw": cap.position_rad(YAW)[ok] - CENTER,
             "roll": cap.position_rad(ROLL)[ok] - CENTER}
    th = cap.arrays["t_host"]
    segs = [{"label": s["label"], "t0": float(th[s["start"]]),
             "t1": float(th[max(s["stop"] - 1, s["start"])]), "info": s.get("info", {})}
            for s in cap.segments if s["stop"] > s["start"]]
    res = analyse(servo, ahrs, segs)
    res["dir"] = str(d)
    res["meta"] = {"test": cap.meta.get("test"), "note": cap.meta.get("note"),
                   "stats": cap.meta.get("stats")}
    return res


def report(res: dict) -> list:
    L = []
    f3 = lambda v, p=3: "[" + " ".join(f"{x:+.{p}f}" for x in v) + "]"   # noqa: E731
    if res.get("meta", {}).get("note"):
        L.append(f"note: {res['meta']['note']}")
    c = res["clock"]
    L.append(f"AHRS clock: drift {c['drift_ppm']:+.0f} ppm, arrival jitter above the "
             f"floor p50 {c['arrival_jitter_ms']['p50']:.2f} / "
             f"p99 {c['arrival_jitter_ms']['p99']:.2f} ms")
    if "ident" in res:
        i = res["ident"]
        L.append(f"ident: AHRS delay {i['lag_ms']:.1f} ms"
                 + ("  (AT THE EDGE OF THE SEARCH -- suspect)" if i["lag_at_edge"] else ""))
        L.append(f"  yaw axis  in sensor frame {f3(i['u_yaw'])}  gyro scale {i['scale_yaw']:.4f}")
        L.append(f"  roll axis in sensor frame {f3(i['u_roll'])}  gyro scale {i['scale_roll']:.4f}")
        L.append(f"  angle between axes {i['axis_angle_deg']:.2f} deg; fit residual "
                 f"{f3(i['resid_dps'])} deg/s (encoder-rate noise, not the gyro's)")
        if min(i["scale_yaw"], i["scale_roll"]) < 0.5:
            L.append("  !! a gyro scale far below 1: the AHRS is not riding on that joint")
    if "lever" in res:
        v = res["lever"]
        L.append(f"lever arm: sensing point {f3(1e3 * v['r_m'], 1)} mm (sensor frame) -> "
                 f"{v['from_roll_axis_mm']:.1f} mm from the roll axis, "
                 f"{v['from_yaw_axis_mm']:.1f} from the yaw axis; accel bias "
                 f"{f3(1e3 * v['bias_ms2'] / G_VENDOR, 1)} mg, residual "
                 f"{f3(v['resid_mg'], 1)} mg, condition {v['condition']:.0f}")
    if res.get("warning"):
        L.append("!! " + res["warning"])
    L.append("")
    L.append("error [deg], sensor heading frame: roll / pitch / heading.  lag = "
             "delay removed, rx = as received.  a_perp = non-gravity accel "
             "perpendicular to gravity [m/s^2]")
    L.append(f"{'segment':24}{'s':>6}{'yaw/roll rms':>13}{'rate':>6}{'a_perp':>7}   "
             f"{'RMS lag':^20}  {'RMS rx':^20}  {'max tilt':>8}  tau r/p")
    for r in res["segments"]:
        tau = " ".join(f"{x:.2f}" for x in r["tau_s"][:2]) if "tau_s" in r else ""
        L.append(f"{r['label'][:23]:24}{r['seconds']:6.1f}"
                 f"{r['motion_rms_deg'][0]:6.1f}{r['motion_rms_deg'][1]:7.1f}"
                 f"{r['roll_rate_rms_dps']:6.0f}{r['a_perp_rms']:7.2f}   "
                 + " ".join(f"{x:6.3f}" for x in r["rms_lag"]) + "  "
                 + " ".join(f"{x:6.3f}" for x in r["rms_rx"])
                 + f"  {r['max_tilt_lag']:8.3f}  {tau}")
        if "target_roll_rate_rms_dps" in r:
            L.append(f"{'':24}  roll rate achieved {r['roll_rate_rms_dps']:.0f} of "
                     f"{r['target_roll_rate_rms_dps']:.0f} deg/s RMS asked")
        for b in r.get("bands", []):
            gp = (f"  gain {b['gain']:.3f} phase {b['phase_deg']:+5.1f} deg"
                  if "gain" in b else "")
            L.append(f"{'':24}  {b['f_lo']:5.2f}-{b['f_hi']:5.2f} Hz  amp "
                     f"{b['motion_amp_deg']:5.2f} deg  a_perp {b['a_perp']:5.2f}  "
                     f"roll {b['rms'][0]:.3f}  pitch {b['rms'][1]:.3f}{gp}")
        if "step" in r:
            st = r["step"]
            L.append(f"{'':24}  final {f3(st['final_deg'])} deg, peak departure "
                     f"{st['peak_dev_deg']:.3f}, settled (< {SETTLE_DEG} deg) after "
                     f"{st['settle_s']:.2f} s")
    L += selfref_report(res.get("selfref") or {})
    rm = res.get("rest_model")
    if rm:
        L += ["", f"AT REST ({rm['segment']}, {rm['seconds']:.0f} s): fused tilt std "
              f"{f3(rm['fused_std_deg'], 4)} deg, raw accelerometer tilt "
              f"{f3(rm['accel_std_deg'], 3)}; best complementary filter tau "
              f"{rm['tau_s']:.2f} s tracks it at r {rm['r'][0]:.3f} / {rm['r'][1]:.3f} "
              f"(accelerometer alone {rm['r_accel_only'][0]:.3f} / {rm['r_accel_only'][1]:.3f})"]
    return L


def selfref_report(sr: dict) -> list:
    if not sr.get("segments"):
        return []
    L = ["", "ENCODER-FREE: fused tilt against the TM151's own gyro, pinned by its "
         "accelerometer in the holds either side (the mechanism is not in this)",
         f"gyro bias from {sr['bias_from_s']:.0f} s of holds {np.round(sr['bias_dps'], 4)} deg/s"]
    if "gyro_scale_roll" in sr:
        L.append(f"gyro scale about the roll axis, against the accelerometer over "
                 f"{len(sr['steps'])} step holds: {sr['gyro_scale_roll']:.4f}")
    L.append(f"{'segment':24}{'roll rms':>9}{'p99':>7}{'rate':>6}{'lag ms':>8}"
             f"{'rest':>7}{'tau s':>7}{'tilt rms':>9}{'pinned':>8}")
    for r in sr["segments"]:
        if "roll_rms_deg" in r:
            L.append(f"{r['label'][:23]:24}{r['roll_rms_deg']:9.3f}{r['roll_p99_deg']:7.3f}"
                     f"{r['roll_rate_rms_dps']:6.0f}{r['lag_ms']:8.1f}"
                     f"{r['after_lag_rms_deg']:7.3f}{r['tau_s']:7.2f}"
                     f"{r['tilt_rms_deg']:9.3f}{r['end_mismatch_deg']:8.2f}")
        else:
            L.append(f"{r['label'][:23]:24}{'':46}{r['tilt_rms_deg']:9.3f}"
                     f"{r['end_mismatch_deg']:8.2f}")
    L.append("  rest = RMS left once the lag is fitted out; pinned = the end "
             "correction spread over the segment [deg]")
    return L


def plot(res: dict, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    s = res["series"]
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    ax[0].plot(s["t"], s["yaw_deg"], label="yaw joint")
    ax[0].plot(s["t"], s["roll_deg"], label="roll joint")
    ax[0].set_ylabel("encoder [deg from ref]")
    ax[0].legend(loc="upper right")
    for j, n in enumerate(("roll", "pitch")):
        ax[1].plot(s["t"], s["err_lag_deg"][:, j], lw=0.7, label=f"{n} error")
    ax[1].set_ylabel("tilt error, delay removed [deg]")
    ax[1].legend(loc="upper right")
    ax[2].plot(s["t"], s["err_lag_deg"][:, 2], lw=0.7, color="C2")
    ax[2].set_ylabel("heading error [deg]")
    ax[3].plot(s["t"], s["a_perp"], lw=0.5, color="C3")
    ax[3].set_ylabel("a_perp [m/s^2]")
    ax[3].set_xlabel("t [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(res.get("dir", ""))
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def track_score(t, pos_deg, cmd_deg, max_lag_s: float = 0.15) -> dict:
    """How faithfully an encoder follows its command, lag aside.

    Position-mode lag is harmless here -- the encoder is the truth, not the
    command -- so the SHAPE is scored: RMS of (encoder - command) after the
    shift that minimises it. `rate_ratio` is achieved over asked RMS rate,
    both smoothed alike, which is how a too-soft servo shows up (it was 0.6 at
    the XL330's default P 400 on this replay). Frames are assumed uniform.
    """
    dt = float(np.median(np.diff(t)))
    best = (np.inf, 0)
    for k in range(int(max_lag_s / dt) + 1):
        e = pos_deg[k:] - cmd_deg[:len(cmd_deg) - k]
        best = min(best, (float(np.sqrt(np.mean(e ** 2))), k))
    rp = np.gradient(_ma(pos_deg, 5), t)
    rc = np.gradient(_ma(cmd_deg, 5), t)
    return {"raw_rms_deg": float(np.sqrt(np.mean((pos_deg - cmd_deg) ** 2))),
            "shape_rms_deg": best[0], "lag_ms": 1e3 * best[1] * dt,
            "rate_ratio": float(np.std(rp) / np.std(rc)) if np.std(rc) > 0 else np.nan}


def tune_rows(cap) -> list:
    rows = []
    segs = cap.segments
    for k, sg in enumerate(segs):
        info = sg.get("info") or {}
        if info.get("kind") != "tune" or sg["stop"] - sg["start"] < 50:
            continue
        sl = slice(sg["start"], sg["stop"])
        t = cap.arrays["t_host"][sl]
        row = {"label": sg["label"], "kp": info["kp"], "ki": info["ki"], "kd": info["kd"]}
        for dxl, name in ((ROLL, "roll"), (YAW, "yaw")):
            pos = np.degrees(cap.position_rad(dxl)[sl] - CENTER)
            cmd = np.degrees(cap.command(dxl)[sl] - CENTER)
            ok = np.isfinite(pos) & np.isfinite(cmd)
            row[name] = track_score(t[ok], pos[ok], cmd[ok])
            row[name]["current_p99_mA"] = float(
                1e3 * np.nanpercentile(np.abs(cap.value("Present Current", dxl)[sl]), 99))
        if k + 1 < len(segs) and segs[k + 1]["label"].startswith("still_"):
            nx = segs[k + 1]
            hs = slice(max(nx["stop"] - int(cap.meta["rate_hz"]), nx["start"]), nx["stop"])
            row["still_counts"] = {name: float(np.nanstd(cap.raw("Present Position", dxl)[hs]))
                                   for dxl, name in ((ROLL, "roll"), (YAW, "yaw"))}
        rows.append(row)
    return rows


def tune_report(cap) -> list:
    rows = tune_rows(cap)
    L = ["servo tracking of the replayed flight, per gain setting (shape = RMS "
         "error once the lag is removed; still = encoder std in counts during "
         "the last 1 s of the hold after it)",
         f"{'P':>6}{'I':>5}{'D':>6}   {'roll: shape  lag rate/asked  I p99':^38}"
         f"   {'yaw: shape  lag rate/asked  I p99':^38}  still r/y"]
    for r in sorted(rows, key=lambda r: r["roll"]["shape_rms_deg"]):
        cells = []
        for n in ("roll", "yaw"):
            x = r[n]
            cells.append(f"{x['shape_rms_deg']:7.3f} deg {x['lag_ms']:4.0f} ms "
                         f"{x['rate_ratio']:5.2f} {x['current_p99_mA']:6.0f} mA")
        still = r.get("still_counts", {})
        L.append(f"{r['kp']:6d}{r['ki']:5d}{r['kd']:6d}   " + "   ".join(cells)
                 + f"   {still.get('roll', np.nan):4.1f} {still.get('yaw', np.nan):4.1f}")
    if rows:
        b = min(rows, key=lambda r: r["roll"]["shape_rms_deg"])
        L.append(f"best roll shape: P {b['kp']} I {b['ki']} D {b['kd']} -- check its "
                 f"rate/asked is near 1 and 'still' is not buzzing before adopting it")
    return L


def report_any(d) -> list:
    """The report a capture directory calls for: `tune` has no AHRS."""
    from aow_sim.hw.bench_log import Capture

    d = Path(d)
    if not (d / "ahrs.npz").exists():
        return tune_report(Capture.load(d))
    return report(analyse_dir(d))


def cmd_analyse(a) -> int:
    d = Path(a.dir)
    if not (d / "ahrs.npz").exists():             # `tune`: servos only
        lines = report_any(d)
        print("\n".join(lines))
        (d / "report.txt").write_text("\n".join(lines) + "\n")
        return 0
    res = analyse_dir(a.dir)
    lines = report(res)
    print("\n".join(lines))
    (d / "report.txt").write_text("\n".join(lines) + "\n")
    summary = {k: v for k, v in res.items() if k != "series"}
    if "selfref" in summary:
        summary["selfref"] = {k: v for k, v in summary["selfref"].items() if k != "series"}
    (d / "analysis.json").write_text(json.dumps(summary, indent=1, default=lambda o: (
        o.tolist() if isinstance(o, np.ndarray) else float(o))) + "\n")
    if a.plot:
        plot(res, d / "analysis.png")
        print(f"plot -> {d / 'analysis.png'}")
    return 0


def cmd_filter_model(a) -> int:
    """Fit the complementary filter to a capture IN MOTION, then ask it what
    the lever arm contributes and what a different one would. ~5 s a pass."""
    from aow_sim.hw.bench_log import Capture

    d = Path(a.dir)
    res = analyse_dir(d)
    sr, u_r = res.get("selfref") or {}, (res.get("ident") or {}).get("u_roll")
    if not sr.get("series") or u_r is None or "lever" not in res:
        raise SystemExit("needs ident segments and moving segments with holds either side")
    cap = Capture.load(d)
    with np.load(d / "ahrs.npz") as z:
        ah = {k: z[k] for k in z.files}
    t, keep, _ = ahrs_clock(ah["t_host"], ah["t_us"])
    t, g, acc = t[keep], ah["gyro"][keep], ah["acc_g"][keep]
    fused = gravity_sensor(quats_to_mats(ah["quat"][keep]))
    bias = np.radians(sr["bias_dps"])
    th = cap.arrays["t_host"]
    spans = {s["label"]: (th[s["start"]], th[s["stop"] - 1]) for s in cap.segments
             if s["stop"] > s["start"]}
    moving = [lab for lab in sr["series"] if not lab.startswith("ident")]

    def seg(x, lab):
        return float(_rms(x[_in(t, spans[lab])]))

    print(f"model minus fused, roll [deg RMS]: moving = {len(moving)} segments pooled")
    fits = {}
    for tau in a.taus:
        v = comp_filter(t, g, acc, fused[0], tau, bias)
        dd = np.degrees(np.cross(fused, v) @ u_r)
        dd -= np.median(dd)
        fits[tau] = (float(np.sqrt(np.mean([seg(dd, lab) ** 2 for lab in moving]))), v)
        rest = seg(dd, "rest") if "rest" in spans else float("nan")
        print(f"  tau {tau:5.2f} s   moving {fits[tau][0]:.3f}   rest {rest:.4f}")
    tau = min(fits, key=lambda k: fits[k][0])
    v = fits[tau][1]
    # The SIM's model ("tm151_filter" in sim_ahrs): the same filter with tau
    # gated on the smoothed rotation rate. What it has to match is here.
    from aow_sim.sim_ahrs import run_tilt_filter
    va = run_tilt_filter(t, g - bias, acc, fused[0])
    dd = np.degrees(np.cross(fused, va) @ u_r)
    dd -= np.median(dd)
    moving_a = float(np.sqrt(np.mean([seg(dd, lab) ** 2 for lab in moving])))
    rest_a = seg(dd, "rest") if "rest" in spans else float("nan")
    print(f"  sim_ahrs 'tm151_filter' (rate-gated)   moving {moving_a:.3f}   rest {rest_a:.4f}")
    r_fit = res["lever"]["r_m"]
    r_perp = r_fit - (r_fit @ u_r) * u_r
    w5 = _ma(g - bias, 5)
    al = np.gradient(_ma(g - bias, 9), t, axis=0)

    def with_lever(dr):
        extra = np.cross(al, dr) + np.cross(w5, np.cross(w5, dr))
        return comp_filter(t, g, acc - extra / G_VENDOR, fused[0], tau, bias)

    v0 = with_lever(-r_perp)                     # the sensing point moved onto the axis
    pred = np.degrees(np.cross(v0, v) @ u_r)
    L = res["lever"]["from_roll_axis_mm"]
    print(f"\nat tau {tau:g} s: the fitted {L:.0f} mm lever arm's predicted part of the "
          f"fused error, and how much of it is there (k = 1: all)")
    for lab in moving:
        s = sr["series"][lab]
        tt = s["t"] + spans[lab][0]
        e = s["fused_roll_deg"] - s["ref_roll_deg"]
        p = np.interp(tt, t, pred)
        X = np.c_[p, np.ones(len(p))]
        c, *_ = np.linalg.lstsq(X, e, rcond=None)
        print(f"  {lab[:23]:24} predicted {_rms(p - p.mean()):.3f}   k {c[0]:+.2f}   corr "
              f"{np.corrcoef(p, e)[0, 1]:+.2f}   error {_rms(e - e.mean()):.3f} -> left "
              f"{_rms(e - X @ c):.3f}")
    rows = [r for r in tau_windows(t, g, acc, fused, bias) if r["contrast"] > 1.5]
    print(f"\ntau per {TAU_WINDOW_S:g} s window ({len(rows)} that tell the taus apart), "
          f"against what the sensor could see")
    for name, key, edges in (("mean ||acc| - its median| [mg]", "acc_dev_mg", (0, 2, 5, 10, 20, 50, 1e3)),
                             ("mean rotation rate [deg/s]", "rate_dps", (0, 1, 5, 15, 30, 60, 1e3))):
        print(f"  {name}")
        x = np.array([r[key] for r in rows])
        y = np.array([r["tau_s"] for r in rows])
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (x >= lo) & (x < hi)
            if m.sum():
                q = np.percentile(y[m], [25, 50, 75])
                print(f"    {lo:5g} - {hi:<5g} n {m.sum():3d}   tau {q[1]:.2f} s  (p25-p75 "
                      f"{q[0]:.2f}-{q[2]:.2f})")
    u = r_perp / np.linalg.norm(r_perp)
    print(f"\nthe same filter's lever-arm error at other distances from the roll axis "
          f"(same side as now; roll deg RMS)")
    for mm in a.lever_mm:
        vv = with_lever(u * mm * 1e-3 - r_perp)
        pp = np.degrees(np.cross(v0, vv) @ u_r)
        print(f"  {mm:5.0f} mm   " + "  ".join(f"{lab[:12]} {seg(pp - np.median(pp), lab):.3f}"
                                          for lab in moving))
    return 0


def cmd_repeat(a) -> int:
    """Two captures of the same plan and seeds: the part of the fused error
    that does not repeat, beside how differently the plate itself moved."""
    sa, sb = (analyse_dir(d).get("selfref", {}).get("series", {}) for d in (a.a, a.b))
    rows = repeat_difference(sa, sb)
    if not rows:
        raise SystemExit("no segment both captures can reference")
    print(f"{'segment':24}{'fused':>8}{'plate':>8}{'shift ms':>10}   "
          f"(run-to-run roll RMS / sqrt 2, deg)")
    for r in rows:
        print(f"{r['label'][:23]:24}{r['fused_rms_deg']:8.3f}{r['ref_rms_deg']:8.3f}"
              f"{r['shift_ms']:10.1f}")
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    dxl, ahrs = onboard_ports()
    bus = argparse.ArgumentParser(add_help=False)
    g = bus.add_argument_group("ports")
    g.add_argument("--port", default=os.environ.get("AOW_DXL_PORT", dxl))
    g.add_argument("--baud", type=int, default=3_000_000)
    g.add_argument("--ahrs-port", default=os.environ.get("AOW_AHRS_PORT", ahrs))

    cap = argparse.ArgumentParser(add_help=False, parents=[bus])
    g = cap.add_argument_group("capture")
    g.add_argument("--rate", type=float, default=200.0, help="servo frame rate [Hz]")
    # 1000, not the XL330's default 400: replaying sim flight 0 on the bare
    # roll servo (2026-09-23), the encoder followed the command's SHAPE to
    # 0.57 / 0.20 / 0.31 deg RMS at P 400 / 1000 / 2500 once each run's lag
    # (70 / 40 / 20 ms) was removed, and 400 delivered 20 of the 34 deg/s RMS
    # roll rate asked. The lag is harmless -- the encoder is the truth, not the
    # command -- but a smoothed motion is not the bike's. `tune` on the bare
    # servos, 20 s of flight 0 per setting: P 1500 / D 1000 best at 0.149 deg,
    # P 1000 / D 0 0.199, P 400 0.547 -- one run each, and the top five within
    # 0.05 deg. Not adopted: load moves it. Re-run `tune` with the AHRS mounted.
    g.add_argument("--kp", type=int, default=1000, help="Position P Gain")
    g.add_argument("--kd", type=int, default=0, help="Position D Gain")
    g.add_argument("--ident-deg", type=float, default=15.0,
                   help="amplitude of the prologue's yaw and roll sines")
    g.add_argument("--end-hold", type=float, default=5.0,
                   help="closing reference hold [s]; long enough to see an "
                        "offset recover (AHRS) or stay put (the rig moved)")
    g.add_argument("--raw-hz", type=float, default=0.0,
                   help="also poll raw_gyro_acc_mag(41) at this rate (0 = off)")
    g.add_argument("--min-qos", type=int, default=4,
                   help="wait for the AHRS to grade itself this well (0 = don't)")
    g.add_argument("--qos-wait", type=float, default=600.0, help="... for at most [s]")
    g.add_argument("--out", default=str(OUT))
    g.add_argument("--tag")
    g.add_argument("--note", default="",
                   help="what is mounted and how -- SAY THE HEIGHT above the roll axis")
    g.add_argument("--yes", action="store_true", help="skip the torque-on prompt")

    g = cap.add_argument_group("motion")
    g.add_argument("--axis", choices=("roll", "yaw"), default="roll",
                   help="step / chirp / sine axis")
    g.add_argument("--rest-seconds", type=float, default=300.0)
    g.add_argument("--step-amps", type=float, nargs="+", default=[5.0, 10.0, -5.0, -10.0])
    g.add_argument("--step-rate", type=float, default=100.0,
                   help="peak deg/s of a step (the sim's roll-rate p99 is 103)")
    g.add_argument("--step-hold", type=float, default=20.0)
    g.add_argument("--chirp-amps", type=float, nargs="+", default=[2.0, 8.0])
    g.add_argument("--chirp-band", type=float, nargs=2, default=[0.1, 8.0],
                   metavar=("F0", "F1"))
    g.add_argument("--chirp-seconds", type=float, default=60.0)
    g.add_argument("--a-max", type=float, default=3000.0,
                   help="deg/s^2 cap on chirps (the sim's roll-accel p99 is 2707)")
    g.add_argument("--max-rate", type=float, default=300.0, help="deg/s cap")
    g.add_argument("--amps", type=float, nargs="+", default=[2.0, 5.0, 10.0, 20.0],
                   help="sine amplitudes")
    g.add_argument("--freqs", type=float, nargs="+", default=[0.25, 0.5, 1.0, 2.0, 4.0],
                   help="sine frequencies")
    g.add_argument("--replay-file", default=str(REPLAY_FILE))
    g.add_argument("--flights", type=int, nargs="+",
                   help="replay flight indices (default: 2 that fell, 2 that did not)")
    g.add_argument("--yaw-flights", type=int, nargs="+",
                   help="yaw-only replay flight indices (default: 1 + 1)")
    g.add_argument("--yaw-limit", type=float, default=35.0,
                   help="yaw is high-passed until it fits inside this [deg]")
    g.add_argument("--sway-seconds", type=float, default=180.0)
    g.add_argument("--roll-rms", type=float, default=2.3)
    g.add_argument("--yaw-rms", type=float, default=0.0)
    g.add_argument("--band", type=float, nargs=2, default=[0.3, 5.0], metavar=("LO", "HI"),
                   help="sway band [Hz]; the sim's roll is 90%% below 3.9 Hz")
    g.add_argument("--seed", type=int, default=0)

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", parents=[bus], help="read-only: servo config and AHRS")
    sub.add_parser("setup", parents=[bus], help="EEPROM: position mode, +-45 deg")
    sub.add_parser("jog", parents=[cap], help="prologue only: home, ref, ident sines")
    p = sub.add_parser("rest", parents=[cap], help="hold centre, no motion")
    p.add_argument("--seconds", dest="rest_seconds", type=float, default=300.0)
    p.add_argument("--limp", action="store_true", help="torque off throughout")
    p = sub.add_parser("wiggle", parents=[cap],
                       help="hold centre, LEDs on: work the backlash by hand")
    p.add_argument("--seconds", dest="wiggle_seconds", type=float, default=20.0)
    p = sub.add_parser("sweep", parents=[cap],
                       help="constant-rate sweeps: rotation without acceleration")
    # Its OWN axis option. `set_defaults(axis=...)` here would rewrite the
    # default of the --axis action every command shares: 2026-09-24 it turned
    # a whole `session`'s roll steps and chirps into yaw ones.
    p.add_argument("--sweep-axis", choices=("roll", "yaw"), default="yaw")
    p.add_argument("--sweep-rates", type=float, nargs="+", default=[5.0, 10.0, 20.0, 40.0])
    p.add_argument("--sweep-amp", type=float, default=35.0)
    p.add_argument("--sweep-accel", type=float, default=300.0, help="reversals [deg/s^2]")
    p.add_argument("--sweep-cycles", type=int, default=2)
    sub.add_parser("step", parents=[cap], help="fast steps and long holds")
    sub.add_parser("chirp", parents=[cap], help="log sweeps, per-octave error")
    sub.add_parser("sine", parents=[cap], help="amplitude x frequency grid")
    sub.add_parser("replay", parents=[cap], help="the sim's standing flights")
    p = sub.add_parser("sway", parents=[cap], help="band-limited random roll (+ yaw)")
    p.add_argument("--seconds", dest="sway_seconds", type=float, default=180.0)
    p = sub.add_parser("session", parents=[cap],
                       help="rest, step, chirp, replay, yaw, sway behind one prologue")
    p.add_argument("--parts", nargs="+", choices=PARTS, default=list(PARTS))
    p.set_defaults(rest_seconds=180.0, sway_seconds=120.0)
    p = sub.add_parser("tune", parents=[cap],
                       help="servo gains: replay one sim flight per P/I/D setting")
    p.add_argument("--tune-kp", type=int, nargs="+", default=[400, 700, 1000, 1500, 2500])
    p.add_argument("--tune-ki", type=int, nargs="+", default=[0])
    p.add_argument("--tune-kd", type=int, nargs="+", default=[0, 300, 1000])
    p.add_argument("--tune-seconds", type=float, default=20.0,
                   help="how much of the flight each setting replays")
    p = sub.add_parser("analyse", help="re-run the analysis on a capture directory")
    p.add_argument("dir")
    p.add_argument("--plot", action="store_true")
    p = sub.add_parser("filter-model",
                       help="fit a complementary filter in motion; price the lever arm")
    p.add_argument("dir")
    p.add_argument("--taus", type=float, nargs="+", default=[0.19, 0.35, 0.5, 0.7, 1.0, 2.0])
    p.add_argument("--lever-mm", type=float, nargs="+", default=[0, 30, 80, 150])
    p = sub.add_parser("repeat", help="two captures of the same seeds: what does not repeat")
    p.add_argument("a")
    p.add_argument("b")
    p = sub.add_parser("export-replay", help="sim flights -> the npz `replay` reads")
    p.add_argument("--src", default=str(FALL_CAUSE_MOTION))
    p.add_argument("--out", default=str(REPLAY_FILE))
    return ap


def main() -> int:
    ap = build_parser()
    a = ap.parse_args()

    if a.cmd == "analyse":
        return cmd_analyse(a)
    if a.cmd == "repeat":
        return cmd_repeat(a)
    if a.cmd == "filter-model":
        return cmd_filter_model(a)
    if a.cmd == "export-replay":
        return cmd_export_replay(a)
    if not a.port or not a.ahrs_port:
        raise SystemExit("no --port / --ahrs-port, and none in control.onboard")
    if a.cmd == "check":
        return cmd_check(a)
    if a.cmd == "setup":
        return cmd_setup(a)
    if a.cmd == "rest":
        a.limp = getattr(a, "limp", False)
    else:
        a.limp = False
    a.test = a.cmd
    return cmd_capture(a)


if __name__ == "__main__":
    raise SystemExit(main())
