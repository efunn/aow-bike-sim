"""Dynamixel X-series bus: one FastSyncRead in, one SyncWrite out, per tick.

Built on ROBOTIS's `dynamixel_sdk`. The design follows the pattern in
`dynamixel-link`: set up INDIRECT ADDRESS blocks once at startup, then every
tick is a single contiguous transfer.

WHY INDIRECT ADDRESSING IS THE WHOLE TRICK HERE.

The three servos need different registers written: the two drives take Goal
Velocity(104), the steer takes Goal Position(116). Written directly that is a
BulkWrite, because SyncWrite cannot mix addresses. But indirect addressing
lets each servo map *its own* goal register to the *same* indirect data
address — so one 4-byte SyncWrite sets velocity on the drives and position on
the steer simultaneously. Reads collapse the same way: Realtime Tick(120),
Present Position(132) and Present Velocity(128) are not contiguous, and
indirection makes them a single 10-byte block.

Result per tick: one FastSyncRead (all three status packets in ONE response
packet, instead of one round trip per servo) and one SyncWrite (broadcast, no
status packets at all — which is why there is no "fast" write instruction).

CONVENIENTLY, XC430-W150 AND XC330-T181 HAVE IDENTICAL CONTROL TABLES for
everything used here — same Operating Mode(11), Torque Enable(64), Goal
Velocity(104), Goal Position(116), Realtime Tick(120), Present Velocity(128),
Present Position(132), Present Input Voltage(144), and the same Indirect
Address 1(168) / Indirect Data 1(224). So one table serves both and no
per-model massaging is needed. Verified against docs/robotis/*.html.

TIMING COMES FROM THE SERVOS, NOT FROM `sleep`. Realtime Tick(120) is the
servo's own millisecond clock, sampled at the instant it read its encoder.
Using its delta as the control-loop dt means the controller integrates over
time that actually elapsed rather than the 1/rate it hoped for, which matters
the moment the loop jitters or drops a tick. The tick is 2 bytes and wraps at
32768 ms (~32.8 s), handled in `_tick_delta_ms`.

VELOCITY IS RE-ESTIMATED FROM POSITION, NOT TAKEN FROM Present Velocity(128).
The servo's own estimate is heavily smoothed — on XL330/XC330 it behaves like
a ~50 ms boxcar, i.e. ~25 ms of lag on a bike whose fall time constant is
113 ms. Instead, encoder counts are differenced over the measured tick delta
and passed through `RateFilter`: a SHORT (default 25 ms) moving average,
weighted toward the present. See that class for the sweep that picked the
defaults; the short answer is that it comes out both quieter and ~3x less
laggy than the servo's own number.

Present Velocity is still read — it is free, sitting in the same block — and
surfaced as `*_reported` so a bring-up can measure how much lag the internal
filter really adds on the physical servos. `velocity_source="reported"` falls
back to it wholesale.

Worth keeping in proportion: in closed-loop sim the bike balanced identically
on ideal, 50-ms-averaged, and differenced feedback (max roll 1.5-1.7 deg
either way), because the fast state comes from the AHRS and these rates only
feed slow outer loops. The filter is cheap insurance for the real machine, not
a fix for a demonstrated instability.

THE FTDI LATENCY TRAP: the `ftdi_sio` kernel driver defaults to a 16 ms
latency timer, which caps any request/response loop at ~30 Hz regardless of
baud rate. `assert_low_latency` checks rather than trusts.

Units, all converted here so nothing downstream does arithmetic:
  * Velocity registers: 0.229 rev/min per LSB, two's complement in a 32-bit
    unsigned field (as is position, which goes negative in extended mode).
  * Position: 4096 counts/rev, the same constant control/steer.py uses.
  * Present Input Voltage(144): 0.1 V per LSB — the bike's only battery gauge.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np

from ..control.steer import XC330_COUNTS_PER_RAD, clamp_extended

PROTOCOL = 2.0

# --- X-series control table: name -> (address, size). Identical on XC430-W150
# and XC330-T181 for every item used here.
CT = {
    "Return Delay Time":     (9, 1),      # EEPROM; default 250 = 500 us (!)
    "Operating Mode":        (11, 1),     # write with torque disabled
    "Torque Enable":         (64, 1),
    "Goal Velocity":         (104, 4),
    "Goal Position":         (116, 4),
    "Realtime Tick":         (120, 2),
    "Present Velocity":      (128, 4),
    "Present Position":      (132, 4),
    "Present Input Voltage": (144, 2),
}
INDIRECT_ADDRESS_1 = 168      # 2 bytes per entry, 28 entries -> 168..223
INDIRECT_DATA_1 = 224         # 1 byte per entry, 28 entries -> 224..251
N_INDIRECT = 28

MODE_VELOCITY = 1
MODE_EXTENDED_POSITION = 4

VEL_LSB_RAD_S = 0.229 * 2 * np.pi / 60.0
VOLT_LSB = 0.1
TICK_WRAP = 32768             # Realtime Tick is 0..32767 ms
POS_WRAP = 4096               # Present Position is 0..4095 in VELOCITY mode

# The rate the sense/actuate loop runs at. Lives HERE rather than in run_bike
# because it is a property of the bus and its filters -- `RateFilter` quantises
# its window to whole ticks of it, and `assert_alias_margin` is checked against
# it. `sim_odometry` imports it so the simulated estimator ticks at the rate
# the Pi will, instead of at whatever rate its caller happens to loop at.
CONTROL_HZ_DEFAULT = 100.0

# What every servo reports each tick, in order. Contiguous once indirected.
READ_BLOCK = ("Realtime Tick", "Present Position", "Present Velocity")


class RateFilter:
    """Weighted moving average over recent position differences.

    The servo's own Present Velocity is roughly a 50 ms boxcar. Raw
    single-step differencing is the opposite extreme: no lag, but one count of
    encoder quantization lands entirely on one sample, so it is noisy at low
    speed. This sits between them — a shorter window, optionally weighted
    toward the present.

    `taper` is the weight given to the OLDEST sample relative to the newest
    (which is always 1.0), so:
        taper = 1.0   uniform boxcar. Consecutive differences telescope, so
                      this is exactly a span difference (newest - oldest) /
                      window: lowest noise, highest lag of the three.
        taper = 0.5   linear ramp to half strength at the window edge.
        taper = 0.0   linear ramp to zero: least lag, most noise.

    Samples are stored as rates, not counts, so a jittery dt is handled per
    sample rather than assumed away.

    `window_ms` is QUANTIZED to whole ticks (`n_taps`), so at 100 Hz a 20 ms
    and a 25 ms window are the same two-tap filter. Read `n_taps` and
    `group_delay_ms` rather than assuming the request was honoured exactly.

    Defaults were swept in sim against ground truth at 100 Hz (noise as RMS
    error vs true hub rate, in mm/s of bike speed):

        window  taper   lag    standstill  drive 0.6  circle
          10 ms   any   5.0 ms    8.33        7.49      6.66   (1 tap: raw)
          25 ms   0.5   8.3 ms    7.85        5.04      4.38   <- default
          50 ms   0.5  21.7 ms   10.74        3.84      3.34
        (servo's own Present Velocity)      ~9.5        ~8.5   at ~25 ms lag

    The default is both quieter AND ~3x less laggy than the servo's estimate.
    Longer windows keep helping while driving but get WORSE at standstill,
    where the residual is real crawl motion being smoothed away rather than
    quantization noise — which is why 25 ms rather than 50 ms.

    WHAT MATTERS IS THE TIME SPAN, NOT THE TAP COUNT. Quantization noise on a
    difference over span T is q/T, and consecutive differences telescope, so
    samples added INSIDE a fixed T carry no extra information about its
    endpoints. Measured, holding the span at 25 ms:

        100 Hz ->  2 taps   21.3 mm/s
        200 Hz ->  5 taps   22.3 mm/s
        500 Hz -> 12 taps   23.7 mm/s

    Same span, 6x the samples, no gain. But at a FIXED RATE more taps is a
    longer span and is emphatically not the same thing (100 Hz, taper 0.5):

        1 tap   10 ms span   lag  5.0 ms   18.0 mm/s
        2 taps  20 ms span   lag  8.3 ms   21.3 mm/s
        4 taps  40 ms span   lag 17.2 ms   30.2 mm/s
        8 taps  80 ms span   lag 35.0 ms   36.4 mm/s

    So there is a genuine optimum: too short and quantization dominates, too
    long and lag error does. Where it sits depends on how excited the signal
    is — the sweep above (both servos, three regimes) picked 25 ms; a heavily
    disturbed single-shaft test prefers nearer 10 ms. Both are simulation.
    Re-tune on logged hardware data rather than trusting either.

    DO NOT DEADBAND SMALL DIFFERENCES. Zeroing |dcount| <= 1 to suppress
    encoder jitter is tempting and measurably wrong: quantization error is
    zero-mean, so averaging recovers sub-count resolution, and a deadband
    destroys exactly that while introducing bias at low speed. Measured
    (25 ms, taper 0.5), deadband vs none:

        standstill   rms 4.11 vs 3.97 mm/s
        crawl 0.15   rms 10.65 vs 10.38, bias -0.34 vs -0.00 mm/s
        drive 0.6    identical (differences are large; it never triggers)

    Never better, and it manufactures a low-speed dead zone precisely where
    the balance loop is working hardest.
    """

    def __init__(self, window_ms: float = 25.0, taper: float = 0.5,
                 nominal_dt_ms: float = 10.0):
        if not 0.0 <= taper <= 1.0:
            raise ValueError(f"taper must be in [0, 1], got {taper}")
        n = max(1, int(round(window_ms / nominal_dt_ms)))
        w = np.linspace(1.0, taper, n) if n > 1 else np.ones(1)
        self.weights = w / w.sum()          # index 0 = most recent
        self.n_taps = n                     # window_ms rounded to whole ticks
        self.window_ms, self.taper = window_ms, taper
        self.nominal_dt_ms = nominal_dt_ms
        self._buf: deque = deque(maxlen=n)

    def update(self, rate: float) -> float:
        """Push one difference-derived rate; return the filtered estimate."""
        self._buf.appendleft(rate)
        w = self.weights[:len(self._buf)]
        return float(np.dot(w, self._buf) / w.sum())

    def peek(self) -> float:
        """The current estimate without pushing a new sample — used when a
        tick is skipped, so a dropped packet holds the last value instead of
        injecting a fake zero."""
        if not self._buf:
            return 0.0
        w = self.weights[:len(self._buf)]
        return float(np.dot(w, self._buf) / w.sum())

    def reset(self) -> None:
        self._buf.clear()

    @property
    def group_delay_ms(self) -> float:
        """Delay of the estimate behind the truth, at DC.

        A single difference already reports the average rate over the interval
        it spans, so it is centred half a sample back; the weighted average
        adds the mean lag of its own taps on top.
        """
        w = self.weights
        return (0.5 + float(np.dot(w, np.arange(len(w))))) * self.nominal_dt_ms


def _signed(v: int, nbytes: int) -> int:
    """Registers are two's complement inside an unsigned field."""
    bits = 8 * nbytes
    return v - (1 << bits) if v >= (1 << (bits - 1)) else v


def _tick_delta_ms(now: int, prev: int) -> int:
    """Elapsed servo-ms, handling the 32768 wrap."""
    return (now - prev) % TICK_WRAP


def _pos_delta(now: int, prev: int) -> int:
    """Shortest signed count delta, unwrapping the single-turn position.

    THE HUB SERVOS WRAP AND THE STEER SERVO DOES NOT. In Velocity Control Mode
    the X-series reports Present Position over ONE ROTATION -- Operating
    Mode(11) value 1 is documented as "Velocity Control Mode (0 deg ~ 360 deg)",
    the sensor is a 12-bit absolute encoder over 360 deg, and the position
    range is "0 ~ 4,095 (1 rotation)", explicitly not used in Extended Position
    Control Mode. So a continuously turning hub rolls 4095 -> 0 every
    revolution, and a plain `now - prev` reads that as a full turn backwards in
    one tick.

    Choosing the SHORTEST path is unambiguous here, and the margin is why:
    at v_max 1.2 m/s the hub turns 3.73 rev/s, and belt_ratio 3.0 puts the
    encoder on the slow side at 1.24 rev/s, so a 100 Hz sample spans ~0.012
    rev against the 0.5 rev aliasing limit -- 40x. `assert_alias_margin`
    checks it rather than assuming it.

    Applied ONLY to velocity-mode servos. The steer runs in Extended Position
    Control Mode over +-256 turns and its winding is MEANINGFUL: `control/
    steer.py` deliberately winds pi per flick and reads the multi-turn angle
    back. Unwrapping that would silently discard turns.
    """
    return ((now - prev + POS_WRAP // 2) % POS_WRAP) - POS_WRAP // 2


def assert_alias_margin(params: dict, control_hz: float,
                        min_margin: float = 5.0) -> float:
    """Fail loudly if the encoder could alias at top speed. Returns the margin.

    `_pos_delta` picks the shortest signed path, which is correct only while
    the shaft turns less than half a revolution between samples. That holds by
    a wide margin today, but it is bought entirely by the belt ratio and the
    sample rate and it DISAPPEARS SILENTLY if either moves: the symptom is not
    an exception, it is a velocity of the wrong sign at high speed, which the
    balance loop will act on.

    Checked rather than trusted, like `assert_low_latency`. Cheap, and it runs
    where the numbers are known.
    """
    v_max = float(params["control"]["drive"]["v_max"])
    r = float(params["omni_wheel"]["outer_radius"])
    belt = float(params["drivetrain"]["belt_ratio"])
    rev_s = v_max / (2 * np.pi * r) / belt      # servo revs/s at top speed
    rev_per_sample = rev_s / control_hz
    margin = 0.5 / rev_per_sample if rev_per_sample else float("inf")
    if margin < min_margin:
        raise RuntimeError(
            f"encoder aliasing margin is only {margin:.1f}x at v_max {v_max} "
            f"m/s and {control_hz:g} Hz ({rev_per_sample:.3f} rev/sample "
            f"against a 0.5 rev limit). _pos_delta picks the SHORTEST path, so "
            f"below ~1x it silently reports the wrong sign. Raise control_hz, "
            f"raise belt_ratio, or lower v_max.")
    return float(margin)


def assert_low_latency(port: str) -> None:
    """Raise unless the FTDI latency timer is 1 ms.

    Checked rather than set: writing it needs root, and a loop silently
    running at 30 Hz is far worse than a startup failure that names the fix.
    """
    dev = Path(port).name
    p = Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer")
    if not p.exists():          # non-FTDI adapter, or not Linux
        return
    value = int(p.read_text().strip())
    if value > 1:
        raise RuntimeError(
            f"{p} is {value} ms; the Dynamixel loop cannot exceed ~{1000//value} Hz.\n"
            f"Fix:  echo 1 | sudo tee {p}\n"
            f"Persist it with a udev rule for ftdi_sio (see "
            f"docs/plans/untethered-setup.md).")


class ServoBus:
    """The three servos as one device.

    Owns every unit conversion between the controller's physical units and
    register counts, including the belt and steering gear ratios — so nothing
    upstream has to remember that `ctrl` is in input-shaft rad/s while the
    servo reports its own output shaft.
    """

    def __init__(self, params: dict, port: str = "/dev/ttyUSB0",
                 baud: int = 3_000_000, ids=(1, 2, 3),
                 velocity_source: str = "differenced",
                 window_ms: float = 25.0, taper: float = 0.5,
                 control_hz: float = CONTROL_HZ_DEFAULT):
        self.id_a, self.id_b, self.id_steer = ids
        self.ids = tuple(ids)
        self.belt_ratio = float(params["drivetrain"]["belt_ratio"])
        self.steer_ratio = float(params["bike"]["steering"]["gear_ratio"])
        self.port_name, self.baud = port, baud
        if velocity_source not in ("differenced", "reported"):
            raise ValueError("velocity_source must be 'differenced' or 'reported'")
        self.velocity_source = velocity_source
        self._filters = {i: RateFilter(window_ms, taper, 1000.0 / control_hz)
                         for i in ids}
        # Per-servo goal register: this is what makes one SyncWrite enough.
        self.goal_item = {self.id_a: "Goal Velocity",
                          self.id_b: "Goal Velocity",
                          self.id_steer: "Goal Position"}
        self._port = self._packet = None
        self._prev = {}          # id -> (tick, counts)
        # Which servos report a SINGLE-TURN position and therefore wrap. The
        # two hubs run in Velocity Control Mode; the steer is Extended
        # Position and its winding carries information. Keyed off the same
        # ids `_setup_indirect` assigns the modes to, so the two cannot drift
        # apart without this line moving too.
        self._wraps = {self.id_a, self.id_b}
        self._fast = True

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> None:
        from dynamixel_sdk import GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
        assert_low_latency(self.port_name)
        self._port = PortHandler(self.port_name)
        if not self._port.openPort():
            raise RuntimeError(f"cannot open {self.port_name}")
        if not self._port.setBaudRate(self.baud):
            raise RuntimeError(f"cannot set {self.baud} baud on {self.port_name}")
        self._packet = PacketHandler(PROTOCOL)

        self.torque(False)                 # indirect setup wants torque off
        self._setup_indirect()

        self._reader = GroupSyncRead(self._port, self._packet,
                                     self.read_addr, self.read_len)
        for i in self.ids:
            if not self._reader.addParam(i):
                raise RuntimeError(f"SyncRead refused id {i}")
        self._writer = GroupSyncWrite(self._port, self._packet,
                                      self.write_addr, self.write_len)
        self._fast = hasattr(self._reader, "fastSyncRead")
        self._configure_modes()
        self.torque(True)

    def _setup_indirect(self) -> None:
        """Point indirect entries at the registers we care about.

        Read block is identical on every servo. The write block deliberately
        is NOT: each servo maps its own goal register to the same indirect
        data address, which is what collapses the write to one SyncWrite.
        """
        next_addr, next_data, used = INDIRECT_ADDRESS_1, INDIRECT_DATA_1, 0

        self.read_addr, self.read_len = next_data, 0
        self.read_offsets = {}
        for name in READ_BLOCK:
            src, size = CT[name]
            self.read_offsets[name] = next_data - self.read_addr
            for k in range(size):
                for i in self.ids:
                    self._write(i, next_addr + 2 * k, 2, src + k)
            next_addr += 2 * size
            next_data += size
            self.read_len += size
            used += size

        self.write_addr, self.write_len = next_data, 4      # one 4-byte goal
        for k in range(4):
            for i in self.ids:
                src, _ = CT[self.goal_item[i]]
                self._write(i, next_addr + 2 * k, 2, src + k)
        used += 4
        if used > N_INDIRECT:
            raise RuntimeError(f"indirect block 1 holds {N_INDIRECT} bytes, "
                               f"need {used}")

    def _configure_modes(self) -> None:
        """Modes live in EEPROM and need torque off, which open() guarantees.

        Return Delay Time is set to 0 here and it matters more than it looks:
        the factory default is 250, in units of 2 us, so every servo waits
        500 us before answering. At 3 Mbps a whole read/write tick is only
        ~330 us of wire time, so the default delay is LONGER THAN THE DATA
        TRANSFER. Zeroing it roughly halves the bus time per tick and is what
        makes headroom above ~100 Hz possible at all.
        """
        for i in self.ids:
            self._write(i, CT["Return Delay Time"][0], 1, 0)
        for i, mode in ((self.id_a, MODE_VELOCITY),
                        (self.id_b, MODE_VELOCITY),
                        (self.id_steer, MODE_EXTENDED_POSITION)):
            self._write(i, CT["Operating Mode"][0], 1, mode)

    def close(self) -> None:
        if self._port is not None:
            self.torque(False)
            self._port.closePort()
            self._port = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # -- raw register helpers ---------------------------------------------

    def _write(self, dxl_id: int, address: int, width: int, value: int) -> None:
        fn = {1: self._packet.write1ByteTxRx, 2: self._packet.write2ByteTxRx,
              4: self._packet.write4ByteTxRx}[width]
        rc, err = fn(self._port, dxl_id, address, value)
        if rc != 0 or err != 0:
            raise RuntimeError(f"write id={dxl_id} addr={address}: rc={rc} err={err}")

    def torque(self, on: bool) -> None:
        for i in self.ids:
            self._write(i, CT["Torque Enable"][0], 1, int(on))

    def pack_voltage(self) -> float:
        """Bus voltage [V], read off a servo. The bike's only battery gauge.

        Deliberately NOT in the per-tick block: it changes on a timescale of
        minutes and the read block is worth keeping small.
        """
        addr, _ = CT["Present Input Voltage"]
        raw, rc, err = self._packet.read2ByteTxRx(self._port, self.id_a, addr)
        if rc != 0 or err != 0:
            raise RuntimeError(f"voltage read: rc={rc} err={err}")
        return raw * VOLT_LSB

    # -- per-tick I/O ------------------------------------------------------

    def read_state(self) -> dict:
        """One FastSyncRead -> per-servo dict, plus the measured dt.

        Returns {'dt': seconds, 'servos': {id: {...}}} where each servo entry
        carries `pos` [rad, servo shaft], `vel` [rad/s, differenced or
        reported], and `vel_reported` [rad/s, the servo's own smoothed
        estimate].

        `dt` is the Realtime Tick delta of the FIRST drive servo — every
        servo runs its own clock, so one is chosen as the timebase rather than
        averaging clocks that were never synchronized.
        """
        rc = (self._reader.fastSyncRead() if self._fast
              else self._reader.txRxPacket())
        if rc != 0:
            raise RuntimeError(f"SyncRead failed: rc={rc}")

        off = self.read_offsets
        out, dt_s = {}, None
        for i in self.ids:
            if not self._reader.isAvailable(i, self.read_addr, self.read_len):
                raise RuntimeError(f"no data for id {i}")

            def get(name):
                a, size = CT[name]
                return self._reader.getData(i, self.read_addr + off[name], size)

            tick = get("Realtime Tick")
            counts = _signed(get("Present Position"), 4)
            vel_rep = _signed(get("Present Velocity"), 4) * VEL_LSB_RAD_S

            filt = self._filters[i]
            prev = self._prev.get(i)
            if prev is None:                 # first tick: no interval yet
                vel_diff = filt.peek()
            else:
                dms = _tick_delta_ms(tick, prev[0])
                if 0 < dms < 500:            # plausible interval
                    # UNWRAPPED for the hubs, plain for the steer -- see
                    # `_pos_delta`. The hubs run in Velocity Control Mode and
                    # report position over a single turn, so they roll
                    # 4095 -> 0 about once a second at speed; the steer runs in
                    # Extended Position Control Mode where the winding is
                    # meaningful and must NOT be unwrapped.
                    #
                    # An earlier version of this comment claimed the 40x
                    # aliasing margin "licensed plain subtraction". Backwards:
                    # the margin does not stop the wrap happening, it makes the
                    # wrap UNAMBIGUOUS to undo, because the true delta
                    # (~0.012 rev) is nowhere near the 0.5 rev limit. It
                    # licenses the unwrap, it does not replace it.
                    d = (_pos_delta(counts, prev[1]) if i in self._wraps
                         else counts - prev[1])
                    raw = (d / XC330_COUNTS_PER_RAD) / (dms * 1e-3)
                    vel_diff = filt.update(raw)
                    if i == self.id_a:
                        dt_s = dms * 1e-3
                else:                        # wrap glitch or stall: hold, do
                    vel_diff = filt.peek()   # not inject a fabricated sample
            self._prev[i] = (tick, counts)

            out[i] = {
                "pos": counts / XC330_COUNTS_PER_RAD,
                "vel": vel_diff if self.velocity_source == "differenced" else vel_rep,
                "vel_reported": vel_rep,
            }
        return {"dt": dt_s, "servos": out}

    def to_controller_units(self, state: dict) -> dict:
        """Servo shaft units -> the units the controllers and estimator use.

        `w_servo_*` stay in servo units because VelocityEstimator applies the
        belt ratio itself; `steer_*` are converted here because the controller
        works in steer-joint radians.
        """
        sv = state["servos"]
        return {
            "dt": state["dt"],
            "w_servo_a": sv[self.id_a]["vel"],
            "w_servo_b": sv[self.id_b]["vel"],
            "steer_pos": sv[self.id_steer]["pos"] / self.steer_ratio,
            "steer_vel": sv[self.id_steer]["vel"] / self.steer_ratio,
            # The servos' own smoothed estimates, carried alongside so a
            # hardware bring-up can compare them against the differenced
            # values and see how much lag the internal filter really adds.
            "w_servo_a_reported": sv[self.id_a]["vel_reported"],
            "w_servo_b_reported": sv[self.id_b]["vel_reported"],
            "steer_vel_reported": sv[self.id_steer]["vel_reported"] / self.steer_ratio,
        }

    def write_commands(self, ctrl, aid: dict) -> None:
        """One SyncWrite from the controller's `ctrl` vector.

        Every servo receives 4 bytes at the same indirect address; indirection
        routes them to Goal Velocity on the drives and Goal Position on the
        steer. `ctrl` is exactly what DriveController wrote: drive entries are
        INPUT-SHAFT rad/s (divide by belt_ratio for the servo), the steer entry
        is an absolute multi-turn steer-joint angle in radians.
        """
        from dynamixel_sdk import DXL_HIBYTE, DXL_HIWORD, DXL_LOBYTE, DXL_LOWORD

        def le4(v: int) -> list:
            v &= 0xFFFFFFFF
            return [DXL_LOBYTE(DXL_LOWORD(v)), DXL_HIBYTE(DXL_LOWORD(v)),
                    DXL_LOBYTE(DXL_HIWORD(v)), DXL_HIBYTE(DXL_HIWORD(v))]

        self._writer.clearParam()
        for dxl_id, key in ((self.id_a, "drive_a"), (self.id_b, "drive_b")):
            w_servo = float(ctrl[aid[key]]) / self.belt_ratio
            self._writer.addParam(dxl_id, le4(int(round(w_servo / VEL_LSB_RAD_S))))

        steer_rad = clamp_extended(float(ctrl[aid["steer"]]) * self.steer_ratio)
        self._writer.addParam(self.id_steer,
                              le4(int(round(steer_rad * XC330_COUNTS_PER_RAD))))

        rc = self._writer.txPacket()
        if rc != 0:
            raise RuntimeError(f"SyncWrite failed: rc={rc}")
