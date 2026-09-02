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

THE CONTROL TABLES ARE PER MODEL AND LIVE IN `control_table.py`. An earlier
version of this file carried a nine-entry hand-typed `CT` and claimed the
XC430-W150 and XC330-T181 "have identical control tables for everything used
here". True of those nine registers, and false as soon as a bench test wants a
tenth:

  * **Address 126 is the same address and width on both and means different
    things** — `Present Load` (duty-derived, 0.1% of max torque) on the XC430,
    `Present Current` (measured mA) on the XC330. Byte-identical in an indirect
    map, different units at decode time.
  * The XC430 has a SECOND indirect block at 578/634; the XC330 does not
    (probed: `rc=0 err=0` vs `rc=0 err=7` at address 578). So a single SyncWrite
    across a mixed bus uses block 1 only, and block 1 is 28 bytes for reads and
    writes together. `IndirectMap` enforces that budget.

`CT` survives below as a compatibility alias over the shared subset. New code
should take a `ControlTable` from `control_table.table_for(model_number)` and
let the bus discover what it is talking to.

THE GENERIC LAYER IS `DynamixelBus` + `IndirectMap`; `ServoBus` IS ONE CALLER.
The bench tests in `docs/plans/first-physical-test.md` each want a different
slice of the control table logged at a different rate, so the indirect map is
built from a list of register names rather than hard-coded, and applied in ONE
SyncWrite over the whole address block. `ServoBus` is then just the bike's
particular choice of map plus its unit conversions.

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
from .control_table import (INDIRECT_ADDRESS_1, INDIRECT_DATA_1,  # noqa: F401
                            N_INDIRECT, ControlTable, Register, table_by_name,
                            table_for)

PROTOCOL = 2.0

# --- Compatibility alias over the shared subset. DERIVED from the vendored
# per-model tables rather than hand-typed, so it cannot drift from them; the
# assertion below is what makes "shared" a checked claim instead of a comment.
# New code should use `control_table.table_for(...)`.
_SHARED = ("Return Delay Time", "Operating Mode", "Torque Enable",
           "Goal Velocity", "Goal Position", "Realtime Tick",
           "Present Velocity", "Present Position", "Present Input Voltage")
CT = {n: (table_by_name("xc430_w150")[n].address,
          table_by_name("xc430_w150")[n].size) for n in _SHARED}
assert CT == {n: (table_by_name("xc330_t181")[n].address,
                  table_by_name("xc330_t181")[n].size) for n in _SHARED}, \
    "XC430 and XC330 disagree on a register CT claims is shared"

# Minimum firmware per model, by table stem.
#
# XC330-T181 needs >= 53, and 53 is EXACT rather than conservative: ROBOTIS's
# own release notes give 53 as the revision that added indirect addressing, it
# is the latest XC330 firmware, and no 51 or 52 was ever published. So the floor
# is really "53 or nothing" and there is no lower version to discover.
#
# The failure below it is silent in both directions, which is why this is a hard
# refusal rather than a warning: firmware 50 accepts every Indirect Address
# write AND ECHOES THEM ALL BACK CORRECTLY, then leaves the Indirect DATA window
# permanently ZERO. An address-only read-back passes; every frame is zeros; no
# call anywhere returns an error. Measured 2026-09-01 on id 104 at fw 50 against
# id 103 at fw 53, same part number, then confirmed fixed by the update.
#
# THE XC430 HAS ITS OWN FIRMWARE NUMBERING and is deliberately absent from this
# table. XC430-W150 at "firmware 50" is the LATEST XC430 build and is unrelated
# to the XC330's 50 -- indirect addressing works on it. Do not add a floor for
# it by pattern-matching the number.
MIN_FIRMWARE = {"xc330_t181": 53}

MODE_CURRENT = 0
MODE_VELOCITY = 1
MODE_POSITION = 3
MODE_EXTENDED_POSITION = 4
MODE_CURRENT_POSITION = 5     # current-based position, the self-righting mode
MODE_PWM = 16

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


class IndirectMap:
    """An allocation of indirect block 1, built from register NAMES.

    Indirect addressing is what lets one SyncRead cover registers that are not
    contiguous, and one SyncWrite hit a DIFFERENT register on each servo. Entry
    N is a 2-byte pointer at ``168 + 2*(N-1)`` naming the source address of ONE
    byte, which then appears at ``224 + (N-1)``.

    Two things this class exists to get right:

    * **The whole block is one pool.** Upstream's ``Indirect Address Write`` /
      ``Indirect Address Read`` entries are a naming convention, not a hardware
      partition — the labels are ignored here and entries are allocated in the
      order they are added. Reads first, then writes, so each is a contiguous
      span in the data area and each needs exactly one group transfer.
    * **Block 1 only, 28 bytes for reads and writes together.** The XC430 has a
      second block at 578/634 and the XC330 does not, so a mixed bus cannot use
      it in a single SyncWrite. The budget is checked in :meth:`apply`.

    A spec is either a register NAME (same on every servo) or a dict mapping
    servo id to name — which is how you point one slot at ``Present Load`` on
    an XC430 and ``Present Current`` on an XC330, or at ``Goal Velocity`` on
    the drives and ``Goal Position`` on the steer. Widths must agree across
    servos or the slots would not line up, and that is checked.

        imap = (IndirectMap(bus.tables)
                .read("Realtime Tick").read("Present Position")
                .read("Present Velocity").read("Present PWM")
                .read({101: "Present Load", 103: "Present Current"}, label="torque")
                .write({101: "Goal Velocity", 103: "Goal Position"}))
        bus.apply_map(imap)
    """

    def __init__(self, tables: dict):
        self.tables = dict(tables)
        self.ids = tuple(self.tables)
        self._reads: list = []      # (label, {id: Register})
        self._writes: list = []

    # -- building ----------------------------------------------------------

    def _resolve(self, spec, label):
        if isinstance(spec, str):
            per_id = {i: self.tables[i][spec] for i in self.ids}
            label = label or spec
        else:
            missing = set(self.ids) - set(spec)
            if missing:
                raise KeyError(
                    f"indirect spec {spec} does not name a register for "
                    f"id(s) {sorted(missing)}; every servo on the bus needs "
                    f"one or the slot cannot line up")
            per_id = {i: self.tables[i][spec[i]] for i in self.ids}
            label = label or "/".join(sorted({r.name for r in per_id.values()}))
        sizes = {r.size for r in per_id.values()}
        if len(sizes) != 1:
            raise ValueError(
                f"{label}: registers differ in width across servos "
                f"({ {i: (r.name, r.size) for i, r in per_id.items()} }). "
                f"Indirect slots are per-byte, so a mixed width would put "
                f"different fields at the same offset.")
        return label, per_id

    def read(self, spec, label: str | None = None) -> "IndirectMap":
        """Add a register to the per-frame READ block. Chainable."""
        self._reads.append(self._resolve(spec, label))
        return self

    def write(self, spec, label: str | None = None) -> "IndirectMap":
        """Add a register to the per-frame WRITE block. Chainable."""
        self._writes.append(self._resolve(spec, label))
        return self

    # -- geometry ----------------------------------------------------------

    @property
    def read_len(self) -> int:
        return sum(next(iter(p.values())).size for _, p in self._reads)

    @property
    def write_len(self) -> int:
        return sum(next(iter(p.values())).size for _, p in self._writes)

    @property
    def n_bytes(self) -> int:
        return self.read_len + self.write_len

    @property
    def read_addr(self) -> int:
        return INDIRECT_DATA_1

    @property
    def write_addr(self) -> int:
        return INDIRECT_DATA_1 + self.read_len

    def _offsets(self, entries, base) -> dict:
        out, off = {}, base
        for label, per_id in entries:
            out[label] = off - base
            off += next(iter(per_id.values())).size
        return out

    @property
    def read_offsets(self) -> dict:
        """label -> byte offset within the read block."""
        return self._offsets(self._reads, INDIRECT_DATA_1)

    @property
    def write_offsets(self) -> dict:
        return self._offsets(self._writes, self.write_addr)

    def register(self, dxl_id: int, label: str) -> Register:
        """The Register this servo has behind a label — the decode key."""
        for lbl, per_id in self._reads + self._writes:
            if lbl == label:
                return per_id[dxl_id]
        raise KeyError(f"no indirect slot labelled {label!r}; have "
                       f"{[l for l, _ in self._reads + self._writes]}")

    @property
    def read_labels(self) -> tuple:
        return tuple(lbl for lbl, _ in self._reads)

    def address_bytes(self, dxl_id: int) -> list:
        """The full indirect-ADDRESS payload for one servo, little-endian.

        ``2 * n_bytes`` bytes starting at :data:`INDIRECT_ADDRESS_1`, which is
        what makes the whole setup a single SyncWrite.
        """
        out = []
        for _, per_id in self._reads + self._writes:
            reg = per_id[dxl_id]
            for k in range(reg.size):
                src = reg.address + k
                out += [src & 0xFF, (src >> 8) & 0xFF]
        return out

    # -- applying ----------------------------------------------------------

    def apply(self, port, packet) -> None:
        """Write the whole map to every servo in ONE SyncWrite.

        Torque must be off: the indirect address entries are refused while a
        servo is torqued, and a refusal here is silent in its consequences —
        every later read would address whatever was there before.
        """
        from dynamixel_sdk import GroupSyncWrite

        if self.n_bytes > N_INDIRECT:
            raise RuntimeError(
                f"indirect block 1 holds {N_INDIRECT} bytes; this map needs "
                f"{self.n_bytes} ({self.read_len} read + {self.write_len} "
                f"write). Drop a register, or narrow one — the XC330 has no "
                f"second block, so a mixed bus cannot spill into 578/634.")
        writer = GroupSyncWrite(port, packet, INDIRECT_ADDRESS_1,
                                2 * self.n_bytes)
        for i in self.ids:
            if not writer.addParam(i, self.address_bytes(i)):
                raise RuntimeError(f"indirect SyncWrite refused id {i}")
        rc = writer.txPacket()
        if rc != 0:
            raise RuntimeError(f"indirect SyncWrite failed: rc={rc}")

    def verify(self, port, packet) -> None:
        """Read the indirect address block back and compare it byte for byte.

        A SyncWrite is a BROADCAST and returns no status packets — which is why
        it costs ~0.03 ms for the whole bus, and why nothing about it is
        confirmed. It is worth paying the read-back once at the start of a long
        capture, where the alternative is discovering it in the analysis.

        This checks the address pointers only. A servo whose firmware does not
        implement indirect addressing at all echoes the pointers back correctly
        and still returns an all-zero data window — that case is caught at
        discovery by the firmware floor in :data:`MIN_FIRMWARE`, not here.
        """
        for i in self.ids:
            want = self.address_bytes(i)
            for k in range(len(want) // 2):
                raw, rc, err = packet.read2ByteTxRx(
                    port, i, INDIRECT_ADDRESS_1 + 2 * k)
                if rc != 0 or err != 0:
                    raise RuntimeError(
                        f"indirect verify id={i} entry {k + 1}: rc={rc} err={err}")
                got = [raw & 0xFF, (raw >> 8) & 0xFF]
                if got != want[2 * k:2 * k + 2]:
                    raise RuntimeError(
                        f"indirect map did not stick: id={i} entry {k + 1} "
                        f"points at {raw}, expected "
                        f"{want[2 * k] | (want[2 * k + 1] << 8)}")


class DynamixelBus:
    """A bus of X-series servos, addressed by register NAME.

    The generic layer: no bike, no belt ratios, no filters. It discovers what
    each id is from its Model Number(0), resolves names through that model's
    own control table, and gives one FastSyncRead / one SyncWrite per frame
    over whatever :class:`IndirectMap` it was handed.

    Every bench test in `docs/plans/first-physical-test.md` wants a different
    slice of the control table, so nothing here is baked in.

        with DynamixelBus("/dev/ttyUSB0", ids=(101, 102, 103, 104)) as bus:
            bus.write(101, "Profile Velocity", 0)     # no trajectory generator
            bus.apply_map(IndirectMap(bus.tables)
                          .read("Realtime Tick").read("Present Position"))
            for row in bus.capture(seconds=5.0, rate_hz=250):
                ...
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 3_000_000,
                 ids=(), protocol: float = PROTOCOL):
        self.port_name, self.baud, self.protocol = port, baud, protocol
        self.ids = tuple(ids)
        self.tables: dict = {}
        self._port = self._packet = None
        self._map = self._reader = self._writer = None
        self._fast = True

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> "DynamixelBus":
        from dynamixel_sdk import PacketHandler, PortHandler

        assert_low_latency(self.port_name)
        self._port = PortHandler(self.port_name)
        if not self._port.openPort():
            raise RuntimeError(f"cannot open {self.port_name}")
        if not self._port.setBaudRate(self.baud):
            raise RuntimeError(f"cannot set {self.baud} baud on {self.port_name}")
        self._packet = PacketHandler(self.protocol)
        self.discover()
        return self

    def close(self) -> None:
        if self._port is not None:
            self._port.closePort()
            self._port = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def discover(self, ids=None) -> dict:
        """Read Model Number(0) from each id and load its control table.

        Discovery rather than configuration: a servo swapped between rigs, or
        a bench wired in a different order, announces what it is instead of
        being silently mis-decoded. Model Number is at address 0 on every
        X-series part, so this needs no table to bootstrap.
        """
        ids = tuple(ids) if ids is not None else self.ids
        tables = {}
        for i in ids:
            raw, rc, err = self._packet.read2ByteTxRx(self._port, i, 0)
            if rc != 0 or err != 0:
                raise RuntimeError(
                    f"id {i} did not answer a Model Number read "
                    f"(rc={rc} err={err}). Check power, baud ({self.baud}) "
                    f"and wiring before anything else.")
            ct = table_for(raw)
            floor = MIN_FIRMWARE.get(ct.name)
            if floor is not None:
                fw, rc, err = self._packet.read1ByteTxRx(self._port, i, 6)
                if rc != 0 or err != 0:
                    raise RuntimeError(
                        f"id {i}: Firmware Version read failed (rc={rc} err={err})")
                if fw < floor:
                    raise RuntimeError(
                        f"id {i} is a {ct.name} on firmware {fw}; this code "
                        f"needs >= {floor}. Firmware 50 accepts and echoes back "
                        f"every Indirect Address write and then leaves the "
                        f"Indirect DATA window permanently ZERO, so every frame "
                        f"from it would be zeros with no error anywhere. Update "
                        f"it in Dynamixel Wizard.")
            tables[i] = ct
        self.ids, self.tables = ids, tables
        return tables

    def scan(self, lo: int = 0, hi: int = 253) -> dict:
        """Ping sweep -> {id: model number}. For bring-up, not for loops."""
        found = {}
        for i in range(lo, hi + 1):
            raw, rc, err = self._packet.read2ByteTxRx(self._port, i, 0)
            if rc == 0 and err == 0:
                found[i] = raw
        return found

    # -- named register access --------------------------------------------

    def _rw(self, size: int, write: bool):
        p = self._packet
        return {(1, False): p.read1ByteTxRx, (2, False): p.read2ByteTxRx,
                (4, False): p.read4ByteTxRx, (1, True): p.write1ByteTxRx,
                (2, True): p.write2ByteTxRx, (4, True): p.write4ByteTxRx}[(size, write)]

    def read_raw(self, dxl_id: int, name: str) -> int:
        reg = self.tables[dxl_id][name]
        raw, rc, err = self._rw(reg.size, False)(self._port, dxl_id, reg.address)
        if rc != 0 or err != 0:
            raise RuntimeError(f"read id={dxl_id} {name}: rc={rc} err={err}")
        return raw

    def read(self, dxl_id: int, name: str) -> float:
        """Read one register, decoded to physical units by that model's table."""
        return self.tables[dxl_id][name].decode(self.read_raw(dxl_id, name))

    def write_raw(self, dxl_id: int, name: str, raw: int) -> None:
        reg = self.tables[dxl_id][name]
        rc, err = self._rw(reg.size, True)(self._port, dxl_id, reg.address, raw)
        if rc != 0 or err != 0:
            raise RuntimeError(f"write id={dxl_id} {name}={raw}: rc={rc} err={err}")

    def write(self, dxl_id: int, name: str, value: float) -> None:
        """Write one register in physical units (raw counts if it has no unit)."""
        self.write_raw(dxl_id, name, self.tables[dxl_id][name].encode(value))

    def write_all(self, name: str, value: float, ids=None) -> None:
        for i in (ids if ids is not None else self.ids):
            self.write(i, name, value)

    def torque(self, on: bool, ids=None) -> None:
        """Torque Enable across the bus in ONE SyncWrite.

        Address 64, one byte, identical on every model here -- so this needs no
        indirection and no per-servo round trip. It was four `write1ByteTxRx`
        calls, which is four request/response pairs at ~2 ms each and was most
        of `apply_map`'s cost.
        """
        from dynamixel_sdk import GroupSyncWrite

        ids = tuple(ids) if ids is not None else self.ids
        regs = {i: self.tables[i]["Torque Enable"] for i in ids}
        addrs = {r.address for r in regs.values()}
        if len(addrs) != 1:                      # cannot happen on X-series
            for i in ids:                        # ... but do not assume it
                self.write_raw(i, "Torque Enable", int(bool(on)))
            return
        writer = GroupSyncWrite(self._port, self._packet, addrs.pop(), 1)
        for i in ids:
            if not writer.addParam(i, [int(bool(on))]):
                raise RuntimeError(f"torque SyncWrite refused id {i}")
        rc = writer.txPacket()
        if rc != 0:
            raise RuntimeError(f"torque SyncWrite failed: rc={rc}")

    def prepare(self, ids=None, return_delay: int = 0,
                zero_profiles: bool = True) -> None:
        """The settings every bench test wants, applied with torque off.

        * **Return Delay Time 0.** The factory default is 250, in units of
          2 us — 500 us before the servo answers. At 3 Mbps a whole read/write
          frame is only ~330 us of wire time, so the default delay is LONGER
          THAN THE DATA TRANSFER and caps the achievable rate on its own.
        * **Profile Velocity and Profile Acceleration 0.** Non-zero profiles
          make the servo run its own trajectory generator, so a step command
          measures that generator rather than the control loop. This is the
          single most likely way to get a confidently wrong answer out of any
          of the bench tests.
        """
        ids = tuple(ids) if ids is not None else self.ids
        self.torque(False, ids)
        for i in ids:
            self.write_raw(i, "Return Delay Time", return_delay)
            if zero_profiles:
                self.write_raw(i, "Profile Acceleration", 0)
                self.write_raw(i, "Profile Velocity", 0)

    # -- per-frame I/O -----------------------------------------------------

    def apply_map(self, imap: IndirectMap, verify: bool = False) -> IndirectMap:
        """Install an indirect map and build the group handlers.

        TWO packets for the whole bus, mixed models included: one SyncWrite to
        drop torque (Torque Enable is address 64 on every model here) and one
        SyncWrite carrying each servo's own address block. Measured 0.06 ms
        total for 4 servos -- it was 8 ms when the torque-off was four separate
        `write1ByteTxRx` round trips, which is where that cost lived, not in
        the map.

        `verify=True` reads the block back; see :meth:`IndirectMap.verify` for
        when that is worth 2*n round trips.
        """
        from dynamixel_sdk import GroupSyncRead, GroupSyncWrite

        self.torque(False)
        imap.apply(self._port, self._packet)
        if verify:
            imap.verify(self._port, self._packet)
        self._map = imap
        if imap.read_len:
            self._reader = GroupSyncRead(self._port, self._packet,
                                         imap.read_addr, imap.read_len)
            for i in imap.ids:
                if not self._reader.addParam(i):
                    raise RuntimeError(f"SyncRead refused id {i}")
            # Fast by DEFAULT, with the plain path as a working fallback.
            #
            # RETRACTION, kept because the wrong version was load-bearing for a
            # while. This block once refused to fall back at all, on a measured
            # `txRxPacket rc=0 lengths {101: 14, 102: 14, 103: 14, 104: 2}` --
            # the last servo truncated with a success return code. That servo
            # was an XC330 on FIRMWARE 50, which does not populate the indirect
            # data window at all (see MIN_FIRMWARE). Re-measured 2026-09-01
            # with all four servos on good firmware, three trials each: both
            # paths return complete 14-byte buffers for every id, on the
            # indirect block and on a direct one. The SDK was fine; the servo
            # was not.
            #
            # Fast remains the default on its own merits -- all status packets
            # come back in ONE response instead of one `readRx` per servo, so
            # the frame stays uniform as servos are added -- and both were ~2 ms
            # here anyway, because the FTDI latency timer dominates.
            #
            # DO NOT FLIP `_fast` ON A LIVE READER. Switching after a
            # fastSyncRead leaves the group buffer in the other layout and the
            # next plain read raises IndexError from `getData`. Build a new bus.
            self._fast = hasattr(self._reader, "fastSyncRead")
        if imap.write_len:
            self._writer = GroupSyncWrite(self._port, self._packet,
                                          imap.write_addr, imap.write_len)
        return imap

    def read_frame(self, decode: bool = True) -> dict:
        """One FastSyncRead -> ``{id: {label: value}}``.

        Fast because all status packets come back in ONE response packet
        instead of a round trip per servo. Values are decoded through each
        servo's OWN register — which is the point of the per-model tables, and
        why the same slot can be milliamps on one servo and percent on another.
        """
        rc = (self._reader.fastSyncRead() if self._fast
              else self._reader.txRxPacket())
        if rc != 0:
            raise RuntimeError(f"SyncRead failed: rc={rc}")
        imap, out = self._map, {}
        offs = imap.read_offsets
        for i in imap.ids:
            if not self._reader.isAvailable(i, imap.read_addr, imap.read_len):
                raise RuntimeError(f"no data for id {i}")
            row = {}
            for label in imap.read_labels:
                reg = imap.register(i, label)
                raw = self._reader.getData(i, imap.read_addr + offs[label],
                                           reg.size)
                row[label] = reg.decode(raw) if decode else raw
            out[i] = row
        return out

    def write_frame(self, values: dict, encode: bool = True) -> None:
        """One SyncWrite of ``{id: value}`` through the map's write slots.

        Indirection routes the same bytes to a different register per servo,
        which is what makes it one transfer rather than a BulkWrite.
        """
        imap = self._map
        width = imap.write_len
        self._writer.clearParam()
        for i, v in values.items():
            reg = imap.register(i, imap._writes[0][0])
            raw = reg.encode(v) if encode else int(v) & 0xFFFFFFFF
            self._writer.addParam(i, [(raw >> (8 * k)) & 0xFF
                                      for k in range(width)])
        rc = self._writer.txPacket()
        if rc != 0:
            raise RuntimeError(f"SyncWrite failed: rc={rc}")

    # -- capture -----------------------------------------------------------

    def capture(self, seconds: float, rate_hz: float = 250.0,
                command=None, warn_overrun: bool = True) -> list:
        """Run a fixed-rate loop, returning one row per frame.

        ``command(t, row) -> {id: value}`` is called after each read and its
        result, if any, goes out as the frame's SyncWrite — so a test supplies
        its waveform as a function of time and reads back what happened, with
        no bespoke loop per test.

        **Timing truth is the servo's Realtime Tick, not the host clock.** The
        host paces the loop, but ``t_host`` is only recorded for diagnosis;
        every row carries each servo's own tick, sampled at the instant it read
        its encoder and immune to bus jitter and host scheduling. Map
        ``"Realtime Tick"`` into the read block or the rows carry no usable
        time base — checked below rather than discovered in the analysis.
        """
        import time

        if "Realtime Tick" not in self._map.read_labels:
            raise RuntimeError(
                "capture() needs 'Realtime Tick' in the read block — it is the "
                "only timing source immune to bus and host jitter. Add "
                ".read('Realtime Tick') to the map.")
        dt = 1.0 / float(rate_hz)
        rows, overruns = [], 0
        t0 = time.perf_counter()
        # RELATIVE to t0, like `t_host` below. It was briefly initialised to the
        # absolute `t0` instead, which made the first slack ~t0 seconds and the
        # loop sleep for hours; the `min(slack, dt)` clamp is belt-and-braces so
        # an arithmetic slip can never again turn into a hang rather than an
        # error. A pacing loop that stalls looks exactly like a wiring fault.
        next_t = 0.0
        while True:
            t_host = time.perf_counter() - t0
            if t_host >= seconds:
                break
            state = self.read_frame()
            row = {"t_host": t_host, "servos": state}
            if command is not None:
                out = command(t_host, state)
                if out:
                    self.write_frame(out)
                    row["command"] = dict(out)
            rows.append(row)
            next_t += dt
            slack = min(next_t - (time.perf_counter() - t0), dt)
            if slack > 0:
                time.sleep(slack)
            else:
                overruns += 1
                next_t = time.perf_counter() - t0
        if overruns and warn_overrun:
            print(f"WARNING: {overruns}/{len(rows)} frames overran "
                  f"{rate_hz:g} Hz; the tick deltas in the rows are the truth.")
        return rows


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
                 control_hz: float = CONTROL_HZ_DEFAULT,
                 models: dict | None = None):
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
        # The bike's servos, declared so the indirect map can be built (and
        # unit-tested) without a bus present. `open()` replaces these with what
        # the hardware actually reports and raises if the two disagree, which
        # is how a miswired bench gets caught before it produces numbers.
        self.models = dict(models) if models else {
            self.id_a: "xc430_w150", self.id_b: "xc430_w150",
            self.id_steer: "xc330_t181"}
        self.tables = {i: table_by_name(m) for i, m in self.models.items()}

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

    def _build_map(self) -> IndirectMap:
        """The bike's indirect layout, as an :class:`IndirectMap`. No I/O.

        Read block is the same registers on every servo. The write block
        deliberately is NOT: each servo maps its OWN goal register to the same
        indirect data address, which is what collapses three different goals
        into one SyncWrite. Written directly this would need a BulkWrite.

        Pure so it can be checked without a bus — see `tests/test_hw_dynamixel.py`.
        """
        imap = IndirectMap({i: self.tables[i] for i in self.ids})
        for name in READ_BLOCK:
            imap.read(name)
        imap.write({i: self.goal_item[i] for i in self.ids}, label="goal")
        self.read_addr, self.read_len = imap.read_addr, imap.read_len
        self.read_offsets = imap.read_offsets
        self.write_addr, self.write_len = imap.write_addr, imap.write_len
        self._map = imap
        return imap

    def _setup_indirect(self) -> None:
        """Build the map and install it in ONE SyncWrite. Torque must be off."""
        self._build_map().apply(self._port, self._packet)

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
