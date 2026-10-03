"""Dynamixel X-series over Protocol 2.0: the generic layer, for any rig.

Nothing here knows about the bike. `DynamixelBus` addresses servos by
register NAME, discovering each one's model and control table
(`control_table.py`); `IndirectMap` lays out indirect block 1 so a frame is
one FastSyncRead and one SyncWrite. The bike's use of it is `bike_bus.py`.

Indirect addressing is the main trick: each servo can map ITS OWN registers
to the same indirect data address, so one SyncWrite can set Goal Velocity on
one servo and Goal Position on another, and scattered registers (Realtime
Tick, Present Position, Present Velocity) read as one contiguous block.

Per-model gotchas, measured: `control_tables/README.md`. What to log and the
traps that each cost a capture: `docs/measurements/servo-logging.md`.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

from .control_table import (INDIRECT_ADDRESS_1, INDIRECT_DATA_1,  # noqa: F401
                            N_INDIRECT, ControlTable, Register, table_for)

PROTOCOL = 2.0

# Firmware floor per model (table stem). Below it the part fails SILENTLY:
# XC330 firmware 50 echoes indirect address writes back correctly and then
# leaves the indirect data window all zeros. The XC430 numbers its firmware
# separately; its 50 is fine. control_tables/README.md.
MIN_FIRMWARE = {"xc330_t181": 53}

MODE_CURRENT = 0
MODE_VELOCITY = 1
MODE_POSITION = 3
MODE_EXTENDED_POSITION = 4
MODE_CURRENT_POSITION = 5     # current-based position (multi-turn, current-capped)
MODE_PWM = 16

# Registers that define what a capture means but do not change per frame;
# `DynamixelBus.snapshot` records whichever of these a model has.
CONFIG_REGISTERS = (
    "Model Number", "Firmware Version", "ID", "Baud Rate", "Return Delay Time",
    "Drive Mode", "Operating Mode", "Homing Offset", "Moving Threshold",
    "Temperature Limit", "Max Voltage Limit", "Min Voltage Limit", "PWM Limit",
    "Current Limit", "Velocity Limit", "Max Position Limit",
    "Min Position Limit", "Shutdown", "Torque Enable", "Status Return Level",
    "Hardware Error Status", "Velocity I Gain", "Velocity P Gain",
    "Position D Gain", "Position I Gain", "Position P Gain",
    "Feedforward 2nd Gain", "Feedforward 1st Gain", "Bus Watchdog",
    "Profile Acceleration", "Profile Velocity", "Present Input Voltage",
    "Present Temperature")

# Hardware Error Status(70) bits. Each latches and drops torque until a REBOOT.
HARDWARE_ERROR_BITS = {0: "input voltage", 2: "overheating", 3: "motor encoder",
                       4: "electrical shock", 5: "overload"}

BUS_WATCHDOG_LSB_MS = 20      # Bus Watchdog(98) unit
BUS_WATCHDOG_TRIPPED = 255    # reads as -1 in its 1-byte field once tripped

# Status packet error byte: bit 7 is ALERT ("a hardware error is latched"),
# bits 0-6 are the result of THIS instruction.
ALERT_BIT = 0x80

VEL_LSB_RAD_S = 0.229 * 2 * math.pi / 60.0    # velocity registers: 0.229 rpm
VOLT_LSB = 0.1                                # Present Input Voltage
TICK_WRAP = 32768             # Realtime Tick is 0..32767 ms
POS_WRAP = 4096               # single-turn Present Position (velocity mode)


def instruction_failed(rc: int, err: int) -> bool:
    """Did the instruction fail? Not `err != 0`: the alert bit rides on every
    reply from a servo with a latched error, including ones that succeeded."""
    return rc != 0 or (err & ~ALERT_BIT) != 0


def describe_hardware_error(raw: int) -> str:
    """Hardware Error Status -> ``"overload, overheating"`` (``"none"`` for 0)."""
    names = [n for b, n in HARDWARE_ERROR_BITS.items() if raw & (1 << b)]
    unknown = raw & ~sum(1 << b for b in HARDWARE_ERROR_BITS)
    if unknown:
        names.append(f"unknown bits 0x{unknown:02x}")
    return ", ".join(names) or "none"


def signed(v: int, nbytes: int) -> int:
    """Two's complement inside an unsigned register field."""
    bits = 8 * nbytes
    return v - (1 << bits) if v >= (1 << (bits - 1)) else v


def tick_delta_ms(now: int, prev: int) -> int:
    """Elapsed Realtime Tick, across its 32768 ms wrap."""
    return (now - prev) % TICK_WRAP


def pos_delta(now: int, prev: int) -> int:
    """Shortest signed count delta across the single-turn 4095 -> 0 wrap.

    For velocity-mode servos only, whose Present Position covers one turn.
    Correct while the shaft turns under half a turn between samples. Never
    for (extended / current-based) position mode, where the winding is real.
    """
    return ((now - prev + POS_WRAP // 2) % POS_WRAP) - POS_WRAP // 2


def assert_low_latency(port: str) -> None:
    """Raise unless the FTDI latency timer is 1 ms (Linux only).

    The `ftdi_sio` default of 16 ms caps a request/response loop near 60 Hz
    whatever the baud. Checked, not set: setting it needs root. macOS has no
    such file and passes silently; there, run `adjust-ftdi-latency`.
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

    Entry N is a 2-byte pointer at ``168 + 2*(N-1)`` to one source byte, which
    then appears at ``224 + (N-1)``. Reads are allocated first, then writes, so
    each is one contiguous span and one group transfer. Block 1 only, 28 bytes
    for reads and writes together: the XC330 has no block 2, so a mixed bus
    cannot use it.

    A spec is a register name (the same on every servo) or ``{id: name}``, to
    point one slot at a different register per servo. Widths must agree.

        imap = (IndirectMap(bus.tables)
                .read("Realtime Tick").read("Present Position")
                .read({101: "Present Load", 103: "Present Current"}, label="effort")
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
        """The Register this servo has behind a label: its decode key."""
        for lbl, per_id in self._reads + self._writes:
            if lbl == label:
                return per_id[dxl_id]
        raise KeyError(f"no indirect slot labelled {label!r}; have "
                       f"{[l for l, _ in self._reads + self._writes]}")

    @property
    def read_labels(self) -> tuple:
        return tuple(lbl for lbl, _ in self._reads)

    @property
    def write_labels(self) -> tuple:
        return tuple(lbl for lbl, _ in self._writes)

    def address_bytes(self, dxl_id: int) -> list:
        """One servo's whole indirect ADDRESS payload, little-endian: the
        ``2 * n_bytes`` bytes from :data:`INDIRECT_ADDRESS_1`."""
        out = []
        for _, per_id in self._reads + self._writes:
            reg = per_id[dxl_id]
            for k in range(reg.size):
                src = reg.address + k
                out += [src & 0xFF, (src >> 8) & 0xFF]
        return out

    # -- applying ----------------------------------------------------------

    def apply(self, port, packet) -> None:
        """Write the whole map to every servo in ONE SyncWrite. Torque must be
        off: a torqued servo refuses the address entries, silently."""
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
        """Read the address pointers back and compare, byte for byte.

        A SyncWrite returns no status, so nothing confirms it landed. This
        checks pointers only: firmware without indirect support echoes them
        correctly anyway, which :data:`MIN_FIRMWARE` catches at discovery.
        """
        for i in self.ids:
            want = self.address_bytes(i)
            for k in range(len(want) // 2):
                raw, rc, err = packet.read2ByteTxRx(
                    port, i, INDIRECT_ADDRESS_1 + 2 * k)
                if instruction_failed(rc, err):
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

    Discovers each id's model from Model Number(0) and resolves names through
    that model's table, then runs one FastSyncRead / one SyncWrite per frame
    over whatever :class:`IndirectMap` it is given.

        with DynamixelBus("/dev/ttyUSB0", ids=(1, 2)) as bus:
            bus.prepare()
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

    @property
    def is_open(self) -> bool:
        return self._port is not None

    def recover_port(self) -> None:
        """Clear the SDK's busy flag and flush the port after an interrupted
        transaction (a signal mid-packet): until then every packet returns
        COMM_PORT_BUSY (-1000) without reaching the wire."""
        self._port.is_using = False
        try:
            self._port.clearPort()
        except Exception:                    # noqa: BLE001 -- best effort
            pass

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def discover(self, ids=None) -> dict:
        """Read each id's Model Number and load its control table; refuse a
        model below its :data:`MIN_FIRMWARE`. -> ``{id: ControlTable}``."""
        ids = tuple(ids) if ids is not None else self.ids
        tables = {}
        for i in ids:
            raw, rc, err = self._packet.read2ByteTxRx(self._port, i, 0)
            if instruction_failed(rc, err):
                raise RuntimeError(
                    f"id {i} did not answer a Model Number read "
                    f"(rc={rc} err={err}). Check power, baud ({self.baud}) "
                    f"and wiring before anything else.")
            ct = table_for(raw)
            floor = MIN_FIRMWARE.get(ct.name)
            if floor is not None:
                fw, rc, err = self._packet.read1ByteTxRx(self._port, i, 6)
                if instruction_failed(rc, err):
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
        if instruction_failed(rc, err):
            raise RuntimeError(f"read id={dxl_id} {name}: rc={rc} err={err}")
        return raw

    def read(self, dxl_id: int, name: str) -> float:
        """Read one register, decoded to physical units by that model's table."""
        return self.tables[dxl_id][name].decode(self.read_raw(dxl_id, name))

    def write_raw(self, dxl_id: int, name: str, raw: int) -> None:
        reg = self.tables[dxl_id][name]
        rc, err = self._rw(reg.size, True)(self._port, dxl_id, reg.address, raw)
        if instruction_failed(rc, err):
            raise RuntimeError(f"write id={dxl_id} {name}={raw}: rc={rc} err={err}")

    def write(self, dxl_id: int, name: str, value: float) -> None:
        """Write one register in physical units (raw counts if it has no unit)."""
        self.write_raw(dxl_id, name, self.tables[dxl_id][name].encode(value))

    def write_all(self, name: str, value: float, ids=None) -> None:
        for i in (ids if ids is not None else self.ids):
            self.write(i, name, value)

    def torque(self, on: bool, ids=None) -> None:
        """Torque Enable for every servo in ONE SyncWrite (address 64 on every
        X-series model; per-servo writes if a table ever disagrees)."""
        from dynamixel_sdk import GroupSyncWrite

        ids = tuple(ids) if ids is not None else self.ids
        regs = {i: self.tables[i]["Torque Enable"] for i in ids}
        addrs = {r.address for r in regs.values()}
        if len(addrs) != 1:
            for i in ids:
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
        """Torque off, Return Delay Time 0, and (by default) Profile Velocity
        and Acceleration 0.

        The factory return delay (500 us) is longer than a whole 3 Mbps frame,
        and a non-zero profile makes a step measure the servo's trajectory
        generator instead of its control loop.
        """
        ids = tuple(ids) if ids is not None else self.ids
        self.torque(False, ids)
        for i in ids:
            self.write_raw(i, "Return Delay Time", return_delay)
            if zero_profiles:
                self.write_raw(i, "Profile Acceleration", 0)
                self.write_raw(i, "Profile Velocity", 0)

    # -- configuration and health -------------------------------------------

    def snapshot(self, names=CONFIG_REGISTERS, ids=None) -> dict:
        """``{id: {name: raw}}`` for each named register the model has. One
        round trip per register: for bring-up, not per frame."""
        ids = tuple(ids) if ids is not None else self.ids
        return {i: {n: self.read_raw(i, n) for n in names if n in self.tables[i]}
                for i in ids}

    def hardware_errors(self, ids=None) -> dict:
        """``{id: raw}`` for each servo with a latched Hardware Error Status.
        A latched servo refuses torque silently, so check before enabling."""
        ids = tuple(ids) if ids is not None else self.ids
        return {i: e for i in ids
                if (e := self.read_raw(i, "Hardware Error Status"))}

    def reboot(self, ids=None, settle_s: float = 1.0) -> None:
        """Reboot: the only way to clear a latched hardware error. Wipes RAM
        (torque, gains, the indirect map), so do it before `prepare` and
        `apply_map`."""
        ids = tuple(ids) if ids is not None else self.ids
        for i in ids:
            rc, err = self._packet.reboot(self._port, i)
            if rc != 0:
                raise RuntimeError(f"reboot id={i}: rc={rc} err={err}")
        time.sleep(settle_s)

    def bus_watchdog(self, ms: float, ids=None) -> None:
        """Arm Bus Watchdog(98) at ``ms`` (20 ms steps), or disarm with 0.

        Armed, a servo stops itself once no packet has arrived for ``ms``.
        Tripped, it reads -1 and IGNORES goal writes, without error, until
        written back to 0 -- so disarm before any deliberate pause in traffic,
        and check `watchdog_tripped` after a capture.
        """
        units = 0 if ms <= 0 else max(1, min(127, int(round(ms / BUS_WATCHDOG_LSB_MS))))
        self.write_all("Bus Watchdog", units, ids)

    def watchdog_tripped(self, ids=None) -> dict:
        """``{id: bool}``: has Bus Watchdog fired since it was armed?"""
        ids = tuple(ids) if ids is not None else self.ids
        return {i: self.read_raw(i, "Bus Watchdog") == BUS_WATCHDOG_TRIPPED
                for i in ids}

    # -- per-frame I/O -----------------------------------------------------

    def apply_map(self, imap: IndirectMap, verify: bool = False) -> IndirectMap:
        """Torque off, install `imap` (one SyncWrite), build the group
        handlers. `verify=True` reads the pointers back (2n round trips)."""
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
            # FastSyncRead where the SDK has it: every status in ONE response.
            # Do not flip `_fast` on a live reader; the buffer layouts differ.
            self._fast = hasattr(self._reader, "fastSyncRead")
            # The decode plan, resolved once rather than per frame.
            offs = imap.read_offsets
            self._read_plan = {
                i: tuple((lbl, imap.read_addr + offs[lbl], imap.register(i, lbl))
                         for lbl in imap.read_labels)
                for i in imap.ids}
        if imap.write_len:
            self._writer = GroupSyncWrite(self._port, self._packet,
                                          imap.write_addr, imap.write_len)
        return imap

    def read_frame(self, decode: bool = True) -> dict:
        """One FastSyncRead -> ``{id: {label: value}}``, each value decoded
        through that servo's own register (so one slot can be mA on one model
        and % on another)."""
        rc = (self._reader.fastSyncRead() if self._fast
              else self._reader.txRxPacket())
        if rc != 0:
            raise RuntimeError(f"SyncRead failed: rc={rc}")
        imap, reader, out = self._map, self._reader, {}
        for i, plan in self._read_plan.items():
            if not reader.isAvailable(i, imap.read_addr, imap.read_len):
                raise RuntimeError(f"no data for id {i}")
            row = {}
            for label, addr, reg in plan:
                raw = reader.getData(i, addr, reg.size)
                row[label] = reg.decode(raw) if decode else raw
            out[i] = row
        return out

    def write_frame(self, values: dict, encode: bool = True) -> None:
        """One SyncWrite of ``{id: value}`` through the map's write slot."""
        imap = self._map
        width = imap.write_len
        self._writer.clearParam()
        for i, v in values.items():
            if encode:
                raw = imap.register(i, imap.write_labels[0]).encode(v)
            else:
                raw = int(v) & 0xFFFFFFFF
            if not self._writer.addParam(i, [(raw >> (8 * k)) & 0xFF
                                             for k in range(width)]):
                raise RuntimeError(f"SyncWrite refused id {i} (listed twice?)")
        rc = self._writer.txPacket()
        if rc != 0:
            raise RuntimeError(f"SyncWrite failed: rc={rc}")

    # -- capture -----------------------------------------------------------

    def capture(self, seconds: float, rate_hz: float = 250.0,
                command=None, warn_overrun: bool = True) -> list:
        """A fixed-rate loop -> one row per frame.

        ``command(t, row) -> {id: value}`` runs after each read; its result
        goes out as that frame's SyncWrite. The time base is each servo's own
        Realtime Tick (required in the read block); ``t_host`` is only for
        diagnosis.
        """
        if "Realtime Tick" not in self._map.read_labels:
            raise RuntimeError(
                "capture() needs 'Realtime Tick' in the read block — it is the "
                "only timing source immune to bus and host jitter. Add "
                ".read('Realtime Tick') to the map.")
        dt = 1.0 / float(rate_hz)
        rows, overruns = [], 0
        t0 = time.perf_counter()
        next_t = 0.0                # relative to t0, like t_host
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
            slack = min(next_t - (time.perf_counter() - t0), dt)   # never sleep > dt
            if slack > 0:
                time.sleep(slack)
            else:
                overruns += 1
                next_t = time.perf_counter() - t0
        if overruns and warn_overrun:
            print(f"WARNING: {overruns}/{len(rows)} frames overran "
                  f"{rate_hz:g} Hz; the tick deltas in the rows are the truth.")
        return rows
