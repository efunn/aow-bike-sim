"""TM151 AHRS reader: UART -> chassis-frame orientation and angular rate.

Runs on its own thread at the sensor's output rate (configurable up to 400 Hz)
and publishes into a single latest-value slot. There is exactly one writer and
the control thread only ever reads the most recent sample, so no lock is
needed and none is taken — a control loop must never block on a sensor.

Wiring: the TM151's UART is TTL 3.3 V compatible, so it goes straight to the
Pi's GPIO UART0 (pins 8/10) with no level shifter. That is what keeps the Zero
2 W's single USB port free for the U2D2.

PROTOCOL: this is a port of SYD Dynamics' EasyProfile C library (v1.2,
`TransducerM_Lib_Protocol_C`), NOT a guess. The wire format is

    AA 55 | size(1) | payload(size) | crc16_lo crc16_hi

with CRC-16/Modbus (init 0xFFFF, reflected poly 0xA001) over `size + payload`,
i.e. everything after the two sync bytes. Note the CRC covers the length byte,
so a corrupted length is caught rather than resyncing on garbage. The
library's `ROUND_UP` padding is vestigial — the `+ roundUpTmp` term is
commented out at both use sites in EasyProtocol.c, so it never reaches the
wire.

We decode exactly one message, `EP_CMD_COMBO_` (43). It is the only packet
carrying quaternion, gyro AND accel atomically under a single timestamp, which
is what the estimator needs — taking attitude and rate from packets sampled at
different instants would inject phase error into the balance loop. At 68 bytes
it also just fits the library's 70-byte payload ceiling.

BAUD: a Combo frame is 68 + 5 = 73 bytes, so 200 Hz needs ~146 kbps. 115200 is
NOT enough. 230400 carries it arithmetically, but at 63% sustained utilization
with no flow control — and the TM151 datasheet (V1.1.6) explicitly recommends
**460800 for 200 Hz ODR**, and 921600/1M for 400 Hz. Follow the vendor: a
dropped frame here is a stale attitude in the balance loop, and the only cost
of the higher rate is a config field.

WIRING (datasheet §3, pin numbers as printed on the baseboard):
    Pin 1 RXD | Pin 2 TXD | Pin 3 VCC 5V | Pin 4 GND | Pin 5 GND
Both UART pins run at TTL 3.3 V and tolerate 5 V, so they connect straight to
the Pi's GPIO with no level shifter in either direction. Pins 4 and 5 are
internally linked — they are one net, not separate power/signal grounds, so
connecting either is sufficient.

There is a second, independent `simpleChecksum` INSIDE the Combo payload
(sum of its bytes, excluding the field itself). It is verified too: the CRC
protects the link, this protects against the sensor itself emitting a
half-updated struct.

FRAME CONVENTIONS (these are the part that is easy to get wrong):
  * Quaternions are (w, x, y, z), matching MuJoCo's qpos[3:7].
  * The gyro is BODY-frame [wx, wy, wz] = [roll rate, pitch rate, yaw rate],
    matching MuJoCo's freejoint qvel[3:6].
  * The sensor is mounted at [0.05, 0, 0.13] in the chassis frame and at some
    unknown fixed ORIENTATION relative to it. Angular velocity is the same
    everywhere on a rigid body so position does not matter for the gyro, but
    orientation does — hence `MountCalibration`.
"""

from __future__ import annotations

import struct
import threading
import time

import numpy as np


def quat_mul(a, b) -> np.ndarray:
    w0, x0, y0, z0 = a
    w1, x1, y1, z1 = b
    return np.array([
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    ])


def quat_conj(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_to_mat(q) -> np.ndarray:
    """(w,x,y,z) -> 3x3 rotation matrix. Same result as mujoco.mju_quat2Mat,
    reimplemented so the sensor thread carries no MuJoCo dependency."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class MountCalibration:
    """Fixed rotation between the AHRS case and the chassis frame.

    Procedure: hold the bike in the reference pose — upright, wheels on the
    ground, pointing along chassis +X — and call `capture`. From then on the
    reference pose reads as identity, which is what the controllers assume.

    Without this, a few degrees of mounting tilt become a permanent roll bias:
    the balance controller trims against it forever and the bike creeps.
    """

    def __init__(self, q_mount=None):
        self.q_mount = np.array([1.0, 0, 0, 0]) if q_mount is None else np.asarray(q_mount)

    def capture(self, q_sensor_at_reference) -> None:
        q = np.asarray(q_sensor_at_reference, dtype=float)
        self.q_mount = q / np.linalg.norm(q)

    def to_chassis_quat(self, q_sensor) -> np.ndarray:
        q = quat_mul(quat_conj(self.q_mount), np.asarray(q_sensor, dtype=float))
        return q / np.linalg.norm(q)

    def to_chassis_vec(self, v_sensor) -> np.ndarray:
        """Rotate a sensor-frame vector (gyro, accel) into the chassis frame."""
        return quat_to_mat(quat_conj(self.q_mount)) @ np.asarray(v_sensor, dtype=float)


# Ep_Status_SysState.qos -- the sensor grading its own fusion, from
# EasyObjectDictionary.h. Rising is better, and the two useful facts are that
# 0 and 1 mean it is NOT MEASURING, and that 4 is what a healthy unit settles
# at about 30 s after boot once DynamicGyroCalib has run.
QOS_BOOTING_OR_ASLEEP = 0
QOS_SYSTEM_FAULT = 1
QOS_LIMITED = 2           # "some functions unavailable / very limited accuracy"
QOS_BASIC = 3             # all functions, basic performance (after static boot)
QOS_FINE = 4              # all functions, fine performance (gyro calib landed)
QOS_VERY_GOOD = 5
QOS_EXTENDED = 7          # "use extended QoS definition" -- not decoded here
QOS_UNKNOWN = -1          # no frame yet, or a path that does not carry it

# What preflight insists on. 3 rather than 4: BASIC is the grade a unit holds
# in the first ~30 seconds after power-on, which is exactly when the bike is
# being armed, and refusing it would make the startup a coin flip against a
# calibration timer. 2 is excluded because the vendor's own words for it are
# "very limited measurement accuracy", which is not a thing to balance on.
QOS_MIN_SERVICE = QOS_BASIC

QOS_NAMES = {QOS_BOOTING_OR_ASLEEP: "booting or asleep", QOS_SYSTEM_FAULT: "system fault",
             QOS_LIMITED: "limited service", QOS_BASIC: "basic service",
             QOS_FINE: "fine service", QOS_VERY_GOOD: "very good service",
             6: "reserved", QOS_EXTENDED: "extended-QoS encoding",
             QOS_UNKNOWN: "unknown"}


class AhrsSample:
    __slots__ = ("quat", "gyro", "accel", "t", "qos")

    def __init__(self, quat, gyro, accel, t, qos=QOS_UNKNOWN):
        self.quat = quat      # (w,x,y,z), chassis frame
        self.gyro = gyro      # [wx,wy,wz] rad/s, chassis frame
        self.accel = accel    # [ax,ay,az] m/s^2, chassis frame, gravity included
        self.t = t            # time.monotonic() at parse
        self.qos = qos        # the sensor's OWN verdict on itself; see QOS_*


SYNC = b"\xaa\x55"
MAX_PAYLOAD = 70              # MAX_PAYLOAD_SIZE_ in EasyProtocol.c
EP_CMD_COMBO = 43
COMBO_SIZE = 68               # sizeof(Ep_Combo), packed
G_TO_MS2 = 9.794              # the library's stated 1 g, not 9.80665

# Ep_Combo, little-endian, after the 4-byte header word. See
# EasyObjectDictionary.h. Scales are applied below, not here.
_COMBO = struct.Struct("<II H hhH iiii iii iii hhh bB HH")


def _build_crc_table() -> list:
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        table.append(crc)
    return table


_CRC_TABLE = _build_crc_table()


def crc16_modbus(data: bytes) -> int:
    """CRC-16/Modbus — EasyProtocol_Checksum_Generate with the CRC option.

    Table-driven rather than the reference bit loop. This runs on the AHRS
    thread for every frame, and in CPython a pure-Python inner loop over 8
    bits x 73 bytes holds the GIL for ~44 us at a time, 200 times a second —
    landing as jitter inside whatever control tick it interrupts. One table
    lookup per byte cuts that ~8x. Verified identical against the reference
    implementation over random inputs (see tests).
    """
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def parse_combo(payload: bytes):
    """Ep_Combo payload -> (quat_wxyz, gyro_xyz, accel_xyz, qos), sensor frame.

    Returns None if the payload's own `simpleChecksum` does not match, which
    catches the sensor emitting a torn struct independently of link integrity.

    `qos` rides along because it is FREE HERE and unobtainable elsewhere: it is
    a field of the frame the loop already decodes, so reading it costs one mask
    and removes any reason to also subscribe to Ep_Status(22).
    """
    if len(payload) != COMBO_SIZE:
        return None
    # simpleChecksum is the last 2 bytes and excludes itself.
    if int.from_bytes(payload[-2:], "little") != sum(payload[:-2]) & 0xFFFF:
        return None

    # `_sys` is Ep_Status_SysState: qos in the low 3 bits (0 booting/asleep,
    # 1 fault, 2 limited, 3 basic, 4 fine, 5 very good). `_rate10` is the
    # FUSION's internal sampling rate in units of TEN Hz -- it reads 40 on this
    # unit, meaning 400 Hz, and it is NOT the output rate, which does not
    # appear in the frame at all. Ep_Status(22) carries the same number as a
    # plain uint16 Hz and agrees: 401.
    (_hdr, _ts, _sys, _roll, _pitch, _yaw, q1, q2, q3, q4,
     wx, wy, wz, ax, ay, az, _mx, _my, _mz,
     _temp, _rate10, _res, _sum) = _COMBO.unpack(payload)

    quat = np.array([q1, q2, q3, q4], dtype=float) * 1e-7     # (w,x,y,z)
    n = np.linalg.norm(quat)
    if not (0.5 < n < 2.0):        # a valid unit quaternion, loosely
        return None
    gyro = np.array([wx, wy, wz], dtype=float) * 1e-5          # rad/s
    accel = np.array([ax, ay, az], dtype=float) * 1e-5 * G_TO_MS2
    return quat / n, gyro, accel, _sys & 0x7


def parse_frame(buf: bytes):
    """Scan a byte buffer for the newest complete Combo frame.

    Returns `(decoded, consumed)`: `decoded` is (quat, gyro, accel) or None,
    and `consumed` is how many leading bytes the caller may discard.

    Deliberately returns the NEWEST complete frame in the buffer, not the
    oldest. If the reader ever falls behind, stale attitude is worthless to a
    balance loop — it should skip to the present rather than work through a
    backlog.
    """
    decoded, consumed, i = None, 0, 0
    while True:
        j = buf.find(SYNC, i)
        if j < 0:
            # No sync ahead; keep only a possible partial sync byte.
            consumed = max(consumed, max(0, len(buf) - 1))
            return decoded, consumed
        if len(buf) < j + 3:
            return decoded, max(consumed, j)      # size byte not here yet
        size = buf[j + 2]
        if size > MAX_PAYLOAD:
            i = j + 2                             # bogus length: not a frame
            continue
        end = j + 3 + size + 2
        if len(buf) < end:
            return decoded, max(consumed, j)      # frame still arriving
        body = buf[j + 2:j + 3 + size]            # size byte + payload
        crc = int.from_bytes(buf[end - 2:end], "little")
        if crc16_modbus(body) == crc:
            payload = buf[j + 3:j + 3 + size]
            cmd = payload[0] & 0x7F if payload else 0   # header word, low bits
            if cmd == EP_CMD_COMBO:
                got = parse_combo(bytes(payload))
                if got is not None:
                    decoded = got
            consumed = end
            i = end
        else:
            i = j + 2                             # bad CRC: resync past it


EP_CMD_REQUEST = 12          # Ep_Request: "send me message <cmd>"
EP_ID_HOST = 2               # from_id the vendor's own tools use


def build_request(cmd: int = EP_CMD_COMBO, to_id: int = 0) -> bytes:
    """A frame asking the sensor to send one message of type `cmd`.

    The only frame this module ever TRANSMITS, and it exists for `poll=True`
    below. Ep_Request's payload is the 4-byte header bitfield, the requested
    command, and three pad bytes. Same construction as
    `analysis/tm151_serial.request`, re-implemented here because `analysis/`
    is not installed on the bike.
    """
    payload = struct.pack("<IB3x",
                          (EP_CMD_REQUEST & 0x7F) | ((EP_ID_HOST & 0x7FF) << 10)
                          | ((to_id & 0x7FF) << 21),
                          cmd & 0x7F)
    body = bytes([len(payload)]) + payload
    return SYNC + body + struct.pack("<H", crc16_modbus(body))


class AhrsReader:
    """Background UART reader publishing the latest sample.

    TWO WAYS TO GET FRAMES, and the default is the designed one:

      * `poll=False` (default) -- the sensor free-runs and pushes Ep_Combo; the
        reader only ever reads. This is what `untethered-setup.md` specifies
        and it is the right mode when the sensor's output profile has Combo
        enabled.
      * `poll=True` -- the reader asks for each Combo with an Ep_Request. Use
        it when the unit is NOT configured to stream Combo, which is a
        configuration held in the sensor's flash and changeable only from the
        vendor's Windows GUI: the public C library, and the one its own
        "Generate Code" exports, both stop at Init/TX_Request/RX and have no
        setters at all (checked 2026-09-16).

    POLLING IS NOT A DOWNGRADE HERE, measured on a real TM151 at 200 Hz ODR:

        polled Ep_Combo, while rpy/status/raw also streamed:
            204 Hz,  latency mean 4.91 ms, p50 4.99, p99 7.61, max 9.11

    At a 200 Hz sample period that is at most one sample of age -- comparable
    to the push path. The objection in `untethered-setup.md` ("a control loop
    must never block on a sensor") does not apply, because the request/reply
    happens on THIS thread and the control thread still only reads the
    latest-value slot. What polling does cost: an 8-byte write per sample, and
    a dependence on our own timing rather than the sensor's.

    At 50 Hz ODR the same measurement was 50 Hz and 20 ms, i.e. useless for the
    loop -- so poll mode is only viable because the ODR is 200. MEASURE the
    output rate rather than reading it out of a frame: the Combo field that
    looks like one (`updateRate`) is the FUSION's internal rate in tens of Hz,
    400 on this unit against a 200 Hz output, so it answers a different
    question and reads 2x high if mistaken for the answer to this one.

    PUSH IS THE DEFAULT AGAIN since 2026-09-16, the output profile having been
    written to flash from a Windows machine and survived a power cycle.
    Measured through this class on the Mac, poll=False, 100 Hz reader ticks:
    201.4 Hz, requests 0, age-at-tick mean 2.32 ms / p50 2.42 / p99 4.87 /
    max 4.93, zero stale raises in 367 ticks.
    """

    def __init__(self, port: str = "/dev/serial0", baud: int = 460800,
                 calibration: MountCalibration | None = None,
                 poll: bool = False, poll_timeout: float = 0.015):
        self.port, self.baud = port, baud
        self.cal = calibration or MountCalibration()
        self.poll = bool(poll)
        self.poll_timeout = float(poll_timeout)
        self._request = build_request()
        self.requests = 0
        self._latest: AhrsSample | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.frames = 0          # Ep_Combo frames decoded
        self.errors = 0
        # Bytes off the wire, whatever they turned out to be. Counted because
        # `frames == 0` on its own cannot tell "nothing is plugged in" from
        # "the sensor is streaming a message we do not decode", and those have
        # completely different fixes. Measured 2026-09-16 on the bench: a
        # factory-configured TM151 streams rpy(35) + raw_gyro_acc_mag(41) +
        # status(22) at 50 Hz and NO Ep_Combo, which `parse_frame` skips
        # silently -- good CRC, wrong command -- so the reader sat at
        # frames=0, errors=0 and looked exactly like an unplugged sensor.
        self.bytes_in = 0

    def start(self) -> None:
        import serial
        self._serial = serial.Serial(self.port, self.baud, timeout=0.05)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="ahrs")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if getattr(self, "_serial", None) is not None:
            self._serial.close()

    def _run(self) -> None:
        buf = bytearray()
        # Self-pacing: ask, and ask again the moment an answer lands. On
        # silence, re-ask after `poll_timeout` rather than waiting forever --
        # a dropped request must not stall the loop, and a duplicate request
        # costs 13 bytes. Inert when `poll` is False.
        #
        # `poll_timeout` MUST BE WELL UNDER `latest()`'s max_age. It was
        # 0.05 == the 50 ms staleness limit, which made a single unanswered
        # request certain to trip it: measured on a Pi 3B+, the age climbed in
        # 10 ms steps (one per control tick) from 34 to 54 ms and the loop died,
        # a few times per 25 s, while the control thread itself was never more
        # than 0.1 ms late. 15 ms is 3x the measured 4.97 ms round trip and 3x
        # under the limit.
        deadline = 0.0
        while not self._stop.is_set():
            if self.poll and time.monotonic() >= deadline:
                try:
                    self._serial.write(self._request)
                    self.requests += 1
                except Exception:
                    self.errors += 1
                deadline = time.monotonic() + self.poll_timeout
            # TAKE WHAT IS THERE, do not wait for a fixed block. `read(256)`
            # blocks until 256 bytes have accumulated, which at the sensor's
            # ~19.5 kB/s is ~13 ms -- so the reader could never publish faster
            # than ~75 Hz however fast the sensor sent, and in poll mode the
            # reply sat in the buffer while we waited for the junk behind it.
            # Measured on a Pi 3B+: this capped AhrsReader at ~100 Hz while the
            # request/reply round trip itself was 4.97 ms.
            n = self._serial.in_waiting
            chunk = self._serial.read(n if n else 1)
            if not chunk:
                continue
            self.bytes_in += len(chunk)
            buf.extend(chunk)
            # Bound the buffer: if sync is never found (wrong baud, wrong
            # wiring) this must not grow without limit.
            if len(buf) > 8 * (MAX_PAYLOAD + 5):
                del buf[:-(MAX_PAYLOAD + 5)]
                self.errors += 1
            try:
                decoded, consumed = parse_frame(bytes(buf))
            except Exception:
                self.errors += 1
                buf.clear()
                continue
            if consumed:
                del buf[:consumed]
            if decoded is None:
                continue
            q_s, g_s, a_s, qos = decoded
            self._latest = AhrsSample(
                quat=self.cal.to_chassis_quat(q_s),
                gyro=self.cal.to_chassis_vec(g_s),
                accel=self.cal.to_chassis_vec(a_s),
                t=time.monotonic(),
                qos=qos,
            )
            self.frames += 1
            deadline = 0.0          # answered -- ask for the next one now

    def wait_ready(self, timeout: float = 2.0) -> float:
        """Block until the first sample lands. -> seconds waited.

        THE STARTUP RACE THIS EXISTS FOR: `start()` only spawns the thread, so
        for the first few milliseconds there is no sample and `latest()`
        raises. `run_bike` used to call `preflight_ahrs` immediately after
        `start()`, and preflight returns on its FIRST failure -- so it reported
        a dead AHRS microseconds after opening a port that was working
        perfectly. In poll mode it can never win that race, because the first
        sample costs a request round trip.

        Raises with the byte counters in the message, which is what separates
        "not plugged in" from "streaming the wrong message".
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._latest is not None:
                return time.monotonic() - t0
            time.sleep(0.005)
        raise RuntimeError(
            f"no AHRS sample in {timeout:.1f} s on {self.port}: "
            f"{self.bytes_in} bytes read, {self.frames} Ep_Combo frames, "
            f"{self.errors} errors, {self.requests} requests sent"
            + ("" if self.poll else
               " -- push mode; if bytes are arriving but no frames decode, the "
               "sensor is not configured to stream Ep_Combo (try ahrs_poll)"))

    def latest(self, max_age: float = 0.05) -> AhrsSample:
        """Most recent sample, or raise if it is stale.

        Stale orientation is more dangerous than no orientation — the
        controller would keep balancing confidently against a frozen attitude
        — so this raises and lets the failsafe cut torque.
        """
        s = self._latest
        if s is None:
            raise RuntimeError("no AHRS sample yet")
        age = time.monotonic() - s.t
        if age > max_age:
            raise RuntimeError(f"AHRS sample is {age*1000:.0f} ms stale "
                               f"(limit {max_age*1000:.0f} ms)")
        return s
