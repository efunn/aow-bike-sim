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


class AhrsSample:
    __slots__ = ("quat", "gyro", "accel", "t")

    def __init__(self, quat, gyro, accel, t):
        self.quat = quat      # (w,x,y,z), chassis frame
        self.gyro = gyro      # [wx,wy,wz] rad/s, chassis frame
        self.accel = accel    # [ax,ay,az] m/s^2, chassis frame, gravity included
        self.t = t            # time.monotonic() at parse


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
    """Ep_Combo payload -> (quat_wxyz, gyro_xyz, accel_xyz) in SI, sensor frame.

    Returns None if the payload's own `simpleChecksum` does not match, which
    catches the sensor emitting a torn struct independently of link integrity.
    """
    if len(payload) != COMBO_SIZE:
        return None
    # simpleChecksum is the last 2 bytes and excludes itself.
    if int.from_bytes(payload[-2:], "little") != sum(payload[:-2]) & 0xFFFF:
        return None

    (_hdr, _ts, _sys, _roll, _pitch, _yaw, q1, q2, q3, q4,
     wx, wy, wz, ax, ay, az, _mx, _my, _mz,
     _temp, _rate, _res, _sum) = _COMBO.unpack(payload)

    quat = np.array([q1, q2, q3, q4], dtype=float) * 1e-7     # (w,x,y,z)
    n = np.linalg.norm(quat)
    if not (0.5 < n < 2.0):        # a valid unit quaternion, loosely
        return None
    gyro = np.array([wx, wy, wz], dtype=float) * 1e-5          # rad/s
    accel = np.array([ax, ay, az], dtype=float) * 1e-5 * G_TO_MS2
    return quat / n, gyro, accel


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


class AhrsReader:
    """Background UART reader publishing the latest sample."""

    def __init__(self, port: str = "/dev/serial0", baud: int = 460800,
                 calibration: MountCalibration | None = None):
        self.port, self.baud = port, baud
        self.cal = calibration or MountCalibration()
        self._latest: AhrsSample | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.frames = 0
        self.errors = 0

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
        while not self._stop.is_set():
            chunk = self._serial.read(256)
            if not chunk:
                continue
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
            q_s, g_s, a_s = decoded
            self._latest = AhrsSample(
                quat=self.cal.to_chassis_quat(q_s),
                gyro=self.cal.to_chassis_vec(g_s),
                accel=self.cal.to_chassis_vec(a_s),
                t=time.monotonic(),
            )
            self.frames += 1

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
