"""The TM151's wire protocol, in Python. No hardware needed to test it.

Ported from the vendor's C library in
`docs/ahrs/TransducerM_Lib_Protocol_C/` -- EasyProtocol.c for the framing and
EasyObjectDictionary.h for the payloads. Nothing here is guessed; every
constant below has a line in that source, and `--selftest` round-trips the
whole thing without a device attached.

WHY PORT IT AT ALL rather than shell out to their example. The point of
recording real data is to CHECK `src/aow_sim/sim_ahrs.py` against the part:
the gyro noise sigma, the orientation RMS, and above all TAU, the correlation
time of the attitude error, which is the one number in that module with no
datasheet source and which the eval results lean on. That comparison wants the
samples in numpy, not on a terminal.

THE FRAME
    0xAA 0x55  <size:u8>  <payload[size]>  <crc16:u16 LE>
    total = size + 5

  * CRC-16/MODBUS (reflected poly 0xA001, init 0xFFFF) over
    `<size> || payload` -- i.e. starting at the size byte, NOT at 0xAA.
    EasyProtocol.c:312 passes `outBuf + HEAD_LENGTH_` with length
    `payloadSize + SIZE_LENGTH_`.
  * ROUND-UP IS VESTIGIAL ON RECEIVE. `ROUND_UP_NUM_ = 4` exists and the RX
    path computes `roundUpTmp`, then does not use it: the length test at
    EasyProtocol.c:555 is `declaredPayloadSize + EP_PKG_MODIFIER_SIZE_`, and
    line 538 has `/*+ roundUpTmp*/` commented out. So frames are NOT padded and
    a parser that expects padding will desync. This cost a reading of the C to
    settle and is the single most likely thing to get wrong here.
  * `declaredPayloadSize` is rejected outside 1..70 (MAX_PAYLOAD_SIZE_), which
    is a cheap resync guard and is reproduced.

THE PAYLOAD HEADER is one little-endian uint32 of bitfields, LSB first:
    cmd:7 | qos:3 | fromId:11 | toId:11
Struct layouts follow C natural alignment on a little-endian machine; the sizes
are asserted at import against the C comments, so a mis-transcribed field
raises here rather than producing plausible nonsense.

UNITS. Note the vendor defines 1 g = 9.794 m/s^2 (main_example.c), a local
gravity value, NOT the 9.81 that `sim_ahrs.GRAVITY` uses. Accelerations are
returned in g, unconverted, and `G_VENDOR` is exported so a comparison can pick
one deliberately instead of inheriting a 0.16% discrepancy by accident.

  python analysis/tm151_serial.py --selftest      # no hardware
  python analysis/tm151_serial.py --ports         # what looks like a TM151
"""

from __future__ import annotations

import argparse
import struct
from dataclasses import dataclass

import numpy as np

HEAD = b"\xaa\x55"
MAX_PAYLOAD = 70                 # EasyProtocol.c: MAX_PAYLOAD_SIZE_
FRAME_OVERHEAD = 5               # 2 head + 1 size + 2 crc

# 1 g, as the VENDOR defines it (main_example.c). sim_ahrs uses 9.81.
G_VENDOR = 9.794

# EasyObjectDictionary.h, "Command Identifier Definition"
CMD = {12: "request", 13: "ack", 22: "status", 31: "q_s1_s", 32: "q_s1_e",
       33: "euler_s1_s", 34: "euler_s1_e", 35: "rpy", 36: "gravity",
       41: "raw_gyro_acc_mag", 43: "combo"}

# struct format -> the C struct, and its documented size.
LAYOUT = {
    "combo":            ("<IIHhhHiiiiiiiiiihhhbBHH", 68),
    "raw_gyro_acc_mag": ("<II9f", 44),
    "rpy":              ("<II3f", 20),
    "q_s1_e":           ("<II4f", 24),
    "gravity":          ("<II3f", 20),
    "status":           ("<IIfHH", 16),
}
for _n, (_f, _sz) in LAYOUT.items():
    assert struct.calcsize(_f) == _sz, f"{_n}: {struct.calcsize(_f)} != {_sz}"


def crc16(data: bytes) -> int:
    """CRC-16/MODBUS, exactly EasyProtocol_Checksum_Generate's else-branch."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            lsb = crc & 1
            crc >>= 1
            if lsb:
                crc ^= 0xA001
    return crc


def build_frame(payload: bytes) -> bytes:
    """Wrap a payload. Used by --selftest and to send requests."""
    if not 1 <= len(payload) <= MAX_PAYLOAD:
        raise ValueError(f"payload must be 1..{MAX_PAYLOAD}, got {len(payload)}")
    body = bytes([len(payload)]) + payload
    return HEAD + body + struct.pack("<H", crc16(body))


def make_header(cmd: int, from_id: int = 2, to_id: int = 0) -> int:
    """Pack the bitfield header. Default from_id is EP_ID_HOST_."""
    return (cmd & 0x7F) | (0 << 7) | ((from_id & 0x7FF) << 10) \
        | ((to_id & 0x7FF) << 21)


def request(cmd: int, to_id: int = 0) -> bytes:
    """A frame asking the device for `cmd`. Ep_Request is header + u8 + 3 pad."""
    return build_frame(struct.pack("<IB3x", make_header(12, to_id=to_id), cmd))


@dataclass
class Packet:
    kind: str
    cmd: int
    from_id: int
    qos: int
    t_us: int                    # device timestamp, microseconds
    fields: dict


def _decode(kind: str, payload: bytes) -> dict:
    fmt, size = LAYOUT[kind]
    v = struct.unpack(fmt, payload[:size])
    if kind == "combo":
        (_h, t, sysstate, roll, pitch, yaw, q1, q2, q3, q4,
         wx, wy, wz, ax, ay, az, mx, my, mz, temp, rate, _r1, _ck) = v
        return {
            "t_us": t, "qos": sysstate & 0x7,
            # roll/pitch are signed 0.01 deg; YAW IS UNSIGNED, 0..360.
            "rpy_deg": np.array([roll * 1e-2, pitch * 1e-2, yaw * 1e-2]),
            "quat": np.array([q1, q2, q3, q4], float) * 1e-7,   # w,x,y,z
            "gyro": np.array([wx, wy, wz], float) * 1e-5,       # rad/s
            "acc": np.array([ax, ay, az], float) * 1e-5,        # g
            "mag": np.array([mx, my, mz], float) * 1e-3,        # earth-field
            "temp_c": float(temp), "rate_hz": rate * 10.0,      # unit: 10 Hz
        }
    if kind == "raw_gyro_acc_mag":
        _h, t, *f = v
        return {"t_us": t, "gyro": np.array(f[0:3]), "acc": np.array(f[3:6]),
                "mag": np.array(f[6:9])}
    if kind == "rpy":
        _h, t, r, p, y = v
        return {"t_us": t, "rpy_deg": np.array([r, p, y])}
    if kind == "q_s1_e":
        _h, t, *q = v
        return {"t_us": t, "quat": np.array(q)}
    if kind == "gravity":
        _h, t, *g = v
        return {"t_us": t, "gravity": np.array(g)}
    if kind == "status":
        _h, t, temp, rate, sysstate = v
        return {"t_us": t, "temp_c": temp, "rate_hz": float(rate),
                "qos": sysstate & 0x7}
    return {"t_us": 0}


class Decoder:
    """Byte stream in, `Packet`s out. Resynchronises on garbage.

    Deliberately a pure function of the bytes, so `--selftest` exercises the
    identical path the serial port drives.
    """

    def __init__(self) -> None:
        self.buf = bytearray()
        self.n_ok = self.n_crc_bad = self.n_resync = 0

    def feed(self, data: bytes) -> list[Packet]:
        self.buf += data
        out: list[Packet] = []
        while True:
            i = self.buf.find(HEAD)
            if i < 0:
                # Keep one byte: a 0xAA may be the head's first half.
                if len(self.buf) > 1:
                    self.n_resync += len(self.buf) - 1
                    del self.buf[:-1]
                return out
            if i:
                self.n_resync += i
                del self.buf[:i]
            if len(self.buf) < 3:
                return out
            size = self.buf[2]
            if not 1 <= size <= MAX_PAYLOAD:      # EasyProtocol.c:517
                self.n_resync += 1
                del self.buf[:1]                  # drop this 0xAA, look again
                continue
            total = size + FRAME_OVERHEAD
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2:3 + size])    # size byte + payload
            got = struct.unpack("<H", self.buf[3 + size:total])[0]
            if got != crc16(body):
                self.n_crc_bad += 1
                self.n_resync += 1
                del self.buf[:1]
                continue
            payload = bytes(self.buf[3:3 + size])
            del self.buf[:total]
            hdr = struct.unpack("<I", payload[:4])[0]
            cmd = hdr & 0x7F
            kind = CMD.get(cmd, f"cmd{cmd}")
            if kind in LAYOUT and size >= LAYOUT[kind][1]:
                f = _decode(kind, payload)
                self.n_ok += 1
                out.append(Packet(kind, cmd, (hdr >> 10) & 0x7FF,
                                  f.get("qos", (hdr >> 7) & 0x7),
                                  f.get("t_us", 0), f))
            else:
                # A known frame we have no layout for, or a short one. Counted
                # as good framing -- the CRC passed -- but not returned.
                self.n_ok += 1


def find_ports() -> list[tuple[str, str]]:
    """Serial ports that plausibly are a TM151, best guess first."""
    from serial.tools import list_ports
    out = []
    for p in list_ports.comports():
        score = 0
        blob = f"{p.description} {p.manufacturer} {p.product}".lower()
        if any(k in blob for k in ("transducer", "syd", "imu", "ahrs")):
            score += 10
        # macOS: prefer /dev/cu.* over /dev/tty.* -- tty.* blocks on open
        # waiting for DCD, which on a USB CDC device simply hangs.
        if "/cu." in p.device:
            score += 3
        if "usbmodem" in p.device or "usbserial" in p.device:
            score += 2
        if score:
            out.append((score, p.device, p.description))
    return [(d, desc) for _s, d, desc in sorted(out, reverse=True)]


def _selftest() -> int:
    """Round-trip every layout, then prove the decoder resyncs through junk."""
    rng = np.random.default_rng(0)
    dec = Decoder()
    print("  round-tripping every payload type through build -> decode")

    combo = struct.pack(
        LAYOUT["combo"][0], make_header(43, from_id=7), 123456, 4,
        -1234, 567, 27000,                       # -12.34, 5.67, 270.00 deg
        9000000, 100000, -200000, 300000,        # quat * 1e-7
        12345, -6789, 101, 1000, -2000, 98000,   # gyro *1e-5, acc *1e-5 g
        150, -75, 300, 31, 40, 0, 0)             # mag, temp, rate=400 Hz
    frames = [build_frame(combo)]
    for kind, args in (("rpy", (1.5, -2.5, 33.0)),
                       ("q_s1_e", (1.0, 0.0, 0.0, 0.0)),
                       ("gravity", (0.0, 0.0, -1.0)),
                       ("raw_gyro_acc_mag", tuple(rng.normal(size=9)))):
        cmd = [k for k, v in CMD.items() if v == kind][0]
        frames.append(build_frame(struct.pack(
            LAYOUT[kind][0], make_header(cmd), 999, *args)))
    frames.append(build_frame(struct.pack(
        LAYOUT["status"][0], make_header(22), 42, 26.5, 400, 5)))

    pkts = dec.feed(b"".join(frames))
    kinds = [p.kind for p in pkts]
    assert kinds == ["combo", "rpy", "q_s1_e", "gravity", "raw_gyro_acc_mag",
                     "status"], kinds
    c = pkts[0].fields
    assert np.allclose(c["rpy_deg"], [-12.34, 5.67, 270.0]), c["rpy_deg"]
    assert np.allclose(c["quat"], [0.9, 0.01, -0.02, 0.03]), c["quat"]
    assert np.allclose(c["gyro"], [0.12345, -0.06789, 0.00101]), c["gyro"]
    assert np.allclose(c["acc"], [0.01, -0.02, 0.98]), c["acc"]
    assert c["rate_hz"] == 400.0 and c["temp_c"] == 31.0
    assert pkts[0].from_id == 7 and pkts[0].qos == 4
    print(f"    {len(pkts)} packets, fields exact")

    print("  resyncing through junk, truncation and a corrupted CRC")
    d2 = Decoder()
    junk = bytes(rng.integers(0, 256, 300, dtype=np.uint8))
    good = build_frame(combo)
    bad = bytearray(good); bad[-1] ^= 0xFF        # break the CRC
    stream = junk + bytes(bad) + good[:7] + good + junk[:50] + good
    n = sum(len(d2.feed(stream[i:i + 7])) for i in range(0, len(stream), 7))
    assert n >= 2, f"recovered {n} packets, expected at least 2"
    assert d2.n_crc_bad >= 1, "the corrupted frame was not detected"
    print(f"    recovered {n} packets, {d2.n_crc_bad} CRC rejects, "
          f"{d2.n_resync} bytes discarded")

    print("  CRC-16/MODBUS against a known vector")
    assert crc16(b"123456789") == 0x4B37, hex(crc16(b"123456789"))
    print("    0x4B37 ok")
    print("\n  SELFTEST PASSED (no hardware involved)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true",
                    help="round-trip the protocol without a device")
    ap.add_argument("--ports", action="store_true",
                    help="list serial ports that look like a TM151")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()
    if args.ports:
        found = find_ports()
        if not found:
            print("  no likely ports. Is it plugged in? Try: ls /dev/cu.*")
            return 1
        for dev, desc in found:
            print(f"  {dev:28} {desc}")
        return 0
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
