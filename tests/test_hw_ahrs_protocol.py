"""TM151 EasyProfile frame decoding.

The parser is a port of SYD Dynamics' EasyProfile C library, so these tests
build frames with an INDEPENDENT encoder written straight from the C struct
definitions in EasyObjectDictionary.h — byte offsets spelled out rather than
reusing the parser's own format string. A shared format string would make the
round trip pass even if the layout were wrong.
"""

import struct

import numpy as np
import pytest

from aow_sim.hw.ahrs import (COMBO_SIZE, EP_CMD_COMBO, G_TO_MS2, MAX_PAYLOAD,
                             SYNC, crc16_modbus, parse_combo, parse_frame)

# Byte-level frame decoding; no bike model, no hardware.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.pure


def make_combo_payload(quat=(1.0, 0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0),
                       accel=(0.0, 0.0, 1.0), cmd=EP_CMD_COMBO,
                       timestamp=123456, break_checksum=False):
    """Build an Ep_Combo payload field-by-field, per EasyObjectDictionary.h.

    quat is (w,x,y,z) unit, gyro rad/s, accel in g.
    """
    b = bytearray(COMBO_SIZE)
    # header bitfield: cmd:7 | qos:3 | fromId:11 | toId:11, LSB-first
    header = (cmd & 0x7F) | (0 << 7) | (3 << 10) | (2 << 21)
    struct.pack_into("<I", b, 0, header)
    struct.pack_into("<I", b, 4, timestamp)
    struct.pack_into("<H", b, 8, 4)                      # sysState: QoS 4
    struct.pack_into("<hhH", b, 10, 0, 0, 0)             # roll, pitch, yaw
    for k, q in enumerate(quat):                         # q1..q4 @ 16, 1e-7
        struct.pack_into("<i", b, 16 + 4 * k, int(round(q / 1e-7)))
    for k, w in enumerate(gyro):                         # wx..wz @ 32, 1e-5
        struct.pack_into("<i", b, 32 + 4 * k, int(round(w / 1e-5)))
    for k, a in enumerate(accel):                        # ax..az @ 44, 1e-5 g
        struct.pack_into("<i", b, 44 + 4 * k, int(round(a / 1e-5)))
    struct.pack_into("<hhh", b, 56, 0, 0, 0)             # mag
    struct.pack_into("<b", b, 62, 25)                    # temperature
    struct.pack_into("<B", b, 63, 20)                    # updateRate (10 Hz)
    struct.pack_into("<H", b, 64, 0)                     # reserved1
    total = sum(b[:-2]) & 0xFFFF
    struct.pack_into("<H", b, 66, total ^ (0xFFFF if break_checksum else 0))
    return bytes(b)


def frame(payload: bytes) -> bytes:
    """Wrap a payload: AA 55 | size | payload | crc16(size+payload)."""
    body = bytes([len(payload)]) + payload
    return SYNC + body + struct.pack("<H", crc16_modbus(body))


# --- the CRC itself ----------------------------------------------------------

def test_crc16_modbus_against_known_vectors():
    """CRC-16/Modbus: init 0xFFFF, reflected poly 0xA001."""
    assert crc16_modbus(b"") == 0xFFFF
    assert crc16_modbus(b"\x00") == 0x40BF
    assert crc16_modbus(b"123456789") == 0x4B37     # the standard check value


# --- payload decoding --------------------------------------------------------

def test_combo_struct_is_68_bytes():
    """Ep_Combo must be exactly 68 bytes and fit the 70-byte payload ceiling."""
    assert COMBO_SIZE == 68
    assert COMBO_SIZE <= MAX_PAYLOAD
    assert len(make_combo_payload()) == COMBO_SIZE


def test_decodes_values_with_correct_scaling():
    q = np.array([0.9239, 0.0, 0.3827, 0.0])        # 45 deg about +Y
    q /= np.linalg.norm(q)
    gyro = (0.10, -0.25, 1.75)
    accel_g = (0.0, 0.0, 1.0)
    got = parse_combo(make_combo_payload(quat=q, gyro=gyro, accel=accel_g))
    assert got is not None
    quat, w, a = got
    assert np.allclose(quat, q, atol=1e-6)
    assert np.allclose(w, gyro, atol=1e-5), "gyro is 1e-5 rad/s per LSB"
    # accel arrives in g and must leave in m/s^2
    assert np.allclose(a, (0.0, 0.0, G_TO_MS2), atol=1e-4)


def test_negative_values_survive_signedness():
    """q, gyro and accel are all int32 two's complement."""
    q = np.array([0.5, -0.5, 0.5, -0.5])
    got = parse_combo(make_combo_payload(quat=q, gyro=(-1.5, -0.001, 2.0),
                                         accel=(-1.0, 0.5, -0.25)))
    quat, w, a = got
    assert np.allclose(quat, q, atol=1e-6)
    assert np.allclose(w, (-1.5, -0.001, 2.0), atol=1e-5)
    assert np.allclose(a, np.array((-1.0, 0.5, -0.25)) * G_TO_MS2, atol=1e-4)


def test_payload_checksum_is_enforced():
    """The in-payload simpleChecksum catches a torn struct from the sensor,
    independently of the link-level CRC."""
    assert parse_combo(make_combo_payload(break_checksum=True)) is None


def test_rejects_non_unit_quaternion():
    """A zeroed or garbage quaternion must not reach the balance controller."""
    assert parse_combo(make_combo_payload(quat=(0.0, 0.0, 0.0, 0.0))) is None


def test_rejects_wrong_length_payload():
    assert parse_combo(make_combo_payload()[:-1]) is None


# --- framing / resync --------------------------------------------------------

def test_parses_a_clean_frame():
    q = np.array([0.7071, 0.7071, 0.0, 0.0])
    decoded, consumed = parse_frame(frame(make_combo_payload(quat=q)))
    assert decoded is not None
    assert np.allclose(decoded[0], q, atol=1e-6)
    assert consumed == COMBO_SIZE + 5


def test_leading_garbage_is_skipped():
    buf = b"\x01\x02\xaa\x03\xff" + frame(make_combo_payload())
    decoded, consumed = parse_frame(buf)
    assert decoded is not None
    assert consumed == len(buf)


def test_partial_frame_is_not_consumed():
    """A frame still arriving must be left in the buffer, not discarded."""
    full = frame(make_combo_payload())
    for cut in (3, 10, len(full) - 1):
        decoded, consumed = parse_frame(full[:cut])
        assert decoded is None
        assert consumed == 0, "must not eat a frame that is still arriving"


def test_returns_newest_frame_when_several_are_buffered():
    """If the reader falls behind, stale attitude is worthless — it must skip
    to the present rather than work through a backlog."""
    old = frame(make_combo_payload(quat=(1.0, 0.0, 0.0, 0.0)))
    new_q = np.array([0.0, 1.0, 0.0, 0.0])
    new = frame(make_combo_payload(quat=new_q))
    decoded, consumed = parse_frame(old + new)
    assert np.allclose(decoded[0], new_q, atol=1e-6)
    assert consumed == len(old) + len(new)


def test_bad_crc_frame_is_rejected_and_resynced_past():
    good_q = np.array([0.0, 0.0, 1.0, 0.0])
    bad = bytearray(frame(make_combo_payload()))
    bad[-1] ^= 0xFF                                  # corrupt the CRC
    decoded, _ = parse_frame(bytes(bad))
    assert decoded is None, "a bad CRC must never decode"
    # ... and a good frame after it is still found
    decoded, _ = parse_frame(bytes(bad) + frame(make_combo_payload(quat=good_q)))
    assert decoded is not None
    assert np.allclose(decoded[0], good_q, atol=1e-6)


def test_absurd_length_byte_does_not_hang_or_consume():
    """A corrupt size byte must be treated as noise, not believed."""
    buf = SYNC + bytes([200]) + b"\x00" * 50
    decoded, consumed = parse_frame(buf)
    assert decoded is None
    assert consumed <= len(buf)


def test_other_command_ids_are_ignored_not_misparsed():
    """Only EP_CMD_COMBO_ (43) is decoded; an RPY or Status packet of the same
    length must not be read as a Combo."""
    decoded, consumed = parse_frame(frame(make_combo_payload(cmd=35)))  # RPY
    assert decoded is None
    assert consumed > 0, "a valid non-Combo frame is still consumed"


def test_sync_bytes_inside_payload_do_not_break_framing():
    """0xAA55 can occur in payload data; the length field is authoritative."""
    q = np.array([0.6, 0.8, 0.0, 0.0])
    q /= np.linalg.norm(q)
    payload = bytearray(make_combo_payload(quat=q))
    # timestamp bytes chosen to embed the sync pattern
    struct.pack_into("<I", payload, 4, 0x0000AA55)
    total = sum(payload[:-2]) & 0xFFFF
    struct.pack_into("<H", payload, 66, total)
    decoded, _ = parse_frame(frame(bytes(payload)))
    assert decoded is not None
    assert np.allclose(decoded[0], q, atol=1e-6)


def test_empty_and_tiny_buffers_are_safe():
    for buf in (b"", b"\xaa", SYNC, SYNC + b"\x44"):
        decoded, consumed = parse_frame(buf)
        assert decoded is None
        assert 0 <= consumed <= len(buf)


@pytest.mark.parametrize("junk", [b"", b"\x00" * 7, b"\xaa" * 5, b"\xaa\x55" * 3])
def test_never_loops_forever_on_hostile_input(junk):
    decoded, consumed = parse_frame(junk + frame(make_combo_payload()) + junk)
    assert decoded is not None       # completes, and still finds the frame


def test_table_crc_matches_the_reference_bit_loop():
    """crc16_modbus is table-driven for speed; it must stay bit-identical to
    the EasyProtocol.c reference over arbitrary input."""
    def reference(data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return crc

    rng = np.random.default_rng(0)
    for n in (0, 1, 2, 7, 68, 73, 200):
        for _ in range(20):
            buf = bytes(rng.integers(0, 256, n, dtype=np.uint8))
            assert crc16_modbus(buf) == reference(buf), buf.hex()
