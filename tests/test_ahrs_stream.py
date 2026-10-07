"""bench/ahrs_stream.py and the bench's port picking: no TM151, no Teensy.

The reader's child-process body runs in-process against a fake port that
plays real frames (tm151_serial.build_frame), so the decode, the batching and
the counters are the ones a run uses. Port picking is by USB vendor, and each
device is checked by what it sends, so a TM151 is never read as force counts.
"""

import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bench"))
sys.path.insert(0, str(ROOT / "analysis"))
import ahrs_stream as ah  # noqa: E402
import force_sensor as fs  # noqa: E402
import tm151_serial as tm  # noqa: E402

pytestmark = pytest.mark.pure


def combo_frame(t_us, roll_cdeg=-1234, acc=(1000, -2000, 98000)):
    return tm.build_frame(struct.pack(
        tm.LAYOUT["combo"][0], tm.make_header(43), t_us, 4,
        roll_cdeg, 567, 27000, 10000000, 0, 0, 0,
        12345, -6789, 101, *acc, 150, -75, 300, 31, 40, 0, 0))


class FakePort:
    """Plays `chunks` one read at a time, then raises like a pulled cable."""

    def __init__(self, chunks):
        self.chunks = list(chunks)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def reset_input_buffer(self):
        pass

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, n):
        if not self.chunks:
            raise OSError("device disconnected")
        return self.chunks.pop(0)


class Conn:
    def __init__(self):
        self.sent = []

    def send(self, x):
        self.sent.append(x)


def run_reader(monkeypatch, chunks):
    import serial
    monkeypatch.setattr(serial, "Serial", lambda *a, **k: FakePort(chunks))
    conn = Conn()
    ah._read("/dev/fake", ah.BAUD, conn)
    return conn.sent


def test_every_frame_is_kept_with_its_device_clock_and_fields(monkeypatch):
    frames = b"".join(combo_frame(1000 * i) for i in range(30))
    sent = run_reader(monkeypatch, [frames[i:i + 50] for i in range(0, len(frames), 50)])
    assert isinstance(sent[-1], str) and "disconnected" in sent[-1]   # the error comes through
    rows = [r for batch, _, _ in sent[:-1] for r in batch]
    assert [r[1] for r in rows] == [1000 * i for i in range(30)]     # all of them, in order
    row = dict(zip(ah.COLUMNS, rows[0]))
    assert row["kind"] == "combo" and row["qos"] == 4
    assert row["roll_deg"] == pytest.approx(-12.34)
    assert row["qw"] == pytest.approx(1.0)
    assert row["az_g"] == pytest.approx(0.98)
    assert row["gx_rad_s"] == pytest.approx(0.12345)
    hosts = [r[0] for r in rows]
    assert hosts == sorted(hosts)


def test_a_port_sending_something_else_reports_what_it_threw_away(monkeypatch):
    """A Teensy's text on the TM151's port: no frames, but the counters say
    bytes arrived -- the content check's evidence."""
    text = [b"123456,2020,1645,1690,1670\r\n"] * 40
    sent = run_reader(monkeypatch, text)
    assert all(not batch for batch, _, _ in sent[:-1])
    assert sent[-2] == ([], 0, sum(len(t) for t in text) - 1)    # one byte kept: a head's half


def test_wait_ready_refuses_a_port_with_no_combo_frames():
    s = ah.AhrsStream.__new__(ah.AhrsStream)
    s.port, s.rows, s.lock = "/dev/ttyACM0", [], __import__("threading").Lock()
    s.crc_bad, s.resync_bytes, s.error = 0, 812, None
    with pytest.raises(SystemExit) as e:
        s.wait_ready(timeout=0.05)
    assert "/dev/ttyACM0" in str(e.value) and "812 bytes discarded" in str(e.value)


def test_the_csv_has_one_row_per_frame(tmp_path, monkeypatch):
    import csv
    frames = b"".join(combo_frame(1000 * i) for i in range(5))
    sent = run_reader(monkeypatch, [frames])
    s = ah.AhrsStream.__new__(ah.AhrsStream)
    s.rows, s.lock = [r for b, _, _ in sent[:-1] for r in b], __import__("threading").Lock()
    assert s.write(tmp_path / "a.csv") == 5
    rows = list(csv.DictReader(open(tmp_path / "a.csv")))
    assert tuple(rows[0]) == ah.COLUMNS and rows[4]["t_us"] == "4000"


def ports(*spec):
    return [SimpleNamespace(device=d, vid=v, pid=0x1, manufacturer=m, product="")
            for d, v, m in spec]


@pytest.mark.parametrize("order", (0, 1))
def test_each_device_is_picked_by_vendor_whatever_the_names_sort_to(monkeypatch, order):
    """On the Pi both are ttyACM*, numbered by plug order; on the Mac both are
    cu.usbmodem*, named by socket. The vendor ID decides, not the name."""
    from serial.tools import list_ports
    names = ["/dev/ttyACM0", "/dev/ttyACM1"][::1 if order else -1]
    found = ports((names[0], 0x16C0, "Teensyduino"),
                  (names[1], 0x0483, "STMicroelectronics"),
                  ("/dev/ttyUSB0", 0x0403, "FTDI"))
    monkeypatch.setattr(list_ports, "comports", lambda: found)
    assert fs.find_port() == names[0]
    assert ah.find_port() == names[1]


def test_no_teensy_refuses_and_lists_what_is_there(monkeypatch):
    from serial.tools import list_ports
    monkeypatch.setattr(list_ports, "comports",
                        lambda: ports(("/dev/ttyACM0", 0x0483, "STMicroelectronics")))
    with pytest.raises(SystemExit) as e:
        fs.find_port()
    assert "0 ports" in str(e.value) and "/dev/ttyACM0  0483:0001" in str(e.value)


def test_the_force_reader_refuses_a_binary_stream(monkeypatch):
    """TM151 frames read as text lines: never parsed as counts, refused."""
    import serial
    blob = b"".join(combo_frame(1000 * i) for i in range(400))
    lines = [x + b"\n" for x in blob.split(b"\n")] * 50

    class Port(FakePort):
        def readline(self):
            return self.chunks.pop(0) if self.chunks else b""
    monkeypatch.setattr(serial, "Serial", lambda *a, **k: Port(lines))
    with pytest.raises(SystemExit) as e:
        next(fs.lines("/dev/ttyACM1"))
    assert "not the force sensor" in str(e.value)
