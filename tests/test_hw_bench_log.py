"""Bench capture recorder: schedule, file format and decoding, on a fake bus.

A hand-held capture is expensive to repeat, so the two things worth pinning
are that the file holds what was read -- raw, round-tripped exactly -- and that
the decoding undoes both wraps a spinning drive servo produces: Realtime Tick
at 32768 ms and single-turn Present Position at 4096 counts. Get either wrong
and every velocity derived from a capture is silently garbage at the wrap.
"""

import time

import numpy as np
import pytest

from aow_sim.hw.bench_log import Capture, Segment, raw_key, record
from aow_sim.hw.control_table import table_by_name
from aow_sim.hw.dynamixel import IndirectMap

# No model, no hardware; well under a second.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.pure

IDS = (101, 102)
COUNTS_PER_S = 3000.0            # ~4.6 rad/s at the servo shaft
START_COUNTS = {101: 4050, 102: 100}   # 101 wraps up through 4095, 102 down through 0
TICK_START = 32740               # wraps 32767 -> 0 about 28 ms in
FAIL_FRAME = 3


class FakeBus:
    """Reads that advance with the host clock, like a spinning shaft."""

    def __init__(self):
        self.ids = IDS
        self.tables = {i: table_by_name("xc430_w150") for i in IDS}
        self.t0 = time.perf_counter()
        self.n = 0
        self.writes = []

    def read_frame(self, decode=True):
        assert decode is False, "the recorder must store raw fields"
        k, self.n = self.n, self.n + 1
        if k == FAIL_FRAME:
            raise RuntimeError("SyncRead failed: rc=-3001")
        el = time.perf_counter() - self.t0
        return {i: {"Realtime Tick": (TICK_START + int(el * 1000)) % 32768,
                    "Present Position": int(START_COUNTS[i] + (1 if i == 101 else -1)
                                            * COUNTS_PER_S * el) % 4096,
                    "Present Load": (-50) & 0xFFFF,
                    "Present Input Voltage": 119}
                for i in IDS}

    def write_frame(self, values):
        self.writes.append(dict(values))


def _imap(bus):
    return (IndirectMap(bus.tables)
            .read("Realtime Tick").read("Present Position")
            .read("Present Load").read("Present Input Voltage")
            .write("Goal Velocity", label="goal"))


@pytest.fixture(scope="module")
def capture():
    bus = FakeBus()
    segs = [Segment("drive", 0.06, lambda t, s: {101: 1.0, 102: -1.0},
                    info={"kind": "drive"}),
            Segment("quiet", 0.06)]
    return record(bus, _imap(bus), segs, rate_hz=1000.0, log=None), bus


def test_round_trip_is_exact(capture, tmp_path):
    cap, _ = capture
    back = Capture.load(cap.save(tmp_path / "cap"))
    assert set(back.arrays) == set(cap.arrays)
    for k, v in cap.arrays.items():
        np.testing.assert_array_equal(back.arrays[k], v, err_msg=k)
    assert back.meta["segments"] == cap.meta["segments"]
    assert back.meta["segments"][0]["info"] == {"kind": "drive"}


def test_segments_tile_the_frames(capture):
    cap, _ = capture
    a, b = cap.segments
    assert a["start"] == 0 and a["stop"] == b["start"] and b["stop"] == len(cap)
    assert len(cap) > 40
    seg = cap.arrays["segment"]
    assert (seg[cap.segment_slice("drive")] == 0).all()
    assert (seg[cap.segment_slice(1)] == 1).all()


def test_a_failed_read_is_kept_and_masked(capture):
    cap, _ = capture
    assert cap.meta["stats"]["dropped"] == 1 and not cap.ok[FAIL_FRAME]
    assert np.isnan(cap.value("Present Load", 101)[FAIL_FRAME])
    assert cap.meta["complete"]


def test_commands_are_recorded_per_servo_and_nan_when_not_sent(capture):
    cap, bus = capture
    drive, quiet = cap.segment_slice(0), cap.segment_slice(1)
    assert (cap.command(101)[drive] == 1.0).all()
    assert (cap.command(102)[drive] == -1.0).all()
    assert np.isnan(cap.arrays["cmd"][quiet]).all()
    assert len(bus.writes) == drive.stop - drive.start


def test_decode_applies_sign_and_unit(capture):
    cap, _ = capture
    ok = cap.ok
    np.testing.assert_allclose(cap.value("Present Load", 101)[ok], -0.05)
    np.testing.assert_allclose(cap.value("Present Input Voltage", 102)[ok], 11.9)


def test_decode_reads_units_from_meta_not_from_the_table(capture):
    """The reason captures are raw on disk: a unit fix re-decodes old files."""
    cap, _ = capture
    meta = {**cap.meta, "read": [dict(e) for e in cap.meta["read"]]}
    entry = next(e for e in meta["read"] if e["label"] == "Present Input Voltage")
    entry["registers"] = {i: {**r, "unit": 0.2} for i, r in entry["registers"].items()}
    np.testing.assert_allclose(
        Capture(cap.arrays, meta).value("Present Input Voltage", 101)[cap.ok], 23.8)


def test_tick_wrap_is_undone(capture):
    cap, _ = capture
    raw = cap.raw("Realtime Tick", 101)[cap.ok]
    assert raw.min() < 100 and raw.max() > 32700, "fixture no longer wraps"
    t = cap.time_s(101)[cap.ok]
    assert (np.diff(t) >= 0).all()
    assert 0.08 < t[-1] < 0.5


@pytest.mark.parametrize("dxl_id,sign", [(101, 1), (102, -1)])
def test_position_wrap_is_undone_in_both_directions(capture, dxl_id, sign):
    cap, _ = capture
    raw = cap.raw("Present Position", dxl_id)[cap.ok]
    assert raw.min() < 500 and raw.max() > 3500, "fixture no longer wraps"
    t, p = cap.time_s(dxl_id)[cap.ok], cap.position_rad(dxl_id)[cap.ok]
    assert (sign * np.diff(p) >= 0).all()
    want = sign * COUNTS_PER_S * 2 * np.pi / 4096
    assert np.polyfit(t, p, 1)[0] == pytest.approx(want, rel=0.1)
    assert np.nanmedian(cap.diff_velocity(dxl_id, 5)) == pytest.approx(want, rel=0.15)


def test_an_interrupt_keeps_what_was_captured():
    bus = FakeBus()

    def cmd(t, s):
        if t > 0.02:
            raise KeyboardInterrupt
        return {101: 0.5, 102: 0.5}

    cap = record(bus, _imap(bus), [Segment("a", 1.0, cmd), Segment("never", 1.0)],
                 rate_hz=1000.0, log=None)
    assert cap.meta["complete"] is False
    assert cap.meta["stopped_by"] == "KeyboardInterrupt"
    assert len(cap) > 5
    n = len(cap)
    assert all(len(v) == n for k, v in cap.arrays.items() if k != "ids")
    assert len(cap.arrays[raw_key("Present Position")]) == n
    assert len(cap.segments) == 1 and cap.segments[0]["stop"] == n


def test_finish_runs_once_when_the_schedule_ends():
    """The hook that stops bus traffic before the slow array conversion; see
    `record`. Two 89 s captures tripped a 100 ms Bus Watchdog in that pause."""
    bus, calls = FakeBus(), []
    cap = record(bus, _imap(bus),
                 [Segment("a", 0.02, lambda t, s: {101: 0.0, 102: 0.0})],
                 rate_hz=1000.0, log=None,
                 finish=lambda b: calls.append((b, len(b.writes))))
    assert calls == [(bus, len(bus.writes))]
    assert cap.meta["finish_error"] is None


def test_finish_runs_on_interrupt_and_a_failing_finish_costs_nothing():
    bus, calls = FakeBus(), []

    def cmd(t, s):
        raise KeyboardInterrupt

    def finish(b):
        calls.append(1)
        raise RuntimeError("port gone")

    cap = record(bus, _imap(bus), [Segment("a", 1.0, cmd)], rate_hz=1000.0,
                 log=None, finish=finish)
    assert calls == [1]
    assert cap.meta["stopped_by"] == "KeyboardInterrupt"
    assert "port gone" in cap.meta["finish_error"]
