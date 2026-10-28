"""hw/clock.py: the record's true wall-clock start, from one SNTP query.

Offline: the query is answered by a server on localhost."""
import socket
import struct
import threading
import time

import pytest

from aow_sim.hw import clock

# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.pure


def _ntp(t: float) -> bytes:
    t += clock._NTP_EPOCH
    return struct.pack("!II", int(t), int((t % 1) * 2 ** 32))


def _server(ahead_s: float, mode: int = 4):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))

    def serve():
        data, peer = s.recvfrom(512)
        now = time.time() + ahead_s
        reply = (bytes([0x20 | mode, 2]) + bytes(30) + _ntp(now) + _ntp(now))
        s.sendto(reply, peer)
        s.close()
    threading.Thread(target=serve, daemon=True).start()
    return s.getsockname()[1]


def test_the_offset_is_the_servers_clock_minus_ours():
    port = _server(ahead_s=600.0)
    got = clock.sntp("127.0.0.1", timeout=1.0, port=port)
    assert got["offset_s"] == pytest.approx(600.0, abs=0.05)
    assert 0 <= got["rtt_s"] < 0.1


def test_a_reply_that_is_not_from_a_server_is_refused():
    with pytest.raises(ValueError):
        clock.sntp("127.0.0.1", timeout=1.0, port=_server(0.0, mode=3))


def test_no_answer_is_an_error_not_a_hang():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))                   # bound, never answers
    t = time.monotonic()
    with pytest.raises(OSError):
        clock.sntp("127.0.0.1", timeout=0.2, port=s.getsockname()[1])
    assert time.monotonic() - t < 1.0
    s.close()


def _meta(start=None, close=None, step=0.0):
    """A run whose row 0 was at local wall 1000 (mono 50), 60 s long; the
    local clock stepped `step` s between row 0 and the close check."""
    c = {"t0_wall": 1000.0, "t0_mono": 50.0}
    if start is not None:
        c["start"] = {"wall": 1000.2, "mono": 50.2, "offset_s": start,
                      "rtt_s": 0.03, "server": "x"}
    if close is not None:
        c["close"] = {"wall": 1060.0 + step, "mono": 110.0, "offset_s": close,
                      "rtt_s": 0.03, "server": "x"}
    return {"clock": c}


def test_the_start_check_corrects_row_0():
    t, how = clock.record_start(_meta(start=636.0, close=636.0))
    assert t.timestamp() == pytest.approx(1636.0)
    assert how.startswith("start check")


def test_a_clock_that_synced_mid_run_is_corrected_from_the_close():
    """The case that matters: booted slow, no network at the start, the
    clock stepped forward 636 s mid-run, the close check reads ~0."""
    m = _meta(close=0.01, step=636.0)
    m["clock"]["start"] = {"error": "OSError: no route"}
    t, how = clock.record_start(m)
    assert t.timestamp() == pytest.approx(1636.01)
    assert "close check" in how and "stepped +636.0 s" in how


def test_no_check_answered_is_unknown():
    m = _meta()
    m["clock"]["start"] = m["clock"]["close"] = {"error": "x"}
    assert clock.record_start(m) is None
    assert clock.record_start({}) is None
