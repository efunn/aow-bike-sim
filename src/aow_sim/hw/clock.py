"""What the Pi's wall clock was worth during a run: one SNTP query, no daemon.

THE PROBLEM. The Pi has no battery-backed clock. At power-up it resumes from
the time it last saved and only corrects when systemd-timesyncd reaches a
server -- 7 min after boot on 2026-10-27, and run 261027-162626's `created`
read ~10.6 min slow (found by lining its link drops up with the Mac's sleeps,
which is not a method). So a record's start time cannot be trusted to sync
footage against, and the system journal is kept in RAM, so a later boot
cannot say how wrong an earlier one was.

THE FIX IS TO ASK, NOT TO WAIT. `check()` sends one SNTP packet to a public
pool and reports the local clock's offset from it. If there is no network it
gives up and says why (`error`); nothing waits on it for longer than the
timeout. `RunRecorder` runs one at the start and one at close, each on its own
thread, so the control loop never sees a DNS lookup.

`record_start(meta)` turns those into the record's true wall-clock start:
from the START check if it answered, else from the CLOSE check carried back
over the monotonic clock -- which is the case that matters, since a bike that
powered up without a synced clock often has one by the end of the run.
"""
from __future__ import annotations

import datetime as _dt
import socket
import struct
import subprocess
import threading
import time

SERVER = "pool.ntp.org"
TIMEOUT_S = 1.5
_NTP_EPOCH = 2208988800          # 1900-01-01 to 1970-01-01, seconds


def _ntp_time(b: bytes) -> float:
    sec, frac = struct.unpack("!II", b)
    return sec - _NTP_EPOCH + frac / 2 ** 32


def sntp(server: str = SERVER, timeout: float = TIMEOUT_S, port: int = 123) -> dict:
    """One query. -> {offset_s: server - local, rtt_s, server}; raises OSError
    (no route, no DNS, timeout) or ValueError (a reply that is not one)."""
    addr = socket.getaddrinfo(server, port, socket.AF_INET, socket.SOCK_DGRAM)[0][4]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(timeout)
        t1 = time.time()
        s.sendto(b"\x23" + bytes(47), addr)       # LI 0, version 4, client
        data, _ = s.recvfrom(512)
        t4 = time.time()
    if len(data) < 48 or data[0] & 0x07 != 4 or data[1] == 0:
        raise ValueError("not a server reply (wrong mode, or stratum 0)")
    rx, tx = _ntp_time(data[32:40]), _ntp_time(data[40:48])
    return {"offset_s": ((rx - t1) + (tx - t4)) / 2,
            "rtt_s": (t4 - t1) - (tx - rx), "server": addr[0]}


def synced() -> bool | None:
    """systemd's own verdict (`timedatectl`), or None where there is none."""
    try:
        out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized",
                              "--value"], capture_output=True, text=True,
                             timeout=1.0).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None
    return {"yes": True, "no": False}.get(out)


def check(server: str = SERVER, timeout: float = TIMEOUT_S) -> dict:
    """-> {wall, mono, synced, offset_s, rtt_s, server} or, with no answer,
    {wall, mono, synced, error}. `wall` and `mono` are taken together, so a
    later reader can carry this check to any other moment of the run."""
    out = {"wall": time.time(), "mono": time.monotonic(), "synced": synced()}
    try:
        out.update(sntp(server, timeout))
    except (OSError, ValueError) as e:
        out["error"] = f"{type(e).__name__}: {e}"
    return out


class Background:
    """`check()` on a daemon thread; `result(wait)` joins for at most `wait`."""

    def __init__(self, fn=check):
        self._out: dict = {}
        self._t = threading.Thread(target=lambda: self._out.update(fn()),
                                   daemon=True, name="clock-check")
        self._t.start()

    def result(self, wait: float = TIMEOUT_S + 0.5) -> dict | None:
        self._t.join(wait)
        return dict(self._out) if self._out else None


def record_start(meta: dict) -> tuple[_dt.datetime, str] | None:
    """The record's row-0 wall time, corrected, and where it came from; None
    if no check answered (`created` is then all there is)."""
    c = meta.get("clock") or {}
    t0_wall, t0_mono = c.get("t0_wall"), c.get("t0_mono")
    if t0_wall is None:
        return None
    for name in ("start", "close"):
        k = c.get(name) or {}
        if "offset_s" not in k:
            continue
        # The local clock at row 0, as the check's own clock would have read
        # it: back from the check over the monotonic clock, so a step the
        # wall clock took between the two (timesyncd catching up mid-run)
        # does not leak in.
        local0 = k["wall"] - (k["mono"] - t0_mono)
        t = local0 + k["offset_s"]
        how = (f"{name} check, {k['server']}, offset {k['offset_s']:+.2f} s, "
               f"rtt {k['rtt_s'] * 1e3:.0f} ms")
        if abs(t0_wall - local0) > 0.5:
            how += f"; the Pi's clock stepped {local0 - t0_wall:+.1f} s mid-run"
        return _dt.datetime.fromtimestamp(t).astimezone(), how
    return None
