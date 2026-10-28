"""The wired Xbox One pad: report parsing, the sign convention into the
bike's command, who owns the velocity, and the righting servo's slew.

Invalidated by: control/gamepad_usb.py's report layout, gamepad.from_state,
run_drive._PadInput, hw/ground.OperatorState's righting keys. No USB needed:
the reports here are built the way the pad sends them, one of them captured
from the PDP at rest (2026-10-19).
"""

from __future__ import annotations

import math
import struct

import numpy as np
import pytest

from aow_sim.control import gamepad as gp
from aow_sim.control.gamepad_usb import PadState, parse_input

pytestmark = pytest.mark.pure

# The PDP at rest, as read off the bus.
IDLE = bytes.fromhex("20 00 03 0e 00 00 00 00 00 00 ec 00 30 01 04 03 9d 04")


def _report(b4=0, b5=0, lt=0, rt=0, lx=0, ly=0, rx=0, ry=0) -> bytes:
    return bytes([0x20, 0x00, 0x07, 0x0E, b4, b5]) + struct.pack(
        "<HHhhhh", lt, rt, lx, ly, rx, ry)


def test_the_captured_idle_report_reads_as_centred():
    s = parse_input(IDLE)
    assert s.buttons == frozenset()
    assert max(abs(s.lx), abs(s.ly), abs(s.rx), abs(s.ry)) < 0.04
    assert s.lt == s.rt == 0.0
    # and it is inside the deadzone the bike's command applies
    p = gp.from_state(s)
    assert gp._dz(p.ly) == gp._dz(p.lx) == gp._dz(p.rx) == 0.0


def test_axes_triggers_and_buttons_land_where_the_standard_layout_says():
    s = parse_input(_report(b4=0b0001_0100, b5=0b0010_0001, lt=1023, rt=512,
                            lx=32767, ly=-32768, rx=-16384, ry=8192))
    assert s.buttons == {"START", "A", "DPAD_UP", "RB"}
    assert (s.lx, s.ly) == (1.0, -1.0)
    assert s.rx == pytest.approx(-0.5, abs=1e-4)
    assert s.ry == pytest.approx(0.25, abs=1e-4)
    assert s.lt == 1.0 and s.rt == pytest.approx(0.5, abs=1e-3)


def test_every_button_bit_has_one_name():
    names = set()
    for b4 in range(2, 8):
        names |= parse_input(_report(b4=1 << b4)).buttons
    for b5 in range(8):
        names |= parse_input(_report(b5=1 << b5)).buttons
    assert len(names) == 14
    assert parse_input(_report(), guide=True).buttons == {"GUIDE"}


def test_a_report_that_is_not_input_is_ignored():
    assert parse_input(bytes.fromhex("03 20 02 04 80 00 00 00")) is None


def test_stick_right_crabs_right_and_turns_clockwise():
    """The standard layout is +x RIGHT; the bike's command is +lateral LEFT
    and +heading rate CCW. Pushing right must mean right."""
    v_max, crab_max, turn = 1.2, 0.48, 1.2
    v_lon, v_lat, psi = gp.apply(gp.from_state(PadState(lx=1.0, rx=1.0)),
                                 0.0, 0.1, v_max, crab_max, turn)
    assert v_lat < 0 and psi < 0                  # right, clockwise
    v_lon, _, _ = gp.apply(gp.from_state(PadState(ly=1.0)), 0.0, 0.1,
                           v_max, crab_max, turn)
    assert v_lon == pytest.approx(v_max)           # stick up = forward


class _FakePad:
    name = "fake"

    def __init__(self):
        self.s, self.p = PadState(), []
        self.connected, self.closed = True, False

    def close(self):
        self.closed = True

    def state(self):
        return self.s

    def presses(self):
        out, self.p = self.p, []
        return out


def test_a_dropped_pad_is_reopened_and_its_stick_released(monkeypatch):
    """The Mac slept, the read died: the next frames reopen it, once a
    second, and a stick held when it went does not stay held."""
    import aow_sim.control.gamepad_usb as gu
    from aow_sim.run_drive import _PadInput
    old, new = _FakePad(), _FakePad()
    pin = _PadInput(old, {"A": ord("5")})
    old.s = PadState(ly=0.5)
    assert pin.drive()[0][0] > 0
    tries = []
    monkeypatch.setattr(gu, "open_gamepad",
                        lambda verbose=True: tries.append(1) or None)
    old.connected = False
    pin.dispatch(lambda k: None)
    pin.dispatch(lambda k: None)                   # inside the second: no retry
    assert len(tries) == 1 and old.closed
    monkeypatch.setattr(gu, "open_gamepad", lambda verbose=True: new)
    pin._t_try -= _PadInput.REOPEN_S
    new.p = ["A"]
    got = []
    pin.dispatch(got.append)
    assert pin.pad is new and got == [ord("5")]
    assert pin.drive()[0] is None                  # the new pad is idle


def test_the_pad_owns_the_velocity_only_while_deflected():
    from aow_sim.run_drive import _PadInput
    fake = _FakePad()
    pin = _PadInput(fake, {"A": ord("5")})
    assert pin.drive() == (None, 0.0)              # idle: the keyboard owns it
    fake.s = PadState(ly=0.5)
    vel, _ = pin.drive()
    assert vel[0] > 0 and vel[1] == 0.0
    fake.s = PadState()
    assert pin.drive()[0] == (0.0, 0.0)            # released: one zero ...
    assert pin.drive()[0] is None                  # ... then hands back
    got = []
    fake.p = ["A", "GUIDE"]
    pin.dispatch(got.append)
    assert got == [ord("5")]                       # unmapped buttons do nothing


def _op():
    from aow_sim.hw.ground import OperatorState
    return OperatorState(v_max=1.2, stow_rad=math.pi,
                         travel_rad=np.deg2rad(117.0))


def test_the_trigger_slew_starts_from_the_servo_and_stays_in_the_stroke():
    op = _op()
    start = math.pi + 0.3                         # the servo is not at stow
    op.slew_righting(0.01, measured=start)
    assert op.righting == pytest.approx(start + 0.01)
    for _ in range(1000):
        op.slew_righting(0.01)
    assert op.righting == pytest.approx(math.pi + op.travel)
    assert op.pos == pytest.approx(1.0)


def test_9_and_4_go_to_the_next_stop_after_a_slew():
    op = _op()
    op.slew_righting(0.4 * op.travel, measured=math.pi)   # pos 0.4
    op.key("4")
    assert op.pos == 0                            # back to centre, not -1
    op.slew_righting(0.4 * op.travel)
    op.key("9")
    assert op.pos == 1
    op.key("9")
    assert op.pos == 1                            # clamped
    op.key("4"), op.key("4"), op.key("4")
    assert op.pos == -1


def test_bike_address_never_blocks_the_station_loop(monkeypatch):
    """`aowbike.local` with the Pi off took 5 s to fail on every sendto, so
    the mirror drew a frame every 5 s. The name now resolves on a thread:
    the loop reads `dest`, None until it is known, and never waits."""
    import time

    from aow_sim.hw.ground import BikeAddress

    assert BikeAddress("127.0.0.1", 9910).dest == ("127.0.0.1", 9910)

    calls = []

    def slow(self):
        calls.append(1)
        time.sleep(0.3)
        if len(calls) == 1:
            raise OSError("nodename nor servname provided")
        return "10.0.0.7"
    monkeypatch.setattr(BikeAddress, "_resolve", slow)
    t0 = time.perf_counter()
    a = BikeAddress("aowbike.local", 9910, retry_s=0.05, verbose=False)
    assert a.dest is None and time.perf_counter() - t0 < 0.05
    deadline = time.perf_counter() + 3.0
    while a.dest is None and time.perf_counter() < deadline:
        time.sleep(0.01)
    a.close()
    assert a.dest == ("10.0.0.7", 9910) and len(calls) >= 2


def test_link_text_says_what_the_station_is_waiting_for():
    """No telemetry yet: the mirror said nothing and looked frozen. The
    link rows now move (2 Hz dots) and echo the command being sent; they
    vanish while telemetry is fresh and come back, as LOST, when it stops."""
    import math

    from aow_sim.hw.telemetry import LINK_STALE_S, link_text

    k, v = link_text("aowbike.local", None, None, 0.0)
    assert k == "link\nsending" and v.startswith("resolving aowbike.local.")
    _, v2 = link_text("aowbike.local", None, None, 0.5)
    assert v2.split("\n")[0] != v.split("\n")[0]          # the dots move
    _, v = link_text("aowbike.local", ("10.0.0.7", 9910), None, 0.0,
                     (0.25, -0.1, math.radians(30)))
    assert "waiting for aowbike.local (10.0.0.7)" in v
    assert "v +0.25" in v and "crab -0.10" in v and "+30 deg" in v
    assert link_text("h", ("1.2.3.4", 1), 0.1, 0.0) == ("", "")
    assert "LOST 3 s ago" in link_text("h", ("1.2.3.4", 1),
                                       LINK_STALE_S + 2.0, 0.0)[1]
