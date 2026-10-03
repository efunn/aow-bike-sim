"""bench/force_drop.py's cam logic, against a fake bus: no servo, no sensor.

The drop rig's XL330 turns a snail cam that must only ever move FORWARD (a
backward move drives the follower into a step's undercut face), lift onto the
right top flat, and label each drop by the step that released it. These pin
that, and the startup checks, without hardware.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bench"))
import force_drop as fd  # noqa: E402

pytestmark = pytest.mark.pure

SEG = fd.TICKS // 4          # four drops per turn
DEG = fd.TICKS / 360.0


class FakeBus:
    """One servo whose shaft jumps straight to each goal, in multi-turn ticks."""

    def __init__(self, pos, dxl_id=1, latched=0):
        self.id, self.pos, self.latched = dxl_id, pos, latched
        self.reg = {"Current Limit": 1750, "Torque Enable": 0,
                    "Position P Gain": 800, "Position I Gain": 0, "Position D Gain": 0}
        self.goals, self.writes = [], []

    def hardware_errors(self, ids=None):
        return {self.id: self.latched} if self.latched else {}

    def prepare(self, ids=None):
        self.reg["Torque Enable"] = 0

    def read_raw(self, i, name):
        if name == "Present Position":
            return self.pos & 0xFFFFFFFF
        if name == "Hardware Error Status":
            return self.latched
        return self.reg[name]

    def write_raw(self, i, name, raw):
        self.writes.append((name, raw))
        if name == "Goal Position":
            goal = fd.signed(raw, 4)
            self.goals.append(goal)
            if self.reg["Torque Enable"]:
                self.pos = goal
        elif name == "Operating Mode":
            self.reg[name] = raw
            self.reg["Position P Gain"] = 640        # the mode write resets the gains
        else:
            self.reg[name] = raw


def cam(pos, direction=-1, index=1500, **kw):
    bus = FakeBus(pos, **kw)
    return fd.Cam(bus, 1, direction, index, 4, park_deg=12, margin_deg=30, ma=300,
                  gains={"Position P Gain": 400, "Position D Gain": 0}), bus


def phase(c, pos):
    """Forward distance from step 0, within one turn."""
    return ((pos - c.step(0)) * c.dir) % fd.TICKS


def assert_forward(bus, direction):
    d = np.diff(np.array(bus.goals) * direction)
    assert (d >= -fd.POS_TOL).all(), f"a backward goal: {bus.goals}"


@pytest.mark.parametrize("direction", (-1, 1))
def test_homing_from_a_ramp_goes_forward_to_the_next_top_flat(direction):
    """Mid-ramp of segment 2: the next step is 2, and nothing is crossed."""
    start = 1500 + direction * (SEG + 40 * DEG)          # 40 deg into step 1's segment
    c, bus = cam(round(start), direction)
    crossed = c.home()
    assert not crossed and c.k == 2
    assert phase(c, bus.pos) == pytest.approx(2 * SEG - 30 * DEG, abs=1)
    assert bus.reg["Torque Enable"] == 1
    assert_forward(bus, direction)


@pytest.mark.parametrize("direction", (-1, 1))
def test_homing_between_a_top_flat_and_its_step_passes_the_step(direction):
    """Within the run-up of step 1: going forward to a top flat crosses step 1
    (an unrecorded drop) and lands on step 2's."""
    start = 1500 + direction * (SEG - 10 * DEG)
    c, bus = cam(round(start), direction)
    assert c.home() is True and c.k == 2
    assert_forward(bus, direction)


def test_each_fire_lifts_holds_and_drops_one_step_forward():
    c, bus = cam(1500 - 5)                                # just past step 0, parked
    c.home()
    ks = [c.fire(hold_s=0.0)[0] for _ in range(6)]
    assert ks == [1, 2, 3, 4, 5, 6]                       # labels: heights[k % 4]
    assert phase(c, bus.pos) == pytest.approx((6 * SEG + 12 * DEG) % fd.TICKS, abs=1)
    assert_forward(bus, -1)
    # every move carries its Goal Current, written just before its Goal
    # Position; the one exception is home()'s hold-where-it-stands, torque off
    names = [n for n, _ in bus.writes]
    before = [names[i - 1] for i, n in enumerate(names) if n == "Goal Position"]
    assert before[0] != "Goal Current"
    assert before[1:] == ["Goal Current"] * (1 + 2 * 6)


def test_stop_parks_forward_and_turns_torque_off():
    """From a top flat, stop goes ON through the step to the next dwell (never
    back down the ramp), then a confirmed Torque Enable 0."""
    c, bus = cam(1500 + SEG + round(40 * DEG))            # mid-ramp, dir -1
    c.home()
    top = bus.pos
    c.stop()
    assert (bus.pos - top) * -1 > 0                       # moved forward (dir -1)
    assert phase(c, bus.pos) == pytest.approx(c.k * SEG % fd.TICKS + 12 * DEG, abs=1)
    assert bus.writes[-1] == ("Torque Enable", 0)
    assert_forward(bus, -1)


def test_stop_when_parked_does_not_move():
    c, bus = cam(1500 - 5)
    c.home()
    c.fire(hold_s=0.0)
    parked, n_goals = bus.pos, len(bus.goals)
    c.stop()
    assert bus.pos == parked and bus.writes[-1] == ("Torque Enable", 0)
    assert all(g == parked for g in bus.goals[n_goals:])


def test_a_latched_error_refuses_with_its_name():
    with pytest.raises(RuntimeError, match="overload"):
        cam(1500, latched=32)


def test_the_mode_is_written_then_the_gains_and_read_back():
    """Writing Operating Mode resets the gains, so they go AFTER it."""
    _, bus = cam(1500)
    names = [n for n, _ in bus.writes]
    assert names.index("Operating Mode") < names.index("Position P Gain")
    assert bus.reg["Position P Gain"] == 400


def test_a_gain_that_does_not_stick_raises():
    class Stubborn(FakeBus):
        def write_raw(self, i, name, raw):
            if name != "Position P Gain":
                super().write_raw(i, name, raw)
    with pytest.raises(RuntimeError, match="did not stick"):
        fd.Cam(Stubborn(1500), 1, -1, 1500, 4, 12, 30, 300, gains={"Position P Gain": 400})


def test_the_edge_speed_is_a_fit_over_the_window():
    rng = np.random.default_rng(0)
    t = np.cumsum(rng.uniform(0.0005, 0.0025, 200))
    ticks = np.round(400.0 * DEG * t)                     # 400 deg/s
    v, n = fd.edge_speed(list(zip(t, ticks)), at_tick=ticks[100])
    assert v == pytest.approx(400.0, rel=0.02) and n >= 10
    assert math.isnan(fd.edge_speed(list(zip(t[:3], ticks[:3])), ticks[1])[0])
