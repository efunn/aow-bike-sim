"""`drivetrain_bench.py --hold-steer`: the steer servo held straight during a
drivetrain test on the assembled bike, against a fake bus.

What it guards: the steer must stay OUT of the drive ids (every default-ids
call -- torque, frames, the watchdog -- would otherwise drive it as a third
drive servo), its goal must be the straight-ahead angle nearest the fork,
written AFTER torque on (the fake, like the servo, drops a goal written with
torque off: control_tables/README.md), and whatever mode and gains it had
must come back afterwards.
"""

import sys

import pytest

sys.path.insert(0, "analysis/servo")
import drivetrain_bench as db  # noqa: E402
from aow_sim.hw.control_table import table_by_name  # noqa: E402
from aow_sim.hw.dynamixel import MODE_EXTENDED_POSITION, MODE_VELOCITY  # noqa: E402

pytestmark = pytest.mark.analysis

XC330 = table_by_name("xc330_t181")
XC430 = table_by_name("xc430_w150")


class FakeBus:
    """Records every register write; reads come from `regs`."""

    def __init__(self, ids=(101, 102), steer_mode=MODE_VELOCITY, present=2100):
        self.ids = tuple(ids)
        self.tables = {i: XC430 for i in ids}
        self.regs = {103: {"Operating Mode": steer_mode, "Present Position": present,
                           "Position P Gain": 640, "Position I Gain": 5,
                           "Position D Gain": 7, "Hardware Error Status": 0}}
        self.log = []
        self.torqued = set()

    def discover(self, ids=None):
        ids = tuple(ids)
        self.ids, self.tables = ids, {i: XC330 for i in ids}   # as the real one: clobbers
        return dict(self.tables)

    def read_raw(self, i, name):
        return self.regs[i][name]

    def write_raw(self, i, name, raw):
        self.log.append(("write", i, name, raw))
        if name.startswith("Goal") and i not in self.torqued:
            return                               # acknowledged, dropped
        self.regs[i][name] = raw

    def torque(self, on, ids=None):
        ids = tuple(ids if ids is not None else self.ids)
        self.log.append(("torque", ids, on))
        self.torqued = (self.torqued | set(ids)) if on else (self.torqued - set(ids))

    def prepare(self, ids=None, **kw):
        self.log.append(("prepare", tuple(ids if ids is not None else self.ids)))


class Args:
    steer_gains = {"Position P Gain": 900, "Position I Gain": 0, "Position D Gain": 0}
    steer_zero_deg = 180.0


@pytest.mark.parametrize("present, goal", [
    (2100, 2048),        # 4.6 deg past straight -> back to it
    (2048, 2048),
    (1000, 0),           # nearer the other end of the wheel: 0 deg is straight too
    (3100, 4096),        # the next turn up, not a 270 deg unwind
    (-1100, -2048),      # extended position goes negative
    (6150, 6144),
])
def test_the_goal_is_the_straight_ahead_angle_nearest_the_fork(present, goal):
    assert db._steer_straight(present, 180.0) == goal


def test_attaching_the_steer_leaves_the_drive_ids_alone():
    bus = FakeBus()
    db._attach(bus, 103)
    assert bus.ids == (101, 102)
    assert set(bus.tables) == {101, 102, 103}
    assert bus.tables[101] is XC430 and bus.tables[103] is XC330


def test_hold_on_sets_mode_and_gains_then_torque_then_ramps_the_goal():
    bus = FakeBus()
    orig = dict(bus.regs[103])
    hold = db._steer_hold_on(bus, 103, orig, Args, sleep=lambda _s: None)
    writes = [e for e in bus.log if e[0] == "write"]
    assert writes[0] == ("write", 103, "Operating Mode", MODE_EXTENDED_POSITION)
    # gains AFTER the mode (a mode write resets them), then torque, then goals
    assert [w[2] for w in writes[1:4]] == ["Position P Gain", "Position I Gain",
                                           "Position D Gain"]
    t_on = bus.log.index(("torque", (103,), True))
    goals = [e[3] for e in bus.log[t_on:] if e[0] == "write" and e[2] == "Goal Position"]
    assert not [e for e in bus.log[:t_on] if e[0] == "write" and e[2] == "Goal Position"]
    assert goals[0] == 2100 and goals[-1] == 2048 == hold["goal"]
    assert all(b <= a for a, b in zip(goals, goals[1:]))       # one way, no overshoot
    assert max(abs(b - a) for a, b in zip(goals, goals[1:])) <= 4   # smooth, not a snap
    assert bus.regs[103]["Goal Position"] == 2048
    assert not any(e[0] == "write" and e[2] == "Bus Watchdog" for e in bus.log)


def test_hold_off_drops_torque_and_restores_mode_then_gains():
    bus = FakeBus()
    orig = dict(bus.regs[103])
    hold = db._steer_hold_on(bus, 103, orig, Args, sleep=lambda _s: None)
    bus.log.clear()
    db._steer_hold_off(bus, 103, orig, hold)
    assert bus.log[0] == ("torque", (103,), False)
    assert [e[2:] for e in bus.log[1:]] == [
        ("Operating Mode", MODE_VELOCITY), ("Position P Gain", 640),
        ("Position I Gain", 5), ("Position D Gain", 7)]
    assert hold["present_after"] == 2100


def test_a_steer_already_in_flight_mode_keeps_it():
    bus = FakeBus(steer_mode=MODE_EXTENDED_POSITION)
    orig = dict(bus.regs[103])
    hold = db._steer_hold_on(bus, 103, orig, Args, sleep=lambda _s: None)
    db._steer_hold_off(bus, 103, orig, hold)
    assert not any(e[0] == "write" and e[2] == "Operating Mode" for e in bus.log)
