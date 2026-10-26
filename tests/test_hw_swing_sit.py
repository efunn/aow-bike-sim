"""Lie down / get up on the real bike: the station's keys, the wire, and
`BikeRunner`'s half of control/swing_sit.py -- with no bike and no bus.

The sequence itself is tested against the sim (analysis/moves/swing_sit.py);
what is new on the bike is the plumbing around it: two press counts on the
wire, the fall guard stood aside while it runs, the drives held while the
policy is off, and everything that must END it (a disengage, a lost link,
the operator taking the wings back).

WHAT INVALIDATES THIS FILE: the UDP command struct (lie_n / up_n), the
SwingSit phases or its Command, or BikeRunner's _sit_* methods.
"""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from aow_sim.control.swing_sit import SwingSit
from aow_sim.hw import run_bike as RB
from aow_sim.hw import telemetry as T
from aow_sim.hw.ground import OperatorState

pytestmark = pytest.mark.pure

STOW = math.pi


# --- the station and the wire -----------------------------------------------

def test_lie_and_up_presses_round_trip_and_a_v3_command_has_neither():
    op = OperatorState(travel_rad=2.0)
    op.key("LIE_DOWN"), op.key("z"), op.key("GET_UP")
    got = T.decode_command(T.encode_command(op.packet()))
    assert (got["lie_n"], got["up_n"]) == (2, 1)
    v3 = T._CMD_V3.pack(3, 5, 1, 0, 370, 0, 12340, T.ABSENT, T.ABSENT)
    got = T.decode_command(v3)
    assert got["stop_n"] == 1 and "lie_n" not in got and "up_n" not in got


def test_a_sit_press_drops_the_stations_wing_goal_and_stops():
    """Otherwise the packet keeps carrying the old goal and the bike snaps
    the crank back to it the moment the sequence lets go."""
    op = OperatorState(travel_rad=2.0)
    op.key("9"), op.key("w")
    op.sync({"psi": 0.7, "state": "engaged"})
    assert op.packet()["righting_rad"] == pytest.approx(STOW + 2.0)
    op.key("LIE_DOWN")
    pkt = op.packet()
    assert "righting_rad" not in pkt and op.pos == 0
    assert op.v == 0.0 and pkt["psi_cmd"] == pytest.approx(0.7)


def test_wing_keys_are_ignored_mid_sequence_and_take_over_once_lying():
    op = OperatorState(travel_rad=2.0)
    op.sync({"state": "sit"})
    op.key("9")
    op.slew_righting(0.5, measured=STOW)
    assert op.righting is None
    op.sync({"state": "lying"})
    op.key("9")
    assert op.righting == pytest.approx(STOW + 2.0)


# --- the bike's half ------------------------------------------------------------

class _Bus:
    policy_ids = (1, 2, 3)
    id_right = 4

    def __init__(self):
        self.calls, self.goal = [], None

    def set_righting(self, g):
        self.goal = g

    def set_righting_current(self, n):
        self.calls.append(("current", int(n)))
        return int(n)

    def arm(self, ids):
        self.calls.append(("arm", tuple(ids)))

    def torque(self, on, ids):
        self.calls.append(("torque", on, tuple(ids)))


def _runner(roll=0.0, state="engaged"):
    r = SimpleNamespace(
        sit=SwingSit(np.radians(116.9), hand_off=False,
                     rest_deg=RB.LIE_REST_DEG), bus=_Bus(),
        guard=RB.FallGuard(),
        torque=True, hold=False, controller="policy",
        link=SimpleNamespace(peer=("192.168.0.114", 5000)), _link_lost=False,
        _pack_low=False, _servo_fault=False, _errs={}, pack=RB.PackMonitor(),
        _righting_current=600, _righting_pos=STOW, _r_stow=STOW, _r_sign=1.0,
        _roll=math.radians(roll), _roll_rate=0.0, _yaw=0.3,
        data=SimpleNamespace(time=0.0, qvel=np.zeros(6), qpos=np.zeros(6),
                             ctrl=np.zeros(3)),
        ctl=SimpleNamespace(_sj=4),
        aid={"drive_a": 0, "drive_b": 1, "steer": 2},
        _sit_gc=None, _sit_steer=None, _sit_off=False, _sit_psi=0.0,
        _sit_said="riding", _lie_seen=None, _up_seen=None, said=[], engaged=[])
    r.guard.state = state
    r._event = lambda kind, text: r.said.append((kind, text))
    r._engage = lambda fresh_read=True: r.engaged.append(fresh_read)
    for name in ("_sit_owns", "_state", "_sit_from", "_sit_start",
                 "_sit_release", "_sit_step", "_hold_reason"):
        setattr(r, name, getattr(RB.BikeRunner, name).__get__(r))
    r._sit_from(0, 0)                    # first contact: no press
    return r


def _run(r, until, roll_of=None, limit=3000):
    """Tick `_sit_step` until `until(r)`, the bike's roll scripted by phase;
    the crank reads back as commanded. -> the commands seen."""
    cmds = []
    for _ in range(limit):
        if until(r):
            return cmds
        if roll_of is not None:
            r._roll = math.radians(roll_of(r.sit.phase))
        cmds.append(r._sit_step())
        if r.bus.goal is not None:
            r._righting_pos = r.bus.goal
        r.data.time += 0.01
    raise AssertionError(f"stuck in {r.sit.phase}")


def test_a_lie_down_runs_to_lying_with_the_guard_stood_aside():
    r = _runner()
    r._sit_from(1, 0)
    assert r._state() == "sit" and r.sit.phase == "stop"
    lean = {"stop": 0, "deploy": 0, "kick": 4, "settle": 6, "lower": 40,
            "limp": 80, "lying": 80}
    cmds = _run(r, lambda r: r.sit.phase == "lying", lambda p: lean[p])
    off = [c for c in cmds if c is not None and not c.policy]
    assert off and any(c.diff for c in off), "the kick drives the differential"
    assert r._state() == "lying" and r.guard.state == "engaged"
    assert r.bus.calls[-1] == ("current", 0), "limp at the end"
    assert r._sit_gc == 600, "the operator's current is kept to restore"
    assert r._sit_steer is not None


def test_a_get_up_holds_the_steer_where_it_reads_not_at_a_stale_ctrl():
    """A fresh process has ctrl 0 whatever the wheel reads. Run 261026-163421
    started with the steer at +201 deg (straight, to the policy, which sees
    sin/cos 2 delta) and the get-up swung it ~200 deg to the ctrl's 0."""
    r = _runner(roll=-73.0, state="standby")
    r.data.qpos[r.ctl._sj] = math.radians(200.7)
    r._sit_from(0, 1)
    r._sit_step()
    assert r._sit_steer == pytest.approx(math.radians(200.7))


def test_lie_down_is_refused_and_said_unless_the_policy_is_driving():
    r = _runner(state="standby")
    r._sit_from(1, 0)
    assert r.sit.phase == "riding"
    assert "ignored" in r.said[-1][1]


def _propped(r):
    """From a cut on the right side: get up and run to `propped`."""
    r._sit_from(0, 1)
    assert r.sit.phase == "lift" and r.guard.state == "engaged"
    assert ("arm", (1, 2, 3)) in r.bus.calls
    _run(r, lambda r: r.sit.phase == "propped",
         lambda p: 80.0 if r._righting_pos - STOW < 1.5 else 6.0)


def test_get_up_stops_propped_with_the_drives_limp_and_waits():
    r = _runner(roll=80.0, state="cut")
    _propped(r)
    cmd = r._sit_step()
    assert cmd.limp and not cmd.policy
    assert r._state() == "propped" and r.guard.state == "standby"
    assert r.bus.calls[-1] == ("torque", False, (1, 2, 3))
    assert r.bus.goal == pytest.approx(STOW + np.radians(110.0))
    for _ in range(500):                 # 5 s: nothing hands it over
        assert r._sit_step().limp
        r.data.time += 0.01
    assert r.engaged == [] and r._state() == "propped"


def test_enter_hands_a_propped_bike_to_the_policy_and_folds_the_blade():
    r = _runner(roll=80.0, state="cut")
    _propped(r)
    r._sit_step()
    r.guard.request_rearm()              # what _rearm_from does on Enter
    cmd = r._sit_step()
    assert cmd.policy and r.sit.phase == "balance"
    assert r.guard.state == "engaged" and r.bus.calls[-1] == ("arm", (1, 2, 3))
    assert r.engaged == [False], "a clean engage on this tick's reading"
    _run(r, lambda r: r.sit.phase == "riding", lambda p: 0.0)
    assert r._state() == "engaged" and r._sit_gc is None
    assert r.bus.calls[-1] == ("current", 600), "the operator's current back"
    assert r.bus.goal == pytest.approx(STOW, abs=1e-6)


def test_enter_while_propped_but_still_swinging_is_refused_and_said():
    r = _runner(roll=80.0, state="cut")
    _propped(r)
    r._sit_step()
    r._roll = math.radians(20.0)
    r.guard.request_rearm()
    r._sit_step()
    assert r._state() == "propped" and "engage ignored" in r.said[-1][1]
    assert not r.guard.consent, "spent: a stale Enter must not fire later"


def test_get_up_is_refused_while_the_policy_is_riding():
    r = _runner()
    r._sit_from(0, 1)
    assert r.sit.phase == "riding" and "policy is driving" in r.said[-1][1]


# --- powered up resting on a blade (2026-10-26) ------------------------------

def _resting(crank_deg=-100.0, roll=-9.0):
    """Standby at power-up, the bike already standing on the left blade."""
    r = _runner(roll=roll, state="standby")
    r._righting_pos = STOW + math.radians(crank_deg)
    return r


def test_lie_down_from_resting_on_a_blade_never_runs_the_policy():
    r = _resting()
    r._sit_from(1, 0)
    assert r.sit.phase == "lower" and r.sit.side == -1.0
    assert ("arm", (1, 2, 3)) in r.bus.calls, "drives held, not limp, lowering"
    first = r._sit_step()
    assert first.crank == pytest.approx(math.radians(-100.0), abs=0.01), \
        "ramped from where the crank IS, not snapped to the lean"
    cmds = _run(r, lambda r: r.sit.phase == "lying", lambda p: -40.0)
    assert all(c is None or not c.policy for c in cmds)
    assert r.engaged == [] and r._state() == "lying"


def test_get_up_from_resting_props_on_the_blade_it_is_on():
    r = _resting()
    r._sit_from(0, 1)
    assert r.sit.phase == "lift" and r.sit.side == -1.0
    _run(r, lambda r: r.sit.phase == "propped", lambda p: -6.0)
    assert r.bus.goal == pytest.approx(STOW - math.radians(110.0))
    assert r.engaged == []


def test_lie_down_from_propped_lowers_without_the_policy():
    r = _resting()
    r._sit_from(0, 1)
    _run(r, lambda r: r.sit.phase == "propped", lambda p: -6.0)
    r._sit_step()                        # limp
    r._sit_from(1, 1)
    assert r.sit.phase == "lower" and r.guard.state == "engaged"
    _run(r, lambda r: r.sit.phase == "lying", lambda p: -40.0)
    assert r.engaged == []


def test_lie_down_from_standby_upright_with_the_wings_stowed_is_refused():
    r = _resting(crank_deg=0.0, roll=1.0)
    r._sit_from(1, 0)
    assert r.sit.phase == "riding" and "not resting on a blade" in r.said[-1][1]
    r = _resting(crank_deg=-100.0, roll=+5.0)    # leaning AWAY from the blade
    r._sit_from(1, 0)
    assert r.sit.phase == "riding"


def test_a_disengage_mid_sequence_ends_it_and_holds_the_crank():
    r = _runner()
    r._sit_from(1, 0)
    _run(r, lambda r: r.sit.phase == "kick", lambda p: 0.0)
    r._righting_pos = STOW + 1.0
    r.guard.standby()                    # what _stop_from does to the guard
    assert r._sit_step() is None
    assert r.sit.phase == "riding" and r._state() == "standby"
    assert r.bus.goal == pytest.approx(STOW + 1.0), "held where it is"
    assert r.bus.calls[-1] == ("current", 600)


def test_taking_the_wings_back_while_lying_limps_the_drives():
    """The station sends a goal again only after 9/4 or the triggers; out of
    `lying` that hands the wings back and leaves the bike cut, on its side."""
    r = _runner()
    r._sit_from(1, 0)
    lean = {"stop": 0, "deploy": 0, "kick": 4, "settle": 6, "lower": 40,
            "limp": 80, "lying": 80}
    _run(r, lambda r: r.sit.phase == "lying", lambda p: lean[p])
    r._sit_release("wings taken back by the operator")
    assert r.guard.state == "cut" and r._state() == "cut"
    assert ("torque", False, (1, 2, 3)) in r.bus.calls
    assert r.bus.calls[-2:] == [("current", 600), ("torque", False, (1, 2, 3))]


def test_apply_operator_keeps_the_station_off_the_wings_mid_sequence():
    """Read from the source, as test_every_field_the_station_sends_is_read
    does: the sit counts are read, and a station goal is not applied while
    the sequence owns the bike."""
    import inspect
    src = inspect.getsource(RB.BikeRunner._apply_operator)
    assert '"lie_n"' in src and '"up_n"' in src
    assert src.index("_sit_owns()") < src.index('set_righting(c["righting_rad"])')


def test_a_lie_down_ends_with_the_crank_past_centre_and_get_up_lifts_the_down_side():
    """The lower runs through centre to LIE_REST_DEG the other way (no drop
    when the current goes); getting up from there must lift the blade the
    bike LIES on, not the one the crank is out towards."""
    r = _runner()
    r._sit_from(1, 0)                    # right side
    lean = {"stop": 0, "deploy": 0, "kick": 4, "settle": 6, "lower": 40,
            "limp": 70, "lying": 70}
    _run(r, lambda r: r.sit.phase == "lying", lambda p: lean[p])
    assert r.bus.goal == pytest.approx(STOW - math.radians(RB.LIE_REST_DEG))
    r._sit_from(1, 1)
    assert r.sit.phase == "lift" and r.sit.side == +1.0
