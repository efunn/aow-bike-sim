"""Lie down / get up on the real bike: the station's keys, the wire, and
`BikeRunner`'s half of control/swing_sit.py -- with no bike and no bus.

The sequence itself is tested against the sim (analysis/moves/swing_sit.py);
what is new on the bike is the plumbing around it: two press counts on the
wire, the fall guard stood aside while it runs, the drives held while the
policy is off, and everything that must END it (a disengage, a lost link,
the operator taking the wings back).

WHAT INVALIDATES THIS FILE: the UDP command struct (lie_n / up_n / park_n), the
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
    id_a, id_b, id_right = 1, 2, 4

    def __init__(self):
        self.calls, self.goal = [], None

    def set_righting(self, g):
        self.goal = g

    @property
    def _righting_goal(self):
        return self.goal

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
                     rest_deg=RB.LIE_REST_DEG, lift_dps=RB.SIT_DPS,
                     lower_dps=RB.SIT_DPS, lower_map=RB.LIE_MAP,
                     lower_roll_dps=RB.LIE_ROLL_DPS,
                     lower_roll_acc=RB.LIE_ROLL_ACC, retry_s=None,
                     pump_s=RB.PUMP_S, prop_current=RB.PROP_CURRENT,
                     nudge_dps=RB.NUDGE_DPS,
                     lift_to_deg=tuple(116.9 if v == "stroke" else v for v in
                                       (RB.LIFT_TO_DEG["left"],
                                        RB.LIFT_TO_DEG["right"])),
                     nudge_to_deg=RB.PROP_ENGAGE_DEG - 0.5), bus=_Bus(),
        guard=RB.FallGuard(),
        torque=True, hold=False, controller="policy",
        link=SimpleNamespace(peer=("192.168.0.114", 5000)), _link_lost=False,
        _pack_low=False, _servo_fault=False, _righting_fault=False, _errs={},
        _health={}, pack=RB.PackMonitor(),
        _righting_current=600, _righting_pos=STOW, _r_stow=STOW, _r_sign=1.0,
        _roll=math.radians(roll), _roll_rate=0.0, _yaw=0.3,
        data=SimpleNamespace(time=0.0, qvel=np.zeros(6), qpos=np.zeros(6),
                             ctrl=np.zeros(3)),
        ctl=SimpleNamespace(_sj=4),
        aid={"drive_a": 0, "drive_b": 1, "steer": 2},
        _sit_gc=None, _sit_steer=None, _sit_steer_t=None, _sit_off=False,
        _sit_psi=0.0,
        _sit_said="riding", _lie_seen=None, _up_seen=None, _park_seen=None,
        _rearm_seen=None,
        _sit_engage_pending=False, said=[], engaged=[])
    r.guard.state = state
    r._event = lambda kind, text: r.said.append((kind, text))
    r._engage = lambda fresh_read=True: r.engaged.append(fresh_read)
    r._right_armed = False
    for name in ("_sit_owns", "_state", "_sit_from", "_sit_start",
                 "_sit_release", "_sit_step", "_sit_steer_step", "_sit_engage",
                 "_arm_righting", "_resting_crank", "_prop_if_resting",
                 "_rearm_from",
                 "_hold_reason", "_watch_errors"):
        setattr(r, name, getattr(RB.BikeRunner, name).__get__(r))
    r._sit_from(0, 0, 0)                 # first contact: no press
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


def test_the_held_steer_eases_to_the_nearer_straight_and_holds_propped():
    """From the first tick, lying or not (a bike lying on a turned wheel is
    only lifted off it), to the NEARER of 0 and 180 at SIT_STEER_DPS; the
    steer stays torqued while propped so the policy gets a straight wheel."""
    r = _runner(roll=-73.0, state="standby")
    r.data.qpos[r.ctl._sj] = math.radians(150.0)
    r._sit_from(0, 1)
    seen = []
    for _ in range(400):
        r._roll = math.radians(-73.0 if r.sit.phase == "lift"
                               and r._righting_pos - STOW > -1.5 else -6.0)
        r._sit_step()
        if r.bus.goal is not None:
            r._righting_pos = r.bus.goal
        seen.append(r._sit_steer)
        r.data.time += 0.01
    assert r.sit.phase == "propped"
    assert seen[20] > seen[0], "moving while still lying"
    assert r._sit_steer == pytest.approx(math.pi), "180, not 0: never a half turn"
    rate = np.abs(np.diff(seen)).max() / 0.01
    assert rate <= math.radians(RB.SIT_STEER_DPS) + 1e-9
    assert ("torque", False, (1, 2)) in r.bus.calls, "only the drives go limp"


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
    assert ("arm", (4,)) in r.bus.calls, "the righting servo, powered by the press"
    _run(r, lambda r: r.sit.phase == "propped",
         lambda p: 80.0 if r._righting_pos - STOW < 1.5 else 6.0)


def test_get_up_stops_propped_with_the_drives_limp_and_waits():
    r = _runner(roll=80.0, state="cut")
    _propped(r)
    cmd = r._sit_step()
    assert cmd.limp and not cmd.policy
    assert r._state() == "propped" and r.guard.state == "standby"
    assert r.bus.calls[-1] == ("torque", False, (1, 2)), "the steer stays on"
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
    r._roll_rate = math.radians(40.0)    # still: it would be nudged
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


def test_a_righting_fault_ends_a_get_up_cuts_and_refuses_the_next():
    """Run 261026-173521: the crank overloaded against a wall mid get-up and
    the sequence went on commanding it, dead, for a minute."""
    r = _runner(roll=-70.0, state="cut")
    r._sit_from(0, 1)
    _run(r, lambda r: r.data.time > 0.5, lambda p: -60.0)
    assert r.sit.phase == "lift"
    r._health = {"righting": {"Hardware Error Status": 32}}
    r._watch_errors()
    assert r.sit.phase == "riding" and r._state() == "cut"
    assert ("torque", False, (1, 2, 3)) in r.bus.calls
    assert r._sit_step() is None
    r._sit_from(0, 2)
    assert r.sit.phase == "riding" and "latched hardware error" in r.said[-1][1]


def test_releasing_a_propped_bike_takes_the_steer_off_too():
    """Propped leaves the steer on; letting go of it must not."""
    r = _runner(roll=80.0, state="cut")
    _propped(r)
    r._sit_step()
    r._sit_release("wings taken back by the operator")
    assert r.bus.calls[-1] == ("torque", False, (1, 2, 3))


def test_the_current_keys_reach_a_sequence_that_owns_the_bike():
    """[ / ] used to be dropped from a lie-down until riding again, so a
    get-up (always from lying) never saw them. They move the current the
    sequence scales from, and it is the one restored at the end."""
    r = _runner()
    r._rearm_from = r._stop_from = lambda n: None
    r._apply_operator = RB.BikeRunner._apply_operator.__get__(r)
    r._sit_from(1, 0)
    lean = {"stop": 0, "deploy": 0, "kick": 4, "settle": 6, "lower": 40,
            "limp": 80, "lying": 80}
    _run(r, lambda r: r.sit.phase == "lying", lambda p: lean[p])
    r._apply_operator({"righting_current": 400})        # ] / [ while lying
    assert r._sit_gc == 400 and r.sit.phase == "lying"
    r._sit_from(1, 1)                                    # get up
    r._sit_step()
    assert r.bus.calls[-1] == ("current", 400), "the lift runs at the new value"
    r._apply_operator({"righting_current": 380})        # and mid-lift
    r._sit_step()
    assert r.bus.calls[-1] == ("current", 380)


# --- park, and the nudge on Enter (2026-10-27) -------------------------------

def test_park_and_up_and_lie_presses_round_trip_and_a_v4_command_has_no_park():
    op = OperatorState(travel_rad=2.0)
    op.key("PARK"), op.key("p"), op.key("z")
    got = T.decode_command(T.encode_command(op.packet()))
    assert (got["park_n"], got["lie_n"], got["up_n"]) == (2, 1, 0)
    v4 = T._CMD_V4.pack(4, 5, 1, 0, 2, 3, 370, 0, 12340, T.ABSENT, T.ABSENT)
    got = T.decode_command(v4)
    assert (got["lie_n"], got["up_n"]) == (2, 3) and "park_n" not in got


@pytest.mark.parametrize("side", [1.0, -1.0])
def test_park_stops_propped_on_the_blade_and_enter_hands_it_back(side):
    r = _runner(roll=side * 0.5)
    r._sit_from(0, 0, 1)
    assert r.sit.phase == "stop" and r._state() == "sit" and r.sit.side == side
    lean = {"stop": 0, "deploy": 0, "kick": 4, "settle": 6}
    cmds = _run(r, lambda r: r.sit.phase == "propped",
                lambda p: side * lean[p])
    assert all(c is None or c.crank * side >= 0 for c in cmds), \
        "the blade on the side it leans, never through centre: no lower"
    r._roll = math.radians(side * 6.0)
    cmd = r._sit_step()
    assert cmd.limp and r.guard.state == "standby", "drives limp, as a get-up"
    assert r.bus.goal == pytest.approx(STOW + side * np.radians(110.0))
    assert r.bus.calls[-1] == ("torque", False, (1, 2))
    r.guard.request_rearm()
    assert r._sit_step().policy and r.sit.phase == "balance"
    assert r.guard.state == "engaged"


def test_park_is_refused_unless_the_policy_is_driving():
    r = _runner(state="standby")
    r._sit_from(0, 0, 1)
    assert r.sit.phase == "riding" and "not driving" in r.said[-1][1]


@pytest.fixture
def nudge_on(monkeypatch):
    """The nudge is OFF on the bike (RB.NUDGE); these tests switch it on."""
    monkeypatch.setattr(RB, "NUDGE", True)


def _sagging(side, sag_deg=17.0, per_deg=1.0):
    """Propped on `side`, leaning `sag_deg` onto the blade; every degree the
    crank goes past the lean brings the roll `per_deg` in."""
    r = _runner(roll=side * 80.0, state="cut")
    r._sit_from(0, 1)
    _run(r, lambda r: r.sit.phase == "propped",
         lambda p: side * (80.0 if side * (r._righting_pos - STOW) < 1.5
                           else 6.0))
    lean = math.radians(110.0)

    def roll(_phase):
        past = math.degrees(side * (r._righting_pos - STOW) - lean)
        return side * (sag_deg - per_deg * max(past, 0.0))
    r._roll = math.radians(roll(None))
    r._sit_step()
    return r, roll


@pytest.mark.parametrize("side", [1.0, -1.0])
def test_enter_on_a_sagging_prop_nudges_the_blade_out_then_engages(side, nudge_on):
    """Run 261026-175020: 29.6 s propped at -13.0 deg, crank 10.5 deg short of
    its goal at 0.33 A -- droop, servo still holding, and Enter refused."""
    r, roll = _sagging(side)
    r.guard.request_rearm()
    r._sit_step()
    assert r.sit.phase == "nudge" and any("nudging" in t for _, t in r.said)
    _run(r, lambda r: r.sit.phase != "nudge", roll)
    assert r.sit.nudged
    r._sit_step()                        # the pending engage
    assert r.sit.phase == "balance" and r.guard.state == "engaged"
    assert r.engaged == [False]
    assert abs(math.degrees(r._roll)) <= RB.PROP_ENGAGE_DEG - 0.5 + 1e-6
    past = math.degrees(side * r.sit._hold - math.radians(110.0))
    assert 0.0 < past <= 116.9 - 110.0 + 1e-6, "out, never past the stroke"
    assert r.sit._hold == pytest.approx(r.sit.side * r.sit._prop), \
        "the hand-off starts from the hold, which is where the crank was"


def test_the_nudge_is_slow(nudge_on):
    r, roll = _sagging(1.0)
    r.guard.request_rearm()
    r._sit_step()
    goals = [r.bus.goal]
    for _ in range(50):
        r._roll = math.radians(roll(None))
        r._sit_step()
        goals.append(r.bus.goal)
        r.data.time += 0.01
    rate = np.abs(np.diff(goals)).max() / 0.01
    assert 0 < rate <= math.radians(RB.NUDGE_DPS) + 1e-9


def test_a_nudge_that_cannot_get_it_in_goes_back_to_the_lean_and_says_so(nudge_on):
    r, roll = _sagging(-1.0, per_deg=0.0)      # nothing it does helps
    r.guard.request_rearm()
    r._sit_step()
    _run(r, lambda r: r.sit.phase != "nudge", roll)
    assert r.sit.failed == "nudge"
    r._sit_step()
    assert r._state() == "propped" and r.engaged == []
    assert "did not get it in" in r.said[-1][1]
    assert r.bus.goal == pytest.approx(STOW - math.radians(110.0))


def test_enter_on_a_prop_leaning_away_from_its_blade_is_not_nudged(nudge_on):
    r, _ = _sagging(1.0)
    r._roll = math.radians(-17.0)
    r.guard.request_rearm()
    r._sit_step()
    assert r.sit.phase == "propped" and "engage ignored" in r.said[-1][1]


def test_a_prop_lets_the_crank_go_and_the_hand_off_holds_it_where_it_is():
    """The linkage holds a prop unpowered (user, 2026-10-27); held, the servo
    drew 0.2-0.35 A for nothing. Enter then holds the blade from where the
    crank sits, not from the goal it was let go at."""
    r = _runner(roll=80.0, state="cut")
    _propped(r)
    for _ in range(150):                 # 1.5 s: past limp_s
        r._sit_step()
        r.data.time += 0.01
    assert r.bus.calls[-1] == ("current", 0)
    assert ("torque", False, (1, 2)) in r.bus.calls, "drives limp, steer on"
    r._righting_pos = STOW + math.radians(101.0)     # settled under load
    r._roll = math.radians(6.0)
    r.guard.request_rearm()
    n = len(r.bus.calls)
    cmd = r._sit_step()
    assert cmd.policy and r.sit.phase == "balance"
    assert cmd.crank == pytest.approx(math.radians(101.0), abs=1e-6)
    assert ("current", 600) in r.bus.calls[n:], "the current back for the hand-off"


@pytest.mark.parametrize("roll", [14.8, -13.6])
def test_enter_takes_a_prop_out_to_fifteen_degrees(roll):
    """The props measured by hand, crank unpowered (user, 2026-10-27): right
    +14.8 at crank +94, left -13.6 at -96.9. Outside the fall re-arm's 12,
    inside the prop's own window."""
    side = math.copysign(1.0, roll)
    r = _runner(roll=side * 80.0, state="cut")
    r._sit_from(0, 1)
    _run(r, lambda r: r.sit.phase == "propped",
         lambda p: side * (80.0 if side * (r._righting_pos - STOW) < 1.5 else 6.0))
    r._roll = math.radians(roll)
    r._sit_step()
    r.guard.request_rearm()
    assert r._sit_step().policy and r.sit.phase == "balance"


def test_enter_past_the_prop_window_is_refused_with_the_nudge_off(monkeypatch):
    monkeypatch.setattr(RB, "NUDGE", False)
    r, _ = _sagging(1.0, sag_deg=16.0)
    r.guard.request_rearm()
    r._sit_step()
    assert r.sit.phase == "propped" and "inside 15 deg" in r.said[-1][1]


@pytest.mark.parametrize("crank_deg,roll", [(-96.9, -13.6), (94.0, 14.8)])
def test_enter_on_a_bike_set_on_its_blade_by_hand_hands_off_as_from_a_prop(
        crank_deg, roll):
    """Standby, crank unpowered and moved by hand to a prop the user measured
    (2026-10-27): Enter takes it through the prop's 15 deg window with the
    blade held where it is, not the fall guard's 12."""
    r = _resting(crank_deg=crank_deg, roll=roll)
    r._rearm_from(0)                     # first contact: recorded only
    r._rearm_from(1)                     # Enter
    assert r.sit.phase == "propped" and r.guard.consent
    cmd = r._sit_step()
    assert cmd.policy and r.sit.phase == "balance" and r.guard.state == "engaged"
    assert cmd.crank == pytest.approx(math.radians(crank_deg), abs=1e-6)
    assert ("arm", (4,)) in r.bus.calls and ("arm", (1, 2, 3)) in r.bus.calls


def test_enter_with_the_wings_stowed_stays_the_fall_guards():
    r = _resting(crank_deg=0.0, roll=13.0)
    r._rearm_from(0)
    r._rearm_from(1)
    assert r.sit.phase == "riding" and r.guard.consent, "left to FallGuard's 12"


def test_a_let_go_prop_sagged_past_the_window_is_nudged_back_in_and_handed_off(
        nudge_on):
    """Run 261027-143131: the left get-ups settled at -15.3 to -15.7 deg with
    the crank let go and back-driven ~3 deg (-97.5 -> -94.4); powered, the
    same side propped at -12.3 to -13.0. Enter powers it and takes that back."""
    r = _runner(roll=-80.0, state="cut")
    r._sit_from(0, 1)
    _run(r, lambda r: r.sit.phase == "propped",
         lambda p: -80.0 if r._righting_pos - STOW > -1.5 else -6.0)
    for _ in range(150):                         # let go
        r._sit_step()
        r.data.time += 0.01
    r._righting_pos = STOW - math.radians(94.4)  # sagged
    r._roll = math.radians(-15.6)
    assert r.bus.calls[-1] == ("current", 0)
    r.guard.request_rearm()
    r._sit_step()
    assert r.sit.phase == "nudge" and ("current", 600) in r.bus.calls[-3:]

    def roll(_p):      # the crank's goal coming back out takes the sag back
        out = math.degrees(-(r.bus.goal - STOW)) - 94.4
        return -(15.6 - min(max(out, 0.0), 3.0))
    _run(r, lambda r: r.sit.phase != "nudge", roll)
    r._sit_step()
    assert r.sit.phase == "balance" and r.guard.state == "engaged"


@pytest.mark.parametrize("side,to", [(-1.0, 116.9), (1.0, 110.0)])
def test_the_lift_pushes_to_its_sides_target_and_a_left_style_stall_still_props(
        side, to):
    """The left lifts stalled at crank 96-99, roll 14-16, with the push
    easing off against a 110 goal (runs 261027-143131, -144242). The goal
    now ramps to the stroke; the stall is still a prop, not BLOCKED, since
    both checks are against the 110 lean."""
    r = _runner(roll=side * 80.0, state="cut")
    r._sit_from(0, 1)
    goals = []
    for _ in range(400):
        u = side * (r._righting_pos - STOW)
        r._roll = math.radians(side * (80.0 if u < 1.5 else 15.5))
        r._sit_step()
        goals.append(side * (r.bus.goal - STOW))
        # the crank follows its goal up to the left side's stall, no further
        r._righting_pos = STOW + side * min(side * (r.bus.goal - STOW),
                                            math.radians(96.9))
        r.data.time += 0.01
        if r.sit.phase != "lift":
            break
    assert max(goals) == pytest.approx(math.radians(to), abs=1e-6), \
        "left to the stroke, right to the lean (run 261027-145541)"
    assert r.sit.phase == "propped" and r.sit.failed is None
