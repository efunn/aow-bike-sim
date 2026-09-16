"""The onboard process's decision logic, with no bike and no bus.

Everything here is pure by construction: `FallGuard` has no clock and no I/O,
and `OperatorState` is the command struct with the keyboard taken off it. That
is deliberate -- these are the two pieces whose bugs are only visible on a
bike that is already falling over, so they are the two worth being able to
test at a desk.

WHAT INVALIDATES THIS FILE: a change to the fall cut/re-arm rules, or to the
UDP command struct either half of it agrees on. The two halves are
`BikeRunner._apply_command` and `ground.OperatorState.packet`; a field added to
one and not the other is silent, which is what `test_every_field_the_station_
sends_is_read` is for.
"""

import math

import pytest

from aow_sim.hw.ground import OperatorState
from aow_sim.hw.run_bike import FallGuard

pytestmark = pytest.mark.pure


# --- the fall guard -------------------------------------------------------

def test_hysteresis_is_required():
    """A re-arm angle at or above the cut angle has no hysteresis at all, so
    the guard would flip state on every wobble across the threshold."""
    with pytest.raises(ValueError, match="hysteresis"):
        FallGuard(cut_deg=60.0, rearm_deg=60.0)
    with pytest.raises(ValueError, match="hysteresis"):
        FallGuard(cut_deg=60.0, rearm_deg=75.0)


def test_it_cuts_past_the_angle_and_not_before():
    g = FallGuard(cut_deg=60.0)
    assert g.update(0.0, math.radians(59.0), 0.0) == ""
    assert g.state == "engaged"
    assert g.update(0.1, math.radians(61.0), 0.0) == "cut"
    assert g.state == "cut"
    # ... and it does not re-announce the cut every tick it stays down.
    assert g.update(0.2, math.radians(90.0), 0.0) == ""
    assert g.cuts == 1


def test_the_guard_and_the_sequencer_share_one_criterion():
    """Not "the numbers agree" — the same function, called from both.

    `control/righting.py`'s lift phase ends on `ready_for_policy`, and so does
    the guard's cut. If these ever become two implementations again, this is
    the test that should stop it.
    """
    import inspect

    from aow_sim.control import recovery, righting
    from aow_sim.hw import run_bike

    assert righting.RECOVER_DEG is recovery.RECOVER_DEG
    assert righting.HANDOFF_RATE is recovery.HANDOFF_RATE
    assert "ready_for_policy" in inspect.getsource(righting.RightingSequencer.step)
    assert "ready_for_policy" in inspect.getsource(run_bike.FallGuard.update)


def test_the_handoff_window_is_in_radians_at_the_call_sites():
    """The one trap in sharing it: the threshold is DEGREES and the argument is
    RADIANS, and `righting.roll_pitch` returns degrees. A degree-valued roll
    passed here is silently true for any plausible lean, so the sequencer has
    to convert — and nothing else in the suite exercises that module.
    """
    import inspect
    import math

    from aow_sim.control.recovery import RECOVER_DEG, ready_for_policy
    from aow_sim.control.righting import RightingSequencer

    assert ready_for_policy(math.radians(RECOVER_DEG - 0.1), 0.0)
    assert not ready_for_policy(math.radians(RECOVER_DEG + 0.1), 0.0)
    assert not ready_for_policy(0.0, 3.1)
    # 12.0 as RADIANS would be 687 degrees and would still pass an unconverted
    # comparison; it must not pass this one.
    assert not ready_for_policy(float(RECOVER_DEG), 0.0)
    assert "deg2rad(roll)" in inspect.getsource(RightingSequencer.step)


def test_the_cut_angle_is_configurable():
    """A bike that comes to rest on a wing at 40 deg never reaches 60, and
    would sit there driving. The threshold has to move with the hardware."""
    g = FallGuard(cut_deg=35.0, rearm_deg=15.0)
    assert g.update(0.0, math.radians(40.0), 0.0) == "cut"


def test_rearm_needs_angle_rate_dwell_and_consent():
    g = FallGuard(cut_deg=60.0, rearm_deg=30.0, rearm_rate_dps=60.0,
                  dwell_s=0.5, auto=False)
    g.update(0.0, math.radians(90.0), 0.0)
    assert g.state == "cut"

    # Inside the angle but swinging through it: not settled.
    assert g.update(1.0, math.radians(10.0), math.radians(300.0)) == ""
    # Settled, but not for long enough yet.
    assert g.update(1.1, math.radians(10.0), 0.0) == ""
    assert g.update(1.5, math.radians(10.0), 0.0) == ""
    # Long enough -- but nobody has said yes.
    assert g.update(1.7, math.radians(10.0), 0.0) == ""
    g.request_rearm()
    assert g.update(1.8, math.radians(10.0), 0.0) == "rearm"
    assert g.state == "engaged"
    # Consent is consumed, not sticky: the next fall waits to be asked again.
    assert not g.consent


def test_the_dwell_restarts_when_it_stops_being_settled():
    """Otherwise a bike that is settled, disturbed, then settled again would
    re-engage on the sum of two half-dwells."""
    g = FallGuard(cut_deg=60.0, rearm_deg=30.0, dwell_s=0.5, auto=True)
    g.update(0.0, math.radians(90.0), 0.0)
    g.update(1.0, math.radians(5.0), 0.0)
    g.update(1.4, math.radians(50.0), 0.0)          # knocked back over
    assert g.update(1.45, math.radians(5.0), 0.0) == ""
    assert g.update(1.8, math.radians(5.0), 0.0) == "", "dwell must restart"
    assert g.update(2.0, math.radians(5.0), 0.0) == "rearm"


def test_auto_rearm_needs_no_consent():
    g = FallGuard(cut_deg=60.0, rearm_deg=30.0, dwell_s=0.1, auto=True)
    g.update(0.0, math.radians(90.0), 0.0)
    g.update(1.0, 0.0, 0.0)
    assert g.update(1.2, 0.0, 0.0) == "rearm"


# --- the command struct ---------------------------------------------------

def test_velocity_is_sent_in_world_frame_along_the_heading():
    op = OperatorState(v_max=1.2)
    op.key("w"), op.key("w")
    op.psi = math.pi / 2
    vx, vy = op.packet()["v_cmd_world"]
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(0.2)


def test_speed_is_clamped_both_ways():
    op = OperatorState(v_max=0.3)
    for _ in range(20):
        op.key("w")
    assert op.v == pytest.approx(0.3)
    for _ in range(40):
        op.key("s")
    assert op.v == pytest.approx(-0.3)


def test_space_stops_without_losing_the_heading():
    op = OperatorState()
    op.key("w"), op.key("a")
    psi = op.psi
    op.key(" ")
    assert op.v == 0.0 and op.psi == psi


def test_rearm_is_edge_triggered():
    """One keypress must ask once. A sticky flag re-arms the bike every 20 ms
    for as long as the station is running, which defeats the consent rule."""
    op = OperatorState()
    op.key("r")
    assert op.packet().get("rearm") is True
    assert "rearm" not in op.packet()


def test_righting_is_absent_until_commanded():
    """Absent means 'do not write a goal at all', which leaves the servo
    holding what it has. Sending a number instead would slam it somewhere."""
    op = OperatorState(travel_rad=math.radians(136.6))
    assert "righting_rad" not in op.packet()
    op.key("9")
    assert "righting_rad" in op.packet()


def test_the_righting_keys_step_through_three_positions():
    """Teleop's scheme, copied: 9 walks one way and 4 the other, clipped at the
    ends, so double-tapping either crosses the whole range. Two keys, not
    three, because 6/7/8 are the heading snaps."""
    op = OperatorState(travel_rad=math.radians(136.6))
    op.key("9")
    assert op.packet()["righting_rad"] == pytest.approx(math.radians(316.6))
    op.key("4")
    assert op.packet()["righting_rad"] == pytest.approx(math.radians(180.0))
    op.key("4")
    assert op.packet()["righting_rad"] == pytest.approx(math.radians(43.4))


def test_the_righting_step_is_clipped_at_both_ends():
    """Otherwise holding 9 winds the servo off into extended position, which is
    a multi-turn wind on a mechanism with a hard stop."""
    op = OperatorState(travel_rad=math.radians(136.6))
    for _ in range(5):
        op.key("9")
    assert op.pos == 1
    assert op.packet()["righting_rad"] == pytest.approx(math.radians(316.6))
    for _ in range(5):
        op.key("4")
    assert op.pos == -1
    assert op.packet()["righting_rad"] == pytest.approx(math.radians(43.4))


def test_the_swing_keys_are_inert_without_a_travel_angle():
    """Stow is a known pose; the stroke is not. Without a travel angle 9 and 7
    would both command the same thing and look like neither key worked."""
    op = OperatorState()
    op.key("9")
    assert "righting_rad" not in op.packet()


def test_the_default_travel_is_the_config_that_won():
    """`_smaller` won the swing-linkage optimisation — see
    docs/plans/wing-linkage-design-and-optimization.md, "Which swing-linkage
    config won". The station reads it rather than carrying a literal."""
    from aow_sim.hw.ground import linkage_travel_deg
    assert linkage_travel_deg() == pytest.approx(136.6)
    assert linkage_travel_deg("config/no_such_file.yaml") is None


def test_every_field_the_station_sends_is_read_by_the_bike():
    """The two halves of the protocol, held against each other.

    `BikeRunner` reads the struct in `_apply_command` and `_apply_operator`; a
    field the station sends that nothing reads is a control the operator thinks
    they have.
    """
    import inspect

    from aow_sim.hw import run_bike

    op = OperatorState()
    for ch in "wasd 94[]r":
        op.key(ch)
    src = (inspect.getsource(run_bike.BikeRunner._apply_command)
           + inspect.getsource(run_bike.BikeRunner._apply_operator))
    for field in op.packet():
        assert f'"{field}"' in src, f"the bike never reads {field!r}"
