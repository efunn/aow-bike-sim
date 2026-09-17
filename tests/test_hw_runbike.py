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


# --- key parity with teleop, and the escape sequences that carry it ----------

def test_arrows_and_slash_do_what_the_letters_do():
    """The station and run_drive's viewer must not need different hands.

    run_drive binds GLFW 265/264/263/262 (up/down/left/right) to throttle,
    brake-reverse, turn left, turn right, and `/` to zero_command. The letters
    stay as aliases for a terminal that swallows escape sequences.
    """
    from aow_sim.hw.ground import OperatorState
    for arrow, letter in (("UP", "w"), ("DOWN", "s"),
                          ("LEFT", "a"), ("RIGHT", "d")):
        a, b = OperatorState(), OperatorState()
        for _ in range(3):
            a.key(arrow)
            b.key(letter)
        assert (a.v, a.psi) == (b.v, b.psi), f"{arrow} != {letter}"


def test_slash_re_aims_the_heading_at_the_bike_like_teleop_does():
    """run_drive's `/` is zero_command, which sets psi to the bike's ACTUAL
    heading so a policy never inherits a stale setpoint. Space historically
    only zeroed velocity here; both keys now do the teleop thing."""
    from aow_sim.hw.ground import OperatorState
    op = OperatorState()
    op.key("a"), op.key("a"), op.key("w")     # heading away, some speed
    assert op.v > 0 and op.psi != 0.0
    op.sync({"psi": 1.25})
    op.key("/")
    assert op.v == 0.0
    assert op.psi == pytest.approx(1.25)


def test_slash_without_telemetry_holds_the_commanded_heading():
    """No packet yet means the station does not KNOW where the bike points,
    and inventing a heading is worse than keeping the operator's."""
    from aow_sim.hw.ground import OperatorState
    op = OperatorState()
    op.key("a")
    psi = op.psi
    op.key("/")
    assert op.v == 0.0 and op.psi == psi


def test_the_righting_current_is_adopted_from_the_bike_once():
    """`[`/`]` must step from what is ON the servo. The station cannot know
    the startup value any other way, and stepping from zero would make the
    operator's first nudge a 300-count drop."""
    from aow_sim.hw.ground import OperatorState
    op = OperatorState()
    assert op.righting_current is None
    op.sync({"righting_current": 300})
    assert op.righting_current == 300
    op.key("]")
    assert op.righting_current == 320
    op.sync({"righting_current": 300})        # a stale echo must not undo it
    assert op.righting_current == 320


def test_decode_keys_handles_split_and_bare_escapes():
    """An arrow is three bytes and ssh splits reads wherever it likes."""
    from aow_sim.hw.ground import decode_keys
    assert decode_keys("\x1b[Aw\x1b[D/") == (["UP", "w", "LEFT", "/"], "")
    assert decode_keys("\x1bOC") == (["RIGHT"], "")        # application cursor
    # split across two reads: nothing is emitted until the sequence completes
    keys, tail = decode_keys("ab\x1b[")
    assert keys == ["a", "b"] and tail == "\x1b["
    assert decode_keys(tail + "B") == (["DOWN"], "")
    # a bare ESC is dropped, not passed through as a command
    assert decode_keys("\x1b") == ([], "\x1b")
    assert decode_keys("\x1bZ") == ([], "")


def test_every_field_the_bike_sends_back_is_named_in_the_stations_sync():
    """The mirror of test_every_field_the_station_sends_is_read_by_the_bike:
    telemetry the station silently ignores is a control nobody has."""
    from aow_sim.hw.ground import OperatorState
    op = OperatorState()
    op.sync({"psi": 0.4, "righting_current": 300})
    assert op.psi_actual == pytest.approx(0.4)
    assert op.righting_current == 300


def test_the_steer_zero_is_pinned_and_is_not_on_the_wrap():
    """180, not 0 or 360: straight ahead sits mid-range rather than on the
    boundary of the servo's single-turn count."""
    from aow_sim.params import load_params
    cfg = load_params()["control"]["onboard"]
    assert cfg["steer_zero_deg"] == 180.0
    assert cfg["steer_zero_deg"] % 360 != 0


# --- the mirror station drives the same command model ------------------------

def test_the_viewer_and_the_terminal_share_one_command_model():
    """`--mirror` claims the two stations are the same UI. That is only true
    if every GLFW code it binds lands on an action `OperatorState` implements
    -- a code mapping to a name nothing handles is a key that silently does
    nothing, which is the worst kind of control."""
    from aow_sim.run_drive import MIRROR_KEYS
    from aow_sim.hw.ground import OperatorState
    handled = {"UP", "DOWN", "LEFT", "RIGHT", "/", " ", "9", "4", "[", "]",
               "r", "q"}
    assert set(MIRROR_KEYS.values()) <= handled, (
        f"unhandled: {set(MIRROR_KEYS.values()) - handled}")
    # and every one of them actually moves the state
    for name in set(MIRROR_KEYS.values()):
        op = OperatorState(travel_rad=1.0)
        before = (op.v, op.psi, op.pos, op.righting_current, op.rearm, op.quit)
        op.key(name)
        after = (op.v, op.psi, op.pos, op.righting_current, op.rearm, op.quit)
        if name not in (" ", "/"):        # stop from a standstill is a no-op
            assert before != after, f"{name!r} changed nothing"


def test_the_arrows_are_bound_to_teleops_own_glfw_codes():
    """265/264/263/262 are what run_drive's teleop binds. If these drift, the
    two UIs stop being one UI and the whole reason for --mirror goes away."""
    from aow_sim.run_drive import MIRROR_KEYS
    assert MIRROR_KEYS[265] == "UP" and MIRROR_KEYS[264] == "DOWN"
    assert MIRROR_KEYS[263] == "LEFT" and MIRROR_KEYS[262] == "RIGHT"


def test_glfw_reports_letters_uppercase():
    """A binding on ord("w") would never fire: GLFW key codes are the
    UNSHIFTED physical key, which for letters is the uppercase code point.
    Pinned because it is invisible until someone presses the key."""
    from aow_sim.run_drive import MIRROR_KEYS
    for ch in "WASDRQ":
        assert ord(ch) in MIRROR_KEYS
        assert ord(ch.lower()) not in MIRROR_KEYS


# --- preflight: a standing condition is not a fault --------------------------

def test_an_accepted_mount_does_not_disarm_the_other_checks():
    """The bug this prevents: the AHRS mount is `GUESS` until the bike can be
    jigged level, so preflight blocked EVERY run and the only way past was
    `--no-preflight` -- which also turns off the |accel|, |gyro| and QoS
    checks. A gate that fires every time trains you to disable the gates that
    catch things."""
    from aow_sim.hw.run_bike import MOUNT_UNCALIBRATED, _report_preflight
    allow = (MOUNT_UNCALIBRATED,)
    # the standing condition alone: accepted, no raise
    _report_preflight([(MOUNT_UNCALIBRATED, "mount is GUESS")], True, allow)
    # a real fault alongside it: still raises, and the mount is not counted
    with pytest.raises(RuntimeError) as e:
        _report_preflight([(MOUNT_UNCALIBRATED, "mount is GUESS"),
                           (None, "gyro running away")], True, allow)
    assert "1 preflight problem" in str(e.value)


def test_the_mount_blocks_when_it_is_not_accepted():
    """It still blocks by default -- an uncalibrated mount on the assembled
    bike is a permanent roll bias, and that is worth stopping for."""
    from aow_sim.hw.run_bike import MOUNT_UNCALIBRATED, _report_preflight
    with pytest.raises(RuntimeError, match="--allow-guess-mount"):
        _report_preflight([(MOUNT_UNCALIBRATED, "mount is GUESS")], True)


def test_the_error_names_the_narrow_flag_only_when_it_would_work():
    """Suggesting --allow-guess-mount when a gyro fault is also present would
    send the operator to a flag that does not clear the block."""
    from aow_sim.hw.run_bike import MOUNT_UNCALIBRATED, _report_preflight
    with pytest.raises(RuntimeError, match="no-preflight"):
        _report_preflight([(MOUNT_UNCALIBRATED, "m"), (None, "gyro")], True)


def test_non_strict_preflight_reports_without_raising():
    from aow_sim.hw.run_bike import _report_preflight
    got = _report_preflight([(None, "gyro running away")], False)
    assert got == ["gyro running away"]


# --- the ports live in config, not in muscle memory --------------------------

def test_port_precedence_is_flag_then_config_then_generic():
    """Typing two by-id paths on every run is how `--no-preflight` got into
    the habit as well. The flag still wins for a swapped cable."""
    import argparse

    from aow_sim.hw.run_bike import _resolve_ports
    from aow_sim.params import load_params
    cfg = load_params()
    none = argparse.Namespace(port=None, ahrs_port=None)
    dxl, ahrs = _resolve_ports(none, cfg)
    assert dxl == cfg["control"]["onboard"]["dxl_port"]
    assert ahrs == cfg["control"]["onboard"]["ahrs_port"]
    over = argparse.Namespace(port="/dev/ttyUSB9", ahrs_port=None)
    assert _resolve_ports(over, cfg)[0] == "/dev/ttyUSB9"
    assert _resolve_ports(none, {}) == ("/dev/ttyUSB0", "/dev/serial0")


def test_the_configured_ports_are_by_id_not_enumeration_order():
    """`ttyUSB0`/`ttyACM0` are assigned in enumeration order and would swap the
    moment a second FTDI device appeared -- which on this bike means the AHRS
    reader opening the servo bus."""
    from aow_sim.params import load_params
    cfg = load_params()["control"]["onboard"]
    for key in ("dxl_port", "ahrs_port"):
        assert cfg[key].startswith("/dev/serial/by-id/"), key


# --- the heading lead clamp --------------------------------------------------

def test_the_lead_clamp_only_blocks_the_direction_that_grows_the_lead():
    """The mirror's first version blocked BOTH directions once the command
    left the +-35 deg band, so the heading froze with no way back. Worse, the
    band can be left with no key pressed at all -- the BIKE moves -- so it
    presented as "the heading command does nothing" with nothing on screen
    pointing at the clamp."""
    from aow_sim.run_drive import _LEAD_MAX, lead_blocks
    over = _LEAD_MAX + 0.2
    assert lead_blocks(+0.01, over, True), "growing the lead must be blocked"
    assert not lead_blocks(-0.01, over, True), "REDUCING it must be allowed"
    assert lead_blocks(-0.01, -over, True)
    assert not lead_blocks(+0.01, -over, True)
    # inside the band both directions are free
    assert not lead_blocks(+0.01, 0.0, True)
    assert not lead_blocks(-0.01, 0.0, True)


def test_a_snap_disarms_the_clamp():
    """6/7/8 command 90/-90/180, which IS a lead by construction. With the
    clamp armed the gate would kill continuous turning one way while allowing
    the other, which reads as "steering broke after a snap"."""
    from aow_sim.run_drive import _LEAD_MAX, lead_blocks
    assert not lead_blocks(+0.01, _LEAD_MAX + 1.0, False)
    assert not lead_blocks(-0.01, -_LEAD_MAX - 1.0, False)
