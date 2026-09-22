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

import numpy as np
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


def test_rearm_is_a_count_that_survives_lost_packets():
    """One keypress must ask once -- and must not be lost with one datagram,
    which the old edge-triggered `rearm: true` was. Every packet carries the
    count; the bike acts on a CHANGE."""
    op = OperatorState()
    assert op.packet()["rearm_n"] == 0
    op.key("r")
    assert op.packet()["rearm_n"] == 1
    assert op.packet()["rearm_n"] == 1          # repeated, not consumed
    op.key("r")
    assert op.packet()["rearm_n"] == 2


def _bike_hearing(peer="A", lost=False):
    from types import SimpleNamespace
    g = FallGuard()
    g.state = "cut"
    r = SimpleNamespace(guard=g, link=SimpleNamespace(peer=peer),
                        _link_lost=lost, _rearm_seen=None, _pack_low=False,
                        _servo_fault=False, _errs={}, said=[])
    r._hold_reason = lambda: RB.BikeRunner._hold_reason(r)
    r._event = lambda kind, text: r.said.append((kind, text))
    return r


def test_the_bike_acts_on_a_change_in_the_count_and_only_then():
    from aow_sim.hw.run_bike import BikeRunner as B
    r = _bike_hearing()
    B._rearm_from(r, 0)
    assert not r.guard.consent                  # first packet: recorded only
    B._rearm_from(r, 0)
    assert not r.guard.consent                  # repeats are not presses
    B._rearm_from(r, 1)
    assert r.guard.consent                      # a press, however many lost


def test_a_new_station_or_a_press_made_blind_is_not_consent():
    from aow_sim.hw.run_bike import BikeRunner as B
    r = _bike_hearing(peer="A")
    B._rearm_from(r, 5)
    r.link.peer = "B"                           # a different station, at 0
    B._rearm_from(r, 0)
    assert not r.guard.consent
    r._link_lost = True                         # pressed during a dropout
    B._rearm_from(r, 1)
    assert not r.guard.consent
    r._link_lost = False                        # the NEXT press counts
    B._rearm_from(r, 2)
    assert r.guard.consent


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
    for ch in "wasd 94[]rl":
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
        op.sync({"state": "engaged", "psi": 0.3})   # the bike has said where
        before = (op.v, op.psi, op.pos, op.righting_current, op.rearms, op.quit)
        op.key(name)
        after = (op.v, op.psi, op.pos, op.righting_current, op.rearms, op.quit)
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


# --- the session lifecycle: signals, the single-instance port, a lost link --
#
# Driven through BikeRunner's own methods on a stand-in `self`, because the
# real __init__ wants a bundle, a bus and an AHRS. What is under test is the
# decision each method makes, which needs none of those.

import errno
import os
import signal
from types import SimpleNamespace

import pytest

from aow_sim.hw import run_bike as RB


class _Bus:
    policy_ids = (1, 2, 3)

    def __init__(self):
        self.calls = []

    def torque(self, on, ids=None):
        self.calls.append(("torque", on, ids))

    def arm(self, ids=None):
        self.calls.append(("arm", ids))

    def open(self):
        self.calls.append(("open",))


def _runner(torque=True, age=0.0):
    link = SimpleNamespace(age=lambda: age, peer=("192.168.0.114", 5000),
                           addr=("0.0.0.0", 9910))
    said = []
    r = SimpleNamespace(bus=_Bus(), guard=FallGuard(), torque=torque,
                        require_link=True, link=link, _link_lost=False,
                        _roll=0.0, _engage=lambda: None, said=said,
                        _servo_fault=False, _errs={},
                        _pack_low=False, pack=RB.PackMonitor(),
                        _health={}, _dt_meas=0.01,
                        _event=lambda kind, text: said.append((kind, text)))
    r._hold_reason = lambda: RB.BikeRunner._hold_reason(r)
    return r


def test_a_rearm_never_energises_a_no_torque_session():
    """It did, until 2026-09-18: `r` after a cut called `bus.arm`
    unconditionally, so the mode promising nothing can move could be made to
    move by the operator's re-arm key."""
    r = _runner(torque=False)
    RB.BikeRunner._on_guard_event(r, "rearm")
    assert not [c for c in r.bus.calls if c[0] == "arm"]
    r = _runner(torque=True)
    RB.BikeRunner._on_guard_event(r, "rearm")
    assert ("arm", (1, 2, 3)) in r.bus.calls


def test_a_dead_link_cuts_and_holds_instead_of_ending_the_run():
    """The process used to exit here, so every closed mirror window meant
    restarting the bike over ssh. Now: cut, torque off the policy servos
    only, and nothing re-arms while nobody is there -- not even
    --auto-rearm, which with no operator would drive a bike nobody can stop."""
    r = _runner(age=5.0)
    r.guard.auto = True
    RB.BikeRunner._watch_link(r)
    assert r.guard.state == "cut" and r.guard.hold
    assert r.bus.calls == [("torque", False, (1, 2, 3))]
    assert r.guard.cuts == 0                     # a lost link is not a fall
    assert r.guard.update(10.0, 0.0, 0.0) == ""  # settled, auto, still held
    assert r.guard.update(11.0, 0.0, 0.0) == ""

    r.link.age = lambda: 0.01                    # a station comes back
    RB.BikeRunner._watch_link(r)
    assert not r.guard.hold and r.guard.state == "cut"
    # ...and the bike SAID both, as `link` events the station will print
    assert [k for k, _ in r.said] == ["link", "link"]
    assert "LOST" in r.said[0][1] and "BACK" in r.said[1][1]
    assert r.guard.update(12.0, 0.0, 0.0) == ""  # the dwell starts over
    assert r.guard.update(12.3, 0.0, 0.0) == "rearm"


def test_consent_given_before_the_link_died_does_not_survive_it():
    r = _runner(age=5.0)
    r.guard.request_rearm()
    RB.BikeRunner._watch_link(r)
    r.link.age = lambda: 0.01
    RB.BikeRunner._watch_link(r)
    r.guard.update(0.0, 0.0, 0.0)
    assert r.guard.update(1.0, 0.0, 0.0) == ""   # needs a fresh `r`
    r.guard.request_rearm()
    assert r.guard.update(1.1, 0.0, 0.0) == "rearm"


def test_a_lost_link_on_a_no_torque_run_writes_no_torque_at_all():
    r = _runner(torque=False, age=5.0)
    RB.BikeRunner._watch_link(r)
    assert r.bus.calls == [] and r.guard.state == "cut"


def test_a_second_instance_leaves_the_bus_alone():
    """The port is the lock. Opening the bus turns torque OFF on every servo,
    so a second run_bike that got that far before finding 9910 taken would
    drop the first one's bike mid-session."""
    def taken():
        raise OSError(errno.EADDRINUSE, "Address already in use")
    r = _runner()
    r.link.start = taken
    with pytest.raises(SystemExit, match="already running"):
        RB.BikeRunner._run(r, True)
    assert r.bus.calls == []


@pytest.fixture
def _restore_signals():
    saved = {s: signal.getsignal(s)
             for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT)}
    yield
    for s, h in saved.items():
        signal.signal(s, h)


@pytest.mark.parametrize("sig", [signal.SIGTERM, signal.SIGHUP])
def test_a_kill_signal_unwinds_through_shutdown(sig, _restore_signals):
    """SIGTERM and SIGHUP used to kill the process on the spot, skipping the
    `finally` that takes torque off -- so `kill`, `timeout` or a dropped
    `ssh -t` left the servos holding their last goal. A real signal, sent to
    this process from inside the run."""
    seen = []

    def body(_preflight):
        os.kill(os.getpid(), sig)
        seen.append("not interrupted")          # never reached

    r = SimpleNamespace(_run=body, _stopped_by=None,
                        shutdown=lambda: seen.append("shutdown"))
    RB.BikeRunner.run(r)
    assert seen == ["shutdown"]
    assert signal.Signals(sig).name in r._stopped_by


@pytest.mark.parametrize("sig", [signal.SIGTERM, signal.SIGINT])
def test_inside_the_loop_a_signal_waits_for_the_tick_boundary(sig,
                                                              _restore_signals):
    """Raising wherever the main thread happens to be lands inside a bus
    transaction most of the time, and leaves the SDK port marked busy -- the
    shutdown's torque-off then returned COMM_PORT_BUSY on the Pi. In the loop
    the handler only records; the work in progress FINISHES."""
    seen = []

    def body(_preflight):
        RB._STOP["defer"] = True             # what _run does at the loop
        os.kill(os.getpid(), sig)
        seen.append("transaction finished")  # reached: nothing was raised
        if RB._STOP["sig"] is not None:      # the loop's own check
            raise RB._stopped(RB._STOP["sig"])

    r = SimpleNamespace(_run=body, _stopped_by="unset",
                        shutdown=lambda: seen.append("shutdown"))
    RB.BikeRunner.run(r)
    assert seen == ["transaction finished", "shutdown"]
    assert r._stopped_by is None if sig == signal.SIGINT else "SIGTERM" in r._stopped_by


def test_the_shutdown_torque_off_survives_an_abandoned_transaction():
    """`is_using` left set by an interrupted packet made every later write
    COMM_PORT_BUSY; and torque() stopped at the first failure, leaving the
    rest energised. close() clears the flag and tries every servo."""
    from aow_sim.hw.dynamixel import ServoBus
    from aow_sim.params import load_params

    b = ServoBus(load_params(), ids=(1, 2, 3), righting_id=4)
    tries = []

    class Port:
        is_using = True                      # an abandoned transaction

        def clearPort(self):
            pass

        def closePort(self):
            tries.append("closed")

    def torque(on, ids=None):
        (i,) = ids
        assert not b._port.is_using, "busy flag not cleared first"
        tries.append(i)
        if i == 2 and tries.count(2) == 1:
            raise RuntimeError("one bad reply")

    b._port, b.torque = Port(), torque
    b.close()
    assert tries == [1, 2, 3, 4, 2, "closed"]   # all tried, the failure retried


def test_a_hardware_error_is_announced_by_the_bike_once_and_when_it_clears():
    """Righting servo: announced, never a cut -- it is not the policy's."""
    r = _runner()
    for err in (0, 0, 36, 36, 0):
        r._health = {"righting": {"Hardware Error Status": err}}
        RB.BikeRunner._watch_errors(r)
    assert [k for k, _ in r.said] == ["error", "error"]
    assert "overheating" in r.said[0][1] and "cleared" in r.said[1][1]
    assert r.guard.state == "engaged"


def test_a_faulted_policy_servo_cuts_and_holds_until_it_clears():
    """2026-09-18: drive A tripped overload at 88.6 s and the policy drove it,
    dead, for 5 s. A latched error means that servo has ALREADY dropped its
    own torque; the bike cannot balance on it, so the rest go limp too and
    nothing re-arms -- not even --auto-rearm -- until it clears."""
    r = _runner()
    r.guard.auto = True
    r._health = {"drive_a": {"Hardware Error Status": 32}}   # overload
    RB.BikeRunner._watch_errors(r)
    assert r.guard.state == "cut" and r.guard.hold
    assert ("torque", False, (1, 2, 3)) in r.bus.calls
    assert [k for k, _ in r.said] == ["error", "cut"]
    assert r.guard.update(1.0, 0.0, 0.0) == "" == r.guard.update(2.0, 0.0, 0.0)
    RB.BikeRunner._watch_errors(r)                           # still latched
    assert [k for k, _ in r.said] == ["error", "cut"]        # said once
    r._health = {"drive_a": {"Hardware Error Status": 0}}
    RB.BikeRunner._watch_errors(r)
    assert not r.guard.hold
    # and a link that comes back does not release a servo that is still bad
    r._health = {"steer": {"Hardware Error Status": 4}}
    RB.BikeRunner._watch_errors(r)
    r._link_lost = True
    r.link.age = lambda: 0.01
    RB.BikeRunner._watch_link(r)
    assert r.guard.hold


def test_the_alert_bit_is_not_a_failed_instruction():
    """err=128 is 'this servo has a latched hardware error', on every reply
    it sends, and the instruction WAS carried out. Treating it as failure
    ended the 2026-09-18 bench run on a torque-off that had worked."""
    from aow_sim.hw.dynamixel import _failed
    assert not _failed(0, 0x80)          # alert only: done
    assert _failed(0, 0x80 | 0x04)       # alert AND a real error
    assert _failed(0, 0x07)              # access error, no alert
    assert _failed(-1000, 0)             # port busy
    assert not _failed(0, 0)


def test_a_latched_error_is_rebooted_away_at_startup():
    from aow_sim.hw.dynamixel import ServoBus
    from aow_sim.params import load_params
    b = ServoBus(load_params(), ids=(1, 2, 3))
    latched = {1: 32}
    rebooted = []

    class Packet:
        def reboot(self, port, i):
            rebooted.append(i)
            latched.pop(i, None)
            return 0, 0

    b._packet = Packet()
    b.read_raw = lambda i, name: latched.get(i, 0)
    import aow_sim.hw.dynamixel as D
    real_sleep = D.time.sleep
    D.time.sleep = lambda s: None
    try:
        got = b._clear_latched_errors()
    finally:
        D.time.sleep = real_sleep
    assert got == {1: 32} and rebooted == [1]



# --- the packed command (hw/telemetry.encode_command) ----------------------

def test_the_command_is_thirteen_bytes_and_round_trips():
    from aow_sim.hw import telemetry as T
    op = OperatorState(travel_rad=2.4)
    op.v, op.psi = 0.37, 1.234
    op.key("9"), op.key("]"), op.key("r"), op.key("l")
    pkt = op.packet()
    wire = T.encode_command(pkt)
    assert len(wire) == T.COMMAND_BYTES == 13
    assert pkt["controller"] == "lqr" and T.decode_command(wire)["controller"] == "lqr"
    got = T.decode_command(wire)
    assert set(got) == set(pkt)               # same fields, same names
    assert got["v_cmd_world"] == pytest.approx(pkt["v_cmd_world"], abs=1e-3)
    assert got["psi_cmd"] == pytest.approx(pkt["psi_cmd"], abs=1e-4)
    assert got["righting_rad"] == pytest.approx(pkt["righting_rad"], abs=1e-3)
    assert got["righting_current"] == pkt["righting_current"]
    assert got["rearm_n"] == pkt["rearm_n"] == 1


def test_an_absent_righting_goal_stays_absent_not_zero():
    """Absent means 'write no goal'; zero would drive the wing to 0 rad."""
    from aow_sim.hw import telemetry as T
    got = T.decode_command(T.encode_command(OperatorState().packet()))
    assert "righting_rad" not in got and "righting_current" not in got


def test_the_heading_goes_wrapped_and_means_the_same():
    """The bike uses the heading only as wrap_pi(psi_cmd - psi), so a
    command three turns round is the same command."""
    import math
    from aow_sim.hw import telemetry as T
    pkt = {"v_cmd_world": [0.0, 0.0], "psi_cmd": 6 * math.pi + 0.5}
    got = T.decode_command(T.encode_command(pkt))["psi_cmd"]
    assert got == pytest.approx(0.5, abs=1e-4)


def test_a_json_station_is_refused_by_name():
    import json
    from aow_sim.hw import telemetry as T
    with pytest.raises(T.CommandFormatError, match="JSON"):
        T.decode_command(json.dumps({"psi_cmd": 0.0}).encode())
    with pytest.raises(T.CommandFormatError, match="v9"):
        T.decode_command(bytes([9]) + bytes(11))



# --- the heading command starts from the BIKE, not from the AHRS's zero ----

def test_a_new_station_sends_no_heading_until_the_bike_says_where_it_points():
    """It sent 0.0 -- the AHRS's arbitrary yaw zero -- so every connect
    commanded a 45-66 deg turn (four sessions, 2026-09-18)."""
    from aow_sim.hw import telemetry as T
    op = OperatorState()
    op.key("LEFT")                                  # nothing to turn from
    assert op.psi is None and "psi_cmd" not in op.packet()
    assert "psi_cmd" not in T.decode_command(T.encode_command(op.packet()))
    op.sync({"state": "engaged", "psi": -1.0})
    assert op.psi == pytest.approx(-1.0)
    assert T.decode_command(T.encode_command(op.packet()))["psi_cmd"] == \
        pytest.approx(-1.0, abs=1e-4)


def test_while_the_bike_is_not_engaged_the_command_follows_it():
    """So a re-arm starts from where the bike points NOW. Engaged, the
    operator's heading is theirs and telemetry does not move it."""
    op = OperatorState()
    op.sync({"state": "engaged", "psi": 0.0})
    op.key("UP"), op.key("LEFT")
    aimed = op.psi
    op.sync({"state": "engaged", "psi": 0.5})
    assert op.psi == aimed and op.v > 0             # engaged: untouched
    op.sync({"state": "cut", "psi": 1.2})
    assert op.psi == pytest.approx(1.2) and op.v == 0.0
    op.sync({"state": "cut", "psi": -0.4})          # the bike is carried round
    assert op.psi == pytest.approx(-0.4)


def _pack(monitor, volts_by_servo, seconds, dt=0.01):
    events = []
    for _ in range(round(seconds / dt)):
        e = monitor.update(dt, volts_by_servo)
        if e:
            events.append(e)
    return events


def test_the_pack_reads_the_least_loaded_servo():
    """Measured 2026-09-18 on the brick: the loaded drive read 0.7 V under
    the others. Drive A alone at 9.5 V with the rest near 11 is ITS wiring,
    not a flat pack."""
    m = RB.PackMonitor()
    m.v = 11.1
    assert _pack(m, [9.5, 10.9, 11.0, 11.1], 30.0) == []
    assert abs(m.v - 11.1) < 1e-6


def test_a_brief_sag_does_not_trip_and_a_sustained_one_does():
    m = RB.PackMonitor()
    m.v = 11.1
    assert _pack(m, [9.0] * 4, 0.3) == []          # a hard move, 300 ms
    assert m.v > m.warn_v
    assert _pack(m, [11.1] * 4, 5.0) == []
    # a pack actually at 9.8 V: warned first, then cut, ~5 s in, and once
    assert _pack(m, [9.8] * 4, 4.0) == ["warn"]
    assert _pack(m, [9.8] * 4, 2.0) == ["cut"]
    # the rebound when the load stops does not undo either
    assert _pack(m, [10.8] * 4, 30.0) == [] and m.low and m.warned


def test_the_pack_monitor_ignores_missing_readings_and_needs_hysteresis():
    m = RB.PackMonitor()
    assert m.update(0.01, [None, 0]) == "" and m.v is None
    assert m.update(0.01, [None, 9.0]) == "cut"    # first reading seeds
    with pytest.raises(ValueError):
        RB.PackMonitor(warn_v=9.9, cut_v=10.0)


def test_a_flat_pack_cuts_and_holds_and_a_returning_link_does_not_release():
    """It used to end the process on a raw 1 Hz drive-A sample. Now it is a
    cut, like a faulted servo: the loop and telemetry go on, so the station
    sees why."""
    r = _runner()
    r.guard.auto = True
    r.pack.v = 9.0
    r._health = {"drive_a": {"Present Input Voltage": 9.0},
                 "righting": {"Present Input Voltage": 9.1}}
    RB.BikeRunner._watch_pack(r)
    assert r.guard.state == "cut" and r.guard.hold
    assert r.bus.calls == [("torque", False, (1, 2, 3))]
    assert [k for k, _ in r.said] == ["cut"] and "9.0 V" in r.said[0][1]   # filtered
    RB.BikeRunner._watch_pack(r)
    assert len(r.said) == 1                          # said once
    r._link_lost = True
    RB.BikeRunner._watch_link(r)
    assert r.guard.hold
    assert r.guard.update(10.0, 0.0, 0.0) == "" == r.guard.update(20.0, 0.0, 0.0)
    # AND IT SAYS SO, to a station that was not there for the cut: the
    # reason is a level in telemetry, the reconnect names it instead of
    # inviting `r`, and a press is answered rather than swallowed.
    assert "pack at 9.0 V" in r._hold_reason()
    assert "still HELD" in r.said[-1][1] and "pack" in r.said[-1][1]
    r._rearm_seen = (r.link.peer, 0)
    RB.BikeRunner._rearm_from(r, 1)
    assert r.said[-1][0] == "hold" and "r ignored" in r.said[-1][1]

    r = _runner(torque=False)
    r.pack.v = 9.0
    r._health = {"steer": {"Present Input Voltage": 9.0}}
    RB.BikeRunner._watch_pack(r)
    assert r.guard.state == "cut" and r.bus.calls == []   # nothing to turn off


def test_a_held_bike_names_its_reason_and_a_free_one_does_not():
    r = _runner()
    assert r._hold_reason() is None
    r._errs = {"drive_a": 32}
    r._servo_fault = True
    assert r._hold_reason().startswith("drive_a faulted")
    r._pack_low, r.pack.v = True, 9.7
    assert r._hold_reason().startswith("pack at 9.7 V")     # worst first
    # a press while ENGAGED is not answered -- there is nothing to refuse
    r._pack_low = r._servo_fault = False
    r._link_lost = False
    r._rearm_seen = (r.link.peer, 0)
    RB.BikeRunner._rearm_from(r, 1)
    assert r.said == []


def test_a_pack_warning_stands_until_it_becomes_a_hold():
    """Operator, 2026-09-18: PACK LOW flashed on the mirror for 5 s and was
    gone. The warning is a level now, like the hold that supersedes it."""
    r = _runner()
    assert RB.BikeRunner._warning(r) is None
    r.pack.v, r._health = 10.3, {"drive_a": {"Present Input Voltage": 10.3}}
    RB.BikeRunner._watch_pack(r)
    assert r.said[-1][0] == "pack"
    assert "pack low: 10.3 V" in RB.BikeRunner._warning(r)
    r._health = {"drive_a": {"Present Input Voltage": 11.0}}   # load off
    RB.BikeRunner._watch_pack(r)
    assert RB.BikeRunner._warning(r) is not None               # still stands
    r.pack.v, r._health = 9.5, {"drive_a": {"Present Input Voltage": 9.5}}
    RB.BikeRunner._watch_pack(r)
    assert RB.BikeRunner._warning(r) is None and r._hold_reason()


# --- the controller switch: policy <-> LQR --------------------------------

def test_a_v1_station_decodes_and_requests_no_controller():
    """A 12-byte v1 datagram (from before the controller byte) still flies;
    it carries no request, so the bike keeps what it is flying."""
    import struct
    from aow_sim.hw import telemetry as T
    wire = struct.pack("<BBhhhhh", 1, 3, 500, 0, T.ABSENT, T.ABSENT, T.ABSENT)
    got = T.decode_command(wire)
    assert "controller" not in got
    assert got["v_cmd_world"] == pytest.approx([0.5, 0.0]) and got["rearm_n"] == 3


def test_l_toggles_the_controller_and_the_station_shows_what_the_bike_flies():
    from aow_sim.hw.ground import OperatorState, _status
    op = OperatorState()
    assert op.packet()["controller"] == "policy"
    op.key("l")
    assert op.packet()["controller"] == "lqr"
    op.sync({"state": "engaged", "psi": 0.0, "controller": "policy"})
    assert op.controller_actual == "policy"
    # asked for the LQR, the bike is flying the policy: shouted
    assert "POLICY" in _status(op, {"state": "engaged", "controller": "policy"}, 0.0)
    op.key("l")
    assert op.packet()["controller"] == "policy"


def test_the_lqr_is_refused_by_name_off_its_design_rate():
    from aow_sim.control.lqr_design import LQRDesign
    z = np.zeros
    d = LQRDesign(K=z((2, 10)), qpos_eq=z(7), fit_r2=z(10), speeds=z(9),
                  Ks=z((9, 2, 10)), fit_r2_grid=z((9, 10)), rate_hz=200.0)
    assert "200 Hz" in RB.lqr_unavailable(d, 100.0)
    d.rate_hz = 100.0
    assert RB.lqr_unavailable(d, 100.0) is None
    d.Ks = z((9, 2, 8))
    assert "8-state" in RB.lqr_unavailable(d, 100.0)


def _switcher(why=None):
    calls, said = [], []
    ctl = SimpleNamespace(
        follow_reset=lambda data: calls.append("follow_reset"),
        engage_general=lambda data, name, reuse: calls.append(("engage", name)),
        set_command=lambda **kw: calls.append(("set_command", kw)))
    r = SimpleNamespace(controller="policy", lqr_why=why, _lqr_refused=None,
                        ctl=ctl, data=None, gen_name="pol",
                        _event=lambda kind, text: said.append((kind, text)))
    return r, calls, said


def test_switching_to_the_lqr_re_anchors_and_back_re_engages_the_policy():
    r, calls, said = _switcher()
    RB.BikeRunner._switch_controller(r, "lqr")
    assert r.controller == "lqr" and calls == ["follow_reset"]
    RB.BikeRunner._switch_controller(r, "policy")
    assert r.controller == "policy" and ("engage", "pol") in calls
    assert [k for k, _ in said] == ["mode", "mode"]


def test_a_refused_switch_is_said_once_and_changes_nothing():
    r, calls, said = _switcher(why="gains designed at 200 Hz")
    for _ in range(50):                   # the station re-sends every packet
        RB.BikeRunner._switch_controller(r, "lqr")
    assert r.controller == "policy" and calls == []
    assert said == [("mode", "LQR refused: gains designed at 200 Hz")]
