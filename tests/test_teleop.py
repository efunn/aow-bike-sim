"""Teleop input-model tests.

The viewer's key callback is Callable[[int], None]: a key going down, never
coming up, and in practice no OS auto-repeat either. The input model must
therefore work under BOTH regimes — discrete steps when every hold is a
single event, continuous ramps when auto-repeat does arrive — so the tests
drive the real `step`/`on_key` closures headlessly under each.
"""

import mujoco
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import extract_state
from aow_sim.control.linearize import settle_upright
from aow_sim.run_drive import (_COAST_DELAY, _LEAD_MAX, _REPEAT_GAP, _STEP_V,
                               _Axis, _KeyState)

UP, DOWN, LEFT, RIGHT = 265, 264, 263, 262


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params, variant="full")


@pytest.fixture(scope="module")
def eq_qpos(model):
    return settle_upright(model).qpos.copy()


# -- the axis model (pure logic) ------------------------------------------

def test_axis_distinguishes_a_fresh_press_from_auto_repeat():
    ax = _Axis()
    assert ax.press(0.0, +1) is True, "first press must be fresh"
    assert not ax.ramping(0.0), "one press is not proof the key is held"
    # an event within the repeat gap proves auto-repeat -> hold-to-ramp
    assert ax.press(_REPEAT_GAP * 0.4, +1) is False
    assert ax.ramping(_REPEAT_GAP * 0.4)
    # a gap longer than the repeat window is a new press, not a hold
    assert ax.press(10.0, +1) is True
    assert not ax.ramping(10.0)
    # reversing direction is always a fresh press
    ax.press(10.0 + _REPEAT_GAP * 0.3, -1)
    assert ax.dir == -1


def test_axis_physical_hold_needs_arming_then_releases_instantly():
    """With true key state the ramp follows the physical key — but only
    after the viewer delivered a keydown, so keys pressed while another
    window has focus can never drive the bike."""
    ax = _Axis()
    # not armed: a physically-down key is ignored
    assert ax.physical_hold(True, False) == 0
    ax.press(0.0, +1)                       # viewer keydown arms it
    assert ax.physical_hold(True, False) == 1
    assert ax.physical_hold(True, False) == 1, "hold should persist"
    assert ax.physical_hold(False, False) == 0, "release must be instant"
    # and it stays released until the viewer arms it again
    assert ax.physical_hold(True, False) == 0
    ax.press(1.0, +1)
    assert ax.physical_hold(True, False) == 1


def test_keystate_degrades_safely():
    """Whatever the platform, _KeyState must not raise, and until a backend
    has actually observed a key going down it must report everything as up —
    that is what keeps a denied permission from being worse than tapping."""
    ks = _KeyState()
    assert isinstance(ks.available, bool)
    assert isinstance(ks.routes, list)
    ks.poll(0.0)
    for name in ("up", "down", "left", "right"):
        assert isinstance(ks.down(name), bool)
    # nothing is being held in a test run, so nothing may be confirmed
    assert not ks.confirmed and ks.source is None
    assert not any(ks.down(n) for n in ("up", "down", "left", "right"))


def test_axis_release_is_detected_by_silence():
    ax = _Axis()
    ax.press(0.0, +1)
    assert not ax.released(_COAST_DELAY * 0.5)
    assert ax.released(_COAST_DELAY * 1.5)
    ax.clear()
    assert ax.released(0.0)


# -- driving the real teleop closures -------------------------------------

def _capture(monkeypatch, model, params, eq_qpos, hockey=False, analytic=False):
    """Run _teleop but capture its callbacks instead of opening a viewer.

    Teleop now boots with the general policy engaged when one exists, so
    tests that exercise the analytic controller ask for `analytic=True`."""
    g = {}

    def fake_loop(m, d, step, on_key, intro, module, draw=None):
        g.update(model=m, data=d, step=step, on_key=on_key, draw=draw)

    monkeypatch.setattr("aow_sim.interactive.teleop_loop", fake_loop)
    from aow_sim.run_drive import _teleop
    g["c"] = _teleop(model, params, eq_qpos, hockey=hockey)
    if analytic and g["c"].mode == "general":
        g["on_key"](ord(","))
        for _ in range(3):
            g["step"](g["model"], g["data"])
            mujoco.mj_step(g["model"], g["data"])
        assert g["c"].mode != "general"
    return g


def _idle(g, seconds):
    m, d, step = g["model"], g["data"], g["step"]
    for _ in range(int(seconds / m.opt.timestep)):
        step(m, d)
        mujoco.mj_step(m, d)


def _tap(g, key, settle=0.05):
    g["on_key"](key)
    _idle(g, settle)


def _hold(g, seconds, key, repeat_delay=0.4, repeat_hz=30.0):
    """Hold a key the way an OS with auto-repeat would report it: one press,
    a delay, then a repeat stream."""
    m, d, step, on_key = g["model"], g["data"], g["step"], g["on_key"]
    on_key(key)
    nxt = d.time + repeat_delay
    for _ in range(int(seconds / m.opt.timestep)):
        if d.time >= nxt:
            on_key(key)
            nxt = d.time + 1.0 / repeat_hz
        step(m, d)
        mujoco.mj_step(m, d)


def test_taps_accumulate_without_auto_repeat(monkeypatch, model, params,
                                             eq_qpos):
    """Regression for "the arrows don't do much": MuJoCo may deliver only one
    event per press, so a fresh press must move the target by itself."""
    g = _capture(monkeypatch, model, params, eq_qpos, analytic=True)
    c = g["c"]
    # taps spaced like a human (well past _REPEAT_GAP, inside _COAST_DELAY)
    for n in (1, 2, 3):
        _tap(g, UP, settle=0.2)
        assert c.profile.target == pytest.approx(n * _STEP_V, abs=1e-6), \
            f"tap {n} did not step the target"
    _idle(g, _COAST_DELAY + 3 * _STEP_V / 0.6 + 0.5)
    assert c.profile.target == pytest.approx(0.0, abs=1e-6), "did not coast"


class _FakeKeys:
    """Stand-in for Quartz key polling, so the true-key-state path is
    testable on any platform."""

    def __init__(self):
        self.available = True
        self.confirmed = True
        self.source = "fake"
        self.routes = ["fake"]
        self.state = dict.fromkeys(("up", "down", "left", "right"), False)

    def poll(self, now):
        pass

    def down(self, name):
        return self.state[name]


def test_true_key_state_holds_then_releases_instantly(monkeypatch, model,
                                                      params, eq_qpos):
    """With real key up/down available, holding ramps and letting go coasts
    immediately — no auto-repeat and no timed grace involved."""
    fake = _FakeKeys()
    monkeypatch.setattr("aow_sim.run_drive._KeyState", lambda: fake)
    g = _capture(monkeypatch, model, params, eq_qpos, analytic=True)
    c = g["c"]

    g["on_key"](UP)                 # viewer keydown arms the axis
    fake.state["up"] = True         # ...and the key is physically held
    _idle(g, 1.5)
    ramped = c.profile.target
    assert ramped > 0.6, f"hold did not ramp (target {ramped:.2f})"

    fake.state["up"] = False        # let go
    _idle(g, 0.25)
    assert c.profile.target < ramped - 0.1, "release did not coast immediately"

    # a key held while the viewer never saw a keydown must do nothing
    parked = c.profile.target
    fake.state["down"] = True
    _idle(g, 0.05)
    assert c.profile.target <= parked, "unarmed key drove the bike"


def test_hold_ramps_to_full_speed_with_auto_repeat(monkeypatch, model,
                                                   params, eq_qpos):
    g = _capture(monkeypatch, model, params, eq_qpos, analytic=True)
    c = g["c"]
    _hold(g, 2.0, UP)
    assert c.profile.target == pytest.approx(c.profile.v_max, abs=1e-6)
    s = extract_state(g["data"], c._ref_pos)
    assert s.v_lon > 0.8, f"bike did not actually accelerate (v={s.v_lon:.2f})"


def test_brake_carries_through_zero_into_reverse(monkeypatch, model, params,
                                                 eq_qpos):
    g = _capture(monkeypatch, model, params, eq_qpos, analytic=True)
    c = g["c"]
    _hold(g, 1.0, UP)
    assert c.profile.target > 0.3
    _hold(g, 1.5, DOWN)
    assert c.profile.target < -0.05, "brake did not carry into reverse"


def test_held_turn_is_lead_clamped(monkeypatch, model, params, eq_qpos):
    """A held turn must not wind the command past what the bike can follow —
    otherwise it keeps spinning long after release."""
    g = _capture(monkeypatch, model, params, eq_qpos, analytic=True)
    c = g["c"]
    _hold(g, 3.0, LEFT)
    lead = abs(float(np.arctan2(np.sin(c._psi_path_target - c._psi),
                                np.cos(c._psi_path_target - c._psi))))
    assert lead <= _LEAD_MAX + np.deg2rad(2), \
        f"command ran away: {np.degrees(lead):.0f} deg of lead"
    assert c._psi_path_target - c._psi_path != pytest.approx(0.0), \
        "the turn should have moved the reference"


def test_turn_reverses_and_heading_holds_after_release(monkeypatch, model,
                                                       params, eq_qpos):
    """Heading is a setpoint: it does not decay the way velocity does."""
    g = _capture(monkeypatch, model, params, eq_qpos, analytic=True)
    c = g["c"]
    _tap(g, LEFT, settle=0.2)
    left_target = c._psi_path_target
    assert left_target > 0.05, "left tap did not move the heading command"
    _tap(g, RIGHT, settle=0.2)
    assert c._psi_path_target < left_target, "right tap did not reverse it"
    parked = c._psi_path_target
    _idle(g, 1.5)
    assert c._psi_path_target == pytest.approx(parked, abs=1e-9)


@pytest.mark.parametrize("key", ["/", "5"])
def test_zero_and_stop_are_immediate(monkeypatch, model, params, eq_qpos, key):
    g = _capture(monkeypatch, model, params, eq_qpos, analytic=True)
    c = g["c"]
    _hold(g, 1.0, UP)
    assert c.profile.target > 0.3
    g["on_key"](ord(key))
    _idle(g, 3 * model.opt.timestep)
    assert c.profile.target == pytest.approx(0.0, abs=1e-9)


def test_overlay_draws_and_toggles(monkeypatch, model, params, eq_qpos):
    g = _capture(monkeypatch, model, params, eq_qpos)
    _hold(g, 0.6, UP)
    scn = mujoco.MjvScene(model, 2000)
    g["draw"](scn, g["model"], g["data"])
    assert scn.ngeom > 24, "dial should draw a rim plus ticks and arrows"
    g["on_key"](ord("2"))
    _idle(g, 3 * model.opt.timestep)
    g["draw"](scn, g["model"], g["data"])
    assert scn.ngeom == 0, "overlay toggle did not clear the dial"


def test_maneuver_key_zeroes_the_speed_intent(monkeypatch, model, params,
                                              eq_qpos):
    from aow_sim.control.flick import MOVES_DIR
    if not (MOVES_DIR / "flick.yaml").exists():
        pytest.skip("needs moves/flick.yaml")
    g = _capture(monkeypatch, model, params, eq_qpos, analytic=True)
    c = g["c"]
    _hold(g, 1.0, UP)
    assert c.profile.target > 0.3
    g["on_key"](ord("8"))
    _idle(g, 3 * model.opt.timestep)
    assert c.mode == "flick"
    assert c.profile.target == pytest.approx(0.0, abs=1e-9)


# -- general (always-on) policy layer --------------------------------------

def _needs_general():
    from aow_sim.control.flick import MOVES_DIR
    from aow_sim.control.general_spec import OBS_DIM
    if not (MOVES_DIR / "general_rl.yaml").exists():
        pytest.skip("run `python -m aow_sim.train_general_rl` first")
    from aow_sim.control.policy import load_policy_npz
    from aow_sim.control.flick import load_move
    if load_move("general_rl").obs_dim != OBS_DIM:
        pytest.skip("moves/general_rl predates the current obs spec — retrain")
    del load_policy_npz


def test_general_policy_is_the_default(monkeypatch, model, params, eq_qpos):
    """Teleop should come up driving the policy, not the analytic controller."""
    _needs_general()
    g = _capture(monkeypatch, model, params, eq_qpos)
    assert g["c"].mode == "general", "policy should be engaged at startup"


def test_general_policy_survives_a_viewer_reset(monkeypatch, model, params,
                                                eq_qpos):
    """The viewer's reset rewinds data.time, which makes _Base.step call
    DriveController.reset -> command_line and silently drop the policy. The
    operator's intent lives in teleop, so it must be re-asserted."""
    _needs_general()
    g = _capture(monkeypatch, model, params, eq_qpos)
    c, d = g["c"], g["data"]
    _idle(g, 0.2)
    assert c.mode == "general"

    mujoco.mj_resetData(model, d)       # what the viewer's reset does
    d.qpos[:] = eq_qpos
    mujoco.mj_forward(model, d)
    _idle(g, 0.05)
    assert c.mode == "general", "policy was dropped by the reset"

    # ...but only while it is wanted: after ',' a reset must stay analytic
    g["on_key"](ord(","))
    _idle(g, 0.05)
    assert c.mode != "general"
    mujoco.mj_resetData(model, d)
    d.qpos[:] = eq_qpos
    mujoco.mj_forward(model, d)
    _idle(g, 0.05)
    assert c.mode != "general", "reset re-engaged a policy that was turned off"

    g["on_key"](ord(","))               # and it can be turned back on
    _idle(g, 0.05)
    assert c.mode == "general"


def test_engaging_general_zeroes_a_stale_command(monkeypatch, model, params,
                                                 eq_qpos):
    """Starting the policy must never inherit leftover speed or heading."""
    _needs_general()
    from aow_sim.run_drive import _command_ref
    # start analytic so a stale command can be built up, then engage
    g = _capture(monkeypatch, model, params, eq_qpos, analytic=True)
    c = g["c"]
    _hold(g, 1.0, UP)
    _tap(g, LEFT, settle=0.1)
    assert c.profile.target > 0.3
    g["on_key"](ord(","))
    _idle(g, 3 * model.opt.timestep)
    assert c.mode == "general"
    h, v = _command_ref(c, g["data"])
    assert np.linalg.norm(v) == pytest.approx(0.0, abs=1e-9), "stale velocity"
    assert abs(h - c._psi) < np.deg2rad(2.0), "stale heading command"


def test_general_mode_snaps_and_shadows_moves(monkeypatch, model, params,
                                              eq_qpos):
    """In general mode 6/7/8 snap the heading instead of firing moves, and
    snaps bypass the lead clamp (a commanded 180 is meant to lead)."""
    _needs_general()
    g = _capture(monkeypatch, model, params, eq_qpos)   # policy is the default
    c = g["c"]
    _idle(g, 3 * model.opt.timestep)
    assert c.mode == "general"
    for key, want in ((ord("6"), 90.0), (ord("7"), -90.0), (ord("8"), 180.0)):
        before = c._gen_psi_cmd
        g["on_key"](key)
        _idle(g, 3 * model.opt.timestep)
        assert np.degrees(c._gen_psi_cmd - before) == pytest.approx(want,
                                                                    abs=1e-6)
        assert c.mode == "general", "a move fired while the policy was engaged"


def test_general_mode_tracks_a_speed_command(monkeypatch, model, params,
                                             eq_qpos):
    """End-to-end: a held throttle in general mode makes the trained policy
    actually drive (the command must reach it and be tracked)."""
    _needs_general()
    from aow_sim.run_drive import _command_ref
    g = _capture(monkeypatch, model, params, eq_qpos)
    c = g["c"]
    g["on_key"](ord(","))
    _idle(g, 0.2)
    _hold(g, 2.5, UP)
    _h, v_cmd = _command_ref(c, g["data"])
    assert np.linalg.norm(v_cmd) > 0.3, "command never built"
    s = extract_state(g["data"], c._ref_pos)
    assert s.v_lon > 0.2, f"policy did not follow the command (v={s.v_lon:.2f})"


def test_general_mode_reports_a_missing_policy(monkeypatch, model, params,
                                               eq_qpos, capsys):
    from aow_sim.control.flick import MOVES_DIR
    if (MOVES_DIR / "general_rl.yaml").exists():
        pytest.skip("a general policy exists; this covers the missing case")
    g = _capture(monkeypatch, model, params, eq_qpos)
    g["on_key"](ord(","))
    _idle(g, 3 * model.opt.timestep)
    assert "train_general_rl" in capsys.readouterr().out
    assert g["c"].mode != "general"
