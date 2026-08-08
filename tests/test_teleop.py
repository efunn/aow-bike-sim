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

    def fake_loop(m, d, step, on_key, intro, module, draw=None, **kw):
        # **kw so viewer-presentation options (show_ui, ...) can be added
        # without every teleop test failing on the signature.
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
        # Derived from the real map, so a key added there (crab_left/right)
        # cannot silently KeyError this stub.
        from aow_sim.run_drive import _MAC_KEYS
        self.state = dict.fromkeys(_MAC_KEYS, False)

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


# --- virtual gamepad + trail ------------------------------------------------

def test_gamepad_axes_map_to_the_command_vector():
    """LEFT stick is the velocity vector, RIGHT stick X is a heading RATE.
    Centred sticks must be the (0,0) command -- an ordinary point, not a
    singularity -- which is why the command is a vector, not (course, speed)."""
    from aow_sim.control import gamepad as gp
    v_max, crab_max, turn = 1.2, 0.48, 1.2

    v_lon, v_lat, psi = gp.apply(gp.Pad(ly=1.0), 0.0, 0.1, v_max, crab_max, turn)
    assert (v_lon, v_lat, psi) == pytest.approx((v_max, 0.0, 0.0))
    v_lon, v_lat, _ = gp.apply(gp.Pad(lx=1.0), 0.0, 0.1, v_max, crab_max, turn)
    assert (v_lon, v_lat) == pytest.approx((0.0, crab_max))
    v_lon, v_lat, _ = gp.apply(gp.Pad(ly=-1.0), 0.0, 0.1, v_max, crab_max, turn)
    assert v_lon == pytest.approx(-v_max)          # reverse is an ordinary point

    # rx INTEGRATES: releasing the stick holds the heading you turned to,
    # where an absolute mapping would snap it back to zero.
    psi = 0.0
    for _ in range(10):
        _, _, psi = gp.apply(gp.Pad(rx=1.0), psi, 0.1, v_max, crab_max, turn)
    assert psi == pytest.approx(turn * 1.0)
    _, _, held = gp.apply(gp.Pad(), psi, 0.1, v_max, crab_max, turn)
    assert held == pytest.approx(psi)

    # centred -> exactly zero, and the deadzone swallows stick bias
    assert gp.apply(gp.Pad(), 0.3, 0.1, v_max, crab_max, turn) == (0.0, 0.0, 0.3)
    drift = gp.apply(gp.Pad(ly=0.05, rx=0.05), 0.0, 0.1, v_max, crab_max, turn)
    assert drift == (0.0, 0.0, 0.0)
    assert gp.to_polar(0.0, 0.0) == (0.0, 0.0)


def _trail_after(model, params, eq_qpos, level, seeded, t_now):
    """Run one _overlay pass at data.time = t_now over a seeded trail."""
    import mujoco
    from aow_sim.control import DriveController
    from aow_sim.run_drive import _fresh, _overlay
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    data.time = t_now
    scn = mujoco.MjvScene(model, maxgeom=2000)
    trail = list(seeded)
    _overlay(scn, model, data, c, [True], trail=trail, trail_level=level)
    return trail


def test_trail_level_zero_is_pen_up_not_erase(model, params, eq_qpos):
    """Level 0 must stop ADDING while keeping what is drawn -- that is what
    makes a disconnected shape (a T) drawable without retracing."""
    seeded = [(0.0, 0.0, 0.0), (0.1, 0.1, 0.0)]
    kept = _trail_after(model, params, eq_qpos, 0.0, seeded, 9.0)
    assert kept == seeded, "pen up must neither append nor expire"


def test_trail_expires_at_the_level_but_infinity_never_does(model, params,
                                                            eq_qpos):
    old = [(0.0, 0.0, 0.0), (0.05, 0.1, 0.0)]        # ancient points
    finite = _trail_after(model, params, eq_qpos, 2.0, old, 9.0)
    assert all(t > 2.0 for t, _, _ in finite), "2 s window kept stale points"

    infinite = _trail_after(model, params, eq_qpos, float("inf"), old, 9.0)
    assert infinite[:2] == old, "inf must never expire history"
    assert len(infinite) == 3, "inf must still append the current point"


def _overlay_scene(model, params, eq_qpos, level, seeded, t_now, maxgeom=2000):
    """One _overlay pass over a seeded trail; returns (trail, scene)."""
    import mujoco
    from aow_sim.control import DriveController
    from aow_sim.run_drive import _fresh, _overlay
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    data.time = t_now
    scn = mujoco.MjvScene(model, maxgeom=maxgeom)
    trail = list(seeded)
    _overlay(scn, model, data, c, [True], trail=trail, trail_level=level)
    return trail, scn


def test_trail_thins_to_the_geom_budget_without_starving_the_dial(
        model, params, eq_qpos):
    """A long `inf` trail must not exhaust scn.maxgeom. Overflowing it drops
    whatever is drawn AFTER the trail -- which is the dial, the one overlay
    you cannot afford to lose."""
    import mujoco
    seeded = [(i * 0.01, i * 0.01, 0.0) for i in range(5000)]
    trail, scn = _overlay_scene(model, params, eq_qpos, float("inf"), seeded,
                                500.0, maxgeom=400)
    assert scn.ngeom < scn.maxgeom, "trail filled the scene to the brim"
    # inf keeps every point in the buffer; only the DRAWING is strided down.
    assert len(trail) == len(seeded) + 1
    arrows = sum(scn.geoms[i].type == mujoco.mjtGeom.mjGEOM_ARROW
                 for i in range(scn.ngeom))
    assert arrows > 0, "dial ticks/rays were starved by the trail"


def test_trail_is_solid_inside_the_window_then_ramps_to_clear(
        model, params, eq_qpos):
    """Alpha is 1.0 within `level` and ramps linearly to 0 across the
    following _TRAIL_FADE_S -- the '2 s solid then 0.5 s fade' look."""
    import mujoco
    from aow_sim.run_drive import _TRAIL, _TRAIL_FADE_S
    now, solid = 10.0, 2.0
    # ages 2.4 and 2.25 sit inside the fade; 0.5 is solid.
    seeded = [(now - 2.4, 0.0, 0.0), (now - 2.25, 0.1, 0.0),
              (now - 0.5, 0.2, 0.0)]
    _, scn = _overlay_scene(model, params, eq_qpos, solid, seeded, now)
    alphas = [round(float(scn.geoms[i].rgba[3]), 3)
              for i in range(scn.ngeom)
              if scn.geoms[i].type == mujoco.mjtGeom.mjGEOM_LINE
              and np.allclose(scn.geoms[i].rgba[:3], _TRAIL, atol=1e-3)]
    assert len(alphas) == 3, f"expected 3 trail segments, got {len(alphas)}"
    horizon = solid + _TRAIL_FADE_S
    assert alphas[0] == pytest.approx((horizon - 2.4) / _TRAIL_FADE_S, abs=1e-3)
    assert alphas[1] == pytest.approx((horizon - 2.25) / _TRAIL_FADE_S, abs=1e-3)
    assert alphas[2] == 1.0, "a point inside the solid window must not fade"
