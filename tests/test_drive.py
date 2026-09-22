"""Drive controller tests: gain schedule, straight sprints, circles, stop."""

import mujoco
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control import DriveController, SpeedProfile
from aow_sim.control.balance import extract_state
from aow_sim.control.linearize import settle_upright
from aow_sim.run_drive import circle_ok, flip_scenario, sprint_scenario

# Closed-loop drive: gain schedule, sprints, circles, and the trajopt moves.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.contact


# The trajectory flick (moves/flick.yaml, optimize_flick.py) and the scripted
# flip (control.flip) were DEPRECATED 2026-09-21: early LQR-handoff manoeuvres,
# authored against the July 2026 plant and never re-run, which fall on the
# current one. Skipped rather than registered red -- a red entry says "a cost
# we are carrying", and nobody is carrying these. The three *_flick_no_snap /
# double_flick cases test the steer-origin bookkeeping at the pi park, which
# only the trajectory flick produces (flick_rl ends near 0), so they go with it.
# test_flick_trajectory_shape still runs: it is pure and checks the file loads.
DEPRECATED_MOVE = pytest.mark.skip(
    reason="deprecated 2026-09-21: early LQR-handoff manoeuvre, July 2026 plant "
           "(see DriveController.command_flip / command_flick)")


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params, variant="full")


@pytest.fixture(scope="module")
def eq_qpos(model):
    return settle_upright(model).qpos.copy()


@pytest.fixture(scope="module")
def controller(params, model):
    return DriveController(params, model)


def test_speed_profile_retarget_and_limits():
    prof = SpeedProfile(accel=2.0, v_max=1.2)
    dt = 0.005
    vs = []
    prof.set_target(1.0)
    for i in range(200):
        if i == 60:
            prof.set_target(-0.5)   # retarget mid-ramp
        vs.append(prof.step(dt))
    vs = np.array(vs)
    assert np.max(np.abs(np.diff(vs))) <= 2.0 * dt + 1e-12   # accel limit
    assert vs[-1] == pytest.approx(-0.5, abs=1e-9)           # reaches target
    prof.set_target(9.9)
    assert prof.target == pytest.approx(1.2)                 # v_max clamp


@pytest.mark.lqr
def test_gain_schedule_designs_everywhere(controller):
    """All mirrored grid speeds produce well-fit, finite gains."""
    from aow_sim.control.linearize import MIN_FIT_R2
    assert len(controller.speeds) == 9
    assert np.all(np.isfinite(controller.Ks))
    # MIN_FIT_R2, not a literal: this said 0.95 (2026-07-19) while the design
    # and test_balance used 0.93, so the suite held two bars for one fit.
    assert np.all(controller.fit_r2_grid > MIN_FIT_R2), (
        f"worst fit R^2 {controller.fit_r2_grid.min():.4f} < {MIN_FIT_R2}")
    # REMOVED 2026-09-22: "reversed-caster signature: steer/roll gain flips
    # sign with speed" (K[1,1] at +v_max times K[1,1] at -v_max < 0). The
    # identified plant has steer acting on roll with the SAME sign at all nine
    # speeds (B +0.89..+2.11) -- as kinematic lateral acceleration
    # v^2 tan(delta)/L would, being even in v -- so a correct design need not
    # flip, and it does not: +0.90 vs +127.8 once the grid ends were
    # identified on the ground (see linearize.settle_rolling).


@pytest.mark.lqr
@pytest.mark.parametrize("v_target,bound", [(0.8, 0.10), (-0.5, 0.10)])
def test_straight_sprint(model, params, eq_qpos, v_target, bound):
    res = sprint_scenario(model, params, eq_qpos, v_target)
    assert res["survived"], res
    assert res["cruise v"] == pytest.approx(v_target, abs=0.1), res
    assert res["max cross-track [m]"] < bound, res
    assert abs(res["final v"]) < 0.05, res
    assert res["max |roll| [deg]"] < 15.0, res


@pytest.mark.lqr
@pytest.mark.parametrize("direction", [+1, -1])
def test_circle_tracks(model, params, eq_qpos, direction):
    ok, err = circle_ok(model, params, eq_qpos, 0.8, direction, v=0.5)
    assert ok, f"circle R=0.8 dir={direction} failed (mean radius err {err:.3f})"
    assert err < 0.12


@pytest.mark.lqr
def test_reverse_circle_tracks(model, params, eq_qpos):
    ok, err = circle_ok(model, params, eq_qpos, 0.8, +1, v=-0.5)
    assert ok, f"reverse circle R=0.8 failed (mean radius err {err:.3f})"


@DEPRECATED_MOVE
@pytest.mark.parametrize("direction", [+1, -1])
def test_flip_completes(model, params, eq_qpos, direction):
    """180-degree swap-ends: stays upright, completes the flip, ends near the
    start, and settles. (Peak mid-spin excursion ~1 L is intrinsic and not
    asserted — see the decisions doc.)"""
    res = flip_scenario(model, params, eq_qpos, direction)
    assert res["survived"], f"fell during flip: {res}"
    assert res["max |roll| [deg]"] < 15.0, res
    assert abs(res["yaw err [deg]"]) < 8.0, res
    assert res["final excursion [L]"] < 0.5, res
    assert res["settled RMS [deg]"] < 1.0, res


def test_flick_trajectory_shape():
    """Feedforward endpoints and monotonic steer sweep 0->pi."""
    from aow_sim.control.flick import FlickTrajectory
    fl = FlickTrajectory.from_params([2.5, 1.0, 2.0, 3.0, -0.4, 0.0, 0.4])
    assert fl.steer(0.0) == pytest.approx(0.0)
    assert fl.steer(2.5) == pytest.approx(np.pi)
    ts = np.linspace(0, 2.5, 50)
    steers = [fl.steer(t) for t in ts]
    assert np.all(np.diff(steers) >= -1e-9), "steer not monotonic"
    assert fl.hub(0.0) == pytest.approx(0.0)
    assert fl.hub(2.5) == pytest.approx(0.0)


@DEPRECATED_MOVE
def test_flick_replay(model, params, eq_qpos):
    """Optimized two-arc flick replay in the as-authored direction: completes
    180, upright, tight side-to-side envelope, settles. Skips if the move file
    hasn't been generated yet."""
    from aow_sim.control.flick import MOVES_DIR
    if not (MOVES_DIR / "flick.yaml").exists():
        pytest.skip("run `python -m aow_sim.optimize_flick` to generate moves/flick.yaml")
    from aow_sim.run_drive import flick_scenario
    res = flick_scenario(model, params, eq_qpos, +1)
    assert res["survived"], f"fell during flick: {res}"
    assert res["max |roll| [deg]"] < 20.0, res
    assert abs(res["yaw err [deg]"]) < 10.0, res
    assert res["lateral env [L]"] < 0.5, res
    assert res["settled RMS [deg]"] < 1.0, res


@DEPRECATED_MOVE
def test_flick_mirror_survives(model, params, eq_qpos):
    """The left/right mirror (direction=-1) is approximate (the entry lean
    breaks symmetry) but must still complete upright and settle."""
    from aow_sim.control.flick import MOVES_DIR
    if not (MOVES_DIR / "flick.yaml").exists():
        pytest.skip("run `python -m aow_sim.optimize_flick` to generate moves/flick.yaml")
    from aow_sim.run_drive import flick_scenario
    res = flick_scenario(model, params, eq_qpos, -1)
    assert res["survived"], f"fell during mirrored flick: {res}"
    assert res["max |roll| [deg]"] < 25.0, res
    assert abs(res["yaw err [deg]"]) < 20.0, res
    assert res["settled RMS [deg]"] < 1.5, res


@pytest.mark.lqr
def test_reverse_pocket_speed_snapped(model, params):
    """Speed targets inside the reverse instability pocket snap to its edge."""
    c = DriveController(params, model)
    c.set_speed(-0.8)
    lo, hi = params["control"]["drive"]["reverse_avoid_band"]
    assert c.profile.target in (lo, hi), (
        f"target {c.profile.target} not snapped out of [{lo}, {hi}]")


@pytest.mark.lqr
def test_stop_from_circle(model, params, eq_qpos):
    ok, _ = circle_ok(model, params, eq_qpos, 0.8, +1, v=0.5, stop_test=True)
    assert ok, "did not settle balanced after stopping from the circle"


def _fresh(model, eq_qpos):
    data = mujoco.MjData(model)
    data.qpos[:] = eq_qpos
    a = np.deg2rad(0.5)
    data.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(model, data)
    return data


# -- 360-steering regression: post-flick parks must never snap the servo ----
#
# The servo command is ZOH at rate_hz with maneuver steer rate-limited to
# steer_rate (8 rad/s) -> a legitimate per-physics-step ctrl change is
# <= ~0.04 rad; the old bug snapped the wheel by pi in one tick. 0.5 rad
# cleanly discriminates.
SNAP = 0.5


def _steer_recorder(c):
    """(on_step, list) recording data.ctrl[steer] every physics step."""
    log = []
    aid = c.aid["steer"]
    return (lambda dd: log.append(float(dd.ctrl[aid]))), log


def _flick_and_settle(model, params, eq_qpos):
    """Fresh sim -> settle -> one flick -> settle. Returns (data, c, log)."""
    from aow_sim.control.balance import run
    from aow_sim.control.flick import MOVES_DIR
    if not (MOVES_DIR / "flick.yaml").exists():
        pytest.skip("run `python -m aow_sim.optimize_flick` to generate moves/flick.yaml")
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    rec, log = _steer_recorder(c)
    run(model, data, c, 1.0, on_step=rec)
    T = c.command_flick(data, +1)
    run(model, data, c, T + 4.0, on_step=rec)
    return data, c, log


@DEPRECATED_MOVE
def test_double_flick(model, params, eq_qpos):
    """A second consecutive flick starts from the pi park and sweeps
    continuously to 2*pi — the old code snapped back through 0."""
    from aow_sim.control.balance import extract_state, run

    data, c, log = _flick_and_settle(model, params, eq_qpos)
    psi0_total = c._psi  # after first flick: ~pi from start
    rec, log2 = _steer_recorder(c)
    T = c.command_flick(data, +1)
    rolls = []
    run(model, data, c, T + 4.0, on_step=lambda dd: (
        rec(dd), rolls.append(extract_state(dd, c._ref_pos).roll)))
    full = np.array(log + log2)
    assert np.max(np.abs(np.diff(full))) < SNAP, "servo snapped"
    assert np.degrees(np.max(np.abs(rolls))) < 25.0, "fell in second flick"
    err = np.degrees(abs(c._psi - psi0_total)) - 180.0
    assert abs(err) < 15.0, f"second flick yaw error {err:+.1f} deg"
    # winding is bounded and explicit: two flicks park at ~2*pi, not beyond
    assert abs(full[-1]) < 2 * np.pi + SNAP


@DEPRECATED_MOVE
def test_heading_after_flick_no_snap(model, params, eq_qpos):
    """command_heading right after a flick used to zero the steer origin and
    snap the servo a half-turn; now it re-syncs to the pi park."""
    from aow_sim.control.balance import extract_state, run

    data, c, log = _flick_and_settle(model, params, eq_qpos)
    psi0 = c._psi
    rec, log2 = _steer_recorder(c)
    c.command_heading(data, np.deg2rad(90.0))
    rolls = []
    run(model, data, c, 4.5, on_step=lambda dd: (
        rec(dd), rolls.append(extract_state(dd, c._ref_pos).roll)))
    full = np.array(log + log2)
    assert np.max(np.abs(np.diff(full))) < SNAP, "servo snapped"
    assert np.degrees(np.max(np.abs(rolls))) < 20.0, "fell during turn"
    err = np.degrees(c._psi - psi0) - 90.0
    assert abs(err) < 8.0, f"heading error {err:+.1f} deg"


@DEPRECATED_MOVE
def test_reset_after_flick_no_snap(model, params, eq_qpos):
    """Controller reset with the wheel parked at pi (e.g. viewer rewind) must
    re-adopt the park as origin, not command the wheel back to 0."""
    from aow_sim.control.balance import run

    data, c, _ = _flick_and_settle(model, params, eq_qpos)
    park = np.pi * np.round(float(data.qpos[c._sj]) / np.pi)
    c.reset(model, data)
    rec, log = _steer_recorder(c)
    run(model, data, c, 2.0, on_step=rec)
    log = np.array(log)
    assert np.max(np.abs(np.diff(log))) < SNAP, "servo snapped"
    assert np.all(np.abs(log - park) < SNAP), "wheel unwound"


@pytest.mark.parametrize("v,delta_deg,tol", [
    (0.0, 90.0, 6.0),      # standstill: arc mode (pivot recipe)
    (0.8, 90.0, 5.0),      # at speed: ff-carried sharp turn (>15 deg steer)
    (0.8, 180.0, 6.0),     # U-turn at speed
    (-0.5, 90.0, 5.0),     # reverse: opposite-signed steer ff
    (-1.2, 90.0, 6.0),     # fast reverse (above the instability pocket)
])
@pytest.mark.lqr
def test_command_heading(model, params, eq_qpos, v, delta_deg, tol):
    """Teleop-style turns track and stay upright at any speed incl. reverse."""
    from aow_sim.control.balance import extract_state, run

    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    if v:
        c.set_speed(v)
        run(model, data, c, 2.0)
    psi0 = c._psi
    c.command_heading(data, np.deg2rad(delta_deg))
    rolls = []
    run(model, data, c, 4.5, on_step=lambda dd: rolls.append(
        extract_state(dd, c._ref_pos).roll))
    max_roll = np.degrees(np.max(np.abs(rolls)))
    err = np.degrees(c._psi - psi0) - delta_deg
    assert max_roll < 20.0, f"fell during turn (max roll {max_roll:.1f} deg)"
    assert abs(err) < tol, f"heading error {err:+.1f} deg"


@pytest.mark.lqr
def test_follow_command_drives_the_analytic_controller_like_the_station_does(
        model, params, eq_qpos):
    """The bike's LQR mode: a world velocity and an absolute heading, streamed
    as the ground station does, flown by follow_command. Forward at 0.5 m/s,
    then a 90 deg heading change while moving; truth sensors (the bike's own
    sensors are the open question -- docs/status.md)."""
    data = mujoco.MjData(model)
    data.qpos[:] = eq_qpos
    mujoco.mj_forward(model, data)
    c = DriveController(params, model)
    c.reset(model, data)
    c.follow_reset(data)
    psi0 = extract_state(data, np.zeros(3)).yaw
    dt = model.opt.timestep
    v_seen, roll_max = [], 0.0
    for k in range(int(8.0 / dt)):
        t = k * dt
        psi = psi0 + (np.pi / 2 if t >= 4.0 else 0.0)
        v = 0.5 if t >= 1.0 else 0.0
        c.follow_command(data, (v * np.cos(psi), v * np.sin(psi)), psi)
        c.step(model, data)
        mujoco.mj_step(model, data)
        s = extract_state(data, np.zeros(3))
        roll_max = max(roll_max, abs(s.roll))
        if 3.0 <= t < 4.0:
            v_seen.append(s.v_lon)
    assert np.degrees(roll_max) < 20.0, f"fell (max roll {np.degrees(roll_max):.0f})"
    assert np.mean(v_seen) == pytest.approx(0.5, abs=0.05)
    turned = np.degrees(np.arctan2(np.sin(s.yaw - psi0), np.cos(s.yaw - psi0)))
    assert turned == pytest.approx(90.0, abs=10.0), f"heading {turned:.1f} deg"
