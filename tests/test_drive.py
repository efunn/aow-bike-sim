"""Drive controller tests: gain schedule, straight sprints, circles, stop."""

import mujoco
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control import DriveController, SpeedProfile
from aow_sim.control.linearize import settle_upright
from aow_sim.run_drive import circle_ok, sprint_scenario


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


def test_gain_schedule_designs_everywhere(controller):
    """All mirrored grid speeds produce well-fit, finite gains."""
    assert len(controller.speeds) == 9
    assert np.all(np.isfinite(controller.Ks))
    assert np.all(controller.fit_r2_grid > 0.95)
    # reversed-caster signature: steer/roll gain flips sign with speed
    i_fwd = np.argmax(controller.speeds)
    i_back = np.argmin(controller.speeds)
    assert controller.Ks[i_fwd][1, 1] * controller.Ks[i_back][1, 1] < 0


@pytest.mark.parametrize("v_target,bound", [(0.8, 0.10), (-0.5, 0.10)])
def test_straight_sprint(model, params, eq_qpos, v_target, bound):
    res = sprint_scenario(model, params, eq_qpos, v_target)
    assert res["survived"], res
    assert res["cruise v"] == pytest.approx(v_target, abs=0.1), res
    assert res["max cross-track [m]"] < bound, res
    assert abs(res["final v"]) < 0.05, res
    assert res["max |roll| [deg]"] < 15.0, res


@pytest.mark.parametrize("direction", [+1, -1])
def test_circle_tracks(model, params, eq_qpos, direction):
    ok, err = circle_ok(model, params, eq_qpos, 0.8, direction, v=0.5)
    assert ok, f"circle R=0.8 dir={direction} failed (mean radius err {err:.3f})"
    assert err < 0.12


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


@pytest.mark.parametrize("v,delta_deg,tol", [
    (0.0, 90.0, 6.0),      # standstill: arc mode (pivot recipe)
    (0.8, 90.0, 5.0),      # at speed: rotating carrot + lean/steer ff
    (-0.5, 45.0, 5.0),     # reverse: opposite-signed steer ff
])
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
