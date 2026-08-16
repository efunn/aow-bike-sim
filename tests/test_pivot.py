"""Crawl-pivot controller tests: profile correctness and closed-loop pivots."""

import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control import YawProfile
from aow_sim.control.linearize import settle_upright
from aow_sim.run_pivot import pivot_scenario

# Closed-loop crawl-pivot.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.contact


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params, variant="full")


@pytest.fixture(scope="module")
def eq_qpos(model):
    return settle_upright(model).qpos.copy()


@pytest.mark.parametrize("delta", [np.pi / 2, -np.pi / 2, np.pi, 3 * np.pi, 0.05])
def test_yaw_profile(delta):
    """Profile integrates to exactly delta and respects rate/accel limits."""
    rate, accel = 1.5, 4.0
    prof = YawProfile(delta, rate, accel)
    dt = 1e-4
    ts = np.arange(0, prof.duration + 0.5, dt)
    offs, rates = zip(*[prof.eval(t)[:2] for t in ts])
    offs, rates = np.array(offs), np.array(rates)
    assert offs[-1] == pytest.approx(delta, abs=1e-9)
    assert np.max(np.abs(rates)) <= rate + 1e-9
    # offset must be the integral of rate
    assert np.trapezoid(rates, ts) == pytest.approx(delta, rel=1e-3, abs=1e-4)
    # accel limit: finite-difference the rate
    assert np.max(np.abs(np.diff(rates) / dt)) <= accel * 1.01


def test_pivot_with_wound_steer(model, params, eq_qpos):
    """A pivot commanded with the steer joint parked at pi (post-flick, or a
    multi-turn XC330 reading) regulates about the park instead of unwinding
    a half-turn."""
    import mujoco

    from aow_sim.control import PivotController
    from aow_sim.control.balance import run

    data = mujoco.MjData(model)
    data.qpos[:] = eq_qpos
    sj = model.joint("steer_joint").qposadr[0]
    data.qpos[sj] += np.pi
    mujoco.mj_forward(model, data)
    c = PivotController(params, model)
    c.reset(model, data)
    aid = c.aid["steer"]
    ctrls = []
    run(model, data, c, 1.0, on_step=lambda dd: ctrls.append(float(dd.ctrl[aid])))
    psi0 = c._psi
    T = c.command_pivot(data, np.pi / 2)
    run(model, data, c, T + 3.0, on_step=lambda dd: ctrls.append(float(dd.ctrl[aid])))
    err = np.degrees(c._psi - psi0 - np.pi / 2)
    assert abs(err) < 5.0, f"pivot heading error {err:+.1f} deg"
    assert np.max(np.abs(np.array(ctrls) - np.pi)) < 0.5, "long-way unwind"


@pytest.mark.parametrize("delta_deg", [90.0, -90.0, 180.0])
def test_pivot_completes_upright(model, params, eq_qpos, delta_deg):
    """Pivot tracks the commanded heading, stays upright, and the front
    contact stays planted (the 'in place' requirement)."""
    res = pivot_scenario(model, params, eq_qpos, delta_deg)
    assert res["survived"], f"fell during {delta_deg} deg pivot: {res}"
    assert abs(res["err@1s [deg]"]) < 8.0, res
    assert abs(res["err@4s [deg]"]) < 3.0, res
    assert res["max |roll| [deg]"] < 15.0, res
    assert res["wander [cm]"] < 8.0, res
