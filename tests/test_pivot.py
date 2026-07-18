"""Crawl-pivot controller tests: profile correctness and closed-loop pivots."""

import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control import YawProfile
from aow_sim.control.linearize import settle_upright
from aow_sim.run_pivot import pivot_scenario


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
