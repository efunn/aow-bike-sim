"""Stationary balance controller tests (PD cascade and identified-model LQR)."""

import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import make_controller
from aow_sim.control.linearize import settle_upright
from aow_sim.run_balance import push_scenario, tilt_scenario

PUSH_N = 2.0  # comfortably inside the ~4 N envelope both controllers recover


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params, variant="full")


@pytest.fixture(scope="module")
def eq_qpos(model):
    return settle_upright(model).qpos.copy()


@pytest.mark.parametrize("name", ["pd", "lqr"])
def test_tilt_recovery(model, params, eq_qpos, name):
    """From a 3 deg lean: stays upright, settles quietly, bounded drift."""
    m = tilt_scenario(model, params, name, eq_qpos, tilt_deg=3.0, duration=10.0)
    assert m["survived"], f"{name} fell over"
    assert m["max |roll| [deg]"] < 15.0
    assert m["tail roll RMS [deg]"] < 1.0, f"{name} wobbles: {m}"
    assert m["max drift [m]"] < 0.15, f"{name} drifted: {m}"


@pytest.mark.parametrize("name", ["pd", "lqr"])
def test_push_recovery(model, params, eq_qpos, name):
    """Recovers a lateral shove at the chassis."""
    assert push_scenario(model, params, name, eq_qpos, PUSH_N), (
        f"{name} failed to recover a {PUSH_N} N x 0.1 s push"
    )


def test_lqr_model_fit_and_steering(model, params):
    """The identified lateral model fits well and the LQR actually uses the
    steering channel (the steer/crawl coordination seen in the toy)."""
    c = make_controller("lqr", params, model)
    assert np.all(c.fit_r2 > 0.98), f"poor lateral-model fit: {c.fit_r2}"
    k_steer_roll = c.K[1, 1]  # steer command per rad of roll
    assert abs(k_steer_roll) > 0.05, "LQR does not use steering for balance"
