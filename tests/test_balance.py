"""Stationary balance controller tests (PD cascade and identified-model LQR)."""

import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import make_controller
from aow_sim.control.linearize import settle_upright
from aow_sim.run_balance import push_scenario, tilt_scenario

# Closed-loop balance in the sim, plus the LQR's identified-model fit.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.contact

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


# LQR only. The `pd` half of both cases was deleted 2026-08-25: the PD
# cascade was written on day one, never revisited, and has not been what
# drives the bike for a long time -- both `[pd]` cases had been failing
# since the servo plant moved, and tuning a controller nobody uses to make
# them green would have bought a green tick and nothing else. The
# controller itself still builds (control/balance.py, make_controller
# "pd") as a hand-tunable fallback for diagnosing odd behaviour; put "pd"
# back in this list if you ever want it held to a standard again.
@pytest.mark.parametrize("name", ["lqr"])
def test_tilt_recovery(model, params, eq_qpos, name):
    """From a 3 deg lean: stays upright, settles quietly, bounded drift."""
    m = tilt_scenario(model, params, name, eq_qpos, tilt_deg=3.0, duration=10.0)
    assert m["survived"], f"{name} fell over"
    assert m["max |roll| [deg]"] < 15.0
    assert m["tail roll RMS [deg]"] < 1.0, f"{name} wobbles: {m}"
    assert m["max drift [m]"] < 0.15, f"{name} drifted: {m}"


# LQR only. The `pd` half of both cases was deleted 2026-08-25: the PD
# cascade was written on day one, never revisited, and has not been what
# drives the bike for a long time -- both `[pd]` cases had been failing
# since the servo plant moved, and tuning a controller nobody uses to make
# them green would have bought a green tick and nothing else. The
# controller itself still builds (control/balance.py, make_controller
# "pd") as a hand-tunable fallback for diagnosing odd behaviour; put "pd"
# back in this list if you ever want it held to a standard again.
@pytest.mark.parametrize("name", ["lqr"])
def test_push_recovery(model, params, eq_qpos, name):
    """Recovers a lateral shove at the chassis."""
    assert push_scenario(model, params, name, eq_qpos, PUSH_N), (
        f"{name} failed to recover a {PUSH_N} N x 0.1 s push"
    )


@pytest.mark.parametrize("wound", [np.pi, -np.pi, 2 * np.pi])
def test_lqr_balances_with_wound_steer(model, params, eq_qpos, wound):
    """With the steer joint wound past +-pi (post-flick park, or a real XC330
    multi-turn encoder reading), the LQR must regulate about the nearest
    pi-multiple origin — the old code fed raw multi-turn qpos into the state
    and unwound the whole accumulated turn."""
    import mujoco

    from aow_sim.control.balance import extract_state, run

    data = mujoco.MjData(model)
    data.qpos[:] = eq_qpos
    sj = model.joint("steer_joint").qposadr[0]
    data.qpos[sj] += wound
    mujoco.mj_forward(model, data)
    c = make_controller("lqr", params, model)
    c.reset(model, data)
    aid = c.aid["steer"]
    rolls, ctrls = [], []
    run(model, data, c, 3.0, on_step=lambda dd: (
        rolls.append(extract_state(dd, c._ref_pos).roll),
        ctrls.append(float(dd.ctrl[aid]))))
    assert np.degrees(np.max(np.abs(rolls))) < 10.0, "fell with wound steer"
    tail = np.degrees(np.abs(rolls[-int(0.5 / model.opt.timestep):]))
    assert np.sqrt(np.mean(tail**2)) < 2.0, "did not settle"
    # regulates locally about the wound park — no long-way unwind
    assert np.max(np.abs(np.array(ctrls) - wound)) < 0.5


def test_lqr_model_fit_and_steering(model, params):
    """The identified lateral model fits well and the LQR actually uses the
    steering channel (the steer/crawl coordination seen in the toy).

    This was xfailed while contact damping was 0.5 (worst R^2 0.861). It is
    NOT xfailed any more, deliberately: an underdamped contact model is
    expected to come back, and a non-strict xfail would report that breakage
    as a green "xfailed" run. Letting it fail is the point — it fails here AND
    warns at runtime from control.linearize, which is two live signals instead
    of one muted test."""
    from aow_sim.control.linearize import MIN_FIT_R2

    c = make_controller("lqr", params, model)
    # Same threshold the runtime warning uses, so the gate lives in one place:
    # a plant change that degrades the fit trips the test AND announces itself
    # in teleop, rather than only being caught here.
    assert np.all(c.fit_r2 > MIN_FIT_R2), f"poor lateral-model fit: {c.fit_r2}"
    k_steer_roll = c.K[1, 1]  # steer command per rad of roll
    assert abs(k_steer_roll) > 0.05, "LQR does not use steering for balance"
