"""Multi-turn steering-frame unit tests (no sim, no model build)."""

import numpy as np
import pytest

from aow_sim.control.steer import (
    STEER_CMD_LIMIT,
    TWO_PI,
    XC330_COUNTS_PER_RAD,
    XC330_COUNTS_PER_REV,
    XC330_MAX_TURNS,
    SteerFrame,
    clamp_extended,
    nearest_multiple,
    wrap_pi,
)

# Multi-turn steering frame arithmetic; no sim, no model build.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.pure


def test_wrap_and_nearest_multiple():
    assert wrap_pi(0.0) == pytest.approx(0.0)
    assert wrap_pi(np.pi + 0.1) == pytest.approx(-np.pi + 0.1)
    assert wrap_pi(-np.pi - 0.1) == pytest.approx(np.pi - 0.1)
    assert wrap_pi(5 * TWO_PI + 0.3) == pytest.approx(0.3)
    assert nearest_multiple(3.0) == pytest.approx(np.pi)
    assert nearest_multiple(-3.3) == pytest.approx(-np.pi)
    assert nearest_multiple(6.5) == pytest.approx(2 * np.pi)
    assert nearest_multiple(0.2) == pytest.approx(0.0)
    assert nearest_multiple(6.5, period=TWO_PI) == pytest.approx(TWO_PI)


def test_frame_sync_and_roundtrip():
    f = SteerFrame()
    assert f.origin == 0.0
    f.sync(3.1)
    assert f.origin == pytest.approx(np.pi)
    assert f.measured(3.1) == pytest.approx(3.1 - np.pi)
    # small-signal after sync always lands in [-pi/2, pi/2]
    for q in (-7.0, -3.3, -0.4, 0.0, 1.5, 3.1, 6.5, 40.0):
        f.sync(q)
        assert abs(f.measured(q)) <= np.pi / 2 + 1e-12
    # measured/command round-trip (inside the extended range)
    f.sync(3.1)
    assert f.command(f.measured(3.1)) == pytest.approx(3.1)
    # origin persists across command calls
    f.command(0.2)
    f.command(-0.3)
    assert f.origin == pytest.approx(np.pi)


def test_command_clamps_xc330_range():
    f = SteerFrame()
    f.origin = 300 * TWO_PI          # beyond the extended-position range
    assert f.command(0.0) == pytest.approx(STEER_CMD_LIMIT)
    f.origin = -300 * TWO_PI
    assert f.command(0.0) == pytest.approx(-STEER_CMD_LIMIT)
    assert clamp_extended(1e9) == pytest.approx(STEER_CMD_LIMIT)
    assert clamp_extended(-1e9) == pytest.approx(-STEER_CMD_LIMIT)
    assert clamp_extended(1.0) == pytest.approx(1.0)


def test_xc330_counts_conversion():
    """The future hardware driver is a pure unit conversion."""
    assert XC330_COUNTS_PER_RAD * TWO_PI == pytest.approx(XC330_COUNTS_PER_REV)
    assert round(STEER_CMD_LIMIT * XC330_COUNTS_PER_RAD) == (
        XC330_MAX_TURNS * XC330_COUNTS_PER_REV)


# --- the steer lead bound (control/steer.py) -------------------------------

def test_the_steer_lead_bound_changes_no_torque():
    """Above STEER_LEAD_MAX the steer actuator must ALREADY be at its torque
    limit, even at speed -- that is the whole argument for the bound being
    invisible to a policy trained without it. Re-derived from bike_params, so
    moving kp, kv or the stall torque fails here instead of silently making
    the bound bite. 10 rad/s covers the 8.24 seen in sim with margin."""
    from aow_sim.control.steer import STEER_LEAD_MAX
    from aow_sim.params import load_params
    p = load_params()
    kp, kv = p["actuators"]["steer_kp"], p["actuators"]["steer_kv"]
    limit = (p["servos"]["xc330_t181"]["stall_torque"]
             * p["bike"]["steering"]["gear_ratio"])
    assert kp * STEER_LEAD_MAX - kv * 10.0 >= limit


def test_the_steer_target_cannot_run_away_from_a_wheel_that_does_not_move():
    """The Pi, torque off, 20 s: 167 -> -4412 deg with the wheel at 169.
    Bounded, the same 20 s at the policy's full 8 rad/s ends 45 deg out."""
    from aow_sim.control.steer import STEER_LEAD_MAX, advance_target
    target, wheel = 0.0, 0.0
    for _ in range(2000):                       # 20 s at 100 Hz
        target = advance_target(target, -8.0, 0.01, wheel)
    assert target == pytest.approx(-STEER_LEAD_MAX)
    # ...and a wheel that follows is untouched
    assert advance_target(1.0, 2.0, 0.01, 0.99) == pytest.approx(1.02)
