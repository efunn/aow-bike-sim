"""The "tm151_filter" AHRS model's pieces, on plain arrays: the rate-gated
time constant, the complementary filter, and the accelerometer's lever arm
inside the part. Numbers from the yaw-roll fixture (docs/status.md).

Invalidated by: `sim_ahrs.py`'s FILTER_* / TM151_ACCEL_OFFSET_M constants or
the filter functions.
"""

import numpy as np
import pytest

from aow_sim.sim_ahrs import (FILTER_RATE_MOTION_DPS, FILTER_RATE_REST_DPS,
                              FILTER_TAU_MOTION_S, FILTER_TAU_REST_S,
                              TM151_ACCEL_OFFSET_M, accel_at_offset, filter_tau,
                              run_tilt_filter)

pytestmark = pytest.mark.pure


def test_tau_is_the_resting_value_still_and_the_moving_value_turning():
    assert filter_tau(0.0) == pytest.approx(FILTER_TAU_REST_S)
    assert filter_tau(FILTER_RATE_REST_DPS) == pytest.approx(FILTER_TAU_REST_S)
    assert filter_tau(FILTER_RATE_MOTION_DPS) == pytest.approx(FILTER_TAU_MOTION_S)
    assert filter_tau(500.0) == pytest.approx(FILTER_TAU_MOTION_S)
    rates = np.geomspace(0.5, 30, 20)
    assert np.all(np.diff([filter_tau(r) for r in rates]) >= 0)


def _time_to_63pct(gyro_dps):
    """Accelerometer tilted 5 deg from the filter's state at t = 0, the body
    turning about the state itself (which leaves it fixed): how long until
    the state has moved 63% of the way."""
    t = np.arange(0, 8, 0.005)
    up = np.array([0.0, 0.0, 1.0])
    acc = np.tile([np.sin(np.radians(5)), 0.0, np.cos(np.radians(5))], (len(t), 1))
    gyro = np.tile(np.radians([0.0, 0.0, gyro_dps]), (len(t), 1))
    u = run_tilt_filter(t, gyro, acc, up, smooth_s=0.01)
    ang = np.degrees(np.arccos(np.clip(u @ up, -1, 1)))
    return t[np.argmax(ang > 5 * (1 - np.exp(-1)))]


def test_the_filter_trusts_the_accelerometer_less_while_turning():
    assert _time_to_63pct(0.0) == pytest.approx(FILTER_TAU_REST_S, rel=0.1)
    assert _time_to_63pct(30.0) == pytest.approx(FILTER_TAU_MOTION_S, rel=0.1)


def test_a_sustained_sideways_specific_force_becomes_tilt():
    """The turn case the fixture never saw: 0.1 g held sideways for longer
    than tau is read as ~5.7 deg of lean. That is the physics this model
    carries and the noise-only levels cannot."""
    t = np.arange(0, 10, 0.005)
    acc = np.tile([0.0, 0.1, 1.0], (len(t), 1))
    u = run_tilt_filter(t, np.zeros((len(t), 3)), acc, [0.0, 0.0, 1.0])
    lean = np.degrees(np.arctan2(u[-1, 1], u[-1, 2]))
    assert lean == pytest.approx(np.degrees(np.arctan(0.1)), abs=0.05)


def test_the_accelerometer_offset_adds_the_rigid_body_terms():
    d = np.array([0.01, 0.0, 0.0])
    a0 = np.array([0.0, 0.0, 9.81])
    w = np.array([0.0, 0.0, 2.0])                   # rad/s about z
    # centripetal: toward the axis, w^2 d
    a = accel_at_offset(a0, w, w, 0.01, d)
    assert np.allclose(a - a0, [-4.0 * 0.01, 0.0, 0.0])
    # tangential: alpha x d, alpha from the two rates
    a = accel_at_offset(a0, np.zeros(3), np.array([0.0, 0.0, -1.0]), 0.01, d)
    assert np.allclose(a - a0, [0.0, 100.0 * 0.01, 0.0])
    assert np.allclose(accel_at_offset(a0, w, None, 0.01, np.zeros(3)), a0 + 0)


def test_the_measured_offset_is_inside_the_housing():
    """30 x 26 mm housing footprint: the chip must be over it."""
    x, y, _ = 1e3 * TM151_ACCEL_OFFSET_M
    assert abs(x) < 13.0 and abs(y) < 15.0


def test_the_gate_skips_the_accelerometer_far_from_one_g():
    """A 3 g impact must not drag the state when gated, and must when not."""
    from aow_sim.sim_ahrs import GRAVITY, tilt_filter_step
    up = np.array([0.0, 0.0, 1.0])
    hit = np.array([2.0, 0.0, 2.2]) * GRAVITY           # ~3 g, 42 deg off
    moved = tilt_filter_step(up, np.zeros(3), hit, 0.01, 0.19)
    kept = tilt_filter_step(up, np.zeros(3), hit, 0.01, 0.19, gate=0.1)
    assert np.degrees(np.arccos(moved @ up)) > 1.0
    assert np.allclose(kept, up)
    near = np.array([0.0, 0.05, 1.0]) * GRAVITY          # within 0.1 g of 1 g
    assert not np.allclose(tilt_filter_step(up, np.zeros(3), near, 0.01, 0.19, gate=0.1), up)
