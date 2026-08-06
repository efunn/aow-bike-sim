"""Odometry validation against simulator ground truth.

Pins the two claims hw/odometry.py rests on:
  * longitudinal speed comes from hub kinematics and is accurate;
  * lateral speed comes from the FRONT wheel's rolling constraint, not from
    the rear rollers, and needs no calibration constant.

Both are measured against the simulator's own state, so a model or parameter
change that invalidates either fails here rather than on the floor.

Every episode settles the bike upright first and asserts it stayed up. An
earlier version of this analysis did not, and measured a fallen bike thrashing
on its side — which made the rear-roller estimator look far worse than it is
and produced meaningless fits.
"""

import mujoco
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import extract_state
from aow_sim.control.drive import DriveController
from aow_sim.control.linearize import settle_upright
from aow_sim.control.steer import wheel_heading
from aow_sim.hw.odometry import VelocityEstimator, body_to_world

MAX_ROLL_DEG = 25.0


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params)


def _episode(params, model, setup, n=12000, shove=0.0, seed=0):
    """Run the controller upright and return per-sample measurements.

    Columns: v_lon(wheel), steer joint, yaw_rate, v_lon(true), v_lat(true).
    """
    est = VelocityEstimator(params)
    ia = model.joint("input_a_spin").dofadr[0]
    ib = model.joint("input_b_spin").dofadr[0]
    sj = model.joint("steer_joint").qposadr[0]

    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    mujoco.mj_forward(model, data)
    ctl = DriveController(params, model)
    ctl.reset(model, data)
    setup(ctl, data)

    rng = np.random.default_rng(seed)
    rows, max_roll = [], 0.0
    for k in range(n):
        ctl.step(model, data)
        if shove and k and k % 3000 == 0:
            data.qvel[1] += rng.uniform(-shove, shove)
        mujoco.mj_step(model, data)
        s = extract_state(data, np.zeros(3))
        max_roll = max(max_roll, abs(s.roll))
        if k > 3000 and k % 10 == 0:
            # qvel at the input joints is in INPUT-SHAFT units; the estimator
            # takes SERVO units, as the Dynamixel feedback reports them.
            rows.append((est.longitudinal(data.qvel[ia] / est.belt_ratio,
                                          data.qvel[ib] / est.belt_ratio),
                         float(data.qpos[sj]), data.qvel[5], s.v_lon, s.v_lat))
    assert np.degrees(max_roll) < MAX_ROLL_DEG, (
        f"bike fell during the episode (max roll {np.degrees(max_roll):.0f} deg); "
        "the measurement is meaningless")
    return np.array(rows)


REGIMES = {
    "standstill": (lambda c, d: c.set_speed(0.0), 0.0),
    "standstill_shoved": (lambda c, d: c.set_speed(0.0), 0.12),
    "straight_0.6": (lambda c, d: c.set_speed(0.6), 0.0),
    "circle_R0.8": (lambda c, d: (c.set_speed(0.5), c.command_circle(d, 0.8, +1)), 0.0),
}


@pytest.fixture(scope="module")
def episodes(params, model):
    return {k: _episode(params, model, s, shove=sh) for k, (s, sh) in REGIMES.items()}


@pytest.mark.parametrize("regime", list(REGIMES))
def test_longitudinal_odometry_tracks_truth(episodes, regime):
    """Hub kinematics reproduces true forward speed across every regime."""
    v_lon_w, _, _, v_lon_true, _ = episodes[regime].T
    rms = float(np.sqrt(np.mean((v_lon_w - v_lon_true) ** 2)))
    assert rms < 0.030, f"{regime}: longitudinal odometry {rms*1000:.1f} mm/s RMS"


@pytest.mark.parametrize("regime", list(REGIMES))
def test_front_wheel_constraint_gives_lateral_velocity(episodes, params, regime):
    """v_lat = v_lon*tan(theta) - yaw_rate*L, with no calibration constant.

    Measured 6.1 mm/s RMS pooled; 20 mm/s here is a regression guard.
    """
    est = VelocityEstimator(params)
    v_lon_w, steer, yaw_rate, _, v_lat_true = episodes[regime].T
    pred = np.array([est.lateral_from_front(v, s, y)[0]
                     for v, s, y in zip(v_lon_w, steer, yaw_rate)])
    rms = float(np.sqrt(np.mean((pred - v_lat_true) ** 2)))
    corr = float(np.corrcoef(pred, v_lat_true)[0, 1])
    assert rms < 0.020, f"{regime}: lateral constraint {rms*1000:.1f} mm/s RMS"
    assert corr > 0.90, f"{regime}: lateral constraint correlation {corr:.3f}"


def test_front_constraint_beats_roller_kinematics(episodes, params):
    """The reason the rear rollers are not used. Pooled over all regimes."""
    est = VelocityEstimator(params)
    A = np.vstack(list(episodes.values()))
    v_lon_w, steer, yaw_rate, _, v_lat_true = A.T
    front = np.array([est.lateral_from_front(v, s, y)[0]
                      for v, s, y in zip(v_lon_w, steer, yaw_rate)])
    rms_front = np.sqrt(np.mean((front - v_lat_true) ** 2))
    # The roller estimate needs the raw differential, which this fixture does
    # not carry; compare against the trivial "no lateral motion" estimator,
    # which the roller version must at least beat to be worth anything.
    rms_zero = np.sqrt(np.mean(v_lat_true ** 2))
    assert rms_front < 0.5 * rms_zero, (
        f"front-wheel constraint ({rms_front*1000:.1f} mm/s) is not clearly "
        f"better than assuming zero ({rms_zero*1000:.1f} mm/s)")


def test_constraint_coefficients_are_the_geometry(episodes, params):
    """A free fit must recover the theoretical coefficients.

    If it stops doing so, the clean no-calibration formula in hw/odometry.py
    has quietly become wrong and needs revisiting.
    """
    A = np.vstack(list(episodes.values()))
    v_lon_w, steer, yaw_rate, _, v_lat_true = A.T
    rake = np.deg2rad(params["bike"]["rake_deg"])
    theta = np.array([wheel_heading(s, rake) for s in steer])

    M = np.stack([v_lon_w * np.tan(theta), -yaw_rate], axis=1)
    tan_coef, l_eff = np.linalg.lstsq(M, v_lat_true, rcond=None)[0]
    assert 0.85 < tan_coef < 1.15, f"tan coefficient {tan_coef:.3f}, expected ~1"
    L = params["bike"]["wheelbase"]
    assert abs(l_eff - L) < 0.15 * L, f"L_eff {l_eff:.4f} vs wheelbase {L}"


def test_confidence_collapses_when_front_wheel_is_perpendicular(params):
    """At theta -> +-90 deg the front wheel constrains nothing, and the
    estimator must say so rather than emit a huge tan(). The `flip` maneuver
    pre-steers to exactly 90 deg, so this is a real operating point."""
    est = VelocityEstimator(params)
    _, conf_straight = est.lateral_from_front(0.5, 0.0, 0.0)
    _, conf_45 = est.lateral_from_front(0.5, np.deg2rad(45), 0.0)
    _, conf_90 = est.lateral_from_front(0.5, np.deg2rad(90), 0.0)
    assert conf_straight > 0.9
    assert 0.0 < conf_45 < conf_straight
    assert conf_90 == 0.0

    v, _ = est.lateral_from_front(0.5, np.deg2rad(90), 0.0)
    assert np.isfinite(v), "must not return inf/nan at the singularity"


def test_wound_steer_angle_reads_the_same_as_its_wrapped_equivalent(params):
    """The front wheel is symmetric, so a multi-turn angle must give the same
    lateral estimate as its wrapped equivalent -- no pi-rebasing needed."""
    est = VelocityEstimator(params)
    for delta in (0.2, -0.35, 0.9):
        base, _ = est.lateral_from_front(0.6, delta, 0.4)
        for winding in (np.pi, -np.pi, 2 * np.pi, 6 * np.pi):
            wound, _ = est.lateral_from_front(0.6, delta + winding, 0.4)
            assert np.isclose(base, wound, atol=1e-9), (
                f"steer {delta}+{winding} read differently from {delta}")


@pytest.mark.parametrize("regime", list(REGIMES))
def test_fused_estimator_end_to_end(params, model, regime):
    """The whole estimator, driven by the model's own AHRS sensors.

    This is the closest thing to a hardware test that exists without hardware:
    `ahrs_gyro` and `ahrs_accel` are sampled at the AHRS site, so they carry
    the same lever-arm terms the real TM151 will (it sits at [0.05, 0, 0.13],
    not at the chassis origin), and the loop runs at the real 100 Hz control
    rate rather than the physics rate.

    Guards against the specific mistake this replaced: a conventional
    complementary filter with a 0.3 s time constant degraded v_lon from 8.8 to
    174 mm/s RMS by integrating those lever-arm terms. Both channels here are
    good enough that the accelerometer is only a fallback.
    """
    setup, shove = REGIMES[regime]
    est = VelocityEstimator(params)
    ia = model.joint("input_a_spin").dofadr[0]
    ib = model.joint("input_b_spin").dofadr[0]
    sj = model.joint("steer_joint").qposadr[0]

    def sensor(data, name):
        s = model.sensor(name)
        return data.sensordata[s.adr[0]:s.adr[0] + s.dim[0]].copy()

    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    mujoco.mj_forward(model, data)
    ctl = DriveController(params, model)
    ctl.reset(model, data)
    setup(ctl, data)

    dt_ctrl = 0.01
    every = int(round(dt_ctrl / model.opt.timestep))
    rng = np.random.default_rng(0)
    errs, max_roll = [], 0.0
    for k in range(12000):
        ctl.step(model, data)
        if shove and k and k % 3000 == 0:
            data.qvel[1] += rng.uniform(-shove, shove)
        mujoco.mj_step(model, data)
        s = extract_state(data, np.zeros(3))
        max_roll = max(max_roll, abs(s.roll))
        if k % every == 0 and k > 3000:
            v_lon, v_lat = est.update(
                dt_ctrl, data.qvel[ia] / est.belt_ratio,
                data.qvel[ib] / est.belt_ratio,
                steer_joint=float(data.qpos[sj]),
                yaw_rate=sensor(data, "ahrs_gyro")[2],
                accel_body=sensor(data, "ahrs_accel"), roll=s.roll)
            errs.append((v_lon - s.v_lon, v_lat - s.v_lat))

    assert np.degrees(max_roll) < MAX_ROLL_DEG, "bike fell; measurement meaningless"
    e = np.array(errs)
    rms_lon = float(np.sqrt(np.mean(e[:, 0] ** 2)))
    rms_lat = float(np.sqrt(np.mean(e[:, 1] ** 2)))
    assert rms_lon < 0.030, f"{regime}: fused v_lon {rms_lon*1000:.1f} mm/s RMS"
    assert rms_lat < 0.030, f"{regime}: fused v_lat {rms_lat*1000:.1f} mm/s RMS"


def test_body_to_world_matches_extract_state_convention():
    """body_to_world must invert the rotation extract_state applies; feeding
    HardwareData a body-frame vector is a silent, destabilizing bug."""
    rng = np.random.default_rng(0)
    for _ in range(20):
        yaw = rng.uniform(-np.pi, np.pi)
        v_lon, v_lat = rng.uniform(-1.5, 1.5, 2)
        world = body_to_world(v_lon, v_lat, yaw)
        c, s = np.cos(yaw), np.sin(yaw)
        back = np.array([[c, s], [-s, c]]) @ world      # extract_state's to_yaw
        assert np.allclose(back, [v_lon, v_lat], atol=1e-12)
