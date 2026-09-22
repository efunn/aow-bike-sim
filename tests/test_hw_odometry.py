"""hw/odometry.py's estimator, on kinematically consistent inputs. No sim.

Each test builds the sensor readings a rigid bike with NO SLIP would produce
-- hub and ring rates, steer joint, gyro, specific force -- and requires the
estimator to hand back the velocity they were built from, exactly. So a red
here is the estimator's CODE: its gearing arithmetic, its raked-axis
geometry, its fusion. Nothing about the contact model, the controller or the
policy can move it.

What this deliberately does NOT cover, and where that went (2026-09-21):

  * whether the simulated wheels actually roll without slipping, and what
    the estimate is worth closed-loop -- tests/test_odometry_in_the_loop.py.
    The open-loop "estimate vs sim truth" tests that used to live here were
    measuring that slip and reporting it as estimator error; under the RL
    policy 5-7 mm/s of a 51-109 mm/s longitudinal error was this module's
    arithmetic and the rest was the wheel.
  * the free fit of the tan / wheelbase coefficients -- retired. On a
    trajectory where yaw_rate ~ v_lon*tan(theta)/L the two regressors are one
    signal, so it measured the conditioning of its own fixture.
"""

import numpy as np
import pytest

from aow_sim.build_model import load_params
from aow_sim.control.steer import steer_for_heading, wheel_heading
from aow_sim.hw.odometry import GRAVITY, VelocityEstimator, body_to_world

# No model build: the estimator is plain arithmetic on its inputs.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.pure

DT = 0.01       # the Pi's 100 Hz control tick


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def est(params):
    return VelocityEstimator(params)


class _Geom:
    """The bike's geometry read STRAIGHT FROM bike_params, never from the
    estimator. Building the inputs from `est.rake` / `est.wheelbase` would use
    the same constant on both sides and cancel any error in it -- measured:
    a zeroed rake, a 2% gearing error and a 5% wheelbase error all passed
    that way."""
    def __init__(self, p):
        self.r = float(p["omni_wheel"]["outer_radius"])
        self.mix_a = float(p["drivetrain"]["mix_hub_a"])
        self.mix_b = float(p["drivetrain"]["mix_hub_b"])
        self.belt = float(p["drivetrain"]["belt_ratio"])
        self.L = float(p["bike"]["wheelbase"])
        self.rake = np.deg2rad(float(p["bike"]["rake_deg"]))


@pytest.fixture(scope="module")
def geom(params):
    return _Geom(params)


def _servo_rates(g, v_lon, differential):
    """Servo-side rates for a hub rolling at v_lon with a given input-shaft
    differential (the part that drives the rollers, not the hub)."""
    w_hub = v_lon / g.r
    # hub = mix_a * w_a + mix_b * w_b; split the remainder as +-differential
    w_a = (w_hub + g.mix_b * differential) / (g.mix_a + g.mix_b)
    w_b = (w_hub - g.mix_a * differential) / (g.mix_a + g.mix_b)
    return w_a / g.belt, w_b / g.belt


def _no_slip_lateral(v_front, theta, yaw_rate, wheelbase):
    """Rear-contact (v_lon, v_lat) of a rigid bike whose front contact moves
    at `v_front` along ground heading `theta`: v_front_body = v_rear +
    yaw_rate x (L, 0). Derived from the geometry, not from the estimator."""
    return v_front * np.cos(theta), v_front * np.sin(theta) - yaw_rate * wheelbase


# -- longitudinal -----------------------------------------------------------

@pytest.mark.parametrize("v_lon", [-1.2, -0.3, 0.0, 0.25, 0.6, 1.2])
@pytest.mark.parametrize("differential", [-8.0, 0.0, 3.0])
def test_longitudinal_is_the_hub_kinematics(est, geom, v_lon, differential):
    """Any split of the two inputs that rolls the hub at v_lon reads v_lon:
    the roller differential must not leak into forward speed."""
    wa, wb = _servo_rates(geom, v_lon, differential)
    assert est.longitudinal(wa, wb) == pytest.approx(v_lon, abs=1e-12)


def test_a_pure_differential_reads_zero_forward_speed(est, geom):
    wa, wb = _servo_rates(geom, 0.0, 5.0)
    assert wa != 0.0 and wb != 0.0
    assert est.longitudinal(wa, wb) == pytest.approx(0.0, abs=1e-12)


# -- lateral, front-wheel constraint ----------------------------------------

@pytest.mark.parametrize("theta_deg", [-60.0, -20.0, 0.0, 5.0, 35.0, 70.0])
@pytest.mark.parametrize("yaw_rate", [-1.5, 0.0, 0.8])
@pytest.mark.parametrize("v_front", [-0.6, 0.4, 1.1])
def test_front_constraint_recovers_the_rigid_body_lateral_velocity(
        est, geom, theta_deg, yaw_rate, v_front):
    """Built from the front contact's velocity and the yaw lever arm, through
    the RAKED steer axis (steer_for_heading), so a rake or sign error in
    wheel_heading shows up here rather than as a mystery lateral bias."""
    theta = np.deg2rad(theta_deg)
    v_lon, v_lat = _no_slip_lateral(v_front, theta, yaw_rate, geom.L)
    steer = steer_for_heading(theta, geom.rake)
    assert wheel_heading(steer, geom.rake) == pytest.approx(theta, abs=1e-12)
    got, conf = est.lateral_from_front(v_lon, steer, yaw_rate)
    assert got == pytest.approx(v_lat, abs=1e-9)
    assert conf > 0.0


def test_confidence_collapses_when_front_wheel_is_perpendicular(est):
    """At theta -> +-90 deg the front wheel constrains nothing, and the
    estimator must say so rather than emit a huge tan()."""
    _, conf_straight = est.lateral_from_front(0.5, 0.0, 0.0)
    _, conf_45 = est.lateral_from_front(0.5, np.deg2rad(45), 0.0)
    _, conf_90 = est.lateral_from_front(0.5, np.deg2rad(90), 0.0)
    assert conf_straight > 0.9
    assert 0.0 < conf_45 < conf_straight
    assert conf_90 == 0.0

    v, _ = est.lateral_from_front(0.5, np.deg2rad(90), 0.0)
    assert np.isfinite(v), "must not return inf/nan at the singularity"


def test_wound_steer_angle_reads_the_same_as_its_wrapped_equivalent(est):
    """The front wheel is symmetric, so a multi-turn angle must give the same
    lateral estimate as its wrapped equivalent -- no pi-rebasing needed."""
    for delta in (0.2, -0.35, 0.9):
        base, _ = est.lateral_from_front(0.6, delta, 0.4)
        for winding in (np.pi, -np.pi, 2 * np.pi, 6 * np.pi):
            wound, _ = est.lateral_from_front(0.6, delta + winding, 0.4)
            assert np.isclose(base, wound, atol=1e-9), (
                f"steer {delta}+{winding} read differently from {delta}")


# -- the fused update --------------------------------------------------------

def _steady_turn(g, v_front, theta, yaw_rate, roll, pitch):
    """Every input of `update` for a steady no-slip turn at a fixed lean."""
    v_lon, v_lat = _no_slip_lateral(v_front, theta, yaw_rate, g.L)
    wa, wb = _servo_rates(g, v_lon, 0.0)
    # specific force: gravity through the lean, plus the centripetal term the
    # estimator subtracts (v_lon * yaw_rate); nothing else in a steady turn
    accel = np.array([
        -GRAVITY * np.sin(pitch),
        GRAVITY * np.sin(roll) * np.cos(pitch) + v_lon * yaw_rate,
        GRAVITY * np.cos(roll) * np.cos(pitch)])
    steer = steer_for_heading(theta, g.rake)
    return (v_lon, v_lat), dict(w_servo_a=wa, w_servo_b=wb, steer_joint=steer,
                                yaw_rate=yaw_rate, accel_body=accel,
                                roll=roll, pitch=pitch)


@pytest.mark.parametrize("theta_deg,yaw_rate,roll_deg", [
    (0.0, 0.0, 0.0), (12.0, 0.6, -4.0), (-30.0, -1.1, 7.0), (55.0, 0.3, 2.0)])
def test_fused_update_converges_to_the_truth_on_consistent_inputs(
        params, geom, theta_deg, yaw_rate, roll_deg):
    """From a wrong start, ticked at 100 Hz, it lands on the velocity the
    inputs were built from. Where confidence < 1 it gets there geometrically,
    which is what the 300 ticks allow for."""
    est = VelocityEstimator(params)
    est.reset(0.3, -0.2)
    truth, kw = _steady_turn(geom, 0.5, np.deg2rad(theta_deg), yaw_rate,
                             np.deg2rad(roll_deg), np.deg2rad(1.5))
    for _ in range(300):
        got = est.update(DT, **kw)
    assert got == pytest.approx(truth, abs=1e-9)


def test_the_accelerometer_never_touches_v_lon(params, geom):
    """Integrating specific force into v_lon cost 8.8 -> 174 mm/s RMS through
    the AHRS lever-arm terms (hw/odometry.py). It must not come back."""
    est = VelocityEstimator(params)
    truth, kw = _steady_turn(geom, 0.6, 0.0, 0.0, 0.0, 0.0)
    for junk in ([25.0, 0, 9.81], [-40.0, 3.0, 0.0], [0.0, 0, 30.0]):
        est.reset()
        kw["accel_body"] = np.array(junk)
        v_lon, _ = est.update(DT, **kw)
        assert v_lon == pytest.approx(truth[0], abs=1e-12)


def test_at_full_confidence_the_accelerometer_is_overruled(params, geom):
    """At conf 1 the correction adopts the constraint outright, so a bad
    lateral accelerometer reading changes nothing."""
    est = VelocityEstimator(params)
    truth, kw = _steady_turn(geom, 0.5, 0.0, 0.4, 0.0, 0.0)
    kw["accel_body"] = kw["accel_body"] + np.array([0.0, 12.0, 0.0])
    _, conf = est.lateral_from_front(truth[0], kw["steer_joint"], kw["yaw_rate"])
    assert conf == pytest.approx(1.0)
    _, v_lat = est.update(DT, **kw)
    assert v_lat == pytest.approx(truth[1], abs=1e-12)


def test_a_blind_front_wheel_coasts_on_acceleration(params, geom):
    """Perpendicular front wheel, confidence 0: v_lat is the integral of the
    gravity- and centripetal-corrected lateral acceleration, and nothing else."""
    est = VelocityEstimator(params)
    est.reset(0.4, 0.05)
    _, kw = _steady_turn(geom, 0.4, 0.0, 0.2, np.deg2rad(3.0), 0.0)
    kw["steer_joint"] = steer_for_heading(np.pi / 2, geom.rake)
    a_lat = 0.3
    kw["accel_body"] = kw["accel_body"] + np.array([0.0, a_lat, 0.0])
    n = 50
    for _ in range(n):
        _, v_lat = est.update(DT, **kw)
    assert est.confidence == 0.0
    assert v_lat == pytest.approx(0.05 + a_lat * DT * n, abs=1e-12)


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
