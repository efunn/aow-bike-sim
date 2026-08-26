"""The simulated estimator: its own clock, and the encoder model.

`SimOdometry` runs `hw/odometry.VelocityEstimator` against the model's own
sensors. Two things about it are easy to get wrong and invisible when they are:

  * IT HAS ITS OWN TICK RATE. The estimator INTEGRATES, so the rate it is
    ticked at changes what it does. Before 2026-08-27 it inherited whatever its
    caller looped at -- 50 Hz from GeneralEnv, 2500 Hz from teleop, against the
    Pi's 100 Hz -- which meant three callers ran three different estimators.
  * THE ENCODER MODEL IS OPTIONAL. "ideal" reads instantaneous joint velocity
    and is a floor on the error; "counts" reproduces the hardware path.
"""

import mujoco
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control.drive import DriveController
from aow_sim.control.linearize import settle_upright
from aow_sim.control.steer import XC330_COUNTS_PER_RAD
from aow_sim.sim_odometry import ENCODERS, SimOdometry

# Rides the contact model: the numbers come from a stepped bike.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.contact


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params)


def _settled(model):
    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    mujoco.mj_forward(model, data)
    return data


def test_estimator_ticks_at_its_own_rate_not_the_callers(model, params):
    """Ticks are counted off `odo_hz`, whatever dt the caller hands over."""
    for hz in (50.0, 100.0, 500.0):
        data = _settled(model)
        odo = SimOdometry(model, params, odo_hz=hz)
        ticks, prev = 0, odo._last
        for _ in range(2000):                     # 2000 * 4e-4 = 0.8 s
            mujoco.mj_step(model, data)
            odo.update(data, model.opt.timestep)
            if odo._last != prev:
                ticks, prev = ticks + 1, odo._last
        expected = 2000 * model.opt.timestep * hz
        assert abs(ticks - expected) <= 2, (
            f"{hz} Hz: {ticks} ticks over 0.8 s, expected ~{expected:.0f}")


def test_value_is_HELD_between_ticks(model, params):
    """Between sense ticks every reader gets the same numbers, as on the bike.
    A caller looping faster than `odo_hz` must not see the estimate change."""
    data = _settled(model)
    odo = SimOdometry(model, params, odo_hz=100.0)
    mujoco.mj_step(model, data)
    odo.update(data, 0.01)                        # force one tick
    held = odo._last
    for _ in range(10):                           # 10 * 4e-4 = 4 ms < 10 ms
        mujoco.mj_step(model, data)
        assert odo.update(data, model.opt.timestep) == held


def test_zero_dt_reads_without_advancing(model, params):
    """`update(data, 0.0)` is the read GeneralEnv._obs does after the substep
    loop has already advanced the estimator. It must not tick."""
    data = _settled(model)
    odo = SimOdometry(model, params)
    mujoco.mj_step(model, data)
    odo.update(data, 0.01)
    held = odo._last
    for _ in range(5):
        assert odo.update(data, 0.0) == held


def test_counts_encoder_quantizes_to_the_servo_resolution(model, params):
    """One count is 2*pi/4096 at the SERVO, i.e. belt_ratio behind the input
    shaft -- which is what puts the encoder on the slow side and buys the
    aliasing margin. Quantising at the input shaft instead would be 3x finer
    and wrong."""
    data = _settled(model)
    odo = SimOdometry(model, params, encoder="counts")
    belt = odo.est.belt_ratio
    for _ in range(50):
        mujoco.mj_step(model, data)
    c = odo._counts(data, "a")
    rad_input = float(odo._read(data, "input_a_pos")[0])
    assert c == round(rad_input / belt * XC330_COUNTS_PER_RAD)
    # and one count really is ~0.236 mm of travel at the wheel
    arc = (2 * np.pi / 4096) * belt * params["omni_wheel"]["outer_radius"]
    assert arc == pytest.approx(0.0002356, abs=1e-6)


def test_both_encoders_agree_on_ONE_trajectory(model, params):
    """`counts` is noisier -- that is the point of it -- but it must not be
    BIASED, or v_lon inherits a scale error the estimator cannot see.

    BOTH ESTIMATORS ARE TICKED ON THE SAME `data`, which is the only way to
    compare them. Two separate runs diverge (the estimate feeds the
    controller), and forcing `qvel` each step does not work either: the
    velocity SENSOR then reports the forced value while position-differencing
    reports the travel the solver actually allowed, so they disagree by however
    much the constraint solver bled off -- an artifact of the harness, not of
    the encoder.
    """
    data = _settled(model)
    ctl = DriveController(params, model)
    ctl.reset(model, data)
    ctl.set_speed(0.6)
    both = {e: SimOdometry(model, params, encoder=e) for e in ENCODERS}
    vs = {e: [] for e in ENCODERS}
    for k in range(6000):
        ctl.step(model, data)
        mujoco.mj_step(model, data)
        for e, odo in both.items():
            odo.update(data, model.opt.timestep)
            if k > 3000:
                vs[e].append(odo._last[0])
    mu = {e: float(np.mean(v)) for e, v in vs.items()}
    sd = {e: float(np.std(v)) for e, v in vs.items()}
    for e in ENCODERS:
        assert mu[e] == pytest.approx(mu["ideal"], abs=0.015), mu

    # AND "counts" IS THE QUIETER OF THE TWO, which is the opposite of what
    # "adds quantisation noise" suggests, so it is pinned here rather than
    # left to surprise someone: measured 15.2 mm/s of spread on `ideal`
    # against 10.8 on `counts`. `ideal` is RAW instantaneous joint velocity
    # with no filter at all, while `counts` goes through the same 25 ms
    # RateFilter the Pi runs -- and that filter removes more than the 4096
    # counts/rev quantisation puts in (one count is 0.236 mm at the wheel,
    # worth q/T = 9.4 mm/s over a 25 ms span).
    #
    # So "ideal" is not a better sensor, it is an UNFILTERED one. It is a floor
    # on ERROR, not on noise, and the trade `counts` makes is variance for lag.
    assert sd["counts"] < sd["ideal"], sd


def test_unknown_encoder_is_refused(model, params):
    with pytest.raises(ValueError, match="encoder must be one of"):
        SimOdometry(model, params, encoder="magic")


def test_reset_clears_the_filters_not_just_the_estimator(model, params):
    """The RateFilter and the previous counts are estimator state too. An
    episode that inherits them is not the episode the bike will fly."""
    data = _settled(model)
    odo = SimOdometry(model, params, encoder="counts")
    for _ in range(200):
        mujoco.mj_step(model, data)
        odo.update(data, model.opt.timestep)
    assert odo._prev_counts, "nothing recorded to clear"
    odo.reset(model, params)
    assert odo._prev_counts == {}
    assert odo._last == (0.0, 0.0)
    assert all(f.peek() == 0.0 for f in odo._filt.values())


def test_reported_is_the_servos_own_estimate_and_lags_three_times_ours(model,
                                                                      params):
    """`reported` models Present Velocity(128), which ServoBus can take
    wholesale via velocity_source="reported". Same counts, different filter:
    the servo smooths like a ~50 ms BOXCAR against our 25 ms / taper 0.5.

    The lag ratio is the whole reason hw/dynamixel.py re-derives velocity from
    position instead of reading the register, so it is pinned here rather than
    left in prose. Measured closed-loop it is not free: general_rl_odo holds
    survival 1.00 on `counts` and drops to 0.85 on `reported`.
    """
    ours = SimOdometry(model, params, encoder="counts")
    servo = SimOdometry(model, params, encoder="reported")
    lag = {k: list(o._filt.values())[0].group_delay_ms
           for k, o in (("counts", ours), ("reported", servo))}
    assert lag["counts"] == pytest.approx(8.3, abs=0.1)
    assert lag["reported"] == pytest.approx(25.0, abs=0.1)
    assert lag["reported"] > 2.5 * lag["counts"]
    # A boxcar has no taper by definition; ours ramps to half weight.
    assert servo.taper == 1.0 and ours.taper == 0.5


def test_explicit_filter_args_override_the_encoder_table(model, params):
    """The encoder picks the filter, but an explicit window/taper wins -- which
    is what lets the lag be SWEPT without inventing an encoder name per point.
    Silently ignoring these args would make a sweep read as a flat line."""
    o = SimOdometry(model, params, encoder="counts", window_ms=50.0, taper=1.0)
    assert (o.window_ms, o.taper) == (50.0, 1.0)
    assert list(o._filt.values())[0].group_delay_ms == pytest.approx(25.0, abs=0.1)
    # and the table still applies when they are not given
    d = SimOdometry(model, params, encoder="counts")
    assert (d.window_ms, d.taper) == (25.0, 0.5)
