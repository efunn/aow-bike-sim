"""The TM151 error model reproduces its datasheet.

Every number asserted here is traceable to
`docs/ahrs/TransducerM_TM151_TM171_Datasheet_EN_V116-R.pdf`, TM151 column.
The point is not that the model is noisy -- it is that it is noisy BY THE
RIGHT AMOUNT, because the whole argument for what the AHRS costs rests on
these magnitudes.
"""

import mujoco
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control.linearize import settle_upright
from aow_sim.sim_ahrs import (ACCEL_MISALIGN_DEG, GYRO_BIAS_STABILITY_DPH,
                              GYRO_NOISE_PP_DPS, LEVELS, ORIENT_RMS_DEG,
                              PP_TO_SIGMA, SimAhrs, _gm_step)

# Sensor error model over the built model's AHRS sensors.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.geometry


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params)


@pytest.fixture(scope="module")
def data(model):
    d = mujoco.MjData(model)
    d.qpos[:] = settle_upright(model).qpos
    mujoco.mj_forward(model, d)
    return d


def _rpy(q):
    w, x, y, z = q
    return np.array([np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                     np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
                     np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))])


def _run(ahrs, data, n, dt=0.01):
    out = {"quat": [], "gyro": [], "accel": []}
    for _ in range(n):
        s = ahrs.sample(data, dt)
        for k in out:
            out[k].append(s[k].copy())
    return {k: np.array(v) for k, v in out.items()}


def test_level_none_is_bit_for_bit_the_clean_sensor(model, params, data):
    """The default must not perturb anything: every policy trained before this
    module existed has to reproduce exactly."""
    a = SimAhrs(model, params, level="none")
    r = _run(a, data, 50)
    for name, key in (("ahrs_quat", "quat"), ("ahrs_gyro", "gyro"),
                      ("ahrs_accel", "accel")):
        adr, dim = a.adr[name]
        truth = np.array(data.sensordata[adr:adr + dim])
        assert np.array_equal(r[key], np.tile(truth, (50, 1)))


@pytest.mark.parametrize("level", ("tm151_static", "tm151"))
def test_orientation_rms_matches_the_datasheet(model, params, data, level):
    """Static <0.5 deg roll/pitch; TM151 dynamic <1.5 deg. The whole
    concern about the AHRS is that 1.5 deg is the size of the roll signal
    itself, so this magnitude has to be right."""
    a = SimAhrs(model, params, level=level, seed=3)
    adr, _ = a.adr["ahrs_quat"]
    true = _rpy(np.array(data.sensordata[adr:adr + 4]))
    r = _run(a, data, 120000)
    err = np.degrees(np.array([_rpy(q) for q in r["quat"]]) - true)
    for axis, spec in enumerate(ORIENT_RMS_DEG[level][:2]):     # roll, pitch
        rms = np.sqrt(np.mean(err[:, axis] ** 2))
        assert rms == pytest.approx(spec, rel=0.10), (axis, rms, spec)


def test_gyro_noise_sigma_is_the_peak_to_peak_spec(model, params, data):
    """<= +-0.5 deg/s peak-to-peak. p-p is window-dependent (see PP_TO_SIGMA),
    so sigma is what gets asserted, and the p-p is checked over the ~1 s window
    the spec's "sampled at 100Hz" implies."""
    a = SimAhrs(model, params, level="tm151", seed=5)
    adr, _ = a.adr["ahrs_gyro"]
    truth = np.array(data.sensordata[adr:adr + 3])
    r = _run(a, data, 40000)
    resid = np.degrees(r["gyro"] - truth)
    assert np.std(resid[:, 0]) == pytest.approx(
        GYRO_NOISE_PP_DPS * PP_TO_SIGMA, rel=0.05)
    pp = np.mean([np.ptp(resid[i:i + 100, 0]) for i in range(0, 39900, 100)])
    assert pp < GYRO_NOISE_PP_DPS, f"1 s peak-to-peak {pp:.3f} over spec"


def test_gyro_bias_wanders_but_stays_bounded(model, params, data):
    """Bias STABILITY 5.5 deg/h is an Allan floor, not a random walk: the
    process must stay near its sigma rather than walking away, or a long
    episode would accumulate a heading error the real part does not have."""
    a = SimAhrs(model, params, level="tm151", seed=7)
    _run(a, data, 60000)
    sigma = np.deg2rad(GYRO_BIAS_STABILITY_DPH / 3600.0)
    assert np.all(np.abs(a._gyro_bias) < 6 * sigma), a._gyro_bias


def test_accel_misalignment_is_fixed_per_power_on_not_per_tick(model, params):
    """A build error is a constant, not a noise source. Redrawing it every tick
    would invent a disturbance the hardware does not have."""
    a = SimAhrs(model, params, level="tm151", seed=11)
    first = a._accel_tilt.copy()
    a.reset(seed=11)
    assert np.array_equal(a._accel_tilt, first), "same seed must repeat"
    assert np.all(np.abs(np.degrees(first)) <= ACCEL_MISALIGN_DEG)
    a.reset(seed=12)
    assert not np.array_equal(a._accel_tilt, first), "different seed must differ"


def test_gauss_markov_is_stationary_and_correlated():
    """The error must WANDER, not walk away and not be white. Both failure
    modes are easy for a controller: a constant offset gets trimmed out, and
    white noise averages away. The damage is in between."""
    rng = np.random.default_rng(0)
    dt, tau, sigma = 0.01, 2.0, 1.0
    x, xs = 0.0, []
    for _ in range(400000):
        x = float(_gm_step(np.array(x), dt, tau, sigma, rng))
        xs.append(x)
    xs = np.array(xs)
    assert np.std(xs) == pytest.approx(sigma, rel=0.05)      # stationary
    lag = int(tau / dt)
    c = np.corrcoef(xs[:-lag], xs[lag:])[0, 1]
    assert c == pytest.approx(np.exp(-1.0), abs=0.05)        # correlated


def test_every_level_is_reachable_and_unknown_ones_raise(model, params):
    for lvl in LEVELS:
        SimAhrs(model, params, level=lvl)
    with pytest.raises(ValueError, match="level must be"):
        SimAhrs(model, params, level="realistic")


def test_tick_samples_on_the_ahrs_clock_not_the_callers(model, params, data):
    """The error process is defined in SECONDS (tau), so it has to advance on
    the sensor's clock. Sampling at the caller's rate would make `tau` mean
    something different in teleop (2500 Hz physics) than in training (50 Hz),
    which is exactly the bug the odometry side had."""
    a = SimAhrs(model, params, level="tm151", seed=0, hz=100.0)
    dt = 0.0004                      # the physics step
    seen, prev = 0, None
    for _ in range(6000):            # 2.4 s of sim time
        a.tick(data, dt)
        cur = a.latest("ahrs_quat").copy()
        if prev is None or not np.array_equal(cur, prev):
            seen += 1
        prev = cur
    assert seen == pytest.approx(240, abs=3), seen


def test_channels_isolate_where_the_damage_comes_from(model, params, data):
    """`orient` must leave the rates alone and `gyro` the attitude, or the
    attribution measured off them means nothing."""
    adr_q, _ = SimAhrs(model, params).adr["ahrs_quat"]
    adr_g, _ = SimAhrs(model, params).adr["ahrs_gyro"]
    true_q = np.array(data.sensordata[adr_q:adr_q + 4])
    true_g = np.array(data.sensordata[adr_g:adr_g + 3])

    o = SimAhrs(model, params, level="tm151", seed=2, channels="orient")
    r = _run(o, data, 400)
    assert np.array_equal(r["gyro"], np.tile(true_g, (400, 1)))
    assert not np.allclose(r["quat"], true_q)

    g = SimAhrs(model, params, level="tm151", seed=2, channels="gyro")
    r = _run(g, data, 400)
    assert np.array_equal(r["quat"], np.tile(true_q, (400, 1)))
    assert not np.allclose(r["gyro"], true_g)


def test_tm171_is_strictly_better_than_the_tm151_on_every_axis(model, params):
    """The upgrade level exists to price a purchasing decision, so it must not
    be accidentally worse anywhere."""
    from aow_sim.sim_ahrs import (MISALIGN_DEG, ORIENT_RMS_DEG,
                                  YAW_DRIFT_DEG_PER_S)
    for a, b in zip(ORIENT_RMS_DEG["tm171"], ORIENT_RMS_DEG["tm151"]):
        assert a <= b
    assert YAW_DRIFT_DEG_PER_S["tm171"] < YAW_DRIFT_DEG_PER_S["tm151"]
    assert MISALIGN_DEG["tm171"] < MISALIGN_DEG["tm151"]


def test_the_parts_differ_in_DYNAMIC_roll_pitch_which_is_the_easy_one_to_miss():
    """Pins the datasheet reading the whole tm171 comparison rests on.

    The page-5 performance table has TWO ROWS per block, tagged in the yaw
    column: static is <0.5/<0.5/<1.0 (TM151) then <0.5/<0.5/<0.8 (TM171);
    dynamic is <1.5/<1.5/3.0deg-per-25min (TM151) then <1.0/<1.0/2.6 (TM171).

    So roll and pitch are IDENTICAL between the parts when static and differ by
    1.5 vs 1.0 deg when dynamic. That is easy to miss -- the static block hides
    it -- and it is the only difference that moves the eval grid. Measured by
    ablation: swapping just this row recovers 0.537 -> 0.635 of the 0.689 the
    full TM171 scores, while swapping yaw drift alone (0.542) or misalignment
    alone (0.519) recovers nothing.
    """
    from aow_sim.sim_ahrs import ORIENT_RMS_DEG, TM171_STATIC_RMS_DEG
    assert ORIENT_RMS_DEG["tm151_static"][:2] == (0.5, 0.5)
    assert ORIENT_RMS_DEG["tm151"][:2] == (1.5, 1.5)
    assert ORIENT_RMS_DEG["tm171"][:2] == (1.0, 1.0)

    # STATIC: the two parts are IDENTICAL in roll and pitch. This is what makes
    # them look interchangeable, and why there is no "tm171_static" level.
    assert TM171_STATIC_RMS_DEG[:2] == ORIENT_RMS_DEG["tm151_static"][:2]
    # DYNAMIC: this is the only place they separate, and the only difference
    # that moved the eval grid.
    assert ORIENT_RMS_DEG["tm151"][:2] > ORIENT_RMS_DEG["tm171"][:2]
    # Yaw separates in BOTH blocks, and is measured to change nothing.
    assert TM171_STATIC_RMS_DEG[2] < ORIENT_RMS_DEG["tm151_static"][2]
