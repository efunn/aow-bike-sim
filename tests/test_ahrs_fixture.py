"""The AHRS fixture's analysis, against a synthetic rig whose answer is known.

`analysis/ahrs_fixture.py` scores the TM151 against two servo encoders, and
everything it reports rests on three fitted things: the joint axes in the
sensor frame, the sensor's delay, and the device-to-host clock map. A sign or
a transpose wrong in any of them still produces plausible-looking degrees, so
this builds a rig with a skewed mount, a tilted base, a known delay, arrival
jitter and a known injected error, and checks each comes back.

Invalidated by: the analysis in `analysis/ahrs_fixture.py`, or
`tm151_serial.py`'s quaternion convention (sensor -> earth).
"""

import sys

import numpy as np
import pytest

sys.path.insert(0, "analysis")
import ahrs_fixture as fx  # noqa: E402

pytestmark = pytest.mark.pure

RATE = 200.0
LAG = 0.012                      # the sensor's delay, s
ERR_DEG = (0.8, 0.5)             # injected roll / pitch error amplitude
# Device clock fast by this fraction. The real one is 0.3% (2026-09-23); 2.5%
# makes this 67 s rig walk the raw clock offset 1.7 s, well past the 0.5 s
# median gate that once cut the reference hold off a 6 min session.
DRIFT = 2.5e-2
R_LEVER = np.array([0.004, -0.003, 0.150])   # sensing point, sensor frame [m]


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _mat_to_quat(R):
    w = np.sqrt(np.maximum(1 + np.trace(R, axis1=1, axis2=2), 1e-12)) / 2
    return np.stack([w, (R[:, 2, 1] - R[:, 1, 2]) / (4 * w),
                     (R[:, 0, 2] - R[:, 2, 0]) / (4 * w),
                     (R[:, 1, 0] - R[:, 0, 1]) / (4 * w)], 1)


CHIRP = {"kind": "chirp", "axis": "roll", "amp_deg": 8.0, "f0": 0.2, "f1": 6.4,
         "a_max": 3000.0, "rate_max": 300.0}


def _rig(seed=0, chirp=None, flip=False, lever=None, shake=None):
    """`chirp`: None, or {"gain", "delay"} -- a 40 s roll chirp replaces the
    sway, the injected error is off, and during it the AHRS reports the roll
    joint scaled by `gain` and `delay` s later than everything else.
    `flip`: the sensor turned 180 deg about its own x on the mount -- upside
    down, as the d mount turned over leaves it. `lever`: the sensing point
    instead of R_LEVER. `shake`: extra acceleration at the sensor [m/s^2,
    earth frame], a function of (t, alpha_world) -- what a plate rattling in
    its play adds and the rigid-body model does not have."""
    rng = np.random.default_rng(seed)
    body = ("chirp", 40) if chirp else ("sway", 40)
    plan = [("ref", 5), ("ident_yaw", 6), ("settle", 2), ("ident_roll", 6),
            ("settle", 3), body, ("ref_end", 5)]
    segs, t0 = [], 0.0
    for label, sec in plan:
        segs.append({"label": label, "t0": t0, "t1": t0 + sec,
                     "info": CHIRP if label == "chirp" else {}})
        t0 += sec
    T = t0

    def joints(t):
        psi, phi = np.zeros_like(t), np.zeros_like(t)
        for s in segs:
            m = (t >= s["t0"]) & (t < s["t1"])
            tt = t[m] - s["t0"]
            env = fx._smooth(tt / 1.0) * fx._smooth((s["t1"] - s["t0"] - tt) / 1.0)
            if s["label"] == "ident_yaw":
                psi[m] = np.radians(15) * env * np.sin(np.pi * tt)
            elif s["label"] == "ident_roll":
                phi[m] = np.radians(15) * env * np.sin(np.pi * tt)
            elif s["label"] == "sway":
                phi[m] = np.radians(15) * env * np.sin(2 * np.pi * 0.7 * tt) \
                    * np.cos(2 * np.pi * 0.13 * tt)
                psi[m] = np.radians(20) * env * np.sin(2 * np.pi * 0.31 * tt + 1)
            elif s["label"] == "chirp":
                seg = fx.chirp("roll", CHIRP["amp_deg"], CHIRP["f0"], CHIRP["f1"],
                               s["t1"] - s["t0"], CHIRP["a_max"], CHIRP["rate_max"])
                phi[m] = [seg.command(x, None)[fx.ROLL] - fx.CENTER for x in tt]
        return psi, phi

    # Mount: neither axis lines up with a sensor axis, nor are they square.
    u_y = _unit([0.08, -0.12, 1.0])
    u_r = _unit([1.0, 0.1, 0.05])
    R0 = (fx.axis_angle([0, 0, 1], [np.radians(40)])[0]
          @ fx.axis_angle([1, 0, 0], [np.radians(3)])[0]
          @ fx.axis_angle([0, 1, 0], [np.radians(-2)])[0])
    if flip:                         # sensor s = F s': the axes and R0 in the new frame
        F = fx.axis_angle([1.0, 0, 0], [np.pi])[0]
        u_y, u_r, R0 = F @ u_y, F @ u_r, R0 @ F
    r_lever = R_LEVER if lever is None else np.asarray(lever, float)

    # Servo frames: quantised encoders, host-timed.
    t_e = np.arange(0, T, 1 / RATE) + 0.0013
    psi_e, phi_e = joints(t_e)
    q = 2 * np.pi / 4096
    servo = {"t": t_e, "yaw": np.round(psi_e / q) * q, "roll": np.round(phi_e / q) * q}

    # AHRS: sampled on its own clock, reporting the truth LAG seconds old,
    # plus an injected error in the heading frame, arriving with jitter.
    tau = np.arange(0.2, T - 0.2, 1 / RATE)
    psi_a, phi_a = joints(tau - LAG)
    R_t = fx.truth_mats(R0, u_y, u_r, psi_a, phi_a)
    h = np.arctan2(R_t[:, 1, 0], R_t[:, 0, 0])
    k_err = 0.0 if chirp else 1.0
    er = k_err * np.radians(ERR_DEG[0]) * np.sin(2 * np.pi * 0.4 * tau)
    ep = k_err * np.radians(ERR_DEG[1]) * np.cos(2 * np.pi * 0.23 * tau)
    e_w = np.stack([np.cos(h) * er - np.sin(h) * ep,
                    np.sin(h) * er + np.cos(h) * ep, np.zeros_like(tau)], 1)
    ang = np.linalg.norm(e_w, axis=1)
    R_e = np.stack([fx.axis_angle(e / a if a > 0 else [1, 0, 0], [a])[0]
                    for e, a in zip(e_w, ang)])
    R_a = R_e @ R_t
    if chirp:
        # What the sensor REPORTS during the chirp; the physics (gyro, accel
        # below) stays the truth's.
        c = segs[5]
        m = (tau - LAG >= c["t0"]) & (tau - LAG < c["t1"])
        _, phi_rep = joints(tau - LAG - chirp["delay"])
        R_a[m] = fx.truth_mats(R0, u_y, u_r, psi_a[m], chirp["gain"] * phi_rep[m])

    # Gyro: body rate of the truth, from its finite difference.
    dpsi = np.gradient(psi_a, tau)
    dphi = np.gradient(phi_a, tau)
    omega = np.stack([fx.axis_angle(u_r, -phi_a[k:k + 1])[0] @ u_y * dpsi[k]
                      + u_r * dphi[k] for k in range(len(tau))])
    gyro = omega + rng.normal(0, np.radians(0.25), omega.shape)

    # Accelerometer: the sensing point R_LEVER from the axes' intersection,
    # reading gravity minus its own acceleration (the TM151's sign), in g.
    p_w = np.einsum("nij,j->ni", R_t, r_lever)
    a_w = np.gradient(np.gradient(p_w, tau, axis=0), tau, axis=0)
    if shake is not None:
        alpha_w = np.gradient(np.einsum("nij,nj->ni", R_t, omega), tau, axis=0)
        a_w = a_w + shake(tau, alpha_w)
    a_s = np.einsum("nji,nj->ni", R_t, a_w)
    acc_g = fx.gravity_sensor(R_t) - a_s / fx.G_VENDOR \
        + rng.normal(0, 0.0015, a_s.shape)

    # The device clock runs FAST, as the real one does.
    t_us = (tau * (1 + DRIFT) * 1e6 + 12_345_678).astype(np.int64) % (1 << 32)
    t_host = tau + 0.0011 + rng.exponential(0.0004, len(tau))
    # The stale first frame from the previous session.
    t_us[0] = (t_us[1] - 7_000_000) % (1 << 32)
    ahrs = {"t_host": t_host, "t_us": t_us, "quat": _mat_to_quat(R_a), "gyro": gyro,
            "acc_g": acc_g}
    return servo, ahrs, segs, u_y, u_r


@pytest.fixture(scope="module")
def result():
    servo, ahrs, segs, u_y, u_r = _rig()
    return fx.analyse(servo, ahrs, segs), u_y, u_r


def test_the_sensor_delay_comes_back(result):
    res, *_ = result
    assert res["ident"]["lag_ms"] == pytest.approx(1e3 * LAG, abs=1.0)
    assert not res["ident"]["lag_at_edge"]


def test_the_joint_axes_come_back_in_the_sensor_frame(result):
    res, u_y, u_r = result
    for got, want in ((res["ident"]["u_yaw"], u_y), (res["ident"]["u_roll"], u_r)):
        assert np.degrees(np.arccos(np.clip(got @ want, -1, 1))) < 0.5
    assert res["ident"]["scale_yaw"] == pytest.approx(1.0, abs=0.02)
    assert res["ident"]["scale_roll"] == pytest.approx(1.0, abs=0.02)


def test_the_injected_error_is_what_gets_reported(result):
    """RMS of A sin is A/sqrt2 -- in roll and pitch separately, which is the
    check that the heading-frame split is not rotated or transposed."""
    res, *_ = result
    sway = next(r for r in res["segments"] if r["label"] == "sway")
    roll, pitch, heading = sway["rms_lag"]
    assert roll == pytest.approx(ERR_DEG[0] / np.sqrt(2), rel=0.1)
    assert pitch == pytest.approx(ERR_DEG[1] / np.sqrt(2), rel=0.1)
    assert heading < 0.1
    # and at rest, with no motion, nothing but the injected error
    ref = next(r for r in res["segments"] if r["label"] == "ref_end")
    assert np.hypot(*ref["rms_lag"][:2]) < 1.0


def test_as_received_carries_the_delay_on_top(result):
    res, *_ = result
    sway = next(r for r in res["segments"] if r["label"] == "sway")
    assert np.hypot(*sway["rms_rx"][:2]) > np.hypot(*sway["rms_lag"][:2])


def test_the_stale_first_frame_is_dropped_by_the_clock_fit(result):
    res, *_ = result
    assert res["clock"]["drift_ppm"] == pytest.approx(-1e6 * DRIFT / (1 + DRIFT), abs=50)
    assert res["clock"]["arrival_jitter_ms"]["p99"] < 5


def test_every_goal_is_clamped_inside_the_eeprom_travel():
    lo = fx.CENTER - np.radians(fx.LIMIT_DEG)
    hi = fx.CENTER + np.radians(fx.LIMIT_DEG)
    for y, r in ((90, -90), (-1e6, 1e6), (fx.SOFT_DEG + 1, 0)):
        g = fx.goal(y, r)
        assert all(lo < v < hi for v in g.values())
    assert fx.SOFT_DEG < fx.LIMIT_DEG


def test_the_lever_arm_comes_back(result):
    """The sensing point from its acceleration: the number that prices a
    mounting height, and the datasheet does not give the element's height."""
    res, u_y, u_r = result
    lv = res["lever"]
    assert np.linalg.norm(lv["r_m"] - R_LEVER) < 0.003
    want = np.linalg.norm(R_LEVER - (R_LEVER @ u_r) * u_r)
    assert lv["from_roll_axis_mm"] == pytest.approx(1e3 * want, abs=3)


def test_a_perp_is_the_lever_arm_acceleration(result):
    """At rest nothing but noise; in the sway, h * alpha from the roll."""
    res, *_ = result
    rows = {r["label"]: r for r in res["segments"]}
    assert rows["ref_end"]["a_perp_rms"] < 0.05
    assert rows["sway"]["a_perp_rms"] > 0.3


# --- the motion generators: every command inside the travel and the caps ----

def _sample(seg, rate=1000.0):
    t = np.arange(0, seg.seconds, 1 / rate)
    g = [seg.command(tt, None) for tt in t]
    return (t, np.degrees(np.array([x[fx.YAW] for x in g]) - fx.CENTER),
            np.degrees(np.array([x[fx.ROLL] for x in g]) - fx.CENTER))


def test_a_chirp_respects_its_acceleration_and_rate_caps():
    seg = fx.chirp("roll", 8.0, 0.1, 8.0, 60.0, a_max=3000.0, rate_max=300.0)
    t, yaw, roll = _sample(seg)
    assert np.all(yaw == 0)
    rate = np.gradient(roll, t)
    acc = np.gradient(rate, t)
    assert np.abs(rate).max() < 300 * 1.15      # the envelope adds a little
    assert np.abs(acc).max() < 3000 * 1.3
    assert np.abs(roll).max() == pytest.approx(8.0, abs=0.2)


def test_a_step_arrives_and_holds_at_the_asked_rate():
    seg = fx.step("roll", 10.0, 100.0, 5.0)
    t, _, roll = _sample(seg)
    assert roll[-1] == pytest.approx(10.0)
    assert np.abs(np.gradient(roll, t)).max() == pytest.approx(100.0, rel=0.02)


def test_the_yaw_highpass_fits_the_travel_and_keeps_the_rates():
    rng = np.random.default_rng(1)
    rate = 100.0
    t = np.arange(0, 60, 1 / rate)
    yaw = 120 * t / 60 + fx.band_noise(60, rate, 5.0, 0.5, 3.0, rng)[:len(t)]
    hp, fc = fx.fit_travel(yaw, rate, 35.0)
    assert np.abs(hp).max() <= 35.0
    r0, r1 = np.gradient(yaw, 1 / rate), np.gradient(hp, 1 / rate)
    assert np.std(r1) == pytest.approx(np.std(r0 - r0.mean()), rel=0.1)


def test_a_session_plan_builds_and_stays_inside_the_travel(tmp_path):
    rng = np.random.default_rng(2)
    n = 3000
    rep = tmp_path / "rep.npz"
    np.savez(rep, roll_deg=rng.normal(0, 2, 4 * n).astype(np.float32),
             yaw_deg=np.cumsum(rng.normal(0, 0.3, 4 * n)).astype(np.float32),
             start=np.arange(5) * n, seed=np.arange(4), fell=np.array([1, 0, 1, 0]),
             rate=np.array(100.0))

    class A:
        ident_deg, axis = 15.0, "roll"
        rest_seconds, sway_seconds = 10.0, 10.0
        step_amps, step_rate, step_hold = [5.0, -10.0], 100.0, 3.0
        chirp_amps, chirp_band, chirp_seconds = [8.0], [0.1, 8.0], 20.0
        a_max, max_rate = 3000.0, 300.0
        replay_file, flights, yaw_flights, yaw_limit = rep, None, None, 35.0
        roll_rms, yaw_rms, band, seed = 2.3, 0.0, [0.3, 5.0], 0
        parts = list(fx.PARTS)

    plan = fx.plan_session(A)
    labels = [s.label for s in plan.segments]
    assert labels[:2] == ["home", "ref"] and labels[-1] == "ref_end"
    assert {"rest", "sway"} <= set(labels)
    assert sum(lbl.startswith("replay_") for lbl in labels) == 4
    assert sum(lbl.startswith("yaw_") for lbl in labels) == 2
    for seg in plan.segments[1:]:                 # home needs a servo state
        _, yaw, roll = _sample(seg, rate=200.0)
        assert np.abs(yaw).max() <= fx.SOFT_DEG and np.abs(roll).max() <= fx.SOFT_DEG


# --- tune -------------------------------------------------------------------

def test_track_score_sees_the_shape_through_the_lag():
    """A servo that follows the command 40 ms late with 0.1 deg of noise has a
    SHAPE error of ~0.1 however large the raw error the lag makes."""
    rng = np.random.default_rng(3)
    t = np.arange(0, 20, 0.005)
    cmd = 5 * np.sin(2 * np.pi * 1.3 * t) + 2 * np.sin(2 * np.pi * 3.1 * t)
    pos = np.interp(t - 0.040, t, cmd) + rng.normal(0, 0.1, len(t))
    s = fx.track_score(t, pos, cmd)
    assert s["lag_ms"] == pytest.approx(40, abs=5)
    assert s["shape_rms_deg"] == pytest.approx(0.1, abs=0.03)
    assert s["raw_rms_deg"] > 5 * s["shape_rms_deg"]
    soft = fx.track_score(t, 0.6 * cmd, cmd)          # a servo that cannot keep up
    assert soft["rate_ratio"] == pytest.approx(0.6, abs=0.02)


def test_a_tune_plan_sets_each_gain_on_entry(tmp_path):
    rng = np.random.default_rng(4)
    n = 3000
    rep = tmp_path / "rep.npz"
    np.savez(rep, roll_deg=rng.normal(0, 2, n).astype(np.float32),
             yaw_deg=np.cumsum(rng.normal(0, 0.3, n)).astype(np.float32),
             start=np.array([0, n]), seed=np.array([0]), fell=np.array([0]),
             rate=np.array(100.0))

    class A:
        replay_file, flights, yaw_limit, tune_seconds = rep, None, 35.0, 5.0
        tune_kp, tune_ki, tune_kd = [400, 1000], [0], [0, 300]

    class Bus:
        def __init__(self):
            self.w = {}

        def write_raw(self, dxl, name, v):
            self.w[dxl, name] = v

    plan = fx.plan_tune(A)
    assert not plan.ahrs
    tunes = [s for s in plan.segments if s.info.get("kind") == "tune"]
    assert len(tunes) == 4
    for seg in tunes:
        bus = Bus()
        seg.enter(bus)
        for dxl in fx.IDS:
            assert bus.w[dxl, "Position P Gain"] == seg.info["kp"]
            assert bus.w[dxl, "Position D Gain"] == seg.info["kd"]
        assert seg.seconds == pytest.approx(5.0, abs=0.02)


# --- chirp gain and phase ----------------------------------------------------

def _bands(gain, delay):
    servo, ahrs, segs, *_ = _rig(chirp={"gain": gain, "delay": delay})
    res = fx.analyse(servo, ahrs, segs)
    return next(r for r in res["segments"] if r["label"] == "chirp")["bands"]


def test_a_sensor_that_under_reads_the_lean_shows_gain_below_one():
    """The pendulum case: gain 0.85 in every octave, no phase."""
    for b in _bands(0.85, 0.0):
        assert b["gain"] == pytest.approx(0.85, abs=0.02), b
        assert abs(b["phase_deg"]) < 2.0, b


def test_a_late_sensor_shows_phase_lag_growing_with_frequency():
    """20 ms late is -7.2 deg at 1 Hz; the fit over an octave lands near the
    octave's geometric centre."""
    bands = _bands(1.0, 0.020)
    for b in bands:
        want = -360 * np.sqrt(b["f_lo"] * b["f_hi"]) * 0.020
        assert b["phase_deg"] == pytest.approx(want, rel=0.25, abs=1.0), b
        assert b["gain"] == pytest.approx(1.0, abs=0.03), b
    assert bands[-1]["phase_deg"] < bands[0]["phase_deg"] - 20


# -- the encoder-free reference ----------------------------------------------
# It is the headline number because the encoders are behind a horn and a
# printed bracket (0.13-0.54 deg lost in the first mounted step holds, ~2 deg
# under a hand wiggle). These pin its SIGN and scale: the rig's gyro and
# accelerometer are the truth's, and during the chirp the fused output alone
# is late by a known delay.

@pytest.fixture(scope="module")
def delayed():
    servo, ahrs, segs, u_y, u_r = _rig(chirp={"gain": 1.0, "delay": 0.020})
    return fx.analyse(servo, ahrs, segs)["selfref"]


def test_the_encoder_free_reference_finds_a_late_output_as_a_positive_lag(delayed):
    row = next(r for r in delayed["segments"] if r["label"] == "chirp")
    assert row["lag_ms"] == pytest.approx(20.0, abs=2.0)
    assert row["after_lag_rms_deg"] < 0.25 * row["roll_rms_deg"]
    # The rig's gyro has no bias, but its noise (0.25 deg/s, twice the real
    # one's) leaves ~0.005 deg/s in the bias pooled from 11 s of holds: ~0.5
    # deg over the 40 s chirp, which the pinning then spreads out.
    assert row["end_mismatch_deg"] < 1.0


def test_the_encoder_free_reference_reads_zero_when_nothing_is_late():
    servo, ahrs, segs, *_ = _rig(chirp={"gain": 1.0, "delay": 0.0})
    row = next(r for r in fx.analyse(servo, ahrs, segs)["selfref"]["segments"]
               if r["label"] == "chirp")
    assert row["roll_rms_deg"] < 0.15          # the reference's floor on this rig
    assert abs(row["lag_ms"]) < 1.0


def test_a_repeat_of_the_same_input_cancels_what_repeats(delayed):
    """Same delay, different noise: the 20 ms lag is deterministic and must
    vanish from the run-to-run difference, though it dominates each run."""
    servo, ahrs, segs, *_ = _rig(seed=1, chirp={"gain": 1.0, "delay": 0.020})
    other = fx.analyse(servo, ahrs, segs)["selfref"]
    rows = {r["label"]: r for r in fx.repeat_difference(delayed["series"], other["series"])}
    lag_rms = next(r for r in delayed["segments"] if r["label"] == "chirp")["roll_rms_deg"]
    assert rows["chirp"]["fused_rms_deg"] < 0.1 * lag_rms
    assert abs(rows["chirp"]["shift_ms"]) <= 1.0


# -- the TM151 as a complementary filter -------------------------------------

def test_the_complementary_filter_turns_and_pulls_the_right_way():
    """Sign conventions, one each: the gyro alone must follow the rig's truth
    through the whole run, and the accelerometer alone must pull a wrong start
    back onto gravity in a still hold. (With both, the rig's 150 mm lever arm
    is SUPPOSED to show in the chirp -- that is the effect being modelled.)"""
    servo, ahrs, segs, *_ = _rig(chirp={"gain": 1.0, "delay": 0.0})
    t, keep, _ = fx.ahrs_clock(ahrs["t_host"], ahrs["t_us"])
    t, g, acc = t[keep], ahrs["gyro"][keep], ahrs["acc_g"][keep]
    down = fx.gravity_sensor(fx.quats_to_mats(ahrs["quat"][keep]))
    ang = lambda v, k: np.degrees(np.arccos(np.clip((v[k] * down[k]).sum(1), -1, 1)))  # noqa: E731

    v = fx.comp_filter(t, g, acc, down[0], 1e9)
    assert np.percentile(ang(v, slice(None)), 99) < 0.3

    ref = fx._in(t, (segs[0]["t0"], segs[0]["t1"]))
    off = fx.axis_angle([1.0, 0, 0], [np.radians(5.0)])[0] @ down[ref][0]
    v = fx.comp_filter(t[ref], g[ref], acc[ref], off, 0.19)
    e = np.degrees(np.arccos(np.clip((v * down[ref]).sum(1), -1, 1)))
    assert e[0] > 4.9 and e[-100:].max() < 0.1


def test_the_rest_model_recovers_the_filter_that_made_the_noise():
    """Still sensor, white accelerometer and gyro noise, 'fused' output made
    by a 0.19 s complementary filter: the fit must name 0.19 and track it."""
    rng = np.random.default_rng(3)
    t = np.arange(0, 120, 1 / RATE)
    down = np.array([0.02, -0.01, -1.0]); down /= np.linalg.norm(down)
    acc = down + rng.normal(0, 0.002, (len(t), 3))
    gyro = rng.normal(0, np.radians(0.13), (len(t), 3))
    v = fx.comp_filter(t, gyro, acc, down, 0.19)
    # a rotation taking earth -z onto v, as the fused quaternion would carry
    z = -v
    x = np.cross([0, 1.0, 0], z); x /= np.linalg.norm(x, axis=1)[:, None]
    y = np.cross(z, x)
    R = np.stack([x, y, z], 2).transpose(0, 2, 1)      # rows: earth axes in sensor frame
    assert np.allclose(fx.gravity_sensor(R), v)
    rm = fx.rest_noise_model(t, gyro, acc, R, (0.0, 120.0))
    assert rm["tau_s"] == 0.19
    assert min(rm["r"]) > 0.95


def test_the_window_fit_names_the_tau_that_made_each_stretch():
    """Two stretches of a still sensor, the 'fused' output made at 0.19 s
    then at 1.0 s: each window must come back near its own."""
    rng = np.random.default_rng(4)
    t = np.arange(0, 24, 1 / RATE)
    down = np.array([0.0, 0.0, -1.0])
    acc = down + rng.normal(0, 0.002, (len(t), 3))
    gyro = rng.normal(0, np.radians(0.13), (len(t), 3))
    half = len(t) // 2
    v1 = fx.comp_filter(t[:half], gyro[:half], acc[:half], down, 0.19)
    v2 = fx.comp_filter(t[half:], gyro[half:], acc[half:], v1[-1], 1.0)
    rows = fx.tau_windows(t, gyro, acc, np.r_[v1, v2], 0.0)
    first = [r["tau_s"] for r in rows if r["t0"] + fx.TAU_WINDOW_S <= t[half]]
    second = [r["tau_s"] for r in rows if r["t0"] >= t[half]]
    assert 0.15 < np.median(first) < 0.25
    assert 0.7 < np.median(second) < 1.4


@pytest.mark.parametrize("rate", [5.0, 20.0, 40.0])
def test_a_sweep_cruises_at_its_rate_inside_the_travel_and_ends_at_rest(rate):
    """The cruise is the measurement -- rotation with no angular acceleration
    -- so it must exist and be at the rate asked; the rest is the travel."""
    t, p = fx.sweep_profile(35.0, rate, 300.0, 2)
    v = np.gradient(p, t)
    a = np.gradient(v, t)
    assert np.abs(p).max() < 35.1 < fx.SOFT_DEG
    assert abs(p[-1]) < 0.05 and abs(v[-1]) < 0.05 * rate
    cruise = np.abs(np.abs(v) - rate) < 1e-6
    inner = cruise & np.roll(cruise, 2) & np.roll(cruise, -2)   # off the stencil's edges
    assert cruise.mean() > 0.7 and np.abs(a[inner]).max() < 1.0
    assert np.abs(a).max() < 300.0 * 1.05


# -- the lever-arm fit, upside down and on a rattling plate ------------------
# 2026-09-24: with the d mount turned over the fit said 58 mm from the roll
# axis where it had said 32-35. These pin what can and cannot do that.

LEVER_BELOW = [0.010, 0.008, -0.035]


def _lever(**kw):
    servo, ahrs, segs, *_ = _rig(lever=LEVER_BELOW, **kw)
    return fx.analyse(servo, ahrs, segs)["lever"]


def test_the_lever_arm_comes_back_with_the_sensor_upside_down():
    """No gravity-sign trap: the same sensing point, sensor turned over."""
    up, down = _lever(), _lever(flip=True)
    for lv in (up, down):
        assert np.linalg.norm(lv["r_m"] - LEVER_BELOW) < 0.003
        assert lv["from_roll_axis_mm"] == pytest.approx(35.9, abs=1.0)


def _plate_rattle(t, alpha_w):
    """Extra acceleration along a RIG-FIXED earth axis, driven by the plate's
    angular acceleration -- a mount rocking in its play as the roll swings."""
    drive = 0.035 * np.linalg.norm(alpha_w, axis=1) * np.sign(alpha_w[:, 0])
    return drive[:, None] * np.array([0.0, 1.0, 0.0])


def test_random_rattle_costs_residual_not_the_lever_arm():
    rng = np.random.default_rng(9)
    lv = _lever(shake=lambda t, a: rng.normal(0, 0.10, (len(t), 3)))
    assert lv["from_roll_axis_mm"] == pytest.approx(35.9, abs=1.5)
    assert min(lv["resid_mg"]) > 8.0


def test_a_rig_fixed_rattle_biases_the_fit_and_turning_over_reverses_it():
    """The diagnostic: a phantom arm that does not turn over with the sensor
    moves the fitted distance one way upright and the OTHER way flipped, so
    the mean of the two is the sensing point and half the gap is the rattle."""
    up = _lever(shake=_plate_rattle)["from_roll_axis_mm"]
    down = _lever(flip=True, shake=_plate_rattle)["from_roll_axis_mm"]
    assert up - 35.9 > 10 and 35.9 - down > 10
    assert (up + down) / 2 == pytest.approx(35.9, abs=3.0)


@pytest.mark.parametrize("cmd", ["session", "step", "chirp", "sine", "jog", "tune"])
def test_every_command_still_defaults_to_the_roll_axis(cmd):
    """2026-09-24: `sweep`'s set_defaults(axis="yaw") rewrote the --axis
    default every command shares, and a whole session's roll steps and chirps
    ran on yaw. The sweep has its own option now; this keeps it that way."""
    ap = fx.build_parser()
    assert ap.parse_args([cmd]).axis == "roll"
    a = ap.parse_args(["sweep"])
    assert a.axis == "roll" and a.sweep_axis == "yaw"
    assert all(s.label.startswith("sweep_yaw") for s in fx.plan_sweep(a).segments
               if s.label.startswith("sweep"))
