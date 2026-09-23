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


def _rig(seed=0, chirp=None):
    """`chirp`: None, or {"gain", "delay"} -- a 40 s roll chirp replaces the
    sway, the injected error is off, and during it the AHRS reports the roll
    joint scaled by `gain` and `delay` s later than everything else."""
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
    p_w = np.einsum("nij,j->ni", R_t, R_LEVER)
    a_w = np.gradient(np.gradient(p_w, tau, axis=0), tau, axis=0)
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
