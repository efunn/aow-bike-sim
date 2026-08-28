"""The TM151's error model, on the simulated AHRS.

`sim_odometry.py` closed the loop on the VELOCITY path: the controller sees
what `hw/odometry.py` reconstructs instead of MuJoCo truth. This closes it on
the ORIENTATION path, which is the other half and the more dangerous one.

WHY IT MATTERS MORE. The velocity entries feed slow outer loops -- that is why
25 ms of encoder lag costs three episodes and 8 ms costs nothing. `roll`,
`roll_rate` and `yaw_rate` are the FAST loop: they are observation entries 0, 1
and 2, nothing averages them, and the bike falls in 113 ms. And the scale is
unfavourable. `general_rl_odo` holds max roll between 0.2 and 3.3 degrees on
the eval grid, while the TM151's dynamic roll/pitch accuracy is <1.5 deg RMS.
The error is the same size as the signal.

WHAT ACTUALLY SEPARATES THE TWO PARTS, since it is easy to get wrong. The
datasheet's STATIC block is identical in roll and pitch (<0.5 deg both), and
the visible differences -- internal update rate 400 vs 800 Hz, gyro
non-linearity 0.3 vs 0.2 % FS, accel misalignment 0.5 vs 0.3 deg, yaw 1.0 vs
0.8 deg static and 3.0 vs 2.6 deg per 25 min -- are all either irrelevant here
or measured to do nothing. The one that matters is in the DYNAMIC block:
roll/pitch <1.5 deg (TM151) against <1.0 deg (TM171). Ablated on the eval grid,
swapping only that row recovers 0.537 -> 0.635 of the 0.689 the full TM171
scores, while yaw drift alone gives 0.542 and misalignment alone 0.519 --
i.e. nothing, against a seed noise floor of about +-0.02.

Update rate is irrelevant because the Pi senses at 100 Hz and both parts run
at 400 Hz or better internally, with a user-configurable ODR that goes down to
100.

WHERE THE NUMBERS COME FROM. `docs/ahrs/TransducerM_TM151_TM171_Datasheet_EN_
V116-R.pdf`, section 2 (IMU Sensor and AHRS Specification) and the summary
table on page 2. TM151 column throughout -- the TM171 is the better part and
several rows differ. They live here as module constants rather than in
`config/bike_params.yaml` ON PURPOSE: `plant_digest` hashes every top-level key
except `control`, so adding them there would invalidate all seven
digest-matching exports for a change that does not move the bike at all.
Measured: `e1ec36bfa670217e -> 5f270674e21c3edc`. `sim_odometry.ENCODER_FILTER`
sets the same precedent.

WHAT IS MODELLED, and at what level. The datasheet specifies the AHRS OUTPUT
accuracy directly, so the orientation error is modelled as an error process on
the output rather than by simulating the internal fusion filter. That is the
same choice `sim_odometry` makes in reusing `RateFilter` instead of modelling
servo firmware: reproduce the specified behaviour, not the mechanism. It also
avoids double-counting -- the controller reads orientation and rate through
SEPARATE datasheet rows, and they are applied separately here.

NOT WHITE NOISE. A fusion filter's orientation error is correlated in time:
gravity bounds it, so it wanders rather than walking away, and a fresh draw
every tick would be both wrong and far too easy to balance against. Each axis
is a first-order Gauss-Markov process -- stationary, with the datasheet RMS and
a correlation time TAU.

MOUNTING POSITION: WHAT THIS DOES AND DOES NOT CAPTURE. Measured 2026-08-27
with five probe sites on ONE chassis and one trajectory, so the lever arm is
isolated from the chaotic divergence that merely moving the 12 g sensor causes:

    position                max |gyro - origin|   RMS |accel - origin|
    origin      [0,0,0]           0.00e+00 d/s          0.0000 m/s^2
    as-built    [.05,0,.13]       0.00e+00              9.8679
    high mast   [.05,0,.30]       0.00e+00             21.4693
    far forward [.20,0,.13]       0.00e+00             16.9199
    off-axis    [.05,.10,.13]     0.00e+00             10.0061

  * THE GYRO DOES NOT CARE, exactly. Angular velocity is a property of the
    rigid body, not of where you measure it, and the model reproduces that to
    machine precision. `roll_rate` and `yaw_rate` -- observation entries 1 and
    2 -- are therefore mount-independent, and that is a real result.

  * THE ACCELEROMETER CARES A LOT, and the arm is measured FROM THE CoM (the
    chassis origin is not a mounting position -- it is the rear axle centre,
    inside the wheel). Whole-bike CoM is chassis [0.083, 0, 0.073]; excess
    specific force against a probe there, over mountable positions only:

        at the CoM        [.083,0,.073]     0 mm     0.000 m/s^2
        as built          [.05,0,.13]      66 mm     4.584
        over rear wheel   [0,0,.066]       83 mm     5.867
        front over steer  [.18,0,.10]     100 mm     7.090
        high mast         [.05,0,.30]     229 mm    16.075

    LINEAR IN THE ARM at ~0.070 m/s^2 per mm, which is what alpha x r and
    omega x (omega x r) both being first order in r predicts. So the as-built
    position is already the best realisable one -- nearer the CoM than over the
    rear wheel -- and a mast is disqualified at more than a gravity of
    spurious signal. This is not
    new -- hw/odometry.py already records a tau=0.3 s accelerometer blend
    degrading v_lon from 8.8 to 174 mm/s RMS for exactly this reason, which is
    why the accelerometer there is a fallback and not a co-equal sensor. The
    MuJoCo sensor sits on `ahrs_site` at the configured position, so this
    module inherits the effect correctly WITHOUT modelling anything.

  * THE ORIENTATION OUTPUT IS MOUNT-INDEPENDENT HERE, AND THAT IS A
    LIMITATION, NOT A FINDING. `ORIENT_RMS_DEG` is applied as a fixed
    datasheet figure whatever `bike.ahrs.pos` says. A real unit derives
    attitude by fusing the gyro against the ACCELEROMETER AS A GRAVITY
    REFERENCE, so lever-arm acceleration corrupts that reference and a
    badly-placed unit should read worse than its datasheet number. The
    datasheet says so itself: footnote [3] warns that parts without vibration
    resistance are "susceptible to low frequency linear acceleration", and the
    dynamic figure is quoted for "typical low-dynamic movements... indoor
    robotic vehicles, low-speed driving". A balancing bike with the IMU on a
    mast is not obviously inside that envelope.

    So DO NOT read a flat eval result across mounting positions as evidence
    that position is free. This module cannot see that effect by construction.
    Deciding it needs either a fusion model or a bench measurement on the real
    part -- see docs/plans/odometry-rewrite.md.

  * THE MAGNETOMETER IS NOT MODELLED AT ALL. There is no magnetometer in the
    MuJoCo model; yaw comes from the true quaternion plus YAW_DRIFT_DEG_PER_S.
    So nothing here can say anything about siting the unit away from motor
    coils or current-carrying wire, and the usual hardware practice applies
    unchanged and unchecked.

TAU WAS A GUESS AND IS NOW MEASURED -- 0.19 +- 0.01 s, from 300 s of a real
TM151 sitting still (`analysis/tm151_check.py`, exponential fit r2 0.999). The
guess was 2.0 s, so the SHAPE was right and the timescale was 10x wrong.

`TAU_ORIENT_S` IS DELIBERATELY STILL 2.0. Changing it would silently reprice
`general_rl_odo_ahrs`, which trained against 2.0 and is partly specialised to
it: re-evaluated at 0.19 that policy goes 0.672 / survival 1.00 -> 0.570 / 0.95.
So the constant is a TRAINING CONTRACT, not a best estimate, and moving it is a
retraining decision rather than an edit. Pass `tau_orient_s` to evaluate at the
measured value; see docs/status.md for the comparison.

And 0.19 s is a RESTING figure. The dynamic correlation time is unmeasured, as
is the dynamic RMS.

RANDOMISING THE TWO OF THEM IS WHAT `set_error_params` EXISTS FOR. Note what
was and was not already random: every episode already drew a fresh error
REALISATION -- noise trajectory, misalignment, orientation walk -- from ONE
FIXED distribution, which is precisely how `general_rl_odo_ahrs` specialised to
tau 2.0. `general_env` now draws the PARAMETERS of that distribution per
episode too, log-uniform, when `randomization.ahrs_orient_rms_deg_range` and
`ahrs_tau_s_range` are set. It lives there rather than in
`control/randomize.py` because `DomainRandomizer` perturbs the MuJoCo model and
the sensor is not in the model.

The range is deliberately NOT centred on the measurements. 0.0142 deg and
0.19 s are RESTING figures and the bike does not rest; the datasheet's dynamic
bound is 1.5 deg. That is a ~100x span with the true moving value unmeasured
somewhere inside it, so the range spans the whole thing log-uniformly and the
measurement contributes a defensible FLOOR, which it did not have before.
"""

from __future__ import annotations

from contextlib import contextmanager

import numpy as np

from .hw.dynamixel import CONTROL_HZ_DEFAULT

# --- datasheet, TM151 column -------------------------------------------------

# Peak-to-peak is quoted "sampled at 100Hz" with NO OBSERVATION WINDOW, and
# that omission matters: the peak-to-peak of a Gaussian grows without bound as
# you watch longer, so "<= 0.5 deg/s p-p" is not a well-posed constraint on
# sigma until a window is fixed. Measured, at sigma = 0.5/6:
#
#     0.5 s (50 samples)   p-p 0.376   4.52 sigma
#     1.0 s (100)          p-p 0.418   5.02
#     2.0 s (200)          p-p 0.459   5.50
#    10.0 s (1000)         p-p 0.539   6.47   <- over spec
#    60.0 s (6000)         p-p 0.621   7.45
#
# So the 1/6 divisor honours the spec for an observation window of a few
# seconds and violates it beyond ~10 s. That is the intended reading of a
# datasheet noise figure, and it is recorded here because a later measurement
# against a 60 s capture would otherwise look like a bug in this module.
PP_TO_SIGMA = 1.0 / 6.0

GYRO_NOISE_PP_DPS = 0.5          # "Noise  <= +-0.5 deg/s  Peak-to-Peak"
GYRO_BIAS_STABILITY_DPH = 5.5    # "Bias Stability 5.5 deg/h" (Allan)
GYRO_G_SENS_DPS_PER_G = 0.1      # "Acceleration Sensitivity < 0.1 deg/s/g"
                                 #   shared by both parts, and APPLIED below
GYRO_BANDWIDTH_HZ = 68.0         # "-3db"
# Non-linearity is RECORDED AND NOT MODELLED, and the reason is the unit:
# "<0.3 % FS" is a fraction of the +-1000 deg/s FULL SCALE, i.e. up to 3 deg/s
# at the extremes of the range. The bike does not go there. Measured over a
# 0.6 m/s drive under general_rl_odo, body rates peak at 22.6 / 29.8 / 34.7
# deg/s -- under 3.5% of full scale -- and a non-linearity is a curve over the
# range, not a constant offset, so the deviation at 3% of FS is a small
# fraction of the 3 deg/s worst case. This is also the TM151/TM171 row that
# LOOKS like it should matter (0.3 vs 0.2 % FS) and does not.
GYRO_NONLINEARITY_PCT_FS = 0.3   # TM151; TM171 is 0.2. Not applied.

ACCEL_NOISE_PP_MG = 12.0         # "Noise <= 12 mg  Peak-to-Peak"
ACCEL_MISALIGN_DEG = 0.5         # "Misalignment < 0.5 deg (TM151)"

# AHRS orientation output, (roll, pitch, yaw) RMS degrees.
#   static  -- "Static accuracy  <0.5 / <0.5 / <1.0 (TM151)"
#   typical -- "Dynamic accuracy (Inertial) <1.5 / <1.5" for roll and pitch.
#              Yaw is quoted as a DRIFT ("3.0 deg error every 25 minutes"), not
#              an RMS, so the static figure is carried and the drift is applied
#              separately -- see YAW_DRIFT_DEG_PER_S.
#   tm171   -- the SAME datasheet's other part, dynamic 1.0 / 1.0 / 0.8. Not a
#              model of a device we own; it is here to price the upgrade,
#              because the damage below is dominated by exactly this row.
ORIENT_RMS_DEG = {
    "tm151_static": (0.5, 0.5, 1.0),
    "tm151": (1.5, 1.5, 1.0),
    "tm171": (1.0, 1.0, 0.8),
}
# Pure-inertial yaw drift: "3.0 deg error every 25 minutes" (TM151), 2.6 for
# the TM171. Gyro and accelerometer rows are otherwise SHARED between the two
# parts, so only orientation, yaw drift and misalignment change with `level`.
YAW_DRIFT_DEG_PER_S = {
    "tm151_static": 3.0 / (25.0 * 60.0),
    "tm151": 3.0 / (25.0 * 60.0),
    "tm171": 2.6 / (25.0 * 60.0),
}
MISALIGN_DEG = {"tm151_static": 0.5, "tm151": 0.5, "tm171": 0.3}

# The TM171's STATIC roll/pitch, recorded as a datasheet fact even though it is
# not a usable level. It is 0.5 deg -- IDENTICAL to the TM151 -- which is the
# reason there is no "tm171_static": it would be the same run twice on the
# axes that matter here. It is also the reason the two parts look
# interchangeable until you read the dynamic block. Kept so that identity is
# checkable instead of being an assertion in prose.
TM171_STATIC_RMS_DEG = (0.5, 0.5, 0.8)

GRAVITY = 9.81

# Correlation time of the orientation error, seconds. NOT A DATASHEET NUMBER.
# Chosen as a plausible middle for a gravity-corrected fusion filter and swept
# rather than trusted. Too short and the error is effectively white, which the
# controller averages away; too long and it is a constant offset, which it
# trims out. The damage is in between, which is exactly why this is a GUESS
# that has to be reported alongside any result that depends on it.
TAU_ORIENT_S = 2.0

# Correlation time of the gyro bias. Also a GUESS. Bias instability is a
# flicker process; a long-tau Gauss-Markov is the usual tractable stand-in.
TAU_BIAS_S = 100.0

LEVELS = ("none", "tm151_static", "tm151", "tm171")

# Which CHANNELS carry their error, so the damage can be attributed. Same idea
# as sim_odometry's lon_only / lat_only, which is what showed that v_lat was
# the whole problem there. The distinction is actionable: an ORIENTATION
# problem is fixed by a better part (the TM171 is 1.0 deg dynamic against the
# TM151's 1.5), a GYRO problem is not, and a MOUNTING problem is fixed for free
# by calibration.
CHANNELS = ("both", "orient", "gyro")


def _gm_step(x, dt: float, tau: float, sigma, rng) -> np.ndarray:
    """One step of a stationary first-order Gauss-Markov process.

    Returns a process with standard deviation `sigma` and autocorrelation
    exp(-dt/tau) -- so it wanders on a `tau` timescale and stays bounded,
    unlike a random walk. At tau -> 0 it degenerates to white noise at sigma.
    """
    a = np.exp(-dt / tau)
    return a * x + np.sqrt(max(0.0, 1.0 - a * a)) * np.asarray(sigma) * rng.standard_normal(np.shape(x))


def rpy_from_quat(q) -> tuple[float, float, float]:
    """(roll, pitch, yaw) from a wxyz quaternion.

    Same convention as `hw/run_bike._rpy` and `sim_odometry._rpy`, which is
    what makes a corrupted quaternion here read identically to a real AHRS
    packet on the Pi.
    """
    w, x, y, z = q
    return (float(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            float(np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))),
            float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def _small_angle_quat(rpy) -> np.ndarray:
    """Rotation quaternion for a small (roll, pitch, yaw) error, wxyz."""
    h = np.asarray(rpy, float) / 2.0
    # Small-angle: cos ~ 1, sin ~ h. Normalised on the way out anyway.
    q = np.array([1.0, h[0], h[1], h[2]])
    return q / np.linalg.norm(q)


class SimAhrs:
    """TM151 error model over the model's own AHRS sensors.

    One sample per tick, cached and shared: the bike has ONE physical AHRS, so
    the estimator and the controller must see the same corrupted numbers. That
    is why `sample()` is separate from the readers -- calling it twice a tick
    would give them different sensors.

    `level`:
      "none"     pass the clean sensors through. The floor, and the default,
                 so every policy trained before this module reproduces.
    Every level names a PART and a CONDITION, because those are two separate
    axes and mixing them produced a table nobody could read. An earlier version
    called these "static" and "typical", which left "is typical the TM151?" a
    fair question with no answer in the name.

      "tm151_static"  the part we have, at its STATIC accuracy (0.5 deg
                      roll/pitch). What a bench calibration would report.
      "tm151"         the part we have, at its DYNAMIC accuracy (1.5 deg).
                      What a moving bike gets: the one to design against.
      "tm171"         the better part in the same datasheet, dynamic (1.0 deg).
                      Not a device we own -- it prices the upgrade.

    There is no "tm171_static": both parts are <0.5 deg static in roll and
    pitch, so it would be identical to "tm151_static" on the axes that matter
    here. THAT IDENTITY IS WHY THE PARTS LOOK INTERCHANGEABLE at a glance --
    the datasheet's static block hides the difference, and only the DYNAMIC
    block separates them (1.5 vs 1.0 deg roll/pitch).
    """

    def __init__(self, model, params: dict, level: str = "none",
                 seed: int = 0, tau_orient_s: float = TAU_ORIENT_S,
                 tau_bias_s: float = TAU_BIAS_S, channels: str = "both",
                 hz: float = CONTROL_HZ_DEFAULT,
                 orient_rms_deg=None):
        if level not in LEVELS:
            raise ValueError(f"level must be one of {LEVELS}, got {level!r}")
        if channels not in CHANNELS:
            raise ValueError(f"channels must be one of {CHANNELS}, "
                             f"got {channels!r}")
        self.level = level
        self.channels = channels
        self.tau_orient_s = float(tau_orient_s)
        self.tau_bias_s = float(tau_bias_s)
        # The orientation RMS triple actually in force. Defaults to the
        # datasheet row for `level`, and is an OVERRIDE rather than a new level
        # because the caller randomising it (general_env) is drawing from a
        # continuum, not choosing a part -- see `set_error_params`.
        self.orient_rms_deg = (ORIENT_RMS_DEG.get(level) if orient_rms_deg
                               is None else tuple(float(x) for x in
                                                  orient_rms_deg))
        self.hz = float(hz)
        self._dt = 1.0 / self.hz
        self.adr = {}
        for name in ("ahrs_gyro", "ahrs_accel", "ahrs_quat"):
            s = model.sensor(name)
            self.adr[name] = (int(s.adr[0]), int(s.dim[0]))
        self.reset(seed)

    def set_error_params(self, *, orient_rms_roll_deg=None,
                         tau_orient_s=None) -> None:
        """Re-point the two UNMEASURED error parameters, between episodes.

        `orient_rms_roll_deg` names the ROLL/PITCH RMS and scales the whole
        triple by the ratio to the level's own roll figure, so the part's
        yaw:roll character is preserved rather than a third number being
        invented. Both arguments are the parameters of the error DISTRIBUTION;
        the realisation is redrawn by `reset` either way.

        Why these two and nothing else: everything else in this module is a
        datasheet row with a stated bound, while the dynamic orientation RMS
        is bounded only above (<1.5 deg) and TAU_ORIENT_S is not in the
        datasheet at all. See the module header.
        """
        if tau_orient_s is not None:
            self.tau_orient_s = float(tau_orient_s)
        if orient_rms_roll_deg is not None and self.level != "none":
            nominal = ORIENT_RMS_DEG[self.level]
            k = float(orient_rms_roll_deg) / nominal[0]
            self.orient_rms_deg = tuple(k * x for x in nominal)

    def reset(self, seed: int = 0) -> None:
        self.rng = np.random.default_rng(seed)
        self._orient_err = np.zeros(3)     # rad, (roll, pitch, yaw)
        self._gyro_bias = np.zeros(3)      # rad/s
        self._yaw_drift = 0.0              # rad, accumulates without bound
        self._cache = None
        self._acc = 0.0
        # A misalignment is a FIXED build error, not noise: drawn once per
        # power-on and constant thereafter. Drawing it per tick would make it
        # a noise source the real part does not have.
        m = np.deg2rad(MISALIGN_DEG.get(self.level, ACCEL_MISALIGN_DEG))
        self._accel_tilt = self.rng.uniform(-m, m, size=3) if self.level != "none" \
            else np.zeros(3)

    def _raw(self, data, name):
        adr, dim = self.adr[name]
        return np.array(data.sensordata[adr:adr + dim], dtype=float)

    def sample(self, data, dt: float) -> dict:
        """Advance the error state by `dt` and cache one corrupted reading."""
        quat = self._raw(data, "ahrs_quat")
        gyro = self._raw(data, "ahrs_gyro")
        accel = self._raw(data, "ahrs_accel")
        if self.level == "none":
            self._cache = {"quat": quat, "gyro": gyro, "accel": accel}
            return self._cache

        rms = np.deg2rad(self.orient_rms_deg)
        self._orient_err = _gm_step(self._orient_err, dt, self.tau_orient_s,
                                    rms, self.rng)
        # Yaw additionally WALKS: the datasheet quotes it as an error per unit
        # time rather than an RMS, because nothing bounds heading the way
        # gravity bounds roll and pitch. Sign is a coin flip per power-on.
        self._yaw_drift += (np.deg2rad(YAW_DRIFT_DEG_PER_S[self.level]) * dt
                            * self.rng.standard_normal())
        err = self._orient_err + np.array([0.0, 0.0, self._yaw_drift])
        q_err = _small_angle_quat(err)
        quat_out = _quat_mul(q_err, quat)
        quat_out /= np.linalg.norm(quat_out)

        # Gyro: white noise + a wandering bias + g-sensitivity. The last one is
        # NOT noise -- it is proportional to the specific force the bike is
        # actually feeling, so it correlates with the manoeuvre.
        sigma_g = np.deg2rad(GYRO_NOISE_PP_DPS * PP_TO_SIGMA)
        bias_sigma = np.deg2rad(GYRO_BIAS_STABILITY_DPH / 3600.0)
        self._gyro_bias = _gm_step(self._gyro_bias, dt, self.tau_bias_s,
                                   bias_sigma, self.rng)
        g_sens = np.deg2rad(GYRO_G_SENS_DPS_PER_G) * (accel / GRAVITY)
        gyro_out = (gyro + self._gyro_bias + g_sens
                    + sigma_g * self.rng.standard_normal(3))

        # Accelerometer: white noise plus the fixed mounting misalignment.
        sigma_a = ACCEL_NOISE_PP_MG * 1e-3 * GRAVITY * PP_TO_SIGMA
        tilt = self._accel_tilt
        # Small-angle rotation of the measured vector; cross product is the
        # first-order term and is all a <0.5 deg error justifies.
        accel_out = (accel + np.cross(tilt, accel)
                     + sigma_a * self.rng.standard_normal(3))

        if self.channels == "orient":
            gyro_out = gyro          # clean rates, corrupted attitude
        elif self.channels == "gyro":
            quat_out = quat          # clean attitude, corrupted rates
        self._cache = {"quat": quat_out, "gyro": gyro_out, "accel": accel_out}
        return self._cache

    def tick(self, data, dt: float) -> dict:
        """Advance by `dt` of ELAPSED time, sampling on the AHRS's own clock.

        `sample()` takes a tick period; this takes however much time the caller
        has burned, and holds the last reading in between -- so teleop looping
        at the 2500 Hz physics step and an env looping at 50 Hz both get a
        sensor sampled at `hz`, which is what the Pi reads. Without this the
        error process runs at the CALLER's rate, and its correlation time
        (measured in seconds) would mean something different in each one.

        Used when nothing else owns the clock. `SimOdometry` calls `sample()`
        directly because it already ticks at the sense rate.
        """
        self._acc += float(dt)
        n = int(self._acc / self._dt)
        if n:
            self._acc -= n * self._dt
            for _ in range(min(n, 4)):
                self.sample(data, self._dt)
        elif self._cache is None:
            self.sample(data, self._dt)
        return self._cache

    def latest(self, name: str) -> np.ndarray:
        """The cached corrupted reading. `sample()` must have run this tick."""
        if self._cache is None:
            raise RuntimeError("SimAhrs.sample() has not run yet")
        return self._cache[name.replace("ahrs_", "")]

    @contextmanager
    def estimated(self, data):
        """Controller sees the corrupted orientation; physics keeps the truth.

        Swaps `qpos[3:7]` and `qvel[3:6]`, which is exactly what
        `hw/state.HardwareData.set_orientation` writes from the real AHRS. Wrap
        ONLY the controller call -- `control/drive.py` makes no MuJoCo calls, so
        it cannot notice, but anything that re-derives from the model would.
        """
        if self.level == "none" or self._cache is None:
            yield
            return
        q0 = data.qpos[3:7].copy()
        w0 = data.qvel[3:6].copy()
        data.qpos[3:7] = self._cache["quat"]
        data.qvel[3:6] = self._cache["gyro"]
        try:
            yield
        finally:
            data.qpos[3:7] = q0
            data.qvel[3:6] = w0
