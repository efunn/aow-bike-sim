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
i.e. nothing, against a seed noise floor of about +-0.02. (The yaw figure was
measured with the pre-2026-09-22 random-walk drift, ~80x too small at 60 s --
see YAW_DRIFT_DEG_PER_S. It does not vouch for the corrected drift.)

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

`TAU_ORIENT_S` IS NOW THE MEASUREMENT, 0.19. It was held at 2.0 for a while on
the grounds that it was a TRAINING CONTRACT rather than a best estimate --
moving it silently reprices `general_rl_odo_ahrs`, which trained against 2.0
and goes 0.672 / survival 1.00 -> 0.570 / 0.95 when re-evaluated at 0.19.

That repricing is now measured rather than feared, and it is small: across
five policies a 10x move in tau costs at most 0.102 of score, and the policy
that never trained against an AHRS at all flips MORE eval episodes across tau
(four) than the one accused of specialising to it (one). The correlation time
is not a live risk -- see `analysis/ahrs_tau.py` and docs/status.md, "The tau
question, answered". So the constant is now simply the number the part has.

NOTHING WAS RETRAINED FOR THIS. The four AHRS policies stand as exported and
are re-scored at the new default. `config/rl_general_odo_ahrs.yaml` pins
`ahrs_tau_s: 2.0` explicitly so the policy that trained at the guess stays
reproducible from its own config; the randomised configs draw from a range and
never touched this default.

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
    # Only the residual WANDER (FILTER_WANDER_DEG): the filter makes the rest.
    "tm151_filter": (0.1, 0.1, 1.0),
}
# Pure-inertial yaw drift: "3.0 deg error every 25 minutes" (TM151), 2.6 for
# the TM171. Gyro and accelerometer rows are otherwise SHARED between the two
# parts, so only orientation, yaw drift and misalignment change with `level`.
#
# A CONSTANT RATE ABOUT THE WORLD VERTICAL, sign drawn once per power-on. With
# no magnetometer the heading is integrated gyro, and a steady rate error --
# Earth rotation is the candidate (15.04 deg/h * sin(latitude) about the local
# vertical) -- walks it linearly about the world axis however the bike is
# tilted. Until 2026-09-22 this drew a fresh normal every sample, which made it
# a random walk: ~0.0015 deg after 60 s instead of 0.12, and rate-dependent.
# The sign stays a coin flip: if Earth rate is the whole cause it is fixed by
# hemisphere, but the datasheet figure is a bound and the cause is unverified.
# The gyro does NOT carry the matching rate; its bias is a separate row.
YAW_DRIFT_DEG_PER_S = {
    "tm151_static": 3.0 / (25.0 * 60.0),
    "tm151": 3.0 / (25.0 * 60.0),
    "tm171": 2.6 / (25.0 * 60.0),
    "tm151_filter": 3.0 / (25.0 * 60.0),
}
MISALIGN_DEG = {"tm151_static": 0.5, "tm151": 0.5, "tm171": 0.3, "tm151_filter": 0.5}

# The TM171's STATIC roll/pitch, recorded as a datasheet fact even though it is
# not a usable level. It is 0.5 deg -- IDENTICAL to the TM151 -- which is the
# reason there is no "tm171_static": it would be the same run twice on the
# axes that matter here. It is also the reason the two parts look
# interchangeable until you read the dynamic block. Kept so that identity is
# checkable instead of being an assertion in prose.
TM171_STATIC_RMS_DEG = (0.5, 0.5, 0.8)

GRAVITY = 9.81

# Correlation time of the orientation error, seconds. MEASURED: 0.19 +- 0.01
# from 300 s of a real TM151 at rest, exponential fit r2 0.999
# (`analysis/tm151_check.py`). Still not a datasheet number, and still a
# RESTING one -- the dynamic value is unmeasured.
#
# The shape of the cost was predicted before it was measured and held up: too
# short and the error is effectively white, which the controller averages
# away; too long and it is a constant offset, which it trims out; the damage
# is a shallow bump in between. `analysis/ahrs_tau.py` measures the whole
# curve. It was 2.0 (a guess) until 2026-08-28.
TAU_ORIENT_S = 0.19

# Correlation time of the gyro bias. Also a GUESS. Bias instability is a
# flicker process; a long-tau Gauss-Markov is the usual tractable stand-in.
TAU_BIAS_S = 100.0

# -- "tm151_filter": the TM151 as its own fusion filter ----------------------
#
# MEASURED on the yaw-roll fixture, 2026-09-23/24 (`analysis/ahrs_fixture.py`,
# docs/status.md "AHRS fixture"). The noise levels above add random,
# slowly-varying noise (a Gauss-Markov process) to the true
# attitude with independent noise; the real part is better described as a
# complementary filter on its OWN gyro and accelerometer:
#
#   at rest   the fused tilt is the accelerometer's, low-passed at 0.19 s, plus
#             the integrated gyro high-passed at the same tau: r 0.92-0.93
#   moving    the same filter fits at ~1 s. tau rises CONTINUOUSLY with the
#             motion -- 0.20 s under ~1 deg/s, ~0.5 at 1-15, ~1.0 past ~15 --
#             and constant-rate yaw sweeps (slow acceleration < 1 mg) still
#             raised it: ROTATION (or vibration), not slow acceleration, is
#             what it keys on. So the gate here is on the rotation rate.
#   error     most of the dynamic error on the fixture was the sensor's own
#             acceleration read as tilt through that filter, which this level
#             gets from the physics at the AHRS site -- including what a turn
#             or a forward acceleration does, which the fixture never saw:
#             a sustained lateral specific force pulls the reading toward it.
#   left over ~0.1 deg RMS wandering over 1-4 s, carried as small slowly-varying noise.
#
# The same sensor turned over on a more rattly (top-heavy) mount fitted tau
# 0.35 s AT REST and ~2 s moving, where this rate gate matches it to 0.195 deg
# against 0.103 on the first mount: the vibration the part feels moves tau as
# well as rotation does, and the sim barely has vibration. So randomise
# FILTER_TAU_MOTION_S over ~0.7-2 s (`set_error_params`) rather than trust
# 1.0. Unmeasured: the gate's smoothing, anything about turns. One unit.

FILTER_TAU_REST_S = 0.19
FILTER_TAU_MOTION_S = 1.0
FILTER_RATE_REST_DPS = 1.0       # tau is FILTER_TAU_REST_S below this ...
FILTER_RATE_MOTION_DPS = 15.0    # ... FILTER_TAU_MOTION_S above, log-linear between
FILTER_RATE_SMOOTH_S = 1.0       # the gate sees a smoothed |gyro| (fitted on 2 s windows)
FILTER_WANDER_DEG = 0.1          # roll/pitch residual after the filter, RMS
FILTER_WANDER_TAU_S = 2.0        # its 1/e time, 1-4 s measured

# Where the ACCELEROMETER sits in the TM151, from the centre of the housing's
# footprint at the housing's mid-height (the fixture CAD's assumed point,
# config/ahrs_fixture_cad.yaml `sensor_height`), in the TM151's own axes
# (datasheet Figure 2: +x toward the pin header, +y left in the top view, +z
# out of the housing's top) [m]. Fixture, 2026-09-23/24:
#   x, y  ~12 mm off the yaw axis in every fit, the SAME way round in the
#         sensor frame with the mount turned 180 deg about x -- so it travels
#         with the part, not the rig; not the fit's signs (synthetic check),
#         not gyro/accelerometer timing (+-5 ms moves it < 1 mm). Mean of the
#         session fits (10.4, 10.0 -> 8.8, 9.5) and yaw-chirp fits (6.2, 10.1
#         -> 8.0, 7.9), each pair averaged across the flip. +-~2 mm. The
#         datasheet drawing's triad sits ~5 mm the OTHER way: illustrative.
#   z     the flip pair, roll-rich sessions: 32.6 mm from the roll axis below,
#         40.8 above. A rattle fixed to the rig adds to one and takes from the
#         other, so the chip is ~36.7 from the axis, 6.7 further than the 30
#         mm mount puts mid-height: -6.7 +-~3 mm, i.e. down at the circuit
#         board -- the only place a chip can be, whose top face is -5.5 mm.
#         Carried as -5.5, the physical bound.
TM151_ACCEL_OFFSET_M = np.array([0.0084, 0.0094, -0.0055])

# How the TM151's axes sit in the AHRS site's (= the chassis's) frame on the
# bike. ASSUMED identity -- x forward, z up -- until the bike's mount is
# built; the real mount goes through `ahrs_q_mount` on hardware. Only the
# accelerometer offset above uses it.
TM151_TO_SITE = np.eye(3)
assert ORIENT_RMS_DEG["tm151_filter"][:2] == (FILTER_WANDER_DEG, FILTER_WANDER_DEG)

LEVELS = ("none", "tm151_static", "tm151", "tm171", "tm151_filter")

# Which CHANNELS carry their error, so the damage can be attributed. Same idea
# as sim_odometry's lon_only / lat_only, which is what showed that v_lat was
# the whole problem there. The distinction is actionable: an ORIENTATION
# problem is fixed by a better part (the TM171 is 1.0 deg dynamic against the
# TM151's 1.5), a GYRO problem is not, and a MOUNTING problem is fixed for free
# by calibration.
CHANNELS = ("both", "orient", "gyro")


def level_tau_orient_s(level: str) -> float:
    """The orientation-error correlation time a level uses when none is given."""
    return FILTER_WANDER_TAU_S if level == "tm151_filter" else TAU_ORIENT_S


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


def _rotate(q, v):
    """v rotated by the wxyz quaternion q."""
    qv = np.array([0.0, *v])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    return _quat_mul(_quat_mul(q, qv), qc)[1:]


def _axis_angle_quat(rotvec) -> np.ndarray:
    th = float(np.linalg.norm(rotvec))
    if th < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.array([np.cos(th / 2), *(np.sin(th / 2) * np.asarray(rotvec) / th)])


def _turn(v, w, dt: float) -> np.ndarray:
    """A world-fixed vector, in a frame turning at w [rad/s] for dt."""
    th = float(np.linalg.norm(w)) * dt
    if th < 1e-12:
        return v
    k = -np.asarray(w) / np.linalg.norm(w)
    return v * np.cos(th) + np.cross(k, v) * np.sin(th) + k * (k @ v) * (1 - np.cos(th))


def filter_tau(rate_dps: float, tau_rest: float = FILTER_TAU_REST_S,
               tau_motion: float = FILTER_TAU_MOTION_S) -> float:
    """The accelerometer-trust time constant at a (smoothed) rotation rate."""
    lo, hi = np.log(FILTER_RATE_REST_DPS), np.log(FILTER_RATE_MOTION_DPS)
    f = np.clip((np.log(max(rate_dps, 1e-6)) - lo) / (hi - lo), 0.0, 1.0)
    return float(np.exp(np.log(tau_rest) + f * (np.log(tau_motion) - np.log(tau_rest))))


# Skip the accelerometer when |acc| is further than this fraction from 1 g.
# UNMEASURED -- the fixture never saw more than ~50 mg -- and None (no gate)
# until it is. It exists because the SIM's accelerometer on a standing bike is
# violent (60% of samples > 0.2 g off, stiff contacts sampled at a point), and
# ungated that reads as ~8 deg of pitch; see docs/status.md.
FILTER_ACC_GATE = None


def tilt_filter_step(u, gyro, acc, dt: float, tau: float, gate=None) -> np.ndarray:
    """One step of the complementary filter on the gravity direction `u` (unit,
    body frame, the same sign convention as `acc`): turn with the gyro [rad/s],
    pull toward the normalised accelerometer at 1/tau -- unless |acc| is more
    than `gate` (a fraction) from 1 g."""
    p = _turn(np.asarray(u, float), gyro, dt)
    na = float(np.linalg.norm(acc))
    if gate is None or abs(na / GRAVITY - 1.0) <= gate:
        p = p + (dt / tau) * (np.asarray(acc) / na - p)
    return p / np.linalg.norm(p)


def run_tilt_filter(t, gyro, acc, u0, tau_rest: float = FILTER_TAU_REST_S,
                    tau_motion: float = FILTER_TAU_MOTION_S,
                    smooth_s: float = FILTER_RATE_SMOOTH_S) -> np.ndarray:
    """The adaptive filter over recorded arrays: what `SimAhrs` runs one step at
    a time, and what `ahrs_fixture filter-model` checks against a real TM151."""
    u = np.empty((len(t), 3))
    u[0] = np.asarray(u0, float) / np.linalg.norm(u0)
    rate = 0.0
    for i in range(1, len(t)):
        dt = float(t[i] - t[i - 1])
        rate += (dt / (smooth_s + dt)) * (np.degrees(np.linalg.norm(gyro[i])) - rate)
        u[i] = tilt_filter_step(u[i - 1], 0.5 * (gyro[i] + gyro[i - 1]), acc[i], dt,
                                filter_tau(rate, tau_rest, tau_motion))
    return u


def accel_at_offset(accel, w, w_prev, dt: float, delta) -> np.ndarray:
    """Specific force at a point `delta` from where `accel` was measured, on the
    same rigid body: + alpha x d + w x (w x d), alpha from successive rates."""
    alpha = (np.asarray(w) - np.asarray(w_prev)) / dt if w_prev is not None else np.zeros(3)
    return np.asarray(accel) + np.cross(alpha, delta) + np.cross(w, np.cross(w, delta))


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
                 seed: int = 0, tau_orient_s: float | None = None,
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
        # None: the level's own. For "tm151_filter" this is the residual
        # wander's time (~2 s), not the 0.19 s the noise levels use.
        if tau_orient_s is None:
            tau_orient_s = level_tau_orient_s(level)
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
        self.filter_tau_motion_s = FILTER_TAU_MOTION_S
        self.adr = {}
        for name in ("ahrs_gyro", "ahrs_accel", "ahrs_quat"):
            s = model.sensor(name)
            self.adr[name] = (int(s.adr[0]), int(s.dim[0]))
        self.reset(seed)

    def set_error_params(self, *, orient_rms_roll_deg=None,
                         tau_orient_s=None, filter_tau_motion_s=None) -> None:
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
        if filter_tau_motion_s is not None:
            # "tm151_filter" only: tau while moving, measured ~1 s on ONE unit
            # standing, with the rule that raises it unknown -- the knob to
            # randomise. (For that level `orient_rms_roll_deg` scales only the
            # residual wander, and `tau_orient_s` is the wander's time.)
            self.filter_tau_motion_s = float(filter_tau_motion_s)
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
        self._u = None                     # tm151_filter: gravity direction, body
        self._rate_s = 0.0                 # its smoothed |gyro|, deg/s
        self._w_prev = None                # for the accelerometer's lever arm
        # A misalignment is a FIXED build error, not noise: drawn once per
        # power-on and constant thereafter. Drawing it per tick would make it
        # a noise source the real part does not have.
        m = np.deg2rad(MISALIGN_DEG.get(self.level, ACCEL_MISALIGN_DEG))
        self._accel_tilt = self.rng.uniform(-m, m, size=3) if self.level != "none" \
            else np.zeros(3)
        # Heading drift direction, also fixed per power-on (see
        # YAW_DRIFT_DEG_PER_S). Drawn after the tilt so the tilt keeps its value.
        self._yaw_drift_sign = float(self.rng.choice((-1.0, 1.0))) \
            if self.level != "none" else 0.0

    def restart(self) -> None:
        """The bike was picked up and stood still until the AHRS settled.

        Clears the filter's state -- its gravity direction, the smoothed rate
        that sets tau, the lever arm's last rate -- so the next sample starts
        from the true attitude. NOT `reset`: the same unit is still powered, so
        its misalignment, drift sign and wandering errors carry on. Teleop's
        respawn needs this; without it "tm151_filter" keeps the fallen bike's
        attitude and a ~1 s tau (longer, gated) and the fresh bike falls.
        """
        self._u = None
        self._rate_s = 0.0
        self._w_prev = None
        self._cache = None
        self._acc = 0.0

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

        if self.level == "tm151_filter":
            # The accelerometer is not at the site: ~1 cm off it in the part.
            d = TM151_TO_SITE @ TM151_ACCEL_OFFSET_M
            accel = accel_at_offset(accel, gyro, self._w_prev, dt, d)
            self._w_prev = gyro.copy()

        rms = np.deg2rad(self.orient_rms_deg)
        self._orient_err = _gm_step(self._orient_err, dt, self.tau_orient_s,
                                    rms, self.rng)
        # Yaw additionally DRIFTS: the datasheet quotes it as an error per unit
        # time rather than an RMS, because nothing bounds heading the way
        # gravity bounds roll and pitch. A steady rate about the WORLD
        # vertical, applied as an exact rotation outermost -- it is unbounded,
        # so it does not belong inside the small-angle error quaternion.
        self._yaw_drift += (np.deg2rad(YAW_DRIFT_DEG_PER_S[self.level]) * dt
                            * self._yaw_drift_sign)
        h = 0.5 * self._yaw_drift
        q_drift = np.array([np.cos(h), 0.0, 0.0, np.sin(h)])
        q_err = _small_angle_quat(self._orient_err)
        if self.level != "tm151_filter":   # that one's attitude is made below
            quat_out = _quat_mul(q_drift, _quat_mul(q_err, quat))
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

        if self.level == "tm151_filter":
            # The part's own fusion, on its own corrupted gyro and accelerometer.
            # MuJoCo's accelerometer reads the specific force (+g UP at rest),
            # so the filter's state is the up direction in the body frame.
            up_true = _rotate(np.array([quat[0], -quat[1], -quat[2], -quat[3]]),
                              np.array([0.0, 0.0, 1.0]))
            if self._u is None:
                self._u = up_true.copy()
            self._rate_s += (dt / (FILTER_RATE_SMOOTH_S + dt)) * (
                np.degrees(np.linalg.norm(gyro_out)) - self._rate_s)
            tau = filter_tau(self._rate_s, FILTER_TAU_REST_S, self.filter_tau_motion_s)
            self._u = tilt_filter_step(self._u, gyro_out, accel_out, dt, tau,
                                       FILTER_ACC_GATE)
            # The attitude whose up direction is the filter's: the truth turned,
            # in the body frame, by the rotation taking up_true onto it.
            c = np.cross(up_true, self._u)
            ang = np.arctan2(np.linalg.norm(c), float(up_true @ self._u))
            rv = c / np.linalg.norm(c) * ang if ang > 1e-12 else np.zeros(3)
            q_tilted = _quat_mul(quat, _axis_angle_quat(-rv))
            quat_out = _quat_mul(q_drift, _quat_mul(q_err, q_tilted))
            quat_out /= np.linalg.norm(quat_out)

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
