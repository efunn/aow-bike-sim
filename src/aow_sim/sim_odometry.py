"""Run the ONBOARD velocity estimator against the SIMULATED sensor suite.

The point is to close a loop that is currently open. In sim every controller
reads `extract_state(data, ...)`, which is MuJoCo ground truth. On the bike the
same call reads `HardwareData`, whose velocity was written by
`hw/odometry.VelocityEstimator` from noisy sensors. So a policy is trained
against a velocity it will never actually be given, and the estimator's error
depends on how the bike moves -- which depends on the policy. Neither side of
that loop can be evaluated while they are kept apart.

NOTHING NEW IS SIMULATED HERE. The model already carries the whole hardware
sensor suite -- `ahrs_gyro`, `ahrs_accel` and `ahrs_quat` on `ahrs_site`, at the
IMU's real chassis position, plus the input-shaft and steer encoders. This
module only reads those instead of reading the truth beside them, and feeds
them to the same estimator class the Pi runs.

THE SWAP IS DELIBERATELY EXPLICIT (see `estimated`). The controller must see
the estimate while PHYSICS still sees the truth, so the estimate cannot simply
be written into `data.qvel` -- that would corrupt the integration. It is safe to
swap around a controller call because `control/drive.py` makes no MuJoCo calls
at all: it reads `data` and writes `data.ctrl`, and never re-derives anything.
Check that assumption still holds before reusing this anywhere else.

WHAT THIS IS NOT, yet: the sim sensors are CLEAN. There is no gyro bias, no
accelerometer noise and no encoder quantisation, so the estimate here is
better than the bike will ever see. Numbers measured through this module are a
FLOOR on the error, not a prediction of it. See
docs/plans/odometry-in-the-loop.md.
"""

from __future__ import annotations

from contextlib import contextmanager

import numpy as np

from .control.steer import XC330_COUNTS_PER_RAD
from .hw.dynamixel import CONTROL_HZ_DEFAULT, RateFilter, _pos_delta
from .hw.odometry import VelocityEstimator, body_to_world

SENSORS = ("ahrs_gyro", "ahrs_accel", "ahrs_quat",
           "input_a_vel", "input_b_vel", "input_a_pos", "input_b_pos",
           "steer_pos")

# ENCODER MODELS. "ideal" reads the joint-velocity sensors: instantaneous,
# unquantised, no lag -- a floor on the error, and what this module did
# exclusively until 2026-08-27. "counts" reproduces the hardware path instead:
# quantise the shaft angle to 4096 counts/rev, difference it over the tick, and
# push that through the same `RateFilter` the Pi runs.
#
# ONE COUNT IS 0.236 mm OF TRAVEL AT THE WHEEL (2*pi/4096 rad at the servo,
# times belt_ratio 3.0, times the 0.0512 m rolling radius). As a VELOCITY it is
# worth q/T, where T is the DIFFERENCING SPAN, not the sample period -- so
# sampling faster does not make it smaller. Measured from that arc:
#
#     span 10 ms -> 23.6 mm/s      span 25 ms -> 9.4 mm/s (the default)
#     span 50 ms -> 4.7 mm/s
#
# against a v_max of 1200 mm/s and an estimator whose v_lat error at standstill
# is already ~42 mm/s. So quantisation is a minor term next to slip -- which is
# the point of measuring it rather than assuming either way.
#
# "reported" is the THIRD option the hardware actually offers: the XC430's own
# Present Velocity(128) register, which `ServoBus(velocity_source="reported")`
# takes wholesale. Same encoder counts, different filter -- the servo smooths
# internally like a ~50 ms BOXCAR (uniform, no taper), i.e. ~25 ms of lag on a
# bike whose fall time constant is 113 ms. That is 3x the lag of our own 25 ms
# / taper 0.5 default, which is why hw/dynamixel.py re-derives velocity from
# position instead of reading the register.
#
# Modelling it as a RateFilter at OUR tick rate is an approximation: the servo
# runs its average on its own clock, not ours. It reproduces the span and the
# group delay, which is what the lag argument turns on.
ENCODERS = ("ideal", "counts", "reported")

# encoder -> (window_ms, taper) for the differencing filter. taper 1.0 is a
# uniform boxcar; 0.5 ramps to half weight at the window edge. See RateFilter.
ENCODER_FILTER = {
    "counts": (25.0, 0.5),      # hw/dynamixel.py's default, swept in sim
    "reported": (50.0, 1.0),    # the servo's own internal estimate
}


def _rpy(quat) -> tuple[float, float, float]:
    """(roll, pitch, yaw) from a wxyz quaternion — the same convention
    `hw/run_bike._rpy` applies to the AHRS reading."""
    w, x, y, z = quat
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return float(roll), float(pitch), float(yaw)


# Speed at which the front-wheel constraint is trusted completely. Below it the
# `v_lon * tan(theta)` term carries proportionally less information, because at
# a standstill the constraint says nothing about lateral motion at all.
# 0.25 m/s is a first pass, not a tuned value.
V_REF = 0.25

# mode -> (which lateral estimator, which channels are ESTIMATED rather than
# taken from truth). The per-channel modes exist because the two are NOT
# equally to blame: measured 2026-08-26, estimating v_lon alone costs at most
# 0.3 deg of extra roll and never falls, while estimating v_lat alone falls in
# every regime. Isolating them in TELEOP is how that gets confirmed by feel
# rather than only in a headless table.
MODES = {
    "front":    ("front", "both"),   # what the Pi runs today
    "blend":    ("blend", "both"),   # experimental front+roller mix
    "lon_only": ("front", "lon"),    # estimated v_lon, TRUE v_lat
    "lat_only": ("front", "lat"),    # TRUE v_lon, estimated v_lat
}


class SimOdometry:
    """`VelocityEstimator` driven by the simulated sensors, not by truth.

    `fusion` selects which lateral estimate the controller is handed:

      "front"  what hw/odometry.py does today -- the front-wheel rolling
               constraint, confidence-weighted by cos^2(theta), with
               accelerometer coasting through the blind spot. This is the
               path the Pi runs.

      "blend"  EXPERIMENTAL, and not in hw/odometry.py yet: a speed-aware
               mix of the front constraint and the ROLLER kinematics. Measured
               2026-08-26 against the current contact model, the rollers are no
               longer the estimator that does not work -- they beat the front
               constraint outright at standstill (25.9 vs 44.4 mm/s) and in
               reverse (20.8 vs 27.6), which is exactly where the front
               constraint is structurally blind. See
               docs/plans/odometry-rewrite.md.

    NOTE "blend" BYPASSES `VelocityEstimator.update`, and therefore its
    accelerometer propagation. That is deliberate for an A/B: the benchmark
    numbers above were measured on the raw per-sample estimates, so this
    reproduces what was measured rather than something adjacent to it.
    """

    def __init__(self, model, params: dict, mode: str = "front",
                 encoder: str = "ideal", odo_hz: float = CONTROL_HZ_DEFAULT,
                 window_ms: float | None = None, taper: float | None = None):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {tuple(MODES)}, got {mode!r}")
        if encoder not in ENCODERS:
            raise ValueError(f"encoder must be one of {ENCODERS}, got {encoder!r}")
        self.mode = mode
        self.encoder = encoder
        self.fusion, self.channels = MODES[mode]
        self.params = params
        # THE ESTIMATOR HAS ITS OWN CLOCK, and this is a fix rather than a
        # feature. `VelocityEstimator` INTEGRATES -- it propagates v_lat on
        # acceleration and applies a confidence-weighted correction once per
        # tick -- so the rate it is ticked at changes what it does. Before
        # this it inherited whatever its caller looped at: 50 Hz from
        # GeneralEnv (`ctrl_dt`), and 2500 Hz from teleop, which passed
        # `model.opt.timestep`. Three callers, three different estimators, none
        # of them the Pi's 100 Hz.
        self.odo_hz = float(odo_hz)
        self.odo_dt = 1.0 / self.odo_hz
        # The encoder picks the filter; an EXPLICIT argument overrides it, so
        # the lag can be swept without inventing a new encoder name. Defaulting
        # these to concrete numbers would have been worse: the table would then
        # silently lose to a default the caller never chose.
        win, tap = ENCODER_FILTER.get(encoder, (25.0, 0.5))
        self.window_ms = win if window_ms is None else float(window_ms)
        self.taper = tap if taper is None else float(taper)
        self.adr = {}
        for name in SENSORS:
            sn = model.sensor(name)
            self.adr[name] = (int(sn.adr[0]), int(sn.dim[0]))
        self._init_state(params)

    def _init_state(self, params) -> None:
        self.est = VelocityEstimator(params)
        self._acc = 0.0                 # unconsumed time since the last tick
        self._last = (0.0, 0.0)         # most recent estimate, held between ticks
        self._filt = {k: RateFilter(self.window_ms, self.taper,
                                    1000.0 / self.odo_hz)
                      for k in ("a", "b")}
        self._prev_counts = {}          # k -> quantised shaft counts

    def _read(self, data, name):
        adr, dim = self.adr[name]
        return data.sensordata[adr:adr + dim]

    def reset(self, model, params) -> None:
        """Fresh filter state. The estimator integrates, so an episode that
        reuses a warmed-up filter is not the episode the bike will fly."""
        self._init_state(params)

    # -- the encoder ------------------------------------------------------

    def _counts(self, data, which: str) -> int:
        """Shaft angle -> quantised encoder counts, as the servo reports them.

        `input_*_pos` is the INPUT SHAFT; the servo sits belt_ratio behind it,
        which is what puts the encoder on the slow side and buys the aliasing
        margin. Quantising here rather than at the input shaft is the whole
        point -- one count is 0.236 mm at the wheel BECAUSE of that ratio.
        """
        rad_input = float(self._read(data, f"input_{which}_pos")[0])
        return int(round(rad_input / self.est.belt_ratio * XC330_COUNTS_PER_RAD))

    def _wheel_rates(self, data, dt: float) -> tuple[float, float]:
        """(w_servo_a, w_servo_b) by the configured encoder model."""
        if self.encoder == "ideal":
            # Instantaneous joint velocity: no quantisation, no lag. A FLOOR.
            return (float(self._read(data, "input_a_vel")[0]) / self.est.belt_ratio,
                    float(self._read(data, "input_b_vel")[0]) / self.est.belt_ratio)
        out = []
        for k in ("a", "b"):
            counts = self._counts(data, k)
            prev = self._prev_counts.get(k)
            self._prev_counts[k] = counts
            if prev is None:                      # first tick: no interval yet
                out.append(self._filt[k].peek())
                continue
            # Unwrapped exactly as hw/dynamixel.read does -- the hubs run in
            # Velocity Control Mode and report a single-turn position.
            d = _pos_delta(counts, prev)
            raw = (d / XC330_COUNTS_PER_RAD) / dt
            out.append(self._filt[k].update(raw))
        return out[0], out[1]

    def _advance(self, data, dt: float) -> tuple[float, float]:
        """One estimator tick from the SIMULATED sensors -> (v_lon, v_lat)."""
        roll, pitch, _ = _rpy(self._read(data, "ahrs_quat"))
        wa, wb = self._wheel_rates(data, dt)
        steer = float(self._read(data, "steer_pos")[0])
        if self.encoder != "ideal":
            # The steer servo is quantised too: XC330 extended position, the
            # same 4096 counts/rev, through the steering gear ratio. It feeds
            # tan(theta), so its resolution matters to v_lat directly.
            ratio = float(self.params["bike"]["steering"]["gear_ratio"])
            q = XC330_COUNTS_PER_RAD * ratio
            steer = round(steer * q) / q
        yaw_rate = float(self._read(data, "ahrs_gyro")[2])
        if self.fusion == "front":
            return self.est.update(
                dt, wa, wb, steer_joint=steer, yaw_rate=yaw_rate,
                accel_body=np.asarray(self._read(data, "ahrs_accel"), float),
                roll=roll, pitch=pitch)
        v_lon = self.est.longitudinal(wa, wb)
        front, conf = self.est.lateral_from_front(v_lon, steer, yaw_rate)
        roller = self.est.roller_lateral(wa, wb)
        # Two independent sources. The front constraint needs forward speed to
        # say anything; the rollers do not, but they are the axis the wheel is
        # designed to slip in. Weighting by BOTH speed and steer angle takes
        # each where it is informative.
        w = float(np.clip(abs(v_lon) / V_REF, 0.0, 1.0)) * conf
        return float(v_lon), float(w * front + (1.0 - w) * roller)

    # -- the clock --------------------------------------------------------

    def update(self, data, dt: float) -> tuple[float, float]:
        """Advance by `dt` of SIMULATED time; return the estimate on hold.

        `dt` is how much time the CALLER has elapsed, not a tick period. The
        estimator ticks at its own `odo_hz` and HOLDS its last value in
        between, which is exactly what the controller sees on the bike: the
        sense loop runs at 100 Hz and every reader between ticks gets the same
        numbers. Callers may therefore loop at any rate -- teleop at the 2500 Hz
        physics step, GeneralEnv at its 50 Hz control step -- and get the same
        estimator either way.

        A caller slower than `odo_dt` consumes the backlog in whole ticks, so
        the filter still sees the sample count it expects rather than one giant
        step. Leftover time carries to the next call.
        """
        self._acc += float(dt)
        n = int(self._acc / self.odo_dt)
        if n:
            self._acc -= n * self.odo_dt
            # Cap the catch-up: a caller that hands over a huge dt (a reset, a
            # paused viewer) must not spin thousands of ticks on one stale
            # `data`. Every tick would read the SAME sensor values anyway.
            for _ in range(min(n, 4)):
                self._last = self._advance(data, self.odo_dt)
        return self._last

    def world_velocity(self, data, dt) -> np.ndarray:
        v_lon, v_lat = self.update(data, dt)
        _, _, yaw = _rpy(self._read(data, "ahrs_quat"))
        return np.asarray(body_to_world(v_lon, v_lat, yaw), dtype=float)

    @contextmanager
    def estimated(self, data, dt: float):
        """Controller sees the estimate; physics keeps the truth.

        Wrap ONLY the controller call:

            with odo.estimated(data, dt):
                ctl.step(model, data)
            mujoco.mj_step(model, data)
        """
        true_v = data.qvel[:2].copy()
        e_lon, e_lat = self.update(data, dt)
        _, _, yaw = _rpy(self._read(data, "ahrs_quat"))
        cy, sy = np.cos(yaw), np.sin(yaw)
        t_lon = cy * true_v[0] + sy * true_v[1]
        t_lat = -sy * true_v[0] + cy * true_v[1]
        lon = e_lon if self.channels in ("both", "lon") else t_lon
        lat = e_lat if self.channels in ("both", "lat") else t_lat
        data.qvel[:2] = np.asarray(body_to_world(lon, lat, yaw), dtype=float)
        try:
            yield
        finally:
            data.qvel[:2] = true_v
