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

from .hw.odometry import VelocityEstimator, body_to_world

SENSORS = ("ahrs_gyro", "ahrs_accel", "ahrs_quat",
           "input_a_vel", "input_b_vel", "steer_pos")


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

    def __init__(self, model, params: dict, mode: str = "front"):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {tuple(MODES)}, got {mode!r}")
        self.mode = mode
        self.fusion, self.channels = MODES[mode]
        self.est = VelocityEstimator(params)
        self.adr = {}
        for name in SENSORS:
            s = model.sensor(name)
            self.adr[name] = (int(s.adr[0]), int(s.dim[0]))

    def _read(self, data, name):
        adr, dim = self.adr[name]
        return data.sensordata[adr:adr + dim]

    def reset(self, model, params) -> None:
        """Fresh filter state. The estimator integrates, so an episode that
        reuses a warmed-up filter is not the episode the bike will fly."""
        self.est = VelocityEstimator(params)

    def update(self, data, dt: float) -> tuple[float, float]:
        """One estimator tick from the SIMULATED sensors -> (v_lon, v_lat)."""
        roll, pitch, _ = _rpy(self._read(data, "ahrs_quat"))
        # Input-shaft units -> servo units, the same conversion
        # tests/test_hw_odometry.py makes; the estimator takes what the
        # Dynamixel feedback reports.
        wa = float(self._read(data, "input_a_vel")[0]) / self.est.belt_ratio
        wb = float(self._read(data, "input_b_vel")[0]) / self.est.belt_ratio
        steer = float(self._read(data, "steer_pos")[0])
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
