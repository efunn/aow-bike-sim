"""Baseline stationary balance controllers.

Two controllers behind one interface:
  PDCascade  — transparent reference: roll PD -> rear lateral crawl velocity,
               slow outer loop on lateral drift -> roll setpoint, weak
               longitudinal P -> common-mode drive. Steer held straight.
  LQRBalance — discrete LQR on the numerically linearized full model
               (see linearize.py); free to use steering + crawl together.

Both run at `control.rate_hz` with zero-order hold between updates (physics
steps much faster), use ground-truth simulator state, and saturate to the
actuator ctrlranges. Sensor-only estimation is a later phase.

Conventions (chassis frame: +X forward, +Y left, +Z up):
  roll > 0  = lean right (-Y side down), from ZYX Euler of the chassis quat.
  Differential drive d = drive_a - drive_b; d > 0 crawls the rear contact
  toward -Y (verified empirically; flips with drivetrain.k_roller sign).
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


def mix(common: float, diff: float) -> tuple[float, float]:
    """(common, differential) -> (drive_a, drive_b) input-shaft commands."""
    return common + diff / 2, common - diff / 2


def lat_gain(params: dict) -> float:
    """Rear-contact lateral velocity [m/s] per unit differential d [rad/s].

    ring_rel = d/2, roller spin = k_roller * d/2, contact speed = spin * rho_eff.
    Sign: d > 0 moves the contact toward -Y (matches test_lateral_crawl).
    """
    roller = params["omni_wheel"]["roller"]
    rho_eff = (roller["big_diameter"] + roller["small_diameter"]) / 4
    return -params["drivetrain"]["k_roller"] / 2 * rho_eff


@dataclass
class BikeState:
    roll: float
    roll_rate: float
    yaw: float
    e_lon: float   # fore/aft drift from the reference point, bike-yaw frame
    e_lat: float   # lateral drift, +Y(left) positive
    v_lon: float
    v_lat: float


def extract_state(data: mujoco.MjData, ref_pos: np.ndarray) -> BikeState:
    """Ground-truth state of the chassis freejoint (qpos[0:7], qvel[0:6])."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, data.qpos[3:7])
    R = R.reshape(3, 3)
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    roll_rate = data.qvel[3]  # freejoint angular velocity is body-frame
    c, s = np.cos(yaw), np.sin(yaw)
    to_yaw = np.array([[c, s], [-s, c]])
    e_lon, e_lat = to_yaw @ (data.qpos[:2] - ref_pos[:2])
    v_lon, v_lat = to_yaw @ data.qvel[:2]
    return BikeState(roll, roll_rate, yaw, e_lon, e_lat, v_lon, v_lat)


class _Base:
    """Shared ZOH scheduling, actuator lookup, and saturation."""

    def __init__(self, params: dict, model: mujoco.MjModel):
        self.params = params
        self.dt = 1.0 / params["control"]["rate_hz"]
        self.aid = {n: model.actuator(n).id for n in ("drive_a", "drive_b", "steer")}
        # Saturate only actuators that declare a ctrlrange (steer is unlimited).
        limited = model.actuator_ctrllimited.astype(bool)
        self.lo = np.where(limited, model.actuator_ctrlrange[:, 0], -np.inf)
        self.hi = np.where(limited, model.actuator_ctrlrange[:, 1], np.inf)
        self._ref_pos: np.ndarray | None = None
        self._next_t = 0.0
        self._u = np.zeros(model.nu)

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._ref_pos = data.qpos[:3].copy()
        self._next_t = data.time
        self._u = np.zeros(model.nu)

    def step(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
        """Call every physics step; writes data.ctrl with ZOH at rate_hz."""
        if self._ref_pos is None or data.time < self._next_t - 2 * self.dt:
            self.reset(model, data)  # first call, or viewer was reset
        if data.time + 1e-12 >= self._next_t:
            u = np.asarray(self._compute(model, data), dtype=float)
            self._u = np.clip(u, self.lo, self.hi)
            self._next_t = data.time + self.dt
        data.ctrl[:] = self._u
        return self._u

    def _compute(self, model, data) -> np.ndarray:  # pragma: no cover
        raise NotImplementedError


class PDCascade(_Base):
    """Roll PD -> crawl velocity; outer lateral P(D) -> roll setpoint;
    weak longitudinal P -> common mode; steer held at zero."""

    def __init__(self, params, model):
        super().__init__(params, model)
        g = params["control"]["pd"]
        self.roll_kp, self.roll_kd = g["roll_kp"], g["roll_kd"]
        self.y_kp, self.y_kd = g["y_kp"], g["y_kd"]
        self.x_kp = g["x_kp"]
        self.max_roll_ref = np.deg2rad(g["max_roll_setpoint_deg"])
        self.lat_per_d = lat_gain(params)

    def _compute(self, model, data):
        s = extract_state(data, self._ref_pos)
        # Drifted left (e_lat > 0): lean right (roll > 0) so the crawl that
        # catches the fall carries the base back to the right.
        roll_ref = np.clip(
            self.y_kp * s.e_lat + self.y_kd * s.v_lat,
            -self.max_roll_ref, self.max_roll_ref,
        )
        # Leaning right beyond setpoint: accelerate the base right (-Y) to get
        # under the CoM. A velocity-source base can't stabilize a pendulum from
        # roll feedback alone (the commanded velocity must be relative to the
        # current base velocity), so this is an acceleration-style law:
        # v_cmd = v_now + roll PD.
        v_lat_cmd = s.v_lat - (
            self.roll_kp * (s.roll - roll_ref) + self.roll_kd * s.roll_rate
        )
        d = v_lat_cmd / self.lat_per_d
        common = -self.x_kp * s.e_lon
        a, b = mix(common, d)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = 0.0
        return u


class LQRBalance(_Base):
    """DLQR on the identified reduced lateral model (see linearize.py).

    [d, steer] = -K x_lat; a separate longitudinal P loop supplies the common
    mode (decoupled from lateral balance)."""

    def __init__(self, params, model):
        super().__init__(params, model)
        from .linearize import design_lqr  # deferred: pulls in scipy

        self.K, self.qpos_eq, self.fit_r2 = design_lqr(params, model)
        self.x_kp = params["control"]["pd"]["x_kp"]
        # Hard steer clamp: the lateral model is identified at small steer
        # angles; letting the loop command large ones leaves the region where
        # the linear design is valid (and did destabilize it in practice).
        self.steer_limit = np.deg2rad(params["control"]["lqr"]["steer_limit_deg"])
        self._sj = model.joint("steer_joint").qposadr[0]
        self._sd = model.joint("steer_joint").dofadr[0]
        self._ref_yaw = 0.0

    def reset(self, model, data):
        super().reset(model, data)
        s = extract_state(data, self._ref_pos)
        self._ref_yaw = s.yaw

    def _compute(self, model, data):
        s = extract_state(data, self._ref_pos)
        yaw_err = np.arctan2(np.sin(s.yaw - self._ref_yaw),
                             np.cos(s.yaw - self._ref_yaw))
        x = np.array([
            s.e_lat, s.roll, yaw_err, data.qpos[self._sj],
            s.v_lat, s.roll_rate, data.qvel[5], data.qvel[self._sd],
        ])
        d, steer = -self.K @ x
        steer = np.clip(steer, -self.steer_limit, self.steer_limit)
        a, b = mix(-self.x_kp * s.e_lon, d)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = steer
        return u


def make_controller(name: str, params: dict, model: mujoco.MjModel):
    if name == "pd":
        return PDCascade(params, model)
    if name == "lqr":
        return LQRBalance(params, model)
    raise ValueError(f"unknown controller {name!r}; expected 'pd' or 'lqr'")


def run(model, data, controller, duration: float, on_step=None) -> None:
    """Advance the sim `duration` seconds with the controller in the loop."""
    for _ in range(int(round(duration / model.opt.timestep))):
        controller.step(model, data)
        mujoco.mj_step(model, data)
        if on_step is not None:
            on_step(data)
