"""Crawl pivot about the front contact: heading control as reference tracking.

The bike yaws by crawling the rear wheel along an arc around the (ideally
stationary) front contact. This is implemented as *reference tracking* on the
identified-model balance LQR: a trapezoidal yaw profile generates a feasible
moving reference (yaw, rear-contact arc position, crawl velocity) plus
feedforward, and the balance gain matrix K only cleans up residuals — which
preserves the light-yaw-weight / heavy-yaw-rate-damping design (see
docs/plans/mujoco-modeling-decisions.md): the feedback never has to *generate*
the pivot, only correct it.

Sign conventions (see balance.py): yaw > 0 is CCW about +Z, so during a
positive pivot the rear contact crawls toward body -Y: v_lat_ref = -psi_dot*L.
Lean feedforward: the CoM's *centripetal* acceleration during a pivot points
along body +X (longitudinal — no roll needed); only the *tangential*
acceleration psi_ddot * r_com is lateral, so the roll reference tracks the
profile's accel/decel ramps and is zero during cruise.
"""

from __future__ import annotations

import mujoco
import numpy as np

from .balance import LQRBalance, extract_state, lat_gain, mix

GRAVITY = 9.81


class YawProfile:
    """Trapezoidal (or triangular) yaw-rate profile for a signed heading change.

    eval(t) -> (offset, rate, accel), all signed; offset goes 0 -> delta.
    """

    def __init__(self, delta: float, rate: float, accel: float):
        self.sign = np.sign(delta) if delta else 1.0
        self.D = abs(delta)
        self.accel = accel
        # Cap cruise rate so a short move becomes triangular.
        self.rate = min(rate, np.sqrt(self.D * accel)) if self.D else 0.0
        self.t_ramp = self.rate / accel if accel > 0 else 0.0
        d_ramps = self.rate * self.t_ramp  # distance covered by both ramps
        self.t_cruise = (self.D - d_ramps) / self.rate if self.rate > 0 else 0.0
        self.duration = 2 * self.t_ramp + self.t_cruise

    def eval(self, t: float) -> tuple[float, float, float]:
        if self.D == 0.0 or t >= self.duration:
            return self.sign * self.D, 0.0, 0.0
        if t < 0.0:
            return 0.0, 0.0, 0.0
        a, r = self.accel, self.rate
        if t < self.t_ramp:
            off, rate, acc = 0.5 * a * t * t, a * t, a
        elif t < self.t_ramp + self.t_cruise:
            off = 0.5 * a * self.t_ramp**2 + r * (t - self.t_ramp)
            rate, acc = r, 0.0
        else:
            te = self.duration - t
            off, rate, acc = self.D - 0.5 * a * te * te, a * te, -a
        return self.sign * off, self.sign * rate, self.sign * acc


class PivotController(LQRBalance):
    """Balance LQR + crawl-pivot reference tracking.

    Between pivots (and before the first `command_pivot`) the references are
    constant, so this *is* the stationary balance controller."""

    def __init__(self, params, model):
        super().__init__(params, model)
        pc = params["control"]["pivot"]
        self.wheelbase = params["bike"]["wheelbase"]
        self.yaw_rate = pc["yaw_rate"]
        self.yaw_accel = pc["yaw_accel"]
        self.lean_ff = pc["lean_ff"]
        self.ff_gain = pc["ff_gain"]
        self.lat_per_d = lat_gain(params)
        self.r_com = 0.0        # front contact -> CoM distance; set at reset
        self._profile: YawProfile | None = None
        self._p_front = np.zeros(2)
        self._psi0 = 0.0
        self._t0 = 0.0
        self._psi = 0.0         # unwrapped yaw
        self._psi_raw_prev = 0.0

    # -- lifecycle ---------------------------------------------------------

    def reset(self, model, data):
        super().reset(model, data)
        mujoco.mj_forward(model, data)
        s = extract_state(data, self._ref_pos)
        self._psi = self._psi_raw_prev = s.yaw
        # Whole-model CoM in the body frame -> distance from the front contact.
        c, si = np.cos(s.yaw), np.sin(s.yaw)
        com_rel = data.subtree_com[0][:2] - data.qpos[:2]
        com_x_body = c * com_rel[0] + si * com_rel[1]
        self.r_com = self.wheelbase - com_x_body
        self._anchor(data, delta=0.0)

    def _anchor(self, data, delta: float) -> None:
        """Start a (possibly zero) profile from the current unwrapped yaw,
        pivoting about the current front-contact ground point."""
        c, s = np.cos(self._psi), np.sin(self._psi)
        self._p_front = data.qpos[:2] + self.wheelbase * np.array([c, s])
        self._psi0 = self._psi
        self._profile = YawProfile(delta, self.yaw_rate, self.yaw_accel)
        self._t0 = data.time

    def command_pivot(self, data, delta_yaw: float) -> float:
        """Begin a pivot by `delta_yaw` rad (any magnitude/sign, multi-turn OK).
        Returns the profile duration in seconds."""
        self._anchor(data, delta_yaw)
        return self._profile.duration

    def time_remaining(self, data) -> float:
        return max(0.0, self._t0 + self._profile.duration - data.time)

    # -- control law -------------------------------------------------------

    def _compute(self, model, data):
        s = extract_state(data, self._ref_pos)
        # Unwrapped yaw (raw atan2 yaw jumps at +-pi).
        dpsi = np.arctan2(np.sin(s.yaw - self._psi_raw_prev),
                          np.cos(s.yaw - self._psi_raw_prev))
        self._psi += dpsi
        self._psi_raw_prev = s.yaw

        off, rate, acc = self._profile.eval(data.time - self._t0)
        psi_ref = self._psi0 + off
        c, si = np.cos(psi_ref), np.sin(psi_ref)
        p_ref = self._p_front - self.wheelbase * np.array([c, si])

        # Position error in the *current* yaw frame (K was identified there).
        cy, sy = np.cos(s.yaw), np.sin(s.yaw)
        err_w = data.qpos[:2] - p_ref
        e_lon = cy * err_w[0] + sy * err_w[1]
        e_lat = -sy * err_w[0] + cy * err_w[1]

        v_lat_ref = -rate * self.wheelbase          # rear crawls -Y for CCW yaw
        roll_ref = self.lean_ff * acc * self.r_com / GRAVITY

        x = np.array([
            e_lat, s.roll - roll_ref, self._psi - psi_ref, data.qpos[self._sj],
            s.v_lat - v_lat_ref, s.roll_rate, data.qvel[5] - rate,
            data.qvel[self._sd],
        ])
        d, steer = -self.K @ x
        d += self.ff_gain * v_lat_ref / self.lat_per_d
        steer = np.clip(steer, -self.steer_limit, self.steer_limit)
        a, b = mix(-self.x_kp * e_lon, d)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = steer
        return u
