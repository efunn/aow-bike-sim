"""Driving controller: straight lines and circles on a gain-scheduled LQR.

Balance at speed is steering-dominated and speed-dependent (backward driving
reverses the caster effect), so gains come from `design_gain_schedule` — the
finite-amplitude identification recipe at a mirrored grid of forward speeds
(v = 0 recovers the stationary controller) — interpolated by measured speed.

Path tracking follows the pivot's recipe: feasible references + feedforward,
feedback only corrects residuals. Modes:
  LINE   — anchor + heading; at v_ref = 0 degenerates to station-keeping.
  CIRCLE — center/radius/direction; references: yaw_rate = dir*v/R, lean into
           the turn atan(v^2/(R g)), kinematic steer atan(L/R).

Steer clamp (circle mode): applied to the feedback correction *around* the
kinematic feedforward, not the absolute angle — tight circles legitimately
need large absolute steer; the identified-model validity argument bounds the
deviation from equilibrium.

Sign conventions (see balance.py): roll > 0 leans right (-Y); steer > 0 turns
the front left (+Y); dir = +1 is a CCW (left) circle, whose center sits to the
bike's left and requires roll_ref < 0 (lean left) and steer_ff > 0.
"""

from __future__ import annotations

import mujoco
import numpy as np

from .balance import LQRBalance, extract_state, mix
from .linearize import design_gain_schedule
from .pivot import YawProfile

GRAVITY = 9.81


class SpeedProfile:
    """Accel-limited tracking of a retargetable speed command."""

    def __init__(self, accel: float, v_max: float):
        self.accel, self.v_max = accel, v_max
        self.v_ref = 0.0
        self.target = 0.0

    def set_target(self, v: float) -> None:
        self.target = float(np.clip(v, -self.v_max, self.v_max))

    def step(self, dt: float) -> float:
        dv = np.clip(self.target - self.v_ref, -self.accel * dt, self.accel * dt)
        self.v_ref += dv
        return self.v_ref


class DriveController(LQRBalance):
    """Line/circle driving on the interpolated gain schedule."""

    def __init__(self, params, model):
        super().__init__(params, model)   # shared machinery; parent K unused
        dc = params["control"]["drive"]
        self.wheelbase = params["bike"]["wheelbase"]
        self.r_wheel = params["omni_wheel"]["outer_radius"]
        self.speed_kp = dc["speed_kp"]
        self.steer_ff_gain = dc["steer_ff_gain"]
        self.lean_ff = dc["lean_ff"]
        self.ki_lat = dc["ki_lat"]
        self.int_limit = np.deg2rad(dc["int_limit_deg"])
        self.yaw_slew = dc["yaw_slew"]
        self.yaw_accel = dc["yaw_accel"]
        self.steer_ff_max = np.deg2rad(dc["steer_ff_max_deg"])
        self.turn_rate_margin = dc["turn_rate_margin"]
        self.yaw_slew_sharp = dc["yaw_slew_sharp"]
        self.reverse_turn_scale = dc["reverse_turn_scale"]
        self.reverse_avoid_band = tuple(dc["reverse_avoid_band"])
        self.flip_cfg = params["control"]["flip"]
        from .balance import lat_gain
        self.lat_per_d = lat_gain(params)
        self._psi_dot_ref = 0.0
        self.profile = SpeedProfile(dc["accel"], dc["v_max"])
        self.speeds, self.Ks, self.fit_r2_grid = design_gain_schedule(params, model)
        # Standstill gains: crawl-vs-roll response used as a self-consistent
        # roll-PD for balance during scripted maneuvers (steer committed).
        self._K0 = self.Ks[int(np.argmin(np.abs(self.speeds)))]
        # mode state
        self.mode = "line"
        self._anchor = np.zeros(2)
        self._psi_path = 0.0
        self._center = np.zeros(2)
        self._radius = 1.0
        self._dir = 1
        self._psi = 0.0
        self._psi_raw_prev = 0.0
        self._stop_pending = False
        self._int_lat = 0.0   # integral steer correction [rad], anti-windup clamped
        # flip-maneuver state
        self._flip_profile: YawProfile | None = None
        self._flip_dir = 1
        self._flip_t0 = 0.0
        self._flip_psi0 = 0.0
        self._flip_center = np.zeros(2)
        self._flip_steer = 0.0   # scripted steer, rate-limited integrator

    # -- gain schedule -----------------------------------------------------

    def _K(self, v: float) -> np.ndarray:
        s = self.speeds
        if v <= s[0]:
            return self.Ks[0]
        if v >= s[-1]:
            return self.Ks[-1]
        i = int(np.searchsorted(s, v)) - 1
        f = (v - s[i]) / (s[i + 1] - s[i])
        return (1 - f) * self.Ks[i] + f * self.Ks[i + 1]

    # -- commands ----------------------------------------------------------

    def reset(self, model, data):
        super().reset(model, data)
        s = extract_state(data, self._ref_pos)
        self._psi = self._psi_raw_prev = s.yaw
        self.profile.v_ref = 0.0
        self.profile.target = 0.0
        self.command_line(data)

    def command_line(self, data, heading: float | None = None) -> None:
        """(Re-)anchor a straight path at the current position. `heading` in
        rad (unwrapped-compatible); defaults to the current heading."""
        self.mode = "line"
        self._anchor = data.qpos[:2].copy()
        self._psi_path = self._psi if heading is None else heading
        self._psi_path_target = self._psi_path
        self._psi_dot_ref = 0.0
        self._int_lat = 0.0

    def command_circle(self, data, radius: float, direction: int) -> None:
        """Circle through the current position; direction +1 = CCW (left)."""
        self.mode = "circle"
        self._radius = radius
        self._dir = 1 if direction >= 0 else -1
        c, s = np.cos(self._psi), np.sin(self._psi)
        self._center = data.qpos[:2] + self._dir * radius * np.array([-s, c])
        self._int_lat = 0.0

    def command_heading(self, data, delta: float) -> None:
        """Turn by `delta` rad. At low speed this runs the pivot recipe ("arc"
        mode: positional reference on the arc around the front contact — the
        position feedback is what brakes yaw momentum at the end of the turn);
        at speed the line heading slews under the bike ("rotating carrot")
        with lean feedforward. Mashable: deltas accumulate."""
        s = extract_state(data, self._ref_pos)
        if self.mode == "arc":
            self._psi_path_target += delta          # extend the ongoing arc
            return
        if abs(s.v_lon) < 0.3:
            self.mode = "arc"
            c_, s_ = np.cos(self._psi), np.sin(self._psi)
            self._center = data.qpos[:2] + self.wheelbase * np.array([c_, s_])
            self._psi_path = self._psi
            self._psi_path_target = self._psi + delta
            self._psi_dot_ref = 0.0
            self._int_lat = 0.0
        else:
            if self.mode != "line":
                self.command_line(data)
            self._psi_path_target += delta

    def command_flip(self, data, direction: int = 1) -> float:
        """180-degree swap-ends about the midline, from ~standstill. Pre-steers
        the front to ~90 deg (frees it to roll laterally), holds while the rear
        crawls the 180 spin, then unwinds. Returns the total duration [s]."""
        d = 1 if direction >= 0 else -1
        self.mode = "flip"
        self._flip_profile = YawProfile(
            d * np.pi, self.flip_cfg["yaw_rate"], self.flip_cfg["yaw_accel"])
        self._flip_dir = d
        self._flip_t0 = data.time
        self._flip_psi0 = self._psi
        c_, s_ = np.cos(self._psi), np.sin(self._psi)
        self._flip_center = data.qpos[:2] + (self.wheelbase / 2) * np.array([c_, s_])
        self._flip_steer = data.qpos[self._sj]
        self.profile.v_ref = 0.0
        self.profile.target = 0.0
        return self.flip_cfg["pre_steer_time"] + self._flip_profile.duration

    def set_speed(self, v: float) -> None:
        """Set the speed target; targets inside the reverse instability pocket
        snap to the nearest band edge (dwelling there diverges — transiting
        during ramps is fine)."""
        lo, hi = self.reverse_avoid_band
        if lo < v < hi:
            v = hi if (v - lo) > (hi - v) else lo
        self.profile.set_target(v)

    def stop(self) -> None:
        """Ramp the speed target to zero (keeps the current path)."""
        self.profile.set_target(0.0)

    def command_stop(self) -> None:
        """Brake and settle where the bike halts: ramp the target to zero and
        drop a fresh line anchor at the moment v_ref reaches zero (re-anchoring
        immediately would pull the bike back by its braking distance)."""
        self.profile.set_target(0.0)
        self._stop_pending = True

    # -- control law -------------------------------------------------------

    def _advance_slew(self, cap: float, max_lag: float = 0.35) -> float:
        """Trapezoid-profile the path heading toward its target; returns the
        current heading-rate reference. Governor: pause while the bike lags
        the reference by more than `max_lag` — the reference's deceleration
        only brakes the bike if the bike is actually on the reference."""
        slew_err = self._psi_path_target - self._psi_path
        des = np.sign(slew_err) * min(
            cap, np.sqrt(2.0 * self.yaw_accel * abs(slew_err)))
        if abs(self._psi - self._psi_path) > max_lag:
            des = 0.0
        self._psi_dot_ref += float(np.clip(
            des - self._psi_dot_ref,
            -self.yaw_accel * self.dt, self.yaw_accel * self.dt))
        step_ = self._psi_dot_ref * self.dt
        if abs(step_) >= abs(slew_err):
            step_ = slew_err
            self._psi_dot_ref = 0.0
        self._psi_path += step_
        return self._psi_dot_ref

    def _flip_compute(self, data, s) -> np.ndarray:
        """Swap-ends maneuver in three phases keyed off τ = time − t0:
          pre-steer — wind the front to hold_deg (yaw held, station-keep);
          spin      — front held, rear crawl tracks the radius-L/2 circle
                      about the captured center + balances (yaw profile runs);
          settle    — unwind the front to 0, station-keep, hand back to line.
        The rear differential is crawl feedback that both tracks the circle and
        balances roll (steer is committed, so its feedback entries are zeroed —
        balance falls to crawl, the standstill regime). The hub closes a slow
        longitudinal loop on the center error, re-centering the front-pivot
        excursion. See the decisions doc for why the mid-spin bulge (~1 L) is
        intrinsic without trajectory optimization."""
        prof = self._flip_profile
        cfg = self.flip_cfg
        L = self.wheelbase
        tau = data.time - self._flip_t0
        hold = np.deg2rad(cfg["hold_deg"]) * self._flip_dir
        t_pre = cfg["pre_steer_time"]

        psi_target = self._flip_psi0 + self._flip_dir * np.pi
        if tau < t_pre:                       # pre-steer (front freed to ~90)
            steer_target = hold * min(1.0, tau / t_pre)
            psi_off = psi_dot_ref = 0.0
        else:                                 # spin (front held at 90)
            psi_off, psi_dot_ref, _ = prof.eval(tau - t_pre)
            steer_target = hold
        dmax = cfg["steer_rate"] * self.dt
        self._flip_steer += float(np.clip(steer_target - self._flip_steer,
                                          -dmax, dmax))

        psi_ref = self._flip_psi0 + psi_off
        cr, sr = np.cos(psi_ref), np.sin(psi_ref)
        p_ref = self._flip_center - (L / 2) * np.array([cr, sr])
        cy, sy = np.cos(s.yaw), np.sin(s.yaw)
        err_w = data.qpos[:2] - p_ref
        e_lon = cy * err_w[0] + sy * err_w[1]
        e_lat = -sy * err_w[0] + cy * err_w[1]
        v_lat_ref = -(L / 2) * psi_dot_ref
        x = np.array([
            e_lat, s.roll, self._psi - psi_ref, 0.0,
            s.v_lat - v_lat_ref, s.roll_rate, data.qvel[5] - psi_dot_ref, 0.0,
        ])
        d_cmd = float(-self._K0[0] @ x)
        v_hub = -cfg["hub_kp"] * e_lon        # longitudinal center loop

        a, b = mix(v_hub, d_cmd)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = self._flip_steer

        # On yaw completion, hand back to line mode — its station-keeping
        # brings the front (held at 90) back to straight and settles the stop.
        if (tau > t_pre + prof.duration
                and abs(self._psi - psi_target) < np.deg2rad(8)
                and abs(data.qvel[5]) < 0.3):
            self.command_line(data, heading=psi_target)
        return u

    def _compute(self, model, data):
        s = extract_state(data, self._ref_pos)
        dpsi = np.arctan2(np.sin(s.yaw - self._psi_raw_prev),
                          np.cos(s.yaw - self._psi_raw_prev))
        self._psi += dpsi
        self._psi_raw_prev = s.yaw

        if self.mode == "flip":
            return self._flip_compute(data, s)

        v_ref = self.profile.step(self.dt)
        if self._stop_pending and v_ref == 0.0:
            self.command_line(data)   # settle right here
            self._stop_pending = False
        p = data.qpos[:2]
        vw = data.qvel[:2]   # world-frame ground velocity

        # NOTE: the identified model's velocity state is the *cross-track rate*
        # (world v_y in the ID frame, which contains v*sin(heading error)) —
        # not the body-frame lateral slip velocity. Feeding the body-frame one
        # loses the dominant v*psi term at speed and destabilizes cruise.
        if self.mode == "circle":
            r_vec = p - self._center
            rho = max(float(np.linalg.norm(r_vec)), 1e-6)
            r_hat = r_vec / rho
            tangent = self._dir * np.array([-r_hat[1], r_hat[0]])
            e_lat = -self._dir * (rho - self._radius)
            e_lat_rate = -self._dir * float(r_hat @ vw)
            psi_t = np.arctan2(tangent[1], tangent[0])
            e_psi = np.arctan2(np.sin(self._psi - psi_t), np.cos(self._psi - psi_t))
            yaw_rate_ref = self._dir * v_ref / self._radius
            roll_ref = -self._dir * self.lean_ff * np.arctan(
                v_ref**2 / (self._radius * GRAVITY))
            steer_ff = self._dir * self.steer_ff_gain * np.arctan(
                self.wheelbase / self._radius)
            e_lon = 0.0
            d_ff = 0.0
        elif self.mode == "arc":
            # Pivot recipe: positional reference on the arc around the front
            # contact. The arc-position feedback brakes yaw momentum at the
            # end of the turn (a heading-only reference lets the bike spin
            # past and diverge in the nonlinear yaw-crawl regime).
            psi_dot_ref = self._advance_slew(self.yaw_slew, max_lag=0.15)
            c_, s_ = np.cos(self._psi_path), np.sin(self._psi_path)
            p_ref = self._center - self.wheelbase * np.array([c_, s_])
            cy, sy = np.cos(s.yaw), np.sin(s.yaw)
            err_w = p - p_ref
            e_lon = cy * err_w[0] + sy * err_w[1]
            e_lat = -sy * err_w[0] + cy * err_w[1]
            v_lat_ref = -psi_dot_ref * self.wheelbase
            e_lat_rate = s.v_lat - v_lat_ref
            e_psi = self._psi - self._psi_path
            yaw_rate_ref = psi_dot_ref
            roll_ref = 0.0
            steer_ff = 0.0
            d_ff = v_lat_ref / self.lat_per_d
            if (self._psi_dot_ref == 0.0
                    and self._psi_path == self._psi_path_target
                    and abs(e_psi) < 0.05
                    and abs(data.qvel[5]) < 0.3):   # yaw momentum spent
                self.command_line(data, heading=self._psi_path_target)
        else:
            # A line-mode turn that decays to near-standstill loses steering
            # authority and the carrot scheme fails — hand the ongoing turn
            # off to arc mode (keeps the slew state and target).
            if (abs(s.v_lon) < 0.25
                    and abs(self._psi_path_target - self._psi_path) > 0.03):
                self.mode = "arc"
                c_, s_ = np.cos(self._psi), np.sin(self._psi)
                self._center = p + self.wheelbase * np.array([c_, s_])
                return self._compute(model, data)
            # Rotating carrot (at speed): the line heading slews under the
            # bike, feedforward-carried like circle mode — the steer ff moves
            # the operating point (up to steer_ff_max) and feedback stays
            # clamped around it, so the deviation from equilibrium remains in
            # the identified model's validity. Turn-rate ceiling = margin x
            # the kinematic arc rate at the ff ceiling.
            steer_rate_cap = (self.turn_rate_margin * abs(s.v_lon)
                              * np.tan(self.steer_ff_max) / self.wheelbase)
            if s.v_lon < 0:
                steer_rate_cap *= self.reverse_turn_scale
            crawl_frac = max(0.0, 1.0 - abs(s.v_lon) / 0.3)
            slew_cap = min(self.yaw_slew_sharp,
                           crawl_frac * self.yaw_slew + steer_rate_cap)
            psi_dot_ref = self._advance_slew(slew_cap)
            if psi_dot_ref:
                self._anchor = p.copy()
            t_hat = np.array([np.cos(self._psi_path), np.sin(self._psi_path)])
            n_hat = np.array([-t_hat[1], t_hat[0]])
            d_vec = p - self._anchor
            e_lat = float(n_hat @ d_vec)
            e_lat_rate = float(n_hat @ vw) - crawl_frac * (-psi_dot_ref * self.wheelbase)
            e_psi = np.arctan2(np.sin(self._psi - self._psi_path),
                               np.cos(self._psi - self._psi_path))
            yaw_rate_ref = psi_dot_ref
            roll_ref = -self.lean_ff * np.arctan(
                s.v_lon * psi_dot_ref / GRAVITY)
            d_ff = crawl_frac * (-psi_dot_ref * self.wheelbase) / self.lat_per_d
            # Kinematic steer for the commanded arc rate; sign flips in
            # reverse (backing turns steer opposite) — without this bias the
            # feedback fights the wrong way and reverse turns diverge. Allowed
            # up to steer_ff_max (well past the feedback clamp): it carries
            # the equilibrium, the clamp bounds only the correction around it.
            if abs(s.v_lon) > 0.25:
                steer_ff = float(np.clip(
                    self.steer_ff_gain
                    * np.arctan(psi_dot_ref * self.wheelbase / s.v_lon),
                    -self.steer_ff_max, self.steer_ff_max))
            else:
                steer_ff = 0.0
            e_lon = float(t_hat @ d_vec)

        # Integral lean trim: at balance the turning radius is set by roll, not
        # steer (R = v^2 / (g tan(roll))), so a fraction-of-a-degree roll
        # residual biases the tracked radius by ~10%. A slow integral on
        # cross-track error trims roll_ref to kill that bias. Sign: parked
        # left of path (e_lat > 0) -> lean more to the right (+roll).
        self._int_lat = float(np.clip(self._int_lat + self.ki_lat * e_lat * self.dt,
                                      -self.int_limit, self.int_limit))
        roll_ref += self._int_lat

        sj, sd = self._sj, self._sd
        x = np.array([
            e_lat, s.roll - roll_ref, e_psi, data.qpos[sj] - steer_ff,
            e_lat_rate, s.roll_rate, data.qvel[5] - yaw_rate_ref,
            data.qvel[sd],
        ])
        d_cmd, steer_fb = -self._K(s.v_lon) @ x
        d_cmd += d_ff
        steer = steer_ff + float(np.clip(steer_fb, -self.steer_limit,
                                         self.steer_limit))

        common = v_ref / self.r_wheel + self.speed_kp * (v_ref - s.v_lon)
        if (self.mode in ("line", "arc") and abs(v_ref) < 0.02
                and abs(self.profile.target) < 0.02):
            common += -self.x_kp * e_lon   # station-keeping (line anchor / arc radius)

        a, b = mix(common, d_cmd)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = steer
        return u
