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

import numpy as np

from .balance import LQRBalance, extract_state, mix
from .pivot import YawProfile
from .steer import clamp_extended, nearest_multiple, wheel_heading

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

    def __init__(self, params, model, design=None):
        super().__init__(params, model, design)   # shared machinery; parent K unused
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
        if design is None:
            from .linearize import design_gain_schedule  # deferred: pulls in scipy
            self.speeds, self.Ks, self.fit_r2_grid = design_gain_schedule(params, model)
        else:
            self.speeds, self.Ks, self.fit_r2_grid = (
                design.speeds, design.Ks, design.fit_r2_grid)
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
        self._flip_base = 0.0    # pi-multiple steer park at maneuver start
        # flick-maneuver (optimized two-arc 180) state
        self._flick = None
        self._flick_dir = 1
        self._flick_t0 = 0.0
        self._flick_p0 = np.zeros(2)
        self._flick_yaw0 = 0.0
        self._flick_steer = 0.0   # commanded steer, rate-limited on unwind
        self._flick_base = 0.0    # pi-multiple steer park at maneuver start
        # ball-shot (RL) maneuver state
        self._ball = None
        self._ball_mirror = False
        self._ball_t0 = 0.0
        self._ball_p0 = np.zeros(2)
        self._ball_steer = 0.0
        self._ball_base = 0.0     # pi-multiple steer park at maneuver start
        self._ball_addr = None    # (qpos_adr, qvel_adr) of the ball freejoint, lazy
        # pivot (RL) maneuver state: front wheel holds its global heading
        self.rake = np.deg2rad(params["bike"]["rake_deg"])
        self._pivot = None
        self._pivot_dir = 1
        self._pivot_t0 = 0.0
        self._pivot_yaw0 = 0.0
        self._pivot_theta0 = 0.0  # held global wheel heading (mod pi)
        self._pivot_u0 = np.zeros(2)
        self._pivot_n0 = np.zeros(2)
        self._pivot_pf0 = np.zeros(2)
        self._pivot_v0 = 0.0      # measured entry speed along the line
        self._pivot_vend = 0.0
        self._pivot_steer = 0.0
        self._pivot_base = 0.0    # pi-multiple steer park at maneuver start
        # general (RL) always-on policy state — NOT a maneuver: no horizon,
        # no start-pose capture, no hand-back. Just a live command.
        self._gen = None
        self._gen_v_cmd = np.zeros(2)   # world-frame velocity command [m/s]
        self._gen_psi_cmd = 0.0         # world heading command [rad, unwrapped]
        self._gen_steer = 0.0           # integrated steer target
        self._gen_prev_a = np.zeros(3)
        self._gen_hold = 0              # ticks left before the next policy query
        self._gen_every = 1             # controller ticks per policy query
        self._gen_u = None              # held action between queries
        self._gen_window_s = 0.0        # policy's velocity-window time constant
        self._gen_zero_lat = False      # trained with v_lat forced to 0?
        # Set by run_drive when --odometry is active. The controller cannot
        # tell truth from estimate on its own -- both arrive as data.qvel --
        # so whoever fills qvel has to say.
        self._odometry_active = False
        self._gen_obs_pitch = False     # does the policy observe pitch?
        self._gen_obs_wings = False     # ...and the wings?
        self._gen_act_wings = False     # does it DRIVE the wings?
        self._gen_wing = 0.0            # integrated wing position target
        self._gen_wing_max = 0.0
        self._gen_vbar_alpha = 1.0      # 1.0 => v_bar is the instantaneous v
        self._gen_v_bar_w = np.zeros(2)  # low-passed WORLD velocity

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
        self.steer_frame.sync(data.qpos[self._sj])

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
        self.steer_frame.sync(data.qpos[self._sj])
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
        self._flip_base = nearest_multiple(data.qpos[self._sj])
        self.profile.v_ref = 0.0
        self.profile.target = 0.0
        return self.flip_cfg["pre_steer_time"] + self._flip_profile.duration

    def command_flick(self, data, direction: int = 1, name: str = "flick") -> float:
        """Two-arc 180 flick (front sweeps 0->180), from ~standstill. Replays
        the offline-optimized `moves/<name>.yaml` feedforward with crawl balance
        underneath. `name`: "flick" (reverse-first) or "flick_fwd" (forward-
        first). Returns the horizon [s]."""
        from .flick import load_move
        self._flick = load_move(name)
        if getattr(self._flick, "kind", "trajectory") == "policy":
            from .flick_spec import OBS_DIM
            if self._flick.obs_dim != OBS_DIM:
                raise ValueError(
                    f"moves/{name} was trained with obs_dim "
                    f"{self._flick.obs_dim} but the current spec is {OBS_DIM}"
                    " — retrain (`python -m aow_sim.train_flick_rl`)")
        self._flick_dir = 1 if direction >= 0 else -1
        self._flick_t0 = data.time
        self._flick_p0 = data.qpos[:2].copy()
        self._flick_yaw0 = self._psi
        self._flick_steer = data.qpos[self._sj]
        self._flick_base = nearest_multiple(data.qpos[self._sj])
        self.mode = "flick"
        # trajectory moves expose .T; RL policy moves expose .horizon
        return getattr(self._flick, "T", getattr(self._flick, "horizon", 4.0))

    def command_ball(self, data, name: str = "ball_rl", mirror: bool = False) -> float:
        """Ball-shot move (docs/plans/ball-shot-move.md): from ~standstill, replay
        the trained `moves/<name>.yaml` RL policy to strike the ball with the side
        stick, then hand back to balance. `mirror=True` reflects a ball-right
        policy to a ball-left start. Returns the replay-safety horizon [s]."""
        from .flick import load_move
        self._ball = load_move(name)
        from .ball_spec import OBS_DIM
        if self._ball.obs_dim != OBS_DIM:
            raise ValueError(
                f"moves/{name} was trained with obs_dim {self._ball.obs_dim}"
                f" but the current spec is {OBS_DIM}"
                " — retrain (`python -m aow_sim.train_ball_rl`)")
        self._ball_mirror = bool(mirror)
        self._ball_t0 = data.time
        self._ball_p0 = data.qpos[:2].copy()
        self._ball_steer = data.qpos[self._sj]
        self._ball_base = nearest_multiple(data.qpos[self._sj])
        self.mode = "ball"
        return getattr(self._ball, "horizon", 5.0)

    def command_pivot_rl(self, data, direction: int = 1, name: str = "pivot_rl",
                         v_end: float = 0.0) -> float:
        """180-deg chassis yaw with the front wheel holding its global ground
        heading (mod pi) — from standstill or a glide. `v_end` [m/s] is the
        target glide speed along the ORIGINAL travel line after the turn (the
        bike then moves backward along its NEW heading). Replays the trained
        `moves/<name>.yaml` RL policy. Returns the horizon [s]."""
        from .flick import load_move
        self._pivot = load_move(name)
        from .pivot_spec import OBS_DIM
        if self._pivot.obs_dim != OBS_DIM:
            raise ValueError(
                f"moves/{name} was trained with obs_dim {self._pivot.obs_dim}"
                f" but the current spec is {OBS_DIM}"
                " — retrain (`python -m aow_sim.train_pivot_rl`)")
        self._pivot_dir = 1 if direction >= 0 else -1
        self._pivot_t0 = data.time
        self._pivot_yaw0 = self._psi
        self._pivot_steer = data.qpos[self._sj]
        self._pivot_base = nearest_multiple(data.qpos[self._sj])
        # Hold target = the wheel's CURRENT global ground heading, base-
        # relative (training starts near steer 0; a pi park must not shift
        # the target — the wheel is pi-symmetric).
        s = extract_state(data, self._ref_pos)
        self._pivot_theta0 = self._psi + wheel_heading(
            float(data.qpos[self._sj]) - self._pivot_base, self.rake)
        self._pivot_u0 = np.array([np.cos(self._pivot_theta0),
                                   np.sin(self._pivot_theta0)])
        self._pivot_n0 = np.array([-np.sin(self._pivot_theta0),
                                   np.cos(self._pivot_theta0)])
        self._pivot_pf0 = data.qpos[:2] + self.wheelbase * np.array(
            [np.cos(s.yaw), np.sin(s.yaw)])
        vf = data.qvel[:2] + self.wheelbase * data.qvel[5] * np.array(
            [-np.sin(s.yaw), np.cos(s.yaw)])
        self._pivot_v0 = float(self._pivot_u0 @ vf)
        self._pivot_vend = float(np.clip(v_end, 0.0,
                                         getattr(self._pivot, "v_max", 0.6)))
        # NOTE: the speed profile is deliberately NOT zeroed — the move may
        # start mid-glide; the profile is re-seeded at handoff.
        self.mode = "pivot_rl"
        return self._pivot.horizon

    # -- general (always-on) policy ----------------------------------------

    def engage_general(self, data, name: str = "general_rl") -> None:
        """Hand the actuators to the general command-conditioned policy and
        leave them there. Unlike every command_* above this is NOT a
        maneuver: there is no horizon and no hand-back, so the only way out
        is another command_* (which re-anchors the analytic controller).

        The initial command is "hold what you are doing now" — current
        heading, current velocity — so engaging never jolts the bike."""
        from .flick import load_move
        self._gen = load_move(name)
        from .general_spec import obs_layout_for, vel_filter_alpha
        # The observation LAYOUT is a property of the policy, carried in its
        # move yaml, so a policy trained without any optional block is 15-wide
        # and still loads here unchanged.
        #
        # Checked as a LAYOUT, not a width. Two optional 2-entry blocks make
        # width ambiguous: a velocity-windowed policy and a pitch-observing
        # one are both 17 wide with entirely different meanings in slots
        # 15-16, and a width check would happily load either as the other and
        # feed the net nonsense with nothing raised.
        self._gen_window_s = float(getattr(self._gen, "vel_window_s", 0.0))
        self._gen_zero_lat = bool(getattr(self._gen, "obs_zero_lat", False))
        # A policy TRAINED on the onboard estimate, being replayed on truth, is
        # getting a CLEANER signal than it ever saw. It will look better than it
        # is. Not an error and deliberately not blocked -- running it both ways
        # is how you find out whether the policy leaned on the estimator's
        # particular error -- but it must not be silent, because nothing else
        # can catch it: obs_odometry does not change the width, so obs_layout
        # sees nothing wrong.
        if (bool(getattr(self._gen, "obs_odometry", False))
                and not self._odometry_active):
            print(f"NOTE: {getattr(self._gen, 'name', '?')} was trained on the "
                  "onboard velocity ESTIMATE, and is being replayed on MuJoCo "
                  "truth.\n  Add --odometry to teleop to match training. On "
                  "hardware this is automatic.")
        self._gen_obs_pitch = bool(getattr(self._gen, "obs_pitch", False))
        self._gen_obs_wings = bool(getattr(self._gen, "obs_wings", False))
        self._gen_act_wings = bool(getattr(self._gen, "act_wings", False))
        # The co-rotating pair. Same single channel, same two observations --
        # what differs is the actuator it drives and that its command is
        # SIGNED, because the mechanism reaches both sides. Kept as its own
        # flag rather than folded into the wings one so the layout guard below
        # still refuses to load a swing policy onto a mirrored-wing bike.
        self._gen_obs_swing = bool(getattr(self._gen, "obs_swing", False))
        self._gen_act_swing = bool(getattr(self._gen, "act_swing", False))
        self._gen_swing = self._gen_obs_swing or self._gen_act_swing
        self._gen_wing_max = np.deg2rad(
            float(getattr(self._gen, "wing_max_deg", 90.0)))
        if self._gen_swing and "swing" not in self.aid:
            raise ValueError(
                f"moves/{name} expects the co-rotating swing wings, but this"
                " model has no `swing` actuator -- build with"
                " build_model(..., swing=True)")
        if (self._gen_obs_wings or self._gen_act_wings) and \
                "wings" not in self.aid:
            raise ValueError(
                f"moves/{name} expects the wings, but this model has no `wings`"
                " actuator — build with build_model(..., wings=True)")
        want = obs_layout_for(self._gen)
        declared = tuple(getattr(self._gen, "obs_layout", ()) or ())
        if declared and declared != want:
            raise ValueError(
                f"moves/{name} declares an observation layout its flags do not"
                f" produce:\n  declared {declared}\n  flags give {want}")
        if self._gen.obs_dim != len(want):
            raise ValueError(
                f"moves/{name} was trained with obs_dim {self._gen.obs_dim}"
                f" but its flags (vel_window_s={self._gen_window_s},"
                f" obs_pitch={self._gen_obs_pitch}) imply {len(want)}"
                " — retrain (`python -m aow_sim.train_general_rl`)")
        # The policy was trained at its own control rate; querying it at the
        # controller's rate would silently change the effective action scale
        # (the steer integrator is rate * dt) and the closed-loop timing.
        # Hold each action for the matching number of controller ticks.
        pol_hz = float(getattr(self._gen, "control_rate_hz", 0.0)
                       or 1.0 / self.dt)
        self._gen_every = max(1, int(round((1.0 / self.dt) / pol_hz)))
        self._gen_dt = self._gen_every * self.dt
        self._gen_hold = 0
        self._gen_u = None
        self._gen_steer = float(data.qpos[self._sj])
        self._gen_prev_a = np.zeros(3)
        # Seed the low-pass from the measured velocity, matching
        # GeneralEnv.reset — engaging must not inject a startup transient the
        # policy never saw in training.
        self._gen_vbar_alpha = vel_filter_alpha(self.dt, self._gen_window_s)
        self._gen_v_bar_w = np.asarray(data.qvel[:2], float).copy()
        self._gen_wing = (float(data.qpos[self._wj]) if self._wj is not None
                          else 0.0)
        s = extract_state(data, self._ref_pos)
        self._gen_psi_cmd = self._psi
        self._gen_v_cmd = data.qvel[:2].copy()
        self.mode = "general"

    def set_command(self, v_cmd_world=None, psi_cmd: float | None = None,
                    dpsi: float | None = None) -> None:
        """Update the live command for the general policy. `v_cmd_world` is a
        world-frame velocity vector [m/s] (stop = (0,0), reverse = the
        opposite vector — both ordinary points, which is why the command is
        a vector and not (course, speed)). `psi_cmd` sets the absolute
        heading; `dpsi` nudges it. Safe to call every tick."""
        if v_cmd_world is not None:
            self._gen_v_cmd = np.asarray(v_cmd_world, dtype=float)[:2].copy()
        if psi_cmd is not None:
            self._gen_psi_cmd = float(psi_cmd)
        if dpsi:
            self._gen_psi_cmd += float(dpsi)

    def set_command_polar(self, speed: float, course_rel: float = 0.0,
                          psi_cmd: float | None = None) -> None:
        """Convenience for teleop: drive at `speed` along `course_rel` radians
        off the commanded heading. Resolved to a vector immediately, so a
        zero speed still leaves a well-defined command."""
        psi = self._gen_psi_cmd if psi_cmd is None else float(psi_cmd)
        th = psi + course_rel
        self.set_command(v_cmd_world=(speed * np.cos(th), speed * np.sin(th)),
                         psi_cmd=psi)

    def _general_compute(self, data, s) -> np.ndarray:
        """Query the general policy and apply. No horizon, no hand-back, no
        start-pose frame: every error is measured against the live command.
        Steer is passed raw (multi-turn) — sin/cos(2*delta) is already
        winding-invariant, so no pi-park rebasing is needed here."""
        from .general_spec import (build_obs, command_to_body, rotate_to_body,
                                   vel_filter_step)
        pol = self._gen
        # Advance the velocity low-pass every CONTROLLER tick, for the same
        # reason the steer integrator below runs at controller rate. alpha was
        # built from `1 - exp(-dt/tau)`, so the continuous time constant is
        # the one the policy trained with regardless of the rate mismatch;
        # this is a finer sampling of the same filter. On hardware qvel[:2] is
        # the odometry's WORLD velocity (hw/state.set_velocity), so no new
        # signal is needed.
        self._gen_v_bar_w = vel_filter_step(self._gen_v_bar_w, data.qvel[:2],
                                            self._gen_vbar_alpha)
        if self._gen_u is None or self._gen_hold <= 0:
            v_cl, v_ct, psi_err = command_to_body(
                self._gen_v_cmd, self._gen_psi_cmd, self._psi)
            vb = None
            if self._gen_window_s > 0.0:
                vb = rotate_to_body(self._gen_v_bar_w[0],
                                    self._gen_v_bar_w[1], self._psi)
            wg = None
            if self._gen_obs_wings or self._gen_obs_swing:
                wg = (float(data.qpos[self._wj]), float(data.qvel[self._wd]))
            obs = build_obs(s.roll, s.roll_rate, data.qvel[5],
                            float(data.qpos[self._sj]),
                            float(data.qvel[self._sd]),
                            s.v_lon, s.v_lat, v_cl, v_ct, psi_err,
                            self._gen_prev_a,
                            zero_lat=self._gen_zero_lat, v_bar=vb,
                            pitch=((s.pitch, s.pitch_rate)
                                   if self._gen_obs_pitch else None),
                            wings=wg)
            act = pol.action(obs)
            steer_rate, hub, diff = act[0], act[1], act[2]
            wing_rate = act[3] if len(act) > 3 else 0.0
            # A zero bound means the channel is disabled; ActionBounds.normalize
            # handles that. Dividing here by hand raised ZeroDivisionError in
            # teleop on a policy trained with hub_max 0.
            self._gen_prev_a = np.array(pol.bounds.normalize(act))
            self._gen_u = (steer_rate, hub, diff, wing_rate)
            self._gen_hold = self._gen_every
        steer_rate, hub, diff, wing_rate = self._gen_u
        self._gen_hold -= 1
        if self._gen_act_wings or self._gen_act_swing:
            # Same clipped rate integration as GeneralEnv.step, at the
            # CONTROLLER rate like the steer integrator beside it. The LOW
            # bound is the mechanism difference and must match the env exactly
            # -- a swing policy replayed against a 0.0 floor would lose one
            # side of its stroke silently.
            lo = -self._gen_wing_max if self._gen_act_swing else 0.0
            self._gen_wing = float(np.clip(
                self._gen_wing + wing_rate * self.dt, lo, self._gen_wing_max))
        # Integrate at the CONTROLLER rate so the commanded steer traces the
        # same ramp the training env produced in one of its longer steps.
        self._gen_steer += steer_rate * self.dt
        if pol.act_dim == 2:                 # feedforward policy: crawl balance
            diff = float(-self._K0[0] @ np.array(
                [s.e_lat, s.roll, 0.0, 0.0, s.v_lat, s.roll_rate, 0.0, 0.0]))
        a, b = mix(hub / self.r_wheel, diff)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = clamp_extended(self._gen_steer)
        if self._gen_act_wings:
            u[self.aid["wings"]] = self._gen_wing
        elif self._gen_act_swing:
            u[self.aid["swing"]] = self._gen_wing
        return u

    def viz_reference(self, data) -> tuple[float, float]:
        """(reference heading [rad, world], reference speed [m/s]) for the
        current mode — for the teleop overlay. Works during flicks (shows the
        180 target heading and the commanded hub speed)."""
        if self.mode == "flick" and self._flick is not None:
            heading = self._flick_yaw0 + self._flick_dir * np.pi
            if getattr(self._flick, "kind", "trajectory") == "policy":
                return heading, 0.0        # policy: target heading, hub not exposed
            tau = data.time - self._flick_t0
            hub = self._flick.hub(min(tau, self._flick.T)) if tau < self._flick.T else 0.0
            return heading, hub
        if self.mode == "flip":
            return self._flip_psi0 + self._flip_dir * np.pi, 0.0
        if self.mode == "pivot_rl":
            return self._pivot_yaw0 + self._pivot_dir * np.pi, -self._pivot_vend
        if self.mode == "general":
            # The command IS the reference: heading plus the speed along it.
            c, sn = np.cos(self._gen_psi_cmd), np.sin(self._gen_psi_cmd)
            return self._gen_psi_cmd, float(c * self._gen_v_cmd[0]
                                            + sn * self._gen_v_cmd[1])
        if self.mode == "circle":
            r_vec = data.qpos[:2] - self._center
            rho = max(float(np.linalg.norm(r_vec)), 1e-6)
            r_hat = r_vec / rho
            tangent = self._dir * np.array([-r_hat[1], r_hat[0]])
            return float(np.arctan2(tangent[1], tangent[0])), self.profile.v_ref
        if self.mode == "arc":
            return self._psi_path, 0.0
        return self._psi_path, self.profile.v_ref   # line

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
            steer_target = self._flip_base + hold * min(1.0, tau / t_pre)
            psi_off = psi_dot_ref = 0.0
        else:                                 # spin (front held at 90)
            psi_off, psi_dot_ref, _ = prof.eval(tau - t_pre)
            steer_target = self._flip_base + hold
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
        u[self.aid["steer"]] = clamp_extended(self._flip_steer)

        # On yaw completion, hand back to line mode — its station-keeping
        # brings the front (held at 90) back to straight and settles the stop.
        if (tau > t_pre + prof.duration
                and abs(self._psi - psi_target) < np.deg2rad(8)
                and abs(data.qvel[5]) < 0.3):
            # The wheel sits at base +- 90, where nearest-pi rounding is
            # ambiguous — keep the maneuver's base as the origin explicitly.
            self.steer_frame.origin = self._flip_base
            self.command_line(data, heading=psi_target)
        return u

    def _flick_policy_compute(self, data, s) -> np.ndarray:
        """Replay an RL policy move (numpy MLPPolicy): build the shared
        observation, query the policy for (steer_rate, hub, diff), integrate the
        steer rate to the servo target, and apply. Full policies drive the
        differential directly; feedforward (2-action) policies get the crawl
        balance underneath. Same completion handoff as the trajectory replay."""
        from .flick_spec import build_obs
        pol = self._flick
        tau = data.time - self._flick_t0
        dd = data.qpos[:2] - self._flick_p0
        e_lat = -np.sin(self._flick_yaw0) * dd[0] + np.cos(self._flick_yaw0) * dd[1]
        yaw_err = pol.target - (self._psi - self._flick_yaw0)
        # Steer obs is relative to the pi-multiple park the maneuver started
        # from: training always starts the wheel near 0, so a post-flick park
        # at pi must not leak into the policy's observation.
        obs = build_obs(s.roll, s.roll_rate, yaw_err, data.qvel[5],
                        data.qpos[self._sj] - self._flick_base,
                        s.v_lon, s.v_lat, e_lat,
                        min(tau / pol.horizon, 1.0))
        steer_rate, hub, diff = pol.action(obs)
        self._flick_steer += steer_rate * self.dt
        if pol.act_dim == 2:                 # feedforward policy: crawl balance
            diff = float(-self._K0[0] @ np.array(
                [e_lat, s.roll, 0.0, 0.0, s.v_lat, s.roll_rate, 0.0, 0.0]))
        a, b = mix(hub / self.r_wheel, diff)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = clamp_extended(self._flick_steer)

        # Hand back to the balance controller as soon as the bike is roughly
        # turned around AND upright — then the balance controller does the final
        # settling. In training the episode ENDED at success, so the policy has
        # no learned post-success behavior; querying it past the turn makes it
        # flail (drive off, fall). We hand off *looser* than the training
        # success (which needs the policy to fully stop): the balance controller
        # only needs the bike near the target heading and upright to catch it.
        # The ~90 deg midpoint is far from the target, so this never fires early.
        # A timeout at the training horizon is the safety net.
        psi_target = self._flick_yaw0 + self._flick_dir * np.pi
        near_done = (abs(yaw_err) < np.deg2rad(20)      # heading ~ at target
                     and abs(s.roll) < np.deg2rad(15)    # upright enough to catch
                     and abs(data.qvel[5]) < 2.0)        # not still spinning fast
        if near_done or tau > pol.horizon:
            self.steer_frame.sync(data.qpos[self._sj])
            self.command_line(data, heading=psi_target)
        return u

    def _ball_addr_lookup(self, model):
        """Lazily resolve the ball freejoint's qpos/qvel addresses (hockey model).
        Returns None if the model has no ball (then the shot degrades to balance)."""
        if self._ball_addr is not None:
            return self._ball_addr
        try:
            jid = int(model.body("ball").jntadr[0])
        except Exception:
            self._ball_addr = (None, None)
            return self._ball_addr
        self._ball_addr = (int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]))
        return self._ball_addr

    def _ball_compute(self, model, data, s) -> np.ndarray:
        """Replay the ball-shot RL policy (numpy MLPPolicy): build the shared
        ball observation, query the policy, integrate steer, mix, and hand back to
        balance once the shot is done. Mirrors _flick_policy_compute."""
        from .ball_spec import build_obs
        pol = self._ball
        tau = data.time - self._ball_t0
        qadr, vadr = self._ball_addr_lookup(model)
        c, sn = np.cos(self._psi), np.sin(self._psi)
        if qadr is not None:
            rel = data.qpos[qadr:qadr + 2] - data.qpos[:2]
            bvel = data.qvel[vadr:vadr + 2]
            bdx = c * rel[0] + sn * rel[1]
            bdy = -sn * rel[0] + c * rel[1]
            bvx = c * bvel[0] + sn * bvel[1]
            bvy = -sn * bvel[0] + c * bvel[1]
            present = 1.0
        else:
            bdx = bdy = bvx = bvy = 0.0
            present = 0.0
        # Heading is measured against the WORLD launch target, matching training
        # (ball_spec.build_obs). Using the start heading here instead would make
        # replay silently disagree with the policy's observation contract.
        heading = self._psi - getattr(pol, "launch_target", 0.0)
        m = -1.0 if self._ball_mirror else 1.0   # reflect lateral obs for ball-left
        obs = build_obs(m * s.roll, m * s.roll_rate, m * heading, m * data.qvel[5],
                        m * (data.qpos[self._sj] - self._ball_base),
                        s.v_lon, m * s.v_lat,
                        bdx, m * bdy, bvx, m * bvy, present,
                        min(tau / pol.horizon, 1.0))
        steer_rate, hub, diff = pol.action(obs)
        steer_rate, diff = m * steer_rate, m * diff   # reflect lateral action back
        self._ball_steer += steer_rate * self.dt
        if pol.act_dim == 2:                 # feedforward policy: crawl balance
            diff = float(-self._K0[0] @ np.array(
                [s.e_lat, s.roll, 0.0, 0.0, s.v_lat, s.roll_rate, 0.0, 0.0]))
        a, b = mix(hub / self.r_wheel, diff)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = clamp_extended(self._ball_steer)

        # Hand back to balance once the horizon elapses or the bike is nearly
        # stopped and upright (the policy has no learned post-success behavior).
        upright_stopped = (abs(s.roll) < np.deg2rad(15) and abs(s.v_lon) < 0.15
                           and abs(data.qvel[5]) < 1.0 and tau > 0.5)
        if tau > pol.horizon or upright_stopped:
            self.steer_frame.sync(data.qpos[self._sj])
            self.command_line(data, heading=self._psi)
        return u

    def _flick_compute(self, data, s) -> np.ndarray:
        """Replay the optimized two-arc flick: feedforward steer + hub from the
        trajectory, rear differential = roll/lateral crawl balance (the same
        law the optimizer's rollout used, so replay matches the optimization).
        Hands back to line mode on completion + settle."""
        fl = self._flick
        if getattr(fl, "kind", "trajectory") == "policy":
            return self._flick_policy_compute(data, s)
        tau = data.time - self._flick_t0
        # Feedforward is authored from a straight start; rebase it onto the
        # pi-multiple park the maneuver began at, so a second flick sweeps
        # base -> base + pi continuously instead of snapping back through 0.
        if tau < fl.T:                       # replay the trajectory feedforward
            self._flick_steer = self._flick_base + self._flick_dir * fl.steer(tau)
            hub = fl.hub(tau)
        else:                                # settle: hold the front where the
            self._flick_steer = (self._flick_base
                                 + self._flick_dir * fl.steer(fl.T))  # ~180
            hub = 0.0                        # no unwind — see below.
        # crawl balance about the flick's start pose (bike-frame lateral), K0
        # roll/lateral response only — steer committed, yaw is the maneuver.
        sb = extract_state(data, self._flick_p0)
        d_bal = float(-self._K0[0] @ np.array([
            sb.e_lat, sb.roll, 0.0, 0.0, sb.v_lat, sb.roll_rate, 0.0, 0.0]))
        a, b = mix(hub / self.r_wheel, d_bal)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = clamp_extended(self._flick_steer)

        psi_target = self._flick_yaw0 + self._flick_dir * np.pi
        if (tau > fl.T and abs(self._psi - psi_target) < np.deg2rad(20)
                and abs(data.qvel[5]) < 0.4):
            # Hand off to line-keeping. The front is at ~180 deg; rather than
            # spinning the servo back (which drags the bike in yaw at
            # standstill, and looks like a snap), adopt the nearest pi
            # multiple of the *measured* angle as the steer origin — the
            # wheel is front-back symmetric so it is longitudinally straight
            # (see steer.py).
            self.steer_frame.sync(data.qpos[self._sj])
            self.command_line(data, heading=psi_target)
        return u

    def _pivot_rl_compute(self, data, s) -> np.ndarray:
        """Replay the pivot policy. The observation must EXACTLY mirror
        pivot_env._obs: same hold/line frames, same v_ref ramp (v0 measured
        at command time -> commanded v_end over the horizon), steer passed
        base-relative. Training is single-direction (+180); direction=-1
        replays via the ball-style m-reflection (odd quantities: roll,
        roll_rate, yaw_rate, steer, hold_raw, v_lat, e_line, and the
        steer_rate/diff actions; wheel_heading is odd, so exact)."""
        from .pivot_spec import build_obs
        pol = self._pivot
        m = float(self._pivot_dir)
        tau = data.time - self._pivot_t0
        phase = min(tau / pol.horizon, 1.0)
        delta = data.qpos[self._sj] - self._pivot_base
        hold_raw = ((self._psi - self._pivot_yaw0)
                    + wheel_heading(float(delta), self.rake)
                    + (self._pivot_yaw0 - self._pivot_theta0))
        yaw_err = pol.target - m * (self._psi - self._pivot_yaw0)
        pf = data.qpos[:2] + self.wheelbase * np.array(
            [np.cos(s.yaw), np.sin(s.yaw)])
        vf = data.qvel[:2] + self.wheelbase * data.qvel[5] * np.array(
            [-np.sin(s.yaw), np.cos(s.yaw)])
        e_line = float(self._pivot_n0 @ (pf - self._pivot_pf0))
        v_along = float(self._pivot_u0 @ vf)
        v_ref = self._pivot_v0 + (self._pivot_vend - self._pivot_v0) * phase
        obs = build_obs(m * s.roll, m * s.roll_rate, yaw_err, m * data.qvel[5],
                        m * delta, m * hold_raw, s.v_lon, m * s.v_lat,
                        v_along - v_ref, self._pivot_vend, m * e_line, phase)
        steer_rate, hub, diff = pol.action(obs)
        steer_rate, diff = m * steer_rate, m * diff   # reflect back
        self._pivot_steer += steer_rate * self.dt
        if pol.act_dim == 2:                 # feedforward policy: crawl balance
            diff = float(-self._K0[0] @ np.array(
                [s.e_lat, s.roll, 0.0, 0.0, s.v_lat, s.roll_rate, 0.0, 0.0]))
        a, b = mix(hub / self.r_wheel, diff)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = clamp_extended(self._pivot_steer)

        # Hand back once roughly turned + upright (the policy has no learned
        # post-success behavior); the horizon is the safety net. The wheel
        # parks at ~ -pi relative to the chassis -> sync the steer frame.
        psi_target = self._pivot_yaw0 + self._pivot_dir * np.pi
        near_done = (abs(yaw_err) < np.deg2rad(20) and abs(s.roll) < np.deg2rad(15)
                     and abs(data.qvel[5]) < 2.0 and tau > 0.5)
        if near_done or tau > pol.horizon:
            self.steer_frame.sync(data.qpos[self._sj])
            self.command_line(data, heading=psi_target)
            # Same global glide, reversed heading: the bike now travels
            # BACKWARD along its own axis. Seed the profile from measured
            # speed so set_speed doesn't slam from a stale reference.
            self.profile.v_ref = s.v_lon
            self.set_speed(-self._pivot_vend)
        return u

    def _compute(self, model, data):
        s = extract_state(data, self._ref_pos)
        dpsi = np.arctan2(np.sin(s.yaw - self._psi_raw_prev),
                          np.cos(s.yaw - self._psi_raw_prev))
        self._psi += dpsi
        self._psi_raw_prev = s.yaw

        if self.mode == "flip":
            return self._flip_compute(data, s)
        if self.mode == "flick":
            return self._flick_compute(data, s)
        if self.mode == "ball":
            return self._ball_compute(model, data, s)
        if self.mode == "pivot_rl":
            return self._pivot_rl_compute(data, s)
        if self.mode == "general":
            return self._general_compute(data, s)

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
            # Wrapped on purpose (unlike arc mode): multi-turn heading
            # commands reach the bike through the slewed reference, so e_psi
            # stays small in normal operation, and wrapping makes a bike far
            # off its line heading recover the short way around.
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
        # Small-signal steer about the frame's pi-multiple origin (steer.py):
        # after a flick the front parks at ~180 deg (front-back-symmetric
        # wheel -> longitudinally "straight") and the frame treats that as
        # zero rather than spinning the servo back, which would drag the bike
        # in yaw at standstill.
        steer_meas = self.steer_frame.measured(data.qpos[sj])
        x = np.array([
            e_lat, s.roll - roll_ref, e_psi, steer_meas - steer_ff,
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
        u[self.aid["steer"]] = self.steer_frame.command(steer)
        return u
