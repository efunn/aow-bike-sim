"""Gymnasium environment for the GENERAL command-conditioned policy
(training only).

One always-on controller tracking a live (velocity vector, heading) command —
see general_spec for the observation contract and why the command is a vector
rather than (course, speed). Three structural differences from the move envs,
all forced by "always-on":

  * The command is RESAMPLED MID-EPISODE as a step change (no ramp), every
    U(resample_s) seconds, because that is what a real operator does: stop,
    reverse, snap to a new heading. The policy must recover from a
    discontinuous setpoint, not track a smooth reference.
  * Episodes terminate ONLY on a fall. There is no "success" to stop at.
  * Reward is bounded-positive tracking plus an alive bonus, not the moves'
    penalty-dominated form. Over a 15 s episode a penalty-only reward makes
    falling early strictly rational; exp(-err^2) tracking plus w_alive
    removes that incentive while staying bounded.

Difficulty is a single scalar `_diff` in [0, 1] scaling the command sampling
ranges (speed, lateral fraction, heading step). It advances when the running
tracking score clears a threshold — the repo's first curriculum.

Optional hockey scene: on a fraction of episodes a ball is parked nearby with
NO ball-specific reward, purely so the policy is robust to striking one. The
shot itself is whatever command sequence a higher level issues.
"""

from __future__ import annotations

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from ..build_model import build_model, load_params, reset_actuator_state
from .balance import extract_state, mix
from .drive import DriveController
from .general_spec import (ACT_DIM, ActionBounds, act_dim_for, build_obs,
                           command_to_body, obs_dim_for, obs_layout,
                           rotate_to_body, scale_action, vel_filter_alpha,
                           vel_filter_step, wrap_pi)
from .linearize import settle_upright
from .randomize import DomainRandomizer

_PARKED = np.array([100.0, 100.0])   # off-scene ball position (as ball_env)


def _load_rl_config(path=None) -> dict:
    import yaml
    from pathlib import Path
    p = path or Path(__file__).resolve().parents[3] / "config" / "rl_general.yaml"
    with open(p) as f:
        return yaml.safe_load(f)


def _per_channel(value, act_dim: int, name: str) -> np.ndarray:
    """A reward weight that may be one number or one number per action channel.

    A scalar broadcasts, so every config written before this existed keeps its
    exact reward -- and setting a per-channel list to a single repeated value
    reproduces the scalar bit for bit. The list form exists because the three
    channels are not interchangeable: under a UNIFORM w_smooth of 0.05 the
    steer and hub channels came off their bounds (mean squared step change
    1.52 -> 0.41 and 0.51 -> 0.18) while `diff` did not move at all
    (1.27 -> 1.23), which is the evidence that the differential is doing
    balance work the other two are not. Pricing it separately is the only way
    to ask how much of that chatter is load-bearing.

    A wrong-length list is an error rather than a silent broadcast: a 2-list
    against 3 channels would quietly leave `diff` unpriced, which is exactly
    the experiment being run and must not happen by accident.
    """
    a = np.atleast_1d(np.asarray(value, float))
    if a.size == 1:
        return np.full(act_dim, float(a[0]))
    if a.size < act_dim:
        raise ValueError(f"{name} has {a.size} entries for {act_dim} action "
                         f"channels; give one value or one per channel")
    return a[:act_dim].copy()


class GeneralEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, params=None, rl_cfg=None, seed=None):
        super().__init__()
        self.p = params or load_params()
        self.cfg = rl_cfg or _load_rl_config()
        env = self.cfg["env"]
        self.hockey = bool(env.get("ball_prob", 0.0) > 0.0)
        # Wings are GATED. Building them is not a small addition: it pulls in
        # the righting shell, adds ~143 g to a ~1016 g bike, raises the CoM
        # ~1.9 mm, and flips the chassis lumps to COLLIDABLE -- a materially
        # different robot, whose numbers never compare to the wingless arms.
        # Off by default so every existing policy and comparison is untouched.
        self.obs_wings = bool(env.get("obs_wings", False))
        self.act_wings = bool(env.get("act_wings", False))
        self.wings = self.obs_wings or self.act_wings
        # The co-rotating pair (build_model _add_swing_wings). An ALTERNATIVE
        # to `wings`, never an addition -- build_spec refuses both, and so does
        # this, loudly, because a config with both set silently builds only one
        # and the run answers nothing about either.
        self.obs_swing = bool(env.get("obs_swing", False))
        self.act_swing = bool(env.get("act_swing", False))
        self.swing = self.obs_swing or self.act_swing
        if self.wings and self.swing:
            raise ValueError("obs/act_wings and obs/act_swing are alternative "
                             "mechanisms -- set one, not both")
        self.model = build_model(self.p, variant="full", hockey=self.hockey,
                                 righting=self.wings or self.swing,
                                 wings=self.wings, swing=self.swing)
        self._eq = settle_upright(self.model).qpos.copy()
        self.data = mujoco.MjData(self.model)
        # Crawl-balance fallback gain from the ball-free model, as ball_env.
        self._K0 = DriveController(
            self.p, build_model(self.p, variant="full"))._K0

        self.full = env["action_space"] == "full"
        self.bounds = ActionBounds(**env["action_bounds"])
        self.ctrl_dt = 1.0 / env["control_rate_hz"]
        self.substeps = max(1, round(self.ctrl_dt / self.model.opt.timestep))
        self.max_steps = int(env["max_episode_s"] / self.ctrl_dt)
        self.v_max = float(env["v_max"])
        self.v_lat_frac = float(env["v_lat_frac"])
        self.p_v_zero = float(env["p_v_zero"])
        self.resample_s = tuple(env["resample_s"])
        self.ball_prob = float(env.get("ball_prob", 0.0))
        self.ball_radius = float(env.get("ball_place_radius", 0.5))
        self.rw = self.cfg["reward"]
        self.rand = self.cfg["randomization"]
        self._rand = DomainRandomizer(self.model, self.rand)
        self.fall = np.deg2rad(self.rw["fall_roll_deg"])
        self.sigma_v = float(self.rw["sigma_v"])
        self.sigma_psi = np.deg2rad(self.rw["sigma_psi_deg"])
        self.w_smooth = _per_channel(
            self.rw["w_smooth"],
            act_dim_for(bool(env.get("act_wings", False)),
                        bool(env.get("act_swing", False))) if
            env["action_space"] == "full" else 2, "w_smooth")
        # Velocity window. Lives under `env:` rather than `reward:` because it
        # changes the OBSERVATION CONTRACT, not just a weight -- and `env:` is
        # where the fields that get exported into the move yaml live. Absent
        # or 0.0 reproduces the instantaneous reward bit for bit.
        self.vel_window_s = float(env.get("vel_window_s", 0.0))
        # See general_spec.build_obs. Pairs with v_lat_frac: 0.0.
        self.zero_lat = bool(env.get("obs_zero_lat", False))
        # THE ONBOARD ESTIMATE, IN THE TRAINING LOOP. The policy sees what
        # hw/odometry.py reconstructs from the simulated encoders and AHRS --
        # what the Pi will actually hand it -- instead of MuJoCo truth.
        #
        # This is the version of the question that zeroing could not answer. A
        # constant 0 carries NO information, and measured 2026-08-26 that is
        # not enough: v_lat is 95% predictable from the other entries near a
        # competent policy's operating point, but not during exploration, so
        # the policy cannot infer what it needs before it can balance. The
        # estimate is wrong but CORRELATED (0.88-0.96 by regime), which is a
        # different thing entirely -- and training on it lets the policy learn
        # this estimator's real error rather than meet it at deployment.
        #
        # Physics is untouched: only the OBSERVATION is replaced.
        self.obs_odometry = bool(env.get("obs_odometry", False))
        self._odo = None
        if self.obs_odometry:
            from ..sim_odometry import SimOdometry
            self._odo = SimOdometry(self.model, params,
                                    mode=env.get("odometry_mode", "front"))
        self._vbar_alpha = vel_filter_alpha(self.ctrl_dt, self.vel_window_s)
        self._settle_steps = int(round(self.vel_window_s / self.ctrl_dt))
        # Pitch. Observed as well as penalized: charging w_pitch against a
        # state the policy cannot see is the defect general_spec documents for
        # prev_action. Unlike world position, pitch and pitch rate come
        # straight off the AHRS on hardware (hw/state.set_orientation), so
        # observing them costs nothing in transfer.
        # Hub magnitude at LOW COMMANDED SPEED. Holding station needs no net
        # wheel rotation -- the LQR does it with 0.0 hub rpm and 0.00 m of rim
        # travel over 15 s, on [differential, steer] alone -- but every RL
        # policy saws the wheel anyway: 79-131 hub rpm and 6.4-10.5 m of rim
        # past the contact patch while the bike goes nowhere.
        #
        # It is not the task. A policy trained on hold ALONE still ran 88 rpm
        # and 7.07 m. It is that nothing prices the channel: `w_effort` is
        # 0.001 and `w_smooth` prices CHANGE, not magnitude, so a large steady
        # hub command is free. Pinning hub_max to 0 removes it entirely (0.6
        # rpm, 0.05 m) and HALVES hold drift, which is what says the motion is
        # gratuitous rather than load-bearing.
        #
        # Priced rather than clamped, deliberately: a hard clamp would also
        # remove the channel during a disturbance recovery, where using the
        # wheel may be exactly right. This fades the charge out as the
        # commanded speed rises, so it bites only where the LQR says it should.
        # 0.0 by default, so every config predating it is bit-identical.
        self.w_hub_idle = float(self.rw.get("w_hub_idle", 0.0))
        # The speed at which a MEASURED motion fully cancels the hub charge.
        # Defaults to sigma_v, which is what arm 5 used, so an absent key
        # reproduces general_rl_glide_pitch_hub2 exactly. It is a knob because
        # the two arms bracket it and neither end is right:
        #   v_max 1.2  (arm 4) -- too stingy. A 0.3 m/s disturbance left 75% of
        #                the charge standing, and the policy stopped reaching
        #                for the wheel to recover (mean |hub| 0.19 vs 0.53).
        #   sigma_v 0.35 (arm 5) -- too generous. Recovery came back and beat
        #                the baseline (7/8 at dv 0.40 vs 0/8), but hold-time
        #                wheel motion went most of the way back to baseline
        #                (rim travel 2.27 -> 6.76 m).
        self.hub_idle_v_scale = float(
            self.rw.get("hub_idle_v_scale", self.sigma_v))
        self.obs_pitch = bool(env.get("obs_pitch", False))
        # Cap on the wing command. 90 deg keeps the deployed foot 11 mm clear
        # of the floor with the bike upright, while the full 105 deg stroke
        # puts it 15 mm BELOW and jacks the bike off its wheels -- that stroke
        # belongs to control/righting.py, not to the policy.
        self.wing_max = np.deg2rad(float(env.get("wing_max_deg", 90.0)))
        # Curriculum gate on the wing RANGE, as [open_lo, open_hi] in
        # difficulty. Below open_lo the cap is 0 -- the wings are pinned stowed
        # and the bike must balance on two wheels; above open_hi it is the full
        # wing_max. Absent => open from the start, which is what the first two
        # wings runs did.
        self.wing_open = tuple(env.get("wing_open", (0.0, 0.0)))

        cur = self.cfg["curriculum"]
        self.cur_on = bool(cur["enabled"])
        self.diff_start = float(cur["start"])
        self.diff_step = float(cur["step"])
        self.diff_thresh = float(cur["advance_score"])
        self._diff = self.diff_start if self.cur_on else 1.0

        act_dim = act_dim_for(self.act_wings, self.act_swing) if self.full else 2
        self.action_space = spaces.Box(-1.0, 1.0, (act_dim,), np.float32)
        self.obs_dim = obs_dim_for(self.vel_window_s, self.obs_pitch,
                                   self.obs_wings, self.obs_swing)
        self.obs_layout = obs_layout(self.vel_window_s, self.obs_pitch,
                                     self.obs_wings, self.obs_swing)
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.obs_dim,),
                                            np.float32)
        self._aid = {n: self.model.actuator(n).id
                     for n in ("drive_a", "drive_b", "steer")}
        if self.wings or self.swing:
            # ONE code path, two mechanisms: the actuator and joint names are
            # the only thing that differs, and the signed clip below is the
            # only behavioural difference.
            act = "swing" if self.swing else "wings"
            jnt = "swing_right_joint" if self.swing else "wing_right_joint"
            self._aid["wings"] = self.model.actuator(act).id
            self._wj = self.model.joint(jnt).qposadr[0]
            self._wd = self.model.joint(jnt).dofadr[0]
        self._sj = self.model.joint("steer_joint").qposadr[0]
        self._sd = self.model.joint("steer_joint").dofadr[0]
        self._chassis = self.model.body("chassis").id
        self._r_rear = self.p["omni_wheel"]["outer_radius"]
        if self.hockey:
            bjid = int(self.model.body("ball").jntadr[0])
            self._ball_q = int(self.model.jnt_qposadr[bjid])
            self._ball_v = int(self.model.jnt_dofadr[bjid])
            self._ball_r = self.p["hockey"]["ball"]["radius"]
        self._np_random, _ = gym.utils.seeding.np_random(seed)

    # -- helpers -----------------------------------------------------------

    def _apply_randomization(self):
        self._rand.apply(self._np_random)

    def _idle_frac(self, v_meas: float = 0.0) -> float:
        """1 when the bike is asked to stand still AND is standing still,
        0 at v_max, linear between. Faded on max(commanded, measured) speed.

        CORRECTION. This first faded on the COMMAND alone, reasoning that the
        charge should be predictable from something in the observation and
        that a measured-speed fade would let the policy duck the penalty by
        moving. Both halves were wrong in the way that mattered:

        - A hold command means full charge, and a disturbance ARRIVES during a
          hold command. So the first version priced the wheel hardest at
          exactly the moment recovery might need it. Measured on
          general_rl_glide_pitch_hub: hold-time wheel motion fell 70% (good),
          but it also stopped reaching for the channel under a lateral kick
          (mean |hub| 0.19 vs the baseline's 0.59) and lost the recoveries
          that go with it.
        - The "duck it by moving" loophole is already closed, by arithmetic,
          by `w_vel`. Drifting at 0.3 m/s under a hold command puts r_vel at
          exp(-0.3^2/0.35^2) = 0.48, i.e. it gives up ~0.78/step of w_vel to
          dodge at most 0.40/step of hub charge. Moving to escape this term is
          a losing trade whichever way the policy plays it.

        So: while the bike is genuinely still, the wheel is expensive; the
        instant it is moving -- commanded OR not -- the charge fades and the
        channel is free for recovery.

        TWO SCALES, because the two speeds mean different things. The COMMAND
        is faded against `v_max`, the span of the channel it is drawn from.
        The MEASURED speed is faded against `sigma_v` -- the reward's own
        notion of "a velocity error that matters" -- because "idle" has to
        mean actually stationary, and against v_max a 0.3 m/s disturbance
        still leaves 75% of the charge standing, which is no relief at the
        moment relief is the whole point. Against sigma_v it leaves 14%.
        Whichever gives MORE relief wins.
        """
        if self.v_max <= 0.0:
            return 1.0
        by_cmd = 1.0 - float(np.linalg.norm(self._v_cmd_w)) / self.v_max
        by_meas = 1.0 - abs(float(v_meas)) / max(self.hub_idle_v_scale, 1e-9)
        return float(np.clip(min(by_cmd, by_meas), 0.0, 1.0))

    def _sample_command(self, rng, first=False):
        """Draw a fresh (world velocity, heading) command as a STEP change.

        Scaled by the curriculum difficulty: at _diff = 0 the commands are
        gentle (small speed, small heading step); at 1 they span the full
        envelope including reverse, lateral crab, and +-180 deg snaps."""
        d = self._diff
        v_lim = self.v_max * (0.25 + 0.75 * d)
        if rng.random() < self.p_v_zero:
            v_lon_w = 0.0
        else:
            v_lon_w = float(rng.uniform(-v_lim, v_lim))
        # Lateral command only opens up with difficulty (crab / pivot-glide).
        v_lat_w = float(rng.uniform(-1, 1) * self.v_lat_frac * v_lim * d)
        # Heading: a step relative to the CURRENT heading, growing to +-pi.
        span = np.deg2rad(30.0) + (np.pi - np.deg2rad(30.0)) * d
        step = 0.0 if first else float(rng.uniform(-span, span))
        psi_cmd = self._psi + step
        # The velocity command is expressed in the world frame, anchored on
        # the heading the operator is asking for (drive "forward" = along the
        # commanded heading, not the current one).
        c, s = np.cos(psi_cmd), np.sin(psi_cmd)
        self._v_cmd_w = np.array([c * v_lon_w - s * v_lat_w,
                                  s * v_lon_w + c * v_lat_w])
        self._psi_cmd = psi_cmd
        self._next_resample = self._step + max(
            1, round(float(rng.uniform(*self.resample_s)) / self.ctrl_dt))
        # Marks the start of the velocity filter's flush window (see step()).
        # NOTE the filter state itself is deliberately NOT reset here: it is a
        # property of the bike's motion, not of the command. Resetting it
        # would be a hidden state jump the policy cannot see in its own
        # observation, and there is no "resample" event on hardware at all
        # (the operator just moves a stick), so drive.py could never match it.
        self._resample_step = self._step

    def _place_ball(self, rng):
        if not self.hockey:
            return
        q = self._ball_q
        if rng.random() < self.ball_prob:
            ang = rng.uniform(-np.pi, np.pi)
            rad = rng.uniform(0.15, self.ball_radius)
            xy = self.data.qpos[:2] + rad * np.array([np.cos(ang), np.sin(ang)])
        else:
            xy = _PARKED
        self.data.qpos[q:q + 2] = xy
        self.data.qpos[q + 2] = self._ball_r
        self.data.qpos[q + 3:q + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[self._ball_v:self._ball_v + 6] = 0.0

    def _wing_cap(self) -> float:
        """Wing range allowed at the current difficulty [rad].

        REVERSED from the first attempt. Runs 1 and 2 had the wings available
        from step 0 and tried to price them away later; both ended unable to
        balance without them (20/20 falls when forced stowed), because a
        policy that learns wing-dependent balance first has to UNLEARN it, and
        a penalty just buys the cheapest escape -- run 2 deployed less but let
        roll reach 43 deg and got caught harder (24% of weight on the feet vs
        9%).

        Opening the range late instead means wingless balance is learned while
        it is the only option, and the wings arrive as an extra capability
        rather than a crutch to be taken away. A cap also cannot be traded
        against tracking reward the way a penalty can.
        """
        lo, hi = self.wing_open
        if hi <= lo:
            return self.wing_max                  # no gate: open from the start
        f = (self._diff - lo) / (hi - lo)
        return self.wing_max * float(np.clip(f, 0.0, 1.0))

    def _w_wing_now(self) -> float:
        """Wing penalty at the current curriculum difficulty.

        Linear from `w_wing` at difficulty 0 to `w_wing_late` at 1. When
        `w_wing_late` is absent it equals `w_wing`, so every config written
        before this ramp existed keeps a flat weight and reproduces exactly.

        `w_wing_ramp` gives the difficulty range the ramp spans, which run 3
        showed is necessary rather than decorative. There the gate opened over
        _diff 0.5-0.8 while this weight ramped on _diff DIRECTLY, so it was
        already ~0.5 when the wings first became usable: they became available
        exactly as they became expensive, and the policy never touched them
        (max 9.5 deg at 700k, ~0 for the rest of training). Keying the ramp to
        a later range leaves a window where the wings are both usable and
        cheap, which is the only way exploring them can pay.
        """
        lo = float(self.rw.get("w_wing", 0.0))
        hi = float(self.rw.get("w_wing_late", lo))
        r = self.rw.get("w_wing_ramp")
        if r and float(r[1]) > float(r[0]):
            f = (self._diff - float(r[0])) / (float(r[1]) - float(r[0]))
            f = float(np.clip(f, 0.0, 1.0))
        else:
            f = self._diff          # keyed straight to difficulty (run 2/3)
        return lo + (hi - lo) * f

    def _advance_vel_filter(self):
        """One tick of the low-pass on measured WORLD velocity.

        Called explicitly from step() rather than from _obs(), which is called
        both at reset and once per step: burying the update in there works
        today but would silently run the filter at double rate the moment
        anyone calls _obs() twice in a tick.
        """
        self._v_bar_w = vel_filter_step(self._v_bar_w, self.data.qvel[:2],
                                        self._vbar_alpha)

    def _obs(self):
        s = extract_state(self.data, self._p0)
        # THE OBSERVATION gets the estimate; `s` stays TRUTH.
        #
        # Keeping them apart is the whole design. The reward is scored on `s`,
        # so it prices what the bike ACTUALLY did; the policy is shown only
        # what the bike can sense. Feed the estimate to both and the policy can
        # be rewarded for fooling its own estimator -- driving the estimate
        # toward the command while the bike does something else -- which is a
        # reward-hacking channel, not a control problem.
        o_lon, o_lat = s.v_lon, s.v_lat
        if self._odo is not None:
            # Stepped at the CONTROL rate, which is what runs on the Pi.
            o_lon, o_lat = self._odo.update(self.data, self.ctrl_dt)
        v_cl, v_ct, psi_err = command_to_body(self._v_cmd_w, self._psi_cmd,
                                              self._psi)
        vb = None
        if self.vel_window_s > 0.0:
            vb = rotate_to_body(self._v_bar_w[0], self._v_bar_w[1], self._psi)
        else:                          # alpha is exactly 1, so v_bar IS v
            vb_lon, vb_lat = s.v_lon, s.v_lat
        if vb is not None:
            vb_lon, vb_lat = vb
        obs = build_obs(s.roll, s.roll_rate, self.data.qvel[5],
                        float(self.data.qpos[self._sj]),
                        float(self.data.qvel[self._sd]),
                        o_lon, o_lat, v_cl, v_ct, psi_err, self._prev_a,
                        zero_lat=self.zero_lat, v_bar=vb,
                        pitch=(s.pitch, s.pitch_rate) if self.obs_pitch else None,
                        wings=((float(self.data.qpos[self._wj]),
                                float(self.data.qvel[self._wd]))
                               if (self.obs_wings or self.obs_swing) else None))
        return obs, s, v_cl, v_ct, psi_err, vb_lon, vb_lat

    # -- curriculum --------------------------------------------------------

    def set_difficulty(self, d: float) -> None:
        self._diff = float(np.clip(d, 0.0, 1.0))

    def _advance_curriculum(self, ep_score: float) -> None:
        if self.cur_on and ep_score > self.diff_thresh:
            self._diff = min(1.0, self._diff + self.diff_step)

    # -- gym API -----------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self._np_random, _ = gym.utils.seeding.np_random(seed)
        rng = self._np_random
        self._apply_randomization()
        self.data.qpos[:] = self._eq
        self.data.qvel[:] = 0.0
        reset_actuator_state(self.model, self.data)
        if self._odo is not None:
            # The estimator INTEGRATES. An episode that inherits a warmed-up
            # filter is not the episode the bike will fly.
            self._odo.reset(self.model, self.p)
        r = self.rand
        if r["enabled"]:
            roll = rng.uniform(-1, 1) * np.deg2rad(r["init_roll_deg"])
            yaw = rng.uniform(-1, 1) * np.deg2rad(r["init_yaw_deg"])
            q = np.zeros(4)
            mujoco.mju_axisAngle2Quat(q, np.array([1.0, 0, 0]), roll)
            qy = np.zeros(4)
            mujoco.mju_axisAngle2Quat(qy, np.array([0, 0, 1.0]), yaw)
            quat = np.zeros(4)
            mujoco.mju_mulQuat(quat, qy, q)
            self.data.qpos[3:7] = quat
            self.data.qpos[:2] += rng.uniform(-1, 1, 2) * r["init_pos_m"]
            self.data.qvel[:6] += rng.uniform(-1, 1, 6) * r["init_vel"]
        else:
            a = np.deg2rad(0.5)
            self.data.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
        mujoco.mj_forward(self.model, self.data)
        self._p0 = self.data.qpos[:2].copy()
        self._psi = extract_state(self.data, self._p0).yaw
        self._raw_prev = self._psi
        self._place_ball(rng)

        self._steer = float(self.data.qpos[self._sj])
        self._prev_a = np.zeros(self.action_space.shape[0])
        self._step = 0
        self._track_sum = 0.0
        self._track_n = 0
        self._resample_step = 0
        # Seed from the MEASURED velocity, not zeros: engage_general has to do
        # the same ("hold what you are doing now"), and the two paths must
        # agree or a policy sees a startup transient in replay it never saw in
        # training.
        self._v_bar_w = self.data.qvel[:2].copy()
        # Wings start STOWED, and the integrator is seeded from the model so
        # replay (engage_general) can seed itself the same way.
        self._wing = (float(self.data.qpos[self._wj])
                      if (self.wings or self.swing) else 0.0)
        opts = options or {}
        if "difficulty" in opts:
            self.set_difficulty(opts["difficulty"])
        self._sample_command(rng, first=True)
        # Deterministic-eval override: a fixed command held for the episode.
        if "v_cmd" in opts or "psi_cmd_rel" in opts:
            v = np.asarray(opts.get("v_cmd", (0.0, 0.0)), float)
            self._psi_cmd = self._psi + float(opts.get("psi_cmd_rel", 0.0))
            c, s = np.cos(self._psi_cmd), np.sin(self._psi_cmd)
            self._v_cmd_w = np.array([c * v[0] - s * v[1],
                                      s * v[0] + c * v[1]])
            self._next_resample = 10 ** 9      # hold for the whole episode
        obs, *_ = self._obs()
        return obs, {}

    def step(self, action):
        action = np.asarray(action, np.float32)
        scaled = scale_action(action, self.bounds)
        steer_rate, hub, diff = scaled[0], scaled[1], scaled[2]
        self._steer += steer_rate * self.ctrl_dt
        if self.act_wings or self.act_swing:
            # A RATE integrated into a position target, exactly like steer,
            # because the wing servo is the same multi-turn XC330 in extended
            # position mode. UNLIKE steer it is clipped: the joint is limited,
            # and past ~96 deg the foot goes under the floor and jacks the
            # bike off its wheels instead of catching a fall.
            # THE CLIP IS THE MECHANISM DIFFERENCE. The mirrored pair only
            # ever deploys outward, so 0..cap is its whole reachable set. The
            # swing pair is signed: -cap..+cap, one wing down or the other,
            # and pinning its low end at 0 would leave a policy able to strike
            # and right on ONE SIDE ONLY -- which would not raise, it would
            # just quietly train half a bike.
            lo = -self._wing_cap() if self.swing else 0.0
            self._wing = float(np.clip(self._wing + scaled[3] * self.ctrl_dt,
                                       lo, self._wing_cap()))
        if not self.full:                       # feedforward: crawl balance
            s0 = extract_state(self.data, self._p0)
            diff = float(-self._K0[0] @ np.array(
                [s0.e_lat, s0.roll, 0, 0, s0.v_lat, s0.roll_rate, 0, 0]))
        a, b = mix(hub / self._r_rear, diff)
        self.data.ctrl[self._aid["drive_a"]] = a
        self.data.ctrl[self._aid["drive_b"]] = b
        self.data.ctrl[self._aid["steer"]] = self._steer
        if self.wings:
            # Held at 0 (stowed) when the policy does not own the channel.
            self.data.ctrl[self._aid["wings"]] = self._wing

        self.data.xfrc_applied[self._chassis, :] = 0.0
        if self.rand["enabled"] and self._np_random.random() < self.rand["disturb_prob"]:
            self.data.xfrc_applied[self._chassis, 1] = (
                self._np_random.uniform(-1, 1) * self.rand["disturb_force_N"])

        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        cur = extract_state(self.data, self._p0).yaw
        self._psi += np.arctan2(np.sin(cur - self._raw_prev),
                                np.cos(cur - self._raw_prev))
        self._raw_prev = cur
        self._step += 1

        self._advance_vel_filter()
        obs, s, v_cl, v_ct, psi_err, vb_lon, vb_lat = self._obs()
        rw = self.rw
        # Tracked against the WINDOWED velocity, so an oscillatory gait is
        # scored on its time average rather than punished at every instant of
        # the oscillation. At vel_window_s = 0, vb_* IS s.v_* and this is the
        # instantaneous form bit for bit.
        # LONGITUDINAL ONLY WHEN THE LATERAL AXIS IS NOT OBSERVED. `vb_lat` is
        # the bike's TRUE lateral velocity, so with obs_zero_lat set the policy
        # was being charged (0 - v_lat_true)^2 every step for motion it cannot
        # see -- the exact thing general_spec forbids for v_bar and prev_action:
        # a reward may not depend on state the policy cannot observe.
        #
        # It is not a small tax. The bike balances by moving laterally, so
        # v_lat_true is never near zero, and r_vel is half the curriculum score
        # (`0.5 * (r_vel + r_head)`). Charged, it holds ep_score under
        # diff_thresh and the curriculum crawls instead of advancing -- observed
        # on the first general_rl_nolat run, 2026-08-26.
        lat_scored = 0.0 if self.zero_lat else 1.0
        v_err2 = (v_cl - vb_lon) ** 2 + lat_scored * (v_ct - vb_lat) ** 2
        v_err2_inst = (v_cl - s.v_lon) ** 2 + lat_scored * (v_ct - s.v_lat) ** 2
        # The FULL 2-D error, always, whatever is being scored. `vel_err` is how
        # policies get compared across runs (analysis/chatter.py), so its
        # meaning must not change with a training flag -- otherwise a nolat
        # policy would look better than a crabbing one purely by measuring less.
        v_err2_full = (v_cl - s.v_lon) ** 2 + (v_ct - s.v_lat) ** 2
        da = action - self._prev_a
        r_vel = np.exp(-v_err2 / self.sigma_v ** 2)
        r_head = np.exp(-(psi_err / self.sigma_psi) ** 2)
        reward = (rw["w_vel"] * r_vel
                  + rw["w_head"] * r_head
                  + rw["w_alive"]
                  - rw["w_upright"] * s.roll ** 2
                  # Pitch was FREE until now: w_upright prices roll and
                  # nothing priced pitch, so a wheelie cost the policy
                  # nothing. Measured on general_rl_glide_og, the front wheel
                  # reaches 79 mm of clearance at 23 deg nose-up under a plain
                  # accelerate command (analysis/liftoff.py). Same quadratic
                  # form as w_upright; 0.0 by default, so every config that
                  # predates it is unchanged.
                  - rw.get("w_pitch", 0.0) * s.pitch ** 2
                  # Hub magnitude, faded out with commanded speed: full charge
                  # at a hold command, nothing at v_max. `action` is the
                  # NORMALISED action, as w_effort above uses it, so the term
                  # is in [0, 1] before weighting. v_max 0 (the hold-only
                  # diagnostic configs) means every command is a hold, so the
                  # fade is 1.0 throughout rather than a division by zero.
                  - (self.w_hub_idle * action[1] ** 2
                     * self._idle_frac(np.hypot(s.v_lon, s.v_lat))
                     if self.full and self.w_hub_idle else 0.0)
                  # Wing deployment, normalised by the allowed range so the
                  # term is in [0, 1], and RAMPED with the curriculum: cheap
                  # at difficulty 0 so the wings work as training wheels while
                  # the policy learns to track, expensive at difficulty 1 so
                  # it has to give them up.
                  #
                  # The late weight is set by arithmetic, not taste. Over a
                  # 750-step episode a 0.5 s save is ~25 steps of full deploy,
                  # and it buys back `penalty_fall` (50). So the late weight
                  # must satisfy 25*w < 50 (a genuine save still pays) and
                  # 750*w >> 50 (living on them does not): w in [0.2, 2.0].
                  # The first wings run used a flat 0.05, which costs 38 over
                  # a whole episode against a 50-point fall -- so riding the
                  # wings permanently was simply cheaper than balancing, and
                  # the policy did exactly that (forcing them stowed made it
                  # fall in 20 of 20 eval episodes).
                  - (self._w_wing_now()
                     * (self._wing / max(self.wing_max, 1e-9)) ** 2  # static cap: normalising by the scheduled one would divide by ~0
                     if (self.wings or self.swing) else 0.0)
                  - rw["w_effort"] * float(action @ action)
                  # Per-channel: w_smooth may be one number (broadcast, the
                  # historical behaviour) or one per action channel.
                  - float(da @ (self.w_smooth * da)))
        self._prev_a = action
        # Curriculum score, NOT the reward. After a command step change the
        # filter takes ~vel_window_s to flush, so r_vel is depressed for that
        # long no matter what the policy does. `track` gates
        # _advance_curriculum, and the lateral command envelope scales with
        # difficulty -- so charging the policy for the flush would stall the
        # curriculum, keep the crab command shut, and make the whole windowed
        # experiment quietly do nothing. Exclude the transient here only; the
        # reward above still pays it, because it is real tracking error.
        # settle_steps is 0 when the window is off, so this is exactly the
        # old `_track_sum / _step`.
        if self._step - self._resample_step >= self._settle_steps:
            self._track_sum += 0.5 * (r_vel + r_head)
            self._track_n += 1

        fell = abs(s.roll) > self.fall or not np.all(np.isfinite(self.data.qpos))
        terminated = False
        if fell:
            reward -= rw["penalty_fall"]
            terminated = True
        truncated = self._step >= self.max_steps

        # Live command: step-change resample (never a ramp).
        if not (terminated or truncated) and self._step >= self._next_resample:
            self._sample_command(self._np_random)

        track = self._track_sum / max(1, self._track_n)
        if terminated or truncated:
            self._advance_curriculum(0.0 if fell else track)
        return obs, float(reward), terminated, truncated, {
            "track": float(track),
            # `vel_err` stays the INSTANTANEOUS error so the metrics blocks in
            # existing moves/*.yaml remain comparable; `vel_err_win` is the
            # one the reward actually argues over. Identical when the window
            # is off.
            # Comparable across every policy: both axes, always.
            "vel_err": float(np.sqrt(v_err2_full)),
            # What this policy is actually optimised against. Equal to vel_err
            # unless obs_zero_lat is set, in which case it is longitudinal only.
            "vel_err_scored": float(np.sqrt(v_err2_inst)),
            "vel_err_win": float(np.sqrt(v_err2)),
            "v_bar_lon": float(vb_lon), "v_bar_lat": float(vb_lat),
            "pitch_deg": float(np.degrees(s.pitch)),
            "wing_deg": (float(np.degrees(self._wing))
                         if (self.wings or self.swing) else 0.0),
            "head_err_deg": float(np.degrees(abs(psi_err))),
            "difficulty": float(self._diff),
            "fell": bool(fell),
            # No task "success" for an always-on controller: surviving the
            # episode while tracking well is the whole objective.
            "success": bool(not fell), "is_success": bool(not fell)}


def make_env(params=None, rl_cfg=None, seed=None):
    """Thunk for SB3 vectorized env constructors."""
    def _thunk():
        return GeneralEnv(params, rl_cfg, seed)
    return _thunk
