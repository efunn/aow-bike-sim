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

from ..build_model import build_model, load_params
from .balance import extract_state, mix
from .drive import DriveController
from .general_spec import (ACT_DIM, ActionBounds, build_obs, command_to_body,
                           obs_dim_for, obs_layout, rotate_to_body,
                           scale_action, vel_filter_alpha, vel_filter_step,
                           wrap_pi)
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
        self.model = build_model(self.p, variant="full", hockey=self.hockey)
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
        self.w_smooth = _per_channel(self.rw["w_smooth"],
                                     ACT_DIM if self.full else 2, "w_smooth")
        # Velocity window. Lives under `env:` rather than `reward:` because it
        # changes the OBSERVATION CONTRACT, not just a weight -- and `env:` is
        # where the fields that get exported into the move yaml live. Absent
        # or 0.0 reproduces the instantaneous reward bit for bit.
        self.vel_window_s = float(env.get("vel_window_s", 0.0))
        self._vbar_alpha = vel_filter_alpha(self.ctrl_dt, self.vel_window_s)
        self._settle_steps = int(round(self.vel_window_s / self.ctrl_dt))
        # Pitch. Observed as well as penalized: charging w_pitch against a
        # state the policy cannot see is the defect general_spec documents for
        # prev_action. Unlike world position, pitch and pitch rate come
        # straight off the AHRS on hardware (hw/state.set_orientation), so
        # observing them costs nothing in transfer.
        self.obs_pitch = bool(env.get("obs_pitch", False))

        cur = self.cfg["curriculum"]
        self.cur_on = bool(cur["enabled"])
        self.diff_start = float(cur["start"])
        self.diff_step = float(cur["step"])
        self.diff_thresh = float(cur["advance_score"])
        self._diff = self.diff_start if self.cur_on else 1.0

        act_dim = ACT_DIM if self.full else 2
        self.action_space = spaces.Box(-1.0, 1.0, (act_dim,), np.float32)
        self.obs_dim = obs_dim_for(self.vel_window_s, self.obs_pitch)
        self.obs_layout = obs_layout(self.vel_window_s, self.obs_pitch)
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.obs_dim,),
                                            np.float32)
        self._aid = {n: self.model.actuator(n).id
                     for n in ("drive_a", "drive_b", "steer")}
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
                        s.v_lon, s.v_lat, v_cl, v_ct, psi_err, self._prev_a,
                        v_bar=vb,
                        pitch=(s.pitch, s.pitch_rate) if self.obs_pitch else None)
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
        steer_rate, hub, diff = scale_action(action, self.bounds)
        self._steer += steer_rate * self.ctrl_dt
        if not self.full:                       # feedforward: crawl balance
            s0 = extract_state(self.data, self._p0)
            diff = float(-self._K0[0] @ np.array(
                [s0.e_lat, s0.roll, 0, 0, s0.v_lat, s0.roll_rate, 0, 0]))
        a, b = mix(hub / self._r_rear, diff)
        self.data.ctrl[self._aid["drive_a"]] = a
        self.data.ctrl[self._aid["drive_b"]] = b
        self.data.ctrl[self._aid["steer"]] = self._steer

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
        v_err2 = (v_cl - vb_lon) ** 2 + (v_ct - vb_lat) ** 2
        v_err2_inst = (v_cl - s.v_lon) ** 2 + (v_ct - s.v_lat) ** 2
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
            "vel_err": float(np.sqrt(v_err2_inst)),
            "vel_err_win": float(np.sqrt(v_err2)),
            "v_bar_lon": float(vb_lon), "v_bar_lat": float(vb_lat),
            "pitch_deg": float(np.degrees(s.pitch)),
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
