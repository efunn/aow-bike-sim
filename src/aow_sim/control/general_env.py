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
from .general_spec import (ACT_DIM, OBS_DIM, ActionBounds, build_obs,
                           command_to_body, scale_action, wrap_pi)
from .linearize import settle_upright

_PARKED = np.array([100.0, 100.0])   # off-scene ball position (as ball_env)


def _load_rl_config(path=None) -> dict:
    import yaml
    from pathlib import Path
    p = path or Path(__file__).resolve().parents[3] / "config" / "rl_general.yaml"
    with open(p) as f:
        return yaml.safe_load(f)


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
        self._mass0 = self.model.body_mass.copy()
        self._friction0 = self.model.geom_friction.copy()
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
        self.fall = np.deg2rad(self.rw["fall_roll_deg"])
        self.sigma_v = float(self.rw["sigma_v"])
        self.sigma_psi = np.deg2rad(self.rw["sigma_psi_deg"])

        cur = self.cfg["curriculum"]
        self.cur_on = bool(cur["enabled"])
        self.diff_start = float(cur["start"])
        self.diff_step = float(cur["step"])
        self.diff_thresh = float(cur["advance_score"])
        self._diff = self.diff_start if self.cur_on else 1.0

        act_dim = ACT_DIM if self.full else 2
        self.action_space = spaces.Box(-1.0, 1.0, (act_dim,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
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
        r, rng = self.rand, self._np_random
        if not r["enabled"]:
            self.model.body_mass[:] = self._mass0
            self.model.geom_friction[:] = self._friction0
            return
        self.model.body_mass[:] = self._mass0 * (
            1 + rng.uniform(-r["mass_frac"], r["mass_frac"], self._mass0.shape))
        self.model.geom_friction[:] = self._friction0
        self.model.geom_friction[:, 0] *= (
            1 + rng.uniform(-r["friction_frac"], r["friction_frac"]))

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

    def _obs(self):
        s = extract_state(self.data, self._p0)
        v_cl, v_ct, psi_err = command_to_body(self._v_cmd_w, self._psi_cmd,
                                              self._psi)
        obs = build_obs(s.roll, s.roll_rate, self.data.qvel[5],
                        float(self.data.qpos[self._sj]),
                        float(self.data.qvel[self._sd]),
                        s.v_lon, s.v_lat, v_cl, v_ct, psi_err, self._prev_a)
        return obs, s, v_cl, v_ct, psi_err

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

        obs, s, v_cl, v_ct, psi_err = self._obs()
        rw = self.rw
        v_err2 = (v_cl - s.v_lon) ** 2 + (v_ct - s.v_lat) ** 2
        r_vel = np.exp(-v_err2 / self.sigma_v ** 2)
        r_head = np.exp(-(psi_err / self.sigma_psi) ** 2)
        reward = (rw["w_vel"] * r_vel
                  + rw["w_head"] * r_head
                  + rw["w_alive"]
                  - rw["w_upright"] * s.roll ** 2
                  - rw["w_effort"] * float(action @ action)
                  - rw["w_smooth"] * float((action - self._prev_a)
                                           @ (action - self._prev_a)))
        self._prev_a = action
        self._track_sum += 0.5 * (r_vel + r_head)

        fell = abs(s.roll) > self.fall or not np.all(np.isfinite(self.data.qpos))
        terminated = False
        if fell:
            reward -= rw["penalty_fall"]
            terminated = True
        truncated = self._step >= self.max_steps

        # Live command: step-change resample (never a ramp).
        if not (terminated or truncated) and self._step >= self._next_resample:
            self._sample_command(self._np_random)

        track = self._track_sum / self._step
        if terminated or truncated:
            self._advance_curriculum(0.0 if fell else track)
        return obs, float(reward), terminated, truncated, {
            "track": float(track),
            "vel_err": float(np.sqrt(v_err2)),
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
