"""Gymnasium environment for learning the two-arc 180 flick (training only).

Imports gymnasium (light; no torch). The observation/action contract and the
crawl-balance fallback are shared with replay via `flick_spec` and `_K0`, so a
policy trained here replays identically through `DriveController`. Reward is the
per-step analog of the trajopt cost: reward yaw progress toward 180, penalize
roll (upright), lateral (side-to-side) offset, and effort/jerk; terminate with a
bonus on a settled 180 and a penalty on falling. Domain randomization (initial
state + mass/friction + disturbance pushes) makes the learned policy robust —
the thing the open-loop trajectory can't be.
"""

from __future__ import annotations

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from ..build_model import build_model, load_params
from .balance import extract_state, mix
from .drive import DriveController
from .flick_spec import ACT_DIM, OBS_DIM, ActionBounds, build_obs, scale_action
from .linearize import settle_upright


def _load_rl_config(path=None) -> dict:
    import yaml
    from pathlib import Path
    p = path or Path(__file__).resolve().parents[3] / "config" / "rl_flick.yaml"
    with open(p) as f:
        return yaml.safe_load(f)


class FlickEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, params=None, rl_cfg=None, seed=None):
        super().__init__()
        self.p = params or load_params()
        self.cfg = rl_cfg or _load_rl_config()
        self.model = build_model(self.p, variant="full")
        self._eq = settle_upright(self.model).qpos.copy()
        self._mass0 = self.model.body_mass.copy()
        self._friction0 = self.model.geom_friction.copy()
        self.data = mujoco.MjData(self.model)
        self._K0 = DriveController(self.p, self.model)._K0

        env = self.cfg["env"]
        self.full = env["action_space"] == "full"
        self.bounds = ActionBounds(**env["action_bounds"])
        self.ctrl_dt = 1.0 / env["control_rate_hz"]
        self.substeps = max(1, round(self.ctrl_dt / self.model.opt.timestep))
        self.max_steps = int(env["max_episode_s"] / self.ctrl_dt)
        self.target = np.deg2rad(env["yaw_target_deg"])
        self.yaw_tol = np.deg2rad(env["success_yaw_tol_deg"])
        self.roll_ok = np.deg2rad(env["success_roll_deg"])
        self.rate_ok = env["success_rate"]
        self.rw = self.cfg["reward"]
        self.rand = self.cfg["randomization"]
        self.fall = np.deg2rad(self.rw["fall_roll_deg"])

        act_dim = ACT_DIM if self.full else 2
        self.action_space = spaces.Box(-1.0, 1.0, (act_dim,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
        self._aid = {n: self.model.actuator(n).id
                     for n in ("drive_a", "drive_b", "steer")}
        self._sj = self.model.joint("steer_joint").qposadr[0]
        self._np_random, _ = gym.utils.seeding.np_random(seed)

    # -- helpers -----------------------------------------------------------

    def _obs(self):
        s = extract_state(self.data, self._p0)
        d = self.data.qpos[:2] - self._p0
        e_lat = -np.sin(self._yaw0) * d[0] + np.cos(self._yaw0) * d[1]
        yaw_err = self.target - (self._psi - self._yaw0)
        return build_obs(s.roll, s.roll_rate, yaw_err, self.data.qvel[5],
                         self.data.qpos[self._sj], s.v_lon, s.v_lat, e_lat,
                         self._step / self.max_steps), s, e_lat, yaw_err

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
        # Single-direction policy (+180); the mirror is a future extension.
        self._p0 = self.data.qpos[:2].copy()
        self._yaw0 = self._psi = extract_state(self.data, self._p0).yaw
        self._raw_prev = self._yaw0
        self._steer = float(self.data.qpos[self._sj])
        self._prev_a = np.zeros(self.action_space.shape[0])
        self._step = 0
        obs, _, _, self._prev_yaw_err = self._obs()
        return obs, {}

    def step(self, action):
        action = np.asarray(action, np.float32)
        steer_rate, hub, diff = scale_action(action, self.bounds)
        self._steer += steer_rate * self.ctrl_dt
        if not self.full:                       # feedforward: crawl balance diff
            s = extract_state(self.data, self._p0)
            diff = float(-self._K0[0] @ np.array(
                [s.e_lat, s.roll, 0, 0, s.v_lat, s.roll_rate, 0, 0]))
        a, b = mix(hub / self.p["omni_wheel"]["outer_radius"], diff)
        self.data.ctrl[self._aid["drive_a"]] = a
        self.data.ctrl[self._aid["drive_b"]] = b
        self.data.ctrl[self._aid["steer"]] = self._steer

        # optional disturbance push (lateral, world frame)
        chassis = self.model.body("chassis").id
        self.data.xfrc_applied[chassis, :] = 0.0
        if self.rand["enabled"] and self._np_random.random() < self.rand["disturb_prob"]:
            self.data.xfrc_applied[chassis, 1] = (
                self._np_random.uniform(-1, 1) * self.rand["disturb_force_N"])

        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        self._psi += np.arctan2(
            np.sin(extract_state(self.data, self._p0).yaw - self._raw_prev),
            np.cos(extract_state(self.data, self._p0).yaw - self._raw_prev))
        self._raw_prev = extract_state(self.data, self._p0).yaw
        self._step += 1

        obs, s, e_lat, yaw_err = self._obs()
        rw = self.rw
        progress = abs(self._prev_yaw_err) - abs(yaw_err)
        reward = (rw["w_yaw_progress"] * progress
                  - rw["w_upright"] * s.roll**2
                  - rw["w_lateral"] * e_lat**2
                  - rw["w_effort"] * float(action @ action)
                  - rw["w_smooth"] * float((action - self._prev_a) @ (action - self._prev_a))
                  - rw["time_penalty"])
        self._prev_yaw_err = yaw_err
        self._prev_a = action

        fell = abs(s.roll) > self.fall or not np.all(np.isfinite(self.data.qpos))
        settled = (abs(yaw_err) < self.yaw_tol and abs(s.roll) < self.roll_ok
                   and abs(s.roll_rate) < self.rate_ok
                   and abs(self.data.qvel[5]) < self.rate_ok
                   and abs(s.v_lon) < self.rate_ok)
        terminated = False
        if fell:
            reward -= rw["penalty_fall"]
            terminated = True
        elif settled:
            reward += rw["bonus_complete"]
            terminated = True
        truncated = self._step >= self.max_steps
        # "is_success" is the key SB3 aggregates into rollout/success_rate.
        return obs, float(reward), terminated, truncated, {
            "yaw_err_deg": float(np.degrees(yaw_err)), "e_lat": float(e_lat),
            "success": bool(settled), "is_success": bool(settled)}


def make_env(params=None, rl_cfg=None, seed=None):
    """Thunk for SB3 vectorized env constructors."""
    def _thunk():
        return FlickEnv(params, rl_cfg, seed)
    return _thunk
