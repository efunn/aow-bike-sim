"""Gymnasium environment for learning the pivot (training only).

The move: the chassis yaws 180 deg while the front wheel holds its global
ground heading (mod pi) — see pivot_spec's module docstring for the frames.
Episodes start already gliding at a sampled v_start along the initial line
(direct qvel injection: chassis + every wheel/shaft rolling consistently —
a scripted pre-roll would cost ~a second of sim per episode) and must end
tracking a sampled v_end, upright, on-heading and on-line. Reward is
yaw progress plus penalties on the mod-pi wheel-heading hold error, the
(phase-weighted) velocity tracking error, and the FRONT-CONTACT lateral
offset from the original line (the chassis necessarily leaves the line —
penalizing it would fight the move). A ball-hit-scale impulse fires once
per episode at the yaw-halfway crossing (magnitude sampled from 0 = miss
up to a full hit), so the policy learns to survive both hitting and
missing. Domain randomization as in flick_env.
"""

from __future__ import annotations

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from ..build_model import build_model, load_params
from .balance import extract_state, mix
from .drive import DriveController
from .linearize import settle_upright
from .pivot_spec import (ACT_DIM, OBS_DIM, ActionBounds, build_obs,
                         scale_action, wheel_heading, wrap_pi)


def _load_rl_config(path=None) -> dict:
    import yaml
    from pathlib import Path
    p = path or Path(__file__).resolve().parents[3] / "config" / "rl_pivot.yaml"
    with open(p) as f:
        return yaml.safe_load(f)


class PivotEnv(gym.Env):
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
        self.v_max = env["v_max"]
        self.p_v_zero = env["p_v_zero"]
        self.yaw_tol = np.deg2rad(env["success_yaw_tol_deg"])
        self.roll_ok = np.deg2rad(env["success_roll_deg"])
        self.rate_ok = env["success_rate"]
        self.v_tol = env["success_v_tol"]
        self.hold_tol = np.deg2rad(env["success_hold_tol_deg"])
        self.line_tol = env["success_line_tol_m"]
        self.hold_steps = max(1, round(env["success_hold_s"] / self.ctrl_dt))
        self.hit_cfg = env["hit_impulse"]
        self.rw = self.cfg["reward"]
        self.rand = self.cfg["randomization"]
        self.fall = np.deg2rad(self.rw["fall_roll_deg"])
        self._rake = np.deg2rad(self.p["bike"]["rake_deg"])
        self.L = self.p["bike"]["wheelbase"]
        self.r_front = self.p["bike"]["front_wheel"]["radius"]
        self.r_rear = self.p["omni_wheel"]["outer_radius"]

        act_dim = ACT_DIM if self.full else 2
        self.action_space = spaces.Box(-1.0, 1.0, (act_dim,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
        self._aid = {n: self.model.actuator(n).id
                     for n in ("drive_a", "drive_b", "steer")}
        self._sj = self.model.joint("steer_joint").qposadr[0]
        # dof addresses for the consistent-glide qvel injection
        self._d_front = self.model.joint("front_spin").dofadr[0]
        self._d_hub = self.model.joint("hub_spin").dofadr[0]
        self._d_in_a = self.model.joint("input_a_spin").dofadr[0]
        self._d_in_b = self.model.joint("input_b_spin").dofadr[0]
        self._chassis = self.model.body("chassis").id
        self._np_random, _ = gym.utils.seeding.np_random(seed)

    # -- helpers -----------------------------------------------------------

    def _front_contact(self, yaw):
        return self.data.qpos[:2] + self.L * np.array([np.cos(yaw), np.sin(yaw)])

    def _front_vel(self, yaw):
        return (self.data.qvel[:2] + self.L * self.data.qvel[5]
                * np.array([-np.sin(yaw), np.cos(yaw)]))

    def _v_ref(self):
        phase = self._step / self.max_steps
        return self._v_start + (self._v_end - self._v_start) * phase

    def _obs(self):
        s = extract_state(self.data, self._p0)
        delta = float(self.data.qpos[self._sj])
        hold_raw = self._psi + wheel_heading(delta, self._rake) - self._theta0
        yaw_err = self.target - (self._psi - self._yaw0)
        e_line = float(self._n0 @ (self._front_contact(s.yaw) - self._pf0))
        v_along = float(self._u0 @ self._front_vel(s.yaw))
        phase = self._step / self.max_steps
        obs = build_obs(s.roll, s.roll_rate, yaw_err, self.data.qvel[5],
                        delta, hold_raw, s.v_lon, s.v_lat,
                        v_along - self._v_ref(), self._v_end, e_line, phase)
        return obs, s, e_line, yaw_err, v_along, hold_raw

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

    def _sample_v(self, rng) -> float:
        if rng.random() < self.p_v_zero:
            return 0.0
        return float(rng.uniform(0.0, self.v_max))

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
        self._yaw0 = self._psi = extract_state(self.data, self._p0).yaw
        self._raw_prev = self._yaw0

        # Velocity targets (overridable for the deterministic eval grid).
        opts = options or {}
        self._v_start = float(opts.get("v_start", self._sample_v(rng)))
        self._v_end = float(opts.get("v_end", self._sample_v(rng)))

        # Glide injection: chassis + all wheels/shafts rolling consistently
        # at v_start along yaw0 (hub = (a+b)/2, ring_abs = a; zero diff =>
        # a = b = v/r; ring_spin is hub-RELATIVE -> 0; rollers -> 0).
        v = self._v_start
        if v:
            c, sn = np.cos(self._yaw0), np.sin(self._yaw0)
            self.data.qvel[0:2] += v * np.array([c, sn])
            self.data.qvel[self._d_front] += v / self.r_front
            w = v / self.r_rear
            self.data.qvel[self._d_hub] += w
            self.data.qvel[self._d_in_a] += w
            self.data.qvel[self._d_in_b] += w
            mujoco.mj_forward(self.model, self.data)

        # Line frame: held global wheel heading + front-contact anchor.
        self._theta0 = self._yaw0 + wheel_heading(
            float(self.data.qpos[self._sj]), self._rake)
        self._u0 = np.array([np.cos(self._theta0), np.sin(self._theta0)])
        self._n0 = np.array([-np.sin(self._theta0), np.cos(self._theta0)])
        self._pf0 = self._front_contact(self._yaw0).copy()

        # Ball-hit impulse: sampled once, fires at the yaw-halfway crossing.
        hi = self.hit_cfg
        hit = r["enabled"] and rng.random() < hi["prob"]
        self._hit_F = float(rng.uniform(0.0, hi["force_N"])) if hit else 0.0
        ang = rng.uniform(-np.pi, np.pi)
        self._hit_dir = np.array([np.cos(ang), np.sin(ang)])
        self._hit_window = max(1, round(hi["window_s"] / self.ctrl_dt))
        self._hit_left = 0
        self._hit_armed = True

        self._steer = float(self.data.qpos[self._sj])
        self._prev_a = np.zeros(self.action_space.shape[0])
        self._step = 0
        self._hold_sq_sum = 0.0
        self._succ_streak = 0
        obs, _, _, self._prev_yaw_err, _, _ = self._obs()
        return obs, {}

    def step(self, action):
        action = np.asarray(action, np.float32)
        steer_rate, hub, diff = scale_action(action, self.bounds)
        self._steer += steer_rate * self.ctrl_dt
        if not self.full:                       # feedforward: crawl balance diff
            s = extract_state(self.data, self._p0)
            diff = float(-self._K0[0] @ np.array(
                [s.e_lat, s.roll, 0, 0, s.v_lat, s.roll_rate, 0, 0]))
        a, b = mix(hub / self.r_rear, diff)
        self.data.ctrl[self._aid["drive_a"]] = a
        self.data.ctrl[self._aid["drive_b"]] = b
        self.data.ctrl[self._aid["steer"]] = self._steer

        # Disturbances: flick-style random shove + the scheduled ball hit.
        self.data.xfrc_applied[self._chassis, :] = 0.0
        if self.rand["enabled"] and self._np_random.random() < self.rand["disturb_prob"]:
            self.data.xfrc_applied[self._chassis, 1] = (
                self._np_random.uniform(-1, 1) * self.rand["disturb_force_N"])
        if self._hit_armed and abs(self._psi - self._yaw0) >= np.pi / 2:
            self._hit_armed = False             # fires at most once
            self._hit_left = self._hit_window
        if self._hit_left > 0:
            self.data.xfrc_applied[self._chassis, 0:2] += self._hit_F * self._hit_dir
            self._hit_left -= 1

        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        cur = extract_state(self.data, self._p0).yaw
        self._psi += np.arctan2(np.sin(cur - self._raw_prev),
                                np.cos(cur - self._raw_prev))
        self._raw_prev = cur
        self._step += 1

        obs, s, e_line, yaw_err, v_along, hold_raw = self._obs()
        rw = self.rw
        e_hold = 0.5 * wrap_pi(2.0 * hold_raw)
        v_err = v_along - self._v_ref()
        phase = self._step / self.max_steps
        progress = abs(self._prev_yaw_err) - abs(yaw_err)
        reward = (rw["w_yaw_progress"] * progress
                  - rw["w_wheel_hold"] * e_hold**2
                  - rw["w_vel"] * phase * v_err**2
                  - rw["w_line"] * e_line**2
                  - rw["w_upright"] * s.roll**2
                  - rw["w_effort"] * float(action @ action)
                  - rw["w_smooth"] * float((action - self._prev_a) @ (action - self._prev_a))
                  - rw["time_penalty"])
        self._prev_yaw_err = yaw_err
        self._prev_a = action
        self._hold_sq_sum += e_hold**2

        fell = abs(s.roll) > self.fall or not np.all(np.isfinite(self.data.qpos))
        ok = (abs(yaw_err) < self.yaw_tol
              and abs(s.roll) < self.roll_ok
              and abs(s.roll_rate) < self.rate_ok
              and abs(self.data.qvel[5]) < self.rate_ok
              and abs(v_along - self._v_end) < self.v_tol
              and abs(e_hold) < self.hold_tol
              and abs(e_line) < self.line_tol)
        self._succ_streak = self._succ_streak + 1 if ok else 0
        settled = self._succ_streak >= self.hold_steps
        terminated = False
        if fell:
            reward -= rw["penalty_fall"]
            terminated = True
        elif settled:
            reward += rw["bonus_complete"]
            terminated = True
        truncated = self._step >= self.max_steps
        hold_rms = np.sqrt(self._hold_sq_sum / self._step)
        return obs, float(reward), terminated, truncated, {
            "yaw_err_deg": float(np.degrees(yaw_err)),
            "e_line": float(e_line),
            "hold_rms_deg": float(np.degrees(hold_rms)),
            "v_err_end": float(v_along - self._v_end),
            "v_start": self._v_start, "v_end": self._v_end,
            "hit_F": self._hit_F,
            "success": bool(settled), "is_success": bool(settled)}


def make_env(params=None, rl_cfg=None, seed=None):
    """Thunk for SB3 vectorized env constructors."""
    def _thunk():
        return PivotEnv(params, rl_cfg, seed)
    return _thunk
