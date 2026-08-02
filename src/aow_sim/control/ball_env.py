"""Gymnasium environment for learning the ball-shot move (training only).

From a standstill the bike accelerates and turns to strike a stationary road-
hockey ball with the side 'stick' (not the wheels), launch it forward, then
recover to balance — and stay upright when there is no ball (catch trials). See
docs/plans/ball-shot-move.md. Structured like `flick_env.FlickEnv`: the model,
substepping, domain randomization, disturbance pushes, and the shared obs/action
contract (`ball_spec`) + crawl-balance `_K0` all follow the flick so a policy
trained here replays identically through `DriveController`. The additions are the
ball (placed at reset in the bike frame, or parked away for no-ball trials) and a
contact scan that classifies ball strikes as stick vs wheel for the reward.
"""

from __future__ import annotations

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from ..build_model import build_model, load_params
from .ball_spec import ACT_DIM, OBS_DIM, ActionBounds, build_obs, scale_action
from .balance import extract_state, mix
from .drive import DriveController
from .linearize import settle_upright

_PARKED = np.array([100.0, 100.0])   # world xy where a no-ball trial hides the ball


def _load_rl_config(path=None) -> dict:
    import yaml
    from pathlib import Path
    p = path or Path(__file__).resolve().parents[3] / "config" / "rl_ball.yaml"
    with open(p) as f:
        return yaml.safe_load(f)


class BallEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, params=None, rl_cfg=None, seed=None):
        super().__init__()
        self.p = params or load_params()
        self.cfg = rl_cfg or _load_rl_config()
        self.model = build_model(self.p, variant="full", hockey=True)
        self._eq = settle_upright(self.model).qpos.copy()
        self._mass0 = self.model.body_mass.copy()
        self._friction0 = self.model.geom_friction.copy()
        self.data = mujoco.MjData(self.model)
        # K0 crawl-balance gain from the clean (ball-free) bike, exactly as replay.
        self._K0 = DriveController(self.p, build_model(self.p, variant="full"))._K0

        env = self.cfg["env"]
        self.full = env["action_space"] == "full"
        self.bounds = ActionBounds(**env["action_bounds"])
        self.ctrl_dt = 1.0 / env["control_rate_hz"]
        self.substeps = max(1, round(self.ctrl_dt / self.model.opt.timestep))
        self.max_steps = int(env["max_episode_s"] / self.ctrl_dt)
        self.ball_start = np.array(env["ball_start"], float)
        self.ball_jitter = env["ball_start_jitter"]
        self.mirror_prob = env.get("mirror_prob", 0.0)
        self.no_ball_prob = env["no_ball_prob"]
        self.launch_target = np.deg2rad(env["launch_target_deg"])
        self.hit_speed_min = env["hit_speed_min"]
        self.roll_ok = np.deg2rad(env["success_roll_deg"])
        self.rate_ok = env["success_rate"]
        self.pitch_free = np.deg2rad(env["pitch_free_deg"])
        self.rw = self.cfg["reward"]
        self.rand = self.cfg["randomization"]
        self.fall = np.deg2rad(self.rw["fall_roll_deg"])

        act_dim = ACT_DIM if self.full else 2
        self.action_space = spaces.Box(-1.0, 1.0, (act_dim,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
        self._aid = {n: self.model.actuator(n).id
                     for n in ("drive_a", "drive_b", "steer")}
        self._sj = self.model.joint("steer_joint").qposadr[0]

        # ball free-joint addressing + contact-geom classification
        bjid = int(self.model.body("ball").jntadr[0])
        self._ball_q = int(self.model.jnt_qposadr[bjid])   # qpos[.:.+7]
        self._ball_v = int(self.model.jnt_dofadr[bjid])    # qvel[.:.+6]
        self._ball_r = self.p["hockey"]["ball"]["radius"]
        self._ball_g = int(self.model.geom("ball").id)
        self._stick_g = {int(self.model.geom(n).id) for n in ("stick_left", "stick_right")}
        # Centre of a stick panel in the chassis frame — the point the approach
        # shaping aims at (see _strike_point).
        _st = self.p["hockey"]["stick"]["pos"]
        self._stick_mid = (float(_st[0]), abs(float(_st[1])))
        self._wheel_g = {int(self.model.geom(g).id) for g in range(self.model.ngeom)
                         if (self.model.geom(g).name or "").startswith("roller")
                         or self.model.geom(g).name == "front_tire"}
        self._np_random, _ = gym.utils.seeding.np_random(seed)

    # -- helpers -----------------------------------------------------------

    def _ball_in_bike_frame(self):
        """Ball position/velocity relative to the bike, rotated into the bike
        (heading) frame. Returns (dx, dy, vx, vy)."""
        c, s = np.cos(self._psi), np.sin(self._psi)
        rel = self.data.qpos[self._ball_q:self._ball_q + 2] - self.data.qpos[:2]
        vel = self.data.qvel[self._ball_v:self._ball_v + 2]
        dx = c * rel[0] + s * rel[1]
        dy = -s * rel[0] + c * rel[1]
        vx = c * vel[0] + s * vel[1]
        vy = -s * vel[0] + c * vel[1]
        return dx, dy, vx, vy

    def _obs(self):
        st = extract_state(self.data, self._p0)
        d = self.data.qpos[:2] - self._p0
        e_lat = -np.sin(self._yaw0) * d[0] + np.cos(self._yaw0) * d[1]
        if self.has_ball:
            bdx, bdy, bvx, bvy = self._ball_in_bike_frame()
            present = 1.0
        else:
            bdx = bdy = bvx = bvy = 0.0
            present = 0.0
        obs = build_obs(st.roll, st.roll_rate, self._psi - self._target_yaw,
                        self.data.qvel[5], self.data.qpos[self._sj],
                        st.v_lon, st.v_lat, bdx, bdy, bvx, bvy, present,
                        self._step / self.max_steps)
        return obs, st, e_lat

    def _strike_point(self):
        """World xy of the centre of the striking stick panel (the side the ball
        is on).

        Approach shaping must close on *this*, not on the chassis origin: the
        chassis frame origin is the rear axle centre, so rewarding the bike for
        closing that distance is literally an instruction to drive the rear
        wheel onto the ball — which is the behaviour that was learned.
        """
        sx, sy = self._stick_mid
        sy *= 1.0 if self._mirror else -1.0        # ball starts right (y<0)
        c, s = np.cos(self._psi), np.sin(self._psi)
        return self.data.qpos[:2] + np.array([c * sx - s * sy, s * sx + c * sy])

    def _ball_offset(self):
        """(vector strike_point -> ball, its norm)."""
        v = self.data.qpos[self._ball_q:self._ball_q + 2] - self._strike_point()
        return v, float(np.linalg.norm(v))

    def _perp_align(self, to_ball, dist):
        """cos of the angle between the striking side's outward normal and the
        direction to the ball: +1 when the bike is broadside to it, which is the
        pose a side-mounted stick must be in to strike."""
        if dist < 1e-9:
            return 1.0
        c, s = np.cos(self._psi), np.sin(self._psi)
        side = 1.0 if self._mirror else -1.0
        outward = np.array([-s, c]) * side          # body +Y (left), signed
        return float(outward @ (to_ball / dist))

    def _pitch(self):
        """Chassis pitch [rad], nose-up positive.

        Not part of BikeState — the balance controllers only track roll — but
        the ball task needs it: nothing else in the reward looks at pitch, so
        flooring the drive to reach the ball earns a wheelie for free, and a
        lifted front wheel also unloads the steering constraint and lets the
        bike yaw more freely into the strike.
        """
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.data.qpos[3:7])
        return float(np.arcsin(np.clip(R.reshape(3, 3)[2, 0], -1.0, 1.0)))

    def _bonus(self):
        """Completion bonus, ramped by achieved launch speed.

        A *flat* bonus makes a gentle tap that reliably completes the optimal
        play: measured on the first good run, ~87% of episode reward came from
        merely arriving and finishing, so hitting harder bought almost nothing
        at the margin while risking the bonus outright. Ramping puts a gradient
        under launch speed instead of a cliff at hit_speed_min — which stays low,
        as a floor for "a shot happened", never as the performance target.
        """
        rw = self.rw
        if not self.has_ball:
            return rw["bonus_complete"]      # no-ball trial: surviving is the task
        ref = max(1e-6, rw["bonus_speed_ref"])
        return rw["bonus_complete"] * float(np.clip(self._peak_proj / ref, 0.0, 1.0))

    def _strike_normal_speed(self):
        """Speed of the stick panel *into* the ball, along the panel's outward
        normal [m/s]. Negative means retreating.

        This is what separates a strike from a sideswipe. `perp_align` only
        constrains the bike's *pose*: a glancing pass is broadside (perp ~ +1)
        the whole way through while carrying almost no normal velocity, so it
        transfers almost no momentum. Momentum transfer goes with the normal
        component of the panel's velocity, so that is what has to be paid for.
        """
        c, s = np.cos(self._psi), np.sin(self._psi)
        side = 1.0 if self._mirror else -1.0
        outward = np.array([-s, c]) * side
        r = self._strike_point() - self.data.qpos[:2]      # lever arm, world
        omega = float(self.data.qvel[5])                   # yaw rate
        v = self.data.qvel[:2] + omega * np.array([-r[1], r[0]])
        return float(v @ outward)

    def _approach_potential(self):
        """Shaping potential Phi: closer is better, and being broadside matters
        more the nearer we get. Applied as the difference Phi' - Phi
        (potential-based shaping), so it *guides* without being farmable —
        lingering in a good pose accrues nothing, only progress pays."""
        rw = self.rw
        to_ball, d = self._ball_offset()
        return (-rw["w_approach"] * d
                + rw["w_perp"] * self._perp_align(to_ball, d)
                * float(np.exp(-d / rw["perp_range"])))

    def _scan_ball_contacts(self):
        """Classify this step's ball contacts. Returns (hit_stick, hit_wheel)."""
        hit_stick = hit_wheel = False
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if self._ball_g not in (g1, g2):
                continue
            other = g2 if g1 == self._ball_g else g1
            if other in self._stick_g:
                hit_stick = True
            elif other in self._wheel_g:
                hit_wheel = True
        return hit_stick, hit_wheel

    def _ball_speed(self):
        return float(np.linalg.norm(self.data.qvel[self._ball_v:self._ball_v + 2]))

    def _projected_speed(self):
        """Ball speed along the world-frame target direction [m/s]. Negative if
        the ball is going backwards relative to the target.

        This, not |v|, is the objective. Rewarding raw speed pays for a fast ball
        in *any* direction, which is how a front-edge scrape that squirted the
        ball out at -98 deg outscored a real shot; direction was left to a weak
        w_angle add-on that a big speed term simply outbid. Projecting makes
        direction intrinsic: a sideways squirt is worth ~0 no matter how fast.
        """
        v = self.data.qvel[self._ball_v:self._ball_v + 2]
        return float(self._target_dir() @ v)

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

    def _place_ball(self):
        """Place the ball at a WORLD-frame position (with jitter/mirror). With
        no_ball_prob, park it far away.

        Deliberately not rotated by the bike's start heading: placing it in the
        start frame pinned the ball, the bike and the launch target rigidly
        together, so init_yaw/init_pos rotated the whole scene and cancelled out
        rather than perturbing anything. World placement makes them real
        disturbances the policy has to correct for.
        """
        rng = self._np_random
        self.has_ball = rng.random() >= self.no_ball_prob
        if not self.has_ball:
            self.data.qpos[self._ball_q:self._ball_q + 2] = _PARKED
            self.data.qpos[self._ball_q + 2] = self._ball_r
            self.data.qpos[self._ball_q + 3:self._ball_q + 7] = [1, 0, 0, 0]
            self.data.qvel[self._ball_v:self._ball_v + 6] = 0.0
            return
        bxy = self.ball_start.copy()
        if rng.random() < self.mirror_prob:
            bxy[1] = -bxy[1]                       # mirror to a ball-left start
            self._mirror = True
        bxy += rng.uniform(-1, 1, 2) * self.ball_jitter
        self.data.qpos[self._ball_q:self._ball_q + 2] = bxy
        self.data.qpos[self._ball_q + 2] = self._ball_r
        self.data.qpos[self._ball_q + 3:self._ball_q + 7] = [1, 0, 0, 0]
        self.data.qvel[self._ball_v:self._ball_v + 6] = 0.0

    # -- gym API -----------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self._np_random, _ = gym.utils.seeding.np_random(seed)
        rng = self._np_random
        self._apply_randomization()
        self.data.qpos[:] = self._eq
        self.data.qvel[:] = 0.0
        self._mirror = False
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

        self._p0 = self.data.qpos[:2].copy()
        self._yaw0 = self._psi = extract_state(self.data, self._p0).yaw
        self._raw_prev = self._yaw0
        self._place_ball()
        # World-frame launch target (0 = world +x), set after _place_ball since
        # mirroring flips it. NOT relative to _yaw0: the goal direction is fixed
        # in the world and the bike's start heading is a disturbance, not a
        # redefinition of where the ball is supposed to go.
        self._target_yaw = -self.launch_target if self._mirror else self.launch_target
        mujoco.mj_forward(self.model, self.data)
        self._steer = float(self.data.qpos[self._sj])
        self._prev_a = np.zeros(self.action_space.shape[0])
        self._step = 0
        self._hit_stick = self._hit_wheel = False
        self._peak_speed = 0.0
        self._peak_proj = 0.0    # peak speed ALONG the target dir — the objective
        self._in_stick_contact = False   # for rising/falling-edge strike detection
        self._launch_paid = False        # launch reward is once per episode
        self._wheel_paid = False         # ... and so is the wheel-hit penalty
        self._strike_paid = False        # ... and the strike (normal-speed) reward
        self._strike_speed = 0.0         # normal approach speed at first contact
        self._peak_pitch = 0.0           # max |pitch| seen (wheelie diagnostic)
        self._prev_phi = self._approach_potential() if self.has_ball else 0.0
        obs, _, _ = self._obs()
        return obs, {}

    def step(self, action):
        action = np.asarray(action, np.float32)
        steer_rate, hub, diff = scale_action(action, self.bounds)
        self._steer += steer_rate * self.ctrl_dt
        if not self.full:                       # feedforward: crawl balance diff
            st = extract_state(self.data, self._p0)
            diff = float(-self._K0[0] @ np.array(
                [st.e_lat, st.roll, 0, 0, st.v_lat, st.roll_rate, 0, 0]))
        a, b = mix(hub / self.p["omni_wheel"]["outer_radius"], diff)
        self.data.ctrl[self._aid["drive_a"]] = a
        self.data.ctrl[self._aid["drive_b"]] = b
        self.data.ctrl[self._aid["steer"]] = self._steer

        chassis = self.model.body("chassis").id
        self.data.xfrc_applied[chassis, :] = 0.0
        if self.rand["enabled"] and self._np_random.random() < self.rand["disturb_prob"]:
            self.data.xfrc_applied[chassis, 1] = (
                self._np_random.uniform(-1, 1) * self.rand["disturb_force_N"])

        # Sampled before the physics substeps: once contact is detected the ball
        # is already being pushed, so the approach velocity has to be read from
        # the instant *before* the strike.
        vn_pre = self._strike_normal_speed() if self.has_ball else 0.0

        hit_stick = hit_wheel = False
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
            if self.has_ball:
                hs, hw = self._scan_ball_contacts()
                hit_stick |= hs
                hit_wheel |= hw
        # unwrap chassis heading
        cur_yaw = extract_state(self.data, self._p0).yaw
        self._psi += np.arctan2(np.sin(cur_yaw - self._raw_prev),
                                np.cos(cur_yaw - self._raw_prev))
        self._raw_prev = cur_yaw
        self._step += 1

        obs, st, e_lat = self._obs()
        rw = self.rw
        # Deadband so ordinary pitch under acceleration is free and only an
        # actual wheelie is charged for.
        pitch = self._pitch()
        self._peak_pitch = max(self._peak_pitch, abs(pitch))
        pitch_excess = max(0.0, abs(pitch) - self.pitch_free)

        reward = (-rw["w_pitch"] * pitch_excess**2
                  - rw["w_upright"] * st.roll**2
                  - rw["w_lateral"] * e_lat**2
                  - rw["w_effort"] * float(action @ action)
                  - rw["w_smooth"] * float((action - self._prev_a) @ (action - self._prev_a))
                  - rw["time_penalty"])

        speed = self._ball_speed() if self.has_ball else 0.0
        proj = self._projected_speed() if self.has_ball else 0.0
        if self.has_ball:
            phi = self._approach_potential()
            if not (self._hit_stick or self._hit_wheel):          # pre-hit approach
                reward += phi - self._prev_phi
            self._prev_phi = phi

            # Launch reward is paid ONCE, on the falling edge of stick contact —
            # i.e. when the ball separates, so `speed` is its true launch speed.
            # Paying it per contact step instead makes a *sustained shove* score
            # far better than a clean strike (contact duration x speed), which is
            # exactly the degenerate carry-the-ball behaviour it used to learn.
            # Rising edge: pay for actually driving the panel into the ball.
            # Without this, a broadside glancing pass scores the full approach
            # shaping while transferring almost no momentum — a sideswipe.
            if hit_stick and not self._in_stick_contact and not self._strike_paid:
                reward += rw["w_strike"] * max(0.0, vn_pre)
                self._strike_paid = True
                self._strike_speed = vn_pre

            released = self._in_stick_contact and not hit_stick
            if released and not self._launch_paid:
                reward += rw["w_launch"] * max(0.0, proj)
                self._launch_paid = True
            self._in_stick_contact = hit_stick
            if hit_wheel and not self._wheel_paid:   # once, for the same reason
                reward -= rw["w_wheel_hit"]
                self._wheel_paid = True
            self._hit_stick |= hit_stick
            self._hit_wheel |= hit_wheel
            self._peak_speed = max(self._peak_speed, speed)
            self._peak_proj = max(self._peak_proj, proj)

        self._prev_a = action

        fell = abs(st.roll) > self.fall or not np.all(np.isfinite(self.data.qpos))
        settled = (abs(st.roll) < self.roll_ok
                   and abs(st.roll_rate) < self.rate_ok
                   and abs(self.data.qvel[5]) < self.rate_ok
                   and abs(st.v_lon) < self.rate_ok)
        shot_taken = self._hit_stick and self._peak_proj > self.hit_speed_min

        terminated = False
        success = False
        if fell:
            reward -= rw["penalty_fall"]
            terminated = True
        elif self.has_ball and shot_taken and settled:
            reward += self._bonus()
            terminated = True
            success = True
        truncated = self._step >= self.max_steps
        if truncated and not terminated and not fell:
            # no-ball trial survived, or ball trial that took its shot in time
            success = (not self.has_ball) or shot_taken
            if success:
                reward += self._bonus()
        # Episode ended mid-contact: settle the unpaid launch reward so ending
        # while still touching the ball is never a way to dodge the accounting.
        if (terminated or truncated) and self._in_stick_contact and not self._launch_paid:
            reward += rw["w_launch"] * max(0.0, proj)
            self._launch_paid = True

        if self.has_ball:                       # approach diagnostics
            _to_ball, _d = self._ball_offset()
            strike_dist, perp = _d, self._perp_align(_to_ball, _d)
        else:
            strike_dist, perp = 0.0, 0.0
        return obs, float(reward), terminated, truncated, {
            "ball_speed": float(self._peak_speed),
            "proj_speed": float(self._peak_proj),
            "launch_deg": float(np.degrees(self._launch_dir())) if self.has_ball else 0.0,
            "hit_stick": bool(self._hit_stick), "hit_wheel": bool(self._hit_wheel),
            "has_ball": bool(self.has_ball),
            "strike_dist": float(strike_dist), "perp_align": float(perp),
            "strike_speed": float(self._strike_speed),
            "peak_pitch_deg": float(np.degrees(self._peak_pitch)),
            "success": bool(success), "is_success": bool(success)}

    # -- launch geometry ---------------------------------------------------

    def _launch_dir(self):
        """World-frame heading of the ball's horizontal velocity (rad)."""
        v = self.data.qvel[self._ball_v:self._ball_v + 2]
        return float(np.arctan2(v[1], v[0]))

    def _target_dir(self):
        """Unit vector of the world-frame launch target direction."""
        return np.array([np.cos(self._target_yaw), np.sin(self._target_yaw)])

def make_env(params=None, rl_cfg=None, seed=None):
    """Thunk for SB3 vectorized env constructors."""
    def _thunk():
        return BallEnv(params, rl_cfg, seed)
    return _thunk
