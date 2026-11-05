"""Gymnasium environment for learning the get-up on the swing module (training only).

From lying on a blade to a prop inside the hand-off window, the crank on
CurrentBasedPositionServo (the measured mode-5 law, gearbox and linkage
friction), the general policy off, the drives held -- the job swing_sit's
`lift` does by script. The observation/action contract is righting_spec.

Why learn it: on the bike the lift is at the servo's limit (status.md,
self-righting row) -- left get-ups stall at roll 62-64 deg with PWM pinned,
and the sim's model is ~0.15 N m light at the crank. So the servo's strength
is randomized DOWN (`servo_strength`) to cover the bike, and the scripted
full-stroke ramp, which in this sim throws the bike over the top, is not the
answer the policy is pushed toward: it is paid for a still prop.

Episode ends:
  success   |roll| under `engage_deg` and |roll rate| under `still_rate`,
            held `hold_s` -- Enter's window (run_bike PROP_ENGAGE_DEG)
  over      past upright by `over_deg` onto the other side
  overload  the heat proxy reaches 1 (below)

THE OVERLOAD ESTIMATE IS THE BENCH'S TRIP LAW (B2, 2026-11-03,
analysis/servo/righting_overload.py, the module's XC330 held at a stall):
time to trip t = `charge_as` / (|I| - `floor_a`), 0.645 A s and 0.413 A,
fit to trips at 730 / 700 / 600 / 500 mA (1.98 / 2.30 / 3.62 / 6.98 s,
within 0.5 s); it predicts the bike's manual trips (670-750 mA: 1.9-2.5 s
against 1.7 and 2.3). On CURRENT: 600 and 500 mA tripped with PWM at
87-92 %, so it is not PWM saturation. So heat climbs at
(|I| - floor_a) / charge_as per s above the floor and trips at 1. Below
it, it drains at 1/`cool_s`: B2b (righting_recovery.py, 11-03) charged
2/3 of a trip at 700 mA, held 1-8 s at ~85 mA, and the next push lasted
2.34-2.62 s against a fresh 2.30 -- all of it back within 1 s. CHECKED
ON THE BIKE: replayed over the righting servo's recorded current in all 77
bike runs, the law with `cool_s` 0.5 predicts all 7 real overloads 0.04-
0.14 s early (the 10-26 pumping run's included, which tells the drain
rates apart: 1.0 is 0.39 s early, 2.0 0.9, none 15 s), misses none, and
calls one trip that did not happen (261026-191739, the wing-rock run).
Untested under 500 mA: the fit says 450 trips in ~18 s and 420 in ~95.

Starting poses are DROPS, not the bike's lie-down: the policy is cut with a
roll kick (and some speed, the drives then held) and the bike falls onto a
limp crank (sim rest ~70 deg roll, crank ~66 deg past centre), onto one
held at centre at full current (riding, module powered), or onto one held
at the bike's lying crank (`rest_crank_deg`, 25 past centre, run_bike
LIE_REST_DEG). Each drop gives MID-FALL states -- every `snap_dt_s` for
`fall_window_s` from the moment |roll| passes `takeover_deg`, where the
drive policy lets go and this one takes over -- and its REST state; an
episode starts at rest with `p_rest`. The sim lies at 70-85 deg where the
bike lies at 74 with its crank at 26.5-28; that mismatch (status.md, row 4)
is carried, not fixed, here.
"""

from __future__ import annotations

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from ..build_model import build_model, load_params, reset_actuator_state
from ..righting_servo import CurrentBasedPositionServo
from .balance import mix
from .crank_lead import lead_goal
from .linearize import settle_upright
from .randomize import DomainRandomizer
from .righting import roll_pitch
from .righting_spec import ACT_DIM, OBS_DIM, RightingBounds, build_obs, scale_action


def _load_rl_config(path=None) -> dict:
    import yaml
    from pathlib import Path
    p = path or Path(__file__).resolve().parents[3] / "config" / "rl" / "rl_righting.yaml"
    with open(p) as f:
        return yaml.safe_load(f)


class RightingEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, params=None, rl_cfg=None, seed=None):
        super().__init__()
        # The IDEAL drive, as the moves train: the drives only hold or
        # crawl the differential here, so the detailed XC430 loop buys nothing.
        from ..drivetrain_model import base_params
        self.p = base_params(params or load_params())
        self.cfg = rl_cfg or _load_rl_config()
        m = self.model = build_model(self.p, variant="full")
        self.data = mujoco.MjData(m)
        self._eq = settle_upright(m).qpos.copy()
        self._swing = m.actuator("swing").id
        self.srv = CurrentBasedPositionServo.attach(
            m, self.p, torque_nm=float(m.actuator_forcerange[self._swing, 1]),
            at_output=True)
        self._ca = int(m.joint("swing_crank_joint").qposadr[0])
        self._cd = int(m.joint("swing_crank_joint").dofadr[0])
        self._stroke = float(m.actuator_ctrlrange[self._swing, 1])
        self._aid = {n: m.actuator(n).id for n in ("drive_a", "drive_b", "steer")}
        self._sj = int(m.joint("steer_joint").qposadr[0])

        env = self.cfg["env"]
        self.bounds = RightingBounds(**env["action_bounds"])
        self.ctrl_dt = 1.0 / env["control_rate_hz"]
        self.substeps = max(1, round(self.ctrl_dt / m.opt.timestep))
        self.max_steps = int(env["max_episode_s"] / self.ctrl_dt)
        self.lead = (None if env.get("lead_deg") is None
                     else np.deg2rad(env["lead_deg"]))
        self.engage = np.deg2rad(env["engage_deg"])
        self.still = float(env["still_rate"])
        self.hold_n = max(1, round(env["hold_s"] / self.ctrl_dt))
        self.over = np.deg2rad(env["over_deg"])
        self.heat_cfg = env["overload"]
        self.rw = self.cfg["reward"]
        self.rand = self.cfg["randomization"]
        self._rand = DomainRandomizer(m, self.rand)

        self.action_space = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
        self._np_random, _ = gym.utils.seeding.np_random(seed)
        self._pool = self._make_pool(env["start"])

    # -- starting poses ------------------------------------------------------

    def _drop(self, side, kick, v, crank_goal, current, st):
        """The bike cut upright with a roll kick toward `side` at forward
        speed `v`, falling onto a crank held at `crank_goal` with `current`
        counts (0: limp). -> (mid-fall [(qpos, qvel)], rest (qpos, qvel))."""
        m, d = self.model, self.data
        d.qpos[:] = self._eq
        d.qvel[:] = 0.0
        d.ctrl[:] = 0.0
        d.qvel[0] = v
        d.qvel[3] = side * kick
        d.ctrl[self._swing] = crank_goal
        self.srv.set_goal_current(current)
        self.srv.reset(d)
        dt = m.opt.timestep
        every = max(1, round(st["snap_dt_s"] / dt))
        window = int(st["fall_window_s"] / dt)
        take = np.deg2rad(st["takeover_deg"])
        falling, k0 = [], None
        for k in range(int(st["drop_s"] / dt)):
            self.srv.pre_step(d)
            mujoco.mj_step(m, d)
            if k0 is None and side * self._roll() > take:
                k0 = k
            if k0 is not None and k - k0 < window and (k - k0) % every == 0:
                falling.append((d.qpos.copy(), d.qvel.copy()))
        return falling, (d.qpos.copy(), d.qvel.copy())

    def _make_pool(self, st):
        """Mid-fall and rest states, both sides, drawn once per env (a drop
        is ~0.15 s). -> {"falling": [(side, qpos, qvel)], "rest": [...]}."""
        rng = np.random.default_rng(int(self._np_random.integers(1 << 31)))
        rest = np.deg2rad(st["rest_crank_deg"])
        modes = ((0.0, 0), (0.0, st["held_current"]), (rest, st["rest_current"]))
        pool = {"falling": [], "rest": []}
        lying = np.deg2rad(st["min_lying_deg"])
        for side in (1, -1):
            for i in range(st["n_per_side"]):
                goal, current = modes[i % len(modes)]
                falling, (q, v) = self._drop(
                    side, rng.uniform(*st["kick_range"]),
                    rng.uniform(-1, 1) * st["v_max"], -side * goal, current, st)
                pool["falling"] += [(side, qf, vf) for qf, vf in falling]
                if side * np.deg2rad(roll_pitch(q[3:7])[0]) > lying:
                    pool["rest"].append((side, q, v * 0.0))
        if not pool["rest"] or not pool["falling"]:
            raise RuntimeError("drops gave no rest or no mid-fall states; "
                               "check env.start")
        return pool

    # -- helpers -------------------------------------------------------------

    def _roll(self):
        return np.deg2rad(roll_pitch(self.data.qpos[3:7])[0])

    def _obs(self):
        d = self.data
        return build_obs(self.side, self._roll(), float(d.qvel[3]),
                         float(d.qpos[self._ca]), float(d.qvel[self._cd]),
                         self.goal, self.srv.current,
                         self.srv.goal_current / max(1.0, self.bounds.current_max),
                         self.heat, self._step / self.max_steps)

    def _heat_step(self, dt):
        h, i = self.heat_cfg, abs(self.srv.current)
        if i > h["floor_a"]:
            self.heat += dt * (i - h["floor_a"]) / h["charge_as"]
        else:
            self.heat = max(0.0, self.heat - dt / h["cool_s"])

    # -- gym API -------------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self._np_random, _ = gym.utils.seeding.np_random(seed)
        rng, r = self._np_random, self.rand
        m, d = self.model, self.data
        self._rand.apply(rng)
        f = rng.uniform(*r["servo_strength"]) if r["enabled"] else 1.0
        self.srv.strength = f
        self.srv.set_supply(rng.uniform(*r["supply"]) if r["enabled"] else 1.0)
        self.strength = f

        opts = options or {}
        kind = opts.get("start") or (
            "rest" if rng.random() < self.cfg["env"]["start"]["p_rest"] else "falling")
        cands = [p for p in self._pool[kind]
                 if "side" not in opts or p[0] == opts["side"]]
        side, q, v = cands[int(rng.integers(len(cands)))]
        self.side, self.start = side, kind
        d.qpos[:] = q
        d.qvel[:] = v
        reset_actuator_state(m, d)
        d.ctrl[:] = 0.0
        self._steer = float(d.qpos[self._sj])
        d.ctrl[self._aid["steer"]] = self._steer
        self.goal = float(d.qpos[self._ca])
        d.ctrl[self._swing] = self.goal
        self.srv.set_goal_current(0)
        self.srv.reset(d)
        if kind == "rest":
            # Settle on the randomized masses, crank limp: how the bike lies.
            for _ in range(int(self.cfg["env"]["start"]["settle_s"] / m.opt.timestep)):
                self.srv.pre_step(d)
                mujoco.mj_step(m, d)
        else:
            mujoco.mj_forward(m, d)
        self.goal = float(d.qpos[self._ca])
        self.heat = 0.0
        self._step = self._held = 0
        self._prev_a = np.zeros(ACT_DIM)
        self._prev_roll = self.side * self._roll()
        return self._obs(), {}

    def step(self, action, command=None):
        """`command` = (goal [rad], Goal Current [counts], diff [rad/s]) in
        the SIDE frame flies those instead of `action` (scripted_ramp: the
        script's lift starts below the policy's goal range)."""
        m, d = self.model, self.data
        if command is None:
            action = np.asarray(action, np.float32)
            goal, gc, diff = scale_action(action, self.bounds, self.side)
        else:
            goal, gc, diff = (self.side * command[0], int(command[1]),
                              self.side * command[2])
            action = self._prev_a
        crank = float(d.qpos[self._ca])
        self.goal = lead_goal(float(np.clip(goal, -self._stroke, self._stroke)),
                              crank, self.lead)
        d.ctrl[self._swing] = self.goal
        self.srv.set_goal_current(gc)
        a, b = mix(0.0, diff)
        d.ctrl[self._aid["drive_a"]], d.ctrl[self._aid["drive_b"]] = a, b
        d.ctrl[self._aid["steer"]] = self._steer
        for _ in range(self.substeps):
            self.srv.pre_step(d)
            mujoco.mj_step(m, d)
            self._heat_step(m.opt.timestep)
        self._step += 1

        obs = self._obs()
        roll, rate_r = float(obs[0]), float(obs[1])
        rw = self.rw
        near = abs(roll) < np.deg2rad(rw["near_deg"])
        da = action - self._prev_a
        ok = abs(roll) < self.engage and abs(rate_r) < self.still
        reward = (rw["w_progress"] * (abs(self._prev_roll) - abs(roll))
                  + rw["w_prop"] * ok
                  - rw["w_rate_near"] * near * rate_r ** 2
                  - rw["w_current"] * self.srv.current ** 2
                  - rw["w_heat"] * self.heat
                  - rw["w_smooth"] * float(da @ da)
                  - rw["time_penalty"])
        self._prev_roll, self._prev_a = roll, action

        self._held = self._held + 1 if ok else 0
        success = self._held >= self.hold_n
        over = roll < -self.over
        tripped = self.heat >= 1.0
        broken = not np.all(np.isfinite(d.qpos))
        terminated = success or over or tripped or broken
        if success:
            reward += rw["bonus_success"]
        if over or broken:
            reward -= rw["penalty_over"]
        if tripped:
            reward -= rw["penalty_overload"]
        truncated = self._step >= self.max_steps
        return obs, float(reward), terminated, truncated, {
            "success": bool(success), "is_success": bool(success),
            "over": bool(over), "overload": bool(tripped),
            "roll_deg": float(np.degrees(roll)), "heat": float(self.heat),
            "strength": float(self.strength), "side": int(self.side),
            "start": self.start}


def scripted_ramp(cfg, dps=120.0):
    """swing_sit's lift as a `command` for RightingEnv.step: the goal slewed
    at `dps` from where it is to the full stroke, Goal Current at the top of
    its range. Marked `raw`: it returns a command, not an action."""
    b = RightingBounds(**cfg["env"]["action_bounds"])
    step = np.radians(dps) / cfg["env"]["control_rate_hz"]

    def act(obs, side):
        goal = float(obs[2] + obs[4]) + step                   # crank + lead
        return (goal, b.current_max, 0.0)                      # env clips
    act.raw = True
    return act


def make_env(params=None, rl_cfg=None, seed=None):
    """Thunk for SB3 vectorized env constructors."""
    def _thunk():
        return RightingEnv(params, rl_cfg, seed)
    return _thunk
