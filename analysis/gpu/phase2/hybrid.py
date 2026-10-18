"""End to end on ONE engine (user's suggestion, 2026-10-14): GeneralEnv's own
Python hooks, obs and reward driving MJWarp physics (the HYBRID), against the
all-Warp GpuGeneralEnv. Both step the same MJWarp, so the contact model is out
of the comparison and what is left is the port (float32 kernels vs float64
Python) -- grown by the bike's chaos, which a 1e-6 rad nudge of the Warp env
against itself measures as the yardstick.

The hybrid: `mujoco.mj_step` / `mj_forward` inside general_env are swapped
for MJWarp (state and the model fields the hooks or the randomizer write go
in, `get_data_into` brings everything back; `mj_forward` leaves the warm
start alone, as C's does). Three deliberate departures from GeneralEnv, each
so the two sides are driven alike, none in what they compute:

  * the gearbox's friction row is found by (type, id): MJWarp's row order is
    not MuJoCo's, and the Python hook reads a fixed row (the kernel searches)
  * the odometry ticks every 25th substep by count, as the port does, not by
    the Python env's drifting float clock (499 ticks in 5 s, not 500)
  * the Warp env is fed the hybrid's own numpy AHRS draws, detent phase,
    misalignment and drift sign, and the same fixed command

Randomization off (the reset's draws are then only those above); the Warp
env's rebase off (the hybrid has none).

  python analysis/gpu/phase2/hybrid.py --steps 150 --v 0.5           # open loop
  python analysis/gpu/phase2/hybrid.py --steps 250 --v 0.5 --closed  # each its own
"""
import argparse
import time
import types

import mujoco
import numpy as np
import warp as wp
import mujoco_warp as mjw

from aow_sim.control import general_env as ge
from aow_sim.control.policy import load_policy_npz
from aow_sim.gpu.env import GpuGeneralEnv
from aow_sim.gpu import ahrs as gahrs
from aow_sim.gpu import odometry as godo

CONFIG = "config/rl_general_v3.yaml"
PUSH = ("qpos", "qvel", "act", "ctrl", "qfrc_applied", "history", "qacc_warmstart")
MODEL = ("dof_frictionloss", "body_mass", "geom_friction", "actuator_forcerange",
         "actuator_ctrlrange")


def _rand_off(cfg):
    cfg["randomization"] = {**cfg["randomization"], "enabled": False}
    return cfg


class WarpPhysics:
    """mj_step / mj_forward on MJWarp for one MjData."""

    def __init__(self, model, nconmax=32, njmax=128):
        self.mjm = model
        self.m = mjw.put_model(model)
        self.d = mjw.make_data(model, nworld=1, nconmax=nconmax, njmax=njmax)

    def _push(self, model, data):
        for f in PUSH:
            getattr(self.d, f).assign(np.asarray(getattr(data, f), np.float32)[None])
        self.d.time.assign(np.array([data.time], np.float32))
        self.d.xfrc_applied.assign(np.asarray(data.xfrc_applied, np.float32)[None])
        for f in MODEL:
            arr = getattr(self.m, f)
            arr.assign(np.asarray(getattr(model, f), np.float32).reshape(arr.numpy().shape))

    def mj_step(self, model, data):
        self._push(model, data)
        mjw.step(self.m, self.d)
        mjw.get_data_into(data, model, self.d)

    def mj_forward(self, model, data):
        ws = data.qacc_warmstart.copy()
        self._push(model, data)
        mjw.forward(self.m, self.d)
        mjw.get_data_into(data, model, self.d)
        data.qacc_warmstart[:] = ws


def make_hybrid(seed):
    cfg = _rand_off(ge._load_rl_config(CONFIG))
    env = ge.GeneralEnv(None, cfg, seed=seed)
    phys = WarpPhysics(env.model)
    shim = types.SimpleNamespace(**{k: getattr(mujoco, k) for k in dir(mujoco)
                                    if not k.startswith("__")})
    shim.mj_step, shim.mj_forward = phys.mj_step, phys.mj_forward
    ge.mujoco = shim          # this process only: general_env's physics -> MJWarp

    # the gearbox's row, by (type, id)
    g = env._gearbox[0]
    def update(data, tau):
        w = g._qvel[g.dof]
        a = abs(tau)
        if -g.w_stick < w < g.w_stick:
            f0, c = g.s0, g.sc
            rows = np.flatnonzero((data.efc_type[:data.nefc] == mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF)
                                  & (data.efc_id[:data.nefc] == g.dof))
            helping = len(rows) == 0 or data.efc_force[rows[-1]] * tau >= 0.0
        else:
            f0, c = g.r0, g.rc
            helping = w * tau < 0.0
        F = (f0 + c * a) / (1.0 - c) if helping else (f0 + c * a) / (1.0 + c)
        T = g._sd[g._touch]
        g._fl[g.dof] = g._ha + g._hb * abs(T) + F
    g.update = update

    # the odometry on the integer 25-substep schedule; the AHRS draws recorded
    odo, ahrs = env._odo, env._ahrs
    every = int(round(odo.odo_dt / env.model.opt.timestep))
    st = {"n": 0}
    zs = []
    a_sample = ahrs.sample
    class Tap:
        def __init__(self, rng):
            self.rng = rng
        def standard_normal(self, *a, **k):
            z = self.rng.standard_normal(*a, **k)
            zs.append(np.array(z, np.float32))
            return z
        def __getattr__(self, name):
            return getattr(self.rng, name)
    def sample(data, dt):
        rng = ahrs.rng
        ahrs.rng = Tap(rng)
        try:
            return a_sample(data, dt)
        finally:
            ahrs.rng = rng
    ahrs.sample = sample
    def odo_update(data, dt):
        if ahrs._cache is None:
            ahrs.sample(data, odo.odo_dt)
        if dt > 0.0:
            st["n"] += 1
            if st["n"] % every == 0:
                ahrs.sample(data, odo.odo_dt)
                odo._last = odo._advance(data, odo.odo_dt)
        return odo._last
    odo.update = odo_update
    return env, st, zs


def make_warp(env, zs, v, psi_rel):
    g = GpuGeneralEnv(CONFIG, nworld=1, seed=0, cfg_override=_rand_off, rebase=False)
    queue = {"k": 0}

    def tick(active=None):
        if active is None or int(active.numpy().sum()):   # a draw is used only when sampled
            if queue["k"] < len(zs):
                g.z.assign(zs[queue["k"]][None])
            else:                    # past the hybrid's episode (a reset the hybrid did not have)
                g.z.zero_()
            queue["k"] += 1
        gahrs.launch_sample(g.c_ahrs, g.s_ahrs, g.odo_dt, g.d.sensordata, g.z, active)
        if active is None:
            godo.launch_advance(g.c_odo, g.s_odo, g.d.sensordata, g.s_ahrs.quat,
                                g.s_ahrs.gyro, g.s_ahrs.accel)
    g._tick = tick

    def on_reset():
        g.s_drv.phase.assign(np.array([env._drive.phase], np.float32))
        g.s_ahrs.accel_tilt.assign(np.asarray(env._ahrs._accel_tilt, np.float32)[None])
        g.s_ahrs.yaw_sign.assign(np.array([env._ahrs._yaw_drift_sign], np.float32))
        g.v_cmd_w.assign(np.asarray(env._v_cmd_w, np.float32)[None])
        g.psi_cmd.assign(np.array([env._psi_cmd], np.float32))
        g.next_resample.assign(np.array([10 ** 9], np.int32))
    g.on_reset = on_reset
    return g, queue


def run(policy, steps, v, psi_deg, seed, closed=False):
    pol = load_policy_npz(policy)
    env, st, zs = make_hybrid(seed)
    opts = {"v_cmd": (v, 0.0), "psi_cmd_rel": np.deg2rad(psi_deg), "difficulty": 1.0}
    oh, _ = env.reset(seed=seed, options=opts)
    g, queue = make_warp(env, zs, v, psi_deg)
    og = g.reset().numpy()[0]
    rows = [(0, np.abs(oh - og).max(), 0.0,
             np.abs(env.data.qpos - g.d.qpos.numpy()[0]).max())]
    act = wp.zeros((1, 3), dtype=float)
    t = time.perf_counter()
    for k in range(steps):
        a = np.clip(pol.mean(oh), -1, 1).astype(np.float32)
        oh, rh, th, trh, _ = env.step(a)
        ag = np.clip(pol.mean(og), -1, 1).astype(np.float32) if closed else a
        act.assign(ag[None])
        og, rg, tg, trg = (x.numpy()[0] for x in g.step(act))
        sums = run.sums
        sums["h"] += rh
        sums["g"] += rg
        if k % 10 == 0:
            qh, qg = env.data.qpos, g.d.qpos.numpy()[0]
            print(f"  step {k + 1:4d}: roll hybrid {np.degrees(np.arctan2(2*(qh[3]*qh[4]+qh[5]*qh[6]), 1-2*(qh[4]**2+qh[5]**2))):6.2f}"
                  f"  warp {np.degrees(np.arctan2(2*(qg[3]*qg[4]+qg[5]*qg[6]), 1-2*(qg[4]**2+qg[5]**2))):6.2f} deg;"
                  f"  x {qh[0]:6.3f} / {qg[0]:6.3f} m", flush=True)
        rows.append((k + 1, np.abs(oh - og).max(), abs(rh - rg),
                     np.abs(env.data.qpos - g.d.qpos.numpy()[0]).max()))
        if th or tg:
            print(f"  episode ended at step {k + 1}: hybrid fell {bool(th)}, warp fell {bool(tg)}")
            break
    return np.array(rows), time.perf_counter() - t, queue["k"], len(zs)


run.sums = {"h": 0.0, "g": 0.0}


def nudge(policy, steps, v, psi_deg, seed, eps=1e-6):
    """The yardstick: the Warp env against itself nudged eps rad on input a."""
    pol = load_policy_npz(policy)
    envs = [GpuGeneralEnv(CONFIG, nworld=1, seed=seed, cfg_override=_rand_off, rebase=False)
            for _ in range(2)]
    obs = [e.reset().numpy()[0] for e in envs]
    for e in envs:
        e.v_cmd_w.assign(np.array([[v * np.cos(np.deg2rad(psi_deg)),
                                    v * np.sin(np.deg2rad(psi_deg))]], np.float32))
        e.psi_cmd.assign(np.array([np.deg2rad(psi_deg)], np.float32))
        e.next_resample.assign(np.array([10 ** 9], np.int32))
    q = envs[1].d.qpos.numpy()
    q[0, envs[1].c_rb.qa] += eps
    envs[1].d.qpos.assign(q)
    act = wp.zeros((1, 3), dtype=float)
    rows = []
    for k in range(steps):
        act.assign(np.clip(pol.mean(obs[0]), -1, 1).astype(np.float32)[None])
        out = [e.step(act) for e in envs]
        obs = [o[0].numpy()[0] for o in out]
        rows.append((k + 1, np.abs(obs[0] - obs[1]).max(),
                     abs(float(out[0][1].numpy()[0] - out[1][1].numpy()[0])),
                     np.abs(envs[0].d.qpos.numpy() - envs[1].d.qpos.numpy()).max()))
    return np.array(rows)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="moves/archive/general_rl_v3_s0.npz")
    ap.add_argument("--steps", type=int, default=150)
    ap.add_argument("--v", type=float, default=0.5)
    ap.add_argument("--psi", type=float, default=0.0, help="heading step [deg]")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--closed", action="store_true",
                    help="each env flown by the policy on its OWN obs (else the Warp env "
                         "replays the hybrid's actions open loop, and falls once chaos "
                         "has parted the two states)")
    a = ap.parse_args()
    wp.config.quiet = True
    wp.init()
    rows, dt, used, drawn = run(a.policy, a.steps, a.v, a.psi, a.seed, a.closed)
    if a.closed:
        n = len(rows) - 1
        print(f"closed loop, {n} steps: mean reward/step hybrid {run.sums['h'] / n:.4f}, "
              f"warp {run.sums['g'] / n:.4f}")
    yard = nudge(a.policy, len(rows) - 1, a.v, a.psi, a.seed)
    print(f"hybrid vs Warp env, command v {a.v} psi {a.psi} deg, {len(rows) - 1} steps "
          f"({dt:.0f} s); AHRS draws used {used} of {drawn}")
    print(f"{'step':>5s} {'t [s]':>6s} {'obs diff':>9s} {'reward diff':>12s} {'qpos diff':>10s}"
          f" | nudge 1e-6: {'obs':>8s} {'qpos':>8s}")
    marks = [0, 1, 2, 5, 10, 25, 50, 100, 150, 250, 500, 750]
    for k in [m for m in marks if m < len(rows)]:
        r = rows[k]
        y = yard[k - 1] if k >= 1 else (0, 0.0, 0.0, 0.0)
        print(f"{int(r[0]):5d} {r[0] * 0.02:6.2f} {r[1]:9.2g} {r[2]:12.2g} {r[3]:10.2g}"
              f" | {' ':11s} {y[1]:8.2g} {y[3]:8.2g}")
