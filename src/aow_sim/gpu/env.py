"""GeneralEnv on MJWarp: N worlds, every hook a kernel, auto-reset inside the
step. The v3 recipe only (each piece's `const_from` refuses anything else).

    env = GpuGeneralEnv("config/rl_general_v3.yaml", nworld=4096, seed=0)
    obs = env.reset()                       # (nworld, 17), wp.array
    obs, reward, terminated, truncated = env.step(action)   # action (nworld, 3)

Built FROM a CPU GeneralEnv of the same config, so every constant, nominal
array and start state is the Python env's own (one source; see the plan's
digest note). One policy step:

  action -> 50 x [drivetrain, gearbox, mjw.step] with the AHRS + odometry
  ticking at 100 Hz -> rebase the drivetrain's angles (rebase.py) -> obs,
  reward, flags -> command resample / curriculum ->
  reset the finished worlds (randomize, pose, hook state, forward under a
  guard, AHRS prime, first obs).

`step` is plain launches, so it can be captured whole (`wp.ScopedCapture`).
"""
from __future__ import annotations

import numpy as np
import warp as wp
import mujoco_warp as mjw

from . import action as gact
from . import ahrs as gahrs
from . import commands as gcmd
from . import drivetrain as gdrv
from . import gearbox as ggb
from . import obs_reward as gobs
from . import odometry as godo
from . import rebase as grb
from . import reset as grst

BATCHED = ("body_mass", "geom_friction", "actuator_forcerange",
           "actuator_ctrlrange", "dof_frictionloss")


class GpuGeneralEnv:
    def __init__(self, config: str, nworld: int, seed: int = 0, device=None,
                 nconmax: int = 32, njmax: int = 128, cfg_override=None,
                 rebase: bool = True):
        from ..control.general_env import GeneralEnv, _load_rl_config
        cfg = _load_rl_config(config)
        if cfg_override:
            cfg = cfg_override(cfg)
        self.cpu = env = GeneralEnv(None, cfg, seed=seed)
        if env.rand["enabled"] and env.rand.get("disturb_prob", 0.0) > 0.0:
            raise NotImplementedError("disturbance pushes are not ported")
        self.nworld = n = int(nworld)
        self.device = device
        self.m = mjw.put_model(env.model, batch_sizes={k: n for k in BATCHED})
        self.d = mjw.make_data(env.model, nworld=n, nconmax=nconmax, njmax=njmax)
        self.substeps = env.substeps
        self.tick_every = int(round(env._odo.odo_dt / env.model.opt.timestep))
        if self.substeps % self.tick_every:
            raise NotImplementedError("the sense rate must divide the policy step")
        self.odo_dt = float(env._odo.odo_dt)

        self.c_act = gact.const_from(env)
        self.c_drv = gdrv.const_from(env._drive)
        self.c_gb = ggb.const_from(env._gearbox[0])
        self.c_ahrs = gahrs.const_from(env._ahrs)
        self.c_odo = godo.const_from(env._odo)
        self.c_obs = gobs.const_from(env)
        self.c_cmd = gcmd.const_from(env)
        self.c_rst = grst.const_from(env)
        self.c_rb = grb.const_from(env)
        self.rebase = bool(rebase)
        self.nom = grst.Nominal(env, device)

        z = lambda *s, dt=float: wp.zeros(s, dtype=dt, device=device)
        self.s_drv = gdrv.DriveState(n, self.c_drv.nc, self.c_drv.nm, device)
        self.s_drv.fc_in.fill_(float(env._drive.fc_in[0]))     # nominal; set_friction_scale off
        if env._drive.fc_in[0] != env._drive.fc_in[1]:
            raise NotImplementedError("per-side Coulomb friction")
        self.s_drv.supply.fill_(1.0)                               # drive_supply_randomize off
        self.s_ahrs = gahrs.AhrsState(n, device)
        self.s_odo = godo.OdoState(n, device)
        self.rng = z(n, dt=wp.uint32)
        wp.launch(gcmd.seed_rng, n, [int(seed), self.rng], device=device)
        self.z = z(n, 12)
        self.steer_target = z(n)
        self.prev_a = z(n, 3)
        self.step_i = z(n, dt=int)
        self.track_sum = z(n)
        self.track_n = z(n, dt=int)
        self.psi = z(n)
        self.raw_prev = z(n)
        self.next_resample = z(n, dt=int)
        self.diff = wp.full(n, float(env._diff), dtype=float, device=device)
        self.v_cmd_w = z(n, 2)
        self.psi_cmd = z(n)
        self.obs = z(n, gobs.OBS_DIM)
        self.reward = z(n)
        self.terminated = z(n, dt=int)
        self.truncated = z(n, dt=int)
        self.done = z(n, dt=int)
        self.action = z(n, 3)
        self.shifts = z(n, 2, dt=int)
        self.guard = grst.ForwardGuard(self.d)
        # Called after the reset kernel, before the forward and the AHRS
        # prime: a test harness overwrites draws here (analysis/gpu_phase2/
        # hybrid.py). None in training.
        self.on_reset = None

    # -- pieces ------------------------------------------------------------

    def _tick(self, active=None):
        """One 100 Hz sense tick: the AHRS samples, then the odometry reads it."""
        d = self.d
        wp.launch(gahrs.draw_z, self.nworld, [self.rng, self.z], device=self.device)
        gahrs.launch_sample(self.c_ahrs, self.s_ahrs, self.odo_dt, d.sensordata, self.z, active)
        if active is None:
            godo.launch_advance(self.c_odo, self.s_odo, d.sensordata, self.s_ahrs.quat,
                                self.s_ahrs.gyro, self.s_ahrs.accel)

    def _reset_done(self):
        d, m, s = self.d, self.m, self.s_drv
        a, o = self.s_ahrs, self.s_odo
        wp.launch(grst.reset_worlds, self.nworld, [
            self.c_rst, self.c_cmd, self.done, self.rng,
            self.nom.mass, self.nom.friction, self.nom.forcerange, self.nom.ctrlrange,
            self.nom.qpos, self.nom.history,
            m.body_mass, m.geom_friction, m.actuator_forcerange, m.actuator_ctrlrange,
            m.dof_frictionloss,
            d.qpos, d.qvel, d.act, d.history, d.qacc_warmstart, d.time, d.ctrl,
            d.qfrc_applied, d.xfrc_applied,
            self.steer_target, self.prev_a, self.step_i, self.track_sum, self.track_n,
            self.psi, self.raw_prev, self.next_resample, self.diff, self.v_cmd_w,
            self.psi_cmd,
            s.k_c, s.k_m, s.cmd, s.th, s.w, s.ref, s.wf, s.fa, s.duty, s.phase,
            s.anchor, s.detent_force,
            o.prev_ok, o.rcount, o.v_lon, o.v_lat,
            a.orient_err, a.gyro_bias, a.yaw_drift, a.yaw_sign, a.accel_tilt, a.u_ok,
            a.w_prev_ok, a.rate_s, a.acc_dev, a.rms, a.tau_orient, a.tau_motion],
            device=self.device)
        if self.on_reset is not None:
            self.on_reset()
        self.guard.save()
        mjw.forward(self.m, self.d)
        self.guard.restore(self.done)
        self._tick(active=self.done)                  # the reset's priming sample
        wp.launch(gobs.reset_obs, self.nworld, [
            self.c_obs, self.done, d.qpos, d.qvel, a.quat, a.gyro, o.v_lon, o.v_lat,
            self.v_cmd_w, self.psi_cmd, self.psi, self.obs], device=self.device)

    # -- API ---------------------------------------------------------------

    def reset(self):
        self.done.fill_(1)
        self._reset_done()
        return self.obs

    def step(self, action):
        d, m = self.d, self.m
        wp.copy(self.action, action)
        gact.launch_apply_action(self.c_act, self.action, d.qpos, self.steer_target, d.ctrl)
        for k in range(self.substeps):
            gdrv.launch_pre_step(self.c_drv, self.s_drv, d.qpos, d.qvel, d.ctrl,
                                 d.qfrc_applied)
            ggb.launch_pre_step(self.c_gb, d.qvel, d.qfrc_actuator, d.sensordata,
                                d.efc.type, d.efc.id, d.efc.force, d.nefc,
                                m.dof_frictionloss)
            mjw.step(m, d)
            if (k + 1) % self.tick_every == 0:
                self._tick()
        if self.rebase:
            wp.launch(grb.rebase, self.nworld, [
                self.c_rb, d.qpos, self.s_drv.ref, self.s_drv.th, self.s_drv.fa,
                self.s_drv.anchor, self.s_odo.prev, self.shifts], device=self.device)
        gobs.launch(self.c_obs, d.qpos, d.qvel, self.s_ahrs.quat, self.s_ahrs.gyro,
                    self.s_odo.v_lon, self.s_odo.v_lat, self.v_cmd_w, self.psi_cmd,
                    self.psi, self.raw_prev, self.action, self.prev_a, self.step_i,
                    self.track_sum, self.track_n, self.obs, self.reward,
                    self.terminated, self.truncated)
        wp.launch(gcmd.after_step, self.nworld, [
            self.c_cmd, self.terminated, self.truncated, self.step_i, self.psi,
            self.track_sum, self.track_n, self.next_resample, self.diff, self.v_cmd_w,
            self.psi_cmd, self.rng], device=self.device)
        wp.launch(_done, self.nworld, [self.terminated, self.truncated, self.done],
                  device=self.device)
        self._reset_done()
        return self.obs, self.reward, self.terminated, self.truncated


@wp.kernel
def _done(term: wp.array(dtype=int), trunc: wp.array(dtype=int), done: wp.array(dtype=int)):
    w = wp.tid()
    done[w] = 0
    if term[w] == 1 or trunc[w] == 1:
        done[w] = 1
