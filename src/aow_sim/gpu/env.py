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

from contextlib import nullcontext

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
                 rebase: bool = True, shared_curriculum_envs: int = 0,
                 curriculum_lead: float = 0.0):
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
        # The curriculum, PER WORLD by default (GeneralEnv's rule: +diff_step
        # on each good episode, in that world). Its pace in samples scales
        # with the world count: reaching 1.0 takes nworld x (1/diff_step)
        # good episodes -- 1600 for SB3's 32 envs, ~205k (>= 154M steps) at
        # 4096 worlds. shared_curriculum_envs = K > 0 instead keeps ONE
        # difficulty for all worlds, raised by diff_step / K per good episode
        # anywhere: the MEAN difficulty then moves per good episode exactly
        # as K per-world envs' mean does (K = 32: v3's SB3 run), at any
        # nworld. Each increment is still earned at the current level. A
        # world picks the shared value up at its next reset, as a GeneralEnv
        # picks up its own at episode end. (2026-10-15, A40 run 2: per world
        # at 4096, difficulty ~0.013 after 20M steps; v3_s0 was at 1.0 by 5M.)
        #
        # curriculum_lead = L > 0 (shared only) counts a good episode only if
        # it was PLAYED within L of the current shared value. An episode is
        # credited at its end, 15 s after its world took the difficulty -- at
        # 4096 worlds ~3M steps of lag, against ~24k for SB3's 32 envs -- so
        # without it the value runs ahead of what any bike has shown: the
        # 2026-10-15 next run went 0.14 -> 1.0 in 6.5M steps (~3200 gradient
        # steps), and its 13M checkpoint held still while every one after the
        # jump floored the hub. With L the value can rise at most ~L per
        # episode length of training. 0 = off (the 10-15 runs' rule).
        self.shared_k = int(shared_curriculum_envs)
        self.cur_lead = float(curriculum_lead)
        if self.cur_lead > 0 and self.shared_k <= 0:
            raise ValueError("curriculum_lead needs the shared curriculum")
        self.diff_shared = wp.full(1, float(env._diff), dtype=float, device=device)
        # the value as of the last step's end: the lead's cutoff, fixed for a
        # whole launch (shared[0] itself moves under the atomic adds)
        self.diff_snap = wp.full(1, float(env._diff), dtype=float, device=device)
        self.c_cmd_step = self.c_cmd
        if self.shared_k > 0:
            if not env.cur_on:
                raise ValueError("shared_curriculum_envs with the curriculum disabled")
            self.c_cmd_step = gcmd.const_from(env)
            self.c_cmd_step.cur_on = 0        # after_step must not advance per world
        self.v_cmd_w = z(n, 2)
        self.psi_cmd = z(n)
        self.obs = z(n, gobs.OBS_DIM)
        self.reward = z(n)
        self.terminated = z(n, dt=int)
        self.truncated = z(n, dt=int)
        self.done = z(n, dt=int)
        self.action = z(n, 3)
        self.shifts = z(n, 2, dt=int)
        # Episode summaries, written at the step an episode ends (before the
        # reset inside the step wipes the sums), for a trainer's logs:
        # ep_end 1 on that step only; last_* the finished episode's return,
        # length in policy steps, tracking score (GeneralEnv's `track`),
        # fall, and the world's curriculum difficulty.
        self.ep_ret = z(n)
        self.ep_end = z(n, dt=int)
        self.last_ret = z(n)
        self.last_len = z(n, dt=int)
        self.last_track = z(n)
        self.last_fell = z(n, dt=int)
        self.last_diff = z(n)
        self.guard = grst.ForwardGuard(self.d)
        # Called after the reset kernel, before the forward and the AHRS
        # prime: a test harness overwrites draws here (analysis/gpu/phase2/
        # hybrid.py). None in training.
        self.on_reset = None
        # an MjwScratch (scratch.py) shares one set of MJWarp working memory
        # between all MJWarp calls -- export_graph sets it for the APIC
        # capture, which would otherwise keep every call's own (12 GB at 16k)
        self.scratch = None

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
            d.qfrc_applied, d.xfrc_applied, d.geom_xmat,
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
        with self._mjw_call():
            mjw.forward(self.m, self.d)
        self.guard.restore(self.done)
        self._tick(active=self.done)                  # the reset's priming sample
        wp.launch(gobs.reset_obs, self.nworld, [
            self.c_obs, self.done, d.qpos, d.qvel, a.quat, a.gyro, o.v_lon, o.v_lat,
            self.v_cmd_w, self.psi_cmd, self.psi, self.obs], device=self.device)

    def _mjw_call(self):
        return self.scratch.call() if self.scratch is not None else nullcontext()

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
            ggb.launch_pre_step(self.c_gb, d.qpos, d.qvel, d.qfrc_actuator, d.sensordata,
                                d.efc.type, d.efc.id, d.efc.force, d.nefc,
                                m.dof_frictionloss)
            with self._mjw_call():
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
            self.c_cmd_step, self.terminated, self.truncated, self.step_i, self.psi,
            self.track_sum, self.track_n, self.next_resample, self.diff, self.v_cmd_w,
            self.psi_cmd, self.rng], device=self.device)
        if self.shared_k > 0:
            wp.launch(_advance_shared, self.nworld, [
                self.terminated, self.truncated, self.track_sum, self.track_n,
                float(self.c_cmd.diff_thresh), float(self.c_cmd.diff_step) / self.shared_k,
                self.diff, self.cur_lead, self.diff_snap, self.diff_shared],
                device=self.device)
        wp.launch(_done, self.nworld, [self.terminated, self.truncated, self.done],
                  device=self.device)
        wp.launch(_episode_stats, self.nworld, [
            self.done, self.terminated, self.reward, self.step_i, self.track_sum,
            self.track_n, self.diff, self.ep_ret, self.ep_end, self.last_ret,
            self.last_len, self.last_track, self.last_fell, self.last_diff],
            device=self.device)
        if self.shared_k > 0:   # after the stats: last_diff is the ENDED episode's
            wp.launch(_take_shared, self.nworld, [self.done, self.diff_shared, self.diff,
                                                  self.diff_snap],
                      device=self.device)
        self._reset_done()
        return self.obs, self.reward, self.terminated, self.truncated


@wp.kernel
def _episode_stats(done: wp.array(dtype=int), term: wp.array(dtype=int),
                   reward: wp.array(dtype=float), step: wp.array(dtype=int),
                   track_sum: wp.array(dtype=float), track_n: wp.array(dtype=int),
                   diff: wp.array(dtype=float), ep_ret: wp.array(dtype=float),
                   ep_end: wp.array(dtype=int), last_ret: wp.array(dtype=float),
                   last_len: wp.array(dtype=int), last_track: wp.array(dtype=float),
                   last_fell: wp.array(dtype=int), last_diff: wp.array(dtype=float)):
    w = wp.tid()
    r = ep_ret[w] + reward[w]
    ep_end[w] = done[w]
    if done[w] == 1:
        last_ret[w] = r
        last_len[w] = step[w]
        last_track[w] = track_sum[w] / float(wp.max(1, track_n[w]))
        last_fell[w] = term[w]
        last_diff[w] = diff[w]
        r = 0.0
    ep_ret[w] = r


@wp.kernel
def _advance_shared(term: wp.array(dtype=int), trunc: wp.array(dtype=int),
                    track_sum: wp.array(dtype=float), track_n: wp.array(dtype=int),
                    thresh: float, inc: float, diff: wp.array(dtype=float),
                    lead: float, snap: wp.array(dtype=float), shared: wp.array(dtype=float)):
    """commands.after_step's advance rule (a truncated episode whose mean
    tracking clears the threshold), adding to the ONE shared difficulty --
    with lead > 0, only from an episode played (diff[w], the difficulty its
    world took at reset) within `lead` of the value at the last step's end."""
    w = wp.tid()
    if trunc[w] == 1 and term[w] == 0:
        if track_sum[w] / float(wp.max(1, track_n[w])) > thresh:
            if lead <= 0.0 or diff[w] >= snap[0] - lead:
                wp.atomic_add(shared, 0, inc)


@wp.kernel
def _take_shared(done: wp.array(dtype=int), shared: wp.array(dtype=float),
                 diff: wp.array(dtype=float), snap: wp.array(dtype=float)):
    """Worlds about to reset take the shared difficulty, capped at 1. Every
    thread computes the same min, so world 0's write-back of the cap races
    with nobody's result."""
    w = wp.tid()
    d = wp.min(1.0, shared[0])
    if w == 0:
        shared[0] = d
        snap[0] = d
    if done[w] == 1:
        diff[w] = d


@wp.kernel
def _done(term: wp.array(dtype=int), trunc: wp.array(dtype=int), done: wp.array(dtype=int)):
    w = wp.tid()
    done[w] = 0
    if term[w] == 1 or trunc[w] == 1:
        done[w] = 1
