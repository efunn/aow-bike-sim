"""The policy-step half of `GeneralEnv.step` after physics, as a Warp kernel:
the heading update (the AHRS yaw, integrated as wrapped deltas), the
observation (`general_spec.build_obs`, the sensor-fed v3 layout: AHRS
attitude and rates, odometry velocity, pitch observed), the reward (scored on
TRUTH, `extract_state`), the curriculum's tracking sum, fall and truncation.

v3's recipe only: no velocity window, no wings, no hub-idle or copper term
(const_from refuses them). Observation layout, 17 entries:
  0 roll  1 roll_rate  2 yaw_rate  3 sin 2 steer  4 cos 2 steer  5 steer_rate
  6 v_lon  7 v_lat  8 v_cmd_lon  9 v_cmd_lat  10 sin psi_err  11 cos psi_err
  12-14 prev action (the one just applied)  15 pitch  16 pitch_rate
and with `obs_style` three more, the style bit (17) and sin/cos of the front
wheel's world heading against its latch (18-19), whose reward term pays for
the wheel staying where it was when the bit came on (general_spec.wheel_err).
"""
from __future__ import annotations

import numpy as np
import warp as wp

from . import commands as gcmd

OBS_DIM = 17                  # v3; + 3 with obs_style (obs_dim_for)


@wp.struct
class RewConst:
    w_vel: float
    w_head: float
    w_alive: float
    w_upright: float
    w_pitch: float
    w_effort: float
    ws0: float
    ws1: float
    ws2: float
    sigma_v: float
    sigma_psi: float
    penalty_fall: float
    fall: float
    max_steps: int
    steer_q: int
    steer_d: int
    steer_rate_on: int    # obs[5] from the odometry's Pi-style steer rate
    style_on: int         # obs[17:20] the style bit and wheel error; w_style paid
    w_style: float
    sigma_style: float    # [rad]
    cos_rake: float


def const_from(env) -> RewConst:
    if (env.vel_window_s or env.zero_lat or env.wings or env.swing or env.w_hub_idle
            or env.w_copper or not env.obs_pitch
            or env.obs_dim != OBS_DIM + 3 * int(env.obs_style)
            or env.wing_touch_fails):
        raise NotImplementedError("the port covers v3's observation and reward")
    rw = env.rw
    c = RewConst()
    c.w_vel, c.w_head, c.w_alive = float(rw["w_vel"]), float(rw["w_head"]), float(rw["w_alive"])
    c.w_upright = float(rw["w_upright"])
    c.w_pitch = float(rw.get("w_pitch", 0.0))
    c.w_effort = float(rw["w_effort"])
    c.ws0, c.ws1, c.ws2 = (float(x) for x in env.w_smooth)
    c.sigma_v, c.sigma_psi = float(env.sigma_v), float(env.sigma_psi)
    c.penalty_fall = float(rw["penalty_fall"])
    c.fall = float(env.fall)
    c.max_steps = int(env.max_steps)
    c.steer_q, c.steer_d = int(env._sj), int(env._sd)
    c.steer_rate_on = int(env._steer_rate is not None)
    c.style_on = int(env.obs_style)
    c.w_style, c.sigma_style = float(env.w_style), float(env.sigma_style)
    c.cos_rake = float(np.cos(env._rake))
    return c


@wp.func
def wrap_pi(x: float):
    return wp.atan2(wp.sin(x), wp.cos(x))


@wp.kernel
def obs_reward(c: RewConst, qpos: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float),
               ahrs_quat: wp.array2d(dtype=float), ahrs_gyro: wp.array2d(dtype=float),
               odo_lon: wp.array(dtype=float), odo_lat: wp.array(dtype=float),
               steer_rate: wp.array(dtype=float),
               v_cmd_w: wp.array2d(dtype=float), psi_cmd: wp.array(dtype=float),
               style: wp.array(dtype=float), latch_obs: wp.array(dtype=float),
               latch_true: wp.array(dtype=float),
               psi: wp.array(dtype=float), raw_prev: wp.array(dtype=float),
               action: wp.array2d(dtype=float), prev_a: wp.array2d(dtype=float),
               step: wp.array(dtype=int), track_sum: wp.array(dtype=float),
               track_n: wp.array(dtype=int),
               obs: wp.array2d(dtype=float), reward: wp.array(dtype=float),
               terminated: wp.array(dtype=int), truncated: wp.array(dtype=int)):
    world = wp.tid()
    # the AHRS attitude (wxyz): roll, textbook pitch, yaw
    qw = ahrs_quat[world, 0]
    qx = ahrs_quat[world, 1]
    qy = ahrs_quat[world, 2]
    qz = ahrs_quat[world, 3]
    a_roll = wp.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    a_pitch = wp.asin(wp.clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    a_yaw = wp.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    # heading: the AHRS yaw, integrated as a wrapped delta
    d = a_yaw - raw_prev[world]
    ps = psi[world] + wp.atan2(wp.sin(d), wp.cos(d))
    psi[world] = ps
    raw_prev[world] = a_yaw
    k = step[world] + 1
    step[world] = k

    # TRUTH, for the reward (balance.extract_state)
    tw = qpos[world, 3]
    tx = qpos[world, 4]
    ty = qpos[world, 5]
    tz = qpos[world, 6]
    r21 = 2.0 * (ty * tz + tw * tx)
    r22 = 1.0 - 2.0 * (tx * tx + ty * ty)
    r20 = 2.0 * (tx * tz - tw * ty)
    r10 = 2.0 * (tx * ty + tw * tz)
    r00 = 1.0 - 2.0 * (ty * ty + tz * tz)
    roll = wp.atan2(r21, r22)
    yaw = wp.atan2(r10, r00)
    pitch = wp.asin(wp.clamp(r20, -1.0, 1.0))
    cy = wp.cos(yaw)
    sy = wp.sin(yaw)
    v_lon = cy * qvel[world, 0] + sy * qvel[world, 1]
    v_lat = -sy * qvel[world, 0] + cy * qvel[world, 1]

    # the command in the body frame of the bike's OWN heading
    cp = wp.cos(ps)
    sp = wp.sin(ps)
    v_cl = cp * v_cmd_w[world, 0] + sp * v_cmd_w[world, 1]
    v_ct = -sp * v_cmd_w[world, 0] + cp * v_cmd_w[world, 1]
    psi_err = wrap_pi(psi_cmd[world] - ps)

    steer = qpos[world, c.steer_q]
    obs[world, 0] = a_roll
    obs[world, 1] = ahrs_gyro[world, 0]
    obs[world, 2] = ahrs_gyro[world, 2]
    obs[world, 3] = wp.sin(2.0 * steer)
    obs[world, 4] = wp.cos(2.0 * steer)
    if c.steer_rate_on == 1:
        obs[world, 5] = steer_rate[world]
    else:
        obs[world, 5] = qvel[world, c.steer_d]
    obs[world, 6] = odo_lon[world]
    obs[world, 7] = odo_lat[world]
    obs[world, 8] = v_cl
    obs[world, 9] = v_ct
    obs[world, 10] = wp.sin(psi_err)
    obs[world, 11] = wp.cos(psi_err)
    # prev_action: the action just applied (GeneralEnv since 2026-10-14,
    # and drive._general_compute on the bike)
    obs[world, 12] = action[world, 0]
    obs[world, 13] = action[world, 1]
    obs[world, 14] = action[world, 2]
    obs[world, 15] = -a_pitch               # nose-up positive (extract_state's sign)
    obs[world, 16] = -ahrs_gyro[world, 1]
    if c.style_on == 1:
        e_obs = float(0.0)
        if style[world] == 1.0:
            e_obs = wrap_pi(gcmd.wheel_world(ps, steer, c.cos_rake) - latch_obs[world])
        obs[world, 17] = style[world]
        obs[world, 18] = wp.sin(e_obs)
        obs[world, 19] = wp.cos(e_obs)

    # reward
    v_err2 = (v_cl - v_lon) * (v_cl - v_lon) + (v_ct - v_lat) * (v_ct - v_lat)
    r_vel = wp.exp(-v_err2 / (c.sigma_v * c.sigma_v))
    r_head = wp.exp(-(psi_err / c.sigma_psi) * (psi_err / c.sigma_psi))
    a0 = action[world, 0]
    a1 = action[world, 1]
    a2 = action[world, 2]
    d0 = a0 - prev_a[world, 0]
    d1 = a1 - prev_a[world, 1]
    d2 = a2 - prev_a[world, 2]
    r = (c.w_vel * r_vel + c.w_head * r_head + c.w_alive
         - c.w_upright * roll * roll - c.w_pitch * pitch * pitch
         - c.w_effort * (a0 * a0 + a1 * a1 + a2 * a2)
         - (d0 * c.ws0 * d0 + d1 * c.ws1 * d1 + d2 * c.ws2 * d2))
    if c.style_on == 1 and style[world] == 1.0:
        # general_spec.wheel_err on truth: the truth yaw, the truth latch
        e = wrap_pi(gcmd.wheel_world(yaw, steer, c.cos_rake) - latch_true[world])
        r = r + c.w_style * wp.exp(-(e / c.sigma_style) * (e / c.sigma_style))
    prev_a[world, 0] = a0
    prev_a[world, 1] = a1
    prev_a[world, 2] = a2
    track_sum[world] = track_sum[world] + 0.5 * (r_vel + r_head)
    track_n[world] = track_n[world] + 1

    finite = int(1)
    for i in range(qpos.shape[1]):
        if not wp.isfinite(qpos[world, i]):
            finite = 0
    term = int(0)
    if wp.abs(roll) > c.fall or finite == 0:
        r = r - c.penalty_fall
        term = 1
    reward[world] = r
    terminated[world] = term
    trunc = int(0)
    if k >= c.max_steps:
        trunc = 1
    truncated[world] = trunc


def launch(c: RewConst, qpos, qvel, ahrs_quat, ahrs_gyro, odo_lon, odo_lat,
           steer_rate, v_cmd_w, psi_cmd, style, latch_obs, latch_true, psi, raw_prev,
           action, prev_a, step,
           track_sum, track_n, obs, reward, terminated, truncated):
    wp.launch(obs_reward, dim=qpos.shape[0], inputs=[
        c, qpos, qvel, ahrs_quat, ahrs_gyro, odo_lon, odo_lat, steer_rate, v_cmd_w, psi_cmd,
        style, latch_obs, latch_true, psi, raw_prev, action, prev_a, step, track_sum, track_n, obs, reward,
        terminated, truncated], device=qpos.device)


@wp.kernel
def reset_obs(c: RewConst, done: wp.array(dtype=int), qpos: wp.array2d(dtype=float),
              qvel: wp.array2d(dtype=float), ahrs_quat: wp.array2d(dtype=float),
              ahrs_gyro: wp.array2d(dtype=float), odo_lon: wp.array(dtype=float),
              odo_lat: wp.array(dtype=float), v_cmd_w: wp.array2d(dtype=float),
              psi_cmd: wp.array(dtype=float), style: wp.array(dtype=float),
              latch_obs: wp.array(dtype=float),
              psi: wp.array(dtype=float), obs: wp.array2d(dtype=float)):
    """The observation `reset` returns (`GeneralEnv._obs` right after the
    reset): no heading update, no reward, prev_action zero. Reset worlds only."""
    world = wp.tid()
    if done[world] == 0:
        return
    qw = ahrs_quat[world, 0]
    qx = ahrs_quat[world, 1]
    qy = ahrs_quat[world, 2]
    qz = ahrs_quat[world, 3]
    ps = psi[world]
    cp = wp.cos(ps)
    sp = wp.sin(ps)
    psi_err = wrap_pi(psi_cmd[world] - ps)
    steer = qpos[world, c.steer_q]
    obs[world, 0] = wp.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    obs[world, 1] = ahrs_gyro[world, 0]
    obs[world, 2] = ahrs_gyro[world, 2]
    obs[world, 3] = wp.sin(2.0 * steer)
    obs[world, 4] = wp.cos(2.0 * steer)
    if c.steer_rate_on == 1:
        # SimSteerRate right after its reset: a fresh filter, no tick yet.
        # The odometry's priming tick does not advance it, so its array still
        # holds the ended episode's value -- not read here.
        obs[world, 5] = 0.0
    else:
        obs[world, 5] = qvel[world, c.steer_d]
    obs[world, 6] = odo_lon[world]
    obs[world, 7] = odo_lat[world]
    obs[world, 8] = cp * v_cmd_w[world, 0] + sp * v_cmd_w[world, 1]
    obs[world, 9] = -sp * v_cmd_w[world, 0] + cp * v_cmd_w[world, 1]
    obs[world, 10] = wp.sin(psi_err)
    obs[world, 11] = wp.cos(psi_err)
    obs[world, 12] = 0.0
    obs[world, 13] = 0.0
    obs[world, 14] = 0.0
    obs[world, 15] = -wp.asin(wp.clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    obs[world, 16] = -ahrs_gyro[world, 1]
    if c.style_on == 1:
        e_obs = float(0.0)
        if style[world] == 1.0:
            e_obs = wrap_pi(gcmd.wheel_world(ps, steer, c.cos_rake) - latch_obs[world])
        obs[world, 17] = style[world]
        obs[world, 18] = wp.sin(e_obs)
        obs[world, 19] = wp.cos(e_obs)
