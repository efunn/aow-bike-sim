"""`sim_ahrs.SimAhrs.sample` at level "tm151_filter", as a Warp kernel: the
TM151 as its own complementary filter on its own corrupted gyro and
accelerometer, plus the orientation wander, the yaw drift, the gyro bias and
the white noise. One thread per world, one call per 100 Hz tick.

The 12 normal draws per sample (orientation, bias, gyro, accelerometer --
numpy's order) come in as an array, so the check can feed numpy's own and
the env can feed Warp's. Quaternions are wxyz vec4s, as MuJoCo's.
"""
from __future__ import annotations

import math

import numpy as np
import warp as wp

from .. import sim_ahrs as sa


@wp.struct
class AhrsConst:
    adr_quat: int
    adr_gyro: int
    adr_accel: int
    offset: wp.vec3          # accelerometer offset from the site [m]
    sigma_g: float           # gyro white noise [rad/s]
    bias_sigma: float        # gyro bias stability [rad/s]
    k_g: float               # gyro g-sensitivity [rad/s per g]
    sigma_a: float           # accelerometer white noise [m/s^2]
    drift_rate: float        # yaw drift [rad/s]
    tau_bias: float
    gravity: float
    rate_smooth_s: float
    log_rate_rest: float
    log_rate_motion: float
    log_tau_rest: float
    acc_hold_s: float
    acc_d0: float            # 0: no accelerometer weighting


class AhrsState:
    """Per world. `rms` (3), `tau_orient` and `tau_motion` are the error
    PARAMETERS (set_error_params); the rest is the process state."""

    def __init__(self, nworld: int, device=None):
        z = lambda *s, dt=float: wp.zeros(s, dtype=dt, device=device)
        self.orient_err = z(nworld, 3)
        self.gyro_bias = z(nworld, 3)
        self.yaw_drift = z(nworld)
        self.yaw_sign = z(nworld)
        self.accel_tilt = z(nworld, 3)
        self.u = z(nworld, 3)
        self.u_ok = z(nworld, dt=int)
        self.w_prev = z(nworld, 3)
        self.w_prev_ok = z(nworld, dt=int)
        self.rate_s = z(nworld)
        self.acc_dev = z(nworld)
        self.rms = z(nworld, 3)
        self.tau_orient = z(nworld)
        self.tau_motion = z(nworld)
        self.quat = z(nworld, 4)        # the cached reading
        self.gyro = z(nworld, 3)
        self.accel = z(nworld, 3)
        self.all_on = wp.full(nworld, 1, dtype=int, device=device)


def const_from(ahrs) -> AhrsConst:
    if ahrs.level != "tm151_filter" or ahrs.channels != "both":
        raise NotImplementedError("the port covers tm151_filter, both channels")
    if sa.FILTER_ACC_GATE is not None:
        raise NotImplementedError("the hard accelerometer gate is not ported")
    c = AhrsConst()
    c.adr_quat = ahrs.adr["ahrs_quat"][0]
    c.adr_gyro = ahrs.adr["ahrs_gyro"][0]
    c.adr_accel = ahrs.adr["ahrs_accel"][0]
    c.offset = wp.vec3(*ahrs.accel_offset_m.tolist())
    c.sigma_g = float(np.deg2rad(sa.GYRO_NOISE_PP_DPS * sa.PP_TO_SIGMA))
    c.bias_sigma = float(np.deg2rad(sa.GYRO_BIAS_STABILITY_DPH / 3600.0))
    c.k_g = float(np.deg2rad(sa.GYRO_G_SENS_DPS_PER_G))
    c.sigma_a = float(sa.ACCEL_NOISE_PP_MG * 1e-3 * sa.GRAVITY * sa.PP_TO_SIGMA)
    c.drift_rate = float(np.deg2rad(sa.YAW_DRIFT_DEG_PER_S[ahrs.level]))
    c.tau_bias = float(ahrs.tau_bias_s)
    c.gravity = float(sa.GRAVITY)
    c.rate_smooth_s = float(sa.FILTER_RATE_SMOOTH_S)
    c.log_rate_rest = math.log(sa.FILTER_RATE_REST_DPS)
    c.log_rate_motion = math.log(sa.FILTER_RATE_MOTION_DPS)
    c.log_tau_rest = math.log(sa.FILTER_TAU_REST_S)
    c.acc_hold_s = float(sa.FILTER_ACC_HOLD_S)
    c.acc_d0 = float(sa.FILTER_ACC_D0_G or 0.0)
    return c


def load_state(s: AhrsState, world: int, py: dict) -> None:
    """One world's state from a SimAhrs's plain-number state (record_reference)."""
    def put(arr, value):
        a = arr.numpy()
        a[world] = value
        arr.assign(a)
    put(s.orient_err, py["_orient_err"])
    put(s.gyro_bias, py["_gyro_bias"])
    put(s.yaw_drift, py["_yaw_drift"])
    put(s.yaw_sign, py["_yaw_drift_sign"])
    put(s.accel_tilt, py["_accel_tilt"])
    put(s.u_ok, 0 if py["_u"] is None else 1)
    if py["_u"] is not None:
        put(s.u, py["_u"])
    put(s.w_prev_ok, 0 if py["_w_prev"] is None else 1)
    if py["_w_prev"] is not None:
        put(s.w_prev, py["_w_prev"])
    put(s.rate_s, py["_rate_s"])
    put(s.acc_dev, py["_acc_dev"])
    put(s.rms, np.deg2rad(py["orient_rms_deg"]))
    put(s.tau_orient, py["tau_orient_s"])
    put(s.tau_motion, py["filter_tau_motion_s"])


@wp.func
def qmul(a: wp.vec4, b: wp.vec4):
    return wp.vec4(a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                   a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                   a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                   a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


@wp.func
def qnormalize(q: wp.vec4):
    return q / wp.length(q)


@wp.kernel
def sample(c: AhrsConst, dt: float, sensordata: wp.array2d(dtype=float),
           z: wp.array2d(dtype=float),
           orient_err: wp.array2d(dtype=float), gyro_bias: wp.array2d(dtype=float),
           yaw_drift: wp.array(dtype=float), yaw_sign: wp.array(dtype=float),
           accel_tilt: wp.array2d(dtype=float), u_arr: wp.array2d(dtype=float),
           u_ok: wp.array(dtype=int), w_prev: wp.array2d(dtype=float),
           w_prev_ok: wp.array(dtype=int), rate_s: wp.array(dtype=float),
           acc_dev: wp.array(dtype=float), rms: wp.array2d(dtype=float),
           tau_orient: wp.array(dtype=float), tau_motion: wp.array(dtype=float),
           quat_out: wp.array2d(dtype=float), gyro_out: wp.array2d(dtype=float),
           accel_out: wp.array2d(dtype=float), active: wp.array(dtype=int)):
    world = wp.tid()
    if active[world] == 0:          # the reset's priming sample: reset worlds only
        return
    quat = wp.vec4(sensordata[world, c.adr_quat], sensordata[world, c.adr_quat + 1],
                   sensordata[world, c.adr_quat + 2], sensordata[world, c.adr_quat + 3])
    gyro = wp.vec3(sensordata[world, c.adr_gyro], sensordata[world, c.adr_gyro + 1],
                   sensordata[world, c.adr_gyro + 2])
    accel = wp.vec3(sensordata[world, c.adr_accel], sensordata[world, c.adr_accel + 1],
                    sensordata[world, c.adr_accel + 2])

    # the accelerometer is ~1 cm off the site: + alpha x d + w x (w x d)
    al = wp.vec3(0.0, 0.0, 0.0)
    if w_prev_ok[world] == 1:
        al = (gyro - wp.vec3(w_prev[world, 0], w_prev[world, 1], w_prev[world, 2])) / dt
    accel = accel + wp.cross(al, c.offset) + wp.cross(gyro, wp.cross(gyro, c.offset))
    w_prev[world, 0] = gyro[0]
    w_prev[world, 1] = gyro[1]
    w_prev[world, 2] = gyro[2]
    w_prev_ok[world] = 1

    # orientation wander (Gauss-Markov)
    a_o = wp.exp(-dt / tau_orient[world])
    s_o = wp.sqrt(wp.max(0.0, 1.0 - a_o * a_o))
    oe = wp.vec3(0.0, 0.0, 0.0)
    for i in range(3):
        v = a_o * orient_err[world, i] + (s_o * rms[world, i]) * z[world, i]
        orient_err[world, i] = v
        oe[i] = v

    yd = yaw_drift[world] + c.drift_rate * dt * yaw_sign[world]
    yaw_drift[world] = yd
    h = 0.5 * yd
    q_drift = wp.vec4(wp.cos(h), 0.0, 0.0, wp.sin(h))
    q_err = qnormalize(wp.vec4(1.0, oe[0] / 2.0, oe[1] / 2.0, oe[2] / 2.0))

    # gyro: bias + g-sensitivity + white noise
    a_b = wp.exp(-dt / c.tau_bias)
    s_b = wp.sqrt(wp.max(0.0, 1.0 - a_b * a_b))
    g_out = wp.vec3(0.0, 0.0, 0.0)
    for i in range(3):
        gb = a_b * gyro_bias[world, i] + (s_b * c.bias_sigma) * z[world, 3 + i]
        gyro_bias[world, i] = gb
        g_out[i] = (gyro[i] + gb) + c.k_g * (accel[i] / c.gravity) + c.sigma_g * z[world, 6 + i]

    # accelerometer: misalignment + white noise
    tilt = wp.vec3(accel_tilt[world, 0], accel_tilt[world, 1], accel_tilt[world, 2])
    ct = wp.cross(tilt, accel)
    a_out = wp.vec3(0.0, 0.0, 0.0)
    for i in range(3):
        a_out[i] = (accel[i] + ct[i]) + c.sigma_a * z[world, 9 + i]

    # the part's own fusion: the up direction in the body frame
    qc = wp.vec4(quat[0], -quat[1], -quat[2], -quat[3])
    up4 = qmul(qmul(qc, wp.vec4(0.0, 0.0, 0.0, 1.0)), quat)
    up_true = wp.vec3(up4[1], up4[2], up4[3])
    u = wp.vec3(u_arr[world, 0], u_arr[world, 1], u_arr[world, 2])
    if u_ok[world] == 0:
        u = up_true
    rs = rate_s[world] + (dt / (c.rate_smooth_s + dt)) * (
        wp.length(g_out) * (180.0 / wp.pi) - rate_s[world])
    rate_s[world] = rs
    f = (wp.log(wp.max(rs, 1e-6)) - c.log_rate_rest) / (c.log_rate_motion - c.log_rate_rest)
    f = wp.clamp(f, 0.0, 1.0)
    tau = wp.exp(c.log_tau_rest + f * (wp.log(tau_motion[world]) - c.log_tau_rest))
    na = wp.length(a_out)
    dev = wp.abs(na / c.gravity - 1.0)
    m = dev
    if c.acc_hold_s > 0.0:
        m = wp.max(dev, acc_dev[world] * wp.exp(-dt / c.acc_hold_s))
    acc_dev[world] = m
    weight = float(1.0)
    if c.acc_d0 > 0.0:
        weight = 1.0 / (1.0 + (m / c.acc_d0) * (m / c.acc_d0))

    # turn with the gyro, then pull toward the accelerometer
    nw = wp.length(g_out)
    th = nw * dt
    p = u
    if th >= 1.0e-12:
        k = -g_out / nw
        cth = wp.cos(th)
        sth = wp.sin(th)
        p = u * cth + wp.cross(k, u) * sth + k * wp.dot(k, u) * (1.0 - cth)
    g = weight * dt / tau
    p = p + g * (a_out / na - p)
    u = p / wp.length(p)
    u_arr[world, 0] = u[0]
    u_arr[world, 1] = u[1]
    u_arr[world, 2] = u[2]
    u_ok[world] = 1

    # the attitude whose up is the filter's: truth turned by up_true -> u
    cr = wp.cross(up_true, u)
    nc = wp.length(cr)
    ang = wp.atan2(nc, wp.dot(up_true, u))
    qa = wp.vec4(1.0, 0.0, 0.0, 0.0)
    if ang > 1.0e-12:
        rv = -(cr / nc) * ang
        t = wp.length(rv)
        if t >= 1.0e-12:
            sh = wp.sin(t / 2.0)
            qa = wp.vec4(wp.cos(t / 2.0), sh * rv[0] / t, sh * rv[1] / t, sh * rv[2] / t)
    q_out = qnormalize(qmul(q_drift, qmul(q_err, qmul(quat, qa))))

    for i in range(4):
        quat_out[world, i] = q_out[i]
    for i in range(3):
        gyro_out[world, i] = g_out[i]
        accel_out[world, i] = a_out[i]


def launch_sample(c: AhrsConst, s: AhrsState, dt: float, sensordata, z, active=None):
    """`active`: an int per world, 0 to skip it; None runs every world."""
    if active is None:
        active = s.all_on
    wp.launch(sample, dim=sensordata.shape[0], inputs=[
        c, float(dt), sensordata, z, s.orient_err, s.gyro_bias, s.yaw_drift,
        s.yaw_sign, s.accel_tilt, s.u, s.u_ok, s.w_prev, s.w_prev_ok, s.rate_s,
        s.acc_dev, s.rms, s.tau_orient, s.tau_motion, s.quat, s.gyro, s.accel,
        active], device=sensordata.device)


@wp.kernel
def draw_z(rng: wp.array(dtype=wp.uint32), z: wp.array2d(dtype=float)):
    """The 12 standard normals a sample consumes, from each world's generator."""
    world = wp.tid()
    s = rng[world]
    for i in range(12):
        z[world, i] = wp.randn(s)
    rng[world] = s
