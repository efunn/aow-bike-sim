"""`sim_odometry.SimOdometry._advance` for encoder "counts", mode "front", as
a Warp kernel: quantise each input shaft to the servo's encoder counts,
difference them across the single-turn wrap (`hw.dynamixel.pos_delta`),
smooth the rate (`hw.rate_filter.RateFilter`), quantise the steer, then
`hw.odometry.VelocityEstimator.update` -- hub kinematics for v_lon, the front
wheel's rolling constraint for v_lat, coasting on the accelerometer where the
steer makes the constraint blind. One call per 100 Hz tick, after the AHRS
kernel, reading its cached outputs (the Pi's order within one sense loop).

With `steer_rate_on`, the same tick also forms the STEER RATE the policy
observes as `sim_odometry.SimSteerRate` "counts" does (the Pi's bike_bus
path): steer counts through the gear ratio, NOT unwrapped (extended
position), differenced, the same RateFilter -- sharing the wheels' first-tick
and tap-count state, since all three start on the same tick.
"""
from __future__ import annotations

import math

import numpy as np
import warp as wp

from ..hw.dynamixel import POS_WRAP
from ..hw.odometry import COS_MIN, GRAVITY
from ..sim_odometry import XC330_COUNTS_PER_RAD

MAX_TAPS = 8


@wp.struct
class OdoConst:
    adr_a: int            # sensordata: input_a_pos
    adr_b: int
    adr_steer: int
    belt: float
    cpr: float            # counts per servo radian
    wrap: int
    steer_q: float        # steer quantum: counts per steer radian
    mix_a: float
    mix_b: float
    r_wheel: float
    wheelbase: float
    cos_rake: float
    cos_min: float
    gravity: float
    dt: float
    n_taps: int
    w0: float             # RateFilter weights, newest first (n_taps <= 2 here)
    w1: float
    steer_rate_on: int    # 1: also form the Pi's steer rate (SimSteerRate "counts")


class OdoState:
    def __init__(self, nworld: int, device=None):
        z = lambda *s, dt=float: wp.zeros(s, dtype=dt, device=device)
        self.prev = z(nworld, 2, dt=int)
        self.prev_ok = z(nworld, dt=int)
        self.rbuf = z(nworld, 2, MAX_TAPS)
        self.rcount = z(nworld, dt=int)
        self.v_lon = z(nworld)
        self.v_lat = z(nworld)
        self.sprev = z(nworld, dt=int)          # steer counts, last tick
        self.sbuf = z(nworld, MAX_TAPS)         # steer raw rates, newest first
        self.steer_rate = z(nworld)             # what obs[5] reads when on


def const_from(odo, steer_rate=None) -> OdoConst:
    if odo.encoder != "counts" or odo.fusion != "front" or odo.channels != "both":
        raise NotImplementedError("the port covers counts / front / both")
    f = odo._filt["a"]
    if f.n_taps > 2:
        raise NotImplementedError("RateFilter with more than 2 taps")
    est = odo.est
    c = OdoConst()
    c.adr_a = odo.adr["input_a_pos"][0]
    c.adr_b = odo.adr["input_b_pos"][0]
    c.adr_steer = odo.adr["steer_pos"][0]
    c.belt = float(est.belt_ratio)
    c.cpr = float(XC330_COUNTS_PER_RAD)
    c.wrap = int(POS_WRAP)
    c.steer_q = float(XC330_COUNTS_PER_RAD * float(
        odo.params["bike"]["steering"]["gear_ratio"]))
    c.mix_a, c.mix_b = float(est.mix_hub_a), float(est.mix_hub_b)
    c.r_wheel = float(est.r_wheel)
    c.wheelbase = float(est.wheelbase)
    c.cos_rake = float(np.cos(est.rake))
    c.cos_min = float(COS_MIN)
    c.gravity = float(GRAVITY)
    c.dt = float(odo.odo_dt)
    c.n_taps = int(f.n_taps)
    w = list(f.weights) + [0.0]
    c.w0, c.w1 = float(w[0]), float(w[1])
    if est.lon_smooth != 1.0:
        raise NotImplementedError("lon_smooth != 1")
    c.steer_rate_on = 0
    if steer_rate is not None and steer_rate.encoder == "counts":
        sf = steer_rate.filt
        if (sf.n_taps != f.n_taps or list(sf.weights) != list(f.weights)
                or steer_rate.odo_dt != odo.odo_dt
                or steer_rate.ratio * steer_rate.cpr != c.steer_q):
            raise NotImplementedError("the steer rate's filter, clock or quantum "
                                      "differs from the odometry's")
        c.steer_rate_on = 1
    return c


@wp.func
def _shaft_rate(c: OdoConst, world: int, k: int, angle: float,
                prev: wp.array2d(dtype=int), ok: int,
                rbuf: wp.array3d(dtype=float), n: int):
    """Counts, delta, raw rate pushed into the filter; returns the filtered
    rate (and holds the last one on the first tick, as `peek`)."""
    counts = int(wp.round(angle / c.belt * c.cpr))
    p = prev[world, k]
    prev[world, k] = counts
    if ok == 0:
        if n == 0:
            return 0.0
        return rbuf[world, k, 0]       # not reached: n is 0 on the first tick
    half = c.wrap // 2
    r = (counts - p + half) % c.wrap      # C's %: make it Python's (floored)
    if r < 0:
        r += c.wrap
    d = r - half
    raw = (float(d) / c.cpr) / c.dt
    if c.n_taps > 1:
        rbuf[world, k, 1] = rbuf[world, k, 0]
    rbuf[world, k, 0] = raw
    m = wp.min(n + 1, c.n_taps)
    if m == 1:
        return raw
    return (c.w0 * rbuf[world, k, 0] + c.w1 * rbuf[world, k, 1]) / (c.w0 + c.w1)


@wp.func
def _steer_rate(c: OdoConst, world: int, angle: float, sprev: wp.array(dtype=int),
                ok: int, sbuf: wp.array2d(dtype=float), n: int):
    """SimSteerRate._advance: counts per steer radian (servo counts x gear
    ratio), no wrap, rate in steer rad/s through the shared filter."""
    counts = int(wp.round(angle * c.steer_q))
    p = sprev[world]
    sprev[world] = counts
    if ok == 0:
        return 0.0                       # a fresh filter's peek
    raw = (float(counts - p) / c.steer_q) / c.dt
    if c.n_taps > 1:
        sbuf[world, 1] = sbuf[world, 0]
    sbuf[world, 0] = raw
    m = wp.min(n + 1, c.n_taps)
    if m == 1:
        return raw
    return (c.w0 * sbuf[world, 0] + c.w1 * sbuf[world, 1]) / (c.w0 + c.w1)


@wp.kernel
def advance(c: OdoConst, sensordata: wp.array2d(dtype=float),
            ahrs_quat: wp.array2d(dtype=float), ahrs_gyro: wp.array2d(dtype=float),
            ahrs_accel: wp.array2d(dtype=float),
            prev: wp.array2d(dtype=int), prev_ok: wp.array(dtype=int),
            rbuf: wp.array3d(dtype=float), rcount: wp.array(dtype=int),
            v_lon: wp.array(dtype=float), v_lat: wp.array(dtype=float),
            sprev: wp.array(dtype=int), sbuf: wp.array2d(dtype=float),
            steer_rate: wp.array(dtype=float)):
    world = wp.tid()
    # roll, pitch from the AHRS quaternion (wxyz)
    qw = ahrs_quat[world, 0]
    qx = ahrs_quat[world, 1]
    qy = ahrs_quat[world, 2]
    qz = ahrs_quat[world, 3]
    roll = wp.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = wp.asin(wp.clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0))

    ok = prev_ok[world]
    n = rcount[world]
    wa = _shaft_rate(c, world, 0, sensordata[world, c.adr_a], prev, ok, rbuf, n)
    wb = _shaft_rate(c, world, 1, sensordata[world, c.adr_b], prev, ok, rbuf, n)
    if c.steer_rate_on == 1:
        steer_rate[world] = _steer_rate(c, world, sensordata[world, c.adr_steer],
                                        sprev, ok, sbuf, n)
    if ok == 1:
        rcount[world] = n + 1
    prev_ok[world] = 1

    steer = sensordata[world, c.adr_steer]
    steer = wp.round(steer * c.steer_q) / c.steer_q
    yaw_rate = ahrs_gyro[world, 2]

    # VelocityEstimator.update
    a_lat = ahrs_accel[world, 1] - c.gravity * wp.sin(roll) * wp.cos(pitch)
    a_lat -= v_lon[world] * yaw_rate
    vl = (c.mix_a * wa * c.belt + c.mix_b * wb * c.belt) * c.r_wheel
    v_lon[world] = vl
    theta = wp.atan2(c.cos_rake * wp.sin(steer), wp.cos(steer))
    cs = wp.cos(theta)
    conf = wp.clamp((wp.abs(cs) - c.cos_min) / (1.0 - c.cos_min), 0.0, 1.0)
    conf = conf * conf
    v_lat_c = float(0.0)
    if wp.abs(cs) < 1.0e-6:
        conf = 0.0
    else:
        v_lat_c = vl * (wp.sin(theta) / cs) - yaw_rate * c.wheelbase
    pred = v_lat[world] + a_lat * c.dt
    v_lat[world] = pred + conf * (v_lat_c - pred)


def launch_advance(c: OdoConst, s: OdoState, sensordata, quat, gyro, accel):
    wp.launch(advance, dim=sensordata.shape[0], inputs=[
        c, sensordata, quat, gyro, accel, s.prev, s.prev_ok, s.rbuf, s.rcount,
        s.v_lon, s.v_lat, s.sprev, s.sbuf, s.steer_rate], device=sensordata.device)
