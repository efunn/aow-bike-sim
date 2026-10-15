"""`drivetrain_model.DrivetrainSim.pre_step` as a Warp kernel, one thread per
world: the XC430 velocity loop (command and measurement delays, the velocity
filter, PI with back-calculation anti-windup), the sticking Coulomb friction
on each input shaft, and the differential's detent.

The Python class is the reference (analysis/gpu_phase2 checks this against a
recorded GeneralEnv run); this mirrors it line for line, in float32.
Supports `update_every = 1` only, which is what every config sets.

State is per world and lives in `DriveState`; the constants are one
`DriveConst`, built from a live DrivetrainSim so the two cannot drift.
"""
from __future__ import annotations

import math

import numpy as np
import warp as wp


@wp.struct
class DriveConst:
    h: float
    belt: float
    stall: float
    w0: float
    kp: float
    ki: float
    vlim: float
    alpha: float
    nc: int
    nm: int
    k_in: float
    c_in: float
    period: float
    width: float
    fmax: float
    k_stick: float
    c_stick: float
    ga: float
    gb: float
    cmd_a: int        # ctrl index of drive_a (commanded input-shaft speed)
    cmd_b: int
    motor_a: int      # ctrl index of drive_a_motor (torque)
    motor_b: int
    qa: int           # qpos address of input_a_spin
    qb: int
    da: int           # dof address of input_a_spin
    db: int


class DriveState:
    """Per-world arrays. Shapes: (nworld,), (nworld, 2) or (nworld, 2, n)."""

    def __init__(self, nworld: int, nc: int, nm: int, device=None):
        z = lambda *s, dt=float: wp.zeros(s, dtype=dt, device=device)
        self.k_c = z(nworld, dt=int)
        self.k_m = z(nworld, dt=int)
        self.cmd = z(nworld, 2, nc)
        self.th = z(nworld, 2, nm)
        self.w = z(nworld, 2, nm)
        self.ref = z(nworld, 2)
        self.wf = z(nworld, 2)
        self.fa = z(nworld, 2)
        self.duty = z(nworld, 2)
        self.fc_in = z(nworld, 2)       # per-world: set_friction_scale
        self.supply = z(nworld)         # per-world: set_supply
        self.phase = z(nworld)          # per-world: drawn at reset
        self.anchor = z(nworld)
        self.detent_force = z(nworld)


def const_from(sim) -> DriveConst:
    """The constants of a live `DrivetrainSim` (servo and detent both on)."""
    if not (sim.servo and sim.detent) or sim.every != 1:
        raise NotImplementedError("the port covers servo + detent at update_every 1")
    c = DriveConst()
    for k in ("h", "belt", "stall", "w0", "kp", "ki", "vlim", "alpha", "k_in",
              "c_in", "period", "width", "fmax", "k_stick", "c_stick", "ga", "gb"):
        setattr(c, k, float(getattr(sim, k)))
    c.nc, c.nm = int(sim.nc), int(sim.nm)
    c.cmd_a, c.cmd_b = (int(i) for i in sim.cmd_id)
    c.motor_a, c.motor_b = (int(i) for i in sim.motor_id)
    c.qa, c.qb = (int(i) for i in sim.qadr)
    c.da, c.db = (int(i) for i in sim.dadr)
    if tuple(sim.qadr) != tuple(sim.in_q) or tuple(sim.dadr) != tuple(sim.in_d):
        raise ValueError("servo and friction shafts differ; the port assumes one pair")
    return c


def load_state(state: DriveState, world: int, py: dict) -> None:
    """Copy one world's state from a `DrivetrainSim`'s plain-number state
    (`vars()` after reset -- what record_reference.py stores)."""
    def put(arr, value):
        a = arr.numpy()
        a[world] = value
        arr.assign(a)
    put(state.k_c, py["_k_c"])
    put(state.k_m, py["_k_m"])
    put(state.cmd, np.array(py["_cmd"]))
    put(state.th, np.array(py["_th"]))
    put(state.w, np.array(py["_w"]))
    put(state.ref, py["_ref"])
    put(state.wf, py["_wf"])
    put(state.fa, py["_fa"])
    put(state.duty, py["duty"])
    put(state.fc_in, py["fc_in"])
    put(state.supply, py["supply"])
    put(state.phase, py["phase"])
    put(state.anchor, py["_anchor"])
    put(state.detent_force, py["detent_force"])


@wp.func
def _servo(c: DriveConst, i: int, world: int, v_cmd: float, th: float, w: float,
           s_cmd: wp.array3d(dtype=float), s_th: wp.array3d(dtype=float),
           s_w: wp.array3d(dtype=float), s_ref: wp.array2d(dtype=float),
           s_wf: wp.array2d(dtype=float), s_duty: wp.array2d(dtype=float),
           kc: int, kc1: int, km: int, km1: int, supply: float):
    v = v_cmd / c.belt
    v = wp.clamp(v, -c.vlim, c.vlim)
    s_cmd[world, i, kc] = v
    v_ref = s_cmd[world, i, kc1]
    s_th[world, i, km] = th
    s_w[world, i, km] = w
    wf = s_wf[world, i] + c.alpha * (s_w[world, i, km1] - s_wf[world, i])
    s_wf[world, i] = wf
    ref = s_ref[world, i] + v_ref * c.h
    duty = c.kp * (v_ref - wf) + c.ki * (ref - s_th[world, i, km1])
    if duty > 1.0:
        if c.ki > 0.0:
            ref -= (duty - 1.0) / c.ki
        duty = 1.0
    elif duty < -1.0:
        if c.ki > 0.0:
            ref -= (duty + 1.0) / c.ki
        duty = -1.0
    s_ref[world, i] = ref
    s_duty[world, i] = duty
    return c.stall * (supply * duty - w / c.w0) / c.belt


@wp.func
def _stick(c: DriveConst, i: int, world: int, x: float, xd: float,
           s_fa: wp.array2d(dtype=float), fc: float):
    f = -c.k_in * (x - s_fa[world, i]) - c.c_in * xd
    if f > fc or f < -fc:
        if f > 0.0:
            f = fc
        else:
            f = -fc
        s_fa[world, i] = x + f / c.k_in
    return f


@wp.kernel
def pre_step(c: DriveConst,
             qpos: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float),
             ctrl: wp.array2d(dtype=float), qfrc_applied: wp.array2d(dtype=float),
             k_c: wp.array(dtype=int), k_m: wp.array(dtype=int),
             s_cmd: wp.array3d(dtype=float), s_th: wp.array3d(dtype=float),
             s_w: wp.array3d(dtype=float), s_ref: wp.array2d(dtype=float),
             s_wf: wp.array2d(dtype=float), s_fa: wp.array2d(dtype=float),
             s_duty: wp.array2d(dtype=float), fc_in: wp.array2d(dtype=float),
             supply: wp.array(dtype=float), phase: wp.array(dtype=float),
             anchor: wp.array(dtype=float), detent_force: wp.array(dtype=float)):
    world = wp.tid()
    kc = k_c[world]
    kc1 = (kc + 1) % c.nc
    km = k_m[world]
    km1 = (km + 1) % c.nm
    sup = supply[world]
    xa = qpos[world, c.qa]
    xb = qpos[world, c.qb]
    va = qvel[world, c.da]
    vb = qvel[world, c.db]

    # the velocity loop, both shafts
    ctrl[world, c.motor_a] = _servo(c, 0, world, ctrl[world, c.cmd_a], xa / c.belt,
                                    va / c.belt, s_cmd, s_th, s_w, s_ref, s_wf,
                                    s_duty, kc, kc1, km, km1, sup)
    ctrl[world, c.motor_b] = _servo(c, 1, world, ctrl[world, c.cmd_b], xb / c.belt,
                                    vb / c.belt, s_cmd, s_th, s_w, s_ref, s_wf,
                                    s_duty, kc, kc1, km, km1, sup)
    k_c[world] = kc1
    k_m[world] = km1

    # sticking Coulomb friction on each input shaft
    fa_ = _stick(c, 0, world, xa, va, s_fa, fc_in[world, 0])
    fb_ = _stick(c, 1, world, xb, vb, s_fa, fc_in[world, 1])

    # the differential's detent
    phi = c.ga * xa + c.gb * xb
    x = phi - phase[world]
    x = x - c.period * wp.floor(x / c.period) - 0.5 * c.period   # Python's % (floored)
    q = float(0.0)
    if wp.abs(x) < 0.5 * c.width:
        cs = wp.cos(wp.pi * x / c.width)
        limit = c.fmax * cs * cs
        vphi = c.ga * va + c.gb * vb
        q = -c.k_stick * (phi - anchor[world]) - c.c_stick * vphi
        if q > limit or q < -limit:
            if q > 0.0:
                q = limit
            else:
                q = -limit
            anchor[world] = phi + q / c.k_stick
    else:
        anchor[world] = phi
    detent_force[world] = q
    qfrc_applied[world, c.da] = fa_ + c.ga * q
    qfrc_applied[world, c.db] = fb_ + c.gb * q


def launch_pre_step(c: DriveConst, s: DriveState, qpos, qvel, ctrl, qfrc_applied):
    wp.launch(pre_step, dim=qpos.shape[0], inputs=[
        c, qpos, qvel, ctrl, qfrc_applied, s.k_c, s.k_m, s.cmd, s.th, s.w, s.ref,
        s.wf, s.fa, s.duty, s.fc_in, s.supply, s.phase, s.anchor, s.detent_force],
        device=qpos.device)
