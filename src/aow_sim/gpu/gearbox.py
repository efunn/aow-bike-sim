"""`gearbox_friction.GearboxFriction.pre_step` for the steer, as a Warp
kernel: the XC330's load-proportional friction plus the headset's, written
into the steer dof's `dof_frictionloss` -- a per-world model field
(`put_model(batch_sizes={"dof_frictionloss": nworld})`).

One difference from the Python hook, forced by MJWarp: the Python one reads
the friction force's sign from `efc_force` at a FIXED row (MuJoCo orders its
rows by type, then by dof). MJWarp numbers rows with atomic counters
(`_friction_dof`: `wp.atomic_add(nefc_out, worldid, 1)`), so the order is not
fixed; the kernel finds the row by (type, id) instead.

THE RIGHTING CRANK (2026-10-17) uses the same kernel when it is ported: its
`base` grows with the crank's angle, base + base_q2 q^2 (the linkage's pin
friction, `righting.module.linkage_friction_*`), read from `qpos[qadr]`. Its
dof has no damping and its torque is the servo model's `qfrc_applied`, not an
actuator's, so pass that array as `qfrc_actuator`. The port of the crank
servo itself does not exist yet (gpu/reset.py refuses a crank); this only
keeps the friction from being the missing piece. `base_q2` 0 is the steer,
bit for bit as before.
"""
from __future__ import annotations

import mujoco
import warp as wp

FRICTION_DOF = int(mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF)


@wp.struct
class GearConst:
    dof: int
    qadr: int
    damp: float
    base: float
    base_q2: float
    s0: float
    sc: float
    r0: float
    rc: float
    w_stick: float
    touch: int        # sensordata address of the headset's axial force; -1: none
    ha: float
    hb: float


def const_from(g) -> GearConst:
    """The constants of a live `GearboxFriction` (the steer's, or the crank's)."""
    c = GearConst()
    c.dof = int(g.dof)
    c.qadr = int(g.qadr)
    c.damp = float(g._damp)
    c.base = float(g.base)
    c.base_q2 = float(getattr(g, "base_q2", 0.0))
    c.s0, c.sc, c.r0, c.rc = float(g.s0), float(g.sc), float(g.r0), float(g.rc)
    c.w_stick = float(g.w_stick)
    c.touch = -1 if g._touch is None else int(g._touch)
    c.ha = float(getattr(g, "_ha", 0.0))
    c.hb = float(getattr(g, "_hb", 0.0))
    return c


@wp.kernel
def pre_step(c: GearConst, qpos: wp.array2d(dtype=float),
             qvel: wp.array2d(dtype=float),
             qfrc_actuator: wp.array2d(dtype=float),
             sensordata: wp.array2d(dtype=float),
             efc_type: wp.array2d(dtype=int), efc_id: wp.array2d(dtype=int),
             efc_force: wp.array2d(dtype=float), nefc: wp.array(dtype=int),
             frictionloss: wp.array2d(dtype=float)):
    world = wp.tid()
    w = qvel[world, c.dof]
    tau = qfrc_actuator[world, c.dof] - c.damp * w
    a = wp.abs(tau)
    f0 = c.r0
    cc = c.rc
    # the friction row, found by (type, id): MJWarp's row order is not fixed
    row = int(-1)
    n = wp.min(nefc[world], efc_type.shape[1])
    for r in range(n):
        if efc_type[world, r] == FRICTION_DOF and efc_id[world, r] == c.dof:
            row = r
    helping = int(0)
    if w > -c.w_stick and w < c.w_stick:
        f0 = c.s0
        cc = c.sc
        # at rest: which way the solver's friction pushed last step; no row
        # yet counts as helping, as in the Python hook
        if row < 0:
            helping = 1
        elif efc_force[world, row] * tau >= 0.0:
            helping = 1
    elif w * tau < 0.0:                           # running: the load drives the motor
        helping = 1
    F = float(0.0)
    if helping == 1:
        F = (f0 + cc * a) / (1.0 - cc)
    else:
        F = (f0 + cc * a) / (1.0 + cc)
    if c.touch < 0:
        q = qpos[world, c.qadr]
        frictionloss[world, c.dof] = c.base + c.base_q2 * q * q + F
    else:
        frictionloss[world, c.dof] = c.ha + c.hb * wp.abs(sensordata[world, c.touch]) + F


def launch_pre_step(c: GearConst, qpos, qvel, qfrc_actuator, sensordata, efc_type,
                    efc_id, efc_force, nefc, frictionloss):
    wp.launch(pre_step, dim=qvel.shape[0], inputs=[
        c, qpos, qvel, qfrc_actuator, sensordata, efc_type, efc_id, efc_force, nefc,
        frictionloss], device=qvel.device)
