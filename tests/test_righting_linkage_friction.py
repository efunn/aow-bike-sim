"""The righting linkage's own friction on the crank: bike_params
`righting.module.linkage_friction_*`, applied by gearbox_friction.GearboxFriction
(`base_q2`) through righting_servo.CurrentBasedPositionServo.attach, and the
same term in the Warp kernel (gpu/gearbox.py).

Pins the IMPLEMENTATION: every expectation is derived from the params, so a
re-fit leaves these green. Whether the curve matches the bench is
analysis/mechanism/righting_sweep.py and the parameter's own note (measured 2026-10-17).

The stroke test is the point of the model: at a Goal Current sized, from the
params alone, to carry the linkage only to ~60 deg, the frictionless crank
reaches a 120 deg goal and the one with the linkage's friction stops short.
"""

import math

import mujoco
import numpy as np
import pytest

from aow_sim import righting_servo as rs
from aow_sim.build_model import build_model, load_params

pytestmark = pytest.mark.righting


@pytest.fixture(scope="module")
def params():
    return load_params()


def _f(params, theta):
    m = params["righting"]["module"]
    return float(m["linkage_friction_nm"]) + float(m["linkage_friction_q2"]) * theta ** 2


def _floating(params, linkage_friction=True, goal_current=None):
    m = build_model(params, righting=True, swing_linkage=True)
    m.opt.gravity[:] = 0.0
    d = mujoco.MjData(m)
    d.qpos[2] += 0.5
    mujoco.mj_forward(m, d)
    srv = rs.CurrentBasedPositionServo.attach(m, params,
                                              linkage_friction=linkage_friction)
    if goal_current is not None:
        srv.set_goal_current(goal_current)
    return m, d, srv


def test_attach_reads_the_module_curve(params):
    _, _, srv = _floating(params)
    mod = params["righting"]["module"]
    assert srv.friction.base == pytest.approx(float(mod["linkage_friction_nm"]))
    assert srv.friction.base_q2 == pytest.approx(float(mod["linkage_friction_q2"]))


def test_off_is_the_gearbox_alone(params):
    _, _, srv = _floating(params, linkage_friction=False)
    assert srv.friction.base == 0.0 and srv.friction.base_q2 == 0.0


@pytest.mark.parametrize("deg", [0.0, 60.0, -100.0])
def test_limit_is_curve_plus_gearbox(params, deg):
    """Moving and driving: the dof's friction limit is f(theta) plus the
    gearbox's running (f0 + c tau)/(1 + c) -- the fixed point documented in
    gearbox_friction.py, on top of the linkage's base."""
    m, d, srv = _floating(params)
    g = srv.friction
    q = math.radians(deg)
    d.qpos[g.qadr] = q
    d.qvel[g.dof] = 1.0                       # well above stick speed
    tau = 0.2
    g.update(d, tau)
    expect = _f(params, q) + (g.r0 + g.rc * tau) / (1 + g.rc)
    assert m.dof_frictionloss[g.dof] == pytest.approx(expect, rel=1e-12)


def test_build_time_value_is_the_constant(params):
    """A loop that never calls the hook keeps base + static: the curve's
    constant term, not its angle term."""
    m, d, srv = _floating(params)
    g = srv.friction
    g.release()
    assert m.dof_frictionloss[g.dof] == pytest.approx(g.base + g.s0)


def _stroke(m, d, srv, goal, seconds):
    aid = m.actuator("swing").id
    for _ in range(int(round(seconds / m.opt.timestep))):
        d.qvel[:6] = 0.0                      # chassis held still
        d.ctrl[aid] = goal
        srv.pre_step(d)
        mujoco.mj_step(m, d)
    return math.degrees(d.qpos[srv.qadr])


def test_friction_stops_a_weak_crank_short(params):
    """Goal Current whose MOVING output just covers f(60 deg): the motor torque
    r0 + (1 + rc) f(60), from the params. Without the linkage's friction that
    current only has the gearbox to beat and carries the crank to its goal."""
    m, d, srv = _floating(params)
    g = srv.friction
    counts = srv.counts_for(g.r0 + (1 + g.rc) * _f(params, math.radians(60)))
    goal = math.radians(120)

    srv.set_goal_current(counts)
    with_f = _stroke(m, d, srv, goal, 3.0)
    m2, d2, srv2 = _floating(params, linkage_friction=False, goal_current=counts)
    without = _stroke(m2, d2, srv2, goal, 3.0)

    assert without > 110.0, f"frictionless crank stopped at {without:.1f} deg"
    assert with_f < 90.0, f"crank with linkage friction reached {with_f:.1f} deg"


def test_warp_kernel_matches_the_python_hook(params):
    """gpu/gearbox.py against GearboxFriction.update, crank (angle term) and
    steer (none), moving both ways and at rest. Skips without Warp (the
    `aow-sim-gpu` env has it; Warp's CPU device is enough)."""
    wp = pytest.importorskip("warp")
    from aow_sim import gearbox_friction as gfr
    from aow_sim.gpu import gearbox as ggb
    wp.init()

    m = build_model(params, righting=True, swing_linkage=True)
    d = mujoco.MjData(m)
    crank = rs.CurrentBasedPositionServo.attach(m, params).friction
    steer = gfr.attach_steer(m, params)
    mujoco.mj_forward(m, d)
    for g in (crank, steer):
        if g is None:
            continue
        g.reset(d)
        for q_deg, w, tau in ((70.0, 2.0, 0.15), (-110.0, -1.5, -0.1),
                              (40.0, 2.0, -0.05), (95.0, 0.0, 0.08)):
            d.qpos[g.qadr] = math.radians(q_deg)
            d.qvel[g.dof] = w
            mujoco.mj_forward(m, d)
            qfrc = np.zeros((1, m.nv))
            qfrc[0, g.dof] = tau + g._damp * w     # the kernel subtracts damping
            g.update(d, tau)
            want = float(m.dof_frictionloss[g.dof])

            fl = wp.array(m.dof_frictionloss[None, :].copy(), dtype=float, device="cpu")
            n = max(int(d.nefc), 1)
            arr = lambda a, dt=float: wp.array(np.asarray(a)[None, :], dtype=dt, device="cpu")
            ggb.launch_pre_step(
                ggb.const_from(g), arr(d.qpos), arr(d.qvel),
                wp.array(qfrc, dtype=float, device="cpu"),
                arr(d.sensordata if m.nsensordata else np.zeros(1)),
                arr(np.resize(d.efc_type, n), int), arr(np.resize(d.efc_id, n), int),
                arr(np.resize(d.efc_force, n)), wp.array([int(d.nefc)], dtype=int, device="cpu"),
                fl)
            got = float(fl.numpy()[0, g.dof])
            assert got == pytest.approx(want, rel=1e-6, abs=1e-9), \
                (g.dof, q_deg, w, tau, got, want)
