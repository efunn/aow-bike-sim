"""Load-proportional gearbox friction for both XC330s: src/aow_sim/gearbox_friction.py.

Pins the IMPLEMENTATION: every bound is derived from the friction constants in
bike_params, so a re-fit leaves these green. A servo holding a load must lift it
only above `(tau - f0)/(1 + c)` of motor torque's worth of load, and give way
only above `(tau + f0)/(1 - c)` -- the two fitted lines -- which is only true
if the friction row the module reads IS this dof's, and its limit reaches the
solver. Whether the constants match the bench is analysis/servo_lift_sim.py.
"""

import copy

import mujoco
import numpy as np
import pytest

from aow_sim import gearbox_friction as gf
from aow_sim import righting_servo as rs
from aow_sim.build_model import build_model, load_params

pytestmark = pytest.mark.righting

G = 9.81


@pytest.fixture(scope="module")
def params():
    return load_params()


def _hinge(motor: bool, mass=0.0, radius=0.044, gravity=True):
    """A hinge about y. With `mass`, a point load at `radius` along +x (level
    at q = 0, where its torque is largest); with `motor`, a torque actuator."""
    spec = mujoco.MjSpec()
    spec.option.timestep = 4e-4
    if not gravity:
        spec.option.gravity = [0, 0, 0]
    b = spec.worldbody.add_body(name="lever")
    b.add_joint(name="hinge", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, -1, 0])
    b.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.004, 0, 0],
               pos=[radius, 0, 0], mass=max(mass, 1e-3))
    if motor:
        a = spec.add_actuator(name="motor")
        a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a.target = "hinge"
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def _run(m, d, hooks, seconds, before=None):
    if before is not None:
        before(d)
    mujoco.mj_forward(m, d)             # the first step sees this step's forces
    q0 = float(d.qpos[0])
    for _ in range(int(round(seconds / m.opt.timestep))):
        if before is not None:
            before(d)
        for h in hooks:
            h.pre_step(d)
        mujoco.mj_step(m, d)
    return float(np.degrees(d.qpos[0] - q0))


def test_the_friction_row_is_this_dofs(params):
    """The fixed-row read relies on MuJoCo's constraint ordering; check it on
    the main model (steer), with the swing crank's servo, and on the rig."""
    from aow_sim.floor_rig import attach_servos, load_rig_cfg, place, resolve
    cases = []
    m = build_model(params, righting=True, swing_linkage=True)
    srv = rs.CurrentBasedPositionServo.attach(m, params, torque_nm=0.3)
    cases.append((m, [gf.attach_steer(m, params), srv], [srv.friction]))
    cfg = copy.deepcopy(load_rig_cfg())
    cfg["servos"] = {"roll": 300, "pitch": 300}
    cfg = resolve(cfg, params)
    m = build_model(params, rig=cfg)
    rig = list(attach_servos(m, params, cfg).values())
    cases.append((m, [gf.attach_steer(m, params), *rig], [s.friction for s in rig]))
    for m, hooks, frics in cases:
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        frics = frics + [h for h in hooks if isinstance(h, gf.GearboxFriction)]
        for _ in range(300):
            for h in hooks:
                h.pre_step(d)
            mujoco.mj_step(m, d)
            for f in frics:
                r = f.row(d)
                assert d.efc_type[r] == mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
                assert d.efc_id[r] == f.dof


@pytest.mark.parametrize("tau", [0.05, 0.2])
def test_a_fixed_torque_drives_holds_and_gives_way_where_the_lines_say(params, tau):
    """Motor torque `tau` against an external load L on a frictioned hinge:
    drives it below (tau - f0)/(1 + c), is back-driven above
    (tau + f0)/(1 - c), and holds in between."""
    c = gf.constants(params)
    f0, k = c["static_nm"], c["static_frac"]
    drive_below = (tau - f0) / (1 + k)
    give_above = (tau + f0) / (1 - k)
    for L, want in ((0.9 * drive_below, "forward"), (1.1 * drive_below, "held"),
                    (0.9 * give_above, "held"), (1.1 * give_above, "back")):
        m, d = _hinge(motor=True, mass=0.137, gravity=False)
        fr = gf.GearboxFriction(m, params, "hinge")
        d.ctrl[0] = tau

        def load(d, L=L):
            d.qfrc_applied[0] = -L
        moved = _run(m, d, [fr], 0.5, before=load)
        got = "forward" if moved > 1.0 else "back" if moved < -1.0 else "held"
        assert got == want, f"tau {tau} load {L:.4f}: {got} ({moved:+.2f} deg)"


@pytest.mark.parametrize("mass", [0.034, 0.137])
def test_the_servo_lifts_and_drops_a_lever_where_the_params_say(params, mass):
    """The whole righting model on a lever like the X330 fixture's: Goal
    Current at the params' lift line +- a few mA lifts / does not, and at the
    fall line +- a few mA drops / holds. Current law, friction and the
    position loop together."""
    srv_p = params["servos"]["xc330_t181"]
    k, i0 = srv_p["current_torque_gain"], srv_p["current_deadband"]
    c = gf.constants(params)
    L = mass * G * 0.044
    lift = i0 + ((1 + c["static_frac"]) * L + c["static_nm"]) / k
    fall = i0 + ((1 - c["static_frac"]) * L - c["static_nm"]) / k
    cases = [(lift + 0.004, 0.35, "up"), (lift - 0.004, 0.35, "not up")]
    if fall > i0 + 0.004:
        cases += [(fall - 0.004, 0.0, "down"), (fall + 0.004, 0.0, "not down")]
    for amps, goal, want in cases:
        # As the bench ran it: hold level at 300 mA, THEN the cap and the goal.
        m, d = _hinge(motor=False, mass=mass)
        srv = rs.CurrentBasedPositionServo(m, params, joint="hinge", actuator=None,
                                           goal_current=300)
        _run(m, d, [srv], 0.5)
        srv.set_goal_current(round(amps * 1000))
        srv.goal = goal
        moved = _run(m, d, [srv], 1.0)
        got = {"up": moved > 1.0, "not up": moved <= 1.0,
               "down": moved < -1.0, "not down": moved >= -1.0}[want]
        assert got, (f"{mass * 1000:.0f} g at {amps * 1000:.1f} mA (lift line "
                     f"{lift * 1000:.1f}, fall {fall * 1000:.1f}): {moved:+.2f} deg")


def test_no_hook_keeps_the_static_value(params):
    """A loop that never calls pre_step still has the no-load static friction
    on the steer and the crank -- a constant, not zero."""
    f0 = gf.constants(params)["static_nm"]
    ratio = params["bike"]["steering"]["gear_ratio"]
    m = build_model(params, righting=True, swing_linkage=True)
    assert m.dof_frictionloss[m.joint("steer_joint").dofadr[0]] == pytest.approx(f0 * ratio)
    assert m.dof_frictionloss[m.joint("swing_crank_joint").dofadr[0]] == pytest.approx(f0)


def test_bearing_friction_stays_under_the_gearbox(params):
    m, d = _hinge(motor=True, mass=0.01, gravity=False)
    fr = gf.GearboxFriction(m, params, "hinge", base=0.02)
    f0 = gf.constants(params)["static_nm"]
    assert m.dof_frictionloss[0] == pytest.approx(0.02 + f0)
    d.ctrl[0] = 0.1
    _run(m, d, [fr], 0.01)
    assert m.dof_frictionloss[0] > 0.02 + f0


def test_the_headset_term_reproduces_the_bench_line(params):
    """bike.steering's headset constants, under the gearbox's running line,
    give back the steer bench's fit (steering-design.md, 2026-09-30): a motor
    turning the headset steadily needs (1 + c) x headset + f0, and at 86-368 g
    along the 15 deg axis that is 7.68 + 0.0518 x grams mN m, rms 0.4."""
    st = params["bike"]["steering"]
    c = gf.constants(params)
    cos = np.cos(np.radians(params["bike"]["rake_deg"]))
    for grams in (86, 158, 268, 368):
        T = grams * 1e-3 * G * cos
        H = st["headset_friction_nm"] + st["headset_friction_per_n"] * T
        tau = (1 + c["running_frac"]) * H + c["running_nm"]
        assert tau * 1000 == pytest.approx(7.68 + 0.0518 * grams, abs=0.5), grams


def test_the_steer_reads_the_headset_load(params):
    """On the full bike the steer's limit is the gearbox's plus a + b|T|, T
    the headset_force sensor along the axis -- the same step's limit with the
    headset switched off differs by exactly that -- and at rest T is the
    front tyre's share of the weight along the axis, less what turns."""
    m = build_model(params)
    d = mujoco.MjData(m)
    st = gf.attach_steer(m, params)
    assert st._touch is not None
    st.reset(d)
    mujoco.mj_forward(m, d)
    for _ in range(int(0.1 / m.opt.timestep)):
        st.pre_step(d)
        mujoco.mj_step(m, d)
    T = float(d.sensor("headset_force").data[2])
    W = float(m.body_subtreemass[0]) * G
    turning = float(m.body_subtreemass[m.body("steer").id]) * G
    cos = np.cos(np.radians(params["bike"]["rake_deg"]))
    assert 0.3 * W * cos - turning < abs(T) < 0.6 * W * cos - turning
    st.update(d, 0.0)
    with_headset = float(m.dof_frictionloss[st.dof])
    adr, st._touch = st._touch, None
    st.update(d, 0.0)
    without = float(m.dof_frictionloss[st.dof])
    st._touch = adr
    s = params["bike"]["steering"]
    assert with_headset - without == pytest.approx(
        s["headset_friction_nm"] + s["headset_friction_per_n"] * abs(T))
    assert without == pytest.approx(st.base + st.s0 / (1 - st.sc))   # at rest, no torque
