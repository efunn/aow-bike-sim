"""The floor rig model: the full bike held by a yaw -> tilt -> slide -> roll -> pitch chain.

WHAT INVALIDATES THIS FILE: `aow_sim.floor_rig`, `config/floor_rig.yaml`, or
the `rig=` branch at the end of `build_model.build_spec` that appends the
chain and welds it to the chassis.

The rig is placeholder physics on purpose -- massless parts, perfect joints --
so what these pin is that it adds NOTHING: the bike keeps its mass and its
qpos layout, a free rig carries none of its weight, a motor torque does not
blow the weld apart, and a locked joint is really gone.
"""
import copy

import mujoco
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.floor_rig import build_rig_model, load_rig_cfg

pytestmark = pytest.mark.geometry


def test_bike_mass_unchanged_and_rig_is_massless():
    p = load_params()
    bike = build_model(p)
    rig = build_rig_model(p)
    m_bike = bike.body_subtreemass[bike.body("chassis").id]
    m_rig = rig.body_subtreemass[rig.body("chassis").id]
    assert m_rig == pytest.approx(m_bike, rel=1e-12)
    # Everything the rig adds on top of the bike: one token mass per rig body.
    n_rig = sum(rig.body(i).name.startswith("rig_") for i in range(rig.nbody))
    assert n_rig == 5
    assert rig.body_subtreemass[0] - m_rig == pytest.approx(n_rig * 1e-6, abs=1e-9)


def test_free_rig_carries_no_weight():
    """All joints free: the floor contacts carry the whole bike."""
    m = build_rig_model()
    d = mujoco.MjData(m)
    for _ in range(int(0.5 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    f, total = np.zeros(6), 0.0
    for i in range(d.ncon):
        mujoco.mj_contactForce(m, d, i, f)
        total += f[0] * abs(d.contact[i].frame[2])
    weight = m.body_subtreemass[m.body("chassis").id] * -m.opt.gravity[2]
    assert total == pytest.approx(weight, rel=0.01)


def test_locked_joint_is_absent():
    cfg = copy.deepcopy(load_rig_cfg())
    cfg["joints"]["tilt"] = "locked"
    m = build_rig_model(cfg=cfg)
    names = {m.joint(j).name for j in range(m.njnt)}
    assert "rig_tilt" not in names
    assert {"rig_yaw", "rig_slide", "rig_roll", "rig_pitch"} <= names


def test_bike_indices_unchanged():
    """The rig is appended: qpos[0:7] is still the chassis freejoint, and the
    bike's actuators keep their indices."""
    p = load_params()
    bike, rig = build_model(p), build_rig_model(p)
    assert rig.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert rig.jnt_bodyid[0] == rig.body("chassis").id
    for i in range(bike.nu):
        assert rig.actuator(i).name == bike.actuator(i).name


def test_pitch_torque_is_stable_and_welded():
    m = build_rig_model()
    d = mujoco.MjData(m)
    d.ctrl[m.actuator("rig_pitch_motor").id] = 0.8
    for _ in range(int(0.3 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    assert np.isfinite(d.qacc).all() and d.time > 0.29      # no auto-reset
    a, b = d.body("rig_pitch"), d.body("chassis")
    rel = a.xmat.reshape(3, 3).T @ (b.xpos - a.xpos)
    assert np.linalg.norm(rel - m.body("chassis").pos) < 1e-3


def _nudged_fall(m, seconds=3.0):
    from aow_sim.floor_rig import place
    d = mujoco.MjData(m)
    place(m, d, 0.0)
    d.qvel[m.joint("rig_roll").dofadr[0]] = 0.3      # rig and chassis together
    d.qvel[3] = 0.3
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)
    return d


def test_hinge_ranges_are_the_configured_degrees():
    """MjSpec reads hinge ranges in DEGREES by default. Radians passed through
    once turned +-55 deg into +-1 deg and locked every rig joint."""
    cfg = load_rig_cfg()
    m = build_rig_model(cfg=cfg)
    got = np.degrees(m.jnt_range[m.joint("rig_tilt").id])
    assert got == pytest.approx(cfg["tilt_range_deg"], abs=1e-6)
    stop = cfg["roll_stop"]["deg"] + cfg["roll_stop"]["limit_margin_deg"]
    assert np.degrees(m.jnt_range[m.joint("rig_roll").id][1]) == pytest.approx(stop)


def test_roll_stop_catches_an_unactuated_fall():
    """The skid lands near its angle instead of the bike rolling on through
    the floor."""
    m = build_rig_model()
    d = _nudged_fall(m)
    roll = abs(np.degrees(d.joint("rig_roll").qpos[0]))
    assert 35.0 < roll < load_rig_cfg()["roll_stop"]["deg"] + 10.0
    a, b = d.body("rig_pitch"), d.body("chassis")
    rel = a.xmat.reshape(3, 3).T @ (b.xpos - a.xpos)
    assert np.linalg.norm(rel - m.body("chassis").pos) < 2e-3


def test_slide_stop_holds_against_a_push():
    m = build_rig_model()
    from aow_sim.floor_rig import place
    d = mujoco.MjData(m)
    place(m, d, 0.0)
    d.xfrc_applied[m.body("chassis").id, 0] = 20.0     # 20 N forward, 1 s
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    hi = load_rig_cfg()["slide_range_m"][1]
    assert d.joint("rig_slide").qpos[0] < hi + 0.005


def test_skids_stay_on_the_roll_member_when_the_bike_wheelies():
    """Driving forward into the slide stop wheelies the bike to its pitch
    limit. The skids must stay on the roll member they are drawn on; on the
    chassis they swung ~93 mm off their outriggers (2026-09-25)."""
    from aow_sim.floor_rig import place
    m = build_rig_model()
    d = mujoco.MjData(m)
    place(m, d, 0.0)
    mujoco.mj_forward(m, d)
    rb = m.body("rig_roll").id
    gs = [m.geom(f"rig_roll_stop_{t}").id for t in ("left", "right")]
    local = lambda g: d.xmat[rb].reshape(3, 3).T @ (d.geom_xpos[g] - d.xpos[rb])
    before = [local(g) for g in gs]
    d.ctrl[[m.actuator("drive_a").id, m.actuator("drive_b").id]] = 5.0
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    assert abs(np.degrees(d.joint("rig_pitch").qpos[0])) > 10.0   # it did wheelie
    for g, b in zip(gs, before):
        assert np.linalg.norm(local(g) - b) < 1e-6


def test_link_masses_friction_and_damping_reach_the_model():
    cfg = copy.deepcopy(load_rig_cfg())
    cfg["link_mass_g"] = {"tilt": 0, "slide": 45, "roll": 55, "pitch": 0}
    cfg["friction"] = {**cfg["friction"], "roll": 0.02, "slide": 0.3}
    cfg["damping"] = {**cfg["damping"], "pitch": 0.01}
    m = build_rig_model(cfg=cfg)
    assert m.body("rig_slide").mass[0] == pytest.approx(0.045)
    assert m.body("rig_roll").mass[0] == pytest.approx(0.055)
    assert m.body("rig_tilt").mass[0] == pytest.approx(1e-6)
    dof = lambda j: m.joint(f"rig_{j}").dofadr[0]
    assert m.dof_frictionloss[dof("roll")] == pytest.approx(0.02)
    assert m.dof_frictionloss[dof("slide")] == pytest.approx(0.3)
    assert m.dof_damping[dof("pitch")] == pytest.approx(0.01)
    assert m.dof_frictionloss[dof("yaw")] == 0.0


def _servo_push(counts, seconds=1.0, push=1.0):
    """Roll servo holding upright; a sideways push on the chassis for the
    whole run. Returns the final roll [deg] and the servo."""
    from aow_sim.floor_rig import attach_servos, place
    p = load_params()
    cfg = copy.deepcopy(load_rig_cfg())
    cfg["servos"] = {"roll": counts, "pitch": None}
    cfg["servo_friction_ma"] = 0        # the servo law alone; friction is tested below
    from aow_sim.floor_rig import resolve
    cfg = resolve(cfg, p)
    m = build_model(p, rig=cfg)
    srv = attach_servos(m, p, cfg)["roll"]
    d = mujoco.MjData(m)
    place(m, d, 0.0)
    d.xfrc_applied[m.body("chassis").id, 1] = push
    for _ in range(int(seconds / m.opt.timestep)):
        srv.pre_step(d)
        mujoco.mj_step(m, d)
    return np.degrees(d.joint("rig_roll").qpos[0]), srv


def test_roll_servo_fixed_is_rigid_and_current_only_caps():
    """`fixed` (Current Limit + top P gain) holds a 3 N side push near
    upright. 300 counts at the righting gains gives way: Goal Current only
    caps the torque, the stiffness below it is the P gain's."""
    soft, _ = _servo_push(300, push=3.0)
    rigid, srv = _servo_push("fixed", push=3.0)
    assert srv.goal_current == 910
    assert abs(rigid) < 1.0
    assert abs(soft) > 10.0
    small, _ = _servo_push(300, push=1.0)      # under its cap it holds
    assert abs(small) < 3.0


def test_servo_friction_rides_with_the_servo_only():
    """`servo_friction_ma` becomes Coulomb friction on an axis only while a
    servo is attached there -- 100 mA is 0.27 N m by the bus-current law --
    and stacks on the joint's own friction."""
    cfg = copy.deepcopy(load_rig_cfg())
    cfg["servo_friction_ma"] = 100
    cfg["servos"] = {"roll": 10, "pitch": None}
    cfg["friction"] = {**cfg["friction"], "roll": 0.01}
    m = build_rig_model(cfg=cfg)
    dof = lambda j: m.joint(f"rig_{j}").dofadr[0]
    srv = load_params()["servos"]["xc330_t181"]
    tau = srv["stall_torque"] * (0.1 / srv["stall_current"]) ** 0.5
    assert m.dof_frictionloss[dof("roll")] == pytest.approx(0.01 + tau, rel=1e-6)
    assert m.dof_frictionloss[dof("pitch")] == 0.0
    cfg["servos"] = {"roll": None, "pitch": None}      # servo off the shaft
    m = build_rig_model(cfg=cfg)
    assert m.dof_frictionloss[m.joint("rig_roll").dofadr[0]] == pytest.approx(0.01)
