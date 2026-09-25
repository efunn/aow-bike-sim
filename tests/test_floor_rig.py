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
