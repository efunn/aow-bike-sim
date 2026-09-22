"""The righting servo in current-based position mode: src/aow_sim/righting_servo.py.

Pins what the model exists for: a loop that regulates BUS current, so torque
at stall follows sqrt(Goal Current); braking by plugging, as measured; and a
crank unable to outrun the motor line. Every speed test also runs the bare actuator
and asserts IT does outrun the motor, so a pass cannot come from a stroke too
gentle to reach the limit. Goal PWM has no knob: measured, it is not a duty
ceiling in mode 5 (see righting_servo.py).
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


def _floating(params, servo_model=True, torque_nm=0.55, braking="plug"):
    """The bike held still in the air: the crank swings the linkage through
    the air with nothing to push on, which is the fastest it can ever go."""
    m = build_model(params, righting=True, swing_linkage=True)
    m.opt.gravity[:] = 0.0
    d = mujoco.MjData(m)
    d.qpos[2] += 0.5
    mujoco.mj_forward(m, d)
    servo = (rs.CurrentBasedPositionServo.attach(m, params, torque_nm=torque_nm,
                                                 braking=braking)
             if servo_model else None)
    return m, d, servo


def _stroke(m, d, servo, goal, seconds=0.4):
    aid = m.actuator("swing").id
    dof = m.jnt_dofadr[m.joint("swing_crank_joint").id]
    w, tau = [], []
    for _ in range(int(round(seconds / m.opt.timestep))):
        d.qvel[:6] = 0.0                        # chassis held still
        d.ctrl[aid] = goal
        if servo is not None:
            servo.pre_step(d)
        mujoco.mj_step(m, d)
        w.append(float(d.qvel[dof]))
        tau.append(servo.torque if servo is not None else float(d.actuator_force[aid]))
    return np.array(w), np.array(tau)


@pytest.mark.pure
def test_factory_gains_are_the_native_actuators(params):
    """P 700 / D 1400 through k5 and kt reproduce servo_kp / servo_kv."""
    srv = params["servos"]["xc330_t181"]
    kt = srv["stall_torque"] / srv["stall_current"]
    g = params["control"]["onboard"]["gains"]["righting"]
    wings = params["righting"]["wings"]
    assert g["Position P Gain"] * rs.K5 * kt == pytest.approx(wings["servo_kp"], rel=2e-3)
    assert g["Position D Gain"] * rs.KD_PER_UNIT * kt == pytest.approx(wings["servo_kv"], rel=2e-3)


def test_absent_without_the_swing_linkage(params):
    assert rs.CurrentBasedPositionServo.attach(build_model(params), params) is None


def test_native_actuator_becomes_a_command_holder(params):
    m = build_model(params, righting=True, swing_linkage=True)
    rs.CurrentBasedPositionServo.attach(m, params, torque_nm=0.55)
    aid = m.actuator("swing").id
    assert m.actuator_gainprm[aid, 0] == 0.0
    assert not m.actuator_biasprm[aid, :3].any()
    # ctrlrange still clamps the goal, so +-travel keeps its meaning
    assert m.actuator_ctrllimited[aid]


def test_goal_current_caps_the_driving_torque(params):
    """Goal Current bounds BUS current, so it bounds the torque while the servo
    drives -- at stall exactly `cap_nm`. Braking by plugging is not bounded by
    it, and is not asserted here."""
    m, d, servo = _floating(params)
    servo.set_goal_current(200)
    w, tau = _stroke(m, d, servo, goal=2.0, seconds=0.1)
    assert tau[0] == pytest.approx(servo.cap_nm, rel=1e-6)          # at stall
    driving = np.sign(tau) == np.sign(w)
    assert np.abs(tau[driving]).max() <= servo.cap_nm + 1e-9


@pytest.mark.pure
def test_bus_current_gives_more_torque_at_stall_than_phase_would(params):
    """At stall I_bus = I_stall u^2, so tau = ts sqrt(I / I_stall): 300 counts
    is ~0.47 N.m, not the kt * 0.3 A = 0.27 a phase-current loop would give."""
    srv = params["servos"]["xc330_t181"]
    ts, i_s = srv["stall_torque"], srv["stall_current"]
    servo = object.__new__(rs.CurrentBasedPositionServo)
    servo.ts, servo.i_stall, servo.supply, servo.goal_current = ts, i_s, 1.0, 300
    assert servo.cap_nm == pytest.approx(ts * math.sqrt(0.3 / i_s))
    assert servo.cap_nm > 1.5 * (ts / i_s) * 0.3


def test_the_loop_holds_bus_current_at_the_demand(params):
    """Present Current -- bus magnitude, signed by the duty -- equals the
    demand until the duty saturates, driving or plugging."""
    m, d, servo = _floating(params)
    for i_bus in (0.05, 0.3, -0.2):
        for w in (0.0, 3.0, -3.0):
            u = servo.duty_for(i_bus, w)
            if abs(u) < 1.0:
                tau = servo.ts * (servo.supply * u - w / servo.w0)
                # bus MAGNITUDE, signed by the duty -- how Present Current reads
                assert math.copysign(abs(u * tau / servo.kt), u) == \
                    pytest.approx(i_bus, rel=1e-9)


def test_braking_plugs_by_default(params):
    """Measured: the bench servos brake with the duty REVERSED against the
    motion. Regen, the alternative, keeps it the same sign."""
    m, d, servo = _floating(params)
    assert servo.braking == "plug"
    assert servo.duty_for(-0.1, 8.0) < 0.0
    m, d, regen = _floating(params, braking="regen")
    assert 0.0 < regen.duty_for(-0.1, 8.0) < 8.0 / regen.w0


def test_goal_current_clamps_to_current_limit(params):
    m, d, servo = _floating(params)
    assert servo.set_goal_current(5000) == rs.CURRENT_LIMIT
    assert servo.set_goal_current(-5) == 0


def test_crank_cannot_outrun_the_motor(params):
    srv = params["servos"]["xc330_t181"]
    w0 = srv["no_load_rpm"] * 2 * math.pi / 60
    m, d, _ = _floating(params, servo_model=False)
    w_ideal, _ = _stroke(m, d, None, goal=2.3)
    assert np.abs(w_ideal).max() > 1.2 * w0, "the bare clip no longer outruns the motor"
    m, d, servo = _floating(params)
    w, _ = _stroke(m, d, servo, goal=2.3)
    assert np.abs(w).max() <= 1.01 * w0


@pytest.mark.parametrize("scale", [0.5, 0.8])
def test_supply_lowers_the_ceiling(params, scale):
    """Supply voltage scales the duty ceiling, so it scales the top speed."""
    srv = params["servos"]["xc330_t181"]
    w0 = srv["no_load_rpm"] * 2 * math.pi / 60
    m, d, servo = _floating(params, torque_nm=0.8)
    servo.set_supply(scale)
    w, _ = _stroke(m, d, servo, goal=2.3)
    assert np.abs(w).max() <= 1.01 * scale * w0


def test_command_arrives_after_the_delay(params):
    m, d, servo = _floating(params)
    n = servo.nc - 1
    assert n * m.opt.timestep == pytest.approx(rs.COMMAND_DELAY_S, abs=m.opt.timestep)
    # Settle at rest first: the first pre_step seeds the pipeline with
    # whatever goal it sees, so a step has to come AFTER it to be delayed.
    _stroke(m, d, servo, goal=0.0, seconds=m.opt.timestep)
    _, tau = _stroke(m, d, servo, goal=2.0, seconds=(n + 2) * m.opt.timestep)
    assert np.abs(tau[:n]).max() < 1e-6
    assert abs(tau[n + 1]) > 0.1
