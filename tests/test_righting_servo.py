"""The righting servo in current-based position mode: src/aow_sim/righting_servo.py.

Pins the IMPLEMENTATION, not the calibration: every expectation is derived
from the constants in bike_params (current_deadband, current_torque_gain, the
friction block), so a re-fit leaves these green and only broken code turns
them red. Whether the constants reproduce the bench is analysis:
analysis/servo_lift_sim.py.

What is pinned: torque linear in the current demand above the drive edge and
zero below it; braking by plugging, as measured; Present Current reading the
demand; a crank unable to outrun the motor line (every speed test also runs
the bare actuator and asserts IT does outrun the motor, so a pass cannot come
from a stroke too gentle to reach the limit); and a loaded lever that lifts
and falls where the params' own lines say.
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


def _floating(params, servo_model=True, torque_nm=0.55):
    """The bike held still in the air: the crank swings the linkage through
    the air with nothing to push on, which is the fastest it can ever go."""
    m = build_model(params, righting=True, swing_linkage=True)
    m.opt.gravity[:] = 0.0
    d = mujoco.MjData(m)
    d.qpos[2] += 0.5
    mujoco.mj_forward(m, d)
    servo = (rs.CurrentBasedPositionServo.attach(m, params, torque_nm=torque_nm)
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
    """P 700 / D 1400 through k5 and the measured k reproduce servo_kp / kv."""
    k = params["servos"]["xc330_t181"]["current_torque_gain"]
    g = params["control"]["onboard"]["gains"]["righting"]
    wings = params["righting"]["wings"]
    assert g["Position P Gain"] * rs.K5 * k == pytest.approx(wings["servo_kp"], rel=5e-3)
    assert g["Position D Gain"] * rs.KD_PER_UNIT * k == pytest.approx(wings["servo_kv"], rel=5e-3)


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
    """At stall the torque is exactly `cap_nm`, k (I - I0); while driving it
    never exceeds it."""
    m, d, servo = _floating(params)
    servo.set_goal_current(200)
    w, tau = _stroke(m, d, servo, goal=2.0, seconds=0.1)
    assert tau[0] == pytest.approx(servo.cap_nm, rel=1e-6)          # at stall
    assert servo.cap_nm == pytest.approx(servo.k * (0.2 - servo.i0), rel=1e-9)
    driving = np.sign(tau) == np.sign(w)
    assert np.abs(tau[driving]).max() <= servo.cap_nm + 1e-9


def test_torque_is_linear_above_the_edge_and_off_below(params):
    m, d, servo = _floating(params)
    for i in (0.0, 0.5 * servo.i0, servo.i0, -servo.i0):
        assert servo.torque_for(i, 0.0)[0] == 0.0
        assert servo.torque_for(i, 5.0)[0] == 0.0      # off, not shorted: no braking
    for i in (0.05, 0.2, -0.3):
        tau, u, sat = servo.torque_for(i, 0.0)
        assert not sat
        assert tau == pytest.approx(math.copysign(servo.k * (abs(i) - servo.i0), i))


def test_counts_for_inverts_cap(params):
    m, d, servo = _floating(params)
    for nm in (0.05, 0.2, 0.55):
        servo.set_goal_current(servo.counts_for(nm))
        assert servo.cap_nm == pytest.approx(nm, abs=servo.k * 1e-3)


def test_present_current_reads_the_demand(params):
    """At stall, below the edge too, Present Current is the demand exactly."""
    m, d, servo = _floating(params)
    for counts in (10, 200):
        servo.set_goal_current(counts)
        _stroke(m, d, servo, goal=0.0, seconds=0.01)
        _stroke(m, d, servo, goal=2.0, seconds=m.opt.timestep * (servo.nc + 1))
        assert servo.current == pytest.approx(counts * rs.AMPS_PER_COUNT)


def test_braking_plugs(params):
    """Measured: the bench servos brake with the duty REVERSED against the
    motion. A braking torque larger than ts x needs exactly that, and gets
    it; a smaller one keeps the duty's sign."""
    m, d, servo = _floating(params)
    tau, u, _ = servo.torque_for(-0.4, 3.0)
    assert -tau > servo.ts * 3.0 / servo.w0 and u < 0.0
    tau, u, _ = servo.torque_for(-0.1, 8.0)
    assert -tau < servo.ts * 8.0 / servo.w0 and u > 0.0


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
