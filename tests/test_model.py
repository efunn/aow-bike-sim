"""Model correctness tests: compilation, couplings, geometry, and behavior."""

import mujoco
import numpy as np
import pytest

from aow_sim import geometry
from aow_sim.build_model import build_model, load_params


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def full_model(params):
    return build_model(params, variant="full")


@pytest.fixture(scope="module")
def testbed_model(params):
    return build_model(params, variant="testbed")


def _step_for(model, data, seconds):
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qacc)), "simulation blew up"


def test_variants_compile_with_expected_dofs(full_model, testbed_model):
    # full: free(6) + steer + front + hub + ring + 8 rollers + 2 inputs = 20
    assert full_model.nv == 20
    assert full_model.nu == 3  # drive_a, drive_b, steer
    # testbed: hub + ring + 8 rollers + 2 inputs = 12
    assert testbed_model.nv == 12
    assert testbed_model.nu == 2
    # 8 roller couplings + 2 gearbox tendon constraints
    assert full_model.neq == 10


def test_steering_joint_unlimited(full_model):
    j = full_model.joint("steer_joint")
    assert not j.limited[0], "steering must allow continuous 360°+ rotation"


def test_envelope_matches_outer_radius(params):
    ow = params["omni_wheel"]
    dev = geometry.envelope_deviation(
        ow["outer_radius"], ow["axle_mount_radius"], ow["roller"]
    )
    assert dev < 0.001, f"cone envelope deviates {dev*1000:.2f} mm from wheel outer radius"


def test_roller_coupling_ratio(params, testbed_model):
    """Drive the ring input: every roller spins by the identical angle, k x ring."""
    m = testbed_model
    d = mujoco.MjData(m)
    d.ctrl[m.actuator("drive_b").id] = 5.0
    _step_for(m, d, 1.0)
    ring = d.qpos[m.joint("ring_spin").qposadr[0]]
    rollers = np.array(
        [d.qpos[m.joint(f"roller_spin_{i}").qposadr[0]] for i in range(8)]
    )
    assert abs(ring) > 1.0, "ring did not spin"
    assert np.ptp(rollers) < 1e-9, "rollers out of sync (rigid gearing violated)"
    k = params["drivetrain"]["k_roller"]
    assert rollers[0] / ring == pytest.approx(k, rel=0.01)


def test_gearbox_mixing(params, testbed_model):
    """Differential: hub (carrier) tracks mix_hub_a * input A when B is held."""
    m = testbed_model
    d = mujoco.MjData(m)
    d.ctrl[m.actuator("drive_a").id] = 4.0
    _step_for(m, d, 1.0)
    hub_v = d.qvel[m.joint("hub_spin").dofadr[0]]
    in_v = d.qvel[m.joint("input_a_spin").dofadr[0]]
    assert hub_v == pytest.approx(in_v * params["drivetrain"]["mix_hub_a"], rel=0.02)


def test_rest_stability(params):
    """With training wheels, the bike stands still: no jitter, drift, or sinking."""
    m = build_model(params, variant="full", training_wheels=True)
    d = mujoco.MjData(m)
    _step_for(m, d, 2.0)
    assert np.linalg.norm(d.qpos[:2]) < 0.005, "bike drifted at rest"
    r_rear = params["omni_wheel"]["outer_radius"]
    assert d.qpos[2] > r_rear - 0.002, "bike sank into the floor"
    assert np.abs(d.qvel).max() < 0.05, "bike jitters at rest"


def test_falls_without_support(params, full_model):
    """No training wheels, no control: the bike tips over like a real bike."""
    d = mujoco.MjData(full_model)
    d.qvel[3] = 0.1  # small roll-rate nudge off the unstable equilibrium
    _step_for(full_model, d, 2.0)
    up_z = np.zeros(9)
    mujoco.mju_quat2Mat(up_z, d.qpos[3:7])
    assert up_z[8] < 0.5, "bike should have fallen over (chassis z-axis tilted > 60°)"


def test_forward_roll(params):
    """Equal drive input rolls the bike forward near the rigid-rolling speed."""
    m = build_model(params, variant="full", training_wheels=True)
    d = mujoco.MjData(m)
    for tag in ("drive_a", "drive_b"):
        d.ctrl[m.actuator(tag).id] = 6.0
    _step_for(m, d, 2.0)
    assert d.qpos[0] > 0.4, f"only advanced {d.qpos[0]:.3f} m"
    assert abs(d.qpos[1]) < 0.05, "veered sideways under symmetric drive"


def test_lateral_crawl(params):
    """Opposed inputs = pure differential: rollers spin, rear wheel crawls
    sideways with (almost) no forward roll (hub = mean of the ring gears = 0)."""
    m = build_model(params, variant="full", training_wheels=True)
    d = mujoco.MjData(m)
    d.ctrl[m.actuator("drive_a").id] = 4.0
    d.ctrl[m.actuator("drive_b").id] = -4.0
    _step_for(m, d, 2.0)
    assert abs(d.qpos[1]) > 0.05, "no lateral crawl (AOW signature behavior missing)"
    assert abs(d.qpos[0]) < 0.5 * abs(d.qpos[1]), "opposed drive should mostly crawl, not roll"


def test_sensors_present(full_model):
    for name in ("ahrs_gyro", "ahrs_accel", "ahrs_quat",
                 "steer_pos", "steer_vel",
                 "input_a_pos", "input_a_vel", "input_b_pos", "input_b_vel"):
        assert full_model.sensor(name) is not None
