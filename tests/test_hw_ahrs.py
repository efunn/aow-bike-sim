"""AHRS frame conventions.

The sensor thread reimplements quaternion math in numpy so it carries no
MuJoCo dependency. That is only safe if it agrees with MuJoCo exactly — a
quarter-degree convention difference between the simulator and the bike is
invisible in code review and shows up as a permanent roll trim on hardware.
"""

import mujoco
import numpy as np
import pytest

from aow_sim.control.balance import extract_state
from aow_sim.hw.ahrs import MountCalibration, quat_conj, quat_mul, quat_to_mat


def _random_quats(n, seed=0):
    rng = np.random.default_rng(seed)
    q = rng.normal(size=(n, 4))
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def test_quat_to_mat_matches_mujoco():
    for q in _random_quats(50):
        mine = quat_to_mat(q)
        theirs = np.zeros(9)
        mujoco.mju_quat2Mat(theirs, q)
        assert np.allclose(mine, theirs.reshape(3, 3), atol=1e-12)


def test_quat_mul_matches_mujoco():
    a, b = _random_quats(30, seed=1), _random_quats(30, seed=2)
    for qa, qb in zip(a, b):
        mine = quat_mul(qa, qb)
        theirs = np.zeros(4)
        mujoco.mju_mulQuat(theirs, qa, qb)
        assert np.allclose(mine, theirs, atol=1e-12)


def test_mount_calibration_zeroes_the_reference_pose():
    """After capture, the reference orientation must read as identity —
    otherwise the mounting tilt becomes a standing roll bias."""
    for q in _random_quats(20, seed=3):
        cal = MountCalibration()
        cal.capture(q)
        out = cal.to_chassis_quat(q)
        assert np.allclose(np.abs(out), [1, 0, 0, 0], atol=1e-12)


def test_mount_calibration_preserves_relative_rotation():
    """A known rotation away from the reference must survive calibration."""
    q_mount = _random_quats(1, seed=4)[0]
    lean = np.deg2rad(7.0)
    q_rel = np.array([np.cos(lean / 2), np.sin(lean / 2), 0.0, 0.0])   # roll
    cal = MountCalibration()
    cal.capture(q_mount)
    chassis = cal.to_chassis_quat(quat_mul(q_mount, q_rel))
    R = quat_to_mat(chassis)
    assert np.isclose(np.arctan2(R[2, 1], R[2, 2]), lean, atol=1e-9)


def test_rpy_matches_extract_state():
    """hw.run_bike._rpy must extract roll/yaw the way extract_state does."""
    from aow_sim.hw.run_bike import _rpy
    from aow_sim.build_model import build_model

    model = build_model()
    data = mujoco.MjData(model)
    for q in _random_quats(25, seed=5):
        data.qpos[3:7] = q
        s = extract_state(data, np.zeros(3))
        roll, _, yaw = _rpy(q)
        assert np.isclose(roll, s.roll, atol=1e-12)
        assert np.isclose(yaw, s.yaw, atol=1e-12)
