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

# Quaternion algebra against MuJoCo as an oracle; no bike model.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.pure


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


# --- "no attitude" has three causes and they need different fixes ---------
# Added 2026-09-16 after a bench session where a healthy TM151 read as a dead
# one: factory-configured units stream rpy(35) + raw_gyro_acc_mag(41) +
# status(22) and NO Ep_Combo, which parse_frame skips silently.

def test_a_valid_non_combo_frame_is_skipped_without_an_error():
    """The behaviour itself is right — this pins it so the DIAGNOSIS stays
    honest about what it means."""
    from aow_sim.hw.ahrs import parse_frame
    frame = _frame(cmd=41, payload_len=44)
    decoded, consumed = parse_frame(frame)
    assert decoded is None, "only Ep_Combo decodes"
    assert consumed == len(frame), "but the frame IS consumed, not resynced past"


def test_the_reader_counts_bytes_as_well_as_frames():
    """`frames == 0` cannot tell an unplugged sensor from a misconfigured one.
    `bytes_in` is what separates them."""
    from aow_sim.hw.ahrs import AhrsReader
    r = AhrsReader("/dev/null")
    assert r.bytes_in == 0 and r.frames == 0


def _frame(cmd: int, payload_len: int) -> bytes:
    """A CRC-valid frame carrying `cmd`, of a length that is not Combo's."""
    from aow_sim.hw.ahrs import crc16_modbus
    payload = bytes([cmd]) + bytes(payload_len - 1)
    body = bytes([payload_len]) + payload
    return b"\xaa\x55" + body + crc16_modbus(body).to_bytes(2, "little")


# --- poll mode ------------------------------------------------------------
# Added 2026-09-16. The sensor on the bench streams rpy/status/raw and NOT
# Ep_Combo, and that profile lives in its flash with no API to change it, so
# the reader can ask for each Combo instead. Strictly additive: push remains
# the default and the designed path.

def test_the_request_frame_is_byte_identical_to_the_vendor_builder():
    """`analysis/tm151_serial.request` is the reference, written from the
    EasyProfile C source. hw/ cannot import it (analysis/ is not installed on
    the bike), so the bytes are re-derived -- and checked, not trusted."""
    import sys
    sys.path.insert(0, "analysis")
    from tm151_serial import request

    from aow_sim.hw.ahrs import EP_CMD_COMBO, build_request
    assert build_request() == request(EP_CMD_COMBO)


def test_the_request_frame_passes_the_same_crc_the_sensor_checks():
    from aow_sim.hw.ahrs import build_request, crc16_modbus
    f = build_request()
    assert f[:2] == b"\xaa\x55"
    body = f[2:-2]
    assert crc16_modbus(body) == int.from_bytes(f[-2:], "little")
    assert body[0] == len(body) - 1, "size byte counts the payload"


def test_push_is_the_default():
    """Poll mode must never arrive by accident -- it is a different contract
    with the sensor, and untethered-setup.md argues for push."""
    from aow_sim.hw.ahrs import AhrsReader
    assert AhrsReader("/dev/null").poll is False
    assert AhrsReader("/dev/null", poll=True).poll is True
    assert AhrsReader("/dev/null").requests == 0
