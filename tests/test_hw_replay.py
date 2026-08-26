"""The test that proves the hardware shim.

The whole untethered design rests on one claim: `HardwareData` carries enough
of mjData that `DriveController` runs on the bike *unmodified*, and a
controller rebuilt from `deploy/bundle.npz` is the same controller the
simulator ran. If that holds, every sim result transfers; if it silently does
not, the bike falls over and the cause is invisible.

So: drive the real controller in a real MuJoCo loop, record the state, then
replay only the slots the hardware backend can actually populate through a
bundle-built controller, and require the emitted `ctrl` to match.

Needs no hardware. The bundle test skips until `export_deploy` has been run.
"""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control.drive import DriveController
from aow_sim.hw.state import HardwareData, load_bundle

# The HardwareData shim contract, and the digest that pins the deploy bundle.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.spec

BUNDLE = Path(__file__).resolve().parents[1] / "deploy" / "bundle.npz"


def _record(model, data, ctrl_source, n_steps):
    """Step the sim, capturing the state the hardware backend would measure
    plus the ctrl the controller emitted."""
    sj = model.joint("steer_joint").qposadr[0]
    sd = model.joint("steer_joint").dofadr[0]
    frames = []
    for _ in range(n_steps):
        ctrl_source.step(model, data)
        frames.append({
            "time": float(data.time),
            "freejoint_qpos": data.qpos[0:7].copy(),
            "freejoint_qvel": data.qvel[0:6].copy(),
            "steer_qpos": float(data.qpos[sj]),
            "steer_qvel": float(data.qvel[sd]),
            "ctrl": data.ctrl.copy(),
        })
        mujoco.mj_step(model, data)
    return frames


def _fill(hd, f, sj, sd):
    hd.time = f["time"]
    hd.qpos[0:7] = f["freejoint_qpos"]
    hd.qvel[0:6] = f["freejoint_qvel"]
    hd.qpos[sj] = f["steer_qpos"]
    hd.qvel[sd] = f["steer_qvel"]


def _replay(controller, model, frames, nq, nv, nu, sj, sd, setup=None):
    """Feed the recorded frames through HardwareData and collect ctrl.

    The controller is initialized from frame 0 exactly the way the bike does
    it — off the shim, never off an mjData — so nothing leaks in from the
    simulator side.
    """
    hd = HardwareData(nq, nv, nu)
    _fill(hd, frames[0], sj, sd)
    controller.reset(model, hd)
    if setup is not None:
        setup(controller, hd)
    out = []
    for f in frames:
        _fill(hd, f, sj, sd)
        controller.step(model, hd)
        out.append(hd.ctrl.copy())
    return out


def test_hardware_data_covers_the_control_path():
    """A controller replayed through HardwareData emits identical commands.

    This is the shim's contract: the four attributes HardwareData exposes, and
    the ~14 numbers it fills, are all the balance/drive path reads.
    """
    params = load_params()
    model = build_model(params)
    data = mujoco.MjData(model)
    sj = model.joint("steer_joint").qposadr[0]
    sd = model.joint("steer_joint").dofadr[0]

    sim_ctl = DriveController(params, model)
    sim_ctl.reset(model, data)
    sim_ctl.set_speed(0.5)
    frames = _record(model, data, sim_ctl, 1500)

    # A second, independent controller -- same class, same gains, but driven
    # only through the shim.
    hw_ctl = DriveController(params, model)
    replayed = _replay(hw_ctl, model, frames, model.nq, model.nv, model.nu, sj, sd,
                       setup=lambda c, hd: c.set_speed(0.5))

    for i, (f, u) in enumerate(zip(frames, replayed)):
        assert np.allclose(f["ctrl"], u, atol=1e-9), (
            f"ctrl diverged at frame {i} (t={f['time']:.4f}): "
            f"sim {f['ctrl']} vs shim {u} -- HardwareData is missing a slot "
            f"the control path reads")


def test_hardware_data_covers_the_general_policy():
    """Same contract for the always-on RL policy, which is what deploys.

    "What deploys" means `control.general_move`, so this now drives whatever
    that names rather than a hardcoded `general_rl` -- which was a different
    policy from the configured one, and the docstring was simply wrong.

    NOTE WHAT THIS DOES AND DOES NOT PROVE. Both sides are fed the SAME
    recorded `qvel`, so it is a determinism check on the control path: given
    identical inputs, the shim emits identical `ctrl`. It says nothing about
    whether that velocity is TRUE or ESTIMATED, and it cannot -- an
    obs_odometry policy passes here whichever signal it is handed. Closed-loop
    survival on the estimate is a separate question and is not tested anywhere;
    see docs/plans/odometry-rewrite.md.
    """
    params = load_params()
    name = params["control"].get("general_move", "general_rl")
    if not (Path(__file__).resolve().parents[1] / "moves" / f"{name}.npz").exists():
        pytest.skip(f"control.general_move names {name}, which is not exported")

    model = build_model(params)
    data = mujoco.MjData(model)
    sj = model.joint("steer_joint").qposadr[0]
    sd = model.joint("steer_joint").dofadr[0]

    sim_ctl = DriveController(params, model)
    sim_ctl.reset(model, data)
    sim_ctl.engage_general(data, name=name)
    sim_ctl.set_command(v_cmd_world=[0.5, 0.0], psi_cmd=0.3)
    frames = _record(model, data, sim_ctl, 1500)

    def _engage(c, hd):
        c.engage_general(hd, name=name)
        c.set_command(v_cmd_world=[0.5, 0.0], psi_cmd=0.3)

    hw_ctl = DriveController(params, model)
    replayed = _replay(hw_ctl, model, frames, model.nq, model.nv, model.nu, sj, sd,
                       setup=_engage)

    for i, (f, u) in enumerate(zip(frames, replayed)):
        assert np.allclose(f["ctrl"], u, atol=1e-9), (
            f"general-policy ctrl diverged at frame {i}: {f['ctrl']} vs {u}")


@pytest.mark.deploy
@pytest.mark.skipif(not BUNDLE.exists(),
                    reason="run `python -m aow_sim.export_deploy` first")
def test_bundle_controller_matches_mujoco_controller():
    """A controller built from deploy/bundle.npz alone == the MuJoCo one.

    Covers the other half of the deployment path: DeployModel must reproduce
    the actuator ids, ctrlranges and joint addresses, and the shipped gain
    schedule must be the one that was designed.
    """
    params = load_params()
    model = build_model(params)
    data = mujoco.MjData(model)
    sj = model.joint("steer_joint").qposadr[0]
    sd = model.joint("steer_joint").dofadr[0]

    design, deploy_model = load_bundle(BUNDLE, params)

    assert deploy_model.nq == model.nq and deploy_model.nv == model.nv
    assert deploy_model.nu == model.nu
    for name in ("drive_a", "drive_b", "steer"):
        assert deploy_model.actuator(name).id == model.actuator(name).id
    assert deploy_model.joint("steer_joint").qposadr[0] == sj
    assert deploy_model.joint("steer_joint").dofadr[0] == sd

    sim_ctl = DriveController(params, model)
    sim_ctl.reset(model, data)
    sim_ctl.set_speed(0.5)
    frames = _record(model, data, sim_ctl, 1500)

    # Built with NO MuJoCo model and NO linearization.
    hw_ctl = DriveController(params, deploy_model, design)
    replayed = _replay(hw_ctl, deploy_model, frames,
                       deploy_model.nq, deploy_model.nv, deploy_model.nu, sj, sd,
                       setup=lambda c, hd: c.set_speed(0.5))

    for i, (f, u) in enumerate(zip(frames, replayed)):
        assert np.allclose(f["ctrl"], u, atol=1e-9), (
            f"bundle controller diverged at frame {i}: {f['ctrl']} vs {u}")


@pytest.mark.deploy
def test_bundle_digest_rejects_stale_params():
    """A bundle designed for different parameters must refuse to load."""
    if not BUNDLE.exists():
        pytest.skip("run `python -m aow_sim.export_deploy` first")
    params = load_params()
    params["bike"]["chassis"]["mass"] *= 1.5     # a bike this bundle is not for
    with pytest.raises(ValueError, match="digest"):
        load_bundle(BUNDLE, params)
