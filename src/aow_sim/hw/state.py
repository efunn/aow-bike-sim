"""mjModel/mjData stand-ins, so the controllers run unmodified on hardware.

The balance/drive control path touches mjData through exactly four attributes
— `qpos`, `qvel`, `time`, `ctrl` — and mjModel through a handful of constants.
That is shallow enough to duck-type, which is much better than refactoring the
controllers behind an abstract state interface: there is then no second code
path to keep in agreement with the simulator, and `tests/test_hw_replay.py`
can prove the two produce identical commands.

What actually gets filled, out of a ~40-DOF model:

    qpos[0:7]            chassis freejoint — [x, y, z, quat(w,x,y,z)]
    qpos[steer_qposadr]  steer joint, multi-turn radians
    qvel[0:6]            chassis freejoint — [vx, vy, vz(world), wx, wy, wz(body)]
    qvel[steer_dofadr]   steer joint rate

Everything else stays zero. The rollers, hub, ring, and front wheel have no
sensors and no controller reads them.

Two conventions inherited from MuJoCo's freejoint, both load-bearing:
  * qvel[0:3] is WORLD-frame linear velocity (extract_state rotates it into
    the body frame by yaw), while qvel[3:6] is BODY-frame angular velocity.
    The odometry estimator must therefore hand back world-frame linear and
    body-frame angular. Getting this backwards is silent and destabilizing.
  * qpos[3:7] is (w, x, y, z), not (x, y, z, w).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..control.lqr_design import LQRDesign


class _Named:
    """Stands in for the objects `model.actuator(name)` / `model.joint(name)`
    return; the controllers only ever read `.id`, `.qposadr`, `.dofadr`."""

    def __init__(self, id=0, qposadr=0, dofadr=0):
        self.id = int(id)
        self.qposadr = np.array([int(qposadr)])
        self.dofadr = np.array([int(dofadr)])


@dataclass
class DeployModel:
    """The mjModel constants `_Base.__init__` and `LQRBalance.__init__` read.

    Built from deploy/bundle.npz — no MJCF compile, no MuJoCo model on the Pi.
    """

    nq: int
    nv: int
    nu: int
    actuator_ctrlrange: np.ndarray
    actuator_ctrllimited: np.ndarray
    _actuators: dict
    _joints: dict

    def actuator(self, name: str) -> _Named:
        return self._actuators[name]

    def joint(self, name: str) -> _Named:
        return self._joints[name]


class HardwareData:
    """Quacks like mjData for the slots the controllers touch.

    `time` is the monotonic control-loop clock in seconds. The controllers use
    it only for zero-order-hold scheduling and maneuver timing, and
    `_Base.step` re-resets itself if time jumps backwards by more than 2 dt —
    so it must be monotonic, but its origin does not matter.
    """

    def __init__(self, nq: int, nv: int, nu: int):
        self.qpos = np.zeros(nq)
        self.qvel = np.zeros(nv)
        self.ctrl = np.zeros(nu)
        self.time = 0.0
        # Upright, identity orientation, so a controller constructed before the
        # first sensor read never sees an invalid (zero-norm) quaternion.
        self.qpos[3] = 1.0

    def set_orientation(self, quat_wxyz, gyro_body) -> None:
        """AHRS -> freejoint pose/rate. `gyro_body` is [wx, wy, wz] rad/s in
        the chassis frame (roll rate, pitch rate, yaw rate)."""
        q = np.asarray(quat_wxyz, dtype=float)
        self.qpos[3:7] = q / np.linalg.norm(q)
        self.qvel[3:6] = np.asarray(gyro_body, dtype=float)

    def set_velocity(self, v_world_xy) -> None:
        """Odometry -> freejoint linear velocity (WORLD frame, z left at 0)."""
        self.qvel[0:2] = np.asarray(v_world_xy, dtype=float)[:2]

    def integrate_position(self, dt: float) -> None:
        """Dead-reckon qpos[:2] from qvel[:2].

        Drifts without bound, and that is accepted: `general_rl` never reads
        world position (see control/general_spec.py), and the LQR/moves only
        need it over the seconds following a fresh anchor.
        """
        self.qpos[0:2] += self.qvel[0:2] * dt


def load_bundle(path: str | Path, params: dict | None = None
                ) -> tuple[LQRDesign, DeployModel]:
    """Load deploy/bundle.npz -> (LQRDesign, DeployModel).

    If `params` is given, the bundle's digest is checked against it. A gain
    schedule designed for different parameters than the bike is running is a
    fall, not a warning — so this raises rather than warns.
    """
    d = np.load(path, allow_pickle=False)

    if params is not None:
        from ..export_deploy import params_digest
        want, got = params_digest(params), str(d["params_digest"])
        if want != got:
            raise ValueError(
                f"bundle {path} was designed for params digest {got}, but the "
                f"loaded bike_params.yaml hashes to {want}. Re-run "
                f"`python -m aow_sim.export_deploy`.")

    design = LQRDesign(
        K=d["K"], qpos_eq=d["qpos_eq"], fit_r2=d["fit_r2"],
        speeds=d["speeds"], Ks=d["Ks"], fit_r2_grid=d["fit_r2_grid"],
    )
    names = [str(n) for n in d["actuator_names"]]
    model = DeployModel(
        nq=int(d["nq"]), nv=int(d["nv"]), nu=int(d["nu"]),
        actuator_ctrlrange=d["actuator_ctrlrange"],
        actuator_ctrllimited=d["actuator_ctrllimited"],
        _actuators={n: _Named(id=i) for n, i in zip(names, d["actuator_ids"])},
        _joints={"steer_joint": _Named(qposadr=d["steer_qposadr"],
                                       dofadr=d["steer_dofadr"])},
    )
    return design, model
