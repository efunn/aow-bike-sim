"""Standing the bike back up: the fall-to-balancing phase machine.

`analysis/no_return.py` established that a fall cannot be caught — 0.13 s from
"visibly falling" to flat — so the mechanism acts AFTER the bike is down, and
what it needs is a sequence rather than a controller:

    lift     slew the mechanism into the floor until the bike rolls upright
    balance  hand off to the general policy once the roll is inside its
             recoverable set, holding the mechanism where it is
    retract  only after the policy has held it for a while — pulling the prop
             out early is the same as never having put it there

That sequence is identical for both candidate mechanisms (see
docs/plans/self-righting.md and the `righting` block in bike_params.yaml), and
the ONE thing that differs between them falls out as a single argument:

    arm    one arm swinging through +-180 deg, so it must swing with
           sign(roll) to reach whichever floor the bike is lying on
                -> `direction = sign(roll0)`
    wings  a mirrored pair deploying together, so whichever wing is on the
           fallen side plants and the stroke never has to know which side
           that was
                -> `direction = +1`, fixed

Used by analysis/self_righting.py (the numbers) and aow_sim.record (the video).
This module owns no geometry: it drives whatever actuator/joint it is handed.
"""

from __future__ import annotations

import numpy as np

import mujoco

from ..build_model import build_model
from .balance import quat_to_mat
from .drive import DriveController
from .linearize import settle_upright

# Hand-back target. Inside the general policy's COLD recoverable set on both
# sides at standstill (analysis/no_return.py: 16.3 deg right, 11.8 deg left)
# with margin — and note it is the weaker (left) side that sets it, which is
# why the policy's left/right asymmetry matters to the mechanism.
RECOVER_DEG = 12.0
HANDOFF_RATE = 3.0      # rad/s; a roll inside the window but still moving fast
                        #   is not a hand-off, it is a bike on its way past


# Mechanism -> (actuator name, joint name, params sub-block). The stroke
# direction is not here because for the arm it depends on which way it fell.
MECHANISMS = {
    "arm": ("righting", "righting_joint", "arm"),
    "wings": ("wings", "wing_right_joint", "wings"),
}


def mechanism(params: dict, wings: bool) -> tuple[str, str, dict]:
    """(actuator, joint, sub-block) for the selected mechanism."""
    act, joint, key = MECHANISMS["wings" if wings else "arm"]
    return act, joint, params["righting"][key]


def roll_pitch(quat) -> tuple[float, float]:
    """Chassis roll and pitch [deg] from a body quaternion."""
    R = quat_to_mat(quat)
    return (np.degrees(np.arctan2(R[2, 1], R[2, 2])),
            np.degrees(-np.arcsin(np.clip(R[2, 0], -1.0, 1.0))))


def settle_fallen(params: dict, roll_deg: float = 100.0, settle: float = 2.0,
                  wings: bool = False):
    """Drop the bike onto its side and let it stop moving; returns the qpos."""
    model = build_model(params, righting=True, wings=wings)
    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    a = np.deg2rad(roll_deg) / 2
    data.qpos[3:7] = [np.cos(a), np.sin(a), 0.0, 0.0]
    data.qpos[2] += 0.02
    mujoco.mj_forward(model, data)
    for _ in range(int(round(settle / model.opt.timestep))):
        mujoco.mj_step(model, data)
    return data.qpos.copy()


class RightingSequencer:
    """Drives one righting mechanism through lift -> balance -> retract.

    Owns the mechanism's `data.ctrl` entry throughout and the DriveController
    from hand-off onward. Call `step(model, data)` once per physics step,
    BEFORE `mj_step`; read `phase` for what it is doing.
    """

    def __init__(self, params: dict, model, *, wings: bool = False,
                 direction: float | None = None, rate: float = 0.7,
                 retract_after: float = 1.0, move: str = "general_rl",
                 design=None):
        self.params, self.wings = params, wings
        act, joint, self.cfg = mechanism(params, wings)
        self.aid = model.actuator(act).id
        self.jadr = model.joint(joint).qposadr[0]
        self.rate, self.retract_after, self.move = rate, retract_after, move
        self.design = design
        self.stow = np.deg2rad(self.cfg["stow_deg"])
        self.gear = self.cfg["gear_ratio"]
        # The wing pair deploys the same way whatever side it fell on; the
        # single arm has to reach for the floor it is lying on.
        self._fixed_direction = direction
        self.phase = "lift"
        self.t_hand = float("nan")
        self.ctrl = None
        self.cmd = 0.0
        self.direction = 1.0
        self._t = 0.0

    def reset(self, model, data) -> None:
        roll0, _ = roll_pitch(data.qpos[3:7])
        self.direction = (self._fixed_direction if self._fixed_direction is not None
                          else (float(np.sign(roll0)) or 1.0))
        self.cmd = float(data.qpos[self.jadr])
        self.phase, self.t_hand, self.ctrl, self._t = "lift", float("nan"), None, 0.0
        self.roll0 = roll0

    @property
    def stroke_deg(self) -> float:
        """How far the mechanism has travelled from stow [deg]."""
        return abs(np.degrees(self.cmd) - self.cfg["stow_deg"])

    @property
    def servo_turns(self) -> float:
        """Revolutions AT THE SERVO for the stroke so far. > 1 means the servo
        has to run in extended-position (multi-turn) mode."""
        return self.stroke_deg * self.gear / 360.0

    def step(self, model, data) -> None:
        dt = model.opt.timestep
        roll, _ = roll_pitch(data.qpos[3:7])
        if self.phase == "lift":
            self.cmd += self.direction * self.rate * dt
            if abs(roll) < RECOVER_DEG and abs(data.qvel[3]) < HANDOFF_RATE:
                self.phase, self.t_hand = "balance", self._t
                self.ctrl = DriveController(self.params, model, design=self.design)
                self.ctrl.reset(model, data)
                self.ctrl.engage_general(data, self.move)
                self.ctrl.set_command(v_cmd_world=(0.0, 0.0))
        else:
            self.ctrl.step(model, data)
            if self._t - self.t_hand > self.retract_after:
                self.phase = "retract"
                # Stop AT stow rather than sweeping through it and out the
                # other side.
                step = self.rate * dt
                self.cmd = (max(self.cmd - step, self.stow) if self.cmd > self.stow
                            else min(self.cmd + step, self.stow))
        data.ctrl[self.aid] = float(np.clip(self.cmd, -np.pi, np.pi))
        self._t += dt
