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

# Where the FOUR-BAR's servo torque peaks, in bike roll. Measured from
# analysis/wing_linkage.py --torque: 0.54 N.m at roll 53-57 deg, with the
# mechanism strong at both ends of the stroke and weak between them.
LINKAGE_PEAK_ROLL_DEG = 55.0
LINKAGE_EASE_BAND_DEG = 35.0


# Mechanism -> (actuator name, joint name, params sub-block). The stroke
# direction is not here because for the arm it depends on which way it fell.
MECHANISMS = {
    "arm": ("righting", "righting_joint", "arm"),
    "wings": ("wings", "wing_right_joint", "wings"),
    # The four-bar drives the CRANK, not a wing. Its stroke is therefore in
    # crank degrees (0..servo_travel_deg, ~180) rather than wing degrees
    # (0..deploy_deg, ~105), and there is no gear ratio to divide by -- the
    # ratio varies through the stroke. Its config also lives in its own file,
    # not in bike_params, so `mechanism` synthesises a matching sub-block.
    "linkage": ("wings", "wing_crank_joint", None),
}


def mechanism(params: dict, wings: bool, linkage: bool = False):
    """(actuator, joint, sub-block) for the selected mechanism."""
    if linkage:
        import yaml
        from ..build_model import LINKAGE_CFG
        cfg = yaml.safe_load(LINKAGE_CFG.read_text())
        act, joint, _ = MECHANISMS["linkage"]
        return act, joint, {
            "stow_deg": 0.0,
            "deploy_deg": float(cfg["stroke"]["servo_travel_deg"]),
            # No reduction: the actuator IS the crank, so servo turns are the
            # crank's own turns. Reporting a ratio here would be a lie that
            # `servo_turns` would then repeat.
            "gear_ratio": 1.0,
            "_linkage": cfg,
        }
    act, joint, key = MECHANISMS["wings" if wings else "arm"]
    return act, joint, params["righting"][key]


def roll_pitch(quat) -> tuple[float, float]:
    """Chassis roll and pitch [deg] from a body quaternion."""
    R = quat_to_mat(quat)
    return (np.degrees(np.arctan2(R[2, 1], R[2, 2])),
            np.degrees(-np.arcsin(np.clip(R[2, 0], -1.0, 1.0))))


def settle_fallen(params: dict, roll_deg: float = 100.0, settle: float = 2.0,
                  wings: bool = False, linkage: bool = False):
    """Drop the bike onto its side and let it stop moving; returns the qpos."""
    model = build_model(params, righting=True, wings=wings and not linkage,
                        linkage=linkage)
    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    a = np.deg2rad(roll_deg) / 2
    data.qpos[3:7] = [np.cos(a), np.sin(a), 0.0, 0.0]
    data.qpos[2] += 0.02
    mujoco.mj_forward(model, data)
    for _ in range(int(round(settle / model.opt.timestep))):
        mujoco.mj_step(model, data)
    return data.qpos.copy()


def settle_inverted(params: dict, roll_deg: float = 180.0, wings: bool = False,
                    settle: float = 4.0, drop: float = 0.02,
                    linkage: bool = False):
    """Drop the bike UPSIDE DOWN and let it find its own rest; returns the qpos.

    Not the same question as `settle_fallen`. On its side the bike is already
    where the mechanism can work; on its back it is not, and whether it gets
    there is a property of the roof ridge, not of the mechanism. Starting a run
    here exercises that first stage. See `self_righting.py invert`."""
    model = build_model(params, righting=True, wings=wings and not linkage,
                        linkage=linkage)
    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    a = np.deg2rad(roll_deg) / 2
    data.qpos[3:7] = [np.cos(a), np.sin(a), 0.0, 0.0]
    data.qpos[2] = 0.30
    mujoco.mj_forward(model, data)
    # Lower it to a real clearance first, so this is a drop and not the solver
    # recovering from a deep interpenetration.
    floor = model.geom("floor").id
    gap = min(mujoco.mj_geomDistance(model, data, floor, g, 2.0, None)
              for g in range(model.ngeom)
              if model.geom_contype[g] and g != floor)
    data.qpos[2] += drop - gap
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
                 linkage: bool = False,
                 direction: float | None = None, rate: float = 0.7,
                 recover_deg: float | None = None,
                 rate_max: float | None = 2.4, rate_ref_deg: float = 30.0,
                 rate_floor: float = 0.25, retract_rate: float | None = None,
                 retract_after: float = 1.0, move: str = "general_rl",
                 design=None, keep_policy: bool = False,
                 step_command: bool = False, torque_cap: float | None = None):
        self.params, self.wings, self.linkage = params, wings, linkage
        act, joint, self.cfg = mechanism(params, wings, linkage)
        self.aid = model.actuator(act).id
        # CURRENT-BASED POSITION MODE. The XC330 mode we actually intend to use
        # takes a position setpoint plus a goal CURRENT, and moves as fast as
        # it can under that cap -- there is no commanded trajectory at all. So
        # the faithful model is: clamp the actuator's forcerange to the goal
        # current, command the endpoint, and let the physics decide the speed.
        #
        # A ramped setpoint is a different mode and a slower one: measured,
        # the rate schedule made the linkage 2.14 s to hand-off where a step
        # command takes 0.15-0.33 s. It also changes what the stroke COSTS,
        # because a slow ramp fights gravity quasi-statically the whole way
        # while a step lets the stroke be dynamic.
        self.step_command = step_command
        if torque_cap is not None:
            # `torque_cap` is the goal current AT THE SERVO, so it has to be
            # multiplied back up by the reduction to become a forcerange, which
            # MuJoCo applies at the JOINT. The geared pair's joint is the wing
            # (forcerange = stall x gear_ratio); the linkage's is the crank
            # itself (gear_ratio 1). Applying a servo-side number directly as a
            # joint-side limit handicapped the gears by exactly the gear ratio
            # -- 4x -- and made them look incapable of current-based position
            # mode when they had simply been given a quarter of the torque.
            at_joint = torque_cap * self.cfg["gear_ratio"]
            model.actuator_forcerange[self.aid] = [-at_joint, at_joint]
        self.torque_cap = torque_cap
        self.jadr = model.joint(joint).qposadr[0]
        self.rate, self.retract_after, self.move = rate, retract_after, move
        self.rate_max = rate_max
        self.rate_ref_deg, self.rate_floor = rate_ref_deg, rate_floor
        # Retract is the UNLOADED stroke -- the bike is already upright and the
        # policy is holding it, so the wings are swinging through air and there
        # is no reason to creep. The only cost is the reaction torque into a
        # bike that is balancing, which is what bounds it.
        # Measured: 2.4 rad/s retracts in 0.57 s against 1.96 s at 0.7, and the
        # bike's worst roll during it is 5.6 deg EITHER WAY -- the reaction
        # torque simply is not the binding constraint, so creeping bought
        # nothing. Falls back to the flat `rate` only if rate_max is disabled.
        self.retract_rate = (retract_rate if retract_rate is not None
                             else (rate_max if rate_max is not None else rate))
        self.design = design
        # keep_policy: run the general policy THROUGHOUT, including while the
        # bike is down and the mechanism is lifting, instead of engaging it at
        # hand-off. Hand-off then becomes a command change rather than a
        # controller swap.
        #
        # Off by default because it is wrong for a FALL TEST -- a policy sawing
        # the bars against the floor is not a fall, and analysis/self_righting
        # deliberately cuts it once the fall is committed. It is right for
        # anything that has to look and behave like the real bike, where
        # nothing actually switches the policy off.
        self.keep_policy = keep_policy
        self.stow = np.deg2rad(self.cfg["stow_deg"])
        self.gear = self.cfg["gear_ratio"]
        # The wing pair deploys the same way whatever side it fell on; the
        # single arm has to reach for the floor it is lying on.
        self._fixed_direction = direction
        # Overridable so a study can ask "how close does the mechanism get?"
        # without editing the module constant. RECOVER_DEG is not a tuning
        # knob — it is what analysis/no_return.py measured the POLICY can
        # recover from — so raising it here is a question, not a fix.
        self.recover_deg = RECOVER_DEG if recover_deg is None else recover_deg
        self.phase = "lift"
        self.t_hand = float("nan")
        self.ctrl = None
        self.cmd = 0.0
        self.direction = 1.0
        self._t = 0.0

    def adopt(self, ctrl) -> None:
        """Take over an ALREADY RUNNING controller (implies keep_policy).

        Reusing the live one matters: constructing a fresh DriveController at
        hand-off re-engages the policy from a clean slate, which throws away
        its command state and any recurrent state the export carries."""
        self.ctrl, self.keep_policy = ctrl, True

    def reset(self, model, data) -> None:
        roll0, _ = roll_pitch(data.qpos[3:7])
        self.direction = (self._fixed_direction if self._fixed_direction is not None
                          else (float(np.sign(roll0)) or 1.0))
        self.cmd = float(data.qpos[self.jadr])
        self.phase, self.t_hand, self._t = "lift", float("nan"), 0.0
        if not self.keep_policy:
            self.ctrl = None
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

    def deploy_rate(self, roll_deg: float) -> float:
        """Commanded wing slew [rad/s], scheduled on how far over the bike is.

        A FLAT rate has to be sized by the worst moment of the stroke, and the
        worst moment is counter-intuitive: torque peaks at the END, 0.52 N.m
        inside 20 deg of upright, against only 0.13-0.15 N.m through the
        20-60 deg middle. The mechanism is cheap while it is levering and
        expensive while it is catching the bike as it arrives.

        So the schedule is fast far from upright and eases into hand-off --
        the opposite of "speed up as it comes up". Measured against a flat
        0.7 rad/s: 0.63 s to hand-off instead of 2.04 s (3.2x) at a slightly
        LOWER peak torque, because the easing removes the arrival spike.

        `rate` remains the flat fallback and is what the retract stroke uses;
        set `rate_max` to None to disable the schedule (the torque study wants
        a constant rate so its reading stays quasi-static)."""
        if self.rate_max is None:
            return self.rate
        if self.linkage:
            # INVERTED relative to the geared pair, because the torque peak is
            # somewhere else entirely. The gear train's worst moment is the
            # END of the stroke (catching the bike as it arrives); the
            # four-bar's is the MIDDLE -- mechanical advantage is high at both
            # ends (7.7:1 at stow, ~79:1 at full deployment, where the crank
            # approaches its input-side dead point) and worst in between.
            # Measured peak servo torque lands near 55 deg of roll.
            #
            # So ease through the middle and run fast at both ends, which is
            # the opposite shape. Reusing the geared schedule here would slow
            # the mechanism exactly where it is strongest and hurry it through
            # the one place it is weak.
            frac = np.clip(abs(abs(roll_deg) - LINKAGE_PEAK_ROLL_DEG)
                           / LINKAGE_EASE_BAND_DEG, self.rate_floor, 1.0)
            return self.rate_max * float(frac)
        frac = np.clip(abs(roll_deg) / self.rate_ref_deg, self.rate_floor, 1.0)
        return self.rate_max * float(frac)

    def step(self, model, data) -> None:
        dt = model.opt.timestep
        roll, _ = roll_pitch(data.qpos[3:7])
        if self.phase == "lift":
            if self.step_command:
                # Endpoint, immediately. The current cap is the only throttle.
                self.cmd = self.direction * np.deg2rad(self.cfg["deploy_deg"])
            else:
                self.cmd += self.direction * self.deploy_rate(roll) * dt
            # The policy keeps driving while the bike is down, if it was never
            # switched off. It has almost no authority on its side -- the rear
            # wheel is not under the CoM -- so this does not do the righting,
            # it just means nothing has to be re-engaged at hand-off.
            if self.keep_policy and self.ctrl is not None:
                self.ctrl.step(model, data)
            if abs(roll) < self.recover_deg and abs(data.qvel[3]) < HANDOFF_RATE:
                self.phase, self.t_hand = "balance", self._t
                if self.ctrl is None:
                    # Built one from scratch, so it has no command yet: park it.
                    self.ctrl = DriveController(self.params, model,
                                                design=self.design)
                    self.ctrl.reset(model, data)
                    self.ctrl.engage_general(data, self.move)
                    self.ctrl.set_command(v_cmd_world=(0.0, 0.0))
                # An ADOPTED controller keeps whatever it was last told. Hand-
                # off is a change of who is holding the bike up, not a change
                # of where it was going -- zeroing here put a visible step in
                # the commanded heading and velocity at the moment of recovery,
                # for no reason. The caller owns the command; the sequencer
                # owns the mechanism.
        else:
            self.ctrl.step(model, data)
            if self._t - self.t_hand > self.retract_after:
                self.phase = "retract"
                if self.step_command:
                    self.cmd = self.stow
                    data.ctrl[self.aid] = float(np.clip(self.cmd, -np.pi, np.pi))
                    self._t += dt
                    return
                # Stop AT stow rather than sweeping through it and out the
                # other side.
                step = self.retract_rate * dt
                self.cmd = (max(self.cmd - step, self.stow) if self.cmd > self.stow
                            else min(self.cmd + step, self.stow))
        data.ctrl[self.aid] = float(np.clip(self.cmd, -np.pi, np.pi))
        self._t += dt
