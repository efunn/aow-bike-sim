"""The righting servo in CURRENT-BASED POSITION MODE (5), as the firmware runs it.

The swing linkage's crank is otherwise a MuJoCo position actuator with a flat
torque clip, `clip(kp e - kv w, +-cap)`. That is the right SHAPE for mode 5 and
the wrong LIMIT: it has no supply voltage and no back-EMF, so the crank can turn
faster than the motor can. Measured on the fall set: the bare clip peaks at
17.5 rad/s where the XC330-T181's no-load speed is 11.8 at 12 V, and it throws
the bike through upright at 419 deg/s where this model gives 233.

The firmware chain, per ROBOTIS's mode-5 block diagram:

    position PID -> current demand -> clip at Goal Current(102)
                 -> current controller -> duty -> inverter -> motor

THE CURRENT LAW IS MEASURED (X330 fixture, 2026-09-26; the numbers and the
captures are in docs/plans/righting-servo-model.md). Above a firmware drive
edge the motor's torque is linear in the current demand:

    tau_m = k sign(i) max(0, |i| - I0),   k 0.83 N m/A, I0 16.5 mA

-- lifted loads of 8-59 mN m at 35-129 mA on one straight line, and one of
102 mN m at 189, slightly under it. Below the edge the drive is off: Present
PWM ~0 at stall through 16 mA, on at 17, both directions. `k` sits near the
datasheet's kt (0.80 / 0.88 = 0.91).

REJECTED, the law this module used before: that the loop regulates BUS
current, I_bus = duty x I_phase, so at stall tau = ts sqrt(I / I_stall) (300
counts -> ~0.47 N m). It came from Present Current's behaviour at no load and
an XL330 fit of Present Current against Present PWM, and no torque was ever
measured against it. The lever measured torque: 59 mN m needed ~110 mA past
the edge where the sqrt law says ~5, and 300 counts gives ~0.24 N m of motor
torque, not 0.47.

The motor line still bounds it: with x = w/w0 and s = V/12 the duty is
u = (tau/ts + x)/s, clipped to +-1, so the torque stays inside
ts (s u - x) for |u| <= 1. Measured on both bench XC330s, bare shaft: top speed
11.5-12.1 rad/s against 11.8 -- the ceiling holds. Braking comes out as
measured ("plug", 242 of 243 frames with the duty REVERSED against the
motion): a braking torque larger than ts x needs a reversed duty, and nothing
here chooses otherwise.

GEARBOX FRICTION is on the crank's dof, load-proportional, from the same
bench: gearbox_friction.py. `pre_step` hands it this step's torque.

GOAL PWM IS NOT A DUTY CEILING IN THIS MODE, measured, so the model has no
Goal PWM knob. Below 885 it caps what Present PWM REPORTS exactly, but the
shaft barely slows: Goal PWM 0.4 runs at ~0.80 of full speed in mode 5, where
duty 0.4 in PWM mode runs at 0.39 (`xc330_current_position`). What it does
limit is unknown, so it is left at 885, the factory value.

Gains are the FIRMWARE's, read from `control.onboard.gains.righting` -- the same
registers the bike writes at startup -- and converted to the PID's output in
amps as bike_params.yaml's `righting.arm.servo_kp/kv` note derives them.
Nothing here reads a key `plant_digest` hashes: `control` is outside it.

What is NOT modelled, and why:

  * the current controller's own bandwidth -- taken as instantaneous;
  * the D term on the ERROR: the firmware may differentiate e, which kicks on a
    goal step; this uses -w, as the native actuator does. The per-tick SCALE
    is supported to order of magnitude (a bare-shaft fit gives 0.25-0.7x it,
    against 1000x for a true d/dt), the exact value is not;
  * the law above ~200 mA: unmeasured (the fixture's printed hub sheared at
    ~0.1-0.2 N m); the line is extrapolated to the 910 mA Current Limit;
  * BRAKING MAGNITUDE, a known mismatch: the bare-shaft frame that showed
    plugging (+9.2 rad/s, Present PWM -0.557, Present Current -0.413 A) implies
    ~1.07 N m of braking by the motor line, where k (I - I0) gives 0.33 and a
    POSITIVE duty. The lever measured the law at stall only; while braking at
    speed the current reading, or the law, differs. The model under-brakes
    there, i.e. overshoots more than the bench servo would;
  * holding's dependence on approach direction, angle and history
    (documented in the plan, not modelled).

HOW IT ACTS. The native `swing` actuator is kept, so `ctrl[swing]` still means
the goal angle and every caller is unchanged, but its gains are zeroed on this
model; the torque goes in through `qfrc_applied` on the crank. `pre_step(data)`
must run immediately before every `mj_step`, as DrivetrainSim's does -- a loop
that forgets it gets a crank that never moves, which is loud rather than subtle.
"""

from __future__ import annotations

import math

from .gearbox_friction import GearboxFriction

#: A/(Kp.rad): the mode-5 position PID's output, read as milliamps. Derived
#: (4096/2pi)/(256*1000); measured 2.525e-3 on an X330. See bike_params.yaml.
K5 = (4096 / (2 * math.pi)) / (256 * 1000)
#: A.s/rad per unit of Position D Gain: (D/16) * (256 k5) * 1 ms loop tick.
#: The soft number -- the per-tick reading is an assumption, see bike_params.
KD_PER_UNIT = (1 / 16) * (256 * K5) * 1e-3
#: A per Goal Current count. The e-manual's ~1 mA/LSB; the vendored model file
#: disagrees, and Station C (R6) in first-physical-test.md settles it.
AMPS_PER_COUNT = 1.0e-3
#: Current Limit(38) on this XC330-T181 (bike_params.yaml, righting_current's
#: note). Goal Current is clamped to it, as BikeBus.set_righting_current does.
CURRENT_LIMIT = 910
#: Frame -> the servo's own reference, measured 4.0 ms on both XC330s in mode 5
#: (servo-measurements.yaml, `xc330_command_delay`) -- frame-quantised, the
#: true value lies in (2, 4] ms. The same figure as the steer and the XC430s.
COMMAND_DELAY_S = 0.004


class CurrentBasedPositionServo:
    """Per-step state for the swing crank's servo. Build with `attach`."""

    @classmethod
    def attach(cls, model, params: dict, torque_nm: float | None = None,
               at_output: bool = False):
        """None when the model has no swing linkage.

        `torque_nm` is the starting Goal Current expressed as the MOTOR torque
        it gives at stall at 12 V -- or, with `at_output`, as the torque the
        crank delivers while MOVING, after running gearbox friction. Pass the
        linkage config's `limits.torque_nm` (the peak the stroke needs at the
        crank) with `at_output` so the model starts at a cap that can do the
        stroke. None starts from `control.onboard.righting_current`, the
        bike's own (untuned) counts."""
        try:
            model.joint("swing_crank_joint")
        except KeyError:
            return None
        srv = cls(model, params, torque_nm if not at_output else None)
        if torque_nm is not None and at_output:
            srv.set_goal_current(srv.counts_for_output(torque_nm))
        return srv

    def __init__(self, model, params: dict, torque_nm: float | None = None,
                 joint: str = "swing_crank_joint",
                 actuator: str | None = "swing", gains: dict | None = None,
                 goal_current: int | None = None, friction_base: float = 0.0):
        """Defaults are the swing crank's. Any other hinge works: `actuator`
        None takes the goal from `self.goal` [rad] instead of a ctrl slot,
        `gains` overrides the Position P/D Gain registers, and `goal_current`
        [counts] overrides `torque_nm`. The floor rig's servos use all four,
        and `friction_base` [N m], the axis's own bearing friction, which the
        gearbox's adds to."""
        srv = params["servos"]["xc330_t181"]
        self.ts = float(srv["stall_torque"])
        self.w0 = float(srv["no_load_rpm"]) * 2 * math.pi / 60
        self.k = float(srv["current_torque_gain"])
        self.i0 = float(srv["current_deadband"])
        g = {**params["control"]["onboard"]["gains"]["righting"], **(gains or {})}
        self.kp_a = float(g["Position P Gain"]) * K5            # A/rad
        self.kd_a = float(g["Position D Gain"]) * KD_PER_UNIT   # A.s/rad
        self.aid = None if actuator is None else model.actuator(actuator).id
        self.goal = 0.0
        jid = model.joint(joint).id
        self.qadr = int(model.jnt_qposadr[jid])
        self.dadr = int(model.jnt_dofadr[jid])
        if self.aid is not None:
            # The native actuator becomes a command holder: ctrl keeps its
            # meaning, the force is ours. Its ctrlrange still clamps the goal.
            model.actuator_gainprm[self.aid, 0] = 0.0
            model.actuator_biasprm[self.aid, :3] = 0.0
        self.friction = GearboxFriction(model, params, joint, base=friction_base)
        self.set_goal_current(
            goal_current if goal_current is not None
            else self.counts_for(torque_nm) if torque_nm is not None
            else params["control"]["onboard"]["righting_current"])
        self.supply = 1.0
        self.nc = max(0, round(COMMAND_DELAY_S / float(model.opt.timestep))) + 1
        self.torque = self.current = self.duty = 0.0
        self._ready = False
        self._t_last = -math.inf

    def counts_for(self, torque_nm: float) -> int:
        """Goal Current [counts] whose stall motor torque is `torque_nm`."""
        return round((self.i0 + float(torque_nm) / self.k) / AMPS_PER_COUNT)

    def counts_for_output(self, torque_nm: float) -> int:
        """Goal Current [counts] that delivers `torque_nm` at the output while
        moving: the motor torque that leaves it after running friction,
        (1 + c) tau + f0."""
        f = self.friction
        return self.counts_for((1.0 + f.rc) * float(torque_nm) + f.r0)

    @property
    def output_nm(self) -> float:
        """`cap_nm` as the output torque while moving, after running friction."""
        f = self.friction
        return max(0.0, (self.cap_nm - f.r0) / (1.0 + f.rc))

    @property
    def cap_nm(self) -> float:
        """Goal Current as the motor torque it gives at STALL, at the present
        supply, before gearbox friction."""
        i = self.goal_current * AMPS_PER_COUNT
        return min(self.ts * self.supply, self.k * max(0.0, i - self.i0))

    def set_goal_current(self, counts) -> int:
        """Goal Current(102) in counts, clamped to [0, Current Limit]."""
        self.goal_current = int(max(0, min(CURRENT_LIMIT, int(counts))))
        return self.goal_current

    def set_supply(self, scale: float) -> None:
        """Battery voltage as a fraction of 12 V; see DrivetrainSim.set_supply."""
        self.supply = float(scale)

    def reset(self, data) -> None:
        """Clear the command pipeline at the CURRENT goal, so a reset does not
        replay a stale setpoint."""
        self._cmd = [self._goal_now(data)] * self.nc
        self._k = 0
        self.torque = self.current = self.duty = 0.0
        data.qfrc_applied[self.dadr] = 0.0
        self.friction.reset(data)
        self._t_last = float(data.time)
        self._ready = True

    def pre_step(self, data) -> None:
        if not self._ready or data.time < self._t_last:
            self.reset(data)
        self._t_last = float(data.time)
        k, k1 = self._k, (self._k + 1) % self.nc
        self._cmd[k] = self._goal_now(data)
        goal = self._cmd[k1]
        self._k = k1
        q = float(data.qpos[self.qadr])
        w = float(data.qvel[self.dadr])
        i_goal = self.goal_current * AMPS_PER_COUNT
        i_cmd = self.kp_a * (goal - q) - self.kd_a * w
        i_cmd = max(-i_goal, min(i_goal, i_cmd))
        tau, u, sat = self.torque_for(i_cmd, w)
        self.duty = u
        self.torque = tau
        # What Present Current reads: the demand, exactly, while the loop can
        # meet it (at stall the bench read the goal to the mA, below the edge
        # too); what the torque implies once the duty saturates.
        self.current = (i_cmd if not sat else
                        math.copysign(abs(tau) / self.k + self.i0, tau) if tau else 0.0)
        self.friction.update(data, tau)
        data.qfrc_applied[self.dadr] = tau

    def _goal_now(self, data) -> float:
        return self.goal if self.aid is None else float(data.ctrl[self.aid])

    def torque_for(self, i_cmd: float, w: float) -> tuple[float, float, bool]:
        """(motor torque, duty, duty saturated) for a current demand `i_cmd`
        [A] at shaft speed `w`: the measured law, inside the motor line."""
        a = abs(i_cmd) - self.i0
        if a <= 0.0:
            # Below the edge the drive is OFF, not shorted: the release tests
            # found 0 mA and torque off indistinguishable, so no braking here.
            return 0.0, 0.0, False
        tau = math.copysign(self.k * a, i_cmd)
        s, x = self.supply, w / self.w0
        u = (tau / self.ts + x) / s
        if u > 1.0 or u < -1.0:
            u = 1.0 if u > 0.0 else -1.0
            return self.ts * (s * u - x), u, True
        return tau, u, False
