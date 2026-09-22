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

THE LOOP REGULATES BUS CURRENT, I_bus = duty * I_phase. That is what the
XC330's Present Current reads as (servo-measurements.yaml, `xc330_no_load`:
non-monotonic in duty at no load, a 16x inconsistency read as phase current),
what the XL330 stall sweep favours above 100 mA, and what the BAM discussion
concluded. Torque follows PHASE current, so at stall it is not kt * I_goal but

    duty = sqrt(I_goal / (s I_stall))   ->   tau = ts sqrt(s I_goal / I_stall)

-- 300 counts gives ~0.47 N.m at 12 V, not the 0.27 a phase reading implies.
The gain from error to torque therefore RISES toward zero error, which is the
amplitude dependence `r5_control_mode_comparison` measured.

In general, with x = w/w0, s = V/12 and the motor line tau = ts (s u - x), the
duty u that makes I_bus = I_stall u (s u - x) equal the demand is a root of

    s u^2 - x u - I_cmd/I_stall = 0

clipped to +-1, where the duty ceiling is the motor line itself. Measured on
both bench XC330s, bare shaft, Goal PWM 885: top speed 11.5-12.1 rad/s against
11.8 -- the ceiling holds.

WHEN THE DEMAND OPPOSES THE MOTION the quadratic has two candidate roots: a
spinning motor draws zero bus current both at u = x/s (no torque) and at u = 0
(windings shorted). `braking` picks one:

    "plug"   reverse duty -- the root an integrating regulator winds onto --
             braking up to full. THE DEFAULT, BECAUSE IT IS MEASURED: on both
             bench XC330s, 242 of 243 frames where the current opposed the
             motion had Present PWM of the OPPOSITE sign to the shaft speed
             (e.g. +9.2 rad/s at PWM -0.557), which only plugging gives.
    "regen"  the root continuous with zero torque at zero demand; braking is
             regenerative and saturates at half the short-circuit torque.
             Kept for sensitivity studies: 1 frame of 243 fits it.

The choice is not cosmetic. On the fall set, centring at hand-off recovers
2/2 at 12 V under "plug" and 1/2 under "regen", and 300 counts lifts the bike
only under "plug".

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
  * gearbox Coulomb friction -- unmeasured on the XC330-T181. By hand the
    XL330 back-drives easily at 200-300 mA, so it is well under the cap.

HOW IT ACTS. The native `swing` actuator is kept, so `ctrl[swing]` still means
the goal angle and every caller is unchanged, but its gains are zeroed on this
model; the torque goes in through `qfrc_applied` on the crank. `pre_step(data)`
must run immediately before every `mj_step`, as DrivetrainSim's does -- a loop
that forgets it gets a crank that never moves, which is loud rather than subtle.
"""

from __future__ import annotations

import math

import mujoco

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
#: note). Goal Current is clamped to it, as ServoBus.set_righting_current does.
CURRENT_LIMIT = 910
#: Frame -> the servo's own reference, measured 4.0 ms on both XC330s in mode 5
#: (servo-measurements.yaml, `xc330_command_delay`) -- frame-quantised, the
#: true value lies in (2, 4] ms. The same figure as the steer and the XC430s.
COMMAND_DELAY_S = 0.004


class CurrentBasedPositionServo:
    """Per-step state for the swing crank's servo. Build with `attach`."""

    @classmethod
    def attach(cls, model, params: dict, torque_nm: float | None = None,
               braking: str = "plug"):
        """None when the model has no swing linkage.

        `torque_nm` is the starting Goal Current expressed as the torque it
        gives AT STALL at 12 V -- pass the linkage config's `limits.torque_nm`
        so the model starts at the cap the mechanism was designed around. None
        starts from `control.onboard.righting_current`, the bike's own
        (untuned) counts."""
        try:
            model.joint("swing_crank_joint")
        except KeyError:
            return None
        return cls(model, params, torque_nm, braking)

    def __init__(self, model, params: dict, torque_nm: float | None = None,
                 braking: str = "plug"):
        if braking not in ("regen", "plug"):
            raise ValueError(f"braking: 'regen' or 'plug', got {braking!r}")
        self.braking = braking
        srv = params["servos"]["xc330_t181"]
        self.ts = float(srv["stall_torque"])
        self.w0 = float(srv["no_load_rpm"]) * 2 * math.pi / 60
        self.i_stall = float(srv["stall_current"])
        self.kt = self.ts / self.i_stall
        g = params["control"]["onboard"]["gains"]["righting"]
        self.kp_a = float(g["Position P Gain"]) * K5            # A/rad
        self.kd_a = float(g["Position D Gain"]) * KD_PER_UNIT   # A.s/rad
        self.aid = model.actuator("swing").id
        jid = model.joint("swing_crank_joint").id
        self.qadr = int(model.jnt_qposadr[jid])
        self.dadr = int(model.jnt_dofadr[jid])
        # The native actuator becomes a command holder: ctrl keeps its meaning,
        # the force is ours. Its ctrlrange still clamps the goal.
        model.actuator_gainprm[self.aid, 0] = 0.0
        model.actuator_biasprm[self.aid, :3] = 0.0
        # Stall torque -> bus current: tau = ts sqrt(I / I_stall) at 12 V.
        self.set_goal_current(
            round(self.i_stall * (torque_nm / self.ts) ** 2 / AMPS_PER_COUNT)
            if torque_nm is not None
            else params["control"]["onboard"]["righting_current"])
        self.supply = 1.0
        self.nc = max(0, round(COMMAND_DELAY_S / float(model.opt.timestep))) + 1
        self.torque = self.current = self.phase_current = self.duty = 0.0
        self._ready = False
        self._t_last = -math.inf

    @property
    def cap_nm(self) -> float:
        """Goal Current as the torque it gives at STALL, at the present supply:
        tau = ts * min(s, sqrt(s * I / I_stall))."""
        m = self.goal_current * AMPS_PER_COUNT / self.i_stall
        return self.ts * min(self.supply, math.sqrt(self.supply * m))

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
        self._cmd = [float(data.ctrl[self.aid])] * self.nc
        self._k = 0
        self.torque = self.current = self.phase_current = self.duty = 0.0
        data.qfrc_applied[self.dadr] = 0.0
        self._t_last = float(data.time)
        self._ready = True

    def pre_step(self, data) -> None:
        if not self._ready or data.time < self._t_last:
            self.reset(data)
        self._t_last = float(data.time)
        k, k1 = self._k, (self._k + 1) % self.nc
        self._cmd[k] = float(data.ctrl[self.aid])
        goal = self._cmd[k1]
        self._k = k1
        q = float(data.qpos[self.qadr])
        w = float(data.qvel[self.dadr])
        i_goal = self.goal_current * AMPS_PER_COUNT
        i_cmd = self.kp_a * (goal - q) - self.kd_a * w
        i_cmd = max(-i_goal, min(i_goal, i_cmd))
        u = self.duty_for(i_cmd, w)
        tau = self.ts * (self.supply * u - w / self.w0)
        self.duty = u
        self.torque = tau
        self.phase_current = tau / self.kt
        # What Present Current reads: bus-current MAGNITUDE, signed by the duty.
        # Measured while plugging: +9.2 rad/s, PWM -0.557, I -0.413 A -- a true
        # bus current would be positive there (the pack is supplying power).
        self.current = math.copysign(abs(u * self.phase_current), u)
        data.qfrc_applied[self.dadr] = tau

    def duty_for(self, i_bus: float, w: float) -> float:
        """The duty that makes the reported current -- bus magnitude, signed by
        the duty -- equal `i_bus` at shaft speed `w`, clipped to +-1."""
        s, x = self.supply, w / self.w0
        m = i_bus / self.i_stall
        sm = 1.0 if m > 0 else (-1.0 if m < 0 else (1.0 if x >= 0 else -1.0))
        if sm * x >= 0.0:                              # driving, or at stall
            u = sm * (abs(x) + math.sqrt(x * x + 4 * s * abs(m))) / (2 * s)
        elif self.braking == "regen":
            disc = x * x - 4 * s * abs(m)
            sx = 1.0 if x > 0 else -1.0
            u = sx * (abs(x) + math.sqrt(max(0.0, disc))) / (2 * s)
        else:                                          # plug
            u = sm * (-abs(x) + math.sqrt(x * x + 4 * s * abs(m))) / (2 * s)
        return max(-1.0, min(1.0, u))
