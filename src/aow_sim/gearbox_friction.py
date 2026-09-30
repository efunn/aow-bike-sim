"""Load-proportional gearbox friction for the XC330-T181, on any hinge it drives.

Measured on the X330 fixture (docs/plans/righting-servo-model.md, "What the
bench says"), one gearbox seen through two firmware modes:

    static   0.51 x load + 5.1 mN m   mode 5, four loads (mode 3: 0.53, 7.0)
    running  0.15 x load + 5.7 mN m   mode 3, the joint fit of four speed lines

where LOAD is the torque the gearbox passes to what it drives. Friction that
scales with the transmitted torque is what makes the servo lift a load at
`(tau - f0) / (1 + c)` but hold one up to `(tau + f0) / (1 - c)`: driving, the
friction opposes the motor; back-driven, it helps it.

HOW. MuJoCo's own dry friction (`dof_frictionloss`, a constraint row the solver
sticks and slips) with its LIMIT reset before every `mj_step`. The limit is
the fixed point of F = f0 + c L with L the transmitted torque:

    friction opposes the motor (driving):   L = tau - F   ->  F = (f0 + c tau) / (1 + c)
    friction helps it (back-driven):        L = tau + F   ->  F = (f0 + c tau) / (1 - c)

static (f0, c) below `stick_speed`, running above. Those two limits put the
thresholds exactly on the fitted lines -- lift above (tau - f0)/(1 + c) of
load, give way above (tau + f0)/(1 - c) -- and depend only on WHICH branch,
not on a lagged friction magnitude (an earlier version fed last step's force
back in, and a suddenly applied load slipped before it converged). Moving,
the branch is the sign of w x tau; at rest, the sign of the friction force
the solver applied last step. With no motor torque the limit is f0/(1 - c):
~10 mN m holding a released load, against the bench's ~8.5.
COST, measured 2026-09-26 on the main model (M4, median of 6 interleaved
blocks): `pre_step` itself ~0.8 us, and `mj_step` 18.3 -> 19.8 us with the
steer's hook against the static value alone (~+8 %, the solver's share
included). MjData attribute access is most of the Python part.

The friction force's SIGN is read from `efc_force` at a FIXED row: MuJoCo orders
constraints equality, dof friction, tendon friction, limits, contacts, and a
dof's friction row exists iff its frictionloss > 0. So the row is `data.ne` +
the number of lower dofs with non-zero frictionloss. Every dof this module
owns keeps frictionloss >= f0 > 0, so its row never disappears; the count of
LOWER rows is taken at `reset`, after every owner has attached. A test checks
the row against `efc_type` / `efc_id` (test_gearbox_friction.py).

A loop that never calls `pre_step` keeps the model's build-time value --
`static_nm`, the no-load static friction (build_model sets it) -- a constant
approximation, not zero.

THE STEER'S HEADSET (2026-09-30). `attach_steer` adds the printed bushing
and thrust face under the gearbox: a + b x |T| [N m], T the axial load
through the headset, read each step from the `headset_force` sensor's z
(the chassis's force on the steer body along the axis, one step old). The
joint's limit is then headset + gearbox(tau), and
with the gearbox passing c of its load on, a motor turning it steadily needs
(1 + c) x headset + f0 -- the bench's line (bike.steering, bike_params). The
build-time value and `release()` stay the gearbox's static alone, so a loop
without the hook (the LQR design) is unchanged. Cost: one sensordata read.

What is NOT modelled (documented, agreed): holding depends on the approach
direction, the angle and the history (the release sweeps); and the static
fraction is from a static test, the running one from a slow one.
"""

from __future__ import annotations


#: The friction constraint's softness WHERE A SERVO HOLDS A GRAVITY LOAD (the
#: righting crank, the rig's axes). MuJoCo's default (solref 0.02 s, solimp
#: 0.9/0.95) lets a load BELOW the friction limit creep at 0.08-0.32 rad/s on
#: the fixture's lever (34-137 g at 44 mm, measured 2026-09-26) -- above
#: `stick_speed`, so a held load would slide out onto running friction. At a
#: time constant of two timesteps and 0.99/0.999 the creep is ~0.001 rad/s.
#:
#: NOT ON THE STEER, which never holds a gravity load. There the stiff row
#: made the analytic LQR's 0.8 m/s U-turn fragile: 16 solref/solimp/hook
#: combinations fell or held with no trend (a knife edge), while at the
#: default softness it held 14.5-15.3 deg of roll for ANY constant friction
#: from 0 to 10 mN m, and with the load-proportional hook (2026-09-26).
SOLIMP = [0.99, 0.999, 0.001, 0.5, 2.0]


def solref(timestep: float) -> list[float]:
    return [max(2.0 * float(timestep), 2e-4), 1.0]


def constants(params: dict, ratio: float = 1.0) -> dict:
    """The friction constants AT A JOINT that turns 1/`ratio` of the servo's
    output (the steering's `gear_ratio`: servo rotation per steer rotation).
    Constant terms scale with the ratio; the fractions are dimensionless."""
    srv = params["servos"]["xc330_t181"]
    return {
        "static_nm": float(srv["friction_static_nm"]) * ratio,
        "static_frac": float(srv["friction_static_fraction"]),
        "running_nm": float(srv["friction_running_nm"]) * ratio,
        "running_frac": float(srv["friction_running_fraction"]),
        "stick_speed": float(srv["friction_stick_speed"]) / ratio,
    }


class GearboxFriction:
    """Friction on one hinge. The motor torque comes from `update(data, tau)`
    (a servo model that computes its own torque), or from the joint's actuator
    and damping via `pre_step(data)` (the steer: a MuJoCo position actuator
    whose back-EMF droop is the joint's damping).

    Written for the substep loop: MjData attribute access costs 60-170 ns
    each (measured), so the fixed-size views are bound once per MjData and
    `efc_force` is read only at rest."""

    def __init__(self, model, params: dict, joint: str, ratio: float = 1.0,
                 base: float = 0.0, holds_load: bool = True):
        """`base` [N m] is the joint's OWN friction (a bearing), kept under
        the gearbox's. `holds_load` stiffens the friction row (see SOLIMP);
        False leaves MuJoCo's default softness (the steer)."""
        self.model = model
        self.base = float(base)
        c = constants(params, ratio)
        self.s0, self.sc = c["static_nm"], c["static_frac"]
        self.r0, self.rc = c["running_nm"], c["running_frac"]
        self.w_stick = c["stick_speed"]
        jid = model.joint(joint).id
        self.dof = int(model.jnt_dofadr[jid])
        self._fl = model.dof_frictionloss
        self._damp = float(model.dof_damping[self.dof])
        self._fl[self.dof] = self.base + self.s0
        if holds_load:
            model.dof_solref[self.dof] = solref(model.opt.timestep)
            model.dof_solimp[self.dof] = SOLIMP
        self.load = 0.0
        self._below = None
        self._data = None
        self._t = -1.0
        self._touch = None      # the steer's headset term: see thrust()

    def thrust(self, adr: int, a: float, b: float) -> None:
        """Add a load-dependent bearing under the gearbox: a + b x |T|, with
        T = sensordata[adr] [N], read every step."""
        self._touch = int(adr)
        self._ha, self._hb = float(a), float(b)

    def reset(self, data) -> None:
        """Bind to `data`, count the friction rows below this dof, and start
        from no load."""
        fl = self._fl
        self._below = int((fl[:self.dof] > 0).sum())
        fl[self.dof] = self.base + self.s0
        self.load = 0.0
        self._data = data
        self._qvel = data.qvel
        self._qfa = data.qfrc_actuator
        self._sd = data.sensordata
        self._t = data.time

    def release(self) -> None:
        """Put the model's build-time value back, for a loop that hands the
        model on to code that does not step this hook."""
        self._fl[self.dof] = self.base + self.s0

    def row(self, data) -> int:
        if self._below is None:
            self.reset(data)
        return data.ne + self._below

    def update(self, data, tau: float) -> None:
        """Set this step's friction limit from the motor torque `tau` [N m at
        the joint]."""
        t = data.time
        if data is not self._data or t < self._t:
            self.reset(data)        # first step, a new MjData, or a reset one
        self._t = t
        w = self._qvel[self.dof]
        a = tau if tau >= 0.0 else -tau
        if -self.w_stick < w < self.w_stick:
            f0, c = self.s0, self.sc
            # At rest: which way the solver's friction pushed last step says
            # whether it holds the motor back (driving) or helps it (the
            # load is winning). No force yet, or no torque: helping.
            r = data.ne + self._below
            helping = r >= data.nefc or data.efc_force[r] * tau >= 0.0
        else:
            f0, c = self.r0, self.rc
            helping = w * tau < 0.0                 # the load drives the motor
        if helping:
            F = (f0 + c * a) / (1.0 - c)
            self.load = a + F
        else:
            F = (f0 + c * a) / (1.0 + c)
            self.load = a - F
        if self._touch is None:
            self._fl[self.dof] = self.base + F
        else:
            T = self._sd[self._touch]
            self._fl[self.dof] = self._ha + self._hb * (T if T >= 0.0 else -T) + F

    def pre_step(self, data) -> None:
        """For a joint driven by a MuJoCo actuator: the motor torque is the
        actuator's force on this dof minus the joint's damping (the steer's
        back-EMF droop lives there under `steer_clip: duty`)."""
        if data is not self._data:
            self.reset(data)
        d = self.dof
        self.update(data, self._qfa[d] - self._damp * self._qvel[d])


def attach(model, params: dict, joint: str, ratio: float = 1.0,
           holds_load: bool = True) -> GearboxFriction | None:
    """Friction on `joint`, or None when the model has no such joint."""
    try:
        model.joint(joint)
    except KeyError:
        return None
    return GearboxFriction(model, params, joint, ratio=ratio, holds_load=holds_load)


def attach_steer(model, params: dict) -> GearboxFriction | None:
    """The steering servo's friction, referred through the steering gear,
    plus the headset's under it when the model has the `headset_force` sensor
    and bike.steering carries the headset constants (module docstring)."""
    st = params["bike"]["steering"]
    g = attach(model, params, "steer_joint", ratio=float(st["gear_ratio"]),
               holds_load=False)
    if g is None or "headset_friction_nm" not in st:
        return g
    try:
        adr = int(model.sensor("headset_force").adr[0]) + 2      # z: along the axis
    except KeyError:
        return g
    g.thrust(adr, float(st["headset_friction_nm"]), float(st["headset_friction_per_n"]))
    return g


def attach_native(model, params: dict) -> list[GearboxFriction]:
    """Friction for every XC330 driven by a NATIVE MuJoCo actuator: the steer,
    and the swing crank when no CurrentBasedPositionServo owns it (that
    model carries its own). Call each one's `pre_step(data)` before every
    `mj_step`, and `reset(data)` on an episode reset."""
    out = [attach_steer(model, params), attach(model, params, "swing_crank_joint")]
    return [f for f in out if f is not None]
