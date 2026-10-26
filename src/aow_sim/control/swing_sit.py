"""Lying down and getting up on the swing four-bar, scripted.

docs/plans/wishlist.md, "getting up and down cleanly". `righting.py`'s
`RightingSequencer` drives the older mechanisms (arm, geared wings, the first
linkage); this drives the bike's own module, the co-rotating swing pair on one
XC330 crank, in current-based position mode. Crank +deploy puts the RIGHT
blade down (it touches a level floor at the full stroke with the bike
upright), -deploy the LEFT. Roll is +ve leaning RIGHT.

DOWN, as gently as the module allows:

    stop     the policy brings the bike to rest (the caller zeroes the
             command); nothing else moves until it is below `still_speed`.
    deploy   crank to `lean_deg` on the chosen side, the policy still
             balancing. NOT the full stroke: at 116.9 deg the blade already
             touches the floor upright, and when the policy is cut it props
             the bike over the OTHER way -- every trial fell away from it.
    kick     policy off, drives held, steer held; the rear differential moves
             the rear contact away from the blade, so the bike leans onto
             it -- held until it leans `lean_on_deg` that way (closed loop).
             If it goes `abort_deg` the OTHER way instead, the policy gets
             the bike back and the blade folds away (`aborted`).
    settle   drives held; the bike comes to rest on wheels + blade (~6 deg).
    lower    crank ramped back to centre at `lower_dps`; the bike rolls down
             onto its side. With `rest_deg` > 0 the ramp carries on THROUGH
             centre to `rest_deg` the other way: where the bike, lying on
             its cage, comes to rest. Stopping at centre left that last
             stretch to gravity, which on the bike is a CLUNK when the
             current goes. The bike's rest is MEASURED at 26.5-28 deg
             (run_bike.LIE_REST_DEG); the sim's is ~77 deg, so a sim run
             that wants the bike's behaviour sets this from the sim.
    limp     Goal Current ramped to 0 over `limp_s`. Held at full current the
             crank sits ~12 deg past centre against the bike's weight and
             limit-cycles (+-4 deg of roll, ~35 deg/s, indefinitely); dropped
             to 0 at once the linkage back-drives and the bike drops the last
             few degrees hard.
    lying    done; nothing driven.

UP, from lying (on purpose or after a fall):

    lift     crank goal RAMPED at `lift_dps` to the full stroke on the side
             the bike is lying on, policy off. A step command throws the
             bike through upright at ~300 deg/s, outside the hand-off gate,
             and over onto its other side.
    balance  the policy takes over once `recovery.ready_for_policy` says so;
             the blade eases back to `release_deg` at `release_dps` -- the
             policy leans ON the planted blade at first, and folding it at
             once dropped the bike onto that side.
    retract  once upright (`upright_deg`) and after `retract_after` s:
             crank to centre at `retract_dps`.
    riding   done.

UP WITHOUT THE HAND-OFF (`hand_off=False`, the real bike's default since
2026-10-26): the lift ramps to `lean_deg` instead of the full stroke --
the lie-down's `settle` pose, the bike resting on wheels + blade -- and
stops there:

    lift     as above, to `lean_deg`; done once the ramp is there and the
             bike is inside the recoverable set (`ready_for_policy`).
    propped  holding. The drives LIMP (`Command.limp`), the blade at
             `lean_deg`. Left by `engage()`, the operator's go-ahead, into
             `balance` and `retract` as above -- or by `lie_from_prop()`.

DOWN FROM A PROP (`lie_from_prop`): a bike already resting on a blade --
`propped`, or powered up that way -- needs no stop, deploy or kick. It
starts at `lower`, from wherever the crank IS, and the policy never runs.

Not the full stroke: there the blade props an upright bike over the OTHER
way once nothing balances it (see `deploy` above).

A LIFT THAT TIMES OUT goes limp and lies, then tries again after `retry_s`
(None: waits for the operator, as before 2026-10-23). The auto-up only fires
from `riding`, so without this a failed get-up stayed down until a key --
seen after repeated knockdowns left the bike rolled onto its cage, where the
lift jackknifes the blade against the shell and pushes the bike the wrong
way; a second press usually worked. Over 20 knockdowns in a row (the web
demo's sim, 3 N shoves, some doubled) all 20 were up in 4.2-7.8 s, 3 on a
retry. A lie-down the operator asked for is not retried: it stays lying.

Every default here came from a sweep in `analysis/moves/swing_sit.py`
(2026-10-16, default bike, ideal drivetrain), whose docstring has the
numbers. Teleop: PageDown / PageUp, and auto-up after a fall.

Pure numpy, no mujoco: the caller reads roll, roll rate and the crank angle
from whatever it has (mjData in sim, the AHRS and the servo on the bike) and
applies `Command`.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .recovery import RECOVER_DEG, ready_for_policy


@dataclass
class Command:
    """What the caller applies this step.

    crank        crank goal [rad]
    current      Goal Current as a fraction of the caller's own (0..1)
    policy       True: the general policy drives. False: hold the drives at
                 zero speed and the steer where it is, plus `diff`.
    diff         rear differential [rad/s] while the policy is off
    limp         the drives and steer unpowered rather than held (`propped`)
    """
    crank: float
    current: float = 1.0
    policy: bool = True
    diff: float = 0.0
    limp: bool = False


class SwingSit:
    #: phases in which the sequence owns the bike
    ACTIVE = ("stop", "deploy", "kick", "settle", "lower", "limp", "lift", "balance",
              "retract")

    def __init__(self, deploy_rad: float, *,
                 lean_deg: float = 110.0, deploy_s: float = 0.6,
                 kick: float = 20.0, settle_s: float = 0.4,
                 lean_on_deg: float = 3.0, kick_max_s: float = 1.5,
                 abort_deg: float = 8.0,
                 release_deg: float = 95.0, release_dps: float = 30.0,
                 upright_deg: float = 3.0, balance_max_s: float = 4.0,
                 retract_dps: float = 120.0,
                 still_speed: float = 0.05, stop_max_s: float = 4.0,
                 lower_dps: float = 30.0, limp_s: float = 1.0,
                 rest_deg: float = 0.0,
                 lift_timeout_s: float = 3.0, retract_after: float = 1.0,
                 lift_policy: bool = False, lift_dps: float | None = 120.0,
                 recover_deg: float = RECOVER_DEG,
                 fallen_deg: float = 60.0, fallen_s: float = 0.5,
                 still_rate: float = 2.0, retry_s: float | None = 1.0,
                 hand_off: bool = True):
        self.deploy = float(deploy_rad)
        self.lean = np.deg2rad(lean_deg)
        self.deploy_s, self.kick = deploy_s, kick
        self.settle_s, self.lower = settle_s, np.deg2rad(lower_dps)
        self.rest = np.deg2rad(rest_deg)
        self.limp_s, self.lift_timeout_s = limp_s, lift_timeout_s
        self.retract_after, self.recover_deg = retract_after, recover_deg
        self.fallen_rad, self.fallen_s = np.deg2rad(fallen_deg), fallen_s
        self.still_rate = still_rate
        self.retry_s = retry_s
        self.hand_off = bool(hand_off)
        self._retry = False         # lying after a failed lift: try again
        self.lean_on, self.kick_max_s = np.deg2rad(lean_on_deg), kick_max_s
        self.abort = np.deg2rad(abort_deg)
        self.release, self.release_rate = np.deg2rad(release_deg), np.deg2rad(release_dps)
        self.upright, self.balance_max_s = np.deg2rad(upright_deg), balance_max_s
        self.retract_rate = np.deg2rad(retract_dps)
        self.still_speed, self.stop_max_s = still_speed, stop_max_s
        self._retract_from = 0.0
        self.lift_policy = lift_policy
        self.lift_rate = None if lift_dps is None else np.deg2rad(lift_dps)
        self._lift_from = None
        self._hold = 0.0
        self.aborted = False
        self._fell = False
        self.phase = "riding"
        self.side = 1.0
        self._t0 = 0.0
        self._crank = 0.0
        self._down_since = None

    @property
    def active(self) -> bool:
        return self.phase in self.ACTIVE

    def _go(self, phase, t):
        if phase in ("lift", "stop"):
            self._fell = False          # a fresh attempt
        self.phase, self._t0 = phase, t

    def lie_down(self, t: float, roll: float = 0.0, side: float | None = None) -> None:
        """Start lying down onto `side` (+1 right, -1 left; default: the way
        the bike already leans, right if level)."""
        self.side = float(side if side is not None else (np.sign(roll) or 1.0))
        self.aborted = False
        self._retry = False
        self._go("stop", t)

    def get_up(self, t: float, roll: float, side: float | None = None) -> None:
        """Start getting up from lying on the side `roll` says (or `side`)."""
        self.side = float(side if side is not None else (np.sign(roll) or 1.0))
        self._lift_from = None
        self._down_since = None     # the next auto-up waits its own fallen_s
        self._retry = False
        self._go("lift", t)

    def lie_from_prop(self, t: float, crank: float) -> None:
        """Lie down from resting on the blade at `crank` [rad]: straight to
        `lower`, ramped from there. The side is the crank's."""
        self.side = float(np.sign(crank) or 1.0)
        self._crank = float(crank)
        self.aborted = False
        self._retry = False
        self._go("lower", t)

    def engage(self, t: float) -> None:
        """`propped` -> `balance`: the policy takes the bike and the blade
        eases off and folds, as after an automatic hand-off."""
        if self.phase != "propped":
            raise RuntimeError(f"engage from {self.phase}, not propped")
        self._hold = self.side * self.lean
        self._go("balance", t)

    def _abort(self, t, crank):
        self.aborted = True
        self._retract_from = crank
        self._go("retract", t)

    def fallen(self, t: float, roll: float, roll_rate: float) -> bool:
        """True once the bike has lain past `fallen_deg`, nearly still, for
        `fallen_s` -- the auto-up trigger. Call every step while riding."""
        if abs(roll) > self.fallen_rad and abs(roll_rate) < self.still_rate:
            if self._down_since is None:
                self._down_since = t
            return t - self._down_since >= self.fallen_s
        self._down_since = None
        return False

    def step(self, t: float, roll: float, roll_rate: float, crank: float,
             speed: float = 0.0) -> Command:
        """One step. `roll` [rad], `roll_rate` [rad/s], `crank` [rad] and
        `speed` [m/s, ground speed] as the bike measures them."""
        dt = t - self._t0
        s = self.side
        if self.phase == "stop":
            # The caller has zeroed the command; the policy brings the bike
            # to rest before any blade moves. A lie-down started at speed
            # deploys into a bike that is still braking and leaning.
            if speed < self.still_speed or dt >= self.stop_max_s:
                self._go("deploy", t)
            return Command(crank)
        if self.phase == "deploy":
            if dt >= self.deploy_s:
                self._go("kick", t)
            return Command(s * self.lean)
        if self.phase in ("kick", "settle") and s * roll < -self.abort:
            # Going over the OTHER way, where no blade is down: give the
            # bike back to the policy while it can still catch it (the
            # recoverable set is ~12 deg) and fold the blade away.
            self._abort(t, crank)
            return Command(crank)
        if self.phase == "kick":
            # CLOSED LOOP: kick until the bike leans `lean_on_deg` onto the
            # blade, not for a fixed time. Measured on the detailed
            # drivetrain, a fixed 0.2 s pulse sometimes left the bike leaning
            # away (the policy had parked it there) and it fell the wrong way.
            if s * roll >= self.lean_on or dt >= self.kick_max_s:
                self._go("settle", t)
            # d > 0 moves the rear contact toward -y (balance.lat_gain); the
            # bike leans the other way, so -s leans it onto blade s.
            return Command(s * self.lean, policy=False, diff=-s * self.kick)
        if self.phase == "settle":
            if dt >= self.settle_s:
                self._go("lower", t)
                self._crank = s * self.lean
            return Command(s * self.lean, policy=False)
        if self.phase == "lower":
            # In the side's frame: from the blade's own side, through centre,
            # to `rest` beyond it.
            u = max(-self.rest, s * self._crank - self.lower * dt)
            if u == -self.rest:
                self._go("limp", t)
            return Command(s * u, policy=False)
        if self.phase == "limp":
            frac = max(0.0, 1.0 - dt / self.limp_s) if self.limp_s > 0 else 0.0
            if frac == 0.0:
                self._go("lying", t)
            return Command(-s * self.rest, current=frac, policy=False)
        if self.phase == "lying":
            if not (self._retry and dt >= self.retry_s):
                return Command(crank, current=0.0, policy=False)
            self.get_up(t, roll)    # the side it lies on NOW
            dt = 0.0
            s = self.side
        if self.phase == "lift":
            if self._lift_from is None:
                self._lift_from = crank
            target = s * (self.deploy if self.hand_off else self.lean)
            goal = target
            if self.lift_rate is not None:          # a ramped goal, not a step
                goal = self._lift_from + np.clip(goal - self._lift_from,
                                                 -self.lift_rate * dt, self.lift_rate * dt)
            if not self.hand_off:
                # A tolerance, not ==: the ramp's sum lands within rounding
                # of the target, and on one side never exactly on it.
                if abs(goal - target) < 1e-6 and ready_for_policy(
                        roll, roll_rate, self.recover_deg):
                    self._go("propped", t)
                    return Command(goal, policy=False, limp=True)
            elif ready_for_policy(roll, roll_rate, self.recover_deg):
                self._go("balance", t)
                self._hold = goal                    # the blade stays put
                return Command(goal)
            if dt > self.lift_timeout_s:
                # Did not come up: let go and lie there, and try again from
                # `lying` after `retry_s` (module docstring).
                self._go("limp", t)
                self._retry = self.retry_s is not None
            return Command(goal, policy=self.lift_policy)
        if self.phase == "propped":
            return Command(s * self.lean, policy=False, limp=True)
        if self.phase in ("balance", "retract") and abs(roll) > self.fallen_rad \
                and not self._fell:
            # The policy lost it after the hand-off and the bike is down again
            # (either side). Fold the blade and go back to riding, so the
            # auto-up's own detector tries again from the side it landed on.
            # Without this, balance held a lying bike for balance_max_s and
            # retract waited on a crank pinned under it (fixes_72M, base
            # drivetrain, 2026-10-16: down at 19.8 s, still lying at 24.5).
            self._fell = True
            self._retract_from = crank
            self._go("retract", t)
            return Command(crank)
        if self.phase == "balance":
            # Ease the blade off the floor before folding it. Right after the
            # hand-off the policy leans a few degrees ONTO the planted blade
            # (a kickstand); snatching it away at once dropped the bike onto
            # that side (detailed drivetrain, 2026-10-16). So: creep back to
            # `release_deg`, then retract only once the bike stands upright.
            mag = max(min(abs(self._hold), self.release),
                      abs(self._hold) - self.release_rate * dt)
            eased = mag <= self.release + 1e-9
            if dt >= self.balance_max_s or (
                    dt >= self.retract_after and eased and abs(roll) < self.upright):
                self._retract_from = s * mag
                self._go("retract", t)
            return Command(s * mag)
        if self.phase == "retract":
            goal = self._retract_from - np.sign(self._retract_from) * min(
                abs(self._retract_from), self.retract_rate * dt)
            if abs(goal) < 1e-9 and (abs(crank) < np.deg2rad(5.0) or self._fell):
                self._go("riding", t)
            return Command(goal)
        return Command(0.0)                       # riding: centre, full current
