"""Multi-turn steering-angle bookkeeping.

The steer joint is an unlimited hinge and the servo (XC330-T181 in extended-
position mode) reports raw multi-turn position: 4096 counts/rev, +-256 turns.
MuJoCo's qpos for an unlimited hinge is the same signal in radians, so a
future hardware driver is a pure unit conversion:

    counts = round(rad * XC330_COUNTS_PER_RAD)

with the +-256-turn extended-position range mapping to +-1,048,576 counts
(enforced here by clamp_extended, the one place that knows the servo limit).

Controllers never see the raw multi-turn angle. SteerFrame maps it to a
small-signal steer angle about a persistent origin that is always an integer
multiple of pi: the front wheel is front-back symmetric, so every pi multiple
is longitudinally straight, and adopting the *nearest* one bounds the
small-signal angle to [-pi/2, pi/2]. This is what lets a post-flick park at
~180 deg read as "straight" instead of a half-turn error the LQR would
unwind (dragging the bike in yaw at standstill — see
docs/plans/mujoco-modeling-decisions.md, "Steer origin after the flick").
With the current geometry (fork_offset = 0 in bike_params.yaml) the front
axle lies on the raked steer axis, so a pi rotation maps the wheel onto the
identical physical configuration — the pi origin is exact, trail included.
If a nonzero fork_offset is ever introduced, odd pi multiples park the axle
on the far side of the axis (reversed effective trail) and this choice
should be revisited.

Same-direction flicks wind the joint pi per flick by design (never unwind at
standstill); the winding is bounded by the extended-position clamp (~512
flicks) and real hardware re-homes at power-up.

Invariant: the origin changes only at discrete events (controller reset,
command_*, maneuver handoff) and is always computed from the *measured*
multi-turn angle, so re-adoption never commands motion. Never re-adopt
inside a control-law _compute.
"""

from __future__ import annotations

import numpy as np

TWO_PI = 2.0 * np.pi
XC330_COUNTS_PER_REV = 4096
XC330_COUNTS_PER_RAD = XC330_COUNTS_PER_REV / TWO_PI     # ~651.9 counts/rad
XC330_MAX_TURNS = 256
STEER_CMD_LIMIT = XC330_MAX_TURNS * TWO_PI               # ~1608.5 rad


def wrap_pi(x: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return float(np.arctan2(np.sin(x), np.cos(x)))


def nearest_multiple(x: float, period: float = np.pi) -> float:
    """The integer multiple of `period` nearest to x."""
    return float(period * np.round(x / period))


def clamp_extended(cmd: float) -> float:
    """Clamp an absolute multi-turn setpoint to the XC330 extended range."""
    return float(np.clip(cmd, -STEER_CMD_LIMIT, STEER_CMD_LIMIT))


def wheel_heading(delta: float, rake: float) -> float:
    """Ground-trace heading of the front wheel plane for steer joint angle
    `delta` about the axis raked back by `rake` [rad], chassis frame.

    The raked axis makes the ground heading differ from the joint angle
    (identical at rake 0; ~1 deg apart at rake 15 deg). Wraps like atan2 —
    continuous within a turn, physically meaningful mod pi (symmetric
    wheel)."""
    return float(np.arctan2(np.cos(rake) * np.sin(delta), np.cos(delta)))


def steer_for_heading(heading: float, rake: float) -> float:
    """Principal-branch inverse of wheel_heading (feedforward/analysis)."""
    return float(np.arctan2(np.sin(heading), np.cos(rake) * np.cos(heading)))


class SteerFrame:
    """Maps between the multi-turn servo angle and the small-signal steer
    angle the linear controllers were designed around (see module docstring
    for the origin convention and its invariant)."""

    def __init__(self) -> None:
        self.origin = 0.0

    def sync(self, measured: float) -> None:
        """Adopt the pi-multiple origin nearest the measured multi-turn
        angle. Continuous by construction: never commands the wheel to move,
        and the resulting small-signal angle lands in [-pi/2, pi/2]."""
        self.origin = nearest_multiple(measured)

    def measured(self, qpos_steer: float) -> float:
        """Small-signal steer = multi-turn measured - origin."""
        return float(qpos_steer) - self.origin

    def command(self, steer_small: float) -> float:
        """Absolute multi-turn setpoint for a small-signal command, clamped
        to +-256 turns (XC330 extended-position range)."""
        return clamp_extended(steer_small + self.origin)
