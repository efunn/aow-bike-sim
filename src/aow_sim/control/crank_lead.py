"""The manual righting crank's two limits, shared by the bike, the station
and teleop so the sim flies the code the bike runs.

  * `lead_goal`  the goal held within `lead` of the crank's measured angle.
    Without it the triggers wound the goal on to the stroke end while the
    crank stalled 40-60 deg short (runs 261102-204438, -231204), and taking
    it back down first had to unwind all of that. The station applies it
    (hw/ground.OperatorState), and teleop.
  * `StallHold`  a crank that stops making progress toward its goal drops to
    a holding Goal Current until the goal moves or the crank moves on. The
    bike applies it (hw/run_bike), and teleop. A CURRENT, not the goal: a
    goal held still still pulls whatever its error asks for (user).

The sequence (control/swing_sit.py) has its own `blocked` rule and owns the
crank while it runs; neither of these applies then.

Pure: no MuJoCo, no I/O -- this runs on the Pi.
"""
from __future__ import annotations

import math

# Measured 2026-11-02 (runs 261102-204438, -231204): manual get-ups stalled
# at roll 62-64 deg with PWM pinned at 885 and 670-750 mA, under the 785
# cap, and tripped OVERLOAD in 1.7 and 2.3 s; a prop held 10 deg short of the
# stroke end drew 0.31-0.34 A for 9-14 s. The PWM saturates at ~16 deg of
# error (32 mA, 56 PWM per deg, the held props), so any lead past that still
# pushes flat out. STALL_S is 0.75 because the same runs had a lift sit 0.5 s
# loading at its start and then get up (-204438 at 24.3 s); replayed with a
# 40 deg lead it holds 0.8 and 1.8 s before the two overloads, and on no
# lift that got up. HOLD_CURRENT is a first value -- the linkage holds a
# prop unpowered (run_bike PROP_CURRENT).
STALL_S = 0.75
STALL_ERR_DEG = 5.0
STALL_MOVE_DEG = 3.0
HOLD_CURRENT = 200          # raw Goal Current counts


def lead_goal(goal: float, crank: float | None, lead: float | None) -> float:
    """`goal` [rad] clamped to within `lead` [rad] of `crank`; unchanged
    with no limit (None) or no reading."""
    if lead is None or crank is None:
        return goal
    return min(crank + lead, max(crank - lead, goal))


class StallHold:
    """Whether the crank has stalled short of its goal, step by step.

    Stalled: the goal STALL_ERR_DEG or more from the crank, and the crank not
    STALL_MOVE_DEG nearer it for STALL_S. Then `holding` until the goal moves
    more than STALL_MOVE_DEG or the crank gets that far on toward it. Being
    back-driven AWAY from the goal is not progress, or a stalled lift would
    sag, re-arm and pump -- which tripped the overload on 10-26 (run
    261026-195319, 5.8 s above 400 mA).
    """

    def __init__(self, stall_s: float = STALL_S,
                 err_deg: float = STALL_ERR_DEG,
                 move_deg: float = STALL_MOVE_DEG):
        self.stall_s = float(stall_s)
        self.err = math.radians(err_deg)
        self.move = math.radians(move_deg)
        self.at = None              # (goal, crank) while holding
        self._t0, self._ref = 0.0, None

    @property
    def holding(self) -> bool:
        return self.at is not None

    def reset(self, t: float, crank: float | None) -> None:
        """Not holding, the progress timer restarted from `crank`."""
        self.at = None
        self._t0, self._ref = t, crank

    def step(self, t: float, goal: float, crank: float) -> str | None:
        """-> "hold" when it starts holding, "goal moved" / "crank moving"
        when it stops, else None."""
        if self.at is not None:
            g0, c0 = self.at
            if abs(goal - g0) > self.move:
                why = "goal moved"
            elif (crank - c0) * math.copysign(1.0, g0 - c0) > self.move:
                why = "crank moving"
            else:
                return None
            self.reset(t, crank)
            return why
        err = goal - crank
        if abs(err) < self.err or self._ref is None \
                or (crank - self._ref) * math.copysign(1.0, err) > self.move:
            self._t0, self._ref = t, crank
            return None
        if t - self._t0 < self.stall_s:
            return None
        self.at = (goal, crank)
        return "hold"
