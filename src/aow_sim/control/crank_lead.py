"""The manual righting crank's two limits, shared by the bike, the station
and teleop so the sim flies the code the bike runs.

  * `lead_goal`  the goal held within `lead` of the crank's measured angle.
    Without it the triggers wound the goal on to the stroke end while the
    crank stalled 40-60 deg short (runs 261102-204438, -231204), and taking
    it back down first had to unwind all of that. The station applies it
    (hw/ground.OperatorState), and teleop.
  * `StallHold`  a crank that stops making progress toward its goal drops to
    a holding Goal Current until the goal moves or the crank moves on --
    ramped on to 0, so a failed lift lowers and a prop sits unpowered. The
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
# lift that got up. HOLD_CURRENT is only the first step of the let-go ramp
# below.
STALL_S = 0.75
STALL_ERR_DEG = 5.0
STALL_MOVE_DEG = 3.0
HOLD_CURRENT = 200          # raw Goal Current counts
# THEN LET GO. Flown 11-03 (run 261103-114128, 24 holds, 201 mA, no
# overload), the hold kept a stalled lift part-way up: 10 holds at roll
# 35-51 deg, all on the left, one for 27.8 s. So a hold ramps on from
# HOLD_CURRENT to 0 over LET_GO_S, in LET_GO_STEPS writes: a failed lift
# lowers onto its blade, and a prop stays up unpowered, as the sequence's
# do (PROP_CURRENT 0; 1-22 s at roll 7-16, runs 261027-143131 to -155148)
# and the user's manual props at 0 do. LET_GO_ROLL_DEG: only holds that
# start at least that far over let go, the rest stay at HOLD_CURRENT (0:
# all of them; None: none).
LET_GO_ROLL_DEG = 0.0
LET_GO_S = 1.0
LET_GO_STEPS = 10
SETTLE_S = 0.3              # a let-go crank still this long has landed


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
    STALL_MOVE_DEG further into the lift, or the gap between goal and crank
    closes that much from its widest (backing off, or the crank getting
    there). Being
    back-driven AWAY from the goal is not progress, or a stalled lift would
    sag, re-arm and pump -- which tripped the overload on 10-26 (run
    261026-195319, 5.8 s above 400 mA).

    `current` is the Goal Current to hold at: a ramp from HOLD_CURRENT to
    0, or HOLD_CURRENT for a hold that started under `let_go_roll_deg`.
    """

    def __init__(self, stall_s: float = STALL_S,
                 err_deg: float = STALL_ERR_DEG,
                 move_deg: float = STALL_MOVE_DEG,
                 let_go_roll_deg: float | None = LET_GO_ROLL_DEG,
                 let_go_s: float = LET_GO_S):
        self.stall_s = float(stall_s)
        self.err = math.radians(err_deg)
        self.move = math.radians(move_deg)
        self.let_go = (None if let_go_roll_deg is None
                       else math.radians(let_go_roll_deg))
        self.let_go_s = float(let_go_s)
        self.at = None              # (goal, crank) while holding
        self.letting_go = False     # this hold ramps to 0
        self.gc = None              # the Goal Current last returned by `current`
        self._held_t = 0.0
        self._t0, self._ref = 0.0, None

    @property
    def holding(self) -> bool:
        return self.at is not None

    def reset(self, t: float, crank: float | None) -> None:
        """Not holding, the progress timer restarted from `crank`."""
        self.at = None
        self.letting_go, self.gc = False, None
        self._t0, self._ref = t, crank

    def current(self, t: float, base: int) -> int | None:
        """While holding, the Goal Current to write now -- min(HOLD_CURRENT,
        `base`, the operator's), ramped to 0 if letting go -- or None when it
        is unchanged since the last call (or not holding)."""
        if self.at is None:
            return None
        gc = min(HOLD_CURRENT, int(base))
        if self.letting_go:
            frac = max(0.0, 1.0 - (t - self._held_t) / self.let_go_s)
            gc = int(round(gc * math.ceil(frac * LET_GO_STEPS - 1e-9)
                           / LET_GO_STEPS))
        if gc == self.gc:
            return None
        self.gc = gc
        return gc

    def step(self, t: float, goal: float, crank: float,
             roll: float | None = None) -> str | None:
        """-> "hold" when it starts holding, "goal moved" / "crank moving"
        when it stops, else None. `roll` [rad], read at the hold's start,
        decides whether it lets go."""
        if self.at is not None:
            g0, c0 = self.at
            side = math.copysign(1.0, g0 - c0)
            gap = (goal - crank) * side
            if self.letting_go:
                # Unpowered, the crank falls with the bike and the station's
                # lead drags the goal after it, in jumps a link's lag behind:
                # run 261103-120130 at 45.7 s, the gap overshot to 55 deg
                # and came back to 40, which read as the operator backing
                # off and put 785 back mid-fall -- the lift pumped. So
                # nothing counts until the crank has been still SETTLE_S,
                # and measured from there.
                if abs(crank - self._still_at) > self.move:
                    self._still_at, self._still_t = crank, t
                if t - self._still_t < SETTLE_S:
                    self._g_ref, self._gap = goal, gap
                    return None
            self._gap = max(self._gap, gap)
            if (goal - self._g_ref) * side > self.move:
                why = "goal moved"              # on, further into the lift
            elif gap < self._gap - self.move:
                # the gap closed from its widest: the operator backing off,
                # or the crank getting there -- except an unpowered one
                # swinging back, which is a bounce, not progress.
                if abs(goal - self._g_ref) > self.move:
                    why = "goal moved"
                elif not self.letting_go:
                    why = "crank moving"
                else:
                    return None
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
        self._g_ref = goal                  # the goal moves are measured from
        self._gap = abs(goal - crank)       # the widest gap since
        self._still_at, self._still_t = crank, t - SETTLE_S
        self._held_t, self.gc = t, None
        self.letting_go = (self.let_go is not None
                           and (roll is None or abs(roll) >= self.let_go))
        return "hold"
