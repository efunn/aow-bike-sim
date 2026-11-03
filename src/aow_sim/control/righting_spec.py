"""Shared observation/action spec for the RL get-up -- dependency-free (numpy).

The single definition of what the righting policy sees and does, imported by
control/righting_env.py (training) and by `RightingPolicy` here (replay, safe
on the Pi). The swing_sit sequence's scripted lift is the thing it would
replace: from lying on a blade to a prop inside the hand-off window, on the
crank alone (plus, optionally, the rear differential while the drives are
otherwise held). The general policy is off throughout, as in the sequence.

MIRRORED: everything is expressed on the side the bike is lying on, `side`
(+1 lying RIGHT, roll +ve; -1 LEFT), so one policy serves both. Crank
+deploy puts the right blade down (swing_sit), so lying right the lift drives
the crank +ve; in the side frame every lift is +ve and upright is roll 0 from
above. `side` is fixed at engage and never re-read: a bike that goes over
the top is then at negative side-frame roll, which is the failure.

Observation (OBS_DIM), side frame:
    roll, roll_rate            [rad, rad/s]  as the AHRS reports them
    crank, crank_rate          [rad, rad/s]  Present Position / Velocity
    lead                       [rad]         goal - crank
    current                    [A]           Present Current
    goal_current               [0..1]        of `current_max`
    heat                       [0..1]        the overload proxy, of its trip
    phase                      [0..1]        elapsed / max_episode

Action (ACT_DIM) in [-1, 1]:
    crank_goal                 goal_mid + a x goal_span [rad], side frame,
                               ABSOLUTE (the bike writes Goal Position)
    goal_current               current_max x min(1, a + 1): a >= 0 is full
    diff                       x diff_max [rad/s] rear differential (0: off)

CENTRED ON A GET-UP, so an untrained policy explores near one. Measured in
the env (2026-11-03, 24 randomized episodes each): a constant goal of 110
deg at full current props 16 (over 7, overload 1); 100 deg props 2; 90
none. The first version slewed the goal at a RATE and the second spanned
lying-to-stroke with 0 in the middle (23 deg): a zero-mean policy then
went nowhere, 0 of 80 random episodes propped, and the first run (3M
steps) settled on lying still.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

OBS_DIM = 9
ACT_DIM = 3


@dataclass(frozen=True)
class RightingBounds:
    goal_mid: float            # rad, side frame, at a = 0
    goal_span: float           # rad per unit a
    current_max: float         # Goal Current counts at a = +1
    diff_max: float = 0.0      # rad/s; 0 disables the channel

    def to_list(self) -> list[float]:
        return [self.goal_mid, self.goal_span, self.current_max, self.diff_max]

    @classmethod
    def from_list(cls, v) -> "RightingBounds":
        return cls(*(float(x) for x in v[:4]))


def build_obs(side, roll, roll_rate, crank, crank_rate, goal, current,
              goal_current_frac, heat_frac, phase) -> np.ndarray:
    """The observation, mirrored onto the lying side (`side` +-1)."""
    s = float(side)
    return np.array([s * roll, s * roll_rate, s * crank, s * crank_rate,
                     s * (goal - crank), s * current, goal_current_frac,
                     heat_frac, phase], dtype=np.float32)


def scale_action(a, bounds: RightingBounds, side) -> tuple[float, int, float]:
    """Normalized action -> (crank goal [rad], Goal Current [counts], rear
    differential [rad/s]), back in the world frame."""
    a = np.clip(np.asarray(a, dtype=float), -1.0, 1.0)
    s = float(side)
    goal = bounds.goal_mid + float(a[0]) * bounds.goal_span
    return (s * goal,
            int(round(min(1.0, float(a[1]) + 1.0) * bounds.current_max)),
            s * float(a[2]) * bounds.diff_max)


class RightingPolicy:
    """An exported righting policy, replayed with numpy alone.

    The npz is control/policy.py's format (save_policy_npz); its `bounds`
    field holds `RightingBounds.to_list()`, so only the network is taken from
    `load_policy_npz` and the scaling is this module's."""

    def __init__(self, path):
        from .policy import load_policy_npz
        self.net = load_policy_npz(path)
        self.bounds = RightingBounds.from_list(np.load(path)["bounds"])

    def action(self, obs, side):
        return scale_action(np.clip(self.net.mean(obs), -1.0, 1.0),
                            self.bounds, side)
