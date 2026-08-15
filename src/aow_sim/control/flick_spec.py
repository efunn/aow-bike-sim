"""Shared observation/action spec for the RL flick — dependency-free (numpy).

This is the single definition of what the policy sees and does, imported by:
  - control/flick_env.py  (the gymnasium training env; needs gymnasium)
  - control/policy.py      (numpy replay; no torch/gymnasium)
  - control/drive.py       (replaying the policy in the controller)

Keeping it here guarantees training and replay agree exactly. Observation is
ground-truth state (matching the analytic controllers); yaw error AND steer are
carried as sin/cos so the policy tracks the 180 deg target and can sweep the
front wheel through any angle — multi-turn included — without wrap
discontinuities (a wrapped steer scalar sawtooths every time the wheel crosses
pi); the fore/aft position x is intentionally excluded (the flick may
translate in x).

Action is 3-dim in [-1, 1]: steer *rate*, hub speed, rear differential. Steer as
a rate (integrated to the position-servo target) lets the policy sweep the front
wheel continuously through and past 90/180 deg.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

OBS_DIM = 11
ACT_DIM = 3   # [steer_rate, hub, diff]; feedforward mode uses only [0:2]


@dataclass(frozen=True)
class ActionBounds:
    steer_rate_max: float   # rad/s
    hub_max: float          # m/s
    diff_max: float         # rad/s (rear differential)
    # OPTIONAL fourth channel, general policy only (see general_spec: the
    # wings are a general-policy experiment, not part of the move contract).
    # Defaulted, so every 3-argument construction in the moves is unchanged
    # and ACT_DIM stays 3.
    wing_rate_max: float = 0.0   # rad/s of wing joint rate command

    def to_list(self) -> list[float]:
        return [self.steer_rate_max, self.hub_max, self.diff_max,
                self.wing_rate_max]

    @classmethod
    def from_list(cls, v) -> "ActionBounds":
        # Length-tolerant: every npz written before the wing channel existed
        # carries three entries and must keep loading unchanged.
        return cls(float(v[0]), float(v[1]), float(v[2]),
                   float(v[3]) if len(v) > 3 else 0.0)


def build_obs(roll, roll_rate, yaw_err, yaw_rate, steer, v_lon, v_lat,
              e_lat, phase) -> np.ndarray:
    """Assemble the observation vector (length OBS_DIM).

    yaw_err  : target_yaw - psi (rad, unwrapped) -> encoded as sin/cos.
    steer    : front steer angle (rad, any winding) -> encoded as sin/cos, so
               the encoding is smooth through pi and multi-turn sweeps never
               see a wrap discontinuity.
    e_lat    : lateral offset from the start pose, bike frame [m].
    phase    : elapsed / max_episode in [0, 1].
    """
    return np.array([
        roll, roll_rate,
        np.sin(yaw_err), np.cos(yaw_err), yaw_rate,
        # np.sin(steer), np.cos(steer), v_lon, v_lat, e_lat, phase,
        np.sin(2*steer), np.cos(2*steer), v_lon, v_lat, e_lat, phase,
    ], dtype=np.float32)


def scale_action(a, bounds: ActionBounds):
    """Map a normalized action to (steer_rate, hub, diff[, wing_rate]).

    Length 3 = full (the policy also drives the differential), length 2 =
    feedforward (the differential comes from the crawl balance instead,
    returned here as 0).

    Length 4 additionally returns a wing RATE and is general-policy only. The
    arity of the RETURN follows the arity of the INPUT, so every move policy --
    which emits at most 3 -- still unpacks three values exactly as before.
    """
    a = np.clip(np.asarray(a, dtype=float), -1.0, 1.0)
    diff = float(a[2]) * bounds.diff_max if a.shape[0] >= 3 else 0.0
    out = (float(a[0]) * bounds.steer_rate_max,
           float(a[1]) * bounds.hub_max, diff)
    if a.shape[0] >= 4:
        out = out + (float(a[3]) * bounds.wing_rate_max,)
    return out
