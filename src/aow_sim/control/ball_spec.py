"""Shared observation/action spec for the RL ball-shot — dependency-free (numpy).

The single definition of what the ball-shot policy sees and does, imported by:
  - control/ball_env.py  (the gymnasium training env; needs gymnasium)
  - control/drive.py     (replaying the policy in the controller)

Same role as `flick_spec` for the 180 flick, so training and replay agree exactly.
The action is identical to the flick's, so the bounds/scaling machinery is reused
from `flick_spec` (`ActionBounds`, `scale_action`) rather than duplicated:

  action (3-dim, [-1, 1]): [steer_rate, hub, diff]  (full mode: policy balances).

The observation is ground-truth bike state plus the ball's pose/velocity expressed
in the bike frame, and a `ball_present` flag so the *same* policy handles the
no-ball ("catch") trials (see docs/plans/ball-shot-move.md). Heading is carried as
the error against the **world-frame** launch target, sin/cos encoded to avoid wrap
discontinuities; fore/aft x of the bike itself is excluded (the bike translates
freely toward the ball).

Heading was previously measured against the bike's own start pose. That made the
launch target unobservable: the ball and the target both rotated with the initial
yaw, so the whole scene was rigidly attached to the start heading and the init_yaw
randomization cancelled out instead of being a disturbance to correct for.
"""

from __future__ import annotations

import numpy as np

# Re-export so env + replay import the action contract from one place.
from .flick_spec import ActionBounds, scale_action  # noqa: F401

OBS_DIM = 15
ACT_DIM = 3   # [steer_rate, hub, diff]; feedforward mode uses only [0:2]


def build_obs(roll, roll_rate, heading, yaw_rate, steer, v_lon, v_lat,
              ball_dx, ball_dy, ball_vx, ball_vy, ball_present, phase) -> np.ndarray:
    """Assemble the observation vector (length OBS_DIM).

    heading      : psi - target_yaw (rad), bike heading vs the WORLD launch
                   target direction -> sin/cos. Carries absolute orientation,
                   which the policy needs to aim at a world-fixed target.
    steer        : front steer angle (rad, any winding) -> encoded as sin/cos,
                   smooth through pi (see flick_spec.build_obs).
    ball_dx/dy   : ball position relative to the bike, bike frame [m].
    ball_vx/vy   : ball velocity in the bike frame [m/s] (~0 until struck).
    ball_present : 1.0 if a ball is in play, else 0.0 (catch/no-ball trials).
    phase        : elapsed / max_episode in [0, 1].
    """
    return np.array([
        roll, roll_rate,
        np.sin(heading), np.cos(heading), yaw_rate,
        # np.sin(steer), np.cos(steer), v_lon, v_lat,
        np.sin(2*steer), np.cos(2*steer), v_lon, v_lat,
        ball_dx, ball_dy, ball_vx, ball_vy,
        float(ball_present), phase,
    ], dtype=np.float32)
