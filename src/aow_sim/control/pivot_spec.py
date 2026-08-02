"""Shared observation/action spec for the RL pivot — dependency-free (numpy).

The single definition of what the pivot policy sees and does, imported by:
  - control/pivot_env.py  (the gymnasium training env; needs gymnasium)
  - control/drive.py      (replaying the policy in the controller)

The pivot: the chassis yaws to a target (180 deg for now) while the FRONT
WHEEL HOLDS ITS GLOBAL GROUND HEADING (mod pi — the wheel is front-back
symmetric). The front is a regular wheel, so the hold is what lets it keep
rolling along the original travel line while the chassis spins around it;
the move works from standstill or a glide (per-episode start/end speed
targets along the original line).

The action is identical to the flick's, so the bounds/scaling machinery is
reused from `flick_spec` (`ActionBounds`, `scale_action`):

  action (3-dim, [-1, 1]): [steer_rate, hub, diff]  (full mode).

Frames (captured at move start, mirrored exactly by replay):
  theta0 = psi0 + wheel_heading(steer0)   — the held global wheel heading;
  u0/n0 along/normal to it; the line anchors at the FRONT CONTACT
  p_f = qpos[:2] + L*[cos psi, sin psi] (not the chassis: the rear must
  swing off the line by construction). v_ref(phase) ramps v_start -> v_end;
  after the turn the bike glides BACKWARD along its new heading.
"""

from __future__ import annotations

import numpy as np

# Re-export so env + replay import the action contract from one place.
from .flick_spec import ActionBounds, scale_action  # noqa: F401
from .steer import steer_for_heading, wheel_heading, wrap_pi  # noqa: F401

OBS_DIM = 15
ACT_DIM = 3   # [steer_rate, hub, diff]


def build_obs(roll, roll_rate, yaw_err, yaw_rate, steer, hold_raw,
              v_lon, v_lat, v_err, v_end, e_line, phase) -> np.ndarray:
    """Assemble the observation vector (length OBS_DIM).

    yaw_err  : target_yaw - (psi - psi0) (rad, unwrapped) -> sin/cos. A
               future sampled yaw target changes only this value, not the
               obs shape.
    steer    : front steer angle (rad, any winding) -> sin/cos of 2*steer,
               matching the flick/ball encoding (pi-symmetric wheel).
    hold_raw : psi + wheel_heading(steer) - theta0 (rad, any winding) ->
               sin/cos of 2*hold_raw: the mod-pi global wheel-heading hold
               error, the move's defining quantity.
    v_lon/v_lat : body-frame speeds (balance).
    v_err    : v_along - v_ref(phase), line frame (v_along = front-contact
               speed along the original global heading).
    v_end    : target end speed along the line (endpoint behavior differs
               qualitatively: settle-to-stop vs keep gliding). v_start is
               NOT observed — it is absorbed into the v_ref ramp.
    e_line   : front-contact lateral offset from the original line [m].
    phase    : elapsed / max_episode in [0, 1].
    """
    return np.array([
        roll, roll_rate,
        np.sin(yaw_err), np.cos(yaw_err), yaw_rate,
        np.sin(2 * steer), np.cos(2 * steer),
        np.sin(2 * hold_raw), np.cos(2 * hold_raw),
        v_lon, v_lat,
        v_err, v_end,
        e_line, phase,
    ], dtype=np.float32)
