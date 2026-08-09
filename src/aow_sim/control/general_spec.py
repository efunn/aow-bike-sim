"""Shared observation/action spec for the GENERAL command-conditioned policy
— dependency-free (numpy).

The single definition of what the general policy sees and does, imported by:
  - control/general_env.py  (the gymnasium training env; needs gymnasium)
  - control/drive.py        (replaying the policy in the controller)

Unlike the move specs (flick/ball/pivot), this one describes an **always-on**
controller, and that forces three differences:

  1. NO `phase`. Every move spec carries `phase = step/max_episode` because
     the move is finite-horizon. A policy that knows "time until my horizon
     ends" cannot run indefinitely, so the general obs is *stationary*:
     identical physical state + identical command -> identical obs, whenever
     it occurs.
  2. NO episode-start frame. The moves measure e_lat/e_line/hold against a
     snapshot taken when the move was commanded (_p0, _yaw0, theta0, _pf0).
     Here every error is measured against the LIVE command, so there is no
     drifting reference and no maneuver boundary.
  3. The command is part of the observation and changes mid-episode.

Command parameterization — a velocity VECTOR, not (course, speed) polar.
Course is undefined at zero speed and flips discontinuously on reverse, so a
polar command would be singular at exactly the two most common operator
inputs ("stop", "back up"). Commanding a velocity vector rotated into the
current body frame makes stop = (0, 0), reverse = (-v, 0), and a hard course
reversal a continuous path through the origin of command space. Because the
rear omni wheel decouples travel direction from heading, the lateral
component is physically meaningful: (v_cmd_lat != 0 with psi_err held) is a
crab, and (v_cmd held while psi_cmd sweeps) is exactly the pivot move.

Heading wraps mod 2*pi (the chassis has a front). Steer wraps mod pi and is
carried as sin/cos(2*delta): the wheel is front-back symmetric, so this is
both winding-invariant (any multi-turn angle encodes identically to its
wrapped equivalent) and smooth through pi. That means the general policy can
consume raw multi-turn qpos with no pi-park rebasing — see control/steer.py.

`prev_action` is fed back deliberately: the reward penalizes
w_smooth*|a - a_prev|^2, and in the move specs a_prev is NOT observable, so
the policy is asked to minimize something it cannot see. Feeding it back
makes that objective well-posed. w_smooth may be one weight or one per action
channel (general_env._per_channel): the three channels are not
interchangeable, since `diff` is the one that catches roll and it declines to
smooth at a price the other two accept.

Action is identical to every other move (steer rate, hub speed, rear
differential), so ActionBounds/scale_action are reused from flick_spec
rather than duplicated.
"""

from __future__ import annotations

import numpy as np

# Re-export so env + replay import the action contract from one place.
from .flick_spec import ActionBounds, scale_action  # noqa: F401
from .steer import wrap_pi  # noqa: F401

OBS_DIM = 15
ACT_DIM = 3   # [steer_rate, hub, diff]; feedforward mode uses only [0:2]


def build_obs(roll, roll_rate, yaw_rate, steer, steer_rate, v_lon, v_lat,
              v_cmd_lon, v_cmd_lat, psi_err, prev_action) -> np.ndarray:
    """Assemble the observation vector (length OBS_DIM).

    All quantities are instantaneous state or live command — nothing here
    depends on elapsed time or on when the policy was engaged.

    roll/roll_rate : chassis lean [rad, rad/s].
    yaw_rate       : chassis yaw rate [rad/s] (qvel[5]).
    steer          : front steer joint angle [rad, any winding] -> sin/cos(2x).
    steer_rate     : steer joint velocity [rad/s]; the action integrates a
                     steer RATE, so its derivative belongs in the state (the
                     LQR has always used it; no move spec did).
    v_lon/v_lat    : measured body-frame velocity [m/s].
    v_cmd_lon/lat  : COMMANDED velocity, rotated into the current body frame
                     [m/s]. Body-frame so the obs is yaw-equivariant.
    psi_err        : wrap_pi(psi_cmd - psi) [rad] -> sin/cos, mod 2*pi.
    prev_action    : the previous normalized action (length ACT_DIM).
    """
    pa = np.asarray(prev_action, dtype=float).reshape(-1)
    return np.array([
        roll, roll_rate, yaw_rate,
        np.sin(2 * steer), np.cos(2 * steer), steer_rate,
        v_lon, v_lat,
        v_cmd_lon, v_cmd_lat,
        np.sin(psi_err), np.cos(psi_err),
        pa[0], pa[1], pa[2] if pa.shape[0] >= 3 else 0.0,
    ], dtype=np.float32)


def command_to_body(v_cmd_world, psi_cmd, psi) -> tuple[float, float, float]:
    """(world velocity command, heading command, current yaw) -> the three
    command numbers the observation wants: (v_cmd_lon, v_cmd_lat, psi_err).

    Shared by the training env and the controller replay so the two cannot
    disagree about the frame convention."""
    c, s = np.cos(psi), np.sin(psi)
    vx, vy = float(v_cmd_world[0]), float(v_cmd_world[1])
    return (c * vx + s * vy,          # body longitudinal
            -s * vx + c * vy,         # body lateral (+Y = left)
            wrap_pi(psi_cmd - psi))
