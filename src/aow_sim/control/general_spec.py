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
     drifting reference and no maneuver boundary. This survives `v_bar`
     below: nothing integrates the COMMAND into a setpoint. What is filtered
     is the bike's own measured velocity.
  3. The command is part of the observation and changes mid-episode.

OPTIONAL WINDOWED VELOCITY (`v_bar`, obs entries 15-16). The differential is
the only actuator for both roll balance and lateral crawl, and under a pure
hold command it is already p95-saturated by balance alone -- so a sustained
pure crab is not physically available at standstill. The only manoeuvre that
produces net lateral motion is an oscillatory wriggle (fore/aft oscillation
against a steering pattern), and instantaneous velocity tracking scores every
instant of that oscillation as error: measured, a perfect wriggle above
0.21 m/s of fore/aft amplitude scores WORSE than standing still. The reward
therefore forbids the one manoeuvre that can satisfy a lateral command.

`v_bar` is a first-order low-pass of the bike's world-frame ground velocity,
rotated into the body frame, which makes the gait's TIME AVERAGE the tracked
quantity. It is in the OBSERVATION and not merely the reward because a reward
may not depend on state the policy cannot see -- the same argument that put
`prev_action` here.

The heading term is deliberately NOT filtered. Heading is position-like, and
an averaged heading error would license the bike to spin as long as it
averaged out.

WIDTH IS A PROPERTY OF THE POLICY, not of this module. `vel_window_s` rides
in the move yaml beside `v_lat_frac` (control/flick.py), so a policy trained
without it still loads and replays unchanged:
  window <= 0  -> OBS_DIM (15), and the reward is the instantaneous one
                  bit for bit -- alpha is exactly 1.0, so v_bar IS v.
  window >  0  -> OBS_DIM_WINDOWED (17), v_bar appended.
The 15-entry layout is a strict PREFIX of the 17-entry one, so every
positional index into an observation stays valid across both.

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

OBS_DIM = 15            # no optional blocks (the original spec)
OBS_DIM_WINDOWED = 17   # ...plus v_bar_lon, v_bar_lat
ACT_DIM = 3   # [steer_rate, hub, diff]; feedforward mode uses only [0:2]

# Optional observation blocks, in APPEND ORDER. Each is a (flag, names) pair;
# the base 15 stay a prefix of every layout, so positional indices into the
# base never move.
_OPTIONAL_BLOCKS = (
    ("vel_window", ("v_bar_lon", "v_bar_lat")),
    ("obs_pitch", ("pitch", "pitch_rate")),
)


# One home for the layout, so nothing has to re-derive it from index
# literals. The first OBS_DIM entries are the un-windowed observation.
OBS_NAMES_BASE = (
    "roll", "roll_rate", "yaw_rate",
    "sin2steer", "cos2steer", "steer_rate",
    "v_lon", "v_lat",
    "v_cmd_lon", "v_cmd_lat",
    "sin_psi_err", "cos_psi_err",
    "prev_steer_rate", "prev_hub", "prev_diff",
)
OBS_NAMES = OBS_NAMES_BASE + ("v_bar_lon", "v_bar_lat", "pitch", "pitch_rate")
# Sign under the sagittal mirror. A filter is linear and time-invariant, so a
# filtered quantity mirrors exactly like its source: v_bar_lon follows v_lon
# (+1), v_bar_lat follows v_lat (-1). Getting these wrong does not raise --
# it silently corrupts every handedness number in
# analysis/mirror_equivariance.py, which is why they live here and not there.
OBS_MIRROR_PARITY = np.array([
    -1, -1, -1,        # roll, roll_rate, yaw_rate
    -1, +1, -1,        # sin2steer, cos2steer, steer_rate
    +1, -1,            # v_lon, v_lat
    +1, -1,            # v_cmd_lon, v_cmd_lat
    -1, +1,            # sin psi_err, cos psi_err
    -1, +1, -1,        # prev_action [steer_rate, hub, diff]
    +1, -1,            # v_bar_lon, v_bar_lat
    +1, +1,            # pitch, pitch_rate -- SAGITTAL, so the mirror leaves
                       #   them alone, unlike roll and yaw which flip
], dtype=float)
assert len(OBS_NAMES) == len(OBS_MIRROR_PARITY)
assert len(OBS_NAMES_BASE) == OBS_DIM


def obs_layout(vel_window_s: float = 0.0, obs_pitch: bool = False) -> tuple:
    """The exact entry names this feature combination produces.

    WIDTH ALONE IS AMBIGUOUS and must not be used as the contract. With two
    optional 2-entry blocks, a velocity-windowed policy and a pitch-observing
    policy are BOTH 17 wide with completely different layouts, and a width
    check would load either as the other and feed the net nonsense without
    raising. So the layout is recorded in the move yaml and compared
    element-wise -- see control/drive.py engage_general.
    """
    names = list(OBS_NAMES_BASE)
    for flag, block in _OPTIONAL_BLOCKS:
        on = (float(vel_window_s) > 0.0 if flag == "vel_window"
              else bool(obs_pitch))
        if on:
            names.extend(block)
    return tuple(names)


def obs_dim_for(vel_window_s: float = 0.0, obs_pitch: bool = False) -> int:
    """Observation width implied by a policy's feature flags.

    Prefer `obs_layout()` for the CONTRACT -- width collides (see its
    docstring). This stays for sizing an array.
    """
    return len(obs_layout(vel_window_s, obs_pitch))


def vel_filter_alpha(dt: float, window_s: float) -> float:
    """Per-tick blend factor of the first-order low-pass on measured velocity.

    `1 - exp(-dt/tau)` rather than `dt/tau` so the CONTINUOUS time constant is
    identical whether the filter is stepped at the env's 50 Hz or the
    controller's rate -- the 200 Hz version is then a finer sampling of the
    same filter, not a different one.

    Returns exactly 1.0 for a non-positive window, which is what makes the
    un-windowed path bit-for-bit identical: v_bar <- v_bar + 1.0*(v - v_bar)
    is v, with no epsilon anywhere.
    """
    w = float(window_s)
    if w <= 0.0:
        return 1.0
    return float(1.0 - np.exp(-float(dt) / w))


def vel_filter_step(v_bar_w, v_w, alpha: float) -> np.ndarray:
    """Advance the low-pass one tick. Both arguments are WORLD frame.

    Filtering in world and rotating at read time is deliberate: an EMA of the
    body-frame components would let a gait that yaws +-15 deg corrupt its own
    average, since the frame the average is taken in would itself be swinging.
    """
    v_bar_w = np.asarray(v_bar_w, dtype=float)
    return v_bar_w + alpha * (np.asarray(v_w, dtype=float)[:2] - v_bar_w)


def build_obs(roll, roll_rate, yaw_rate, steer, steer_rate, v_lon, v_lat,
              v_cmd_lon, v_cmd_lat, psi_err, prev_action,
              v_bar=None, pitch=None) -> np.ndarray:
    """Assemble the observation vector (length OBS_DIM, or OBS_DIM_WINDOWED
    when `v_bar` is given).

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
    v_bar          : (v_bar_lon, v_bar_lat) [m/s], the low-passed measured
                     velocity ALREADY rotated into the body frame, or None for
                     a policy trained without a velocity window. Appended, so
                     the un-windowed layout stays a prefix of this one.
    pitch          : (pitch, pitch_rate) [rad, rad/s], +ve = NOSE UP, or None
                     for a policy that does not observe pitch. Present so the
                     w_pitch penalty is not charged against a state the policy
                     cannot see -- the same argument that put prev_action in.
                     Both come off the AHRS on hardware.
    """
    pa = np.asarray(prev_action, dtype=float).reshape(-1)
    obs = [
        roll, roll_rate, yaw_rate,
        np.sin(2 * steer), np.cos(2 * steer), steer_rate,
        v_lon, v_lat,
        v_cmd_lon, v_cmd_lat,
        np.sin(psi_err), np.cos(psi_err),
        pa[0], pa[1], pa[2] if pa.shape[0] >= 3 else 0.0,
    ]
    # Append order must match _OPTIONAL_BLOCKS.
    if v_bar is not None:
        obs.extend((float(v_bar[0]), float(v_bar[1])))
    if pitch is not None:
        obs.extend((float(pitch[0]), float(pitch[1])))
    return np.array(obs, dtype=np.float32)


def rotate_to_body(vx, vy, psi) -> tuple[float, float]:
    """World-frame planar vector -> body frame (+X forward, +Y left)."""
    c, s = np.cos(psi), np.sin(psi)
    vx, vy = float(vx), float(vy)
    return c * vx + s * vy, -s * vx + c * vy


def command_to_body(v_cmd_world, psi_cmd, psi) -> tuple[float, float, float]:
    """(world velocity command, heading command, current yaw) -> the three
    command numbers the observation wants: (v_cmd_lon, v_cmd_lat, psi_err).

    Shared by the training env and the controller replay so the two cannot
    disagree about the frame convention."""
    v_lon, v_lat = rotate_to_body(v_cmd_world[0], v_cmd_world[1], psi)
    return v_lon, v_lat, wrap_pi(psi_cmd - psi)
