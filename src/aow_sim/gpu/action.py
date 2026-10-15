"""The policy-step half of `GeneralEnv.step` before physics, as a Warp
kernel: clip and scale the action (`flick_spec.scale_action`), integrate the
steer RATE into a position target kept within STEER_LEAD_MAX of the measured
steer (`steer.advance_target`), and mix hub + differential into the two
drive commands (`balance.mix`). The full 3-channel action only -- v3's.
"""
from __future__ import annotations

import warp as wp


@wp.struct
class ActConst:
    steer_rate_max: float
    hub_max: float
    diff_max: float
    dt: float          # control period
    lead_max: float    # steer.STEER_LEAD_MAX
    r_rear: float      # omni wheel outer radius
    steer_q: int       # qpos address of steer_joint
    ctrl_a: int
    ctrl_b: int
    ctrl_steer: int


def const_from(env) -> ActConst:
    from ..control.steer import STEER_LEAD_MAX
    if not env.full or env.act_wings or env.act_swing or \
            env.diff_fade[1] > env.diff_fade[0]:
        raise NotImplementedError("the port covers the plain 3-channel action")
    c = ActConst()
    c.steer_rate_max = float(env.bounds.steer_rate_max)
    c.hub_max = float(env.bounds.hub_max)
    c.diff_max = float(env.bounds.diff_max)
    c.dt = float(env.ctrl_dt)
    c.lead_max = float(STEER_LEAD_MAX)
    c.r_rear = float(env._r_rear)
    c.steer_q = int(env._sj)
    c.ctrl_a, c.ctrl_b, c.ctrl_steer = (int(env._aid[k]) for k in ("drive_a", "drive_b", "steer"))
    return c


@wp.kernel
def apply_action(c: ActConst, action: wp.array2d(dtype=float),
                 qpos: wp.array2d(dtype=float), steer_target: wp.array(dtype=float),
                 ctrl: wp.array2d(dtype=float)):
    world = wp.tid()
    rate = wp.clamp(action[world, 0], -1.0, 1.0) * c.steer_rate_max
    hub = wp.clamp(action[world, 1], -1.0, 1.0) * c.hub_max
    diff = wp.clamp(action[world, 2], -1.0, 1.0) * c.diff_max
    measured = qpos[world, c.steer_q]
    t = steer_target[world] + rate * c.dt
    t = wp.clamp(t, measured - c.lead_max, measured + c.lead_max)
    steer_target[world] = t
    common = hub / c.r_rear
    ctrl[world, c.ctrl_a] = common + 0.5 * diff
    ctrl[world, c.ctrl_b] = common - 0.5 * diff
    ctrl[world, c.ctrl_steer] = t


def launch_apply_action(c: ActConst, action, qpos, steer_target, ctrl):
    wp.launch(apply_action, dim=action.shape[0],
              inputs=[c, action, qpos, steer_target, ctrl], device=action.device)
