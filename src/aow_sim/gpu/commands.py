"""GeneralEnv's command side as Warp code: the FAMILY sampler
(`_sample_family_command`, `_family_weights`, `_signed_magnitude`), the
resample schedule (`_schedule_resample`), and the curriculum
(`_advance_curriculum`). Difficulty is PER WORLD, as each GeneralEnv in a
vectorised training run carries its own.

Random draws come from Warp's per-world generator, so they cannot reproduce
numpy's stream; analysis/gpu/phase2/check_commands.py compares the
distributions instead.
"""
from __future__ import annotations

import numpy as np
import warp as wp


@wp.struct
class CmdConst:
    hold_min: float
    hold_max: float
    hold_decay: float
    tip_on: float
    tip_full: float
    tip_w: float
    str_on: float
    str_full: float
    str_w: float
    mov_on: float
    mov_full: float
    mov_w: float
    v_max: float
    v_lo: float
    step_lo: float
    v_exp: float
    step_exp: float
    res_lo: float
    res_hi: float
    ctrl_dt: float
    cur_on: int
    diff_step: float
    diff_thresh: float
    style_on: int         # draw the style bit with every command
    style_p: float        # P(on) = style_p x difficulty
    steer_q: int          # for the latch: the steer's qpos address
    cos_rake: float


def const_from(env) -> CmdConst:
    f = env.families
    if f is None:
        raise NotImplementedError("the port covers the family sampler only")
    c = CmdConst()
    c.hold_min, c.hold_max, c.hold_decay = f["hold_min"], f["hold_max"], f["hold_decay"]
    c.tip_on, c.tip_full, c.tip_w = f["turn_in_place"]
    c.str_on, c.str_full, c.str_w = f["straight"]
    c.mov_on, c.mov_full, c.mov_w = f["moving_turn"]
    c.v_max, c.v_lo, c.step_lo = float(env.v_max), float(env.v_lo), float(env.step_lo)
    c.v_exp, c.step_exp = float(env.v_exponent), float(env.step_exponent)
    c.res_lo, c.res_hi = (float(x) for x in env.resample_s)
    c.ctrl_dt = float(env.ctrl_dt)
    c.cur_on = int(env.cur_on)
    c.diff_step, c.diff_thresh = float(env.diff_step), float(env.diff_thresh)
    c.style_on, c.style_p = int(env.obs_style), float(env.style_p)
    c.steer_q, c.cos_rake = int(env._sj), float(np.cos(env._rake))
    return c


@wp.func
def _ramp(d: float, onset: float, full: float):
    frac = float(0.0)
    if d >= full:
        frac = 1.0
    elif d <= onset:
        frac = 0.0
    else:
        frac = (d - onset) / (full - onset)
    return frac


@wp.func
def signed_magnitude(lo: float, hi: float, exponent: float, s: wp.uint32):
    """+-|x|, |x| with density ~ |x|^exponent on [lo, hi]; returns (x, state)."""
    u = wp.randf(s)
    k = exponent + 1.0
    mag = wp.pow(wp.pow(lo, k) + u * (wp.pow(hi, k) - wp.pow(lo, k)), 1.0 / k)
    if wp.randf(s) >= 0.5:
        mag = -mag
    return mag, s


@wp.func
def sample_family(c: CmdConst, d: float, psi: float, first: int,
                  v_cmd_w: wp.array2d(dtype=float), psi_cmd: wp.array(dtype=float),
                  world: int, s: wp.uint32):
    """One draw of `_sample_family_command`; returns the advanced state."""
    p_hold = c.hold_min + (c.hold_max - c.hold_min) * wp.pow(1.0 - d, c.hold_decay)
    r_tip = c.tip_w * _ramp(d, c.tip_on, c.tip_full)
    r_str = c.str_w * _ramp(d, c.str_on, c.str_full)
    r_mov = c.mov_w * _ramp(d, c.mov_on, c.mov_full)
    total = r_tip + r_str + r_mov
    p_tip = float(0.0)
    p_str = float(0.0)
    if total <= 0.0:
        p_hold = 1.0
    else:
        share = (1.0 - p_hold) / total
        p_tip = r_tip * share
        p_str = r_str * share
    u = wp.randf(s)
    v_hi = c.v_max * (0.25 + 0.75 * d)
    step_hi = c.step_lo + (wp.pi - c.step_lo) * d
    c_hold = p_hold
    c_tip = c_hold + p_tip
    c_str = c_tip + p_str
    v_lon = float(0.0)
    if u >= c_tip:                                   # straight, or moving turn
        v_lon, s = signed_magnitude(c.v_lo, wp.max(v_hi, c.v_lo), c.v_exp, s)
    step = float(0.0)
    want_step = (u >= c_hold and u < c_tip) or u >= c_str
    if want_step and first == 0:
        step, s = signed_magnitude(c.step_lo, wp.max(step_hi, c.step_lo), c.step_exp, s)
    if v_lon == 0.0 and step == 0.0:
        # a hold: psi_cmd stays where it was (anchored at reset)
        v_cmd_w[world, 0] = 0.0
        v_cmd_w[world, 1] = 0.0
        if first == 1:
            psi_cmd[world] = psi
    else:
        pc = psi + step
        v_cmd_w[world, 0] = wp.cos(pc) * v_lon
        v_cmd_w[world, 1] = wp.sin(pc) * v_lon
        psi_cmd[world] = pc
    return s


@wp.func
def wheel_world(psi: float, steer: float, cos_rake: float):
    """general_spec.wheel_world_heading."""
    return psi + wp.atan2(cos_rake * wp.sin(steer), wp.cos(steer))


@wp.func
def truth_yaw(qpos: wp.array2d(dtype=float), world: int):
    """extract_state's yaw off the free joint's quaternion."""
    tw = qpos[world, 3]
    tx = qpos[world, 4]
    ty = qpos[world, 5]
    tz = qpos[world, 6]
    return wp.atan2(2.0 * (tx * ty + tw * tz), 1.0 - 2.0 * (ty * ty + tz * tz))


@wp.func
def sample_style(c: CmdConst, d: float, v_cmd_w: wp.array2d(dtype=float),
                 psi: float, qpos: wp.array2d(dtype=float), style: wp.array(dtype=float),
                 latch_obs: wp.array(dtype=float), latch_true: wp.array(dtype=float),
                 world: int, s: wp.uint32):
    """`_sample_style` + `_set_style`: after the command, before the schedule;
    no draw when off. Active only at a zero speed command; the wheel's world
    heading latched on the rising edge (the bike's psi, and truth)."""
    if c.style_on == 1:
        drawn = wp.randf(s) < c.style_p * d
        on = float(0.0)
        if drawn and v_cmd_w[world, 0] == 0.0 and v_cmd_w[world, 1] == 0.0:
            on = 1.0
        if on == 1.0 and style[world] == 0.0:
            steer = qpos[world, c.steer_q]
            latch_obs[world] = wheel_world(psi, steer, c.cos_rake)
            latch_true[world] = wheel_world(truth_yaw(qpos, world), steer, c.cos_rake)
        style[world] = on
    return s


@wp.func
def schedule(c: CmdConst, step: int, s: wp.uint32):
    """`_schedule_resample`: the step the next change lands on."""
    t = c.res_lo + wp.randf(s) * (c.res_hi - c.res_lo)
    n = int(wp.round(t / c.ctrl_dt))
    return step + wp.max(1, n), s


@wp.kernel
def after_step(c: CmdConst, terminated: wp.array(dtype=int),
               truncated: wp.array(dtype=int), step: wp.array(dtype=int),
               psi: wp.array(dtype=float), track_sum: wp.array(dtype=float),
               track_n: wp.array(dtype=int), next_resample: wp.array(dtype=int),
               diff: wp.array(dtype=float), v_cmd_w: wp.array2d(dtype=float),
               psi_cmd: wp.array(dtype=float), qpos: wp.array2d(dtype=float),
               style: wp.array(dtype=float), latch_obs: wp.array(dtype=float),
               latch_true: wp.array(dtype=float), rng: wp.array(dtype=wp.uint32)):
    """The end of GeneralEnv.step after the reward: a live resample, or at an
    episode end the curriculum's advance (the reset is a separate kernel)."""
    world = wp.tid()
    s = rng[world]
    done = terminated[world] == 1 or truncated[world] == 1
    if (not done) and step[world] >= next_resample[world]:
        s = sample_family(c, diff[world], psi[world], 0, v_cmd_w, psi_cmd, world, s)
        s = sample_style(c, diff[world], v_cmd_w, psi[world], qpos, style, latch_obs,
                         latch_true, world, s)
        nr = int(0)
        nr, s = schedule(c, step[world], s)
        next_resample[world] = nr
    if done and c.cur_on == 1:
        score = float(0.0)
        if terminated[world] == 0:
            score = track_sum[world] / float(wp.max(1, track_n[world]))
        if score > c.diff_thresh:
            diff[world] = wp.min(1.0, diff[world] + c.diff_step)
    rng[world] = s


@wp.kernel
def draw_many(c: CmdConst, d: float, rng: wp.array(dtype=wp.uint32),
              v_cmd_w: wp.array2d(dtype=float), psi_cmd: wp.array(dtype=float),
              next_resample: wp.array(dtype=int)):
    """For check_commands: one fresh (not first) draw per thread at psi 0."""
    world = wp.tid()
    s = rng[world]
    s = sample_family(c, d, 0.0, 0, v_cmd_w, psi_cmd, world, s)
    nr = int(0)
    nr, s = schedule(c, 0, s)
    next_resample[world] = nr
    rng[world] = s


@wp.kernel
def seed_rng(seed: int, rng: wp.array(dtype=wp.uint32)):
    world = wp.tid()
    rng[world] = wp.rand_init(seed, world)
