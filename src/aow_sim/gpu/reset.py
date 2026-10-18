"""`GeneralEnv.reset` for the worlds whose episode just ended, as Warp
kernels, so the reset can live inside the captured step (PufferLib-style
auto-reset).

`reset_worlds` does everything up to `mj_forward`: the domain randomization
(`randomize.DomainRandomizer.apply`: per-body mass, one sliding-friction scale,
one supply scale on force AND ctrl range), the start pose (`init_roll_deg`
etc.), the actuator state (`build_model.reset_actuator_state`: act, the
steer's delay line, the warm start -- here as a fresh MjData's at time 0, and
the world's clock goes to 0 with it), every hook's memory (drivetrain,
gearbox, odometry, AHRS), and the episode's scalars and first command.

Then `mjw.forward` runs -- on every world, since MJWarp cannot mask it -- so
`ForwardGuard` saves and restores what the next substep's hooks read for the
worlds that did NOT reset: the Python env never calls forward mid-episode.
Then the AHRS primes (its kernel with the reset mask) and `reset_obs` builds
the first observation.
"""
from __future__ import annotations

import math

import numpy as np
import warp as wp

from . import commands as gcmd
from .. import sim_ahrs as sa


@wp.struct
class ResetConst:
    rand_on: int
    mass_frac: float
    friction_frac: float
    actuator_frac: float
    init_roll: float          # [rad]
    init_yaw: float
    init_pos: float
    init_vel: float
    fixed_roll: float         # the randomization-off start lean, 0.5 deg
    # floor tilt (GeneralEnv._tilt_floor); tilt_max 0 = off, no draw
    tilt_max: float           # [rad]
    tilt_cur: int             # 1: x the world's difficulty
    floor_g: int              # the floor's geom id
    steer_q: int
    # drivetrain
    belt: float
    period: float
    ga: float
    gb: float
    qa: int
    qb: int
    da: int
    db: int
    motor_a: int
    motor_b: int
    # gearbox
    gb_dof: int
    gb_static: float          # base + s0: the build-time value
    # AHRS
    misalign: float
    rms0: wp.vec3
    tau_orient0: float
    tau_motion0: float


def const_from(env) -> ResetConst:
    r = env.rand
    for k in ("solref_frac", "dampratio_range", "ahrs_orient_rms_deg_range",
              "ahrs_tau_s_range", "drive_supply_randomize", "drive_friction_side_frac"):
        if r.get(k):
            raise NotImplementedError(f"randomization `{k}` is not ported")
    if env.hockey or env._crank is not None:
        raise NotImplementedError("the port covers v3: no ball, no crank servo")
    c = ResetConst()
    c.rand_on = int(bool(r["enabled"]))
    c.mass_frac = float(r["mass_frac"])
    c.friction_frac = float(r["friction_frac"])
    c.actuator_frac = float(r.get("actuator_frac", 0.0))
    c.init_roll = math.radians(float(r["init_roll_deg"]))
    c.init_yaw = math.radians(float(r["init_yaw_deg"]))
    c.init_pos = float(r["init_pos_m"])
    c.init_vel = float(r["init_vel"])
    c.fixed_roll = math.radians(0.5)
    c.tilt_max = float(env.tilt_max)
    c.tilt_cur = int(env.tilt_cur)
    c.floor_g = int(env._floor_g)
    c.steer_q = int(env._sj)
    dr = env._drive
    c.belt, c.period, c.ga, c.gb = float(dr.belt), float(dr.period), float(dr.ga), float(dr.gb)
    c.qa, c.qb = (int(i) for i in dr.qadr)
    c.da, c.db = (int(i) for i in dr.dadr)
    c.motor_a, c.motor_b = (int(i) for i in dr.motor_id)
    g = env._gearbox[0]
    c.gb_dof = int(g.dof)
    c.gb_static = float(g.base + g.s0)
    a = env._ahrs
    c.misalign = math.radians(sa.MISALIGN_DEG.get(a.level, sa.ACCEL_MISALIGN_DEG))
    c.rms0 = wp.vec3(*np.deg2rad(sa.ORIENT_RMS_DEG[a.level]).tolist())
    c.tau_orient0 = float(a.tau_orient_s)
    c.tau_motion0 = float(sa.FILTER_TAU_MOTION_S)
    return c


class Nominal:
    """The model arrays randomization scales, as built, plus the start state."""

    def __init__(self, env, device=None):
        m = env.model
        rz = env._rand
        f = lambda x: wp.array(np.asarray(x, np.float32), dtype=float, device=device)
        self.mass = f(rz._mass0)
        self.friction = f(rz._friction0[:, 0])
        self.forcerange = f(rz._forcerange0)
        self.ctrlrange = f(rz._ctrlrange0)
        self.qpos = f(env._eq)
        import mujoco
        self.history = f(mujoco.MjData(m).history)       # a fresh line at time 0


@wp.func
def _uniform(s: wp.uint32, lo: float, hi: float):
    return lo + wp.randf(s) * (hi - lo), s


@wp.kernel
def reset_worlds(c: ResetConst, cc: gcmd.CmdConst, done: wp.array(dtype=int),
                 rng: wp.array(dtype=wp.uint32),
                 # nominal model + start
                 mass0: wp.array(dtype=float), fric0: wp.array(dtype=float),
                 frange0: wp.array2d(dtype=float), crange0: wp.array2d(dtype=float),
                 qpos0: wp.array(dtype=float), hist0: wp.array(dtype=float),
                 # per-world model fields
                 body_mass: wp.array2d(dtype=float), geom_friction: wp.array2d(dtype=wp.vec3),
                 forcerange: wp.array2d(dtype=wp.vec2), ctrlrange: wp.array2d(dtype=wp.vec2),
                 frictionloss: wp.array2d(dtype=float),
                 # data
                 qpos: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float),
                 act: wp.array2d(dtype=float), history: wp.array2d(dtype=float),
                 qacc_warmstart: wp.array2d(dtype=float), time: wp.array(dtype=float),
                 ctrl: wp.array2d(dtype=float), qfrc_applied: wp.array2d(dtype=float),
                 xfrc_applied: wp.array2d(dtype=wp.spatial_vector),
                 geom_xmat: wp.array2d(dtype=wp.mat33),
                 # env scalars
                 steer_target: wp.array(dtype=float), prev_a: wp.array2d(dtype=float),
                 step: wp.array(dtype=int), track_sum: wp.array(dtype=float),
                 track_n: wp.array(dtype=int), psi: wp.array(dtype=float),
                 raw_prev: wp.array(dtype=float), next_resample: wp.array(dtype=int),
                 diff: wp.array(dtype=float), v_cmd_w: wp.array2d(dtype=float),
                 psi_cmd: wp.array(dtype=float), style: wp.array(dtype=float),
                 latch_obs: wp.array(dtype=float), latch_true: wp.array(dtype=float),
                 # drivetrain state
                 k_c: wp.array(dtype=int), k_m: wp.array(dtype=int),
                 s_cmd: wp.array3d(dtype=float), s_th: wp.array3d(dtype=float),
                 s_w: wp.array3d(dtype=float), s_ref: wp.array2d(dtype=float),
                 s_wf: wp.array2d(dtype=float), s_fa: wp.array2d(dtype=float),
                 s_duty: wp.array2d(dtype=float), phase: wp.array(dtype=float),
                 anchor: wp.array(dtype=float), detent_force: wp.array(dtype=float),
                 # odometry state
                 o_prev_ok: wp.array(dtype=int), o_rcount: wp.array(dtype=int),
                 o_lon: wp.array(dtype=float), o_lat: wp.array(dtype=float),
                 # AHRS state
                 a_oe: wp.array2d(dtype=float), a_gb: wp.array2d(dtype=float),
                 a_yd: wp.array(dtype=float), a_sign: wp.array(dtype=float),
                 a_tilt: wp.array2d(dtype=float), a_u_ok: wp.array(dtype=int),
                 a_wp_ok: wp.array(dtype=int), a_rate: wp.array(dtype=float),
                 a_dev: wp.array(dtype=float), a_rms: wp.array2d(dtype=float),
                 a_tau: wp.array(dtype=float), a_taum: wp.array(dtype=float)):
    world = wp.tid()
    if done[world] == 0:
        return
    s = rng[world]

    # -- domain randomization (DomainRandomizer.apply) --
    for b in range(mass0.shape[0]):
        scale = float(1.0)
        if c.rand_on == 1:
            scale, s = _uniform(s, 1.0 - c.mass_frac, 1.0 + c.mass_frac)
        body_mass[world, b] = mass0[b] * scale
    fs = float(1.0)
    vs = float(1.0)
    if c.rand_on == 1:
        fs, s = _uniform(s, 1.0 - c.friction_frac, 1.0 + c.friction_frac)
        if c.actuator_frac > 0.0:
            vs, s = _uniform(s, 1.0 - c.actuator_frac, 1.0 + c.actuator_frac)
    for g in range(fric0.shape[0]):
        f3 = geom_friction[world, g]
        geom_friction[world, g] = wp.vec3(fric0[g] * fs, f3[1], f3[2])
    for u in range(frange0.shape[0]):
        forcerange[world, u] = wp.vec2(frange0[u, 0] * vs, frange0[u, 1] * vs)
        ctrlrange[world, u] = wp.vec2(crange0[u, 0] * vs, crange0[u, 1] * vs)

    # -- the start pose --
    for i in range(qpos0.shape[0]):
        qpos[world, i] = qpos0[i]
    for i in range(qvel.shape[1]):
        qvel[world, i] = 0.0
    roll = c.fixed_roll
    yaw = float(0.0)
    if c.rand_on == 1:
        r1 = float(0.0)
        r1, s = _uniform(s, -1.0, 1.0)
        roll = r1 * c.init_roll
        y1 = float(0.0)
        y1, s = _uniform(s, -1.0, 1.0)
        yaw = y1 * c.init_yaw
        for i in range(2):
            p1 = float(0.0)
            p1, s = _uniform(s, -1.0, 1.0)
            qpos[world, i] = qpos[world, i] + p1 * c.init_pos
        for i in range(6):
            v1 = float(0.0)
            v1, s = _uniform(s, -1.0, 1.0)
            qvel[world, i] = qvel[world, i] + v1 * c.init_vel
    # quat = q_yaw * q_roll (wxyz)
    cr = wp.cos(0.5 * roll)
    sr = wp.sin(0.5 * roll)
    cy = wp.cos(0.5 * yaw)
    sy = wp.sin(0.5 * yaw)
    qpos[world, 3] = cy * cr
    qpos[world, 4] = cy * sr
    qpos[world, 5] = sy * sr
    qpos[world, 6] = sy * cr

    # -- the floor tilt (GeneralEnv._tilt_floor): OFF unless tilt_max > 0, and
    # then the draws come after every other start-pose draw, as on the CPU.
    # The floor is a world-body geom, which MJWarp poses ONCE in make_data
    # and never again (smooth._geom_local_to_global returns early for it), so
    # its world pose is written here directly; geom_xpos stays at the origin,
    # which the plane passes through. The bike goes on level-to-the-ground,
    # rotated about the origin with the floor, as teleop's place_bike does.
    if c.tilt_max > 0.0:
        frac = float(1.0)
        if c.tilt_cur == 1:
            frac = diff[world]
        t1 = float(0.0)
        t1, s = _uniform(s, 0.0, 1.0)
        th = t1 * c.tilt_max * frac
        br = float(0.0)
        br, s = _uniform(s, -3.14159265358979, 3.14159265358979)
        # unit axis (-sin br, cos br, 0); rotation th about it (wxyz)
        ax = -wp.sin(br)
        ay = wp.cos(br)
        fw = wp.cos(0.5 * th)
        fx = wp.sin(0.5 * th) * ax
        fy = wp.sin(0.5 * th) * ay
        R = wp.quat_to_matrix(wp.quat(fx, fy, 0.0, fw))      # warp: xyzw
        geom_xmat[world, c.floor_g] = R
        tp0 = wp.vec3(qpos[world, 0], qpos[world, 1], qpos[world, 2])
        tp1 = R * tp0
        qpos[world, 0] = tp1[0]
        qpos[world, 1] = tp1[1]
        qpos[world, 2] = tp1[2]
        tv0 = wp.vec3(qvel[world, 0], qvel[world, 1], qvel[world, 2])
        tv1 = R * tv0
        qvel[world, 0] = tv1[0]
        qvel[world, 1] = tv1[1]
        qvel[world, 2] = tv1[2]
        # q_floor * q_bike, wxyz
        bw = qpos[world, 3]
        bx = qpos[world, 4]
        by = qpos[world, 5]
        bz = qpos[world, 6]
        qpos[world, 3] = fw * bw - fx * bx - fy * by
        qpos[world, 4] = fw * bx + fx * bw + fy * bz
        qpos[world, 5] = fw * by - fx * bz + fy * bw
        qpos[world, 6] = fw * bz + fx * by - fy * bx

    # -- actuator state, clock, applied forces --
    for i in range(act.shape[1]):
        act[world, i] = 0.0
    for i in range(hist0.shape[0]):
        history[world, i] = hist0[i]
    for i in range(qacc_warmstart.shape[1]):
        qacc_warmstart[world, i] = 0.0
        qfrc_applied[world, i] = 0.0
    time[world] = 0.0
    for b in range(xfrc_applied.shape[1]):
        xfrc_applied[world, b] = wp.spatial_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # -- drivetrain (DrivetrainSim.reset) --
    k_c[world] = 0
    k_m[world] = 0
    for i in range(2):
        q = qpos[world, c.qa]
        v = qvel[world, c.da]
        if i == 1:
            q = qpos[world, c.qb]
            v = qvel[world, c.db]
        th = q / c.belt
        w = v / c.belt
        s_ref[world, i] = th
        s_wf[world, i] = w
        for j in range(s_cmd.shape[2]):
            s_cmd[world, i, j] = 0.0
        for j in range(s_th.shape[2]):
            s_th[world, i, j] = th
            s_w[world, i, j] = w
        s_duty[world, i] = 0.0
        s_fa[world, i] = q
    ctrl[world, c.motor_a] = 0.0
    ctrl[world, c.motor_b] = 0.0
    ph = float(0.0)
    ph, s = _uniform(s, 0.0, c.period)
    phase[world] = ph
    anchor[world] = c.ga * qpos[world, c.qa] + c.gb * qpos[world, c.qb]
    detent_force[world] = 0.0

    # -- gearbox: the build-time static value --
    frictionloss[world, c.gb_dof] = c.gb_static

    # -- odometry: fresh --
    o_prev_ok[world] = 0
    o_rcount[world] = 0
    o_lon[world] = 0.0
    o_lat[world] = 0.0

    # -- AHRS: fresh realisation, nominal parameters --
    for i in range(3):
        a_oe[world, i] = 0.0
        a_gb[world, i] = 0.0
        t1 = float(0.0)
        t1, s = _uniform(s, -c.misalign, c.misalign)
        a_tilt[world, i] = t1
        a_rms[world, i] = c.rms0[i]
    a_yd[world] = 0.0
    if wp.randf(s) < 0.5:
        a_sign[world] = -1.0
    else:
        a_sign[world] = 1.0
    a_u_ok[world] = 0
    a_wp_ok[world] = 0
    a_rate[world] = 0.0
    a_dev[world] = 0.0
    a_tau[world] = c.tau_orient0
    a_taum[world] = c.tau_motion0

    # -- the episode's scalars and first command --
    # truth yaw of the pose just set (extract_state), = `yaw`
    tw = qpos[world, 3]
    tx = qpos[world, 4]
    ty = qpos[world, 5]
    tz = qpos[world, 6]
    ps = wp.atan2(2.0 * (tx * ty + tw * tz), 1.0 - 2.0 * (ty * ty + tz * tz))
    psi[world] = ps
    raw_prev[world] = ps
    steer_target[world] = qpos[world, c.steer_q]
    for i in range(3):
        prev_a[world, i] = 0.0
    step[world] = 0
    track_sum[world] = 0.0
    track_n[world] = 0
    s = gcmd.sample_family(cc, diff[world], ps, 1, v_cmd_w, psi_cmd, world, s)
    style[world] = 0.0                     # an episode starts with the bit off
    s = gcmd.sample_style(cc, diff[world], v_cmd_w, ps, qpos, style, latch_obs,
                          latch_true, world, s)
    nr = int(0)
    nr, s = gcmd.schedule(cc, 0, s)
    next_resample[world] = nr
    rng[world] = s


class ForwardGuard:
    """Save, then restore after `mjw.forward`, what the next substep's hooks
    read, for the worlds that did NOT reset; zero the warm start of those
    that did (reset_actuator_state's, which mj_forward leaves alone)."""

    FIELDS = ("sensordata", "qfrc_actuator", "qacc_warmstart", "actuator_force")

    def __init__(self, d):
        self.d = d
        self.saved = {k: wp.empty_like(getattr(d, k)) for k in self.FIELDS}
        self.efc = {k: wp.empty_like(getattr(d.efc, k)) for k in ("type", "id", "force")}
        self.counts = {k: wp.empty_like(getattr(d, k)) for k in ("ne", "nf", "nl", "nefc")}

    def save(self):
        for k, a in self.saved.items():
            wp.copy(a, getattr(self.d, k))
        for k, a in self.efc.items():
            wp.copy(a, getattr(self.d.efc, k))
        for k, a in self.counts.items():
            wp.copy(a, getattr(self.d, k))

    def restore(self, done):
        n = done.shape[0]
        for k, a in self.saved.items():
            wp.launch(_blend2f, n, [done, a, getattr(self.d, k)], device=done.device)
        wp.launch(_blend2i, n, [done, self.efc["type"], self.d.efc.type], device=done.device)
        wp.launch(_blend2i, n, [done, self.efc["id"], self.d.efc.id], device=done.device)
        wp.launch(_blend2f, n, [done, self.efc["force"], self.d.efc.force], device=done.device)
        for k, a in self.counts.items():
            wp.launch(_blend1i, n, [done, a, getattr(self.d, k)], device=done.device)
        wp.launch(_zero_where_done, n, [done, self.d.qacc_warmstart], device=done.device)


@wp.kernel
def _blend2f(done: wp.array(dtype=int), saved: wp.array2d(dtype=float),
             live: wp.array2d(dtype=float)):
    w = wp.tid()
    if done[w] == 0:
        for i in range(live.shape[1]):
            live[w, i] = saved[w, i]


@wp.kernel
def _blend2i(done: wp.array(dtype=int), saved: wp.array2d(dtype=int),
             live: wp.array2d(dtype=int)):
    w = wp.tid()
    if done[w] == 0:
        for i in range(live.shape[1]):
            live[w, i] = saved[w, i]


@wp.kernel
def _blend1i(done: wp.array(dtype=int), saved: wp.array(dtype=int),
             live: wp.array(dtype=int)):
    w = wp.tid()
    if done[w] == 0:
        live[w] = saved[w]


@wp.kernel
def _zero_where_done(done: wp.array(dtype=int), a: wp.array2d(dtype=float)):
    w = wp.tid()
    if done[w] == 1:
        for i in range(a.shape[1]):
            a[w, i] = 0.0
