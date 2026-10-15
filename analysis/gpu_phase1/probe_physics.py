"""Phase 1 of docs/plans/gpu-training.md: the bike's compiled model in MJWarp.

Needs only v3_rigid.mjb + v3_init_state.npz (export recipe in the plan) and
mujoco==3.10.0, mujoco-warp==3.10.0.3. No aow_sim. Runs on CPU (dry run) or
CUDA; prints one JSON line per measurement so the numbers can be pasted back.

  python probe_physics.py compile --nworld 1024   # first: kernel compile time
  python probe_physics.py track            # C vs Warp, one world
  python probe_physics.py perworld         # batched model fields per world
  python probe_physics.py sweep --nworld 1024 4096 16384 32768
  python probe_physics.py sweep_mjx --nworld 1024 4096   # reference
"""
import argparse
import json
import time

import mujoco
import numpy as np
import warp as wp
import mujoco_warp as mjw

NCONMAX, NJMAX = 32, 128     # per world; C has 2 contacts / 11 rows at reset
UNLIMITED_RANGE = 0.3   # rad for the steer, N m for the motors


def load():
    mjm = mujoco.MjModel.from_binary_path("v3_rigid.mjb")
    s = np.load("v3_init_state.npz")
    mjd = mujoco.MjData(mjm)
    mjd.qpos[:], mjd.qvel[:], mjd.ctrl[:] = s["qpos"], s["qvel"], s["ctrl"]
    mujoco.mj_forward(mjm, mjd)
    return mjm, mjd


def ctrl_seq(mjm, n, seed=0):
    """Smooth random controls inside ctrlrange: something that moves every
    actuator, so the check is not of a bike standing still."""
    rng = np.random.default_rng(seed)
    lo, hi = mjm.actuator_ctrlrange.T.copy()
    # unlimited actuators: steer target (rad) and the drive motors (N m);
    # drive_a/b are zero-gain integrators the env's drivetrain hook feeds
    free = mjm.actuator_ctrllimited == 0
    lo[free], hi[free] = -UNLIMITED_RANGE, UNLIMITED_RANGE
    knots = rng.uniform(0.2, 0.8, (n // 50 + 2, mjm.nu))
    t = np.arange(n) / 50
    u = np.stack([np.interp(t, np.arange(len(knots)), knots[:, j]) for j in range(mjm.nu)], 1)
    return lo + (hi - lo) * u


def emit(**kw):
    print(json.dumps(kw), flush=True)


def _pose(mjm, qpos):
    """Chassis position (m) and roll/pitch/yaw (deg) from the free joint."""
    j = int(np.flatnonzero(mjm.jnt_type == mujoco.mjtJoint.mjJNT_FREE)[0])
    a = mjm.jnt_qposadr[j]
    w, x, y, z = qpos[a + 3:a + 7]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.asarray(qpos[a:a + 3]), np.degrees([roll, pitch, yaw])


def _joint_names(mjm):
    out = []
    for j in range(mjm.njnt):
        n = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)
        k = {0: 7, 1: 4}.get(int(mjm.jnt_type[j]), 1)
        out += [f"{n}[{i}]" for i in range(k)] if k > 1 else [n]
    return out


def compile_(args):
    """Kernel compile cost, from whatever cache state the machine is in (run
    it first on a fresh pod: that is the cost a Docker image would remove)."""
    t = time.perf_counter()
    mjm, mjd = load()
    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd, nworld=args.nworld[0], nconmax=NCONMAX, njmax=NJMAX)
    t_put = time.perf_counter() - t
    t = time.perf_counter()
    mjw.step(m, d)
    wp.synchronize()
    t_first = time.perf_counter() - t
    t = time.perf_counter()
    mjw.step(m, d)
    wp.synchronize()
    t_second = time.perf_counter() - t
    emit(probe="compile", device=str(wp.get_device()), nworld=args.nworld[0],
         put_s=round(t_put, 3), first_step_s=round(t_first, 2), second_step_s=round(t_second, 4))   # cache size: run_all.sh du's ~/.cache/warp


def track(args):
    """Warp vs C on the chassis pose, against C vs C nudged 1e-7 m: the
    model is chaotic enough that float rounding alone grows, so the nudged
    C run is the yardstick, not zero."""
    mjm, mjd = load()
    names = _joint_names(mjm)
    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd, nworld=1, nconmax=NCONMAX, njmax=NJMAX)
    nud = mujoco.MjData(mjm)
    nud.qpos[:], nud.qvel[:] = mjd.qpos, mjd.qvel
    nud.qpos[2] += 1e-7
    u = ctrl_seq(mjm, args.steps) * args.ctrl_scale
    out = mujoco.MjData(mjm)
    for k in range(args.steps):
        mjd.ctrl[:] = nud.ctrl[:] = u[k]
        mujoco.mj_step(mjm, mjd)
        mujoco.mj_step(mjm, nud)
        wp.copy(d.ctrl, wp.array(u[k][None].astype(np.float32), dtype=float))
        mjw.step(m, d)
        if (k + 1) % args.every == 0 or k + 1 == args.steps:
            mjw.get_data_into(out, mjm, d)
            pc, ac = _pose(mjm, mjd.qpos)
            pw, aw = _pose(mjm, out.qpos)
            pn, an = _pose(mjm, nud.qpos)
            jd = np.abs(out.qpos - mjd.qpos)
            emit(probe="track", t_s=round((k + 1) * mjm.opt.timestep, 4),
                 warp_pos_mm=round(1e3 * float(np.abs(pw - pc).max()), 3),
                 warp_rpy_deg=np.round(np.abs(aw - ac), 3).tolist(),
                 nudge_pos_mm=round(1e3 * float(np.abs(pn - pc).max()), 3),
                 nudge_rpy_deg=np.round(np.abs(an - ac), 3).tolist(),
                 worst_joint=names[int(jd.argmax())], worst_joint_diff=round(float(jd.max()), 4),
                 roll_c_deg=round(float(ac[0]), 2),
                 ncon_c=int(mjd.ncon), ncon_w=int(out.ncon))


def perworld(args):
    """Four worlds, one batched field each run, each world a different
    scale. Pass: world 0 (scale 1) matches C, the others drift away from it
    and from each other."""
    mjm, mjd0 = load()
    n = 4
    scales = np.array([1.0, 0.8, 1.2, 1.5])
    u = ctrl_seq(mjm, args.steps)
    for field in ("body_mass", "geom_friction", "actuator_gainprm",
                  "dof_frictionloss", "dof_damping"):
        m = mjw.put_model(mjm, batch_sizes={field: n})
        arr = getattr(m, field).numpy()          # (n, ...)
        base = np.asarray(getattr(mjm, field))
        # an offset so zero entries (most dofs' damping) vary too; world 0 = C
        new = np.stack([base * s + 0.01 * abs(s - 1) * field.startswith("dof_") for s in scales])
        new = new.astype(arr.dtype)
        getattr(m, field).assign(new.reshape(arr.shape))
        d = mjw.put_data(mjm, mjd0, nworld=n, nconmax=NCONMAX, njmax=NJMAX)
        mjd = mujoco.MjData(mjm)
        mjd.qpos[:], mjd.qvel[:], mjd.ctrl[:] = mjd0.qpos, mjd0.qvel, mjd0.ctrl
        for k in range(args.steps):
            mjd.ctrl[:] = u[k]
            mujoco.mj_step(mjm, mjd)
            wp.copy(d.ctrl, wp.array(np.tile(u[k], (n, 1)).astype(np.float32), dtype=float))
            mjw.step(m, d)
        q = d.qpos.numpy()
        p0, a0 = _pose(mjm, mjd.qpos)
        poses = [_pose(mjm, q[i]) for i in range(n)]
        emit(probe="perworld", field=field, steps=args.steps,
             w0_vs_c_mm=round(1e3 * float(np.abs(poses[0][0] - p0).max()), 3),
             w0_vs_c_rpy_deg=np.round(np.abs(poses[0][1] - a0), 3).tolist(),
             others_vs_w0_mm=[round(1e3 * float(np.abs(p[0] - poses[0][0]).max()), 3) for p in poses[1:]],
             finite=bool(np.isfinite(q).all()))


def sweep(args):
    mjm, mjd = load()
    m = mjw.put_model(mjm)
    for nw in args.nworld:
        d = mjw.put_data(mjm, mjd, nworld=nw, nconmax=NCONMAX, njmax=NJMAX)
        free0, total = _mem()
        mjw.step(m, d)                                   # jit / warm
        wp.synchronize()
        graph = None
        if wp.get_device().is_cuda:
            with wp.ScopedCapture() as cap:
                for _ in range(args.chunk):
                    mjw.step(m, d)
            graph = cap.graph
        reps = max(1, args.steps // args.chunk)
        wp.synchronize()
        t = time.perf_counter()
        for _ in range(reps):
            if graph is not None:
                wp.capture_launch(graph)
            else:
                for _ in range(args.chunk):
                    mjw.step(m, d)
        wp.synchronize()
        dt = time.perf_counter() - t
        free1, _ = _mem()
        q = d.qpos.numpy()
        emit(probe="sweep", nworld=nw, graph=graph is not None,
             steps_per_world=reps * args.chunk,
             physics_steps_per_s=round(nw * reps * args.chunk / dt),
             wall_s=round(dt, 3), mem_used_gb=_gb(free0, free1, total),
             nan_worlds=int((~np.isfinite(q).all(1)).sum()),
             nacon=int(d.nacon.numpy()[0]) if hasattr(d, "nacon") else None)
        del d


def sweep_mjx(args):
    """MJX on the same device, for reference: vmapped step, `chunk` steps
    per jitted scan."""
    import jax
    import jax.numpy as jnp
    from mujoco import mjx
    mjm, mjd = load()
    m = mjx.put_model(mjm)
    x0 = mjx.put_data(mjm, mjd)

    @jax.jit
    def run(x):
        return jax.lax.scan(lambda c, _: (jax.vmap(mjx.step, (None, 0))(m, c), None),
                            x, None, length=args.chunk)[0]

    for nw in args.nworld:
        x = jax.tree_util.tree_map(lambda a: jnp.broadcast_to(a, (nw,) + a.shape), x0)
        x = run(x)
        jax.block_until_ready(x)
        reps = max(1, args.steps // args.chunk)
        t = time.perf_counter()
        for _ in range(reps):
            x = run(x)
        jax.block_until_ready(x)
        dt = time.perf_counter() - t
        q = np.asarray(x.qpos)
        emit(probe="sweep_mjx", nworld=nw, steps_per_world=reps * args.chunk,
             physics_steps_per_s=round(nw * reps * args.chunk / dt),
             wall_s=round(dt, 3), nan_worlds=int((~np.isfinite(q).all(1)).sum()),
             device=str(jax.devices()[0]))


def _mem():
    dev = wp.get_device()
    if dev.is_cuda:
        return dev.free_memory, dev.total_memory
    return None, None


def _gb(free0, free1, total):
    if total is None:
        return None
    return {"total": round(total / 2**30, 2), "free_after": round(free1 / 2**30, 2)}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("probe", choices=["compile", "track", "perworld", "sweep", "sweep_mjx"])
    ap.add_argument("--steps", type=int, default=250)       # 250 x 0.4 ms = 100 ms
    ap.add_argument("--every", type=int, default=25)
    ap.add_argument("--ctrl-scale", type=float, default=1.0)
    ap.add_argument("--chunk", type=int, default=50)        # steps per captured graph
    ap.add_argument("--nworld", type=int, nargs="+", default=[1024, 4096, 16384, 32768])
    a = ap.parse_args()
    wp.config.quiet = True
    {"compile": compile_, "track": track, "perworld": perworld, "sweep": sweep, "sweep_mjx": sweep_mjx}[a.probe](a)
