"""What would a CHEAPER omni-wheel contact model cost us, and what would it buy?

The shipped wheel is anatomically correct: 8 axles, 16 truncated-cone meshes, 8
hinge joints, 8 equality constraints (docs/plans/mujoco-modeling-decisions.md).
That file records "fast approximation models ... deferred until this reference
model is validated". This script is the deferred measurement.

WHAT IT VARIES. The same `config/bike_params.yaml`, the same drivetrain
kinematics, the same actuators — only the rear-wheel contact scheme changes:

  cones     the shipped model: n axles x 2 truncated-cone MESHES     (reference)
  spheres2  n axles x 2 SPHERES at the cone big-end ridge stations
  spheres1  n axles x 1 SPHERE centred on the axle
  capsules  n axles x 1 CAPSULE spanning the pair
  torus     no rollers at all: one crowned disc (minor radius = roller radius)
            on the hub. Rides and leans like the real wheel; CANNOT CRAB.
            Included as the cost floor and the smoothness ceiling, not as a
            candidate.

THE RIG, AND WHY NOT THE BIKE. Every measurement below runs a loaded carriage
(3 slide DOFs + optional roll) on the omni wheel alone. A whole bike would make
the comparison depend on whether each variant happened to stay upright, which
is a controller question, not a contact question. The rig loads the wheel at
0.60 kg -- roughly the rear-axle share of the bike -- and drives it open loop.

THE ARM THAT ANSWERS THE QUESTION is `--study transfer`: one trained policy,
unmodified weights, the standard eval grid, only the rear wheel swapped. Note
that `cones-8` there is this file's fork of the shipped builder and must score
IDENTICALLY to `unpatched` -- that equality is the control, and if it ever
breaks, every other row in this script is measuring the fork instead of the
question.

  python analysis/contact_surrogates.py                # everything but transfer
  python analysis/contact_surrogates.py --study transfer
  python analysis/contact_surrogates.py --study cost chatter

Writes analysis/plots/contact_surrogates.png and prints the tables. Builds its
own models; reads no run artifacts and writes nothing else.
"""

from __future__ import annotations

import argparse
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from aow_sim import geometry
from aow_sim.build_model import (
    DYN_CONAFF, DYN_CONTYPE, _add_world, _apply_options, _contact_friction,
    _quat_z_to, _Y_AXIS_QUAT, load_params,
)

MODES = ("cones", "spheres2", "spheres1", "capsules", "torus")
# Read from the config rather than hard-coded, so the "<- shipped" markers
# below can never quietly point at a setting the repo has moved off.
_p0 = load_params()["sim"]
SHIPPED = (float(_p0["timestep"]), int(_p0["mesh_segments"]))
LOAD_KG = 0.60          # rear-axle share of the bike, near enough
DRIVE_RAD_S = 20.0      # both input shafts -> ~1.0 m/s, mid-envelope


# --------------------------------------------------------------------------
# model construction
# --------------------------------------------------------------------------
def _add_aow(spec, parent, p, mode="cones", n_axles=None):
    """`build_model._add_aow` with the contact scheme as a parameter.

    A near-copy rather than an import: the point is to vary the one thing the
    shipped builder deliberately fixes, and forking it here keeps
    build_model.py free of a knob nothing in the product needs.
    """
    ow, dt, sim = p["omni_wheel"], p["drivetrain"], p["sim"]
    roller = ow["roller"]
    R, a = ow["outer_radius"], ow["axle_mount_radius"]
    n = n_axles or ow["n_axles"]

    hub = parent.add_body(name="aow_hub")
    hub.add_joint(name="hub_spin", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0],
                  damping=dt["hub_joint_damping"],
                  frictionloss=dt["hub_joint_frictionloss"])
    hub.add_geom(name="hub_body", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                 size=[ow["hub"]["body_radius"], ow["hub"]["body_width"] / 2, 0],
                 quat=_Y_AXIS_QUAT, mass=ow["hub"]["mass"],
                 contype=0, conaffinity=0, rgba=[0.25, 0.25, 0.3, 1])

    ring = hub.add_body(name="roller_ring")
    ring.add_joint(name="ring_spin", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    ring.add_geom(name="ring_body", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                  size=[ow["ring"]["body_radius"], ow["ring"]["body_width"] / 2, 0],
                  quat=_Y_AXIS_QUAT, mass=ow["ring"]["mass"],
                  contype=0, conaffinity=0, rgba=[0.8, 0.5, 0.1, 1])

    fr = dict(contype=DYN_CONTYPE, conaffinity=DYN_CONAFF, condim=sim["condim"],
              friction=_contact_friction(sim), rgba=[0.15, 0.15, 0.15, 1])
    # Total roller mass held fixed across n, so a roller-count sweep is a
    # geometry sweep and not also an inertia sweep.
    m_pair = roller["pair_mass"] * ow["n_axles"] / n
    s_ridge = roller["pair_gap"] / 2                        # cone big-end ridge
    s_center = s_ridge + roller["length"] / 2

    if mode == "cones":
        cone = spec.add_mesh(name="roller_cone")
        cone.uservert = geometry.truncated_cone_vertices(
            roller["big_diameter"] / 2, roller["small_diameter"] / 2,
            roller["length"], sim["mesh_segments"]).flatten()

    if mode == "torus":
        disc = spec.add_mesh(name="rear_tyre")
        rho = roller["big_diameter"] / 2
        disc.uservert = geometry.crowned_wheel_vertices(
            R, 1.96 * rho, rho, sim["mesh_segments"]).flatten()
        hub.add_geom(name="rear_disc", type=mujoco.mjtGeom.mjGEOM_MESH,
                     meshname="rear_tyre", quat=_Y_AXIS_QUAT,
                     mass=roller["pair_mass"] * ow["n_axles"], **fr)
    else:
        for i in range(n):
            th = 2 * np.pi * i / n
            radial = np.array([np.cos(th), 0.0, np.sin(th)])
            tangent = np.array([-np.sin(th), 0.0, np.cos(th)])
            axle = hub.add_body(name=f"roller_axle_{i}", pos=a * radial)
            axle.add_joint(name=f"roller_spin_{i}", type=mujoco.mjtJoint.mjJNT_HINGE,
                           axis=tangent, damping=dt["roller_joint_damping"],
                           frictionloss=dt["roller_joint_frictionloss"])
            if mode == "cones":
                for side in (-1, 1):
                    axle.add_geom(name=f"roller_{i}_{side}",
                                  type=mujoco.mjtGeom.mjGEOM_MESH, meshname="roller_cone",
                                  pos=side * s_center * tangent,
                                  quat=_quat_z_to(side * tangent), mass=m_pair / 2, **fr)
            elif mode == "spheres2":
                # Radius set so the sphere tops out on the wheel envelope at the
                # station where the real cone's ridge does.
                rad = R - np.hypot(a, s_ridge)
                for side in (-1, 1):
                    axle.add_geom(name=f"roller_{i}_{side}",
                                  type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[rad, 0, 0],
                                  pos=side * s_ridge * tangent, mass=m_pair / 2, **fr)
            elif mode == "spheres1":
                axle.add_geom(name=f"roller_{i}", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                              size=[R - a, 0, 0], mass=m_pair, **fr)
            elif mode == "capsules":
                axle.add_geom(name=f"roller_{i}", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                              size=[R - a, s_ridge + roller["length"] / 2, 0],
                              quat=_quat_z_to(tangent), mass=m_pair, **fr)
            else:
                raise ValueError(f"unknown mode {mode!r}")

        for i in range(n):
            eq = spec.add_equality()
            eq.type = mujoco.mjtEq.mjEQ_JOINT
            eq.name1, eq.name2 = f"roller_spin_{i}", "ring_spin"
            eq.data[:5] = [0.0, dt["k_roller"], 0.0, 0.0, 0.0]
            eq.solref = [0.005, 1.0]

    y_off = ow["width"] / 2 + dt["input_pulley_offset"]
    for tag, y in (("a", y_off), ("b", -y_off)):
        shaft = parent.add_body(name=f"input_{tag}", pos=[0, y, 0])
        shaft.add_joint(name=f"input_{tag}_spin", type=mujoco.mjtJoint.mjJNT_HINGE,
                        axis=[0, 1, 0], armature=dt["input_armature"])
        shaft.add_geom(name=f"input_{tag}_pulley", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                       size=[0.008, 0.004, 0], quat=_Y_AXIS_QUAT, mass=0.005,
                       contype=0, conaffinity=0, rgba=[0.6, 0.6, 0.65, 1])

    for name, wraps in {
        "gear_hub": [("hub_spin", 1.0), ("input_a_spin", -dt["mix_hub_a"]),
                     ("input_b_spin", -dt["mix_hub_b"])],
        "gear_ring": [("ring_spin", 1.0), ("hub_spin", 1.0),
                      ("input_a_spin", -dt["mix_ring_a"]),
                      ("input_b_spin", -dt["mix_ring_b"])],
    }.items():
        ten = spec.add_tendon(name=name)
        for joint, coef in wraps:
            ten.wrap_joint(joint, coef)
        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_TENDON
        eq.name1 = name
        eq.solref = [0.005, 1.0]

    servo, belt = p["servos"]["xc430_w150"], dt["belt_ratio"]
    for tag in ("a", "b"):
        act = spec.add_actuator(name=f"drive_{tag}")
        act.set_to_velocity(kv=p["actuators"]["drive_kv"])
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.target = f"input_{tag}_spin"
        speed = servo["no_load_rpm"] * 2 * np.pi / 60 * belt
        act.ctrlrange = [-speed, speed]
        act.forcerange = [-servo["stall_torque"] / belt, servo["stall_torque"] / belt]


def build_rig(mode="cones", n_axles=None, timestep=None, segments=None,
              solref=None, roll=False):
    """Loaded carriage on the omni wheel. Identical across every variant."""
    p = load_params()
    if timestep:
        p["sim"]["timestep"] = timestep
    if segments:
        p["sim"]["mesh_segments"] = segments
    if solref:
        p["sim"]["contact_solref"] = list(solref)
    spec = mujoco.MjSpec()
    spec.modelname = f"aow_rig_{mode}"
    _apply_options(spec, p)
    _add_world(spec, p)
    car = spec.worldbody.add_body(name="carriage",
                                  pos=[0, 0, p["omni_wheel"]["outer_radius"]])
    for nm, ax in (("car_x", [1, 0, 0]), ("car_y", [0, 1, 0]), ("car_z", [0, 0, 1])):
        car.add_joint(name=nm, type=mujoco.mjtJoint.mjJNT_SLIDE, axis=ax)
    if roll:
        car.add_joint(name="car_roll", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0])
    car.add_geom(name="car_mass", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.02] * 3,
                 pos=[0, 0, 0.05], mass=LOAD_KG, contype=0, conaffinity=0,
                 rgba=[0.3, 0.5, 0.8, 0.5])
    _add_aow(spec, car, p, mode=mode, n_axles=n_axles)
    return spec.compile()


def _actuators(model):
    return {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
            for i in range(model.nu)}


# --------------------------------------------------------------------------
# one measurement: drive the rig and watch the contact
# --------------------------------------------------------------------------
def drive(model, seconds=2.0, settle=1.0, spd=DRIVE_RAD_S):
    """Roll the rig at `spd` and report speed, ride height ripple, how often
    the wheel is off the ground, and the peak normal force in body weights."""
    d = mujoco.MjData(model)
    dt = model.opt.timestep
    act = _actuators(model)
    for _ in range(int(settle / dt)):
        mujoco.mj_step(model, d)
    d.ctrl[act["drive_a"]] = d.ctrl[act["drive_b"]] = spd
    for _ in range(int(settle / dt)):
        mujoco.mj_step(model, d)

    n = int(seconds / dt)
    z = np.empty(n)
    weight = 9.81 * float(np.sum(model.body_mass))
    f, air, peak, pen = np.zeros(6), 0, 0.0, 0.0
    t0 = time.perf_counter()
    for i in range(n):
        mujoco.mj_step(model, d)
        z[i] = d.qpos[2]
        air += d.ncon == 0
        total = 0.0
        for c in range(d.ncon):
            mujoco.mj_contactForce(model, d, c, f)
            total += f[0]
            pen = min(pen, d.contact.dist[c])
        peak = max(peak, total)
    wall = time.perf_counter() - t0
    # Report the ride-height swing alongside the airborne FRACTION, never the
    # fraction alone -- analysis/liftoff.py makes the case: a wheel that breaks
    # contact by 0.1 mm and one that clears 5 mm score the same percentage.
    return dict(v=float(d.qvel[0]), z_pp_mm=1e3 * (z.max() - z.min()),
                airborne=air / n, peak_w=peak / weight, pen_mm=-1e3 * pen,
                realtime=seconds / wall, steps_s=n / wall)


# --------------------------------------------------------------------------
# studies
# --------------------------------------------------------------------------
def study_cost(dt=None):
    """Where does the step time actually go?"""
    print("\n=== COST: what is the roller machinery worth in step time? ===")
    print(f"{'rear wheel':12s} {'nv':>3s} {'ngeom':>5s} {'neq':>3s} "
          f"{'ksteps/s':>9s} {'speedup':>7s}")
    base = None
    for mode in MODES:
        m = build_rig(mode, timestep=dt)
        r = drive(m)
        base = base or r["steps_s"]
        print(f"{mode:12s} {m.nv:3d} {m.ngeom:5d} {m.neq:3d} "
              f"{r['steps_s'] / 1e3:9.1f} {r['steps_s'] / base:7.2f}")
    print("  Reading: swapping 16 cone MESHES for primitives buys ~10-15%. Deleting")
    print("  the roller multibody entirely (torus) buys ~2x. The cost is DOFs and")
    print("  equality rows, not collision geometry -- so cheaper roller shapes are")
    print("  not where a speedup lives.")


def study_chatter(dt=None):
    """How rough is the ride, and does simplifying the geometry smooth it?"""
    print("\n=== CHATTER: ride height ripple and contact loss at ~1 m/s ===")
    print(f"{'rear wheel':16s} {'v m/s':>6s} {'z p-p mm':>8s} {'off ground':>10s} "
          f"{'peak Fn/W':>9s}")
    rows = [("cones", 8), ("cones", 4), ("cones", 16), ("cones", 24),
            ("spheres1", 8), ("spheres2", 8), ("capsules", 8), ("torus", None)]
    out = []
    for mode, n in rows:
        r = drive(build_rig(mode, n_axles=n, timestep=dt))
        tag = f"{mode}-{n}" if n else mode
        out.append((tag, r))
        print(f"{tag:16s} {r['v']:6.2f} {r['z_pp_mm']:8.3f} {r['airborne']*100:9.1f}% "
              f"{r['peak_w']:9.2f}")
    print("  READ THE TWO COLUMNS TOGETHER. cones-24 breaks contact as often as")
    print("  cones-8 but swings 0.13 mm doing it against 0.84 mm -- same percentage,")
    print("  different phenomenon (the point analysis/liftoff.py makes).")
    print("  The 8-fold ripple is REAL: docs/measurements/omni-wheel-protocol.md §1")
    print("  measures Ø102.35 max / Ø101.75 min, i.e. 0.60 mm, and the model")
    print("  reproduces it. Note that every geometric SIMPLIFICATION makes the ride")
    print("  rougher, never smoother -- the cone pair's swept contact line is a")
    print("  better envelope than one sphere per axle.")

    print("\n  Is any of it numerical? Tessellation sweep on the SMOOTH tyre, where")
    print("  faceting is the only ripple there is (the 8-roller polygon swamps it):")
    print(f"  {'segments':>8s} {'facet um':>9s} {'z p-p mm':>8s} {'off ground':>10s} {'peak Fn/W':>9s}")
    for seg in (16, 32, 64, 128):
        r = drive(build_rig("torus", timestep=dt, segments=seg))
        facet = 1e6 * 0.0512 * (1 - np.cos(np.pi / seg))
        print(f"  {seg:8d} {facet:9.1f} {r['z_pp_mm']:8.3f} {r['airborne']*100:9.1f}% "
              f"{r['peak_w']:9.2f}")
    print("  A perfectly round wheel at the shipped mesh_segments: 32 still bounces.")
    print("  That is pure numerics, and it is what the FRONT tyre is made of. 64")
    print("  segments costs ~3% of step time on the full bike and removes most of it.")
    return out


def study_stiffness(dt=None):
    """Is the bouncing geometry, or is it the least-known parameter in the model?"""
    print("\n=== STIFFNESS: contact_solref moves the chatter more than geometry does ===")
    print(f"{'solref':22s} {'rest sink mm':>12s} {'off ground':>10s} {'peak Fn/W':>9s}")
    for solref in [(0.005, 1.0), (0.005, 0.5), (0.01, 1.0), (0.01, 2.0),
                   (0.02, 1.0), (0.02, 2.0), (0.02, 4.0), (0.04, 2.0)]:
        m = build_rig("cones", timestep=dt, solref=solref)
        d = mujoco.MjData(m)
        for _ in range(int(1.2 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        sink = -1e3 * min(d.contact.dist[:d.ncon]) if d.ncon else float("nan")
        r = drive(m)
        tag = f"[{solref[0]}, {solref[1]}]" + (" <- shipped" if solref == (0.005, 1.0) else "")
        print(f"{tag:22s} {sink:12.3f} {r['airborne']*100:9.1f}% {r['peak_w']:9.2f}")
    print("  Softer and OVERDAMPED (dampratio > 1, which is what filled TPU does)")
    print("  cuts contact loss and halves peak load at 8 rollers, unchanged. The")
    print("  randomizer currently sweeps dampratio 0.2-1.0, i.e. only the bouncier")
    print("  half. contact_solref is still a GUESS: docs/measurements/contact-protocol.md.")


def study_refsafe():
    """`sim.timestep` and `sim.contact_solref` are one decision, not two.

    Two couplings. The HARD one is MuJoCo's `refsafe` (mjDSBL_REFSAFE, on
    unless disabled): a POSITIVE solref timeconst is silently raised to
    2*timestep, so past dt = timeconst/2 the sim is not modelling the contact
    that was configured. The SOFT one arrives earlier and gradually -- the step
    has to resolve the contact's own time constant, and what governs is
    timeconst/timestep, not either alone.
    """
    def rest_sink(dt, solref, refsafe=True):
        m = build_rig("cones", timestep=dt, solref=solref)
        if not refsafe:
            m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_REFSAFE)
        d = mujoco.MjData(m)
        for _ in range(int(1.5 / dt)):
            mujoco.mj_step(m, d)
        if not d.ncon or not np.all(np.isfinite(d.qpos)) or abs(d.qpos[2]) > 0.05:
            return float("nan")
        return -1e3 * float(np.min(d.contact.dist[:d.ncon]))

    dt = 1.0e-3
    print("\n=== REFSAFE: MuJoCo raises timeconst to 2*timestep without saying so ===")
    print(f"timestep {dt:g} s, so the floor is 2*dt = {2 * dt:g} s")
    print(f"{'timeconst':>10s} {'tc/(2dt)':>9s} {'refsafe ON':>12s} {'refsafe OFF':>12s}")
    for tc in (2.5e-4, 5e-4, 1e-3, 1.5e-3, 2e-3, 3e-3, 5e-3, 1e-2):
        print(f"{tc:10.1e} {tc / (2 * dt):9.2f} {rest_sink(dt, (tc, 1.0)):12.4f} "
              f"{rest_sink(dt, (tc, 1.0), refsafe=False):12.4f}")
    print("  Rest sink [mm]. Every timeconst at or below 2*dt gives the SAME contact,")
    print("  because it is not the one asked for -- it is 2*dt. Turning refsafe off")
    print("  shows what the clamp is protecting against.")

    print("\n  The soft limit: steps per contact time constant, rolling at ~1 m/s.")
    print(f"  {'timeconst':>10s} {'timestep':>9s} {'steps/tc':>9s} {'2dt/tc':>7s} "
          f"{'peak Fn/W':>10s} {'vs finest':>10s}")
    for tc in (0.002, 0.005, 0.020):
        ref = None
        for step in (5e-5, 2e-4, 4e-4, 6e-4, 1e-3, 2e-3):
            if 2 * step > tc:
                print(f"  {tc:10.3f} {step:9.1e} {tc / step:9.1f} {2 * step / tc:7.2f}"
                      "   -- refsafe clamps from here --")
                break
            r = drive(build_rig("cones", timestep=step, solref=(tc, 1.0)))
            ref = ref or r["peak_w"]
            print(f"  {tc:10.3f} {step:9.1e} {tc / step:9.1f} {2 * step / tc:7.2f} "
                  f"{r['peak_w']:10.2f} {r['peak_w'] / ref:9.2f}x")
    print("  Roughly ten steps per contact time constant holds peak force to a few")
    print("  percent, at every timeconst. The shipped pair sits at 12.5.")

    print("\n  NEGATIVE convention (solref = [-stiffness, -damping]), which")
    print("  status.md recommends for system ID: refsafe does NOT apply.")
    print(f"  {'solref':>18s} " + " ".join(f"{v:>9.0e}" for v in (2e-4, 1e-3, 3e-3, 5e-3)))
    for k, b in [(-8e3, -60), (-4e4, -150), (-2e5, -400), (-1e6, -900)]:
        row = " ".join(f"{rest_sink(step, (k, b)):9.4f}" for step in (2e-4, 1e-3, 3e-3, 5e-3))
        print(f"  {f'({k:.0e}, {b:.0f})':>18s} {row}")
    print("  nan = diverged. Stiff pairs blow up instead of being silently softened,")
    print("  so the conversion also removes the timestep guard. (-4e4, -150)")
    print("  reproduces the shipped positive pair's rest sink exactly.")


def study_transfer(policy="general_rl_smooth_stiff", cfg_name="rl_general_smooth.yaml"):
    """Same weights, different rear wheel (and different timestep).

    Actions are divided by the policy's own bounds before they reach the env:
    `pol.action` returns PHYSICAL units and `env.step` wants the normalized
    action, exactly as analysis/chatter.py does it. Feeding physical units
    straight in saturates every channel and the bike topples in under a second,
    which reads convincingly like a broken policy rather than a broken harness.
    """
    import functools
    from pathlib import Path

    import yaml

    import aow_sim.build_model as bm
    from aow_sim.train_general_rl import _eval_episodes, _score, eval_cmds
    from rsa_policies import env_for, load_general

    repo = Path(__file__).resolve().parents[1]
    pol = load_general(policy)
    b = pol.bounds
    scale = np.array([b.steer_rate_max, b.hub_max, b.diff_max])
    cfg = yaml.safe_load((repo / "config" / cfg_name).read_text())
    cfg["env"] = {**cfg["env"],
                  "v_lat_frac": float(getattr(pol, "v_lat_frac", 0.4)),
                  "randomize": {**cfg["env"].get("randomize", {}), "enabled": False}}
    cmds = eval_cmds(cfg["env"]["v_max"])

    def score_one(params, mode=None, n=None):
        orig = bm._add_aow
        if mode:
            bm._add_aow = functools.partial(_add_aow, mode=mode, n_axles=n)
        try:
            env = env_for(pol, params, cfg)
            width = env.action_space.shape[0]
            agg, _ = _eval_episodes(
                env, lambda o: (np.asarray(pol.action(o), float) / scale)[:width], cmds)
        finally:
            bm._add_aow = orig
        return agg

    print(f"\n=== TRANSFER: {policy}, {len(cmds)} eval commands, randomization off ===")
    print(f"{'rear wheel':16s} {'score':>6s} {'survive':>7s} {'track_geo':>9s} "
          f"{'vel_err':>7s} {'head deg':>8s} {'wall s':>6s}")
    for mode, n in [(None, None), ("cones", 8), ("cones", 16), ("capsules", 8),
                    ("spheres2", 8), ("spheres1", 8)]:
        t0 = time.perf_counter()
        agg = score_one(load_params(), mode, n)
        tag = "unpatched" if mode is None else f"{mode}-{n}"
        print(f"{tag:16s} {_score(agg):6.3f} {agg['survive_rate']:7.2f} "
              f"{agg['track_geo']:9.3f} {agg['vel_err']:7.3f} {agg['head_err_deg']:8.1f} "
              f"{time.perf_counter() - t0:6.1f}", flush=True)
    print("  Survival 1.00 on every scheme and at most ~5% of tracking between them,")
    print("  ordered by §CHATTER's ride roughness. The policy is not living off")
    print("  roller detail. `unpatched` and `cones-8` must agree exactly.")

    print(f"\n{'timestep':>10s} {'segments':>8s} {'score':>6s} {'survive':>7s} "
          f"{'track_geo':>9s} {'wall s':>6s}")
    for dt, seg in [(2e-4, 32), (4e-4, 32), (6e-4, 32), (1e-3, 32), (6e-4, 64)]:
        p = load_params()
        p["sim"]["timestep"], p["sim"]["mesh_segments"] = dt, seg
        t0 = time.perf_counter()
        agg = score_one(p)
        star = "  <- shipped" if (dt, seg) == SHIPPED else ""
        print(f"{dt:10.1e} {seg:8d} {_score(agg):6.3f} {agg['survive_rate']:7.2f} "
              f"{agg['track_geo']:9.3f} {time.perf_counter() - t0:6.1f}{star}", flush=True)
    print("  2.5x for a score change smaller than the spread between roller schemes")
    print("  above. Replay only: this does not show that a policy TRAINED at 6e-4")
    print("  transfers back, and it exercises no impacts.")


def study_front():
    """The one finding here that is not about the omni wheel at all.

    The front tyre is a smooth crowned mesh, so ANY contact loss it shows is
    tessellation, full stop. Measured on the whole bike under the LQR, not on
    the rig, because that is where it would be paid.
    """
    from aow_sim.build_model import build_model
    from aow_sim.control.balance import run
    from aow_sim.control.drive import DriveController
    from aow_sim.control.linearize import settle_upright
    from aow_sim.run_balance import _tilted_data

    print("\n=== FRONT TYRE: contact loss that is purely numerical ===")
    print("driving straight at 0.5 m/s, full bike, LQR in the loop")
    print(f"{'segments':>8s} {'facet um':>9s} {'v m/s':>6s} {'front off-gnd':>14s} "
          f"{'front z p-p mm':>15s} {'rear off-gnd':>13s}")
    for seg in (32, 64, 128):
        p = load_params()
        p["sim"]["mesh_segments"] = seg
        m = build_model(p)
        d = _tilted_data(m, settle_upright(m).qpos.copy(), 0.5)
        c = DriveController(p, m)
        c.reset(m, d)
        c.command_line(d)
        c.set_speed(0.5)
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(m.ngeom)]
        front = {i for i, nm in enumerate(names) if nm and "front" in nm}
        rear = {i for i, nm in enumerate(names) if nm and "roller" in nm}
        st = {"fa": 0, "ra": 0, "n": 0, "z": [], "t": 0.0}

        def rec(data, _st=st, _m=m, _f=front, _r=rear):
            _st["t"] += _m.opt.timestep
            if _st["t"] < 2.0:            # skip the speed ramp
                return
            g = set()
            for k in range(data.ncon):
                g.add(data.contact.geom1[k])
                g.add(data.contact.geom2[k])
            _st["fa"] += not (g & _f)
            _st["ra"] += not (g & _r)
            _st["n"] += 1
            _st["z"].append(data.xpos[_m.body("front_wheel").id][2])

        run(m, d, c, 8.0, on_step=rec)
        z = np.array(st["z"])
        facet = 1e6 * 0.050 * (1 - np.cos(np.pi / seg))
        star = "  <- shipped" if seg == SHIPPED[1] else ""
        print(f"{seg:8d} {facet:9.1f} {d.qvel[0]:6.2f} {st['fa'] / max(st['n'], 1) * 100:13.1f}% "
              f"{1e3 * (z.max() - z.min()):15.3f} {st['ra'] / max(st['n'], 1) * 100:12.1f}%{star}")
    print("  The front tyre is round. At mesh_segments: 32 it nevertheless spends")
    print("  ~18% of a 0.5 m/s run off the ground, swinging 0.26 mm -- the same")
    print("  scale as the 0.25 mm facet sagitta. 64 segments removes essentially")
    print("  all of it and costs ~3% of step time on the full bike. The rear column")
    print("  barely moves, which is the control: the omni wheel's ripple is real")
    print("  geometry and does not care about tessellation.")


def study_lean(dt=None):
    """Why the obvious one-contact surrogate (a ball) is not available."""
    print("\n=== LEAN: the contact must walk outboard, or the bike is a different bike ===")
    R = load_params()["omni_wheel"]["outer_radius"]
    phis = np.deg2rad([0, 5, 10, 15, 20])
    print(f"{'lean deg':>8s} {'thin disc':>10s} {'ball':>6s} {'cones-8':>8s} {'torus':>7s}")
    got = {}
    for mode, n in (("cones", 8), ("torus", None)):
        m = build_rig(mode, n_axles=n, timestep=dt, roll=True)
        d = mujoco.MjData(m)
        jid = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i): i for i in range(m.njnt)}
        qa = m.jnt_qposadr
        col = []
        for phi in phis:
            per_phase = []
            for hub in np.linspace(0, 2 * np.pi / (n or 8), 9):
                mujoco.mj_resetData(m, d)
                d.qpos[qa[jid["car_roll"]]] = phi
                d.qpos[qa[jid["hub_spin"]]] = hub
                lo, hi = -0.02, 0.02              # bisect down onto first contact
                for _ in range(60):
                    mid = (lo + hi) / 2
                    d.qpos[qa[jid["car_z"]]] = mid
                    mujoco.mj_forward(m, d)
                    if (min(d.contact.dist[:d.ncon]) if d.ncon else 1.0) < 0:
                        lo = mid
                    else:
                        hi = mid
                d.qpos[qa[jid["car_z"]]] = hi
                mujoco.mj_forward(m, d)
                if d.ncon:
                    per_phase.append(d.contact.pos[:d.ncon, 1].mean()
                                     - d.xpos[m.body("carriage").id][1])
            col.append(np.mean(per_phase) if per_phase else np.nan)
        got[mode] = np.array(col)
    for i, phi in enumerate(phis):
        print(f"{np.degrees(phi):8.0f} {1e3 * R * np.sin(phi):10.2f} {0.0:6.2f} "
              f"{1e3 * got['cones'][i]:8.2f} {1e3 * got['torus'][i]:7.2f}")
    print("  mm the contact patch moves off the wheel centre-plane. The shipped wheel")
    print("  tracks R*sin(phi) -- it is a thin disc, and the bike is an inverted")
    print("  pendulum pivoting at the GROUND. A ball-wheel surrogate holds the contact")
    print("  under the axle, which pivots the pendulum at the AXLE instead: the")
    print("  toppling lever arm drops from h_com to (h_com - R), about 42% less at")
    print("  this geometry. That rules out the single-sphere reduction outright.")
    return phis, got


def study_timestep():
    """How much of `sim.timestep` is margin?"""
    print("\n=== TIMESTEP: the contact statistics are converged well below 2e-4 ===")
    print(f"{'dt (s)':>8s} {'v m/s':>6s} {'z p-p mm':>8s} {'off ground':>10s} "
          f"{'peak Fn/W':>9s} {'sink mm':>8s} {'xRT':>7s}")
    for dt in (5e-5, 1e-4, 2e-4, 4e-4, 6e-4, 1e-3, 2e-3):
        r = drive(build_rig("cones", timestep=dt))
        star = "  <- shipped" if dt == SHIPPED[0] else ""
        print(f"{dt:8.1e} {r['v']:6.2f} {r['z_pp_mm']:8.3f} {r['airborne']*100:9.1f}% "
              f"{r['peak_w']:9.2f} {r['pen_mm']:8.3f} {r['realtime']:7.1f}{star}")
    print("  Everything is flat from 5e-5 to 6e-4 and starts to drift at 1e-3, so the")
    print("  CONTACT is not what bounds the timestep. The LQR's finite-amplitude")
    print("  system ID is: its worst fit R^2 goes 0.975 (2e-4) -> 0.973 (4e-4) ->")
    print("  0.941 (6e-4, at +0.80 m/s), and test_gain_schedule floors it at 0.95.")
    print("  That is why the config sits at 4e-4 and not 6e-4 -- see the note in")
    print("  bike_params.yaml. Nothing in a policy eval can see this bound.")


def plot(chatter_rows, lean, path):
    phis, got = lean
    R = load_params()["omni_wheel"]["outer_radius"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

    # Ride ripple spans two decades (cones-4 is 13.7 mm), so it gets its own
    # log axis -- shown on one linear scale the 8/16/24-roller comparison, which
    # is the actual question, disappears into the baseline.
    tags = [t for t, _ in chatter_rows]
    air = [100 * r["airborne"] for _, r in chatter_rows]
    zpp = [r["z_pp_mm"] for _, r in chatter_rows]
    x = np.arange(len(tags))
    ax1.bar(x, air, 0.55, color="#c0504d", alpha=0.85, label="off the ground [%]")
    ax1.set_ylabel("off the ground [%]", color="#c0504d")
    ax1.tick_params(axis="y", labelcolor="#c0504d")
    ax1b = ax1.twinx()
    ax1b.set_yscale("log")
    ax1b.plot(x, zpp, "o-", color="#1f4e79", label="ride ripple [mm, log]")
    ax1b.set_ylabel("ride-height swing [mm, log]", color="#1f4e79")
    ax1b.tick_params(axis="y", labelcolor="#1f4e79")
    ax1.set_xticks(x)
    ax1.set_xticklabels(tags, rotation=35, ha="right", fontsize=8)
    ax1.set_title("Rolling at ~1 m/s: simplifying the geometry\n"
                  "makes the ride rougher, not smoother")
    ax1.grid(axis="y", alpha=0.3)
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax1b.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper center")

    deg = np.degrees(phis)
    ax2.plot(deg, 1e3 * R * np.sin(phis), "k--", label="thin disc  R sin(phi)")
    ax2.plot(deg, np.zeros_like(deg), ":", color="#888", label="ball wheel  (0)")
    ax2.plot(deg, 1e3 * got["cones"], "o-", color="#4f81bd", label="shipped 8-roller AOW")
    ax2.plot(deg, 1e3 * got["torus"], "s-", color="#9bbb59", label="smooth torus surrogate")
    ax2.set_xlabel("lean [deg]")
    ax2.set_ylabel("contact offset from wheel plane [mm]")
    ax2.set_title("The lean lever arm a surrogate must reproduce")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=140)
    print(f"\nwrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--study", nargs="+",
                    choices=["cost", "chatter", "stiffness", "lean", "timestep",
                             "front", "refsafe", "transfer", "all"],
                    default=["all"])
    ap.add_argument("--dt", type=float, default=None,
                    help="override sim.timestep for every study except `timestep`")
    ap.add_argument("--policy", default="general_rl_smooth_stiff",
                    help="policy for --study transfer")
    ap.add_argument("--rl-config", default="rl_general_smooth.yaml",
                    help="config matching --policy")
    args = ap.parse_args()
    want = set(args.study)
    # `transfer` is opt-in: it needs a trained export and takes minutes, while
    # everything else builds its own models and runs in seconds.
    if "all" in want:
        want = {"cost", "chatter", "stiffness", "lean", "timestep", "front",
                "refsafe"}

    rows = lean = None
    if "cost" in want:
        study_cost(args.dt)
    if "chatter" in want:
        rows = study_chatter(args.dt)
    if "stiffness" in want:
        study_stiffness(args.dt)
    if "lean" in want:
        lean = study_lean(args.dt)
    if "timestep" in want:
        study_timestep()
    if "front" in want:
        study_front()
    if "refsafe" in want:
        study_refsafe()
    if "transfer" in want:
        study_transfer(args.policy, args.rl_config)
    if rows and lean:
        from pathlib import Path
        out = Path(__file__).resolve().parent / "plots" / "contact_surrogates.png"
        out.parent.mkdir(exist_ok=True)
        plot(rows, lean, out)


if __name__ == "__main__":
    main()
