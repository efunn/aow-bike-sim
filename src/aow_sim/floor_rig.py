"""The floor rig in MuJoCo: the full bike held by a yaw -> tilt -> roll -> pitch chain.

`build_spec(..., rig=cfg)` builds the whole bike exactly as it always does --
freejoint and all -- and then APPENDS this chain, whose last body is welded to
the chassis. So any change to the bike rolls into the rig, and every existing
qpos / ctrl / sensor index is untouched: qpos[0:7] is still the chassis, which
is what teleop, the controllers, `settle_upright` and the envs all assume.
The rig's own joints, motors and sensors come after everything else.

  world -> YAW   vertical axis, `yaw_distance_m` ahead of the rear contact
        -> TILT  lateral axis at the head, `tilt_height_m` up. The long arm.
        -> SLIDE fore/aft along the arm (the rolling direction)
        -> ROLL  along the line through both contacts, `roll_height_m` up
        -> PITCH lateral axis through the fake CoG (the bike's own by default)
        == WELD to the chassis

Every rig body sits at the world origin and the joints carry their own
positions, so at qpos 0 the weld's relative pose is the identity. Rig bodies
are massless in intent: 1 mg with a tiny inertia, because MuJoCo refuses a
moving body with none, plus a small ARMATURE on each rig joint so a motor has
something to push against (see RIG_ARMATURE). Joints are frictionless. The
weld is a near-hard equality constraint: 0.2 mm worst case through a fall.
Visual geoms have no mass and no contact.

WITH TILT FREE, THE PITCH MOTOR IS NOT A PURE LOAD TRANSFER. The arm cannot
carry a moment about the tilt axis, so a pitch torque M comes back as a
vertical force M / (yaw_distance - fake CoG ahead) at the fake CoG, on top of
the +-M/wheelbase shift between the contacts. At 0.8 N m and the defaults
that is 1.9 N: rear 5.9 -> 3.0 N, front 4.1 -> 8.9 N (measured 2026-09-25).
A longer arm shrinks the vertical part.

    python -m aow_sim.floor_rig --png runs/floor_rig_smoke.png
    mjpython -m aow_sim.floor_rig --view
    mjpython -m aow_sim.run_drive --teleop --rig
"""
from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np
import yaml

RIG_CFG = Path(__file__).resolve().parents[2] / "config" / "floor_rig.yaml"
TOKEN_MASS = 1e-6             # kg: "massless" as far as MuJoCo allows
TOKEN_INERTIA = 1e-10         # kg m^2
# Rotor-like inertia on each rig JOINT (not on a body: no mass, no weight),
# set per axis to `armature_frac` of the BIKE'S OWN inertia along that axis
# (see `resolve`). Why it has to be that large, measured 2026-09-25:
#   none       a motor torque on a 1e-10 kg m^2 link: NaN at 2 ms.
#   1e-5 flat  motors fine, but every constraint that closes through the rig
#              (the weld, a locked or limited joint, a skid) is softened in
#              proportion to 1/inertia, i.e. ~200x softer than the contacts
#              it fights: the slide ran 286 mm past a 60 mm stop and the weld
#              let the chassis roll 100 deg away from the rig.
#   1 % of the bike's   weld <= 1.4 mm, slide stops hold, across all modes.
# The cost is 1 % extra inertia on each rig axis.
RIG_ARMATURE = 1e-5           # kg m^2 -- fallback when `resolve` has not run
RIG_RGBA = {"fixed": [0.55, 0.55, 0.55, 0.5], "yaw": [0.25, 0.45, 0.85, 0.55],
            "roll": [0.6, 0.35, 0.8, 0.5], "pitch": [0.95, 0.55, 0.15, 0.7]}
JOINTS = ("yaw", "tilt", "slide", "roll", "pitch")


def load_rig_cfg(path: str | Path | None = None) -> dict:
    return yaml.safe_load(Path(path or RIG_CFG).read_text())


def apply_mode(cfg: dict, mode: str | None = None, lock=(), free=()) -> dict:
    """A config with a named `modes:` preset and per-axis overrides applied.

    A preset may set `joints`, `motors` and any top-level key; `lock`/`free`
    are applied last, so a command-line axis always wins over the preset."""
    import copy
    out = copy.deepcopy(cfg)
    if mode:
        presets = cfg.get("modes") or {}
        if mode not in presets:
            raise ValueError(f"floor_rig: no mode {mode!r}; have {sorted(presets)}")
        for k, v in (presets[mode] or {}).items():
            if isinstance(v, dict) and isinstance(out.get(k), dict):
                out[k] = {**out[k], **v}
            else:
                out[k] = v
        out["mode"] = mode
    for j in lock:
        out["joints"][j] = "locked"
    for j in free:
        out["joints"][j] = "free"
    return out


def _bike_inertia(m, d):
    """Mass, CoG and the inertia tensor about the CoG of the chassis subtree."""
    root = m.body("chassis").id
    ids = [i for i in range(m.nbody) if i == root or _descends(m, i, root)]
    mass = float(sum(m.body_mass[i] for i in ids))
    c = d.subtree_com[root].copy()
    inertia = np.zeros((3, 3))
    for i in ids:
        R = d.ximat[i].reshape(3, 3)
        r = d.xipos[i] - c
        inertia += R @ np.diag(m.body_inertia[i]) @ R.T
        inertia += m.body_mass[i] * (r @ r * np.eye(3) - np.outer(r, r))
    return mass, c, inertia


def _descends(m, i, root):
    while i:
        i = m.body_parentid[i]
        if i == root:
            return True
    return False


def resolve(cfg: dict, params: dict) -> dict:
    """Fill in what depends on the bike itself, compiled on its own: the fake
    CoG (`fake_cog: com`) and the per-axis armature (`armature_frac` of the
    bike's inertia along each rig axis)."""
    cfg = dict(cfg)
    from .build_model import build_model
    m = build_model(params)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mass, c, I = _bike_inertia(m, d)
    if cfg.get("fake_cog", "com") == "com":
        cfg["fake_cog"] = [float(c[0]), float(c[2])]
    cx, cz = cfg["fake_cog"]
    yx, th, rh = cfg["yaw_distance_m"], cfg["tilt_height_m"], cfg["roll_height_m"]
    frac = cfg.get("armature_frac", 0.01)
    # The bike's inertia seen by each rig joint, about that joint's own axis.
    cfg["armature"] = {k: float(frac * v) for k, v in {
        "yaw": I[2, 2] + mass * ((c[0] - yx) ** 2 + c[1] ** 2),
        "tilt": I[1, 1] + mass * ((c[0] - yx) ** 2 + (c[2] - th) ** 2),
        "slide": mass,
        "roll": I[0, 0] + mass * (c[1] ** 2 + (c[2] - rh) ** 2),
        "pitch": I[1, 1] + mass * ((c[0] - cx) ** 2 + (c[2] - cz) ** 2),
    }.items()}
    for j in JOINTS:
        if cfg["joints"].get(j) not in ("free", "locked"):
            raise ValueError(f"floor_rig joints.{j}: 'free' or 'locked', "
                             f"got {cfg['joints'].get(j)!r}")
    return cfg


def _visual(body, rgba, **kw):
    body.add_geom(density=0, contype=0, conaffinity=0, group=1, rgba=rgba, **kw)


def _cap(body, a, b, r, rgba):
    _visual(body, rgba, type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[r, 0, 0],
            fromto=np.concatenate([a, b]))


def _link(parent, name, joint, axis, pos, kind=mujoco.mjtJoint.mjJNT_HINGE,
          armature=RIG_ARMATURE, limit=None, tc=2e-4, degree=True):
    body = parent.add_body(name=f"rig_{name}", pos=[0, 0, 0])
    body.explicitinertial = True
    body.mass = TOKEN_MASS
    body.inertia = [TOKEN_INERTIA] * 3
    if joint == "free":
        # Armature is kg m^2 on a hinge and kg on a slide.
        j = body.add_joint(name=f"rig_{name}", type=kind,
                           axis=axis, pos=pos, armature=armature)
        if limit is not None:
            # `limit` is SI (rad / m). A hinge range is read in the spec's
            # angle unit, which is DEGREES unless compiler.degree is off --
            # passing radians made +-55 deg into +-1 deg, and every rig joint
            # was quietly locked (2026-09-25).
            if kind == mujoco.mjtJoint.mjJNT_HINGE and degree:
                limit = [float(np.degrees(v)) for v in limit]
            j.limited = mujoco.mjtLimited.mjLIMITED_TRUE
            j.range = list(limit)
            j.solref_limit = [tc, 1.0]
            j.solimp_limit = [0.999, 0.9999, 0.001, 0.5, 2.0]
    return body


def roll_stop_height(halfspan: float, stop_deg: float, radius: float,
                     axis_h: float) -> float:
    """Height of a skid centre at +-`halfspan` that touches the floor when the
    roll member has rolled `stop_deg` about an axis `axis_h` up."""
    t = np.radians(stop_deg)
    return axis_h + (radius - axis_h + halfspan * np.sin(t)) / np.cos(t)


def add_chain(spec: mujoco.MjSpec, params: dict, cfg: dict) -> None:
    """Append the rig under the worldbody and weld its last body to the chassis.

    Call it LAST in `build_spec`, so nothing of the bike's shifts index.
    `cfg` must already be `resolve`d (fake_cog as numbers)."""
    from .build_model import DYN_CONAFF, DYN_CONTYPE, _contact_friction
    wb = params["bike"]["wheelbase"]
    yx, th = cfg["yaw_distance_m"], cfg["tilt_height_m"]
    rh = cfg["roll_height_m"]
    cx, cz = cfg["fake_cog"]
    sx = wb + cfg["front_support_ahead_m"]      # roll bearing, ahead of the front wheel
    hw = cfg["yoke_half_width_m"]
    J = cfg["joints"]
    xr = sx - 0.03                               # the roll member's front frame
    arm = cfg.get("armature") or {}
    tc = max(2 * spec.option.timestep, 2e-4)     # stiffest stable time constant
    L = lambda name, **kw: dict(armature=arm.get(name, RIG_ARMATURE), tc=tc,
                                degree=bool(spec.compiler.degree), **kw)
    travel = cfg.get("slide_range_m")
    stop = cfg.get("roll_stop")
    # The roll joint is limited a little past the skids' landing angle. The
    # skids stop a FALL at the floor; the limit stops the bike tumbling over a
    # skid once the tilt lets the roll axis lift (seen 2026-09-25: a driven
    # bike rolled to 180 deg over its skid with the tilt free).
    roll_lim = None
    if stop:
        r = np.radians(stop["deg"] + stop.get("limit_margin_deg", 10))
        roll_lim = [-r, r]
    # Hinge ranges in degrees from the config; null = unlimited. The tilt one
    # matters most: unlimited, a driven bike levered the whole arm over the
    # head and ended upside down (2026-09-25).
    rng = lambda k: (None if cfg.get(k) is None
                     else [float(np.radians(v)) for v in cfg[k]])

    # Fixed: a base disc at the yaw axis and the post up to the head.
    w = spec.worldbody
    _visual(w, RIG_RGBA["fixed"], type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[0.089, 0.004, 0], pos=[yx, 0, 0.004])
    _cap(w, [yx, 0, 0.008], [yx, 0, th - 0.012], 0.006, RIG_RGBA["fixed"])

    # YAW: the head.
    yaw = _link(w, "yaw", J["yaw"], [0, 0, 1], [yx, 0, 0],
                **L("yaw", limit=rng("yaw_range_deg")))
    _visual(yaw, RIG_RGBA["yaw"], type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.012, 0.018, 0.012], pos=[yx, 0, th])

    # TILT: the long arm from the head back toward the roll bearing. Slide stop
    # markers ride on it, at the ends of the carriage's travel.
    tilt = _link(yaw, "tilt", J["tilt"], [0, 1, 0], [yx, 0, th],
                 **L("tilt", limit=rng("tilt_range_deg")))
    back = min(travel[0], 0.0) if travel else -0.06
    _cap(tilt, [yx, 0, th], [sx + back - 0.012, 0, th], 0.004, RIG_RGBA["yaw"])
    if travel and J["slide"] == "free":
        for e in travel:
            _visual(tilt, RIG_RGBA["fixed"], type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=[0.002, 0.009, 0.009], pos=[sx + e, 0, th])

    # SLIDE: fore/aft along the arm, carrying the drop post to the roll bearing.
    slide = _link(tilt, "slide", J["slide"], [1, 0, 0], [0, 0, 0],
                  kind=mujoco.mjtJoint.mjJNT_SLIDE, **L("slide", limit=travel))
    _visual(slide, RIG_RGBA["yaw"], type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.012, 0.008, 0.008], pos=[sx, 0, th])
    _cap(slide, [sx, 0, th], [sx, 0, rh + 0.012], 0.004, RIG_RGBA["yaw"])

    # ROLL: the bearing on the contact line, then the roll member -- a frame up
    # to CoG height and back to the pitch stubs either side of the bike, so
    # nothing of it reaches the floor before the skids do.
    roll = _link(slide, "roll", J["roll"], [1, 0, 0], [0, 0, rh],
                 **L("roll", limit=roll_lim))
    _visual(roll, RIG_RGBA["roll"], type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[0.008, 0.012, 0], pos=[sx - 0.012, 0, rh + 0.008],
            quat=[np.sqrt(0.5), 0, np.sqrt(0.5), 0])
    _cap(roll, [sx - 0.024, 0, rh + 0.008], [xr, 0, rh + 0.008], 0.003, RIG_RGBA["roll"])
    _cap(roll, [xr, 0, rh + 0.008], [xr, 0, cz], 0.003, RIG_RGBA["roll"])
    _cap(roll, [xr, -hw, cz], [xr, hw, cz], 0.003, RIG_RGBA["roll"])
    for s in (-1, 1):
        _cap(roll, [xr, s * hw, cz], [cx, s * hw, cz], 0.003, RIG_RGBA["roll"])

    # ROLL STOPS: skids on outriggers from the crossbar ends, placed so they
    # meet the floor at `roll_stop.deg`. REAL contact (floor only), so a fall
    # ends on the skid instead of the bike rolling on through the floor.
    #
    # THE COLLIDER RIDES ON THE CHASSIS, not on the roll member it is drawn on.
    # MuJoCo's contact softness scales with the inertia behind the contact, and
    # behind a 1 mg rig link there is almost none: a skid there touched the
    # floor and the bike rolled straight through to 180 deg (measured
    # 2026-09-25). On the chassis it has the bike behind it. The only
    # difference is that the skid now pitches with the bike -- a few degrees.
    if stop:
        r_s, span = stop["radius_m"], stop["halfspan_m"]
        hz = roll_stop_height(span, stop["deg"], r_s, rh)
        chassis = spec.body("chassis")
        c0 = np.asarray(chassis.pos, float)          # chassis frame at qpos0
        for s, tag in ((-1, "right"), (1, "left")):
            _cap(roll, [xr, s * hw, cz], [xr, s * span, hz], 0.003, RIG_RGBA["roll"])
            chassis.add_geom(name=f"rig_roll_stop_{tag}",
                             type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[r_s, 0, 0],
                             pos=np.array([xr, s * span, hz]) - c0, density=0,
                             contype=DYN_CONTYPE, conaffinity=DYN_CONAFF,
                             condim=3, friction=_contact_friction(params["sim"]),
                             rgba=[0.35, 0.2, 0.5, 0.9])

    # PITCH: stubs on the fake CoG axis. Welded to the chassis below.
    pitch = _link(roll, "pitch", J["pitch"], [0, 1, 0], [cx, 0, cz],
                  **L("pitch", limit=rng("pitch_range_deg")))
    for s in (-1, 1):
        _visual(pitch, RIG_RGBA["pitch"], type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                size=[0.007, 0.01, 0], pos=[cx, s * (hw - 0.01), cz],
                quat=[np.sqrt(0.5), np.sqrt(0.5), 0, 0])
    _visual(pitch, RIG_RGBA["pitch"], type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.006, 0, 0], pos=[cx, 0, cz])

    # Placeholder XC330s on the instrumented axes: torque motors, no servo model.
    tau = params["servos"]["xc330_t181"]["stall_torque"]
    for j in ("roll", "pitch"):
        if cfg.get("actuators", {}).get(j) and J[j] == "free":
            a = spec.add_actuator(name=f"rig_{j}_motor", target=f"rig_{j}",
                                  trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.set_to_motor()
            a.ctrllimited = True
            a.ctrlrange = [-tau, tau]
    for j in JOINTS:
        if J[j] == "free":
            spec.add_sensor(name=f"rig_{j}_pos", type=mujoco.mjtSensor.mjSENS_JOINTPOS,
                            objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=f"rig_{j}")
    # The weld: relpose left at zero, so MuJoCo takes it from qpos0, where the
    # rig and the chassis are both in their reference pose. Stiff: a time
    # constant of two steps, the fastest MuJoCo keeps stable.
    spec.add_equality(name="rig_weld", type=mujoco.mjtEq.mjEQ_WELD,
                      objtype=mujoco.mjtObj.mjOBJ_BODY,
                      name1="rig_pitch", name2="chassis",
                      solref=[tc, 1.0], solimp=[0.999, 0.9999, 0.001, 0.5, 2.0])


# ---------------------------------------------------------------------------
# Runtime helpers: teleop and scripts use these instead of writing qpos by hand.

def has_rig(model) -> bool:
    return any(model.body(i).name == "rig_pitch" for i in range(model.nbody))


def qpos_for(model, yaw: float = 0.0, base=None) -> np.ndarray:
    """A full qpos with every rig joint at its reference except `yaw`, and the
    chassis freejoint put exactly where the rig then holds it (so the weld
    starts with zero error). The bike's other joints are taken from `base`
    (default `model.qpos0`)."""
    d = mujoco.MjData(model)
    d.qpos[:] = model.qpos0 if base is None else base
    for j in JOINTS:
        name = f"rig_{j}"
        if any(model.joint(i).name == name for i in range(model.njnt)):
            d.qpos[model.joint(name).qposadr[0]] = yaw if j == "yaw" else 0.0
    mujoco.mj_kinematics(model, d)
    b = model.body("chassis").id
    pitch = d.body("rig_pitch")
    R = pitch.xmat.reshape(3, 3)
    pos = pitch.xpos + R @ model.body_pos[b]
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, pitch.xquat, model.body_quat[b])
    adr = model.jnt_qposadr[model.body_jntadr[b]]
    d.qpos[adr:adr + 3] = pos
    d.qpos[adr + 3:adr + 7] = quat
    return d.qpos.copy()


def place(model, data, yaw: float = 0.0) -> None:
    """Respawn in the rig: rig at its reference with the arm swung to `yaw`,
    chassis matched to it, everything at rest."""
    data.qpos[:] = qpos_for(model, yaw, base=data.qpos)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def motor_step(model, data, cfg: dict) -> None:
    """Hold the rig motors at `cfg['motors']` torques [N m]. Call AFTER the
    bike's controller, which writes the whole ctrl vector every step."""
    for j, tau in (cfg.get("motors") or {}).items():
        name = f"rig_{j}_motor"
        if any(model.actuator(i).name == name for i in range(model.nu)):
            aid = model.actuator(name).id
            lo, hi = model.actuator_ctrlrange[aid]
            data.ctrl[aid] = float(np.clip(tau, lo, hi))


def describe_rig(cfg: dict) -> str:
    """One line for a banner: where the axes are and which joints are free."""
    J = cfg["joints"]
    cx, cz = cfg["fake_cog"]
    return (f"floor rig: yaw axis {cfg['yaw_distance_m']*1000:.0f} mm ahead, tilt "
            f"{cfg['tilt_height_m']*1000:.0f} mm up, roll {cfg['roll_height_m']*1000:.0f} mm "
            f"up, pitch at fake CoG ({cx*1000:.0f} ahead, {cz*1000:.0f} up); "
            + " ".join(f"{j} {J[j]}" for j in JOINTS)
            + (f"; slide travel {cfg['slide_range_m']} m" if cfg.get("slide_range_m")
               and J["slide"] == "free" else "")
            + (f"; roll stops at {cfg['roll_stop']['deg']} deg" if cfg.get("roll_stop") else "")
            + (f"; motors {cfg['motors']} N m" if any((cfg.get("motors") or {}).values()) else "")
            + (f"  [mode {cfg['mode']}]" if cfg.get("mode") else "")
            + "  [massless rig, frictionless joints]")


def build_rig_model(params: dict | None = None, cfg: dict | str | Path | None = None,
                    **bike_kw) -> mujoco.MjModel:
    from .build_model import build_model, load_params
    p = params or load_params()
    c = cfg if isinstance(cfg, dict) else load_rig_cfg(cfg)
    return build_model(p, rig=resolve(c, p), **bike_kw)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help=f"default {RIG_CFG}")
    ap.add_argument("--png", default=None, help="render one frame here")
    ap.add_argument("--view", action="store_true", help="interactive (needs mjpython)")
    ap.add_argument("--settle", type=float, default=0.5,
                    help="seconds to simulate before the render")
    args = ap.parse_args()
    m = build_rig_model(cfg=args.config)
    d = mujoco.MjData(m)
    names = [m.joint(j).name for j in range(m.njnt) if m.joint(j).name.startswith("rig_")]
    print(f"rig joints: {names}; nq {m.nq}, nu {m.nu}, bike mass "
          f"{m.body_subtreemass[m.body('chassis').id]*1000:.0f} g")
    for _ in range(int(args.settle / m.opt.timestep)):
        mujoco.mj_step(m, d)
    print("after settle: " + ", ".join(
        f"{n} {d.joint(n).qpos[0]*1000:+.1f} mm" if m.jnt_type[m.joint(n).id]
        == mujoco.mjtJoint.mjJNT_SLIDE else f"{n} {np.degrees(d.joint(n).qpos[0]):+.2f} deg"
        for n in names))
    if args.png:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0.25, 0, 0.08]
        cam.distance, cam.azimuth, cam.elevation = 0.95, 135, -25
        m.vis.global_.offwidth, m.vis.global_.offheight = 1080, 720
        with mujoco.Renderer(m, 720, 1080) as r:
            opt = mujoco.MjvOption()
            opt.geomgroup[:] = 1
            r.update_scene(d, cam, opt)
            import PIL.Image
            PIL.Image.fromarray(r.render()).save(args.png)
        print(f"wrote {args.png}")
    if args.view:
        from mujoco import viewer
        viewer.launch(m, d)


if __name__ == "__main__":
    main()
