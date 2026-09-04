"""Parametric MJCF builder: config/bike_params.yaml -> MuJoCo model.

Two variants from the same parameters:
  full    — the whole bike (chassis freejoint, steering, front wheel, AOW).
  testbed — the omni wheel + drive input shafts on a stand welded to the world,
            mirroring the physical system-ID rig.

Modeling scheme (see docs/plans/mujoco-modeling-decisions.md):
  - 8 roller axles, each with two truncated-cone convex meshes; axle spin is
    coupled to the ring-vs-hub relative angle by joint equality constraints.
  - The toy gearbox is pure kinematics: fixed tendons + tendon equality map the
    two input shafts to hub and ring rotation via a 2x2 mixing matrix.
  - Only {rollers, tire, training wheels} <-> floor make contact:
    floor contype=1 conaffinity=2, dynamic contact geoms contype=2 conaffinity=1
    (so dynamic geoms never collide with each other), everything else 0/0.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np
import yaml

from . import geometry
# Re-exported: params loading is MuJoCo-free so the Pi can use it (params.py).
from .params import DEFAULT_PARAMS, _normalize, load_params  # noqa: F401


FLOOR_CONTYPE, FLOOR_CONAFF = 1, 2
FLOOR_GRID_M = 0.25   # default metres per checker square; override with
                      #   sim.floor_grid_m (record.py exposes --grid)
DYN_CONTYPE, DYN_CONAFF = 2, 1
# Hockey extras (build_model(..., hockey=True)). The base 2-bit scheme lets
# dynamic geoms touch only the floor; the ball needs to touch floor + stick +
# wheels (a wheel strike is physical, so it can be detected and penalized) while
# the bike's own dynamic geoms still never self-collide. Two more bits do it:
#   ball  contype 4, conaff 1|2|8 = 11  -> sees floor(1), dyn wheels(2), stick(8)
#   stick contype 8, conaff 1|4  =  5   -> sees floor(1), ball(4)
BALL_CONTYPE, BALL_CONAFF = 4, FLOOR_CONTYPE | DYN_CONTYPE | 8   # = 11
STICK_CONTYPE, STICK_CONAFF = 8, FLOOR_CONTYPE | BALL_CONTYPE     # = 5

INTEGRATORS = {
    "euler": mujoco.mjtIntegrator.mjINT_EULER,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "rk4": mujoco.mjtIntegrator.mjINT_RK4,
}
CONES = {
    "pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL,
    "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC,
}


def _quat_z_to(v) -> np.ndarray:
    """Quaternion (w,x,y,z) rotating local +Z onto direction v."""
    v = np.asarray(v, dtype=float)
    v = v / np.linalg.norm(v)
    z = np.array([0.0, 0.0, 1.0])
    c = float(z @ v)
    if c > 1 - 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if c < -1 + 1e-12:
        return np.array([0.0, 1.0, 0.0, 0.0])
    axis = np.cross(z, v)
    axis /= np.linalg.norm(axis)
    half = np.arccos(c) / 2
    return np.concatenate([[np.cos(half)], np.sin(half) * axis])


_Y_AXIS_QUAT = _quat_z_to([0, 1, 0])  # for cylinders/wheels whose axis is bike-lateral


def _contact_friction(sim: dict) -> list[float]:
    return [sim["friction_sliding"], sim["friction_torsional"], 0.0001]


def _add_aow(spec: mujoco.MjSpec, parent, p: dict) -> None:
    """Omni wheel assembly + input shafts + couplings + drive actuators.

    `parent` is the body carrying the rear axle (chassis or testbed stand);
    the rear axle is along +Y through the parent's frame origin.
    """
    ow, dt, sim = p["omni_wheel"], p["drivetrain"], p["sim"]
    roller = ow["roller"]

    cone = spec.add_mesh(name="roller_cone")
    cone.uservert = geometry.truncated_cone_vertices(
        roller["big_diameter"] / 2,
        roller["small_diameter"] / 2,
        roller["length"],
        sim["mesh_segments"],
    ).flatten()

    hub = parent.add_body(name="aow_hub")
    hub.add_joint(
        name="hub_spin",
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=[0, 1, 0],
        damping=dt["hub_joint_damping"],
        frictionloss=dt["hub_joint_frictionloss"],
    )
    hub.add_geom(
        name="hub_body",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[ow["hub"]["body_radius"], ow["hub"]["body_width"] / 2, 0],
        quat=_Y_AXIS_QUAT,
        mass=ow["hub"]["mass"],
        contype=0,
        conaffinity=0,
        rgba=[0.25, 0.25, 0.3, 1],
    )

    ring = hub.add_body(name="roller_ring")
    ring.add_joint(  # angle is RELATIVE to the hub -> simple roller couplings
        name="ring_spin", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0]
    )
    ring.add_geom(
        name="ring_body",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[ow["ring"]["body_radius"], ow["ring"]["body_width"] / 2, 0],
        quat=_Y_AXIS_QUAT,
        mass=ow["ring"]["mass"],
        contype=0,
        conaffinity=0,
        rgba=[0.8, 0.5, 0.1, 1],
    )

    n = ow["n_axles"]
    cant = np.deg2rad(ow["axle_cant_deg"])
    s_center = roller["pair_gap"] / 2 + roller["length"] / 2
    big_inward = roller.get("big_end_inward", True)
    for i in range(n):
        theta = 2 * np.pi * i / n
        radial = np.array([np.cos(theta), 0.0, np.sin(theta)])
        tangent = np.array([-np.sin(theta), 0.0, np.cos(theta)])
        if cant:
            tangent = tangent * np.cos(cant) - np.array([0.0, 1.0, 0.0]) * np.sin(cant)
        axle = hub.add_body(name=f"roller_axle_{i}", pos=ow["axle_mount_radius"] * radial)
        axle.add_joint(
            name=f"roller_spin_{i}",
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=tangent,
            damping=dt["roller_joint_damping"],
            frictionloss=dt["roller_joint_frictionloss"],
        )
        for side in (-1, 1):
            # mesh +Z runs big end -> small end; big ends face the axle center
            z_dir = side * tangent if big_inward else -side * tangent
            axle.add_geom(
                name=f"roller_{i}_{'a' if side < 0 else 'b'}",
                type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname="roller_cone",
                pos=side * s_center * tangent,
                quat=_quat_z_to(z_dir),
                mass=roller["pair_mass"] / 2,
                contype=DYN_CONTYPE,
                conaffinity=DYN_CONAFF,
                condim=sim["condim"],
                friction=_contact_friction(sim),
                rgba=[0.15, 0.15, 0.15, 1],
            )

    # Roller couplings: axle spin = k_roller * ring relative angle (rigid gearing).
    for i in range(n):
        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_JOINT
        eq.name1, eq.name2 = f"roller_spin_{i}", "ring_spin"
        eq.data[:5] = [0.0, dt["k_roller"], 0.0, 0.0, 0.0]
        eq.solref = [0.005, 1.0]

    # Input shafts = the two ring-gear shafts: coaxial with the rear axle, one
    # per side (XC430s attach here via belts). Lateral offset is a placeholder
    # until the mount/pulley design is final.
    y_off = ow["width"] / 2 + dt["input_pulley_offset"]
    for tag, y in (("a", y_off), ("b", -y_off)):
        shaft = parent.add_body(name=f"input_{tag}", pos=[0, y, 0])
        shaft.add_joint(
            name=f"input_{tag}_spin",
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=[0, 1, 0],
            armature=dt["input_armature"],
        )
        shaft.add_geom(
            name=f"input_{tag}_pulley",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[0.008, 0.004, 0],
            quat=_Y_AXIS_QUAT,
            mass=0.005,
            contype=0,
            conaffinity=0,
            rgba=[0.6, 0.6, 0.65, 1],
        )

    # Gearbox kinematics: hub = mha*a + mhb*b ; ring_abs (= hub + ring_rel) = mra*a + mrb*b.
    mixes = {
        "gear_hub": [("hub_spin", 1.0),
                     ("input_a_spin", -dt["mix_hub_a"]),
                     ("input_b_spin", -dt["mix_hub_b"])],
        "gear_ring": [("ring_spin", 1.0), ("hub_spin", 1.0),
                      ("input_a_spin", -dt["mix_ring_a"]),
                      ("input_b_spin", -dt["mix_ring_b"])],
    }
    for name, wraps in mixes.items():
        ten = spec.add_tendon(name=name)
        for joint, coef in wraps:
            ten.wrap_joint(joint, coef)
        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_TENDON
        eq.name1 = name
        eq.solref = [0.005, 1.0]

    # Drive actuators: XC430 velocity mode through the belt (speed x belt_ratio,
    # torque / belt_ratio, both seen at the input shaft).
    servo = p["servos"]["xc430_w150"]
    belt = dt["belt_ratio"]
    max_speed = servo["no_load_rpm"] * 2 * np.pi / 60 * belt
    max_torque = servo["stall_torque"] / belt
    kv = p["actuators"]["drive_kv"]
    ki = p["actuators"].get("drive_ki", 0.0)
    for tag in ("a", "b"):
        act = spec.add_actuator(name=f"drive_{tag}")
        if ki:
            # Velocity PI. A PI velocity loop IS a P position loop whose
            # setpoint ramps at the commanded speed, and MuJoCo expresses that
            # natively: dyntype=integrator makes act = integral(ctrl) dt, so
            #
            #   force = ki*(act - theta) - kv*w = ki*integral(ctrl - w) dt - kv*w
            #
            # `ctrl` still means commanded input-shaft velocity, so nu is
            # unchanged and no caller has to learn a new command; na goes
            # 0 -> 2, which is why every hand-rolled reset in this repo has to
            # call reset_actuator_state (see its docstring for the cost of
            # forgetting). What this form DROPS relative to real firmware is
            # the kv*w_cmd proportional feedforward -- the conservative
            # direction, since the whole defect being fixed is a sim that is
            # more capable than the hardware. servo-protocol.md section 4
            # measures the feedforward stiffness that would restore it.
            act.dyntype = mujoco.mjtDyn.mjDYN_INTEGRATOR
            act.dynprm[0] = 1.0
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.gainprm[0] = ki
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            act.biasprm[:3] = [0.0, -ki, -kv]
        else:
            # ki = 0 is the shipped P-only plant, bit-exact: the PI branch with
            # ki = 0 would drop the kv*ctrl term entirely, not reduce to this.
            act.set_to_velocity(kv=kv)
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.target = f"input_{tag}_spin"
        act.ctrlrange = [-max_speed, max_speed]
        act.forcerange = [-max_torque, max_torque]

    for tag in ("a", "b"):
        for stype, suffix in (
            (mujoco.mjtSensor.mjSENS_JOINTPOS, "pos"),
            (mujoco.mjtSensor.mjSENS_JOINTVEL, "vel"),
        ):
            s = spec.add_sensor(name=f"input_{tag}_{suffix}")
            s.type = stype
            s.objtype = mujoco.mjtObj.mjOBJ_JOINT
            s.objname = f"input_{tag}_spin"


def _add_hockey(spec: mujoco.MjSpec, chassis, p: dict) -> None:
    """Add the ball-shot extras (see docs/plans/ball-shot-move.md): a translucent
    'hockey stick' panel on each side of the chassis and a free road-hockey ball.

    Stick — thin ABS-like box from ~center-x to the rear, partially over the rear
    wheel, with ground clearance so leans don't scrape it (roll is limited in the
    RL env rather than checked here). Ball — a world free body placed at rest; its
    start pose is written at env reset, not baked in. Collision classes give
    ball<->{floor,stick,wheels} and stick<->{floor,ball} while leaving the bike's
    own dynamic geoms non-self-colliding (see the *_CONTYPE/_CONAFF constants)."""
    hk, sim = p["hockey"], p["sim"]
    # THE STICK IS BUILT ONLY IF THE CASE SIDES ARE NOT. They are the same part
    # in the real bike -- 4 mm ABS panels down each side -- and the stick was
    # always a stand-in for a case that did not exist yet. Where `case_*` is
    # configured, those panels ARE the striking surface (contype 2, which the
    # ball's conaffinity already sees), and a second translucent slab at a
    # different station would only get in the way of the ball.
    #
    # Falling back keeps bike_params.yaml, moves/ball_rl.npz and
    # tests/test_ball_rl.py working exactly as before, since that file has no
    # `case_*` keys.
    if "case_thickness" not in p["righting"]["wings"]:
        st = hk["stick"]
        for side, tag in ((1, "left"), (-1, "right")):
            px, py, pz = st["pos"]
            chassis.add_geom(
                name=f"stick_{tag}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[st["length"] / 2, st["thickness"] / 2, st["height"] / 2],
                pos=[px, side * py, pz],
                mass=st["mass"],
                contype=STICK_CONTYPE,
                conaffinity=STICK_CONAFF,
                condim=sim["condim"],
                friction=_contact_friction(sim),
                rgba=[0.8, 0.5, 0.1, 0.35],   # translucent, for sim visibility
            )

    ball = hk["ball"]
    body = spec.worldbody.add_body(name="ball", pos=[ball["start"][0],
                                                     ball["start"][1], ball["radius"]])
    body.add_freejoint()
    body.add_geom(
        name="ball",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[ball["radius"], 0, 0],
        mass=ball["mass"],
        contype=BALL_CONTYPE,
        conaffinity=BALL_CONAFF,
        condim=sim["condim"],
        friction=[ball["friction_sliding"], sim["friction_torsional"], 0.0001],
        rgba=[0.95, 0.45, 0.1, 1],
    )


def _add_righting(spec: mujoco.MjSpec, chassis, p: dict, arm: bool = True) -> None:
    """Self-righting study extras (build_model(..., righting=True)); see
    docs/plans/self-righting.md and the `righting` block in bike_params.yaml.

    Two side rails that decide what the bike comes to rest on, and one arm on
    an extra XC330 that swings through +-180 deg so a single servo can push
    from either side. Both are optional: omit the sub-block to leave it out.

    `arm=False` keeps the rails and drops the arm, which is how the wing
    variant gets the bumper geometry without the arm's mass — the two
    mechanisms are alternatives and are never built into the same model.

    Note the CALLER is responsible for making the chassis lumps collidable —
    that is done in build_spec while the lumps are being added, because a fall
    that sinks through the servos rests on nothing."""
    rg, sim = p["righting"], p["sim"]

    if "roof" in rg:
        # A capsule along +X: round in the ROLL plane, so an inverted bike
        # rolls off it instead of balancing on it, with hemispherical ends
        # giving the fore/aft doming for free. The flat-topped AHRS is what
        # currently defines the top of the bike and what touches at 180 deg;
        # this has to sit proud of it to take over that job.
        r = rg["roof"]
        chassis.add_geom(
            name="roof",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=[r["radius"], 0, 0],
            fromto=[r["x_start"], 0.0, r["height"],
                    r["x_end"], 0.0, r["height"]],
            mass=r["mass"],
            contype=DYN_CONTYPE,
            conaffinity=DYN_CONAFF,
            condim=sim["condim"],
            friction=_contact_friction(sim),
            rgba=[0.35, 0.6, 0.85, 0.55],
        )

    if "bumper" in rg:
        b = rg["bumper"]
        for side, tag in ((1, "left"), (-1, "right")):
            chassis.add_geom(
                name=f"bumper_{tag}",
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[b["radius"], 0, 0],
                fromto=[b["x_start"], side * b["half_span"], b["height"],
                        b["x_end"], side * b["half_span"], b["height"]],
                mass=b["mass"],
                contype=DYN_CONTYPE,
                conaffinity=DYN_CONAFF,
                condim=sim["condim"],
                friction=_contact_friction(sim),
                rgba=[0.9, 0.75, 0.2, 1],
            )

    if not arm or "arm" not in rg:
        return
    a = rg["arm"]
    pivot = np.asarray(a["pivot"], dtype=float)
    # The servo body itself rides on the chassis; only its mass matters here.
    chassis.add_geom(
        name="servo_righting",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=np.array(p["servos"]["xc330_t181"]["box_size"]) / 2,
        pos=pivot,
        mass=a["servo_mass"],
        contype=0,
        conaffinity=0,
        rgba=[0.1, 0.1, 0.1, 1],
    )
    arm = chassis.add_body(name="righting_arm", pos=pivot)
    arm.add_joint(  # fore/aft axis: the arm swings in the roll plane
        name="righting_joint", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0]
    )
    # Stowed at 0 = straight up, so the arm is out of the way of the ground and
    # of the hockey stick panels; +-1 rad reaches down to either side.
    arm.add_geom(
        name="righting_arm",
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        size=[a["radius"], 0, 0],
        fromto=[0, 0, 0, 0, 0, a["length"]],
        mass=a["mass"] * 0.5,
        contype=DYN_CONTYPE,
        conaffinity=DYN_CONAFF,
        condim=sim["condim"],
        friction=_contact_friction(sim),
        rgba=[0.85, 0.2, 0.2, 1],
    )
    arm.add_geom(
        name="righting_foot",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[a["foot_radius"], 0, 0],
        pos=[0, 0, a["length"]],
        mass=a["mass"] * 0.5,
        contype=DYN_CONTYPE,
        conaffinity=DYN_CONAFF,
        condim=sim["condim"],
        friction=_contact_friction(sim),
        rgba=[0.85, 0.2, 0.2, 1],
    )
    # Torque at the ARM = servo stall x the reduction; the study reports it
    # back divided by the same ratio, so a sweep can be read against the
    # servo's datasheet number whatever the gearing.
    tau = p["servos"]["xc330_t181"]["stall_torque"] * a["gear_ratio"]
    act = spec.add_actuator(name="righting")
    act.set_to_position(kp=a["servo_kp"] * a["gear_ratio"],
                        kv=a["servo_kv"] * a["gear_ratio"])
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = "righting_joint"
    act.forcerange = [-tau, tau]

    for stype, suffix in (
        (mujoco.mjtSensor.mjSENS_JOINTPOS, "pos"),
        (mujoco.mjtSensor.mjSENS_JOINTVEL, "vel"),
    ):
        s = spec.add_sensor(name=f"righting_{suffix}")
        s.type = stype
        s.objtype = mujoco.mjtObj.mjOBJ_JOINT
        s.objname = "righting_joint"


def wing_fit(p: dict) -> dict:
    """Gear-train geometry for the wing pair, and what it costs to fit.

    Topology: the two WING GEARS MESH EACH OTHER DIRECTLY, and the XC330 drives
    only one of them through a pinion. Two consequences, and both are the point
    of doing it this way:

      * The mesh IS the reversal. Two meshed gears counter-rotate, so the
        mirror-symmetric deployment falls out of the gear train itself and the
        separate idler the sketch worried about disappears.
      * The reduction no longer touches the stance. The discs are on pivots
        2*half_span apart and are equal, so each one is exactly

            r_disc = half_span                    (INDEPENDENT of the ratio)

        and the ratio only sets the pinion that drives one of them:

            r_pinion = r_disc / gear_ratio

    That is a real decoupling. With a central pinion meshing both wings,
    r_disc grew with the ratio, so buying torque widened the bike; here the
    reduction is free of the envelope and only has to stay manufacturable.

    Two hard limits remain:

      pinion  3D-printed teeth have a minimum workable pitch radius; below it
              the reduction is not manufacturable at this disc size.
      floor   the disc is centred on the pivot, so it cannot be larger than
              the pivot's height above the floor without grounding out. With
              a direct mesh this is a constraint on the PIVOT SPACING, not on
              the ratio -- it no longer moves when the reduction does.

    Everything here is derived, so a swept `gear_ratio` re-reports it for free.
    """
    w = p["righting"]["wings"]
    half_span, ratio = w["pivot"][1], w["gear_ratio"]
    r_disc = half_span                      # direct wing-to-wing mesh
    r_pinion = r_disc / ratio
    ride_h = p["omni_wheel"]["outer_radius"] + w["pivot"][2]
    # Only the pinion still puts a ceiling on the ratio; the floor limit is now
    # a property of the pivot alone, so it is pass/fail rather than a ceiling.
    by_pinion = r_disc / w["min_pinion_radius"]
    # The crank has to carry the leg CLEAR OF THE DRIVEN DISC: the leg lands on
    # the floor, not on the gear it is bolted to. With a direct mesh the disc
    # is half_span, so this is a constraint on the ENVELOPE --
    #     bike_width >= 2 * (half_span + r_disc) = 4 * half_span
    # -- and, unlike before, it does not move with the reduction.
    crank_reach = w["crank_length"] * np.sin(np.deg2rad(w["crank_deg"]))
    return {"disc_radius": r_disc, "pinion_radius": r_pinion,
            "pivot_height": ride_h,
            "crank_reach": crank_reach,
            "min_bike_width": 2.0 * (half_span + r_disc),
            "leg_stands_on_gear": crank_reach < r_disc,
            "grounds_out": r_disc > ride_h,
            "pinion_too_small": r_pinion < w["min_pinion_radius"],
            "max_ratio": by_pinion,
            "max_ratio_by": "pinion"}


LINKAGE_CFG = Path(__file__).resolve().parents[2] / "config" / "wing_linkage_locking.yaml"


def derive_linkage_roof(p: dict, cfg: dict) -> dict:
    """Re-derive the roof ridge from the LINKAGE's stow envelope.

    `params.derive_righting()` sizes the roof from the GEARED wing's envelope:
    radius = the stow half-span, axis at the wing TIP height, so the tips sit
    tangent to the circle and the crest ends up a full radius proud of them.
    That last part is what makes an inverted bike roll off instead of perching.

    The linkage's stowed wing is a different shape entirely -- a full-length
    panel running ground_clearance..(bike_height - half_span) -- and
    it was inheriting the geared roof unchanged. Its panel tops then landed
    LEVEL with the crest, so inverted the bike stood on two panel edges plus
    the ridge: a stable three-point stance. Measured, that lost 1 of 8 falls
    outright, resting at 180 deg on `roof, wing_left, wing_right`.

    Same rule, applied to the right envelope: axis at the panel top, radius the
    stow half-span. The crest then clears the panel by a radius again.
    """
    b = cfg["bike"]
    r_rear_mm = p["omni_wheel"]["outer_radius"] * 1000.0
    half_span_mm = b["bike_width"] / 2.0
    # Identical rule to params.derive_righting() for the geared pair: radius is
    # the stow half-span, axis a radius below the crest. `bike_height` is the
    # CREST in both files, so the panel tops out at the axis and sits tangent.
    return {
        "radius": half_span_mm / 1000.0,
        "height": (b["bike_height"] - half_span_mm - r_rear_mm) / 1000.0,
    }


def _add_case_sides(chassis, p: dict, plate_pos, plate_half, sim: dict) -> None:
    """The two FIXED plates per side that, with the stowed wing, close the wall.

    `plate_pos` / `plate_half` are the stowed wing plate's centre and
    half-extents in CHASSIS coordinates -- the case is defined off the wing
    rather than from its own numbers, so the three pieces cannot drift out of
    plane with each other.

    Skipped entirely when the `case_*` keys are absent, which is what keeps
    bike_params.yaml building unchanged.
    """
    w = p["righting"]["wings"]
    if "case_thickness" not in w:
        return
    th = w["case_thickness"]
    clear = w["case_clearance"] - p["omni_wheel"]["outer_radius"]   # -> axle frame
    rear_x = w["case_rear_x"]
    gap = w.get("case_gap", 0.0)
    m_each = w.get("case_mass", 0.015) / 2.0
    px, py, pz = plate_pos
    hx, _, hz = plate_half
    wing_lo, wing_front = pz - hz, px - hx

    for side in (1, -1):
        y = side * abs(py)
        # SKIRT: everything below the stowed wing, running the FULL length of
        # the case -- from `case_rear_x` forward to the wing's trailing edge --
        # not just the wing's own station. Below the wing there is nothing to
        # clear, so there is no reason to stop where the wing starts.
        skirt_front = px + hx
        if wing_lo > clear and skirt_front > rear_x:
            chassis.add_geom(
                name=f"case_skirt_{'left' if side > 0 else 'right'}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[(skirt_front - rear_x) / 2, th / 2,
                      (wing_lo - gap - clear) / 2],
                pos=[(skirt_front + rear_x) / 2, y, (wing_lo - gap + clear) / 2],
                mass=m_each, contype=DYN_CONTYPE, conaffinity=DYN_CONAFF,
                condim=sim["condim"], friction=_contact_friction(sim),
                # Translucent: these panels sit directly between the camera and
                # the rear wheel in the `wheel` view, and the wheel is the whole
                # reason that view exists.
                rgba=[0.30, 0.32, 0.38, 0.30])
        # UPPER: the wing's silhouette continued rearward.
        if wing_front > rear_x:
            chassis.add_geom(
                name=f"case_upper_{'left' if side > 0 else 'right'}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[(wing_front - gap - rear_x) / 2, th / 2, hz],
                pos=[(wing_front - gap + rear_x) / 2, y, pz],
                mass=m_each, contype=DYN_CONTYPE, conaffinity=DYN_CONAFF,
                condim=sim["condim"], friction=_contact_friction(sim),
                # Translucent: these panels sit directly between the camera and
                # the rear wheel in the `wheel` view, and the wheel is the whole
                # reason that view exists.
                rgba=[0.30, 0.32, 0.38, 0.30])


def _add_wing_linkage(spec: mujoco.MjSpec, chassis, p: dict, cfg: dict) -> None:
    """The four-bar wing mechanism (docs/plans/wing-linkage-design-and-optimization.md).

    An alternative to the gear train in `_add_wings`. Same job -- one servo,
    two mirrored-ish wings -- but the ratio VARIES through the stroke instead
    of being fixed, which is what lets the deployed pose sit at the crank's
    input-side dead point and hold the bike with no servo current.

    CLOSED LOOP, so it cannot be a pure tree. Each side is
        crank (one body, BOTH arms, hinged at the servo)
          -> coupler (hinged at that arm's tip, free end carries a site)
        wing (hinged at its own pivot on the chassis, carries a site)
    and an mjEQ_CONNECT between the two sites closes the four-bar. The crank is
    a SINGLE body with two arms because that is what it physically is: both
    arms keyed to one shaft, `angle_between_first_links` apart.

    FRAMES. The linkage config is millimetres measured from the FLOOR and the
    centreline; everything here is metres from the REAR AXLE. `wheel_radius`
    is the only conversion, and getting it wrong puts the whole mechanism a
    wheel-radius off the ground rather than failing loudly.
    """
    b, m_, st = cfg["bike"], cfg["mechanism"], cfg["stroke"]
    sim = p["sim"]
    w_ref = p["righting"]["wings"]          # mass/radius/servo reused from there
    r_rear = p["omni_wheel"]["outer_radius"]
    px = w_ref["pivot"][0]                  # fore/aft station, from the geared study

    def to_bike(y_mm, z_mm):
        """(y, z) mm above the floor -> (y, z) m relative to the rear axle."""
        return np.array([y_mm / 1000.0, z_mm / 1000.0 - r_rear])

    servo = to_bike(0.0, b["wheel_radius"] + m_["servo_offset"])
    pivot_z = to_bike(0.0, b["wheel_radius"])[1]
    half_span = b["bike_width"] / 2000.0
    pivot_y = m_["wing_pivot_offset"] / 1000.0
    stow_out = half_span - pivot_y
    wing_lo = to_bike(0.0, b["ground_clearance"])[1]
    # `bike_height` is the roof CREST, so the stowed panel tops out a roof
    # radius below it -- at the roof axis, where its tips sit tangent to the
    # rolling surface. Must match analysis/wing_linkage.py::Linkage exactly:
    # this read `bike_height` directly for a while, which built a MuJoCo wing
    # a half-span longer than the one the 2D study was optimising.
    wing_hi = to_bike(0.0, b["bike_height"] - b["bike_width"] / 2.0)[1]

    crank = chassis.add_body(name="wing_crank", pos=[px, servo[0], servo[1]])
    crank.add_joint(name="wing_crank_joint", type=mujoco.mjtJoint.mjJNT_HINGE,
                    axis=[1, 0, 0])
    tips, attach0 = {}, {}
    for side, tag in ((-1, "right"), (1, "left")):
        ang = m_["first_link_angle_deg"] + (m_["angle_between_first_links"]
                                            if tag == "left" else 0.0)
        L = m_["wing_first_link_length"][tag] / 1000.0
        tip = L * np.array([np.cos(np.deg2rad(ang)), np.sin(np.deg2rad(ang))])
        tips[tag] = tip
        crank.add_geom(
            name=f"wing_crank_{tag}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.004, 0, 0],
            fromto=[0, 0, 0, 0, tip[0], tip[1]],
            mass=w_ref["mass"] * 0.15, contype=0, conaffinity=0,
            rgba=[0.5, 0.2, 0.6, 1])
        off = np.asarray(m_["wing_attach_offset"], float) / 1000.0
        attach0[tag] = (np.array([side * pivot_y, pivot_z])
                        + np.array([side * off[0], off[1]]))

    for side, tag in ((-1, "right"), (1, "left")):
        # Coupler: hinged at its crank tip, reaching to the attach point. Its
        # length is whatever closes the loop at STOW, exactly as the 2D study
        # derives it -- so the model and the study cannot disagree about it.
        tip_world = servo + tips[tag]
        vec = attach0[tag] - tip_world
        coup = crank.add_body(
            name=f"wing_coupler_{tag}", pos=[0.0, tips[tag][0], tips[tag][1]])
        coup.add_joint(name=f"wing_coupler_{tag}_joint",
                       type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0])
        coup.add_geom(
            name=f"wing_coupler_{tag}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.0035, 0, 0],
            fromto=[0, 0, 0, 0, vec[0], vec[1]],
            mass=w_ref["mass"] * 0.2, contype=0, conaffinity=0,
            rgba=[0.2, 0.6, 0.3, 1])
        coup.add_site(name=f"wing_coupler_{tag}_end",
                      pos=[0.0, vec[0], vec[1]], size=[0.003, 0, 0])

        wing = chassis.add_body(name=f"wing_{tag}",
                                pos=[px, side * pivot_y, pivot_z])
        wing.add_joint(name=f"wing_{tag}_joint",
                       type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0])
        # The wing is a flat PLATE alongside the bike: the bike lies ON it and
        # the mechanism levers it out, so the whole face is the contact.
        #
        # It was a capsule, which is a LINE contact — the bike could pitch
        # freely about it because nothing resisted rotation along the bike's
        # own axis. A plate with real fore/aft extent is what actually stops
        # that, and it is also what gets built. `panel_length_x` and
        # `panel_offset_x` are optional: without them the plate falls back to
        # the capsule's diameter, so existing configs build as before.
        lo = np.array([side * stow_out, wing_lo - pivot_z])
        hi = np.array([side * stow_out, wing_hi - pivot_z])
        th = w_ref.get("panel_thickness", 2 * w_ref["radius"])
        plate_x = w_ref.get("panel_length_x", 2 * w_ref["radius"])
        wing.add_geom(
            name=f"wing_{tag}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[plate_x / 2, th / 2, float(hi[1] - lo[1]) / 2],
            pos=[w_ref.get("panel_offset_x", 0.0),
                 float(lo[0]) - side * th / 2,
                 float(lo[1] + hi[1]) / 2],
            mass=w_ref["mass"], contype=DYN_CONTYPE, conaffinity=DYN_CONAFF,
            condim=sim["condim"], friction=_contact_friction(sim),
            rgba=[0.85, 0.2, 0.2, 1] if side < 0 else [0.2, 0.4, 0.8, 1])
        if side > 0:      # once, off the left plate; the right is its mirror
            # px is the linkage STATION; panel_offset_x is relative to it.
            # The wing geom carries the offset because it lives in the wing
            # body, but the case plates hang off the CHASSIS and need the
            # absolute station -- getting this wrong put the skirt 145 mm
            # behind the wing and silently dropped the upper panel.
            _add_case_sides(chassis, p,
                            (px + w_ref.get("panel_offset_x", 0.0),
                             side * pivot_y + float(lo[0]) - side * th / 2,
                             float(lo[1] + hi[1]) / 2),
                            (plate_x / 2, th / 2, float(hi[1] - lo[1]) / 2), sim)
        att_local = attach0[tag] - np.array([side * pivot_y, pivot_z])
        wing.add_site(name=f"wing_{tag}_attach",
                      pos=[0.0, att_local[0], att_local[1]], size=[0.003, 0, 0])

        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_CONNECT
        eq.objtype = mujoco.mjtObj.mjOBJ_SITE
        eq.name1 = f"wing_coupler_{tag}_end"
        eq.name2 = f"wing_{tag}_attach"
        # STIFF. A loose loop closure shows up as the coupler visibly detaching
        # from the wing and as torque that goes somewhere other than the load;
        # the default let the joint drift ~0.5 mm in a smoke test.
        eq.solref = [0.002, 1.0]
        eq.solimp = [0.99, 0.9999, 1e-4, 0.5, 2.0]

    # One actuator, on the shared crank shaft.
    xc330 = p["servos"]["xc330_t181"]
    act = spec.add_actuator(name="wings")
    act.set_to_position(kp=w_ref["servo_kp"], kv=w_ref["servo_kv"])
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = "wing_crank_joint"
    act.forcerange = [-xc330["stall_torque"], xc330["stall_torque"]]
    for stype, suffix in ((mujoco.mjtSensor.mjSENS_JOINTPOS, "pos"),
                          (mujoco.mjtSensor.mjSENS_JOINTVEL, "vel")):
        sen = spec.add_sensor(name=f"wings_{suffix}")
        sen.type = stype
        sen.objtype = mujoco.mjtObj.mjOBJ_JOINT
        sen.objname = "wing_crank_joint"


def _add_wings(spec: mujoco.MjSpec, chassis, p: dict) -> None:
    """The wing-pair righting mechanism (build_model(..., wings=True)); the
    alternative to `_add_righting`'s single arm. See docs/plans/self-righting.md
    and the `righting.wings` block in bike_params.yaml.

    A mirrored wing per side, both driven by ONE servo through a gear train
    (meshed gears at the two pivots with a reversal on one side). The gear
    train is deliberately NOT modelled; all it contributes to the dynamics is

      inversion — a joint equality theta_left = mirror * theta_right, which IS
                  the reversal gear, and
      reduction — `gear_ratio`, folded into the actuator's forcerange and gains
                  exactly as the arm and the steering do.

    So the mechanism is one actuator on `wing_right_joint` and a constraint,
    and the servo's own datasheet torque is recovered by dividing the reported
    actuator force by the same ratio.

    Geometry. Each wing is a rigid dogleg: a short crank off the pivot, then
    the long leg to a foot. The two wings are MIRROR IMAGES through the XZ
    plane, which takes both halves — the geoms are mirrored in y here AND the
    joint angles carry opposite signs via the equality. Hinge axis is body +X
    and positive rotates toward -Y (the bike's right), the same convention the
    arm uses, so the right wing deploys at +d and the left at -d and the pair
    swings outboard and down together.

    `crank_deg` is measured from body +Z and cranks OUTBOARD, so the stowed leg
    lies alongside the bike rather than converging on the centreline. That is a
    packaging requirement the simulation cannot enforce and will not complain
    about — see the collision note below — because an inboard crank puts the
    stowed leg straight through the drive servos.

    The gear discs are drawn but weightless and non-colliding: they exist so a
    reduction can be judged on FIT, since the XC330 pinion has a minimum
    printable size and the disc it drives grows with the ratio. See
    `wing_fit` for the constraints that come out of that.

    Joint ranges are the mechanism's hard stops: [0, +deploy] on the right,
    [-deploy, 0] on the left, so the pair cannot be commanded backwards into
    the chassis. `deploy_deg` is set well past floor contact on purpose — if a
    stop rather than the floor takes the load, the actuator force reads low and
    a torque sweep would silently report the stroke as cheap.

    KNOWN MODELLING LIMIT: the wings carry the same DYN_CONTYPE/DYN_CONAFF as
    every other dynamic geom, so they collide with the floor and never with the
    bike's own geoms. Stowed-wing interference with the chassis is a geometric
    check, not something this simulation will catch.

    As with `_add_righting`, the CALLER makes the chassis lumps collidable."""
    w, sim = p["righting"]["wings"], p["sim"]
    px, py, pz = w["pivot"]
    crank = np.deg2rad(w["crank_deg"])
    deploy, stow = w["deploy_deg"], w["stow_deg"]

    # Servo + gear train ride on the chassis; only their mass matters here.
    # ON THE CENTRELINE, and lifted clear of where the two wing discs mesh:
    # they touch at y = 0, so a servo sitting at the pivot height would be
    # inside the mesh. Raising it by the disc radius puts its shaft above the
    # meshing point, where a pinion can reach one disc. The actual routing
    # (which disc it drives, and the fore/aft offset that lets the pinion clear
    # the other one) is a design detail -- what the model needs right is the
    # MASS on the centreline and at a plausible height, not the tooth geometry.
    fit = wing_fit(p)
    servo_z = pz + fit["disc_radius"]
    chassis.add_geom(
        name="servo_wings",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=np.array(p["servos"]["xc330_t181"]["box_size"]) / 2,
        pos=[px, 0.0, servo_z],
        mass=w["servo_mass"] + w["gearbox_mass"],
        contype=0,
        conaffinity=0,
        rgba=[0.1, 0.1, 0.1, 1],
    )
    # The driving pinion, at the servo. Drawn only, like the discs -- see
    # `wing_fit` for why its size is what limits the reduction. It drives ONE
    # disc; the other is carried by the direct wing-to-wing mesh, which is also
    # what reverses it.
    chassis.add_geom(
        name="wing_pinion",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[fit["pinion_radius"], w["gear_width"] / 2, 0],
        quat=_quat_z_to([1, 0, 0]),
        pos=[px, 0.0, servo_z],
        mass=0.0,
        contype=0,
        conaffinity=0,
        rgba=[0.4, 0.4, 0.45, 0.5],
    )

    # Split the per-wing mass: the foot is the heavy end, the two capsules
    # share the rest in proportion to their length.
    span = w["crank_length"] + w["length"]
    m_foot = w["mass"] * 0.4
    m_span = w["mass"] * 0.6

    for side, tag in ((1, "left"), (-1, "right")):
        # Outboard is +side * Y: +Y for the left wing, -Y for the right. The
        # crank leans the stowed leg AWAY from the centreline so it parks
        # alongside the bike instead of through the drive servos.
        elbow = np.array([0.0,
                          side * np.sin(crank) * w["crank_length"],
                          np.cos(crank) * w["crank_length"]])
        foot = elbow + np.array([0.0, 0.0, w["length"]])
        wing = chassis.add_body(name=f"wing_{tag}", pos=[px, side * py, pz])
        lo, hi = sorted((stow, -side * deploy))
        wing.add_joint(
            name=f"wing_{tag}_joint",
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=[1, 0, 0],
            # DEGREES: spec.compiler.degree defaults to 1, so a joint range is
            # read in degrees and converted at compile time. Passing radians
            # here does not error — it silently builds a +-2.4 deg stop.
            range=[lo, hi],
            limited=mujoco.mjtLimited.mjLIMITED_TRUE,
        )
        for name, a, b, mass in (
            (f"wing_{tag}_crank", np.zeros(3), elbow, m_span * w["crank_length"] / span),
            (f"wing_{tag}_leg", elbow, foot, m_span * w["length"] / span),
        ):
            wing.add_geom(
                name=name,
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[w["radius"], 0, 0],
                fromto=np.concatenate([a, b]),
                mass=mass,
                contype=DYN_CONTYPE,
                conaffinity=DYN_CONAFF,
                condim=sim["condim"],
                friction=_contact_friction(sim),
                rgba=[0.85, 0.2, 0.2, 1],
            )
        # The foot. A bare SPHERE is a point contact, and a point contact is
        # what makes the torque trace spiky: the whole ground reaction acts at
        # one fixed radius from the pivot, so the moment arm is whatever the
        # geometry happens to give at that instant. Peak/mean over a stroke is
        # ~3.4, which means the servo is sized almost entirely by transients
        # and barely at all by the work it does.
        #
        # `rocker_deg` > 0 replaces it with a SKID: an arc of spheres sweeping
        # back from the tip about the wing pivot, at a radius growing linearly
        # from foot_radius to rocker_radius. The ground contact then MIGRATES
        # along the skid as the wing turns, the way a rocking chair rolls
        # instead of pivoting on a point, and the moment arm changes smoothly
        # rather than in steps. It is also the shape that is easy to actually
        # make -- a curved edge on a printed wing, not a separate part.
        rock = w.get("rocker_deg", 0.0)
        if rock <= 0.0:
            wing.add_geom(
                name=f"wing_{tag}_foot",
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[w["foot_radius"], 0, 0],
                pos=foot,
                mass=m_foot,
                contype=DYN_CONTYPE,
                conaffinity=DYN_CONAFF,
                condim=sim["condim"],
                friction=_contact_friction(sim),
                rgba=[0.85, 0.2, 0.2, 1],
            )
        else:
            n = int(w.get("rocker_segments", 6))
            r_tip = float(w["foot_radius"])
            r_heel = float(w.get("rocker_radius", r_tip))
            reach = float(np.linalg.norm(foot))          # pivot -> tip
            phi0 = np.arctan2(foot[1], foot[2])          # tip bearing, in-plane
            for i in range(n):
                fr = i / max(n - 1, 1)
                a = phi0 - side * np.deg2rad(rock) * fr  # sweep BACK from the tip
                rad = reach - (r_heel - r_tip) * fr      # skid rises toward the heel
                wing.add_geom(
                    name=f"wing_{tag}_foot" if i == 0 else f"wing_{tag}_skid{i}",
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[r_tip + (r_heel - r_tip) * fr, 0, 0],
                    pos=[0.0, rad * np.sin(a), rad * np.cos(a)],
                    mass=m_foot / n,
                    contype=DYN_CONTYPE,
                    conaffinity=DYN_CONAFF,
                    condim=sim["condim"],
                    friction=_contact_friction(sim),
                    rgba=[0.85, 0.2, 0.2, 1],
                )
        # The driven gear, fixed to the wing and centred on its pivot. Drawn
        # only: weightless (the mass is already in `mass`/`gearbox_mass`) and
        # non-colliding, because what it is here for is judging whether the
        # thing FITS -- against the floor, against its opposite number, and
        # against the chassis -- not what it weighs.
        wing.add_geom(
            name=f"wing_{tag}_gear",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[fit["disc_radius"], w["gear_width"] / 2, 0],
            quat=_quat_z_to([1, 0, 0]),        # disc axis = the hinge axis
            mass=0.0,
            contype=0,
            conaffinity=0,
            rgba=[0.9, 0.3, 0.3, 0.35],
        )

    # THE REVERSAL GEAR: theta_left = mirror * theta_right. Same joint-equality
    # pattern as the roller couplings above, and the only thing that makes one
    # servo drive two wings.
    eq = spec.add_equality()
    eq.type = mujoco.mjtEq.mjEQ_JOINT
    eq.name1, eq.name2 = "wing_left_joint", "wing_right_joint"
    eq.data[:5] = [0.0, w["mirror"], 0.0, 0.0, 0.0]
    eq.solref = [0.005, 1.0]

    # One actuator for the pair, on the driven (right) wing. Torque at the
    # MECHANISM = servo stall x the reduction; the study divides by the same
    # ratio so a swept number reads against the XC330 datasheet directly.
    tau = p["servos"]["xc330_t181"]["stall_torque"] * w["gear_ratio"]
    act = spec.add_actuator(name="wings")
    act.set_to_position(kp=w["servo_kp"] * w["gear_ratio"],
                        kv=w["servo_kv"] * w["gear_ratio"])
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = "wing_right_joint"
    act.forcerange = [-tau, tau]
    # No ctrlrange: the stroke is 135 deg at the wing, which is 405 deg at the
    # servo through the 3:1 — this needs EXTENDED POSITION (multi-turn) mode on
    # the real XC330, same as the steering actuator.

    for stype, suffix in (
        (mujoco.mjtSensor.mjSENS_JOINTPOS, "pos"),
        (mujoco.mjtSensor.mjSENS_JOINTVEL, "vel"),
    ):
        s = spec.add_sensor(name=f"wings_{suffix}")
        s.type = stype
        s.objtype = mujoco.mjtObj.mjOBJ_JOINT
        s.objname = "wing_right_joint"


SWING_LINKAGE_CFG = (Path(__file__).resolve().parents[2]
                     / "config" / "swing_linkage.yaml")


def _add_swing_linkage(spec: mujoco.MjSpec, chassis, p: dict, cfg: dict) -> None:
    """Co-rotating FOUR-BAR wing pair (build_model(..., swing_linkage=True)).

    The driveable counterpart to analysis/swing_linkage.py, which until now was
    a planar study with no model behind it -- the optimised geometries could be
    looked at but not driven, trained on, or collided.

    Same closed-loop construction as `_add_wing_linkage`, and the difference is
    the one that defines this mechanism:

        _add_wing_linkage   two crank arms `angle_between_first_links` apart,
                            with per-side link lengths, driving a MIRRORED pair
                            -- both wings deploy outward together.
        this                two crank arms `angle_between_cranks` apart on the
                            same shaft with ONE set of link lengths shared by
                            both sides, so the pair CO-ROTATES: one wing swings
                            down and out while the other comes up and in.

    Sharing the lengths is what makes the rest pose symmetric BY CONSTRUCTION
    rather than by tuning. A co-rotating pair rests in the middle of its range,
    and an asymmetric rest pose is a standing roll bias the balance controller
    trims out forever -- so the shared lengths make that unrepresentable.

    CLOSED LOOP, so it cannot be a pure tree, exactly as the mirrored linkage:
        crank (one body, BOTH arms, hinged at the servo)
          -> coupler (hinged at that arm's tip, free end carries a site)
        wing (hinged at its own pivot on the chassis, carries a site)
    and an mjEQ_CONNECT between the two sites closes each four-bar.

    FRAMES. The config is millimetres from the FLOOR and the centreline, as the
    2D study uses; everything here is metres from the REAR AXLE. `wheel_radius`
    is the only conversion and getting it wrong puts the whole mechanism a
    wheel radius off the ground rather than failing loudly.
    """
    b, m_, st = cfg["bike"], cfg["mechanism"], cfg["stroke"]
    sim = p["sim"]
    w_ref = p["righting"]["wings"]           # mass/servo reused from there
    r_rear = p["omni_wheel"]["outer_radius"]
    px = w_ref["pivot"][0]                   # fore/aft station

    def to_bike(y_mm, z_mm):
        return np.array([y_mm / 1000.0, z_mm / 1000.0 - r_rear])

    servo = to_bike(0.0, b["wheel_radius"] + m_["servo_offset"])
    pivot_y = m_["wing_pivot_x"] / 1000.0
    pivot_z = to_bike(0.0, b["wheel_radius"] + m_["wing_pivot_z"])[1]
    crank_len = m_["crank_length"] / 1000.0
    coupler_len = m_["coupler_length"] / 1000.0
    rocker_len = m_["rocker_length"] / 1000.0
    between = m_["angle_between_cranks"]
    rest = np.deg2rad(m_["wing_angle_from_rocker"])
    norm_off = m_["wing_norm_offset"] / 1000.0
    z_max = m_["wing_z_max"] / 1000.0

    # Panel bottom sits at ground clearance, DERIVED rather than configured --
    # the same convention the 2D study uses, so the two cannot disagree about
    # where the panel starts.
    ground_clear = to_bike(0.0, b["ground_clearance"])[1]

    def arm_dir(side, travel_deg):
        base = 90.0 - side * between / 2.0
        a = np.deg2rad(base + travel_deg)
        return np.array([np.cos(a), np.sin(a)])

    def rest_joint(side):
        """Rocker joint at rest: the circle-circle intersection FURTHER from
        the centreline. The inboard branch folds the rocker through the
        chassis, and a four-bar that starts on the wrong branch cannot be
        driven onto the right one -- it is assembled differently, not merely
        posed differently."""
        pivot = np.array([side * pivot_y, pivot_z])
        c = servo + crank_len * arm_dir(side, 0.0)
        d = c - pivot
        L = float(np.linalg.norm(d))
        a_ = (rocker_len ** 2 - coupler_len ** 2 + L ** 2) / (2 * L)
        h_ = np.sqrt(max(rocker_len ** 2 - a_ ** 2, 0.0))
        base_pt = pivot + a_ * d / L
        perp = np.array([-d[1], d[0]]) / L
        return pivot, max([base_pt + h_ * perp, base_pt - h_ * perp],
                          key=lambda q: abs(q[0]))

    # `wing_angle_mode: vertical_rest` DERIVES the panel bearing so the panels
    # stand vertical at rest, exactly as `wing_z_min` below is derived so the
    # panel bottom sits at ground clearance. Reproduced here rather than read
    # from the file for the same reason: analysis/swing_linkage.py resolves it
    # too, and a hand-written config that has not been through that study's
    # `--save` carries the UNRESOLVED `wing_angle_from_rocker` -- so a builder
    # that only read the number would model a splayed mechanism while the study
    # modelled a vertical one, from one file, silently.
    mode = m_.get("wing_angle_mode", "fixed")
    if mode in ("vertical_rest", "flat_deploy"):
        # `flat_deploy` pins the panel FLAT at the deployed toggle instead of
        # upright at rest. The study resolves both into the same key, and a
        # saved config carries the number -- this is the fallback for a
        # hand-written config that has not been through `--save`, and for
        # `vertical_rest` it is exact. For `flat_deploy` the toggle solve lives
        # in the study; a config using it must be saved from there.
        if mode == "vertical_rest":
            pv, j0 = rest_joint(-1)
            r0 = j0 - pv
            rest = np.arctan2(r0[1], r0[0]) - np.pi / 2.0
    elif mode != "fixed":
        raise ValueError(f"wing_angle_mode: expected 'fixed', 'vertical_rest' "
                         f"or 'flat_deploy', got {mode!r}")

    crank = chassis.add_body(name="swing_crank", pos=[px, servo[0], servo[1]])
    crank.add_joint(name="swing_crank_joint", type=mujoco.mjtJoint.mjJNT_HINGE,
                    axis=[1, 0, 0])
    tips = {}
    for side, tag in ((-1, "right"), (1, "left")):
        tip = crank_len * arm_dir(side, 0.0)
        tips[tag] = tip
        crank.add_geom(
            name=f"swing_crank_{tag}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.004, 0, 0],
            fromto=[0, 0, 0, 0, tip[0], tip[1]],
            mass=w_ref["mass"] * 0.15, contype=0, conaffinity=0,
            rgba=[0.5, 0.2, 0.6, 1])

    for side, tag in ((-1, "right"), (1, "left")):
        pivot, joint0 = rest_joint(side)
        c = servo + tips[tag]

        coup = crank.add_body(name=f"swing_coupler_{tag}",
                              pos=[0.0, tips[tag][0], tips[tag][1]])
        # DAMPED. The loop's passive joints need dissipation or the mechanism
        # rings: with the crank held at a commanded angle the plate was seen
        # wandering 56 -> 222 mm while the crank itself sat within 0.07 deg of
        # its target. mjEQ_CONNECT is a 3-DOF point weld and every hinge here is
        # about +X, so one constraint row is redundant and the solver has slack
        # to chatter in; damping is what makes the pose settle instead. It is
        # also physically real -- these are bearings, not ideal pins.
        coup.add_joint(name=f"swing_coupler_{tag}_joint",
                       type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0],
                       damping=[0.002, 0, 0])
        vec = joint0 - c
        coup.add_geom(
            name=f"swing_coupler_{tag}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.0035, 0, 0],
            fromto=[0, 0, 0, 0, vec[0], vec[1]],
            mass=w_ref["mass"] * 0.2, contype=0, conaffinity=0,
            rgba=[0.2, 0.6, 0.3, 1])
        coup.add_site(name=f"swing_coupler_{tag}_end",
                      pos=[0.0, vec[0], vec[1]], size=[0.003, 0, 0])

        wing = chassis.add_body(name=f"swing_wing_{tag}",
                                pos=[px, side * pivot_y, pivot_z])
        wing.add_joint(name=f"swing_wing_{tag}_joint",
                       type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0],
                       damping=[0.002, 0, 0])
        # The rocker IS the wing: the panel is rigid with it, hung at
        # `wing_angle_from_rocker` off its bearing and offset normal to it.
        r = joint0 - pivot
        rdir = r / float(np.linalg.norm(r))
        wa = np.arctan2(rdir[1], rdir[0]) + side * rest
        w_hat = np.array([np.cos(wa), np.sin(wa)])
        n_hat = np.array([w_hat[1], -w_hat[0]])
        origin = (joint0 - pivot) + side * norm_off * n_hat
        z_min = float((ground_clear - pivot_z - origin[1]) / w_hat[1])
        lo_p, hi_p = origin + z_min * w_hat, origin + z_max * w_hat
        mid = 0.5 * (lo_p + hi_p)
        length = float(np.linalg.norm(hi_p - lo_p))
        wing.add_geom(
            name=f"swing_wing_{tag}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[w_ref.get("panel_length_x", 0.030), 0.003, length / 2],
            pos=[0.0, mid[0], mid[1]],
            quat=_quat_z_to([0.0, float(w_hat[0]), float(w_hat[1])]),
            mass=w_ref["mass"], contype=DYN_CONTYPE, conaffinity=DYN_CONAFF,
            condim=sim["condim"], friction=_contact_friction(sim),
            rgba=[0.85, 0.2, 0.2, 1] if side < 0 else [0.2, 0.4, 0.8, 1])
        att = joint0 - pivot
        wing.add_site(name=f"swing_wing_{tag}_attach",
                      pos=[0.0, att[0], att[1]], size=[0.003, 0, 0])

        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_CONNECT
        eq.objtype = mujoco.mjtObj.mjOBJ_SITE
        eq.name1 = f"swing_coupler_{tag}_end"
        eq.name2 = f"swing_wing_{tag}_attach"
        # STIFF, same reasoning as the mirrored linkage: a loose closure shows
        # up as the coupler visibly detaching and as torque going somewhere
        # other than the load.
        eq.solref = [0.002, 1.0]
        eq.solimp = [0.99, 0.9999, 1e-4, 0.5, 2.0]

    xc330 = p["servos"][cfg.get("servo", "xc330_t181")] if isinstance(
        cfg.get("servo", "xc330_t181"), str) else p["servos"]["xc330_t181"]
    act = spec.add_actuator(name="swing")
    act.set_to_position(kp=w_ref["servo_kp"], kv=w_ref["servo_kv"])
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = "swing_crank_joint"
    act.forcerange = [-xc330["stall_torque"], xc330["stall_torque"]]
    trav = np.deg2rad(float(st["crank_travel_deg"]))
    # SIGNED range, unlike the mirrored linkage's one-way travel: -t deploys
    # one side and +t the other, and clamping it to [0, t] would silently make
    # the mechanism one-sided.
    act.ctrlrange = [-trav, trav]
    act.ctrllimited = True
    for stype, suffix in ((mujoco.mjtSensor.mjSENS_JOINTPOS, "pos"),
                          (mujoco.mjtSensor.mjSENS_JOINTVEL, "vel")):
        sen = spec.add_sensor(name=f"swing_{suffix}")
        sen.type = stype
        sen.objtype = mujoco.mjtObj.mjOBJ_JOINT
        sen.objname = "swing_crank_joint"


SWING_CFG = Path(__file__).resolve().parents[2] / "config" / "swing_wings.yaml"


def _add_swing_wings(spec: mujoco.MjSpec, chassis, p: dict, cfg: dict) -> None:
    """Co-rotating wing pair (build_model(..., swing=True)); the THIRD righting
    mechanism, after `arm` and the mirrored `wings`. See config/swing_wings.yaml
    for why the coupling sign is the whole difference.

    ONE SIGN separates this from `_add_wings`: the joint equality is
    theta_left = +1 * theta_right rather than -1. A direct gear mesh reverses,
    which is what makes the stock pair mirrored and side-agnostic; an idler or
    a belt does not, which makes this pair co-rotate and lets it put one wing
    down while the other tucks up. Everything else here follows from that:

      * THE RANGE IS TWO-SIDED. `_add_wings` gives each joint a one-sided range
        (left [-deploy, 0], right [0, deploy]) because the mirrored stroke only
        ever goes one way. Co-rotation needs +-deploy on BOTH, and getting this
        wrong is silent: the two one-sided ranges intersect only at zero, so a
        co-rotating pair built with them simply cannot move.
      * THE REST POSE IS A SYMMETRIC V, at +-`rest_deg` from vertical, so the
        centre position carries no lateral CoM offset. A co-rotating pair that
        rested off-centre would be a standing roll bias the balance controller
        trims out forever.
      * THERE IS JOINT DAMPING. `_add_wings` has none, which is fine for a
        quasi-static torque reading but not here, where the whole point of the
        lower reduction is tip SPEED. Without b = tau_stall/w_noload referred
        through the ratio, a position actuator drives the wing arbitrarily fast
        and every strike number is fiction.
    """
    servo = p["servos"][cfg["servo"]]
    px, py, pz = cfg["pivot"]
    rest = np.deg2rad(float(cfg["rest_deg"]))
    L = float(cfg["length"])
    ratio = float(cfg["gear_ratio"])
    deploy = float(cfg["deploy_deg"])
    plate, sim = cfg["plate"], p["sim"]

    tau = servo["stall_torque"] * ratio
    w_wing = servo["no_load_rpm"] * 2 * np.pi / 60.0 / ratio

    for side, tag in ((1, "left"), (-1, "right")):
        wing = spec_body = chassis.add_body(
            name=f"swing_{tag}", pos=[px, side * py, pz])
        wing.add_joint(
            name=f"swing_{tag}_joint",
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=[1, 0, 0],
            range=[-deploy, deploy],      # DEGREES: spec.compiler.degree is on
            limited=mujoco.mjtLimited.mjLIMITED_TRUE,
            damping=[tau / w_wing, 0, 0],
        )
        # Leg direction at rest: splayed OUTBOARD by `rest_deg` from +Z. The
        # plate's long axis runs along it and its broad face leads the swing,
        # which is the face that meets the ball.
        d = np.array([0.0, side * np.sin(rest), np.cos(rest)])
        wing.add_geom(
            name=f"swing_{tag}_plate",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[plate["x_length"] / 2, plate["thickness"] / 2, L / 2],
            pos=(L / 2) * d,
            # Rotate local +Z onto the leg direction; the plate is then thin
            # along the swing tangent and broad across it.
            quat=_quat_z_to(d),
            mass=float(cfg["mass"]),
            contype=DYN_CONTYPE,
            conaffinity=DYN_CONAFF,
            condim=sim["condim"],
            friction=_contact_friction(sim),
            rgba=[0.9, 0.55, 0.1, 1],
        )

    # THE COUPLING, and the one line that defines this mechanism.
    eq = spec.add_equality()
    eq.type = mujoco.mjtEq.mjEQ_JOINT
    eq.name1, eq.name2 = "swing_left_joint", "swing_right_joint"
    eq.data[:5] = [0.0, 1.0, 0.0, 0.0, 0.0]     # +1 = co-rotate; -1 would mirror
    eq.solref = [0.005, 1.0]

    # DEFEAT THE PARENT-CHILD CONTACT FILTER. MuJoCo excludes contacts between
    # a body and its parent, and both wings hang off the chassis -- so without
    # explicit pairs the rising wing sweeps clean through the frame, the drive
    # servos and the battery, and the stroke looks fine while being physically
    # impossible. That is exactly how the first version of this mechanism ran
    # at 45 deg past its real limit. The pairs make the model REFUSE the pose
    # rather than leaving `deploy_deg` as a number someone has to trust.
    for tag in ("left", "right"):
        for other in ("chassis_box", "servo_drive_left", "servo_drive_right"):
            if not any(g.name == other for g in chassis.geoms):
                continue
            pair = spec.add_pair()
            pair.geomname1 = f"swing_{tag}_plate"
            pair.geomname2 = other
            pair.condim = sim["condim"]

    act = spec.add_actuator(name="swing")
    act.set_to_position(kp=cfg["servo_kp"] * ratio, kv=cfg["servo_kv"] * ratio)
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = "swing_right_joint"
    act.forcerange = [-tau, tau]
    act.ctrlrange = [-np.deg2rad(deploy), np.deg2rad(deploy)]
    act.ctrllimited = True

    for stype, suffix in ((mujoco.mjtSensor.mjSENS_JOINTPOS, "pos"),
                          (mujoco.mjtSensor.mjSENS_JOINTVEL, "vel")):
        s = spec.add_sensor(name=f"swing_{suffix}")
        s.type = stype
        s.objtype = mujoco.mjtObj.mjOBJ_JOINT
        s.objname = "swing_right_joint"


def swing_poses(cfg: dict, r_rear: float) -> dict:
    """The three teleop positions and where each puts the two feet [mm above
    the floor]. Lives here so the model and the numbers cannot drift apart."""
    rest = np.deg2rad(float(cfg["rest_deg"]))
    L, pz, py = float(cfg["length"]), float(cfg["pivot"][2]), float(cfg["pivot"][1])
    out = {}
    for name, th in (("right", np.deg2rad(cfg["deploy_deg"])),
                     ("centre", 0.0),
                     ("left", -np.deg2rad(cfg["deploy_deg"]))):
        near = pz + r_rear + L * np.cos(rest + abs(th))
        far = pz + r_rear + L * np.cos(rest - abs(th))
        out[name] = {"theta_deg": float(np.degrees(th)),
                     "down_foot_z": min(near, far) if th else near,
                     "up_foot_z": max(near, far) if th else far,
                     "down_foot_y": py + L * np.sin(rest + abs(th))}
    return out


FLYWHEEL_CFG = Path(__file__).resolve().parents[2] / "config" / "flywheel.yaml"

FLYWHEEL_AXIS = {"roll": [1.0, 0.0, 0.0], "yaw": [0.0, 0.0, 1.0]}


def _add_flywheel(spec: mujoco.MjSpec, chassis, p: dict, cfg: dict) -> None:
    """A reaction wheel on the chassis (build_model(..., flywheel=True)).

    Config comes from `config/flywheel.yaml`, NOT `bike_params.yaml`, because
    that file is hashed into `params_digest` and this study has not been
    decided. Same pattern as the wing linkage.

    TWO THINGS HERE ARE EASY TO GET WRONG AND BOTH CHANGE THE ANSWER.

    1. INERTIA IS SET EXPLICITLY, not derived from the geom. MuJoCo's solid
       cylinder gives I = m r^2 / 2; a rim-weighted wheel is I = m r^2. That
       factor of two is not cosmetic — the radius is capped by the 80 mm body
       and momentum goes as m r^2, so rim-weighting is the only free doubling
       on the table. Drawing the geom at the true radius and *also* getting
       the rim inertia needs `explicitinertial`.

    2. THE SPEED LIMIT IS JOINT DAMPING, not a clamp. A DC motor delivers
       tau = tau_stall * (1 - w / w_noload), which is exactly a torque source
       in parallel with damping b = tau_stall / w_noload. Modelled that way,
       the momentum budget H = I w_noload enforces ITSELF: the wheel simply
       cannot be driven past its no-load speed, and the saturation that
       defines a reaction wheel's usefulness emerges from the physics instead
       of from a hand-written limit that a policy could be tuned against.

       Referred through a step-up ratio N (flywheel revs per servo rev):
           tau_fw = tau_servo / N,   w_fw = w_servo * N,
           b_fw   = tau_fw / w_fw = tau_servo / (N^2 * w_servo).
    """
    servo = p["servos"][cfg["servo"]]
    axis = str(cfg["axis"]).lower()
    if axis not in FLYWHEEL_AXIS:
        raise ValueError(f"flywheel axis {axis!r}; expected one of {sorted(FLYWHEEL_AXIS)}")

    n = float(cfg["gear_ratio"])
    m, r = float(cfg["mass"]), float(cfg["radius"])
    inertia_spin = m * r * r * float(cfg["rim_fraction"])
    # Transverse inertia of a ring/disc is half the polar value; the wheel is
    # rigid about those axes anyway, so only the spin term does any work.
    inertia_trans = inertia_spin / 2

    tau_servo = servo["stall_torque"]
    w_servo = servo["no_load_rpm"] * 2 * np.pi / 60.0
    tau_fw = tau_servo / n
    w_fw = w_servo * n
    damping = tau_fw / w_fw

    # The mount and gear train are chassis mass, not wheel mass — they do not
    # spin, so lumping them into the wheel would inflate the momentum budget.
    chassis.add_geom(
        name="flywheel_bracket",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.015, 0.015, 0.006],
        pos=cfg["pos"],
        mass=float(cfg["bracket_mass"]),
        contype=0,
        conaffinity=0,
        rgba=[0.35, 0.35, 0.4, 1],
    )

    wheel = chassis.add_body(name="flywheel", pos=cfg["pos"])
    wheel.add_joint(
        name="flywheel_joint",
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=FLYWHEEL_AXIS[axis],
        damping=damping,
    )
    wheel.add_geom(
        name="flywheel_disc",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[r, 0.004, 0],
        quat=_quat_z_to(FLYWHEEL_AXIS[axis]),
        mass=m,
        contype=0,          # a reaction wheel that touches anything is a bug
        conaffinity=0,
        rgba=[0.85, 0.65, 0.15, 0.85],
    )
    wheel.explicitinertial = True
    wheel.mass = m
    wheel.ipos = [0.0, 0.0, 0.0]
    wheel.inertia = ([inertia_trans, inertia_trans, inertia_spin] if axis == "yaw"
                     else [inertia_spin, inertia_trans, inertia_trans])

    act = spec.add_actuator(name="flywheel")
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = "flywheel_joint"
    act.gainprm[0] = 1.0
    act.ctrlrange = [-tau_fw, tau_fw]
    act.ctrllimited = True
    act.forcerange = [-tau_fw, tau_fw]
    act.forcelimited = True

    s = spec.add_sensor(name="flywheel_vel")
    s.type = mujoco.mjtSensor.mjSENS_JOINTVEL
    s.objtype = mujoco.mjtObj.mjOBJ_JOINT
    s.objname = "flywheel_joint"
    s = spec.add_sensor(name="flywheel_pos")
    s.type = mujoco.mjtSensor.mjSENS_JOINTPOS
    s.objtype = mujoco.mjtObj.mjOBJ_JOINT
    s.objname = "flywheel_joint"


def flywheel_budget(p: dict, cfg: dict) -> dict:
    """The four numbers that decide whether a reaction wheel is worth building,
    without running any physics. Kept next to `_add_flywheel` so the model and
    the arithmetic cannot drift apart."""
    servo = p["servos"][cfg["servo"]]
    n = float(cfg["gear_ratio"])
    inertia = float(cfg["mass"]) * float(cfg["radius"]) ** 2 * float(cfg["rim_fraction"])
    w_fw = servo["no_load_rpm"] * 2 * np.pi / 60.0 * n
    tau_fw = servo["stall_torque"] / n
    return {
        "inertia": inertia,
        "tau_max": tau_fw,
        "w_max": w_fw,
        "momentum": inertia * w_fw,       # the whole of the authority
        "spin_up_s": inertia * w_fw / tau_fw,
        "added_mass": float(cfg["mass"]) + float(cfg["bracket_mass"]),
    }


def _add_world(spec: mujoco.MjSpec, p: dict) -> None:
    sim = p["sim"]
    # Checkered floor. Not decoration: against a plain plane the bike has no
    # visual reference at all, so translation is invisible in any tracked
    # camera -- it reads as a bike wobbling on the spot while the world stays
    # put. `run_drive._overlay` draws a grid for the same reason, but that one
    # is +-3 m of scene geoms tied to the teleop dial; a texture is unbounded,
    # costs no geom budget, and shows up in every recording.
    tex = spec.add_texture(
        name="floor_grid",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        width=300, height=300,
        rgb1=[0.78, 0.78, 0.80], rgb2=[0.86, 0.86, 0.88],
    )
    mat = spec.add_material(name="floor_grid")
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "floor_grid"
    # texuniform + texrepeat in WORLD units: one square per FLOOR_GRID_M, so
    # the squares stay the same physical size whatever floor_size is and can
    # be read as a distance scale.
    mat.texuniform = True
    pitch = sim.get("floor_grid_m") or FLOOR_GRID_M
    mat.texrepeat = [1.0 / pitch, 1.0 / pitch]
    # Matte. The material default (specular 0.5) puts a specular highlight on
    # the floor from the DIRECTIONAL sun, and on a flat plane that is a broad
    # smear rather than a small glint -- it washed the checker out across the
    # middle of every plan view, which defeats the point of having a grid.
    # Measured centre-minus-corner brightness: +15.0 at specular 0.5, -3.4 at
    # 0 (i.e. uniform). A floor is not a shiny surface anyway.
    mat.specular = 0.0
    mat.shininess = 0.0
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        material="floor_grid",
        size=[sim["floor_size"], sim["floor_size"], 0.1],
        contype=FLOOR_CONTYPE,
        conaffinity=FLOOR_CONAFF,
        condim=sim["condim"],
        friction=_contact_friction(sim),
        rgba=[0.85, 0.85, 0.85, 1],
    )
    # DIRECTIONAL, not positional. A point light at the origin lights the bike
    # only while it stays near the origin -- drive a few metres and it falls
    # off into flat grey, which shows up in any recording with real
    # translation (the drive scripts, and the righting demo especially, which
    # ends ~8 m out). Directional light has no falloff, so the bike is lit the
    # same wherever it is. `pos` is ignored for a directional light; the
    # direction is what matters.
    sun = spec.worldbody.add_light(name="sun", pos=[0.5, 0.3, 2.0],
                                   dir=[-0.2, -0.1, -1.0])
    sun.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL


def _apply_options(spec: mujoco.MjSpec, p: dict) -> None:
    sim = p["sim"]
    spec.option.timestep = sim["timestep"]
    spec.option.integrator = INTEGRATORS[sim["integrator"]]
    spec.option.cone = CONES[sim["cone"]]
    spec.option.impratio = sim["impratio"]
    # Contact stiffness, on the main default so every geom inherits it. This
    # ran on MuJoCo's stock [0.02, 1] until it was measured: at a 20 ms
    # contact time constant the rollers sank 2.1-2.8 mm into the floor under a
    # hold command -- a fifth of the 11 mm roller radius, and near-identical
    # across five different policies, which is the signature of the contact
    # model rather than the controller setting the behaviour. Penetration goes
    # with the square of the time constant, so 5 ms is ~16x stiffer.
    # NOTE the equality solrefs elsewhere in this file are also 0.005 but are
    # a different constraint class (gear couplings), and never governed this.
    spec.default.geom.solref = list(sim["contact_solref"])
    # Set to the value MuJoCo was already defaulting to, so this is a physics
    # no-op -- but it stops being an invisible inherited choice, and `dmin`
    # confounds both bench tests (see the note in bike_params.yaml).
    spec.default.geom.solimp = list(sim["contact_solimp"])


def build_spec(
    params: dict | None = None,
    variant: str = "full",
    training_wheels: bool = False,
    hockey: bool = False,
    payload: bool = True,
    righting: bool = False,
    wings: bool = False,
    linkage: bool = False,
    linkage_cfg: str | Path | None = None,
    flywheel: bool = False,
    flywheel_cfg: str | Path | None = None,
    swing: bool = False,
    swing_cfg: str | Path | None = None,
    swing_linkage: bool = False,
    swing_linkage_cfg: str | Path | None = None,
) -> mujoco.MjSpec:
    p = params or load_params()
    spec = mujoco.MjSpec()
    spec.modelname = f"aow_bike_{variant}"
    _apply_options(spec, p)
    _add_world(spec, p)

    if variant == "testbed":
        stand = spec.worldbody.add_body(
            name="stand",
            pos=[0, 0, p["omni_wheel"]["outer_radius"] + p["testbed"]["stand_clearance"]],
        )
        stand.add_geom(  # static, no joint -> welded to the world
            name="stand_post",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.01, 0.05, 0.005],
            pos=[0, 0, 0.06],
            mass=0.2,
            contype=0,
            conaffinity=0,
            rgba=[0.5, 0.55, 0.6, 1],
        )
        _add_aow(spec, stand, p)
        return spec
    if variant != "full":
        raise ValueError(f"unknown variant {variant!r}; expected 'full' or 'testbed'")

    bike, ow = p["bike"], p["omni_wheel"]
    r_rear, r_front = ow["outer_radius"], bike["front_wheel"]["radius"]

    # Chassis frame: origin at the rear axle center, +X toward the front wheel.
    chassis = spec.worldbody.add_body(name="chassis", pos=[0, 0, r_rear])
    chassis.add_freejoint()
    # The chassis lumps are inertia primitives, not contact shapes — normally
    # nothing above the wheels can touch anything, which is exactly right for a
    # bike that stays upright. A FALLEN bike lands on them, so the righting
    # study turns them into the outer shell they physically are.
    shell = ((DYN_CONTYPE, DYN_CONAFF)
             if (righting or wings or swing or swing_linkage) else (0, 0))
    ch = bike["chassis"]
    chassis.add_geom(
        name="chassis_box",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=np.array(ch["box_size"]) / 2,
        pos=ch["com_pos"],
        mass=ch["mass"],
        contype=shell[0],
        conaffinity=shell[1],
        condim=p["sim"]["condim"],
        friction=_contact_friction(p["sim"]),
        rgba=[0.2, 0.4, 0.7, 0.6],
    )
    lumps = [
        ("servo_drive_left", p["servos"]["xc430_w150"], p["servos"]["xc430_w150"]["pos_left"]),
        ("servo_drive_right", p["servos"]["xc430_w150"], p["servos"]["xc430_w150"]["pos_right"]),
        ("servo_steer", p["servos"]["xc330_t181"], p["servos"]["xc330_t181"]["pos"]),
        ("ahrs", bike["ahrs"], bike["ahrs"]["pos"]),
    ]
    # Untethered running gear (~190 g, +23%). On by default: the bike we are
    # actually building carries it, and a policy trained without it is
    # optimistic about a bike that does not exist. payload=False recovers the
    # tethered bike for comparison. See docs/plans/untethered-setup.md.
    if payload:
        lumps += [(f"payload_{k}", v, v["pos"]) for k, v in bike["payload"].items()]
    rgba = {"ahrs": [0.7, 0.1, 0.1, 1], "payload_battery": [0.9, 0.5, 0.1, 1],
            "payload_electronics": [0.2, 0.7, 0.3, 1]}
    for name, part, pos in lumps:
        chassis.add_geom(
            name=name,
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=np.array(part["box_size"]) / 2,
            pos=pos,
            mass=part["mass"],
            contype=shell[0],
            conaffinity=shell[1],
            condim=p["sim"]["condim"],
            friction=_contact_friction(p["sim"]),
            rgba=rgba.get(name, [0.1, 0.1, 0.1, 1]),
        )
    chassis.add_site(name="ahrs_site", pos=bike["ahrs"]["pos"])

    if training_wheels:
        tw = p["training_wheels"]
        for side, tag in ((1, "left"), (-1, "right")):
            chassis.add_geom(
                name=f"training_wheel_{tag}",
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[tw["radius"], 0, 0],
                pos=[tw["pos_x"], side * tw["half_span"], tw["radius"] + tw["clearance"] - r_rear],
                mass=0.001,
                contype=DYN_CONTYPE,
                conaffinity=DYN_CONAFF,
                condim=3,
                friction=[0.05, 0.0, 0.0],  # near-frictionless casters
                rgba=[0.9, 0.9, 0.2, 1],
            )

    # Steering: axis tilted back by rake, front axle offset forward of the axis.
    rake = np.deg2rad(bike["rake_deg"])
    steer_axis = np.array([-np.sin(rake), 0.0, np.cos(rake)])  # up-back
    offset_dir = np.array([np.cos(rake), 0.0, np.sin(rake)])  # perp, axis -> axle
    steer = chassis.add_body(
        name="steer", pos=[bike["wheelbase"], 0, r_front - r_rear]  # origin at front axle
    )
    steer.add_joint(
        name="steer_joint",
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=steer_axis,
        pos=-bike["fork_offset"] * offset_dir,  # axis line passes behind the axle
    )
    fork_top = -bike["fork_offset"] * offset_dir + 0.10 * steer_axis
    steer.add_geom(
        name="fork",
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        size=[0.005, 0, 0],
        fromto=np.concatenate([[0, 0, 0], fork_top]),
        mass=bike["fork_mass"],
        contype=0,
        conaffinity=0,
        rgba=[0.6, 0.6, 0.65, 1],
    )

    fw = bike["front_wheel"]
    tire = spec.add_mesh(name="front_tire")
    tire.uservert = geometry.crowned_wheel_vertices(
        fw["radius"], fw["width"], fw["crown_radius"], p["sim"]["mesh_segments"]
    ).flatten()
    front = steer.add_body(name="front_wheel")
    front.add_joint(name="front_spin", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    front.add_geom(
        name="front_tire",
        type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname="front_tire",
        quat=_Y_AXIS_QUAT,
        mass=fw["mass"],
        contype=DYN_CONTYPE,
        conaffinity=DYN_CONAFF,
        condim=p["sim"]["condim"],
        friction=_contact_friction(p["sim"]),
        rgba=[0.15, 0.15, 0.15, 1],
    )

    _add_aow(spec, chassis, p)

    # Steering actuator: XC330 in extended position mode through the steering gear.
    xc330 = p["servos"]["xc330_t181"]
    ratio = bike["steering"]["gear_ratio"]
    act = spec.add_actuator(name="steer")
    act.set_to_position(kp=p["actuators"]["steer_kp"], kv=p["actuators"]["steer_kv"])
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.target = "steer_joint"
    act.forcerange = [-xc330["stall_torque"] * ratio, xc330["stall_torque"] * ratio]
    # no ctrlrange: continuous 360°+ steering, joint is unlimited

    for stype, name in (
        (mujoco.mjtSensor.mjSENS_GYRO, "ahrs_gyro"),
        (mujoco.mjtSensor.mjSENS_ACCELEROMETER, "ahrs_accel"),
        (mujoco.mjtSensor.mjSENS_FRAMEQUAT, "ahrs_quat"),
    ):
        s = spec.add_sensor(name=name)
        s.type = stype
        s.objtype = mujoco.mjtObj.mjOBJ_SITE
        s.objname = "ahrs_site"
    for stype, suffix in (
        (mujoco.mjtSensor.mjSENS_JOINTPOS, "pos"),
        (mujoco.mjtSensor.mjSENS_JOINTVEL, "vel"),
    ):
        s = spec.add_sensor(name=f"steer_{suffix}")
        s.type = stype
        s.objtype = mujoco.mjtObj.mjOBJ_JOINT
        s.objname = "steer_joint"

    if hockey:
        _add_hockey(spec, chassis, p)
    # The linkage is an ALTERNATIVE to the geared pair, not an addition: it
    # brings its own wings, so building both would put four wings on the bike
    # and double-count the mass.
    #
    # Its roof must be re-derived FIRST, because `_add_righting` builds the
    # roof geom from `p` and whatever is in `p` at that moment is what gets
    # built. Deriving it afterwards silently did nothing -- the model kept the
    # geared roof and kept losing the same fall.
    lk_cfg = None
    if linkage:
        # `linkage_cfg` lets a caller build a DIFFERENT four-bar without
        # touching the module default, so an exploratory geometry never
        # becomes the one the rest of the repo silently builds.
        lk_cfg = yaml.safe_load(Path(linkage_cfg or LINKAGE_CFG).read_text())
        p = {**p, "righting": {**p["righting"],
                               "roof": {**p["righting"]["roof"],
                                        **derive_linkage_roof(p, lk_cfg)}}}
    # `wings` implies the righting shell, and swaps the single arm for the wing
    # pair — the bumper rails are shared, the two mechanisms never coexist.
    if righting or wings or linkage or swing or swing_linkage:
        _add_righting(spec, chassis, p,
                      arm=not (wings or linkage or swing or swing_linkage))
    if wings:
        _add_wings(spec, chassis, p)
    if linkage:
        _add_wing_linkage(spec, chassis, p, lk_cfg)
    # Last, so the flywheel's actuator lands after every existing one and no
    # saved policy's action indices shift under it.
    if flywheel:
        _add_flywheel(spec, chassis, p,
                      yaml.safe_load(Path(flywheel_cfg or FLYWHEEL_CFG).read_text()))
    # The co-rotating pair is an ALTERNATIVE to `wings`/`linkage`, never an
    # addition -- building both would put four wings on the bike and
    # double-count the mass, the same trap the linkage note above describes.
    # The co-rotating FOUR-BAR. Alternative to every other mechanism here, for
    # the same reason they are alternatives to each other: two of them would put
    # four wings on the bike and double-count the mass.
    if swing_linkage:
        if wings or linkage or swing:
            raise ValueError("swing_linkage is an ALTERNATIVE to "
                             "wings/linkage/swing -- pick one mechanism")
        _add_swing_linkage(spec, chassis, p, yaml.safe_load(
            Path(swing_linkage_cfg or SWING_LINKAGE_CFG).read_text()))
    if swing:
        if wings or linkage:
            raise ValueError("swing wings are an ALTERNATIVE to wings/linkage, "
                             "not an addition -- pick one mechanism")
        _add_swing_wings(spec, chassis, p,
                         yaml.safe_load(Path(swing_cfg or SWING_CFG).read_text()))

    return spec


def tune_lighting(model: mujoco.MjModel) -> None:
    """Viewing-only lighting tweaks, shared by the recorder and the viewer.

    Not part of `build_spec` because these live on `model.vis`, which is render
    state rather than model structure — but they must be applied identically in
    both places or a teleop session and its recording look like different
    simulators.

    Weighted toward AMBIENT: the headlight is a point light AT THE CAMERA, so a
    plan view lights the floor directly beneath brightest and smears out the
    floor checker in the middle of frame. Ambient is uniform and has no such
    hotspot. Measured on a top view, blown floor pixels 3.0% -> 0.2% moving
    from 0.28/0.22 to these values, and a tracked 3/4 view gets slightly
    BRIGHTER (mean 132.7 -> 138.3) rather than darker. Some diffuse is kept:
    at 0.06 the bike loses its shading and reads as a flat decal.
    """
    model.vis.headlight.ambient[:] = 0.36
    model.vis.headlight.diffuse[:] = 0.10
    model.vis.headlight.specular[:] = 0.0

    # SHADOW COVERAGE. For a directional light the shadow map is a square of
    # half-size `shadowclip * stat.extent` centred on the model origin -- at
    # the stock shadowclip of 1.0 and this model's extent of 2.4 m that is a
    # +-2.4 m box, and the bike simply stops casting a shadow once it drives
    # out of it. Measured shadow strength at the default: 0.74 at x=0, 0.05 at
    # x=-4, 0.01 at x=-12, i.e. gone.
    #
    # Size it from the FLOOR instead, so it always covers wherever the bike can
    # actually be, and raise the shadow texture to pay for the bigger area --
    # spreading the same texels over a wider square is what makes a large
    # shadowclip look washed out. At clip 6 / size 8192 the strength is
    # 0.70/0.69/0.68/0.54 across x = 0..-12, against 0.74 for the stock setup
    # at the origin only.
    half = float(model.geom_size[model.geom("floor").id][0]) or 3.0
    model.vis.map.shadowclip = max(1.0, half / max(model.stat.extent, 1e-6))
    model.vis.quality.shadowsize = max(int(model.vis.quality.shadowsize), 8192)


def reset_actuator_state(model: mujoco.MjModel, data: mujoco.MjData,
                         act=None) -> None:
    """Restore actuator activations alongside a hand-rolled qpos/qvel reset.

    At `actuators.drive_ki` > 0 the drive actuators carry an integrator, so
    `model.na` is 2 rather than 0 and `data.act` is as much of the state as
    `qvel` is. Every reset in this repo writes qpos/qvel by hand instead of
    calling `mj_resetData`, so without this call the integrator LEAKS across
    episode and rollout boundaries: each episode inherits whatever the previous
    one wound up to, as an unbounded hidden input nothing accounts for.

    Measured cost of the leak (drive_kv 0.016016, drive_ki 0.6): the LQR gain
    schedule's worst fit R^2 falls 0.9727 -> 0.7543. Restoring `act` recovers
    0.9412 of that. The rest is the genuine hidden-state problem described in
    docs/plans/aow-contact-approximations.md section 6b, which this does not
    fix and cannot.

    `act` is the activation to restore; None means zero, which is the right
    equilibrium for any reset that starts the input shafts at rest. Callers
    that reset to a ROLLING equilibrium must pass that equilibrium's own `act`
    -- the standing integral there is what holds the speed against droop.

    A no-op at ki = 0, where na = 0 and the drives are memoryless.
    """
    if not model.na:
        return
    data.act[:] = 0.0 if act is None else act


def build_model(
    params: dict | None = None, variant: str = "full", training_wheels: bool = False,
    hockey: bool = False, payload: bool = True, righting: bool = False,
    wings: bool = False, linkage: bool = False,
    linkage_cfg: str | Path | None = None,
    flywheel: bool = False, flywheel_cfg: str | Path | None = None,
    swing: bool = False, swing_cfg: str | Path | None = None,
    swing_linkage: bool = False, swing_linkage_cfg: str | Path | None = None,
) -> mujoco.MjModel:
    return build_spec(params, variant, training_wheels, hockey, payload,
                      righting, wings, linkage, linkage_cfg,
                      flywheel, flywheel_cfg, swing, swing_cfg,
                      swing_linkage, swing_linkage_cfg).compile()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--variant", choices=["full", "testbed"], default="full")
    ap.add_argument("--params", default=None, help="path to bike_params.yaml")
    ap.add_argument("--training-wheels", action="store_true")
    ap.add_argument("--hockey", action="store_true",
                    help="add the ball-shot stick panels + road-hockey ball")
    ap.add_argument("--no-payload", action="store_true",
                    help="omit the untethered running gear (battery + electronics)")
    ap.add_argument("--righting", action="store_true",
                    help="collidable chassis shell + the self-righting bumpers/arm")
    ap.add_argument("--wings", action="store_true",
                    help="the mirrored wing pair instead of the single righting "
                         "arm (implies --righting)")
    ap.add_argument("-o", "--output", default=None, help="write MJCF XML here")
    args = ap.parse_args()
    spec = build_spec(load_params(args.params), args.variant, args.training_wheels,
                      args.hockey, not args.no_payload, args.righting, args.wings)
    spec.compile()  # validate
    xml = spec.to_xml()
    if args.output:
        Path(args.output).write_text(xml)
        print(f"wrote {args.output}")
    else:
        print(xml)


if __name__ == "__main__":
    main()
