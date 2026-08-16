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
    for tag in ("a", "b"):
        act = spec.add_actuator(name=f"drive_{tag}")
        act.set_to_velocity(kv=p["actuators"]["drive_kv"])
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
        # The wing is a flat panel alongside the bike: the bike lies ON it and
        # the mechanism levers it out, so the whole face is the contact, not a
        # tip. Modelled as one long capsule for now.
        lo = np.array([side * stow_out, wing_lo - pivot_z])
        hi = np.array([side * stow_out, wing_hi - pivot_z])
        wing.add_geom(
            name=f"wing_{tag}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[w_ref["radius"], 0, 0],
            fromto=[0, lo[0], lo[1], 0, hi[0], hi[1]],
            mass=w_ref["mass"], contype=DYN_CONTYPE, conaffinity=DYN_CONAFF,
            condim=sim["condim"], friction=_contact_friction(sim),
            rgba=[0.85, 0.2, 0.2, 1] if side < 0 else [0.2, 0.4, 0.8, 1])
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
    shell = ((DYN_CONTYPE, DYN_CONAFF) if (righting or wings) else (0, 0))
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
        lk_cfg = yaml.safe_load(LINKAGE_CFG.read_text())
        p = {**p, "righting": {**p["righting"],
                               "roof": {**p["righting"]["roof"],
                                        **derive_linkage_roof(p, lk_cfg)}}}
    # `wings` implies the righting shell, and swaps the single arm for the wing
    # pair — the bumper rails are shared, the two mechanisms never coexist.
    if righting or wings or linkage:
        _add_righting(spec, chassis, p, arm=not (wings or linkage))
    if wings:
        _add_wings(spec, chassis, p)
    if linkage:
        _add_wing_linkage(spec, chassis, p, lk_cfg)

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


def build_model(
    params: dict | None = None, variant: str = "full", training_wheels: bool = False,
    hockey: bool = False, payload: bool = True, righting: bool = False,
    wings: bool = False, linkage: bool = False,
) -> mujoco.MjModel:
    return build_spec(params, variant, training_wheels, hockey, payload,
                      righting, wings, linkage).compile()


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
