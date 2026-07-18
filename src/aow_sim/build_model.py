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

DEFAULT_PARAMS = Path(__file__).resolve().parents[2] / "config" / "bike_params.yaml"

FLOOR_CONTYPE, FLOOR_CONAFF = 1, 2
DYN_CONTYPE, DYN_CONAFF = 2, 1

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


def _normalize(node):
    """Strip {value:, source:} wrappers, leaving plain values."""
    if isinstance(node, dict):
        if "value" in node and "source" in node:
            return node["value"]
        return {k: _normalize(v) for k, v in node.items()}
    return node


def load_params(path: str | Path | None = None) -> dict:
    with open(path or DEFAULT_PARAMS) as f:
        return _normalize(yaml.safe_load(f))


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


def _add_world(spec: mujoco.MjSpec, p: dict) -> None:
    sim = p["sim"]
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[sim["floor_size"], sim["floor_size"], 0.1],
        contype=FLOOR_CONTYPE,
        conaffinity=FLOOR_CONAFF,
        condim=sim["condim"],
        friction=_contact_friction(sim),
        rgba=[0.85, 0.85, 0.85, 1],
    )
    spec.worldbody.add_light(name="sun", pos=[0.5, 0.3, 2.0], dir=[-0.2, -0.1, -1.0])


def _apply_options(spec: mujoco.MjSpec, p: dict) -> None:
    sim = p["sim"]
    spec.option.timestep = sim["timestep"]
    spec.option.integrator = INTEGRATORS[sim["integrator"]]
    spec.option.cone = CONES[sim["cone"]]
    spec.option.impratio = sim["impratio"]


def build_spec(
    params: dict | None = None,
    variant: str = "full",
    training_wheels: bool = False,
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
    ch = bike["chassis"]
    chassis.add_geom(
        name="chassis_box",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=np.array(ch["box_size"]) / 2,
        pos=ch["com_pos"],
        mass=ch["mass"],
        contype=0,
        conaffinity=0,
        rgba=[0.2, 0.4, 0.7, 0.6],
    )
    for name, servo, pos in (
        ("servo_drive_left", p["servos"]["xc430_w150"], p["servos"]["xc430_w150"]["pos_left"]),
        ("servo_drive_right", p["servos"]["xc430_w150"], p["servos"]["xc430_w150"]["pos_right"]),
        ("servo_steer", p["servos"]["xc330_t181"], p["servos"]["xc330_t181"]["pos"]),
        ("ahrs", bike["ahrs"], bike["ahrs"]["pos"]),
    ):
        chassis.add_geom(
            name=name,
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=np.array(servo["box_size"]) / 2,
            pos=pos,
            mass=servo["mass"],
            contype=0,
            conaffinity=0,
            rgba=[0.1, 0.1, 0.1, 1] if name != "ahrs" else [0.7, 0.1, 0.1, 1],
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

    return spec


def build_model(
    params: dict | None = None, variant: str = "full", training_wheels: bool = False
) -> mujoco.MjModel:
    return build_spec(params, variant, training_wheels).compile()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--variant", choices=["full", "testbed"], default="full")
    ap.add_argument("--params", default=None, help="path to bike_params.yaml")
    ap.add_argument("--training-wheels", action="store_true")
    ap.add_argument("-o", "--output", default=None, help="write MJCF XML here")
    args = ap.parse_args()
    spec = build_spec(load_params(args.params), args.variant, args.training_wheels)
    spec.compile()  # validate
    xml = spec.to_xml()
    if args.output:
        Path(args.output).write_text(xml)
        print(f"wrote {args.output}")
    else:
        print(xml)


if __name__ == "__main__":
    main()
