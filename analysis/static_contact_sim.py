"""Static load vs sink of the rear omni wheel, simulated the way the bench measures it.

The rear wheel exactly as the bike builds it (`build_model._add_aow`: same
meshes, same contact settings) rides on a damped vertical slide above the
floor. The hub is held at one phase by joint equalities, standing in for the
servos holding position, and the rollers are locked. A load sits on the axle,
the slide settles to rest, and the result is read the way a dial indicator on
the axle would read it: axle height. Sink = the unloaded rolling radius at that
phase minus the axle height.

THE SINK IS NOT A MATERIAL PROPERTY IN MUJOCO. At rest, MuJoCo's soft contact
sinks roughly in proportion to load / (the mass compiled around the contact).
The solver scales each contact by an effective mass taken from
`body_invweight0`, which is computed at compile time. So these numbers belong
to THIS rig's masses. Measured 2026-10-01, 44 N, solref [0.005, 1], changing
only the carriage mass:

    carriage mass                 0.01 kg   0.5 kg   4.0 kg
    front tire, sink              0.447     0.072    0.010 mm
    rear, one roller point        0.564     0.472    0.446 mm
    front, solref (-2e4, -100)    0.660     0.142    0.020 mm

The rear barely moves because the HUB's spin dominates its effective mass:
each roller sits 40 mm off the hub axis, and invweight0 sees the hub turning
on its own small inertia, because it ignores the belt couplings to the
servos. Adding inertia to the hub joint cut the rear's sink; adding it to the
rollers or the belt inputs changed nothing. The front tire's effective mass
is the carriage. The negative form scales the same way. Changing `body_mass` AFTER compiling does not move invweight0, so it
changes nothing: an earlier "force and mass give identical results" check was
that artifact. The full bike resting on its own weight (rear 5.46 N, 0.135 mm)
agrees with the rear here (0.107 mm). Do not read the front table as the
bike's front.

For numbers that belong to the BIKE, use `contact_calibration.static_curve`:
the same settling, on the whole bike's model, so it carries the bike's
masses. This rig is what showed the mass dependence, and it is the quick way
to see per-point contact behaviour (`--points`).

Phases are named by the axle angle from straight down. One gotcha for
anything that sets the hub angle: turn `input_a_spin` and `input_b_spin` by
the same angle (1:1 in joint space). Otherwise the belt tendons are violated
and the forces are wrong. The run asserts the couplings hold before stepping.

    python analysis/static_contact_sim.py
    python analysis/static_contact_sim.py --points          # where each contact sits
    python analysis/static_contact_sim.py --solref 0.0035,1.0
    python analysis/static_contact_sim.py --wheel front   # the crowned front tire
"""
from __future__ import annotations

import argparse

import mujoco
import numpy as np

from aow_sim import geometry
from aow_sim.build_model import (_Y_AXIS_QUAT, _add_aow, _add_world, _apply_options,
                                 _part_contact, load_params)
from wheel_slowmo import clearance_mm, wheel_vertices

G = 9.81
LOADS = (3.0, 5.5, 11.0, 22.0, 44.0)
# rear: axle angle from straight down [deg]
PHASES = {
    "two big ends (same axle)": 0.0,
    "big-end point (ridge)": 6.0,
    "flat on one roller": 13.0,
    "small-end point": 19.0,
    "two small ends (next axles)": 22.5,
}
# front: wheel angle [deg]; a mesh vertex points straight down at 0, and the
# middle of a facet at half a segment (180 / mesh_segments, filled in below)
FRONT_PHASES = {"vertex down": 0.0, "facet down": None}
SETTLE_S = 1.5
SLIDE_DAMPING = 40.0      # N s/m; only sets how fast it comes to rest


def _add_front(spec, car, p) -> None:
    """The front tire as the bike builds it (build_model, steer section)."""
    fw = p["bike"]["front_wheel"]
    mesh = spec.add_mesh(name="front_tire")
    mesh.uservert = geometry.crowned_wheel_vertices(
        fw["radius"], fw["width"], fw["crown_radius"], p["sim"]["mesh_segments"]
    ).flatten()
    body = car.add_body(name="front_wheel")
    body.add_joint(name="front_spin", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    body.add_geom(name="front_tire", type=mujoco.mjtGeom.mjGEOM_MESH,
                  meshname="front_tire", quat=_Y_AXIS_QUAT, mass=fw["mass"],
                  **_part_contact(p["sim"], "front_tire"))


def build(phase_deg: float, solref=None, wheel: str = "rear") -> mujoco.MjModel:
    p = load_params()
    if solref is not None:
        p["sim"]["contact_solref"] = list(solref)
    spec = mujoco.MjSpec()
    _apply_options(spec, p)
    _add_world(spec, p)
    r = (p["omni_wheel"]["outer_radius"] if wheel == "rear"
         else p["bike"]["front_wheel"]["radius"])
    car = spec.worldbody.add_body(name="carriage", pos=[0, 0, r + 0.001])
    car.add_joint(name="slide", type=mujoco.mjtJoint.mjJNT_SLIDE,
                  axis=[0, 0, 1], damping=SLIDE_DAMPING)
    car.add_geom(name="tray", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                 size=[0.005, 0, 0], mass=0.01, contype=0, conaffinity=0)
    if wheel == "rear":
        _add_aow(spec, car, p)
        locks = (("hub_spin", np.radians(phase_deg)), ("ring_spin", 0.0))
    else:
        _add_front(spec, car, p)
        locks = (("front_spin", np.radians(phase_deg)),)
    for joint, value in locks:
        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_JOINT
        eq.name1 = joint
        eq.data[:5] = [value, 0, 0, 0, 0]
        eq.solref = [0.002, 1.0]
    return spec.compile()


def set_phase(m, d, phase_deg: float) -> None:
    """Rear: hub at phase_deg, ring 0, inputs where the belt tendons put them
    (1:1). Front: the wheel at phase_deg."""
    names = ("hub_spin", "input_a_spin", "input_b_spin")
    if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "front_spin") >= 0:
        names = ("front_spin",)
    for j in names:
        d.qpos[m.jnt_qposadr[m.joint(j).id]] = np.radians(phase_deg)


def axle_angle(m, d) -> float:
    """Angle of the axle nearest straight down, from straight down [deg]."""
    hub = d.xpos[m.body("aow_hub").id]
    angles = []
    for i in range(8):
        v = d.xpos[m.body(f"roller_axle_{i}").id] - hub
        angles.append(np.degrees(np.arctan2(v[0], -v[2])))
    return min(angles, key=abs)


def hub_for(axle_deg: float) -> float:
    """Hub angle that puts the nearest axle `axle_deg` from straight down."""
    m = build(0.0)
    d = mujoco.MjData(m)
    set_phase(m, d, 0.0)
    mujoco.mj_kinematics(m, d)
    a0 = axle_angle(m, d)
    set_phase(m, d, 1.0)
    mujoco.mj_kinematics(m, d)
    return (axle_deg - a0) / (axle_angle(m, d) - a0)


def settle(phase_deg: float, load_n: float, solref=None, wheel: str = "rear") -> dict:
    m = build(phase_deg, solref, wheel)
    d = mujoco.MjData(m)
    tyre = {i for i in range(m.ngeom)
            if m.geom(i).name.startswith("roller_") or m.geom(i).name == "front_tire"}
    floor = m.geom("floor").id
    verts = wheel_vertices(m, tyre)
    car = m.body("carriage").id
    set_phase(m, d, phase_deg)
    mujoco.mj_forward(m, d)
    worst = max((abs(x) for x, t in zip(d.efc_pos, d.efc_type)
                 if t == mujoco.mjtConstraint.mjCNSTR_EQUALITY), default=0.0)
    assert worst < 1e-6, f"couplings violated at start: {worst}"
    radius = d.xpos[car][2] * 1e3 - clearance_mm(d, verts)
    own = sum(m.body_mass[b] for b in range(1, m.nbody)) * G
    d.xfrc_applied[car, 2] = -(load_n - own)
    for _ in range(int(SETTLE_S / m.opt.timestep)):
        mujoco.mj_step(m, d)
    n_total, points, f = 0.0, [], np.zeros(6)
    hub = d.xpos[car]
    for i in range(d.ncon):
        c = d.contact[i]
        if floor not in (c.geom1, c.geom2):
            continue
        mujoco.mj_contactForce(m, d, i, f)
        fz = f[0] * abs(c.frame[2])
        n_total += fz
        g = c.geom2 if c.geom1 == floor else c.geom1
        points.append(dict(geom=m.geom(g).name, x_mm=(c.pos[0] - hub[0]) * 1e3,
                           y_mm=(c.pos[1] - hub[1]) * 1e3,
                           depth_mm=-c.dist * 1e3, n=fz))
    axle = d.xpos[car][2] * 1e3
    return dict(radius=radius, axle=axle, sink=radius - axle, n=n_total,
                points=points, speed=abs(d.qvel[m.jnt_dofadr[m.joint("slide").id]]))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--solref", help="timeconst,dampratio (default: the config's)")
    ap.add_argument("--wheel", choices=("rear", "front"), default="rear")
    ap.add_argument("--points", action="store_true",
                    help="list each contact: roller, position ahead of the axle, depth, force")
    args = ap.parse_args()
    solref = [float(x) for x in args.solref.split(",")] if args.solref else None
    print(f"contact_solref {solref or load_params()['sim']['contact_solref']}; "
          f"sink in mm below the unloaded radius at each phase")
    print(f"{'phase (axle from down)':34s}" + "".join(f"{L:>8.1f} N" for L in LOADS))
    if args.wheel == "rear":
        phases = {n: hub_for(ax) for n, ax in PHASES.items()}
        labels = {n: f"{n} ({ax:g} deg)" for n, ax in PHASES.items()}
    else:
        half = 180.0 / load_params()["sim"]["mesh_segments"]
        phases = {n: (half if v is None else v) for n, v in FRONT_PHASES.items()}
        labels = {n: f"{n} ({phases[n]:g} deg)" for n in phases}
    for name, phase in phases.items():
        rows = [settle(phase, L, solref, args.wheel) for L in LOADS]
        for r, L in zip(rows, LOADS):
            assert abs(r["n"] - L) < 0.01 and r["speed"] < 1e-5, (name, L, r["n"], r["speed"])
        print(f"{labels[name]:34s}"
              + "".join(f"{r['sink']:7.3f} {len(r['points'])}p" for r in rows))
        if args.points:
            for r, L in zip(rows, LOADS):
                pts = "; ".join(f"{p['geom']} x {p['x_mm']:+.2f} y {p['y_mm']:+.2f} mm, "
                                f"depth {p['depth_mm']:.3f}, {p['n']:.1f} N"
                                for p in r["points"])
                print(f"      {L:5.1f} N: {pts}")


if __name__ == "__main__":
    main()
