"""Can the rear drive lift the nose at all? An open-loop wheelie launch.

docs/plans/wishlist.md, "wheelie". Before any policy: drive the rear wheel
open-loop and see whether the front wheel leaves the floor, how far the nose
comes up, and what limits it (torque, the servo's speed droop, or roller
traction).

The floor's sliding friction is params `sim.friction_sliding` (0.9, a GUESS):
the printed rear friction use is what a real floor must supply.

ROLL IS PINNED. The question is the pitch plane only, so after every physics
step the chassis freejoint is projected onto it: y, roll and yaw (and their
rates) are zeroed, x, z and pitch are kept. The same projection
`linearize.settle_upright` uses for roll, extended to the sagittal plane. It
is a kinematic constraint, not a rig: the floor rig (floor_rig.py) would add
an arm, a pitch pivot at the CoG and skids, all of which are in the way of a
wheelie. Steer is held at 0 by its own servo.

Two entries:

  launch   from rest, both drive inputs commanded to full forward speed.
  brake    reverse at --v-rev, then full forward: the bike's own speed does
           the work instead of stall torque alone.

and `--ballast` adds a point mass to the chassis (g, x, z, from the rear
axle) to ask whether a heavier rear helps.

    python analysis/moves/wheelie_launch.py
    python analysis/moves/wheelie_launch.py --ballast 100 0 0 --ballast 100 -0.04 0.04
    python analysis/moves/wheelie_launch.py --mu 0.7

IDEAL drivetrain (the velocity-servo actuators, no detent or slop): an
upper bound on what the drive can do.
"""
from __future__ import annotations

import argparse

import mujoco
import numpy as np

from aow_sim.build_model import build_model, load_params
from aow_sim.control.linearize import settle_upright


def pitch_of(q):
    """Nose-up pitch [rad] of a quaternion with no roll or yaw (w, 0, y, 0):
    MuJoCo's +y rotation pitches the nose DOWN, so negate."""
    return -2.0 * np.arctan2(q[2], q[0])


def pin(d):
    """Project the chassis freejoint onto the sagittal plane."""
    th = pitch_of(d.qpos[3:7]) if abs(d.qpos[4]) + abs(d.qpos[6]) < 0.5 else pitch_of(
        d.qpos[3:7] / np.linalg.norm(d.qpos[3:7]))
    d.qpos[1] = 0.0
    d.qpos[3:7] = [np.cos(-th / 2), 0.0, np.sin(-th / 2), 0.0]
    d.qvel[1] = 0.0
    d.qvel[3] = 0.0
    d.qvel[5] = 0.0


def add_ballast(m, d, grams, x, z):
    """Lump a point mass into the chassis at (x, z) from the rear axle."""
    b = m.body("chassis").id
    mb, cb = m.body_mass[b], m.body_ipos[b].copy()
    pm = grams / 1000.0
    axle = d.xpos[m.body("aow_hub").id] - d.xpos[b]      # rear axle, chassis frame
    p = axle + np.array([x, 0.0, z])
    c = (mb * cb + pm * p) / (mb + pm)
    # inertia: parallel-axis shift of the old body plus the point mass, kept
    # diagonal in the body's inertial frame (an approximation; small terms)
    I = m.body_inertia[b].copy()
    for k, (a1, a2) in enumerate(((1, 2), (0, 2), (0, 1))):
        I[k] += mb * ((cb - c)[a1] ** 2 + (cb - c)[a2] ** 2) + pm * ((p - c)[a1] ** 2 + (p - c)[a2] ** 2)
    m.body_mass[b], m.body_ipos[b], m.body_inertia[b] = mb + pm, c, I


def run(params, entry, ballast=None, v_rev=0.8, T=1.0):
    m = build_model(params)
    d = settle_upright(m)
    if ballast:
        add_ballast(m, d, *ballast)
        mujoco.mj_setConst(m, d)
        d = settle_upright(m)
    dt = m.opt.timestep
    aa, ab, st = (m.actuator(n).id for n in ("drive_a", "drive_b", "steer"))
    wmax = float(m.actuator_ctrlrange[aa][1])
    r = float(d.xpos[m.body("aow_hub").id][2])
    front = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name == "front_wheel"]
    hub = m.body("aow_hub").id
    mujoco.mj_forward(m, d)
    com = d.subtree_com[1] - d.xpos[hub]
    mass = m.body_subtreemass[1]

    def step(w):
        d.ctrl[aa] = d.ctrl[ab] = w
        d.ctrl[st] = 0.0
        mujoco.mj_step(m, d)
        pin(d)

    if entry == "brake":                       # back up to speed first
        for _ in range(int(1.5 / dt)):
            step(-v_rev / r)
    t0 = d.time
    out = dict(t=[], pitch=[], v=[], front_n=[], rear_mu=[])
    for _ in range(int(T / dt)):
        step(wmax)
        fn, rn, rt = 0.0, 0.0, 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            f = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, f)
            if c.geom1 in front or c.geom2 in front:
                fn += f[0]
            else:                              # the rear wheel's rollers
                rn += f[0]
                rt += np.hypot(f[1], f[2])
        out["t"].append(d.time - t0)
        out["pitch"].append(np.degrees(pitch_of(d.qpos[3:7])))
        out["v"].append(d.qvel[0])
        out["front_n"].append(fn)
        out["rear_mu"].append(rt / rn if rn > 1e-3 else 0.0)
    o = {k: np.asarray(v) for k, v in out.items()}
    o.update(mass=mass, com=com, r=r, wmax=wmax)
    return o


def report(name, o):
    lift = o["front_n"] < 1e-6
    t_lift = o["t"][np.argmax(lift)] if lift.any() else None
    bal = np.degrees(np.arctan2(o["com"][0], o["com"][2]))
    up = o["pitch"] >= bal
    kb = np.argmax(up)
    early = o["t"] <= (o["t"][kb] if up.any() else o["t"][-1])
    print(f"{name}: {o['mass']:.3f} kg, CoM {o['com'][0]*1e3:.1f} mm ahead of / "
          f"{o['com'][2]*1e3:.1f} above the axle, balance point {bal:.1f} deg nose-up")
    print(f"    front lifts at {'%.3f s' % t_lift if t_lift is not None else 'NEVER'}; "
          + (f"balance point at {o['t'][kb]:.3f} s, {o['v'][kb]:+.2f} m/s; "
             if up.any() else "balance point never reached; ")
          + f"max nose-up {o['pitch'].max():.1f} deg; "
          f"rear friction use (|Ft|/Fn) up to then {o['rear_mu'][early].max():.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--v-rev", type=float, default=0.8, help="reverse speed before the brake entry [m/s]")
    ap.add_argument("--ballast", type=float, nargs=3, action="append", metavar=("G", "X", "Z"),
                    help="extra point mass [g] at (x, z) [m] from the rear axle; repeatable")
    ap.add_argument("--mu", type=float, help="override sim.friction_sliding")
    a = ap.parse_args()
    p = load_params()
    if a.mu is not None:
        p["sim"]["friction_sliding"] = a.mu
        print(f"floor sliding friction {a.mu} (params: 0.9, GUESS)")
    for bal in [None] + (a.ballast or []):
        tag = "" if bal is None else f" +{bal[0]:.0f} g @ ({bal[1]:+.2f}, {bal[2]:+.2f})"
        report("launch" + tag, run(p, "launch", bal))
        report(f"brake from {-a.v_rev:+.1f}" + tag, run(p, "brake", bal, v_rev=a.v_rev))


if __name__ == "__main__":
    main()
