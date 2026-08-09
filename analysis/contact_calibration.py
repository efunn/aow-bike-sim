"""What the two bench tests you can already do would pin down in `solref`.

`sim.contact_solref` is a PAIR, and the two numbers are separately
identifiable by two separate experiments:

  timeconst  sets contact STIFFNESS -> a static load-deflection test.
             Put a known weight on the wheel, measure how far it sinks.
  dampratio  sets contact DAMPING, i.e. restitution -> a drop test.
             Drop the wheel, count the bounces / measure rebound height.

That separation is the useful part: a drop-test rig is the fiddlier of the
two, and it is only needed for `dampratio`. Stiffness -- the number that
produced the 2 mm of sink this whole thread started with -- comes from a
weight and a caliper.

  python analysis/contact_calibration.py
  python analysis/contact_calibration.py --load-kg 4.5 --drop-mm 35

WHAT THE MODEL SAYS NOW. dampratio is 1.0, which is CRITICAL damping, and a
critically damped contact cannot bounce. A wheel that audibly bounces two or
three times is under-damped, so if the bench test bounces and the model does
not, dampratio is the parameter that is wrong -- not timeconst, and not the
friction coefficients.

METHOD, static. Not a settling simulation: the bike is placed at a series of
prescribed heights and mj_forward is called at each, reading the rear-wheel
normal force against the measured penetration. That is the load-deflection
curve of the contact itself, with no dynamics, no balance controller, and no
question of whether it had settled. It is directly comparable to setting a
weight on the wheel and measuring the sink, because the curve is a property of
the contact, not of what is stacked on top of it.

METHOD, drop. The bike is lifted clear and released, and the rear wheel's
clearance is tracked through the bounce sequence. Restitution is taken from
successive apex heights, e = sqrt(h2/h1). The bench test drops the wheel
ALONE, so the absolute apex heights will not match a whole-bike drop; what
transfers is the mapping from dampratio to bounciness, which is what needs
calibrating.

Read-only: builds models in memory, writes nothing, changes no config.
"""

from __future__ import annotations

import argparse

import mujoco
import numpy as np

from aow_sim.build_model import build_model, load_params
from aow_sim.control.linearize import settle_upright
from wheel_slowmo import clearance_mm, wheel_vertices

G = 9.81


def _model(timeconst, dampratio):
    """A model with the contact solref overridden, everything else stock."""
    m = build_model(load_params(), variant="full")
    m.geom_solref[:, 0] = timeconst
    m.geom_solref[:, 1] = dampratio
    return m


def _rear(model):
    names = [model.geom(i).name for i in range(model.ngeom)]
    return ({i for i, n in enumerate(names) if n.startswith("roller_")},
            names.index("floor"))


def _rear_normal_force(model, data, rear, floor):
    """Total vertical contact force on the rear wheel [N]."""
    f, buf = 0.0, np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if floor not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == floor else c.geom1
        if other not in rear:
            continue
        mujoco.mj_contactForce(model, data, i, buf)
        # buf[0] is along the contact normal, which for the floor is +z
        f += float(buf[0]) * abs(c.frame[2])
    return f


def static_curve(timeconst, dampratio, depths_mm):
    """(penetration mm, rear normal force N) at prescribed heights."""
    m = _model(timeconst, dampratio)
    d = mujoco.MjData(m)
    q0 = settle_upright(m).qpos.copy()
    rear, floor = _rear(m)
    verts = wheel_vertices(m, rear)
    out = []
    for dz in depths_mm:
        d.qpos[:] = q0
        d.qpos[2] -= dz * 1e-3
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        pen = -clearance_mm(d, verts)          # mm, positive = sunk in
        out.append((pen, _rear_normal_force(m, d, rear, floor)))
    return np.array(out)


def deflection_at(curve, force_n):
    """Invert the load-deflection curve: how far does it sink under this
    load? Returns NaN if the probe range never reached that force."""
    pen, f = curve[:, 0], curve[:, 1]
    ok = np.argsort(f)
    if force_n > f.max():
        return float("nan")
    return float(np.interp(force_n, f[ok], pen[ok]))


def drop_test(timeconst, dampratio, drop_mm, seconds=1.2):
    """Release the bike from `drop_mm` clear of the floor; return the apex
    heights [mm] of the rear wheel through the bounce sequence."""
    m = _model(timeconst, dampratio)
    d = mujoco.MjData(m)
    d.qpos[:] = settle_upright(m).qpos
    rear, _floor = _rear(m)
    verts = wheel_vertices(m, rear)
    mujoco.mj_forward(m, d)
    d.qpos[2] += drop_mm * 1e-3 - clearance_mm(d, verts) * 1e-3
    d.qvel[:] = 0.0
    n = int(seconds / m.opt.timestep)
    h = np.empty(n)
    for i in range(n):
        mujoco.mj_step(m, d)
        h[i] = clearance_mm(d, verts)
    # apexes: local maxima of clearance that are actually off the floor
    apex = []
    for i in range(1, n - 1):
        if h[i] > h[i - 1] and h[i] >= h[i + 1] and h[i] > 0.05:
            if not apex or i - apex[-1][0] > 200:      # 40 ms debounce
                apex.append((i, h[i]))
    return [v for _i, v in apex]


def restitution(apexes, drop_mm):
    """e from successive apex heights. The first entry is the release height,
    so the first REBOUND is apexes[0] if the drop is counted separately."""
    hs = [drop_mm] + list(apexes)
    return [float(np.sqrt(hs[i + 1] / hs[i])) for i in range(len(hs) - 1)
            if hs[i] > 0]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--load-kg", type=float, default=4.5,
                    help="static bench load on top of the wheel [kg]")
    ap.add_argument("--drop-mm", type=float, default=35.0,
                    help="bench drop height [mm]")
    ap.add_argument("--timeconsts", type=float, nargs="*",
                    default=[0.02, 0.01, 0.005, 0.002])
    ap.add_argument("--dampratios", type=float, nargs="*",
                    default=[1.0, 0.7, 0.5, 0.3, 0.2])
    args = ap.parse_args()

    p = load_params()
    cur = p["sim"]["contact_solref"]
    bike_n = float(build_model(p, variant="full").body_subtreemass[1]) * G
    load_n = args.load_kg * G
    print(f"config now: contact_solref {cur}")
    print(f"bike weight {bike_n:.1f} N; bench load {args.load_kg} kg "
          f"= {load_n:.1f} N\n")

    depths = np.linspace(0.0, 12.0, 90)
    print("STATIC  load-deflection of the rear wheel contact")
    print(f"{'timeconst':>10}{'sink @ bike wt':>16}{'sink @ 2x':>11}"
          f"{'sink @ bench':>14}{'k at bench':>12}")
    print(f"{'[s]':>10}{'[mm]':>16}{'[mm]':>11}{'[mm]':>14}{'[N/mm]':>12}")
    for tc in args.timeconsts:
        c = static_curve(tc, 1.0, depths)
        d1 = deflection_at(c, bike_n)
        d2 = deflection_at(c, 2 * bike_n)
        db = deflection_at(c, load_n)
        k = load_n / db if db == db and db > 0 else float("nan")
        star = "  <- current" if abs(tc - cur[0]) < 1e-9 else ""
        print(f"{tc:>10.4f}{d1:>16.3f}{d2:>11.3f}{db:>14.3f}{k:>12.1f}{star}")

    print(f"\nDROP  released {args.drop_mm:.0f} mm clear, rear-wheel apexes")
    print(f"{'dampratio':>10}{'bounces':>9}{'apex heights [mm]':>34}"
          f"{'restitution':>13}")
    for dr in args.dampratios:
        ap_h = drop_test(cur[0], dr, args.drop_mm)
        e = restitution(ap_h, args.drop_mm)
        star = "  <- current" if abs(dr - cur[1]) < 1e-9 else ""
        hs = " ".join(f"{v:.1f}" for v in ap_h[:5]) or "-"
        print(f"{dr:>10.2f}{len(ap_h):>9}{hs:>34}"
              f"{(f'{e[0]:.2f}' if e else '-'):>13}{star}")

    print("\nHOW TO USE THIS. Match the static column to a weight-on-wheel\n"
          "measurement to fix `timeconst`, then match the bounce count and\n"
          "first rebound height to a drop to fix `dampratio`. They are\n"
          "independent, so the static test alone is already worth doing.")


if __name__ == "__main__":
    main()
