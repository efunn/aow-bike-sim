"""Every mass in a bike file, where it sits, and how far it pulls the CoM.

Built from the compiled model, standing upright at rest, so it is what the
sim flies: the chassis residual (fasteners) spread over the frame parts, the
printed-mass scale applied, the righting module and wings at their stowed
pose. Each row is one part (a frame or payload entry, a body's own mass);
`pulls` is mass x (its height - the CoM's) / total mass, so the column sums
to zero and says which parts hold the CoM up.

    python analysis/mechanism/mass_table.py config/variants/mini.yaml
    python analysis/mechanism/mass_table.py config/variants/mini.yaml --against
    python analysis/mechanism/mass_table.py --sort x

--against adds the built bike's total for comparison. Read-only.
"""
from __future__ import annotations

import argparse
import re
from collections import defaultdict

import mujoco
import numpy as np

from aow_sim.build_model import build_spec, load_params


def table(path=None):
    """[(part, kg, xyz m)], total kg, CoM xyz m."""
    spec = build_spec(load_params(path))
    gmass = {g.name: g.mass for g in spec.geoms if g.name}
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    parts = defaultdict(lambda: [0.0, np.zeros(3)])
    on_geoms = defaultdict(float)
    for i in range(m.ngeom):
        mass = gmass.get(m.geom(i).name, 0.0)
        if not mass > 0 or m.geom_bodyid[i] == 0:
            continue
        name = re.sub(r"_\d+$", "", m.geom(i).name)
        name = re.sub(r"^roller_\d+_[ab]$", "rollers", name)
        parts[name][0] += mass
        parts[name][1] += mass * d.geom_xpos[i]
        on_geoms[m.geom_bodyid[i]] += mass
    # Mass a body carries with no geom mass behind it (an explicit inertial).
    for b in range(1, m.nbody):
        rest = m.body_mass[b] - on_geoms[b]
        if rest > 1e-5:
            name = m.body(b).name
            name = "play bodies" if name.endswith("_play") else name
            parts[name][0] += rest
            parts[name][1] += rest * d.xipos[b]
    rows = [(k, v[0], v[1] / v[0]) for k, v in parts.items()]
    total = sum(r[1] for r in rows)
    com = sum(r[1] * r[2] for r in rows) / total
    return rows, total, com


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("params", nargs="?", default=None,
                    help="a bike file (default: config/bike_params.yaml)")
    ap.add_argument("--sort", choices=("mass", "z", "x", "pull"), default="mass")
    ap.add_argument("--against", action="store_true",
                    help="also print the built bike's total and CoM")
    a = ap.parse_args()
    rows, total, com = table(a.params)
    key = {"mass": lambda r: -r[1], "z": lambda r: -r[2][2], "x": lambda r: -r[2][0],
           "pull": lambda r: -r[1] * (r[2][2] - com[2])}[a.sort]
    print(f"{'part':30s} {'g':>6s} {'x':>6s} {'y':>6s} {'z':>6s} {'pulls':>7s}   (mm; x ahead of the "
          "rear axle, y left, z above the floor; pulls = on the CoM height)")
    for name, mass, x in sorted(rows, key=key):
        pull = mass * (x[2] - com[2]) / total
        print(f"{name:30s} {mass*1e3:6.1f} {x[0]*1e3:6.0f} {x[1]*1e3:+6.0f} {x[2]*1e3:6.0f} "
              f"{pull*1e3:+7.2f}")
    print(f"{'TOTAL':30s} {total*1e3:6.1f} {com[0]*1e3:6.1f} {com[1]*1e3:+6.1f} {com[2]*1e3:6.1f}")
    if a.against:
        _r, t0, c0 = table(None)
        print(f"{'built bike':30s} {t0*1e3:6.1f} {c0[0]*1e3:6.1f} {c0[1]*1e3:+6.1f} {c0[2]*1e3:6.1f}")


if __name__ == "__main__":
    main()
