"""Does the sim's XC330 reproduce the X330 fixture's lift and fall currents?

The CALIBRATION check for righting_servo.py + gearbox_friction.py, kept out of
tests/ on purpose: the tests pin the implementation against the params' own
lines, so a re-fit leaves them green; this compares the params against the
BENCH, and is what to rerun after any re-fit.

A lever like the fixture's (a point load at the arm's radius, level at q = 0),
driven by CurrentBasedPositionServo in mode 5 at the righting gains, run as
servo_lift.py ran the bench: hold level at 300 mA, then set Goal Current and
step the goal 20 deg up (lift) or hold it level (fall). The sim's threshold is
the lowest current, 1 mA steps, at which the load moves > 1 deg that way in
1.2 s -- the bench's `moved_deg` and try length.

The bench numbers are copied from docs/plans/righting-servo-model.md ("What the
bench says"), because traces/ is not in git; `servo_lift.py fit` reproduces
them from the captures.

    python analysis/servo_lift_sim.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from aow_sim.build_model import load_params  # noqa: E402
from aow_sim.righting_servo import CurrentBasedPositionServo  # noqa: E402

# (mass g, radius mm, bench lift mA, bench fall mA or None) -- 137 g, not 117.
BENCH = [
    (34, 25, 35, None),
    (34, 44, 49, 17),
    (137, 25, 79, 27),
    (137, 44, 129, 43),
    (236, 44, 189, None),       # one rep, 180-198; fell never tested pre-shear
]
G = 9.81


def lever(mass_g: float, radius_mm: float):
    spec = mujoco.MjSpec()
    spec.option.timestep = 4e-4
    b = spec.worldbody.add_body(name="lever")
    b.add_joint(name="hinge", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, -1, 0])
    b.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.004, 0, 0],
               pos=[radius_mm / 1000, 0, 0], mass=mass_g / 1000)
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def moved(params, mass_g, radius_mm, ma, goal_rad) -> float:
    m, d = lever(mass_g, radius_mm)
    srv = CurrentBasedPositionServo(m, params, joint="hinge", actuator=None,
                                    goal_current=300)

    def go(seconds):
        for _ in range(int(round(seconds / m.opt.timestep))):
            srv.pre_step(d)
            mujoco.mj_step(m, d)
    go(0.8)
    q0 = float(d.qpos[0])
    srv.set_goal_current(ma)
    srv.goal = goal_rad
    go(1.2)
    return math.degrees(float(d.qpos[0]) - q0)


def threshold(params, mass_g, radius_mm, kind: str) -> int | None:
    if kind == "lift":
        for ma in range(0, 400):
            if moved(params, mass_g, radius_mm, ma, math.radians(20)) > 1.0:
                return ma
        return None
    last = None
    for ma in range(0, 300):          # the highest current at which it still falls
        if moved(params, mass_g, radius_mm, ma, 0.0) < -1.0:
            last = ma
        elif last is not None:
            break
    return None if last is None else last + 1


def main() -> int:
    params = load_params()
    print(f"{'load':>16} {'torque':>9} | {'lift sim':>8} {'bench':>6} | "
          f"{'fall sim':>8} {'bench':>6}")
    for mass, r, b_lift, b_fall in BENCH:
        tau = mass / 1000 * G * r / 1000 * 1000
        s_lift = threshold(params, mass, r, "lift")
        s_fall = threshold(params, mass, r, "fall")
        f = lambda v: "--" if v is None else f"{v}"
        print(f"{mass:>5} g at {r:>2} mm {tau:>6.1f} mNm | {f(s_lift):>8} {b_lift:>6} | "
              f"{f(s_fall):>8} {f(b_fall):>6}")
    print("\nfall: the lowest current that HOLDS a load released from level at 300 mA"
          "\n(bench: the midpoint between the last fall and the next hold)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
