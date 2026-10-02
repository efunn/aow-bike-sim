"""What the two bench tests you can already do would pin down in `solref`.

`sim.contact_solref` is a PAIR, and the two numbers are NOT separately
identifiable. An earlier version of this file claimed they were, one
experiment each; that was wrong, and the tables it printed were wrong with it.

What MuJoCo actually does with a POSITIVE solref = (timeconst, dampratio)
(see the solver-parameter section of its modeling docs):

    b = 2 / (d_width * timeconst)                      <- damping
    k = d(r) / (d_width^2 * timeconst^2 * dampratio^2) <- stiffness

Read those carefully, because the names mislead:

  timeconst  enters BOTH. It is the only thing that sets damping, and it
             also sets stiffness (as 1/timeconst^2).
  dampratio  sets STIFFNESS ONLY, as 1/dampratio^2. It does not appear in
             `b` at all.

So `dampratio` is not a damping coefficient — it is the ratio of the actual
damping to the critical damping FOR THE RESULTING STIFFNESS. Lowering it from
1.0 to 0.5 at fixed timeconst leaves damping untouched and makes the contact
FOUR TIMES STIFFER, which is what makes it underdamped and bouncy. Verified
against this model, the rear wheel at rest on the bike's own weight (settled,
2026-10-01): the sink falls 3.9x going 1.0 -> 0.5 and 10.8x going 1.0 -> 0.3,
against the 4x and 11.1x the formula predicts. Away from rest the ratio
shrinks (2.2x at 44 N), because `solimp` stiffens the contact with depth.

CONSEQUENCE FOR THE BENCH TESTS. A static load-deflection reading constrains
the PRODUCT `timeconst * dampratio`, not timeconst alone, so it cannot fix
either number by itself. The static and drop tests have to be solved jointly.
Concretely, a 4.5 kg (44.1 N) reading of "about 1 mm" on the rear implies
timeconst ~0.009 at dampratio 1.0 and ~0.018 at 0.5 (settled, 2026-10-01) --
a factor of two in the answer, from an assumption rather than a measurement.
(The posed curve this file used to print said ~0.0035 at 1.0.)

If you want the two decoupled, use the NEGATIVE convention: MuJoCo reads a
negative solref as (-stiffness, -damping) directly, and its own docs
recommend that form for system identification. Then a static test gives
stiffness, a drop test gives damping, and neither contaminates the other.

NEITHER FORM IS A STIFFNESS IN N/m. In both, the settled sink at a given
force scales with the masses compiled around the contact (`body_invweight0`):
on a wheel-on-a-slide rig the front tire sank 0.447 / 0.072 / 0.010 mm at
44 N with a 0.01 / 0.5 / 4 kg carriage, and the negative form 0.660 / 0.142
/ 0.020 (analysis/static_contact_sim.py, 2026-10-01). So fit on THE BIKE's
model, and refit whenever its masses change: doubling the GUESS chassis mass
took the rear load +42% but its sink only +31%.

  python analysis/contact_calibration.py
  python analysis/contact_calibration.py --load-kg 4.5 --drop-mm 35

WHAT THE MODEL SAYS NOW. dampratio is 1.0, which is CRITICAL damping, and a
critically damped contact cannot bounce. A wheel that audibly bounces two or
three times is under-damped, so if the bench test bounces and the model does
not, dampratio is the parameter that is wrong -- not timeconst, and not the
friction coefficients.

METHOD, static. A settling simulation of the whole bike (`static_curve`):
held upright with height, pitch and fore-aft free, wheels locked at a phase,
a vertical force on the axle until that wheel carries the load, then stepped
to rest. Sink = how far the wheel's lowest point is below the floor.
Comparable to a weight on the axle and a dial on it.

It used to place the bike at prescribed heights and read the force from one
mj_forward. That is not a resting state: it agreed at the wheel's own load
and over-read the sink above it, ~3.4x at 44 N. Every sink printed before
2026-10-01 is from that version.

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


def _wheel_normal_force(model, data, geoms, floor):
    """Total vertical contact force between the floor and `geoms` [N]."""
    f, buf = 0.0, np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if floor not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == floor else c.geom1
        if other not in geoms:
            continue
        mujoco.mj_contactForce(model, data, i, buf)
        # buf[0] is along the contact normal, which for the floor is +z
        f += float(buf[0]) * abs(c.frame[2])
    return f


SETTLE_MIN_S = 0.1        # an overdamped soft contact creeps; don't stop early
SETTLE_MAX_S = 3.0
SETTLE_TOL_MM = 5e-4      # sink change over the last SETTLE_WINDOW_S
SETTLE_WINDOW_S = 0.02
LOAD_TOL_N = 0.02         # static_curve: how close the wheel's load must come
LOAD_ROUNDS = 8


def _settle(m, d, verts):
    """Step until the sink stops moving. Height, pitch and fore-aft are free.

    After every step the chassis' lateral, roll and yaw, and every joint
    below it (steer, wheel spin, hub, rollers, inputs), are put back where
    they started, position AND velocity. That holds the bike upright with the
    hub at its phase. Zeroing velocity alone is not enough: each step still
    moves the position by ~dt^2 * accel, and on a sloped part of a roller the
    hub crept 4 deg in 3 s.

    Fore-aft must stay FREE. With the wheels locked, a held x lets the floor
    push sideways on a wheel with nothing to balance it, and that push at
    floor level pitches the bike and moves load between the wheels. Held, the
    rear's share at rest ran 5.8 -> 9.2 N across roller phases (5.4 N of
    sideways push at the two-small-ends phase); free, 5.8 -> 6.4 N
    (2026-10-01). The per-phase sinks at 44 N moved by <= 0.006 mm, because
    the push is corrected to the target load anyway.

    Editing the state leaves the compiled masses alone, and those matter
    here: see `static_curve`. Returns the time taken [s].
    """
    hold = d.qpos.copy()
    assert abs(hold[4]) < 1e-9 and abs(hold[6]) < 1e-9, "start pose must be roll/yaw free"
    window = max(1, int(SETTLE_WINDOW_S / m.opt.timestep))
    hist = []
    t0 = d.time
    while d.time - t0 < SETTLE_MAX_S:
        mujoco.mj_step(m, d)
        pitch = 2.0 * np.arctan2(d.qpos[5], d.qpos[3])
        d.qpos[1] = hold[1]
        d.qpos[3:7] = [np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0]
        d.qpos[7:] = hold[7:]
        d.qvel[[1, 3, 5]] = 0.0              # free joint: keep vx, vz, pitch rate
        d.qvel[6:] = 0.0
        hist.append(clearance_mm(d, verts))
        if (d.time - t0 > SETTLE_MIN_S
                and abs(hist[-1] - hist[-1 - window]) < SETTLE_TOL_MM):
            return d.time - t0
    raise RuntimeError(f"no rest within {SETTLE_MAX_S} s")


def _axle_from_down(m, d):
    """Rear: angle of the roller axle nearest straight down [deg]."""
    hub = d.xpos[m.body("aow_hub").id]
    angles = []
    for i in range(8):
        v = d.xpos[m.body(f"roller_axle_{i}").id] - hub
        angles.append(np.degrees(np.arctan2(v[0], -v[2])))
    return min(angles, key=abs)


def static_curve(timeconst, dampratio, loads_n, wheel="rear", axle_deg=None):
    """(sink mm, normal force N) of one wheel, the whole bike settled at rest.

    The bike starts from `settle_upright` and a vertical force goes on the
    axle (the hub body, or the front wheel's) until that wheel carries each
    load; then `_settle`. A load at or below the wheel's own share (or
    `None`) gets no push: that row is the bike resting on its own weight.
    Lifting toward zero was tried (2026-10-01) and does not converge at the
    small-end phases: the contact jumps between points as the bike pitches.
    Where it did converge it mattered: zeroed at 1.2 N rather than 9.5 N,
    the sink added by 26.7 N went 0.267 -> 0.302 mm with two big ends down
    and 0.239 -> 0.363 mm flat on one roller. The contact is softest near
    zero depth (`solimp`), so a comparison with a bench reading has to start
    from the bench's zero.

    `axle_deg` puts the rear's nearest roller axle that far from straight
    down, turning the hub and the belt inputs 1:1 (the tendons need both)
    before settling.

    It must be the WHOLE bike: in MuJoCo the settled sink at a given force
    depends on the masses compiled around the contact (`body_invweight0`),
    so a wheel on a rig of other masses gives other numbers. Measured
    2026-10-01 with analysis/static_contact_sim.py: at 44 N the front tire
    sank 0.447 / 0.072 / 0.010 mm with a 0.01 / 0.5 / 4 kg carriage.

    This replaced a posed version: lower the bike by dz, one mj_forward,
    read the force. That is not a resting state, so it matched rest only at
    the wheel's own load and over-read the sink above it: rear ~1.95 mm at
    44 N, where settled gives ~0.45. Every sink this file printed before
    2026-10-01 came from the posed version.
    """
    m = _model(timeconst, dampratio)
    rest = settle_upright(m)
    rear, floor = _rear(m)
    geoms = rear if wheel == "rear" else {m.geom("front_tire").id}
    body = m.body("aow_hub" if wheel == "rear" else "front_wheel").id
    verts = wheel_vertices(m, geoms)
    out = []
    for load in loads_n:
        d = mujoco.MjData(m)
        d.qpos[:] = rest.qpos
        if axle_deg is not None:
            mujoco.mj_kinematics(m, d)
            turn = _axle_from_down(m, d) - axle_deg     # +hub moves the axle -
            for j in ("hub_spin", "input_a_spin", "input_b_spin"):
                d.qpos[m.jnt_qposadr[m.joint(j).id]] += np.radians(turn)
        mujoco.mj_forward(m, d)
        _settle(m, d, verts)
        own = _wheel_normal_force(m, d, geoms, floor)
        if load is not None and load > own:
            # the push also moves pitch, and the contact points with it, so
            # the share shifts: correct until the wheel carries the load.
            err = load - own
            for _ in range(LOAD_ROUNDS):
                d.xfrc_applied[body, 2] -= err
                _settle(m, d, verts)
                err = load - _wheel_normal_force(m, d, geoms, floor)
                if abs(err) < LOAD_TOL_N:
                    break
            else:
                raise RuntimeError(f"{wheel} load {load:.2f} N: still {err:+.3f} N off")
        out.append((-clearance_mm(d, verts), _wheel_normal_force(m, d, geoms, floor)))
    return np.array(out)


def rest_sink(timeconst, dampratio, wheel="rear"):
    """(sink mm, load N) of one wheel with the bike resting on its own weight."""
    return tuple(static_curve(timeconst, dampratio, [None], wheel)[0])


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
    load_n = args.load_kg * G
    print(f"config now: contact_solref {cur}")
    print(f"bench load {args.load_kg} kg = {load_n:.1f} N on the rear axle\n")

    print("STATIC  rear wheel, whole bike settled (see static_curve)")
    print(f"{'timeconst':>10}{'at rest':>16}{'own load':>10}{'sink @ 2x':>11}"
          f"{'sink @ bench':>14}{'load/sink':>11}")
    print(f"{'[s]':>10}{'[mm]':>16}{'[N]':>10}{'[mm]':>11}{'[mm]':>14}{'[N/mm]':>11}")
    # At the CONFIG's dampratio, not a hardcoded 1.0: a literal 1.0 once
    # printed a table for a contact 4x softer than the one the model runs,
    # and a "ruled out" conclusion came straight off it.
    for tc in args.timeconsts:
        rest, own = rest_sink(tc, cur[1])
        c = static_curve(tc, cur[1], [2 * own, load_n])
        d2, db = c[0, 0], c[1, 0]
        k = load_n / db if db > 0 else float("nan")
        star = "  <- current" if abs(tc - cur[0]) < 1e-9 else ""
        print(f"{tc:>10.4f}{rest:>16.3f}{own:>10.2f}{d2:>11.3f}{db:>14.3f}{k:>11.1f}{star}")

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

    print("\nHOW TO USE THIS. The static column is printed at the CONFIG's\n"
          f"dampratio ({cur[1]}), because stiffness goes as 1/dampratio^2 --\n"
          "the two are NOT independent and a static reading alone fixes only\n"
          "the product. Match the static column AND the drop column together,\n"
          "or switch to a negative solref (-stiffness, -damping), which is\n"
          "what MuJoCo recommends for system ID and does decouple them.\n"
          "Either way, fit on THIS bike's model: neither form is a stiffness\n"
          "in N/m, the sink also scales with the compiled masses.\n"
          "See this file's header for the formulas and the measured check.")


if __name__ == "__main__":
    main()
