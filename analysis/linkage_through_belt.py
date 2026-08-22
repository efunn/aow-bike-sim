"""Can the self-righting four-bar thread the drive belts?

    python analysis/linkage_through_belt.py              # -> analysis/plots/linkage_through_belt.png
    python analysis/linkage_through_belt.py --tag alt    # a variant, at its own name

A FEASIBILITY MAP, not a decision. Everything is read from `aow_sim.cad_layout`
and `analysis.wing_linkage`, so the answer cannot drift from the layout the CAD
is drawn against. Changes nothing — see `analysis/wing_linkage.py` for the rule.

THE QUESTION. The linkage was pushed from a 75 mm fore-aft station to 130 mm
purely to clear the drive belts and then the servo cases. That is 55 mm of
wheelbase spent on clearance. If the belt geometry were different — different
tooth counts, a different belt length, a different servo station — could the
linkage come back?

WHAT "THROUGH THE PULLEYS" ACTUALLY MEANS, because the obvious reading is
wrong. The two input pulleys leave |y| < 22.0 mm completely clear: a 44 mm
corridor straight through the middle, which the linkage already passes through.
The clash is OUTBOARD. The wing pivots sit at |y| = 30.0 mm, inside the belt's
lateral band of 22.0..31.0 mm, so the cranks, couplers and the wing panel cross
that band and have to miss the belt hull in (x, z) while they do.

WHICH MEANS THE TEST IS BANDED, NOT PLANAR. The linkage is planar at ONE
fore/aft station; the belts live in a lateral band. A study done in the (y, z)
view alone reports collisions that lateral separation has already resolved —
which is probably why the numbers recorded at `drive_servo_angle_deg` in
bike_params_cad.yaml ("0 deg collides by 3.9 mm and 45 deg by 37.5") read as
severe. Those numbers are also stale: they predate the belt plane moving
24.0 -> 26.5, the straddle 16.35 -> 15.2563, and the servos reaching
37.372/52.628. The conclusion may well survive; the arithmetic does not.

So every keep-out here carries its own lateral extent, and a linkage point
conflicts only where the two share a band.

THE COUPLED COST, which is the whole reason this is a map and not a number:
shrinking the servo pulley is the main way to open a corridor, and it goes
straight through `belt_ratio`. At 3.0 the drive tops out near 1.06 m/s against
a `control.drive.v_max` of 1.2. Cut the ratio to make room and you buy
clearance with top speed. Every candidate below prints its ratio and its speed
beside its station, or the map lies by omission.

WHAT IT FOUND, 2026-08-21. The expectation written here first was "probably not
at this ratio". That was WRONG, and pleasantly:

  * The belt IS the binding constraint today. Belt-limited minimum station
    123.5 mm; delete the belts entirely and the floor is 109.0, set by
    `drive_mount_side_b` and `drive_mount_radial_out` — the sleeve walls. The
    linkage actually sits at 130, so ~6.5 mm of that is margin, not belt.
  * SHORTER BELT, SAME PULLEYS, FREE. 45T/15T on a 68T (340 mm) belt instead of
    74T (370 mm): station 111.0, ratio and top speed IDENTICAL. 12.5 mm for the
    price of a different belt.
  * BEST AT UNCHANGED RATIO: 36T/12T on a 62T (310 mm) belt. Station 105.0,
    ratio still exactly 3.00, still 1.06 m/s. 18.5 mm won with no drivetrain
    cost at all — smaller pulleys at the same ratio, closer together.
  * BELOW RATIO 3 it keeps buying, then saturates: 2.67 -> 98.0, 2.33 -> 96.0,
    and nothing under 2.33 wins another millimetre. 96 mm is where the moved
    servo cases hit the rear wheel, not where the belt runs out.

So the four-bar cannot go back to 75 mm, but 105 is reachable for nothing and
96 for a third of the top speed. The wall past that is the mount, not the belt.

WHAT THIS MODEL DOES NOT DO, stated because the ranking depends on it. The
servo cases and their mount are TRANSLATED radially with C rather than
re-solved: they keep their size, clocking and tangential straddle. That is
right to first order and it is what stops short-C candidates scoring for
packing they have not paid for, but a real re-solve would move the straddle and
the plate outline too. Anything on the shortlist wants re-deriving through
`cad_layout` before it is believed. Belt tooth counts are also swept as
integers; stocked HTD 5M lengths are a subset, so a winner is a supplier
question, not an order.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml

from aow_sim.cad_layout import (CAD_PARAMS, LINKAGE_CFG, belt_centre_distance,
                                belt_tangent, build, load_params, load_sources)

# Ratio 3.0 tops the drive out here on a 3S average — see `belt_ratio` in
# bike_params_cad.yaml. Speed scales with the ratio (wheel side faster).
V_AT_RATIO_3 = 1.06
V_MAX_CMD = 1.2

# Fore/aft stations to test, mm from the rear axle. 40 is inside the rear
# wheel; 200 is the front axle. The current answer is 130.
STATION_LO, STATION_HI, STATION_STEP = 40.0, 200.0, 0.5

AXLE_H = 51.2       # rear axle above the floor; wing_linkage measures z from
                    # the FLOOR and the layout measures it from the AXLE.

# Validity floors for a candidate drivetrain, independent of the linkage.
CASE_WHEEL_CLEAR = 2.0   # mm between the servo case and the rear wheel
PULLEY_GAP_MIN = 6.0     # mm between the two pulley ENVELOPES — a belt has to
                         # wrap and leave, and touching envelopes is not a drive


# --------------------------------------------------------------------------
# keep-outs, derived from the export


class KeepOut:
    """A solid, as a lateral band plus a footprint in the (x, z) plane.

    The reduction is exact for what is being asked. The linkage occupies ONE
    fore/aft station, so a keep-out can only ever be hit where its lateral
    extent overlaps a linkage point's `y` — and at that point all that matters
    is whether (station, z) is inside the footprint.
    """

    def __init__(self, name, y_lo, y_hi, kind, data):
        self.name, self.y_lo, self.y_hi = name, y_lo, y_hi
        self.kind, self.data = kind, data

    def moved(self, dx, dz):
        """A copy shifted in (x, z). The lateral band is unchanged.

        Needed because the servo cases and their mount are NOT independent of
        the belt: both sit at radius C from the rear axle, so a candidate with
        a different centre distance carries them with it. Holding them still
        while the pulleys move makes every short-C candidate look better than
        it is, which is the one way this map could actively mislead.
        """
        if self.kind == "circle":
            cx, cz, r = self.data
            return KeepOut(self.name, self.y_lo, self.y_hi, "circle",
                           (cx + dx, cz + dz, r))
        return KeepOut(self.name, self.y_lo, self.y_hi, "poly",
                       [(x + dx, z + dz) for x, z in self.data])

    def spans(self, y):
        return self.y_lo - 1e-9 <= y <= self.y_hi + 1e-9

    def contains(self, x, z):
        if self.kind == "circle":
            cx, cz, r = self.data
            return (x - cx) ** 2 + (z - cz) ** 2 <= r * r
        poly = self.data                       # convex, counter-clockwise
        n = len(poly)
        sign = 0
        for i in range(n):
            ax, az = poly[i]
            bx, bz = poly[(i + 1) % n]
            cr = (bx - ax) * (z - az) - (bz - az) * (x - ax)
            if abs(cr) < 1e-12:
                continue
            s = 1 if cr > 0 else -1
            if sign and s != sign:
                return False
            sign = s
        return True


def _hull2d(pts):
    """Convex hull of a small point set — monotone chain, no scipy."""
    pts = sorted(set(map(tuple, np.round(pts, 9))))
    if len(pts) < 3:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2:
                (ax, az), (bx, bz) = out[-2], out[-1]
                if (bx - ax) * (p[1] - az) - (bz - az) * (p[0] - ax) <= 0:
                    out.pop()
                else:
                    break
            out.append(p)
        return out

    return half(pts)[:-1] + half(pts[::-1])[:-1]


def keep_outs(items, groups=("drivetrain", "servos", "mount", "belts")):
    """Every solid in `groups`, as banded (x, z) footprints. Millimetres."""
    out = []
    for it in items:
        if it["group"] not in groups:
            continue
        pos = np.asarray(it["pos"], float) * 1000.0
        if "box" in it:
            e = np.asarray(it["box"], float) * 1000.0
            if "frame" in it:
                # THE ORDERING TRAP, same as the exporter: box[0] runs along
                # frame[1] and box[1] along frame[0]. Getting this backwards
                # here would silently rotate every servo 90 degrees.
                f = [np.asarray(v, float) for v in it["frame"]]
                axes = [f[1] * e[0] / 2, f[0] * e[1] / 2, f[2] * e[2] / 2]
            elif "zaxis" in it:
                h = np.asarray(it["zaxis"], float)
                h = h / np.linalg.norm(h)
                d = np.array([0.0, 1.0, 0.0])
                w = np.cross(d, h)
                axes = [w * e[0] / 2, d * e[1] / 2, h * e[2] / 2]
            else:
                axes = [np.array([e[0] / 2, 0, 0]), np.array([0, e[1] / 2, 0]),
                        np.array([0, 0, e[2] / 2])]
            cor = np.array([pos + sx * axes[0] + sy * axes[1] + sz * axes[2]
                            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            out.append(KeepOut(it["name"], cor[:, 1].min(), cor[:, 1].max(),
                               "poly", _hull2d(cor[:, [0, 2]])))
        elif "cyl" in it or "cap" in it:
            r, ln, ax = it.get("cyl") or it["cap"]
            r, ln = r * 1000.0, ln * 1000.0
            ax = np.asarray(ax, float)
            if abs(ax[1]) > 0.99:                    # lateral axis: a disc in (x, z)
                out.append(KeepOut(it["name"], pos[1] - ln / 2, pos[1] + ln / 2,
                                   "circle", (pos[0], pos[2], r)))
            else:                                     # rare — bound it in a box
                half = ax * ln / 2
                lo, hi = pos - abs(half) - r, pos + abs(half) + r
                out.append(KeepOut(it["name"], lo[1], hi[1], "poly",
                                   [(lo[0], lo[2]), (hi[0], lo[2]),
                                    (hi[0], hi[2]), (lo[0], hi[2])]))
    return out


def belt_keep_outs(teeth_in, teeth_sv, belt_teeth, angle_deg, sep_deg,
                   pitch, width, flange_margin, thickness, plane):
    """The belt-and-pulley keep-out for a CANDIDATE drivetrain, in mm.

    Rebuilt from tooth counts rather than read from the export, because that is
    the whole point of the sweep. Same envelope convention as `cad_layout` —
    pitch diameter plus half the flange margin — so the numbers are comparable.
    """
    d_in = teeth_in * pitch / np.pi
    d_sv = teeth_sv * pitch / np.pi
    try:
        C = belt_centre_distance(belt_teeth * pitch, d_in, d_sv) * 1000.0
    except ValueError:
        # A belt too short to wrap both pulleys. Not an error in the sweep —
        # most of a tooth-count grid is geometrically impossible, and the
        # helper refusing loudly is what keeps a nonsense C out of the map.
        return None, float("nan")
    r_in = (d_in / 2 + flange_margin / 2) * 1000.0
    r_sv = (d_sv / 2 + flange_margin / 2) * 1000.0
    if C <= r_in + r_sv:
        return None, C                       # pulleys overlap: not a drivetrain
    w, t, pl = width * 1000.0, thickness * 1000.0, plane * 1000.0
    y_lo, y_hi = pl - w / 2, pl + w / 2
    ko = []
    for tag, sgn in (("left", 1.0), ("right", -1.0)):
        th = np.radians(angle_deg + (-sep_deg / 2 if tag == "left" else sep_deg / 2))
        c2 = (C * np.cos(th), C * np.sin(th))
        for side in (-1, 1):
            p1, p2, u = belt_tangent(c2, r_in, r_sv, side)
            quad = [p1, p2, p2 + u * t, p1 + u * t]
            # BOTH sides of the bike, and the MIRROR of each onto the other —
            # the symmetric keep-out the drawn belts show, in numbers.
            for lo, hi in ((y_lo, y_hi), (-y_hi, -y_lo)):
                ko.append(KeepOut(f"belt_{tag}_{side}", lo, hi, "poly",
                                  _hull2d(np.array(quad))))
        for lo, hi in ((y_lo, y_hi), (-y_hi, -y_lo)):
            ko.append(KeepOut(f"pulley_sv_{tag}", lo, hi, "circle",
                              (c2[0], c2[1], r_sv)))
    ko.append(KeepOut("pulley_in", -y_hi, y_hi, "circle", (0.0, 0.0, r_in)))
    return ko, C


# --------------------------------------------------------------------------
# the linkage, as points that must clear


def linkage_points(cfg, travel_step=4.0):
    """Every (y, z_from_axle) the mechanism occupies over the whole stroke.

    Sampled from `analysis.wing_linkage.Linkage`, which is the same solver the
    linkage design was optimised with — not a second copy of the kinematics.
    Segments are sampled along their length, so the SWEPT path is covered
    rather than only the joints, which is what the crank_sweep arcs in the
    export exist to make the same point about.
    """
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from wing_linkage import Linkage

    lk = Linkage(cfg)
    pts = []
    travel = float(cfg["stroke"]["servo_travel_deg"])
    for t in np.arange(0.0, travel + travel_step, travel_step):
        for tag in ("left", "right"):
            attach, wing = lk.solve(tag, float(t))
            if attach is None:
                continue
            tip = lk.crank_tip(tag, float(t))
            side = 1 if tag == "left" else -1
            piv = lk.pivot(side)
            segs = [(lk.servo, tip), (tip, attach), (piv, attach)]
            if wing is not None:
                segs.append(lk.wing_line(tag, wing))
            for a, b in segs:
                a, b = np.asarray(a, float), np.asarray(b, float)
                n = max(2, int(np.linalg.norm(b - a) / 2.0))
                for s in np.linspace(0.0, 1.0, n):
                    q = a + s * (b - a)
                    pts.append((q[0], q[1] - AXLE_H))   # floor -> axle datum
    return np.unique(np.round(np.array(pts), 3), axis=0)


def _z_band(ko, x):
    """The z-interval a keep-out occupies on the vertical line at `x`.

    Returns (lo, hi) or None. For a circle it is a chord; for a CONVEX polygon
    the vertical line meets the boundary at most twice, so min/max of the edge
    crossings is exact.
    """
    if ko.kind == "circle":
        cx, cz, r = ko.data
        dx = x - cx
        if abs(dx) >= r:
            return None
        h = np.sqrt(r * r - dx * dx)
        return cz - h, cz + h
    poly = ko.data
    zs = []
    n = len(poly)
    for i in range(n):
        ax, az = poly[i]
        bx, bz = poly[(i + 1) % n]
        if ax == bx:
            if ax == x:
                zs += [az, bz]
            continue
        if min(ax, bx) <= x <= max(ax, bx):
            zs.append(az + (bz - az) * (x - ax) / (bx - ax))
    return (min(zs), max(zs)) if zs else None


def min_station(pts, kos, lo=STATION_LO, hi=STATION_HI, step=STATION_STEP):
    """Smallest fore/aft station at which nothing in `pts` hits anything.

    Scanned per keep-out rather than per point. At a fixed station a keep-out
    occupies ONE z-interval, so the question "does any linkage point fall in
    it" is a binary search over that keep-out's band-filtered z values —
    O(stations x keepouts x log n) instead of a triple loop, which is the
    difference between the sweep taking seconds and taking hours.
    """
    Y, Z = pts[:, 0], pts[:, 1]
    banded = []
    for k in kos:
        zs = np.sort(Z[(Y >= k.y_lo - 1e-9) & (Y <= k.y_hi + 1e-9)])
        if zs.size:
            banded.append((k, zs))
    for x in np.arange(lo, hi + step, step):
        clear = True
        for k, zs in banded:
            iv = _z_band(k, x)
            if iv is None:
                continue
            i0, i1 = np.searchsorted(zs, iv[0], "left"), np.searchsorted(zs, iv[1], "right")
            if i1 > i0:
                clear = False
                break
        if clear:
            return float(x)
    return None


def _blocking(pts, kos, x):
    """Which keep-outs bite at station x — the useful half of a `no`."""
    Y, Z = pts[:, 0], pts[:, 1]
    hit = {}
    for k in kos:
        iv = _z_band(k, x)
        if iv is None:
            continue
        m = ((Y >= k.y_lo - 1e-9) & (Y <= k.y_hi + 1e-9)
             & (Z >= iv[0]) & (Z <= iv[1]))
        if m.any():
            hit[k.name] = hit.get(k.name, 0) + int(m.sum())
    return sorted(hit.items(), key=lambda kv: -kv[1])


# --------------------------------------------------------------------------
# the sweep


def candidates(teeth_in=(12, 14, 15, 16, 18, 20),
               teeth_sv=(20, 24, 28, 32, 36, 40, 45, 50, 56, 60),
               belt_teeth=range(56, 112, 2)):
    """Pulley and belt combinations worth asking about.

    HTD 5M pulley tooth counts are catalogue items, not free reals, and with a
    5 mm pitch every integer belt tooth count IS a 5 mm length increment — but
    stocked lengths are still a subset, so treat a winner as a shortlist entry
    to check against a supplier, not as an order.
    """
    for ti in teeth_in:
        for ts in teeth_sv:
            for bt in belt_teeth:
                yield ti, ts, bt


def run(params, items, cfg, angle_deg=None, sep_deg=None):
    dt = params["drivetrain"]
    be = dt["belt"]
    angle_deg = dt["drive_servo_angle_deg"] if angle_deg is None else angle_deg
    sep_deg = dt["drive_servo_separation_deg"] if sep_deg is None else sep_deg
    plane = abs(next(i for i in items
                     if i["name"] == "pulley_input_left")["pos"][1])
    # Everything that is NOT the belt drive stays fixed: servo cases, mount
    # plates, wheels, axle mounts. Only the belt geometry is swept, so a
    # station that improves does so for the reason under test.
    # Split by whether the part rides on the servo station or not. The cases
    # and their mount move with C; the wheel and the axle mounts do not.
    fixed_moves = keep_outs(items, groups=("servos", "mount"))
    fixed_moves = [k for k in fixed_moves if "steer" not in k.name]
    fixed_still = [k for k in keep_outs(items, groups=("drivetrain",))
                   if not k.name.startswith("pulley_")]
    fixed_still += [k for k in keep_outs(items, groups=("servos",))
                    if "steer" in k.name]
    fixed = fixed_moves + fixed_still
    pts = linkage_points(cfg)

    # The as-built centre distance, so a candidate's C can be expressed as a
    # radial displacement of the servo station rather than an absolute.
    ow = params["omni_wheel"]
    d4 = params["servos"]["xc430_w150"]
    wheel_r = ow["outer_radius"] * 1000.0
    case_h = d4["box_size"][2] * 1000.0
    shaft_end = d4["shaft_from_end"] * 1000.0

    def r_in_env(t):
        return (t * be["pitch"] / np.pi / 2 + be["flange_margin"] / 2) * 1000.0

    r_sv_env = r_in_env

    C0 = belt_centre_distance(be["length"],
                              be["teeth_input"] * be["pitch"] / np.pi,
                              be["teeth_servo"] * be["pitch"] / np.pi) * 1000.0
    th0 = np.radians(angle_deg)
    rad = np.array([np.cos(th0), np.sin(th0)])

    rows = []
    for ti, ts, bt in candidates():
        ko, C = belt_keep_outs(ti, ts, bt, angle_deg, sep_deg, be["pitch"],
                               be["width"], be["flange_margin"],
                               be["thickness"], plane)
        if ko is None:
            continue
        ratio = ts / ti
        # VALIDITY BEFORE CLEARANCE. Pulling the servos in toward the axle is
        # what wins station, and taken far enough it buries the cases inside
        # the rear wheel — which scores beautifully against the linkage and
        # cannot be built. The case's radially-inner face has to stay outside
        # the wheel, and the two pulley envelopes have to leave room for a belt
        # to actually wrap. Without these the map recommends nonsense.
        rc = C - (case_h / 2 - shaft_end)
        if rc - case_h / 2 < wheel_r + CASE_WHEEL_CLEAR:
            continue
        if C - (r_in_env(ti) + r_sv_env(ts)) < PULLEY_GAP_MIN:
            continue
        # Carry the cases and the mount out along the 45 deg radial with C.
        # A translation, not a re-solve: the cases keep their size, clocking
        # and tangential straddle, which is right to first order and is what
        # makes a short-C candidate pay for the packing it actually moves.
        d = (C - C0) * rad
        moved = [k.moved(d[0], d[1]) for k in fixed_moves] + fixed_still
        st = min_station(pts, moved + ko)
        rows.append({"teeth_in": ti, "teeth_sv": ts, "belt_teeth": bt,
                     "ratio": ratio, "C": C, "station": st,
                     "v": V_AT_RATIO_3 * ratio / 3.0})
    return rows, pts, fixed


def report(rows, base):
    ok = [r for r in rows if r["station"] is not None]
    print(f"{len(rows)} candidates, {len(ok)} with a feasible station\n")
    print(f"BASELINE (as built): {base['teeth_sv']}T/{base['teeth_in']}T on a "
          f"{base['belt_teeth']}T belt, ratio {base['ratio']:.2f}, "
          f"C {base['C']:.2f} mm")
    print(f"  station {base['station']}, v_max {base['v']:.2f} m/s "
          f"(commanded ceiling {V_MAX_CMD})\n")
    # Only candidates that BEAT the baseline station are interesting, and only
    # if they keep enough ratio to be a drivetrain.
    better = sorted((r for r in ok if base["station"] is not None
                     and r["station"] < base["station"] - 1e-9),
                    key=lambda r: (r["station"], -r["ratio"]))
    if not better:
        print("NOTHING beats the baseline station. The belt is not what is "
              "holding the linkage forward.")
        return better
    # The decision-relevant cut: how much station is available WITHOUT giving
    # up drive. A list sorted by station alone hides that the top of it costs
    # half the top speed.
    print("best station at or above each ratio floor:")
    print(f"{'ratio>=':>8} {'station':>8} {'won':>6} {'ratio':>6} {'v':>5} "
          f"{'sv/in':>8} {'belt':>5} {'C mm':>7}")
    for floor in (3.0, 2.5, 2.25, 2.0, 1.0):
        c = [r for r in ok if r["ratio"] >= floor - 1e-9]
        if not c:
            continue
        b = min(c, key=lambda r: r["station"])
        print(f"{floor:8.2f} {b['station']:8.1f} "
              f"{base['station'] - b['station']:6.1f} {b['ratio']:6.2f} "
              f"{b['v']:5.2f} {str(b['teeth_sv']) + '/' + str(b['teeth_in']):>8} "
              f"{b['belt_teeth']:5d} {b['C']:7.2f}")
    same = sorted((r for r in ok if r["teeth_sv"] == base["teeth_sv"]
                   and r["teeth_in"] == base["teeth_in"]),
                  key=lambda r: r["station"])
    if same:
        b = same[0]
        print(f"\nsame pulleys, belt length alone: {b['belt_teeth']}T "
              f"({b['belt_teeth'] * 5} mm) -> station {b['station']:.1f} "
              f"(won {base['station'] - b['station']:.1f}), ratio unchanged")
    print(f"\nall candidates beating the baseline station:")
    print(f"{'station':>8} {'ratio':>6} {'v m/s':>6} {'sv':>4} {'in':>4} "
          f"{'belt':>5} {'C mm':>7}")
    seen = set()
    for r in better:
        key = round(r["station"], 1)
        if key in seen:
            continue
        seen.add(key)
        print(f"{r['station']:8.1f} {r['ratio']:6.2f} {r['v']:6.2f} "
              f"{r['teeth_sv']:4d} {r['teeth_in']:4d} {r['belt_teeth']:5d} "
              f"{r['C']:7.2f}")
    return better


def plot(rows, base, out: Path):
    ok = [r for r in rows if r["station"] is not None]
    fig, ax = plt.subplots(figsize=(9, 6))
    if ok:
        sc = ax.scatter([r["ratio"] for r in ok], [r["station"] for r in ok],
                        c=[r["v"] for r in ok], cmap="viridis", s=14,
                        alpha=0.75, edgecolors="none")
        fig.colorbar(sc, ax=ax, label="top speed [m/s], 3S average")
    if base["station"] is not None:
        ax.axhline(base["station"], color="crimson", lw=1.2, ls="--",
                   label=f"as built: {base['station']:.0f} mm @ ratio "
                         f"{base['ratio']:.2f}")
        ax.plot([base["ratio"]], [base["station"]], "*", ms=18,
                color="crimson", zorder=5)
    ax.axvspan(0, V_MAX_CMD / V_AT_RATIO_3 * 3.0, color="0.9", zorder=0)
    ax.axvline(V_MAX_CMD / V_AT_RATIO_3 * 3.0, color="0.4", lw=1.0,
               label=f"ratio for the commanded {V_MAX_CMD} m/s ceiling")
    ax.set_xlabel("belt ratio  (teeth_servo / teeth_input) — higher is faster, "
                  "weaker")
    ax.set_ylabel("minimum linkage station [mm from the rear axle]\nlower is "
                  "wheelbase won back")
    ax.set_title("Can the four-bar thread the belts?\n"
                 "minimum fore/aft station vs belt ratio, banded 3D clearance")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    print(f"\nwrote {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", default=CAD_PARAMS)
    ap.add_argument("--linkage-config", default=LINKAGE_CFG)
    ap.add_argument("--tag", default="", help="suffix for the output name, so a "
                    "variant lands at its own path instead of overwriting")
    args = ap.parse_args()

    params = load_params(args.params)
    raw = load_sources(args.params)
    items = build(params, raw, "linkage", False, False, args.linkage_config)
    cfg = yaml.safe_load(Path(args.linkage_config).read_text())

    rows, pts, fixed = run(params, items, cfg)
    be = params["drivetrain"]["belt"]
    bi, bs = int(be["teeth_input"]), int(be["teeth_servo"])
    bbt = int(round(be["length"] / be["pitch"]))
    base = next((r for r in rows if r["teeth_in"] == bi and r["teeth_sv"] == bs
                 and r["belt_teeth"] == bbt), None)
    if base is None:
        ko, C = belt_keep_outs(bi, bs, bbt, params["drivetrain"]["drive_servo_angle_deg"],
                               params["drivetrain"]["drive_servo_separation_deg"],
                               be["pitch"], be["width"], be["flange_margin"],
                               be["thickness"],
                               abs(next(i for i in items
                                        if i["name"] == "pulley_input_left")["pos"][1]))
        base = {"teeth_in": bi, "teeth_sv": bs, "belt_teeth": bbt,
                "ratio": bs / bi, "C": C, "station": min_station(pts, fixed + ko),
                "v": V_AT_RATIO_3 * (bs / bi) / 3.0}
    report(rows, base)
    if base["station"] is not None:
        print("\nWhat bites at the current 130 mm station, if anything:")
        ko, _ = belt_keep_outs(bi, bs, bbt,
                               params["drivetrain"]["drive_servo_angle_deg"],
                               params["drivetrain"]["drive_servo_separation_deg"],
                               be["pitch"], be["width"], be["flange_margin"],
                               be["thickness"],
                               abs(next(i for i in items
                                        if i["name"] == "pulley_input_left")["pos"][1]))
        for name, n in _blocking(pts, fixed + ko, 130.0)[:6]:
            print(f"  {name:32} {n} sampled points inside")
    stem = f"linkage_through_belt{('_' + args.tag) if args.tag else ''}"
    plot(rows, base, Path(__file__).resolve().parent / "plots" / f"{stem}.png")


if __name__ == "__main__":
    main()
