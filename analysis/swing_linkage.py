"""The SWING linkage study: a co-rotating four-bar pair on one servo.

Sibling of analysis/wing_linkage.py, and the difference is the coupling. The
mirrored linkage deploys BOTH wings outward together, so the stroke never has
to know which side the bike fell on. This one CO-ROTATES: one crank body with
two arms keyed to the same shaft, so turning it puts one wing down and lifts
the other. Turn the other way and they swap.

    python analysis/swing_linkage.py                # labelled rest geometry
    python analysis/swing_linkage.py --sweep        # reach + ratio vs crank
    python analysis/swing_linkage.py --video        # deploy left/right/left
    python analysis/swing_linkage.py --righting --video
    python analysis/swing_linkage.py --optimize --save config/swing_opt.yaml

Everything is the 2D front/back view: (y, z) in millimetres, from the FLOOR and
the CENTRELINE, +y to the bike's left. Reads config/swing_linkage.yaml and
writes analysis/plots/. Deliberately outside bike_params.yaml and the params
digest, exactly like the mirrored study.

WHAT IT BUYS, AND WHAT IT COSTS.

  + One flat face on ONE side, with the other tucked up. The mirrored pair
    cannot do this: deploying far enough to reach a ball on the right plants
    the left wing too, and the bike becomes a four-point stance it cannot fall
    out of -- measured at a frozen -0.17 deg roll, which is a parking brake,
    not balance.
  + The far wing CANNOT reach the chassis. On a bare hinge pair this is the
    binding constraint and has to be enforced with a stroke limit and explicit
    contact pairs; here the four-bar's rocker arc does it geometrically.
  - It must be told which side to go. The mirrored pair's side-agnosticism is
    given up, and the rule is not obvious -- see the 90 deg sign flip in
    config/swing_wings.yaml.

SYMMETRY IS BY CONSTRUCTION, NOT BY SEARCH. One crank length, one coupler, one
rocker, shared by both sides. The mirrored study carries per-side first links
and derives per-side couplers, because its asymmetry is useful. Here it would
be a defect: a co-rotating pair rests in the MIDDLE of its range, and a rest
pose that is not laterally symmetric is a standing roll bias the balance
controller trims out forever. Sharing the lengths makes an asymmetric rest pose
unrepresentable rather than something to check for afterwards.

That also halves the work: side L at crank -t is exactly side R at crank +t, so
every metric here characterises ONE side and mirrors it. `--check` asserts it.

THE FOUR BAR, per side, in the same order the mirrored study names them:

    link 1  servo crank   shaft -> crank tip. TWO arms on one rigid body,
                          `angle_between_cranks` apart.
    link 2  coupler       crank tip -> rocker joint.
    link 3  rocker        joint -> wing pivot. Rigid with the wing panel,
                          which hangs off it at `wing_angle_from_rocker`.
    link 4  ground        wing pivot -> shaft. Virtual/fixed.

Traced from the `swing-wings-geom-mock` sketch; `--check` re-verifies the
kinematics against the as-drawn points so this file and the CAD cannot drift.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config" / "swing_linkage.yaml"

GRAVITY = 9.81
BIKE_MASS_KG = 1.159      # same figure the mirrored study uses, so the two
COM_Z_MM = 123.5          # torque numbers are comparable
BALL_R_MM = 33.5          # road hockey ball radius. REFERENCE ONLY -- this
                          # mechanism does not strike the ball (fixed panels do
                          # that); see BRACE_ARM_MM.
RECOVERABLE_DEG = 30.9    # analysis/no_return.py: the largest lean the policy
                          # can still save. Anything the mechanism drags on the
                          # floor INSIDE this angle is a wing fouling a save.

_C = {"right": "#d62728", "left": "#1f77b4"}

# As-drawn points from the sketch, for `--check`. These are the +y side, which
# by the repo's convention is the bike's LEFT -- worth stating, because reading
# them as "right" mirrors the crank arms and reproduces a plausible-looking
# mechanism that is not the drawn one. That is exactly how the first pass here
# went wrong, twice.
# STALE as of 2026-08-25: these are points from the older
# `swing-wings-geom-mock` sketch. The live geometry is `wing-linkage-straight`,
# whose driving variables config/swing_linkage.yaml now carries, but whose
# POINT coordinates have not been re-read. So `--check`'s point comparison will
# report MISMATCH; the symmetry half of that check is unaffected and still
# meaningful. Re-trace before relying on the point half again.
_SKETCH = {"joint": (50.62, 30.27), "foot": (44.21, 10.68), "top": (97.40, 95.36)}


def _plots_dir():
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _rot(v, deg):
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


# --------------------------------------------------------------------------
# geometry


class SwingLinkage:
    """One bike's worth of co-rotating geometry.

    `side` is +1 for the bike's LEFT wing (+y) and -1 for the RIGHT (-y),
    matching the convention everywhere else in the repo.

    Crank travel is SIGNED and measured from the rest pose: POSITIVE deploys
    the right (-y) wing, negative the left. There is no separate "stow" -- rest
    is the middle of the range, which is the whole character of the mechanism.
    """

    def __init__(self, cfg: dict):
        b, m = cfg["bike"], cfg["mechanism"]
        self.cfg = cfg
        self.wheel_radius = b["wheel_radius"]
        self.half_span = b["bike_width"] / 2.0
        self.bike_height = b["bike_height"]
        # Sketch v=0 sits at the wheel radius, so everything here is in sketch
        # coordinates and `z_floor()` is the only place that converts.
        self.shaft = np.array([0.0, m["servo_offset"]])
        self.pivot_x = m["wing_pivot_x"]
        self.pivot_z = m["wing_pivot_z"]
        self.crank = m["crank_length"]
        self.coupler = m["coupler_length"]
        self.rocker = m["rocker_length"]
        self.between = m["angle_between_cranks"]
        self.wing_from_rocker = m["wing_angle_from_rocker"]
        self.wing_norm = m["wing_norm_offset"]
        self.wing_z_max = m["wing_z_max"]
        # DERIVED, not configured: the panel's lower edge sits at
        # `bike.ground_clearance` in the rest pose. Same convention as
        # wing_linkage's `wing_bottom = ground_clearance` -- it is the lowest
        # the panel may hang while the bike is driving, so it is a property of
        # the bike, not something to search over. `wing_z_min` in the config is
        # kept as the as-drawn value for reference and is ignored here.
        self.wing_z_min = m["wing_z_min"]          # provisional, for rest_joint
        self._derive_panel_bottom(b["ground_clearance"])
        # Branch memory, as in the mirrored study: a four-bar that flips branch
        # has physically come apart, so every solve is continued from the last.
        self._last: dict[int, np.ndarray] = {}
        self._limit: float | None = None      # assembly_limit memo; geometry
                                              # is immutable after __init__
        self._walk_memo: dict[float, list] = {}
        self.reset()

    def _derive_panel_bottom(self, ground_clearance: float) -> None:
        """Set `wing_z_min` so the panel's lower edge sits at ground clearance.

        Solved at the REST pose, where the panel is symmetric. The panel origin
        and direction come from the rocker, so this is one linear solve rather
        than a search.
        """
        joint = self.rest_joint(-1)
        p = self.pivot(-1)
        r = joint - p
        rdir = r / float(np.linalg.norm(r))
        wa = np.arctan2(rdir[1], rdir[0]) - np.deg2rad(self.wing_from_rocker)
        w = np.array([np.cos(wa), np.sin(wa)])
        n = np.array([w[1], -w[0]])
        origin = joint - self.wing_norm * n
        want = ground_clearance - self.wheel_radius       # sketch frame
        if abs(w[1]) < 1e-6:
            return                                        # panel horizontal
        self.wing_z_min = float((want - origin[1]) / w[1])

    # -- fixed points ------------------------------------------------------

    def z_floor(self, v: float) -> float:
        """Sketch height -> height above the FLOOR [mm]."""
        return v + self.wheel_radius

    def pivot(self, side: int) -> np.ndarray:
        return np.array([side * self.pivot_x, self.pivot_z])

    def crank_tip(self, side: int, travel_deg: float) -> np.ndarray:
        """Crank tip for one arm.

        Both arms are on ONE body `between` apart, so a single `travel_deg`
        moves both. The right arm (side -1) sits at +between/2 from vertical
        and the left at -between/2, which is what makes rest symmetric.
        """
        base = 90.0 - side * self.between / 2.0
        a = np.deg2rad(base + travel_deg)
        return self.shaft + self.crank * np.array([np.cos(a), np.sin(a)])

    def rest_joint(self, side: int) -> np.ndarray:
        """The rocker joint at rest, used to seed the assembly branch."""
        p = self.pivot(side)
        c = self.crank_tip(side, 0.0)
        d = c - p
        L = float(np.linalg.norm(d))
        a = (self.rocker**2 - self.coupler**2 + L**2) / (2 * L)
        h = np.sqrt(max(self.rocker**2 - a**2, 0.0))
        base = p + a * d / L
        perp = np.array([-d[1], d[0]]) / L
        # Outboard branch: the one further from the centreline is the assembled
        # pose in the sketch, and the inboard one folds the rocker through the
        # chassis.
        return max([base + h * perp, base - h * perp], key=lambda q: abs(q[0]))

    def reset(self) -> None:
        """Forget the branch memory. Call before any sweep that does not start
        at rest, or the first solve continues from wherever the last one left
        off and the branch choice silently changes."""
        self._last = {s: self.rest_joint(s) for s in (-1, 1)}

    # -- the moving pose ---------------------------------------------------

    def solve(self, side: int, travel_deg: float):
        """Rocker joint and wing rotation for a given crank travel.

        Returns (joint, wing_deg) or (None, None) when the loop cannot close --
        which is a real answer, not an error: it is the assembly limit, and the
        stroke has to stop before it.
        """
        p = self.pivot(side)
        c = self.crank_tip(side, travel_deg)
        d = c - p
        L = float(np.linalg.norm(d))
        if L > self.rocker + self.coupler or L < abs(self.rocker - self.coupler) or L == 0:
            return None, None
        a = (self.rocker**2 - self.coupler**2 + L**2) / (2 * L)
        h2 = self.rocker**2 - a**2
        if h2 < 0:
            return None, None
        h = np.sqrt(h2)
        base = p + a * d / L
        perp = np.array([-d[1], d[0]]) / L
        # NOT `self._last.get(side, self.rest_joint(side))`: Python evaluates a
        # .get default EAGERLY, so that form ran a full circle-circle solve on
        # every call even when the key was present -- 4590 wasted rest_joint
        # calls per objective evaluation.
        ref = self._last[side] if side in self._last else self.rest_joint(side)
        joint = min([base + h * perp, base - h * perp],
                    key=lambda q: float(np.linalg.norm(q - ref)))
        self._last[side] = joint
        v0 = self.rest_joint(side) - p
        v1 = joint - p
        ang = np.degrees(np.arctan2(v1[1], v1[0]) - np.arctan2(v0[1], v0[0]))
        return joint, float((ang + 180) % 360 - 180)

    def wing_points(self, side: int, joint: np.ndarray):
        """(foot, top) of the wing panel, in sketch coordinates.

        The panel is rigid with the rocker: it runs at `wing_angle_from_rocker`
        off the rocker's bearing, offset `wing_norm_offset` normal to it, and
        spans `wing_z_min`..`wing_z_max` along its own axis. The FOOT is the
        z_min end -- the one that reaches the ground.
        """
        p = self.pivot(side)
        r = joint - p
        rdir = r / float(np.linalg.norm(r))
        wa = np.arctan2(rdir[1], rdir[0]) + side * np.deg2rad(self.wing_from_rocker)
        w = np.array([np.cos(wa), np.sin(wa)])
        n = np.array([w[1], -w[0]])
        origin = joint + side * self.wing_norm * n
        return origin + self.wing_z_min * w, origin + self.wing_z_max * w

    def pose(self, side: int, travel_deg: float):
        """Everything about one side at one crank angle, or None."""
        joint, wd = self.solve(side, travel_deg)
        if joint is None:
            return None
        foot, top = self.wing_points(side, joint)
        return {"joint": joint, "wing_deg": wd, "foot": foot, "top": top,
                "crank_tip": self.crank_tip(side, travel_deg),
                "pivot": self.pivot(side)}


# --------------------------------------------------------------------------
# kinematic evaluation


def sweep(lk: SwingLinkage, travel: float, step: float = 1.0):
    """Both sides across a signed crank sweep, rest outward.

    Walks OUT FROM REST in each direction rather than sweeping straight
    through, because the branch memory has to be continued from the assembled
    pose -- starting at one end and marching across passes through rest with a
    branch chosen at the far limit, which is how a four-bar study silently
    reports the folded-through solution.
    """
    ts = np.concatenate([np.arange(0.0, -travel - step, -step)[::-1],
                         np.arange(step, travel + step, step)])
    out: dict[float, dict] = {}
    for direction in (1.0, -1.0):
        lk.reset()
        t = 0.0
        while t * direction <= travel + 1e-9:
            row = {}
            ok = True
            for side in (-1, 1):
                pz = lk.pose(side, t)
                if pz is None:
                    ok = False
                    break
                row[side] = pz
            if not ok:
                break
            out[round(t, 6)] = row
            t += direction * step
    return [t for t in sorted(out) if abs(t) <= travel + 1e-9], out


def assembly_limit(lk: SwingLinkage, cap: float = 200.0, step: float = 1.0) -> float:
    """Largest |crank travel| at which BOTH sides still close [deg].

    Memoised: it walks 400 poses and was being recomputed several times per
    objective evaluation by callers that each wanted the stroke bound.
    """
    if lk._limit is not None:
        return lk._limit

    def closes(t: float) -> bool:
        return all(lk.pose(side, sgn * t) is not None
                   for side in (-1, 1) for sgn in (1, -1))

    # COARSE first. Most candidates here are crank-rockers whose crank turns
    # all the way, so the fine walk paid 400 poses to discover nothing. Step
    # out at 5 deg, then refine the one interval that actually broke.
    lk.reset()
    coarse = 5.0
    t, last_ok = coarse, 0.0
    while t <= cap:
        if not closes(t):
            break
        last_ok = t
        t += coarse
    else:
        lk._limit = cap
        return cap
    lk.reset()
    f = last_ok
    while f + step <= min(t, cap) and closes(f + step):
        f += step
    lk._limit = max(f, 0.0)
    return lk._limit


def reach(lk: SwingLinkage, travel: float, step: float = 1.0):
    """Lowest the deploying foot gets, and where [mm above the floor, deg].

    POSITIVE travel deploys the RIGHT (-y) wing, which is the side scored
    throughout: symmetry means the left is its mirror. The sign is not a free
    choice -- it falls out of `crank_tip`, where the right arm sits at
    +between/2 from vertical, so turning the shaft positively swings it DOWN.
    """
    lk.reset()
    best, at = 1e9, 0.0
    t = 0.0
    while t <= travel + 1e-9:
        pz = lk.pose(-1, t)
        if pz is None:
            break
        z = lk.z_floor(float(pz["foot"][1]))
        if z < best:
            best, at = z, t
        t += step
    return best, at


def top_extents(lk: SwingLinkage, travel: float, step: float = 2.0):
    """Non-deployed wing top |y|: (rest, end-of-stroke, max, where) [mm].

    THE TWO ENDPOINTS ARE WHAT IS SCORED, equally: the symmetric rest pose the
    bike drives in, and the fully-deployed pose it sits in during a shot. The
    max over the sweep is reported and plotted as a safety check but NOT
    scored -- it is dominated by whichever endpoint is worse and is blind to
    the other, which is how a design that ended at 74 mm beat one that ended at
    37 mm on an identical peak.

    THE thing to minimise on this mechanism. There is no width envelope to fit
    inside -- the wings may protrude -- so the goal is simply that they stick
    out as little as possible, both at the symmetric rest pose and while one
    side is down and the other is hanging.

    Measured on the RISING side across the whole stroke, and t = 0 is the rest
    pose, so one sweep covers both cases. Sweeping matters: the as-drawn
    geometry starts at 97 mm and comes IN to 34 mm around mid-stroke (the far
    wing meeting the side of the bike), while a design scored only at rest went
    the other way -- 75 mm at rest but 111 mm by the end, which is worse
    everywhere that counts and looked better on the metric.
    """
    lk.reset()
    worst, at, rest, end = 0.0, 0.0, None, None
    t = 0.0
    while t <= travel + 1e-9:
        pz = lk.pose(1, t)
        if pz is None:
            break
        y = abs(float(pz["top"][0]))
        if rest is None:
            rest = y
        end = y
        if y > worst:
            worst, at = y, t
        t += step
    if rest is None:
        return 1e3, 1e3, 1e3, 0.0
    return rest, end, worst, at


def far_inboard_deg(lk: SwingLinkage, travel: float, step: float = 2.0) -> float:
    """Least angle the RISING wing's panel makes with vertical [deg].

    Positive is outboard; NEGATIVE means the panel has swung past vertical and
    is leaning in over the chassis. That crossing is the real limit on this
    mechanism, and it is angular, not lateral -- an earlier version of this
    file constrained the far wing's DISTANCE from the centreline in mm, which
    is a different quantity that is neither necessary nor sufficient.

    The far arm does not sweep monotonically inward: it goes IN, reaches a
    minimum where the crank and coupler go collinear -- the input dead point --
    and then comes back OUT. That minimum is fixed by the LINK LENGTHS ALONE.

    Measured, holding the lengths and sweeping only the crank angle: the
    minimum is -0.2 deg at every `angle_between_cranks` from 20 to 90, over the
    driven stroke as well as the full assembly range. The angle changes the
    stroke LENGTH (136 deg at 20, 100 deg at 90) and the rest setpoint; it does
    not change how far in the far arm reaches, because a stroke long enough to
    right the bike passes the dead point either way.

    So the never-inward property is a pure LENGTH constraint, and the angle is
    free afterwards to place the rest pose. That is why the design procedure
    works: size the lengths so the collinear pose itself clears vertical, then
    relax the angle to taste. (An earlier version of this docstring claimed the
    angle controlled the exposure. It does not.)
    """
    lk.reset()
    worst = 1e9
    t = 0.0
    while t <= travel + 1e-9:
        pz = lk.pose(1, t)
        if pz is None:
            break
        v = pz["top"] - pz["foot"]
        worst = min(worst, float(np.degrees(np.arctan2(v[0], v[1]))))
        t += step
    return worst


def foot_outboard_mm(lk: SwingLinkage, travel: float, step: float = 2.0) -> float:
    """Least margin between the deploying foot and its OWN pivot [mm].

    Negative means the panel has swung inboard PAST its own hinge, which is
    geometric nonsense in a real part: the panel would have to pass through the
    bracket carrying it. Nothing else here catches it -- a design doing this
    can still satisfy reach, hand-off, torque and the far-wing angle, and one
    did (foot to |y| 0.2 mm against a pivot at 8.0 mm).
    """
    lk.reset()
    py = abs(float(lk.pivot(-1)[0]))
    worst = 1e9
    t = 0.0
    while t <= travel + 1e-9:
        pz = lk.pose(-1, t)
        if pz is None:
            break
        worst = min(worst, abs(float(pz["foot"][0])) - py)
        t += step
    return worst


def far_clearance(lk: SwingLinkage, travel: float, step: float = 1.0) -> float:
    """Closest the RISING wing gets to the centreline over the stroke [mm].

    THE constraint that a bare hinge pair cannot satisfy. Co-rotation swings
    one wing toward the centreline as the other swings away, and on a free
    hinge it goes straight through the chassis -- through the drive servos at
    |y| 15.8..44.25 and the battery inboard of them. The four-bar's rocker arc
    is what bounds it, so this reports the margin the geometry provides rather
    than a limit someone has to remember to configure.
    """
    lk.reset()
    worst = 1e9
    t = 0.0
    while t <= travel + 1e-9:
        pz = lk.pose(1, t)           # left rises while right deploys
        if pz is None:
            break
        for q in (pz["joint"], pz["foot"], pz["top"]):
            worst = min(worst, abs(float(q[0])))
        t += step
    return worst


def rest_ground_angle(lk: SwingLinkage) -> float:
    """Roll angle at which the RESTING wing first touches the floor [deg].

    Must clear RECOVERABLE_DEG, or a lean the policy could still save drags a
    wing -- the same constraint the geared pair's pivot height turns on.
    """
    lk.reset()
    pz = lk.pose(-1, 0.0)
    if pz is None:
        return 0.0
    worst = 90.0
    for q in (pz["foot"], pz["top"]):
        y, z = abs(float(q[0])), lk.z_floor(float(q[1]))
        if y < 1e-6:
            continue
        worst = min(worst, float(np.degrees(np.arctan2(z, y))))
    return worst


TRANS_END_FRAC = 0.85
"""Fraction of the stroke the transmission-angle floor applies over.

The floor exists to keep FORCE AUTHORITY where the mechanism is working. It
must NOT be applied at the deployed end, because that is exactly where a
four-bar should be allowed to approach its input-side dead point: near it the
wing cannot backdrive the servo, so the deployed pose holds with little or no
current. Forbidding it throws away the one thing a linkage does that a gear
train cannot.

Measured on both real geometries, the minimum sits at 98-100% of the stroke
while mid-stroke runs 85-87 deg -- so a blanket floor was constraining nothing
but the toggle, and it was the binding constraint in every seed.
"""


def min_transmission(lk: SwingLinkage, travel: float, step: float = 3.0,
                     frac: float = TRANS_END_FRAC) -> float:
    """Worst transmission angle over the stroke [deg].

    Angle between coupler and rocker at the joint. Same meaning and the same
    trap as the mirrored study: a reach-only objective walks straight into a
    dead point, because the dead point costs nothing on reach while being the
    least buildable part of the design.
    """
    lk.reset()
    worst = 180.0
    limit = travel * frac
    t = 0.0
    while t <= limit + 1e-9:
        for side in (-1, 1):
            pz = lk.pose(side, t)
            if pz is None:
                return 0.0
            v1 = pz["crank_tip"] - pz["joint"]
            v2 = pz["pivot"] - pz["joint"]
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 < 1e-9 or n2 < 1e-9:
                return 0.0
            mu = np.degrees(np.arccos(np.clip(float(v1 @ v2) / (n1 * n2), -1, 1)))
            worst = min(worst, min(mu, 180.0 - mu))
        t += step
    return worst


def stow_half_width(lk: SwingLinkage) -> float:
    """Widest |lateral| the whole mechanism reaches AT REST [mm].

    Rest, not swept, and for the same reason the mirrored study gives: rest is
    the bike's driving envelope, while everything past it happens with the bike
    already committed to a move.
    """
    lk.reset()
    worst = 0.0
    for side in (-1, 1):
        pz = lk.pose(side, 0.0)
        if pz is None:
            return 1e3
        for q in (pz["crank_tip"], pz["joint"], pz["foot"], pz["top"]):
            worst = max(worst, abs(float(q[0])))
    return worst


def ratio_curve(lk: SwingLinkage, travel: float, step: float = 1.0):
    """(travel, wing_deg, d wing/d crank) for the deploying side."""
    lk.reset()
    ts, wds = [], []
    t = 0.0
    while t <= travel + 1e-9:
        pz = lk.pose(-1, t)
        if pz is None:
            break
        ts.append(t)
        wds.append(pz["wing_deg"])
        t += step
    ts, wds = np.asarray(ts), np.asarray(wds)
    if len(ts) < 3:
        return ts, wds, np.zeros_like(ts)
    return ts, wds, np.gradient(wds, ts)


# --------------------------------------------------------------------------
# statics


def torque_curve(lk: SwingLinkage, travel: float, mass: float = BIKE_MASS_KG,
                 com_z: float = COM_Z_MM, step: float = 1.0):
    """Servo torque through the stroke, bike lying on the deploying wing.

    Same load case as the mirrored study, and the same virtual-work step: the
    bike is on its side ON the wing, the wing stays flat on the ground, and
    driving it out rotates the BIKE up around the pivot.

        roll  phi   = 90 - wing_deg
        tau_wing    = m g * |lever|          weight moment about the pivot
        tau_servo   = tau_wing * d(wing)/d(crank)

    Reported for the RIGHT wing deploying; symmetry gives the other side.
    """
    ts, wds, grad = ratio_curve(lk, travel, step)
    if not len(ts):
        return ts, wds, np.zeros(0), np.zeros(0)
    piv = lk.pivot(-1)
    com0 = np.array([0.0, com_z - lk.wheel_radius])   # sketch frame
    tau_w, tau_s = [], []
    for wd, g in zip(wds, grad):
        phi = 90.0 - abs(wd)
        com = piv + _rot(com0 - piv, -(90.0 - phi))
        lever = abs(float(com[0] - piv[0])) / 1000.0
        tw = mass * GRAVITY * lever
        tau_w.append(tw)
        tau_s.append(tw * abs(float(g)))
    return ts, wds, np.asarray(tau_w), np.asarray(tau_s)


def peak_torque(lk: SwingLinkage, travel: float) -> float:
    _ts, _wd, _tw, tsv = torque_curve(lk, travel)
    return float(np.max(np.abs(tsv))) if len(tsv) else 1e3


# --------------------------------------------------------------------------
# checks


def cmd_check(cfg) -> None:
    """Re-derive the as-drawn sketch points, and assert the symmetry claim.

    This file's whole authority is that it models the CAD. Both halves are
    cheap and both have caught a sign error already -- the crank arm bases were
    mirrored on the first pass, which reproduced a plausible-looking mechanism
    that was not the drawn one.
    """
    lk = SwingLinkage(cfg)
    pz = lk.pose(1, 0.0)              # +y side = the bike's LEFT
    ok = True
    print("as-drawn sketch points (+y / left side, crank at rest):")
    for name, want in _SKETCH.items():
        got = pz[{"joint": "joint", "foot": "foot", "top": "top"}[name]]
        d = float(np.linalg.norm(np.asarray(want) - got))
        flag = "" if d < 0.05 else "   <-- MISMATCH"
        ok &= d < 0.05
        print(f"   {name:6s} model ({got[0]:7.2f},{got[1]:7.2f})  "
              f"sketch ({want[0]:7.2f},{want[1]:7.2f})   d={d:.3f}{flag}")

    print("\nmirror symmetry (L at -t must equal R at +t, reflected):")
    worst = 0.0
    for t in (10.0, 30.0, 60.0, 90.0):
        lk.reset()
        a = lk.pose(-1, -t)
        lk.reset()
        b = lk.pose(1, t)
        if a is None or b is None:
            print(f"   t={t:5.1f}  one side does not close")
            ok = False
            continue
        for key in ("joint", "foot", "top"):
            d = float(np.linalg.norm(
                a[key] - np.array([-b[key][0], b[key][1]])))
            worst = max(worst, d)
    print(f"   worst mismatch over the stroke: {worst:.6f} mm"
          + ("" if worst < 1e-6 else "   <-- NOT SYMMETRIC"))
    ok &= worst < 1e-6
    if not ok:
        print("\n  NOTE: the sketch comparison is only meaningful for the config\n"
              "  TRACED from swing-wings-geom-mock. An optimised or hand-edited\n"
              "  geometry is EXPECTED to fail it -- what must still hold there is\n"
              "  the symmetry check above.")
    print("\n" + ("OK" if ok else "FAILED"))


def cmd_sweep(cfg, out: Path) -> None:
    """Reach, ratio and clearance against crank travel."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lk = SwingLinkage(cfg)
    lim = assembly_limit(lk, cap=_ASSEMBLY_CAP)
    use, use_hand, _lo = useful_stroke(lk)
    travel = min(float(cfg["stroke"]["crank_travel_deg"]), lim)
    ts, wds, grad = ratio_curve(lk, travel)
    zs = []
    lk.reset()
    for t in ts:
        pz = lk.pose(-1, float(t))
        zs.append(lk.z_floor(float(pz["foot"][1])) if pz else np.nan)
    zs = np.asarray(zs)
    lo, at = reach(lk, travel)

    print(f"assembly limit        {lim:6.1f} deg of crank")
    bt, blow, barm = brace(lk)
    print(f"brace                 {blow:6.1f} mm above floor at crank {bt:.0f},"
          f" {barm:.0f} mm outboard"
          + ("" if blow <= 2.0 else "   <-- CANNOT BRACE"))
    print(f"useful stroke         {use:6.1f} deg   (assembly limit {lim:.0f})")
    print(f"hand-off roll there   {use_hand:6.1f} deg   (window +-{HANDOFF_WINDOW_DEG:.0f})")
    print(f"far wing inboard      {far_inboard_deg(lk, use):6.1f} deg from vertical"
          f"   (min +{MIN_FAR_INBOARD_DEG:.0f}, negative = into the body)")
    print(f"  (lateral, fyi)      {far_clearance(lk, use):6.1f} mm from the centreline")
    print(f"foot vs own pivot     {foot_outboard_mm(lk, use):6.1f} mm"
          + ("" if foot_outboard_mm(lk, use) >= 0 else "   <-- SWINGS INBOARD"))
    print(f"rest grounds at       {rest_ground_angle(lk):6.1f} deg of roll"
          f"   (recoverable set {RECOVERABLE_DEG})")
    _r, _e, _m, _a = top_extents(lk, use)
    print(f"rest half-width       {stow_half_width(lk):6.1f} mm")
    print(f"wing-top rest / end   {_r:6.1f} / {_e:.1f} mm      <- scored, equal weight")
    print(f"           max        {_m:6.1f} mm at crank {_a:.0f}   (check only)")
    print(f"worst transmission    {min_transmission(lk, use):6.1f} deg")
    print(f"peak servo torque     {peak_torque(lk, travel):6.3f} N.m")
    print_feasibility(lk, cfg)

    fig, axes = plt.subplots(4, 1, figsize=(8.5, 11.0), sharex=True)
    axes[0].plot(ts, zs, color=_C["right"], lw=2)
    # The FLOOR is the reference, because bracing is what the panel has to
    # reach. The ball centre used to be drawn here; it is not a target for this
    # mechanism -- fixed panels hit the ball.
    axes[0].axhline(0.0, color="0.3", lw=1.2)
    axes[0].annotate("floor — the brace has to get here", (ts[0], 0.0),
                     fontsize=8, va="bottom", color="0.3")
    axes[0].set_ylabel("deploying foot [mm above floor]")
    axes[1].plot(ts, wds, color=_C["right"], lw=2)
    axes[1].set_ylabel("wing rotation [deg]")
    axes[2].plot(ts, grad, color=_C["right"], lw=2)
    axes[2].axhline(0.0, color="0.7", lw=1)
    axes[2].set_ylabel("d(wing)/d(crank)")
    lk.reset()
    tops = [(float(t), abs(float(lk.pose(1, float(t))["top"][0])))
            for t in ts if lk.pose(1, float(t)) is not None]
    if tops:
        tt, tv = zip(*tops)
        axes[3].plot(tt, tv, color=_C["left"], lw=2)
        axes[3].axhline(cfg["bike"]["bike_width"] / 2.0, ls="--", color="0.5")
        axes[3].annotate("bike half-width", (tt[0], cfg["bike"]["bike_width"] / 2.0),
                         fontsize=8, va="bottom", color="0.4")
    axes[3].set_ylabel("non-deployed wing top |y| [mm]")
    axes[3].set_xlabel("crank travel [deg]  (positive deploys the right wing)")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].set_title("swing linkage: reach and ratio through the stroke")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"\nwrote {out}")


def _draw(ax, lk: SwingLinkage, t: float, *, alpha: float = 1.0,
          labels: bool = False) -> None:
    """One pose, both sides, in the body frame."""
    ax.axhline(0.0, color="0.85", lw=1, zorder=0)
    for side, tag in ((-1, "right"), (1, "left")):
        pz = lk.pose(side, t)
        if pz is None:
            continue
        sh = lk.shaft
        pts = {k: np.array([pz[k][0], lk.z_floor(pz[k][1])]) for k in
               ("joint", "foot", "top", "crank_tip", "pivot")}
        shaft = np.array([sh[0], lk.z_floor(sh[1])])
        ax.plot(*zip(shaft, pts["crank_tip"]), color="0.35", lw=2.5,
                alpha=alpha, zorder=3)
        ax.plot(*zip(pts["crank_tip"], pts["joint"]), color="0.55", lw=2,
                alpha=alpha, zorder=3)
        ax.plot(*zip(pts["pivot"], pts["joint"]), color="0.35", lw=2,
                alpha=alpha, zorder=3)
        ax.plot(*zip(pts["foot"], pts["top"]), color=_C[tag], lw=5,
                solid_capstyle="round", alpha=alpha, zorder=4)
        ax.plot(*pts["pivot"], "o", ms=5, color="0.2", alpha=alpha, zorder=5)
        if labels:
            ax.annotate(tag, pts["top"], fontsize=8, color=_C[tag],
                        ha="center", va="bottom")
    ax.plot(*np.array([lk.shaft[0], lk.z_floor(lk.shaft[1])]), "o", ms=7,
            color="0.1", zorder=6)


def cmd_rest(cfg, out: Path) -> None:
    """The rest pose, labelled — the mechanism's driving envelope."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lk = SwingLinkage(cfg)
    fig, ax = plt.subplots(figsize=(7.0, 6.5))
    _draw(ax, lk, 0.0, labels=True)
    ang = rest_ground_angle(lk)
    ax.set_title(f"swing linkage at REST — symmetric, half-width "
                 f"{stow_half_width(lk):.0f} mm, grounds at {ang:.0f}°")
    ax.set_xlabel("y [mm]")
    ax.set_ylabel("z above floor [mm]")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")


def cmd_video(cfg, out: Path, fps: int = 30, seconds: float = 6.0) -> None:
    """Animate right -> rest -> left -> rest, which is the whole mechanism."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    try:
        import imageio.v2 as imageio
    except ImportError as e:
        raise SystemExit("needs imageio: pip install -e '.[viz]'") from e

    lk = SwingLinkage(cfg)
    travel = min(float(cfg["stroke"]["crank_travel_deg"]), assembly_limit(lk))
    n = int(fps * seconds)
    q = max(n // 4, 2)
    sched = np.concatenate([np.linspace(0, -travel, q),
                            np.linspace(-travel, 0, q),
                            np.linspace(0, travel, q),
                            np.linspace(travel, 0, n - 3 * q)])
    out.parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(out, fps=fps, macro_block_size=1) as w:
        for t in sched:
            fig, ax = plt.subplots(figsize=(7.0, 6.4))
            lk.reset()
            _draw(ax, lk, float(t))
            pz = lk.pose(-1, float(t))
            z = lk.z_floor(float(pz["foot"][1])) if pz else float("nan")
            ax.axhline(0.0, color="0.3", lw=1.2)
            ax.set_xlim(-160, 160)
            ax.set_ylim(-10, 230)
            ax.set_aspect("equal")
            ax.grid(alpha=0.25)
            ax.set_title(f"crank {t:+6.1f}°     right foot {z:5.1f} mm")
            ax.set_xlabel("y [mm]")
            ax.set_ylabel("z above floor [mm]")
            fig.tight_layout()
            fig.canvas.draw()
            w.append_data(np.asarray(fig.canvas.buffer_rgba())[..., :3])
            plt.close(fig)
    print(f"wrote {out}")


def cmd_righting_video(cfg, out: Path, fps: int = 25, seconds: float = 7.0) -> None:
    """The bike pushing itself up, in the GROUND frame.

    The pose that matters is not the mechanism in the body frame -- it is the
    bike rotating up around a wing that stays flat on the floor. Drawn by
    rotating the whole assembly about the deploying pivot so the wing lies on
    z = 0, which is the same construction the mirrored study uses.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    try:
        import imageio.v2 as imageio
    except ImportError as e:
        raise SystemExit("needs imageio: pip install -e '.[viz]'") from e

    lk = SwingLinkage(cfg)
    travel = min(float(cfg["stroke"]["crank_travel_deg"]), assembly_limit(lk))
    n = int(fps * seconds)
    # POSITIVE travel, because positive is what deploys side -1 -- the side
    # this shot grounds on. Scheduling negative here laid the RED (right) panel
    # flat while the BLUE (left) one was the one actually descending, so the
    # bike appeared to right itself on the wrong wing. Same sign that had to be
    # fixed in `reach`, `ratio_curve` and `min_transmission`; this call site was
    # missed because it is the only one that names a side and a sign separately.
    sched = np.concatenate([np.linspace(0, travel, n // 2),
                            np.linspace(travel, 0, n - n // 2)])
    out.parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(out, fps=fps, macro_block_size=1) as w:
        for t in sched:
            fig, ax = plt.subplots(figsize=(7.4, 6.0))
            lk.reset()
            pz = lk.pose(-1, float(t))
            if pz is not None:
                foot = np.array([pz["foot"][0], lk.z_floor(pz["foot"][1])])
                top = np.array([pz["top"][0], lk.z_floor(pz["top"][1])])
                v = top - foot
                # Rotate so the panel lies flat. TWO rotations do that -- panel
                # along +x or along -x -- and only one of them leaves the bike
                # ABOVE the floor. Picking blind put the CoM 90 mm UNDER it and
                # rendered the whole animation upside down and off-screen, which
                # is what "it is all underwater" looked like.
                phi = -np.degrees(np.arctan2(v[1], v[0]))
                if _rot(np.array([0.0, COM_Z_MM]) - foot, phi)[1] < 0.0:
                    phi += 180.0

                def G(p):
                    p = np.asarray(p, float)
                    return _rot(p - foot, phi)

                for side, tag in ((-1, "right"), (1, "left")):
                    q = lk.pose(side, float(t))
                    if q is None:
                        continue
                    a = G([q["foot"][0], lk.z_floor(q["foot"][1])])
                    b = G([q["top"][0], lk.z_floor(q["top"][1])])
                    ax.plot([a[0], b[0]], [a[1], b[1]], color=_C[tag], lw=5,
                            solid_capstyle="round", zorder=4)
                hw, hh = lk.half_span, lk.bike_height
                box = np.array([[-hw, 0], [hw, 0], [hw, hh], [-hw, hh], [-hw, 0]],
                               float)
                body = np.array([G(p) for p in box])
                ax.plot(body[:, 0], body[:, 1], color="0.4", lw=1.6, zorder=2)
                com = G([0.0, COM_Z_MM])
                ax.plot(*com, "o", ms=7, color="0.15", zorder=6)
                roll = 90.0 - abs(pz["wing_deg"])
                ax.set_title(f"crank {t:+6.1f}°     roll {roll:5.1f}°")
            ax.axhline(0.0, color="0.2", lw=1.5, zorder=1)
            ax.set_xlim(-230, 230)
            ax.set_ylim(-20, 260)
            ax.set_aspect("equal")
            ax.grid(alpha=0.25)
            ax.set_xlabel("[mm]")
            fig.tight_layout()
            fig.canvas.draw()
            w.append_data(np.asarray(fig.canvas.buffer_rgba())[..., :3])
            plt.close(fig)
    print(f"wrote {out}")


# --------------------------------------------------------------------------
# optimisation

# THE LINKAGE ONLY. The panel is NOT searched, and that is a correction: it was
# carrying four of nine variables and none of them should have been free.
#
#   panel BOTTOM  sits at `bike.ground_clearance` by construction -- derived in
#                 SwingLinkage.__init__, not searched. It is a design
#                 convention (the lowest the panel may hang while driving),
#                 exactly as wing_linkage sets `wing_bottom = ground_clearance`.
#   panel TOP     is not a reach parameter. It can always be extended; it only
#                 has to clear the CoM when the bike is on its side, and its
#                 one real consequence here is the WIDTH EXTENT at rest.
#   panel MOUNT   (`wing_angle_from_rocker`, `wing_norm_offset`) is a fixed
#                 feature of the part, taken from the sketch.
#
# Searching them let the optimiser "solve" bracing by stretching the panel
# rather than by improving the linkage, which is not a mechanism improvement at
# all -- and it kept mounting the panel on the wrong side of the rocker until
# the bounds forbade it. Six variables, all of them linkage.
_VARS = [
    # LATERAL offset of each hinge from the CENTRELINE -- the two sit at
    # +-this. NOT the distance from the wing, despite reading that way. `x`
    # here is the repo's `y`; the name is the sketch's. Cannot go negative
    # (that would put a hinge past the midline), and 15 is the floor for two
    # hinges that have to carry real pins. Capped at the bike's half-width.
    ("wing_pivot_x",           ("mechanism", "wing_pivot_x"),           15.0,  40.0),
    # 15 mm floor is a buildability limit, not a physical one: the as-drawn
    # crank is 32 mm and the search will happily go to 9 mm if allowed.
    ("crank_length",           ("mechanism", "crank_length"),           15.0,  60.0),
    ("coupler_length",         ("mechanism", "coupler_length"),         30.0, 140.0),
    # Coupler/wing attach point -> wing hinge. A VIRTUAL link (it is what
    # drives the wing rather than a separate part), but it still has to be long
    # enough for the mechanism to fit around, so the same 15 mm floor as the
    # crank.
    ("rocker_length",          ("mechanism", "rocker_length"),          15.0,  90.0),
    # Sets the REST SETPOINT and nothing else that binds -- see the note in
    # config/swing_linkage.yaml. Reported as a primary output.
    ("angle_between_cranks",   ("mechanism", "angle_between_cranks"),   20.0, 170.0),
    # Fairly free: the shaft can sit higher without trouble, so the range is
    # wide in both directions rather than pinned near the axle.
    ("servo_offset",           ("mechanism", "servo_offset"),           20.0, 160.0),
]

TORQUE_BUDGET_NM = 0.55
"""Peak servo torque the mechanism may need anywhere in the stroke [N.m].

The XC330-T181 is 0.80 N.m at 12 V, 0.76 at the 3S nominal 11.1 V and ~0.66 at
the 9.9 V pack cutoff. 0.55 leaves real margin at cutoff rather than passing on
a full battery and stalling on a half-empty one.

Missing once already, and it showed: `peak_torque` was computed and printed
every run but never scored, so the first search that hit the righting target
came back needing 0.659 N.m -- 1.002x the cutoff torque.
"""

MIN_TRANSMISSION_DEG = 30.0
MIN_FAR_INBOARD_DEG = 5.0
"""Margin the RISING wing must keep OUTBOARD of vertical, in degrees.

The mechanism's own limit, as built: the non-righting arm swings in, bottoms
out where crank and coupler go collinear, and swings back out. Past vertical it
is leaning in over the chassis. 5 deg rather than 0 so link tolerance does not
put it through.

REPLACES a millimetres-from-the-centreline constraint, which measured the wrong
thing: the as-drawn sketch sits at -0.2 deg (tuned by eye to exactly this
boundary) while reading 29.4 mm laterally, and those two numbers do not rank
candidates the same way.
"""

HANDOFF_WINDOW_DEG = 6.0
"""Roll the bike may be left at when the stroke ends, in degrees.

THE righting criterion for this mechanism, and it is NOT the mirrored study's
`TARGET_WING_DEG = 90`. That constant works there because its stowed wing
starts flat on the ground at the fallen pose, so roll = 90 - rotation and
"90 deg of wing rotation" and "upright" are the same statement.

Here the rest pose is a splayed V, NOT flush with the bike's vertical, so a
tipped bike does not land flat on a wing and that relation never holds. What
decides whether the bike ends on its wheels is the OTHER end of the stroke:
the panel's angle from horizontal in the body frame. Lay that panel flat on
the floor and the bike rests at exactly that angle -- so driving it to zero is
driving the bike upright, whatever the wing did on the way.

+-12 deg is the hand-off window from bike_params.yaml: inside it the balance
policy can take over.

Scoring rotation-from-rest instead reported both real geometries as unable to
right the bike (52.6 and 35.5 deg of a supposed 90) when the as-drawn sketch
in fact hands off at 5.3 deg. Borrowing a sibling study's constant without its
assumption is what caused that.
"""


def handoff_roll(lk: SwingLinkage, travel: float) -> float:
    """Roll the bike would rest at, standing on the panel at `travel` [deg].

    The panel is a LINE, not a ray, so 170 deg from horizontal is 10 deg of
    roll -- which side is up is decided by the bike, not by the vector.
    """
    lk.reset()
    pz = lk.pose(-1, travel)
    if pz is None:
        return 180.0
    v = pz["top"] - pz["foot"]
    a = abs(float(np.degrees(np.arctan2(v[1], v[0]))))
    return min(a, 180.0 - a)


BRACE_ARM_MM = 90.0
"""Lateral reach wanted from the bracing foot, in millimetres.

WHAT THE MECHANISM IS ACTUALLY FOR, alongside righting. A ball arrives at one
side of the bike; the wing on the OTHER side goes down to the floor and props
it, so the impulse pushes the bike onto a support instead of over. Hitting the
ball is not this mechanism's job -- there are fixed panels for that.

Two things make a brace work, and the first is pass/fail: the panel has to
REACH THE FLOOR, or it carries nothing. Past that, the further outboard the
contact sits the more overturning moment it resists, so the arm is rewarded
rather than merely permitted -- capped at 90 mm because beyond that the stowed
envelope and the far-wing clearance start paying for it.

NOT a ball-strike reach. An earlier version of this file scored the foot
against the 33.5 mm ball centre, which was never a requirement anyone stated --
it leaked in from the ball-shot study and quietly became an objective. The
panel can be made longer to reach whatever it needs to reach; what it must do
is touch the floor and stay out of the servos.
"""

_ASSEMBLY_CAP = 200.0
_MAX_HALF: float | None = None    # None = the bike's own half-width
"""Largest crank travel the assembly search will consider [deg]."""


def _walk(lk: SwingLinkage, step: float = 2.0):
    """One continuous sweep of the deploying side: [(travel, handoff, low, arm)].

    ONE walk, because every metric here wants the same samples and the first
    version took a fresh one per metric -- each resetting the branch memory on
    every sample, which both wasted the continuation and paid two extra
    circle-circle solves per point.
    """
    if step in lk._walk_memo:
        return lk._walk_memo[step]
    lim = assembly_limit(lk, cap=_ASSEMBLY_CAP)
    lk.reset()
    rows = []
    t = 0.0
    while t <= lim + 1e-9:
        pz = lk.pose(-1, t)
        if pz is None:
            break
        v = pz["top"] - pz["foot"]
        a = abs(float(np.degrees(np.arctan2(v[1], v[0]))))
        ends = (pz["foot"], pz["top"])
        zs = [lk.z_floor(float(q[1])) for q in ends]
        k = int(np.argmin(zs))
        rows.append((t, min(a, 180.0 - a), zs[k], abs(float(ends[k][0]))))
        t += step
    lk._walk_memo[step] = rows
    return rows


def brace(lk: SwingLinkage, step: float = 2.0):
    """Best bracing pose: (travel, lowest panel point, lateral arm) [deg, mm, mm].

    Searched over the WHOLE assembly range, not the righting stroke: bracing
    and righting are two commanded positions of one mechanism and they do not
    land at the same crank angle. On the optimised geometry the hand-off is at
    128 deg and the lowest panel point at 150.

    Measured on the whole panel rather than the foot tip, because a panel that
    lies down onto the floor braces on its edge, and the tip is not necessarily
    the lowest part of it.
    """
    rows = _walk(lk, step)
    if not rows:
        return 0.0, 1e9, 0.0
    t, _h, low, arm = min(rows, key=lambda r: r[2])
    return t, low, arm


def useful_stroke(lk: SwingLinkage) -> tuple[float, float, float]:
    """(travel to drive, hand-off roll there, foot height there).

    The stroke ends at the OUTPUT DEAD POINT -- where wing rotation stops
    increasing -- not at the assembly limit. This is a crank-rocker, so the
    crank keeps turning long past the point where the output has peaked: past
    it the wing swings BACK, the foot rises again, and the far wing carries on
    toward the centreline. Scoring over the full assembly range prices travel
    nobody would ever command, and it did exactly that -- `far_clearance` at
    200 deg reported an intrusion that happens 60 deg past the useful end.

    Keyed on HAND-OFF ROLL rather than on foot height: the stroke you would
    actually command is the one that leaves the bike most upright, and the two
    optimise at different crank angles -- see HANDOFF_WINDOW_DEG.
    """
    rows = _walk(lk)
    if not rows:
        return 0.0, 180.0, 1e9
    use, best, _low, _arm = min(rows, key=lambda r: r[1])
    lk.reset()
    pz = lk.pose(-1, use)
    z = lk.z_floor(float(pz["foot"][1])) if pz else 1e9
    return use, best, z


def feasibility(lk: SwingLinkage, cfg: dict, budgets=None) -> list[tuple]:
    """Every constraint as an explicit pass/fail with its margin.

    SEPARATE FROM THE OBJECTIVE ON PURPOSE. The objective is a weighted sum of
    soft penalties, which is right for steering a search and wrong for
    answering "does this design clear my constraints?" -- a run can and
    repeatedly did report `objective 0.000` while sitting exactly on four
    limits at once, because a soft penalty is zero AT the boundary.

    This is the report to read when you have come here WITH a problem (a link
    that flexed, a hinge that bound, a pose that would not calibrate), added a
    constraint for it, and want to know whether a candidate actually clears it
    rather than how it scored.

    Returns (name, value, op, limit, unit, ok) so callers can print or filter.
    """
    t_bud, far_min, trans_min, half_cap = (
        budgets if budgets is not None
        else (TORQUE_BUDGET_NM, MIN_FAR_INBOARD_DEG, MIN_TRANSMISSION_DEG, _MAX_HALF))
    use, hand, _lo = useful_stroke(lk)
    span = use
    bt, blow, barm = brace(lk)
    e_rest, e_end, e_max, e_at = top_extents(lk, span)
    rows = [
        ("hand-off roll",      hand,                        "<=", HANDOFF_WINDOW_DEG, "deg"),
        # The panel only has to end up FLAT relative to the floor -- the bike
        # falls onto the whole wing, so the contact is the panel, not a point,
        # and whatever height it sits at is fine. That is the same quantity as
        # `hand-off roll` above, so there is no separate brace criterion: the
        # height and arm below are reported, not scored.
        ("far wing outboard",  far_inboard_deg(lk, span),   ">=", far_min,            "deg"),
        ("foot vs own pivot",  foot_outboard_mm(lk, span),  ">=", 0.0,                "mm"),
        ("transmission angle", min_transmission(lk, span),  ">=", trans_min,          "deg"),
        ("peak servo torque",  peak_torque(lk, span),       "<=", t_bud,              "N.m"),
        ("rest ground clear",  rest_ground_angle(lk),       ">=", RECOVERABLE_DEG + 5.0, "deg"),
    ]
    out = []
    for name, val, op, lim, unit in rows:
        ok = (val <= lim + 1e-9) if op == "<=" else (val >= lim - 1e-9)
        out.append((name, val, op, lim, unit, ok))
    # Reported, never scored as pass/fail: there is no envelope to fit inside,
    # only a protrusion to minimise.
    out.append(("wing-top rest", e_rest, "min", float("nan"), "mm", None))
    out.append(("wing-top end", e_end, "min", float("nan"), "mm", None))
    out.append(("wing-top max", e_max, f"at {e_at:.0f}", float("nan"), "mm", None))
    out.append(("panel height there", blow, "fyi", float("nan"), "mm", None))
    out.append(("contact |y|", barm, "fyi", float("nan"), "mm", None))
    return out


def print_feasibility(lk, cfg, budgets=None) -> bool:
    rows = feasibility(lk, cfg, budgets)
    hard = [r for r in rows if r[5] is not None]
    npass = sum(1 for r in hard if r[5])
    print(f"\n  feasibility: {npass}/{len(hard)} constraints met")
    for name, val, op, lim, unit, ok in rows:
        if ok is None:
            print(f"     {'':4s} {name:20s} {val:8.1f} {unit:4s} ({op})")
            continue
        margin = (lim - val) if op == "<=" else (val - lim)
        tag = "PASS" if ok else "FAIL"
        tight = "  <- ON THE LIMIT" if ok and abs(margin) < 0.02 * max(abs(lim), 1.0) else ""
        print(f"     {tag:4s} {name:20s} {val:8.1f} {unit:4s} {op} {lim:7.1f}"
              f"   margin {margin:+8.2f}{tight}")
    return npass == len(hard)


def _apply(cfg: dict, x) -> dict:
    import copy as _copy
    c = _copy.deepcopy(cfg)
    for (_, path, _lo, _hi), v in zip(_VARS, x):
        node = c
        for k in path[:-1]:
            node = node[k]
        node[path[-1]] = float(v)
    return c


def _objective(x, cfg, budgets=None):
    """Feasibility first, then minimise how far the wing tops stick out.

    TWO-STAGE, NOT A WEIGHTED SUM. While any constraint is violated the value
    is 100 + the violation, so the search only ever trades violations against
    each other. Once everything is satisfied it is the protrusion in mm. A
    single weighted sum let the search BUY violations with millimetres, and it
    did -- across five seeds the narrowest designs were the least feasible.

    The protrusion term is max(rest, end), NOT their mean and not their
    difference. The mean lets one endpoint grow while the other shrinks at zero
    net cost; the difference alone lets both grow without bound so long as they
    grow together. max() drives the worse endpoint down, and once they are
    equal it can only improve by lowering both.
    """
    # BUDGETS ARE PASSED, NOT READ FROM GLOBALS, and that is load-bearing the
    # moment `workers=-1` is used: DE spawns processes, macOS spawn re-imports
    # this module, and a worker would come up with the DEFAULT constants. A
    # `--torque-budget 0.45` run would then score 0.55 in every worker and 0.45
    # only in the parent -- optimising something other than what was asked for,
    # with no error anywhere.
    t_bud, far_min, trans_min, half_cap = (
        budgets if budgets is not None
        else (TORQUE_BUDGET_NM, MIN_FAR_INBOARD_DEG, MIN_TRANSMISSION_DEG, _MAX_HALF))
    try:
        lk = SwingLinkage(_apply(cfg, x))
    except Exception:
        return 1e3

    # ONE stroke, ending where the panel is most nearly horizontal. Bracing
    # means the panel finishes FLAT -- the bike lands on the whole wing, so the
    # contact is the panel, not a point, and the height it sits at is fine.
    # That is the same pose `useful_stroke` already finds, so the brace pose
    # and the hand-off pose are one pose, not two.
    use, hand, _lo = useful_stroke(lk)
    if use < 20.0 or not np.isfinite(hand):
        return 1e3                      # barely moves before it comes apart
    span = use

    # RIGHTING: how far from upright the bike is left when the stroke ends.
    short_rot = max(0.0, hand - HANDOFF_WINDOW_DEG)
    # Cheap bail before the sweeps below, keeping a gradient so DE can descend
    # toward feasibility rather than stranding on a plateau.
    if short_rot > 25.0:
        return 200.0 + 2.0 * short_rot

    far = max(0.0, far_min - far_inboard_deg(lk, span))
    inboard = max(0.0, -foot_outboard_mm(lk, span))
    bind = max(0.0, trans_min - min_transmission(lk, span))
    over = max(0.0, peak_torque(lk, span) - t_bud)
    ground = max(0.0, RECOVERABLE_DEG + 5.0 - rest_ground_angle(lk))

    viol = (2.0 * short_rot + 0.60 * far + 1.00 * inboard
            + 0.30 * bind + 60.0 * over + 1.20 * ground)
    if viol > 1e-9:
        return 100.0 + viol

    e_rest, e_end, _e_max, _e_at = top_extents(lk, span)
    return max(e_rest, e_end)


def cmd_optimize(cfg, save: Path | None, seed: int, iters: int,
                 torque: float | None = None, min_far: float | None = None,
                 min_trans: float | None = None, max_half: float | None = None) -> None:
    """Search the shared link lengths.

    The budgets are OVERRIDABLE rather than edited in place: the module
    constants are what the tracked configs were produced against, and a study
    whose committed figures cannot be reproduced from its own defaults is the
    failure analysis/ conventions exist to prevent.
    """
    from scipy.optimize import differential_evolution

    global TORQUE_BUDGET_NM, MIN_FAR_INBOARD_DEG, MIN_TRANSMISSION_DEG, _MAX_HALF
    if torque is not None:
        TORQUE_BUDGET_NM = torque
    if min_far is not None:
        MIN_FAR_INBOARD_DEG = min_far
    if min_trans is not None:
        MIN_TRANSMISSION_DEG = min_trans
    _MAX_HALF = max_half
    print(f"budgets: torque {TORQUE_BUDGET_NM} N.m   far {MIN_FAR_INBOARD_DEG} deg"
          f"   transmission {MIN_TRANSMISSION_DEG} deg"
          + (f"   half-width {max_half} mm" if max_half else ""))

    lk0 = SwingLinkage(cfg)
    use0, hand0, lo0 = useful_stroke(lk0)
    print(f"start:  hand-off {hand0:.1f} deg (window {HANDOFF_WINDOW_DEG:.0f})   "
          f"brace {brace(lk0)[1]:.1f} mm above floor   "
          f"far {far_inboard_deg(lk0, use0):.1f} deg from vertical   "
          f"rest grounds {rest_ground_angle(lk0):.1f} deg")
    bounds = [(lo, hi) for _n, _p, lo, hi in _VARS]
    buds = (TORQUE_BUDGET_NM, MIN_FAR_INBOARD_DEG, MIN_TRANSMISSION_DEG, _MAX_HALF)
    # workers=-1: the objective is a pure function of (x, cfg, budgets), all
    # picklable, so this parallelises across cores for free. `updating` must be
    # "deferred" once workers != 1 -- scipy will not do immediate updating
    # across processes.
    res = differential_evolution(_objective, bounds, args=(cfg, buds), seed=seed,
                                 maxiter=iters, tol=1e-8, polish=True,
                                 init="sobol", mutation=(0.4, 1.0),
                                 recombination=0.85,
                                 workers=-1, updating="deferred")
    best = _apply(cfg, res.x)
    lk = SwingLinkage(best)
    use, hand, lo = useful_stroke(lk)
    at = use
    # The DRIVEN range, not the righting stroke -- the same span feasibility()
    # uses, so the summary and the table cannot disagree about the same
    # quantity. They did: 30.1 vs 9.1 deg of far-wing clearance in one report.
    lim = use
    print(f"\nbest (objective {res.fun:.3f}"
          + ("  -- INFEASIBLE, value is 100 + violation)" if res.fun >= 100 else ")"))
    for (name, _p, _l, _h), v in zip(_VARS, res.x):
        print(f"   {name:24s} {v:8.2f}")
    print(f"\n   useful stroke      {use:6.1f} deg"
          f"   (assembly limit {assembly_limit(lk, cap=_ASSEMBLY_CAP):.0f})")
    print(f"   hand-off roll      {hand:6.1f} deg"
          f"   (window +-{HANDOFF_WINDOW_DEG:.0f})")
    bt, blow, barm = brace(lk)
    print(f"   brace             {blow:6.1f} mm above the floor at crank {bt:.0f},"
          f" {barm:.0f} mm outboard"
          + ("" if blow <= 2.0 else "   <-- CANNOT BRACE"))
    print(f"   far wing inboard   {far_inboard_deg(lk, lim):6.1f} deg from vertical"
          f"   (min +{MIN_FAR_INBOARD_DEG})")
    print(f"   foot vs own pivot  {foot_outboard_mm(lk, lim):6.1f} mm"
          + ("" if foot_outboard_mm(lk, lim) >= 0 else "   <-- SWINGS INBOARD"))
    print(f"   rest grounds at    {rest_ground_angle(lk):6.1f} deg"
          f"   (recoverable {RECOVERABLE_DEG})")
    print(f"   worst transmission {min_transmission(lk, lim):6.1f} deg")
    _r, _e, _m, _a = top_extents(lk, use)
    print(f"   angle between cranks {best['mechanism']['angle_between_cranks']:6.1f} deg"
          f"   <- sets the rest setpoint")
    print(f"   rest half-width    {stow_half_width(lk):6.1f} mm")
    print(f"   wing-top rest/end  {_r:6.1f} / {_e:.1f} mm   max {_m:.1f} at {_a:.0f}")
    print(f"   peak servo torque  {peak_torque(lk, lim):6.3f} N.m"
          f"   (budget {TORQUE_BUDGET_NM}, XC330 has 0.66 at cutoff)")
    ok = print_feasibility(lk, best, buds)
    if not ok:
        print("\n  NOT FEASIBLE -- the objective may still read low, because a"
              "\n  soft penalty is zero AT a boundary and small just past it.")
    if save is not None:
        best["stroke"]["crank_travel_deg"] = float(round(abs(at), 1))
        save.write_text(yaml.safe_dump(best, sort_keys=False))
        print(f"\nwrote {save}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=CONFIG)
    ap.add_argument("--check", action="store_true",
                    help="verify the model against the as-drawn sketch and "
                         "assert left/right symmetry")
    ap.add_argument("--sweep", action="store_true",
                    help="reach, wing angle and ratio against crank travel")
    ap.add_argument("--video", action="store_true",
                    help="animate right -> rest -> left -> rest")
    ap.add_argument("--righting", action="store_true",
                    help="ground-frame righting (with --video, animate it)")
    ap.add_argument("--optimize", action="store_true",
                    help="search the shared link lengths for a foot that "
                         "reaches the ball centre before the dead point")
    ap.add_argument("--save", type=Path, default=None,
                    help="write the optimised config here")
    ap.add_argument("--torque-budget", type=float, default=None,
                    help="N.m the servo may need anywhere in the stroke "
                         f"(default {TORQUE_BUDGET_NM})")
    ap.add_argument("--min-far", type=float, default=None,
                    help="deg the RISING wing must stay outboard of vertical "
                         f"(default {MIN_FAR_INBOARD_DEG})")
    ap.add_argument("--min-transmission", type=float, default=None,
                    help="worst transmission angle allowed [deg] "
                         f"(default {MIN_TRANSMISSION_DEG})")
    ap.add_argument("--max-half", type=float, default=None,
                    help="mm the STOWED mechanism may reach laterally")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--tag", default="",
                    help="suffix for default output names, so one config's "
                         "figures stay apart from another's")
    a = ap.parse_args()
    cfg = yaml.safe_load(a.config.read_text())

    def default_out(stem: str, ext: str) -> Path:
        return a.out or _plots_dir() / f"{stem}{a.tag}.{ext}"

    if a.check:
        cmd_check(cfg)
    elif a.optimize:
        cmd_optimize(cfg, a.save, a.seed, a.iters, a.torque_budget,
                     a.min_far, a.min_transmission, a.max_half)
    elif a.righting:
        if a.video:
            cmd_righting_video(cfg, a.out or Path("traces") /
                               f"swing_righting{a.tag}.mp4", a.fps, a.seconds)
        else:
            raise SystemExit("--righting currently needs --video")
    elif a.video:
        cmd_video(cfg, a.out or Path("traces") / f"swing_linkage{a.tag}.mp4",
                  a.fps, a.seconds)
    elif a.sweep:
        cmd_sweep(cfg, default_out("swing_linkage_sweep", "png"))
    else:
        cmd_rest(cfg, default_out("swing_linkage_rest", "png"))


if __name__ == "__main__":
    main()
