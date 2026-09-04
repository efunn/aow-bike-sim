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
    python analysis/swing_linkage.py --config <result>.yaml --videos --tag _mine

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

TWO REST POSES ARE REPRESENTABLE, and `mechanism.wing_angle_mode` picks:

    fixed          `wing_angle_from_rocker` is a number in the config, and the
                   rest pose is whatever the link lengths make it -- a splayed
                   V on both real geometries, 18.8 deg out on the hand-drawn
                   one and 20.1 on the first optimised one.
    vertical_rest  `wing_angle_from_rocker` is DERIVED so the panels stand
                   VERTICAL at rest, exactly as `wing_z_min` is derived so the
                   panel's lower edge sits at ground clearance. The stroke then
                   swings the rising wing INWARD over the chassis rather than
                   merely less-outward, and the panel's own splay stops paying
                   for width the bike carries all the time.

Vertical rest changes what `far_inboard_deg` means and therefore what bounds
it. Splayed, the rising wing never crosses vertical and `MIN_FAR_INBOARD_DEG`
is a positive margin; vertical, it crosses immediately and the constant becomes
a NEGATIVE ALLOWANCE -- how far in the wing may lean. What actually stops it is
not the angle at all but `clearance.panel_keepout`, the chassis volume the
panel may not enter. See `panel_keepout_gap`.

INTERFERENCE IS CHECKED, which it was not. Every metric here was a property of
one link or one panel, so nothing looked at whether two links pass through each
other -- and on a co-rotating pair sharing one crank body they can, which is
why the hand-drawn geometry was built rather than an optimised one. See
`min_link_gap`.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import NamedTuple

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

_MISS = object()
"""Sentinel for the pose memo. `None` is a REAL answer here -- it is the four-bar
failing to close -- so a plain `.get(key)` cannot tell a miss from a legitimate
"does not close", and would re-solve those every time."""

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


class _Bar:
    """Progress for the two commands slow enough to wonder about.

    tqdm WHEN IT IS THERE, a flushed one-liner when it is not. `tqdm` lives in
    the `rl` extra of pyproject.toml, not the base install, and this study has
    no business dragging a training dependency in -- so it is imported
    optionally and the fallback is the same `print(..., flush=True)` idiom the
    rest of analysis/ uses.

    OFF WHEN STDERR IS NOT A TTY, which is why the background runs in this
    repo's logs are clean. A bar redirected into a file is a few thousand lines
    of carriage returns, and every optimise run here gets redirected sooner or
    later.
    """

    off = False
    """Set by `--no-progress`. A bar is the first thing to rule out when a run
    misbehaves, and it should not take editing the file to do that."""

    def __init__(self, total: int, desc: str):
        self.total, self.desc, self.n, self.note = total, desc, 0, ""
        self.on = sys.stderr.isatty() and not _Bar.off
        self._t = None
        if self.on:
            try:
                from tqdm import tqdm
                self._t = tqdm(total=total, desc=desc, unit="it",
                               leave=False, file=sys.stderr)
            except ImportError:
                self._t = None

    def update(self, n: int = 1, note: str = "") -> None:
        self.n += n
        if note:
            self.note = note
        if not self.on:
            return
        if self._t is not None:
            if note:
                self._t.set_postfix_str(note, refresh=False)
            self._t.update(n)
        elif self.n % max(self.total // 40, 1) == 0:
            pct = 100.0 * self.n / max(self.total, 1)
            print(f"\r  {self.desc}: {pct:5.1f}%  {self.note}   ",
                  end="", file=sys.stderr, flush=True)

    def close(self) -> None:
        if not self.on:
            return
        if self._t is not None:
            self._t.close()
        else:
            print("\r" + " " * 78 + "\r", end="", file=sys.stderr, flush=True)


def _plots_dir():
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _rot(v, deg):
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


class KeepOut:
    """A rectangle the WING PANEL may not enter, in the front view.

    PANEL ONLY, and that is the whole reason this is a small list of boxes
    rather than a chassis model. The LINKS cannot be policed this way: the
    crank shaft sits ON the centreline and its arms sweep +-`crank_length`
    through it, so any keep-out wide enough to describe the chassis flags the
    crank on the first sample. What keeps the links honest is `min_link_gap`
    -- they occupy one thin fore/aft plane and the only thing in it is the
    servo carrying them.

    The panel is different: 90 mm deep in CAD, spanning fore/aft 60..150, so it
    sweeps across the drive servos, the battery corner, the power board and the
    Pi. Its keep-out is the union of those, projected into the front view.

    `half_width` is |y| and `z_lo`/`z_hi` are ABOVE THE FLOOR, matching what
    every report here prints. The sketch frame is converted on the way in.
    """

    __slots__ = ("name", "half_width", "z_lo", "z_hi")

    def __init__(self, name, half_width, z_lo, z_hi):
        self.name = name
        self.half_width = float(half_width)
        # null/None means UNBOUNDED. Both faces are real constraints and a
        # search will escape through either: measured, `_flat` takes its
        # clearance by passing UNDER z_lo at z = 80.0 and `_vertical` by
        # passing OVER z_hi at z = 189.0 against a box ending at 190. If the
        # chassis does not actually stop there, say so with null rather than
        # leaving the design a gap it cannot use in the real part.
        self.z_lo = -1e9 if z_lo is None else float(z_lo)
        self.z_hi = 1e9 if z_hi is None else float(z_hi)


_DEFAULT_PLANES = {"crank": 0, "coupler": 1, "rocker": 2, "panel": "all"}
"""Which fore/aft plane each member occupies.

THIS REPLACED A `cross_side` BOOLEAN that described the machine wrongly. The
boolean said "the two four-bars are staggered from each other", so it dropped
every left-to-right link pair -- INCLUDING coupler-vs-coupler, which is the one
pair that actually collides. (The scored constraint was never affected:
`coupler_gap` tests that pair directly and always did. What the boolean broke
was the full `min_link_gap` audit, which quietly stopped checking it.)

The real layout is per-MEMBER, not per-side: the two cranks share a plane, the
two couplers share a different one, and so on. So crank-vs-opposite-coupler
cannot touch however close they look in the front view, while coupler-vs-
opposite-coupler is a genuine interference in a shared plane.

`"all"` means the member spans every plane and is checked against everything.
The panel is 90 mm deep in CAD against links a few mm thick, so it is not in a
plane in any useful sense.
"""


def _planes(clearance: dict) -> dict:
    if "cross_side" in clearance:
        raise ValueError(
            "clearance.cross_side is gone -- it described the mechanism "
            "wrongly (see _DEFAULT_PLANES). Use clearance.planes, e.g.\n"
            "  planes: {crank: 0, coupler: 1, rocker: 2, panel: all}")
    out = dict(_DEFAULT_PLANES)
    out.update(clearance.get("planes") or {})
    return out


def _keepouts(clearance: dict) -> list:
    """The configured boxes, INFLATED by the panel's own width.

    `wing_width_mm` is added to every box's half-width rather than modelled on
    the panel, and that is a deliberate approximation. This study is a front
    view: the panel is a LINE in it, with no thickness to collide with. Growing
    the obstacle by the panel's half-width is the standard Minkowski trick and
    gives the right answer for a panel whose width is perpendicular to the
    view; it is wrong in detail for one that is swept at an angle, which is
    accepted -- it errs conservative, and the alternative is a 3D model this
    study is deliberately not.
    """
    grow = float(clearance.get("wing_width_mm", 0.0)) / 2.0
    return [KeepOut(k.get("name", f"box{i}"), k["half_width"] + grow,
                    k.get("z_lo"), k.get("z_hi"))
            for i, k in enumerate(clearance.get("panel_keepout", []) or [])]


def _circle_circle(p, r1, q, r2):
    """Both intersections of two circles, or [] when they do not meet."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    L = float(np.linalg.norm(d))
    if L < 1e-12 or L > r1 + r2 or L < abs(r1 - r2):
        return []
    a = (r1 * r1 - r2 * r2 + L * L) / (2.0 * L)
    h2 = r1 * r1 - a * a
    if h2 < 0.0:
        return []
    h = np.sqrt(h2)
    base = p + a * d / L
    perp = np.array([-d[1], d[0]]) / L
    return [base + h * perp, base - h * perp]


def _branch_sign(J, C, P) -> float:
    """Which way the coupler-rocker elbow bends, as +-1.

    THE assembly invariant. A four-bar cannot change this without coming apart,
    so it is what picks the reachable solution out of a circle-circle pair --
    and it works at a toggle, where the crank-coupler elbow has gone straight
    and cannot decide anything.
    """
    v, w = J - P, C - J
    return float(np.sign(v[0] * w[1] - v[1] * w[0]))


def _seg_seg_dist(a0, a1, b0, b1) -> float:
    """Closest approach of two 2D segments [mm].

    Exact, and by the short route: in 2D two segments either CROSS -- distance
    zero -- or their closest approach is attained at an endpoint of one of
    them. So an orientation test plus four point-to-segment distances is the
    whole answer, with no parametric interior solve to get the clamping wrong
    in. (The first version here did have it wrong: it clamped `sc` against a
    `tc` it then recomputed from the clamped `sc`, which is a single Gauss-
    Seidel sweep, not a projection, and it is only correct by accident.)

    Zero for a crossing and zero for a graze, deliberately: both are equally
    forbidden and the constraint is a POSITIVE gap, never zero.

    SCALAR FLOATS, NOT NUMPY, and that is not premature. `min_link_gap` calls
    this ~5600 times per objective evaluation and the optimiser makes ~20000 of
    those, so the numpy version -- eight small array allocations per call --
    put a 220-iteration search past an hour and made the whole constraint
    impractical to search under. Unpacking to floats is ~15x faster here and
    gives bit-identical answers; `--check` pins them.
    """
    ax0, ay0 = float(a0[0]), float(a0[1])
    ax1, ay1 = float(a1[0]), float(a1[1])
    bx0, by0 = float(b0[0]), float(b0[1])
    bx1, by1 = float(b1[0]), float(b1[1])

    ux, uy = ax1 - ax0, ay1 - ay0
    vx, vy = bx1 - bx0, by1 - by0
    d1 = ux * (by0 - ay0) - uy * (bx0 - ax0)
    d2 = ux * (by1 - ay0) - uy * (bx1 - ax0)
    d3 = vx * (ay0 - by0) - vy * (ax0 - bx0)
    d4 = vx * (ay1 - by0) - vy * (ax1 - bx0)
    if ((d1 > 0.0) != (d2 > 0.0)) and ((d3 > 0.0) != (d4 > 0.0)):
        return 0.0

    best = 1e9
    for px, py, qx, qy, ex, ey in (
            (ax0, ay0, bx0, by0, vx, vy), (ax1, ay1, bx0, by0, vx, vy),
            (bx0, by0, ax0, ay0, ux, uy), (bx1, by1, ax0, ay0, ux, uy)):
        L2 = ex * ex + ey * ey
        t_ = 0.0 if L2 < 1e-12 else ((px - qx) * ex + (py - qy) * ey) / L2
        t_ = 0.0 if t_ < 0.0 else (1.0 if t_ > 1.0 else t_)
        dx, dy = px - (qx + t_ * ex), py - (qy + t_ * ey)
        d_ = dx * dx + dy * dy
        if d_ < best:
            best = d_
    return float(np.sqrt(best))


def _point_box_sdf(y: float, z: float, box: KeepOut) -> float:
    """Signed distance from (|y|, z) to a keep-out box [mm].

    Positive outside, negative inside (the penetration depth). Mirrored in y
    because the boxes are centred on the centreline and the panel works both
    sides.
    """
    qy = abs(y) - box.half_width
    qz = box.z_lo - z
    if z - box.z_hi > qz:
        qz = z - box.z_hi
    oy = qy if qy > 0.0 else 0.0
    oz = qz if qz > 0.0 else 0.0
    out = (oy * oy + oz * oz) ** 0.5
    m = qy if qy > qz else qz
    return out + (m if m < 0.0 else 0.0)


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

    # WHAT THE CONFIG ACTUALLY CONTAINS, because it is four different kinds of
    # thing in one file and that is confusing until it is written down:
    #
    #   INPUT      `bike`, `limits`, `clearance`, and the panel features in
    #              `mechanism` -- wing_angle_mode, wing_norm_offset, wing_z_max,
    #              wing_pivot_z. These are the constraints. A file with only
    #              these is a complete input to `--optimize`.
    #   SEARCHED   the six in `_VARS`. `--optimize` OVERWRITES them, so their
    #              value on input is a seed for the "start:" line and nothing
    #              more. Omit them and `_SEED_LENGTHS` stands in.
    #   DERIVED    `wing_z_min` always, and `wing_angle_from_rocker` under
    #              wing_angle_mode vertical_rest / flat_deploy. Written back on
    #              load so a saved config carries the resolved number.
    #   OUTPUT     `stroke.crank_travel_deg`, written by `--save`.
    #
    # A config saved from `--optimize` is a full mechanism, because build_model
    # reads it as one. A config written BY HAND to pose a question need only
    # carry the INPUT rows.

    _SEED_LENGTHS = {
        "wing_pivot_x": 18.0, "crank_length": 32.0, "coupler_length": 82.0,
        "rocker_length": 44.5, "angle_between_cranks": 45.0,
        "servo_offset": 83.5,
    }
    """Stand-ins for the searched variables when a constraints-only file omits
    them. The hand-drawn geometry's numbers, chosen because they are known to
    assemble -- a seed that does not close makes `--optimize` print a useless
    "start:" line before searching perfectly well anyway."""

    _PANEL_DEFAULTS = {"wing_pivot_z": 0.0, "wing_norm_offset": 5.0,
                       "wing_z_max": 80.0, "wing_z_min": -20.0,
                       "wing_angle_from_rocker": 15.0,
                       "wing_angle_mode": "fixed"}

    def __init__(self, cfg: dict):
        b = cfg["bike"]
        m = cfg.setdefault("mechanism", {})
        for k, v in self._SEED_LENGTHS.items():
            m.setdefault(k, v)
        for k, v in self._PANEL_DEFAULTS.items():
            m.setdefault(k, v)
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
        # `rest_joint` is a pure function of the link lengths and of the shaft
        # and pivot positions -- none of which move after this point -- but
        # `solve` calls it once per solve for its reference angle, which the
        # profile showed as 25050 circle-circle solves per objective
        # evaluation. Cached HERE: after the four-bar is set, before the two
        # panel derivations below, both of which want it.
        self._rest_j: dict[int, np.ndarray] = {
            side: self._rest_joint_raw(side) for side in (-1, 1)}
        self._branch: dict[int, float] = {
            side: _branch_sign(self._rest_j[side], self.crank_tip(side, 0.0),
                               self.pivot(side)) for side in (-1, 1)}
        self._pose_memo: dict[tuple, object] = {}
        self._toggle_memo: dict[tuple, object] = {}
        # ORDER MATTERS: the panel bearing has to be settled before the panel
        # bottom is solved, because `_derive_panel_bottom` projects along the
        # panel's own axis and that axis is what this sets.
        self.wing_angle_mode = m.get("wing_angle_mode", "fixed")
        if self.wing_angle_mode == "vertical_rest":
            self._derive_wing_angle(0.0, "rest")
        elif self.wing_angle_mode == "flat_deploy":
            self._derive_wing_angle(90.0, "deploy")
        elif self.wing_angle_mode != "fixed":
            raise ValueError(
                f"wing_angle_mode: expected 'fixed', 'vertical_rest' or "
                f"'flat_deploy', got {self.wing_angle_mode!r}")
        self._derive_panel_bottom(b["ground_clearance"])
        self._derive_panel_top(m)
        self.clearance = cfg.get("clearance", {}) or {}
        self.keepout = _keepouts(self.clearance)
        self.planes = _planes(self.clearance)
        # The couplers pass each other near the end of the stroke, in whatever
        # plane the stagger puts them; this is the width they need there.
        self.coupler_width = float(
            self.clearance.get("coupler_width_mm", MIN_LINK_GAP_MM))
        # UNSATISFIABLE ABOVE `wing_norm_offset`, and silently so before this
        # check. The coupler ends at the rocker joint and the panel is offset
        # `wing_norm_offset` normal to the rocker, so the coupler-to-panel gap
        # is pinned at EXACTLY that value by construction -- no link length
        # moves it. Since `min_link_gap` scores every pair against
        # `coupler_width_mm`, asking for more than the offset makes every
        # geometry in the space infeasible and the search reports a stranded
        # run with no hint why.
        if self.coupler_width > self.wing_norm + 1e-9:
            raise ValueError(
                f"clearance.coupler_width_mm ({self.coupler_width}) exceeds "
                f"mechanism.wing_norm_offset ({self.wing_norm}). The coupler "
                f"passes the panel at exactly wing_norm_offset by "
                f"construction, so this can never be satisfied by any link "
                f"lengths. Raise wing_norm_offset to at least "
                f"{self.coupler_width} as well.")
        # Branch memory, as in the mirrored study: a four-bar that flips branch
        # has physically come apart, so every solve is continued from the last.
        self._last: dict[int, np.ndarray] = {}
        self._limit: float | None = None      # assembly_limit memo; geometry
                                              # is immutable after __init__
        self._seg_memo: dict[tuple, list] = {}
        self._angles_memo = None
        self._eval_memo: dict = {}
        self.reset()

    def _derive_wing_angle(self, off_vertical_deg: float, at: str) -> None:
        """Set `wing_angle_from_rocker` to pin the panel's attitude at ONE pose.

        One linear solve, not a search, for the same reason `wing_z_min` is
        derived rather than searched: the panel bearing is a CONSEQUENCE of the
        link lengths once you say what attitude you want and where.

            wa       = rocker_bearing + side * wing_angle_from_rocker
            =>  wing_angle_from_rocker = side * (rocker_bearing - wa_wanted)

        evaluated on side -1, where `side * x` is `-x`.

        ONE POSE, NOT TWO, and that is the whole trade between the modes. The
        rocker swings by a fixed amount between rest and the deployed toggle --
        a link-length property -- so pinning the panel upright at rest FIXES
        what it does at full deployment, and pinning it flat at full deployment
        fixes the parked attitude. They cannot both be chosen:

            vertical_rest   panel upright when parked. The narrowest possible
                            parked slab, at whatever hand-off roll falls out.
            flat_deploy     panel flat at the DEPLOYED TOGGLE. Hand-off roll is
                            then 0 by construction, the commanded stroke IS the
                            toggle, and the fourth angle disappears -- but the
                            parked attitude is whatever the rocker swing leaves.
            fixed           neither pinned; the number in the config is used.

        `at` picks which pose is pinned, `off_vertical_deg` what to pin it to
        (0 = upright, 90 = flat).

        WRITTEN BACK INTO THE CONFIG, which is the load-bearing half. The
        MuJoCo builder reads `mechanism.wing_angle_from_rocker` as a plain
        number (build_model.py) and knows nothing about modes -- so a config
        saved from an optimise run has to carry the RESOLVED angle or the sim
        and this study silently model different mechanisms. Resolving it here
        makes `--save` emit it for free, and re-deriving from a saved config is
        idempotent because the pose it solves for is already met.
        """
        P = self.pivot(-1)
        if at == "rest":
            J = self.rest_joint(-1)
        else:
            tog = self.toggle(-1, "extended")
            if tog is None:            # no reachable deployed limit; leave it
                return
            J = tog[1]
        r = J - P
        bearing = float(np.degrees(np.arctan2(r[1], r[0])))
        want = 90.0 - off_vertical_deg      # panel bearing: 90 up, 0 flat
        # Two solutions 180 apart, because a panel is a LINE. Take the one that
        # leaves the panel pointing UP at rest, which is what makes `foot` the
        # low end and lets `_derive_panel_bottom` place it at ground clearance.
        rest_bearing = float(np.degrees(np.arctan2(
            *(self.rest_joint(-1) - P)[::-1])))
        best = None
        for cand in (bearing - want, bearing - want - 180.0):
            wa_rest = np.deg2rad(rest_bearing - cand)
            if np.sin(wa_rest) > 0.0 and (best is None or abs(cand) < abs(best)):
                best = cand
        if best is None:
            best = bearing - want
        # Normalised to (-180, 180]: the raw solve comes out unwrapped (-342.9
        # on the hand-drawn lengths) and that number goes straight into a saved
        # config and from there into build_model, where it reads as a mistake.
        self.wing_from_rocker = float((best + 180.0) % 360.0 - 180.0)
        self.cfg["mechanism"]["wing_angle_from_rocker"] = self.wing_from_rocker

    def _panel_axis_at_rest(self):
        """(origin, direction) of the panel at rest, in sketch coordinates."""
        joint = self.rest_joint(-1)
        p = self.pivot(-1)
        r = joint - p
        rdir = r / float(np.linalg.norm(r))
        wa = np.arctan2(rdir[1], rdir[0]) - np.deg2rad(self.wing_from_rocker)
        w = np.array([np.cos(wa), np.sin(wa)])
        n = np.array([w[1], -w[0]])
        return joint - self.wing_norm * n, w

    def _axis_coord_for_height(self, above_floor: float):
        """Panel-axis coordinate that puts a panel point at this height, or None.

        `wing_z_min` and `wing_z_max` are coordinates ALONG THE PANEL, measured
        from an origin that hangs off the rocker joint -- so the same number
        means a different height on every geometry, and the panel that number
        describes is a different LENGTH on every geometry. Measured across the
        six tracked configs at an identical `wing_z_max: 80`, panel length runs
        86.8 to 109.0 mm and the top sits anywhere from 142.7 to 170.9 mm above
        the floor. Comparing two designs' widths without fixing this compares
        two different wings.

        This is the conversion that lets a config ask for a HEIGHT instead.
        """
        origin, w = self._panel_axis_at_rest()
        if abs(w[1]) < 1e-6:
            return None                                   # panel horizontal
        return float((above_floor - self.wheel_radius - origin[1]) / w[1])

    def _derive_panel_bottom(self, ground_clearance: float) -> None:
        """Set `wing_z_min` so the panel's lower edge sits at ground clearance.

        Solved at the REST pose, where the panel is symmetric. The panel origin
        and direction come from the rocker, so this is one linear solve rather
        than a search.
        """
        got = self._axis_coord_for_height(ground_clearance)
        if got is not None:
            self.wing_z_min = got

    def _derive_panel_top(self, m: dict) -> None:
        """Set `wing_z_max` from whichever basis `panel_span_mode` names.

            z_max        use `wing_z_max` as written -- the axis coordinate.
                         Both the panel's LENGTH and its top HEIGHT then float
                         with the linkage, which is the historical behaviour
                         and the wrong basis for comparing two designs.
            length       `panel_length_mm` above the derived bottom. Same wing
                         on every candidate; the top height floats.
            top_height   `panel_top_z_mm` above the floor at rest. Same reach
                         over the CoM on every candidate; the length floats.

        Runs AFTER `_derive_panel_bottom`, because `length` is measured from it.
        """
        mode = m.get("panel_span_mode", "z_max")
        if mode == "z_max":
            return
        if mode == "length":
            want = m.get("panel_length_mm")
            if want is None:
                raise ValueError("panel_span_mode: length needs "
                                 "mechanism.panel_length_mm")
            self.wing_z_max = self.wing_z_min + float(want)
        elif mode == "top_height":
            want = m.get("panel_top_z_mm")
            if want is None:
                raise ValueError("panel_span_mode: top_height needs "
                                 "mechanism.panel_top_z_mm")
            got = self._axis_coord_for_height(float(want))
            if got is not None:
                self.wing_z_max = got
        else:
            raise ValueError(f"panel_span_mode: expected 'z_max', 'length' or "
                             f"'top_height', got {mode!r}")
        # Written back for the same reason the panel bearing is: build_model
        # reads `wing_z_max` as a plain number and knows nothing about modes.
        self.cfg["mechanism"]["wing_z_max"] = self.wing_z_max

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
        hit = self._rest_j.get(side)
        return self._rest_joint_raw(side) if hit is None else hit

    def _rest_joint_raw(self, side: int) -> np.ndarray:
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

    # -- the three angles that decide the mechanism ------------------------
    #
    # THE WHOLE KINEMATIC STORY IS THREE POSES, and every one of them is a
    # closed-form circle-circle solve rather than something to walk to. This
    # replaced dense sweeps that cost 810 `pose()` calls per objective
    # evaluation, measured; the profile said the walking WAS the cost.
    #
    #   1. REST, crank input 0. Symmetric by construction. Fixes the parked
    #      envelope and, with the panel bearing, the rest attitude.
    #   2. The RISING wing's most inboard excursion. Occurs where that side's
    #      crank and coupler go collinear OVERLAPPING (folded back on each
    #      other) -- the rocker's near limit. Decides the chassis keep-out.
    #   3. The DEPLOYED wing's final angle. That side's crank and coupler
    #      collinear EXTENDED -- the rocker's far limit. Decides the final wing
    #      angle, the servo travel, and whether the two couplers foul.
    #
    # Both limits are the rocker's toggle positions, and they are where they
    # are because at crank-coupler collinear the crank's motion is
    # instantaneously along the coupler and drives no rocker rotation. Checked
    # against a 0.25 deg walk on all five tracked geometries: the folded toggle
    # reproduces the rising wing's most inboard pose to 0.2 deg on every one.
    #
    # WHAT THIS DOES NOT COVER, and it is not a rounding error: peak servo
    # torque is never at any of the three -- see `peak_torque`.

    def toggle(self, side: int, kind: str):
        """Crank travel and joint at a crank+coupler collinear pose, or None.

        `kind` is "extended" (crank and coupler in line, |SJ| = a + b) or
        "folded" (coupler doubled back over the crank, |SJ| = |b - a|).

        Returns the SMALLEST POSITIVE travel on the assembled branch. Positive,
        because the mechanism is mirror-symmetric and only ever commanded one
        way -- the other direction is this one reflected.
        """
        key = (side, kind)
        if key in self._toggle_memo:
            return self._toggle_memo[key]
        a, b, c = self.crank, self.coupler, self.rocker
        reach, sgn = ((a + b, 1.0) if kind == "extended" else (abs(b - a), -1.0))
        P = self.pivot(side)
        want = _branch_sign(self.rest_joint(side), self.crank_tip(side, 0.0), P)
        base = 90.0 - side * self.between / 2.0
        best = None
        for J in _circle_circle(self.shaft, reach, P, c):
            u = J - self.shaft
            n = float(np.linalg.norm(u))
            if n < 1e-9:
                continue
            u = u / n
            # C is at `a` from the shaft ALONG the coupler line: forward for
            # the extended pose, backward for the folded one. Both give
            # C - J = -b * u, which is why one branch test serves for both.
            C = self.shaft + sgn * a * u
            if _branch_sign(J, C, P) != want:
                continue
            phi = np.degrees(np.arctan2(sgn * u[1], sgn * u[0]))
            t = (phi - base) % 360.0
            if best is None or t < best[0]:
                best = (float(t), J)
        self._toggle_memo[key] = best
        return best

    def flat_travel(self, side: int):
        """Crank travel that lays the PANEL horizontal, or None.

        Closed form for the same reason the toggles are: the panel is rigid
        with the rocker, so asking for a panel bearing names a rocker bearing,
        which names the joint, which names the crank tip by one more
        circle-circle.

        Needed because the stroke you would COMMAND is not always the stroke
        the mechanism can reach. The deployed toggle is the far limit; if the
        panel passes through horizontal before it, driving further only rolls
        the bike back off the wing. Two of the five tracked geometries do pass
        through it early (the hand-drawn one at 112.0 of a 125.4 limit, and
        `_compact` at 106.6 of 138.5), so this is the common case, not an edge.
        """
        a, b, c = self.crank, self.coupler, self.rocker
        P = self.pivot(side)
        want = _branch_sign(self.rest_joint(side), self.crank_tip(side, 0.0), P)
        base = 90.0 - side * self.between / 2.0
        out = []
        # panel bearing = rocker bearing + side * wing_from_rocker, and the
        # panel is a LINE: 0 and 180 are both horizontal.
        for target in (0.0, 180.0):
            th = np.deg2rad(target - side * self.wing_from_rocker)
            J = P + c * np.array([np.cos(th), np.sin(th)])
            for C in _circle_circle(self.shaft, a, J, b):
                if _branch_sign(J, C, P) != want:
                    continue
                v = C - self.shaft
                out.append((np.degrees(np.arctan2(v[1], v[0])) - base) % 360.0)
        return float(min(out)) if out else None

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
        # BRANCH BY THE ASSEMBLY INVARIANT, not by nearest-previous-solution.
        # A four-bar cannot change the sign of its coupler-rocker elbow without
        # coming apart, so that sign picks the reachable solution outright --
        # the same test `toggle` and `rocker_travels` already use.
        #
        # This REPLACED continuation from the last solve, which made `pose` a
        # function of call ORDER as well as of (side, travel). That was fine
        # while every metric walked in small steps from rest and is a hazard
        # now that they jump straight to closed-form candidate travels tens of
        # degrees apart: nearest-to-reference can pick the folded-through
        # solution when the reference is far away. It also blocked memoising,
        # because the same (side, travel) could legitimately return two
        # different answers.
        #
        # Continuation is kept as the fallback for the degenerate case where
        # the sign test cannot separate them (both solutions on the same side,
        # i.e. at or through a singularity).
        cands = [base + h * perp, base - h * perp]
        want = self._branch[side]
        ok = [q for q in cands if _branch_sign(q, c, p) == want]
        if len(ok) == 1:
            joint = ok[0]
        else:
            ref = self._last[side] if side in self._last else self.rest_joint(side)
            joint = min(cands, key=lambda q: float(np.linalg.norm(q - ref)))
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
        """Everything about one side at one crank angle, or None.

        ONE FOUR-BAR, one side. The whole mechanism at a given crank angle is
        two of these -- the deploying linkage and the rising one are different
        configurations of the same link lengths, and neither is the other's
        mirror at the SAME travel.

        Memoised, which is only sound because `solve` now picks its branch by
        the assembly invariant rather than by continuation: (side, travel) has
        one answer regardless of what was solved before it. Several metrics ask
        for the same candidate travels -- `far_inboard_deg` and `panel_extents`
        wanted an identical three poses on the rising side, and `coupler_gap`
        and `_segment_poses` overlapped on six more.
        """
        key = (side, round(travel_deg, 9))
        hit = self._pose_memo.get(key, _MISS)
        if hit is not _MISS:
            return hit
        out = self._pose_uncached(side, travel_deg)
        self._pose_memo[key] = out
        return out

    def _pose_uncached(self, side: int, travel_deg: float):
        joint, wd = self.solve(side, travel_deg)
        if joint is None:
            return None
        foot, top = self.wing_points(side, joint)
        return {"joint": joint, "wing_deg": wd, "foot": foot, "top": top,
                "crank_tip": self.crank_tip(side, travel_deg),
                "pivot": self.pivot(side)}


# --------------------------------------------------------------------------
# the three angles, and the candidate travels every metric is evaluated at


class Angles(NamedTuple):
    """The crank travels that decide this mechanism. Degrees, all positive.

    ONE DIRECTION ONLY. The pair is mirror-symmetric by construction -- shared
    link lengths, arms at +-between/2 -- so side L at travel -t IS side R at
    +t, reflected. Sweeping both ways was checking the same machine twice: for
    every pair (crankR, couplerL) at +t there is (crankL, couplerR) at -t, and
    both sides are already evaluated at every +t. The earlier version of this
    file argued the opposite in a docstring and swept both; that was wrong, and
    it doubled the cost of every clearance metric.
    """

    rest: float       # 0.0, always. The symmetric parked pose.
    fold: float       # rising side's crank+coupler collinear OVERLAPPING
    deploy: float     # deploying side's crank+coupler collinear EXTENDED
    command: float    # the travel actually driven: min(deploy, panel-flat)


def critical_angles(lk: SwingLinkage) -> Angles:
    """Rest, the rising wing's inboard limit, and the deployed limit.

    `command` is min(deploy, flat_travel) because the deployed toggle is what
    the mechanism CAN reach and the flat-panel pose is what you would ASK for:
    past horizontal the bike rolls back off the wing it is standing on, so more
    travel is not more righting. Two of the five tracked geometries reach
    horizontal first (hand-drawn at 112.0 of a 125.4 limit, `_compact` at 106.6
    of 138.5), so both cases are live.

    Returns zeros for `fold`/`deploy` when the toggle does not exist -- a
    four-bar whose rocker cannot reach its limit is not assemblable through the
    stroke, and callers read `command == 0` as "unusable".
    """
    hit = lk._angles_memo
    if hit is not None:
        return hit
    dep = lk.toggle(-1, "extended")
    fol = lk.toggle(1, "folded")
    if dep is None:
        out = Angles(0.0, 0.0 if fol is None else fol[0], 0.0, 0.0)
    else:
        flat = lk.flat_travel(-1)
        cmd = dep[0] if flat is None else min(dep[0], flat)
        out = Angles(0.0, 0.0 if fol is None else fol[0], dep[0], float(cmd))
    lk._angles_memo = out
    return out


def rocker_travels(lk: SwingLinkage, side: int, rocker_deg: float) -> list:
    """Crank travels putting the ROCKER at a given bearing, assembled branch.

    The inverse of the four-bar, and the reason so much here is closed form:
    the panel is RIGID with the rocker, so any question of the form "where is
    the panel at angle X" is a rocker bearing, which names the joint, which
    names the crank tip by one more circle-circle.
    """
    P = lk.pivot(side)
    want = _branch_sign(lk.rest_joint(side), lk.crank_tip(side, 0.0), P)
    base = 90.0 - side * lk.between / 2.0
    th = np.deg2rad(rocker_deg)
    J = P + lk.rocker * np.array([np.cos(th), np.sin(th)])
    out = []
    for C in _circle_circle(lk.shaft, lk.crank, J, lk.coupler):
        if _branch_sign(J, C, P) != want:
            continue
        v = C - lk.shaft
        out.append(float((np.degrees(np.arctan2(v[1], v[0])) - base) % 360.0))
    return out


def _rocker_bearing(lk: SwingLinkage, side: int, travel: float):
    pz = lk.pose(side, travel)
    if pz is None:
        return None
    r = pz["joint"] - pz["pivot"]
    return float(np.degrees(np.arctan2(r[1], r[0])))


def panel_stationary_travels(lk: SwingLinkage, side: int, hi: float) -> list:
    """Travels where a panel end's |y| stops changing, within [0, hi].

    A panel end is a fixed vector on the rocker, so its lateral coordinate is
    `pivot_y + R cos(theta4 + psi)` -- a pure sinusoid in the ROCKER bearing.
    Its stationary bearings are therefore -psi and -psi+180 exactly, and
    `rocker_travels` maps each back to a crank travel.

    This is what makes the protrusion and keep-out numbers trustworthy without
    a sweep: the interval ends are in the candidate set already, and these are
    the only interior places either can turn around.
    """
    # From the CACHED rest joint, not from a solve: the panel's ends are fixed
    # vectors on the rocker, so their offsets are `wing_points` of the rest
    # joint and need no pose at all. Solving for them cost four poses per call,
    # and this is called from every `evaluation_travels`.
    P = lk.pivot(side)
    j0 = lk.rest_joint(side)
    r0 = j0 - P
    th0 = np.arctan2(r0[1], r0[0])
    ends = dict(zip(("foot", "top"), lk.wing_points(side, j0)))
    out = []
    for key in ("foot", "top"):
        d = ends[key] - P
        c_, s_ = np.cos(-th0), np.sin(-th0)
        local = np.array([c_ * d[0] - s_ * d[1], s_ * d[0] + c_ * d[1]])
        psi = float(np.degrees(np.arctan2(local[1], local[0])))
        for target in (-psi, -psi + 180.0):
            out.extend(rocker_travels(lk, side, target))
    return [t for t in out if -1e-9 <= t <= hi + 1e-9]


def transmission_travels(lk: SwingLinkage, side: int, hi: float) -> list:
    """Travels where the transmission angle can be extremal, within [0, hi].

    Closed form, and exactly so. The angle at the joint between coupler and
    rocker has cos(mu) = (b^2 + c^2 - L^2) / (2bc), where L is crank-tip to
    pivot -- and L^2 = g^2 + a^2 + 2 a g cos(crank - ground bearing) is a plain
    sinusoid in the crank angle. So the unfolded mu is MONOTONE in L and can
    only turn where L does; the fold to min(mu, 180-mu) adds one more place,
    where mu crosses 90.

    Candidates: the interval ends, the two crank angles that make L extremal,
    and the mu = 90 crossings. Checked against a 0.05 deg walk on all five
    tracked geometries -- worst disagreement 0.013 deg, and that is the walk's
    own quantisation at the window edge.
    """
    a, b, c = lk.crank, lk.coupler, lk.rocker
    D = lk.shaft - lk.pivot(side)
    g = float(np.linalg.norm(D))
    if g < 1e-9 or a < 1e-9:
        return [0.0, hi]
    phi_d = float(np.degrees(np.arctan2(D[1], D[0])))
    base = 90.0 - side * lk.between / 2.0
    cand = [0.0, hi]
    for k in (0.0, 180.0):                      # L at its max and min
        cand.append((phi_d + k - base) % 360.0)
    cs = (b * b + c * c - g * g - a * a) / (2.0 * a * g)     # mu = 90
    if abs(cs) <= 1.0:
        d = float(np.degrees(np.arccos(cs)))
        cand += [(phi_d + d - base) % 360.0, (phi_d - d - base) % 360.0]
    return sorted({round(t, 9) for t in cand if -1e-9 <= t <= hi + 1e-9})


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


def evaluation_travels(lk: SwingLinkage, side: int, hi: float | None = None) -> list:
    """Every crank travel a geometric metric can be extremal at, for one side.

    The three critical angles, plus the travels where either panel end's |y|
    stops changing. That second set is what makes "max over the stroke" honest
    without a sweep: a panel end is a fixed vector on the rocker, so its
    lateral coordinate is a pure sinusoid in the rocker bearing and can turn
    around in exactly two places, both closed form.

    `--check --verify` walks the same metrics at 0.25 deg and prints the
    disagreement, so the claim is auditable rather than asserted.
    """
    A = critical_angles(lk)
    if hi is None:
        hit = lk._eval_memo.get(side)
        if hit is not None:
            return hit
    hi = A.command if hi is None else hi
    cand = {0.0, hi}
    for t in (A.fold, A.deploy, A.command):
        if 0.0 <= t <= hi + 1e-9:
            cand.add(float(t))
    cand.update(panel_stationary_travels(lk, side, hi))
    out = sorted(cand)
    if abs(hi - A.command) < 1e-12:
        lk._eval_memo[side] = out
    return out


def panel_extents(lk: SwingLinkage, travel: float | None = None, step: float = 2.0):
    """Non-deployed wing's widest |y|: (rest, end-of-stroke, max, where) [mm].

    THE TWO ENDPOINTS ARE WHAT IS SCORED, equally: the symmetric rest pose the
    bike drives in, and the fully-deployed pose it sits in during a shot. The
    max is reported and plotted as a safety check but NOT scored -- it is
    dominated by whichever endpoint is worse and is blind to the other, which
    is how a design that ended at 74 mm beat one that ended at 37 mm on an
    identical peak.

    THE thing to minimise on this mechanism. There is no width envelope to fit
    inside -- the wings may protrude -- so the goal is simply that they stick
    out as little as possible, at both poses the bike actually holds.

    WAS `top_extents`, measuring the panel's TOP corner alone. That was a proxy
    that held only while the rest pose was a splayed V; under
    `wing_angle_mode: vertical_rest` the rising panel's top comes IN as its
    foot goes OUT, and a top-only score reads a widening design as narrowing.

    AND IT WAS A SWEEP. Now three named poses plus the two closed-form turning
    points per panel end -- `evaluation_travels`. `travel` and `step` are
    accepted and ignored so existing call sites keep working.
    """
    A = critical_angles(lk)
    if A.command <= 0.0:
        return 1e3, 1e3, 1e3, 0.0
    rest = end = None
    worst, at = 0.0, 0.0
    for t in evaluation_travels(lk, 1):
        lk.reset()
        pz = lk.pose(1, t)
        if pz is None:
            continue
        y = max(abs(float(pz["top"][0])), abs(float(pz["foot"][0])))
        if t <= 1e-9:
            rest = y
        if abs(t - A.command) < 1e-6:
            end = y
        if y > worst:
            worst, at = y, t
    if rest is None or end is None:
        return 1e3, 1e3, 1e3, 0.0
    return rest, end, worst, at


def rest_wing_deg(lk: SwingLinkage) -> float:
    """Panel angle from vertical at the REST pose [deg], positive outboard.

    The quantity `wing_angle_mode: vertical_rest` drives to zero. Reported
    under BOTH modes, because under `vertical_rest` it is the cheap
    confirmation that the derivation did what it claims, and under `fixed` it
    is how much rest width the panel's own splay is costing: the hand-drawn
    geometry sits at 18.8 deg out and the first optimised one at 20.1.

    Measured on the rising side (+y) so the sign reads as it does everywhere
    else here -- positive is away from the centreline.
    """
    lk.reset()
    pz = lk.pose(1, 0.0)
    if pz is None:
        return 90.0
    v = pz["top"] - pz["foot"]
    return float(np.degrees(np.arctan2(v[0], v[1])))


def far_inboard_deg(lk: SwingLinkage, travel: float | None = None,
                    step: float = 2.0) -> float:
    """Least angle the RISING wing's panel makes with vertical [deg].

    Positive is outboard; NEGATIVE means the panel has swung past vertical and
    is leaning in over the chassis. That crossing is the real limit on this
    mechanism, and it is angular, not lateral -- an earlier version of this
    file constrained the far wing's DISTANCE from the centreline in mm, which
    is a different quantity that is neither necessary nor sufficient.

    ONE POSE, not a sweep. The far arm does not sweep monotonically inward: it
    goes in, reaches a minimum, and comes back out. That minimum is where the
    rising side's crank and coupler go COLLINEAR OVERLAPPING -- the rocker's
    near toggle -- and `critical_angles(lk).fold` is that travel in closed
    form. Verified against a 0.25 deg walk on all five tracked geometries: the
    toggle reproduces the walk's minimum to 0.2 deg, which is the walk's own
    step.

    The minimum is fixed by the LINK LENGTHS ALONE. Measured, holding the
    lengths and sweeping only `angle_between_cranks`, it is -0.2 deg at every
    value from 20 to 90 -- the angle changes the stroke LENGTH and the rest
    setpoint, not how far in the far arm reaches. So the never-inward property
    is a pure LENGTH constraint and the angle is free afterwards to place the
    rest pose.
    """
    A = critical_angles(lk)
    if A.command <= 0.0:
        return -1e3
    worst = 1e9
    for t in (0.0, A.fold, A.command):
        lk.reset()
        pz = lk.pose(1, t)
        if pz is None:
            continue
        v = pz["top"] - pz["foot"]
        worst = min(worst, float(np.degrees(np.arctan2(v[0], v[1]))))
    return worst if worst < 1e9 else -1e3


def foot_outboard_mm(lk: SwingLinkage, travel: float | None = None,
                     step: float = 2.0) -> float:
    """Least margin between the deploying foot and its OWN pivot [mm].

    Negative means the panel has swung inboard PAST its own hinge, which is
    geometric nonsense in a real part: the panel would have to pass through the
    bracket carrying it. Nothing else here catches it -- a design doing this
    can still satisfy reach, hand-off, torque and the far-wing angle, and one
    did (foot to |y| 0.2 mm against a pivot at 8.0 mm).

    Candidate travels, not a sweep: the foot's |y| is a sinusoid in the rocker
    bearing, so it turns around in two closed-form places and is otherwise
    monotone between the stroke's ends.
    """
    if critical_angles(lk).command <= 0.0:
        return -1e3
    py = abs(float(lk.pivot(-1)[0]))
    worst = 1e9
    for t in evaluation_travels(lk, -1):
        lk.reset()
        pz = lk.pose(-1, t)
        if pz is None:
            continue
        worst = min(worst, abs(float(pz["foot"][0])) - py)
    return worst if worst < 1e9 else -1e3


def far_clearance(lk: SwingLinkage, travel: float | None = None,
                  step: float = 1.0) -> float:
    """Closest the RISING wing gets to the centreline over the stroke [mm].

    Reported alongside `far_inboard_deg` for orientation only -- the angle is
    what is scored, and the two do not rank candidates alike. Candidate
    travels, same reasoning as `foot_outboard_mm`.
    """
    if critical_angles(lk).command <= 0.0:
        return 0.0
    worst = 1e9
    for t in evaluation_travels(lk, 1):
        lk.reset()
        pz = lk.pose(1, t)
        if pz is None:
            continue
        for q in (pz["joint"], pz["foot"], pz["top"]):
            worst = min(worst, abs(float(q[0])))
    return worst if worst < 1e9 else 0.0


def _segment_poses(lk: SwingLinkage, travels=None):
    """[(travel, named segments for BOTH sides)] at the candidate travels.

    ONE DIRECTION. The pair is mirror-symmetric by construction, and both sides
    are evaluated at every travel, so for every pair (crankR, couplerL) at +t
    the mirror pair (crankL, couplerR) is checked at the same +t. An earlier
    version swept both directions on the argument that they see different
    pairs; they see the same pairs reflected, and it was doing twice the work.
    """
    # A scalar means an old caller passing the stroke length. It is not a
    # travel list, and the candidate set already ends at the commanded stroke,
    # so it is ignored rather than misread as a one-pose sweep.
    if travels is None or not hasattr(travels, "__len__"):
        travels = sorted(set(evaluation_travels(lk, -1))
                         | set(evaluation_travels(lk, 1)))
    key = tuple(round(t, 6) for t in travels)
    hit = lk._seg_memo.get(key)
    if hit is not None:
        return hit
    rows = []
    for t in travels:
        segs = []
        lk.reset()
        for side in (-1, 1):
            one = link_segments(lk, side, t)
            if one is None:
                segs = []
                break
            segs.extend(one)
        if segs:
            rows.append((t, segs))
    lk._seg_memo[key] = rows
    return rows


def link_segments(lk: SwingLinkage, side: int, travel_deg: float):
    """The four drawn members of one side, as ((name, p, q), ...).

    Exactly what `_draw` puts on the page, so a gap this reports and a crossing
    someone sees in the figure are the same event. Sketch coordinates.
    """
    pz = lk.pose(side, travel_deg)
    if pz is None:
        return None
    tag = "R" if side < 0 else "L"
    return (
        (f"crank{tag}",   lk.shaft,        pz["crank_tip"]),
        (f"coupler{tag}", pz["crank_tip"], pz["joint"]),
        (f"rocker{tag}",  pz["pivot"],     pz["joint"]),
        (f"panel{tag}",   pz["foot"],      pz["top"]),
    )


# Pairs that share a pin or a rigid body, and so are ALLOWED to touch.
# Everything not listed here is checked.
#
#   crank-coupler    share the crank-tip pin
#   coupler-rocker   share the rocker joint
#   rocker-panel     ONE rigid body, offset `wing_norm_offset` by design
#
# COUPLER-PANEL IS NOT ADJACENT, and listing it here was a mistake. They are
# connected only THROUGH the wing body -- the coupler ends at the rocker joint
# while the panel is offset `wing_norm_offset` normal to the rocker, so the two
# segments never actually meet. The 5.00 mm every geometry reported for this
# pair is not an artefact to suppress, it IS `wing_norm_offset`: the design
# clearance at the pin end. Suppressing it hid a real failure mode. Sweeping
# `wing_angle_from_rocker` on `_compact` with everything else fixed, the pair
# holds 5.00 mm up to ~61 deg and then the panel swings THROUGH the coupler --
# 0.04 mm at 70 deg, 0.03 at 80, closest at the MIDDLE of the panel rather
# than at the pin. Nothing else in the study catches that.
#   crankR-crankL    ONE rigid body -- two arms keyed to the same shaft
_ADJACENT = frozenset({
    ("crankR", "couplerR"), ("couplerR", "rockerR"), ("rockerR", "panelR"),
    ("crankL", "couplerL"), ("couplerL", "rockerL"), ("rockerL", "panelL"),
    ("crankR", "crankL"),
})


def _pair_ok(a: str, b: str) -> bool:
    return (a, b) in _ADJACENT or (b, a) in _ADJACENT


def _same_plane(lk, a: str, b: str) -> bool:
    """Can these two members reach each other at all? Names carry a side suffix."""
    pa, pb = lk.planes.get(a[:-1]), lk.planes.get(b[:-1])
    return pa == "all" or pb == "all" or pa == pb


MIN_LINK_GAP_MM = 4.0
"""Clearance every non-adjacent pair of members must keep [mm].

Default for `clearance.coupler_width_mm`, and the floor `min_link_gap` is read
against in the full check. `pin_support_radius` in the Part Studio is 0.1 in =
2.54 mm, so a boss is ~5 mm across and two of them passing need ~5 mm centre to
centre; 4.0 is that, less a little. Set it from the real part.
"""


def coupler_gap(lk: SwingLinkage):
    """Closest the two COUPLERS come to each other: (mm, travel).

    THE cross-side pair that decides whether the mechanism can be built, and
    the one that separates the geometries here. The crank-vs-opposite-coupler
    crossing reads 0.00 mm on every geometry in this repo and is structural to
    a co-rotating pair -- the crank arms sweep across the centreline and the
    opposite coupler runs through that sweep -- so no search designs it out and
    the fore/aft stagger is what answers it. The couplers are different: they
    approach each other at the END of the stroke, by an amount the link lengths
    control, and `clearance.coupler_width_mm` is the margin they need.

    Measured against a 0.25 deg walk on all five tracked geometries, the
    minimum is at the last sample of the stroke every time -- hand-drawn
    14.32 mm, `_opt` 2.84, `_compact` 3.40, `_vertical` 5.70, `_margin` 5.00.
    So the deployed toggle is where it is checked, with rest and mid-stroke
    carried as cheap insurance rather than because they have ever bound.
    """
    A = critical_angles(lk)
    if A.command <= 0.0:
        return 0.0, 0.0
    best, at = 1e9, 0.0
    for t in sorted({0.0, A.fold, A.command * 0.5, A.command, A.deploy}):
        if t > A.deploy + 1e-9:
            continue
        lk.reset()
        a = link_segments(lk, -1, t)
        b = link_segments(lk, 1, t)
        if a is None or b is None:
            continue
        d = _seg_seg_dist(a[1][1], a[1][2], b[1][1], b[1][2])
        if d < best:
            best, at = d, t
    return best, at


def coupler_gap_commanded(lk: SwingLinkage):
    """Closest the couplers come within the COMMANDED stroke: (mm, travel).

    SEPARATE FROM `coupler_gap`, and the difference is a real design question
    rather than bookkeeping. The couplers keep closing past the commanded
    stroke, so the two numbers differ exactly when the panel reaches horizontal
    before the deployed toggle -- `_compact` is 1.15 mm at its 138.5 deg
    mechanical limit and 3.40 mm at the 106.6 deg it is actually driven to.

    Which one binds is a question about the SERVO, not the linkage: a position
    -mode servo commanded to `command` never visits the rest, but nothing stops
    a hand, an overshoot, or a back-drive from taking the mechanism to its own
    limit -- and there the parts are 1.15 mm apart. `coupler_gap` (the limit)
    is what is scored, because a part that can be driven into itself is a part
    that will be; this is reported beside it so the cost of that choice is
    visible rather than assumed.
    """
    A = critical_angles(lk)
    if A.command <= 0.0:
        return 0.0, 0.0
    best, at = 1e9, 0.0
    for t in sorted({0.0, A.fold, A.command * 0.5, A.command}):
        lk.reset()
        a = link_segments(lk, -1, t)
        b = link_segments(lk, 1, t)
        if a is None or b is None:
            continue
        d = _seg_seg_dist(a[1][1], a[1][2], b[1][1], b[1][2])
        if d < best:
            best, at = d, t
    return best, at


def min_link_gap(lk: SwingLinkage, travel=None, step: float = 2.0,
                 cross_side=None, to_limit: bool = True):
    """Worst clearance between any two non-adjacent members: (mm, pair, deg).

    THE FULL AUDIT. Which pairs are eligible comes from `clearance.planes`:
    members in different fore/aft planes cannot touch however close they look
    here, and a member marked `all` (the panel, 90 mm deep) is checked against
    everything.

    THE OBJECTIVE DOES NOT SCORE THIS -- `coupler_gap` does. Across every
    geometry in this repo the runner-up to the coupler pair sits at 29-47 mm,
    so the general check has never been the binding one while costing a pose
    per candidate travel per pair. It is kept because it is the honest full
    answer and because `--check` runs it: if a future geometry gets tight
    somewhere unexpected, this is what will say where.

    `cross_side` is accepted and IGNORED. It was a boolean that described the
    machine wrongly -- "the two four-bars are staggered from each other", which
    dropped coupler-vs-opposite-coupler, the one cross-side pair that actually
    collides. `clearance.planes` replaced it; see `_DEFAULT_PLANES`.
    """
    if travel is None and to_limit:
        # OVER [0, deploy], not [0, command]. Same reasoning `coupler_gap`
        # gives: a part that can be driven into itself is a part that will be,
        # and the two ranges differ exactly when the panel reaches horizontal
        # before the toggle.
        A = critical_angles(lk)
        travel = sorted(set(evaluation_travels(lk, -1, A.deploy))
                        | set(evaluation_travels(lk, 1, A.deploy)))
    rows = _segment_poses(lk, travel)
    if not rows:
        return 0.0, "does not close", 0.0
    names = [n for n, _a, _b in rows[0][1]]
    pairs = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            na, nb = names[i], names[j]
            if _pair_ok(na, nb) or not _same_plane(lk, na, nb):
                continue
            pairs.append((i, j))
    worst, who, at = 1e9, "", 0.0
    for t, segs in rows:
        for i, j in pairs:
            na, a0, a1 = segs[i]
            nb, b0, b1 = segs[j]
            d = _seg_seg_dist(a0, a1, b0, b1)
            if d < worst:
                worst, who, at = d, f"{na}-{nb}", t
    return worst, who, at


def panel_keepout_gap(lk: SwingLinkage, travel=None, step: float = 2.0,
                      samples: int = 25):
    """Worst panel-to-keep-out clearance: (mm, box, deg).

    Positive is clear, NEGATIVE is the penetration depth. Returns
    (inf, "", 0.0) when no keep-out is configured, so a config without one is
    unconstrained rather than trivially failing.

    THIS is what bounds an inward swing, not `far_inboard_deg`. The angle says
    how far the rising panel has tipped past vertical; it says nothing about
    whether the volume it tipped into is occupied. Under
    `wing_angle_mode: vertical_rest` the panel crosses vertical at the first
    degree of stroke, so an angular limit alone either forbids the mode
    outright or permits it all the way into the battery.

    EVALUATED AT THE CANDIDATE TRAVELS. The panel is rigid with the rocker, so
    its position is a function of one angle and both of its ends turn around
    only where `panel_stationary_travels` says. In practice the worst pose is
    the rising side's FOLDED toggle on every geometry here: against a 0.25 deg
    walk the minimum landed within 0.05 deg of that toggle on all five.
    `--check` re-runs the walk and prints the disagreement, so this is
    auditable rather than asserted.

    The boxes are inflated by `clearance.wing_width_mm` when they are loaded --
    this is a front view and the panel is a LINE in it, so a real panel's width
    has to enter as extra keep-out rather than as panel geometry.
    """
    if not lk.keepout:
        return float("inf"), "", 0.0
    A = critical_angles(lk)
    if A.command <= 0.0:
        return -1e3, "does not close", 0.0
    us = [i / (samples - 1.0) for i in range(samples)]
    worst, who, at = 1e9, "", 0.0
    for t, segs in _segment_poses(lk, travel):
        for name, foot, top in segs:
            if not name.startswith("panel"):
                continue
            fy, fz = float(foot[0]), lk.z_floor(float(foot[1]))
            dy = float(top[0]) - fy
            dz = lk.z_floor(float(top[1])) - fz
            for u in us:
                y, z = fy + u * dy, fz + u * dz
                for box in lk.keepout:
                    d = _point_box_sdf(y, z, box)
                    if d < worst:
                        worst, who, at = d, box.name, t
    return worst, who, at


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


def min_transmission(lk: SwingLinkage, travel: float | None = None,
                     step: float = 3.0, frac: float = TRANS_END_FRAC) -> float:
    """Worst transmission angle over the driven stroke [deg].

    Angle between coupler and rocker at the joint. Same meaning and the same
    trap as the mirrored study: a reach-only objective walks straight into a
    dead point, because the dead point costs nothing on reach while being the
    least buildable part of the design.

    CLOSED-FORM CANDIDATES, not a sweep -- see `transmission_travels`. cos(mu)
    is an affine function of L^2, and L^2 is a plain sinusoid in the crank
    angle, so the unfolded mu is monotone in L and turns only where L does; the
    fold to min(mu, 180-mu) adds the mu = 90 crossing. Five candidate travels
    per side against a 0.05 deg walk on all five tracked geometries: worst
    disagreement 0.013 deg, and that is the walk's quantisation at the edge of
    the window.

    BOTH SIDES at the same positive travel, which is not the mirror the rest of
    this file exploits: at +t one linkage is deploying and the other rising,
    and they are genuinely different configurations. What the mirror saves is
    the NEGATIVE travels, not the second side.
    """
    A = critical_angles(lk)
    if A.command <= 0.0:
        return 0.0
    hi = A.command * frac
    worst = 180.0
    for side in (-1, 1):
        for t in transmission_travels(lk, side, hi):
            lk.reset()
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


def ratio_at(lk: SwingLinkage, side: int, travel: float, pose=None):
    """d(rocker) / d(crank) at one pose, closed form. None if it does not close.

        w4 / w2 = a sin(th2 - th3) / (c sin(th4 - th3))

    the standard four-bar velocity relation, with th2/th3/th4 the crank,
    coupler and rocker bearings. EXACT, which is the point: `ratio_curve` got
    the same number by np.gradient over a uniform grid, so anything wanting a
    ratio had to walk one. Agreement with that grid is 0.001-0.005% of peak on
    all five tracked geometries.

    It goes to zero exactly at the crank+coupler collinear poses, which is not
    a numerical accident -- it is what a toggle IS, and it is why the deployed
    pose holds with no servo current.
    """
    pz = lk.pose(side, travel) if pose is None else pose
    if pz is None:
        return None
    C, J, P = pz["crank_tip"], pz["joint"], pz["pivot"]
    v2, v3, v4 = C - lk.shaft, J - C, J - P
    th2 = np.arctan2(v2[1], v2[0])
    th3 = np.arctan2(v3[1], v3[0])
    th4 = np.arctan2(v4[1], v4[0])
    den = lk.rocker * np.sin(th4 - th3)
    if abs(den) < 1e-12:
        return None
    return float(lk.crank * np.sin(th2 - th3) / den)


def torque_at(lk: SwingLinkage, travel: float, mass: float = BIKE_MASS_KG,
              com_z: float = COM_Z_MM) -> float:
    """Servo torque at ONE crank travel [N.m], no grid.

    Same load case as `torque_curve`: the bike lying on the deploying wing,
    the wing flat on the ground, the stroke rotating the BIKE up about the
    pivot. tau_servo = m g |lever| * |d(wing)/d(crank)|.
    """
    pz = lk.pose(-1, travel)
    if pz is None:
        return 1e3
    # `ratio_at(lk, -1, travel)` would solve the same pose a second time, and
    # the golden section calls this ~35 times -- it was 55% of every objective
    # evaluation's poses on its own. Pass the pose we already have.
    g = ratio_at(lk, -1, travel, pz)
    if g is None:
        return 0.0
    phi = 90.0 - abs(pz["wing_deg"])
    piv = lk.pivot(-1)
    com0 = np.array([0.0, com_z - lk.wheel_radius])
    com = piv + _rot(com0 - piv, -(90.0 - phi))
    lever = abs(float(com[0] - piv[0])) / 1000.0
    return float(mass * GRAVITY * lever * abs(g))


def peak_torque(lk: SwingLinkage, travel: float | None = None) -> float:
    """Largest servo torque anywhere in the commanded stroke [N.m].

    THE ONE THING THE THREE ANGLES CANNOT GIVE YOU, and not by a small margin.

    tau_servo = tau_wing * |d(wing)/d(crank)| is a PRODUCT of two factors that
    peak at opposite ends. The weight moment is largest at travel 0, where the
    bike lies on its side and the CoM hangs furthest from the pivot, and falls
    to nothing as the bike comes upright. The velocity ratio is EXACTLY ZERO at
    the deployed toggle -- that is the definition of the toggle and the
    self-locking property the design wants -- and small at rest. So the maximum
    is strictly interior, and measured on the five tracked geometries it sits
    at 54-81% of the stroke and is 4 to 6 times the value at travel 0:

        geometry     peak      at      travel 0   at the end
        hand-drawn   0.510   64.5%       0.125       0.254
        _opt         0.405   60.9%       0.080       0.020
        _compact     0.548   80.7%       0.088       0.491
        _vertical    0.550   69.0%       0.081       0.006
        _margin      0.420   54.3%       0.128       0.006

    Evaluating torque at any of rest, fold or deploy would therefore report
    roughly a fifth of the real load, and at the deployed toggle would report
    zero. There is no closed form for the maximum -- setting the derivative to
    zero is transcendental in the four-bar angles -- so this is the one place a
    search is unavoidable.

    GOLDEN SECTION, not a walk, and that is safe rather than hopeful:
    |tau|(travel) has exactly ONE turning point on every geometry here, checked
    at 0.25 deg. ~40 poses to 1e-4 deg against ~110-140 for the old 1 deg walk,
    and it no longer needs a uniform grid because `ratio_at` is closed form.

    `travel` is accepted and IGNORED, so old call sites keep working; the
    stroke is `critical_angles(lk).command` and always was meant to be.
    """
    hi = critical_angles(lk).command
    if hi <= 0.0:
        return 1e3
    inv = (np.sqrt(5.0) - 1.0) / 2.0
    a, b = 0.0, hi
    c, d = b - inv * (b - a), a + inv * (b - a)
    fc, fd = torque_at(lk, c), torque_at(lk, d)
    for _ in range(40):
        if fc > fd:
            b, d, fd = d, c, fc
            c = b - inv * (b - a)
            fc = torque_at(lk, c)
        else:
            a, c, fc = c, d, fd
            d = a + inv * (b - a)
            fd = torque_at(lk, d)
        if b - a < 1e-4:
            break
    return float(max(fc, fd, torque_at(lk, 0.0), torque_at(lk, hi)))


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

    # The interference primitives, against cases worked by hand. They are two
    # dozen lines of geometry that nothing else in this file exercises, and a
    # segment distance that is quietly wrong reports a design as buildable --
    # which is a worse failure than reporting it as broken, because nobody goes
    # and looks. The first version of `_seg_seg_dist` WAS wrong: it clamped one
    # parameter against the other and then recomputed the other from the
    # clamped one, which is a Gauss-Seidel sweep rather than a projection.
    print("\nsegment and keep-out primitives:")
    cases = [
        ("crossing X",        _seg_seg_dist((0, 0), (10, 10), (0, 10), (10, 0)), 0.0),
        ("parallel, 3 apart", _seg_seg_dist((0, 0), (10, 0), (0, 3), (10, 3)), 3.0),
        ("collinear, gap 5",  _seg_seg_dist((0, 0), (10, 0), (15, 0), (20, 0)), 5.0),
        ("skew endpoints",    _seg_seg_dist((0, 0), (1, 0), (5, 4), (9, 9)),
                              float(np.hypot(4, 4))),
        ("shared endpoint",   _seg_seg_dist((0, 0), (5, 5), (5, 5), (10, 0)), 0.0),
    ]
    box = KeepOut("t", 10.0, 0.0, 100.0)
    cases += [
        ("box: 4 outboard",   _point_box_sdf(14.0, 50.0, box), 4.0),
        ("box: mirrored",     _point_box_sdf(-14.0, 50.0, box), 4.0),
        ("box: above",        _point_box_sdf(0.0, 103.0, box), 3.0),
        ("box: corner",       _point_box_sdf(13.0, 104.0, box), 5.0),
        ("box: 2 inside",     _point_box_sdf(8.0, 50.0, box), -2.0),
    ]
    for name, got, want in cases:
        bad = abs(got - want) > 1e-9
        ok &= not bad
        print(f"   {name:18s} {got:9.5f}  want {want:9.5f}"
              + ("   <-- WRONG" if bad else ""))

    # `vertical_rest` has to be IDEMPOTENT, because the derived angle is written
    # back into the config and a saved config gets re-loaded by this same class.
    # A derivation that moved on the second load would drift a design every time
    # it was reopened.
    import copy as _copy
    vcfg = _copy.deepcopy(cfg)
    vcfg["mechanism"]["wing_angle_mode"] = "vertical_rest"
    v1 = SwingLinkage(vcfg)
    v2 = SwingLinkage(_copy.deepcopy(v1.cfg))
    d1, d2 = abs(rest_wing_deg(v1)), abs(v1.wing_from_rocker - v2.wing_from_rocker)
    ok &= d1 < 1e-9 and d2 < 1e-9
    # THE CANDIDATE SETS, against a dense walk. Every geometric metric here now
    # evaluates at a handful of closed-form travels instead of sweeping, which
    # is the whole speed argument -- and an argument that is only worth having
    # if it is checkable. This walks 0.25 deg and prints the disagreement.
    lkc = SwingLinkage(cfg)
    A = critical_angles(lkc)
    if A.command > 0.0:
        ts = np.arange(0.0, A.deploy + 1e-9, 0.25)
        w_far, w_foot, w_ext, w_keep, w_coup = 1e9, 1e9, 0.0, 1e9, 1e9
        w_trans = 180.0
        for t in ts:
            lkc.reset()
            a, b = lkc.pose(-1, float(t)), lkc.pose(1, float(t))
            if a is None or b is None:
                break
            v = b["top"] - b["foot"]
            if t <= A.command + 1e-9:
                w_far = min(w_far, float(np.degrees(np.arctan2(v[0], v[1]))))
                w_foot = min(w_foot, abs(float(a["foot"][0]))
                             - abs(float(lkc.pivot(-1)[0])))
                w_ext = max(w_ext, max(abs(float(b["top"][0])),
                                       abs(float(b["foot"][0]))))
                for pz in (a, b):
                    for q in (pz["foot"], pz["top"]):
                        for u in np.linspace(0, 1, 25):
                            r = pz["foot"] + u * (pz["top"] - pz["foot"])
                            for bx in lkc.keepout:
                                w_keep = min(w_keep, _point_box_sdf(
                                    float(r[0]), lkc.z_floor(float(r[1])), bx))
                        break
                if t <= A.command * TRANS_END_FRAC + 1e-9:
                    for pz in (a, b):
                        v1 = pz["crank_tip"] - pz["joint"]
                        v2 = pz["pivot"] - pz["joint"]
                        mu = np.degrees(np.arccos(np.clip(
                            float(v1 @ v2) / (np.linalg.norm(v1)
                                              * np.linalg.norm(v2)), -1, 1)))
                        w_trans = min(w_trans, mu, 180.0 - mu)
            sa, sb = link_segments(lkc, -1, float(t)), link_segments(lkc, 1, float(t))
            w_coup = min(w_coup, _seg_seg_dist(sa[1][1], sa[1][2],
                                               sb[1][1], sb[1][2]))
        print("\ncandidate travels vs a 0.25 deg walk"
              f"   (fold {A.fold:.2f}, deploy {A.deploy:.2f}, command {A.command:.2f}):")
        # Tolerance 0.25, which is the WALK's step. Every residual seen so far
        # is the walk stopping short of the exact endpoint, and in every case
        # the candidate value is the conservative one -- a larger maximum, a
        # smaller gap. The closed form is the more accurate number here; the
        # walk is the sanity check, not the reference.
        rows = [("far wing inboard", far_inboard_deg(lkc), w_far, 0.25, "deg"),
                ("foot vs own pivot", foot_outboard_mm(lkc), w_foot, 0.25, "mm"),
                ("panel |y| max", panel_extents(lkc)[2], w_ext, 0.25, "mm"),
                ("panel keep-out", panel_keepout_gap(lkc)[0], w_keep, 0.25, "mm"),
                ("coupler-coupler", coupler_gap(lkc)[0], w_coup, 0.25, "mm"),
                ("link interference", min_link_gap(lkc)[0],
                 min_link_gap(lkc, travel=list(np.arange(0.0, A.deploy + 1e-9, 0.25)))[0],
                 0.25, "mm"),
                ("transmission", min_transmission(lkc), w_trans, 0.25, "deg")]
        for name, fast, slow, tol, unit in rows:
            if not np.isfinite(slow) or abs(slow) > 1e8:
                continue
            d = abs(fast - slow)
            ok &= d <= tol
            print(f"   {name:18s} {fast:9.3f} vs {slow:9.3f} {unit:3s}  d={d:.4f}"
                  + ("" if d <= tol else f"   <-- OVER {tol}"))

    # The panel span, same idempotence argument as the bearing below: the
    # resolved `wing_z_max` is written back into the config and a saved config
    # gets re-loaded by this same class, so a derivation that moved on the
    # second load would drift a design every time it was reopened.
    import copy as _copy2
    for mode, key, want in (("length", "panel_length_mm", 100.0),
                            ("top_height", "panel_top_z_mm", 150.0)):
        scfg = _copy2.deepcopy(cfg)
        scfg["mechanism"]["panel_span_mode"] = mode
        scfg["mechanism"][key] = want
        s1 = SwingLinkage(scfg)
        s2 = SwingLinkage(_copy2.deepcopy(s1.cfg))
        if mode == "length":
            got = s1.wing_z_max - s1.wing_z_min
        else:
            got = s1.z_floor(float(s1.pose(-1, 0.0)["top"][1]))
        drift = abs(s1.wing_z_max - s2.wing_z_max)
        bad = abs(got - want) > 1e-6 or drift > 1e-9
        ok &= not bad
        print(f"\npanel_span_mode: {mode}"
              f"\n   asked {want:.3f}, got {got:.3f}, re-load drift {drift:.2e}"
              + ("" if not bad else "   <-- WRONG"))

    print(f"\nvertical_rest derivation:"
          f"\n   rest panel off vertical  {d1:.9f} deg"
          + ("" if d1 < 1e-9 else "   <-- NOT VERTICAL")
          + f"\n   re-load drift            {d2:.9f} deg"
          + ("" if d2 < 1e-9 else "   <-- NOT IDEMPOTENT"))
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
    travel = critical_angles(lk).command
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
    print(f"hand-off roll there   {use_hand:6.1f} deg"
          f"   (window +-{_budgets(lk).handoff:.0f})")
    print(f"far wing inboard      {far_inboard_deg(lk, use):6.1f} deg from vertical"
          f"   (limit {_budgets(lk).far_min:+.0f}, negative = into the body)")
    print(f"  (lateral, fyi)      {far_clearance(lk, use):6.1f} mm from the centreline")
    print(f"foot vs own pivot     {foot_outboard_mm(lk, use):6.1f} mm"
          + ("" if foot_outboard_mm(lk, use) >= 0 else "   <-- SWINGS INBOARD"))
    print(f"rest grounds at       {rest_ground_angle(lk):6.1f} deg of roll"
          f"   (recoverable set {RECOVERABLE_DEG})")
    _r, _e, _m, _a = panel_extents(lk, use)
    print(f"rest half-width       {stow_half_width(lk):6.1f} mm")
    print(f"rest off vertical     {rest_wing_deg(lk):6.1f} deg   (+ outboard;"
          f" mode {lk.wing_angle_mode})")
    print(f"panel |y| rest / end  {_r:6.1f} / {_e:.1f} mm      <- scored, equal weight")
    print(f"           max        {_m:6.1f} mm at crank {_a:.0f}   (check only)")
    _g, _ga = coupler_gap(lk)
    print(f"coupler-coupler       {_g:6.1f} mm   at crank {_ga:+.0f}"
          f"   (needs {lk.coupler_width})")
    _f, _fp, _fa = min_link_gap(lk)
    print(f"  all pairs (fyi)     {_f:6.1f} mm   worst {_fp} at crank {_fa:+.0f}")
    _k, _kb, _ka = panel_keepout_gap(lk)
    if np.isfinite(_k):
        print(f"panel keep-out        {_k:6.1f} mm   worst {_kb} at crank {_ka:+.0f}"
              + ("" if _k >= 0 else "   <-- INSIDE THE CHASSIS"))
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
    axes[3].set_ylabel("non-deployed wing |y| [mm]")
    axes[3].set_xlabel("crank travel [deg]  (positive deploys the right wing)")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].set_title("swing linkage: reach and ratio through the stroke")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"\nwrote {out}")


def _draw_keepout(ax, lk: SwingLinkage) -> None:
    """The panel keep-out boxes, mirrored, in the body frame.

    Drawn because a number in a table does not tell you WHICH way to move the
    design, and the figure does: a panel clipping the top corner of the box
    wants a shorter panel, one driving through its side wants a different
    linkage.
    """
    from matplotlib.patches import Rectangle
    # CLAMPED TO THE AXES. An unbounded face is stored as +-1e9, and drawing a
    # patch that tall made matplotlib take minutes per frame -- the video
    # render looked hung and was not. The sentinel is a modelling convenience;
    # nothing should ever hand it to a renderer.
    zlo_ax, zhi_ax = -50.0, 260.0
    for box in lk.keepout:
        lo = max(box.z_lo, zlo_ax)
        hi = min(box.z_hi, zhi_ax)
        if hi <= lo:
            continue
        for sgn in (-1, 1):
            x0 = 0.0 if sgn > 0 else -box.half_width
            ax.add_patch(Rectangle((x0, lo), box.half_width, hi - lo,
                                   facecolor="0.85", edgecolor="0.6",
                                   ls="--", lw=1.0, zorder=1))
    if lk.keepout:
        b0 = lk.keepout[0]
        b0_top = min(b0.z_hi, zhi_ax)
        # "PANEL ONLY" on the figure, because the crank and couplers are drawn
        # straight through this box and look like violations. They are not:
        # the shaft is ON the centreline and the box is the volume the PANEL
        # may not sweep into. See the KeepOut docstring.
        ax.annotate("chassis keep-out (panel only)"
                    + ("" if b0.z_hi < 1e8 else " — open above"),
                    (0.0, b0_top), fontsize=8,
                    ha="center", va="top" if b0.z_hi >= 1e8 else "bottom",
                    color="0.45")


def _draw(ax, lk: SwingLinkage, t: float, *, alpha: float = 1.0,
          labels: bool = False) -> None:
    """One pose, both sides, in the body frame."""
    ax.axhline(0.0, color="0.85", lw=1, zorder=0)
    _draw_keepout(ax, lk)
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
    ax.set_title(f"swing linkage at REST — half-width {stow_half_width(lk):.0f} mm, "
                 f"{rest_wing_deg(lk):+.0f}° off vertical\ngrounds at {ang:.0f}°, "
                 f"mode {lk.wing_angle_mode}", fontsize=11)
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
    # `critical_angles`, not `cfg["stroke"]["crank_travel_deg"]`: that key is an
    # OUTPUT of `--optimize --save`, so a constraints-only config does not carry
    # it and the videos refused to render exactly the files a search had not
    # been run on yet. The commanded travel is derived from the link lengths.
    travel = critical_angles(lk).command
    # FOUR EQUAL QUARTERS, and the floor on `n` is what stops the last one
    # going negative: `q = max(n // 4, 2)` can exceed n/4 for a short clip, so
    # `n - 3q` went NEGATIVE and numpy raised
    # "Number of samples, -1, must be non-negative". Crashed at
    # `--seconds 0.2 --fps 25` (n = 5) and at every n < 8.
    n = max(int(fps * seconds), 8)
    q = max(n // 4, 2)
    sched = np.concatenate([np.linspace(0, -travel, q),
                            np.linspace(-travel, 0, q),
                            np.linspace(0, travel, q),
                            np.linspace(travel, 0, max(n - 3 * q, 1))])
    out.parent.mkdir(parents=True, exist_ok=True)
    bar = _Bar(len(sched), out.name)
    with imageio.get_writer(out, fps=fps, macro_block_size=1) as w:
        for t in sched:
            bar.update(1)
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
    bar.close()
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
    # `critical_angles`, not `cfg["stroke"]["crank_travel_deg"]`: that key is an
    # OUTPUT of `--optimize --save`, so a constraints-only config does not carry
    # it and the videos refused to render exactly the files a search had not
    # been run on yet. The commanded travel is derived from the link lengths.
    travel = critical_angles(lk).command
    n = max(int(fps * seconds), 4)
    # POSITIVE travel, because positive is what deploys side -1 -- the side
    # this shot grounds on. Scheduling negative here laid the RED (right) panel
    # flat while the BLUE (left) one was the one actually descending, so the
    # bike appeared to right itself on the wrong wing. Same sign that had to be
    # fixed in `reach`, `ratio_curve` and `min_transmission`; this call site was
    # missed because it is the only one that names a side and a sign separately.
    sched = np.concatenate([np.linspace(0, travel, n // 2),
                            np.linspace(travel, 0, n - n // 2)])
    out.parent.mkdir(parents=True, exist_ok=True)
    bar = _Bar(len(sched), out.name)
    with imageio.get_writer(out, fps=fps, macro_block_size=1) as w:
        for t in sched:
            bar.update(1)
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
    bar.close()
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

MAY BE SET NEGATIVE, and under `wing_angle_mode: vertical_rest` it has to be.
A vertical rest pose means the rising panel starts AT vertical and crosses it
on the first degree of stroke, so +5 is unsatisfiable by construction there and
the constant changes job: it stops being a margin outboard of vertical and
becomes an ALLOWANCE inboard of it -- how far the wing may lean in over the
chassis. `VERTICAL_FAR_INBOARD_DEG` is the default in that mode.

It is a bound on the LEAN, never a safety check. What decides whether the
volume the panel leans into is occupied is `panel_keepout_gap`, and that is a
separate constraint answering a separate question. Do not read a satisfied
angle as clearance.
"""

VERTICAL_FAR_INBOARD_DEG = -25.0
"""`MIN_FAR_INBOARD_DEG` default under `wing_angle_mode: vertical_rest` [deg].

Negative: an allowance inboard, not a margin outboard. -25 is "slight" for a
panel whose top is 80 mm up -- about 34 mm of inboard travel at the top corner
-- and is deliberately looser than the keep-out, so the geometry is what binds
and this only stops the search running the panel flat over the roof.
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
"""Largest crank travel the assembly search will consider [deg]."""

_MAX_HALF: float | None = None
"""Cap on `stow_half_width`, the whole mechanism's rest envelope [mm].

None leaves it uncapped, which is the honest default: there is no width the
bike must fit inside, only a protrusion worth minimising, and that is what the
objective already does.

IT WAS A NO-OP. `--max-half` has existed since this file's first version and
both `feasibility` and `_objective` unpacked it into a `half_cap` they then
never read, so every run that passed it optimised exactly as if it had not.
Now scored -- as a hard constraint, which is what a cap is; the protrusion
objective is the soft half and they are not the same statement.
"""

_MIN_LINK_GAP: float | None = None      # None = MIN_LINK_GAP_MM
_SKIP_TORQUE = False                    # --skip-torque: drop it from the search
_TORQUE_SET = False                     # --torque-budget was given, so it wins
_TRANS_SET = False                      #   over the config's `limits:` block
_REST_VERTICAL_DEG: float | None = None
"""Tolerance on `rest_wing_deg`, or None to leave it unscored [deg].

Redundant under `wing_angle_mode: vertical_rest`, where the angle is DERIVED to
zero and this can only confirm it. It earns its place under `fixed`, where
asking for a near-vertical rest pose without deriving it is a legitimate thing
to want -- a panel bearing that has to be a round number on a printed part, for
instance.
"""


class Budgets(NamedTuple):
    """Everything the objective is scored against, passed rather than read.

    A NAMED TUPLE, NOT A BARE ONE, and that is the fix for a real hazard rather
    than tidiness. This has to cross a process boundary -- `workers=-1` spawns
    workers, macOS spawn re-imports the module, and a worker that read the
    module constants would score the DEFAULTS while the parent scored the
    flags. It was already a 4-tuple for that reason; adding three more fields
    to a positional tuple unpacked in two places is how the wrong budget ends
    up in the wrong slot with no error anywhere.
    """

    torque: float
    far_min: float
    trans_min: float
    half_cap: float | None
    link_gap: float          # coupler width: the couplers' margin at the end
    rest_vert: float | None
    score_torque: bool = True
    handoff: float = HANDOFF_WINDOW_DEG
    ground_clear: float = RECOVERABLE_DEG + 5.0


_LIMIT_KEYS = {
    "torque_nm": "torque",
    "far_inboard_deg": "far_min",
    "transmission_deg": "trans_min",
    "max_half_width_mm": "half_cap",
    "coupler_width_mm": "link_gap",
    "rest_off_vertical_deg": "rest_vert",
    "handoff_window_deg": "handoff",
    "ground_clear_deg": "ground_clear",
}
"""`limits:` keys in the config, and the `Budgets` field each one sets.

THE POINT OF THE BLOCK: before it, half the constraints lived in the yaml
(`clearance`) and half in module constants reachable only through flags, so
there was no single document saying what a design had to satisfy. A file
carrying `bike`, `limits` and `clearance` now IS that document.

PRECEDENCE is flag > `limits:` > module default, and the module defaults stay
authoritative for a config that says nothing -- the tracked figures here were
produced against them, and a study whose committed numbers cannot be reproduced
from its own defaults is the failure analysis/ conventions exist to prevent.
"""


def _budgets(lk: SwingLinkage | None = None, far: float | None = None) -> Budgets:
    """Module defaults, overlaid with the config's `limits:`, overlaid with flags.

    PRECEDENCE IS flag > `limits:` > default, and `far_min` is the one that has
    to say so explicitly. Its default depends on the REST MODE -- an outboard
    margin under `fixed`, an inboard allowance under `vertical_rest`, because a
    vertical panel crosses vertical on the first degree of stroke and +5 is
    unsatisfiable there.

    THAT MODE SUBSTITUTION USED TO EAT THE CONFIG. The default was applied
    first, into the same local the flag used, and the flag block then ran
    `if far is not None: over["far_min"] = far` -- which was ALWAYS true,
    because the default had already filled it. So `limits.far_inboard_deg` was
    silently overwritten by 5.0 on every run and the key did nothing at all:
    measured, settings from -20 to +40 gave a bit-identical geometry. The
    default now applies only when neither a flag nor the file supplied one.
    """
    b = Budgets(TORQUE_BUDGET_NM, MIN_FAR_INBOARD_DEG, MIN_TRANSMISSION_DEG,
                _MAX_HALF, MIN_LINK_GAP_MM, _REST_VERTICAL_DEG, not _SKIP_TORQUE)
    if lk is None:
        return b if far is None else b._replace(far_min=far)
    # 1. the config's `limits:` block
    limits = lk.cfg.get("limits") or {}
    unknown = set(limits) - set(_LIMIT_KEYS)
    if unknown:
        raise ValueError(f"limits: unknown key(s) {sorted(unknown)}; "
                         f"expected {sorted(_LIMIT_KEYS)}")
    over = {_LIMIT_KEYS[k]: v for k, v in limits.items() if v is not None
            or k in ("max_half_width_mm", "rest_off_vertical_deg")}
    # `coupler_width_mm` has lived in `clearance:` since it was added; that
    # spelling keeps working rather than moving and breaking every config.
    over.setdefault("link_gap", lk.coupler_width)
    # 2. the mode-dependent far default, ONLY if nothing else supplied one
    if far is None and "far_inboard_deg" not in limits:
        over["far_min"] = (VERTICAL_FAR_INBOARD_DEG
                           if lk.wing_angle_mode == "vertical_rest"
                           else MIN_FAR_INBOARD_DEG)
    # 3. flags last, so they beat the file
    if far is not None:
        over["far_min"] = far
    if _MIN_LINK_GAP is not None:
        over["link_gap"] = _MIN_LINK_GAP
    if _REST_VERTICAL_DEG is not None:
        over["rest_vert"] = _REST_VERTICAL_DEG
    if _MAX_HALF is not None:
        over["half_cap"] = _MAX_HALF
    if _TORQUE_SET:
        over["torque"] = TORQUE_BUDGET_NM
    if _TRANS_SET:
        over["trans_min"] = MIN_TRANSMISSION_DEG
    return b._replace(**over)


# `_walk` LIVED HERE and is gone. It walked the whole assembly range at 2 deg
# to find the useful stroke, the brace pose and the reach, and the profile put
# it and `assembly_limit` at the top of the objective's cost. Everything it
# answered is now closed form -- see `critical_angles`.


def brace(lk: SwingLinkage, step: float = 2.0):
    """Best bracing pose: (travel, lowest panel point, lateral arm) [deg, mm, mm].

    Measured on the whole panel rather than the foot tip, because a panel that
    lies down onto the floor braces on its edge and the tip is not necessarily
    its lowest part.

    SEARCHED OVER THE DRIVEN STROKE, not the whole assembly range. The old
    version walked to the assembly limit -- up to 200 deg -- and so priced
    poses past the deployed toggle that nobody would ever command; the panel
    swings back up past the toggle, so the "best" it found could be on the far
    side of a limit the servo will not be driven through.

    Candidate travels, same closed-form set as the other geometric metrics.
    """
    if critical_angles(lk).command <= 0.0:
        return 0.0, 1e9, 0.0
    best = (0.0, 1e9, 0.0)
    for t in evaluation_travels(lk, -1):
        lk.reset()
        pz = lk.pose(-1, t)
        if pz is None:
            continue
        ends = (pz["foot"], pz["top"])
        zs = [lk.z_floor(float(q[1])) for q in ends]
        k = int(np.argmin(zs))
        if zs[k] < best[1]:
            best = (t, zs[k], abs(float(ends[k][0])))
    return best


def useful_stroke(lk: SwingLinkage) -> tuple:
    """(travel to drive, hand-off roll there, foot height there).

    CLOSED FORM, and it replaced a walk over the whole assembly range that was
    the single most expensive thing in the objective.

    The stroke ends at the OUTPUT DEAD POINT -- where wing rotation stops
    increasing -- not at the assembly limit. This is a crank-rocker, so the
    crank keeps turning long past the point where the output has peaked: past
    it the wing swings BACK, the foot rises again, and the far wing carries on
    toward the centreline. Scoring over the full assembly range prices travel
    nobody would ever command, and it did exactly that.

    That dead point is `critical_angles(lk).deploy`. But the stroke you would
    COMMAND is `.command`, which is the earlier of the dead point and the pose
    that lays the panel flat: past horizontal the bike rolls back off the wing
    it is standing on. Both closed form, and together they reproduce the old
    walk on all five tracked geometries -- 112.04 vs 112, 128.97 vs 128,
    106.60 vs 106, 142.02 vs 142, 123.48 vs 124, where the walk's error is its
    own 1-2 deg step.
    """
    A = critical_angles(lk)
    if A.command <= 0.0:
        return 0.0, 180.0, 1e9
    lk.reset()
    pz = lk.pose(-1, A.command)
    if pz is None:
        return 0.0, 180.0, 1e9
    v = pz["top"] - pz["foot"]
    a = abs(float(np.degrees(np.arctan2(v[1], v[0]))))
    return A.command, min(a, 180.0 - a), lk.z_floor(float(pz["foot"][1]))


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
    b = Budgets(*budgets) if budgets is not None else _budgets(lk)
    use, hand, _lo = useful_stroke(lk)
    span = use
    bt, blow, barm = brace(lk)
    e_rest, e_end, e_max, e_at = panel_extents(lk, span)
    gap, gap_pair, gap_at = min_link_gap(lk)
    cgap, cgap_at = coupler_gap(lk)
    keep, keep_box, keep_at = panel_keepout_gap(lk)
    rows = [
        ("hand-off roll",      hand,                        "<=", b.handoff,          "deg"),
        # The panel only has to end up FLAT relative to the floor -- the bike
        # falls onto the whole wing, so the contact is the panel, not a point,
        # and whatever height it sits at is fine. That is the same quantity as
        # `hand-off roll` above, so there is no separate brace criterion: the
        # height and arm below are reported, not scored.
        ("far wing outboard",  far_inboard_deg(lk, span),   ">=", b.far_min,          "deg"),
        ("foot vs own pivot",  foot_outboard_mm(lk, span),  ">=", 0.0,                "mm"),
        ("transmission angle", min_transmission(lk, span),  ">=", b.trans_min,        "deg"),
        ("peak servo torque",  peak_torque(lk, span),       "<=", b.torque,           "N.m"),
        ("rest ground clear",  rest_ground_angle(lk),       ">=", b.ground_clear,     "deg"),
        # THE TWO THAT KNOW TWO PARTS EXIST AT ONCE. Everything above is a
        # property of one link or one panel. EVERY pair that can touch, over
        # the mechanism's own range -- which members can touch is
        # `clearance.planes`, and the tightest is named in the fyi rows below.
        # Scored at the mechanism's OWN limit, not at the commanded stroke: a
        # part that can be driven into itself is a part that will be.
        ("link interference",  gap,                         ">=", b.link_gap,         "mm"),
    ]
    if lk.keepout:
        rows.append(("panel keep-out",  keep,               ">=", 0.0,                "mm"))
    if b.half_cap is not None:
        rows.append(("rest half-width", stow_half_width(lk), "<=", b.half_cap,        "mm"))
    if b.rest_vert is not None:
        rows.append(("rest off vertical", abs(rest_wing_deg(lk)), "<=", b.rest_vert,  "deg"))
    out = []
    for name, val, op, lim, unit in rows:
        ok = (val <= lim + 1e-9) if op == "<=" else (val >= lim - 1e-9)
        out.append((name, val, op, lim, unit, ok))
    # Reported, never scored as pass/fail: there is no envelope to fit inside,
    # only a protrusion to minimise.
    out.append(("panel |y| rest", e_rest, "min", float("nan"), "mm", None))
    out.append(("panel |y| end", e_end, "min", float("nan"), "mm", None))
    out.append(("panel |y| max", e_max, f"at {e_at:.0f}", float("nan"), "mm", None))
    out.append(("panel height there", blow, "fyi", float("nan"), "mm", None))
    out.append(("contact |y|", barm, "fyi", float("nan"), "mm", None))
    if b.rest_vert is None:
        out.append(("rest off vertical", rest_wing_deg(lk), "+ is outboard",
                    float("nan"), "deg", None))
    # WHICH pair and WHERE, because the number alone is not actionable: a 2 mm
    # gap between couplerL and panelR at +90 deg is a different design problem
    # from the same 2 mm between crankR and rockerR at rest.
    out.append(("worst pair", gap, f"{gap_pair or '-'} at crank {gap_at:+.0f}",
                float("nan"), "mm", None))
    ccom, _cc_at = coupler_gap_commanded(lk)
    out.append(("couplers", cgap, f"at the {cgap_at:.0f} deg limit; {ccom:.2f} at "
                f"the commanded {critical_angles(lk).command:.0f}",
                float("nan"), "mm", None))
    if lk.keepout and np.isfinite(keep):
        out.append(("worst keep-out", keep, f"{keep_box} at crank {keep_at:+.0f}",
                    float("nan"), "mm", None))
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


def assembly_residual(lk: SwingLinkage) -> float:
    """How far this geometry is from having reachable rocker limits [mm].

    Zero when both toggles exist. Otherwise the millimetres by which the
    circle-circle conditions fail, which is the closest thing to a distance
    from "unbuildable" to "buildable" the link lengths admit.

    THE POINT IS TO GIVE THE SEARCH A SLOPE. A toggle exists iff the crank tip
    can reach the rocker's circle, i.e. |g - c| <= reach <= g + c, with reach
    a + b extended and |b - a| folded, g the shaft-to-pivot distance and c the
    rocker. Those inequalities are cheap and their VIOLATION is a length.
    """
    a, b, c = lk.crank, lk.coupler, lk.rocker
    g = float(np.linalg.norm(lk.shaft - lk.pivot(-1)))
    out = 0.0
    for reach in (a + b, abs(b - a)):
        out += max(0.0, reach - (g + c)) + max(0.0, abs(g - c) - reach)
    return out


INFEASIBLE = 1e4
"""Where the infeasible bands start.

IT USED TO BE 100, AND THAT INVERTED THE RANKING. The two-stage design assumes
every infeasible value outranks every feasible one -- but the feasible value is
a PROTRUSION IN MILLIMETRES and nothing stopped it exceeding 100. Measured on
the shipped constraints file: seeds 1 and 6 converge on designs 173.8 mm and
158.6 mm wide that sit EXACTLY on a constraint boundary, missing by 0.0000, and
so score `100 + 1e-9`. Differential evolution then prefers that 100.000 to any
honestly feasible design wider than 100 mm, and cannot climb out toward the
66.3 mm optimum. Those two seeds were not unlucky; they were being told a wide
boundary-sitting design was the best thing they had found.

1e4 is above any protrusion this mechanism can physically produce -- the widest
geometry the bounds admit is a few hundred millimetres -- so the bands are
strictly ordered again whatever the search turns up.
"""

PLATEAU = 4e4
"""Objective value for a geometry that does not assemble at all.

A FLAT VALUE HERE STOPS THE SEARCH DEAD, and that is not a hypothetical:
differential evolution's convergence test is on the SPREAD of the population's
energies, so once every member returns the same number `std` is 0 and scipy
reports "Optimization terminated successfully" having found nothing. Measured
with the search bounds pinned to a 1 mm box, 100% of the population bails and
DE stops at nit=1; with a merely tight constraint set it stops after a few
dozen generations, which is what "it drops out at 33 iters" looks like.

So nothing below returns a bare constant any more. The ladder, worst first:

    40000 + residual   does not assemble; `assembly_residual` in mm
    30000 + (20 - use) assembles but the stroke is under 20 deg
    20000 + 2*short    a usable stroke that cannot right the bike
    10000 + violation  rights the bike, breaks a constraint
    the protrusion     feasible, in mm

Each rung is strictly above the one below, so the search never trades a
category for a category -- only positions within one, and the direction out.
"""


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
    try:
        lk = SwingLinkage(_apply(cfg, x))
    except Exception:
        # No geometry at all -- nothing to measure a distance from, so this is
        # the one rung with no slope. It is also the rarest: the constructor
        # only raises on a bad mode or a degenerate solve.
        return PLATEAU + 1e4
    b = Budgets(*budgets) if budgets is not None else _budgets(lk)
    resid = assembly_residual(lk)
    if resid > 0.0:
        return PLATEAU + resid

    # ONE stroke, ending where the panel is most nearly horizontal. Bracing
    # means the panel finishes FLAT -- the bike lands on the whole wing, so the
    # contact is the panel, not a point, and the height it sits at is fine.
    # That is the same pose `useful_stroke` already finds, so the brace pose
    # and the hand-off pose are one pose, not two.
    use, hand, _lo = useful_stroke(lk)
    if not np.isfinite(hand):
        return PLATEAU + resid
    if use < 20.0:
        # Assembles, but barely moves. Ordered by HOW MUCH stroke it got, so
        # the search can climb out toward a usable one.
        return 3e4 + (20.0 - use)
    span = use

    # RIGHTING: how far from upright the bike is left when the stroke ends.
    short_rot = max(0.0, hand - b.handoff)
    # Cheap bail before the sweeps below, keeping a gradient so DE can descend
    # toward feasibility rather than stranding on a plateau.
    if short_rot > 25.0:
        return 2e4 + 2.0 * short_rot

    far = max(0.0, b.far_min - far_inboard_deg(lk, span))
    inboard = max(0.0, -foot_outboard_mm(lk, span))
    bind = max(0.0, b.trans_min - min_transmission(lk, span))
    # SKIPPABLE, and it is the only constraint that is. Torque is 31 of the 49
    # four-bar solves an evaluation costs -- a bounded search, because the peak
    # is interior and no single pose stands in for it (measured across 120
    # geometries: the best single-pose proxy averages 90% of the true peak and
    # bottoms out at 3%). Dropping it makes the search ~3x faster and its
    # answer unsound; `cmd_optimize` re-scores the winner and says so loudly.
    over = 0.0 if not b.score_torque else max(0.0, peak_torque(lk) - b.torque)
    ground = max(0.0, b.ground_clear - rest_ground_angle(lk))
    # Two couplers overlapping is not a soft trade against a millimetre of
    # protrusion, so it is weighted above everything except torque: a design
    # that cannot be assembled is not a design that is slightly too wide.
    gap = max(0.0, b.link_gap - min_link_gap(lk)[0])
    keep = max(0.0, -panel_keepout_gap(lk)[0])
    half = (0.0 if b.half_cap is None
            else max(0.0, stow_half_width(lk) - b.half_cap))
    vert = (0.0 if b.rest_vert is None
            else max(0.0, abs(rest_wing_deg(lk)) - b.rest_vert))

    viol = (2.0 * short_rot + 0.60 * far + 1.00 * inboard
            + 0.30 * bind + 60.0 * over + 1.20 * ground
            + 3.00 * gap + 3.00 * keep + 1.00 * half + 1.00 * vert)
    if viol > 1e-9:
        return INFEASIBLE + viol

    e_rest, e_end, _e_max, _e_at = panel_extents(lk, span)
    return max(e_rest, e_end)


def _search(cfg, seeds, iters):
    """Best objective over a few seeds: (value, x). Quiet -- no bar, no report."""
    from scipy.optimize import differential_evolution
    lk0 = SwingLinkage(_copy_cfg(cfg))
    buds = _budgets(lk0)
    cb = cfg.get("bounds") or {}
    bd = [(float(cb[n][0]), float(cb[n][1])) if n in cb else (lo, hi)
          for n, _p, lo, hi in _VARS]
    best = (1e9, None)
    for sd in seeds:
        r = differential_evolution(_objective, bd, args=(cfg, buds), seed=sd,
                                   maxiter=iters, tol=1e-8, polish=True,
                                   init="sobol", mutation=(0.4, 1.0),
                                   recombination=0.85, workers=-1,
                                   updating="deferred")
        if float(r.fun) < best[0]:
            best = (float(r.fun), r.x)
    return best


def _copy_cfg(cfg):
    import copy as _c
    return _c.deepcopy(cfg)


# What `--why` is allowed to relax, and by how much. Each entry is
# (label, how to apply a relaxation to a config copy).
#
# THE STEPS ARE DELIBERATELY COARSE. This answers "which line in my file is
# costing me width", not "what is the derivative" -- a shadow price computed by
# re-running a stochastic global search is noisy at any step size, and a big
# step makes the signal survive that noise. Read the ORDER, not the millimetre.
_RELAX = [
    ("limits.torque_nm            +0.10 N.m",
     lambda c: c.setdefault("limits", {}).__setitem__(
         "torque_nm", (c.get("limits", {}).get("torque_nm") or TORQUE_BUDGET_NM) + 0.10)),
    ("limits.transmission_deg     -10 deg",
     lambda c: c.setdefault("limits", {}).__setitem__(
         "transmission_deg", (c.get("limits", {}).get("transmission_deg")
                              or MIN_TRANSMISSION_DEG) - 10.0)),
    ("limits.far_inboard_deg      -5 deg",
     lambda c: c.setdefault("limits", {}).__setitem__(
         "far_inboard_deg", (c.get("limits", {}).get("far_inboard_deg")
                             or MIN_FAR_INBOARD_DEG) - 5.0)),
    ("limits.handoff_window_deg   +3 deg",
     lambda c: c.setdefault("limits", {}).__setitem__(
         "handoff_window_deg", (c.get("limits", {}).get("handoff_window_deg")
                                or HANDOFF_WINDOW_DEG) + 3.0)),
    ("limits.ground_clear_deg     -10 deg",
     lambda c: c.setdefault("limits", {}).__setitem__(
         "ground_clear_deg", (c.get("limits", {}).get("ground_clear_deg")
                              or RECOVERABLE_DEG + 5.0) - 10.0)),
    ("clearance.coupler_width_mm  -2 mm",
     lambda c: c.setdefault("clearance", {}).__setitem__(
         "coupler_width_mm", max(0.5, (c.get("clearance", {})
                                       .get("coupler_width_mm") or MIN_LINK_GAP_MM) - 2.0)),),
    ("keep-out half_width         -5 mm",
     lambda c: [b.__setitem__("half_width", max(1.0, b["half_width"] - 5.0))
                for b in (c.get("clearance", {}).get("panel_keepout") or [])]),
    ("keep-out z_lo               +15 mm",
     lambda c: [b.__setitem__("z_lo", (b.get("z_lo") or 0.0) + 15.0)
                for b in (c.get("clearance", {}).get("panel_keepout") or [])]),
    ("mechanism.wing_z_max        -15 mm",
     lambda c: c["mechanism"].__setitem__(
         "wing_z_max", c["mechanism"]["wing_z_max"] - 15.0)),
]


def _bounds_relaxations():
    """One entry per search variable: drop ITS bound back to the study default.

    PER-VARIABLE AND ALL THE WAY BACK, not a fixed step. A `bounds:` floor set
    to 55 mm cannot be diagnosed by relaxing it 5 mm -- the design is still
    pinned and the row reports "buys nothing", which reads as "not the
    problem" when it is exactly the problem. Asking "what if I had not
    restricted this one at all" is both the honest question and the one whose
    answer is actionable.
    """
    out = []
    for name, _p, lo, hi in _VARS:
        def apply(c, _n=name, _lo=lo, _hi=hi):
            b = dict(c.get("bounds") or {})
            b.pop(_n, None)
            c["bounds"] = b
        out.append((f"bounds.{name:20s} -> study default", apply))
    return out


_RELAX += _bounds_relaxations()


def cmd_why(cfg, seeds: int = 3, iters: int = 250) -> None:
    """Which constraint is costing the width? Relax each, re-search, rank.

    FOR THE CASE THE FEASIBILITY TABLE CANNOT ANSWER: a run that converges,
    satisfies everything, and returns a mechanism far wider than it should be.
    The table says which rows are ON a limit, but not which of them is the one
    actually holding the design open -- several are always tight, and relaxing
    most of them buys nothing.

    So this measures it: relax one line at a time by a coarse step, re-run the
    search, and report the millimetres it bought. READ THE ORDER, NOT THE
    NUMBER. Each row is a stochastic global search, so a couple of millimetres
    is noise; a row that buys thirty is the answer.
    """
    base_seeds = tuple(range(1, seeds + 1))
    w0, x0 = _search(_copy_cfg(cfg), base_seeds, iters)
    lk0 = SwingLinkage(_apply(cfg, x0)) if x0 is not None else SwingLinkage(_copy_cfg(cfg))
    print(f"baseline: {w0:.1f} mm over {seeds} seeds x {iters} iters"
          + ("   -- INFEASIBLE, so the ranking below is about reaching "
             "feasibility at all" if w0 >= INFEASIBLE else ""))
    if x0 is not None and w0 < INFEASIBLE:
        rows = [r for r in feasibility(lk0, cfg) if r[5] is not None]
        tight = [r[0] for r in rows
                 if r[5] and abs((r[3] - r[1]) if r[2] == "<=" else (r[1] - r[3]))
                 < 0.02 * max(abs(r[3]), 1.0)]
        print(f"  on a limit at the optimum: {', '.join(tight) if tight else 'none'}")
    print()
    out = []
    bar = _Bar(len(_RELAX), "relaxing")
    for label, apply in _RELAX:
        c = _copy_cfg(cfg)
        try:
            apply(c)
            w, _x = _search(c, base_seeds, iters)
        except Exception as e:
            w = float("nan")
        out.append((label, w))
        bar.update(1, label.split(".")[-1][:24])
    bar.close()
    print(f"{'relaxing this...':38s} {'gives':>9s}   {'buys':>8s}")
    for label, w in sorted(out, key=lambda kv: kv[1]):
        if not np.isfinite(w):
            print(f"  {label:36s} {'error':>9s}")
            continue
        buys = w0 - w
        flag = "   <-- THE BINDING ONE" if buys > 10.0 else ""
        print(f"  {label:36s} {w:9.1f} {buys:+8.1f}{flag}")
    print("\n  Coarse steps and a stochastic search, so a few mm is noise."
          "\n  A row buying tens of mm is the constraint holding the design open.")


def cmd_optimize(cfg, save: Path | None, seed: int, iters: int, restarts: int = 1,
                 tol: float = 1e-8,
                 torque: float | None = None, min_far: float | None = None,
                 min_trans: float | None = None, max_half: float | None = None,
                 link_gap: float | None = None, rest_vert: float | None = None,
                 skip_torque: bool = False) -> None:
    """Search the shared link lengths.

    The budgets are OVERRIDABLE rather than edited in place: the module
    constants are what the tracked configs were produced against, and a study
    whose committed figures cannot be reproduced from its own defaults is the
    failure analysis/ conventions exist to prevent.
    """
    from scipy.optimize import differential_evolution

    global TORQUE_BUDGET_NM, MIN_TRANSMISSION_DEG, _MAX_HALF
    global _MIN_LINK_GAP, _REST_VERTICAL_DEG, _SKIP_TORQUE
    _SKIP_TORQUE = bool(skip_torque)
    global _TORQUE_SET, _TRANS_SET
    _TORQUE_SET = torque is not None
    _TRANS_SET = min_trans is not None
    if torque is not None:
        TORQUE_BUDGET_NM = torque
    if min_trans is not None:
        MIN_TRANSMISSION_DEG = min_trans
    _MAX_HALF = max_half
    _MIN_LINK_GAP = link_gap
    _REST_VERTICAL_DEG = rest_vert

    lk0 = SwingLinkage(cfg)
    buds = _budgets(lk0, min_far)
    print(f"budgets: torque {buds.torque} N.m   far {buds.far_min} deg"
          f"   transmission {buds.trans_min} deg   link gap {buds.link_gap} mm"
          + (f"   half-width {buds.half_cap} mm" if buds.half_cap else "")
          + (f"   rest vertical +-{buds.rest_vert} deg" if buds.rest_vert else ""))
    if _SKIP_TORQUE:
        print("  --skip-torque: peak torque is NOT scored. ~3x faster, and the\n"
              "  result is not a feasible design until the line below says so.")
    print(f"rest pose: {lk0.wing_angle_mode}"
          + (f"   (wing_angle_from_rocker derived to "
             f"{lk0.wing_from_rocker:.2f} deg at the seed)"
             if lk0.wing_angle_mode == "vertical_rest" else "")
          + f"   {len(lk0.keepout)} keep-out box(es)"
          + f"   planes {lk0.planes}")

    use0, hand0, lo0 = useful_stroke(lk0)
    print(f"start:  hand-off {hand0:.1f} deg (window {buds.handoff:.0f})   "
          f"brace {brace(lk0)[1]:.1f} mm above floor   "
          f"far {far_inboard_deg(lk0, use0):.1f} deg from vertical   "
          f"rest grounds {rest_ground_angle(lk0):.1f} deg")
    # BOUNDS ARE CONSTRAINTS and belong in the file with the others. They were
    # module-only, so a constraints config could not say "my crank cannot be
    # shorter than 20 mm" -- the floors were invisible from outside this file
    # even though `_compact` sits ON one of them.
    cfg_bounds = cfg.get("bounds") or {}
    unknown = set(cfg_bounds) - {n for n, _p, _l, _h in _VARS}
    if unknown:
        raise ValueError(f"bounds: unknown key(s) {sorted(unknown)}; expected "
                         f"{sorted(n for n, _p, _l, _h in _VARS)}")
    bounds = []
    for name, _p, lo, hi in _VARS:
        got = cfg_bounds.get(name)
        bounds.append((float(got[0]), float(got[1])) if got else (lo, hi))
    for (name, _p, _l, _h), (lo, hi) in zip(_VARS, bounds):
        if name in cfg_bounds:
            print(f"  bound from config: {name:22s} [{lo}, {hi}]")
    # workers=-1: the objective is a pure function of (x, cfg, budgets), all
    # picklable, so this parallelises across cores for free. `updating` must be
    # "deferred" once workers != 1 -- scipy will not do immediate updating
    # across processes.
    # RESTARTS: independent seeds, keep the best. Differential evolution on
    # this objective is not reliably repeatable -- measured on the shipped
    # constraints file, 10 of 12 seeds reach the optimum and two strand on a
    # constraint boundary, and which two moves whenever the objective changes
    # at all. That is not a bug to tune out; it is what a stochastic global
    # search on a mostly-infeasible space does. Running several and taking the
    # best is the cheap, honest answer, and the spread is printed so a run that
    # found the same answer every time can be told from one that got lucky.
    bar = _Bar(iters * restarts, f"search x{restarts}")
    tried = []
    res = None
    try:
        # EXACTLY ONE PARAMETER, NAMED `intermediate_result`. scipy INTROSPECTS
        # the callback signature to choose between the OptimizeResult form and
        # the legacy `(x, convergence)` one -- so adding a second parameter,
        # even a bound default like `_k=k` to carry the seed, silently switches
        # it to the legacy form and hands you an ndarray. That is an
        # AttributeError on `.fun` in the middle of a long run. The seed comes
        # from a mutable cell instead.
        cur = {"sd": seed}

        def _tick(intermediate_result):
            f = float(intermediate_result.fun)
            bar.update(1, f"seed {cur['sd']} best {f:9.3f}"
                       + ("  (infeasible)" if f >= INFEASIBLE else " mm"))

        for k in range(restarts):
            sd = seed + k
            cur["sd"] = sd
            r = differential_evolution(_objective, bounds, args=(cfg, buds),
                                       seed=sd, maxiter=iters, tol=tol,
                                       polish=True, init="sobol",
                                       mutation=(0.4, 1.0), recombination=0.85,
                                       callback=_tick, workers=-1,
                                       updating="deferred")
            tried.append((sd, int(r.nit), float(r.fun)))
            if res is None or float(r.fun) < float(res.fun):
                res = r
    finally:
        bar.close()
    if restarts > 1:
        good = [f for _s, _n, f in tried if f < INFEASIBLE]
        print(f"\n{restarts} restarts, seeds {seed}..{seed + restarts - 1}:")
        for sd, nit, f in tried:
            mark = "  <- kept" if f == float(res.fun) else ""
            print(f"   seed {sd:3d}  nit {nit:4d}  {f:9.3f}"
                  + ("  INFEASIBLE" if f >= INFEASIBLE else "") + mark)
        if good:
            print(f"   {len(good)}/{restarts} feasible; best {min(good):.3f}, "
                  f"worst {max(good):.3f}, spread {max(good) - min(good):.3f} mm")
        else:
            print(f"   0/{restarts} feasible -- this is a constraint-set "
                  f"problem, not a seed problem. Try --why.")
    best = _apply(cfg, res.x)
    lk = SwingLinkage(best)
    use, hand, lo = useful_stroke(lk)
    at = use
    # The DRIVEN range, not the righting stroke -- the same span feasibility()
    # uses, so the summary and the table cannot disagree about the same
    # quantity. They did: 30.1 vs 9.1 deg of far-wing clearance in one report.
    lim = use
    # WHY IT STOPPED, always. scipy reports "Optimization terminated
    # successfully" for the CONVERGENCE test -- the spread of the population's
    # energies falling under `tol` -- which says nothing about whether it found
    # anything. A run that bails identically on every candidate has zero spread
    # and gets that same cheerful message at nit=1. Print the generation count
    # against what was asked for, and say plainly when it stopped short.
    print(f"\nstopped after {res.nit}/{iters} generations: {res.message}")
    if res.nit < iters:
        print("   Early stop is the CONVERGENCE test, not an error: the "
              "population's\n   objective values stopped spreading. That is "
              "the good outcome when the\n   value is low, and a stranded "
              "search when it is not -- see the rung below.")
    f = float(res.fun)
    rung = ("feasible" if f < INFEASIBLE else
            "on a constraint (10000 + violation)" if f < 2e4 else
            "cannot right the bike (20000 + roll short)" if f < 3e4 else
            "stroke under 20 deg (30000 + shortfall)" if f < 4e4 else
            "DOES NOT ASSEMBLE (40000 + mm from closing)")
    print(f"best (objective {f:.3f}) -- {rung}")
    if f >= 3e4:
        print("   The search never found a working mechanism. That is a "
              "CONSTRAINT SET\n   problem, not a seed problem: widen `bounds:` "
              "or relax `limits:`, and\n   check `--sweep` on a geometry you "
              "believe in first.")
    for (name, _p, _l, _h), v in zip(_VARS, res.x):
        lo, hi = bounds[_VARS.index((name, _p, _l, _h))]
        edge = ("  <- AT THE LOWER BOUND" if abs(v - lo) < 1e-3 else
                "  <- AT THE UPPER BOUND" if abs(v - hi) < 1e-3 else "")
        print(f"   {name:24s} {v:8.2f}{edge}")
    print(f"\n   useful stroke      {use:6.1f} deg"
          f"   (assembly limit {assembly_limit(lk, cap=_ASSEMBLY_CAP):.0f})")
    print(f"   hand-off roll      {hand:6.1f} deg"
          f"   (window +-{buds.handoff:.0f})")
    bt, blow, barm = brace(lk)
    print(f"   brace             {blow:6.1f} mm above the floor at crank {bt:.0f},"
          f" {barm:.0f} mm outboard"
          + ("" if blow <= 2.0 else "   <-- CANNOT BRACE"))
    print(f"   far wing inboard   {far_inboard_deg(lk, lim):6.1f} deg from vertical"
          f"   (limit {buds.far_min:+.0f})")
    print(f"   foot vs own pivot  {foot_outboard_mm(lk, lim):6.1f} mm"
          + ("" if foot_outboard_mm(lk, lim) >= 0 else "   <-- SWINGS INBOARD"))
    print(f"   rest grounds at    {rest_ground_angle(lk):6.1f} deg"
          f"   (recoverable {RECOVERABLE_DEG})")
    print(f"   worst transmission {min_transmission(lk, lim):6.1f} deg")
    _r, _e, _m, _a = panel_extents(lk, use)
    print(f"   angle between cranks {best['mechanism']['angle_between_cranks']:6.1f} deg"
          f"   <- sets the rest setpoint")
    print(f"   rest half-width    {stow_half_width(lk):6.1f} mm")
    print(f"   rest off vertical  {rest_wing_deg(lk):6.1f} deg   (+ is outboard)")
    print(f"   panel |y| rest/end {_r:6.1f} / {_e:.1f} mm   max {_m:.1f} at {_a:.0f}")
    _g, _ga = coupler_gap(lk)
    print(f"   coupler-coupler    {_g:6.1f} mm   at crank {_ga:+.0f}"
          f"   (needs {buds.link_gap})")
    _k, _kb, _ka = panel_keepout_gap(lk)
    if np.isfinite(_k):
        print(f"   panel keep-out     {_k:6.1f} mm   worst {_kb} at crank {_ka:+.0f}"
              + ("" if _k >= 0 else "   <-- INSIDE THE CHASSIS"))
    print(f"   peak servo torque  {peak_torque(lk, lim):6.3f} N.m"
          f"   (budget {TORQUE_BUDGET_NM}, XC330 has 0.66 at cutoff)")
    if _SKIP_TORQUE:
        got = peak_torque(lk)
        print(f"\n  torque was NOT searched: peak {got:.3f} N.m against a "
              f"{buds.torque} budget"
              + ("   -- OVER, re-run without --skip-torque" if got > buds.torque
                 else "   -- under it, so this one happens to be usable"))
    # Scored WITH torque whatever the search did, so the table never reports a
    # constraint as met that was simply not looked at.
    ok = print_feasibility(lk, best, buds._replace(score_torque=True))
    if not ok:
        print("\n  NOT FEASIBLE -- the objective may still read low, because a"
              "\n  soft penalty is zero AT a boundary and small just past it.")
    if save is not None:
        # setdefault, because a CONSTRAINTS-ONLY input has no `stroke:` block
        # to write into -- the commanded travel is an output of this run, not
        # something the file had to declare.
        best.setdefault("stroke", {})["crank_travel_deg"] = float(round(abs(at), 1))
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
    ap.add_argument("--why", action="store_true",
                    help="a converged but disappointing result: relax each "
                         "constraint in turn, re-search, and rank them by the "
                         "millimetres each one buys. `--seed` is the seed COUNT "
                         "here and `--iters` the per-search budget.")
    ap.add_argument("--no-progress", action="store_true",
                    help="never draw a progress bar. The bar is already off "
                         "when stderr is not a TTY; this turns it off in a "
                         "terminal too, which is the first thing to try if a "
                         "run misbehaves.")
    ap.add_argument("--videos", action="store_true",
                    help="write BOTH videos for this config -- the body-frame "
                         "sweep and the ground-frame righting. `--out` is a "
                         "DIRECTORY here (default traces/) and `--tag` names "
                         "the pair. This is the one to run on an --optimize "
                         "result.")
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
                    help="mm the STOWED mechanism may reach laterally (this "
                         "was accepted and silently ignored until 2026-09-03)")
    ap.add_argument("--min-link-gap", type=float, default=None,
                    help="mm every non-adjacent pair of members must keep "
                         f"apart (default {MIN_LINK_GAP_MM})")
    ap.add_argument("--skip-torque", action="store_true",
                    help="do not score peak servo torque during the search. "
                         "~3x faster (it is 31 of the 49 four-bar solves an "
                         "evaluation costs) and the answer is unsound -- the "
                         "winner is re-scored with it and the run says so")
    ap.add_argument("--rest-vertical", type=float, default=None,
                    help="deg the REST panel may sit off vertical. Redundant "
                         "under mechanism.wing_angle_mode: vertical_rest, "
                         "which derives it to zero")
    ap.add_argument("--restarts", type=int, default=1,
                    help="run this many independent searches from consecutive "
                         "seeds and keep the best. The per-seed results and "
                         "their spread are printed, so a repeatable answer can "
                         "be told from a lucky one. Local minima on this "
                         "objective are real: 10/12 seeds reach the optimum on "
                         "the shipped constraints file.")
    ap.add_argument("--tol", type=float, default=1e-8,
                    help="scipy's convergence tolerance on the SPREAD of the "
                         "population's objective values. 0 disables it, so "
                         "every search runs the full --iters instead of "
                         "stopping when the population stops spreading.")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--tag", default="",
                    help="suffix for default output names, so one config's "
                         "figures stay apart from another's")
    a = ap.parse_args()
    _Bar.off = a.no_progress
    cfg = yaml.safe_load(a.config.read_text())

    def default_out(stem: str, ext: str) -> Path:
        return a.out or _plots_dir() / f"{stem}{a.tag}.{ext}"

    if a.check:
        cmd_check(cfg)
    elif a.why:
        cmd_why(cfg, a.seed if a.seed > 1 else 3, a.iters)
    elif a.optimize:
        cmd_optimize(cfg, a.save, a.seed, a.iters, a.restarts, a.tol,
                     a.torque_budget, a.min_far, a.min_transmission, a.max_half,
                     a.min_link_gap, a.rest_vertical, a.skip_torque)
    elif a.videos:
        # BOTH, which is what you want after a search: the mechanism in the
        # body frame and the bike righting itself in the ground frame answer
        # different questions, and neither alone says whether a geometry is
        # worth building. `--out` names a DIRECTORY here, not a file.
        d = a.out or Path("traces")
        d.mkdir(parents=True, exist_ok=True)
        cmd_video(cfg, d / f"swing_linkage{a.tag}.mp4", a.fps, a.seconds)
        cmd_righting_video(cfg, d / f"swing_righting{a.tag}.mp4",
                           a.fps, a.seconds)
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
