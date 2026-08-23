"""The wing LINKAGE study: a four-bar per wing, both on one servo.

An alternative to the gear train in `righting.wings` — see
docs/plans/wing-linkage-design-and-optimization.md. Gears give a rigidly
mirrored pair and a constant ratio; a linkage gives a ratio that VARIES through
the stroke, which is the whole point: the torque the servo must supply is
worst at one part of the stroke, and a linkage can be shaped to spend its
mechanical advantage there. The price is that the two sides stop being mirror
images.

    python analysis/wing_linkage.py            # labelled stowed geometry
    python analysis/wing_linkage.py --deploy   # stowed -> deployed sweep

Everything is the 2D front/back view: (y, z) in millimetres, from the FLOOR and
the CENTRELINE, +y to the bike's left. Reads config/wing_linkage.yaml and
writes analysis/plots/. Changes nothing else — this study is deliberately
outside bike_params.yaml and the params digest.

THE FOUR BAR, per wing, in the order the brief names them:

    link 1  servo crank      servo centre -> crank tip. Points AWAY from its
                             own wing (right wing's crank goes up+left).
    link 2  coupler          crank tip -> attach point on the wing. Reaches
                             BACK toward the wing.
    link 3  rocker           attach point -> wing pivot. Rigid with the wing,
                             so the wing IS this link.
    link 4  ground           wing pivot -> servo centre. Virtual/fixed.

Drawn from STOWED, and the coupler lengths are derived from that pose rather
than specified, so the asymmetry falls out instead of being imposed.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config" / "wing_linkage.yaml"


def _plots_dir():
    """analysis/plots/, created on demand."""
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(parents=True, exist_ok=True)
    return d


# --------------------------------------------------------------------------
# geometry


def _rot(v, deg):
    """Rotate a 2-vector CCW in the (y, z) plane."""
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


class Linkage:
    """One bike's worth of geometry, solved from the stowed pose.

    `side` is +1 for the bike's LEFT wing (+y) and -1 for the RIGHT (-y),
    matching the sign convention everywhere else in the repo.
    """

    def __init__(self, cfg: dict):
        b, m = cfg["bike"], cfg["mechanism"]
        self.cfg = cfg
        self.servo = np.array([0.0, b["wheel_radius"] + m["servo_offset"]])
        self.pivot_z = b["wheel_radius"]
        self.half_span = b["bike_width"] / 2.0
        self.pivot_y = m["wing_pivot_offset"]
        # The wing spans floor-clearance to the top of the bike when stowed,
        # so its length is not a free parameter.
        # `bike_height` is the TOP OF THE BIKE -- the roof crest -- exactly as
        # in bike_params.yaml. The stowed wing therefore tops out a roof RADIUS
        # below it, at the roof axis, which is where the geared wing tip sits
        # too and is what makes the tips tangent to the rolling surface.
        #
        # This used to read `wing_top = bike_height`, i.e. the panel went all
        # the way to the number called "bike height" and the roof was then
        # stacked on top. That made the bike a radius taller than its own
        # bike_height and looked like the linkage "forcing a taller roof",
        # when it was only the name meaning two different things in two files.
        self.wing_bottom = b["ground_clearance"]
        self.wing_top = b["bike_height"] - self.half_span
        self.wing_length = self.wing_top - self.wing_bottom
        # ...and its outer face sits this far outboard of its own pivot.
        self.stow_offset = self.half_span - self.pivot_y

        self.crank = {}
        self.attach0 = {}
        self.coupler = {}
        self.crank_angle0 = {}
        for side, tag in ((-1, "right"), (1, "left")):
            self.crank[tag] = m["wing_first_link_length"][tag]
            ang = m["first_link_angle_deg"]
            if tag == "left":
                ang += m["angle_between_first_links"]
            self.crank_angle0[tag] = ang
            off = np.asarray(m["wing_attach_offset"], float)
            # Mirror the attach offset across the centreline for the right wing.
            self.attach0[tag] = self.pivot(side) + np.array([side * off[0], off[1]])
            # DERIVED: whatever closes the loop at stow.
            self.coupler[tag] = float(np.linalg.norm(
                self.attach0[tag] - self.crank_tip(tag, 0.0)))

    # -- fixed points ------------------------------------------------------

    def pivot(self, side: int) -> np.ndarray:
        return np.array([side * self.pivot_y, self.pivot_z])

    def crank_tip(self, tag: str, travel_deg: float) -> np.ndarray:
        a = self.crank_angle0[tag] + travel_deg
        return self.servo + self.crank[tag] * np.array(
            [np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a))])

    # -- the moving pose ---------------------------------------------------

    def solve(self, tag: str, travel_deg: float):
        """Wing angle for a given servo travel, by closing the four-bar.

        The attach point lies at a fixed radius from BOTH the wing pivot (it is
        rigid with the wing) and the crank tip (the coupler). So it is one of
        the two circle-circle intersections, and the branch is chosen as the
        one nearer the stowed solution — a four-bar that flips branch has
        physically come apart.
        """
        side = -1 if tag == "right" else 1
        p, c = self.pivot(side), self.crank_tip(tag, travel_deg)
        r_rocker = float(np.linalg.norm(self.attach0[tag] - p))
        r_coupler = self.coupler[tag]
        d = float(np.linalg.norm(c - p))
        if d > r_rocker + r_coupler or d < abs(r_rocker - r_coupler) or d == 0:
            return None, None            # cannot close: link lengths too short
        a = (r_rocker**2 - r_coupler**2 + d**2) / (2 * d)
        h2 = r_rocker**2 - a**2
        if h2 < 0:
            return None, None
        h = np.sqrt(h2)
        base = p + a * (c - p) / d
        perp = np.array([-(c - p)[1], (c - p)[0]]) / d
        cands = [base + h * perp, base - h * perp]
        ref = self.attach0[tag] if travel_deg == 0 else self._last.get(tag, self.attach0[tag])
        attach = min(cands, key=lambda q: np.linalg.norm(q - ref))
        if not hasattr(self, "_last"):
            self._last = {}
        self._last[tag] = attach
        # Wing angle = how far the rocker has swung from its stowed bearing.
        v0 = self.attach0[tag] - p
        v1 = attach - p
        ang = np.degrees(np.arctan2(v1[1], v1[0]) - np.arctan2(v0[1], v0[0]))
        return attach, float((ang + 180) % 360 - 180)

    def wing_line(self, tag: str, wing_deg: float):
        """The wing's outer face as a segment, swung by `wing_deg` from stow."""
        side = -1 if tag == "right" else 1
        p = self.pivot(side)
        out = np.array([side * self.stow_offset, 0.0])
        lo = p + out + np.array([0.0, self.wing_bottom - self.pivot_z])
        hi = p + out + np.array([0.0, self.wing_top - self.pivot_z])
        return (p + _rot(lo - p, wing_deg), p + _rot(hi - p, wing_deg))


# --------------------------------------------------------------------------
# kinematic evaluation


TARGET_WING_DEG = 90.0
"""Wing rotation needed to right the bike, in degrees.

The load case is NOT a tip poking the floor. The bike lies ON the wing at ~90
deg of roll, and the mechanism pushes the wing OUT from under it: the wing
stays flat on the ground and the bike rotates up around it. So the wing has to
sweep about as far as the bike has to roll, and 90 deg is the whole job.
"""


def sweep(lk: "Linkage", max_travel: float = 360.0, step: float = 1.0):
    """(travel, |wing angle| right, |wing angle| left) from stow.

    Resets the four-bar branch memory first: `solve` picks the assembly branch
    nearest the previous pose, so a sweep that starts mid-stroke can inherit
    the wrong one and report a mechanism that never existed.
    """
    lk._last = {}
    ts, ar, al = [], [], []
    for t in np.arange(0.0, max_travel + step, step):
        got = {}
        for tag in ("right", "left"):
            _, wd = lk.solve(tag, float(t))
            got[tag] = None if wd is None else abs(wd)
        if got["right"] is None or got["left"] is None:
            break                       # linkage came apart; stroke ends here
        ts.append(float(t)); ar.append(got["right"]); al.append(got["left"])
    return np.array(ts), np.array(ar), np.array(al)


def best_pose(lk: "Linkage", target: float = None, max_travel: float = 360.0):
    """The servo angle at which BOTH wings are closest to `target`.

    This is the design question, and it is not the same as "how far can each
    wing get". There is ONE servo, so both wings are at whatever angle that one
    input puts them at; a linkage that sends the left wing to 90 deg while the
    right is at 40 has not righted the bike, however far the left went. So
    score the best SIMULTANEOUS pose inside the monotonic window.

    Returns (travel, right, left, worst_error).
    """
    target = TARGET_WING_DEG if target is None else target
    ts, ar, al = sweep_window(lk, max_travel)
    if not len(ts):
        return 0.0, 0.0, 0.0, 1e3
    err = np.maximum(np.abs(ar - target), np.abs(al - target))
    k = int(np.argmin(err))
    return float(ts[k]), float(ar[k]), float(al[k]), float(err[k])


def sweep_window(lk: "Linkage", max_travel: float = 360.0, tol: float = 0.05):
    """Unwrapped DEPLOYMENT for both sides across the monotonic window.

    Deployment is signed so that POSITIVE always means "this wing's far end is
    swinging outboard, the way it has to go to get under the bike". The right
    wing (at -y) does that by rotating CCW and the left wing CW, so the left
    angle is negated here.

    That sign convention is not cosmetic. Scoring |angle| instead let an
    optimiser hand back a mechanism driving the LEFT wing 90 deg INBOARD --
    sweeping it through the bike and out the far side, dipping 55 mm below the
    floor on the way -- and score it a perfect zero, because 90 deg the wrong
    way has the same magnitude as 90 deg the right way. With the sign folded
    in, a wing going the wrong way simply reads as negative deployment, the
    monotonic window closes immediately, and the design scores as the failure
    it is.
    """
    lk._last = {}
    step = 2.0
    raw = {"right": 0.0, "left": 0.0}
    acc = {"right": 0.0, "left": 0.0}
    prev_r = prev_l = 0.0
    ts, ar, al = [], [], []
    t = 0.0
    while t <= max_travel:
        got = {}
        for tag in ("right", "left"):
            _, wd = lk.solve(tag, t)
            if wd is None:
                got[tag] = None
                continue
            d = (wd - raw[tag] + 180.0) % 360.0 - 180.0
            acc[tag] += d
            raw[tag] = wd
            # +1 for the right wing, -1 for the left: both become "outboard".
            got[tag] = acc[tag] * (1.0 if tag == "right" else -1.0)
        if got["right"] is None or got["left"] is None:
            break
        if t > 0 and (got["right"] < prev_r - tol or got["left"] < prev_l - tol):
            break
        if got["right"] < -tol or got["left"] < -tol:
            break                       # driven INBOARD; not a deployment
        prev_r, prev_l = got["right"], got["left"]
        ts.append(t); ar.append(prev_r); al.append(prev_l)
        t += step
    return np.array(ts), np.array(ar), np.array(al)


def window(lk: "Linkage", max_travel: float = 360.0, tol: float = 0.05):
    """The usable stroke: how far the servo can turn with BOTH wings still
    deploying, and how far each has rotated by then.

    Per the brief, one side inherently reaches full deployment before the
    other; what is not allowed is running on into the region where the leading
    side has begun to RETRACT. So the window closes at the first turning point
    of either side, and the design is scored at that instant.
    """
    # Incremental with an early exit. Sweeping the whole 360 first and then
    # looking for the turning point wasted ~4x the work in the optimiser,
    # because the window typically closes inside 90 deg.
    lk._last = {}
    step = 2.0
    prev_r = prev_l = 0.0
    best = (0.0, 0.0, 0.0)
    t = 0.0
    while t <= max_travel:
        got = {}
        for tag in ("right", "left"):
            _, wd = lk.solve(tag, t)
            got[tag] = None if wd is None else abs(wd)
        if got["right"] is None or got["left"] is None:
            break
        if t > 0 and (got["right"] < prev_r - tol or got["left"] < prev_l - tol):
            break
        prev_r, prev_l = got["right"], got["left"]
        best = (t, prev_r, prev_l)
        t += step
    return best


# --------------------------------------------------------------------------
# drawing

_C = {"right": "#d62728", "left": "#1f77b4"}


def _draw(ax, lk: Linkage, travel: float, *, labels: bool, alpha: float = 1.0,
          wing_lw: float = 5.0):
    for tag in ("right", "left"):
        side = -1 if tag == "right" else 1
        col = _C[tag]
        p = lk.pivot(side)
        c = lk.crank_tip(tag, travel)
        attach, wing_deg = lk.solve(tag, travel)
        if attach is None:
            continue
        lo, hi = lk.wing_line(tag, wing_deg)
        ax.plot([lo[0], hi[0]], [lo[1], hi[1]], color=col, lw=wing_lw,
                alpha=alpha, solid_capstyle="round", zorder=2)
        ax.plot([p[0], attach[0]], [p[1], attach[1]], color=col, lw=2.0,
                alpha=alpha, zorder=3)                       # link 3, rocker
        ax.plot([c[0], attach[0]], [c[1], attach[1]], color="#2ca02c", lw=2.0,
                alpha=alpha, zorder=3)                       # link 2, coupler
        ax.plot([lk.servo[0], c[0]], [lk.servo[1], c[1]], color="#7f2fa0",
                lw=2.6, alpha=alpha, zorder=4)               # link 1, crank
        for q in (p, c, attach):
            ax.plot(*q, "o", ms=5, color="k", alpha=alpha, zorder=5)
        if labels:
            ax.plot([p[0], lk.servo[0]], [p[1], lk.servo[1]], ls=":", lw=1.2,
                    color="0.45", zorder=1)                  # link 4, ground
            ax.annotate(f"{tag} wing", hi, color=col, fontsize=9, ha="center",
                        fontweight="bold",
                        textcoords="offset points", xytext=(0, 8))
            ax.annotate("pivot", p, color=col, fontsize=7, ha="right",
                        textcoords="offset points", xytext=(-7, -3))
            ax.annotate("attach", attach, color=col, fontsize=7,
                        ha="left" if side > 0 else "right",
                        textcoords="offset points",
                        xytext=(8 * side, 7))


def cmd_stowed(cfg, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lk = Linkage(cfg)
    fig, ax = plt.subplots(figsize=(8.4, 7.0))
    _draw(ax, lk, 0.0, labels=True)

    ax.plot(*lk.servo, "s", ms=11, color="#7f2fa0", zorder=6)
    ax.annotate("XC330\n(drives both link 1s)", lk.servo, fontsize=7.5,
                ha="center", va="top", color="#7f2fa0",
                textcoords="offset points", xytext=(0, -9))
    ax.axhline(0, color="k", lw=2)
    ax.axhline(lk.pivot_z, color="0.8", lw=0.8, ls="--")
    ax.annotate(f"rear axle  z={lk.pivot_z:.1f}", (-lk.half_span - 34, lk.pivot_z),
                fontsize=7, color="0.4", va="bottom")

    # Dimensions the brief names, kept OUT of the middle of the mechanism --
    # the four-bar occupies a small patch around the centreline and every
    # label placed there lands on top of another one.
    def dim_h(y0, y1, z, text):
        ax.annotate("", (y0, z), (y1, z),
                    arrowprops=dict(arrowstyle="<->", color="0.35", lw=1.0))
        ax.annotate(text, (0.5 * (y0 + y1), z), fontsize=7, color="0.25",
                    ha="center", textcoords="offset points", xytext=(0, 4))

    def dim_v(y, z0, z1, text, ha="left", dx=4):
        ax.annotate("", (y, z0), (y, z1),
                    arrowprops=dict(arrowstyle="<->", color="0.35", lw=1.0))
        ax.annotate(text, (y, 0.5 * (z0 + z1)), fontsize=7, color="0.25",
                    rotation=90, va="center", ha=ha,
                    textcoords="offset points", xytext=(dx, 0))

    top = lk.wing_top
    dim_h(-lk.half_span, lk.half_span, top + 20, f"bike_width {2*lk.half_span:.0f}")
    dim_h(-lk.pivot_y, 0, top + 4, f"wing_pivot_offset {lk.pivot_y:.0f}")
    dim_h(-lk.half_span, -lk.pivot_y, top - 12,
          f"wing_stow_offset {lk.stow_offset:.0f} (derived)")
    dim_v(lk.half_span + 26, 0, top, f"bike_height {top:.1f}")
    dim_v(-lk.half_span - 26, 0, lk.wing_bottom,
          f"ground_clearance {lk.wing_bottom:.0f}", ha="right", dx=-4)
    dim_v(-lk.half_span - 26, lk.wing_bottom, top,
          f"wing_length {lk.wing_length:.1f} (derived)", ha="right", dx=-4)
    dim_v(-14, lk.pivot_z, lk.servo[1], f"servo_offset {lk.servo[1]-lk.pivot_z:.0f}",
          ha="right", dx=-4)

    from matplotlib.lines import Line2D
    ax.legend(handles=[
        Line2D([], [], color="#7f2fa0", lw=2.6, label="link 1  servo crank"),
        Line2D([], [], color="#2ca02c", lw=2.0, label="link 2  coupler (derived)"),
        Line2D([], [], color="0.3", lw=2.0, label="link 3  rocker = the wing"),
        Line2D([], [], color="0.45", lw=1.2, ls=":", label="link 4  ground (virtual)"),
    ], fontsize=7.5, loc="upper left", framealpha=0.92)

    off = cfg["mechanism"]["wing_attach_offset"]
    txt = (f"wing_attach_offset  ({off[0]:.0f}, {off[1]:.0f}) from pivot\n"
           f"first_link_angle    {lk.crank_angle0['right']:.0f}° (right crank, CCW from +y)\n"
           f"between first links {cfg['mechanism']['angle_between_first_links']:.0f}°\n"
           f"link1  R {lk.crank['right']:.1f}   L {lk.crank['left']:.1f}\n"
           f"link2  R {lk.coupler['right']:.2f}   L {lk.coupler['left']:.2f}   (derived)")
    ax.text(0.985, 0.985, txt, transform=ax.transAxes, fontsize=7,
            family="monospace", ha="right", va="top",
            bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="0.7", alpha=0.93))

    ax.set_aspect("equal")
    ax.set_xlabel("y [mm]   (+y = the bike's LEFT)")
    ax.set_ylabel("z [mm] above the floor")
    ax.set_title("wing linkage — STOWED, one four-bar per wing on a single servo",
                 fontsize=10)
    ax.grid(alpha=0.25)
    ax.set_xlim(-lk.half_span - 62, lk.half_span + 62)
    ax.set_ylim(-16, lk.wing_top + 62)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")
    return lk


def cmd_deploy(cfg, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lk = Linkage(cfg)
    n = int(cfg["stroke"]["steps"])
    travels = np.linspace(0.0, cfg["stroke"]["servo_travel_deg"], n)
    fig, ax = plt.subplots(figsize=(8.4, 7.0))
    for k, t in enumerate(travels):
        a = 0.22 + 0.78 * k / max(n - 1, 1)
        _draw(ax, lk, float(t), labels=False, alpha=a,
              wing_lw=5.0 if k in (0, n - 1) else 2.5)
    ax.plot(*lk.servo, "s", ms=11, color="#7f2fa0", zorder=6)
    ax.axhline(0, color="k", lw=2)
    ax.set_aspect("equal")
    ax.set_xlabel("y [mm]   (+y = the bike's LEFT)")
    ax.set_ylabel("z [mm] above the floor")
    ax.set_title(f"wing linkage — stowed → deployed over "
                 f"{travels[-1]:.0f}° of servo travel", fontsize=10)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")
    return lk


# --------------------------------------------------------------------------
# statics


GRAVITY = 9.81
BIKE_MASS_KG = 1.159      # bare bike 1.016 + the righting kit, as measured in
                          #   bike_params.yaml. A linkage kit will differ.
COM_Z_MM = 123.5          # CoM height above the floor, bike upright


def torque_curve(lk: "Linkage", fell_on: str, mass: float = BIKE_MASS_KG,
                 com_z: float = COM_Z_MM):
    """Servo torque through the stroke, for a bike lying on ONE wing.

    The load case, per the design brief: the bike is on its side ON the wing,
    the wing stays flat on the ground, and driving the wing out rotates the
    BIKE up around it. So with the wing taken as ground-fixed:

        roll  phi = 90 - theta          the body rotates as the wing deploys
        tau_wing = m g * |y_com - y_pivot|      weight moment about the pivot
        tau_servo = tau_wing * dtheta/dpsi      virtual work through the linkage

    Only the fallen-side wing is loaded; the other swings in air. Since the two
    sides have different velocity ratios, the servo torque DEPENDS ON WHICH
    SIDE THE BIKE FELL ON — an asymmetry the geared design does not have.

    Returns (servo angle, roll, tau_wing, tau_servo), all arrays.
    """
    ts, ar, al = sweep_window(lk)
    dep = ar if fell_on == "right" else al
    ratio = np.gradient(dep, ts)
    # CoM relative to that wing's pivot, in BODY frame (mm).
    side = -1 if fell_on == "right" else 1
    ry = -side * lk.pivot_y
    rz = com_z - lk.pivot_z
    phi = np.deg2rad(90.0 - dep)                 # roll, 90 = on its side
    # Rotate the body vector into the world as the bike rolls up.
    y_world = np.cos(phi) * ry + np.sin(phi) * rz
    tau_wing = mass * GRAVITY * np.abs(y_world) / 1000.0      # N.m
    return ts, np.degrees(phi), tau_wing, tau_wing * ratio


def walk_to(lk: "Linkage", travel: float, step: float = 2.0) -> None:
    """Step the linkage from stow up to `travel` so the branch memory is right.

    `solve` picks the assembly branch nearest the previous pose, so jumping
    straight to a large travel can land on the branch the mechanism cannot
    physically reach. Anything drawing a single far-from-stow pose has to walk
    there.
    """
    lk._last = {}
    t = 0.0
    while t < travel:
        for tag in ("right", "left"):
            lk.solve(tag, t)
        t = min(t + step, travel)
    for tag in ("right", "left"):
        lk.solve(tag, travel)


def _turn_marker(ax, lk: "Linkage", travel: float):
    """Curved arrow at the servo showing which way it drives.

    Travel is measured CCW from +y in the drawn frame, so positive travel is
    ANTICLOCKWISE as plotted. Worth marking explicitly: the front/back view
    flips handedness relative to standing behind the bike, and the two cranks
    are 180 deg apart, so "which way does it turn" is not guessable from a
    still.
    """
    from matplotlib.patches import FancyArrowPatch
    ccw = travel >= 0
    # ABOVE the servo: the cranks fan out sideways and the couplers run down,
    # so overhead is the only reliably clear space in both poses.
    r = 34.0
    a0, a1 = (35.0, 145.0) if ccw else (145.0, 35.0)
    p0 = lk.servo + r * np.array([np.cos(np.deg2rad(a0)), np.sin(np.deg2rad(a0))])
    p1 = lk.servo + r * np.array([np.cos(np.deg2rad(a1)), np.sin(np.deg2rad(a1))])
    ax.add_patch(FancyArrowPatch(
        tuple(p0), tuple(p1), connectionstyle=f"arc3,rad={-0.42 if ccw else 0.42}",
        arrowstyle="-|>", mutation_scale=22, lw=2.6, color="#7f2fa0", zorder=8))
    ax.annotate(f"{'CCW' if ccw else 'CW'}  {abs(travel):.0f}°",
                lk.servo + np.array([0.0, r + 12]), color="#7f2fa0",
                fontsize=10, ha="center", va="bottom", fontweight="bold",
                zorder=8)


def cmd_panels(cfg, out: Path):
    """Stowed and deployed side by side -- the two poses that have to be right."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lk = Linkage(cfg)
    travel = float(cfg["stroke"]["servo_travel_deg"])
    fig, axes = plt.subplots(1, 2, figsize=(11.6, 6.4), sharex=True, sharey=True)

    for ax, t, name in ((axes[0], 0.0, "STOWED"), (axes[1], travel, "DEPLOYED")):
        walk_to(lk, t)
        _draw(ax, lk, t, labels=False)
        ax.plot(*lk.servo, "s", ms=11, color="#7f2fa0", zorder=6)
        for side in (-1, 1):
            ax.plot([lk.pivot(side)[0], lk.servo[0]],
                    [lk.pivot(side)[1], lk.servo[1]], ls=":", lw=1.1,
                    color="0.5", zorder=1)
        ax.axhline(0, color="k", lw=2)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.set_xlabel("y [mm]   (+y = the bike's LEFT)")
        angs = []
        for tag in ("right", "left"):
            _, wd = lk.solve(tag, t)
            angs.append(0.0 if wd is None else abs(wd))
        ax.set_title(f"{name}   servo {t:.0f}°\n"
                     f"right wing {angs[0]:.1f}°   left wing {angs[1]:.1f}°",
                     fontsize=10)
    _turn_marker(axes[1], lk, travel)
    axes[0].set_ylabel("z [mm] above the floor")

    # Lowest point either wing reaches anywhere in the stroke. Drawn with the
    # bike UPRIGHT, so this is the clearance when the wings are deployed and
    # the bike is back on its wheels -- the pose the righting sequence ends in.
    lk._last = {}
    worst, worst_tag = 1e9, ""
    for t in np.arange(0.0, travel + 0.5, 1.0):
        for tag in ("right", "left"):
            _, wd = lk.solve(tag, float(t))
            if wd is None:
                continue
            lo, hi = lk.wing_line(tag, wd)
            if min(lo[1], hi[1]) < worst:
                worst, worst_tag = min(lo[1], hi[1]), tag
    axes[1].annotate(
        f"lowest wing point over the stroke: {worst:+.1f} mm ({worst_tag})"
        + ("" if worst >= 0 else "  — CLIPS THE FLOOR"),
        (0.5, 0.02), xycoords="axes fraction", ha="center", fontsize=8,
        color="0.25" if worst >= 0 else "#d62728",
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="0.75", alpha=0.9))

    from matplotlib.lines import Line2D
    axes[0].legend(handles=[
        Line2D([], [], color="#7f2fa0", lw=2.6, label="link 1  crank"),
        Line2D([], [], color="#2ca02c", lw=2.0, label="link 2  coupler"),
        Line2D([], [], color="0.3", lw=2.0, label="link 3  rocker = wing"),
        Line2D([], [], color="0.5", lw=1.1, ls=":", label="link 4  ground"),
    ], fontsize=7.5, loc="upper left", framealpha=0.92)
    fig.suptitle("wing linkage — start and end of the deploy stroke", fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")
    return lk


def cmd_righting_video(cfg, out: Path, fps: int = 25, seconds: float = 7.0):
    """Animate the GROUND-frame righting, both fall sides side by side."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    try:
        import imageio.v2 as imageio
    except ImportError as e:
        raise SystemExit("needs imageio: pip install -e '.[viz]'") from e

    lk = Linkage(cfg)
    travel = float(cfg["stroke"]["servo_travel_deg"])
    n = int(fps * seconds)
    sched = np.concatenate([np.linspace(0, travel, n // 2),
                            np.linspace(travel, 0, n - n // 2)])
    w_n = BIKE_MASS_KG * GRAVITY
    SC = 3.0
    out.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(out, fps=fps, macro_block_size=1)
    for k, t in enumerate(sched):
        fig, axes = plt.subplots(1, 2, figsize=(11.0, 5.6), sharex=True, sharey=True)
        for ax, fell in zip(axes, ("right", "left")):
            walk_to(lk, float(t))
            st = statics(lk, fell, float(t))
            G, lo, hi = ground_frame(lk, fell, float(t))
            if st is None or G is None:
                continue
            _, wd = lk.solve(fell, float(t))
            ax.plot([lo[0], hi[0]], [lo[1], hi[1]], color=_C[fell], lw=6,
                    solid_capstyle="round", zorder=3)
            hw, hh = lk.half_span, lk.wing_top
            box = np.array([[-hw, 0], [hw, 0], [hw, hh], [-hw, hh], [-hw, 0]], float)
            body = np.array([G(q) for q in box])
            ax.plot(body[:, 0], body[:, 1], color="0.4", lw=1.5, zorder=2)
            o_tag = "left" if fell == "right" else "right"
            _, od = lk.solve(o_tag, float(t))
            olo, ohi = (G(x) for x in lk.wing_line(o_tag, od))
            ax.plot([olo[0], ohi[0]], [olo[1], ohi[1]], color=_C[o_tag],
                    lw=4, alpha=0.5, zorder=2)
            c, at = G(st["crank_tip"]), G(st["attach"])
            sv, pv = G(lk.servo), G(st["pivot"])
            ax.plot([sv[0], c[0]], [sv[1], c[1]], color="#7f2fa0", lw=2.4, zorder=4)
            ax.plot([c[0], at[0]], [c[1], at[1]], color="#2ca02c", lw=2.0, zorder=4)
            ax.plot([at[0], pv[0]], [at[1], pv[1]], color=_C[fell], lw=1.8, zorder=4)
            for q in (sv, c, at, pv):
                ax.plot(*q, "o", ms=4, color="k", zorder=6)
            com = G(np.array([0.0, COM_Z_MM]))
            ax.plot(*com, "o", ms=8, color="#111", zorder=7)
            ax.annotate("", com + [0, -w_n * SC], com, zorder=7,
                        arrowprops=dict(arrowstyle="-|>", color="#b00", lw=2.2,
                                        mutation_scale=13))
            grf = np.array([com[0], 0.0])       # resultant sits under the CoM
            ax.annotate("", grf + [0, w_n * SC], grf, zorder=7,
                        arrowprops=dict(arrowstyle="-|>", color="#0a0", lw=2.2,
                                        mutation_scale=13))
            u = at - c
            u = u / max(np.linalg.norm(u), 1e-9)
            ax.annotate("", at + u * st["f_coupler"] * SC, at, zorder=7,
                        arrowprops=dict(arrowstyle="-|>", color="#2ca02c",
                                        lw=1.5, mutation_scale=10))
            ax.axhline(0, color="k", lw=2)
            ax.set_aspect("equal"); ax.grid(alpha=0.2)
            ax.set_xlim(-250, 250); ax.set_ylim(-20, 300)
            ax.set_xlabel("y [mm]")
            ax.set_title(f"fell {fell}   servo {t:5.0f}°   roll {st['roll_deg']:5.1f}°\n"
                         f"$\\tau_{{servo}}$ {st['tau_servo']:.3f} N·m   "
                         f"$F_{{coupler}}$ {st['f_coupler']:5.1f} N", fontsize=9)
        axes[0].set_ylabel("z above floor [mm]")
        fig.suptitle("righting itself — wing flat on the floor, body rotating up",
                     fontsize=11)
        fig.tight_layout()
        fig.canvas.draw()
        writer.append_data(np.ascontiguousarray(
            np.asarray(fig.canvas.buffer_rgba())[:, :, :3]))
        plt.close(fig)
    writer.close()
    print(f"wrote {out}  ({len(sched)} frames, {fps} fps)")
    return lk


PANEL_T = 4.0          # mm, ABS sheet -- same stock as the wing plate
PANEL_CLEAR = 15.0     # mm, ground clearance under the fixed skirt


def stick_bands(lk, travel_max: float, n: int = 241):
    """How high a FIXED side panel may reach before the wing sweeps into it.

    The panel sits in the wing's own plane (outer face at the envelope
    half-width, `PANEL_T` thick inboard of it). The wing is a line in this
    y-z section, so for each step of the stroke we ask how low it dips WITHIN
    that y-band, and the skirt has to stay under the minimum of that.

    Returns (skirt_top_mm, stowed_bottom_mm). The gap between them is the
    clearance the mechanism needs and is why the fixed panel cannot simply run
    up to the stowed wing.
    """
    y_out = lk.half_span
    y_in = y_out - PANEL_T
    lo_z = np.inf
    for t in np.linspace(0.0, travel_max, n):
        walk_to(lk, float(t))
        for tag in ("right", "left"):
            _, wd = lk.solve(tag, float(t))
            if wd is None:
                continue
            a, b = lk.wing_line(tag, wd)
            a, b = np.array(a, float), np.array(b, float)
            # sample the segment; keep the points whose |y| is inside the band
            for f in np.linspace(0.0, 1.0, 61):
                q = a + f * (b - a)
                if y_in - 1e-9 <= abs(q[0]) <= y_out + 1e-9:
                    lo_z = min(lo_z, q[1])
    walk_to(lk, 0.0)
    _, wd0 = lk.solve("left", 0.0)
    stow_lo = min(lk.wing_line("left", wd0)[0][1], lk.wing_line("left", wd0)[1][1])
    return float(lo_z), float(stow_lo)


def draw_stick(ax, lk, skirt_top: float):
    """The proposed FIXED panels, in the wing's plane, both sides."""
    import matplotlib.patches as mp
    y_out, y_in = lk.half_span, lk.half_span - PANEL_T
    for side in (-1, 1):
        y0 = side * y_in if side > 0 else side * y_out
        ax.add_patch(mp.Rectangle((min(side * y_in, side * y_out), PANEL_CLEAR),
                                  PANEL_T, max(0.0, skirt_top - PANEL_CLEAR),
                                  facecolor="#ff9f1c", edgecolor="#b36b00",
                                  alpha=0.55, zorder=1.5))
    ax.axhline(skirt_top, color="#b36b00", lw=0.9, ls="--", alpha=0.8, zorder=1.4)


def cmd_video(cfg, out: Path, fps: int = 30, seconds: float = 6.0,
              stick: bool = False):
    """Animate the mechanism deploying and retracting.

    Self-contained: this is the 2D kinematic model, not MuJoCo, so it shows
    exactly what the linkage solver believes and nothing else. Retract is
    included because it is the half of the cycle a dead point makes
    interesting -- if the mechanism can only be driven one way, that shows up
    here as a stall rather than as a number.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    try:
        import imageio.v2 as imageio
    except ImportError as e:
        raise SystemExit("needs imageio: pip install -e '.[viz]'") from e

    lk = Linkage(cfg)
    travel = float(cfg["stroke"]["servo_travel_deg"])
    n = int(fps * seconds)
    # Out and back, so the retract is on screen too.
    half = n // 2
    schedule = np.concatenate([np.linspace(0, travel, half),
                               np.linspace(travel, 0, n - half)])
    ts, ar, al = sweep_window(lk)
    _, _, twr, tsr = torque_curve(lk, "right")
    skirt_top = stow_lo = None
    if stick:
        skirt_top, stow_lo = stick_bands(lk, travel)
        print(f"fixed-skirt clearance, in the wing's own plane "
              f"(|y| {lk.half_span - PANEL_T:.1f}..{lk.half_span:.1f} mm):")
        print(f"  wing dips to        {skirt_top:6.1f} mm during the stroke")
        print(f"  stowed wing bottom  {stow_lo:6.1f} mm")
        print(f"  -> skirt may reach  {skirt_top:6.1f} mm; the open band it "
              f"cannot cover is {stow_lo - skirt_top:.1f} mm tall")

    out.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(out, fps=fps, macro_block_size=1)
    lim = max(200.0, lk.wing_top + 20)
    for k, t in enumerate(schedule):
        walk_to(lk, float(t))
        fig, (ax, axr) = plt.subplots(
            2, 1, figsize=(7.2, 8.0), height_ratios=[3, 1])
        _draw(ax, lk, float(t), labels=False)
        ax.plot(*lk.servo, "s", ms=11, color="#7f2fa0", zorder=6)
        for side in (-1, 1):
            ax.plot([lk.pivot(side)[0], lk.servo[0]],
                    [lk.pivot(side)[1], lk.servo[1]], ls=":", lw=1.1,
                    color="0.5", zorder=1)
        if stick:
            draw_stick(ax, lk, skirt_top)
        ax.axhline(0, color="k", lw=2)
        ax.set_aspect("equal"); ax.grid(alpha=0.25)
        ax.set_xlim(-lim, lim); ax.set_ylim(-25, lk.wing_top + 45)
        ax.set_xlabel("y [mm]   (+y = the bike's LEFT)")
        ax.set_ylabel("z [mm] above the floor")
        phase = "DEPLOY" if k < half else "RETRACT"
        angs = []
        for tag in ("right", "left"):
            _, wd = lk.solve(tag, float(t))
            angs.append(0.0 if wd is None else abs(wd))
        ax.set_title(f"{phase}   servo {t:5.0f}°   "
                     f"wings {angs[0]:5.1f}° / {angs[1]:5.1f}°", fontsize=11)

        axr.plot(ts, np.abs(tsr), color="#d62728", lw=1.8,
                 label="servo torque, fell right")
        axr.axhline(0.80 * 9.9 / 12, color="k", lw=0.9,
                    label="XC330 stall @9.9 V")
        axr.axvline(min(t, ts[-1]), color="#7f2fa0", lw=1.6)
        axr.set_xlim(0, ts[-1]); axr.set_ylim(0, max(0.7, np.abs(tsr).max() * 1.15))
        axr.set_xlabel("servo travel [deg]"); axr.set_ylabel("N·m")
        axr.legend(fontsize=7, loc="upper left"); axr.grid(alpha=0.3)
        fig.tight_layout()
        fig.canvas.draw()
        frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        writer.append_data(np.ascontiguousarray(frame))
        plt.close(fig)
    writer.close()
    print(f"wrote {out}  ({len(schedule)} frames, {fps} fps)")
    return lk


def statics(lk: "Linkage", fell_on: str, travel: float,
            mass: float = BIKE_MASS_KG, com_z: float = COM_Z_MM):
    """Full quasi-static force balance at one stroke position.

    The bike lies ON the wing; the wing stays flat on the ground; driving the
    wing out rotates the BODY up about the wing pivot. So with the wing taken
    as ground-fixed, the body's free body is: weight at the CoM, a pin reaction
    at the wing pivot, and the mechanism moment. That gives

        tau_wing = m g * (horizontal pivot->CoM distance)

    -- which is where the bike's weight enters, and it has been in the torque
    curve all along. It is just invisible when the bike is not drawn.

    The COUPLER IS A TWO-FORCE MEMBER: pinned at both ends and effectively
    massless, so its force is purely AXIAL. That closes the rest in one step:

        F_coupler = tau_wing / d_perp(pivot, coupler line)
        tau_servo = F_coupler * d_perp(servo, coupler line)

    and every pin force follows. d_perp at the wing is r_rocker*sin(mu), so a
    small transmission angle shows up here as a large coupler force -- the same
    fact the mu constraint encodes, now in newtons.

    Returns a dict of scalars and vectors, all SI (N, N.m, m).
    """
    side = -1 if fell_on == "right" else 1
    at, wing_deg = lk.solve(fell_on, travel)
    if at is None:
        return None
    p = lk.pivot(side)
    c = lk.crank_tip(fell_on, travel)

    # Body attitude: the wing is on the ground, so the body has rolled up by
    # however far the wing has swung relative to it.
    phi = np.deg2rad(90.0 - abs(wing_deg))
    ry, rz = -side * lk.pivot_y, com_z - lk.pivot_z        # CoM rel pivot, body
    lever_mm = np.cos(phi) * ry + np.sin(phi) * rz          # horizontal, world
    tau_wing = mass * GRAVITY * abs(lever_mm) / 1000.0      # N.m

    def perp(point, a, b):
        """Perpendicular distance from `point` to the line through a,b [m]."""
        d = b - a
        n = np.linalg.norm(d)
        if n < 1e-9:
            return 0.0
        # 2D cross product written out: np.cross on 2-vectors is deprecated.
        r = point - a
        return abs(float(d[0] * r[1] - d[1] * r[0])) / n / 1000.0

    d_wing = perp(p, c, at)          # pivot to the coupler's line of action
    d_servo = perp(lk.servo, c, at)  # servo to the same line
    f_coupler = tau_wing / max(d_wing, 1e-6)
    tau_servo = f_coupler * d_servo

    # Pin at the wing pivot: reacts the body's weight plus the coupler push.
    u = (at - c) / max(np.linalg.norm(at - c), 1e-9)        # coupler direction
    f_c_vec = f_coupler * u
    weight = np.array([0.0, -mass * GRAVITY])
    f_pivot = -(weight + f_c_vec)
    # Ground reaction on the wing: whatever holds the whole thing up.
    grf = np.array([0.0, mass * GRAVITY])
    return {
        "wing_deg": abs(wing_deg), "roll_deg": np.degrees(phi),
        "tau_wing": tau_wing, "tau_servo": tau_servo,
        "f_coupler": f_coupler, "f_pivot": float(np.linalg.norm(f_pivot)),
        "f_crank_pin": f_coupler,      # two-force member: same at both ends
        "f_servo_bearing": f_coupler,  # reacted through the crank into the shaft
        "grf": grf, "grf_at": p, "lever_mm": lever_mm,
        "attach": at, "crank_tip": c, "pivot": p,
        "d_wing_mm": d_wing * 1000.0, "d_servo_mm": d_servo * 1000.0,
    }


def ground_frame(lk: "Linkage", fell: str, travel: float):
    """Transform from BODY frame to GROUND frame for a bike lying on `fell`.

    Rotates about the wing pivot until that wing lies flat, then drops the
    assembly so the wing sits on z = 0. Returns (G, wing_lo, wing_hi) where G
    maps a body-frame point into the ground frame.

    Extracted because mixing the two frames is an easy and silent mistake:
    `statics` reports the CoM lever as a GROUND-frame horizontal distance while
    `wing_line` returns BODY-frame endpoints, and comparing them directly
    reported the ground reaction running off the end of the panel when it does
    nothing of the kind.
    """
    side = -1 if fell == "right" else 1
    st = statics(lk, fell, travel)
    if st is None:
        return None, None, None
    p = st["pivot"]
    _, wing_deg = lk.solve(fell, travel)
    rot = -side * np.deg2rad(st["roll_deg"])
    R = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])
    raw = [(np.asarray(q, float) - p) @ R.T + p
           for q in lk.wing_line(fell, wing_deg)]
    drop = min(raw[0][1], raw[1][1])

    def G(q):
        v = (np.asarray(q, float) - p) @ R.T + p
        return np.array([v[0], v[1] - drop])

    return G, G(lk.wing_line(fell, wing_deg)[0]), G(lk.wing_line(fell, wing_deg)[1])


def cmd_righting(cfg, out: Path):
    """The bike righting ITSELF, both fall sides, with the force vectors.

    Drawn in the GROUND frame, which is the whole point. Everywhere else in
    this study the bike is held still and the wing swings; here the wing is
    pinned flat on the floor -- because that is what it is doing, the bike is
    lying on it -- and the BODY rotates up around it. Same kinematics, one
    rigid transform apart, but only this frame shows whether anything is
    actually being lifted.

    That matters because a torque curve cannot answer it. `tau_wing = m g *
    lever` has the bike's weight in it and always did, but newton-metres look
    identical whether the mechanism is raising a bike or waving in free air.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lk = Linkage(cfg)
    travel = float(cfg["stroke"]["servo_travel_deg"])
    fracs = [0.0, 0.35, 0.65, 1.0]
    fig, axes = plt.subplots(2, len(fracs), figsize=(4.1 * len(fracs), 8.6),
                             sharex=True, sharey=True)
    w_n = BIKE_MASS_KG * GRAVITY
    SC = 3.0                                   # mm of arrow per newton

    for row, fell in enumerate(("right", "left")):
        side = -1 if fell == "right" else 1
        lk._last = {}
        for col, fr in enumerate(fracs):
            t = fr * travel
            walk_to(lk, t)
            st = statics(lk, fell, t)
            ax = axes[row][col]
            if st is None:
                continue
            p = st["pivot"]
            _, wing_deg = lk.solve(fell, t)
            # ONE rigid transform from body frame to ground frame: rotate about
            # the wing pivot until this wing lies flat, then drop the whole
            # assembly so that wing sits on z = 0.
            rot = -side * np.deg2rad(st["roll_deg"])
            R = np.array([[np.cos(rot), -np.sin(rot)],
                          [np.sin(rot), np.cos(rot)]])
            def T(q):
                return (np.asarray(q, float) - p) @ R.T + p
            lo, hi = (T(x) for x in lk.wing_line(fell, wing_deg))
            drop = min(lo[1], hi[1])
            def G(q):
                v = T(q); return np.array([v[0], v[1] - drop])

            # the fallen wing, flat on the floor
            lo, hi = G(lk.wing_line(fell, wing_deg)[0]), G(lk.wing_line(fell, wing_deg)[1])
            ax.plot([lo[0], hi[0]], [lo[1], hi[1]], color=_C[fell], lw=6,
                    solid_capstyle="round", zorder=3)
            # the body envelope
            hw, hh = lk.half_span, lk.wing_top
            box = np.array([[-hw, 0.0], [hw, 0.0], [hw, hh], [-hw, hh], [-hw, 0.0]])
            body = np.array([G(q) for q in box])
            ax.plot(body[:, 0], body[:, 1], color="0.4", lw=1.5, zorder=2)
            # the OTHER wing, still stowed on the body
            o_tag = "left" if fell == "right" else "right"
            _, o_deg = lk.solve(o_tag, t)
            olo, ohi = (G(x) for x in lk.wing_line(o_tag, o_deg))
            ax.plot([olo[0], ohi[0]], [olo[1], ohi[1]], color=_C[o_tag],
                    lw=4, alpha=0.55, zorder=2)
            # the linkage
            c, at = G(st["crank_tip"]), G(st["attach"])
            sv, pv = G(lk.servo), G(p)
            ax.plot([sv[0], c[0]], [sv[1], c[1]], color="#7f2fa0", lw=2.4, zorder=4)
            ax.plot([c[0], at[0]], [c[1], at[1]], color="#2ca02c", lw=2.0, zorder=4)
            ax.plot([at[0], pv[0]], [at[1], pv[1]], color=_C[fell], lw=1.8, zorder=4)
            for q in (sv, c, at, pv):
                ax.plot(*q, "o", ms=4, color="k", zorder=6)

            com = G(np.array([0.0, COM_Z_MM]))
            ax.plot(*com, "o", ms=8, color="#111", zorder=7)
            ax.annotate("", com + np.array([0, -w_n * SC]), com,
                        arrowprops=dict(arrowstyle="-|>", color="#b00", lw=2.2,
                                        mutation_scale=13), zorder=7)
            # Ground reaction, DIRECTLY UNDER THE CoM. Not a modelling
            # choice -- it falls out. The body is supported only through the
            # wing, so wing moment balance gives
            #     x_grf = x_pivot + tau_wing / N ,  N = m g
            # and tau_wing/N is exactly the horizontal pivot->CoM lever, i.e.
            # the resultant sits under the centre of mass. Picking the wing's
            # low edge (what this did before) is meaningless once the panel is
            # flat: both ends are level and floating-point noise chose one,
            # which is why the arrow jumped from inside to outside.
            grf_at = np.array([com[0], 0.0])
            ax.annotate("", grf_at + np.array([0, w_n * SC]), grf_at,
                        arrowprops=dict(arrowstyle="-|>", color="#0a0", lw=2.2,
                                        mutation_scale=13), zorder=7)
            u = at - c
            u = u / max(np.linalg.norm(u), 1e-9)
            ax.annotate("", at + u * st["f_coupler"] * SC, at,
                        arrowprops=dict(arrowstyle="-|>", color="#2ca02c",
                                        lw=1.5, mutation_scale=10), zorder=7)
            ax.axhline(0, color="k", lw=2)
            ax.set_aspect("equal"); ax.grid(alpha=0.2)
            ax.set_title(f"servo {t:.0f}°   roll {st['roll_deg']:.0f}°\n"
                         f"$\\tau_{{servo}}$ {st['tau_servo']:.3f} N·m   "
                         f"$F_{{coupler}}$ {st['f_coupler']:.1f} N", fontsize=9)
            if col == 0:
                ax.set_ylabel(f"fell {fell}\nz above floor [mm]")
    for ax in axes[1]:
        ax.set_xlabel("y [mm]")
    axes[0][0].set_xlim(-250, 250); axes[0][0].set_ylim(-20, 300)
    fig.suptitle("righting itself, ground frame — the wing is on the floor and "
                 "the BODY rotates up.   red = weight,  green up = ground "
                 "reaction,  green along link = coupler force", fontsize=10.5)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")
    return lk


def cmd_forces(cfg, out: Path):
    """Peak pin forces over the stroke, both fall sides. Printed, not plotted."""
    lk = Linkage(cfg)
    travel = float(cfg["stroke"]["servo_travel_deg"])
    print("Peak QUASI-STATIC pin loads over the deploy stroke.\n"
          "The coupler is a two-force member, so its pins at BOTH ends carry\n"
          "the same axial force; the wing pivot additionally reacts the body's\n"
          "weight. Bike mass %.3f kg.\n" % BIKE_MASS_KG)
    print(f"{'fell on':>8} {'coupler pins':>13} {'wing pivot':>12} "
          f"{'servo shaft':>12} {'at servo deg':>13}")
    worst_off = {}
    for fell in ("right", "left"):
        side = -1 if fell == "right" else 1
        lk._last = {}
        best = None
        off = []
        t = 0.0
        while t <= travel:
            st = statics(lk, fell, t)
            if st:
                if best is None or st["f_coupler"] > best[0]["f_coupler"]:
                    best = (st, t)
                # Where the resultant sits ALONG the wing, 0 = near tip,
                # 1 = far tip. Outside [0, 1] the bike is tipping off the panel
                # rather than being lifted by it. Both points taken in the
                # GROUND frame so the comparison is meaningful.
                G, lo, hi = ground_frame(lk, fell, t)
                if G is not None:
                    com = G(np.array([0.0, COM_Z_MM]))
                    span = hi - lo
                    n2 = float(span @ span)
                    if n2 > 1e-9:
                        off.append(float(((com - lo) @ span) / n2))
            t += 2.0
        st, tt = best
        worst_off[fell] = (min(off), max(off)) if off else (float("nan"),) * 2
        print(f"{fell:>8} {st['f_coupler']:>12.1f}N {st['f_pivot']:>11.1f}N "
              f"{st['f_servo_bearing']:>11.1f}N {tt:>12.0f}")
    print("\nWhere the ground reaction sits along the wing "
          "(0 = near tip, 1 = far tip):")
    for fell, (a, b) in worst_off.items():
        note = ("inside the panel throughout" if a >= 0.0 and b <= 1.0 else
                "slips just past the near tip early in the stroke")
        print(f"  fell {fell:5s}: {a:+.2f} .. {b:+.2f}   {note}")
    print("  Note: a small NEGATIVE reading is not a modelling error. Early in\n"
          "  the stroke the bike is also resting on its roof and bumper, so the\n"
          "  'wing carries everything' assumption is weakest exactly there --\n"
          "  which is where this reports the CoM sitting slightly outboard of\n"
          "  the panel. The MuJoCo model is what resolves the real split.")
    return lk


def cmd_torque(cfg, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lk = Linkage(cfg)
    stall99, stall111 = 0.80 * 9.9 / 12, 0.76
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8.0, 7.0), sharex=True)
    print(f"{'fell on':>8} {'peak servo':>11} {'at servo':>9} {'at roll':>8} "
          f"{'frac 9.9V':>10}")
    for fell, col in (("right", "#d62728"), ("left", "#1f77b4")):
        ts, roll, tw, tsv = torque_curve(lk, fell)
        a1.plot(ts, tw, color=col, ls="--", lw=1.2,
                label=f"at the wing, fell {fell}")
        a1.plot(ts, np.abs(tsv), color=col, lw=2.2,
                label=f"at the SERVO, fell {fell}")
        a2.plot(ts, np.gradient(ts * 0 + (roll[0] - roll), ts), alpha=0)  # keep axes
        k = int(np.argmax(np.abs(tsv)))
        print(f"{fell:>8} {abs(tsv[k]):>10.3f}N {ts[k]:>8.0f}° {roll[k]:>7.0f}° "
              f"{abs(tsv[k])/stall99:>10.2f}")
    for y, lbl, st in ((stall99, "XC330 stall @9.9 V (0.66)", "-"),
                       (stall111, "@11.1 V (0.76)", ":")):
        a1.axhline(y, color="k", lw=0.9, ls=st)
        a1.annotate(lbl, (0.02, y), xycoords=("axes fraction", "data"),
                    fontsize=7, va="bottom")
    a1.axhline(0.339, color="#2ca02c", lw=1.2, ls="-.")
    a1.annotate("geared 2:1 needs 0.339", (0.02, 0.339),
                xycoords=("axes fraction", "data"), fontsize=7, va="bottom",
                color="#2ca02c")
    a1.set_ylabel("torque [N·m]")
    a1.legend(fontsize=7, ncol=2)
    a1.grid(alpha=0.3)
    a1.set_title("wing linkage — what the servo has to hold, "
                 "bike lying on one wing", fontsize=10)

    ts, ar, al = sweep_window(lk)
    a2.plot(ts, np.gradient(ar, ts), color="#d62728", label="right dθ/dψ")
    a2.plot(ts, np.gradient(al, ts), color="#1f77b4", label="left dθ/dψ")
    a2.axhline(0.5, color="#2ca02c", ls="-.", lw=1.2, label="geared 2:1 (flat)")
    a2.set_xlabel("servo travel [deg]")
    a2.set_ylabel("wing deg per servo deg")
    a2.legend(fontsize=7)
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"\nwrote {out}")
    return lk


# --------------------------------------------------------------------------
# optimisation


# name -> (path in cfg, low, high). The brief's guidance on each:
#   pivot offset   smaller than the first guess -- 30 mm made it awkward to
#                  stow again after deploying.
#   ground clear.  larger. It also SHORTENS the wing, which was a bit too long
#                  (wing_length = bike_height - ground_clearance).
#   attach offset  free, and can sit much further up the wing, which just
#                  means longer links.
#   servo z        free, and generally wants to move with the attach point.
#   crank angles   free; the 180 deg between the two arms can change, at the
#                  cost of the links fouling each other in the real part.
# Deliberately wide. The first pass pinned three variables against their
# bounds, which means those numbers were reporting my guesses rather than the
# mechanism's preference. Crank angles are now unrestricted, the servo may sit
# below the axle or far above it, and the attach point may sit below its own
# pivot -- all of which are buildable with curved links and z-separation, at
# the cost of care about the links fouling each other.
_VARS = [
    ("wing_pivot_offset",       ("mechanism", "wing_pivot_offset"),      5.0,  45.0),
    ("ground_clearance",        ("bike", "ground_clearance"),           10.0,  90.0),
    ("servo_offset",            ("mechanism", "servo_offset"),         -20.0, 140.0),
    ("attach_y",                ("mechanism", "wing_attach_offset", 0), -60.0,  60.0),
    ("attach_z",                ("mechanism", "wing_attach_offset", 1), -40.0, 160.0),
    ("link1_right",             ("mechanism", "wing_first_link_length", "right"), 8.0, 90.0),
    ("link1_left",              ("mechanism", "wing_first_link_length", "left"),  8.0, 90.0),
    ("first_link_angle_deg",    ("mechanism", "first_link_angle_deg"),   0.0, 360.0),
    ("angle_between_first_links", ("mechanism", "angle_between_first_links"), 0.0, 360.0),
]


def _apply(cfg: dict, x) -> dict:
    import copy as _copy
    c = _copy.deepcopy(cfg)
    for (_, path, _lo, _hi), v in zip(_VARS, x):
        node = c
        for k in path[:-1]:
            node = node[k]
        node[path[-1]] = float(v)
    return c


MIN_TRANSMISSION_DEG = 30.0
"""Worst transmission angle the mechanism may reach anywhere in the stroke.

The angle between coupler and rocker at the attach point. 90 deg transmits
perfectly; below ~40 deg the pair is nearly collinear and the coupler pushes
almost straight along the rocker, so it barely torques the output. At a few
degrees it is a DEAD POINT: no force authority over the wing, extreme
sensitivity to backlash and link tolerance, and no reliable way to drive back
out of it.

A torque-only objective will walk straight into one, because the dead point
here happens to sit where the bike is nearly upright and the load is almost
zero -- so it costs nothing on the metric while being the least buildable part
of the design. Priced explicitly for that reason.
"""

MIN_FLOOR_MM = 2.0
"""Clearance the wings must keep from the floor, anywhere in the stroke.

Measured with the bike UPRIGHT, which is not the pose it rights itself from --
during the stroke the bike is on its side and this floor line is not the ground
it pushes on. Where it bites is the pose the sequence ENDS in: back on its
wheels, wings still fully deployed, waiting to retract. A wing that is through
the floor there is a wing wedged against the ground while the policy is trying
to balance.
"""


def floor_clearance(lk: "Linkage", travel: float) -> float:
    """Lowest point either wing reaches over the stroke [mm]."""
    lk._last = {}
    worst = 1e9
    t = 0.0
    while t <= travel:
        for tag in ("right", "left"):
            _, wd = lk.solve(tag, t)
            if wd is None:
                return -1e3
            lo, hi = lk.wing_line(tag, wd)
            worst = min(worst, float(lo[1]), float(hi[1]))
        t += 3.0
    return worst


def min_transmission(lk: "Linkage", travel: float) -> float:
    """Worst transmission angle over the stroke [deg], both sides."""
    lk._last = {}
    worst = 180.0
    t = 0.0
    while t <= travel:
        for tag in ("right", "left"):
            side = -1 if tag == "right" else 1
            p, c = lk.pivot(side), lk.crank_tip(tag, t)
            at, _ = lk.solve(tag, t)
            if at is None:
                return 0.0
            v1, v2 = c - at, p - at
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 < 1e-9 or n2 < 1e-9:
                return 0.0
            mu = np.degrees(np.arccos(np.clip(float(v1 @ v2) / (n1 * n2), -1, 1)))
            worst = min(worst, min(mu, 180.0 - mu))
        t += 3.0
    return worst


def end_advantage(lk: "Linkage", travel: float) -> float:
    """Mechanical advantage at the fully deployed pose, worse of the two sides.

    MA = tau_wing / tau_servo = 1 / (dtheta/dpsi), taken kinematically so it
    does not depend on how much load happens to be there.

    High MA at the END is the toggle-clamp property: approaching the INPUT-side
    dead point (crank collinear with coupler) means the wing cannot backdrive
    the servo, so the deployed pose is held with little or no current. Gears
    cannot do this at all -- a 2:1 reduction needs continuous holding torque
    for as long as the wings are down. It is the one axis on which a linkage
    can beat them outright rather than trail them.

    The cost is sensitivity: near a dead point, link tolerance and backlash
    turn into large errors in the final wing angle, and driving THROUGH the
    dead point flips the assembly branch and jams.
    """
    ts, ar, al = sweep_window(lk, travel + 1.0)
    if len(ts) < 3:
        return 0.0
    gr, gl = np.gradient(ar, ts), np.gradient(al, ts)
    worst_rate = max(abs(float(gr[-1])), abs(float(gl[-1])))
    return 1.0 / max(worst_rate, 1e-6)


def peak_torque(lk: "Linkage") -> float:
    """Worst servo torque over the stroke, taking the worse of the two fall
    sides. That is the number the servo has to be specified against: the bike
    does not get to choose which side it falls on."""
    worst = 0.0
    for fell in ("right", "left"):
        ts, _roll, _tw, tsv = torque_curve(lk, fell)
        if not len(ts):
            return 1e3
        worst = max(worst, float(np.max(np.abs(tsv))))
    return worst


def _objective(x, cfg):
    """Negative worst-side deployment, with the constraints folded in.

    Scored at the END of the common monotonic window, which is the pose the
    mechanism can actually be driven to. Maximising the WORST side is the
    point: a linkage that throws one wing to 120 deg and leaves the other at
    50 has not righted anything.
    """
    try:
        lk = Linkage(_apply(cfg, x))
    except Exception:
        return 1e3
    T, r, l, err = best_pose(lk)
    if T < 5.0 or T > 350.0:
        return 1e3          # never leaves stow, or needs more than one turn
    kin = err + 0.01 * abs(r - l)
    if not _WITH_TORQUE:
        return kin
    # Kinematics FIRST, then torque. Many linkages solve the motion -- that is
    # the whole reason to bring statics in -- so the objective admits any
    # design that gets both wings to the target within `_KIN_TOL`, and then
    # ranks those by what the servo actually has to hold. Outside the
    # tolerance, torque is irrelevant: a mechanism that does not deploy cannot
    # be cheap.
    if kin > _KIN_TOL:
        return 100.0 + kin
    # Clearance is a soft constraint with a hard slope: 1 mm of floor
    # intrusion costs 0.05 N.m of "torque", so a design that gouges by a
    # centimetre is never cheaper than one that clears. Kept soft rather than
    # rejecting outright so the search can approach the boundary from inside
    # infeasible territory instead of falling off a cliff.
    short = max(0.0, MIN_FLOOR_MM - floor_clearance(lk, T))
    # Transmission angle is a constraint in BOTH modes. It used to live only
    # on the torque branch, which let lock mode chase the input-side dead point
    # while wandering into an OUTPUT-side one -- seed 2 came back at 24.2 deg,
    # below the healthy floor, purely because nothing was checking.
    bind = max(0.0, MIN_TRANSMISSION_DEG - min_transmission(lk, T))
    # Stowed width: a crank that pokes out of the envelope while parked has
    # defeated the point of narrowing the bike. Soft, with a steep slope, for
    # the same reason as the floor constraint.
    wide = (0.0 if _MAX_STOW_HALF is None
            else max(0.0, stow_half_width(lk) - _MAX_STOW_HALF))
    # Hinge interference: hard to see in a 2D animation, fatal in metal.
    cross = (0.0 if not _NO_CROSSOVER
             else max(0.0, pivot_crossover(lk, T) + _HINGE_MARGIN))
    if _MODE == "lock":
        # Torque becomes a FEASIBILITY constraint rather than the objective,
        # and what we maximise is the advantage at the deployed pose. Without
        # the torque budget the search would happily buy a spectacular end
        # toggle by making the middle of the stroke unliftable.
        pk = peak_torque(lk)
        over = max(0.0, pk - _TORQUE_BUDGET)
        # Capped: past MA ~30 the pose is locked for any practical purpose and
        # chasing a truer singularity just buys tolerance sensitivity. Leaving
        # it uncapped also gives the search a huge dynamic range to no benefit.
        ma = min(end_advantage(lk, T), 30.0)
        return -ma + 40.0 * over + 2.0 * short + 1.5 * bind + 2.0 * wide + 3.0 * cross
    # 0.02 N.m per degree of transmission shortfall: a design 20 deg into the
    # bad region pays 0.4 N.m, which is the whole torque budget. Enough to
    # keep it out of dead points without forbidding a brief dip.
    return peak_torque(lk) + 0.05 * short + 0.02 * bind + 0.05 * wide + 0.08 * cross


def stow_half_width(lk: "Linkage") -> float:
    """Widest |lateral| the MECHANISM reaches when STOWED [mm].

    Crank tips, coupler line and attach points only — not the wing panel, whose
    lateral position IS the half-span by definition.

    Stowed, not swept, and that distinction is the whole point: the stowed pose
    is the bike's driving envelope, while everything past it happens with the
    bike already on its side, where sticking out costs nothing. Constraining
    the full sweep instead would throw away most of the useful designs.
    """
    worst = 0.0
    for tag in ("right", "left"):
        tip = lk.crank_tip(tag, 0.0)
        att = lk.attach0[tag]
        for f in np.linspace(0.0, 1.0, 21):          # along the coupler too
            q = tip + f * (att - tip)
            worst = max(worst, abs(float(q[0])))
        worst = max(worst, abs(float(tip[0])), abs(float(att[0])))
    return worst


def stow_roof_margin(lk: "Linkage", cfg: dict) -> float:
    """Smallest clearance from the STOWED mechanism to the roof surface [mm].

    Negative means something pokes THROUGH the roof, which defeats the shell:
    an inverted bike is then resting on a crank arm instead of rolling on the
    cylinder. Same points as `stow_half_width`, but measured radially from the
    roof axis rather than laterally, which is the constraint that actually
    matters — the roof is a cylinder, not a slab.
    """
    R = cfg["bike"]["bike_width"] / 2.0
    axis_z = cfg["bike"]["bike_height"] - R          # mm above the floor
    worst = 1e9
    for tag in ("right", "left"):
        tip, att = lk.crank_tip(tag, 0.0), lk.attach0[tag]
        for f in np.linspace(0.0, 1.0, 21):
            q = tip + f * (att - tip)
            # ONLY points above the roof axis. The roof is a cylinder covering
            # the TOP of the bike, so it is the high parts that end up at the
            # bottom when inverted and could touch down before the shell does.
            # Points below the axis are simply outside its angular coverage —
            # the wheels are 105 mm from the axis and that is not a defect.
            if q[1] <= axis_z:
                continue
            worst = min(worst, R - float(np.hypot(q[0], q[1] - axis_z)))
    return worst if worst < 1e8 else float("inf")


def pivot_crossover(lk: "Linkage", travel: float) -> float:
    """How far a wing's attach point reaches PAST the opposite pivot [mm].

    Positive = interference. The wing pivots are the one place that has to
    carry a real hinge — a pin with length along the fore/aft axis, not a
    point — so nothing from the other side may occupy that lateral station.
    An attach point that crosses it means the two sides' hardware wants the
    same space, and no amount of fore/aft staggering fixes a hinge that has to
    be long.

    Checked over the WHOLE stroke, unlike the envelope constraint: sticking out
    of the envelope while deploying is free, but two parts trying to occupy one
    volume is interference whenever it happens.
    """
    p_off = lk.pivot(1)[0]                    # +y pivot station
    worst = -1e9
    lk._last = {}
    t = 0.0
    while t <= travel:
        for tag in ("right", "left"):
            att, _ = lk.solve(tag, t)
            y = float(att[0])
            worst = max(worst, (-p_off - y) if tag == "left" else (y - p_off))
        t += 4.0
    return worst


_WITH_TORQUE = False
_KIN_TOL = 3.0
# Half-width the STOWED mechanism may not exceed [mm]. None = unconstrained,
# which is what every pre-existing config was optimised under.
_MAX_STOW_HALF = None
# Keep each wing's attach point clear of the OPPOSITE pivot, by this margin.
_NO_CROSSOVER = False
_HINGE_MARGIN = 4.0       # mm of hinge boss to leave room for
_MODE = "torque"          # "torque" = minimise peak; "lock" = maximise end MA
_TORQUE_BUDGET = 0.55     # N.m the servo may need anywhere in the stroke


def cmd_optimize(cfg, out: Path, seed: int, iters: int, with_torque: bool = False,
                 mode: str = "torque", max_stow_half: float | None = None,
                 max_crank: float | None = None, no_crossover: bool = False,
                 crank_angle=None, min_pivot: float | None = None):
    from scipy.optimize import differential_evolution

    global _WITH_TORQUE, _MODE, _MAX_STOW_HALF, _NO_CROSSOVER
    _WITH_TORQUE = with_torque or mode == "lock"
    _MODE = mode
    _MAX_STOW_HALF = max_stow_half
    _NO_CROSSOVER = no_crossover
    if no_crossover:
        print(f"attach points kept {_HINGE_MARGIN:.0f} mm clear of the opposite pivot")
    if max_stow_half is not None:
        print(f"stowed mechanism constrained to +/-{max_stow_half:.1f} mm")

    lk0 = Linkage(cfg)
    T0, r0, l0, e0 = best_pose(lk0)
    print(f"start:  best pose at {T0:5.0f} deg servo -> right {r0:6.1f}  "
          f"left {l0:6.1f}   worst error {e0:6.1f}"
          + (f"   peak servo torque {peak_torque(lk0):.3f} N.m" if with_torque else ""))
    def _bound(name, lo, hi):
        if max_crank and name.startswith("link1"):
            hi = min(hi, max_crank)
        if crank_angle and name == "first_link_angle_deg":
            lo, hi = max(lo, crank_angle[0]), min(hi, crank_angle[1])
        if min_pivot and name == "wing_pivot_offset":
            lo = max(lo, min_pivot)
        return (lo, hi)
    bounds = [_bound(name, lo, hi) for name, _p, lo, hi in _VARS]
    if max_crank:
        print(f"crank arms capped at {max_crank:.1f} mm")
    if crank_angle:
        # Keeps the search in the ARMS-UP-AND-OUT basin. Left free, it drifts
        # to ~277 deg — cranks pointing down and inward — which collapses the
        # whole mechanism onto the centreline and leaves no room for pins.
        print(f"crank angle held to {crank_angle[0]:.0f}..{crank_angle[1]:.0f} deg")
    if min_pivot:
        print(f"wing pivots at least +/-{min_pivot:.1f} mm apart")
    res = differential_evolution(
        _objective, bounds, args=(cfg,),
        seed=seed, maxiter=iters, tol=1e-6, polish=True, disp=False)
    best = _apply(cfg, res.x)
    lk = Linkage(best)
    T, r, l, e = best_pose(lk)
    Tw, _, _ = window(lk)
    print(f"best:   best pose at {T:5.0f} deg servo -> right {r:6.1f}  "
          f"left {l:6.1f}   worst error {e:6.1f}   (target "
          f"{TARGET_WING_DEG:.0f} on BOTH)\n")
    for (name, _p, lo, hi), v in zip(_VARS, res.x):
        edge = "  <-- AT BOUND" if min(abs(v-lo), abs(v-hi)) < 1e-6 else ""
        print(f"  {name:26s} {v:8.2f}   [{lo:.0f}, {hi:.0f}]{edge}")
    print(f"\n  wing_length (derived)      {lk.wing_length:8.2f}")
    for tag in ("right", "left"):
        print(f"  link2_{tag:<20s} {lk.coupler[tag]:8.2f}   (derived)")
    print(f"\n  servo travel to that pose  {T:8.2f} deg  "
          f"({T/360:.2f} turn -- single-turn limit is 1.00)")
    print(f"  monotonic window ends at   {Tw:8.2f} deg  "
          f"({'pose is inside it' if T <= Tw + 1e-9 else 'POSE IS OUTSIDE IT'})")
    pk = peak_torque(lk)
    print(f"  peak servo torque          {pk:8.3f} N.m  "
          f"({pk/(0.80*9.9/12):.2f} of the 9.9 V stall; geared 2:1 needs 0.339)")
    print(f"  stowed mechanism half-width {stow_half_width(lk):7.2f} mm  "
          f"(envelope +/-{cfg['bike']['bike_width']/2:.1f})")
    xo = pivot_crossover(lk, T)
    print(f"  attach vs opposite pivot  {-xo:9.2f} mm clearance  "
          f"({'ok' if xo < 0 else 'CROSSES THE HINGE'})")
    rm = stow_roof_margin(lk, best)
    print(f"  stowed clearance to roof    {rm:7.2f} mm  "
          f"({'inside the shell' if rm > 0 else 'POKES THROUGH THE ROOF'})")
    fc = floor_clearance(lk, T)
    print(f"  floor clearance            {fc:8.2f} mm  "
          f"({'ok' if fc >= MIN_FLOOR_MM else 'BELOW THE ' + str(MIN_FLOOR_MM) + ' mm MINIMUM'})")
    mu = min_transmission(lk, T)
    print(f"  worst transmission angle   {mu:8.2f} deg "
          f"({'ok' if mu >= MIN_TRANSMISSION_DEG else 'near a dead point'})")
    ma = end_advantage(lk, T)
    print(f"  MA at full deployment      {ma:8.2f}     "
          f"({'SELF-LOCKING' if ma > 5 else 'servo must hold it'})"
          f"  -> holds {ma * 0.66:.2f} N.m of wing load at stall")
    if out:
        out.write_text(yaml.safe_dump(best, sort_keys=False))
        print(f"\nwrote {out}")
    return lk


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=CONFIG)
    ap.add_argument("--deploy", action="store_true",
                    help="stowed -> deployed sweep instead of the labelled stow")
    ap.add_argument("--righting", action="store_true",
                    help="the bike pushing itself up, both sides, with vectors")
    ap.add_argument("--forces", action="store_true",
                    help="peak pin loads over the stroke (summary, no plot)")
    ap.add_argument("--video", action="store_true",
                    help="animate deploy + retract (2D, self-contained)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--stick", action="store_true",
                    help="overlay the proposed FIXED side panels and report "
                         "how high they may reach before the deploying wing "
                         "sweeps into them")
    ap.add_argument("--panels", action="store_true",
                    help="stowed and deployed side by side, with a turn marker")
    ap.add_argument("--torque", action="store_true",
                    help="servo torque through the stroke, both fall sides")
    ap.add_argument("--lock", action="store_true",
                    help="optimise for a SELF-LOCKING deployed pose (max MA at "
                         "the end) with torque as a budget instead")
    ap.add_argument("--crank-angle", type=float, nargs=2, default=None,
                    metavar=("LO", "HI"),
                    help="bound first_link_angle_deg, to hold the search in the "
                         "arms-up basin instead of the arms-down one")
    ap.add_argument("--min-pivot", type=float, default=None,
                    help="minimum wing_pivot_offset [mm], so the two hinges "
                         "have room for real pins")
    ap.add_argument("--no-crossover", action="store_true",
                    help="forbid a wing's attach point from crossing the "
                         "opposite pivot, so each pivot can carry a real hinge")
    ap.add_argument("--max-crank", type=float, default=None,
                    help="upper bound on both crank arm lengths [mm]")
    ap.add_argument("--fit-envelope", action="store_true",
                    help="keep the STOWED mechanism inside bike_width/2. Off by "
                         "default so the committed configs stay reproducible")
    ap.add_argument("--optimize", action="store_true",
                    help="search link geometry for both wings reaching "
                         f"{TARGET_WING_DEG:.0f} deg inside one monotonic stroke")
    ap.add_argument("--save", type=Path, default=None,
                    help="write the optimised config here")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--iters", type=int, default=400)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--tag", default="",
                    help="suffix for the default output filenames, to keep one "
                         "config's figures apart from another's. The committed "
                         "set in analysis/plots/ is the untagged --config "
                         "wing_linkage.yaml run, plus `--tag _opt` and "
                         "`--tag _lock` for the two --save'd variants")
    a = ap.parse_args()
    cfg = yaml.safe_load(a.config.read_text())

    def default_out(stem: str, ext: str) -> Path:
        """Where a figure lands when --out is not given.

        Every tracked figure has to be reproducible AT THE NAME IT IS TRACKED
        UNDER, or the analysis/ convention quietly stops holding: the _opt and
        _lock variants were originally one-off `--out` runs, so nothing in the
        repo said which config produced them or how to redraw one.
        """
        return a.out or _plots_dir() / f"{stem}{a.tag}.{ext}"

    if a.optimize:
        cmd_optimize(cfg, a.save, a.seed, a.iters, with_torque=a.torque,
                     mode="lock" if a.lock else "torque",
                     max_stow_half=(cfg["bike"]["bike_width"] / 2
                                    if a.fit_envelope else None),
                     max_crank=a.max_crank, no_crossover=a.no_crossover,
                     crank_angle=a.crank_angle, min_pivot=a.min_pivot)
        return
    if a.righting:
        if a.video:
            cmd_righting_video(cfg, default_out("wing_linkage_righting", "mp4"),
                               a.fps, a.seconds)
        else:
            cmd_righting(cfg, default_out("wing_linkage_righting", "png"))
        return
    if a.forces:
        cmd_forces(cfg, a.out)
        return
    if a.video:
        cmd_video(cfg, default_out("wing_linkage", "mp4"),
                  a.fps, a.seconds, stick=a.stick)
        return
    if a.panels:
        cmd_panels(cfg, default_out("wing_linkage_panels", "png"))
        return
    if a.torque:
        cmd_torque(cfg, default_out("wing_linkage_torque", "png"))
        return
    out = default_out("wing_linkage_deploy" if a.deploy
                      else "wing_linkage_stowed", "png")
    lk = cmd_deploy(cfg, out) if a.deploy else cmd_stowed(cfg, out)

    print(f"\n  wing length (derived)      {lk.wing_length:.1f} mm "
          f"(ground_clearance {lk.wing_bottom:.0f} -> bike_height {lk.wing_top:.1f})")
    print(f"  wing outer face at |y| =   {lk.half_span:.0f} mm "
          f"(pivot {lk.pivot_y:.0f} + stow offset {lk.stow_offset:.0f})")
    for tag in ("right", "left"):
        print(f"  {tag:5s}: link1 {lk.crank[tag]:5.1f}  link2 {lk.coupler[tag]:6.2f} "
              f"(derived)  crank at stow {lk.crank_angle0[tag]:6.1f}°")
    d = abs(lk.coupler['right'] - lk.coupler['left'])
    print(f"  coupler asymmetry          {d:.2f} mm "
          f"({'expected — the two cranks start 180° apart' if d else 'symmetric'})")


if __name__ == "__main__":
    main()
