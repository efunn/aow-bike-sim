"""The drive-servo mount: what the packing leaves free, and a two-plate cage.

    python analysis/servo_mount.py                 # -> analysis/plots/servo_mount_cage.png
    python analysis/servo_mount.py --tag alt       # a variant, at its own name

A PROPOSAL, not a decision. Everything is read from `aow_sim.cad_layout`, so
the figure cannot drift from the layout the CAD is drawn against; if a servo
angle or a pulley diameter moves, re-run this and the drawing moves with it.
Changes nothing — see the header of `analysis/wing_linkage.py` for the rule.

THE PROBLEM. Both drive servos face their horns outboard, because the belt
plane is fixed by the input pulley and the servo pulley has to lie in it. That
puts each case INBOARD of its own belt, and the two cases overlap across the
centreline. Of the six faces on an XC430:

    horn face      under its own 75.6 mm pulley                    dead
    back face      under the OTHER servo's pulley                  dead
    inner side     ~2 mm from the neighbouring case                dead
    outer side     34 x 46.5, 4 x M2 + 4 slots at 12 x 24          FREE
    shaft end      28.5 x 34, 2 x M2                               FREE
    far end        28.5 x 34, 4 x M2 at 12 x 16                    FREE

The back face is the one worth arguing about, because it carries the P.C.D 16
pattern the ROBOTIS frames use. Its four taps sit 8 mm off the axis and the
neighbouring shaft is only ~30.5 mm away, so they land between 22.5 and 38.5 mm
from that neighbour's pulley AXIS — against a pulley of radius 37.8. At best
one tap clears, by well under a millimetre, and only for one clocking of the
pattern. That is not a mount.

THE PROPOSAL. Both cases share `body_up`, so their end faces are coplanar: one
plate across both far ends and one across both shaft ends makes a cage that
takes the belt tension along the 46.5 mm dimension. Both plates live inside the
belt planes, which is the clearance that actually has to hold.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon, Rectangle

from aow_sim.cad_layout import CAD_PARAMS, LINKAGE_CFG, build, load_params, \
    load_sources, to_cad_dir, to_cad_pos

PLATE_GAP = 1.0     # mm of air between a plate and the pulley face beside it


def _plots_dir():
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(exist_ok=True)
    return d


def spine_plane(params, plate_mm):
    """Belt plane that puts the two back faces either side of a centre plate.

    The left case runs from its horn-face datum inward, so its back face lands
    at (plane - pw/2 - horn - D) off the centreline. Setting that to half the
    plate thickness is the whole condition — everything else about the mount
    follows from the two servos no longer sharing any of the centreline.
    """
    dt, sv = params["drivetrain"], params["servos"]["xc430_w150"]
    be = dt["belt"]
    pw = (be["width"] + 2 * be["flange_thickness"]) * 1000.0
    return (pw / 2 + sv["horn_thickness"] * 1000.0
            + sv["box_size"][0] * 1000.0 + plate_mm / 2)


def pad_plane(params, plate_mm):
    """Belt plane for a back-face PAD — one per servo, not one shared plate.

    THE CHEAP VERSION, and the one to reach for first. A pad on the left
    servo's back face has to clear the RIGHT PULLEY, which is what actually
    shadows it; it does NOT have to clear the right servo's case, because the
    two are separated tangentially and a pad covering only the P.C.D 16 stays
    well inside that separation. So the condition is

        (41.5 - plane) + t  <=  plane - pw/2

    and every millimetre of widening opens TWO of gap, one from each side.
    Contrast `spine_plane`, which insists the two cases stop overlapping at
    all and costs an order of magnitude more width for it.
    """
    dt, sv = params["drivetrain"], params["servos"]["xc430_w150"]
    be = dt["belt"]
    pw = (be["width"] + 2 * be["flange_thickness"]) * 1000.0
    return (pw + sv["horn_thickness"] * 1000.0
            + sv["box_size"][0] * 1000.0 + plate_mm) / 2


def sleeve_separation(params, C_mm):
    """Straddle angle with the two cases TOUCHING — the sleeve holds them.

    The separation exists only to keep the cases apart; a sleeve that grips
    both from outside makes the gap unnecessary, so this is the floor.
    """
    W = params["servos"]["xc430_w150"]["box_size"][1] * 1000.0
    return 2.0 * np.degrees(np.arcsin((W / 2) / C_mm))


def collect(params_path=CAD_PARAMS, linkage_cfg=LINKAGE_CFG, spine=None,
            sleeve=None):
    """Everything the drawing needs, in CAD mm, keyed by name."""
    params = load_params(params_path)
    if spine is not None:
        # WIDEN, then drop the straddle. The tangential separation exists only
        # because the two cases overlap across the centreline; once they do not,
        # both servos can sit at exactly the mean angle and the machine goes
        # properly mirror-symmetric.
        ow, dt = params["omni_wheel"], params["drivetrain"]
        be = dt["belt"]
        pw = (be["width"] + 2 * be["flange_thickness"])
        dt["wheel_clearance"] = (spine_plane(params, spine) / 1000.0
                                 - ow["width"] / 2 - pw / 2)
        dt["drive_servo_separation_deg"] = 0.0
    if sleeve is not None:
        # C is set by the belt and is not ours to choose, so read it off a
        # default build rather than re-deriving the belt equation here.
        base = build(load_params(params_path), load_sources(params_path),
                     "linkage", False, False, linkage_cfg)
        pul = next(i for i in base if i["name"] == "pulley_servo_left")
        c = to_cad_pos(pul["pos"])
        params["drivetrain"]["drive_servo_separation_deg"] = sleeve_separation(
            params, float(np.hypot(c[1], c[2])))
    items = build(params, load_sources(params_path), "linkage", False, False,
                  linkage_cfg)
    out = {}
    for it in items:
        d = {"pos": np.array(to_cad_pos(it["pos"]), float)}
        if "box" in it:
            # Extents are servo-local (W, D, H) whenever a frame is attached.
            d["box"] = np.array(it["box"], float) * 1000.0
            if "frame" in it:
                d["axes"] = np.array([to_cad_dir(a) for a in it["frame"]],
                                     float)
        if "cyl" in it:
            r, ln, ax = it["cyl"]
            d.update(r=r * 1000.0, ln=ln * 1000.0,
                     ax=np.array(to_cad_dir(ax), float))
        if "holes" in it:
            d["holes"] = np.array([to_cad_pos(q) for q in it["holes"]], float)
        out[it["name"]] = d
    return params, out


def cage(g):
    """The two plate positions, as radial coordinates along the mean radius.

    Derived rather than chosen: the plates sit ON the two end faces, so their
    radii are the case centre's radial coordinate plus or minus half the case
    height. The mean radius is the shared `body_up`, which is why one flat
    plate can reach both servos at once.
    """
    left = g["servo_drive_left"]
    u = left["axes"][2]                       # body_up, shared by both servos
    half_h = left["box"][2] / 2
    r_case = float(left["pos"] @ u)
    return u, r_case - half_h, r_case + half_h


def belt_tangents(c1, r1, c2, r2):
    """The two outer tangent lines of an open belt, in the (y, z) plane.

    Schematic: it uses the drawn pulley radii rather than the pitch circles, so
    the lines sit a flange's thickness proud of where the belt really runs.
    They are here to show WHERE the belt sweeps, which is what a plate has to
    miss, not to size anything.
    """
    d = c2 - c1
    dist = float(np.linalg.norm(d))
    a = np.arctan2(d[1], d[0]) 
    b = np.arcsin(np.clip((r2 - r1) / dist, -1, 1))
    out = []
    for s_ in (1.0, -1.0):
        th = a + s_ * (np.pi / 2 + b)
        u_ = np.array([np.cos(th), np.sin(th)])
        out.append((c1 + r1 * u_, c2 + r2 * u_))
    return out


def draw(g, params, ax1, ax2, spine=None):
    u, r_in, r_out = cage(g)
    v = np.cross(u, np.array([1.0, 0.0, 0.0]))   # tangential, in the belt plane

    def yz(p):
        return (p[1], p[2])

    # ---- panel 1: the belt plane, looking along the axle ------------------
    ax1.add_patch(Circle((0, 0), g["omni_wheel_rear"]["r"], fc="0.92",
                         ec="0.65", zorder=0))
    r_inp = g["pulley_input_left"]["r"]
    ax1.add_patch(Circle((0, 0), r_inp, fc="0.78", ec="0.55", zorder=1))
    for tag, col in (("left", "#1f77b4"), ("right", "#d62728")):
        pul = g[f"pulley_servo_{tag}"]
        pc = np.array(yz(pul["pos"]))
        ax1.add_patch(Circle(pc, pul["r"], fc="none", ec=col, ls=":", lw=1.1,
                             zorder=2))
        for a, b in belt_tangents(np.zeros(2), r_inp, pc, pul["r"]):
            ax1.plot([a[0], b[0]], [a[1], b[1]], "-", color=col, lw=0.8,
                     alpha=0.45, zorder=2)

        # box is (D, W, H) and axes are (w, d, h) — D runs along the SHAFT, so
        # what shows in this plane is W across by H along the radius.
        sv = g[f"servo_drive_{tag}"]
        w_hat, _, h_hat = sv["axes"]
        hw, hh = sv["box"][1] / 2, sv["box"][2] / 2
        corners = [sv["pos"] + sw * hw * w_hat + sh * hh * h_hat
                   for sw, sh in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        ax1.add_patch(Polygon([yz(c) for c in corners], closed=True, fc=col,
                              alpha=0.22, ec=col, lw=1.4, zorder=3))
        ax1.plot(*yz(g[f"servo_drive_{tag}_shaft"]["pos"]), "o", color=col,
                 ms=4.5, zorder=6)
        for q in g[f"servo_drive_{tag}_case_holes_back"]["holes"]:
            ax1.plot(*yz(q), "x", color=col, ms=5, mew=1.2, zorder=6)

    # The plates, edge-on: a line perpendicular to u at each end-face radius.
    # In spine mode there is no cage — the mount is a single plate ON the
    # centreline, which this view sees end-on and cannot show, so the end-face
    # lines are drawn faint as the alternative they replace.
    span = 34.0
    for r, lbl, va in ((r_in, f"inner plate   r = {r_in:.0f}", "top"),
                       (r_out, f"outer plate   r = {r_out:.0f}", "bottom")):
        a, b = r * u - span * v, r * u + span * v
        ax1.plot([a[1], b[1]], [a[2], b[2]], color="#2ca02c",
                 lw=1.5 if spine else 4.0, alpha=0.35 if spine else 1.0,
                 ls=":" if spine else "-", solid_capstyle="butt", zorder=7)
        ax1.annotate(lbl, yz(r * u - (span + 3) * v),
                     color="#2ca02c", alpha=0.4 if spine else 1.0,
                     fontsize=8, ha="right", va=va, zorder=8)
    ax1.plot([0, 128 * u[1]], [0, 128 * u[2]], "--", color="0.55", lw=0.8,
             zorder=1)
    ax1.annotate("mean radius\n(shared body_up)", yz(30 * u), fontsize=7,
                 color="0.45", ha="left", va="top")
    ax1.annotate("rear wheel", (-38, -38), fontsize=7, color="0.35",
                 ha="center", va="center")
    ax1.annotate("input pulley", (-r_inp - 3, 0), fontsize=7, color="0.35",
                 ha="right", va="center")
    ax1.annotate("thin lines = belt run (schematic: drawn radii, not pitch)",
                 (0.02, 0.975), xycoords="axes fraction", fontsize=6.5,
                 color="0.45", va="top")

    ax1.set_aspect("equal")
    ax1.set_xlim(-70, 165)
    ax1.set_ylim(-70, 160)
    ax1.set_xlabel("CAD +Y, forward  [mm]")
    ax1.set_ylabel("CAD +Z, up  [mm]")
    ax1.set_title("Belt plane — looking along the rear axle\n" +
                  ("dotted = servo pulley,  x = back-face taps, now CLEAR;  "
                   "the two servos coincide in this view"
                   if spine else
                   "dotted = servo pulleys,  x = back-face taps (all blocked)"),
                  fontsize=9)
    ax1.grid(alpha=0.25, lw=0.5)

    # ---- panel 2: the width budget ---------------------------------------
    plane = abs(g["pulley_servo_left"]["pos"][0])
    bw = params["drivetrain"]["belt"]["width"] * 1000.0
    pw = g["pulley_servo_left"]["ln"]
    sv, horn = g["servo_drive_left"], g["servo_drive_left_horn"]
    boss = g["servo_drive_left_horn_boss"]
    pulley_face = plane - pw / 2               # 18.5 — the binding surface
    plate_edge = pulley_face - PLATE_GAP

    def band(centre, length):
        return centre - length / 2, centre + length / 2

    case = band(sv["pos"][0], sv["box"][0])    # D runs along the shaft axis
    bars = [
        ("rear wheel", *band(0, g["omni_wheel_rear"]["ln"]), "0.85", False),
        ("belt", -(plane + bw / 2), -(plane - bw / 2), "#8c564b", True),
        ("servo pulley", -(plane + pw / 2), -pulley_face, "#9467bd", True),
        ("horn boss", *band(boss["pos"][0], boss["ln"]), "#ff7f0e", True),
        ("horn disc", *band(horn["pos"][0], horn["ln"]), "#ff7f0e", True),
        ("servo case (L)", *case, "#1f77b4", False),
        ("servo case (R)", -case[1], -case[0], "#d62728", False),
        ("PROPOSED spine" if spine else "PROPOSED plates",
         -spine / 2 if spine else -plate_edge,
         spine / 2 if spine else plate_edge, "#2ca02c", False),
    ]
    for i, (lbl, lo, hi, col, mirror) in enumerate(bars):
        y = len(bars) - i
        ax2.add_patch(Rectangle((lo, y - 0.32), hi - lo, 0.64, fc=col,
                                alpha=0.8, ec="0.3", lw=0.6))
        ax2.annotate(f"{lo:+.1f} .. {hi:+.1f}", ((lo + hi) / 2, y + 0.02),
                     ha="center", va="center", fontsize=6.5, color="0.1")
        if mirror:
            ax2.add_patch(Rectangle((-hi, y - 0.32), hi - lo, 0.64, fc=col,
                                    alpha=0.30, ec="0.5", lw=0.5, ls=":"))
    ax2.set_yticks([len(bars) - i for i in range(len(bars))])
    ax2.set_yticklabels([b[0] for b in bars], fontsize=8)
    for x in (-plate_edge, plate_edge):
        ax2.axvline(x, color="#2ca02c", ls="--", lw=0.9)
    ax2.axvline(0, color="0.6", lw=0.8)
    half = plane + pw / 2 + g["axle_mount_left"]["ln"]
    ax2.annotate(f"rear half-width {half:.1f} mm  ->  {2 * half:.0f} mm overall",
                 (0.5, 0.015), xycoords="axes fraction", ha="center",
                 fontsize=7.5, color="0.3")
    ax2.set_xlim(-36, 36) if not spine else ax2.set_xlim(-62, 62)
    ax2.set_ylim(0.3, len(bars) + 0.8)
    ax2.set_xlabel("CAD X, across the bike  [mm]")
    ax2.set_title(f"Width budget at the servo station\n"
                  f"the pulley face at {pulley_face:.1f} is the binding "
                  f"surface, not the belt at {plane - bw / 2:.1f}", fontsize=9)
    ax2.grid(axis="x", alpha=0.25, lw=0.5)
    return r_in, r_out, plate_edge


def draw_sleeve(g, params, ax1, ax2, wall):
    """The sleeve: one closed tube round BOTH cases, gripping them by shape.

    The point of it is that the reaction torque goes into the tube walls in
    bearing, so the fasteners only have to stop the servo drifting along its
    own shaft — which is what makes it the kind option to a servo you want to
    reuse, since every alternative puts M2 self-tappers in shear.
    """
    u, _, _ = cage(g)
    v = np.cross(u, np.array([1.0, 0.0, 0.0]))
    sv = g["servo_drive_left"]
    W, D, H = sv["box"][1], sv["box"][0], sv["box"][2]

    def uv(p):
        return np.array([float(p @ u), float(p @ v)])

    def yz(p):
        return (p[1], p[2])

    # ---- panel 1: the belt plane ------------------------------------------
    ax1.add_patch(Circle((0, 0), g["omni_wheel_rear"]["r"], fc="0.92",
                         ec="0.65", zorder=0))
    ax1.add_patch(Circle((0, 0), g["pulley_input_left"]["r"], fc="0.78",
                         ec="0.55", zorder=1))
    for tag, col in (("left", "#1f77b4"), ("right", "#d62728")):
        pul = g[f"pulley_servo_{tag}"]
        ax1.add_patch(Circle(yz(pul["pos"]), pul["r"], fc="none", ec=col,
                             ls=":", lw=1.0, zorder=2))
        c = g[f"servo_drive_{tag}"]
        w_hat, _, h_hat = c["axes"]
        pts = [c["pos"] + sw * W / 2 * w_hat + sh * H / 2 * h_hat
               for sw, sh in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        ax1.add_patch(Polygon([yz(q) for q in pts], closed=True, fc=col,
                              alpha=0.25, ec=col, lw=1.3, zorder=3))
        ax1.plot(*yz(g[f"servo_drive_{tag}_shaft"]["pos"]), "o", color=col,
                 ms=4.5, zorder=5)

    # The tube, seen end-on: it wraps both cases with one wall thickness.
    ctr = uv(sv["pos"])[0] * u                      # radial coord, on the axis
    for half, style in ((W + wall, dict(fc="#2ca02c", alpha=0.18,
                                        ec="#2ca02c", lw=1.8)),):
        pts = [ctr + sv * half * v + sh * (H / 2 + wall) * u
               for sv_, sh in ((1, 1), (1, -1), (-1, -1), (-1, 1))
               for sv in (sv_,)]
        ax1.add_patch(Polygon([yz(q) for q in pts], closed=True, zorder=2,
                              **style))
    ax1.annotate(f"sleeve, {wall:.0f} mm wall\n"
                 f"{2 * W:.0f} x {H:.0f} cavity",
                 yz(ctr + (W + wall + 4) * v), color="#2ca02c", fontsize=8,
                 ha="left", va="center")
    ax1.set_aspect("equal")
    ax1.set_xlim(-70, 165)
    ax1.set_ylim(-70, 160)
    ax1.set_xlabel("CAD +Y, forward  [mm]")
    ax1.set_ylabel("CAD +Z, up  [mm]")
    ax1.set_title("Belt plane — the two cases now TOUCH\n"
                  f"straddle drops to "
                  f"{params['drivetrain']['drive_servo_separation_deg']:.2f} deg "
                  f"(gap 0), from 16.35", fontsize=9)
    ax1.grid(alpha=0.25, lw=0.5)

    # ---- panel 2: the axial section, x across, tangential up --------------
    tan = {t: uv(g[f"servo_drive_{t}"]["pos"])[1] for t in ("left", "right")}
    lo_x = sv["pos"][0] - D / 2
    hi_x = sv["pos"][0] + D / 2
    cav = (-max(abs(lo_x), abs(hi_x)), max(abs(lo_x), abs(hi_x)))

    def rect(ax, x0, x1, y0, y1, **kw):
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, **kw))

    plane = abs(g["pulley_servo_left"]["pos"][0])
    pw = g["pulley_servo_left"]["ln"]
    pul_r = g["pulley_servo_left"]["r"]
    face = plane - pw / 2               # the opposite pulley's inner face

    # THE END STOP CANNOT BE A FULL WALL. Past a servo's inboard face there is
    # only (face - cav) of air before the OTHER servo's pulley — the same 1 mm
    # that killed the back-face mount. A stop is only free where it is further
    # than the pulley radius from that pulley's axis, which is the outer strip
    # of each servo's tangential band. So: side walls the full length, and a
    # narrow tab folded in at each end, out at the tangential edge.
    # A point at tangential t is (t - tan_right) from the right pulley's axis,
    # so it escapes that pulley's disc at t = pul_r + tan_right — about 5 mm
    # short of the band's outer edge, which is all the tab gets.
    clear = pul_r + tan["right"]
    body = min(cav[1] + wall, face)     # walls stop at the pulley face
    for t0, sgn in ((tan["left"], 1.0), (tan["right"], -1.0)):
        rect(ax2, -body, body, t0 + sgn * W / 2, t0 + sgn * (W / 2 + wall),
             fc="#2ca02c", alpha=0.35, ec="#2ca02c")
    rect(ax2, cav[1], cav[1] + wall, clear, tan["left"] + W / 2,
         fc="#2ca02c", alpha=0.75, ec="#2ca02c")
    rect(ax2, cav[0] - wall, cav[0], tan["right"] - W / 2, -clear,
         fc="#2ca02c", alpha=0.75, ec="#2ca02c")
    ax2.annotate(f"end tab, {tan['left'] + W / 2 - clear:.1f} mm wide —\n"
                 f"the left servo butts on it.\nAny wider and it fouls the "
                 f"RIGHT pulley:\nonly {face - cav[1]:.0f} mm of air in there",
                 (cav[1] + wall + 2, clear - 1), fontsize=7, color="#2ca02c",
                 ha="left", va="top")
    ax2.annotate("end tab —\nright servo", (cav[0] - wall - 2, -clear + 1),
                 fontsize=7, color="#2ca02c", ha="right", va="bottom")
    for t0 in (clear, -clear):
        ax2.plot([-52, 52], [t0, t0], ls="--", lw=0.7, color="#9467bd")
    ax2.annotate(f"beyond here, clear of the opposite pulley (r {pul_r:.1f})",
                 (-51, clear + 0.8), fontsize=6.5, color="#9467bd", ha="left",
                 va="bottom")
    for tag, col, sgn in (("left", "#1f77b4", -1.0), ("right", "#d62728", 1.0)):
        c = g[f"servo_drive_{tag}"]
        t0 = tan[tag]
        rect(ax2, c["pos"][0] - D / 2, c["pos"][0] + D / 2, t0 - W / 2,
             t0 + W / 2, fc=col, alpha=0.3, ec=col, lw=1.2)
        ax2.annotate(f"{tag} servo\n{D:.0f} long", (c["pos"][0], t0),
                     ha="center", va="center", fontsize=7.5, color=col)
        hn = g[f"servo_drive_{tag}_horn"]
        rect(ax2, hn["pos"][0] - hn["ln"] / 2, hn["pos"][0] + hn["ln"] / 2,
             t0 - hn["r"], t0 + hn["r"], fc="#ff7f0e", alpha=0.7, ec="0.3",
             lw=0.5)
        pul = g[f"pulley_servo_{tag}"]
        rect(ax2, sgn * (plane - pw / 2), sgn * (plane + pw / 2),
             t0 - pul["r"], t0 + pul["r"], fc="#9467bd", alpha=0.16,
             ec="#9467bd", lw=0.8, ls=":")
    for x, lbl in ((cav[0], f"{cav[0]:+.1f}"), (cav[1], f"{cav[1]:+.1f}")):
        ax2.axvline(x, color="0.4", ls="--", lw=0.8)
    ax2.annotate(f"cavity {cav[1] - cav[0]:.0f} long, cases {D:.0f} — "
                 f"{cav[1] - cav[0] - D:.0f} mm of float each, and the two sit "
                 f"{abs(2 * sv['pos'][0]):.0f} mm staggered\n"
                 f"because each horn faces its own belt. The whole sleeve has "
                 f"to live inside x = +/-{face:.1f}, the pulley faces.",
                 (0, tan["right"] - W / 2 - wall - 8), ha="center", va="top",
                 fontsize=7.5, color="0.25")
    ax2.set_aspect("equal")
    ax2.set_xlim(-52, 52)
    ax2.set_ylim(-62, 42)
    ax2.set_xlabel("CAD X, across the bike  [mm]")
    ax2.set_ylabel("tangential, in the belt plane  [mm]")
    ax2.set_title("Section on the mean radius — how the sleeve captures both\n"
                  "dotted = servo pulleys,  orange = horns", fontsize=9)
    ax2.grid(alpha=0.25, lw=0.5)
    return cav


def draw_plate(g, params, ax1, ax2):
    """Plate and collar as one printed body, and the width chain behind it.

    The plate is sized to the collar's OUTER footprint rather than to the
    cases, so the walls rise off its edge instead of stopping short of it, and
    the whole thing comes off the bed in one direction.
    """
    dt = params["drivetrain"]
    wall = dt["drive_mount_wall"] * 1000.0
    t_pl = dt["drive_mount_plate"] * 1000.0
    gap = dt["drive_mount_gap"] * 1000.0
    u, _, _ = cage(g)
    v = np.cross(u, np.array([1.0, 0.0, 0.0]))
    sv = g["servo_drive_left"]
    W, D, H = sv["box"][1], sv["box"][0], sv["box"][2]

    def uv(p):
        return np.array([float(p @ u), float(p @ v)])

    def yz(p):
        return (p[1], p[2])

    t_half = abs(uv(sv["pos"])[1]) + W / 2
    ctr = uv(sv["pos"])[0] * u

    def frame(a, b, **kw):
        pts = [ctr + sa * a * v + sb * b * u
               for sa, sb in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        ax1.add_patch(Polygon([yz(q) for q in pts], closed=True, **kw))

    # ---- panel 1: looking along the axle, straight at the plate -----------
    for tag, col in (("left", "#1f77b4"), ("right", "#d62728")):
        pul = g[f"pulley_servo_{tag}"]
        ax1.add_patch(Circle(yz(pul["pos"]), pul["r"], fc="none", ec=col,
                             ls=":", lw=0.9, zorder=1))
    frame(t_half + wall, H / 2 + wall, fc="#2ca02c", alpha=0.30,
          ec="#2ca02c", lw=1.6, zorder=2)
    frame(t_half, H / 2, fc="white", alpha=1.0, ec="#2ca02c", lw=1.2,
          ls="--", zorder=3)
    for tag, col in (("left", "#1f77b4"), ("right", "#d62728")):
        c = g[f"servo_drive_{tag}"]
        w_hat, _, h_hat = c["axes"]
        pts = [c["pos"] + sa * W / 2 * w_hat + sb * H / 2 * h_hat
               for sa, sb in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        ax1.add_patch(Polygon([yz(q) for q in pts], closed=True, fc=col,
                              alpha=0.15, ec=col, lw=1.0, zorder=4))
    rel = g["drive_mount_relief"]
    ax1.add_patch(Circle(yz(rel["pos"]), rel["r"], fc="white", ec="#d62728",
                         lw=1.6, zorder=5))
    ax1.annotate(f"horn relief\nPhi {2 * rel['r']:.1f}", yz(rel["pos"]),
                 ha="center", va="center", fontsize=7, color="#d62728",
                 zorder=6)
    for key, col in (("servo_drive_right_case_holes_horn", "#d62728"),
                     ("servo_drive_left_case_holes_back", "#1f77b4")):
        for q in g[key]["holes"]:
            ax1.plot(*yz(q), "o", mfc="none", mec=col, ms=6, mew=1.4, zorder=7)
    ax1.annotate("8 x M2.5 on the 22 x 40 case patterns —\n"
                 "one servo's horn-side face, the other's back",
                 (0.03, 0.03), xycoords="axes fraction", fontsize=7.5,
                 color="0.25")
    ax1.set_aspect("equal")
    # Bound it by the plate's four corners, not by two opposite ones: the plate
    # is rotated 45 deg in this view, so a diagonal pair bounds nothing.
    corners = np.array([yz(ctr + sa * (t_half + wall) * v
                           + sb * (H / 2 + wall) * u)
                        for sa in (1, -1) for sb in (1, -1)])
    pad = 14.0
    ax1.set_xlim(corners[:, 0].min() - pad, corners[:, 0].max() + pad)
    ax1.set_ylim(corners[:, 1].min() - pad, corners[:, 1].max() + pad)
    ax1.set_xlabel("CAD +Y, forward  [mm]")
    ax1.set_ylabel("CAD +Z, up  [mm]")
    ax1.set_title(f"Plate and collar, seen along the axle\n"
                  f"plate {2 * (t_half + wall):.1f} x {H + 2 * wall:.1f}, "
                  f"cavity {2 * t_half:.1f} x {H:.1f}, wall {wall:.0f}",
                  fontsize=9)
    ax1.grid(alpha=0.2, lw=0.5)

    # ---- panel 2: the width chain ----------------------------------------
    plane = abs(g["pulley_servo_left"]["pos"][0])
    pw = g["pulley_servo_left"]["ln"]
    face = D / 2                      # case face, and where the plate starts
    tan = {t: uv(g[f"servo_drive_{t}"]["pos"])[1] for t in ("left", "right")}

    def rect(x0, x1, y0, y1, **kw):
        ax2.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, **kw))

    for tag, col, sgn in (("left", "#1f77b4", -1.0), ("right", "#d62728", 1.0)):
        t0 = tan[tag]
        rect(-face, face, t0 - W / 2, t0 + W / 2, fc=col, alpha=0.25, ec=col)
        ax2.annotate(f"{tag}", (0, t0), ha="center", va="center", fontsize=8,
                     color=col)
        hn = g[f"servo_drive_{tag}_horn"]
        rect(hn["pos"][0] - hn["ln"] / 2, hn["pos"][0] + hn["ln"] / 2,
             t0 - hn["r"], t0 + hn["r"], fc="#ff7f0e", alpha=0.75, ec="0.3",
             lw=0.5)
        rect(sgn * (plane - pw / 2), sgn * (plane + pw / 2),
             t0 - g[f"pulley_servo_{tag}"]["r"], t0 + g[f"pulley_servo_{tag}"]["r"],
             fc="#9467bd", alpha=0.14, ec="#9467bd", lw=0.8, ls=":")
    lim = t_half + wall
    rect(face, face + t_pl, -lim, lim, fc="#2ca02c", alpha=0.55, ec="#2ca02c")
    for t0, sgn in ((lim - wall / 2, 1), (-lim + wall / 2, -1)):
        rect(-face, face, t0 - wall / 2, t0 + wall / 2, fc="#2ca02c",
             alpha=0.55, ec="#2ca02c")
    rect(face + t_pl, face + t_pl + gap, -lim, lim, fc="none", ec="0.4",
         ls=":", lw=1.0)
    for i, (x, lbl, col) in enumerate((
            (face, f"case face {face:.1f}", "0.3"),
            (face + t_pl, f"plate {t_pl:.0f} mm", "#2ca02c"),
            (face + t_pl + gap, f"gap {gap:.0f} mm", "0.3"),
            (plane - pw / 2, f"pulley face {plane - pw / 2:.1f}", "#9467bd"))):
        ax2.axvline(x, color=col, ls="--", lw=0.8)
        ax2.annotate(lbl, (x + 1.5, lim + 3 + 7 * i), fontsize=7, color=col,
                     ha="left", va="bottom",
                     arrowprops=dict(arrowstyle="-", color=col, lw=0.6,
                                     shrinkA=0, shrinkB=0),
                     xytext=(x + 12, lim + 3 + 7 * i))
    ax2.set_aspect("equal")
    ax2.set_xlim(-46, 46)
    ax2.set_ylim(-lim - 8, lim + 38)
    ax2.set_xlabel("CAD X, across the bike  [mm]")
    ax2.set_ylabel("tangential  [mm]")
    ax2.set_title(f"The width chain — servos do NOT move\n"
                  f"17.0 case + {t_pl:.0f} plate + {gap:.0f} gap = pulley face "
                  f"at {plane - pw / 2:.1f}", fontsize=9)
    ax2.grid(alpha=0.2, lw=0.5)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=CAD_PARAMS)
    ap.add_argument("--linkage-config", default=LINKAGE_CFG)
    ap.add_argument("--spine", type=float, nargs="?", const=3.0, default=None,
                    metavar="MM",
                    help="scheme the WIDENED alternative instead: widen until "
                         "the two back faces clear a centre plate of this "
                         "thickness (default 3 mm), drop the servo straddle to "
                         "zero, and report what the width costs")
    ap.add_argument("--sleeve", type=float, nargs="?", const=3.0, default=None,
                    metavar="WALL",
                    help="sketch the SLEEVE instead: one tube round both "
                         "cases with this wall thickness (default 3 mm), gap "
                         "closed to zero")
    ap.add_argument("--plate", action="store_true",
                    help="draw the plate-and-collar as built: outline, horn "
                         "relief, the eight screws, and the width chain")
    ap.add_argument("--tag", default=None,
                    help="output goes to plots/servo_mount_<tag>.png, so a "
                         "variant never overwrites the tracked figure")
    args = ap.parse_args()

    params, g = collect(args.params, args.linkage_config, args.spine,
                        args.sleeve)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6.2),
                                   gridspec_kw={"width_ratios": [1.15, 1]})
    if args.plate:
        draw_plate(g, params, ax1, ax2)
        fig.suptitle("Drive-servo mount — plate and collar, as exported",
                     fontsize=11)
    elif args.sleeve:
        cav = draw_sleeve(g, params, ax1, ax2, args.sleeve)
        fig.suptitle(f"Drive-servo mount — sleeve, {args.sleeve:.0f} mm wall "
                     f"(PROPOSAL)", fontsize=11)
    else:
        r_in, r_out, edge = draw(g, params, ax1, ax2, args.spine)
        fig.suptitle("Drive-servo mount — "
                     + (f"centre spine, {args.spine:.0f} mm plate, servos "
                        f"un-straddled (PROPOSAL)" if args.spine
                        else "two-plate cage (PROPOSAL)"), fontsize=11)
    fig.tight_layout()
    tag = args.tag or ("plate" if args.plate else
                       "sleeve" if args.sleeve else
                       "spine" if args.spine else "cage")
    out = _plots_dir() / f"servo_mount_{tag}.png"
    fig.savefig(out, dpi=160)
    print(f"wrote {out}")

    # The cheap back-face option, always reported: it is the number most
    # likely to be wanted and it costs nothing to print.
    for t in (3.0, 5.0):
        pl = pad_plane(params, t)
        w = 2 * (pl + g["pulley_servo_left"]["ln"] / 2
                 + g["axle_mount_left"]["ln"])
        print(f"  back-face pad, {t:.0f} mm plate: belt plane {pl:.2f} mm "
              f"-> rear width {w:.1f} mm")
    if args.plate:
        return
    if args.sleeve:
        print(f"  sleeve cavity {cav[1] - cav[0]:.1f} mm along the shaft axis")
        return

    wheel = g["omni_wheel_rear"]["r"]
    print(f"  inner plate  r={r_in:6.2f} mm   "
          f"clears the rear wheel (r={wheel:.1f}) by {r_in - wheel:.1f} mm")
    print(f"  outer plate  r={r_out:6.2f} mm")
    print(f"  both plates fit within |x| <= {abs(edge):.1f} mm, "
          f"so {2 * abs(edge):.1f} mm wide — inside the belts either side")


if __name__ == "__main__":
    main()
