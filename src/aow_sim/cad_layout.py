"""Emit the component layout in CAD coordinates, for drawing the real bike.

    python -m aow_sim.cad_layout            # -> docs/measurements/cad_layout.yaml

WHY THIS EXISTS: `config/bike_params.yaml` is the simulator's frame — **+X
forward, +Y left, +Z up**, metres, kilograms. The CAD model is drawn in the
usual mechanical convention — **+X right, +Y forward, +Z up**, millimetres,
grams. Hand-converting between them once per component is exactly the kind of
sign error that survives review, so it is done here, once, in code.

Both frames are right-handed, so the mapping is a pure 90 deg rotation about Z
with no mirroring:

    cad_x = -model_y        (right      = -left)
    cad_y =  model_x        (forward    =  forward)
    cad_z =  model_z        (up         =  up)

and box extents swap X and Y with it. The ORIGIN is unchanged and is the
**rear axle**, which sits `omni_wheel.outer_radius` (51.2 mm) above the floor
when the bike is upright.

DIRECTION OF TRUTH: this file is generated FROM the simulation, so it is the
sim's current belief about the layout, not a measurement of anything. Most of
it is `design` or `GUESS`. As parts get drawn and built, the real numbers go
back into `bike_params.yaml` (with `source: measured`) and this gets
regenerated -- not edited. It is an export, not a source.
"""

from __future__ import annotations

import argparse
from math import cos, radians, sin
from pathlib import Path

from .params import DEFAULT_PARAMS, load_params

CAD_PARAMS = "config/bike_params_cad.yaml"
# The 75 mm-envelope solve, not the 120 mm one the sim still builds.
LINKAGE_CFG = "config/wing_linkage_w75.yaml"
OUT = "docs/measurements/cad_layout.yaml"
OUT_FS = "docs/measurements/cad_layout.fs"

# Model-frame axes. +X forward, +Y left, +Z up.
AXIS_LATERAL = (0.0, 1.0, 0.0)        # wheel axles, across the bike
AXIS_LONGITUDINAL = (1.0, 0.0, 0.0)   # roof and bumpers, fore-aft


def load_sources(path=None) -> dict:
    """Raw YAML, so the `source:` tags survive.

    `load_params` normalizes `{value:, source:}` down to bare numbers (and
    derives the righting geometry), which is what every other consumer wants
    and exactly what this one must not lose. So values come from the normalized
    tree and provenance from the raw one.
    """
    import yaml
    with open(path or DEFAULT_PARAMS) as f:
        return yaml.safe_load(f)


def src(raw: dict, dotted: str, default: str = "design") -> str:
    """`src(raw, "bike.chassis.mass")` -> its source tag, or `default`."""
    node = raw
    for key in dotted.split("."):
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    if isinstance(node, dict):
        return str(node.get("source", default))
    return default


def _mm(v):
    # `+ 0.0` normalizes -0.0, which the sign flip on y produces for every
    # centreline part and which reads as a real offset at a glance.
    return round(float(v) * 1000.0, 2) + 0.0


def to_cad_pos(p):
    """model [x,y,z] m -> CAD [x,y,z] mm."""
    return [_mm(-p[1]), _mm(p[0]), _mm(p[2])]


def to_cad_dir(a):
    """model unit direction -> CAD unit direction.

    Same rotation as `to_cad_pos` but WITHOUT the translation, because a
    direction has no origin. Kept separate so the two can never be confused.
    """
    return [round(float(-a[1]), 6) + 0.0, round(float(a[0]), 6) + 0.0,
            round(float(a[2]), 6) + 0.0]


def frame_rotation(x_cad, y_cad, z_cad):
    """Rotation carrying the CAD axes onto (x_cad, y_cad, z_cad) -> (axis, deg).

    `fCuboid` can only build an axis-aligned box, so an arbitrarily oriented
    one is built square and rotated. A single axis-angle covers ANY rotation,
    which is why this replaces the earlier "tilt +Z onto a direction" helper
    for the servos: their orientation needs all three axes pinned, not one.
    """
    import numpy as _n
    R = _n.column_stack([_n.asarray(x_cad, float),
                         _n.asarray(y_cad, float),
                         _n.asarray(z_cad, float)])
    det = float(_n.linalg.det(R))
    if abs(det - 1.0) > 1e-6:
        raise ValueError(f"frame is not a proper rotation (det={det:+.3f}); "
                         "check the handedness of the local axes")
    ang = _n.arccos(max(-1.0, min(1.0, (_n.trace(R) - 1.0) / 2.0)))
    if ang < 1e-9:
        return None
    if abs(ang - _n.pi) < 1e-6:
        # 180 deg: the skew part vanishes, so take the axis from R + I.
        w, v = _n.linalg.eigh(R + _n.eye(3))
        ax = v[:, int(_n.argmax(w))]
    else:
        ax = _n.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                       R[1, 0] - R[0, 1]]) / (2.0 * _n.sin(ang))
    ax = ax / _n.linalg.norm(ax)
    return ([round(float(c), 6) + 0.0 for c in ax],
            round(float(_n.degrees(ang)), 4))


def rotation_from_z(cad_axis):
    """CAD +Z -> `cad_axis`, as (rotation_axis, degrees), or None if aligned.

    Done in Python so FeatureScript only has to call `rotationAround` with two
    numbers it was handed, rather than do cross products and clamping in a
    language we are debugging over a slow loop.
    """
    from math import acos, degrees, sqrt
    z = (0.0, 0.0, 1.0)
    a = cad_axis
    n = sqrt(sum(c * c for c in a))
    a = tuple(c / n for c in a)
    cross = (z[1] * a[2] - z[2] * a[1],
             z[2] * a[0] - z[0] * a[2],
             z[0] * a[1] - z[1] * a[0])
    m = sqrt(sum(c * c for c in cross))
    if m < 1e-9:
        return None                      # already along +Z (or exactly -Z)
    axis = tuple(round(c / m, 6) + 0.0 for c in cross)
    ang = degrees(acos(max(-1.0, min(1.0, sum(z[i] * a[i] for i in range(3))))))
    return axis, round(ang, 4)


def to_cad_extent(e):
    """model full extents [dx,dy,dz] m -> CAD [dx,dy,dz] mm (X and Y swap)."""
    return [_mm(e[1]), _mm(e[0]), _mm(e[2])]


def _g(x):
    return round(float(x) * 1000.0, 1)


def _steer_axis_offset(b: dict, d3: dict) -> float:
    """Perpendicular distance from the steer servo centre to the steering axis.

    Reported rather than silently corrected: this file is an export, and a
    position that disagrees with the mechanism belongs back in bike_params.
    """
    from math import cos, radians, sin, sqrt
    rake = radians(b["rake_deg"])
    axis = (-sin(rake), 0.0, cos(rake))
    p0 = (b["wheelbase"], 0.0, b["front_wheel"]["radius"]
          - b["chassis"].get("_axle", 0.0))
    p0 = (b["wheelbase"], 0.0, 0.0)          # axis passes through the front axle
    d = [d3["pos"][i] - p0[i] for i in range(3)]
    t = sum(d[i] * axis[i] for i in range(3))
    perp = [d[i] - t * axis[i] for i in range(3)]
    return sqrt(sum(c * c for c in perp))


def _add_linkage(add, params: dict, raw: dict, d3: dict,
                 bumpers: bool = False,
                 linkage_cfg: str = LINKAGE_CFG) -> None:
    """The four-bar wing linkage — `build_model(linkage=True)`.

    An ALTERNATIVE to the geared wing pair, never built alongside it. Same job
    (one servo, two wings) but the ratio varies through the stroke, so the
    deployed pose sits at the crank's dead point and holds the bike with no
    servo current.

    FRAMES: the linkage config is millimetres from the FLOOR
    and the centreline; everything here is metres from the REAR AXLE. The
    wheel radius is the only conversion and getting it wrong puts the whole
    mechanism a wheel-radius off the ground rather than failing loudly —
    the same warning `_add_wing_linkage` carries.
    """
    import yaml
    from pathlib import Path as _P

    cfg = yaml.safe_load(_P(linkage_cfg).read_text())
    bk, mech, st = cfg["bike"], cfg["mechanism"], cfg["stroke"]
    r_rear = params["omni_wheel"]["outer_radius"]
    px = params["righting"]["wings"]["pivot"][0]      # fore/aft station
    w_ref = params["righting"]["wings"]

    def z_of(mm):
        return mm / 1000.0 - r_rear

    servo_z = z_of(bk["wheel_radius"] + mech["servo_offset"])
    pivot_y = mech["wing_pivot_offset"] / 1000.0
    half_span = bk["bike_width"] / 2000.0
    lo, hi = z_of(bk["ground_clearance"]), z_of(bk["bike_height"] - half_span * 1000)

    add("linkage_crank_servo", "righting", [px, 0.0, servo_z],
        box=d3["box_size"], mount=("linkage_crank_servo", d3, "shaft"),
        mass=w_ref["servo_mass"],
        source={"pos": "design (linkage cfg)", "size": "datasheet",
                "mass": "datasheet"},
        note=f"XC330-T181 driving BOTH cranks off one shaft, hinge axis "
             f"fore-aft. Arms {mech['wing_first_link_length']['right']:.1f} / "
             f"{mech['wing_first_link_length']['left']:.1f} mm at "
             f"{mech['first_link_angle_deg']:.1f} deg and "
             f"{mech['angle_between_first_links']:.1f} deg apart — deliberately "
             f"UNEQUAL, so the two sides reach their dead points at different "
             f"crank angles. Run in current-based position mode at "
             f"{st['goal_current_nm']} N.m; that is a WINDOW (0.38-0.50 works "
             f"8/8 falls), not a minimum.")

    add("linkage_crank_shaft", "righting", [px, 0.0, servo_z],
        source={"pos": "design (linkage cfg)"},
        note="The crank shaft — both arms are keyed to it, and it is the "
             "servo's output axis (fore-aft, CAD Y). Mate the real ROBOTIS "
             "model to THIS, not to the box: the box orientation about this "
             "axis is a guess about which face carries the horn.")

    # The four-bar in its STOWED pose, as points plus thin links. Every one of
    # these is a fixed pose of a MOVING mechanism — drawn so the geometry is
    # legible while packaging, not because the parts live here.
    import numpy as _np
    ang_r = _np.deg2rad(mech["first_link_angle_deg"])
    ang_l = _np.deg2rad(mech["first_link_angle_deg"]
                        + mech["angle_between_first_links"])
    off = _np.asarray(mech["wing_attach_offset"], float) / 1000.0
    nodes = {}
    for tag, ang, sgn in (("right", ang_r, -1), ("left", ang_l, +1)):
        L = mech["wing_first_link_length"][tag] / 1000.0
        tip = [px, L * _np.cos(ang), servo_z + L * _np.sin(ang)]
        attach = [px, sgn * pivot_y + sgn * off[0], off[1]]
        pivot = [px, sgn * pivot_y, 0.0]
        nodes[tag] = (tip, attach, pivot)
        add(f"crank_tip_{tag}", "righting", tip,
            source={"pos": "derived from linkage cfg"},
            note=f"Tip of the {tag} crank arm, stowed: "
                 f"{mech['wing_first_link_length'][tag]:.1f} mm at "
                 f"{_np.rad2deg(ang):.1f} deg from the shaft. NOTE THE CROSSOVER "
                 f"— this arm sits on the far side of the centreline from the "
                 f"{tag} wing it drives.")
        add(f"wing_attach_{tag}", "righting", attach,
            source={"pos": "derived from linkage cfg"},
            note=f"Where the coupler meets the {tag} wing, "
                 f"{off[0]*1000:.1f} / {off[1]*1000:.1f} mm from that wing's "
                 "pivot. Stowed pose.")

    # Swept path of each crank arm tip over the full stroke. `crank_tip` in
    # analysis/wing_linkage.py is servo + L*[cos(a0 + travel), sin(a0 + travel)]
    # in the (y, z) plane, so travel ADDS to the start angle — the arc runs
    # a0 -> a0 + servo_travel_deg. Three points are enough for a sketch arc.
    travel = float(st["servo_travel_deg"])
    for tag, ang0 in (("right", ang_r), ("left", ang_l)):
        L = mech["wing_first_link_length"][tag] / 1000.0
        pts = []
        for frac in (0.0, 0.5, 1.0):
            a = ang0 + _np.deg2rad(travel) * frac
            pts.append([px, L * _np.cos(a), servo_z + L * _np.sin(a)])
        add(f"crank_sweep_{tag}", "righting", pts[1], arc=pts,
            source={"pos": "derived from linkage cfg + stroke"},
            note=f"Path swept by the {tag} crank arm tip over the full "
                 f"{travel:.0f} deg stroke, radius "
                 f"{mech['wing_first_link_length'][tag]:.1f} mm about the crank "
                 f"shaft. THIS IS THE CLEARANCE THAT MATTERS for the crank — "
                 "nothing may enter it. The couplers and wings sweep too, but "
                 "their envelopes are not circular and are not drawn.")

    def add_link(name, p0, p1, note):
        import numpy as _n
        a, bb = _n.asarray(p0, float), _n.asarray(p1, float)
        d = bb - a
        ln = float(_n.linalg.norm(d))
        add(name, "righting", list((a + bb) / 2),
            cyl=(0.0015, ln, tuple(d / ln)), seg=(list(a), list(bb)),
            source={"pos": "derived from linkage cfg"}, note=note)

    for tag in ("right", "left"):
        tip, attach, pivot = nodes[tag]
        add_link(f"link_crank_{tag}", [px, 0.0, servo_z], tip,
                 f"Crank arm ({tag}), stowed. MOVES — drawn as a thin rod so "
                 "the stowed pose is legible, not as a part.")
        add_link(f"link_coupler_{tag}", tip, attach,
                 f"Coupler ({tag}), stowed. This is the link that crosses the "
                 "centreline. MOVES.")
        add_link(f"link_rocker_{tag}", pivot, attach,
                 f"Wing rocker ({tag}) — pivot to coupler attach. Fixed to the "
                 "wing panel, so it swings with it.")

    for tag, sgn in (("left", +1), ("right", -1)):
        add(f"wing_{tag}_pivot", "righting", [px, sgn * pivot_y, 0.0],
            source={"pos": "design (linkage cfg)"},
            note=f"Wing hinge on the chassis, {_mm(pivot_y)} mm off the "
                 f"centreline and level with the rear axle. The coupler runs "
                 f"from the crank arm tip to an attach point "
                 f"{mech['wing_attach_offset'][0]:.1f} / "
                 f"{mech['wing_attach_offset'][1]:.1f} mm from this pivot; "
                 "that link moves, so it is not drawn.")
        # A flat PLATE, not the sim's collision capsule. Thickness is the
        # lateral extent, so the plate's outer face lands exactly on the
        # half-span instead of a capsule radius pushing 5 mm past it.
        th = w_ref.get("panel_thickness", 0.004)
        add(f"wing_{tag}_panel", "righting",
            [px + w_ref.get("panel_offset_x", 0.0),
             sgn * (half_span - th / 2), (lo + hi) / 2],
            box=(w_ref.get("panel_length_x", 0.09), th, hi - lo),
            mass=w_ref["mass"],
            source={"pos": "derived from linkage cfg + panel offset",
                    "size": "design",
                    "mass": src(raw, "righting.wings.mass")},
            note=f"STOWED pose. Flat plate {_mm(w_ref.get('panel_length_x', 0.09))} "
                 f"x {_mm(hi - lo)} x {_mm(th)} mm, OUTER FACE on the "
                 f"{_mm(half_span)} mm half-span, spanning z {_mm(lo)}..{_mm(hi)}. "
                 f"Centre sits {_mm(-w_ref.get('panel_offset_x', 0.0))} mm "
                 f"rearward of the linkage station — the mechanism is planar but "
                 "the plate is what touches the floor and wants area behind it. "
                 "It sweeps outboard and down when deployed; leave that clear.")

    rmm = half_span * 1000.0
    add("roof", "righting", [0.0925, 0.0, (bk["bike_height"] - rmm) / 1000.0 - r_rear],
        cap=(half_span, 0.145, AXIS_LONGITUDINAL),
        mass=params["righting"]["roof"]["mass"],
        source={"pos": "design", "size": "DERIVED from the linkage stow envelope",
                "mass": src(raw, "righting.roof.mass")},
        note="Re-derived for the LINKAGE, whose stowed wing is a full-length "
             "panel rather than the geared pair's shape. Axis at the panel "
             "top, radius the stow half-span, so the crest clears the panels "
             "by a radius — without that an inverted bike perches on two panel "
             "edges plus the ridge instead of rolling off.")

    if not bumpers:
        return
    bp = params["righting"]["bumper"]
    for tag, sgn in (("left", +1), ("right", -1)):
        add(f"bumper_{tag}", "righting",
            [(bp["x_start"] + bp["x_end"]) / 2, sgn * bp["half_span"], bp["height"]],
            cap=(bp["radius"], bp["x_end"] - bp["x_start"], AXIS_LONGITUDINAL),
            mass=bp["mass"],
            source={"pos": "design", "size": "design",
                    "mass": src(raw, "righting.bumper.mass")},
            note="Spans the drive servos and sets the resting stance on a side "
                 "fall. Shared by both righting mechanisms.")


def build(params: dict, raw: dict, mechanism: str = "linkage",
          bumpers: bool = False, chassis_box: bool = False,
          linkage_cfg: str = LINKAGE_CFG) -> list[dict]:
    """Component list in MODEL units (m, kg); converted on the way out."""
    b, ow, dt, sv = params["bike"], params["omni_wheel"], params["drivetrain"], params["servos"]
    rg = params.get("righting", {})
    axle_h = ow["outer_radius"]
    items: list[dict] = []

    def add(name, group, pos, *, box=None, cyl=None, cap=None, zaxis=None,
            seg=None, frame=None, arc=None, mount=None, mass=None,
            source=None, note=None):
        if mount:
            _nm, _spec, *_a = mount
            _sp, pos, frame = mount_of(_nm, _spec, pos,
                                       _a[0] if _a else "centre")
        d = {"name": name, "group": group, "pos": pos, "mass": mass,
             "source": source or {}}
        if zaxis:
            d["zaxis"] = zaxis
        if seg:
            d["seg"] = seg
        if frame:
            d["frame"] = frame
        if arc:
            d["arc"] = arc
        if box:
            d["box"] = box
        if cyl:
            d["cyl"] = cyl
        if cap:
            d["cap"] = cap
        if note:
            d["note"] = note
        items.append(d)

    def mount_of(name, spec, pos, anchor="centre"):
        """(shaft_point, box_centre, local axes) for one installed servo.

        Local frame: +D is the shaft axis (horn faces +D), +H is `body_up`,
        +W completes it. The shaft sits on the horn FACE, i.e. D/2 out from
        the centre less `shaft_from_horn_face`, and (H/2 - shaft_from_end)
        toward +H. Both offsets come off the ROBOTIS drawings.
        """
        import numpy as _n
        mt = params.get("cad_mounts", {}).get(name, {})
        d_hat = _n.asarray(mt.get("shaft_axis", [0, 1, 0]), float)
        d_hat = d_hat / _n.linalg.norm(d_hat) * mt.get("horn_dir", 1.0)
        h_hat = _n.asarray(mt.get("body_up", [0, 0, 1]), float)
        h_hat = h_hat - d_hat * float(h_hat @ d_hat)          # re-orthogonalize
        h_hat = h_hat / _n.linalg.norm(h_hat)
        # w = d CROSS h, so that (w, d, h) is RIGHT-handed and w x d = h.
        # The other order gives a left-handed triple, whose matrix is a
        # reflection rather than a rotation — det -1, and the axis-angle
        # extraction then divides by zero.
        w_hat = _n.cross(d_hat, h_hat)
        D, H = spec["box_size"][0], spec["box_size"][2]
        off = (d_hat * (D / 2 - spec.get("shaft_from_horn_face", 0.0))
               + h_hat * (H / 2 - spec.get("shaft_from_end", 0.0))
               * spec.get("shaft_dir", 1.0))
        # anchor="shaft" means the caller knows where the AXIS is — a crank
        # joint, a steering axis — and the case is derived from it. That is the
        # honest direction whenever the mechanism fixes the axis.
        if anchor == "shaft":
            sp = _n.asarray(pos, float)
            centre = sp - off
        else:
            centre = _n.asarray(pos, float)
            sp = centre + off
        return [float(v) for v in sp], [float(v) for v in centre], (w_hat, d_hat, h_hat)


    # ---- frame -------------------------------------------------------
    ch = b["chassis"]
    if chassis_box:
        add("chassis_box", "frame", ch["com_pos"], box=ch["box_size"], mass=ch["mass"],
            source={"pos": "design", "size": "design",
                    "mass": src(raw, "bike.chassis.mass")},
            note="Inertia primitive for frame + wiring, NOT a real part, and "
                 "OFF BY DEFAULT (--chassis-box) — the roof already covers this "
                 "envelope and the block corresponds to nothing you would draw. "
                 "It still carries the frame's mass in the sim.")

    # ---- drivetrain --------------------------------------------------
    n_ax = ow["n_axles"]
    add("omni_wheel_rear", "drivetrain", [0.0, 0.0, 0.0],
        cyl=(ow["outer_radius"], ow["width"], AXIS_LATERAL),
        mass=ow["hub"]["mass"] + ow["ring"]["mass"] + n_ax * ow["roller"]["pair_mass"],
        source={"pos": "origin", "size": src(raw, "omni_wheel.outer_radius"),
                "mass": "measured"},
        note=f"THE ORIGIN. Axle sits {_mm(axle_h)} mm above the floor when "
             f"upright. {int(n_ax)} axles x 2 truncated-cone rollers. A bevel "
             "DIFFERENTIAL inside: the two inputs are the ring-gear shafts, and "
             "hub speed is their mean.")
    # --- belt drive: pulleys, axle mounts, and the servo station ---------
    import numpy as _np
    be = dt["belt"]
    pw = be["width"] + 2 * be["flange_thickness"]          # pulley axial width
    plane = ow["width"] / 2 + dt["wheel_clearance"] + pw / 2
    d_in = be["teeth_input"] * be["pitch"] / _np.pi
    d_sv = be["teeth_servo"] * be["pitch"] / _np.pi
    # L = 2C + (pi/2)(D1+D2) + (D1-D2)^2/4C, solved for C.
    _a, _b, _c = 2.0, (_np.pi / 2) * (d_sv + d_in) - be["length"], (d_sv - d_in) ** 2 / 4
    C = float((-_b + _np.sqrt(_b * _b - 4 * _a * _c)) / (2 * _a))
    C_m = C
    am = dt["axle_mount"]
    am_y = ow["width"] / 2 + dt["wheel_clearance"] + pw + am["width"] / 2

    for tag, sgn in (("left", +1), ("right", -1)):
        add(f"pulley_input_{tag}", "drivetrain", [0.0, sgn * plane, 0.0],
            cyl=(d_in / 2 + be["flange_margin"] / 2, pw, AXIS_LATERAL),
            source={"pos": "derived from belt width + wheel clearance",
                    "size": f"{int(be['teeth_input'])}T HTD{int(be['pitch']*1000)}M"},
            note=f"{int(be['teeth_input'])}T pulley on the ring-gear input "
                 f"shaft, coaxial with the rear axle. Pitch dia {_mm(d_in)} mm; "
                 f"envelope is the flange dia. Belt centre plane at "
                 f"{_mm(plane)} mm — the minimum a {_mm(be['width'])} mm belt "
                 f"allows over a {_mm(ow['width'])} mm wheel.")
        add(f"axle_mount_{tag}", "drivetrain", [0.0, sgn * am_y, 0.0],
            cyl=(d_in / 2 + be["flange_margin"] / 2 + am["diameter_margin"] / 2,
                 am["width"], AXIS_LATERAL),
            source={"pos": "derived", "size": "design"},
            note="Where the rear axle is picked up, outboard of the pulley and "
                 "a little larger so the belt never touches it. This is the "
                 "stub the chassis eventually attaches to — the widest thing "
                 f"at the rear, at {_mm(am_y + am['width'] / 2)} mm half-width.")

    # The servo sits on the circle of radius C about the rear axle, IN the belt
    # plane. Angle is a free choice; the current one is preserved.
    # Angular station about the rear axle: 0 = level and forward, 90 = above.
    # The two servos STRADDLE it. Their cases sit inboard of their own belt
    # planes and so overlap across the centreline; separating them tangentially
    # is what keeps them apart, and the half-separation is set by the case
    # width plus a gap over the chord at radius C.
    # Separation is SOLVED offline as a 2D packing problem and recorded in the
    # config — see `drive_servo_separation_deg` there for the alternatives and
    # why the parallel arrangement was chosen over the true minimum. It is not
    # re-solved here: this file exports the layout, it does not optimise it.
    th0 = _np.deg2rad(dt["drive_servo_angle_deg"])
    dth = _np.deg2rad(dt["drive_servo_separation_deg"]) / 2
    servo_angle = {"left": th0 - dth, "right": th0 + dth}
    for tag, sgn in (("left", +1), ("right", -1)):
        _t = servo_angle[tag]
        add(f"pulley_servo_{tag}", "drivetrain",
            [C * _np.cos(_t), sgn * plane, C * _np.sin(_t)],
            cyl=(d_sv / 2 + be["flange_margin"] / 2, pw, AXIS_LATERAL),
            source={"pos": "derived — centre distance C on the belt plane",
                    "size": f"{int(be['teeth_servo'])}T HTD{int(be['pitch']*1000)}M"},
            note=f"{int(be['teeth_servo'])}T driving pulley, pitch dia "
                 f"{_mm(d_sv)} mm. Centre distance {_mm(C)} mm falls out of the "
                 f"{_mm(be['length'])} mm belt and the tooth counts — it is not "
                 f"a placement choice. Only the ANGLE about the rear axle is "
                 "free, so both servos may sit anywhere on that circle and "
                 "still share identical belts.")

    # ---- steering ----------------------------------------------------
    fw = b["front_wheel"]
    front_z = fw["radius"] - axle_h
    add("front_wheel", "steering", [b["wheelbase"], 0.0, front_z],
        cyl=(fw["radius"], fw["width"], AXIS_LATERAL), mass=fw["mass"],
        source={"pos": src(raw, "bike.wheelbase"),
                "size": src(raw, "bike.front_wheel.radius"),
                "mass": src(raw, "bike.front_wheel.mass")},
        note=f"Axle sits {_mm(-front_z)} mm BELOW the rear axle: the front wheel "
             f"is {_mm(fw['radius'])} mm radius against the rear's {_mm(axle_h)} "
             f"mm. Tire crown radius {_mm(fw['crown_radius'])} mm. This wheel "
             "cannot slide sideways, which is what makes the lateral velocity "
             "estimator work.")
    # The steering axis is raked BACK from vertical, so travelling up it moves
    # you aft: model direction (-sin, 0, cos). The fork capsule hangs half its
    # length up that axis from the front axle. Getting this from the rake
    # rather than from a fudged z is what stops it rendering as a horizontal
    # cylinder through the front wheel.
    rake = radians(b["rake_deg"])
    fork_axis = (-sin(rake), 0.0, cos(rake))
    # The steer column is ONE number: the shaft sits (wheel radius + clearance)
    # up the axis from the front axle, and the fork spans exactly that. Nothing
    # here is a free length any more — it was a hardcoded 100 mm before, which
    # is why the servo floated so high.
    fork_len = fw["radius"] + b["steering"]["servo_clearance"]
    fork_pos = [b["wheelbase"] + fork_axis[0] * fork_len / 2, 0.0,
                front_z + fork_axis[2] * fork_len / 2]
    add("fork", "steering", fork_pos,
        cap=(0.005, fork_len, fork_axis),
        mass=b["fork_mass"],
        source={"pos": "design", "size": "design",
                "mass": src(raw, "bike.fork_mass")},
        note=f"Steering axis raked {b['rake_deg']} deg back from vertical, fork "
             f"offset {_mm(b['fork_offset'])} mm. `axis` IS the steering axis — "
             "sketch the head tube and any clamp perpendicular to it, not to "
             "the world. pos_mm is the capsule centre; the axis passes through "
             "the front axle.")

    # ---- servos ------------------------------------------------------
    d4 = sv["xc430_w150"]
    for tag, key in (("left", "pos_left"), ("right", "pos_right")):
        nm = f"servo_drive_{tag}"
        sgn = +1 if tag == "left" else -1
        _t = servo_angle[tag]
        # body_up comes from cad_mounts (both servos share it, so the cases are
        # parallel). Anchored on the shaft: the belt fixes it, case follows.
        #
        # The horn face sits at the pulley's INNER face, not on the belt plane:
        # the pulley bolts ONTO the horn and its body occupies the plane. Using
        # `plane` here put the actuation point inside the pulley and pushed each
        # servo 5.5 mm too far outboard.
        horn_y = plane - pw / 2 + dt.get("pulley_hub_offset", 0.0) * 1000
        shaft = [C * _np.cos(_t), sgn * horn_y, C * _np.sin(_t)]
        sp, centre, axes = mount_of(nm, d4, shaft, "shaft")
        add(nm, "servos", centre, box=d4["box_size"], frame=axes,
            mass=d4["mass"],
            source={"pos": "design", "size": "datasheet",
                    "mass": src(raw, "servos.xc430_w150.mass")},
            note=f"XC430-W150, drives one ring-gear input by belt. POSITION is "
                 f"derived: the shaft must lie in the belt plane at "
                 f"{_mm(plane)} mm and {_mm(C)} mm from the rear axle. Only the "
                 f"angle about that axle is free. Orientation comes from "
                 "`cad_mounts`; the two bodies overlap near the centreline, so "
                 "give them different angles rather than trying to fit them "
                 "side by side.")
        add(f"{nm}_shaft", "servos", sp,
            source={"pos": "datasheet offsets + cad_mounts"},
            note=f"Output shaft, ON THE HORN FACE: "
                 f"{_mm(d4['box_size'][0] / 2)} mm out along the axis and "
                 f"{_mm(d4['box_size'][2] / 2 - d4['shaft_from_end'])} mm off "
                 f"the case centre along body_up. Mate the ROBOTIS model here. "
                 f"Cables exit the OPPOSITE face — keep routing room there.")
    d3 = sv["xc330_t181"]
    params.setdefault("cad_mounts", {}).setdefault("servo_steer", {})[
        "shaft_axis"] = [-fork_axis[0], -fork_axis[1], -fork_axis[2]]
    # Anchored on the SHAFT: the axis and the clearance fix where it goes, so
    # the case is derived rather than positioned and then checked.
    steer_shaft_pt = [b["wheelbase"] + fork_axis[0] * fork_len, 0.0,
                      front_z + fork_axis[2] * fork_len]
    sp_st, c_st, ax_st = mount_of("servo_steer", d3, steer_shaft_pt, "shaft")
    add("servo_steer", "servos", c_st, box=d3["box_size"], frame=ax_st,
        mass=d3["mass"],
        source={"pos": "design", "size": "datasheet",
                "mass": src(raw, "servos.xc330_t181.mass")},
        note=f"XC330-T181, extended-position mode. Its +Z (opposite the horn) "
             f"runs along the STEERING AXIS, so the horn faces down the axis "
             f"toward the front axle. NOTE the sim models this box "
             f"axis-aligned and {_mm(_steer_axis_offset(b, d3))} mm off the "
             f"steering axis; at gear_ratio 1.0 the real servo must be "
             f"coaxial, so bike_params is the thing to correct, not this "
             f"export. Loses its multi-turn count across a power cycle, so it "
             f"needs homing or an index feature.")

    add("servo_steer_shaft", "servos", sp_st,
        source={"pos": "derived — front axle, on the steering axis"},
        note=f"Output shaft on the horn face, pointing DOWN the steering axis "
             f"toward the front axle. Direct drive at gear_ratio "
             f"{b['steering']['gear_ratio']} forces it coaxial, so it is "
             f"DERIVED: {_mm(fw['radius'])} mm of wheel radius plus "
             f"{_mm(b['steering']['servo_clearance'])} mm of "
             f"`steering.servo_clearance`, up the axis from the axle. Change "
             "the clearance to move the whole steer column; the case follows.")

    # ---- electronics -------------------------------------------------
    ah = b["ahrs"]
    add("ahrs_tm151", "electronics", ah["pos"], box=ah["box_size"], mass=ah["mass"],
        source={"pos": "design", "size": src(raw, "bike.ahrs.mass"),
                "mass": src(raw, "bike.ahrs.mass")},
        note="SYD Dynamics TM151, datasheet size and mass. NOTE the "
             "simulator's own bike_params.yaml still carries the old "
             "30 x 30 x 12 mm / 12 g placeholders — this is the CAD copy, and "
             "the two are meant to converge when it becomes authoritative. "
             "Long axis fore-aft and thin axis vertical is a MOUNTING "
             "assumption; MountCalibration absorbs any fixed orientation, so "
             "it only has to be right for the envelope. Position hardly "
             "matters (quaternion and gyro are position-invariant on a rigid "
             "body) but mount RIGIDITY does — compliance becomes a resonance "
             "the gyro reports as real body rotation.")
    payload_notes = {
        "battery": "3S 1300-1500 mAh LiPo, slung under the frame between the "
                   "wheels. Must stay on the centerline: lateral offset is a "
                   "standing roll bias the controller trims out continuously. "
                   "Its leads are the strongest magnetic source on the bike — "
                   "keep them twisted and away from the AHRS.",
        "pi": "Raspberry Pi Zero 2 W. Board outline is exact; the microSD "
              "protrudes past it. Keep the ANTENNA end away from the pack — "
              "a LiPo is effectively RF-opaque. Powered through GPIO pins 2/4, "
              "so the micro-USB OTG port must stay reachable for the U2D2.",
        "u2d2": "ROBOTIS U2D2 with its USB cable. Needs its micro-USB and its "
                "3-pin JST-EH TTL port both accessible; the USB cable to the "
                "Pi wants to be as short as routing allows.",
        "power_board": "Buck regulator, splitter perfboard, bulk caps and the "
                       "XT30 trunk as one lump. The tallest items are the "
                       "standing electrolytics. This is where the main switch "
                       "and fuse tie in, so it wants to be reachable.",
    }
    for key, part in (b.get("payload") or {}).items():
        frame = None
        box = part["box_size"]
        if "body_up" in part:
            # Oriented pack: box_size is (length, width, thickness) in the
            # pack's OWN frame, with length along body_up. Reordered to the
            # (D, W, H) the framed-box path expects.
            import numpy as _n
            h = _n.asarray(part["body_up"], float); h /= _n.linalg.norm(h)
            d = _n.array([0.0, 1.0, 0.0])                 # lateral
            w = _n.cross(d, h)
            frame = (w, d, h)
            box = (part["box_size"][1], part["box_size"][2], part["box_size"][0])
        add(f"payload_{key}", "electronics", part["pos"], box=box, frame=frame,
            mass=part["mass"],
            source={"pos": "design (plausible layout, NOT a design)",
                    "size": "design",
                    "mass": src(raw, f"bike.payload.{key}.mass")},
            note=payload_notes.get(key, ""))

    # ---- self-righting ------------------------------------------------
    if mechanism == "linkage":
        _add_linkage(add, params, raw, d3, bumpers, linkage_cfg)
    elif mechanism == "wings" and "roof" in rg:
        rf, w, bp = rg["roof"], rg["wings"], rg["bumper"]
        add("roof", "righting", [(rf["x_start"] + rf["x_end"]) / 2, 0.0, rf["height"]],
            cap=(rf["radius"], rf["x_end"] - rf["x_start"], AXIS_LONGITUDINAL),
            mass=rf["mass"],
            source={"pos": "design", "size": "DERIVED from bike_width/bike_height",
                    "mass": src(raw, "righting.roof.mass")},
            note=f"Rounded shell so a fall rolls to a known attitude. Radius "
                 f"{_mm(rf['radius'])} mm and height {_mm(rf['height'])} mm are "
                 f"DERIVED from righting.bike_width ({_mm(rg['bike_width'])} mm) "
                 f"and bike_height ({_mm(rg['bike_height'])} mm) — change those, "
                 "not these.")
        for tag, sgn in (("left", +1), ("right", -1)):
            add(f"bumper_{tag}", "righting",
                [(bp["x_start"] + bp["x_end"]) / 2, sgn * bp["half_span"],
                 bp["height"]],
                cap=(bp["radius"], bp["x_end"] - bp["x_start"], AXIS_LONGITUDINAL),
                mass=bp["mass"],
                source={"pos": "design", "size": "design",
                        "mass": src(raw, "righting.bumper.mass")},
                note="Spans the drive servos and sets the resting stance after a "
                     "side fall. Shared by both righting mechanisms.")
        piv = w["pivot"]
        for tag, sgn in (("left", +1), ("right", -1)):
            add(f"wing_{tag}_pivot", "righting", [piv[0], sgn * piv[1], piv[2]],
                mass=w["mass"],
                source={"pos": src(raw, "righting.wings.pivot"),
                        "mass": src(raw, "righting.wings.mass")},
                note=f"Hinge for one wing (mass is per wing). Crank "
                     f"{_mm(w['crank_length'])} mm out to the elbow at "
                     f"{w['crank_deg']} deg, then a {_mm(w['length'])} mm leg to "
                     f"the roof axis — both DERIVED. Deploys to "
                     f"{w['deploy_deg']} deg. Lowering both to the floor is the "
                     "candidate AHRS calibration pose.")
        add("servo_wings", "righting", [piv[0], 0.0, piv[2]], box=d3["box_size"],
            mass=w["servo_mass"] + w["gearbox_mass"],
            source={"pos": "design", "size": "datasheet",
                    "mass": src(raw, "righting.wings.servo_mass")},
            note=f"A fourth XC330-T181 driving BOTH wings through a "
                 f"{w['gear_ratio']}:1 gear train; mass here includes the "
                 f"{_g(w['gearbox_mass'])} g gearbox. The wings are mirrored, so "
                 "one servo moves the pair.")
    return items


def render(items: list[dict], params: dict, linkage_used: str = "") -> str:
    axle_h = params["omni_wheel"]["outer_radius"]
    L = [
        "# AOW bike — component layout in CAD coordinates.",
        "#",
        "# GENERATED — do not edit. Regenerate with:",
        "#     python -m aow_sim.cad_layout",
        "#",
        "# FRAME:  +X right, +Y forward, +Z up.   UNITS: mm and g.",
        f"# ORIGIN: the REAR AXLE, {_mm(axle_h)} mm above the floor when upright.",
        "#",
        "# This is the SIMULATOR's belief about the layout, exported for drawing.",
        "# It is NOT a measurement — most entries are `design` or `GUESS`, and the",
        "# payload positions are explicitly a plausible layout rather than a",
        "# design. As parts get drawn and built, the real numbers go back into",
        "# config/bike_params.yaml with `source: measured` and this file is",
        "# REGENERATED. It is an export, never a source.",
        f"# Wing linkage geometry from: {linkage_used}",
        "#",
        "# Conversion from the sim frame (+X forward, +Y left, +Z up, m/kg):",
        "#     cad_x = -model_y ;  cad_y = model_x ;  cad_z = model_z",
        "# Both frames are right-handed, so this is a pure 90 deg rotation about Z",
        "# with no mirroring. Box extents swap X and Y along with it.",
        "#",
        "# `pos_mm` is the component CENTRE unless its note says otherwise.",
        "# `axis` is named in the frame it was authored in: `y_model` means the",
        "# axis runs left-right across the bike, i.e. along CAD X.",
        "",
    ]
    groups: dict[str, list[dict]] = {}
    for it in items:
        groups.setdefault(it["group"], []).append(it)

    headings = {
        "frame": "Chassis — one inertia primitive, not a drawn part.",
        "drivetrain": "Rear omni-wheel and its two belt-driven inputs.",
        "steering": "Front wheel, fork, steering axis.",
        "servos": "Dynamixels. Drive pair and steer; the wing servo is under `righting`.",
        "electronics": "AHRS and the untethered payload.",
        "righting": "Self-righting candidates — NOTHING HERE IS BUILT YET.",
    }
    for grp in ("frame", "drivetrain", "steering", "servos", "electronics", "righting"):
        if grp not in groups:
            continue
        L += [f"# --- {headings[grp]}", f"{grp}:"]
        for it in groups[grp]:
            L.append(f"  {it['name']}:")
            L.append(f"    pos_mm: {to_cad_pos(it['pos'])}")
            if "box" in it:
                L.append("    shape: box")
                e = ([_mm(it["box"][1]), _mm(it["box"][0]), _mm(it["box"][2])]
                     if "frame" in it else to_cad_extent(it["box"]))
                L.append(f"    envelope_mm: {e}")
            if "arc" in it:
                a, m, e = (to_cad_pos(q) for q in it["arc"])
                L.append("    shape: arc")
                L.append(f"    radius_mm: {_mm(_arc_radius(it['arc']))}")
                L.append(f"    arc_start_mm: {[a[0], a[2]]}")
                L.append(f"    arc_mid_mm: {[m[0], m[2]]}")
                L.append(f"    arc_end_mm: {[e[0], e[2]]}")
                L.append("    plane: linkage station, sketch coords (CAD x, z)")
            if "box" in it:
                rot = None
                if "frame" in it:
                    w, d, h = (to_cad_dir(v) for v in it["frame"])
                    L.append(f"    axis_width: {w}")
                    L.append(f"    axis_shaft: {d}   # horn faces this way")
                    L.append(f"    axis_up: {h}")
                    rot = frame_rotation(w, d, h)
                elif "zaxis" in it:
                    za = to_cad_dir(it["zaxis"])
                    L.append(f"    z_axis: {za}   # box +Z, CAD frame")
                    rot = rotation_from_z(za)
                if rot:
                    L.append(f"    rotate_about: {list(rot[0])}")
                    L.append(f"    rotate_deg: {rot[1]}")
            elif "cyl" in it or "cap" in it:
                r, ln, ax = it.get("cyl") or it["cap"]
                L.append(f"    shape: {'cylinder' if 'cyl' in it else 'capsule'}")
                L.append(f"    radius_mm: {_mm(r)}")
                L.append(f"    length_mm: {_mm(ln)}")
                L.append(f"    axis: {to_cad_dir(ax)}   # unit vector, CAD frame")
            else:
                L.append("    shape: point")
            if it.get("mass") is not None:
                L.append(f"    mass_g: {_g(it['mass'])}")
            L.append("    source: {%s}" % ", ".join(
                f"{k}: {v}" for k, v in it["source"].items()))
            if "note" in it:
                L.append("    note: >-")
                L += [f"      {ln}" for ln in _wrap(it["note"], 66)]
            L.append("")
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------------------
# FeatureScript output
# ---------------------------------------------------------------------------
#
# UNTESTED AGAINST ONSHAPE. The DATA below is generated from the same source as
# the YAML and is the valuable half; the ~40 lines of drawing code that consume
# it are a first draft written from the FeatureScript docs, not from a running
# document. If the API calls need fixing, fix them and regenerate — the data
# block is machine-written and will not be disturbed.
#
# Two things it does once inserted into a Part Studio:
#   1. `setVariable` for every coordinate, so sketches can reference
#      `#aow_servo_drive_left_y` and stay linked to the sim's numbers;
#   2. optionally draws each envelope as a solid, so the packaging problem is
#      visible in space instead of in a table.

FS_HEADER = """FeatureScript {ver};
import(path : "onshape/std/geometry.fs", version : "{ver}.0");
import(path : "onshape/std/variable.fs", version : "{ver}.0");

// AOW bike component layout — GENERATED, do not edit.
//   python -m aow_sim.cad_layout --format featurescript
//
// FRAME:  +X right, +Y forward, +Z up.  ORIGIN: the rear axle,
// {axle} mm above the floor when the bike is upright.
//
// Exported from the simulator (config/bike_params.yaml). Most entries are
// `design` or `GUESS` — see docs/measurements/cad_layout.yaml for the
// provenance of every number, which is deliberately NOT duplicated here.
//
// !! The version number on the two lines above must match your document. The
// !! easiest fix is to create the Feature Studio first, then replace only the
// !! body below its auto-inserted header.
"""

FS_FEATURE = """
annotation { "Feature Type Name" : "AOW bike layout" }
export const aowBikeLayout = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Publish variables" }
        definition.publishVariables is boolean;

        annotation { "Name" : "Draw envelopes" }
        definition.drawEnvelopes is boolean;

        annotation { "Name" : "Draw origin points" }
        definition.drawPoints is boolean;

        annotation { "Name" : "Draw four-bar sketch" }
        definition.drawFourBarSketch is boolean;

        annotation { "Name" : "Draw axis planes" }
        definition.drawAxisPlanes is boolean;

        annotation { "Name" : "Plane size" }
        isLength(definition.planeSize, LENGTH_BOUNDS);
    }
    {
        // Each block is independently switchable, and they run cheapest and
        // safest first. A runtime error anywhere aborts the WHOLE feature, so
        // if one of these misbehaves you can still get the others by turning
        // it off — which is how the missing-shape bug cost us the variables.

        if (definition.publishVariables)
        {
            for (var name in keys(AOW_LAYOUT))
            {
                var c = AOW_LAYOUT[name];
                setVariable(context, "aow_" ~ name ~ "_x", c.pos[0]);
                setVariable(context, "aow_" ~ name ~ "_y", c.pos[1]);
                setVariable(context, "aow_" ~ name ~ "_z", c.pos[2]);
            }
        }

        if (definition.drawEnvelopes)
        {
            for (var name in keys(AOW_LAYOUT))
            {
                var c = AOW_LAYOUT[name];
                var subId = id + ("solid_" ~ name);

                // Dispatch on KNOWN shapes only. An unrecognised or missing
                // shape must draw nothing rather than fall through to code
                // that dereferences keys the entry does not have.
                if (c.shape == "box")
                {
                    fCuboid(context, subId, {
                            "corner1" : c.pos - c.size / 2,
                            "corner2" : c.pos + c.size / 2
                    });

                    // fCuboid is axis-aligned only, so an oriented box is
                    // built square and then rotated about its own centre.
                    // The axis and angle are precomputed by the generator.
                    if (c.rotAxis != undefined)
                    {
                        opTransform(context, id + ("rot_" ~ name), {
                                "bodies" : qCreatedBy(subId, EntityType.BODY),
                                "transform" : rotationAround(
                                        line(c.pos, c.rotAxis), c.rotDeg)
                        });
                    }
                }
                else if (c.shape == "cylinder" || c.shape == "capsule")
                {
                    // fCylinder, not opCylinder — solid primitives are the f*
                    // family. Capsules are drawn as plain cylinders: the end
                    // caps matter to the contact model, not to clearance.
                    var half = c.axis * c.length / 2;
                    fCylinder(context, subId, {
                            "topCenter" : c.pos + half,
                            "bottomCenter" : c.pos - half,
                            "radius" : c.radius
                    });
                }
                else
                {
                    continue;   // "point" — nothing solid to draw
                }

                // Without this every body lands in the list as "Part N".
                setProperty(context, {
                        "entities" : qCreatedBy(subId, EntityType.BODY),
                        "propertyType" : PropertyType.NAME,
                        "value" : name
                });
            }
        }

        if (definition.drawPoints)
        {
            for (var name in keys(AOW_LAYOUT))
            {
                var subId = id + ("point_" ~ name);
                opPoint(context, subId, { "point" : AOW_LAYOUT[name].pos });
                setProperty(context, {
                        "entities" : qCreatedBy(subId, EntityType.BODY),
                        "propertyType" : PropertyType.NAME,
                        "value" : name ~ "_origin"
                });
            }
        }

        if (definition.drawFourBarSketch && size(AOW_FOURBAR) > 0)
        {
            // A real sketch of construction lines. Cheap because the mechanism
            // is planar: one plane at the linkage's fore/aft station, normal
            // CAD +Y, and the 2D coordinates are just (CAD x, CAD z).
            // Normal is CAD -Y, not +Y. A sketch plane's local Y is
            // (normal CROSS xDir): with normal +Y that gives -Z and the whole
            // mechanism draws upside down. With -Y it gives +Z, so the emitted
            // (CAD x, CAD z) pairs mean what they say.
            var sk = newSketchOnPlane(context, id + "fourbar", {
                    "sketchPlane" : plane(
                            vector(0, 1, 0) * AOW_FOURBAR_STATION,
                            vector(0, -1, 0),
                            vector(1, 0, 0))
            });
            for (var seg in AOW_FOURBAR)
            {
                skLineSegment(sk, seg.name, {
                        "start" : seg.start,
                        "end" : seg.end,
                        "construction" : true
                });
            }
            for (var a in AOW_FOURBAR_ARCS)
            {
                // Three-point arc: the crank tip's swept path. Construction,
                // because it is a keep-out boundary rather than a part.
                skArc(sk, a.name, {
                        "start" : a.start,
                        "mid" : a.mid,
                        "end" : a.end,
                        "construction" : true
                });
            }
            skSolve(sk);
        }

        if (definition.drawAxisPlanes)
        {
            for (var name in keys(AOW_LAYOUT))
            {
                var c = AOW_LAYOUT[name];
                if (c.shape != "cylinder" && c.shape != "capsule")
                    continue;

                // Normal to the component axis, through its centre. For the
                // fork this is the plane perpendicular to the STEERING AXIS,
                // which is the one you want for the head tube and any clamp —
                // sketching those against a world plane is what puts the rake
                // in wrong.
                var subId = id + ("plane_" ~ name);
                opPlane(context, subId, {
                        "plane" : plane(c.pos, c.axis),
                        "width" : definition.planeSize,
                        "height" : definition.planeSize
                });
                // NOT NAMED, and it cannot be. Planes and mate connectors
                // carry no metadata — the UI derives their names from the
                // FEATURE that made them, and only a `cPlane`-named feature
                // can name a plane. So every plane a custom feature emits is
                // named after that feature, whatever setProperty does. Both
                // the filtered query (silently names nothing) and the
                // unfiltered one (throws, taking the plane with it) were
                // dead ends for the same underlying reason. If per-plane
                // names matter, that is an argument for separate feature
                // instances, not for more code here.
            }
        }
    });
"""

def render_featurescript(items: list[dict], params: dict, ver: str = "3044") -> str:
    axle_h = params["omni_wheel"]["outer_radius"]
    L = [FS_HEADER.format(ver=ver, axle=_mm(axle_h)), "", "export const AOW_LAYOUT = {"]
    entries = []
    for it in items:
        pos = to_cad_pos(it["pos"])
        # Fields are collected and joined rather than appended with trailing
        # commas: FeatureScript map literals do not accept a dangling comma
        # before the closing brace.
        f = [f'"group" : "{it["group"]}"',
             f'"pos" : vector({pos[0]}, {pos[1]}, {pos[2]}) * millimeter']
        if "box" in it:
            # A framed box is authored in servo-local (W, D, H) and rotated
            # into place; an unframed one keeps the model-frame extents.
            e = ([_mm(it["box"][1]), _mm(it["box"][0]), _mm(it["box"][2])]
                 if "frame" in it else to_cad_extent(it["box"]))
            f.append('"shape" : "box"')
            f.append(f'"size" : vector({e[0]}, {e[1]}, {e[2]}) * millimeter')
            rot = None
            if "frame" in it:
                w, d, h = it["frame"]
                # Box is built axis-aligned as (W, D, H) on CAD (x, y, z), then
                # rotated so those land on the servo's real local axes.
                rot = frame_rotation(to_cad_dir(w), to_cad_dir(d), to_cad_dir(h))
            elif "zaxis" in it:
                rot = rotation_from_z(to_cad_dir(it["zaxis"]))
            if True:
                if rot:
                    ax, ang = rot
                    f.append(f'"rotAxis" : vector({ax[0]}, {ax[1]}, {ax[2]})')
                    f.append(f'"rotDeg" : {ang} * degree')
        elif "cyl" in it or "cap" in it:
            r, ln, ax = it.get("cyl") or it["cap"]
            a = to_cad_dir(ax)
            axis = f"vector({a[0]}, {a[1]}, {a[2]})"
            f.append(f'"shape" : "{"cylinder" if "cyl" in it else "capsule"}"')
            f.append(f'"radius" : {_mm(r)} * millimeter')
            f.append(f'"length" : {_mm(ln)} * millimeter')
            f.append(f'"axis" : {axis}')
        else:
            # The wing pivots are bare points — no envelope. They still need a
            # `shape` key: FeatureScript returns undefined for a missing map
            # key, `undefined == "box"` is merely false, and the else branch
            # then dereferences c.axis/c.length/c.radius and kills the whole
            # feature regeneration — variables included.
            f.append('"shape" : "point"')
        if it.get("mass") is not None:
            f.append(f'"mass_g" : {_g(it["mass"])}')
        body = ",\n".join(f"        {line}" for line in f)
        entries.append(f'    "{it["name"]}" : {{\n{body}\n    }}')
    L.append(",\n".join(entries))
    L.append("};")

    # The four-bar is PLANAR — every node sits at one fore/aft station — so it
    # can be a real sketch of construction lines rather than a bundle of thin
    # rods. Sketch 2D coords are (CAD x, CAD z) on a plane whose normal is
    # CAD +Y, offset to that station.
    segs = [it for it in items if "seg" in it]
    if segs:
        y0 = to_cad_pos(segs[0]["seg"][0])[1]
        L.append("")
        L.append(f"export const AOW_FOURBAR_STATION = {y0} * millimeter;")
        L.append("export const AOW_FOURBAR = [")
        rows = []
        for it in segs:
            a, bb = (to_cad_pos(q) for q in it["seg"])
            rows.append(f'    {{ "name" : "{it["name"]}", '
                        f'"start" : vector({a[0]}, {a[2]}) * millimeter, '
                        f'"end" : vector({bb[0]}, {bb[2]}) * millimeter }}')
        L.append(",\n".join(rows))
        L.append("];")
        arcs = [it for it in items if "arc" in it]
        if arcs:
            L.append("")
            L.append("export const AOW_FOURBAR_ARCS = [")
            rows = []
            for it in arcs:
                a, m, e = (to_cad_pos(q) for q in it["arc"])
                rows.append(f'    {{ "name" : "{it["name"]}", '
                            f'"start" : vector({a[0]}, {a[2]}) * millimeter, '
                            f'"mid" : vector({m[0]}, {m[2]}) * millimeter, '
                            f'"end" : vector({e[0]}, {e[2]}) * millimeter }}')
            L.append(",\n".join(rows))
            L.append("];")
    L.append(FS_FEATURE)
    return "\n".join(L)


def _arc_radius(pts):
    """Radius of the circle through three points — a cheap self-check that the
    sampled sweep really is an arc about the crank shaft."""
    import numpy as _n
    a, b, c = (_n.asarray(q, float) for q in pts)
    ab, ac = b - a, c - a
    n = _n.cross(ab, ac)
    if _n.linalg.norm(n) < 1e-12:
        return 0.0
    return float(_n.linalg.norm(ab) * _n.linalg.norm(ac)
                 * _n.linalg.norm(b - c) / (2 * _n.linalg.norm(n)))


def _wrap(text: str, width: int) -> list[str]:
    import textwrap
    return textwrap.wrap(" ".join(text.split()), width)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=CAD_PARAMS,
                    help="defaults to the CAD scratch copy, NOT the "
                         "simulator's bike_params.yaml — see that file's header")
    ap.add_argument("-o", "--output", default=None)
    ap.add_argument("--format", choices=("yaml", "featurescript"), default="yaml")
    ap.add_argument("--righting", choices=("linkage", "wings", "none"),
                    default="linkage",
                    help="which self-righting mechanism to include; they are "
                         "alternatives and are never built together")
    ap.add_argument("--linkage-config", default=LINKAGE_CFG,
                    help=f"wing-linkage geometry (default {LINKAGE_CFG})")
    ap.add_argument("--chassis-box", action="store_true",
                    help="include the chassis inertia primitive (off by "
                         "default: it is not a physical part and the roof "
                         "already bounds that envelope)")
    ap.add_argument("--bumpers", action="store_true",
                    help="include the righting bumpers (off by default: they "
                         "are noise while the chassis is being drawn)")
    ap.add_argument("--fs-version", default="3044",
                    help="FeatureScript version header; must match the document")
    args = ap.parse_args()

    params, raw = load_params(args.params), load_sources(args.params)
    items = build(params, raw, args.righting, args.bumpers, args.chassis_box,
                  args.linkage_config)
    if args.format == "yaml":
        text, default_out = render(items, params, args.linkage_config), OUT
    else:
        text = render_featurescript(items, params, args.fs_version)
        default_out = OUT_FS
    out = Path(args.output or default_out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    st = next((i for i in items if i["name"] == "servo_steer"), None)
    if st:
        print(f"  steer servo box centre (derived) -> "
              f"servos.xc330_t181.pos: {[round(v, 6) for v in st['pos']]}")
    total = sum(i["mass"] for i in items if i.get("mass") and i["group"] != "righting")
    print(f"wrote {out}  ({len(items)} components)")
    print(f"  modelled mass excluding righting: {_g(total)} g")


if __name__ == "__main__":
    main()
