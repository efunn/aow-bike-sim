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


def _np_cross(a, b):
    """3-vector cross product, without dragging numpy into module scope."""
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _deg(r):
    """Radians -> degrees, rounded the way _mm rounds mm."""
    from math import degrees
    return round(degrees(r), 6)


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


def _add_linkage(add, add_horn, add_case_holes, params: dict, raw: dict, d3: dict,
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

    _ck = add("linkage_crank_servo", "righting", [px, 0.0, servo_z],
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

    add_case_holes("linkage_crank_servo", "righting", d3, _ck[1], _ck[2])

    add_horn("linkage_crank_horn", "righting", "linkage_crank_servo", d3,
             [px, 0.0, servo_z],
             note="BOTH crank arms are keyed to this one horn, at "
                  "deliberately unequal angles — so it is the part that will "
                  "actually be replaced by something custom, and the disc is "
                  "only standing in for its hub.")

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
            holes=None, normal=None, source=None, note=None):
        solved = None
        if mount:
            _nm, _spec, *_a = mount
            _sp, pos, frame = mount_of(_nm, _spec, pos,
                                       _a[0] if _a else "centre")
            solved = (_sp, pos, frame)
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
        if holes:
            d["holes"] = holes
        if normal is not None:
            d["normal"] = normal
        if note:
            d["note"] = note
        items.append(d)
        # Callers that let `add` do the mount solve get the result back rather
        # than having to redo it; `_add_linkage` needs the case frame for the
        # hole pattern and has no access to `mount_of`.
        return solved

    def mount_of(name, spec, pos, anchor="centre"):
        """(shaft_point, box_centre, local axes) for one installed servo.

        Local frame: +D is the shaft axis (horn faces +D), +H is `body_up`,
        +W completes it.

        The shaft point is the MOUNTING DATUM: the outer face of the horn, the
        surface a pulley or a bracket actually bolts to. `box_size` D is the
        CASE alone, so that datum sits D/2 out from the case centre less
        `shaft_from_horn_face` — which is NEGATIVE, equal to -horn_thickness,
        because the datum stands proud of the case rather than inside it. The
        axis is also (H/2 - shaft_from_end) toward +H. Every one of those
        numbers comes off the ROBOTIS drawings.
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

    def add_horn(name, group, mount_name, spec, shaft_pt, note=""):
        """The output horn as a disc on the horn face, facing away from the case.

        A ROUGH ENVELOPE and nothing more — the real part is splined and
        scalloped, and the ROBOTIS STEP carries that into the CAD. What it is
        here for is the two millimetres of depth it adds beyond the case, which
        is the part that decides whether something fits.

        The axis is read from `cad_mounts` exactly as `mount_of` reads it, so
        the disc cannot drift out of agreement with the case it sits on —
        including for the steer servo, whose `shaft_axis` this file overwrites
        at runtime with the raked steering axis.
        """
        import numpy as _n
        t, dia = spec.get("horn_thickness"), spec.get("horn_diameter")
        if not t or not dia:
            return              # a servo whose horn has not been measured
        mt = params.get("cad_mounts", {}).get(mount_name, {})
        d_hat = _n.asarray(mt.get("shaft_axis", [0, 1, 0]), float)
        d_hat = d_hat / _n.linalg.norm(d_hat) * mt.get("horn_dir", 1.0)
        # The disc fills the gap between the case face and the datum, so it
        # runs INWARD from the shaft point in every case — the datum being the
        # horn's outer face is what makes that one rule instead of a per-model
        # flag. ROBOTIS is not consistent about whether the depth it quotes
        # includes the horn (the XC330's 26 does, the XC430's 34 does not), but
        # `box_size` now carries the case alone for both, so the ambiguity is
        # spent at the config and not here.
        sp = _n.asarray(shaft_pt, float)
        add(name, group, [float(v) for v in sp - d_hat * t / 2],
            cyl=(dia / 2, t, tuple(float(v) for v in d_hat)),
            source={"pos": "derived — under the mounting datum",
                    "size": "measured (ROBOTIS drawing)"},
            note=f"Output horn, {_mm(dia)} mm across and {_mm(t)} mm thick, "
                 f"drawn as a plain disc and nothing more. It fills the "
                 f"{_mm(t)} mm between the case face and the MOUNTING DATUM, "
                 f"which is the point of the same name — so the real horn face "
                 f"lands on that plane and a ROBOTIS STEP dropped onto it is "
                 f"in the right place. NO MASS: the datasheet servo figure is "
                 f"taken to be the assembled unit including the horn. " + note)

        # The centre boss, where the drawing dimensions one. It is the only
        # thing on this servo that sticks out PAST the mounting datum, which
        # makes it the thing a flat-faced part has to counterbore for.
        bp, bd = spec.get("boss_projection"), spec.get("boss_diameter")
        if bp and bd:
            add(name + "_boss", group, [float(v) for v in sp + d_hat * bp / 2],
                cyl=(bd / 2, bp, tuple(float(v) for v in d_hat)),
                source={"pos": "derived — proud of the mounting datum",
                        "size": "measured (ROBOTIS drawing)"},
                note=f"Centre boss, {_mm(bd)} mm across, standing {_mm(bp)} mm "
                     f"PAST the horn face. Anything bolted flat to that face "
                     f"has to clear it — counterbore, or sit on the boss "
                     f"instead. Drawn because it is the one feature of the "
                     f"servo that the mounting datum does not bound.")


    def add_case_holes(name, group, spec, centre, axes):
        """The four case holes on each of the two horn-axis faces.

        A REFERENCE, not an envelope: nothing is drawn solid and nothing here
        claims what the holes are for. The pattern is centred on the FACE, not
        on the shaft axis — the axis is `shaft_from_end` off that centre — so
        emitting it from the case frame is the only way to get it right without
        a sign error per servo.

        Both faces, because which one is reachable is exactly the question the
        mount has to answer, and it differs per servo: the drive servos' horn
        faces are under a pulley and their back faces are under the OTHER
        servo's pulley, while the steer servo's horn face is clear.
        """
        import numpy as _n
        pat = spec.get("case_hole_pattern")
        if not pat:
            return              # a servo whose face pattern is not measured
        w_hat, d_hat, h_hat = (_n.asarray(a, float) for a in axes)
        a, b = pat[0] / 2, pat[1] / 2
        depth = spec["box_size"][0]
        for tag, sgn in (("horn", 1.0), ("back", -1.0)):
            fc = _n.asarray(centre, float) + d_hat * sgn * depth / 2
            pts = [fc + w_hat * sw * a + h_hat * sh * b
                   for sw in (1.0, -1.0) for sh in (1.0, -1.0)]
            add(f"{name}_case_holes_{tag}", group,
                [float(v) for v in fc],
                holes=[[float(v) for v in q] for q in pts],
                source={"pos": "derived — case frame",
                        "size": "measured (ROBOTIS drawing)"},
                note=f"Case hole pattern, {_mm(pat[0])} x {_mm(pat[1])} mm "
                     f"centred on the {tag}-side face — four reference points, "
                     f"nothing solid. The shaft axis is NOT the centre of it: "
                     f"the axis sits {_mm(spec['box_size'][2] / 2 - spec['shaft_from_end'])} "
                     f"mm off along body_up, so the holes land asymmetrically "
                     f"about it.")


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

    # THE BELT PLANE IS DERIVED, from whichever of two clearances binds.
    #
    #   wheel  the pulley flange has to miss the rear wheel;
    #   servo  the two cases sit SYMMETRIC about the centreline — which is what
    #          lets one flat plate touch both, since they face opposite ways —
    #          and the plate outboard of them has to miss the pulley.
    #
    # The second one is the new constraint and it is currently the binding one.
    # Writing it as a max rather than a number means that if the wheel ever
    # gets wider, or the plate thinner, the plane follows instead of going
    # quietly wrong.
    d4 = sv["xc430_w150"]
    plate = dt.get("drive_mount_plate", 0.0)
    # A standoff the plate forces, and a standoff the pulley's own hub already
    # provides, are the same room — so take the larger, do not add them.
    gap = dt.get("drive_mount_gap", 0.0)
    hub = max(dt.get("pulley_hub_offset", 0.0),
              plate + gap - d4["horn_thickness"], 0.0)
    plane_wheel = ow["width"] / 2 + dt["wheel_clearance"] + pw / 2
    plane_servo = d4["box_size"][0] / 2 + d4["horn_thickness"] + hub + pw / 2
    plane = max(plane_wheel, plane_servo)
    d_in = be["teeth_input"] * be["pitch"] / _np.pi
    d_sv = be["teeth_servo"] * be["pitch"] / _np.pi
    # L = 2C + (pi/2)(D1+D2) + (D1-D2)^2/4C, solved for C.
    _a, _b, _c = 2.0, (_np.pi / 2) * (d_sv + d_in) - be["length"], (d_sv - d_in) ** 2 / 4
    C = float((-_b + _np.sqrt(_b * _b - 4 * _a * _c)) / (2 * _a))
    C_m = C
    am = dt["axle_mount"]
    # OFF THE PLANE, not off the wheel chain. It used to re-derive the pulley's
    # outer face from wheel_clearance, which was the same number while the
    # plane was wheel-driven and silently wrong the moment it stopped being.
    am_y = plane + pw / 2 + am["width"] / 2

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
        #
        # MINUS the standoff, not plus, and in metres, not millimetres. A
        # standoff holds the horn face further INBOARD of the pulley's inner
        # face; the old expression moved it outboard, INTO the pulley, and
        # scaled it by 1000 into the bargain. Both were invisible at zero.
        horn_y = plane - pw / 2 - hub
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
        add_case_holes(nm, "servos", d4, centre, axes)
        add_horn(f"{nm}_horn", "servos", nm, d4, sp,
                 note="The driving pulley bolts ONTO it and is far larger "
                      "(71.6 mm pitch dia against 20.5 mm), so this disc sits "
                      "wholly inside the pulley envelope and adds nothing to "
                      "the drivetrain keep-out. It matters for the MOUNT, "
                      "which has to clear it.")
        add(f"{nm}_shaft", "servos", sp,
            source={"pos": "datasheet offsets + cad_mounts"},
            note=f"Output shaft, ON THE HORN FACE — the MOUNTING DATUM, so "
                 f"mate the ROBOTIS model's horn face to this point: "
                 f"{_mm(d4['box_size'][0] / 2 - d4['shaft_from_horn_face'])} "
                 f"mm out along the axis and "
                 f"{_mm(d4['box_size'][2] / 2 - d4['shaft_from_end'])} mm off "
                 f"the case centre along body_up. Mate the ROBOTIS model here. "
                 f"Cables exit the OPPOSITE face — keep routing room there.")
    if plate > 0:
        # A PLATE ON EACH SIDE. Each lateral side of the bike presents one
        # servo's horn face and the other's back face, so one plane per side
        # touches both cases — and the second plate is FREE, because the belt
        # plane was already pushed out to clear the first and the geometry is
        # symmetric. Eight M2.5 per side, sixteen in all, and the load path
        # into the frame stops being one-sided.
        #
        # Their OUTLINE is a placeholder — a rectangle sized to the collar — but
        # their two faces are not: the inboard one lies on the case faces and
        # the outboard one is what the pulley stands off from.
        _half = d4["box_size"][0] / 2
        _wall = dt.get("drive_mount_wall", 0.003)
        # The cavity is the case pair plus a fit clearance; the walls sit
        # outside that. One place for the clearance means the plates, sized to
        # the collar's outer footprint, follow it too.
        _cav = dt.get("drive_mount_cavity_clearance", 0.0)
        _cav_t = C_m * _np.sin(dth) + d4["box_size"][1] / 2 + _cav
        _cav_r = d4["box_size"][2] / 2 + _cav
        _t0 = _cav_t + _wall
        _rc = C_m - (d4["box_size"][2] / 2 - d4["shaft_from_end"])
        _rel = dt.get("drive_mount_relief", 0.0)
        # `sgn` is the MODEL y sign; CAD x is its negation, so sgn -1 is the
        # bike's right. Each plate is relieved for the servo whose HORN faces
        # it, which is the one on that same side.
        for tag, sgn in (("right", -1.0), ("left", 1.0)):
            add(f"drive_mount_plate_{tag}", "mount",
                [_rc * _np.cos(th0), sgn * (_half + plate / 2),
                 _rc * _np.sin(th0)],
                box=(2 * _t0, plate, 2 * (_cav_r + _wall)),
                zaxis=[_np.cos(th0), 0.0, _np.sin(th0)],
                source={"pos": "derived — on the case faces",
                        "size": "design (outline is a placeholder)"},
                note=f"{_mm(plate)} mm, on the bike's {tag}. It lies on the "
                     f"{tag} servo's horn-side face and the other servo's back "
                     f"face — they face opposite ways, so one plane touches "
                     f"both — and takes 8 M2.5 machine screws on the 22 x 40 "
                     f"case patterns. The cases are symmetric about the "
                     f"centreline at +/-{_mm(_half)} mm precisely so this can "
                     f"be flat rather than stepped, and that symmetry is what "
                     f"forced the belt plane out to {_mm(plane)} mm.")

            if _rel > 0:
                # A VOID, not a part: the hole the plate needs where the horn
                # passes through it. Drawn as a solid because the layout has no
                # boolean, and named so nobody mistakes it for material.
                _hd = d4["horn_diameter"] + 2 * _rel
                _th = servo_angle[tag]
                add(f"drive_mount_relief_{tag}", "mount",
                    [C_m * _np.cos(_th), sgn * (_half + plate / 2),
                     C_m * _np.sin(_th)],
                    cyl=(_hd / 2, plate, AXIS_LATERAL),
                    source={"pos": "derived — on the horn axis",
                            "size": "design"},
                    note=f"A VOID. The {_mm(_hd)} mm hole the {tag} plate needs "
                         f"where the {tag} servo's horn passes through it — "
                         f"{_mm(d4['horn_diameter'])} mm horn plus "
                         f"{_mm(_rel)} mm of radial clearance. Concentric with "
                         f"the SHAFT, not with the plate. Only the servo whose "
                         f"horn faces this plate needs one; the other presents "
                         f"its flat back face.")

        # THE SLEEVE: a collar joining the two plates, running along the four
        # sides of the case pair. Walls rather than one solid, because that is
        # what it is — the servos are captured by shape and the sixteen screws
        # only have to stop them drifting along their own shafts.
        #
        # It stops at the case faces, |x| = D/2, well inside the pulley faces,
        # so nothing about it is near a belt.
        if _wall > 0:
            r_half = _cav_r
            # All four, less whichever is the ceiling. The build axis is
            # TANGENTIAL — across the two servos — so it is a SIDE wall that
            # comes off and both radial walls stand vertical. Getting this
            # backwards costs a wall in the wrong place, so it is named in the
            # config rather than inferred.
            walls = [("side_a", _cav_t + _wall / 2, 0.0, _wall,
                      2 * (r_half + _wall)),
                     ("side_b", -(_cav_t + _wall / 2), 0.0, _wall,
                      2 * (r_half + _wall)),
                     ("radial_in", 0.0, -(r_half + _wall / 2), 2 * _t0, _wall),
                     ("radial_out", 0.0, r_half + _wall / 2, 2 * _t0, _wall)]
            _open = dt.get("drive_mount_open_wall", "side_b")
            walls = [w for w in walls if w[0] != _open]
            for nm_, off_t, off_r, ext_t, ext_r in walls:
                cu, su = _np.cos(th0), _np.sin(th0)
                cx = _rc * cu + off_r * cu - off_t * su
                cz = _rc * su + off_r * su + off_t * cu
                # Extents are model-frame (x, y, z) BEFORE the zaxis rotation,
                # which is about model Y and so leaves the lateral one alone:
                # tangential, lateral, radial.
                add(f"drive_mount_{nm_}", "mount", [cx, 0.0, cz],
                    box=(ext_t, d4["box_size"][0], ext_r),
                    zaxis=[cu, 0.0, su],
                    source={"pos": "derived — around the case pair",
                            "size": "design"},
                    note=f"Sleeve wall, {_mm(_wall)} mm, joining the two "
                         f"plates. The servos share a face, so the pair "
                         f"presents ONE rectangle and it takes four sides, not "
                         f"eight — of which one is left open, being the "
                         f"ceiling in the print. Torque goes "
                         f"in here as bearing on the case walls, which is why "
                         f"the screws only have to retain.")

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

    add_case_holes("servo_steer", "servos", d3, c_st, ax_st)
    add_horn("servo_steer_horn", "servos", "servo_steer", d3, sp_st,
             note="Faces DOWN the steering axis toward the front axle, so its "
                  "thickness comes straight off the fork clearance.")
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
    # ---- print planes ------------------------------------------------
    #
    # THE THREE PLANES CAD STARTS FROM. Not envelopes and not derived from any
    # part — they are the orientations the printed parts will be laid down in,
    # and they are here because getting them from the geometry beats eyeballing
    # them off a model.
    #
    # `normal` is a model-frame unit vector; the FeatureScript draws each as a
    # construction plane through `pos`. Only the ORIENTATION is meant to be
    # authoritative — offset the plane in Onshape to wherever the part sits.
    # The plane CONTAINS the steering axis and the axle direction, so its
    # normal is the cross product of the two: mostly forward, tilted up by the
    # rake. Not the plane PERPENDICULAR to the steering axis, which is a
    # different thing and was wrong here twice.
    _fn = _np_cross(fork_axis, AXIS_LATERAL)
    add("plane_fork_print", "planes", [b["wheelbase"], 0.0, front_z],
        normal=[-v for v in _fn],
        source={"pos": "derived — front axle", "size": "design"},
        note=f"THE FORK'S OWN PLANE, and the one to both sketch and print it "
             f"in. Looking down its normal you see the fork from the FRONT: "
             f"two legs coming down left and right and meeting at the top. It "
             f"holds the axle direction and the steering axis at once, so it "
             f"is the front view tilted back by the {_deg(rake)} deg of rake — "
             f"which is why no world plane will do. Sketch the profile here "
             f"and extrude fore-aft; print it lying in this plane and both "
             f"legs sit flat on the bed with the layers running along them.")

    def _belt_tangent(c2, r1, r2, side):
        """External tangent of two circles, in the model (x, z) plane.

        `side` +1 takes the tangent anticlockwise of the centre line, -1 the
        clockwise one — the upper and lower runs of the belt as seen from the
        bike's left.
        """
        d = _np.hypot(c2[0], c2[1])
        a = _np.arctan2(c2[1], c2[0])
        bb = _np.arcsin((r2 - r1) / d)
        th = a + side * (_np.pi / 2 + bb)
        u_ = _np.array([_np.cos(th), _np.sin(th)])
        p1 = r1 * u_
        p2 = _np.asarray(c2, float) + r2 * u_
        return p1, p2

    r_in_env = d_in / 2 + be["flange_margin"] / 2
    r_sv_env = d_sv / 2 + be["flange_margin"] / 2
    # ALL FOUR RUNS, because THE TWO SIDES ARE NOT MIRROR IMAGES. The servos
    # straddle 45 deg rather than sharing it, so the left belt spans one pair of
    # angles and the right another — overlapping, but offset. Drawing only the
    # two extremes made the rear look symmetric, and anything threading between
    # the belts has two different corridors to satisfy.
    for tag, sgn, side, word in (("left", 1, -1, "lower"),
                                 ("left", 1, 1, "upper"),
                                 ("right", -1, -1, "lower"),
                                 ("right", -1, 1, "upper")):
        _t = servo_angle[tag]
        c2 = (C * _np.cos(_t), C * _np.sin(_t))
        p1, p2 = _belt_tangent(c2, r_in_env, r_sv_env, side)
        run = p2 - p1
        run = run / _np.linalg.norm(run)
        # Normal to the belt run, in the same plane. The plane therefore holds
        # the run AND the axle direction, which is the sheet a dropout is.
        nrm = [-run[1], 0.0, run[0]]
        mid = (p1 + p2) / 2
        add(f"plane_belt_{tag}_{word}", "planes",
            [float(mid[0]), sgn * plane, float(mid[1])], normal=nrm,
            source={"pos": "derived — belt tangent midpoint",
                    "size": "design"},
            note=f"CLEARANCE, not a print plane. Parallel to the "
                 f"{word.upper()} belt run — the {tag} servo's {word} tangent, "
                 f"one of the two extreme angles the belts reach — and "
                 f"therefore part of the boundary of the belt-and-pulley hull. "
                 f"A rear dropout has to run from the axle OUTBOARD of its own "
                 f"belt and end up INBOARD of it at the servo mount, so "
                 f"somewhere it crosses the belt plane, and it can only do "
                 f"that outside this line. Like a chainstay threading past the "
                 f"chain. Which means the crossing dictates the part's shape "
                 f"and a single flat build plane will not align with both the "
                 f"sleeve and the arm — printing off this was the first guess "
                 f"and it does not survive the geometry. Placed in the belt "
                 f"plane at {_mm(plane)} mm; the ORIENTATION is what is "
                 f"derived, not the offset.")

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
        _add_linkage(add, add_horn, add_case_holes, params, raw, d3,
                     bumpers, linkage_cfg)
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
    headings["mount"] = "Drive-servo mount — plate and sleeve. PROPOSAL."
    headings["planes"] = ("Sketch planes CAD starts from. Orientations, not "
                          "parts.")
    # Ordered where an order is known, then WHATEVER ELSE EXISTS. The fixed
    # tuple used to be the whole list, so a new group rendered nowhere and the
    # only symptom was a component count that did not match the file.
    known = ("frame", "drivetrain", "steering", "servos", "mount", "planes",
             "electronics", "righting")
    for grp in known + tuple(g for g in groups if g not in known):
        if grp not in groups:
            continue
        headings.setdefault(grp, grp)
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
            elif "normal" in it:
                n = to_cad_dir(it["normal"])
                L.append("    shape: plane")
                L.append(f"    normal: {n}   # unit vector, CAD frame")
            elif "holes" in it:
                L.append("    shape: holes")
                L.append("    holes_mm:")
                for q in it["holes"]:
                    c = to_cad_pos(q)
                    L.append(f"      - [{c[0]}, {c[1]}, {c[2]}]")
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
# PARTLY TESTED AGAINST ONSHAPE. The DATA below is generated from the same
# source as the YAML and is the valuable half; the drawing code that consumes
# it was written from the FeatureScript docs. Everything except the query
# variables has been through a real document. If a call needs fixing, fix it
# and regenerate — the data block is machine-written and will not be disturbed.
#
# What the studio gives a Part Studio, in the order to insert them:
#   1. `AOW layout variables` — `setVariable` for every coordinate, so sketches
#      reference `#aow_servo_drive_left_y` and stay linked to the sim's numbers;
#   2. one feature PER GROUP — envelopes, origin points and axis planes for
#      that group alone, so the node can be renamed and suppressed on its own.
#      `AOW mount` is the drive-servo plate and sleeve; `AOW planes` is the
#      three print planes and carries no tickboxes, everything in it being a
#      plane already;
#   3. `AOW four-bar sketch` — the righting linkage as construction geometry;
#   4. `AOW bike layout` — the superseded all-in-one node, kept only so
#      documents that already have it inserted do not lose their geometry.

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
//
// This studio defines SEVERAL features; they all show up under Custom features
// once it is committed. Insert `AOW layout variables` once and first, then one
// `AOW <group>` per group you are working on, and RENAME each node — that name
// is the only handle Onshape will give you on the planes it draws.
"""

# --- the emitted FeatureScript ---------------------------------------------
#
# ONE Feature Studio, SEVERAL features. The original `aowBikeLayout` drew
# everything under a single node with five checkboxes, which is the only shape
# a Part Studio tree can take from one feature. Splitting it buys three things
# a checkbox cannot: a tree node per group that can be RENAMED (Onshape derives
# the name of a plane from the feature that made it, and refuses `setProperty`
# on one), independent suppression while packing one group, and a smaller blast
# radius when a runtime error aborts a feature.
#
# `aowBikeLayout` stays, delegating to the same helpers at the same sub-ids, so
# a Part Studio that already has it inserted keeps its geometry and every
# downstream reference to it. Deleting it would have orphaned them.
#
# Lines marked @QV@ publish QUERY VARIABLES — a named selection (`#aow_q_fork`)
# that downstream features can consume in a selection field, which is the one
# mechanism that survives geometry it points at being regenerated. They are
# emitted unless --no-query-vars. `setQueryVariable` arrived in release 1.203;
# an older document will fail to COMPILE THE WHOLE STUDIO on it, since an
# unknown function is a compile error and not a runtime one -- regenerate with
# --no-query-vars if that happens.

FS_HELPERS = '''
export const AOW_PLANE_BOUNDS =
{
    (meter)      : [1e-5, 0.06, 500],
    (centimeter) : 6.0,
    (millimeter) : 60.0,
    (inch)       : 2.5,
    (foot)       : 0.2,
    (yard)       : 0.07
} as LengthBoundSpec;

// ---------------------------------------------------------------------------
// Shared drawing helpers
// ---------------------------------------------------------------------------
// Every feature below is a thin wrapper around these. Sharing them is not just
// about duplication: each component is drawn at the SAME sub-id whichever
// feature draws it, and an Onshape entity id is a deterministic function of
// the id of the operation that made it. Keying sub-ids by component NAME
// rather than by a loop counter is what makes a regeneration safe — adding or
// removing a component leaves every other component's ids untouched.

export function aowEnvelope(context is Context, id is Id, name is string)
{
    var c = AOW_LAYOUT[name];
    var subId = id + ("solid_" ~ name);

    // Dispatch on KNOWN shapes only. An unrecognised or missing shape must
    // draw nothing rather than fall through to code that dereferences keys the
    // entry does not have.
    if (c.shape == "box")
    {
        fCuboid(context, subId, {
                "corner1" : c.pos - c.size / 2,
                "corner2" : c.pos + c.size / 2
        });

        // fCuboid is axis-aligned only, so an oriented box is built square and
        // then rotated about its own centre. The axis and angle are
        // precomputed by the generator.
        if (c.rotAxis != undefined)
        {
            opTransform(context, id + ("rot_" ~ name), {
                    "bodies" : qCreatedBy(subId, EntityType.BODY),
                    "transform" : rotationAround(line(c.pos, c.rotAxis), c.rotDeg)
            });
        }
    }
    else if (c.shape == "cylinder" || c.shape == "capsule")
    {
        // fCylinder, not opCylinder — solid primitives are the f* family.
        // Capsules are drawn as plain cylinders: the end caps matter to the
        // contact model, not to clearance.
        var half = c.axis * c.length / 2;
        fCylinder(context, subId, {
                "topCenter" : c.pos + half,
                "bottomCenter" : c.pos - half,
                "radius" : c.radius
        });
    }
    else
    {
        return;     // "point" — nothing solid to draw
    }

    // Without this every body lands in the list as "Part N". Note that a name
    // the USER has since edited by hand can never be overwritten from
    // FeatureScript again — reset it under part > properties if you want the
    // generated name back.
    setProperty(context, {
            "entities" : qCreatedBy(subId, EntityType.BODY),
            "propertyType" : PropertyType.NAME,
            "value" : name
    });
@QV@    setQueryVariable(context, "aow_q_" ~ name, qCreatedBy(subId, EntityType.BODY));
}

export function aowPoint(context is Context, id is Id, name is string)
{
    var c = AOW_LAYOUT[name];

    // A hole pattern is one entry carrying several positions. Each gets its
    // own sub-id keyed by INDEX — safe here, unlike a loop counter over the
    // whole layout, because the four corners of a rectangle cannot be
    // reordered or added to without the pattern itself changing.
    if (c.points != undefined)
    {
        for (var i = 0; i < size(c.points); i += 1)
        {
            var holeId = id + ("hole_" ~ name ~ "_" ~ i);
            opPoint(context, holeId, { "point" : c.points[i] });
            setProperty(context, {
                    "entities" : qCreatedBy(holeId, EntityType.BODY),
                    "propertyType" : PropertyType.NAME,
                    "value" : name ~ "_" ~ i
            });
        }
@QV@        setQueryVariable(context, "aow_q_" ~ name, qCreatedBy(id, EntityType.BODY));
        return;
    }

    var subId = id + ("point_" ~ name);
    opPoint(context, subId, { "point" : c.pos });
    setProperty(context, {
            "entities" : qCreatedBy(subId, EntityType.BODY),
            "propertyType" : PropertyType.NAME,
            "value" : name ~ "_origin"
    });
@QV@    setQueryVariable(context, "aow_q_" ~ name ~ "_point", qCreatedBy(subId, EntityType.BODY));
}

export function aowAxisPlane(context is Context, id is Id, name is string,
        size is ValueWithUnits)
{
    var c = AOW_LAYOUT[name];

    // Two kinds reach here. A cylinder or capsule gets a plane normal to its
    // own axis — for the fork that is the plane perpendicular to the STEERING
    // AXIS, which is the one you want for the head tube and any clamp, since
    // sketching those against a world plane is what puts the rake in wrong.
    // A "plane" entry carries its normal directly; those are the print planes
    // and they are not derived from any part.
    var nrm;
    if (c.shape == "cylinder" || c.shape == "capsule")
        nrm = c.axis;
    else if (c.shape == "plane")
        nrm = c.normal;
    else
        return;

    var subId = id + ("plane_" ~ name);
    opPlane(context, subId, {
            "plane" : plane(c.pos, nrm),
            "width" : size,
            "height" : size
    });
    // The plane is NOT NAMED, and it cannot be. Planes and mate connectors
    // carry no metadata — the UI derives their names from the FEATURE that
    // made them, and that derivation is hardcoded to the feature type literally
    // called `cPlane`. Both the filtered query (silently names nothing) and the
    // unfiltered one (throws, taking the plane with it) were dead ends for the
    // same underlying reason. So a plane is always "Plane N" under whatever
    // feature drew it. What works instead is the query variable below: name
    // the REFERENCE rather than the plane, and a downstream sketch picks
    // `#aow_q_fork_plane` out of its plane field without anyone clicking a
    // "Plane 9" that a regeneration might renumber.
@QV@    setQueryVariable(context, "aow_q_" ~ name ~ "_plane", qCreatedBy(subId));
}

export function aowFourBar(context is Context, id is Id)
{
    if (size(AOW_FOURBAR) == 0)
        return;     // built with --righting none

    // A real sketch of construction lines. Cheap because the mechanism is
    // planar: one plane at the linkage's fore/aft station and the 2D
    // coordinates are just (CAD x, CAD z).
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
        // Three-point arc: the crank tip's swept path. Construction, because
        // it is a keep-out boundary rather than a part.
        skArc(sk, a.name, {
                "start" : a.start,
                "mid" : a.mid,
                "end" : a.end,
                "construction" : true
        });
    }
    skSolve(sk);
@QV@    setQueryVariable(context, "aow_q_fourbar", qCreatedBy(id + "fourbar", EntityType.EDGE));
}

export function aowDrawGroup(context is Context, id is Id, definition is map,
        group is string)
{
    for (var name in keys(AOW_LAYOUT))
    {
        if (AOW_LAYOUT[name].group != group)
            continue;
        if (definition.drawEnvelopes)
            aowEnvelope(context, id, name);
        if (definition.drawPoints)
            aowPoint(context, id, name);
        if (definition.drawAxisPlanes)
            aowAxisPlane(context, id, name, definition.planeSize);
    }
}

// Shared dialog for every per-group feature, so they stay identical without
// the generator emitting the same annotations once per group.
export predicate aowGroupPredicate(definition is map)
{
    annotation { "Name" : "Draw envelopes", "Default" : true }
    definition.drawEnvelopes is boolean;

    annotation { "Name" : "Draw origin points" }
    definition.drawPoints is boolean;

    annotation { "Name" : "Draw axis planes" }
    definition.drawAxisPlanes is boolean;

    if (definition.drawAxisPlanes)
    {
        annotation { "Name" : "Plane size" }
        isLength(definition.planeSize, AOW_PLANE_BOUNDS);
    }
}
'''

FS_VARIABLES = '''
// ---------------------------------------------------------------------------
// Features
// ---------------------------------------------------------------------------
// Insert `AOW layout variables` FIRST and once. It draws nothing, cannot fail,
// and is what makes `#aow_servo_steer_z` resolve in every sketch below it.

annotation { "Feature Type Name" : "AOW layout variables" }
export const aowLayoutVariables = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        for (var name in keys(AOW_LAYOUT))
        {
            var c = AOW_LAYOUT[name];
            setVariable(context, "aow_" ~ name ~ "_x", c.pos[0]);
            setVariable(context, "aow_" ~ name ~ "_y", c.pos[1]);
            setVariable(context, "aow_" ~ name ~ "_z", c.pos[2]);
        }
        setVariable(context, "aow_fourbar_station", AOW_FOURBAR_STATION);
    });
'''

FS_GROUP = '''
annotation { "Feature Type Name" : "AOW @TITLE@" }
export const @IDENT@ = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        aowGroupPredicate(definition);
    }
    {
        aowDrawGroup(context, id, definition, "@GROUP@");
    });
'''

FS_PLANES_GROUP = '''
annotation { "Feature Type Name" : "AOW planes" }
export const aowGroupPlanes = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Plane size" }
        isLength(definition.planeSize, AOW_PLANE_BOUNDS);
    }
    {
        // No checkboxes. Everything in this group IS a plane, so the three the
        // other groups carry would read "draw nothing", "draw nothing" and
        // "draw the only thing there is" — insert it and it works. The dialog
        // is synthesised rather than read off the definition so that
        // `aowDrawGroup` stays the single code path.
        aowDrawGroup(context, id, {
                "drawEnvelopes" : false,
                "drawPoints" : false,
                "drawAxisPlanes" : true,
                "planeSize" : definition.planeSize
        }, "planes");
    });
'''

FS_FOURBAR = '''
annotation { "Feature Type Name" : "AOW four-bar sketch" }
export const aowFourBarSketch = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        aowFourBar(context, id);
    });
'''

FS_LEGACY = '''
// ---------------------------------------------------------------------------
// The original single-node feature. SUPERSEDED by the features above, and kept
// only because a Part Studio that already has it inserted would lose all of
// its geometry — and every downstream reference into that geometry — the
// moment this feature type stopped existing. It delegates to the same helpers
// at the same sub-ids, so every entity id it produces is the one it produced
// before and nothing downstream of it moves. Only the ORDER of creation
// changed (interleaved per component rather than all solids, then all
// points), which the part list shows and nothing else depends on.
// Prefer the per-group features for new work; they can be named.

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
        isLength(definition.planeSize, AOW_PLANE_BOUNDS);
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

        for (var name in keys(AOW_LAYOUT))
        {
            if (definition.drawEnvelopes)
                aowEnvelope(context, id, name);
            if (definition.drawPoints)
                aowPoint(context, id, name);
            if (definition.drawAxisPlanes)
                aowAxisPlane(context, id, name, definition.planeSize);
        }

        if (definition.drawFourBarSketch)
            aowFourBar(context, id);
    });
'''


def _fs_ident(group: str) -> str:
    """`righting` -> `aowGroupRighting`. Prefixed rather than bare so a group
    can never collide with a helper name, and sanitised because a FeatureScript
    identifier is not allowed the characters a YAML key is."""
    parts = [p for p in "".join(
        ch if ch.isalnum() else " " for ch in group).split()]
    return "aowGroup" + "".join(p[:1].upper() + p[1:] for p in parts)


def _fs_qv(text: str, query_vars: bool) -> str:
    """Resolve the @QV@ marker: keep the line without it, or drop the line."""
    out = []
    for ln in text.split("\n"):
        if "@QV@" not in ln:
            out.append(ln)
        elif query_vars:
            out.append(ln.replace("@QV@", ""))
    return "\n".join(out)

def render_featurescript(items: list[dict], params: dict, ver: str = "3044",
                         query_vars: bool = True) -> str:
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
        elif "normal" in it:
            n = to_cad_dir(it["normal"])
            f.append('"shape" : "plane"')
            f.append(f'"normal" : vector({n[0]}, {n[1]}, {n[2]})')
        elif "normal" in it:
            n = to_cad_dir(it["normal"])
            f.append('"shape" : "plane"')
            f.append(f'"normal" : vector({n[0]}, {n[1]}, {n[2]})')
        elif "holes" in it:
            # A hole PATTERN travels as one entry with a list of positions
            # rather than four entries, so eight of these do not bury the
            # layout. The feature draws a named point per position.
            f.append('"shape" : "holes"')
            pts = []
            for q in it["holes"]:
                c = to_cad_pos(q)
                pts.append(f"vector({c[0]}, {c[1]}, {c[2]}) * millimeter")
            f.append('"points" : [' + ", ".join(pts) + ']')
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
    # Emitted even when EMPTY (--righting none). The features reference these
    # constants unconditionally, and in FeatureScript an undefined symbol is a
    # compile error that takes down the whole Feature Studio, not a runtime one
    # that could be guarded at the call site.
    segs = [it for it in items if "seg" in it]
    arcs = [it for it in items if "arc" in it]
    y0 = to_cad_pos(segs[0]["seg"][0])[1] if segs else 0.0
    L.append("")
    L.append(f"export const AOW_FOURBAR_STATION = {y0} * millimeter;")
    rows = []
    for it in segs:
        a, bb = (to_cad_pos(q) for q in it["seg"])
        rows.append(f'    {{ "name" : "{it["name"]}", '
                    f'"start" : vector({a[0]}, {a[2]}) * millimeter, '
                    f'"end" : vector({bb[0]}, {bb[2]}) * millimeter }}')
    L.append("export const AOW_FOURBAR = [\n" + ",\n".join(rows) + "\n];"
             if rows else "export const AOW_FOURBAR = [];")
    rows = []
    for it in arcs:
        a, m, e = (to_cad_pos(q) for q in it["arc"])
        rows.append(f'    {{ "name" : "{it["name"]}", '
                    f'"start" : vector({a[0]}, {a[2]}) * millimeter, '
                    f'"mid" : vector({m[0]}, {m[2]}) * millimeter, '
                    f'"end" : vector({e[0]}, {e[2]}) * millimeter }}')
    L.append("")
    L.append("export const AOW_FOURBAR_ARCS = [\n" + ",\n".join(rows) + "\n];"
             if rows else "export const AOW_FOURBAR_ARCS = [];")

    L.append(FS_HELPERS)
    L.append(FS_VARIABLES)
    # One feature per group, in the order the groups first appear in the
    # layout, which is the order they get built in.
    groups = list(dict.fromkeys(it["group"] for it in items))
    for g in groups:
        if g == "planes":
            L.append(FS_PLANES_GROUP)      # planes only — no tickboxes to tick
            continue
        L.append(FS_GROUP.replace("@TITLE@", g.replace("_", " "))
                 .replace("@IDENT@", _fs_ident(g)).replace("@GROUP@", g))
    L.append(FS_FOURBAR)
    L.append(FS_LEGACY)
    return _fs_qv("\n".join(L), query_vars)


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
    ap.add_argument("--no-query-vars", dest="query_vars", action="store_false",
                    help="omit setQueryVariable calls. `setQueryVariable` "
                         "arrived in Onshape 1.203 and an unknown function is "
                         "a COMPILE error, so on an older document it takes "
                         "the whole Feature Studio down rather than one "
                         "feature — this is the escape hatch")
    args = ap.parse_args()

    params, raw = load_params(args.params), load_sources(args.params)
    items = build(params, raw, args.righting, args.bumpers, args.chassis_box,
                  args.linkage_config)
    if args.format == "yaml":
        text, default_out = render(items, params, args.linkage_config), OUT
    else:
        text = render_featurescript(items, params, args.fs_version,
                                    args.query_vars)
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
    # The rear width, and WHICH clearance set it — the belt plane is a max of
    # two and reading the number without knowing which one won is how it drifts.
    am = next((i for i in items if i["name"] == "axle_mount_left"), None)
    if am:
        half = abs(to_cad_pos(am["pos"])[0]) + _mm(am["cyl"][1]) / 2
        print(f"  rear width {2 * half:.1f} mm (half {half:.1f})")
    print(f"  modelled mass excluding righting: {_g(total)} g")


if __name__ == "__main__":
    main()
