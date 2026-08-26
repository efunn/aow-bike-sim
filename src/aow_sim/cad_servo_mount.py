"""The X330 horn-pin custom feature: generate it, check it, push it.

A screwless mount for a Dynamixel X330 horn. The horn carries four Phi 1.6
blind holes on a Phi 12 bolt circle (ROBOTIS X330 drawing, and the [X330 IDLER]
view repeats it for the back horn), so a printed part can hang off it on four
printed pins with no fasteners at all. Two things do the holding and they are
not equal partners: the pins locate and drive, but most of the STABILITY is the
collar latching round the outside of the Phi 16 horn, which is why the well is
always cut and is not an option.

WHY THE PROFILES LIVE IN PYTHON. Every revolved shape here -- pin, root relief,
horn well, collar -- is emitted as a polygon by the functions below, and the
same polygon is fed to `revolve_volume` to say what the result should measure.
So `--check` is testing that Onshape's revolve, pattern and boolean did what
the polygon says, which is where the bugs have actually been. It is NOT an
independent check of the shape itself; that still needs eyes on the part.

WHY THIS IS NOT PART OF `cad_layout`. That module generates a whole document
and its push overwrites its Feature Studio wholesale. This is the opposite kind
of artifact: a few small features, hand-designed, that a person inserts once and
then drives from the dialog. It gets its own studio tab and its own push, and
--push refuses the cad_layout tab outright.

THE SPLIT AT `SPLIT_MARK` IS LOAD-BEARING. Above it is pure geometry taking a
CoordSystem; below it is the Onshape UI layer -- enums, precondition,
defineFeature. --check keeps the top half and throws the bottom away, which is
the only reason the feature can be exercised without a human picking a mate
connector. Keep new geometry above the line and new dialog below it.

    python -m aow_sim.cad_servo_mount -o docs/measurements/servo_mount.fs
    python -m aow_sim.cad_servo_mount --check
    python -m aow_sim.cad_servo_mount --push horn_features
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

from .params import load_params

CAD_PARAMS = "config/bike_params_cad.yaml"
OUT_FS = "docs/measurements/servo_mount.fs"
SPLIT_MARK = "// ==== UI LAYER BELOW -- dropped by --check ===="

# Which yaml servo key backs which FeatureScript enum value. XL330 is not a
# separate block because it is not a separate part: the drawing's title block
# reads X330, and XC330/XL330 differ in the motor, not the case or the horn.
MODELS = {"XC330": "xc330_t181", "XL330": "xc330_t181"}

# yaml key -> FeatureScript field. Only these; a servo missing any of them is
# left out of the table rather than filled in with a plausible number.
FIELDS = {
    "horn_bolt_circle":        "hornBoltCircle",
    "horn_hole_dia":           "hornHoleDia",
    "horn_hole_depth_max":     "hornHoleDepthMax",
    "horn_hole_count":         "hornHoleCount",
    "horn_diameter":           "hornDiameter",
    "horn_thickness":          "hornThickness",
    "horn_pin_clearance":      "pinClearance",
    "horn_pin_length":         "pinLength",
    "horn_pin_tip_chamfer":    "tipChamfer",
    "horn_pin_root_relief":    "rootRelief",
    "horn_pin_root_width":     "rootWidth",
    "horn_pin_root_chamfer":   "rootChamfer",
    "horn_bore_clearance":     "boreClearance",
    "horn_bore_mouth_chamfer": "boreMouthChamfer",
    "horn_bore_undercut":      "boreUndercut",
    "horn_bore_land":          "boreLand",
    "horn_case_offset":        "caseOffset",
    "horn_collar_outer_dia":   "collarOuterDia",
    "horn_collar_roof":        "collarRoof",
    "case_relief_dia":            "caseReliefDia",
    "case_pin_clearance":         "casePinClearance",
    "case_pin_length":            "casePinLength",
    "case_pin_root_relief_dia":   "casePinReliefDia",
    "case_pin_root_relief_depth": "casePinReliefDepth",
    "case_pin_root_chamfer":      "casePinRootChamfer",
    "case_pin_row_offset":        "casePinRowOffset",
    "case_side_clearance":        "caseSideClearance",
    "case_nest_clearance":        "caseNestClearance",
    "case_nest_engagement":       "caseNestLength",
    "case_top_wall":              "caseTopWall",
    "case_bottom_wall":           "caseBottomWall",
    "case_grip_length":           "caseGripLength",
    "case_cap_thickness":         "caseCapThickness",
    "case_face_clearance":        "caseFaceClearance",
    "case_wrap_length":           "caseWrapLength",
}
COUNTS = {"hornHoleCount"}   # emitted bare, not as a length

# The dialog parameters, in the order they appear in the Fit group. Each is a
# FeatureScript `const` inside servoMountGeometry, so profile expressions stay
# readable instead of inlining a division five times.
DIALOG = ("pinClearance", "pinLength", "tipChamfer", "rootRelief", "rootWidth",
          "rootChamfer", "boreClearance", "boreMouthChamfer", "boreUndercut",
          "boreLand", "caseOffset", "collarOuterDia", "collarRoof")

CASE_DIALOG = ("casePinClearance", "casePinLength", "casePinReliefDia",
               "casePinReliefDepth", "casePinRootChamfer", "caseSideClearance",
               "caseNestClearance", "caseTopWall", "caseBottomWall",
               "caseGripLength", "caseNestLength", "caseCapThickness",
               "caseFaceClearance", "caseWrapLength")

BORE_OVERSHOOT = 1.0    # mm the well cutter runs past the case face
CAVITY_OVERSHOOT = 1.0  # mm a cavity runs past the shell it is cut from


class Lin:
    """A length as a linear combination of named FeatureScript consts.

    Exists so one polygon definition can be rendered BOTH as FeatureScript
    expressions (which must stay driven by the dialog parameters) and as
    millimetre floats (which the check needs to predict a volume). Writing the
    profiles twice was the alternative, and two copies of a profile drift.
    """

    def __init__(self, terms: dict[str, float] | None = None):
        self.terms = {k: v for k, v in (terms or {}).items() if v}

    def _combine(self, other, sign: float) -> "Lin":
        out = dict(self.terms)
        for k, v in other.terms.items():
            out[k] = out.get(k, 0.0) + sign * v
        return Lin(out)

    def __add__(self, o): return self._combine(o, 1.0)
    def __sub__(self, o): return self._combine(o, -1.0)
    def __neg__(self): return Lin({k: -v for k, v in self.terms.items()})
    def __mul__(self, k: float): return Lin({n: v * k for n, v in self.terms.items()})
    __rmul__ = __mul__

    def fs(self) -> str:
        if not self.terms:
            return "0 * millimeter"     # a bare 0 is not a length in FS
        out = ""
        for name, c in self.terms.items():
            mag = "" if abs(c) == 1 else f"{abs(c):g} * "
            if not out:
                out = ("-" if c < 0 else "") + mag + name
            else:
                out += (" - " if c < 0 else " + ") + mag + name
        return out

    def val(self, env: dict[str, float]) -> float:
        return sum(c * env[n] for n, c in self.terms.items())


def S(name: str) -> Lin:
    return Lin({name: 1.0})


ZERO = Lin()


# --------------------------------------------------------------------------
# The profiles. Each is a closed polygon in (u, v) = (radial from its own
# revolve axis, axial with +v pointing OUT of the servo). The datum v = 0 is
# the horn's outer face, so the horn itself occupies v in [-hornThickness, 0]
# and the printed part lives above it.
#
# Zero-length edges are expected, not avoided: every optional feature is a
# dialog parameter that can be set to 0, and the FeatureScript helper filters
# degenerate segments so one polygon serves every setting. Branching per
# combination would be 2^4 variants of the same shape.
# --------------------------------------------------------------------------

def pin_profile() -> list[tuple[Lin, Lin]]:
    """Shaft plus optional tip lead-in, revolved about the pin's own axis."""
    r, L, c = S("pinR"), S("pinL"), S("tipCh")
    return [(ZERO, ZERO), (r, ZERO), (r, -(L - c)), (r - c, -L), (ZERO, -L)]


def relief_profile() -> list[tuple[Lin, Lin]]:
    """The elephant-foot groove: an annulus cut UP into the well floor.

    Revolving a rectangle that does not touch the axis gives a tube, which is
    the groove -- the material inside it is the pin. The inner corner at the
    groove's DEEP end is chamfered, so cutting that corner off the CUTTER
    leaves a conical flare where the pin meets solid part: a gusset, so
    relieving the elephant foot does not leave the pin on a sharp step.

    The chamfer sits at the deep end and NOT at the mouth. Once the groove is
    cut, the pin's root is the groove's floor -- the horn mount plane is just
    where the pin passes through. Chamfering at the mouth gusseted a spot that
    carries nothing and sat flush against the mount plane.
    """
    r, w, d, ch = S("pinR"), S("relW"), S("relD"), S("relCh")
    return [(r, ZERO), (r + w, ZERO), (r + w, d), (r + ch, d), (r, d - ch)]


def bore_profile(clip: bool = False) -> list[tuple[Lin, Lin]]:
    """The horn well, revolved about the shaft axis.

    Mouth inward to the floor: a lead-in chamfer to get it started over the
    horn, the straight bore that grips it, a 45 degree undercut chamfer opening
    OUTWARD, then a flat land at the wider radius running to the floor. The
    land is a relief around the horn's top edge, and the chamfer gives the
    floor a 45 degree transition to bridge to rather than a flat roof.

    THE CHAMFER OPENS OUT. It used to close in onto the floor, which put the
    floor at Phi 15.7 against a Phi 16 horn -- the rim landed on a lip, and
    enlarging the chamfer drove it further into the horn. Opening outward means
    the same number widens the relief instead. There is no separate radial step
    any more either: at 45 degrees the radial and axial legs are equal by
    construction, so a second parameter could only disagree with the first.

    The mouth is at -wellT, which is `caseOffset` SHORT of the case face, so
    the collar's rim cannot rub on the case.

    `clip` drops the overshoot past the mouth. The cutter has to run past it to
    cut cleanly, but that overshoot is outside the collar, so the volume the
    check predicts must not count it.
    """
    R, T = S("boreR"), S("wellT")
    mc, u, land = S("mouthCh"), S("boreUndercut"), S("boreLand")
    mouth = [] if clip else [(ZERO, -T - S("over")), (R + mc, -T - S("over"))]
    return mouth + [
        (R + mc, -T),
        (R, -T + mc),           # lead-in chamfer
        (R, -(u + land)),       # the straight bore, gripping the horn
        (R + u, -land),         # undercut chamfer, opening OUTWARD
        (R + u, ZERO),          # flat land, clearing the horn's top edge
        (ZERO, ZERO),
    ] + ([(ZERO, -T)] if clip else [])


def collar_profile() -> list[tuple[Lin, Lin]]:
    """The standalone puck, built only when no target part is picked."""
    Rc, T, roof = S("collarR"), S("wellT"), S("roof")
    return [(ZERO, -T), (Rc, -T), (Rc, roof), (ZERO, roof)]


def revolve_volume(poly: list[tuple[float, float]]) -> float:
    """Exact volume of a closed polygon revolved about u = 0.

    Sum of frustum contributions round the boundary; sign falls out of the
    winding, so take the magnitude and stop caring which way it was drawn.
    """
    poly = dedupe(poly)
    total = 0.0
    for i, (u1, v1) in enumerate(poly):
        u2, v2 = poly[(i + 1) % len(poly)]
        total += (v2 - v1) * (u1 * u1 + u1 * u2 + u2 * u2) / 3.0
    return abs(total) * math.pi


def dedupe(poly: list[tuple[float, float]], tol: float = 1e-9):
    """Drop zero-length edges -- the same filter the FeatureScript applies."""
    out: list[tuple[float, float]] = []
    for p in poly:
        if not out or math.dist(p, out[-1]) > tol:
            out.append(p)
    while len(out) > 1 and math.dist(out[0], out[-1]) <= tol:
        out.pop()
    return out


def servo_table(params: dict) -> dict[str, dict]:
    """The subset of `servos` that has a complete horn fastener interface."""
    out = {}
    for name, key in MODELS.items():
        spec = params["servos"].get(key, {})
        if missing := [k for k in FIELDS if k not in spec]:
            print(f"  skipping {name}: {key} lacks {', '.join(missing)}")
            continue
        out[name] = {FIELDS[k]: spec[k] for k in FIELDS}
        # Three dimensions live in lists rather than as scalars, so they
        # cannot come through FIELDS. box_size is D x W x H with D along the
        # shaft; case_hole_pattern is [across, along].
        d, w, h = spec["box_size"]
        out[name].update(caseDepth=d, caseWidth=w, caseHeight=h,
                         shaftFromEnd=spec["shaft_from_end"],
                         caseHoleSpanX=spec["case_hole_pattern"][0])
    return out


def env(table: dict[str, dict], servo: str = "XC330") -> dict[str, float]:
    """Millimetre values for every const the profiles name, at the defaults."""
    t = {k: (v if k in COUNTS else v * 1000) for k, v in table[servo].items()}
    return {
        "pinR": (t["hornHoleDia"] - t["pinClearance"]) / 2,
        "pinL": t["pinLength"], "tipCh": t["tipChamfer"],
        "relD": t["rootRelief"], "relW": t["rootWidth"],
        "relCh": t["rootChamfer"],
        "boreR": (t["hornDiameter"] + t["boreClearance"]) / 2,
        "hornT": t["hornThickness"], "mouthCh": t["boreMouthChamfer"],
        "wellT": t["hornThickness"] - t["caseOffset"],
        "boreLand": t["boreLand"], "boreUndercut": t["boreUndercut"],
        "over": BORE_OVERSHOOT,
        "collarR": t["collarOuterDia"] / 2, "roof": t["collarRoof"],
        "pcd": t["hornBoltCircle"], "n": t["hornHoleCount"],
    }


def case_env(table: dict[str, dict], servo: str = "XC330") -> dict[str, float]:
    """Millimetre values for the case shell, at the defaults.

    Binds `pinR`, `pinL`, `relD`, `relW`, `relCh` -- the very names the horn
    profiles are written against -- to the CASE numbers. That is the whole
    reason `pin_profile` and `relief_profile` need no case-specific variant:
    the polygon is the same shape, the constants under it are not.
    """
    t = {k: (v if k in COUNTS else v * 1000) for k, v in table[servo].items()}
    pin_r = (t["caseReliefDia"] - t["casePinClearance"]) / 2
    e = {
        # The pin is lengthened by the face clearance so `casePinLength`
        # keeps meaning depth INTO the hole rather than overall length.
        "pinR": pin_r,
        "pinL": t["casePinLength"] + t["caseFaceClearance"], "tipCh": 0.0,
        "relD": t["casePinReliefDepth"],
        "relW": t["casePinReliefDia"] / 2 - pin_r,
        "relCh": t["casePinRootChamfer"],
        "hw": t["caseWidth"] / 2,
        "endY": t["caseHeight"] - t["shaftFromEnd"],
        "hornZ": -t["hornThickness"],
        "caseDepth": t["caseDepth"], "rowX": t["caseHoleSpanX"] / 2,
        "sc": t["caseSideClearance"], "topWall": t["caseTopWall"],
        "nestClr": t["caseNestClearance"], "botWall": t["caseBottomWall"],
        "grip": t["caseGripLength"], "nest": t["caseNestLength"],
        "capT": t["caseCapThickness"], "wrap": t["caseWrapLength"],
        "faceClr": t["caseFaceClearance"],
        "over": CAVITY_OVERSHOOT,
    }
    e["backZ"] = e["hornZ"] - e["caseDepth"]
    e["rowY"] = e["endY"] - t["caseHeight"] / 2 + t["casePinRowOffset"]
    e["inner"] = e["hw"] + e["sc"]
    e["topOuter"] = e["inner"] + e["topWall"]
    e["nestBore"] = e["topOuter"] + e["nestClr"]
    e["botOuter"] = e["nestBore"] + e["botWall"]
    e["skirt"] = e["caseDepth"] - e["grip"]
    return e


def _fs_num(field: str, v: float) -> str:
    if field in COUNTS:
        return str(int(v))
    return f"{v * 1000:.4g} * millimeter"    # yaml is metres, CAD reads mm


def lint_fs(text: str) -> None:
    """Catch generator mistakes that only a human reading the studio would see.

    `--check` compiles the geometry layer and never the UI layer, so a mistake
    in the dialog reaches the document intact and every feature in the studio
    goes red at once. A doubled brace did exactly that on 2026-08-25: the case
    Fit group was built with quadruple braces, correct only if the result is
    run through a second f-string, and `{{` survived into the FeatureScript.

    A literal brace pair can never be valid here -- nothing generated emits
    one -- so its presence is unambiguous.
    """
    if bad := [f"line {i}: {l.strip()}" for i, l in enumerate(text.splitlines(), 1)
               if "{{" in l or "}}" in l]:
        raise SystemExit("generated FeatureScript has literal doubled braces, "
                         "which means an f-string escaped one level too many:\n  "
                         + "\n  ".join(bad[:5]))


def _fs_poly(poly: list[tuple[Lin, Lin]]) -> str:
    pts = ",\n            ".join(f"vector({u.fs()}, {v.fs()})" for u, v in poly)
    return f"[{pts}]"


def build_fs(table: dict[str, dict], fs_version: str = "3044") -> str:
    rows = []
    for name, t in table.items():
        fields = ",\n".join(f'        "{k}" : {_fs_num(k, v)}'
                            for k, v in t.items())
        rows.append(f'    "{name}" : {{\n{fields}\n    }}')
    table_fs = ",\n".join(rows)

    enum_vals = ",\n".join(
        f'    annotation {{ "Name" : "{n}" }}\n    {n}' for n in table)
    key_fn = "\n".join(
        f"    if (m == ServoModel.{n}) return \"{n}\";" for n in table)
    default = next(iter(table))
    dialog_pass = ",\n".join(f'                  "{k}" : definition.{k}'
                             for k in DIALOG)
    dialog_ui = "\n\n".join(f"""            annotation {{ "Name" : "{lbl}" }}
            isLength(definition.{k}, {{ (millimeter) : [{lo}, {d:g}, {hi}] }} as LengthBoundSpec);"""
        for k, lbl, lo, hi, d in [
            ("pinClearance", "Pin clearance (diametral)", 0.0, 1.0,
             table[default]["pinClearance"] * 1000),
            ("pinLength", "Pin length", 0.5, 3.0,
             table[default]["pinLength"] * 1000),
            ("tipChamfer", "Pin tip lead-in", 0.0, 1.0,
             table[default]["tipChamfer"] * 1000),
            ("rootRelief", "Root relief depth", 0.0, 2.0,
             table[default]["rootRelief"] * 1000),
            ("rootWidth", "Root relief width", 0.0, 2.0,
             table[default]["rootWidth"] * 1000),
            ("rootChamfer", "Root relief inner chamfer", 0.0, 1.0,
             table[default]["rootChamfer"] * 1000),
            ("boreClearance", "Well clearance (diametral)", 0.0, 1.0,
             table[default]["boreClearance"] * 1000),
            ("boreMouthChamfer", "Well mouth lead-in", 0.0, 2.0,
             table[default]["boreMouthChamfer"] * 1000),
            ("boreUndercut", "Well undercut chamfer (radial = axial)", 0.0, 2.0,
             table[default]["boreUndercut"] * 1000),
            ("boreLand", "Well land at the undercut", 0.0, 3.0,
             table[default]["boreLand"] * 1000),
            ("caseOffset", "Clearance to the case face", 0.0, 2.0,
             table[default]["caseOffset"] * 1000),
            ("collarOuterDia", "Collar outside diameter", 16.5, 60.0,
             table[default]["collarOuterDia"] * 1000),
            ("collarRoof", "Collar roof thickness", 0.5, 20.0,
             table[default]["collarRoof"] * 1000),
        ])

    # The case shell reuses these two polygons verbatim -- the FeatureScript
    # binds pinR/relD/relW/relCh to the case numbers before they are read.
    pin_poly, relief_poly = _fs_poly(pin_profile()), _fs_poly(relief_profile())
    case_labels = {
        "caseFaceClearance":  ("Clearance on the horn/back faces", 0.0, 2.0),
        "casePinClearance":   ("Pin clearance (diametral, in the Phi 2 bore)", 0.0, 1.0),
        "casePinLength":      ("Pin length", 0.5, 4.0),
        "casePinReliefDia":   ("Root relief diameter", 0.0, 8.0),
        "casePinReliefDepth": ("Root relief depth", 0.0, 2.0),
        "casePinRootChamfer": ("Root relief inner chamfer", 0.0, 1.0),
        "caseSideClearance":  ("Clearance on the case sides (per side)", 0.0, 1.0),
        "caseNestClearance":  ("Nest clearance (per side)", 0.0, 1.0),
        "caseTopWall":        ("Top half wall", 0.5, 6.0),
        "caseBottomWall":     ("Bottom half wall", 0.5, 6.0),
        "caseGripLength":     ("Bottom half grip on the servo", 0.0, 10.0),
        "caseNestLength":     ("Nest engagement", 0.0, 20.0),
        "caseCapThickness":   ("Cap thickness over the face", 0.5, 20.0),
        # 16.5 is not a taste bound: past it the cap fouls the Phi 16 horn.
        "caseWrapLength":     ("Wrap length from the far end", 2.0, 16.5),
    }
    # DOUBLE braces, not quadruple. This string is substituted into the big
    # f-string as a VALUE, and a substituted value is not re-scanned for
    # braces -- so `{{{{` would survive into the FeatureScript as a literal
    # `{{`, which compiles nowhere. Quadrupling is only right when the result
    # is itself run through another f-string. It is not.
    case_dialog_ui = "\n\n".join(
        f"""            annotation {{ "Name" : "{case_labels[k][0]}" }}
            isLength(definition.{k}, {{ (millimeter) : [{case_labels[k][1]}, """
        f"""{table[default][k] * 1000:g}, {case_labels[k][2]}] }} as LengthBoundSpec);"""
        for k in CASE_DIALOG)
    case_dialog_pass = ",\n".join(f'                  "{k}" : definition.{k}'
                                   for k in CASE_DIALOG)

    return f"""FeatureScript {fs_version};
import(path : "onshape/std/geometry.fs", version : "{fs_version}.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_servo_mount --push horn_features
 *
 * The command above is deliberately BARE. Wrapped in backticks it is shell
 * command substitution, and an edge filter in front of the Onshape API rejects
 * the whole push with a bare nginx 403 -- no JSON, so it never reaches Onshape.
 * Backticks elsewhere are fine and so is the bare command; only the two
 * together trip it. Measured 2026-08-25, one probe each.
 *
 * Numbers come from {CAD_PARAMS}, which cites
 * docs/robotis/XC-330.pdf for every measured one. The four well-profile
 * dimensions are marked GUESS there: the shape is from a description of the
 * existing dynamixel_wrench_with_idler part, not a measurement of it.
 *
 * The mounting datum is the OUTER FACE OF THE HORN with +Z pointing out of the
 * servo -- the same datum cad_layout uses, so a mate connector that lands a
 * ROBOTIS STEP correctly also drives this feature correctly.
 */

export const SERVO_MOUNT_TABLE = {{
{table_fs}
}};

/**
 * Sketch a closed polygon, dropping zero-length edges.
 *
 * Not a convenience. Every optional feature here -- the tip lead-in, the root
 * chamfer, the well's step -- is a dialog parameter that may be set to zero,
 * and a zero-length sketch segment is not a degenerate shape but a solve
 * error. Filtering here lets ONE profile serve every setting, instead of a
 * branch per combination of which features are switched off.
 */
export function skPolygon(sk, pts is array)
{{
    const tol = 1e-8 * meter;
    var keep = [];
    for (var p in pts)
        if (size(keep) == 0 || norm(p - keep[size(keep) - 1]) > tol)
            keep = append(keep, p);
    var poly = [];
    for (var i = 0; i < size(keep); i += 1)
        if (!(i == size(keep) - 1 && norm(keep[i] - keep[0]) < tol))
            poly = append(poly, keep[i]);
    for (var i = 0; i < size(poly); i += 1)
        skLineSegment(sk, "s" ~ i, {{ "start" : poly[i],
                                     "end"   : poly[(i + 1) % size(poly)] }});
}}

/** Sketch a profile, revolve it a full turn, and bin the sketch. */
export function revolveProfile(context is Context, id is Id, tag is string,
                               profPlane is Plane, axis is Line, pts is array)
{{
    var sk = newSketchOnPlane(context, id + tag, {{ "sketchPlane" : profPlane }});
    skPolygon(sk, pts);
    skSolve(sk);
    opRevolve(context, id + (tag ~ "Rev"), {{
            "entities"     : qSketchRegion(id + tag),
            "axis"         : axis,
            "angleForward" : 360 * degree }});
    opDeleteBodies(context, id + (tag ~ "Del"), {{
            "entities" : qCreatedBy(id + tag, EntityType.BODY) }});
}}

/**
 * Build the pins, the cutters and (when wanted) the collar, about cs.
 *
 * Takes a CoordSystem rather than a Query on purpose. A Query needs a human to
 * pick a mate connector, and that makes the geometry untestable; a CoordSystem
 * can be synthesised, which is what --check does.
 *
 * Returns pins, cutters and collar as separate queries and booleans NOTHING.
 * The caller decides, because the ORDER matters: the well clears the horn's
 * whole envelope and the pins stand inside it, so unioning first loses them.
 */
export function servoMountGeometry(context is Context, id is Id,
                                   cs is CoordSystem, opt is map) returns map
{{
    const t = SERVO_MOUNT_TABLE[opt.servo];

    const pinR         = (t.hornHoleDia - opt.pinClearance) / 2;
    const pinL         = opt.pinLength;
    const tipCh        = opt.tipChamfer;
    const relD         = opt.rootRelief;
    const relW         = opt.rootWidth;
    const relCh        = opt.rootChamfer;
    const boreR        = (t.hornDiameter + opt.boreClearance) / 2;
    const hornT        = t.hornThickness;
    const mouthCh      = opt.boreMouthChamfer;
    const boreLand     = opt.boreLand;
    const boreUndercut = opt.boreUndercut;
    // The well stops short of the case face by caseOffset, so the collar's rim
    // clears the case instead of rubbing on it.
    const wellT        = hornT - opt.caseOffset;
    const collarR      = opt.collarOuterDia / 2;
    const roof         = opt.collarRoof;
    const over         = {BORE_OVERSHOOT:g} * millimeter;

    // A CoordSystem carries origin, xAxis and zAxis and NOTHING ELSE. There is
    // no cs.yAxis; reading one gives undefined, and the first arithmetic on it
    // fails as "Operand for '-' was not a number" several lines from the
    // actual mistake.
    const yAxis = cross(cs.zAxis, cs.xAxis);

    // Both profile planes have normal -Y and x-direction +X, which makes the
    // sketch's (u, v) read as (radial, axial out of the servo). With normal +Y
    // the v axis comes out backwards and everything is built INSIDE the servo,
    // which renders as nothing and looks like a failed revolve.
    const pinAxisPt  = cs.origin + t.hornBoltCircle / 2 * cs.xAxis;
    const pinPlane   = plane(pinAxisPt, -yAxis, cs.xAxis);
    const pinAxis    = line(pinAxisPt, cs.zAxis);
    const axialPlane = plane(cs.origin, -yAxis, cs.xAxis);
    const shaftAxis  = line(cs.origin, cs.zAxis);

    revolveProfile(context, id, "pin", pinPlane, pinAxis,
        {_fs_poly(pin_profile())});
    revolveProfile(context, id, "relief", pinPlane, pinAxis,
        {_fs_poly(relief_profile())});
    revolveProfile(context, id, "bore", axialPlane, shaftAxis,
        {_fs_poly(bore_profile())});
    if (opt.collar)
        revolveProfile(context, id, "collar", axialPlane, shaftAxis,
            {_fs_poly(collar_profile())});

    // Ring the pin and its groove round the bolt circle. One opPattern each,
    // not one for both, so the two stay separable: pins get unioned and
    // grooves get subtracted, and a query that mixes them cannot do either.
    var xf = [];
    var names = [];
    for (var i = 1; i < t.hornHoleCount; i += 1)
    {{
        xf = append(xf, rotationAround(shaftAxis,
                                       i * (360 / t.hornHoleCount) * degree));
        names = append(names, "i" ~ i);
    }}
    opPattern(context, id + "pinRing", {{
            "entities"      : qCreatedBy(id + "pinRev", EntityType.BODY),
            "transforms"    : xf,
            "instanceNames" : names }});
    opPattern(context, id + "reliefRing", {{
            "entities"      : qCreatedBy(id + "reliefRev", EntityType.BODY),
            "transforms"    : xf,
            "instanceNames" : names }});

    return {{
        "pins" : qUnion([qCreatedBy(id + "pinRev", EntityType.BODY),
                         qCreatedBy(id + "pinRing", EntityType.BODY)]),
        "cutters" : qUnion([qCreatedBy(id + "reliefRev", EntityType.BODY),
                            qCreatedBy(id + "reliefRing", EntityType.BODY),
                            qCreatedBy(id + "boreRev", EntityType.BODY)]),
        "collar" : qCreatedBy(id + "collarRev", EntityType.BODY)
    }};
}}

/**
 * The whole build: geometry, then the booleans, in the one order that works.
 *
 * With no target the feature makes its own collar, because four pins floating
 * on a bolt circle are four separate solids and not a part. The collar is what
 * they union INTO, and it is also what actually holds the mount on -- the pins
 * locate and drive, the collar latching round the outside does the rest.
 *
 * Returns the query for the finished body.
 */
export function servoMountBuild(context is Context, id is Id, cs is CoordSystem,
                                opt is map, target is Query) returns Query
{{
    const standalone = isQueryEmpty(context, target);
    const g = servoMountGeometry(context, id + "geom", cs,
                                 mergeMaps(opt, {{ "collar" : standalone }}));
    const into = standalone ? g.collar : target;

    opBoolean(context, id + "cut", {{
            "tools"         : g.cutters,
            "targets"       : into,
            "operationType" : BooleanOperationType.SUBTRACTION }});
    // UNION takes `tools` ONLY -- every body to be merged, target included --
    // and NO `targets` key. Written the way SUBTRACTION is written, it unions
    // the four pins with each other, which does nothing because they do not
    // touch, and leaves the collar alone: four loose pins and a bare collar,
    // five bodies, with no error anywhere. Volume said so before body count
    // did, and only because the shortfall was exactly four pins.
    opBoolean(context, id + "add", {{
            "tools"         : qUnion([g.pins, into]),
            "operationType" : BooleanOperationType.UNION }});
    return into;
}}

/** A rectangular solid in the datum frame: |x| <= xHalf, y0..y1, z0..z1. */
export function boxSolid(context is Context, id is Id, tag is string,
                         cs is CoordSystem, xHalf, y0, y1, z0, z1)
{{
    var sk = newSketchOnPlane(context, id + tag, {{
            "sketchPlane" : plane(cs.origin + z0 * cs.zAxis, cs.zAxis, cs.xAxis) }});
    skRectangle(sk, "r", {{ "firstCorner"  : vector(-xHalf, y0),
                           "secondCorner" : vector(xHalf, y1) }});
    skSolve(sk);
    opExtrude(context, id + (tag ~ "Ext"), {{
            "entities"  : qSketchRegion(id + tag),
            "direction" : cs.zAxis,
            "endBound"  : BoundingType.BLIND,
            "endDepth"  : z1 - z0 }});
    opDeleteBodies(context, id + (tag ~ "Del"), {{
            "entities" : qCreatedBy(id + tag, EntityType.BODY) }});
}}

/**
 * One half of the two-part case shell, about the SAME datum as the horn pin:
 * the horn's outer face, +Z out of the servo, +Y toward the far end.
 *
 * The shell never stands further off the servo than it has to. Over the two
 * 20 x 34 faces its whole extent is the cap. Round the other three faces the
 * depth splits into exactly three runs that sum to the case depth: the bottom
 * half gripping the servo alone, then the nest where both halves overlap, then
 * the top half running on to the horn face. Only the first two are parameters;
 * the third is the remainder, so the three cannot disagree with the case.
 *
 * The near end -- the shaft end -- is left open. There is nothing to hold on to
 * there: of the two hole rows, the one 7.5 from the shaft axis is inside the
 * Phi 16 horn, so only the far row at 22.5 is usable and the shell wraps that
 * end. That is also what caps how far the cap can reach before it fouls it.
 *
 * pinR, pinL, relD, relW and relCh below are the same names the horn profiles
 * are written against, bound here to the CASE numbers -- which is why both
 * features share one pin polygon and one relief polygon.
 */
export function caseShellGeometry(context is Context, id is Id, cs is CoordSystem,
                                  opt is map) returns map
{{
    const t   = SERVO_MOUNT_TABLE[opt.servo];
    const top = opt.part == "TOP";

    // The case pin seats in the Phi 2 RELIEF bore of the drawing's Detail A/B,
    // NOT the Phi 1.6 tapping section the horn pins use. Different hole,
    // different clearance; the two must not be unified.
    const pinR  = (t.caseReliefDia - opt.casePinClearance) / 2;
    // Lengthened by the face clearance so casePinLength keeps meaning depth
    // INTO the hole. Otherwise opening the clearance would silently shorten
    // the grip rather than standing the cap off.
    const pinL  = opt.casePinLength + opt.caseFaceClearance;
    const tipCh = 0 * millimeter;
    const relD  = opt.casePinReliefDepth;
    const relW  = opt.casePinReliefDia / 2 - pinR;
    const relCh = opt.casePinRootChamfer;

    const hw    = t.caseWidth / 2;
    const endY  = t.caseHeight - t.shaftFromEnd;
    const hornZ = -t.hornThickness;
    const backZ = hornZ - t.caseDepth;
    const rowX  = t.caseHoleSpanX / 2;
    const rowY  = endY - t.caseHeight / 2 + t.casePinRowOffset;

    const sc       = opt.caseSideClearance;
    const inner    = hw + sc;
    const topOuter = inner + opt.caseTopWall;
    const nestBore = topOuter + opt.caseNestClearance;
    const botOuter = nestBore + opt.caseBottomWall;
    const skirt    = t.caseDepth - opt.caseGripLength;
    const over     = {CAVITY_OVERSHOOT:g} * millimeter;

    const y0    = endY - opt.caseWrapLength;   // the open end, toward the shaft
    const fc    = opt.caseFaceClearance;
    const seatZ = top ? hornZ + fc : backZ - fc;
    const outZ  = top ? cs.zAxis : -cs.zAxis;

    if (top)
    {{
        boxSolid(context, id, "shell", cs, topOuter,
                 y0, endY + sc + opt.caseTopWall,
                 hornZ - skirt, seatZ + opt.caseCapThickness);
        boxSolid(context, id, "cav", cs, inner,
                 y0 - over, endY + sc, hornZ - skirt - over, seatZ);
    }}
    else
    {{
        boxSolid(context, id, "shell", cs, botOuter,
                 y0, endY + sc + opt.caseTopWall + opt.caseNestClearance
                     + opt.caseBottomWall,
                 seatZ - opt.caseCapThickness,
                 backZ + opt.caseGripLength + opt.caseNestLength);
        boxSolid(context, id, "cav", cs, inner,
                 y0 - over, endY + sc, seatZ, backZ + opt.caseGripLength);
        boxSolid(context, id, "nest", cs, nestBore,
                 y0 - over, endY + sc + opt.caseTopWall + opt.caseNestClearance,
                 backZ + opt.caseGripLength,
                 backZ + opt.caseGripLength + opt.caseNestLength + over);
    }}

    // One pin at +rowX, mirrored to -rowX. Two per face, not four: see above.
    const yA        = cross(cs.zAxis, cs.xAxis);
    const pinOrigin = cs.origin + rowX * cs.xAxis + rowY * yA + seatZ * cs.zAxis;
    const pinPlane  = plane(pinOrigin, -cross(outZ, cs.xAxis), cs.xAxis);
    const pinAxis   = line(pinOrigin, outZ);

    revolveProfile(context, id, "pin", pinPlane, pinAxis,
        {pin_poly});
    revolveProfile(context, id, "relief", pinPlane, pinAxis,
        {relief_poly});

    const mirror = [transform(-2 * rowX * cs.xAxis)];
    opPattern(context, id + "pinRing", {{
            "entities"      : qCreatedBy(id + "pinRev", EntityType.BODY),
            "transforms"    : mirror,
            "instanceNames" : ["i1"] }});
    opPattern(context, id + "reliefRing", {{
            "entities"      : qCreatedBy(id + "reliefRev", EntityType.BODY),
            "transforms"    : mirror,
            "instanceNames" : ["i1"] }});

    return {{
        "shell" : qCreatedBy(id + "shellExt", EntityType.BODY),
        "pins"  : qUnion([qCreatedBy(id + "pinRev", EntityType.BODY),
                          qCreatedBy(id + "pinRing", EntityType.BODY)]),
        "cutters" : qUnion([qCreatedBy(id + "cavExt", EntityType.BODY),
                            qCreatedBy(id + "nestExt", EntityType.BODY),
                            qCreatedBy(id + "reliefRev", EntityType.BODY),
                            qCreatedBy(id + "reliefRing", EntityType.BODY)])
    }};
}}

/** Case shell: cavities out, then pins in. Same order rule as the horn. */
export function caseShellBuild(context is Context, id is Id, cs is CoordSystem,
                               opt is map, target is Query) returns Query
{{
    const g = caseShellGeometry(context, id + "geom", cs, opt);
    var into = g.shell;
    if (!isQueryEmpty(context, target))
    {{
        opBoolean(context, id + "merge", {{
                "tools"         : qUnion([g.shell, target]),
                "operationType" : BooleanOperationType.UNION }});
        into = target;
    }}
    opBoolean(context, id + "cut", {{
            "tools"         : g.cutters,
            "targets"       : into,
            "operationType" : BooleanOperationType.SUBTRACTION }});
    opBoolean(context, id + "add", {{
            "tools"         : qUnion([g.pins, into]),
            "operationType" : BooleanOperationType.UNION }});
    return into;
}}

/**
 * Both halves from ONE feature invocation, sharing one set of fit numbers.
 *
 * They were two features with a Top/Bottom switch, which meant every clearance
 * had to be typed twice and could drift apart between them -- and the numbers
 * that MUST agree are exactly the ones describing the joint between the two.
 *
 * `flip` spins the frame 180 degrees about the datum's own Z. The horn feature
 * does not need it: four pins on a bolt circle look the same from any quarter
 * turn, so the datum's X direction never mattered there. The shell is not
 * symmetric -- it wraps the far end and leaves the shaft end open -- so a datum
 * whose X happens to point the other way builds it on the wrong side of the
 * servo. Flipping here is cheaper than re-making the mate connector.
 */
export function caseShellPair(context is Context, id is Id, cs is CoordSystem,
                              opt is map) returns Query
{{
    const useCS = opt.flip ? coordSystem(cs.origin, -cs.xAxis, cs.zAxis) : cs;
    var made = [];
    // `!= false`, not `== true`. These are new parameters on a feature that is
    // already inserted in live documents; if Onshape does not backfill a
    // default into an existing instance it reads as undefined, and under
    // `== true` both halves would vanish on the next regeneration. Unset
    // builds. Same reasoning as the group tickboxes in cad_layout.
    if (opt.makeTop != false)
        made = append(made, caseShellBuild(context, id + "top", useCS,
                mergeMaps(opt, {{ "part" : "TOP" }}), qNothing()));
    if (opt.makeBottom != false)
        made = append(made, caseShellBuild(context, id + "bot", useCS,
                mergeMaps(opt, {{ "part" : "BOTTOM" }}), qNothing()));
    return qUnion(made);
}}


{SPLIT_MARK}

export enum ServoModel
{{
{enum_vals}
}}

export function servoKey(m is ServoModel) returns string
{{
{key_fn}
    return "{default}";
}}

annotation {{ "Feature Type Name" : "X330 horn pin",
             "Filter Selector" : "allparts" }}
export const x330HornPin = defineFeature(function(context is Context, id is Id,
                                                  definition is map)
    precondition
    {{
        annotation {{ "Name" : "Servo" }}
        definition.servo is ServoModel;

        annotation {{ "Name" : "Horn datum",
                     "Filter" : BodyType.MATE_CONNECTOR,
                     "MaxNumberOfPicks" : 1 }}
        definition.datum is Query;

        annotation {{ "Name" : "Part to modify (leave empty for a collar)",
                     "Filter" : EntityType.BODY && BodyType.SOLID }}
        definition.target is Query;

        annotation {{ "Group Name" : "Fit", "Collapsed By Default" : true }}
        {{
{dialog_ui}
        }}
    }}
    {{
        servoMountBuild(context, id + "build",
                evMateConnector(context, {{ "mateConnector" : definition.datum }}),
                {{ "servo" : servoKey(definition.servo),
{dialog_pass} }},
                definition.target);
    }});

annotation {{ "Feature Type Name" : "X330 case shell",
             "Filter Selector" : "allparts" }}
export const x330CaseShell = defineFeature(function(context is Context, id is Id,
                                                    definition is map)
    precondition
    {{
        annotation {{ "Name" : "Servo" }}
        definition.servo is ServoModel;

        annotation {{ "Name" : "Horn datum",
                     "Filter" : BodyType.MATE_CONNECTOR,
                     "MaxNumberOfPicks" : 1 }}
        definition.datum is Query;

        // Both halves from one feature. They were a Top/Bottom enum on two
        // separate features, which meant retyping every clearance twice --
        // and the numbers that must agree are precisely the ones describing
        // the joint between them.
        annotation {{ "Name" : "Top half (horn side)", "Default" : true }}
        definition.makeTop is boolean;

        annotation {{ "Name" : "Bottom half (back)", "Default" : true }}
        definition.makeBottom is boolean;

        annotation {{ "Name" : "Flip 180 degrees about the datum" }}
        definition.flip is boolean;

        annotation {{ "Group Name" : "Fit", "Collapsed By Default" : true }}
        {{
{case_dialog_ui}
        }}
    }}
    {{
        caseShellPair(context, id + "build",
                evMateConnector(context, {{ "mateConnector" : definition.datum }}),
                {{ "servo" : servoKey(definition.servo),
                  "makeTop" : definition.makeTop,
                  "makeBottom" : definition.makeBottom,
                  "flip" : definition.flip,
{case_dialog_pass} }});
    }});
"""
# --------------------------------------------------------------------------
# --check: compile AND RUN the geometry layer, then measure what it built.
#
# Compiling clean proves almost nothing here -- a revolve that builds nothing,
# a pattern that never fires and a units slip all compile. So the driver below
# measures body counts, volumes and bounding boxes, and `check()` compares them
# against numbers worked out from the drawing on THIS side. That is the whole
# point: the assertions are independent of the FeatureScript, so agreeing means
# something.
#
# ONE billable call, and a failed compile still bills (see onshape.py). Every
# question goes in the one script.
# --------------------------------------------------------------------------

def _block_end(lines: list[str], i: int) -> int:
    """Index of the line closing the brace block that opens at or after `i`."""
    depth, started = 0, False
    for k in range(i, len(lines)):
        depth += lines[k].count("{") - lines[k].count("}")
        started = started or "{" in lines[k]
        if started and depth == 0:
            return k
    raise ValueError("unterminated block")


def _strip_comments(fs: str) -> str:
    """Drop every comment before sending FeatureScript to the eval endpoint.

    NOT cosmetic, and not about size. An edge filter in front of the Onshape
    API rejects a payload containing a COMMAND IN BACKTICKS -- shell command
    substitution -- with a bare nginx 403 carrying no Onshape JSON, so the call
    never reaches Onshape at all. Narrowed 2026-08-25 by probing the push
    endpoint, which takes any text: backticks round a harmless word passed, the
    bare command passed, the two together did not.

    The generated header now writes its regeneration command unquoted, so a
    push goes through with all its comments intact. Stripping here is belt and
    braces for the CHECK path only: comments cannot change what the geometry
    does, so the check has nothing to lose, and a future comment that trips some
    other rule then costs a confusing failure on the push alone rather than on
    both.

    If a push ever does come back as a bare 403, look for a new backticked
    command first. The fallback is the browser -- pasting into the Feature
    Studio always works and is quota-exempt.
    """
    fs = re.sub(r"/\*.*?\*/", "", fs, flags=re.S)
    fs = re.sub(r"^\s*//.*$", "", fs, flags=re.M)
    return "\n".join(l for l in fs.splitlines() if l.strip())


def check_wrapper(fs: str) -> str:
    """Rewrite the geometry layer into the bare function expression eval wants.

    DEFINITION ORDER MATTERS HERE AND NOWHERE ELSE. Top-level FeatureScript
    lets one function call another declared later in the file; this rewrite
    turns each into a `const f = function(...)` STATEMENT inside one body, and
    a statement cannot call a const declared below it. So a caller must sit
    after its callee in the generated file -- which is why caseShellBuild is
    written before caseShellPair even though the pair reads as the outer idea.

    Simpler than `cad_layout._eval_wrapper` because the split earns it: there
    are no enums, no `precondition` and no synthesised definition map to get
    wrong, since everything that needs a human is below SPLIT_MARK and gets
    thrown away. What is left is one const and one function.
    """
    head = _strip_comments(fs.split(SPLIT_MARK)[0]).splitlines()
    out, i = [], 0
    while i < len(head):
        line = head[i]
        if line.startswith(("FeatureScript ", "import(")):
            i += 1
        elif m := re.match(r"^export function (\w+)\(", line):
            end = _block_end(head, i)
            blk = head[i:end + 1]
            blk[0] = blk[0].replace(f"export function {m.group(1)}(",
                                    f"const {m.group(1)} = function(", 1)
            blk[-1] += ";"
            out.extend(blk)
            i = end + 1
        else:
            out.append(re.sub(r"^export const ", "const ", line))
            i += 1

    default = next(iter(TABLE_FOR_CHECK))
    opt = ", ".join([f'"servo" : "{default}"'] +
                    [f'"{k}" : {TABLE_FOR_CHECK[default][k] * 1000:.4g} * millimeter'
                     for k in DIALOG])

    def case(tag: str, cs: str) -> str:
        return f"""
    {{
        const id = makeId("chk{tag}");
        servoMountBuild(context, id, {cs}, {{ {opt} }}, qNothing());
        // EVERY solid under `id`, not the query the build returned. The
        // returned query names the collar, so it reports one body whether or
        // not the pins actually merged into it -- which is exactly the failure
        // being looked for. Scoped to qCreatedBy so a body already in the tab
        // cannot inflate it; see the `check` tab note in config/onshape.yaml.
        const q = qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID);
        println("{tag}_bodies=" ~ toString(size(evaluateQuery(context, q))));
        println("{tag}_vol="    ~ toString(evVolume(context,
                {{ "entities" : q }}) / MM3));
        const b = evBox3d(context, {{ "topology" : q, "tight" : true }});
        println("{tag}_xmin=" ~ toString(b.minCorner[0] / millimeter));
        println("{tag}_xmax=" ~ toString(b.maxCorner[0] / millimeter));
        println("{tag}_zmin=" ~ toString(b.minCorner[2] / millimeter));
        println("{tag}_zmax=" ~ toString(b.maxCorner[2] / millimeter));
    }}"""

    body = "\n".join(out)
    # Case A is world-aligned so every bounding box is checkable against the
    # drawing. Case B is a rigid motion of it: the boxes are then meaningless
    # but the VOLUME must be identical, which is what catches geometry quietly
    # built about the world axes instead of about cs.
    a = case("A", "coordSystem(vector(0, 0, 0) * meter, "
                  "vector(1, 0, 0), vector(0, 0, 1))")
    b = case("B", "coordSystem(vector(37, -11, 5) * millimeter, "
                  "vector(2, 1, 2) / 3, vector(1, 2, -2) / 3)")

    copt = ", ".join([f'"servo" : "{default}"'] +
                     [f'"{k}" : {TABLE_FOR_CHECK[default][k] * 1000:.4g} * millimeter'
                      for k in CASE_DIALOG])

    def shell(tag: str, top: bool, bottom: bool, flip: bool, cs: str) -> str:
        flags = (f'"makeTop" : {str(top).lower()}, '
                 f'"makeBottom" : {str(bottom).lower()}, '
                 f'"flip" : {str(flip).lower()}')
        return f"""
    {{
        const id = makeId("chk{tag}");
        caseShellPair(context, id, {cs}, {{ {copt}, {flags} }});
        const q = qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID);
        println("{tag}_bodies=" ~ toString(size(evaluateQuery(context, q))));
        println("{tag}_vol="    ~ toString(evVolume(context,
                {{ "entities" : q }}) / MM3));
        const b = evBox3d(context, {{ "topology" : q, "tight" : true }});
        println("{tag}_xmin=" ~ toString(b.minCorner[0] / millimeter));
        println("{tag}_xmax=" ~ toString(b.maxCorner[0] / millimeter));
        println("{tag}_ymin=" ~ toString(b.minCorner[1] / millimeter));
        println("{tag}_ymax=" ~ toString(b.maxCorner[1] / millimeter));
        println("{tag}_zmin=" ~ toString(b.minCorner[2] / millimeter));
        println("{tag}_zmax=" ~ toString(b.maxCorner[2] / millimeter));
    }}"""

    world = ("coordSystem(vector(0, 0, 0) * meter, "
             "vector(1, 0, 0), vector(0, 0, 1))")
    c = shell("C", True, True, False, world)      # both halves
    d = shell("D", True, False, False, world)     # top only: the checkbox works
    # A rigid motion. The revolves are covered by B, but boxSolid is separate
    # machinery and an axis-aligned box is exactly the thing that can end up
    # built about the world by accident and never noticed.
    ee = shell("E", True, True, False,
               "coordSystem(vector(-19, 6, 23) * millimeter, "
               "vector(2, -1, 2) / 3, vector(2, 2, -1) / 3)")
    f = shell("F", True, True, True, world)       # flipped: Y must mirror
    return (f"function(context is Context, queries)\n{{\n{body}\n"
            f"    const MM3 = millimeter * millimeter * millimeter;\n"
            f"{a}\n{b}\n{c}\n{d}\n{ee}\n{f}\n"
            f'    return "ran to completion";\n}}\n')


TABLE_FOR_CHECK: dict = {}


def expected(table: dict[str, dict], servo: str = "XC330") -> dict[str, float]:
    """What the standalone build must measure, from the profile polygons.

    Deliberately assembled the same way the booleans are, term by term, rather
    than as one closed-form number: if the collar, the well, the pins and the
    grooves are each right and the assembly is wrong, this still says so.

    The three no-overlap facts it rests on, all true at the defaults and worth
    re-checking if the bolt circle or the collar ever moves: the pins sit
    entirely inside the well void, the grooves sit entirely in the roof above
    it, and no two grooves touch.
    """
    e = env(table, servo)
    n = int(e["n"])

    def vol(poly):
        return revolve_volume([(u.val(e), v.val(e)) for u, v in poly])

    total = (vol(collar_profile())
             - vol(bore_profile(clip=True))
             + n * vol(pin_profile())
             - n * vol(relief_profile()))
    return {
        "bodies": 1.0, "vol": total,
        "xmin": -e["collarR"], "xmax": e["collarR"],
        # The collar's rim is at -wellT, but the pins hang past it into the
        # horn's holes, so the lowest point is whichever of the two reaches
        # further. With caseOffset backing the rim off, that is now the pins.
        "zmin": -max(e["wellT"], e["pinL"]), "zmax": e["roof"],
    }


def _bvol(b) -> float:
    return (max(0.0, b[1] - b[0]) * max(0.0, b[3] - b[2])
            * max(0.0, b[5] - b[4]))


def _bisect(a, b):
    return (max(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]),
            min(a[3], b[3]), max(a[4], b[4]), min(a[5], b[5]))


def _case_boxes(e: dict[str, float], part: str):
    """(outer, [cavities]) for one half, in millimetres about the datum."""
    y0, sc, fc = e["endY"] - e["wrap"], e["sc"], e["faceClr"]
    if part == "TOP":
        seat = e["hornZ"] + fc
        outer = (-e["topOuter"], e["topOuter"], y0, e["endY"] + sc + e["topWall"],
                 e["hornZ"] - e["skirt"], seat + e["capT"])
        cavs = [(-e["inner"], e["inner"], y0 - e["over"], e["endY"] + sc,
                 e["hornZ"] - e["skirt"] - e["over"], seat)]
    else:
        seat = e["backZ"] - fc
        outY = e["endY"] + sc + e["topWall"] + e["nestClr"] + e["botWall"]
        zTop = e["backZ"] + e["grip"] + e["nest"]
        outer = (-e["botOuter"], e["botOuter"], y0, outY, seat - e["capT"], zTop)
        cavs = [(-e["inner"], e["inner"], y0 - e["over"], e["endY"] + sc,
                 seat, e["backZ"] + e["grip"]),
                (-e["nestBore"], e["nestBore"], y0 - e["over"],
                 e["endY"] + sc + e["topWall"] + e["nestClr"],
                 e["backZ"] + e["grip"], zTop + e["over"])]
    return outer, cavs


def expected_pair(table: dict[str, dict], halves: tuple[str, ...],
                  flip: bool = False, servo: str = "XC330") -> dict[str, float]:
    """What the pair feature must measure: boxes here, revolves from polygons.

    Rests on three no-overlap facts, all true at the defaults: the pins sit
    entirely inside the cavities, the reliefs entirely in the cap, and the grip
    and nest cavities meet at a plane without overlapping. Push the wrap, cap
    or grip far enough and any can break, at which point this over-predicts and
    the check says so.

    The two halves are separate bodies and never touch -- the nest carries
    0.1 a side -- so the volumes simply add.
    """
    e = case_env(table, servo)

    def vol(poly):
        return revolve_volume([(u.val(e), v.val(e)) for u, v in poly])

    per_half = 2 * vol(pin_profile()) - 2 * vol(relief_profile())
    total, boxes = 0.0, []
    for part in halves:
        outer, cavs = _case_boxes(e, part)
        total += (_bvol(outer) - sum(_bvol(_bisect(outer, c)) for c in cavs)
                  + per_half)
        boxes.append(outer)
    x0 = min(b[0] for b in boxes); x1 = max(b[1] for b in boxes)
    y0 = min(b[2] for b in boxes); y1 = max(b[3] for b in boxes)
    z0 = min(b[4] for b in boxes); z1 = max(b[5] for b in boxes)
    if flip:
        # 180 degrees about the datum's Z: (x, y) -> (-x, -y). X is symmetric
        # here so only Y actually moves, which is the point -- the shell wraps
        # the far end, and after a flip it must wrap the other way.
        x0, x1, y0, y1 = -x1, -x0, -y1, -y0
    return {"bodies": float(len(halves)), "vol": total,
            "xmin": x0, "xmax": x1, "ymin": y0, "ymax": y1,
            "zmin": z0, "zmax": z1}


def check(text: str, table: dict[str, dict], target: str | None) -> bool:
    from . import onshape

    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    got = dict(l.split("=", 1) for l in console.splitlines() if "=" in l)
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console)
        print(onshape.budget_line())
        return False

    ok = True
    plan = [("A", expected(table), True), ("B", expected(table), False),
            ("C", expected_pair(table, ("TOP", "BOTTOM")), True),
            ("D", expected_pair(table, ("TOP",)), True),
            ("E", expected_pair(table, ("TOP", "BOTTOM")), False),
            ("F", expected_pair(table, ("TOP", "BOTTOM"), flip=True), True)]
    print(f"  {'measurement':14} {'wanted':>10} {'got':>10}  (mm, mm^3)")
    for tag, want, boxes in plan:
        for k, w in want.items():
            if not boxes and k not in ("bodies", "vol"):
                continue        # rigid motion: the box is not invariant
            raw = got.get(f"{tag}_{k}")
            if raw is None:
                # A MISSING key is still a failure: the print never ran.
                print(f"  {tag}_{k:12} {'--':>10} {'MISSING':>10}  FAIL")
                ok = False
                continue
            v = float(raw)
            bad = abs(v - w) > max(0.002 * abs(w), 1e-4)
            ok = ok and not bad
            print(f"  {tag}_{k:12} {w:10.4f} {v:10.4f}  {'FAIL' if bad else 'ok'}")
    print(onshape.budget_line())
    return ok


def verify_studio(text: str, target: str | None) -> bool:
    """Compile the PUSHED studio and list what it defines. One billable call.

    This is the only check that sees the UI layer. `--check` rewrites the file
    into a bare function and throws away everything below SPLIT_MARK -- enums,
    precondition, defineFeature, every annotation -- so a mistake in the dialog
    compiles nowhere and `--check` still reports green. That happened on
    2026-08-25: a doubled brace in the case Fit group put literal `{{` into the
    studio, every feature in it went red at once, and the check had said ok.

    `featurespecs` compiles the whole studio server-side and returns one spec
    per feature. A studio that does not compile cannot produce them, so the
    count is the verdict: fewer specs than the file defines means broken.

    Note this reports on the STUDIO, not on any inserted instance. A feature
    that compiles but throws when it runs shows up in the Part Studio that
    uses it, under `featureStates` on GET /partstudios/.../features.
    """
    import json
    from . import onshape

    want = text.count('"Feature Type Name"')
    url = onshape.resolve(target, "horn_features")
    did, wvm, wid, eid = onshape.parse_url(url)
    try:
        raw, _ = onshape._call(
            "GET", f"/featurestudios/d/{did}/{wvm}/{wid}/e/{eid}/featurespecs",
            what="compile-verify the pushed studio", doc=did, elem=eid)
    except onshape.OnshapeError as e:
        print(f"  studio did NOT compile: {str(e).splitlines()[0]}")
        return False
    specs = json.loads(raw).get("featureSpecs") or []
    for sp in specs:
        m = sp.get("message", sp)
        print(f"  defines {m.get('featureTypeName'):22} "
              f"{len(m.get('parameters') or []):2} parameters")
    if len(specs) != want:
        print(f"  MISMATCH: the file defines {want} features, the studio "
              f"compiled {len(specs)}")
        return False
    return True


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=CAD_PARAMS)
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044",
                    help="std version. A studio pinned older behaves "
                         "differently; --check always runs at the CURRENT one")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="",
                    default=None,
                    help="compile and RUN the geometry layer, then measure it. "
                         "ONE billable call; defaults to the `check` tab")
    ap.add_argument("--push", metavar="TAB|URL", nargs="?", const="",
                    default=None,
                    help="replace a Feature Studio's contents. NOT the "
                         "`feature_studio` tab -- cad_layout owns that one")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the check script instead of spending a call")
    args = ap.parse_args()

    table = servo_table(load_params(args.params))
    if not table:
        raise SystemExit("no servo has a complete horn fastener interface")
    globals()["TABLE_FOR_CHECK"] = table
    text = build_fs(table, args.fs_version)
    lint_fs(text)
    Path(args.output).write_text(text)
    print(f"wrote {len(text)} chars -> {args.output}  "
          f"({', '.join(table)})")

    if args.dry_run:
        print(check_wrapper(text))
        return
    if args.check is not None and not check(text, table, args.check or None):
        raise SystemExit("check FAILED -- not pushing")
    if args.push is not None:
        from . import onshape
        if args.push in ("", "feature_studio"):
            raise SystemExit(
                "refusing to push at `feature_studio`: cad_layout overwrites "
                "that tab wholesale. Give this feature its own studio tab.")
        url = onshape.resolve(args.push, args.push)
        onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}")
        # Always, not on a flag. A push cannot fail on bad FeatureScript -- the
        # contents endpoint takes any text -- so without this a broken studio
        # lands silently and is found by a human seeing red in the tree.
        ok = verify_studio(text, args.push)
        print(onshape.budget_line())
        if not ok:
            raise SystemExit("pushed, but the studio does not compile")


if __name__ == "__main__":
    main()
