"""The yaw-roll AHRS fixture: every printed part, placed, joined and checked.

What `analysis/ahrs_fixture.py` runs on. Generates ONE custom feature,
`AHRS fixture`, into its own Feature Studio; inserted into the `ahrs_fixture`
Part Studio it builds the printed parts round the hardware envelopes:

    part              print   what it is
    yaw base          Z+      back-half case shell on a plate; clamp tab to +X,
                              four 10-32 clearance holes
    yaw cover         Z-      horn-half case shell
    yaw idler         Z+      stepped plug; arm to -X; leg up the shaft end
    yaw arm           Z-      horn pins; screw down into the idler leg; nut
                              for the roll base's screw along X
    roll base         X+      back-half case shell; leg down to the yaw arm;
                              a cable clip each side
    roll cover        X-      horn-half case shell
    roll idler        X+      stepped plug; arm up; leg along +X over the servo
    roll horn mount   X-      horn pins; screw into the idler leg; nut for the
                              TM151 mount's screw, which is ON the roll axis
    TM151 mount       Z+      the 0 mount, or the d mount -- which, turned
                              half a turn about the roll axis, is also +d

"Z-" means the part prints with its -Z side facing up. Every mounting pin
points up as printed, which is what fixes each orientation.

THE SHAPE. The yaw servo's far end points +X, so the U from its horn to its
idler goes round the SHAFT end (9.5 mm to the case end, not 24.5), on the side
where the roll servo comes down to meet it. The roll servo's far end points
down and its horn faces +X; the TM151 sits beyond the horn on the roll axis,
centred over the yaw axis, so both axes pass through the 0 mount's sensing
point. Both servos stay centred at 180 deg: the idler U's cannot turn half a
revolution, so the +-d swap is the TM151 mount turned over, not the servo.

EVERY SCREW JOINT is the as-built crank/idler one -- a 6-32 x 3/8" flat head,
the countersunk part 4.6 thick at the screw, a small-pattern nut in a slot
2 mm past the joint -- plus a 45 deg locating ridge in a groove across the
joint, which stops it rotating about the screw. 45 deg so it prints in any
orientation. Nut slots print as bridges; their supports wait for the layout.

DERIVED, never typed: the U legs' radius (outside the case's swept corner),
the roll horn's distance from the yaw axis (the TM151 lands centred), and the
roll axis height (the swept roll stage clears the yaw arm by
`stage_clearance`).

    python -m aow_sim.cad_ahrs_fixture                 # write docs/cad/ahrs_fixture.fs
    python -m aow_sim.cad_ahrs_fixture --check         # parts, clashes over the travel, print check
    python -m aow_sim.cad_ahrs_fixture --push fixture_features
    python -m aow_sim.cad_ahrs_fixture --shot          # render -> docs/cad/ahrs_fixture.png

Numbers: config/ahrs_fixture_cad.yaml (TM151 and the fixture), plus the X330
envelope and mount interface cad_servo_mount already reads.
"""

from __future__ import annotations

import argparse
import itertools
import math
from pathlib import Path

import numpy as np
import yaml

from . import cad_servo_mount as sm
from .params import _normalize, load_params

FIXTURE_PARAMS = "config/ahrs_fixture_cad.yaml"
OUT_FS = "docs/cad/ahrs_fixture.fs"
OUT_PNG = "docs/cad/ahrs_fixture.png"
SPLIT_MARK = "// ==== UI LAYER BELOW -- dropped by --check ===="
SERVO_KEY = "xc330_t181"    # the X330 case: XC330 and XL330 share it
CONFIGS = ("BELOW", "ZERO", "ABOVE")
PARTS = ("yaw base", "yaw cover", "yaw idler", "yaw arm", "roll base",
         "roll cover", "roll idler", "roll horn mount", "TM151 mount")
# Pairs allowed to overlap. Only the keep-out, which is a space and not a
# part: the servo envelopes carry the real pin holes and the idler recess,
# so every pin, shell, horn well and idler plug is CHECKED against them.
INTENDED = (("TM151", "USB keep-out"),)
# NOT listed: the TM151 mount against the TM151. Its pins are Phi 1.9 in the
# board's Phi 2.10 holes and shorter than the board, so the check proves they
# fit rather than excusing them.
POSES = (-45, -30, -15, 15, 30, 45)


def load(fixture_path: str = FIXTURE_PARAMS, cad_path: str = sm.CAD_PARAMS,
         mount_path: str = sm.MOUNT_PARAMS) -> dict:
    """Everything the fixture reads, in MILLIMETRES (degrees for travel)."""
    raw = _normalize(yaml.safe_load(Path(fixture_path).read_text()))
    params = load_params(cad_path)
    mounts = sm.load_mounts(mount_path)
    servo = params["servos"][SERVO_KEY]
    d, w, h = servo["box_size"]
    mm = lambda v: v * 1000.0    # noqa: E731
    tm = {k: ([mm(x) for x in v] if isinstance(v, list) else mm(v))
          for k, v in raw["tm151"].items()}
    fx = {k: (v if k == "travel_deg" else mm(v)) for k, v in raw["fixture"].items()}
    x330 = {"caseDepth": mm(d), "caseWidth": mm(w), "caseHeight": mm(h),
            "shaftFromEnd": mm(servo["shaft_from_end"]),
            "hornThickness": mm(servo["horn_thickness"]),
            "hornDiameter": mm(servo["horn_diameter"])}
    return {"tm151": tm, "fixture": fx, "x330": x330,
            "table": sm.servo_table(params, mounts),
            "screw": sm.screw_table(mounts)}


# --------------------------------------------------------------------------
# The layout in Python: what the FeatureScript derives, derived independently.
# --------------------------------------------------------------------------

def swept_depth(r_down: float, half_w: float, travel_deg: float) -> float:
    """How far below the roll axis a section reaches over +-travel.

    The section is a rectangle whose far edge is `r_down` below the axis
    (negative: above it) and `half_w` either side; its lowest point over the
    swing is a corner, at r cos(th) + w |sin(th)|. That peaks at the hypotenuse
    when the corner's own angle is inside the travel, else at the travel's end.
    """
    T = math.radians(travel_deg)
    if r_down > 0 and math.atan2(half_w, r_down) <= T:
        return math.hypot(r_down, half_w)
    return max(r_down, r_down * math.cos(T) + half_w * math.sin(T))


def plate_half_width(data: dict) -> float:
    """The TM151 mount plate's half width: the board's, or wider so each
    pin's root-relief groove keeps `mount_rim` of plate outside it."""
    tm, f = data["tm151"], data["fixture"]
    relief = data["table"]["XC330"]["casePinReliefDia"] * 1000 / 2
    return max(tm["board_width"] / 2, tm["hole_span_y"] / 2 + relief + f["mount_rim"])


def shell_corner(data: dict) -> float:
    """Radius from the shaft of the full-wrap COVER's outer corner at the
    shaft end -- the thing a U leg swings round. The base keeps the far-end
    wrap only, so the cover is what reaches round the shaft end."""
    t = {k: v * 1000 for k, v in data["table"]["XC330"].items() if k != "hornHoleCount"}
    walls = t["caseSideClearance"] + t["caseTopWall"]
    return math.hypot(data["x330"]["shaftFromEnd"] + walls, data["x330"]["caseWidth"] / 2 + walls)


def layout(data: dict, offset: float | None = None) -> dict:
    """The derived dimensions, world millimetres."""
    tm, f, sv = data["tm151"], data["fixture"], data["x330"]
    t = data["table"]["XC330"]
    d = f["ahrs_offset"] if offset is None else offset
    s, p, hw = tm["sensor_height"], f["arm_plate"], plate_half_width(data)
    T = f["travel_deg"]
    depth = max(swept_depth(d + s + p, hw, T),            # the d mount, below
                swept_depth(f["mount_wall_top"], hw, T),  # it turned over: its wall
                swept_depth(s + p, hw, T),                # the 0 mount's plate
                t["collarOuterDia"] * 1000 / 2)           # the horn mount's disc
    corner = shell_corner(data)
    half_len = tm["board_length"] / 2 - tm["housing_offset_x"]   # to the header edge
    return {
        "d": d,
        "H": depth + f["yaw_arm_top"] + f["stage_clearance"],
        "R_s": corner + f["sweep_clearance"] + f["leg_width"] / 2,
        "g": f["horn_mount_thickness"] + f["joint_plate"] + f["tm151_gap"] + half_len,
    }


def base_tab(data: dict) -> dict:
    """The yaw base plate's x extent and its four bolt-hole centres, world mm.
    The FeatureScript derives the same; this is what the test holds it to."""
    f, sv = data["fixture"], data["x330"]
    t = {k: v * 1000 for k, v in data["table"]["XC330"].items() if k != "hornHoleCount"}
    far = sv["caseHeight"] - sv["shaftFromEnd"]
    shell_y1 = far + t["caseSideClearance"] + t["caseTopWall"] + t["caseNestClearance"] + t["caseBottomWall"]
    back = sv["hornThickness"] + sv["caseDepth"]
    cap_out = -back - t["caseFaceClearance"] - t["caseCapThickness"]
    x_tab = layout(data)["g"] - cap_out + f["tab_beyond"]
    xc = (shell_y1 + x_tab) / 2
    holes = [(xc + sx * f["bolt_span_x"] / 2, sy * f["bolt_span_y"] / 2)
             for sx in (1, -1) for sy in (1, -1)]
    return {"x0": -shell_y1, "x1": x_tab, "shell_end": shell_y1, "holes": holes}


def tm151_boxes(tm: dict) -> dict[str, tuple]:
    """The TM151's boxes in its own sensor frame: (x0, x1, y0, y1, z0, z1)."""
    s, bt = tm["sensor_height"], tm["board_thickness"]
    cx = -tm["housing_offset_x"]                 # the board's centre
    L, W = tm["board_length"], tm["board_width"]
    zb, zt = -s, -s + bt
    edge = cx - L / 2                            # the USB (-x) edge
    ko = tm["usb_plug_keepout"]
    uy = tm["usb_offset_y"]
    return {
        "board":   (cx - L / 2, cx + L / 2, -W / 2, W / 2, zb, zt),
        "housing": (-tm["housing_length"] / 2, tm["housing_length"] / 2,
                    -tm["housing_width"] / 2, tm["housing_width"] / 2,
                    zt, zt + tm["housing_height"]),
        "usb":     (edge, edge + tm["usb_length"], uy - tm["usb_width"] / 2,
                    uy + tm["usb_width"] / 2, zt, zt + tm["usb_height"]),
        # outboard of the board edge only: the plug, not the receptacle
        "keepout": (edge - ko[0], edge, uy - ko[1] / 2, uy + ko[1] / 2,
                    zt + tm["usb_height"] / 2 - ko[2] / 2,
                    zt + tm["usb_height"] / 2 + ko[2] / 2),
    }


def mount_boxes(data: dict, config: str, offset: float | None = None) -> list[tuple]:
    """TM151 mount (plate, wall) and TM151 (board, housing) as WORLD boxes at
    the NATIVE pose -- ABOVE is BELOW turned about the roll axis, which the
    caller does. Each box is (x0, x1, y0, y1, z0, z1)."""
    tm, f = data["tm151"], data["fixture"]
    L = layout(data, offset)
    dd = 0.0 if config == "ZERO" else L["d"]
    H, s, p = L["H"], tm["sensor_height"], f["arm_plate"]
    x0 = -L["g"] + f["horn_mount_thickness"]
    edge = tm["housing_offset_x"] + tm["board_length"] / 2     # USB edge, world +X
    hw = plate_half_width(data)
    zb = H - dd - s
    out = [(x0, edge, -hw, hw, zb - p, zb),                          # plate
           (x0, x0 + f["joint_plate"], -hw, hw, zb - p, H + f["mount_wall_top"])]
    # the TM151: sensor x = world -X, z = world +Z, origin (0, 0, H - dd)
    for k in ("board", "housing"):
        b = tm151_boxes(tm)[k]
        out.append((-b[1], -b[0], -b[3], -b[2], H - dd + b[4], H - dd + b[5]))
    return out


def _corners(b):
    return np.array(list(itertools.product(b[0:2], b[2:4], b[4:6])))


def _rot_x(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def lowest_swept(data: dict, config: str, offset: float | None = None,
                 n: int = 721) -> float:
    """Lowest world z the TM151 mount and TM151 reach over the travel."""
    L = layout(data, offset)
    axis = np.array([0.0, 0.0, L["H"]])
    pts = np.vstack([_corners(b) for b in mount_boxes(data, config, offset)]) - axis
    if config == "ABOVE":
        pts = pts @ _rot_x(180).T
    T = data["fixture"]["travel_deg"]
    return min(((pts @ _rot_x(a).T) + axis)[:, 2].min()
               for a in np.linspace(-T, T, n))


# --------------------------------------------------------------------------
# The FeatureScript
# --------------------------------------------------------------------------

def _fs_map(name: str, d: dict) -> str:
    rows = []
    for k, v in d.items():
        if isinstance(v, str):
            rows.append(f'    "{k}" : "{v}"')
        elif isinstance(v, list):
            rows.append(f'    "{k}" : [' + ", ".join(f"{x:.4g} * millimeter"
                                                     for x in v) + "]")
        elif k in ("travel_deg", "cskAngle"):
            rows.append(f'    "{k}" : {v:g} * degree')
        else:
            rows.append(f'    "{k}" : {v:.4g} * millimeter')
    return f"export const {name} = {{\n" + ",\n".join(rows) + "\n};"


def _servo_mount_layer(data: dict, fs_version: str) -> str:
    """cad_servo_mount's geometry layer, verbatim, header stripped: the horn
    pin, case shell, idler and screw functions every part here is built with.
    Copied rather than imported -- an in-document import pins a microversion,
    which would have to be re-typed after every push of that studio."""
    text = sm.build_fs(data["table"], fs_version, data["screw"])
    head = text.split(sm.SPLIT_MARK)[0]
    head = head[head.index("*/") + 2:]           # drop its GENERATED banner
    return "\n".join(l for l in head.splitlines()
                     if not l.startswith(("FeatureScript ", "import(")))


FS = r'''FeatureScript %VERSION%;
import(path : "onshape/std/geometry.fs", version : "%VERSION%.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_ahrs_fixture --push fixture_features
 *
 * Numbers from config/ahrs_fixture_cad.yaml, config/servo_mounts.yaml and
 * bike_params_cad.yaml. Millimetres.
 *
 * WORLD FRAME: origin on the yaw horn's outer face, on the yaw axis; +Z up
 * the yaw axis; +X along the roll axis, from the roll servo toward the TM151.
 */

%TM151%

%X330%

%FIXTURE%

%HORN_OPT%

%CASE_OPT%

%IDLER_OPT%

%SCREW_OPT%

// ---- the servo-mount geometry, copied from the horn-mount-gen studio ----
%SERVO_MOUNT%
// ---- end of the copy ----

/** A box between two corners given in `cs`. One body, created by `id`. */
export function boxIn(context is Context, id is Id, cs is CoordSystem,
                      lo is Vector, hi is Vector) returns Query
{
    fCuboid(context, id, { "corner1" : lo, "corner2" : hi });
    opTransform(context, id + "xf", {
            "bodies"    : qCreatedBy(id, EntityType.BODY),
            "transform" : toWorld(cs) });
    return qCreatedBy(id, EntityType.BODY);
}

/** A world-aligned cylinder between two points. */
export function cylW(context is Context, id is Id, p0 is Vector, p1 is Vector,
                     r is ValueWithUnits) returns Query
{
    fCylinder(context, id, { "bottomCenter" : p0, "topCenter" : p1, "radius" : r });
    return qCreatedBy(id, EntityType.BODY);
}

/** A world-aligned box. */
export function boxW(context is Context, id is Id, lo is Vector, hi is Vector) returns Query
{
    fCuboid(context, id, { "corner1" : lo, "corner2" : hi });
    return qCreatedBy(id, EntityType.BODY);
}

/** A cylinder between two axis points given in `cs`. */
export function cylIn(context is Context, id is Id, cs is CoordSystem,
                      p0 is Vector, p1 is Vector, r is ValueWithUnits) returns Query
{
    fCylinder(context, id, { "bottomCenter" : toWorld(cs, p0),
                             "topCenter"    : toWorld(cs, p1),
                             "radius"       : r });
    return qCreatedBy(id, EntityType.BODY);
}

export function unite(context is Context, id is Id, qs is array) returns Query
{
    opBoolean(context, id, { "tools" : qUnion(qs),
                             "operationType" : BooleanOperationType.UNION });
    return qUnion(qs);
}

/**
 * The body under `scope` containing `pt`. How a part is found again after
 * booleans: a union keeps ONE of its tools' identities and which one is not
 * specified, so a query naming the primitive it was started from can come
 * back empty. A point inside the part cannot.
 */
export function partAt(context is Context, scope is Id, pt is Vector) returns Query
{
    return qContainsPoint(qBodyType(qCreatedBy(scope, EntityType.BODY),
                                    BodyType.SOLID), pt);
}

/** Name, colour and describe a body. */
export function dress(context is Context, q is Query, name is string, c is Color,
                      note is string)
{
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME,
                           "value" : name });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE,
                           "value" : c });
    if (note != "")
        setProperty(context, { "entities" : q,
                               "propertyType" : PropertyType.DESCRIPTION,
                               "value" : note });
}

/**
 * The 45 deg locating ridge: a flat-topped prism along `along`, standing
 * `h` off the joint plane at `origin` in direction `up`, plus a millimetre
 * of root below the plane so it unions into its part. `grow` offsets both
 * flanks outward, normal to themselves -- the groove is the ridge grown by
 * the clearance.
 */
export function ridgeSolid(context is Context, id is Id, origin is Vector,
                           along is Vector, up is Vector, h is ValueWithUnits,
                           grow is ValueWithUnits, half is ValueWithUnits) returns Query
{
    const side = cross(up, along);
    const flat = FIXTURE.ridge_flat / 2;
    // 45 deg flanks up to a flat the hole's width; `grow` offsets every face
    // outward along its own normal, which moves the flanks' feet out by
    // grow * sqrt(2) and the flat's edges by grow * (sqrt(2) - 1)
    const a  = flat + h + grow * sqrt(2);
    const tt = flat + grow * (sqrt(2) - 1);
    const mm = millimeter;
    var sk = newSketchOnPlane(context, id + "sk", {
            "sketchPlane" : plane(origin - half * along, along, side) });
    skPolygon(sk, [vector(-a, -1 * mm), vector(a, -1 * mm), vector(a, 0 * mm),
                   vector(tt, h + grow), vector(-tt, h + grow), vector(-a, 0 * mm)]);
    skSolve(sk);
    opExtrude(context, id + "ext", {
            "entities"  : qSketchRegion(id + "sk"),
            "direction" : along,
            "endBound"  : BoundingType.BLIND,
            "endDepth"  : 2 * half });
    opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
    // Both ends cut back at 45 deg from the root, so a ridge standing on
    // end as printed has no flat ledge under it (the TM151 mount's did).
    // Sketched in the (along, up) plane, extruded across the whole width.
    const W = a + 1 * mm;
    for (var e in [1, -1])
    {
        const tag = e > 0 ? "endP" : "endN";
        const hs  = h + grow + 2 * mm;
        // normal -e*side with x along e*along keeps v = +up at both ends
        // (side x along = -up); the cut is everything past the 45 deg line
        // through the ridge's foot at the end, v = half - u
        polyPrism(context, id, tag, origin + e * W * side, -e * side, e * along,
                  [vector(half + 1 * mm, -1 * mm), vector(half + 3 * mm, -1 * mm),
                   vector(half + 3 * mm, hs), vector(half - hs, hs)], 2 * W);
    }
    opBoolean(context, id + "taper", {
            "tools" : qUnion([qCreatedBy(id + "endPExt", EntityType.BODY),
                              qCreatedBy(id + "endNExt", EntityType.BODY)]),
            "targets" : qCreatedBy(id + "ext", EntityType.BODY),
            "operationType" : BooleanOperationType.SUBTRACTION });
    return qCreatedBy(id + "ext", EntityType.BODY);
}

/**
 * One 6-32 joint. `head` is the countersink's centre on the countersunk
 * part's outer face, `zIn` points along the shank, and the joint plane is
 * joint_plate further on. The nut slot is 2 mm past the joint in the nut
 * part and runs out along `slotDir`. `pocket` carries the head's bore on
 * outward, for a countersunk part thicker than joint_plate at the screw.
 * The ridge goes on whichever part prints it better; the other gets the
 * groove.
 */
export function screwJoint(context is Context, id is Id, head is Vector,
                           zIn is Vector, slotDir is Vector, pocket is ValueWithUnits,
                           cskPart is Query, nutPart is Query, ridgeOnNut is boolean,
                           ridgeDir is Vector, ridgeHalf is ValueWithUnits,
                           cskUp is Vector, nutUp is Vector)
{
    const f  = FIXTURE;
    const mm = millimeter;
    const cs = coordSystem(head, slotDir, zIn);
    const nd = f.joint_plate + 2 * mm;
    // The slot runs clean THROUGH the nut part both ways -- easier to clear
    // after printing -- and is cut from the nut part only, so it can never
    // open a slot through the countersunk part further along.
    screwJointGeometry(context, id + "scr", cs, mergeMaps(SCREW_OPT, {
            "nutDepth" : nd, "headPocket" : pocket, "bothWays" : true,
            "nutSlotLength" : 200 * mm }));
    const bore = qCreatedBy(id + "scr" + "screwRev", EntityType.BODY);
    const slot = qCreatedBy(id + "scr" + "slotExt", EntityType.BODY);
    const jp  = head + f.joint_plate * zIn;
    const up  = ridgeOnNut ? -zIn : zIn;
    const ridgePart = ridgeOnNut ? nutPart : cskPart;
    const other     = ridgeOnNut ? cskPart : nutPart;
    const r = ridgeSolid(context, id + "ridge", jp, ridgeDir, up, f.ridge_height,
                         0 * millimeter, ridgeHalf);
    opBoolean(context, id + "rAdd", { "tools" : qUnion([ridgePart, r]),
            "operationType" : BooleanOperationType.UNION });
    const g = ridgeSolid(context, id + "groove", jp, ridgeDir, up, f.ridge_height,
                         f.ridge_clearance, ridgeHalf + 1 * millimeter);
    opBoolean(context, id + "gCut", { "tools" : g, "targets" : other,
            "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "slotCut", { "tools" : slot, "targets" : nutPart,
            "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "cut", { "tools" : bore,
            "targets" : qUnion([cskPart, nutPart]),
            "operationType" : BooleanOperationType.SUBTRACTION });

    // TEARDROP the bore in any part where it lies horizontal as printed: a
    // 45 deg point on its crown, so the hole's roof is not a flat overhang.
    // The countersink needs none -- a 90 deg cone on a horizontal axis is a
    // 45 deg overhang at its crown already.
    const rB = SCREW_OPT.holeDia / 2;
    const parts = [[cskPart, cskUp, "tdC"], [nutPart, nutUp, "tdN"]];
    for (var pu in parts)
    {
        if (abs(dot(pu[1], zIn)) > 0.5)
            continue;
        const upP = normalize(pu[1] - dot(pu[1], zIn) * zIn);
        polyPrism(context, id, pu[2], head - 1 * mm * zIn, zIn, cross(upP, zIn),
                  [vector(0 * mm, 0 * mm), vector(rB / sqrt(2), rB / sqrt(2)),
                   vector(0 * mm, rB * sqrt(2)), vector(-rB / sqrt(2), rB / sqrt(2))],
                  SCREW_OPT.holeDepth + 1 * mm);
        opBoolean(context, id + (pu[2] ~ "Cut"), {
                "tools" : qCreatedBy(id + (pu[2] ~ "Ext"), EntityType.BODY),
                "targets" : pu[0],
                "operationType" : BooleanOperationType.SUBTRACTION });
    }

    // Sacrificial bridging, when the slot's roof is a ceiling as the nut part
    // prints: layer 1 bridges the slot except a hole-wide strip across it,
    // layer 2 bridges that strip except the hole's square. Drilled or poked
    // out after printing. A slot standing on end (nutUp across the screw)
    // prints without a roof and gets none.
    const along = dot(nutUp, zIn);
    if (abs(along) > 0.5)
    {
        const L  = f.bridge_layer;
        const rH = SCREW_OPT.holeDia / 2;
        const w  = SCREW_OPT.nutSlotWidth;
        const d0 = along > 0 ? nd + SCREW_OPT.nutSlotThickness : nd;  // the roof
        const sg = along > 0 ? 1 : -1;                                 // up, in depth
        const l1 = boxIn(context, id + "br1", cs, vector(-rH, -w / 2, d0 + (sg > 0 ? 0 * mm : -L)),
                         vector(rH, w / 2, d0 + (sg > 0 ? L : 0 * mm)));
        const l2 = boxIn(context, id + "br2", cs, vector(-rH, -rH, d0 + sg * L + (sg > 0 ? 0 * mm : -L)),
                         vector(rH, rH, d0 + sg * L + (sg > 0 ? L : 0 * mm)));
        // Both layers only REMOVE: the round hole already sits inside the
        // strip and the square, so cutting them leaves layer 1 open across
        // the strip and layer 2 open over the square, and nothing else.
        opBoolean(context, id + "brCut", { "tools" : qUnion([l1, l2]), "targets" : nutPart,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }
}

/**
 * An X330 envelope about its horn datum, as TWO bodies -- the case, with the
 * back-face idler recess and the pin holes cut in it, and the horn with its
 * four -- because the horn turns with the output and the case does not, and
 * the clash sweep needs to know. The holes are what let the check test the
 * pins and shells instead of excusing them.
 */
export function x330Envelope(context is Context, id is Id, cs is CoordSystem) returns map
{
    const t  = X330;
    const st = SERVO_MOUNT_TABLE["XC330"];
    const hw = t.caseWidth / 2;
    const zc = -t.hornThickness;
    const zb = zc - t.caseDepth;
    const mm = millimeter;
    const caseBody = boxIn(context, id + "case", cs, vector(-hw, -t.shaftFromEnd, zb),
                      vector(hw, t.caseHeight - t.shaftFromEnd, zc));
    const r1 = cylIn(context, id + "r1", cs, vector(0 * mm, 0 * mm, zb - 1 * mm),
                     vector(0 * mm, 0 * mm, zb + st.idlerRecessOuterDepth),
                     st.idlerRecessOuterDia / 2);
    const r2 = cylIn(context, id + "r2", cs, vector(0 * mm, 0 * mm, zb),
                     vector(0 * mm, 0 * mm, zb + st.idlerRecessOuterDepth + st.idlerRecessInnerDepth),
                     st.idlerRecessInnerDia / 2);
    // the four case holes the shells' pins go into -- only the far row is
    // used -- as the Phi 2 relief bores, horn face and back face
    const rowX = st.caseHoleSpanX / 2;
    const rowY = t.caseHeight - t.shaftFromEnd - t.caseHeight / 2 + st.casePinRowOffset;
    var holes = [r1, r2];
    for (var sx in [1, -1])
    {
        holes = append(holes, cylIn(context, id + ("hf" ~ (sx > 0 ? "p" : "n")), cs,
                vector(sx * rowX, rowY, zc - st.caseReliefDepthHorn), vector(sx * rowX, rowY, zc + 1 * mm),
                st.caseReliefDia / 2));
        holes = append(holes, cylIn(context, id + ("hb" ~ (sx > 0 ? "p" : "n")), cs,
                vector(sx * rowX, rowY, zb - 1 * mm), vector(sx * rowX, rowY, zb + st.caseReliefDepthBack),
                st.caseReliefDia / 2));
    }
    opBoolean(context, id + "recess", { "tools" : qUnion(holes), "targets" : caseBody,
            "operationType" : BooleanOperationType.SUBTRACTION });
    const horn = cylIn(context, id + "horn", cs, vector(0, 0, 0) * mm,
                       vector(0 * mm, 0 * mm, zc), t.hornDiameter / 2);
    // and the horn's four, where the horn pins go
    var hh = [];
    for (var i = 0; i < st.hornHoleCount; i += 1)
    {
        const a = i * (360 / st.hornHoleCount) * degree;
        const c = st.hornBoltCircle / 2;
        hh = append(hh, cylIn(context, id + ("hh" ~ i), cs,
                vector(c * cos(a), c * sin(a), -st.hornHoleDepthMax),
                vector(c * cos(a), c * sin(a), 1 * mm), st.hornHoleDia / 2));
    }
    opBoolean(context, id + "hornHoles", { "tools" : qUnion(hh), "targets" : horn,
            "operationType" : BooleanOperationType.SUBTRACTION });
    return { "caseQ" : caseBody, "horn" : horn };
}

/**
 * The TM151 about its SENSOR FRAME (the datasheet's axes; origin the assumed
 * sensing point, `s` above the board's bottom face): board with its four
 * holes, housing and USB-C receptacle as one body; the plug keep-out, outboard
 * of the board edge, as another.
 */
export function tm151Envelope(context is Context, id is Id, cs is CoordSystem,
                              s is ValueWithUnits) returns map
{
    const t  = TM151;
    const cx = -t.housing_offset_x;
    const zb = -s;
    const zt = zb + t.board_thickness;
    const edge = cx - t.board_length / 2;
    const mm = millimeter;
    const board = boxIn(context, id + "board", cs, vector(edge, -t.board_width / 2, zb),
                        vector(cx + t.board_length / 2, t.board_width / 2, zt));
    var holes = [];
    for (var i = 0; i < 4; i += 1)
    {
        const hx = cx + t.hole_offset_x + ((i % 2 == 0) ? 1 : -1) * t.hole_span_x / 2;
        const hy = ((i < 2) ? 1 : -1) * t.hole_span_y / 2;
        holes = append(holes, cylIn(context, id + ("hole" ~ i), cs,
                vector(hx, hy, zb - 1 * mm), vector(hx, hy, zt + 1 * mm), t.hole_dia / 2));
    }
    opBoolean(context, id + "drill", { "tools" : qUnion(holes), "targets" : board,
            "operationType" : BooleanOperationType.SUBTRACTION });
    const housing = boxIn(context, id + "housing", cs,
          vector(-t.housing_length / 2, -t.housing_width / 2, zt),
          vector(t.housing_length / 2, t.housing_width / 2, zt + t.housing_height));
    const usb = boxIn(context, id + "usb", cs,
          vector(edge, t.usb_offset_y - t.usb_width / 2, zt),
          vector(edge + t.usb_length, t.usb_offset_y + t.usb_width / 2, zt + t.usb_height));
    const body = unite(context, id + "u", [board, housing, usb]);
    const ko = t.usb_plug_keepout;
    const zm = zt + t.usb_height / 2;
    const keep = boxIn(context, id + "keepout", cs,
          vector(edge - ko[0], t.usb_offset_y - ko[1] / 2, zm - ko[2] / 2),
          vector(edge, t.usb_offset_y + ko[1] / 2, zm + ko[2] / 2));
    return { "body" : body, "keepout" : keep };
}

/** Lowest point below the roll axis of a swept section; see layout(). */
export function sweptDepth(rDown is ValueWithUnits, halfW is ValueWithUnits,
                           travel is ValueWithUnits) returns ValueWithUnits
{
    if (rDown > 0 * meter && atan2(halfW, rDown) <= travel)
        return sqrt(rDown * rDown + halfW * halfW);
    return max(rDown, rDown * cos(travel) + halfW * sin(travel));
}

/** Every derived dimension, from the offset and the sensing-point guess. */
export function plateHalfWidth() returns ValueWithUnits
{
    // the board's, or wider so each pin's root-relief groove keeps mount_rim
    // of plate outside it
    return max(TM151.board_width / 2, TM151.hole_span_y / 2
               + CASE_OPT.casePinReliefDia / 2 + FIXTURE.mount_rim);
}

export function fixtureLayout(d is ValueWithUnits, s is ValueWithUnits) returns map
{
    const f  = FIXTURE;
    const t  = TM151;
    const x  = X330;
    const hw = plateHalfWidth();
    const T  = f.travel_deg;
    const p  = f.arm_plate;
    const depth = max(max(sweptDepth(d + s + p, hw, T), sweptDepth(f.mount_wall_top, hw, T)),
                      max(sweptDepth(s + p, hw, T), HORN_OPT.collarOuterDia / 2));
    // the full-wrap COVER's outer corner at the shaft end: what a U leg
    // swings round (the base keeps the far-end wrap only)
    const walls  = CASE_OPT.caseSideClearance + CASE_OPT.caseTopWall;
    const cy = x.shaftFromEnd + walls;
    const cx = x.caseWidth / 2 + walls;
    const corner = sqrt(cy * cy + cx * cx);
    return {
        "H"   : depth + f.yaw_arm_top + f.stage_clearance,
        "R_s" : corner + f.sweep_clearance + f.leg_width / 2,
        "g"   : f.horn_mount_thickness + f.joint_plate + f.tm151_gap
                + t.board_length / 2 - t.housing_offset_x
    };
}

/**
 * Every part and envelope. `config` is BELOW (the d mount as printed, TM151
 * under the roll axis), ZERO, or ABOVE (the d mount turned half a turn).
 *
 * Returns the derived layout, and for the check: which bodies move with
 * which joint, and each printed part with the direction that prints UP.
 */
export function ahrsFixtureBuild(context is Context, id is Id, opt is map) returns map
{
    const f  = FIXTURE;
    const t  = TM151;
    const x  = X330;
    const L  = fixtureLayout(opt.ahrsOffset, opt.sensorHeight);
    const H  = L.H;
    const Rs = L.R_s;
    const g  = L.g;
    const mm = millimeter;
    const O  = vector(0, 0, 0) * meter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const P  = id + "parts";

    const lw  = f.leg_width;
    const jpT = f.joint_plate;
    const top = f.yaw_arm_top;
    const wellT = x.hornThickness - HORN_OPT.caseOffset;
    const collarR = HORN_OPT.collarOuterDia / 2;
    const backZ = -(x.hornThickness + x.caseDepth);
    const farEnd = x.caseHeight - x.shaftFromEnd;
    // the case shell's own extents, datum-relative (see caseShellGeometry)
    const shellY0  = farEnd - CASE_OPT.caseWrapLength;
    const shellY1  = farEnd + CASE_OPT.caseSideClearance + CASE_OPT.caseTopWall
                     + CASE_OPT.caseNestClearance + CASE_OPT.caseBottomWall;
    const botOuter = x.caseWidth / 2 + CASE_OPT.caseSideClearance + CASE_OPT.caseTopWall
                     + CASE_OPT.caseNestClearance + CASE_OPT.caseBottomWall;
    const capOut   = backZ - CASE_OPT.caseFaceClearance - CASE_OPT.caseCapThickness;
    const nestTop  = backZ + CASE_OPT.caseGripLength + CASE_OPT.caseNestLength;
    const idlerLow = backZ - IDLER_OPT.idlerFaceGap - f.idler_arm_thickness;

    // ---- the servos: yaw far end +X, roll far end DOWN
    const yawCs  = coordSystem(O, -Y, Z);
    const rollCs = coordSystem(vector(-g, 0 * mm, H), -Y, X);
    const xJ2    = -g + nestTop;              // roll base's front face = yaw arm's end
    const xJ3    = -g + f.horn_mount_thickness - jpT;
    const zJ1    = top - jpT;
    const iOpt   = mergeMaps(IDLER_OPT, { "idlerCollarDia" : lw,
                                          "idlerCollarThickness" : f.idler_arm_thickness });

    // ---- yaw arm: over the horn, out to -X past the idler leg, to the roll base
    const armDisc = cylIn(context, P + "yawArmDisc", yawCs, vector(0 * mm, 0 * mm, -wellT),
                          vector(0 * mm, 0 * mm, top), collarR);
    const armBar  = boxW(context, P + "yawArmBar", vector(xJ2, -f.yaw_arm_width / 2, zJ1),
                         vector(0 * mm, f.yaw_arm_width / 2, top));
    const armBoss = boxW(context, P + "yawArmBoss",
                         vector(xJ2, -f.yaw_arm_width / 2, -f.yaw_boss_bottom),
                         vector(-(Rs + lw / 2 + 1 * mm), f.yaw_arm_width / 2, top));
    unite(context, P + "yawArmU", [armDisc, armBar, armBoss]);
    const yawArmPt = vector(-12 * mm, 6 * mm, top - 1 * mm);
    servoMountBuild(context, P + "yawHorn", yawCs, HORN_OPT, partAt(context, P, yawArmPt));

    // ---- yaw idler: plug, arm to -X under the back face, leg up to the arm
    const iArm = boxW(context, P + "yawIdlerArm", vector(-(Rs + lw / 2), -lw / 2, idlerLow),
                      vector(0 * mm, lw / 2, backZ - 0.5 * mm));
    const iLeg = boxW(context, P + "yawIdlerLeg", vector(-(Rs + lw / 2), -lw / 2, idlerLow),
                      vector(-(Rs - lw / 2), lw / 2, zJ1));
    unite(context, P + "yawIdlerU", [iArm, iLeg]);
    const yawIdlerPt = vector(-Rs, 0 * mm, -15 * mm);   // in the leg, below the screw
    idlerBuild(context, P + "yawIdler", yawCs, iOpt, partAt(context, P, yawIdlerPt));

    // ---- yaw base: shell round the far end, on a plate whose clamp tab runs
    // out to +X past the far end. The plate is the one that first ran to -X
    // past the roll servo, turned half a turn about the yaw axis: it now
    // spans -shellY1 .. the old tab's reach mirrored.
    const zTable = idlerLow - f.base_gap - f.base_plate;
    const xTab   = g - capOut + f.tab_beyond;
    const bPlate = boxW(context, P + "basePlate", vector(-shellY1, -f.base_width / 2, zTable),
                        vector(xTab, f.base_width / 2, zTable + f.base_plate));
    const spacer = boxW(context, P + "baseSpacer", vector(shellY0, -botOuter, zTable + f.base_plate / 2),
                        vector(shellY1, botOuter, capOut + 0.5 * mm));
    unite(context, P + "baseU", [bPlate, spacer]);
    const yawBasePt = vector(xTab - 5 * mm, 0 * mm, zTable + f.base_plate / 2);
    caseShellBuild(context, P + "yawBase", yawCs, mergeMaps(CASE_OPT, { "part" : "BOTTOM" }),
                   partAt(context, P, yawBasePt));
    // four 10-32 clearance holes, centred on the tab beyond the shell
    const xBolt = (shellY1 + xTab) / 2;
    var bolts = [];
    for (var i = 0; i < 4; i += 1)
    {
        const bx = xBolt + ((i % 2 == 0) ? 1 : -1) * f.bolt_span_x / 2;
        const by = ((i < 2) ? 1 : -1) * f.bolt_span_y / 2;
        bolts = append(bolts, cylW(context, P + ("bolt" ~ i), vector(bx, by, zTable - 1 * mm),
                                   vector(bx, by, zTable + f.base_plate + 1 * mm), f.bolt_hole_dia / 2));
    }
    opBoolean(context, P + "boltHoles", { "tools" : qUnion(bolts),
            "targets" : partAt(context, P, yawBasePt),
            "operationType" : BooleanOperationType.SUBTRACTION });
    caseShellBuild(context, P + "yawCover", yawCs, mergeMaps(CASE_OPT, { "part" : "TOP", "fullWrap" : true }),
                   qNothing());
    const yawCoverPt = vector(20 * mm, 0 * mm, -1 * mm);

    // ---- roll base: shell round the far end (down), leg down to the yaw arm
    const rLeg = boxW(context, P + "rollBaseLeg", vector(-g + capOut, -botOuter, -f.yaw_boss_bottom),
                      vector(xJ2, botOuter, H - shellY0));
    const rollBasePt = vector(-g + capOut + 5 * mm, 10 * mm, 10 * mm);
    caseShellBuild(context, P + "rollBase", rollCs, mergeMaps(CASE_OPT, { "part" : "BOTTOM" }),
                   partAt(context, P, rollBasePt));
    // a cable clip on each side of the leg: a stem out from the side, a lip
    // back up along it -- two rectangles in (Y, Z), run straight up from the
    // bed (X+) so there is nothing to overhang. Channel open at the top.
    const clipX0 = -g + capOut;
    const clipZ  = (H - shellY0 - f.yaw_boss_bottom) / 2 - f.cable_clip_height / 2;
    var clips = [partAt(context, P, rollBasePt)];
    for (var sgn in [1, -1])
    {
        const tag = sgn > 0 ? "P" : "N";
        const y0c = sgn * (botOuter - 0.5 * mm);                       // into the leg
        const yCh = sgn * (botOuter + f.cable_clip_channel);
        const yOut = sgn * (botOuter + f.cable_clip_channel + f.cable_clip_wall);
        clips = append(clips, boxW(context, P + ("clipStem" ~ tag),
                vector(clipX0, min(y0c, yOut), clipZ),
                vector(clipX0 + f.cable_clip_length, max(y0c, yOut), clipZ + f.cable_clip_wall)));
        clips = append(clips, boxW(context, P + ("clipLip" ~ tag),
                vector(clipX0, min(yCh, yOut), clipZ),
                vector(clipX0 + f.cable_clip_length, max(yCh, yOut), clipZ + f.cable_clip_height)));
    }
    opBoolean(context, P + "clips", { "tools" : qUnion(clips),
            "operationType" : BooleanOperationType.UNION });
    caseShellBuild(context, P + "rollCover", rollCs, mergeMaps(CASE_OPT, { "part" : "TOP", "fullWrap" : true }),
                   qNothing());
    const rollCoverPt = vector(-g - 1 * mm, 0 * mm, H - 20 * mm);

    // ---- roll horn mount: disc over the horn, arm up to the idler leg
    const hDisc = cylIn(context, P + "hornDisc", rollCs, vector(0 * mm, 0 * mm, -wellT),
                        vector(0 * mm, 0 * mm, f.horn_mount_thickness), collarR);
    const hArm  = boxW(context, P + "hornArm", vector(xJ3, -lw / 2, H),
                       vector(-g + f.horn_mount_thickness, lw / 2, H + Rs + lw / 2));
    unite(context, P + "hornU", [hDisc, hArm]);
    const hornPt = vector(xJ3 + 2 * mm, 3 * mm, H + 12 * mm);
    servoMountBuild(context, P + "rollHorn", rollCs, HORN_OPT, partAt(context, P, hornPt));

    // ---- roll idler: plug, arm up behind the back face, leg forward over it
    const xBack = -g + backZ;
    const rArm = boxW(context, P + "rollIdlerArm", vector(xBack - f.idler_arm_thickness, -lw / 2, H),
                      vector(xBack - 0.5 * mm, lw / 2, H + Rs + lw / 2));
    const rLegI = boxW(context, P + "rollIdlerLeg",
                       vector(xBack - f.idler_arm_thickness, -lw / 2, H + Rs - lw / 2),
                       vector(xJ3, lw / 2, H + Rs + lw / 2));
    unite(context, P + "rollIdlerU", [rArm, rLegI]);
    const rollIdlerPt = vector((xBack + xJ3) / 2, 0 * mm, H + Rs);
    idlerBuild(context, P + "rollIdler", rollCs, iOpt, partAt(context, P, rollIdlerPt));

    // ---- TM151 mount at its NATIVE pose: plate under the TM151, wall to the
    // horn mount; the TM151 under the roll axis (on it, for ZERO)
    const dd   = opt.config == "ZERO" ? 0 * mm : opt.ahrsOffset;
    const s    = opt.sensorHeight;
    const hw   = plateHalfWidth();
    const xW   = -g + f.horn_mount_thickness;
    const edge = t.housing_offset_x + t.board_length / 2;
    const zPl  = H - dd - s;
    const mPlate = boxW(context, P + "mPlate", vector(xW, -hw, zPl - f.arm_plate),
                        vector(edge, hw, zPl));
    const mWall  = boxW(context, P + "mWall", vector(xW, -hw, zPl - f.arm_plate),
                        vector(xW + jpT, hw, H + f.mount_wall_top));
    unite(context, P + "mU", [mPlate, mWall]);
    const mountPt = vector(10 * mm, 10 * mm, zPl - f.arm_plate / 2);
    // locating pins in the board's four holes, with the case pins' root relief
    const pinR  = (t.hole_dia - f.tm151_pin_clearance) / 2;
    const pinL  = f.tm151_pin_length;
    const tipCh = 0 * mm;
    const relD  = CASE_OPT.casePinReliefDepth;
    const relW  = CASE_OPT.casePinReliefDia / 2 - pinR;
    const relCh = CASE_OPT.casePinRootChamfer;
    var pins = [];
    var reliefs = [];
    for (var i = 0; i < 4; i += 1)
    {
        // sensor (hx, hy) -> world (-hx, -hy): sensor x is world -X
        const hx = -t.housing_offset_x + t.hole_offset_x + ((i % 2 == 0) ? 1 : -1) * t.hole_span_x / 2;
        const hy = ((i < 2) ? 1 : -1) * t.hole_span_y / 2;
        const c  = vector(-hx, -hy, zPl);
        // the profiles' v runs "out of the servo"; here that is DOWN into the
        // plate, so the pins stand up +Z as they do off the horn feature
        revolveProfile(context, P + ("pin" ~ i), "p", plane(c, Y, X), line(c, Z), %PIN_POLY%);
        revolveProfile(context, P + ("rel" ~ i), "r", plane(c, Y, X), line(c, Z), %RELIEF_POLY%);
        pins = append(pins, qCreatedBy(P + ("pin" ~ i) + "pRev", EntityType.BODY));
        reliefs = append(reliefs, qCreatedBy(P + ("rel" ~ i) + "rRev", EntityType.BODY));
    }
    opBoolean(context, P + "mRelief", { "tools" : qUnion(reliefs),
            "targets" : partAt(context, P, mountPt),
            "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, P + "mPins", { "tools" : qUnion(append(pins, partAt(context, P, mountPt))),
            "operationType" : BooleanOperationType.UNION });

    // ---- the four screw joints
    const yawArm   = partAt(context, P, yawArmPt);
    const yawIdler = partAt(context, P, yawIdlerPt);
    const rollBase = partAt(context, P, rollBasePt);
    const horn     = partAt(context, P, hornPt);
    const rIdler   = partAt(context, P, rollIdlerPt);
    const mount    = partAt(context, P, mountPt);
    // J1 yaw arm -> yaw idler leg, vertical. The nut slot runs out the leg's
    // SIDE (+Y): out along -X it cut into the yaw arm's end boss, which sits
    // 1 mm outboard of the leg at the nut's height, and would have blocked
    // the nut going in (the print check found it, 2026-09-23).
    screwJoint(context, P + "j1", vector(-Rs, 0 * mm, top), -Z, Y, 0 * mm,
               yawArm, yawIdler, true, Y, lw / 2, -Z, Z);
    // J2 roll base -> yaw arm end, along +X, the head at the bottom of a pocket
    const zS2 = (top - f.yaw_boss_bottom) / 2;
    screwJoint(context, P + "j2", vector(xJ2 - jpT, 0 * mm, zS2), X, Z,
               (xJ2 - jpT) - (-g + capOut), rollBase, yawArm, true, Y,
               f.yaw_arm_width / 2 - 1 * mm, X, -Z);
    // J3 roll horn mount -> roll idler leg, along -X, nut slot out along +Z
    screwJoint(context, P + "j3", vector(xW, 0 * mm, H + Rs), -X, Z,
               0 * mm, horn, rIdler, true, Y, lw / 2, -X, X);
    // J4 TM151 mount -> horn mount, ON the roll axis, so turning the mount
    // over leaves the screw where it was; the ridge runs along Z through the
    // axis for the same reason. Ridge on the mount (on its side as printed),
    // groove in the horn mount's bed face. The nut slot runs along Y, across
    // the horn mount, clear of its arm to the idler leg.
    screwJoint(context, P + "j4", vector(xW + jpT, 0 * mm, H), -X, Y, 0 * mm,
               mount, horn, false, Z, f.mount_wall_top - 1 * mm, Z, -X);

    // ---- hardware
    const HW = id + "hw";
    const yawS  = x330Envelope(context, HW + "yaw", yawCs);
    const rollS = x330Envelope(context, HW + "roll", rollCs);
    const tmCs  = coordSystem(vector(0 * mm, 0 * mm, H - dd), -X, Z);
    const tm    = tm151Envelope(context, HW + "tm", tmCs, s);

    // ABOVE: the same mount, turned half a turn about the roll axis
    const rollAxis = line(vector(0 * mm, 0 * mm, H), X);
    var mountFinalPt = mountPt;
    if (opt.config == "ABOVE")
    {
        opTransform(context, id + "flip", {
                "bodies" : qUnion([mount, tm.body, tm.keepout]),
                "transform" : rotationAround(rollAxis, 180 * degree) });
        mountFinalPt = vector(10 * mm, -10 * mm, 2 * H - (zPl - f.arm_plate / 2));
    }

    // ---- names, colours, print orientation
    const baseC = color(0.62, 0.64, 0.68);
    const yawC  = color(0.93, 0.56, 0.20);
    const rollC = color(0.30, 0.68, 0.42);
    const mountName = "TM151 mount " ~ (opt.config == "ZERO" ? "0"
                      : toString(roundToPrecision(opt.ahrsOffset / mm, 0))) ~ " mm";
    // [name, point inside it, colour, print-up label, print-up vector, stage]
    const parts = [
        ["yaw base", yawBasePt, baseC, "Z+", Z, "static"],
        ["yaw cover", yawCoverPt, baseC, "Z-", -Z, "static"],
        ["yaw idler", yawIdlerPt, yawC, "Z+", Z, "yaw"],
        ["yaw arm", yawArmPt, yawC, "Z-", -Z, "yaw"],
        ["roll base", rollBasePt, yawC, "X+", X, "yaw"],
        ["roll cover", rollCoverPt, yawC, "X-", -X, "yaw"],
        ["roll idler", rollIdlerPt, rollC, "X+", X, "roll"],
        ["roll horn mount", hornPt, rollC, "X-", -X, "roll"],
        [mountName, mountFinalPt, rollC, "Z+", opt.config == "ABOVE" ? -Z : Z, "roll"]];
    var stages = { "static" : [yawS.caseQ], "yaw" : [yawS.horn, rollS.caseQ],
                   "roll" : [rollS.horn, tm.body] };
    var prints = [];
    for (var n in parts)
    {
        const q = partAt(context, P, n[1]);
        dress(context, q, n[0] ~ " [print " ~ n[3] ~ "]", n[2],
              "Print with the " ~ n[3] ~ " side facing UP (native pose).");
        stages[n[5]] = append(stages[n[5]], q);
        prints = append(prints, [n[0], q, n[4]]);
    }
    const dark = color(0.16, 0.16, 0.18);
    dress(context, yawS.caseQ, "yaw X330 case", dark, "");
    dress(context, yawS.horn, "yaw X330 horn", color(0.3, 0.3, 0.32), "");
    dress(context, rollS.caseQ, "roll X330 case", dark, "");
    dress(context, rollS.horn, "roll X330 horn", color(0.3, 0.3, 0.32), "");
    dress(context, tm.body, "TM151", color(0.20, 0.45, 0.85), "");
    if (opt.drawKeepout != false)
    {
        dress(context, tm.keepout, "USB keep-out", color(0.85, 0.2, 0.75, 0.25), "");
        stages["roll"] = append(stages["roll"], tm.keepout);
    }
    else
        opDeleteBodies(context, id + "noKO", { "entities" : tm.keepout });

    // the horn datums, for hand-drawn features later
    opMateConnector(context, id + "yawMC", { "coordSystem" : yawCs, "owner" : yawS.caseQ });
    opMateConnector(context, id + "rollMC", { "coordSystem" : rollCs, "owner" : rollS.caseQ });

    if (opt.drawGhosts == true)
    {
        for (var sgn in [1, -1])
        {
            const tag = sgn > 0 ? "gP" : "gN";
            opPattern(context, id + tag, { "entities" : tm.body,
                    "transforms" : [rotationAround(rollAxis, sgn * f.travel_deg)],
                    "instanceNames" : ["g"] });
            dress(context, qCreatedBy(id + tag, EntityType.BODY),
                  "TM151 ghost " ~ (sgn > 0 ? "+" : "-")
                  ~ toString(roundToPrecision(f.travel_deg / degree, 0)),
                  color(0.20, 0.45, 0.85, 0.25), "");
        }
    }
    return { "L" : L, "stages" : stages, "prints" : prints, "rollAxis" : rollAxis };
}

%SPLIT%

export enum FixtureConfig
{
    annotation { "Name" : "d mount, TM151 BELOW the roll axis (as printed)" }
    BELOW,
    annotation { "Name" : "0 mount, sensing point on the roll axis" }
    ZERO,
    annotation { "Name" : "d mount turned over, TM151 ABOVE the roll axis" }
    ABOVE
}

annotation { "Feature Type Name" : "AHRS fixture",
             "Feature Type Description" : "The yaw-roll TM151 fixture: printed parts, servo and TM151 envelopes" }
export const ahrsFixture = defineFeature(function(context is Context, id is Id,
                                                  definition is map)
    precondition
    {
        annotation { "Name" : "AHRS offset from the roll axis (d)" }
        isLength(definition.ahrsOffset, { (millimeter) : [0, %OFFSET%, 200] } as LengthBoundSpec);

        annotation { "Name" : "Which TM151 mount is fitted" }
        definition.config is FixtureConfig;

        annotation { "Name" : "Draw the TM151 at the ends of travel", "Default" : true }
        definition.drawGhosts is boolean;

        annotation { "Name" : "Draw the USB plug keep-out", "Default" : true }
        definition.drawKeepout is boolean;

        annotation { "Group Name" : "Assumptions", "Collapsed By Default" : true }
        {
            // A GUESS: the datasheet does not locate the sensing element.
            // The fixture's analysis fits the real one; this only sets where
            // "0 mm" puts the housing.
            annotation { "Name" : "Sensing point above the board's bottom face" }
            isLength(definition.sensorHeight, { (millimeter) : [0, %SENSOR%, 12.6] } as LengthBoundSpec);
        }
    }
    {
        const r = ahrsFixtureBuild(context, id + "build", {
                "ahrsOffset" : definition.ahrsOffset,
                "config" : definition.config == FixtureConfig.ZERO ? "ZERO"
                           : (definition.config == FixtureConfig.ABOVE ? "ABOVE" : "BELOW"),
                "sensorHeight" : definition.sensorHeight,
                "drawGhosts" : definition.drawGhosts,
                "drawKeepout" : definition.drawKeepout });
        setVariable(context, "ahrsRollAxisHeight", r.L.H);
        setVariable(context, "ahrsOffset", definition.ahrsOffset);
        reportFeatureInfo(context, id, "Roll axis " ~ toString(roundToPrecision(r.L.H / millimeter, 2))
                ~ " mm above the yaw horn face; roll horn " ~ toString(roundToPrecision(r.L.g / millimeter, 2))
                ~ " mm from the yaw axis");
    });
'''


def build_fs(data: dict, fs_version: str = "3044") -> str:
    tm, fx = data["tm151"], data["fixture"]
    t = data["table"]["XC330"]
    pick = lambda keys: {"servo": "XC330", **{k: t[k] * 1000 for k in keys}}  # noqa: E731
    subs = {
        "%VERSION%": fs_version,
        "%TM151%": _fs_map("TM151", tm),
        "%X330%": _fs_map("X330", data["x330"]),
        "%FIXTURE%": _fs_map("FIXTURE", fx),
        "%HORN_OPT%": _fs_map("HORN_OPT", pick(sm.DIALOG)),
        "%CASE_OPT%": _fs_map("CASE_OPT", pick(sm.CASE_DIALOG)),
        "%IDLER_OPT%": _fs_map("IDLER_OPT", pick(sm.IDLER_DIALOG)),
        "%SCREW_OPT%": _fs_map("SCREW_OPT", data["screw"]),
        "%SERVO_MOUNT%": _servo_mount_layer(data, fs_version),
        # the TM151 pins reuse the horn/case pin profiles; the FeatureScript
        # binds pinR, pinL, tipCh, relD, relW and relCh before reading them
        "%PIN_POLY%": sm._fs_poly(sm.pin_profile()),
        "%RELIEF_POLY%": sm._fs_poly(sm.relief_profile()),
        "%SPLIT%": SPLIT_MARK,
        "%OFFSET%": f"{fx['ahrs_offset']:g}",
        "%SENSOR%": f"{tm['sensor_height']:g}",
    }
    text = FS
    for k, v in subs.items():
        text = text.replace(k, v)
    return text


def check_wrapper(fs: str, data: dict, config: str) -> str:
    """Build every part, then print: the layout, every body, every collision
    at rest and at each pose of each joint, and each part's downward faces
    in its print orientation. All judging happens in `check`, in Python."""
    fx, tm = data["fixture"], data["tm151"]
    opt = (f'{{ "ahrsOffset" : {fx["ahrs_offset"]:.4g} * millimeter, "config" : "{config}", '
           f'"sensorHeight" : {tm["sensor_height"]:.4g} * millimeter, '
           f'"drawGhosts" : false, "drawKeepout" : true }}')
    poses = "[" + ", ".join(str(a) for a in POSES) + "]"
    return f"""function(context is Context, queries)
{{
{sm.geometry_layer(fs, SPLIT_MARK)}
    const id = makeId("chk");
    const r = ahrsFixtureBuild(context, id, {opt});
    println("H=" ~ toString(r.L.H / millimeter));
    println("R_s=" ~ toString(r.L.R_s / millimeter));
    println("g=" ~ toString(r.L.g / millimeter));
    const name = function(q) returns string
    {{
        return getProperty(context, {{ "entity" : q, "propertyType" : PropertyType.NAME }});
    }};
    const all = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    for (var b in all)
        println("BODY|" ~ name(b));
    const clashes = function(pose is string, tools, targets)
    {{
        for (var c in evCollision(context, {{ "tools" : tools, "targets" : targets }}))
            println("COLL|" ~ pose ~ "|" ~ name(c.toolBody) ~ "|" ~ name(c.targetBody)
                    ~ "|" ~ toString(c["type"]));
    }};
    for (var i = 0; i + 1 < size(all); i += 1)
        clashes("rest", all[i], qUnion(subArray(all, i + 1, size(all))));
    // Tag the stages with attributes BEFORE moving anything: the parts'
    // queries find them by a point inside, which a rotated body no longer
    // contains, so the move back would find nothing. Attributes travel.
    for (var sname in ["static", "yaw", "roll"])
        for (var q in r.stages[sname])
            setAttribute(context, {{ "entities" : q, "name" : "stg_" ~ sname, "attribute" : sname }});
    const st   = qHasAttribute("stg_static");
    const yaw  = qHasAttribute("stg_yaw");
    const roll = qHasAttribute("stg_roll");
    const up   = qUnion([yaw, roll]);
    const yawAxis = line(vector(0, 0, 0) * meter, vector(0, 0, 1));
    var k = 0;
    for (var a in {poses})
    {{
        opTransform(context, id + ("ty" ~ k), {{ "bodies" : up,
                "transform" : rotationAround(yawAxis, a * degree) }});
        clashes("yaw " ~ a, up, st);
        opTransform(context, id + ("tyb" ~ k), {{ "bodies" : up,
                "transform" : rotationAround(yawAxis, -a * degree) }});
        opTransform(context, id + ("tr" ~ k), {{ "bodies" : roll,
                "transform" : rotationAround(r.rollAxis, a * degree) }});
        clashes("roll " ~ a, roll, qUnion([st, yaw]));
        opTransform(context, id + ("trb" ~ k), {{ "bodies" : roll,
                "transform" : rotationAround(r.rollAxis, -a * degree) }});
        k += 1;
    }}
    // print check: planar faces facing within 20 deg of straight down and
    // above the bed overhang past 70 deg -- bridges, or faults
    for (var pr in r.prints)
    {{
        const bodies = evaluateQuery(context, pr[1]);
        println("PART|" ~ pr[0] ~ "|" ~ toString(size(bodies)));
        if (size(bodies) != 1)
            continue;
        const upv = pr[2];
        const cs = coordSystem(vector(0, 0, 0) * meter, perpendicularVector(upv), upv);
        const bed = evBox3d(context, {{ "topology" : bodies[0], "cSys" : cs, "tight" : true }}).minCorner[2];
        var bedArea = 0 * meter * meter;
        for (var fc in evaluateQuery(context, qGeometry(qOwnedByBody(bodies[0], EntityType.FACE),
                                                        GeometryType.PLANE)))
        {{
            const pl = evPlane(context, {{ "face" : fc }});
            // strictly past 70 deg: a face AT the limit (the shells' slopes
            // and V's are cut exactly there) is allowed
            if (-dot(pl.normal, upv) < cos(19.5 * degree))
                continue;
            const fb = evBox3d(context, {{ "topology" : fc, "tight" : true }});
            const h = evBox3d(context, {{ "topology" : fc, "cSys" : cs, "tight" : true }}).minCorner[2] - bed;
            const ar = evArea(context, {{ "entities" : fc }});
            if (h < 0.01 * millimeter)
                bedArea += ar;
            else
                println("OVER|" ~ pr[0] ~ "|" ~ toString(ar / (millimeter * millimeter))
                        ~ "|" ~ toString(h / millimeter) ~ "|"
                        ~ toString(roundToPrecision((fb.minCorner[0] + fb.maxCorner[0]) / 2 / millimeter, 1)) ~ ","
                        ~ toString(roundToPrecision((fb.minCorner[1] + fb.maxCorner[1]) / 2 / millimeter, 1)) ~ ","
                        ~ toString(roundToPrecision((fb.minCorner[2] + fb.maxCorner[2]) / 2 / millimeter, 1)));
        }}
        // horizontal bores whose surface still reaches its crown: a flat
        // roof as printed, wanting a teardrop. Teardropped ones stop at 45.
        for (var fc in evaluateQuery(context, qGeometry(qOwnedByBody(bodies[0], EntityType.FACE),
                                                        GeometryType.CYLINDER)))
        {{
            const cyl = evSurfaceDefinition(context, {{ "face" : fc }});
            if (cyl.radius < 1 * millimeter || abs(dot(cyl.coordSystem.zAxis, upv)) > 0.1)
                continue;
            const axisH = dot(cyl.coordSystem.origin, upv);
            const top = evBox3d(context, {{ "topology" : fc, "cSys" : cs, "tight" : true }}).maxCorner[2];
            // concave (a hole) if the part is not on the far side: test the
            // point just above the crown -- empty for a hole
            const crown = cyl.coordSystem.origin + (cyl.radius + 0.05 * millimeter) * upv;
            const isHole = isQueryEmpty(context, qContainsPoint(bodies[0], crown));
            if (isHole && top - axisH > cyl.radius * cos(45 * degree) + 0.02 * millimeter)
                println("CROWN|" ~ pr[0] ~ "|" ~ toString(roundToPrecision(cyl.radius / millimeter, 2)) ~ "|"
                        ~ toString(roundToPrecision(cyl.coordSystem.origin[0] / millimeter, 1)) ~ ","
                        ~ toString(roundToPrecision(cyl.coordSystem.origin[1] / millimeter, 1)) ~ ","
                        ~ toString(roundToPrecision(cyl.coordSystem.origin[2] / millimeter, 1)));
        }}
        println("BED|" ~ pr[0] ~ "|" ~ toString(bedArea / (millimeter * millimeter)));
    }}
    return "ran to completion";
}}
"""


HARDWARE = ("yaw X330 case", "yaw X330 horn", "roll X330 case", "roll X330 horn",
            "TM151", "USB keep-out", "TM151 ghost")


def canon(name: str) -> str:
    """A body's name without its suffixes: the LONGEST known name it starts
    with, so "TM151 mount 30 mm [print Z+]" is a TM151 mount, not a TM151."""
    hits = [k for k in PARTS + HARDWARE if name == k or name.startswith(k + " ")]
    return max(hits, key=len) if hits else name


def _intended(a: str, b: str) -> bool:
    pair = {canon(a), canon(b)}
    return any(pair == {x, y} for x, y in INTENDED)


def check(text: str, data: dict, target: str | None, config: str) -> tuple[bool, dict]:
    from . import onshape

    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text, data, config), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console[-3000:])
        print(onshape.budget_line())
        return False, {}
    rows = [l.split("|") for l in console.splitlines()]
    got = dict(l.split("=", 1) for l in console.splitlines() if "=" in l and "|" not in l)
    L = layout(data)
    ok = True
    for k in ("H", "R_s", "g"):
        v = float(got.get(k, "nan"))
        bad = not abs(v - L[k]) < 1e-3
        ok &= not bad
        print(f"  {k:4} wanted {L[k]:8.3f}  got {v:8.3f}  {'FAIL' if bad else 'ok'}")
    counts = {r[1]: int(r[2]) for r in rows if r[0] == "PART"}
    for n, c in counts.items():
        ok &= c == 1
        print(f"  part {n:18} {c} body  {'ok' if c == 1 else 'FAIL'}")
    ok &= len(counts) == len(PARTS)
    clash = [r for r in rows if r[0] == "COLL" and "ABUT" not in r[4]
             and not _intended(r[2], r[3])]
    ok &= not clash
    print(f"  interference at rest and over both joints ({', '.join(map(str, POSES))} deg): "
          f"{len(clash)}")
    for r in clash[:40]:
        print(f"    {r[1]:9} {r[2]}  x  {r[3]}  ({r[4]})")
    print("  print check -- planar faces within 20 deg of facing down, above the bed:")
    for r in rows:
        if r[0] == "BED":
            over = sorted((x for x in rows if x[0] == "OVER" and x[1] == r[1]),
                          key=lambda o: -float(o[2]))
            print(f"    {r[1]:18} bed {float(r[2]):7.1f} mm^2, {len(over)} downward faces"
                  + "".join(f"\n        {float(o[2]):7.2f} mm^2, {float(o[3]):6.2f} up, at ({o[4]})"
                            for o in over))
    crowns = [r for r in rows if r[0] == "CROWN"]
    print(f"  horizontal holes with a flat crown as printed (want a teardrop): {len(crowns)}")
    for r in crowns:
        print(f"    {r[1]:18} r {r[2]} mm, axis through ({r[3]})")
    print(onshape.budget_line())
    return ok, {"rows": rows}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fixture", default=FIXTURE_PARAMS)
    ap.add_argument("--params", default=sm.CAD_PARAMS)
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="build it in Onshape and check it; ONE billable call per "
                         "--config, defaults to the `check` tab")
    ap.add_argument("--config", choices=CONFIGS, nargs="+", default=["BELOW"],
                    help="which TM151 mount(s) --check builds")
    ap.add_argument("--push", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="replace a Feature Studio's contents (its own tab only)")
    ap.add_argument("--shot", metavar="TAB|URL", nargs="?", const="", default=None,
                    help=f"render the Part Studio (default `ahrs_fixture`) to {OUT_PNG}; "
                         "ONE billable call")
    ap.add_argument("--view", default="isometric")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the check script instead of spending a call")
    args = ap.parse_args()

    data = load(args.fixture, args.params)
    text = build_fs(data, args.fs_version)
    sm.lint_fs(text)
    Path(args.output).write_text(text)
    L = layout(data)
    print(f"wrote {len(text)} chars -> {args.output}")
    print(f"  d = {L['d']:g}: roll axis {L['H']:.2f} mm above the yaw horn face, "
          f"roll horn {L['g']:.2f} from the yaw axis, U legs at {L['R_s']:.2f}")
    if args.dry_run:
        print(check_wrapper(text, data, args.config[0]))
        return
    if args.check is not None:
        for c in args.config:
            print(f"--- check, config {c}")
            if not check(text, data, args.check or None, c)[0]:
                raise SystemExit("check FAILED -- not pushing")
    if args.push is not None:
        from . import onshape
        if args.push in ("", "feature_studio", "horn_features", "swing_features"):
            raise SystemExit(f"refusing to push at {args.push or 'the default'!r}: "
                             "each generator owns its own studio tab")
        url = onshape.resolve(args.push, args.push)
        onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}")
        print(onshape.budget_line())
    if args.shot is not None:
        from . import onshape
        url = onshape.resolve(args.shot or None, "ahrs_fixture")
        out, _ = onshape.shaded_view(url, Path(OUT_PNG), view=args.view)
        print(f"rendered -> {out}")
        print(onshape.budget_line())


if __name__ == "__main__":
    main()
