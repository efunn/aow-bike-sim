"""The swing linkage as a driveable Onshape sketch feature.

`analysis/swing_linkage.py` answers questions about the co-rotating four-bar
pair; this puts the same geometry into CAD as a CUSTOM FEATURE with a crank
input you can turn. Insert it several times on one plane at different inputs
and the overlay IS the motion study -- no animation, no assembly, no mates.

    python -m aow_sim.cad_swing_linkage                        # default config
    python -m aow_sim.cad_swing_linkage --config config/swing_linkage_flat.yaml
    python -m aow_sim.cad_swing_linkage --check                # 1 billable call
    python -m aow_sim.cad_swing_linkage --push swing_features  # 2 calls

THE FOUR-BAR IS SOLVED IN FEATURESCRIPT, NOT HERE. That is the whole point of
the feature and it is also the cost: `swingSolve` below is a second
implementation of `SwingLinkage.solve`, in a language with no numpy, and two
implementations of one circle-circle branch rule are two chances to pick the
folded-through assembly. `--check` exists for exactly that -- it runs the
generated script on Onshape's servers and compares every joint, foot and top
against this repo's solver at seven crank angles, in ONE call.

WHAT COMES FROM THE CONFIG AND WHAT COMES FROM THE DIALOG. The config supplies
the DEFAULTS, resolved through `SwingLinkage` rather than read raw: two of the
numbers a swing config carries are derived at load, and reading them off the
yaml gives a different mechanism from the one the study plotted.

    wing_angle_from_rocker   derived under wing_angle_mode vertical_rest and
                             flat_deploy. `swing_linkage_smaller.yaml` is
                             flat_deploy, whose solve lives in the study.
    wing_z_min               ALWAYS derived, so the panel's lower edge sits at
                             `bike.ground_clearance` -- and, unlike wing_z_max,
                             never written back to the file. The -20.0 in every
                             config is the as-drawn value and is ignored.

After that the dialog owns them. Every mechanism number is an editable field,
so a variant can be tried in the browser without regenerating anything; come
back here when the answer is worth keeping in a config.

COORDINATES. The study is a front view in (y, z) millimetres from the
CENTRELINE and the AXLE -- its `wing_z_min` and friends are measured along the
panel, not upward. The sketch is laid out with u = -y and v = z, because CAD x
is -y everywhere else in this repo (`cad_layout.to_cad_pos`), so on the world
front plane this lands exactly where `aowFourBarSketch` puts the steering
linkage. The floor is at v = -wheel_radius, which is what `Draw the floor line`
draws and what the chassis keep-out heights are measured from.

WHERE IT PUSHES. Nowhere by default, and never to `feature_studio` (cad_layout
overwrites that tab wholesale) or `horn_features` (cad_servo_mount owns it). A
push needs its own Feature Studio tab: make one in the browser, which is free,
add its id to config/onshape.yaml under a name, and pass the name.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from string import Template

import yaml

# The third module in this package to want these two, and they are already
# duplicated between cad_layout and cad_servo_mount. Imported rather than
# copied a third time.
from .cad_servo_mount import _block_end, _strip_comments

ROOT = Path(__file__).resolve().parents[2]
# `SwingLinkage` lives in analysis/, which is not a package. Same sys.path
# trick analysis/linkage_through_belt.py uses to reach analysis.wing_linkage,
# and preferable to a third copy of the derivations: build_model already
# carries a partial one and its own comment says flat_deploy is beyond it.
sys.path.insert(0, str(ROOT / "analysis"))
from swing_linkage import SwingLinkage          # noqa: E402

CONFIG = "config/swing_linkage_smaller.yaml"
OUT_FS = "docs/measurements/swing_linkage.fs"
SPLIT_MARK = "// ==== UI LAYER BELOW -- dropped by --check ===="

# Tabs another generator owns. A push overwrites a studio's whole contents, so
# landing on one of these deletes it.
OWNED = {"feature_studio": "cad_layout", "horn_features": "cad_servo_mount"}

# (FeatureScript field, SwingLinkage attribute, label, lo, hi). Values come off
# the OBJECT, never off the yaml -- see the module docstring. `lo`/`hi` are the
# dialog's bounds in mm or degrees; they are room to experiment, not physics.
LENGTHS = [
    ("servoOffset",     "shaft[1]",  "Servo shaft above the axle",      0.0, 300.0),
    ("wingPivotX",      "pivot_x",   "Wing pivot out from centreline",  0.0, 300.0),
    ("wingPivotZ",      "pivot_z",   "Wing pivot above the axle",    -300.0, 300.0),
    ("crankLength",     "crank",     "Crank length",                    1.0, 300.0),
    ("couplerLength",   "coupler",   "Coupler length",                  1.0, 400.0),
    ("rockerLength",    "rocker",    "Rocker length",                   1.0, 300.0),
    ("wingNormOffset",  "wing_norm", "Panel standoff from the rocker",  0.0, 100.0),
    ("wingZMin",        "wing_z_min", "Panel foot along its own axis", -300.0, 300.0),
    ("wingZMax",        "wing_z_max", "Panel top along its own axis",  -300.0, 300.0),
    ("wheelRadius",     "wheel_radius", "Wheel radius (sets the floor)", 0.0, 300.0),
]
ANGLES = [
    ("angleBetweenCranks",  "between",         "Angle between the crank arms", 0.0, 180.0),
    ("wingAngleFromRocker", "wing_from_rocker", "Panel angle off the rocker", -180.0, 180.0),
]

# Crank inputs --check compares against, as a fraction of the config's stroke.
# Both signs, because the mechanism is only symmetric by CONSTRUCTION -- a port
# that mirrors the crank arms reproduces rest exactly and nothing else.
CHECK_FRACTIONS = (-0.9, -0.5, -0.25, 0.0, 0.25, 0.5, 0.9)


def resolved(cfg: dict) -> tuple[SwingLinkage, dict]:
    """The linkage, and every dialog default as a plain number."""
    lk = SwingLinkage(cfg)
    # `shaft` is the only one that is a point rather than a scalar attribute.
    getters = {"shaft[1]": lambda: float(lk.shaft[1])}
    vals = {fs: getters[attr]() if attr in getters else float(getattr(lk, attr))
            for fs, attr, *_ in LENGTHS + ANGLES}
    return lk, vals


def keepout_fs(cfg: dict) -> str:
    """The panel keep-out boxes as a FeatureScript constant.

    The RAW configured boxes, not `_keepouts`' inflated ones. Those grow by
    half the panel width so a front-view line test stands in for a solid; drawn
    on top of the mechanism that would be a chassis nobody built.
    """
    boxes = (cfg.get("clearance") or {}).get("panel_keepout") or []
    rows = []
    for i, k in enumerate(boxes):
        hi = k.get("z_hi")
        rows.append(
            '    { "name" : "%s", "halfWidth" : %g, "zLo" : %s, "zHi" : %s }'
            % (k.get("name", f"box{i}"), float(k["half_width"]),
               "undefined" if k.get("z_lo") is None else f"{float(k['z_lo']):g}",
               "undefined" if hi is None else f"{float(hi):g}"))
    return "[\n" + ",\n".join(rows) + "\n]" if rows else "[]"


# --------------------------------------------------------------------------
# the emitted FeatureScript
#
# string.Template, NOT an f-string, and that is the point. FeatureScript is
# almost entirely braces; an f-string has to double every one of them, and on
# 2026-08-25 a quadrupled pair put a literal `{{` into the horn-pin studio and
# turned every feature in it red while --check still reported green (see
# cad_servo_mount.lint_fs). With `$name` substitution the mistake is not
# representable -- the braces below are exactly the braces that get emitted.

FS = Template(r'''FeatureScript $ver;
import(path : "onshape/std/geometry.fs", version : "$ver.0");

/* GENERATED, do not hand-edit: the next push replaces this studio
   wholesale. Regenerate with
       python -m aow_sim.cad_swing_linkage --config $config

   The co-rotating swing linkage, drawn as a sketch at a crank input you pick.
   Insert the feature once per pose you want to see -- 0, a third of the
   stroke, two thirds, the end -- all on the same plane. The overlay is the
   motion study; nothing here is an assembly and nothing mates.

   Stroke from the study: $stroke deg of crank travel. Positive input deploys
   the RIGHT (-y) wing and lifts the left; negative swaps them. Past the
   assembly limit the four-bar cannot close and the feature errors rather than
   drawing a mechanism that has come apart.

   Sketch coordinates are u = -y, v = z, both in mm from the centreline and the
   REAR AXLE, which is where aowFourBarSketch draws too. The floor is at
   v = -wheelRadius.
*/

export const SWING_KEEPOUT = $keepout;

// -------------------------------------------------------------------------
// the four-bar, ported from analysis/swing_linkage.py
//
// Lengths are plain numbers in MILLIMETRES throughout this layer and only
// become ValueWithUnits at the sketch. Angles are the other way round: they
// carry units, because cos/sin demand it and a bare radian passed as degrees
// is a mechanism that looks plausible and is wrong.

export function swingShaft(g is map) returns Vector
{
    return vector(0, g.servoOffset);
}

export function swingPivot(g is map, side is number) returns Vector
{
    return vector(side * g.wingPivotX, g.wingPivotZ);
}

export function swingCrankTip(g is map, side is number, travel is ValueWithUnits) returns Vector
{
    // Both arms are on ONE body `angleBetweenCranks` apart, so a single input
    // moves both: the right arm (side -1) sits at +between/2 from vertical and
    // the left at -between/2, which is what makes rest symmetric.
    const a = (90 - side * g.angleBetweenCranks / 2) * degree + travel;
    return swingShaft(g) + g.crankLength * vector(cos(a), sin(a));
}

export function swingBranchSign(J is Vector, C is Vector, P is Vector) returns number
{
    // Which way the coupler-rocker elbow bends. THE assembly invariant: a
    // four-bar cannot change it without coming apart, so it picks the
    // reachable solution out of the circle-circle pair -- and it still decides
    // at a toggle, where the crank-coupler elbow has gone straight.
    const v = J - P;
    const w = C - J;
    const cross = v[0] * w[1] - v[1] * w[0];
    if (cross > 0)
        return 1;
    if (cross < 0)
        return -1;
    return 0;
}

export function swingCircleCircle(p is Vector, r1 is number, q is Vector, r2 is number) returns array
{
    const d = q - p;
    const L = norm(d);
    if (L < 1e-9 || L > r1 + r2 || L < abs(r1 - r2))
        return [];
    const a = (r1 * r1 - r2 * r2 + L * L) / (2 * L);
    const h2 = r1 * r1 - a * a;
    if (h2 < 0)
        return [];
    const h = sqrt(h2);
    const base = p + a * d / L;
    const perp = vector(-d[1], d[0]) / L;
    return [base + h * perp, base - h * perp];
}

export function swingRestJoint(g is map, side is number) returns Vector
{
    const P = swingPivot(g, side);
    const C = swingCrankTip(g, side, 0 * degree);
    const sols = swingCircleCircle(P, g.rockerLength, C, g.couplerLength);
    if (size(sols) == 0)
        throw regenError("The swing linkage cannot assemble at rest: the crank, coupler and rocker lengths do not close.");
    // OUTBOARD branch -- the one further from the centreline. The inboard one
    // folds the rocker through the chassis, and a four-bar assembled that way
    // cannot be driven onto the other.
    return abs(sols[0][0]) >= abs(sols[1][0]) ? sols[0] : sols[1];
}

export function swingSolve(g is map, side is number, travel is ValueWithUnits)
{
    const P = swingPivot(g, side);
    const C = swingCrankTip(g, side, travel);
    const sols = swingCircleCircle(P, g.rockerLength, C, g.couplerLength);
    if (size(sols) == 0)
        return undefined;                  // a real answer: the assembly limit
    const rest = swingRestJoint(g, side);
    const want = swingBranchSign(rest, swingCrankTip(g, side, 0 * degree), P);
    var ok = [];
    for (var q in sols)
    {
        if (swingBranchSign(q, C, P) == want)
            ok = append(ok, q);
    }
    if (size(ok) == 1)
        return ok[0];
    // Degenerate: at or through a singularity the sign cannot separate them.
    // Fall back to the nearest to rest, which is what the study does after
    // reset() -- and every instance of this feature IS a fresh reset.
    return norm(sols[0] - rest) <= norm(sols[1] - rest) ? sols[0] : sols[1];
}

export function swingPanel(g is map, side is number, joint is Vector) returns map
{
    // The panel is rigid with the rocker: it runs at wingAngleFromRocker off
    // the rocker's bearing, stood off wingNormOffset normal to it, and spans
    // wingZMin..wingZMax ALONG ITS OWN AXIS. The foot is the wingZMin end.
    const P = swingPivot(g, side);
    const r = joint - P;
    const rdir = r / norm(r);
    const wa = atan2(rdir[1], rdir[0]) + side * g.wingAngleFromRocker * degree;
    const w = vector(cos(wa), sin(wa));
    const n = vector(w[1], -w[0]);
    const origin = joint + side * g.wingNormOffset * n;
    return { "origin" : origin, "foot" : origin + g.wingZMin * w,
             "top" : origin + g.wingZMax * w };
}

export function swingPose(g is map, side is number, travel is ValueWithUnits) returns map
{
    const joint = swingSolve(g, side, travel);
    if (joint == undefined)
        return { "closes" : false };
    const pan = swingPanel(g, side, joint);
    return { "closes" : true, "joint" : joint, "pivot" : swingPivot(g, side),
             "tip" : swingCrankTip(g, side, travel), "origin" : pan.origin,
             "foot" : pan.foot, "top" : pan.top };
}

export function swingTo2d(g is map, p is Vector)
{
    // u = -y. The bike's LEFT is +y in the study and CAD -x everywhere in this
    // repo, so the drawing lands on the world front plane the same way round
    // as every other generated view.
    return vector(-p[0], p[1] + g.zOffset) * millimeter;
}

export function swingParams(definition is map) returns map
{
    return {
        "servoOffset" : definition.servoOffset / millimeter,
        "wingPivotX" : definition.wingPivotX / millimeter,
        "wingPivotZ" : definition.wingPivotZ / millimeter,
        "crankLength" : definition.crankLength / millimeter,
        "couplerLength" : definition.couplerLength / millimeter,
        "rockerLength" : definition.rockerLength / millimeter,
        "wingNormOffset" : definition.wingNormOffset / millimeter,
        "wingZMin" : definition.wingZMin / millimeter,
        "wingZMax" : definition.wingZMax / millimeter,
        "wheelRadius" : definition.wheelRadius / millimeter,
        "angleBetweenCranks" : definition.angleBetweenCranks / degree,
        "wingAngleFromRocker" : definition.wingAngleFromRocker / degree,
        "zOffset" : definition.zOffset / millimeter
    };
}

export function swingSketchPlane(context is Context, sketchPlane is Query) returns Plane
{
    // A PLANE, never the query itself. `newSketch(context, id, { "sketchPlane"
    // : <query> })` on a datum plane built NOTHING -- no edges, no vertices
    // and no error -- where the same segments through `newSketchOnPlane` drew
    // every one. Measured 2026-09-03 on the check tab: 0 entities against 4
    // edges from 4 segments. Resolving the plane here fails LOUDLY instead,
    // which is the only reason the next paragraph is known.
    //
    // QueryFilterCompound.ALLOWS_PLANE lets the user pick three different
    // things and all three want different handling. A mate connector is not a
    // plane at all. A datum plane is a BODY, and evPlane wants the FACE that
    // body owns -- handing it the body is CANNOT_RESOLVE_PLANE. Only a planar
    // face of a real part arrives ready to use.
    const mates = evaluateQuery(context, qBodyType(sketchPlane, BodyType.MATE_CONNECTOR));
    if (size(mates) > 0)
        return plane(evMateConnector(context, { "mateConnector" : mates[0] }));
    var faces = qEntityFilter(sketchPlane, EntityType.FACE);
    if (size(evaluateQuery(context, faces)) == 0)
        faces = qOwnedByBody(sketchPlane, EntityType.FACE);
    return evPlane(context, { "face" : faces });
}

export function swingLinkageSketch(context is Context, id is Id, sketchPlane is Plane,
        g is map, travel is ValueWithUnits, opts is map) returns Query
{
    // BOTH POSES FIRST, then one sketch. A side that cannot close has to stop
    // the feature before any geometry exists, or the tree carries half a
    // mechanism under a red error and the half looks fine.
    var poses = [];
    for (var s in [{ "side" : -1, "tag" : "right" }, { "side" : 1, "tag" : "left" }])
    {
        const p = swingPose(g, s.side, travel);
        if (!p.closes)
            throw regenError("The " ~ s.tag ~ " four-bar cannot close at this crank input: it is past the assembly limit. Back the input off.");
        poses = append(poses, mergeMaps(p, { "tag" : s.tag }));
    }

    var maxZ = 0;
    var halfY = g.wingPivotX;
    for (var p in poses)
    {
        maxZ = max(maxZ, max(p.top[1], p.foot[1]));
        halfY = max(halfY, max(abs(p.top[0]), abs(p.foot[0])));
    }

    const sk = newSketchOnPlane(context, id, { "sketchPlane" : sketchPlane });
    const shaft = swingShaft(g);
    for (var p in poses)
    {
        // The three moving links are solid; the ground link and the panel
        // standoff are construction, because neither is a part -- the ground
        // is the servo-to-pivot distance and the standoff is a bracket.
        skLineSegment(sk, "crank_" ~ p.tag, {
                "start" : swingTo2d(g, shaft), "end" : swingTo2d(g, p.tip) });
        skLineSegment(sk, "coupler_" ~ p.tag, {
                "start" : swingTo2d(g, p.tip), "end" : swingTo2d(g, p.joint) });
        skLineSegment(sk, "rocker_" ~ p.tag, {
                "start" : swingTo2d(g, p.pivot), "end" : swingTo2d(g, p.joint) });
        skLineSegment(sk, "panel_" ~ p.tag, {
                "start" : swingTo2d(g, p.foot), "end" : swingTo2d(g, p.top) });
        skLineSegment(sk, "ground_" ~ p.tag, {
                "start" : swingTo2d(g, p.pivot), "end" : swingTo2d(g, shaft),
                "construction" : true });
        if (abs(g.wingNormOffset) > 1e-9)
        {
            skLineSegment(sk, "standoff_" ~ p.tag, {
                    "start" : swingTo2d(g, p.joint),
                    "end" : swingTo2d(g, p.origin), "construction" : true });
        }
        skPoint(sk, "pivot_" ~ p.tag, { "position" : swingTo2d(g, p.pivot) });
    }
    skPoint(sk, "shaft", { "position" : swingTo2d(g, shaft) });

    if (opts.drawFloor)
    {
        const half = halfY + 20;
        skLineSegment(sk, "floor", {
                "start" : swingTo2d(g, vector(-half, -g.wheelRadius)),
                "end" : swingTo2d(g, vector(half, -g.wheelRadius)),
                "construction" : true });
    }

    if (opts.drawKeepout)
    {
        // Keep-out heights are ABOVE THE FLOOR, matching what the study
        // prints; the sketch is measured from the axle, so every one drops by
        // the wheel radius on the way in. An unbounded face is drawn to the
        // top of whatever the mechanism reaches, since it has to be drawn
        // somewhere and a line at 1e9 is not a drawing.
        for (var i = 0; i < size(SWING_KEEPOUT); i += 1)
        {
            const k = SWING_KEEPOUT[i];
            const zLo = k.zLo == undefined ? -g.wheelRadius : k.zLo - g.wheelRadius;
            const zHi = k.zHi == undefined ? maxZ + 20 : k.zHi - g.wheelRadius;
            const corner = [vector(-k.halfWidth, zLo), vector(k.halfWidth, zLo),
                            vector(k.halfWidth, zHi), vector(-k.halfWidth, zHi)];
            for (var j = 0; j < 4; j += 1)
            {
                skLineSegment(sk, "keepout_" ~ k.name ~ "_" ~ toString(j), {
                        "start" : swingTo2d(g, corner[j]),
                        "end" : swingTo2d(g, corner[j == 3 ? 0 : j + 1]),
                        "construction" : true });
            }
        }
    }

    skSolve(sk);
    return qCreatedBy(id, EntityType.EDGE);
}

$split

annotation { "Feature Type Name" : "AOW swing linkage" }
export const aowSwingLinkage = defineFeature(function(context is Context, id is Id,
                                                      definition is map)
    precondition
    {
        annotation { "Name" : "Sketch plane",
                     "Filter" : QueryFilterCompound.ALLOWS_PLANE,
                     "MaxNumberOfPicks" : 1 }
        definition.sketchPlane is Query;

        // THE POINT OF THE FEATURE. Insert it again on the same plane with a
        // different input and the two sketches overlay; four of them is the
        // stroke. Signed, because the pair co-rotates and rest is the MIDDLE
        // of the range rather than one end of it.
        annotation { "Name" : "Crank input" }
        isAngle(definition.crankTravel, { (degree) : [-360, 0, 360] } as AngleBoundSpec);

        annotation { "Group Name" : "Mechanism", "Collapsed By Default" : true }
        {
$mech_ui
        }

        annotation { "Group Name" : "Drawing", "Collapsed By Default" : true }
        {
            annotation { "Name" : "Floor line", "Default" : true }
            definition.drawFloor is boolean;

            annotation { "Name" : "Chassis keep-out", "Default" : true }
            definition.drawKeepout is boolean;

            annotation { "Name" : "Shift the whole drawing up" }
            isLength(definition.zOffset, { (millimeter) : [-1000, 0, 1000] } as LengthBoundSpec);
        }
    }
    {
        swingLinkageSketch(context, id + "sketch",
                swingSketchPlane(context, definition.sketchPlane),
                swingParams(definition), definition.crankTravel,
                { "drawFloor" : definition.drawFloor,
                  "drawKeepout" : definition.drawKeepout });
    });
''')


def build_fs(cfg: dict, vals: dict, ver: str = "3044",
             config_path: str = CONFIG) -> str:
    rows = []
    for fs, _attr, label, lo, hi in LENGTHS:
        rows.append(
            f'            annotation {{ "Name" : "{label}" }}\n'
            f'            isLength(definition.{fs}, {{ (millimeter) : '
            f'[{lo:g}, {vals[fs]:.6g}, {hi:g}] }} as LengthBoundSpec);')
    for fs, _attr, label, lo, hi in ANGLES:
        rows.append(
            f'            annotation {{ "Name" : "{label}" }}\n'
            f'            isAngle(definition.{fs}, {{ (degree) : '
            f'[{lo:g}, {vals[fs]:.6g}, {hi:g}] }} as AngleBoundSpec);')
    stroke = (cfg.get("stroke") or {}).get("crank_travel_deg")
    return FS.substitute(
        ver=ver, config=config_path, keepout=keepout_fs(cfg),
        stroke="unknown" if stroke is None else f"{float(stroke):.1f}",
        split=SPLIT_MARK, mech_ui="\n\n".join(rows))


def lint_fs(text: str) -> None:
    """Generator mistakes only a human reading the studio would otherwise see.

    `--check` compiles the geometry layer and never the UI layer, so a mistake
    in the dialog reaches the document intact and every feature in the studio
    goes red at once. Braces cannot be doubled here the way an f-string doubles
    them -- that is why this is a Template -- but an unsubstituted `$name` and
    an unbalanced brace both still land silently.
    """
    if bad := re.findall(r"\$\{?\w+", text):
        raise SystemExit(f"unsubstituted Template placeholders: {bad[:5]}")
    # A COMMAND IN BACKTICKS is rejected by an edge filter in front of the
    # Onshape API, 403 with an nginx page rather than anything from Onshape.
    # `cad_servo_mount._strip_comments` records this for the eval endpoint;
    # the CONTENTS endpoint refuses it too, which cost a push on 2026-09-03 --
    # the header cited its own regeneration command in backticks. Backticked
    # IDENTIFIERS are fine and both existing studios are full of them, so the
    # test is the space.
    if bad := re.findall(r"`[^`\n]* [^`\n]*`", text):
        raise SystemExit(
            "generated FeatureScript quotes a command in backticks, which the "
            "edge filter in front of the Onshape API 403s:\n  "
            + "\n  ".join(bad[:5]))
    depth = text.count("{") - text.count("}")
    if depth:
        raise SystemExit(f"generated FeatureScript has {depth:+d} unbalanced braces")


# --------------------------------------------------------------------------
# --check: run the ported four-bar on Onshape and compare it to this repo's


def check_wrapper(fs: str, vals: dict, travels: list[float]) -> str:
    """Rewrite the geometry layer into the bare function expression eval wants.

    DEFINITION ORDER MATTERS HERE AND NOWHERE ELSE: the rewrite turns each
    top-level function into a `const f = function(...)` STATEMENT inside one
    body, and a statement cannot call a const declared below it. Every callee
    in the emitted file sits above its caller for that reason.
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

    g = ", ".join(f'"{k}" : {v:.10g}' for k, v in vals.items()) + ', "zOffset" : 0'
    probes = []
    for n, t in enumerate(travels):
        for side, tag in ((-1, "right"), (1, "left")):
            probes.append(f"""
    {{
        const p = swingPose(g, {side}, {t:.10g} * degree);
        println("t{n}_{tag}_closes=" ~ toString(p.closes));
        if (p.closes)
        {{
            println("t{n}_{tag}_joint=" ~ toString(p.joint[0]) ~ "," ~ toString(p.joint[1]));
            println("t{n}_{tag}_tip=" ~ toString(p.tip[0]) ~ "," ~ toString(p.tip[1]));
            println("t{n}_{tag}_foot=" ~ toString(p.foot[0]) ~ "," ~ toString(p.foot[1]));
            println("t{n}_{tag}_top=" ~ toString(p.top[0]) ~ "," ~ toString(p.top[1]));
        }}
    }}""")
    # The sketch is built too, on a datum plane made here, THROUGH
    # swingSketchPlane -- so the check covers the exact call chain the feature
    # uses. It proves the drawing half runs: a name collision or a zero-length
    # segment throws at skSolve, and a sketch that draws nothing at all reports
    # zero rather than throwing (see swingSketchPlane). The pose printouts
    # above would have noticed neither.
    build = """
    {
        const pid = makeId("chkplane");
        opPlane(context, pid, { "plane" : XY_PLANE,
                "width" : 400 * millimeter, "height" : 400 * millimeter });
        const q = swingLinkageSketch(context, makeId("chksketch"),
                swingSketchPlane(context, qCreatedBy(pid, EntityType.BODY)),
                g, 0 * degree, { "drawFloor" : true, "drawKeepout" : true });
        println("sketch_edges=" ~ toString(size(evaluateQuery(context, q))));
    }"""
    body = "\n".join(out)
    return (f"function(context is Context, queries)\n{{\n{body}\n"
            f"    const g = {{ {g} }};\n" + "\n".join(probes) + build + "\n}\n")


def expected(lk: SwingLinkage, travels: list[float]) -> dict[str, str]:
    """What this repo's solver says, keyed the way the FeatureScript prints."""
    want = {}
    for n, t in enumerate(travels):
        for side, tag in ((-1, "right"), (1, "left")):
            lk.reset()
            p = lk.pose(side, t)
            key = f"t{n}_{tag}"
            want[f"{key}_closes"] = "false" if p is None else "true"
            if p is None:
                continue
            for name, v in (("joint", p["joint"]), ("tip", p["crank_tip"]),
                            ("foot", p["foot"]), ("top", p["top"])):
                want[f"{key}_{name}"] = (float(v[0]), float(v[1]))
    return want


def check(text: str, lk: SwingLinkage, vals: dict, travels: list[float],
          target: str | None) -> bool:
    from . import onshape

    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text, vals, travels), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    got = dict(l.split("=", 1) for l in console.splitlines() if "=" in l)
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console)
        print(onshape.budget_line())
        return False

    ok = True
    print(f"  {'probe':22} {'python':>19} {'onshape':>19}")
    for key, want in expected(lk, travels).items():
        raw = got.get(key)
        if raw is None:
            # A MISSING key is a failure: the print never ran.
            print(f"  {key:22} {'--':>19} {'MISSING':>19}  FAIL")
            ok = False
            continue
        if isinstance(want, str):
            bad = raw.strip() != want
            print(f"  {key:22} {want:>19} {raw.strip():>19}  "
                  f"{'FAIL' if bad else 'ok'}")
        else:
            gv = [float(v) for v in raw.split(",")]
            bad = max(abs(a - b) for a, b in zip(want, gv)) > 1e-4
            print(f"  {key:22} {want[0]:9.4f},{want[1]:9.4f} "
                  f"{gv[0]:9.4f},{gv[1]:9.4f}  {'FAIL' if bad else 'ok'}")
        ok = ok and not bad
    # A FLOOR, not an equality, because the exact count is config-dependent:
    # each keep-out box in `clearance.panel_keepout` adds four. Eight is the
    # four solid links per side, twice, and nothing can drop below it. The
    # default config measures 17 -- 12 links, the floor line, and one box --
    # which also settles that construction geometry does count as an EDGE.
    edges = int(got.get("sketch_edges") or 0)
    bad = edges < 8
    print(f"  {'sketch_edges':22} {'>= 8':>19} {edges:>19}  "
          f"{'FAIL' if bad else 'ok'}")
    ok = ok and not bad
    print(onshape.budget_line())
    return ok


def verify_studio(text: str, target: str) -> bool:
    """Compile the PUSHED studio and list what it defines. One billable call.

    The only check that sees the UI layer: `--check` throws everything below
    SPLIT_MARK away, so a mistake in the dialog compiles nowhere and the check
    still reports green.
    """
    import json
    from . import onshape

    want = text.count('"Feature Type Name"')
    url = onshape.resolve(target, target)
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


def check_travels(lk: SwingLinkage, cfg: dict) -> list[float]:
    """Crank inputs to probe: fractions of the config's own stroke.

    Clipped to what the mechanism can actually reach, so a config whose saved
    stroke is stale asks a question with an answer instead of comparing two
    "does not close" strings.
    """
    from swing_linkage import assembly_limit

    stroke = (cfg.get("stroke") or {}).get("crank_travel_deg")
    span = float(stroke) if stroke else assembly_limit(lk)
    span = min(span, assembly_limit(lk))
    return [round(f * span, 4) for f in CHECK_FRACTIONS]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=CONFIG,
                    help="swing linkage config the dialog defaults come from")
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044",
                    help="std version. A studio pinned older behaves "
                         "differently; --check always runs at the CURRENT one")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="",
                    default=None,
                    help="run the ported four-bar on Onshape and compare it to "
                         "this repo's solver. ONE billable call; defaults to "
                         "the `check` tab")
    ap.add_argument("--push", metavar="TAB|URL",
                    help="replace a Feature Studio's contents. Needs its OWN "
                         "tab -- cad_layout and cad_servo_mount own theirs")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the check script instead of spending a call")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    lk, vals = resolved(cfg)
    text = build_fs(cfg, vals, args.fs_version, args.config)
    lint_fs(text)
    Path(args.output).write_text(text)
    print(f"wrote {len(text)} chars -> {args.output}")
    print("  " + "  ".join(f"{k}={v:.4g}" for k, v in vals.items()))

    travels = check_travels(lk, cfg)
    if args.dry_run:
        print(check_wrapper(text, vals, travels))
        return
    if args.check is not None and not check(text, lk, vals, travels, args.check or None):
        raise SystemExit("check FAILED -- not pushing")
    if args.push is not None:
        from . import onshape
        if args.push in OWNED:
            raise SystemExit(
                f"refusing to push at `{args.push}`: {OWNED[args.push]} "
                f"overwrites that tab wholesale. Give this feature its own "
                f"Feature Studio tab (free, in the browser) and name it in "
                f"config/onshape.yaml.")
        url = onshape.resolve(args.push, args.push)
        onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}")
        # Always, not on a flag. A push CANNOT fail on bad FeatureScript -- the
        # contents endpoint takes any text -- so without this a broken studio
        # lands silently and is found by a human seeing red in the tree.
        ok = verify_studio(text, args.push)
        print(onshape.budget_line())
        if not ok:
            raise SystemExit("pushed, but the studio does not compile")


if __name__ == "__main__":
    main()
