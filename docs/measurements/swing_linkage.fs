FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* GENERATED, do not hand-edit: the next push replaces this studio
   wholesale. Regenerate with
       python -m aow_sim.cad_swing_linkage --config config/swing_linkage_smaller.yaml

   The co-rotating swing linkage, drawn as a sketch at a crank input you pick.
   Insert the feature once per pose you want to see -- 0, a third of the
   stroke, two thirds, the end -- all on the same plane. The overlay is the
   motion study; nothing here is an assembly and nothing mates.

   Stroke from the study: 136.6 deg of crank travel. Positive input deploys
   the RIGHT (-y) wing and lifts the left; negative swaps them. Past the
   assembly limit the four-bar cannot close and the feature errors rather than
   drawing a mechanism that has come apart.

   Sketch coordinates are u = -y, v = z, both in mm from the centreline and the
   REAR AXLE, which is where aowFourBarSketch draws too. The floor is at
   v = -wheelRadius.
*/

export const SWING_KEEPOUT = [
    { "name" : "chassis_core", "halfWidth" : 35, "zLo" : 50, "zHi" : undefined }
];

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

// ==== UI LAYER BELOW -- dropped by --check ====

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
            annotation { "Name" : "Servo shaft above the axle" }
            isLength(definition.servoOffset, { (millimeter) : [0, 50, 300] } as LengthBoundSpec);

            annotation { "Name" : "Wing pivot out from centreline" }
            isLength(definition.wingPivotX, { (millimeter) : [0, 5, 300] } as LengthBoundSpec);

            annotation { "Name" : "Wing pivot above the axle" }
            isLength(definition.wingPivotZ, { (millimeter) : [-300, -10, 300] } as LengthBoundSpec);

            annotation { "Name" : "Crank length" }
            isLength(definition.crankLength, { (millimeter) : [1, 32.4372, 300] } as LengthBoundSpec);

            annotation { "Name" : "Coupler length" }
            isLength(definition.couplerLength, { (millimeter) : [1, 59.6379, 400] } as LengthBoundSpec);

            annotation { "Name" : "Rocker length" }
            isLength(definition.rockerLength, { (millimeter) : [1, 48.0175, 300] } as LengthBoundSpec);

            annotation { "Name" : "Panel standoff from the rocker" }
            isLength(definition.wingNormOffset, { (millimeter) : [0, 15, 100] } as LengthBoundSpec);

            annotation { "Name" : "Panel foot along its own axis" }
            isLength(definition.wingZMin, { (millimeter) : [-300, -27.0824, 300] } as LengthBoundSpec);

            annotation { "Name" : "Panel top along its own axis" }
            isLength(definition.wingZMax, { (millimeter) : [-300, 72.9176, 300] } as LengthBoundSpec);

            annotation { "Name" : "Wheel radius (sets the floor)" }
            isLength(definition.wheelRadius, { (millimeter) : [0, 51.2, 300] } as LengthBoundSpec);

            annotation { "Name" : "Angle between the crank arms" }
            isAngle(definition.angleBetweenCranks, { (degree) : [0, 21.5344, 180] } as AngleBoundSpec);

            annotation { "Name" : "Panel angle off the rocker" }
            isAngle(definition.wingAngleFromRocker, { (degree) : [-180, 21.3737, 180] } as AngleBoundSpec);
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
