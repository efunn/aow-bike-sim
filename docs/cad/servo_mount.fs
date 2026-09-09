FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_servo_mount --push horn_features
 *
 * The command above is deliberately BARE. Wrapped in backticks it is shell
 * command substitution, and an edge filter in front of the Onshape API rejects
 * the whole push with a bare nginx 403 -- no JSON, so it never reaches Onshape.
 * Backticks elsewhere are fine and so is the bare command; only the two
 * together trip it. Measured 2026-08-25, one probe each.
 *
 * Numbers come from config/bike_params_cad.yaml, which cites
 * docs/robotis/XC-330.pdf for every measured one. The four well-profile
 * dimensions are marked GUESS there: the shape is from a description of the
 * existing dynamixel_wrench_with_idler part, not a measurement of it.
 *
 * The mounting datum is the OUTER FACE OF THE HORN with +Z pointing out of the
 * servo -- the same datum cad_layout uses, so a mate connector that lands a
 * ROBOTIS STEP correctly also drives this feature correctly.
 */

export const SERVO_MOUNT_TABLE = {
    "XC330" : {
        "hornBoltCircle" : 12 * millimeter,
        "hornHoleDia" : 1.6 * millimeter,
        "hornHoleDepthMax" : 3 * millimeter,
        "hornHoleCount" : 4,
        "hornDiameter" : 16 * millimeter,
        "hornThickness" : 3 * millimeter,
        "pinClearance" : 0.2 * millimeter,
        "pinLength" : 2.6 * millimeter,
        "tipChamfer" : 0 * millimeter,
        "rootRelief" : 0.8 * millimeter,
        "rootWidth" : 1 * millimeter,
        "rootChamfer" : 0.6 * millimeter,
        "boreClearance" : 0.1 * millimeter,
        "boreMouthChamfer" : 0.6 * millimeter,
        "boreUndercut" : 0.6 * millimeter,
        "boreLand" : 0 * millimeter,
        "caseOffset" : 0.4 * millimeter,
        "collarOuterDia" : 20 * millimeter,
        "collarRoof" : 2 * millimeter,
        "caseReliefDia" : 2 * millimeter,
        "casePinClearance" : 0.1 * millimeter,
        "casePinLength" : 3 * millimeter,
        "casePinReliefDia" : 4.3 * millimeter,
        "casePinReliefDepth" : 0.8 * millimeter,
        "casePinRootChamfer" : 0.6 * millimeter,
        "casePinRowOffset" : 15 * millimeter,
        "caseSideClearance" : 0.05 * millimeter,
        "caseNestClearance" : 0.1 * millimeter,
        "caseNestLength" : 6.8 * millimeter,
        "caseTopWall" : 2.3 * millimeter,
        "caseBottomWall" : 1.6 * millimeter,
        "caseGripLength" : 2 * millimeter,
        "caseCapThickness" : 4 * millimeter,
        "caseFaceClearance" : 0 * millimeter,
        "caseWrapLength" : 10 * millimeter,
        "caseDepth" : 23 * millimeter,
        "caseWidth" : 20 * millimeter,
        "caseHeight" : 34 * millimeter,
        "shaftFromEnd" : 9.5 * millimeter,
        "caseHoleSpanX" : 16 * millimeter
    },
    "XL330" : {
        "hornBoltCircle" : 12 * millimeter,
        "hornHoleDia" : 1.6 * millimeter,
        "hornHoleDepthMax" : 3 * millimeter,
        "hornHoleCount" : 4,
        "hornDiameter" : 16 * millimeter,
        "hornThickness" : 3 * millimeter,
        "pinClearance" : 0.2 * millimeter,
        "pinLength" : 2.6 * millimeter,
        "tipChamfer" : 0 * millimeter,
        "rootRelief" : 0.8 * millimeter,
        "rootWidth" : 1 * millimeter,
        "rootChamfer" : 0.6 * millimeter,
        "boreClearance" : 0.1 * millimeter,
        "boreMouthChamfer" : 0.6 * millimeter,
        "boreUndercut" : 0.6 * millimeter,
        "boreLand" : 0 * millimeter,
        "caseOffset" : 0.4 * millimeter,
        "collarOuterDia" : 20 * millimeter,
        "collarRoof" : 2 * millimeter,
        "caseReliefDia" : 2 * millimeter,
        "casePinClearance" : 0.1 * millimeter,
        "casePinLength" : 3 * millimeter,
        "casePinReliefDia" : 4.3 * millimeter,
        "casePinReliefDepth" : 0.8 * millimeter,
        "casePinRootChamfer" : 0.6 * millimeter,
        "casePinRowOffset" : 15 * millimeter,
        "caseSideClearance" : 0.05 * millimeter,
        "caseNestClearance" : 0.1 * millimeter,
        "caseNestLength" : 6.8 * millimeter,
        "caseTopWall" : 2.3 * millimeter,
        "caseBottomWall" : 1.6 * millimeter,
        "caseGripLength" : 2 * millimeter,
        "caseCapThickness" : 4 * millimeter,
        "caseFaceClearance" : 0 * millimeter,
        "caseWrapLength" : 10 * millimeter,
        "caseDepth" : 23 * millimeter,
        "caseWidth" : 20 * millimeter,
        "caseHeight" : 34 * millimeter,
        "shaftFromEnd" : 9.5 * millimeter,
        "caseHoleSpanX" : 16 * millimeter
    }
};

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
{
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
        skLineSegment(sk, "s" ~ i, { "start" : poly[i],
                                     "end"   : poly[(i + 1) % size(poly)] });
}

/** Sketch a profile, revolve it a full turn, and bin the sketch. */
export function revolveProfile(context is Context, id is Id, tag is string,
                               profPlane is Plane, axis is Line, pts is array)
{
    var sk = newSketchOnPlane(context, id + tag, { "sketchPlane" : profPlane });
    skPolygon(sk, pts);
    skSolve(sk);
    opRevolve(context, id + (tag ~ "Rev"), {
            "entities"     : qSketchRegion(id + tag),
            "axis"         : axis,
            "angleForward" : 360 * degree });
    opDeleteBodies(context, id + (tag ~ "Del"), {
            "entities" : qCreatedBy(id + tag, EntityType.BODY) });
}

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
{
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
    const over         = 1 * millimeter;

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
        [vector(0 * millimeter, 0 * millimeter),
            vector(pinR, 0 * millimeter),
            vector(pinR, -pinL + tipCh),
            vector(pinR - tipCh, -pinL),
            vector(0 * millimeter, -pinL)]);
    revolveProfile(context, id, "relief", pinPlane, pinAxis,
        [vector(pinR, 0 * millimeter),
            vector(pinR + relW, 0 * millimeter),
            vector(pinR + relW, relD),
            vector(pinR + relCh, relD),
            vector(pinR, relD - relCh)]);
    revolveProfile(context, id, "bore", axialPlane, shaftAxis,
        [vector(0 * millimeter, -wellT - over),
            vector(boreR + mouthCh, -wellT - over),
            vector(boreR + mouthCh, -wellT),
            vector(boreR, -wellT + mouthCh),
            vector(boreR, -boreUndercut - boreLand),
            vector(boreR + boreUndercut, -boreLand),
            vector(boreR + boreUndercut, 0 * millimeter),
            vector(0 * millimeter, 0 * millimeter)]);
    if (opt.collar)
        revolveProfile(context, id, "collar", axialPlane, shaftAxis,
            [vector(0 * millimeter, -wellT),
            vector(collarR, -wellT),
            vector(collarR, roof),
            vector(0 * millimeter, roof)]);

    // Ring the pin and its groove round the bolt circle. One opPattern each,
    // not one for both, so the two stay separable: pins get unioned and
    // grooves get subtracted, and a query that mixes them cannot do either.
    var xf = [];
    var names = [];
    for (var i = 1; i < t.hornHoleCount; i += 1)
    {
        xf = append(xf, rotationAround(shaftAxis,
                                       i * (360 / t.hornHoleCount) * degree));
        names = append(names, "i" ~ i);
    }
    opPattern(context, id + "pinRing", {
            "entities"      : qCreatedBy(id + "pinRev", EntityType.BODY),
            "transforms"    : xf,
            "instanceNames" : names });
    opPattern(context, id + "reliefRing", {
            "entities"      : qCreatedBy(id + "reliefRev", EntityType.BODY),
            "transforms"    : xf,
            "instanceNames" : names });

    return {
        "pins" : qUnion([qCreatedBy(id + "pinRev", EntityType.BODY),
                         qCreatedBy(id + "pinRing", EntityType.BODY)]),
        "cutters" : qUnion([qCreatedBy(id + "reliefRev", EntityType.BODY),
                            qCreatedBy(id + "reliefRing", EntityType.BODY),
                            qCreatedBy(id + "boreRev", EntityType.BODY)]),
        "collar" : qCreatedBy(id + "collarRev", EntityType.BODY)
    };
}

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
{
    const standalone = isQueryEmpty(context, target);
    const g = servoMountGeometry(context, id + "geom", cs,
                                 mergeMaps(opt, { "collar" : standalone }));
    const into = standalone ? g.collar : target;

    opBoolean(context, id + "cut", {
            "tools"         : g.cutters,
            "targets"       : into,
            "operationType" : BooleanOperationType.SUBTRACTION });
    // UNION takes `tools` ONLY -- every body to be merged, target included --
    // and NO `targets` key. Written the way SUBTRACTION is written, it unions
    // the four pins with each other, which does nothing because they do not
    // touch, and leaves the collar alone: four loose pins and a bare collar,
    // five bodies, with no error anywhere. Volume said so before body count
    // did, and only because the shortfall was exactly four pins.
    opBoolean(context, id + "add", {
            "tools"         : qUnion([g.pins, into]),
            "operationType" : BooleanOperationType.UNION });
    return into;
}

/** A rectangular solid in the datum frame: |x| <= xHalf, y0..y1, z0..z1. */
export function boxSolid(context is Context, id is Id, tag is string,
                         cs is CoordSystem, xHalf, y0, y1, z0, z1)
{
    var sk = newSketchOnPlane(context, id + tag, {
            "sketchPlane" : plane(cs.origin + z0 * cs.zAxis, cs.zAxis, cs.xAxis) });
    skRectangle(sk, "r", { "firstCorner"  : vector(-xHalf, y0),
                           "secondCorner" : vector(xHalf, y1) });
    skSolve(sk);
    opExtrude(context, id + (tag ~ "Ext"), {
            "entities"  : qSketchRegion(id + tag),
            "direction" : cs.zAxis,
            "endBound"  : BoundingType.BLIND,
            "endDepth"  : z1 - z0 });
    opDeleteBodies(context, id + (tag ~ "Del"), {
            "entities" : qCreatedBy(id + tag, EntityType.BODY) });
}

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
{
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
    const over     = 1 * millimeter;

    const y0    = endY - opt.caseWrapLength;   // the open end, toward the shaft
    const fc    = opt.caseFaceClearance;
    const seatZ = top ? hornZ + fc : backZ - fc;
    const outZ  = top ? cs.zAxis : -cs.zAxis;

    if (top)
    {
        boxSolid(context, id, "shell", cs, topOuter,
                 y0, endY + sc + opt.caseTopWall,
                 hornZ - skirt, seatZ + opt.caseCapThickness);
        boxSolid(context, id, "cav", cs, inner,
                 y0 - over, endY + sc, hornZ - skirt - over, seatZ);
    }
    else
    {
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
    }

    // One pin at +rowX, mirrored to -rowX. Two per face, not four: see above.
    const yA        = cross(cs.zAxis, cs.xAxis);
    const pinOrigin = cs.origin + rowX * cs.xAxis + rowY * yA + seatZ * cs.zAxis;
    const pinPlane  = plane(pinOrigin, -cross(outZ, cs.xAxis), cs.xAxis);
    const pinAxis   = line(pinOrigin, outZ);

    revolveProfile(context, id, "pin", pinPlane, pinAxis,
        [vector(0 * millimeter, 0 * millimeter),
            vector(pinR, 0 * millimeter),
            vector(pinR, -pinL + tipCh),
            vector(pinR - tipCh, -pinL),
            vector(0 * millimeter, -pinL)]);
    revolveProfile(context, id, "relief", pinPlane, pinAxis,
        [vector(pinR, 0 * millimeter),
            vector(pinR + relW, 0 * millimeter),
            vector(pinR + relW, relD),
            vector(pinR + relCh, relD),
            vector(pinR, relD - relCh)]);

    const mirror = [transform(-2 * rowX * cs.xAxis)];
    opPattern(context, id + "pinRing", {
            "entities"      : qCreatedBy(id + "pinRev", EntityType.BODY),
            "transforms"    : mirror,
            "instanceNames" : ["i1"] });
    opPattern(context, id + "reliefRing", {
            "entities"      : qCreatedBy(id + "reliefRev", EntityType.BODY),
            "transforms"    : mirror,
            "instanceNames" : ["i1"] });

    return {
        "shell" : qCreatedBy(id + "shellExt", EntityType.BODY),
        "pins"  : qUnion([qCreatedBy(id + "pinRev", EntityType.BODY),
                          qCreatedBy(id + "pinRing", EntityType.BODY)]),
        "cutters" : qUnion([qCreatedBy(id + "cavExt", EntityType.BODY),
                            qCreatedBy(id + "nestExt", EntityType.BODY),
                            qCreatedBy(id + "reliefRev", EntityType.BODY),
                            qCreatedBy(id + "reliefRing", EntityType.BODY)])
    };
}

/** Case shell: cavities out, then pins in. Same order rule as the horn. */
export function caseShellBuild(context is Context, id is Id, cs is CoordSystem,
                               opt is map, target is Query) returns Query
{
    const g = caseShellGeometry(context, id + "geom", cs, opt);
    var into = g.shell;
    if (!isQueryEmpty(context, target))
    {
        opBoolean(context, id + "merge", {
                "tools"         : qUnion([g.shell, target]),
                "operationType" : BooleanOperationType.UNION });
        into = target;
    }
    opBoolean(context, id + "cut", {
            "tools"         : g.cutters,
            "targets"       : into,
            "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "add", {
            "tools"         : qUnion([g.pins, into]),
            "operationType" : BooleanOperationType.UNION });
    return into;
}

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
{
    const useCS = opt.flip ? coordSystem(cs.origin, -cs.xAxis, cs.zAxis) : cs;
    var made = [];
    // `!= false`, not `== true`. These are new parameters on a feature that is
    // already inserted in live documents; if Onshape does not backfill a
    // default into an existing instance it reads as undefined, and under
    // `== true` both halves would vanish on the next regeneration. Unset
    // builds. Same reasoning as the group tickboxes in cad_layout.
    if (opt.makeTop != false)
        made = append(made, caseShellBuild(context, id + "top", useCS,
                mergeMaps(opt, { "part" : "TOP" }), qNothing()));
    if (opt.makeBottom != false)
        made = append(made, caseShellBuild(context, id + "bot", useCS,
                mergeMaps(opt, { "part" : "BOTTOM" }), qNothing()));
    return qUnion(made);
}


// ==== UI LAYER BELOW -- dropped by --check ====

export enum ServoModel
{
    annotation { "Name" : "XC330" }
    XC330,
    annotation { "Name" : "XL330" }
    XL330
}

export function servoKey(m is ServoModel) returns string
{
    if (m == ServoModel.XC330) return "XC330";
    if (m == ServoModel.XL330) return "XL330";
    return "XC330";
}

annotation { "Feature Type Name" : "X330 horn pin",
             "Filter Selector" : "allparts" }
export const x330HornPin = defineFeature(function(context is Context, id is Id,
                                                  definition is map)
    precondition
    {
        annotation { "Name" : "Servo" }
        definition.servo is ServoModel;

        annotation { "Name" : "Horn datum",
                     "Filter" : BodyType.MATE_CONNECTOR,
                     "MaxNumberOfPicks" : 1 }
        definition.datum is Query;

        annotation { "Name" : "Part to modify (leave empty for a collar)",
                     "Filter" : EntityType.BODY && BodyType.SOLID }
        definition.target is Query;

        annotation { "Group Name" : "Fit", "Collapsed By Default" : true }
        {
            annotation { "Name" : "Pin clearance (diametral)" }
            isLength(definition.pinClearance, { (millimeter) : [0.0, 0.2, 1.0] } as LengthBoundSpec);

            annotation { "Name" : "Pin length" }
            isLength(definition.pinLength, { (millimeter) : [0.5, 2.6, 3.0] } as LengthBoundSpec);

            annotation { "Name" : "Pin tip lead-in" }
            isLength(definition.tipChamfer, { (millimeter) : [0.0, 0, 1.0] } as LengthBoundSpec);

            annotation { "Name" : "Root relief depth" }
            isLength(definition.rootRelief, { (millimeter) : [0.0, 0.8, 2.0] } as LengthBoundSpec);

            annotation { "Name" : "Root relief width" }
            isLength(definition.rootWidth, { (millimeter) : [0.0, 1, 2.0] } as LengthBoundSpec);

            annotation { "Name" : "Root relief inner chamfer" }
            isLength(definition.rootChamfer, { (millimeter) : [0.0, 0.6, 1.0] } as LengthBoundSpec);

            annotation { "Name" : "Well clearance (diametral)" }
            isLength(definition.boreClearance, { (millimeter) : [0.0, 0.1, 1.0] } as LengthBoundSpec);

            annotation { "Name" : "Well mouth lead-in" }
            isLength(definition.boreMouthChamfer, { (millimeter) : [0.0, 0.6, 2.0] } as LengthBoundSpec);

            annotation { "Name" : "Well undercut chamfer (radial = axial)" }
            isLength(definition.boreUndercut, { (millimeter) : [0.0, 0.6, 2.0] } as LengthBoundSpec);

            annotation { "Name" : "Well land at the undercut" }
            isLength(definition.boreLand, { (millimeter) : [0.0, 0, 3.0] } as LengthBoundSpec);

            annotation { "Name" : "Clearance to the case face" }
            isLength(definition.caseOffset, { (millimeter) : [0.0, 0.4, 2.0] } as LengthBoundSpec);

            annotation { "Name" : "Collar outside diameter" }
            isLength(definition.collarOuterDia, { (millimeter) : [16.5, 20, 60.0] } as LengthBoundSpec);

            annotation { "Name" : "Collar roof thickness" }
            isLength(definition.collarRoof, { (millimeter) : [0.5, 2, 20.0] } as LengthBoundSpec);
        }
    }
    {
        servoMountBuild(context, id + "build",
                evMateConnector(context, { "mateConnector" : definition.datum }),
                { "servo" : servoKey(definition.servo),
                  "pinClearance" : definition.pinClearance,
                  "pinLength" : definition.pinLength,
                  "tipChamfer" : definition.tipChamfer,
                  "rootRelief" : definition.rootRelief,
                  "rootWidth" : definition.rootWidth,
                  "rootChamfer" : definition.rootChamfer,
                  "boreClearance" : definition.boreClearance,
                  "boreMouthChamfer" : definition.boreMouthChamfer,
                  "boreUndercut" : definition.boreUndercut,
                  "boreLand" : definition.boreLand,
                  "caseOffset" : definition.caseOffset,
                  "collarOuterDia" : definition.collarOuterDia,
                  "collarRoof" : definition.collarRoof },
                definition.target);
    });

annotation { "Feature Type Name" : "X330 case shell",
             "Filter Selector" : "allparts" }
export const x330CaseShell = defineFeature(function(context is Context, id is Id,
                                                    definition is map)
    precondition
    {
        annotation { "Name" : "Servo" }
        definition.servo is ServoModel;

        annotation { "Name" : "Horn datum",
                     "Filter" : BodyType.MATE_CONNECTOR,
                     "MaxNumberOfPicks" : 1 }
        definition.datum is Query;

        // Both halves from one feature. They were a Top/Bottom enum on two
        // separate features, which meant retyping every clearance twice --
        // and the numbers that must agree are precisely the ones describing
        // the joint between them.
        annotation { "Name" : "Top half (horn side)", "Default" : true }
        definition.makeTop is boolean;

        annotation { "Name" : "Bottom half (back)", "Default" : true }
        definition.makeBottom is boolean;

        annotation { "Name" : "Flip 180 degrees about the datum" }
        definition.flip is boolean;

        annotation { "Group Name" : "Fit", "Collapsed By Default" : true }
        {
            annotation { "Name" : "Pin clearance (diametral, in the Phi 2 bore)" }
            isLength(definition.casePinClearance, { (millimeter) : [0.0, 0.1, 1.0] } as LengthBoundSpec);

            annotation { "Name" : "Pin length" }
            isLength(definition.casePinLength, { (millimeter) : [0.5, 3, 4.0] } as LengthBoundSpec);

            annotation { "Name" : "Root relief diameter" }
            isLength(definition.casePinReliefDia, { (millimeter) : [0.0, 4.3, 8.0] } as LengthBoundSpec);

            annotation { "Name" : "Root relief depth" }
            isLength(definition.casePinReliefDepth, { (millimeter) : [0.0, 0.8, 2.0] } as LengthBoundSpec);

            annotation { "Name" : "Root relief inner chamfer" }
            isLength(definition.casePinRootChamfer, { (millimeter) : [0.0, 0.6, 1.0] } as LengthBoundSpec);

            annotation { "Name" : "Clearance on the case sides (per side)" }
            isLength(definition.caseSideClearance, { (millimeter) : [0.0, 0.05, 1.0] } as LengthBoundSpec);

            annotation { "Name" : "Nest clearance (per side)" }
            isLength(definition.caseNestClearance, { (millimeter) : [0.0, 0.1, 1.0] } as LengthBoundSpec);

            annotation { "Name" : "Top half wall" }
            isLength(definition.caseTopWall, { (millimeter) : [0.5, 2.3, 6.0] } as LengthBoundSpec);

            annotation { "Name" : "Bottom half wall" }
            isLength(definition.caseBottomWall, { (millimeter) : [0.5, 1.6, 6.0] } as LengthBoundSpec);

            annotation { "Name" : "Bottom half grip on the servo" }
            isLength(definition.caseGripLength, { (millimeter) : [0.0, 2, 10.0] } as LengthBoundSpec);

            annotation { "Name" : "Nest engagement" }
            isLength(definition.caseNestLength, { (millimeter) : [0.0, 6.8, 20.0] } as LengthBoundSpec);

            annotation { "Name" : "Cap thickness over the face" }
            isLength(definition.caseCapThickness, { (millimeter) : [0.5, 4, 20.0] } as LengthBoundSpec);

            annotation { "Name" : "Clearance on the horn/back faces" }
            isLength(definition.caseFaceClearance, { (millimeter) : [0.0, 0, 2.0] } as LengthBoundSpec);

            annotation { "Name" : "Wrap length from the far end" }
            isLength(definition.caseWrapLength, { (millimeter) : [2.0, 10, 16.5] } as LengthBoundSpec);
        }
    }
    {
        caseShellPair(context, id + "build",
                evMateConnector(context, { "mateConnector" : definition.datum }),
                { "servo" : servoKey(definition.servo),
                  "makeTop" : definition.makeTop,
                  "makeBottom" : definition.makeBottom,
                  "flip" : definition.flip,
                  "casePinClearance" : definition.casePinClearance,
                  "casePinLength" : definition.casePinLength,
                  "casePinReliefDia" : definition.casePinReliefDia,
                  "casePinReliefDepth" : definition.casePinReliefDepth,
                  "casePinRootChamfer" : definition.casePinRootChamfer,
                  "caseSideClearance" : definition.caseSideClearance,
                  "caseNestClearance" : definition.caseNestClearance,
                  "caseTopWall" : definition.caseTopWall,
                  "caseBottomWall" : definition.caseBottomWall,
                  "caseGripLength" : definition.caseGripLength,
                  "caseNestLength" : definition.caseNestLength,
                  "caseCapThickness" : definition.caseCapThickness,
                  "caseFaceClearance" : definition.caseFaceClearance,
                  "caseWrapLength" : definition.caseWrapLength });
    });
