FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_steering --push steering_features
 *
 * Numbers from config/steering_cad.yaml, config/servo_mounts.yaml and
 * bike_params_cad.yaml, every derived one computed in aow_sim.cad_steering's
 * layout() and carried here as ST. Millimetres.
 *
 * MODULE FRAME: origin on the front axle's centre, +Z up the steering axis,
 * +Y forward, +X right. The steer servo's horn faces -Z at ST.zH; its body
 * runs back (-Y). In the bike the module tilts back by the rake about X.
 */

export const X330 = {
    "caseDepth" : 23 * millimeter,
    "caseWidth" : 20 * millimeter,
    "caseHeight" : 34 * millimeter,
    "shaftFromEnd" : 9.5 * millimeter,
    "hornThickness" : 3 * millimeter,
    "hornDiameter" : 16 * millimeter
};

export const ST = {
    "R" : 51.25 * millimeter,
    "zH" : 96.25 * millimeter,
    "zCF" : 99.25 * millimeter,
    "zBack" : 122.25 * millimeter,
    "hubBot" : 91.05 * millimeter,
    "socketFloor" : 94.35 * millimeter,
    "upTop" : 90.55 * millimeter,
    "lugTop" : 93.85 * millimeter,
    "nutFloor" : 87.55 * millimeter,
    "zS" : 83.05 * millimeter,
    "socketTop" : 86.35 * millimeter,
    "upSockTop" : 86.35 * millimeter,
    "sleeveTop" : 82.85 * millimeter,
    "blockBot" : 55.25 * millimeter,
    "blockTop" : 67.25 * millimeter,
    "caseBot" : 68.25 * millimeter,
    "sleeve" : 14.6 * millimeter,
    "nutTop" : 89.93 * millimeter,
    "tip" : 90.43 * millimeter,
    "headFace" : 80.905 * millimeter,
    "cskTop" : 82.855 * millimeter,
    "cbR" : 3.95 * millimeter,
    "cbTop" : 80.705 * millimeter,
    "shaftR" : 7 * millimeter,
    "xIn" : 13.5 * millimeter,
    "xOut" : 22.5 * millimeter,
    "xBoss" : 6.7 * millimeter,
    "knurlEnd" : 11.5 * millimeter,
    "xBlk" : 17.5 * millimeter,
    "ledge" : 4 * millimeter,
    "forkPocket" : 0.4 * millimeter,
    "zFS" : 61.25 * millimeter,
    "cheekIn" : 9.1 * millimeter,
    "forkHalf" : 12.1 * millimeter,
    "forkSlotX" : 12.5 * millimeter,
    "thrustR" : 9 * millimeter,
    "nutR" : 3.78164 * millimeter,
    "lugR0" : 4.78164 * millimeter,
    "upR" : 9 * millimeter,
    "lugHalf" : 1.2 * millimeter,
    "lugR1" : 8.91964 * millimeter,
    "sockHalf" : 1.35 * millimeter,
    "sockR0" : 2.8 * millimeter,
    "islandWall" : 1.2598 * millimeter,
    "hubR" : 9 * millimeter,
    "hubCbR" : 1.85 * millimeter,
    "hubCbDepth" : 1.8 * millimeter,
    "m2Engaged" : 2.6 * millimeter,
    "cbToSocket" : 1.04264 * millimeter,
    "cbToRim" : 1.15 * millimeter,
    "spR" : 5.5 * millimeter,
    "spFlat" : 3 * millimeter,
    "sockSpR" : 5.65 * millimeter,
    "sockSpFlat" : 3.15 * millimeter,
    "boreR" : 7.1 * millimeter,
    "inner" : 10.05 * millimeter,
    "topOuter" : 12.35 * millimeter,
    "nestBore" : 12.45 * millimeter,
    "botOuter" : 14.05 * millimeter,
    "cavR" : 9.5 * millimeter,
    "lowerPinSeat" : 0.515856 * millimeter,
    "lowerNearPins" : 1 * millimeter,
    "yServoNear" : 9.55 * millimeter,
    "yServoFar" : -24.55 * millimeter,
    "yWallNear" : 11.85 * millimeter,
    "yWallFar" : -26.85 * millimeter,
    "yNestNear" : 11.95 * millimeter,
    "yUpNear" : 13.55 * millimeter,
    "yUpFar" : -28.55 * millimeter,
    "yWrap" : -14.5 * millimeter,
    "cupHalf" : 12.35 * millimeter,
    "cupNear" : 11.85 * millimeter,
    "wallTop" : 110.75 * millimeter,
    "upBot" : 103.95 * millimeter,
    "upTopZ" : 126.25 * millimeter,
    "winNear" : -3 * millimeter,
    "winFar" : -14.5 * millimeter,
    "winBot" : 110.75 * millimeter,
    "winApex" : 108.657 * millimeter,
    "slotIn" : 6 * millimeter,
    "yP" : -35.05 * millimeter,
    "plateBack" : -39.65 * millimeter,
    "zLS" : 83.75 * millimeter,
    "zUS" : 120.25 * millimeter,
    "padBot" : 114.25 * millimeter,
    "tireOD" : 102.5 * millimeter,
    "tireW" : 24 * millimeter,
    "tireFlat" : 10 * millimeter,
    "tireMin" : 71 * millimeter,
    "webW" : 13 * millimeter,
    "hubW" : 13 * millimeter,
    "hubDiaA" : 12.5 * millimeter,
    "hubDiaB" : 11.5 * millimeter,
    "bossDia" : 6 * millimeter,
    "axleDia" : 4 * millimeter,
    "headH" : 1 * millimeter,
    "headDia" : 7 * millimeter,
    "axleHole" : 3.9 * millimeter,
    "bossChamferDeg" : 45 * degree,
    "tabL" : 45 * millimeter,
    "tabW" : 50 * millimeter,
    "tabT" : 5 * millimeter,
    "tabHole" : 5.1 * millimeter,
    "tabSpanX" : 38.1 * millimeter,
    "tabSpanD" : 25.4 * millimeter,
    "rakeDeg" : 15 * degree,
    "legHalf" : 7.5 * millimeter,
    "blockHalf" : 9 * millimeter,
    "spigotH" : 3 * millimeter,
    "m2Hole" : 2.1 * millimeter,
    "padHalf" : 7 * millimeter,
    "ridgeHalf" : 6 * millimeter,
    "bridgeLayer" : 0.2 * millimeter,
    "wellT" : 2.6 * millimeter
};

export const FIXTURE = {
    "joint_plate" : 4.6 * millimeter,
    "ridge_height" : 1.2 * millimeter,
    "ridge_flat" : 4.6 * millimeter,
    "ridge_clearance" : 0.15 * millimeter,
    "bridge_layer" : 0.2 * millimeter
};

export const HORN_OPT = {
    "servo" : "XC330",
    "pinClearance" : 0.1 * millimeter,
    "pinLength" : 1.6 * millimeter,
    "tipChamfer" : 0 * millimeter,
    "rootRelief" : 0 * millimeter,
    "rootWidth" : 1 * millimeter,
    "rootChamfer" : 0.6 * millimeter,
    "boreClearance" : 0 * millimeter,
    "boreMouthChamfer" : 0.4 * millimeter,
    "boreUndercut" : 0.6 * millimeter,
    "boreLand" : 0 * millimeter,
    "caseOffset" : 0.4 * millimeter,
    "collarOuterDia" : 20 * millimeter,
    "collarRoof" : 2 * millimeter
};

export const CASE_OPT = {
    "servo" : "XC330",
    "casePinClearance" : 0.1 * millimeter,
    "casePinLength" : 1.5 * millimeter,
    "casePinReliefDia" : 4.3 * millimeter,
    "casePinReliefDepth" : 0.8 * millimeter,
    "casePinRootChamfer" : 0.6 * millimeter,
    "caseSideClearance" : 0.05 * millimeter,
    "caseNestClearance" : 0.1 * millimeter,
    "caseTopWall" : 2.3 * millimeter,
    "caseBottomWall" : 1.6 * millimeter,
    "caseGripLength" : 11.5 * millimeter,
    "caseNestLength" : 6.8 * millimeter,
    "caseCapThickness" : 4 * millimeter,
    "caseFaceClearance" : 0 * millimeter,
    "caseWrapLength" : 10 * millimeter
};

export const SCREW_OPT = {
    "holeDia" : 3.6 * millimeter,
    "headDia" : 7.5 * millimeter,
    "holeDepth" : 12 * millimeter,
    "nutSlotWidth" : 6.55 * millimeter,
    "nutSlotThickness" : 3 * millimeter,
    "nutDepth" : 6.6 * millimeter,
    "nutSlotLength" : 12 * millimeter,
    "cskAngle" : 90 * degree
};

// ---- the servo-mount geometry, copied from the horn-mount-gen studio ----


export const SERVO_MOUNT_TABLE = {
    "XC330" : {
        "hornBoltCircle" : 12 * millimeter,
        "hornHoleDia" : 1.6 * millimeter,
        "hornHoleDepthMax" : 3 * millimeter,
        "hornHoleCount" : 4,
        "hornDiameter" : 16 * millimeter,
        "hornThickness" : 3 * millimeter,
        "pinClearance" : 0.2 * millimeter,
        "pinLength" : 1.3 * millimeter,
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
        "caseReliefDepthHorn" : 3.5 * millimeter,
        "caseReliefDepthBack" : 4.5 * millimeter,
        "casePinClearance" : 0.1 * millimeter,
        "casePinLength" : 1.5 * millimeter,
        "casePinReliefDia" : 4.3 * millimeter,
        "casePinReliefDepth" : 0.8 * millimeter,
        "casePinRootChamfer" : 0.6 * millimeter,
        "casePinRowOffset" : 15 * millimeter,
        "caseSideClearance" : 0.05 * millimeter,
        "caseNestClearance" : 0.1 * millimeter,
        "caseNestLength" : 6.8 * millimeter,
        "caseTopWall" : 2.3 * millimeter,
        "caseBottomWall" : 1.6 * millimeter,
        "caseGripLength" : 11.5 * millimeter,
        "caseCapThickness" : 4 * millimeter,
        "caseFaceClearance" : 0 * millimeter,
        "caseWrapLength" : 10 * millimeter,
        "caseWindowNear" : 3.5 * millimeter,
        "caseWindowDepth" : 9.55 * millimeter,
        "caseWrapOverhang" : 70 * degree,
        "idlerRecessOuterDia" : 6.7 * millimeter,
        "idlerRecessOuterDepth" : 1.2 * millimeter,
        "idlerRecessInnerDia" : 5.2 * millimeter,
        "idlerRecessInnerDepth" : 0.6 * millimeter,
        "idlerPlugClearance" : 0.2 * millimeter,
        "idlerEndGap" : 0 * millimeter,
        "idlerFaceGap" : 0 * millimeter,
        "idlerCollarDia" : 11 * millimeter,
        "idlerCollarThickness" : 3 * millimeter,
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
        "pinLength" : 1.3 * millimeter,
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
        "caseReliefDepthHorn" : 3.5 * millimeter,
        "caseReliefDepthBack" : 4.5 * millimeter,
        "casePinClearance" : 0.1 * millimeter,
        "casePinLength" : 1.5 * millimeter,
        "casePinReliefDia" : 4.3 * millimeter,
        "casePinReliefDepth" : 0.8 * millimeter,
        "casePinRootChamfer" : 0.6 * millimeter,
        "casePinRowOffset" : 15 * millimeter,
        "caseSideClearance" : 0.05 * millimeter,
        "caseNestClearance" : 0.1 * millimeter,
        "caseNestLength" : 6.8 * millimeter,
        "caseTopWall" : 2.3 * millimeter,
        "caseBottomWall" : 1.6 * millimeter,
        "caseGripLength" : 11.5 * millimeter,
        "caseCapThickness" : 4 * millimeter,
        "caseFaceClearance" : 0 * millimeter,
        "caseWrapLength" : 10 * millimeter,
        "caseWindowNear" : 3.5 * millimeter,
        "caseWindowDepth" : 9.55 * millimeter,
        "caseWrapOverhang" : 70 * degree,
        "idlerRecessOuterDia" : 6.7 * millimeter,
        "idlerRecessOuterDepth" : 1.2 * millimeter,
        "idlerRecessInnerDia" : 5.2 * millimeter,
        "idlerRecessInnerDepth" : 0.6 * millimeter,
        "idlerPlugClearance" : 0.2 * millimeter,
        "idlerEndGap" : 0 * millimeter,
        "idlerFaceGap" : 0 * millimeter,
        "idlerCollarDia" : 11 * millimeter,
        "idlerCollarThickness" : 3 * millimeter,
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
    // A relief of zero depth is NO relief, not a degenerate one: with the
    // inner chamfer still set, the polygon folds into a triangle that would
    // notch the pin itself.
    const relief = relD > 0 * meter && relW > 0 * meter;
    if (relief)
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
    if (relief)
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
 * A closed polygon sketched on plane(origin, normal, xDir) -- (u, v) along
 * xDir and normal x xDir -- and extruded `depth` along the normal.
 */
export function polyPrism(context is Context, id is Id, tag is string,
                          origin is Vector, normal is Vector, xDir is Vector,
                          pts is array, depth is ValueWithUnits)
{
    var sk = newSketchOnPlane(context, id + tag, {
            "sketchPlane" : plane(origin, normal, xDir) });
    skPolygon(sk, pts);
    skSolve(sk);
    opExtrude(context, id + (tag ~ "Ext"), {
            "entities"  : qSketchRegion(id + tag),
            "direction" : normal,
            "endBound"  : BoundingType.BLIND,
            "endDepth"  : depth });
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
    const capT  = opt.caseCapThickness;

    // FULL WRAP, the COVER (top half) only: its walls run the whole servo,
    // round the shaft end too, and only the CAP stays at the far end (the
    // horn needs the rest of the face). Printed cap-down, a wall past the cap
    // has nothing under it, so its edge nearest the cap face slopes at the
    // overhang limit from the cap's edge toward the shaft end, and carries on
    // round the corners and across the end wall at the same slope, meeting
    // mid-width. Where the base is not, the walls
    // run on down: to just above the cable connectors in their window, and to
    // the back face beyond it. From the hand-drawn case-side-wall /
    // case-end-wall in wing-linkage-shorter, 2026-09-23.
    //
    // NOT the base: a base wrapped the same way would need walls hanging
    // above its cap-down bed with nothing under them, and it cannot be
    // printed (the user, 2026-09-23). It keeps the far-end wrap.
    const full  = opt.fullWrap == true && top;
    const yNi   = -(t.shaftFromEnd + sc);          // the end wall's inner face
    const outer = top ? topOuter : botOuter;
    const yS    = full ? yNi - (outer - inner) : y0;
    const yC    = full ? yNi : y0 - over;

    if (top)
    {
        boxSolid(context, id, "shell", cs, topOuter,
                 yS, endY + sc + opt.caseTopWall,
                 full ? backZ : hornZ - skirt, seatZ + capT);
        boxSolid(context, id, "cav", cs, inner,
                 yC, endY + sc, (full ? backZ : hornZ - skirt) - over, seatZ);
    }
    else
    {
        boxSolid(context, id, "shell", cs, botOuter,
                 yS, endY + sc + opt.caseTopWall + opt.caseNestClearance
                     + opt.caseBottomWall,
                 seatZ - capT,
                 backZ + opt.caseGripLength + opt.caseNestLength);
        boxSolid(context, id, "cav", cs, inner,
                 yC, endY + sc, seatZ, backZ + opt.caseGripLength);
        boxSolid(context, id, "nest", cs, nestBore,
                 full ? yNi - opt.caseTopWall - opt.caseNestClearance : y0 - over,
                 endY + sc + opt.caseTopWall + opt.caseNestClearance,
                 backZ + opt.caseGripLength,
                 backZ + opt.caseGripLength + opt.caseNestLength + over);
    }

    var wrapCut = [];
    if (full)
    {
        const tanS = tan(90 * degree - t.caseWrapOverhang);
        const W    = outer + 1 * millimeter;
        const yA0  = cross(cs.zAxis, cs.xAxis);
        const zCap = top ? seatZ + capT : seatZ - capT;   // the bed, as printed
        const sgn  = top ? 1 : -1;                          // bed-ward along z
        const far  = zCap + sgn * 1 * millimeter;           // past the bed
        const e    = 1 * millimeter;
        // the cap, cut back to the far-end wrap
        boxSolid(context, id, "capCut", cs, W, yS - e, y0,
                 top ? seatZ : far, top ? far : seatZ);
        // the walls' bed-ward edge: through (y0, cap face) at the overhang slope
        const zFace = top ? hornZ : seatZ - capT;
        const zAt = function(y) { return zFace - sgn * (y0 - y) * tanS; };
        polyPrism(context, id, "slopeCut", cs.origin - W * cs.xAxis, cs.xAxis, yA0,
                  [vector(y0, zFace), vector(yS - e, zAt(yS - e)),
                   vector(yS - e, far), vector(y0, far)], 2 * W);
        // the end wall's edge: a cone about each INNER corner of the U, at
        // the same slope. Every layer then grows out of the one below it:
        // the shortest way round the corner from the side wall is a straight
        // line from that corner, so the edge rises at the slope along it.
        // Not a V from the OUTER corner, as first drawn: that started a whole
        // wall thickness too high, so the corner's first layer was a level
        // strip hanging off the side wall -- seen on the printer 2026-09-23.
        // Each cone is kept to its own half, and past the side wall's inner
        // face the side wall's own slope already rules: trimmed by
        // SUBTRACTION, cone as the target, so it keeps its identity for the
        // cutter query (an intersection left a stray, unnamed body behind,
        // 2026-09-23 -- a billed call).
        const zc = zAt(yNi);
        const dy = yNi - yS + e;
        const R  = sqrt(inner * inner + dy * dy) + e;
        const zR = zc - sgn * (R * tanS + e);
        const z0 = min(zR, far) - e;
        const z1 = max(zR, far) + e;
        for (var s in [1, -1])
        {
            const tag   = s > 0 ? "coneP" : "coneN";
            const pivot = cs.origin + s * inner * cs.xAxis + yNi * yA0;
            const rad   = -s * cs.xAxis;
            revolveProfile(context, id, tag, plane(pivot, cross(rad, cs.zAxis), rad),
                           line(pivot, cs.zAxis),
                           [vector(0 * millimeter, zc), vector(R, zc - sgn * R * tanS),
                            vector(R, far), vector(0 * millimeter, far)]);
            // [a, b] across (s x), [c, d] along y: the other half, past the
            // pivot, and beyond the end wall's inner face
            const trims = [[-R - e, 0 * millimeter, yNi - R - e, yNi + R + e],
                           [inner, inner + R + e, yNi - R - e, yNi + R + e],
                           [0 * millimeter, inner, yNi, yNi + R + e]];
            var tq = [];
            for (var k = 0; k < 3; k += 1)
            {
                const tr = trims[k];
                boxSolid(context, id, tag ~ "Trim" ~ k,
                         coordSystem(cs.origin + s * (tr[0] + tr[1]) / 2 * cs.xAxis, cs.xAxis, cs.zAxis),
                         (tr[1] - tr[0]) / 2, tr[2], tr[3], z0, z1);
                tq = append(tq, qCreatedBy(id + (tag ~ "Trim" ~ k ~ "Ext"), EntityType.BODY));
            }
            opBoolean(context, id + (tag ~ "Keep"), {
                    "tools"         : qUnion(tq),
                    "targets"       : qCreatedBy(id + (tag ~ "Rev"), EntityType.BODY),
                    "operationType" : BooleanOperationType.SUBTRACTION });
        }
        // the cable connectors' window, both sides, back face up
        boxSolid(context, id, "window", cs, W, t.caseWindowNear, y0,
                 backZ - fc - capT - e, backZ + t.caseWindowDepth);
        // and over the base, the cover stops where the base's nest takes it
        boxSolid(context, id, "baseZone", cs, W, y0, endY + sc + opt.caseTopWall + e,
                 backZ - e, hornZ - skirt);
        wrapCut = [qCreatedBy(id + "capCutExt", EntityType.BODY),
                   qCreatedBy(id + "slopeCutExt", EntityType.BODY),
                   qCreatedBy(id + "conePRev", EntityType.BODY),
                   qCreatedBy(id + "coneNRev", EntityType.BODY),
                   qCreatedBy(id + "windowExt", EntityType.BODY),
                   qCreatedBy(id + "baseZoneExt", EntityType.BODY)];
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
        "cutters" : qUnion(concatenateArrays([wrapCut, [qCreatedBy(id + "cavExt", EntityType.BODY),
                            qCreatedBy(id + "nestExt", EntityType.BODY),
                            qCreatedBy(id + "reliefRev", EntityType.BODY),
                            qCreatedBy(id + "reliefRing", EntityType.BODY)]]))
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

/**
 * The idler plug: two steps into the back-face recess and a collar outside.
 *
 * Off the SAME datum as the horn features -- the horn's outer face, +Z out of
 * the horn side -- so one mate connector drives both ends of the servo. The
 * back face is hornThickness + caseDepth down -Z, and the profile is revolved
 * with its axial coordinate turned round to point out of the BACK.
 */
export function idlerGeometry(context is Context, id is Id, cs is CoordSystem,
                              opt is map) returns Query
{
    const t = SERVO_MOUNT_TABLE[opt.servo];
    const idlerRo      = (t.idlerRecessOuterDia - opt.idlerPlugClearance) / 2;
    const idlerRi      = (t.idlerRecessInnerDia - opt.idlerPlugClearance) / 2;
    const idlerDo      = t.idlerRecessOuterDepth;
    const idlerDi      = t.idlerRecessInnerDepth;
    const idlerEndGap  = opt.idlerEndGap;
    const idlerFaceGap = opt.idlerFaceGap;
    const idlerRc      = opt.idlerCollarDia / 2;
    const idlerT       = opt.idlerCollarThickness;

    const out   = -cs.zAxis;
    const o     = cs.origin - (t.hornThickness + t.caseDepth) * cs.zAxis;
    const yA    = cross(out, cs.xAxis);
    revolveProfile(context, id, "idler", plane(o, -yA, cs.xAxis), line(o, out),
        [vector(0 * millimeter, -idlerDo - idlerDi + idlerEndGap),
            vector(idlerRi, -idlerDo - idlerDi + idlerEndGap),
            vector(idlerRi, -idlerDo + idlerEndGap),
            vector(idlerRo, -idlerDo + idlerEndGap),
            vector(idlerRo, idlerFaceGap),
            vector(idlerRc, idlerFaceGap),
            vector(idlerRc, idlerFaceGap + idlerT),
            vector(0 * millimeter, idlerFaceGap + idlerT)]);
    return qCreatedBy(id + "idlerRev", EntityType.BODY);
}

/** The plug, merged into `target` when one is picked. */
export function idlerBuild(context is Context, id is Id, cs is CoordSystem,
                           opt is map, target is Query) returns Query
{
    const plug = idlerGeometry(context, id + "geom", cs, opt);
    if (isQueryEmpty(context, target))
        return plug;
    opBoolean(context, id + "add", {
            "tools"         : qUnion([plug, target]),
            "operationType" : BooleanOperationType.UNION });
    return target;
}

/**
 * A flat-head screw and a captive nut: countersink, clearance hole, and a
 * slot the nut slides into sideways.
 *
 * `cs` is on the surface the head sits in with +Z INTO the part along the
 * shank (the dialog flips a picked mate connector, whose Z points out of its
 * face). The slot runs out along +X by nutSlotLength; its closed end is the
 * nut's circumradius behind the axis, so the hex corners are not clipped --
 * or, with bothWays, the slot runs nutSlotLength out along -X as well.
 * Optional opt.headPocket extends the head's bore outward past the surface.
 *
 * Returns the cutter bodies; booleans nothing.
 */
export function screwJointGeometry(context is Context, id is Id,
                                   cs is CoordSystem, opt is map) returns Query
{
    const headR     = opt.headDia / 2;
    const holeR     = opt.holeDia / 2;
    const holeDepth = opt.holeDepth;
    const cskDepth  = (headR - holeR) / tan(opt.cskAngle / 2);
    // headPocket, when given, runs the head's bore on OUTWARD past the
    // surface: a countersink at the bottom of a pocket, for a part thicker
    // than the screw can reach through. The dialog does not offer it; the
    // fixture generator does.
    const over      = 1 * millimeter
                      + (opt.headPocket == undefined ? 0 * millimeter : opt.headPocket);
    const yA        = cross(cs.zAxis, cs.xAxis);

    revolveProfile(context, id, "screw", plane(cs.origin, -yA, cs.xAxis),
        line(cs.origin, cs.zAxis), [vector(0 * millimeter, -over),
            vector(headR, -over),
            vector(headR, 0 * millimeter),
            vector(holeR, cskDepth),
            vector(holeR, holeDepth),
            vector(0 * millimeter, holeDepth)]);

    // boxSolid wants the slot's WIDTH on its x, so turn the frame a quarter:
    // x' = the datum's Y, y' = -X. Out along +X is then y' negative.
    const w      = opt.nutSlotWidth;
    const back   = opt.bothWays ? opt.nutSlotLength : w / sqrt(3);
    const slotCs = coordSystem(cs.origin, yA, cs.zAxis);
    boxSolid(context, id, "slot", slotCs, w / 2, -opt.nutSlotLength, back,
             opt.nutDepth, opt.nutDepth + opt.nutSlotThickness);
    return qUnion([qCreatedBy(id + "screwRev", EntityType.BODY),
                   qCreatedBy(id + "slotExt", EntityType.BODY)]);
}


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

/** A radial bar at angle `a` about the Z axis: r0..r1, half-width hw, z0..z1. */
export function radialBar(context is Context, id is Id, tag is string, a is ValueWithUnits,
                          r0 is ValueWithUnits, r1 is ValueWithUnits, hw is ValueWithUnits,
                          z0 is ValueWithUnits, z1 is ValueWithUnits) returns Query
{
    const d = vector(cos(a), sin(a));
    const n = vector(-sin(a), cos(a));
    polyPrism(context, id, tag, vector(0 * millimeter, 0 * millimeter, z0), vector(0, 0, 1), vector(1, 0, 0),
              [r0 * d - hw * n, r1 * d - hw * n, r1 * d + hw * n, r0 * d + hw * n], z1 - z0);
    return qCreatedBy(id + (tag ~ "Ext"), EntityType.BODY);
}

/** A double-D about the Z axis: radius r, flats at +-flat along X, z0..z1. One body. */
export function doubleD(context is Context, id is Id, r is ValueWithUnits, flat is ValueWithUnits,
                        z0 is ValueWithUnits, z1 is ValueWithUnits) returns Query
{
    const mm = millimeter;
    const c = cylW(context, id + "cyl", vector(0 * mm, 0 * mm, z0), vector(0 * mm, 0 * mm, z1), r);
    var cut = [];
    for (var sg in [1, -1])
        cut = append(cut, boxW(context, id + (sg > 0 ? "fP" : "fN"),
                vector(sg > 0 ? flat : -r - 1 * mm, -r - 1 * mm, z0 - 1 * mm),
                vector(sg > 0 ? r + 1 * mm : -flat, r + 1 * mm, z1 + 1 * mm)));
    opBoolean(context, id + "flats", { "tools" : qUnion(cut), "targets" : c,
            "operationType" : BooleanOperationType.SUBTRACTION });
    return c;
}

/**
 * Case pins in the servo's OTHER hole row (the one caseShellGeometry does not
 * use, 2 x casePinRowOffset toward the shaft end), with their root reliefs,
 * onto `target`. The shared shell geometry built one row over, everything but
 * its pins and reliefs thrown away -- so these are the far row's pins exactly.
 */
export function nearRowPins(context is Context, id is Id, cs is CoordSystem, part is string,
                            target is Query)
{
    const t  = SERVO_MOUNT_TABLE["XC330"];
    const yA = cross(cs.zAxis, cs.xAxis);
    const sh = coordSystem(cs.origin - 2 * t.casePinRowOffset * yA, cs.xAxis, cs.zAxis);
    const g  = caseShellGeometry(context, id + "g", sh, mergeMaps(CASE_OPT, { "part" : part }));
    const rel = qUnion([qCreatedBy(id + "g" + "reliefRev", EntityType.BODY),
                        qCreatedBy(id + "g" + "reliefRing", EntityType.BODY)]);
    opDeleteBodies(context, id + "junk", { "entities" : qSubtraction(qCreatedBy(id + "g", EntityType.BODY),
                                                                   qUnion([g.pins, rel])) });
    opBoolean(context, id + "cut", { "tools" : rel, "targets" : target,
            "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "add", { "tools" : qUnion([g.pins, target]),
            "operationType" : BooleanOperationType.UNION });
}

/**
 * A 6-32 joint with NO ridge: screwJoint's bore, nut slot and teardrops,
 * for a joint something else already locates (the fork's ledge and cheeks).
 */
export function plainScrewJoint(context is Context, id is Id, head is Vector,
                                zIn is Vector, slotDir is Vector, pocket is ValueWithUnits,
                                cskPart is Query, nutPart is Query, cskUp is Vector, nutUp is Vector)
{
    const mm = millimeter;
    const cs = coordSystem(head, slotDir, zIn);
    screwJointGeometry(context, id + "scr", cs, mergeMaps(SCREW_OPT, {
            "nutDepth" : FIXTURE.joint_plate + 2 * mm, "headPocket" : pocket, "bothWays" : true,
            "nutSlotLength" : 200 * mm }));
    opBoolean(context, id + "slotCut", { "tools" : qCreatedBy(id + "scr" + "slotExt", EntityType.BODY),
            "targets" : nutPart, "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "cut", { "tools" : qCreatedBy(id + "scr" + "screwRev", EntityType.BODY),
            "targets" : qUnion([cskPart, nutPart]), "operationType" : BooleanOperationType.SUBTRACTION });
    const rB = SCREW_OPT.holeDia / 2;
    for (var pu in [[cskPart, cskUp, "tdC"], [nutPart, nutUp, "tdN"]])
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
                "targets" : pu[0], "operationType" : BooleanOperationType.SUBTRACTION });
    }
}

/**
 * Every part and the hardware. `opt.attach` SCREWS or PINS (the hub on the
 * horn), `opt.key` CROSS or FLAT (the upper's lugs). Returns the stages (what
 * is fixed, what turns with the steer), each printed part with the direction
 * that prints UP, and the steering axis.
 */
export function steeringBuild(context is Context, id is Id, opt is map) returns map
{
    const L  = ST;
    const mm = millimeter;
    const O  = vector(0, 0, 0) * meter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const P  = id + "parts";
    const H  = id + "hw";
    const pins = opt.attach == "PINS";
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };
    const at = function(x, y, z) returns Vector { return vector(x, y, z); };
    const holeR = SCREW_OPT.holeDia / 2;
    const T = SERVO_MOUNT_TABLE["XC330"];

    // ---- the servo: horn face at zH facing -Z; its local y = cross(-Z, X) = -Y.
    // The envelope has holes in the far row only; the near row's are added
    // here, so the near-row pins are tested against real holes.
    const cs = coordSystem(at(0 * mm, 0 * mm, L.zH), X, -Z);
    const sv = x330Envelope(context, H + "servo", cs);
    {
        const zc = -X330.hornThickness;
        const zb = zc - X330.caseDepth;
        const rowY = X330.caseHeight - X330.shaftFromEnd - X330.caseHeight / 2 - T.casePinRowOffset;
        var nh = [];
        for (var sx in [1, -1])
        {
            const x = sx * T.caseHoleSpanX / 2;
            nh = append(nh, cylIn(context, H + ("nhf" ~ (sx > 0 ? "p" : "n")), cs,
                    vector(x, rowY, zc - T.caseReliefDepthHorn), vector(x, rowY, zc + 1 * mm), T.caseReliefDia / 2));
            nh = append(nh, cylIn(context, H + ("nhb" ~ (sx > 0 ? "p" : "n")), cs,
                    vector(x, rowY, zb - 1 * mm), vector(x, rowY, zb + T.caseReliefDepthBack), T.caseReliefDia / 2));
        }
        opBoolean(context, H + "nearHoles", { "tools" : qUnion(nh), "targets" : sv.caseQ,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }

    // ---- mock wheel: tire envelope, web + hubs, axle (profiles in (x, z), about X)
    step("wheel");
    const xz = plane(O, -Y, X);
    const tw = L.tireW / 2;
    const tf = L.tireFlat / 2;
    const sh = tw - tf;
    revolveProfile(context, H, "tire", xz, line(O, X),
            [vector(-tw, L.tireMin / 2), vector(tw, L.tireMin / 2), vector(tw, L.R - sh),
             vector(tf, L.R), vector(-tf, L.R), vector(-tw, L.R - sh)]);
    const tire = qCreatedBy(H + "tireRev", EntityType.BODY);
    const web = cylW(context, H + "web", at(-L.webW / 2, 0 * mm, 0 * mm), at(L.webW / 2, 0 * mm, 0 * mm), L.tireMin / 2);
    const hA = cylW(context, H + "hubA", at(0 * mm, 0 * mm, 0 * mm), at(L.hubW / 2, 0 * mm, 0 * mm), L.hubDiaA / 2);
    const hB = cylW(context, H + "hubB", at(-L.hubW / 2, 0 * mm, 0 * mm), at(0 * mm, 0 * mm, 0 * mm), L.hubDiaB / 2);
    unite(context, H + "wheelU", [web, hA, hB]);
    const wheelPt = at(0 * mm, 0 * mm, L.tireMin / 2 - 2 * mm);
    opBoolean(context, H + "wheelBore", { "tools" : cylW(context, H + "bore", at(-L.xOut, 0 * mm, 0 * mm),
            at(L.xOut, 0 * mm, 0 * mm), L.axleDia / 2), "targets" : partAt(context, H, wheelPt),
            "operationType" : BooleanOperationType.SUBTRACTION });
    const shank = cylW(context, H + "shank", at(-L.xOut, 0 * mm, 0 * mm), at(L.xOut, 0 * mm, 0 * mm), L.axleDia / 2);
    const head = cylW(context, H + "head", at(L.xOut, 0 * mm, 0 * mm), at(L.xOut + L.headH, 0 * mm, 0 * mm), L.headDia / 2);
    unite(context, H + "axleU", [shank, head]);
    const axlePt = at(0 * mm, 0 * mm, 0 * mm);

    // ---- fork halves: the leg beside the tire, then, above it, a plate
    // stepped OUT to the block's width -- the step is the ledge the block
    // sits on -- with cheeks round the block's +-Y faces. The boss in to the
    // hub's insert.
    step("fork");
    var forkPt = {};
    for (var sg in [1, -1])
    {
        const tag = sg > 0 ? "fR" : "fL";
        const zf = L.blockBot - (L.forkHalf - L.legHalf);
        polyPrism(context, P, tag, at(sg * L.xIn, 0 * mm, 0 * mm), sg * X, sg * Y,
                  [vector(-L.legHalf, 0 * mm), vector(L.legHalf, 0 * mm), vector(L.legHalf, zf),
                   vector(L.forkHalf, L.blockBot), vector(-L.forkHalf, L.blockBot),
                   vector(-L.legHalf, zf)], L.xOut - L.xIn);
        // plate and cheeks start AT the leg's top, so nothing stands proud of the flare
        var plateParts = [boxW(context, P + (tag ~ "Plate"), at(sg > 0 ? L.xBlk : -L.xOut, -L.forkHalf, L.blockBot),
                               at(sg > 0 ? L.xOut : -L.xBlk, L.forkHalf, L.blockTop))];
        for (var sy in [1, -1])
            plateParts = append(plateParts, boxW(context, P + (tag ~ "Cheek" ~ (sy > 0 ? "P" : "N")),
                    at(sg > 0 ? L.xIn : -L.xBlk - 1 * mm, sy > 0 ? L.cheekIn : -L.forkHalf, L.blockBot),
                    at(sg > 0 ? L.xBlk + 1 * mm : -L.xIn, sy > 0 ? L.forkHalf : -L.cheekIn, L.blockTop)));
        const plate = qUnion(plateParts);
        const endR = cylW(context, P + (tag ~ "End"), at(sg * L.xIn, 0 * mm, 0 * mm), at(sg * L.xOut, 0 * mm, 0 * mm), L.legHalf);
        // the boss: bossDia on the hub's face, flared back to the leg's width
        const r0 = L.bossDia / 2;
        const xc = L.xBoss + (L.legHalf - r0) / tan(L.bossChamferDeg);
        revolveProfile(context, P, tag ~ "Boss", xz, line(O, X),
                [vector(sg * L.xBoss, 0 * mm), vector(sg * L.xBoss, r0), vector(sg * xc, L.legHalf),
                 vector(sg * (L.xIn + 1 * mm), L.legHalf), vector(sg * (L.xIn + 1 * mm), 0 * mm)]);
        const boss = qCreatedBy(P + (tag ~ "BossRev"), EntityType.BODY);
        unite(context, P + (tag ~ "U"), [qCreatedBy(P + (tag ~ "Ext"), EntityType.BODY), plate, endR, boss]);
        forkPt[tag] = at(sg * (L.xOut - 1 * mm), 0 * mm, L.blockBot / 2);
        // one hole size both sides: the knurl leg (+X) and the other are one part
        const hole = cylW(context, P + (tag ~ "Hole"), at(sg * (L.xBoss - 1 * mm), 0 * mm, 0 * mm),
                          at(sg * (L.xOut + 1 * mm), 0 * mm, 0 * mm), L.axleHole / 2);
        opBoolean(context, P + (tag ~ "Cut"), { "tools" : hole,
                "targets" : partAt(context, P, forkPt[tag]),
                "operationType" : BooleanOperationType.SUBTRACTION });
    }

    // ---- headset lower: block on the ledges, thrust boss, shaft, double-D
    // spigot; the central screw's long bore from below, its 45 deg seat, hole
    step("headset lower");
    const block = boxW(context, P + "block", at(-L.xBlk, -L.blockHalf, L.blockBot), at(L.xBlk, L.blockHalf, L.blockTop));
    const shaft = cylW(context, P + "shaft", at(0 * mm, 0 * mm, L.blockTop - 1 * mm), at(0 * mm, 0 * mm, L.zS), L.shaftR);
    const spig = doubleD(context, P + "spigot", L.spR, L.spFlat, L.zS - 1 * mm, L.zS + L.spigotH);
    // the thrust boss: the front's weight bears up through this annulus only
    const thrust = cylW(context, P + "thrust", at(0 * mm, 0 * mm, L.blockTop - 1 * mm), at(0 * mm, 0 * mm, L.caseBot), L.thrustR);
    unite(context, P + "hlU", [block, shaft, spig, thrust]);
    const hlPt = at(0 * mm, -L.blockHalf + 1 * mm, L.blockBot + 1 * mm);
    revolveProfile(context, P, "hlSeat", xz, line(O, Z),
            [vector(0 * mm, L.cbTop - 1 * mm), vector(L.cbR, L.cbTop - 1 * mm), vector(L.cbR, L.cbTop),
             vector(holeR, L.cskTop), vector(0 * mm, L.cskTop)]);
    opBoolean(context, P + "hlCut", { "tools" : qUnion([
            cylW(context, P + "hlBore", at(0 * mm, 0 * mm, L.blockBot - 1 * mm), at(0 * mm, 0 * mm, L.cbTop), L.cbR),
            qCreatedBy(P + "hlSeatRev", EntityType.BODY),
            cylW(context, P + "hlHole", at(0 * mm, 0 * mm, L.cskTop - 0.1 * mm), at(0 * mm, 0 * mm, L.zS + L.spigotH + 1 * mm), holeR)]),
            "targets" : partAt(context, P, hlPt), "operationType" : BooleanOperationType.SUBTRACTION });

    // ---- headset upper: body, lugs out to the rim; the double-D socket, its
    // ceiling bridged in two steps round the hole, the hole, the nut's hex
    // pocket from the top
    step("headset upper");
    const upBody = cylW(context, P + "upBody", at(0 * mm, 0 * mm, L.zS), at(0 * mm, 0 * mm, L.upTop), L.upR);
    var upLugs = [upBody];
    const nLug = opt.key == "FLAT" ? 2 : 4;
    for (var k = 0; k < nLug; k += 1)
        upLugs = append(upLugs, radialBar(context, P, "lug" ~ k, (45 + k * 360 / nLug) * degree,
                L.lugR0, L.lugR1, L.lugHalf, L.upTop - 0.5 * mm, L.lugTop));
    unite(context, P + "upU", upLugs);
    const upPt = at((L.upR + L.sockSpR) / 2, 0 * mm, L.zS + 0.5 * mm);
    const sock = doubleD(context, P + "upSock", L.sockSpR, L.sockSpFlat, L.zS - 1 * mm, L.upSockTop);
    // layer 1 over the socket: a hole-wide strip across its short (flat to
    // flat) span; layer 2: the hole's square. Each layer bridges wall to wall.
    const bl = L.bridgeLayer;
    const br1 = boxW(context, P + "upBr1", at(-L.sockSpFlat, -holeR, L.upSockTop - 0.1 * mm), at(L.sockSpFlat, holeR, L.upSockTop + bl));
    const br2 = boxW(context, P + "upBr2", at(-holeR, -holeR, L.upSockTop + bl - 0.1 * mm), at(holeR, holeR, L.upSockTop + 2 * bl));
    const upHole = cylW(context, P + "upHole", at(0 * mm, 0 * mm, L.upSockTop), at(0 * mm, 0 * mm, L.upTop + 1 * mm), holeR);
    var hex = [];
    for (var k = 0; k < 6; k += 1)
        hex = append(hex, L.nutR * vector(cos(k * 60 * degree), sin(k * 60 * degree)));
    polyPrism(context, P, "upNut", at(0 * mm, 0 * mm, L.nutFloor), Z, X, hex, L.upTop - L.nutFloor + 1 * mm);
    opBoolean(context, P + "upCut", { "tools" : qUnion([sock, br1, br2, upHole,
            qCreatedBy(P + "upNutExt", EntityType.BODY)]),
            "targets" : partAt(context, P, upPt), "operationType" : BooleanOperationType.SUBTRACTION });

    // ---- horn hub: disc (a well round the horn when PINS), the cross socket
    // from a centre island out through the rim, the M2s between its arms
    step("horn hub");
    cylW(context, P + "hubDisc", at(0 * mm, 0 * mm, L.hubBot), at(0 * mm, 0 * mm, L.zH + (pins ? L.wellT : 0 * mm)), L.hubR);
    const hubPt = at(0 * mm, 0 * mm, L.hubBot + 0.5 * mm);
    if (pins)
        servoMountBuild(context, P + "hubHorn", cs, HORN_OPT, partAt(context, P, hubPt));
    var hubCut = [];
    for (var k = 0; k < 4; k += 1)
    {
        hubCut = append(hubCut, radialBar(context, P, "sock" ~ k, (45 + k * 90) * degree, L.sockR0, L.hubR + 1 * mm,
                                          L.sockHalf, L.hubBot - 1 * mm, L.socketFloor));
        if (!pins)
        {
            // the horn's M2 holes lie on the X and Y axes (x330Envelope's ring)
            const b = (k * 90) * degree;
            const p = T.hornBoltCircle / 2 * vector(cos(b), sin(b), 0);
            hubCut = append(hubCut, cylW(context, P + ("m2" ~ k), at(p[0], p[1], L.hubBot - 1 * mm),
                    at(p[0], p[1], L.zH + 1 * mm), L.m2Hole / 2));
            hubCut = append(hubCut, cylW(context, P + ("m2cb" ~ k), at(p[0], p[1], L.hubBot - 1 * mm),
                    at(p[0], p[1], L.hubBot + L.hubCbDepth), L.hubCbR));
        }
    }
    opBoolean(context, P + "hubCut", { "tools" : qUnion(hubCut), "targets" : partAt(context, P, hubPt),
            "operationType" : BooleanOperationType.SUBTRACTION });

    // ---- lower case: cup + walls, flat underside on the block's boss; servo
    // pocket, turning cavity, bushing; the X330 horn-half shell (cap, far-row
    // pins) onto it, and near-row pins where they clear the cavity
    step("lower case");
    const cup = boxW(context, P + "cup", at(-L.cupHalf, L.yP, L.caseBot), at(L.cupHalf, L.cupNear, L.zCF));
    const walls = boxW(context, P + "walls", at(-L.topOuter, L.yWallFar, L.zCF - 1 * mm), at(L.topOuter, L.yWallNear, L.wallTop));
    unite(context, P + "lcU", [cup, walls]);
    const lcPt = at(L.cupHalf - 0.5 * mm, L.yP + 1 * mm, L.caseBot + 1 * mm);
    opBoolean(context, P + "lcCut", { "tools" : qUnion([
            boxW(context, P + "lcPocket", at(-L.inner, L.yServoFar, L.zCF), at(L.inner, L.yServoNear, L.zBack + 1 * mm)),
            cylW(context, P + "lcCav", at(0 * mm, 0 * mm, L.sleeveTop), at(0 * mm, 0 * mm, L.zCF + 0.5 * mm), L.cavR),
            cylW(context, P + "lcBore", at(0 * mm, 0 * mm, L.caseBot - 1 * mm), at(0 * mm, 0 * mm, L.sleeveTop + 1 * mm), L.boreR)]),
            "targets" : partAt(context, P, lcPt), "operationType" : BooleanOperationType.SUBTRACTION });
    caseShellBuild(context, P + "lcShell", cs, mergeMaps(CASE_OPT, { "part" : "TOP" }), partAt(context, P, lcPt));
    if (L.lowerNearPins > 0.5 * mm)
        nearRowPins(context, P + "lcNear", cs, "TOP", partAt(context, P, lcPt));

    // ---- upper case: the X330 back-half shell, then the cap and the walls
    // carried round the whole servo, nesting over the lower's; the side walls
    // cleared over the connectors' window down to the lower case, the edge
    // left a gable in the nest shell; slots through the cap to plug the
    // connectors (an "H" left); near-row pins in the back
    step("upper case");
    caseShellBuild(context, P + "ucShell", cs, mergeMaps(CASE_OPT, { "part" : "BOTTOM" }), qNothing());
    const ucPt = at(L.botOuter - 0.5 * mm, (L.yWallFar + L.yWrap) / 2, L.upTopZ - 0.5 * mm);
    const yJ = L.yWrap - 1 * mm;       // a millimetre into the shell's own far wrap
    const ucWrap = boxW(context, P + "ucWrap", at(-L.botOuter, yJ, L.upBot), at(L.botOuter, L.yUpNear, L.upTopZ));
    // the window, in (y, z), through both side walls along X; its far edge
    // (as printed) a gable from each end up to the apex mid-span
    polyPrism(context, P, "ucWin", at(-L.botOuter - 1 * mm, 0 * mm, 0 * mm), X, Y,
              [vector(L.winFar, L.zBack), vector(L.winFar, L.winBot), vector((L.winFar + L.winNear) / 2, L.winApex),
               vector(L.winNear, L.winBot), vector(L.winNear, L.zBack)], 2 * L.botOuter + 2 * mm);
    const ucPad = boxW(context, P + "ucPad", at(-L.padHalf, L.yP, L.padBot), at(L.padHalf, L.yUpFar + 1 * mm, L.upTopZ));
    unite(context, P + "ucU", [partAt(context, P, ucPt), ucWrap, ucPad]);
    opBoolean(context, P + "ucCut", { "tools" : qUnion([
            boxW(context, P + "ucNest", at(-L.nestBore, yJ - 0.1 * mm, L.upBot - 1 * mm), at(L.nestBore, L.yNestNear, L.wallTop)),
            boxW(context, P + "ucCav", at(-L.inner, yJ - 0.1 * mm, L.wallTop - 1 * mm), at(L.inner, L.yServoNear, L.zBack)),
            qCreatedBy(P + "ucWinExt", EntityType.BODY),
            boxW(context, P + "ucSlotP", at(L.slotIn, L.winFar, L.zBack - 0.1 * mm), at(L.botOuter + 1 * mm, L.winNear, L.upTopZ + 1 * mm)),
            boxW(context, P + "ucSlotN", at(-L.botOuter - 1 * mm, L.winFar, L.zBack - 0.1 * mm), at(-L.slotIn, L.winNear, L.upTopZ + 1 * mm))]),
            "targets" : partAt(context, P, ucPt), "operationType" : BooleanOperationType.SUBTRACTION });
    nearRowPins(context, P + "ucNear", cs, "BOTTOM", partAt(context, P, ucPt));

    // ---- mount plate (placeholder), and the clamp tab off its bottom edge,
    // back, in the plane that is level at the bike's rake: its underside
    // through the plate's back-bottom edge, normal nT = world up
    step("mount plate");
    const plate = boxW(context, P + "plate", at(-L.botOuter, L.plateBack, L.caseBot), at(L.botOuter, L.yP, L.upTopZ));
    const plPt = at(L.botOuter - 1 * mm, (L.plateBack + L.yP) / 2, L.caseBot + 1 * mm);
    const nT = vector(0, sin(L.rakeDeg), cos(L.rakeDeg));
    const dT = vector(0, -cos(L.rakeDeg), sin(L.rakeDeg));      // along the tab, away from the plate
    const t0 = at(0 * mm, L.plateBack, L.caseBot);
    // sketch on the +X side face, normal -X: (u, v) = (dT, cross(-X, dT) = nT)
    polyPrism(context, P, "tab", t0 + L.tabW / 2 * X, -X, dT,
              [vector(0 * mm, 0 * mm), vector(L.tabL, 0 * mm), vector(L.tabL, L.tabT), vector(0 * mm, L.tabT)], L.tabW);
    // a FOOT: the plate's bottom widened to the tab's width, so the tab's
    // root stands on plate all the way across (printed -Y up, a tab wider
    // than the plate hung its root 3.3 mm off the bed)
    const foot = boxW(context, P + "plateFoot", at(-L.tabW / 2, L.plateBack, L.caseBot),
                      at(L.tabW / 2, L.yP, L.caseBot + L.tabT / cos(L.rakeDeg) + 1 * mm));
    unite(context, P + "plU", [plate, foot, qCreatedBy(P + "tabExt", EntityType.BODY)]);
    // four 10-32 through it, teardropped toward -Y (up, as the plate prints)
    const upT = normalize(-Y - dot(-Y, nT) * nT);
    const rT = L.tabHole / 2;
    var tabCut = [];
    for (var i = 0; i < 4; i += 1)
    {
        const c = t0 + ((i < 2) ? 1 : -1) * L.tabSpanX / 2 * X
                     + (L.tabL / 2 + ((i % 2 == 0) ? 1 : -1) * L.tabSpanD / 2) * dT - 1 * mm * nT;
        tabCut = append(tabCut, cylW(context, P + ("tabHole" ~ i), c, c + (L.tabT + 2 * mm) * nT, rT));
        polyPrism(context, P, "tabTd" ~ i, c, nT, cross(upT, nT),
                  [vector(0 * mm, 0 * mm), vector(rT / sqrt(2), rT / sqrt(2)),
                   vector(0 * mm, rT * sqrt(2)), vector(-rT / sqrt(2), rT / sqrt(2))], L.tabT + 2 * mm);
        tabCut = append(tabCut, qCreatedBy(P + ("tabTd" ~ i ~ "Ext"), EntityType.BODY));
    }
    opBoolean(context, P + "tabCut", { "tools" : qUnion(tabCut), "targets" : partAt(context, P, plPt),
            "operationType" : BooleanOperationType.SUBTRACTION });

    // ---- the screw joints. Fork -> block: one a side, head in the fork's
    // outer face, no ridge -- the ledge takes the vertical load, the cheeks
    // locate Y. Cases -> plate: heads in the plate's back, ridges on the cases.
    step("joints");
    const jp = FIXTURE.joint_plate;
    for (var sg in [1, -1])
    {
        const tag = sg > 0 ? "fR" : "fL";
        plainScrewJoint(context, P + ("j" ~ tag), at(sg * (L.xBlk + jp), 0 * mm, L.zFS),
                        -sg * X, Z, L.forkPocket, partAt(context, P, forkPt[tag]), partAt(context, P, hlPt),
                        -sg * X, Z);
    }
    screwJoint(context, P + "jLC", at(0 * mm, L.plateBack, L.zLS), Y, Z, 0 * mm,
               partAt(context, P, plPt), partAt(context, P, lcPt), true, X, L.ridgeHalf, Y, Z);
    screwJoint(context, P + "jUC", at(0 * mm, L.plateBack, L.zUS), Y, Z, 0 * mm,
               partAt(context, P, plPt), partAt(context, P, ucPt), true, X, L.padHalf - 1 * mm, Y, -Z);

    // ---- names, colours, print orientation
    const parts = [["fork R", forkPt["fR"], "X-", -X, "steer"],
                   ["fork L", forkPt["fL"], "X+", X, "steer"],
                   ["headset lower", hlPt, "Z+", Z, "steer"],
                   ["headset upper", upPt, "Z+", Z, "steer"],
                   ["horn hub", hubPt, pins ? "Z+" : "Z-", pins ? Z : -Z, "steer"],
                   ["lower case", lcPt, "Z+", Z, "static"],
                   ["upper case", ucPt, "Z-", -Z, "static"],
                   ["mount plate", plPt, "Y-", -Y, "static"]];
    const turnC  = color(0.93, 0.56, 0.20);
    const fixedC = color(0.62, 0.64, 0.68);
    var stages = { "static" : [sv.caseQ], "steer" : [sv.horn] };
    var prints = [];
    for (var n in parts)
    {
        const q = partAt(context, P, n[1]);
        dress(context, q, n[0] ~ " [print " ~ n[2] ~ "]", n[4] == "steer" ? turnC : fixedC,
              "Print with the " ~ n[2] ~ " side facing UP (module frame).");
        stages[n[4]] = append(stages[n[4]], q);
        prints = append(prints, [n[0], q, n[3]]);
    }
    const hw = [[tire, "tire", color(0.10, 0.10, 0.11)],
                [partAt(context, H, wheelPt), "wheel", color(0.85, 0.85, 0.80)],
                [partAt(context, H, axlePt), "axle", color(0.55, 0.57, 0.60)]];
    for (var h in hw)
    {
        dress(context, h[0], h[1], h[2], "Mock front wheel: an envelope, not a print.");
        stages["steer"] = append(stages["steer"], h[0]);
    }
    dress(context, sv.caseQ, "X330 case", color(0.16, 0.16, 0.18), "");
    dress(context, sv.horn, "X330 horn", color(0.3, 0.3, 0.32), "");
    opMateConnector(context, id + "axleMC", { "coordSystem" : coordSystem(O, X, Z), "owner" : partAt(context, P, hlPt) });
    return { "stages" : stages, "prints" : prints, "axis" : line(O, Z) };
}

// ==== UI LAYER BELOW -- dropped by --check ====

export enum SteerHubAttach
{
    annotation { "Name" : "M2 x 6 self-tappers into the horn (hub prints Z-)" }
    SCREWS,
    annotation { "Name" : "Horn pins, for a test fit (hub prints Z+)" }
    PINS
}

export enum SteerKey
{
    annotation { "Name" : "Cross: four lugs" }
    CROSS,
    annotation { "Name" : "Flat key: two lugs in one arm of the cross" }
    FLAT
}

annotation { "Feature Type Name" : "AOW steering",
             "Feature Type Description" : "Front steer module: mock wheel, fork, headset, XC330 cases and horn hub; Z up the steering axis" }
export const aowSteering = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Hub on the horn" }
        definition.attach is SteerHubAttach;

        annotation { "Name" : "Headset upper's key" }
        definition.key is SteerKey;

        annotation { "Name" : "Steer angle" }
        isAngle(definition.steer, { (degree) : [-180, 0, 180] } as AngleBoundSpec);
    }
    {
        const r = steeringBuild(context, id + "build", {
                "attach" : definition.attach == SteerHubAttach.PINS ? "PINS" : "SCREWS",
                "key" : definition.key == SteerKey.FLAT ? "FLAT" : "CROSS" });
        if (definition.steer != 0 * degree)
            opTransform(context, id + "steer", {
                    "bodies" : qUnion(r.stages["steer"]),
                    "transform" : rotationAround(r.axis, definition.steer) });
        reportFeatureInfo(context, id, "Horn face " ~ toString(roundToPrecision(ST.zH / millimeter, 2))
                ~ " mm up the axis from the axle; bushing " ~ toString(roundToPrecision(ST.sleeve / millimeter, 2)) ~ " mm");
    });
