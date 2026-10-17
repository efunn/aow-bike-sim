FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_bike_v2 --push bike_v2_features
 *
 * The drive, steering-v2 and righting modules' geometry layers, merged (each
 * module's own studio is generated from the same config), placed in the bike
 * frame, and the electronics as envelopes. No chassis. Numbers from
 * config/bike_v2_cad.yaml through aow_sim.cad_bike_v2's layout(), carried
 * here as BK2. Millimetres.
 *
 * BIKE FRAME: origin on the rear axle's centre, +Y forward, +X right, +Z up.
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
    "forkSlotX" : 12.8 * millimeter,
    "thrustR" : 9 * millimeter,
    "nutR" : 3.72391 * millimeter,
    "lugR0" : 4.72391 * millimeter,
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
    "v2HeadZ" : 108.55 * millimeter,
    "v2RrY" : 17.8 * millimeter,
    "v2RrHalfX" : 5.25 * millimeter,
    "v2RrOut" : 23.05 * millimeter,
    "v2LugTop" : 118.05 * millimeter,
    "v2RrPocket" : 10 * millimeter,
    "v2BlkTop" : 98.25 * millimeter,
    "v2BlkBot" : 87.05 * millimeter,
    "v2SideZ" : 75.55 * millimeter,
    "v2SideRidge" : 6 * millimeter,
    "v2SideRidgeTop" : 79.05 * millimeter,
    "v2SideX" : 16 * millimeter,
    "v2SideBoreWall" : 1.5 * millimeter,
    "v2SideNutWall" : 1.8 * millimeter,
    "v2PadTop" : 80.8 * millimeter,
    "v2Foot" : -12.55 * millimeter,
    "v2Rise" : 85.25 * millimeter,
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

export const STEER_V2 = true;

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
    "nutSlotWidth" : 6.45 * millimeter,
    "nutSlotThickness" : 2.7 * millimeter,
    "nutDepth" : 6.6 * millimeter,
    "nutSlotLength" : 12 * millimeter,
    "cskAngle" : 90 * degree
};

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
    const wellT        = hornT - opt.caseOffset;
    const collarR      = opt.collarOuterDia / 2;
    const roof         = opt.collarRoof;
    const over         = 1 * millimeter;
    const yAxis = cross(cs.zAxis, cs.xAxis);
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
    opBoolean(context, id + "add", {
            "tools"         : qUnion([g.pins, into]),
            "operationType" : BooleanOperationType.UNION });
    return into;
}

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

export function caseShellGeometry(context is Context, id is Id, cs is CoordSystem,
                                  opt is map) returns map
{
    const t   = SERVO_MOUNT_TABLE[opt.servo];
    const top = opt.part == "TOP";
    const pinR  = (t.caseReliefDia - opt.casePinClearance) / 2;
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
        boxSolid(context, id, "capCut", cs, W, yS - e, y0,
                 top ? seatZ : far, top ? far : seatZ);
        const zFace = top ? hornZ : seatZ - capT;
        const zAt = function(y) { return zFace - sgn * (y0 - y) * tanS; };
        polyPrism(context, id, "slopeCut", cs.origin - W * cs.xAxis, cs.xAxis, yA0,
                  [vector(y0, zFace), vector(yS - e, zAt(yS - e)),
                   vector(yS - e, far), vector(y0, far)], 2 * W);
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
        boxSolid(context, id, "window", cs, W, t.caseWindowNear, y0,
                 backZ - fc - capT - e, backZ + t.caseWindowDepth);
        boxSolid(context, id, "baseZone", cs, W, y0, endY + sc + opt.caseTopWall + e,
                 backZ - e, hornZ - skirt);
        wrapCut = [qCreatedBy(id + "capCutExt", EntityType.BODY),
                   qCreatedBy(id + "slopeCutExt", EntityType.BODY),
                   qCreatedBy(id + "conePRev", EntityType.BODY),
                   qCreatedBy(id + "coneNRev", EntityType.BODY),
                   qCreatedBy(id + "windowExt", EntityType.BODY),
                   qCreatedBy(id + "baseZoneExt", EntityType.BODY)];
    }
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

export function caseShellPair(context is Context, id is Id, cs is CoordSystem,
                              opt is map) returns Query
{
    const useCS = opt.flip ? coordSystem(cs.origin, -cs.xAxis, cs.zAxis) : cs;
    var made = [];
    if (opt.makeTop != false)
        made = append(made, caseShellBuild(context, id + "top", useCS,
                mergeMaps(opt, { "part" : "TOP" }), qNothing()));
    if (opt.makeBottom != false)
        made = append(made, caseShellBuild(context, id + "bot", useCS,
                mergeMaps(opt, { "part" : "BOTTOM" }), qNothing()));
    return qUnion(made);
}

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

export function screwJointGeometry(context is Context, id is Id,
                                   cs is CoordSystem, opt is map) returns Query
{
    const headR     = opt.headDia / 2;
    const holeR     = opt.holeDia / 2;
    const holeDepth = opt.holeDepth;
    const cskDepth  = (headR - holeR) / tan(opt.cskAngle / 2);
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
    const w      = opt.nutSlotWidth;
    const back   = opt.bothWays ? opt.nutSlotLength : w / sqrt(3);
    const slotCs = coordSystem(cs.origin, yA, cs.zAxis);
    boxSolid(context, id, "slot", slotCs, w / 2, -opt.nutSlotLength, back,
             opt.nutDepth, opt.nutDepth + opt.nutSlotThickness);
    return qUnion([qCreatedBy(id + "screwRev", EntityType.BODY),
                   qCreatedBy(id + "slotExt", EntityType.BODY)]);
}

export function boxIn(context is Context, id is Id, cs is CoordSystem,
                      lo is Vector, hi is Vector) returns Query
{
    fCuboid(context, id, { "corner1" : lo, "corner2" : hi });
    opTransform(context, id + "xf", {
            "bodies"    : qCreatedBy(id, EntityType.BODY),
            "transform" : toWorld(cs) });
    return qCreatedBy(id, EntityType.BODY);
}

export function cylW(context is Context, id is Id, p0 is Vector, p1 is Vector,
                     r is ValueWithUnits) returns Query
{
    fCylinder(context, id, { "bottomCenter" : p0, "topCenter" : p1, "radius" : r });
    return qCreatedBy(id, EntityType.BODY);
}

export function boxW(context is Context, id is Id, lo is Vector, hi is Vector) returns Query
{
    fCuboid(context, id, { "corner1" : lo, "corner2" : hi });
    return qCreatedBy(id, EntityType.BODY);
}

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

export function partAt(context is Context, scope is Id, pt is Vector) returns Query
{
    return qContainsPoint(qBodyType(qCreatedBy(scope, EntityType.BODY),
                                    BodyType.SOLID), pt);
}

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

export function ridgeSolid(context is Context, id is Id, origin is Vector,
                           along is Vector, up is Vector, h is ValueWithUnits,
                           grow is ValueWithUnits, half is ValueWithUnits) returns Query
{
    const side = cross(up, along);
    const flat = FIXTURE.ridge_flat / 2;
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
    const W = a + 1 * mm;
    for (var e in [1, -1])
    {
        const tag = e > 0 ? "endP" : "endN";
        const hs  = h + grow + 2 * mm;
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
        opBoolean(context, id + "brCut", { "tools" : qUnion([l1, l2]), "targets" : nutPart,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }
}

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

export function blindScrewJoint(context is Context, id is Id, head is Vector, zIn is Vector,
                                slotDir is Vector, slotLen is ValueWithUnits, pocket is ValueWithUnits,
                                cskPart is Query, nutPart is Query, ridge is boolean, ridgeDir is Vector,
                                ridgeHalf is ValueWithUnits,
                                cskUp is Vector, nutUp is Vector)
{
    const f  = FIXTURE;
    const mm = millimeter;
    const cs = coordSystem(head, slotDir, zIn);
    const nd = f.joint_plate + 2 * mm;
    screwJointGeometry(context, id + "scr", cs, mergeMaps(SCREW_OPT, {
            "nutDepth" : nd, "headPocket" : pocket, "bothWays" : false, "nutSlotLength" : slotLen }));
    if (ridge)
    {
        const r = ridgeSolid(context, id + "ridge", head + f.joint_plate * zIn, ridgeDir, -zIn, f.ridge_height,
                             0 * mm, ridgeHalf);
        opBoolean(context, id + "rAdd", { "tools" : qUnion([nutPart, r]),
                "operationType" : BooleanOperationType.UNION });
    }
    opBoolean(context, id + "slotCut", { "tools" : qCreatedBy(id + "scr" + "slotExt", EntityType.BODY),
            "targets" : nutPart, "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "cut", { "tools" : qCreatedBy(id + "scr" + "screwRev", EntityType.BODY),
            "targets" : qUnion([cskPart, nutPart]), "operationType" : BooleanOperationType.SUBTRACTION });
    const rB = SCREW_OPT.holeDia / 2;
    for (var pu in [[cskPart, cskUp, "tdC"], [nutPart, nutUp, "tdN"]])
    {
        if (abs(dot(pu[1], zIn)) > 0.5 || isQueryEmpty(context, pu[0]))
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
    const along = dot(nutUp, zIn);
    if (abs(along) > 0.5)
    {
        const L  = f.bridge_layer;
        const rH = SCREW_OPT.holeDia / 2;
        const w  = SCREW_OPT.nutSlotWidth;
        const d0 = along > 0 ? nd + SCREW_OPT.nutSlotThickness : nd;
        const sg = along > 0 ? 1 : -1;
        const l1 = boxIn(context, id + "br1", cs, vector(-rH, -w / 2, d0 + (sg > 0 ? 0 * mm : -L)),
                         vector(rH, w / 2, d0 + (sg > 0 ? L : 0 * mm)));
        const l2 = boxIn(context, id + "br2", cs, vector(-rH, -rH, d0 + sg * L + (sg > 0 ? 0 * mm : -L)),
                         vector(rH, rH, d0 + sg * L + (sg > 0 ? L : 0 * mm)));
        opBoolean(context, id + "brCut", { "tools" : qUnion([l1, l2]), "targets" : nutPart,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }
}

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
    const v2 = opt.v2 == undefined ? STEER_V2 : opt.v2;
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };
    const at = function(x, y, z) returns Vector { return vector(x, y, z); };
    const holeR = SCREW_OPT.holeDia / 2;
    const T = SERVO_MOUNT_TABLE["XC330"];
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
        var plateParts = [boxW(context, P + (tag ~ "Plate"), at(sg > 0 ? L.xBlk : -L.xOut, -L.forkHalf, L.blockBot),
                               at(sg > 0 ? L.xOut : -L.xBlk, L.forkHalf, L.blockTop))];
        for (var sy in [1, -1])
            plateParts = append(plateParts, boxW(context, P + (tag ~ "Cheek" ~ (sy > 0 ? "P" : "N")),
                    at(sg > 0 ? L.xIn : -L.xBlk - 1 * mm, sy > 0 ? L.cheekIn : -L.forkHalf, L.blockBot),
                    at(sg > 0 ? L.xBlk + 1 * mm : -L.xIn, sy > 0 ? L.forkHalf : -L.cheekIn, L.blockTop)));
        const plate = qUnion(plateParts);
        const endR = cylW(context, P + (tag ~ "End"), at(sg * L.xIn, 0 * mm, 0 * mm), at(sg * L.xOut, 0 * mm, 0 * mm), L.legHalf);
        const r0 = L.bossDia / 2;
        const xc = L.xBoss + (L.legHalf - r0) / tan(L.bossChamferDeg);
        revolveProfile(context, P, tag ~ "Boss", xz, line(O, X),
                [vector(sg * L.xBoss, 0 * mm), vector(sg * L.xBoss, r0), vector(sg * xc, L.legHalf),
                 vector(sg * (L.xIn + 1 * mm), L.legHalf), vector(sg * (L.xIn + 1 * mm), 0 * mm)]);
        const boss = qCreatedBy(P + (tag ~ "BossRev"), EntityType.BODY);
        unite(context, P + (tag ~ "U"), [qCreatedBy(P + (tag ~ "Ext"), EntityType.BODY), plate, endR, boss]);
        forkPt[tag] = at(sg * (L.xOut - 1 * mm), 0 * mm, L.blockBot / 2);
        const hole = cylW(context, P + (tag ~ "Hole"), at(sg * (L.xBoss - 1 * mm), 0 * mm, 0 * mm),
                          at(sg * (L.xOut + 1 * mm), 0 * mm, 0 * mm), L.axleHole / 2);
        opBoolean(context, P + (tag ~ "Cut"), { "tools" : hole,
                "targets" : partAt(context, P, forkPt[tag]),
                "operationType" : BooleanOperationType.SUBTRACTION });
    }
    step("headset lower");
    const block = boxW(context, P + "block", at(-L.xBlk, -L.blockHalf, L.blockBot), at(L.xBlk, L.blockHalf, L.blockTop));
    const shaft = cylW(context, P + "shaft", at(0 * mm, 0 * mm, L.blockTop - 1 * mm), at(0 * mm, 0 * mm, L.zS), L.shaftR);
    const spig = doubleD(context, P + "spigot", L.spR, L.spFlat, L.zS - 1 * mm, L.zS + L.spigotH);
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
    step("lower case");
    const yCup = v2 ? L.yWallFar : L.yP;
    const cup = boxW(context, P + "cup", at(-L.cupHalf, yCup, L.caseBot), at(L.cupHalf, L.cupNear, L.zCF));
    const walls = boxW(context, P + "walls", at(-L.topOuter, L.yWallFar, L.zCF - 1 * mm), at(L.topOuter, L.yWallNear, L.wallTop));
    var lcAdd = [cup, walls];
    if (v2)
    {
        for (var sg in [1, -1])
        {
            const tg = sg > 0 ? "P" : "N";
            lcAdd = append(lcAdd, boxW(context, P + ("pad" ~ tg),
                    at(sg > 0 ? L.cupHalf - 0.5 * mm : -L.v2SideX, L.yWallFar, L.caseBot),
                    at(sg > 0 ? L.v2SideX : -L.cupHalf + 0.5 * mm, L.cupNear, L.v2PadTop)));
        }
    }
    if (v2)
    {
        polyPrism(context, P, "rrBlk", at(-L.v2RrHalfX, 0 * mm, 0 * mm), X, Y,
                  [vector(L.yWallNear - 0.5 * mm, L.v2BlkBot - 0.5 * mm), vector(L.yWallNear, L.v2BlkBot),
                   vector(L.v2RrOut, L.v2BlkTop), vector(L.v2RrOut, L.upBot), vector(L.yWallNear - 0.5 * mm, L.upBot)],
                  2 * L.v2RrHalfX);
        lcAdd = append(lcAdd, qCreatedBy(P + "rrBlkExt", EntityType.BODY));
    }
    unite(context, P + "lcU", lcAdd);
    var lcPt = v2 ? at(L.cupHalf - 0.5 * mm, -L.cavR - 2.5 * mm, L.zCF - 2 * mm)
                  : at(L.cupHalf - 0.5 * mm, yCup + 1 * mm, L.caseBot + 1 * mm);
    opBoolean(context, P + "lcCut", { "tools" : qUnion([
            boxW(context, P + "lcPocket", at(-L.inner, L.yServoFar, L.zCF), at(L.inner, L.yServoNear, L.zBack + 1 * mm)),
            cylW(context, P + "lcCav", at(0 * mm, 0 * mm, L.sleeveTop), at(0 * mm, 0 * mm, L.zCF + 0.5 * mm), L.cavR),
            cylW(context, P + "lcBore", at(0 * mm, 0 * mm, L.caseBot - 1 * mm), at(0 * mm, 0 * mm, L.sleeveTop + 1 * mm), L.boreR)]),
            "targets" : partAt(context, P, lcPt), "operationType" : BooleanOperationType.SUBTRACTION });
    caseShellBuild(context, P + "lcShell", cs, mergeMaps(CASE_OPT, { "part" : "TOP" }), partAt(context, P, lcPt));
    if (L.lowerNearPins > 0.5 * mm)
        nearRowPins(context, P + "lcNear", cs, "TOP", partAt(context, P, lcPt));
    if (v2)
    {
        const run = L.v2Foot - L.yWallFar + 1 * mm;
        polyPrism(context, P, "lcCh", at(-L.v2SideX - 1 * mm, 0 * mm, 0 * mm), X, Y,
                  [vector(L.v2Foot, L.caseBot - 1 * mm), vector(L.v2Foot, L.v2Rise),
                   vector(L.yWallFar - 1 * mm, L.v2Rise + run), vector(L.yWallFar - 1 * mm, L.caseBot - 1 * mm)],
                  2 * L.v2SideX + 2 * mm);
        opBoolean(context, P + "lcChCut", { "tools" : qCreatedBy(P + "lcChExt", EntityType.BODY),
                "targets" : partAt(context, P, lcPt), "operationType" : BooleanOperationType.SUBTRACTION });
    }
    step("upper case");
    caseShellBuild(context, P + "ucShell", cs, mergeMaps(CASE_OPT, { "part" : "BOTTOM" }), qNothing());
    var ucPt = at(L.botOuter - 0.5 * mm, (L.yWallFar + L.yWrap) / 2, L.upTopZ - 0.5 * mm);
    const yJ = L.yWrap - 1 * mm;       // a millimetre into the shell's own far wrap
    const ucWrap = boxW(context, P + "ucWrap", at(-L.botOuter, yJ, L.upBot), at(L.botOuter, L.yUpNear, L.upTopZ));
    polyPrism(context, P, "ucWin", at(-L.botOuter - 1 * mm, 0 * mm, 0 * mm), X, Y,
              [vector(L.winFar, L.zBack), vector(L.winFar, L.winBot), vector((L.winFar + L.winNear) / 2, L.winApex),
               vector(L.winNear, L.winBot), vector(L.winNear, L.zBack)], 2 * L.botOuter + 2 * mm);
    var ucAdd = [partAt(context, P, ucPt), ucWrap];
    if (v2)
    {
        polyPrism(context, P, "ucLug", at(-L.v2RrHalfX, 0 * mm, 0 * mm), X, Y,
                  [vector(L.yUpNear - 0.5 * mm, L.upBot), vector(L.v2RrOut, L.upBot), vector(L.v2RrOut, L.v2HeadZ),
                   vector(L.yUpNear - 0.5 * mm, L.v2LugTop + 0.5 * mm)], 2 * L.v2RrHalfX);
        ucAdd = append(ucAdd, qCreatedBy(P + "ucLugExt", EntityType.BODY));
    }
    else
        ucAdd = append(ucAdd, boxW(context, P + "ucPad", at(-L.padHalf, L.yP, L.padBot), at(L.padHalf, L.yUpFar + 1 * mm, L.upTopZ)));
    unite(context, P + "ucU", ucAdd);
    opBoolean(context, P + "ucCut", { "tools" : qUnion([
            boxW(context, P + "ucNest", at(-L.nestBore, yJ - 0.1 * mm, L.upBot - 1 * mm), at(L.nestBore, L.yNestNear, L.wallTop)),
            boxW(context, P + "ucCav", at(-L.inner, yJ - 0.1 * mm, L.wallTop - 1 * mm), at(L.inner, L.yServoNear, L.zBack)),
            qCreatedBy(P + "ucWinExt", EntityType.BODY),
            boxW(context, P + "ucSlotP", at(L.slotIn, L.winFar, L.zBack - 0.1 * mm), at(L.botOuter + 1 * mm, L.winNear, L.upTopZ + 1 * mm)),
            boxW(context, P + "ucSlotN", at(-L.botOuter - 1 * mm, L.winFar, L.zBack - 0.1 * mm), at(-L.slotIn, L.winNear, L.upTopZ + 1 * mm))]),
            "targets" : partAt(context, P, ucPt), "operationType" : BooleanOperationType.SUBTRACTION });
    nearRowPins(context, P + "ucNear", cs, "BOTTOM", partAt(context, P, ucPt));
    const plPt = at(L.botOuter - 1 * mm, (L.plateBack + L.yP) / 2, L.caseBot + 1 * mm);
    if (!v2)
    {
        step("mount plate");
        const plate = boxW(context, P + "plate", at(-L.botOuter, L.plateBack, L.caseBot), at(L.botOuter, L.yP, L.upTopZ));
        const nT = vector(0, sin(L.rakeDeg), cos(L.rakeDeg));
        const dT = vector(0, -cos(L.rakeDeg), sin(L.rakeDeg));      // along the tab, away from the plate
        const t0 = at(0 * mm, L.plateBack, L.caseBot);
        polyPrism(context, P, "tab", t0 + L.tabW / 2 * X, -X, dT,
                  [vector(0 * mm, 0 * mm), vector(L.tabL, 0 * mm), vector(L.tabL, L.tabT), vector(0 * mm, L.tabT)], L.tabW);
        const foot = boxW(context, P + "plateFoot", at(-L.tabW / 2, L.plateBack, L.caseBot),
                          at(L.tabW / 2, L.yP, L.caseBot + L.tabT / cos(L.rakeDeg) + 1 * mm));
        unite(context, P + "plU", [plate, foot, qCreatedBy(P + "tabExt", EntityType.BODY)]);
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
    }
    step("joints");
    const jp = FIXTURE.joint_plate;
    for (var sg in [1, -1])
    {
        const tag = sg > 0 ? "fR" : "fL";
        plainScrewJoint(context, P + ("j" ~ tag), at(sg * (L.xBlk + jp), 0 * mm, L.zFS),
                        -sg * X, Z, L.forkPocket, partAt(context, P, forkPt[tag]), partAt(context, P, hlPt),
                        -sg * X, Z);
    }
    if (!v2)
    {
        screwJoint(context, P + "jLC", at(0 * mm, L.plateBack, L.zLS), Y, Z, 0 * mm,
                   partAt(context, P, plPt), partAt(context, P, lcPt), true, X, L.ridgeHalf, Y, Z);
        screwJoint(context, P + "jUC", at(0 * mm, L.plateBack, L.zUS), Y, Z, 0 * mm,
                   partAt(context, P, plPt), partAt(context, P, ucPt), true, X, L.padHalf - 1 * mm, Y, -Z);
    }
    else
    {
        blindScrewJoint(context, P + "jUL", at(0 * mm, L.v2RrY, L.v2HeadZ), -Z, Y,
                        L.v2RrOut - L.v2RrY + 1 * mm, L.v2RrPocket,
                        partAt(context, P, ucPt), partAt(context, P, lcPt), false, X, 0 * mm, -Z, Z);
        var k = 0;
        for (var sg in [1, -1])
        {
            const hd = at(sg * (L.v2SideX + FIXTURE.joint_plate), 0 * mm, L.v2SideZ);
            const r0 = L.caseBot - 1.5 * mm;
            const rr = ridgeSolid(context, P + ("jSr" ~ k), at(sg * L.v2SideX, 0 * mm, (r0 + L.v2SideRidgeTop) / 2), Z, sg * X,
                                  FIXTURE.ridge_height, 0 * mm, (L.v2SideRidgeTop - r0) / 2);
            opBoolean(context, P + ("jSc" ~ k), { "tools" : boxW(context, P + ("jSb" ~ k), at(-50 * mm, -50 * mm, L.caseBot - 10 * mm),
                    at(50 * mm, 50 * mm, L.caseBot)), "targets" : rr, "operationType" : BooleanOperationType.SUBTRACTION });
            opBoolean(context, P + ("jSu" ~ k), { "tools" : qUnion([partAt(context, P, lcPt), rr]),
                    "operationType" : BooleanOperationType.UNION });
            blindScrewJoint(context, P + ("jS" ~ k), hd, -sg * X, -Y, -L.v2Foot + 1 * mm, 0 * mm, qNothing(),
                            partAt(context, P, lcPt), false, Y, 0 * mm, Z, Z);
            k += 1;
        }
        const rz = function(p is Vector) returns Vector { return vector(-p[0], -p[1], p[2]); };
        opTransform(context, id + "turn", { "bodies" : qUnion([sv.caseQ, partAt(context, P, lcPt), partAt(context, P, ucPt)]),
                "transform" : rotationAround(line(O, Z), 180 * degree) });
        lcPt = rz(lcPt);
        ucPt = rz(ucPt);
    }
    var parts = [["fork R", forkPt["fR"], "X-", -X, "steer"],
                 ["fork L", forkPt["fL"], "X+", X, "steer"],
                 ["headset lower", hlPt, "Z+", Z, "steer"],
                 ["headset upper", upPt, "Z+", Z, "steer"],
                 ["horn hub", hubPt, pins ? "Z+" : "Z-", pins ? Z : -Z, "steer"],
                 ["lower case", lcPt, "Z+", Z, "static"],
                 ["upper case", ucPt, "Z-", -Z, "static"]];
    if (!v2)
        parts = append(parts, ["mount plate", plPt, "Y-", -Y, "static"]);
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

export const DR = {
    "C" : 107.345 * millimeter,
    "zs" : 14.25 * millimeter,
    "Ys" : 106.395 * millimeter,
    "rP15" : 11.9366 * millimeter,
    "rP45" : 35.8099 * millimeter,
    "caseHalf" : 17 * millimeter,
    "xHorn" : 19 * millimeter,
    "xPo" : 20.6 * millimeter,
    "padH" : 1.6 * millimeter,
    "flangeIn" : 21.2 * millimeter,
    "teethIn" : 22.7 * millimeter,
    "teethOut" : 32.7 * millimeter,
    "outerFace" : 34.2 * millimeter,
    "beltX0" : 23.2 * millimeter,
    "beltX1" : 32.2 * millimeter,
    "rTip15" : 11.3651 * millimeter,
    "rRoot15" : 9.37512 * millimeter,
    "rLip15" : 11.8651 * millimeter,
    "rFl15" : 13.2933 * millimeter,
    "land15" : 0.157368 * millimeter,
    "rTip45" : 35.2384 * millimeter,
    "rRoot45" : 33.2384 * millimeter,
    "rLip45" : 35.7384 * millimeter,
    "rFl45" : 37.1665 * millimeter,
    "land45" : 0.328502 * millimeter,
    "hubFace" : 15 * millimeter,
    "hubR" : 7.5 * millimeter,
    "xCh" : 15.4067 * millimeter,
    "splineLen" : 4 * millimeter,
    "bore15R" : 2.6 * millimeter,
    "cbR" : 7.5 * millimeter,
    "xCb" : 22.25 * millimeter,
    "xCe" : 19.421 * millimeter,
    "xCi" : 35.2 * millimeter,
    "xCo" : 40.2 * millimeter,
    "stubR" : 7 * millimeter,
    "rC0" : 5.3195 * millimeter,
    "rC1" : 3.58745 * millimeter,
    "xRel" : 20.8511 * millimeter,
    "xA" : 23.2511 * millimeter,
    "xE" : 19.8511 * millimeter,
    "csBoreR" : 2.6 * millimeter,
    "endRing" : 0.987451 * millimeter,
    "clampX" : 25.2 * millimeter,
    "hexR" : 4.6188 * millimeter,
    "sfe" : 11.25 * millimeter,
    "Yb" : 71.1453 * millimeter,
    "Yf" : 117.645 * millimeter,
    "Yhc" : 94.3953 * millimeter,
    "Zi" : 28.54 * millimeter,
    "Zp" : 29.75 * millimeter,
    "skirtTop" : 12 * millimeter,
    "Ysk0" : 69.8953 * millimeter,
    "Ytab0" : 118.895 * millimeter,
    "Yfront" : 142.895 * millimeter,
    "Yc0" : 54.2 * millimeter,
    "Yj1" : 67.5179 * millimeter,
    "Yr0" : 47.5179 * millimeter,
    "riserBelt" : 2.77185 * millimeter,
    "riserTop" : 20.6 * millimeter,
    "tongueTop" : 18.2 * millimeter,
    "tongueEnd" : 18.5 * millimeter,
    "tongueHalf" : 3.5 * millimeter,
    "tongueClr" : 0.15 * millimeter,
    "holeR" : 1.8 * millimeter,
    "headR" : 3.75 * millimeter,
    "cskDepth" : 1.95 * millimeter,
    "headBoreR" : 4.05 * millimeter,
    "xH" : 23.5 * millimeter,
    "cskWeb" : 0.95 * millimeter,
    "tip" : 13.975 * millimeter,
    "nutSlotT" : 2.8 * millimeter,
    "hN0" : 3.8 * millimeter,
    "hN1" : 6.6 * millimeter,
    "hT" : 7.2 * millimeter,
    "hTip" : 6.625 * millimeter,
    "tipPast" : 0.445 * millimeter,
    "xN0" : 16.8 * millimeter,
    "web" : 1.4 * millimeter,
    "tipSlot" : 0.6 * millimeter,
    "xRin" : 12.4 * millimeter,
    "travel" : 3 * millimeter,
    "nutHalfY" : 3.225 * millimeter,
    "nutCornerY" : 3.72391 * millimeter,
    "Yscr" : 61.2339 * millimeter,
    "xTab" : 16 * millimeter,
    "yF0" : 124.895 * millimeter,
    "yF1" : 136.895 * millimeter,
    "zF" : 23 * millimeter,
    "ridgeHalf" : 6 * millimeter,
    "fixHead" : 0.674504 * millimeter,
    "cyFront" : 15 * millimeter,
    "cyRiserCh" : 5 * millimeter,
    "cyFillet" : 3 * millimeter,
    "cyEdgeCh" : 2 * millimeter,
    "reliefR" : 11.05 * millimeter,
    "wireY0" : 86.8953 * millimeter,
    "wireY1" : 97.4953 * millimeter,
    "m25Seat" : 18.2 * millimeter,
    "spacerTop" : 14.2 * millimeter,
    "rcLeg" : 16.9 * millimeter,
    "caseW" : 28.5 * millimeter,
    "caseH" : 46.5 * millimeter,
    "hornT" : 2 * millimeter,
    "hornR" : 10.25 * millimeter,
    "bossP" : 1.9 * millimeter,
    "bossR" : 3.95 * millimeter,
    "holeHW" : 11 * millimeter,
    "holeHH" : 20 * millimeter,
    "recessR" : 2.25 * millimeter,
    "recessD" : 3 * millimeter,
    "pinHoleR" : 0.8 * millimeter,
    "coreHalf" : 12.5 * millimeter,
    "coreR" : 35 * millimeter,
    "rimHalf" : 16.5 * millimeter,
    "rimOR" : 42 * millimeter,
    "rimIR" : 33 * millimeter,
    "hubStubR" : 7.5 * millimeter,
    "wheelBoreR" : 2.5 * millimeter,
    "rm" : 40 * millimeter,
    "rollBigR" : 11 * millimeter,
    "rollSmallR" : 9.5 * millimeter,
    "rollLen" : 7.5 * millimeter,
    "pairGap" : 10 * millimeter,
    "axlesN" : 8,
    "axleR" : 2.5 * millimeter,
    "hexAF" : 8 * millimeter,
    "nutL" : 4 * millimeter,
    "headL" : 5 * millimeter,
    "flCyl" : 0.5 * millimeter,
    "padR" : 10.25 * millimeter,
    "padCh" : 0.6 * millimeter,
    "bossClrR" : 4.1 * millimeter,
    "pinR" : 0.7 * millimeter,
    "pinL" : 2 * millimeter,
    "pinC" : 8 * millimeter,
    "m2R" : 1.05 * millimeter,
    "m2CbR" : 2 * millimeter,
    "skClr" : 0.04 * millimeter,
    "reliefCh" : 0.6 * millimeter,
    "spacerR" : 2.15 * millimeter,
    "m25R" : 1.35 * millimeter,
    "m25CbR" : 2.5 * millimeter,
    "armHalf" : 9.5 * millimeter,
    "csBossR" : 9.5 * millimeter,
    "bedCh" : 0.6 * millimeter,
    "fixGap" : 0.5 * millimeter,
    "bridgeLayer" : 0.2 * millimeter,
    "tiltDeg" : 45 * degree,
    "xSeat" : 21 * millimeter,
    "m2Engaged" : 4 * millimeter
};

export const HTD15 = [
    [[2.20879, 11.1484], [1.88635, 11.0842], [1.70356, 10.8109]],
    [[1.70356, 10.8109], [1.23774, 9.90009], [0.36397, 9.36805]],
    [[0.36397, 9.36805], [0, 9.37512], [-0.36397, 9.36805]],
    [[-0.36397, 9.36805], [-1.23774, 9.90009], [-1.70356, 10.8109]],
    [[-1.70356, 10.8109], [-1.88635, 11.0842], [-2.20879, 11.1484]],
    [[-2.20879, 11.1484], [-2.36294, 11.1168], [-2.51664, 11.083]],
    [[-2.51664, 11.083], [-2.78507, 10.8931], [-2.8409, 10.5691]],
    [[-2.8409, 10.5691], [-2.896, 9.54761], [-3.47783, 8.70618]],
    [[-3.47783, 8.70618], [-3.8132, 8.5646], [-4.14283, 8.4101]],
    [[-4.14283, 8.4101], [-5.15745, 8.54075], [-5.95345, 9.18332]],
    [[-5.95345, 9.18332], [-6.2316, 9.35864], [-6.5523, 9.28619]],
    [[-6.5523, 9.28619], [-6.68025, 9.19458], [-6.80692, 9.1012]],
    [[-6.80692, 9.1012], [-6.97492, 8.81858], [-6.89414, 8.49987]],
    [[-6.89414, 8.49987], [-6.52899, 7.54427], [-6.71828, 6.53893]],
    [[-6.71828, 6.53893], [-6.96707, 6.27318], [-7.20536, 5.99797]],
    [[-7.20536, 5.99797], [-8.1854, 5.70463], [-9.17394, 5.9679]],
    [[-9.17394, 5.9679], [-9.49935, 6.01492], [-9.76286, 5.8183]],
    [[-9.76286, 5.8183], [-9.84248, 5.68256], [-9.92022, 5.54574]],
    [[-9.92022, 5.54574], [-9.95875, 5.21922], [-9.75532, 4.96092]],
    [[-9.75532, 4.96092], [-9.03306, 4.23645], [-8.79707, 3.24104]],
    [[-8.79707, 3.24104], [-8.91627, 2.89707], [-9.02202, 2.54873]],
    [[-9.02202, 2.54873], [-9.79802, 1.88214], [-10.8082, 1.72056]],
    [[-10.8082, 1.72056], [-11.1246, 1.63117], [-11.2853, 1.34437]],
    [[-11.2853, 1.34437], [-11.3029, 1.18798], [-11.3182, 1.03136]],
    [[-11.3182, 1.03136], [-11.2206, 0.717402], [-10.9297, 0.564181]],
    [[-10.9297, 0.564181], [-9.97523, 0.196113], [-9.35478, -0.617252]],
    [[-9.35478, -0.617252], [-9.32376, -0.979967], [-9.27869, -1.3412]],
    [[-9.27869, -1.3412], [-9.71647, -2.26579], [-10.5736, -2.82427]],
    [[-10.5736, -2.82427], [-10.8263, -3.03462], [-10.8565, -3.36202]],
    [[-10.8565, -3.36202], [-10.8089, -3.51201], [-10.7592, -3.66134]],
    [[-10.7592, -3.66134], [-10.5423, -3.90846], [-10.2143, -3.93011]],
    [[-10.2143, -3.93011], [-9.19259, -3.87813], [-8.29496, -4.36882]],
    [[-8.29496, -4.36882], [-8.11909, -4.68756], [-7.93099, -4.99923]],
    [[-7.93099, -4.99923], [-7.95486, -6.02195], [-8.51071, -6.88076]],
    [[-8.51071, -6.88076], [-8.65599, -7.1757], [-8.55042, -7.48708]],
    [[-8.55042, -7.48708], [-8.44593, -7.60475], [-8.33982, -7.72096]],
    [[-8.33982, -7.72096], [-8.04119, -7.85851], [-7.73268, -7.74485]],
    [[-7.73268, -7.74485], [-6.82047, -7.28181], [-5.80086, -7.36498]],
    [[-5.80086, -7.36498], [-5.51056, -7.58463], [-5.21195, -7.79285]],
    [[-5.21195, -7.79285], [-4.81778, -8.73686], [-4.97626, -9.74751]],
    [[-4.97626, -9.74751], [-4.98902, -10.076], [-4.76593, -10.3176]],
    [[-4.76593, -10.3176], [-4.62261, -10.3826], [-4.47841, -10.4456]],
    [[-4.47841, -10.4456], [-4.14965, -10.4498], [-3.91404, -10.2204]],
    [[-3.91404, -10.2204], [-3.26903, -9.42641], [-2.30374, -9.08766]],
    [[-2.30374, -9.08766], [-1.9492, -9.17025], [-1.59171, -9.23901]],
    [[-1.59171, -9.23901], [-0.847656, -9.94108], [-0.581377, -10.9288]],
    [[-0.581377, -10.9288], [-0.459402, -11.2341], [-0.157363, -11.364]],
    [[-0.157363, -11.364], [-0, -11.3651], [0.157363, -11.364]],
    [[0.157363, -11.364], [0.459402, -11.2341], [0.581377, -10.9288]],
    [[0.581377, -10.9288], [0.847656, -9.94108], [1.59171, -9.23901]],
    [[1.59171, -9.23901], [1.9492, -9.17025], [2.30374, -9.08766]],
    [[2.30374, -9.08766], [3.26903, -9.42641], [3.91404, -10.2204]],
    [[3.91404, -10.2204], [4.14965, -10.4498], [4.47841, -10.4456]],
    [[4.47841, -10.4456], [4.62261, -10.3826], [4.76593, -10.3176]],
    [[4.76593, -10.3176], [4.98902, -10.076], [4.97626, -9.74751]],
    [[4.97626, -9.74751], [4.81778, -8.73686], [5.21195, -7.79285]],
    [[5.21195, -7.79285], [5.51056, -7.58463], [5.80086, -7.36498]],
    [[5.80086, -7.36498], [6.82047, -7.28181], [7.73268, -7.74485]],
    [[7.73268, -7.74485], [8.04119, -7.85851], [8.33982, -7.72096]],
    [[8.33982, -7.72096], [8.44593, -7.60475], [8.55042, -7.48708]],
    [[8.55042, -7.48708], [8.65599, -7.1757], [8.51071, -6.88076]],
    [[8.51071, -6.88076], [7.95486, -6.02195], [7.93099, -4.99923]],
    [[7.93099, -4.99923], [8.11909, -4.68756], [8.29496, -4.36882]],
    [[8.29496, -4.36882], [9.19259, -3.87813], [10.2143, -3.93011]],
    [[10.2143, -3.93011], [10.5423, -3.90846], [10.7592, -3.66134]],
    [[10.7592, -3.66134], [10.8089, -3.51201], [10.8565, -3.36202]],
    [[10.8565, -3.36202], [10.8263, -3.03462], [10.5736, -2.82427]],
    [[10.5736, -2.82427], [9.71647, -2.26579], [9.27869, -1.3412]],
    [[9.27869, -1.3412], [9.32376, -0.979967], [9.35478, -0.617252]],
    [[9.35478, -0.617252], [9.97523, 0.196113], [10.9297, 0.564181]],
    [[10.9297, 0.564181], [11.2206, 0.717402], [11.3182, 1.03136]],
    [[11.3182, 1.03136], [11.3029, 1.18798], [11.2853, 1.34437]],
    [[11.2853, 1.34437], [11.1246, 1.63117], [10.8082, 1.72056]],
    [[10.8082, 1.72056], [9.79802, 1.88214], [9.02202, 2.54873]],
    [[9.02202, 2.54873], [8.91627, 2.89707], [8.79707, 3.24104]],
    [[8.79707, 3.24104], [9.03306, 4.23645], [9.75532, 4.96092]],
    [[9.75532, 4.96092], [9.95875, 5.21922], [9.92022, 5.54574]],
    [[9.92022, 5.54574], [9.84248, 5.68256], [9.76286, 5.8183]],
    [[9.76286, 5.8183], [9.49935, 6.01492], [9.17394, 5.9679]],
    [[9.17394, 5.9679], [8.1854, 5.70463], [7.20536, 5.99797]],
    [[7.20536, 5.99797], [6.96707, 6.27318], [6.71828, 6.53893]],
    [[6.71828, 6.53893], [6.52899, 7.54427], [6.89414, 8.49987]],
    [[6.89414, 8.49987], [6.97492, 8.81858], [6.80692, 9.1012]],
    [[6.80692, 9.1012], [6.68025, 9.19458], [6.5523, 9.28619]],
    [[6.5523, 9.28619], [6.2316, 9.35864], [5.95345, 9.18332]],
    [[5.95345, 9.18332], [5.15745, 8.54075], [4.14283, 8.4101]],
    [[4.14283, 8.4101], [3.8132, 8.5646], [3.47783, 8.70618]],
    [[3.47783, 8.70618], [2.896, 9.54761], [2.8409, 10.5691]],
    [[2.8409, 10.5691], [2.78507, 10.8931], [2.51664, 11.083]],
    [[2.51664, 11.083], [2.36294, 11.1168], [2.20879, 11.1484]]
];

export const HTD45 = [
    [[2.1303, 35.1739], [1.82123, 35.0684], [1.67545, 34.7761]],
    [[1.67545, 34.7761], [1.25991, 33.7815], [0.329764, 33.2367]],
    [[0.329764, 33.2367], [0, 33.2384], [-0.329764, 33.2367]],
    [[-0.329764, 33.2367], [-1.25991, 33.7815], [-1.67545, 34.7761]],
    [[-1.67545, 34.7761], [-1.82123, 35.0684], [-2.1303, 35.1739]],
    [[-2.1303, 35.1739], [-2.4581, 35.1525], [-2.78569, 35.1281]],
    [[-2.78569, 35.1281], [-3.07707, 34.9806], [-3.18075, 34.6709]],
    [[-3.18075, 34.6709], [-3.45383, 33.6281], [-4.2991, 32.9592]],
    [[-4.2991, 32.9592], [-4.62589, 32.9149], [-4.95221, 32.8674]],
    [[-4.95221, 32.8674], [-5.94912, 33.2774], [-6.49905, 34.2045]],
    [[-6.49905, 34.2045], [-6.68408, 34.4736], [-7.00483, 34.5351]],
    [[-7.00483, 34.5351], [-7.32647, 34.4683], [-7.64747, 34.3985]],
    [[-7.64747, 34.3985], [-7.91548, 34.2119], [-7.97505, 33.8908]],
    [[-7.97505, 33.8908], [-8.10034, 32.8201], [-8.84429, 32.0401]],
    [[-8.84429, 32.0401], [-9.16173, 31.9508], [-9.47827, 31.8583]],
    [[-9.47827, 31.8583], [-10.5225, 32.1256], [-11.1961, 32.9671]],
    [[-11.1961, 32.9671], [-11.4168, 33.2079], [-11.743, 33.2241]],
    [[-11.743, 33.2241], [-12.0522, 33.1132], [-12.3604, 32.9994]],
    [[-12.3604, 32.9994], [-12.5998, 32.7773], [-12.6141, 32.451]],
    [[-12.6141, 32.451], [-12.5892, 31.3734], [-13.2173, 30.4974]],
    [[-13.2173, 30.4974], [-13.5193, 30.3648], [-13.8198, 30.2291]],
    [[-13.8198, 30.2291], [-14.8912, 30.3485], [-15.6753, 31.0881]],
    [[-15.6753, 31.0881], [-15.9274, 31.2958], [-16.2526, 31.2665]],
    [[-16.2526, 31.2665], [-16.5434, 31.1136], [-16.8327, 30.9581]],
    [[-16.8327, 30.9581], [-17.0389, 30.7048], [-17.0077, 30.3797]],
    [[-17.0077, 30.3797], [-16.833, 29.316], [-17.3331, 28.3611]],
    [[-17.3331, 28.3611], [-17.6136, 28.1877], [-17.8924, 28.0116]],
    [[-17.8924, 28.0116], [-18.9699, 27.9807], [-19.8494, 28.604]],
    [[-19.8494, 28.604], [-20.1279, 28.7746], [-20.4459, 28.7003]],
    [[-20.4459, 28.7003], [-20.7126, 28.5084], [-20.9775, 28.3141]],
    [[-20.9775, 28.3141], [-21.1464, 28.0346], [-21.0702, 27.717]],
    [[-21.0702, 27.717], [-20.7492, 26.688], [-21.1115, 25.6728]],
    [[-21.1115, 25.6728], [-21.3652, 25.4621], [-21.6168, 25.2488]],
    [[-21.6168, 25.2488], [-22.6795, 25.0683], [-23.6371, 25.5631]],
    [[-23.6371, 25.5631], [-23.9367, 25.6933], [-24.2413, 25.5754]],
    [[-24.2413, 25.5754], [-24.4786, 25.3484], [-24.7139, 25.1191]],
    [[-24.7139, 25.1191], [-24.8422, 24.8188], [-24.7226, 24.5149]],
    [[-24.7226, 24.5149], [-24.2615, 23.5405], [-24.479, 22.4848]],
    [[-24.479, 22.4848], [-24.7009, 22.2408], [-24.9204, 21.9946]],
    [[-24.9204, 21.9946], [-25.9476, 21.6679], [-26.9648, 22.0247]],
    [[-26.9648, 22.0247], [-27.2795, 22.1119], [-27.5648, 21.9528]],
    [[-27.5648, 21.9528], [-27.7682, 21.6949], [-27.9692, 21.4351]],
    [[-27.9692, 21.4351], [-28.0546, 21.1199], [-27.8938, 20.8356]],
    [[-27.8938, 20.8356], [-27.3016, 19.9349], [-27.3701, 18.8591]],
    [[-27.3701, 18.8591], [-27.5559, 18.5867], [-27.7389, 18.3124]],
    [[-27.7389, 18.3124], [-28.7107, 17.8459], [-29.7676, 18.0575]],
    [[-29.7676, 18.0575], [-30.0914, 18.1001], [-30.3517, 17.9029]],
    [[-30.3517, 17.9029], [-30.5173, 17.6192], [-30.6802, 17.3339]],
    [[-30.6802, 17.3339], [-30.7209, 17.0099], [-30.5221, 16.7507]],
    [[-30.5221, 16.7507], [-29.8103, 15.9412], [-29.7284, 14.8664]],
    [[-29.7284, 14.8664], [-29.8744, 14.5707], [-30.0175, 14.2736]],
    [[-30.0175, 14.2736], [-30.9149, 13.6764], [-31.991, 13.739]],
    [[-31.991, 13.739], [-32.3176, 13.7361], [-32.548, 13.5045]],
    [[-32.548, 13.5045], [-32.6724, 13.2005], [-32.7941, 12.8954]],
    [[-32.7941, 12.8954], [-32.7892, 12.5688], [-32.5563, 12.3399]],
    [[-32.5563, 12.3399], [-31.7388, 11.6373], [-31.5081, 10.5843]],
    [[-31.5081, 10.5843], [-31.6116, 10.2712], [-31.7119, 9.95709]],
    [[-31.7119, 9.95709], [-32.5174, 9.24081], [-33.5918, 9.15296]],
    [[-33.5918, 9.15296], [-33.9148, 9.10463], [-34.1107, 8.8433]],
    [[-34.1107, 8.8433], [-34.1916, 8.52493], [-34.2696, 8.20582]],
    [[-34.2696, 8.20582], [-34.2194, 7.88312], [-33.9569, 7.68881]],
    [[-33.9569, 7.68881], [-33.0495, 7.10686], [-32.6745, 6.09625]],
    [[-32.6745, 6.09625], [-32.7334, 5.77178], [-32.789, 5.44674]],
    [[-32.789, 5.44674], [-33.4871, 4.62533], [-34.5387, 4.38881]],
    [[-34.5387, 4.38881], [-34.8519, 4.296], [-35.0095, 4.00995]],
    [[-35.0095, 4.00995], [-35.0453, 3.68341], [-35.0781, 3.35655]],
    [[-35.0781, 3.35655], [-34.9835, 3.04399], [-34.6965, 2.8881]],
    [[-34.6965, 2.8881], [-33.717, 2.4381], [-33.205, 1.48951]],
    [[-33.205, 1.48951], [-33.2181, 1.16], [-33.228, 0.830382]],
    [[-33.228, 0.830382], [-33.8049, -0.080183], [-34.8134, -0.460765]],
    [[-34.8134, -0.460765], [-35.1106, -0.59625], [-35.2268, -0.90145]],
    [[-35.2268, -0.90145], [-35.2169, -1.2298], [-35.2039, -1.55804]],
    [[-35.2039, -1.55804], [-35.0666, -1.85439], [-34.7607, -1.96882]],
    [[-34.7607, -1.96882], [-33.7281, -2.27812], [-33.0891, -3.14623]],
    [[-33.0891, -3.14623], [-33.0563, -3.47436], [-33.0202, -3.80214]],
    [[-33.0202, -3.80214], [-33.4647, -4.78413], [-34.4105, -5.30137]],
    [[-34.4105, -5.30137], [-34.6859, -5.47689], [-34.7585, -5.7953]],
    [[-34.7585, -5.7953], [-34.703, -6.11908], [-34.6445, -6.44232]],
    [[-34.6445, -6.44232], [-34.4673, -6.71668], [-34.1485, -6.78742]],
    [[-34.1485, -6.78742], [-33.0828, -6.95], [-32.3292, -7.72072]],
    [[-32.3292, -7.72072], [-32.251, -8.04109], [-32.1697, -8.36066]],
    [[-32.1697, -8.36066], [-32.4732, -9.39497], [-33.3378, -10.0388]],
    [[-33.3378, -10.0388], [-33.5861, -10.2509], [-33.6137, -10.5764]],
    [[-33.6137, -10.5764], [-33.5137, -10.8893], [-33.4107, -11.2012]],
    [[-33.4107, -11.2012], [-33.1971, -11.4482], [-32.8715, -11.4739]],
    [[-32.8715, -11.4739], [-31.7936, -11.4866], [-30.9401, -12.1449]],
    [[-30.9401, -12.1449], [-30.8181, -12.4513], [-30.693, -12.7564]],
    [[-30.693, -12.7564], [-30.8497, -13.8229], [-31.6162, -14.5808]],
    [[-31.6162, -14.5808], [-31.8326, -14.8255], [-31.8147, -15.1516]],
    [[-31.8147, -15.1516], [-31.672, -15.4475], [-31.5267, -15.7421]],
    [[-31.5267, -15.7421], [-31.2807, -15.957], [-30.9547, -15.9371]],
    [[-30.9547, -15.9371], [-29.8856, -15.7996], [-28.9487, -16.3328]],
    [[-28.9487, -16.3328], [-28.7853, -16.6192], [-28.619, -16.9039]],
    [[-28.619, -16.9039], [-28.6257, -17.9819], [-29.2793, -18.839]],
    [[-29.2793, -18.839], [-29.4595, -19.1114], [-29.3964, -19.4318]],
    [[-29.3964, -19.4318], [-29.2139, -19.705], [-29.029, -19.9765]],
    [[-29.029, -19.9765], [-28.7555, -20.1551], [-28.4355, -20.09]],
    [[-28.4355, -20.09], [-27.3959, -19.8051], [-26.3939, -20.2027]],
    [[-26.3939, -20.2027], [-26.1922, -20.4636], [-25.9879, -20.7224]],
    [[-25.9879, -20.7224], [-25.8445, -21.7908], [-26.3724, -22.7306]],
    [[-26.3724, -22.7306], [-26.513, -23.0254], [-26.4059, -23.3339]],
    [[-26.4059, -23.3339], [-26.1872, -23.5791], [-25.9663, -23.8222]],
    [[-25.9663, -23.8222], [-25.6706, -23.9609], [-25.3627, -23.852]],
    [[-25.3627, -23.852], [-24.3729, -23.4252], [-23.3254, -23.6794]],
    [[-23.3254, -23.6794], [-23.0893, -23.9097], [-22.851, -24.1376]],
    [[-22.851, -24.1376], [-22.5603, -25.1756], [-22.9523, -26.1797]],
    [[-22.9523, -26.1797], [-23.0505, -26.4912], [-22.9014, -26.7818]],
    [[-22.9014, -26.7818], [-22.6508, -26.9942], [-22.3982, -27.2041]],
    [[-22.3982, -27.2041], [-22.0861, -27.3004], [-21.7964, -27.1497]],
    [[-21.7964, -27.1497], [-20.8756, -26.5893], [-19.8028, -26.6952]],
    [[-19.8028, -26.6952], [-19.537, -26.8904], [-19.2693, -27.0829]],
    [[-19.2693, -27.0829], [-18.837, -28.0704], [-19.0854, -29.1193]],
    [[-19.0854, -29.1193], [-19.1393, -29.4414], [-18.9513, -29.7084]],
    [[-18.9513, -29.7084], [-18.6735, -29.8838], [-18.3941, -30.0566]],
    [[-18.3941, -30.0566], [-18.0717, -30.1085], [-17.8057, -29.9189]],
    [[-17.8057, -29.9189], [-16.9719, -29.2358], [-15.8949, -29.1915]],
    [[-15.8949, -29.1915], [-15.6045, -29.3477], [-15.3125, -29.5011]],
    [[-15.3125, -29.5011], [-14.747, -30.4188], [-14.8471, -31.4921]],
    [[-14.8471, -31.4921], [-14.8556, -31.8186], [-14.6322, -32.0568]],
    [[-14.6322, -32.0568], [-14.3327, -32.1918], [-14.032, -32.3241]],
    [[-14.032, -32.3241], [-13.7055, -32.3306], [-13.4685, -32.1058]],
    [[-13.4685, -32.1058], [-12.7379, -31.3133], [-11.6775, -31.1195]],
    [[-11.6775, -31.1195], [-11.3682, -31.2338], [-11.0578, -31.3451]],
    [[-11.0578, -31.3451], [-10.37, -32.1751], [-10.3197, -33.2519]],
    [[-10.3197, -33.2519], [-10.2827, -33.5764], [-10.0284, -33.7813]],
    [[-10.0284, -33.7813], [-9.71301, -33.8733], [-9.39682, -33.9624]],
    [[-9.39682, -33.9624], [-9.07256, -33.9234], [-8.8692, -33.6678]],
    [[-8.8692, -33.6678], [-8.25595, -32.7813], [-7.23286, -32.4419]],
    [[-7.23286, -32.4419], [-6.91064, -32.512], [-6.58775, -32.579]],
    [[-6.58775, -32.579], [-5.79119, -33.3052], [-5.59152, -34.3645]],
    [[-5.59152, -34.3645], [-5.50969, -34.6807], [-5.22932, -34.8482]],
    [[-5.22932, -34.8482], [-4.90423, -34.8954], [-4.57872, -34.9396]],
    [[-4.57872, -34.9396], [-4.26304, -34.8559], [-4.09723, -34.5745]],
    [[-4.09723, -34.5745], [-3.61332, -33.6113], [-2.64744, -33.1328]],
    [[-2.64744, -33.1328], [-2.31859, -33.1574], [-1.98952, -33.1788]],
    [[-1.98952, -33.1788], [-1.09964, -33.7871], [-0.754487, -34.8083]],
    [[-0.754487, -34.8083], [-0.629454, -35.11], [-0.328497, -35.2368]],
    [[-0.328497, -35.2368], [-0, -35.2384], [0.328497, -35.2368]],
    [[0.328497, -35.2368], [0.629454, -35.11], [0.754487, -34.8083]],
    [[0.754487, -34.8083], [1.09964, -33.7871], [1.98952, -33.1788]],
    [[1.98952, -33.1788], [2.31859, -33.1574], [2.64744, -33.1328]],
    [[2.64744, -33.1328], [3.61332, -33.6113], [4.09723, -34.5745]],
    [[4.09723, -34.5745], [4.26304, -34.8559], [4.57872, -34.9396]],
    [[4.57872, -34.9396], [4.90423, -34.8954], [5.22932, -34.8482]],
    [[5.22932, -34.8482], [5.50969, -34.6807], [5.59152, -34.3645]],
    [[5.59152, -34.3645], [5.79119, -33.3052], [6.58775, -32.579]],
    [[6.58775, -32.579], [6.91064, -32.512], [7.23286, -32.4419]],
    [[7.23286, -32.4419], [8.25595, -32.7813], [8.8692, -33.6678]],
    [[8.8692, -33.6678], [9.07256, -33.9234], [9.39682, -33.9624]],
    [[9.39682, -33.9624], [9.71301, -33.8733], [10.0284, -33.7813]],
    [[10.0284, -33.7813], [10.2827, -33.5764], [10.3197, -33.2519]],
    [[10.3197, -33.2519], [10.37, -32.1751], [11.0578, -31.3451]],
    [[11.0578, -31.3451], [11.3682, -31.2338], [11.6775, -31.1195]],
    [[11.6775, -31.1195], [12.7379, -31.3133], [13.4685, -32.1058]],
    [[13.4685, -32.1058], [13.7055, -32.3306], [14.032, -32.3241]],
    [[14.032, -32.3241], [14.3327, -32.1918], [14.6322, -32.0568]],
    [[14.6322, -32.0568], [14.8556, -31.8186], [14.8471, -31.4921]],
    [[14.8471, -31.4921], [14.747, -30.4188], [15.3125, -29.5011]],
    [[15.3125, -29.5011], [15.6045, -29.3477], [15.8949, -29.1915]],
    [[15.8949, -29.1915], [16.9719, -29.2358], [17.8057, -29.9189]],
    [[17.8057, -29.9189], [18.0717, -30.1085], [18.3941, -30.0566]],
    [[18.3941, -30.0566], [18.6735, -29.8838], [18.9513, -29.7084]],
    [[18.9513, -29.7084], [19.1393, -29.4414], [19.0854, -29.1193]],
    [[19.0854, -29.1193], [18.837, -28.0704], [19.2693, -27.0829]],
    [[19.2693, -27.0829], [19.537, -26.8904], [19.8028, -26.6952]],
    [[19.8028, -26.6952], [20.8756, -26.5893], [21.7964, -27.1497]],
    [[21.7964, -27.1497], [22.0861, -27.3004], [22.3982, -27.2041]],
    [[22.3982, -27.2041], [22.6508, -26.9942], [22.9014, -26.7818]],
    [[22.9014, -26.7818], [23.0505, -26.4912], [22.9523, -26.1797]],
    [[22.9523, -26.1797], [22.5603, -25.1756], [22.851, -24.1376]],
    [[22.851, -24.1376], [23.0893, -23.9097], [23.3254, -23.6794]],
    [[23.3254, -23.6794], [24.3729, -23.4252], [25.3627, -23.852]],
    [[25.3627, -23.852], [25.6706, -23.9609], [25.9663, -23.8222]],
    [[25.9663, -23.8222], [26.1872, -23.5791], [26.4059, -23.3339]],
    [[26.4059, -23.3339], [26.513, -23.0254], [26.3724, -22.7306]],
    [[26.3724, -22.7306], [25.8445, -21.7908], [25.9879, -20.7224]],
    [[25.9879, -20.7224], [26.1922, -20.4636], [26.3939, -20.2027]],
    [[26.3939, -20.2027], [27.3959, -19.8051], [28.4355, -20.09]],
    [[28.4355, -20.09], [28.7555, -20.1551], [29.029, -19.9765]],
    [[29.029, -19.9765], [29.2139, -19.705], [29.3964, -19.4318]],
    [[29.3964, -19.4318], [29.4595, -19.1114], [29.2793, -18.839]],
    [[29.2793, -18.839], [28.6257, -17.9819], [28.619, -16.9039]],
    [[28.619, -16.9039], [28.7853, -16.6192], [28.9487, -16.3328]],
    [[28.9487, -16.3328], [29.8856, -15.7996], [30.9547, -15.9371]],
    [[30.9547, -15.9371], [31.2807, -15.957], [31.5267, -15.7421]],
    [[31.5267, -15.7421], [31.672, -15.4475], [31.8147, -15.1516]],
    [[31.8147, -15.1516], [31.8326, -14.8255], [31.6162, -14.5808]],
    [[31.6162, -14.5808], [30.8497, -13.8229], [30.693, -12.7564]],
    [[30.693, -12.7564], [30.8181, -12.4513], [30.9401, -12.1449]],
    [[30.9401, -12.1449], [31.7936, -11.4866], [32.8715, -11.4739]],
    [[32.8715, -11.4739], [33.1971, -11.4482], [33.4107, -11.2012]],
    [[33.4107, -11.2012], [33.5137, -10.8893], [33.6137, -10.5764]],
    [[33.6137, -10.5764], [33.5861, -10.2509], [33.3378, -10.0388]],
    [[33.3378, -10.0388], [32.4732, -9.39497], [32.1697, -8.36066]],
    [[32.1697, -8.36066], [32.251, -8.04109], [32.3292, -7.72072]],
    [[32.3292, -7.72072], [33.0828, -6.95], [34.1485, -6.78742]],
    [[34.1485, -6.78742], [34.4673, -6.71668], [34.6445, -6.44232]],
    [[34.6445, -6.44232], [34.703, -6.11908], [34.7585, -5.7953]],
    [[34.7585, -5.7953], [34.6859, -5.47689], [34.4105, -5.30137]],
    [[34.4105, -5.30137], [33.4647, -4.78413], [33.0202, -3.80214]],
    [[33.0202, -3.80214], [33.0563, -3.47436], [33.0891, -3.14623]],
    [[33.0891, -3.14623], [33.7281, -2.27812], [34.7607, -1.96882]],
    [[34.7607, -1.96882], [35.0666, -1.85439], [35.2039, -1.55804]],
    [[35.2039, -1.55804], [35.2169, -1.2298], [35.2268, -0.90145]],
    [[35.2268, -0.90145], [35.1106, -0.59625], [34.8134, -0.460765]],
    [[34.8134, -0.460765], [33.8049, -0.080183], [33.228, 0.830382]],
    [[33.228, 0.830382], [33.2181, 1.16], [33.205, 1.48951]],
    [[33.205, 1.48951], [33.717, 2.4381], [34.6965, 2.8881]],
    [[34.6965, 2.8881], [34.9835, 3.04399], [35.0781, 3.35655]],
    [[35.0781, 3.35655], [35.0453, 3.68341], [35.0095, 4.00995]],
    [[35.0095, 4.00995], [34.8519, 4.296], [34.5387, 4.38881]],
    [[34.5387, 4.38881], [33.4871, 4.62533], [32.789, 5.44674]],
    [[32.789, 5.44674], [32.7334, 5.77178], [32.6745, 6.09625]],
    [[32.6745, 6.09625], [33.0495, 7.10686], [33.9569, 7.68881]],
    [[33.9569, 7.68881], [34.2194, 7.88312], [34.2696, 8.20582]],
    [[34.2696, 8.20582], [34.1916, 8.52493], [34.1107, 8.8433]],
    [[34.1107, 8.8433], [33.9148, 9.10463], [33.5918, 9.15296]],
    [[33.5918, 9.15296], [32.5174, 9.24081], [31.7119, 9.95709]],
    [[31.7119, 9.95709], [31.6116, 10.2712], [31.5081, 10.5843]],
    [[31.5081, 10.5843], [31.7388, 11.6373], [32.5563, 12.3399]],
    [[32.5563, 12.3399], [32.7892, 12.5688], [32.7941, 12.8954]],
    [[32.7941, 12.8954], [32.6724, 13.2005], [32.548, 13.5045]],
    [[32.548, 13.5045], [32.3176, 13.7361], [31.991, 13.739]],
    [[31.991, 13.739], [30.9149, 13.6764], [30.0175, 14.2736]],
    [[30.0175, 14.2736], [29.8744, 14.5707], [29.7284, 14.8664]],
    [[29.7284, 14.8664], [29.8103, 15.9412], [30.5221, 16.7507]],
    [[30.5221, 16.7507], [30.7209, 17.0099], [30.6802, 17.3339]],
    [[30.6802, 17.3339], [30.5173, 17.6192], [30.3517, 17.9029]],
    [[30.3517, 17.9029], [30.0914, 18.1001], [29.7676, 18.0575]],
    [[29.7676, 18.0575], [28.7107, 17.8459], [27.7389, 18.3124]],
    [[27.7389, 18.3124], [27.5559, 18.5867], [27.3701, 18.8591]],
    [[27.3701, 18.8591], [27.3016, 19.9349], [27.8938, 20.8356]],
    [[27.8938, 20.8356], [28.0546, 21.1199], [27.9692, 21.4351]],
    [[27.9692, 21.4351], [27.7682, 21.6949], [27.5648, 21.9528]],
    [[27.5648, 21.9528], [27.2795, 22.1119], [26.9648, 22.0247]],
    [[26.9648, 22.0247], [25.9476, 21.6679], [24.9204, 21.9946]],
    [[24.9204, 21.9946], [24.7009, 22.2408], [24.479, 22.4848]],
    [[24.479, 22.4848], [24.2615, 23.5405], [24.7226, 24.5149]],
    [[24.7226, 24.5149], [24.8422, 24.8188], [24.7139, 25.1191]],
    [[24.7139, 25.1191], [24.4786, 25.3484], [24.2413, 25.5754]],
    [[24.2413, 25.5754], [23.9367, 25.6933], [23.6371, 25.5631]],
    [[23.6371, 25.5631], [22.6795, 25.0683], [21.6168, 25.2488]],
    [[21.6168, 25.2488], [21.3652, 25.4621], [21.1115, 25.6728]],
    [[21.1115, 25.6728], [20.7492, 26.688], [21.0702, 27.717]],
    [[21.0702, 27.717], [21.1464, 28.0346], [20.9775, 28.3141]],
    [[20.9775, 28.3141], [20.7126, 28.5084], [20.4459, 28.7003]],
    [[20.4459, 28.7003], [20.1279, 28.7746], [19.8494, 28.604]],
    [[19.8494, 28.604], [18.9699, 27.9807], [17.8924, 28.0116]],
    [[17.8924, 28.0116], [17.6136, 28.1877], [17.3331, 28.3611]],
    [[17.3331, 28.3611], [16.833, 29.316], [17.0077, 30.3797]],
    [[17.0077, 30.3797], [17.0389, 30.7048], [16.8327, 30.9581]],
    [[16.8327, 30.9581], [16.5434, 31.1136], [16.2526, 31.2665]],
    [[16.2526, 31.2665], [15.9274, 31.2958], [15.6753, 31.0881]],
    [[15.6753, 31.0881], [14.8912, 30.3485], [13.8198, 30.2291]],
    [[13.8198, 30.2291], [13.5193, 30.3648], [13.2173, 30.4974]],
    [[13.2173, 30.4974], [12.5892, 31.3734], [12.6141, 32.451]],
    [[12.6141, 32.451], [12.5998, 32.7773], [12.3604, 32.9994]],
    [[12.3604, 32.9994], [12.0522, 33.1132], [11.743, 33.2241]],
    [[11.743, 33.2241], [11.4168, 33.2079], [11.1961, 32.9671]],
    [[11.1961, 32.9671], [10.5225, 32.1256], [9.47827, 31.8583]],
    [[9.47827, 31.8583], [9.16173, 31.9508], [8.84429, 32.0401]],
    [[8.84429, 32.0401], [8.10034, 32.8201], [7.97505, 33.8908]],
    [[7.97505, 33.8908], [7.91548, 34.2119], [7.64747, 34.3985]],
    [[7.64747, 34.3985], [7.32647, 34.4683], [7.00483, 34.5351]],
    [[7.00483, 34.5351], [6.68408, 34.4736], [6.49905, 34.2045]],
    [[6.49905, 34.2045], [5.94912, 33.2774], [4.95221, 32.8674]],
    [[4.95221, 32.8674], [4.62589, 32.9149], [4.2991, 32.9592]],
    [[4.2991, 32.9592], [3.45383, 33.6281], [3.18075, 34.6709]],
    [[3.18075, 34.6709], [3.07707, 34.9806], [2.78569, 35.1281]],
    [[2.78569, 35.1281], [2.4581, 35.1525], [2.1303, 35.1739]]
];

export const SPLINE = [
    [[1.77416, 5.62693], [0, 5.9], [-1.77416, 5.62693]],
    [[-1.77416, 5.62693], [-1.32311, 4.19635]],
    [[-1.32311, 4.19635], [-2.58625, 3.55967], [-3.58211, 2.55509]],
    [[-3.58211, 2.55509], [-4.80328, 3.42615]],
    [[-4.80328, 3.42615], [-5.61123, 1.8232], [-5.89977, 0.051487]],
    [[-5.89977, 0.051487], [-4.39983, 0.038397]],
    [[-4.39983, 0.038397], [-4.18465, -1.35967], [-3.53697, -2.61722]],
    [[-3.53697, -2.61722], [-4.74275, -3.50945]],
    [[-4.74275, -3.50945], [-3.46793, -4.7732], [-1.8721, -5.59511]],
    [[-1.8721, -5.59511], [-1.39614, -4.17262]],
    [[-1.39614, -4.17262], [-0, -4.4], [1.39614, -4.17262]],
    [[1.39614, -4.17262], [1.8721, -5.59511]],
    [[1.8721, -5.59511], [3.46793, -4.7732], [4.74275, -3.50945]],
    [[4.74275, -3.50945], [3.53697, -2.61722]],
    [[3.53697, -2.61722], [4.18465, -1.35967], [4.39983, 0.038397]],
    [[4.39983, 0.038397], [5.89977, 0.051487]],
    [[5.89977, 0.051487], [5.61123, 1.8232], [4.80328, 3.42615]],
    [[4.80328, 3.42615], [3.58211, 2.55509]],
    [[3.58211, 2.55509], [2.58625, 3.55967], [1.32311, 4.19635]],
    [[1.32311, 4.19635], [1.77416, 5.62693]]
];

export const BELT = [
    [[-1.13829, 12.4562], [103.084, 21.9804]],
    [[103.084, 21.9804], [142.455, -19.0796], [93.6672, -48.3322]],
    [[93.6672, -48.3322], [-4.376, -11.7177]],
    [[-4.376, -11.7177], [-12.3974, 1.66044], [-1.13829, 12.4562]],
    [[-0.810672, 8.87116], [103.412, 18.3953]],
    [[103.412, 18.3953], [138.887, -18.6017], [94.9266, -44.9597]],
    [[94.9266, -44.9597], [-3.11653, -8.34517]],
    [[-3.11653, -8.34517], [-8.82928, 1.18254], [-0.810672, 8.87116]]
];

export const RAIL = { "innerX" : 16, "plate" : 4.6, "outlineBike" : [[82.555, 41.896], [86, 38.6], [150, 38.6], [160.619, 60.022], [171.727, 62.998], [207.515005, 72.5872642], [204.215062, 84.9028185], [201.765572, 86.3170321], [194.231351, 84.2982435], [192.675716, 81.6038048], [168.479, 75.12], [105.11, 63.04], [104.404, 63.746]], "rearBike" : [[29.2388654, 62.8730689], [-7.01500503, 72.5872642], [-3.71506221, 84.9028185], [-1.26557247, 86.3170321], [6.26864898, 84.2982435], [7.8242839, 81.6038048], [32.8305441, 74.9033976], [29.2388654, 71.3117189]], "pieces" : [[[88.000146, -28.7502546], [88.1055049, -33.5168614], [133.360339, -78.7716954], [156.016747, -71.1328209], [165.975639, -76.8830132], [198.062214, -95.4083205], [204.437214, -84.3664966], [203.705163, -81.6344458], [196.950165, -77.7344458], [193.944909, -78.5397017], [172.250505, -66.014782], [118.900005, -29.7479823], [118.900005, -28.7495475]], [[65.1329734, 23.7829734], [46.3665891, 56.2873044], [57.408413, 62.6623044], [60.1404639, 61.9302536], [64.0404639, 55.1752554], [63.235208, 52.1699996], [76.1794007, 29.75], [71.1, 29.75]]], "leftPieces" : [[[118.895294, 20], [150.000294, 20], [150.000294, 33.5], [118.895294, 33.5]], [[64.7, 29.75], [76.2, 29.75], [76.2, 41.25], [64.7, 41.25]]], "stops" : [{ "kind" : "back", "x" : [8, 16.5], "y" : [55.7, 57.7], "z" : [42.5, 50], "face" : 57.7, "t" : 2 }, { "kind" : "side", "x" : [11.3, 16.5], "y" : [55.7, 60.7], "z" : [42.5, 50], "face" : 57.7, "t" : 2 }], "postRear" : [[46.3665891, 56.2873044], [57.408413, 62.6623044], [60.1404639, 61.9302536], [64.0404639, 55.1752554], [63.235208, 52.1699996]], "casePoly" : [[54.2, -12.85], [71.1, -29.75], [118.895294, -29.75], [118.895294, 29.75], [71.1, 29.75], [54.2, 12.85]], "joints" : [{ "tag" : "steer", "head" : [179.585323, -76.3109638], "dir" : [0.5, 0.866025404], "half" : 3.5, "room" : 1.49948508, "gMid" : -2.65, "gHalf" : 7.15 }, { "tag" : "bridge", "head" : [116.018899, -54.3938356], "dir" : [0.707106781, 0.707106781], "half" : 3, "room" : 1.2255, "gMid" : -1.738, "gHalf" : 5.738 }, { "tag" : "pi_low", "head" : [147.000004, -55.4999627], "dir" : [-0.707106781, -0.707106781], "half" : 3, "room" : 1.74966324, "gMid" : -2.25, "gHalf" : 6.25 }], "leftJoints" : [{ "tag" : "carrier", "head" : [70.4500386, 35.5000131], "dir" : [-1.8369702e-16, -1], "half" : 3, "room" : 1.99996144, "gMid" : -4.5, "gHalf" : 8.5 }, { "tag" : "carrier_front", "head" : [125.000003, 26.9999532], "dir" : [-1.8369702e-16, -1], "half" : 3, "room" : 2.35470926, "gMid" : -3.5, "gHalf" : 7.5 }, { "tag" : "pi_top", "head" : [140.999991, 26.9999532], "dir" : [-0.707106781, -0.707106781], "half" : 3, "room" : 2.7500468, "gMid" : -4.75, "gHalf" : 8.75 }], "anchor" : [123.743687, -60.1040764], "ahY1" : 122.395294, "ahN0" : 119.895294, "ahN1" : 121.495294, "ahTip" : 119.995294, "ahHoleR" : 1.2, "ahHoleY0" : 118.295294, "ahX" : 15, "ahZ" : [-25.7, 5.3], "ahNutHalf" : 2.05, "ahNutOut" : 17.4094011, "ahTabHalf" : 3.05, "ahTabIn" : 12, "ahBoard" : { "x" : 17, "y0" : 122.395294, "y1" : 123.995294, "z0" : -29.75, "z1" : 10.25 }, "ahRailGap" : 1.96333968, "ahScrewPast" : 0, "earGap" : 9.75, "cageWall" : 0.95, "cageNd" : 5.55, "cage" : [{ "head" : [56.078286, 54.966152], "slot" : [-0.5, 0.866025404], "slotLen" : 7, "bike" : [0.786397524, 78.5202751, 75] }, { "head" : [196.741062, -85.6966236], "slot" : [0.866025404, -0.5], "slotLen" : 7, "bike" : [199.713602, 78.5202751, 105] }] };

export function segPrism(context is Context, id is Id, tag is string, origin is Vector,
                         normal is Vector, xDir is Vector, segs is array, depth is ValueWithUnits,
                         outerOnly is boolean) returns Query
{
    const mm = millimeter;
    var sk = newSketchOnPlane(context, id + tag, { "sketchPlane" : plane(origin, normal, xDir) });
    for (var i = 0; i < size(segs); i += 1)
    {
        const s = segs[i];
        if (size(s) == 2)
            skLineSegment(sk, "s" ~ i, { "start" : vector(s[0][0], s[0][1]) * mm,
                                         "end" : vector(s[1][0], s[1][1]) * mm });
        else
            skArc(sk, "s" ~ i, { "start" : vector(s[0][0], s[0][1]) * mm,
                                 "mid" : vector(s[1][0], s[1][1]) * mm,
                                 "end" : vector(s[2][0], s[2][1]) * mm });
    }
    skSolve(sk);
    opExtrude(context, id + (tag ~ "Ext"), {
            "entities" : qSketchRegion(id + tag, outerOnly),
            "direction" : normal,
            "endBound" : BoundingType.BLIND,
            "endDepth" : depth });
    opDeleteBodies(context, id + (tag ~ "Del"), { "entities" : qCreatedBy(id + tag, EntityType.BODY) });
    return qCreatedBy(id + (tag ~ "Ext"), EntityType.BODY);
}

export function cutFrom(context is Context, id is Id, target is Query, tools is array)
{
    opBoolean(context, id, { "tools" : qUnion(tools), "targets" : target,
            "operationType" : BooleanOperationType.SUBTRACTION });
}

export function cbLayers(context is Context, id is Id, x0 is ValueWithUnits, y is ValueWithUnits,
                         z is ValueWithUnits, cbR is ValueWithUnits, holeR is ValueWithUnits,
                         u is Vector, ext is ValueWithUnits, bl is ValueWithUnits) returns array
{
    const mm = millimeter;
    const X = vector(1, 0, 0);
    const Y = vector(0, 1, 0);
    const n = vector(-u[1], u[0]);
    const rect = function(a0, a1, hw) returns array
    {
        return [a0 * u - hw * n, a1 * u - hw * n, a1 * u + hw * n, a0 * u + hw * n];
    };
    const far = max([ext, cbR]) + 1 * mm;
    polyPrism(context, id, "l1", vector(x0 - 0.5 * mm, y, z), X, Y, rect(-cbR - 1 * mm, far, holeR), bl + 0.5 * mm);
    var keep = [cylW(context, id + "kc", vector(x0 - 1 * mm, y, z), vector(x0 + bl + 1 * mm, y, z), cbR)];
    if (ext > 0 * mm)
    {
        polyPrism(context, id, "ks", vector(x0 - 1 * mm, y, z), X, Y, rect(0 * mm, far + 1 * mm, cbR), bl + 2 * mm);
        keep = append(keep, qCreatedBy(id + "ksExt", EntityType.BODY));
    }
    const out = cylW(context, id + "out", vector(x0 - 0.9 * mm, y, z), vector(x0 + bl + 0.9 * mm, y, z), far + cbR + 2 * mm);
    cutFrom(context, id + "ring", out, keep);
    cutFrom(context, id + "clip", qCreatedBy(id + "l1Ext", EntityType.BODY), [out]);
    polyPrism(context, id, "l2", vector(x0 + bl - 0.1 * mm, y, z), X, Y, rect(-holeR, holeR, holeR), bl + 0.1 * mm);
    return [qCreatedBy(id + "l1Ext", EntityType.BODY), qCreatedBy(id + "l2Ext", EntityType.BODY)];
}

export function xc430Envelope(context is Context, id is Id, cs is CoordSystem) returns map
{
    const L  = DR;
    const mm = millimeter;
    const zc = -L.hornT;
    const zb = zc - 2 * L.caseHalf;
    const caseQ = boxIn(context, id + "case", cs, vector(-(L.caseH - L.sfe), -L.caseW / 2, zb),
                        vector(L.sfe, L.caseW / 2, zc));
    const horn = cylIn(context, id + "horn", cs, vector(0 * mm, 0 * mm, zc), vector(0 * mm, 0 * mm, 0 * mm), L.hornR);
    const boss = cylIn(context, id + "boss", cs, vector(0 * mm, 0 * mm, -0.1 * mm), vector(0 * mm, 0 * mm, L.bossP), L.bossR);
    unite(context, id + "hornU", [horn, boss]);
    var rec = [];
    var k = 0;
    const xc = L.sfe - L.caseH / 2;
    for (var sx in [1, -1])
    {
        for (var sy in [1, -1])
        {
            const x = xc + sx * L.holeHH;
            const y = sy * L.holeHW;
            rec = append(rec, cylIn(context, id + ("rf" ~ k), cs, vector(x, y, zc - L.recessD), vector(x, y, zc + 0.1 * mm), L.recessR));
            rec = append(rec, cylIn(context, id + ("rb" ~ k), cs, vector(x, y, zb - 0.1 * mm), vector(x, y, zb + L.recessD), L.recessR));
            k += 1;
        }
    }
    cutFrom(context, id + "recess", caseQ, rec);
    var ph = [];
    for (var i = 0; i < 4; i += 1)
    {
        const p = L.pinC * vector(cos(i * 90 * degree), sin(i * 90 * degree));
        ph = append(ph, cylIn(context, id + ("ph" ~ i), cs, vector(p[0], p[1], zc - 0.2 * mm), vector(p[0], p[1], 0.1 * mm), L.pinHoleR));
    }
    cutFrom(context, id + "pinHoles", qUnion([caseQ, horn, boss]), ph);
    return { "caseQ" : caseQ, "horn" : qUnion([horn, boss]) };
}

export function driveBuild(context is Context, id is Id, opt is map) returns map
{
    const L  = DR;
    const mm = millimeter;
    const O  = vector(0, 0, 0) * meter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const P  = id + "parts";
    const H  = id + "hw";
    const bl = L.bridgeLayer;
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };
    const at = function(x, y, z) returns Vector { return vector(x, y, z); };
    const flip = function(p is Vector) returns Vector { return vector(-p[0], p[1], -p[2]); };
    const ya = L.Ys;
    const za = -L.zs;
    const pa = at(0 * mm, ya, za);
    const xz  = plane(O, -Y, X);
    const xzA = plane(pa, -Y, X);
    const z0 = 0 * mm;
    step("servo");
    const sv = xc430Envelope(context, H + "servoA", coordSystem(at(-L.xHorn, ya, za), Y, -X));
    step("wheel");
    var wp = [cylW(context, H + "core", at(-L.coreHalf, z0, z0), at(L.coreHalf, z0, z0), L.coreR)];
    for (var sg in [1, -1])
    {
        const tg = sg > 0 ? "P" : "N";
        const rim = cylW(context, H + ("rim" ~ tg), at(sg * (L.coreHalf - 0.01 * mm), z0, z0), at(sg * L.rimHalf, z0, z0), L.rimOR);
        cutFrom(context, H + ("rimCut" ~ tg), rim, [cylW(context, H + ("rimIn" ~ tg), at(sg * L.coreHalf, z0, z0),
                at(sg * (L.rimHalf + 1 * mm), z0, z0), L.rimIR)]);
        wp = append(wp, rim);
        wp = append(wp, cylW(context, H + ("stub" ~ tg), at(sg * (L.coreHalf - 0.1 * mm), z0, z0), at(sg * L.hubFace, z0, z0), L.hubStubR));
    }
    for (var k = 0; k < L.axlesN; k += 1)
    {
        const th = k * 360 / L.axlesN * degree;
        const e = vector(0, cos(th), sin(th));
        const t = vector(0, -sin(th), cos(th));
        const c = L.rm * e;
        for (var sg in [1, -1])
        {
            const tg = "rl" ~ k ~ (sg > 0 ? "P" : "N");
            const a = sg * L.pairGap / 2;
            const b = sg * (L.pairGap / 2 + L.rollLen);
            revolveProfile(context, H, tg, plane(c, e, t), line(c, t),
                    [vector(a, z0), vector(b, z0), vector(b, L.rollSmallR), vector(a, L.rollBigR)]);
            wp = append(wp, qCreatedBy(H + (tg ~ "Rev"), EntityType.BODY));
        }
    }
    unite(context, H + "wheelU", wp);
    const wheelPt = at(z0, z0, L.coreR - 2 * mm);
    const wheel = partAt(context, H, wheelPt);
    cutFrom(context, H + "wheelBore", wheel, [
            cylW(context, H + "wBore", at(-L.rimHalf - 1 * mm, z0, z0), at(L.rimHalf + 1 * mm, z0, z0), L.wheelBoreR),
            segPrism(context, H, "wSplN", at(-L.hubFace - 0.5 * mm, z0, z0), X, Y, SPLINE, L.splineLen + 0.5 * mm, false),
            segPrism(context, H, "wSplP", at(L.hubFace + 0.5 * mm, z0, z0), -X, Y, SPLINE, L.splineLen + 0.5 * mm, false)]);
    var hex = [];
    var hexIn = [];
    for (var k = 0; k < 6; k += 1)
    {
        const a = (30 + k * 60) * degree;
        hex = append(hex, L.hexR * vector(cos(a), sin(a)));
        hexIn = append(hexIn, (L.hexR - 0.06 * mm) * vector(cos(a), sin(a)));
    }
    const shank = cylW(context, H + "shank", at(-L.clampX - L.nutL, z0, z0), at(L.clampX, z0, z0), L.axleR);
    polyPrism(context, H, "nut", at(-L.clampX - L.nutL, z0, z0), X, Y, hexIn, L.nutL);
    const head = cylW(context, H + "head", at(L.clampX, z0, z0), at(L.clampX + L.headL, z0, z0), L.hexAF / 2 - 0.05 * mm);
    unite(context, H + "axleU", [shank, qCreatedBy(H + "nutExt", EntityType.BODY), head]);
    const axlePt = at(z0, z0, z0);
    step("belt");
    segPrism(context, H, "beltL", at(-L.beltX1, z0, z0), X, Y, BELT, L.beltX1 - L.beltX0, true);
    const beltPt = at(-(L.beltX0 + L.beltX1) / 2, ya + L.rP45 - 1 * mm, za);
    step("spline pulley");
    revolveProfile(context, P, "spRev", xz, line(O, X),
            [vector(-L.hubFace, z0), vector(-L.hubFace, L.hubR), vector(-L.xCh, L.hubR),
             vector(-L.flangeIn, L.rFl15), vector(-(L.flangeIn + L.flCyl), L.rFl15),
             vector(-L.teethIn, L.rLip15), vector(-L.teethIn, L.rRoot15 - 1 * mm),
             vector(-L.teethOut, L.rRoot15 - 1 * mm), vector(-L.teethOut, L.rLip15),
             vector(-(L.outerFace - L.flCyl), L.rFl15), vector(-L.outerFace, L.rFl15), vector(-L.outerFace, z0)]);
    const spT = segPrism(context, P, "spTeeth", at(-L.teethOut, z0, z0), X, Y, HTD15, L.teethOut - L.teethIn, false);
    const spS = segPrism(context, P, "spStub", at(-L.hubFace - 0.2 * mm, z0, z0), X, Y, SPLINE, L.splineLen + 0.2 * mm, false);
    unite(context, P + "spU", [qCreatedBy(P + "spRevRev", EntityType.BODY), spT, spS]);
    const spPt = at(-(L.teethIn + L.teethOut) / 2, z0, (L.cbR + L.rRoot15) / 2);
    revolveProfile(context, P, "spCone", xz, line(O, X),
            [vector(-L.xCe, z0), vector(-L.xCe, L.bore15R), vector(-L.xCb, L.cbR),
             vector(-L.outerFace - 1 * mm, L.cbR), vector(-L.outerFace - 1 * mm, z0)]);
    cutFrom(context, P + "spCut", partAt(context, P, spPt), [
            qCreatedBy(P + "spConeRev", EntityType.BODY),
            cylW(context, P + "spBore", at(-(L.hubFace - L.splineLen) + 1 * mm, z0, z0), at(-L.xCe - 0.2 * mm, z0, z0), L.bore15R)]);
    step("drive pulley");
    revolveProfile(context, P, "dpRev", xzA, line(pa, X),
            [vector(-L.xHorn, z0), vector(-L.xHorn, L.padR), vector(-L.xPo, L.padR),
             vector(-L.flangeIn, L.padR + L.padCh), vector(-L.flangeIn, L.rFl45),
             vector(-(L.flangeIn + L.flCyl), L.rFl45), vector(-L.teethIn, L.rLip45),
             vector(-L.teethIn, L.rRoot45 - 1 * mm), vector(-L.teethOut, L.rRoot45 - 1 * mm),
             vector(-L.teethOut, L.rLip45), vector(-(L.outerFace - L.flCyl), L.rFl45),
             vector(-L.outerFace, L.rFl45), vector(-L.outerFace, z0)]);
    const dpT = segPrism(context, P, "dpTeeth", at(-L.teethOut, ya, za), X, Y, HTD45, L.teethOut - L.teethIn, false);
    var dpAdd = [qCreatedBy(P + "dpRevRev", EntityType.BODY), dpT];
    for (var i = 0; i < 4; i += 1)
    {
        const q = L.pinC * vector(cos(i * 90 * degree), sin(i * 90 * degree));
        dpAdd = append(dpAdd, cylW(context, P + ("dpPin" ~ i), at(-L.xHorn - 0.1 * mm, ya + q[0], za + q[1]),
                at(-(L.xHorn - L.pinL), ya + q[0], za + q[1]), L.pinR));
    }
    unite(context, P + "dpU", dpAdd);
    const dpPt = at(-(L.teethIn + L.teethOut) / 2, ya, za + 20 * mm);
    var dpCut = [cylW(context, P + "dpBoss", at(-L.xHorn + 0.1 * mm, ya, za), at(-L.flangeIn, ya, za), L.bossClrR)];
    for (var i = 0; i < 4; i += 1)
    {
        const q = L.pinC * vector(cos((45 + i * 90) * degree), sin((45 + i * 90) * degree));
        const y = ya + q[0];
        const z = za + q[1];
        dpCut = append(dpCut, cylW(context, P + ("dpCb" ~ i), at(-L.outerFace - 1 * mm, y, z), at(-L.xSeat, y, z), L.m2CbR));
        dpCut = concatenateArrays([dpCut, cbLayers(context, P + ("dpL" ~ i), -L.xSeat, y, z, L.m2CbR, L.m2R, vector(1, 0), 0 * mm, bl)]);
        dpCut = append(dpCut, cylW(context, P + ("dpM2" ~ i), at(-(L.xSeat - 2 * bl) - 0.01 * mm, y, z), at(-L.xHorn + 0.1 * mm, y, z), L.m2R));
    }
    cutFrom(context, P + "dpCut", partAt(context, P, dpPt), dpCut);
    step("case side");
    const fix = opt.fixture == true;
    var csBody = [boxW(context, P + "csPlate", at(-L.xPo, L.Yc0, -L.Zp), at(-L.caseHalf, fix ? L.Yfront : L.Ytab0, L.Zp)),
                  boxW(context, P + "csSkirt", at(-L.caseHalf - 0.01 * mm, L.Ysk0, -L.Zp), at(-L.skirtTop, L.Ytab0, L.Zp)),
                  boxW(context, P + "csRear", at(-L.xPo, L.Yc0, -L.Zp), at(-L.xRin, L.Yb - L.skClr, L.Zp))];
    if (fix)
        csBody = append(csBody, boxW(context, P + "csTab", at(-L.xPo, L.Yf + L.skClr, -L.Zp), at(-L.xTab, L.Yfront, L.Zp)));
    unite(context, P + "csU", csBody);
    const csPt = fix ? at(-L.xPo + 0.5 * mm, L.Ytab0 + 1 * mm, z0) : at(-L.xPo + 0.5 * mm, L.Ytab0 - 0.5 * mm, z0);
    cutFrom(context, P + "csPocket", partAt(context, P, csPt), [
            boxW(context, P + "csPk", at(-L.skirtTop + 1 * mm, L.Yb - L.skClr, -L.Zi), at(-L.caseHalf, L.Yf + L.skClr, L.Zi))]);
    var spc = [partAt(context, P, csPt)];
    var holes = [];
    var k2 = 0;
    for (var zc in [za, -za])
    {
        for (var sy in [1, -1])
        {
            for (var sz in [1, -1])
            {
                const y = L.Yhc + sy * L.holeHH;
                const z = zc + sz * L.holeHW;
                spc = append(spc, cylW(context, P + ("csSp" ~ k2), at(-L.caseHalf - 0.01 * mm, y, z), at(-L.spacerTop, y, z), L.spacerR));
                holes = append(holes, cylW(context, P + ("csCb" ~ k2), at(-L.xPo - 1 * mm, y, z), at(-L.m25Seat, y, z), L.m25CbR));
                const dv = vector(ya - y, za - z);
                const dist = norm(dv);
                const near = dist - L.m25CbR < L.reliefR + L.reliefCh + 1 * mm;
                const u = near ? dv / dist : vector(1, 0);
                const ext = near ? dist - L.reliefR + 0.5 * mm : 0 * mm;
                holes = concatenateArrays([holes, cbLayers(context, P + ("csL" ~ k2), -L.m25Seat, y, z, L.m25CbR, L.m25R, u, ext, bl)]);
                if (near)
                {
                    const nn = vector(-u[1], u[0]);
                    polyPrism(context, P, "csX" ~ k2, at(-L.xPo - 1 * mm, y, z), X, Y,
                              [-L.m25CbR * nn, ext * u - L.m25CbR * nn, ext * u + L.m25CbR * nn, L.m25CbR * nn],
                              1 * mm + L.xPo - L.m25Seat);
                    holes = append(holes, qCreatedBy(P + ("csX" ~ k2 ~ "Ext"), EntityType.BODY));
                }
                holes = append(holes, cylW(context, P + ("csH" ~ k2), at(-(L.m25Seat - 2 * bl) - 0.01 * mm, y, z), at(-L.spacerTop + 0.5 * mm, y, z), L.m25R));
                k2 += 1;
            }
        }
    }
    unite(context, P + "csSpU", spc);
    if (L.rcLeg > 0 * mm)
    {
        for (var sz in [1, -1])
        {
            polyPrism(context, P, "csRc" ~ (sz > 0 ? "p" : "n"), at(-L.xPo - 1 * mm, 0 * mm, 0 * mm), X, Y,
                      [vector(L.Yc0 - 1 * mm, sz * (L.Zp - L.rcLeg - 1 * mm)), vector(L.Yc0 + L.rcLeg + 1 * mm, sz * (L.Zp + 1 * mm)),
                       vector(L.Yc0 - 1 * mm, sz * (L.Zp + 1 * mm))], L.xPo - L.skirtTop + 2 * mm);
            holes = append(holes, qCreatedBy(P + ("csRc" ~ (sz > 0 ? "p" : "n") ~ "Ext"), EntityType.BODY));
        }
    }
    revolveProfile(context, P, "csRelCh", xzA, line(pa, X),
            [vector(-L.xPo - 0.1 * mm, z0), vector(-L.xPo - 0.1 * mm, L.reliefR + L.reliefCh + 0.1 * mm),
             vector(-L.xPo + L.reliefCh, L.reliefR), vector(-L.xPo + L.reliefCh, z0)]);
    holes = append(holes, qCreatedBy(P + "csRelChRev", EntityType.BODY));
    holes = append(holes, cylW(context, P + "csRel", at(-L.xPo - 1 * mm, ya, za), at(-L.caseHalf + 0.01 * mm, ya, za), L.reliefR));
    holes = append(holes, boxW(context, P + "csWire", at(-L.skirtTop + 1 * mm, L.wireY0, L.Zi - 0.5 * mm), at(-L.caseHalf, L.wireY1, L.Zp + 1 * mm)));
    holes = append(holes, boxW(context, P + "csChan", at(-L.xPo - 1 * mm, L.Yc0 - 1 * mm, -(L.tongueHalf + L.tongueClr)),
            at(-L.tongueTop, L.Yj1, L.tongueHalf + L.tongueClr)));
    const stadium = function(tag is string, x0, x1) returns array
    {
        return [cylW(context, P + (tag ~ "0"), at(x0, L.Yscr, z0), at(x1, L.Yscr, z0), L.holeR),
                cylW(context, P + (tag ~ "1"), at(x0, L.Yscr - L.travel, z0), at(x1, L.Yscr - L.travel, z0), L.holeR),
                boxW(context, P + (tag ~ "B"), at(x0, L.Yscr - L.travel, -L.holeR), at(x1, L.Yscr, L.holeR))];
    };
    const ys0 = L.Yscr - L.travel - L.holeR;
    const ys1 = L.Yscr + L.holeR;
    const chW = L.tongueHalf + L.tongueClr;
    const xNc = L.xN0 - L.nutSlotT;
    holes = concatenateArrays([holes,
            [boxW(context, P + "csChL1", at(-L.tongueTop - 0.01 * mm, ys0, -chW), at(-(L.tongueTop - bl), ys1, chW))],
            stadium("csSl", -(L.tongueTop - bl) - 0.01 * mm, -L.xN0 + 0.01 * mm),
            [boxW(context, P + "csNut", at(-L.xN0, L.Yc0 - 1 * mm, -L.nutHalfY),
                  at(-xNc, L.Yscr + L.nutCornerY + 0.2 * mm, L.nutHalfY)),
             boxW(context, P + "csNtL1", at(-xNc - 0.01 * mm, ys0, -L.nutHalfY), at(-(xNc - bl), ys1, L.nutHalfY))],
            stadium("csTp", -(xNc - bl) - 0.01 * mm, -(xNc - L.tipSlot))]);
    cutFrom(context, P + "csCut", partAt(context, P, csPt), holes);
    step("chainstay");
    const cyPt = at(-(L.xCo + L.xCi) / 2, (L.Yr0 + L.csBossR) / 2, z0);
    revolveProfile(context, P, "cyStub", xz, line(O, X),
            [vector(-L.xCi - 0.5 * mm, z0), vector(-L.xCi - 0.5 * mm, L.stubR), vector(-L.xA, L.stubR),
             vector(-L.xRel, L.rC0), vector(-L.xE, L.rC1), vector(-L.xE, z0)]);
    unite(context, P + "cyU", [
            boxW(context, P + "cyBed", at(-L.xCo, z0, -L.armHalf), at(-L.xCi, L.Yj1, L.armHalf)),
            cylW(context, P + "cyBoss", at(-L.xCo, z0, z0), at(-L.xCi, z0, z0), L.csBossR),
            boxW(context, P + "cyRiser", at(-L.xCi - 0.5 * mm, L.Yr0, -L.armHalf), at(-L.riserTop, L.Yj1, L.armHalf)),
            boxW(context, P + "cyTongue", at(-L.riserTop - 0.5 * mm, L.Yc0, -L.tongueHalf), at(-L.tongueEnd, L.Yj1, L.tongueHalf)),
            qCreatedBy(P + "cyStubRev", EntityType.BODY)]);
    polyPrism(context, P, "cyHex", at(-L.xCo - 1 * mm, z0, z0), X, Y, hex, L.xCo + 1 * mm - L.clampX);
    const ma = L.hexAF / 2;
    const mr = L.hexR + L.bedCh;
    revolveProfile(context, P, "cyMouth", xz, line(O, X),
            [vector(-L.xCo - 0.1 * mm, z0), vector(-L.xCo - 0.1 * mm, mr + 0.1 * mm),
             vector(-L.xCo + (mr - ma), ma), vector(-L.xCo + (mr - ma), z0)]);
    const ys = L.Yscr;
    const cbTop = L.xH + (L.headBoreR - L.headR);
    revolveProfile(context, P, "cyCsk", plane(at(z0, ys, z0), -Y, X), line(at(z0, ys, z0), X),
            [vector(-cbTop - 1 * mm, z0), vector(-cbTop - 1 * mm, L.headBoreR), vector(-cbTop, L.headBoreR),
             vector(-(L.xH - L.cskDepth), L.holeR), vector(-(L.xH - L.cskDepth), z0)]);
    cutFrom(context, P + "cyCut", partAt(context, P, cyPt), [
            qCreatedBy(P + "cyHexExt", EntityType.BODY), qCreatedBy(P + "cyMouthRev", EntityType.BODY),
            boxW(context, P + "cyB1", at(-L.clampX - 0.5 * mm, -ma, -L.csBoreR), at(-(L.clampX - bl), ma, L.csBoreR)),
            boxW(context, P + "cyB2", at(-(L.clampX - bl) - 0.5 * mm, -L.csBoreR, -L.csBoreR), at(-(L.clampX - 2 * bl), L.csBoreR, L.csBoreR)),
            cylW(context, P + "cyBore", at(-(L.clampX - 2 * bl) - 0.01 * mm, z0, z0), at(-L.xE + 1 * mm, z0, z0), L.csBoreR),
            cylW(context, P + "cyHead", at(-L.xCo - 1 * mm, ys, z0), at(-cbTop, ys, z0), L.headBoreR),
            qCreatedBy(P + "cyCskRev", EntityType.BODY),
            cylW(context, P + "cyHole", at(-(L.xH - L.cskDepth) - 0.01 * mm, ys, z0), at(-L.tongueTop + 1 * mm, ys, z0), L.holeR)]);
    const cy = partAt(context, P, cyPt);
    const edgeAt = function(p is Vector) returns Query
    {
        return qContainsPoint(qOwnedByBody(cy, EntityType.EDGE), p);
    };
    const c1 = L.cyFront;
    const c2 = L.cyRiserCh;
    opChamfer(context, P + "cyCh1", { "entities" : edgeAt(at(-L.xCo, L.Yj1, z0)),
            "chamferType" : ChamferType.EQUAL_OFFSETS, "width" : c1 });
    opChamfer(context, P + "cyCh2", { "entities" : qUnion([edgeAt(at(-L.riserTop, L.Yr0, z0)), edgeAt(at(-L.xCi, L.Yr0, z0))]),
            "chamferType" : ChamferType.EQUAL_OFFSETS, "width" : c2 });
    opFillet(context, P + "cyFil", { "entities" : qUnion([
            edgeAt(at(-L.xCo, L.Yj1 - c1, z0)), edgeAt(at(-(L.xCo - c1), L.Yj1, z0)),
            edgeAt(at(-(L.riserTop + c2), L.Yr0, z0)), edgeAt(at(-L.riserTop, L.Yr0 + c2, z0)),
            edgeAt(at(-L.xCi, L.Yr0 - c2, z0)), edgeAt(at(-(L.xCi - c2), L.Yr0, z0))]),
            "radius" : L.cyFillet });
    opChamfer(context, P + "cyCh3", { "entities" : edgeAt(at(-L.xCo, -L.csBossR, z0)),
            "chamferType" : ChamferType.EQUAL_OFFSETS, "width" : L.cyEdgeCh, "tangentPropagation" : true });
    step("right side");
    const rot = rotationAround(line(O, Y), 180 * degree);
    const horn = at(-L.xHorn + 0.5 * mm, ya + 5 * mm, za);
    const svPt = at(z0, L.Yhc, za);
    const lefts = [spPt, dpPt, csPt, cyPt];
    var lq = [];
    for (var p in lefts)
        lq = append(lq, partAt(context, P, p));
    opPattern(context, P + "rot", { "entities" : qUnion(lq), "transforms" : [rot], "instanceNames" : ["R"] });
    if (opt.rail == true)
    {
        step("rails");
        const R  = RAIL;
        const RL = id + "rail";
        const RB = RL + "b";       // the rail's bodies (several: the tabs and the stub stand apart from the band)
        const RC = RL + "c";       // its cutters
        for (var k = 0; k < size(R.pieces); k += 1)
        {
            var pts = [];
            for (var q in R.pieces[k])
                pts = append(pts, vector(q[0], q[1]) * mm);
            polyPrism(context, RB, "pc" ~ k, at(-L.xPo, z0, z0), X, Y, pts, L.xPo - R.innerX * mm);
        }
        for (var k = 0; k < size(R.stops); k += 1)
        {
            const sb = R.stops[k];
            boxW(context, RB + ("stop" ~ k), at(-sb.x[1] * mm, sb.y[0] * mm, sb.z[0] * mm),
                 at(-sb.x[0] * mm, sb.y[1] * mm, sb.z[1] * mm));
        }
        boxW(context, RB + "tab", at(-L.xPo, L.Ytab0 - 0.5 * mm, (R.ahZ[0] - R.ahTabHalf) * mm),
             at(-R.ahTabIn * mm, R.ahY1 * mm, (R.ahZ[size(R.ahZ) - 1] + R.ahTabHalf) * mm));
        const railL = qBodyType(qCreatedBy(RB, EntityType.BODY), BodyType.SOLID);
        var cp = [];
        for (var q in R.casePoly)
            cp = append(cp, vector(q[0], q[1]) * mm);
        polyPrism(context, RC, "fp", at(-L.caseHalf, z0, z0), X, Y, cp, L.caseHalf - R.innerX * mm + 0.1 * mm);
        var rc = [qCreatedBy(RC + "fpExt", EntityType.BODY)];
        const rT = R.ahHoleR * mm;
        for (var k = 0; k < size(R.ahZ); k += 1)
        {
            step("rail ahrs " ~ k);
            const z = R.ahZ[k] * mm;
            const x = -R.ahX * mm;
            rc = append(rc, cylW(context, RC + ("ahH" ~ k), at(x, R.ahHoleY0 * mm, z), at(x, R.ahY1 * mm + 0.1 * mm, z), rT));
            polyPrism(context, RC, "ahT" ~ k, at(x, R.ahHoleY0 * mm, z), Y, Z,
                      [vector(0 * mm, 0 * mm), vector(rT / sqrt(2), rT / sqrt(2)), vector(0 * mm, rT * sqrt(2)),
                       vector(-rT / sqrt(2), rT / sqrt(2))], (R.ahY1 - R.ahHoleY0) * mm + 0.1 * mm);
            rc = append(rc, qCreatedBy(RC + ("ahT" ~ k ~ "Ext"), EntityType.BODY));
            rc = append(rc, boxW(context, RC + ("ahN" ~ k), at(-R.ahNutOut * mm, R.ahN0 * mm, z - R.ahNutHalf * mm),
                                 at(-R.ahTabIn * mm + 1 * mm, R.ahN1 * mm, z + R.ahNutHalf * mm)));
        }
        const railJoint = function(S is Id, k is number, j is map) returns array
        {
            const h = at(-L.xPo, j.head[0] * mm, j.head[1] * mm);
            const J = S + ("j" ~ k);
            screwJointGeometry(context, J, coordSystem(h, vector(0, j.dir[0], j.dir[1]), X), mergeMaps(SCREW_OPT, {
                    "nutDepth" : FIXTURE.joint_plate + 2 * mm, "headPocket" : 0 * mm, "bothWays" : false,
                    "nutSlotLength" : 1 * mm }));
            opDeleteBodies(context, S + ("jd" ~ k), { "entities" : qCreatedBy(J + "slotExt", EntityType.BODY) });
            const jp = h + FIXTURE.joint_plate * X;
            const g = ridgeSolid(context, S + ("g" ~ k), jp + j.gMid * mm * Y, Y, -X, FIXTURE.ridge_height,
                                 FIXTURE.ridge_clearance, j.gHalf * mm);
            opTransform(context, S + ("gx" ~ k), { "bodies" : g,
                    "transform" : rotationAround(line(jp, X), atan2(j.dir[1], j.dir[0])) });
            return [qCreatedBy(J + "screwRev", EntityType.BODY), g];
        };
        for (var k = 0; k < size(R.joints); k += 1)
        {
            step("rail joint " ~ k);
            rc = concatenateArrays([rc, railJoint(RC, k, R.joints[k])]);
        }
        const bl = FIXTURE.bridge_layer;
        const rH = SCREW_OPT.holeDia / 2;
        const sw = SCREW_OPT.nutSlotWidth;
        for (var k = 0; k < size(R.cage); k += 1)
        {
            step("rail cage " ~ k);
            const c = R.cage[k];
            const h = at(-L.xPo - FIXTURE.joint_plate, c.head[0] * mm, c.head[1] * mm);
            const cs = coordSystem(h, vector(0, c.slot[0], c.slot[1]), X);
            const J = RC + ("cgJ" ~ k);
            screwJointGeometry(context, J, cs, mergeMaps(SCREW_OPT, {
                    "nutDepth" : R.cageNd * mm, "headPocket" : 0 * mm, "bothWays" : false,
                    "nutSlotLength" : c.slotLen * mm }));
            rc = append(rc, qCreatedBy(J + "screwRev", EntityType.BODY));
            rc = append(rc, qCreatedBy(J + "slotExt", EntityType.BODY));
            const d0 = R.cageNd * mm + SCREW_OPT.nutSlotThickness;
            rc = append(rc, boxIn(context, RC + ("cgB" ~ k), cs, vector(-rH, -sw / 2, d0), vector(rH, sw / 2, d0 + bl)));
            rc = append(rc, boxIn(context, RC + ("cgC" ~ k), cs, vector(-rH, -rH, d0 + bl), vector(rH, rH, d0 + 2 * bl)));
        }
        step("rail cut");
        cutFrom(context, RC + "cut", railL, rc);
        step("rail mirror");
        opPattern(context, RL + "mir", { "entities" : railL,
                "transforms" : [mirrorAcross(plane(O, X))], "instanceNames" : ["R"] });
        opBoolean(context, RL + "uL", { "tools" : qUnion([partAt(context, P, csPt), railL]),
                "operationType" : BooleanOperationType.UNION });
        opBoolean(context, RL + "uR", { "tools" : qUnion([partAt(context, P, flip(csPt)),
                                                          qBodyType(qCreatedBy(RL + "mir", EntityType.BODY), BodyType.SOLID)]),
                "operationType" : BooleanOperationType.UNION });
        if (size(R.leftPieces) > 0 || size(R.leftJoints) > 0)
        {
            step("rail left");
            const RE = RL + "e";
            const RF = RL + "f";
            for (var k = 0; k < size(R.leftPieces); k += 1)
            {
                var pts = [];
                for (var q in R.leftPieces[k])
                    pts = append(pts, vector(q[0], q[1]) * mm);
                polyPrism(context, RE, "pc" ~ k, at(-L.xPo, z0, z0), X, Y, pts, L.xPo - R.innerX * mm);
            }
            opBoolean(context, RL + "eU", { "tools" : qUnion([partAt(context, P, csPt),
                                                              qBodyType(qCreatedBy(RE, EntityType.BODY), BodyType.SOLID)]),
                    "operationType" : BooleanOperationType.UNION });
            var ec = [];
            for (var k = 0; k < size(R.leftJoints); k += 1)
                ec = concatenateArrays([ec, railJoint(RF, k, R.leftJoints[k])]);
            if (size(ec) > 0)
                cutFrom(context, RL + "fcut", partAt(context, P, csPt), ec);
        }
    }
    const fxPt = at(z0, L.Yfront - 1 * mm, z0);
    if (fix)
    {
        step("fixture");
        boxW(context, P + "fixture", at(-L.xTab, L.Ytab0 + L.fixGap, -L.Zp), at(L.xTab, L.Yfront, L.Zp));
        var jn = 0;
        for (var sg in [1, -1])
        {
            const cq = partAt(context, P, sg > 0 ? csPt : flip(csPt));
            for (var y in [L.yF0, L.yF1])
            {
                screwJoint(context, P + ("jF" ~ jn), at(-sg * L.xPo, y, sg * L.zF), sg * X, sg * Z, 0 * mm,
                           cq, partAt(context, P, fxPt), false, Y, L.ridgeHalf, sg * X, Y);
                jn += 1;
            }
        }
    }
    const HR = id + "hwR";
    opPattern(context, HR, { "entities" : qUnion([sv.caseQ, sv.horn, partAt(context, H, beltPt)]),
            "transforms" : [rot], "instanceNames" : ["R"] });
    const pulleyC = color(0.93, 0.56, 0.20);
    const frameC  = color(0.62, 0.64, 0.68);
    var parts = [["spline pulley L", spPt, "X+", X, "move", pulleyC], ["spline pulley R", flip(spPt), "X-", -X, "move", pulleyC],
                 ["drive pulley L", dpPt, "X+", X, "fixed", pulleyC], ["drive pulley R", flip(dpPt), "X-", -X, "fixed", pulleyC],
                 ["case side L", csPt, "X+", X, "fixed", frameC], ["case side R", flip(csPt), "X-", -X, "fixed", frameC],
                 ["chainstay L", cyPt, "X+", X, "move", frameC], ["chainstay R", flip(cyPt), "X-", -X, "move", frameC]];
    if (fix)
        parts = append(parts, ["fixture mock", fxPt, "Y+", Y, "fixed", color(0.45, 0.62, 0.45)]);
    var stages = { "fixed" : [], "move" : [] };
    var prints = [];
    for (var n in parts)
    {
        const q = partAt(context, P, n[1]);
        dress(context, q, n[0] ~ " [print " ~ n[2] ~ "]", n[5],
              "Print with the " ~ n[2] ~ " side facing UP (module frame).");
        stages[n[4]] = append(stages[n[4]], q);
        prints = append(prints, [n[0], q, n[3]]);
    }
    const dark = color(0.16, 0.16, 0.18);
    const hw = [[svPt, "XC430 A", dark, "fixed"], [flip(svPt), "XC430 B", dark, "fixed"],
                [horn, "XC430 horn A", color(0.3, 0.3, 0.32), "fixed"], [flip(horn), "XC430 horn B", color(0.3, 0.3, 0.32), "fixed"],
                [beltPt, "belt L", color(0.10, 0.10, 0.11), "belt"], [flip(beltPt), "belt R", color(0.10, 0.10, 0.11), "belt"],
                [wheelPt, "rear wheel", color(0.36, 0.56, 0.76), "move"], [axlePt, "M5 axle", color(0.55, 0.57, 0.60), "move"]];
    var belts = [];
    for (var h in hw)
    {
        const q = qContainsPoint(qBodyType(qUnion([qCreatedBy(H, EntityType.BODY), qCreatedBy(HR, EntityType.BODY)]),
                                           BodyType.SOLID), h[0]);
        dress(context, q, h[1], h[2], "Hardware envelope, not a print.");
        if (h[3] == "belt")
            belts = append(belts, q);
        else
            stages[h[3]] = append(stages[h[3]], q);
    }
    if (opt.shift != undefined && opt.shift != 0 * mm)
        opTransform(context, id + "shift", { "bodies" : qUnion(stages["move"]), "transform" : transform(-opt.shift * Y) });
    opMateConnector(context, id + "axleMC", { "coordSystem" : coordSystem(O, X, Z), "owner" : partAt(context, P, fix ? fxPt : csPt) });
    return { "stages" : stages, "belts" : belts, "prints" : prints };
}

export const RG = { "yH" : -29.7, "zC" : 25.0913896732, "servoX" : 1, "ucAdd" : [{ "k" : "box", "lo" : [-14.05, -59.7, 9.5913896732], "hi" : [14.05, -37.4, 38.6413896732] }, { "k" : "prismX", "pts" : [[-49.7, 38.1413896732], [-37.4, 38.1413896732], [-37.4, 47.5152989094], [-40.8260907637, 47.5152989094], [-49.7, 38.6413896732]], "x0" : -5.25, "x1" : 5.25 }], "ucCut" : [{ "k" : "box", "lo" : [-12.45, -44.2, 9.4913896732], "hi" : [12.45, -36.4, 37.0413896732] }, { "k" : "box", "lo" : [-10.05, -55.7, 9.4913896732], "hi" : [10.05, -43.2, 34.6413896732] }, { "k" : "prismX", "pts" : [[-55.7, 10.5913896732], [-44.2, 10.5913896732], [-42.107171153, 16.3413896732], [-44.2, 22.0913896732], [-55.7, 22.0913896732]], "x0" : -15.05, "x1" : 15.05 }, { "k" : "box", "lo" : [6, -60.7, 10.5913896732], "hi" : [15.05, -55.6, 22.0913896732] }, { "k" : "box", "lo" : [-15.05, -60.7, 10.5913896732], "hi" : [-6, -55.6, 22.0913896732] }], "ucShellPt" : [-13.55, -59.2, 4.4163896732], "lowerNearPins" : 1, "joints" : [{ "tag" : "jF", "ridge" : true, "through" : false, "slotLen" : 6, "head" : [0, 18.5, 53.1755186068], "zIn" : [0, 0, -1], "slot" : [0, 1, 0], "pocket" : 5.4, "csk" : "bridge", "nut" : "bulkheadF", "ridgeOnNut" : true, "ridgeDir" : [1, 0, 0], "ridgeHalf" : 11.35, "cskUp" : [0, 0, 1], "nutUp" : [0, -1, 0] }, { "tag" : "jR", "ridge" : true, "through" : false, "slotLen" : 6, "head" : [0, -18.5, 53.1755186068], "zIn" : [0, 0, -1], "slot" : [0, 1, 0], "pocket" : 5.4, "csk" : "bridge", "nut" : "lowercase", "ridgeOnNut" : true, "ridgeDir" : [1, 0, 0], "ridgeHalf" : 11.35, "cskUp" : [0, 0, 1], "nutUp" : [0, -1, 0] }, { "tag" : "jUL", "ridge" : false, "through" : false, "slotLen" : 5.9239092363, "head" : [0, -42, 42.5913896732], "zIn" : [0, 1, 0], "slot" : [0, 0, 1], "pocket" : 8.7, "csk" : "uppercase", "nut" : "lowercase", "ridgeOnNut" : false, "ridgeDir" : [1, 0, 0], "ridgeHalf" : 0, "cskUp" : [0, 1, 0], "nutUp" : [0, -1, 0] }, { "ridgeMid" : -1.75, "ridgeClipZ" : 48.5755186068, "tag" : "jS0", "ridge" : true, "through" : false, "slotLen" : 6, "head" : [20.6, 0, 53.5755186068], "zIn" : [-1, 0, 0], "slot" : [0, 0, 1], "pocket" : 0, "csk" : "railR", "nut" : "bridge", "ridgeOnNut" : true, "ridgeDir" : [0, 0, 1], "ridgeHalf" : 4.75, "cskUp" : [-1, 0, 0], "nutUp" : [0, 0, 1] }, { "ridgeMid" : -1.75, "ridgeClipZ" : 48.5755186068, "tag" : "jS1", "ridge" : true, "through" : false, "slotLen" : 6, "head" : [-20.6, 0, 53.5755186068], "zIn" : [1, 0, 0], "slot" : [0, 0, 1], "pocket" : 0, "csk" : "railL", "nut" : "bridge", "ridgeOnNut" : true, "ridgeDir" : [0, 0, 1], "ridgeHalf" : 4.75, "cskUp" : [1, 0, 0], "nutUp" : [0, 0, 1] }, { "tag" : "jWR", "ridge" : false, "through" : false, "slotLen" : 11, "head" : [37.7223675386, -17.5, 26.1802141396], "zIn" : [0, 1, 0], "slot" : [0.253983824, 0, 0.9672084662], "pocket" : 1.4, "csk" : "wingR", "nut" : "rockerR", "ridgeOnNut" : false, "ridgeDir" : [0.253983824, 0, 0.9672084662], "ridgeHalf" : 0, "cskUp" : [0, -1, 0], "nutUp" : [0, 1, 0] }, { "tag" : "jWL", "ridge" : false, "through" : false, "slotLen" : 11, "head" : [-37.7223675386, 17.5, 26.1802141396], "zIn" : [0, -1, 0], "slot" : [-0.253983824, 0, 0.9672084662], "pocket" : 1.4, "csk" : "wingL", "nut" : "rockerL", "ridgeOnNut" : false, "ridgeDir" : [-0.253983824, 0, 0.9672084662], "ridgeHalf" : 0, "cskUp" : [0, 1, 0], "nutUp" : [0, -1, 0] }, { "tag" : "jKR", "ridge" : false, "through" : false, "slotLen" : 11, "head" : [37.7223675386, 10.9, 26.1802141396], "zIn" : [0, -1, 0], "slot" : [0.253983824, 0, 0.9672084662], "pocket" : 1.4, "csk" : "wingKR", "nut" : "knuckleR", "ridgeOnNut" : false, "ridgeDir" : [0.253983824, 0, 0.9672084662], "ridgeHalf" : 0, "cskUp" : [0, 1, 0], "nutUp" : [0, -1, 0] }, { "tag" : "jKL", "ridge" : false, "through" : false, "slotLen" : 11, "head" : [-37.7223675386, -10.9, 26.1802141396], "zIn" : [0, 1, 0], "slot" : [-0.253983824, 0, 0.9672084662], "pocket" : 1.4, "csk" : "wingKL", "nut" : "knuckleL", "ridgeOnNut" : false, "ridgeDir" : [-0.253983824, 0, 0.9672084662], "ridgeHalf" : 0, "cskUp" : [0, -1, 0], "nutUp" : [0, 1, 0] }] };

export const RG_PARTS = [{ "slug" : "crankR", "name" : "half crank R", "group" : "crank", "up" : [0, -1, 0], "add" : [{ "k" : "slot", "pts" : [[-8.4239440376, 26.2259234327], [-6.4418395423, 40.9431032465], [6.4418395758, 40.9431032329], [8.42394404, 26.2259234149]], "mids" : [[0.0000000227, 46.5755186068], [-0.000000009, 16.5913896732]], "a" : [0, 25.0913896732], "b" : [0.0000000158, 40.0755186068], "ra" : 8.5, "rb" : 6.5, "y0" : -12.9, "y1" : -6.9 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -13.3, "y1" : -12.9, "r" : 8.2 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -24, "y1" : -12.9, "r" : 7 }, { "k" : "prism", "pts" : [[3.0405591591, 26.4348925574], [5.6542915954, 29.0486249937], [3.9572353206, 30.7456812685], [1.3435028843, 28.1319488323]], "y0" : -27.3, "y1" : -24 }, { "k" : "prism", "pts" : [[-1.3435028843, 28.1319488323], [-3.9572353206, 30.7456812685], [-5.6542915954, 29.0486249937], [-3.0405591591, 26.4348925574]], "y0" : -27.3, "y1" : -24 }, { "k" : "prism", "pts" : [[-3.0405591591, 23.7478867889], [-5.6542915954, 21.1341543526], [-3.9572353206, 19.4370980778], [-1.3435028843, 22.050830514]], "y0" : -27.3, "y1" : -24 }, { "k" : "prism", "pts" : [[1.3435028843, 22.050830514], [3.9572353206, 19.4370980778], [5.6542915954, 21.1341543526], [3.0405591591, 23.7478867889]], "y0" : -27.3, "y1" : -24 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -13.9, "y1" : -5.9, "r" : 3 }], "anchor" : [0.0000000079, -9.9, 32.58345414], "note" : "Print the web face down. Crankpin hole: ream to press. The lugs drive it from the horn hub.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "couplerR", "name" : "coupler R", "group" : "cR", "up" : [0, -1, 0], "add" : [{ "k" : "slot", "pts" : [[4.8045911379, 44.4534066992], [30.4703828311, 16.2860272904], [20.861200587, 7.5302511055], [-4.8045911062, 35.6976305143]], "mids" : [[30.0436798015, 7.1035480759], [-4.3778880766, 44.8801097288]], "a" : [0.0000000158, 40.0755186068], "b" : [25.665791709, 11.908139198], "ra" : 6.5, "rb" : 6.5, "y0" : -6.3, "y1" : -0.3 }, { "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -6.7, "y1" : -6.3, "r" : 5.5 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -6.7, "y1" : -6.3, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -7.3, "y1" : 0.7, "r" : 3 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -7.3, "y1" : 0.7, "r" : 3 }], "anchor" : [12.8328958624, -3.3, 25.9918289024], "note" : "38.11 mm centres. Both holes: ream to run. Print the bossed face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "rockerR", "name" : "rocker R", "group" : "wR", "up" : [0, 1, 0], "add" : [{ "k" : "slot", "pts" : [[-2.4897011732, 4.9042214538], [22.7234175952, 17.7040372798], [28.1913762917, 5.9188635727], [2.1370331084, -5.067848606]], "mids" : [[31.5620624122, 14.6438277393], [-4.9891521334, -2.3148133811]], "a" : [0, 0], "b" : [25.665791709, 11.908139198], "ra" : 5.5, "rb" : 6.5, "y0" : -12.9, "y1" : -6.9 }, { "k" : "slot", "pts" : [[21.1779496091, 16.6101891005], [34.615399931, 29.4354794568], [41.4510695977, 23.6609323065], [31.0516946833, 8.2691765502]], "mids" : [[40.6263281323, 29.6178023742], [21.4711819626, 6.9427339703]], "a" : [25.665791709, 11.908139198], "b" : [37.7223675386, 26.1802141396], "ra" : 6.5, "rb" : 4.5, "y0" : -12.9, "y1" : -6.9 }, { "k" : "prism", "pts" : [[40.0185716299, 15.2382103574], [45.0982481096, 34.582379682], [35.4261634473, 37.1222179219], [30.3464869676, 17.7780485972]], "y0" : -12.9, "y1" : -6.9 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : -6.9, "y1" : -6.5, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -13.9, "y1" : -5.9, "r" : 3 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -13.9, "y1" : -5.9, "r" : 3 }], "anchor" : [12.8328958545, -9.9, 5.954069599], "note" : "One plate in the web plane: hub, arm, ear and the wing's boss. The 6-32's nut in the boss, slid in from its top end; the screw from the wing's tab beyond. Its inner face's ring at the rod bears on the other wing's knuckle. Rod: ream to run; pin: to press. Print the inner face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "knuckleR", "name" : "knuckle R", "group" : "wR", "up" : [0, -1, 0], "add" : [{ "k" : "slot", "pts" : [[-2.4897011732, 4.9042214538], [22.7234175952, 17.7040372798], [28.1913762917, 5.9188635727], [2.1370331084, -5.067848606]], "mids" : [[31.5620624122, 14.6438277393], [-4.9891521334, -2.3148133811]], "a" : [0, 0], "b" : [25.665791709, 11.908139198], "ra" : 5.5, "rb" : 6.5, "y0" : 0.3, "y1" : 6.3 }, { "k" : "slot", "pts" : [[21.1779496091, 16.6101891005], [34.615399931, 29.4354794568], [41.4510695977, 23.6609323065], [31.0516946833, 8.2691765502]], "mids" : [[40.6263281323, 29.6178023742], [21.4711819626, 6.9427339703]], "a" : [25.665791709, 11.908139198], "b" : [37.7223675386, 26.1802141396], "ra" : 6.5, "rb" : 4.5, "y0" : 0.3, "y1" : 6.3 }, { "k" : "prism", "pts" : [[40.0185716299, 15.2382103574], [45.0982481096, 34.582379682], [35.4261634473, 37.1222179219], [30.3464869676, 17.7780485972]], "y0" : 0.3, "y1" : 6.3 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -0.1, "y1" : 0.3, "r" : 5.5 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : -0.1, "y1" : 0.3, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -0.7, "y1" : 7.3, "r" : 3 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -0.7, "y1" : 7.3, "r" : 3 }], "anchor" : [12.8328958545, 3.3, 5.954069599], "note" : "The wing's second support, in coupler L's layer: the rocker's outline, its pin through rocker, coupler and knuckle. The 6-32's nut in the boss, slid in from its top end; the screw from the wing's tab beyond. Its inner face's ring at the pin bears on its own coupler across the mid-plane. Rod: ream to run; pin: to press. Print the inner face up. Knuckle R only: a second ring, at the rod, on knuckle L.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "wingR", "name" : "wing R stub (rocker)", "group" : "wR", "up" : [0, -1, 0], "add" : [{ "k" : "prism", "pts" : [[40.0185716299, 15.2382103574], [45.0982481096, 34.582379682], [35.4261634473, 37.1222179219], [30.3464869676, 17.7780485972]], "y0" : -18.9, "y1" : -12.9 }], "cut" : [], "anchor" : [39.3053469877, -15.9, 16.4594026476], "note" : "STUB: the rocker-side tab block only. A 6-32 along Y from each tab into the rocker / knuckle's nut: one from each side. Print the outer (-Y) face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "wingKR", "name" : "wing R stub (knuckle)", "group" : "wR", "up" : [0, 1, 0], "add" : [{ "k" : "prism", "pts" : [[40.0185716299, 15.2382103574], [45.0982481096, 34.582379682], [35.4261634473, 37.1222179219], [30.3464869676, 17.7780485972]], "y0" : 6.3, "y1" : 12.3 }], "cut" : [], "anchor" : [39.3053469877, 9.3, 16.4594026476], "note" : "STUB: the knuckle-side tab block only. Print the outer (+Y) face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "bladeR", "name" : "wing R blade", "group" : "wR", "up" : undefined, "add" : [{ "k" : "prism", "pts" : [[63.9, 106.2], [38.5, 9.4], [15.69, -18.88], [19.57, -21.96], [43.3, 8.2], [44.5543599041, 9.8199118401], [45.7769460244, 11.4507698023], [46.9677592875, 13.0925764726], [48.12680062, 14.7453344373], [49.2540709485, 16.4090462825], [50.3495711999, 18.0837145944], [51.4133023005, 19.7693419591], [52.4452651772, 21.4659309627], [53.4454607566, 23.1734841915], [54.4138899652, 24.8920042315], [55.3505537298, 26.6214936689], [56.255452977, 28.36195509], [57.1285886334, 30.1133910808], [57.9699616256, 31.8758042274], [58.7795728804, 33.6491971161], [59.5574233244, 35.433572333], [60.3035138841, 37.2289324643], [61.0178454863, 39.0352800961], [61.7004190576, 40.8526178145], [62.3512355246, 42.6809482058], [62.970295814, 44.520273856], [63.5576008524, 46.3705973513], [64.1131515665, 48.231921278], [64.6369488829, 50.104248222], [65.1289937283, 51.9875807697], [65.5892870292, 53.8819215071], [66.0178297124, 55.7872730204], [66.4146227044, 57.7036378957], [66.779666932, 59.6310187193], [67.1129633217, 61.5694180772], [67.4145128003, 63.5188385556], [67.6843162943, 65.4792827407], [67.9223747304, 67.4507532186], [68.1286890353, 69.4332525755], [68.3032601355, 71.4267833975], [68.4460889578, 73.4313482708], [68.5571764287, 75.4469497816], [68.636523475, 77.4735905159], [68.6841310232, 79.51127306], [68.7, 81.56], [68.7, 104.9]], "y0" : -61.25, "y1" : 20.75, "segs" : [[[63.9, 106.2], [38.5, 9.4]], [[38.5, 9.4], [15.69, -18.88]], [[15.69, -18.88], [19.57, -21.96]], [[19.57, -21.96], [43.3, 8.2]], [[43.3, 8.2], [60.2366280657, 29.725861882], [68.7, 54.17], [68.7, 81.56]], [[68.7, 81.56], [68.7, 104.9]], [[68.7, 104.9], [63.9, 106.2]]] }], "cut" : [], "anchor" : [52.1610683515, -20.25, 57.5413990477], "note" : "DUMMY: the blade's whole external shape (config/swing_explore/blade_v3.yaml). How it holds the stubs: open.", "custom" : false, "mock" : false, "color" : [0.35, 0.55, 0.85], "kind" : "dummy" }, { "slug" : "rpinR", "name" : "rocker pin R", "group" : "wR", "up" : undefined, "add" : [{ "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -12.9, "y1" : 6.3, "r" : 3 }], "cut" : [], "anchor" : [25.665791709, -3.3, 11.908139198], "note" : "Rod stock: pressed in the rocker and the knuckle, runs in the coupler between them.", "custom" : false, "mock" : false, "color" : [0.3, 0.3, 0.32], "kind" : "steel" }, { "slug" : "crankL", "name" : "half crank L", "group" : "crank", "up" : [0, 1, 0], "add" : [{ "k" : "slot", "pts" : [[8.4239440376, 26.2259234327], [6.4418395423, 40.9431032465], [-6.4418395758, 40.9431032329], [-8.42394404, 26.2259234149]], "mids" : [[-0.0000000227, 46.5755186068], [0.000000009, 16.5913896732]], "a" : [0, 25.0913896732], "b" : [-0.0000000158, 40.0755186068], "ra" : 8.5, "rb" : 6.5, "y0" : 6.9, "y1" : 12.9 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 12.9, "y1" : 13.3, "r" : 8.2 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 12.9, "y1" : 24, "r" : 7 }], "cut" : [{ "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : 5.9, "y1" : 13.9, "r" : 3 }], "anchor" : [-0.0000000079, 9.9, 32.58345414], "note" : "Print the web face down. Crankpin hole: ream to press. No lugs: it only supports.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "couplerL", "name" : "coupler L", "group" : "cL", "up" : [0, 1, 0], "add" : [{ "k" : "slot", "pts" : [[-4.8045911379, 44.4534066992], [-30.4703828311, 16.2860272904], [-20.861200587, 7.5302511055], [4.8045911062, 35.6976305143]], "mids" : [[-30.0436798015, 7.1035480759], [4.3778880766, 44.8801097288]], "a" : [-0.0000000158, 40.0755186068], "b" : [-25.665791709, 11.908139198], "ra" : 6.5, "rb" : 6.5, "y0" : 0.3, "y1" : 6.3 }, { "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : 6.3, "y1" : 6.7, "r" : 5.5 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : 6.3, "y1" : 6.7, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : -0.7, "y1" : 7.3, "r" : 3 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -0.7, "y1" : 7.3, "r" : 3 }], "anchor" : [-12.8328958624, 3.3, 25.9918289024], "note" : "38.11 mm centres. Both holes: ream to run. Print the bossed face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "rockerL", "name" : "rocker L", "group" : "wL", "up" : [0, -1, 0], "add" : [{ "k" : "slot", "pts" : [[2.4897011732, 4.9042214538], [-22.7234175952, 17.7040372798], [-28.1913762917, 5.9188635727], [-2.1370331084, -5.067848606]], "mids" : [[-31.5620624122, 14.6438277393], [4.9891521334, -2.3148133811]], "a" : [0, 0], "b" : [-25.665791709, 11.908139198], "ra" : 5.5, "rb" : 6.5, "y0" : 6.9, "y1" : 12.9 }, { "k" : "slot", "pts" : [[-21.1779496091, 16.6101891005], [-34.615399931, 29.4354794568], [-41.4510695977, 23.6609323065], [-31.0516946833, 8.2691765502]], "mids" : [[-40.6263281323, 29.6178023742], [-21.4711819626, 6.9427339703]], "a" : [-25.665791709, 11.908139198], "b" : [-37.7223675386, 26.1802141396], "ra" : 6.5, "rb" : 4.5, "y0" : 6.9, "y1" : 12.9 }, { "k" : "prism", "pts" : [[-30.3464869676, 17.7780485972], [-35.4261634473, 37.1222179219], [-45.0982481096, 34.582379682], [-40.0185716299, 15.2382103574]], "y0" : 6.9, "y1" : 12.9 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : 6.5, "y1" : 6.9, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 5.9, "y1" : 13.9, "r" : 3 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : 5.9, "y1" : 13.9, "r" : 3 }], "anchor" : [-12.8328958545, 9.9, 5.954069599], "note" : "One plate in the web plane: hub, arm, ear and the wing's boss. The 6-32's nut in the boss, slid in from its top end; the screw from the wing's tab beyond. Its inner face's ring at the rod bears on the other wing's knuckle. Rod: ream to run; pin: to press. Print the inner face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "knuckleL", "name" : "knuckle L", "group" : "wL", "up" : [0, 1, 0], "add" : [{ "k" : "slot", "pts" : [[2.4897011732, 4.9042214538], [-22.7234175952, 17.7040372798], [-28.1913762917, 5.9188635727], [-2.1370331084, -5.067848606]], "mids" : [[-31.5620624122, 14.6438277393], [4.9891521334, -2.3148133811]], "a" : [0, 0], "b" : [-25.665791709, 11.908139198], "ra" : 5.5, "rb" : 6.5, "y0" : -6.3, "y1" : -0.3 }, { "k" : "slot", "pts" : [[-21.1779496091, 16.6101891005], [-34.615399931, 29.4354794568], [-41.4510695977, 23.6609323065], [-31.0516946833, 8.2691765502]], "mids" : [[-40.6263281323, 29.6178023742], [-21.4711819626, 6.9427339703]], "a" : [-25.665791709, 11.908139198], "b" : [-37.7223675386, 26.1802141396], "ra" : 6.5, "rb" : 4.5, "y0" : -6.3, "y1" : -0.3 }, { "k" : "prism", "pts" : [[-30.3464869676, 17.7780485972], [-35.4261634473, 37.1222179219], [-45.0982481096, 34.582379682], [-40.0185716299, 15.2382103574]], "y0" : -6.3, "y1" : -0.3 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -0.3, "y1" : 0.1, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -7.3, "y1" : 0.7, "r" : 3 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -7.3, "y1" : 0.7, "r" : 3 }], "anchor" : [-12.8328958545, -3.3, 5.954069599], "note" : "The wing's second support, in coupler L's layer: the rocker's outline, its pin through rocker, coupler and knuckle. The 6-32's nut in the boss, slid in from its top end; the screw from the wing's tab beyond. Its inner face's ring at the pin bears on its own coupler across the mid-plane. Rod: ream to run; pin: to press. Print the inner face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "wingL", "name" : "wing L stub (rocker)", "group" : "wL", "up" : [0, 1, 0], "add" : [{ "k" : "prism", "pts" : [[-30.3464869676, 17.7780485972], [-35.4261634473, 37.1222179219], [-45.0982481096, 34.582379682], [-40.0185716299, 15.2382103574]], "y0" : 12.9, "y1" : 18.9 }], "cut" : [], "anchor" : [-39.3053469877, 15.9, 16.4594026476], "note" : "STUB: the rocker-side tab block only. A 6-32 along Y from each tab into the rocker / knuckle's nut: one from each side. Print the outer (-Y) face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "wingKL", "name" : "wing L stub (knuckle)", "group" : "wL", "up" : [0, -1, 0], "add" : [{ "k" : "prism", "pts" : [[-30.3464869676, 17.7780485972], [-35.4261634473, 37.1222179219], [-45.0982481096, 34.582379682], [-40.0185716299, 15.2382103574]], "y0" : -12.3, "y1" : -6.3 }], "cut" : [], "anchor" : [-39.3053469877, -9.3, 16.4594026476], "note" : "STUB: the knuckle-side tab block only. Print the outer (+Y) face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "bladeL", "name" : "wing L blade", "group" : "wL", "up" : undefined, "add" : [{ "k" : "prism", "pts" : [[-68.7, 104.9], [-68.7, 81.56], [-68.6841310232, 79.51127306], [-68.636523475, 77.4735905159], [-68.5571764287, 75.4469497816], [-68.4460889578, 73.4313482708], [-68.3032601355, 71.4267833975], [-68.1286890353, 69.4332525755], [-67.9223747304, 67.4507532186], [-67.6843162943, 65.4792827407], [-67.4145128003, 63.5188385556], [-67.1129633217, 61.5694180772], [-66.779666932, 59.6310187193], [-66.4146227044, 57.7036378957], [-66.0178297124, 55.7872730204], [-65.5892870292, 53.8819215071], [-65.1289937283, 51.9875807697], [-64.6369488829, 50.104248222], [-64.1131515665, 48.231921278], [-63.5576008524, 46.3705973513], [-62.970295814, 44.520273856], [-62.3512355246, 42.6809482058], [-61.7004190576, 40.8526178145], [-61.0178454863, 39.0352800961], [-60.3035138841, 37.2289324643], [-59.5574233244, 35.433572333], [-58.7795728804, 33.6491971161], [-57.9699616256, 31.8758042274], [-57.1285886334, 30.1133910808], [-56.255452977, 28.36195509], [-55.3505537298, 26.6214936689], [-54.4138899652, 24.8920042315], [-53.4454607566, 23.1734841915], [-52.4452651772, 21.4659309627], [-51.4133023005, 19.7693419591], [-50.3495711999, 18.0837145944], [-49.2540709485, 16.4090462825], [-48.12680062, 14.7453344373], [-46.9677592875, 13.0925764726], [-45.7769460244, 11.4507698023], [-44.5543599041, 9.8199118401], [-43.3, 8.2], [-19.57, -21.96], [-15.69, -18.88], [-38.5, 9.4], [-63.9, 106.2]], "y0" : -61.25, "y1" : 20.75, "segs" : [[[-63.9, 106.2], [-68.7, 104.9]], [[-68.7, 104.9], [-68.7, 81.56]], [[-68.7, 81.56], [-68.7, 54.17], [-60.2366280657, 29.725861882], [-43.3, 8.2]], [[-43.3, 8.2], [-19.57, -21.96]], [[-19.57, -21.96], [-15.69, -18.88]], [[-15.69, -18.88], [-38.5, 9.4]], [[-38.5, 9.4], [-63.9, 106.2]]] }], "cut" : [], "anchor" : [-52.1610683515, -20.25, 57.5413990477], "note" : "DUMMY: the blade's whole external shape (config/swing_explore/blade_v3.yaml). How it holds the stubs: open.", "custom" : false, "mock" : false, "color" : [0.35, 0.55, 0.85], "kind" : "dummy" }, { "slug" : "rpinL", "name" : "rocker pin L", "group" : "wL", "up" : undefined, "add" : [{ "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -6.3, "y1" : 12.9, "r" : 3 }], "cut" : [], "anchor" : [-25.665791709, 3.3, 11.908139198], "note" : "Rod stock: pressed in the rocker and the knuckle, runs in the coupler between them.", "custom" : false, "mock" : false, "color" : [0.3, 0.3, 0.32], "kind" : "steel" }, { "slug" : "bulkheadF", "name" : "front bulkhead", "group" : "fixed", "up" : [0, -1, 0], "add" : [{ "k" : "prism", "pts" : [[-12.35, 25.09139], [-8.965752, -0.784402], [-8.86327, -1.562834], [-8.693332, -2.329371], [-8.457234, -3.078181], [-8.15677, -3.803564], [-7.794229, -4.5], [-7.372368, -5.162188], [-6.8944, -5.785088], [-6.363961, -6.363961], [-5.785088, -6.8944], [-5.162188, -7.372368], [-4.5, -7.794229], [-3.803564, -8.15677], [-3.078181, -8.457234], [-2.329371, -8.693332], [-1.562834, -8.86327], [-0.784402, -8.965752], [0, -9], [0.784402, -8.965752], [1.562834, -8.86327], [2.329371, -8.693332], [3.078181, -8.457234], [3.803564, -8.15677], [4.5, -7.794229], [5.162188, -7.372368], [5.785088, -6.8944], [6.363961, -6.363961], [6.8944, -5.785088], [7.372368, -5.162188], [7.794229, -4.5], [8.15677, -3.803564], [8.457234, -3.078181], [8.693332, -2.329371], [8.86327, -1.562834], [8.965752, -0.784402], [12.35, 25.09139], [12.35, 48.575519], [-12.35, 48.575519]], "y0" : 13.5, "y1" : 23.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 12.5, "y1" : 24.5, "r" : 3 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 12.5, "y1" : 24.5, "r" : 7.1 }], "anchor" : [0, 18.5, 12.5456948366], "note" : "Front journal bearing (ream to fit), the rod (ream to press). Print the inner face up.", "custom" : false, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "lowercase", "name" : "lower case", "group" : "fixed", "up" : [0, -1, 0], "add" : [{ "k" : "prism", "pts" : [[-12.35, -1.75861], [-5.785088, -6.8944], [-5.162188, -7.372368], [-4.5, -7.794229], [-3.803564, -8.15677], [-3.078181, -8.457234], [-2.329371, -8.693332], [-1.562834, -8.86327], [-0.784402, -8.965752], [0, -9], [0.784402, -8.965752], [1.562834, -8.86327], [2.329371, -8.693332], [3.078181, -8.457234], [3.803564, -8.15677], [4.5, -7.794229], [5.162188, -7.372368], [5.785088, -6.8944], [12.35, -1.75861], [12.35, 48.575519], [-12.35, 48.575519]], "y0" : -23.5, "y1" : -13.5 }, { "k" : "box", "lo" : [-12.35, -44.2, -1.7586103268], "hi" : [12.35, -23.5, 36.9413896732] }, { "k" : "box", "lo" : [-5.25, -37.4, 36.4413896732], "hi" : [5.25, -23, 47.5152989094] }], "cut" : [{ "k" : "box", "lo" : [-10.05, -54.7, 0.5413896732], "hi" : [10.05, -32.7, 34.6413896732] }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -33.2, "y1" : -23.5, "r" : 9.5 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -24.5, "y1" : -12.5, "r" : 7.1 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : -24.5, "y1" : -12.5, "r" : 3 }], "anchor" : [0, -18.5, 12.5456948366], "note" : "Rear journal bearing (ream to fit), the rod, the hub's cavity, the XC330's horn half. A block back over the servo holds the upper case's nut (slot from its top).", "custom" : false, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "uppercase", "name" : "upper case", "group" : "fixed", "up" : [0, 1, 0], "add" : [], "cut" : [], "anchor" : [-13.55, -59.2, 4.4163896732], "note" : "The XC330's back half, the servo turned long end down. One 6-32 forward into the lower case, from a lug on its top (45 deg back face). Connectors plug in through the cap.", "custom" : true, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "hub", "name" : "horn hub", "group" : "crank", "up" : [0, 1, 0], "add" : [{ "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -29.7, "y1" : -24.5, "r" : 9 }], "cut" : [{ "k" : "prism", "pts" : [[2.9344931419, 26.1166945059], [8.0256619665, 31.2078633304], [6.1164736573, 33.1170516396], [1.0253048327, 28.0258828151]], "y0" : -27.8, "y1" : -23.5 }, { "k" : "prism", "pts" : [[-1.0253048327, 28.0258828151], [-6.1164736573, 33.1170516396], [-8.0256619665, 31.2078633304], [-2.9344931419, 26.1166945059]], "y0" : -27.8, "y1" : -23.5 }, { "k" : "prism", "pts" : [[-2.9344931419, 24.0660848404], [-8.0256619665, 18.9749160159], [-6.1164736573, 17.0657277067], [-1.0253048327, 22.1568965312]], "y0" : -27.8, "y1" : -23.5 }, { "k" : "prism", "pts" : [[1.0253048327, 22.1568965312], [6.1164736573, 17.0657277067], [8.0256619665, 18.9749160159], [2.9344931419, 24.0660848404]], "y0" : -27.8, "y1" : -23.5 }, { "k" : "cyl", "x" : 6, "z" : 25.0913896732, "y0" : -30.7, "y1" : -23.5, "r" : 1.05 }, { "k" : "cyl", "x" : 6, "z" : 25.0913896732, "y0" : -26.3, "y1" : -23.5, "r" : 1.85 }, { "k" : "cyl", "x" : 0, "z" : 31.0913896732, "y0" : -30.7, "y1" : -23.5, "r" : 1.05 }, { "k" : "cyl", "x" : 0, "z" : 31.0913896732, "y0" : -26.3, "y1" : -23.5, "r" : 1.85 }, { "k" : "cyl", "x" : -6, "z" : 25.0913896732, "y0" : -30.7, "y1" : -23.5, "r" : 1.05 }, { "k" : "cyl", "x" : -6, "z" : 25.0913896732, "y0" : -26.3, "y1" : -23.5, "r" : 1.85 }, { "k" : "cyl", "x" : 0, "z" : 19.0913896732, "y0" : -30.7, "y1" : -23.5, "r" : 1.05 }, { "k" : "cyl", "x" : 0, "z" : 19.0913896732, "y0" : -26.3, "y1" : -23.5, "r" : 1.85 }], "anchor" : [4.6193976626, -29.2, 27.004806835], "note" : "aow-bike-steering's horn hub: M2 x 6 into the horn, the cross socket for the lugs.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "washer", "name" : "centre washer", "group" : "crank", "up" : [0, 1, 0], "add" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -0.2, "y1" : 0.2, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -1.2, "y1" : 1.2, "r" : 3 }], "anchor" : [4.2500000158, 0, 40.0755186068], "note" : "Loose on the crankpin between the couplers: print two layers, or cut from Delrin/PTFE shim.", "custom" : false, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "crankpin", "name" : "crankpin", "group" : "crank", "up" : undefined, "add" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -12.9, "y1" : 12.9, "r" : 3 }], "cut" : [], "anchor" : [0.0000000158, 1.3, 40.0755186068], "note" : "Rod stock: pressed in both webs, the couplers run on it.", "custom" : false, "mock" : false, "color" : [0.3, 0.3, 0.32], "kind" : "steel" }, { "slug" : "rod", "name" : "wing rod", "group" : "fixed", "up" : undefined, "add" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -23.5, "y1" : 23.5, "r" : 3 }], "cut" : [], "anchor" : [0, 0, 0], "note" : "Rod stock: pressed in both bearings, the hubs run on it.", "custom" : false, "mock" : false, "color" : [0.3, 0.3, 0.32], "kind" : "steel" }, { "slug" : "bridge", "name" : "bridge", "group" : "fixed", "up" : [0, 0, 1], "add" : [{ "k" : "box", "lo" : [-16, -23.5, 48.5755186068], "hi" : [16, 23.5, 58.5755186068] }], "cut" : [], "anchor" : [13, 12, 50.5755186068], "note" : "Ties the front bulkhead, lower and upper cases. Top face: the chassis joints.", "custom" : false, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "railR", "name" : "side rail R (placeholder)", "group" : "fixed", "up" : [-1, 0, 0], "add" : [{ "k" : "box", "lo" : [16, -7.5, 47.5755186068], "hi" : [20.6, 7.5, 59.5755186068] }], "cut" : [], "anchor" : [18.3, 6.5, 58.5755186068], "note" : "PLACEHOLDER for the bike's side rail (the drive case side grown forward).", "custom" : false, "mock" : true, "color" : [0.85, 0.85, 0.85], "kind" : "print" }, { "slug" : "railL", "name" : "side rail L (placeholder)", "group" : "fixed", "up" : [1, 0, 0], "add" : [{ "k" : "box", "lo" : [-20.6, -7.5, 47.5755186068], "hi" : [-16, 7.5, 59.5755186068] }], "cut" : [], "anchor" : [-18.3, 6.5, 58.5755186068], "note" : "PLACEHOLDER for the bike's side rail (the drive case side grown forward).", "custom" : false, "mock" : true, "color" : [0.85, 0.85, 0.85], "kind" : "print" }, { "slug" : "floor", "name" : "floor (mock)", "group" : "fixed", "up" : undefined, "add" : [{ "k" : "box", "lo" : [-130, -64.7, -43.2], "hi" : [130, 33.5, -41.2] }], "cut" : [], "anchor" : [100, 0, -42.2], "note" : "", "custom" : false, "mock" : true, "color" : [0.85, 0.85, 0.85], "kind" : "mock" }];

export const RG_POSES = { "-1.00" : { "crank" : { "p" : [0, 25.0913896732], "a" : 243.1, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -5.067133972, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 292.8286068734, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -41.3576458053, "d" : [-13.3628091893, -21.7634689379] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -30.8808812943, "d" : [-13.3628091893, -21.7634689379] } }, "-0.75" : { "crank" : { "p" : [0, 25.0913896732], "a" : -87.675, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -13.2921096768, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 312.8439396037, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -35.220127473, "d" : [-14.9717938521, -14.3762562291] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -25.2280425928, "d" : [-14.9717938521, -14.3762562291] } }, "-0.56" : { "crank" : { "p" : [0, 25.0913896732], "a" : -65.464, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -14.0817028468, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 327.0341493602, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -26.8817503035, "d" : [-13.6310700691, -8.7617460218] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -20.1658031719, "d" : [-13.6310700691, -8.7617460218] } }, "-0.50" : { "crank" : { "p" : [0, 25.0913896732], "a" : -58.45, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -13.6515683061, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 331.2640908343, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -23.9844409587, "d" : [-12.7692330468, -7.1437968169] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -18.4030365014, "d" : [-12.7692330468, -7.1437968169] } }, "-0.25" : { "crank" : { "p" : [0, 25.0913896732], "a" : -29.225, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -8.945217653, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : -12.7558358912, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -11.6318889078, "d" : [-7.3158585566, -1.9073428716] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -10.0691646269, "d" : [-7.3158585566, -1.9073428716] } }, "+0.00" : { "crank" : { "p" : [0, 25.0913896732], "a" : 0, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 0, "d" : [0, 0] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 0, "d" : [0, 0] } }, "+0.25" : { "crank" : { "p" : [0, 25.0913896732], "a" : 29.225, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 12.7558358912, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 8.945217653, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 10.0691646145, "d" : [7.3158585526, -1.9073428871] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 11.6318889158, "d" : [7.3158585526, -1.9073428871] } }, "+0.50" : { "crank" : { "p" : [0, 25.0913896732], "a" : 58.45, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 28.7359091657, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 13.6515683061, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 18.4030364725, "d" : [12.7692330317, -7.1437968438] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 23.9844409706, "d" : [12.7692330317, -7.1437968438] } }, "+0.56" : { "crank" : { "p" : [0, 25.0913896732], "a" : 65.464, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 32.9658506398, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 14.0817028468, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 20.1658031387, "d" : [13.6310700506, -8.7617460506] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 26.8817503158, "d" : [13.6310700506, -8.7617460506] } }, "+0.75" : { "crank" : { "p" : [0, 25.0913896732], "a" : 87.675, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 47.1560603963, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 13.2921096768, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 25.2280425455, "d" : [14.9717938217, -14.3762562607] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 35.2201274847, "d" : [14.9717938217, -14.3762562607] } }, "+1.00" : { "crank" : { "p" : [0, 25.0913896732], "a" : 116.9, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 67.1713931266, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 5.067133972, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 30.8808812296, "d" : [13.3628091433, -21.7634689661] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 41.35764581, "d" : [13.3628091433, -21.7634689661] } } };

export function rgV(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]) * millimeter;
}

export function rgU(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]);
}

export function rgJoint(context is Context, id is Id, j is map, cskPart is Query, nutPart is Query)
{
    const f  = FIXTURE;
    const mm = millimeter;
    const head = rgV(j.head);
    const zIn = rgU(j.zIn);
    const cs = coordSystem(head, rgU(j.slot), zIn);
    const nd = f.joint_plate + 2 * mm;
    screwJointGeometry(context, id + "scr", cs, mergeMaps(SCREW_OPT, {
            "nutDepth" : nd, "headPocket" : j.pocket * mm, "bothWays" : j.through,
            "nutSlotLength" : j.through ? 200 * mm : j.slotLen * mm }));
    const bore = qCreatedBy(id + "scr" + "screwRev", EntityType.BODY);
    const slot = qCreatedBy(id + "scr" + "slotExt", EntityType.BODY);
    if (j.ridge)
    {
        const jp = head + f.joint_plate * zIn;
        const up = j.ridgeOnNut ? -zIn : zIn;
        const ridgePart = j.ridgeOnNut ? nutPart : cskPart;
        const other     = j.ridgeOnNut ? cskPart : nutPart;
        const rd = rgU(j.ridgeDir);
        const jm = jp + (j.ridgeMid == undefined ? 0 : j.ridgeMid) * mm * rd;
        const r = ridgeSolid(context, id + "ridge", jm, rd, up, f.ridge_height, 0 * mm, j.ridgeHalf * mm);
        if (j.ridgeClipZ != undefined)
            opBoolean(context, id + "rClip", { "tools" : boxW(context, id + "rClipB",
                    vector(-500, -500, j.ridgeClipZ - 100) * mm, vector(500, 500, j.ridgeClipZ) * mm),
                    "targets" : r, "operationType" : BooleanOperationType.SUBTRACTION });
        opBoolean(context, id + "rAdd", { "tools" : qUnion([ridgePart, r]),
                "operationType" : BooleanOperationType.UNION });
        const g = ridgeSolid(context, id + "groove", jm, rd, up, f.ridge_height,
                             f.ridge_clearance, j.ridgeHalf * mm + 1 * mm);
        opBoolean(context, id + "gCut", { "tools" : g, "targets" : other,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }
    opBoolean(context, id + "slotCut", { "tools" : slot, "targets" : nutPart,
            "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "cut", { "tools" : bore, "targets" : qUnion([cskPart, nutPart]),
            "operationType" : BooleanOperationType.SUBTRACTION });
    const rB = SCREW_OPT.holeDia / 2;
    for (var pu in [[cskPart, rgU(j.cskUp), "tdC"], [nutPart, rgU(j.nutUp), "tdN"]])
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
    const along = dot(rgU(j.nutUp), zIn);
    if (abs(along) > 0.5)
    {
        const L  = f.bridge_layer;
        const rH = SCREW_OPT.holeDia / 2;
        const w  = SCREW_OPT.nutSlotWidth;
        const d0 = along > 0 ? nd + SCREW_OPT.nutSlotThickness : nd;
        const sg = along > 0 ? 1 : -1;
        const l1 = boxIn(context, id + "br1", cs, vector(-rH, -w / 2, d0 + (sg > 0 ? 0 * mm : -L)),
                         vector(rH, w / 2, d0 + (sg > 0 ? L : 0 * mm)));
        const l2 = boxIn(context, id + "br2", cs, vector(-rH, -rH, d0 + sg * L + (sg > 0 ? 0 * mm : -L)),
                         vector(rH, rH, d0 + sg * L + (sg > 0 ? L : 0 * mm)));
        opBoolean(context, id + "brCut", { "tools" : qUnion([l1, l2]), "targets" : nutPart,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }
}

export function rgPrim(context is Context, id is Id, q is map) returns Query
{
    const mm = millimeter;
    if (q.k == "cyl")
    {
        fCylinder(context, id, { "bottomCenter" : vector(q.x, q.y0, q.z) * mm,
                                 "topCenter"    : vector(q.x, q.y1, q.z) * mm,
                                 "radius"       : q.r * mm });
    }
    else if (q.k == "box")
    {
        fCuboid(context, id, { "corner1" : rgV(q.lo), "corner2" : rgV(q.hi) });
    }
    else if (q.k == "poly")
    {
        var sk = newSketchOnPlane(context, id + "sk", { "sketchPlane" : plane(vector(0, 0, 0) * mm, vector(0, 1, 0), vector(1, 0, 0)) });
        for (var i = 0; i < size(q.pts); i += 1)
        {
            const a = q.pts[i];
            const b = q.pts[(i + 1) % size(q.pts)];
            skLineSegment(sk, "s" ~ i, { "start" : vector(a[0], -a[1]) * mm, "end" : vector(b[0], -b[1]) * mm });
        }
        skSolve(sk);
        opExtrude(context, id + "ext", {
                "entities"  : qSketchRegion(id + "sk"),
                "direction" : vector(0, -1, 0),
                "endBound"  : BoundingType.BLIND,
                "endDepth"  : q.depth * mm });
        opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
        const xd = normalize(rgU(q.xdir));
        const nd = normalize(rgU(q.normal) - dot(rgU(q.normal), xd) * xd);
        opTransform(context, id + "xf", { "bodies" : qCreatedBy(id + "ext", EntityType.BODY),
                "transform" : toWorld(coordSystem(rgV(q.origin), xd, normalize(cross(nd, xd)))) });
    }
    else
    {
        const alongX = q.k == "prismX";
        const pl = alongX ? plane(vector(q.x0, 0, 0) * mm, vector(1, 0, 0), vector(0, 1, 0))
                          : plane(vector(0, q.y0, 0) * mm, vector(0, 1, 0), vector(1, 0, 0));
        var sk = newSketchOnPlane(context, id + "sk", { "sketchPlane" : pl });
        const P = function(v) { return alongX ? vector(v[0], v[1]) * mm : vector(v[0], -v[1]) * mm; };
        if (q.k == "slot")
        {
            skLineSegment(sk, "l1", { "start" : P(q.pts[0]), "end" : P(q.pts[1]) });
            skArc(sk, "ab", { "start" : P(q.pts[1]), "mid" : P(q.mids[0]), "end" : P(q.pts[2]) });
            skLineSegment(sk, "l2", { "start" : P(q.pts[2]), "end" : P(q.pts[3]) });
            skArc(sk, "aa", { "start" : P(q.pts[3]), "mid" : P(q.mids[1]), "end" : P(q.pts[0]) });
        }
        else if (q.segs != undefined)
        {
            for (var i = 0; i < size(q.segs); i += 1)
            {
                const s = q.segs[i];
                if (size(s) == 2)
                    skLineSegment(sk, "s" ~ i, { "start" : P(s[0]), "end" : P(s[1]) });
                else
                    skBezier(sk, "b" ~ i, { "points" : [P(s[0]), P(s[1]), P(s[2]), P(s[3])] });
            }
        }
        else
        {
            for (var i = 0; i < size(q.pts); i += 1)
                skLineSegment(sk, "s" ~ i, { "start" : P(q.pts[i]), "end" : P(q.pts[(i + 1) % size(q.pts)]) });
        }
        skSolve(sk);
        opExtrude(context, id + "ext", {
                "entities"  : qSketchRegion(id + "sk"),
                "direction" : alongX ? vector(1, 0, 0) : vector(0, 1, 0),
                "endBound"  : BoundingType.BLIND,
                "endDepth"  : (alongX ? q.x1 - q.x0 : q.y1 - q.y0) * mm });
        opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
    }
    return qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID);
}

export function rgPart(context is Context, id is Id, p is map)
{
    var adds = [];
    for (var i = 0; i < size(p.add); i += 1)
        adds = append(adds, rgPrim(context, id + ("a" ~ i), p.add[i]));
    if (size(adds) > 1)
        opBoolean(context, id + "uni", { "tools" : qUnion(adds),
                                         "operationType" : BooleanOperationType.UNION });
    const body = qUnion(evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID)));
    if (size(p.cut) > 0)
    {
        var cuts = [];
        for (var i = 0; i < size(p.cut); i += 1)
            cuts = append(cuts, rgPrim(context, id + ("c" ~ i), p.cut[i]));
        opBoolean(context, id + "sub", { "tools" : qUnion(cuts), "targets" : body,
                                         "operationType" : BooleanOperationType.SUBTRACTION });
    }
}

export function rgFind(context is Context, scope is Id, slug is string) returns Query
{
    for (var p in RG_PARTS)
        if (p.slug == slug)
            return partAt(context, scope, rgV(p.anchor));
    throw regenError("no part " ~ slug);
}

export function rightingBuild(context is Context, id is Id, opt is map) returns map
{
    const mm = millimeter;
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };
    for (var p in RG_PARTS)
    {
        if (p.custom || (p.mock && !opt.mocks))
            continue;
        step(p.slug);
        rgPart(context, id + p.slug, p);
    }
    step("servo");
    const cs = coordSystem(vector(0, RG.yH, RG.zC) * mm, vector(RG.servoX, 0, 0), vector(0, 1, 0));
    const sv = x330Envelope(context, id + "servo", cs);
    {
        const T = SERVO_MOUNT_TABLE["XC330"];
        const zc = -X330.hornThickness;
        const zb = zc - X330.caseDepth;
        const rowY = X330.caseHeight - X330.shaftFromEnd - X330.caseHeight / 2 - T.casePinRowOffset;
        var nh = [];
        for (var sx in [1, -1])
        {
            const x = sx * T.caseHoleSpanX / 2;
            nh = append(nh, cylIn(context, id + ("nhf" ~ (sx > 0 ? "p" : "n")), cs,
                    vector(x, rowY, zc - T.caseReliefDepthHorn), vector(x, rowY, zc + 1 * mm), T.caseReliefDia / 2));
            nh = append(nh, cylIn(context, id + ("nhb" ~ (sx > 0 ? "p" : "n")), cs,
                    vector(x, rowY, zb - 1 * mm), vector(x, rowY, zb + T.caseReliefDepthBack), T.caseReliefDia / 2));
        }
        opBoolean(context, id + "nearHoles", { "tools" : qUnion(nh), "targets" : sv.caseQ,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }
    step("lower case shell");
    caseShellBuild(context, id + "lcShell", cs, mergeMaps(CASE_OPT, { "part" : "TOP" }),
                   rgFind(context, id, "lowercase"));
    if (RG.lowerNearPins > 0.5)
        nearRowPins(context, id + "lcNear", cs, "TOP", rgFind(context, id, "lowercase"));
    step("upper case");
    caseShellBuild(context, id + "ucShell", cs, mergeMaps(CASE_OPT, { "part" : "BOTTOM" }), qNothing());
    var ucAdd = [qUnion(evaluateQuery(context, partAt(context, id, rgV(RG.ucShellPt))))];
    for (var i = 0; i < size(RG.ucAdd); i += 1)
        ucAdd = append(ucAdd, rgPrim(context, id + ("ucA" ~ i), RG.ucAdd[i]));
    opBoolean(context, id + "ucU", { "tools" : qUnion(ucAdd), "operationType" : BooleanOperationType.UNION });
    const ucT = qUnion(evaluateQuery(context, rgFind(context, id, "uppercase")));
    var ucCut = [];
    for (var i = 0; i < size(RG.ucCut); i += 1)
        ucCut = append(ucCut, rgPrim(context, id + ("ucC" ~ i), RG.ucCut[i]));
    opBoolean(context, id + "ucSub", { "tools" : qUnion(ucCut), "targets" : ucT,
            "operationType" : BooleanOperationType.SUBTRACTION });
    nearRowPins(context, id + "ucNear", cs, "BOTTOM", rgFind(context, id, "uppercase"));
    step("joints");
    for (var j in RG.joints)
    {
        if (!opt.mocks && (j.csk == "chassis" || j.nut == "chassis" || j.csk == "railR" || j.csk == "railL"))
            continue;
        step("joint " ~ j.tag);
        rgJoint(context, id + j.tag, j, rgFind(context, id, j.csk), rgFind(context, id, j.nut));
    }
    step("dress");
    var groups = { "fixed" : [], "crank" : [], "cR" : [], "cL" : [], "wR" : [], "wL" : [] };
    var prints = [];
    for (var p in RG_PARTS)
    {
        if (p.mock && !opt.mocks)
            continue;
        const q = qUnion(evaluateQuery(context, rgFind(context, id, p.slug)));
        dress(context, q, p.name, color(p.color[0], p.color[1], p.color[2]), p.note);
        groups[p.group] = append(groups[p.group], q);
        if (p.up != undefined && p.kind == "print")
            prints = append(prints, [p.name, q, rgU(p.up)]);
    }
    if (opt.mocks)
    {
        dress(context, sv.caseQ, "X330 case", color(0.16, 0.16, 0.18), "");
        dress(context, sv.horn, "X330 horn", color(0.3, 0.3, 0.32), "");
        groups["fixed"] = append(groups["fixed"], sv.caseQ);
        groups["crank"] = append(groups["crank"], sv.horn);
    }
    else
        opDeleteBodies(context, id + "noServo", { "entities" : qUnion([sv.caseQ, sv.horn]) });
    return { "groups" : groups, "prints" : prints };
}

export function rgXf(g is map) returns Transform
{
    const mm = millimeter;
    return transform(vector(g.d[0], 0, g.d[1]) * mm)
         * rotationAround(line(vector(g.p[0], 0, g.p[1]) * mm, vector(0, 1, 0)), g.a * degree);
}

export function rgPose(context is Context, id is Id, groups is map, key is string, back is boolean)
{
    const ps = RG_POSES[key];
    for (var g in ["crank", "cR", "cL", "wR", "wL"])
    {
        if (size(groups[g]) == 0)
            continue;
        const xf = rgXf(ps[g]);
        opTransform(context, id + g, { "bodies" : qUnion(groups[g]),
                                       "transform" : back ? inverse(xf) : xf });
    }
}

export const BK2 = { "wb" : 200.5, "yr" : 120.5, "zr" : -10, "tilt" : 45, "rake" : 15, "sOrigin" : [0, 200.5, 0.05], "sAxis" : [0, -0.2588190451, 0.9659258263], "rgDrop" : [[18.3, 6.5, 58.5755186068], [-18.3, 6.5, 58.5755186068], [100, 0, -42.2]], "comps" : [{ "name" : "battery", "frame" : "drive", "at" : [0, 87, 45.75], "rx" : 0, "boxes" : [{ "name" : "battery", "lo" : [-11, -29, -15], "hi" : [11, 29, 15], "color" : [0.85, 0.75, 0.2] }], "lift" : false }, { "name" : "AHRS", "frame" : "drive", "at" : [0, 128.6953, -9.75], "rx" : 0, "boxes" : [{ "name" : "AHRS", "lo" : [-17, -6.3, -20], "hi" : [17, -4.7, 20], "color" : [0.15, 0.35, 0.2] }, { "name" : "AHRS housing", "lo" : [-15, -4.7, -13.66], "hi" : [15, 6.3, 12.34], "color" : [0.55, 0.15, 0.15] }, { "name" : "AHRS USB-C", "lo" : [1.65, -4.7, 12.4], "hi" : [10.75, -1.3, 20], "color" : [0.75, 0.75, 0.78] }, { "name" : "AHRS USB plug (keep-out)", "lo" : [0, -6.25, 20], "hi" : [12.4, 0.25, 50], "color" : [0.85, 0.85, 0.9] }], "holes" : { "at" : [[15, -15.95], [-15, -15.95], [15, 15.05], [-15, 15.05]], "r" : 1.05, "y" : [-7.3, -3.7] }, "lift" : false }, { "name" : "Pi 3B+", "frame" : "drive", "at" : [0, 164.4, -5.8], "rx" : 0, "boxes" : [{ "name" : "Pi 3B+", "lo" : [-28, -7.4, -44.5], "hi" : [28, -6, 40.5], "color" : [0.1, 0.5, 0.2] }, { "name" : "Pi USB + Ethernet", "lo" : [-26.5, -6, 25], "hi" : [26.5, 10, 42.5], "color" : [0.75, 0.75, 0.78] }, { "name" : "Pi GPIO header", "lo" : [22, -6, -37.5], "hi" : [27, 2.5, 13.5], "color" : [0.15, 0.15, 0.15] }], "holes" : { "at" : [[24.5, -41], [-24.5, -41], [24.5, 17], [-24.5, 17]], "r" : 1.375, "y" : [-8.7, -4.7] }, "lift" : true }, { "name" : "U2D2", "frame" : "drive", "at" : [0, 143.55, -19], "rx" : 0, "boxes" : [{ "name" : "U2D2", "lo" : [-9, -7.45, -24], "hi" : [9, 7.45, 24], "color" : [0.2, 0.2, 0.25] }, { "name" : "U2D2 USB plug (keep-out)", "lo" : [-6.2, -3.25, 24], "hi" : [6.2, 3.25, 54], "color" : [0.85, 0.85, 0.9] }, { "name" : "U2D2 JST plugs (keep-out)", "lo" : [-7, -13.45, -24], "hi" : [7, -7.45, -7], "color" : [0.85, 0.85, 0.9] }], "lift" : true }, { "name" : "power board", "frame" : "drive", "at" : [-27.4, 115.45, 59.75], "rx" : 0, "boxes" : [{ "name" : "power board", "lo" : [4.7, -20.25, -26], "hi" : [6.3, 6.25, -1], "color" : [0.15, 0.4, 0.15] }, { "name" : "power board components", "lo" : [-6.3, -20.25, -26], "hi" : [4.7, 6.25, -1], "color" : [0.3, 0.3, 0.3] }, { "name" : "XT30", "lo" : [0.8, -13.92, -1], "hi" : [6.3, -3.72, 10], "color" : [0.9, 0.75, 0.1] }, { "name" : "XT30 plug (keep-out)", "lo" : [0.8, -13.92, 10], "hi" : [6.3, -3.72, 26], "color" : [0.85, 0.85, 0.9] }, { "name" : "5 V plug (keep-out)", "lo" : [-3.8, 6.25, -24.73], "hi" : [4.7, 20.25, -19.65], "color" : [0.85, 0.85, 0.9] }, { "name" : "JST plug (keep-out)", "lo" : [-3.8, 6.25, -17.11], "hi" : [4.7, 20.25, -9.49], "color" : [0.85, 0.85, 0.9] }], "holes" : { "ax" : "x", "r" : 1.45, "y" : [3.7, 7.3], "at" : [[-15.17, -20.92], [-15.17, -5.68]] }, "lift" : false }, { "name" : "switch", "frame" : "drive", "at" : [-25, 84, 58.5], "rx" : 0, "boxes" : [{ "name" : "switch", "lo" : [-6.5, -9.7, -11.25], "hi" : [6.5, 9.7, 7.25], "color" : [0.12, 0.12, 0.12] }, { "name" : "switch (through the shelf)", "lo" : [-6.5, -9.7, 7.25], "hi" : [6.5, 9.7, 9.25], "color" : [0.12, 0.12, 0.12] }, { "name" : "switch bezel", "lo" : [-7.5, -10.75, 9.25], "hi" : [7.5, 10.75, 11.25], "color" : [0.8, 0.1, 0.1] }], "lift" : false }], "carriers" : [{ "name" : "power holder", "color" : [0.93, 0.56, 0.2], "note" : "Printed -X (its battery-side face on the bed). The power board, the switch in the shelf, strap slits; two 6-32 into the left rail.", "adds" : [{ "tag" : "panel", "x" : [-16, -11.3], "pts" : [[25.438965005, 67.5118184856], [63.388795336, 105.4616488166], [68.673680331, 100.1767638216], [78.5939150359, 110.0969985265], [68.8896126585, 119.8013009039], [60.5990511477, 111.5107393931], [38.148410845, 133.9613796958], [-1.4310926801, 94.3818761707]] }, { "tag" : "shelf", "x" : [-33, -12.3], "pts" : [[3.8537319575, 96.8382736835], [34.2098260738, 127.1943677998], [32.7956125114, 128.6085813622], [2.4395183951, 98.2524872459]] }, { "tag" : "gusset0", "kind" : "wedge", "y" : [71.2, 72.8], "rot" : 45, "pts" : [[-15.5, 66.25], [-15.5, 59.25], [-16, 59.25], [-33, 65.75], [-33, 66.25]] }, { "tag" : "gusset1", "kind" : "wedge", "y" : [97.2, 98.8], "rot" : 45, "pts" : [[-15.5, 66.25], [-15.5, 59.25], [-16, 59.25], [-33, 65.75], [-33, 66.25]] }, { "tag" : "gusset2", "kind" : "wedge", "y" : [112.53, 114.13], "rot" : 45, "pts" : [[-15.5, 66.25], [-15.5, 59.25], [-16, 59.25], [-33, 65.75], [-33, 66.25]] }], "cuts" : [{ "tag" : "cutout", "x" : [-31.5, -18.5], "pts" : [[6.7528697603, 98.323197924], [20.4707413154, 112.041069479], [17.6423141906, 114.8694966038], [3.9244426356, 101.1516250487]] }, { "tag" : "xt30", "x" : [-26.9, -20.8], "pts" : [[25.7952553777, 117.3655835413], [33.4320086145, 125.0023367782], [30.6035814898, 127.8307639029], [22.9668282529, 120.1940106661]] }, { "tag" : "wires", "x" : [-17, -10.3], "pts" : [[40.7613783381, 81.4200182563], [48.9638169998, 89.622456918], [44.7211763127, 93.8650976052], [36.5187376509, 85.6626589434]] }, { "tag" : "strapUp", "x" : [-17, -10.3], "pts" : [[11.2783531599, 97.1918270741], [25.4204887837, 111.3339626978], [21.8849548777, 114.8694966038], [7.742819254, 100.72736098]] }, { "tag" : "strapLow", "x" : [-17, -10.3], "pts" : [[33.9057701579, 74.5644100761], [48.0479057816, 88.7065456999], [43.8052650945, 92.949186387], [29.6631294708, 78.8070507632]] }], "cyls" : [{ "tag" : "so0", "op" : "add", "p0" : [-15.5, 43.4517117039, 98.3656243309], "p1" : [-21.1, 43.4517117039, 98.3656243309], "r" : 3 }, { "tag" : "pl0", "op" : "cut", "p0" : [-10.8, 43.4517117039, 98.3656243309], "p1" : [-21.6, 43.4517117039, 98.3656243309], "r" : 1.15 }, { "tag" : "so1", "op" : "add", "p0" : [-15.5, 32.6754043586, 109.1419316761], "p1" : [-21.1, 32.6754043586, 109.1419316761], "r" : 3 }, { "tag" : "pl1", "op" : "cut", "p0" : [-10.8, 32.6754043586, 109.1419316761], "p1" : [-21.6, 32.6754043586, 109.1419316761], "r" : 1.15 }], "joints" : [{ "side" : "A", "head" : [-20.6, 24.7134, 74.918], "zIn" : [1, 0, 0], "slot" : [0, -0.7071067812, -0.7071067812], "nutDepth" : 5.6, "slotLen" : 5.7239092363, "holeDepth" : 8.3, "ridgeAt" : [-16, 23.2991864376, 76.3322135624], "ridgeHalf" : 5, "ridgeUp" : [-1, 0, 0], "ridgeRot" : 45 }, { "side" : "F", "head" : [-20.6, 69.2965, 107.4802], "zIn" : [1, 0, 0], "slot" : [0, 0.7071067812, 0.7071067812], "nutDepth" : 5.6, "slotLen" : 9.4246212025, "holeDepth" : 10.3, "ridgeAt" : [-16, 67.8822864376, 108.8944135624], "ridgeHalf" : 5, "ridgeUp" : [-1, 0, 0], "ridgeRot" : 45 }], "pt" : [-13.65, 27.5418091272, 95.0705067305] }, { "name" : "Pi carrier", "color" : [0.3, 0.55, 0.85], "lift" : true, "note" : "Printed. The U2D2 under the plate on two M2, the Pi over it on four spacers (4-40 self-tapping into the plate); three 6-32 into the rails, lifts out upward.", "adds" : [{ "tag" : "plate", "x" : [-28, 28], "pts" : [[142.3405950529, 71.2056528655], [143.7548086152, 72.6198664279], [83.6507322144, 132.7239428287], [82.236518652, 131.3097292663]] }, { "tag" : "cheekR", "x" : [9.3, 10.9], "pts" : [[122.3294731453, 84.1457069612], [126.2185604418, 88.0347942577], [114.9048519428, 99.3485027567], [111.0157646463, 95.4594154602]] }, { "tag" : "cheekL", "x" : [-10.9, -9.3], "pts" : [[122.3294731453, 84.1457069612], [126.2185604418, 88.0347942577], [114.9048519428, 99.3485027567], [111.0157646463, 95.4594154602]] }, { "tag" : "lugT", "x" : [-16, -11.3], "pts" : [[80.6102, 111.5867932188], [91.6383807404, 122.6149739592], [83.9312739592, 130.3220807404], [72.9030932188, 119.2939]] }, { "tag" : "lugLL", "x" : [-16, -11.3], "pts" : [[143.1891, 57.4931932188], [151.0352909121, 65.3393841309], [143.047701834, 73.326973209], [135.2015109219, 65.4807822969]] }, { "tag" : "lugLR", "x" : [11.3, 16], "pts" : [[143.1891, 57.4931932188], [151.0352909121, 65.3393841309], [143.047701834, 73.326973209], [135.2015109219, 65.4807822969]] }], "cuts" : [], "cyls" : [{ "tag" : "bs0", "op" : "add", "p0" : [24.5, 137.037294194, 70.8520994749], "p1" : [24.5, 140.2192747093, 74.0340799902], "r" : 3 }, { "tag" : "pl0", "op" : "cut", "p0" : [24.5, 136.6837408034, 70.4985460843], "p1" : [24.5, 141.6334882717, 75.4482935526], "r" : 1.15 }, { "tag" : "bs1", "op" : "add", "p0" : [-24.5, 137.037294194, 70.8520994749], "p1" : [-24.5, 140.2192747093, 74.0340799902], "r" : 3 }, { "tag" : "pl1", "op" : "cut", "p0" : [-24.5, 136.6837408034, 70.4985460843], "p1" : [-24.5, 141.6334882717, 75.4482935526], "r" : 1.15 }, { "tag" : "bs2", "op" : "add", "p0" : [24.5, 96.0251008851, 111.8642927837], "p1" : [24.5, 99.2070814005, 115.0462732991], "r" : 3 }, { "tag" : "pl2", "op" : "cut", "p0" : [24.5, 95.6715474945, 111.5107393931], "p1" : [24.5, 100.6212949628, 116.4604868614], "r" : 1.15 }, { "tag" : "bs3", "op" : "add", "p0" : [-24.5, 96.0251008851, 111.8642927837], "p1" : [-24.5, 99.2070814005, 115.0462732991], "r" : 3 }, { "tag" : "pl3", "op" : "cut", "p0" : [-24.5, 95.6715474945, 111.5107393931], "p1" : [-24.5, 100.6212949628, 116.4604868614], "r" : 1.15 }, { "tag" : "uh0", "op" : "cut", "p0" : [0, 134.703841816, 78.1352993211], "p1" : [0, 136.8251621596, 80.2566196647], "r" : 1.2 }, { "tag" : "uh1", "op" : "cut", "p0" : [0, 105.0053570062, 107.8337841309], "p1" : [0, 107.1266773498, 109.9551044745], "r" : 1.2 }], "joints" : [{ "side" : "T", "head" : [-20.6, 80.6102, 118.7939], "zIn" : [1, 0, 0], "slot" : [0, -0.7071067812, 0.7071067812], "nutDepth" : 5.6, "slotLen" : 6.8033008589, "holeDepth" : 10.3, "ridgeAt" : [-16, 80.6102, 119.2939], "ridgeHalf" : 3.5, "ridgeUp" : [-1, 0, 0], "ridgeRot" : 0 }, { "side" : "LL", "head" : [-20.6, 143.1891, 64.7003], "zIn" : [1, 0, 0], "slot" : [0, 0.7071067812, 0.7071067812], "nutDepth" : 5.6, "slotLen" : 6.9999955232, "holeDepth" : 10.3, "ridgeAt" : [-16, 143.1891, 65.2003], "ridgeHalf" : 3.5, "ridgeUp" : [-1, 0, 0], "ridgeRot" : 0 }, { "side" : "LR", "head" : [20.6, 143.1891, 64.7003], "zIn" : [-1, 0, 0], "slot" : [0, 0.7071067812, 0.7071067812], "nutDepth" : 5.6, "slotLen" : 6.9999955232, "holeDepth" : 10.3, "ridgeAt" : [16, 143.1891, 65.2003], "ridgeHalf" : 3.5, "ridgeUp" : [1, 0, 0], "ridgeRot" : 0 }], "pt" : [10.15, 112.9956636336, 101.9647978471] }, { "name" : "Pi spacer 1", "color" : [0.85, 0.85, 0.85], "lift" : true, "note" : "Printed, separate (4-40 clearance). Under the Pi's board.", "adds" : [], "cuts" : [], "cyls" : [{ "tag" : "sp", "op" : "add", "p0" : [24.5, 141.2799348811, 75.094740162], "p1" : [24.5, 144.1083620058, 77.9231672868], "r" : 3 }, { "tag" : "spb", "op" : "cut", "p0" : [24.5, 140.9263814905, 74.7411867714], "p1" : [24.5, 144.4619153964, 78.2767206774], "r" : 1.65 }], "joints" : [], "pt" : [26.825, 142.6941484434, 76.5089537244] }, { "name" : "Pi spacer 2", "color" : [0.85, 0.85, 0.85], "lift" : true, "note" : "Printed, separate (4-40 clearance). Under the Pi's board.", "adds" : [], "cuts" : [], "cyls" : [{ "tag" : "sp", "op" : "add", "p0" : [-24.5, 141.2799348811, 75.094740162], "p1" : [-24.5, 144.1083620058, 77.9231672868], "r" : 3 }, { "tag" : "spb", "op" : "cut", "p0" : [-24.5, 140.9263814905, 74.7411867714], "p1" : [-24.5, 144.4619153964, 78.2767206774], "r" : 1.65 }], "joints" : [], "pt" : [-22.175, 142.6941484434, 76.5089537244] }, { "name" : "Pi spacer 3", "color" : [0.85, 0.85, 0.85], "lift" : true, "note" : "Printed, separate (4-40 clearance). Under the Pi's board.", "adds" : [], "cuts" : [], "cyls" : [{ "tag" : "sp", "op" : "add", "p0" : [24.5, 100.2677415723, 116.1069334708], "p1" : [24.5, 103.096168697, 118.9353605956], "r" : 3 }, { "tag" : "spb", "op" : "cut", "p0" : [24.5, 99.9141881817, 115.7533800802], "p1" : [24.5, 103.4497220876, 119.2889139862], "r" : 1.65 }], "joints" : [], "pt" : [26.825, 101.6819551346, 117.5211470332] }, { "name" : "Pi spacer 4", "color" : [0.85, 0.85, 0.85], "lift" : true, "note" : "Printed, separate (4-40 clearance). Under the Pi's board.", "adds" : [], "cuts" : [], "cyls" : [{ "tag" : "sp", "op" : "add", "p0" : [-24.5, 100.2677415723, 116.1069334708], "p1" : [-24.5, 103.096168697, 118.9353605956], "r" : 3 }, { "tag" : "spb", "op" : "cut", "p0" : [-24.5, 99.9141881817, 115.7533800802], "p1" : [-24.5, 103.4497220876, 119.2889139862], "r" : 1.65 }], "joints" : [], "pt" : [-22.175, 101.6819551346, 117.5211470332] }] };

/** A bike-frame vector from a plain [x, y, z] in mm. */
export function bk2V(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]) * millimeter;
}

/**
 * The battery + power carrier (cad_power): prisms on the bike's (y, z) over
 * x ranges, united, cut; then one 6-32 a side, the head in the rail (cut by
 * the drive), the nut and its slot here, and the ridge on the lug's face.
 * Printed; found again by a point in its floor.
 */
export function bk2Carrier(context is Context, id is Id, C is map) returns Query
{
    const mm = millimeter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const A  = id + "a";
    const K  = id + "k";
    const prism = function(scope is Id, p is map)
    {
        var pts = [];
        for (var q in p.pts)
            pts = append(pts, vector(q[0], q[1]) * mm);
        polyPrism(context, scope, p.tag, vector(p.x[0], 0, 0) * mm, X, Y, pts, (p.x[1] - p.x[0]) * mm);
    };
    // everything under A first, then everything under K: an Id's children
    // must be contiguous in the feature history
    for (var a in C.adds)
    {
        if (a.kind == "wedge")
        {
            // a profile in the DRIVE frame's (x, z), thick along its y: drawn
            // on an axis-aligned plane (in-plane (x, -z)) and turned about X
            // into the bike (an oblique sketch will not extrude reliably)
            var pts = [];
            for (var wq in a.pts)
                pts = append(pts, vector(wq[0], -wq[1]) * mm);
            polyPrism(context, A, a.tag, vector(0, a.y[0], 0) * mm, Y, X, pts, (a.y[1] - a.y[0]) * mm);
            opTransform(context, A + (a.tag ~ "Rot"), { "bodies" : qCreatedBy(A + (a.tag ~ "Ext"), EntityType.BODY),
                    "transform" : rotationAround(line(vector(0, 0, 0) * mm, X), a.rot * degree) });
        }
        else
            prism(A, a);
    }
    for (var c in C.cyls)
        if (c.op == "add")
            cylW(context, A + c.tag, bk2V(c.p0), bk2V(c.p1), c.r * mm);
    // a single body (a Pi spacer) has nothing to unite with
    const tools = qBodyType(qCreatedBy(A, EntityType.BODY), BodyType.SOLID);
    if (size(evaluateQuery(context, tools)) > 1)
        opBoolean(context, id + "u", { "tools" : tools, "operationType" : BooleanOperationType.UNION });
    for (var c in C.cuts)
        prism(K, c);
    for (var c in C.cyls)
        if (c.op != "add")
            cylW(context, K + c.tag, bk2V(c.p0), bk2V(c.p1), c.r * mm);
    const pt = bk2V(C.pt);
    if (size(C.cuts) > 0 || size(C.cyls) > 0)
        cutFrom(context, id + "cut", partAt(context, id, pt), [qBodyType(qCreatedBy(K, EntityType.BODY), BodyType.SOLID)]);
    for (var j in C.joints)
    {
        const J = id + ("j" ~ j.side);
        // the ridge FIRST, then the hole and the nut slot through it (the
        // other way round the ridge plugged the screw's way into the rail's
        // groove, 2026-10-16)
        const r = ridgeSolid(context, J + "ridge", bk2V(j.ridgeAt), Z, vector(j.ridgeUp[0], j.ridgeUp[1], j.ridgeUp[2]),
                             FIXTURE.ridge_height, 0 * mm, j.ridgeHalf * mm);
        // drawn along Z and turned about X onto the slide (the drive's Z)
        if (j.ridgeRot != undefined && abs(j.ridgeRot) > 1e-9)
            opTransform(context, J + "ridgeRot", { "bodies" : r,
                    "transform" : rotationAround(line(bk2V(j.ridgeAt), X), j.ridgeRot * degree) });
        opBoolean(context, J + "rU", { "tools" : qUnion([partAt(context, id, pt), r]),
                "operationType" : BooleanOperationType.UNION });
        const cs = coordSystem(bk2V(j.head), vector(j.slot[0], j.slot[1], j.slot[2]), vector(j.zIn[0], j.zIn[1], j.zIn[2]));
        screwJointGeometry(context, J, cs, mergeMaps(SCREW_OPT, {
                "nutDepth" : j.nutDepth * mm, "headPocket" : 0 * mm, "bothWays" : false,
                "nutSlotLength" : j.slotLen * mm, "holeDepth" : j.holeDepth * mm }));
        cutFrom(context, J + "cut", partAt(context, id, pt), [qCreatedBy(J + "screwRev", EntityType.BODY),
                                                             qCreatedBy(J + "slotExt", EntityType.BODY)]);
    }
    const q = partAt(context, id, pt);
    dress(context, q, C.name, color(C.color[0], C.color[1], C.color[2]), C.note);
    setAttribute(context, { "entities" : q, "name" : "bk_mod", "attribute" : "bike" });
    if (C.lift == true)
        setAttribute(context, { "entities" : q, "name" : "bk_lift", "attribute" : "pi" });
    return q;
}

/**
 * The whole bike, v2. opt: steer (angle), pose (a RG_POSES key), mocks
 * (bool: the electronics envelopes). Returns what the check sweeps.
 */
export function bikeV2Build(context is Context, id is Id, opt is map) returns map
{
    const mm = millimeter;
    const X  = vector(1, 0, 0);
    const Z  = vector(0, 0, 1);
    const O  = vector(0, 0, 0) * mm;
    const dXf = rotationAround(line(O, X), BK2.tilt * degree);
    const sXf = transform(bk2V(BK2.sOrigin)) * rotationAround(line(O, X), BK2.rake * degree);
    const rXf = transform(vector(0, BK2.yr, BK2.zr) * mm);
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };

    step("modules");
    const STi = id + "st";
    const DRi = id + "dr";
    const RGi = id + "rg";
    const rs = steeringBuild(context, STi, { "attach" : "SCREWS", "key" : "CROSS", "v2" : true });
    driveBuild(context, DRi, { "shift" : 0 * mm, "fixture" : false, "rail" : true });
    const rr = rightingBuild(context, RGi, { "mocks" : true });
    // the righting's floor mock and chassis placeholder: not in v2
    for (var k = 0; k < size(BK2.rgDrop); k += 1)
        opDeleteBodies(context, id + ("rgDrop" ~ k), { "entities" : partAt(context, RGi, bk2V(BK2.rgDrop[k])) });
    for (var g in [[STi, "steering"], [DRi, "drive"], [RGi, "righting"]])
        setAttribute(context, { "entities" : qBodyType(qCreatedBy(g[0], EntityType.BODY), BodyType.SOLID),
                                "name" : "bk_mod", "attribute" : g[1] });

    step("place");
    // tag what moves BEFORE anything does: the stage and group queries find
    // their bodies by a point inside, which a moved body no longer contains
    for (var q in rs.stages["steer"])
        setAttribute(context, { "entities" : q, "name" : "bk_steer", "attribute" : "steer" });
    for (var g in ["crank", "cR", "cL", "wR", "wL"])
        for (var q in rr.groups[g])
            setAttribute(context, { "entities" : q, "name" : "bk_" ~ g, "attribute" : g });
    if (opt.steer != undefined && opt.steer != 0 * degree)
        opTransform(context, id + "steer", { "bodies" : qHasAttribute("bk_steer"),
                "transform" : rotationAround(rs.axis, opt.steer) });
    if (opt.pose != undefined && opt.pose != "+0.00")
        rgPose(context, id + "pose", rr.groups, opt.pose, false);
    opTransform(context, id + "stXf", { "bodies" : qBodyType(qCreatedBy(STi, EntityType.BODY), BodyType.SOLID), "transform" : sXf });
    opTransform(context, id + "drXf", { "bodies" : qBodyType(qCreatedBy(DRi, EntityType.BODY), BodyType.SOLID), "transform" : dXf });
    opTransform(context, id + "rgXf", { "bodies" : qBodyType(qCreatedBy(RGi, EntityType.BODY), BodyType.SOLID), "transform" : rXf });

    step("components");
    if (opt.mocks != false)
        for (var k = 0; k < size(BK2.comps); k += 1)
        {
            const c = BK2.comps[k];
            var xf = transform(bk2V(c.at)) * rotationAround(line(O, X), c.rx * degree);
            if (c.frame == "drive")
                xf = dXf * xf;
            else if (c.frame == "steer")
                xf = sXf * xf;
            for (var i = 0; i < size(c.boxes); i += 1)
            {
                const b = c.boxes[i];
                const C = id + ("comp" ~ k ~ "_" ~ i);
                const q = boxIn(context, C, coordSystem(O, X, Z), bk2V(b.lo), bk2V(b.hi));
                // the Pi's mounting holes, through the board (its first box)
                if (i == 0 && c.holes != undefined)
                {
                    var hs = [];
                    for (var j = 0; j < size(c.holes.at); j += 1)
                    {
                        const a = c.holes.at[j];
                        // along the box's y (the Pi, the AHRS), or its x (the
                        // power board, which faces the side)
                        const ax = c.holes.ax == "x";
                        // NOT under C: boxIn's query is everything created under
                        // C, so a cutter there is its own target (BOOLEAN_INVALID)
                        hs = append(hs, cylW(context, id + ("compH" ~ k ~ "_" ~ j),
                                             bk2V(ax ? [c.holes.y[0], a[0], a[1]] : [a[0], c.holes.y[0], a[1]]),
                                             bk2V(ax ? [c.holes.y[1], a[0], a[1]] : [a[0], c.holes.y[1], a[1]]), c.holes.r * mm));
                    }
                    opBoolean(context, id + ("compHc" ~ k), { "tools" : qUnion(hs), "targets" : q,
                            "operationType" : BooleanOperationType.SUBTRACTION });
                }
                // its own id: boxIn already used C + "xf" for its own placement
                opTransform(context, id + ("compXf" ~ k ~ "_" ~ i), { "bodies" : q, "transform" : xf });
                dress(context, q, b.name, color(b.color[0], b.color[1], b.color[2]),
                      "Not a print. Free to move: config/bike_v2_cad.yaml.");
                setAttribute(context, { "entities" : q, "name" : "bk_mod", "attribute" : "bike" });
                // on the Pi carrier: the check lifts them out with it
                if (c.lift == true)
                    setAttribute(context, { "entities" : q, "name" : "bk_lift", "attribute" : "pi" });
            }
        }
    step("carriers");
    for (var k = 0; k < size(BK2.carriers); k += 1)
        bk2Carrier(context, id + ("pc" ~ k), BK2.carriers[k]);
    return { "steer" : rs, "righting" : rr, "rXf" : rXf,
             "sAxis" : line(bk2V(BK2.sOrigin), vector(BK2.sAxis[0], BK2.sAxis[1], BK2.sAxis[2])) };
}

// ==== UI LAYER BELOW -- dropped by --check ====

export enum BikeV2Pose
{
    annotation { "Name" : "Rest (stowed)" }
    REST,
    annotation { "Name" : "R 50 %" }
    R50,
    annotation { "Name" : "R 100 %" }
    R100,
    annotation { "Name" : "L 50 %" }
    L50,
    annotation { "Name" : "L 100 %" }
    L100
}

annotation { "Feature Type Name" : "AOW bike v2",
             "Feature Type Description" : "The whole bike, v2: drive, steering v2 and righting placed, the electronics free-floating, no chassis" }
export const aowBikeV2 = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Steer angle" }
        isAngle(definition.steer, { (degree) : [-180, 0, 180] } as AngleBoundSpec);
        annotation { "Name" : "Righting pose" }
        definition.pose is BikeV2Pose;
        annotation { "Name" : "Show the electronics and battery", "Default" : true }
        definition.mocks is boolean;
    }
    {
        const keys = { BikeV2Pose.REST : "+0.00", BikeV2Pose.R50 : "+0.50", BikeV2Pose.R100 : "+1.00",
                       BikeV2Pose.L50 : "-0.50", BikeV2Pose.L100 : "-1.00" };
        bikeV2Build(context, id + "build", { "steer" : definition.steer, "pose" : keys[definition.pose],
                                             "mocks" : definition.mocks });
        reportFeatureInfo(context, id, "Wheelbase 200.5 mm (CAD only); righting rod 120.5 mm ahead of the rear axle, 41.2 mm off the floor");
    });
