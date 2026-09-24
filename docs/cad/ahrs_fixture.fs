FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_ahrs_fixture --push fixture_features
 *
 * Numbers from config/ahrs_fixture_cad.yaml, config/servo_mounts.yaml and
 * bike_params_cad.yaml. Millimetres.
 *
 * WORLD FRAME: origin on the yaw horn's outer face, on the yaw axis; +Z up
 * the yaw axis; +X along the roll axis, from the roll servo toward the TM151.
 */

export const TM151 = {
    "board_length" : 40 * millimeter,
    "board_width" : 34 * millimeter,
    "board_thickness" : 1.6 * millimeter,
    "housing_length" : 26 * millimeter,
    "housing_width" : 30 * millimeter,
    "housing_height" : 11 * millimeter,
    "housing_offset_x" : 0.66 * millimeter,
    "hole_dia" : 2.1 * millimeter,
    "hole_span_x" : 31 * millimeter,
    "hole_span_y" : 30 * millimeter,
    "hole_offset_x" : 0.45 * millimeter,
    "usb_width" : 9.1 * millimeter,
    "usb_length" : 7.6 * millimeter,
    "usb_height" : 3.4 * millimeter,
    "usb_offset_y" : -6.2 * millimeter,
    "usb_plug_keepout" : [32 * millimeter, 13 * millimeter, 7 * millimeter],
    "sensor_height" : 7.1 * millimeter
};

export const X330 = {
    "caseDepth" : 23 * millimeter,
    "caseWidth" : 20 * millimeter,
    "caseHeight" : 34 * millimeter,
    "shaftFromEnd" : 9.5 * millimeter,
    "hornThickness" : 3 * millimeter,
    "hornDiameter" : 16 * millimeter
};

export const FIXTURE = {
    "ahrs_offset" : 30 * millimeter,
    "stage_clearance" : 8 * millimeter,
    "arm_plate" : 3 * millimeter,
    "mount_wall_top" : 6 * millimeter,
    "tm151_gap" : 1 * millimeter,
    "horn_mount_thickness" : 8 * millimeter,
    "joint_plate" : 4.6 * millimeter,
    "leg_width" : 11 * millimeter,
    "sweep_clearance" : 1 * millimeter,
    "idler_arm_thickness" : 4 * millimeter,
    "yaw_arm_top" : 4 * millimeter,
    "yaw_arm_width" : 20 * millimeter,
    "yaw_boss_bottom" : 6 * millimeter,
    "base_gap" : 1.5 * millimeter,
    "base_plate" : 4 * millimeter,
    "base_width" : 40 * millimeter,
    "tab_beyond" : 25 * millimeter,
    "bolt_hole_dia" : 5.1 * millimeter,
    "bolt_span_x" : 38.1 * millimeter,
    "bolt_span_y" : 25.4 * millimeter,
    "cable_clip_channel" : 3 * millimeter,
    "cable_clip_wall" : 1.5 * millimeter,
    "cable_clip_height" : 6 * millimeter,
    "cable_clip_length" : 4 * millimeter,
    "ridge_height" : 1.2 * millimeter,
    "ridge_flat" : 4.6 * millimeter,
    "ridge_clearance" : 0.15 * millimeter,
    "tm151_pin_clearance" : 0.2 * millimeter,
    "tm151_pin_length" : 1.4 * millimeter,
    "mount_rim" : 1.2 * millimeter,
    "bridge_layer" : 0.2 * millimeter,
    "travel_deg" : 45 * degree
};

export const HORN_OPT = {
    "servo" : "XC330",
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

export const IDLER_OPT = {
    "servo" : "XC330",
    "idlerPlugClearance" : 0.2 * millimeter,
    "idlerEndGap" : 0 * millimeter,
    "idlerFaceGap" : 0 * millimeter,
    "idlerCollarDia" : 11 * millimeter,
    "idlerCollarThickness" : 3 * millimeter
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
        revolveProfile(context, P + ("pin" ~ i), "p", plane(c, Y, X), line(c, Z), [vector(0 * millimeter, 0 * millimeter),
            vector(pinR, 0 * millimeter),
            vector(pinR, -pinL + tipCh),
            vector(pinR - tipCh, -pinL),
            vector(0 * millimeter, -pinL)]);
        revolveProfile(context, P + ("rel" ~ i), "r", plane(c, Y, X), line(c, Z), [vector(pinR, 0 * millimeter),
            vector(pinR + relW, 0 * millimeter),
            vector(pinR + relW, relD),
            vector(pinR + relCh, relD),
            vector(pinR, relD - relCh)]);
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

// ==== UI LAYER BELOW -- dropped by --check ====

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
        isLength(definition.ahrsOffset, { (millimeter) : [0, 30, 200] } as LengthBoundSpec);

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
            isLength(definition.sensorHeight, { (millimeter) : [0, 7.1, 12.6] } as LengthBoundSpec);
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
