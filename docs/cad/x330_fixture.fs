FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_x330_fixture --push x330_fixture_features
 *
 * Numbers from config/x330_fixture_cad.yaml, config/servo_mounts.yaml and
 * bike_params_cad.yaml. Millimetres.
 *
 * WORLD FRAME: origin on the horn's outer face, on the shaft axis; +X out of
 * the horn; +Z up; the servo's far end DOWN. The lever rests level along Y,
 * an arm each way.
 */

export const X330 = {
    "caseDepth" : 23 * millimeter,
    "caseWidth" : 20 * millimeter,
    "caseHeight" : 34 * millimeter,
    "shaftFromEnd" : 9.5 * millimeter,
    "hornThickness" : 3 * millimeter,
    "hornDiameter" : 16 * millimeter
};

export const FIXTURE = {
    "horn_pin_length" : 2.6 * millimeter,
    "cover_cap_thickness" : 2.5 * millimeter,
    "lever_gap" : 1.5 * millimeter,
    "arm_thickness" : 6 * millimeter,
    "arm_height" : 12 * millimeter,
    "arm_length" : 50 * millimeter,
    "mass_slot_width" : 5.1 * millimeter,
    "mass_slot_r0" : 14 * millimeter,
    "mass_slot_r1" : 44 * millimeter,
    "tick_pitch" : 5 * millimeter,
    "tick_r0" : 10 * millimeter,
    "tick_width" : 0.6 * millimeter,
    "tick_depth" : 0.5 * millimeter,
    "leg_width" : 11 * millimeter,
    "sweep_clearance" : 1 * millimeter,
    "idler_arm_thickness" : 4 * millimeter,
    "adapter_top" : 6 * millimeter,
    "hub_thickness" : 6 * millimeter,
    "boss_dia" : 11 * millimeter,
    "boss_length" : 1 * millimeter,
    "slot_width" : 2.2 * millimeter,
    "slot_depth" : 3 * millimeter,
    "key_thickness" : 2 * millimeter,
    "key_span" : 16 * millimeter,
    "key_length" : 5.6 * millimeter,
    "bearing_bore" : 8 * millimeter,
    "bearing_od" : 22 * millimeter,
    "bearing_width" : 7 * millimeter,
    "bearing_gap" : 2 * millimeter,
    "bore_clearance" : 0.15 * millimeter,
    "shaft_clearance" : 0.1 * millimeter,
    "block_wall" : 3 * millimeter,
    "cradle_clearance" : 0.3 * millimeter,
    "cradle_wall" : 3 * millimeter,
    "cradle_height" : 20 * millimeter,
    "spring_od" : 6 * millimeter,
    "spring_gap" : 2 * millimeter,
    "spring_height" : 12.5 * millimeter,
    "stop_deg" : 45 * degree,
    "stop_flat" : 1 * millimeter,
    "stop_contact" : 5 * millimeter,
    "ledge_thickness" : 3 * millimeter,
    "yoke_stop_length" : 17.5 * millimeter,
    "post_stop_deg" : 10 * degree,
    "stop_radius" : 40 * millimeter,
    "stop_post" : 6 * millimeter,
    "base_gap" : 5 * millimeter,
    "cover_leg_gap" : 0.2 * millimeter,
    "head_recess" : 1 * millimeter,
    "plate_rim" : 3 * millimeter,
    "tab_beyond" : 50 * millimeter,
    "bolt_hole_dia" : 5.1 * millimeter,
    "bolt_span_x" : 38.1 * millimeter,
    "bolt_span_y" : 25.4 * millimeter,
    "joint_plate" : 4.6 * millimeter,
    "ridge_height" : 1.2 * millimeter,
    "ridge_flat" : 4.6 * millimeter,
    "ridge_clearance" : 0.15 * millimeter,
    "bridge_layer" : 0.2 * millimeter
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
 * The lever's two arms, level along Y between x0 and x1, arm_length each
 * way. Returns the arms and, as ONE query of cutters, each arm's mass slot
 * (through along X, a 10-32 slides in it) and the ticks across the top face
 * every tick_pitch from the shaft axis -- full width every second one, half
 * width from the front (x1) face between.
 */
export function leverArms(context is Context, id is Id, x0 is ValueWithUnits,
                          x1 is ValueWithUnits) returns map
{
    const f  = FIXTURE;
    const mm = millimeter;
    const hA = f.arm_height / 2;
    const arms = boxW(context, id + "arms", vector(x0, -f.arm_length, -hA),
                      vector(x1, f.arm_length, hA));
    const w = f.mass_slot_width / 2;
    var cut = [];
    for (var sg in [1, -1])
    {
        const t = sg > 0 ? "P" : "N";
        cut = append(cut, boxW(context, id + ("slot" ~ t),
                vector(x0 - 1 * mm, sg > 0 ? f.mass_slot_r0 : -f.mass_slot_r1, -w),
                vector(x1 + 1 * mm, sg > 0 ? f.mass_slot_r1 : -f.mass_slot_r0, w)));
        for (var r in [f.mass_slot_r0, f.mass_slot_r1])
            cut = append(cut, cylW(context, id + ("slotEnd" ~ t ~ roundToPrecision(r / mm, 0)),
                    vector(x0 - 1 * mm, sg * r, 0 * mm), vector(x1 + 1 * mm, sg * r, 0 * mm), w));
        var k = 0;
        for (var r = f.tick_r0; r < f.arm_length - f.tick_width; r += f.tick_pitch)
        {
            const major = abs(roundToPrecision(r / f.tick_pitch, 0) % 2) < 0.5;
            const xa = major ? x0 - 1 * mm : (x0 + x1) / 2;
            cut = append(cut, boxW(context, id + ("tick" ~ t ~ k),
                    vector(xa, sg * r - f.tick_width / 2, hA - f.tick_depth),
                    vector(x1 + 1 * mm, sg * r + f.tick_width / 2, hA + 1 * mm)));
            k += 1;
        }
    }
    return { "arms" : arms, "holes" : qUnion(cut), "leverEnd" : f.arm_length };
}

/**
 * The clamp plate, x0..x1 by +-yHalf under the servo, with the clamp tab
 * behind it to -X and four 10-32 holes. Top face at zTop; joint_plate +
 * head_recess thick, so a countersunk head from below sits in a counterbore.
 * `posts`: a stop post under each arm, centred on postX, that catches the
 * lever stop_deg below level (the SCREWDRIVER version's stop).
 */
export function clampPlate(context is Context, id is Id, x0 is ValueWithUnits,
                           x1 is ValueWithUnits, yHalf is ValueWithUnits,
                           zTop is ValueWithUnits, posts is boolean,
                           postX is ValueWithUnits) returns map
{
    const f  = FIXTURE;
    const mm = millimeter;
    const postR = f.stop_radius;
    const postH = f.stop_post / 2;
    const rOut  = postR + postH;
    const postTop = -(rOut * sin(f.post_stop_deg) + f.arm_height / 2 * cos(f.post_stop_deg));
    const xTab = x0 - f.tab_beyond;
    const yW   = posts ? max(yHalf, rOut + f.plate_rim) : yHalf;
    const zBot = zTop - f.joint_plate - f.head_recess;
    var bodies = [boxW(context, id + "plate", vector(xTab, -yW, zBot), vector(x1, yW, zTop))];
    if (posts)
        for (var sg in [1, -1])
            bodies = append(bodies, boxW(context, id + ("post" ~ (sg > 0 ? "P" : "N")),
                    vector(postX - postH, sg > 0 ? postR - postH : -rOut, zTop - 1 * mm),
                    vector(postX + postH, sg > 0 ? rOut : -(postR - postH), postTop)));
    if (posts)                  // a union of ONE body is a BOOLEAN_BAD_INPUT
        unite(context, id + "u", bodies);
    const pt = vector(xTab + 2 * mm, yW - 2 * mm, zBot + 1 * mm);
    const xb = (xTab + x0) / 2;
    var bolts = [];
    for (var i = 0; i < 4; i += 1)
    {
        const bx = xb + ((i < 2) ? 1 : -1) * f.bolt_span_x / 2;
        const by = ((i % 2 == 0) ? 1 : -1) * f.bolt_span_y / 2;
        bolts = append(bolts, cylW(context, id + ("bolt" ~ i), vector(bx, by, zBot - 1 * mm),
                                   vector(bx, by, zTop + 1 * mm), f.bolt_hole_dia / 2));
    }
    return { "pt" : pt, "bolts" : qUnion(bolts), "zBot" : zBot, "postTop" : postTop,
             "head" : zBot + f.head_recess };
}

/**
 * Every part and the hardware, for `opt.version` IDLER or SCREWDRIVER.
 * Returns the derived layout, and for the check: which bodies are static and
 * which turn with the horn, and each printed part with the direction that
 * prints UP.
 */
export function x330FixtureBuild(context is Context, id is Id, opt is map) returns map
{
    const f  = FIXTURE;
    const x  = X330;
    const mm = millimeter;
    const O  = vector(0, 0, 0) * meter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const P  = id + "parts";
    const idler = opt.version != "SCREWDRIVER";
    // step markers for the check's console: which boolean an error follows
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };

    const wellT   = x.hornThickness - HORN_OPT.caseOffset;
    const collarR = HORN_OPT.collarOuterDia / 2;
    const backZ   = -(x.hornThickness + x.caseDepth);
    const hornZ   = -x.hornThickness;
    const farEnd  = x.caseHeight - x.shaftFromEnd;
    const horn    = mergeMaps(HORN_OPT, { "pinLength" : f.horn_pin_length });
    const jpT     = f.joint_plate;

    // ---- the servo: horn +X, far end DOWN (local y = cross(X, -Y) = -Z)
    const cs = coordSystem(O, -Y, X);
    const sv = x330Envelope(context, id + "hw", cs);
    const HW = id + "hw2";

    var parts = [];                   // [name, point, print label, print up, stage]
    var stages = { "static" : [sv.caseQ], "lever" : [sv.horn] };
    var L = {};
    var leverPt;
    if (idler)
    {
        const shellY0 = farEnd - CASE_OPT.caseWrapLength;
        const shellY1 = farEnd + CASE_OPT.caseSideClearance + CASE_OPT.caseTopWall
                        + CASE_OPT.caseNestClearance + CASE_OPT.caseBottomWall;
        const botOuter = x.caseWidth / 2 + CASE_OPT.caseSideClearance + CASE_OPT.caseTopWall
                         + CASE_OPT.caseNestClearance + CASE_OPT.caseBottomWall;
        const capOut  = backZ - CASE_OPT.caseFaceClearance - CASE_OPT.caseCapThickness;
        const nestTop = backZ + CASE_OPT.caseGripLength + CASE_OPT.caseNestLength;
        // the yoke's U swings round the full-wrap cover's shaft-end corner
        const walls  = CASE_OPT.caseSideClearance + CASE_OPT.caseTopWall;
        const cy = x.shaftFromEnd + walls;
        const cx = x.caseWidth / 2 + walls;
        const lw = f.leg_width;
        // the stop: flat landing on the cover's top needs it to reach past
        // cy * tan(stop) (plus the centre flat's reach); ledges widen it to W
        const sS = sin(f.stop_deg);
        const cS = cos(f.stop_deg);
        const a  = f.stop_flat;
        const W  = (a + cy * sS) / cS + f.stop_contact;
        const yExt = W * cS - cy * sS;          // V face ends on the circle through (W, cy)
        const Rs = max(sqrt(cy * cy + cx * cx), sqrt(cy * cy + W * W)) + f.sweep_clearance + lw / 2;
        // The cover's cap ends just under the horn face; the lever sits
        // lever_gap past it.
        const capOuter = hornZ + CASE_OPT.caseFaceClearance + f.cover_cap_thickness;
        const armX0 = capOuter + f.lever_gap;
        const leverTop = armX0 + f.arm_thickness;
        const xJ = leverTop - jpT;

        step("lever");
        // ---- lever: disc over the horn, arms both ways, leg up to the yoke
        const disc = cylW(context, P + "disc", vector(-wellT, 0 * mm, 0 * mm),
                          vector(leverTop, 0 * mm, 0 * mm), collarR);
        const la = leverArms(context, P + "lv", armX0, leverTop);
        const leg = boxW(context, P + "leg", vector(xJ, -lw / 2, 0 * mm),
                         vector(leverTop, lw / 2, Rs + lw / 2));
        unite(context, P + "leverU", [disc, la.arms, leg]);
        // in the arm's wall above the mass slot, below the ticks
        leverPt = vector((armX0 + leverTop) / 2, -30 * mm, (f.mass_slot_width + f.arm_height) / 4);
        step("hornPins");
        servoMountBuild(context, P + "horn", cs, horn, partAt(context, P, leverPt));
        step("armCut");
        opBoolean(context, P + "armCut", { "tools" : la.holes,
                "targets" : partAt(context, P, leverPt),
                "operationType" : BooleanOperationType.SUBTRACTION });

        // ---- base and cover: each shell with a leg down to the plate, so the
        // masses' off-axis torque reaches the plate through both
        const zTop = -shellY1 - f.base_gap;
        const xCv0 = nestTop + f.cover_leg_gap;
        boxW(context, P + "baseLeg", vector(capOut, -botOuter, zTop), vector(nestTop, botOuter, -shellY0));
        const basePt = vector(capOut + 2 * mm, botOuter - 2 * mm, zTop + 1 * mm);
        step("base");
        caseShellBuild(context, P + "base", cs, mergeMaps(CASE_OPT, { "part" : "BOTTOM" }),
                       partAt(context, P, basePt));
        // The cover is built on its own and the leg added after: the shell's
        // own print cuts are made for a standalone part and would carve a leg
        // given as its target. The leg reaches 1 mm into the far-end wall
        // (outside at -(farEnd + side clearance + top wall)), never the case.
        step("cover");
        caseShellBuild(context, P + "cover", cs, mergeMaps(CASE_OPT, { "part" : "TOP", "fullWrap" : true,
                       "caseCapThickness" : f.cover_cap_thickness }), qNothing());
        const cvWall = -(farEnd + CASE_OPT.caseSideClearance + CASE_OPT.caseTopWall);
        const cvLeg = boxW(context, P + "coverLeg", vector(xCv0, -botOuter, zTop),
                           vector(capOuter, botOuter, cvWall + 1 * mm));
        step("coverU");
        opBoolean(context, P + "coverU", { "tools" : qUnion([cvLeg,
                partAt(context, P, vector(-1 * mm, 0 * mm, -20 * mm))]),
                "operationType" : BooleanOperationType.UNION });
        const coverPt = vector(capOuter - 1 * mm, botOuter - 2 * mm, zTop + 1 * mm);
        // ---- the cover's own flat top, measured: where the ledges start
        var xa = 1 * meter;
        var xb = -1 * meter;
        for (var fc in evaluateQuery(context, qGeometry(qOwnedByBody(partAt(context, P, coverPt),
                                                                     EntityType.FACE), GeometryType.PLANE)))
        {
            const pl = evPlane(context, { "face" : fc });
            if (dot(pl.normal, Z) > 0.999 && abs(pl.origin[2] - cy) < 0.01 * mm)
            {
                const bb = evBox3d(context, { "topology" : fc, "tight" : true });
                xa = min(xa, bb.minCorner[0]);
                xb = max(xb, bb.maxCorner[0]);
            }
        }
        if (xb < xa)
            throw regenError("x330 fixture: no top face found on the cover");

        // ---- the ledges: the cover's top widened to W. Each grows out of the
        // side wall the way the shell's end wall grows round its corners
        // (caseShellGeometry): a cone at the shells' overhang slope about the
        // ledge's INNER corner (its underside meeting the side wall's outer
        // face), apex where the side wall's own sloped bed-ward edge is at
        // that height. Every layer then grows out of the one below it; the
        // ledge does not start a wall's worth too high and hang a level strip
        // off the wall. The cover prints -X up.
        const tanO = tan(90 * degree - SERVO_MOUNT_TABLE["XC330"].caseWrapOverhang);
        const z0 = cy - f.ledge_thickness;
        // the side wall's bed-ward edge at height z, from the shell's slope
        // (through the cap face at the wrap's open end)
        const xWall = function(z) returns ValueWithUnits
        {
            return hornZ - (farEnd - CASE_OPT.caseWrapLength + z) * tanO;
        };
        if (abs(xWall(cy) - xb) > 0.01 * mm)
            throw regenError("x330 fixture: the cover's slope is not where the ledges assume");
        const X0 = xWall(z0);
        const Rl = sqrt((W - cx + 1 * mm) * (W - cx + 1 * mm) + f.ledge_thickness * f.ledge_thickness) + 1 * mm;
        var ledges = [partAt(context, P, coverPt)];
        for (var sg in [1, -1])
        {
            const tag = sg > 0 ? "ledgeP" : "ledgeN";
            const slab = boxW(context, P + tag, vector(backZ, sg > 0 ? cx - 0.5 * mm : -W, z0),
                              vector(X0 + 1 * mm, sg > 0 ? W : -(cx - 0.5 * mm), cy));
            // everything bed-ward of the cone about the corner line (y = sg cx, z = z0)
            const pivot = vector(0 * mm, sg * cx, z0);
            const rad = sg * Y;
            revolveProfile(context, P, tag ~ "Cone", plane(pivot, cross(rad, X), rad), line(pivot, X),
                           [vector(0 * mm, X0), vector(Rl, X0 - Rl * tanO),
                            vector(Rl, capOuter + 2 * mm), vector(0 * mm, capOuter + 2 * mm)]);
            opBoolean(context, P + (tag ~ "Trim"), {
                    "tools" : qCreatedBy(P + (tag ~ "ConeRev"), EntityType.BODY),
                    "targets" : slab,
                    "operationType" : BooleanOperationType.SUBTRACTION });
            ledges = append(ledges, slab);
        }
        step("ledges");
        opBoolean(context, P + "ledgeU", { "tools" : qUnion(ledges),
                "operationType" : BooleanOperationType.UNION });

        // ---- yoke: idler plug, arm up behind the back face, bridge forward,
        // and the STOP: a V underside, each face at stop_deg, so the yoke
        // turned stop_deg either way lands one face FLAT on the cover's
        // widened top (z = cy). In the yoke's frame that face is the plane
        // z = (cy + |y| sin) / cos, from the centre flat (|y| = a) out to the
        // circle through the ledge corner (|y| = yExt). It runs
        // yoke_stop_length from the yoke's bed (as printed).
        step("yoke");
        const tI = f.idler_arm_thickness;
        const zT = Rs - lw / 2 + 1 * mm;
        const yArm = boxW(context, P + "yokeArm", vector(backZ - tI, -lw / 2, 0 * mm),
                          vector(backZ - 0.5 * mm, lw / 2, Rs + lw / 2));
        const yBr  = boxW(context, P + "yokeBridge", vector(backZ - tI, -lw / 2, Rs - lw / 2),
                          vector(xJ, lw / 2, Rs + lw / 2));
        const xS1 = min(backZ - tI + f.yoke_stop_length, xJ);
        const zV = function(y) returns ValueWithUnits { return (cy + y * sS) / cS; };
        polyPrism(context, P, "yokeStop", vector(backZ - tI, 0 * mm, 0 * mm), X, Y,
                  [vector(-yExt, zT), vector(-yExt, zV(yExt)), vector(-a, zV(a)),
                   vector(a, zV(a)), vector(yExt, zV(yExt)), vector(yExt, zT)], xS1 - (backZ - tI));
        unite(context, P + "yokeU", [yArm, yBr, qCreatedBy(P + "yokeStopExt", EntityType.BODY)]);
        const yokePt = vector((backZ + xJ) / 2, 0 * mm, Rs);
        step("idler");
        idlerBuild(context, P + "idler", cs, mergeMaps(IDLER_OPT, {
                "idlerCollarDia" : lw, "idlerCollarThickness" : tI }),
                partAt(context, P, yokePt));

        step("plate");
        const pl = clampPlate(context, P + "pl", capOut - f.plate_rim, capOuter + f.plate_rim,
                              botOuter + f.plate_rim, zTop, false, 0 * mm);
        step("bolts");
        opBoolean(context, P + "boltHoles", { "tools" : pl.bolts,
                "targets" : partAt(context, P, pl.pt),
                "operationType" : BooleanOperationType.SUBTRACTION });

        // ---- joints: each leg onto the plate from below, the head counter-
        // bored under flush so the plate clamps flat, the nut slots in line
        // along X (UP as base and cover print), the locating ridge ON each
        // leg and the groove in the plate -- a groove in a leg left long thin
        // walls either side of it; the lever's leg onto the
        // yoke's bridge, the AHRS fixture's roll-stage joint exactly
        const plateQ = partAt(context, P, pl.pt);
        step("j1");
        screwJoint(context, P + "j1", vector((capOut + nestTop) / 2, 0 * mm, pl.head), Z, X,
                   f.head_recess, plateQ, partAt(context, P, basePt), true, X,
                   (nestTop - capOut) / 2 - 1 * mm, Z, X);
        step("j3");
        screwJoint(context, P + "j3", vector((xCv0 + capOuter) / 2, 0 * mm, pl.head), Z, X,
                   f.head_recess, partAt(context, P, pl.pt), partAt(context, P, coverPt), true, X,
                   (capOuter - xCv0) / 2 - 1 * mm, Z, -X);
        step("j2");
        screwJoint(context, P + "j2", vector(leverTop, 0 * mm, Rs), -X, Z, 0 * mm,
                   partAt(context, P, leverPt), partAt(context, P, yokePt), true, Y, lw / 2, -X, X);

        parts = [["plate", pl.pt, "Z+", Z, "static"], ["base", basePt, "X+", X, "static"],
                 ["cover", coverPt, "X-", -X, "static"], ["lever", leverPt, "X-", -X, "lever"],
                 ["yoke", yokePt, "X+", X, "lever"]];
        L = { "plateTop" : zTop, "plateBottom" : pl.zBot, "stopHeight" : (cy + a * sS) / cS,
              "ledgeHalfWidth" : W,
              "leverEnd" : la.leverEnd, "R_s" : Rs, "leverTop" : leverTop,
              "coverTopX0" : xa, "coverTopX1" : xb };
    }
    else
    {
        const a0 = f.adapter_top;
        const h1 = a0 + f.hub_thickness;
        const bx0 = h1 + f.boss_length;
        const bw  = f.bearing_width;
        const bx1 = bx0 + 2 * bw + f.bearing_gap;
        const rB  = f.bearing_od / 2;
        const zTop = -farEnd;                   // the servo stands on the plate
        const sw = f.slot_width / 2;

        // ---- adapter: horn pins, a key slot across its outer face
        cylW(context, P + "adDisc", vector(-wellT, 0 * mm, 0 * mm), vector(a0, 0 * mm, 0 * mm), collarR);
        const adPt = vector(1.5 * mm, 0 * mm, 6 * mm);
        servoMountBuild(context, P + "adHorn", cs, horn, partAt(context, P, adPt));
        const adSlot = boxW(context, P + "adSlot", vector(a0 - f.slot_depth, -collarR - 1 * mm, -sw),
                            vector(a0 + 1 * mm, collarR + 1 * mm, sw));
        opBoolean(context, P + "adSlotCut", { "tools" : adSlot, "targets" : partAt(context, P, adPt),
                "operationType" : BooleanOperationType.SUBTRACTION });

        // ---- lever: hub with a key slot, arms both ways, boss, shaft
        const hub  = cylW(context, P + "hub", vector(a0, 0 * mm, 0 * mm), vector(h1, 0 * mm, 0 * mm), collarR);
        const la   = leverArms(context, P + "lv", a0, h1);
        const boss = cylW(context, P + "boss", vector(h1 - 0.5 * mm, 0 * mm, 0 * mm),
                          vector(bx0, 0 * mm, 0 * mm), f.boss_dia / 2);
        const shaft = cylW(context, P + "shaft", vector(h1 - 0.5 * mm, 0 * mm, 0 * mm),
                           vector(bx1 + 1 * mm, 0 * mm, 0 * mm), (f.bearing_bore - f.shaft_clearance) / 2);
        unite(context, P + "leverU", [hub, la.arms, boss, shaft]);
        leverPt = vector((a0 + h1) / 2, -30 * mm, (f.mass_slot_width + f.arm_height) / 4);
        const hubSlot = boxW(context, P + "hubSlot", vector(a0 - 1 * mm, -collarR - 1 * mm, -sw),
                             vector(a0 + f.slot_depth, collarR + 1 * mm, sw));
        opBoolean(context, P + "hubCut", { "tools" : qUnion([hubSlot, la.holes]),
                "targets" : partAt(context, P, leverPt),
                "operationType" : BooleanOperationType.SUBTRACTION });

        // ---- key: loose, between the two slots, float at both ends
        const kx0 = a0 - f.slot_depth + (2 * f.slot_depth - f.key_length) / 2;
        boxW(context, P + "key", vector(kx0, -f.key_span / 2, -f.key_thickness / 2),
             vector(kx0 + f.key_length, f.key_span / 2, f.key_thickness / 2));
        const keyPt = vector(kx0 + f.key_length / 2, 0 * mm, 0 * mm);

        // ---- bearing block: two 608s flush with its faces
        const yB = rB + f.block_wall;
        const blk = boxW(context, P + "block", vector(bx0, -yB, zTop), vector(bx1, yB, rB + f.block_wall));
        const bore = cylW(context, P + "bore", vector(bx0 - 1 * mm, 0 * mm, 0 * mm),
                          vector(bx1 + 1 * mm, 0 * mm, 0 * mm), (f.bearing_od + f.bore_clearance) / 2);
        opBoolean(context, P + "boreCut", { "tools" : bore, "targets" : blk,
                "operationType" : BooleanOperationType.SUBTRACTION });
        const blockPt = vector((bx0 + bx1) / 2, yB - 1 * mm, zTop + 1 * mm);

        // ---- plate with the cradle: side walls round the case with float,
        // a back wall behind the spring, the spring's channel open at the top
        const inY  = x.caseWidth / 2 + f.cradle_clearance;
        const outY = inY + f.cradle_wall;
        const xbw  = backZ - f.spring_gap - f.cradle_wall;
        const zC   = zTop + f.cradle_height;
        const pl = clampPlate(context, P + "pl", xbw - f.plate_rim, bx1 + f.plate_rim,
                              max(outY, rB + f.block_wall) + f.plate_rim, zTop, true, (a0 + h1) / 2);
        var cradle = [partAt(context, P, pl.pt)];
        for (var sg in [1, -1])
            cradle = append(cradle, boxW(context, P + ("wall" ~ (sg > 0 ? "P" : "N")),
                    vector(xbw, sg > 0 ? inY : -outY, zTop - 1 * mm),
                    vector(hornZ - 1 * mm, sg > 0 ? outY : -inY, zC)));
        cradle = append(cradle, boxW(context, P + "backWall", vector(xbw, -outY, zTop - 1 * mm),
                                     vector(xbw + f.cradle_wall, outY, zC)));
        opBoolean(context, P + "cradleU", { "tools" : qUnion(cradle),
                "operationType" : BooleanOperationType.UNION });
        const zS = zTop + f.spring_height;
        const rS = f.spring_od / 2;
        const chan = boxW(context, P + "chan", vector(xbw + 1 * mm, -(rS + 0.2 * mm), zS - rS - 0.2 * mm),
                          vector(backZ, rS + 0.2 * mm, zC + 1 * mm));
        opBoolean(context, P + "plateCut", { "tools" : qUnion([pl.bolts, chan]),
                "targets" : partAt(context, P, pl.pt),
                "operationType" : BooleanOperationType.SUBTRACTION });

        // ---- joint: block onto the plate from below, as the base is in IDLER
        screwJoint(context, P + "j1", vector((bx0 + bx1) / 2, 0 * mm, pl.head), Z, X, f.head_recess,
                   partAt(context, P, pl.pt), partAt(context, P, blockPt), true, X,
                   (bx1 - bx0) / 2 - 1 * mm, Z, X);

        // ---- hardware: the two bearings, the spring
        var hw = [];
        for (var k in [0, 1])
        {
            const x0 = k == 0 ? bx0 : bx1 - bw;
            const ring = cylW(context, HW + ("brg" ~ k), vector(x0, 0 * mm, 0 * mm),
                              vector(x0 + bw, 0 * mm, 0 * mm), rB);
            const hole = cylW(context, HW + ("brgH" ~ k), vector(x0 - 1 * mm, 0 * mm, 0 * mm),
                              vector(x0 + bw + 1 * mm, 0 * mm, 0 * mm), f.bearing_bore / 2);
            opBoolean(context, HW + ("brgC" ~ k), { "tools" : hole, "targets" : ring,
                    "operationType" : BooleanOperationType.SUBTRACTION });
            dress(context, ring, "608 bearing " ~ (k + 1), color(0.75, 0.76, 0.8), "");
            hw = append(hw, ring);
        }
        const spring = cylW(context, HW + "spring", vector(xbw + 1 * mm, 0 * mm, zS),
                            vector(backZ, 0 * mm, zS), rS);
        dress(context, spring, "spring", color(0.75, 0.76, 0.8), "");
        stages["static"] = concatenateArrays([stages["static"], hw, [spring]]);

        parts = [["plate", pl.pt, "Z+", Z, "static"], ["block", blockPt, "X+", X, "static"],
                 ["adapter", adPt, "X-", -X, "lever"], ["key", keyPt, "Z+", Z, "lever"],
                 ["lever", leverPt, "X+", X, "lever"]];
        L = { "plateTop" : zTop, "plateBottom" : pl.zBot, "postTop" : pl.postTop,
              "leverEnd" : la.leverEnd, "blockX0" : bx0, "blockX1" : bx1 };
    }

    // ---- names, colours, print orientation
    const baseC  = color(0.62, 0.64, 0.68);
    const leverC = color(0.93, 0.56, 0.20);
    var prints = [];
    for (var n in parts)
    {
        const q = partAt(context, P, n[1]);
        dress(context, q, n[0] ~ " [print " ~ n[2] ~ "]", n[4] == "lever" ? leverC : baseC,
              "Print with the " ~ n[2] ~ " side facing UP (native pose).");
        stages[n[4]] = append(stages[n[4]], q);
        prints = append(prints, [n[0], q, n[3]]);
    }
    dress(context, sv.caseQ, "X330 case", color(0.16, 0.16, 0.18), "");
    dress(context, sv.horn, "X330 horn", color(0.3, 0.3, 0.32), "");
    opMateConnector(context, id + "hornMC", { "coordSystem" : cs, "owner" : sv.caseQ });
    return { "stages" : stages, "prints" : prints, "shaft" : line(O, X),
             "lever" : partAt(context, P, leverPt), "L" : L };
}

// ==== UI LAYER BELOW -- dropped by --check ====

export enum X330FixtureVersion
{
    annotation { "Name" : "Idler: lever on the horn, yoke round to the idler (easiest build)" }
    IDLER,
    annotation { "Name" : "Screwdriver: servo in a cradle drives a lever on its own bearings (like the rig)" }
    SCREWDRIVER
}

annotation { "Feature Type Name" : "X330 fixture",
             "Feature Type Description" : "One X330, shaft horizontal, a two-armed lever for hung-mass torque tests" }
export const x330Fixture = defineFeature(function(context is Context, id is Id,
                                                  definition is map)
    precondition
    {
        annotation { "Name" : "Version" }
        definition.version is X330FixtureVersion;

        annotation { "Name" : "Lever angle (+ = -Y arm down)" }
        isAngle(definition.leverAngle, { (degree) : [-45, 0, 45] } as AngleBoundSpec);
    }
    {
        const r = x330FixtureBuild(context, id + "build", {
                "version" : definition.version == X330FixtureVersion.SCREWDRIVER ? "SCREWDRIVER" : "IDLER" });
        if (definition.leverAngle != 0 * degree)
            opTransform(context, id + "swing", {
                    "bodies" : qUnion(r.stages["lever"]),
                    "transform" : rotationAround(r.shaft, definition.leverAngle) });
        reportFeatureInfo(context, id, "Shaft axis " ~ toString(roundToPrecision(-r.L.plateBottom / millimeter, 2))
                ~ " mm above the plate's bottom face; lever reach +-"
                ~ toString(roundToPrecision(r.L.leverEnd / millimeter, 1)) ~ " mm");
    });
