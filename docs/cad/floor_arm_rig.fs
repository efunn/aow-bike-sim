FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* The floor arm rig -- GEOMETRY ONLY, best estimate of 2026-09-25.
 * Hand-written, not generated: this file IS the source, pushed as-is into
 * the `floor-rig-gen` Feature Studio of aow-bike (tabs.floor_rig_gen in
 * config/onshape.yaml) and inserted in `aow-bike-rig`, around the real bike
 * parts already drawn there.
 *
 * Plan: docs/plans/pre-assembly-bench-checklist.md, "Rig 1 -- the floor arm".
 *
 * FRAME = the aow-bike-rig tab's own: rear wheel centre at the origin, rear
 * axle along x, bike forward = +y, FLOOR AT z = -51.2 (the rear wheel's
 * radius). Rear contact (0, 0, FLOOR); front contact (0, 200, FLOOR).
 * Millimetres. Nothing here may intersect the bike parts in that tab.
 *
 * THE CHAIN: 7" base plate + ballast (round, so anything that yaws clears
 * it at any angle) -> YAW (low bearing, vertical axis through (0, L)) ->
 * YAW BAR (2" x 12" x 1/8" flat bar, laid flat; its slot sets L by where it
 * is clamped -- THE variable part, swap it per test) -> drop plate -> ROLL
 * (DEAD shaft, on the floor-level line through both contacts; bearings in
 * the roll housing) -> PITCH (DEAD shafts in the roll yoke at the front-axle
 * station y = 200, pitchHeight above the floor; bearings in the gantry
 * uprights) -> GANTRY (U-frame round the rear wheel: drivetrain, load tray).
 *
 * INSTRUMENTS: an XC330 on each of roll and pitch, body on the rolling side,
 * driving its dead shaft through a SCREWDRIVER ADAPTER: a disc on long pins
 * into the horn, with a parallel-sided blade into a slot across the shaft
 * end. Nothing else holds the adapter: the horn and the shaft end sandwich
 * it, the cradle's spring pushing the servo toward the shaft. The blade runs
 * vertically, so the servo drops into its open-topped cradle from above;
 * the cradle walls stop it turning about the shaft and leave it a little
 * side float. Lift the servo out to free the axis.
 *
 * COLOURS = what moves together:
 *   grey    fixed: base plate, ballast, yaw bearing housing
 *   blue    yaws: shaft, clamp, the flat bar, drop plate, dead roll shaft
 *   purple  rolls, does not pitch: roll housing, roll yoke, dead pitch
 *           shafts, roll servo cradle
 *   orange  rolls AND pitches -- the gantry: rails, pitch uprights,
 *           drivetrain clamps, load uprights, pitch servo cradle
 *   dark    the two XC330s and their horns
 *   black   screwdriver adapters, lock pins, the clamp bolt
 *   yellow  load tray and weights
 *   red     TM151 (heading, roll, pitch)
 */

const FLOOR = -51.2;

annotation { "Feature Type Name" : "Floor arm rig" }
export const floorArmRig = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Yaw axis distance from rear contact" }
        isLength(definition.pivotDistance, { (millimeter) : [460, 500, 610] } as LengthBoundSpec);
        annotation { "Name" : "Roll axis height above floor" }
        isLength(definition.rollHeight, { (millimeter) : [25, 25, 40] } as LengthBoundSpec);
        annotation { "Name" : "Pitch axis height above floor" }
        isLength(definition.pitchHeight, { (millimeter) : [100, 124, 150] } as LengthBoundSpec);
    }
    {
        // ==== BODY: everything below runs verbatim in the clip check ====
        const L = definition.pivotDistance / millimeter;
        const F = FLOOR;
        const zr = F + definition.rollHeight / millimeter;    // roll axis
        const zp = F + definition.pitchHeight / millimeter;   // pitch axis
        const yp = 200;                                       // front-axle station
        const zb = F + 80;                                    // underside of the yaw bar
        const T = 3.175;                                      // 1/8"
        const W = 25.4;                                       // half of 2"

        const GREY = [0.62, 0.62, 0.62];
        const BLUE = [0.25, 0.45, 0.85];
        const PURPLE = [0.6, 0.35, 0.8];
        const ORANGE = [0.95, 0.55, 0.15];
        const DARK = [0.2, 0.2, 0.2];
        const BLACK = [0.05, 0.05, 0.05];
        const YELLOW = [0.9, 0.8, 0.3];
        const RED = [0.85, 0.15, 0.15];

        // [name, corner1, corner2, colour]
        const blocks = [
            // yaws
            ["yaw_clamp_block", [-30, L - 30, F + 70], [30, L + 30, zb], BLUE],
            ["yaw_bar_2x12x0.125", [-W, 340, zb], [W, 340 + 304.8, zb + T], BLUE],
            ["drop_plate", [-10, 350, zr - 14], [10, 362, zb], BLUE],
            // rolls, does not pitch
            ["roll_housing", [-25, 300, zr - 14], [25, 340, zr + 14], PURPLE],
            ["roll_yoke_cross_left", [-65, 310, zr - 6], [-25, 330, zr + 6], PURPLE],
            ["roll_yoke_cross_right", [25, 310, zr - 6], [65, 330, zr + 6], PURPLE],
            ["roll_yoke_plate_left", [-65, yp - 10, zr - 6], [-55, 330, zp + 10], PURPLE],
            ["roll_yoke_plate_right", [55, yp - 10, zr - 6], [65, 330, zp + 10], PURPLE],
            // roll servo cradle: open top, spring at the back
            ["roll_cradle_floor", [-13.5, 254, zr - 17], [13.5, 300, zr - 13], PURPLE],
            ["roll_cradle_wall_left", [-13.5, 254, zr - 13], [-10.5, 289, zr + 21], PURPLE],
            ["roll_cradle_wall_right", [10.5, 254, zr - 13], [13.5, 289, zr + 21], PURPLE],
            ["roll_cradle_back", [-10.5, 254, zr - 13], [10.5, 258, zr + 21], PURPLE],
            ["roll_xc330", [-10, 263, zr - 13], [10, 289, zr + 21], DARK],
            // roll screwdriver adapter blade (vertical), into the shaft-end slot
            ["roll_adapter_blade", [-1, 294, zr - 6], [1, 300.4, zr + 6], BLACK],
            // the gantry: rolls and pitches
            ["gantry_rail_left", [-50, -62, zr - 6], [-44, yp + 10, zr + 6], ORANGE],
            ["gantry_rail_right", [44, -62, zr - 6], [50, yp + 10, zr + 6], ORANGE],
            ["gantry_rear", [-44, -62, zr - 6], [44, -50, zr + 6], ORANGE],
            ["pitch_upright_left", [-50, yp - 10, zr + 6], [-44, yp + 10, zp + 10], ORANGE],
            ["pitch_upright_right", [44, yp - 10, zr + 6], [50, yp + 10, zp + 10], ORANGE],
            // pitch servo cradle: open top, spring at the inboard end. Floor and
            // rear wall stop at x = -6: the fork (x +-5) passes under the servo.
            ["pitch_cradle_bracket", [-44, yp - 13.5, zp - 18], [-24, yp + 13.5, zp - 14], ORANGE],
            ["pitch_cradle_floor", [-24, yp - 13.5, zp - 18], [-6, yp + 13.5, zp - 14], ORANGE],
            ["pitch_cradle_wall_front", [-24, yp + 10.5, zp - 14], [10, yp + 13.5, zp + 20], ORANGE],
            ["pitch_cradle_wall_rear", [-24, yp - 13.5, zp - 14], [-6, yp - 10.5, zp + 20], ORANGE],
            ["pitch_cradle_back", [6, yp - 10.5, zp - 14], [10, yp + 10.5, zp + 20], ORANGE],
            ["pitch_xc330", [-22, yp - 10, zp - 14], [1, yp + 10, zp + 20], DARK],
            // pitch screwdriver adapter blade (vertical)
            ["pitch_adapter_blade", [-33.4, yp - 1, zp - 5], [-27, yp + 1, zp + 5], BLACK],
            // drivetrain clamps, from the drive cases' outer faces (x = +-40.2)
            ["drive_clamp_left", [-44, 10, zr + 6], [-40.2, 50, 10], ORANGE],
            ["drive_clamp_right", [40.2, 10, zr + 6], [44, 50, 10], ORANGE],
            // load path: uprights from the rails, tray over the rear axle (y = 0)
            ["load_upright_left", [-50, -6, zr + 6], [-44, 6, 134], ORANGE],
            ["load_upright_right", [44, -6, zr + 6], [50, 6, 134], ORANGE],
            ["load_tray", [-50, -25, 134], [50, 25, 140], YELLOW],
            ["load_weights", [-40, -20, 140], [40, 20, 170], YELLOW],
            ["tm151", [50, -15, 60], [61, 15, 86], RED]
        ];
        // [name, bottom centre, top centre, radius, colour]
        const cylinders = [
            // fixed: round, so nothing that yaws can meet it at any angle
            ["base_plate_7in", [0, L, F], [0, L, F + 6.35], 88.9, GREY],
            ["ballast_nonmagnetic", [0, L, F + 6.35], [0, L, F + 40], 88.9, GREY],
            ["yaw_housing", [0, L, F + 40], [0, L, F + 68], 25, GREY],
            // yaws
            ["yaw_shaft", [0, L, F + 40], [0, L, F + 70], 6, BLUE],
            ["yaw_clamp_bolt", [0, L, F + 70], [0, L, zb + T + 6], 3, BLACK],
            ["yaw_clamp_washer", [0, L, zb + T], [0, L, zb + T + 2], 9, BLACK],
            ["yaw_lock_pin", [28, L, F + 34], [28, L, F + 70], 2, BLACK],
            ["roll_shaft_dead", [0, 297, zr], [0, 356, zr], 6, BLUE],
            ["roll_lock_pin", [0, 310, zr + 10], [0, 360, zr + 10], 2, BLACK],
            // roll servo: horn, adapter disc, pins into the horn
            ["roll_xc330_horn", [0, 289, zr], [0, 291, zr], 10, DARK],
            ["roll_adapter_disc", [0, 291, zr], [0, 294, zr], 10, BLACK],
            ["roll_adapter_pin_a", [7, 289, zr], [7, 294, zr], 1, BLACK],
            ["roll_adapter_pin_b", [-7, 289, zr], [-7, 294, zr], 1, BLACK],
            ["roll_cradle_spring", [0, 258, zr + 4], [0, 263, zr + 4], 3, PURPLE],
            // pitch
            ["pitch_shaft_dead_left", [-65, yp, zp], [-30, yp, zp], 5, PURPLE],
            ["pitch_shaft_dead_right", [44, yp, zp], [65, yp, zp], 5, PURPLE],
            ["pitch_lock_pin", [44, yp, zp - 20], [65, yp, zp - 20], 2, BLACK],
            ["pitch_adapter_disc", [-27, yp, zp], [-24, yp, zp], 10, BLACK],
            ["pitch_xc330_horn", [-24, yp, zp], [-22, yp, zp], 10, DARK],
            ["pitch_adapter_pin_a", [-27, yp + 7, zp], [-22, yp + 7, zp], 1, BLACK],
            ["pitch_adapter_pin_b", [-27, yp - 7, zp], [-22, yp - 7, zp], 1, BLACK],
            ["pitch_cradle_spring", [1, yp, zp + 3], [6, yp, zp + 3], 3, ORANGE]
        ];
        // [cutter name, corner1, corner2, the body it is cut from]
        const cuts = [
            ["cut_yaw_bar_slot", [-3.2, 380, zb - 1], [3.2, 620, zb + T + 1], "yaw_bar_2x12x0.125"],
            ["cut_roll_shaft_slot", [-1.1, 296, zr - 7], [1.1, 300.5, zr + 7], "roll_shaft_dead"],
            ["cut_pitch_shaft_slot", [-33.5, yp - 1.1, zp - 6], [-29, yp + 1.1, zp + 6], "pitch_shaft_dead_left"]
        ];

        for (var b in blocks)
        {
            fCuboid(context, id + b[0], { "corner1" : vector(b[1]) * millimeter,
                                          "corner2" : vector(b[2]) * millimeter });
        }
        for (var c in cylinders)
        {
            fCylinder(context, id + c[0], { "bottomCenter" : vector(c[1]) * millimeter,
                                            "topCenter" : vector(c[2]) * millimeter,
                                            "radius" : c[3] * millimeter });
        }
        for (var e in concatenateArrays([blocks, cylinders]))
        {
            const q = qCreatedBy(id + e[0], EntityType.BODY);
            const rgb = e[size(e) - 1];
            setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : e[0] });
            setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE,
                        "value" : color(rgb[0], rgb[1], rgb[2]) });
        }
        // Slots: cut by SUBTRACTION with the cutter as the tool, so the target
        // keeps its identity and its name.
        for (var k in cuts)
        {
            fCuboid(context, id + k[0], { "corner1" : vector(k[1]) * millimeter,
                                          "corner2" : vector(k[2]) * millimeter });
            opBoolean(context, id + (k[0] ~ "_op"), {
                        "tools" : qCreatedBy(id + k[0], EntityType.BODY),
                        "targets" : qCreatedBy(id + k[3], EntityType.BODY),
                        "operationType" : BooleanOperationType.SUBTRACTION });
        }
        // ==== END BODY ====
    });
