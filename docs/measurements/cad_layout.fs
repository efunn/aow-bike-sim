FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");
import(path : "onshape/std/variable.fs", version : "3044.0");

// AOW bike component layout — GENERATED, do not edit.
//   python -m aow_sim.cad_layout --format featurescript
//
// FRAME:  +X right, +Y forward, +Z up.  ORIGIN: the rear axle,
// 51.2 mm above the floor when the bike is upright.
//
// Exported from the simulator (config/bike_params.yaml). Most entries are
// `design` or `GUESS` — see docs/measurements/cad_layout.yaml for the
// provenance of every number, which is deliberately NOT duplicated here.
//
// !! The version number on the two lines above must match your document. The
// !! easiest fix is to create the Feature Studio first, then replace only the
// !! body below its auto-inserted header.


export const AOW_LAYOUT = {
    "omni_wheel_rear" : {
        "group" : "drivetrain",
        "pos" : vector(0.0, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 51.2 * millimeter,
        "length" : 33.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "mass_g" : 115.2
    },
    "pulley_input_left" : {
        "group" : "drivetrain",
        "pos" : vector(-24.0, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 13.94 * millimeter,
        "length" : 11.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0)
    },
    "axle_mount_left" : {
        "group" : "drivetrain",
        "pos" : vector(-33.5, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 15.94 * millimeter,
        "length" : 8.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0)
    },
    "pulley_input_right" : {
        "group" : "drivetrain",
        "pos" : vector(24.0, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 13.94 * millimeter,
        "length" : 11.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0)
    },
    "axle_mount_right" : {
        "group" : "drivetrain",
        "pos" : vector(33.5, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 15.94 * millimeter,
        "length" : 8.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0)
    },
    "pulley_servo_left" : {
        "group" : "drivetrain",
        "pos" : vector(-24.0, 85.93, 64.34) * millimeter,
        "shape" : "cylinder",
        "radius" : 37.81 * millimeter,
        "length" : 11.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0)
    },
    "pulley_servo_right" : {
        "group" : "drivetrain",
        "pos" : vector(24.0, 64.34, 85.93) * millimeter,
        "shape" : "cylinder",
        "radius" : 37.81 * millimeter,
        "length" : 11.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0)
    },
    "front_wheel" : {
        "group" : "steering",
        "pos" : vector(0.0, 200.0, -1.2) * millimeter,
        "shape" : "cylinder",
        "radius" : 50.0 * millimeter,
        "length" : 28.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "mass_g" : 60.0
    },
    "fork" : {
        "group" : "steering",
        "pos" : vector(0.0, 188.35, 42.27) * millimeter,
        "shape" : "capsule",
        "radius" : 5.0 * millimeter,
        "length" : 90.0 * millimeter,
        "axis" : vector(0.0, -0.258819, 0.965926),
        "mass_g" : 25.0
    },
    "servo_drive_left" : {
        "group" : "servos",
        "pos" : vector(-1.5, 77.44, 55.85) * millimeter,
        "shape" : "box",
        "size" : vector(28.5, 34.0, 46.5) * millimeter,
        "rotAxis" : vector(-0.357407, 0.357407, 0.862856),
        "rotDeg" : 98.4211 * degree,
        "mass_g" : 65.0
    },
    "servo_drive_left_shaft" : {
        "group" : "servos",
        "pos" : vector(-18.5, 85.93, 64.34) * millimeter,
        "shape" : "point"
    },
    "servo_drive_right" : {
        "group" : "servos",
        "pos" : vector(1.5, 55.85, 77.44) * millimeter,
        "shape" : "box",
        "size" : vector(28.5, 34.0, 46.5) * millimeter,
        "rotAxis" : vector(-0.357407, -0.357407, -0.862856),
        "rotDeg" : 98.4211 * degree,
        "mass_g" : 65.0
    },
    "servo_drive_right_shaft" : {
        "group" : "servos",
        "pos" : vector(18.5, 64.34, 85.93) * millimeter,
        "shape" : "point"
    },
    "servo_steer" : {
        "group" : "servos",
        "pos" : vector(0.0, 166.1, 96.35) * millimeter,
        "shape" : "box",
        "size" : vector(20.0, 26.0, 34.0) * millimeter,
        "rotAxis" : vector(-1.0, 0.0, 0.0),
        "rotDeg" : 75.0 * degree,
        "mass_g" : 23.0
    },
    "servo_steer_shaft" : {
        "group" : "servos",
        "pos" : vector(0.0, 176.71, 85.73) * millimeter,
        "shape" : "point"
    },
    "ahrs_tm151" : {
        "group" : "electronics",
        "pos" : vector(0.0, 50.0, 130.0) * millimeter,
        "shape" : "box",
        "size" : vector(34.0, 40.0, 12.6) * millimeter,
        "mass_g" : 19.0
    },
    "payload_battery" : {
        "group" : "electronics",
        "pos" : vector(0.0, 25.9, 100.2) * millimeter,
        "shape" : "box",
        "size" : vector(26.0, 35.0, 72.0) * millimeter,
        "rotAxis" : vector(-0.357407, 0.357407, 0.862856),
        "rotDeg" : 98.4211 * degree,
        "mass_g" : 115.0
    },
    "payload_pi" : {
        "group" : "electronics",
        "pos" : vector(0.0, 90.0, 108.0) * millimeter,
        "shape" : "box",
        "size" : vector(30.0, 65.0, 5.0) * millimeter,
        "mass_g" : 11.0
    },
    "payload_u2d2" : {
        "group" : "electronics",
        "pos" : vector(0.0, 90.0, 120.0) * millimeter,
        "shape" : "box",
        "size" : vector(18.0, 48.0, 14.9) * millimeter,
        "mass_g" : 9.0
    },
    "payload_power_board" : {
        "group" : "electronics",
        "pos" : vector(0.0, 90.0, 93.0) * millimeter,
        "shape" : "box",
        "size" : vector(30.0, 30.0, 25.0) * millimeter,
        "mass_g" : 56.0
    },
    "linkage_crank_servo" : {
        "group" : "righting",
        "pos" : vector(0.0, 117.0, 75.96) * millimeter,
        "shape" : "box",
        "size" : vector(20.0, 26.0, 34.0) * millimeter,
        "mass_g" : 23.0
    },
    "linkage_crank_shaft" : {
        "group" : "righting",
        "pos" : vector(0.0, 130.0, 83.46) * millimeter,
        "shape" : "point"
    },
    "crank_tip_right" : {
        "group" : "righting",
        "pos" : vector(-14.48, 130.0, 110.39) * millimeter,
        "shape" : "point"
    },
    "wing_attach_right" : {
        "group" : "righting",
        "pos" : vector(29.21, 130.0, 42.66) * millimeter,
        "shape" : "point"
    },
    "crank_tip_left" : {
        "group" : "righting",
        "pos" : vector(18.51, 130.0, 108.34) * millimeter,
        "shape" : "point"
    },
    "wing_attach_left" : {
        "group" : "righting",
        "pos" : vector(-29.21, 130.0, 42.66) * millimeter,
        "shape" : "point"
    },
    "crank_sweep_right" : {
        "group" : "righting",
        "pos" : vector(26.41, 130.0, 98.87) * millimeter,
        "shape" : "point"
    },
    "crank_sweep_left" : {
        "group" : "righting",
        "pos" : vector(25.51, 130.0, 65.83) * millimeter,
        "shape" : "point"
    },
    "link_crank_right" : {
        "group" : "righting",
        "pos" : vector(-7.24, 130.0, 96.92) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 30.58 * millimeter,
        "axis" : vector(-0.473492, 0.0, 0.880798)
    },
    "link_coupler_right" : {
        "group" : "righting",
        "pos" : vector(7.36, 130.0, 76.52) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 80.59 * millimeter,
        "axis" : vector(0.542037, 0.0, -0.840355)
    },
    "link_rocker_right" : {
        "group" : "righting",
        "pos" : vector(23.25, 130.0, 21.33) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 44.3 * millimeter,
        "axis" : vector(0.269058, 0.0, 0.963124)
    },
    "link_crank_left" : {
        "group" : "righting",
        "pos" : vector(9.25, 130.0, 95.9) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 31.01 * millimeter,
        "axis" : vector(0.596784, 0.0, 0.802402)
    },
    "link_coupler_left" : {
        "group" : "righting",
        "pos" : vector(-5.35, 130.0, 75.5) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 81.18 * millimeter,
        "axis" : vector(-0.587747, 0.0, -0.809045)
    },
    "link_rocker_left" : {
        "group" : "righting",
        "pos" : vector(-23.25, 130.0, 21.33) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 44.3 * millimeter,
        "axis" : vector(-0.269058, 0.0, 0.963124)
    },
    "wing_left_pivot" : {
        "group" : "righting",
        "pos" : vector(-17.29, 130.0, 0.0) * millimeter,
        "shape" : "point"
    },
    "wing_left_panel" : {
        "group" : "righting",
        "pos" : vector(-35.5, 105.0, 73.94) * millimeter,
        "shape" : "box",
        "size" : vector(4.0, 90.0, 107.12) * millimeter,
        "mass_g" : 20.0
    },
    "wing_right_pivot" : {
        "group" : "righting",
        "pos" : vector(17.29, 130.0, 0.0) * millimeter,
        "shape" : "point"
    },
    "wing_right_panel" : {
        "group" : "righting",
        "pos" : vector(35.5, 105.0, 73.94) * millimeter,
        "shape" : "box",
        "size" : vector(4.0, 90.0, 107.12) * millimeter,
        "mass_g" : 20.0
    },
    "roof" : {
        "group" : "righting",
        "pos" : vector(0.0, 92.5, 127.5) * millimeter,
        "shape" : "capsule",
        "radius" : 37.5 * millimeter,
        "length" : 145.0 * millimeter,
        "axis" : vector(0.0, 1.0, 0.0),
        "mass_g" : 45.0
    }
};

export const AOW_FOURBAR_STATION = 130.0 * millimeter;
export const AOW_FOURBAR = [
    { "name" : "link_crank_right", "start" : vector(0.0, 83.46) * millimeter, "end" : vector(-14.48, 110.39) * millimeter },
    { "name" : "link_coupler_right", "start" : vector(-14.48, 110.39) * millimeter, "end" : vector(29.21, 42.66) * millimeter },
    { "name" : "link_rocker_right", "start" : vector(17.29, 0.0) * millimeter, "end" : vector(29.21, 42.66) * millimeter },
    { "name" : "link_crank_left", "start" : vector(0.0, 83.46) * millimeter, "end" : vector(18.51, 108.34) * millimeter },
    { "name" : "link_coupler_left", "start" : vector(18.51, 108.34) * millimeter, "end" : vector(-29.21, 42.66) * millimeter },
    { "name" : "link_rocker_left", "start" : vector(-17.29, 0.0) * millimeter, "end" : vector(-29.21, 42.66) * millimeter }
];

export const AOW_FOURBAR_ARCS = [
    { "name" : "crank_sweep_right", "start" : vector(-14.48, 110.39) * millimeter, "mid" : vector(26.41, 98.87) * millimeter, "end" : vector(16.32, 57.6) * millimeter },
    { "name" : "crank_sweep_left", "start" : vector(18.51, 108.34) * millimeter, "mid" : vector(25.51, 65.83) * millimeter, "end" : vector(-16.73, 57.34) * millimeter }
];

annotation { "Feature Type Name" : "AOW bike layout" }
export const aowBikeLayout = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Publish variables" }
        definition.publishVariables is boolean;

        annotation { "Name" : "Draw envelopes" }
        definition.drawEnvelopes is boolean;

        annotation { "Name" : "Draw origin points" }
        definition.drawPoints is boolean;

        annotation { "Name" : "Draw four-bar sketch" }
        definition.drawFourBarSketch is boolean;

        annotation { "Name" : "Draw axis planes" }
        definition.drawAxisPlanes is boolean;

        annotation { "Name" : "Plane size" }
        isLength(definition.planeSize, LENGTH_BOUNDS);
    }
    {
        // Each block is independently switchable, and they run cheapest and
        // safest first. A runtime error anywhere aborts the WHOLE feature, so
        // if one of these misbehaves you can still get the others by turning
        // it off — which is how the missing-shape bug cost us the variables.

        if (definition.publishVariables)
        {
            for (var name in keys(AOW_LAYOUT))
            {
                var c = AOW_LAYOUT[name];
                setVariable(context, "aow_" ~ name ~ "_x", c.pos[0]);
                setVariable(context, "aow_" ~ name ~ "_y", c.pos[1]);
                setVariable(context, "aow_" ~ name ~ "_z", c.pos[2]);
            }
        }

        if (definition.drawEnvelopes)
        {
            for (var name in keys(AOW_LAYOUT))
            {
                var c = AOW_LAYOUT[name];
                var subId = id + ("solid_" ~ name);

                // Dispatch on KNOWN shapes only. An unrecognised or missing
                // shape must draw nothing rather than fall through to code
                // that dereferences keys the entry does not have.
                if (c.shape == "box")
                {
                    fCuboid(context, subId, {
                            "corner1" : c.pos - c.size / 2,
                            "corner2" : c.pos + c.size / 2
                    });

                    // fCuboid is axis-aligned only, so an oriented box is
                    // built square and then rotated about its own centre.
                    // The axis and angle are precomputed by the generator.
                    if (c.rotAxis != undefined)
                    {
                        opTransform(context, id + ("rot_" ~ name), {
                                "bodies" : qCreatedBy(subId, EntityType.BODY),
                                "transform" : rotationAround(
                                        line(c.pos, c.rotAxis), c.rotDeg)
                        });
                    }
                }
                else if (c.shape == "cylinder" || c.shape == "capsule")
                {
                    // fCylinder, not opCylinder — solid primitives are the f*
                    // family. Capsules are drawn as plain cylinders: the end
                    // caps matter to the contact model, not to clearance.
                    var half = c.axis * c.length / 2;
                    fCylinder(context, subId, {
                            "topCenter" : c.pos + half,
                            "bottomCenter" : c.pos - half,
                            "radius" : c.radius
                    });
                }
                else
                {
                    continue;   // "point" — nothing solid to draw
                }

                // Without this every body lands in the list as "Part N".
                setProperty(context, {
                        "entities" : qCreatedBy(subId, EntityType.BODY),
                        "propertyType" : PropertyType.NAME,
                        "value" : name
                });
            }
        }

        if (definition.drawPoints)
        {
            for (var name in keys(AOW_LAYOUT))
            {
                var subId = id + ("point_" ~ name);
                opPoint(context, subId, { "point" : AOW_LAYOUT[name].pos });
                setProperty(context, {
                        "entities" : qCreatedBy(subId, EntityType.BODY),
                        "propertyType" : PropertyType.NAME,
                        "value" : name ~ "_origin"
                });
            }
        }

        if (definition.drawFourBarSketch && size(AOW_FOURBAR) > 0)
        {
            // A real sketch of construction lines. Cheap because the mechanism
            // is planar: one plane at the linkage's fore/aft station, normal
            // CAD +Y, and the 2D coordinates are just (CAD x, CAD z).
            // Normal is CAD -Y, not +Y. A sketch plane's local Y is
            // (normal CROSS xDir): with normal +Y that gives -Z and the whole
            // mechanism draws upside down. With -Y it gives +Z, so the emitted
            // (CAD x, CAD z) pairs mean what they say.
            var sk = newSketchOnPlane(context, id + "fourbar", {
                    "sketchPlane" : plane(
                            vector(0, 1, 0) * AOW_FOURBAR_STATION,
                            vector(0, -1, 0),
                            vector(1, 0, 0))
            });
            for (var seg in AOW_FOURBAR)
            {
                skLineSegment(sk, seg.name, {
                        "start" : seg.start,
                        "end" : seg.end,
                        "construction" : true
                });
            }
            for (var a in AOW_FOURBAR_ARCS)
            {
                // Three-point arc: the crank tip's swept path. Construction,
                // because it is a keep-out boundary rather than a part.
                skArc(sk, a.name, {
                        "start" : a.start,
                        "mid" : a.mid,
                        "end" : a.end,
                        "construction" : true
                });
            }
            skSolve(sk);
        }

        if (definition.drawAxisPlanes)
        {
            for (var name in keys(AOW_LAYOUT))
            {
                var c = AOW_LAYOUT[name];
                if (c.shape != "cylinder" && c.shape != "capsule")
                    continue;

                // Normal to the component axis, through its centre. For the
                // fork this is the plane perpendicular to the STEERING AXIS,
                // which is the one you want for the head tube and any clamp —
                // sketching those against a world plane is what puts the rake
                // in wrong.
                var subId = id + ("plane_" ~ name);
                opPlane(context, subId, {
                        "plane" : plane(c.pos, c.axis),
                        "width" : definition.planeSize,
                        "height" : definition.planeSize
                });
                // NOT NAMED, and it cannot be. Planes and mate connectors
                // carry no metadata — the UI derives their names from the
                // FEATURE that made them, and only a `cPlane`-named feature
                // can name a plane. So every plane a custom feature emits is
                // named after that feature, whatever setProperty does. Both
                // the filtered query (silently names nothing) and the
                // unfiltered one (throws, taking the plane with it) were
                // dead ends for the same underlying reason. If per-plane
                // names matter, that is an argument for separate feature
                // instances, not for more code here.
            }
        }
    });
