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
//
// This studio defines SEVERAL features; they all show up under Custom features
// once it is committed. Insert `AOW layout variables` once and first, then one
// `AOW <group>` per group you are working on, and RENAME each node — that name
// is the only handle Onshape will give you on the planes it draws.


export const AOW_LAYOUT = {
    "omni_wheel_rear" : {
        "group" : "drivetrain",
        "pos" : vector(0.0, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 51.2 * millimeter,
        "length" : 33.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.36, 0.56, 0.76, 0.85],
        "mass_g" : 115.2
    },
    "pulley_input_left" : {
        "group" : "drivetrain",
        "pos" : vector(-26.5, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 13.94 * millimeter,
        "length" : 11.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.36, 0.56, 0.76, 0.85]
    },
    "axle_mount_left" : {
        "group" : "drivetrain",
        "pos" : vector(-36.0, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 15.94 * millimeter,
        "length" : 8.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.36, 0.56, 0.76, 0.85]
    },
    "pulley_input_right" : {
        "group" : "drivetrain",
        "pos" : vector(26.5, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 13.94 * millimeter,
        "length" : 11.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.36, 0.56, 0.76, 0.85]
    },
    "axle_mount_right" : {
        "group" : "drivetrain",
        "pos" : vector(36.0, 0.0, 0.0) * millimeter,
        "shape" : "cylinder",
        "radius" : 15.94 * millimeter,
        "length" : 8.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.36, 0.56, 0.76, 0.85]
    },
    "pulley_servo_left" : {
        "group" : "drivetrain",
        "pos" : vector(-26.5, 85.31, 65.16) * millimeter,
        "shape" : "cylinder",
        "radius" : 37.81 * millimeter,
        "length" : 11.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.36, 0.56, 0.76, 0.85]
    },
    "pulley_servo_right" : {
        "group" : "drivetrain",
        "pos" : vector(26.5, 65.16, 85.31) * millimeter,
        "shape" : "cylinder",
        "radius" : 37.81 * millimeter,
        "length" : 11.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.36, 0.56, 0.76, 0.85]
    },
    "front_wheel" : {
        "group" : "steering",
        "pos" : vector(0.0, 200.0, -1.2) * millimeter,
        "shape" : "cylinder",
        "radius" : 50.0 * millimeter,
        "length" : 28.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.24, 0.68, 0.58, 0.85],
        "mass_g" : 60.0
    },
    "fork" : {
        "group" : "steering",
        "pos" : vector(0.0, 188.35, 42.27) * millimeter,
        "shape" : "capsule",
        "radius" : 5.0 * millimeter,
        "length" : 90.0 * millimeter,
        "axis" : vector(0.0, -0.258819, 0.965926),
        "rgba" : [0.24, 0.68, 0.58, 0.85],
        "mass_g" : 25.0
    },
    "servo_drive_left" : {
        "group" : "servos",
        "pos" : vector(0.0, 76.82, 56.67) * millimeter,
        "shape" : "box",
        "size" : vector(28.5, 34.0, 46.5) * millimeter,
        "rotAxis" : vector(-0.357407, 0.357407, 0.862856),
        "rotDeg" : 98.4211 * degree,
        "rgba" : [0.9, 0.62, 0.18, 0.85],
        "mass_g" : 65.0
    },
    "servo_drive_left_case_holes_horn" : {
        "group" : "servos",
        "pos" : vector(-17.0, 76.82, 56.67) * millimeter,
        "shape" : "holes",
        "points" : [vector(-17.0, 98.74, 63.04) * millimeter, vector(-17.0, 70.46, 34.75) * millimeter, vector(-17.0, 83.19, 78.59) * millimeter, vector(-17.0, 54.9, 50.31) * millimeter]
    },
    "servo_drive_left_case_holes_back" : {
        "group" : "servos",
        "pos" : vector(17.0, 76.82, 56.67) * millimeter,
        "shape" : "holes",
        "points" : [vector(17.0, 98.74, 63.04) * millimeter, vector(17.0, 70.46, 34.75) * millimeter, vector(17.0, 83.19, 78.59) * millimeter, vector(17.0, 54.9, 50.31) * millimeter]
    },
    "servo_drive_left_horn" : {
        "group" : "servos",
        "pos" : vector(-18.0, 85.31, 65.16) * millimeter,
        "shape" : "cylinder",
        "radius" : 10.25 * millimeter,
        "length" : 2.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.9, 0.62, 0.18, 0.85]
    },
    "servo_drive_left_horn_boss" : {
        "group" : "servos",
        "pos" : vector(-19.95, 85.31, 65.16) * millimeter,
        "shape" : "cylinder",
        "radius" : 3.95 * millimeter,
        "length" : 1.9 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.9, 0.62, 0.18, 0.85]
    },
    "servo_drive_left_shaft" : {
        "group" : "servos",
        "pos" : vector(-19.0, 85.31, 65.16) * millimeter,
        "shape" : "point"
    },
    "servo_drive_right" : {
        "group" : "servos",
        "pos" : vector(0.0, 56.67, 76.82) * millimeter,
        "shape" : "box",
        "size" : vector(28.5, 34.0, 46.5) * millimeter,
        "rotAxis" : vector(-0.357407, -0.357407, -0.862856),
        "rotDeg" : 98.4211 * degree,
        "rgba" : [0.9, 0.62, 0.18, 0.85],
        "mass_g" : 65.0
    },
    "servo_drive_right_case_holes_horn" : {
        "group" : "servos",
        "pos" : vector(17.0, 56.67, 76.82) * millimeter,
        "shape" : "holes",
        "points" : [vector(17.0, 63.04, 98.74) * millimeter, vector(17.0, 34.75, 70.46) * millimeter, vector(17.0, 78.59, 83.19) * millimeter, vector(17.0, 50.31, 54.9) * millimeter]
    },
    "servo_drive_right_case_holes_back" : {
        "group" : "servos",
        "pos" : vector(-17.0, 56.67, 76.82) * millimeter,
        "shape" : "holes",
        "points" : [vector(-17.0, 63.04, 98.74) * millimeter, vector(-17.0, 34.75, 70.46) * millimeter, vector(-17.0, 78.59, 83.19) * millimeter, vector(-17.0, 50.31, 54.9) * millimeter]
    },
    "servo_drive_right_horn" : {
        "group" : "servos",
        "pos" : vector(18.0, 65.16, 85.31) * millimeter,
        "shape" : "cylinder",
        "radius" : 10.25 * millimeter,
        "length" : 2.0 * millimeter,
        "axis" : vector(1.0, 0.0, 0.0),
        "rgba" : [0.9, 0.62, 0.18, 0.85]
    },
    "servo_drive_right_horn_boss" : {
        "group" : "servos",
        "pos" : vector(19.95, 65.16, 85.31) * millimeter,
        "shape" : "cylinder",
        "radius" : 3.95 * millimeter,
        "length" : 1.9 * millimeter,
        "axis" : vector(1.0, 0.0, 0.0),
        "rgba" : [0.9, 0.62, 0.18, 0.85]
    },
    "servo_drive_right_shaft" : {
        "group" : "servos",
        "pos" : vector(19.0, 65.16, 85.31) * millimeter,
        "shape" : "point"
    },
    "drive_mount_plate_right" : {
        "group" : "mount",
        "pos" : vector(18.5, 67.42, 67.42) * millimeter,
        "shape" : "box",
        "size" : vector(3.0, 63.5, 53.0) * millimeter,
        "rotAxis" : vector(-1.0, 0.0, 0.0),
        "rotDeg" : 45.0 * degree,
        "rgba" : [0.62, 0.62, 0.66, 0.85]
    },
    "drive_mount_relief_right" : {
        "group" : "mount",
        "pos" : vector(18.5, 65.16, 85.31) * millimeter,
        "shape" : "cylinder",
        "radius" : 11.75 * millimeter,
        "length" : 3.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.62, 0.62, 0.66, 0.85]
    },
    "drive_mount_plate_left" : {
        "group" : "mount",
        "pos" : vector(-18.5, 67.42, 67.42) * millimeter,
        "shape" : "box",
        "size" : vector(3.0, 63.5, 53.0) * millimeter,
        "rotAxis" : vector(-1.0, 0.0, 0.0),
        "rotDeg" : 45.0 * degree,
        "rgba" : [0.62, 0.62, 0.66, 0.85]
    },
    "drive_mount_relief_left" : {
        "group" : "mount",
        "pos" : vector(-18.5, 85.31, 65.16) * millimeter,
        "shape" : "cylinder",
        "radius" : 11.75 * millimeter,
        "length" : 3.0 * millimeter,
        "axis" : vector(-1.0, 0.0, 0.0),
        "rgba" : [0.62, 0.62, 0.66, 0.85]
    },
    "drive_mount_side_a" : {
        "group" : "mount",
        "pos" : vector(0.0, 46.03, 88.81) * millimeter,
        "shape" : "box",
        "size" : vector(34.0, 3.0, 53.0) * millimeter,
        "rotAxis" : vector(-1.0, 0.0, 0.0),
        "rotDeg" : 45.0 * degree,
        "rgba" : [0.62, 0.62, 0.66, 0.85]
    },
    "drive_mount_side_b" : {
        "group" : "mount",
        "pos" : vector(0.0, 88.81, 46.03) * millimeter,
        "shape" : "box",
        "size" : vector(34.0, 3.0, 53.0) * millimeter,
        "rotAxis" : vector(-1.0, 0.0, 0.0),
        "rotDeg" : 45.0 * degree,
        "rgba" : [0.62, 0.62, 0.66, 0.85]
    },
    "drive_mount_radial_in" : {
        "group" : "mount",
        "pos" : vector(0.0, 49.74, 49.74) * millimeter,
        "shape" : "box",
        "size" : vector(34.0, 63.5, 3.0) * millimeter,
        "rotAxis" : vector(-1.0, 0.0, 0.0),
        "rotDeg" : 45.0 * degree,
        "rgba" : [0.62, 0.62, 0.66, 0.85]
    },
    "drive_mount_radial_out" : {
        "group" : "mount",
        "pos" : vector(0.0, 85.1, 85.1) * millimeter,
        "shape" : "box",
        "size" : vector(34.0, 63.5, 3.0) * millimeter,
        "rotAxis" : vector(-1.0, 0.0, 0.0),
        "rotDeg" : 45.0 * degree,
        "rgba" : [0.62, 0.62, 0.66, 0.85]
    },
    "servo_steer" : {
        "group" : "servos",
        "pos" : vector(0.0, 165.71, 97.8) * millimeter,
        "shape" : "box",
        "size" : vector(20.0, 23.0, 34.0) * millimeter,
        "rotAxis" : vector(-1.0, 0.0, 0.0),
        "rotDeg" : 75.0 * degree,
        "rgba" : [0.9, 0.62, 0.18, 0.85],
        "mass_g" : 23.0
    },
    "servo_steer_case_holes_horn" : {
        "group" : "servos",
        "pos" : vector(0.0, 168.69, 86.69) * millimeter,
        "shape" : "holes",
        "points" : [vector(8.0, 183.17, 90.57) * millimeter, vector(8.0, 154.2, 82.81) * millimeter, vector(-8.0, 183.17, 90.57) * millimeter, vector(-8.0, 154.2, 82.81) * millimeter]
    },
    "servo_steer_case_holes_back" : {
        "group" : "servos",
        "pos" : vector(0.0, 162.73, 108.91) * millimeter,
        "shape" : "holes",
        "points" : [vector(8.0, 177.22, 112.79) * millimeter, vector(8.0, 148.24, 105.02) * millimeter, vector(-8.0, 177.22, 112.79) * millimeter, vector(-8.0, 148.24, 105.02) * millimeter]
    },
    "servo_steer_horn" : {
        "group" : "servos",
        "pos" : vector(0.0, 176.32, 87.18) * millimeter,
        "shape" : "cylinder",
        "radius" : 8.0 * millimeter,
        "length" : 3.0 * millimeter,
        "axis" : vector(0.0, 0.258819, -0.965926),
        "rgba" : [0.9, 0.62, 0.18, 0.85]
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
        "rgba" : [0.4, 0.74, 0.34, 0.85],
        "mass_g" : 19.0
    },
    "plane_fork_print" : {
        "group" : "planes",
        "pos" : vector(0.0, 200.0, -1.2) * millimeter,
        "shape" : "plane",
        "normal" : vector(0.0, 0.965926, 0.258819)
    },
    "plane_fork_print_offset" : {
        "group" : "planes",
        "pos" : vector(0.0, 207.73, 0.87) * millimeter,
        "shape" : "plane",
        "normal" : vector(0.0, 0.965926, 0.258819)
    },
    "plane_drive_mount_print" : {
        "group" : "planes",
        "pos" : vector(0.0, 96.06, 55.75) * millimeter,
        "shape" : "plane",
        "normal" : vector(0.0, -0.707107, 0.707107)
    },
    "plane_belt_left_lower" : {
        "group" : "planes",
        "pos" : vector(-26.5, 53.39, 9.04) * millimeter,
        "shape" : "plane",
        "normal" : vector(0.0, -0.415043, 0.909802)
    },
    "belt_left_lower" : {
        "group" : "belts",
        "pos" : vector(-26.5, 54.14, 7.4) * millimeter,
        "shape" : "box",
        "size" : vector(104.66, 9.0, 3.6) * millimeter,
        "rotAxis" : vector(-0.698903, 0.698903, 0.151887),
        "rotDeg" : 162.727 * degree,
        "rgba" : [0.13, 0.14, 0.16, 1.0]
    },
    "belt_left_lower_mirrored" : {
        "group" : "belts_mirror",
        "pos" : vector(26.5, 54.14, 7.4) * millimeter,
        "shape" : "box",
        "size" : vector(104.66, 9.0, 3.6) * millimeter,
        "rotAxis" : vector(-0.698903, 0.698903, 0.151887),
        "rotDeg" : 162.727 * degree,
        "rgba" : [0.9, 0.25, 0.6, 0.3]
    },
    "plane_belt_left_upper" : {
        "group" : "planes",
        "pos" : vector(-26.5, 22.77, 49.13) * millimeter,
        "shape" : "plane",
        "normal" : vector(0.0, -0.768526, 0.639819)
    },
    "belt_left_upper" : {
        "group" : "belts",
        "pos" : vector(-26.5, 21.39, 50.28) * millimeter,
        "shape" : "box",
        "size" : vector(104.66, 9.0, 3.6) * millimeter,
        "rotAxis" : vector(0.39065, -0.39065, 0.833538),
        "rotDeg" : 100.375 * degree,
        "rgba" : [0.13, 0.14, 0.16, 1.0]
    },
    "belt_left_upper_mirrored" : {
        "group" : "belts_mirror",
        "pos" : vector(26.5, 21.39, 50.28) * millimeter,
        "shape" : "box",
        "size" : vector(104.66, 9.0, 3.6) * millimeter,
        "rotAxis" : vector(0.39065, -0.39065, 0.833538),
        "rotDeg" : 100.375 * degree,
        "rgba" : [0.9, 0.25, 0.6, 0.3]
    },
    "plane_belt_right_lower" : {
        "group" : "planes",
        "pos" : vector(26.5, 49.13, 22.77) * millimeter,
        "shape" : "plane",
        "normal" : vector(0.0, -0.639819, 0.768526)
    },
    "belt_right_lower" : {
        "group" : "belts",
        "pos" : vector(26.5, 50.28, 21.39) * millimeter,
        "shape" : "box",
        "size" : vector(104.66, 9.0, 3.6) * millimeter,
        "rotAxis" : vector(-0.685046, 0.685046, 0.247836),
        "rotDeg" : 152.161 * degree,
        "rgba" : [0.13, 0.14, 0.16, 1.0]
    },
    "belt_right_lower_mirrored" : {
        "group" : "belts_mirror",
        "pos" : vector(-26.5, 50.28, 21.39) * millimeter,
        "shape" : "box",
        "size" : vector(104.66, 9.0, 3.6) * millimeter,
        "rotAxis" : vector(-0.685046, 0.685046, 0.247836),
        "rotDeg" : 152.161 * degree,
        "rgba" : [0.9, 0.25, 0.6, 0.3]
    },
    "plane_belt_right_upper" : {
        "group" : "planes",
        "pos" : vector(26.5, 9.04, 53.39) * millimeter,
        "shape" : "plane",
        "normal" : vector(0.0, -0.909802, 0.415043)
    },
    "belt_right_upper" : {
        "group" : "belts",
        "pos" : vector(26.5, 7.4, 54.14) * millimeter,
        "shape" : "box",
        "size" : vector(104.66, 9.0, 3.6) * millimeter,
        "rotAxis" : vector(0.475702, -0.475702, 0.739875),
        "rotDeg" : 107.0064 * degree,
        "rgba" : [0.13, 0.14, 0.16, 1.0]
    },
    "belt_right_upper_mirrored" : {
        "group" : "belts_mirror",
        "pos" : vector(-26.5, 7.4, 54.14) * millimeter,
        "shape" : "box",
        "size" : vector(104.66, 9.0, 3.6) * millimeter,
        "rotAxis" : vector(0.475702, -0.475702, 0.739875),
        "rotDeg" : 107.0064 * degree,
        "rgba" : [0.9, 0.25, 0.6, 0.3]
    },
    "payload_battery" : {
        "group" : "electronics",
        "pos" : vector(0.0, 25.9, 100.2) * millimeter,
        "shape" : "box",
        "size" : vector(26.0, 35.0, 72.0) * millimeter,
        "rotAxis" : vector(-0.357407, 0.357407, 0.862856),
        "rotDeg" : 98.4211 * degree,
        "rgba" : [0.4, 0.74, 0.34, 0.85],
        "mass_g" : 115.0
    },
    "payload_pi" : {
        "group" : "electronics",
        "pos" : vector(0.0, 90.0, 108.0) * millimeter,
        "shape" : "box",
        "size" : vector(30.0, 65.0, 5.0) * millimeter,
        "rgba" : [0.4, 0.74, 0.34, 0.85],
        "mass_g" : 11.0
    },
    "payload_u2d2" : {
        "group" : "electronics",
        "pos" : vector(0.0, 90.0, 120.0) * millimeter,
        "shape" : "box",
        "size" : vector(18.0, 48.0, 14.9) * millimeter,
        "rgba" : [0.4, 0.74, 0.34, 0.85],
        "mass_g" : 9.0
    },
    "payload_power_board" : {
        "group" : "electronics",
        "pos" : vector(0.0, 90.0, 93.0) * millimeter,
        "shape" : "box",
        "size" : vector(30.0, 30.0, 25.0) * millimeter,
        "rgba" : [0.4, 0.74, 0.34, 0.85],
        "mass_g" : 56.0
    },
    "linkage_crank_servo" : {
        "group" : "righting",
        "pos" : vector(0.0, 115.5, 75.96) * millimeter,
        "shape" : "box",
        "size" : vector(20.0, 23.0, 34.0) * millimeter,
        "rgba" : [0.58, 0.42, 0.84, 0.85],
        "mass_g" : 23.0
    },
    "linkage_crank_servo_case_holes_horn" : {
        "group" : "righting",
        "pos" : vector(0.0, 127.0, 75.96) * millimeter,
        "shape" : "holes",
        "points" : [vector(8.0, 127.0, 90.96) * millimeter, vector(8.0, 127.0, 60.96) * millimeter, vector(-8.0, 127.0, 90.96) * millimeter, vector(-8.0, 127.0, 60.96) * millimeter]
    },
    "linkage_crank_servo_case_holes_back" : {
        "group" : "righting",
        "pos" : vector(0.0, 104.0, 75.96) * millimeter,
        "shape" : "holes",
        "points" : [vector(8.0, 104.0, 90.96) * millimeter, vector(8.0, 104.0, 60.96) * millimeter, vector(-8.0, 104.0, 90.96) * millimeter, vector(-8.0, 104.0, 60.96) * millimeter]
    },
    "linkage_crank_horn" : {
        "group" : "righting",
        "pos" : vector(0.0, 128.5, 83.46) * millimeter,
        "shape" : "cylinder",
        "radius" : 8.0 * millimeter,
        "length" : 3.0 * millimeter,
        "axis" : vector(0.0, 1.0, 0.0),
        "rgba" : [0.58, 0.42, 0.84, 0.85]
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
        "axis" : vector(-0.473492, 0.0, 0.880798),
        "rgba" : [0.58, 0.42, 0.84, 0.85]
    },
    "link_coupler_right" : {
        "group" : "righting",
        "pos" : vector(7.36, 130.0, 76.52) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 80.59 * millimeter,
        "axis" : vector(0.542037, 0.0, -0.840355),
        "rgba" : [0.58, 0.42, 0.84, 0.85]
    },
    "link_rocker_right" : {
        "group" : "righting",
        "pos" : vector(23.25, 130.0, 21.33) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 44.3 * millimeter,
        "axis" : vector(0.269058, 0.0, 0.963124),
        "rgba" : [0.58, 0.42, 0.84, 0.85]
    },
    "link_crank_left" : {
        "group" : "righting",
        "pos" : vector(9.25, 130.0, 95.9) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 31.01 * millimeter,
        "axis" : vector(0.596784, 0.0, 0.802402),
        "rgba" : [0.58, 0.42, 0.84, 0.85]
    },
    "link_coupler_left" : {
        "group" : "righting",
        "pos" : vector(-5.35, 130.0, 75.5) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 81.18 * millimeter,
        "axis" : vector(-0.587747, 0.0, -0.809045),
        "rgba" : [0.58, 0.42, 0.84, 0.85]
    },
    "link_rocker_left" : {
        "group" : "righting",
        "pos" : vector(-23.25, 130.0, 21.33) * millimeter,
        "shape" : "cylinder",
        "radius" : 1.5 * millimeter,
        "length" : 44.3 * millimeter,
        "axis" : vector(-0.269058, 0.0, 0.963124),
        "rgba" : [0.58, 0.42, 0.84, 0.85]
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
        "rgba" : [0.58, 0.42, 0.84, 0.85],
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
        "rgba" : [0.58, 0.42, 0.84, 0.85],
        "mass_g" : 20.0
    },
    "roof" : {
        "group" : "righting",
        "pos" : vector(0.0, 92.5, 127.5) * millimeter,
        "shape" : "capsule",
        "radius" : 37.5 * millimeter,
        "length" : 145.0 * millimeter,
        "axis" : vector(0.0, 1.0, 0.0),
        "rgba" : [0.58, 0.42, 0.84, 0.85],
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

export const AOW_PLANE_BOUNDS =
{
    (meter)      : [1e-5, 0.06, 500],
    (centimeter) : 6.0,
    (millimeter) : 60.0,
    (inch)       : 2.5,
    (foot)       : 0.2,
    (yard)       : 0.07
} as LengthBoundSpec;

// ---------------------------------------------------------------------------
// Shared drawing helpers
// ---------------------------------------------------------------------------
// Every feature below is a thin wrapper around these. Sharing them is not just
// about duplication: each component is drawn at the SAME sub-id whichever
// feature draws it, and an Onshape entity id is a deterministic function of
// the id of the operation that made it. Keying sub-ids by component NAME
// rather than by a loop counter is what makes a regeneration safe — adding or
// removing a component leaves every other component's ids untouched.

export function aowEnvelope(context is Context, id is Id, name is string)
{
    var c = AOW_LAYOUT[name];
    var subId = id + ("solid_" ~ name);

    // Dispatch on KNOWN shapes only. An unrecognised or missing shape must
    // draw nothing rather than fall through to code that dereferences keys the
    // entry does not have.
    if (c.shape == "box")
    {
        fCuboid(context, subId, {
                "corner1" : c.pos - c.size / 2,
                "corner2" : c.pos + c.size / 2
        });

        // fCuboid is axis-aligned only, so an oriented box is built square and
        // then rotated about its own centre. The axis and angle are
        // precomputed by the generator.
        if (c.rotAxis != undefined)
        {
            opTransform(context, id + ("rot_" ~ name), {
                    "bodies" : qCreatedBy(subId, EntityType.BODY),
                    "transform" : rotationAround(line(c.pos, c.rotAxis), c.rotDeg)
            });
        }
    }
    else if (c.shape == "cylinder" || c.shape == "capsule")
    {
        // fCylinder, not opCylinder — solid primitives are the f* family.
        // Capsules are drawn as plain cylinders: the end caps matter to the
        // contact model, not to clearance.
        var half = c.axis * c.length / 2;
        fCylinder(context, subId, {
                "topCenter" : c.pos + half,
                "bottomCenter" : c.pos - half,
                "radius" : c.radius
        });
    }
    else
    {
        return;     // "point" — nothing solid to draw
    }

    // Without this every body lands in the list as "Part N". Note that a name
    // the USER has since edited by hand can never be overwritten from
    // FeatureScript again — reset it under part > properties if you want the
    // generated name back.
    setProperty(context, {
            "entities" : qCreatedBy(subId, EntityType.BODY),
            "propertyType" : PropertyType.NAME,
            "value" : name
    });

    // Colour, for the entries that carry one — the belts, where a REAL run and
    // a MIRRORED one have to be told apart at a glance. Same hand-edit rule as
    // the name above: recolour a body in the UI and FeatureScript can never
    // set its appearance again. The alpha is what makes a mirrored belt read
    // as a ghost rather than a part.
    if (c.rgba != undefined)
    {
        setProperty(context, {
                "entities" : qCreatedBy(subId, EntityType.BODY),
                "propertyType" : PropertyType.APPEARANCE,
                "value" : color(c.rgba[0], c.rgba[1], c.rgba[2], c.rgba[3])
        });
    }
    setQueryVariable(context, "aow_q_" ~ name, qCreatedBy(subId, EntityType.BODY));
}

export function aowPoint(context is Context, id is Id, name is string)
{
    var c = AOW_LAYOUT[name];

    // A hole pattern is one entry carrying several positions. Each gets its
    // own sub-id keyed by INDEX — safe here, unlike a loop counter over the
    // whole layout, because the four corners of a rectangle cannot be
    // reordered or added to without the pattern itself changing.
    if (c.points != undefined)
    {
        for (var i = 0; i < size(c.points); i += 1)
        {
            var holeId = id + ("hole_" ~ name ~ "_" ~ i);
            opPoint(context, holeId, { "point" : c.points[i] });
            setProperty(context, {
                    "entities" : qCreatedBy(holeId, EntityType.BODY),
                    "propertyType" : PropertyType.NAME,
                    "value" : name ~ "_" ~ i
            });
        }
        setQueryVariable(context, "aow_q_" ~ name, qCreatedBy(id, EntityType.BODY));
        return;
    }

    var subId = id + ("point_" ~ name);
    opPoint(context, subId, { "point" : c.pos });
    setProperty(context, {
            "entities" : qCreatedBy(subId, EntityType.BODY),
            "propertyType" : PropertyType.NAME,
            "value" : name ~ "_origin"
    });
    setQueryVariable(context, "aow_q_" ~ name ~ "_point", qCreatedBy(subId, EntityType.BODY));
}

export function aowAxisPlane(context is Context, id is Id, name is string,
        size is ValueWithUnits)
{
    var c = AOW_LAYOUT[name];

    // Two kinds reach here. A cylinder or capsule gets a plane normal to its
    // own axis — for the fork that is the plane perpendicular to the STEERING
    // AXIS, which is the one you want for the head tube and any clamp, since
    // sketching those against a world plane is what puts the rake in wrong.
    // A "plane" entry carries its normal directly; those are the print planes
    // and they are not derived from any part.
    var nrm;
    if (c.shape == "cylinder" || c.shape == "capsule")
        nrm = c.axis;
    else if (c.shape == "plane")
        nrm = c.normal;
    else
        return;

    var subId = id + ("plane_" ~ name);
    opPlane(context, subId, {
            "plane" : plane(c.pos, nrm),
            "width" : size,
            "height" : size
    });
    // The plane is NOT NAMED, and it cannot be. Planes and mate connectors
    // carry no metadata — the UI derives their names from the FEATURE that
    // made them, and that derivation is hardcoded to the feature type literally
    // called `cPlane`. Both the filtered query (silently names nothing) and the
    // unfiltered one (throws, taking the plane with it) were dead ends for the
    // same underlying reason. So a plane is always "Plane N" under whatever
    // feature drew it. What works instead is the query variable below: name
    // the REFERENCE rather than the plane, and a downstream sketch picks
    // `#aow_q_fork_plane` out of its plane field without anyone clicking a
    // "Plane 9" that a regeneration might renumber.
    setQueryVariable(context, "aow_q_" ~ name ~ "_plane", qCreatedBy(subId));
}

export function aowFourBar(context is Context, id is Id)
{
    if (size(AOW_FOURBAR) == 0)
        return;     // built with --righting none

    // A real sketch of construction lines. Cheap because the mechanism is
    // planar: one plane at the linkage's fore/aft station and the 2D
    // coordinates are just (CAD x, CAD z).
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
        // Three-point arc: the crank tip's swept path. Construction, because
        // it is a keep-out boundary rather than a part.
        skArc(sk, a.name, {
                "start" : a.start,
                "mid" : a.mid,
                "end" : a.end,
                "construction" : true
        });
    }
    skSolve(sk);
    setQueryVariable(context, "aow_q_fourbar", qCreatedBy(id + "fourbar", EntityType.EDGE));
}

export function aowDrawGroup(context is Context, id is Id, definition is map,
        group is string)
{
    for (var name in keys(AOW_LAYOUT))
    {
        if (AOW_LAYOUT[name].group != group)
            continue;
        if (definition.drawEnvelopes)
            aowEnvelope(context, id, name);
        if (definition.drawPoints)
            aowPoint(context, id, name);
        if (definition.drawAxisPlanes)
            aowAxisPlane(context, id, name, definition.planeSize);
    }
}

// Shared dialog for every per-group feature, so they stay identical without
// the generator emitting the same annotations once per group.
export predicate aowGroupPredicate(definition is map)
{
    annotation { "Name" : "Draw envelopes", "Default" : true }
    definition.drawEnvelopes is boolean;

    annotation { "Name" : "Draw origin points" }
    definition.drawPoints is boolean;

    annotation { "Name" : "Draw axis planes" }
    definition.drawAxisPlanes is boolean;

    if (definition.drawAxisPlanes)
    {
        annotation { "Name" : "Plane size" }
        isLength(definition.planeSize, AOW_PLANE_BOUNDS);
    }
}


// ---------------------------------------------------------------------------
// Features
// ---------------------------------------------------------------------------
// Insert `AOW layout variables` FIRST and once. It draws nothing, cannot fail,
// and is what makes `#aow_servo_steer_z` resolve in every sketch below it.

annotation { "Feature Type Name" : "AOW layout variables" }
export const aowLayoutVariables = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        for (var name in keys(AOW_LAYOUT))
        {
            var c = AOW_LAYOUT[name];
            setVariable(context, "aow_" ~ name ~ "_x", c.pos[0]);
            setVariable(context, "aow_" ~ name ~ "_y", c.pos[1]);
            setVariable(context, "aow_" ~ name ~ "_z", c.pos[2]);
        }
        setVariable(context, "aow_fourbar_station", AOW_FOURBAR_STATION);
    });


annotation { "Feature Type Name" : "AOW drivetrain" }
export const aowGroupDrivetrain = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        aowGroupPredicate(definition);
    }
    {
        aowDrawGroup(context, id, definition, "drivetrain");
    });


annotation { "Feature Type Name" : "AOW steering" }
export const aowGroupSteering = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        aowGroupPredicate(definition);
    }
    {
        aowDrawGroup(context, id, definition, "steering");
    });


annotation { "Feature Type Name" : "AOW servos" }
export const aowGroupServos = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        aowGroupPredicate(definition);
    }
    {
        aowDrawGroup(context, id, definition, "servos");
    });


annotation { "Feature Type Name" : "AOW mount" }
export const aowGroupMount = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        aowGroupPredicate(definition);
    }
    {
        aowDrawGroup(context, id, definition, "mount");
    });


annotation { "Feature Type Name" : "AOW electronics" }
export const aowGroupElectronics = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        aowGroupPredicate(definition);
    }
    {
        aowDrawGroup(context, id, definition, "electronics");
    });


annotation { "Feature Type Name" : "AOW planes" }
export const aowGroupPlanes = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Plane size" }
        isLength(definition.planeSize, AOW_PLANE_BOUNDS);
    }
    {
        // No checkboxes. Everything in this group IS a plane, so the three the
        // other groups carry would read "draw nothing", "draw nothing" and
        // "draw the only thing there is" — insert it and it works. The dialog
        // is synthesised rather than read off the definition so that
        // `aowDrawGroup` stays the single code path.
        aowDrawGroup(context, id, {
                "drawEnvelopes" : false,
                "drawPoints" : false,
                "drawAxisPlanes" : true,
                "planeSize" : definition.planeSize
        }, "planes");
    });


annotation { "Feature Type Name" : "AOW belts" }
export const aowGroupBelts = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        aowGroupPredicate(definition);
    }
    {
        aowDrawGroup(context, id, definition, "belts");
    });


annotation { "Feature Type Name" : "AOW belts mirror" }
export const aowGroupBeltsMirror = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        aowGroupPredicate(definition);
    }
    {
        aowDrawGroup(context, id, definition, "belts_mirror");
    });


annotation { "Feature Type Name" : "AOW righting" }
export const aowGroupRighting = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        aowGroupPredicate(definition);
    }
    {
        aowDrawGroup(context, id, definition, "righting");
    });


annotation { "Feature Type Name" : "AOW four-bar sketch" }
export const aowFourBarSketch = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        aowFourBar(context, id);
    });


// ---------------------------------------------------------------------------
// The original single-node feature. SUPERSEDED by the features above, and kept
// only because a Part Studio that already has it inserted would lose all of
// its geometry — and every downstream reference into that geometry — the
// moment this feature type stopped existing. It delegates to the same helpers
// at the same sub-ids, so every entity id it produces is the one it produced
// before and nothing downstream of it moves. Only the ORDER of creation
// changed (interleaved per component rather than all solids, then all
// points), which the part list shows and nothing else depends on.
// Prefer the per-group features for new work; they can be named.

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
        isLength(definition.planeSize, AOW_PLANE_BOUNDS);

        // A NAMED SECTION, not nine loose checkboxes. Appended bare they land
        // under a numeric field at the bottom of a fifteen-field dialog, below
        // the fold and visually unrelated to the draw toggles above — which is
        // exactly where the first version of this put them, and nobody found
        // them. A group heading makes the block one obvious thing.
        annotation { "Group Name" : "Which groups to draw", "Collapsed By Default" : false }
        {
            annotation { "Name" : "Draw drivetrain", "Default" : true }
            definition.drawDrivetrain is boolean;

            annotation { "Name" : "Draw steering", "Default" : true }
            definition.drawSteering is boolean;

            annotation { "Name" : "Draw servos", "Default" : true }
            definition.drawServos is boolean;

            annotation { "Name" : "Draw mount", "Default" : true }
            definition.drawMount is boolean;

            annotation { "Name" : "Draw electronics", "Default" : true }
            definition.drawElectronics is boolean;

            annotation { "Name" : "Draw planes", "Default" : true }
            definition.drawPlanes is boolean;

            annotation { "Name" : "Draw belts", "Default" : true }
            definition.drawBelts is boolean;

            annotation { "Name" : "Draw belts mirror", "Default" : true }
            definition.drawBeltsMirror is boolean;

            annotation { "Name" : "Draw righting", "Default" : true }
            definition.drawRighting is boolean;
        }
    }
    {
        // PER-GROUP SWITCHES. The per-group FEATURES above are still the
        // better answer — a tree node can be renamed, reordered, suppressed
        // and folded, and a checkbox can do none of those — but a document
        // already built on this single node cannot get any of that without
        // re-inserting eleven features and losing every entity id. So the
        // groups are ALSO switchable here, and "hide all the electronics" is
        // one tickbox rather than a migration.
        //
        // TESTED FOR `== false`, NOT `!= true`, and the difference matters.
        // These parameters are NEW on a feature that is already inserted in a
        // live document. If Onshape does not backfill the annotation default
        // into an existing instance, every one of them reads `undefined` —
        // and under `!= true` the entire model would silently disappear on
        // the next regeneration. Under `== false` an unset parameter DRAWS,
        // so the worst case is that a checkbox does nothing until the feature
        // is edited once. Wrong-but-visible beats wrong-and-empty.
        var groupOn = {
            "drivetrain" : definition.drawDrivetrain,
            "steering" : definition.drawSteering,
            "servos" : definition.drawServos,
            "mount" : definition.drawMount,
            "electronics" : definition.drawElectronics,
            "planes" : definition.drawPlanes,
            "belts" : definition.drawBelts,
            "belts_mirror" : definition.drawBeltsMirror,
            "righting" : definition.drawRighting
        };

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

        for (var name in keys(AOW_LAYOUT))
        {
            // Gated on the group BEFORE the shape switches, so a group that is
            // off costs nothing and cannot fail.
            if (groupOn[AOW_LAYOUT[name].group] == false)
                continue;
            if (definition.drawEnvelopes)
                aowEnvelope(context, id, name);
            if (definition.drawPoints)
                aowPoint(context, id, name);
            if (definition.drawAxisPlanes)
                aowAxisPlane(context, id, name, definition.planeSize);
        }

        if (definition.drawFourBarSketch)
            aowFourBar(context, id);
    });
