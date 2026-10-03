FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_drop_rig --push drop_rig_features
 *
 * Numbers from config/drop_rig_cad.yaml, cad_steering's and cad_drive's
 * layouts (the fork and chainstay joints) and config/servo_mounts.yaml, every
 * derived one computed in aow_sim.cad_drop_rig's layout() and carried here as
 * RIG. Millimetres.
 *
 * RIG FRAME: origin on the drop sensor's button top, +Z up, +X along the arm
 * toward the hinge, +Y along the wheel's axle.
 */

export const X330 = {
    "caseDepth" : 23 * millimeter,
    "caseWidth" : 20 * millimeter,
    "caseHeight" : 34 * millimeter,
    "shaftFromEnd" : 9.5 * millimeter,
    "hornThickness" : 3 * millimeter,
    "hornDiameter" : 16 * millimeter
};

export const RIG = {
    "zPcbTop" : -8.25 * millimeter,
    "zPcbBot" : -9.85 * millimeter,
    "zBase" : -16.85 * millimeter,
    "tBase" : 5.6 * millimeter,
    "zBaseBot" : -22.45 * millimeter,
    "shellY0" : 14.5 * millimeter,
    "shellY1" : 28.55 * millimeter,
    "botOuter" : 14.05 * millimeter,
    "backZ" : -26 * millimeter,
    "capOut" : -30 * millimeter,
    "nestTop" : -7.7 * millimeter,
    "capOuter" : -0.5 * millimeter,
    "xCv0" : -7.5 * millimeter,
    "cvWallY" : 26.85 * millimeter,
    "legY1" : 30.55 * millimeter,
    "Xc" : 80 * millimeter,
    "Yh" : 3 * millimeter,
    "Zc" : 13.7 * millimeter,
    "wellT" : 2.6 * millimeter,
    "camT" : 5 * millimeter,
    "camGap" : 0.5 * millimeter,
    "coverGap" : 1 * millimeter,
    "collarR" : 10 * millimeter,
    "Zf" : 26.7 * millimeter,
    "rCamMax" : 15 * millimeter,
    "coverTop" : 25.55 * millimeter,
    "padClear" : 1.15 * millimeter,
    "padHalfY" : 3.25 * millimeter,
    "padX0" : 75.5 * millimeter,
    "padX1" : 81 * millimeter,
    "parkClear" : 0.873301 * millimeter,
    "RaF" : 51.25 * millimeter,
    "RaR" : 51.2 * millimeter,
    "camClearF" : 22.1242 * millimeter,
    "camClearR" : 22.153 * millimeter,
    "hTop" : 9 * millimeter,
    "barH" : 5.75 * millimeter,
    "barW" : 19.75 * millimeter,
    "hWall" : 2.25 * millimeter,
    "housingH" : 10.25 * millimeter,
    "housingHalfW" : 12.125 * millimeter,
    "slotFloor" : 2 * millimeter,
    "slotDepth" : 20 * millimeter,
    "endF" : 67.25 * millimeter,
    "endR" : 71.1053 * millimeter,
    "slotX0" : 73.1053 * millimeter,
    "forkSlotIn" : 12.5 * millimeter,
    "webX0F" : 59.95 * millimeter,
    "webX0R" : 60 * millimeter,
    "spare" : 150 * millimeter,
    "Rr" : 51.2 * millimeter,
    "rimHalfR" : 16.5 * millimeter,
    "buttonClear" : 6.58407 * millimeter,
    "bx0" : -38.29 * millimeter,
    "bx1" : 97.05 * millimeter,
    "by0" : -36.05 * millimeter,
    "by1" : 36 * millimeter,
    "soR" : 3.5 * millimeter,
    "pilotR" : 1.25 * millimeter,
    "pilotBot" : -23.45 * millimeter,
    "px0" : -35.29 * millimeter,
    "px1" : 61.23 * millimeter,
    "py0" : -33.05 * millimeter,
    "py1" : 32.99 * millimeter,
    "pcbHoleR" : 1.63 * millimeter,
    "fsHalfX" : 8.635 * millimeter,
    "fsY0" : -12.7 * millimeter,
    "fsY1" : 12.4 * millimeter,
    "buttonR" : 6.1 * millimeter,
    "head_recess" : 1 * millimeter,
    "S_blockBot" : 55.25 * millimeter,
    "S_blockTop" : 67.25 * millimeter,
    "S_xBlk" : 17.5 * millimeter,
    "S_blockHalf" : 9 * millimeter,
    "S_zFS" : 61.25 * millimeter,
    "S_forkPocket" : 0.4 * millimeter,
    "S_xIn" : 13.5 * millimeter,
    "S_xOut" : 22.5 * millimeter,
    "S_forkHalf" : 12.1 * millimeter,
    "S_cheekIn" : 9.1 * millimeter,
    "S_legHalf" : 7.5 * millimeter,
    "S_tireW" : 24 * millimeter,
    "S_R" : 51.25 * millimeter,
    "D_xPo" : 20.6 * millimeter,
    "D_Yc0" : 54.2 * millimeter,
    "D_Yj1" : 67.5179 * millimeter,
    "D_Yr0" : 47.5179 * millimeter,
    "D_Yscr" : 61.2339 * millimeter,
    "D_travel" : 3 * millimeter,
    "D_tongueTop" : 18.2 * millimeter,
    "D_tongueEnd" : 18.5 * millimeter,
    "D_tongueHalf" : 3.5 * millimeter,
    "D_tongueClr" : 0.15 * millimeter,
    "D_xN0" : 16.8 * millimeter,
    "D_nutSlotT" : 3 * millimeter,
    "D_tipSlot" : 0.6 * millimeter,
    "D_holeR" : 1.8 * millimeter,
    "D_nutHalfY" : 3.275 * millimeter,
    "D_nutCornerY" : 3.78164 * millimeter,
    "D_xCi" : 35.2 * millimeter,
    "D_xCo" : 40.2 * millimeter,
    "D_armHalf" : 9.5 * millimeter,
    "D_csBossR" : 9.5 * millimeter
};

export const CAM = [
    vector(3.10583, 11.5911) * millimeter,
    vector(2.71642, 11.6885) * millimeter,
    vector(2.32396, 11.7728) * millimeter,
    vector(1.92891, 11.844) * millimeter,
    vector(1.5317, 11.9018) * millimeter,
    vector(1.13277, 11.9464) * millimeter,
    vector(0.732582, 11.9776) * millimeter,
    vector(0.33157, 11.9954) * millimeter,
    vector(-0.0698128, 11.9998) * millimeter,
    vector(-0.471118, 11.9907) * millimeter,
    vector(-0.871896, 11.9683) * millimeter,
    vector(-1.2717, 11.9324) * millimeter,
    vector(-1.67008, 11.8832) * millimeter,
    vector(-1.95424, 11.8905) * millimeter,
    vector(-2.23965, 11.8909) * millimeter,
    vector(-2.52613, 11.8845) * millimeter,
    vector(-2.81351, 11.8711) * millimeter,
    vector(-3.10164, 11.8508) * millimeter,
    vector(-3.39034, 11.8235) * millimeter,
    vector(-3.67944, 11.7892) * millimeter,
    vector(-3.96877, 11.7477) * millimeter,
    vector(-4.25815, 11.6992) * millimeter,
    vector(-4.54742, 11.6435) * millimeter,
    vector(-4.83638, 11.5807) * millimeter,
    vector(-5.12488, 11.5107) * millimeter,
    vector(-5.41273, 11.4335) * millimeter,
    vector(-5.69975, 11.3491) * millimeter,
    vector(-5.98576, 11.2576) * millimeter,
    vector(-6.27059, 11.1588) * millimeter,
    vector(-6.55405, 11.0529) * millimeter,
    vector(-6.83596, 10.9398) * millimeter,
    vector(-7.11614, 10.8196) * millimeter,
    vector(-7.39441, 10.6922) * millimeter,
    vector(-7.6706, 10.5577) * millimeter,
    vector(-7.94451, 10.4161) * millimeter,
    vector(-8.21597, 10.2674) * millimeter,
    vector(-8.4848, 10.1118) * millimeter,
    vector(-8.75081, 9.94916) * millimeter,
    vector(-9.01384, 9.77961) * millimeter,
    vector(-9.27369, 9.60319) * millimeter,
    vector(-9.53019, 9.41995) * millimeter,
    vector(-9.78318, 9.22995) * millimeter,
    vector(-10.0325, 9.03326) * millimeter,
    vector(-10.4169, 8.58706) * millimeter,
    vector(-10.7816, 8.1245) * millimeter,
    vector(-11.1257, 7.64648) * millimeter,
    vector(-11.4486, 7.15391) * millimeter,
    vector(-11.7498, 6.64772) * millimeter,
    vector(-12.0286, 6.12887) * millimeter,
    vector(-12.2845, 5.59836) * millimeter,
    vector(-12.517, 5.05719) * millimeter,
    vector(-12.7257, 4.50639) * millimeter,
    vector(-12.9101, 3.94702) * millimeter,
    vector(-13.07, 3.38013) * millimeter,
    vector(-13.205, 2.80681) * millimeter,
    vector(-11.5911, 3.10583) * millimeter,
    vector(-11.6885, 2.71642) * millimeter,
    vector(-11.7728, 2.32396) * millimeter,
    vector(-11.844, 1.92891) * millimeter,
    vector(-11.9018, 1.5317) * millimeter,
    vector(-11.9464, 1.13277) * millimeter,
    vector(-11.9776, 0.732582) * millimeter,
    vector(-11.9954, 0.33157) * millimeter,
    vector(-11.9998, -0.0698128) * millimeter,
    vector(-11.9907, -0.471118) * millimeter,
    vector(-11.9683, -0.871896) * millimeter,
    vector(-11.9324, -1.2717) * millimeter,
    vector(-11.8832, -1.67008) * millimeter,
    vector(-11.9069, -1.95695) * millimeter,
    vector(-11.9237, -2.24582) * millimeter,
    vector(-11.9334, -2.53652) * millimeter,
    vector(-11.936, -2.82889) * millimeter,
    vector(-11.9315, -3.12274) * millimeter,
    vector(-11.9196, -3.4179) * millimeter,
    vector(-11.9005, -3.7142) * millimeter,
    vector(-11.874, -4.01144) * millimeter,
    vector(-11.8401, -4.30945) * millimeter,
    vector(-11.7987, -4.60805) * millimeter,
    vector(-11.7498, -4.90704) * millimeter,
    vector(-11.6934, -5.20623) * millimeter,
    vector(-11.6293, -5.50544) * millimeter,
    vector(-11.5576, -5.80447) * millimeter,
    vector(-11.4783, -6.10313) * millimeter,
    vector(-11.3913, -6.40123) * millimeter,
    vector(-11.2966, -6.69856) * millimeter,
    vector(-11.1942, -6.99493) * millimeter,
    vector(-11.0841, -7.29015) * millimeter,
    vector(-10.9663, -7.58401) * millimeter,
    vector(-10.8408, -7.87632) * millimeter,
    vector(-10.7076, -8.16687) * millimeter,
    vector(-10.5667, -8.45547) * millimeter,
    vector(-10.4182, -8.74191) * millimeter,
    vector(-10.262, -9.02599) * millimeter,
    vector(-10.0982, -9.30752) * millimeter,
    vector(-9.92689, -9.58629) * millimeter,
    vector(-9.748, -9.86209) * millimeter,
    vector(-9.56163, -10.1347) * millimeter,
    vector(-9.36783, -10.404) * millimeter,
    vector(-8.9051, -10.8027) * millimeter,
    vector(-8.42541, -11.1809) * millimeter,
    vector(-7.92969, -11.5378) * millimeter,
    vector(-7.41887, -11.8727) * millimeter,
    vector(-6.89393, -12.185) * millimeter,
    vector(-6.35587, -12.4741) * millimeter,
    vector(-5.80571, -12.7395) * millimeter,
    vector(-5.24449, -12.9806) * millimeter,
    vector(-4.6733, -13.197) * millimeter,
    vector(-4.0932, -13.3883) * millimeter,
    vector(-3.50532, -13.5541) * millimeter,
    vector(-2.91076, -13.6941) * millimeter,
    vector(-3.10583, -11.5911) * millimeter,
    vector(-2.71642, -11.6885) * millimeter,
    vector(-2.32396, -11.7728) * millimeter,
    vector(-1.92891, -11.844) * millimeter,
    vector(-1.5317, -11.9018) * millimeter,
    vector(-1.13277, -11.9464) * millimeter,
    vector(-0.732582, -11.9776) * millimeter,
    vector(-0.33157, -11.9954) * millimeter,
    vector(0.0698128, -11.9998) * millimeter,
    vector(0.471118, -11.9907) * millimeter,
    vector(0.871896, -11.9683) * millimeter,
    vector(1.2717, -11.9324) * millimeter,
    vector(1.67008, -11.8832) * millimeter,
    vector(1.95965, -11.9234) * millimeter,
    vector(2.25199, -11.9564) * millimeter,
    vector(2.54692, -11.9823) * millimeter,
    vector(2.84426, -12.0009) * millimeter,
    vector(3.14384, -12.0121) * millimeter,
    vector(3.44547, -12.0158) * millimeter,
    vector(3.74896, -12.0119) * millimeter,
    vector(4.05412, -12.0004) * millimeter,
    vector(4.36076, -11.9811) * millimeter,
    vector(4.66868, -11.954) * millimeter,
    vector(4.97769, -11.919) * millimeter,
    vector(5.28758, -11.8761) * millimeter,
    vector(5.59815, -11.8252) * millimeter,
    vector(5.90919, -11.7662) * millimeter,
    vector(6.2205, -11.6991) * millimeter,
    vector(6.53186, -11.6238) * millimeter,
    vector(6.84307, -11.5403) * millimeter,
    vector(7.15391, -11.4486) * millimeter,
    vector(7.46416, -11.3487) * millimeter,
    vector(7.77362, -11.2405) * millimeter,
    vector(8.08205, -11.124) * millimeter,
    vector(8.38924, -10.9992) * millimeter,
    vector(8.69497, -10.8661) * millimeter,
    vector(8.99903, -10.7246) * millimeter,
    vector(9.30118, -10.5749) * millimeter,
    vector(9.6012, -10.4169) * millimeter,
    vector(9.89888, -10.2506) * millimeter,
    vector(10.194, -10.0761) * millimeter,
    vector(10.4863, -9.89332) * millimeter,
    vector(10.7756, -9.70239) * millimeter,
    vector(11.1886, -9.22313) * millimeter,
    vector(11.5802, -8.72632) * millimeter,
    vector(11.9498, -8.21289) * millimeter,
    vector(12.2967, -7.68383) * millimeter,
    vector(12.6202, -7.14014) * millimeter,
    vector(12.9196, -6.58286) * millimeter,
    vector(13.1944, -6.01305) * millimeter,
    vector(13.4442, -5.4318) * millimeter,
    vector(13.6683, -4.8402) * millimeter,
    vector(13.8664, -4.23939) * millimeter,
    vector(14.0381, -3.63051) * millimeter,
    vector(14.1831, -3.01472) * millimeter,
    vector(11.5911, -3.10583) * millimeter,
    vector(11.6885, -2.71642) * millimeter,
    vector(11.7728, -2.32396) * millimeter,
    vector(11.844, -1.92891) * millimeter,
    vector(11.9018, -1.5317) * millimeter,
    vector(11.9464, -1.13277) * millimeter,
    vector(11.9776, -0.732582) * millimeter,
    vector(11.9954, -0.33157) * millimeter,
    vector(11.9998, 0.0698128) * millimeter,
    vector(11.9907, 0.471118) * millimeter,
    vector(11.9683, 0.871896) * millimeter,
    vector(11.9324, 1.2717) * millimeter,
    vector(11.8832, 1.67008) * millimeter,
    vector(11.9398, 1.96235) * millimeter,
    vector(11.9892, 2.25816) * millimeter,
    vector(12.0312, 2.55731) * millimeter,
    vector(12.0658, 2.85964) * millimeter,
    vector(12.0927, 3.16494) * millimeter,
    vector(12.1119, 3.47303) * millimeter,
    vector(12.1233, 3.78371) * millimeter,
    vector(12.1267, 4.09679) * millimeter,
    vector(12.122, 4.41206) * millimeter,
    vector(12.1092, 4.72931) * millimeter,
    vector(12.0882, 5.04834) * millimeter,
    vector(12.0588, 5.36892) * millimeter,
    vector(12.021, 5.69085) * millimeter,
    vector(11.9747, 6.01391) * millimeter,
    vector(11.9198, 6.33787) * millimeter,
    vector(11.8563, 6.6625) * millimeter,
    vector(11.784, 6.98758) * millimeter,
    vector(11.7031, 7.31289) * millimeter,
    vector(11.6133, 7.63817) * millimeter,
    vector(11.5147, 7.96322) * millimeter,
    vector(11.4071, 8.28777) * millimeter,
    vector(11.2907, 8.61161) * millimeter,
    vector(11.1654, 8.93448) * millimeter,
    vector(11.031, 9.25614) * millimeter,
    vector(10.8878, 9.57636) * millimeter,
    vector(10.7355, 9.89489) * millimeter,
    vector(10.5743, 10.2115) * millimeter,
    vector(10.4041, 10.5259) * millimeter,
    vector(10.225, 10.8379) * millimeter,
    vector(10.037, 11.1472) * millimeter,
    vector(9.54117, 11.5744) * millimeter,
    vector(9.02723, 11.9795) * millimeter,
    vector(8.49609, 12.3619) * millimeter,
    vector(7.94879, 12.7207) * millimeter,
    vector(7.38635, 13.0553) * millimeter,
    vector(6.80986, 13.3651) * millimeter,
    vector(6.2204, 13.6494) * millimeter,
    vector(5.6191, 13.9078) * millimeter,
    vector(5.0071, 14.1396) * millimeter,
    vector(4.38558, 14.3446) * millimeter,
    vector(3.7557, 14.5222) * millimeter,
    vector(3.11868, 14.6722) * millimeter
];

export const PCB_HOLES = [
    vector(-30.21, 24.1) * millimeter,
    vector(-30.21, -10.19) * millimeter,
    vector(56.15, -10.19) * millimeter,
    vector(56.15, 24.1) * millimeter,
    vector(12.97, 20.29) * millimeter
];

export const BUTTONS = [
    vector(51.07, 6.32) * millimeter,
    vector(25.94, 0) * millimeter,
    vector(0, 0) * millimeter,
    vector(-25.13, 6.32) * millimeter
];

export const FIXTURE = {
    "joint_plate" : 4.6 * millimeter,
    "ridge_height" : 1.2 * millimeter,
    "ridge_flat" : 4.6 * millimeter,
    "ridge_clearance" : 0.15 * millimeter,
    "bridge_layer" : 0.2 * millimeter,
    "head_recess" : 1 * millimeter
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

/**
 * A 6-32 joint with NO ridge: screwJoint's bore, nut slot and teardrops
 * (the steering's plainScrewJoint, verbatim).
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

/** Subtract `tools` from `target`. */
export function cutFrom(context is Context, id is Id, target is Query, tools is array)
{
    opBoolean(context, id, { "tools" : qUnion(tools), "targets" : target,
            "operationType" : BooleanOperationType.SUBTRACTION });
}

/**
 * The bar housing, the web and the follower pad, common to both interfaces:
 * from the block's +X end `x0`, the housing's top flush with the block's top
 * (zTop). Returns the bodies to unite and the bar slot to cut.
 */
export function barAndFollower(context is Context, id is Id, x0 is ValueWithUnits,
                               zTop is ValueWithUnits, xs is ValueWithUnits) returns map
{
    const L  = RIG;
    const mm = millimeter;
    const v  = function(x, y, z) returns Vector { return vector(x, y, z); };
    const x1 = L.slotX0 + L.slotDepth;
    const zB = zTop - L.housingH;
    const housing = boxW(context, id + "housing", v(x0 - 1 * mm, -L.housingHalfW, zB), v(x1, L.housingHalfW, zTop));
    // the web in (X, Z): the pad's face Zf from padX0 to padX1 (the release
    // corner, square); upstream of padX0 its underside rises at 45 deg to the
    // block's underside at xs,
    // so it prints X+ off the block with no overhang past 45 deg
    const zBlk = zTop - 2 * L.hTop;
    polyPrism(context, id, "web", v(0 * mm, L.padHalfY, 0 * mm), vector(0, -1, 0), vector(1, 0, 0),
              [vector(xs, zBlk + 0.5 * mm), vector(xs, zBlk), vector(L.padX0, L.Zf), vector(L.padX1, L.Zf),
               vector(L.padX1, zB + 0.5 * mm), vector(x0 - 1 * mm, zB + 0.5 * mm), vector(x0 - 1 * mm, zBlk + 0.5 * mm)],
              2 * L.padHalfY);
    const web = qCreatedBy(id + "webExt", EntityType.BODY);
    const slot = boxW(context, id + "bar", v(L.slotX0, -L.barW / 2, zTop - L.hWall - L.barH),
                      v(x1 + 1 * mm, L.barW / 2, zTop - L.hWall));
    return { "bodies" : [housing, web], "slot" : slot };
}

/**
 * The front interface: cad_steering's headset block (fork ledges under its
 * -X face, cheeks round its +-Z faces, one 6-32 a side into a nut slot along
 * X), then the bar housing and the follower. With the fork mock and tire
 * when `mocks`. Returns the part's point and the hardware.
 */
export function frontInterface(context is Context, id is Id, mocks is boolean) returns map
{
    const L  = RIG;
    const mm = millimeter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const v  = function(x, y, z) returns Vector { return vector(x, y, z); };
    const za = L.RaF;
    const block = boxW(context, id + "block", v(L.S_blockBot, -L.S_xBlk, za - L.S_blockHalf),
                       v(L.S_blockTop, L.S_xBlk, za + L.S_blockHalf));
    const bf = barAndFollower(context, id + "bf", L.S_blockTop, za + L.hTop, L.webX0F);
    unite(context, id + "u", concatenateArrays([[block], bf.bodies]));
    const pt = v(L.S_blockBot + 1 * mm, 0 * mm, za + L.S_blockHalf - 1 * mm);
    cutFrom(context, id + "barCut", partAt(context, id, pt), [bf.slot]);
    var hw = [];
    // the fork mock: leg up from the axle, the plate beside the block, cheeks
    var forkPts = [];
    for (var sg in [1, -1])
    {
        const tag = sg > 0 ? "P" : "N";
        const yi = function(a, b) returns array { return sg > 0 ? [a, b] : [-b, -a]; };
        const yl = yi(L.S_xIn, L.S_xOut);
        const yp = yi(L.S_xBlk, L.S_xOut);
        const yc = yi(L.S_xIn, L.S_xBlk);
        var fb = [boxW(context, id + ("leg" ~ tag), v(0 * mm, yl[0], za - L.S_legHalf), v(L.S_blockBot, yl[1], za + L.S_legHalf)),
                  boxW(context, id + ("plate" ~ tag), v(L.S_blockBot - 1 * mm, yp[0], za - L.S_forkHalf),
                       v(L.S_blockTop, yp[1], za + L.S_forkHalf))];
        for (var sz in [1, -1])
            fb = append(fb, boxW(context, id + ("cheek" ~ tag ~ (sz > 0 ? "P" : "N")),
                    v(L.S_blockBot - 1 * mm, yc[0], sz > 0 ? za + L.S_cheekIn : za - L.S_forkHalf),
                    v(L.S_blockTop, yc[1], sz > 0 ? za + L.S_forkHalf : za - L.S_cheekIn)));
        unite(context, id + ("forkU" ~ tag), fb);
        forkPts = append(forkPts, v(1 * mm, sg * (L.S_xOut - 1 * mm), za));
    }
    for (var i = 0; i < 2; i += 1)
    {
        const sg = i == 0 ? 1 : -1;
        plainScrewJoint(context, id + ("j" ~ i), v(L.S_zFS, sg * (L.S_xBlk + FIXTURE.joint_plate), za),
                        -sg * Y, X, L.S_forkPocket, partAt(context, id, forkPts[i]), partAt(context, id, pt),
                        -sg * Y, X);
        hw = append(hw, [partAt(context, id, forkPts[i]), "fork mock", color(0.93, 0.56, 0.20)]);
    }
    if (mocks)
    {
        const tire = cylW(context, id + "tire", v(0 * mm, -L.S_tireW / 2, za), v(0 * mm, L.S_tireW / 2, za), L.S_R);
        hw = append(hw, [tire, "front tire", color(0.10, 0.10, 0.11)]);
    }
    else
        opDeleteBodies(context, id + "noMock", { "entities" : qUnion([partAt(context, id, forkPts[0]),
                partAt(context, id, forkPts[1])]) });
    return { "pt" : pt, "hw" : mocks ? hw : [] };
}

/**
 * The rear interface: one block between the chainstays, its +-Y faces where
 * the case sides' outer faces are, each with cad_drive's channel, screw slot
 * (the tension travel), nut slot from the -X face and tip slot. Then the bar
 * housing and the follower. With chainstay mocks and the wheel when `mocks`.
 */
export function rearInterface(context is Context, id is Id, mocks is boolean) returns map
{
    const L  = RIG;
    const mm = millimeter;
    const v  = function(x, y, z) returns Vector { return vector(x, y, z); };
    const za = L.RaR;
    const block = boxW(context, id + "block", v(L.D_Yc0, -L.D_xPo, za - L.hTop), v(L.endR, L.D_xPo, za + L.hTop));
    const bf = barAndFollower(context, id + "bf", L.endR, za + L.hTop, L.webX0R);
    unite(context, id + "u", concatenateArrays([[block], bf.bodies]));
    const pt = v(L.endR - 1 * mm, 0 * mm, za + L.hTop - 1 * mm);
    var cuts = [bf.slot];
    const chW = L.D_tongueHalf + L.D_tongueClr;
    const xNc = L.D_xN0 - L.D_nutSlotT;
    for (var sg in [1, -1])
    {
        const tag = sg > 0 ? "P" : "N";
        // y between two module-X magnitudes, on this side
        const yb = function(a, b) returns array { return sg > 0 ? [min(a, b), max(a, b)] : [-max(a, b), -min(a, b)]; };
        const ych = yb(L.D_tongueTop, L.D_xPo + 1 * mm);
        cuts = append(cuts, boxW(context, id + ("chan" ~ tag), v(L.D_Yc0 - 1 * mm, ych[0], za - chW), v(L.D_Yj1, ych[1], za + chW)));
        // the screw's slot (as long as the travel) from the channel floor to
        // the tip slot's end; the nut slot in from the -X face
        const ysl = yb(L.D_tongueTop + 0.01 * mm, xNc - L.D_tipSlot);
        for (var e in [[L.D_Yscr, "a"], [L.D_Yscr - L.D_travel, "b"]])
            cuts = append(cuts, cylW(context, id + ("sl" ~ tag ~ e[1]), v(e[0], ysl[0], za), v(e[0], ysl[1], za), L.D_holeR));
        cuts = append(cuts, boxW(context, id + ("slB" ~ tag), v(L.D_Yscr - L.D_travel, ysl[0], za - L.D_holeR),
                                 v(L.D_Yscr, ysl[1], za + L.D_holeR)));
        // its +X end is the crown as printed (X+): a 45 deg point on it
        const rB = L.D_holeR;
        polyPrism(context, id, "td" ~ tag, v(L.D_Yscr, ysl[0], za), vector(0, 1, 0), vector(0, 0, 1),
                  [vector(0 * mm, 0 * mm), vector(rB / sqrt(2), rB / sqrt(2)), vector(0 * mm, rB * sqrt(2)),
                   vector(-rB / sqrt(2), rB / sqrt(2))], ysl[1] - ysl[0]);
        cuts = append(cuts, qCreatedBy(id + ("td" ~ tag ~ "Ext"), EntityType.BODY));
        const yn = yb(L.D_xN0, xNc);
        cuts = append(cuts, boxW(context, id + ("nut" ~ tag), v(L.D_Yc0 - 1 * mm, yn[0], za - L.D_nutHalfY),
                                 v(L.D_Yscr + L.D_nutCornerY + 0.2 * mm, yn[1], za + L.D_nutHalfY)));
    }
    cutFrom(context, id + "cut", partAt(context, id, pt), cuts);
    var hw = [];
    if (mocks)
    {
        for (var sg in [1, -1])
        {
            const tag = sg > 0 ? "P" : "N";
            const yb = function(a, b) returns array { return sg > 0 ? [min(a, b), max(a, b)] : [-max(a, b), -min(a, b)]; };
            const yBed = yb(L.D_xCi, L.D_xCo);
            const yRis = yb(L.D_xPo, L.D_xCi + 0.5 * mm);
            const yTon = yb(L.D_tongueEnd, L.D_xPo + 0.5 * mm);
            unite(context, id + ("csU" ~ tag), [
                    boxW(context, id + ("bed" ~ tag), v(0 * mm, yBed[0], za - L.D_armHalf), v(L.D_Yj1, yBed[1], za + L.D_armHalf)),
                    cylW(context, id + ("boss" ~ tag), v(0 * mm, yBed[0], za), v(0 * mm, yBed[1], za), L.D_csBossR),
                    boxW(context, id + ("riser" ~ tag), v(L.D_Yr0, yRis[0], za - L.D_armHalf), v(L.D_Yj1, yRis[1], za + L.D_armHalf)),
                    boxW(context, id + ("tongue" ~ tag), v(L.D_Yc0, yTon[0], za - L.D_tongueHalf), v(L.D_Yj1, yTon[1], za + L.D_tongueHalf))]);
            hw = append(hw, [partAt(context, id, v(1 * mm, sg * (L.D_xCo - 1 * mm), za)), "chainstay mock", color(0.93, 0.56, 0.20)]);
        }
        const wheel = cylW(context, id + "wheel", v(0 * mm, -L.rimHalfR, za), v(0 * mm, L.rimHalfR, za), L.Rr);
        hw = append(hw, [wheel, "rear wheel", color(0.10, 0.10, 0.11)]);
    }
    return { "pt" : pt, "hw" : hw };
}

/**
 * Every part and the hardware, with `opt.wheel` FRONT or REAR on the rig and
 * the other interface drawn RIG.spare along +Y. Returns each printed part
 * with the direction that prints UP.
 */
export function dropRigBuild(context is Context, id is Id, opt is map) returns map
{
    const L  = RIG;
    const mm = millimeter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const P  = id + "parts";
    const H  = id + "hw";
    const v  = function(x, y, z) returns Vector { return vector(x, y, z); };
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };
    const front = opt.wheel != "REAR";

    // ---- the servo: horn face at (Xc, Yh, Zc) facing -Y; local x = -X, so
    // local y = cross(-Y, -X) = -Z: its far end down
    step("servo");
    const cs = coordSystem(v(L.Xc, L.Yh, L.Zc), -X, -Y);
    const sv = x330Envelope(context, H + "servo", cs);

    // ---- the board and the sensors (envelopes, not the parts)
    step("pcb");
    const pcb = boxW(context, H + "pcb", v(L.px0, L.py0, L.zPcbBot), v(L.px1, L.py1, L.zPcbTop));
    var pcbHoles = [];
    for (var i = 0; i < size(PCB_HOLES); i += 1)
        pcbHoles = append(pcbHoles, cylW(context, H + ("ph" ~ i), v(PCB_HOLES[i][0], PCB_HOLES[i][1], L.zPcbBot - 1 * mm),
                                         v(PCB_HOLES[i][0], PCB_HOLES[i][1], L.zPcbTop + 1 * mm), L.pcbHoleR));
    cutFrom(context, H + "phCut", pcb, pcbHoles);
    var sensors = [];
    for (var i = 0; i < size(BUTTONS); i += 1)
    {
        const b = BUTTONS[i];
        const body = boxW(context, H + ("fsb" ~ i), v(b[0] - L.fsHalfX, b[1] + L.fsY0, L.zPcbTop), v(b[0] + L.fsHalfX, b[1] + L.fsY1, -2.5 * mm));
        const btn = cylW(context, H + ("fsk" ~ i), v(b[0], b[1], -3 * mm), v(b[0], b[1], 0 * mm), L.buttonR);
        unite(context, H + ("fsu" ~ i), [body, btn]);
        sensors = append(sensors, v(b[0], b[1], -1 * mm));
    }

    // ---- base: plate, standoffs with M3 pilots
    step("base");
    var baseBodies = [boxW(context, P + "plate", v(L.bx0, L.by0, L.zBaseBot), v(L.bx1, L.by1, L.zBase))];
    var pilots = [];
    for (var i = 0; i < size(PCB_HOLES); i += 1)
    {
        const h = PCB_HOLES[i];
        baseBodies = append(baseBodies, cylW(context, P + ("so" ~ i), v(h[0], h[1], L.zBase - 1 * mm), v(h[0], h[1], L.zPcbBot), L.soR));
        pilots = append(pilots, cylW(context, P + ("pl" ~ i), v(h[0], h[1], L.pilotBot), v(h[0], h[1], L.zPcbBot + 1 * mm), L.pilotR));
    }
    unite(context, P + "baseU", baseBodies);
    const basePt = v(L.bx0 + 1 * mm, L.by0 + 1 * mm, L.zBaseBot + 1 * mm);
    cutFrom(context, P + "pilots", partAt(context, P, basePt), pilots);

    // ---- back shell and cover, each with a leg down to the base (the X330
    // fixture's IDLER base and cover, turned so the horn faces +Y)
    step("back shell");
    boxIn(context, P + "legB", cs, vector(-L.botOuter, L.shellY0, L.capOut), vector(L.botOuter, L.legY1, L.nestTop));
    const backPt = toWorld(cs, vector(L.botOuter - 2 * mm, L.legY1 - 1 * mm, L.capOut + 2 * mm));
    // by a point, not by the box's id: the shell's union may keep either identity
    caseShellBuild(context, P + "back", cs, mergeMaps(CASE_OPT, { "part" : "BOTTOM" }), partAt(context, P, backPt));
    step("cover");
    caseShellBuild(context, P + "cover", cs, mergeMaps(CASE_OPT, { "part" : "TOP", "fullWrap" : true,
                   "caseCapThickness" : L.capOuter + X330.hornThickness - CASE_OPT.caseFaceClearance }), qNothing());
    // in the cap, beside the horn (the X330 fixture's own point)
    const coverShellPt = toWorld(cs, vector(0 * mm, 20 * mm, -1 * mm));
    const legC = boxIn(context, P + "legC", cs, vector(-L.botOuter, L.cvWallY - 1 * mm, L.xCv0), vector(L.botOuter, L.legY1, L.capOuter));
    opBoolean(context, P + "coverU", { "tools" : qUnion([legC, partAt(context, P, coverShellPt)]),
            "operationType" : BooleanOperationType.UNION });
    const coverPt = toWorld(cs, vector(L.botOuter - 2 * mm, L.legY1 - 1 * mm, L.capOuter - 1 * mm));

    // ---- the cam: the collar over the horn, the profile beyond it, the pins
    step("cam");
    const collar = cylIn(context, P + "collar", cs, vector(0 * mm, 0 * mm, -L.wellT), vector(0 * mm, 0 * mm, L.camGap + 0.5 * mm), L.collarR);
    const camO = toWorld(cs, vector(0 * mm, 0 * mm, L.camGap));
    // looking AT the horn (from -Y) the viewer's right is +X and up is +Z
    polyPrism(context, P, "camProf", camO, -Y, X, CAM, L.camT);
    unite(context, P + "camU", [collar, qCreatedBy(P + "camProfExt", EntityType.BODY)]);
    const camPt = toWorld(cs, vector(L.collarR + 0.5 * mm, 0 * mm, L.camGap + 0.5 * mm));
    servoMountBuild(context, P + "camHorn", cs, HORN_OPT, partAt(context, P, camPt));

    // ---- the joints: each leg onto the base from below, the head
    // counterbored under flush, the nut slot along Y
    step("joints");
    const baseQ = partAt(context, P, basePt);
    plainScrewJoint(context, P + "jB", toWorld(cs, vector(0 * mm, L.legY1 + L.tBase - FIXTURE.head_recess, (L.capOut + L.nestTop) / 2)),
                    Z, Y, FIXTURE.head_recess, baseQ, partAt(context, P, backPt), Z, -Y);
    plainScrewJoint(context, P + "jC", toWorld(cs, vector(0 * mm, L.legY1 + L.tBase - FIXTURE.head_recess, (L.xCv0 + L.capOuter) / 2)),
                    Z, Y, FIXTURE.head_recess, partAt(context, P, basePt), partAt(context, P, coverPt), Z, Y);

    // ---- the interfaces: the chosen one on the rig with its mocks, the
    // other moved aside
    step("front interface");
    const fi = frontInterface(context, P + "fi", front);
    step("rear interface");
    const ri = rearInterface(context, P + "ri", !front);
    const spareQ = front ? partAt(context, P + "ri", ri.pt) : partAt(context, P + "fi", fi.pt);
    opTransform(context, P + "spare", { "bodies" : spareQ, "transform" : transform(L.spare * Y) });
    const fiPt = front ? fi.pt : fi.pt + L.spare * Y;
    const riPt = front ? ri.pt + L.spare * Y : ri.pt;

    // ---- names, colours, print orientation
    step("dress");
    const partC  = color(0.62, 0.64, 0.68);
    const movesC = color(0.30, 0.55, 0.85);
    const parts = [["base", partAt(context, P, basePt), "Z+", Z, partC],
                   ["back shell", partAt(context, P, backPt), "Y-", -Y, partC],
                   ["cover", partAt(context, P, coverPt), "Y+", Y, partC],
                   ["cam", partAt(context, P, camPt), "Y+", Y, movesC],
                   ["front interface", partAt(context, P + "fi", fiPt), "X+", X, movesC],
                   ["rear interface", partAt(context, P + "ri", riPt), "X+", X, movesC]];
    var prints = [];
    for (var n in parts)
    {
        dress(context, n[1], n[0] ~ " [print " ~ n[2] ~ "]", n[4],
              "Print with the " ~ n[2] ~ " side facing UP (rig frame).");
        prints = append(prints, [n[0], n[1], n[3]]);
    }
    dress(context, sv.caseQ, "X330 case", color(0.16, 0.16, 0.18), "");
    dress(context, sv.horn, "X330 horn", color(0.3, 0.3, 0.32), "");
    dress(context, pcb, "PCB", color(0.10, 0.45, 0.20), "The force-sensor board (key-holder-v5): an envelope.");
    for (var i = 0; i < size(sensors); i += 1)
        dress(context, partAt(context, H, sensors[i]), "FS20 " ~ (i + 1), color(0.85, 0.85, 0.80),
              norm(BUTTONS[i]) < 0.001 * mm ? "The drop sensor." : "");
    for (var h in concatenateArrays([fi.hw, ri.hw]))
        dress(context, h[0], h[1], h[2], "Mock: an envelope, not a print.");
    return { "prints" : prints };
}

// ==== UI LAYER BELOW -- dropped by --check ====

export enum DropRigWheel
{
    annotation { "Name" : "Front: the fork's headset block" }
    FRONT,
    annotation { "Name" : "Rear: the chainstays' case-side joint" }
    REAR
}

annotation { "Feature Type Name" : "AOW drop rig",
             "Feature Type Description" : "Drop release rig: base, XL330 shells, cam, front and rear wheel interfaces; Z up, X along the arm" }
export const aowDropRig = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Wheel on the rig" }
        definition.wheel is DropRigWheel;
    }
    {
        dropRigBuild(context, id + "build", {
                "wheel" : definition.wheel == DropRigWheel.REAR ? "REAR" : "FRONT" });
        reportFeatureInfo(context, id, "Follower " ~ toString(roundToPrecision(RIG.Zf / millimeter, 2))
                ~ " mm over the button; cam centre " ~ toString(roundToPrecision(RIG.Zc / millimeter, 2))
                ~ "; axle " ~ toString(roundToPrecision((definition.wheel == DropRigWheel.REAR ? RIG.RaR : RIG.RaF) / millimeter, 2)));
    });
