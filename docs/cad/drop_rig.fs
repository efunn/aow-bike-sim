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
    "tipT" : 0.8 * millimeter,
    "tipW" : 1 * millimeter,
    "tipH" : 6 * millimeter,
    "tipFit" : 0.05 * millimeter,
    "noseR" : 0.45 * millimeter,
    "noseH" : 1.5 * millimeter,
    "noseBelow" : 2.3 * millimeter,
    "waveRest" : 10.7 * millimeter,
    "waveLift" : 0.7 * millimeter,
    "waveT" : 5 * millimeter,
    "waveOff" : -100 * millimeter,
    "wavePitch" : 32 * millimeter,
    "tipZl" : 27.45 * millimeter,
    "tipZb" : 26.65 * millimeter,
    "tipZt" : 33.45 * millimeter,
    "tipXd" : 82.05 * millimeter,
    "tipXi0" : 75.4293 * millimeter,
    "tipXoB" : 74.8151 * millimeter,
    "tipXoT" : 68.0151 * millimeter,
    "tipYi" : 3.3 * millimeter,
    "tipYo" : 4.3 * millimeter,
    "noseYn" : 2.6 * millimeter,
    "coverY" : 3.5 * millimeter,
    "tipCoverClear" : 0.35 * millimeter,
    "tipCamClear" : 0.756755 * millimeter,
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

export const WAVES = [
    [vector(0.0000, 11.4000) * millimeter, vector(-0.1990, 11.3983) * millimeter, vector(-0.3979, 11.3931) * millimeter, vector(-0.5966, 11.3844) * millimeter, vector(-0.7952, 11.3722) * millimeter, vector(-0.9936, 11.3566) * millimeter, vector(-1.1916, 11.3375) * millimeter, vector(-1.3893, 11.3150) * millimeter, vector(-1.5866, 11.2891) * millimeter, vector(-1.7834, 11.2596) * millimeter, vector(-1.9796, 11.2268) * millimeter, vector(-2.1752, 11.1905) * millimeter, vector(-2.3702, 11.1509) * millimeter, vector(-2.5644, 11.1078) * millimeter, vector(-2.7579, 11.0614) * millimeter, vector(-2.9505, 11.0116) * millimeter, vector(-3.1423, 10.9584) * millimeter, vector(-3.3330, 10.9019) * millimeter, vector(-3.5228, 10.8420) * millimeter, vector(-3.7115, 10.7789) * millimeter, vector(-3.8990, 10.7125) * millimeter, vector(-4.0854, 10.6428) * millimeter, vector(-4.2705, 10.5699) * millimeter, vector(-4.4543, 10.4938) * millimeter, vector(-4.6368, 10.4144) * millimeter, vector(-4.8178, 10.3319) * millimeter, vector(-4.9974, 10.2463) * millimeter, vector(-5.1755, 10.1575) * millimeter, vector(-5.3520, 10.0656) * millimeter, vector(-5.5268, 9.9707) * millimeter, vector(-5.7000, 9.8727) * millimeter, vector(-5.8714, 9.7717) * millimeter, vector(-6.0411, 9.6677) * millimeter, vector(-6.2089, 9.5608) * millimeter, vector(-6.3748, 9.4510) * millimeter, vector(-6.5388, 9.3383) * millimeter, vector(-6.7008, 9.2228) * millimeter, vector(-6.8607, 9.1044) * millimeter, vector(-7.0185, 8.9833) * millimeter, vector(-7.1743, 8.8595) * millimeter, vector(-7.3278, 8.7329) * millimeter, vector(-7.4791, 8.6037) * millimeter, vector(-7.6281, 8.4719) * millimeter, vector(-7.7748, 8.3374) * millimeter, vector(-7.9191, 8.2005) * millimeter, vector(-8.0610, 8.0610) * millimeter, vector(-8.2005, 7.9191) * millimeter, vector(-8.3374, 7.7748) * millimeter, vector(-8.4719, 7.6281) * millimeter, vector(-8.6037, 7.4791) * millimeter, vector(-8.7329, 7.3278) * millimeter, vector(-8.8595, 7.1743) * millimeter, vector(-8.9833, 7.0185) * millimeter, vector(-9.1044, 6.8607) * millimeter, vector(-9.2228, 6.7008) * millimeter, vector(-9.3383, 6.5388) * millimeter, vector(-9.4510, 6.3748) * millimeter, vector(-9.5608, 6.2089) * millimeter, vector(-9.6677, 6.0411) * millimeter, vector(-9.7717, 5.8714) * millimeter, vector(-9.8727, 5.7000) * millimeter, vector(-9.9707, 5.5268) * millimeter, vector(-10.0656, 5.3520) * millimeter, vector(-10.1575, 5.1755) * millimeter, vector(-10.2463, 4.9974) * millimeter, vector(-10.3319, 4.8178) * millimeter, vector(-10.4144, 4.6368) * millimeter, vector(-10.4938, 4.4543) * millimeter, vector(-10.5699, 4.2705) * millimeter, vector(-10.6428, 4.0854) * millimeter, vector(-10.7125, 3.8990) * millimeter, vector(-10.7789, 3.7115) * millimeter, vector(-10.8420, 3.5228) * millimeter, vector(-10.9019, 3.3330) * millimeter, vector(-10.9584, 3.1423) * millimeter, vector(-11.0116, 2.9505) * millimeter, vector(-11.0614, 2.7579) * millimeter, vector(-11.1078, 2.5644) * millimeter, vector(-11.1509, 2.3702) * millimeter, vector(-11.1905, 2.1752) * millimeter, vector(-11.2268, 1.9796) * millimeter, vector(-11.2596, 1.7834) * millimeter, vector(-11.2891, 1.5866) * millimeter, vector(-11.3150, 1.3893) * millimeter, vector(-11.3375, 1.1916) * millimeter, vector(-11.3566, 0.9936) * millimeter, vector(-11.3722, 0.7952) * millimeter, vector(-11.3844, 0.5966) * millimeter, vector(-11.3931, 0.3979) * millimeter, vector(-11.3983, 0.1990) * millimeter, vector(-11.4000, 0.0000) * millimeter, vector(-11.3983, -0.1990) * millimeter, vector(-11.3931, -0.3979) * millimeter, vector(-11.3844, -0.5966) * millimeter, vector(-11.3722, -0.7952) * millimeter, vector(-11.3566, -0.9936) * millimeter, vector(-11.3375, -1.1916) * millimeter, vector(-11.3150, -1.3893) * millimeter, vector(-11.2891, -1.5866) * millimeter, vector(-11.2596, -1.7834) * millimeter, vector(-11.2268, -1.9796) * millimeter, vector(-11.1905, -2.1752) * millimeter, vector(-11.1509, -2.3702) * millimeter, vector(-11.1078, -2.5644) * millimeter, vector(-11.0614, -2.7579) * millimeter, vector(-11.0116, -2.9505) * millimeter, vector(-10.9584, -3.1423) * millimeter, vector(-10.9019, -3.3330) * millimeter, vector(-10.8420, -3.5228) * millimeter, vector(-10.7789, -3.7115) * millimeter, vector(-10.7125, -3.8990) * millimeter, vector(-10.6428, -4.0854) * millimeter, vector(-10.5699, -4.2705) * millimeter, vector(-10.4938, -4.4543) * millimeter, vector(-10.4144, -4.6368) * millimeter, vector(-10.3319, -4.8178) * millimeter, vector(-10.2463, -4.9974) * millimeter, vector(-10.1575, -5.1755) * millimeter, vector(-10.0656, -5.3520) * millimeter, vector(-9.9707, -5.5268) * millimeter, vector(-9.8727, -5.7000) * millimeter, vector(-9.7717, -5.8714) * millimeter, vector(-9.6677, -6.0411) * millimeter, vector(-9.5608, -6.2089) * millimeter, vector(-9.4510, -6.3748) * millimeter, vector(-9.3383, -6.5388) * millimeter, vector(-9.2228, -6.7008) * millimeter, vector(-9.1044, -6.8607) * millimeter, vector(-8.9833, -7.0185) * millimeter, vector(-8.8595, -7.1743) * millimeter, vector(-8.7329, -7.3278) * millimeter, vector(-8.6037, -7.4791) * millimeter, vector(-8.4719, -7.6281) * millimeter, vector(-8.3374, -7.7748) * millimeter, vector(-8.2005, -7.9191) * millimeter, vector(-8.0610, -8.0610) * millimeter, vector(-7.9191, -8.2005) * millimeter, vector(-7.7748, -8.3374) * millimeter, vector(-7.6281, -8.4719) * millimeter, vector(-7.4791, -8.6037) * millimeter, vector(-7.3278, -8.7329) * millimeter, vector(-7.1743, -8.8595) * millimeter, vector(-7.0185, -8.9833) * millimeter, vector(-6.8607, -9.1044) * millimeter, vector(-6.7008, -9.2228) * millimeter, vector(-6.5388, -9.3383) * millimeter, vector(-6.3748, -9.4510) * millimeter, vector(-6.2089, -9.5608) * millimeter, vector(-6.0411, -9.6677) * millimeter, vector(-5.8714, -9.7717) * millimeter, vector(-5.7000, -9.8727) * millimeter, vector(-5.5268, -9.9707) * millimeter, vector(-5.3520, -10.0656) * millimeter, vector(-5.1755, -10.1575) * millimeter, vector(-4.9974, -10.2463) * millimeter, vector(-4.8178, -10.3319) * millimeter, vector(-4.6368, -10.4144) * millimeter, vector(-4.4543, -10.4938) * millimeter, vector(-4.2705, -10.5699) * millimeter, vector(-4.0854, -10.6428) * millimeter, vector(-3.8990, -10.7125) * millimeter, vector(-3.7115, -10.7789) * millimeter, vector(-3.5228, -10.8420) * millimeter, vector(-3.3330, -10.9019) * millimeter, vector(-3.1423, -10.9584) * millimeter, vector(-2.9505, -11.0116) * millimeter, vector(-2.7579, -11.0614) * millimeter, vector(-2.5644, -11.1078) * millimeter, vector(-2.3702, -11.1509) * millimeter, vector(-2.1752, -11.1905) * millimeter, vector(-1.9796, -11.2268) * millimeter, vector(-1.7834, -11.2596) * millimeter, vector(-1.5866, -11.2891) * millimeter, vector(-1.3893, -11.3150) * millimeter, vector(-1.1916, -11.3375) * millimeter, vector(-0.9936, -11.3566) * millimeter, vector(-0.7952, -11.3722) * millimeter, vector(-0.5966, -11.3844) * millimeter, vector(-0.3979, -11.3931) * millimeter, vector(-0.1990, -11.3983) * millimeter, vector(-0.0000, -11.4000) * millimeter, vector(0.1990, -11.3983) * millimeter, vector(0.3979, -11.3931) * millimeter, vector(0.5966, -11.3844) * millimeter, vector(0.7952, -11.3722) * millimeter, vector(0.9936, -11.3566) * millimeter, vector(1.1916, -11.3375) * millimeter, vector(1.3893, -11.3150) * millimeter, vector(1.5866, -11.2891) * millimeter, vector(1.7834, -11.2596) * millimeter, vector(1.9796, -11.2268) * millimeter, vector(2.1752, -11.1905) * millimeter, vector(2.3702, -11.1509) * millimeter, vector(2.5644, -11.1078) * millimeter, vector(2.7579, -11.0614) * millimeter, vector(2.9505, -11.0116) * millimeter, vector(3.1423, -10.9584) * millimeter, vector(3.3330, -10.9019) * millimeter, vector(3.5228, -10.8420) * millimeter, vector(3.7115, -10.7789) * millimeter, vector(3.8990, -10.7125) * millimeter, vector(4.0854, -10.6428) * millimeter, vector(4.2705, -10.5699) * millimeter, vector(4.4543, -10.4938) * millimeter, vector(4.6368, -10.4144) * millimeter, vector(4.8178, -10.3319) * millimeter, vector(4.9974, -10.2463) * millimeter, vector(5.1755, -10.1575) * millimeter, vector(5.3520, -10.0656) * millimeter, vector(5.5268, -9.9707) * millimeter, vector(5.7000, -9.8727) * millimeter, vector(5.8714, -9.7717) * millimeter, vector(6.0411, -9.6677) * millimeter, vector(6.2089, -9.5608) * millimeter, vector(6.3748, -9.4510) * millimeter, vector(6.5388, -9.3383) * millimeter, vector(6.7008, -9.2228) * millimeter, vector(6.8607, -9.1044) * millimeter, vector(7.0185, -8.9833) * millimeter, vector(7.1743, -8.8595) * millimeter, vector(7.3278, -8.7329) * millimeter, vector(7.4791, -8.6037) * millimeter, vector(7.6281, -8.4719) * millimeter, vector(7.7748, -8.3374) * millimeter, vector(7.9191, -8.2005) * millimeter, vector(8.0610, -8.0610) * millimeter, vector(8.2005, -7.9191) * millimeter, vector(8.3374, -7.7748) * millimeter, vector(8.4719, -7.6281) * millimeter, vector(8.6037, -7.4791) * millimeter, vector(8.7329, -7.3278) * millimeter, vector(8.8595, -7.1743) * millimeter, vector(8.9833, -7.0185) * millimeter, vector(9.1044, -6.8607) * millimeter, vector(9.2228, -6.7008) * millimeter, vector(9.3383, -6.5388) * millimeter, vector(9.4510, -6.3748) * millimeter, vector(9.5608, -6.2089) * millimeter, vector(9.6677, -6.0411) * millimeter, vector(9.7717, -5.8714) * millimeter, vector(9.8727, -5.7000) * millimeter, vector(9.9707, -5.5268) * millimeter, vector(10.0656, -5.3520) * millimeter, vector(10.1575, -5.1755) * millimeter, vector(10.2463, -4.9974) * millimeter, vector(10.3319, -4.8178) * millimeter, vector(10.4144, -4.6368) * millimeter, vector(10.4938, -4.4543) * millimeter, vector(10.5699, -4.2705) * millimeter, vector(10.6428, -4.0854) * millimeter, vector(10.7125, -3.8990) * millimeter, vector(10.7789, -3.7115) * millimeter, vector(10.8420, -3.5228) * millimeter, vector(10.9019, -3.3330) * millimeter, vector(10.9584, -3.1423) * millimeter, vector(11.0116, -2.9505) * millimeter, vector(11.0614, -2.7579) * millimeter, vector(11.1078, -2.5644) * millimeter, vector(11.1509, -2.3702) * millimeter, vector(11.1905, -2.1752) * millimeter, vector(11.2268, -1.9796) * millimeter, vector(11.2596, -1.7834) * millimeter, vector(11.2891, -1.5866) * millimeter, vector(11.3150, -1.3893) * millimeter, vector(11.3375, -1.1916) * millimeter, vector(11.3566, -0.9936) * millimeter, vector(11.3722, -0.7952) * millimeter, vector(11.3844, -0.5966) * millimeter, vector(11.3931, -0.3979) * millimeter, vector(11.3983, -0.1990) * millimeter, vector(11.4000, -0.0000) * millimeter, vector(11.3983, 0.1990) * millimeter, vector(11.3931, 0.3979) * millimeter, vector(11.3844, 0.5966) * millimeter, vector(11.3722, 0.7952) * millimeter, vector(11.3566, 0.9936) * millimeter, vector(11.3375, 1.1916) * millimeter, vector(11.3150, 1.3893) * millimeter, vector(11.2891, 1.5866) * millimeter, vector(11.2596, 1.7834) * millimeter, vector(11.2268, 1.9796) * millimeter, vector(11.1905, 2.1752) * millimeter, vector(11.1509, 2.3702) * millimeter, vector(11.1078, 2.5644) * millimeter, vector(11.0614, 2.7579) * millimeter, vector(11.0116, 2.9505) * millimeter, vector(10.9584, 3.1423) * millimeter, vector(10.9019, 3.3330) * millimeter, vector(10.8420, 3.5228) * millimeter, vector(10.7789, 3.7115) * millimeter, vector(10.7125, 3.8990) * millimeter, vector(10.6428, 4.0854) * millimeter, vector(10.5699, 4.2705) * millimeter, vector(10.4938, 4.4543) * millimeter, vector(10.4144, 4.6368) * millimeter, vector(10.3319, 4.8178) * millimeter, vector(10.2463, 4.9974) * millimeter, vector(10.1575, 5.1755) * millimeter, vector(10.0656, 5.3520) * millimeter, vector(9.9707, 5.5268) * millimeter, vector(9.8727, 5.7000) * millimeter, vector(9.7717, 5.8714) * millimeter, vector(9.6677, 6.0411) * millimeter, vector(9.5608, 6.2089) * millimeter, vector(9.4510, 6.3748) * millimeter, vector(9.3383, 6.5388) * millimeter, vector(9.2228, 6.7008) * millimeter, vector(9.1044, 6.8607) * millimeter, vector(8.9833, 7.0185) * millimeter, vector(8.8595, 7.1743) * millimeter, vector(8.7329, 7.3278) * millimeter, vector(8.6037, 7.4791) * millimeter, vector(8.4719, 7.6281) * millimeter, vector(8.3374, 7.7748) * millimeter, vector(8.2005, 7.9191) * millimeter, vector(8.0610, 8.0610) * millimeter, vector(7.9191, 8.2005) * millimeter, vector(7.7748, 8.3374) * millimeter, vector(7.6281, 8.4719) * millimeter, vector(7.4791, 8.6037) * millimeter, vector(7.3278, 8.7329) * millimeter, vector(7.1743, 8.8595) * millimeter, vector(7.0185, 8.9833) * millimeter, vector(6.8607, 9.1044) * millimeter, vector(6.7008, 9.2228) * millimeter, vector(6.5388, 9.3383) * millimeter, vector(6.3748, 9.4510) * millimeter, vector(6.2089, 9.5608) * millimeter, vector(6.0411, 9.6677) * millimeter, vector(5.8714, 9.7717) * millimeter, vector(5.7000, 9.8727) * millimeter, vector(5.5268, 9.9707) * millimeter, vector(5.3520, 10.0656) * millimeter, vector(5.1755, 10.1575) * millimeter, vector(4.9974, 10.2463) * millimeter, vector(4.8178, 10.3319) * millimeter, vector(4.6368, 10.4144) * millimeter, vector(4.4543, 10.4938) * millimeter, vector(4.2705, 10.5699) * millimeter, vector(4.0854, 10.6428) * millimeter, vector(3.8990, 10.7125) * millimeter, vector(3.7115, 10.7789) * millimeter, vector(3.5228, 10.8420) * millimeter, vector(3.3330, 10.9019) * millimeter, vector(3.1423, 10.9584) * millimeter, vector(2.9505, 11.0116) * millimeter, vector(2.7579, 11.0614) * millimeter, vector(2.5644, 11.1078) * millimeter, vector(2.3702, 11.1509) * millimeter, vector(2.1752, 11.1905) * millimeter, vector(1.9796, 11.2268) * millimeter, vector(1.7834, 11.2596) * millimeter, vector(1.5866, 11.2891) * millimeter, vector(1.3893, 11.3150) * millimeter, vector(1.1916, 11.3375) * millimeter, vector(0.9936, 11.3566) * millimeter, vector(0.7952, 11.3722) * millimeter, vector(0.5966, 11.3844) * millimeter, vector(0.3979, 11.3931) * millimeter, vector(0.1990, 11.3983) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.2000, 11.3985) * millimeter, vector(-0.3999, 11.3939) * millimeter, vector(-0.5997, 11.3863) * millimeter, vector(-0.7994, 11.3756) * millimeter, vector(-0.9990, 11.3619) * millimeter, vector(-1.1984, 11.3451) * millimeter, vector(-1.3976, 11.3253) * millimeter, vector(-1.5965, 11.3025) * millimeter, vector(-1.7950, 11.2766) * millimeter, vector(-1.9933, 11.2476) * millimeter, vector(-2.1911, 11.2157) * millimeter, vector(-2.3886, 11.1806) * millimeter, vector(-2.5856, 11.1426) * millimeter, vector(-2.7821, 11.1015) * millimeter, vector(-2.9780, 11.0574) * millimeter, vector(-3.1734, 11.0102) * millimeter, vector(-3.3681, 10.9600) * millimeter, vector(-3.5622, 10.9068) * millimeter, vector(-3.7556, 10.8505) * millimeter, vector(-3.9483, 10.7913) * millimeter, vector(-4.1401, 10.7290) * millimeter, vector(-4.3312, 10.6637) * millimeter, vector(-4.5214, 10.5954) * millimeter, vector(-4.7107, 10.5241) * millimeter, vector(-4.8990, 10.4498) * millimeter, vector(-5.0864, 10.3725) * millimeter, vector(-5.2727, 10.2922) * millimeter, vector(-5.4580, 10.2090) * millimeter, vector(-5.6421, 10.1227) * millimeter, vector(-5.8251, 10.0335) * millimeter, vector(-6.0069, 9.9413) * millimeter, vector(-6.1874, 9.8462) * millimeter, vector(-6.3667, 9.7481) * millimeter, vector(-6.5446, 9.6471) * millimeter, vector(-6.7211, 9.5432) * millimeter, vector(-6.8962, 9.4363) * millimeter, vector(-7.0698, 9.3266) * millimeter, vector(-7.2419, 9.2139) * millimeter, vector(-7.4125, 9.0984) * millimeter, vector(-7.5814, 8.9800) * millimeter, vector(-7.7487, 8.8588) * millimeter, vector(-7.9142, 8.7347) * millimeter, vector(-8.0781, 8.6078) * millimeter, vector(-8.2401, 8.4780) * millimeter, vector(-8.4002, 8.3455) * millimeter, vector(-8.5585, 8.2102) * millimeter, vector(-8.7148, 8.0721) * millimeter, vector(-8.8691, 7.9313) * millimeter, vector(-9.0214, 7.7878) * millimeter, vector(-9.1716, 7.6416) * millimeter, vector(-9.3196, 7.4927) * millimeter, vector(-9.4654, 7.3411) * millimeter, vector(-9.6090, 7.1870) * millimeter, vector(-9.7504, 7.0302) * millimeter, vector(-9.8893, 6.8708) * millimeter, vector(-10.0259, 6.7089) * millimeter, vector(-10.1601, 6.5444) * millimeter, vector(-10.2917, 6.3775) * millimeter, vector(-10.4208, 6.2081) * millimeter, vector(-10.5473, 6.0362) * millimeter, vector(-10.6712, 5.8620) * millimeter, vector(-10.7924, 5.6853) * millimeter, vector(-10.9109, 5.5064) * millimeter, vector(-11.0265, 5.3251) * millimeter, vector(-11.1394, 5.1416) * millimeter, vector(-11.2493, 4.9558) * millimeter, vector(-11.3563, 4.7679) * millimeter, vector(-11.4603, 4.5778) * millimeter, vector(-11.5613, 4.3856) * millimeter, vector(-11.6593, 4.1914) * millimeter, vector(-11.7541, 3.9951) * millimeter, vector(-11.8457, 3.7968) * millimeter, vector(-11.9341, 3.5967) * millimeter, vector(-12.0192, 3.3946) * millimeter, vector(-12.1011, 3.1907) * millimeter, vector(-12.1796, 2.9851) * millimeter, vector(-12.2546, 2.7777) * millimeter, vector(-12.3263, 2.5686) * millimeter, vector(-12.3944, 2.3579) * millimeter, vector(-12.4591, 2.1456) * millimeter, vector(-12.5201, 1.9319) * millimeter, vector(-12.5776, 1.7166) * millimeter, vector(-12.6314, 1.5000) * millimeter, vector(-12.6815, 1.2820) * millimeter, vector(-12.7279, 1.0628) * millimeter, vector(-12.7706, 0.8424) * millimeter, vector(-12.8094, 0.6208) * millimeter, vector(-12.8444, 0.3981) * millimeter, vector(-12.8756, 0.1744) * millimeter, vector(-12.9028, -0.0502) * millimeter, vector(-12.9261, -0.2758) * millimeter, vector(-12.9455, -0.5021) * millimeter, vector(-12.9609, -0.7292) * millimeter, vector(-12.9722, -0.9570) * millimeter, vector(-12.9796, -1.1853) * millimeter, vector(-12.9828, -1.4142) * millimeter, vector(-12.9820, -1.6436) * millimeter, vector(-12.9770, -1.8733) * millimeter, vector(-12.9679, -2.1033) * millimeter, vector(-12.9547, -2.3336) * millimeter, vector(-12.9373, -2.5640) * millimeter, vector(-12.9157, -2.7944) * millimeter, vector(-12.8899, -3.0249) * millimeter, vector(-12.8599, -3.2553) * millimeter, vector(-12.8256, -3.4855) * millimeter, vector(-12.7871, -3.7154) * millimeter, vector(-12.7444, -3.9451) * millimeter, vector(-12.6974, -4.1743) * millimeter, vector(-12.6462, -4.4030) * millimeter, vector(-12.5907, -4.6311) * millimeter, vector(-12.5310, -4.8586) * millimeter, vector(-12.4670, -5.0853) * millimeter, vector(-12.3987, -5.3111) * millimeter, vector(-12.3262, -5.5361) * millimeter, vector(-12.2494, -5.7600) * millimeter, vector(-12.1684, -5.9829) * millimeter, vector(-12.0832, -6.2046) * millimeter, vector(-11.9937, -6.4250) * millimeter, vector(-11.9001, -6.6441) * millimeter, vector(-11.8022, -6.8617) * millimeter, vector(-11.7002, -7.0778) * millimeter, vector(-11.5941, -7.2923) * millimeter, vector(-11.4838, -7.5051) * millimeter, vector(-11.3694, -7.7162) * millimeter, vector(-11.2510, -7.9253) * millimeter, vector(-11.1285, -8.1326) * millimeter, vector(-11.0019, -8.3377) * millimeter, vector(-10.8714, -8.5408) * millimeter, vector(-10.7369, -8.7417) * millimeter, vector(-10.5985, -8.9402) * millimeter, vector(-10.4563, -9.1364) * millimeter, vector(-10.3101, -9.3302) * millimeter, vector(-10.1602, -9.5214) * millimeter, vector(-10.0066, -9.7100) * millimeter, vector(-9.8492, -9.8959) * millimeter, vector(-9.6881, -10.0790) * millimeter, vector(-9.5235, -10.2593) * millimeter, vector(-9.3552, -10.4366) * millimeter, vector(-9.1835, -10.6109) * millimeter, vector(-9.0083, -10.7822) * millimeter, vector(-8.8297, -10.9502) * millimeter, vector(-8.6478, -11.1151) * millimeter, vector(-8.4626, -11.2766) * millimeter, vector(-8.2742, -11.4347) * millimeter, vector(-8.0826, -11.5894) * millimeter, vector(-7.8880, -11.7406) * millimeter, vector(-7.6903, -11.8882) * millimeter, vector(-7.4897, -12.0321) * millimeter, vector(-7.2862, -12.1723) * millimeter, vector(-7.0799, -12.3087) * millimeter, vector(-6.8709, -12.4413) * millimeter, vector(-6.6592, -12.5700) * millimeter, vector(-6.4449, -12.6947) * millimeter, vector(-6.2281, -12.8154) * millimeter, vector(-6.0089, -12.9320) * millimeter, vector(-5.7874, -13.0445) * millimeter, vector(-5.5636, -13.1528) * millimeter, vector(-5.3376, -13.2569) * millimeter, vector(-5.1096, -13.3567) * millimeter, vector(-4.8796, -13.4522) * millimeter, vector(-4.6476, -13.5433) * millimeter, vector(-4.4138, -13.6300) * millimeter, vector(-4.1783, -13.7122) * millimeter, vector(-3.9411, -13.7899) * millimeter, vector(-3.7024, -13.8632) * millimeter, vector(-3.4622, -13.9318) * millimeter, vector(-3.2207, -13.9959) * millimeter, vector(-2.9779, -14.0553) * millimeter, vector(-2.7339, -14.1101) * millimeter, vector(-2.4888, -14.1602) * millimeter, vector(-2.2427, -14.2056) * millimeter, vector(-1.9958, -14.2463) * millimeter, vector(-1.7481, -14.2823) * millimeter, vector(-1.4996, -14.3135) * millimeter, vector(-1.2506, -14.3399) * millimeter, vector(-1.0011, -14.3615) * millimeter, vector(-0.7512, -14.3783) * millimeter, vector(-0.5009, -14.3904) * millimeter, vector(-0.2505, -14.3976) * millimeter, vector(-0.0000, -14.4000) * millimeter, vector(0.2505, -14.3976) * millimeter, vector(0.5009, -14.3904) * millimeter, vector(0.7512, -14.3783) * millimeter, vector(1.0011, -14.3615) * millimeter, vector(1.2506, -14.3399) * millimeter, vector(1.4996, -14.3135) * millimeter, vector(1.7481, -14.2823) * millimeter, vector(1.9958, -14.2463) * millimeter, vector(2.2427, -14.2056) * millimeter, vector(2.4888, -14.1602) * millimeter, vector(2.7339, -14.1101) * millimeter, vector(2.9779, -14.0553) * millimeter, vector(3.2207, -13.9959) * millimeter, vector(3.4622, -13.9318) * millimeter, vector(3.7024, -13.8632) * millimeter, vector(3.9411, -13.7899) * millimeter, vector(4.1783, -13.7122) * millimeter, vector(4.4138, -13.6300) * millimeter, vector(4.6476, -13.5433) * millimeter, vector(4.8796, -13.4522) * millimeter, vector(5.1096, -13.3567) * millimeter, vector(5.3376, -13.2569) * millimeter, vector(5.5636, -13.1528) * millimeter, vector(5.7874, -13.0445) * millimeter, vector(6.0089, -12.9320) * millimeter, vector(6.2281, -12.8154) * millimeter, vector(6.4449, -12.6947) * millimeter, vector(6.6592, -12.5700) * millimeter, vector(6.8709, -12.4413) * millimeter, vector(7.0799, -12.3087) * millimeter, vector(7.2862, -12.1723) * millimeter, vector(7.4897, -12.0321) * millimeter, vector(7.6903, -11.8882) * millimeter, vector(7.8880, -11.7406) * millimeter, vector(8.0826, -11.5894) * millimeter, vector(8.2742, -11.4347) * millimeter, vector(8.4626, -11.2766) * millimeter, vector(8.6478, -11.1151) * millimeter, vector(8.8297, -10.9502) * millimeter, vector(9.0083, -10.7822) * millimeter, vector(9.1835, -10.6109) * millimeter, vector(9.3552, -10.4366) * millimeter, vector(9.5235, -10.2593) * millimeter, vector(9.6881, -10.0790) * millimeter, vector(9.8492, -9.8959) * millimeter, vector(10.0066, -9.7100) * millimeter, vector(10.1602, -9.5214) * millimeter, vector(10.3101, -9.3302) * millimeter, vector(10.4563, -9.1364) * millimeter, vector(10.5985, -8.9402) * millimeter, vector(10.7369, -8.7417) * millimeter, vector(10.8714, -8.5408) * millimeter, vector(11.0019, -8.3377) * millimeter, vector(11.1285, -8.1326) * millimeter, vector(11.2510, -7.9253) * millimeter, vector(11.3694, -7.7162) * millimeter, vector(11.4838, -7.5051) * millimeter, vector(11.5941, -7.2923) * millimeter, vector(11.7002, -7.0778) * millimeter, vector(11.8022, -6.8617) * millimeter, vector(11.9001, -6.6441) * millimeter, vector(11.9937, -6.4250) * millimeter, vector(12.0832, -6.2046) * millimeter, vector(12.1684, -5.9829) * millimeter, vector(12.2494, -5.7600) * millimeter, vector(12.3262, -5.5361) * millimeter, vector(12.3987, -5.3111) * millimeter, vector(12.4670, -5.0853) * millimeter, vector(12.5310, -4.8586) * millimeter, vector(12.5907, -4.6311) * millimeter, vector(12.6462, -4.4030) * millimeter, vector(12.6974, -4.1743) * millimeter, vector(12.7444, -3.9451) * millimeter, vector(12.7871, -3.7154) * millimeter, vector(12.8256, -3.4855) * millimeter, vector(12.8599, -3.2553) * millimeter, vector(12.8899, -3.0249) * millimeter, vector(12.9157, -2.7944) * millimeter, vector(12.9373, -2.5640) * millimeter, vector(12.9547, -2.3336) * millimeter, vector(12.9679, -2.1033) * millimeter, vector(12.9770, -1.8733) * millimeter, vector(12.9820, -1.6436) * millimeter, vector(12.9828, -1.4142) * millimeter, vector(12.9796, -1.1853) * millimeter, vector(12.9722, -0.9570) * millimeter, vector(12.9609, -0.7292) * millimeter, vector(12.9455, -0.5021) * millimeter, vector(12.9261, -0.2758) * millimeter, vector(12.9028, -0.0502) * millimeter, vector(12.8756, 0.1744) * millimeter, vector(12.8444, 0.3981) * millimeter, vector(12.8094, 0.6208) * millimeter, vector(12.7706, 0.8424) * millimeter, vector(12.7279, 1.0628) * millimeter, vector(12.6815, 1.2820) * millimeter, vector(12.6314, 1.5000) * millimeter, vector(12.5776, 1.7166) * millimeter, vector(12.5201, 1.9319) * millimeter, vector(12.4591, 2.1456) * millimeter, vector(12.3944, 2.3579) * millimeter, vector(12.3263, 2.5686) * millimeter, vector(12.2546, 2.7777) * millimeter, vector(12.1796, 2.9851) * millimeter, vector(12.1011, 3.1907) * millimeter, vector(12.0192, 3.3946) * millimeter, vector(11.9341, 3.5967) * millimeter, vector(11.8457, 3.7968) * millimeter, vector(11.7541, 3.9951) * millimeter, vector(11.6593, 4.1914) * millimeter, vector(11.5613, 4.3856) * millimeter, vector(11.4603, 4.5778) * millimeter, vector(11.3563, 4.7679) * millimeter, vector(11.2493, 4.9558) * millimeter, vector(11.1394, 5.1416) * millimeter, vector(11.0265, 5.3251) * millimeter, vector(10.9109, 5.5064) * millimeter, vector(10.7924, 5.6853) * millimeter, vector(10.6712, 5.8620) * millimeter, vector(10.5473, 6.0362) * millimeter, vector(10.4208, 6.2081) * millimeter, vector(10.2917, 6.3775) * millimeter, vector(10.1601, 6.5444) * millimeter, vector(10.0259, 6.7089) * millimeter, vector(9.8893, 6.8708) * millimeter, vector(9.7504, 7.0302) * millimeter, vector(9.6090, 7.1870) * millimeter, vector(9.4654, 7.3411) * millimeter, vector(9.3196, 7.4927) * millimeter, vector(9.1716, 7.6416) * millimeter, vector(9.0214, 7.7878) * millimeter, vector(8.8691, 7.9313) * millimeter, vector(8.7148, 8.0721) * millimeter, vector(8.5585, 8.2102) * millimeter, vector(8.4002, 8.3455) * millimeter, vector(8.2401, 8.4780) * millimeter, vector(8.0781, 8.6078) * millimeter, vector(7.9142, 8.7347) * millimeter, vector(7.7487, 8.8588) * millimeter, vector(7.5814, 8.9800) * millimeter, vector(7.4125, 9.0984) * millimeter, vector(7.2419, 9.2139) * millimeter, vector(7.0698, 9.3266) * millimeter, vector(6.8962, 9.4363) * millimeter, vector(6.7211, 9.5432) * millimeter, vector(6.5446, 9.6471) * millimeter, vector(6.3667, 9.7481) * millimeter, vector(6.1874, 9.8462) * millimeter, vector(6.0069, 9.9413) * millimeter, vector(5.8251, 10.0335) * millimeter, vector(5.6421, 10.1227) * millimeter, vector(5.4580, 10.2090) * millimeter, vector(5.2727, 10.2922) * millimeter, vector(5.0864, 10.3725) * millimeter, vector(4.8990, 10.4498) * millimeter, vector(4.7107, 10.5241) * millimeter, vector(4.5214, 10.5954) * millimeter, vector(4.3312, 10.6637) * millimeter, vector(4.1401, 10.7290) * millimeter, vector(3.9483, 10.7913) * millimeter, vector(3.7556, 10.8505) * millimeter, vector(3.5622, 10.9068) * millimeter, vector(3.3681, 10.9600) * millimeter, vector(3.1734, 11.0102) * millimeter, vector(2.9780, 11.0574) * millimeter, vector(2.7821, 11.1015) * millimeter, vector(2.5856, 11.1426) * millimeter, vector(2.3886, 11.1806) * millimeter, vector(2.1911, 11.2157) * millimeter, vector(1.9933, 11.2476) * millimeter, vector(1.7950, 11.2766) * millimeter, vector(1.5965, 11.3025) * millimeter, vector(1.3976, 11.3253) * millimeter, vector(1.1984, 11.3451) * millimeter, vector(0.9990, 11.3619) * millimeter, vector(0.7994, 11.3756) * millimeter, vector(0.5997, 11.3863) * millimeter, vector(0.3999, 11.3939) * millimeter, vector(0.2000, 11.3985) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.2117, 11.4011) * millimeter, vector(-0.4236, 11.4046) * millimeter, vector(-0.6357, 11.4102) * millimeter, vector(-0.8483, 11.4179) * millimeter, vector(-1.0614, 11.4276) * millimeter, vector(-1.2752, 11.4390) * millimeter, vector(-1.4899, 11.4520) * millimeter, vector(-1.7055, 11.4663) * millimeter, vector(-1.9221, 11.4816) * millimeter, vector(-2.1400, 11.4977) * millimeter, vector(-2.3591, 11.5142) * millimeter, vector(-2.5796, 11.5306) * millimeter, vector(-2.8014, 11.5468) * millimeter, vector(-3.0247, 11.5623) * millimeter, vector(-3.2495, 11.5766) * millimeter, vector(-3.4757, 11.5894) * millimeter, vector(-3.7033, 11.6003) * millimeter, vector(-3.9322, 11.6088) * millimeter, vector(-4.1624, 11.6146) * millimeter, vector(-4.3937, 11.6172) * millimeter, vector(-4.6260, 11.6162) * millimeter, vector(-4.8591, 11.6111) * millimeter, vector(-5.0928, 11.6017) * millimeter, vector(-5.3269, 11.5875) * millimeter, vector(-5.5611, 11.5682) * millimeter, vector(-5.7951, 11.5433) * millimeter, vector(-6.0286, 11.5127) * millimeter, vector(-6.2614, 11.4759) * millimeter, vector(-6.4931, 11.4327) * millimeter, vector(-6.7233, 11.3829) * millimeter, vector(-6.9517, 11.3262) * millimeter, vector(-7.1779, 11.2625) * millimeter, vector(-7.4016, 11.1915) * millimeter, vector(-7.6224, 11.1131) * millimeter, vector(-7.8399, 11.0273) * millimeter, vector(-8.0537, 10.9340) * millimeter, vector(-8.2636, 10.8331) * millimeter, vector(-8.4690, 10.7246) * millimeter, vector(-8.6698, 10.6086) * millimeter, vector(-8.8655, 10.4850) * millimeter, vector(-9.0559, 10.3540) * millimeter, vector(-9.2406, 10.2157) * millimeter, vector(-9.4194, 10.0701) * millimeter, vector(-9.5919, 9.9175) * millimeter, vector(-9.7581, 9.7581) * millimeter, vector(-9.9175, 9.5919) * millimeter, vector(-10.0701, 9.4194) * millimeter, vector(-10.2157, 9.2406) * millimeter, vector(-10.3540, 9.0559) * millimeter, vector(-10.4850, 8.8655) * millimeter, vector(-10.6086, 8.6698) * millimeter, vector(-10.7246, 8.4690) * millimeter, vector(-10.8331, 8.2636) * millimeter, vector(-10.9340, 8.0537) * millimeter, vector(-11.0273, 7.8399) * millimeter, vector(-11.1131, 7.6224) * millimeter, vector(-11.1915, 7.4016) * millimeter, vector(-11.2625, 7.1779) * millimeter, vector(-11.3262, 6.9517) * millimeter, vector(-11.3829, 6.7233) * millimeter, vector(-11.4327, 6.4931) * millimeter, vector(-11.4759, 6.2614) * millimeter, vector(-11.5127, 6.0286) * millimeter, vector(-11.5433, 5.7951) * millimeter, vector(-11.5682, 5.5611) * millimeter, vector(-11.5875, 5.3269) * millimeter, vector(-11.6017, 5.0928) * millimeter, vector(-11.6111, 4.8591) * millimeter, vector(-11.6162, 4.6260) * millimeter, vector(-11.6172, 4.3937) * millimeter, vector(-11.6146, 4.1624) * millimeter, vector(-11.6088, 3.9322) * millimeter, vector(-11.6003, 3.7033) * millimeter, vector(-11.5894, 3.4757) * millimeter, vector(-11.5766, 3.2495) * millimeter, vector(-11.5623, 3.0247) * millimeter, vector(-11.5468, 2.8014) * millimeter, vector(-11.5306, 2.5796) * millimeter, vector(-11.5142, 2.3591) * millimeter, vector(-11.4977, 2.1400) * millimeter, vector(-11.4816, 1.9221) * millimeter, vector(-11.4663, 1.7055) * millimeter, vector(-11.4520, 1.4899) * millimeter, vector(-11.4390, 1.2752) * millimeter, vector(-11.4276, 1.0614) * millimeter, vector(-11.4179, 0.8483) * millimeter, vector(-11.4102, 0.6357) * millimeter, vector(-11.4046, 0.4236) * millimeter, vector(-11.4011, 0.2117) * millimeter, vector(-11.4000, 0.0000) * millimeter, vector(-11.4011, -0.2117) * millimeter, vector(-11.4046, -0.4236) * millimeter, vector(-11.4102, -0.6357) * millimeter, vector(-11.4179, -0.8483) * millimeter, vector(-11.4276, -1.0614) * millimeter, vector(-11.4390, -1.2752) * millimeter, vector(-11.4520, -1.4899) * millimeter, vector(-11.4663, -1.7055) * millimeter, vector(-11.4816, -1.9221) * millimeter, vector(-11.4977, -2.1400) * millimeter, vector(-11.5142, -2.3591) * millimeter, vector(-11.5306, -2.5796) * millimeter, vector(-11.5468, -2.8014) * millimeter, vector(-11.5623, -3.0247) * millimeter, vector(-11.5766, -3.2495) * millimeter, vector(-11.5894, -3.4757) * millimeter, vector(-11.6003, -3.7033) * millimeter, vector(-11.6088, -3.9322) * millimeter, vector(-11.6146, -4.1624) * millimeter, vector(-11.6172, -4.3937) * millimeter, vector(-11.6162, -4.6260) * millimeter, vector(-11.6111, -4.8591) * millimeter, vector(-11.6017, -5.0928) * millimeter, vector(-11.5875, -5.3269) * millimeter, vector(-11.5682, -5.5611) * millimeter, vector(-11.5433, -5.7951) * millimeter, vector(-11.5127, -6.0286) * millimeter, vector(-11.4759, -6.2614) * millimeter, vector(-11.4327, -6.4931) * millimeter, vector(-11.3829, -6.7233) * millimeter, vector(-11.3262, -6.9517) * millimeter, vector(-11.2625, -7.1779) * millimeter, vector(-11.1915, -7.4016) * millimeter, vector(-11.1131, -7.6224) * millimeter, vector(-11.0273, -7.8399) * millimeter, vector(-10.9340, -8.0537) * millimeter, vector(-10.8331, -8.2636) * millimeter, vector(-10.7246, -8.4690) * millimeter, vector(-10.6086, -8.6698) * millimeter, vector(-10.4850, -8.8655) * millimeter, vector(-10.3540, -9.0559) * millimeter, vector(-10.2157, -9.2406) * millimeter, vector(-10.0701, -9.4194) * millimeter, vector(-9.9175, -9.5919) * millimeter, vector(-9.7581, -9.7581) * millimeter, vector(-9.5919, -9.9175) * millimeter, vector(-9.4194, -10.0701) * millimeter, vector(-9.2406, -10.2157) * millimeter, vector(-9.0559, -10.3540) * millimeter, vector(-8.8655, -10.4850) * millimeter, vector(-8.6698, -10.6086) * millimeter, vector(-8.4690, -10.7246) * millimeter, vector(-8.2636, -10.8331) * millimeter, vector(-8.0537, -10.9340) * millimeter, vector(-7.8399, -11.0273) * millimeter, vector(-7.6224, -11.1131) * millimeter, vector(-7.4016, -11.1915) * millimeter, vector(-7.1779, -11.2625) * millimeter, vector(-6.9517, -11.3262) * millimeter, vector(-6.7233, -11.3829) * millimeter, vector(-6.4931, -11.4327) * millimeter, vector(-6.2614, -11.4759) * millimeter, vector(-6.0286, -11.5127) * millimeter, vector(-5.7951, -11.5433) * millimeter, vector(-5.5611, -11.5682) * millimeter, vector(-5.3269, -11.5875) * millimeter, vector(-5.0928, -11.6017) * millimeter, vector(-4.8591, -11.6111) * millimeter, vector(-4.6260, -11.6162) * millimeter, vector(-4.3937, -11.6172) * millimeter, vector(-4.1624, -11.6146) * millimeter, vector(-3.9322, -11.6088) * millimeter, vector(-3.7033, -11.6003) * millimeter, vector(-3.4757, -11.5894) * millimeter, vector(-3.2495, -11.5766) * millimeter, vector(-3.0247, -11.5623) * millimeter, vector(-2.8014, -11.5468) * millimeter, vector(-2.5796, -11.5306) * millimeter, vector(-2.3591, -11.5142) * millimeter, vector(-2.1400, -11.4977) * millimeter, vector(-1.9221, -11.4816) * millimeter, vector(-1.7055, -11.4663) * millimeter, vector(-1.4899, -11.4520) * millimeter, vector(-1.2752, -11.4390) * millimeter, vector(-1.0614, -11.4276) * millimeter, vector(-0.8483, -11.4179) * millimeter, vector(-0.6357, -11.4102) * millimeter, vector(-0.4236, -11.4046) * millimeter, vector(-0.2117, -11.4011) * millimeter, vector(-0.0000, -11.4000) * millimeter, vector(0.2117, -11.4011) * millimeter, vector(0.4236, -11.4046) * millimeter, vector(0.6357, -11.4102) * millimeter, vector(0.8483, -11.4179) * millimeter, vector(1.0614, -11.4276) * millimeter, vector(1.2752, -11.4390) * millimeter, vector(1.4899, -11.4520) * millimeter, vector(1.7055, -11.4663) * millimeter, vector(1.9221, -11.4816) * millimeter, vector(2.1400, -11.4977) * millimeter, vector(2.3591, -11.5142) * millimeter, vector(2.5796, -11.5306) * millimeter, vector(2.8014, -11.5468) * millimeter, vector(3.0247, -11.5623) * millimeter, vector(3.2495, -11.5766) * millimeter, vector(3.4757, -11.5894) * millimeter, vector(3.7033, -11.6003) * millimeter, vector(3.9322, -11.6088) * millimeter, vector(4.1624, -11.6146) * millimeter, vector(4.3937, -11.6172) * millimeter, vector(4.6260, -11.6162) * millimeter, vector(4.8591, -11.6111) * millimeter, vector(5.0928, -11.6017) * millimeter, vector(5.3269, -11.5875) * millimeter, vector(5.5611, -11.5682) * millimeter, vector(5.7951, -11.5433) * millimeter, vector(6.0286, -11.5127) * millimeter, vector(6.2614, -11.4759) * millimeter, vector(6.4931, -11.4327) * millimeter, vector(6.7233, -11.3829) * millimeter, vector(6.9517, -11.3262) * millimeter, vector(7.1779, -11.2625) * millimeter, vector(7.4016, -11.1915) * millimeter, vector(7.6224, -11.1131) * millimeter, vector(7.8399, -11.0273) * millimeter, vector(8.0537, -10.9340) * millimeter, vector(8.2636, -10.8331) * millimeter, vector(8.4690, -10.7246) * millimeter, vector(8.6698, -10.6086) * millimeter, vector(8.8655, -10.4850) * millimeter, vector(9.0559, -10.3540) * millimeter, vector(9.2406, -10.2157) * millimeter, vector(9.4194, -10.0701) * millimeter, vector(9.5919, -9.9175) * millimeter, vector(9.7581, -9.7581) * millimeter, vector(9.9175, -9.5919) * millimeter, vector(10.0701, -9.4194) * millimeter, vector(10.2157, -9.2406) * millimeter, vector(10.3540, -9.0559) * millimeter, vector(10.4850, -8.8655) * millimeter, vector(10.6086, -8.6698) * millimeter, vector(10.7246, -8.4690) * millimeter, vector(10.8331, -8.2636) * millimeter, vector(10.9340, -8.0537) * millimeter, vector(11.0273, -7.8399) * millimeter, vector(11.1131, -7.6224) * millimeter, vector(11.1915, -7.4016) * millimeter, vector(11.2625, -7.1779) * millimeter, vector(11.3262, -6.9517) * millimeter, vector(11.3829, -6.7233) * millimeter, vector(11.4327, -6.4931) * millimeter, vector(11.4759, -6.2614) * millimeter, vector(11.5127, -6.0286) * millimeter, vector(11.5433, -5.7951) * millimeter, vector(11.5682, -5.5611) * millimeter, vector(11.5875, -5.3269) * millimeter, vector(11.6017, -5.0928) * millimeter, vector(11.6111, -4.8591) * millimeter, vector(11.6162, -4.6260) * millimeter, vector(11.6172, -4.3937) * millimeter, vector(11.6146, -4.1624) * millimeter, vector(11.6088, -3.9322) * millimeter, vector(11.6003, -3.7033) * millimeter, vector(11.5894, -3.4757) * millimeter, vector(11.5766, -3.2495) * millimeter, vector(11.5623, -3.0247) * millimeter, vector(11.5468, -2.8014) * millimeter, vector(11.5306, -2.5796) * millimeter, vector(11.5142, -2.3591) * millimeter, vector(11.4977, -2.1400) * millimeter, vector(11.4816, -1.9221) * millimeter, vector(11.4663, -1.7055) * millimeter, vector(11.4520, -1.4899) * millimeter, vector(11.4390, -1.2752) * millimeter, vector(11.4276, -1.0614) * millimeter, vector(11.4179, -0.8483) * millimeter, vector(11.4102, -0.6357) * millimeter, vector(11.4046, -0.4236) * millimeter, vector(11.4011, -0.2117) * millimeter, vector(11.4000, -0.0000) * millimeter, vector(11.4011, 0.2117) * millimeter, vector(11.4046, 0.4236) * millimeter, vector(11.4102, 0.6357) * millimeter, vector(11.4179, 0.8483) * millimeter, vector(11.4276, 1.0614) * millimeter, vector(11.4390, 1.2752) * millimeter, vector(11.4520, 1.4899) * millimeter, vector(11.4663, 1.7055) * millimeter, vector(11.4816, 1.9221) * millimeter, vector(11.4977, 2.1400) * millimeter, vector(11.5142, 2.3591) * millimeter, vector(11.5306, 2.5796) * millimeter, vector(11.5468, 2.8014) * millimeter, vector(11.5623, 3.0247) * millimeter, vector(11.5766, 3.2495) * millimeter, vector(11.5894, 3.4757) * millimeter, vector(11.6003, 3.7033) * millimeter, vector(11.6088, 3.9322) * millimeter, vector(11.6146, 4.1624) * millimeter, vector(11.6172, 4.3937) * millimeter, vector(11.6162, 4.6260) * millimeter, vector(11.6111, 4.8591) * millimeter, vector(11.6017, 5.0928) * millimeter, vector(11.5875, 5.3269) * millimeter, vector(11.5682, 5.5611) * millimeter, vector(11.5433, 5.7951) * millimeter, vector(11.5127, 6.0286) * millimeter, vector(11.4759, 6.2614) * millimeter, vector(11.4327, 6.4931) * millimeter, vector(11.3829, 6.7233) * millimeter, vector(11.3262, 6.9517) * millimeter, vector(11.2625, 7.1779) * millimeter, vector(11.1915, 7.4016) * millimeter, vector(11.1131, 7.6224) * millimeter, vector(11.0273, 7.8399) * millimeter, vector(10.9340, 8.0537) * millimeter, vector(10.8331, 8.2636) * millimeter, vector(10.7246, 8.4690) * millimeter, vector(10.6086, 8.6698) * millimeter, vector(10.4850, 8.8655) * millimeter, vector(10.3540, 9.0559) * millimeter, vector(10.2157, 9.2406) * millimeter, vector(10.0701, 9.4194) * millimeter, vector(9.9175, 9.5919) * millimeter, vector(9.7581, 9.7581) * millimeter, vector(9.5919, 9.9175) * millimeter, vector(9.4194, 10.0701) * millimeter, vector(9.2406, 10.2157) * millimeter, vector(9.0559, 10.3540) * millimeter, vector(8.8655, 10.4850) * millimeter, vector(8.6698, 10.6086) * millimeter, vector(8.4690, 10.7246) * millimeter, vector(8.2636, 10.8331) * millimeter, vector(8.0537, 10.9340) * millimeter, vector(7.8399, 11.0273) * millimeter, vector(7.6224, 11.1131) * millimeter, vector(7.4016, 11.1915) * millimeter, vector(7.1779, 11.2625) * millimeter, vector(6.9517, 11.3262) * millimeter, vector(6.7233, 11.3829) * millimeter, vector(6.4931, 11.4327) * millimeter, vector(6.2614, 11.4759) * millimeter, vector(6.0286, 11.5127) * millimeter, vector(5.7951, 11.5433) * millimeter, vector(5.5611, 11.5682) * millimeter, vector(5.3269, 11.5875) * millimeter, vector(5.0928, 11.6017) * millimeter, vector(4.8591, 11.6111) * millimeter, vector(4.6260, 11.6162) * millimeter, vector(4.3937, 11.6172) * millimeter, vector(4.1624, 11.6146) * millimeter, vector(3.9322, 11.6088) * millimeter, vector(3.7033, 11.6003) * millimeter, vector(3.4757, 11.5894) * millimeter, vector(3.2495, 11.5766) * millimeter, vector(3.0247, 11.5623) * millimeter, vector(2.8014, 11.5468) * millimeter, vector(2.5796, 11.5306) * millimeter, vector(2.3591, 11.5142) * millimeter, vector(2.1400, 11.4977) * millimeter, vector(1.9221, 11.4816) * millimeter, vector(1.7055, 11.4663) * millimeter, vector(1.4899, 11.4520) * millimeter, vector(1.2752, 11.4390) * millimeter, vector(1.0614, 11.4276) * millimeter, vector(0.8483, 11.4179) * millimeter, vector(0.6357, 11.4102) * millimeter, vector(0.4236, 11.4046) * millimeter, vector(0.2117, 11.4011) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.1985, 11.4010) * millimeter, vector(-0.3969, 11.4040) * millimeter, vector(-0.5952, 11.4087) * millimeter, vector(-0.7935, 11.4150) * millimeter, vector(-0.9916, 11.4225) * millimeter, vector(-1.1895, 11.4308) * millimeter, vector(-1.3872, 11.4394) * millimeter, vector(-1.5846, 11.4477) * millimeter, vector(-1.7816, 11.4552) * millimeter, vector(-1.9783, 11.4613) * millimeter, vector(-2.1744, 11.4652) * millimeter, vector(-2.3697, 11.4665) * millimeter, vector(-2.5643, 11.4645) * millimeter, vector(-2.7578, 11.4586) * millimeter, vector(-2.9502, 11.4482) * millimeter, vector(-3.1411, 11.4329) * millimeter, vector(-3.3303, 11.4121) * millimeter, vector(-3.5177, 11.3854) * millimeter, vector(-3.7030, 11.3526) * millimeter, vector(-3.8860, 11.3132) * millimeter, vector(-4.0666, 11.2670) * millimeter, vector(-4.2446, 11.2139) * millimeter, vector(-4.4198, 11.1538) * millimeter, vector(-4.5922, 11.0866) * millimeter, vector(-4.7616, 11.0122) * millimeter, vector(-4.9281, 10.9308) * millimeter, vector(-5.0914, 10.8425) * millimeter, vector(-5.2518, 10.7475) * millimeter, vector(-5.4091, 10.6459) * millimeter, vector(-5.5633, 10.5381) * millimeter, vector(-5.7147, 10.4245) * millimeter, vector(-5.8632, 10.3053) * millimeter, vector(-6.0090, 10.1812) * millimeter, vector(-6.1524, 10.0526) * millimeter, vector(-6.2934, 9.9199) * millimeter, vector(-6.4324, 9.7837) * millimeter, vector(-6.5697, 9.6447) * millimeter, vector(-6.7055, 9.5032) * millimeter, vector(-6.8402, 9.3599) * millimeter, vector(-6.9743, 9.2152) * millimeter, vector(-7.1080, 9.0697) * millimeter, vector(-7.2417, 8.9239) * millimeter, vector(-7.3758, 8.7781) * millimeter, vector(-7.5106, 8.6327) * millimeter, vector(-7.6463, 8.4881) * millimeter, vector(-7.7832, 8.3444) * millimeter, vector(-7.9214, 8.2021) * millimeter, vector(-8.0610, 8.0610) * millimeter, vector(-8.2021, 7.9214) * millimeter, vector(-8.3444, 7.7832) * millimeter, vector(-8.4881, 7.6463) * millimeter, vector(-8.6327, 7.5106) * millimeter, vector(-8.7781, 7.3758) * millimeter, vector(-8.9239, 7.2417) * millimeter, vector(-9.0697, 7.1080) * millimeter, vector(-9.2152, 6.9743) * millimeter, vector(-9.3599, 6.8402) * millimeter, vector(-9.5032, 6.7055) * millimeter, vector(-9.6447, 6.5697) * millimeter, vector(-9.7837, 6.4324) * millimeter, vector(-9.9199, 6.2934) * millimeter, vector(-10.0526, 6.1524) * millimeter, vector(-10.1812, 6.0090) * millimeter, vector(-10.3053, 5.8632) * millimeter, vector(-10.4245, 5.7147) * millimeter, vector(-10.5381, 5.5633) * millimeter, vector(-10.6459, 5.4091) * millimeter, vector(-10.7475, 5.2518) * millimeter, vector(-10.8425, 5.0914) * millimeter, vector(-10.9308, 4.9281) * millimeter, vector(-11.0122, 4.7616) * millimeter, vector(-11.0866, 4.5922) * millimeter, vector(-11.1538, 4.4198) * millimeter, vector(-11.2139, 4.2446) * millimeter, vector(-11.2670, 4.0666) * millimeter, vector(-11.3132, 3.8860) * millimeter, vector(-11.3526, 3.7030) * millimeter, vector(-11.3854, 3.5177) * millimeter, vector(-11.4121, 3.3303) * millimeter, vector(-11.4329, 3.1411) * millimeter, vector(-11.4482, 2.9502) * millimeter, vector(-11.4586, 2.7578) * millimeter, vector(-11.4645, 2.5643) * millimeter, vector(-11.4665, 2.3697) * millimeter, vector(-11.4652, 2.1744) * millimeter, vector(-11.4613, 1.9783) * millimeter, vector(-11.4552, 1.7816) * millimeter, vector(-11.4477, 1.5846) * millimeter, vector(-11.4394, 1.3872) * millimeter, vector(-11.4308, 1.1895) * millimeter, vector(-11.4225, 0.9916) * millimeter, vector(-11.4150, 0.7935) * millimeter, vector(-11.4087, 0.5952) * millimeter, vector(-11.4040, 0.3969) * millimeter, vector(-11.4010, 0.1985) * millimeter, vector(-11.4000, 0.0000) * millimeter, vector(-11.4010, -0.1985) * millimeter, vector(-11.4040, -0.3969) * millimeter, vector(-11.4087, -0.5952) * millimeter, vector(-11.4150, -0.7935) * millimeter, vector(-11.4225, -0.9916) * millimeter, vector(-11.4308, -1.1895) * millimeter, vector(-11.4394, -1.3872) * millimeter, vector(-11.4477, -1.5846) * millimeter, vector(-11.4552, -1.7816) * millimeter, vector(-11.4613, -1.9783) * millimeter, vector(-11.4652, -2.1744) * millimeter, vector(-11.4665, -2.3697) * millimeter, vector(-11.4645, -2.5643) * millimeter, vector(-11.4586, -2.7578) * millimeter, vector(-11.4482, -2.9502) * millimeter, vector(-11.4329, -3.1411) * millimeter, vector(-11.4121, -3.3303) * millimeter, vector(-11.3854, -3.5177) * millimeter, vector(-11.3526, -3.7030) * millimeter, vector(-11.3132, -3.8860) * millimeter, vector(-11.2670, -4.0666) * millimeter, vector(-11.2139, -4.2446) * millimeter, vector(-11.1538, -4.4198) * millimeter, vector(-11.0866, -4.5922) * millimeter, vector(-11.0122, -4.7616) * millimeter, vector(-10.9308, -4.9281) * millimeter, vector(-10.8425, -5.0914) * millimeter, vector(-10.7475, -5.2518) * millimeter, vector(-10.6459, -5.4091) * millimeter, vector(-10.5381, -5.5633) * millimeter, vector(-10.4245, -5.7147) * millimeter, vector(-10.3053, -5.8632) * millimeter, vector(-10.1812, -6.0090) * millimeter, vector(-10.0526, -6.1524) * millimeter, vector(-9.9199, -6.2934) * millimeter, vector(-9.7837, -6.4324) * millimeter, vector(-9.6447, -6.5697) * millimeter, vector(-9.5032, -6.7055) * millimeter, vector(-9.3599, -6.8402) * millimeter, vector(-9.2152, -6.9743) * millimeter, vector(-9.0697, -7.1080) * millimeter, vector(-8.9239, -7.2417) * millimeter, vector(-8.7781, -7.3758) * millimeter, vector(-8.6327, -7.5106) * millimeter, vector(-8.4881, -7.6463) * millimeter, vector(-8.3444, -7.7832) * millimeter, vector(-8.2021, -7.9214) * millimeter, vector(-8.0610, -8.0610) * millimeter, vector(-7.9214, -8.2021) * millimeter, vector(-7.7832, -8.3444) * millimeter, vector(-7.6463, -8.4881) * millimeter, vector(-7.5106, -8.6327) * millimeter, vector(-7.3758, -8.7781) * millimeter, vector(-7.2417, -8.9239) * millimeter, vector(-7.1080, -9.0697) * millimeter, vector(-6.9743, -9.2152) * millimeter, vector(-6.8402, -9.3599) * millimeter, vector(-6.7055, -9.5032) * millimeter, vector(-6.5697, -9.6447) * millimeter, vector(-6.4324, -9.7837) * millimeter, vector(-6.2934, -9.9199) * millimeter, vector(-6.1524, -10.0526) * millimeter, vector(-6.0090, -10.1812) * millimeter, vector(-5.8632, -10.3053) * millimeter, vector(-5.7147, -10.4245) * millimeter, vector(-5.5633, -10.5381) * millimeter, vector(-5.4091, -10.6459) * millimeter, vector(-5.2518, -10.7475) * millimeter, vector(-5.0914, -10.8425) * millimeter, vector(-4.9281, -10.9308) * millimeter, vector(-4.7616, -11.0122) * millimeter, vector(-4.5922, -11.0866) * millimeter, vector(-4.4198, -11.1538) * millimeter, vector(-4.2446, -11.2139) * millimeter, vector(-4.0666, -11.2670) * millimeter, vector(-3.8860, -11.3132) * millimeter, vector(-3.7030, -11.3526) * millimeter, vector(-3.5177, -11.3854) * millimeter, vector(-3.3303, -11.4121) * millimeter, vector(-3.1411, -11.4329) * millimeter, vector(-2.9502, -11.4482) * millimeter, vector(-2.7578, -11.4586) * millimeter, vector(-2.5643, -11.4645) * millimeter, vector(-2.3697, -11.4665) * millimeter, vector(-2.1744, -11.4652) * millimeter, vector(-1.9783, -11.4613) * millimeter, vector(-1.7816, -11.4552) * millimeter, vector(-1.5846, -11.4477) * millimeter, vector(-1.3872, -11.4394) * millimeter, vector(-1.1895, -11.4308) * millimeter, vector(-0.9916, -11.4225) * millimeter, vector(-0.7935, -11.4150) * millimeter, vector(-0.5952, -11.4087) * millimeter, vector(-0.3969, -11.4040) * millimeter, vector(-0.1985, -11.4010) * millimeter, vector(-0.0000, -11.4000) * millimeter, vector(0.1985, -11.4010) * millimeter, vector(0.3969, -11.4040) * millimeter, vector(0.5952, -11.4087) * millimeter, vector(0.7935, -11.4150) * millimeter, vector(0.9916, -11.4225) * millimeter, vector(1.1895, -11.4308) * millimeter, vector(1.3872, -11.4394) * millimeter, vector(1.5846, -11.4477) * millimeter, vector(1.7816, -11.4552) * millimeter, vector(1.9783, -11.4613) * millimeter, vector(2.1744, -11.4652) * millimeter, vector(2.3697, -11.4665) * millimeter, vector(2.5643, -11.4645) * millimeter, vector(2.7578, -11.4586) * millimeter, vector(2.9502, -11.4482) * millimeter, vector(3.1411, -11.4329) * millimeter, vector(3.3303, -11.4121) * millimeter, vector(3.5177, -11.3854) * millimeter, vector(3.7030, -11.3526) * millimeter, vector(3.8860, -11.3132) * millimeter, vector(4.0666, -11.2670) * millimeter, vector(4.2446, -11.2139) * millimeter, vector(4.4198, -11.1538) * millimeter, vector(4.5922, -11.0866) * millimeter, vector(4.7616, -11.0122) * millimeter, vector(4.9281, -10.9308) * millimeter, vector(5.0914, -10.8425) * millimeter, vector(5.2518, -10.7475) * millimeter, vector(5.4091, -10.6459) * millimeter, vector(5.5633, -10.5381) * millimeter, vector(5.7147, -10.4245) * millimeter, vector(5.8632, -10.3053) * millimeter, vector(6.0090, -10.1812) * millimeter, vector(6.1524, -10.0526) * millimeter, vector(6.2934, -9.9199) * millimeter, vector(6.4324, -9.7837) * millimeter, vector(6.5697, -9.6447) * millimeter, vector(6.7055, -9.5032) * millimeter, vector(6.8402, -9.3599) * millimeter, vector(6.9743, -9.2152) * millimeter, vector(7.1080, -9.0697) * millimeter, vector(7.2417, -8.9239) * millimeter, vector(7.3758, -8.7781) * millimeter, vector(7.5106, -8.6327) * millimeter, vector(7.6463, -8.4881) * millimeter, vector(7.7832, -8.3444) * millimeter, vector(7.9214, -8.2021) * millimeter, vector(8.0610, -8.0610) * millimeter, vector(8.2021, -7.9214) * millimeter, vector(8.3444, -7.7832) * millimeter, vector(8.4881, -7.6463) * millimeter, vector(8.6327, -7.5106) * millimeter, vector(8.7781, -7.3758) * millimeter, vector(8.9239, -7.2417) * millimeter, vector(9.0697, -7.1080) * millimeter, vector(9.2152, -6.9743) * millimeter, vector(9.3599, -6.8402) * millimeter, vector(9.5032, -6.7055) * millimeter, vector(9.6447, -6.5697) * millimeter, vector(9.7837, -6.4324) * millimeter, vector(9.9199, -6.2934) * millimeter, vector(10.0526, -6.1524) * millimeter, vector(10.1812, -6.0090) * millimeter, vector(10.3053, -5.8632) * millimeter, vector(10.4245, -5.7147) * millimeter, vector(10.5381, -5.5633) * millimeter, vector(10.6459, -5.4091) * millimeter, vector(10.7475, -5.2518) * millimeter, vector(10.8425, -5.0914) * millimeter, vector(10.9308, -4.9281) * millimeter, vector(11.0122, -4.7616) * millimeter, vector(11.0866, -4.5922) * millimeter, vector(11.1538, -4.4198) * millimeter, vector(11.2139, -4.2446) * millimeter, vector(11.2670, -4.0666) * millimeter, vector(11.3132, -3.8860) * millimeter, vector(11.3526, -3.7030) * millimeter, vector(11.3854, -3.5177) * millimeter, vector(11.4121, -3.3303) * millimeter, vector(11.4329, -3.1411) * millimeter, vector(11.4482, -2.9502) * millimeter, vector(11.4586, -2.7578) * millimeter, vector(11.4645, -2.5643) * millimeter, vector(11.4665, -2.3697) * millimeter, vector(11.4652, -2.1744) * millimeter, vector(11.4613, -1.9783) * millimeter, vector(11.4552, -1.7816) * millimeter, vector(11.4477, -1.5846) * millimeter, vector(11.4394, -1.3872) * millimeter, vector(11.4308, -1.1895) * millimeter, vector(11.4225, -0.9916) * millimeter, vector(11.4150, -0.7935) * millimeter, vector(11.4087, -0.5952) * millimeter, vector(11.4040, -0.3969) * millimeter, vector(11.4010, -0.1985) * millimeter, vector(11.4000, -0.0000) * millimeter, vector(11.4010, 0.1985) * millimeter, vector(11.4040, 0.3969) * millimeter, vector(11.4087, 0.5952) * millimeter, vector(11.4150, 0.7935) * millimeter, vector(11.4225, 0.9916) * millimeter, vector(11.4308, 1.1895) * millimeter, vector(11.4394, 1.3872) * millimeter, vector(11.4477, 1.5846) * millimeter, vector(11.4552, 1.7816) * millimeter, vector(11.4613, 1.9783) * millimeter, vector(11.4652, 2.1744) * millimeter, vector(11.4665, 2.3697) * millimeter, vector(11.4645, 2.5643) * millimeter, vector(11.4586, 2.7578) * millimeter, vector(11.4482, 2.9502) * millimeter, vector(11.4329, 3.1411) * millimeter, vector(11.4121, 3.3303) * millimeter, vector(11.3854, 3.5177) * millimeter, vector(11.3526, 3.7030) * millimeter, vector(11.3132, 3.8860) * millimeter, vector(11.2670, 4.0666) * millimeter, vector(11.2139, 4.2446) * millimeter, vector(11.1538, 4.4198) * millimeter, vector(11.0866, 4.5922) * millimeter, vector(11.0122, 4.7616) * millimeter, vector(10.9308, 4.9281) * millimeter, vector(10.8425, 5.0914) * millimeter, vector(10.7475, 5.2518) * millimeter, vector(10.6459, 5.4091) * millimeter, vector(10.5381, 5.5633) * millimeter, vector(10.4245, 5.7147) * millimeter, vector(10.3053, 5.8632) * millimeter, vector(10.1812, 6.0090) * millimeter, vector(10.0526, 6.1524) * millimeter, vector(9.9199, 6.2934) * millimeter, vector(9.7837, 6.4324) * millimeter, vector(9.6447, 6.5697) * millimeter, vector(9.5032, 6.7055) * millimeter, vector(9.3599, 6.8402) * millimeter, vector(9.2152, 6.9743) * millimeter, vector(9.0697, 7.1080) * millimeter, vector(8.9239, 7.2417) * millimeter, vector(8.7781, 7.3758) * millimeter, vector(8.6327, 7.5106) * millimeter, vector(8.4881, 7.6463) * millimeter, vector(8.3444, 7.7832) * millimeter, vector(8.2021, 7.9214) * millimeter, vector(8.0610, 8.0610) * millimeter, vector(7.9214, 8.2021) * millimeter, vector(7.7832, 8.3444) * millimeter, vector(7.6463, 8.4881) * millimeter, vector(7.5106, 8.6327) * millimeter, vector(7.3758, 8.7781) * millimeter, vector(7.2417, 8.9239) * millimeter, vector(7.1080, 9.0697) * millimeter, vector(6.9743, 9.2152) * millimeter, vector(6.8402, 9.3599) * millimeter, vector(6.7055, 9.5032) * millimeter, vector(6.5697, 9.6447) * millimeter, vector(6.4324, 9.7837) * millimeter, vector(6.2934, 9.9199) * millimeter, vector(6.1524, 10.0526) * millimeter, vector(6.0090, 10.1812) * millimeter, vector(5.8632, 10.3053) * millimeter, vector(5.7147, 10.4245) * millimeter, vector(5.5633, 10.5381) * millimeter, vector(5.4091, 10.6459) * millimeter, vector(5.2518, 10.7475) * millimeter, vector(5.0914, 10.8425) * millimeter, vector(4.9281, 10.9308) * millimeter, vector(4.7616, 11.0122) * millimeter, vector(4.5922, 11.0866) * millimeter, vector(4.4198, 11.1538) * millimeter, vector(4.2446, 11.2139) * millimeter, vector(4.0666, 11.2670) * millimeter, vector(3.8860, 11.3132) * millimeter, vector(3.7030, 11.3526) * millimeter, vector(3.5177, 11.3854) * millimeter, vector(3.3303, 11.4121) * millimeter, vector(3.1411, 11.4329) * millimeter, vector(2.9502, 11.4482) * millimeter, vector(2.7578, 11.4586) * millimeter, vector(2.5643, 11.4645) * millimeter, vector(2.3697, 11.4665) * millimeter, vector(2.1744, 11.4652) * millimeter, vector(1.9783, 11.4613) * millimeter, vector(1.7816, 11.4552) * millimeter, vector(1.5846, 11.4477) * millimeter, vector(1.3872, 11.4394) * millimeter, vector(1.1895, 11.4308) * millimeter, vector(0.9916, 11.4225) * millimeter, vector(0.7935, 11.4150) * millimeter, vector(0.5952, 11.4087) * millimeter, vector(0.3969, 11.4040) * millimeter, vector(0.1985, 11.4010) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.0992, 11.4003) * millimeter, vector(-0.1984, 11.4010) * millimeter, vector(-0.2973, 11.4022) * millimeter, vector(-0.3961, 11.4038) * millimeter, vector(-0.4945, 11.4057) * millimeter, vector(-0.5925, 11.4078) * millimeter, vector(-0.6902, 11.4100) * millimeter, vector(-0.7873, 11.4122) * millimeter, vector(-0.8838, 11.4141) * millimeter, vector(-0.9798, 11.4158) * millimeter, vector(-1.0751, 11.4170) * millimeter, vector(-1.1698, 11.4176) * millimeter, vector(-1.2638, 11.4174) * millimeter, vector(-1.3571, 11.4162) * millimeter, vector(-1.4497, 11.4140) * millimeter, vector(-1.5415, 11.4106) * millimeter, vector(-1.6327, 11.4059) * millimeter, vector(-1.7231, 11.3997) * millimeter, vector(-1.8129, 11.3920) * millimeter, vector(-1.9021, 11.3827) * millimeter, vector(-1.9907, 11.3717) * millimeter, vector(-2.0787, 11.3590) * millimeter, vector(-2.1662, 11.3444) * millimeter, vector(-2.2533, 11.3281) * millimeter, vector(-2.3400, 11.3099) * millimeter, vector(-2.4264, 11.2898) * millimeter, vector(-2.5126, 11.2679) * millimeter, vector(-2.5987, 11.2441) * millimeter, vector(-2.6846, 11.2186) * millimeter, vector(-2.7705, 11.1914) * millimeter, vector(-2.8564, 11.1624) * millimeter, vector(-2.9425, 11.1319) * millimeter, vector(-3.0286, 11.0999) * millimeter, vector(-3.1150, 11.0665) * millimeter, vector(-3.2016, 11.0319) * millimeter, vector(-3.2886, 10.9961) * millimeter, vector(-3.3758, 10.9594) * millimeter, vector(-3.4634, 10.9218) * millimeter, vector(-3.5515, 10.8835) * millimeter, vector(-3.6399, 10.8447) * millimeter, vector(-3.7288, 10.8056) * millimeter, vector(-3.8181, 10.7662) * millimeter, vector(-3.9079, 10.7267) * millimeter, vector(-3.9981, 10.6873) * millimeter, vector(-4.0887, 10.6480) * millimeter, vector(-4.1797, 10.6090) * millimeter, vector(-4.2710, 10.5704) * millimeter, vector(-4.3626, 10.5322) * millimeter, vector(-4.4544, 10.4945) * millimeter, vector(-4.5462, 10.4572) * millimeter, vector(-4.6381, 10.4205) * millimeter, vector(-4.7300, 10.3841) * millimeter, vector(-4.8216, 10.3482) * millimeter, vector(-4.9130, 10.3127) * millimeter, vector(-5.0040, 10.2773) * millimeter, vector(-5.0946, 10.2422) * millimeter, vector(-5.1846, 10.2071) * millimeter, vector(-5.2739, 10.1719) * millimeter, vector(-5.3624, 10.1365) * millimeter, vector(-5.4501, 10.1008) * millimeter, vector(-5.5368, 10.0646) * millimeter, vector(-5.6226, 10.0279) * millimeter, vector(-5.7073, 9.9904) * millimeter, vector(-5.7908, 9.9521) * millimeter, vector(-5.8732, 9.9129) * millimeter, vector(-5.9544, 9.8725) * millimeter, vector(-6.0345, 9.8311) * millimeter, vector(-6.1133, 9.7884) * millimeter, vector(-6.1909, 9.7443) * millimeter, vector(-6.2673, 9.6988) * millimeter, vector(-6.3426, 9.6519) * millimeter, vector(-6.4168, 9.6035) * millimeter, vector(-6.4900, 9.5535) * millimeter, vector(-6.5622, 9.5018) * millimeter, vector(-6.6334, 9.4486) * millimeter, vector(-6.7038, 9.3938) * millimeter, vector(-6.7734, 9.3373) * millimeter, vector(-6.8424, 9.2792) * millimeter, vector(-6.9107, 9.2196) * millimeter, vector(-6.9785, 9.1585) * millimeter, vector(-7.0459, 9.0960) * millimeter, vector(-7.1129, 9.0321) * millimeter, vector(-7.1797, 8.9669) * millimeter, vector(-7.2463, 8.9006) * millimeter, vector(-7.3128, 8.8333) * millimeter, vector(-7.3794, 8.7650) * millimeter, vector(-7.4461, 8.6960) * millimeter, vector(-7.5129, 8.6263) * millimeter, vector(-7.5801, 8.5561) * millimeter, vector(-7.6475, 8.4855) * millimeter, vector(-7.7154, 8.4147) * millimeter, vector(-7.7836, 8.3437) * millimeter, vector(-7.8523, 8.2728) * millimeter, vector(-7.9215, 8.2020) * millimeter, vector(-7.9910, 8.1314) * millimeter, vector(-8.0610, 8.0610) * millimeter, vector(-8.1314, 7.9910) * millimeter, vector(-8.2020, 7.9215) * millimeter, vector(-8.2728, 7.8523) * millimeter, vector(-8.3437, 7.7836) * millimeter, vector(-8.4147, 7.7154) * millimeter, vector(-8.4855, 7.6475) * millimeter, vector(-8.5561, 7.5801) * millimeter, vector(-8.6263, 7.5129) * millimeter, vector(-8.6960, 7.4461) * millimeter, vector(-8.7650, 7.3794) * millimeter, vector(-8.8333, 7.3128) * millimeter, vector(-8.9006, 7.2463) * millimeter, vector(-8.9669, 7.1797) * millimeter, vector(-9.0321, 7.1129) * millimeter, vector(-9.0960, 7.0459) * millimeter, vector(-9.1585, 6.9785) * millimeter, vector(-9.2196, 6.9107) * millimeter, vector(-9.2792, 6.8424) * millimeter, vector(-9.3373, 6.7734) * millimeter, vector(-9.3938, 6.7038) * millimeter, vector(-9.4486, 6.6334) * millimeter, vector(-9.5018, 6.5622) * millimeter, vector(-9.5535, 6.4900) * millimeter, vector(-9.6035, 6.4168) * millimeter, vector(-9.6519, 6.3426) * millimeter, vector(-9.6988, 6.2673) * millimeter, vector(-9.7443, 6.1909) * millimeter, vector(-9.7884, 6.1133) * millimeter, vector(-9.8311, 6.0345) * millimeter, vector(-9.8725, 5.9544) * millimeter, vector(-9.9129, 5.8732) * millimeter, vector(-9.9521, 5.7908) * millimeter, vector(-9.9904, 5.7073) * millimeter, vector(-10.0279, 5.6226) * millimeter, vector(-10.0646, 5.5368) * millimeter, vector(-10.1008, 5.4501) * millimeter, vector(-10.1365, 5.3624) * millimeter, vector(-10.1719, 5.2739) * millimeter, vector(-10.2071, 5.1846) * millimeter, vector(-10.2422, 5.0946) * millimeter, vector(-10.2773, 5.0040) * millimeter, vector(-10.3127, 4.9130) * millimeter, vector(-10.3482, 4.8216) * millimeter, vector(-10.3841, 4.7300) * millimeter, vector(-10.4205, 4.6381) * millimeter, vector(-10.4572, 4.5462) * millimeter, vector(-10.4945, 4.4544) * millimeter, vector(-10.5322, 4.3626) * millimeter, vector(-10.5704, 4.2710) * millimeter, vector(-10.6090, 4.1797) * millimeter, vector(-10.6480, 4.0887) * millimeter, vector(-10.6873, 3.9981) * millimeter, vector(-10.7267, 3.9079) * millimeter, vector(-10.7662, 3.8181) * millimeter, vector(-10.8056, 3.7288) * millimeter, vector(-10.8447, 3.6399) * millimeter, vector(-10.8835, 3.5515) * millimeter, vector(-10.9218, 3.4634) * millimeter, vector(-10.9594, 3.3758) * millimeter, vector(-10.9961, 3.2886) * millimeter, vector(-11.0319, 3.2016) * millimeter, vector(-11.0665, 3.1150) * millimeter, vector(-11.0999, 3.0286) * millimeter, vector(-11.1319, 2.9425) * millimeter, vector(-11.1624, 2.8564) * millimeter, vector(-11.1914, 2.7705) * millimeter, vector(-11.2186, 2.6846) * millimeter, vector(-11.2441, 2.5987) * millimeter, vector(-11.2679, 2.5126) * millimeter, vector(-11.2898, 2.4264) * millimeter, vector(-11.3099, 2.3400) * millimeter, vector(-11.3281, 2.2533) * millimeter, vector(-11.3444, 2.1662) * millimeter, vector(-11.3590, 2.0787) * millimeter, vector(-11.3717, 1.9907) * millimeter, vector(-11.3827, 1.9021) * millimeter, vector(-11.3920, 1.8129) * millimeter, vector(-11.3997, 1.7231) * millimeter, vector(-11.4059, 1.6327) * millimeter, vector(-11.4106, 1.5415) * millimeter, vector(-11.4140, 1.4497) * millimeter, vector(-11.4162, 1.3571) * millimeter, vector(-11.4174, 1.2638) * millimeter, vector(-11.4176, 1.1698) * millimeter, vector(-11.4170, 1.0751) * millimeter, vector(-11.4158, 0.9798) * millimeter, vector(-11.4141, 0.8838) * millimeter, vector(-11.4122, 0.7873) * millimeter, vector(-11.4100, 0.6902) * millimeter, vector(-11.4078, 0.5925) * millimeter, vector(-11.4057, 0.4945) * millimeter, vector(-11.4038, 0.3961) * millimeter, vector(-11.4022, 0.2973) * millimeter, vector(-11.4010, 0.1984) * millimeter, vector(-11.4003, 0.0992) * millimeter, vector(-11.4000, 0.0000) * millimeter, vector(-11.4003, -0.0992) * millimeter, vector(-11.4010, -0.1984) * millimeter, vector(-11.4022, -0.2973) * millimeter, vector(-11.4038, -0.3961) * millimeter, vector(-11.4057, -0.4945) * millimeter, vector(-11.4078, -0.5925) * millimeter, vector(-11.4100, -0.6902) * millimeter, vector(-11.4122, -0.7873) * millimeter, vector(-11.4141, -0.8838) * millimeter, vector(-11.4158, -0.9798) * millimeter, vector(-11.4170, -1.0751) * millimeter, vector(-11.4176, -1.1698) * millimeter, vector(-11.4174, -1.2638) * millimeter, vector(-11.4162, -1.3571) * millimeter, vector(-11.4140, -1.4497) * millimeter, vector(-11.4106, -1.5415) * millimeter, vector(-11.4059, -1.6327) * millimeter, vector(-11.3997, -1.7231) * millimeter, vector(-11.3920, -1.8129) * millimeter, vector(-11.3827, -1.9021) * millimeter, vector(-11.3717, -1.9907) * millimeter, vector(-11.3590, -2.0787) * millimeter, vector(-11.3444, -2.1662) * millimeter, vector(-11.3281, -2.2533) * millimeter, vector(-11.3099, -2.3400) * millimeter, vector(-11.2898, -2.4264) * millimeter, vector(-11.2679, -2.5126) * millimeter, vector(-11.2441, -2.5987) * millimeter, vector(-11.2186, -2.6846) * millimeter, vector(-11.1914, -2.7705) * millimeter, vector(-11.1624, -2.8564) * millimeter, vector(-11.1319, -2.9425) * millimeter, vector(-11.0999, -3.0286) * millimeter, vector(-11.0665, -3.1150) * millimeter, vector(-11.0319, -3.2016) * millimeter, vector(-10.9961, -3.2886) * millimeter, vector(-10.9594, -3.3758) * millimeter, vector(-10.9218, -3.4634) * millimeter, vector(-10.8835, -3.5515) * millimeter, vector(-10.8447, -3.6399) * millimeter, vector(-10.8056, -3.7288) * millimeter, vector(-10.7662, -3.8181) * millimeter, vector(-10.7267, -3.9079) * millimeter, vector(-10.6873, -3.9981) * millimeter, vector(-10.6480, -4.0887) * millimeter, vector(-10.6090, -4.1797) * millimeter, vector(-10.5704, -4.2710) * millimeter, vector(-10.5322, -4.3626) * millimeter, vector(-10.4945, -4.4544) * millimeter, vector(-10.4572, -4.5462) * millimeter, vector(-10.4205, -4.6381) * millimeter, vector(-10.3841, -4.7300) * millimeter, vector(-10.3482, -4.8216) * millimeter, vector(-10.3127, -4.9130) * millimeter, vector(-10.2773, -5.0040) * millimeter, vector(-10.2422, -5.0946) * millimeter, vector(-10.2071, -5.1846) * millimeter, vector(-10.1719, -5.2739) * millimeter, vector(-10.1365, -5.3624) * millimeter, vector(-10.1008, -5.4501) * millimeter, vector(-10.0646, -5.5368) * millimeter, vector(-10.0279, -5.6226) * millimeter, vector(-9.9904, -5.7073) * millimeter, vector(-9.9521, -5.7908) * millimeter, vector(-9.9129, -5.8732) * millimeter, vector(-9.8725, -5.9544) * millimeter, vector(-9.8311, -6.0345) * millimeter, vector(-9.7884, -6.1133) * millimeter, vector(-9.7443, -6.1909) * millimeter, vector(-9.6988, -6.2673) * millimeter, vector(-9.6519, -6.3426) * millimeter, vector(-9.6035, -6.4168) * millimeter, vector(-9.5535, -6.4900) * millimeter, vector(-9.5018, -6.5622) * millimeter, vector(-9.4486, -6.6334) * millimeter, vector(-9.3938, -6.7038) * millimeter, vector(-9.3373, -6.7734) * millimeter, vector(-9.2792, -6.8424) * millimeter, vector(-9.2196, -6.9107) * millimeter, vector(-9.1585, -6.9785) * millimeter, vector(-9.0960, -7.0459) * millimeter, vector(-9.0321, -7.1129) * millimeter, vector(-8.9669, -7.1797) * millimeter, vector(-8.9006, -7.2463) * millimeter, vector(-8.8333, -7.3128) * millimeter, vector(-8.7650, -7.3794) * millimeter, vector(-8.6960, -7.4461) * millimeter, vector(-8.6263, -7.5129) * millimeter, vector(-8.5561, -7.5801) * millimeter, vector(-8.4855, -7.6475) * millimeter, vector(-8.4147, -7.7154) * millimeter, vector(-8.3437, -7.7836) * millimeter, vector(-8.2728, -7.8523) * millimeter, vector(-8.2020, -7.9215) * millimeter, vector(-8.1314, -7.9910) * millimeter, vector(-8.0610, -8.0610) * millimeter, vector(-7.9910, -8.1314) * millimeter, vector(-7.9215, -8.2020) * millimeter, vector(-7.8523, -8.2728) * millimeter, vector(-7.7836, -8.3437) * millimeter, vector(-7.7154, -8.4147) * millimeter, vector(-7.6475, -8.4855) * millimeter, vector(-7.5801, -8.5561) * millimeter, vector(-7.5129, -8.6263) * millimeter, vector(-7.4461, -8.6960) * millimeter, vector(-7.3794, -8.7650) * millimeter, vector(-7.3128, -8.8333) * millimeter, vector(-7.2463, -8.9006) * millimeter, vector(-7.1797, -8.9669) * millimeter, vector(-7.1129, -9.0321) * millimeter, vector(-7.0459, -9.0960) * millimeter, vector(-6.9785, -9.1585) * millimeter, vector(-6.9107, -9.2196) * millimeter, vector(-6.8424, -9.2792) * millimeter, vector(-6.7734, -9.3373) * millimeter, vector(-6.7038, -9.3938) * millimeter, vector(-6.6334, -9.4486) * millimeter, vector(-6.5622, -9.5018) * millimeter, vector(-6.4900, -9.5535) * millimeter, vector(-6.4168, -9.6035) * millimeter, vector(-6.3426, -9.6519) * millimeter, vector(-6.2673, -9.6988) * millimeter, vector(-6.1909, -9.7443) * millimeter, vector(-6.1133, -9.7884) * millimeter, vector(-6.0345, -9.8311) * millimeter, vector(-5.9544, -9.8725) * millimeter, vector(-5.8732, -9.9129) * millimeter, vector(-5.7908, -9.9521) * millimeter, vector(-5.7073, -9.9904) * millimeter, vector(-5.6226, -10.0279) * millimeter, vector(-5.5368, -10.0646) * millimeter, vector(-5.4501, -10.1008) * millimeter, vector(-5.3624, -10.1365) * millimeter, vector(-5.2739, -10.1719) * millimeter, vector(-5.1846, -10.2071) * millimeter, vector(-5.0946, -10.2422) * millimeter, vector(-5.0040, -10.2773) * millimeter, vector(-4.9130, -10.3127) * millimeter, vector(-4.8216, -10.3482) * millimeter, vector(-4.7300, -10.3841) * millimeter, vector(-4.6381, -10.4205) * millimeter, vector(-4.5462, -10.4572) * millimeter, vector(-4.4544, -10.4945) * millimeter, vector(-4.3626, -10.5322) * millimeter, vector(-4.2710, -10.5704) * millimeter, vector(-4.1797, -10.6090) * millimeter, vector(-4.0887, -10.6480) * millimeter, vector(-3.9981, -10.6873) * millimeter, vector(-3.9079, -10.7267) * millimeter, vector(-3.8181, -10.7662) * millimeter, vector(-3.7288, -10.8056) * millimeter, vector(-3.6399, -10.8447) * millimeter, vector(-3.5515, -10.8835) * millimeter, vector(-3.4634, -10.9218) * millimeter, vector(-3.3758, -10.9594) * millimeter, vector(-3.2886, -10.9961) * millimeter, vector(-3.2016, -11.0319) * millimeter, vector(-3.1150, -11.0665) * millimeter, vector(-3.0286, -11.0999) * millimeter, vector(-2.9425, -11.1319) * millimeter, vector(-2.8564, -11.1624) * millimeter, vector(-2.7705, -11.1914) * millimeter, vector(-2.6846, -11.2186) * millimeter, vector(-2.5987, -11.2441) * millimeter, vector(-2.5126, -11.2679) * millimeter, vector(-2.4264, -11.2898) * millimeter, vector(-2.3400, -11.3099) * millimeter, vector(-2.2533, -11.3281) * millimeter, vector(-2.1662, -11.3444) * millimeter, vector(-2.0787, -11.3590) * millimeter, vector(-1.9907, -11.3717) * millimeter, vector(-1.9021, -11.3827) * millimeter, vector(-1.8129, -11.3920) * millimeter, vector(-1.7231, -11.3997) * millimeter, vector(-1.6327, -11.4059) * millimeter, vector(-1.5415, -11.4106) * millimeter, vector(-1.4497, -11.4140) * millimeter, vector(-1.3571, -11.4162) * millimeter, vector(-1.2638, -11.4174) * millimeter, vector(-1.1698, -11.4176) * millimeter, vector(-1.0751, -11.4170) * millimeter, vector(-0.9798, -11.4158) * millimeter, vector(-0.8838, -11.4141) * millimeter, vector(-0.7873, -11.4122) * millimeter, vector(-0.6902, -11.4100) * millimeter, vector(-0.5925, -11.4078) * millimeter, vector(-0.4945, -11.4057) * millimeter, vector(-0.3961, -11.4038) * millimeter, vector(-0.2973, -11.4022) * millimeter, vector(-0.1984, -11.4010) * millimeter, vector(-0.0992, -11.4003) * millimeter, vector(-0.0000, -11.4000) * millimeter, vector(0.0992, -11.4003) * millimeter, vector(0.1984, -11.4010) * millimeter, vector(0.2973, -11.4022) * millimeter, vector(0.3961, -11.4038) * millimeter, vector(0.4945, -11.4057) * millimeter, vector(0.5925, -11.4078) * millimeter, vector(0.6902, -11.4100) * millimeter, vector(0.7873, -11.4122) * millimeter, vector(0.8838, -11.4141) * millimeter, vector(0.9798, -11.4158) * millimeter, vector(1.0751, -11.4170) * millimeter, vector(1.1698, -11.4176) * millimeter, vector(1.2638, -11.4174) * millimeter, vector(1.3571, -11.4162) * millimeter, vector(1.4497, -11.4140) * millimeter, vector(1.5415, -11.4106) * millimeter, vector(1.6327, -11.4059) * millimeter, vector(1.7231, -11.3997) * millimeter, vector(1.8129, -11.3920) * millimeter, vector(1.9021, -11.3827) * millimeter, vector(1.9907, -11.3717) * millimeter, vector(2.0787, -11.3590) * millimeter, vector(2.1662, -11.3444) * millimeter, vector(2.2533, -11.3281) * millimeter, vector(2.3400, -11.3099) * millimeter, vector(2.4264, -11.2898) * millimeter, vector(2.5126, -11.2679) * millimeter, vector(2.5987, -11.2441) * millimeter, vector(2.6846, -11.2186) * millimeter, vector(2.7705, -11.1914) * millimeter, vector(2.8564, -11.1624) * millimeter, vector(2.9425, -11.1319) * millimeter, vector(3.0286, -11.0999) * millimeter, vector(3.1150, -11.0665) * millimeter, vector(3.2016, -11.0319) * millimeter, vector(3.2886, -10.9961) * millimeter, vector(3.3758, -10.9594) * millimeter, vector(3.4634, -10.9218) * millimeter, vector(3.5515, -10.8835) * millimeter, vector(3.6399, -10.8447) * millimeter, vector(3.7288, -10.8056) * millimeter, vector(3.8181, -10.7662) * millimeter, vector(3.9079, -10.7267) * millimeter, vector(3.9981, -10.6873) * millimeter, vector(4.0887, -10.6480) * millimeter, vector(4.1797, -10.6090) * millimeter, vector(4.2710, -10.5704) * millimeter, vector(4.3626, -10.5322) * millimeter, vector(4.4544, -10.4945) * millimeter, vector(4.5462, -10.4572) * millimeter, vector(4.6381, -10.4205) * millimeter, vector(4.7300, -10.3841) * millimeter, vector(4.8216, -10.3482) * millimeter, vector(4.9130, -10.3127) * millimeter, vector(5.0040, -10.2773) * millimeter, vector(5.0946, -10.2422) * millimeter, vector(5.1846, -10.2071) * millimeter, vector(5.2739, -10.1719) * millimeter, vector(5.3624, -10.1365) * millimeter, vector(5.4501, -10.1008) * millimeter, vector(5.5368, -10.0646) * millimeter, vector(5.6226, -10.0279) * millimeter, vector(5.7073, -9.9904) * millimeter, vector(5.7908, -9.9521) * millimeter, vector(5.8732, -9.9129) * millimeter, vector(5.9544, -9.8725) * millimeter, vector(6.0345, -9.8311) * millimeter, vector(6.1133, -9.7884) * millimeter, vector(6.1909, -9.7443) * millimeter, vector(6.2673, -9.6988) * millimeter, vector(6.3426, -9.6519) * millimeter, vector(6.4168, -9.6035) * millimeter, vector(6.4900, -9.5535) * millimeter, vector(6.5622, -9.5018) * millimeter, vector(6.6334, -9.4486) * millimeter, vector(6.7038, -9.3938) * millimeter, vector(6.7734, -9.3373) * millimeter, vector(6.8424, -9.2792) * millimeter, vector(6.9107, -9.2196) * millimeter, vector(6.9785, -9.1585) * millimeter, vector(7.0459, -9.0960) * millimeter, vector(7.1129, -9.0321) * millimeter, vector(7.1797, -8.9669) * millimeter, vector(7.2463, -8.9006) * millimeter, vector(7.3128, -8.8333) * millimeter, vector(7.3794, -8.7650) * millimeter, vector(7.4461, -8.6960) * millimeter, vector(7.5129, -8.6263) * millimeter, vector(7.5801, -8.5561) * millimeter, vector(7.6475, -8.4855) * millimeter, vector(7.7154, -8.4147) * millimeter, vector(7.7836, -8.3437) * millimeter, vector(7.8523, -8.2728) * millimeter, vector(7.9215, -8.2020) * millimeter, vector(7.9910, -8.1314) * millimeter, vector(8.0610, -8.0610) * millimeter, vector(8.1314, -7.9910) * millimeter, vector(8.2020, -7.9215) * millimeter, vector(8.2728, -7.8523) * millimeter, vector(8.3437, -7.7836) * millimeter, vector(8.4147, -7.7154) * millimeter, vector(8.4855, -7.6475) * millimeter, vector(8.5561, -7.5801) * millimeter, vector(8.6263, -7.5129) * millimeter, vector(8.6960, -7.4461) * millimeter, vector(8.7650, -7.3794) * millimeter, vector(8.8333, -7.3128) * millimeter, vector(8.9006, -7.2463) * millimeter, vector(8.9669, -7.1797) * millimeter, vector(9.0321, -7.1129) * millimeter, vector(9.0960, -7.0459) * millimeter, vector(9.1585, -6.9785) * millimeter, vector(9.2196, -6.9107) * millimeter, vector(9.2792, -6.8424) * millimeter, vector(9.3373, -6.7734) * millimeter, vector(9.3938, -6.7038) * millimeter, vector(9.4486, -6.6334) * millimeter, vector(9.5018, -6.5622) * millimeter, vector(9.5535, -6.4900) * millimeter, vector(9.6035, -6.4168) * millimeter, vector(9.6519, -6.3426) * millimeter, vector(9.6988, -6.2673) * millimeter, vector(9.7443, -6.1909) * millimeter, vector(9.7884, -6.1133) * millimeter, vector(9.8311, -6.0345) * millimeter, vector(9.8725, -5.9544) * millimeter, vector(9.9129, -5.8732) * millimeter, vector(9.9521, -5.7908) * millimeter, vector(9.9904, -5.7073) * millimeter, vector(10.0279, -5.6226) * millimeter, vector(10.0646, -5.5368) * millimeter, vector(10.1008, -5.4501) * millimeter, vector(10.1365, -5.3624) * millimeter, vector(10.1719, -5.2739) * millimeter, vector(10.2071, -5.1846) * millimeter, vector(10.2422, -5.0946) * millimeter, vector(10.2773, -5.0040) * millimeter, vector(10.3127, -4.9130) * millimeter, vector(10.3482, -4.8216) * millimeter, vector(10.3841, -4.7300) * millimeter, vector(10.4205, -4.6381) * millimeter, vector(10.4572, -4.5462) * millimeter, vector(10.4945, -4.4544) * millimeter, vector(10.5322, -4.3626) * millimeter, vector(10.5704, -4.2710) * millimeter, vector(10.6090, -4.1797) * millimeter, vector(10.6480, -4.0887) * millimeter, vector(10.6873, -3.9981) * millimeter, vector(10.7267, -3.9079) * millimeter, vector(10.7662, -3.8181) * millimeter, vector(10.8056, -3.7288) * millimeter, vector(10.8447, -3.6399) * millimeter, vector(10.8835, -3.5515) * millimeter, vector(10.9218, -3.4634) * millimeter, vector(10.9594, -3.3758) * millimeter, vector(10.9961, -3.2886) * millimeter, vector(11.0319, -3.2016) * millimeter, vector(11.0665, -3.1150) * millimeter, vector(11.0999, -3.0286) * millimeter, vector(11.1319, -2.9425) * millimeter, vector(11.1624, -2.8564) * millimeter, vector(11.1914, -2.7705) * millimeter, vector(11.2186, -2.6846) * millimeter, vector(11.2441, -2.5987) * millimeter, vector(11.2679, -2.5126) * millimeter, vector(11.2898, -2.4264) * millimeter, vector(11.3099, -2.3400) * millimeter, vector(11.3281, -2.2533) * millimeter, vector(11.3444, -2.1662) * millimeter, vector(11.3590, -2.0787) * millimeter, vector(11.3717, -1.9907) * millimeter, vector(11.3827, -1.9021) * millimeter, vector(11.3920, -1.8129) * millimeter, vector(11.3997, -1.7231) * millimeter, vector(11.4059, -1.6327) * millimeter, vector(11.4106, -1.5415) * millimeter, vector(11.4140, -1.4497) * millimeter, vector(11.4162, -1.3571) * millimeter, vector(11.4174, -1.2638) * millimeter, vector(11.4176, -1.1698) * millimeter, vector(11.4170, -1.0751) * millimeter, vector(11.4158, -0.9798) * millimeter, vector(11.4141, -0.8838) * millimeter, vector(11.4122, -0.7873) * millimeter, vector(11.4100, -0.6902) * millimeter, vector(11.4078, -0.5925) * millimeter, vector(11.4057, -0.4945) * millimeter, vector(11.4038, -0.3961) * millimeter, vector(11.4022, -0.2973) * millimeter, vector(11.4010, -0.1984) * millimeter, vector(11.4003, -0.0992) * millimeter, vector(11.4000, -0.0000) * millimeter, vector(11.4003, 0.0992) * millimeter, vector(11.4010, 0.1984) * millimeter, vector(11.4022, 0.2973) * millimeter, vector(11.4038, 0.3961) * millimeter, vector(11.4057, 0.4945) * millimeter, vector(11.4078, 0.5925) * millimeter, vector(11.4100, 0.6902) * millimeter, vector(11.4122, 0.7873) * millimeter, vector(11.4141, 0.8838) * millimeter, vector(11.4158, 0.9798) * millimeter, vector(11.4170, 1.0751) * millimeter, vector(11.4176, 1.1698) * millimeter, vector(11.4174, 1.2638) * millimeter, vector(11.4162, 1.3571) * millimeter, vector(11.4140, 1.4497) * millimeter, vector(11.4106, 1.5415) * millimeter, vector(11.4059, 1.6327) * millimeter, vector(11.3997, 1.7231) * millimeter, vector(11.3920, 1.8129) * millimeter, vector(11.3827, 1.9021) * millimeter, vector(11.3717, 1.9907) * millimeter, vector(11.3590, 2.0787) * millimeter, vector(11.3444, 2.1662) * millimeter, vector(11.3281, 2.2533) * millimeter, vector(11.3099, 2.3400) * millimeter, vector(11.2898, 2.4264) * millimeter, vector(11.2679, 2.5126) * millimeter, vector(11.2441, 2.5987) * millimeter, vector(11.2186, 2.6846) * millimeter, vector(11.1914, 2.7705) * millimeter, vector(11.1624, 2.8564) * millimeter, vector(11.1319, 2.9425) * millimeter, vector(11.0999, 3.0286) * millimeter, vector(11.0665, 3.1150) * millimeter, vector(11.0319, 3.2016) * millimeter, vector(10.9961, 3.2886) * millimeter, vector(10.9594, 3.3758) * millimeter, vector(10.9218, 3.4634) * millimeter, vector(10.8835, 3.5515) * millimeter, vector(10.8447, 3.6399) * millimeter, vector(10.8056, 3.7288) * millimeter, vector(10.7662, 3.8181) * millimeter, vector(10.7267, 3.9079) * millimeter, vector(10.6873, 3.9981) * millimeter, vector(10.6480, 4.0887) * millimeter, vector(10.6090, 4.1797) * millimeter, vector(10.5704, 4.2710) * millimeter, vector(10.5322, 4.3626) * millimeter, vector(10.4945, 4.4544) * millimeter, vector(10.4572, 4.5462) * millimeter, vector(10.4205, 4.6381) * millimeter, vector(10.3841, 4.7300) * millimeter, vector(10.3482, 4.8216) * millimeter, vector(10.3127, 4.9130) * millimeter, vector(10.2773, 5.0040) * millimeter, vector(10.2422, 5.0946) * millimeter, vector(10.2071, 5.1846) * millimeter, vector(10.1719, 5.2739) * millimeter, vector(10.1365, 5.3624) * millimeter, vector(10.1008, 5.4501) * millimeter, vector(10.0646, 5.5368) * millimeter, vector(10.0279, 5.6226) * millimeter, vector(9.9904, 5.7073) * millimeter, vector(9.9521, 5.7908) * millimeter, vector(9.9129, 5.8732) * millimeter, vector(9.8725, 5.9544) * millimeter, vector(9.8311, 6.0345) * millimeter, vector(9.7884, 6.1133) * millimeter, vector(9.7443, 6.1909) * millimeter, vector(9.6988, 6.2673) * millimeter, vector(9.6519, 6.3426) * millimeter, vector(9.6035, 6.4168) * millimeter, vector(9.5535, 6.4900) * millimeter, vector(9.5018, 6.5622) * millimeter, vector(9.4486, 6.6334) * millimeter, vector(9.3938, 6.7038) * millimeter, vector(9.3373, 6.7734) * millimeter, vector(9.2792, 6.8424) * millimeter, vector(9.2196, 6.9107) * millimeter, vector(9.1585, 6.9785) * millimeter, vector(9.0960, 7.0459) * millimeter, vector(9.0321, 7.1129) * millimeter, vector(8.9669, 7.1797) * millimeter, vector(8.9006, 7.2463) * millimeter, vector(8.8333, 7.3128) * millimeter, vector(8.7650, 7.3794) * millimeter, vector(8.6960, 7.4461) * millimeter, vector(8.6263, 7.5129) * millimeter, vector(8.5561, 7.5801) * millimeter, vector(8.4855, 7.6475) * millimeter, vector(8.4147, 7.7154) * millimeter, vector(8.3437, 7.7836) * millimeter, vector(8.2728, 7.8523) * millimeter, vector(8.2020, 7.9215) * millimeter, vector(8.1314, 7.9910) * millimeter, vector(8.0610, 8.0610) * millimeter, vector(7.9910, 8.1314) * millimeter, vector(7.9215, 8.2020) * millimeter, vector(7.8523, 8.2728) * millimeter, vector(7.7836, 8.3437) * millimeter, vector(7.7154, 8.4147) * millimeter, vector(7.6475, 8.4855) * millimeter, vector(7.5801, 8.5561) * millimeter, vector(7.5129, 8.6263) * millimeter, vector(7.4461, 8.6960) * millimeter, vector(7.3794, 8.7650) * millimeter, vector(7.3128, 8.8333) * millimeter, vector(7.2463, 8.9006) * millimeter, vector(7.1797, 8.9669) * millimeter, vector(7.1129, 9.0321) * millimeter, vector(7.0459, 9.0960) * millimeter, vector(6.9785, 9.1585) * millimeter, vector(6.9107, 9.2196) * millimeter, vector(6.8424, 9.2792) * millimeter, vector(6.7734, 9.3373) * millimeter, vector(6.7038, 9.3938) * millimeter, vector(6.6334, 9.4486) * millimeter, vector(6.5622, 9.5018) * millimeter, vector(6.4900, 9.5535) * millimeter, vector(6.4168, 9.6035) * millimeter, vector(6.3426, 9.6519) * millimeter, vector(6.2673, 9.6988) * millimeter, vector(6.1909, 9.7443) * millimeter, vector(6.1133, 9.7884) * millimeter, vector(6.0345, 9.8311) * millimeter, vector(5.9544, 9.8725) * millimeter, vector(5.8732, 9.9129) * millimeter, vector(5.7908, 9.9521) * millimeter, vector(5.7073, 9.9904) * millimeter, vector(5.6226, 10.0279) * millimeter, vector(5.5368, 10.0646) * millimeter, vector(5.4501, 10.1008) * millimeter, vector(5.3624, 10.1365) * millimeter, vector(5.2739, 10.1719) * millimeter, vector(5.1846, 10.2071) * millimeter, vector(5.0946, 10.2422) * millimeter, vector(5.0040, 10.2773) * millimeter, vector(4.9130, 10.3127) * millimeter, vector(4.8216, 10.3482) * millimeter, vector(4.7300, 10.3841) * millimeter, vector(4.6381, 10.4205) * millimeter, vector(4.5462, 10.4572) * millimeter, vector(4.4544, 10.4945) * millimeter, vector(4.3626, 10.5322) * millimeter, vector(4.2710, 10.5704) * millimeter, vector(4.1797, 10.6090) * millimeter, vector(4.0887, 10.6480) * millimeter, vector(3.9981, 10.6873) * millimeter, vector(3.9079, 10.7267) * millimeter, vector(3.8181, 10.7662) * millimeter, vector(3.7288, 10.8056) * millimeter, vector(3.6399, 10.8447) * millimeter, vector(3.5515, 10.8835) * millimeter, vector(3.4634, 10.9218) * millimeter, vector(3.3758, 10.9594) * millimeter, vector(3.2886, 10.9961) * millimeter, vector(3.2016, 11.0319) * millimeter, vector(3.1150, 11.0665) * millimeter, vector(3.0286, 11.0999) * millimeter, vector(2.9425, 11.1319) * millimeter, vector(2.8564, 11.1624) * millimeter, vector(2.7705, 11.1914) * millimeter, vector(2.6846, 11.2186) * millimeter, vector(2.5987, 11.2441) * millimeter, vector(2.5126, 11.2679) * millimeter, vector(2.4264, 11.2898) * millimeter, vector(2.3400, 11.3099) * millimeter, vector(2.2533, 11.3281) * millimeter, vector(2.1662, 11.3444) * millimeter, vector(2.0787, 11.3590) * millimeter, vector(1.9907, 11.3717) * millimeter, vector(1.9021, 11.3827) * millimeter, vector(1.8129, 11.3920) * millimeter, vector(1.7231, 11.3997) * millimeter, vector(1.6327, 11.4059) * millimeter, vector(1.5415, 11.4106) * millimeter, vector(1.4497, 11.4140) * millimeter, vector(1.3571, 11.4162) * millimeter, vector(1.2638, 11.4174) * millimeter, vector(1.1698, 11.4176) * millimeter, vector(1.0751, 11.4170) * millimeter, vector(0.9798, 11.4158) * millimeter, vector(0.8838, 11.4141) * millimeter, vector(0.7873, 11.4122) * millimeter, vector(0.6902, 11.4100) * millimeter, vector(0.5925, 11.4078) * millimeter, vector(0.4945, 11.4057) * millimeter, vector(0.3961, 11.4038) * millimeter, vector(0.2973, 11.4022) * millimeter, vector(0.1984, 11.4010) * millimeter, vector(0.0992, 11.4003) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.0498, 11.4001) * millimeter, vector(-0.0996, 11.4003) * millimeter, vector(-0.1492, 11.4006) * millimeter, vector(-0.1987, 11.4011) * millimeter, vector(-0.2480, 11.4017) * millimeter, vector(-0.2971, 11.4023) * millimeter, vector(-0.3459, 11.4030) * millimeter, vector(-0.3943, 11.4037) * millimeter, vector(-0.4425, 11.4043) * millimeter, vector(-0.4902, 11.4049) * millimeter, vector(-0.5376, 11.4054) * millimeter, vector(-0.5846, 11.4057) * millimeter, vector(-0.6312, 11.4058) * millimeter, vector(-0.6773, 11.4057) * millimeter, vector(-0.7231, 11.4053) * millimeter, vector(-0.7685, 11.4046) * millimeter, vector(-0.8136, 11.4036) * millimeter, vector(-0.8582, 11.4022) * millimeter, vector(-0.9026, 11.4004) * millimeter, vector(-0.9468, 11.3982) * millimeter, vector(-0.9906, 11.3956) * millimeter, vector(-1.0343, 11.3925) * millimeter, vector(-1.0779, 11.3889) * millimeter, vector(-1.1213, 11.3849) * millimeter, vector(-1.1647, 11.3804) * millimeter, vector(-1.2081, 11.3754) * millimeter, vector(-1.2516, 11.3699) * millimeter, vector(-1.2951, 11.3639) * millimeter, vector(-1.3388, 11.3575) * millimeter, vector(-1.3827, 11.3506) * millimeter, vector(-1.4268, 11.3432) * millimeter, vector(-1.4712, 11.3354) * millimeter, vector(-1.5158, 11.3273) * millimeter, vector(-1.5608, 11.3187) * millimeter, vector(-1.6061, 11.3098) * millimeter, vector(-1.6518, 11.3006) * millimeter, vector(-1.6978, 11.2911) * millimeter, vector(-1.7442, 11.2814) * millimeter, vector(-1.7909, 11.2715) * millimeter, vector(-1.8380, 11.2615) * millimeter, vector(-1.8854, 11.2514) * millimeter, vector(-1.9331, 11.2412) * millimeter, vector(-1.9811, 11.2310) * millimeter, vector(-2.0294, 11.2208) * millimeter, vector(-2.0778, 11.2107) * millimeter, vector(-2.1264, 11.2007) * millimeter, vector(-2.1752, 11.1907) * millimeter, vector(-2.2240, 11.1810) * millimeter, vector(-2.2729, 11.1713) * millimeter, vector(-2.3217, 11.1618) * millimeter, vector(-2.3705, 11.1525) * millimeter, vector(-2.4191, 11.1433) * millimeter, vector(-2.4676, 11.1342) * millimeter, vector(-2.5159, 11.1253) * millimeter, vector(-2.5638, 11.1164) * millimeter, vector(-2.6115, 11.1076) * millimeter, vector(-2.6588, 11.0989) * millimeter, vector(-2.7058, 11.0901) * millimeter, vector(-2.7524, 11.0814) * millimeter, vector(-2.7985, 11.0725) * millimeter, vector(-2.8442, 11.0635) * millimeter, vector(-2.8895, 11.0544) * millimeter, vector(-2.9343, 11.0451) * millimeter, vector(-2.9787, 11.0356) * millimeter, vector(-3.0227, 11.0258) * millimeter, vector(-3.0662, 11.0157) * millimeter, vector(-3.1094, 11.0053) * millimeter, vector(-3.1522, 10.9945) * millimeter, vector(-3.1948, 10.9834) * millimeter, vector(-3.2370, 10.9718) * millimeter, vector(-3.2790, 10.9598) * millimeter, vector(-3.3209, 10.9474) * millimeter, vector(-3.3625, 10.9345) * millimeter, vector(-3.4041, 10.9211) * millimeter, vector(-3.4457, 10.9073) * millimeter, vector(-3.4872, 10.8929) * millimeter, vector(-3.5288, 10.8781) * millimeter, vector(-3.5705, 10.8627) * millimeter, vector(-3.6124, 10.8469) * millimeter, vector(-3.6544, 10.8306) * millimeter, vector(-3.6966, 10.8139) * millimeter, vector(-3.7390, 10.7967) * millimeter, vector(-3.7817, 10.7791) * millimeter, vector(-3.8247, 10.7612) * millimeter, vector(-3.8680, 10.7429) * millimeter, vector(-3.9116, 10.7244) * millimeter, vector(-3.9555, 10.7056) * millimeter, vector(-3.9997, 10.6865) * millimeter, vector(-4.0442, 10.6674) * millimeter, vector(-4.0890, 10.6481) * millimeter, vector(-4.1341, 10.6287) * millimeter, vector(-4.1794, 10.6093) * millimeter, vector(-4.2250, 10.5899) * millimeter, vector(-4.2707, 10.5706) * millimeter, vector(-4.3166, 10.5514) * millimeter, vector(-4.3626, 10.5322) * millimeter, vector(-4.4086, 10.5132) * millimeter, vector(-4.4547, 10.4944) * millimeter, vector(-4.5007, 10.4757) * millimeter, vector(-4.5466, 10.4572) * millimeter, vector(-4.5924, 10.4389) * millimeter, vector(-4.6379, 10.4207) * millimeter, vector(-4.6833, 10.4026) * millimeter, vector(-4.7283, 10.3847) * millimeter, vector(-4.7730, 10.3669) * millimeter, vector(-4.8174, 10.3492) * millimeter, vector(-4.8613, 10.3315) * millimeter, vector(-4.9049, 10.3138) * millimeter, vector(-4.9479, 10.2961) * millimeter, vector(-4.9906, 10.2783) * millimeter, vector(-5.0327, 10.2604) * millimeter, vector(-5.0744, 10.2424) * millimeter, vector(-5.1156, 10.2242) * millimeter, vector(-5.1564, 10.2058) * millimeter, vector(-5.1967, 10.1872) * millimeter, vector(-5.2366, 10.1683) * millimeter, vector(-5.2761, 10.1491) * millimeter, vector(-5.3153, 10.1295) * millimeter, vector(-5.3542, 10.1095) * millimeter, vector(-5.3928, 10.0892) * millimeter, vector(-5.4311, 10.0684) * millimeter, vector(-5.4693, 10.0472) * millimeter, vector(-5.5074, 10.0255) * millimeter, vector(-5.5453, 10.0033) * millimeter, vector(-5.5832, 9.9806) * millimeter, vector(-5.6211, 9.9574) * millimeter, vector(-5.6591, 9.9337) * millimeter, vector(-5.6971, 9.9096) * millimeter, vector(-5.7352, 9.8849) * millimeter, vector(-5.7735, 9.8598) * millimeter, vector(-5.8119, 9.8343) * millimeter, vector(-5.8506, 9.8083) * millimeter, vector(-5.8895, 9.7819) * millimeter, vector(-5.9286, 9.7552) * millimeter, vector(-5.9680, 9.7282) * millimeter, vector(-6.0077, 9.7009) * millimeter, vector(-6.0476, 9.6734) * millimeter, vector(-6.0878, 9.6457) * millimeter, vector(-6.1282, 9.6179) * millimeter, vector(-6.1689, 9.5901) * millimeter, vector(-6.2098, 9.5622) * millimeter, vector(-6.2509, 9.5343) * millimeter, vector(-6.2921, 9.5065) * millimeter, vector(-6.3335, 9.4788) * millimeter, vector(-6.3750, 9.4511) * millimeter, vector(-6.4164, 9.4237) * millimeter, vector(-6.4579, 9.3964) * millimeter, vector(-6.4993, 9.3693) * millimeter, vector(-6.5407, 9.3424) * millimeter, vector(-6.5818, 9.3156) * millimeter, vector(-6.6227, 9.2891) * millimeter, vector(-6.6634, 9.2627) * millimeter, vector(-6.7038, 9.2365) * millimeter, vector(-6.7438, 9.2105) * millimeter, vector(-6.7835, 9.1846) * millimeter, vector(-6.8227, 9.1587) * millimeter, vector(-6.8615, 9.1329) * millimeter, vector(-6.8999, 9.1072) * millimeter, vector(-6.9377, 9.0814) * millimeter, vector(-6.9751, 9.0556) * millimeter, vector(-7.0120, 9.0298) * millimeter, vector(-7.0483, 9.0038) * millimeter, vector(-7.0843, 8.9776) * millimeter, vector(-7.1197, 8.9513) * millimeter, vector(-7.1547, 8.9247) * millimeter, vector(-7.1893, 8.8979) * millimeter, vector(-7.2236, 8.8707) * millimeter, vector(-7.2575, 8.8432) * millimeter, vector(-7.2910, 8.8154) * millimeter, vector(-7.3243, 8.7871) * millimeter, vector(-7.3574, 8.7584) * millimeter, vector(-7.3903, 8.7292) * millimeter, vector(-7.4231, 8.6996) * millimeter, vector(-7.4557, 8.6695) * millimeter, vector(-7.4883, 8.6388) * millimeter, vector(-7.5209, 8.6077) * millimeter, vector(-7.5535, 8.5761) * millimeter, vector(-7.5861, 8.5440) * millimeter, vector(-7.6188, 8.5114) * millimeter, vector(-7.6517, 8.4784) * millimeter, vector(-7.6847, 8.4450) * millimeter, vector(-7.7179, 8.4111) * millimeter, vector(-7.7512, 8.3770) * millimeter, vector(-7.7848, 8.3425) * millimeter, vector(-7.8186, 8.3077) * millimeter, vector(-7.8526, 8.2727) * millimeter, vector(-7.8868, 8.2376) * millimeter, vector(-7.9213, 8.2023) * millimeter, vector(-7.9560, 8.1670) * millimeter, vector(-7.9908, 8.1316) * millimeter, vector(-8.0259, 8.0963) * millimeter, vector(-8.0610, 8.0610) * millimeter, vector(-8.0963, 8.0259) * millimeter, vector(-8.1316, 7.9908) * millimeter, vector(-8.1670, 7.9560) * millimeter, vector(-8.2023, 7.9213) * millimeter, vector(-8.2376, 7.8868) * millimeter, vector(-8.2727, 7.8526) * millimeter, vector(-8.3077, 7.8186) * millimeter, vector(-8.3425, 7.7848) * millimeter, vector(-8.3770, 7.7512) * millimeter, vector(-8.4111, 7.7179) * millimeter, vector(-8.4450, 7.6847) * millimeter, vector(-8.4784, 7.6517) * millimeter, vector(-8.5114, 7.6188) * millimeter, vector(-8.5440, 7.5861) * millimeter, vector(-8.5761, 7.5535) * millimeter, vector(-8.6077, 7.5209) * millimeter, vector(-8.6388, 7.4883) * millimeter, vector(-8.6695, 7.4557) * millimeter, vector(-8.6996, 7.4231) * millimeter, vector(-8.7292, 7.3903) * millimeter, vector(-8.7584, 7.3574) * millimeter, vector(-8.7871, 7.3243) * millimeter, vector(-8.8154, 7.2910) * millimeter, vector(-8.8432, 7.2575) * millimeter, vector(-8.8707, 7.2236) * millimeter, vector(-8.8979, 7.1893) * millimeter, vector(-8.9247, 7.1547) * millimeter, vector(-8.9513, 7.1197) * millimeter, vector(-8.9776, 7.0843) * millimeter, vector(-9.0038, 7.0483) * millimeter, vector(-9.0298, 7.0120) * millimeter, vector(-9.0556, 6.9751) * millimeter, vector(-9.0814, 6.9377) * millimeter, vector(-9.1072, 6.8999) * millimeter, vector(-9.1329, 6.8615) * millimeter, vector(-9.1587, 6.8227) * millimeter, vector(-9.1846, 6.7835) * millimeter, vector(-9.2105, 6.7438) * millimeter, vector(-9.2365, 6.7038) * millimeter, vector(-9.2627, 6.6634) * millimeter, vector(-9.2891, 6.6227) * millimeter, vector(-9.3156, 6.5818) * millimeter, vector(-9.3424, 6.5407) * millimeter, vector(-9.3693, 6.4993) * millimeter, vector(-9.3964, 6.4579) * millimeter, vector(-9.4237, 6.4164) * millimeter, vector(-9.4511, 6.3750) * millimeter, vector(-9.4788, 6.3335) * millimeter, vector(-9.5065, 6.2921) * millimeter, vector(-9.5343, 6.2509) * millimeter, vector(-9.5622, 6.2098) * millimeter, vector(-9.5901, 6.1689) * millimeter, vector(-9.6179, 6.1282) * millimeter, vector(-9.6457, 6.0878) * millimeter, vector(-9.6734, 6.0476) * millimeter, vector(-9.7009, 6.0077) * millimeter, vector(-9.7282, 5.9680) * millimeter, vector(-9.7552, 5.9286) * millimeter, vector(-9.7819, 5.8895) * millimeter, vector(-9.8083, 5.8506) * millimeter, vector(-9.8343, 5.8119) * millimeter, vector(-9.8598, 5.7735) * millimeter, vector(-9.8849, 5.7352) * millimeter, vector(-9.9096, 5.6971) * millimeter, vector(-9.9337, 5.6591) * millimeter, vector(-9.9574, 5.6211) * millimeter, vector(-9.9806, 5.5832) * millimeter, vector(-10.0033, 5.5453) * millimeter, vector(-10.0255, 5.5074) * millimeter, vector(-10.0472, 5.4693) * millimeter, vector(-10.0684, 5.4311) * millimeter, vector(-10.0892, 5.3928) * millimeter, vector(-10.1095, 5.3542) * millimeter, vector(-10.1295, 5.3153) * millimeter, vector(-10.1491, 5.2761) * millimeter, vector(-10.1683, 5.2366) * millimeter, vector(-10.1872, 5.1967) * millimeter, vector(-10.2058, 5.1564) * millimeter, vector(-10.2242, 5.1156) * millimeter, vector(-10.2424, 5.0744) * millimeter, vector(-10.2604, 5.0327) * millimeter, vector(-10.2783, 4.9906) * millimeter, vector(-10.2961, 4.9479) * millimeter, vector(-10.3138, 4.9049) * millimeter, vector(-10.3315, 4.8613) * millimeter, vector(-10.3492, 4.8174) * millimeter, vector(-10.3669, 4.7730) * millimeter, vector(-10.3847, 4.7283) * millimeter, vector(-10.4026, 4.6833) * millimeter, vector(-10.4207, 4.6379) * millimeter, vector(-10.4389, 4.5924) * millimeter, vector(-10.4572, 4.5466) * millimeter, vector(-10.4757, 4.5007) * millimeter, vector(-10.4944, 4.4547) * millimeter, vector(-10.5132, 4.4086) * millimeter, vector(-10.5322, 4.3626) * millimeter, vector(-10.5514, 4.3166) * millimeter, vector(-10.5706, 4.2707) * millimeter, vector(-10.5899, 4.2250) * millimeter, vector(-10.6093, 4.1794) * millimeter, vector(-10.6287, 4.1341) * millimeter, vector(-10.6481, 4.0890) * millimeter, vector(-10.6674, 4.0442) * millimeter, vector(-10.6865, 3.9997) * millimeter, vector(-10.7056, 3.9555) * millimeter, vector(-10.7244, 3.9116) * millimeter, vector(-10.7429, 3.8680) * millimeter, vector(-10.7612, 3.8247) * millimeter, vector(-10.7791, 3.7817) * millimeter, vector(-10.7967, 3.7390) * millimeter, vector(-10.8139, 3.6966) * millimeter, vector(-10.8306, 3.6544) * millimeter, vector(-10.8469, 3.6124) * millimeter, vector(-10.8627, 3.5705) * millimeter, vector(-10.8781, 3.5288) * millimeter, vector(-10.8929, 3.4872) * millimeter, vector(-10.9073, 3.4457) * millimeter, vector(-10.9211, 3.4041) * millimeter, vector(-10.9345, 3.3625) * millimeter, vector(-10.9474, 3.3209) * millimeter, vector(-10.9598, 3.2790) * millimeter, vector(-10.9718, 3.2370) * millimeter, vector(-10.9834, 3.1948) * millimeter, vector(-10.9945, 3.1522) * millimeter, vector(-11.0053, 3.1094) * millimeter, vector(-11.0157, 3.0662) * millimeter, vector(-11.0258, 3.0227) * millimeter, vector(-11.0356, 2.9787) * millimeter, vector(-11.0451, 2.9343) * millimeter, vector(-11.0544, 2.8895) * millimeter, vector(-11.0635, 2.8442) * millimeter, vector(-11.0725, 2.7985) * millimeter, vector(-11.0814, 2.7524) * millimeter, vector(-11.0901, 2.7058) * millimeter, vector(-11.0989, 2.6588) * millimeter, vector(-11.1076, 2.6115) * millimeter, vector(-11.1164, 2.5638) * millimeter, vector(-11.1253, 2.5159) * millimeter, vector(-11.1342, 2.4676) * millimeter, vector(-11.1433, 2.4191) * millimeter, vector(-11.1525, 2.3705) * millimeter, vector(-11.1618, 2.3217) * millimeter, vector(-11.1713, 2.2729) * millimeter, vector(-11.1810, 2.2240) * millimeter, vector(-11.1907, 2.1752) * millimeter, vector(-11.2007, 2.1264) * millimeter, vector(-11.2107, 2.0778) * millimeter, vector(-11.2208, 2.0294) * millimeter, vector(-11.2310, 1.9811) * millimeter, vector(-11.2412, 1.9331) * millimeter, vector(-11.2514, 1.8854) * millimeter, vector(-11.2615, 1.8380) * millimeter, vector(-11.2715, 1.7909) * millimeter, vector(-11.2814, 1.7442) * millimeter, vector(-11.2911, 1.6978) * millimeter, vector(-11.3006, 1.6518) * millimeter, vector(-11.3098, 1.6061) * millimeter, vector(-11.3187, 1.5608) * millimeter, vector(-11.3273, 1.5158) * millimeter, vector(-11.3354, 1.4712) * millimeter, vector(-11.3432, 1.4268) * millimeter, vector(-11.3506, 1.3827) * millimeter, vector(-11.3575, 1.3388) * millimeter, vector(-11.3639, 1.2951) * millimeter, vector(-11.3699, 1.2516) * millimeter, vector(-11.3754, 1.2081) * millimeter, vector(-11.3804, 1.1647) * millimeter, vector(-11.3849, 1.1213) * millimeter, vector(-11.3889, 1.0779) * millimeter, vector(-11.3925, 1.0343) * millimeter, vector(-11.3956, 0.9906) * millimeter, vector(-11.3982, 0.9468) * millimeter, vector(-11.4004, 0.9026) * millimeter, vector(-11.4022, 0.8582) * millimeter, vector(-11.4036, 0.8136) * millimeter, vector(-11.4046, 0.7685) * millimeter, vector(-11.4053, 0.7231) * millimeter, vector(-11.4057, 0.6773) * millimeter, vector(-11.4058, 0.6312) * millimeter, vector(-11.4057, 0.5846) * millimeter, vector(-11.4054, 0.5376) * millimeter, vector(-11.4049, 0.4902) * millimeter, vector(-11.4043, 0.4425) * millimeter, vector(-11.4037, 0.3943) * millimeter, vector(-11.4030, 0.3459) * millimeter, vector(-11.4023, 0.2971) * millimeter, vector(-11.4017, 0.2480) * millimeter, vector(-11.4011, 0.1987) * millimeter, vector(-11.4006, 0.1492) * millimeter, vector(-11.4003, 0.0996) * millimeter, vector(-11.4001, 0.0498) * millimeter, vector(-11.4000, 0.0000) * millimeter, vector(-11.4001, -0.0498) * millimeter, vector(-11.4003, -0.0996) * millimeter, vector(-11.4006, -0.1492) * millimeter, vector(-11.4011, -0.1987) * millimeter, vector(-11.4017, -0.2480) * millimeter, vector(-11.4023, -0.2971) * millimeter, vector(-11.4030, -0.3459) * millimeter, vector(-11.4037, -0.3943) * millimeter, vector(-11.4043, -0.4425) * millimeter, vector(-11.4049, -0.4902) * millimeter, vector(-11.4054, -0.5376) * millimeter, vector(-11.4057, -0.5846) * millimeter, vector(-11.4058, -0.6312) * millimeter, vector(-11.4057, -0.6773) * millimeter, vector(-11.4053, -0.7231) * millimeter, vector(-11.4046, -0.7685) * millimeter, vector(-11.4036, -0.8136) * millimeter, vector(-11.4022, -0.8582) * millimeter, vector(-11.4004, -0.9026) * millimeter, vector(-11.3982, -0.9468) * millimeter, vector(-11.3956, -0.9906) * millimeter, vector(-11.3925, -1.0343) * millimeter, vector(-11.3889, -1.0779) * millimeter, vector(-11.3849, -1.1213) * millimeter, vector(-11.3804, -1.1647) * millimeter, vector(-11.3754, -1.2081) * millimeter, vector(-11.3699, -1.2516) * millimeter, vector(-11.3639, -1.2951) * millimeter, vector(-11.3575, -1.3388) * millimeter, vector(-11.3506, -1.3827) * millimeter, vector(-11.3432, -1.4268) * millimeter, vector(-11.3354, -1.4712) * millimeter, vector(-11.3273, -1.5158) * millimeter, vector(-11.3187, -1.5608) * millimeter, vector(-11.3098, -1.6061) * millimeter, vector(-11.3006, -1.6518) * millimeter, vector(-11.2911, -1.6978) * millimeter, vector(-11.2814, -1.7442) * millimeter, vector(-11.2715, -1.7909) * millimeter, vector(-11.2615, -1.8380) * millimeter, vector(-11.2514, -1.8854) * millimeter, vector(-11.2412, -1.9331) * millimeter, vector(-11.2310, -1.9811) * millimeter, vector(-11.2208, -2.0294) * millimeter, vector(-11.2107, -2.0778) * millimeter, vector(-11.2007, -2.1264) * millimeter, vector(-11.1907, -2.1752) * millimeter, vector(-11.1810, -2.2240) * millimeter, vector(-11.1713, -2.2729) * millimeter, vector(-11.1618, -2.3217) * millimeter, vector(-11.1525, -2.3705) * millimeter, vector(-11.1433, -2.4191) * millimeter, vector(-11.1342, -2.4676) * millimeter, vector(-11.1253, -2.5159) * millimeter, vector(-11.1164, -2.5638) * millimeter, vector(-11.1076, -2.6115) * millimeter, vector(-11.0989, -2.6588) * millimeter, vector(-11.0901, -2.7058) * millimeter, vector(-11.0814, -2.7524) * millimeter, vector(-11.0725, -2.7985) * millimeter, vector(-11.0635, -2.8442) * millimeter, vector(-11.0544, -2.8895) * millimeter, vector(-11.0451, -2.9343) * millimeter, vector(-11.0356, -2.9787) * millimeter, vector(-11.0258, -3.0227) * millimeter, vector(-11.0157, -3.0662) * millimeter, vector(-11.0053, -3.1094) * millimeter, vector(-10.9945, -3.1522) * millimeter, vector(-10.9834, -3.1948) * millimeter, vector(-10.9718, -3.2370) * millimeter, vector(-10.9598, -3.2790) * millimeter, vector(-10.9474, -3.3209) * millimeter, vector(-10.9345, -3.3625) * millimeter, vector(-10.9211, -3.4041) * millimeter, vector(-10.9073, -3.4457) * millimeter, vector(-10.8929, -3.4872) * millimeter, vector(-10.8781, -3.5288) * millimeter, vector(-10.8627, -3.5705) * millimeter, vector(-10.8469, -3.6124) * millimeter, vector(-10.8306, -3.6544) * millimeter, vector(-10.8139, -3.6966) * millimeter, vector(-10.7967, -3.7390) * millimeter, vector(-10.7791, -3.7817) * millimeter, vector(-10.7612, -3.8247) * millimeter, vector(-10.7429, -3.8680) * millimeter, vector(-10.7244, -3.9116) * millimeter, vector(-10.7056, -3.9555) * millimeter, vector(-10.6865, -3.9997) * millimeter, vector(-10.6674, -4.0442) * millimeter, vector(-10.6481, -4.0890) * millimeter, vector(-10.6287, -4.1341) * millimeter, vector(-10.6093, -4.1794) * millimeter, vector(-10.5899, -4.2250) * millimeter, vector(-10.5706, -4.2707) * millimeter, vector(-10.5514, -4.3166) * millimeter, vector(-10.5322, -4.3626) * millimeter, vector(-10.5132, -4.4086) * millimeter, vector(-10.4944, -4.4547) * millimeter, vector(-10.4757, -4.5007) * millimeter, vector(-10.4572, -4.5466) * millimeter, vector(-10.4389, -4.5924) * millimeter, vector(-10.4207, -4.6379) * millimeter, vector(-10.4026, -4.6833) * millimeter, vector(-10.3847, -4.7283) * millimeter, vector(-10.3669, -4.7730) * millimeter, vector(-10.3492, -4.8174) * millimeter, vector(-10.3315, -4.8613) * millimeter, vector(-10.3138, -4.9049) * millimeter, vector(-10.2961, -4.9479) * millimeter, vector(-10.2783, -4.9906) * millimeter, vector(-10.2604, -5.0327) * millimeter, vector(-10.2424, -5.0744) * millimeter, vector(-10.2242, -5.1156) * millimeter, vector(-10.2058, -5.1564) * millimeter, vector(-10.1872, -5.1967) * millimeter, vector(-10.1683, -5.2366) * millimeter, vector(-10.1491, -5.2761) * millimeter, vector(-10.1295, -5.3153) * millimeter, vector(-10.1095, -5.3542) * millimeter, vector(-10.0892, -5.3928) * millimeter, vector(-10.0684, -5.4311) * millimeter, vector(-10.0472, -5.4693) * millimeter, vector(-10.0255, -5.5074) * millimeter, vector(-10.0033, -5.5453) * millimeter, vector(-9.9806, -5.5832) * millimeter, vector(-9.9574, -5.6211) * millimeter, vector(-9.9337, -5.6591) * millimeter, vector(-9.9096, -5.6971) * millimeter, vector(-9.8849, -5.7352) * millimeter, vector(-9.8598, -5.7735) * millimeter, vector(-9.8343, -5.8119) * millimeter, vector(-9.8083, -5.8506) * millimeter, vector(-9.7819, -5.8895) * millimeter, vector(-9.7552, -5.9286) * millimeter, vector(-9.7282, -5.9680) * millimeter, vector(-9.7009, -6.0077) * millimeter, vector(-9.6734, -6.0476) * millimeter, vector(-9.6457, -6.0878) * millimeter, vector(-9.6179, -6.1282) * millimeter, vector(-9.5901, -6.1689) * millimeter, vector(-9.5622, -6.2098) * millimeter, vector(-9.5343, -6.2509) * millimeter, vector(-9.5065, -6.2921) * millimeter, vector(-9.4788, -6.3335) * millimeter, vector(-9.4511, -6.3750) * millimeter, vector(-9.4237, -6.4164) * millimeter, vector(-9.3964, -6.4579) * millimeter, vector(-9.3693, -6.4993) * millimeter, vector(-9.3424, -6.5407) * millimeter, vector(-9.3156, -6.5818) * millimeter, vector(-9.2891, -6.6227) * millimeter, vector(-9.2627, -6.6634) * millimeter, vector(-9.2365, -6.7038) * millimeter, vector(-9.2105, -6.7438) * millimeter, vector(-9.1846, -6.7835) * millimeter, vector(-9.1587, -6.8227) * millimeter, vector(-9.1329, -6.8615) * millimeter, vector(-9.1072, -6.8999) * millimeter, vector(-9.0814, -6.9377) * millimeter, vector(-9.0556, -6.9751) * millimeter, vector(-9.0298, -7.0120) * millimeter, vector(-9.0038, -7.0483) * millimeter, vector(-8.9776, -7.0843) * millimeter, vector(-8.9513, -7.1197) * millimeter, vector(-8.9247, -7.1547) * millimeter, vector(-8.8979, -7.1893) * millimeter, vector(-8.8707, -7.2236) * millimeter, vector(-8.8432, -7.2575) * millimeter, vector(-8.8154, -7.2910) * millimeter, vector(-8.7871, -7.3243) * millimeter, vector(-8.7584, -7.3574) * millimeter, vector(-8.7292, -7.3903) * millimeter, vector(-8.6996, -7.4231) * millimeter, vector(-8.6695, -7.4557) * millimeter, vector(-8.6388, -7.4883) * millimeter, vector(-8.6077, -7.5209) * millimeter, vector(-8.5761, -7.5535) * millimeter, vector(-8.5440, -7.5861) * millimeter, vector(-8.5114, -7.6188) * millimeter, vector(-8.4784, -7.6517) * millimeter, vector(-8.4450, -7.6847) * millimeter, vector(-8.4111, -7.7179) * millimeter, vector(-8.3770, -7.7512) * millimeter, vector(-8.3425, -7.7848) * millimeter, vector(-8.3077, -7.8186) * millimeter, vector(-8.2727, -7.8526) * millimeter, vector(-8.2376, -7.8868) * millimeter, vector(-8.2023, -7.9213) * millimeter, vector(-8.1670, -7.9560) * millimeter, vector(-8.1316, -7.9908) * millimeter, vector(-8.0963, -8.0259) * millimeter, vector(-8.0610, -8.0610) * millimeter, vector(-8.0259, -8.0963) * millimeter, vector(-7.9908, -8.1316) * millimeter, vector(-7.9560, -8.1670) * millimeter, vector(-7.9213, -8.2023) * millimeter, vector(-7.8868, -8.2376) * millimeter, vector(-7.8526, -8.2727) * millimeter, vector(-7.8186, -8.3077) * millimeter, vector(-7.7848, -8.3425) * millimeter, vector(-7.7512, -8.3770) * millimeter, vector(-7.7179, -8.4111) * millimeter, vector(-7.6847, -8.4450) * millimeter, vector(-7.6517, -8.4784) * millimeter, vector(-7.6188, -8.5114) * millimeter, vector(-7.5861, -8.5440) * millimeter, vector(-7.5535, -8.5761) * millimeter, vector(-7.5209, -8.6077) * millimeter, vector(-7.4883, -8.6388) * millimeter, vector(-7.4557, -8.6695) * millimeter, vector(-7.4231, -8.6996) * millimeter, vector(-7.3903, -8.7292) * millimeter, vector(-7.3574, -8.7584) * millimeter, vector(-7.3243, -8.7871) * millimeter, vector(-7.2910, -8.8154) * millimeter, vector(-7.2575, -8.8432) * millimeter, vector(-7.2236, -8.8707) * millimeter, vector(-7.1893, -8.8979) * millimeter, vector(-7.1547, -8.9247) * millimeter, vector(-7.1197, -8.9513) * millimeter, vector(-7.0843, -8.9776) * millimeter, vector(-7.0483, -9.0038) * millimeter, vector(-7.0120, -9.0298) * millimeter, vector(-6.9751, -9.0556) * millimeter, vector(-6.9377, -9.0814) * millimeter, vector(-6.8999, -9.1072) * millimeter, vector(-6.8615, -9.1329) * millimeter, vector(-6.8227, -9.1587) * millimeter, vector(-6.7835, -9.1846) * millimeter, vector(-6.7438, -9.2105) * millimeter, vector(-6.7038, -9.2365) * millimeter, vector(-6.6634, -9.2627) * millimeter, vector(-6.6227, -9.2891) * millimeter, vector(-6.5818, -9.3156) * millimeter, vector(-6.5407, -9.3424) * millimeter, vector(-6.4993, -9.3693) * millimeter, vector(-6.4579, -9.3964) * millimeter, vector(-6.4164, -9.4237) * millimeter, vector(-6.3750, -9.4511) * millimeter, vector(-6.3335, -9.4788) * millimeter, vector(-6.2921, -9.5065) * millimeter, vector(-6.2509, -9.5343) * millimeter, vector(-6.2098, -9.5622) * millimeter, vector(-6.1689, -9.5901) * millimeter, vector(-6.1282, -9.6179) * millimeter, vector(-6.0878, -9.6457) * millimeter, vector(-6.0476, -9.6734) * millimeter, vector(-6.0077, -9.7009) * millimeter, vector(-5.9680, -9.7282) * millimeter, vector(-5.9286, -9.7552) * millimeter, vector(-5.8895, -9.7819) * millimeter, vector(-5.8506, -9.8083) * millimeter, vector(-5.8119, -9.8343) * millimeter, vector(-5.7735, -9.8598) * millimeter, vector(-5.7352, -9.8849) * millimeter, vector(-5.6971, -9.9096) * millimeter, vector(-5.6591, -9.9337) * millimeter, vector(-5.6211, -9.9574) * millimeter, vector(-5.5832, -9.9806) * millimeter, vector(-5.5453, -10.0033) * millimeter, vector(-5.5074, -10.0255) * millimeter, vector(-5.4693, -10.0472) * millimeter, vector(-5.4311, -10.0684) * millimeter, vector(-5.3928, -10.0892) * millimeter, vector(-5.3542, -10.1095) * millimeter, vector(-5.3153, -10.1295) * millimeter, vector(-5.2761, -10.1491) * millimeter, vector(-5.2366, -10.1683) * millimeter, vector(-5.1967, -10.1872) * millimeter, vector(-5.1564, -10.2058) * millimeter, vector(-5.1156, -10.2242) * millimeter, vector(-5.0744, -10.2424) * millimeter, vector(-5.0327, -10.2604) * millimeter, vector(-4.9906, -10.2783) * millimeter, vector(-4.9479, -10.2961) * millimeter, vector(-4.9049, -10.3138) * millimeter, vector(-4.8613, -10.3315) * millimeter, vector(-4.8174, -10.3492) * millimeter, vector(-4.7730, -10.3669) * millimeter, vector(-4.7283, -10.3847) * millimeter, vector(-4.6833, -10.4026) * millimeter, vector(-4.6379, -10.4207) * millimeter, vector(-4.5924, -10.4389) * millimeter, vector(-4.5466, -10.4572) * millimeter, vector(-4.5007, -10.4757) * millimeter, vector(-4.4547, -10.4944) * millimeter, vector(-4.4086, -10.5132) * millimeter, vector(-4.3626, -10.5322) * millimeter, vector(-4.3166, -10.5514) * millimeter, vector(-4.2707, -10.5706) * millimeter, vector(-4.2250, -10.5899) * millimeter, vector(-4.1794, -10.6093) * millimeter, vector(-4.1341, -10.6287) * millimeter, vector(-4.0890, -10.6481) * millimeter, vector(-4.0442, -10.6674) * millimeter, vector(-3.9997, -10.6865) * millimeter, vector(-3.9555, -10.7056) * millimeter, vector(-3.9116, -10.7244) * millimeter, vector(-3.8680, -10.7429) * millimeter, vector(-3.8247, -10.7612) * millimeter, vector(-3.7817, -10.7791) * millimeter, vector(-3.7390, -10.7967) * millimeter, vector(-3.6966, -10.8139) * millimeter, vector(-3.6544, -10.8306) * millimeter, vector(-3.6124, -10.8469) * millimeter, vector(-3.5705, -10.8627) * millimeter, vector(-3.5288, -10.8781) * millimeter, vector(-3.4872, -10.8929) * millimeter, vector(-3.4457, -10.9073) * millimeter, vector(-3.4041, -10.9211) * millimeter, vector(-3.3625, -10.9345) * millimeter, vector(-3.3209, -10.9474) * millimeter, vector(-3.2790, -10.9598) * millimeter, vector(-3.2370, -10.9718) * millimeter, vector(-3.1948, -10.9834) * millimeter, vector(-3.1522, -10.9945) * millimeter, vector(-3.1094, -11.0053) * millimeter, vector(-3.0662, -11.0157) * millimeter, vector(-3.0227, -11.0258) * millimeter, vector(-2.9787, -11.0356) * millimeter, vector(-2.9343, -11.0451) * millimeter, vector(-2.8895, -11.0544) * millimeter, vector(-2.8442, -11.0635) * millimeter, vector(-2.7985, -11.0725) * millimeter, vector(-2.7524, -11.0814) * millimeter, vector(-2.7058, -11.0901) * millimeter, vector(-2.6588, -11.0989) * millimeter, vector(-2.6115, -11.1076) * millimeter, vector(-2.5638, -11.1164) * millimeter, vector(-2.5159, -11.1253) * millimeter, vector(-2.4676, -11.1342) * millimeter, vector(-2.4191, -11.1433) * millimeter, vector(-2.3705, -11.1525) * millimeter, vector(-2.3217, -11.1618) * millimeter, vector(-2.2729, -11.1713) * millimeter, vector(-2.2240, -11.1810) * millimeter, vector(-2.1752, -11.1907) * millimeter, vector(-2.1264, -11.2007) * millimeter, vector(-2.0778, -11.2107) * millimeter, vector(-2.0294, -11.2208) * millimeter, vector(-1.9811, -11.2310) * millimeter, vector(-1.9331, -11.2412) * millimeter, vector(-1.8854, -11.2514) * millimeter, vector(-1.8380, -11.2615) * millimeter, vector(-1.7909, -11.2715) * millimeter, vector(-1.7442, -11.2814) * millimeter, vector(-1.6978, -11.2911) * millimeter, vector(-1.6518, -11.3006) * millimeter, vector(-1.6061, -11.3098) * millimeter, vector(-1.5608, -11.3187) * millimeter, vector(-1.5158, -11.3273) * millimeter, vector(-1.4712, -11.3354) * millimeter, vector(-1.4268, -11.3432) * millimeter, vector(-1.3827, -11.3506) * millimeter, vector(-1.3388, -11.3575) * millimeter, vector(-1.2951, -11.3639) * millimeter, vector(-1.2516, -11.3699) * millimeter, vector(-1.2081, -11.3754) * millimeter, vector(-1.1647, -11.3804) * millimeter, vector(-1.1213, -11.3849) * millimeter, vector(-1.0779, -11.3889) * millimeter, vector(-1.0343, -11.3925) * millimeter, vector(-0.9906, -11.3956) * millimeter, vector(-0.9468, -11.3982) * millimeter, vector(-0.9026, -11.4004) * millimeter, vector(-0.8582, -11.4022) * millimeter, vector(-0.8136, -11.4036) * millimeter, vector(-0.7685, -11.4046) * millimeter, vector(-0.7231, -11.4053) * millimeter, vector(-0.6773, -11.4057) * millimeter, vector(-0.6312, -11.4058) * millimeter, vector(-0.5846, -11.4057) * millimeter, vector(-0.5376, -11.4054) * millimeter, vector(-0.4902, -11.4049) * millimeter, vector(-0.4425, -11.4043) * millimeter, vector(-0.3943, -11.4037) * millimeter, vector(-0.3459, -11.4030) * millimeter, vector(-0.2971, -11.4023) * millimeter, vector(-0.2480, -11.4017) * millimeter, vector(-0.1987, -11.4011) * millimeter, vector(-0.1492, -11.4006) * millimeter, vector(-0.0996, -11.4003) * millimeter, vector(-0.0498, -11.4001) * millimeter, vector(-0.0000, -11.4000) * millimeter, vector(0.0498, -11.4001) * millimeter, vector(0.0996, -11.4003) * millimeter, vector(0.1492, -11.4006) * millimeter, vector(0.1987, -11.4011) * millimeter, vector(0.2480, -11.4017) * millimeter, vector(0.2971, -11.4023) * millimeter, vector(0.3459, -11.4030) * millimeter, vector(0.3943, -11.4037) * millimeter, vector(0.4425, -11.4043) * millimeter, vector(0.4902, -11.4049) * millimeter, vector(0.5376, -11.4054) * millimeter, vector(0.5846, -11.4057) * millimeter, vector(0.6312, -11.4058) * millimeter, vector(0.6773, -11.4057) * millimeter, vector(0.7231, -11.4053) * millimeter, vector(0.7685, -11.4046) * millimeter, vector(0.8136, -11.4036) * millimeter, vector(0.8582, -11.4022) * millimeter, vector(0.9026, -11.4004) * millimeter, vector(0.9468, -11.3982) * millimeter, vector(0.9906, -11.3956) * millimeter, vector(1.0343, -11.3925) * millimeter, vector(1.0779, -11.3889) * millimeter, vector(1.1213, -11.3849) * millimeter, vector(1.1647, -11.3804) * millimeter, vector(1.2081, -11.3754) * millimeter, vector(1.2516, -11.3699) * millimeter, vector(1.2951, -11.3639) * millimeter, vector(1.3388, -11.3575) * millimeter, vector(1.3827, -11.3506) * millimeter, vector(1.4268, -11.3432) * millimeter, vector(1.4712, -11.3354) * millimeter, vector(1.5158, -11.3273) * millimeter, vector(1.5608, -11.3187) * millimeter, vector(1.6061, -11.3098) * millimeter, vector(1.6518, -11.3006) * millimeter, vector(1.6978, -11.2911) * millimeter, vector(1.7442, -11.2814) * millimeter, vector(1.7909, -11.2715) * millimeter, vector(1.8380, -11.2615) * millimeter, vector(1.8854, -11.2514) * millimeter, vector(1.9331, -11.2412) * millimeter, vector(1.9811, -11.2310) * millimeter, vector(2.0294, -11.2208) * millimeter, vector(2.0778, -11.2107) * millimeter, vector(2.1264, -11.2007) * millimeter, vector(2.1752, -11.1907) * millimeter, vector(2.2240, -11.1810) * millimeter, vector(2.2729, -11.1713) * millimeter, vector(2.3217, -11.1618) * millimeter, vector(2.3705, -11.1525) * millimeter, vector(2.4191, -11.1433) * millimeter, vector(2.4676, -11.1342) * millimeter, vector(2.5159, -11.1253) * millimeter, vector(2.5638, -11.1164) * millimeter, vector(2.6115, -11.1076) * millimeter, vector(2.6588, -11.0989) * millimeter, vector(2.7058, -11.0901) * millimeter, vector(2.7524, -11.0814) * millimeter, vector(2.7985, -11.0725) * millimeter, vector(2.8442, -11.0635) * millimeter, vector(2.8895, -11.0544) * millimeter, vector(2.9343, -11.0451) * millimeter, vector(2.9787, -11.0356) * millimeter, vector(3.0227, -11.0258) * millimeter, vector(3.0662, -11.0157) * millimeter, vector(3.1094, -11.0053) * millimeter, vector(3.1522, -10.9945) * millimeter, vector(3.1948, -10.9834) * millimeter, vector(3.2370, -10.9718) * millimeter, vector(3.2790, -10.9598) * millimeter, vector(3.3209, -10.9474) * millimeter, vector(3.3625, -10.9345) * millimeter, vector(3.4041, -10.9211) * millimeter, vector(3.4457, -10.9073) * millimeter, vector(3.4872, -10.8929) * millimeter, vector(3.5288, -10.8781) * millimeter, vector(3.5705, -10.8627) * millimeter, vector(3.6124, -10.8469) * millimeter, vector(3.6544, -10.8306) * millimeter, vector(3.6966, -10.8139) * millimeter, vector(3.7390, -10.7967) * millimeter, vector(3.7817, -10.7791) * millimeter, vector(3.8247, -10.7612) * millimeter, vector(3.8680, -10.7429) * millimeter, vector(3.9116, -10.7244) * millimeter, vector(3.9555, -10.7056) * millimeter, vector(3.9997, -10.6865) * millimeter, vector(4.0442, -10.6674) * millimeter, vector(4.0890, -10.6481) * millimeter, vector(4.1341, -10.6287) * millimeter, vector(4.1794, -10.6093) * millimeter, vector(4.2250, -10.5899) * millimeter, vector(4.2707, -10.5706) * millimeter, vector(4.3166, -10.5514) * millimeter, vector(4.3626, -10.5322) * millimeter, vector(4.4086, -10.5132) * millimeter, vector(4.4547, -10.4944) * millimeter, vector(4.5007, -10.4757) * millimeter, vector(4.5466, -10.4572) * millimeter, vector(4.5924, -10.4389) * millimeter, vector(4.6379, -10.4207) * millimeter, vector(4.6833, -10.4026) * millimeter, vector(4.7283, -10.3847) * millimeter, vector(4.7730, -10.3669) * millimeter, vector(4.8174, -10.3492) * millimeter, vector(4.8613, -10.3315) * millimeter, vector(4.9049, -10.3138) * millimeter, vector(4.9479, -10.2961) * millimeter, vector(4.9906, -10.2783) * millimeter, vector(5.0327, -10.2604) * millimeter, vector(5.0744, -10.2424) * millimeter, vector(5.1156, -10.2242) * millimeter, vector(5.1564, -10.2058) * millimeter, vector(5.1967, -10.1872) * millimeter, vector(5.2366, -10.1683) * millimeter, vector(5.2761, -10.1491) * millimeter, vector(5.3153, -10.1295) * millimeter, vector(5.3542, -10.1095) * millimeter, vector(5.3928, -10.0892) * millimeter, vector(5.4311, -10.0684) * millimeter, vector(5.4693, -10.0472) * millimeter, vector(5.5074, -10.0255) * millimeter, vector(5.5453, -10.0033) * millimeter, vector(5.5832, -9.9806) * millimeter, vector(5.6211, -9.9574) * millimeter, vector(5.6591, -9.9337) * millimeter, vector(5.6971, -9.9096) * millimeter, vector(5.7352, -9.8849) * millimeter, vector(5.7735, -9.8598) * millimeter, vector(5.8119, -9.8343) * millimeter, vector(5.8506, -9.8083) * millimeter, vector(5.8895, -9.7819) * millimeter, vector(5.9286, -9.7552) * millimeter, vector(5.9680, -9.7282) * millimeter, vector(6.0077, -9.7009) * millimeter, vector(6.0476, -9.6734) * millimeter, vector(6.0878, -9.6457) * millimeter, vector(6.1282, -9.6179) * millimeter, vector(6.1689, -9.5901) * millimeter, vector(6.2098, -9.5622) * millimeter, vector(6.2509, -9.5343) * millimeter, vector(6.2921, -9.5065) * millimeter, vector(6.3335, -9.4788) * millimeter, vector(6.3750, -9.4511) * millimeter, vector(6.4164, -9.4237) * millimeter, vector(6.4579, -9.3964) * millimeter, vector(6.4993, -9.3693) * millimeter, vector(6.5407, -9.3424) * millimeter, vector(6.5818, -9.3156) * millimeter, vector(6.6227, -9.2891) * millimeter, vector(6.6634, -9.2627) * millimeter, vector(6.7038, -9.2365) * millimeter, vector(6.7438, -9.2105) * millimeter, vector(6.7835, -9.1846) * millimeter, vector(6.8227, -9.1587) * millimeter, vector(6.8615, -9.1329) * millimeter, vector(6.8999, -9.1072) * millimeter, vector(6.9377, -9.0814) * millimeter, vector(6.9751, -9.0556) * millimeter, vector(7.0120, -9.0298) * millimeter, vector(7.0483, -9.0038) * millimeter, vector(7.0843, -8.9776) * millimeter, vector(7.1197, -8.9513) * millimeter, vector(7.1547, -8.9247) * millimeter, vector(7.1893, -8.8979) * millimeter, vector(7.2236, -8.8707) * millimeter, vector(7.2575, -8.8432) * millimeter, vector(7.2910, -8.8154) * millimeter, vector(7.3243, -8.7871) * millimeter, vector(7.3574, -8.7584) * millimeter, vector(7.3903, -8.7292) * millimeter, vector(7.4231, -8.6996) * millimeter, vector(7.4557, -8.6695) * millimeter, vector(7.4883, -8.6388) * millimeter, vector(7.5209, -8.6077) * millimeter, vector(7.5535, -8.5761) * millimeter, vector(7.5861, -8.5440) * millimeter, vector(7.6188, -8.5114) * millimeter, vector(7.6517, -8.4784) * millimeter, vector(7.6847, -8.4450) * millimeter, vector(7.7179, -8.4111) * millimeter, vector(7.7512, -8.3770) * millimeter, vector(7.7848, -8.3425) * millimeter, vector(7.8186, -8.3077) * millimeter, vector(7.8526, -8.2727) * millimeter, vector(7.8868, -8.2376) * millimeter, vector(7.9213, -8.2023) * millimeter, vector(7.9560, -8.1670) * millimeter, vector(7.9908, -8.1316) * millimeter, vector(8.0259, -8.0963) * millimeter, vector(8.0610, -8.0610) * millimeter, vector(8.0963, -8.0259) * millimeter, vector(8.1316, -7.9908) * millimeter, vector(8.1670, -7.9560) * millimeter, vector(8.2023, -7.9213) * millimeter, vector(8.2376, -7.8868) * millimeter, vector(8.2727, -7.8526) * millimeter, vector(8.3077, -7.8186) * millimeter, vector(8.3425, -7.7848) * millimeter, vector(8.3770, -7.7512) * millimeter, vector(8.4111, -7.7179) * millimeter, vector(8.4450, -7.6847) * millimeter, vector(8.4784, -7.6517) * millimeter, vector(8.5114, -7.6188) * millimeter, vector(8.5440, -7.5861) * millimeter, vector(8.5761, -7.5535) * millimeter, vector(8.6077, -7.5209) * millimeter, vector(8.6388, -7.4883) * millimeter, vector(8.6695, -7.4557) * millimeter, vector(8.6996, -7.4231) * millimeter, vector(8.7292, -7.3903) * millimeter, vector(8.7584, -7.3574) * millimeter, vector(8.7871, -7.3243) * millimeter, vector(8.8154, -7.2910) * millimeter, vector(8.8432, -7.2575) * millimeter, vector(8.8707, -7.2236) * millimeter, vector(8.8979, -7.1893) * millimeter, vector(8.9247, -7.1547) * millimeter, vector(8.9513, -7.1197) * millimeter, vector(8.9776, -7.0843) * millimeter, vector(9.0038, -7.0483) * millimeter, vector(9.0298, -7.0120) * millimeter, vector(9.0556, -6.9751) * millimeter, vector(9.0814, -6.9377) * millimeter, vector(9.1072, -6.8999) * millimeter, vector(9.1329, -6.8615) * millimeter, vector(9.1587, -6.8227) * millimeter, vector(9.1846, -6.7835) * millimeter, vector(9.2105, -6.7438) * millimeter, vector(9.2365, -6.7038) * millimeter, vector(9.2627, -6.6634) * millimeter, vector(9.2891, -6.6227) * millimeter, vector(9.3156, -6.5818) * millimeter, vector(9.3424, -6.5407) * millimeter, vector(9.3693, -6.4993) * millimeter, vector(9.3964, -6.4579) * millimeter, vector(9.4237, -6.4164) * millimeter, vector(9.4511, -6.3750) * millimeter, vector(9.4788, -6.3335) * millimeter, vector(9.5065, -6.2921) * millimeter, vector(9.5343, -6.2509) * millimeter, vector(9.5622, -6.2098) * millimeter, vector(9.5901, -6.1689) * millimeter, vector(9.6179, -6.1282) * millimeter, vector(9.6457, -6.0878) * millimeter, vector(9.6734, -6.0476) * millimeter, vector(9.7009, -6.0077) * millimeter, vector(9.7282, -5.9680) * millimeter, vector(9.7552, -5.9286) * millimeter, vector(9.7819, -5.8895) * millimeter, vector(9.8083, -5.8506) * millimeter, vector(9.8343, -5.8119) * millimeter, vector(9.8598, -5.7735) * millimeter, vector(9.8849, -5.7352) * millimeter, vector(9.9096, -5.6971) * millimeter, vector(9.9337, -5.6591) * millimeter, vector(9.9574, -5.6211) * millimeter, vector(9.9806, -5.5832) * millimeter, vector(10.0033, -5.5453) * millimeter, vector(10.0255, -5.5074) * millimeter, vector(10.0472, -5.4693) * millimeter, vector(10.0684, -5.4311) * millimeter, vector(10.0892, -5.3928) * millimeter, vector(10.1095, -5.3542) * millimeter, vector(10.1295, -5.3153) * millimeter, vector(10.1491, -5.2761) * millimeter, vector(10.1683, -5.2366) * millimeter, vector(10.1872, -5.1967) * millimeter, vector(10.2058, -5.1564) * millimeter, vector(10.2242, -5.1156) * millimeter, vector(10.2424, -5.0744) * millimeter, vector(10.2604, -5.0327) * millimeter, vector(10.2783, -4.9906) * millimeter, vector(10.2961, -4.9479) * millimeter, vector(10.3138, -4.9049) * millimeter, vector(10.3315, -4.8613) * millimeter, vector(10.3492, -4.8174) * millimeter, vector(10.3669, -4.7730) * millimeter, vector(10.3847, -4.7283) * millimeter, vector(10.4026, -4.6833) * millimeter, vector(10.4207, -4.6379) * millimeter, vector(10.4389, -4.5924) * millimeter, vector(10.4572, -4.5466) * millimeter, vector(10.4757, -4.5007) * millimeter, vector(10.4944, -4.4547) * millimeter, vector(10.5132, -4.4086) * millimeter, vector(10.5322, -4.3626) * millimeter, vector(10.5514, -4.3166) * millimeter, vector(10.5706, -4.2707) * millimeter, vector(10.5899, -4.2250) * millimeter, vector(10.6093, -4.1794) * millimeter, vector(10.6287, -4.1341) * millimeter, vector(10.6481, -4.0890) * millimeter, vector(10.6674, -4.0442) * millimeter, vector(10.6865, -3.9997) * millimeter, vector(10.7056, -3.9555) * millimeter, vector(10.7244, -3.9116) * millimeter, vector(10.7429, -3.8680) * millimeter, vector(10.7612, -3.8247) * millimeter, vector(10.7791, -3.7817) * millimeter, vector(10.7967, -3.7390) * millimeter, vector(10.8139, -3.6966) * millimeter, vector(10.8306, -3.6544) * millimeter, vector(10.8469, -3.6124) * millimeter, vector(10.8627, -3.5705) * millimeter, vector(10.8781, -3.5288) * millimeter, vector(10.8929, -3.4872) * millimeter, vector(10.9073, -3.4457) * millimeter, vector(10.9211, -3.4041) * millimeter, vector(10.9345, -3.3625) * millimeter, vector(10.9474, -3.3209) * millimeter, vector(10.9598, -3.2790) * millimeter, vector(10.9718, -3.2370) * millimeter, vector(10.9834, -3.1948) * millimeter, vector(10.9945, -3.1522) * millimeter, vector(11.0053, -3.1094) * millimeter, vector(11.0157, -3.0662) * millimeter, vector(11.0258, -3.0227) * millimeter, vector(11.0356, -2.9787) * millimeter, vector(11.0451, -2.9343) * millimeter, vector(11.0544, -2.8895) * millimeter, vector(11.0635, -2.8442) * millimeter, vector(11.0725, -2.7985) * millimeter, vector(11.0814, -2.7524) * millimeter, vector(11.0901, -2.7058) * millimeter, vector(11.0989, -2.6588) * millimeter, vector(11.1076, -2.6115) * millimeter, vector(11.1164, -2.5638) * millimeter, vector(11.1253, -2.5159) * millimeter, vector(11.1342, -2.4676) * millimeter, vector(11.1433, -2.4191) * millimeter, vector(11.1525, -2.3705) * millimeter, vector(11.1618, -2.3217) * millimeter, vector(11.1713, -2.2729) * millimeter, vector(11.1810, -2.2240) * millimeter, vector(11.1907, -2.1752) * millimeter, vector(11.2007, -2.1264) * millimeter, vector(11.2107, -2.0778) * millimeter, vector(11.2208, -2.0294) * millimeter, vector(11.2310, -1.9811) * millimeter, vector(11.2412, -1.9331) * millimeter, vector(11.2514, -1.8854) * millimeter, vector(11.2615, -1.8380) * millimeter, vector(11.2715, -1.7909) * millimeter, vector(11.2814, -1.7442) * millimeter, vector(11.2911, -1.6978) * millimeter, vector(11.3006, -1.6518) * millimeter, vector(11.3098, -1.6061) * millimeter, vector(11.3187, -1.5608) * millimeter, vector(11.3273, -1.5158) * millimeter, vector(11.3354, -1.4712) * millimeter, vector(11.3432, -1.4268) * millimeter, vector(11.3506, -1.3827) * millimeter, vector(11.3575, -1.3388) * millimeter, vector(11.3639, -1.2951) * millimeter, vector(11.3699, -1.2516) * millimeter, vector(11.3754, -1.2081) * millimeter, vector(11.3804, -1.1647) * millimeter, vector(11.3849, -1.1213) * millimeter, vector(11.3889, -1.0779) * millimeter, vector(11.3925, -1.0343) * millimeter, vector(11.3956, -0.9906) * millimeter, vector(11.3982, -0.9468) * millimeter, vector(11.4004, -0.9026) * millimeter, vector(11.4022, -0.8582) * millimeter, vector(11.4036, -0.8136) * millimeter, vector(11.4046, -0.7685) * millimeter, vector(11.4053, -0.7231) * millimeter, vector(11.4057, -0.6773) * millimeter, vector(11.4058, -0.6312) * millimeter, vector(11.4057, -0.5846) * millimeter, vector(11.4054, -0.5376) * millimeter, vector(11.4049, -0.4902) * millimeter, vector(11.4043, -0.4425) * millimeter, vector(11.4037, -0.3943) * millimeter, vector(11.4030, -0.3459) * millimeter, vector(11.4023, -0.2971) * millimeter, vector(11.4017, -0.2480) * millimeter, vector(11.4011, -0.1987) * millimeter, vector(11.4006, -0.1492) * millimeter, vector(11.4003, -0.0996) * millimeter, vector(11.4001, -0.0498) * millimeter, vector(11.4000, -0.0000) * millimeter, vector(11.4001, 0.0498) * millimeter, vector(11.4003, 0.0996) * millimeter, vector(11.4006, 0.1492) * millimeter, vector(11.4011, 0.1987) * millimeter, vector(11.4017, 0.2480) * millimeter, vector(11.4023, 0.2971) * millimeter, vector(11.4030, 0.3459) * millimeter, vector(11.4037, 0.3943) * millimeter, vector(11.4043, 0.4425) * millimeter, vector(11.4049, 0.4902) * millimeter, vector(11.4054, 0.5376) * millimeter, vector(11.4057, 0.5846) * millimeter, vector(11.4058, 0.6312) * millimeter, vector(11.4057, 0.6773) * millimeter, vector(11.4053, 0.7231) * millimeter, vector(11.4046, 0.7685) * millimeter, vector(11.4036, 0.8136) * millimeter, vector(11.4022, 0.8582) * millimeter, vector(11.4004, 0.9026) * millimeter, vector(11.3982, 0.9468) * millimeter, vector(11.3956, 0.9906) * millimeter, vector(11.3925, 1.0343) * millimeter, vector(11.3889, 1.0779) * millimeter, vector(11.3849, 1.1213) * millimeter, vector(11.3804, 1.1647) * millimeter, vector(11.3754, 1.2081) * millimeter, vector(11.3699, 1.2516) * millimeter, vector(11.3639, 1.2951) * millimeter, vector(11.3575, 1.3388) * millimeter, vector(11.3506, 1.3827) * millimeter, vector(11.3432, 1.4268) * millimeter, vector(11.3354, 1.4712) * millimeter, vector(11.3273, 1.5158) * millimeter, vector(11.3187, 1.5608) * millimeter, vector(11.3098, 1.6061) * millimeter, vector(11.3006, 1.6518) * millimeter, vector(11.2911, 1.6978) * millimeter, vector(11.2814, 1.7442) * millimeter, vector(11.2715, 1.7909) * millimeter, vector(11.2615, 1.8380) * millimeter, vector(11.2514, 1.8854) * millimeter, vector(11.2412, 1.9331) * millimeter, vector(11.2310, 1.9811) * millimeter, vector(11.2208, 2.0294) * millimeter, vector(11.2107, 2.0778) * millimeter, vector(11.2007, 2.1264) * millimeter, vector(11.1907, 2.1752) * millimeter, vector(11.1810, 2.2240) * millimeter, vector(11.1713, 2.2729) * millimeter, vector(11.1618, 2.3217) * millimeter, vector(11.1525, 2.3705) * millimeter, vector(11.1433, 2.4191) * millimeter, vector(11.1342, 2.4676) * millimeter, vector(11.1253, 2.5159) * millimeter, vector(11.1164, 2.5638) * millimeter, vector(11.1076, 2.6115) * millimeter, vector(11.0989, 2.6588) * millimeter, vector(11.0901, 2.7058) * millimeter, vector(11.0814, 2.7524) * millimeter, vector(11.0725, 2.7985) * millimeter, vector(11.0635, 2.8442) * millimeter, vector(11.0544, 2.8895) * millimeter, vector(11.0451, 2.9343) * millimeter, vector(11.0356, 2.9787) * millimeter, vector(11.0258, 3.0227) * millimeter, vector(11.0157, 3.0662) * millimeter, vector(11.0053, 3.1094) * millimeter, vector(10.9945, 3.1522) * millimeter, vector(10.9834, 3.1948) * millimeter, vector(10.9718, 3.2370) * millimeter, vector(10.9598, 3.2790) * millimeter, vector(10.9474, 3.3209) * millimeter, vector(10.9345, 3.3625) * millimeter, vector(10.9211, 3.4041) * millimeter, vector(10.9073, 3.4457) * millimeter, vector(10.8929, 3.4872) * millimeter, vector(10.8781, 3.5288) * millimeter, vector(10.8627, 3.5705) * millimeter, vector(10.8469, 3.6124) * millimeter, vector(10.8306, 3.6544) * millimeter, vector(10.8139, 3.6966) * millimeter, vector(10.7967, 3.7390) * millimeter, vector(10.7791, 3.7817) * millimeter, vector(10.7612, 3.8247) * millimeter, vector(10.7429, 3.8680) * millimeter, vector(10.7244, 3.9116) * millimeter, vector(10.7056, 3.9555) * millimeter, vector(10.6865, 3.9997) * millimeter, vector(10.6674, 4.0442) * millimeter, vector(10.6481, 4.0890) * millimeter, vector(10.6287, 4.1341) * millimeter, vector(10.6093, 4.1794) * millimeter, vector(10.5899, 4.2250) * millimeter, vector(10.5706, 4.2707) * millimeter, vector(10.5514, 4.3166) * millimeter, vector(10.5322, 4.3626) * millimeter, vector(10.5132, 4.4086) * millimeter, vector(10.4944, 4.4547) * millimeter, vector(10.4757, 4.5007) * millimeter, vector(10.4572, 4.5466) * millimeter, vector(10.4389, 4.5924) * millimeter, vector(10.4207, 4.6379) * millimeter, vector(10.4026, 4.6833) * millimeter, vector(10.3847, 4.7283) * millimeter, vector(10.3669, 4.7730) * millimeter, vector(10.3492, 4.8174) * millimeter, vector(10.3315, 4.8613) * millimeter, vector(10.3138, 4.9049) * millimeter, vector(10.2961, 4.9479) * millimeter, vector(10.2783, 4.9906) * millimeter, vector(10.2604, 5.0327) * millimeter, vector(10.2424, 5.0744) * millimeter, vector(10.2242, 5.1156) * millimeter, vector(10.2058, 5.1564) * millimeter, vector(10.1872, 5.1967) * millimeter, vector(10.1683, 5.2366) * millimeter, vector(10.1491, 5.2761) * millimeter, vector(10.1295, 5.3153) * millimeter, vector(10.1095, 5.3542) * millimeter, vector(10.0892, 5.3928) * millimeter, vector(10.0684, 5.4311) * millimeter, vector(10.0472, 5.4693) * millimeter, vector(10.0255, 5.5074) * millimeter, vector(10.0033, 5.5453) * millimeter, vector(9.9806, 5.5832) * millimeter, vector(9.9574, 5.6211) * millimeter, vector(9.9337, 5.6591) * millimeter, vector(9.9096, 5.6971) * millimeter, vector(9.8849, 5.7352) * millimeter, vector(9.8598, 5.7735) * millimeter, vector(9.8343, 5.8119) * millimeter, vector(9.8083, 5.8506) * millimeter, vector(9.7819, 5.8895) * millimeter, vector(9.7552, 5.9286) * millimeter, vector(9.7282, 5.9680) * millimeter, vector(9.7009, 6.0077) * millimeter, vector(9.6734, 6.0476) * millimeter, vector(9.6457, 6.0878) * millimeter, vector(9.6179, 6.1282) * millimeter, vector(9.5901, 6.1689) * millimeter, vector(9.5622, 6.2098) * millimeter, vector(9.5343, 6.2509) * millimeter, vector(9.5065, 6.2921) * millimeter, vector(9.4788, 6.3335) * millimeter, vector(9.4511, 6.3750) * millimeter, vector(9.4237, 6.4164) * millimeter, vector(9.3964, 6.4579) * millimeter, vector(9.3693, 6.4993) * millimeter, vector(9.3424, 6.5407) * millimeter, vector(9.3156, 6.5818) * millimeter, vector(9.2891, 6.6227) * millimeter, vector(9.2627, 6.6634) * millimeter, vector(9.2365, 6.7038) * millimeter, vector(9.2105, 6.7438) * millimeter, vector(9.1846, 6.7835) * millimeter, vector(9.1587, 6.8227) * millimeter, vector(9.1329, 6.8615) * millimeter, vector(9.1072, 6.8999) * millimeter, vector(9.0814, 6.9377) * millimeter, vector(9.0556, 6.9751) * millimeter, vector(9.0298, 7.0120) * millimeter, vector(9.0038, 7.0483) * millimeter, vector(8.9776, 7.0843) * millimeter, vector(8.9513, 7.1197) * millimeter, vector(8.9247, 7.1547) * millimeter, vector(8.8979, 7.1893) * millimeter, vector(8.8707, 7.2236) * millimeter, vector(8.8432, 7.2575) * millimeter, vector(8.8154, 7.2910) * millimeter, vector(8.7871, 7.3243) * millimeter, vector(8.7584, 7.3574) * millimeter, vector(8.7292, 7.3903) * millimeter, vector(8.6996, 7.4231) * millimeter, vector(8.6695, 7.4557) * millimeter, vector(8.6388, 7.4883) * millimeter, vector(8.6077, 7.5209) * millimeter, vector(8.5761, 7.5535) * millimeter, vector(8.5440, 7.5861) * millimeter, vector(8.5114, 7.6188) * millimeter, vector(8.4784, 7.6517) * millimeter, vector(8.4450, 7.6847) * millimeter, vector(8.4111, 7.7179) * millimeter, vector(8.3770, 7.7512) * millimeter, vector(8.3425, 7.7848) * millimeter, vector(8.3077, 7.8186) * millimeter, vector(8.2727, 7.8526) * millimeter, vector(8.2376, 7.8868) * millimeter, vector(8.2023, 7.9213) * millimeter, vector(8.1670, 7.9560) * millimeter, vector(8.1316, 7.9908) * millimeter, vector(8.0963, 8.0259) * millimeter, vector(8.0610, 8.0610) * millimeter, vector(8.0259, 8.0963) * millimeter, vector(7.9908, 8.1316) * millimeter, vector(7.9560, 8.1670) * millimeter, vector(7.9213, 8.2023) * millimeter, vector(7.8868, 8.2376) * millimeter, vector(7.8526, 8.2727) * millimeter, vector(7.8186, 8.3077) * millimeter, vector(7.7848, 8.3425) * millimeter, vector(7.7512, 8.3770) * millimeter, vector(7.7179, 8.4111) * millimeter, vector(7.6847, 8.4450) * millimeter, vector(7.6517, 8.4784) * millimeter, vector(7.6188, 8.5114) * millimeter, vector(7.5861, 8.5440) * millimeter, vector(7.5535, 8.5761) * millimeter, vector(7.5209, 8.6077) * millimeter, vector(7.4883, 8.6388) * millimeter, vector(7.4557, 8.6695) * millimeter, vector(7.4231, 8.6996) * millimeter, vector(7.3903, 8.7292) * millimeter, vector(7.3574, 8.7584) * millimeter, vector(7.3243, 8.7871) * millimeter, vector(7.2910, 8.8154) * millimeter, vector(7.2575, 8.8432) * millimeter, vector(7.2236, 8.8707) * millimeter, vector(7.1893, 8.8979) * millimeter, vector(7.1547, 8.9247) * millimeter, vector(7.1197, 8.9513) * millimeter, vector(7.0843, 8.9776) * millimeter, vector(7.0483, 9.0038) * millimeter, vector(7.0120, 9.0298) * millimeter, vector(6.9751, 9.0556) * millimeter, vector(6.9377, 9.0814) * millimeter, vector(6.8999, 9.1072) * millimeter, vector(6.8615, 9.1329) * millimeter, vector(6.8227, 9.1587) * millimeter, vector(6.7835, 9.1846) * millimeter, vector(6.7438, 9.2105) * millimeter, vector(6.7038, 9.2365) * millimeter, vector(6.6634, 9.2627) * millimeter, vector(6.6227, 9.2891) * millimeter, vector(6.5818, 9.3156) * millimeter, vector(6.5407, 9.3424) * millimeter, vector(6.4993, 9.3693) * millimeter, vector(6.4579, 9.3964) * millimeter, vector(6.4164, 9.4237) * millimeter, vector(6.3750, 9.4511) * millimeter, vector(6.3335, 9.4788) * millimeter, vector(6.2921, 9.5065) * millimeter, vector(6.2509, 9.5343) * millimeter, vector(6.2098, 9.5622) * millimeter, vector(6.1689, 9.5901) * millimeter, vector(6.1282, 9.6179) * millimeter, vector(6.0878, 9.6457) * millimeter, vector(6.0476, 9.6734) * millimeter, vector(6.0077, 9.7009) * millimeter, vector(5.9680, 9.7282) * millimeter, vector(5.9286, 9.7552) * millimeter, vector(5.8895, 9.7819) * millimeter, vector(5.8506, 9.8083) * millimeter, vector(5.8119, 9.8343) * millimeter, vector(5.7735, 9.8598) * millimeter, vector(5.7352, 9.8849) * millimeter, vector(5.6971, 9.9096) * millimeter, vector(5.6591, 9.9337) * millimeter, vector(5.6211, 9.9574) * millimeter, vector(5.5832, 9.9806) * millimeter, vector(5.5453, 10.0033) * millimeter, vector(5.5074, 10.0255) * millimeter, vector(5.4693, 10.0472) * millimeter, vector(5.4311, 10.0684) * millimeter, vector(5.3928, 10.0892) * millimeter, vector(5.3542, 10.1095) * millimeter, vector(5.3153, 10.1295) * millimeter, vector(5.2761, 10.1491) * millimeter, vector(5.2366, 10.1683) * millimeter, vector(5.1967, 10.1872) * millimeter, vector(5.1564, 10.2058) * millimeter, vector(5.1156, 10.2242) * millimeter, vector(5.0744, 10.2424) * millimeter, vector(5.0327, 10.2604) * millimeter, vector(4.9906, 10.2783) * millimeter, vector(4.9479, 10.2961) * millimeter, vector(4.9049, 10.3138) * millimeter, vector(4.8613, 10.3315) * millimeter, vector(4.8174, 10.3492) * millimeter, vector(4.7730, 10.3669) * millimeter, vector(4.7283, 10.3847) * millimeter, vector(4.6833, 10.4026) * millimeter, vector(4.6379, 10.4207) * millimeter, vector(4.5924, 10.4389) * millimeter, vector(4.5466, 10.4572) * millimeter, vector(4.5007, 10.4757) * millimeter, vector(4.4547, 10.4944) * millimeter, vector(4.4086, 10.5132) * millimeter, vector(4.3626, 10.5322) * millimeter, vector(4.3166, 10.5514) * millimeter, vector(4.2707, 10.5706) * millimeter, vector(4.2250, 10.5899) * millimeter, vector(4.1794, 10.6093) * millimeter, vector(4.1341, 10.6287) * millimeter, vector(4.0890, 10.6481) * millimeter, vector(4.0442, 10.6674) * millimeter, vector(3.9997, 10.6865) * millimeter, vector(3.9555, 10.7056) * millimeter, vector(3.9116, 10.7244) * millimeter, vector(3.8680, 10.7429) * millimeter, vector(3.8247, 10.7612) * millimeter, vector(3.7817, 10.7791) * millimeter, vector(3.7390, 10.7967) * millimeter, vector(3.6966, 10.8139) * millimeter, vector(3.6544, 10.8306) * millimeter, vector(3.6124, 10.8469) * millimeter, vector(3.5705, 10.8627) * millimeter, vector(3.5288, 10.8781) * millimeter, vector(3.4872, 10.8929) * millimeter, vector(3.4457, 10.9073) * millimeter, vector(3.4041, 10.9211) * millimeter, vector(3.3625, 10.9345) * millimeter, vector(3.3209, 10.9474) * millimeter, vector(3.2790, 10.9598) * millimeter, vector(3.2370, 10.9718) * millimeter, vector(3.1948, 10.9834) * millimeter, vector(3.1522, 10.9945) * millimeter, vector(3.1094, 11.0053) * millimeter, vector(3.0662, 11.0157) * millimeter, vector(3.0227, 11.0258) * millimeter, vector(2.9787, 11.0356) * millimeter, vector(2.9343, 11.0451) * millimeter, vector(2.8895, 11.0544) * millimeter, vector(2.8442, 11.0635) * millimeter, vector(2.7985, 11.0725) * millimeter, vector(2.7524, 11.0814) * millimeter, vector(2.7058, 11.0901) * millimeter, vector(2.6588, 11.0989) * millimeter, vector(2.6115, 11.1076) * millimeter, vector(2.5638, 11.1164) * millimeter, vector(2.5159, 11.1253) * millimeter, vector(2.4676, 11.1342) * millimeter, vector(2.4191, 11.1433) * millimeter, vector(2.3705, 11.1525) * millimeter, vector(2.3217, 11.1618) * millimeter, vector(2.2729, 11.1713) * millimeter, vector(2.2240, 11.1810) * millimeter, vector(2.1752, 11.1907) * millimeter, vector(2.1264, 11.2007) * millimeter, vector(2.0778, 11.2107) * millimeter, vector(2.0294, 11.2208) * millimeter, vector(1.9811, 11.2310) * millimeter, vector(1.9331, 11.2412) * millimeter, vector(1.8854, 11.2514) * millimeter, vector(1.8380, 11.2615) * millimeter, vector(1.7909, 11.2715) * millimeter, vector(1.7442, 11.2814) * millimeter, vector(1.6978, 11.2911) * millimeter, vector(1.6518, 11.3006) * millimeter, vector(1.6061, 11.3098) * millimeter, vector(1.5608, 11.3187) * millimeter, vector(1.5158, 11.3273) * millimeter, vector(1.4712, 11.3354) * millimeter, vector(1.4268, 11.3432) * millimeter, vector(1.3827, 11.3506) * millimeter, vector(1.3388, 11.3575) * millimeter, vector(1.2951, 11.3639) * millimeter, vector(1.2516, 11.3699) * millimeter, vector(1.2081, 11.3754) * millimeter, vector(1.1647, 11.3804) * millimeter, vector(1.1213, 11.3849) * millimeter, vector(1.0779, 11.3889) * millimeter, vector(1.0343, 11.3925) * millimeter, vector(0.9906, 11.3956) * millimeter, vector(0.9468, 11.3982) * millimeter, vector(0.9026, 11.4004) * millimeter, vector(0.8582, 11.4022) * millimeter, vector(0.8136, 11.4036) * millimeter, vector(0.7685, 11.4046) * millimeter, vector(0.7231, 11.4053) * millimeter, vector(0.6773, 11.4057) * millimeter, vector(0.6312, 11.4058) * millimeter, vector(0.5846, 11.4057) * millimeter, vector(0.5376, 11.4054) * millimeter, vector(0.4902, 11.4049) * millimeter, vector(0.4425, 11.4043) * millimeter, vector(0.3943, 11.4037) * millimeter, vector(0.3459, 11.4030) * millimeter, vector(0.2971, 11.4023) * millimeter, vector(0.2480, 11.4017) * millimeter, vector(0.1987, 11.4011) * millimeter, vector(0.1492, 11.4006) * millimeter, vector(0.0996, 11.4003) * millimeter, vector(0.0498, 11.4001) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.1925, 11.3997) * millimeter, vector(-0.3849, 11.3988) * millimeter, vector(-0.5773, 11.3972) * millimeter, vector(-0.7696, 11.3947) * millimeter, vector(-0.9617, 11.3914) * millimeter, vector(-1.1537, 11.3868) * millimeter, vector(-1.3454, 11.3809) * millimeter, vector(-1.5367, 11.3734) * millimeter, vector(-1.7277, 11.3640) * millimeter, vector(-1.9183, 11.3524) * millimeter, vector(-2.1083, 11.3384) * millimeter, vector(-2.2976, 11.3215) * millimeter, vector(-2.4863, 11.3016) * millimeter, vector(-2.6740, 11.2783) * millimeter, vector(-2.8608, 11.2514) * millimeter, vector(-3.0465, 11.2205) * millimeter, vector(-3.2309, 11.1855) * millimeter, vector(-3.4140, 11.1461) * millimeter, vector(-3.5956, 11.1021) * millimeter, vector(-3.7756, 11.0534) * millimeter, vector(-3.9539, 10.9999) * millimeter, vector(-4.1303, 10.9414) * millimeter, vector(-4.3049, 10.8779) * millimeter, vector(-4.4774, 10.8094) * millimeter, vector(-4.6478, 10.7358) * millimeter, vector(-4.8161, 10.6573) * millimeter, vector(-4.9823, 10.5739) * millimeter, vector(-5.1462, 10.4857) * millimeter, vector(-5.3079, 10.3929) * millimeter, vector(-5.4674, 10.2955) * millimeter, vector(-5.6247, 10.1939) * millimeter, vector(-5.7799, 10.0883) * millimeter, vector(-5.9330, 9.9788) * millimeter, vector(-6.0842, 9.8658) * millimeter, vector(-6.2334, 9.7495) * millimeter, vector(-6.3808, 9.6302) * millimeter, vector(-6.5267, 9.5082) * millimeter, vector(-6.6709, 9.3838) * millimeter, vector(-6.8139, 9.2573) * millimeter, vector(-6.9556, 9.1289) * millimeter, vector(-7.0962, 8.9989) * millimeter, vector(-7.2359, 8.8675) * millimeter, vector(-7.3748, 8.7350) * millimeter, vector(-7.5131, 8.6015) * millimeter, vector(-7.6508, 8.4672) * millimeter, vector(-7.7880, 8.3323) * millimeter, vector(-7.9247, 8.1969) * millimeter, vector(-8.0610, 8.0610) * millimeter, vector(-8.1969, 7.9247) * millimeter, vector(-8.3323, 7.7880) * millimeter, vector(-8.4672, 7.6508) * millimeter, vector(-8.6015, 7.5131) * millimeter, vector(-8.7350, 7.3748) * millimeter, vector(-8.8675, 7.2359) * millimeter, vector(-8.9989, 7.0962) * millimeter, vector(-9.1289, 6.9556) * millimeter, vector(-9.2573, 6.8139) * millimeter, vector(-9.3838, 6.6709) * millimeter, vector(-9.5082, 6.5267) * millimeter, vector(-9.6302, 6.3808) * millimeter, vector(-9.7495, 6.2334) * millimeter, vector(-9.8658, 6.0842) * millimeter, vector(-9.9788, 5.9330) * millimeter, vector(-10.0883, 5.7799) * millimeter, vector(-10.1939, 5.6247) * millimeter, vector(-10.2955, 5.4674) * millimeter, vector(-10.3929, 5.3079) * millimeter, vector(-10.4857, 5.1462) * millimeter, vector(-10.5739, 4.9823) * millimeter, vector(-10.6573, 4.8161) * millimeter, vector(-10.7358, 4.6478) * millimeter, vector(-10.8094, 4.4774) * millimeter, vector(-10.8779, 4.3049) * millimeter, vector(-10.9414, 4.1303) * millimeter, vector(-10.9999, 3.9539) * millimeter, vector(-11.0534, 3.7756) * millimeter, vector(-11.1021, 3.5956) * millimeter, vector(-11.1461, 3.4140) * millimeter, vector(-11.1855, 3.2309) * millimeter, vector(-11.2205, 3.0465) * millimeter, vector(-11.2514, 2.8608) * millimeter, vector(-11.2783, 2.6740) * millimeter, vector(-11.3016, 2.4863) * millimeter, vector(-11.3215, 2.2976) * millimeter, vector(-11.3384, 2.1083) * millimeter, vector(-11.3524, 1.9183) * millimeter, vector(-11.3640, 1.7277) * millimeter, vector(-11.3734, 1.5367) * millimeter, vector(-11.3809, 1.3454) * millimeter, vector(-11.3868, 1.1537) * millimeter, vector(-11.3914, 0.9617) * millimeter, vector(-11.3947, 0.7696) * millimeter, vector(-11.3972, 0.5773) * millimeter, vector(-11.3988, 0.3849) * millimeter, vector(-11.3997, 0.1925) * millimeter, vector(-11.4000, 0.0000) * millimeter, vector(-11.3997, -0.1925) * millimeter, vector(-11.3988, -0.3849) * millimeter, vector(-11.3972, -0.5773) * millimeter, vector(-11.3947, -0.7696) * millimeter, vector(-11.3914, -0.9617) * millimeter, vector(-11.3868, -1.1537) * millimeter, vector(-11.3809, -1.3454) * millimeter, vector(-11.3734, -1.5367) * millimeter, vector(-11.3640, -1.7277) * millimeter, vector(-11.3524, -1.9183) * millimeter, vector(-11.3384, -2.1083) * millimeter, vector(-11.3215, -2.2976) * millimeter, vector(-11.3016, -2.4863) * millimeter, vector(-11.2783, -2.6740) * millimeter, vector(-11.2514, -2.8608) * millimeter, vector(-11.2205, -3.0465) * millimeter, vector(-11.1855, -3.2309) * millimeter, vector(-11.1461, -3.4140) * millimeter, vector(-11.1021, -3.5956) * millimeter, vector(-11.0534, -3.7756) * millimeter, vector(-10.9999, -3.9539) * millimeter, vector(-10.9414, -4.1303) * millimeter, vector(-10.8779, -4.3049) * millimeter, vector(-10.8094, -4.4774) * millimeter, vector(-10.7358, -4.6478) * millimeter, vector(-10.6573, -4.8161) * millimeter, vector(-10.5739, -4.9823) * millimeter, vector(-10.4857, -5.1462) * millimeter, vector(-10.3929, -5.3079) * millimeter, vector(-10.2955, -5.4674) * millimeter, vector(-10.1939, -5.6247) * millimeter, vector(-10.0883, -5.7799) * millimeter, vector(-9.9788, -5.9330) * millimeter, vector(-9.8658, -6.0842) * millimeter, vector(-9.7495, -6.2334) * millimeter, vector(-9.6302, -6.3808) * millimeter, vector(-9.5082, -6.5267) * millimeter, vector(-9.3838, -6.6709) * millimeter, vector(-9.2573, -6.8139) * millimeter, vector(-9.1289, -6.9556) * millimeter, vector(-8.9989, -7.0962) * millimeter, vector(-8.8675, -7.2359) * millimeter, vector(-8.7350, -7.3748) * millimeter, vector(-8.6015, -7.5131) * millimeter, vector(-8.4672, -7.6508) * millimeter, vector(-8.3323, -7.7880) * millimeter, vector(-8.1969, -7.9247) * millimeter, vector(-8.0610, -8.0610) * millimeter, vector(-7.9247, -8.1969) * millimeter, vector(-7.7880, -8.3323) * millimeter, vector(-7.6508, -8.4672) * millimeter, vector(-7.5131, -8.6015) * millimeter, vector(-7.3748, -8.7350) * millimeter, vector(-7.2359, -8.8675) * millimeter, vector(-7.0962, -8.9989) * millimeter, vector(-6.9556, -9.1289) * millimeter, vector(-6.8139, -9.2573) * millimeter, vector(-6.6709, -9.3838) * millimeter, vector(-6.5267, -9.5082) * millimeter, vector(-6.3808, -9.6302) * millimeter, vector(-6.2334, -9.7495) * millimeter, vector(-6.0842, -9.8658) * millimeter, vector(-5.9330, -9.9788) * millimeter, vector(-5.7799, -10.0883) * millimeter, vector(-5.6247, -10.1939) * millimeter, vector(-5.4674, -10.2955) * millimeter, vector(-5.3079, -10.3929) * millimeter, vector(-5.1462, -10.4857) * millimeter, vector(-4.9823, -10.5739) * millimeter, vector(-4.8161, -10.6573) * millimeter, vector(-4.6478, -10.7358) * millimeter, vector(-4.4774, -10.8094) * millimeter, vector(-4.3049, -10.8779) * millimeter, vector(-4.1303, -10.9414) * millimeter, vector(-3.9539, -10.9999) * millimeter, vector(-3.7756, -11.0534) * millimeter, vector(-3.5956, -11.1021) * millimeter, vector(-3.4140, -11.1461) * millimeter, vector(-3.2309, -11.1855) * millimeter, vector(-3.0465, -11.2205) * millimeter, vector(-2.8608, -11.2514) * millimeter, vector(-2.6740, -11.2783) * millimeter, vector(-2.4863, -11.3016) * millimeter, vector(-2.2976, -11.3215) * millimeter, vector(-2.1083, -11.3384) * millimeter, vector(-1.9183, -11.3524) * millimeter, vector(-1.7277, -11.3640) * millimeter, vector(-1.5367, -11.3734) * millimeter, vector(-1.3454, -11.3809) * millimeter, vector(-1.1537, -11.3868) * millimeter, vector(-0.9617, -11.3914) * millimeter, vector(-0.7696, -11.3947) * millimeter, vector(-0.5773, -11.3972) * millimeter, vector(-0.3849, -11.3988) * millimeter, vector(-0.1925, -11.3997) * millimeter, vector(-0.0000, -11.4000) * millimeter, vector(0.1925, -11.3997) * millimeter, vector(0.3849, -11.3988) * millimeter, vector(0.5773, -11.3972) * millimeter, vector(0.7696, -11.3947) * millimeter, vector(0.9617, -11.3914) * millimeter, vector(1.1537, -11.3868) * millimeter, vector(1.3454, -11.3809) * millimeter, vector(1.5367, -11.3734) * millimeter, vector(1.7277, -11.3640) * millimeter, vector(1.9183, -11.3524) * millimeter, vector(2.1083, -11.3384) * millimeter, vector(2.2976, -11.3215) * millimeter, vector(2.4863, -11.3016) * millimeter, vector(2.6740, -11.2783) * millimeter, vector(2.8608, -11.2514) * millimeter, vector(3.0465, -11.2205) * millimeter, vector(3.2309, -11.1855) * millimeter, vector(3.4140, -11.1461) * millimeter, vector(3.5956, -11.1021) * millimeter, vector(3.7756, -11.0534) * millimeter, vector(3.9539, -10.9999) * millimeter, vector(4.1303, -10.9414) * millimeter, vector(4.3049, -10.8779) * millimeter, vector(4.4774, -10.8094) * millimeter, vector(4.6478, -10.7358) * millimeter, vector(4.8161, -10.6573) * millimeter, vector(4.9823, -10.5739) * millimeter, vector(5.1462, -10.4857) * millimeter, vector(5.3079, -10.3929) * millimeter, vector(5.4674, -10.2955) * millimeter, vector(5.6247, -10.1939) * millimeter, vector(5.7799, -10.0883) * millimeter, vector(5.9330, -9.9788) * millimeter, vector(6.0842, -9.8658) * millimeter, vector(6.2334, -9.7495) * millimeter, vector(6.3808, -9.6302) * millimeter, vector(6.5267, -9.5082) * millimeter, vector(6.6709, -9.3838) * millimeter, vector(6.8139, -9.2573) * millimeter, vector(6.9556, -9.1289) * millimeter, vector(7.0962, -8.9989) * millimeter, vector(7.2359, -8.8675) * millimeter, vector(7.3748, -8.7350) * millimeter, vector(7.5131, -8.6015) * millimeter, vector(7.6508, -8.4672) * millimeter, vector(7.7880, -8.3323) * millimeter, vector(7.9247, -8.1969) * millimeter, vector(8.0610, -8.0610) * millimeter, vector(8.1969, -7.9247) * millimeter, vector(8.3323, -7.7880) * millimeter, vector(8.4672, -7.6508) * millimeter, vector(8.6015, -7.5131) * millimeter, vector(8.7350, -7.3748) * millimeter, vector(8.8675, -7.2359) * millimeter, vector(8.9989, -7.0962) * millimeter, vector(9.1289, -6.9556) * millimeter, vector(9.2573, -6.8139) * millimeter, vector(9.3838, -6.6709) * millimeter, vector(9.5082, -6.5267) * millimeter, vector(9.6302, -6.3808) * millimeter, vector(9.7495, -6.2334) * millimeter, vector(9.8658, -6.0842) * millimeter, vector(9.9788, -5.9330) * millimeter, vector(10.0883, -5.7799) * millimeter, vector(10.1939, -5.6247) * millimeter, vector(10.2955, -5.4674) * millimeter, vector(10.3929, -5.3079) * millimeter, vector(10.4857, -5.1462) * millimeter, vector(10.5739, -4.9823) * millimeter, vector(10.6573, -4.8161) * millimeter, vector(10.7358, -4.6478) * millimeter, vector(10.8094, -4.4774) * millimeter, vector(10.8779, -4.3049) * millimeter, vector(10.9414, -4.1303) * millimeter, vector(10.9999, -3.9539) * millimeter, vector(11.0534, -3.7756) * millimeter, vector(11.1021, -3.5956) * millimeter, vector(11.1461, -3.4140) * millimeter, vector(11.1855, -3.2309) * millimeter, vector(11.2205, -3.0465) * millimeter, vector(11.2514, -2.8608) * millimeter, vector(11.2783, -2.6740) * millimeter, vector(11.3016, -2.4863) * millimeter, vector(11.3215, -2.2976) * millimeter, vector(11.3384, -2.1083) * millimeter, vector(11.3524, -1.9183) * millimeter, vector(11.3640, -1.7277) * millimeter, vector(11.3734, -1.5367) * millimeter, vector(11.3809, -1.3454) * millimeter, vector(11.3868, -1.1537) * millimeter, vector(11.3914, -0.9617) * millimeter, vector(11.3947, -0.7696) * millimeter, vector(11.3972, -0.5773) * millimeter, vector(11.3988, -0.3849) * millimeter, vector(11.3997, -0.1925) * millimeter, vector(11.4000, -0.0000) * millimeter, vector(11.3997, 0.1925) * millimeter, vector(11.3988, 0.3849) * millimeter, vector(11.3972, 0.5773) * millimeter, vector(11.3947, 0.7696) * millimeter, vector(11.3914, 0.9617) * millimeter, vector(11.3868, 1.1537) * millimeter, vector(11.3809, 1.3454) * millimeter, vector(11.3734, 1.5367) * millimeter, vector(11.3640, 1.7277) * millimeter, vector(11.3524, 1.9183) * millimeter, vector(11.3384, 2.1083) * millimeter, vector(11.3215, 2.2976) * millimeter, vector(11.3016, 2.4863) * millimeter, vector(11.2783, 2.6740) * millimeter, vector(11.2514, 2.8608) * millimeter, vector(11.2205, 3.0465) * millimeter, vector(11.1855, 3.2309) * millimeter, vector(11.1461, 3.4140) * millimeter, vector(11.1021, 3.5956) * millimeter, vector(11.0534, 3.7756) * millimeter, vector(10.9999, 3.9539) * millimeter, vector(10.9414, 4.1303) * millimeter, vector(10.8779, 4.3049) * millimeter, vector(10.8094, 4.4774) * millimeter, vector(10.7358, 4.6478) * millimeter, vector(10.6573, 4.8161) * millimeter, vector(10.5739, 4.9823) * millimeter, vector(10.4857, 5.1462) * millimeter, vector(10.3929, 5.3079) * millimeter, vector(10.2955, 5.4674) * millimeter, vector(10.1939, 5.6247) * millimeter, vector(10.0883, 5.7799) * millimeter, vector(9.9788, 5.9330) * millimeter, vector(9.8658, 6.0842) * millimeter, vector(9.7495, 6.2334) * millimeter, vector(9.6302, 6.3808) * millimeter, vector(9.5082, 6.5267) * millimeter, vector(9.3838, 6.6709) * millimeter, vector(9.2573, 6.8139) * millimeter, vector(9.1289, 6.9556) * millimeter, vector(8.9989, 7.0962) * millimeter, vector(8.8675, 7.2359) * millimeter, vector(8.7350, 7.3748) * millimeter, vector(8.6015, 7.5131) * millimeter, vector(8.4672, 7.6508) * millimeter, vector(8.3323, 7.7880) * millimeter, vector(8.1969, 7.9247) * millimeter, vector(8.0610, 8.0610) * millimeter, vector(7.9247, 8.1969) * millimeter, vector(7.7880, 8.3323) * millimeter, vector(7.6508, 8.4672) * millimeter, vector(7.5131, 8.6015) * millimeter, vector(7.3748, 8.7350) * millimeter, vector(7.2359, 8.8675) * millimeter, vector(7.0962, 8.9989) * millimeter, vector(6.9556, 9.1289) * millimeter, vector(6.8139, 9.2573) * millimeter, vector(6.6709, 9.3838) * millimeter, vector(6.5267, 9.5082) * millimeter, vector(6.3808, 9.6302) * millimeter, vector(6.2334, 9.7495) * millimeter, vector(6.0842, 9.8658) * millimeter, vector(5.9330, 9.9788) * millimeter, vector(5.7799, 10.0883) * millimeter, vector(5.6247, 10.1939) * millimeter, vector(5.4674, 10.2955) * millimeter, vector(5.3079, 10.3929) * millimeter, vector(5.1462, 10.4857) * millimeter, vector(4.9823, 10.5739) * millimeter, vector(4.8161, 10.6573) * millimeter, vector(4.6478, 10.7358) * millimeter, vector(4.4774, 10.8094) * millimeter, vector(4.3049, 10.8779) * millimeter, vector(4.1303, 10.9414) * millimeter, vector(3.9539, 10.9999) * millimeter, vector(3.7756, 11.0534) * millimeter, vector(3.5956, 11.1021) * millimeter, vector(3.4140, 11.1461) * millimeter, vector(3.2309, 11.1855) * millimeter, vector(3.0465, 11.2205) * millimeter, vector(2.8608, 11.2514) * millimeter, vector(2.6740, 11.2783) * millimeter, vector(2.4863, 11.3016) * millimeter, vector(2.2976, 11.3215) * millimeter, vector(2.1083, 11.3384) * millimeter, vector(1.9183, 11.3524) * millimeter, vector(1.7277, 11.3640) * millimeter, vector(1.5367, 11.3734) * millimeter, vector(1.3454, 11.3809) * millimeter, vector(1.1537, 11.3868) * millimeter, vector(0.9617, 11.3914) * millimeter, vector(0.7696, 11.3947) * millimeter, vector(0.5773, 11.3972) * millimeter, vector(0.3849, 11.3988) * millimeter, vector(0.1925, 11.3997) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.2064, 11.4029) * millimeter, vector(-0.4127, 11.4113) * millimeter, vector(-0.6188, 11.4251) * millimeter, vector(-0.8247, 11.4438) * millimeter, vector(-1.0303, 11.4665) * millimeter, vector(-1.2357, 11.4927) * millimeter, vector(-1.4408, 11.5213) * millimeter, vector(-1.6457, 11.5514) * millimeter, vector(-1.8503, 11.5820) * millimeter, vector(-2.0546, 11.6119) * millimeter, vector(-2.2585, 11.6402) * millimeter, vector(-2.4618, 11.6658) * millimeter, vector(-2.6644, 11.6875) * millimeter, vector(-2.8658, 11.7045) * millimeter, vector(-3.0658, 11.7159) * millimeter, vector(-3.2640, 11.7207) * millimeter, vector(-3.4601, 11.7182) * millimeter, vector(-3.6537, 11.7078) * millimeter, vector(-3.8444, 11.6890) * millimeter, vector(-4.0319, 11.6613) * millimeter, vector(-4.2159, 11.6243) * millimeter, vector(-4.3963, 11.5779) * millimeter, vector(-4.5728, 11.5219) * millimeter, vector(-4.7453, 11.4561) * millimeter, vector(-4.9138, 11.3806) * millimeter, vector(-5.0782, 11.2955) * millimeter, vector(-5.2385, 11.2008) * millimeter, vector(-5.3948, 11.0968) * millimeter, vector(-5.5469, 10.9838) * millimeter, vector(-5.6951, 10.8623) * millimeter, vector(-5.8393, 10.7327) * millimeter, vector(-5.9797, 10.5958) * millimeter, vector(-6.1165, 10.4522) * millimeter, vector(-6.2500, 10.3028) * millimeter, vector(-6.3804, 10.1483) * millimeter, vector(-6.5082, 9.9897) * millimeter, vector(-6.6339, 9.8279) * millimeter, vector(-6.7580, 9.6637) * millimeter, vector(-6.8813, 9.4981) * millimeter, vector(-7.0044, 9.3318) * millimeter, vector(-7.1280, 9.1656) * millimeter, vector(-7.2528, 9.0003) * millimeter, vector(-7.3795, 8.8366) * millimeter, vector(-7.5088, 8.6751) * millimeter, vector(-7.6412, 8.5164) * millimeter, vector(-7.7772, 8.3609) * millimeter, vector(-7.9171, 8.2090) * millimeter, vector(-8.0610, 8.0610) * millimeter, vector(-8.2090, 7.9171) * millimeter, vector(-8.3609, 7.7772) * millimeter, vector(-8.5164, 7.6412) * millimeter, vector(-8.6751, 7.5088) * millimeter, vector(-8.8366, 7.3795) * millimeter, vector(-9.0003, 7.2528) * millimeter, vector(-9.1656, 7.1280) * millimeter, vector(-9.3318, 7.0044) * millimeter, vector(-9.4981, 6.8813) * millimeter, vector(-9.6637, 6.7580) * millimeter, vector(-9.8279, 6.6339) * millimeter, vector(-9.9897, 6.5082) * millimeter, vector(-10.1483, 6.3804) * millimeter, vector(-10.3028, 6.2500) * millimeter, vector(-10.4522, 6.1165) * millimeter, vector(-10.5958, 5.9797) * millimeter, vector(-10.7327, 5.8393) * millimeter, vector(-10.8623, 5.6951) * millimeter, vector(-10.9838, 5.5469) * millimeter, vector(-11.0968, 5.3948) * millimeter, vector(-11.2008, 5.2385) * millimeter, vector(-11.2955, 5.0782) * millimeter, vector(-11.3806, 4.9138) * millimeter, vector(-11.4561, 4.7453) * millimeter, vector(-11.5219, 4.5728) * millimeter, vector(-11.5779, 4.3963) * millimeter, vector(-11.6243, 4.2159) * millimeter, vector(-11.6613, 4.0319) * millimeter, vector(-11.6890, 3.8444) * millimeter, vector(-11.7078, 3.6537) * millimeter, vector(-11.7182, 3.4601) * millimeter, vector(-11.7207, 3.2640) * millimeter, vector(-11.7159, 3.0658) * millimeter, vector(-11.7045, 2.8658) * millimeter, vector(-11.6875, 2.6644) * millimeter, vector(-11.6658, 2.4618) * millimeter, vector(-11.6402, 2.2585) * millimeter, vector(-11.6119, 2.0546) * millimeter, vector(-11.5820, 1.8503) * millimeter, vector(-11.5514, 1.6457) * millimeter, vector(-11.5213, 1.4408) * millimeter, vector(-11.4927, 1.2357) * millimeter, vector(-11.4665, 1.0303) * millimeter, vector(-11.4438, 0.8247) * millimeter, vector(-11.4251, 0.6188) * millimeter, vector(-11.4113, 0.4127) * millimeter, vector(-11.4029, 0.2064) * millimeter, vector(-11.4000, 0.0000) * millimeter, vector(-11.4029, -0.2064) * millimeter, vector(-11.4113, -0.4127) * millimeter, vector(-11.4251, -0.6188) * millimeter, vector(-11.4438, -0.8247) * millimeter, vector(-11.4665, -1.0303) * millimeter, vector(-11.4927, -1.2357) * millimeter, vector(-11.5213, -1.4408) * millimeter, vector(-11.5514, -1.6457) * millimeter, vector(-11.5820, -1.8503) * millimeter, vector(-11.6119, -2.0546) * millimeter, vector(-11.6402, -2.2585) * millimeter, vector(-11.6658, -2.4618) * millimeter, vector(-11.6875, -2.6644) * millimeter, vector(-11.7045, -2.8658) * millimeter, vector(-11.7159, -3.0658) * millimeter, vector(-11.7207, -3.2640) * millimeter, vector(-11.7182, -3.4601) * millimeter, vector(-11.7078, -3.6537) * millimeter, vector(-11.6890, -3.8444) * millimeter, vector(-11.6613, -4.0319) * millimeter, vector(-11.6243, -4.2159) * millimeter, vector(-11.5779, -4.3963) * millimeter, vector(-11.5219, -4.5728) * millimeter, vector(-11.4561, -4.7453) * millimeter, vector(-11.3806, -4.9138) * millimeter, vector(-11.2955, -5.0782) * millimeter, vector(-11.2008, -5.2385) * millimeter, vector(-11.0968, -5.3948) * millimeter, vector(-10.9838, -5.5469) * millimeter, vector(-10.8623, -5.6951) * millimeter, vector(-10.7327, -5.8393) * millimeter, vector(-10.5958, -5.9797) * millimeter, vector(-10.4522, -6.1165) * millimeter, vector(-10.3028, -6.2500) * millimeter, vector(-10.1483, -6.3804) * millimeter, vector(-9.9897, -6.5082) * millimeter, vector(-9.8279, -6.6339) * millimeter, vector(-9.6637, -6.7580) * millimeter, vector(-9.4981, -6.8813) * millimeter, vector(-9.3318, -7.0044) * millimeter, vector(-9.1656, -7.1280) * millimeter, vector(-9.0003, -7.2528) * millimeter, vector(-8.8366, -7.3795) * millimeter, vector(-8.6751, -7.5088) * millimeter, vector(-8.5164, -7.6412) * millimeter, vector(-8.3609, -7.7772) * millimeter, vector(-8.2090, -7.9171) * millimeter, vector(-8.0610, -8.0610) * millimeter, vector(-7.9171, -8.2090) * millimeter, vector(-7.7772, -8.3609) * millimeter, vector(-7.6412, -8.5164) * millimeter, vector(-7.5088, -8.6751) * millimeter, vector(-7.3795, -8.8366) * millimeter, vector(-7.2528, -9.0003) * millimeter, vector(-7.1280, -9.1656) * millimeter, vector(-7.0044, -9.3318) * millimeter, vector(-6.8813, -9.4981) * millimeter, vector(-6.7580, -9.6637) * millimeter, vector(-6.6339, -9.8279) * millimeter, vector(-6.5082, -9.9897) * millimeter, vector(-6.3804, -10.1483) * millimeter, vector(-6.2500, -10.3028) * millimeter, vector(-6.1165, -10.4522) * millimeter, vector(-5.9797, -10.5958) * millimeter, vector(-5.8393, -10.7327) * millimeter, vector(-5.6951, -10.8623) * millimeter, vector(-5.5469, -10.9838) * millimeter, vector(-5.3948, -11.0968) * millimeter, vector(-5.2385, -11.2008) * millimeter, vector(-5.0782, -11.2955) * millimeter, vector(-4.9138, -11.3806) * millimeter, vector(-4.7453, -11.4561) * millimeter, vector(-4.5728, -11.5219) * millimeter, vector(-4.3963, -11.5779) * millimeter, vector(-4.2159, -11.6243) * millimeter, vector(-4.0319, -11.6613) * millimeter, vector(-3.8444, -11.6890) * millimeter, vector(-3.6537, -11.7078) * millimeter, vector(-3.4601, -11.7182) * millimeter, vector(-3.2640, -11.7207) * millimeter, vector(-3.0658, -11.7159) * millimeter, vector(-2.8658, -11.7045) * millimeter, vector(-2.6644, -11.6875) * millimeter, vector(-2.4618, -11.6658) * millimeter, vector(-2.2585, -11.6402) * millimeter, vector(-2.0546, -11.6119) * millimeter, vector(-1.8503, -11.5820) * millimeter, vector(-1.6457, -11.5514) * millimeter, vector(-1.4408, -11.5213) * millimeter, vector(-1.2357, -11.4927) * millimeter, vector(-1.0303, -11.4665) * millimeter, vector(-0.8247, -11.4438) * millimeter, vector(-0.6188, -11.4251) * millimeter, vector(-0.4127, -11.4113) * millimeter, vector(-0.2064, -11.4029) * millimeter, vector(-0.0000, -11.4000) * millimeter, vector(0.2064, -11.4029) * millimeter, vector(0.4127, -11.4113) * millimeter, vector(0.6188, -11.4251) * millimeter, vector(0.8247, -11.4438) * millimeter, vector(1.0303, -11.4665) * millimeter, vector(1.2357, -11.4927) * millimeter, vector(1.4408, -11.5213) * millimeter, vector(1.6457, -11.5514) * millimeter, vector(1.8503, -11.5820) * millimeter, vector(2.0546, -11.6119) * millimeter, vector(2.2585, -11.6402) * millimeter, vector(2.4618, -11.6658) * millimeter, vector(2.6644, -11.6875) * millimeter, vector(2.8658, -11.7045) * millimeter, vector(3.0658, -11.7159) * millimeter, vector(3.2640, -11.7207) * millimeter, vector(3.4601, -11.7182) * millimeter, vector(3.6537, -11.7078) * millimeter, vector(3.8444, -11.6890) * millimeter, vector(4.0319, -11.6613) * millimeter, vector(4.2159, -11.6243) * millimeter, vector(4.3963, -11.5779) * millimeter, vector(4.5728, -11.5219) * millimeter, vector(4.7453, -11.4561) * millimeter, vector(4.9138, -11.3806) * millimeter, vector(5.0782, -11.2955) * millimeter, vector(5.2385, -11.2008) * millimeter, vector(5.3948, -11.0968) * millimeter, vector(5.5469, -10.9838) * millimeter, vector(5.6951, -10.8623) * millimeter, vector(5.8393, -10.7327) * millimeter, vector(5.9797, -10.5958) * millimeter, vector(6.1165, -10.4522) * millimeter, vector(6.2500, -10.3028) * millimeter, vector(6.3804, -10.1483) * millimeter, vector(6.5082, -9.9897) * millimeter, vector(6.6339, -9.8279) * millimeter, vector(6.7580, -9.6637) * millimeter, vector(6.8813, -9.4981) * millimeter, vector(7.0044, -9.3318) * millimeter, vector(7.1280, -9.1656) * millimeter, vector(7.2528, -9.0003) * millimeter, vector(7.3795, -8.8366) * millimeter, vector(7.5088, -8.6751) * millimeter, vector(7.6412, -8.5164) * millimeter, vector(7.7772, -8.3609) * millimeter, vector(7.9171, -8.2090) * millimeter, vector(8.0610, -8.0610) * millimeter, vector(8.2090, -7.9171) * millimeter, vector(8.3609, -7.7772) * millimeter, vector(8.5164, -7.6412) * millimeter, vector(8.6751, -7.5088) * millimeter, vector(8.8366, -7.3795) * millimeter, vector(9.0003, -7.2528) * millimeter, vector(9.1656, -7.1280) * millimeter, vector(9.3318, -7.0044) * millimeter, vector(9.4981, -6.8813) * millimeter, vector(9.6637, -6.7580) * millimeter, vector(9.8279, -6.6339) * millimeter, vector(9.9897, -6.5082) * millimeter, vector(10.1483, -6.3804) * millimeter, vector(10.3028, -6.2500) * millimeter, vector(10.4522, -6.1165) * millimeter, vector(10.5958, -5.9797) * millimeter, vector(10.7327, -5.8393) * millimeter, vector(10.8623, -5.6951) * millimeter, vector(10.9838, -5.5469) * millimeter, vector(11.0968, -5.3948) * millimeter, vector(11.2008, -5.2385) * millimeter, vector(11.2955, -5.0782) * millimeter, vector(11.3806, -4.9138) * millimeter, vector(11.4561, -4.7453) * millimeter, vector(11.5219, -4.5728) * millimeter, vector(11.5779, -4.3963) * millimeter, vector(11.6243, -4.2159) * millimeter, vector(11.6613, -4.0319) * millimeter, vector(11.6890, -3.8444) * millimeter, vector(11.7078, -3.6537) * millimeter, vector(11.7182, -3.4601) * millimeter, vector(11.7207, -3.2640) * millimeter, vector(11.7159, -3.0658) * millimeter, vector(11.7045, -2.8658) * millimeter, vector(11.6875, -2.6644) * millimeter, vector(11.6658, -2.4618) * millimeter, vector(11.6402, -2.2585) * millimeter, vector(11.6119, -2.0546) * millimeter, vector(11.5820, -1.8503) * millimeter, vector(11.5514, -1.6457) * millimeter, vector(11.5213, -1.4408) * millimeter, vector(11.4927, -1.2357) * millimeter, vector(11.4665, -1.0303) * millimeter, vector(11.4438, -0.8247) * millimeter, vector(11.4251, -0.6188) * millimeter, vector(11.4113, -0.4127) * millimeter, vector(11.4029, -0.2064) * millimeter, vector(11.4000, -0.0000) * millimeter, vector(11.4029, 0.2064) * millimeter, vector(11.4113, 0.4127) * millimeter, vector(11.4251, 0.6188) * millimeter, vector(11.4438, 0.8247) * millimeter, vector(11.4665, 1.0303) * millimeter, vector(11.4927, 1.2357) * millimeter, vector(11.5213, 1.4408) * millimeter, vector(11.5514, 1.6457) * millimeter, vector(11.5820, 1.8503) * millimeter, vector(11.6119, 2.0546) * millimeter, vector(11.6402, 2.2585) * millimeter, vector(11.6658, 2.4618) * millimeter, vector(11.6875, 2.6644) * millimeter, vector(11.7045, 2.8658) * millimeter, vector(11.7159, 3.0658) * millimeter, vector(11.7207, 3.2640) * millimeter, vector(11.7182, 3.4601) * millimeter, vector(11.7078, 3.6537) * millimeter, vector(11.6890, 3.8444) * millimeter, vector(11.6613, 4.0319) * millimeter, vector(11.6243, 4.2159) * millimeter, vector(11.5779, 4.3963) * millimeter, vector(11.5219, 4.5728) * millimeter, vector(11.4561, 4.7453) * millimeter, vector(11.3806, 4.9138) * millimeter, vector(11.2955, 5.0782) * millimeter, vector(11.2008, 5.2385) * millimeter, vector(11.0968, 5.3948) * millimeter, vector(10.9838, 5.5469) * millimeter, vector(10.8623, 5.6951) * millimeter, vector(10.7327, 5.8393) * millimeter, vector(10.5958, 5.9797) * millimeter, vector(10.4522, 6.1165) * millimeter, vector(10.3028, 6.2500) * millimeter, vector(10.1483, 6.3804) * millimeter, vector(9.9897, 6.5082) * millimeter, vector(9.8279, 6.6339) * millimeter, vector(9.6637, 6.7580) * millimeter, vector(9.4981, 6.8813) * millimeter, vector(9.3318, 7.0044) * millimeter, vector(9.1656, 7.1280) * millimeter, vector(9.0003, 7.2528) * millimeter, vector(8.8366, 7.3795) * millimeter, vector(8.6751, 7.5088) * millimeter, vector(8.5164, 7.6412) * millimeter, vector(8.3609, 7.7772) * millimeter, vector(8.2090, 7.9171) * millimeter, vector(8.0610, 8.0610) * millimeter, vector(7.9171, 8.2090) * millimeter, vector(7.7772, 8.3609) * millimeter, vector(7.6412, 8.5164) * millimeter, vector(7.5088, 8.6751) * millimeter, vector(7.3795, 8.8366) * millimeter, vector(7.2528, 9.0003) * millimeter, vector(7.1280, 9.1656) * millimeter, vector(7.0044, 9.3318) * millimeter, vector(6.8813, 9.4981) * millimeter, vector(6.7580, 9.6637) * millimeter, vector(6.6339, 9.8279) * millimeter, vector(6.5082, 9.9897) * millimeter, vector(6.3804, 10.1483) * millimeter, vector(6.2500, 10.3028) * millimeter, vector(6.1165, 10.4522) * millimeter, vector(5.9797, 10.5958) * millimeter, vector(5.8393, 10.7327) * millimeter, vector(5.6951, 10.8623) * millimeter, vector(5.5469, 10.9838) * millimeter, vector(5.3948, 11.0968) * millimeter, vector(5.2385, 11.2008) * millimeter, vector(5.0782, 11.2955) * millimeter, vector(4.9138, 11.3806) * millimeter, vector(4.7453, 11.4561) * millimeter, vector(4.5728, 11.5219) * millimeter, vector(4.3963, 11.5779) * millimeter, vector(4.2159, 11.6243) * millimeter, vector(4.0319, 11.6613) * millimeter, vector(3.8444, 11.6890) * millimeter, vector(3.6537, 11.7078) * millimeter, vector(3.4601, 11.7182) * millimeter, vector(3.2640, 11.7207) * millimeter, vector(3.0658, 11.7159) * millimeter, vector(2.8658, 11.7045) * millimeter, vector(2.6644, 11.6875) * millimeter, vector(2.4618, 11.6658) * millimeter, vector(2.2585, 11.6402) * millimeter, vector(2.0546, 11.6119) * millimeter, vector(1.8503, 11.5820) * millimeter, vector(1.6457, 11.5514) * millimeter, vector(1.4408, 11.5213) * millimeter, vector(1.2357, 11.4927) * millimeter, vector(1.0303, 11.4665) * millimeter, vector(0.8247, 11.4438) * millimeter, vector(0.6188, 11.4251) * millimeter, vector(0.4127, 11.4113) * millimeter, vector(0.2064, 11.4029) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.2020, 11.3989) * millimeter, vector(-0.4039, 11.3956) * millimeter, vector(-0.6058, 11.3901) * millimeter, vector(-0.8078, 11.3825) * millimeter, vector(-1.0097, 11.3725) * millimeter, vector(-1.2117, 11.3604) * millimeter, vector(-1.4136, 11.3460) * millimeter, vector(-1.6155, 11.3292) * millimeter, vector(-1.8174, 11.3102) * millimeter, vector(-2.0192, 11.2887) * millimeter, vector(-2.2210, 11.2649) * millimeter, vector(-2.4227, 11.2386) * millimeter, vector(-2.6243, 11.2098) * millimeter, vector(-2.8258, 11.1784) * millimeter, vector(-3.0272, 11.1444) * millimeter, vector(-3.2284, 11.1078) * millimeter, vector(-3.4293, 11.0684) * millimeter, vector(-3.6300, 11.0263) * millimeter, vector(-3.8305, 10.9813) * millimeter, vector(-4.0306, 10.9334) * millimeter, vector(-4.2302, 10.8825) * millimeter, vector(-4.4295, 10.8287) * millimeter, vector(-4.6282, 10.7717) * millimeter, vector(-4.8263, 10.7116) * millimeter, vector(-5.0238, 10.6482) * millimeter, vector(-5.2206, 10.5816) * millimeter, vector(-5.4165, 10.5117) * millimeter, vector(-5.6116, 10.4384) * millimeter, vector(-5.8056, 10.3616) * millimeter, vector(-5.9986, 10.2814) * millimeter, vector(-6.1905, 10.1977) * millimeter, vector(-6.3810, 10.1103) * millimeter, vector(-6.5702, 10.0194) * millimeter, vector(-6.7579, 9.9249) * millimeter, vector(-6.9440, 9.8266) * millimeter, vector(-7.1284, 9.7247) * millimeter, vector(-7.3110, 9.6190) * millimeter, vector(-7.4917, 9.5096) * millimeter, vector(-7.6703, 9.3965) * millimeter, vector(-7.8467, 9.2796) * millimeter, vector(-8.0209, 9.1590) * millimeter, vector(-8.1926, 9.0346) * millimeter, vector(-8.3618, 8.9065) * millimeter, vector(-8.5284, 8.7747) * millimeter, vector(-8.6921, 8.6392) * millimeter, vector(-8.8530, 8.5001) * millimeter, vector(-9.0109, 8.3573) * millimeter, vector(-9.1656, 8.2110) * millimeter, vector(-9.3171, 8.0611) * millimeter, vector(-9.4652, 7.9078) * millimeter, vector(-9.6098, 7.7511) * millimeter, vector(-9.7509, 7.5910) * millimeter, vector(-9.8882, 7.4277) * millimeter, vector(-10.0218, 7.2612) * millimeter, vector(-10.1515, 7.0915) * millimeter, vector(-10.2772, 6.9189) * millimeter, vector(-10.3989, 6.7433) * millimeter, vector(-10.5164, 6.5649) * millimeter, vector(-10.6297, 6.3838) * millimeter, vector(-10.7387, 6.2000) * millimeter, vector(-10.8434, 6.0137) * millimeter, vector(-10.9436, 5.8250) * millimeter, vector(-11.0393, 5.6340) * millimeter, vector(-11.1306, 5.4409) * millimeter, vector(-11.2172, 5.2457) * millimeter, vector(-11.2993, 5.0485) * millimeter, vector(-11.3767, 4.8496) * millimeter, vector(-11.4495, 4.6490) * millimeter, vector(-11.5175, 4.4468) * millimeter, vector(-11.5810, 4.2432) * millimeter, vector(-11.6397, 4.0382) * millimeter, vector(-11.6937, 3.8321) * millimeter, vector(-11.7431, 3.6250) * millimeter, vector(-11.7878, 3.4169) * millimeter, vector(-11.8278, 3.2080) * millimeter, vector(-11.8633, 2.9985) * millimeter, vector(-11.8942, 2.7883) * millimeter, vector(-11.9205, 2.5777) * millimeter, vector(-11.9423, 2.3668) * millimeter, vector(-11.9597, 2.1557) * millimeter, vector(-11.9727, 1.9444) * millimeter, vector(-11.9814, 1.7332) * millimeter, vector(-11.9858, 1.5220) * millimeter, vector(-11.9860, 1.3111) * millimeter, vector(-11.9821, 1.1004) * millimeter, vector(-11.9741, 0.8901) * millimeter, vector(-11.9622, 0.6803) * millimeter, vector(-11.9463, 0.4709) * millimeter, vector(-11.9267, 0.2623) * millimeter, vector(-11.9033, 0.0543) * millimeter, vector(-11.8763, -0.1530) * millimeter, vector(-11.8457, -0.3594) * millimeter, vector(-11.8116, -0.5650) * millimeter, vector(-11.7742, -0.7697) * millimeter, vector(-11.7335, -0.9734) * millimeter, vector(-11.6896, -1.1761) * millimeter, vector(-11.6426, -1.3777) * millimeter, vector(-11.5926, -1.5783) * millimeter, vector(-11.5397, -1.7778) * millimeter, vector(-11.4839, -1.9761) * millimeter, vector(-11.4253, -2.1734) * millimeter, vector(-11.3641, -2.3694) * millimeter, vector(-11.3002, -2.5643) * millimeter, vector(-11.2338, -2.7581) * millimeter, vector(-11.1650, -2.9506) * millimeter, vector(-11.0937, -3.1420) * millimeter, vector(-11.0201, -3.3322) * millimeter, vector(-10.9443, -3.5212) * millimeter, vector(-10.8662, -3.7090) * millimeter, vector(-10.7859, -3.8957) * millimeter, vector(-10.7036, -4.0812) * millimeter, vector(-10.6192, -4.2656) * millimeter, vector(-10.5327, -4.4488) * millimeter, vector(-10.4442, -4.6309) * millimeter, vector(-10.3538, -4.8118) * millimeter, vector(-10.2614, -4.9917) * millimeter, vector(-10.1671, -5.1704) * millimeter, vector(-10.0709, -5.3480) * millimeter, vector(-9.9727, -5.5246) * millimeter, vector(-9.8727, -5.7000) * millimeter, vector(-9.7708, -5.8743) * millimeter, vector(-9.6670, -6.0476) * millimeter, vector(-9.5612, -6.2198) * millimeter, vector(-9.4536, -6.3908) * millimeter, vector(-9.3440, -6.5607) * millimeter, vector(-9.2326, -6.7295) * millimeter, vector(-9.1191, -6.8972) * millimeter, vector(-9.0037, -7.0637) * millimeter, vector(-8.8862, -7.2290) * millimeter, vector(-8.7667, -7.3931) * millimeter, vector(-8.6452, -7.5559) * millimeter, vector(-8.5216, -7.7174) * millimeter, vector(-8.3958, -7.8776) * millimeter, vector(-8.2679, -8.0364) * millimeter, vector(-8.1378, -8.1938) * millimeter, vector(-8.0055, -8.3497) * millimeter, vector(-7.8709, -8.5041) * millimeter, vector(-7.7340, -8.6569) * millimeter, vector(-7.5948, -8.8079) * millimeter, vector(-7.4533, -8.9573) * millimeter, vector(-7.3094, -9.1048) * millimeter, vector(-7.1632, -9.2504) * millimeter, vector(-7.0145, -9.3940) * millimeter, vector(-6.8633, -9.5355) * millimeter, vector(-6.7097, -9.6749) * millimeter, vector(-6.5537, -9.8119) * millimeter, vector(-6.3951, -9.9467) * millimeter, vector(-6.2341, -10.0789) * millimeter, vector(-6.0706, -10.2086) * millimeter, vector(-5.9047, -10.3357) * millimeter, vector(-5.7362, -10.4599) * millimeter, vector(-5.5653, -10.5813) * millimeter, vector(-5.3920, -10.6997) * millimeter, vector(-5.2162, -10.8149) * millimeter, vector(-5.0381, -10.9270) * millimeter, vector(-4.8576, -11.0357) * millimeter, vector(-4.6748, -11.1410) * millimeter, vector(-4.4897, -11.2428) * millimeter, vector(-4.3024, -11.3409) * millimeter, vector(-4.1130, -11.4353) * millimeter, vector(-3.9214, -11.5258) * millimeter, vector(-3.7279, -11.6123) * millimeter, vector(-3.5323, -11.6948) * millimeter, vector(-3.3349, -11.7731) * millimeter, vector(-3.1357, -11.8472) * millimeter, vector(-2.9348, -11.9170) * millimeter, vector(-2.7322, -11.9823) * millimeter, vector(-2.5281, -12.0431) * millimeter, vector(-2.3226, -12.0994) * millimeter, vector(-2.1158, -12.1510) * millimeter, vector(-1.9077, -12.1979) * millimeter, vector(-1.6986, -12.2400) * millimeter, vector(-1.4885, -12.2773) * millimeter, vector(-1.2775, -12.3097) * millimeter, vector(-1.0657, -12.3372) * millimeter, vector(-0.8533, -12.3598) * millimeter, vector(-0.6404, -12.3774) * millimeter, vector(-0.4272, -12.3899) * millimeter, vector(-0.2136, -12.3975) * millimeter, vector(-0.0000, -12.4000) * millimeter, vector(0.2136, -12.3975) * millimeter, vector(0.4272, -12.3899) * millimeter, vector(0.6404, -12.3774) * millimeter, vector(0.8533, -12.3598) * millimeter, vector(1.0657, -12.3372) * millimeter, vector(1.2775, -12.3097) * millimeter, vector(1.4885, -12.2773) * millimeter, vector(1.6986, -12.2400) * millimeter, vector(1.9077, -12.1979) * millimeter, vector(2.1158, -12.1510) * millimeter, vector(2.3226, -12.0994) * millimeter, vector(2.5281, -12.0431) * millimeter, vector(2.7322, -11.9823) * millimeter, vector(2.9348, -11.9170) * millimeter, vector(3.1357, -11.8472) * millimeter, vector(3.3349, -11.7731) * millimeter, vector(3.5323, -11.6948) * millimeter, vector(3.7279, -11.6123) * millimeter, vector(3.9214, -11.5258) * millimeter, vector(4.1130, -11.4353) * millimeter, vector(4.3024, -11.3409) * millimeter, vector(4.4897, -11.2428) * millimeter, vector(4.6748, -11.1410) * millimeter, vector(4.8576, -11.0357) * millimeter, vector(5.0381, -10.9270) * millimeter, vector(5.2162, -10.8149) * millimeter, vector(5.3920, -10.6997) * millimeter, vector(5.5653, -10.5813) * millimeter, vector(5.7362, -10.4599) * millimeter, vector(5.9047, -10.3357) * millimeter, vector(6.0706, -10.2086) * millimeter, vector(6.2341, -10.0789) * millimeter, vector(6.3951, -9.9467) * millimeter, vector(6.5537, -9.8119) * millimeter, vector(6.7097, -9.6749) * millimeter, vector(6.8633, -9.5355) * millimeter, vector(7.0145, -9.3940) * millimeter, vector(7.1632, -9.2504) * millimeter, vector(7.3094, -9.1048) * millimeter, vector(7.4533, -8.9573) * millimeter, vector(7.5948, -8.8079) * millimeter, vector(7.7340, -8.6569) * millimeter, vector(7.8709, -8.5041) * millimeter, vector(8.0055, -8.3497) * millimeter, vector(8.1378, -8.1938) * millimeter, vector(8.2679, -8.0364) * millimeter, vector(8.3958, -7.8776) * millimeter, vector(8.5216, -7.7174) * millimeter, vector(8.6452, -7.5559) * millimeter, vector(8.7667, -7.3931) * millimeter, vector(8.8862, -7.2290) * millimeter, vector(9.0037, -7.0637) * millimeter, vector(9.1191, -6.8972) * millimeter, vector(9.2326, -6.7295) * millimeter, vector(9.3440, -6.5607) * millimeter, vector(9.4536, -6.3908) * millimeter, vector(9.5612, -6.2198) * millimeter, vector(9.6670, -6.0476) * millimeter, vector(9.7708, -5.8743) * millimeter, vector(9.8727, -5.7000) * millimeter, vector(9.9727, -5.5246) * millimeter, vector(10.0709, -5.3480) * millimeter, vector(10.1671, -5.1704) * millimeter, vector(10.2614, -4.9917) * millimeter, vector(10.3538, -4.8118) * millimeter, vector(10.4442, -4.6309) * millimeter, vector(10.5327, -4.4488) * millimeter, vector(10.6192, -4.2656) * millimeter, vector(10.7036, -4.0812) * millimeter, vector(10.7859, -3.8957) * millimeter, vector(10.8662, -3.7090) * millimeter, vector(10.9443, -3.5212) * millimeter, vector(11.0201, -3.3322) * millimeter, vector(11.0937, -3.1420) * millimeter, vector(11.1650, -2.9506) * millimeter, vector(11.2338, -2.7581) * millimeter, vector(11.3002, -2.5643) * millimeter, vector(11.3641, -2.3694) * millimeter, vector(11.4253, -2.1734) * millimeter, vector(11.4839, -1.9761) * millimeter, vector(11.5397, -1.7778) * millimeter, vector(11.5926, -1.5783) * millimeter, vector(11.6426, -1.3777) * millimeter, vector(11.6896, -1.1761) * millimeter, vector(11.7335, -0.9734) * millimeter, vector(11.7742, -0.7697) * millimeter, vector(11.8116, -0.5650) * millimeter, vector(11.8457, -0.3594) * millimeter, vector(11.8763, -0.1530) * millimeter, vector(11.9033, 0.0543) * millimeter, vector(11.9267, 0.2623) * millimeter, vector(11.9463, 0.4709) * millimeter, vector(11.9622, 0.6803) * millimeter, vector(11.9741, 0.8901) * millimeter, vector(11.9821, 1.1004) * millimeter, vector(11.9860, 1.3111) * millimeter, vector(11.9858, 1.5220) * millimeter, vector(11.9814, 1.7332) * millimeter, vector(11.9727, 1.9444) * millimeter, vector(11.9597, 2.1557) * millimeter, vector(11.9423, 2.3668) * millimeter, vector(11.9205, 2.5777) * millimeter, vector(11.8942, 2.7883) * millimeter, vector(11.8633, 2.9985) * millimeter, vector(11.8278, 3.2080) * millimeter, vector(11.7878, 3.4169) * millimeter, vector(11.7431, 3.6250) * millimeter, vector(11.6937, 3.8321) * millimeter, vector(11.6397, 4.0382) * millimeter, vector(11.5810, 4.2432) * millimeter, vector(11.5175, 4.4468) * millimeter, vector(11.4495, 4.6490) * millimeter, vector(11.3767, 4.8496) * millimeter, vector(11.2993, 5.0485) * millimeter, vector(11.2172, 5.2457) * millimeter, vector(11.1306, 5.4409) * millimeter, vector(11.0393, 5.6340) * millimeter, vector(10.9436, 5.8250) * millimeter, vector(10.8434, 6.0137) * millimeter, vector(10.7387, 6.2000) * millimeter, vector(10.6297, 6.3838) * millimeter, vector(10.5164, 6.5649) * millimeter, vector(10.3989, 6.7433) * millimeter, vector(10.2772, 6.9189) * millimeter, vector(10.1515, 7.0915) * millimeter, vector(10.0218, 7.2612) * millimeter, vector(9.8882, 7.4277) * millimeter, vector(9.7509, 7.5910) * millimeter, vector(9.6098, 7.7511) * millimeter, vector(9.4652, 7.9078) * millimeter, vector(9.3171, 8.0611) * millimeter, vector(9.1656, 8.2110) * millimeter, vector(9.0109, 8.3573) * millimeter, vector(8.8530, 8.5001) * millimeter, vector(8.6921, 8.6392) * millimeter, vector(8.5284, 8.7747) * millimeter, vector(8.3618, 8.9065) * millimeter, vector(8.1926, 9.0346) * millimeter, vector(8.0209, 9.1590) * millimeter, vector(7.8467, 9.2796) * millimeter, vector(7.6703, 9.3965) * millimeter, vector(7.4917, 9.5096) * millimeter, vector(7.3110, 9.6190) * millimeter, vector(7.1284, 9.7247) * millimeter, vector(6.9440, 9.8266) * millimeter, vector(6.7579, 9.9249) * millimeter, vector(6.5702, 10.0194) * millimeter, vector(6.3810, 10.1103) * millimeter, vector(6.1905, 10.1977) * millimeter, vector(5.9986, 10.2814) * millimeter, vector(5.8056, 10.3616) * millimeter, vector(5.6116, 10.4384) * millimeter, vector(5.4165, 10.5117) * millimeter, vector(5.2206, 10.5816) * millimeter, vector(5.0238, 10.6482) * millimeter, vector(4.8263, 10.7116) * millimeter, vector(4.6282, 10.7717) * millimeter, vector(4.4295, 10.8287) * millimeter, vector(4.2302, 10.8825) * millimeter, vector(4.0306, 10.9334) * millimeter, vector(3.8305, 10.9813) * millimeter, vector(3.6300, 11.0263) * millimeter, vector(3.4293, 11.0684) * millimeter, vector(3.2284, 11.1078) * millimeter, vector(3.0272, 11.1444) * millimeter, vector(2.8258, 11.1784) * millimeter, vector(2.6243, 11.2098) * millimeter, vector(2.4227, 11.2386) * millimeter, vector(2.2210, 11.2649) * millimeter, vector(2.0192, 11.2887) * millimeter, vector(1.8174, 11.3102) * millimeter, vector(1.6155, 11.3292) * millimeter, vector(1.4136, 11.3460) * millimeter, vector(1.2117, 11.3604) * millimeter, vector(1.0097, 11.3725) * millimeter, vector(0.8078, 11.3825) * millimeter, vector(0.6058, 11.3901) * millimeter, vector(0.4039, 11.3956) * millimeter, vector(0.2020, 11.3989) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.1212, 11.4003) * millimeter, vector(-0.2424, 11.4010) * millimeter, vector(-0.3634, 11.4022) * millimeter, vector(-0.4842, 11.4038) * millimeter, vector(-0.6047, 11.4057) * millimeter, vector(-0.7248, 11.4077) * millimeter, vector(-0.8445, 11.4097) * millimeter, vector(-0.9638, 11.4115) * millimeter, vector(-1.0824, 11.4129) * millimeter, vector(-1.2005, 11.4137) * millimeter, vector(-1.3180, 11.4138) * millimeter, vector(-1.4348, 11.4129) * millimeter, vector(-1.5509, 11.4108) * millimeter, vector(-1.6662, 11.4074) * millimeter, vector(-1.7808, 11.4024) * millimeter, vector(-1.8946, 11.3956) * millimeter, vector(-2.0075, 11.3870) * millimeter, vector(-2.1197, 11.3763) * millimeter, vector(-2.2310, 11.3634) * millimeter, vector(-2.3415, 11.3482) * millimeter, vector(-2.4513, 11.3307) * millimeter, vector(-2.5602, 11.3106) * millimeter, vector(-2.6685, 11.2881) * millimeter, vector(-2.7761, 11.2629) * millimeter, vector(-2.8830, 11.2352) * millimeter, vector(-2.9893, 11.2049) * millimeter, vector(-3.0951, 11.1720) * millimeter, vector(-3.2005, 11.1365) * millimeter, vector(-3.3054, 11.0986) * millimeter, vector(-3.4100, 11.0583) * millimeter, vector(-3.5142, 11.0156) * millimeter, vector(-3.6183, 10.9708) * millimeter, vector(-3.7221, 10.9239) * millimeter, vector(-3.8259, 10.8751) * millimeter, vector(-3.9296, 10.8245) * millimeter, vector(-4.0334, 10.7724) * millimeter, vector(-4.1372, 10.7189) * millimeter, vector(-4.2412, 10.6643) * millimeter, vector(-4.3454, 10.6086) * millimeter, vector(-4.4498, 10.5522) * millimeter, vector(-4.5545, 10.4952) * millimeter, vector(-4.6596, 10.4378) * millimeter, vector(-4.7651, 10.3802) * millimeter, vector(-4.8709, 10.3226) * millimeter, vector(-4.9771, 10.2651) * millimeter, vector(-5.0837, 10.2078) * millimeter, vector(-5.1906, 10.1508) * millimeter, vector(-5.2978, 10.0942) * millimeter, vector(-5.4053, 10.0381) * millimeter, vector(-5.5130, 9.9825) * millimeter, vector(-5.6207, 9.9273) * millimeter, vector(-5.7284, 9.8726) * millimeter, vector(-5.8359, 9.8182) * millimeter, vector(-5.9432, 9.7642) * millimeter, vector(-6.0501, 9.7103) * millimeter, vector(-6.1565, 9.6565) * millimeter, vector(-6.2623, 9.6026) * millimeter, vector(-6.3672, 9.5484) * millimeter, vector(-6.4713, 9.4939) * millimeter, vector(-6.5743, 9.4388) * millimeter, vector(-6.6761, 9.3831) * millimeter, vector(-6.7766, 9.3264) * millimeter, vector(-6.8758, 9.2687) * millimeter, vector(-6.9734, 9.2099) * millimeter, vector(-7.0694, 9.1497) * millimeter, vector(-7.1637, 9.0881) * millimeter, vector(-7.2563, 9.0250) * millimeter, vector(-7.3471, 8.9602) * millimeter, vector(-7.4361, 8.8936) * millimeter, vector(-7.5233, 8.8253) * millimeter, vector(-7.6087, 8.7550) * millimeter, vector(-7.6922, 8.6827) * millimeter, vector(-7.7740, 8.6085) * millimeter, vector(-7.8541, 8.5322) * millimeter, vector(-7.9325, 8.4539) * millimeter, vector(-8.0093, 8.3736) * millimeter, vector(-8.0845, 8.2912) * millimeter, vector(-8.1584, 8.2069) * millimeter, vector(-8.2309, 8.1207) * millimeter, vector(-8.3022, 8.0326) * millimeter, vector(-8.3724, 7.9428) * millimeter, vector(-8.4416, 7.8514) * millimeter, vector(-8.5099, 7.7584) * millimeter, vector(-8.5776, 7.6641) * millimeter, vector(-8.6447, 7.5685) * millimeter, vector(-8.7113, 7.4718) * millimeter, vector(-8.7777, 7.3741) * millimeter, vector(-8.8440, 7.2756) * millimeter, vector(-8.9102, 7.1765) * millimeter, vector(-8.9766, 7.0768) * millimeter, vector(-9.0432, 6.9768) * millimeter, vector(-9.1101, 6.8766) * millimeter, vector(-9.1774, 6.7763) * millimeter, vector(-9.2452, 6.6760) * millimeter, vector(-9.3134, 6.5759) * millimeter, vector(-9.3820, 6.4759) * millimeter, vector(-9.4511, 6.3763) * millimeter, vector(-9.5206, 6.2770) * millimeter, vector(-9.5903, 6.1781) * millimeter, vector(-9.6602, 6.0796) * millimeter, vector(-9.7302, 5.9815) * millimeter, vector(-9.8001, 5.8838) * millimeter, vector(-9.8697, 5.7864) * millimeter, vector(-9.9389, 5.6893) * millimeter, vector(-10.0075, 5.5924) * millimeter, vector(-10.0753, 5.4957) * millimeter, vector(-10.1421, 5.3991) * millimeter, vector(-10.2077, 5.3025) * millimeter, vector(-10.2719, 5.2057) * millimeter, vector(-10.3346, 5.1089) * millimeter, vector(-10.3956, 5.0117) * millimeter, vector(-10.4546, 4.9143) * millimeter, vector(-10.5117, 4.8164) * millimeter, vector(-10.5666, 4.7180) * millimeter, vector(-10.6193, 4.6191) * millimeter, vector(-10.6695, 4.5195) * millimeter, vector(-10.7174, 4.4192) * millimeter, vector(-10.7628, 4.3181) * millimeter, vector(-10.8058, 4.2162) * millimeter, vector(-10.8462, 4.1134) * millimeter, vector(-10.8841, 4.0097) * millimeter, vector(-10.9196, 3.9049) * millimeter, vector(-10.9526, 3.7992) * millimeter, vector(-10.9833, 3.6923) * millimeter, vector(-11.0116, 3.5844) * millimeter, vector(-11.0378, 3.4755) * millimeter, vector(-11.0620, 3.3654) * millimeter, vector(-11.0842, 3.2543) * millimeter, vector(-11.1046, 3.1422) * millimeter, vector(-11.1234, 3.0291) * millimeter, vector(-11.1407, 2.9150) * millimeter, vector(-11.1567, 2.8000) * millimeter, vector(-11.1717, 2.6842) * millimeter, vector(-11.1858, 2.5676) * millimeter, vector(-11.1992, 2.4502) * millimeter, vector(-11.2121, 2.3322) * millimeter, vector(-11.2247, 2.2137) * millimeter, vector(-11.2371, 2.0946) * millimeter, vector(-11.2496, 1.9751) * millimeter, vector(-11.2623, 1.8552) * millimeter, vector(-11.2753, 1.7351) * millimeter, vector(-11.2887, 1.6149) * millimeter, vector(-11.3025, 1.4945) * millimeter, vector(-11.3169, 1.3741) * millimeter, vector(-11.3318, 1.2538) * millimeter, vector(-11.3471, 1.1336) * millimeter, vector(-11.3629, 1.0136) * millimeter, vector(-11.3790, 0.8939) * millimeter, vector(-11.3954, 0.7745) * millimeter, vector(-11.4119, 0.6555) * millimeter, vector(-11.4283, 0.5369) * millimeter, vector(-11.4444, 0.4188) * millimeter, vector(-11.4601, 0.3011) * millimeter, vector(-11.4752, 0.1840) * millimeter, vector(-11.4895, 0.0674) * millimeter, vector(-11.5026, -0.0487) * millimeter, vector(-11.5146, -0.1642) * millimeter, vector(-11.5251, -0.2791) * millimeter, vector(-11.5339, -0.3934) * millimeter, vector(-11.5409, -0.5072) * millimeter, vector(-11.5459, -0.6203) * millimeter, vector(-11.5488, -0.7330) * millimeter, vector(-11.5495, -0.8450) * millimeter, vector(-11.5477, -0.9566) * millimeter, vector(-11.5435, -1.0676) * millimeter, vector(-11.5368, -1.1782) * millimeter, vector(-11.5274, -1.2884) * millimeter, vector(-11.5154, -1.3982) * millimeter, vector(-11.5008, -1.5077) * millimeter, vector(-11.4835, -1.6169) * millimeter, vector(-11.4636, -1.7259) * millimeter, vector(-11.4411, -1.8348) * millimeter, vector(-11.4161, -1.9435) * millimeter, vector(-11.3887, -2.0522) * millimeter, vector(-11.3589, -2.1608) * millimeter, vector(-11.3269, -2.2695) * millimeter, vector(-11.2929, -2.3783) * millimeter, vector(-11.2569, -2.4872) * millimeter, vector(-11.2193, -2.5962) * millimeter, vector(-11.1800, -2.7055) * millimeter, vector(-11.1395, -2.8150) * millimeter, vector(-11.0977, -2.9248) * millimeter, vector(-11.0551, -3.0350) * millimeter, vector(-11.0116, -3.1454) * millimeter, vector(-10.9677, -3.2563) * millimeter, vector(-10.9234, -3.3675) * millimeter, vector(-10.8789, -3.4791) * millimeter, vector(-10.8345, -3.5911) * millimeter, vector(-10.7901, -3.7035) * millimeter, vector(-10.7461, -3.8162) * millimeter, vector(-10.7024, -3.9292) * millimeter, vector(-10.6592, -4.0425) * millimeter, vector(-10.6164, -4.1559) * millimeter, vector(-10.5742, -4.2695) * millimeter, vector(-10.5324, -4.3831) * millimeter, vector(-10.4911, -4.4966) * millimeter, vector(-10.4501, -4.6099) * millimeter, vector(-10.4093, -4.7229) * millimeter, vector(-10.3687, -4.8356) * millimeter, vector(-10.3281, -4.9477) * millimeter, vector(-10.2874, -5.0592) * millimeter, vector(-10.2463, -5.1699) * millimeter, vector(-10.2047, -5.2797) * millimeter, vector(-10.1625, -5.3886) * millimeter, vector(-10.1194, -5.4964) * millimeter, vector(-10.0752, -5.6031) * millimeter, vector(-10.0299, -5.7084) * millimeter, vector(-9.9833, -5.8124) * millimeter, vector(-9.9351, -5.9149) * millimeter, vector(-9.8854, -6.0160) * millimeter, vector(-9.8338, -6.1155) * millimeter, vector(-9.7805, -6.2135) * millimeter, vector(-9.7251, -6.3099) * millimeter, vector(-9.6677, -6.4047) * millimeter, vector(-9.6083, -6.4979) * millimeter, vector(-9.5466, -6.5896) * millimeter, vector(-9.4828, -6.6797) * millimeter, vector(-9.4167, -6.7684) * millimeter, vector(-9.3484, -6.8556) * millimeter, vector(-9.2779, -6.9416) * millimeter, vector(-9.2053, -7.0262) * millimeter, vector(-9.1305, -7.1097) * millimeter, vector(-9.0536, -7.1920) * millimeter, vector(-8.9748, -7.2734) * millimeter, vector(-8.8941, -7.3539) * millimeter, vector(-8.8117, -7.4336) * millimeter, vector(-8.7276, -7.5127) * millimeter, vector(-8.6421, -7.5912) * millimeter, vector(-8.5553, -7.6694) * millimeter, vector(-8.4673, -7.7472) * millimeter, vector(-8.3784, -7.8249) * millimeter, vector(-8.2886, -7.9025) * millimeter, vector(-8.1981, -7.9802) * millimeter, vector(-8.1072, -8.0581) * millimeter, vector(-8.0160, -8.1363) * millimeter, vector(-7.9246, -8.2148) * millimeter, vector(-7.8331, -8.2937) * millimeter, vector(-7.7417, -8.3730) * millimeter, vector(-7.6505, -8.4528) * millimeter, vector(-7.5596, -8.5330) * millimeter, vector(-7.4690, -8.6136) * millimeter, vector(-7.3788, -8.6945) * millimeter, vector(-7.2891, -8.7757) * millimeter, vector(-7.1997, -8.8570) * millimeter, vector(-7.1108, -8.9383) * millimeter, vector(-7.0222, -9.0194) * millimeter, vector(-6.9339, -9.1003) * millimeter, vector(-6.8458, -9.1807) * millimeter, vector(-6.7579, -9.2604) * millimeter, vector(-6.6701, -9.3394) * millimeter, vector(-6.5822, -9.4174) * millimeter, vector(-6.4942, -9.4941) * millimeter, vector(-6.4059, -9.5696) * millimeter, vector(-6.3173, -9.6435) * millimeter, vector(-6.2282, -9.7157) * millimeter, vector(-6.1386, -9.7861) * millimeter, vector(-6.0483, -9.8545) * millimeter, vector(-5.9573, -9.9209) * millimeter, vector(-5.8654, -9.9851) * millimeter, vector(-5.7726, -10.0470) * millimeter, vector(-5.6788, -10.1066) * millimeter, vector(-5.5840, -10.1639) * millimeter, vector(-5.4880, -10.2188) * millimeter, vector(-5.3908, -10.2713) * millimeter, vector(-5.2924, -10.3214) * millimeter, vector(-5.1927, -10.3693) * millimeter, vector(-5.0916, -10.4148) * millimeter, vector(-4.9893, -10.4581) * millimeter, vector(-4.8856, -10.4993) * millimeter, vector(-4.7806, -10.5384) * millimeter, vector(-4.6743, -10.5757) * millimeter, vector(-4.5667, -10.6111) * millimeter, vector(-4.4578, -10.6449) * millimeter, vector(-4.3478, -10.6772) * millimeter, vector(-4.2366, -10.7081) * millimeter, vector(-4.1244, -10.7379) * millimeter, vector(-4.0112, -10.7667) * millimeter, vector(-3.8971, -10.7948) * millimeter, vector(-3.7823, -10.8222) * millimeter, vector(-3.6667, -10.8492) * millimeter, vector(-3.5505, -10.8760) * millimeter, vector(-3.4338, -10.9027) * millimeter, vector(-3.3167, -10.9295) * millimeter, vector(-3.1992, -10.9566) * millimeter, vector(-3.0816, -10.9839) * millimeter, vector(-2.9638, -11.0117) * millimeter, vector(-2.8460, -11.0400) * millimeter, vector(-2.7282, -11.0687) * millimeter, vector(-2.6105, -11.0980) * millimeter, vector(-2.4931, -11.1277) * millimeter, vector(-2.3759, -11.1579) * millimeter, vector(-2.2590, -11.1883) * millimeter, vector(-2.1424, -11.2190) * millimeter, vector(-2.0263, -11.2497) * millimeter, vector(-1.9105, -11.2802) * millimeter, vector(-1.7952, -11.3105) * millimeter, vector(-1.6803, -11.3403) * millimeter, vector(-1.5658, -11.3694) * millimeter, vector(-1.4518, -11.3976) * millimeter, vector(-1.3382, -11.4246) * millimeter, vector(-1.2250, -11.4504) * millimeter, vector(-1.1122, -11.4747) * millimeter, vector(-0.9997, -11.4972) * millimeter, vector(-0.8876, -11.5179) * millimeter, vector(-0.7759, -11.5365) * millimeter, vector(-0.6645, -11.5530) * millimeter, vector(-0.5533, -11.5671) * millimeter, vector(-0.4423, -11.5788) * millimeter, vector(-0.3316, -11.5880) * millimeter, vector(-0.2210, -11.5947) * millimeter, vector(-0.1105, -11.5987) * millimeter, vector(-0.0000, -11.6000) * millimeter, vector(0.1105, -11.5987) * millimeter, vector(0.2210, -11.5947) * millimeter, vector(0.3316, -11.5880) * millimeter, vector(0.4423, -11.5788) * millimeter, vector(0.5533, -11.5671) * millimeter, vector(0.6645, -11.5530) * millimeter, vector(0.7759, -11.5365) * millimeter, vector(0.8876, -11.5179) * millimeter, vector(0.9997, -11.4972) * millimeter, vector(1.1122, -11.4747) * millimeter, vector(1.2250, -11.4504) * millimeter, vector(1.3382, -11.4246) * millimeter, vector(1.4518, -11.3976) * millimeter, vector(1.5658, -11.3694) * millimeter, vector(1.6803, -11.3403) * millimeter, vector(1.7952, -11.3105) * millimeter, vector(1.9105, -11.2802) * millimeter, vector(2.0263, -11.2497) * millimeter, vector(2.1424, -11.2190) * millimeter, vector(2.2590, -11.1883) * millimeter, vector(2.3759, -11.1579) * millimeter, vector(2.4931, -11.1277) * millimeter, vector(2.6105, -11.0980) * millimeter, vector(2.7282, -11.0687) * millimeter, vector(2.8460, -11.0400) * millimeter, vector(2.9638, -11.0117) * millimeter, vector(3.0816, -10.9839) * millimeter, vector(3.1992, -10.9566) * millimeter, vector(3.3167, -10.9295) * millimeter, vector(3.4338, -10.9027) * millimeter, vector(3.5505, -10.8760) * millimeter, vector(3.6667, -10.8492) * millimeter, vector(3.7823, -10.8222) * millimeter, vector(3.8971, -10.7948) * millimeter, vector(4.0112, -10.7667) * millimeter, vector(4.1244, -10.7379) * millimeter, vector(4.2366, -10.7081) * millimeter, vector(4.3478, -10.6772) * millimeter, vector(4.4578, -10.6449) * millimeter, vector(4.5667, -10.6111) * millimeter, vector(4.6743, -10.5757) * millimeter, vector(4.7806, -10.5384) * millimeter, vector(4.8856, -10.4993) * millimeter, vector(4.9893, -10.4581) * millimeter, vector(5.0916, -10.4148) * millimeter, vector(5.1927, -10.3693) * millimeter, vector(5.2924, -10.3214) * millimeter, vector(5.3908, -10.2713) * millimeter, vector(5.4880, -10.2188) * millimeter, vector(5.5840, -10.1639) * millimeter, vector(5.6788, -10.1066) * millimeter, vector(5.7726, -10.0470) * millimeter, vector(5.8654, -9.9851) * millimeter, vector(5.9573, -9.9209) * millimeter, vector(6.0483, -9.8545) * millimeter, vector(6.1386, -9.7861) * millimeter, vector(6.2282, -9.7157) * millimeter, vector(6.3173, -9.6435) * millimeter, vector(6.4059, -9.5696) * millimeter, vector(6.4942, -9.4941) * millimeter, vector(6.5822, -9.4174) * millimeter, vector(6.6701, -9.3394) * millimeter, vector(6.7579, -9.2604) * millimeter, vector(6.8458, -9.1807) * millimeter, vector(6.9339, -9.1003) * millimeter, vector(7.0222, -9.0194) * millimeter, vector(7.1108, -8.9383) * millimeter, vector(7.1997, -8.8570) * millimeter, vector(7.2891, -8.7757) * millimeter, vector(7.3788, -8.6945) * millimeter, vector(7.4690, -8.6136) * millimeter, vector(7.5596, -8.5330) * millimeter, vector(7.6505, -8.4528) * millimeter, vector(7.7417, -8.3730) * millimeter, vector(7.8331, -8.2937) * millimeter, vector(7.9246, -8.2148) * millimeter, vector(8.0160, -8.1363) * millimeter, vector(8.1072, -8.0581) * millimeter, vector(8.1981, -7.9802) * millimeter, vector(8.2886, -7.9025) * millimeter, vector(8.3784, -7.8249) * millimeter, vector(8.4673, -7.7472) * millimeter, vector(8.5553, -7.6694) * millimeter, vector(8.6421, -7.5912) * millimeter, vector(8.7276, -7.5127) * millimeter, vector(8.8117, -7.4336) * millimeter, vector(8.8941, -7.3539) * millimeter, vector(8.9748, -7.2734) * millimeter, vector(9.0536, -7.1920) * millimeter, vector(9.1305, -7.1097) * millimeter, vector(9.2053, -7.0262) * millimeter, vector(9.2779, -6.9416) * millimeter, vector(9.3484, -6.8556) * millimeter, vector(9.4167, -6.7684) * millimeter, vector(9.4828, -6.6797) * millimeter, vector(9.5466, -6.5896) * millimeter, vector(9.6083, -6.4979) * millimeter, vector(9.6677, -6.4047) * millimeter, vector(9.7251, -6.3099) * millimeter, vector(9.7805, -6.2135) * millimeter, vector(9.8338, -6.1155) * millimeter, vector(9.8854, -6.0160) * millimeter, vector(9.9351, -5.9149) * millimeter, vector(9.9833, -5.8124) * millimeter, vector(10.0299, -5.7084) * millimeter, vector(10.0752, -5.6031) * millimeter, vector(10.1194, -5.4964) * millimeter, vector(10.1625, -5.3886) * millimeter, vector(10.2047, -5.2797) * millimeter, vector(10.2463, -5.1699) * millimeter, vector(10.2874, -5.0592) * millimeter, vector(10.3281, -4.9477) * millimeter, vector(10.3687, -4.8356) * millimeter, vector(10.4093, -4.7229) * millimeter, vector(10.4501, -4.6099) * millimeter, vector(10.4911, -4.4966) * millimeter, vector(10.5324, -4.3831) * millimeter, vector(10.5742, -4.2695) * millimeter, vector(10.6164, -4.1559) * millimeter, vector(10.6592, -4.0425) * millimeter, vector(10.7024, -3.9292) * millimeter, vector(10.7461, -3.8162) * millimeter, vector(10.7901, -3.7035) * millimeter, vector(10.8345, -3.5911) * millimeter, vector(10.8789, -3.4791) * millimeter, vector(10.9234, -3.3675) * millimeter, vector(10.9677, -3.2563) * millimeter, vector(11.0116, -3.1454) * millimeter, vector(11.0551, -3.0350) * millimeter, vector(11.0977, -2.9248) * millimeter, vector(11.1395, -2.8150) * millimeter, vector(11.1800, -2.7055) * millimeter, vector(11.2193, -2.5962) * millimeter, vector(11.2569, -2.4872) * millimeter, vector(11.2929, -2.3783) * millimeter, vector(11.3269, -2.2695) * millimeter, vector(11.3589, -2.1608) * millimeter, vector(11.3887, -2.0522) * millimeter, vector(11.4161, -1.9435) * millimeter, vector(11.4411, -1.8348) * millimeter, vector(11.4636, -1.7259) * millimeter, vector(11.4835, -1.6169) * millimeter, vector(11.5008, -1.5077) * millimeter, vector(11.5154, -1.3982) * millimeter, vector(11.5274, -1.2884) * millimeter, vector(11.5368, -1.1782) * millimeter, vector(11.5435, -1.0676) * millimeter, vector(11.5477, -0.9566) * millimeter, vector(11.5495, -0.8450) * millimeter, vector(11.5488, -0.7330) * millimeter, vector(11.5459, -0.6203) * millimeter, vector(11.5409, -0.5072) * millimeter, vector(11.5339, -0.3934) * millimeter, vector(11.5251, -0.2791) * millimeter, vector(11.5146, -0.1642) * millimeter, vector(11.5026, -0.0487) * millimeter, vector(11.4895, 0.0674) * millimeter, vector(11.4752, 0.1840) * millimeter, vector(11.4601, 0.3011) * millimeter, vector(11.4444, 0.4188) * millimeter, vector(11.4283, 0.5369) * millimeter, vector(11.4119, 0.6555) * millimeter, vector(11.3954, 0.7745) * millimeter, vector(11.3790, 0.8939) * millimeter, vector(11.3629, 1.0136) * millimeter, vector(11.3471, 1.1336) * millimeter, vector(11.3318, 1.2538) * millimeter, vector(11.3169, 1.3741) * millimeter, vector(11.3025, 1.4945) * millimeter, vector(11.2887, 1.6149) * millimeter, vector(11.2753, 1.7351) * millimeter, vector(11.2623, 1.8552) * millimeter, vector(11.2496, 1.9751) * millimeter, vector(11.2371, 2.0946) * millimeter, vector(11.2247, 2.2137) * millimeter, vector(11.2121, 2.3322) * millimeter, vector(11.1992, 2.4502) * millimeter, vector(11.1858, 2.5676) * millimeter, vector(11.1717, 2.6842) * millimeter, vector(11.1567, 2.8000) * millimeter, vector(11.1407, 2.9150) * millimeter, vector(11.1234, 3.0291) * millimeter, vector(11.1046, 3.1422) * millimeter, vector(11.0842, 3.2543) * millimeter, vector(11.0620, 3.3654) * millimeter, vector(11.0378, 3.4755) * millimeter, vector(11.0116, 3.5844) * millimeter, vector(10.9833, 3.6923) * millimeter, vector(10.9526, 3.7992) * millimeter, vector(10.9196, 3.9049) * millimeter, vector(10.8841, 4.0097) * millimeter, vector(10.8462, 4.1134) * millimeter, vector(10.8058, 4.2162) * millimeter, vector(10.7628, 4.3181) * millimeter, vector(10.7174, 4.4192) * millimeter, vector(10.6695, 4.5195) * millimeter, vector(10.6193, 4.6191) * millimeter, vector(10.5666, 4.7180) * millimeter, vector(10.5117, 4.8164) * millimeter, vector(10.4546, 4.9143) * millimeter, vector(10.3956, 5.0117) * millimeter, vector(10.3346, 5.1089) * millimeter, vector(10.2719, 5.2057) * millimeter, vector(10.2077, 5.3025) * millimeter, vector(10.1421, 5.3991) * millimeter, vector(10.0753, 5.4957) * millimeter, vector(10.0075, 5.5924) * millimeter, vector(9.9389, 5.6893) * millimeter, vector(9.8697, 5.7864) * millimeter, vector(9.8001, 5.8838) * millimeter, vector(9.7302, 5.9815) * millimeter, vector(9.6602, 6.0796) * millimeter, vector(9.5903, 6.1781) * millimeter, vector(9.5206, 6.2770) * millimeter, vector(9.4511, 6.3763) * millimeter, vector(9.3820, 6.4759) * millimeter, vector(9.3134, 6.5759) * millimeter, vector(9.2452, 6.6760) * millimeter, vector(9.1774, 6.7763) * millimeter, vector(9.1101, 6.8766) * millimeter, vector(9.0432, 6.9768) * millimeter, vector(8.9766, 7.0768) * millimeter, vector(8.9102, 7.1765) * millimeter, vector(8.8440, 7.2756) * millimeter, vector(8.7777, 7.3741) * millimeter, vector(8.7113, 7.4718) * millimeter, vector(8.6447, 7.5685) * millimeter, vector(8.5776, 7.6641) * millimeter, vector(8.5099, 7.7584) * millimeter, vector(8.4416, 7.8514) * millimeter, vector(8.3724, 7.9428) * millimeter, vector(8.3022, 8.0326) * millimeter, vector(8.2309, 8.1207) * millimeter, vector(8.1584, 8.2069) * millimeter, vector(8.0845, 8.2912) * millimeter, vector(8.0093, 8.3736) * millimeter, vector(7.9325, 8.4539) * millimeter, vector(7.8541, 8.5322) * millimeter, vector(7.7740, 8.6085) * millimeter, vector(7.6922, 8.6827) * millimeter, vector(7.6087, 8.7550) * millimeter, vector(7.5233, 8.8253) * millimeter, vector(7.4361, 8.8936) * millimeter, vector(7.3471, 8.9602) * millimeter, vector(7.2563, 9.0250) * millimeter, vector(7.1637, 9.0881) * millimeter, vector(7.0694, 9.1497) * millimeter, vector(6.9734, 9.2099) * millimeter, vector(6.8758, 9.2687) * millimeter, vector(6.7766, 9.3264) * millimeter, vector(6.6761, 9.3831) * millimeter, vector(6.5743, 9.4388) * millimeter, vector(6.4713, 9.4939) * millimeter, vector(6.3672, 9.5484) * millimeter, vector(6.2623, 9.6026) * millimeter, vector(6.1565, 9.6565) * millimeter, vector(6.0501, 9.7103) * millimeter, vector(5.9432, 9.7642) * millimeter, vector(5.8359, 9.8182) * millimeter, vector(5.7284, 9.8726) * millimeter, vector(5.6207, 9.9273) * millimeter, vector(5.5130, 9.9825) * millimeter, vector(5.4053, 10.0381) * millimeter, vector(5.2978, 10.0942) * millimeter, vector(5.1906, 10.1508) * millimeter, vector(5.0837, 10.2078) * millimeter, vector(4.9771, 10.2651) * millimeter, vector(4.8709, 10.3226) * millimeter, vector(4.7651, 10.3802) * millimeter, vector(4.6596, 10.4378) * millimeter, vector(4.5545, 10.4952) * millimeter, vector(4.4498, 10.5522) * millimeter, vector(4.3454, 10.6086) * millimeter, vector(4.2412, 10.6643) * millimeter, vector(4.1372, 10.7189) * millimeter, vector(4.0334, 10.7724) * millimeter, vector(3.9296, 10.8245) * millimeter, vector(3.8259, 10.8751) * millimeter, vector(3.7221, 10.9239) * millimeter, vector(3.6183, 10.9708) * millimeter, vector(3.5142, 11.0156) * millimeter, vector(3.4100, 11.0583) * millimeter, vector(3.3054, 11.0986) * millimeter, vector(3.2005, 11.1365) * millimeter, vector(3.0951, 11.1720) * millimeter, vector(2.9893, 11.2049) * millimeter, vector(2.8830, 11.2352) * millimeter, vector(2.7761, 11.2629) * millimeter, vector(2.6685, 11.2881) * millimeter, vector(2.5602, 11.3106) * millimeter, vector(2.4513, 11.3307) * millimeter, vector(2.3415, 11.3482) * millimeter, vector(2.2310, 11.3634) * millimeter, vector(2.1197, 11.3763) * millimeter, vector(2.0075, 11.3870) * millimeter, vector(1.8946, 11.3956) * millimeter, vector(1.7808, 11.4024) * millimeter, vector(1.6662, 11.4074) * millimeter, vector(1.5509, 11.4108) * millimeter, vector(1.4348, 11.4129) * millimeter, vector(1.3180, 11.4138) * millimeter, vector(1.2005, 11.4137) * millimeter, vector(1.0824, 11.4129) * millimeter, vector(0.9638, 11.4115) * millimeter, vector(0.8445, 11.4097) * millimeter, vector(0.7248, 11.4077) * millimeter, vector(0.6047, 11.4057) * millimeter, vector(0.4842, 11.4038) * millimeter, vector(0.3634, 11.4022) * millimeter, vector(0.2424, 11.4010) * millimeter, vector(0.1212, 11.4003) * millimeter],
    [vector(0.0000, 11.4000) * millimeter, vector(-0.1230, 11.4005) * millimeter, vector(-0.2459, 11.4020) * millimeter, vector(-0.3686, 11.4044) * millimeter, vector(-0.4912, 11.4076) * millimeter, vector(-0.6135, 11.4116) * millimeter, vector(-0.7354, 11.4161) * millimeter, vector(-0.8571, 11.4211) * millimeter, vector(-0.9783, 11.4263) * millimeter, vector(-1.0990, 11.4316) * millimeter, vector(-1.2193, 11.4366) * millimeter, vector(-1.3390, 11.4413) * millimeter, vector(-1.4582, 11.4453) * millimeter, vector(-1.5769, 11.4486) * millimeter, vector(-1.6949, 11.4507) * millimeter, vector(-1.8123, 11.4517) * millimeter, vector(-1.9291, 11.4512) * millimeter, vector(-2.0453, 11.4490) * millimeter, vector(-2.1609, 11.4451) * millimeter, vector(-2.2759, 11.4393) * millimeter, vector(-2.3902, 11.4314) * millimeter, vector(-2.5040, 11.4214) * millimeter, vector(-2.6172, 11.4092) * millimeter, vector(-2.7299, 11.3946) * millimeter, vector(-2.8421, 11.3777) * millimeter, vector(-2.9539, 11.3584) * millimeter, vector(-3.0653, 11.3368) * millimeter, vector(-3.1764, 11.3127) * millimeter, vector(-3.2873, 11.2864) * millimeter, vector(-3.3980, 11.2578) * millimeter, vector(-3.5086, 11.2270) * millimeter, vector(-3.6191, 11.1941) * millimeter, vector(-3.7297, 11.1592) * millimeter, vector(-3.8404, 11.1224) * millimeter, vector(-3.9513, 11.0840) * millimeter, vector(-4.0624, 11.0440) * millimeter, vector(-4.1738, 11.0026) * millimeter, vector(-4.2856, 10.9600) * millimeter, vector(-4.3979, 10.9163) * millimeter, vector(-4.5106, 10.8719) * millimeter, vector(-4.6239, 10.8268) * millimeter, vector(-4.7378, 10.7812) * millimeter, vector(-4.8522, 10.7353) * millimeter, vector(-4.9673, 10.6893) * millimeter, vector(-5.0830, 10.6433) * millimeter, vector(-5.1993, 10.5974) * millimeter, vector(-5.3162, 10.5517) * millimeter, vector(-5.4337, 10.5063) * millimeter, vector(-5.5516, 10.4613) * millimeter, vector(-5.6700, 10.4166) * millimeter, vector(-5.7887, 10.3723) * millimeter, vector(-5.9077, 10.3283) * millimeter, vector(-6.0268, 10.2845) * millimeter, vector(-6.1459, 10.2409) * millimeter, vector(-6.2650, 10.1973) * millimeter, vector(-6.3838, 10.1536) * millimeter, vector(-6.5023, 10.1097) * millimeter, vector(-6.6203, 10.0653) * millimeter, vector(-6.7377, 10.0204) * millimeter, vector(-6.8543, 9.9747) * millimeter, vector(-6.9700, 9.9281) * millimeter, vector(-7.0847, 9.8804) * millimeter, vector(-7.1981, 9.8315) * millimeter, vector(-7.3103, 9.7811) * millimeter, vector(-7.4210, 9.7291) * millimeter, vector(-7.5302, 9.6754) * millimeter, vector(-7.6378, 9.6199) * millimeter, vector(-7.7437, 9.5624) * millimeter, vector(-7.8478, 9.5029) * millimeter, vector(-7.9501, 9.4412) * millimeter, vector(-8.0505, 9.3773) * millimeter, vector(-8.1491, 9.3111) * millimeter, vector(-8.2458, 9.2426) * millimeter, vector(-8.3406, 9.1717) * millimeter, vector(-8.4336, 9.0984) * millimeter, vector(-8.5249, 9.0228) * millimeter, vector(-8.6145, 8.9448) * millimeter, vector(-8.7024, 8.8644) * millimeter, vector(-8.7888, 8.7818) * millimeter, vector(-8.8737, 8.6969) * millimeter, vector(-8.9572, 8.6099) * millimeter, vector(-9.0395, 8.5208) * millimeter, vector(-9.1207, 8.4298) * millimeter, vector(-9.2008, 8.3369) * millimeter, vector(-9.2800, 8.2422) * millimeter, vector(-9.3584, 8.1461) * millimeter, vector(-9.4361, 8.0484) * millimeter, vector(-9.5134, 7.9495) * millimeter, vector(-9.5902, 7.8495) * millimeter, vector(-9.6667, 7.7485) * millimeter, vector(-9.7430, 7.6467) * millimeter, vector(-9.8192, 7.5441) * millimeter, vector(-9.8954, 7.4411) * millimeter, vector(-9.9715, 7.3375) * millimeter, vector(-10.0477, 7.2337) * millimeter, vector(-10.1240, 7.1297) * millimeter, vector(-10.2003, 7.0255) * millimeter, vector(-10.2767, 6.9213) * millimeter, vector(-10.3529, 6.8170) * millimeter, vector(-10.4290, 6.7128) * millimeter, vector(-10.5049, 6.6086) * millimeter, vector(-10.5804, 6.5044) * millimeter, vector(-10.6553, 6.4003) * millimeter, vector(-10.7296, 6.2961) * millimeter, vector(-10.8031, 6.1920) * millimeter, vector(-10.8754, 6.0877) * millimeter, vector(-10.9466, 5.9833) * millimeter, vector(-11.0163, 5.8787) * millimeter, vector(-11.0845, 5.7738) * millimeter, vector(-11.1508, 5.6686) * millimeter, vector(-11.2152, 5.5630) * millimeter, vector(-11.2774, 5.4569) * millimeter, vector(-11.3373, 5.3502) * millimeter, vector(-11.3947, 5.2430) * millimeter, vector(-11.4496, 5.1351) * millimeter, vector(-11.5017, 5.0265) * millimeter, vector(-11.5511, 4.9170) * millimeter, vector(-11.5977, 4.8068) * millimeter, vector(-11.6413, 4.6956) * millimeter, vector(-11.6820, 4.5835) * millimeter, vector(-11.7197, 4.4705) * millimeter, vector(-11.7545, 4.3564) * millimeter, vector(-11.7863, 4.2413) * millimeter, vector(-11.8152, 4.1251) * millimeter, vector(-11.8413, 4.0079) * millimeter, vector(-11.8647, 3.8896) * millimeter, vector(-11.8853, 3.7703) * millimeter, vector(-11.9034, 3.6499) * millimeter, vector(-11.9191, 3.5286) * millimeter, vector(-11.9325, 3.4063) * millimeter, vector(-11.9438, 3.2831) * millimeter, vector(-11.9532, 3.1591) * millimeter, vector(-11.9608, 3.0344) * millimeter, vector(-11.9669, 2.9090) * millimeter, vector(-11.9716, 2.7829) * millimeter, vector(-11.9751, 2.6563) * millimeter, vector(-11.9778, 2.5293) * millimeter, vector(-11.9796, 2.4019) * millimeter, vector(-11.9810, 2.2742) * millimeter, vector(-11.9819, 2.1462) * millimeter, vector(-11.9827, 2.0181) * millimeter, vector(-11.9833, 1.8899) * millimeter, vector(-11.9840, 1.7618) * millimeter, vector(-11.9849, 1.6337) * millimeter, vector(-11.9860, 1.5058) * millimeter, vector(-11.9873, 1.3781) * millimeter, vector(-11.9889, 1.2508) * millimeter, vector(-11.9907, 1.1238) * millimeter, vector(-11.9926, 0.9972) * millimeter, vector(-11.9947, 0.8712) * millimeter, vector(-11.9967, 0.7458) * millimeter, vector(-11.9986, 0.6209) * millimeter, vector(-12.0002, 0.4967) * millimeter, vector(-12.0013, 0.3732) * millimeter, vector(-12.0018, 0.2505) * millimeter, vector(-12.0014, 0.1285) * millimeter, vector(-12.0001, 0.0072) * millimeter, vector(-11.9975, -0.1132) * millimeter, vector(-11.9935, -0.2328) * millimeter, vector(-11.9880, -0.3516) * millimeter, vector(-11.9807, -0.4697) * millimeter, vector(-11.9716, -0.5869) * millimeter, vector(-11.9604, -0.7033) * millimeter, vector(-11.9470, -0.8190) * millimeter, vector(-11.9314, -0.9339) * millimeter, vector(-11.9135, -1.0482) * millimeter, vector(-11.8930, -1.1618) * millimeter, vector(-11.8701, -1.2748) * millimeter, vector(-11.8447, -1.3872) * millimeter, vector(-11.8168, -1.4990) * millimeter, vector(-11.7863, -1.6104) * millimeter, vector(-11.7533, -1.7214) * millimeter, vector(-11.7178, -1.8320) * millimeter, vector(-11.6800, -1.9422) * millimeter, vector(-11.6399, -2.0522) * millimeter, vector(-11.5976, -2.1619) * millimeter, vector(-11.5533, -2.2715) * millimeter, vector(-11.5071, -2.3809) * millimeter, vector(-11.4592, -2.4902) * millimeter, vector(-11.4097, -2.5995) * millimeter, vector(-11.3590, -2.7089) * millimeter, vector(-11.3071, -2.8183) * millimeter, vector(-11.2543, -2.9278) * millimeter, vector(-11.2009, -3.0375) * millimeter, vector(-11.1470, -3.1474) * millimeter, vector(-11.0929, -3.2576) * millimeter, vector(-11.0387, -3.3681) * millimeter, vector(-10.9847, -3.4790) * millimeter, vector(-10.9311, -3.5902) * millimeter, vector(-10.8779, -3.7017) * millimeter, vector(-10.8254, -3.8137) * millimeter, vector(-10.7737, -3.9259) * millimeter, vector(-10.7228, -4.0385) * millimeter, vector(-10.6728, -4.1513) * millimeter, vector(-10.6237, -4.2643) * millimeter, vector(-10.5755, -4.3774) * millimeter, vector(-10.5283, -4.4905) * millimeter, vector(-10.4818, -4.6036) * millimeter, vector(-10.4360, -4.7165) * millimeter, vector(-10.3908, -4.8291) * millimeter, vector(-10.3461, -4.9413) * millimeter, vector(-10.3016, -5.0530) * millimeter, vector(-10.2573, -5.1641) * millimeter, vector(-10.2129, -5.2745) * millimeter, vector(-10.1682, -5.3841) * millimeter, vector(-10.1231, -5.4927) * millimeter, vector(-10.0774, -5.6004) * millimeter, vector(-10.0308, -5.7070) * millimeter, vector(-9.9833, -5.8124) * millimeter, vector(-9.9346, -5.9166) * millimeter, vector(-9.8846, -6.0195) * millimeter, vector(-9.8333, -6.1211) * millimeter, vector(-9.7803, -6.2213) * millimeter, vector(-9.7257, -6.3202) * millimeter, vector(-9.6694, -6.4178) * millimeter, vector(-9.6112, -6.5140) * millimeter, vector(-9.5511, -6.6089) * millimeter, vector(-9.4891, -6.7026) * millimeter, vector(-9.4252, -6.7950) * millimeter, vector(-9.3593, -6.8863) * millimeter, vector(-9.2915, -6.9766) * millimeter, vector(-9.2217, -7.0658) * millimeter, vector(-9.1500, -7.1542) * millimeter, vector(-9.0766, -7.2418) * millimeter, vector(-9.0014, -7.3288) * millimeter, vector(-8.9246, -7.4151) * millimeter, vector(-8.8462, -7.5011) * millimeter, vector(-8.7665, -7.5868) * millimeter, vector(-8.6855, -7.6723) * millimeter, vector(-8.6033, -7.7578) * millimeter, vector(-8.5203, -7.8433) * millimeter, vector(-8.4364, -7.9291) * millimeter, vector(-8.3519, -8.0153) * millimeter, vector(-8.2669, -8.1019) * millimeter, vector(-8.1816, -8.1890) * millimeter, vector(-8.0961, -8.2768) * millimeter, vector(-8.0105, -8.3653) * millimeter, vector(-7.9251, -8.4546) * millimeter, vector(-7.8398, -8.5446) * millimeter, vector(-7.7548, -8.6353) * millimeter, vector(-7.6701, -8.7268) * millimeter, vector(-7.5858, -8.8189) * millimeter, vector(-7.5019, -8.9117) * millimeter, vector(-7.4184, -9.0049) * millimeter, vector(-7.3352, -9.0985) * millimeter, vector(-7.2524, -9.1924) * millimeter, vector(-7.1698, -9.2863) * millimeter, vector(-7.0874, -9.3802) * millimeter, vector(-7.0051, -9.4738) * millimeter, vector(-6.9227, -9.5670) * millimeter, vector(-6.8402, -9.6596) * millimeter, vector(-6.7574, -9.7513) * millimeter, vector(-6.6742, -9.8420) * millimeter, vector(-6.5904, -9.9316) * millimeter, vector(-6.5061, -10.0197) * millimeter, vector(-6.4210, -10.1063) * millimeter, vector(-6.3350, -10.1912) * millimeter, vector(-6.2481, -10.2742) * millimeter, vector(-6.1601, -10.3553) * millimeter, vector(-6.0710, -10.4342) * millimeter, vector(-5.9807, -10.5109) * millimeter, vector(-5.8890, -10.5854) * millimeter, vector(-5.7961, -10.6575) * millimeter, vector(-5.7017, -10.7273) * millimeter, vector(-5.6059, -10.7947) * millimeter, vector(-5.5085, -10.8597) * millimeter, vector(-5.4097, -10.9223) * millimeter, vector(-5.3093, -10.9827) * millimeter, vector(-5.2073, -11.0408) * millimeter, vector(-5.1037, -11.0968) * millimeter, vector(-4.9986, -11.1507) * millimeter, vector(-4.8919, -11.2027) * millimeter, vector(-4.7837, -11.2528) * millimeter, vector(-4.6740, -11.3012) * millimeter, vector(-4.5629, -11.3481) * millimeter, vector(-4.4505, -11.3936) * millimeter, vector(-4.3367, -11.4378) * millimeter, vector(-4.2218, -11.4810) * millimeter, vector(-4.1057, -11.5232) * millimeter, vector(-3.9885, -11.5647) * millimeter, vector(-3.8705, -11.6057) * millimeter, vector(-3.7516, -11.6462) * millimeter, vector(-3.6319, -11.6864) * millimeter, vector(-3.5116, -11.7265) * millimeter, vector(-3.3908, -11.7665) * millimeter, vector(-3.2695, -11.8065) * millimeter, vector(-3.1478, -11.8466) * millimeter, vector(-3.0258, -11.8869) * millimeter, vector(-2.9037, -11.9273) * millimeter, vector(-2.7814, -11.9678) * millimeter, vector(-2.6591, -12.0084) * millimeter, vector(-2.5367, -12.0489) * millimeter, vector(-2.4144, -12.0893) * millimeter, vector(-2.2922, -12.1295) * millimeter, vector(-2.1701, -12.1692) * millimeter, vector(-2.0481, -12.2084) * millimeter, vector(-1.9262, -12.2468) * millimeter, vector(-1.8046, -12.2842) * millimeter, vector(-1.6831, -12.3205) * millimeter, vector(-1.5617, -12.3554) * millimeter, vector(-1.4406, -12.3886) * millimeter, vector(-1.3197, -12.4201) * millimeter, vector(-1.1989, -12.4496) * millimeter, vector(-1.0783, -12.4769) * millimeter, vector(-0.9580, -12.5018) * millimeter, vector(-0.8378, -12.5242) * millimeter, vector(-0.7177, -12.5439) * millimeter, vector(-0.5978, -12.5608) * millimeter, vector(-0.4781, -12.5748) * millimeter, vector(-0.3585, -12.5858) * millimeter, vector(-0.2389, -12.5937) * millimeter, vector(-0.1194, -12.5984) * millimeter, vector(-0.0000, -12.6000) * millimeter, vector(0.1194, -12.5984) * millimeter, vector(0.2389, -12.5937) * millimeter, vector(0.3585, -12.5858) * millimeter, vector(0.4781, -12.5748) * millimeter, vector(0.5978, -12.5608) * millimeter, vector(0.7177, -12.5439) * millimeter, vector(0.8378, -12.5242) * millimeter, vector(0.9580, -12.5018) * millimeter, vector(1.0783, -12.4769) * millimeter, vector(1.1989, -12.4496) * millimeter, vector(1.3197, -12.4201) * millimeter, vector(1.4406, -12.3886) * millimeter, vector(1.5617, -12.3554) * millimeter, vector(1.6831, -12.3205) * millimeter, vector(1.8046, -12.2842) * millimeter, vector(1.9262, -12.2468) * millimeter, vector(2.0481, -12.2084) * millimeter, vector(2.1701, -12.1692) * millimeter, vector(2.2922, -12.1295) * millimeter, vector(2.4144, -12.0893) * millimeter, vector(2.5367, -12.0489) * millimeter, vector(2.6591, -12.0084) * millimeter, vector(2.7814, -11.9678) * millimeter, vector(2.9037, -11.9273) * millimeter, vector(3.0258, -11.8869) * millimeter, vector(3.1478, -11.8466) * millimeter, vector(3.2695, -11.8065) * millimeter, vector(3.3908, -11.7665) * millimeter, vector(3.5116, -11.7265) * millimeter, vector(3.6319, -11.6864) * millimeter, vector(3.7516, -11.6462) * millimeter, vector(3.8705, -11.6057) * millimeter, vector(3.9885, -11.5647) * millimeter, vector(4.1057, -11.5232) * millimeter, vector(4.2218, -11.4810) * millimeter, vector(4.3367, -11.4378) * millimeter, vector(4.4505, -11.3936) * millimeter, vector(4.5629, -11.3481) * millimeter, vector(4.6740, -11.3012) * millimeter, vector(4.7837, -11.2528) * millimeter, vector(4.8919, -11.2027) * millimeter, vector(4.9986, -11.1507) * millimeter, vector(5.1037, -11.0968) * millimeter, vector(5.2073, -11.0408) * millimeter, vector(5.3093, -10.9827) * millimeter, vector(5.4097, -10.9223) * millimeter, vector(5.5085, -10.8597) * millimeter, vector(5.6059, -10.7947) * millimeter, vector(5.7017, -10.7273) * millimeter, vector(5.7961, -10.6575) * millimeter, vector(5.8890, -10.5854) * millimeter, vector(5.9807, -10.5109) * millimeter, vector(6.0710, -10.4342) * millimeter, vector(6.1601, -10.3553) * millimeter, vector(6.2481, -10.2742) * millimeter, vector(6.3350, -10.1912) * millimeter, vector(6.4210, -10.1063) * millimeter, vector(6.5061, -10.0197) * millimeter, vector(6.5904, -9.9316) * millimeter, vector(6.6742, -9.8420) * millimeter, vector(6.7574, -9.7513) * millimeter, vector(6.8402, -9.6596) * millimeter, vector(6.9227, -9.5670) * millimeter, vector(7.0051, -9.4738) * millimeter, vector(7.0874, -9.3802) * millimeter, vector(7.1698, -9.2863) * millimeter, vector(7.2524, -9.1924) * millimeter, vector(7.3352, -9.0985) * millimeter, vector(7.4184, -9.0049) * millimeter, vector(7.5019, -8.9117) * millimeter, vector(7.5858, -8.8189) * millimeter, vector(7.6701, -8.7268) * millimeter, vector(7.7548, -8.6353) * millimeter, vector(7.8398, -8.5446) * millimeter, vector(7.9251, -8.4546) * millimeter, vector(8.0105, -8.3653) * millimeter, vector(8.0961, -8.2768) * millimeter, vector(8.1816, -8.1890) * millimeter, vector(8.2669, -8.1019) * millimeter, vector(8.3519, -8.0153) * millimeter, vector(8.4364, -7.9291) * millimeter, vector(8.5203, -7.8433) * millimeter, vector(8.6033, -7.7578) * millimeter, vector(8.6855, -7.6723) * millimeter, vector(8.7665, -7.5868) * millimeter, vector(8.8462, -7.5011) * millimeter, vector(8.9246, -7.4151) * millimeter, vector(9.0014, -7.3288) * millimeter, vector(9.0766, -7.2418) * millimeter, vector(9.1500, -7.1542) * millimeter, vector(9.2217, -7.0658) * millimeter, vector(9.2915, -6.9766) * millimeter, vector(9.3593, -6.8863) * millimeter, vector(9.4252, -6.7950) * millimeter, vector(9.4891, -6.7026) * millimeter, vector(9.5511, -6.6089) * millimeter, vector(9.6112, -6.5140) * millimeter, vector(9.6694, -6.4178) * millimeter, vector(9.7257, -6.3202) * millimeter, vector(9.7803, -6.2213) * millimeter, vector(9.8333, -6.1211) * millimeter, vector(9.8846, -6.0195) * millimeter, vector(9.9346, -5.9166) * millimeter, vector(9.9833, -5.8124) * millimeter, vector(10.0308, -5.7070) * millimeter, vector(10.0774, -5.6004) * millimeter, vector(10.1231, -5.4927) * millimeter, vector(10.1682, -5.3841) * millimeter, vector(10.2129, -5.2745) * millimeter, vector(10.2573, -5.1641) * millimeter, vector(10.3016, -5.0530) * millimeter, vector(10.3461, -4.9413) * millimeter, vector(10.3908, -4.8291) * millimeter, vector(10.4360, -4.7165) * millimeter, vector(10.4818, -4.6036) * millimeter, vector(10.5283, -4.4905) * millimeter, vector(10.5755, -4.3774) * millimeter, vector(10.6237, -4.2643) * millimeter, vector(10.6728, -4.1513) * millimeter, vector(10.7228, -4.0385) * millimeter, vector(10.7737, -3.9259) * millimeter, vector(10.8254, -3.8137) * millimeter, vector(10.8779, -3.7017) * millimeter, vector(10.9311, -3.5902) * millimeter, vector(10.9847, -3.4790) * millimeter, vector(11.0387, -3.3681) * millimeter, vector(11.0929, -3.2576) * millimeter, vector(11.1470, -3.1474) * millimeter, vector(11.2009, -3.0375) * millimeter, vector(11.2543, -2.9278) * millimeter, vector(11.3071, -2.8183) * millimeter, vector(11.3590, -2.7089) * millimeter, vector(11.4097, -2.5995) * millimeter, vector(11.4592, -2.4902) * millimeter, vector(11.5071, -2.3809) * millimeter, vector(11.5533, -2.2715) * millimeter, vector(11.5976, -2.1619) * millimeter, vector(11.6399, -2.0522) * millimeter, vector(11.6800, -1.9422) * millimeter, vector(11.7178, -1.8320) * millimeter, vector(11.7533, -1.7214) * millimeter, vector(11.7863, -1.6104) * millimeter, vector(11.8168, -1.4990) * millimeter, vector(11.8447, -1.3872) * millimeter, vector(11.8701, -1.2748) * millimeter, vector(11.8930, -1.1618) * millimeter, vector(11.9135, -1.0482) * millimeter, vector(11.9314, -0.9339) * millimeter, vector(11.9470, -0.8190) * millimeter, vector(11.9604, -0.7033) * millimeter, vector(11.9716, -0.5869) * millimeter, vector(11.9807, -0.4697) * millimeter, vector(11.9880, -0.3516) * millimeter, vector(11.9935, -0.2328) * millimeter, vector(11.9975, -0.1132) * millimeter, vector(12.0001, 0.0072) * millimeter, vector(12.0014, 0.1285) * millimeter, vector(12.0018, 0.2505) * millimeter, vector(12.0013, 0.3732) * millimeter, vector(12.0002, 0.4967) * millimeter, vector(11.9986, 0.6209) * millimeter, vector(11.9967, 0.7458) * millimeter, vector(11.9947, 0.8712) * millimeter, vector(11.9926, 0.9972) * millimeter, vector(11.9907, 1.1238) * millimeter, vector(11.9889, 1.2508) * millimeter, vector(11.9873, 1.3781) * millimeter, vector(11.9860, 1.5058) * millimeter, vector(11.9849, 1.6337) * millimeter, vector(11.9840, 1.7618) * millimeter, vector(11.9833, 1.8899) * millimeter, vector(11.9827, 2.0181) * millimeter, vector(11.9819, 2.1462) * millimeter, vector(11.9810, 2.2742) * millimeter, vector(11.9796, 2.4019) * millimeter, vector(11.9778, 2.5293) * millimeter, vector(11.9751, 2.6563) * millimeter, vector(11.9716, 2.7829) * millimeter, vector(11.9669, 2.9090) * millimeter, vector(11.9608, 3.0344) * millimeter, vector(11.9532, 3.1591) * millimeter, vector(11.9438, 3.2831) * millimeter, vector(11.9325, 3.4063) * millimeter, vector(11.9191, 3.5286) * millimeter, vector(11.9034, 3.6499) * millimeter, vector(11.8853, 3.7703) * millimeter, vector(11.8647, 3.8896) * millimeter, vector(11.8413, 4.0079) * millimeter, vector(11.8152, 4.1251) * millimeter, vector(11.7863, 4.2413) * millimeter, vector(11.7545, 4.3564) * millimeter, vector(11.7197, 4.4705) * millimeter, vector(11.6820, 4.5835) * millimeter, vector(11.6413, 4.6956) * millimeter, vector(11.5977, 4.8068) * millimeter, vector(11.5511, 4.9170) * millimeter, vector(11.5017, 5.0265) * millimeter, vector(11.4496, 5.1351) * millimeter, vector(11.3947, 5.2430) * millimeter, vector(11.3373, 5.3502) * millimeter, vector(11.2774, 5.4569) * millimeter, vector(11.2152, 5.5630) * millimeter, vector(11.1508, 5.6686) * millimeter, vector(11.0845, 5.7738) * millimeter, vector(11.0163, 5.8787) * millimeter, vector(10.9466, 5.9833) * millimeter, vector(10.8754, 6.0877) * millimeter, vector(10.8031, 6.1920) * millimeter, vector(10.7296, 6.2961) * millimeter, vector(10.6553, 6.4003) * millimeter, vector(10.5804, 6.5044) * millimeter, vector(10.5049, 6.6086) * millimeter, vector(10.4290, 6.7128) * millimeter, vector(10.3529, 6.8170) * millimeter, vector(10.2767, 6.9213) * millimeter, vector(10.2003, 7.0255) * millimeter, vector(10.1240, 7.1297) * millimeter, vector(10.0477, 7.2337) * millimeter, vector(9.9715, 7.3375) * millimeter, vector(9.8954, 7.4411) * millimeter, vector(9.8192, 7.5441) * millimeter, vector(9.7430, 7.6467) * millimeter, vector(9.6667, 7.7485) * millimeter, vector(9.5902, 7.8495) * millimeter, vector(9.5134, 7.9495) * millimeter, vector(9.4361, 8.0484) * millimeter, vector(9.3584, 8.1461) * millimeter, vector(9.2800, 8.2422) * millimeter, vector(9.2008, 8.3369) * millimeter, vector(9.1207, 8.4298) * millimeter, vector(9.0395, 8.5208) * millimeter, vector(8.9572, 8.6099) * millimeter, vector(8.8737, 8.6969) * millimeter, vector(8.7888, 8.7818) * millimeter, vector(8.7024, 8.8644) * millimeter, vector(8.6145, 8.9448) * millimeter, vector(8.5249, 9.0228) * millimeter, vector(8.4336, 9.0984) * millimeter, vector(8.3406, 9.1717) * millimeter, vector(8.2458, 9.2426) * millimeter, vector(8.1491, 9.3111) * millimeter, vector(8.0505, 9.3773) * millimeter, vector(7.9501, 9.4412) * millimeter, vector(7.8478, 9.5029) * millimeter, vector(7.7437, 9.5624) * millimeter, vector(7.6378, 9.6199) * millimeter, vector(7.5302, 9.6754) * millimeter, vector(7.4210, 9.7291) * millimeter, vector(7.3103, 9.7811) * millimeter, vector(7.1981, 9.8315) * millimeter, vector(7.0847, 9.8804) * millimeter, vector(6.9700, 9.9281) * millimeter, vector(6.8543, 9.9747) * millimeter, vector(6.7377, 10.0204) * millimeter, vector(6.6203, 10.0653) * millimeter, vector(6.5023, 10.1097) * millimeter, vector(6.3838, 10.1536) * millimeter, vector(6.2650, 10.1973) * millimeter, vector(6.1459, 10.2409) * millimeter, vector(6.0268, 10.2845) * millimeter, vector(5.9077, 10.3283) * millimeter, vector(5.7887, 10.3723) * millimeter, vector(5.6700, 10.4166) * millimeter, vector(5.5516, 10.4613) * millimeter, vector(5.4337, 10.5063) * millimeter, vector(5.3162, 10.5517) * millimeter, vector(5.1993, 10.5974) * millimeter, vector(5.0830, 10.6433) * millimeter, vector(4.9673, 10.6893) * millimeter, vector(4.8522, 10.7353) * millimeter, vector(4.7378, 10.7812) * millimeter, vector(4.6239, 10.8268) * millimeter, vector(4.5106, 10.8719) * millimeter, vector(4.3979, 10.9163) * millimeter, vector(4.2856, 10.9600) * millimeter, vector(4.1738, 11.0026) * millimeter, vector(4.0624, 11.0440) * millimeter, vector(3.9513, 11.0840) * millimeter, vector(3.8404, 11.1224) * millimeter, vector(3.7297, 11.1592) * millimeter, vector(3.6191, 11.1941) * millimeter, vector(3.5086, 11.2270) * millimeter, vector(3.3980, 11.2578) * millimeter, vector(3.2873, 11.2864) * millimeter, vector(3.1764, 11.3127) * millimeter, vector(3.0653, 11.3368) * millimeter, vector(2.9539, 11.3584) * millimeter, vector(2.8421, 11.3777) * millimeter, vector(2.7299, 11.3946) * millimeter, vector(2.6172, 11.4092) * millimeter, vector(2.5040, 11.4214) * millimeter, vector(2.3902, 11.4314) * millimeter, vector(2.2759, 11.4393) * millimeter, vector(2.1609, 11.4451) * millimeter, vector(2.0453, 11.4490) * millimeter, vector(1.9291, 11.4512) * millimeter, vector(1.8123, 11.4517) * millimeter, vector(1.6949, 11.4507) * millimeter, vector(1.5769, 11.4486) * millimeter, vector(1.4582, 11.4453) * millimeter, vector(1.3390, 11.4413) * millimeter, vector(1.2193, 11.4366) * millimeter, vector(1.0990, 11.4316) * millimeter, vector(0.9783, 11.4263) * millimeter, vector(0.8571, 11.4211) * millimeter, vector(0.7354, 11.4161) * millimeter, vector(0.6135, 11.4116) * millimeter, vector(0.4912, 11.4076) * millimeter, vector(0.3686, 11.4044) * millimeter, vector(0.2459, 11.4020) * millimeter, vector(0.1230, 11.4005) * millimeter]
];
export const WAVE_NAMES = ["wave cam blank", "wave cam 1x3.0", "wave cam 4x2.4", "wave cam 8x0.6", "wave cam 16x0.15", "wave cam 32x0.04", "wave cam 8x0.3", "wave cam 8x1.0", "wave cam 3x1.0", "wave cam 13x0.2", "wave cam 3x1.0+13x0.2"];
export const WAVE_TIP = 3;

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

    // ---- AHRS mode, drawn before the printed rig and moved RIG.waveOff along Y before the
    // drop cam exists, so no point query below can find them: the wave cam
    // on the horn's pins, and the follower tip lifted onto one of its valleys
    step("wave cam");
    var wavePts = [];
    for (var i = 0; i < size(WAVES); i += 1)
    {
        const W = P + ("wave" ~ i);
        const wCollar = cylIn(context, W + "collar", cs, vector(0 * mm, 0 * mm, -L.wellT), vector(0 * mm, 0 * mm, L.camGap + 0.5 * mm), L.collarR);
        polyPrism(context, W, "prof", toWorld(cs, vector(0 * mm, 0 * mm, L.camGap)), -Y, X, WAVES[i], L.waveT);
        unite(context, W + "u", [wCollar, qCreatedBy(W + "profExt", EntityType.BODY)]);
        const wPt = toWorld(cs, vector(L.collarR + 0.5 * mm, 0 * mm, L.camGap + 0.5 * mm));
        servoMountBuild(context, W + "horn", cs, HORN_OPT, partAt(context, W, wPt));
        const shift = L.waveOff * Y - i * L.wavePitch * X;
        opTransform(context, W + "move", { "bodies" : partAt(context, W, wPt), "transform" : transform(shift) });
        wavePts = append(wavePts, wPt + shift);
    }
    step("follower tip");
    const T = P + "tip";
    polyPrism(context, T, "outer", v(0 * mm, L.tipYo, 0 * mm), -Y, X,
              [vector(L.tipXd, L.tipZb), vector(L.tipXd, L.tipZt), vector(L.tipXoT, L.tipZt), vector(L.tipXoB, L.tipZb)],
              2 * L.tipYo);
    polyPrism(context, T, "pocket", v(0 * mm, L.tipYi, 0 * mm), -Y, X,
              [vector(L.padX1 + L.tipFit, L.tipZl), vector(L.tipXi0, L.tipZl),
               vector(L.tipXi0 - L.tipH - 1 * mm, L.tipZt + 1 * mm), vector(L.padX1 + L.tipFit, L.tipZt + 1 * mm)],
              2 * L.tipYi);
    const tipOuter = qCreatedBy(T + "outerExt", EntityType.BODY);
    cutFrom(context, T + "pocketCut", tipOuter, [qCreatedBy(T + "pocketExt", EntityType.BODY)]);
    // the nose in (X, Z): straight sides from inside the plate, a round end
    const zc = L.tipZb - L.noseH + L.noseR;
    var nose = [vector(L.Xc + L.noseR, L.tipZb + 0.2 * mm), vector(L.Xc + L.noseR, zc)];
    for (var k = 1; k < 12; k += 1)
        nose = append(nose, vector(L.Xc + L.noseR * cos(-k * 15 * degree), zc + L.noseR * sin(-k * 15 * degree)));
    nose = concatenateArrays([nose, [vector(L.Xc - L.noseR, zc), vector(L.Xc - L.noseR, L.tipZb + 0.2 * mm)]]);
    // from the -Y face (the bed, printed Y+) to just past the cam's +Y face
    polyPrism(context, T, "nose", v(0 * mm, L.noseYn, 0 * mm), -Y, X, nose, L.noseYn + L.tipYo);
    unite(context, T + "u", [tipOuter, qCreatedBy(T + "noseExt", EntityType.BODY)]);
    const tipPt = v(L.Xc - 2 * mm, 0 * mm, L.tipZb + L.tipT / 2);
    const tipShift = L.waveOff * Y - WAVE_TIP * L.wavePitch * X;
    opTransform(context, T + "move", { "bodies" : partAt(context, T, tipPt), "transform" : transform(tipShift) });
    const tipPtM = tipPt + tipShift;

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
    var parts = [["base", partAt(context, P, basePt), "Z+", Z, partC],
                   ["back shell", partAt(context, P, backPt), "Y-", -Y, partC],
                   ["cover", partAt(context, P, coverPt), "Y+", Y, partC],
                   ["cam", partAt(context, P, camPt), "Y+", Y, movesC],
                   ["front interface", partAt(context, P + "fi", fiPt), "X+", X, movesC],
                   ["rear interface", partAt(context, P + "ri", riPt), "X+", X, movesC],
                   ["follower tip", partAt(context, T, tipPtM), "Y+", Y, movesC]];
    for (var i = 0; i < size(WAVES); i += 1)
        parts = append(parts, [WAVE_NAMES[i], partAt(context, P + ("wave" ~ i), wavePts[i]), "Y+", Y, movesC]);
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
