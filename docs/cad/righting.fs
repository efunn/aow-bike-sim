FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_righting --push righting_features
 *
 * Every primitive of the simple parts, the upper case's additions and cuts,
 * the joints and the poses are computed in aow_sim.cad_righting and carried
 * here as RG, RG_PARTS, RG_POSES. Millimetres. MODULE FRAME: origin on the
 * wing rod at the mid-plane between the couplers, +Y forward, +X right, +Z
 * up; the XC330 at the rear, horn forward.
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
    "joint_plate" : 4.6 * millimeter,
    "ridge_height" : 1.2 * millimeter,
    "ridge_flat" : 4.6 * millimeter,
    "ridge_clearance" : 0.15 * millimeter,
    "bridge_layer" : 0.2 * millimeter
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

export const RG = { "yH" : -29.7, "zC" : 25.0913896732, "servoX" : 1, "ucAdd" : [{ "k" : "box", "lo" : [-14.05, -59.7, 9.5913896732], "hi" : [14.05, -37.4, 38.6413896732] }, { "k" : "prismX", "pts" : [[-49.7, 38.1413896732], [-37.4, 38.1413896732], [-37.4, 47.5730339363], [-40.7683557368, 47.5730339363], [-49.7, 38.6413896732]], "x0" : -5.25, "x1" : 5.25 }], "ucCut" : [{ "k" : "box", "lo" : [-12.45, -44.2, 9.4913896732], "hi" : [12.45, -36.4, 37.0413896732] }, { "k" : "box", "lo" : [-10.05, -55.7, 9.4913896732], "hi" : [10.05, -43.2, 34.6413896732] }, { "k" : "prismX", "pts" : [[-55.7, 10.5913896732], [-44.2, 10.5913896732], [-42.107171153, 16.3413896732], [-44.2, 22.0913896732], [-55.7, 22.0913896732]], "x0" : -15.05, "x1" : 15.05 }, { "k" : "box", "lo" : [6, -60.7, 10.5913896732], "hi" : [15.05, -55.6, 22.0913896732] }, { "k" : "box", "lo" : [-15.05, -60.7, 10.5913896732], "hi" : [-6, -55.6, 22.0913896732] }], "ucShellPt" : [-13.55, -59.2, 4.4163896732], "lowerNearPins" : 1, "joints" : [{ "tag" : "jF", "ridge" : true, "through" : false, "slotLen" : 6, "head" : [0, 18.5, 53.1755186068], "zIn" : [0, 0, -1], "slot" : [0, 1, 0], "pocket" : 3.4, "csk" : "bridge", "nut" : "bulkheadF", "ridgeOnNut" : true, "ridgeDir" : [1, 0, 0], "ridgeHalf" : 11.35, "cskUp" : [0, 0, 1], "nutUp" : [0, -1, 0] }, { "tag" : "jR", "ridge" : true, "through" : false, "slotLen" : 6, "head" : [0, -18.5, 53.1755186068], "zIn" : [0, 0, -1], "slot" : [0, 1, 0], "pocket" : 3.4, "csk" : "bridge", "nut" : "lowercase", "ridgeOnNut" : true, "ridgeDir" : [1, 0, 0], "ridgeHalf" : 11.35, "cskUp" : [0, 0, 1], "nutUp" : [0, -1, 0] }, { "tag" : "jUL", "ridge" : false, "through" : false, "slotLen" : 5.9816442632, "head" : [0, -42, 42.5913896732], "zIn" : [0, 1, 0], "slot" : [0, 0, 1], "pocket" : 8.7, "csk" : "uppercase", "nut" : "lowercase", "ridgeOnNut" : false, "ridgeDir" : [1, 0, 0], "ridgeHalf" : 0, "cskUp" : [0, 1, 0], "nutUp" : [0, -1, 0] }, { "tag" : "jC0", "ridge" : true, "through" : true, "slotLen" : 0, "head" : [0, 8, 61.1755186068], "zIn" : [0, 0, -1], "slot" : [1, 0, 0], "pocket" : 0, "csk" : "chassis", "nut" : "bridge", "ridgeOnNut" : true, "ridgeDir" : [0, 1, 0], "ridgeHalf" : 6, "cskUp" : [0, 0, 1], "nutUp" : [0, 0, 1] }, { "tag" : "jC1", "ridge" : true, "through" : true, "slotLen" : 0, "head" : [0, -8, 61.1755186068], "zIn" : [0, 0, -1], "slot" : [1, 0, 0], "pocket" : 0, "csk" : "chassis", "nut" : "bridge", "ridgeOnNut" : true, "ridgeDir" : [0, 1, 0], "ridgeHalf" : 6, "cskUp" : [0, 0, 1], "nutUp" : [0, 0, 1] }, { "tag" : "jWR", "ridge" : false, "through" : false, "slotLen" : 11, "head" : [37.7223675386, -17.5, 26.1802141396], "zIn" : [0, 1, 0], "slot" : [0.253983824, 0, 0.9672084662], "pocket" : 1.4, "csk" : "wingR", "nut" : "rockerR", "ridgeOnNut" : false, "ridgeDir" : [0.253983824, 0, 0.9672084662], "ridgeHalf" : 0, "cskUp" : [-0.9672084662, 0, 0.253983824], "nutUp" : [0, 1, 0] }, { "tag" : "jWL", "ridge" : false, "through" : false, "slotLen" : 11, "head" : [-37.7223675386, 17.5, 26.1802141396], "zIn" : [0, -1, 0], "slot" : [-0.253983824, 0, 0.9672084662], "pocket" : 1.4, "csk" : "wingL", "nut" : "rockerL", "ridgeOnNut" : false, "ridgeDir" : [-0.253983824, 0, 0.9672084662], "ridgeHalf" : 0, "cskUp" : [0.9672084662, 0, 0.253983824], "nutUp" : [0, -1, 0] }, { "tag" : "jKR", "ridge" : false, "through" : false, "slotLen" : 11, "head" : [37.7223675386, 10.9, 26.1802141396], "zIn" : [0, -1, 0], "slot" : [0.253983824, 0, 0.9672084662], "pocket" : 1.4, "csk" : "wingR", "nut" : "knuckleR", "ridgeOnNut" : false, "ridgeDir" : [0.253983824, 0, 0.9672084662], "ridgeHalf" : 0, "cskUp" : [-0.9672084662, 0, 0.253983824], "nutUp" : [0, -1, 0] }, { "tag" : "jKL", "ridge" : false, "through" : false, "slotLen" : 11, "head" : [-37.7223675386, -10.9, 26.1802141396], "zIn" : [0, 1, 0], "slot" : [-0.253983824, 0, 0.9672084662], "pocket" : 1.4, "csk" : "wingL", "nut" : "knuckleL", "ridgeOnNut" : false, "ridgeDir" : [-0.253983824, 0, 0.9672084662], "ridgeHalf" : 0, "cskUp" : [0.9672084662, 0, 0.253983824], "nutUp" : [0, 1, 0] }] };

export const RG_PARTS = [{ "slug" : "crankR", "name" : "half crank R", "group" : "crank", "up" : [0, -1, 0], "add" : [{ "k" : "slot", "pts" : [[-8.4239440376, 26.2259234327], [-6.4418395423, 40.9431032465], [6.4418395758, 40.9431032329], [8.42394404, 26.2259234149]], "mids" : [[0.0000000227, 46.5755186068], [-0.000000009, 16.5913896732]], "a" : [0, 25.0913896732], "b" : [0.0000000158, 40.0755186068], "ra" : 8.5, "rb" : 6.5, "y0" : -12.9, "y1" : -6.9 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -13.3, "y1" : -12.9, "r" : 8.2 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -24, "y1" : -12.9, "r" : 7 }, { "k" : "prism", "pts" : [[3.0405591591, 26.4348925574], [5.6542915954, 29.0486249937], [3.9572353206, 30.7456812685], [1.3435028843, 28.1319488323]], "y0" : -27.3, "y1" : -24 }, { "k" : "prism", "pts" : [[-1.3435028843, 28.1319488323], [-3.9572353206, 30.7456812685], [-5.6542915954, 29.0486249937], [-3.0405591591, 26.4348925574]], "y0" : -27.3, "y1" : -24 }, { "k" : "prism", "pts" : [[-3.0405591591, 23.7478867889], [-5.6542915954, 21.1341543526], [-3.9572353206, 19.4370980778], [-1.3435028843, 22.050830514]], "y0" : -27.3, "y1" : -24 }, { "k" : "prism", "pts" : [[1.3435028843, 22.050830514], [3.9572353206, 19.4370980778], [5.6542915954, 21.1341543526], [3.0405591591, 23.7478867889]], "y0" : -27.3, "y1" : -24 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -13.9, "y1" : -5.9, "r" : 3 }], "anchor" : [0.0000000079, -9.9, 32.58345414], "note" : "Print the web face down. Crankpin hole: ream to press. The lugs drive it from the horn hub.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "couplerR", "name" : "coupler R", "group" : "cR", "up" : [0, -1, 0], "add" : [{ "k" : "slot", "pts" : [[4.8045911379, 44.4534066992], [30.4703828311, 16.2860272904], [20.861200587, 7.5302511055], [-4.8045911062, 35.6976305143]], "mids" : [[30.0436798015, 7.1035480759], [-4.3778880766, 44.8801097288]], "a" : [0.0000000158, 40.0755186068], "b" : [25.665791709, 11.908139198], "ra" : 6.5, "rb" : 6.5, "y0" : -6.3, "y1" : -0.3 }, { "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -6.7, "y1" : -6.3, "r" : 5.5 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -6.7, "y1" : -6.3, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -7.3, "y1" : 0.7, "r" : 3 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -7.3, "y1" : 0.7, "r" : 3 }], "anchor" : [12.8328958624, -3.3, 25.9918289024], "note" : "38.11 mm centres. Both holes: ream to run. Print the bossed face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "rockerR", "name" : "rocker R", "group" : "wR", "up" : [0, 1, 0], "add" : [{ "k" : "slot", "pts" : [[-2.4897011732, 4.9042214538], [22.7234175952, 17.7040372798], [28.1913762917, 5.9188635727], [2.1370331084, -5.067848606]], "mids" : [[31.5620624122, 14.6438277393], [-4.9891521334, -2.3148133811]], "a" : [0, 0], "b" : [25.665791709, 11.908139198], "ra" : 5.5, "rb" : 6.5, "y0" : -12.9, "y1" : -6.9 }, { "k" : "slot", "pts" : [[21.1779496091, 16.6101891005], [34.615399931, 29.4354794568], [41.4510695977, 23.6609323065], [31.0516946833, 8.2691765502]], "mids" : [[40.6263281323, 29.6178023742], [21.4711819626, 6.9427339703]], "a" : [25.665791709, 11.908139198], "b" : [37.7223675386, 26.1802141396], "ra" : 6.5, "rb" : 4.5, "y0" : -12.9, "y1" : -6.9 }, { "k" : "prism", "pts" : [[40.0185716299, 15.2382103574], [45.0982481096, 34.582379682], [35.4261634473, 37.1222179219], [30.3464869676, 17.7780485972]], "y0" : -12.9, "y1" : -6.9 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : -6.9, "y1" : -6.5, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -13.9, "y1" : -5.9, "r" : 3 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -13.9, "y1" : -5.9, "r" : 3 }], "anchor" : [12.8328958545, -9.9, 5.954069599], "note" : "One plate in the web plane: hub, arm, ear and the wing's boss. The 6-32's nut in the boss, slid in from its top end; the screw from the wing's tab beyond. Its inner face's ring at the rod bears on the other wing's knuckle. Rod: ream to run; pin: to press. Print the inner face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "knuckleR", "name" : "knuckle R", "group" : "wR", "up" : [0, -1, 0], "add" : [{ "k" : "slot", "pts" : [[-2.4897011732, 4.9042214538], [22.7234175952, 17.7040372798], [28.1913762917, 5.9188635727], [2.1370331084, -5.067848606]], "mids" : [[31.5620624122, 14.6438277393], [-4.9891521334, -2.3148133811]], "a" : [0, 0], "b" : [25.665791709, 11.908139198], "ra" : 5.5, "rb" : 6.5, "y0" : 0.3, "y1" : 6.3 }, { "k" : "slot", "pts" : [[21.1779496091, 16.6101891005], [34.615399931, 29.4354794568], [41.4510695977, 23.6609323065], [31.0516946833, 8.2691765502]], "mids" : [[40.6263281323, 29.6178023742], [21.4711819626, 6.9427339703]], "a" : [25.665791709, 11.908139198], "b" : [37.7223675386, 26.1802141396], "ra" : 6.5, "rb" : 4.5, "y0" : 0.3, "y1" : 6.3 }, { "k" : "prism", "pts" : [[40.0185716299, 15.2382103574], [45.0982481096, 34.582379682], [35.4261634473, 37.1222179219], [30.3464869676, 17.7780485972]], "y0" : 0.3, "y1" : 6.3 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -0.1, "y1" : 0.3, "r" : 5.5 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : -0.1, "y1" : 0.3, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -0.7, "y1" : 7.3, "r" : 3 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -0.7, "y1" : 7.3, "r" : 3 }], "anchor" : [12.8328958545, 3.3, 5.954069599], "note" : "The wing's second support, in coupler L's layer: the rocker's outline, its pin through rocker, coupler and knuckle. The 6-32's nut in the boss, slid in from its top end; the screw from the wing's tab beyond. Its inner face's ring at the pin bears on its own coupler across the mid-plane. Rod: ream to run; pin: to press. Print the inner face up. Knuckle R only: a second ring, at the rod, on knuckle L.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "wingR", "name" : "wing R", "group" : "wR", "up" : [-0.9672084662, 0, 0.253983824], "add" : [{ "k" : "prism", "pts" : [[43.3307110172, 8.16504044], [51.4581933847, 39.1157113595], [46.6221510535, 40.3856304794], [38.494668686, 9.43495956]], "y0" : -22.65, "y1" : 16.05 }, { "k" : "poly", "origin" : [40.9126898516, 0, 8.8], "normal" : [0.253983824, 0, 0.9672084662], "xdir" : [0, 1, 0], "pts" : [[-22.15, -2.5], [-22.65, -2.5], [-24.9, -0.25], [-24.9, 0.25], [-22.65, 2.5], [-22.15, 2.5]], "depth" : 32 }, { "k" : "poly", "origin" : [40.9126898516, 0, 8.8], "normal" : [0.253983824, 0, 0.9672084662], "xdir" : [0, 1, 0], "pts" : [[15.55, -2.5], [16.05, -2.5], [18.3, -0.25], [18.3, 0.25], [16.05, 2.5], [15.55, 2.5]], "depth" : 32 }, { "k" : "prism", "pts" : [[40.0185716299, 15.2382103574], [45.0982481096, 34.582379682], [35.4261634473, 37.1222179219], [30.3464869676, 17.7780485972]], "y0" : -18.9, "y1" : -12.9 }, { "k" : "prism", "pts" : [[40.0185716299, 15.2382103574], [45.0982481096, 34.582379682], [35.4261634473, 37.1222179219], [30.3464869676, 17.7780485972]], "y0" : 6.3, "y1" : 12.3 }], "cut" : [], "anchor" : [44.9764310354, -3.3, 24.2753354597], "note" : "The STUB: print the outer face down. A 6-32 along Y from each tab into the rocker / knuckle's nut: one from each side. Its two Y ends are tongues (both faces at 45 deg) the blade's legs slide down over.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "bladeR", "name" : "wing R blade", "group" : "wR", "up" : undefined, "add" : [{ "k" : "prism", "pts" : [[51.4581933847, 39.1157113595], [68.7290934156, 104.8858870634], [63.8930510845, 106.1558061833], [46.6221510535, 40.3856304794]], "y0" : -41, "y1" : 41 }, { "k" : "prism", "pts" : [[43.3307110172, 8.16504044], [51.5851852967, 39.5993155926], [46.7491429655, 40.8692347126], [38.494668686, 9.43495956]], "y0" : -27.4, "y1" : -22.65 }, { "k" : "prism", "pts" : [[43.3307110172, 8.16504044], [51.5851852967, 39.5993155926], [46.7491429655, 40.8692347126], [38.494668686, 9.43495956]], "y0" : 16.05, "y1" : 20.8 }], "cut" : [{ "k" : "poly", "origin" : [40.6587060276, 0, 7.8327915338], "normal" : [0.253983824, 0, 0.9672084662], "xdir" : [0, 1, 0], "pts" : [[-25.1, -0.3328427125], [-25.1, 0.3328427125], [-21.9328427125, 3.5], [-18.15, 3.5], [-18.15, -3.5], [-21.9328427125, -3.5]], "depth" : 33 }, { "k" : "poly", "origin" : [40.6587060276, 0, 7.8327915338], "normal" : [0.253983824, 0, 0.9672084662], "xdir" : [0, 1, 0], "pts" : [[18.5, -0.3328427125], [18.5, 0.3328427125], [15.3328427125, 3.5], [11.55, 3.5], [11.55, -3.5], [15.3328427125, -3.5]], "depth" : 33 }], "anchor" : [57.6756222346, 0, 72.6357587714], "note" : "DUMMY: the wing proper, an inverted U over the stub: its legs hold the stub's chamfered ends, its body sits on the stub's top. The panel's own thickness. Retained up the panel: open.", "custom" : false, "mock" : false, "color" : [0.35, 0.55, 0.85], "kind" : "dummy" }, { "slug" : "rpinR", "name" : "rocker pin R", "group" : "wR", "up" : undefined, "add" : [{ "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -12.9, "y1" : 6.3, "r" : 3 }], "cut" : [], "anchor" : [25.665791709, -3.3, 11.908139198], "note" : "Rod stock: pressed in the rocker and the knuckle, runs in the coupler between them.", "custom" : false, "mock" : false, "color" : [0.3, 0.3, 0.32], "kind" : "steel" }, { "slug" : "crankL", "name" : "half crank L", "group" : "crank", "up" : [0, 1, 0], "add" : [{ "k" : "slot", "pts" : [[8.4239440376, 26.2259234327], [6.4418395423, 40.9431032465], [-6.4418395758, 40.9431032329], [-8.42394404, 26.2259234149]], "mids" : [[-0.0000000227, 46.5755186068], [0.000000009, 16.5913896732]], "a" : [0, 25.0913896732], "b" : [-0.0000000158, 40.0755186068], "ra" : 8.5, "rb" : 6.5, "y0" : 6.9, "y1" : 12.9 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 12.9, "y1" : 13.3, "r" : 8.2 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 12.9, "y1" : 24, "r" : 7 }], "cut" : [{ "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : 5.9, "y1" : 13.9, "r" : 3 }], "anchor" : [-0.0000000079, 9.9, 32.58345414], "note" : "Print the web face down. Crankpin hole: ream to press. No lugs: it only supports.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "couplerL", "name" : "coupler L", "group" : "cL", "up" : [0, 1, 0], "add" : [{ "k" : "slot", "pts" : [[-4.8045911379, 44.4534066992], [-30.4703828311, 16.2860272904], [-20.861200587, 7.5302511055], [4.8045911062, 35.6976305143]], "mids" : [[-30.0436798015, 7.1035480759], [4.3778880766, 44.8801097288]], "a" : [-0.0000000158, 40.0755186068], "b" : [-25.665791709, 11.908139198], "ra" : 6.5, "rb" : 6.5, "y0" : 0.3, "y1" : 6.3 }, { "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : 6.3, "y1" : 6.7, "r" : 5.5 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : 6.3, "y1" : 6.7, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : -0.7, "y1" : 7.3, "r" : 3 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -0.7, "y1" : 7.3, "r" : 3 }], "anchor" : [-12.8328958624, 3.3, 25.9918289024], "note" : "38.11 mm centres. Both holes: ream to run. Print the bossed face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "rockerL", "name" : "rocker L", "group" : "wL", "up" : [0, -1, 0], "add" : [{ "k" : "slot", "pts" : [[2.4897011732, 4.9042214538], [-22.7234175952, 17.7040372798], [-28.1913762917, 5.9188635727], [-2.1370331084, -5.067848606]], "mids" : [[-31.5620624122, 14.6438277393], [4.9891521334, -2.3148133811]], "a" : [0, 0], "b" : [-25.665791709, 11.908139198], "ra" : 5.5, "rb" : 6.5, "y0" : 6.9, "y1" : 12.9 }, { "k" : "slot", "pts" : [[-21.1779496091, 16.6101891005], [-34.615399931, 29.4354794568], [-41.4510695977, 23.6609323065], [-31.0516946833, 8.2691765502]], "mids" : [[-40.6263281323, 29.6178023742], [-21.4711819626, 6.9427339703]], "a" : [-25.665791709, 11.908139198], "b" : [-37.7223675386, 26.1802141396], "ra" : 6.5, "rb" : 4.5, "y0" : 6.9, "y1" : 12.9 }, { "k" : "prism", "pts" : [[-30.3464869676, 17.7780485972], [-35.4261634473, 37.1222179219], [-45.0982481096, 34.582379682], [-40.0185716299, 15.2382103574]], "y0" : 6.9, "y1" : 12.9 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : 6.5, "y1" : 6.9, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 5.9, "y1" : 13.9, "r" : 3 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : 5.9, "y1" : 13.9, "r" : 3 }], "anchor" : [-12.8328958545, 9.9, 5.954069599], "note" : "One plate in the web plane: hub, arm, ear and the wing's boss. The 6-32's nut in the boss, slid in from its top end; the screw from the wing's tab beyond. Its inner face's ring at the rod bears on the other wing's knuckle. Rod: ream to run; pin: to press. Print the inner face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "knuckleL", "name" : "knuckle L", "group" : "wL", "up" : [0, 1, 0], "add" : [{ "k" : "slot", "pts" : [[2.4897011732, 4.9042214538], [-22.7234175952, 17.7040372798], [-28.1913762917, 5.9188635727], [-2.1370331084, -5.067848606]], "mids" : [[-31.5620624122, 14.6438277393], [4.9891521334, -2.3148133811]], "a" : [0, 0], "b" : [-25.665791709, 11.908139198], "ra" : 5.5, "rb" : 6.5, "y0" : -6.3, "y1" : -0.3 }, { "k" : "slot", "pts" : [[-21.1779496091, 16.6101891005], [-34.615399931, 29.4354794568], [-41.4510695977, 23.6609323065], [-31.0516946833, 8.2691765502]], "mids" : [[-40.6263281323, 29.6178023742], [-21.4711819626, 6.9427339703]], "a" : [-25.665791709, 11.908139198], "b" : [-37.7223675386, 26.1802141396], "ra" : 6.5, "rb" : 4.5, "y0" : -6.3, "y1" : -0.3 }, { "k" : "prism", "pts" : [[-30.3464869676, 17.7780485972], [-35.4261634473, 37.1222179219], [-45.0982481096, 34.582379682], [-40.0185716299, 15.2382103574]], "y0" : -6.3, "y1" : -0.3 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -0.3, "y1" : 0.1, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -7.3, "y1" : 0.7, "r" : 3 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -7.3, "y1" : 0.7, "r" : 3 }], "anchor" : [-12.8328958545, -3.3, 5.954069599], "note" : "The wing's second support, in coupler L's layer: the rocker's outline, its pin through rocker, coupler and knuckle. The 6-32's nut in the boss, slid in from its top end; the screw from the wing's tab beyond. Its inner face's ring at the pin bears on its own coupler across the mid-plane. Rod: ream to run; pin: to press. Print the inner face up.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "wingL", "name" : "wing L", "group" : "wL", "up" : [0.9672084662, 0, 0.253983824], "add" : [{ "k" : "prism", "pts" : [[-38.494668686, 9.43495956], [-46.6221510535, 40.3856304794], [-51.4581933847, 39.1157113595], [-43.3307110172, 8.16504044]], "y0" : -16.05, "y1" : 22.65 }, { "k" : "poly", "origin" : [-40.9126898516, 0, 8.8], "normal" : [-0.253983824, 0, 0.9672084662], "xdir" : [0, -1, 0], "pts" : [[-22.15, -2.5], [-22.65, -2.5], [-24.9, -0.25], [-24.9, 0.25], [-22.65, 2.5], [-22.15, 2.5]], "depth" : 32 }, { "k" : "poly", "origin" : [-40.9126898516, 0, 8.8], "normal" : [-0.253983824, 0, 0.9672084662], "xdir" : [0, -1, 0], "pts" : [[15.55, -2.5], [16.05, -2.5], [18.3, -0.25], [18.3, 0.25], [16.05, 2.5], [15.55, 2.5]], "depth" : 32 }, { "k" : "prism", "pts" : [[-30.3464869676, 17.7780485972], [-35.4261634473, 37.1222179219], [-45.0982481096, 34.582379682], [-40.0185716299, 15.2382103574]], "y0" : 12.9, "y1" : 18.9 }, { "k" : "prism", "pts" : [[-30.3464869676, 17.7780485972], [-35.4261634473, 37.1222179219], [-45.0982481096, 34.582379682], [-40.0185716299, 15.2382103574]], "y0" : -12.3, "y1" : -6.3 }], "cut" : [], "anchor" : [-44.9764310354, 3.3, 24.2753354597], "note" : "The STUB: print the outer face down. A 6-32 along Y from each tab into the rocker / knuckle's nut: one from each side. Its two Y ends are tongues (both faces at 45 deg) the blade's legs slide down over.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "bladeL", "name" : "wing L blade", "group" : "wL", "up" : undefined, "add" : [{ "k" : "prism", "pts" : [[-46.6221510535, 40.3856304794], [-63.8930510845, 106.1558061833], [-68.7290934156, 104.8858870634], [-51.4581933847, 39.1157113595]], "y0" : -41, "y1" : 41 }, { "k" : "prism", "pts" : [[-38.494668686, 9.43495956], [-46.7491429655, 40.8692347126], [-51.5851852967, 39.5993155926], [-43.3307110172, 8.16504044]], "y0" : 22.65, "y1" : 27.4 }, { "k" : "prism", "pts" : [[-38.494668686, 9.43495956], [-46.7491429655, 40.8692347126], [-51.5851852967, 39.5993155926], [-43.3307110172, 8.16504044]], "y0" : -20.8, "y1" : -16.05 }], "cut" : [{ "k" : "poly", "origin" : [-40.6587060276, 0, 7.8327915338], "normal" : [-0.253983824, 0, 0.9672084662], "xdir" : [0, -1, 0], "pts" : [[-25.1, -0.3328427125], [-25.1, 0.3328427125], [-21.9328427125, 3.5], [-18.15, 3.5], [-18.15, -3.5], [-21.9328427125, -3.5]], "depth" : 33 }, { "k" : "poly", "origin" : [-40.6587060276, 0, 7.8327915338], "normal" : [-0.253983824, 0, 0.9672084662], "xdir" : [0, -1, 0], "pts" : [[18.5, -0.3328427125], [18.5, 0.3328427125], [15.3328427125, 3.5], [11.55, 3.5], [11.55, -3.5], [15.3328427125, -3.5]], "depth" : 33 }], "anchor" : [-57.6756222346, 0, 72.6357587714], "note" : "DUMMY: the wing proper, an inverted U over the stub: its legs hold the stub's chamfered ends, its body sits on the stub's top. The panel's own thickness. Retained up the panel: open.", "custom" : false, "mock" : false, "color" : [0.35, 0.55, 0.85], "kind" : "dummy" }, { "slug" : "rpinL", "name" : "rocker pin L", "group" : "wL", "up" : undefined, "add" : [{ "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -6.3, "y1" : 12.9, "r" : 3 }], "cut" : [], "anchor" : [-25.665791709, 3.3, 11.908139198], "note" : "Rod stock: pressed in the rocker and the knuckle, runs in the coupler between them.", "custom" : false, "mock" : false, "color" : [0.3, 0.3, 0.32], "kind" : "steel" }, { "slug" : "bulkheadF", "name" : "front bulkhead", "group" : "fixed", "up" : [0, -1, 0], "add" : [{ "k" : "prism", "pts" : [[-12.35, 25.09139], [-8.965752, -0.784402], [-8.86327, -1.562834], [-8.693332, -2.329371], [-8.457234, -3.078181], [-8.15677, -3.803564], [-7.794229, -4.5], [-7.372368, -5.162188], [-6.8944, -5.785088], [-6.363961, -6.363961], [-5.785088, -6.8944], [-5.162188, -7.372368], [-4.5, -7.794229], [-3.803564, -8.15677], [-3.078181, -8.457234], [-2.329371, -8.693332], [-1.562834, -8.86327], [-0.784402, -8.965752], [0, -9], [0.784402, -8.965752], [1.562834, -8.86327], [2.329371, -8.693332], [3.078181, -8.457234], [3.803564, -8.15677], [4.5, -7.794229], [5.162188, -7.372368], [5.785088, -6.8944], [6.363961, -6.363961], [6.8944, -5.785088], [7.372368, -5.162188], [7.794229, -4.5], [8.15677, -3.803564], [8.457234, -3.078181], [8.693332, -2.329371], [8.86327, -1.562834], [8.965752, -0.784402], [12.35, 25.09139], [12.35, 48.575519], [-12.35, 48.575519]], "y0" : 13.5, "y1" : 23.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 12.5, "y1" : 24.5, "r" : 3 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 12.5, "y1" : 24.5, "r" : 7.1 }], "anchor" : [0, 18.5, 12.5456948366], "note" : "Front journal bearing (ream to fit), the rod (ream to press). Print the inner face up.", "custom" : false, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "lowercase", "name" : "lower case", "group" : "fixed", "up" : [0, -1, 0], "add" : [{ "k" : "prism", "pts" : [[-12.35, -1.75861], [-5.785088, -6.8944], [-5.162188, -7.372368], [-4.5, -7.794229], [-3.803564, -8.15677], [-3.078181, -8.457234], [-2.329371, -8.693332], [-1.562834, -8.86327], [-0.784402, -8.965752], [0, -9], [0.784402, -8.965752], [1.562834, -8.86327], [2.329371, -8.693332], [3.078181, -8.457234], [3.803564, -8.15677], [4.5, -7.794229], [5.162188, -7.372368], [5.785088, -6.8944], [12.35, -1.75861], [12.35, 48.575519], [-12.35, 48.575519]], "y0" : -23.5, "y1" : -13.5 }, { "k" : "box", "lo" : [-12.35, -44.2, -1.7586103268], "hi" : [12.35, -23.5, 36.9413896732] }, { "k" : "box", "lo" : [-5.25, -37.4, 36.4413896732], "hi" : [5.25, -23, 47.5730339363] }], "cut" : [{ "k" : "box", "lo" : [-10.05, -54.7, 0.5413896732], "hi" : [10.05, -32.7, 34.6413896732] }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -33.2, "y1" : -23.5, "r" : 9.5 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -24.5, "y1" : -12.5, "r" : 7.1 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : -24.5, "y1" : -12.5, "r" : 3 }], "anchor" : [0, -18.5, 12.5456948366], "note" : "Rear journal bearing (ream to fit), the rod, the hub's cavity, the XC330's horn half. A block back over the servo holds the upper case's nut (slot from its top).", "custom" : false, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "uppercase", "name" : "upper case", "group" : "fixed", "up" : [0, 1, 0], "add" : [], "cut" : [], "anchor" : [-13.55, -59.2, 4.4163896732], "note" : "The XC330's back half, the servo turned long end down. One 6-32 forward into the lower case, from a lug on its top (45 deg back face). Connectors plug in through the cap.", "custom" : true, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "hub", "name" : "horn hub", "group" : "crank", "up" : [0, 1, 0], "add" : [{ "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -29.7, "y1" : -24.5, "r" : 9 }], "cut" : [{ "k" : "prism", "pts" : [[2.9344931419, 26.1166945059], [8.0256619665, 31.2078633304], [6.1164736573, 33.1170516396], [1.0253048327, 28.0258828151]], "y0" : -27.8, "y1" : -23.5 }, { "k" : "prism", "pts" : [[-1.0253048327, 28.0258828151], [-6.1164736573, 33.1170516396], [-8.0256619665, 31.2078633304], [-2.9344931419, 26.1166945059]], "y0" : -27.8, "y1" : -23.5 }, { "k" : "prism", "pts" : [[-2.9344931419, 24.0660848404], [-8.0256619665, 18.9749160159], [-6.1164736573, 17.0657277067], [-1.0253048327, 22.1568965312]], "y0" : -27.8, "y1" : -23.5 }, { "k" : "prism", "pts" : [[1.0253048327, 22.1568965312], [6.1164736573, 17.0657277067], [8.0256619665, 18.9749160159], [2.9344931419, 24.0660848404]], "y0" : -27.8, "y1" : -23.5 }, { "k" : "cyl", "x" : 6, "z" : 25.0913896732, "y0" : -30.7, "y1" : -23.5, "r" : 1.05 }, { "k" : "cyl", "x" : 6, "z" : 25.0913896732, "y0" : -26.3, "y1" : -23.5, "r" : 1.85 }, { "k" : "cyl", "x" : 0, "z" : 31.0913896732, "y0" : -30.7, "y1" : -23.5, "r" : 1.05 }, { "k" : "cyl", "x" : 0, "z" : 31.0913896732, "y0" : -26.3, "y1" : -23.5, "r" : 1.85 }, { "k" : "cyl", "x" : -6, "z" : 25.0913896732, "y0" : -30.7, "y1" : -23.5, "r" : 1.05 }, { "k" : "cyl", "x" : -6, "z" : 25.0913896732, "y0" : -26.3, "y1" : -23.5, "r" : 1.85 }, { "k" : "cyl", "x" : 0, "z" : 19.0913896732, "y0" : -30.7, "y1" : -23.5, "r" : 1.05 }, { "k" : "cyl", "x" : 0, "z" : 19.0913896732, "y0" : -26.3, "y1" : -23.5, "r" : 1.85 }], "anchor" : [4.6193976626, -29.2, 27.004806835], "note" : "aow-bike-steering's horn hub: M2 x 6 into the horn, the cross socket for the lugs.", "custom" : false, "mock" : false, "color" : [0.93, 0.56, 0.2], "kind" : "print" }, { "slug" : "washer", "name" : "centre washer", "group" : "crank", "up" : [0, 1, 0], "add" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -0.2, "y1" : 0.2, "r" : 5.5 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -1.2, "y1" : 1.2, "r" : 3 }], "anchor" : [4.2500000158, 0, 40.0755186068], "note" : "Loose on the crankpin between the couplers: print two layers, or cut from Delrin/PTFE shim.", "custom" : false, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "crankpin", "name" : "crankpin", "group" : "crank", "up" : undefined, "add" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -12.9, "y1" : 12.9, "r" : 3 }], "cut" : [], "anchor" : [0.0000000158, 1.3, 40.0755186068], "note" : "Rod stock: pressed in both webs, the couplers run on it.", "custom" : false, "mock" : false, "color" : [0.3, 0.3, 0.32], "kind" : "steel" }, { "slug" : "rod", "name" : "wing rod", "group" : "fixed", "up" : undefined, "add" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -23.5, "y1" : 23.5, "r" : 3 }], "cut" : [], "anchor" : [0, 0, 0], "note" : "Rod stock: pressed in both bearings, the hubs run on it.", "custom" : false, "mock" : false, "color" : [0.3, 0.3, 0.32], "kind" : "steel" }, { "slug" : "bridge", "name" : "bridge", "group" : "fixed", "up" : [0, 0, 1], "add" : [{ "k" : "box", "lo" : [-14.75, -23.5, 48.5755186068], "hi" : [14.75, 23.5, 56.5755186068] }], "cut" : [], "anchor" : [11.75, 12, 50.5755186068], "note" : "Ties the front bulkhead, lower and upper cases. Top face: the chassis joints.", "custom" : false, "mock" : false, "color" : [0.62, 0.64, 0.68], "kind" : "print" }, { "slug" : "chassis", "name" : "chassis plate (placeholder)", "group" : "fixed", "up" : [0, 0, 1], "add" : [{ "k" : "box", "lo" : [-16, -18, 56.5755186068], "hi" : [16, 23.5, 61.1755186068] }], "cut" : [], "anchor" : [14, 12, 57.5755186068], "note" : "PLACEHOLDER for the chassis: where the bridge's two 6-32 and ridges land.", "custom" : false, "mock" : true, "color" : [0.85, 0.85, 0.85], "kind" : "print" }, { "slug" : "floor", "name" : "floor (mock)", "group" : "fixed", "up" : undefined, "add" : [{ "k" : "box", "lo" : [-130, -64.7, -43.2], "hi" : [130, 33.5, -41.2] }], "cut" : [], "anchor" : [100, 0, -42.2], "note" : "", "custom" : false, "mock" : true, "color" : [0.85, 0.85, 0.85], "kind" : "mock" }];

export const RG_POSES = { "-1.00" : { "crank" : { "p" : [0, 25.0913896732], "a" : 230.7, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 2.6798048387, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 284.0766645315, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -40.2642141254, "d" : [-11.5953215024, -24.4747895809] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -32.9793165092, "d" : [-11.5953215024, -24.4747895809] } }, "-0.75" : { "crank" : { "p" : [0, 25.0913896732], "a" : 263.025, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -11.7730025422, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 306.6068553466, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -38.0264383263, "d" : [-14.8732348998, -16.803745373] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -27.1397088644, "d" : [-14.8732348998, -16.803745373] } }, "-0.56" : { "crank" : { "p" : [0, 25.0913896732], "a" : -72.408, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -14.207694032, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 322.7192991998, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -29.6586423238, "d" : [-14.2833643483, -10.4553737796] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -21.8297524981, "d" : [-14.2833643483, -10.4553737796] } }, "-0.50" : { "crank" : { "p" : [0, 25.0913896732], "a" : -64.65, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -14.0469614626, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 327.5318901749, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -26.5495191007, "d" : [-13.5412961599, -8.5687241421] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -19.9655521756, "d" : [-13.5412961599, -8.5687241421] } }, "-0.25" : { "crank" : { "p" : [0, 25.0913896732], "a" : -32.325, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -9.6533976711, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : -14.3105771293, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -12.9302418337, "d" : [-8.0123301116, -2.3221114666] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -11.0337329566, "d" : [-8.0123301116, -2.3221114666] } }, "+0.00" : { "crank" : { "p" : [0, 25.0913896732], "a" : 0, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 0, "d" : [0, 0] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 0, "d" : [0, 0] } }, "+0.25" : { "crank" : { "p" : [0, 25.0913896732], "a" : 32.325, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 14.3105771293, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 9.6533976711, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 11.0337329426, "d" : [8.0123301067, -2.3221114835] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 12.9302418423, "d" : [8.0123301067, -2.3221114835] } }, "+0.50" : { "crank" : { "p" : [0, 25.0913896732], "a" : 64.65, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 32.4681098251, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 14.0469614626, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 19.9655521429, "d" : [13.5412961418, -8.5687241707] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 26.549519113, "d" : [13.5412961418, -8.5687241707] } }, "+0.56" : { "crank" : { "p" : [0, 25.0913896732], "a" : 72.408, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 37.2807008002, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 14.207694032, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 21.8297524605, "d" : [14.2833643262, -10.4553738098] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 29.6586423362, "d" : [14.2833643262, -10.4553738098] } }, "+0.75" : { "crank" : { "p" : [0, 25.0913896732], "a" : 96.975, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 53.3931446534, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 11.7730025422, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 27.1397088113, "d" : [14.8732348643, -16.8037454044] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 38.0264383368, "d" : [14.8732348643, -16.8037454044] } }, "+1.00" : { "crank" : { "p" : [0, 25.0913896732], "a" : 129.3, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 75.9233354685, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : -2.6798048387, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 32.9793164383, "d" : [11.5953214507, -24.4747896054] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 40.2642141229, "d" : [11.5953214507, -24.4747896054] } } };


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
 * Case pins in the servo's OTHER hole row, onto `target`: aow-bike-steering's
 * nearRowPins, verbatim.
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

export function rgV(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]) * millimeter;
}

export function rgU(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]);
}

/**
 * One 6-32 x 3/8 joint: the shared screwJoint (screw bore, countersink, nut
 * slot, teardrops, bridging, ridge and groove) with two options it lacks: no
 * ridge (`j.ridge` false) and a BLIND slot (`j.through` false: out along
 * `j.slot` for `j.slotLen`, back only past the nut).
 */
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
        const r = ridgeSolid(context, id + "ridge", jp, rgU(j.ridgeDir), up, f.ridge_height,
                             0 * mm, j.ridgeHalf * mm);
        opBoolean(context, id + "rAdd", { "tools" : qUnion([ridgePart, r]),
                "operationType" : BooleanOperationType.UNION });
        const g = ridgeSolid(context, id + "groove", jp, rgU(j.ridgeDir), up, f.ridge_height,
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

/** A cylinder along Y, a prism from an (X, Z) or (Y, Z) outline, a slot, or a box. */
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
        // sketched on a world plane, as the prisms are, then moved into
        // place: sketched straight on its oblique plane it would not extrude
        // (EXTRUDE_FAILED, 2026-10-07). Local (a, -t, b) -> origin + a xdir
        // + t normal + b (normal x xdir).
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
        // sketch plane normal +Y (x axis +X, second axis -Z), or normal +X
        // (x axis +Y, second axis +Z)
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

/** One simple part: primitives united, cuts subtracted. */
export function rgPart(context is Context, id is Id, p is map)
{
    var adds = [];
    for (var i = 0; i < size(p.add); i += 1)
        adds = append(adds, rgPrim(context, id + ("a" ~ i), p.add[i]));
    if (size(adds) > 1)
        opBoolean(context, id + "uni", { "tools" : qUnion(adds),
                                         "operationType" : BooleanOperationType.UNION });
    // EVALUATED before the cutters exist: they are created under this same id
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

/** A part found by its anchor point, among everything the build made. */
export function rgFind(context is Context, scope is Id, slug is string) returns Query
{
    for (var p in RG_PARTS)
        if (p.slug == slug)
            return partAt(context, scope, rgV(p.anchor));
    throw regenError("no part " ~ slug);
}

/**
 * Every part and the hardware. Returns the bodies by motion group and each
 * printed part with the direction that prints UP.
 */
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

    // ---- the XC330, horn facing +Y, its long end up (or down: RG.servoX); the near hole row
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

    // ---- lower case: the shared horn-half shell onto the body, near-row pins
    step("lower case shell");
    caseShellBuild(context, id + "lcShell", cs, mergeMaps(CASE_OPT, { "part" : "TOP" }),
                   rgFind(context, id, "lowercase"));
    if (RG.lowerNearPins > 0.5)
        nearRowPins(context, id + "lcNear", cs, "TOP", rgFind(context, id, "lowercase"));

    // ---- upper case: the shared back-half shell, the wrap and the pad, the
    // nest, the cavity, the cable window and the connector slots
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

    // ---- the 6-32 joints
    step("joints");
    for (var j in RG.joints)
    {
        if (!opt.mocks && (j.csk == "chassis" || j.nut == "chassis"))
            continue;
        step("joint " ~ j.tag);
        rgJoint(context, id + j.tag, j, rgFind(context, id, j.csk), rgFind(context, id, j.nut));
    }

    // ---- names, colours, groups, print orientation
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

/** The transform that puts one group at one pose. */
export function rgXf(g is map) returns Transform
{
    const mm = millimeter;
    return transform(vector(g.d[0], 0, g.d[1]) * mm)
         * rotationAround(line(vector(g.p[0], 0, g.p[1]) * mm, vector(0, 1, 0)), g.a * degree);
}

/** Move every moving group to pose `key` from rest, or back with `back`. */
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

// ==== UI LAYER BELOW -- dropped by --check ====

export enum RightingPose
{
    annotation { "Name" : "Rest" } REST,
    annotation { "Name" : "Right wing 25%" } R25,
    annotation { "Name" : "Right wing 50%" } R50,
    annotation { "Name" : "Right wing 75%" } R75,
    annotation { "Name" : "Right wing down (100%)" } R100,
    annotation { "Name" : "Left wing 25%" } L25,
    annotation { "Name" : "Left wing 50%" } L50,
    annotation { "Name" : "Left wing 75%" } L75,
    annotation { "Name" : "Left wing down (100%)" } L100
}

annotation { "Feature Type Name" : "AOW righting",
             "Feature Type Description" : "Printed swing linkage: crankshaft on two bearings, XC330 cases and horn hub, wings; Y forward, origin on the wing rod" }
export const aowRighting = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Pose" }
        definition.pose is RightingPose;
        annotation { "Name" : "Show mocks (servo, chassis plate, floor)", "Default" : true }
        definition.mocks is boolean;
    }
    {
        const r = rightingBuild(context, id + "build", { "mocks" : definition.mocks });
        const keys = { RightingPose.REST : "+0.00", RightingPose.R25 : "+0.25", RightingPose.R50 : "+0.50", RightingPose.R75 : "+0.75", RightingPose.R100 : "+1.00", RightingPose.L25 : "-0.25", RightingPose.L50 : "-0.50", RightingPose.L75 : "-0.75", RightingPose.L100 : "-1.00" };
        const key = keys[definition.pose];
        if (key != "+0.00")
            rgPose(context, id + "pose", r.groups, key, false);
        reportFeatureInfo(context, id, "Diamond x1.25: crank 14.98, coupler 38.11, rocker 28.29; crank axis 25.09 above the rod, rod 41.2 above the floor; rods 6 mm");
    });
