"""The front steer module: mock front wheel, fork, headset, the steer XC330's
cases and horn hub, as ONE custom feature, `AOW steering`, in its own Feature
Studio. Spec: docs/plans/steering-design.md. Numbers:
config/steering_cad.yaml, plus the X330 envelope and mount interface
cad_servo_mount reads, plus bike_params_cad's steering.servo_clearance.

MODULE FRAME: Z up the steering axis, Y forward, X right, origin on the front
axle's centre. The servo's horn faces DOWN the axis and its body runs back
(-Y). In the bike the module tilts back by rake_deg about the axle.

    part            print   what it is
    fork R / L      X- / X+ leg plate, axle boss inward to the hub; two 6-32
                            each, countersunk outside, into the block
    headset lower   Z+      block between the fork tops, shaft up through the
                            bushing, double-D spigot; the central nut in a
                            side slot in the shaft, trapped by the bushing
    headset upper   Z+      double-D socket, the central 6-32 countersunk
                            from the top, lugs up into the hub
    lower case      Z+      the plain bushing, the cup round the turning
                            parts, walls round the servo's horn half, one 6-32
    upper case      Z-      the back-half shell capped over the whole back
                            face, a pad with the second 6-32
    horn hub        Z- / Z+ M2 x 6 self-tappers (SCREWS) or horn pins (PINS);
                            the cross socket
    mount plate     Y+      PLACEHOLDER for the fixture/chassis

No ball bearing: the toy has none, and a printed sleeve can be reamed. The
central screw goes in from the TOP, because the block's underside is 4 mm off
the tire and a screw from below is unreachable with the wheel on.

The screw joint, ridge, case shells, horn pins and X330 envelope are the AHRS
fixture's own FeatureScript (cad_ahrs_fixture.FS_HELPERS), and so is the
print check.

    python -m aow_sim.cad_steering                     # write docs/cad/steering.fs
    python -m aow_sim.cad_steering --check             # both variants, ONE call
    python -m aow_sim.cad_steering --push steering_features
    python -m aow_sim.cad_steering --shot              # -> docs/cad/steering.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml

from . import cad_ahrs_fixture as af
from . import cad_servo_mount as sm
from .params import _normalize, load_params

STEER_PARAMS = "config/steering_cad.yaml"
X330_PARAMS = "config/x330_fixture_cad.yaml"
OUT_FS = "docs/cad/steering.fs"
OUT_PNG = "docs/cad/steering{}.png"
SPLIT_MARK = af.SPLIT_MARK
PARTS = ("fork R", "fork L", "headset lower", "headset upper", "horn hub",
         "lower case", "upper case", "mount plate")
HARDWARE = ("X330 case", "X330 horn", "wheel", "tire", "axle")
# Interference that is the design: both legs' holes are a press fit on the
# axle (one size, so the fork halves are one part).
INTENDED = ({"axle", "fork L"}, {"axle", "fork R"})
# (attach, key) built by --check: between them every dialog option.
VARIANTS = (("SCREWS", "CROSS"), ("PINS", "FLAT"))
SWEEP = (30, 90, 180, 270)      # steer angles the clash sweep visits
PLA = 1.24e-3                   # g / mm^3, solid


def load(steer_path: str = STEER_PARAMS, cad_path: str = sm.CAD_PARAMS,
         mount_path: str = sm.MOUNT_PARAMS) -> dict:
    """Everything the module reads, in MILLIMETRES."""
    raw = _normalize(yaml.safe_load(Path(steer_path).read_text()))
    params = load_params(cad_path)
    mounts = sm.load_mounts(mount_path)
    servo = params["servos"][af.SERVO_KEY]
    d, w, h = servo["box_size"]
    mm = lambda v: v * 1000.0    # noqa: E731
    s = {sec: {k: (v if k.endswith("_deg") else mm(v)) for k, v in vals.items()}
         for sec, vals in raw.items()}
    # The PINS hub's horn attach is the X330 fixture's LATEST (user,
    # 2026-09-28), read from its own config so the two cannot drift: one pin
    # diameter set directly, no root relief, the Phi 16.0 well.
    fx = _normalize(yaml.safe_load(Path(X330_PARAMS).read_text()))["fixture"]
    horn_pins = {"pinLength": mm(fx["horn_pin_length"]),
                 "pinDiameter": mm(fx["horn_pin_diameters"][0]),
                 "rootRelief": mm(fx["horn_pin_root_relief"]),
                 "boreClearance": mm(fx["horn_bore_clearance"]),
                 "boreMouthChamfer": mm(fx["horn_bore_mouth_chamfer"])}
    x330 = {"caseDepth": mm(d), "caseWidth": mm(w), "caseHeight": mm(h),
            "shaftFromEnd": mm(servo["shaft_from_end"]),
            "hornThickness": mm(servo["horn_thickness"]),
            "hornDiameter": mm(servo["horn_diameter"])}
    return {"s": s, "x330": x330, "horn_pins": horn_pins,
            "servo_clearance": mm(params["bike"]["steering"]["servo_clearance"]),
            "rake_deg": params["bike"]["rake_deg"],
            "table": sm.servo_table(params, mounts),
            "screw": sm.screw_table(mounts)}


def layout(data: dict) -> dict:
    """Every derived dimension, module frame, mm. The FeatureScript builds from
    exactly these (the ST map), and --check measures the parts against them.
    Raises if the stack does not fit or a wall goes too thin."""
    s, sv, sc = data["s"], data["x330"], data["screw"]
    t = {k: v * 1000 for k, v in data["table"]["XC330"].items() if k != "hornHoleCount"}
    wh, ax, fk, hs, cp, hb, cs = (s[k] for k in ("wheel", "axle", "fork", "headset",
                                                 "coupling", "hub", "cases"))
    wall = s["print"]["min_wall"]
    bl = s["joint"]["bridge_layer"]
    hole_r = sc["holeDia"] / 2

    def need(ok: bool, what: str) -> None:
        if not ok:
            raise ValueError(what)

    R = wh["tire_od"] / 2
    L = {"R": R}
    # ---- from the top: the horn face, then hub, lugs, headset upper, step
    L["zH"] = R + data["servo_clearance"]
    L["zCF"] = L["zH"] + sv["hornThickness"]
    L["zBack"] = L["zCF"] + sv["caseDepth"]
    L["hubBot"] = L["zH"] - hb["thickness"]
    L["socketFloor"] = L["hubBot"] + cp["socket_depth"]
    L["upTop"] = L["hubBot"] - cp["hub_gap"]
    L["lugTop"] = L["socketFloor"] - cp["lug_tip_gap"]
    socket = hs["spigot_height"] + hs["socket_extra_depth"]
    L["nutFloor"] = L["upTop"] - hs["nut_pocket_depth"]
    L["zS"] = L["nutFloor"] - hs["web"] - 2 * bl - socket   # the step: upper sits here
    L["socketTop"] = L["zS"] + socket
    # the upper's own socket ceiling: the nominal one, trimmed (upper only)
    L["upSockTop"] = L["socketTop"] - hs["upper_socket_trim"]
    need(L["upSockTop"] > L["zS"] + 1.0, "upper_socket_trim leaves almost no socket")
    L["sleeveTop"] = L["zS"] - hs["end_play"]
    # ---- from the bottom: the block over the tire
    L["blockBot"] = R + fk["tire_gap"]
    L["blockTop"] = L["blockBot"] + fk["block_height"]
    L["caseBot"] = L["blockTop"] + hs["thrust_boss_height"]
    L["sleeve"] = L["sleeveTop"] - L["caseBot"]
    need(L["sleeve"] >= hs["min_sleeve"],
         f"the bushing is {L['sleeve']:.2f} mm, under min_sleeve {hs['min_sleeve']:.2f}: "
         "raise steering.servo_clearance")
    # ---- the central 6-32, from below: tip just past the nut in the upper,
    # head seated in the shaft up a long bore
    L["nutTop"] = L["nutFloor"] + hs["nut_thickness"]
    L["tip"] = L["nutTop"] + hs["tip_past_nut"]
    need(L["tip"] <= L["upTop"] - 0.05, "the central screw's tip stands out of the upper")
    L["headFace"] = L["tip"] - hs["central_screw_length"]
    cone = (sc["headDia"] - sc["holeDia"]) / 2 / math.tan(math.radians(sc["cskAngle"] / 2))
    L["cskTop"] = L["headFace"] + cone
    L["cbR"] = (sc["headDia"] + hs["head_bore_clearance"]) / 2
    L["cbTop"] = L["headFace"] - (L["cbR"] - sc["headDia"] / 2)   # the 45 deg seat runs on to the bore
    need(L["cskTop"] <= L["zS"], "the central screw's head seat reaches the step: a longer screw")
    L["shaftR"] = hs["shaft_dia"] / 2
    need(L["shaftR"] - L["cbR"] >= 2 * wall, "the shaft is too thin round the central screw's bore")
    # ---- fork and axle, along X
    L["xIn"] = wh["tire_width"] / 2 + fk["tire_side_clearance"]
    L["xOut"] = (ax["length"] - ax["head_height"]) / 2   # shank leg face to leg face, head proud on +X
    L["xBoss"] = wh["hub_width"] / 2 + fk["hub_gap"]
    L["knurlEnd"] = L["xOut"] - ax["knurl_length"]
    need(L["knurlEnd"] >= L["xBoss"], "the axle's knurl runs past the +X boss into the wheel")
    need(fk["boss_dia"] < min(wh["hub_insert_dia_a"], wh["hub_insert_dia_b"]),
         "the bosses are wider than the smaller hub insert")
    L["xBlk"] = fk["block_half_width"]
    L["ledge"] = L["xBlk"] - L["xIn"]
    need(L["ledge"] > 1.0, "the block is no wider than the gap between the legs: no ledge")
    jp = s["joint"]["joint_plate"]
    L["forkPocket"] = L["xOut"] - L["xBlk"] - jp
    need(L["forkPocket"] >= 0, "the fork plate beside the block is thinner than the joint plate")
    L["zFS"] = (L["blockBot"] + L["blockTop"]) / 2
    need(sc["headDia"] < fk["block_height"] - 2 * wall, "the fork joint's head does not fit the block's height")
    # the cheeks round the block's +-Y faces: as deep as the ledge is wide
    L["cheekIn"] = fk["block_depth"] / 2 + fk["cheek_clearance"]
    L["forkHalf"] = L["cheekIn"] + fk["cheek_thickness"]
    need(fk["cheek_thickness"] >= 2 * wall, "the fork's cheeks are under two walls thick")
    need(L["forkHalf"] > fk["leg_width"] / 2, "the fork's top is narrower than its leg")
    # the fork joint's nut slot, through the block along Z, must miss the thrust boss
    L["forkSlotX"] = L["xBlk"] - 2.0 - sc["nutSlotThickness"]
    L["thrustR"] = hs["thrust_boss_dia"] / 2
    need(L["forkSlotX"] >= L["thrustR"], "the fork's nut slots cut into the thrust boss")
    need(hs["thrust_boss_dia"] <= fk["block_depth"], "the thrust boss overhangs the block")
    # ---- the coupling, radially
    L["nutR"] = sc["nutSlotWidth"] / math.sqrt(3)          # the nut pocket's corners
    L["lugR0"] = L["nutR"] + wall
    L["upR"] = hs["upper_radius"]
    L["lugHalf"] = cp["lug_width"] / 2
    L["lugR1"] = math.sqrt(L["upR"] ** 2 - L["lugHalf"] ** 2)   # corners on the rim
    L["sockHalf"] = L["lugHalf"] + cp["lug_clearance"]
    L["sockR0"] = cp["socket_r0"]
    need(L["sockR0"] <= L["lugR0"] - cp["lug_clearance"], "the socket starts outside the lugs' roots")
    L["islandWall"] = 2 * (L["sockR0"] * math.sin(math.radians(45)) - L["sockHalf"])
    need(L["islandWall"] >= wall, f"the hub's centre island is {L['islandWall']:.2f} mm between arms")
    L["hubR"] = hb["radius"]
    bc = t["hornBoltCircle"] / 2
    L["hubCbR"] = (hb["screw_head_dia"] + hb["screw_head_clearance"]) / 2
    L["hubCbDepth"] = hb["thickness"] - hb["screw_flange"]
    L["m2Engaged"] = hb["screw_length"] - hb["screw_flange"]
    L["cbToSocket"] = bc * math.sin(math.radians(45)) - L["sockHalf"] - L["hubCbR"]
    need(L["cbToSocket"] >= wall, f"only {L['cbToSocket']:.2f} mm between an M2 counterbore and the socket")
    L["cbToRim"] = L["hubR"] - bc - L["hubCbR"]
    need(L["cbToRim"] >= wall, "the M2 counterbores break through the hub's rim")
    need(hb["thickness"] - cp["socket_depth"] >= wall, "the socket floor is thinner than min_wall")
    need(L["m2Engaged"] <= t["hornHoleDepthMax"] - 0.2, "the M2 x 6 would bottom in the horn's hole")
    need(L["hubCbDepth"] >= hb["screw_head_height"] + 0.1, "the M2 heads stand proud of the hub")
    # ---- the spigot and its socket
    c = hs["spigot_clearance"]
    L["spR"], L["spFlat"] = hs["spigot_dia"] / 2, hs["spigot_flats"] / 2
    L["sockSpR"], L["sockSpFlat"] = L["spR"] + c, L["spFlat"] + c
    L["boreR"] = (hs["shaft_dia"] + hs["bore_clearance"]) / 2
    need(L["spFlat"] - hole_r >= wall, "the spigot's flats are too close to the screw")
    need(L["shaftR"] - L["spR"] >= 0.75, "the step the upper sits on is under 0.75 mm wide")
    need(L["upR"] - L["sockSpR"] >= 2 * wall, "the upper's wall round the socket is too thin")
    # ---- the cases
    inner = sv["caseWidth"] / 2 + t["caseSideClearance"]
    L["inner"] = inner
    L["topOuter"] = inner + t["caseTopWall"]
    L["nestBore"] = L["topOuter"] + t["caseNestClearance"]
    L["botOuter"] = L["nestBore"] + t["caseBottomWall"]
    L["cavR"] = max(hb["radius"], hs["upper_radius"]) + cs["cavity_clearance"]
    need(L["cavR"] <= inner, "the turning cavity is wider than the servo pocket: a ledge hangs over it")
    endY = sv["caseHeight"] - sv["shaftFromEnd"]
    # the case's near hole row, 7.5 toward the shaft end: does a pin there stand
    # on solid, clear of the turning cavity?
    near = math.hypot(t["caseHoleSpanX"] / 2, endY - sv["caseHeight"] / 2 - t["casePinRowOffset"])
    L["lowerPinSeat"] = near - (t["caseReliefDia"] - t["casePinClearance"]) / 2 - L["cavR"]
    L["lowerNearPins"] = 1.0 if (cs["lower_near_pins"] > 0
                                 and L["lowerPinSeat"] >= cs["min_pin_seat"]) else 0.0
    L["yServoNear"] = sv["shaftFromEnd"] + t["caseSideClearance"]     # module +Y
    L["yServoFar"] = -(endY + t["caseSideClearance"])                 # module -Y
    L["yWallNear"] = L["yServoNear"] + t["caseTopWall"]
    L["yWallFar"] = L["yServoFar"] - t["caseTopWall"]
    L["yNestNear"] = L["yWallNear"] + t["caseNestClearance"]
    L["yUpNear"] = L["yNestNear"] + t["caseBottomWall"]
    L["yUpFar"] = L["yWallFar"] - t["caseNestClearance"] - t["caseBottomWall"]
    L["yWrap"] = -(endY - t["caseWrapLength"])                       # where the shells' far wrap starts
    L["cupHalf"] = max(L["topOuter"], L["cavR"] + cs["cup_wall"])
    L["cupNear"] = max(L["yWallNear"], L["cavR"] + cs["cup_wall"])
    L["wallTop"] = L["zBack"] - t["caseGripLength"]
    L["upBot"] = L["wallTop"] - t["caseNestLength"]
    L["upTopZ"] = L["zBack"] + t["caseFaceClearance"] + t["caseCapThickness"]
    # the cable window through the upper case's side walls, bridged over
    m = cs["cable_window_margin"]
    L["winNear"] = -t["caseWindowNear"] + m
    L["winFar"] = L["yWrap"]
    need(L["zBack"] - t["caseWindowDepth"] >= L["wallTop"] + wall,
         "the lower case's walls reach the cable connectors")
    # the window cuts the upper's grip wall right down to the lower case's
    # walls; the edge across it is in the nest shell, a gable: its apex
    L["winBot"] = L["wallTop"]
    L["winApex"] = L["winBot"] - (L["winNear"] - L["winFar"]) / 2 * math.tan(math.radians(cs["bridge_taper_deg"]))
    need(L["winApex"] >= L["upBot"] + 2.0, "the window's gable runs out of the upper case's wall")
    # the connector slots through the cap, each side from the centre strip out
    L["slotIn"] = cs["cable_strip_width"] / 2
    need(L["inner"] - L["slotIn"] >= 2.0, "the connector slots are too narrow to reach a plug through")
    L["yP"] = L["yUpFar"] - cs["pad_depth"]                          # the mount plane
    L["plateBack"] = L["yP"] - jp
    L["zLS"] = (L["caseBot"] + L["zCF"]) / 2
    L["zUS"] = L["upTopZ"] - cs["pad_height"] / 2
    L["padBot"] = L["upTopZ"] - cs["pad_height"]
    need(L["padBot"] >= L["wallTop"] + 1.0, "the upper case's pad reaches down to the lower case's walls")
    return L


def extents(L: dict, attach: str) -> dict[str, tuple[float, float]]:
    """Each part's Z extent as built -- what --check measures."""
    hub_top = L["zH"] + (L["wellT"] if attach == "PINS" else 0.0)
    return {"headset lower": (L["blockBot"], L["zS"] + L["spigotH"]),
            "headset upper": (L["zS"], L["lugTop"]),
            "horn hub": (L["hubBot"], hub_top),
            "lower case": (L["caseBot"], L["wallTop"]),
            "upper case": (L["upBot"], L["upTopZ"]),
            "mount plate": (L["caseBot"], L["upTopZ"]),
            "fork R": (-L["legHalf"], L["blockTop"]),
            "fork L": (-L["legHalf"], L["blockTop"])}


def full_layout(data: dict) -> dict:
    """layout() plus the few config values the FeatureScript reads directly."""
    s = data["s"]
    L = layout(data)
    t = data["table"]["XC330"]
    L.update({
        "tireOD": s["wheel"]["tire_od"], "tireW": s["wheel"]["tire_width"],
        "tireFlat": s["wheel"]["tire_flat_width"], "tireMin": s["wheel"]["tire_min_dia"],
        "webW": s["wheel"]["web_width"], "hubW": s["wheel"]["hub_width"],
        "hubDiaA": s["wheel"]["hub_dia_a"], "hubDiaB": s["wheel"]["hub_dia_b"],
        "bossDia": s["fork"]["boss_dia"],
        "axleDia": s["axle"]["dia"], "headH": s["axle"]["head_height"],
        "headDia": s["axle"]["head_dia"],
        "axleHole": s["axle"]["hole_dia"], "bossChamferDeg": s["fork"]["boss_chamfer_deg"],
        "tabL": s["plate"]["tab_length"], "tabW": s["plate"]["tab_width"],
        "tabT": s["plate"]["tab_thickness"], "tabHole": s["plate"]["tab_hole_dia"],
        "tabSpanX": s["plate"]["tab_span_x"], "tabSpanD": s["plate"]["tab_span_along"],
        "rakeDeg": data["rake_deg"],
        "legHalf": s["fork"]["leg_width"] / 2, "blockHalf": s["fork"]["block_depth"] / 2,

        "spigotH": s["headset"]["spigot_height"],
        "m2Hole": s["hub"]["screw_hole_dia"],
        "padHalf": s["cases"]["pad_half_width"], "ridgeHalf": s["cases"]["ridge_half"],
        "bridgeLayer": s["joint"]["bridge_layer"],
        "wellT": t["hornThickness"] * 1000 - t["caseOffset"] * 1000,
    })
    return L


def mass_estimate(vols: dict[str, float]) -> str:
    return ", ".join(f"{k} {v * PLA:.1f} g" for k, v in vols.items())


def _fs_map(name: str, d: dict) -> str:
    rows = []
    for k, v in d.items():
        if isinstance(v, str):
            rows.append(f'    "{k}" : "{v}"')
        elif k in ("cskAngle",) or k.endswith("Deg"):
            rows.append(f'    "{k}" : {v:g} * degree')
        else:
            rows.append(f'    "{k}" : {v:.6g} * millimeter')
    return f"export const {name} = {{\n" + ",\n".join(rows) + "\n};"


FS = r'''FeatureScript %VERSION%;
import(path : "onshape/std/geometry.fs", version : "%VERSION%.0");

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

%X330%

%ST%

%FIXTURE%

%HORN_OPT%

%CASE_OPT%

%SCREW_OPT%

// ---- the servo-mount geometry, copied from the horn-mount-gen studio ----
%SERVO_MOUNT%
// ---- end of the copy ----

%HELPERS%
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

%SPLIT%

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
'''


def horn_opt(data: dict) -> dict:
    """The shared horn-pin dialog numbers with the X330 fixture's overrides:
    pins set by DIAMETER (as a clearance in the Phi 1.6 hole), no root
    relief, the well at Phi 16.0 with a 0.4 lead-in."""
    t = data["table"]["XC330"]
    d = {"servo": "XC330", **{k: t[k] * 1000 for k in sm.DIALOG}}
    hp = data["horn_pins"]
    d.update({k: hp[k] for k in ("pinLength", "rootRelief", "boreClearance", "boreMouthChamfer")})
    d["pinClearance"] = t["hornHoleDia"] * 1000 - hp["pinDiameter"]
    return d


def build_fs(data: dict, fs_version: str = "3044") -> str:
    t = data["table"]["XC330"]
    pick = lambda keys: {"servo": "XC330", **{k: t[k] * 1000 for k in keys}}  # noqa: E731
    subs = {
        "%VERSION%": fs_version,
        "%X330%": _fs_map("X330", data["x330"]),
        "%ST%": _fs_map("ST", full_layout(data)),
        "%FIXTURE%": _fs_map("FIXTURE", data["s"]["joint"]),
        "%HORN_OPT%": _fs_map("HORN_OPT", horn_opt(data)),
        "%CASE_OPT%": _fs_map("CASE_OPT", pick(sm.CASE_DIALOG)),
        "%SCREW_OPT%": _fs_map("SCREW_OPT", data["screw"]),
        "%SERVO_MOUNT%": af._servo_mount_layer(data, fs_version),
        "%HELPERS%": af.FS_HELPERS.rstrip("\n") + "\n",
        "%SPLIT%": SPLIT_MARK,
    }
    text = FS
    for k, v in subs.items():
        text = text.replace(k, v)
    return text


def check_wrapper(fs: str, variants=VARIANTS) -> str:
    """Build each variant, then print: every body, each part's Z extent and
    volume, every collision at rest and at each steer angle in SWEEP (turning
    parts against fixed ones), and the print check. Each variant built,
    checked and deleted before the next -- all in ONE call."""
    vs = "[" + ", ".join(f'["{a}", "{k}"]' for a, k in variants) + "]"
    sweep = "[" + ", ".join(f"{a}" for a in SWEEP) + "]"
    return f"""function(context is Context, queries)
{{
{sm.geometry_layer(fs, SPLIT_MARK)}
    const name = function(q) returns string
    {{
        const n = getProperty(context, {{ "entity" : q, "propertyType" : PropertyType.NAME }});
        return n == undefined ? "UNNAMED" : n;
    }};
    const clashes = function(pose is string, tools, targets)
    {{
        for (var c in evCollision(context, {{ "tools" : tools, "targets" : targets }}))
            println("COLL|" ~ pose ~ "|" ~ name(c.toolBody) ~ "|" ~ name(c.targetBody)
                    ~ "|" ~ toString(c["type"]));
    }};
    for (var va in {vs})
    {{
    println("VER=" ~ va[0] ~ "/" ~ va[1]);
    const id = makeId("chk" ~ va[0] ~ va[1]);
    const r = steeringBuild(context, id, {{ "attach" : va[0], "key" : va[1], "debug" : true }});
    const all = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    for (var b in all)
    {{
        const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
        println("BODY|" ~ name(b) ~ "|" ~ toString(bb.minCorner[2] / millimeter) ~ "|"
                ~ toString(bb.maxCorner[2] / millimeter) ~ "|"
                ~ toString(evVolume(context, {{ "entities" : b }}) / (millimeter * millimeter * millimeter)));
    }}
    for (var i = 0; i + 1 < size(all); i += 1)
        clashes("rest", all[i], qUnion(subArray(all, i + 1, size(all))));
    for (var q in r.stages["static"])
        setAttribute(context, {{ "entities" : q, "name" : "stg_static", "attribute" : "static" }});
    for (var q in r.stages["steer"])
        setAttribute(context, {{ "entities" : q, "name" : "stg_steer", "attribute" : "steer" }});
    const st = qHasAttribute("stg_static");
    const tu = qHasAttribute("stg_steer");
    var k = 0;
    for (var a in {sweep})
    {{
        opTransform(context, id + ("t" ~ k), {{ "bodies" : tu, "transform" : rotationAround(r.axis, a * degree) }});
        clashes("steer " ~ a, tu, st);
        opTransform(context, id + ("tb" ~ k), {{ "bodies" : tu, "transform" : rotationAround(r.axis, -a * degree) }});
        k += 1;
    }}
{af.PRINT_CHECK_FS}    opDeleteBodies(context, id + "clear", {{ "entities" : qCreatedBy(id, EntityType.BODY) }});
    }}
    return "ran to completion";
}}
"""


def canon(name: str) -> str:
    hits = [k for k in PARTS + HARDWARE if name == k or name.startswith(k + " ")]
    return max(hits, key=len) if hits else name


def check(text: str, data: dict, target: str | None, variants=VARIANTS) -> bool:
    """ONE billable call for every variant."""
    from . import onshape

    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text, variants), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    Path("traces").mkdir(exist_ok=True)
    Path("traces/steering_check.txt").write_text(console)
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console[-3000:])
        print(onshape.budget_line())
        return False
    sections = console.split("VER=")[1:]
    ok = len(sections) == len(variants)
    if not ok:
        print(f"  ran {len(sections)} of {len(variants)} variants -- FAIL")
    for sec in sections:
        ver, _, body = sec.partition("\n")
        print(f"--- {ver}")
        ok &= judge(body, data, ver.split("/")[0].strip())
    print(onshape.budget_line())
    return ok


def judge(console: str, data: dict, attach: str) -> bool:
    rows = [l.split("|") for l in console.splitlines()]
    L = full_layout(data)
    want = extents(L, attach)
    ok = True
    bodies = [r for r in rows if r[0] == "BODY"]
    strays = [r for r in bodies if r[1] == "UNNAMED"]
    ok &= not strays
    if strays:
        print(f"  {len(strays)} UNNAMED bodies -- a cutter left behind  FAIL")
    vols = {}
    for r in bodies:
        n = canon(r[1])
        if n in want:
            lo, hi = float(r[2]), float(r[3])
            good = abs(lo - want[n][0]) < 1e-3 and abs(hi - want[n][1]) < 1e-3
            ok &= good
            vols[n] = float(r[4])
            print(f"  {n:14} z {lo:8.3f} .. {hi:8.3f}  (want {want[n][0]:8.3f} .. "
                  f"{want[n][1]:8.3f})  {'ok' if good else 'FAIL'}")
    counts = {r[1]: int(r[2]) for r in rows if r[0] == "PART"}
    for n, c in counts.items():
        ok &= c == 1
        if c != 1:
            print(f"  part {n} is {c} bodies  FAIL")
    ok &= len(counts) == len(PARTS)
    colls = [r for r in rows if r[0] == "COLL" and "ABUT" not in r[4]]
    real = [r for r in colls if {canon(r[2]), canon(r[3])} not in INTENDED]
    ok &= not real
    print(f"  interference, at rest and steered {SWEEP} deg: {len(real)}"
          f"  (+{len(colls) - len(real)} intended: the axle's press fit in the legs)")
    fv = [vols.get("fork R"), vols.get("fork L")]
    same = None not in fv and abs(fv[0] - fv[1]) < 1e-3
    ok &= same
    print(f"  fork halves the same part (volumes {fv}): {'ok' if same else 'FAIL'}")
    for r in real[:40]:
        print(f"    {r[1]:10} {r[2]}  x  {r[3]}  ({r[4]})")
    print(f"  solid PLA: {mass_estimate(vols)}")
    for r in rows:
        if r[0] == "BED":
            over = sorted((x for x in rows if x[0] == "OVER" and x[1] == r[1]),
                          key=lambda o: -float(o[2]))
            print(f"    {r[1]:14} bed {float(r[2]):7.1f} mm^2, {len(over)} downward faces"
                  + "".join(f"\n        {float(o[2]):7.2f} mm^2, {float(o[3]):6.2f} up, at ({o[4]})"
                            for o in over))
    crowns = [r for r in rows if r[0] == "CROWN"]
    print(f"  horizontal holes with a flat crown as printed (want a teardrop): {len(crowns)}")
    for r in crowns:
        print(f"    {r[1]:14} r {r[2]} mm, axis through ({r[3]})")
    hangs = [r for r in rows if r[0] == "HANG"]
    print(f"  level edges hanging in the air as printed: {len(hangs)}")
    for r in hangs:
        print(f"    {r[1]:14} {float(r[2]):5.2f} mm long, {float(r[3]):6.2f} up, at ({r[4]})")
    return ok


def report(data: dict) -> str:
    """The axial stack, top down, for the terminal and the plan doc."""
    L = full_layout(data)
    rows = [("zBack", "servo back face"), ("winApex", "cable window's gable apex (upper case nest shell)"),
            ("wallTop", "lower case walls' top"), ("zCF", "servo case face (horn side)"),
            ("zH", "horn face"), ("socketFloor", "hub socket floor"), ("lugTop", "lug tips"),
            ("hubBot", "hub underside"), ("upTop", "headset upper top"), ("tip", "central screw tip"),
            ("nutFloor", "nut pocket floor"), ("socketTop", "spigot socket ceiling"),
            ("zS", "step: headset upper sits here"), ("sleeveTop", "bushing top"),
            ("cskTop", "central screw seat (top of the cone)"), ("headFace", "central screw head face"),
            ("caseBot", "thrust face: block's boss / lower case underside"), ("blockTop", "block top"),
            ("zFS", "fork screw"), ("blockBot", "block underside, on the fork ledges"), ("R", "tire top")]
    out = [f"  {L[k]:7.2f}  {what}" for k, what in rows]
    out.append(f"  bushing {L['sleeve']:.2f} mm; M2 engages {L['m2Engaged']:.2f} of the horn's 3.0\n"
               f"  hub walls: counterbore-socket {L['cbToSocket']:.2f}, counterbore-rim "
               f"{L['cbToRim']:.2f}, centre island {L['islandWall']:.2f}; lug roots "
               f"{L['lugR0'] - L['nutR']:.2f} off the nut pocket\n"
               f"  lower case near-row pins: {'ON' if L['lowerNearPins'] else 'OFF'}, "
               f"{L['lowerPinSeat']:.2f} mm clear of the turning cavity; fork ledges "
               f"{L['ledge']:.2f} wide")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steer", default=STEER_PARAMS)
    ap.add_argument("--params", default=sm.CAD_PARAMS)
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="build both variants in Onshape and check them; ONE billable "
                         "call, defaults to the `check` tab")
    ap.add_argument("--push", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="replace a Feature Studio's contents (its own tab only)")
    ap.add_argument("--shot", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="render a Part Studio (default `steering`); ONE billable call")
    ap.add_argument("--view", default="isometric")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the check script instead of spending a call")
    args = ap.parse_args()

    data = load(args.steer, args.params)
    text = build_fs(data, args.fs_version)
    sm.lint_fs(text)
    Path(args.output).write_text(text)
    print(f"wrote {len(text)} chars -> {args.output}")
    print(report(data))
    if args.dry_run:
        print(check_wrapper(text))
        return
    if args.check is not None:
        if not check(text, data, args.check or None):
            raise SystemExit("check FAILED -- not pushing")
    if args.push is not None:
        from . import onshape
        if args.push != "steering_features":
            raise SystemExit(f"refusing to push at {args.push or 'the default'!r}: "
                             "this generator owns `steering_features` only")
        url = onshape.resolve(args.push, args.push)
        reply = onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}  (microversion "
              f"{reply.get('sourceMicroversion', '?')})")
        print(onshape.budget_line())
    if args.shot is not None:
        from . import onshape
        url = onshape.resolve(args.shot or None, "steering")
        tag = "" if args.view == "isometric" else "_" + args.view
        out, _ = onshape.shaded_view(url, Path(OUT_PNG.format(tag)), view=args.view)
        print(f"rendered -> {out}")
        print(onshape.budget_line())


if __name__ == "__main__":
    main()
