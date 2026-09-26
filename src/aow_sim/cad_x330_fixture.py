"""The X330 load-test fixture: one bare X330, shaft horizontal, a two-armed lever.

For the load test in docs/plans/pre-assembly-bench-checklist.md: known masses
hung on the lever measure the back-drive slip torque with torque OFF and the
Goal Current that lifts them with torque ON -- splitting the ~22 mA
breakaway `analysis/servo_breakaway.py` measured into deadband and friction.
Generates ONE custom feature, `X330 fixture`, into its own Feature Studio,
with TWO VERSIONS picked in its dialog -- each with the friction the real
roll mechanism would have, and no more:

    IDLER (easiest to build; less like the rig's roll) -- the main one
    plate   Z+      small clamp plate, tab to -X; heads counterbored flush
    base    X+      back-half case shell, leg down to the plate, one 6-32
    cover   X-      horn-half case shell, full wrap, leg down to the plate,
                    one 6-32: the masses' off-axis torque goes through it too
    lever   X-      horn disc and pins, 50 mm arms both ways, each with a
                    10-32 mass slot and 5 mm ticks; leg up to the yoke
    yoke    X+      idler plug, over the top to the lever -- the AHRS
                    fixture's roll horn mount + idler U -- and the STOP: a
                    V underside (centre flat) that lands flat on the cover's
                    top, widened by ledges, at +-stop_deg (45, the full rig's)

    SCREWDRIVER (the rig's roll mechanism: the servo only drives)
    plate   Z+      clamp plate + open-top cradle, spring channel, a stop post
                    under each arm
    adapter X-      horn pins on one face, a key slot across the other
    key     Z+      loose flat key between the adapter's and the hub's slots
    lever   X+      hub with its slot, arms both ways, boss, shaft
    block   X+      two 608 bearings the shaft runs in, one 6-32 down

THE ORIENTATION is the AHRS fixture's roll servo: far end DOWN, horn facing
+X, so the lever swings in the vertical Y-Z plane. Masses are a 10-32 screw
and washers in either arm's slot, lined up by the ticks; the lever's own
weight balances. IDLER stops at `stop_deg` (45, the full rig's roll stop)
either side of level; SCREWDRIVER, whose plate is too close under the arms
for that, at `post_stop_deg` (10) on posts.

HORN PINS at the original 2.6 mm (the shared default is 1.3 since 2026-09-23):
the load goes through them. Case pins stay at 1.5. In SCREWDRIVER the spring
pushes the servo +X -- horn, adapter, hub, the first bearing's INNER race --
and the cradle walls stop the case turning while letting it float; the key
between two slots forgives misalignment in one direction.

The screw joint, ridge, case shells, idler and X330 envelope are the AHRS
fixture's own FeatureScript (cad_ahrs_fixture.FS_HELPERS), and so is the
print check.

    python -m aow_sim.cad_x330_fixture                 # write docs/cad/x330_fixture.fs
    python -m aow_sim.cad_x330_fixture --check         # both versions, ONE call
    python -m aow_sim.cad_x330_fixture --push x330_fixture_features
    python -m aow_sim.cad_x330_fixture --shot          # IDLER tab -> docs/cad/x330_fixture_idler.png
    python -m aow_sim.cad_x330_fixture --shot x330_fixture_screwdriver --version SCREWDRIVER
    python -m aow_sim.cad_x330_fixture --shot --version COUPON   # the pin coupon's tab

Numbers: config/x330_fixture_cad.yaml, plus the X330 envelope and mount
interface cad_servo_mount reads.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml

from . import cad_ahrs_fixture as af
from . import cad_servo_mount as sm
from .params import _normalize, load_params

FIXTURE_PARAMS = "config/x330_fixture_cad.yaml"
OUT_FS = "docs/cad/x330_fixture.fs"
OUT_PNG = "docs/cad/x330_fixture_{}.png"
SPLIT_MARK = af.SPLIT_MARK
VERSIONS = ("IDLER", "SCREWDRIVER")
PARTS = {"IDLER": ("plate", "base", "cover", "hub", "lever", "yoke"),
         "SCREWDRIVER": ("plate", "block", "adapter", "key", "lever")}
HARDWARE = ("X330 case", "X330 horn", "608 bearing", "spring")
# The lever only swings between its stops, so the clash sweep visits each
# stop from both sides: just short of it must be clear, 1 deg past it the
# lever must hit that post and nothing else.
DEG_KEYS = ("stop_deg", "post_stop_deg", "blade_flank_deg")
# What the lever must hit, and only that, 1 deg past its stop.
STOP_PAIR = {"IDLER": {"yoke", "cover"}, "SCREWDRIVER": {"lever", "plate"}}


def load(fixture_path: str = FIXTURE_PARAMS, cad_path: str = sm.CAD_PARAMS,
         mount_path: str = sm.MOUNT_PARAMS) -> dict:
    """Everything the fixture reads, in MILLIMETRES (degrees for angles)."""
    raw = _normalize(yaml.safe_load(Path(fixture_path).read_text()))
    params = load_params(cad_path)
    mounts = sm.load_mounts(mount_path)
    servo = params["servos"][af.SERVO_KEY]
    d, w, h = servo["box_size"]
    mm = lambda v: v * 1000.0    # noqa: E731
    fx = {k: (v if k in DEG_KEYS else [mm(x) for x in v] if isinstance(v, list) else mm(v))
          for k, v in raw["fixture"].items()}
    x330 = {"caseDepth": mm(d), "caseWidth": mm(w), "caseHeight": mm(h),
            "shaftFromEnd": mm(servo["shaft_from_end"]),
            "hornThickness": mm(servo["horn_thickness"]),
            "hornDiameter": mm(servo["horn_diameter"])}
    return {"fixture": fx, "x330": x330,
            "table": sm.servo_table(params, mounts),
            "screw": sm.screw_table(mounts)}


def layout(data: dict, version: str = "IDLER") -> dict:
    """The derived dimensions, world mm -- what the FeatureScript must agree with."""
    f, sv = data["fixture"], data["x330"]
    t = {k: v * 1000 for k, v in data["table"]["XC330"].items() if k != "hornHoleCount"}
    far = sv["caseHeight"] - sv["shaftFromEnd"]
    s = math.radians(f["stop_deg"])
    out = {"leverEnd": f["arm_length"]}
    if version == "IDLER":
        shell_y1 = far + t["caseSideClearance"] + t["caseTopWall"] + t["caseNestClearance"] + t["caseBottomWall"]
        walls = t["caseSideClearance"] + t["caseTopWall"]
        cy, cx = sv["shaftFromEnd"] + walls, sv["caseWidth"] / 2 + walls
        corner = math.hypot(cy, cx)
        sS, cS = math.sin(s), math.cos(s)
        W = (f["stop_flat"] + cy * sS) / cS + f["stop_contact"]
        hub = hub_stack(data)
        out.update(plateTop=-shell_y1 - f["base_gap"], hubTop=hub["hubTop"],
                   R_s=max(corner, math.hypot(W, cy)) + f["sweep_clearance"] + f["leg_width"] / 2,
                   # the V stop's centre flat: faces at stop_deg through z = cy
                   stopHeight=(cy + f["stop_flat"] * sS) / cS,
                   ledgeHalfWidth=W,
                   leverTop=hub["leverTop"])
    else:
        r_out = f["stop_radius"] + f["stop_post"] / 2
        s = math.radians(f["post_stop_deg"])
        bx0 = f["adapter_top"] + f["hub_thickness"] + f["boss_length"]
        out.update(plateTop=-far, blockX0=bx0,
                   blockX1=bx0 + 2 * f["bearing_width"] + f["bearing_gap"],
                   postTop=-(r_out * math.sin(s) + f["arm_height"] / 2 * math.cos(s)))
    out["plateBottom"] = out["plateTop"] - f["joint_plate"] - f["head_recess"]
    return out


def hub_stack(data: dict) -> dict:
    """The IDLER's horn hub and the lever on it, along the shaft (X, mm from
    the horn's outer face). Raises if the 6-32 x 3/8 cannot reach its nut, or
    would reach the horn."""
    f, sc = data["fixture"], data["screw"]
    hub_top = f["nut_pocket_depth"] + f["hub_web"] + f["blade_depth"]
    lever0 = hub_top + f["blade_gap"]
    lever_top = lever0 + f["arm_thickness"]
    tip = f["screw_tip_clearance"]
    head = tip + f["screw_length"]
    csk = (sc["headDia"] - sc["holeDia"]) / 2 / math.tan(math.radians(sc["cskAngle"] / 2))
    nut0 = f["nut_pocket_depth"] - f["nut_thickness"]     # the nut, pulled to the pocket's end
    out = {"hubTop": hub_top, "lever0": lever0, "leverTop": lever_top, "screwTip": tip,
           "headTop": head, "headRecess": lever_top - head, "cskBottom": head - csk,
           "nutEngaged": f["nut_pocket_depth"] - max(tip, nut0)}
    if not 0 < tip <= nut0:
        raise ValueError(f"the screw tip at {tip:.2f} mm does not pass the whole nut "
                         f"({nut0:.2f}..{f['nut_pocket_depth']:.2f}) short of the horn")
    if out["headRecess"] < 0 or out["cskBottom"] < lever0 + 0.5:
        raise ValueError(f"the 6-32 head does not fit in the lever: {out}")
    return out


def _fs_map(name: str, d: dict) -> str:
    rows = []
    for k, v in d.items():
        if isinstance(v, str):
            rows.append(f'    "{k}" : "{v}"')
        elif isinstance(v, list):
            rows.append(f'    "{k}" : [' + ", ".join(f"{x:.4g} * millimeter" for x in v) + "]")
        elif k in DEG_KEYS or k == "cskAngle":
            rows.append(f'    "{k}" : {v:g} * degree')
        else:
            rows.append(f'    "{k}" : {v:.4g} * millimeter')
    return f"export const {name} = {{\n" + ",\n".join(rows) + "\n};"


FS = r'''FeatureScript %VERSION%;
import(path : "onshape/std/geometry.fs", version : "%VERSION%.0");

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

%X330%

%FIXTURE%

%HORN_OPT%

%CASE_OPT%

%IDLER_OPT%

%SCREW_OPT%

// ---- the servo-mount geometry, copied from the horn-mount-gen studio ----
%SERVO_MOUNT%
// ---- end of the copy ----

%HELPERS%
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
    // the horn attach, with this fixture's overrides (config: THE HORN ATTACH)
    const horn    = mergeMaps(HORN_OPT, { "pinLength" : f.horn_pin_length,
                                          "rootRelief" : f.horn_pin_root_relief,
                                          "boreClearance" : f.horn_bore_clearance,
                                          "boreMouthChamfer" : f.horn_bore_mouth_chamfer });
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
        // The cover's cap ends just under the horn face.
        const capOuter = hornZ + CASE_OPT.caseFaceClearance + f.cover_cap_thickness;
        // ---- the horn HUB and the lever on it (config: THE HORN HUB). Along
        // X from the horn face: nut pocket, web, the blade groove, blade_gap,
        // the lever.
        const hubTop  = f.nut_pocket_depth + f.hub_web + f.blade_depth;
        const armX0   = hubTop + f.blade_gap;
        const leverTop = armX0 + f.arm_thickness;
        const xJ = leverTop - jpT;
        const tB  = tan(f.blade_flank_deg);
        const bw  = f.blade_width / 2;
        const gF  = hubTop - f.blade_depth;            // the groove's floor
        const holeR = SCREW_OPT.holeDia / 2;
        const headR = SCREW_OPT.headDia / 2;
        const headTop = f.screw_tip_clearance + f.screw_length;
        const cskBot  = headTop - (headR - holeR) / tan(SCREW_OPT.cskAngle / 2);
        // the blade runs across the hub, inside its round
        const bl = sqrt(collarR * collarR - bw * bw) - 0.2 * mm;

        // One hub per horn_pin_diameters entry: the first is assembled, each
        // other one is built in place, then parked behind the servo as a
        // spare to print. Built spares-first, so partAt finds one hub at a time.
        const nutR = SCREW_OPT.nutSlotWidth / sqrt(3);
        var hex = [];
        for (var k = 0; k < 6; k += 1)
            hex = append(hex, vector(nutR * cos(k * 60 * degree), nutR * sin(k * 60 * degree)));
        const hubPt = vector(f.nut_pocket_depth + f.hub_web / 2, 0 * mm, collarR - 1.5 * mm);
        var spares = [];
        for (var i = size(f.horn_pin_diameters) - 1; i >= 0; i -= 1)
        {
            const hT = "hub" ~ i;
            step(hT);
            cylW(context, P + (hT ~ "Disc"), vector(-wellT, 0 * mm, 0 * mm), vector(hubTop, 0 * mm, 0 * mm), collarR);
            servoMountBuild(context, P + (hT ~ "Horn"), cs, mergeMaps(horn, {
                    "pinClearance" : SERVO_MOUNT_TABLE["XC330"].hornHoleDia - f.horn_pin_diameters[i] }),
                    partAt(context, P, hubPt));
            // groove along Y in the outer face, profile in (z, x)
            polyPrism(context, P, hT ~ "Groove", vector(0 * mm, -collarR - 1 * mm, 0 * mm), Y, Z,
                      [vector(-bw, hubTop + 1 * mm), vector(-bw, hubTop),
                       vector(-(bw - f.blade_depth * tB), gF), vector(bw - f.blade_depth * tB, gF),
                       vector(bw, hubTop), vector(bw, hubTop + 1 * mm)], 2 * collarR + 2 * mm);
            // hex pocket for the nut in the horn-side floor, profile in (y, z)
            polyPrism(context, P, hT ~ "Nut", vector(-1 * mm, 0 * mm, 0 * mm), X, Y, hex,
                      f.nut_pocket_depth + 1 * mm);
            const hubHole = cylW(context, P + (hT ~ "Hole"), vector(-1 * mm, 0 * mm, 0 * mm),
                                 vector(hubTop + 1 * mm, 0 * mm, 0 * mm), holeR);
            opBoolean(context, P + (hT ~ "Cut"), { "tools" : qUnion([
                    qCreatedBy(P + (hT ~ "GrooveExt"), EntityType.BODY),
                    qCreatedBy(P + (hT ~ "NutExt"), EntityType.BODY), hubHole]),
                    "targets" : partAt(context, P, hubPt),
                    "operationType" : BooleanOperationType.SUBTRACTION });
            if (i > 0)
            {
                const park = vector(-60 * mm - i * 25 * mm, 0 * mm, 0 * mm);
                opTransform(context, P + (hT ~ "Park"), { "bodies" : partAt(context, P, hubPt),
                        "transform" : transform(park) });
                spares = append(spares, [i, hubPt + park]);
            }
        }

        step("lever");
        // ---- lever: disc over the hub, arms both ways, the blade under it,
        // leg up to the yoke
        const disc = cylW(context, P + "disc", vector(armX0, 0 * mm, 0 * mm),
                          vector(leverTop, 0 * mm, 0 * mm), collarR);
        const la = leverArms(context, P + "lv", armX0, leverTop);
        const leg = boxW(context, P + "leg", vector(xJ, -lw / 2, 0 * mm),
                         vector(leverTop, lw / 2, Rs + lw / 2));
        // the blade: the groove's own flanks, its tip blade_gap off the floor,
        // a neck of the groove's mouth width up into the lever
        const tipX = gF + f.blade_gap;
        polyPrism(context, P, "blade", vector(0 * mm, -bl, 0 * mm), Y, Z,
                  [vector(-bw, armX0 + 0.5 * mm), vector(-bw, hubTop),
                   vector(-(bw - (hubTop - tipX) * tB), tipX), vector(bw - (hubTop - tipX) * tB, tipX),
                   vector(bw, hubTop), vector(bw, armX0 + 0.5 * mm)], 2 * bl);
        unite(context, P + "leverU", [disc, la.arms, leg, qCreatedBy(P + "bladeExt", EntityType.BODY)]);
        // in the arm's wall above the mass slot, below the ticks
        leverPt = vector((armX0 + leverTop) / 2, -30 * mm, (f.mass_slot_width + f.arm_height) / 4);
        // counterbore + countersink from the outer face, then the hole
        revolveProfile(context, P, "csk", plane(O, cross(Y, X), Y), line(O, X),
                       [vector(0 * mm, cskBot), vector(holeR, cskBot), vector(headR, headTop),
                        vector(headR + f.counterbore_clearance / 2, headTop),
                        vector(headR + f.counterbore_clearance / 2, leverTop + 1 * mm),
                        vector(0 * mm, leverTop + 1 * mm)]);
        const lvHole = cylW(context, P + "lvHole", vector(tipX - 1 * mm, 0 * mm, 0 * mm),
                            vector(leverTop + 1 * mm, 0 * mm, 0 * mm), holeR);
        step("armCut");
        opBoolean(context, P + "armCut", { "tools" : qUnion([la.holes, lvHole,
                qCreatedBy(P + "cskRev", EntityType.BODY)]),
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

        const pinName = function(i) returns string
        {
            return " (pins " ~ toString(roundToPrecision(f.horn_pin_diameters[i] / mm, 2)) ~ ")";
        };
        parts = [["plate", pl.pt, "Z+", Z, "static"], ["base", basePt, "X+", X, "static"],
                 ["cover", coverPt, "X-", -X, "static"], ["hub" ~ pinName(0), hubPt, "X-", -X, "lever"],
                 ["lever", leverPt, "X-", -X, "lever"],
                 ["yoke", yokePt, "X+", X, "lever"]];
        for (var sp in spares)
            parts = append(parts, ["spare hub" ~ pinName(sp[0]), sp[1], "X-", -X, "static"]);
        L = { "plateTop" : zTop, "plateBottom" : pl.zBot, "stopHeight" : (cy + a * sS) / cS,
              "ledgeHalfWidth" : W, "hubTop" : hubTop,
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
        servoMountBuild(context, P + "adHorn", cs, mergeMaps(horn, {
                "pinClearance" : SERVO_MOUNT_TABLE["XC330"].hornHoleDia - f.horn_pin_diameters[0] }),
                partAt(context, P, adPt));
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

/**
 * The pin coupon: a set of four pins on the horn's bolt circle per
 * (diameter, length), standing +Z on a flat board, one part. Row r is
 * coupon_diameters[r] (notches on the -X edge: r + 1), column c is
 * coupon_lengths[c] (notches on the -Y edge: c + 1). Plain cylinders, no
 * chamfers: the fixture's pins have none either.
 */
export function x330PinCouponBuild(context is Context, id is Id) returns Query
{
    const f  = FIXTURE;
    const t  = SERVO_MOUNT_TABLE["XC330"];
    const mm = millimeter;
    const p  = f.coupon_pitch;
    const nR = size(f.coupon_diameters);
    const nC = size(f.coupon_lengths);
    const board = boxW(context, id + "board", vector(0 * mm, 0 * mm, -f.coupon_board),
                       vector(nC * p, nR * p, 0 * mm));
    var bodies = [board];
    for (var r = 0; r < nR; r += 1)
        for (var c = 0; c < nC; c += 1)
            for (var k = 0; k < t.hornHoleCount; k += 1)
            {
                const a = (45 + k * 360 / t.hornHoleCount) * degree;
                const x = (c + 0.5) * p + t.hornBoltCircle / 2 * cos(a);
                const y = (r + 0.5) * p + t.hornBoltCircle / 2 * sin(a);
                bodies = append(bodies, cylW(context, id + ("pin" ~ r ~ "_" ~ c ~ "_" ~ k),
                        vector(x, y, -f.coupon_board / 2), vector(x, y, f.coupon_lengths[c]),
                        f.coupon_diameters[r] / 2));
            }
    unite(context, id + "all", bodies);
    var notches = [];
    for (var r = 0; r < nR; r += 1)
        for (var k = 0; k <= r; k += 1)
        {
            const y = (r + 0.5) * p + (k - r / 2) * 1.6 * mm;
            notches = append(notches, boxW(context, id + ("nr" ~ r ~ "_" ~ k),
                    vector(-1 * mm, y - 0.4 * mm, -f.coupon_board - 1 * mm), vector(1 * mm, y + 0.4 * mm, 1 * mm)));
        }
    for (var c = 0; c < nC; c += 1)
        for (var k = 0; k <= c; k += 1)
        {
            const x = (c + 0.5) * p + (k - c / 2) * 1.6 * mm;
            notches = append(notches, boxW(context, id + ("nc" ~ c ~ "_" ~ k),
                    vector(x - 0.4 * mm, -1 * mm, -f.coupon_board - 1 * mm), vector(x + 0.4 * mm, 1 * mm, 1 * mm)));
        }
    const q = qContainsPoint(qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID),
                             vector(nC * p / 2, nR * p / 2, -f.coupon_board / 2));
    opBoolean(context, id + "notch", { "tools" : qUnion(notches), "targets" : q,
            "operationType" : BooleanOperationType.SUBTRACTION });
    var rows = "";
    for (var r = 0; r < nR; r += 1)
        rows = rows ~ (r == 0 ? "" : ", ") ~ toString(roundToPrecision(f.coupon_diameters[r] / mm, 3));
    var cols = "";
    for (var c = 0; c < nC; c += 1)
        cols = cols ~ (c == 0 ? "" : ", ") ~ toString(roundToPrecision(f.coupon_lengths[c] / mm, 2));
    dress(context, q, "pin coupon [print Z+]", color(0.93, 0.56, 0.20),
          "Rows (-X edge notches 1..): pin dia " ~ rows ~ " mm. Columns (-Y edge notches 1..): length "
          ~ cols ~ " mm. Print Z+ up.");
    return q;
}

%SPLIT%

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

annotation { "Feature Type Name" : "X330 pin coupon",
             "Feature Type Description" : "Horn pin sets by diameter and length on one board, for the slicer" }
export const x330PinCoupon = defineFeature(function(context is Context, id is Id,
                                                    definition is map)
    precondition
    {
    }
    {
        x330PinCouponBuild(context, id + "build");
    });
'''


def build_fs(data: dict, fs_version: str = "3044") -> str:
    t = data["table"]["XC330"]
    pick = lambda keys: {"servo": "XC330", **{k: t[k] * 1000 for k in keys}}  # noqa: E731
    subs = {
        "%VERSION%": fs_version,
        "%X330%": _fs_map("X330", data["x330"]),
        "%FIXTURE%": _fs_map("FIXTURE", data["fixture"]),
        "%HORN_OPT%": _fs_map("HORN_OPT", pick(sm.DIALOG)),
        "%CASE_OPT%": _fs_map("CASE_OPT", pick(sm.CASE_DIALOG)),
        "%IDLER_OPT%": _fs_map("IDLER_OPT", pick(sm.IDLER_DIALOG)),
        "%SCREW_OPT%": _fs_map("SCREW_OPT", data["screw"]),
        "%SERVO_MOUNT%": af._servo_mount_layer(data, fs_version),
        "%HELPERS%": af.FS_HELPERS.rstrip("\n") + "\n",
        "%SPLIT%": SPLIT_MARK,
    }
    text = FS
    for k, v in subs.items():
        text = text.replace(k, v)
    return text


def check_wrapper(fs: str, data: dict, versions=VERSIONS) -> str:
    """Build each version, then print: the layout, every body, every collision
    at rest and at each stop from both sides, the lever's own mass properties,
    and the print check. Both versions in ONE call, each built, checked and
    deleted before the next. Judged in `check`."""
    f = data["fixture"]
    vs = "[" + ", ".join(f'"{v}"' for v in versions) + "]"
    PRINT_CHECK = af.PRINT_CHECK_FS
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
    for (var ver in {vs})
    {{
    println("VER=" ~ ver);
    const id = makeId("chk" ~ ver);
    const r = x330FixtureBuild(context, id, {{ "version" : ver, "debug" : true }});
    for (var key in keys(r.L))
        println(key ~ "=" ~ toString(r.L[key] / millimeter));
    const all = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    for (var b in all)
        println("BODY|" ~ name(b));
    for (var i = 0; i + 1 < size(all); i += 1)
        clashes("rest", all[i], qUnion(subArray(all, i + 1, size(all))));
    // the lever's (and yoke's) own weight about the shaft: volume, centroid
    const mv = qUnion(subArray(r.stages["lever"], 1, size(r.stages["lever"])));
    const lv = evVolume(context, {{ "entities" : mv }});
    const lc = evApproximateCentroid(context, {{ "entities" : mv }});
    println("LEVERVOL=" ~ toString(lv / (millimeter * millimeter * millimeter)));
    println("LEVERCY=" ~ toString(lc[1] / millimeter));
    for (var q in r.stages["static"])
        setAttribute(context, {{ "entities" : q, "name" : "stg_static", "attribute" : "static" }});
    for (var q in r.stages["lever"])
        setAttribute(context, {{ "entities" : q, "name" : "stg_lever", "attribute" : "lever" }});
    const st = qHasAttribute("stg_static");
    const lev = qHasAttribute("stg_lever");
    var k = 0;
    const stopDeg = ver == "SCREWDRIVER" ? {f['post_stop_deg']:g} : {f['stop_deg']:g};
    for (var a in [stopDeg - 0.5, -(stopDeg - 0.5), stopDeg + 1, -(stopDeg + 1)])
    {{
        opTransform(context, id + ("t" ~ k), {{ "bodies" : lev,
                "transform" : rotationAround(r.shaft, a * degree) }});
        clashes("lever " ~ a, lev, st);
        opTransform(context, id + ("tb" ~ k), {{ "bodies" : lev,
                "transform" : rotationAround(r.shaft, -a * degree) }});
        k += 1;
    }}
{PRINT_CHECK}    opDeleteBodies(context, id + "clear", {{ "entities" : qCreatedBy(id, EntityType.BODY) }});
    }}
    // the pin coupon: one named body, its size and volume
    const cid = makeId("chkCoupon");
    x330PinCouponBuild(context, cid);
    const cb = evaluateQuery(context, qBodyType(qCreatedBy(cid, EntityType.BODY), BodyType.SOLID));
    for (var b in cb)
    {{
        const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
        println("CPN|" ~ name(b) ~ "|" ~ toString(evVolume(context, {{ "entities" : b }})
                / (millimeter * millimeter * millimeter)) ~ "|"
                ~ toString((bb.maxCorner - bb.minCorner) / millimeter));
    }}
    return "ran to completion";
}}
"""


def canon(name: str, version: str) -> str:
    hits = [k for k in PARTS[version] + HARDWARE if name == k or name.startswith(k + " ")]
    return max(hits, key=len) if hits else name


def check(text: str, data: dict, target: str | None, versions=VERSIONS) -> bool:
    """ONE billable call for every version."""
    from . import onshape

    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text, data, versions), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console[-3000:])
        print(onshape.budget_line())
        return False
    ok = judge_coupon(console, data)
    sections = console.split("VER=")[1:]
    ok &= len(sections) == len(versions)
    if len(sections) != len(versions):
        print(f"  ran {len(sections)} of {len(versions)} versions -- FAIL")
    for sec in sections:
        ver, _, body = sec.partition("\n")
        print(f"--- {ver}")
        ok &= judge(body, data, ver.strip())
    print(onshape.budget_line())
    return ok


def coupon_volume(data: dict) -> float:
    """What the coupon should measure [mm^3]: board less its notches, plus
    every pin above the board."""
    f, t = data["fixture"], data["table"]["XC330"]
    p, tb = f["coupon_pitch"], f["coupon_board"]
    dia, ln = f["coupon_diameters"], f["coupon_lengths"]
    n = int(t["hornHoleCount"])
    notches = sum(r + 1 for r in range(len(dia))) + sum(c + 1 for c in range(len(ln)))
    pins = sum(n * math.pi * (d / 2) ** 2 * L for d in dia for L in ln)
    return len(ln) * p * len(dia) * p * tb - notches * 1.0 * 0.8 * tb + pins


def judge_coupon(console: str, data: dict) -> bool:
    rows = [l.split("|") for l in console.splitlines() if l.startswith("CPN|")]
    want = coupon_volume(data)
    print("--- COUPON")
    ok = len(rows) == 1 and rows[0][1].startswith("pin coupon")
    for r in rows:
        v = float(r[2])
        good = abs(v - want) / want < 1e-3
        ok &= good
        print(f"  {r[1]}: {v:.1f} mm^3 (want {want:.1f}) {'ok' if good else 'FAIL'}, box {r[3]} mm")
    if len(rows) != 1:
        print(f"  {len(rows)} coupon bodies, want 1  FAIL")
    return ok


def judge(console: str, data: dict, version: str) -> bool:
    rows = [l.split("|") for l in console.splitlines()]
    got = dict(l.split("=", 1) for l in console.splitlines() if "=" in l and "|" not in l)
    L = layout(data, version)
    ok = True
    for k, want in L.items():
        v = float(got.get(k, "nan"))
        bad = not abs(v - want) < 1e-3
        ok &= not bad
        print(f"  {k:12} wanted {want:8.3f}  got {v:8.3f}  {'FAIL' if bad else 'ok'}")
    for k in sorted(set(got) - set(L) - {"LEVERVOL", "LEVERCY"}):
        print(f"  {k:12} measured {float(got[k]):8.3f}")
    strays = sum(1 for r in rows if r[0] == "BODY" and r[1] == "UNNAMED")
    ok &= not strays
    if strays:
        print(f"  {strays} UNNAMED bodies -- a cutter left behind  FAIL")
    counts = {r[1]: int(r[2]) for r in rows if r[0] == "PART"}
    for n, c in counts.items():
        ok &= c == 1
        print(f"  part {n:10} {c} body  {'ok' if c == 1 else 'FAIL'}")
    spares = len(data["fixture"]["horn_pin_diameters"]) - 1 if version == "IDLER" else 0
    ok &= len(counts) == len(PARTS[version]) + spares
    stop = data["fixture"]["stop_deg" if version == "IDLER" else "post_stop_deg"]
    colls = [r for r in rows if r[0] == "COLL" and "ABUT" not in r[4]]
    past = {f"lever {stop + 1:g}", f"lever {-(stop + 1):g}"}
    real = [r for r in colls if r[1] not in past]
    ok &= not real
    print(f"  interference at rest and just short of both stops (+-{stop - 0.5:g} deg): {len(real)}")
    for r in real[:40]:
        print(f"    {r[1]:12} {r[2]}  x  {r[3]}  ({r[4]})")
    for pose in sorted(past):
        hit = [r for r in colls if r[1] == pose]
        good = bool(hit) and all({canon(r[2], version), canon(r[3], version)} == STOP_PAIR[version]
                                 for r in hit)
        ok &= good
        print(f"  the stop, {pose.split()[1]} deg: the lever hits "
              f"{', '.join(sorted({canon(r[2], version) + ' x ' + canon(r[3], version) for r in hit})) or 'NOTHING'}"
              f"  {'ok' if good else 'FAIL'}")
    vol, cy = float(got.get("LEVERVOL", "nan")), float(got.get("LEVERCY", "nan"))
    g_solid = vol * 1.24e-3
    print(f"  moving parts: {vol:.0f} mm^3 (= {g_solid:.1f} g at solid PLA), centroid "
          f"{cy:+.2f} mm off the shaft in Y -> {g_solid * 9.81e-3 * abs(cy):.2f} mN m level if solid")
    for r in rows:
        if r[0] == "BED":
            over = sorted((x for x in rows if x[0] == "OVER" and x[1] == r[1]),
                          key=lambda o: -float(o[2]))
            print(f"    {r[1]:10} bed {float(r[2]):7.1f} mm^2, {len(over)} downward faces"
                  + "".join(f"\n        {float(o[2]):7.2f} mm^2, {float(o[3]):6.2f} up, at ({o[4]})"
                            for o in over))
    crowns = [r for r in rows if r[0] == "CROWN"]
    print(f"  horizontal holes with a flat crown as printed (want a teardrop): {len(crowns)}")
    for r in crowns:
        print(f"    {r[1]:10} r {r[2]} mm, axis through ({r[3]})")
    hangs = [r for r in rows if r[0] == "HANG"]
    print(f"  level edges hanging in the air as printed: {len(hangs)}")
    for r in hangs:
        print(f"    {r[1]:10} {float(r[2]):5.2f} mm long, {float(r[3]):6.2f} up, at ({r[4]})")
    return ok


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fixture", default=FIXTURE_PARAMS)
    ap.add_argument("--params", default=sm.CAD_PARAMS)
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="build both versions in Onshape and check them; ONE billable "
                         "call, defaults to the `check` tab")
    ap.add_argument("--push", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="replace a Feature Studio's contents (its own tab only)")
    ap.add_argument("--shot", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="render a Part Studio (default `x330_fixture`); ONE billable call")
    ap.add_argument("--version", choices=VERSIONS + ("COUPON",), default="IDLER",
                    help="which version --shot's Part Studio shows (names the png)")
    ap.add_argument("--view", default="isometric")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the check script instead of spending a call")
    args = ap.parse_args()

    data = load(args.fixture, args.params)
    text = build_fs(data, args.fs_version)
    sm.lint_fs(text)
    Path(args.output).write_text(text)
    print(f"wrote {len(text)} chars -> {args.output}")
    for v in VERSIONS:
        L = layout(data, v)
        print(f"  {v}: shaft {-L['plateBottom']:.2f} mm above the plate's bottom; "
              f"lever reach +-{L['leverEnd']:.1f} mm")
    if args.dry_run:
        print(check_wrapper(text, data))
        return
    if args.check is not None:
        if not check(text, data, args.check or None):
            raise SystemExit("check FAILED -- not pushing")
    if args.push is not None:
        from . import onshape
        if args.push in ("", "feature_studio", "horn_features", "swing_features",
                         "fixture_features", "floor_rig_gen"):
            raise SystemExit(f"refusing to push at {args.push or 'the default'!r}: "
                             "each generator owns its own studio tab")
        url = onshape.resolve(args.push, args.push)
        reply = onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}  (microversion "
              f"{reply.get('sourceMicroversion', '?')})")
        print(onshape.budget_line())
    if args.shot is not None:
        from . import onshape
        url = onshape.resolve(args.shot or None,
                              "x330_pin_coupon" if args.version == "COUPON" else "x330_fixture")
        tag = args.version.lower() + ("" if args.view == "isometric" else "_" + args.view)
        out, _ = onshape.shaded_view(url, Path(OUT_PNG.format(tag)), view=args.view)
        print(f"rendered -> {out}")
        print(onshape.budget_line())


if __name__ == "__main__":
    main()
