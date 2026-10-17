"""The rear drive module: mock rear wheel, the two XC430s, the spline pulleys,
drive pulleys, case sides and chainstays (and, on request, a mock fixture),
as ONE custom feature, `AOW drive`, in its own Feature Studio. Spec:
docs/plans/drive-design.md. Numbers: config/drive_cad.yaml, plus
bike_params_cad's belt, XC430 and omni wheel blocks.

MODULE FRAME: origin on the rear axle, +Y along the mean belt (toward the
servos), +X right, +Z up. In the bike it tilts up by drivetrain.
drive_servo_angle_deg (45) about X.

Every part is built on the LEFT and its right twin is the same solid rotated
180 deg about Y, which maps servo A (horn -X, below the belt line) onto servo
B (horn +X, above it). So each pair is one part, by construction.

    part             print   what it is
    spline pulley    X+/X-   15T; spline stub into the wheel's side gear; a
                             60 deg thrust cone the chainstay bears on
    drive pulley     X+/X-   45T on the XC430 horn: 4 pins, 4 M2 x 6
    case side        X+/X-   plate on one servo's horn face and the other's
                             back; skirt, spacers, 8 M2.5; the chainstay
                             channel at -Y (the fixture tab at +Y only
                             with the fixture)
    chainstay        X+/X-   axle boss, hex pocket (M5 head / nut), bearing
                             stub, a riser inside the belt loop, the tongue;
                             the user's chamfers, gusset and fillets
    fixture mock     Y+      PLACEHOLDER between the tabs, 2 x 6-32 a side.
                             OFF in the module since 2026-10-11 (the dialog's
                             "Fixture mock" box); cad_bike (v1) always builds
                             it, its chassis grows from the block

The axle stack is the user's hand drawing (50.5 between the M5's head and
nut, which preload chainstay -> pulley cone -> wheel hub). The belt plane is
set from the servo side (horn face + pad + flange gap) and the spline pulley
is checked to land its teeth in the same band.

    python -m aow_sim.cad_drive                  # write docs/cad/drive.fs
    python -m aow_sim.cad_drive --check          # ONE call
    python -m aow_sim.cad_drive --push drive_features
    python -m aow_sim.cad_drive --shot           # -> docs/cad/drive.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml

from . import cad_ahrs_fixture as af
from . import cad_servo_mount as sm
from .params import _normalize, load_params

DRIVE_PARAMS = "config/drive_cad.yaml"
AHRS_PARAMS = "config/ahrs_fixture_cad.yaml"     # the TM151's board and holes
OUT_FS = "docs/cad/drive.fs"
OUT_PNG = "docs/cad/drive{}.png"
SPLIT_MARK = af.SPLIT_MARK
PARTS = ("spline pulley L", "spline pulley R", "drive pulley L", "drive pulley R",
         "case side L", "case side R", "chainstay L", "chainstay R")
FIXTURE_PART = "fixture mock"     # only with opt.fixture; --check builds without
HARDWARE = ("XC430 A", "XC430 B", "XC430 horn A", "XC430 horn B", "belt L", "belt R",
            "rear wheel", "M5 axle")
# The belts are drawn as plain rings round the pitch line, so they run
# through the pulleys' teeth; the axle is on the wheel's own bore; the
# chainstays' cones are drawn into the spline pulleys' by the preload.
INTENDED = ({"belt L", "drive pulley L"}, {"belt L", "spline pulley L"},
            {"belt R", "drive pulley R"}, {"belt R", "spline pulley R"},
            {"M5 axle", "rear wheel"},
            {"chainstay L", "spline pulley L"}, {"chainstay R", "spline pulley R"})
PAIRS = (("spline pulley L", "spline pulley R"), ("drive pulley L", "drive pulley R"),
         ("case side L", "case side R"), ("chainstay L", "chainstay R"))


def load(drive_path: str = DRIVE_PARAMS, cad_path: str = sm.CAD_PARAMS,
         mount_path: str = sm.MOUNT_PARAMS, snug: bool = True) -> dict:
    """Everything the module reads, in MILLIMETRES. `snug`: the tighter nut
    slots (False for the frozen v1 bike, printed with the as-built ones)."""
    raw = _normalize(yaml.safe_load(Path(drive_path).read_text()))
    params = load_params(cad_path)
    mounts = sm.load_mounts(mount_path)
    mm = lambda v: [mm(x) for x in v] if isinstance(v, list) else v * 1000.0    # noqa: E731
    s = {sec: {k: (v if k.endswith(("_deg", "_n")) else mm(v)) for k, v in vals.items()}
         for sec, vals in raw.items()}
    tm = _normalize(yaml.safe_load(Path(AHRS_PARAMS).read_text()))["tm151"]
    tm151 = {k: mm(v) for k, v in tm.items() if isinstance(v, (int, float))}
    sv = params["servos"]["xc430_w150"]
    d, w, h = sv["box_size"]
    xc430 = {"caseDepth": mm(d), "caseWidth": mm(w), "caseHeight": mm(h),
             "shaftFromEnd": mm(sv["shaft_from_end"]),
             "hornThickness": mm(sv["horn_thickness"]), "hornDiameter": mm(sv["horn_diameter"]),
             "bossProjection": mm(sv["boss_projection"]), "bossDiameter": mm(sv["boss_diameter"]),
             "holeW": mm(sv["case_hole_pattern"][0]), "holeH": mm(sv["case_hole_pattern"][1])}
    dt = params["drivetrain"]
    b = dt["belt"]
    belt = {"pitch": mm(b["pitch"]), "width": mm(b["width"]), "length": mm(b["length"]),
            "thickness": mm(b["thickness"]),
            "teethServo": int(b["teeth_servo"]), "teethInput": int(b["teeth_input"])}
    ow = params["omni_wheel"]
    wheel = {"R": mm(ow["outer_radius"]), "rm": mm(ow["axle_mount_radius"]),
             "nAxles": int(ow["n_axles"]), "bigD": mm(ow["roller"]["big_diameter"]),
             "smallD": mm(ow["roller"]["small_diameter"]), "len": mm(ow["roller"]["length"]),
             "gap": mm(ow["roller"]["pair_gap"])}
    return {"s": s, "xc430": xc430, "belt": belt, "wheel": wheel, "tm151": tm151, "snug": snug,
            "servo_gap": mm(dt["drive_servo_gap"]),
            "tilt_deg": dt["drive_servo_angle_deg"],
            "table": sm.servo_table(params, mounts), "screw": sm.screw_table(mounts, snug=snug)}


# ---------------------------------------------------------------- 2D geometry
def _u(v):
    n = math.hypot(*v)
    return (v[0] / n, v[1] / n)


def _add(a, b, k=1.0):
    return (a[0] + k * b[0], a[1] + k * b[1])


def _circles(c0, r0, c1, r1):
    """The two intersections of two circles."""
    dx, dy = c1[0] - c0[0], c1[1] - c0[1]
    d = math.hypot(dx, dy)
    if d > r0 + r1 or d < abs(r0 - r1):
        raise ValueError("circles do not meet")
    a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
    h = math.sqrt(max(r0 ** 2 - a ** 2, 0.0))
    m = (c0[0] + a * dx / d, c0[1] + a * dy / d)
    return ((m[0] + h * dy / d, m[1] - h * dx / d), (m[0] - h * dy / d, m[1] + h * dx / d))


def _rot(p, a):
    c, s = math.cos(a), math.sin(a)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1])


def _mid(c, r, p0, p1):
    """The midpoint of the SHORTER arc p0 -> p1 on the circle (c, r)."""
    m = _u(_add(_u(_add(p0, c, -1)), _u(_add(p1, c, -1))))
    return _add(c, m, r)


def _rnd(p):
    return (round(p[0], 6), round(p[1], 6))


def htd_groove(n: int, pitch: float, pld: float, g: dict) -> dict:
    """One HTD groove, centred on +y, pulley centre at the origin: the key
    points of its root arc, flank arcs and tip fillets (the hand drawing's
    generator: flank centres crossed over the centreline)."""
    rt = n * pitch / (2 * math.pi) - pld
    rr = rt - g["depth"]
    rc = rt - g["flank_center"]
    c = g["flank_offset"]
    cp = (-c, math.sqrt(rc ** 2 - c ** 2))              # centre of the +x flank
    p1 = max(_circles((0, 0), rr, cp, g["flank_r"]), key=lambda p: p[0])
    f = max(_circles((0, 0), rt - g["tip_fillet"], cp, g["flank_r"] + g["tip_fillet"]),
            key=lambda p: p[0])
    t1 = _add(cp, _u(_add(f, cp, -1)), g["flank_r"])
    t2 = (f[0] * rt / math.hypot(*f), f[1] * rt / math.hypot(*f))
    return {"rt": rt, "rr": rr, "cp": cp, "p1": p1, "f": f, "t1": t1, "t2": t2}


def htd_outline(n: int, pitch: float, pld: float, g: dict) -> list:
    """The whole pulley's outline as ("L"|"A", points...) segments, CCW, mm."""
    q = htd_groove(n, pitch, pld, g)
    rt, rr = q["rt"], q["rr"]
    mir = lambda p: (-p[0], p[1])     # noqa: E731
    # CCW through one groove: t2 -> t1 (fillet) -> p1 (flank) -> p1' (root)
    # -> t1' (flank) -> t2' (fillet)
    local = [("A", q["t2"], _mid(q["f"], g["tip_fillet"], q["t2"], q["t1"]), q["t1"]),
             ("A", q["t1"], _mid(q["cp"], g["flank_r"], q["t1"], q["p1"]), q["p1"]),
             ("A", q["p1"], (0.0, rr), mir(q["p1"])),
             ("A", mir(q["p1"]), mir(_mid(q["cp"], g["flank_r"], q["t1"], q["p1"])), mir(q["t1"])),
             ("A", mir(q["t1"]), mir(_mid(q["f"], g["tip_fillet"], q["t2"], q["t1"])), mir(q["t2"]))]
    segs = []
    step = 2 * math.pi / n
    half_land = step / 2 - math.atan2(q["t2"][0], q["t2"][1])
    for k in range(n):
        a = k * step
        for sg in local:
            segs.append((sg[0],) + tuple(_rnd(_rot(p, a)) for p in sg[1:]))
        # tip land to the next groove's t2
        e = _rnd(_rot(mir(q["t2"]), a))
        nx = _rnd(_rot(q["t2"], a + step))
        mid = _rot((0.0, rt), a + step / 2)
        segs.append(("A", e, _rnd(mid), nx))
    return segs, half_land * rt


def spline_outline(ro: float, ri: float, lobe_deg: float, n: int) -> list:
    """The male spline: n lobes (arcs on ro, lobe_deg wide), radial flanks,
    root arcs on ri. CCW."""
    h = math.radians(lobe_deg) / 2
    step = 2 * math.pi / n
    pol = lambda r, a: _rnd((r * math.cos(a), r * math.sin(a)))   # noqa: E731
    segs = []
    for k in range(n):
        a = math.pi / 2 + k * step
        segs.append(("A", pol(ro, a - h), pol(ro, a), pol(ro, a + h)))
        segs.append(("L", pol(ro, a + h), pol(ri, a + h)))
        segs.append(("A", pol(ri, a + h), pol(ri, a + step / 2), pol(ri, a + step - h)))
        segs.append(("L", pol(ri, a + step - h), pol(ro, a + step - h)))
    return segs


def tangents(c1, r1, c2, r2):
    """External tangents of two circles: (A+, B+, m+), (A-, B-, m-), with m the
    outward normal each side."""
    d = (c2[0] - c1[0], c2[1] - c1[1])
    D = math.hypot(*d)
    u = (d[0] / D, d[1] / D)
    n = (-u[1], u[0])
    ca = (r1 - r2) / D
    sa = math.sqrt(1 - ca ** 2)
    out = []
    for sg in (1, -1):
        m = (u[0] * ca + sg * n[0] * sa, u[1] * ca + sg * n[1] * sa)
        out.append((_add(c1, m, r1), _add(c2, m, r2), m))
    return out, u


def hull_loop(c1, r1, c2, r2) -> list:
    """The outline of the convex hull of two circles (a belt's side)."""
    (ap, bp, _), (am, bm, _) = tangents(c1, r1, c2, r2)[0]
    u = tangents(c1, r1, c2, r2)[1]
    return [("L", _rnd(ap), _rnd(bp)),
            ("A", _rnd(bp), _rnd(_add(c2, u, r2)), _rnd(bm)),
            ("L", _rnd(bm), _rnd(am)),
            ("A", _rnd(am), _rnd(_add(c1, u, -r1)), _rnd(ap))]


def inside_hull(p, c1, r1, c2, r2) -> float:
    """How far p is inside both straight runs of the hull (negative = out)."""
    (t_p, t_m), _ = tangents(c1, r1, c2, r2)
    return min(-((p[0] - c1[0]) * m[0] + (p[1] - c1[1]) * m[1] - r1) for _, _, m in (t_p, t_m))


# --------------------------------------------------------------------- layout
def centre_distance(bt: dict) -> float:
    d1 = bt["teethInput"] * bt["pitch"] / math.pi
    d2 = bt["teethServo"] * bt["pitch"] / math.pi
    b = bt["length"] - math.pi / 2 * (d1 + d2)
    a = (d2 - d1) ** 2 / 4
    return (b + math.sqrt(b * b - 8 * a)) / 4


def layout(data: dict) -> dict:
    """Every derived dimension, module frame, mm, X as MAGNITUDES on the left
    side (the FeatureScript negates). Raises if something does not fit."""
    s, sv, bt, wh, sc = data["s"], data["xc430"], data["belt"], data["wheel"], data["screw"]
    pu, dp, cs, ch, tn, fx = (s[k] for k in ("pulley", "drive_pulley", "case", "chainstay",
                                             "tension", "fixture"))
    wall = s["print"]["min_wall"]
    bl = s["joint"]["bridge_layer"]

    def need(ok: bool, what: str) -> None:
        if not ok:
            raise ValueError(what)

    def snap(h):
        """Up to the next layer: a ceiling's height from its part's bed."""
        return math.ceil(h / bl - 1e-6) * bl

    L: dict = {}
    # ---- belt and the servo station
    L["C"] = C = centre_distance(bt)
    L["zs"] = zs = (sv["caseWidth"] + data["servo_gap"]) / 2
    L["Ys"] = Ys = math.sqrt(C ** 2 - zs ** 2)
    L["rP15"] = bt["teethInput"] * bt["pitch"] / (2 * math.pi)
    L["rP45"] = bt["teethServo"] * bt["pitch"] / (2 * math.pi)
    # ---- the servo, the case side and the drive pulley along X
    L["caseHalf"] = sv["caseDepth"] / 2
    L["xHorn"] = L["caseHalf"] + sv["hornThickness"]
    L["xPo"] = L["caseHalf"] + cs["plate_thickness"]            # case side's outer face
    L["padH"] = L["xPo"] - L["xHorn"]
    need(L["padH"] > 0.5, "the case plate is not thicker than the horn: no pad")
    L["flangeIn"] = L["xPo"] + dp["flange_gap"]
    L["teethIn"] = L["flangeIn"] + pu["flange_cyl"] + pu["flange_cone_len"]
    L["teethOut"] = L["teethIn"] + pu["tooth_width"]
    L["outerFace"] = L["teethOut"] + pu["flange_cone_len"] + pu["flange_cyl"]
    L["beltX0"] = (L["teethIn"] + L["teethOut"] - bt["width"]) / 2
    L["beltX1"] = L["beltX0"] + bt["width"]
    rise = pu["flange_cone_len"] * math.tan(math.radians(pu["flange_cone_deg"]))
    for n, key in ((bt["teethInput"], "15"), (bt["teethServo"], "45")):
        g = s[f"teeth_{key}"]
        q = htd_groove(n, bt["pitch"], pu["pld"], g)
        L[f"rTip{key}"], L[f"rRoot{key}"] = q["rt"], q["rr"]
        L[f"rLip{key}"] = q["rt"] + pu["flange_lip"]
        L[f"rFl{key}"] = L[f"rLip{key}"] + rise
        _, land = htd_outline(n, bt["pitch"], pu["pld"], g)
        L[f"land{key}"] = land
        need(land > 0.05, f"the {key}T grooves leave no tip land")
    # ---- the spline pulley: shoulder on the wheel's hub face, teeth in the
    # band the servo side set
    wl = s["wheel"]
    L["hubFace"] = wl["hub_face_x"]
    L["hubR"] = pu["hub_dia"] / 2
    L["xCh"] = L["flangeIn"] - (L["rFl15"] - L["hubR"]) / math.tan(math.radians(pu["hub_chamfer_deg"]))
    need(L["xCh"] >= L["hubFace"], "the spline pulley's hub chamfer runs past its shoulder: "
         "the belt plane is too close to the wheel")
    L["splineLen"] = s["spline"]["length"]
    L["bore15R"] = pu["bore_dia"] / 2
    L["cbR"] = pu["counterbore_dia"] / 2
    L["xCb"] = L["teethIn"] - pu["counterbore_floor_inset"]
    tc = math.tan(math.radians(pu["cone_deg"]))
    L["xCe"] = L["xCb"] - (L["cbR"] - L["bore15R"]) / tc
    need(L["xCe"] > L["hubFace"] - L["splineLen"], "the thrust cone runs into the spline stub")
    need(L["rRoot15"] - L["cbR"] >= wall, "the counterbore is too close to the 15T's roots")
    need(s["spline"]["root_dia"] / 2 - L["bore15R"] >= wall, "the spline stub is too thin round the bore")
    # ---- the chainstay's bearing stub, from the pulley's cone and the gap
    L["xCi"] = L["outerFace"] + ch["pulley_gap"]
    L["xCo"] = L["xCi"] + ch["thickness"]
    L["stubR"] = ch["stub_dia"] / 2
    need(L["stubR"] < L["cbR"], "the chainstay's stub is wider than the pulley's counterbore")
    L["rC0"] = L["stubR"] - math.tan(math.radians(ch["relief_cone_deg"])) * ch["relief_len"]
    L["rC1"] = L["rC0"] - math.tan(math.radians(ch["bearing_cone_deg"])) * ch["bearing_len"]
    # the stub's 60 deg nose sits cone_preload INTO the pulley's seat, as
    # drawn: the M5 seats it by flexing the arm
    x_c0 = L["xCb"] - (L["cbR"] - L["rC0"]) / tc - ch["cone_preload"]
    L["xRel"] = x_c0
    L["xA"] = x_c0 + ch["relief_len"]
    L["xE"] = x_c0 - ch["bearing_len"]
    L["csBoreR"] = ch["bore_dia"] / 2
    L["endRing"] = L["rC1"] - L["csBoreR"]
    need(L["endRing"] >= 0.5 * wall, "the stub's end face ring is under half a wall")
    need(L["xE"] > L["xCe"], "the chainstay's stub reaches the pulley's bore")
    # ---- the M5: clamp faces (hex pocket floors) at +-clamp_span/2
    ax = s["axle"]
    # the hex pocket's floor is a ceiling as the chainstay prints (outer face
    # down): on the layer grid, i.e. up to bl/2 deeper than clamp_span says
    L["clampX"] = L["xCo"] - snap(L["xCo"] - ax["clamp_span"] / 2)
    need(L["xA"] < L["clampX"] < L["xCo"] - 2 * wall, "the hex pocket's floor misses the chainstay")
    L["hexR"] = ax["hex_af"] / math.sqrt(3)
    need(L["stubR"] - L["hexR"] >= 2 * wall, "the stub is too thin round the hex pocket")
    # ---- the servos' envelope in (Y, Z)
    L["sfe"] = sv["shaftFromEnd"]
    L["Yb"] = Ys - (sv["caseHeight"] - L["sfe"])
    L["Yf"] = Ys + L["sfe"]
    L["Yhc"] = Ys - (sv["caseHeight"] / 2 - L["sfe"])             # the case face's centre
    c, w = cs["skirt_clearance"], cs["skirt_wall"]
    L["Zi"] = sv["caseWidth"] + c
    L["Zp"] = L["Zi"] + w
    L["skirtTop"] = L["caseHalf"] - cs["skirt_depth"]
    L["Ysk0"] = L["Yb"] - c - w
    L["Ytab0"] = L["Yf"] + c + w
    L["Yfront"] = L["Ytab0"] + fx["tab_length"]
    # ---- chainstay riser and tongue: behind the 45T flange, inside the belt
    L["Yc0"] = wh["R"] + cs["tire_clearance"]
    ah = ch["arm_width"] / 2
    dz = zs - ah                                          # the riser's corner nearest servo A
    rfl = L["rFl45"] + ch["flange_clearance"]
    L["Yj1"] = Ys - math.sqrt(rfl ** 2 - dz ** 2)
    L["Yr0"] = L["Yj1"] - ch["riser_length"]
    need(L["Yr0"] > L["rFl15"] + ch["flange_clearance"], "the riser reaches the 15T's flange")
    back = bt["thickness"] - pu["pld"]
    r1 = L["rP15"] - back - ch["belt_clearance"]
    r2 = L["rP45"] - back - ch["belt_clearance"]
    L["riserBelt"] = min(inside_hull((y, z), (0.0, 0.0), r1, (Ys, -zs), r2)
                         for y in (L["Yr0"], L["Yj1"]) for z in (ah, -ah))
    need(L["riserBelt"] >= 0, f"the riser is {-L['riserBelt']:.2f} mm into the belt")
    # ---- the tension joint, along X. The riser's flat clamps on the case
    # side's outer face; the tongue (a key) stops short of the channel floor
    L["riserTop"] = L["xPo"]
    L["tongueTop"] = L["xPo"] - tn["tongue_depth"]              # the channel's floor
    L["tongueEnd"] = L["tongueTop"] + tn["tongue_top_gap"]      # the tongue's top
    L["tongueHalf"] = tn["tongue_width"] / 2
    L["tongueClr"] = tn["tongue_clearance"]
    L["holeR"], L["headR"] = sc["holeDia"] / 2, sc["headDia"] / 2
    need(L["tongueHalf"] - L["holeR"] >= 1.5 * wall, "the tongue is too narrow round the screw")
    L["cskDepth"] = (L["headR"] - L["holeR"]) / math.tan(math.radians(sc["cskAngle"] / 2))
    L["headBoreR"] = (sc["headDia"] + tn["head_bore_clearance"]) / 2
    # the head's bore ends in a ceiling (the ring round the seat's top) on the
    # chainstay's layer grid; csk_web is a minimum, the head goes out to it
    ring = L["headBoreR"] - L["headR"]
    h_cb = L["xCo"] - (L["xPo"] + tn["csk_web"] + L["cskDepth"] + ring)
    h_cb = math.floor(h_cb / bl + 1e-6) * bl
    L["xH"] = L["xCo"] - h_cb - ring                              # head face (magnitude)
    L["cskWeb"] = L["xH"] - L["cskDepth"] - L["xPo"]
    L["tip"] = L["xH"] - tn["screw_length"]
    # Heights from the bed (the case side prints on its outer face), every
    # ceiling on the layer grid. The nut bears on the web when tight, so it
    # sits on the nut slot's floor; the tip has to clear the nut and stay in
    # the tip slot.
    need(abs(snap(tn["tongue_depth"]) - tn["tongue_depth"]) < 1e-6,
         "tongue_depth is off the layer grid: the channel's ceiling would be")
    L["nutSlotT"] = sc["nutSlotThickness"]
    hN0 = snap(tn["tongue_depth"] + tn["min_web"])               # nut slot floor
    hN1 = snap(hN0 + L["nutSlotT"])                               # its ceiling
    hT = snap(hN1 + tn["tip_slot"])                               # the tip slot's ceiling
    h_tip = L["xPo"] - L["tip"]
    L["hN0"], L["hN1"], L["hT"], L["hTip"] = hN0, hN1, hT, h_tip
    L["tipPast"] = h_tip - (hN0 + tn["nut_thickness"])
    need(L["tipPast"] >= tn["tip_past_nut"], f"the screw's tip is only {L['tipPast']:.2f} past the nut")
    need(h_tip <= hT - bl, "the screw's tip reaches the tip slot's ceiling")
    L["xN0"] = L["xPo"] - hN0                                      # nut slot's outboard face (magnitude)
    L["nutSlotT"] = hN1 - hN0
    L["web"] = hN0 - tn["tongue_depth"]
    L["tipSlot"] = hT - hN1
    L["xRin"] = L["xPo"] - snap(hT + tn["nut_roof"])
    L["travel"] = tn["travel"]
    L["nutHalfY"] = sc["nutSlotWidth"] / 2
    # the screw's station along Y: its head bore inside the tongue, its slot
    # (with the travel) closed in the web; the nut slot runs out the back
    L["nutCornerY"] = L["nutHalfY"] * 2 / math.sqrt(3)
    lo = max(L["Yr0"] + L["headBoreR"] + wall, L["Yc0"] + L["travel"] + L["holeR"] + wall)
    hi = min(L["Yj1"] - L["headBoreR"] - wall, L["Yb"] - L["nutCornerY"] - wall)
    need(lo <= hi, f"no room for the tension screw along Y ({lo:.2f} > {hi:.2f})")
    L["Yscr"] = (lo + hi) / 2
    # ---- the fixture tab's screws: heads clear of the drive pulley's flange
    L["xTab"] = L["xPo"] - s["joint"]["joint_plate"]
    L["yF0"] = L["Ytab0"] + fx["screw_y0"]
    L["yF1"] = L["Ytab0"] + fx["screw_y1"]
    L["zF"] = fx["screw_z"]
    L["ridgeHalf"] = (fx["screw_y1"] - fx["screw_y0"]) / 2
    need(fx["screw_y0"] >= L["ridgeHalf"] and fx["tab_length"] - fx["screw_y1"] >= L["ridgeHalf"],
         "the fixture ridge runs off the tab")
    L["fixHead"] = min(math.hypot(y - Ys, L["zF"] + zs) for y in (L["yF0"], L["yF1"])) \
        - L["rFl45"] - L["headR"]
    need(L["fixHead"] >= fx["head_clearance"], "a fixture screw's head is behind the drive pulley")
    need(L["Zp"] - L["zF"] - L["headR"] >= 2 * wall, "a fixture screw's head runs off the plate")
    # ---- the chainstay's finishing (the user's hand edits, 2026-10-11): the
    # front chamfer and its fillets stay off the riser's top, the riser's
    # chamfer and its fillet off the tongue, the gusset on the bed
    L["cyFront"], L["cyRiserCh"] = ch["front_chamfer"], ch["riser_chamfer"]
    L["cyFillet"], L["cyEdgeCh"] = ch["chamfer_fillet"], ch["edge_chamfer"]
    run = L["cyFillet"] * math.tan(math.radians(22.5))     # a fillet's tangent run on a 135 deg edge
    need(L["xCo"] - L["cyFront"] - run >= L["riserTop"] + L["cyEdgeCh"],
         "the chainstay's front chamfer reaches the riser's top")
    need(L["Yr0"] + L["cyRiserCh"] + run <= L["Yc0"], "the riser's chamfer runs into the tongue")
    need(L["xCi"] - L["cyRiserCh"] - run >= L["riserTop"], "the gusset runs past the riser's top")
    # ---- the case plate's horn relief and wire gap
    L["reliefR"] = cs["relief_dia"] / 2
    need(L["Yf"] + c - Ys > L["reliefR"], "the horn relief breaks the skirt")
    L["wireY0"] = Ys - cs["wire_gap_from_shaft"] - cs["wire_gap_width"] / 2
    L["wireY1"] = L["wireY0"] + cs["wire_gap_width"]
    L["m25Seat"] = L["xPo"] - cs["screw_cb_depth"]
    L["spacerTop"] = L["caseHalf"] - cs["spacer_height"]
    # the rear block's +-Z corners at 45 deg (user, 2026-10-07): only the
    # chainstay's channel and the tension screw are in it, at mid-height
    L["rcLeg"] = cs["rear_chamfer"]
    if L["rcLeg"] > 0:
        need(L["Yc0"] + L["rcLeg"] <= L["Yb"] - c + 1e-6, "the case side's rear chamfer reaches the servo pocket")
        need(L["Zp"] - L["rcLeg"] >= L["tongueHalf"] + L["tongueClr"] + 2 * wall,
             "the case side's rear chamfer reaches the chainstay's channel")
        need(L["Zp"] - L["rcLeg"] >= L["nutHalfY"] + 2 * wall, "the case side's rear chamfer reaches the tension nut")
    return L


def to_drive(p) -> tuple[float, float]:
    """A bike-frame side-view point (y, z) in the module frame (the drive is
    turned up 45 deg about X; DRIVE_TILT is checked against the config)."""
    a = math.radians(DRIVE_TILT)
    return (p[0] * math.cos(a) + p[1] * math.sin(a), -p[0] * math.sin(a) + p[1] * math.cos(a))


DRIVE_TILT = 45.0


def _inside(p, poly) -> bool:
    n, ok = len(poly), False
    for i in range(n):
        (y0, z0), (y1, z1) = poly[i], poly[(i + 1) % n]
        if (z0 > p[1]) != (z1 > p[1]) and p[0] < y0 + (p[1] - z0) * (y1 - y0) / (z1 - z0):
            ok = not ok
    return ok


def _edge_dist(p, poly) -> float:
    best = math.inf
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        d = (b[0] - a[0], b[1] - a[1])
        u = max(0.0, min(1.0, ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (d[0] ** 2 + d[1] ** 2)))
        best = min(best, math.hypot(p[0] - a[0] - u * d[0], p[1] - a[1] - u * d[1]))
    return best


def _cross_lines(p, d, q, e):
    """Where the line p + t d meets the line q + u e (2D)."""
    den = d[0] * e[1] - d[1] * e[0]
    t = ((q[0] - p[0]) * e[1] - (q[1] - p[1]) * e[0]) / den
    return (p[0] + t * d[0], p[1] + t * d[1])


def rail_outlines(data: dict, L: dict) -> dict:
    """The rails' side view in the BIKE frame: the main outline with the
    front cage post drawn into it, and the rear arm and post -- the front
    post mirrored about mid-wheelbase, the arm's edges continuing its bottom
    and the band's top until they meet the case side's rear chamfer face and
    top edge. Also the posts' screws and slot directions."""
    rl = data["s"]["rail"]
    ay, az = rl["steer_axle"]
    r = math.radians(rl["steer_rake_deg"])
    c, s = math.cos(r), math.sin(r)
    st = lambda ym, zm: (ay + ym * c - zm * s, az + ym * s + zm * c)   # noqa: E731  steering module -> bike
    mir = lambda q: (ay - q[0], q[1])                                   # noqa: E731  about mid-wheelbase
    p0, p1 = rl["cage_post_ym"]
    z0, zb = rl["steer_band_zm"]
    zt, lead = rl["cage_post_top_zm"], rl["cage_post_lead"]
    # the post, free edge (p1) first, the rail's side (p0) 45 deg down to the band's top
    post_m = [(p1, z0), (p1, zt - lead), (p1 - lead, zt), (p0 + (zt - zb), zt), (p0, zb)]
    if p0 + (zt - zb) > p1 - lead:
        raise ValueError("the cage post's top is narrower than its lead-ins")
    post = [st(*q) for q in post_m]
    outline = [tuple(q) for q in rl["outline_bottom"]] + post + [tuple(q) for q in rl["outline_top"]]
    # the rear: the mirrored post; its arm continues the post's bottom
    # (module z z0) and the band's top (zb) forward to the case side
    rpost = [mir(q) for q in post]
    k = math.sqrt(0.5)
    bike = lambda yd, zd: ((yd - zd) * k, (yd + zd) * k)               # noqa: E731  drive -> bike
    ch0 = bike(L["Yc0"], L["Zp"] - L["rcLeg"])                          # the rear chamfer, bottom
    ch1 = bike(L["Yc0"] + L["rcLeg"], L["Zp"])                          # its top = the top edge's rear end
    top1 = bike(L["Ytab0"], L["Zp"])
    up = (0.0, 1.0)
    d_bot = (rpost[0][0] - mir(st(p0, z0))[0], rpost[0][1] - mir(st(p0, z0))[1])   # along the post's bottom
    d_top = (rpost[-1][0] - mir(st(p0 - 10, zb))[0], rpost[-1][1] - mir(st(p0 - 10, zb))[1])
    B = _cross_lines(rpost[0], d_bot, ch0, (ch1[0] - ch0[0], ch1[1] - ch0[1]))
    T = _cross_lines(rpost[-1], d_top, ch1, (top1[0] - ch1[0], top1[1] - ch1[1]))
    if not (ch0[1] < B[1] < ch1[1]):
        raise ValueError(f"the rear arm's bottom misses the case side's rear chamfer (z {B[1]:.2f})")
    if not (ch1[0] < T[0] < top1[0]):
        raise ValueError("the rear arm's top misses the case side's top edge")
    rear = [B] + rpost + [T, ch1]
    scr_f = st((p0 + p1) / 2, rl["cage_screw_zm"])
    posts = [{"screw": mir(scr_f), "axis": 90.0 - rl["steer_rake_deg"], "free": (-c, s)},
             {"screw": scr_f, "axis": 90.0 + rl["steer_rake_deg"], "free": (c, s)}]
    return {"outline": outline, "rear": rear, "posts": posts, "postHalf": (p1 - p0) / 2,
            "screwAbove": zt - rl["cage_screw_zm"], "screwBelow": rl["cage_screw_zm"] - z0, "lead": lead}


# the rail joints on the LEFT rail only (added after the mirror): the power
# holder's two and the Pi carrier's top one (user, 2026-10-16 night)
LEFT_JOINTS = ("carrier", "carrier_front", "pi_top")


def rail_layout(data: dict, L: dict) -> dict:
    """The side rails and the AHRS tabs, module frame, mm: the outline pieces
    as (y, z) on the left rail's plane, the joints, the tabs. Raises if
    something does not fit."""
    s, sc = data["s"], data["screw"]
    rl, ah, jn, tm = s["rail"], s["ahrs"], s["joint"], data["tm151"]
    wall = s["print"]["min_wall"]

    def need(ok: bool, what: str) -> None:
        if not ok:
            raise ValueError(what)

    need(abs(data["tilt_deg"] - DRIVE_TILT) < 1e-9, "the drive's tilt is not the 45 deg the rail's outline is drawn for")
    R: dict = {"innerX": rl["inner_x"]}
    R["plate"] = L["xPo"] - rl["inner_x"]
    need(abs(R["plate"] - jn["joint_plate"]) < 1e-6, f"the rail is {R['plate']:.2f} thick, not the joint plate")
    need(rl["inner_x"] < L["caseHalf"], "the rail's inner face is outside the case side's")
    RO = rail_outlines(data, L)
    R["outlineBike"] = [list(q) for q in RO["outline"]]
    R["rearBike"] = [list(q) for q in RO["rear"]]
    out = [to_drive(q) for q in RO["outline"]]
    rear = [to_drive(q) for q in RO["rear"]]
    R["pieces"] = [[list(q) for q in out], [list(q) for q in rear]]
    # the power holder's two ears are on the LEFT rail only (user,
    # 2026-10-16 night: its attachments "should only be on LHS case (the
    # features are mirrored to RHS unnecessarily)"); the FeatureScript adds
    # them, and their joints, after the mirror
    R["leftPieces"] = []
    # the front ear (the holder's fore joint): off the case side's front
    # edge at its top, over the AHRS tab; it ends ON the front edge
    ear = []
    if "front_ear" in rl:
        el, ez0, ez1 = rl["front_ear"]
        ear = [(L["Ytab0"], ez0), (L["Ytab0"] + el, ez0), (L["Ytab0"] + el, ez1), (L["Ytab0"], ez1)]
        R["leftPieces"].append([list(q) for q in ear])
    # the back ear (the holder's aft joint): up off the case side's top
    # edge in the corner the rear arm makes with it, mostly inside the arm,
    # and ahead of nothing but the strap's run; it stands ON the top edge
    bear = []
    if "back_ear" in rl:
        by0, by1, bz1 = rl["back_ear"]
        need((L["Yc0"] + L["rcLeg"] <= by0 or _inside((by0, L["Zp"]), rear)) and L["Yc0"] + L["rcLeg"] < by1 <= L["Ytab0"],
             "the back ear runs off the case side's top edge and the rear arm")
        bear = [(by0, L["Zp"]), (by1, L["Zp"]), (by1, bz1), (by0, bz1)]
        R["leftPieces"].append([list(q) for q in bear])
    # the battery stops, on both arms (user, 2026-10-16 night: the arms to
    # the rear roll cage "should have some hints (like 2mm lateral, going
    # slightly behind the battery) so the battery couldn't slide backwards
    # into the rear wheel"): a lip off each arm's inner face, in past the
    # battery's side, its front face just behind the battery's rear face.
    # Wholly on the arm, and off the cage post (the cage's socket goes over
    # the post). cad_bike_v2 (cad_power) refuses it off the battery.
    R["stops"] = []
    R["postRear"] = [list(q) for q in rear[1:-2]]
    if "battery_stop" in rl:
        # behind the battery (`back`), and a short leg forward along its side
        # (`side`, user: "a similar hint so the battery is also constrained
        # laterally a bit"), its inner face side_x, leg long past the back one
        sy1, st_, sz0, sz1, sxi, ssx, sleg = rl["battery_stop"]
        need(sxi < ssx < rl["inner_x"], "the battery stop does not reach in off the rail's inner face")
        for kind, x0, ya, yb in (("back", sxi, sy1 - st_, sy1), ("side", ssx, sy1 - st_, sy1 + sleg)):
            for q in ((ya, sz0), (yb, sz0), (yb, sz1), (ya, sz1)):
                need(_inside(q, rear) and _edge_dist(q, rear) >= 0.5 - 1e-6, "the battery stop runs off the rear arm")
                need(not _inside(q, rear[1:-2]) and _edge_dist(q, rear[1:-2]) >= 1.0 - 1e-6,
                     "the battery stop runs into the rear cage post")
            R["stops"].append({"kind": kind, "x": [x0, rl["inner_x"] + 0.5], "y": [ya, yb], "z": [sz0, sz1],
                               "face": sy1, "t": st_})
    # the case side's outline (its rear corners chamfered): inside it the
    # rail stays the plate's thickness, the servos being there
    rc = L["rcLeg"]
    R["casePoly"] = [[L["Yc0"], -(L["Zp"] - rc)], [L["Yc0"] + rc, -L["Zp"]], [L["Ytab0"], -L["Zp"]],
                     [L["Ytab0"], L["Zp"]], [L["Yc0"] + rc, L["Zp"]], [L["Yc0"], L["Zp"] - rc]]
    # each piece keeps a wall off the case side's M2.5 counterbores (the
    # union would fill them). The outline overlaps the case; the rear arm
    # ends ON its chamfer face and top edge, so it unites across them
    cbs = [(L["Yhc"] + sy * L["holeHH"], zc + sz * L["holeHW"])
           for zc in (-L["zs"], L["zs"]) for sy in (1, -1) for sz in (1, -1)]
    for nm, pc in (("outline", out), ("rear arm", rear), ("front ear", ear), ("back ear", bear)):
        if not pc:
            continue
        for c in cbs:
            need(not _inside(c, pc) and _edge_dist(c, pc) >= L["m25CbR"] + wall - 1e-3,
                 f"the rail's {nm} comes over an M2.5 counterbore at ({c[0]:.1f}, {c[1]:.1f})")
    need(any(L["Yc0"] < y < L["Ytab0"] and abs(z) < L["Zp"] for y, z in out),
         "the rail's outline does not reach into the case side")
    # the joints: the head's countersink and the groove inside the outline,
    # a wall to spare
    R["joints"] = []
    hr = sc["headDia"] / 2
    gw = jn["ridge_flat"] / 2 + jn["ridge_height"] + jn["ridge_clearance"] * math.sqrt(2)
    case_poly = [tuple(q) for q in R["casePoly"]]
    R["leftJoints"] = []
    for tag in ("steer", "bridge", "carrier", "carrier_front", "pi_low", "pi_top"):
        if f"{tag}_joint" not in rl:
            continue
        h = to_drive(rl[f"{tag}_joint"])
        # the pieces it is in (they unite: the back ear overlaps the rear arm)
        allp = [q for q in (out, rear, ear, bear) if q]
        pcs = [q for q in allp if _inside(h, q)]
        need(bool(pcs), f"the rail's {tag} joint is not on the rail")
        a = math.radians(rl[f"{tag}_ridge_deg"] - DRIVE_TILT)
        d = (math.cos(a), math.sin(a))
        half = rl[f"{tag}_ridge_half"]
        n = (-d[1], d[0])
        # the groove's extent along d from the head; open toward -d, it runs
        # out through the rail's edge (`*_groove_open` past the head), so the
        # other part's ridge slides into it -- up from below for the steering
        # and the bridge, down from above for the carrier (d down)
        hi = half + 1.0
        lo = -rl[f"{tag}_groove_open"]
        pts = [h] + [(h[0] + u * d[0] + v * n[0], h[1] + u * d[1] + v * n[1]) for u in (0.0, hi) for v in (-gw, gw)]
        for q in pts:
            need(any(_inside(q, c) for c in allp), f"the rail's {tag} joint runs off its outline")
        # the groove must run out through the rail's edge, its 45 deg end taper
        # (ridge_height + clearance long) wholly outside the rail
        # -- across its whole width, and clear of every piece of the rail
        # (the ears unite with the arm)
        # (nine lines across it: three let a corner of the ear between them,
        # and the Pi carrier's ridge hit it lifting out, 2026-10-16)
        tl = jn["ridge_height"] + jn["ridge_clearance"] + 0.5
        for v in [gw * (i / 4 - 1) for i in range(9)]:
            e = (h[0] + (lo + tl) * d[0] + v * n[0], h[1] + (lo + tl) * d[1] + v * n[1])
            need(not any(_inside(e, q) for q in allp),
                 f"the rail's {tag} groove's end taper is inside the rail: open it further (rail.{tag}_groove_open)")
        # a wall round the countersink, off the pieces' edge (at least the
        # farthest edge of one piece holding it) and off the case side's
        # outline (inside it the rail is only the plate's 3.6)
        room = min(max(_edge_dist(h, c) for c in pcs), _edge_dist(h, case_poly)) - hr
        need(not _inside(h, case_poly) and room >= wall,
             f"the rail's {tag} joint's countersink is {room:.2f} from its edge")
        # the holder's (carrier*) and the Pi carrier's top one (pi_top) on
        # the left rail only, as their ears; the Pi carrier's low ones
        # (pi_low) on both
        R["leftJoints" if tag in LEFT_JOINTS else "joints"].append(
            {"tag": tag, "head": list(h), "dir": list(d), "half": half, "room": room,
             "gMid": (lo + hi) / 2, "gHalf": (hi - lo) / 2})
    # a point inside the rail and outside the case side (to find it by)
    yb, zb = to_drive((130.0, 45.0))
    need(_inside((yb, zb), out) and not (L["Yc0"] < yb < L["Ytab0"] and abs(zb) < L["Zp"]),
         "the rail's anchor is not in the rail")
    R["anchor"] = [yb, zb]
    # ---- the AHRS tabs: board back face on the tabs' fronts, the nut slot
    # behind a web, a wall behind it; holes off the TM151's own pattern
    nT = ah["nut_thickness"] + ah["nut_clearance"]
    R["ahY1"] = L["Ytab0"] + ah["back_wall"] + nT + ah["web"]       # the tabs' fronts = the board's back face
    R["ahN0"] = R["ahY1"] - ah["web"] - nT
    R["ahN1"] = R["ahY1"] - ah["web"]
    R["ahTip"] = R["ahY1"] + tm["board_thickness"] - ah["screw_length"]
    # the nut pulled up to the web: the screw's end within 0.2 of its far face
    need(R["ahTip"] - (R["ahN1"] - ah["nut_thickness"]) <= 0.2 + 1e-9, "the AHRS screws do not reach through the nut")
    need(R["ahTip"] - L["Ytab0"] >= 0.3, "the AHRS screws reach the case side's skirt wall")
    R["ahHoleR"] = ah["hole_dia"] / 2
    R["ahHoleY0"] = L["Ytab0"] - 0.6
    need(R["ahHoleY0"] > L["Yf"] + L["skClr"] + 0.3, "the AHRS holes run into the servo pocket")
    hx = tm["hole_span_y"] / 2                                        # across the board: drive X
    zc = ah["board_z"] + ah["tm151_x_up_n"] * tm["hole_offset_x"]       # its +x up (1) or down (-1) the face
    R["ahX"] = hx
    R["ahZ"] = [zc - tm["hole_span_x"] / 2, zc + tm["hole_span_x"] / 2]
    R["ahNutHalf"] = ah["nut_af"] / 2 + ah["nut_clearance"] / 2
    R["ahNutOut"] = hx + ah["nut_af"] / math.sqrt(3) + ah["nut_clearance"]   # the slot's floor, |x|
    R["ahTabHalf"] = R["ahNutHalf"] + ah["tab_wall"]
    R["ahTabIn"] = ah["tab_inner_x"]
    need(R["ahTabIn"] < hx - R["ahHoleR"] - 0.3, "the AHRS tabs stop short of the holes")
    need(L["xPo"] - R["ahNutOut"] >= 2 * wall, "the AHRS nut slots come through the case side's outer face")
    for z in R["ahZ"]:
        need(abs(z) + R["ahTabHalf"] <= L["Zp"], "an AHRS tab runs off the case side")
    bw, bl = tm["board_width"] / 2, tm["board_length"] / 2
    need(bw <= L["caseHalf"] + 1e-6, "the TM151's board is wider than between the case sides")
    R["ahBoard"] = {"x": bw, "y0": R["ahY1"], "y1": R["ahY1"] + tm["board_thickness"],
                    "z0": ah["board_z"] - bl, "z1": ah["board_z"] + bl}
    need(R["ahBoard"]["z0"] >= -L["Zp"] - 1e-6, "the TM151 hangs under the case sides")
    # the board (|x| to 17) over the rail's top edge (from |x| 16): they share
    # x, so the board's lower corners must stay out of the outline
    b = R["ahBoard"]
    corners = [(b["y0"], b["z0"]), (b["y1"], b["z0"])]
    need(not any(_inside(q, out) for q in corners), "the rail runs into the TM151's board")
    R["ahRailGap"] = min(_edge_dist(q, out) for q in corners)
    need(R["ahRailGap"] >= 1.0, f"the rail's top edge is {R['ahRailGap']:.2f} under the TM151's board")
    R["ahScrewPast"] = (R["ahN1"] - ah["nut_thickness"]) - R["ahTip"]   # past the pulled-up nut's far face
    # the front ear stands over the AHRS tabs and the TM151's board (which
    # reaches |x| 17, inside the ear's 16..20.6)
    if ear:
        ez0 = rl["front_ear"][1]
        R["earGap"] = ez0 - max(R["ahZ"][-1] + R["ahTabHalf"], R["ahBoard"]["z1"])
        need(R["earGap"] >= 1.0, f"the rail's front ear is {R['earGap']:.2f} over the AHRS tab / board")
    # ---- the cage posts: each the male feature the cage's socket slides
    # down over (drawn in the outlines above), one 6-32 in from the side, the
    # nut slot from the screw out through the post's free edge
    w, T = sc["nutSlotWidth"], sc["nutSlotThickness"]
    jpl = jn["joint_plate"]
    R["cageWall"] = (R["plate"] - T) / 2                          # each side of the nut, in x
    need(R["cageWall"] >= 0.9 - 1e-9, f"the cage nut slots leave {R['cageWall']:.2f} walls (two lines: 0.9)")
    R["cageNd"] = jpl + R["cageWall"]                              # head (cage, outside) to the nut
    need(RO["screwAbove"] - RO["lead"] - w / 2 >= 1.0 and RO["screwBelow"] - w / 2 >= 1.5,
         "a cage post is too short round its nut slot")
    R["cage"] = []
    for pq in RO["posts"]:
        f = pq["free"]
        fd = (f[0] * math.cos(math.radians(DRIVE_TILT)) + f[1] * math.sin(math.radians(DRIVE_TILT)),
              -f[0] * math.sin(math.radians(DRIVE_TILT)) + f[1] * math.cos(math.radians(DRIVE_TILT)))
        R["cage"].append({"head": list(to_drive(pq["screw"])), "slot": list(fd),
                          "slotLen": RO["postHalf"] + 1.0, "bike": list(pq["screw"]) + [pq["axis"]]})
    return R


def full_layout(data: dict) -> dict:
    """layout() plus the few config values the FeatureScript reads directly."""
    s, sv, wh = data["s"], data["xc430"], data["wheel"]
    L = layout(data)
    wl, dp, cs, ch = s["wheel"], s["drive_pulley"], s["case"], s["chainstay"]
    L.update({
        "caseW": sv["caseWidth"], "caseH": sv["caseHeight"], "hornT": sv["hornThickness"],
        "hornR": sv["hornDiameter"] / 2, "bossP": sv["bossProjection"], "bossR": sv["bossDiameter"] / 2,
        "holeHW": sv["holeW"] / 2, "holeHH": sv["holeH"] / 2,
        "recessR": s["servo"]["recess_dia"] / 2, "recessD": s["servo"]["recess_depth"],
        "pinHoleR": s["servo"]["horn_pin_hole_dia"] / 2,
        "coreHalf": wl["core_half_width"], "coreR": wl["core_dia"] / 2,
        "rimHalf": wl["rim_half_width"], "rimOR": wl["rim_outer_dia"] / 2,
        "rimIR": wl["rim_inner_dia"] / 2, "hubStubR": wl["hub_stub_dia"] / 2,
        "wheelBoreR": wl["bore_dia"] / 2,
        "rm": wh["rm"], "rollBigR": wh["bigD"] / 2, "rollSmallR": wh["smallD"] / 2,
        "rollLen": wh["len"], "pairGap": wh["gap"], "axlesN": wh["nAxles"],
        "axleR": s["axle"]["dia"] / 2, "hexAF": s["axle"]["hex_af"],
        "nutL": s["axle"]["nut_length"], "headL": s["axle"]["head_length"],
        "flCyl": s["pulley"]["flange_cyl"],
        "padR": dp["pad_dia"] / 2, "padCh": dp["pad_chamfer"],
        "bossClrR": dp["boss_clearance_dia"] / 2, "pinR": dp["pin_dia"] / 2,
        "pinL": dp["pin_length"], "pinC": dp["pin_circle"] / 2,
        "m2R": dp["screw_hole_dia"] / 2, "m2CbR": dp["screw_cb_dia"] / 2,
        "skClr": cs["skirt_clearance"], "reliefCh": cs["relief_chamfer"],
        "spacerR": cs["spacer_dia"] / 2, "m25R": cs["screw_hole_dia"] / 2,
        "m25CbR": cs["screw_cb_dia"] / 2,
        "armHalf": ch["arm_width"] / 2, "csBossR": ch["boss_dia"] / 2, "bedCh": ch["bed_chamfer"],
        "fixGap": s["fixture"]["gap"], "bridgeLayer": s["joint"]["bridge_layer"],
        "tiltDeg": data["tilt_deg"],
    })
    L["xSeat"] = L["xHorn"] + dp["screw_seat"]
    L["m2Engaged"] = dp["screw_length"] - dp["screw_seat"]
    return L


def outlines(data: dict) -> dict:
    """The sketch profiles the FeatureScript extrudes, in mm: both pulleys'
    teeth, the spline, the left belt (outer and inner loop)."""
    s, bt = data["s"], data["belt"]
    L = layout(data)
    pu = s["pulley"]
    h15, _ = htd_outline(bt["teethInput"], bt["pitch"], pu["pld"], s["teeth_15"])
    h45, _ = htd_outline(bt["teethServo"], bt["pitch"], pu["pld"], s["teeth_45"])
    sp = s["spline"]
    spl = spline_outline(sp["outer_dia"] / 2, sp["root_dia"] / 2, sp["lobe_deg"], int(sp["lobes_n"]))
    back = bt["thickness"] - pu["pld"]
    c1, c2 = (0.0, 0.0), (L["Ys"], -L["zs"])
    belt = (hull_loop(c1, L["rP15"] + pu["pld"], c2, L["rP45"] + pu["pld"])
            + hull_loop(c1, L["rP15"] - back, c2, L["rP45"] - back))
    return {"HTD15": h15, "HTD45": h45, "SPLINE": spl, "BELT": belt}


def _fs_map(name: str, d: dict) -> str:
    rows = []
    for k, v in d.items():
        if isinstance(v, str):
            rows.append(f'    "{k}" : "{v}"')
        elif k in ("cskAngle",) or k.endswith("Deg"):
            rows.append(f'    "{k}" : {v:g} * degree')
        elif k.endswith("N"):
            rows.append(f'    "{k}" : {v:g}')
        else:
            rows.append(f'    "{k}" : {v:.6g} * millimeter')
    return f"export const {name} = {{\n" + ",\n".join(rows) + "\n};"


def _fs_val(v) -> str:
    """A plain FeatureScript literal: numbers (mm, unitless), strings, lists, maps."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return f"{v:.9g}"
    if isinstance(v, str):
        return f'"{v}"'
    if isinstance(v, dict):
        return "{ " + ", ".join(f'"{k}" : {_fs_val(x)}' for k, x in v.items()) + " }"
    return "[" + ", ".join(_fs_val(x) for x in v) + "]"


def _fs_segs(name: str, segs: list) -> str:
    """Segments as [[x, y], ...] lists of plain numbers (mm): 2 points a line,
    3 an arc through its midpoint."""
    rows = ["[" + ", ".join(f"[{p[0]:.6g}, {p[1]:.6g}]" for p in sg[1:]) + "]" for sg in segs]
    return f"export const {name} = [\n    " + ",\n    ".join(rows) + "\n];"


FS = r'''FeatureScript %VERSION%;
import(path : "onshape/std/geometry.fs", version : "%VERSION%.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_drive --push drive_features
 *
 * Numbers from config/drive_cad.yaml and bike_params_cad.yaml, every derived
 * one computed in aow_sim.cad_drive's layout() and carried here as DR.
 * Millimetres. X in DR is a MAGNITUDE on the left side; the code negates.
 *
 * MODULE FRAME: origin on the rear axle, +Y along the mean belt (toward the
 * servos), +X right, +Z up. Every part is built on the left and its right
 * twin is the same solid turned 180 deg about Y.
 */

%DR%

%FIXTURE%

%SCREW_OPT%

%SEGS%

/* The side rails and the AHRS tabs (rail_layout), plain mm numbers: the
 * outline pieces as (y, z) on the left rail's plane, the joints' heads (y, z)
 * and ridge directions, the tabs. */
export const RAIL = %RAIL%;

// ---- the servo-mount geometry, copied from the horn-mount-gen studio ----
%SERVO_MOUNT%
// ---- end of the copy ----

%HELPERS%
/** A closed profile of lines and 3-point arcs (plain mm numbers) on a plane,
 * extruded along its normal. `outerOnly` drops regions inside inner loops. */
export function segPrism(context is Context, id is Id, tag is string, origin is Vector,
                         normal is Vector, xDir is Vector, segs is array, depth is ValueWithUnits,
                         outerOnly is boolean) returns Query
{
    const mm = millimeter;
    var sk = newSketchOnPlane(context, id + tag, { "sketchPlane" : plane(origin, normal, xDir) });
    for (var i = 0; i < size(segs); i += 1)
    {
        const s = segs[i];
        if (size(s) == 2)
            skLineSegment(sk, "s" ~ i, { "start" : vector(s[0][0], s[0][1]) * mm,
                                         "end" : vector(s[1][0], s[1][1]) * mm });
        else
            skArc(sk, "s" ~ i, { "start" : vector(s[0][0], s[0][1]) * mm,
                                 "mid" : vector(s[1][0], s[1][1]) * mm,
                                 "end" : vector(s[2][0], s[2][1]) * mm });
    }
    skSolve(sk);
    opExtrude(context, id + (tag ~ "Ext"), {
            "entities" : qSketchRegion(id + tag, outerOnly),
            "direction" : normal,
            "endBound" : BoundingType.BLIND,
            "endDepth" : depth });
    opDeleteBodies(context, id + (tag ~ "Del"), { "entities" : qCreatedBy(id + tag, EntityType.BODY) });
    return qCreatedBy(id + (tag ~ "Ext"), EntityType.BODY);
}

/** Subtract `tools` from `target`. */
export function cutFrom(context is Context, id is Id, target is Query, tools is array)
{
    opBoolean(context, id, { "tools" : qUnion(tools), "targets" : target,
            "operationType" : BooleanOperationType.SUBTRACTION });
}

/**
 * A counterbore's two sacrificial bridge layers, for a hole running +X from a
 * seat at x0 (y, z). Layer 1 is a hole-wide strip along u, CLIPPED to what
 * the counterbore covers -- its ends are the counterbore's own arc, so it
 * neither leaves a lip nor cuts past the circle -- and, with `ext` > 0, to a
 * slot of the counterbore's width running ext along u. Layer 2 is the hole's
 * square. Returns the two cutters.
 */
export function cbLayers(context is Context, id is Id, x0 is ValueWithUnits, y is ValueWithUnits,
                         z is ValueWithUnits, cbR is ValueWithUnits, holeR is ValueWithUnits,
                         u is Vector, ext is ValueWithUnits, bl is ValueWithUnits) returns array
{
    const mm = millimeter;
    const X = vector(1, 0, 0);
    const Y = vector(0, 1, 0);
    const n = vector(-u[1], u[0]);
    const rect = function(a0, a1, hw) returns array
    {
        return [a0 * u - hw * n, a1 * u - hw * n, a1 * u + hw * n, a0 * u + hw * n];
    };
    const far = max([ext, cbR]) + 1 * mm;
    polyPrism(context, id, "l1", vector(x0 - 0.5 * mm, y, z), X, Y, rect(-cbR - 1 * mm, far, holeR), bl + 0.5 * mm);
    var keep = [cylW(context, id + "kc", vector(x0 - 1 * mm, y, z), vector(x0 + bl + 1 * mm, y, z), cbR)];
    if (ext > 0 * mm)
    {
        polyPrism(context, id, "ks", vector(x0 - 1 * mm, y, z), X, Y, rect(0 * mm, far + 1 * mm, cbR), bl + 2 * mm);
        keep = append(keep, qCreatedBy(id + "ksExt", EntityType.BODY));
    }
    const out = cylW(context, id + "out", vector(x0 - 0.9 * mm, y, z), vector(x0 + bl + 0.9 * mm, y, z), far + cbR + 2 * mm);
    cutFrom(context, id + "ring", out, keep);
    cutFrom(context, id + "clip", qCreatedBy(id + "l1Ext", EntityType.BODY), [out]);
    polyPrism(context, id, "l2", vector(x0 + bl - 0.1 * mm, y, z), X, Y, rect(-holeR, holeR, holeR), bl + 0.1 * mm);
    return [qCreatedBy(id + "l1Ext", EntityType.BODY), qCreatedBy(id + "l2Ext", EntityType.BODY)];
}

/**
 * The XC430 as an envelope about its horn datum: `cs` origin on the horn's
 * outer face, z out of the servo, x along the case toward the shaft's end.
 * The case-screw counterbores (both faces) and the horn's pin holes are cut,
 * so the spacers and pins are tested against holes.
 */
export function xc430Envelope(context is Context, id is Id, cs is CoordSystem) returns map
{
    const L  = DR;
    const mm = millimeter;
    const zc = -L.hornT;
    const zb = zc - 2 * L.caseHalf;
    const caseQ = boxIn(context, id + "case", cs, vector(-(L.caseH - L.sfe), -L.caseW / 2, zb),
                        vector(L.sfe, L.caseW / 2, zc));
    const horn = cylIn(context, id + "horn", cs, vector(0 * mm, 0 * mm, zc), vector(0 * mm, 0 * mm, 0 * mm), L.hornR);
    const boss = cylIn(context, id + "boss", cs, vector(0 * mm, 0 * mm, -0.1 * mm), vector(0 * mm, 0 * mm, L.bossP), L.bossR);
    unite(context, id + "hornU", [horn, boss]);
    var rec = [];
    var k = 0;
    const xc = L.sfe - L.caseH / 2;
    for (var sx in [1, -1])
    {
        for (var sy in [1, -1])
        {
            const x = xc + sx * L.holeHH;
            const y = sy * L.holeHW;
            rec = append(rec, cylIn(context, id + ("rf" ~ k), cs, vector(x, y, zc - L.recessD), vector(x, y, zc + 0.1 * mm), L.recessR));
            rec = append(rec, cylIn(context, id + ("rb" ~ k), cs, vector(x, y, zb - 0.1 * mm), vector(x, y, zb + L.recessD), L.recessR));
            k += 1;
        }
    }
    cutFrom(context, id + "recess", caseQ, rec);
    var ph = [];
    for (var i = 0; i < 4; i += 1)
    {
        const p = L.pinC * vector(cos(i * 90 * degree), sin(i * 90 * degree));
        ph = append(ph, cylIn(context, id + ("ph" ~ i), cs, vector(p[0], p[1], zc - 0.2 * mm), vector(p[0], p[1], 0.1 * mm), L.pinHoleR));
    }
    cutFrom(context, id + "pinHoles", qUnion([caseQ, horn, boss]), ph);
    return { "caseQ" : caseQ, "horn" : qUnion([horn, boss]) };
}

/**
 * Everything. `opt.shift` moves the axle group (chainstays, spline pulleys,
 * wheel, M5) back by the belt-tension travel. Returns the stages (what the
 * tension moves, what stays), the belts, and each printed part with the
 * direction that prints UP.
 */
export function driveBuild(context is Context, id is Id, opt is map) returns map
{
    const L  = DR;
    const mm = millimeter;
    const O  = vector(0, 0, 0) * meter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const P  = id + "parts";
    const H  = id + "hw";
    const bl = L.bridgeLayer;
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };
    const at = function(x, y, z) returns Vector { return vector(x, y, z); };
    const flip = function(p is Vector) returns Vector { return vector(-p[0], p[1], -p[2]); };
    const ya = L.Ys;
    const za = -L.zs;
    const pa = at(0 * mm, ya, za);
    const xz  = plane(O, -Y, X);
    const xzA = plane(pa, -Y, X);
    const z0 = 0 * mm;

    // ---- servo A: horn face at -xHorn facing -X, its case running back (-Y)
    step("servo");
    const sv = xc430Envelope(context, H + "servoA", coordSystem(at(-L.xHorn, ya, za), Y, -X));

    // ---- mock rear wheel: core, rim rings, hub stubs, rollers; the bore and
    // the female splines cut from it
    step("wheel");
    var wp = [cylW(context, H + "core", at(-L.coreHalf, z0, z0), at(L.coreHalf, z0, z0), L.coreR)];
    for (var sg in [1, -1])
    {
        const tg = sg > 0 ? "P" : "N";
        const rim = cylW(context, H + ("rim" ~ tg), at(sg * (L.coreHalf - 0.01 * mm), z0, z0), at(sg * L.rimHalf, z0, z0), L.rimOR);
        cutFrom(context, H + ("rimCut" ~ tg), rim, [cylW(context, H + ("rimIn" ~ tg), at(sg * L.coreHalf, z0, z0),
                at(sg * (L.rimHalf + 1 * mm), z0, z0), L.rimIR)]);
        wp = append(wp, rim);
        wp = append(wp, cylW(context, H + ("stub" ~ tg), at(sg * (L.coreHalf - 0.1 * mm), z0, z0), at(sg * L.hubFace, z0, z0), L.hubStubR));
    }
    for (var k = 0; k < L.axlesN; k += 1)
    {
        const th = k * 360 / L.axlesN * degree;
        const e = vector(0, cos(th), sin(th));
        const t = vector(0, -sin(th), cos(th));
        const c = L.rm * e;
        for (var sg in [1, -1])
        {
            const tg = "rl" ~ k ~ (sg > 0 ? "P" : "N");
            const a = sg * L.pairGap / 2;
            const b = sg * (L.pairGap / 2 + L.rollLen);
            revolveProfile(context, H, tg, plane(c, e, t), line(c, t),
                    [vector(a, z0), vector(b, z0), vector(b, L.rollSmallR), vector(a, L.rollBigR)]);
            wp = append(wp, qCreatedBy(H + (tg ~ "Rev"), EntityType.BODY));
        }
    }
    unite(context, H + "wheelU", wp);
    const wheelPt = at(z0, z0, L.coreR - 2 * mm);
    const wheel = partAt(context, H, wheelPt);
    cutFrom(context, H + "wheelBore", wheel, [
            cylW(context, H + "wBore", at(-L.rimHalf - 1 * mm, z0, z0), at(L.rimHalf + 1 * mm, z0, z0), L.wheelBoreR),
            segPrism(context, H, "wSplN", at(-L.hubFace - 0.5 * mm, z0, z0), X, Y, SPLINE, L.splineLen + 0.5 * mm, false),
            segPrism(context, H, "wSplP", at(L.hubFace + 0.5 * mm, z0, z0), -X, Y, SPLINE, L.splineLen + 0.5 * mm, false)]);

    // ---- the M5: nut on the left, socket head on the right, each in its
    // chainstay's hex pocket
    var hex = [];
    var hexIn = [];
    for (var k = 0; k < 6; k += 1)
    {
        const a = (30 + k * 60) * degree;
        hex = append(hex, L.hexR * vector(cos(a), sin(a)));
        hexIn = append(hexIn, (L.hexR - 0.06 * mm) * vector(cos(a), sin(a)));
    }
    const shank = cylW(context, H + "shank", at(-L.clampX - L.nutL, z0, z0), at(L.clampX, z0, z0), L.axleR);
    polyPrism(context, H, "nut", at(-L.clampX - L.nutL, z0, z0), X, Y, hexIn, L.nutL);
    const head = cylW(context, H + "head", at(L.clampX, z0, z0), at(L.clampX + L.headL, z0, z0), L.hexAF / 2 - 0.05 * mm);
    unite(context, H + "axleU", [shank, qCreatedBy(H + "nutExt", EntityType.BODY), head]);
    const axlePt = at(z0, z0, z0);

    // ---- belt L, a ring round the pitch line: back to tooth tips
    step("belt");
    segPrism(context, H, "beltL", at(-L.beltX1, z0, z0), X, Y, BELT, L.beltX1 - L.beltX0, true);
    const beltPt = at(-(L.beltX0 + L.beltX1) / 2, ya + L.rP45 - 1 * mm, za);

    // ---- spline pulley L: hub, inner flange (chamfered down to the hub),
    // teeth, outer flange; the spline stub; the bore, thrust cone, counterbore
    step("spline pulley");
    revolveProfile(context, P, "spRev", xz, line(O, X),
            [vector(-L.hubFace, z0), vector(-L.hubFace, L.hubR), vector(-L.xCh, L.hubR),
             vector(-L.flangeIn, L.rFl15), vector(-(L.flangeIn + L.flCyl), L.rFl15),
             vector(-L.teethIn, L.rLip15), vector(-L.teethIn, L.rRoot15 - 1 * mm),
             vector(-L.teethOut, L.rRoot15 - 1 * mm), vector(-L.teethOut, L.rLip15),
             vector(-(L.outerFace - L.flCyl), L.rFl15), vector(-L.outerFace, L.rFl15), vector(-L.outerFace, z0)]);
    const spT = segPrism(context, P, "spTeeth", at(-L.teethOut, z0, z0), X, Y, HTD15, L.teethOut - L.teethIn, false);
    const spS = segPrism(context, P, "spStub", at(-L.hubFace - 0.2 * mm, z0, z0), X, Y, SPLINE, L.splineLen + 0.2 * mm, false);
    unite(context, P + "spU", [qCreatedBy(P + "spRevRev", EntityType.BODY), spT, spS]);
    const spPt = at(-(L.teethIn + L.teethOut) / 2, z0, (L.cbR + L.rRoot15) / 2);
    revolveProfile(context, P, "spCone", xz, line(O, X),
            [vector(-L.xCe, z0), vector(-L.xCe, L.bore15R), vector(-L.xCb, L.cbR),
             vector(-L.outerFace - 1 * mm, L.cbR), vector(-L.outerFace - 1 * mm, z0)]);
    cutFrom(context, P + "spCut", partAt(context, P, spPt), [
            qCreatedBy(P + "spConeRev", EntityType.BODY),
            cylW(context, P + "spBore", at(-(L.hubFace - L.splineLen) + 1 * mm, z0, z0), at(-L.xCe - 0.2 * mm, z0, z0), L.bore15R)]);

    // ---- drive pulley L on servo A's horn: pad into the case's relief, the
    // flanges and teeth, 4 pins; boss clearance, M2s on the diagonals from
    // outside, their counterbores bridged in two layers
    step("drive pulley");
    revolveProfile(context, P, "dpRev", xzA, line(pa, X),
            [vector(-L.xHorn, z0), vector(-L.xHorn, L.padR), vector(-L.xPo, L.padR),
             vector(-L.flangeIn, L.padR + L.padCh), vector(-L.flangeIn, L.rFl45),
             vector(-(L.flangeIn + L.flCyl), L.rFl45), vector(-L.teethIn, L.rLip45),
             vector(-L.teethIn, L.rRoot45 - 1 * mm), vector(-L.teethOut, L.rRoot45 - 1 * mm),
             vector(-L.teethOut, L.rLip45), vector(-(L.outerFace - L.flCyl), L.rFl45),
             vector(-L.outerFace, L.rFl45), vector(-L.outerFace, z0)]);
    const dpT = segPrism(context, P, "dpTeeth", at(-L.teethOut, ya, za), X, Y, HTD45, L.teethOut - L.teethIn, false);
    var dpAdd = [qCreatedBy(P + "dpRevRev", EntityType.BODY), dpT];
    for (var i = 0; i < 4; i += 1)
    {
        const q = L.pinC * vector(cos(i * 90 * degree), sin(i * 90 * degree));
        dpAdd = append(dpAdd, cylW(context, P + ("dpPin" ~ i), at(-L.xHorn - 0.1 * mm, ya + q[0], za + q[1]),
                at(-(L.xHorn - L.pinL), ya + q[0], za + q[1]), L.pinR));
    }
    unite(context, P + "dpU", dpAdd);
    const dpPt = at(-(L.teethIn + L.teethOut) / 2, ya, za + 20 * mm);
    var dpCut = [cylW(context, P + "dpBoss", at(-L.xHorn + 0.1 * mm, ya, za), at(-L.flangeIn, ya, za), L.bossClrR)];
    for (var i = 0; i < 4; i += 1)
    {
        const q = L.pinC * vector(cos((45 + i * 90) * degree), sin((45 + i * 90) * degree));
        const y = ya + q[0];
        const z = za + q[1];
        dpCut = append(dpCut, cylW(context, P + ("dpCb" ~ i), at(-L.outerFace - 1 * mm, y, z), at(-L.xSeat, y, z), L.m2CbR));
        dpCut = concatenateArrays([dpCut, cbLayers(context, P + ("dpL" ~ i), -L.xSeat, y, z, L.m2CbR, L.m2R, vector(1, 0), 0 * mm, bl)]);
        dpCut = append(dpCut, cylW(context, P + ("dpM2" ~ i), at(-(L.xSeat - 2 * bl) - 0.01 * mm, y, z), at(-L.xHorn + 0.1 * mm, y, z), L.m2R));
    }
    cutFrom(context, P + "dpCut", partAt(context, P, dpPt), dpCut);

    // ---- case side L: plate, skirt round both servos, the rear block with
    // the chainstay's channel, the fixture tab; pocket, spacers, horn relief,
    // wire gap, 8 M2.5, the tension joint's slot and nut slot
    step("case side");
    // without the fixture the plate ends at the skirt's front wall, no tab
    const fix = opt.fixture == true;
    var csBody = [boxW(context, P + "csPlate", at(-L.xPo, L.Yc0, -L.Zp), at(-L.caseHalf, fix ? L.Yfront : L.Ytab0, L.Zp)),
                  boxW(context, P + "csSkirt", at(-L.caseHalf - 0.01 * mm, L.Ysk0, -L.Zp), at(-L.skirtTop, L.Ytab0, L.Zp)),
                  boxW(context, P + "csRear", at(-L.xPo, L.Yc0, -L.Zp), at(-L.xRin, L.Yb - L.skClr, L.Zp))];
    if (fix)
        csBody = append(csBody, boxW(context, P + "csTab", at(-L.xPo, L.Yf + L.skClr, -L.Zp), at(-L.xTab, L.Yfront, L.Zp)));
    unite(context, P + "csU", csBody);
    const csPt = fix ? at(-L.xPo + 0.5 * mm, L.Ytab0 + 1 * mm, z0) : at(-L.xPo + 0.5 * mm, L.Ytab0 - 0.5 * mm, z0);
    cutFrom(context, P + "csPocket", partAt(context, P, csPt), [
            boxW(context, P + "csPk", at(-L.skirtTop + 1 * mm, L.Yb - L.skClr, -L.Zi), at(-L.caseHalf, L.Yf + L.skClr, L.Zi))]);
    var spc = [partAt(context, P, csPt)];
    var holes = [];
    var k2 = 0;
    for (var zc in [za, -za])
    {
        for (var sy in [1, -1])
        {
            for (var sz in [1, -1])
            {
                const y = L.Yhc + sy * L.holeHH;
                const z = zc + sz * L.holeHW;
                spc = append(spc, cylW(context, P + ("csSp" ~ k2), at(-L.caseHalf - 0.01 * mm, y, z), at(-L.spacerTop, y, z), L.spacerR));
                holes = append(holes, cylW(context, P + ("csCb" ~ k2), at(-L.xPo - 1 * mm, y, z), at(-L.m25Seat, y, z), L.m25CbR));
                // where the counterbore would leave a sliver against the horn
                // relief, it runs on into the relief as a slot aimed at the
                // horn's centre and the bridge layers turn with it (the hand
                // drawing's way)
                const dv = vector(ya - y, za - z);
                const dist = norm(dv);
                const near = dist - L.m25CbR < L.reliefR + L.reliefCh + 1 * mm;
                const u = near ? dv / dist : vector(1, 0);
                const ext = near ? dist - L.reliefR + 0.5 * mm : 0 * mm;
                holes = concatenateArrays([holes, cbLayers(context, P + ("csL" ~ k2), -L.m25Seat, y, z, L.m25CbR, L.m25R, u, ext, bl)]);
                if (near)
                {
                    const nn = vector(-u[1], u[0]);
                    polyPrism(context, P, "csX" ~ k2, at(-L.xPo - 1 * mm, y, z), X, Y,
                              [-L.m25CbR * nn, ext * u - L.m25CbR * nn, ext * u + L.m25CbR * nn, L.m25CbR * nn],
                              1 * mm + L.xPo - L.m25Seat);
                    holes = append(holes, qCreatedBy(P + ("csX" ~ k2 ~ "Ext"), EntityType.BODY));
                }
                holes = append(holes, cylW(context, P + ("csH" ~ k2), at(-(L.m25Seat - 2 * bl) - 0.01 * mm, y, z), at(-L.spacerTop + 0.5 * mm, y, z), L.m25R));
                k2 += 1;
            }
        }
    }
    unite(context, P + "csSpU", spc);
    if (L.rcLeg > 0 * mm)
    {
        // the rear block's +-Z corners, 45 deg (prisms in Y-Z along X)
        for (var sz in [1, -1])
        {
            polyPrism(context, P, "csRc" ~ (sz > 0 ? "p" : "n"), at(-L.xPo - 1 * mm, 0 * mm, 0 * mm), X, Y,
                      [vector(L.Yc0 - 1 * mm, sz * (L.Zp - L.rcLeg - 1 * mm)), vector(L.Yc0 + L.rcLeg + 1 * mm, sz * (L.Zp + 1 * mm)),
                       vector(L.Yc0 - 1 * mm, sz * (L.Zp + 1 * mm))], L.xPo - L.skirtTop + 2 * mm);
            holes = append(holes, qCreatedBy(P + ("csRc" ~ (sz > 0 ? "p" : "n") ~ "Ext"), EntityType.BODY));
        }
    }
    revolveProfile(context, P, "csRelCh", xzA, line(pa, X),
            [vector(-L.xPo - 0.1 * mm, z0), vector(-L.xPo - 0.1 * mm, L.reliefR + L.reliefCh + 0.1 * mm),
             vector(-L.xPo + L.reliefCh, L.reliefR), vector(-L.xPo + L.reliefCh, z0)]);
    holes = append(holes, qCreatedBy(P + "csRelChRev", EntityType.BODY));
    holes = append(holes, cylW(context, P + "csRel", at(-L.xPo - 1 * mm, ya, za), at(-L.caseHalf + 0.01 * mm, ya, za), L.reliefR));
    holes = append(holes, boxW(context, P + "csWire", at(-L.skirtTop + 1 * mm, L.wireY0, L.Zi - 0.5 * mm), at(-L.caseHalf, L.wireY1, L.Zp + 1 * mm)));
    // the channel, open behind and at the outer face, closed in front: a
    // plain rectangular keyway (the tongue stops short of its floor)
    holes = append(holes, boxW(context, P + "csChan", at(-L.xPo - 1 * mm, L.Yc0 - 1 * mm, -(L.tongueHalf + L.tongueClr)),
            at(-L.tongueTop, L.Yj1, L.tongueHalf + L.tongueClr)));
    // the screw's slot through the web, as long as the travel; the nut's
    // slot along Y, in from the back edge, the nut sliding with the screw;
    // past it a shallow slot for the tip. Where each slot leaves a ceiling
    // (the channel's, the nut slot's) it gets a counterbore's two layers:
    // layer 1 cut across the ceiling's whole span over the slot's length,
    // layer 2 the slot itself, bridged along Y.
    const stadium = function(tag is string, x0, x1) returns array
    {
        return [cylW(context, P + (tag ~ "0"), at(x0, L.Yscr, z0), at(x1, L.Yscr, z0), L.holeR),
                cylW(context, P + (tag ~ "1"), at(x0, L.Yscr - L.travel, z0), at(x1, L.Yscr - L.travel, z0), L.holeR),
                boxW(context, P + (tag ~ "B"), at(x0, L.Yscr - L.travel, -L.holeR), at(x1, L.Yscr, L.holeR))];
    };
    const ys0 = L.Yscr - L.travel - L.holeR;
    const ys1 = L.Yscr + L.holeR;
    const chW = L.tongueHalf + L.tongueClr;
    const xNc = L.xN0 - L.nutSlotT;
    holes = concatenateArrays([holes,
            [boxW(context, P + "csChL1", at(-L.tongueTop - 0.01 * mm, ys0, -chW), at(-(L.tongueTop - bl), ys1, chW))],
            stadium("csSl", -(L.tongueTop - bl) - 0.01 * mm, -L.xN0 + 0.01 * mm),
            [boxW(context, P + "csNut", at(-L.xN0, L.Yc0 - 1 * mm, -L.nutHalfY),
                  at(-xNc, L.Yscr + L.nutCornerY + 0.2 * mm, L.nutHalfY)),
             boxW(context, P + "csNtL1", at(-xNc - 0.01 * mm, ys0, -L.nutHalfY), at(-(xNc - bl), ys1, L.nutHalfY))],
            stadium("csTp", -(xNc - bl) - 0.01 * mm, -(xNc - L.tipSlot))]);
    cutFrom(context, P + "csCut", partAt(context, P, csPt), holes);

    // ---- chainstay L: the bed plate and boss, the riser inside the belt
    // loop, the tongue; the bearing stub; the hex pocket (its mouth chamfered,
    // its floor the clamp face, bridged to the bore), the tension screw's
    // countersink at the bottom of a bore from the outer face
    step("chainstay");
    const cyPt = at(-(L.xCo + L.xCi) / 2, (L.Yr0 + L.csBossR) / 2, z0);
    revolveProfile(context, P, "cyStub", xz, line(O, X),
            [vector(-L.xCi - 0.5 * mm, z0), vector(-L.xCi - 0.5 * mm, L.stubR), vector(-L.xA, L.stubR),
             vector(-L.xRel, L.rC0), vector(-L.xE, L.rC1), vector(-L.xE, z0)]);
    unite(context, P + "cyU", [
            boxW(context, P + "cyBed", at(-L.xCo, z0, -L.armHalf), at(-L.xCi, L.Yj1, L.armHalf)),
            cylW(context, P + "cyBoss", at(-L.xCo, z0, z0), at(-L.xCi, z0, z0), L.csBossR),
            boxW(context, P + "cyRiser", at(-L.xCi - 0.5 * mm, L.Yr0, -L.armHalf), at(-L.riserTop, L.Yj1, L.armHalf)),
            boxW(context, P + "cyTongue", at(-L.riserTop - 0.5 * mm, L.Yc0, -L.tongueHalf), at(-L.tongueEnd, L.Yj1, L.tongueHalf)),
            qCreatedBy(P + "cyStubRev", EntityType.BODY)]);
    polyPrism(context, P, "cyHex", at(-L.xCo - 1 * mm, z0, z0), X, Y, hex, L.xCo + 1 * mm - L.clampX);
    const ma = L.hexAF / 2;
    const mr = L.hexR + L.bedCh;
    revolveProfile(context, P, "cyMouth", xz, line(O, X),
            [vector(-L.xCo - 0.1 * mm, z0), vector(-L.xCo - 0.1 * mm, mr + 0.1 * mm),
             vector(-L.xCo + (mr - ma), ma), vector(-L.xCo + (mr - ma), z0)]);
    const ys = L.Yscr;
    const cbTop = L.xH + (L.headBoreR - L.headR);
    revolveProfile(context, P, "cyCsk", plane(at(z0, ys, z0), -Y, X), line(at(z0, ys, z0), X),
            [vector(-cbTop - 1 * mm, z0), vector(-cbTop - 1 * mm, L.headBoreR), vector(-cbTop, L.headBoreR),
             vector(-(L.xH - L.cskDepth), L.holeR), vector(-(L.xH - L.cskDepth), z0)]);
    cutFrom(context, P + "cyCut", partAt(context, P, cyPt), [
            qCreatedBy(P + "cyHexExt", EntityType.BODY), qCreatedBy(P + "cyMouthRev", EntityType.BODY),
            boxW(context, P + "cyB1", at(-L.clampX - 0.5 * mm, -ma, -L.csBoreR), at(-(L.clampX - bl), ma, L.csBoreR)),
            boxW(context, P + "cyB2", at(-(L.clampX - bl) - 0.5 * mm, -L.csBoreR, -L.csBoreR), at(-(L.clampX - 2 * bl), L.csBoreR, L.csBoreR)),
            cylW(context, P + "cyBore", at(-(L.clampX - 2 * bl) - 0.01 * mm, z0, z0), at(-L.xE + 1 * mm, z0, z0), L.csBoreR),
            cylW(context, P + "cyHead", at(-L.xCo - 1 * mm, ys, z0), at(-cbTop, ys, z0), L.headBoreR),
            qCreatedBy(P + "cyCskRev", EntityType.BODY),
            cylW(context, P + "cyHole", at(-(L.xH - L.cskDepth) - 0.01 * mm, ys, z0), at(-L.tongueTop + 1 * mm, ys, z0), L.holeR)]);
    // finishing, the user's hand edits (2026-10-11), every edge found by a
    // point on it at z 0: the front-outer corner, the riser's rear-top edge
    // and a gusset of the same size in the concave corner riser x bed; a
    // fillet on both long edges of each chamfer; then the outer face's rim,
    // round the boss and on along the side edges (one tangent chain)
    const cy = partAt(context, P, cyPt);
    const edgeAt = function(p is Vector) returns Query
    {
        return qContainsPoint(qOwnedByBody(cy, EntityType.EDGE), p);
    };
    const c1 = L.cyFront;
    const c2 = L.cyRiserCh;
    opChamfer(context, P + "cyCh1", { "entities" : edgeAt(at(-L.xCo, L.Yj1, z0)),
            "chamferType" : ChamferType.EQUAL_OFFSETS, "width" : c1 });
    opChamfer(context, P + "cyCh2", { "entities" : qUnion([edgeAt(at(-L.riserTop, L.Yr0, z0)), edgeAt(at(-L.xCi, L.Yr0, z0))]),
            "chamferType" : ChamferType.EQUAL_OFFSETS, "width" : c2 });
    opFillet(context, P + "cyFil", { "entities" : qUnion([
            edgeAt(at(-L.xCo, L.Yj1 - c1, z0)), edgeAt(at(-(L.xCo - c1), L.Yj1, z0)),
            edgeAt(at(-(L.riserTop + c2), L.Yr0, z0)), edgeAt(at(-L.riserTop, L.Yr0 + c2, z0)),
            edgeAt(at(-L.xCi, L.Yr0 - c2, z0)), edgeAt(at(-(L.xCi - c2), L.Yr0, z0))]),
            "radius" : L.cyFillet });
    opChamfer(context, P + "cyCh3", { "entities" : edgeAt(at(-L.xCo, -L.csBossR, z0)),
            "chamferType" : ChamferType.EQUAL_OFFSETS, "width" : L.cyEdgeCh, "tangentPropagation" : true });

    // ---- the right side: every left part and its hardware turned 180 deg
    // about Y. One part per pair, by construction.
    step("right side");
    const rot = rotationAround(line(O, Y), 180 * degree);
    const horn = at(-L.xHorn + 0.5 * mm, ya + 5 * mm, za);
    const svPt = at(z0, L.Yhc, za);
    const lefts = [spPt, dpPt, csPt, cyPt];
    var lq = [];
    for (var p in lefts)
        lq = append(lq, partAt(context, P, p));
    opPattern(context, P + "rot", { "entities" : qUnion(lq), "transforms" : [rot], "instanceNames" : ["R"] });

    // ---- the side rails (aow-bike-whole-v2) and the AHRS tabs. Built on the
    // left and MIRRORED to the right -- a rail is not symmetric under the turn
    // that makes the case sides one part -- then each united into its case
    // side. The rail prints on the case side's outer face with it.
    if (opt.rail == true)
    {
        step("rails");
        const R  = RAIL;
        const RL = id + "rail";
        const RB = RL + "b";       // the rail's bodies (several: the tabs and the stub stand apart from the band)
        const RC = RL + "c";       // its cutters
        for (var k = 0; k < size(R.pieces); k += 1)
        {
            var pts = [];
            for (var q in R.pieces[k])
                pts = append(pts, vector(q[0], q[1]) * mm);
            polyPrism(context, RB, "pc" ~ k, at(-L.xPo, z0, z0), X, Y, pts, L.xPo - R.innerX * mm);
        }
        // the battery stops: a lip off each arm's inner face (both rails)
        for (var k = 0; k < size(R.stops); k += 1)
        {
            const sb = R.stops[k];
            boxW(context, RB + ("stop" ~ k), at(-sb.x[1] * mm, sb.y[0] * mm, sb.z[0] * mm),
                 at(-sb.x[0] * mm, sb.y[1] * mm, sb.z[1] * mm));
        }
        // the AHRS tabs: the case side's plate and skirt run forward to the
        // board's back face, inward to the tab's end
        boxW(context, RB + "tab", at(-L.xPo, L.Ytab0 - 0.5 * mm, (R.ahZ[0] - R.ahTabHalf) * mm),
             at(-R.ahTabIn * mm, R.ahY1 * mm, (R.ahZ[size(R.ahZ) - 1] + R.ahTabHalf) * mm));
        const railL = qBodyType(qCreatedBy(RB, EntityType.BODY), BodyType.SOLID);
        // inside the case side's outline (its rear corners chamfered) the
        // rail stays the plate's 3.6
        var cp = [];
        for (var q in R.casePoly)
            cp = append(cp, vector(q[0], q[1]) * mm);
        polyPrism(context, RC, "fp", at(-L.caseHalf, z0, z0), X, Y, cp, L.caseHalf - R.innerX * mm + 0.1 * mm);
        var rc = [qCreatedBy(RC + "fpExt", EntityType.BODY)];
        // each M2: the hole through the tab (teardropped: it lies level as the
        // case side prints, X+ up), the nut's slot down from the tab's end
        const rT = R.ahHoleR * mm;
        for (var k = 0; k < size(R.ahZ); k += 1)
        {
            step("rail ahrs " ~ k);
            const z = R.ahZ[k] * mm;
            const x = -R.ahX * mm;
            rc = append(rc, cylW(context, RC + ("ahH" ~ k), at(x, R.ahHoleY0 * mm, z), at(x, R.ahY1 * mm + 0.1 * mm, z), rT));
            polyPrism(context, RC, "ahT" ~ k, at(x, R.ahHoleY0 * mm, z), Y, Z,
                      [vector(0 * mm, 0 * mm), vector(rT / sqrt(2), rT / sqrt(2)), vector(0 * mm, rT * sqrt(2)),
                       vector(-rT / sqrt(2), rT / sqrt(2))], (R.ahY1 - R.ahHoleY0) * mm + 0.1 * mm);
            rc = append(rc, qCreatedBy(RC + ("ahT" ~ k ~ "Ext"), EntityType.BODY));
            rc = append(rc, boxW(context, RC + ("ahN" ~ k), at(-R.ahNutOut * mm, R.ahN0 * mm, z - R.ahNutHalf * mm),
                                 at(-R.ahTabIn * mm + 1 * mm, R.ahN1 * mm, z + R.ahNutHalf * mm)));
        }
        // the joints: a countersink from the outer face and the groove for
        // the other part's ridge on the inner face (the house joint, cut here
        // half by half: the steering and the righting cut their own)
        const railJoint = function(S is Id, k is number, j is map) returns array
        {
            const h = at(-L.xPo, j.head[0] * mm, j.head[1] * mm);
            const J = S + ("j" ~ k);
            screwJointGeometry(context, J, coordSystem(h, vector(0, j.dir[0], j.dir[1]), X), mergeMaps(SCREW_OPT, {
                    "nutDepth" : FIXTURE.joint_plate + 2 * mm, "headPocket" : 0 * mm, "bothWays" : false,
                    "nutSlotLength" : 1 * mm }));
            opDeleteBodies(context, S + ("jd" ~ k), { "entities" : qCreatedBy(J + "slotExt", EntityType.BODY) });
            // the groove drawn along Y and turned about X into the joint's
            // direction: ridgeSolid's sketch will not extrude along an oblique
            // direction (EXTRUDE_FAILED, 2026-10-16; the righting found the same)
            const jp = h + FIXTURE.joint_plate * X;
            const g = ridgeSolid(context, S + ("g" ~ k), jp + j.gMid * mm * Y, Y, -X, FIXTURE.ridge_height,
                                 FIXTURE.ridge_clearance, j.gHalf * mm);
            opTransform(context, S + ("gx" ~ k), { "bodies" : g,
                    "transform" : rotationAround(line(jp, X), atan2(j.dir[1], j.dir[0])) });
            return [qCreatedBy(J + "screwRev", EntityType.BODY), g];
        };
        for (var k = 0; k < size(R.joints); k += 1)
        {
            step("rail joint " ~ k);
            rc = concatenateArrays([rc, railJoint(RC, k, R.joints[k])]);
        }
        // the cage posts' joints: the head in the cage (not drawn) joint_plate
        // outside the rail; the nut centred in the post's thickness, its slot
        // out through the post's free edge (across its axis), its roof
        // bridged in two layers as the rail prints
        const bl = FIXTURE.bridge_layer;
        const rH = SCREW_OPT.holeDia / 2;
        const sw = SCREW_OPT.nutSlotWidth;
        for (var k = 0; k < size(R.cage); k += 1)
        {
            step("rail cage " ~ k);
            const c = R.cage[k];
            const h = at(-L.xPo - FIXTURE.joint_plate, c.head[0] * mm, c.head[1] * mm);
            const cs = coordSystem(h, vector(0, c.slot[0], c.slot[1]), X);
            const J = RC + ("cgJ" ~ k);
            screwJointGeometry(context, J, cs, mergeMaps(SCREW_OPT, {
                    "nutDepth" : R.cageNd * mm, "headPocket" : 0 * mm, "bothWays" : false,
                    "nutSlotLength" : c.slotLen * mm }));
            rc = append(rc, qCreatedBy(J + "screwRev", EntityType.BODY));
            rc = append(rc, qCreatedBy(J + "slotExt", EntityType.BODY));
            const d0 = R.cageNd * mm + SCREW_OPT.nutSlotThickness;
            rc = append(rc, boxIn(context, RC + ("cgB" ~ k), cs, vector(-rH, -sw / 2, d0), vector(rH, sw / 2, d0 + bl)));
            rc = append(rc, boxIn(context, RC + ("cgC" ~ k), cs, vector(-rH, -rH, d0 + bl), vector(rH, rH, d0 + 2 * bl)));
        }
        step("rail cut");
        cutFrom(context, RC + "cut", railL, rc);
        step("rail mirror");
        opPattern(context, RL + "mir", { "entities" : railL,
                "transforms" : [mirrorAcross(plane(O, X))], "instanceNames" : ["R"] });
        // the case side first: the united body keeps its identity
        opBoolean(context, RL + "uL", { "tools" : qUnion([partAt(context, P, csPt), railL]),
                "operationType" : BooleanOperationType.UNION });
        opBoolean(context, RL + "uR", { "tools" : qUnion([partAt(context, P, flip(csPt)),
                                                          qBodyType(qCreatedBy(RL + "mir", EntityType.BODY), BodyType.SOLID)]),
                "operationType" : BooleanOperationType.UNION });
        // the power holder's ears and joints, on the LEFT rail only: added
        // after the mirror, united into the left case side, then cut
        if (size(R.leftPieces) > 0 || size(R.leftJoints) > 0)
        {
            step("rail left");
            const RE = RL + "e";
            const RF = RL + "f";
            for (var k = 0; k < size(R.leftPieces); k += 1)
            {
                var pts = [];
                for (var q in R.leftPieces[k])
                    pts = append(pts, vector(q[0], q[1]) * mm);
                polyPrism(context, RE, "pc" ~ k, at(-L.xPo, z0, z0), X, Y, pts, L.xPo - R.innerX * mm);
            }
            opBoolean(context, RL + "eU", { "tools" : qUnion([partAt(context, P, csPt),
                                                              qBodyType(qCreatedBy(RE, EntityType.BODY), BodyType.SOLID)]),
                    "operationType" : BooleanOperationType.UNION });
            var ec = [];
            for (var k = 0; k < size(R.leftJoints); k += 1)
                ec = concatenateArrays([ec, railJoint(RF, k, R.leftJoints[k])]);
            if (size(ec) > 0)
                cutFrom(context, RL + "fcut", partAt(context, P, csPt), ec);
        }
    }

    // ---- fixture mock between the tabs; 2 x 6-32 a side, heads in the case
    // sides' outer faces, nuts in the block, a ridge along Y on each case
    const fxPt = at(z0, L.Yfront - 1 * mm, z0);
    if (fix)
    {
        step("fixture");
        boxW(context, P + "fixture", at(-L.xTab, L.Ytab0 + L.fixGap, -L.Zp), at(L.xTab, L.Yfront, L.Zp));
        var jn = 0;
        for (var sg in [1, -1])
        {
            const cq = partAt(context, P, sg > 0 ? csPt : flip(csPt));
            for (var y in [L.yF0, L.yF1])
            {
                screwJoint(context, P + ("jF" ~ jn), at(-sg * L.xPo, y, sg * L.zF), sg * X, sg * Z, 0 * mm,
                           cq, partAt(context, P, fxPt), false, Y, L.ridgeHalf, sg * X, Y);
                jn += 1;
            }
        }
    }

    // the hardware's right side, under its own id (an id's operations must be contiguous)
    const HR = id + "hwR";
    opPattern(context, HR, { "entities" : qUnion([sv.caseQ, sv.horn, partAt(context, H, beltPt)]),
            "transforms" : [rot], "instanceNames" : ["R"] });

    // ---- names, colours, print orientation
    const pulleyC = color(0.93, 0.56, 0.20);
    const frameC  = color(0.62, 0.64, 0.68);
    var parts = [["spline pulley L", spPt, "X+", X, "move", pulleyC], ["spline pulley R", flip(spPt), "X-", -X, "move", pulleyC],
                 ["drive pulley L", dpPt, "X+", X, "fixed", pulleyC], ["drive pulley R", flip(dpPt), "X-", -X, "fixed", pulleyC],
                 ["case side L", csPt, "X+", X, "fixed", frameC], ["case side R", flip(csPt), "X-", -X, "fixed", frameC],
                 ["chainstay L", cyPt, "X+", X, "move", frameC], ["chainstay R", flip(cyPt), "X-", -X, "move", frameC]];
    if (fix)
        parts = append(parts, ["fixture mock", fxPt, "Y+", Y, "fixed", color(0.45, 0.62, 0.45)]);
    var stages = { "fixed" : [], "move" : [] };
    var prints = [];
    for (var n in parts)
    {
        const q = partAt(context, P, n[1]);
        dress(context, q, n[0] ~ " [print " ~ n[2] ~ "]", n[5],
              "Print with the " ~ n[2] ~ " side facing UP (module frame).");
        stages[n[4]] = append(stages[n[4]], q);
        prints = append(prints, [n[0], q, n[3]]);
    }
    const dark = color(0.16, 0.16, 0.18);
    const hw = [[svPt, "XC430 A", dark, "fixed"], [flip(svPt), "XC430 B", dark, "fixed"],
                [horn, "XC430 horn A", color(0.3, 0.3, 0.32), "fixed"], [flip(horn), "XC430 horn B", color(0.3, 0.3, 0.32), "fixed"],
                [beltPt, "belt L", color(0.10, 0.10, 0.11), "belt"], [flip(beltPt), "belt R", color(0.10, 0.10, 0.11), "belt"],
                [wheelPt, "rear wheel", color(0.36, 0.56, 0.76), "move"], [axlePt, "M5 axle", color(0.55, 0.57, 0.60), "move"]];
    var belts = [];
    for (var h in hw)
    {
        const q = qContainsPoint(qBodyType(qUnion([qCreatedBy(H, EntityType.BODY), qCreatedBy(HR, EntityType.BODY)]),
                                           BodyType.SOLID), h[0]);
        dress(context, q, h[1], h[2], "Hardware envelope, not a print.");
        if (h[3] == "belt")
            belts = append(belts, q);
        else
            stages[h[3]] = append(stages[h[3]], q);
    }
    if (opt.shift != undefined && opt.shift != 0 * mm)
        opTransform(context, id + "shift", { "bodies" : qUnion(stages["move"]), "transform" : transform(-opt.shift * Y) });
    opMateConnector(context, id + "axleMC", { "coordSystem" : coordSystem(O, X, Z), "owner" : partAt(context, P, fix ? fxPt : csPt) });
    return { "stages" : stages, "belts" : belts, "prints" : prints };
}

%SPLIT%

annotation { "Feature Type Name" : "AOW drive",
             "Feature Type Description" : "Rear drive module: mock wheel, XC430s, pulleys, case sides, chainstays, optional fixture mock; Y along the belt" }
export const aowDrive = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Chainstays back by (belt tension)" }
        isLength(definition.shift, { (millimeter) : [0, 0, %TRAVEL%] } as LengthBoundSpec);
        annotation { "Name" : "Fixture mock (and the case sides' tabs)", "Default" : false }
        definition.fixture is boolean;
        annotation { "Name" : "Side rails, cage stubs and AHRS tabs (v2 bike)", "Default" : true }
        definition.rail is boolean;
    }
    {
        driveBuild(context, id + "build", { "shift" : definition.shift, "fixture" : definition.fixture,
                                            "rail" : definition.rail });
        reportFeatureInfo(context, id, "Belt centres " ~ toString(roundToPrecision(DR.C / millimeter, 2))
                ~ " mm; tension travel " ~ toString(roundToPrecision(DR.travel / millimeter, 2)) ~ " mm");
    });
'''


def build_fs(data: dict, fs_version: str = "3044") -> str:
    L = full_layout(data)
    segs = outlines(data)
    subs = {
        "%VERSION%": fs_version,
        "%DR%": _fs_map("DR", L),
        "%FIXTURE%": _fs_map("FIXTURE", data["s"]["joint"]),
        "%SCREW_OPT%": _fs_map("SCREW_OPT", data["screw"]),
        "%SEGS%": "\n\n".join(_fs_segs(k, v) for k, v in segs.items()),
        "%SERVO_MOUNT%": af._servo_mount_layer(data, fs_version),
        "%HELPERS%": af.FS_HELPERS.rstrip("\n") + "\n",
        "%SPLIT%": SPLIT_MARK,
        "%TRAVEL%": f"{L['travel']:g}",
        # the rails are the v2 bike's, with its snug nut slots; the frozen v1
        # (as-built slots) never builds them
        "%RAIL%": _fs_val(rail_layout(data, L)) if data.get("snug", True) else "{}",
    }
    text = FS
    for k, v in subs.items():
        text = text.replace(k, v)
    return text


def hole_probes(data: dict, L: dict) -> list:
    """Points to test the rails' screw holes by, MODULE frame (mm): three on
    each joint's axis through the rail (want no body: the hole is open) and
    one in the wall beside it (want a body: the rail is there, so an open
    axis is not merely a missing rail). [tag, side, x, y, z, want]."""
    R = rail_layout(data, L)
    hr = data["screw"]["headDia"] / 2
    pieces = [q for q in R["pieces"] + R["leftPieces"]]
    out = []
    for j, sides in [(j, (1, -1)) for j in R["joints"]] + [(j, (1,)) for j in R["leftJoints"]]:
        y, z = j["head"]
        ctl = next(((y + dy * (hr + 0.75), z + dz * (hr + 0.75)) for dy, dz in ((1, 0), (-1, 0), (0, 1), (0, -1))
                    if any(_inside((y + dy * (hr + 0.75), z + dz * (hr + 0.75)), pc)
                           and _edge_dist((y + dy * (hr + 0.75), z + dz * (hr + 0.75)), pc) >= 0.4 for pc in pieces)),
                   None)
        for sg in sides:
            sd = "L" if sg > 0 else "R"
            for dx in (0.6, 2.3, 4.0):
                out.append([j["tag"], sd, -sg * (L["xPo"] - dx), y, z, 0])
            if ctl is not None:
                out.append([j["tag"], sd, -sg * (L["xPo"] - 1.0), ctl[0], ctl[1], 1])
    return out


def probe_fs(probes: list) -> str:
    """The check's FeatureScript for hole_probes (in whatever frame they
    were given): one HOLEP row each, the bodies found at the point."""
    rows = ", ".join("[" + ", ".join(f'"{v}"' if isinstance(v, str) else f"{v:.6g}" for v in pr) + "]" for pr in probes)
    return f"""    const probeSolids = qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID);
    for (var pr in [{rows}])
        println("HOLEP|" ~ pr[0] ~ "|" ~ pr[1] ~ "|" ~ toString(pr[5]) ~ "|"
                ~ size(evaluateQuery(context, qContainsPoint(probeSolids, vector(pr[2], pr[3], pr[4]) * millimeter))));
"""


def probe_verdict(rows: list) -> tuple[bool, str]:
    """Judge the HOLEP rows: want 0 = the hole open, 1 = the wall there."""
    pr = [r for r in rows if r[0] == "HOLEP"]
    bad = [r for r in pr if (int(r[4]) > 0) != (r[3] == "1")]
    txt = (f"  rail screw holes: {sum(r[3] == '0' for r in pr)} axis probes open, "
           f"{sum(r[3] == '1' for r in pr)} wall probes solid, {len(bad)} wrong"
           + "".join(f"\n    {r[1]} {r[2]}: want {'wall' if r[3] == '1' else 'hole'}, found {r[4]} bodies  FAIL"
                     for r in bad))
    return (bool(pr) and not bad), txt


def check_wrapper(fs: str, probes: list = ()) -> str:
    """Build once, then print: every body's X extent and volume, every
    collision at rest, the axle group moved back by the full travel against
    everything fixed (belts left out -- they are drawn at nominal), and the
    print check. ONE call."""
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
    const id = makeId("chk");
    const r = driveBuild(context, id, {{ "debug" : true, "rail" : true }});
    const all = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    for (var b in all)
    {{
        const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
        println("BODY|" ~ name(b) ~ "|" ~ toString(bb.minCorner[0] / millimeter) ~ "|"
                ~ toString(bb.maxCorner[0] / millimeter) ~ "|"
                ~ toString(evVolume(context, {{ "entities" : b }}) / (millimeter * millimeter * millimeter)));
    }}
    // each rail joint's screw hole OPEN on its axis, and the wall beside it
    // solid (hole_probes; the left-only ears' holes were reported missing,
    // user 2026-10-16)
{probe_fs(probes)}    for (var i = 0; i + 1 < size(all); i += 1)
        clashes("rest", all[i], qUnion(subArray(all, i + 1, size(all))));
    const mv = qUnion(r.stages["move"]);
    opTransform(context, id + "back", {{ "bodies" : mv, "transform" : transform(-DR.travel * vector(0, 1, 0)) }});
    clashes("back", mv, qUnion(r.stages["fixed"]));
    opTransform(context, id + "fwd", {{ "bodies" : mv, "transform" : transform(DR.travel * vector(0, 1, 0)) }});
{af.PRINT_CHECK_FS}    return "ran to completion";
}}
"""


def canon(name: str) -> str:
    hits = [k for k in PARTS + HARDWARE if name == k or name.startswith(k + " ")]
    return max(hits, key=len) if hits else name


def extents(L: dict) -> dict[str, tuple[float, float]]:
    """Each part's X extent as built -- what --check measures."""
    e = {"spline pulley L": (-L["outerFace"], -(L["hubFace"] - L["splineLen"])),
         "drive pulley L": (-L["outerFace"], -(L["xHorn"] - L["pinL"])),
         "case side L": (-L["xPo"], -min(L["xRin"], L["skirtTop"])),
         "chainstay L": (-L["xCo"], -L["tongueEnd"])}
    for k in list(e):
        if k.endswith(" L"):
            lo, hi = e[k]
            e[k[:-1] + "R"] = (-hi, -lo)
    return e


def check(text: str, data: dict, target: str | None) -> bool:
    """ONE billable call."""
    from . import onshape

    url = onshape.resolve(target, "check")
    probes = hole_probes(data, full_layout(data)) if data.get("snug", True) else []
    reply = onshape.eval_featurescript(check_wrapper(text, probes), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    Path("traces").mkdir(exist_ok=True)
    Path("traces/drive_check.txt").write_text(console)
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console[-3000:])
        print(onshape.budget_line())
        return False
    ok = judge(console, data)
    print(onshape.budget_line())
    return ok


def judge(console: str, data: dict) -> bool:
    rows = [l.split("|") for l in console.splitlines()]
    L = full_layout(data)
    want = extents(L)
    R = rail_layout(data, L) if data.get("snug", True) else {}
    # the rails' battery stops reach in past the skirt
    if R.get("stops"):
        xin = min(min(st["x"][0] for st in R["stops"]), -want["case side L"][1])
        want["case side L"] = (want["case side L"][0], -xin)
        want["case side R"] = (xin, want["case side R"][1])
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
            print(f"  {n:16} x {lo:8.3f} .. {hi:8.3f}  (want {want[n][0]:8.3f} .. "
                  f"{want[n][1]:8.3f})  {'ok' if good else 'FAIL'}")
    counts = {r[1]: int(r[2]) for r in rows if r[0] == "PART"}
    for n, c in counts.items():
        ok &= c == 1
        if c != 1:
            print(f"  part {n} is {c} bodies  FAIL")
    ok &= len(counts) == len(PARTS)
    if R.get("joints"):
        pok, ptxt = probe_verdict(rows)
        ok &= pok
        print(ptxt)
    colls = [r for r in rows if r[0] == "COLL" and "ABUT" not in r[4]]
    real = [r for r in colls if {canon(r[2]), canon(r[3])} not in INTENDED]
    ok &= not real
    print(f"  interference at nominal and with the chainstays back {L['travel']:g} mm: {len(real)}"
          f"  (+{len(colls) - len(real)} intended: belts through the teeth, the M5 in the wheel, the cone preload)")
    for r in real[:40]:
        print(f"    {r[1]:6} {r[2]}  x  {r[3]}  ({r[4]})")
    extra = bool(R.get("leftPieces") or R.get("leftJoints"))
    for a, b in PAIRS:
        va, vb = vols.get(a), vols.get(b)
        # with the side rails the case sides are MIRRORS (each rail is its
        # side's mirror, the case under it the turned twin): equal volumes,
        # to the kernel's rounding, not one part. The power holder's ears
        # are on the left only, so there the left is the heavier
        mirror = a == "case side L"
        if mirror and extra:
            same = None not in (va, vb) and va - vb > 1.0
            print(f"  {a[:-2]:14} L/R mirror images but for the left's ears ({va}, {vb}; +{(va or 0) - (vb or 0):.1f}): "
                  f"{'ok' if same else 'FAIL'}")
            ok &= same
            continue
        same = None not in (va, vb) and abs(va - vb) < (1e-2 if mirror else 1e-3)
        ok &= same
        print(f"  {a[:-2]:14} L/R {'mirror images' if mirror else 'the same part'} ({va}, {vb}): "
              f"{'ok' if same else 'FAIL'}")
    for r in rows:
        if r[0] == "BED":
            over = sorted((x for x in rows if x[0] == "OVER" and x[1] == r[1]),
                          key=lambda o: -float(o[2]))
            print(f"    {r[1]:16} bed {float(r[2]):7.1f} mm^2, {len(over)} downward faces"
                  + "".join(f"\n        {float(o[2]):7.2f} mm^2, {float(o[3]):6.2f} up, at ({o[4]})"
                            for o in over[:12]))
    crowns = [r for r in rows if r[0] == "CROWN"]
    print(f"  horizontal holes with a flat crown as printed (want a teardrop): {len(crowns)}")
    for r in crowns:
        print(f"    {r[1]:16} r {r[2]} mm, axis through ({r[3]})")
    hangs = [r for r in rows if r[0] == "HANG"]
    print(f"  level edges hanging in the air as printed: {len(hangs)}")
    for r in hangs:
        print(f"    {r[1]:16} {float(r[2]):5.2f} mm long, {float(r[3]):6.2f} up, at ({r[4]})")
    return ok


def report(data: dict) -> str:
    L = full_layout(data)
    rows = [("belt centre distance (45T/15T, 370)", L["C"]), ("servo shafts at Y", L["Ys"]),
            ("servo shafts at Z", L["zs"]),
            ("-- along X, magnitudes each side --", None),
            ("wheel hub face / spline pulley shoulder", L["hubFace"]),
            ("chainstay stub end (60 deg cone foot)", L["xE"]),
            ("horn face", L["xHorn"]), ("case side outer face", L["xPo"]),
            ("pulley inner flange", L["flangeIn"]), ("tooth band", L["teethIn"]),
            ("belt", L["beltX0"]), ("tooth band end", L["teethOut"]),
            ("pulley outer faces", L["outerFace"]), ("M5 clamp face (hex pocket floor)", L["clampX"]),
            ("chainstay inner / outer face", L["xCi"]), ("", L["xCo"]),
            ("-- tension joint --", None),
            ("6-32 head face (in the riser)", L["xH"]),
            ("riser flat on the case side's outer face (the clamp)", L["riserTop"]),
            ("tongue top", L["tongueEnd"]), ("channel floor", L["tongueTop"]),
            ("nut slot", L["xN0"]), ("6-32 tip", L["tip"]),
            ("tip slot floor", L["xN0"] - L["nutSlotT"] - L["tipSlot"]),
            ("case rear block inner face", L["xRin"]),
            ("-- along Y --", None),
            ("case rear edge (tire + clearance)", L["Yc0"]), ("riser", L["Yr0"]),
            ("tension screw", L["Yscr"]), ("riser/tongue front = channel end", L["Yj1"]),
            ("servo backs", L["Yb"]), ("servo fronts", L["Yf"]),
            ("case sides' front (the skirt's wall)", L["Ytab0"]),
            ("front with the fixture's tabs (off)", L["Yfront"])]
    out = [f"  {'' if v is None else f'{v:8.2f}'}  {k}" for k, v in rows]
    out.append(f"  tip {L['tipPast']:.2f} past the nut; countersink web {L['cskWeb']:.2f}; "
               f"hex floor {L['xCo'] - L['clampX']:.2f} up the chainstay (clamp span {2 * L['clampX']:.2f})")
    out.append(f"  riser inside the belt by {L['riserBelt']:.2f}; web {L['web']:.2f}; tip lands "
               f"{L['land15']:.2f}/{L['land45']:.2f} (15T/45T); stub end ring {L['endRing']:.2f}; "
               f"fixture heads {L['fixHead']:.2f} off the pulley; M2 engages {L['m2Engaged']:.1f}")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drive", default=DRIVE_PARAMS)
    ap.add_argument("--params", default=sm.CAD_PARAMS)
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="build in Onshape and check it; ONE billable call, `check` tab")
    ap.add_argument("--push", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="replace a Feature Studio's contents (its own tab only)")
    ap.add_argument("--shot", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="render a Part Studio (default `drive`); ONE billable call")
    ap.add_argument("--view", default="isometric")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the check script instead of spending a call")
    args = ap.parse_args()

    data = load(args.drive, args.params)
    text = build_fs(data, args.fs_version)
    sm.lint_fs(text)
    Path(args.output).write_text(text)
    print(f"wrote {len(text)} chars -> {args.output}")
    print(report(data))
    if args.dry_run:
        print(check_wrapper(text, hole_probes(data, full_layout(data))))
        return
    if args.check is not None:
        if not check(text, data, args.check or None):
            raise SystemExit("check FAILED -- not pushing")
    if args.push is not None:
        from . import onshape
        if args.push != "drive_features":
            raise SystemExit(f"refusing to push at {args.push or 'the default'!r}: "
                             "this generator owns `drive_features` only")
        url = onshape.resolve(args.push, args.push)
        reply = onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}  (microversion "
              f"{reply.get('sourceMicroversion', '?')})")
        print(onshape.budget_line())
    if args.shot is not None:
        from . import onshape
        url = onshape.resolve(args.shot or None, "drive")
        tag = "" if args.view == "isometric" else "_" + args.view
        out, _ = onshape.shaded_view(url, Path(OUT_PNG.format(tag)), view=args.view)
        print(f"rendered -> {out}")
        print(onshape.budget_line())


if __name__ == "__main__":
    main()
