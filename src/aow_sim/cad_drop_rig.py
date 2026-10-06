"""The drop release rig's printed parts, as ONE custom feature, `AOW drop rig`,
in its own Feature Studio. Spec: docs/plans/drop-release-rig.md, "Parts".
Numbers: config/drop_rig_cad.yaml, plus cad_steering's and cad_drive's own
layouts for the two wheel interfaces, plus the X330 envelope and case shells
cad_servo_mount and the X330 fixture use.

RIG FRAME: origin on the drop sensor's button top, +Z up, +X along the arm
toward the hinge, +Y along the wheel's axle. The wheel stands on the button,
axle straight above it; the arm leaves the board over its short edge.

    part              print   what it is
    base              Z+      plate under the PCB (5 standoffs, M3 self-tap
                              pilots, through) and the servo; both legs' 6-32s from
                              below, heads counterbored under flush
    back shell        Y-      the X330 back-half case shell, leg down to the base
    cover             Y+      the horn-half shell, full wrap, leg down to the base
    cam               Y+      bench/drop_cam.py's profile on the horn's pins
    front interface   X+      the fork's headset block (its ledges, cheeks and
                              two 6-32 nut slots, from cad_steering), the bar
                              slot, the follower
    rear interface    X+      both case sides' chainstay joint (channel, screw
                              slot, nut slot, from cad_drive), the bar slot,
                              the follower
    wave cam <name>   Y+      AHRS mode, one per `wave_kit` cam: drop_cam.py's
                              tones on the horn's pins, drawn `wave.offset`
                              along Y, `wave.pitch` apart along -X
    follower tip      Y+      AHRS mode: a jam-on cap over either follower pad
                              with a rounded nose, drawn on the wave cam there

Both interfaces put the follower at the SAME place over the button -- the
front tire and the rear wheel differ by 0.05 mm in radius -- so one base and
one cam serve both. The dialog picks which wheel is on the rig; the other
interface is drawn `spare_offset` along +Y. Everything past the bar slot (the
bar, the hinge) is rigged by hand (user, 2026-10-02).

Drawn NOMINAL: wheel resting on the button, arm level, the follower `gap`
over the cam's dwell. No shims under the servo; height is trimmed at the
axle. Each interface carries its own wheel's radius (the rear's is the
contact under test, `wheels.rear_radius_offset`), and puts its follower at
the same height over the button.

    python -m aow_sim.cad_drop_rig                     # write docs/cad/drop_rig.fs
    python -m aow_sim.cad_drop_rig --check             # both wheels, ONE call
    python -m aow_sim.cad_drop_rig --push drop_rig_features
    python -m aow_sim.cad_drop_rig --shot              # -> docs/cad/drop_rig.png
"""

from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path

import yaml

from . import cad_ahrs_fixture as af
from . import cad_drive as dr
from . import cad_servo_mount as sm
from . import cad_steering as st
from .params import _normalize

RIG_PARAMS = "config/drop_rig_cad.yaml"
X330_PARAMS = "config/x330_fixture_cad.yaml"
CAM_SCRIPT = "bench/drop_cam.py"
OUT_FS = "docs/cad/drop_rig.fs"
OUT_PNG = "docs/cad/drop_rig{}.png"
SPLIT_MARK = af.SPLIT_MARK
WHEELS = ("FRONT", "REAR")
PARTS = ("base", "back shell", "cover", "cam", "front interface", "rear interface",
         "wave cam", "follower tip")
HARDWARE = ("X330 case", "X330 horn", "PCB", "FS20", "front tire", "fork mock",
            "rear wheel", "chainstay mock")
# Contacts that are the design: the screw bores cut into the mocks only.
INTENDED: tuple[set, ...] = ()
CAM_SEGS = 30          # sketch segments per ramp


def _cam_module():
    spec = importlib.util.spec_from_file_location("drop_cam", CAM_SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load(rig_path: str = RIG_PARAMS) -> dict:
    """Everything the rig reads, in MILLIMETRES (degrees for _deg keys)."""
    raw = _normalize(yaml.safe_load(Path(rig_path).read_text()))
    kit = raw.pop("wave_kit", [])                # already mm, a list of cams
    mm = lambda v: v * 1000.0    # noqa: E731
    r = {sec: {k: (v if k.endswith(("_deg", "_n")) else [mm(x) for x in v] if isinstance(v, list) else mm(v))
               for k, v in vals.items()} for sec, vals in raw.items()}
    r["wave_kit"] = [{"name": c["name"], "tones": [tuple(t) for t in c["tones"]]} for c in kit]
    # the rig's origin is the DROP button: move the board under it
    pc = r["pcb"]
    i = int(pc["drop_button_n"]) - 1
    bx, by = pc["buttons_x"][i], pc["buttons_y"][i]
    for k in ("x0", "x1"):
        pc[k] -= bx
    for k in ("y0", "y1"):
        pc[k] -= by
    for k in ("holes_x", "buttons_x"):
        pc[k] = [x - bx for x in pc[k]]
    for k in ("holes_y", "buttons_y"):
        pc[k] = [y - by for y in pc[k]]
    sdata = st.load()
    ddata = dr.load()
    fx = _normalize(yaml.safe_load(Path(X330_PARAMS).read_text()))["fixture"]
    joint = {k: mm(fx[k]) for k in ("joint_plate", "ridge_height", "ridge_flat",
                                    "ridge_clearance", "bridge_layer", "head_recess")}
    return {"r": r, "steer": sdata, "drive": ddata, "S": st.full_layout(sdata),
            "D": dr.full_layout(ddata), "x330": sdata["x330"], "table": sdata["table"],
            "screw": sdata["screw"], "horn_pins": sdata["horn_pins"], "joint": joint}


def layout(data: dict) -> dict:
    """Every derived dimension, rig frame, mm. The FeatureScript builds from
    exactly these (the RIG map). Raises if something does not fit."""
    r, S, D, sv, j = data["r"], data["S"], data["D"], data["x330"], data["joint"]
    t = {k: v * 1000 for k, v in data["table"]["XC330"].items() if k != "hornHoleCount"}
    pc, fs, bs, so, cm, it = (r[k] for k in ("pcb", "fs20", "base", "servo", "cam", "interface"))

    def need(ok: bool, what: str) -> None:
        if not ok:
            raise ValueError(what)

    L: dict = {}
    # ---- the board and the base, down from the button's top
    L["zPcbTop"] = -fs["height"]
    L["zPcbBot"] = L["zPcbTop"] - pc["thickness"]
    L["zBase"] = L["zPcbBot"] - bs["standoff"]
    L["tBase"] = j["joint_plate"] + j["head_recess"]
    L["zBaseBot"] = L["zBase"] - L["tBase"]
    need(L["zPcbBot"] - fs["legs_below"] - L["zBase"] >= 1.0,
         "the FS20 legs reach within 1 mm of the base: a taller standoff")
    # ---- the servo, about its horn face: far end DOWN, horn facing +Y
    side, topW, nestC, botW = (t[k] for k in ("caseSideClearance", "caseTopWall",
                                              "caseNestClearance", "caseBottomWall"))
    far = sv["caseHeight"] - sv["shaftFromEnd"]
    L["shellY0"] = far - t["caseWrapLength"]
    L["shellY1"] = far + side + topW + nestC + botW
    L["botOuter"] = sv["caseWidth"] / 2 + side + topW + nestC + botW
    L["backZ"] = -(sv["hornThickness"] + sv["caseDepth"])
    L["capOut"] = L["backZ"] - t["caseFaceClearance"] - t["caseCapThickness"]
    L["nestTop"] = L["backZ"] + t["caseGripLength"] + t["caseNestLength"]
    L["capOuter"] = -sv["hornThickness"] + t["caseFaceClearance"] + so["cover_cap_thickness"]
    L["xCv0"] = L["nestTop"] + so["cover_leg_gap"]
    L["cvWallY"] = far + side + topW            # the cover's far wall, outside (local y)
    L["legY1"] = L["shellY1"] + so["base_gap"]  # the legs' feet (local y)
    L["Xc"] = so["x"]
    # the horn faces -Y, its body at +Y: looking at the horn (from -Y) the cam
    # turns clockwise, so its material runs +X under the follower. Downstream
    # is +X: the release is the pad's square +X end, the chamfer upstream.
    L["Yh"] = cm["thickness"] / 2 + cm["face_gap"]         # horn face: the cam centred on Y = 0
    L["Zc"] = L["zBase"] + L["legY1"]                     # the shaft
    L["wellT"] = sv["hornThickness"] - t["caseOffset"]
    L["camT"] = cm["thickness"]
    L["camGap"] = cm["face_gap"]
    L["coverGap"] = L["camGap"] - L["capOuter"]
    need(L["coverGap"] >= 0.5, f"the cam's back face is {L['coverGap']:.2f} mm off the cover's cap")
    L["collarR"] = t["collarOuterDia"] / 2
    need(L["collarR"] < cm["r_dwell"], "the horn collar is wider than the cam's dwell")
    # ---- the follower: the resting gap above the dwell
    L["Zf"] = L["Zc"] + cm["r_dwell"] + cm["gap"]
    L["rCamMax"] = cm["r_dwell"] + cm["gap"] + max(cm["drops"])
    L["coverTop"] = L["Zc"] + sv["shaftFromEnd"] + side + topW
    L["padClear"] = L["Zf"] - L["coverTop"]
    need(L["padClear"] >= 0.5, f"the follower is {L['padClear']:.2f} mm over the cover")
    L["padHalfY"] = cm["thickness"] / 2 + it["follower_margin"]
    L["padX0"] = L["Xc"] - it["follower_upstream"]      # the chamfer starts here
    L["padX1"] = L["Xc"] + it["follower_downstream"]    # the release corner
    L["parkClear"] = park_clearance(data)
    need(L["parkClear"] >= 0.5, f"parked, the cam is only {L['parkClear']:.2f} mm under the follower")
    wave_layout(data, L, need)
    # ---- the wheels: axle heights, and the cam's clearance to each
    L["RaF"] = S["R"]
    L["RaR"] = data["drive"]["wheel"]["R"] + r["wheels"]["rear_radius_offset"]   # the contact under test
    for k in ("RaF", "RaR"):
        cl = math.hypot(L["Xc"], L[k] - L["Zc"]) - L[k] - L["rCamMax"]
        need(cl >= 5.0, f"the cam is {cl:.1f} mm off the {k} wheel")
        L[f"camClear{k[-1]}"] = cl
    # ---- the interfaces: block +X end, the bar housing flush with the
    # block's top, the web and pad under it. They print X+ (the block's -X
    # face on the bed), so the web's underside rises from the pad's UPSTREAM
    # end at 45 deg until it meets the block's underside. The bar slot's
    # floor is the SAME distance from the axle in both, so the bar bottomed
    # in either puts its axle -- and the follower -- in the same place.
    # block's top over the axle, BOTH wheels: the rear block takes the fork
    # block's half depth, which clears its channels and nut slots
    L["hTop"] = S["blockHalf"]
    need(L["hTop"] - max(data["D"]["tongueHalf"] + data["D"]["tongueClr"], data["D"]["nutHalfY"]) >= 2.0,
         "the rear block is under 2 mm round its channel and nut slot")
    L["barH"], L["barW"] = it["bar_height"], it["bar_width"]
    L["hWall"] = it["housing_wall"]
    L["housingH"] = L["barH"] + 2 * L["hWall"]
    L["housingHalfW"] = L["barW"] / 2 + L["hWall"]
    L["slotFloor"], L["slotDepth"] = it["slot_floor"], it["slot_depth"]
    L["endF"] = S["blockTop"]
    L["endR"] = D["Yb"] - D["skClr"]
    L["slotX0"] = max(L["endF"], L["endR"]) + L["slotFloor"]
    need(L["padX1"] <= L["slotX0"] + L["slotDepth"], "the follower runs past the bar housing")
    # the fork's nut slots run along X through the whole front interface
    fork_slot_in = S["xBlk"] + j["joint_plate"] - (j["joint_plate"] + 2.0) - data["screw"]["nutSlotThickness"]
    L["forkSlotIn"] = fork_slot_in
    need(L["housingHalfW"] < fork_slot_in - 0.1, "the bar housing reaches the fork's nut slots")
    need(L["padHalfY"] < fork_slot_in, "the follower web reaches the fork's nut slots")
    for k, x0 in (("F", S["blockBot"]), ("R", D["Yc0"])):
        xs = L["padX0"] - (L["RaF" if k == "F" else "RaR"] - L["hTop"] - L["Zf"])
        need(x0 <= xs < L["end" + k] - 1.0, f"the {k} web's 45 deg chamfer misses the block's underside")
        L["webX0" + k] = xs
    L["spare"] = it["spare_offset"]
    L["Rr"] = L["RaR"]
    L["rimHalfR"] = D["rimHalf"]
    # ---- every OTHER button clear of the wheel standing on the drop one
    L["buttonClear"] = 1e9
    for x, y in zip(pc["buttons_x"], pc["buttons_y"]):
        if abs(x) < 1e-6 and abs(y) < 1e-6:
            continue
        for R, hw in ((S["R"], S["tireW"] / 2), (L["RaR"], D["rimHalf"])):
            if abs(y) - fs["button_dia"] / 2 < hw and abs(x) < R:
                L["buttonClear"] = min(L["buttonClear"], R - math.sqrt(R * R - x * x))
    need(L["buttonClear"] >= 3.0, f"a wheel passes {L['buttonClear']:.2f} mm over another button")
    # ---- the base's outline: the board and the servo's legs, plus a rim
    L["bx0"] = pc["x0"] - bs["rim"]
    L["bx1"] = L["Xc"] + L["botOuter"] + bs["rim"]
    L["by0"] = pc["y0"] - bs["rim"]
    L["by1"] = max(pc["y1"], L["Yh"] - L["capOut"]) + bs["rim"]
    L["soR"], L["pilotR"] = bs["standoff_dia"] / 2, bs["pilot_dia"] / 2
    L["pilotBot"] = L["zBaseBot"] - 1.0         # THROUGH: a tap can run all the way (user)
    return L


def full_layout(data: dict) -> dict:
    """layout() plus the plain config and borrowed values the FeatureScript reads."""
    r, S, D = data["r"], data["S"], data["D"]
    L = layout(data)
    pc, fs = r["pcb"], r["fs20"]
    L.update({"px0": pc["x0"], "px1": pc["x1"], "py0": pc["y0"], "py1": pc["y1"],
              "pcbHoleR": 1.63, "fsHalfX": fs["body_x"] / 2, "fsY0": fs["body_y0"],
              "fsY1": fs["body_y1"], "buttonR": fs["button_dia"] / 2,
              "head_recess": data["joint"]["head_recess"]})
    # the front block and fork, borrowed from cad_steering (module -> rig:
    # X = module Z, Y = module X, Z = axle + module Y)
    for k in ("blockBot", "blockTop", "xBlk", "blockHalf", "zFS", "forkPocket", "xIn", "xOut",
              "forkHalf", "cheekIn", "legHalf", "tireW"):
        L["S_" + k] = S[k]
    L["S_R"] = S["R"]
    # the rear joint, borrowed from cad_drive (module -> rig: X = module Y,
    # Y = module X, Z = axle + module Z)
    for k in ("xPo", "Yc0", "Yj1", "Yr0", "Yscr", "travel", "tongueTop", "tongueEnd", "tongueHalf",
              "tongueClr", "xN0", "nutSlotT", "tipSlot", "holeR", "nutHalfY", "nutCornerY",
              "xCi", "xCo", "armHalf", "csBossR"):
        L["D_" + k] = D[k]
    return L


def cam_points(data: dict) -> list[tuple[float, float]]:
    """drop_cam.py's own outline, as looking AT the horn (x right, y up), mm."""
    cm = data["r"]["cam"]
    pts = _cam_module().profile(cm["r_dwell"], cm["gap"], cm["drops"], cm["dwell_deg"],
                                cm["undercut_deg"], cm["top_deg"], n=CAM_SEGS)
    # PARKED as a run starts (plan, "Running it"): the follower (+y, looking at
    # the horn) park_deg into the first segment's dwell, the 2 mm step just passed and
    # the 0.5 mm drop next
    a = math.radians(90.0 - data["r"]["interface"]["park_deg"])
    pts = [(x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)) for x, y in pts]
    out = []
    for p in pts:                          # drop exact repeats: a zero-length segment
        if not out or math.dist(p, out[-1]) > 1e-4:
            out.append(p)
    if math.dist(out[0], out[-1]) < 1e-4:
        out.pop()
    return out


def wave_layout(data: dict, L: dict, need) -> None:
    """AHRS mode: the follower tip over the pad and the wave cam it rides.
    Rig frame, mm. The tip's nose bottom sits `nose_below` under the pad's
    face, so the wave is drawn for a follower resting that much lower."""
    r = data["r"]
    wv, tp, cm = r["wave"], r["tip"], r["cam"]
    L["tipT"], L["tipW"], L["tipH"], L["tipFit"] = tp["plate"], tp["wall"], tp["height"], tp["fit"]
    L["noseR"] = tp["nose_width"] / 2
    L["noseH"] = tp["nose_height"]
    need(L["noseH"] > L["noseR"], "the nose is shorter than its round end")
    L["noseBelow"] = L["tipT"] + L["noseH"]
    L["waveRest"] = L["Zf"] - L["Zc"] - L["noseBelow"]     # nose bottom over the cam axis, wheel resting
    L["waveLift"] = wv["lift_min"]
    L["waveT"] = wv["thickness"]
    L["waveOff"] = wv["offset"]
    L["wavePitch"] = wv["pitch"]
    kit = r["wave_kit"]
    need(len(kit) > 0, "the wave kit is empty")
    need(0 <= int(wv["tip_on_n"]) < len(kit), "wave.tip_on_n is not a cam in the kit")
    # in place, over the pad; z of the pad face as drawn: lifted onto a valley
    # plus 0.05, so polygon chords cannot touch
    L["tipZl"] = L["Zf"] + L["waveLift"] + 0.05
    L["tipZb"] = L["tipZl"] - L["tipT"]
    L["tipZt"] = L["tipZl"] + L["tipH"]
    L["tipXd"] = L["padX1"] + L["tipFit"] + L["tipW"]       # downstream outer face
    L["tipXi0"] = L["padX0"] - L["tipFit"] * math.sqrt(2)   # the inner chamfer's foot
    L["tipXoB"] = L["tipXi0"] - L["tipW"] * math.sqrt(2) + L["tipT"]
    L["tipXoT"] = L["tipXi0"] - L["tipW"] * math.sqrt(2) - L["tipH"]
    L["tipYi"] = L["padHalfY"] + L["tipFit"]
    L["tipYo"] = L["tipYi"] + L["tipW"]
    L["noseYn"] = wv["thickness"] / 2 + tp["nose_margin"]   # the nose's +Y end; -Y it runs to -tipYo
    # the cover's cap face, +Y of the cam: the nose stops short of it, and the
    # plate and walls pass over its top -- checked with the wheel RESTING, so
    # the tip clears the rig whether or not the cam holds it up
    L["coverY"] = L["Yh"] - L["capOuter"]
    need(L["noseYn"] <= L["coverY"] - 0.5, "the nose's +Y end is within 0.5 mm of the cover")
    L["tipCoverClear"] = L["Zf"] - L["tipT"] - L["coverTop"]
    need(L["tipCoverClear"] >= 0.3, f"resting, the tip is {L['tipCoverClear']:.2f} mm over the cover")
    # every cam in the kit: drop_cam.py's checks, the horn collar, the drop
    # cam's envelope, and the tip's plate away from the nose
    dc = _cam_module()
    L["tipCamClear"] = 1e9
    for c in kit:
        lines, ok = dc.wave_report(L["waveRest"], L["noseR"], L["waveLift"], c["tones"],
                                   5.0, 618.0, 124.0, 205.0)
        need(ok, f"wave cam {c['name']} fails drop_cam.py's checks: " + " ".join(lines))
        radii = [math.hypot(*q) for q in wave_outline(data, c["tones"])]
        need(min(radii) - L["collarR"] >= 0.5, f"wave cam {c['name']} is within 0.5 mm of the horn collar")
        need(max(radii) <= L["rCamMax"], f"wave cam {c['name']} is outside the drop cam's envelope")
        L["tipCamClear"] = min(L["tipCamClear"], tip_cam_clearance(data, L, c["tones"]))
    need(L["tipCamClear"] >= 0.5, f"a wave cam comes {L['tipCamClear']:.2f} mm under the tip's plate")


def wave_outline(data: dict, tones, n_per_lobe: int = 48) -> list[tuple[float, float]]:
    """A wave cam in drop_cam.py's frame, mm. Coarser than drop_cam.py's own
    (48 points a lobe, >= 360): every point is a sketch segment in Onshape."""
    wv, tp, cm = data["r"]["wave"], data["r"]["tip"], data["r"]["cam"]
    rest = cm["r_dwell"] + cm["gap"] - tp["plate"] - tp["nose_height"]
    n = max(360, n_per_lobe * max((t[0] for t in tones), default=1))
    return _cam_module().wave_profile(rest, tp["nose_width"] / 2, wv["lift_min"], list(tones), n=n)


def wave_points(data: dict, tones) -> list[tuple[float, float]]:
    """A wave cam as looking AT the horn (x right, y up), mm, turned so its
    lowest point is under the follower (+y), where the tip is drawn."""
    a = math.pi / 2 - _cam_module().valley_angle(list(tones))
    return [(x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))
            for x, y in wave_outline(data, tones)]


def tip_cam_clearance(data: dict, L: dict, tones) -> float:
    """Worst gap between a wave cam and the tip's underside away from the
    nose -- the plate, then its outer 45 deg chamfer upstream -- over a turn,
    the nose riding the cam (radial follower)."""
    import numpy as np
    R = _cam_module().wave_pitch(L["waveRest"], L["noseR"], L["waveLift"], list(tones))[0]
    P = np.array(wave_outline(data, tones))
    worst = 1e9
    x0, x1 = L["tipXoB"] - L["Xc"], L["tipXd"] - L["Xc"]
    for phi in np.linspace(0, 2 * np.pi, 721):
        zb = R(phi) - L["noseR"] + L["noseH"]               # plate bottom over the cam axis
        a = np.pi / 2 - phi                                  # cam angle phi under +y
        x = P[:, 0] * np.cos(a) - P[:, 1] * np.sin(a)
        y = P[:, 0] * np.sin(a) + P[:, 1] * np.cos(a)
        m = (x <= x1) & (y > 0)
        under = np.where(x[m] >= x0, zb, zb + (x0 - x[m]))
        worst = min(worst, float((under - y[m]).min()))
    return worst


def park_clearance(data: dict) -> float:
    """Parked park_deg past each step, how far the cam stays under the
    follower (worst segment): the pad, and upstream of it the web's 45 deg
    chamfer over the incoming ramp; downstream, past the release corner,
    nothing. Analytic cam:
    dwell, linear ramp, top flat, the previous top flat behind the step."""
    import numpy as np
    cm, it = data["r"]["cam"], data["r"]["interface"]
    drops, r0, gap = cm["drops"], cm["r_dwell"], cm["gap"]
    dw, tp = cm["dwell_deg"], cm["top_deg"]
    ramp = 360.0 / len(drops) - dw - tp
    rest = r0 + gap
    worst = -1e9
    dl = np.radians(np.linspace(-45, 45, 9001))
    for i, h in enumerate(drops):
        psi = it["park_deg"] + np.degrees(dl)
        rr = np.where(psi < 0, rest + drops[i - 1],
                      np.where(psi < dw, r0, np.where(psi < dw + ramp,
                               r0 + (gap + h) * (psi - dw) / ramp, rest + h)))
        x, y = -rr * np.sin(dl), rr * np.cos(dl)     # +x downstream
        # the follower's underside: flat over the pad, then the 45 deg chamfer
        face = rest + np.clip(-it["follower_upstream"] - x, 0.0, None)
        m = x <= it["follower_downstream"]
        worst = max(worst, float((y - face)[m].max()))
    return -worst


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


def _fs_array(name: str, rows: list) -> str:
    body = ",\n".join("    vector(" + ", ".join(f"{v:.6g}" for v in row) + ") * millimeter"
                      for row in rows)
    return f"export const {name} = [\n{body}\n];"


def _fs_waves(data: dict) -> str:
    """The kit: WAVES[i] the outline, WAVE_NAMES[i] its part name, WAVE_TIP the
    cam the tip is drawn on."""
    kit = data["r"]["wave_kit"]
    arrays = ",\n".join("    [" + ", ".join(f"vector({x:.4f}, {y:.4f}) * millimeter" for x, y in wave_points(data, c["tones"]))
                         + "]" for c in kit)
    names = ", ".join(f'"wave cam {c["name"]}"' for c in kit)
    return (f"export const WAVES = [\n{arrays}\n];\nexport const WAVE_NAMES = [{names}];\n"
            f"export const WAVE_TIP = {int(data['r']['wave']['tip_on_n'])};")


FS = r'''FeatureScript %VERSION%;
import(path : "onshape/std/geometry.fs", version : "%VERSION%.0");

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

%X330%

%RIG%

%CAM%

%WAVE%

%PCB_HOLES%

%BUTTONS%

%FIXTURE%

%HORN_OPT%

%CASE_OPT%

%SCREW_OPT%

// ---- the servo-mount geometry, copied from the horn-mount-gen studio ----
%SERVO_MOUNT%
// ---- end of the copy ----

%HELPERS%
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

%SPLIT%

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
'''


def build_fs(data: dict, fs_version: str = "3044") -> str:
    t = data["table"]["XC330"]
    pick = lambda keys: {"servo": "XC330", **{k: t[k] * 1000 for k in keys}}  # noqa: E731
    pc = data["r"]["pcb"]
    subs = {
        "%VERSION%": fs_version,
        "%X330%": _fs_map("X330", data["x330"]),
        "%RIG%": _fs_map("RIG", full_layout(data)),
        "%CAM%": _fs_array("CAM", cam_points(data)),
        "%WAVE%": _fs_waves(data),
        "%PCB_HOLES%": _fs_array("PCB_HOLES", list(zip(pc["holes_x"], pc["holes_y"]))),
        "%BUTTONS%": _fs_array("BUTTONS", list(zip(pc["buttons_x"], pc["buttons_y"]))),
        "%FIXTURE%": _fs_map("FIXTURE", data["joint"]),
        "%HORN_OPT%": _fs_map("HORN_OPT", st.horn_opt(data)),
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


def check_wrapper(fs: str, wheels=WHEELS) -> str:
    """Build each wheel's rig, then print every body's box and volume, every
    collision, and the print check. Both built, checked and deleted in ONE call."""
    ws = "[" + ", ".join(f'"{w}"' for w in wheels) + "]"
    return f"""function(context is Context, queries)
{{
{sm.geometry_layer(fs, SPLIT_MARK)}
    const name = function(q) returns string
    {{
        const n = getProperty(context, {{ "entity" : q, "propertyType" : PropertyType.NAME }});
        return n == undefined ? "UNNAMED" : n;
    }};
    const v3 = function(p) returns string
    {{
        return toString(roundToPrecision(p[0] / millimeter, 3)) ~ "|" ~ toString(roundToPrecision(p[1] / millimeter, 3))
               ~ "|" ~ toString(roundToPrecision(p[2] / millimeter, 3));
    }};
    for (var w in {ws})
    {{
    println("VER=" ~ w);
    const id = makeId("chk" ~ w);
    const r = dropRigBuild(context, id, {{ "wheel" : w, "debug" : true }});
    const all = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    for (var b in all)
    {{
        const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
        println("BODY|" ~ name(b) ~ "|" ~ v3(bb.minCorner) ~ "|" ~ v3(bb.maxCorner) ~ "|"
                ~ toString(roundToPrecision(evVolume(context, {{ "entities" : b }}) / (millimeter ^ 3), 1)));
    }}
    for (var i = 0; i + 1 < size(all); i += 1)
        for (var c in evCollision(context, {{ "tools" : all[i], "targets" : qUnion(subArray(all, i + 1, size(all))) }}))
            println("COLL|rest|" ~ name(c.toolBody) ~ "|" ~ name(c.targetBody) ~ "|" ~ toString(c["type"]));
{af.PRINT_CHECK_FS}    opDeleteBodies(context, id + "clear", {{ "entities" : qCreatedBy(id, EntityType.BODY) }});
    }}
    return "ran to completion";
}}
"""


def canon(name: str) -> str:
    base = name.split(" [print")[0]
    hits = [k for k in PARTS + HARDWARE if base == k or base.startswith(k + " ")]
    return max(hits, key=len) if hits else base


def judge(console: str, data: dict, wheel: str) -> bool:
    rows = [l.split("|") for l in console.splitlines()]
    L = full_layout(data)
    ok = True
    bodies = [r for r in rows if r[0] == "BODY"]
    strays = [r for r in bodies if r[1] == "UNNAMED"]
    ok &= not strays
    if strays:
        print(f"  {len(strays)} UNNAMED bodies -- a cutter left behind  FAIL")
        for r in strays[:6]:
            print(f"    {r[2:]}")
    vols = {}
    for r in bodies:
        n = canon(r[1])
        lo = tuple(map(float, r[2:5]))
        hi = tuple(map(float, r[5:8]))
        if n in PARTS:
            vols[n] = float(r[8])
            print(f"  {n:16} x {lo[0]:8.2f} .. {hi[0]:8.2f}  y {lo[1]:8.2f} .. {hi[1]:8.2f}"
                  f"  z {lo[2]:8.2f} .. {hi[2]:8.2f}  {float(r[8]) * st.PLA:6.1f} g")
    # the follower's face and the cam's dwell, as drawn
    for r in bodies:
        n = canon(r[1])
        if n == ("front interface" if wheel == "FRONT" else "rear interface"):
            got = float(r[4])
            good = abs(got - L["Zf"]) < 1e-3
            ok &= good
            print(f"  follower face z {got:.3f} (want {L['Zf']:.3f})  {'ok' if good else 'FAIL'}")
    counts = {r[1]: int(r[2]) for r in rows if r[0] == "PART"}
    for n, c in counts.items():
        ok &= c == 1
        if c != 1:
            print(f"  part {n} is {c} bodies  FAIL")
    want = len(PARTS) - 1 + len(data["r"]["wave_kit"])     # "wave cam" is one per kit cam
    ok &= len(counts) == want
    if len(counts) != want:
        print(f"  {len(counts)} printed parts, want {want}  FAIL")
    colls = [r for r in rows if r[0] == "COLL" and "ABUT" not in r[4]]
    real = [r for r in colls if {canon(r[2]), canon(r[3])} not in INTENDED]
    ok &= not real
    print(f"  interference at rest: {len(real)}")
    for r in real[:40]:
        print(f"    {r[2]}  x  {r[3]}  ({r[4]})")
    for r in rows:
        if r[0] == "BED":
            over = sorted((x for x in rows if x[0] == "OVER" and x[1] == r[1]),
                          key=lambda o: -float(o[2]))
            print(f"    {r[1]:16} bed {float(r[2]):7.1f} mm^2, {len(over)} downward faces"
                  + "".join(f"\n        {float(o[2]):7.2f} mm^2, {float(o[3]):6.2f} up, at ({o[4]})"
                            for o in over[:8]))
    crowns = [r for r in rows if r[0] == "CROWN"]
    print(f"  horizontal holes with a flat crown as printed (want a teardrop): {len(crowns)}")
    for r in crowns:
        print(f"    {r[1]:16} r {r[2]} mm, axis through ({r[3]})")
    hangs = [r for r in rows if r[0] == "HANG"]
    print(f"  level edges hanging in the air as printed: {len(hangs)}")
    for r in hangs:
        print(f"    {r[1]:16} {float(r[2]):5.2f} mm long, {float(r[3]):6.2f} up, at ({r[4]})")
    return ok


def check(text: str, data: dict, target: str | None, wheels=WHEELS) -> bool:
    """ONE billable call for both wheels."""
    from . import onshape

    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text, wheels), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    Path("traces").mkdir(exist_ok=True)
    Path("traces/drop_rig_check.txt").write_text(console)
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console[-3000:])
        print(onshape.budget_line())
        return False
    sections = console.split("VER=")[1:]
    ok = len(sections) == len(wheels)
    if not ok:
        print(f"  ran {len(sections)} of {len(wheels)} wheels -- FAIL")
    for sec in sections:
        ver, _, body = sec.partition("\n")
        print(f"--- {ver}")
        ok &= judge(body, data, ver.strip())
    print(onshape.budget_line())
    return ok


def report(data: dict) -> str:
    """The stack, up from the base, for the terminal and the plan doc."""
    L = full_layout(data)
    rows = [("zBaseBot", "base underside"), ("zBase", "base top: the legs and the standoffs"),
            ("zPcbBot", "PCB underside"), ("zPcbTop", "PCB top"),
            ("Zc", "cam centre / XL330 shaft"), ("coverTop", "cover's top"),
            ("Zf", "follower face, wheel resting"),
            ("RaR", "rear axle (the contact under test)"), ("RaF", "front axle (tire OD / 2)")]
    out = [f"  {L[k]:7.2f}  {what}" for k, what in rows]
    out.append(f"  follower over the cover {L['padClear']:.2f} mm; cam to the "
               f"tire {L['camClearF']:.1f} (front) / {L['camClearR']:.1f} (rear) mm; parked, the cam "
               f"{L['parkClear']:.2f} mm under the follower (worst segment)\n"
               f"  cam back face {L['coverGap']:.2f} mm off the cover's cap; other buttons "
               f"{L['buttonClear']:.1f} mm under the wheels; base {L['bx1'] - L['bx0']:.0f} x "
               f"{L['by1'] - L['by0']:.0f} mm\n"
               f"  AHRS mode: tip nose {2 * L['noseR']:.2f} wide, {L['noseBelow']:.2f} under the pad; "
               f"over the cover {L['tipCoverClear']:.2f} (resting), over the wave cams "
               f">= {L['tipCamClear']:.2f} mm (plate, in use); {len(data['r']['wave_kit'])} cams drawn "
               f"{L['waveOff']:.0f} mm along Y\n"
               f"  cam centre {L['Xc']:.1f} mm along the arm from the button; bar slot "
               f"{L['barH']:.2f} x {L['barW']:.2f} x {L['slotDepth']:.1f}, its centre "
               f"{L['hTop'] - L['hWall'] - L['barH'] / 2:.2f} mm over the axle")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rig", default=RIG_PARAMS)
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="build both wheels in Onshape and check them; ONE billable "
                         "call, defaults to the `check` tab")
    ap.add_argument("--push", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="replace a Feature Studio's contents (its own tab only)")
    ap.add_argument("--shot", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="render a Part Studio (default `drop_rig`); ONE billable call")
    ap.add_argument("--view", default="isometric")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the check script instead of spending a call")
    args = ap.parse_args()

    data = load(args.rig)
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
        if args.push != "drop_rig_features":
            raise SystemExit(f"refusing to push at {args.push or 'the default'!r}: "
                             "this generator owns `drop_rig_features` only")
        url = onshape.resolve(args.push, args.push)
        reply = onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}  (microversion "
              f"{reply.get('sourceMicroversion', '?')})")
        print(onshape.budget_line())
    if args.shot is not None:
        from . import onshape
        url = onshape.resolve(args.shot or None, "drop_rig")
        tag = "" if args.view == "isometric" else "_" + args.view
        out, _ = onshape.shaded_view(url, Path(OUT_PNG.format(tag)), view=args.view)
        print(f"rendered -> {out}")
        print(onshape.budget_line())


if __name__ == "__main__":
    main()
