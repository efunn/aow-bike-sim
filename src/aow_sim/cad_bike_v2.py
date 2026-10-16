"""aow-bike-whole-v2: the drive, steering-v2 and righting modules placed in
the bike frame, the electronics as free-floating envelopes, and NO chassis
yet. ONE custom feature, `AOW bike v2`, in its own Feature Studio. Spec:
docs/plans/aow-bike-whole-v2.md. Numbers: config/bike_v2_cad.yaml, plus
each module's own generator and config.

v1 (`aow_sim.cad_bike`, `aow-bike-whole`) grew a chassis, tray, carrier and
cage round a placement fitted constraint by constraint; v2 starts again with
only the modules and the parts that must be carried, so the layout can be
found first and the structure designed round it.

BIKE FRAME: origin on the rear axle, +Y forward, +X right, +Z up. As v1:
the drive turned up by drivetrain.drive_servo_angle_deg about X; the
steering at the front axle, raked by bike.rake_deg; the righting moved to
its station (its own floor mock and chassis-plate placeholder dropped).

    python -m aow_sim.cad_bike_v2                       # docs/cad/bike_v2.fs
    python -m aow_sim.cad_bike_v2 --check               # ONE call
    python -m aow_sim.cad_bike_v2 --push bike_v2_features

The check: every body, interference between modules and the envelopes at
rest, steered 30/90/180/270, and the righting at every pose (each module's
own internal clashes are its own check's business), and the envelopes'
bounding boxes. Moving a component is an edit to the config; or move it in
the browser with a Transform after the feature, and say where it went.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml

from . import cad_ahrs_fixture as af
from . import cad_bike as cb
from . import cad_drive as cd
from . import cad_pi_carrier as cpc
from . import cad_power as cpw
from . import cad_righting as cr
from . import cad_servo_mount as sm
from . import cad_steering as cs
from .params import _normalize, load_params

PARAMS = "config/bike_v2_cad.yaml"
OUT_FS = "docs/cad/bike_v2.fs"
SPLIT_MARK = af.SPLIT_MARK
TAB = "bike_v2_features"
PART_STUDIO = "bike_v2"
POSES = cb.POSES
STEER_SWEEP = cb.STEER_SWEEP
COMPONENTS = ("battery", "ahrs", "pi", "u2d2")      # + the power board and switch, from cad_power
# the USB-C plug's overmold, a typical one (ahrs_fixture_cad.yaml
# usb_plug_keepout, GUESS): length past the receptacle, width, height, m
USB_KEEPOUT = (0.030, 0.0124, 0.0065)
NAMES = {"battery": "battery", "ahrs": "AHRS", "pi": "Pi 3B+", "u2d2": "U2D2", "power": "power board",
         "switch": "switch"}
COLORS = {"battery": (0.85, 0.75, 0.2), "ahrs": (0.55, 0.15, 0.15), "pi": (0.1, 0.5, 0.2),
          "u2d2": (0.2, 0.2, 0.25), "power": (0.15, 0.4, 0.15)}


def module_texts() -> dict[str, str]:
    """Each module's Feature Studio as v2 builds it, from local config now."""
    rd = cr.load()
    return {"steering": cs.build_fs(cs.load(snug=True), v2=True),
            "drive": cd.build_fs(cd.load()),
            "righting": cr.build_fs(cr.layout(rd), rd)}


def load(path: str = PARAMS) -> dict:
    raw = _normalize(yaml.safe_load(Path(path).read_text()))
    rd = cr.load()
    sd, dd = cs.load(snug=True), cd.load()
    D = cd.full_layout(dd)
    RA = cd.rail_layout(dd, D)
    return {"s": raw, "params": load_params(sm.CAD_PARAMS), "steer": cs.layout(sd), "steerData": sd,
            "drive": D, "driveData": dd, "rail": RA, "rd": rd, "rL": cr.layout(rd),
            "power": cpw.layout(raw["power"], D, RA, dd, raw["components"]["battery"]),
            "piCarrier": cpc.layout(raw["pi_carrier"], D, raw["pi"], raw["components"],
                                    _ahrs_top(raw["components"]["ahrs"]), _ahrs_housing_end(raw, dd))}


def _ahrs_housing_end(raw: dict, dd: dict) -> float:
    """The drive z of the AHRS housing's upper end (the U2D2's JSTs face the
    drive past it)."""
    c = raw["components"]["ahrs"]
    h = [v / 2 for v in c["size"]]
    hb = next(b for b in tm151_boxes(dd["tm151"], h, int(dd["s"]["ahrs"]["tm151_x_up_n"]))
              if b["name"] == "AHRS housing")
    return c["at"][2] + hb["hi"][2]


def _ahrs_top(c: dict) -> float:
    """The drive y of the AHRS envelope's outer face (its housing's top)."""
    return c["at"][1] + c["size"][1] / 2


def layout(data: dict) -> dict:
    s, p, S, rL = data["s"], data["params"], data["steer"], data["rL"]
    pl = s["placement"]
    R = p["omni_wheel"]["outer_radius"] * 1000
    tilt = p["drivetrain"]["drive_servo_angle_deg"]
    rake = p["bike"]["rake_deg"]
    yr, front = pl["righting_y"], pl["front"]
    wb = yr + front
    zr = -R - rL["floor_z"]
    zf = S["R"] - R
    L = {"wb": wb, "yr": yr, "zr": zr, "zf": zf, "tilt": tilt, "rake": rake, "R": R, "floor": -R,
         "sOrigin": [0.0, wb, zf], "sAxis": cb.rx([0, 0, 1], rake)}
    # the blades clear the front tyre's whole steer sweep at every pose
    room = cb.wing_front_room(rL, front, zf - zr, S["R"])
    for slug, r in room.items():
        if r["now"] > r["max"] - pl["wing_margin"]:
            raise ValueError(f"{slug}'s front end (module y {r['now']:.1f}) reaches the front tyre's steer "
                             f"sweep: placement.front >= {front + r['now'] - r['max'] + pl['wing_margin']:.1f}")
    L["wingRoom"] = {k: v["max"] - v["now"] for k, v in room.items()}
    # the blades centred on the wheelbase (user, 2026-10-11): their span is
    # the righting module's (wing.blade_y), so a different wheelbase or rod
    # station needs a different span there
    by = data["rd"]["s"]["wing"].get("blade_y")
    if data["rd"]["s"]["wing"]["blade"] == "outline":
        mid = wb / 2 - yr
        half = (by[1] - by[0]) / 2
        if abs((by[0] + by[1]) / 2 - mid) > 0.25:
            raise ValueError(f"the blades are not centred on the wheelbase ({wb:g}, mid {wb / 2:g}): "
                             f"righting_cad.yaml wing.blade_y [{mid - half:g}, {mid + half:g}]")
        L["bladeY"] = [by[0] + yr, by[1] + yr]
    # the righting's parts v2 leaves out, found by a point inside each: its
    # floor, the old chassis plate and the side rails' placeholders (the
    # drive's rails are the real ones)
    L["rgDrop"] = [q["anchor"] for q in rL["parts"] if q["slug"] in ("floor", "chassis", "railR", "railL")]
    L["railCheck"] = rail_joints(data, L)
    comps = []
    for k in COMPONENTS:
        c = s["components"][k]
        if c.get("at") is None:
            continue                     # parked: not drawn (bike_v2_cad.yaml says why)
        if c["frame"] not in ("bike", "drive", "steer"):
            raise ValueError(f"components.{k}.frame: bike, drive or steer, not {c['frame']!r}")
        h = [v / 2 for v in c["size"]]
        tm = data["driveData"]["tm151"]
        sx = int(data["driveData"]["s"]["ahrs"]["tm151_x_up_n"])
        boxes = (pi_boxes(s["pi"], h) if k == "pi" else tm151_boxes(tm, h, sx) if k == "ahrs"
                 else u2d2_boxes(h) if k == "u2d2"
                 else [{"name": NAMES[k], "lo": [-v for v in h], "hi": h, "color": list(COLORS[k])}])
        holes = pi_holes(s["pi"], h) if k == "pi" else tm151_holes(tm, h, sx) if k == "ahrs" else None
        comps.append({"key": k, "name": NAMES[k], "frame": c["frame"], "size": list(c["size"]),
                      "at": list(c["at"]), "rx": float(c.get("rx") or 0.0), "mass": c.get("mass"),
                      "boxes": boxes, "holes": holes})
    # the power board and the switch, on the carrier (cad_power), drive frame
    P = data["power"]
    for k, src, holes in (("power", P["board"]["boxes"], P["board"]["holes"]), ("switch", P["switch"]["boxes"], None)):
        lo = [min(b["lo"][i] for b in src) for i in range(3)]
        hi = [max(b["hi"][i] for b in src) for i in range(3)]
        at = [(lo[i] + hi[i]) / 2 for i in range(3)]
        rel = [{**b, "lo": [b["lo"][i] - at[i] for i in range(3)], "hi": [b["hi"][i] - at[i] for i in range(3)]}
               for b in src]
        hr = None if holes is None else {"ax": "x", "r": holes["r"], "y": [v - at[0] for v in holes["x"]],
                                         "at": [[y - at[1], z - at[2]] for y, z in holes["at"]]}
        comps.append({"key": k, "name": NAMES[k], "frame": "drive", "size": [hi[i] - lo[i] for i in range(3)],
                      "at": at, "rx": 0.0, "mass": None, "boxes": rel, "holes": hr})
    L["comps"] = comps
    keys = ("adds", "cuts", "cyls", "joints", "pt")
    L["carriers"] = [{"name": "power holder", "color": [0.93, 0.56, 0.20],
                      "note": "Printed -X (its battery-side face on the bed). The power board, the switch in the "
                              "shelf, strap slits; two 6-32 into the left rail.", **{k: P[k] for k in keys}},
                     {"name": "Pi carrier (outline)", "color": [0.30, 0.55, 0.85],
                      "note": "Printed. The Pi 3B+ over the U2D2; NOT attached yet (cad_pi_carrier).",
                      **{k: data["piCarrier"][k] for k in keys}}]
    return L


def rail_joints(data: dict, L: dict) -> dict:
    """The side rails (cad_drive `rail:`) against the parts they bolt to: the
    head on the rail's outer face, at the steering's / the righting's own
    joint, the ridges the length the rail's grooves are cut for. Refuses a
    mismatch, with the value to use; returns the misses (all ~0)."""
    D, RA, S = data["drive"], data["rail"], data["steer"]
    rl = data["driveData"]["s"]["rail"]
    rr = data["rd"]["s"]["rail"]
    jp = data["steerData"]["s"]["joint"]["joint_plate"]
    out = {}
    # the steering's side joint: head jp outside its pad, on the axis at v2SideZ
    if abs(S["v2SideX"] + jp - D["xPo"]) > 1e-6:
        raise ValueError(f"the steering's side heads are at |x| {S['v2SideX'] + jp:g}, the rail's outer face at "
                         f"{D['xPo']:g}: steering_cad.yaml v2.side_pad {D['xPo'] - jp - S['cupHalf']:g}")
    h = cb._add(L["sOrigin"], cb.rx([0.0, 0.0, S["v2SideZ"]], L["rake"]))
    miss = math.hypot(h[1] - rl["steer_joint"][0], h[2] - rl["steer_joint"][1])
    if miss > 0.01:
        raise ValueError(f"the rail's steering joint is {miss:.2f} off the steering's: drive_cad.yaml "
                         f"rail.steer_joint [{h[1] / 1000:.6f}, {h[2] / 1000:.6f}]")
    above = data["steerData"]["s"]["v2"]["side_ridge_above"]
    if abs(rl["steer_ridge_deg"] - (90.0 + L["rake"])) > 1e-9 or abs(rl["steer_ridge_half"] - above) > 1e-9:
        raise ValueError("the rail's steering groove is not along the steering's ridge, up its axis (rail.steer_ridge_*)")
    # the ridge runs down to the pad's bottom: the groove must reach past it
    if rl["steer_groove_open"] < S["v2SideZ"] - S["caseBot"]:
        raise ValueError("the rail's steering groove stops short of the ridge's bottom (rail.steer_groove_open)")
    out["steer"] = miss
    # under the fork's top the rail stays out of the disc the fork sweeps
    # round the steer axis (its legs out to xOut, its plate +-forkHalf): any
    # steer angle, not only the check's samples
    rf = math.hypot(S["xOut"], S["forkHalf"])
    room = math.inf
    pts = data["rail"]["outlineBike"]
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        for k in range(41):
            y = a[0] + (b[0] - a[0]) * k / 40 - L["sOrigin"][1]
            z = a[1] + (b[1] - a[1]) * k / 40 - L["sOrigin"][2]
            ym, zm = cb.rx([0.0, y, z], -L["rake"])[1:]
            if zm < S["blockTop"] + 0.5:
                room = min(room, math.hypot(rl["inner_x"], ym) - rf)
    if room < 1.0:
        raise ValueError(f"the rail comes {room:.2f} from the fork's swept disc (r {rf:.1f}) under its top")
    out["forkRoom"] = room
    # the righting's bridge joints
    rL = data["rL"]
    if not rr.get("joints"):
        raise ValueError("righting_cad.yaml rail.joints is off: the v2 bike's rails bolt to the bridge")
    if abs(rr["outer_x"] - D["xPo"]) > 1e-6:
        raise ValueError(f"righting_cad.yaml rail.outer_x {rr['outer_x']:g}: the rail's outer face is {D['xPo']:g}")
    hy, hz = rr["joint_y"] + L["yr"], rL["Zb"] + rr["joint_above_bridge"] + L["zr"]
    miss = math.hypot(hy - rl["bridge_joint"][0], hz - rl["bridge_joint"][1])
    if miss > 0.01:
        raise ValueError(f"the rail's bridge joint is {miss:.2f} off the righting's: drive_cad.yaml "
                         f"rail.bridge_joint [{hy / 1000:.6f}, {hz / 1000:.7f}]")
    if abs(rl["bridge_ridge_deg"] - 90.0) > 1e-9 or abs(rl["bridge_ridge_half"] - rr["ridge_half"]) > 1e-9:
        raise ValueError("the rail's bridge groove is not along the bridge's ridge (rail.bridge_ridge_*)")
    out["bridge"] = miss
    # the cage posts are drawn in the steering's frame and mirrored about
    # mid-wheelbase from it: that frame must be the bike's
    ax_, az_ = rl["steer_axle"]
    if abs(ax_ - L["wb"]) > 1e-6 or abs(az_ - L["sOrigin"][2]) > 1e-6 or abs(rl["steer_rake_deg"] - L["rake"]) > 1e-9:
        raise ValueError(f"drive_cad.yaml rail.steer_axle / steer_rake_deg are not the bike's: "
                         f"[{L['wb'] / 1000:g}, {L['sOrigin'][2] / 1000:g}], {L['rake']:g}")
    zb = rl["steer_band_zm"]
    if abs(zb[0] - S["caseBot"]) > 1e-6 or abs(zb[1] - S["v2PadTop"]) > 1e-6:
        raise ValueError(f"rail.steer_band_zm is not the steering's lower case bottom and pad top: "
                         f"[{S['caseBot'] / 1000:g}, {S['v2PadTop'] / 1000:g}]")
    (yr_, zr_, _), (yf_, zf_, _) = sorted(c["bike"] for c in data["rail"]["cage"])
    off = abs((yr_ + yf_) - L["wb"]) + abs(zr_ - zf_)
    out["cageMirror"] = off
    # the TM151 on its tabs: the envelope's back face on the tabs' fronts
    tm = data["driveData"]["tm151"]
    th = tm["board_thickness"] + tm["housing_height"]
    want = [0.0, RA["ahY1"] + th / 2, data["driveData"]["s"]["ahrs"]["board_z"]]
    c = data["s"]["components"]["ahrs"]
    if c["frame"] != "drive" or max(abs(a - b) for a, b in zip(c["at"], want)) > 0.01:
        raise ValueError(f"the AHRS is not on its tabs: bike_v2_cad.yaml components.ahrs frame drive, at "
                         f"[{want[0]:g}, {want[1]:.3f}, {want[2]:g}]")
    if [round(v, 6) for v in c["size"]] != [round(v, 6) for v in (tm["board_width"], th, tm["board_length"])]:
        raise ValueError("components.ahrs.size is not the TM151's (ahrs_fixture_cad.yaml tm151)")
    return out


def pi_boxes(pi: dict, h: list) -> list[dict]:
    """The Pi 3B+ in its box's frame: board, USB + Ethernet block at +z (its
    ports 2 proud of the board's end), GPIO header along the +x edge. Each
    must lie inside the box."""
    bw, bl, bt = pi["board"]
    y0 = pi["board_y"] - bt / 2                         # the board's underside
    y1 = y0 + bt
    z0, z1 = pi["board_z"] - bl / 2, pi["board_z"] + bl / 2
    ux, uz, uh = pi["usb_block"]
    gw, gl, gh = pi["gpio"]
    hz0 = z0 + pi["hole_inset"]
    out = [{"name": "Pi 3B+", "lo": [-bw / 2, y0, z0], "hi": [bw / 2, y1, z1], "color": [0.1, 0.5, 0.2]},
           {"name": "Pi USB + Ethernet", "lo": [-ux / 2, y1, z1 + 2.0 - uz], "hi": [ux / 2, y1 + uh, z1 + 2.0],
            "color": [0.75, 0.75, 0.78]},
           {"name": "Pi GPIO header", "lo": [bw / 2 - 1.0 - gw, y1, hz0 + 3.5], "hi": [bw / 2 - 1.0, y1 + gh, hz0 + 3.5 + gl],
            "color": [0.15, 0.15, 0.15]}]
    for b in out:
        if any(b["lo"][i] < -h[i] - 1e-9 or b["hi"][i] > h[i] + 1e-9 for i in range(3)):
            raise ValueError(f"{b['name']} sticks out of the Pi's box ({b['lo']}, {b['hi']} vs +-{h})")
    return out


def tm151_boxes(tm: dict, h: list, sx: int) -> list[dict]:
    """The TM151 in its box's frame (x across its 34, y out of the board
    through the housing, z along its 40; its own +x, the pin-header edge, is
    sx * z, so its USB-C edge is at -sx * z): the board, the housing
    (off-centre along its x), the USB-C receptacle flush with its edge, and
    a KEEP-OUT for the plug's overmold (ahrs_fixture_cad.yaml
    usb_plug_keepout, a GUESS) out past that edge. The board is the first
    box (the holes are cut from it)."""
    y0 = -h[1]
    y1 = y0 + tm["board_thickness"]
    hz = sx * tm["housing_offset_x"]
    L2 = tm["board_length"] / 2
    ux, uz, uh = tm["usb_width"], tm["usb_length"], tm["usb_height"]
    # its +y is the box's z x x... = sx * the box's +x (right-handed: z_tm = y)
    uy = sx * tm["usb_offset_y"]
    kl, kw, kh = (v * 1000 for v in USB_KEEPOUT)
    ue = -sx * L2                                            # the USB edge, box z
    span = lambda a, b: [min(a, b), max(a, b)]               # noqa: E731
    uzr, kzr = span(ue, ue + sx * uz), span(ue, ue - sx * kl)
    out = [{"name": "AHRS", "lo": [-tm["board_width"] / 2, y0, -L2], "hi": [tm["board_width"] / 2, y1, L2],
            "color": [0.15, 0.35, 0.2]},
           {"name": "AHRS housing", "lo": [-tm["housing_width"] / 2, y1, hz - tm["housing_length"] / 2],
            "hi": [tm["housing_width"] / 2, y1 + tm["housing_height"], hz + tm["housing_length"] / 2],
            "color": [0.55, 0.15, 0.15]},
           {"name": "AHRS USB-C", "lo": [uy - ux / 2, y1, uzr[0]], "hi": [uy + ux / 2, y1 + uh, uzr[1]],
            "color": [0.75, 0.75, 0.78]},
           {"name": "AHRS USB plug (keep-out)", "lo": [uy - kw / 2, y1 + uh / 2 - kh / 2, kzr[0]],
            "hi": [uy + kw / 2, y1 + uh / 2 + kh / 2, kzr[1]], "color": [0.85, 0.85, 0.9]}]
    for b in out[:3]:
        if any(b["lo"][i] < -h[i] - 1e-9 or b["hi"][i] > h[i] + 1e-9 for i in range(3)):
            raise ValueError(f"{b['name']} sticks out of the AHRS's box ({b['lo']}, {b['hi']} vs +-{h})")
    return out


U2D2_JST = (14.0, 17.0, 6.0)   # the JST row's keep-out: across, along from the end, out of the face (GUESS)


def u2d2_boxes(h: list) -> list[dict]:
    """The U2D2 in its box's frame (x its 18, y its 14.9, z its 48), from the
    ROBOTIS drawing the user sent (2026-10-16): the USB-C in the +z end; the
    JST connectors (RS485, TTL, UART) in its -y face near the -z end, the
    plugs going in square to that face. Mounted with the USB the AHRS's way
    (up the face, +z) and the JSTs facing the drive (-y: down and back in the
    bike), past the AHRS housing's top end (user). KEEP-OUTs: the USB-C's as
    the AHRS's (USB_KEEPOUT, a GUESS); the JSTs' small (U2D2_JST, GUESS)."""
    kl, kw, kh = (v * 1000 for v in USB_KEEPOUT)
    jw, jl, jd = U2D2_JST
    return [{"name": "U2D2", "lo": [-v for v in h], "hi": list(h), "color": list(COLORS["u2d2"])},
            {"name": "U2D2 USB plug (keep-out)", "lo": [-kw / 2, -kh / 2, h[2]], "hi": [kw / 2, kh / 2, h[2] + kl],
             "color": [0.85, 0.85, 0.9]},
            {"name": "U2D2 JST plugs (keep-out)", "lo": [-jw / 2, -h[1] - jd, -h[2]], "hi": [jw / 2, -h[1], -h[2] + jl],
             "color": [0.85, 0.85, 0.9]}]


def tm151_holes(tm: dict, h: list, sx: int) -> dict:
    """The four M2 holes through the board, 31 along its length (0.45 off
    centre toward its +x, which is sx * z) by 30 across: (x, z) in the box
    frame, the radius."""
    hx, hz = tm["hole_span_y"] / 2, tm["hole_span_x"] / 2
    zc = sx * tm["hole_offset_x"]
    return {"at": [[sx * hx, zc + sz * hz] for sz in (-1, 1) for sx in (1, -1)], "r": tm["hole_dia"] / 2,
            "y": [-h[1] - 1.0, -h[1] + tm["board_thickness"] + 1.0]}


def pi_holes(pi: dict, h: list) -> dict:
    """The four M2.5 holes through the board, hole_inset from the end away
    from the USB/Ethernet block: (x, z) in the box frame, and the radius."""
    hx, hy = pi["holes"]
    z0 = pi["board_z"] - pi["board"][1] / 2 + pi["hole_inset"]
    return {"at": [[sx * hx / 2, z0 + k * hy] for k in (0, 1) for sx in (1, -1)], "r": pi["hole_dia"] / 2,
            "y": [pi["board_y"] - 2.0, pi["board_y"] + 2.0]}


def to_bike(L: dict, c: dict, p) -> list[float]:
    """A point in component c's own box frame (about its centre) in the bike frame."""
    q = cb.rx(p, c["rx"])
    q = [q[i] + c["at"][i] for i in range(3)]
    if c["frame"] == "drive":
        return cb.rx(q, L["tilt"])
    if c["frame"] == "steer":
        return cb._add(L["sOrigin"], cb.rx(q, L["rake"]))
    return q


def comp_box(L: dict, c: dict, box: dict | None = None) -> tuple[list, list]:
    """The bike-frame axis-aligned bounding box of one of component c's
    boxes (its whole box if none)."""
    lo, hi = (box["lo"], box["hi"]) if box else ([-v / 2 for v in c["size"]], [v / 2 for v in c["size"]])
    pts = [to_bike(L, c, [a[0], b[1], d[2]]) for a in (lo, hi) for b in (lo, hi) for d in (lo, hi)]
    return ([min(q[i] for q in pts) for i in range(3)], [max(q[i] for q in pts) for i in range(3)])


FS = r'''FeatureScript %VERSION%;
import(path : "onshape/std/geometry.fs", version : "%VERSION%.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_bike_v2 --push bike_v2_features
 *
 * The drive, steering-v2 and righting modules' geometry layers, merged (each
 * module's own studio is generated from the same config), placed in the bike
 * frame, and the electronics as envelopes. No chassis. Numbers from
 * config/bike_v2_cad.yaml through aow_sim.cad_bike_v2's layout(), carried
 * here as BK2. Millimetres.
 *
 * BIKE FRAME: origin on the rear axle's centre, +Y forward, +X right, +Z up.
 */

%MODULES%

export const BK2 = %BK2%;

/** A bike-frame vector from a plain [x, y, z] in mm. */
export function bk2V(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]) * millimeter;
}

/**
 * The battery + power carrier (cad_power): prisms on the bike's (y, z) over
 * x ranges, united, cut; then one 6-32 a side, the head in the rail (cut by
 * the drive), the nut and its slot here, and the ridge on the lug's face.
 * Printed; found again by a point in its floor.
 */
export function bk2Carrier(context is Context, id is Id, C is map) returns Query
{
    const mm = millimeter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const A  = id + "a";
    const K  = id + "k";
    const prism = function(scope is Id, p is map)
    {
        var pts = [];
        for (var q in p.pts)
            pts = append(pts, vector(q[0], q[1]) * mm);
        polyPrism(context, scope, p.tag, vector(p.x[0], 0, 0) * mm, X, Y, pts, (p.x[1] - p.x[0]) * mm);
    };
    // everything under A first, then everything under K: an Id's children
    // must be contiguous in the feature history
    for (var a in C.adds)
        prism(A, a);
    for (var c in C.cyls)
        if (c.op == "add")
            cylW(context, A + c.tag, bk2V(c.p0), bk2V(c.p1), c.r * mm);
    opBoolean(context, id + "u", { "tools" : qBodyType(qCreatedBy(A, EntityType.BODY), BodyType.SOLID),
            "operationType" : BooleanOperationType.UNION });
    for (var c in C.cuts)
        prism(K, c);
    for (var c in C.cyls)
        if (c.op != "add")
            cylW(context, K + c.tag, bk2V(c.p0), bk2V(c.p1), c.r * mm);
    const pt = bk2V(C.pt);
    if (size(C.cuts) > 0 || size(C.cyls) > 0)
        cutFrom(context, id + "cut", partAt(context, id, pt), [qBodyType(qCreatedBy(K, EntityType.BODY), BodyType.SOLID)]);
    for (var j in C.joints)
    {
        const J = id + ("j" ~ j.side);
        const cs = coordSystem(bk2V(j.head), vector(j.slot[0], j.slot[1], j.slot[2]), vector(j.zIn[0], j.zIn[1], j.zIn[2]));
        screwJointGeometry(context, J, cs, mergeMaps(SCREW_OPT, {
                "nutDepth" : j.nutDepth * mm, "headPocket" : 0 * mm, "bothWays" : false,
                "nutSlotLength" : j.slotLen * mm, "holeDepth" : j.holeDepth * mm }));
        cutFrom(context, J + "cut", partAt(context, id, pt), [qCreatedBy(J + "screwRev", EntityType.BODY),
                                                             qCreatedBy(J + "slotExt", EntityType.BODY)]);
        const r = ridgeSolid(context, J + "ridge", bk2V(j.ridgeAt), Z, vector(j.ridgeUp[0], j.ridgeUp[1], j.ridgeUp[2]),
                             FIXTURE.ridge_height, 0 * mm, j.ridgeHalf * mm);
        opBoolean(context, J + "rU", { "tools" : qUnion([partAt(context, id, pt), r]),
                "operationType" : BooleanOperationType.UNION });
    }
    const q = partAt(context, id, pt);
    dress(context, q, C.name, color(C.color[0], C.color[1], C.color[2]), C.note);
    setAttribute(context, { "entities" : q, "name" : "bk_mod", "attribute" : "bike" });
    return q;
}

/**
 * The whole bike, v2. opt: steer (angle), pose (a RG_POSES key), mocks
 * (bool: the electronics envelopes). Returns what the check sweeps.
 */
export function bikeV2Build(context is Context, id is Id, opt is map) returns map
{
    const mm = millimeter;
    const X  = vector(1, 0, 0);
    const Z  = vector(0, 0, 1);
    const O  = vector(0, 0, 0) * mm;
    const dXf = rotationAround(line(O, X), BK2.tilt * degree);
    const sXf = transform(bk2V(BK2.sOrigin)) * rotationAround(line(O, X), BK2.rake * degree);
    const rXf = transform(vector(0, BK2.yr, BK2.zr) * mm);
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };

    step("modules");
    const STi = id + "st";
    const DRi = id + "dr";
    const RGi = id + "rg";
    const rs = steeringBuild(context, STi, { "attach" : "SCREWS", "key" : "CROSS", "v2" : true });
    driveBuild(context, DRi, { "shift" : 0 * mm, "fixture" : false, "rail" : true });
    const rr = rightingBuild(context, RGi, { "mocks" : true });
    // the righting's floor mock and chassis placeholder: not in v2
    for (var k = 0; k < size(BK2.rgDrop); k += 1)
        opDeleteBodies(context, id + ("rgDrop" ~ k), { "entities" : partAt(context, RGi, bk2V(BK2.rgDrop[k])) });
    for (var g in [[STi, "steering"], [DRi, "drive"], [RGi, "righting"]])
        setAttribute(context, { "entities" : qBodyType(qCreatedBy(g[0], EntityType.BODY), BodyType.SOLID),
                                "name" : "bk_mod", "attribute" : g[1] });

    step("place");
    // tag what moves BEFORE anything does: the stage and group queries find
    // their bodies by a point inside, which a moved body no longer contains
    for (var q in rs.stages["steer"])
        setAttribute(context, { "entities" : q, "name" : "bk_steer", "attribute" : "steer" });
    for (var g in ["crank", "cR", "cL", "wR", "wL"])
        for (var q in rr.groups[g])
            setAttribute(context, { "entities" : q, "name" : "bk_" ~ g, "attribute" : g });
    if (opt.steer != undefined && opt.steer != 0 * degree)
        opTransform(context, id + "steer", { "bodies" : qHasAttribute("bk_steer"),
                "transform" : rotationAround(rs.axis, opt.steer) });
    if (opt.pose != undefined && opt.pose != "+0.00")
        rgPose(context, id + "pose", rr.groups, opt.pose, false);
    opTransform(context, id + "stXf", { "bodies" : qBodyType(qCreatedBy(STi, EntityType.BODY), BodyType.SOLID), "transform" : sXf });
    opTransform(context, id + "drXf", { "bodies" : qBodyType(qCreatedBy(DRi, EntityType.BODY), BodyType.SOLID), "transform" : dXf });
    opTransform(context, id + "rgXf", { "bodies" : qBodyType(qCreatedBy(RGi, EntityType.BODY), BodyType.SOLID), "transform" : rXf });

    step("components");
    if (opt.mocks != false)
        for (var k = 0; k < size(BK2.comps); k += 1)
        {
            const c = BK2.comps[k];
            var xf = transform(bk2V(c.at)) * rotationAround(line(O, X), c.rx * degree);
            if (c.frame == "drive")
                xf = dXf * xf;
            else if (c.frame == "steer")
                xf = sXf * xf;
            for (var i = 0; i < size(c.boxes); i += 1)
            {
                const b = c.boxes[i];
                const C = id + ("comp" ~ k ~ "_" ~ i);
                const q = boxIn(context, C, coordSystem(O, X, Z), bk2V(b.lo), bk2V(b.hi));
                // the Pi's mounting holes, through the board (its first box)
                if (i == 0 && c.holes != undefined)
                {
                    var hs = [];
                    for (var j = 0; j < size(c.holes.at); j += 1)
                    {
                        const a = c.holes.at[j];
                        // along the box's y (the Pi, the AHRS), or its x (the
                        // power board, which faces the side)
                        const ax = c.holes.ax == "x";
                        // NOT under C: boxIn's query is everything created under
                        // C, so a cutter there is its own target (BOOLEAN_INVALID)
                        hs = append(hs, cylW(context, id + ("compH" ~ k ~ "_" ~ j),
                                             bk2V(ax ? [c.holes.y[0], a[0], a[1]] : [a[0], c.holes.y[0], a[1]]),
                                             bk2V(ax ? [c.holes.y[1], a[0], a[1]] : [a[0], c.holes.y[1], a[1]]), c.holes.r * mm));
                    }
                    opBoolean(context, id + ("compHc" ~ k), { "tools" : qUnion(hs), "targets" : q,
                            "operationType" : BooleanOperationType.SUBTRACTION });
                }
                // its own id: boxIn already used C + "xf" for its own placement
                opTransform(context, id + ("compXf" ~ k ~ "_" ~ i), { "bodies" : q, "transform" : xf });
                dress(context, q, b.name, color(b.color[0], b.color[1], b.color[2]),
                      "Not a print. Free to move: config/bike_v2_cad.yaml.");
                setAttribute(context, { "entities" : q, "name" : "bk_mod", "attribute" : "bike" });
            }
        }
    step("carriers");
    for (var k = 0; k < size(BK2.carriers); k += 1)
        bk2Carrier(context, id + ("pc" ~ k), BK2.carriers[k]);
    return { "steer" : rs, "righting" : rr, "rXf" : rXf,
             "sAxis" : line(bk2V(BK2.sOrigin), vector(BK2.sAxis[0], BK2.sAxis[1], BK2.sAxis[2])) };
}

%SPLIT%

export enum BikeV2Pose
{
    annotation { "Name" : "Rest (stowed)" }
    REST,
    annotation { "Name" : "R 50 %" }
    R50,
    annotation { "Name" : "R 100 %" }
    R100,
    annotation { "Name" : "L 50 %" }
    L50,
    annotation { "Name" : "L 100 %" }
    L100
}

annotation { "Feature Type Name" : "AOW bike v2",
             "Feature Type Description" : "The whole bike, v2: drive, steering v2 and righting placed, the electronics free-floating, no chassis" }
export const aowBikeV2 = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Steer angle" }
        isAngle(definition.steer, { (degree) : [-180, 0, 180] } as AngleBoundSpec);
        annotation { "Name" : "Righting pose" }
        definition.pose is BikeV2Pose;
        annotation { "Name" : "Show the electronics and battery", "Default" : true }
        definition.mocks is boolean;
    }
    {
        const keys = { BikeV2Pose.REST : "+0.00", BikeV2Pose.R50 : "+0.50", BikeV2Pose.R100 : "+1.00",
                       BikeV2Pose.L50 : "-0.50", BikeV2Pose.L100 : "-1.00" };
        bikeV2Build(context, id + "build", { "steer" : definition.steer, "pose" : keys[definition.pose],
                                             "mocks" : definition.mocks });
        reportFeatureInfo(context, id, "%INFO%");
    });
'''


def build_fs(L: dict, fs_version: str = "3044") -> str:
    keep = ("wb", "yr", "zr", "tilt", "rake", "sOrigin", "sAxis", "rgDrop", "comps", "carriers")
    bk = {k: L[k] for k in keep}
    bk["comps"] = [{k: c[k] for k in ("name", "frame", "at", "rx", "boxes", "holes") if c[k] is not None}
                   for c in L["comps"]]
    info = (f"Wheelbase {L['wb']:g} mm (CAD only); righting rod {L['yr']:g} mm ahead of the rear axle, "
            f"{L['zr'] - L['floor']:.1f} mm off the floor")
    subs = {"%VERSION%": fs_version, "%MODULES%": cb.merge_layers(module_texts()), "%BK2%": cb._fs(bk),
            "%SPLIT%": SPLIT_MARK, "%INFO%": info}
    text = FS
    for k, v in subs.items():
        text = text.replace(k, v)
    if "getProperty" in sm._strip_comments(text):
        raise SystemExit("getProperty in the studio: it throws in a Part Studio; find parts by a point")
    return text


def check_wrapper(fs: str) -> str:
    """Build once; print every body's box, then every clash between modules
    and envelopes at rest, steered, and at each righting pose. ONE call."""
    sweep = "[" + ", ".join(str(a) for a in STEER_SWEEP) + "]"
    poses = "[" + ", ".join(f'"{k}"' for k in POSES if k != "+0.00") + "]"
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
    const mm = millimeter;
    const id = makeId("chk");
    const r = bikeV2Build(context, id, {{ "mocks" : true, "debug" : true }});
    const all = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    for (var b in all)
    {{
        const at = getAttributes(context, {{ "entities" : b, "name" : "bk_mod" }});
        if (size(at) > 0 && at[0] != "bike")
            setProperty(context, {{ "entities" : b, "propertyType" : PropertyType.NAME, "value" : at[0] ~ ": " ~ name(b) }});
    }}
    for (var b in all)
    {{
        const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
        println("BODY|" ~ name(b) ~ "|" ~ toString(bb.minCorner[0] / mm) ~ "|" ~ toString(bb.minCorner[1] / mm)
                ~ "|" ~ toString(bb.minCorner[2] / mm) ~ "|" ~ toString(bb.maxCorner[0] / mm) ~ "|"
                ~ toString(bb.maxCorner[1] / mm) ~ "|" ~ toString(bb.maxCorner[2] / mm) ~ "|"
                ~ toString(evVolume(context, {{ "entities" : b }}) / (mm * mm * mm)));
    }}
    for (var i = 0; i + 1 < size(all); i += 1)
        clashes("rest", all[i], qUnion(subArray(all, i + 1, size(all))));
    const tu = qHasAttribute("bk_steer");
    const tuL = evaluateQuery(context, tu);
    var others = [];
    for (var b in all)
        if (!isIn(b, tuL))
            others = append(others, b);
    var k = 0;
    for (var a in {sweep})
    {{
        opTransform(context, id + ("t" ~ k), {{ "bodies" : tu, "transform" : rotationAround(r.sAxis, a * degree) }});
        clashes("steer " ~ a, tu, qUnion(others));
        opTransform(context, id + ("tb" ~ k), {{ "bodies" : tu, "transform" : rotationAround(r.sAxis, -a * degree) }});
        k += 1;
    }}
    const gs = ["crank", "cR", "cL", "wR", "wL"];
    var mvL = [];
    for (var g in gs)
        mvL = concatenateArrays([mvL, evaluateQuery(context, qHasAttribute("bk_" ~ g))]);
    var rest = [];
    for (var b in all)
    {{
        if (isIn(b, mvL) || match(name(b), "^righting: .*").hasMatch)
            continue;
        rest = append(rest, b);
    }}
    k = 0;
    for (var key in {poses})
    {{
        const ps = RG_POSES[key];
        for (var g in gs)
            opTransform(context, id + ("p" ~ k ~ g), {{ "bodies" : qHasAttribute("bk_" ~ g),
                    "transform" : r.rXf * rgXf(ps[g]) * inverse(r.rXf) }});
        clashes("pose " ~ key, qUnion(mvL), qUnion(rest));
        for (var g in gs)
            opTransform(context, id + ("pb" ~ k ~ g), {{ "bodies" : qHasAttribute("bk_" ~ g),
                    "transform" : r.rXf * inverse(rgXf(ps[g])) * inverse(r.rXf) }});
        k += 1;
    }}
    return "ran to completion";
}}
"""


def _module(name: str) -> str:
    for m in ("steering", "drive", "righting"):
        if name.startswith(m + ": "):
            return m
    return "bike"


def judge(console: str, L: dict) -> bool:
    rows = [l.split("|") for l in console.splitlines()]
    ok = True
    bodies = [r for r in rows if r[0] == "BODY"]
    strays = [r for r in bodies if r[1].endswith("UNNAMED")]
    if strays:
        ok = False
        print(f"  {len(strays)} UNNAMED bodies  FAIL")
    print(f"  {len(bodies)} bodies")
    want = {b["name"]: comp_box(L, c, b) for c in L["comps"] for b in c["boxes"]}
    for r in bodies:
        if r[1] in want:
            lo, hi = [float(v) for v in r[2:5]], [float(v) for v in r[5:8]]
            good = all(abs(lo[i] - want[r[1]][0][i]) < 1e-3 and abs(hi[i] - want[r[1]][1][i]) < 1e-3 for i in range(3))
            ok &= good
            print(f"  {r[1]:18} x {lo[0]:6.1f}..{hi[0]:5.1f}  y {lo[1]:6.1f}..{hi[1]:5.1f}  "
                  f"z {lo[2]:6.1f}..{hi[2]:5.1f}  {'ok' if good else 'FAIL (placed off its config)'}")
    hits: dict[tuple, set] = {}
    keep: dict[tuple, set] = {}
    for r in rows:
        if r[0] != "COLL" or r[4].startswith("ABUT"):
            continue
        a, b = sorted((r[2], r[3]))
        if _module(a) == _module(b) != "bike":
            continue
        # a KEEP-OUT (room a cable's plug needs) is reported, not failed
        (keep if "(keep-out)" in a + b else hits).setdefault((a, b), set()).add(r[1])
    for (a, b), ps in sorted(keep.items()):
        print(f"  KEEP-OUT reached: {a}  x  {b}   [{', '.join(sorted(ps))}]")
    if hits:
        ok = False
        print(f"  {len(hits)} interfering pairs:")
        for (a, b), ps in sorted(hits.items()):
            print(f"    {a}  x  {b}   [{', '.join(sorted(ps))}]")
    else:
        print("  no interference across modules and envelopes: at rest, steered "
              f"{'/'.join(str(a) for a in STEER_SWEEP)}, the righting at every pose")
    return ok


def check(text: str, L: dict, target: str | None) -> bool:
    from . import onshape
    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    out = Path("traces/bike_v2_cad/check.txt")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(console)
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console[-3000:])
        print(onshape.budget_line())
        return False
    ok = judge(console, L)
    print(f"  console -> {out}")
    print(onshape.budget_line())
    return ok


def report(L: dict) -> str:
    out = [f"  wheelbase {L['wb']:g} (righting rod at {L['yr']:g}, front axle {L['wb'] - L['yr']:g} ahead of it); "
           f"rod {L['zr'] - L['floor']:.1f} off the floor",
           "  blades' spare to the front tyre's steer sweep: "
           + ", ".join(f"{k} {v:.1f}" for k, v in L["wingRoom"].items())
           + (f"; the blades span y {L['bladeY'][0]:g}..{L['bladeY'][1]:g}, centred on the wheelbase"
              if "bladeY" in L else ""),
           "  the electronics, bike frame (bounding box):"]
    for c in L["comps"]:
        lo, hi = comp_box(L, c)       # its whole box (the Pi's detail sits inside)
        m = "" if c["mass"] is None else f"  {c['mass']:g} g"
        out.append(f"    {c['name']:12} x {lo[0]:6.1f}..{hi[0]:5.1f}  y {lo[1]:6.1f}..{hi[1]:6.1f}  "
                   f"z {lo[2]:6.1f}..{hi[2]:6.1f}  ({c['frame']} frame){m}")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=PARAMS)
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="build in Onshape and check it; ONE billable call, `check` tab")
    ap.add_argument("--push", metavar="TAB|URL", nargs="?", const="", default=None,
                    help=f"replace a Feature Studio's contents (`{TAB}` only)")
    ap.add_argument("--dry-run", action="store_true", help="print the check script instead of spending a call")
    args = ap.parse_args()

    L = layout(load(args.config))
    text = build_fs(L, args.fs_version)
    sm.lint_fs(text)
    Path(args.output).write_text(text)
    print(f"wrote {len(text)} chars -> {args.output}")
    print(report(L))
    if args.dry_run:
        print(check_wrapper(text))
        return
    if args.check is not None:
        if not check(text, L, args.check or None):
            raise SystemExit("check FAILED -- not pushing")
    if args.push is not None:
        from . import onshape
        if args.push != TAB:
            raise SystemExit(f"refusing to push at {args.push or 'the default'!r}: this generator owns `{TAB}` only")
        url = onshape.resolve(args.push, args.push)
        reply = onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}  (microversion {reply.get('sourceMicroversion', '?')})")
        print(onshape.budget_line())


if __name__ == "__main__":
    main()
