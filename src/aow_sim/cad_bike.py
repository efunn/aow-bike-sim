"""The whole bike in one Part Studio: the drive, steering and righting modules
placed in the bike frame, plus the parts that tie them together -- the
chassis, the battery holder, the electronics carrier and the roll cage. ONE
custom feature, `AOW bike`, in its own Feature Studio. Numbers:
config/bike_cad.yaml, and each module's own config through its generator.

MODULE COMPOSITION, NOT IMPORT. Each module's generator writes a Feature
Studio whose geometry layer (above SPLIT_MARK) is pure functions:
`driveBuild`, `steeringBuild`, `rightingBuild`. This generator takes those
three layers, merges them into one studio, and calls each build under its own
id before moving its bodies into place. The helpers the modules share
(`servoMountGeometry`, `dress`, `screwJoint`, ...) come from the same Python
strings, so they are textually identical; `merge_layers` keeps one copy and
REFUSES if two ever differ, rather than letting the last one win.

Why not an FS `import` of the three studios: a cross-tab import is pinned to
a microversion, and the `--check` eval cannot import at all, so the check
would test different code from the push. Composing here means the push and
the check are the same text, built from the same local config the module
generators read. A module change reaches this studio on the next
`cad_bike --push`, not before.

BIKE FRAME: origin on the rear axle's centre, +Y forward, +X right, +Z up;
the floor at Z = -(rear wheel radius). The same axes as every module frame,
so each placement is one rotation about X and a translation:

    drive     origin on the rear axle, its +Y (the belt) tilted up by
              drivetrain.drive_servo_angle_deg
    steering  origin on the front axle, `wheelbase` forward, its +Z (the
              steering axis) tilted back by bike.rake_deg
    righting  origin on the wing rod, `righting_y` forward, rod height from
              the module (41.2 off the floor)

THE NEW PARTS, and where they borrow from the modules:

    part             print  what it is
    chassis          Z+     the deck over the righting's bridge, a wedge up
                            to the drive, the steering's mount plate, posts
                            for the carrier. It IS the three modules'
                            chassis placeholders (the drive's fixture block,
                            the steering's mount plate less its bench tab,
                            the righting's chassis plate) united with the
                            new pieces, so every module joint is the one its
                            generator cut and checked.
    battery tray     +Z(d)  on the drive's back, between the drive pulleys;
                            a tongue screwed onto the drive block's top
    carrier          Z-     the electronics: Pi 3B+ on top, U2D2 / AHRS /
                            power board underneath, the switch in a tab
                            off the rear-right corner
    cage spine       X      the roll bar in the mid-plane: a foot on the
                            drive block's front face, a foot behind the
                            steering plate's extension, a rail over the top
    cage rib 1..n    Y      arches across the spine

    python -m aow_sim.cad_bike                   # write docs/cad/bike.fs
    python -m aow_sim.cad_bike --check           # ONE call, `check` tab
    python -m aow_sim.cad_bike --push bike_features
    python -m aow_sim.cad_bike --shot            # -> docs/cad/bike.png
    python -m aow_sim.cad_bike --fit 2           # ONE call: righting fit sweep
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import yaml

from . import cad_ahrs_fixture as af
from . import cad_drive as cd
from . import cad_righting as cr
from . import cad_servo_mount as sm
from . import cad_steering as cs
from .params import _normalize, load_params

PARAMS = "config/bike_cad.yaml"
OUT_FS = "docs/cad/bike.fs"
OUT_PNG = "docs/cad/bike{}.png"
SPLIT_MARK = af.SPLIT_MARK
TAB = "bike_features"
PART_STUDIO = "bike_assembly"
POSES = ("+0.00", "+0.25", "+0.50", "+0.75", "+1.00", "-0.25", "-0.50", "-0.56", "-0.75", "-1.00")
STEER_SWEEP = (30, 90, 180, 270)
ASA = 1.07                      # g/cm^3, solid (righting_cad.yaml materials)


# --------------------------------------------------------------------------
# the modules' geometry layers, merged
# --------------------------------------------------------------------------

def module_texts() -> dict[str, str]:
    """Each module's whole Feature Studio, generated from local config now."""
    rd = cr.load()
    return {"steering": cs.build_fs(cs.load()),
            "drive": cd.build_fs(cd.load()),
            "righting": cr.build_fs(cr.layout(rd), rd)}


def _decls(fs: str) -> list[tuple[str, str]]:
    """(name, text) for every top-level declaration above SPLIT_MARK."""
    head = sm._strip_comments(fs.split(SPLIT_MARK)[0]).splitlines()
    out, i = [], 0
    while i < len(head):
        line = head[i]
        if line.startswith(("FeatureScript ", "import(")):
            i += 1
            continue
        m = re.match(r"^export (const|function|enum|predicate|type) (\w+)", line)
        if not m:
            raise ValueError(f"unexpected top-level line: {line[:80]!r}")
        end = i
        if m.group(1) != "const" or line.count("{") != line.count("}"):
            end = sm._block_end(head, i)
        # a multi-line const may also close on a later line without braces
        while not head[end].rstrip().endswith((";", "}")):
            end += 1
        out.append((m.group(2), "\n".join(head[i:end + 1])))
        i = end + 1
    return out


def merge_layers(texts: dict[str, str]) -> str:
    """One geometry layer from several, shared declarations kept once.

    Order is first appearance, module by module, so a helper is always
    declared before the first module function that calls it -- which the
    eval rewrite needs (see cad_servo_mount.check_wrapper)."""
    seen: dict[str, tuple[str, str]] = {}
    order: list[str] = []
    for mod, fs in texts.items():
        for name, text in _decls(fs):
            if name in seen:
                if seen[name][1] != text:
                    raise SystemExit(f"{name!r} differs between {seen[name][0]} and "
                                     f"{mod}: regenerate both from the same helpers")
                continue
            seen[name] = (mod, text)
            order.append(name)
    return "\n\n".join(seen[n][1] for n in order)


# --------------------------------------------------------------------------
# layout: every number the studio builds from, bike frame unless noted, mm
# --------------------------------------------------------------------------

def load(path: str = PARAMS) -> dict:
    raw = _normalize(yaml.safe_load(Path(path).read_text()))
    p = load_params(sm.CAD_PARAMS)
    rd = cr.load()
    return {"s": raw, "params": p, "steer": cs.layout(cs.load()),
            "drive": cd.full_layout(cd.load()), "rd": rd, "rL": cr.layout(rd)}


def rx(p, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [p[0], p[1] * c - p[2] * s, p[1] * s + p[2] * c]


def _add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def rounded_path(pts, R: float, n: int = 6) -> list:
    """A polyline with every corner replaced by an arc of centreline radius
    R tangent to both legs, sampled at n points: a cable's path. Raises if a
    leg is too short for the bends at its two ends (the cable cannot bend
    tighter than R)."""
    import numpy as np
    P = [np.asarray(p, float) for p in pts]
    tan = [0.0] * len(P)
    geo = [None] * len(P)
    for i in range(1, len(P) - 1):
        a, b, c = P[i - 1], P[i], P[i + 1]
        u1, u2 = (a - b) / np.linalg.norm(a - b), (c - b) / np.linalg.norm(c - b)
        th = math.acos(float(np.clip(np.dot(u1, u2), -1.0, 1.0)))   # between the legs
        if th > math.pi - 1e-6:
            continue                                                 # straight through
        tan[i] = R / math.tan(th / 2)
        geo[i] = (u1, u2, th)
    for i in range(len(P) - 1):
        if tan[i] + tan[i + 1] > np.linalg.norm(P[i + 1] - P[i]) + 1e-6:
            raise ValueError(f"cable leg {P[i].round(1).tolist()} -> {P[i + 1].round(1).tolist()} "
                             f"too short for its {R:g} mm bends")
    out = [P[0]]
    for i in range(1, len(P) - 1):
        if geo[i] is None:
            out.append(P[i])
            continue
        u1, u2, th = geo[i]
        b = P[i]
        ctr = b + (u1 + u2) / np.linalg.norm(u1 + u2) * R / math.sin(th / 2)
        v1, v2 = b + u1 * tan[i] - ctr, b + u2 * tan[i] - ctr
        ang = math.acos(float(np.clip(np.dot(v1, v2) / (R * R), -1.0, 1.0)))
        for k in range(n + 1):
            f = k / n
            out.append(ctr + (math.sin((1 - f) * ang) * v1 + math.sin(f * ang) * v2) / math.sin(ang))
    out.append(P[-1])
    return [[round(float(v), 4) for v in p] for p in out]


def layout(data: dict) -> dict:
    s, p, S, D, rL = data["s"], data["params"], data["steer"], data["drive"], data["rL"]
    pl, ch, bt, el, rc = s["placement"], s["chassis"], s["battery"], s["electronics"], s["rollcage"]
    R = p["omni_wheel"]["outer_radius"] * 1000
    tilt = p["drivetrain"]["drive_servo_angle_deg"]
    rake = p["bike"]["rake_deg"]
    wb, yr = pl["wheelbase"], pl["righting_y"]
    zr = -R - rL["floor_z"]
    zf = S["R"] - R
    plate = next(q for q in rL["parts"] if q["slug"] == "chassis")["add"][0]
    jp = 4.6                                           # FIXTURE.joint_plate

    drv = lambda q: rx(q, tilt)                                        # noqa: E731
    ste = lambda q: _add([0, wb, zf], rx(q, rake))                     # noqa: E731

    L = {"wb": wb, "yr": yr, "zr": zr, "zf": zf, "tilt": tilt, "rake": rake, "R": R,
         "floor": -R}
    L["dAxis"] = rx([0, 0, 1], tilt)
    L["sAxis"] = rx([0, 0, 1], rake)
    L["sOrigin"] = [0, wb, zf]

    # ---- chassis
    z0 = plate["lo"][2] + zr
    z1 = plate["hi"][2] + zr
    A = drv([0, D["Ytab0"] + D["fixGap"], -D["Zp"]])          # block, rear-bottom edge
    B = drv([0, D["Yfront"], -D["Zp"]])                       # block, front-bottom edge
    pb = ste([0, S["plateBack"], S["caseBot"]])               # steering plate, back-bottom
    lcb = ste([0, S["plateBack"] + S["yP"] - S["plateBack"] - 1.2, S["caseBot"]])  # lower case, back-bottom
    # in pieces round the righting's plate, which keeps its own joints: a
    # neck as narrow as the wedge (the lower drive pulley comes down to deck
    # height at |x| 17-34 as far forward as y ~121) where the plate does not
    # reach the wedge, and the full width in front of the plate, stopping
    # short of the steering's lower case (its bottom rises from there)
    py0, py1 = plate["lo"][1] + yr, plate["hi"][1] + yr
    y_front = lcb[1] - 0.3
    boxes = [[[-ch["deck_half"], py1, z0], [ch["deck_half"], y_front, z1]]]
    if py0 > A[1]:
        boxes.append([[-ch["wedge_half"], A[1], z0], [ch["wedge_half"], py0, z1]])
    # no righting joint may sit under the wedge: its head would be buried
    for yj in data["rd"]["s"]["chassis"]["joints_y"]:
        if A[1] - 4.0 < yj + yr < B[1] + 4.0:
            raise ValueError(f"the righting's chassis joint at y {yj + yr:.1f} sits under the drive "
                             f"block's wedge ({A[1]:.1f}..{B[1]:.1f}): its head would be buried")
    L["deck"] = {"z0": z0, "z1": z1, "boxes": boxes}
    L["wedge"] = {"pts": [[A[1], z0], [B[1], z0], [B[1], B[2]], [A[1], A[2]]],
                  "half": ch["wedge_half"]}
    # a gusset behind the steering plate, stopping under its lower joint's head
    zg = z1 + 8.0
    yg = pb[1] - (zg - pb[2]) * math.tan(math.radians(rake))
    L["gusset"] = {"pts": [[pb[1] - 10.0, z1 - 1.0], [pb[1] + 0.3, z1 - 1.0], [yg + 0.3, zg]],
                   "half": S["botOuter"]}
    # steering plate, module frame: the bench tab and foot trimmed, an
    # extension above the upper case for the cage's front foot
    L["stTrim"] = [[[-60, S["plateBack"] - 80, S["caseBot"] - 40], [60, S["plateBack"], S["upTopZ"] + 60]],
                   [[S["botOuter"], S["plateBack"] - 1, S["caseBot"] - 40], [60, S["yP"] + 1, S["upTopZ"] + 1]],
                   [[-60, S["plateBack"] - 1, S["caseBot"] - 40], [-S["botOuter"], S["yP"] + 1, S["upTopZ"] + 1]]]
    L["stExt"] = [[-ch["steer_ext_half"], S["plateBack"], S["upTopZ"] - 1],
                  [ch["steer_ext_half"], S["yP"], S["upTopZ"] + ch["steer_ext"]]]
    L["deckPt"] = [0, (A[1] + B[1]) / 2, (z0 + z1) / 2]
    # the drive's four case-side screws run along X through the block: level
    # as the chassis prints deck-down, so each bore gets a 45 deg crown (+Z)
    L["blockBores"] = [drv([0, y, sz * D["zF"]]) for y in (D["yF0"], D["yF1"]) for sz in (1, -1)]
    L["blockBoreX"] = D["xPo"] + 1.0
    # a point inside each module's placeholder, once placed: how the chassis
    # finds them again for the union (a union keeps only one identity)
    anchor = next(q for q in rL["parts"] if q["slug"] == "chassis")["anchor"]
    L["rgFloorPt"] = next(q for q in rL["parts"] if q["slug"] == "floor")["anchor"]
    L["stPlatePt"] = [0, (S["plateBack"] + S["yP"]) / 2, S["caseBot"] + 6.0]
    L["holdPts"] = [drv([0, D["Yfront"] - 1.0, 0]),
                    ste([0, (S["plateBack"] + S["yP"]) / 2, S["caseBot"] + 6.0]),
                    _add([0, yr, zr], anchor)]

    joints = []

    def joint(tag, frame, head, zin, slot, slot_len, csk, nut, csk_up, nut_up):
        """zin and slot in `frame`; the print-up vectors in the bike frame."""
        joints.append({"tag": tag, "frame": frame, "head": head, "zIn": zin, "slot": slot,
                       "len": slot_len, "csk": csk, "nut": nut, "cskUp": csk_up, "nutUp": nut_up})

    dZ = rx([0, 0, 1], tilt)                         # drive +Z / -Y, in the bike frame
    dmY = rx([0, -1, 0], tilt)

    # ---- electronics carrier, DRIVE frame: x across, y out of the drive
    # block's front face, z along the face (up-back) = edge + u, u up the
    # carrier from its lower edge
    cw, cl, ct = el["carrier"]
    fy = D["Yfront"]                                  # the face
    cy0 = fy + el["gap"]                              # carrier underside
    cy1 = cy0 + ct
    e0 = el["edge"]
    W_ = lambda u: e0 + u                             # noqa: E731
    nh = el["narrow_half"]
    L["carrier"] = {"plates": [[[-nh, cy0, W_(0)], [nh, cy1, W_(el["wide_u"] + 1.0)]],
                               [[-cw / 2, cy0, W_(el["wide_u"])], [cw / 2, cy1, W_(cl)]]],
                    "pt": [0.0, cy1 - 0.5, W_(cl / 2)]}
    pr = ch["post_dia"] / 2
    ptop = cy1 - el["boss"]
    posts = [[sx * px, pz] for px, pz in ch["post_at"] for sx in (1, -1)]
    for x, z in posts:
        if abs(x) + pr > 16.0 or abs(z) + pr > D["Zp"]:
            raise ValueError(f"post at ({x:g}, {z:g}) is off the drive block's face")
    L["posts"] = {"cyl": [[[x, fy - 0.5, z], [x, ptop, z]] for x, z in posts], "r": pr}
    bosses = [[[x, ptop, z], [x, cy0 + 0.5, z], pr + 1.0] for x, z in posts]
    cgz = W_(el["cage_u"])
    if el["cage_u"] - 6.0 < el["pi"]["u_usb"] + el["pi"]["board"][1]:
        raise ValueError("the cage's rear foot is under the Pi: its strut would rise through the board")
    bosses.append([[0.0, cy1 - 7.0, cgz], [0.0, cy0 + 0.5, cgz], 5.5])   # the cage foot's nut
    for k, (x, z) in enumerate(posts):
        joint(f"jPost{k}", "drive", [x, cy1, z], [0, -1, 0], [1 if x > 0 else -1, 0, 0], pr + 2.0,
              "carrier", "chassis", dmY, [0, 0, 1])
    pi = el["pi"]
    bw, bl, bth = pi["board"]
    zu = W_(pi["u_usb"])
    py_ = cy1 + pi["standoff"]                        # the board's underside
    hx, hy = pi["holes"]
    hi_ = pi["hole_inset"]
    L["piHoles"] = {"at": [[sx * hx / 2, zu + bl - hi_ - k * hy] for k in (0, 1) for sx in (1, -1)],
                    "y": [cy0 - 1.0, cy1 + 1.0], "cb": [cy0 - 1.0, cy0 + 1.6]}
    mocks = []
    mocks.append(("battery", "drive",
                  [-bt["size"][0] / 2, bt["y0"], D["Zp"] + bt["gap"] + bt["floor"]],
                  [bt["size"][0] / 2, bt["y0"] + bt["size"][1],
                   D["Zp"] + bt["gap"] + bt["floor"] + bt["size"][2]], [0.2, 0.35, 0.75]))
    mocks.append(("Pi 3B+", "drive", [-bw / 2, py_, zu], [bw / 2, py_ + bth, zu + bl], [0.1, 0.5, 0.2]))
    ux, uz_, uh = pi["usb_block"]
    port = zu - 2.0                                   # the ports' faces
    mocks.append(("Pi USB + Ethernet", "drive", [-ux / 2, py_ + bth, port],
                  [ux / 2, py_ + bth + uh, port + uz_], [0.75, 0.75, 0.78]))
    mocks.append(("Pi GPIO header", "drive", [-bw / 2 + 1.0, py_ + bth, zu + bl - 7.0 - 51.0],
                  [-bw / 2 + 6.0, py_ + bth + 8.5, zu + bl - 7.0], [0.15, 0.15, 0.15]))
    # the two cables (user): USB-A plugs in the left stack, a 6 mm hard stub,
    # then 6 mm-radius bends down past the carrier's lower edge and back
    # under it into each device's USB-C plug. The upper plug's cable nests
    # outside the lower one's and shifts across to reach the AHRS.
    us = el["usb"]
    ux0, (uw, ul) = us["x0"], us["plug"]
    sd, sl_ = us["stub"]
    br = us["bend_r"]
    cpw, cpt, cpl = us["c_plug"]
    bt_ = py_ + bth                                   # the board's top
    for k, (y0_, y1_) in enumerate(((bt_, bt_ + uh / 2), (bt_ + uh / 2, bt_ + uh))):
        mocks.append((f"USB-A plug {k + 1}", "drive", [ux0, y0_, port - ul], [ux0 + uw, y1_, port],
                      [0.2, 0.2, 0.2]))
    xc = ux0 + uw / 2
    ends = {}
    for nm, key in (("U2D2", "u2d2"), ("AHRS", "ahrs")):
        d = el[key]
        xm = d["x0"] + d["size"][0] / 2
        ym = cy0 - d["size"][2] / 2
        w1 = W_(d["u0"])                              # the device's end
        mocks.append((f"USB-C plug ({nm})", "drive", [xm - cpw / 2, ym - cpt / 2, w1 - cpl],
                      [xm + cpw / 2, ym + cpt / 2, w1], [0.2, 0.2, 0.2]))
        ends[nm] = (xm, ym, w1 - cpl)
    wA = port - ul - sl_ - br                         # the lower cable's corner: stub, then the bend
    if W_(0) < wA + br + sd / 2 + 0.5:
        raise ValueError("the carrier's lower edge reaches the cables' turn")
    wB = wA - us["nest"]
    yl, yh = bt_ + uh / 4, bt_ + 3 * uh / 4           # the two plugs' centres
    xA, yA, eA = ends["U2D2"]
    xB, yB, eB = ends["AHRS"]
    cs_ = us["c_stub"]
    cables = [("USB cable to U2D2", [[xc, yl, port - ul], [xc, yl, wA], [xc, yA, wA],
                                     [xA, yA, eA - cs_], [xA, yA, eA]]),
              ("USB cable to AHRS", [[xc, yh, port - ul], [xc, yh, wB], [xc + us["shift"], yB, wB],
                                     [xB, yB, eB - cs_], [xB, yB, eB]])]
    L["cables"] = []
    for nm, pts in cables:
        P = rounded_path(pts, br)
        # round joints only where the path turns, and not within a radius of
        # either end (one at the last bend sample poked into the USB-C plug)
        import numpy as np
        A = np.array(P)
        js = [i for i in range(1, len(P) - 1)
              if np.linalg.norm(np.cross(A[i] - A[i - 1], A[i + 1] - A[i])) > 1e-6
              and np.linalg.norm(A[i] - A[0]) > sd / 2 + 0.1 and np.linalg.norm(A[i] - A[-1]) > sd / 2 + 0.1]
        L["cables"].append({"name": nm, "pts": P, "joints": js, "r": sd / 2})
    rails = []
    for nm, key, col in (("U2D2", "u2d2", [0.2, 0.2, 0.25]), ("AHRS", "ahrs", [0.55, 0.15, 0.15]),
                         ("power board", "pdb", [0.15, 0.4, 0.15])):
        d = el[key]
        sx_, su_, sh = d["size"]
        lo = [d["x0"], cy0 - sh, W_(d["u0"])]
        hi = [d["x0"] + sx_, cy0, W_(d["u0"] + su_)]
        mocks.append((nm, "drive", lo, hi, col))
        t, h, c = el["rail"]
        x0, x1, za, zb = lo[0] - c, hi[0] + c, lo[2] - c, hi[2] + c
        yr0 = cy0 - h
        sides = [[[x0 - t, yr0, za - t], [x0, cy0 + 0.5, zb + t]],
                 [[x1, yr0, za - t], [x1 + t, cy0 + 0.5, zb + t]],
                 [[x0 - t, yr0, zb], [x1 + t, cy0 + 0.5, zb + t]]]
        if key == "pdb":                   # the U2D2 and AHRS plug in from the carrier's lower edge
            sides.append([[x0 - t, yr0, za - t], [x1 + t, cy0 + 0.5, za]])
        rails += [(nm, b) for b in sides]
    # a rail between two devices packed edge to edge, or over a post or the
    # switch, would cut into its neighbour: drop it
    devs = [(m[0], m[2], m[3]) for m in mocks if m[0] in ("U2D2", "AHRS", "power board")]
    devs += [("post", [x - pr - 1.0, fy, z - pr - 1.0], [x + pr + 1.0, ptop, z + pr + 1.0]) for x, z in posts]
    sw = el["switch"]
    co, ca = sw["cutout"]
    tx = sw["tab_x"]
    sgn = 1 if tx > 0 else -1
    inner = tx - sgn * sw["panel"]
    swz = W_(sw["u"])
    devs.append(("switch", [min(inner, inner - sgn * sw["depth"]) - 0.5, cy0 - 1.5 - co, swz - ca / 2 - 0.5],
                 [max(inner, inner - sgn * sw["depth"]) + 0.5, cy0, swz + ca / 2 + 0.5]))
    L["rails"] = [b for nm, b in rails
                  if not any(o != nm and all(b[0][i] < hi[i] and lo[i] < b[1][i] for i in range(3))
                             for o, lo, hi in devs)]
    # nothing under the carrier may sit on a post or its boss
    for nm, lo, hi in devs:
        if nm == "post":
            continue
        for x, z in posts:
            if lo[0] < x + pr + 1.0 and x - pr - 1.0 < hi[0] and lo[2] < z + pr + 1.0 and z - pr - 1.0 < hi[2]:
                raise ValueError(f"the {nm} sits on the post at ({x:g}, {z:g}): move it or the edge")
    sy0, sy1 = cy0 - 1.5 - co, cy0 - 1.5                 # the cutout, out of the face
    L["sw"] = {"tab": [[min(tx, tx - sgn * sw["panel"]), sy0 - 2.0, swz - ca / 2 - 3.0],
                       [max(tx, tx - sgn * sw["panel"]), cy0 + 0.5, swz + ca / 2 + 3.0]],
               "cut": [[tx - 1 - (sw["panel"] if sgn > 0 else 0), sy0, swz - ca / 2],
                       [tx + 1 + (sw["panel"] if sgn < 0 else 0), sy1, swz + ca / 2]]}
    mocks.append(("switch", "drive", [min(inner, inner - sgn * sw["depth"]), sy0, swz - ca / 2],
                  [max(inner, inner - sgn * sw["depth"]), sy1, swz + ca / 2], [0.1, 0.1, 0.1]))
    bz = sw["bezel"]
    mocks.append(("switch bezel", "drive", [min(tx, tx + sgn * bz[2]), (sy0 + sy1) / 2 - bz[0] / 2, swz - bz[1] / 2],
                  [max(tx, tx + sgn * bz[2]), (sy0 + sy1) / 2 + bz[0] / 2, swz + bz[1] / 2], [0.8, 0.1, 0.1]))
    L["bosses"] = bosses

    # ---- battery tray, DRIVE frame
    Zp = D["Zp"]
    fz0 = Zp + bt["gap"]
    fz1 = fz0 + bt["floor"]
    wi = bt["size"][0] / 2 + bt["clearance"]
    wo = wi + bt["wall"]
    ya = bt["y0"] - bt["clearance"]
    yb = bt["y0"] + bt["size"][1] + bt["clearance"]
    yt0, yt1 = D["Ytab0"] + D["fixGap"], D["Yfront"]
    sl, sh = bt["strap"]
    L["tray"] = {
        "floor": [[-wo, ya - bt["wall"], fz0], [wo, yt0, fz1]],
        "tongue": [[-wo, yt0 - 1.0, Zp], [wo, yt1, Zp + jp]],
        "walls": [[[-wo, ya - bt["wall"], fz0], [-wi, yb, fz1 + bt["wall_height"]]],
                  [[wi, ya - bt["wall"], fz0], [wo, yb, fz1 + bt["wall_height"]]],
                  [[-wo, ya - bt["wall"], fz0], [wo, ya, fz1 + bt["wall_height"]]],
                  [[-wo, yb, fz0], [wo, yb + bt["wall"], fz1 + bt["wall_height"]]]],
        "straps": [[[-wo - 1, yc - sl / 2, fz1 + 0.5], [wo + 1, yc - sl / 2 + sl, fz1 + 0.5 + sh]]
                   for yc in (bt["y0"] + bt["size"][1] * 0.28, bt["y0"] + bt["size"][1] * 0.72)],
        "pt": [3.0, (yt0 + yt1) / 2, Zp + jp / 2]}
    # tongue screw, nut slot out through the block's face (under the carrier)
    joint("jTray", "drive", [bt["joint_x"], (D["yF0"] + D["yF1"]) / 2, Zp + jp], [0, 0, -1], [0, 1, 0],
          14.0, "tray", "chassis", dZ, [0, 0, 1])
    L["mocks"] = [{"name": n, "frame": f, "lo": lo, "hi": hi, "color": c}
                  for n, f, lo, hi, c in mocks]

    # ---- roll cage
    th, w = rc["spine"]
    fh = rc["foot_half"]
    # the rear foot on the carrier's top past the Pi, the front foot behind
    # the steering plate's extension; the bars end on the feet's OUTER faces
    rear_foot = [[-fh, cy1, cgz - 6.0], [fh, cy1 + jp, cgz + 6.0]]
    rf = drv([0, cy1 + jp, cgz])
    front_foot = [[-fh, S["plateBack"] - jp, S["upTopZ"] + 1.5],
                  [fh, S["plateBack"], S["upTopZ"] + ch["steer_ext"]]]
    zj = S["upTopZ"] + (1.5 + ch["steer_ext"]) / 2
    ff = ste([0, S["plateBack"] - jp, zj])
    # what the rail must clear: every corner of the electronics' envelopes
    tops = []
    for m in mocks:
        if m[1] != "drive" or m[0] == "battery":
            continue
        for x in (m[2][0], m[3][0]):
            for y in (m[2][1], m[3][1]):
                for z in (m[2][2], m[3][2]):
                    tops.append(drv([x, y, z]))
    plate_top = []
    for b in L["carrier"]["plates"]:
        for x in (b[0][0], b[1][0]):
            for z in (b[0][2], b[1][2]):
                plate_top.append(drv([x, b[1][1], z]))
    rz = max(max(q[2] for q in tops) + rc["clearance"], max(q[2] for q in plate_top) + 1.0) + w / 2
    # the rail runs level, then down to the front foot: the bend as far
    # forward as the clearance over the envelopes allows
    def ok(yb_):
        for q in tops:
            if yb_ <= q[1] <= ff[1]:
                zl = rz + (ff[2] - rz) * (q[1] - yb_) / (ff[1] - yb_)
                if zl - w / 2 * math.hypot(1, (ff[2] - rz) / (ff[1] - yb_)) < q[2] + rc["clearance"]:
                    return False
        return True
    # start over the highest point and step forward until the slope clears
    # everything under it (the cable keep-out reaches further forward than
    # the front foot, but low, where the slope is far above it)
    y_bend = max(tops, key=lambda q: q[2])[1]
    while not ok(y_bend):
        y_bend += 1.0
        if y_bend > ff[1] - 10.0:
            raise ValueError("the cage's slope cannot clear the electronics: raise the rail or move the foot")
    ry0 = rc["rail_y0"]
    segs = [[[ry0, rz], [y_bend, rz]],
            [[rf[1], rf[2]], [rf[1], rz]],
            [[y_bend, rz], [ff[1], ff[2]]]]
    # the point that finds the spine: on the rail midway between two ribs,
    # clear of every rib joint's bore and nut slot (one at 42.4 was in a slot)
    ys = sorted(rc["ribs_y"])
    pt_y = (ys[0] + ys[1]) / 2 if len(ys) > 1 else (ry0 + y_bend) / 2
    L["spine"] = {"segs": segs, "half": th / 2, "w": w, "rearFoot": rear_foot, "frontFoot": front_foot,
                  "pt": [0, pt_y, rz]}
    joint("jSpR", "drive", [0, cy1 + jp, cgz], [0, -1, 0], [1, 0, 0], 8.0, "spine", "carrier",
          [1, 0, 0], dmY)
    joint("jSpF", "steer", [0, S["yP"], zj], [0, -1, 0], [1, 0, 0], th / 2 + 4.0, "chassis", "spine",
          [0, 0, 1], [1, 0, 0])
    rt_y, rt_r = rc["rib"]
    ribs = []
    top = rz + w / 2
    sad = th / 2 + 2.0                          # the saddle's half-width, over the rail's
    for k, y in enumerate(rc["ribs_y"]):
        if not ry0 < y < y_bend:
            raise ValueError(f"rib at y {y:g} is off the level rail ({ry0:g}..{y_bend:.1f})")
        # the inner arc meets the rail's flat top at |x| = sad, and a saddle
        # fills the sliver under the crown: a flat seat, not a line contact
        zc = top - math.sqrt((rc["rib_radius"] - rt_r / 2) ** 2 - sad ** 2)
        ribs.append({"y": y - rt_y / 2, "t": rt_y, "zc": zc, "r": rc["rib_radius"], "tr": rt_r,
                     "half": rc["rib_half_angle"],
                     "saddle": [[-sad, y - rt_y / 2, top], [sad, y + rt_y / 2, zc + rc["rib_radius"]]],
                     # off the crown: the joint's bore runs down the middle
                     "pt": [10.0, y, zc + math.sqrt(rc["rib_radius"] ** 2 - 100.0)]})
        joint(f"jRib{k}", "bike", [0, y, zc + rc["rib_radius"] + rt_r / 2], [0, 0, -1], [1, 0, 0],
              th / 2 + 3.0, f"rib{k}", "spine", [0, 1, 0], [1, 0, 0])
    L["ribs"] = ribs
    L["joints"] = joints
    return L


def _fs(v) -> str:
    return cr._fs(v)



# --------------------------------------------------------------------------
# FeatureScript
# --------------------------------------------------------------------------

FS = r'''FeatureScript %VERSION%;
import(path : "onshape/std/geometry.fs", version : "%VERSION%.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_bike --push bike_features
 *
 * The drive, steering and righting modules' geometry layers, merged (each
 * module's own studio is generated by aow_sim.cad_drive / cad_steering /
 * cad_righting from the same config), then the bike's own parts. Numbers
 * from config/bike_cad.yaml through aow_sim.cad_bike's layout(), carried
 * here as BK. Millimetres.
 *
 * BIKE FRAME: origin on the rear axle's centre, +Y forward, +X right, +Z up.
 */

%MODULES%

export const BK = %BK%;

/** A bike-frame vector from a plain [x, y, z] in mm. */
export function bkV(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]) * millimeter;
}

/** A box between two corners given in `cs`, as plain mm arrays. */
export function bkBox(context is Context, id is Id, cs is CoordSystem, b is array) returns Query
{
    return boxIn(context, id, cs, bkV(b[0]), bkV(b[1]));
}

/** A prism on a YZ polygon (plain mm), from x = -half to +half. */
export function bkPrismYZ(context is Context, id is Id, tag is string, pts is array, half is number) returns Query
{
    var p2 = [];
    for (var q in pts)
        p2 = append(p2, vector(q[0], q[1]) * millimeter);
    polyPrism(context, id, tag, vector(-half, 0, 0) * millimeter, vector(1, 0, 0), vector(0, 1, 0),
              p2, 2 * half * millimeter);
    return qCreatedBy(id + (tag ~ "Ext"), EntityType.BODY);
}

/** A bar of width w between two YZ points, round-ended, x = -half..half. */
export function bkBar(context is Context, id is Id, tag is string, a is array, b is array,
                      w is number, half is number, round is boolean) returns array
{
    const d = normalize(vector(b[0] - a[0], b[1] - a[1]));
    const n = vector(-d[1], d[0]) * w / 2;
    const q = bkPrismYZ(context, id, tag, [[a[0] + n[0], a[1] + n[1]], [b[0] + n[0], b[1] + n[1]],
                                         [b[0] - n[0], b[1] - n[1]], [a[0] - n[0], a[1] - n[1]]], half);
    if (!round)
        return [q];
    // round ends ONLY where nothing else meets the bar: a circle the bar's
    // own width is tangent to any other bar of that width through its centre,
    // and a tangent union comes back non-manifold
    const ca = cylW(context, id + (tag ~ "A"), bkV([-half, a[0], a[1]]), bkV([half, a[0], a[1]]), w / 2 * millimeter);
    const cb = cylW(context, id + (tag ~ "B"), bkV([-half, b[0], b[1]]), bkV([half, b[0], b[1]]), w / 2 * millimeter);
    return [q, ca, cb];
}

/** An arch across Y: centreline radius r about (x 0, zc), radial thickness
 * tr, `half` degrees each side of the top, from y for t along +Y. */
export function bkArc(context is Context, id is Id, tag is string, rb is map) returns Query
{
    var pts = [];
    const n = 24;
    for (var i = 0; i <= n; i += 1)
    {
        const a = (-rb.half + 2 * rb.half * i / n) * degree;
        pts = append(pts, vector((rb.r + rb.tr / 2) * sin(a), rb.zc + (rb.r + rb.tr / 2) * cos(a)) * millimeter);
    }
    for (var i = n; i >= 0; i -= 1)
    {
        const a = (-rb.half + 2 * rb.half * i / n) * degree;
        pts = append(pts, vector((rb.r - rb.tr / 2) * sin(a), rb.zc + (rb.r - rb.tr / 2) * cos(a)) * millimeter);
    }
    // sketched on the plane y = rb.y, normal +Y, whose own x is -X
    var p2 = [];
    for (var q in pts)
        p2 = append(p2, vector(-q[0], q[1]));
    polyPrism(context, id, tag, vector(0, rb.y, 0) * millimeter, vector(0, 1, 0), vector(-1, 0, 0),
              p2, rb.t * millimeter);
    return qCreatedBy(id + (tag ~ "Ext"), EntityType.BODY);
}

/**
 * One 6-32 x 3/8 flat head joint like the modules' `screwJoint`, but with a
 * ONE-WAY nut slot of `slotLen`, out along `slotDir` to the face the nut
 * goes in from -- so it cannot cut through everything else in a part the
 * size of the chassis. No ridge. Teardrops and the slot's bridging layers as
 * `screwJoint` does them.
 */
export function bkJoint(context is Context, id is Id, head is Vector, zIn is Vector, slotDir is Vector,
                        slotLen is ValueWithUnits, cskQ is Query, nutQ is Query,
                        cskUp is Vector, nutUp is Vector)
{
    // RESOLVE THE PARTS FIRST: a part found by a point inside it (partAt)
    // also finds this joint's own screw cutter once it exists, if the point
    // is on the screw's axis -- and the cut then subtracts the cutter from
    // itself (BOOLEAN_INVALID, 2026-09-30, the cage ribs)
    const cskPart = qUnion(evaluateQuery(context, cskQ));
    const nutPart = qUnion(evaluateQuery(context, nutQ));
    const mm = millimeter;
    const f  = FIXTURE;
    const cs = coordSystem(head, slotDir, zIn);
    const nd = f.joint_plate + 2 * mm;
    screwJointGeometry(context, id + "scr", cs, mergeMaps(SCREW_OPT, {
            "nutDepth" : nd, "bothWays" : false, "nutSlotLength" : slotLen }));
    opBoolean(context, id + "slotCut", { "tools" : qCreatedBy(id + "scr" + "slotExt", EntityType.BODY),
            "targets" : nutPart, "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "cut", { "tools" : qCreatedBy(id + "scr" + "screwRev", EntityType.BODY),
            "targets" : qUnion([cskPart, nutPart]), "operationType" : BooleanOperationType.SUBTRACTION });
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
                "targets" : pu[0], "operationType" : BooleanOperationType.SUBTRACTION });
    }
    const along = dot(nutUp, zIn);
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

/** Every solid body under `sid`. */
export function bkSolids(context is Context, sid is Id) returns Query
{
    return qBodyType(qCreatedBy(sid, EntityType.BODY), BodyType.SOLID);
}

/**
 * The whole bike. opt: steer (angle), pose (a RG_POSES key), mocks (bool:
 * the electronics and battery envelopes). Returns the groups the check
 * sweeps and the new parts' print orientations.
 */
export function bikeBuild(context is Context, id is Id, opt is map) returns map
{
    const mm = millimeter;
    const X  = vector(1, 0, 0);
    const Y  = vector(0, 1, 0);
    const Z  = vector(0, 0, 1);
    const O  = vector(0, 0, 0) * mm;
    const W  = coordSystem(O, X, Z);
    const csD = coordSystem(O, X, vector(BK.dAxis[0], BK.dAxis[1], BK.dAxis[2]));
    const csS = coordSystem(bkV(BK.sOrigin), X, vector(BK.sAxis[0], BK.sAxis[1], BK.sAxis[2]));
    const dXf = rotationAround(line(O, X), BK.tilt * degree);
    const sXf = transform(bkV(BK.sOrigin)) * rotationAround(line(O, X), BK.rake * degree);
    const rXf = transform(vector(0, BK.yr, BK.zr) * mm);
    const toD = function(v is Vector) returns Vector { return (toWorld(csD, v * mm) - toWorld(csD, O)) / mm; };
    const toS = function(v is Vector) returns Vector { return (toWorld(csS, v * mm) - toWorld(csS, O)) / mm; };
    const P = id + "bk";
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };

    step("modules");
    // ---- the three modules, each in its own frame
    const STi = id + "st";
    const DRi = id + "dr";
    const RGi = id + "rg";
    const rs = steeringBuild(context, STi, { "attach" : "SCREWS", "key" : "CROSS" });
    driveBuild(context, DRi, { "shift" : 0 * mm });
    const rr = rightingBuild(context, RGi, { "mocks" : true });
    // NO NAMES DURING REGENERATION: getProperty works in an eval and throws
    // in a Part Studio (2026-09-30), so every part is found by a point
    opDeleteBodies(context, id + "floor", { "entities" : partAt(context, RGi, bkV(BK.rgFloorPt)) });
    // which module each body came from, for the check's names
    for (var g in [[STi, "steering"], [DRi, "drive"], [RGi, "righting"]])
        setAttribute(context, { "entities" : bkSolids(context, g[0]), "name" : "bk_mod", "attribute" : g[1] });

    step("steering plate");
    // the steering's mount plate becomes the chassis's front: the bench tab
    // and foot go, an extension rises above the upper case
    const Q = id + "pre";       // an id's operations must be contiguous: P comes after the moves
    const stPlate = qUnion(evaluateQuery(context, partAt(context, STi, bkV(BK.stPlatePt))));
    var trims = [];
    for (var k = 0; k < size(BK.stTrim); k += 1)
        trims = append(trims, bkBox(context, Q + ("stTrim" ~ k), W, BK.stTrim[k]));
    opBoolean(context, Q + "stTrimCut", { "tools" : qUnion(trims), "targets" : stPlate,
            "operationType" : BooleanOperationType.SUBTRACTION });
    const stExt = bkBox(context, Q + "stExt", W, BK.stExt);
    opBoolean(context, Q + "stExtU", { "tools" : qUnion([stPlate, stExt]),
            "operationType" : BooleanOperationType.UNION });

    step("place");
    // tag what moves BEFORE anything does: the modules' stage and group
    // queries find their bodies by a point inside, which a moved body no
    // longer contains (the AHRS fixture's check learned this first)
    for (var q in rs.stages["steer"])
        setAttribute(context, { "entities" : q, "name" : "bk_steer", "attribute" : "steer" });
    for (var g in ["crank", "cR", "cL", "wR", "wL"])
        for (var q in rr.groups[g])
            setAttribute(context, { "entities" : q, "name" : "bk_" ~ g, "attribute" : g });

    // poses in the modules' own frames, before they move
    if (opt.steer != undefined && opt.steer != 0 * degree)
        opTransform(context, id + "steer", { "bodies" : qHasAttribute("bk_steer"),
                "transform" : rotationAround(rs.axis, opt.steer) });
    if (opt.pose != undefined && opt.pose != "+0.00")
        rgPose(context, id + "pose", rr.groups, opt.pose, false);

    opTransform(context, id + "stXf", { "bodies" : bkSolids(context, STi), "transform" : sXf });
    opTransform(context, id + "drXf", { "bodies" : bkSolids(context, DRi), "transform" : dXf });
    opTransform(context, id + "rgXf", { "bodies" : bkSolids(context, RGi), "transform" : rXf });

    // ---- chassis: the deck, the wedge up to the drive block, the gusset
    // behind the steering plate, the posts, and the three placeholders
    const dk = BK.deck;
    const elec = opt.electronics != false;      // false: no carrier, posts, cage or envelopes (the --fit e probe)
    step("chassis");
    var chq = [bkPrismYZ(context, P, "wedge", BK.wedge.pts, BK.wedge.half),
               bkPrismYZ(context, P, "gusset", BK.gusset.pts, BK.gusset.half)];
    for (var k = 0; k < size(BK.deck.boxes); k += 1)
        chq = append(chq, bkBox(context, P + ("deck" ~ k), W, BK.deck.boxes[k]));
    for (var k = 0; k < (elec ? size(BK.posts.cyl) : 0); k += 1)
        chq = append(chq, cylIn(context, P + ("post" ~ k), csD, bkV(BK.posts.cyl[k][0]), bkV(BK.posts.cyl[k][1]),
                                BK.posts.r * mm));
    for (var pt in BK.holdPts)
        chq = append(chq, partAt(context, id, bkV(pt)));
    opBoolean(context, P + "chU", { "tools" : qUnion(chq), "operationType" : BooleanOperationType.UNION });
    const chassis = function() returns Query { return partAt(context, id, bkV(BK.deckPt)); };
    var tdq = [];
    const rB = SCREW_OPT.holeDia / 2;
    for (var k = 0; k < size(BK.blockBores); k += 1)
    {
        const c = BK.blockBores[k];
        // sketched on the YZ plane, its x = +Y, its y = +Z: the point up
        polyPrism(context, P, "blkTd" ~ k, vector(-BK.blockBoreX, c[1], c[2]) * mm, X, Y,
                  [vector(0 * mm, 0 * mm), vector(rB / sqrt(2), rB / sqrt(2)),
                   vector(0 * mm, rB * sqrt(2)), vector(-rB / sqrt(2), rB / sqrt(2))], 2 * BK.blockBoreX * mm);
        tdq = append(tdq, qCreatedBy(P + ("blkTd" ~ k ~ "Ext"), EntityType.BODY));
    }
    opBoolean(context, P + "blkTdCut", { "tools" : qUnion(tdq), "targets" : chassis(),
            "operationType" : BooleanOperationType.SUBTRACTION });

    step("tray");
    // ---- battery tray, drive frame
    const tr = BK.tray;
    var tq = [bkBox(context, P + "trFloor", csD, tr.floor), bkBox(context, P + "trTongue", csD, tr.tongue)];
    for (var k = 0; k < size(tr.walls); k += 1)
        tq = append(tq, bkBox(context, P + ("trWall" ~ k), csD, tr.walls[k]));
    opBoolean(context, P + "trU", { "tools" : qUnion(tq), "operationType" : BooleanOperationType.UNION });
    const tray = function() returns Query { return partAt(context, P, toWorld(csD, bkV(tr.pt))); };
    var sq = [];
    for (var k = 0; k < size(tr.straps); k += 1)
        sq = append(sq, bkBox(context, P + ("trStrap" ~ k), csD, tr.straps[k]));
    opBoolean(context, P + "trStrapCut", { "tools" : qUnion(sq), "targets" : tray(),
            "operationType" : BooleanOperationType.SUBTRACTION });

    var carrier = function() returns Query { return qNothing(); };
    var spine = function() returns Query { return qNothing(); };
    if (elec)
    {
        step("carrier");
        // ---- electronics carrier, drive frame: plate, bosses, cradle rails,
        // the switch tab; holes for the Pi's standoffs
        var cq = [bkBox(context, P + "carrier0", csD, BK.carrier.plates[0]),
                  bkBox(context, P + "carrier1", csD, BK.carrier.plates[1])];
        for (var k = 0; k < size(BK.bosses); k += 1)
            cq = append(cq, cylIn(context, P + ("boss" ~ k), csD, bkV(BK.bosses[k][0]), bkV(BK.bosses[k][1]),
                                  BK.bosses[k][2] * mm));
        for (var k = 0; k < size(BK.rails); k += 1)
            cq = append(cq, bkBox(context, P + ("rail" ~ k), csD, BK.rails[k]));
        cq = append(cq, bkBox(context, P + "swTab", csD, BK.sw.tab));
        opBoolean(context, P + "cU", { "tools" : qUnion(cq), "operationType" : BooleanOperationType.UNION });
        carrier = function() returns Query { return partAt(context, P, toWorld(csD, bkV(BK.carrier.pt))); };
        var hq = [bkBox(context, P + "swCut", csD, BK.sw.cut)];
        const ph = BK.piHoles;
        for (var k = 0; k < size(ph.at); k += 1)
        {
            const h = ph.at[k];
            hq = append(hq, cylIn(context, P + ("piH" ~ k), csD, bkV([h[0], ph.y[0], h[1]]), bkV([h[0], ph.y[1], h[1]]), 1.45 * mm));
            hq = append(hq, cylIn(context, P + ("piC" ~ k), csD, bkV([h[0], ph.cb[0], h[1]]), bkV([h[0], ph.cb[1], h[1]]), 2.6 * mm));
        }
        opBoolean(context, P + "cCut", { "tools" : qUnion(hq), "targets" : carrier(),
                "operationType" : BooleanOperationType.SUBTRACTION });

        step("cage");
        // ---- roll cage: the spine, then the ribs
        const sp = BK.spine;
        var spq = [bkBox(context, P + "spRearFoot", csD, sp.rearFoot), bkBox(context, P + "spFrontFoot", csS, sp.frontFoot)];
        for (var k = 0; k < size(sp.segs); k += 1)
            spq = concatenateArrays([spq, bkBar(context, P, "spSeg" ~ k, sp.segs[k][0], sp.segs[k][1], sp.w, sp.half, k == 0)]);
        opBoolean(context, P + "spU", { "tools" : qUnion(spq), "operationType" : BooleanOperationType.UNION });
        spine = function() returns Query { return partAt(context, P, bkV(sp.pt)); };
        for (var k = 0; k < size(BK.ribs); k += 1)
        {
            const arc = bkArc(context, P, "rib" ~ k, BK.ribs[k]);
            opBoolean(context, P + ("ribU" ~ k), { "tools" : qUnion([arc, bkBox(context, P + ("ribSad" ~ k), W, BK.ribs[k].saddle)]),
                    "operationType" : BooleanOperationType.UNION });
        }
    }

    // ---- joints, now that every part is in place: data, frame by frame
    step("joints");
    var parts = { "chassis" : chassis(), "tray" : tray(), "carrier" : carrier(), "spine" : spine() };
    for (var k = 0; k < size(BK.ribs); k += 1)
        parts["rib" ~ k] = elec ? partAt(context, P, bkV(BK.ribs[k].pt)) : qNothing();
    for (var j in BK.joints)
    {
        if (!elec && j.tag != "jTray")
            continue;
        const cs = j.frame == "drive" ? csD : (j.frame == "steer" ? csS : W);
        const dv = function(a is array) returns Vector
        {
            return (toWorld(cs, bkV(a)) - toWorld(cs, O)) / mm;
        };
        bkJoint(context, P + j.tag, toWorld(cs, bkV(j.head)), dv(j.zIn), dv(j.slot), j.len * mm,
                parts[j.csk], parts[j.nut], vector(j.cskUp[0], j.cskUp[1], j.cskUp[2]),
                vector(j.nutUp[0], j.nutUp[1], j.nutUp[2]));
    }

    step("mocks");
    // ---- hardware envelopes
    if (opt.mocks != false)
        for (var k = 0; k < size(BK.mocks); k += 1)
        {
            const m = BK.mocks[k];
            if (!elec && m.name != "battery")
                continue;
            const q = bkBox(context, id + ("mock" ~ k), m.frame == "drive" ? csD : W, [m.lo, m.hi]);
            dress(context, q, m.name, color(m.color[0], m.color[1], m.color[2]), "Envelope, not a print.");
        }
    // the cables: a 6 mm pipe along each path, round at every sample point
    if (opt.mocks != false && elec)
        for (var k = 0; k < size(BK.cables); k += 1)
        {
            const cb = BK.cables[k];
            const C = id + ("cable" ~ k);
            var cq = [];
            for (var i = 0; i + 1 < size(cb.pts); i += 1)
                cq = append(cq, cylIn(context, C + ("s" ~ i), csD, bkV(cb.pts[i]), bkV(cb.pts[i + 1]), cb.r * mm));
            for (var i in cb.joints)
            {
                opSphere(context, C + ("j" ~ i), { "center" : toWorld(csD, bkV(cb.pts[i])), "radius" : cb.r * mm });
                cq = append(cq, qCreatedBy(C + ("j" ~ i), EntityType.BODY));
            }
            opBoolean(context, C + "u", { "tools" : qUnion(cq), "operationType" : BooleanOperationType.UNION });
            dress(context, qUnion(cq), cb.name, color(0.95, 0.85, 0.2), "6 mm cable, 6 mm-radius bends (user). Not a print.");
        }

    const frameC = color(0.62, 0.64, 0.68);
    const cageC  = color(0.85, 0.3, 0.1);
    dress(context, chassis(), "chassis [print Z+]", frameC,
          "The deck over the righting's bridge, the wedge up to the drive block, the steering plate, the carrier's posts off the block's face. Print the deck down.");
    dress(context, tray(), "battery tray [print +Z drive]", frameC,
          "On the drive's back between the pulleys; the tongue screws onto the drive block. A strap through the wall slots.");
    if (elec)
    dress(context, carrier(), "electronics carrier [print underside up]", frameC,
          "Leans on the drive block's front face on four chassis posts. Pi 3B+ on top (M2.5 standoffs, screws from below), USB edge down the slope; U2D2 / AHRS / power board in the cradles underneath; the switch in the left tab; the cage's rear foot on the upper end. Print the underside up.");
    var prints = [["chassis", chassis(), Z], ["battery tray", tray(), toD(Z)]];
    if (elec)
    {
        dress(context, spine(), "cage spine [print X]", cageC, "The roll bar: the carrier's upper end to the steering plate. Print on its side.");
        prints = concatenateArrays([prints, [["electronics carrier", carrier(), -toD(Y)], ["cage spine", spine(), X]]]);
    }
    for (var k = 0; k < (elec ? size(BK.ribs) : 0); k += 1)
    {
        const q = partAt(context, P, bkV(BK.ribs[k].pt));
        dress(context, q, "cage rib " ~ (k + 1) ~ " [print Y]", cageC, "An arch across the spine. Print flat.");
        prints = append(prints, ["cage rib " ~ (k + 1), q, Y]);
    }

    return { "steer" : rs, "righting" : rr, "prints" : prints,
             "sXf" : sXf, "rXf" : rXf, "sAxis" : line(bkV(BK.sOrigin), toS(Z)) };
}

%SPLIT%

export enum BikePose
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

annotation { "Feature Type Name" : "AOW bike",
             "Feature Type Description" : "The whole bike: drive, steering and righting modules placed, with the chassis, battery tray, electronics carrier and roll cage" }
export const aowBike = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Steer angle" }
        isAngle(definition.steer, { (degree) : [-180, 0, 180] } as AngleBoundSpec);
        annotation { "Name" : "Righting pose" }
        definition.pose is BikePose;
        annotation { "Name" : "Show battery and electronics envelopes", "Default" : true }
        definition.mocks is boolean;
    }
    {
        const keys = { BikePose.REST : "+0.00", BikePose.R50 : "+0.50", BikePose.R100 : "+1.00",
                       BikePose.L50 : "-0.50", BikePose.L100 : "-1.00" };
        bikeBuild(context, id + "build", { "steer" : definition.steer, "pose" : keys[definition.pose],
                                           "mocks" : definition.mocks });
        reportFeatureInfo(context, id, "%INFO%");
    });
'''


def build_fs(L: dict, fs_version: str = "3044") -> str:
    texts = module_texts()
    keep = ("wb", "yr", "zr", "zf", "tilt", "rake", "dAxis", "sAxis", "sOrigin", "deck", "wedge",
            "gusset", "stTrim", "stExt", "deckPt", "holdPts", "rgFloorPt", "stPlatePt", "blockBores",
            "blockBoreX", "carrier", "posts", "bosses", "piHoles", "sw", "rails", "tray", "mocks",
            "cables", "spine", "ribs", "joints")
    bk = {k: L[k] for k in keep}
    info = (f"Wheelbase {L['wb']:g} mm (CAD only; the sim's is 200); righting rod "
            f"{L['yr']:g} mm ahead of the rear axle, {L['zr'] - L['floor']:.1f} mm off the floor")
    subs = {"%VERSION%": fs_version, "%MODULES%": merge_layers(texts), "%BK%": _fs(bk),
            "%SPLIT%": SPLIT_MARK, "%INFO%": info}
    text = FS
    for k, v in subs.items():
        text = text.replace(k, v)
    # A Part Studio refuses getProperty during regeneration ("Cannot get
    # properties during feature regeneration") while the eval allows it, so
    # --check cannot catch one: refuse it here (2026-09-30, a blank render).
    if "getProperty" in sm._strip_comments(text):
        raise SystemExit("getProperty in the studio: it throws in a Part Studio; find parts by a point")
    return text


# --------------------------------------------------------------------------
# check
# --------------------------------------------------------------------------

def check_wrapper(fs: str) -> str:
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
    const r = bikeBuild(context, id, {{ "mocks" : true, "debug" : true }});
    const all = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    // module prefixes, for the report only (the studio cannot read names)
    for (var b in all)
    {{
        const at = getAttributes(context, {{ "entities" : b, "name" : "bk_mod" }});
        if (size(at) > 0 && !match(name(b), "chassis.*").hasMatch)
            setProperty(context, {{ "entities" : b, "propertyType" : PropertyType.NAME, "value" : at[0] ~ ": " ~ name(b) }});
    }}
    for (var b in all)
    {{
        const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
        println("BODY|" ~ name(b) ~ "|" ~ toString(bb.minCorner[0] / mm) ~ "|" ~ toString(bb.minCorner[1] / mm)
                ~ "|" ~ toString(bb.minCorner[2] / mm) ~ "|" ~ toString(bb.maxCorner[0] / mm) ~ "|"
                ~ toString(bb.maxCorner[1] / mm) ~ "|" ~ toString(bb.maxCorner[2] / mm) ~ "|"
                ~ toString(evVolume(context, {{ "entities" : b }}) / (mm * mm * mm)) ~ "|"
                ~ toString(evArea(context, {{ "entities" : qOwnedByBody(b, EntityType.FACE) }}) / (mm * mm)));
    }}
    for (var i = 0; i + 1 < size(all); i += 1)
        clashes("rest", all[i], qUnion(subArray(all, i + 1, size(all))));
    // the steer: its turning stage (tagged in bikeBuild) about the bike's steering axis
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
    // the righting: its moving groups, posed about the module's own axes
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
{af.PRINT_CHECK_FS}    return "ran to completion";
}}
"""


NEW = ("chassis", "battery tray", "electronics carrier", "cage spine", "cage rib")
SKIN = 0.9          # mm: 2 perimeters x ~0.45, and ~4-5 top/bottom layers at 0.2 -- UNCALIBRATED
INFILL = 0.15


def printed_mass(vol_mm3: float, area_mm2: float, skin: float = SKIN, infill: float = INFILL) -> float:
    """Grams as sliced: a skin of `skin` over the whole surface, sparse infill
    inside it. A back-of-the-envelope figure, not a slicer's: it ignores the
    skin doubling up at edges and thin walls printing solid beyond the cap
    below. Calibrate `skin` against a printed part on a scale."""
    shell = min(vol_mm3, area_mm2 * skin)
    return (shell + infill * (vol_mm3 - shell)) / 1000 * ASA


def _module(name: str) -> str:
    for m in ("steering", "drive", "righting"):
        if name.startswith(m + ": "):
            return m
    return "bike"


def judge(console: str) -> bool:
    rows = [l.split("|") for l in console.splitlines()]
    ok = True
    bodies = [r for r in rows if r[0] == "BODY"]
    strays = [r for r in bodies if r[1].endswith("UNNAMED")]
    if strays:
        ok = False
        print(f"  {len(strays)} UNNAMED bodies -- a cutter left behind  FAIL")
    print(f"  {len(bodies)} bodies")
    for r in bodies:
        if r[1].startswith(NEW):
            lo = [float(v) for v in r[2:5]]
            hi = [float(v) for v in r[5:8]]
            vol, area = float(r[8]), float(r[9])
            print(f"  {r[1]:44} x {lo[0]:6.1f}..{hi[0]:5.1f}  y {lo[1]:6.1f}..{hi[1]:5.1f}  "
                  f"z {lo[2]:6.1f}..{hi[2]:5.1f}  {vol / 1000 * ASA:5.1f} g solid, "
                  f"~{printed_mass(vol, area):4.1f} g printed")
    # collisions: a module's internal ones are its own check's business
    hits: dict[tuple, set] = {}
    for r in rows:
        if r[0] != "COLL" or r[4].startswith("ABUT"):
            continue
        a, b = sorted((r[2], r[3]))
        if _module(a) == _module(b) != "bike":
            continue
        hits.setdefault((a, b), set()).add(r[1])
    if hits:
        ok = False
        print(f"  {len(hits)} interfering pairs:")
        for (a, b), poses in sorted(hits.items()):
            print(f"    {a}  x  {b}   [{', '.join(sorted(poses))}]")
    else:
        print("  no interference across modules and new parts, at rest, steered "
              f"{'/'.join(str(a) for a in STEER_SWEEP)}, righting at every pose")
    for tag in ("OVER", "CROWN", "HANG"):
        for r in (r for r in rows if r[0] == tag):
            print(f"  {tag} {' | '.join(r[1:])}")
    parts = {r[1]: int(r[2]) for r in rows if r[0] == "PART"}
    bad = {k: v for k, v in parts.items() if v != 1}
    if bad:
        ok = False
        print(f"  print-check parts not found as one body: {bad}  FAIL")
    return ok


def check(text: str, target: str | None) -> bool:
    from . import onshape
    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    out = Path("traces/bike_cad/check.txt")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(console)
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console[-3000:])
        print(onshape.budget_line())
        return False
    ok = judge(console)
    print(f"  console -> {out}")
    print(onshape.budget_line())
    return ok


def report(L: dict) -> str:
    c = [L["carrier"]["plates"][0][0], L["carrier"]["plates"][1][1]]
    seg = L["spine"]["segs"]
    return "\n".join([
        f"wheelbase {L['wb']:g}; righting rod at y {L['yr']:g}, z {L['zr']:g} "
        f"({L['zr'] - L['floor']:.1f} off the floor)",
        f"deck z {L['deck']['z0']:.2f}..{L['deck']['z1']:.2f}; "
        + "; ".join(f"y {b[0][1]:.1f}..{b[1][1]:.1f} |x|<={b[1][0]:g}" for b in L["deck"]["boxes"]),
        f"carrier (drive frame) {c[1][1] - c[0][1]:g} thick, {c[0][1] - 142.9:.1f}..{c[1][1] - 142.9:.1f} "
        f"off the block's face, z {c[0][2]:g}..{c[1][2]:g} along it",
        f"cage rail z {seg[0][0][1]:.1f} from y {seg[0][0][0]:g} to {seg[0][1][0]:.1f}, then down to "
        f"the front foot at ({seg[2][1][0]:.1f}, {seg[2][1][1]:.1f}); ribs at y "
        + ", ".join(f"{r['y'] + r['t'] / 2:g}" for r in L["ribs"])])


# --------------------------------------------------------------------------
# the fit probe that set the wheelbase (2026-09-30)
# --------------------------------------------------------------------------

FIT2 = """function(context is Context, queries)
{
%LAYER%
    const mm = millimeter;
    const name = function(q) returns string
    {
        const n = getProperty(context, { "entity" : q, "propertyType" : PropertyType.NAME });
        return n == undefined ? "UNNAMED" : n;
    };
    const solids = function(sid is Id) returns Query
    {
        return qBodyType(qCreatedBy(sid, EntityType.BODY), BodyType.SOLID);
    };
    const X = vector(1, 0, 0);
    const O = vector(0, 0, 0) * mm;
    driveBuild(context, makeId("dr"), { "shift" : 0 * mm });
    opTransform(context, makeId("drT"), { "bodies" : solids(makeId("dr")),
            "transform" : rotationAround(line(O, X), %TILT% * degree) });
    steeringBuild(context, makeId("st"), { "attach" : "SCREWS", "key" : "CROSS" });
    opTransform(context, makeId("stT"), { "bodies" : solids(makeId("st")),
            "transform" : transform(vector(0, 0, %ZF%) * mm) * rotationAround(line(O, X), %RAKE% * degree) });
    // the front tyre turning 360 deg about an axis through its centre sweeps a ball
    opSphere(context, makeId("ball"), { "center" : vector(0, 0, %ZF%) * mm, "radius" : %RB% * mm });
    rightingBuild(context, makeId("rg"), { "mocks" : true });
    var gone = [];
    for (var sid in [makeId("dr"), makeId("st"), makeId("rg")])
        for (var b in evaluateQuery(context, solids(sid)))
            if (match(name(b), "(fixture mock|mount plate|chassis plate|floor).*").hasMatch)
                gone = append(gone, b);
    opDeleteBodies(context, makeId("placeholders"), { "entities" : qUnion(gone) });
    opTransform(context, makeId("rgZ"), { "bodies" : solids(makeId("rg")),
            "transform" : transform(vector(0, 0, %ZR%) * mm) });
    // the axle group alone: what is left at the rear if the tower were elsewhere
    var axleQ = [];
    for (var b in evaluateQuery(context, solids(makeId("dr"))))
        if (match(name(b), "(rear wheel|M5 axle|chainstay|spline pulley).*").hasMatch)
            axleQ = append(axleQ, b);
    const rgQ = solids(makeId("rg"));
    const frontQ = qUnion([solids(makeId("st")), solids(makeId("ball"))]);
    // 1. rear: righting slid forward from the rear axle
    var at = 0;
    for (var y = %Y0%; y <= %Y1%; y += %DY%)
    {
        opTransform(context, makeId("r" ~ y), { "bodies" : rgQ, "transform" : transform(vector(0, y - at, 0) * mm) });
        at = y;
        const cc = evCollision(context, { "tools" : rgQ, "targets" : solids(makeId("dr")) });
        println("REAR|" ~ y ~ "|" ~ size(cc)
                ~ "|" ~ size(evCollision(context, { "tools" : rgQ, "targets" : qUnion(axleQ) })));
        var seen = {};
        for (var c in cc)
        {
            const k = name(c.toolBody) ~ " x " ~ name(c.targetBody);
            if (seen[k] == undefined && c["type"] != ClashType.ABUT_NO_CLASS)
            {
                seen[k] = true;
                println("PAIR|" ~ y ~ "|" ~ k);
            }
        }
    }
    opTransform(context, makeId("rBack"), { "bodies" : rgQ, "transform" : transform(vector(0, -at, 0) * mm) });
    // 2. front: the front axle slid forward of the righting's origin
    at = 0;
    for (var f = %F0%; f <= %F1%; f += %DF%)
    {
        opTransform(context, makeId("f" ~ f), { "bodies" : frontQ,
                "transform" : transform(vector(0, f - at, 0) * mm) });
        at = f;
        println("FRONT|" ~ f ~ "|" ~ size(evCollision(context, { "tools" : rgQ, "targets" : frontQ })));
    }
}"""


def fit2_probe(rear=(140, 175, 2.5), front=(80, 90, 2.5)) -> str:
    """REAR|y|vs whole drive|vs axle group, FRONT|offset|vs steering + ball."""
    p = load_params(sm.CAD_PARAMS)
    R = p["omni_wheel"]["outer_radius"] * 1000
    st = cs.layout(cs.load())
    rL = cr.layout(cr.load())
    body = sm.geometry_layer("\n".join(["FeatureScript 3044;", merge_layers(module_texts()), SPLIT_MARK]))
    subs = {"%LAYER%": body, "%TILT%": f"{p['drivetrain']['drive_servo_angle_deg']:g}",
            "%ZF%": f"{st['R'] - R:g}", "%RAKE%": f"{p['bike']['rake_deg']:g}",
            "%RB%": f"{st['R']:g}", "%ZR%": f"{-R - rL['floor_z']:g}",
            "%Y0%": str(rear[0]), "%Y1%": str(rear[1]), "%DY%": str(rear[2]),
            "%F0%": str(front[0]), "%F1%": str(front[1]), "%DF%": str(front[2])}
    s = FIT2
    for k, v in subs.items():
        s = s.replace(k, v)
    return s


FITE = """function(context is Context, queries)
{
%LAYER%
    const mm = millimeter;
    const name = function(q) returns string
    {
        const n = getProperty(context, { "entity" : q, "propertyType" : PropertyType.NAME });
        return n == undefined ? "UNNAMED" : n;
    };
    const id = makeId("fe");
    const r = bikeBuild(context, id, { "electronics" : false });
    const rest = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    const csD = coordSystem(vector(0, 0, 0) * mm, vector(1, 0, 0), vector(BK.dAxis[0], BK.dAxis[1], BK.dAxis[2]));
    const gs = ["crank", "cR", "cL", "wR", "wL"];
    var k = 0;
    for (var cfg in %CFGS%)
    {
        const E = makeId("e" ~ k);
        var es = [];
        for (var b in %BOXES%)
        {
            const q = boxIn(context, E + b[0], csD, bkV([b[1][0], b[1][1] + cfg[1], b[1][2] + cfg[0]]),
                            bkV([b[2][0], b[2][1] + cfg[1], b[2][2] + cfg[0]]));
            setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : b[3] });
            es = append(es, q);
        }
        const eq = qUnion(es);
        // closures capture by value in FeatureScript: return, do not assign
        const note = function(pose is string) returns array
        {
            var out = [];
            for (var c in evCollision(context, { "tools" : eq, "targets" : qUnion(rest) }))
                if (c["type"] != ClashType.ABUT_NO_CLASS)
                    out = append(out, name(c.targetBody) ~ " @ " ~ pose ~ " <- " ~ name(c.toolBody));
            return out;
        };
        var hits = note("rest");
        for (var a in [90, 180, 270])
        {
            opTransform(context, E + ("t" ~ a), { "bodies" : qHasAttribute("bk_steer"), "transform" : rotationAround(r.sAxis, a * degree) });
            hits = concatenateArrays([hits, note("steer " ~ a)]);
            opTransform(context, E + ("tb" ~ a), { "bodies" : qHasAttribute("bk_steer"), "transform" : rotationAround(r.sAxis, -a * degree) });
        }
        var pk = 0;
        for (var key in ["+0.50", "+1.00", "-0.50", "-0.56", "-1.00"])
        {
            const ps = RG_POSES[key];
            for (var g in gs)
                opTransform(context, E + ("p" ~ pk ~ g), { "bodies" : qHasAttribute("bk_" ~ g),
                        "transform" : r.rXf * rgXf(ps[g]) * inverse(r.rXf) });
            hits = concatenateArrays([hits, note("pose " ~ key)]);
            for (var g in gs)
                opTransform(context, E + ("pb" ~ pk ~ g), { "bodies" : qHasAttribute("bk_" ~ g),
                        "transform" : r.rXf * inverse(rgXf(ps[g])) * inverse(r.rXf) });
            pk += 1;
        }
        println("CFG|" ~ cfg[0] ~ "|" ~ cfg[1] ~ "|" ~ size(hits));
        for (var h in hits)
            println("HIT|" ~ cfg[0] ~ "|" ~ cfg[1] ~ "|" ~ h);
        opDeleteBodies(context, E + "del", { "entities" : eq });
        k += 1;
    }
}"""


def fite_probe(L: dict, slides=(0, -2.5, -5, -7.5, -10, -12.5, -15), gaps=(0, 3)) -> str:
    """The electronics' envelopes slid down the drive's face (drive z) and off
    it (drive y), against the bike built without them: rest, steered, posed."""
    boxes = [[f"c{i}", b[0], b[1], ("carrier, narrow part", "carrier, wide part")[i]]
             for i, b in enumerate(L["carrier"]["plates"])]
    boxes.append(["sw", L["sw"]["tab"][0], L["sw"]["tab"][1], "switch tab"])
    for i, m in enumerate(L["mocks"]):
        if m["frame"] == "drive" and m["name"] != "battery":
            boxes.append([f"m{i}", m["lo"], m["hi"], m["name"]])
    body = sm.geometry_layer(build_fs(L).split(SPLIT_MARK)[0] + "\n" + SPLIT_MARK)
    cfgs = [[s_, g_] for s_ in slides for g_ in gaps]
    return (FITE.replace("%LAYER%", body).replace("%CFGS%", _fs(cfgs)).replace("%BOXES%", _fs(boxes)))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", default=PARAMS)
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="build the whole bike in Onshape and check it; ONE billable call")
    ap.add_argument("--push", metavar="TAB|URL", nargs="?", const="", default=None,
                    help=f"replace a Feature Studio's contents (`{TAB}` only)")
    ap.add_argument("--shot", metavar="TAB|URL", nargs="?", const="", default=None,
                    help=f"render a Part Studio (default `{PART_STUDIO}`); ONE billable call")
    ap.add_argument("--view", default="isometric")
    ap.add_argument("--fit", choices=["2", "e"], default=None,
                    help="ONE call: slide the righting against the placed drive and steering")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the check script's size instead of spending a call")
    args = ap.parse_args()

    if args.fit == "e":
        from . import onshape
        script = fite_probe(layout(load(args.params)))
        sm.lint_fs(script)
        reply = onshape.eval_featurescript(script, onshape.resolve(None, "check"))
        for n in onshape.notice_lines(reply):
            print(n)
        con = reply.get("console", "")
        Path("traces/bike_cad").mkdir(parents=True, exist_ok=True)
        Path("traces/bike_cad/fit_electronics.txt").write_text(con)
        print(con)
        print(onshape.budget_line())
        return
    if args.fit:
        from . import onshape
        script = fit2_probe()
        sm.lint_fs(script)
        reply = onshape.eval_featurescript(script, onshape.resolve(None, "check"))
        for n in onshape.notice_lines(reply):
            print(n)
        con = reply.get("console", "")
        Path("traces/bike_cad").mkdir(parents=True, exist_ok=True)
        Path("traces/bike_cad/fit2_righting.txt").write_text(con)
        print(con)
        print(onshape.budget_line())
        return

    data = load(args.params)
    L = layout(data)
    text = build_fs(L, args.fs_version)
    sm.lint_fs(text)
    Path(args.output).write_text(text)
    print(f"wrote {len(text)} chars -> {args.output}")
    print(report(L))
    if args.dry_run:
        print(f"check script: {len(check_wrapper(text))} chars")
        return
    if args.check is not None:
        if not check(text, args.check or None):
            raise SystemExit("check FAILED -- not pushing")
    if args.push is not None:
        from . import onshape
        if args.push != TAB:
            raise SystemExit(f"refusing to push at {args.push or 'the default'!r}: "
                             f"this generator owns `{TAB}` only")
        url = onshape.resolve(args.push, args.push)
        reply = onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}  (microversion "
              f"{reply.get('sourceMicroversion', '?')})")
        print(onshape.budget_line())
    if args.shot is not None:
        from . import onshape
        url = onshape.resolve(args.shot or None, PART_STUDIO)
        tag = "" if args.view == "isometric" else "_" + args.view
        out, _ = onshape.shaded_view(url, Path(OUT_PNG.format(tag)), view=args.view)
        print(f"rendered -> {out}")
        print(onshape.budget_line())


if __name__ == "__main__":
    main()
