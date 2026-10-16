"""The battery / electronics / switch holder for aow-bike-whole-v2: ONE
printed panel on the left, between the battery and the left rail (user,
2026-10-16 night: "instead of a case, it's just a panel (printed -X) with an
overhang panel that goes around the XT30 connector and allows the power
switch to be panel installed"). Layout only: cad_bike_v2 draws it, and its
--check is the check. Spec: docs/plans/aow-bike-whole-v2-complete.md.
Numbers: config/bike_v2_cad.yaml `power:`.

    the panel     |x| 11.3..16 (the battery's side + 0.3 to the rail's inner
                  face), its +X face on the bed, everything else standing
                  off its -X face; it sits on the case sides' top edge
    the joints    two 6-32 into the LEFT rail, the flat heads in the rail's
                  outer face: AFT of the board on the rear arm, FORE on the
                  case side's front ear. The nut in the panel, the panel's
                  ridge in the rail's groove, open up: it drops in from above
    the board     on two standoffs outboard of the rail, square to the drive
                  (XT30 up and back, front plugs up and forward)
    the shelf     off the panel's top over the board's top edge: round the
                  XT30, and the switch panel-mounted in it, aft of the board
    the slits     two strap slits, upper and lower (the straps: user's call)
    the notch     over the left case side's wire gap: the servo B wires
                  come up there and run forward under the battery

All drawn in the DRIVE frame; the part comes out as polygons in the BIKE
frame's (y, z), extruded over an x range (turned rectangles); the board and
switch boxes stay drive-frame boxes. Mm.
"""

from __future__ import annotations

import math

K = math.sqrt(0.5)      # the drive is turned 45 deg about X


def bike(yd: float, zd: float) -> tuple[float, float]:
    """Drive frame (y, z) -> bike frame (y, z)."""
    return ((yd - zd) * K, (yd + zd) * K)


def drive(y: float, z: float) -> tuple[float, float]:
    """Bike frame (y, z) -> drive frame (y, z)."""
    return ((y + z) * K, (z - y) * K)


def dpoly(pts) -> list[list[float]]:
    """A drive-frame (y, z) polygon as a bike-frame one."""
    return [list(bike(*q)) for q in pts]


def drect(y0: float, y1: float, z0: float, z1: float) -> list[list[float]]:
    """A drive-frame (y, z) rectangle as a bike-frame polygon, CCW."""
    return dpoly(((y0, z0), (y1, z0), (y1, z1), (y0, z1)))


def _box(name: str, lo, hi, color) -> dict:
    return {"name": name, "lo": list(lo), "hi": list(hi), "color": list(color)}


def _inside(p, poly) -> bool:
    ok = False
    for i in range(len(poly)):
        (y0, z0), (y1, z1) = poly[i], poly[(i + 1) % len(poly)]
        if (z0 > p[1]) != (z1 > p[1]) and p[0] < y0 + (p[1] - z0) * (y1 - y0) / (z1 - z0):
            ok = not ok
    return ok


def layout(p: dict, D: dict, R: dict, dd: dict, battery: dict) -> dict:
    """p: bike_v2_cad.yaml `power:`; D, R: cad_drive full_layout and
    rail_layout; dd: cad_drive.load(); battery: components.battery. Raises
    if something does not fit."""

    def need(ok: bool, what: str) -> None:
        if not ok:
            raise ValueError(what)

    sc, jn, tn = dd["screw"], dd["s"]["joint"], dd["s"]["tension"]
    bd, sw = p["board"], p["switch"]
    need(abs(dd["tilt_deg"] - 45.0) < 1e-9, "the holder is drawn for the drive turned 45 deg")
    P: dict = {}
    Zp = D["Zp"]
    xo, xi = D["xPo"], R["innerX"]                     # the rail's outer face, its inner face (the joint plane)
    wi = battery["size"][0] / 2 + p["clearance"]      # the panel's battery-side (+X, bed) face, |x|
    need(battery["frame"] == "drive" and abs(battery["at"][0]) < 1e-9, "the battery is not on the centreline")
    need(xi - wi >= 4.0, "the panel between the battery and the rail is too thin")
    P.update({"Zp": Zp, "wi": wi, "xi": xi})
    # ---- the joints: head in the rail's outer face, nut in the panel
    nd = R["plate"] + p["nut_wall"]                   # head to the nut's near face
    nT, nW = sc["nutSlotThickness"], sc["nutSlotWidth"]
    xn0, xn1 = xo - nd, xo - nd - nT                  # the nut slot, |x|
    need(xn1 - wi >= p["lug_wall"] - 1e-9, f"the panel's wall behind the nuts is {xn1 - wi:.2f}")
    tip = xo - p["screw_length"]
    need(tip <= xn0 - tn["nut_thickness"] + 0.05, "the holder's screws do not reach through their nuts")
    need(tip >= wi, "the holder's screws reach the battery's side")
    rl_ = nW / math.sqrt(3) + p["lug_wall"]            # nut corner + a wall
    heads = []
    for tag, (hy, hz) in zip(("carrier", "carrier_front"), p["joints"]):
        rj = next((j for j in R["joints"] if j["tag"] == tag), None)
        need(rj is not None, f"drive_cad.yaml has no rail.{tag}_joint")
        miss = math.dist(bike(*rj["head"]), (hy, hz))
        if miss > 0.01:
            raise ValueError(f"the holder's {tag} joint is {miss:.2f} off the rail's: drive_cad.yaml rail.{tag}_joint "
                             f"[{hy / 1000:g}, {hz / 1000:g}]")
        need(p["ridge"][0] <= rj["half"] + 1e-9, f"the holder's {tag} ridge runs below the rail groove's closed end")
        heads.append(rj["head"])                       # drive (y, z)
    (ay, az), (fy, fz) = heads
    # ---- the board, square to the drive: rear-bottom corner (y0, z0),
    # its plane the bike's side, solder face in
    bw, bh, bt = bd["size"]
    y0, z0 = bd["corner"]
    sdia = bd["standoff_dia"]
    xs = bd["solder_x"]                                # the board's solder (back) face, |x|
    need(xs > xo, "the board's back is inside the rail's outer face")
    xc = xs + bt                                       # its component face
    pt = bd["pitch"]
    holes = [[y0 + i * pt, z0 + j * pt] for i, j in bd["holes"]]
    pc = (D["Ys"], -D["zs"])                           # the left drive pulley (it is in the board's x)
    flange = D["rFl45"]
    low = math.dist(pc, (min(max(pc[0], y0), y0 + bw), min(max(pc[1], z0), z0 + bh)))
    need(low - flange >= 2.0, f"the board is {low - flange:.2f} off the left drive pulley's flange")
    cab = z0 - Zp
    need(cab >= bd["cable_gap"] - 1e-9, f"the board's bottom is {cab:.2f} off the case side's top edge")
    xt_l, xt_t, xt_up, _ = bd["xt30"]
    xty = y0 + bd["xt30_pin"] * pt
    top = z0 + bh + xt_up                              # the XT30's top
    rows = [[z0 + (a - 0.5) * pt, z0 + (b + 0.5) * pt] for a, b in bd["front_rows"]]
    hl, hh = bd["header"]
    kf, ku = bd["plug_keepout"]
    green, black, grey = (0.15, 0.4, 0.15), (0.12, 0.12, 0.12), (0.85, 0.85, 0.9)
    board = [_box("power board", [-xc, y0, z0], [-xs, y0 + bw, z0 + bh], green),
             _box("power board components", [-(xc + bd["components"]), y0, z0], [-xc, y0 + bw, z0 + bh], (0.3, 0.3, 0.3)),
             _box("XT30", [-(xs + xt_t), xty - xt_l / 2, z0 + bh], [-xs, xty + xt_l / 2, top], (0.9, 0.75, 0.1)),
             _box("XT30 plug (keep-out)", [-(xs + xt_t), xty - xt_l / 2, top], [-xs, xty + xt_l / 2, top + ku], grey)]
    for (za, zb_), nm in zip(rows, ("5 V plug (keep-out)", "JST plug (keep-out)")):
        board.append(_box(nm, [-(xc + hh), y0 + bw, za], [-xc, y0 + bw + kf, zb_], grey))
    need(xty + xt_l / 2 <= y0 + bw and xty - xt_l / 2 >= y0, "the XT30 runs off the board's top edge")
    need(all(b <= z0 + bh for _, b in rows), "a front header runs off the board")
    P["board"] = {"boxes": board, "holes": {"at": holes, "r": bd["hole_dia"] / 2, "x": [-xc - 1.0, -xs + 1.0]},
                  "xs": xs, "xc": xc, "y": [y0, y0 + bw], "z": [z0, z0 + bh], "flangeGap": low - flange,
                  "xt30Top": top, "cableGap": cab}
    # ---- the switch, aft of the board (-y), panel-mounted in the shelf,
    # its bezel's top flush with the XT30's
    co, ca = sw["cutout"]
    bzx, bzy, bzh = sw["bezel"]
    pz1 = top - bzh                                    # the shelf's top
    pz0 = pz1 - sw["panel"]
    cx0 = xs + xt_t / 2 - co / 2                       # the cutout, |x|: centred on the XT30
    cx1 = cx0 + co
    need(cx0 >= xi + 1.0, "the switch's cutout reaches the panel")
    cy1 = y0 - sw["gap"]
    cy0 = cy1 - ca
    m = sw["margin"]
    mx, my = (cx0 + cx1) / 2, (cy0 + cy1) / 2
    P["switch"] = {"boxes": [
        _box("switch", [-cx1, cy0, pz0 - sw["depth"]], [-cx0, cy1, pz0], black),
        _box("switch (through the shelf)", [-cx1, cy0, pz0], [-cx0, cy1, pz1], black),
        _box("switch bezel", [-mx - bzx / 2, my - bzy / 2, pz1], [-mx + bzx / 2, my + bzy / 2, pz1 + bzh], (0.8, 0.1, 0.1))],
        "panel": [pz0, pz1], "cut": [cx0, cx1, cy0, cy1]}
    # ---- the panel: from the aft nut's lug to the fore one's, on the case
    # sides' top edge, dropping in front of the case to the fore joint; up
    # to the shelf over the switch and the board, front_top ahead of it
    ya0 = min(ay - rl_, cy0 - sw["margin"])            # the aft lug, or the shelf's aft end
    yf1 = fy + p["ridge"][1] * K + 3.5 * K + 1.0       # past the fore ridge's top corner
    yc = D["Ytab0"] + 0.5                              # in front of the case side (its skirt's front wall)
    zf0 = fz - rl_
    ytop = max(y0 + bw, p["slits"][0] + p["slits"][1] + 1.0)   # the high part: the board, the slits
    outline = [(ya0, Zp), (yc, Zp), (yc, zf0), (yf1, zf0), (yf1, p["front_top"]), (ytop, p["front_top"]),
               (ytop, pz1), (ya0, pz1)]
    need(fz + rl_ <= p["front_top"] and zf0 < Zp, "the fore joint is not on the panel")
    need(az - rl_ >= Zp - 1e-9 or az - nW / 2 - p["lug_wall"] >= Zp, "the aft joint's nut runs under the case sides' top edge")
    adds = [{"tag": "panel", "x": [-xi, -wi], "pts": dpoly(outline)}]
    xt_c = bd["xt30_clearance"]
    shelf_y1 = xty + xt_l / 2 + xt_c + m
    need(shelf_y1 <= y0 + bw + 1e-9, "the shelf runs past the board")
    adds.append({"tag": "shelf", "x": [-(cx1 + m), -(wi + 1.0)], "pts": drect(cy0 - m, shelf_y1, pz0, pz1)})
    cuts = [{"tag": "cutout", "x": [-cx1, -cx0], "pts": drect(cy0, cy1, pz0 - 1, pz1 + 1)},
            {"tag": "xt30", "x": [-(xs + xt_t + xt_c), -(xs - xt_c)],
             "pts": drect(xty - xt_l / 2 - xt_c, xty + xt_l / 2 + xt_c, pz0 - 1, pz1 + 1)}]
    need(xs - xt_c > xi, "the XT30's hole in the shelf reaches the panel")
    # the wire notch over the left case side's wire gap
    wn0, wn1 = D["wireY0"] - p["wire_notch"][0], D["wireY1"] + p["wire_notch"][0]
    cuts.append({"tag": "wires", "x": [-xi - 1, -wi + 1], "pts": drect(wn0, wn1, Zp - 1, Zp + p["wire_notch"][1])})
    P["wireNotch"] = [wn0, wn1]
    # the strap slits, upper and lower
    s0, sl, swd = p["slits"]
    for k, z in enumerate(p["slit_z"]):
        cuts.append({"tag": f"slit{k}", "x": [-xi - 1, -wi + 1], "pts": drect(s0, s0 + sl, z, z + swd)})
        need(z >= Zp + p["wire_notch"][1] + 1.0 or s0 >= wn1 + 1.0 or s0 + sl <= wn0 - 1.0,
             "a strap slit runs into the wire notch")
        need(z + swd <= pz0 - 1.0, "a strap slit runs into the shelf")
    P["slits"] = [[s0, s0 + sl, z, z + swd] for z in p["slit_z"]]
    # the standoffs and their pilots, off the panel's -X face
    pt3 = lambda x, y, z: [x, *bike(y, z)]             # noqa: E731
    cyls = []
    for k, (y, z) in enumerate(holes):
        cyls.append({"tag": f"so{k}", "op": "add", "p0": pt3(-xi + 0.5, y, z), "p1": pt3(-xs, y, z), "r": sdia / 2})
        cyls.append({"tag": f"pl{k}", "op": "cut", "p0": pt3(-wi + 0.5, y, z), "p1": pt3(-xs - 0.5, y, z),
                     "r": bd["pilot_dia"] / 2})
        for s in P["slits"]:
            need(not (s[0] - sdia / 2 < y < s[1] + sdia / 2 and s[2] - sdia / 2 < z < s[3] + sdia / 2),
                 "a strap slit runs under a board standoff")
    # the joints' cutters and ridges: aft, the nut slot out the panel's aft
    # edge; fore, out its front
    P["joints"] = []
    for side, (hy_, hz_), sl_dir, edge in (("A", heads[0], -1, ya0), ("F", heads[1], 1, yf1)):
        hb = bike(hy_, hz_)
        r0, r1 = hb[1] - p["ridge"][0], hb[1] + p["ridge"][1]
        # the ridge's four root corners inside the panel (drive frame)
        gw = jn["ridge_flat"] / 2 + jn["ridge_height"]
        for zz in (r0, r1):
            for yy in (hb[0] - gw, hb[0] + gw):
                need(_inside(drive(yy, zz), outline), f"the holder's {side} ridge runs off the panel")
        P["joints"].append({"side": side, "head": [-xo, hb[0], hb[1]], "zIn": [1, 0, 0],
                            "slot": [0, sl_dir * K, sl_dir * K], "nutDepth": nd,
                            "slotLen": abs(edge - hy_) + 1.0, "holeDepth": nd + nT,
                            "ridgeAt": [-xi, hb[0], (r0 + r1) / 2], "ridgeHalf": (r1 - r0) / 2,
                            "ridgeUp": [-1, 0, 0]})
    P.update({"nut": [xn0, xn1], "tip": tip, "tipToPack": tip - battery["size"][0] / 2, "outline": outline})
    # ---- the rail screws' heads clear in side view (user: the screw
    # "should enter fully clear from the side view"): nothing outboard of
    # the rail over either, a millimetre round the head; and clear of the
    # left pulley's belt (its outer face) and flange
    hd = sc["headDia"] / 2 + 1.0
    outboard = board + P["switch"]["boxes"] + [
        {"name": "standoff", "lo": [-xs, y - sdia / 2, z - sdia / 2], "hi": [-xi, y + sdia / 2, z + sdia / 2]}
        for y, z in holes] + [{"name": "shelf", "lo": [0, cy0 - m, pz0], "hi": [0, shelf_y1, pz1]}]
    view = math.inf
    for hy_, hz_ in heads:
        for bx_ in outboard:
            dy = max(bx_["lo"][1] - hy_, 0.0, hy_ - bx_["hi"][1])
            dz = max(bx_["lo"][2] - hz_, 0.0, hz_ - bx_["hi"][2])
            view = min(view, math.hypot(dy, dz) - hd)
    need(view >= 0, f"something on the holder covers a rail screw's head in side view ({view:.2f})")
    belt = D["rP45"] + dd["belt"]["thickness"] - dd["s"]["pulley"]["pld"]
    pv = min(math.dist(pc, h) - max(belt, flange) - hd for h in heads)
    need(pv >= 0, f"a rail screw's head is behind the left drive pulley in side view ({pv:.2f})")
    P["headView"], P["pulleyView"] = view, pv
    # ---- keep it out of the tyre
    rmin = D["Yc0"]                                    # the tyre's radius + the drive's clearance
    near = min(math.hypot(*q) for a in adds for q in a["pts"])
    need(near >= rmin, f"the holder comes {near:.1f} from the rear axle (the tyre + clearance: {rmin:.1f})")
    P["tyreRoom"] = near - rmin
    P["adds"], P["cuts"], P["cyls"] = adds, cuts, cyls
    # a point inside the part, to find it by: the panel over the upper slit,
    # in solid material, inside no cutter (else partAt finds the cutter too:
    # BOOLEAN_SAME_INPUT, 2026-10-16)
    q = (s0 + sl / 2, (p["slit_z"][1] + swd + pz0) / 2)
    need(_inside(q, outline), "the holder's finding point is off the panel")
    for c in cuts:
        need(not _inside(bike(*q), c["pts"]), f"the holder's finding point is in the {c['tag']} cut")
    for y, z in holes:
        need(math.dist(q, (y, z)) > bd["pilot_dia"], "the holder's finding point is in a pilot hole")
    P["pt"] = [-(xi + wi) / 2, *bike(*q)]
    return P
