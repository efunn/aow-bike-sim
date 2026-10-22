"""The Pi + U2D2 carrier for aow-bike-whole-v2. Layout only: cad.bike_v2
draws it. Spec: docs/plans/aow-bike-whole-v2-complete.md ("Pi + U2D2
carrier"). Numbers: config/cad/bike_v2_cad.yaml `pi_carrier:`.

DRIVE frame, over the drive's front face (the front-top; the power board
went to the back-top beside the battery):

    the plate     parallel to the face, past the pulleys' rims
    the U2D2      under it, between the drive pulleys and the AHRS, slid down
                  the face until its JSTs are in front of the AHRS; two M2
                  tapping screws through the plate into its Pi-side face, 42
                  apart (user), in a short cradle
    the Pi 3B+    over it on four SEPARATE printed spacers (user: so the
                  carrier prints more easily round its three joints), 4-40
                  machine screws through the board and the spacers,
                  self-tapping into the plate and a boss behind it (user)
    the joints    THREE 6-32, the power holder's house joint turned round:
                  the flat head in the rail's outer face, the nut in a lug of
                  the carrier's inboard of the rail, the lug's ridge in the
                  rail's groove. ONE at the top on the LEFT, on the front
                  ear's run forward (user: "another extra ear here from the
                  drive case which attaches to the LHS Pi case"); TWO low, one
                  a side, in the rails' lower arms above the righting's
                  bridge (user: "at the bottom, near the righting mechanism").
                  It lifts out STRAIGHT UP in the bike (the drive's +Y+Z):
                  the grooves are open upward

Assembly (user): the U2D2 onto the plate, then the Pi on its spacers, then
the carrier into the bike. Every piece is a drive-frame box or polygon,
returned as bike-frame (y, z) polygons over an x range, as cad.power's. Mm.
"""

from __future__ import annotations

import math

from .power import K, _hull_gap, _inside, _poly_off_rect, bike, dpoly, drect

LIFT = (K, K)           # the way it comes out, drive (y, z): straight up in the bike


def layout(p: dict, D: dict, R: dict, dd: dict, pi: dict, comps: dict, ahrs_top: float, ahrs_low: float,
           jst_len: float, holder: dict, bridge_top: float) -> dict:
    """p: bike_v2_cad.yaml `pi_carrier:`; D, R: cad.drive full_layout and
    rail_layout; dd: cad.drive.load(); pi: the `pi:` detail; comps:
    `components:`; ahrs_top: the drive y of the AHRS housing's top face;
    ahrs_low: the drive z of the AHRS housing's lower end; jst_len: the
    U2D2's JST keep-out, along it from its lower end; holder: cad.power's
    layout; bridge_top: the bike z of the righting bridge's top face.
    Raises if something does not fit."""

    def need(ok: bool, what: str) -> None:
        if not ok:
            raise ValueError(what)

    sc, jn, tn = dd["screw"], dd["s"]["joint"], dd["s"]["tension"]
    bw, bl, bt = pi["board"]
    t = p["plate"]
    y0p = p["plate_y0"]
    y1 = y0p + t                                        # the plate's outer face
    sp_h, sp_od, sp_bore = p["pi_spacer"]
    under = y1 + sp_h                                   # the Pi board's underside
    # the Pi's box centre: its board's underside is board_y - bt/2 in the box
    want_pi = [0.0, under - (pi["board_y"] - bt / 2), p["pi_z"]]
    u = comps["u2d2"]
    ux, uy, uz = u["size"]
    want_u = [0.0, y0p - uy / 2, p["u2d2_z"]]
    for k, c, want in (("pi", comps["pi"], want_pi), ("u2d2", u, want_u)):
        if c.get("at") is None or c["frame"] != "drive" or max(abs(a - b) for a, b in zip(c["at"], want)) > 1e-6:
            raise ValueError(f"the {k} is not on the Pi carrier: bike_v2_cad.yaml components.{k} frame drive, "
                             f"at [{want[0]:g}, {want[1]:g}, {want[2]:g}]")
    # the U2D2 between the drive pulleys and clear of the AHRS's housing
    need(ux / 2 + 1.0 <= D["flangeIn"], "the U2D2 is not between the drive pulleys")
    gap = (y0p - uy) - ahrs_top
    need(gap >= 0.5, f"the U2D2 is {gap:.2f} off the AHRS's housing")
    # its JSTs face the drive (down and back in the bike) near its lower end:
    # their keep-out wholly past the lower end of the AHRS's HOUSING, in
    # front of it (user, 2026-10-16 night: "only the bump actually needs to
    # be cleared"; the board under it ends lower but is behind the keep-out)
    jst = ahrs_low - (p["u2d2_z"] - uz / 2 + jst_len)
    need(jst >= 0.5, f"the U2D2's JSTs' keep-out is {jst:.2f} past the AHRS housing's lower end: pi_carrier.u2d2_z "
                     f"{p['u2d2_z'] + jst - 0.5:g}")
    # the plate past the pulleys' rims
    rim = D["Ys"] + D["rFl45"]
    need(y0p >= rim + 1.0, f"the Pi carrier's plate (|x| past the pulleys) is inside their rims ({rim:.1f})")
    # ---- the U2D2's two M2 (user: "2x m2tap screws located centrally and
    # 42mm apart", on its face toward the Pi): through the plate from the
    # Pi's side, their heads under the Pi
    us, ud, uh = p["u2d2_screws"]                       # span, clearance hole, head height
    uholes = [p["u2d2_z"] - us / 2, p["u2d2_z"] + us / 2]
    need(us / 2 + ud / 2 <= uz / 2, "the U2D2's screws run off its ends")
    need(sp_h >= uh + 1.0, f"the Pi's spacers ({sp_h:g}) leave under 1 over the U2D2's screw heads")
    # ---- the plate: the Pi's PCB's footprint and no more (user: "no
    # necessary reason to extend beyond the footprint made by the pi's
    # PCB"), its holes' bosses and the U2D2's screws inside it
    z0 = p["pi_z"] + pi["board_z"] - bl / 2             # the Pi board's ends, drive z
    z1 = z0 + bl
    hx, hy = pi["holes"]
    hz0 = z0 + pi["hole_inset"]
    holes = [(sx * hx / 2, hz0 + k * hy) for k in (0, 1) for sx in (1, -1)]
    bdep, bdia = p["pi_boss"]
    br = bdia / 2
    zl, zh, xh = z0, z1, bw / 2
    need(max(abs(x) for x, _ in holes) + br <= xh and min(z for _, z in holes) - br >= zl
         and max(z for _, z in holes) + br <= zh, "a Pi boss runs off the Pi's footprint")
    need(uholes[0] - ud / 2 >= zl + 1.0 and uholes[1] + ud / 2 <= zh - 1.0, "a U2D2 screw runs off the plate")
    # the plate and the Pi (on its spacers) over the rails' top edges in
    # side view (the plate spans them across)
    railGap = min(min(_edge_dist_rect(R["pieces"][0], q) for q in
                      ((y0p, y1, zl, zh), (y1, under + bt, z0, z1))), 99.0)
    need(railGap >= p["rail_gap"] - 1e-6, f"the Pi carrier's plate or the Pi comes {railGap:.2f} from the rail's top edge")
    adds = [{"tag": "plate", "x": [-xh, xh], "pts": drect(y0p, y1, zl, zh)}]
    # the U2D2's cradle: two short cheeks at mid-length, its ends and their
    # connectors left open
    cw, cd, cl = p["cradle"]                             # wall, depth, length
    ci = ux / 2 + p["cradle_clearance"]
    for sg, tg in ((1, "R"), (-1, "L")):
        adds.append({"tag": "cheek" + tg, "x": sorted([sg * ci, sg * (ci + cw)]),
                     "pts": drect(y0p - cd, y0p + 0.5, p["u2d2_z"] - cl / 2, p["u2d2_z"] + cl / 2)})
    need(all(abs(z - p["u2d2_z"]) - ud / 2 >= cl / 2 + 0.5 for z in uholes), "a U2D2 screw runs into its cradle")
    cyls = []
    pt3 = lambda x, yd, zd: [x, *bike(yd, zd)]          # noqa: E731
    # the Pi's holes: a boss behind the plate at each (the 4-40 self-taps
    # through the plate and the boss), clear of the drive pulleys and the
    # belt over them
    back = dd["belt"]["thickness"] - dd["s"]["pulley"]["pld"]
    r15 = max(D["rP15"] + back, D["rFl15"])
    r45 = max(D["rP45"] + back, D["rFl45"])
    bossGap = math.inf
    for k, (x, z) in enumerate(holes):
        cyls.append({"tag": f"bs{k}", "op": "add", "p0": pt3(x, y0p - bdep, z), "p1": pt3(x, y0p + 0.5, z), "r": br})
        cyls.append({"tag": f"pl{k}", "op": "cut", "p0": pt3(x, y0p - bdep - 0.5, z), "p1": pt3(x, y1 + 0.5, z),
                     "r": p["pi_pilot_dia"] / 2})
        need(abs(x) - br >= ci + cw + 0.5 or abs(z - p["u2d2_z"]) - br > cl / 2, "a Pi boss runs into the cradle")
        if abs(x) + br > D["flangeIn"]:
            pc = (D["Ys"], -D["zs"] if x < 0 else D["zs"])      # the left pulley is at -zs
            ny, nz = min(max(pc[0], y0p - bdep), y0p), min(max(pc[1], z - br), z + br)
            bossGap = min(bossGap, math.dist(pc, (ny, nz)) - r45)
    need(bossGap >= 1.0, f"a Pi boss comes {bossGap:.2f} from a drive pulley's belt")
    for k, z in enumerate(uholes):
        cyls.append({"tag": f"uh{k}", "op": "cut", "p0": pt3(0.0, y0p - 0.5, z), "p1": pt3(0.0, y1 + 0.5, z),
                     "r": ud / 2})
    # ---- the three joints
    xo, xi = D["xPo"], R["innerX"]                      # the rail's outer face, its inner face (the joint plane)
    nd = R["plate"] + p["nut_wall"]                     # head to the nut's near face
    nT, nW = sc["nutSlotThickness"], sc["nutSlotWidth"]
    xn0, xn1 = xo - nd, xo - nd - nT                    # the nut slot, |x|
    xl = xn1 - p["lug_wall"]                            # the lugs' inner face, |x|
    # the holes go THROUGH the lugs (user, 2026-10-16 night: "the 3x screw
    # holes for the pi carrier can be thru holes"): any screw that passes
    # its nut, the shortest head to the nut's far face
    need(xl - ux / 2 >= 1.0, "the Pi carrier's lugs run into the U2D2")
    minLen = xo - xn0 + tn["nut_thickness"]
    rl_ = nW / math.sqrt(3) + p["lug_wall"]            # nut corner + a wall
    gw = jn["ridge_flat"] / 2 + jn["ridge_height"]
    r0, r1 = p["ridge"]                                 # down (closed), up (open) of each head, bike z
    hd = sc["headDia"] / 2 + 1.0
    hold_front = max(q[0] for q in holder["outline"])   # the power holder's front, drive y
    outboard = holder["board"]["boxes"] + holder["switch"]["boxes"]
    lugs, joints, view, pview, lowest, holder_gap = [], [], math.inf, math.inf, math.inf, math.nan
    for tag, (hyb, hzb), sides in (("pi_top", p["joints"][0], (-1,)), ("pi_low", p["joints"][1], (-1, 1))):
        src = R["leftJoints"] if sides == (-1,) else R["joints"]
        rj = next((j for j in src if j["tag"] == tag), None)
        need(rj is not None, f"drive_cad.yaml has no rail.{tag}_joint ({'left only' if sides == (-1,) else 'both'})")
        miss = math.dist(bike(*rj["head"]), (hyb, hzb))
        if miss > 0.01:
            raise ValueError(f"the Pi carrier's {tag} joint is {miss:.2f} off the rail's: drive_cad.yaml rail.{tag}_joint "
                             f"[{hyb / 1000:g}, {hzb / 1000:g}]")
        need(max(abs(rj["dir"][0] + K), abs(rj["dir"][1] + K)) < 1e-9,
             f"the rail's {tag} groove is not closed toward the bike's down (the carrier lifts out upward: ridge_deg 270)")
        need(r0 <= rj["half"] + 1e-9, f"the Pi carrier's {tag} ridge runs below the rail groove's closed end")
        hy, hz = rj["head"]
        # the ridge's corners, drive (y, z): along the lift, gw either side
        rc = [(hy + s * LIFT[0] + v * -K, hz + s * LIFT[1] + v * K) for s in (-r0, r1) for v in (-gw, gw)]
        # the lug round the nut and the ridge, half a mm past the ridge
        ylo = min([hy - rl_] + [q[0] - 0.5 for q in rc])
        zlo = min([hz - rl_] + [q[1] - 0.5 for q in rc])
        if tag == "pi_top":
            # behind the plate, up from the head over the nut and the ridge;
            # the nut slot out its top (the drive's +Z: up in the bike)
            yhi, zhi = y0p + 0.5, max([hz + rl_] + [q[1] + 0.5 for q in rc])
            holder_gap = ylo - hold_front
            need(ylo >= hold_front + p["holder_gap"] - 1e-9,
                 f"the Pi carrier's top lug is {ylo - hold_front:.2f} off the power holder's front")
            need(zhi <= zh, "the Pi carrier's top lug runs over the plate's top")
            slot, slen = [0.0, -K, K], zhi - hz + 1.0
        else:
            # under the plate, down to the head; the nut slot out its front
            # (the drive's +Y: up and forward in the bike)
            # flush with the plate's outer face (user: 0.51 proud before)
            yhi, zhi = y1, zl + 1.0
            need(yhi >= hy + rl_ and max(q[0] + 0.5 for q in rc) <= yhi,
                 "the Pi carrier's low lug's nut or ridge runs past the plate's outer face")
            need(zhi >= hz + rl_, "the Pi carrier's low lug does not reach the plate")
            slot, slen = [0.0, K, K], yhi - hy + 1.0
        poly = [(ylo, zlo), (yhi, zlo), (yhi, zhi), (ylo, zhi)]
        need(all(_inside(q, poly) for q in rc), f"the Pi carrier's {tag} ridge runs off its lug")
        need(ylo >= ahrs_top + 0.5, f"the Pi carrier's {tag} lug runs into the AHRS")
        # above the righting's bridge (it is as wide as between the rails)
        lowest = min(lowest, min(bike(*q)[1] for q in poly))
        mid = bike(hy + (r1 - r0) / 2 * LIFT[0], hz + (r1 - r0) / 2 * LIFT[1])
        hb = bike(hy, hz)
        for sg in sides:
            sd = "T" if tag == "pi_top" else ("LL" if sg < 0 else "LR")
            lugs.append({"tag": "lug" + sd, "x": sorted([sg * xi, sg * xl]), "pts": dpoly(poly)})
            joints.append({"side": sd, "head": [sg * xo, hb[0], hb[1]], "zIn": [-sg, 0, 0], "slot": slot,
                           "nutDepth": nd, "slotLen": slen, "holeDepth": xo - xl + 1.0,
                           "ridgeAt": [sg * xi, mid[0], mid[1]], "ridgeHalf": (r0 + r1) / 2,
                           "ridgeUp": [sg, 0, 0], "ridgeRot": 0.0})
            # the head clear in side view: of the holder's board and switch
            # (the left side), of the belt over its pulleys
            if sg < 0:
                for b in outboard:
                    dy = max(b["lo"][1] - hy, 0.0, hy - b["hi"][1])
                    dz = max(b["lo"][2] - hz, 0.0, hz - b["hi"][2])
                    view = min(view, math.hypot(dy, dz) - hd)
            pview = min(pview, _hull_gap((hy, hz), (0.0, 0.0), r15, (D["Ys"], sg * D["zs"]), r45) - hd)
    need(view >= 0, f"the power holder covers a Pi carrier screw's head in side view ({view:.2f})")
    need(pview >= 0, f"a Pi carrier screw's head is behind a belt or a pulley in side view ({pview:.2f})")
    need(lowest >= bridge_top + 1.0, f"the Pi carrier's low lugs come {lowest - bridge_top:.2f} over the righting's bridge")
    adds += lugs
    # ---- the Pi's spacers, SEPARATE prints (user: "the pi standoffs will
    # need to be separate spacers (I can print them, clearance for 4-40)"):
    # each from the plate's outer face to the board's underside
    spacers = []
    for k, (x, z) in enumerate(holes):
        rm = (sp_od + sp_bore) / 4                      # mid-wall, to find it by
        spacers.append({"adds": [], "cuts": [], "joints": [],
                        "cyls": [{"tag": "sp", "op": "add", "p0": pt3(x, y1, z), "p1": pt3(x, under, z), "r": sp_od / 2},
                                 {"tag": "spb", "op": "cut", "p0": pt3(x, y1 - 0.5, z), "p1": pt3(x, under + 0.5, z),
                                  "r": sp_bore / 2}],
                        "pt": pt3(x + rm, (y1 + under) / 2, z)})
    need(sp_od >= sp_bore + 2 * dd["s"]["print"]["min_wall"], "the Pi's spacers are thinner than two walls")
    rimGap = y0p - rim
    # a point inside the plate, in no cutter (not on the U2D2's screws at x 0)
    ptx = (ux / 2 + xl) / 2
    return {"adds": adds, "cuts": [], "cyls": cyls, "joints": joints,
            "pt": [ptx, *bike(y0p + t / 2, (zl + zh) / 2)], "spacers": spacers, "holderGap": holder_gap,
            "u2d2Gap": gap, "jstPast": jst, "railGap": railGap, "rimGap": rimGap, "plate": [y0p, y1, zl, zh, xh], "holes": holes,
            "u2d2Holes": uholes, "headView": view, "pulleyView": pview, "bridgeGap": lowest - bridge_top,
            "bossGap": bossGap, "nut": [xn0, xn1], "lugIn": xl, "minScrew": minLen,
            "piStack": {"board": bt, "spacer": sp_h, "plate": t, "boss": bdep}}


def _edge_dist_rect(poly, rect) -> float:
    """How far the rectangle (y0, y1, z0, z1) is from the polygon (y, z),
    0 if they meet: the least distance between their outlines, sampled."""
    y0, y1, z0, z1 = rect
    box = [(y0, z0), (y1, z0), (y1, z1), (y0, z1)]
    if any(_inside(q, poly) for q in box) or not _poly_off_rect(poly, rect, 0.0):
        return 0.0
    best = math.inf
    for a, b in zip(poly, poly[1:] + poly[:1]):
        n = max(2, int(math.dist(a, b) / 0.1))
        for i in range(n + 1):
            q = (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
            dy = max(y0 - q[0], 0.0, q[0] - y1)
            dz = max(z0 - q[1], 0.0, q[1] - z1)
            best = min(best, math.hypot(dy, dz))
    return best
