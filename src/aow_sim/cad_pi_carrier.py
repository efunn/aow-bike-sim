"""The Pi + U2D2 carrier for aow-bike-whole-v2, an OUTLINE: self-contained,
NOT yet attached to anything (user, 2026-10-16: "build the self-contained
carrier first before deciding" how it meets the rails). Layout only:
cad_bike_v2 draws it. Spec: docs/plans/aow-bike-whole-v2-complete.md ("Pi +
U2D2 carrier"). Numbers: config/bike_v2_cad.yaml `pi_carrier:`.

DRIVE frame, over the drive's front face (the front-top; the power board
went to the back-top beside the battery): a plate parallel to the face,
the Pi 3B+ on standoffs on its outer side ("Pi on top"), the U2D2 under it
between the drive pulleys, in a short cradle, between the plate and the
AHRS. Every piece is a drive-frame box, returned as bike-frame (y, z)
polygons over an x range, as cad_power's. Mm.
"""

from __future__ import annotations

from .cad_power import bike, drect


def layout(p: dict, D: dict, pi: dict, comps: dict, ahrs_top: float, housing_end: float) -> dict:
    """p: bike_v2_cad.yaml `pi_carrier:`; D: cad_drive full_layout; pi: the
    `pi:` detail; comps: `components:`; ahrs_top: the drive y of the AHRS
    housing's top face. Raises if something does not fit."""

    def need(ok: bool, what: str) -> None:
        if not ok:
            raise ValueError(what)

    bw, bl, bt = pi["board"]
    t = p["plate"]
    y1 = p["plate_y0"] + t                              # the plate's outer face
    under = y1 + p["pi_standoff"][0]                    # the Pi board's underside
    # the Pi's box centre: its board's underside is board_y - bt/2 in the box
    want_pi = [0.0, under - (pi["board_y"] - bt / 2), p["pi_z"]]
    u = comps["u2d2"]
    ux, uy, uz = u["size"]
    want_u = [0.0, p["plate_y0"] - uy / 2, p["u2d2_z"]]
    for k, c, want in (("pi", comps["pi"], want_pi), ("u2d2", u, want_u)):
        if c.get("at") is None or c["frame"] != "drive" or max(abs(a - b) for a, b in zip(c["at"], want)) > 1e-6:
            raise ValueError(f"the {k} is not on the Pi carrier: bike_v2_cad.yaml components.{k} frame drive, "
                             f"at [{want[0]:g}, {want[1]:g}, {want[2]:g}]")
    # the U2D2 between the drive pulleys and clear of the AHRS's housing
    pul = D["flangeIn"]                                 # the drive pulleys' flanges, |x|
    need(ux / 2 + 1.0 <= pul, "the U2D2 is not between the drive pulleys")
    gap = (p["plate_y0"] - uy) - ahrs_top
    need(gap >= 0.5, f"the U2D2 is {gap:.2f} off the AHRS's housing")
    # its JSTs face the drive (down and back in the bike) near its lower end:
    # slid up the face until they are past the housing's upper end (user)
    jst = (p["u2d2_z"] - uz / 2) - housing_end
    need(jst >= 0.5, f"the U2D2's JSTs are {jst:.2f} past the AHRS housing's end: pi_carrier.u2d2_z "
                     f"{p['u2d2_z'] - jst + 0.5:g}")
    # the Pi's board (and so the plate) past the pulleys' rims
    rim = D["Ys"] + D["rFl45"]
    need(p["plate_y0"] >= rim + 1.0, f"the Pi carrier's plate (|x| past the pulleys) is inside their rims ({rim:.1f})")
    z0 = p["pi_z"] + pi["board_z"] - bl / 2             # the Pi board's ends, drive z
    z1 = z0 + bl
    m = p["margin"]
    hx, hy = pi["holes"]
    hz0 = z0 + pi["hole_inset"]
    holes = [(sx * hx / 2, hz0 + k * hy) for k in (0, 1) for sx in (1, -1)]
    so_r = p["pi_standoff"][1] / 2
    zl = min(z0, min(z for _, z in holes) - so_r) - m
    zh = max(z1, max(z for _, z in holes) + so_r) + m
    xh = bw / 2 + m
    adds = [{"tag": "plate", "x": [-xh, xh], "pts": drect(p["plate_y0"], y1, zl, zh)}]
    # the U2D2's cradle: two short cheeks at mid-length, its ends and their
    # connectors left open
    cw, cd, cl = p["cradle"]                             # wall, depth, length
    ci = ux / 2 + p["cradle_clearance"]
    for sg, tg in ((1, "R"), (-1, "L")):
        adds.append({"tag": "cheek" + tg, "x": sorted([sg * ci, sg * (ci + cw)]),
                     "pts": drect(p["plate_y0"] - cd, p["plate_y0"] + 0.5, p["u2d2_z"] - cl / 2, p["u2d2_z"] + cl / 2)})
    cyls = []
    pt3 = lambda x, yd, zd: [x, *bike(yd, zd)]          # noqa: E731
    for k, (x, z) in enumerate(holes):
        cyls.append({"tag": f"so{k}", "op": "add", "p0": pt3(x, y1 - 0.5, z), "p1": pt3(x, under, z), "r": so_r})
        cyls.append({"tag": f"pl{k}", "op": "cut", "p0": pt3(x, p["plate_y0"] - 0.5, z), "p1": pt3(x, under + 0.5, z),
                     "r": p["pi_pilot_dia"] / 2})
        need(abs(x) - so_r >= ci + cw + 0.5 or abs(z - p["u2d2_z"]) - so_r > cl / 2, "a Pi standoff runs into the cradle")
    rimGap = p["plate_y0"] - rim
    return {"adds": adds, "cuts": [], "cyls": cyls, "joints": [],
            "pt": [0.0, *bike(p["plate_y0"] + t / 2, (zl + zh) / 2)],
            "u2d2Gap": gap, "jstPast": jst, "rimGap": rimGap, "plate": [p["plate_y0"], y1, zl, zh, xh], "holes": holes}
