"""The printed righting module's CAD, checked without Onshape.

`--check` builds the FeatureScript in Onshape and measures every part and pose;
these check `layout()` itself -- the axial stack and its thrust bosses, the
washer, the diamond-only guard, the L/R twins, the pose transforms, the
clearances the layout was chosen for -- and that the generated FeatureScript
lints.
Invalidated by: config/righting_cad.yaml, the linkage config it names,
steering_cad.yaml's hub and coupling blocks, the X330 envelope and mount
tables, or the shared FeatureScript in cad_ahrs_fixture / cad_servo_mount.
"""

import itertools
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree
import yaml

from aow_sim import cad_righting as cr
from aow_sim import cad_servo_mount as sm

pytestmark = pytest.mark.cad


@pytest.fixture(scope="module")
def data():
    return cr.load()


@pytest.fixture(scope="module")
def L(data):
    return cr.layout(data)


def variant(tmp_path, **edits) -> dict:
    """load() on a copy of the config with `section__key=value` edits."""
    raw = yaml.safe_load(Path(cr.PARAMS).read_text())
    for k, v in edits.items():
        sec, key = k.split("__")
        raw[sec][key]["value"] = v
    p = tmp_path / "righting_cad.yaml"
    p.write_text(yaml.safe_dump(raw))
    return cr.load(str(p))


def part(L, slug):
    return next(p for p in L["parts"] if p["slug"] == slug)


def test_the_pose_transforms_land_the_pins_on_the_solver(L):
    """Every dialog and check pose: each group's rotation + shift carries the
    rest crankpin and rocker pins exactly where SwingLinkage puts them."""
    cr.self_check(L)
    for f in cr.CHECK_POSES:
        assert f"{f:+.2f}" in L["poses"]


def test_the_featurescript_builds_and_lints(data, L):
    text = cr.build_fs(L, data)
    sm.lint_fs(text)
    assert "export const aowRighting = defineFeature" in text
    assert text.count(cr.SPLIT_MARK) == 1
    for p in L["parts"]:
        assert f'"{p["name"]}"' in text


def test_the_stack_opens_every_bossed_gap_to_boss_plus_clearance(data, L):
    """Along Y from the mid-plane: couplers (and knuckles), web, bearing,
    each running pair boss + clearance apart. The mid-plane gap holds the
    crankpin's washer, and is boss + clearance too: the knuckles' rings at
    the rocker pins and knuckle R's at the rod cross it."""
    st, ws, Y = data["s"]["stack"], data["s"]["washer"], L["Y"]
    gb = st["boss"] + st["clearance"]
    assert st["thrust"] == "loop"
    assert Y["cpl_in"] == pytest.approx(ws["thickness"] / 2 + ws["clearance"])
    assert 2 * Y["cpl_in"] == pytest.approx(gb)
    assert Y["web_in"] - Y["cpl_out"] == pytest.approx(gb)
    assert Y["bh_in"] - Y["hz_out"] == pytest.approx(gb)
    order = ["cpl_in", "cpl_out", "web_in", "web_out", "hz_out", "bh_in", "bh_out"]
    assert all(Y[a] <= Y[b] for a, b in zip(order, order[1:]))      # hz_out = web_out when outboard
    assert all(Y[a] < Y[b] for a, b in zip(order, order[1:]) if (a, b) != ("web_out", "hz_out"))
    # the knuckles in the coupler layers: nothing outside the bulkheads but the hub
    assert (Y["kn_in"], Y["kn_out"]) == (Y["cpl_in"], Y["cpl_out"])
    assert Y["rod_back"] == Y["rod_front"] == Y["bh_out"]


def test_outboard_joint_makes_every_moving_link_one_plate(data, L):
    """wing.joint outboard (user, 2026-10-05): crank web, rocker and knuckles
    are each one stack layer thick along Y, as the couplers are; the wing's
    rocker-side tab sits past the rocker, not beside it."""
    assert data["s"]["wing"]["joint"] == "outboard"
    st, Y = data["s"]["stack"], L["Y"]
    gb = st["boss"] + st["clearance"]
    for slug, t in (("rockerR", st["web"]), ("crankR", st["web"]), ("couplerR", st["coupler"]),
                    ("knuckleL", st["knuckle"]), ("knuckleR", st["knuckle"])):
        p = part(L, slug)
        # the plate itself, without a thrust ring (gb thin) or the crank's journal and lugs
        ys = [(q["y0"], q["y1"]) for q in p["add"]
              if q["k"] == "slot" or (q["k"] == "prism" and slug != "crankR")]
        assert {round(b - a, 6) for a, b in ys} == {t}, slug
    assert st["knuckle"] == pytest.approx(6.0)
    assert Y["hz_out"] == Y["web_out"]
    want = (-Y["web_out"] - data["s"]["wing"]["tab"], -Y["web_out"])
    assert any((q["y0"], q["y1"]) == pytest.approx(want) for q in part(L, "wingR")["add"] if q["k"] == "prism")
    assert Y["bh_in"] - Y["web_out"] == pytest.approx(gb)


def test_the_washer_is_four_numbers(tmp_path, data, L):
    """An off-the-shelf washer is a config edit: its thickness moves the whole
    stack by the same amount; 0 removes it; a bore under the rod is refused."""
    nylon = cr.layout(variant(tmp_path, washer__thickness=0.8, washer__od=12.0, washer__id=6.4))
    shift = (0.8 / 2 + data["s"]["washer"]["clearance"]) - L["Y"]["cpl_in"]
    for k in ("cpl_in", "web_in", "bh_out", "kn_out"):
        assert nylon["Y"][k] == pytest.approx(L["Y"][k] + shift)
    w = part(nylon, "washer")
    assert w["add"][0]["r"] == pytest.approx(6.0) and w["cut"][0]["r"] == pytest.approx(3.2)
    # the rod's is drawn only in the first thrust scheme: knuckle R's ring closes that gap
    assert not any(p["slug"] == "rodwasher" for p in L["parts"])
    none = cr.layout(variant(tmp_path, washer__thickness=0.0))
    assert not any(p["slug"] == "washer" for p in none["parts"])
    assert none["Y"]["cpl_in"] == pytest.approx(data["s"]["stack"]["gap"] / 2)
    with pytest.raises(ValueError, match="bore"):
        cr.layout(variant(tmp_path, washer__thickness=0.4, washer__id=5.0))


def test_only_a_diamond_is_drawn(tmp_path):
    """Two crank arms or two hinges is another module, not another number."""
    with pytest.raises(ValueError, match="not a diamond"):
        variant(tmp_path, linkage__config="config/swing_linkage_stagger.yaml")


def test_one_rod_size_sizes_every_metal_hole(tmp_path):
    """1/4 in stock: the four rods and every hole they run or press in follow."""
    Lq = cr.layout(variant(tmp_path, rod__dia=6.35))
    r = 6.35 / 2
    for slug in ("rod", "crankpin", "rpinR", "rpinL"):
        assert part(Lq, slug)["add"][0]["r"] == pytest.approx(r)
    for slug in ("couplerR", "rockerR", "knuckleR", "bulkheadF", "lowercase", "crankR"):
        assert any(q["k"] == "cyl" and q["r"] == pytest.approx(r) for q in part(Lq, slug)["cut"]), slug


def test_the_left_parts_are_the_right_ones_turned(L):
    """Each L/R pair is one part: the left one's primitives are the right one's
    turned 180 deg about Z. The crank halves are not a pair: only the rear one
    has lugs."""
    for p in L["parts"]:
        if not p["twin"]:
            continue
        q = part(L, p["twin"])
        assert p["add"] == [cr.turned_prim(v) for v in q["add"]], p["slug"]
        assert p["cut"] == [cr.turned_prim(v) for v in q["cut"]], p["slug"]
    lugs = lambda s: sum(q["k"] == "prism" for q in part(L, s)["add"])  # noqa: E731
    assert lugs("crankR") == 4 and lugs("crankL") == 0


def test_the_bearing_nut_slots_are_blind_the_rest_run_through(L, tmp_path):
    """The front bulkhead's, lower case's and upper case's slots run along Y
    and stop, out through the face each part prints on (user); every other
    slot runs through."""
    js = {j["tag"]: j for j in L["RG"]["joints"]}
    wing = ("jWR", "jWL", "jKR", "jKL")
    # the turned servo's upper case to the lower case (user, 2026-10-07):
    # along +Y, its nut slot up out of the lower case's block (the nut in from the top)
    ju = js["jUL"]
    assert not ju["through"] and ju["slot"] == [0.0, 0.0, 1.0] and ju["zIn"] == [0.0, 1.0, 0.0]
    assert (ju["csk"], ju["nut"]) == ("uppercase", "lowercase")
    j = js["jF"]
    assert not j["through"] and j["slot"] == [0.0, 1.0, 0.0]
    assert j["slotLen"] > (L["Y"]["bh_out"] - L["Y"]["bh_in"]) / 2      # it reaches the face
    # the lower case's ran through for a while with the servo turned; the
    # block that holds the upper case's nut closes its rear face again
    assert not js["jR"]["through"] and js["jR"]["slot"] == [0.0, 1.0, 0.0]
    # the upper case's, turned along Y (2026-10-05): out through its rear
    # face, its bed as it prints cap-down
    # no upper-case joint while the turned servo's case has no attachment;
    # with the servo up, the central one turned along Y (2026-10-05): out
    # through the case's rear face, its bed as it prints cap-down
    assert "jU" not in js
    ju = next(j for j in cr.layout(variant(tmp_path, cases__servo_end="up"))["RG"]["joints"] if j["tag"] == "jU")
    assert not ju["through"] and ju["slot"] == [0.0, -1.0, 0.0] and part(L, "uppercase")["up"] == [0.0, 1.0, 0.0]
    # the wings' nuts in the rockers and knuckles: blind, out through the boss's top end
    for t in wing:
        assert not js[t]["through"] and js[t]["nut"][:-1] in ("rocker", "knuckle")
    assert all(j["through"] for t, j in js.items() if t not in ("jF", "jR", "jU", "jUL") + wing)
    # front bulkhead prints inner face up, so its slot leaves by the outer face
    # (+Y, its bed); the lower case prints inner face down (bed at +Y too)
    assert part(L, "bulkheadF")["up"] == [0.0, -1.0, 0.0]
    assert part(L, "lowercase")["up"] == [0.0, -1.0, 0.0]


def test_the_links_hold_the_nuts_and_the_screws_come_from_both_sides(data, L):
    """`wing.nut_in: link` (user, 2026-10-07): each 6-32's head in the wing's
    tab, its nut in the rocker or knuckle with a wall past it; a wing's two
    screws run toward each other, so both go in from outside the module."""
    st, wg = data["s"]["stack"], data["s"]["wing"]
    wall, nut = data["s"]["print"]["min_wall"], data["screw"]["nutSlotThickness"]
    assert st["web"] - 2.0 - nut >= wall and st["coupler"] - 2.0 - nut >= wall
    assert wg["tab"] >= data["s"]["joint"]["joint_plate"]
    js = {j["tag"]: j for j in L["RG"]["joints"]}
    for side in "RL":
        w, k = js["jW" + side], js["jK" + side]
        assert w["csk"] == k["csk"] == "wing" + side
        assert w["zIn"][1] * k["zIn"][1] == -1.0
        # each head faces away from the mid-plane: it points in at the module
        for j in (w, k):
            assert j["zIn"][1] * j["head"][1] < 0


def _outline(prims, n=48):
    pts = []
    for q in prims:
        if q["k"] == "slot":
            a, b = np.array(q["a"]), np.array(q["b"])
            for k in np.linspace(0, 1, 12):
                c, r = a + k * (b - a), q["ra"] + k * (q["rb"] - q["ra"])
                pts += [c + r * np.array([math.cos(t), math.sin(t)]) for t in np.linspace(0, 2 * math.pi, n)]
        elif q["k"] == "prism":
            P = np.array(q["pts"])
            pts += [P[i] + k * (P[(i + 1) % len(P)] - P[i]) for i in range(len(P)) for k in np.linspace(0, 1, 12)]
    return np.array(pts)


def test_the_rear_knuckle_passes_under_the_servo_cases(tmp_path):
    """`knuckle_at: outside`: wing L's second knuckle sits under the XC330's
    lower case; over the whole stroke its ear stays well below the case's
    underside (the dog-leg's first reason for being)."""
    data = variant(tmp_path, wing__knuckle_at="outside", stack__thrust="frame", cases__servo_end="up",
                   wing__blade="none")
    L = cr.layout(data)
    kn = _outline(part(L, "knuckleL")["add"])
    case_bottom = L["C"][1] - (data["x330"]["shaftFromEnd"] + 0.05 + 2.3)
    worst = -np.inf
    for f in np.linspace(-1, 1, 41):
        g = cr.poses(data["lk"], data["T"] * f, L["C"], L["pin"], L["J"])["wL"]
        P = np.array([cr.apply(g, p) for p in kn])
        band = P[np.abs(P[:, 0]) <= 14.05 + 0.5]
        if len(band):
            worst = max(worst, band[:, 1].max())
    assert worst < case_bottom - 3.0


def test_knuckle_r_sits_behind_knuckle_l(tmp_path):
    """`knuckle_at: outside`, `wing.knuckle_r: rear` (user, 2026-09-30): knuckle R behind knuckle L,
    a bossed gap between them, the rod stopping at the front bulkhead, and
    wing R as long as wing L, shifted back so its tab meets the knuckle."""
    data = variant(tmp_path, wing__knuckle_at="outside", stack__thrust="frame", cases__servo_end="up",
                   wing__blade="none")
    L = cr.layout(data)
    assert data["s"]["wing"]["knuckle_r"] == "rear"
    Y, st = L["Y"], data["s"]["stack"]
    kl, kr = part(L, "knuckleL")["bbox"], part(L, "knuckleR")["bbox"]
    # behind, its ring (in its box) a running clearance off knuckle L's back face
    assert kr[1][1] == pytest.approx(kl[0][1] - st["clearance"])
    rod = part(L, "rod")["bbox"]
    assert rod[1][1] == pytest.approx(Y["bh_out"]) and rod[0][1] < kr[0][1]
    wl, wr = part(L, "wingL")["bbox"], part(L, "wingR")["bbox"]
    assert wr[1][1] - wr[0][1] == pytest.approx(wl[1][1] - wl[0][1])
    assert wr[0][1] == pytest.approx(kr[0][1] - data["s"]["wing"]["tab"])
    # the module now ends at the front bearing: the bulkhead, and the crank's
    # journal standing its 0.5 proud of it
    front = max(p["bbox"][1][1] for p in L["parts"]
                if p["bbox"] and not p["mock"] and p["slug"] not in ("wingL", "wingR", "bladeL", "bladeR"))
    assert front == pytest.approx(Y["jn_end"])


# Where the blades' toes hold the crank under full torque, plus margin: the
# farthest any knuckle and the other wing's coupler can get to each other.
TOE_STOP_MAX_DEG = 131.0


def test_the_knuckles_sit_beside_the_other_wings_coupler(data, L):
    """`wing.knuckle_at: coupler` (user, 2026-10-07): each knuckle in the
    other wing's coupler layer, so a wing's rocker and knuckle straddle its
    own coupler and the stack does not grow; every L part its R turned
    again. The other coupler closes on the knuckle past ~130 deg (it passes
    over the rod 7.6 from its axis, hence the thinner hub): 2.19 mm least at
    128.4, 1.99 at 129.3, 0.41 at 136.25. So the BLADES' TOES are the stop
    (user, 2026-10-09: "make the toe block the collision"), meeting at ~129.9
    and held inside 131 under full torque (test_righting_v2). Swept to 131
    both ways, it keeps 1.5 mm (~1.7 at 130.5). A dogleg coupler is the
    noted way to more travel (config/righting_blade.yaml)."""
    Y = L["Y"]
    kr, kl = part(L, "knuckleR")["bbox"], part(L, "knuckleL")["bbox"]
    bz = data["s"]["stack"]["boss"]
    assert (kr[0][1], kr[1][1]) == pytest.approx((Y["cpl_in"] - bz, Y["cpl_out"]))   # its ring into the mid gap
    assert (kl[0][1], kl[1][1]) == pytest.approx((-Y["cpl_out"], -Y["cpl_in"] + bz))
    assert part(L, "wingL")["twin"] == "wingR"
    # knuckle L is knuckle R turned, less knuckle R's ring at the rod
    ring = [q for q in part(L, "knuckleR")["add"] if q["k"] == "cyl" and q["x"] == 0.0 and q["z"] == 0.0]
    assert len(ring) == 1 and ring[0]["y1"] == pytest.approx(Y["cpl_in"])
    assert part(L, "knuckleL")["add"] == [cr.turned_prim(q) for q in part(L, "knuckleR")["add"] if q not in ring]
    rod = part(L, "rod")["bbox"]
    assert (rod[0][1], rod[1][1]) == pytest.approx((-Y["bh_out"], Y["bh_out"]))
    front = max(p["bbox"][1][1] for p in L["parts"]
                if p["bbox"] and not p["mock"] and p["slug"] not in ("wingL", "wingR", "bladeL", "bladeR"))
    assert front == pytest.approx(Y["jn_end"])
    kn = _outline([q for q in part(L, "knuckleR")["add"] if q["y0"] < Y["cpl_out"]], n=96)
    cp = _outline(part(L, "couplerL")["add"], n=96)
    worst = np.inf
    for f in np.linspace(-1, 1, 81):
        ps = cr.poses(data["lk"], TOE_STOP_MAX_DEG * f, L["C"], L["pin"], L["J"])
        A = np.array([cr.apply(ps["wR"], p) for p in kn])
        B = np.array([cr.apply(ps["cL"], p) for p in cp])
        near = B[np.linalg.norm(B - A.mean(0), axis=1) < np.ptp(A, axis=0).max() + 5]
        if len(near):
            worst = min(worst, cKDTree(near).query(A)[0].min())
    assert worst > 1.5


def _covers(q, pt, y):
    """Does primitive q's solid (holes ignored) hold point (x, z) at depth y?"""
    if q["k"] not in ("cyl", "slot", "box", "prism"):
        return False                                # the wing stub's ends: no ring bears there
    lo, hi = (q["lo"][1], q["hi"][1]) if q["k"] == "box" else (q["y0"], q["y1"])
    if not lo < y < hi:
        return False
    x, z = pt
    if q["k"] == "cyl":
        return math.hypot(x - q["x"], z - q["z"]) <= q["r"]
    if q["k"] == "slot":
        a, b = np.array(q["a"]), np.array(q["b"])
        t = np.clip(np.dot(np.array(pt) - a, b - a) / np.dot(b - a, b - a), 0, 1)
        return np.linalg.norm(np.array(pt) - (a + t * (b - a))) <= q["ra"] + t * (q["rb"] - q["ra"])
    if q["k"] == "box":
        return q["lo"][0] <= x <= q["hi"][0] and q["lo"][2] <= z <= q["hi"][2]
    if q["k"] == "prism":
        P, inside = q["pts"], False
        for i in range(len(P)):
            (x0, z0), (x1, z1) = P[i], P[i - 1]
            if (z0 > z) != (z1 > z) and x < x0 + (z - z0) * (x1 - x0) / (z1 - z0):
                inside = not inside
        return inside
    return False


def test_the_thrust_rings_locate_every_moving_group(data, L):
    """`stack.thrust: loop` (2026-10-07): read every ring off the layout,
    find the part it bears on (the next part along Y holding the ring's
    centre), and check no set of moving groups can slide along Y either way.
    At the mid-plane, knuckle R's ring closes the rod's gap (user); only the
    crankpin's needs its washer, and it is not counted here. Nothing bears
    on the front bulkhead."""
    st = data["s"]["stack"]
    bz, cl = st["boss"], st["clearance"]
    contacts = []
    for p in L["parts"]:
        if p["kind"] != "print" or p["mock"] or p["slug"] in ("washer", "rodwasher"):
            continue
        for q in p["add"]:
            if q["k"] != "cyl" or abs((q["y1"] - q["y0"]) - bz) > 1e-9:
                continue
            # a point on the ring itself, off the hole's axis (the crank's
            # shoulder ring sits round its journal)
            pt = (q["x"] + 0.9 * q["r"], q["z"])
            body = [r for r in p["add"] if r is not q]
            up = any(_covers(r, pt, q["y0"] - 0.05) for r in body)       # stands off the +Y face
            assert up != any(_covers(r, pt, q["y1"] + 0.05) for r in body), (p["slug"], q)
            y_far = q["y1"] + cl + 0.05 if up else q["y0"] - cl - 0.05
            on = [o for o in L["parts"] if o is not p and o["add"] and not o["mock"]
                  and any(_covers(r, pt, y_far) for r in o["add"])]
            assert len(on) == 1, (p["slug"], q, [o["slug"] for o in on])
            lo, hi = (p, on[0]) if up else (on[0], p)
            contacts.append((lo["group"], hi["group"], lo["slug"], hi["slug"]))
    pairs = {(a, b) for _, _, a, b in contacts}
    assert not pairs & {("couplerR", "couplerL"), ("rockerL", "bulkheadF")}
    assert {("knuckleL", "couplerL"), ("couplerR", "knuckleR"), ("rockerR", "knuckleL"),
            ("knuckleR", "rockerL"), ("knuckleL", "knuckleR")} <= pairs
    moving = ["crank", "cR", "cL", "wR", "wL"]
    for n in range(1, len(moving) + 1):
        for S in itertools.combinations(moving, n):
            S = set(S)
            assert any(a in S and b not in S for a, b, *_ in contacts), f"{S} slides +Y"
            assert any(b in S and a not in S for a, b, *_ in contacts), f"{S} slides -Y"


def test_the_wing_is_a_stub_and_the_blade_an_inverted_u_over_it(data, L):
    """`wing.blade: ends` (user, 2026-10-07): the printed wing is a stub
    whose two Y ends are tongues, both faces chamfered 45 deg along the
    panel; the blade (a dummy) is an inverted U over it. Across the panel
    (n, + inward) both keep the panel's own faces, so nothing grows and the
    floor clearance at full stroke is the panel's. Along Y the blade spans
    the panel's ends, the stub stops short of them by the legs."""
    wg = data["s"]["wing"]
    half = wg["panel_thickness"] / 2
    foot, top = L["foot"], L["top"]
    w = (top - foot) / np.linalg.norm(top - foot)
    n_in = np.array([-w[1], w[0]])
    if np.dot(n_in, -foot) < 0:
        n_in = -n_in
    w3, n3 = np.array([w[0], 0, w[1]]), np.array([n_in[0], 0, n_in[1]])

    def ext(prims):
        """(u, n, y) ranges over every corner, relative to the panel."""
        P = []
        for q in prims:
            if q["k"] == "poly":
                P += list(cr.poly_corners(q))
            elif q["k"] == "prism":
                P += [np.array([x, y, z]) for x, z in q["pts"] for y in (q["y0"], q["y1"])]
        P = np.array(P) - np.array([foot[0], 0, foot[1]])
        return P @ w3, P @ n3, P[:, 1]

    blade, stub = part(L, "bladeR"), part(L, "wingR")
    assert blade["kind"] == "dummy" and part(L, "bladeL")["twin"] == "bladeR"
    u, n, y = ext(blade["add"])
    assert n.min() == pytest.approx(-half) and n.max() == pytest.approx(half)
    span = (-wg["panel_back"], wg["panel_rear"])
    assert (y.min(), y.max()) == pytest.approx(span)
    panel = [q for q in stub["add"] if q["k"] in ("prism", "poly")][:3]       # core + the two tongues
    u, n, y = ext(panel)
    assert n.min() == pytest.approx(-half) and n.max() == pytest.approx(half)
    # a border round the tabs and bosses (user: print as little as tests it)
    lw, c, bd = *wg["blade_ends"], wg["stub_border"]
    tabs = [(q["y0"], q["y1"]) for q in stub["add"] if q["k"] == "prism"][1:]
    assert (y.min(), y.max()) == pytest.approx((min(a for a, _ in tabs) - bd, max(b for _, b in tabs) + bd))
    assert u.max() == pytest.approx(wg["boss_from_foot"] + wg["boss_length"] + bd)
    # each tongue: both faces chamfered 45 deg, c deep in Y and in n (as it
    # prints outer face down the outer chamfer faces down at 45, no steeper)
    for q in panel[1:]:
        flat = {round(a, 6) for a, b in q["pts"] if abs(abs(b) - (half - c)) < 1e-9}
        assert len(flat) == 1
        f = flat.pop()
        assert min(abs(a - f) for a, b in q["pts"] if abs(abs(b) - half) < 1e-9) == pytest.approx(c)
