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

import math
from pathlib import Path

import numpy as np
import pytest
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
    """Along Y from the mid-plane: couplers, web, hub zone, bearing, knuckle,
    each running pair boss + clearance apart; the couplers round the washer."""
    st, ws, Y = data["s"]["stack"], data["s"]["washer"], L["Y"]
    gb = st["boss"] + st["clearance"]
    assert Y["cpl_in"] == pytest.approx(ws["thickness"] / 2 + ws["clearance"])
    assert Y["web_in"] - Y["cpl_out"] == pytest.approx(gb)
    assert Y["bh_in"] - Y["hz_out"] == pytest.approx(gb)
    assert Y["kn_in"] - Y["bh_out"] == pytest.approx(gb)
    order = ["cpl_in", "cpl_out", "web_in", "web_out", "hz_out", "bh_in", "bh_out", "kn_in", "kn_out", "rod"]
    assert all(Y[a] < Y[b] for a, b in zip(order, order[1:]))


def test_the_washer_is_four_numbers(tmp_path, data, L):
    """An off-the-shelf washer is a config edit: its thickness moves the whole
    stack by the same amount; 0 removes it; a bore under the rod is refused."""
    nylon = cr.layout(variant(tmp_path, washer__thickness=0.8, washer__od=12.0, washer__id=6.4))
    shift = (0.8 - data["s"]["washer"]["thickness"]) / 2
    for k in ("cpl_in", "web_in", "bh_out", "kn_out"):
        assert nylon["Y"][k] == pytest.approx(L["Y"][k] + shift)
    w = part(nylon, "washer")
    assert w["add"][0]["r"] == pytest.approx(6.0) and w["cut"][0]["r"] == pytest.approx(3.2)
    none = cr.layout(variant(tmp_path, washer__thickness=0.0))
    assert not any(p["slug"] == "washer" for p in none["parts"])
    assert none["Y"]["cpl_in"] == pytest.approx(data["s"]["stack"]["gap"] / 2)
    with pytest.raises(ValueError, match="bore"):
        cr.layout(variant(tmp_path, washer__id=5.0))


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


def test_the_bearing_nut_slots_are_blind_the_rest_run_through(L):
    """The front bulkhead's and lower case's slots run along Y and stop, out
    through the face each part prints on (user); every other slot runs through."""
    js = {j["tag"]: j for j in L["RG"]["joints"]}
    for tag, part_up in (("jF", -1.0), ("jR", -1.0)):
        j = js[tag]
        assert not j["through"] and j["slot"] == [0.0, 1.0, 0.0]
        half = (L["Y"]["bh_out"] - L["Y"]["bh_in"]) / 2
        assert j["slotLen"] > half                          # it reaches the face
    assert all(j["through"] for t, j in js.items() if t not in ("jF", "jR"))
    # front bulkhead prints inner face up, so its slot leaves by the outer face
    # (+Y, its bed); the lower case prints inner face down (bed at +Y too)
    assert part(L, "bulkheadF")["up"] == [0.0, -1.0, 0.0]
    assert part(L, "lowercase")["up"] == [0.0, -1.0, 0.0]


def test_the_rocker_side_tab_stays_in_the_web_plane(data, L):
    """The panel's tab beside the rocker reaches the couplers' radius, so it
    must stop at the web plane's edge; the nut in it keeps a wall."""
    Y, wg = L["Y"], data["s"]["wing"]
    assert Y["web_out"] - wg["tab"] >= Y["web_in"] - 1e-9
    assert wg["tab"] - 2.0 - data["screw"]["nutSlotThickness"] >= data["s"]["print"]["min_wall"]


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


def test_the_rear_knuckle_passes_under_the_servo_cases(data, L):
    """Wing L's second knuckle sits under the XC330's lower case: over the
    whole stroke its ear stays well below the case's underside (the dog-leg's
    reason for being; the rocker has none)."""
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


def test_knuckle_r_sits_behind_knuckle_l(data, L):
    """`wing.knuckle_r: rear` (user, 2026-09-30): knuckle R behind knuckle L,
    a bossed gap between them, the rod stopping at the front bulkhead, and
    wing R as long as wing L, shifted back so its tab meets the knuckle."""
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
                if p["bbox"] and not p["mock"] and p["slug"] not in ("wingL", "wingR"))
    assert front == pytest.approx(Y["jn_end"])
