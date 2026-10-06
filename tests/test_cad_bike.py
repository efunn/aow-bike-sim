"""The whole-bike CAD, checked without Onshape.

`--check` builds the bike in Onshape and sweeps the steer and the righting
for interference; these check what can be checked locally -- that the three
modules' geometry layers merge (shared helpers kept once, a disagreement
refused), that the studio lints and reads no names during regeneration, and
that the layout's own clearances hold.
Invalidated by: config/bike_cad.yaml, any module generator (cad_drive,
cad_steering, cad_righting) or its config, or the shared FeatureScript in
cad_ahrs_fixture / cad_servo_mount.
"""

import pytest

from aow_sim import cad_bike as cb
from aow_sim import cad_servo_mount as sm

pytestmark = pytest.mark.cad


@pytest.fixture(scope="module")
def texts():
    return cb.module_texts()


@pytest.fixture(scope="module")
def data():
    return cb.load()


@pytest.fixture(scope="module")
def L(data):
    return cb.layout(data)


@pytest.fixture(scope="module")
def L_full(data):
    """The layout with everything that is switched off for now switched back
    on: the underside devices and their cables (which need the Pi's USB edge
    down), the cage, the first layout's gap and single post row."""
    import copy
    d = copy.deepcopy(data)
    el = d["s"]["electronics"]
    el.update(underside=True, gap=19.0, edge=-33.0)
    el["pi"]["usb_edge"] = "down"
    d["s"]["rollcage"]["enabled"] = True
    d["s"]["chassis"].update(post_at=[[11.0, 24.5]], post_mirror=True)
    # at the placement they were laid out for: the cage's slope does not
    # clear the electronics at the shorter wheelbase (it is tentative anyway)
    d["s"]["placement"].update(wheelbase=250.0, righting_y=160.0)
    return cb.layout(d)


@pytest.fixture(scope="module")
def studio(L):
    return cb.build_fs(L)


def test_the_full_layout_still_builds(L_full):
    """Switched-off parts are switched off, not deleted: the studio with them
    all on still lints."""
    sm.lint_fs(cb.build_fs(L_full))
    names = {m["name"] for m in L_full["mocks"]}
    assert {"U2D2", "AHRS", "power board", "switch"} <= names and L_full["cables"] and L_full["ribs"]


def test_shared_helpers_are_kept_once(texts):
    merged = cb.merge_layers(texts)
    for name in ("servoMountGeometry", "screwJoint", "dress", "partAt"):
        assert merged.count(f"export function {name}(") == 1, name
    for name in ("steeringBuild", "driveBuild", "rightingBuild", "rgPose"):
        assert f"export function {name}(" in merged, name


def test_a_shared_helper_that_differs_is_refused(texts):
    t = dict(texts)
    # a code edit: a comment would not count, the merge compares stripped text
    t["drive"] = t["drive"].replace("EntityType.BODY),\n                                    BodyType.SOLID), pt);",
                                    "EntityType.BODY),\n                                    BodyType.SHEET), pt);", 1)
    assert t["drive"] != texts["drive"]
    with pytest.raises(SystemExit, match="partAt"):
        cb.merge_layers(t)


def test_studio_lints_and_reads_no_names(studio):
    sm.lint_fs(studio)
    # a Part Studio throws on getProperty during regeneration; the eval does not
    assert "getProperty" not in sm._strip_comments(studio)


def test_lint_catches_a_reserved_word_read_as_a_key():
    with pytest.raises(SystemExit, match="switch"):
        sm.lint_fs("const a = BK.switch.tab;")
    with pytest.raises(SystemExit, match="box"):
        sm.lint_fs("const a = BK.carrier.box;")      # a chain: the first key must not hide the second


def test_posts_stand_on_the_drive_blocks_face(L):
    """The carrier's four posts land inside the block's front face (|x| <= 16,
    |z| <= 29.75 in the drive frame), clear of the tray screw's nut slot."""
    r = L["posts"]["r"]
    tray = next(j for j in L["joints"] if j["tag"] == "jTray")
    for (x, y, z), _ in L["posts"]["cyl"]:
        assert abs(x) + r <= 16.0 and abs(z) + r <= 29.75, (x, z)
        assert abs(x - tray["head"][0]) - r >= 6.55 / 2, (x, tray["head"][0])


def test_off_for_now_means_absent(L):
    """underside false, rollcage off (user, 2026-10-05): no devices, cables,
    rails, switch or cage; the carrier is one plate round the Pi."""
    names = {m["name"] for m in L["mocks"]}
    assert not names & {"U2D2", "AHRS", "power board", "switch", "switch bezel"}
    assert not any(n.startswith("USB") for n in names)
    assert not L["cables"] and not L["rails"] and not L["sw"] and not L["ribs"] and not L["cage"]
    assert not any(j["tag"].startswith(("jSp", "jRib")) for j in L["joints"])
    pi = next(m for m in L["mocks"] if m["name"] == "Pi 3B+")
    plates = L["carrier"]["plates"]
    lo = [min(b[0][i] for b in plates) for i in range(3)]
    hi = [max(b[1][i] for b in plates) for i in range(3)]
    assert lo[2] < pi["lo"][2] and pi["hi"][2] < hi[2]
    assert lo[0] <= pi["lo"][0] and pi["hi"][0] <= hi[0]


def test_the_carrier_works_round_the_deck_the_pulleys_and_its_holes(L, data):
    """On the face (2026-10-05): the middle stops over the deck, the legs
    carry the board's lower holes, a pocket clears each drive pulley, and the
    printed standoffs' self-tap pilots stop short of every pocket."""
    from aow_sim.cad_bike import data_deck_top, rx
    el = data["s"]["electronics"]
    mid, legR, legL = L["carrier"]["plates"]
    assert rx([0, mid[0][1], mid[0][2]], L["tilt"])[2] >= data_deck_top(data["rL"], L["zr"]) + el["notch"]["deck_clear"] - 1e-6
    assert legR[0][0] == mid[1][0] and legR[0][2] < mid[0][2]
    for hx, hz in L["piHoles"]["at"]:
        leg = legR if hx > 0 else legL
        assert leg[0][0] < hx < leg[1][0] and leg[0][2] < hz < leg[1][2]
    for plo, phi in L["carrier"]["pockets"]:
        assert L["piHoles"]["y"][0] >= phi[1] + 0.5 - 1e-9
    assert len(L["carrier"]["pockets"]) == 2
    assert L["piPilotDepth"] >= data["s"]["electronics"]["pi"]["standoff"] + 2.0
    standoffs = [b for b in L["bosses"] if [b[0][0], b[0][2]] in L["piHoles"]["at"]]
    assert len(standoffs) == 4


def test_the_pi_turned_puts_its_ports_up_and_its_holes_through_the_board(L, data):
    """usb_edge up: the USB/Ethernet block at the board's upper end, the
    holes 3.5 from the other end and through both the board and the plate."""
    pi = next(m for m in L["mocks"] if m["name"] == "Pi 3B+")
    usb = next(m for m in L["mocks"] if m["name"] == "Pi USB + Ethernet")
    assert usb["hi"][2] == pytest.approx(pi["hi"][2] + 2.0)
    h = pi["holes"]
    assert h["at"] == L["piHoles"]["at"] and len(h["at"]) == 4
    zs = sorted({z for _, z in h["at"]})
    assert zs[0] - pi["lo"][2] == pytest.approx(3.5) and zs[1] - zs[0] == pytest.approx(58.0)
    assert h["y"][0] < pi["lo"][1] and pi["hi"][1] < h["y"][1]


def test_every_carrier_screw_has_a_nut_somewhere(L, L_full, data):
    """Near the face (2026-10-05) the carrier is the joint plate on short
    pads and the nuts sit in the drive block, their slots out of its sides;
    further off, each post is long enough for its own nut."""
    el = data["s"]["electronics"]
    assert el["carrier"][2] >= el["boss"]                 # the head's plate is the carrier itself
    for j in (j for j in L["joints"] if j["tag"].startswith("jPost")):
        x = j["head"][0]
        assert abs(x) + j["len"] >= 16.0 + 0.5            # the slot leaves the block's side (|x| 16)
    for (_, y0, _), (_, y1, _) in L["posts"]["cyl"]:
        assert y1 - y0 <= el["gap"] + 0.6                 # pads, not posts
    for (_, y0, _), (_, y1, _) in L_full["posts"]["cyl"]:
        assert y1 - y0 >= 6.0


def test_no_cradle_rail_cuts_into_a_device_or_a_post(L_full):
    L = L_full
    devs = [m for m in L["mocks"] if m["name"] in ("U2D2", "AHRS", "power board")]
    for b in L["rails"]:
        for m in devs:
            assert not all(b[0][i] < m["hi"][i] and m["lo"][i] < b[1][i] for i in range(3)), m["name"]
    r = L["posts"]["r"]
    for b in L["rails"]:
        for (x, y, z), (_, y1, _) in L["posts"]["cyl"]:
            assert not (b[0][0] < x + r and x - r < b[1][0] and b[0][2] < z + r and z - r < b[1][2]
                        and b[0][1] < y1), (x, z)


def test_the_cage_clears_the_electronics(L_full):
    """The rail and its slope stay `clearance` over every corner of the
    electronics' envelopes, the cable keep-out included."""
    L = L_full
    seg = L["spine"]["segs"]
    (y0, rz), (yb, _) = seg[0]
    (_, _), (yf, zf) = seg[2]
    w = L["spine"]["w"]
    for m in L["mocks"]:
        if m["frame"] != "drive" or m["name"] == "battery":
            continue
        for x in (m["lo"][0], m["hi"][0]):
            for y in (m["lo"][1], m["hi"][1]):
                for z in (m["lo"][2], m["hi"][2]):
                    q = cb.rx([x, y, z], L["tilt"])
                    if y0 <= q[1] <= yb:
                        assert rz - w / 2 > q[2], (m["name"], q)
                    elif yb < q[1] <= yf:
                        zl = rz + (zf - rz) * (q[1] - yb) / (yf - yb)
                        assert zl - w / 2 > q[2], (m["name"], q)


def test_righting_rod_height_comes_from_the_module(L):
    # the floor at -R, the rod 41.2 above it
    assert L["zr"] - L["floor"] == pytest.approx(41.2, abs=0.05)


def test_the_cables_bend_no_tighter_than_specified(L_full):
    """Each cable's path: no bend tighter than 6 mm centreline radius (user),
    the 6 mm hard stub straight out of its plug, and the turn outside the
    carrier's lower edge."""
    L = L_full
    import numpy as np
    for c in L["cables"]:
        P = np.array(c["pts"])
        # radius of curvature through every three consecutive samples
        for a, b, d in zip(P, P[1:], P[2:]):
            ab, bd, ad = np.linalg.norm(b - a), np.linalg.norm(d - b), np.linalg.norm(d - a)
            area2 = np.linalg.norm(np.cross(b - a, d - a))
            if area2 > 1e-9:
                assert ab * bd * ad / (2 * area2) >= 6.0 - 0.05, c["name"]
        # the first 6 mm past the plug run straight along the plug's axis
        assert np.allclose(P[1][[0, 1]], P[0][[0, 1]]) and P[0][2] - P[1][2] >= 6.0 - 1e-6
    edge = L["carrier"]["plates"][0][0][2]
    turn = max(min(p[2] for p in c["pts"]) for c in L["cables"])
    assert turn + 3.0 < edge


def test_a_righting_change_is_noticed(data, L, tmp_path):
    """The placement was probed against one righting; a different linkage or
    stack gives a different digest (and cad_bike says so), the same one the same."""
    import copy
    from aow_sim import cad_righting as cr
    assert cb.righting_digest(data["rL"]) == cb.righting_digest(cr.layout(cr.load()))
    rd = copy.deepcopy(data["rd"])
    rd["s"]["stack"]["coupler"] = 5.0
    assert cb.righting_digest(cr.layout(rd)) != L["rgDigest"]


def test_the_carrier_has_two_screws_on_a_diagonal(L):
    """Two screws (user, 2026-10-05), opposite corners, off the tray nut's
    slot at x 0."""
    js = [j for j in L["joints"] if j["tag"].startswith("jPost")]
    assert len(js) == 2
    (x0, _, z0), (x1, _, z1) = (j["head"] for j in js)
    assert x0 * x1 < 0 and z0 != z1


def test_fit_righting_judges_the_probe(L, data, tmp_path):
    """`--fit righting` passes when the righting keeps rear_clear at
    righting_y and front_clear to the straight front wheel, and fails, naming
    what binds, when either is short. Canned probe output; no Onshape."""
    yr = L["yr"]
    out = tmp_path / "fit.txt"

    def fake(rows):
        class O:
            @staticmethod
            def resolve(*a):
                return "url"

            @staticmethod
            def eval_featurescript(script, url):
                return {"console": "\n".join(rows)}

            @staticmethod
            def notice_lines(reply):
                return []
        return O

    good = [f"REARD|{yr + 0.25}|0.9|+0.00|bridge|XC430 A|0,0,0", f"REARD|{yr}|0.6|+0.00|bridge|XC430 A|0,0,0",
            f"REARD|{yr - 0.25}|0.4|-1.00|bridge|XC430 A|0,0,0", f"REARMIN|{yr}",
            "FRONT|80|5.25|front bulkhead|tire|0,23.5,0|wings-ball|3.2"]
    assert cb.fit_righting(L, data, fake(good), out)
    tight = good[:1] + [f"REARD|{yr}|0.3|+0.56|wing L|drive pulley L|0,0,0", f"REARMIN|{yr + 0.25}", good[-1]]
    assert not cb.fit_righting(L, data, fake(tight), out)
    close = good[:-1] + ["FRONT|80|4.1|front bulkhead|tire|0,23.5,0|wings-ball|3.2"]
    assert not cb.fit_righting(L, data, fake(close), out)
