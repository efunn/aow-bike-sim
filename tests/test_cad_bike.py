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
def studio(L):
    return cb.build_fs(L)


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


def test_no_cradle_rail_cuts_into_a_device_or_a_post(L):
    devs = [m for m in L["mocks"] if m["name"] in ("U2D2", "AHRS", "power board")]
    for b in L["rails"]:
        for m in devs:
            assert not all(b[0][i] < m["hi"][i] and m["lo"][i] < b[1][i] for i in range(3)), m["name"]
    r = L["posts"]["r"]
    for b in L["rails"]:
        for (x, y, z), (_, y1, _) in L["posts"]["cyl"]:
            assert not (b[0][0] < x + r and x - r < b[1][0] and b[0][2] < z + r and z - r < b[1][2]
                        and b[0][1] < y1), (x, z)


def test_the_cage_clears_the_electronics(L):
    """The rail and its slope stay `clearance` over every corner of the
    electronics' envelopes, the cable keep-out included."""
    from aow_sim import cad_bike as cb
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


def test_the_cables_bend_no_tighter_than_specified(L):
    """Each cable's path: no bend tighter than 6 mm centreline radius (user),
    the 6 mm hard stub straight out of its plug, and the turn outside the
    carrier's lower edge."""
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
