"""The v2 whole-bike CAD (aow-bike-whole-v2), checked without Onshape.

`--check` builds it in Onshape and sweeps the steer and the righting for
interference between modules and envelopes; these check what can be checked
locally -- the modules v2 composes (steering v2, the drive without its
fixture, the righting's V3 blade), the blades' room to the front tyre, the
envelopes where their config puts them, and that the studio lints.
Invalidated by: config/bike_v2_cad.yaml, any module generator (cad_drive,
cad_steering, cad_righting) or its config, or the shared FeatureScript in
cad_ahrs_fixture / cad_servo_mount.
"""

import pytest

from aow_sim import cad_bike_v2 as b2
from aow_sim import cad_servo_mount as sm

pytestmark = pytest.mark.cad


@pytest.fixture(scope="module")
def data():
    return b2.load()


@pytest.fixture(scope="module")
def L(data):
    return b2.layout(data)


def test_the_modules_are_the_v2_ones(data):
    """Steering v2 (servo out the front), the drive without the fixture mock,
    the righting drawing its blade file (V3) as stubs and a dummy blade."""
    t = b2.module_texts()
    assert "export const STEER_V2 = true;" in t["steering"]
    assert data["rd"]["s"]["wing"]["blade"] == "outline"
    assert {"wingR", "wingKR", "bladeR"} <= {p["slug"] for p in data["rL"]["parts"]}
    fs = b2.build_fs(b2.layout(data))
    assert '"fixture" : false' in fs.split("export function bikeV2Build")[1]


def test_the_blades_clear_the_front_tyres_sweep(data, L):
    assert min(L["wingRoom"].values()) >= data["s"]["placement"]["wing_margin"]


def test_each_envelope_is_its_size_where_its_config_puts_it(L):
    """A drive-frame box turns with the drive (45 deg about X): its X extent
    stays its width, and its centre lands at the drive's transform of `at`."""
    for c in L["comps"]:
        lo, hi = b2.comp_box(L, c)
        assert hi[0] - lo[0] == pytest.approx(c["size"][0])
        centre = b2.to_bike(L, c, [0, 0, 0])
        assert [(a + b) / 2 for a, b in zip(lo, hi)] == pytest.approx(centre)


def test_no_chassis_and_the_rightings_placeholders_dropped(L):
    """No chassis, deck or cage of the bike's own: the rails are the drive's,
    and the printed parts v2 adds are the two carriers (cad_power,
    cad_pi_carrier), the Pi's four separate spacers and the rollcages."""
    body = b2.build_fs(L).split("export function bikeV2Build")[1].split(b2.SPLIT_MARK)[0]
    assert not any(w in body for w in ("deck", "tray"))
    assert body.count("bk2Cage(") == 1
    assert body.count("bk2Carrier(") == 1 and len(L["carriers"]) == 6
    assert [c["name"] for c in L["carriers"]][2:] == [f"Pi spacer {k}" for k in (1, 2, 3, 4)]
    # the floor and the two rail placeholders (with the rails the righting
    # draws no chassis plate)
    assert len(L["rgDrop"]) == 3


def test_the_featurescript_builds_and_lints(L):
    text = b2.build_fs(L)
    sm.lint_fs(text)
    assert "getProperty" not in sm._strip_comments(text)


def test_the_rails_meet_their_joints(data, L):
    """The drive's side rails bolt to the steering's side pads and the
    righting's bridge lugs where those modules put their own joints, the
    rail's grooves along their ridges, clear of the fork's swept disc; the
    righting's rail placeholders are dropped; the AHRS sits on its tabs."""
    rc = L["railCheck"]
    assert rc["steer"] < 0.01 and rc["bridge"] < 0.01
    assert rc["forkRoom"] >= 1.0
    assert len(L["rgDrop"]) == 3
    fs = b2.build_fs(L).split("export function bikeV2Build")[1]
    assert '"rail" : true' in fs


def test_the_rollcage_fits_the_rails_and_its_cap(data, L):
    """The rails' cage tongues mirror about mid-wheelbase, the rear one's
    face inside the rear ring; the ring is the blades' width, 270 deg; the
    bayonet's lugs sit in their grooves against the stops and go in
    through the gaps (or the opening) 15 deg back; the bars' chins stop
    short of the bed on a chamfer, clear of the front pads and the holder's
    panel; the cap's cone prints
    at the user's ~60 deg (rollcage() refuses the rest)."""
    rc, C = data["s"]["rollcage"], L["cage"]
    assert L["railCheck"]["cageMirror"] == pytest.approx(0.0, abs=1e-6)
    assert 0 < C["tongueFace"] < rc["depth"]
    assert rc["r_out"] * 2 == pytest.approx(137.4)
    assert C["ringArc"][1] * 2 == pytest.approx(360 - rc["opening_deg"])
    for (a0, a1), g in zip(C["lugs_deg"], sorted(rc["slots_deg"])):
        assert a1 == pytest.approx(g + rc["slot_deg"]) and a0 > g
        back = (a0 - rc["slot_deg"], a1 - rc["slot_deg"])
        assert back[1] <= rc["opening_deg"] / 2 + 1e-9 or any(g0 <= back[0] and back[1] <= g1 for g0, g1 in C["gaps_deg"])
    # the chin's one chamfer: 45 deg at its inboard corner on the bed
    ye0, ye1 = C["chinY"]
    assert ye0 > C["tongueFace"] + 2.0 and ye1 - ye0 == pytest.approx(rc["bar_split_x"] - data["rail"]["cageIn"][0]["x"][0])
    # the sheet's groove: 1.0 at the face to a 0.5 flat end 3.0 deep
    s = C["sheet"]
    assert s[2][0] - s[3][0] == pytest.approx(0.5) and s[2][1] == pytest.approx(rc["depth"] - 3.0)
    assert 55.0 < C["cone_over_deg"] < 62.0 and C["cap_wall"] >= 1.5
    assert C["wb"] == pytest.approx(L["wb"])

