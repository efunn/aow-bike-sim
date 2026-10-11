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
    body = b2.build_fs(L).split("export function bikeV2Build")[1].split(b2.SPLIT_MARK)[0]
    assert not any(w in body for w in ("deck", "tray", "carrier", "cage"))
    assert len(L["rgDrop"]) == 2


def test_the_featurescript_builds_and_lints(L):
    text = b2.build_fs(L)
    sm.lint_fs(text)
    assert "getProperty" not in sm._strip_comments(text)
