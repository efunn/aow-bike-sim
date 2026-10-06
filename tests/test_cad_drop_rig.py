"""The drop release rig's CAD, checked without Onshape.

`--check` builds the FeatureScript in Onshape and measures the parts;
these check `layout()` itself -- the stack from the base to the follower, the
cam's parked clearance, the interfaces borrowed from the fork and the
chainstay -- and that the generated FeatureScript lints.
Invalidated by: config/drop_rig_cad.yaml, bench/drop_cam.py's profile,
config/steering_cad.yaml or config/drive_cad.yaml (the two joints), the X330
envelope and case shells, or the shared FeatureScript in cad_ahrs_fixture /
cad_servo_mount.
"""

import math

import pytest

from aow_sim import cad_drop_rig as rig
from aow_sim import cad_servo_mount as sm

pytestmark = pytest.mark.cad


@pytest.fixture(scope="module")
def data():
    return rig.load()


@pytest.fixture(scope="module")
def L(data):
    return rig.full_layout(data)


def test_the_follower_rests_the_gap_over_the_dwell(data, L):
    """Drawn nominal: base top -> legs -> shaft -> dwell + gap = follower face,
    so the sketched lifts ARE the cam's drops; the follower clears the cover."""
    cm, so = data["r"]["cam"], data["r"]["servo"]
    assert L["Zc"] == pytest.approx(L["zBase"] + L["shellY1"] + so["base_gap"])
    assert L["Zf"] == pytest.approx(L["Zc"] + cm["r_dwell"] + cm["gap"])
    for h in cm["drops"]:
        assert (cm["r_dwell"] + cm["gap"] + h) - (L["Zf"] - L["Zc"]) == pytest.approx(h)
    assert L["padClear"] >= 0.5
    order = ["Zf", "coverTop", "Zc", "zPcbTop", "zPcbBot", "zBase", "zBaseBot"]
    assert all(L[a] > L[b] for a, b in zip(order, order[1:])), order


def test_parked_the_cam_is_clear_of_the_follower(data, L):
    """Parked park_deg past a step (force_drop.py --cam's default), every segment
    stays under the follower's pad and its chamfer: nothing touches the arm
    after a drop.
    And a CENTRED 10 mm pad -- the plan's first sketch -- would not."""
    assert L["parkClear"] >= 0.5
    it = data["r"]["interface"]
    centred = {**data, "r": {**data["r"], "interface": {**it, "follower_downstream": 5.0,
                                                        "follower_upstream": 5.0}}}
    assert rig.park_clearance(centred) < 0


def test_the_pad_covers_a_flat_followers_contact_offset():
    """Upstream, the pad reaches past where a flat follower touches the
    steepest ramp: dr/dtheta for the 2 mm drop."""
    d = rig.load()
    cm, it = d["r"]["cam"], d["r"]["interface"]
    ramp = math.radians(360 / len(cm["drops"]) - cm["dwell_deg"] - cm["top_deg"])
    assert it["follower_upstream"] >= (cm["gap"] + max(cm["drops"])) / ramp


def test_the_cam_clears_the_cover_by_a_millimetre(L):
    """The cam's back face 1 mm off the cover's cap (user)."""
    assert L["coverGap"] == pytest.approx(1.0)


def test_the_wheels_clear_the_other_buttons_and_the_pilots_run_through(data, L):
    """Standing on button 3, both wheels pass >= 3 mm over the others; the
    M3 pilots open through the base's underside, so a tap can run through."""
    assert data["r"]["pcb"]["drop_button_n"] == 3
    assert L["buttonClear"] >= 3.0
    assert L["pilotBot"] < L["zBaseBot"]


def test_the_web_chamfer_starts_under_the_block(L):
    """Printed X+, the web's underside rises at 45 deg from the pad's
    downstream corner and meets the block's underside."""
    assert L["webX0F"] >= L["S_blockBot"] and L["webX0R"] >= L["D_Yc0"]
    assert L["padX0"] - L["webX0F"] == pytest.approx(L["RaF"] - L["hTop"] - L["Zf"])


def test_each_interface_carries_its_own_wheels_radius(data, L):
    """Front: the tire's measured OD. Rear: omni outer_radius plus the
    contact offset under test. The follower is at one height over the button
    for both, so the follower's drop below each axle differs."""
    assert L["RaF"] == pytest.approx(102.5 / 2)
    assert L["RaR"] == pytest.approx(data["drive"]["wheel"]["R"] + data["r"]["wheels"]["rear_radius_offset"])


def test_one_base_serves_both_wheels(L):
    """The cam well clear of both wheels, and ONE
    bar-slot floor for both: the bar bottomed in either puts its axle over the
    button and its follower over the cam. The follower under the housing."""
    assert min(L["camClearF"], L["camClearR"]) >= 5.0
    assert L["slotX0"] >= max(L["endF"], L["endR"]) + L["slotFloor"]
    assert L["padX0"] < L["padX1"] <= L["slotX0"] + L["slotDepth"]


def test_the_release_corner_is_downstream_and_square(data, L):
    """The cam's material runs +X under the follower (horn facing -Y, turning
    clockwise looking at it), so the pad's +X end is the release and the
    chamfer is upstream."""
    it = data["r"]["interface"]
    assert L["padX1"] - L["Xc"] == pytest.approx(it["follower_downstream"])
    assert L["Xc"] - L["padX0"] == pytest.approx(it["follower_upstream"])
    assert L["Yh"] > 0


def test_the_interfaces_are_the_modules_own_joints(data, L):
    """Front: the steering's headset block. Rear: the drive's case-side outer
    faces. Read from those layouts, not copied."""
    S, D = data["S"], data["D"]
    assert (L["S_blockBot"], L["S_blockTop"], L["S_xBlk"]) == (S["blockBot"], S["blockTop"], S["xBlk"])
    assert (L["D_xPo"], L["D_Yj1"], L["D_tongueTop"]) == (D["xPo"], D["Yj1"], D["tongueTop"])
    assert L["housingHalfW"] < L["forkSlotIn"]


def test_the_cam_is_drop_cams_own_outline(data):
    """Four segments, the radii drop_cam.py draws, a closed polygon with no
    repeated points."""
    pts = rig.cam_points(data)
    cm = data["r"]["cam"]
    radii = [math.hypot(x, y) for x, y in pts]
    assert min(radii) == pytest.approx(cm["r_dwell"], abs=1e-6)
    assert max(radii) == pytest.approx(cm["r_dwell"] + cm["gap"] + max(cm["drops"]), abs=1e-6)
    assert all(math.dist(a, b) > 1e-4 for a, b in zip(pts, pts[1:] + pts[:1]))


@pytest.mark.parametrize("tones", [[(8, 0.6, 0)], [(32, 0.04, 0)], [(3, 1.0, 0), (13, 0.2, 0)]])
def test_the_wave_cams_nose_touches_and_never_cuts(tones):
    """A wave cam is the nose centre's path offset by the nose radius: at
    every cam angle a nose centred on that path touches the outline and does
    not cut into it, and the lowest point is lift_min over rest."""
    np = pytest.importorskip("numpy")
    dc = rig._cam_module()
    rest, nose, lift = 10.7, 0.45, 0.7
    P = np.array(dc.wave_profile(rest, nose, lift, tones, n=6000))
    R = dc.wave_pitch(rest, nose, lift, tones)[0]
    gaps = [np.min(np.hypot(*(P - [R(a) * math.cos(a), R(a) * math.sin(a)]).T)) - nose
            for a in np.linspace(0, 2 * math.pi, 400)]
    assert min(gaps) > -1e-4 and max(gaps) < 2e-3
    assert np.hypot(*P.T).min() == pytest.approx(rest + lift, abs=2e-3)


def test_the_featurescript_lints(data):
    sm.lint_fs(rig.build_fs(data))


def test_the_tip_jams_on_the_pad_and_its_nose_is_over_the_cam(data, L):
    """The tip's pocket is the pad's web plus the fit; the nose is centred
    over the cam axis and full height across the cam's width."""
    assert L["tipYi"] == pytest.approx(L["padHalfY"] + L["tipFit"])
    assert L["tipXi0"] < L["padX0"] and L["tipXd"] - L["tipW"] > L["padX1"]
    assert L["tipXoB"] < L["tipXi0"]                  # the plate reaches under the whole pad
    assert L["noseYn"] >= data["r"]["wave"]["thickness"] / 2
    assert 2 * L["noseR"] == pytest.approx(data["r"]["tip"]["nose_width"])


def test_the_wave_kit_fits_the_rig_in_use(data, L):
    """The tip clears the cover with the wheel RESTING, its nose stopping
    short of it; every cam in the kit clears the tip's plate away from the
    nose, the horn collar, and the drop cam's envelope; each is drawn with
    its lowest point under the follower, where the tip sits."""
    assert L["tipCoverClear"] >= 0.3
    assert L["noseYn"] <= L["coverY"] - 0.5
    assert L["tipCamClear"] >= 0.5
    assert len(data["r"]["wave_kit"]) >= 2
    for c in data["r"]["wave_kit"]:
        pts = rig.wave_points(data, c["tones"])
        radii = [math.hypot(x, y) for x, y in pts]
        assert min(radii) - L["collarR"] >= 0.5, c["name"]
        assert max(radii) <= L["rCamMax"], c["name"]
        top = max(y for x, y in pts if abs(x) < 0.1)
        assert L["tipZb"] - L["noseH"] - L["Zc"] == pytest.approx(top + 0.05, abs=0.02), c["name"]
