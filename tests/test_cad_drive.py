"""The rear drive module's CAD, checked without Onshape.

`--check` builds the FeatureScript in Onshape and measures every part against
`layout()`; these check `layout()` itself -- the axle stack, the belt plane,
the tension joint, the clearances the parts depend on -- the sketch profiles,
and that the generated FeatureScript lints.
Invalidated by: config/drive_cad.yaml, bike_params_cad's belt, XC430 and
omni_wheel blocks, or the shared FeatureScript in cad_ahrs_fixture /
cad_servo_mount.
"""

import math

import pytest

from aow_sim import cad_drive as cd
from aow_sim import cad_servo_mount as sm

pytestmark = pytest.mark.pure


@pytest.fixture(scope="module")
def data():
    return cd.load()


@pytest.fixture(scope="module")
def L(data):
    return cd.full_layout(data)


def test_the_belt_centres_solve_the_belt_length(data, L):
    """45T/15T on the 370 mm belt, at the exact length (nominal = what drove)."""
    bt = data["belt"]
    d1, d2 = bt["teethInput"] * 5 / math.pi, bt["teethServo"] * 5 / math.pi
    C = L["C"]
    assert 2 * C + math.pi / 2 * (d1 + d2) + (d2 - d1) ** 2 / (4 * C) == pytest.approx(370.0)
    assert C == pytest.approx(107.35, abs=0.01)
    assert math.hypot(L["Ys"], L["zs"]) == pytest.approx(C)
    assert L["zs"] == pytest.approx(28.5 / 2)          # the cases side by side, no gap


def test_the_axle_stack_is_the_hand_drawing(L):
    """Every station along X as drawn and printed, and the M5's head-to-nut
    span the user measured (50.5)."""
    drawn = {"hubFace": 15.0, "xHorn": 19.0, "xPo": 20.6, "flangeIn": 21.2, "teethIn": 22.7,
             "teethOut": 32.7, "outerFace": 34.2, "xCi": 35.2, "xCo": 40.2, "xE": 19.85,
             "xA": 23.25, "xCb": 22.25, "xCe": 19.42}
    for k, v in drawn.items():
        assert L[k] == pytest.approx(v, abs=0.01), k
    # the hex floor snapped to the chainstay's layer grid: up to half a layer
    # deeper each side than the M5's measured 50.5
    assert 50.5 - 0.2 <= 2 * L["clampX"] <= 50.5
    assert L["xA"] < L["clampX"] < L["xCo"]


def test_both_pulleys_share_the_belt_plane(data, L):
    """The belt plane is set from the servo side; the spline pulley's teeth
    land in the same band, the belt centred on it."""
    assert (L["beltX0"] + L["beltX1"]) / 2 == pytest.approx((L["teethIn"] + L["teethOut"]) / 2)
    assert L["beltX1"] - L["beltX0"] == pytest.approx(data["belt"]["width"])
    assert L["xCh"] >= L["hubFace"]


def test_the_cone_bearing_has_its_gap_and_the_stub_clears(data, L):
    """The chainstay's 60 deg nose is drawn cone_preload INTO the pulley's
    seat, as in the hand drawing (the M5 seats it by flexing the arm); the
    stub inside the counterbore, its end short of the bore."""
    tc = math.tan(math.radians(60))
    pulley_x = L["xCb"] - (L["cbR"] - L["rC0"]) / tc
    assert pulley_x - L["xRel"] == pytest.approx(data["s"]["chainstay"]["cone_preload"])
    assert L["stubR"] < L["cbR"] and L["xE"] > L["xCe"] and L["rC1"] > L["csBoreR"]


def test_the_htd_groove_is_the_drawn_one(data):
    """15T: tip 11.36, root 9.37, fillet centres at (+-2.13, 10.73) -- as read
    off the hand drawing's generator."""
    s = data["s"]
    q = cd.htd_groove(15, 5.0, s["pulley"]["pld"], s["teeth_15"])
    assert q["rt"] == pytest.approx(11.365, abs=0.005)
    assert q["rr"] == pytest.approx(9.375, abs=0.01)
    assert q["f"][0] == pytest.approx(2.13, abs=0.01) and q["f"][1] == pytest.approx(10.73, abs=0.01)
    assert q["t1"][0] == pytest.approx(1.70, abs=0.01)
    q45 = cd.htd_groove(45, 5.0, s["pulley"]["pld"], s["teeth_45"])
    assert q45["rt"] == pytest.approx(35.24, abs=0.01) and q45["rr"] == pytest.approx(33.24, abs=0.01)


def test_every_profile_is_closed(data):
    """Each segment starts where the last ended (the belt is two loops)."""
    for k, segs in cd.outlines(data).items():
        breaks = sum(a[-1] != b[1] for a, b in zip(segs, segs[1:] + segs[:1]))
        assert breaks == (2 if k == "BELT" else 0), k
    assert len(cd.outlines(data)["SPLINE"]) == 4 * 5


def test_the_spline_keeps_the_users_three_variables(data):
    sp = data["s"]["spline"]
    assert (sp["outer_dia"], sp["root_dia"], sp["lobe_deg"]) == pytest.approx((11.8, 8.8, 35.0))


def test_the_riser_sits_inside_the_belt_and_behind_the_flange(data, L):
    """The chainstay crosses the belt plane inside the loop, clear of both
    runs and the 45T's flange; the case's rear block clears the wheel."""
    assert L["riserBelt"] >= 0
    ch = data["s"]["chainstay"]
    corner = math.hypot(L["Ys"] - L["Yj1"], L["zs"] - L["armHalf"])
    assert corner == pytest.approx(L["rFl45"] + ch["flange_clearance"])
    assert L["Yc0"] >= data["wheel"]["R"] + data["s"]["case"]["tire_clearance"] - 1e-9
    assert L["Yr0"] < L["Yc0"] < L["Yscr"] < L["Yj1"] < L["Yb"]


def test_the_tension_screw_reaches_its_sliding_nut(data, L):
    """6-32 x 3/8 flat head, head outboard; the tip through the nut, a slot
    past the nut slot so it cannot bottom out; the web down to min_web (user);
    the screw slot as long as the travel, closed in the case's rear block,
    the nut slot running out its back."""
    tn, sc = data["s"]["tension"], data["screw"]
    assert L["xH"] - L["tip"] == pytest.approx(25.4 * 3 / 8)
    assert L["xN0"] - L["nutSlotT"] < L["tip"] + tn["tip_past_nut"]
    assert L["web"] >= tn["min_web"]
    assert L["tipPast"] >= tn["tip_past_nut"] and L["hTip"] < L["hT"]
    assert L["xRin"] < L["xN0"] - L["nutSlotT"] - tn["tip_slot"]     # the tip's slot, under a roof


def test_every_ceiling_is_on_the_layer_grid(data, L):
    """User: a ceiling's height from its part's bed is a whole number of
    layers, or the slicer rounds the bridge to the wrong layer. Case side
    (bed = outer face): the channel, the nut slot, the tip slot, the roof,
    the M2.5 seats. Chainstay (bed = outer face): the hex pocket's floor,
    the head bore's ring."""
    bl = data["s"]["joint"]["bridge_layer"]
    on = lambda h: abs(h / bl - round(h / bl)) < 1e-6     # noqa: E731
    case = {"channel": L["xPo"] - L["tongueTop"], "nut slot floor": L["hN0"],
            "nut slot": L["hN1"], "tip slot": L["hT"], "roof": L["xPo"] - L["xRin"],
            "M2.5 seat": L["xPo"] - L["m25Seat"]}
    stay = {"hex floor": L["xCo"] - L["clampX"],
            "head bore": L["xCo"] - (L["xH"] + L["headBoreR"] - L["headR"])}
    for k, h in {**case, **stay}.items():
        assert on(h), (k, h)
    assert data["s"]["tension"]["tip_slot"] == pytest.approx(0.6)
    assert L["Yscr"] - L["travel"] - L["holeR"] > L["Yc0"]
    assert L["Yscr"] + L["nutCornerY"] < L["Yb"]
    assert L["Yscr"] + L["headBoreR"] < L["Yj1"]
    assert L["travel"] > 0 and data["screw"]["cskAngle"] == pytest.approx(90.0)


def test_the_fixture_screws_are_reachable_with_the_pulley_on(L):
    """Their heads sit outside the drive pulley's flange, on the plate."""
    for y in (L["yF0"], L["yF1"]):
        assert math.hypot(y - L["Ys"], L["zF"] + L["zs"]) - L["rFl45"] - L["headR"] > 0
    assert L["zF"] + L["headR"] < L["Zp"]
    assert L["yF0"] - L["ridgeHalf"] >= L["Ytab0"] - 1e-9


def test_the_riser_flat_clamps_and_the_tongue_only_locates(data, L):
    """User: the riser's big flat clamps on the case side's outer face; the
    tongue is a rectangular key (no V to cam out) stopping tongue_top_gap
    short of the channel floor, narrow enough that the channel's ceiling is a
    short bridge, and only the shank passes it (the countersink is in the
    riser)."""
    tn = data["s"]["tension"]
    assert L["riserTop"] == pytest.approx(L["xPo"])
    assert L["tongueEnd"] - L["tongueTop"] == pytest.approx(tn["tongue_top_gap"])
    assert 2 * (L["tongueHalf"] + tn["tongue_clearance"]) <= 8.0     # was a 14.3 bridge
    assert L["cskWeb"] >= tn["csk_web"] and L["xH"] - L["cskDepth"] > L["xPo"]
    assert L["tongueHalf"] - L["holeR"] >= 1.5 * data["s"]["print"]["min_wall"]


def test_a_too_short_riser_room_is_refused(data):
    """layout() raises rather than drawing a tension screw with no room."""
    bad = {**data, "s": {**data["s"], "tension": {**data["s"]["tension"], "travel": 12.0}}}
    with pytest.raises(ValueError, match="tension screw"):
        cd.layout(bad)


def test_the_featurescript_builds_and_lints(data):
    text = cd.build_fs(data)
    sm.lint_fs(text)
    assert "export const aowDrive = defineFeature" in text
    assert text.count(cd.SPLIT_MARK) == 1
    for part in cd.PARTS:
        assert f'"{part}' in text
