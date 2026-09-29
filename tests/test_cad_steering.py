"""The front steer module's CAD, checked without Onshape.

`--check` builds the FeatureScript in Onshape and measures every part against
`layout()`; these check `layout()` itself -- the axial stack and the fits the
parts depend on -- and that the generated FeatureScript lints.
Invalidated by: config/steering_cad.yaml, bike_params_cad's
steering.servo_clearance, the X330 envelope and mount interface, or the shared
FeatureScript in cad_ahrs_fixture / cad_servo_mount.
"""

import math

import pytest

from aow_sim import cad_servo_mount as sm
from aow_sim import cad_steering as st

pytestmark = pytest.mark.pure


@pytest.fixture(scope="module")
def data():
    return st.load()


@pytest.fixture(scope="module")
def L(data):
    return st.full_layout(data)


def test_the_stack_is_pinned_from_both_ends(data, L):
    """Horn face = tire radius + servo_clearance up the axis; block = tire_gap
    over the tire; the bushing is what is left, and long enough."""
    s = data["s"]
    assert L["R"] == pytest.approx(102.5 / 2)          # measured tire OD
    assert L["zH"] == pytest.approx(L["R"] + data["servo_clearance"])
    assert L["blockBot"] == pytest.approx(L["R"] + s["fork"]["tire_gap"])
    assert L["sleeve"] == pytest.approx(L["sleeveTop"] - L["caseBot"])
    assert L["sleeve"] >= s["headset"]["min_sleeve"]
    # top down, nothing overlaps: hub over lugs over upper over step over bushing
    order = ["zH", "socketFloor", "lugTop", "hubBot", "upTop", "nutFloor",
             "socketTop", "zS", "sleeveTop", "caseBot", "blockTop", "blockBot", "R"]
    assert all(L[a] > L[b] for a, b in zip(order, order[1:])), order


def test_the_central_screw_runs_from_below_into_the_nut_in_the_upper(data, L):
    """A 6-32 x 3/8 flat head up a bore in the headset lower: its tip passes
    the nut sitting in the upper's pocket and stays inside the upper; its seat
    is in the shaft, below the step."""
    hs = data["s"]["headset"]
    assert hs["central_screw_length"] == pytest.approx(25.4 * 3 / 8)
    assert L["nutTop"] < L["tip"] < L["upTop"]
    assert L["tip"] - L["headFace"] == pytest.approx(hs["central_screw_length"])
    assert L["blockBot"] < L["cbTop"] < L["headFace"] < L["cskTop"] <= L["zS"]
    assert L["shaftR"] - L["cbR"] >= 2 * data["s"]["print"]["min_wall"]


def test_a_servo_too_low_is_refused(data):
    """layout() raises rather than drawing a stack that cannot be built."""
    with pytest.raises(ValueError, match="bushing"):
        st.layout({**data, "servo_clearance": 33.0})


def test_the_upper_bears_on_the_step_not_the_spigot_or_the_sleeve(data, L):
    """The socket is deeper than the spigot, and the step sits end_play above
    the sleeve's top -- so the clamp squeezes upper onto lower and never the
    bushing, and the upper's skirt is the lift-off retainer."""
    hs = data["s"]["headset"]
    assert L["socketTop"] - L["zS"] > hs["spigot_height"]
    assert L["zS"] - L["sleeveTop"] == pytest.approx(hs["end_play"])
    assert L["upR"] > L["boreR"]                      # the skirt overlaps the sleeve top
    assert L["sockSpR"] > L["spR"] and L["sockSpFlat"] > L["spFlat"]


def test_every_hub_wall_is_two_perimeters(data, L):
    """The user's rule: >= min_wall (2 perimeters) between any two features of
    the hub, set by the SCREWS version -- counterbore to socket arm,
    counterbore to rim, the centre island between arms -- and the lugs' roots
    off the upper's nut pocket."""
    w = data["s"]["print"]["min_wall"]
    assert w == pytest.approx(1.0)
    for k in ("cbToSocket", "cbToRim", "islandWall"):
        assert L[k] >= w, k
    assert L["lugR0"] - L["nutR"] >= w
    hb = data["s"]["hub"]
    assert hb["screw_head_dia"] == pytest.approx(3.4) and hb["screw_head_height"] == pytest.approx(1.5)
    assert L["hubCbDepth"] >= hb["screw_head_height"]
    assert L["m2Engaged"] <= data["table"]["XC330"]["hornHoleDepthMax"] * 1000 - 0.2


def test_the_socket_is_the_bigger_negative(L):
    """Hub socket arms start nearer the centre than the lugs and run out
    through the rim; the lugs run to the upper's rim; neither is a full cross."""
    assert 0 < L["sockR0"] < L["lugR0"]
    assert L["lugR1"] < L["upR"] and L["lugR1"] > L["upR"] - 0.5
    assert L["cavR"] <= L["inner"]        # no ledge over the cavity as printed


def test_the_fork_clears_the_tire_and_carries_the_block_on_a_ledge(data, L):
    """Leg inner faces outside the tire, the block wider than the gap between
    the legs (the ledge), one screw a side whose nut slot misses the thrust
    boss, equal bosses under the smaller insert, the shank leg face to leg
    face with the head proud, the knurl in the leg."""
    wh, ax, fk = data["s"]["wheel"], data["s"]["axle"], data["s"]["fork"]
    assert L["xIn"] > wh["tire_width"] / 2
    assert L["ledge"] > 1.0 and L["forkPocket"] >= 0
    # cheeks round the block's +-Y faces, two walls thick, a clearance off it
    assert L["cheekIn"] > fk["block_depth"] / 2
    assert L["forkHalf"] - L["cheekIn"] >= 2 * data["s"]["print"]["min_wall"]
    assert fk["leg_width"] == pytest.approx(15.0)
    assert L["forkSlotX"] >= L["thrustR"]
    assert fk["boss_dia"] < min(wh["hub_insert_dia_a"], wh["hub_insert_dia_b"])
    assert L["xBoss"] > wh["hub_width"] / 2
    assert 2 * L["xOut"] + ax["head_height"] == pytest.approx(ax["length"])
    assert L["knurlEnd"] >= L["xBoss"]
    assert L["blockBot"] > L["R"]


def test_the_cases_clear_the_cables_and_carry_both_pin_rows(data, L):
    """The connectors' window is cut down to the lower case's walls, the edge
    left across it a 20 deg gable in the nest shell (no flat bridge); slots
    through the cap either side of a 12 mm strip, to plug the connectors;
    the lower case's near-row pins stand clear of the turning cavity (else
    they are left off)."""
    cs = data["s"]["cases"]
    assert L["winApex"] > L["upBot"]
    rise, half = L["winBot"] - L["winApex"], (L["winNear"] - L["winFar"]) / 2
    assert rise / half == pytest.approx(math.tan(math.radians(cs["bridge_taper_deg"])))
    assert cs["bridge_taper_deg"] == pytest.approx(20.0)
    assert 2 * L["slotIn"] == pytest.approx(12.0)          # the H's strip, measured
    assert L["winBot"] == pytest.approx(L["wallTop"])      # cut down to the lower case
    assert L["lowerNearPins"] == 1.0
    assert L["lowerPinSeat"] >= data["s"]["cases"]["min_pin_seat"]


def test_the_pins_hub_is_the_x330_fixtures_latest(data):
    """PINS reads the X330 fixture's horn attach from its own config: pins by
    diameter (Phi 1.5 in the Phi 1.6 hole), no root relief, a Phi 16.0 well."""
    h = st.horn_opt(data)
    assert h["pinClearance"] == pytest.approx(0.1)
    assert h["rootRelief"] == 0.0
    assert data["table"]["XC330"]["hornDiameter"] * 1000 + h["boreClearance"] == pytest.approx(16.0)
    assert h["boreMouthChamfer"] == pytest.approx(0.4)


def test_the_spigot_is_six_across_and_still_two_perimeters_round_the_screw(data, L):
    assert data["s"]["headset"]["spigot_flats"] == pytest.approx(6.0)
    assert L["spFlat"] - data["screw"]["holeDia"] / 2 >= data["s"]["print"]["min_wall"]


def test_the_socket_trim_is_upper_only_and_the_tab_is_at_the_bikes_rake(data, L):
    """upper_socket_trim moves the upper's socket ceiling and nothing in the
    stack; the plate's tab is level when the module sits at bike.rake_deg."""
    trim = 0.2
    d2 = {**data, "s": {**data["s"], "headset": {**data["s"]["headset"], "upper_socket_trim": trim}}}
    L2 = st.layout(d2)
    assert L2["upSockTop"] == pytest.approx(L["upSockTop"] - trim)
    for k in ("zS", "socketTop", "sleeveTop", "nutFloor", "headFace", "caseBot"):
        assert L2[k] == pytest.approx(L[k]), k
    assert L["rakeDeg"] == pytest.approx(15.0)


def test_the_featurescript_builds_and_lints(data):
    text = st.build_fs(data)
    sm.lint_fs(text)
    assert "export const aowSteering = defineFeature" in text
    assert text.count(st.SPLIT_MARK) == 1
    for part in st.PARTS:
        assert f'"{part}"' in text
    # the case joints carry the ridge on the case (ridgeOnNut true)
    assert text.count("L.ridgeHalf, Y, Z)") == 1 and ", true, X, L.ridgeHalf" in text
