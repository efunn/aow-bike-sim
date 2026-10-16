"""The battery / electronics / switch holder's layout (cad_power) and the
Pi + U2D2 carrier outline (cad_pi_carrier), checked without Onshape.

cad_bike_v2 --check builds them in the bike and sweeps them for clashes;
these check the fits the layouts themselves promise: the two joints on the
rail's, the nuts and the cut screws between the rail and the battery, the
heads clear in side view, the board square to the drive beside the battery,
the switch flush with the XT30 in the shelf, the slits, the wire notch, the
U2D2 between the pulleys with its JSTs past the AHRS.
Invalidated by: config/bike_v2_cad.yaml `power:` / `pi_carrier:` /
components, config/drive_cad.yaml rail.carrier_* / front_ear / case /
pulley, the 6-32 table.
"""

import math

import pytest

from aow_sim import cad_bike_v2 as b2
from aow_sim import cad_power as cpw

pytestmark = pytest.mark.cad


@pytest.fixture(scope="module")
def data():
    return b2.load()


@pytest.fixture(scope="module")
def P(data):
    return data["power"]


def test_two_joints_into_the_left_rail_fore_and_aft_of_the_board(data, P):
    """Aft on the rear arm, fore on the case side's front ear, the heads on
    the rail's outer face; each groove opens UP (the holder drops in from
    above); the board between them."""
    tags = {j["tag"]: j for j in data["rail"]["joints"]}
    a, f = tags["carrier"], tags["carrier_front"]
    assert a["dir"][0] < 0 and a["dir"][1] < 0 and f["dir"][0] < 0 and f["dir"][1] < 0
    assert a["head"][0] < P["board"]["y"][0] and f["head"][0] > P["board"]["y"][1]
    assert [j["head"][0] for j in P["joints"]] == [-data["drive"]["xPo"]] * 2
    assert data["rail"]["earGap"] >= 1.0


def test_the_nuts_and_the_cut_screws_sit_between_the_rail_and_the_battery(data, P):
    """The panel is only battery to rail: each nut a wall past the joint
    plane and a wall short of the battery's side, the screws (cut down)
    through the nuts and clear of the pack."""
    n0, n1 = P["nut"]
    assert P["xi"] - n0 >= 1.0 - 1e-9 and n1 - P["wi"] >= 1.0 - 1e-9
    assert P["tip"] <= n0 - data["driveData"]["s"]["tension"]["nut_thickness"] + 0.05
    assert P["tipToPack"] >= 1.0


def test_the_screws_enter_clear_in_side_view(P):
    """User: the screw "should enter fully clear from the side view": nothing
    on the holder over either head, nor the left pulley's belt."""
    assert P["headView"] >= 0 and P["pulleyView"] >= 0


def test_the_board_beside_the_battery_and_the_switch_flush_with_the_xt30(data, P):
    """Square to the drive (XT30 up and back), outboard of the rail, a
    cable's room off the case side's top edge, off the left pulley's flange;
    its screws a grid's (2, 2) and 6 pitches up; the switch aft of it in the
    shelf, its bezel's top level with the XT30's."""
    bd = P["board"]
    assert bd["xs"] > data["drive"]["xPo"]
    assert bd["cableGap"] >= data["s"]["power"]["board"]["cable_gap"]
    assert bd["flangeGap"] >= 2.0
    (ya, za), (yb, zb) = bd["holes"]["at"]
    assert ya == yb and zb - za == pytest.approx(6 * 2.54)
    bezel = next(b for b in P["switch"]["boxes"] if b["name"] == "switch bezel")
    assert bezel["hi"][2] == pytest.approx(bd["xt30Top"])
    sw = next(b for b in P["switch"]["boxes"] if b["name"] == "switch")
    assert sw["hi"][1] <= bd["y"][0]                       # aft of the board
    assert sw["hi"][2] == pytest.approx(P["switch"]["panel"][0])


def test_two_slits_and_the_wire_notch(data, P):
    """Two strap slits, upper and lower (user); the panel's bottom notched
    over the left case side's wire gap, where the servo B wires come up."""
    lo, up = sorted(P["slits"], key=lambda s: s[2])
    assert up[2] > lo[3]
    n0, n1 = P["wireNotch"]
    assert n0 < data["drive"]["wireY0"] and n1 > data["drive"]["wireY1"]
    assert {c["tag"] for c in P["cuts"]} >= {"slit0", "slit1", "wires", "cutout", "xt30"}


def test_no_case_round_the_battery(P):
    """A panel, not a case (user): one part on the left, nothing at +x."""
    assert all(max(a["x"]) < 0 for a in P["adds"])


def test_the_pi_over_the_u2d2_between_the_pulleys(data):
    """The Pi carrier's outline: the U2D2 between the drive pulleys, off the
    AHRS's housing, its JSTs past the housing's end; under a plate past the
    pulleys' rims; the Pi on it; attached to nothing yet (user)."""
    Q = data["piCarrier"]
    assert Q["u2d2Gap"] >= 0.5 and Q["jstPast"] >= 0.5 and Q["rimGap"] >= 1.0
    assert data["s"]["components"]["u2d2"]["size"][0] / 2 < data["drive"]["flangeIn"]
    assert Q["joints"] == []


def test_the_frames_round_trip():
    for q in ((62.37, 44.12), (120.0, -10.0)):
        assert cpw.drive(*cpw.bike(*q)) == pytest.approx(q)
    assert math.isclose(cpw.bike(10.0, 10.0)[0], 0.0, abs_tol=1e-12)
