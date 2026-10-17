"""The battery / electronics / switch holder's layout (cad_power) and the
Pi + U2D2 carrier outline (cad_pi_carrier), checked without Onshape.

cad_bike_v2 --check builds them in the bike and sweeps them for clashes;
these check the fits the layouts themselves promise: the two joints on the
rail's, the nuts and the cut screws between the rail and the battery, the
heads clear in side view, the board square to the drive beside the battery,
the switch flush with the XT30 in the shelf, the strap's path and the
gussets round it, the wire notch, the battery stops, the U2D2 between the
pulleys with its JSTs in front of the AHRS.
Invalidated by: config/bike_v2_cad.yaml `power:` / `pi_carrier:` /
components, config/drive_cad.yaml rail.carrier_* / front_ear / back_ear /
battery_stop / case / pulley, the 6-32 table.
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
    """On the LEFT rail's two ears only: aft on the back ear in the rear
    arm's corner, fore on the front ear, the heads on the rail's outer face;
    each groove closed toward the drive's -Z, open along +Z (the holder
    slides in from the back + top, user); the board between them; the
    holder's ridges turned onto the slide."""
    assert not any(j["tag"].startswith("carrier") for j in data["rail"]["joints"])
    tags = {j["tag"]: j for j in data["rail"]["leftJoints"]}
    a, f = tags["carrier"], tags["carrier_front"]
    assert a["dir"] == pytest.approx([0.0, -1.0]) and f["dir"] == pytest.approx([0.0, -1.0])
    assert a["head"][0] < P["board"]["y"][0] and f["head"][0] > P["board"]["y"][1]
    assert all(j["ridgeRot"] == 45.0 for j in P["joints"])
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
    on the holder over either head, nor the left belt (both pulleys) and the
    pulleys' flanges."""
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


def test_the_strap_round_the_battery_and_the_wire_notch(data, P):
    """One strap near the switch, directly over and under the battery
    (user): the upper slit at the battery's top, the lower a notch in the
    panel's bottom edge; its run up the panel's outer face inside the
    switch's body and clear of the board's standoffs; a gusset either side
    of it under the shelf. The wire notch over the case side's wire gap."""
    bat = data["s"]["components"]["battery"]
    st = P["strap"]
    assert st["upper"][0] == pytest.approx(bat["at"][2] + bat["size"][2] / 2)
    assert st["upper"][1] == pytest.approx(P["switch"]["panel"][0])      # up to the shelf (user)
    assert st["lower"][0] == pytest.approx(data["drive"]["Zp"])
    # the lower notch as tall as the wire notch (no step, user)
    assert st["lower"][1] == pytest.approx(data["drive"]["Zp"] + data["s"]["power"]["wire_notch"][1])
    assert P["switch"]["cut"][0] - P["xi"] >= st["room"] - 1e-9
    # three gussets: aft (1.5 off the switch's cutout), ahead of the strap,
    # past the XT30
    (a0, a1), (f0, f1), (x0, x1) = P["gussets"]
    assert a1 <= st["y"][0] and a1 <= P["switch"]["cut"][2] - 1.5 + 1e-9 and f0 >= st["y"][1] and x0 > f1
    for y, _ in P["board"]["holes"]["at"]:
        assert y - 3.0 >= st["y"][1]
    n0, n1 = P["wireNotch"]
    assert n0 < data["drive"]["wireY0"] and n1 > data["drive"]["wireY1"]
    assert {c["tag"] for c in P["cuts"]} >= {"strapUp", "strapLow", "wires", "cutout", "xt30"}


def test_the_battery_stops_behind_the_battery(data, P):
    """User: hints on the rear arms "so the battery couldn't slide backwards
    into the rear wheel": just behind its rear face, 2 in past its side."""
    bat = data["s"]["components"]["battery"]
    back, side = P["stops"]
    assert back["y"][1] == pytest.approx(bat["at"][1] - bat["size"][1] / 2 - 0.3)
    assert back["x"][0] == pytest.approx(bat["size"][0] / 2 - 3.0)
    # and a leg along its side, 0.3 off it (user: "constrained laterally")
    assert side["x"][0] == pytest.approx(bat["size"][0] / 2 + 0.3) and side["y"][1] > back["y"][1]


def test_no_case_round_the_battery(P):
    """A panel, not a case (user): one part on the left, nothing at +x."""
    assert all(max(a["x"]) < 0 for a in P["adds"] if "x" in a)
    assert all(max(q[0] for q in a["pts"]) < 0 for a in P["adds"] if a.get("kind") == "wedge")


def test_the_pi_over_the_u2d2_between_the_pulleys(data):
    """The Pi carrier's outline: the U2D2 between the drive pulleys, off the
    AHRS's housing, its JSTs' keep-out in front of the AHRS's lower end
    (user); under a plate past the pulleys' rims; the Pi on it; attached to
    nothing yet (user)."""
    Q = data["piCarrier"]
    assert Q["u2d2Gap"] >= 0.5 and Q["jstPast"] >= 0.5 and Q["rimGap"] >= 1.0
    assert data["s"]["components"]["u2d2"]["size"][0] / 2 < data["drive"]["flangeIn"]
    assert Q["joints"] == []


def test_the_frames_round_trip():
    for q in ((62.37, 44.12), (120.0, -10.0)):
        assert cpw.drive(*cpw.bike(*q)) == pytest.approx(q)
    assert math.isclose(cpw.bike(10.0, 10.0)[0], 0.0, abs_tol=1e-12)
