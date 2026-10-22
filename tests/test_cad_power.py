"""The battery / electronics / switch holder's layout (cad.power) and the
Pi + U2D2 carrier (cad.pi_carrier), checked without Onshape.

cad.bike_v2 --check builds them in the bike and sweeps them for clashes;
these check the fits the layouts themselves promise: the two joints on the
rail's, the nuts and the cut screws between the rail and the battery, the
heads clear in side view, the board square to the drive beside the battery,
the switch flush with the XT30 in the shelf, the strap's path and the
gussets round it, the wire notch, the battery stops, the U2D2 between the
pulleys with its JSTs in front of the AHRS, the Pi carrier's three joints.
Invalidated by: config/cad/bike_v2_cad.yaml `power:` / `pi_carrier:` /
components, config/cad/drive_cad.yaml rail.carrier_* / pi_* / front_ear /
back_ear / battery_stop / case / pulley, the 6-32 table.
"""

import math

import pytest

from aow_sim.cad import bike_v2 as b2
from aow_sim.cad import power as cpw

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
    # the aft screw cut down short of the battery, the fore hole through the
    # panel (user)
    aft, fore = (next(j for j in P["joints"] if j["side"] == s) for s in "AF")
    xo = data["drive"]["xPo"]
    assert xo - aft["holeDepth"] > P["wi"] and xo - fore["holeDepth"] < P["wi"]


def test_the_screws_enter_clear_in_side_view(P):
    """User: the screw "should enter fully clear from the side view": nothing
    on the holder over either head, nor the left belt (both pulleys) and the
    pulleys' flanges."""
    assert P["headView"] >= 0 and P["pulleyView"] >= 0


def test_the_panel_reaches_back_only_at_its_aft_ear(P):
    """The aft lug's extra (5.47) is an ear at the bottom, up to the aft
    ridge's top + 1; above it the panel stops at the shelf's aft gusset
    (user, 2026-10-19)."""
    ya0, ya1, ze = P["aftEar"]
    assert ya1 - ya0 == pytest.approx(5.474, abs=0.01) and ya1 == pytest.approx(P["gussets"][0][0])
    ys = [q[0] for q in P["outline"] if q[1] > ze + 1e-6]
    assert min(ys) == pytest.approx(ya1)
    assert min(q[0] for q in P["outline"]) == pytest.approx(ya0)


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
    """The U2D2 between the drive pulleys, off the AHRS's housing, its JSTs'
    keep-out in front of the AHRS's lower end (user); under a plate past
    the pulleys' rims, on two M2 through it 42 apart (user); the Pi over it
    on four spacers clear of those screws' heads, its 4-40s self-tapping
    into the plate and a boss behind it, the bosses off the belts."""
    Q = data["piCarrier"]
    p = data["s"]["pi_carrier"]
    assert Q["u2d2Gap"] >= 0.5 and Q["jstPast"] >= 0.5 and Q["rimGap"] >= 1.0
    assert data["s"]["components"]["u2d2"]["size"][0] / 2 < data["drive"]["flangeIn"]
    a, b = Q["u2d2Holes"]
    assert b - a == pytest.approx(42.0) and (a + b) / 2 == pytest.approx(p["u2d2_z"])
    assert p["pi_spacer"][0] >= p["u2d2_screws"][2] + 1.0
    assert Q["bossGap"] >= 1.0
    assert len(Q["spacers"]) == 4 and not any(a["tag"].startswith("so") for a in Q["adds"])
    # the plate is the Pi's PCB's footprint, no more (user), a gap off the
    # rails' top edge in side view
    pi = data["s"]["pi"]
    y0, y1, zl, zh, xh = Q["plate"]
    assert xh == pytest.approx(pi["board"][0] / 2) and zh - zl == pytest.approx(pi["board"][1])
    assert Q["railGap"] >= p["rail_gap"] - 1e-6
    # the JSTs in front of the AHRS's housing (the bump), not its whole
    # envelope (user)
    ah = data["s"]["components"]["ahrs"]
    assert b2._ahrs_bump_low(ah, data["driveData"]) > ah["at"][2] - ah["size"][2] / 2


def test_the_pi_carrier_on_three_screws_lifting_out_upward(data):
    """User: "three screws total holding it in (two at the bottom, one at
    the LHS top)": the top one on the LEFT only, the low ones a side, each
    head in the rail's outer face and its nut in a lug inboard of the rail;
    the ridges along the bike's Z (it lifts out straight up); the top lug
    off the power holder's front, the low ones over the righting's bridge;
    the heads clear in side view."""
    Q = data["piCarrier"]
    xo, xi = data["drive"]["xPo"], data["rail"]["innerX"]
    assert sorted(j["side"] for j in Q["joints"]) == ["LL", "LR", "T"]
    for j in Q["joints"]:
        assert abs(j["head"][0]) == pytest.approx(xo) and abs(j["ridgeAt"][0]) == pytest.approx(xi)
        assert j["ridgeRot"] == 0.0 and j["zIn"][0] == -math.copysign(1, j["head"][0])
    assert next(j for j in Q["joints"] if j["side"] == "T")["head"][0] < 0
    n0, n1 = Q["nut"]
    assert xi > n0 > n1 > Q["lugIn"]
    # through holes (user): through the lug, past its inner face
    assert all(j["holeDepth"] > abs(j["head"][0]) - Q["lugIn"] for j in Q["joints"])
    assert Q["holderGap"] >= data["s"]["pi_carrier"]["holder_gap"] - 1e-9
    assert Q["bridgeGap"] >= 1.0 and Q["headView"] >= 0 and Q["pulleyView"] >= 0
    # nothing proud of the plate's outer face (user: the low lugs were 0.51)
    y1 = Q["plate"][1]
    assert all(max(cpw.drive(*q)[0] for q in a["pts"]) <= y1 + 1e-6 for a in Q["adds"] if "pts" in a)


def test_the_frames_round_trip():
    for q in ((62.37, 44.12), (120.0, -10.0)):
        assert cpw.drive(*cpw.bike(*q)) == pytest.approx(q)
    assert math.isclose(cpw.bike(10.0, 10.0)[0], 0.0, abs_tol=1e-12)
