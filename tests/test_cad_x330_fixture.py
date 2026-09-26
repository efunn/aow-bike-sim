"""The X330 load-test fixture, checked without Onshape.

`--check` builds the FeatureScript in Onshape and looks for clashes and print
faults; these check the Python side it is compared against, and the things
the load test's numbers depend on.
Invalidated by: config/x330_fixture_cad.yaml, the X330 envelope in
bike_params_cad.yaml, or the shared FeatureScript in cad_ahrs_fixture.
"""

import math
import re

import pytest

from aow_sim import cad_servo_mount as sm
from aow_sim import cad_x330_fixture as fx

pytestmark = pytest.mark.pure


@pytest.fixture(scope="module")
def data():
    return fx.load()


def test_the_horn_pins_are_the_full_length_and_the_case_pins_are_not(data):
    """The lever drives its load through the horn pins, so they are the
    original 2.6 -- the shared default is 1.3. Case pins keep the shared 1.5."""
    text = fx.build_fs(data)
    assert data["fixture"]["horn_pin_length"] == pytest.approx(2.6)
    assert '"pinLength" : f.horn_pin_length' in text
    assert "export enum X330FixtureVersion" in text
    assert data["table"]["XC330"]["casePinLength"] * 1000 == pytest.approx(1.5)


def test_screwdriver_post_stops_the_lever_at_its_stop_angle(data):
    """The lever's underside, turned post_stop_deg, meets the post top at the
    post's outer edge -- and no sooner at its inner edge."""
    f, L = data["fixture"], fx.layout(data, "SCREWDRIVER")
    s = math.radians(f["post_stop_deg"])
    under = lambda r: -(r * math.sin(s) + f["arm_height"] / 2 * math.cos(s))  # noqa: E731
    r_out = f["stop_radius"] + f["stop_post"] / 2
    assert under(r_out) == pytest.approx(L["postTop"])
    assert under(r_out - f["stop_post"]) > L["postTop"]
    assert r_out < f["arm_length"]


def test_idler_stop_is_the_full_rigs_roll_stop(data):
    import yaml
    rig = yaml.safe_load(open("config/floor_rig.yaml"))
    assert data["fixture"]["stop_deg"] == pytest.approx(rig["roll_stop"]["deg"])


def _cover_top(data):
    sv = data["x330"]
    t = {k: v * 1000 for k, v in data["table"]["XC330"].items() if k != "hornHoleCount"}
    walls = t["caseSideClearance"] + t["caseTopWall"]
    return sv["shaftFromEnd"] + walls, sv["caseWidth"] / 2 + walls


def test_idler_yoke_stop_lands_flat_on_the_widened_cover(data):
    """Each V face, turned stop_deg, lies on the cover's top plane z = cy over
    stop_contact of width, all of it on the ledge-widened top, and all of it
    beyond cy * tan(stop), where a swinging face first can land flat."""
    f, L = data["fixture"], fx.layout(data, "IDLER")
    cy, cx = _cover_top(data)
    s = math.radians(f["stop_deg"])
    W = L["ledgeHalfWidth"]
    y_ext = W * math.cos(s) - cy * math.sin(s)
    ws = []
    for y in (-f["stop_flat"], -(f["stop_flat"] + y_ext) / 2, -y_ext):
        z = (cy - y * math.sin(s)) / math.cos(s)
        assert y * math.sin(s) + z * math.cos(s) == pytest.approx(cy)
        ws.append(y * math.cos(s) - z * math.sin(s))
    assert max(ws) - min(ws) == pytest.approx(f["stop_contact"])
    assert min(ws) == pytest.approx(-W)
    assert max(ws) < -cy * math.tan(s)


def test_idler_yoke_never_enters_the_cover_before_its_stop(data):
    """Sweep the stop's section and the bridge through +-stop_deg: nothing
    of the yoke is inside the cover or its ledges short of the stop."""
    import numpy as np
    f, L = data["fixture"], fx.layout(data, "IDLER")
    cy, cx = _cover_top(data)
    s = math.radians(f["stop_deg"])
    W, a, lw, Rs = L["ledgeHalfWidth"], f["stop_flat"], f["leg_width"], L["R_s"]
    y_ext = W * math.cos(s) - cy * math.sin(s)
    pts = []
    for y in np.linspace(-y_ext, y_ext, 201):
        zb = (cy + max(abs(y), a) * math.sin(s)) / math.cos(s)
        pts += [(y, z) for z in np.linspace(zb, Rs - lw / 2 + 1, 15)]
    pts += [(y, z) for y in np.linspace(-lw / 2, lw / 2, 30)
            for z in np.linspace(Rs - lw / 2, Rs + lw / 2, 10)]
    P = np.array(pts)
    for deg in np.linspace(-f["stop_deg"] + 0.25, f["stop_deg"] - 0.25, 121):
        t = math.radians(deg)
        y = P[:, 0] * math.cos(t) - P[:, 1] * math.sin(t)
        z = P[:, 0] * math.sin(t) + P[:, 1] * math.cos(t)
        body = (np.abs(y) <= cx) & (z <= cy)
        ledge = (np.abs(y) <= W) & (z <= cy) & (z >= cy - f["ledge_thickness"])
        assert not (body | ledge).any(), deg


def test_idler_arms_clear_the_plate_at_the_stop(data):
    """The lever hangs off the table's edge; what it must clear is the plate.
    Over the plate's half-width, the arm's lowest point at the stop stays
    above the plate's top."""
    f, L = data["fixture"], fx.layout(data, "IDLER")
    s = math.radians(f["stop_deg"])
    cy, cx = _cover_top(data)
    half = data["x330"]["caseWidth"] / 2 + 4.05 + f["plate_rim"]    # botOuter + rim
    r = half / math.cos(s) + f["arm_height"] / 2 * math.tan(s)      # arm radius over the plate edge
    lowest = -(r * math.sin(s) + f["arm_height"] / 2 * math.cos(s))
    assert lowest - L["plateTop"] > 5.0


def test_idler_lever_clears_the_thinned_cover_cap(data):
    """The cap's face sits under the horn face and the arms lever_gap past
    it: 1.5 mm, for the mechanism's sag under load."""
    f, sv, L = data["fixture"], data["x330"], fx.layout(data, "IDLER")
    cap_face = -sv["hornThickness"] + f["cover_cap_thickness"]
    assert cap_face <= 0.0                                   # not proud of the horn face
    assert L["leverTop"] - f["arm_thickness"] - cap_face == pytest.approx(1.5)
    assert L["leverTop"] - f["joint_plate"] > 0              # the leg's screw clears the cap


def test_the_mass_slots_and_ticks_fit_the_arm(data):
    f = data["fixture"]
    w = f["mass_slot_width"]
    assert w >= 5.0                                # a 10-32 (4.83 major) slides
    assert f["mass_slot_r1"] + w / 2 < f["arm_length"] - 2.0
    assert f["mass_slot_r0"] - w / 2 > f["leg_width"] / 2      # clear of the yoke leg
    wall = (f["arm_height"] - w) / 2
    assert wall - f["tick_depth"] > 2.5            # ticks leave the wall over the slot
    ticks = [f["tick_r0"] + k * f["tick_pitch"] for k in range(20)
             if f["tick_r0"] + k * f["tick_pitch"] < f["arm_length"] - f["tick_width"]]
    assert ticks[0] == pytest.approx(10.0) and ticks[-1] == pytest.approx(45.0)


def test_the_heads_sit_under_flush(data):
    """Both countersunk heads start head_recess up from the plate's bottom,
    so the plate clamps flat on anything."""
    f = data["fixture"]
    for v in fx.VERSIONS:
        L = fx.layout(data, v)
        assert L["plateTop"] - L["plateBottom"] == pytest.approx(f["joint_plate"] + f["head_recess"])
    assert f["head_recess"] > 0


def test_the_screw_reaches_its_nut_and_stays_under_the_case(data):
    """6-32 x 3/8 from under the plate: the nut slot 2 mm past the joint, the
    tip inside the slot, and all of it below the servo's far end."""
    f, sv, L = data["fixture"], data["x330"], fx.layout(data)
    sc = data["screw"]
    head = L["plateBottom"] + f["head_recess"]
    nut0 = head + f["joint_plate"] + 2.0
    tip = head + 9.525
    assert nut0 < tip <= nut0 + sc["nutSlotThickness"] + 0.5
    assert nut0 + sc["nutSlotThickness"] < -(sv["caseHeight"] - sv["shaftFromEnd"])


def test_generates_and_lints(data):
    text = fx.build_fs(data)
    sm.lint_fs(text)
    assert "export function x330FixtureBuild" in text
    assert "export function screwJoint" in text               # the shared helpers
    assert not re.search(r"%[A-Z_]+%", text)                  # every slot filled


def test_the_check_script_carries_the_stop_probe(data):
    w = fx.check_wrapper(fx.build_fs(data), data)
    f = data["fixture"]
    assert f"? {f['post_stop_deg']:g} : {f['stop_deg']:g};" in w
    assert "stopDeg + 1" in w and "stopDeg - 0.5" in w
    assert fx.STOP_PAIR["IDLER"] == {"yoke", "cover"}
    assert 'println("BED|"' in w                               # the shared print check


def test_screwdriver_stack_closes_through_the_inner_race(data):
    """The spring's push goes horn -> adapter -> hub -> boss -> the first
    bearing's INNER race: the boss must clear the outer race, the key must
    float in both slots, and the shaft must run in the bore."""
    f, L = data["fixture"], fx.layout(data, "SCREWDRIVER")
    inner_race_od = 12.0                                  # 608, nominal
    assert f["bearing_bore"] < f["boss_dia"] < inner_race_od
    assert f["key_length"] < 2 * f["slot_depth"]
    assert f["key_thickness"] < f["slot_width"]
    assert f["bearing_bore"] - f["shaft_clearance"] < f["bearing_bore"]
    assert L["blockX1"] - L["blockX0"] == pytest.approx(2 * f["bearing_width"] + f["bearing_gap"])


def test_screwdriver_block_screw_stays_under_the_bore(data):
    f, L = data["fixture"], fx.layout(data, "SCREWDRIVER")
    tip = L["plateBottom"] + f["head_recess"] + 9.525     # 6-32 x 3/8 flat head
    assert tip < -f["bearing_od"] / 2


def test_screwdriver_servo_stands_on_the_plate_with_the_shaft_on_the_bearing_axis(data):
    sv, L = data["x330"], fx.layout(data, "SCREWDRIVER")
    assert L["plateTop"] == pytest.approx(-(sv["caseHeight"] - sv["shaftFromEnd"]))


def test_idler_yoke_clears_the_cover_corner(data):
    f, sv, L = data["fixture"], data["x330"], fx.layout(data, "IDLER")
    t = {k: v * 1000 for k, v in data["table"]["XC330"].items() if k != "hornHoleCount"}
    walls = t["caseSideClearance"] + t["caseTopWall"]
    cy = sv["shaftFromEnd"] + walls
    corner = max(math.hypot(cy, sv["caseWidth"] / 2 + walls), math.hypot(cy, L["ledgeHalfWidth"]))
    assert L["R_s"] - f["leg_width"] / 2 - corner == pytest.approx(f["sweep_clearance"])


def test_idler_keel_ends_short_of_the_lever_joints_nut_slot(data):
    """The keel runs yoke_stop_length from the yoke's bed; the lever joint's
    nut slot (2 mm past the joint plane, slot thickness deep) runs through
    the yoke along Z and must not cut it into a thin feature."""
    f, sv, L = data["fixture"], data["x330"], fx.layout(data, "IDLER")
    back = -(sv["hornThickness"] + sv["caseDepth"])
    keel_end = back - f["idler_arm_thickness"] + f["yoke_stop_length"]
    x_joint = L["leverTop"] - f["joint_plate"]
    slot_far = x_joint - 2.0 - data["screw"]["nutSlotThickness"]
    assert keel_end < slot_far - 5.0


def test_idler_ledges_grow_out_of_the_side_wall_at_the_overhang_limit(data):
    """The ledge's bed-ward edge is a cone about its inner corner at the
    shells' slope, apex on the side wall's own sloped edge at that height --
    so it starts where the wall below it does, not at the top face's end
    (that was a wall's worth too high: the shell's own 2026-09-23 lesson)."""
    f, sv = data["fixture"], data["x330"]
    t = {k: v * 1000 for k, v in data["table"]["XC330"].items() if k != "hornHoleCount"}
    import yaml
    mounts = yaml.safe_load(open("config/servo_mounts.yaml"))
    over = mounts["servos"]["xc330_t181"]["case_wrap_overhang_deg"]["value"]
    tan_o = math.tan(math.radians(90 - over))
    cy, cx = _cover_top(data)
    far = sv["caseHeight"] - sv["shaftFromEnd"]
    x_wall = lambda z: -sv["hornThickness"] - (far - t["caseWrapLength"] + z) * tan_o  # noqa: E731
    z0 = cy - f["ledge_thickness"]
    assert x_wall(cy) == pytest.approx(-12.591, abs=0.01)          # measured in Onshape
    assert x_wall(z0) > x_wall(cy)                                 # the wall reaches further bed-ward low down
    # the cone about the corner line reaches the ledge's top-inner edge no
    # later than the wall itself does there: continuous with the wall
    assert x_wall(z0) - f["ledge_thickness"] * tan_o == pytest.approx(x_wall(cy))
