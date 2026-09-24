"""The AHRS fixture layout and the X330 idler, checked without Onshape.

`--check` on each generator builds the FeatureScript in Onshape and looks for
clashes; these check the PYTHON side it is compared against -- that the
derived roll-axis height really clears the yaw arm over the whole swing, in
every mount, and that the idler's defaults reproduce the part that was
actually printed.
Invalidated by: config/ahrs_fixture_cad.yaml, the idler block of
config/servo_mounts.yaml, or the X330 envelope in bike_params_cad.yaml.
"""

import math
import re

import numpy as np
import pytest

from aow_sim import cad_ahrs_fixture as fx
from aow_sim import cad_servo_mount as sm

pytestmark = pytest.mark.pure


@pytest.fixture(scope="module")
def data():
    return fx.load()


@pytest.mark.parametrize("r,w", [(35.5, 17), (5.0, 17), (-3.0, 17), (60, 17),
                                 (0.0, 17), (10.1, 17)])
def test_swept_depth_is_the_brute_force_maximum(r, w):
    th = np.radians(np.linspace(-45, 45, 90001))
    brute = np.max(np.maximum(r * np.cos(th) + w * np.sin(th),
                              r * np.cos(th) - w * np.sin(th)))
    assert fx.swept_depth(r, w, 45.0) == pytest.approx(brute, abs=1e-6)


@pytest.mark.parametrize("offset", [0.0, 30.0, 70.0])
def test_the_roll_stage_clears_the_yaw_arm_over_the_whole_swing(data, offset):
    """Sweep the actual boxes -- mount plate and wall, TM151 board and
    housing -- in all three mounts: the lowest point anywhere is the
    clearance above the yaw arm, and exactly it for the binding mount."""
    f = data["fixture"]
    floor = f["yaw_arm_top"] + f["stage_clearance"]
    lows = [fx.lowest_swept(data, c, offset) for c in fx.CONFIGS]
    assert min(lows) == pytest.approx(floor, abs=0.02)


def test_the_roll_axis_rises_with_the_offset(data):
    hs = [fx.layout(data, d)["H"] for d in (0, 30, 50, 70)]
    assert np.all(np.diff(hs) > 0)


def test_the_legs_clear_the_covers_corner(data):
    """A U leg sweeps round the servo's shaft end, which the full-wrap COVER
    now wraps: its inner face stays outside the cover's outer corner. (The
    base keeps the far-end wrap only -- a wrapped base cannot be printed.)"""
    f, sv = data["fixture"], data["x330"]
    t = {k: v * 1000 for k, v in data["table"]["XC330"].items() if k != "hornHoleCount"}
    walls = t["caseSideClearance"] + t["caseTopWall"]
    corner = math.hypot(sv["shaftFromEnd"] + walls, sv["caseWidth"] / 2 + walls)
    L = fx.layout(data)
    assert L["R_s"] - f["leg_width"] / 2 - corner == pytest.approx(f["sweep_clearance"])
    assert corner > math.hypot(sv["shaftFromEnd"], sv["caseWidth"] / 2) + 3


def test_the_mount_plate_keeps_a_rim_outside_the_pin_reliefs(data):
    f, tm = data["fixture"], data["tm151"]
    relief = data["table"]["XC330"]["casePinReliefDia"] * 1000 / 2
    hw = fx.plate_half_width(data)
    assert hw - (tm["hole_span_y"] / 2 + relief) == pytest.approx(f["mount_rim"])
    assert hw > tm["board_width"] / 2


def test_the_tm151_lands_centred_over_the_yaw_axis(data):
    """The roll horn's distance is derived so the housing centre is at x = 0."""
    tm, f = data["tm151"], data["fixture"]
    L = fx.layout(data)
    header_edge = -L["g"] + f["horn_mount_thickness"] + f["joint_plate"] + f["tm151_gap"]
    assert header_edge + tm["board_length"] / 2 - tm["housing_offset_x"] == pytest.approx(0.0)


def test_the_on_axis_screw_stops_short_of_the_horn(data):
    """J4's 3/8" screw goes -X from the mount's face into the horn mount;
    it must end before the horn face, or it bottoms on the horn."""
    f, sc = data["fixture"], data["screw"]
    reach = 9.525                               # head face to tip, 3/8"
    room = f["joint_plate"] + f["horn_mount_thickness"]   # head face to horn face
    assert room - reach > 2.0
    # and the nut slot, 2..5 past the joint, sits inside the horn mount
    assert f["joint_plate"] + 2 + sc["nutSlotThickness"] < room


def test_the_bolt_holes_sit_on_the_tab_clear_of_everything(data):
    """Four 10-32 clearance holes, wholly on the +X tab: past the shell, and
    inside the plate's edges with a wall to spare."""
    f = data["fixture"]
    b = fx.base_tab(data)
    r = f["bolt_hole_dia"] / 2
    assert b["x1"] > 0 > b["x0"]                    # the tab runs to +X now
    for x, y in b["holes"]:
        assert x - r > b["shell_end"] + 3
        assert x + r < b["x1"] - 3
        assert abs(y) + r < f["base_width"] / 2 - 3
    assert f["bolt_hole_dia"] == pytest.approx(5.1)  # #10 free fit, 0.201"


def test_the_tm151_hole_pattern_is_the_drawings(data):
    tm = data["tm151"]
    assert (tm["hole_span_x"], tm["hole_span_y"], tm["hole_dia"]) == (31.0, 30.0, 2.1)
    b = fx.tm151_boxes(tm)["board"]
    assert (b[1] - b[0], b[3] - b[2]) == pytest.approx((40.0, 34.0))


def test_the_idler_defaults_are_the_printed_part():
    """Phi 6.5 x 1.2 and Phi 5.0 x 0.6 steps bottomed in the recess, and a
    Phi 11 collar seated on the back face: wing-linkage-shorter's idler."""
    table = sm.servo_table(sm.load_params(sm.CAD_PARAMS))
    e = sm.idler_env(table)
    poly = [(u.val(e), v.val(e)) for u, v in sm.idler_profile()]
    assert 2 * e["idlerRo"] == pytest.approx(6.5)
    assert 2 * e["idlerRi"] == pytest.approx(5.0)
    assert min(v for _, v in poly) == pytest.approx(-1.8)      # recess floor
    assert (2 * e["idlerRc"], 0.0) in [(pytest.approx(2 * u), pytest.approx(v))
                                       for u, v in poly]
    assert e["backZ"] == pytest.approx(-26.0)                  # 3 + 23


def test_the_screw_cutter_volume_adds_up():
    sc = sm.screw_table()
    e = sm.screw_env(sc)
    # 90 deg countersink: the cone's depth equals its radial step
    assert e["cskDepth"] == pytest.approx(e["headR"] - e["holeR"])
    one, both = sm.expected_screw(sc, False), sm.expected_screw(sc, True)
    extra = sc["nutSlotWidth"] * sc["nutSlotThickness"] * (
        sc["nutSlotLength"] - sc["nutSlotWidth"] / math.sqrt(3))
    assert both["vol"] - one["vol"] == pytest.approx(extra)


def test_only_the_keep_out_is_excused_from_the_clash_check():
    """Pins, shells, horn wells and idler plugs are all checked against
    envelopes that carry the real holes; nothing is excused but the plug's
    keep-out, and "TM151" must not swallow "TM151 mount"."""
    assert fx._intended("TM151", "USB keep-out")
    assert not fx._intended("TM151 mount 30 mm [print Z+]", "USB keep-out")
    assert not fx._intended("TM151 mount 30 mm [print Z+]", "TM151")
    assert not fx._intended("yaw arm [print Z-]", "yaw X330 horn")
    assert fx.canon("TM151 mount 30 mm [print Z+]") == "TM151 mount"


def test_both_studios_generate_and_lint(data):
    text = fx.build_fs(data)
    sm.lint_fs(text)
    assert fx.SPLIT_MARK in text
    assert not re.search(r"%[A-Z_0-9]+%", text)      # every placeholder filled
    # the copied servo-mount layer comes through whole, banner dropped
    assert "export function caseShellBuild" in text and "horn_features" not in text
    table = sm.servo_table(sm.load_params(sm.CAD_PARAMS))
    mount = sm.build_fs(table)
    sm.lint_fs(mount)
    assert "x330Idler" in mount and "screwAndNut632" in mount
    # the full-wrap cover's end-wall edge is the two cones, not the old V
    assert "coneP" in mount and "vCut" not in mount


def test_one_check_call_builds_every_mount(data):
    """All three mounts in ONE eval, each deleted before the next, with the
    hanging-edge report in it."""
    w = fx.check_wrapper(fx.build_fs(data), data, list(fx.CONFIGS))
    sm.lint_fs(w)
    assert all(f'"{c}"' in w for c in fx.CONFIGS)
    assert w.count("ahrsFixtureBuild(context, id,") == 1
    assert "opDeleteBodies(context, id + \"clear\"" in w and "HANG|" in w


@pytest.mark.parametrize("src", ["const box = 1;", "function f(case is Query) {}"])
def test_lint_refuses_a_reserved_word(src):
    with pytest.raises(SystemExit, match="reserved word"):
        sm.lint_fs(src)
