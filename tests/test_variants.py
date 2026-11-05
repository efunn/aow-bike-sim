"""config/variants/: a bike that is bike_params.yaml with a few keys changed.

WHAT INVALIDATES THIS FILE: `params._read_bike` / `_merge_variant` /
`drive_servo`, or a file under config/variants/.

A variant names its base with `variant_of:` and may only change keys the base
holds -- a typo must fail loudly, not load the base bike under the variant's
name. It is a different plant, so it must hash to a different plant_digest
(a policy trained on one warns on the other), and the base must hash where it
did before the mechanism existed.
"""
from pathlib import Path

import pytest

from aow_sim.params import (DEFAULT_PARAMS, drive_servo, load_params,
                            plant_digest)

pytestmark = pytest.mark.pure

VARIANTS = sorted((DEFAULT_PARAMS.parent / "variants").glob("*.yaml"))


def test_there_are_variants():
    assert {v.stem for v in VARIANTS} >= {"mini", "mini_m077"}


@pytest.mark.parametrize("path", VARIANTS, ids=lambda p: p.stem)
def test_variant_loads_as_its_own_plant(path):
    base, var = load_params(), load_params(path)
    assert plant_digest(var) != plant_digest(base)
    # Everything a variant does not name is the base's.
    assert var["bike"]["wheelbase"] == base["bike"]["wheelbase"]
    assert var["servos"]["xc330_t181"] == base["servos"]["xc330_t181"]


def test_base_drive_is_the_xc430():
    """The selector is absent from bike_params.yaml, so the built bike's
    digest did not move when it was added."""
    p = load_params()
    assert "drive_servo" not in p["drivetrain"]
    assert drive_servo(p) is p["servos"]["xc430_w150"]


def test_mini_drives():
    m288 = load_params(DEFAULT_PARAMS.parent / "variants" / "mini.yaml")
    m077 = load_params(DEFAULT_PARAMS.parent / "variants" / "mini_m077.yaml")
    assert drive_servo(m288)["stall_torque"] == 0.60
    assert drive_servo(m077)["stall_torque"] == 0.228
    # The same pulley pair, flipped.
    assert m288["drivetrain"]["belt_ratio"] * m077["drivetrain"]["belt_ratio"] \
        == pytest.approx(1.0, abs=1e-4)
    # Each servo's own factory velocity gains, not the XC430's 100 / 1920:
    # at those the M077's loop chatters (docs/plans/mini-bike.md).
    gains = lambda p: (p["drivetrain_model"]["servo"]["velocity_p_gain"],  # noqa: E731
                       p["drivetrain_model"]["servo"]["velocity_i_gain"])
    assert gains(m288) == (180, 1600)
    assert gains(m077) == (40, 400)
    # A variant of a variant inherits what it does not change.
    assert m077["drivetrain_model"]["servo"]["kp_per_unit"] == \
        m288["drivetrain_model"]["servo"]["kp_per_unit"]
    for p in (m288, m077):
        parts = p["bike"]["frame_parts"]
        assert parts["servo_drive_left"]["servo"] == p["drivetrain"]["drive_servo"]
        assert parts["servo_drive_left"]["geoms"]   # merged, not replaced


def test_a_typo_is_an_error(tmp_path: Path):
    f = tmp_path / "typo.yaml"
    f.write_text(f"variant_of: {DEFAULT_PARAMS}\n"
                 "drivetrain:\n  belt_raito: 2.0\n")
    with pytest.raises(KeyError, match="belt_raito"):
        load_params(f)


def test_a_new_servo_block_is_allowed(tmp_path: Path):
    f = tmp_path / "new.yaml"
    f.write_text(f"variant_of: {DEFAULT_PARAMS}\n"
                 "servos:\n  made_up:\n    stall_torque: 1.0\n")
    assert load_params(f)["servos"]["made_up"]["stall_torque"] == 1.0


def test_replace_owns_the_block(tmp_path: Path):
    """`!replace` swaps a whole block -- here the frame parts, down to one --
    while everything outside it is still inherited."""
    f = tmp_path / "layout.yaml"
    f.write_text(f"variant_of: {DEFAULT_PARAMS}\n"
                 "bike:\n  frame_parts: !replace\n"
                 "    only_part:\n      mass: 0.1\n"
                 "      geoms:\n        - {type: box, pos: [0.1, 0, 0.1], size: [0.01, 0.01, 0.01]}\n")
    p, base = load_params(f), load_params()
    assert list(p["bike"]["frame_parts"]) == ["only_part"]
    assert type(p["bike"]["frame_parts"]) is dict      # the tag does not leak
    assert p["bike"]["payload"] == base["bike"]["payload"]
    assert p["sim"] == base["sim"]


def test_replace_needs_a_block_to_replace(tmp_path: Path):
    f = tmp_path / "typo.yaml"
    f.write_text(f"variant_of: {DEFAULT_PARAMS}\n"
                 "bike:\n  frame_partz: !replace\n    a: {mass: 0.1}\n")
    with pytest.raises(KeyError, match="frame_partz"):
        load_params(f)
