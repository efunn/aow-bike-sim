"""check_move_digest as teleop calls it: the spawn dial must not read as a new plant.

Invalidated by: a change to `check_move_digest`, `plant_digest`, or what
`run_drive.main` adds to params in memory.
"""
from types import SimpleNamespace

import pytest

from aow_sim.build_model import load_params
from aow_sim.control.flick import check_move_digest
from aow_sim.params import plant_digest
from aow_sim.run_drive import FLOOR_TILT_STEPS, without_spawn_dial

pytestmark = pytest.mark.pure


def _teleop_params(params):
    # What run_drive.main does for the spawn dial.
    return {**params, "sim": {**params["sim"],
                              "floor_tilt_steps": FLOOR_TILT_STEPS,
                              "floor_tilt_bearing_deg": 0.0}}


def _move(params):
    return SimpleNamespace(name="today", plant_digest=plant_digest(params))


def test_spawn_dial_does_not_fail_a_current_policy():
    params = load_params()
    teleop = _teleop_params(params)
    # The bug this pins: hashed with the dial in, a current policy warns.
    assert check_move_digest(_move(params), teleop, warn=lambda m: None)
    assert check_move_digest(_move(params), without_spawn_dial(teleop),
                             warn=lambda m: None) == ""


def test_a_real_plant_change_still_warns():
    params = load_params()
    moved = {**params, "bike": {**params["bike"], "wheelbase": 0.25}}
    assert check_move_digest(_move(params),
                             without_spawn_dial(_teleop_params(moved)),
                             warn=lambda m: None)
