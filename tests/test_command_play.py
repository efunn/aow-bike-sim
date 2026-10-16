"""Timed command plays: control/command_play.py, and the files in config/plays.

Pinned: step holds each key until the next and linear ramps between them;
the last key stands after the end; psi is read in DEGREES and returned in
radians; malformed keys are refused; every shipped play loads.
"""
from pathlib import Path

import numpy as np
import pytest

from aow_sim.control.command_play import Play, load_play

pytestmark = pytest.mark.pure

KEYS = [[0.0, 1.0, 0.0, 0.0], [1.0, 1.0, 0.2, 90.0], [2.0, 0.0, 0.0, 90.0]]


def test_step_holds_each_key_and_the_last_one_stands():
    pl = Play.from_keys(KEYS)
    assert pl.at(0.0) == (1.0, 0.0, 0.0)
    assert pl.at(0.999) == (1.0, 0.0, 0.0)
    v, vl, psi = pl.at(1.0)
    assert (v, vl) == (1.0, 0.2) and psi == pytest.approx(np.pi / 2)
    assert pl.at(5.0)[0] == 0.0 and pl.duration == 2.0


def test_linear_ramps_between_keys():
    pl = Play.from_keys(KEYS, interp="linear")
    v, vl, psi = pl.at(0.5)
    assert vl == pytest.approx(0.1) and psi == pytest.approx(np.pi / 4)
    assert pl.at(1.5)[0] == pytest.approx(0.5)


@pytest.mark.parametrize("keys", [
    [[0.5, 1, 0, 0], [1.0, 0, 0, 0]],          # does not start at 0
    [[0.0, 1, 0, 0], [0.0, 0, 0, 0]],          # not increasing
    [[0.0, 1, 0]],                             # three columns
])
def test_malformed_keys_are_refused(keys):
    with pytest.raises(ValueError):
        Play.from_keys(keys)


@pytest.mark.parametrize("path", sorted(
    (Path(__file__).resolve().parents[1] / "config" / "plays").glob("*.yaml")),
    ids=lambda p: p.stem)
def test_shipped_plays_load(path):
    pl = load_play(path)
    assert pl.duration > 0 and pl.name == path.stem


def test_a_fifth_column_drives_the_crank_and_a_ball_is_parked_in_the_bike_frame():
    pl = Play.from_keys([[0.0, 1.0, 0.0, 0.0, 105.0], [1.0, 0.0, 0.0, 0.0, 0.0]],
                        ball=[0.6, -0.1])
    assert pl.crank_at(0.5) == pytest.approx(np.deg2rad(105.0))
    assert pl.crank_at(2.0) == 0.0
    assert Play.from_keys(KEYS).crank_at(0.5) is None, "4 columns: crank untouched"
    # bike at (1, 2) facing +y: "ahead" is +y, "left" is -x
    x, y = pl.ball_world((1.0, 2.0), np.pi / 2)
    assert (x, y) == (pytest.approx(1.1), pytest.approx(2.6))
