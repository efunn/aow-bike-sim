"""The rollcage as a contact in the stepthrough's rest
(analysis/mechanism/swing_linkage.rest_on_points `fixed=` / `near=`).

Synthetic: a two-point wing and the cage ring, so the answers are known.
Pinned: with the wing tucked, the bike rests on the cage and the wing
carries nothing; with the wing far out, the cage changes nothing; and with
more than one rest available, `near` picks the one closest to the last pose
(the 270 deg ring also offers rests rolled past flat onto its top); and
`pin_forces` handed a rest (the stepthrough's, on blade and cage) agrees
with its own on the panel line's.

WHAT INVALIDATES THIS FILE: rest_on_points' search or regimes, cage_points,
pin_forces, or righting.rollcage's radius / height in bike_params.
"""
import sys

import numpy as np
import pytest

sys.path.insert(0, "analysis/mechanism")
import swing_linkage as sl  # noqa: E402

pytestmark = pytest.mark.analysis

R_WHEEL = 51.0


def test_the_cage_ring_is_the_bikes_and_open_at_the_bottom():
    pts = np.array(list(sl.cage_points().values()))
    r = np.hypot(pts[:, 0], pts[:, 1] - 100.0)
    assert r == pytest.approx(68.7, abs=1e-9)
    # nothing in the 90 deg opening under the axis
    assert not np.any((pts[:, 1] < 100.0 - 68.7 * np.cos(np.radians(45.0)) - 1e-6))


def test_a_tucked_wing_leaves_the_bike_on_the_cage_and_unloaded():
    """A short wing inside the ring: lying over, the cage is lowest."""
    wing = {"a": np.array([-30.0, 60.0]), "b": np.array([-40.0, 90.0])}
    r = sl.rest_on_points(wing, [-20.0, 60.0], R_WHEEL, [-35.0, 75.0],
                          fixed=sl.cage_points())
    assert r is not None and r["regime"] == "cage" and r["load"] == 0.0
    without = sl.rest_on_points(wing, [-20.0, 60.0], R_WHEEL, [-35.0, 75.0])
    assert without["regime"] != "cage"


def test_a_wing_far_out_is_unchanged_by_the_cage():
    wing = {"a": np.array([-150.0, -30.0]), "b": np.array([-160.0, 40.0])}
    args = (wing, [-30.0, 60.0], R_WHEEL, [-150.0, 0.0])
    a = sl.rest_on_points(*args)
    b = sl.rest_on_points(*args, fixed=sl.cage_points())
    assert (a["regime"], a["to_floor"]) == (b["regime"], b["to_floor"])


def test_near_picks_the_rest_closest_to_the_last_pose_or_none():
    wing = {"a": np.array([-30.0, 60.0]), "b": np.array([-40.0, 90.0])}
    args = (wing, [-20.0, 60.0], R_WHEEL, [-35.0, 75.0])
    cage = sl.cage_points()
    lo = sl.rest_on_points(*args, fixed=cage, near=60.0)
    hi = sl.rest_on_points(*args, fixed=cage, near=170.0, near_tol=180.0)
    assert lo is not None and hi is not None
    assert abs(lo["to_floor"][0] - 60.0) <= abs(hi["to_floor"][0] - 60.0)
    assert sl.rest_on_points(*args, fixed=cage, near=-500.0) is None


@pytest.mark.parametrize("travel", [20.0, 60.0, 100.0])
def test_pin_forces_from_a_given_rest_match_the_panel_lines_own(travel):
    """The stepthrough hands `pin_forces` the rest it draws. Given the panel
    line's own rest (its ends as the support points), the answer must be
    the panel line's: the same free bodies, the floor at the same point."""
    import yaml
    from pathlib import Path
    cfg = yaml.safe_load(Path("config/linkage/swing_linkage_smaller.yaml").read_text())
    lk = sl.SwingLinkage(cfg)
    rest = sl.resting_pose(lk, travel)
    assert rest is not None
    own, given = sl.pin_forces(lk, travel), sl.pin_forces(lk, travel, rest=rest)
    for side in ("R", "L"):
        for k in ("coupler", "hinge"):
            assert given[side][k] == pytest.approx(own[side][k], rel=1e-6, abs=1e-6)
    assert given["shaft"] == pytest.approx(own["shaft"], rel=1e-6, abs=1e-6)
