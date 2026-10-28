"""The finishing pass (cad.finish): hand features replayed by the generator,
checked without Onshape.

`cad.bike_v2 --check` and `cad.righting --check` replay them in Onshape and
compare every body's volume with the hand model's; these check what the
replay data promises locally -- every pick has a point, every region pick
names a sketch drawn before it, the rollcage rear carries each front edit,
the sketch frame and the twins' names, and that both studios carry it.
Invalidated by: config/cad/finish/*.yaml (re-baked by `python -m
aow_sim.cad.finish`), cad.finish, cad.hand_edits --picks.
"""

import math

import pytest

from aow_sim.cad import bike_v2 as b2
from aow_sim.cad import finish as cf
from aow_sim.cad import righting as cr

pytestmark = pytest.mark.cad

TABS = ("bike_v2", "righting")


def _queries(op):
    return [q for v in op["params"].values() if isinstance(v, dict) and "q" in v for q in v["q"]]


@pytest.mark.parametrize("tab", TABS)
def test_every_op_is_a_feature_it_can_replay_with_every_pick_placed(tab):
    ops = cf.load(tab)
    assert ops, f"config/cad/finish/{tab}.yaml is missing or empty"
    drawn = set()
    for op in ops:
        if op["op"] == "sketch":
            drawn.add(op["name"])
            assert math.isclose(sum(v * v for v in op["normal"]), 1, abs_tol=1e-4)
            continue
        assert op["op"] in cf.FEATURES
        qs = _queries(op)
        assert qs, f"{op['name']} picks nothing"
        for q in qs:
            assert len(q["p"]) == 3
            assert q["k"] in "EFBR"
            if q["k"] == "R":
                assert q["s"] in drawn, f"{op['name']} extrudes a sketch not drawn before it"
            assert not q.get("m"), f"{op['name']}: an unresolved mirror candidate baked in"


def test_the_rollcage_rear_gets_every_front_edit_turned():
    """The rear ring is the front one's print: each op on the front comes
    again on the rear, its points turned 180 deg about Z through
    mid-wheelbase."""
    wb = b2.layout(b2.load())["wb"]
    ops = {o["name"]: o for o in cf.load("bike_v2")}
    front = [o for o in ops.values() if o["op"] != "sketch" and not o["name"].endswith("(rear)")
             and all(q["k"] == "R" or q.get("nm") == "rollcage front" for q in _queries(o))]
    assert len(front) >= 10
    for o in front:
        rear = ops[o["name"] + " (rear)"]
        for a, b in zip(_queries(o), _queries(rear)):
            assert b["p"] == pytest.approx([-a["p"][0], wb - a["p"][1], a["p"][2]], abs=1e-4)


def test_a_sketch_frame_is_the_world_origin_projected_onto_its_plane():
    # Sketch 1's plane as read back: a point on the Pi carrier's face, its normal and x
    n = [0, -math.sqrt(0.5), -math.sqrt(0.5)]
    sk = cf._sketch({"entities": [], "name": "s"}, {"origin": [0, 112.28856, 101.25769], "normal": n, "x": [1, 0, 0]})
    # Sketch 1's corner (-28, 14.7) is the Pi carrier cut's face corner, read back
    assert cf._from_local(sk, [-28, 14.7]) == pytest.approx([-28, 96.38, 117.17], abs=0.01)
    assert cf._to_local(sk, cf._from_local(sk, [3.0, -4.0])) == pytest.approx([3.0, -4.0])


def test_the_twins():
    assert cf.twin("case side L [print X+]") == "case side R [print X-]"
    assert cf.twin("wing R stub (knuckle)") == "wing L stub (rocker)"
    assert cf.twin("rocker R") == "rocker L"
    assert cf.twin("lower case [print Z+]") == "lower case [print Z+]"


def test_both_studios_replay_their_edits():
    """The righting's layer carries the interpreter and RG_FIN, called at
    rest; v2 carries BK2_FIN and turns the steer and the pose on after it."""
    rd = cr.load()
    fs = cr.build_fs(cr.layout(rd), rd)
    assert "export function finishOps" in fs and "export const RG_FIN =" in fs
    v2 = b2.build_fs(b2.layout(b2.load()))
    assert v2.count("export function finishOps") == 1
    body = v2.split("export function bikeV2Build")[1]
    assert body.index('step("finish")') < body.index('step("pose")')
    assert '"finish" : true' in body
