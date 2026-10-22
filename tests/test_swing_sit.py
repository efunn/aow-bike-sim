"""The lie-down / get-up sequence on the swing module: control/swing_sit.py.

The state machine only, against hand-fed roll, rate and crank -- no model.
Whether the sequence is GENTLE, and whether it gets the bike up, is the
flown question: analysis/moves/swing_sit.py, and test_teleop's
test_swing_lie_down_get_up_and_auto_up for the teleop wiring.

Pinned: nothing moves until the bike has stopped; the down phases run in
order and end limp with the policy off; the kick leans the bike ONTO the
chosen blade (diff sign against balance.lat_gain) and is held until it
does; a bike going over the OTHER way is handed back to the policy; the
lift hands to the policy only inside recovery.ready_for_policy; the blade
eases off before it folds, and folds only with the bike upright; a lift
that never gets there lets go; and the fall detector needs the bike down
AND nearly still for its whole dwell.
"""
import numpy as np
import pytest

from aow_sim.control.recovery import RECOVER_DEG
from aow_sim.control.swing_sit import SwingSit

pytestmark = pytest.mark.pure

DEPLOY = np.deg2rad(116.9)
DT = 0.001


def run(seq, seconds, t, roll=0.0, rate=0.0, crank=0.0, speed=0.0):
    """Step `seq` for `seconds` at fixed inputs; return (t, phases, cmds).
    `roll` may be a function of the phase, for the closed-loop kick."""
    phases, cmds = [], []
    for _ in range(int(round(seconds / DT))):
        r = roll(seq.phase) if callable(roll) else roll
        cmds.append(seq.step(t, r, rate, crank, speed))
        if not phases or phases[-1] != seq.phase:
            phases.append(seq.phase)
        t += DT
    return t, phases, cmds


def test_nothing_moves_until_the_bike_has_stopped():
    seq = SwingSit(DEPLOY)
    seq.lie_down(0.0, side=1.0)
    t, phases, cmds = run(seq, 1.0, 0.0, speed=0.5, crank=0.1)
    assert phases == ["stop"] and all(c.policy and c.crank == 0.1 for c in cmds)
    t, phases, _ = run(seq, 0.01, t, speed=0.0)
    assert phases[-1] == "deploy"


@pytest.mark.parametrize("side", [+1.0, -1.0])
def test_down_runs_in_order_and_ends_limp(side):
    seq = SwingSit(DEPLOY)
    seq.lie_down(0.0, side=side)
    # The bike leans onto the blade only once kicked.
    lean = lambda ph: side * np.deg2rad(5.0) if ph not in ("stop", "deploy") else 0.0  # noqa: E731
    t, phases, cmds = run(seq, 8.0, 0.0, roll=lean)
    # at rest, "stop" ends on its first step, so the record starts at deploy
    assert phases == ["deploy", "kick", "settle", "lower", "limp", "lying"]
    deploy = [c for c in cmds if c.crank == pytest.approx(side * seq.lean) and c.policy]
    assert deploy, "the blade goes down with the policy still balancing"
    kick = [c for c in cmds if c.diff != 0.0]
    assert kick and all(not c.policy for c in kick)
    # d > 0 moves the rear contact toward -y, leaning the bike toward +y
    # (left, -roll): the kick must lean it the OTHER way from the contact,
    # onto blade `side` (crank +deploy = right blade = +roll).
    assert all(np.sign(c.diff) == -side for c in kick)
    assert cmds[-1].current == 0.0 and not cmds[-1].policy
    assert not seq.active and seq.phase == "lying"


def test_kick_is_held_until_the_bike_leans_onto_the_blade():
    seq = SwingSit(DEPLOY, kick_max_s=5.0)
    seq.lie_down(0.0, side=1.0)
    t, phases, _ = run(seq, seq.deploy_s + 2.0, 0.0)       # level: still kicking
    assert phases[-1] == "kick"
    t, phases, _ = run(seq, 0.01, t, roll=np.deg2rad(4.0))
    assert phases[-1] == "settle"


def test_going_over_the_other_way_hands_back_to_the_policy():
    seq = SwingSit(DEPLOY)
    seq.lie_down(0.0, side=1.0)
    t, phases, _ = run(seq, seq.deploy_s + 0.1, 0.0)
    assert phases[-1] == "kick"
    t, phases, cmds = run(seq, 0.01, t, roll=-np.deg2rad(10.0), crank=seq.lean)
    assert seq.aborted and phases[-1] == "retract" and cmds[-1].policy
    t, phases, cmds = run(seq, 2.0, t, crank=0.0)
    assert phases[-1] == "riding" and cmds[-1].crank == 0.0


def test_lean_side_defaults_to_the_way_it_already_leans():
    seq = SwingSit(DEPLOY)
    seq.lie_down(0.0, roll=-0.02)
    assert seq.side == -1.0
    seq.lie_down(0.0, roll=0.0)
    assert seq.side == +1.0


def test_lift_hands_off_only_inside_the_recoverable_window():
    seq = SwingSit(DEPLOY)
    seq.get_up(0.0, roll=np.deg2rad(80))
    assert seq.side == +1.0
    # far over: keeps lifting, policy off, the goal ramps toward +deploy
    t, phases, cmds = run(seq, 0.5, 0.0, roll=np.deg2rad(60))
    assert phases == ["lift"] and not cmds[-1].policy
    assert 0 < cmds[-1].crank < DEPLOY
    t, phases, cmds = run(seq, 0.6, t, roll=np.deg2rad(60))    # ramp reaches the stroke
    assert cmds[-1].crank == pytest.approx(DEPLOY)
    # inside the angle but moving fast: still lifting
    t, phases, _ = run(seq, 0.01, t, roll=np.deg2rad(RECOVER_DEG - 2), rate=5.0)
    assert phases == ["lift"]
    # inside both: hand-off
    t, phases, cmds = run(seq, 0.01, t, roll=np.deg2rad(RECOVER_DEG - 2))
    assert phases[-1] == "balance" and cmds[-1].policy
    held = cmds[-1].crank
    # leaning on the blade: it eases back to release_deg but does NOT fold
    t, phases, cmds = run(seq, seq.balance_max_s - 0.1, t, roll=np.deg2rad(5.0))
    assert phases == ["balance"]
    assert cmds[-1].crank == pytest.approx(seq.release)
    assert all(0 < c.crank <= held + 1e-12 for c in cmds)
    assert np.all(np.diff([c.crank for c in cmds]) <= 1e-12), "eases, never jumps back"
    # upright: it folds, ramped, and is riding once the crank is home
    t, phases, cmds = run(seq, 0.01, t, roll=0.0)
    assert phases[-1] == "retract"
    t, phases, cmds = run(seq, 2.0, t, crank=0.0)
    assert phases[-1] == "riding" and not seq.active


def test_a_lift_that_never_gets_there_lets_go():
    seq = SwingSit(DEPLOY)
    seq.get_up(0.0, roll=np.deg2rad(-80))
    t, phases, _ = run(seq, seq.lift_timeout_s + seq.limp_s + 0.1, 0.0,
                       roll=np.deg2rad(-70))
    assert phases == ["lift", "limp", "lying"]


def test_fall_detector_needs_down_and_still_for_its_dwell():
    seq = SwingSit(DEPLOY)
    down = np.deg2rad(85)
    assert not seq.fallen(0.0, down, 0.0)
    assert not seq.fallen(seq.fallen_s / 2, down, 0.0)
    # moving fast resets the dwell: a bike on its way over is not lying there
    assert not seq.fallen(seq.fallen_s * 0.9, down, 5.0)
    assert not seq.fallen(seq.fallen_s * 1.5, down, 0.0)
    assert seq.fallen(seq.fallen_s * 2.6, down, 0.0)
    assert not seq.fallen(10.0, np.deg2rad(20), 0.0)


@pytest.mark.parametrize("lands", [+1, -1], ids=["same_side", "other_side"])
def test_a_fall_after_the_hand_off_folds_and_rearms_the_auto_up(lands):
    """The policy can lose the bike after it takes over. The sequence must
    let go and fold, not hold a lying bike in balance for balance_max_s or
    wait in retract on a crank pinned under it -- and the next auto-up must
    wait its own dwell, not fire on the first one's stale timer."""
    seq = SwingSit(DEPLOY)
    assert seq.fallen(0.0, np.deg2rad(85), 0.0) is False
    assert seq.fallen(seq.fallen_s, np.deg2rad(85), 0.0)
    seq.get_up(seq.fallen_s, roll=np.deg2rad(85))
    t, phases, _ = run(seq, 0.01, seq.fallen_s, roll=np.deg2rad(RECOVER_DEG - 2))
    assert phases[-1] == "balance"
    down = lands * np.deg2rad(80)
    pinned = seq.side * DEPLOY if lands > 0 else 0.0   # under the bike, or free
    t, phases, cmds = run(seq, 0.01, t, roll=down, crank=pinned)
    assert phases[0] == "retract"      # (a folded crank finishes it at once)
    t, phases, _ = run(seq, 2.0, t, roll=down, crank=pinned)
    assert phases[-1] == "riding" and not seq.active
    assert not seq.fallen(t, down, 0.0), "fired on the first trigger's timer"
    assert seq.fallen(t + seq.fallen_s, down, 0.0)
