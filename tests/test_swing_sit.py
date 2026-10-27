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
that never gets there lets go, and tries again after `retry_s` unless the
lie-down was the operator's; and the fall detector needs the bike down
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


def test_a_failed_lift_tries_again_and_a_chosen_lie_down_does_not():
    """After a lift times out it lies for `retry_s`, then lifts again -- on
    the side it is lying on THEN. A lie-down the operator asked for stays
    down. retry_s None is the old behaviour: wait for the operator."""
    seq = SwingSit(DEPLOY)
    seq.get_up(0.0, roll=np.deg2rad(-80))
    t, phases, cmds = run(seq, seq.lift_timeout_s + seq.limp_s + seq.retry_s + 0.1,
                          0.0, roll=np.deg2rad(70))
    assert phases == ["lift", "limp", "lying", "lift"]
    assert seq.side == 1.0                   # it rolled over: lift the other side
    assert cmds[-1].crank > 0.0 and not cmds[-1].policy

    seq = SwingSit(DEPLOY, retry_s=None)
    seq.get_up(0.0, roll=np.deg2rad(-80))
    _, phases, _ = run(seq, seq.lift_timeout_s + seq.limp_s + 3.0, 0.0,
                       roll=np.deg2rad(-70))
    assert phases == ["lift", "limp", "lying"]

    seq = SwingSit(DEPLOY)
    seq.lie_down(0.0, roll=0.0)
    seq.phase = "lying"                      # a finished, chosen lie-down
    _, phases, _ = run(seq, 3.0, 0.0, roll=np.deg2rad(-75))
    assert phases == ["lying"]


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


# --- up without the hand-off (hand_off=False, the real bike's default) ------

def test_without_the_hand_off_the_lift_stops_at_the_lean_and_props():
    """The lift ramps to `lean_deg`, not the full stroke, and ends PROPPED --
    drives limp, blade held -- instead of handing to the policy, however
    inside the recoverable set the bike is."""
    seq = SwingSit(DEPLOY, hand_off=False)
    seq.get_up(0.0, np.deg2rad(80.0))
    crank, t = 0.0, 0.0
    for _ in range(int(3.0 / DT)):
        c = seq.step(t, np.deg2rad(5.0), 0.0, crank)
        assert not c.policy and abs(c.crank) <= np.deg2rad(110.0) + 1e-9
        crank, t = c.crank, t + DT
        if seq.phase == "propped":
            break
    assert seq.phase == "propped" and not seq.active
    c = seq.step(t + 10.0, np.deg2rad(5.0), 0.0, crank)
    assert c.limp and not c.policy and c.crank == pytest.approx(np.deg2rad(110.0))
    assert seq.phase == "propped", "it waits, for as long as it takes"


def test_engage_from_propped_eases_the_blade_and_folds_it():
    seq = SwingSit(DEPLOY, hand_off=False)
    seq.get_up(0.0, np.deg2rad(-80.0))
    t, crank = 0.0, 0.0
    while seq.phase != "propped":
        crank = seq.step(t, np.deg2rad(-5.0), 0.0, crank).crank
        t += DT
    seq.engage(t)
    assert seq.phase == "balance"
    c = seq.step(t + DT, 0.0, 0.0, crank)
    assert c.policy and c.crank == pytest.approx(-np.deg2rad(110.0), abs=1e-3)
    phases = set()
    for _ in range(int(6.0 / DT)):
        t += DT
        crank = seq.step(t, 0.0, 0.0, crank).crank
        phases.add(seq.phase)
        if seq.phase == "riding":
            break
    assert {"balance", "retract", "riding"} <= phases and crank == pytest.approx(0.0)


def test_engage_is_only_from_propped():
    seq = SwingSit(DEPLOY, hand_off=False)
    with pytest.raises(RuntimeError, match="propped"):
        seq.engage(0.0)


def test_rest_deg_carries_the_lower_through_centre_and_limp_holds_it_there():
    seq = SwingSit(DEPLOY, rest_deg=75.0)
    seq.lie_from_prop(0.0, np.deg2rad(-110.0))           # left side
    t, crank, seen = 0.0, np.deg2rad(-110.0), []
    while seq.phase in ("lower", "limp"):
        c = seq.step(t, np.deg2rad(-60.0), 0.0, crank)
        crank, t = c.crank, t + DT
        seen.append(c.crank)
    assert seq.phase == "lying"
    assert max(seen) == pytest.approx(np.deg2rad(75.0)), "past centre, the other way"
    assert np.all(np.diff(seen) >= -1e-12), "one way only, at the ramp"
    assert seen[-1] == pytest.approx(np.deg2rad(75.0)), "limp holds the rest crank"


def test_a_lift_that_stalls_outside_the_window_props_rather_than_dropping():
    """The bike's left side stalls at a 15-17 deg prop (runs 261026-163044,
    -163421). Timing out there let go and dropped it at up to 614 deg/s;
    now a still bike on the blade is propped, and Enter's gate decides."""
    seq = SwingSit(DEPLOY, hand_off=False)
    seq.get_up(0.0, np.deg2rad(-80.0))
    crank, t = 0.0, 0.0
    while seq.phase == "lift" and t < seq.lift_timeout_s + 1.0:
        crank = seq.step(t, np.deg2rad(-15.5), 0.0, crank).crank
        t += DT
    assert seq.phase == "propped" and t < seq.lift_timeout_s
    seq = SwingSit(DEPLOY, hand_off=False)                # still lying: back down
    seq.get_up(0.0, np.deg2rad(-80.0))
    crank, t = 0.0, 0.0
    while seq.phase == "lift":
        crank = seq.step(t, np.deg2rad(-75.0), 0.0, crank).crank
        t += DT
    assert seq.phase == "lower" and seq.failed == "timeout"
    assert seq.step(t, np.deg2rad(-75.0), 0.0, crank).crank == pytest.approx(crank, abs=1e-3), \
        "from where the crank is, not a step"


MAP = ((110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0, -10),
       (8.0, 10.7, 17.3, 23.8, 29.4, 34.4, 39.2, 44.0, 48.9, 54.1, 61.9,
        69.2, 73.4))


@pytest.mark.parametrize("side", [1.0, -1.0])
def test_a_lower_with_a_map_ramps_the_roll_not_the_crank(side):
    """Along the map's roll the rate is capped and starts and ends at zero;
    the crank falls one way only, from where it was, to the rest."""
    seq = SwingSit(DEPLOY, rest_deg=25.0, lower_dps=120.0, lower_map=MAP,
                   lower_roll_dps=60.0, lower_roll_acc=240.0)
    seq.lie_from_prop(0.0, side * np.deg2rad(102.0))
    u, t = [], 0.0
    while seq.phase == "lower":
        u.append(side * seq.step(t, 0.0, 0.0, 0.0).crank)
        t += DT
    u = np.degrees(u)
    assert u[0] == pytest.approx(102.0) and u[-1] == pytest.approx(-25.0)
    assert np.all(np.diff(u) <= 1e-9)
    on = u > MAP[0][-1]
    w = np.interp(-u[on], -np.array(MAP[0], float), MAP[1])
    rate = np.diff(w) / DT
    assert rate.max() <= 60.0 + 1e-6 and rate[:5].max() < 5.0 and rate[-5:].max() < 5.0
    assert 1.3 < t < 1.6


def test_a_blocked_lift_lowers_back_before_it_can_overload():
    """Run 261026-173521: the crank stopped at -64 against a wall while its
    ramp ran to -110, was held there at 700 mA, and tripped overload 1.2 s
    later. A crank behind its ramp that stops moving is blocked: the lift
    comes back down from where it is. Never propped."""
    seq = SwingSit(DEPLOY, hand_off=False, lift_dps=120.0, retry_s=None)
    seq.get_up(0.0, np.deg2rad(-70.0))
    wall, t, crank = np.deg2rad(-64.0), 0.0, 0.0
    while seq.phase == "lift":
        goal = seq.step(t, np.deg2rad(-34.0), 0.0, crank).crank
        crank = max(goal, wall)                       # lag-free, until the wall
        t += DT
    assert seq.phase == "lower" and seq.failed == "blocked"
    # well inside the 1.5 s the wall stall took to trip overload
    assert t < (64.0 + np.degrees(seq.block)) / 120.0 + seq.block_s + 0.05 and t < 1.3
    while seq.phase in ("lower", "limp"):
        crank = seq.step(t, np.deg2rad(-50.0), 0.0, crank).crank
        t += DT
    assert seq.phase == "lying"
    seq.step(t + 5.0, np.deg2rad(-73.0), 0.0, crank)
    assert seq.phase == "lying", "no retry: the operator presses again"


def test_a_stall_far_short_of_the_lean_is_not_a_prop():
    """Still and below the fallen roll, but 46 deg short: not on the blade."""
    seq = SwingSit(DEPLOY, hand_off=False, block_deg=90.0)  # isolate the prop rule
    seq.get_up(0.0, np.deg2rad(-70.0))
    t, crank = 0.0, 0.0
    while seq.phase == "lift":
        crank = max(seq.step(t, np.deg2rad(-34.0), 0.0, crank).crank, np.deg2rad(-64.0))
        t += DT
    assert seq.phase == "lower" and seq.failed == "timeout"


def test_a_slow_lift_is_not_a_blocked_one():
    """Run 261026-175020, battery on: the crank near its current cap lags
    35-58 deg behind the ramp and still lifts the bike. Moving, it is not
    blocked, however far behind."""
    seq = SwingSit(DEPLOY, hand_off=False, lift_dps=120.0, retry_s=None)
    seq.get_up(0.0, np.deg2rad(-74.0))
    t, crank = 0.0, np.deg2rad(25.0)
    while seq.phase == "lift" and t < 2.5:
        seq.step(t, np.deg2rad(-60.0), 0.0, crank)
        crank -= np.deg2rad(60.0) * DT                # half the ramp's speed
        t += DT
    assert seq.phase == "lift" and seq.failed is None


def test_a_bike_thrown_over_the_top_is_not_propped():
    """In the sim a 240 deg/s lift at full current threw the bike over to
    -50 deg on the far side, and the stall rule called that a prop. A prop
    leans onto the blade that is down."""
    seq = SwingSit(DEPLOY, hand_off=False)
    seq.get_up(0.0, np.deg2rad(-74.0))                    # left blade
    crank, t = 0.0, 0.0
    while seq.phase == "lift" and t < seq.lift_timeout_s + 0.5:
        crank = seq.step(t, np.deg2rad(+50.0), 0.0, crank).crank
        t += DT
    assert seq.phase != "propped"


def test_a_stalled_crank_is_not_blocked_while_the_bike_still_rolls_up():
    """Run 261026-184729: the crank stopped dead and the bike coasted on up
    8-10 deg for the whole 0.3 s. Blocked counts from when it stops."""
    seq = SwingSit(DEPLOY, hand_off=False, lift_dps=120.0, retry_s=None)
    seq.get_up(0.0, np.deg2rad(74.0))                     # right blade
    t, crank, stuck_at = 0.0, 0.0, None
    while seq.phase == "lift" and t < 3.0:
        if stuck_at is None and crank >= np.deg2rad(20.0):
            stuck_at = t
        up = 0.0 if stuck_at is None else min(t - stuck_at, 0.5)   # rolls up 0.5 s
        c = seq.step(t, np.deg2rad(60.0 - 40.0 * up),
                     -np.deg2rad(40.0) if 0 < up < 0.5 else 0.0, crank)
        crank = min(c.crank, np.deg2rad(20.0))
        t += DT
    assert seq.failed == "blocked"
    # restarts on each new best degree, so up to ~1 deg of climb early
    assert t - stuck_at >= 0.5 + seq.block_s - 0.05, "not while it still rolled"


def test_a_bike_rocking_on_the_wing_is_blocked_but_one_still_gaining_is_not():
    """Run 261026-191739: crank dead at 9.7 deg, the bike rocking on the wing
    between 62 and 76 deg at ~1.25 Hz. Each upswing is not progress; only a
    new best roll restarts the wait."""
    def fly(roll_of_t):
        seq = SwingSit(DEPLOY, hand_off=False, lift_dps=120.0, retry_s=None)
        seq.get_up(0.0, np.deg2rad(74.0))
        t, crank, stuck = 0.0, 0.0, None
        while seq.phase == "lift" and t < 3.0:
            if stuck is None and crank >= np.deg2rad(10.0):
                stuck = t
            w = 72.0 if stuck is None else roll_of_t(t - stuck)
            dw = (roll_of_t(t - stuck + DT) - w) / DT if stuck is not None else 0.0
            crank = min(seq.step(t, np.deg2rad(w), np.deg2rad(dw), crank).crank,
                        np.deg2rad(10.0))
            t += DT
        return seq, t - stuck
    rock = lambda u: 69.0 + 7.0 * np.cos(2 * np.pi * 1.25 * u)    # 62..76, no gain
    seq, after = fly(rock)
    assert seq.failed == "blocked" and after < 1.2, "rocking in place: given up"
    climb = lambda u: 72.0 - 10.0 * u                               # still gaining
    seq, after = fly(climb)
    assert seq.failed != "blocked" or after > 2.0


def _pump(seq, crank_of, roll_of, rate_of, T=8.0):
    """Fly a left lift; the crank, roll and rate are functions of (t, cmd).
    -> [(t, phase, cmd)]."""
    seq.get_up(0.0, np.deg2rad(-74.0))
    t, crank, out, cmd = 0.0, np.deg2rad(25.0), [], None
    while seq.phase in ("lift", "pump") and t < T:
        cmd = seq.step(t, np.deg2rad(roll_of(t)), np.deg2rad(rate_of(t)), crank)
        out.append((t, seq.phase, cmd))
        crank = crank_of(t, cmd, crank)
        t += DT
    return out


def test_a_blocked_lift_pumps_with_the_rock_then_lowers_when_its_time_is_up():
    """Run 261026-191739: crank dead at 9.7, the bike rocking on the wing at
    ~1.25 Hz. Push while it swings up (rate toward upright), ease off while
    it swings back; give up after `pump_s` in all."""
    seq = SwingSit(DEPLOY, hand_off=False, lift_dps=120.0, retry_s=None, pump_s=2.0)
    wall = np.deg2rad(-10.0)
    rock = lambda t: 69.0 + 7.0 * np.cos(2 * np.pi * 1.25 * t)
    rate = lambda t: -7.0 * 2 * np.pi * 1.25 * np.sin(2 * np.pi * 1.25 * t)
    out = _pump(seq, lambda t, c, k: max(c.crank, wall), lambda t: -rock(t), lambda t: -rate(t))
    pumped = [(t, c) for t, ph, c in out if ph == "pump"]
    assert pumped and seq.phase == "lower" and seq.failed == "blocked"
    assert 1.9 < pumped[-1][0] - pumped[0][0] < 2.1, "its time budget"
    for t, c in pumped[5:]:
        up = rate(t) < -5.0 * 1.0                     # swinging toward upright (side frame)
        down = rate(t) > 5.0
        if up:
            assert c.current == 1.0 and c.crank == pytest.approx(np.deg2rad(-110.0))
        elif down:
            assert c.current == seq.pump_ease


def test_a_pump_that_gets_the_crank_up_hands_to_the_lift_and_props():
    seq = SwingSit(DEPLOY, hand_off=False, lift_dps=120.0, retry_s=None, pump_s=5.0)
    state = {"free": None}
    def crank_of(t, c, k):
        if seq.phase == "pump" and state["free"] is None:
            state["free"] = t + 1.0                    # gets through after a second
        if state["free"] is not None and t > state["free"]:
            return k + np.sign(c.crank - k) * min(abs(c.crank - k), np.deg2rad(100.0) * DT)
        return max(c.crank, np.deg2rad(-10.0))
    roll_of = lambda t: -10.0 if state["free"] and t > state["free"] + 1.0 else -65.0
    out = _pump(seq, crank_of, roll_of, lambda t: 0.0)
    phases = [ph for _, ph, _ in out]
    assert "pump" in phases and seq.phase == "propped"
