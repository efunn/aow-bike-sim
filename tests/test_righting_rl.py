"""The RL get-up: control/righting_spec.py and control/righting_env.py.

Pinned: the spec is a pure mirror (lying left reads as lying right, and the
action comes back on the left); Goal Current spans 0..current_max; an
exported npz replays with its OWN bounds; the env's starting poses lie on
the side they claim; a limp crank never gets up; and the scripted
120 deg/s full-stroke ramp props at servo strength 0.7 and goes over at
1.0 -- the two ends the eval grid is read against (train_righting_rl
EVAL_STRENGTHS). Whether a policy gets up is the training run's question.
"""
import copy

import numpy as np
import pytest

from aow_sim.control.righting_spec import (OBS_DIM, RightingBounds,
                                           RightingPolicy, build_obs,
                                           scale_action)

B = RightingBounds(goal_mid=1.5, goal_span=1.0, current_max=785, diff_max=10.0)


@pytest.mark.pure
def test_obs_is_mirrored_onto_the_lying_side():
    right = build_obs(1, 1.2, -0.3, -1.1, 0.4, -0.9, 0.2, 0.5, 0.1, 0.25)
    left = build_obs(-1, -1.2, 0.3, 1.1, -0.4, 0.9, -0.2, 0.5, 0.1, 0.25)
    assert right.shape == (OBS_DIM,)
    np.testing.assert_array_equal(right, left)
    assert right[4] == pytest.approx(0.2)          # lead = goal - crank


@pytest.mark.pure
def test_action_comes_back_on_the_lying_side():
    a = np.array([0.5, 0.2, -0.3])
    r, l = scale_action(a, B, 1), scale_action(a, B, -1)
    assert r[0] == pytest.approx(2.0) and l[0] == pytest.approx(-2.0)
    assert scale_action([-1, 0, 0], B, 1)[0] == pytest.approx(0.5)
    assert scale_action([0, 0, 0], B, -1)[0] == pytest.approx(-1.5)
    assert r[2] == pytest.approx(-3.0) and l[2] == pytest.approx(3.0)
    assert r[1] == l[1]
    assert scale_action([0, -1, 0], B, 1)[1] == 0
    assert scale_action([0, -0.5, 0], B, 1)[1] == 392
    assert scale_action([0, 0, 0], B, 1)[1] == 785          # a >= 0: full
    assert scale_action([0, 1, 0], B, 1)[1] == 785
    assert scale_action([0, 5, 0], B, 1)[1] == 785   # clipped


@pytest.mark.pure
def test_exported_npz_replays_with_its_own_bounds(tmp_path):
    from aow_sim.control.policy import save_policy_npz
    rng = np.random.default_rng(0)
    layers = [(rng.standard_normal((8, OBS_DIM)), rng.standard_normal(8)),
              (rng.standard_normal((3, 8)), rng.standard_normal(3))]
    p = tmp_path / "r.npz"
    save_policy_npz(p, layers, "tanh", np.zeros(OBS_DIM), np.ones(OBS_DIM), B)
    pol = RightingPolicy(p)
    assert pol.bounds == B
    obs = rng.standard_normal(OBS_DIM)
    raw = np.clip(pol.net.mean(obs), -1, 1)
    assert pol.action(obs, -1) == scale_action(raw, B, -1)


def _env(strength=None):
    from aow_sim.control.righting_env import RightingEnv, _load_rl_config
    cfg = copy.deepcopy(_load_rl_config())
    cfg["env"]["start"]["n_per_side"] = 2
    if strength is not None:
        cfg["randomization"].update(servo_strength=[strength, strength],
                                    supply=[1.0, 1.0], mass_frac=0.0,
                                    friction_frac=0.0)
    return RightingEnv(rl_cfg=cfg, seed=0)


def _fly(env, act, side):
    obs, _ = env.reset(seed=0, options={"side": side, "start": "rest"})
    while True:
        obs, _r, te, tr, info = env.step(act)
        if te or tr:
            return obs, info


@pytest.mark.righting
def test_starts_lie_on_their_side_and_a_limp_crank_stays_down():
    env = _env()
    for kind in ("rest", "falling"):
        assert {p[0] for p in env._pool[kind]} == {1, -1}
    for side in (1, -1):
        obs, _ = env.reset(seed=1, options={"side": side, "start": "falling"})
        assert obs[0] > np.deg2rad(55)              # past the takeover, falling
        obs, _ = env.reset(seed=1, options={"side": side, "start": "rest"})
        assert obs[0] > np.deg2rad(50)              # side frame: lying = +ve
        obs, info = _fly(env, np.array([0.0, -1.0, 0.0]), side)
        assert not info["success"] and obs[0] > np.deg2rad(50)


@pytest.mark.righting
@pytest.mark.parametrize("strength, outcome", [(0.7, "success"), (1.0, "over")])
def test_the_scripted_ramp_props_weak_and_goes_over_strong(strength, outcome):
    env = _env(strength)
    from aow_sim.control.righting_env import scripted_ramp
    ramp = scripted_ramp(env.cfg)
    for side in (1, -1):
        obs, _ = env.reset(seed=0, options={"side": side, "start": "rest"})
        while True:
            obs, _r, te, tr, info = env.step(None, command=ramp(obs, side))
            if te or tr:
                break
        assert info[outcome], (side, info)
