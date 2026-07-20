"""RL flick tests. The spec + numpy-policy tests are dependency-free and always
run; the env test skips without gymnasium; the replay test skips without a
trained moves/flick_rl artifact (like the trajopt flick-replay test)."""

import tempfile
from pathlib import Path

import numpy as np
import pytest

from aow_sim.control.flick_spec import (ACT_DIM, OBS_DIM, ActionBounds, build_obs,
                                        scale_action)
from aow_sim.control.policy import load_policy_npz, save_policy_npz


def test_obs_and_action_spec():
    obs = build_obs(0.01, 0.1, np.pi / 2, 0.2, 3.0, 0.4, -0.1, 0.05, 0.5)
    assert obs.shape == (OBS_DIM,)
    assert np.all(np.isfinite(obs))
    assert abs(obs[5]) <= np.pi + 1e-6          # steer wrapped
    b = ActionBounds(8.0, 0.6, 30.0)
    sr, hub, diff = scale_action([2.0, -2.0, 0.5], b)   # clips to [-1,1]
    assert (sr, hub, diff) == pytest.approx((8.0, -0.6, 15.0))
    # 2-action (feedforward) form returns diff 0
    assert scale_action([0.5, 0.5], b)[2] == 0.0


def test_numpy_policy_forward_roundtrips():
    rng = np.random.default_rng(0)
    W0, b0 = rng.standard_normal((16, OBS_DIM)) * 0.1, np.zeros(16)
    W1, b1 = rng.standard_normal((ACT_DIM, 16)) * 0.1, np.zeros(ACT_DIM)
    mean, var = np.full(OBS_DIM, 1.0), np.full(OBS_DIM, 4.0)
    bounds = ActionBounds(8.0, 0.6, 30.0)
    obs = build_obs(0.02, -0.1, 1.0, 0.3, 2.0, 0.1, -0.2, 0.03, 0.4)
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "pol.npz"
        save_policy_npz(p, [(W0, b0), (W1, b1)], "tanh", mean, var, bounds)
        pol = load_policy_npz(p)
    x = np.clip((obs - mean) / np.sqrt(var + 1e-8), -10, 10)
    expect = scale_action(np.clip(W1 @ np.tanh(W0 @ x + b0) + b1, -1, 1), bounds)
    assert pol.action(obs) == pytest.approx(expect, abs=1e-6)
    assert pol.act_dim == ACT_DIM


def test_env_reset_step():
    gym = pytest.importorskip("gymnasium")   # skip if [rl] not installed
    from aow_sim.control.flick_env import FlickEnv
    env = FlickEnv(seed=0)
    assert env.observation_space.shape == (OBS_DIM,)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (OBS_DIM,) and np.all(np.isfinite(obs))
    rng = np.random.default_rng(1)
    term = trunc = False
    steps = 0
    while not (term or trunc) and steps < env.max_steps + 1:
        obs, r, term, trunc, info = env.step(rng.uniform(-1, 1, env.action_space.shape))
        assert np.all(np.isfinite(obs)) and np.isfinite(r)
        steps += 1
    assert term or trunc      # episodes end (fall / success / timeout)
    # determinism
    o1, _ = FlickEnv(seed=0).reset(seed=5)
    o2, _ = FlickEnv(seed=0).reset(seed=5)
    assert np.allclose(o1, o2)


def test_rl_move_replays():
    """If a trained RL policy exists, it loads and replays without blowing up."""
    from aow_sim.control.flick import MOVES_DIR
    if not (MOVES_DIR / "flick_rl.npz").exists():
        pytest.skip("run `python -m aow_sim.train_flick_rl` to produce moves/flick_rl")
    import mujoco
    from aow_sim.build_model import build_model, load_params
    from aow_sim.control import DriveController, run
    from aow_sim.control.linearize import settle_upright
    p = load_params()
    m = build_model(p, variant="full")
    eq = settle_upright(m)
    c = DriveController(p, m)
    d = mujoco.MjData(m)
    d.qpos[:] = eq.qpos
    a = np.deg2rad(0.5)
    d.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(m, d)
    c.reset(m, d)
    run(m, d, c, 1.0)
    T = c.command_flick(d, +1, name="flick_rl")
    run(m, d, c, T + 2.0)
    assert np.all(np.isfinite(d.qpos))
