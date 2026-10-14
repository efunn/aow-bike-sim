"""RecurrentSmoothPPO / MemoryPolicy / MinGRUPolicy (src/aow_sim/recurrent_ppo.py).

What would make a memory run silently wrong, each checked here:
  - the chunked training pass not reproducing the rollout's own state
    (stored-state bookkeeping, episode-start resets inside a chunk);
  - the numpy replay (the Pi's) not matching the trained net over a SEQUENCE;
  - a save/load dropping the memory (scan and export load with PPO.load).
On a toy env with random-length episodes, so resets land inside chunks. Torch
and SB3 are imported inside the tests -- `pytest -m pure` imports every file.
"""
import numpy as np
import pytest

pytestmark = pytest.mark.trainer


def _toy_env_cls():
    import gymnasium as gym

    class Toy(gym.Env):
        """5-wide obs, 3 actions (the replay's minimum width), 5-20 step
        episodes, a reward that needs the PREVIOUS observation."""
        observation_space = gym.spaces.Box(-np.inf, np.inf, (5,), np.float32)
        action_space = gym.spaces.Box(-1.0, 1.0, (3,), np.float32)

        def reset(self, seed=None, options=None):
            super().reset(seed=seed)
            self._t, self._len = 0, int(self.np_random.integers(5, 21))
            self._o = self.np_random.standard_normal(5).astype(np.float32)
            return self._o, {}

        def step(self, a):
            prev, self._t = self._o, self._t + 1
            self._o = self.np_random.standard_normal(5).astype(np.float32)
            r = -float(np.sum((np.asarray(a) - prev[:3]) ** 2))
            return self._o, r, False, self._t >= self._len, {}

    return Toy


def _model(smooth=None, n_envs=3, n_steps=64, bptt=16, batch=32, seed=1):
    pytest.importorskip("gymnasium")
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    from aow_sim.recurrent_ppo import MemoryPolicy, RecurrentSmoothPPO
    Toy = _toy_env_cls()
    venv = VecNormalize(DummyVecEnv([Toy] * n_envs), norm_obs=True, norm_reward=True)
    return RecurrentSmoothPPO(
        MemoryPolicy, venv, n_steps=n_steps, batch_size=batch, n_epochs=2,
        seed=seed, device="cpu", bptt=bptt, smooth=smooth,
        policy_kwargs=dict(net_arch=[16, 16], memory_hidden=8))


def test_sequence_pass_equals_stepping_with_resets():
    import torch as th
    from aow_sim.recurrent_ppo import MinGRU
    th.manual_seed(0)
    m = MinGRU(6)
    x = th.randn(12, 4, 6)
    starts = (th.rand(12, 4) < 0.2).float()
    h0 = th.rand(4, 6)
    out, hs, _ = m(x, h0, starts)
    h = h0
    for t in range(12):
        o, hh, _ = m(x[t:t + 1], h, starts[t:t + 1])
        assert th.allclose(o[0], out[t], atol=1e-6)
        h = hh[0]
        assert th.allclose(h, hs[t], atol=1e-6)


def test_epoch0_chunks_reproduce_the_rollout_log_probs():
    """Before any update, re-running every chunk from its STORED start state
    must give back the rollout's log-probs: the state bookkeeping and the
    in-chunk episode resets are right, or this is off by far more than
    float rounding."""
    import torch as th
    model = _model()
    _total, cb = model._setup_learn(10_000, None)
    model.collect_rollouts(model.env, cb, model.rollout_buffer, model.n_steps)
    d = model._chunks()
    assert d["starts"][:, 1:].sum() > 0, "no episode start inside a chunk: vacuous"
    with th.no_grad():
        _v, logp, *_ = model.policy.evaluate_seq(
            d["obs"].transpose(0, 1), d["actions"].transpose(0, 1),
            d["h0"], d["starts"].transpose(0, 1))
    err = (logp.transpose(0, 1) - d["old_log_prob"]).abs().max().item()
    assert err < 1e-5, err   # 5e-7 measured; a zeroed h0 or ignored resets: 5e-3+


def test_numpy_replay_matches_the_trained_net(tmp_path):
    """_export + _verify_export, the trainer's own path: a 64-step sequence
    from an empty memory, numpy against torch."""
    from aow_sim.control.policy import MinGRUPolicy, load_policy_npz
    from aow_sim.train_general_rl import _export, _verify_export
    model = _model(smooth={"bound": 0.1, "temporal": 0.04})
    model.learn(model.n_steps * model.n_envs)
    vn = model.get_vec_normalize_env()
    cfg = {"env": {"action_bounds": {"steer_rate_max": 2.0, "hub_max": 0.5,
                                     "diff_max": 3.0}}}
    npz = tmp_path / "p.npz"
    _export(model, vn, cfg, npz)
    pol = load_policy_npz(npz)
    assert isinstance(pol, MinGRUPolicy)
    assert _verify_export(model, vn, npz) < 1e-5

    # reset() really empties it: replay after a reset == a fresh load
    rng = np.random.default_rng(1)
    seq = rng.standard_normal((10, 5))
    first = [pol.mean(o) for o in seq]
    pol.reset()
    again = [pol.mean(o) for o in seq]
    assert np.allclose(first, again)
    # ...and the memory matters: the same obs after different histories differ
    pol.reset()
    pol.mean(seq[0])
    a = pol.mean(seq[5])
    pol.reset()
    b = pol.mean(seq[5])
    assert not np.allclose(a, b)

    # peek is a what-if: it answers like action() but leaves the memory alone
    pol.reset()
    pol.action(seq[0])
    h = pol.h.copy()
    assert np.allclose(pol.peek(seq[1]), pol.action(seq[1]))
    pol.h = h
    pol.peek(seq[2])
    assert np.array_equal(pol.h, h)


def test_save_load_keeps_the_memory(tmp_path):
    """Scan, best-model export and --export-from all load with PPO.load."""
    from stable_baselines3 import PPO
    from aow_sim.recurrent_ppo import MemoryPolicy, sb3_actor
    model = _model()
    model.learn(model.n_steps * model.n_envs)
    model.save(tmp_path / "m")
    back = PPO.load(tmp_path / "m", device="cpu")
    assert isinstance(back.policy, MemoryPolicy)
    rng = np.random.default_rng(2)
    a1, a2 = sb3_actor(model, None), sb3_actor(back, None)
    for _ in range(20):
        o = rng.standard_normal(5).astype(np.float32)
        assert np.allclose(a1(o), a2(o), atol=1e-6)


def test_stateless_entry_points_refuse():
    import torch as th
    model = _model()
    with pytest.raises(NotImplementedError):
        model.policy(th.zeros(1, 5))
    with pytest.raises(NotImplementedError):
        model.policy.evaluate_actions(th.zeros(1, 5), th.zeros(1, 3))


def test_bptt_must_divide_the_rollout():
    with pytest.raises(ValueError, match="bptt"):
        _model(bptt=24)


def test_parse_memory():
    from aow_sim.recurrent_ppo import parse_memory
    assert parse_memory(None) is None
    assert parse_memory({}) is None
    assert parse_memory({"hidden": 64}) == {"hidden": 64, "bptt": 32}
    with pytest.raises(ValueError, match="unknown"):
        parse_memory({"hiden": 64})
