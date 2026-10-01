"""SmoothPPO: smoothness terms on the policy mean (src/aow_sim/smooth_ppo.py).

The guard that matters is the first test below: with every weight at 0, one
update is bit-identical to stock PPO. `SmoothPPO.train` is a COPY of SB3's, so
an SB3 upgrade that changes the original shows up here and nowhere else.
Torch and SB3 are imported inside the tests -- `pytest -m pure` still imports
every test file.
"""
import numpy as np
import pytest

pytestmark = pytest.mark.trainer


def _pendulum_models(bias=None, **kw):
    """Stock PPO and SmoothPPO, same seed, one rollout + update each, on a
    classic-control env: no MuJoCo model to build. Each is built AND trained
    before the next is built: SB3 seeds the GLOBAL torch RNG at construction,
    so building both first would hand the second one the first's leftover
    stream."""
    pytest.importorskip("gymnasium")
    import torch as th
    from stable_baselines3 import PPO
    from aow_sim.smooth_ppo import SmoothPPO

    common = dict(n_steps=64, batch_size=32, n_epochs=2, seed=3, device="cpu",
                  policy_kwargs=dict(net_arch=[16, 16]))
    def run(m):
        if bias is not None:      # start the mean where a term can see it
            with th.no_grad():
                m.policy.action_net.bias.fill_(bias)
        m.learn(64)
        return m

    a = run(PPO("MlpPolicy", "Pendulum-v1", **common))
    b = run(SmoothPPO("MlpPolicy", "Pendulum-v1", **common, **kw))
    return a, b


def _params(m):
    return {k: v.detach().clone() for k, v in m.policy.state_dict().items()}


def test_zero_weights_are_bit_identical_to_stock_ppo():
    import torch as th
    a, b = _pendulum_models(smooth=None)
    pa, pb = _params(a), _params(b)
    assert pa.keys() == pb.keys()
    for k in pa:
        assert th.equal(pa[k], pb[k]), k


def test_a_weight_changes_the_update():
    """Not vacuous: the same run with a term on moves the weights. A fresh
    policy's mean sits near 0, where the bound term has no gradient at all, so
    both start with the output bias at 3 -- past the bound."""
    import torch as th
    for term in ("bound", "temporal", "second"):
        a, b = _pendulum_models(bias=3.0, smooth={term: 1.0})
        assert any(not th.equal(x, y) for x, y in
                   zip(_params(a).values(), _params(b).values())), term


def test_term_values_on_known_signals():
    import torch as th
    from aow_sim.smooth_ppo import bound_term, second_term, temporal_term

    mu = th.tensor([[0.5, -0.9, 1.0], [2.0, -3.0, 0.0]])
    assert bound_term(mu).tolist() == pytest.approx([0.0, 1.0 + 4.0])

    t = th.arange(5.0)[:, None] * th.tensor([[0.3, -0.2, 0.0]])  # a ramp
    assert second_term(t[:-2], t[1:-1], t[2:]).abs().max().item() < 1e-12
    assert temporal_term(t[:-1], t[1:]).tolist() == pytest.approx([0.13] * 4)

    z = th.tensor([[1.0], [-1.0], [1.0]])                         # a zig-zag
    assert temporal_term(z[:1], z[1:2]).item() == 4.0
    assert second_term(z[:1], z[1:2], z[2:]).item() == 16.0


def test_windows_never_straddle_an_episode_start():
    from aow_sim.smooth_ppo import windows

    # 6 steps, 2 envs. env 0 starts a new episode at step 3; env 1 never does.
    starts = np.zeros((6, 2))
    starts[0] = 1
    starts[3, 0] = 1
    t, e = windows(starts, 2)
    pairs = set(zip(t.tolist(), e.tolist()))
    assert (2, 0) not in pairs                     # 2 -> 3 crosses the reset
    assert {(0, 0), (1, 0), (3, 0), (4, 0)} <= pairs
    assert {(i, 1) for i in range(5)} <= pairs
    assert len(pairs) == 4 + 5
    t, e = windows(starts, 3)
    triples = set(zip(t.tolist(), e.tolist()))
    assert triples == {(0, 0), (3, 0)} | {(i, 1) for i in range(4)}


def test_parse_smooth_rejects_a_typo_and_fills_absent_terms():
    from aow_sim.smooth_ppo import parse_smooth

    assert parse_smooth(None) == {"bound": 0.0, "temporal": 0.0, "second": 0.0}
    assert parse_smooth({"second": 0.015})["second"] == 0.015
    with pytest.raises(ValueError, match="unknown term"):
        parse_smooth({"temporl": 0.04})
    with pytest.raises(ValueError, match=">= 0"):
        parse_smooth({"bound": -1})


def test_a_smooth_checkpoint_loads_both_ways(tmp_path):
    """The trainer resumes with SmoothPPO.load and exports with PPO.load."""
    import torch as th
    from stable_baselines3 import PPO
    from aow_sim.smooth_ppo import SmoothPPO

    _a, b = _pendulum_models(smooth={"bound": 0.1, "temporal": 0.04})
    z = tmp_path / "m.zip"
    b.save(z)
    again = SmoothPPO.load(z, device="cpu")
    assert again.smooth == b.smooth
    plain = PPO.load(z, device="cpu")
    for k, v in _params(b).items():
        assert th.equal(v, _params(plain)[k])
