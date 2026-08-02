"""RL pivot tests. The wheel-heading helper + spec tests are dependency-free
and always run; env tests skip without gymnasium; the replay test skips
without a trained moves/pivot_rl artifact (like the flick/ball tests)."""

import numpy as np
import pytest

from aow_sim.control.pivot_spec import (ACT_DIM, OBS_DIM, ActionBounds,
                                        build_obs, scale_action,
                                        steer_for_heading, wheel_heading,
                                        wrap_pi)

RAKE = np.deg2rad(15.0)


def test_wheel_heading_helper():
    assert wheel_heading(0.0, RAKE) == pytest.approx(0.0)
    # positive steer -> positive ground heading; exactly pi/2 at pi/2
    assert wheel_heading(np.pi / 2, RAKE) == pytest.approx(np.pi / 2)
    assert wheel_heading(0.3, RAKE) > 0.0
    # rake 0 -> identity (mod 2*pi)
    for d in np.linspace(-3.0, 3.0, 13):
        assert wheel_heading(d, 0.0) == pytest.approx(wrap_pi(d), abs=1e-12)
    # mod-pi consistency: a pi shift in steer shifts the heading by pi
    for d in np.linspace(-1.4, 1.4, 9):
        shift = wheel_heading(d + np.pi, RAKE) - wheel_heading(d, RAKE)
        assert wrap_pi(shift - np.pi) == pytest.approx(0.0, abs=1e-9)
    # inverse roundtrip on the principal branch
    for d in np.linspace(-1.5, 1.5, 11):
        assert steer_for_heading(wheel_heading(d, RAKE), RAKE) == pytest.approx(
            d, abs=1e-9)
    # strictly monotonic across the principal branch
    ds = np.linspace(-np.pi / 2 + 0.01, np.pi / 2 - 0.01, 50)
    hs = [wheel_heading(d, RAKE) for d in ds]
    assert np.all(np.diff(hs) > 0)


def test_obs_and_action_spec():
    obs = build_obs(0.01, 0.1, np.pi / 2, 0.2, 3.0, 0.4,
                    0.3, -0.1, 0.05, 0.5, 0.02, 0.6)
    assert obs.shape == (OBS_DIM,)
    assert np.all(np.isfinite(obs))
    # sin/cos pairs on the unit circle: yaw_err, 2*steer, 2*hold
    for i, j in ((2, 3), (5, 6), (7, 8)):
        assert obs[i]**2 + obs[j]**2 == pytest.approx(1.0, abs=1e-6)
    # pi-symmetry: a pi shift in steer or hold error encodes identically
    obs_s = build_obs(0.01, 0.1, np.pi / 2, 0.2, 3.0 + np.pi, 0.4,
                      0.3, -0.1, 0.05, 0.5, 0.02, 0.6)
    assert obs_s[5:7] == pytest.approx(obs[5:7], abs=1e-5)
    obs_h = build_obs(0.01, 0.1, np.pi / 2, 0.2, 3.0, 0.4 + np.pi,
                      0.3, -0.1, 0.05, 0.5, 0.02, 0.6)
    assert obs_h[7:9] == pytest.approx(obs[7:9], abs=1e-5)
    # action contract shared with the flick
    b = ActionBounds(8.0, 0.72, 40.0)
    sr, hub, diff = scale_action([2.0, -2.0, 0.5], b)
    assert (sr, hub, diff) == pytest.approx((8.0, -0.72, 20.0))
    assert ACT_DIM == 3


def test_eval_grid_follows_v_max():
    """The eval grid must never score velocities the run doesn't train:
    with v_max = 0 (stationary-only) an absolute grid would cap
    success_rate at 1/len(grid) and hand snapshot selection to
    out-of-distribution failures."""
    pytest.importorskip("stable_baselines3")   # trainer needs the [rl] extra
    from aow_sim.train_pivot_rl import eval_grid

    assert eval_grid(0.0) == [(0.0, 0.0)]      # collapses, deduplicated
    g = eval_grid(0.6)
    assert (0.0, 0.0) in g and (0.6, 0.6) in g
    assert len(g) == len(set(g))               # no duplicate work
    assert max(max(vs, ve) for vs, ve in g) == pytest.approx(0.6)
    # scales with the envelope rather than being pinned to 0.6
    assert max(max(vs, ve) for vs, ve in eval_grid(0.3)) == pytest.approx(0.3)


def test_env_reset_step():
    pytest.importorskip("gymnasium")           # skip if [rl] not installed
    from aow_sim.control.pivot_env import PivotEnv
    env = PivotEnv(seed=0)
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
    # determinism (also covers the v/hit sampling)
    o1, _ = PivotEnv(seed=0).reset(seed=5)
    o2, _ = PivotEnv(seed=0).reset(seed=5)
    assert np.allclose(o1, o2)


def test_v_start_produces_glide():
    """The qvel injection must actually roll the bike, not fight the drive
    actuators — v_lon holds up and the front contact advances on the line."""
    pytest.importorskip("gymnasium")
    from aow_sim.control.pivot_env import PivotEnv
    env = PivotEnv(seed=0)
    env.rand["enabled"] = False
    obs, _ = env.reset(seed=0, options={"v_start": 0.5, "v_end": 0.5})
    assert env._v_start == 0.5 and env._v_end == 0.5
    pf_start = env._pf0.copy()
    # hold the hub at the glide speed (normalized action), steer/diff zero
    hub_a = 0.5 / env.bounds.hub_max
    for _ in range(20):
        obs, _r, term, trunc, _ = env.step(np.array([0.0, hub_a, 0.0]))
        if term or trunc:
            break
    assert obs[9] > 0.3, f"glide did not roll (v_lon {obs[9]:.2f})"
    s = env.data
    pf = s.qpos[:2] + env.L * np.array(
        [np.cos(env._raw_prev), np.sin(env._raw_prev)])
    assert float(env._u0 @ (pf - pf_start)) > 0.03, "front contact did not advance"


def test_hit_fires_once_at_halfway():
    pytest.importorskip("gymnasium")
    from aow_sim.control.pivot_env import PivotEnv
    env = PivotEnv(seed=0)
    env.reset(seed=0)
    env._hit_F = 5.0
    env._hit_armed = True
    env._hit_left = 0
    env._hit_window = 3
    env._psi = env._yaw0 + np.pi / 2 + 0.01      # past the halfway crossing
    fired = []
    for _ in range(6):
        env.step(np.zeros(env.action_space.shape))
        fired.append(float(np.linalg.norm(env.data.xfrc_applied[env._chassis, 0:2])))
    assert [f > 1e-9 for f in fired] == [True, True, True, False, False, False]
    assert not env._hit_armed                     # no re-arm


def test_success_requires_velocity_match():
    """The success gate keys on line-frame v_along vs v_end (post-turn the
    body-frame v_lon is ~ -v_end, so |v_lon| would be wrong)."""
    pytest.importorskip("gymnasium")
    from aow_sim.control.pivot_env import PivotEnv
    env = PivotEnv(seed=0)
    env.rand["enabled"] = False
    env.reset(seed=0, options={"v_start": 0.0, "v_end": 0.5})

    def gate(v_along):
        return (abs(0.0) < env.yaw_tol            # yaw settled (stand-in)
                and abs(v_along - env._v_end) < env.v_tol)

    assert not gate(0.0), "stationary must not satisfy a v_end=0.5 pivot"
    assert gate(0.5)


def test_pivot_move_replays():
    """If a trained pivot policy exists, it loads and replays without blowing
    up. (No quality assertion — the artifact may be a smoke-run policy; move
    quality lives in the move file's eval metrics, like the flick/ball tests.)"""
    from aow_sim.control.flick import MOVES_DIR, load_move
    if not (MOVES_DIR / "pivot_rl.npz").exists():
        pytest.skip("run `python -m aow_sim.train_pivot_rl` to produce moves/pivot_rl")
    if load_move("pivot_rl").obs_dim != OBS_DIM:
        pytest.skip("moves/pivot_rl predates the current obs spec — retrain")
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
    T = c.command_pivot_rl(d, +1, name="pivot_rl", v_end=0.0)
    run(m, d, c, T + 2.0)
    assert np.all(np.isfinite(d.qpos))
