"""RL ball-shot tests. The spec test is dependency-free and always runs; the
model/collision test needs only mujoco; the env test skips without gymnasium; the
replay test skips without a trained moves/ball_rl artifact (like the flick tests)."""

import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control.ball_spec import OBS_DIM, ACT_DIM, ActionBounds, build_obs, scale_action


def test_obs_and_action_spec():
    obs = build_obs(0.01, 0.1, np.pi / 3, 0.2, 3.0, 0.4, -0.1,
                    0.3, -0.2, 0.0, 0.0, 1.0, 0.5)
    assert obs.shape == (OBS_DIM,)
    assert np.all(np.isfinite(obs))
    assert obs[5]**2 + obs[6]**2 == pytest.approx(1.0, abs=1e-6)  # sin/cos steer
    assert obs[13] == 1.0                         # ball_present flag
    # action contract is shared with the flick
    b = ActionBounds(8.0, 0.72, 30.0)
    sr, hub, diff = scale_action([2.0, -2.0, 0.5], b)   # clips to [-1,1]
    assert (sr, hub, diff) == pytest.approx((8.0, -0.72, 15.0))
    assert scale_action([0.5, 0.5], b)[2] == 0.0        # feedforward form -> diff 0
    assert ACT_DIM == 3


def _collide(m, a, b):
    ia, ib = m.geom(a).id, m.geom(b).id
    ct, ca = m.geom_contype, m.geom_conaffinity
    return bool((ct[ia] & ca[ib]) or (ct[ib] & ca[ia]))


def test_hockey_model_collision_classes():
    p = load_params()
    m = build_model(p, variant="full", hockey=True)
    m0 = build_model(p, variant="full", hockey=False)
    assert m.ngeom - m0.ngeom == 3            # 2 sticks + 1 ball, base unchanged
    # ball must strike floor, stick, and wheels (so a wheel hit can be penalized)
    assert _collide(m, "ball", "floor")
    assert _collide(m, "ball", "stick_left")
    assert _collide(m, "ball", "front_tire")
    assert _collide(m, "ball", "roller_0_a")
    # stick collides with floor + ball but not the bike's own dynamic geoms
    assert _collide(m, "stick_left", "floor")
    assert not _collide(m, "stick_left", "front_tire")
    assert not _collide(m, "stick_left", "stick_right")
    # base bike model is byte-for-byte unaffected by the hockey flag
    assert m0.ngeom == build_model(p, variant="full").ngeom


def test_env_reset_step():
    pytest.importorskip("gymnasium")           # skip if [rl] not installed
    from aow_sim.control.ball_env import BallEnv
    env = BallEnv(seed=0)
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
    assert term or trunc                        # episodes end (fall / shot / timeout)
    # determinism
    o1, _ = BallEnv(seed=0).reset(seed=5)
    o2, _ = BallEnv(seed=0).reset(seed=5)
    assert np.allclose(o1, o2)


def test_no_ball_trial_toggles_present_flag():
    pytest.importorskip("gymnasium")
    from aow_sim.control.ball_env import BallEnv
    env = BallEnv(seed=0)
    env.no_ball_prob = 1.0                      # force a catch/no-ball episode
    obs, _ = env.reset(seed=2)
    assert env.has_ball is False
    assert obs[13] == 0.0                       # ball_present flag off
    # a no-ball trial is just a balance task; a few steps stay finite
    for _ in range(20):
        obs, r, term, trunc, _ = env.step(np.zeros(env.action_space.shape))
        assert np.all(np.isfinite(obs)) and np.isfinite(r)
        if term or trunc:
            break


def test_launch_reward_paid_once_on_release():
    """Regression: the launch reward must fire once, on the falling edge of stick
    contact — not on every step of a sustained contact. Paying per contact step
    made a slow shove (contact duration x speed) outscore a clean strike, and the
    policy duly learned to carry the ball instead of hitting it."""
    pytest.importorskip("gymnasium")
    from aow_sim.control.ball_env import BallEnv
    env = BallEnv(seed=0)
    env.reset(seed=0)
    assert env.has_ball
    # Script contact for control steps 0-4, release at step 5 (the substep scan
    # sees env._step, which increments only after the substep loop).
    env._ball_speed = lambda: 2.0
    env._projected_speed = lambda: 2.0      # reward keys on the projection
    env._scan_ball_contacts = lambda: (env._step < 5, False)
    # Keep the success/terminate path out of it: the bike starts settled, so a
    # registered shot would otherwise end the episode instantly on bonus_complete.
    env.hit_speed_min = 10.0
    w = env.rw
    launch = w["w_launch"] * 2.0
    rewards = []
    for _ in range(8):
        _obs, r, term, trunc, _info = env.step(np.zeros(env.action_space.shape))
        rewards.append(r)
        if term or trunc:
            break
    big = [i for i, r in enumerate(rewards) if r > 0.5 * launch]
    assert big == [5], f"launch reward should fire once at release, got {big}"
    assert env._launch_paid


def test_strike_normal_speed_rejects_sideswipe():
    """Regression: a broadside glancing pass must score ~0 strike speed.

    perp_align constrains pose only, so sliding past the ball while broadside
    satisfies it fully whilst transferring no momentum — the policy duly learned
    to sideswipe. Momentum transfer follows the panel's *normal* velocity, so
    that is what _strike_normal_speed measures and what the reward pays for.
    """
    pytest.importorskip("gymnasium")
    from aow_sim.control.ball_env import BallEnv
    env = BallEnv(seed=0)
    env.rand["enabled"] = False
    env.reset(seed=0)
    env._psi = np.deg2rad(90)          # broadside; outward normal points +x

    def vn(vx, vy, omega):
        env.data.qvel[0], env.data.qvel[1], env.data.qvel[5] = vx, vy, omega
        return env._strike_normal_speed()

    # rolling along its own heading (+y) past the ball: no normal component
    assert vn(0.0, 0.7, 0.0) == pytest.approx(0.0, abs=1e-6)
    # driving straight into the ball, and pivoting to swing the tail in: positive
    assert vn(0.7, 0.0, 0.0) == pytest.approx(0.7, abs=1e-6)
    assert vn(0.7, 0.0, -3.0) > 0.7    # pivot adds to the translation
    # retreating is negative, and the reward clamps it at zero
    assert vn(-0.5, 0.0, 0.0) < 0.0


def test_launch_reward_is_direction_projected():
    """Regression: launch reward keys on speed ALONG the world target, not |v|.

    Rewarding raw speed pays for a fast ball in any direction — a front-edge
    scrape that squirted the ball out at -98 deg scored as the best shot of a run.
    Projection makes direction intrinsic: sideways and backwards are worth zero
    and do not count as a shot at all, however fast.
    """
    pytest.importorskip("gymnasium")
    from aow_sim.control.ball_env import BallEnv
    env = BallEnv(seed=0)
    env.rand["enabled"] = False
    env.reset(seed=0)
    speed = 0.605                      # same speed, different directions

    def proj_at(deg):
        v = speed * np.array([np.cos(np.deg2rad(deg)), np.sin(np.deg2rad(deg))])
        env.data.qvel[env._ball_v:env._ball_v + 2] = v
        return env._projected_speed()

    assert proj_at(0.0) == pytest.approx(speed, abs=1e-6)      # straight at target
    assert proj_at(-98.2) < 0.0                                # sideways/back: nil
    assert proj_at(-180.0) == pytest.approx(-speed, abs=1e-6)
    assert 0 < proj_at(-35.4) < speed                          # partial credit

    # a fast sideways squirt earns no bonus and is not a shot
    env._peak_proj = max(0.0, proj_at(-98.2))
    assert env._bonus() == 0.0
    assert not (env._peak_proj > env.hit_speed_min)
    # a forward launch of the same speed earns full credit
    env._peak_proj = proj_at(0.0)
    assert env._bonus() == pytest.approx(env.rw["bonus_complete"], rel=1e-6)
    assert env._peak_proj > env.hit_speed_min


def test_ball_move_replays():
    """If a trained ball policy exists, it loads and replays without blowing up."""
    from aow_sim.control.flick import MOVES_DIR, load_move
    if not (MOVES_DIR / "ball_rl.npz").exists():
        pytest.skip("run `python -m aow_sim.train_ball_rl` to produce moves/ball_rl")
    if load_move("ball_rl").obs_dim != OBS_DIM:
        pytest.skip("moves/ball_rl predates the current obs spec — retrain")
    import mujoco
    from aow_sim.control import DriveController, run
    from aow_sim.control.linearize import settle_upright
    p = load_params()
    m = build_model(p, variant="full", hockey=True)
    eq = settle_upright(m)
    c = DriveController(p, m)
    d = mujoco.MjData(m)
    d.qpos[:] = eq.qpos
    a = np.deg2rad(0.5)
    d.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(m, d)
    c.reset(m, d)
    run(m, d, c, 1.0)
    T = c.command_ball(d, name="ball_rl")
    run(m, d, c, T + 2.0)
    assert np.all(np.isfinite(d.qpos))
