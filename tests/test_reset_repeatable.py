"""A reused env must reproduce a fresh one: reset(seed) is the whole state.

Every RL env resets by hand (qpos/qvel written, no `mj_resetData`), so
anything MjData carries that the hand reset forgets leaks from one episode
into the next. Until 2026-10-14 the steer's command delay line did: each
episode's first 4 ms drove the steer toward the PREVIOUS episode's last
target, and the same seeded eval episode differed by up to 0.68 m of qpos
depending on what ran before it (build_model.reset_actuator_state).
"""
import numpy as np
import pytest

pytestmark = pytest.mark.spec

_OPTS = {"v_cmd": (0.5, 0.0), "psi_cmd_rel": 0.0, "difficulty": 1.0}


def _env():
    from aow_sim.build_model import load_params
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    from aow_sim.drivetrain_model import from_env_config
    from aow_sim.train_general_rl import _eval_cfg
    cfg = _eval_cfg(_load_rl_config("config/rl_general_v3.yaml"))
    return GeneralEnv(from_env_config(load_params(), cfg["env"]), cfg)


def _episode(env, acts):
    env.reset(seed=7, options=_OPTS)
    q = []
    for a in acts:
        env.step(a[:env.action_space.shape[0]])
        q.append(env.data.qpos.copy())
    return np.array(q)


def test_a_reused_env_reproduces_a_fresh_one_exactly():
    acts = np.random.default_rng(0).uniform(-0.3, 0.3, (40, 4))
    ref = _episode(_env(), acts)
    env = _env()
    # dirty it: a different episode that winds the steer target well away
    _episode(env, np.tile([1.0, 0.2, 0.0, 0.0], (40, 1)))
    assert abs(env.data.ctrl[env.model.actuator("steer").id]) > 0.1, "not dirtied"
    again = _episode(env, acts)
    assert np.array_equal(again, ref), np.abs(again - ref).max()


def test_reset_empties_the_command_delay_line():
    import mujoco
    env = _env()
    m = env.model
    assert m.actuator_history[:, 0].any(), "no delayed actuator: vacuous"
    _episode(env, np.tile([1.0, 0.0, 0.0, 0.0], (20, 1)))
    env.reset(seed=7, options=_OPTS)
    assert np.array_equal(env.data.history, mujoco.MjData(m).history
                          + _time_shift(m, env.data.time))


def _time_shift(m, t):
    """A fresh MjData's history with its timestamps moved to time `t`: the
    layout per delayed actuator is [user, cursor, n times, n values]."""
    out = np.zeros(m.nhistory)
    for i in np.flatnonzero(m.actuator_history[:, 0]):
        n, adr = int(m.actuator_history[i, 0]), int(m.actuator_historyadr[i])
        out[adr + 2:adr + 2 + n] = t
    return out
