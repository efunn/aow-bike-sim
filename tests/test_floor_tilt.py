"""The per-episode floor tilt in GeneralEnv (`floor_tilt_max_deg`, default 0).

Pinned: off by default, and then the env is untouched -- the same seeded
episode as before the option existed, the floor level; on, the floor really
turns (its WORLD pose, which MuJoCo takes from the body and ignores
geom_quat for a geom compiled with geom_sameframe = 1 -- the trap this
caught), the bike spawns on it with no more contact depth than on level
ground, the tilt stays inside its range and scales with difficulty under
`floor_tilt_curriculum`.

The GPU reset (gpu/reset.py) does the same in a kernel; it needs Warp and
is checked by hand, not here.
"""
import numpy as np
import pytest

pytestmark = pytest.mark.spec

_OPTS = {"v_cmd": (0.0, 0.0), "psi_cmd_rel": 0.0, "difficulty": 1.0}


def _env(**env_over):
    from aow_sim.build_model import load_params
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    from aow_sim.drivetrain_model import from_env_config
    from aow_sim.train_general_rl import _eval_cfg
    cfg = _eval_cfg(_load_rl_config("config/rl_general_v3.yaml"))
    cfg = {**cfg, "env": {**cfg["env"], **env_over}}
    return GeneralEnv(from_env_config(load_params(), cfg["env"]), cfg)


def _floor_normal(env):
    return env.data.geom_xmat[env._floor_g].reshape(3, 3)[:, 2]


def _deepest(env):
    d = env.data
    return min((d.contact[i].dist for i in range(d.ncon)), default=0.0)


def test_off_by_default_and_then_nothing_changes():
    a, b = _env(), _env(floor_tilt_max_deg=0.0)
    assert a.tilt_max == 0.0
    oa, _ = a.reset(seed=3, options=_OPTS)
    ob, _ = b.reset(seed=3, options=_OPTS)
    assert np.array_equal(oa, ob) and np.array_equal(a.data.qpos, b.data.qpos)
    assert np.allclose(_floor_normal(a), [0, 0, 1])
    assert a.model.geom_sameframe[a._floor_g] == 1, "untouched when off"


@pytest.mark.parametrize("seed", range(4))
def test_the_floor_turns_and_the_bike_spawns_on_it(seed):
    level = _env()
    level.reset(seed=seed, options=_OPTS)
    env = _env(floor_tilt_max_deg=15.0, floor_tilt_curriculum=False)
    env.reset(seed=seed, options=_OPTS)
    tilt = np.degrees(np.arccos(_floor_normal(env)[2]))
    assert tilt == pytest.approx(env.floor_tilt_deg, abs=1e-6)
    assert 0.0 <= env.floor_tilt_deg <= 15.0
    # Spawned on the plane: no deeper than on level ground (measured: 28.9 mm
    # into a floor that never actually turned, before the sameframe fix).
    assert _deepest(env) > _deepest(level) - 1e-3
    for _ in range(25):                    # and it stays on it, not through it
        env.step(np.zeros(env.action_space.shape[0]))
    assert np.isfinite(env.data.qpos).all()
    n = _floor_normal(env)
    assert env.data.qpos[:3] @ n > 0.02, "the axle sank through the floor"


def test_the_curriculum_scales_the_tilt():
    env = _env(floor_tilt_max_deg=15.0)            # curriculum on by default
    tilts = []
    for seed in range(8):
        env.reset(seed=seed, options={**_OPTS, "difficulty": 0.2})
        tilts.append(env.floor_tilt_deg)
    assert max(tilts) <= 0.2 * 15.0 + 1e-9 and max(tilts) > 0
