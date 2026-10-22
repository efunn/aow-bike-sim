"""The steer rate the policy observes, formed as the Pi forms it.

`hw/bike_bus.py` differences the steer servo's counts per 100 Hz tick and
smooths them through RateFilter(25 ms, 0.5); training used to hand the policy
MuJoCo's exact steer velocity instead. `sim_odometry.SimSteerRate` is the
Pi's path in the sim, `GeneralEnv` observes it under `steer_rate_encoder:
counts`, the move yaml records it, and `gpu/odometry.py` computes it in the
same tick as the wheel rates. What invalidates these: the obs[5] contract
(general_spec), the RateFilter or bike_bus's differencing, either env's
sense tick.
"""
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params
from aow_sim.control.steer import XC330_COUNTS_PER_RAD
from aow_sim.sim_odometry import SimSteerRate

pytestmark = pytest.mark.spec

CFG = "config/rl/rl_general_v3.yaml"


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params)


def _bike_bus_reference(angles, ratio, dt=0.01, weights=(1.0, 0.5)):
    """bike_bus.read_state for an extended-position servo, written out: counts
    (servo = steer x ratio), backward difference, 2-tap weighted mean, back to
    steer rad/s. The first tick has no interval and reads 0."""
    out, prev, buf = [], None, []
    for a in angles:
        c = int(round(a * ratio * XC330_COUNTS_PER_RAD))
        if prev is None:
            out.append(0.0)
        else:
            buf = [((c - prev) / XC330_COUNTS_PER_RAD) / dt] + buf[:1]
            w = np.array(weights[:len(buf)])
            out.append(float(np.dot(w, buf) / w.sum()) / ratio)
        prev = c
    return np.array(out)


def test_counts_match_the_pis_arithmetic_tick_for_tick(model, params):
    est = SimSteerRate(model, params, encoder="counts")
    data = mujoco.MjData(model)
    t = np.arange(60) * est.odo_dt
    angles = 0.3 * np.sin(2 * np.pi * 3.0 * t) + 0.7 * t      # a sine on a ramp
    got = []
    for a in angles:
        data.sensordata[est.adr] = a
        got.append(est.update(data, est.odo_dt))   # exactly one tick each
        assert est.update(data, 0.0) == got[-1]    # held between ticks
    np.testing.assert_allclose(got, _bike_bus_reference(angles, est.ratio),
                               rtol=0, atol=1e-12)


def test_it_ticks_on_the_odometrys_substeps(model, params):
    """Same clock arithmetic as SimOdometry, so in GeneralEnv's substep loop
    the steer and the wheels are sampled on the same physics step -- as one
    bike_bus frame reads every servo at once."""
    from aow_sim.sim_odometry import SimOdometry
    est = SimSteerRate(model, params, encoder="counts")
    odo = SimOdometry(model, params, encoder="counts")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for k in range(2500):                       # 1 s of physics steps
        a0, o0 = est._acc, odo._acc
        est.update(data, model.opt.timestep)
        odo.update(data, model.opt.timestep)
        assert (est._acc < a0) == (odo._acc < o0), f"substep {k}"


def test_it_lags_the_exact_rate(model, params):
    """The point of the whole thing: ~8 ms behind at the Pi's settings."""
    est = SimSteerRate(model, params, encoder="counts")
    data = mujoco.MjData(model)
    f, A = 5.0, 0.2
    t = np.arange(200) * est.odo_dt
    vals = []
    for tk in t:
        data.sensordata[est.adr] = A * np.sin(2 * np.pi * f * tk)
        vals.append(est._advance(data, est.odo_dt))
    vals = np.array(vals[20:])
    tt = t[20:]
    X = np.c_[np.sin(2 * np.pi * f * tt), np.cos(2 * np.pi * f * tt)]
    a, b = np.linalg.lstsq(X, vals, rcond=None)[0]
    exact_phase = np.pi / 2                     # d/dt sin = cos
    lag_s = (exact_phase - np.arctan2(b, a)) / (2 * np.pi * f)
    assert 0.006 < lag_s < 0.011, f"lag {lag_s * 1000:.1f} ms"


def test_joint_is_the_exact_velocity(model, params):
    est = SimSteerRate(model, params, encoder="joint")
    data = mujoco.MjData(model)
    data.qvel[est.dof] = 1.234
    assert est.update(data, model.opt.timestep) == 1.234


def test_estimated_swaps_only_the_steer_rate(model, params):
    est = SimSteerRate(model, params, encoder="counts")
    data = mujoco.MjData(model)
    data.qvel[:] = np.arange(model.nv) * 0.1
    before = data.qvel.copy()
    with est.estimated(data, model.opt.timestep):
        assert data.qvel[est.dof] == est._last
        keep = np.ones(model.nv, bool)
        keep[est.dof] = False
        np.testing.assert_array_equal(data.qvel[keep], before[keep])
    np.testing.assert_array_equal(data.qvel, before)


def _env(encoder):
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    cfg = _load_rl_config(CFG)
    cfg = {**cfg, "env": {**cfg["env"], "steer_rate_encoder": encoder}}
    return GeneralEnv(None, cfg, seed=0)


def test_env_observes_the_estimate_under_counts():
    env = _env("counts")
    obs, _ = env.reset(seed=0)
    assert obs[5] == 0.0                        # fresh filter, no tick yet
    rng = np.random.default_rng(0)
    for _ in range(25):
        obs, *_ = env.step(rng.uniform(-1, 1, env.action_space.shape[0]))
    assert obs[5] == env._steer_rate.update(env.data, 0.0)
    assert obs[5] != env.data.qvel[env._sd]


def test_env_default_is_the_old_exact_signal():
    env = _env("joint")
    assert env._steer_rate is None
    env.reset(seed=0)
    obs, *_ = env.step(np.full(env.action_space.shape[0], 0.5))
    assert obs[5] == np.float32(env.data.qvel[env._sd])


def test_the_move_yaml_records_it_and_replay_reads_it(params):
    from aow_sim.control.general_env import _load_rl_config
    from aow_sim.control.general_spec import policy_env_overrides
    from aow_sim.train_general_rl import move_doc
    cfg = _load_rl_config(CFG)
    assert cfg["env"]["steer_rate_encoder"] == "counts"
    doc = move_doc("x_smoke", params, cfg, trained={})
    assert doc["steer_rate_encoder"] == "counts"
    del cfg["env"]["steer_rate_encoder"]
    assert move_doc("x_smoke", params, cfg, trained={})["steer_rate_encoder"] == "joint"
    # An export from before the field reads as "joint" and builds a "joint" env.
    from aow_sim.control.flick import load_move
    old = load_move(params["control"]["general_move"])
    assert old.steer_rate_encoder == "joint"
    assert policy_env_overrides(old)["steer_rate_encoder"] == "joint"
    old.steer_rate_encoder = "counts"
    assert policy_env_overrides(old)["steer_rate_encoder"] == "counts"


def test_warp_kernel_matches_the_python_estimate():
    """gpu/odometry.py's steer channel against SimSteerRate, tick for tick, on
    Warp's CPU device. Angles are float32-exact so both round the same."""
    wp = pytest.importorskip("warp")
    pytest.importorskip("mujoco_warp")
    from aow_sim.gpu import odometry as godo
    wp.config.quiet = True
    wp.init()
    env = _env("counts")
    c = godo.const_from(env._odo, env._steer_rate)
    assert c.steer_rate_on == 1
    assert godo.const_from(env._odo).steer_rate_on == 0
    s = godo.OdoState(1, device="cpu")
    est = env._steer_rate
    est.reset()
    data = mujoco.MjData(env.model)
    nsd = env.model.nsensordata
    quat = wp.array(np.array([[1.0, 0, 0, 0]], np.float32), device="cpu")
    vec = wp.array(np.zeros((1, 3), np.float32), device="cpu")
    vec_g = wp.array(np.array([[0, 0, 9.81]], np.float32), device="cpu")
    t = np.arange(40) * est.odo_dt
    angles = np.round((0.4 * np.sin(2 * np.pi * 4.0 * t) + 1.5 * t) * 4096) / 4096
    for a in angles:
        sd = np.zeros((1, nsd), np.float32)
        sd[0, est.adr] = a
        godo.launch_advance(c, s, wp.array(sd, device="cpu"), quat, vec, vec_g)
        data.sensordata[est.adr] = a
        cpu = est._advance(data, est.odo_dt)
        gpu = float(s.steer_rate.numpy()[0])
        assert gpu == pytest.approx(cpu, rel=1e-5, abs=1e-4), (a, cpu, gpu)
