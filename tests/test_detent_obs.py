"""The detent phase as an observation (`obs_detent`): sin/cos of where
ring-vs-hub sits in its 7.5 deg notch, 0 at the sawtooth's flip, and the
measured diff rate over diff_max (general_spec.detent_angle).

The phase is fixed in the raw drive encoders until a rebuild
(physical-bike-tests.md item 6), so the bike measures it from onboard.yaml's
`detent_flip` and the sim from the episode's draw (DrivetrainSim.flip).
GeneralEnv, drive.py and the Warp kernels each form the three entries; these
check they agree, and that the bench's frame maps onto the model's through
servo_ids / servo_sign. What invalidates these: the general obs layout
(general_spec), the detent's shape or phase convention (drivetrain_model),
the onboard frame (servo_ids, servo_sign, detent_flip), the odometry's rates.
"""
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from aow_sim.control import general_spec as gs

pytestmark = pytest.mark.spec

CFG = "config/rl/rl_general_v5.yaml"
DETENT = ("sin_detent", "cos_detent", "diff_rate")
P = gs.DETENT_PERIOD
BELT = 3.0


def _cfg(**env):
    with open(CFG) as f:
        cfg = yaml.safe_load(f)
    cfg["env"].update(env)
    return cfg


# -- the spec ------------------------------------------------------------------

def test_layout_appends_three_entries_last():
    base = gs.obs_layout(obs_pitch=True)
    assert gs.obs_layout(obs_pitch=True, obs_detent=True) == base + DETENT
    assert gs.obs_dim_for(obs_pitch=True, obs_detent=True) == 20
    both = gs.obs_layout(obs_pitch=True, obs_style=True, obs_detent=True)
    assert both[-6:-3] == ("style", "sin_wheel_err", "cos_wheel_err") and both[-3:] == DETENT
    par = dict(zip(gs.OBS_NAMES, gs.OBS_MIRROR_PARITY))
    assert (par["sin_detent"], par["cos_detent"], par["diff_rate"]) == (-1, 1, -1)
    assert par["diff_rate"] == par["prev_diff"]


def test_policy_flags_default_off_for_old_exports():
    old = SimpleNamespace(obs_pitch=True)
    assert gs.policy_flags(old)["obs_detent"] is False
    new = SimpleNamespace(obs_pitch=True, obs_detent=True)
    assert gs.policy_env_overrides(new)["obs_detent"] is True
    assert gs.obs_layout_for(new)[-3:] == DETENT


def test_build_obs_appends_sin_cos_rate():
    args = (0.1, 0.0, 0.0, 0.2, 0.0, 0.3, 0.0, 0.5, 0.0, 0.4, np.zeros(3))
    plain = gs.build_obs(*args, pitch=(0.01, 0.0))
    o = gs.build_obs(*args, pitch=(0.01, 0.0), detent=(0.7, -0.25))
    np.testing.assert_array_equal(o[:17], plain)
    np.testing.assert_allclose(o[17:], [np.sin(0.7), np.cos(0.7), -0.25], rtol=1e-6)


@pytest.mark.parametrize("k", [-3, 0, 1, 72])
def test_angle_is_zero_at_the_flip_and_pi_midway(k):
    flip = np.deg2rad(0.43)
    assert np.cos(gs.detent_angle(flip + k * P, flip)) == pytest.approx(1.0)
    assert gs.detent_angle(flip + (k + 0.5) * P, flip) == pytest.approx(np.pi)
    assert 0.0 <= gs.detent_angle(flip - 1e-3 + k * P, flip) < 2 * np.pi


# -- the bench's frame onto the model's ----------------------------------------

def _shafts(theta, ids, signs):
    """BikeBus.to_controller_units' shaft_pos_a / _b from raw servo angles."""
    return tuple(theta[i] * BELT * s for i, s in zip(ids[:2], signs[:2]))


@pytest.mark.parametrize("ids,signs,eps", [
    ([102, 101, 103], [-1, 1], -1.0),     # today: A and B swapped (2026-10-24)
    ([101, 102, 103], [1, -1], 1.0),      # the swap reverted
])
def test_raw_encoders_at_the_bench_flip_read_zero(ids, signs, eps):
    """Servo angles whose BELT (theta_101 + theta_102) / 2 sits at the bench's
    flip, through either servo_ids / servo_sign, give angle 0 in the model's
    frame -- whatever the turn counts."""
    onboard = {"servo_ids": ids, "servo_sign": signs,
               "detent_flip": {"deg": 0.43, "frame_ids": [101, 102],
                               "frame_signs": [1, -1]}}
    assert gs.detent_frame_sign(ids, signs, [101, 102], [1, -1]) == eps
    flip = gs.detent_flip_model(onboard)
    rng = np.random.default_rng(0)
    for _ in range(5):
        t101 = rng.uniform(-40, 40)
        s_bench = np.deg2rad(0.43) + rng.integers(-500, 500) * P   # ring-vs-hub
        theta = {101: t101, 102: 2 * s_bench / BELT - t101}
        a, b = _shafts(theta, ids, signs)
        phi = 0.5 * a - 0.5 * b                  # ga = 1 - 0.5, gb = 0 - 0.5
        assert np.cos(gs.detent_angle(phi, flip)) == pytest.approx(1.0, abs=1e-9)


def test_the_onboard_file_converts():
    from aow_sim.params import load_params
    o = load_params()["control"]["onboard"]
    assert abs(gs.detent_flip_model(o)) == pytest.approx(np.deg2rad(o["detent_flip"]["deg"]))


def test_a_foreign_frame_is_refused():
    with pytest.raises(ValueError):
        gs.detent_frame_sign([1, 2, 3], [1, -1], [101, 102], [1, -1])
    with pytest.raises(ValueError):
        gs.detent_frame_sign([101, 102, 103], [1, 1], [101, 102], [1, -1])


# -- the sim -------------------------------------------------------------------

@pytest.fixture(scope="module")
def env():
    from aow_sim.control.general_env import GeneralEnv
    return GeneralEnv(None, _cfg(), seed=0)


def test_flip_is_the_top_of_the_sawtooth(env):
    """Just past the flip the hill pushes ring-vs-hub further on, just
    before it pushes back: the top, as the bench's creep locates it. Shafts at
    rest right after a reset, so the frictions read 0 and only the hill acts."""
    import mujoco
    dr = env._drive
    data = mujoco.MjData(env.model)
    qa, qb = dr.in_q
    for eps_deg in (-0.3, 0.3):
        mujoco.mj_resetData(env.model, data)
        dr.reset(data, np.random.default_rng(5))
        phi = dr.flip + np.deg2rad(eps_deg) + 40 * P
        data.qpos[qa], data.qpos[qb] = phi, -phi        # 0.5 a - 0.5 b = phi
        mujoco.mj_forward(env.model, data)
        dr.reset(data)                                  # anchors at this pose
        dr.pre_step(data)
        push = data.qfrc_applied[dr.in_d[0]] / dr.ga    # the force on phi
        assert np.sign(push) == np.sign(eps_deg)


def test_env_observes_angle_and_measured_rate(env):
    assert env.obs_layout[-3:] == DETENT and env.obs_dim == 20
    obs, _ = env.reset(seed=3)
    np.testing.assert_allclose(obs[19], 0.0)             # a fresh filter
    for k in range(12):
        obs, *_ = env.step(np.array([0.0, 0.1, 0.4 if k < 6 else -0.4]))
        ang = gs.detent_angle(env._drive.ring_vs_hub(env.data), env._drive.flip)
        np.testing.assert_allclose(obs[17:19], [np.sin(ang), np.cos(ang)], atol=1e-6)
        wa, wb = env._odo.last_rates
        assert obs[19] == pytest.approx(BELT * (wa - wb) / env.bounds.diff_max, abs=1e-6)
    assert obs[19] < 0 < obs[14] or obs[19] < 0          # follows the -0.4 command


def test_needs_the_detent_and_the_odometry():
    from aow_sim.control.general_env import GeneralEnv
    with pytest.raises(ValueError, match="obs_detent"):
        GeneralEnv(None, _cfg(obs_odometry=False), seed=0)


def test_move_doc_records_the_flag():
    from aow_sim.build_model import load_params
    from aow_sim.train_general_rl import move_doc
    doc = move_doc("general_detent_smoke", load_params(), _cfg(), {})
    assert doc["obs_detent"] is True and tuple(doc["obs_layout"][-3:]) == DETENT


def test_drive_controller_forms_the_same_entries(env):
    """drive.py's _detent_obs: in the sim from the model and the attached
    drivetrain's live flip; on the bike from fed shaft angles and a set flip."""
    from aow_sim.control.drive import DriveController
    ctl = DriveController(env.p, env.model)
    ctl._gen = SimpleNamespace(bounds=env.bounds)
    env.reset(seed=4)
    for _ in range(3):
        env.step(np.array([0.0, 0.0, 0.3]))
    ctl.crawl.rate = 1.7
    ang, rate = ctl._detent_obs(env.data)
    assert ang == pytest.approx(gs.detent_angle(env._drive.ring_vs_hub(env.data),
                                                env._drive.flip))
    assert rate == pytest.approx(1.7 / env.bounds.diff_max)
    ctl.set_detent_flip(0.01)
    ctl.feed_drive_angles(0.2, -0.1)
    assert ctl._detent_obs(env.data)[0] == pytest.approx(gs.detent_angle(0.15, 0.01))


# -- the GPU kernels -------------------------------------------------------------

def test_warp_obs_kernel_matches_the_env(env):
    """gpu/obs_reward.py's three entries against GeneralEnv's on Warp's CPU
    device, from the env's own state, phase and odometry rates."""
    wp = pytest.importorskip("warp")
    pytest.importorskip("mujoco_warp")
    from aow_sim.gpu import obs_reward as gor
    wp.config.quiet = True
    wp.init()
    c = gor.const_from(env)
    assert c.detent_on == 1 and c.det_i == 17
    env.reset(seed=6)
    for a in np.random.default_rng(2).uniform(-1, 1, (8, 3)):
        want, *_ = env.step(a)
    f = lambda x: wp.array(np.atleast_1d(np.asarray(x, np.float32)), dtype=float, device="cpu")
    f2 = lambda x: wp.array(np.asarray(x, np.float32)[None], dtype=float, device="cpu")
    i1 = lambda x: wp.array(np.array([x], np.int32), dtype=int, device="cpu")
    wa, wb = env._odo.last_rates
    obs = wp.zeros((1, 20), dtype=float, device="cpu")
    gor.launch(c, f2(env.data.qpos), f2(env.data.qvel), f2([1.0, 0, 0, 0]), f2(np.zeros(3)),
               f(0.0), f(0.0), f(0.0), f2(np.zeros(2)), f(0.0), f(0.0), f(0.0), f(0.0),
               f(env._drive.phase), f(BELT * (wa - wb)), f(0.0), f(0.0),
               f2(np.zeros(3)), f2(np.zeros(3)), i1(0), f(0.0), i1(0), obs,
               wp.zeros(1, dtype=float, device="cpu"), i1(0), i1(0))
    np.testing.assert_allclose(obs.numpy()[0, 17:], want[17:], atol=2e-4)


def test_warp_odometry_keeps_the_diff_rate(env):
    """gpu/odometry.py's w_diff against SimOdometry.last_rates, tick for tick."""
    wp = pytest.importorskip("warp")
    pytest.importorskip("mujoco_warp")
    import mujoco
    from aow_sim.gpu import odometry as godo
    wp.config.quiet = True
    wp.init()
    odo = env._odo
    c = godo.const_from(odo)
    s = godo.OdoState(1, device="cpu")
    odo.reset(env.model, env.p)
    data = mujoco.MjData(env.model)
    quat = wp.array(np.array([[1.0, 0, 0, 0]], np.float32), device="cpu")
    vec = wp.array(np.zeros((1, 3), np.float32), device="cpu")
    vec_g = wp.array(np.array([[0, 0, 9.81]], np.float32), device="cpu")
    adr_a, adr_b = odo.adr["input_a_pos"][0], odo.adr["input_b_pos"][0]
    t = np.arange(30) * odo.odo_dt
    for ti in t:
        a = np.float32(np.round((3.0 * ti + 0.2 * np.sin(20 * ti)) * 4096) / 4096)
        b = np.float32(np.round((-2.0 * ti) * 4096) / 4096)
        sd = np.zeros((1, env.model.nsensordata), np.float32)
        sd[0, adr_a], sd[0, adr_b] = a, b
        godo.launch_advance(c, s, wp.array(sd, device="cpu"), quat, vec, vec_g)
        data.sensordata[adr_a], data.sensordata[adr_b] = a, b
        wa, wb = odo._wheel_rates(data, odo.odo_dt)
        assert float(s.w_diff.numpy()[0]) == pytest.approx(BELT * (wa - wb), rel=1e-4, abs=1e-3)
