"""The style bit: one command entry that asks the general policy to turn in
place around a held front wheel (general_spec.wheel_err).

`obs_style` appends (style, sin, cos of the wheel error) to the observation;
GeneralEnv draws the bit with every command (p = style_p x difficulty), lets
it act only at a zero speed command, latches the front wheel's world heading
on its rising edge and pays w_style for staying near it; the move yaml
records it; drive.py forms the same three entries; the Warp kernels do the
same on the GPU. What invalidates these: the general obs layout
(general_spec), the style draw, gate, latch or reward in either env, the
move yaml's flag.
"""
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from aow_sim.control import general_spec as gs
from aow_sim.control.steer import wheel_heading

pytestmark = pytest.mark.spec

CFG = "config/rl/rl_general_v3_style.yaml"
RAKE = np.deg2rad(15.0)
STYLE = ("style", "sin_wheel_err", "cos_wheel_err")


def _cfg(**env):
    with open(CFG) as f:
        cfg = yaml.safe_load(f)
    cfg["env"].update(env)
    return cfg


def test_layout_appends_three_entries_last():
    base = gs.obs_layout(obs_pitch=True)
    assert gs.obs_layout(obs_pitch=True, obs_style=True) == base + STYLE
    assert gs.obs_dim_for(obs_pitch=True, obs_style=True) == 20
    assert len(gs.OBS_NAMES) == len(gs.OBS_MIRROR_PARITY)
    par = dict(zip(gs.OBS_NAMES, gs.OBS_MIRROR_PARITY))
    assert (par["style"], par["sin_wheel_err"], par["cos_wheel_err"]) == (1, -1, 1)
    assert par["sin_wheel_err"] == par["sin_psi_err"]     # yaw-like, like psi_err


def test_policy_flags_default_off_for_old_exports():
    old = SimpleNamespace(obs_pitch=True)            # a move before the field
    assert gs.policy_flags(old)["obs_style"] is False
    assert gs.obs_layout_for(old) == gs.obs_layout(obs_pitch=True)
    new = SimpleNamespace(obs_pitch=True, obs_style=True)
    assert gs.policy_env_overrides(new)["obs_style"] is True
    assert gs.obs_layout_for(new)[-3:] == STYLE


def test_build_obs_appends_bit_sin_cos():
    args = (0.1, 0.0, 0.0, 0.2, 0.0, 0.3, 0.0, 0.5, 0.0, 0.4, np.zeros(3))
    plain = gs.build_obs(*args, pitch=(0.01, 0.0))
    o = gs.build_obs(*args, pitch=(0.01, 0.0), style=(1.0, 0.5))
    assert o.shape == (20,)
    np.testing.assert_array_equal(o[:17], plain)
    np.testing.assert_allclose(o[17:], [1.0, np.sin(0.5), np.cos(0.5)], rtol=1e-6)
    off = gs.build_obs(*args, pitch=(0.01, 0.0), style=(0.0, 0.0))
    np.testing.assert_array_equal(off[17:], [0.0, 0.0, 1.0])


@pytest.mark.parametrize("steer", [0.0, 0.4, 1.3, np.pi / 2, 2.8, -4.0])
def test_wheel_err_is_the_wheels_world_heading_against_the_latch(steer):
    psi, latch = 0.7, -0.2
    want = psi + wheel_heading(steer, RAKE) - latch
    got = gs.wheel_err(psi, steer, latch, RAKE)
    assert np.isclose(np.cos(got), np.cos(want)) and np.isclose(np.sin(got), np.sin(want))
    assert -np.pi < got <= np.pi
    # a pivot: the chassis turns, the steer counter-turns, the wheel holds
    lat = gs.wheel_world_heading(0.0, 0.0, RAKE)
    assert gs.wheel_err(np.pi / 2, -np.pi / 2, lat, RAKE) == pytest.approx(0.0, abs=1e-12)


def test_a_steered_half_turn_is_not_free():
    """mod 2*pi, not mod pi: the wheel is symmetric, so mod pi a steered 180
    deg spin would come back to its latch."""
    lat = gs.wheel_world_heading(0.0, 0.0, RAKE)
    assert abs(gs.wheel_err(np.pi, 0.0, lat, RAKE)) == pytest.approx(np.pi)


def test_active_only_at_zero_speed():
    assert gs.style_active(1, (0.0, 0.0))
    assert not gs.style_active(1, (0.3, 0.0))
    assert not gs.style_active(1, (0.0, -0.2))
    assert not gs.style_active(0, (0.0, 0.0))


@pytest.fixture(scope="module")
def env():
    from aow_sim.control.general_env import GeneralEnv
    return GeneralEnv(None, _cfg(), seed=0)


def _reset(e, bit, v=(0.0, 0.0), dpsi=1.5):
    return e.reset(seed=7, options={"v_cmd": v, "psi_cmd_rel": dpsi, "style": bit})[0]


def test_env_latches_and_pays_for_a_held_wheel(env):
    """Same seed, same actions, bit 0 vs 1 on an in-place turn: the physics is
    identical, the obs carries (bit, sin e, cos e) against the latch taken at
    reset, and the rewards differ by exactly w_style x exp(-(e/sigma)^2)."""
    from aow_sim.control.general_env import GeneralEnv
    from aow_sim.control.balance import extract_state
    other = GeneralEnv(None, _cfg(), seed=0)
    acts = np.random.default_rng(3).uniform(-0.8, 0.8, (25, 3))
    o0, o1 = _reset(env, 0.0), _reset(other, 1.0)
    np.testing.assert_array_equal(o0[17:], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(o1[17:], [1.0, 0.0, 1.0], atol=1e-7)   # e = 0 at the latch
    steer0 = float(other.data.qpos[other._sj])
    assert other._latch_true == pytest.approx(gs.wheel_world_heading(
        extract_state(other.data, other._p0).yaw, steer0, other._rake))
    moved = 0.0
    for a in acts:
        (o0, r0, *_, i0), (o1, r1, *_, i1) = env.step(a), other.step(a)
        np.testing.assert_array_equal(o0[:17], o1[:17])
        e_obs = gs.wheel_err(other._psi, float(other.data.qpos[other._sj]),
                             other._latch_obs, other._rake)
        np.testing.assert_allclose(o1[17:], [1.0, np.sin(e_obs), np.cos(e_obs)],
                                   atol=1e-6)
        e = np.radians(i1["wheel_err_deg"])
        assert r1 - r0 == pytest.approx(
            other.w_style * np.exp(-(e / other.sigma_style) ** 2), abs=1e-9)
        assert i0["wheel_err_deg"] == 0.0
        moved = max(moved, abs(e))
    assert moved > np.radians(2)            # the actions really moved the wheel


def test_the_gate_keeps_it_off_while_moving(env):
    o = _reset(env, 1.0, v=(0.6, 0.0), dpsi=0.0)
    assert env._style == 0.0
    np.testing.assert_array_equal(o[17:], [0.0, 0.0, 1.0])


def test_latch_only_on_the_rising_edge(env):
    _reset(env, 1.0)
    lat = env._latch_obs
    for a in np.random.default_rng(4).uniform(-0.8, 0.8, (10, 3)):
        env.step(a)
    env._set_style(1)                      # on -> on: kept
    assert env._latch_obs == lat
    env._set_style(0)
    env._set_style(1)                      # off -> on: re-taken where the wheel is
    assert env._latch_obs == pytest.approx(gs.wheel_world_heading(
        env._psi, float(env.data.qpos[env._sj]), env._rake))


def test_draw_follows_the_curriculum(env):
    _reset(env, 0.0)                       # an in-place command: the gate is open
    rng = np.random.default_rng(0)
    for d, lo, hi in ((0.0, 0.0, 0.0), (0.5, 0.20, 0.30), (1.0, 0.45, 0.55)):
        env.set_difficulty(d)
        on = []
        for _ in range(2000):
            env._style = 0.0
            env._sample_style(rng)
            on.append(env._style)
        assert set(on) <= {0.0, 1.0}
        assert lo <= np.mean(on) <= hi, (d, np.mean(on))


def test_style_without_obs_style_is_refused():
    from aow_sim.control.general_env import GeneralEnv
    with pytest.raises(ValueError, match="obs_style"):
        GeneralEnv(None, _cfg(obs_style=False), seed=0)


def test_v3_is_untouched_by_the_feature():
    """No obs_style: 17 wide, and the bit is never drawn (no rng consumed)."""
    from aow_sim.control.general_env import GeneralEnv
    with open("config/rl/rl_general_v3.yaml") as f:
        e = GeneralEnv(None, yaml.safe_load(f), seed=0)
    assert e.obs_dim == 17 and "style" not in e.obs_layout
    rng = np.random.default_rng(5)
    state = rng.bit_generator.state
    e._sample_style(rng)
    assert rng.bit_generator.state == state and e._style == 0.0


def test_move_doc_records_the_flag():
    from aow_sim.build_model import load_params
    from aow_sim.train_general_rl import move_doc
    doc = move_doc("general_style_smoke", load_params(), _cfg(), {})
    assert doc["obs_style"] is True and tuple(doc["obs_layout"][-3:]) == STYLE
    with open("config/rl/rl_general_v3.yaml") as f:
        plain = move_doc("general_plain_smoke", load_params(), yaml.safe_load(f), {})
    assert plain["obs_style"] is False and "style" not in plain["obs_layout"]


def test_warp_kernels_match_the_spec():
    """gpu/obs_reward.py's style entries and term against wheel_err, on Warp's
    CPU device: one state, launched with the bit 0 and 1 and given latches."""
    wp = pytest.importorskip("warp")
    pytest.importorskip("mujoco_warp")
    from aow_sim.control.general_env import GeneralEnv
    from aow_sim.control.balance import extract_state
    from aow_sim.gpu import obs_reward as gor
    wp.config.quiet = True
    wp.init()
    e = GeneralEnv(None, _cfg(), seed=0)
    c = gor.const_from(e)
    assert c.style_on == 1
    e.reset(seed=1)
    for a in np.random.default_rng(2).uniform(-1, 1, (10, 3)):
        e.step(a)
    qpos, qvel = e.data.qpos.copy(), e.data.qvel.copy()
    psi0, lat_obs, lat_true = 0.3, 1.1, -0.4
    f = lambda x: wp.array(np.atleast_1d(np.asarray(x, np.float32)), dtype=float, device="cpu")
    f2 = lambda x: wp.array(np.asarray(x, np.float32)[None], dtype=float, device="cpu")
    i1 = lambda x: wp.array(np.array([x], np.int32), dtype=int, device="cpu")
    out = {}
    for bit in (0.0, 1.0):
        obs = wp.zeros((1, 20), dtype=float, device="cpu")
        rew = wp.zeros(1, dtype=float, device="cpu")
        psi = f(psi0)
        # identity AHRS quaternion and raw_prev 0: the heading update adds 0
        gor.launch(c, f2(qpos), f2(qvel), f2([1.0, 0, 0, 0]), f2(np.zeros(3)),
                   f(0.0), f(0.0), f(0.0), f2(np.zeros(2)), f(0.0), f(bit),
                   f(lat_obs), f(lat_true), psi, f(0.0), f2(np.zeros(3)),
                   f2(np.zeros(3)), i1(0), f(0.0), i1(0), obs, rew, i1(0), i1(0))
        out[bit] = (obs.numpy()[0], float(rew.numpy()[0]))
    steer = float(qpos[e._sj])
    e_obs = gs.wheel_err(psi0, steer, lat_obs, e._rake)
    np.testing.assert_allclose(out[1.0][0][17:], [1.0, np.sin(e_obs), np.cos(e_obs)],
                               atol=1e-5)
    np.testing.assert_allclose(out[0.0][0][17:], [0.0, 0.0, 1.0], atol=1e-7)
    e_true = gs.wheel_err(extract_state(e.data, e._p0).yaw, steer, lat_true, e._rake)
    want = e.w_style * np.exp(-(e_true / e.sigma_style) ** 2)
    assert out[1.0][1] - out[0.0][1] == pytest.approx(want, rel=1e-4, abs=1e-5)


def _cfg_mult():
    cfg = _cfg()
    cfg["reward"]["style_form"] = "mult"
    return cfg


def test_mult_pays_only_as_far_as_the_heading_is_reached():
    """style_form mult (run 2): the bit multiplies the heading term, so on an
    in-place turn it pays w_head x r_head x w_style x r_style -- ~0 while the
    heading is far off, i.e. refusing the turn earns nothing from it."""
    from aow_sim.control.general_env import GeneralEnv
    e0, e1 = GeneralEnv(None, _cfg_mult(), seed=0), GeneralEnv(None, _cfg_mult(), seed=0)
    assert e1.style_form == "mult"
    acts = np.random.default_rng(3).uniform(-0.8, 0.8, (25, 3))
    for dpsi in (1.5, 0.1):
        _reset(e0, 0.0, dpsi=dpsi), _reset(e1, 1.0, dpsi=dpsi)
        for a in acts:
            (_o0, r0, *_), (_o1, r1, *_, i1) = e0.step(a), e1.step(a)
            r_head = np.exp(-(np.radians(i1["head_err_deg"]) / e1.sigma_psi) ** 2)
            r_style = np.exp(-(np.radians(i1["wheel_err_deg"]) / e1.sigma_style) ** 2)
            assert r1 - r0 == pytest.approx(
                e1.rw["w_head"] * r_head * e1.w_style * r_style, abs=1e-9)
            if dpsi == 1.5 and i1["head_err_deg"] > 60:
                assert r1 - r0 < 0.01       # far from heading: the bit pays ~nothing


def test_style_form_is_checked():
    from aow_sim.control.general_env import GeneralEnv
    cfg = _cfg()
    cfg["reward"]["style_form"] = "multiply"
    with pytest.raises(ValueError, match="style_form"):
        GeneralEnv(None, cfg, seed=0)


def test_warp_kernel_mult_matches_the_cpu_env():
    """gpu/obs_reward.py under style_form mult: the bit's reward difference
    is w_head x r_head x w_style x r_style. r_head is on the bike's OWN
    heading psi (as GeneralEnv's), here psi0 unchanged by the launch;
    r_style on truth."""
    wp = pytest.importorskip("warp")
    pytest.importorskip("mujoco_warp")
    from aow_sim.control.general_env import GeneralEnv
    from aow_sim.control.balance import extract_state
    from aow_sim.gpu import obs_reward as gor
    wp.config.quiet = True
    wp.init()
    e = GeneralEnv(None, _cfg_mult(), seed=0)
    c = gor.const_from(e)
    assert c.style_mult == 1
    e.reset(seed=1)
    for a in np.random.default_rng(2).uniform(-1, 1, (10, 3)):
        e.step(a)
    qpos, qvel = e.data.qpos.copy(), e.data.qvel.copy()
    psi0, lat_obs, lat_true, psi_cmd = 0.3, 1.1, -0.4, 0.5
    f = lambda x: wp.array(np.atleast_1d(np.asarray(x, np.float32)), dtype=float, device="cpu")
    f2 = lambda x: wp.array(np.asarray(x, np.float32)[None], dtype=float, device="cpu")
    i1 = lambda x: wp.array(np.array([x], np.int32), dtype=int, device="cpu")
    rew = {}
    for bit in (0.0, 1.0):
        obs = wp.zeros((1, 20), dtype=float, device="cpu")
        r = wp.zeros(1, dtype=float, device="cpu")
        gor.launch(c, f2(qpos), f2(qvel), f2([1.0, 0, 0, 0]), f2(np.zeros(3)),
                   f(0.0), f(0.0), f(0.0), f2(np.zeros(2)), f(psi_cmd), f(bit),
                   f(lat_obs), f(lat_true), f(psi0), f(0.0), f2(np.zeros(3)),
                   f2(np.zeros(3)), i1(0), f(0.0), i1(0), obs, r, i1(0), i1(0))
        rew[bit] = float(r.numpy()[0])
    yaw = extract_state(e.data, e._p0).yaw
    steer = float(qpos[e._sj])
    e_true = gs.wheel_err(yaw, steer, lat_true, e._rake)
    psi_err = np.arctan2(np.sin(psi_cmd - psi0), np.cos(psi_cmd - psi0))
    want = (e.rw["w_head"] * np.exp(-(psi_err / e.sigma_psi) ** 2)
            * e.w_style * np.exp(-(e_true / e.sigma_style) ** 2))
    assert want > 0.05                      # the case is not a trivial zero
    assert rew[1.0] - rew[0.0] == pytest.approx(want, rel=1e-3, abs=1e-5)
