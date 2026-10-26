"""Wheel play (`wheel_play:`): absent from bike_params, so the default model is
untouched; configured, it adds limited hinges sized from travel at the ground."""
import numpy as np
import pytest

from aow_sim.build_model import build_model, load_params

pytestmark = pytest.mark.geometry

PLAY = {"front_axle": {"travel_mm": 2.5}, "headset": {"travel_mm": 2.5},
        "rear_axle": {"travel_mm": 2.5}}


def test_absent_or_zero_play_builds_the_same_model():
    """No key, an empty block and zero travel are all the rigid bike: the
    play bodies are not inserted, so joint addresses (which the deploy bundle
    pins) do not move."""
    p = load_params()
    ref = build_model(p)
    for play in ({}, {"front_axle": {"travel_mm": 0}, "rear_axle": {"travel_mm": 0}}):
        m = build_model({**p, "wheel_play": play})
        assert (m.nq, m.nv, m.nbody) == (ref.nq, ref.nv, ref.nbody)
        np.testing.assert_array_equal(m.qpos0, ref.qpos0)
        np.testing.assert_array_equal(m.body_mass, ref.body_mass)


def test_travel_at_the_ground_sets_each_half_angle():
    p = load_params()
    m = build_model({**p, "wheel_play": PLAY})
    r_rear, r_front = p["omni_wheel"]["outer_radius"], p["bike"]["front_wheel"]["radius"]
    for name, lever in (("rear_axle_play_yaw", r_rear), ("rear_axle_play_camber", r_rear),
                        ("front_axle_play_yaw", r_front), ("front_axle_play_camber", r_front)):
        j = m.joint(name)
        assert m.jnt_limited[j.id]
        np.testing.assert_allclose(j.range, [-1.25e-3 / lever, 1.25e-3 / lever], rtol=1e-9)
    # the headset's lever is its pivot's height above the ground, longer than a wheel radius
    h = m.joint("headset_play_tilt").range[1]
    assert 0 < h < 1.25e-3 / r_front
    # the wheels still hang off the play: the carried bodies' parents moved
    parent = lambda b: m.body(int(m.body_parentid[m.body(b).id])).name
    assert parent("aow_hub") == "rear_axle_play"
    assert parent("front_wheel") == "front_axle_play"
    assert parent("front_axle_play") == "headset_play"


def test_play_adds_no_real_mass():
    p = load_params()
    a, b = build_model(p), build_model({**p, "wheel_play": PLAY})
    assert b.body_subtreemass[1] - a.body_subtreemass[1] < 1e-3


def test_the_stays_swing_the_rear_about_their_root():
    """rear_stays: two hinges through the midpoint of the chainstays' front
    ends, above and ahead of the axle; the hub hangs off them, then off the
    axle play. `travel_mm_<hinge>` overrides one hinge."""
    from aow_sim.build_model import _stays_hinges
    p = load_params()
    root, levers, axes = _stays_hinges(p)
    assert root[0] > 0 and root[2] > 0 and abs(root[1]) < 1e-9
    m = build_model({**p, "wheel_play": {"rear_stays": {"travel_mm": 2.5, "travel_mm_vertical": 1.0},
                                         "rear_axle": {"travel_mm": 2.5}}})
    np.testing.assert_allclose(m.joint("rear_stays_play_lateral").range[1], 1.25e-3 / levers["lateral"])
    np.testing.assert_allclose(m.joint("rear_stays_play_vertical").range[1], 0.5e-3 / levers["vertical"])
    np.testing.assert_allclose(m.jnt_pos[m.joint("rear_stays_play_lateral").id], root, atol=1e-9)
    parent = lambda b: m.body(int(m.body_parentid[m.body(b).id])).name
    assert parent("rear_axle_play") == "rear_stays_play"
    assert parent("aow_hub") == "rear_axle_play"


def test_an_rl_config_resolves_and_records_its_play(tmp_path):
    """`env.wheel_play` (drivetrain_model.from_env_config): absent leaves the
    params' own, a path and its dict resolve alike and idempotently, false
    strips it; the move yaml records it and a loaded policy carries it back
    into the env overrides, so replay builds the plant it trained on."""
    import yaml
    from aow_sim.control.general_env import _load_rl_config
    from aow_sim.control.general_spec import policy_env_overrides
    from aow_sim.drivetrain_model import from_env_config
    from aow_sim.params import plant_digest
    from aow_sim.train_general_rl import move_doc

    p = load_params()
    assert from_env_config(p, {}) == from_env_config(p, {"wheel_play": None})
    assert "wheel_play" not in from_env_config(p, {})
    f = tmp_path / "play.yaml"
    f.write_text(yaml.safe_dump({"note": "x", "wheel_play": PLAY}))
    by_path = from_env_config(p, {"wheel_play": str(f)})
    assert by_path == from_env_config(p, {"wheel_play": PLAY})
    assert by_path["wheel_play"] == PLAY
    assert from_env_config(by_path, {"wheel_play": str(f)}) == by_path
    assert plant_digest(by_path) != plant_digest(p)
    assert "wheel_play" not in from_env_config(by_path, {"wheel_play": False})
    # a policy trained without play is judged on whatever the caller built
    assert from_env_config(by_path, {"wheel_play": None})["wheel_play"] == PLAY

    cfg = _load_rl_config("config/rl/rl_general_v4.yaml")
    doc = move_doc("x_smoke", from_env_config(p, cfg["env"]), cfg, {})
    assert doc["wheel_play"] == cfg["env"]["wheel_play"]

    class Pol:
        wheel_play = doc["wheel_play"]
    assert policy_env_overrides(Pol())["wheel_play"] == doc["wheel_play"]
    assert policy_env_overrides(object())["wheel_play"] is None
