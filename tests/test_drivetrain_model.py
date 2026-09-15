"""The opt-in detailed drivetrain: config/drivetrain_model.yaml + drivetrain_model.py.

Two halves, both pinned. The overlay must be INVISIBLE when absent -- every
existing plant, policy and digest depends on that -- and each part must do the
one thing it was fitted to do: the servo tracks and holds through its integral,
and the detent turns a slow diff move into stick-slip, which is the reason it
exists (without it the replayed creep grid never stalls at all).
"""

import math

import mujoco
import numpy as np
import pytest

from aow_sim import drivetrain_model as dm
from aow_sim.build_model import build_model, build_spec, load_params

pytestmark = pytest.mark.drivetrain


@pytest.fixture(scope="module")
def params():
    return load_params()


def _testbed(params, without=()):
    p = dm.with_drivetrain(params, without=without)
    m = build_model(p, variant="testbed")
    return p, m, mujoco.MjData(m), dm.DrivetrainSim.attach(m, p)


def _run(m, d, sim, a, b, seconds):
    """Hold input-shaft speed commands (a, b) and return per-step ring/hub rates."""
    ia, ib = m.actuator("drive_a").id, m.actuator("drive_b").id
    hub = m.joint("hub_spin").dofadr[0]
    ring = m.joint("ring_spin").dofadr[0]
    out = []
    for _ in range(int(round(seconds / m.opt.timestep))):
        d.ctrl[ia], d.ctrl[ib] = a, b
        sim.pre_step(d)
        mujoco.mj_step(m, d)
        out.append((d.qvel[hub], d.qvel[ring]))
    return np.array(out)


def test_absent_overlay_changes_nothing(params):
    """Overlay loaded with every part off compiles to the identical model."""
    off = dm.with_drivetrain(params, without=dm.PARTS)
    assert build_spec(params).to_xml() == build_spec(off).to_xml()
    assert dm.DrivetrainSim.attach(build_model(off), off) is None
    assert dm.base_params(params) is params


def test_servo_structure(params):
    p = dm.with_drivetrain(params, without=("detent", "roller_slop"))
    m = build_model(p)
    base = build_model(params)
    assert m.nu == base.nu + 2, "two torque actuators appended"
    assert m.na == 0, "the native velocity-PI integrators are gone"
    for n in ("drive_a", "drive_b", "steer"):
        assert m.actuator(n).id == base.actuator(n).id, "no existing index moved"
    assert m.actuator("drive_a_motor").id == base.nu
    assert m.actuator_gainprm[m.actuator("drive_a").id][0] == 0.0
    s, belt = p[dm.KEY]["servo"], params["drivetrain"]["belt_ratio"]
    dof = m.joint("input_a_spin").dofadr[0]
    assert m.dof_armature[dof] == pytest.approx(s["rotor_inertia"] / belt ** 2)
    # Coulomb friction is applied by DrivetrainSim, not as a joint frictionloss.
    assert m.dof_frictionloss[dof] == pytest.approx(base.dof_frictionloss[dof])


def test_slop_structure(params):
    p = dm.with_drivetrain(params, without=("servo", "detent"))
    m, base = build_model(p), build_model(params)
    n = params["omni_wheel"]["n_axles"]
    assert m.neq == base.neq - n, "rigid roller couplings removed"
    assert m.ntendon == base.ntendon + n
    half = math.radians(p[dm.KEY]["roller_slop"]["half_play_deg"])
    t = m.tendon("slop_roller_spin_0").id
    assert m.tendon_range[t] == pytest.approx([-half, half])


def test_servo_tracks_a_common_command(params):
    p, m, d, sim = _testbed(params, without=("detent", "roller_slop"))
    v = 0.2 * 11.1 * params["drivetrain"]["belt_ratio"]   # 20 % no-load, input shaft
    rates = _run(m, d, sim, v, v, 1.5)
    tail = rates[-int(0.5 / m.opt.timestep):, 0]
    assert np.mean(tail) == pytest.approx(v, rel=0.03)
    assert 0.0 < sim.duty[0] < 1.0


def test_integral_holds_against_a_push(params):
    """Goal 0 plus a steady hub torque: a spring, not a drift."""
    p, m, d, sim = _testbed(params, without=("detent", "roller_slop"))
    d.qfrc_applied[m.joint("hub_spin").dofadr[0]] = 0.2
    _run(m, d, sim, 0.0, 0.0, 2.0)
    hub_q = d.qpos[m.joint("hub_spin").qposadr[0]]
    assert abs(d.qvel[m.joint("hub_spin").dofadr[0]]) < 0.05
    assert abs(hub_q) < 0.3


def test_detent_makes_slow_diff_stick(params):
    """1 % diff creep: stalls with the detent, none without it."""
    v = 0.01 * 11.1 * params["drivetrain"]["belt_ratio"]
    stuck = {}
    for label, without in (("on", ("roller_slop",)), ("off", ("detent", "roller_slop"))):
        p, m, d, sim = _testbed(params, without=without)
        rates = _run(m, d, sim, v, -v, 4.0)[int(1.0 / m.opt.timestep):, 1]
        stuck[label] = float(np.mean(np.abs(rates) < 0.25 * v))
    assert stuck["on"] > 0.2, stuck
    assert stuck["off"] < 0.05, stuck


def test_viewer_rewind_resets_the_loop(params):
    p, m, d, sim = _testbed(params, without=("detent", "roller_slop"))
    _run(m, d, sim, 5.0, 5.0, 0.3)
    mujoco.mj_resetData(m, d)
    sim.pre_step(d)
    assert sim.duty == [0.0, 0.0] or max(abs(x) for x in sim.duty) < 1e-9


def test_general_env_runs_the_overlay(params):
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    cfg = _load_rl_config()
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False},
           "env": {**cfg["env"], "ball_prob": 0.0}}
    env = GeneralEnv(dm.with_drivetrain(params), cfg, seed=0)
    assert env._drive is not None
    obs, _ = env.reset(seed=0)
    for _ in range(10):
        obs, *_ = env.step(np.zeros(env.action_space.shape))
    assert np.all(np.isfinite(obs))


def _roller_dev_deg(m, d, k):
    return math.degrees(d.qpos[m.joint("roller_spin_0").qposadr[0]]
                        - k * d.qpos[m.joint("ring_spin").qposadr[0]])


def test_roller_play_is_critically_damped(params):
    """By hand a flicked roller returns to centre with no visible overshoot."""
    p = dm.with_drivetrain(params)
    m = build_model(p, variant="testbed")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    dof = m.joint("roller_spin_0").dofadr[0]
    inertia = d.qM[m.dof_Madr[dof]]
    r = p[dm.KEY]["roller_slop"]
    c = r["damping"] + m.dof_damping[dof]
    zeta = c / (2 * math.sqrt(r["centring_stiffness"] * inertia))
    assert 0.8 <= zeta <= 1.5, zeta


@pytest.mark.parametrize("start", ["flick", "release"])
def test_roller_returns_to_centre_without_overshoot(params, start):
    p, m, d, sim = _testbed(params)
    k = params["drivetrain"]["k_roller"]
    half = p[dm.KEY]["roller_slop"]["half_play_deg"]
    rq, rd = m.joint("roller_spin_0").qposadr[0], m.joint("roller_spin_0").dofadr[0]
    if start == "flick":
        d.qvel[rd] = 20.0
    else:
        d.qpos[rq] += math.radians(half - 0.5)      # held at the end, let go
    devs = []
    for _ in range(int(0.5 / m.opt.timestep)):
        sim.pre_step(d)
        mujoco.mj_step(m, d)
        devs.append(_roller_dev_deg(m, d, k))
    devs = np.array(devs)
    peak = devs.max()
    assert peak > 0.5 * half, "the flick or release did not reach the play"
    assert devs.min() > -0.1 * peak, f"overshoot {devs.min():.2f} deg"
    assert abs(devs[-1]) < 1.0


def test_env_config_selects_the_plant(params):
    env = {"drivetrain_model": True, "drivetrain_without": ["roller_slop"],
           "servo_gains": [400, 1920]}
    p = dm.from_env_config(params, env)
    assert p[dm.KEY]["servo"]["velocity_p_gain"] == 400
    assert not dm.enabled(p, "roller_slop")
    assert dm.from_env_config(params, {}) is params
    forced = dm.with_drivetrain(params, without=("detent",))
    assert dm.from_env_config(forced, env) is forced, "params carrying a plant win"
    again = dm.from_env_config(params, {"drivetrain_model": p[dm.KEY]})
    assert again[dm.KEY] == p[dm.KEY], "an exported overlay dict round-trips"


def test_policy_carries_its_drivetrain():
    from types import SimpleNamespace

    from aow_sim.control.general_spec import policy_env_overrides
    assert policy_env_overrides(SimpleNamespace())["drivetrain_model"] is None
    overlay = {"servo": {"enabled": True}}
    got = policy_env_overrides(SimpleNamespace(drivetrain_model=overlay))
    assert got["drivetrain_model"] is overlay


def test_teleop_gains_follow_the_policy_unless_pinned(params):
    import copy
    file_cfg = dm.load()
    p400 = dm.with_drivetrain({}, gains=(400, 1920))[dm.KEY]
    # What launch compiles: a drivetrain flag pins the file, else the startup
    # policy's record, else --servo-gains alone means the file, else ideal.
    assert dm.teleop_base(p400, drivetrain=True) == file_cfg
    assert dm.teleop_base(p400) == p400
    assert dm.teleop_base(None, gains=(200, 1920)) == file_cfg
    assert dm.teleop_base(None) is None

    base = dm.teleop_base(None, drivetrain=True)
    got, notes = dm.teleop_overlay(base, p400)
    assert got["servo"]["velocity_p_gain"] == 400 and not notes
    got, notes = dm.teleop_overlay(base, None)
    assert got["servo"]["velocity_p_gain"] == file_cfg["servo"]["velocity_p_gain"]
    assert any("IDEAL" in n for n in notes)
    got, notes = dm.teleop_overlay(base, p400, gains=(200, 1920))
    assert got["servo"]["velocity_p_gain"] == 200
    assert any("P400" in n for n in notes)
    got, notes = dm.teleop_overlay(None, p400)
    assert got is None and notes, "an ideal session cannot grow a drivetrain"

    soft = copy.deepcopy(p400)
    soft["detent"]["friction"] = 0.05
    got, notes = dm.teleop_overlay(base, soft)
    assert got["detent"]["friction"] == file_cfg["detent"]["friction"]
    assert any("detent.friction" in n for n in notes), "reported, not applied"

    # The swap needs no rebuild: the same compiled model takes either gain.
    m = build_model(dm.with_drivetrain(params), variant="testbed")
    for p_gain in (100, 400):
        g, _ = dm.teleop_overlay(base, dm.with_drivetrain({}, gains=(p_gain, 1920))[dm.KEY])
        sim = dm.DrivetrainSim.attach(m, {**params, dm.KEY: g})
        assert sim.kp == pytest.approx(g["servo"]["kp_per_unit"] * p_gain)


def _randomized_env(params, **rand):
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    cfg = _load_rl_config()
    cfg = {**cfg,
           "randomization": {**cfg["randomization"], "enabled": True, **rand},
           "env": {**cfg["env"], "ball_prob": 0.0, "drivetrain_model": True}}
    return GeneralEnv(params, cfg, seed=0)


def test_drive_randomization_is_off_by_default(params):
    """actuator_frac still draws a battery for steer; the drive stays nominal."""
    env = _randomized_env(params, actuator_frac=0.15)
    for seed in range(3):
        env.reset(seed=seed)
        d = env._drive
        assert d.supply == 1.0
        assert d.fc_in == d._fc_nominal
    assert env._rand.supply_scale != 1.0


def test_drive_randomization_when_asked(params):
    env = _randomized_env(params, actuator_frac=0.15, drive_supply_randomize=True,
                          drive_friction_side_frac=0.3)
    sided = False
    for seed in range(4):
        env.reset(seed=seed)
        d = env._drive
        assert d.supply == pytest.approx(env._rand.supply_scale)
        assert 0.85 <= d.supply <= 1.15
        ra, rb = (d.fc_in[i] / d._fc_nominal[i] for i in (0, 1))
        assert 0.7 <= ra <= 1.3 and 0.7 <= rb <= 1.3
        sided |= abs(ra - rb) > 1e-6
    assert sided, "the two sides are drawn independently"
