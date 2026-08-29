"""The two modes the bike is evaluated in, and the collapse between them.

WHAT INVALIDATES THIS FILE: a change to the sensor models (`sim_ahrs.py`,
`sim_odometry.py`), to the eval grid or `_score`, or a repoint of
`control.general_move`. Marker `contact` -- it builds a model and runs
rollouts, so it costs seconds, not milliseconds.

WHY IT EXISTS. Until 2026-08-28 nothing in the suite ran a policy against the
sensors the bike will actually have, and the gap it left is not subtle: the
truth-trained default scored 0.808 on the eval grid and 0.044 with the onboard
velocity estimate and a TM151 attitude in the loop. A regression that swapped
the default back to a truth-trained policy, or that silently disabled the
sensor models, would have shown a completely green suite. `test_sensor_mode_
catches_a_truth_trained_policy` is that specific hole, and it is the reason
the file is worth its seconds.

WHY THE ASSERTION IS SURVIVAL AND NOT RMS AGAINST TRUTH. This was measured the
wrong way round once already: an open-loop accuracy objective (estimator error
against MuJoCo truth) selected a WORSE estimator, because an estimator can be
more accurate on average and worse exactly where the controller needs it. The
question is always whether the bike stays up while doing what it was told, so
the assertion is survival over commands, with `track_geo` as a secondary floor.

FOUR COMMANDS, NOT TWENTY. The full grid is 20 commands at 15 s -- about 30 s
serial, which is a third of the `contact` marker's whole budget for one file.
The subset (hold, forward, reverse, 90 degree turn) reproduces every number
this file asserts and separates the modes by more than the thresholds' margin,
so it is a REGRESSION GUARD rather than a measurement. The measurement is
`analysis/chatter.py` and the tables in docs/status.md; do not re-derive
policy standings from here.

THE THRESHOLDS ARE FLOORS UNDER MEASURED VALUES, deliberately slack. Measured
2026-08-28 on this subset:

    mode / policy                          surv  track_geo   asserted
    IDEAL   smooth_diff_pi on truth        1.00      0.951   >= 0.90
    SENSOR  odo_ahrs tm151 tau 2.0         1.00      0.859   >= 0.78
    SENSOR  odo_ahrs tm151 tau 0.19        1.00      0.849   >= 0.78
    GUARD   smooth_diff_pi in sensor mode  0.25      0.493   surv <= 0.50

A tighter floor would make this a policy-quality test, which it is not: it
exists to catch the modes being wired up wrong, and a real quality regression
shows up as a fall long before it shows up as 0.02 of track.
"""

from pathlib import Path

import numpy as np
import pytest

from aow_sim.build_model import load_params
from aow_sim.control.flick import load_move
from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.general_spec import policy_env_overrides
from aow_sim.train_general_rl import _eval_episodes, eval_cmds

# Model build plus rollouts. See `pytest --markers`.
pytestmark = pytest.mark.contact

REPO = Path(__file__).resolve().parents[1]

# The ceiling reference: trained on MuJoCo truth, and what "if the sensing
# were perfect" is worth. NOT the default any more -- see bike_params.yaml.
IDEAL_POLICY = "general_rl_smooth_diff_pi"
# Trained against the onboard velocity estimate AND a TM151 attitude, and what
# `control.general_move` names as of 2026-08-28.
SENSOR_POLICY = "general_rl_odo_ahrs"


@pytest.fixture(scope="module")
def params():
    return load_params()


def _subset(v_max: float):
    """hold, forward, reverse, 90-degree turn -- indices 0, 3, 7, 1 of the
    full grid, so a row here is directly comparable to the same row of
    `analysis/chatter.py` rather than being a differently-scaled command."""
    g = eval_cmds(v_max)
    return [g[0], g[3], g[7], g[1]]


def _eval(name, params, *, ahrs="none", encoder=None, tau=2.0,
          force_odometry=False):
    """Run one policy in one sensor mode over the subset.

    `force_odometry` overrides the policy's own declaration, which is the only
    way to ask the DEPLOYMENT question of a truth-trained policy: left to
    itself it declares `obs_odometry` false and is handed MuJoCo truth, i.e. a
    cleaner signal than the Pi will ever give it.
    """
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    # Randomization OFF: this is a fixed point, and the AHRS parameter ranges
    # under `randomization:` are a TRAINING distribution. Leaving it on would
    # make every threshold here a distribution rather than a number.
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    if ahrs != "none":
        cfg = {**cfg, "env": {**cfg["env"], "ahrs_level": ahrs,
                              "ahrs_tau_s": tau}}
    pol = load_move(name)
    if encoder is not None:
        pol.odometry_encoder = encoder
    over = policy_env_overrides(pol)
    if force_odometry:
        over = {**over, "obs_odometry": True}
    env = GeneralEnv(params, {**cfg, "env": {**cfg["env"], **over}})
    scales = np.array([pol.bounds.steer_rate_max, pol.bounds.hub_max,
                       pol.bounds.diff_max,
                       max(pol.bounds.wing_rate_max, 1e-9)])

    def act(obs):
        a = np.asarray(pol.action(obs), float)
        return (a / scales[:len(a)])[:env.action_space.shape[0]]

    m, _ = _eval_episodes(env, act, _subset(cfg["env"]["v_max"]))
    return m


def test_ideal_mode_is_the_ceiling(params):
    """Truth sensors, truth-trained policy: the number the sensor modes are
    measured against. If THIS moves, the plant moved and every sensor result
    in docs/status.md is provisional -- so it is asserted separately rather
    than folded into the sensor test."""
    m = _eval(IDEAL_POLICY, params)
    assert m["survive_rate"] == 1.0
    assert m["track_geo"] >= 0.90, m["track_geo"]


@pytest.mark.parametrize("tau", [2.0, 0.19])
def test_sensor_mode_survives_on_the_policy_that_trained_for_it(params, tau):
    """The sensors the bike will actually have: quantised encoders through the
    Pi's own RateFilter, and a TM151 attitude.

    BOTH TAUS, because the policy trained at 2.0 and the real part measures
    0.19 -- and on the full grid that difference costs it 0.672 / 1.00 ->
    0.570 / 0.95. The subset does not resolve that (both are 1.00 here), which
    is the honest limit of a 4-command guard: it catches the modes being
    broken, not the specialisation. `general_rl_odo_ahrs_rand` is the run that
    addresses the specialisation.
    """
    m = _eval(SENSOR_POLICY, params, ahrs="tm151", encoder="counts", tau=tau)
    assert m["survive_rate"] == 1.0
    assert m["track_geo"] >= 0.78, m["track_geo"]


def test_sensor_mode_catches_a_truth_trained_policy(params):
    """THE HOLE THIS FILE WAS WRITTEN FOR.

    A policy trained on MuJoCo truth, handed the onboard estimate and a TM151
    attitude, collapses -- 0.808 / 1.00 to 0.044 / 0.15 on the full grid, and
    survival 0.25 on this subset. Nothing in the suite caught that, so a
    repoint of `control.general_move` back to a truth-trained export would
    have been a completely green change.

    Asserted as a CEILING on survival rather than a floor on anything: the
    claim is that sensor mode is genuinely hard, and a version of these models
    that a truth-trained policy sails through is a version that is not
    modelling the sensors.
    """
    m = _eval(IDEAL_POLICY, params, ahrs="tm151", encoder="counts",
              force_odometry=True)
    assert m["survive_rate"] <= 0.50, (
        f"a truth-trained policy survived {m['survive_rate']:.2f} of the "
        "sensor-mode subset. Either the sensor models stopped being applied "
        "or the grid stopped being hard -- both make every sensor number in "
        "docs/status.md meaningless.")


def test_the_configured_default_is_a_sensor_trained_policy(params):
    """`control.general_move` must name a policy that declares `obs_odometry`.

    Cheap, and it is the specific regression the collapse above makes
    expensive: the default drives the BIKE (hw/run_bike reads this key), so a
    truth-trained default is a policy that falls on first power-on. Deliberately
    a property of the export -- 'was it trained on the estimate' -- and not a
    name comparison, so a better sensor-trained export can replace it freely.
    """
    name = params["control"]["general_move"]
    pol = load_move(name)
    assert getattr(pol, "obs_odometry", False), (
        f"control.general_move names {name!r}, which was trained on MuJoCo "
        "truth. See test_sensor_mode_catches_a_truth_trained_policy.")


def test_every_orientation_channel_in_the_obs_comes_from_the_sensor(params):
    """No observation entry may read MuJoCo truth while an AHRS is configured.

    `obs_pitch` did exactly that until 2026-08-29: `_obs` decoded the corrupted
    quaternion, DISCARDED its pitch component, and fed the policy `s.pitch`
    from the model — a perfect signal the bike does not have, in the one
    workstream whose entire point is to remove those. Roll was routed
    correctly, so nothing looked wrong. It was found by asking where obs pitch
    came from, one config edit before a 12M-step run would have trained
    against it.

    Asserted as "closer to the sensor than to truth" rather than on an
    absolute error, so it cannot be defeated by the error model getting
    quieter — the point is WHICH SOURCE the entry follows, not how noisy it is.
    """
    import numpy as np

    from aow_sim.control.balance import extract_state
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    from aow_sim.sim_ahrs import rpy_from_quat

    cfg = _load_rl_config(REPO / "config" / "rl_general_odo_ahrs_pitch.yaml")
    env = GeneralEnv(params, cfg)
    lay = env.obs_layout
    assert "pitch" in lay, "config no longer sets obs_pitch; test is vacuous"

    obs, _ = env.reset(seed=3, options={"v_cmd": (0.5, 0.0),
                                        "psi_cmd_rel": 0.0, "difficulty": 1.0})
    zero = np.zeros(env.action_space.shape[0], np.float32)
    err = {k: [0.0, 0.0] for k in ("roll", "pitch")}
    series = {k: [] for k in ("roll", "pitch", "roll_truth", "pitch_truth")}
    for _ in range(200):
        obs, *_ = env.step(zero)
        s = extract_state(env.data, env._p0)
        a_roll, a_pitch_textbook, _yaw = rpy_from_quat(env._ahrs.latest("ahrs_quat"))
        # NEGATED: extract_state reports pitch as arcsin(R[2,0]), MINUS the
        # textbook ZYX pitch rpy_from_quat returns (balance.py:84). Comparing
        # against the un-negated value is how a sign flip in _obs would pass
        # this test while feeding the policy the wrong convention.
        a_pitch = -a_pitch_textbook
        for k, truth, sensor in (("roll", s.roll, a_roll),
                                 ("pitch", s.pitch, a_pitch)):
            err[k][0] += abs(obs[lay.index(k)] - truth)
            err[k][1] += abs(obs[lay.index(k)] - sensor)
            series[k].append(obs[lay.index(k)])
            series[k + "_truth"].append(truth)

    # A SIGN FLIP IS NOT CAUGHT by "closer to the sensor than to truth" alone
    # -- a negated channel is far from both. Assert the sign explicitly.
    for k in ("roll", "pitch"):
        c = np.corrcoef(np.array(series[k]), np.array(series[k + "_truth"]))[0, 1]
        assert c > 0.2, (
            f"obs `{k}` is ANTI-correlated with the true value (corr {c:+.3f})"
            f" -- the sensor convention is inverted relative to extract_state,"
            f" which is what hw/state.set_orientation feeds on the bike.")

    for k, (to_truth, to_sensor) in err.items():
        assert to_sensor < to_truth, (
            f"obs `{k}` tracks MuJoCo TRUTH, not the AHRS: mean error "
            f"{np.degrees(to_truth / 200):.3f} deg against truth and "
            f"{np.degrees(to_sensor / 200):.3f} deg against the sensor. An "
            f"observation entry reading truth while an AHRS is configured is "
            f"a signal the bike will not have.")
