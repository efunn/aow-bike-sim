"""The shipped policy flown on the sensors it trained with, and the wheel slip
it produces. Two questions, one set of flights:

  CLOSED LOOP -- does `control.general_move`, handed the onboard velocity
  ESTIMATE and the TM151 attitude model its move yaml says it trained
  against, survive and track the command? This is the objective the
  estimator is selected on (hw/odometry.py: "select estimators on CLOSED-LOOP
  SURVIVAL, not on RMS against truth").

  SLIP -- how far does that policy break the no-slip assumptions the
  estimator rests on? Rear: hub spin x radius against the hub centre's actual
  velocity. Front: the front wheel's velocity along its own axle, which the
  rolling constraint assumes is zero. A tripwire against a recorded baseline,
  so a retrain that works the wheels harder shows up here and says so -- it
  is a property of the policy and the contact model, NOT of the estimator's
  code, which tests/test_hw_odometry.py covers exactly and with no sim.

Replaced 2026-09-21 the old open-loop "estimate vs truth RMS" tests, which
measured the slip above and reported it as estimator error. Measured then, on
the rear wheel, of 51-109 mm/s total error: 5-7 was the estimator's gearing
arithmetic, 2-3 the hub-vs-chassis reference point, and the rest slip.

The slip numbers rest on `friction_sliding` 0.9 and `contact_solimp`, both
GUESS; contact calibration will move them, and the baseline with them.
"""

import mujoco
import numpy as np
import pytest
import yaml

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import extract_state
from aow_sim.control.drive import DriveController
from aow_sim.control.flick import MOVES_DIR
from aow_sim.control.linearize import settle_upright
from aow_sim.sim_ahrs import TAU_ORIENT_S, SimAhrs
from aow_sim.sim_odometry import SimOdometry

# Stepped bike (contact) driven by an exported policy (policy).
# See `pytest --markers` for what each one means.
pytestmark = [pytest.mark.contact, pytest.mark.policy]

MAX_ROLL_DEG = 25.0
N_STEPS = 12000           # 4.8 s at the 4e-4 timestep
WARMUP = 3000             # samples start after 1.2 s
TAIL = 6000               # "where it ended up": the last 2.4 s

# (speed [m/s], yaw rate [rad/s], shove [m/s]). circle_R0.8 is 0.5 / 0.625:
# the command-conditioned policy is asked for a circle by advancing its
# commanded heading.
REGIMES = {
    "standstill": (0.0, 0.0, 0.0),
    "standstill_shoved": (0.0, 0.0, 0.12),
    "straight_0.6": (0.6, 0.0, 0.0),
    "circle_R0.8": (0.5, 0.625, 0.0),
}
MOVING = ["straight_0.6", "circle_R0.8"]

# RMS slip [mm/s], MEDIAN over AHRS seeds SLIP_SEEDS, measured 2026-09-22 on
# general_rl_cmd_curriculum2b flown on its own sensors. A TRIPWIRE, not a spec:
# the assert is "the same median is not more than 1.5x this". Re-measure and
# rewrite when the policy or the contact moves.
#
# A MEDIAN, BECAUSE ONE SEED IS ONE DRAW. It was seed 0 alone until the
# yaw-drift fix moved the AHRS rng stream and straight_0.6 went 67.2 -> 120.8
# with nothing physical changed. Rear slip across 8 seeds spans 30-119
# (standstill) and 61-121 (straight), so a single draw against a 1.5x margin
# was a coin toss; seeds 4 and 7 were already over it before the fix. The
# 5-seed median moved at most 1.22x across that fix (straight 84.5 -> 102.8)
# and at most 1.36x between two disjoint 5-seed sets.
SLIP_BASELINE_POLICY = "general_rl_cmd_curriculum2b"
SLIP_SEEDS = range(5)
REAR_SLIP_BASELINE = {"standstill": 64.6, "standstill_shoved": 75.3,
                      "straight_0.6": 102.8, "circle_R0.8": 115.8}
FRONT_SLIP_BASELINE = {"standstill": 27.2, "standstill_shoved": 37.0,
                       "straight_0.6": 38.8, "circle_R0.8": 48.1}
SLIP_MARGIN = 1.5


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params)


@pytest.fixture(scope="module")
def policy(params):
    name = params["control"].get("general_move", "general_rl")
    path = MOVES_DIR / f"{name}.yaml"
    if not path.exists() or not (MOVES_DIR / f"{name}.npz").exists():
        pytest.skip(f"control.general_move names {name}, which is not exported")
    return name, yaml.safe_load(path.read_text()) or {}


def _fly(params, model, policy, regime, seed=0):
    """One episode of the configured policy on the sensors it trained with.

    Mirrors record.py: the move yaml says which AHRS level / tau and which
    odometry encoder, and the controller sees the ESTIMATE while physics
    keeps the truth. `seed` seeds the AHRS error model AND, in
    standstill_shoved, the shoves.
    """
    name, spec = policy
    speed, yaw_rate, shove = REGIMES[regime]
    dt = model.opt.timestep
    ahrs = None
    if str(spec.get("ahrs_level") or "none") != "none":
        ahrs = SimAhrs(model, params, level=spec["ahrs_level"], seed=seed,
                       tau_orient_s=float(spec.get("ahrs_tau_s", TAU_ORIENT_S)))
    odo = None
    if spec.get("obs_odometry"):
        odo = SimOdometry(model, params, mode="front",
                          encoder=str(spec.get("odometry_encoder") or "ideal"),
                          ahrs=ahrs)

    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    mujoco.mj_forward(model, data)
    ctl = DriveController(params, model)
    ctl.reset(model, data)
    ctl._odometry_active = odo is not None
    ctl._ahrs_active = ahrs is not None
    ctl.engage_general(data, name=name)
    ctl.set_command_polar(speed)

    hub = model.joint("hub_spin").dofadr[0]
    r = float(params["omni_wheel"]["outer_radius"])
    hb, fb = model.body("aow_hub").id, model.body("front_wheel").id
    v6 = np.zeros(6)
    rng = np.random.default_rng(seed)
    rows, max_roll = [], 0.0
    for k in range(N_STEPS):
        if yaw_rate:
            ctl.set_command_polar(speed, psi_cmd=ctl._gen_psi_cmd + yaw_rate * dt)
        if odo is not None:
            with odo.estimated(data, dt):
                ctl.step(model, data)
        elif ahrs is not None:
            ahrs.tick(data, dt)
            with ahrs.estimated(data):
                ctl.step(model, data)
        else:
            ctl.step(model, data)
        if shove and k and k % 3000 == 0:
            data.qvel[1] += rng.uniform(-shove, shove)
        mujoco.mj_step(model, data)
        s = extract_state(data, np.zeros(3))
        max_roll = max(max_roll, abs(s.roll))
        if k >= WARMUP:
            fwd = np.array([np.cos(s.yaw), np.sin(s.yaw)])
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, hb, v6, 0)
            rear_slip = data.qvel[hub] * r - v6[3:5] @ fwd
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, fb, v6, 0)
            axle = data.xmat[fb].reshape(3, 3)[:2, 1]
            front_slip = v6[3:5] @ (axle / (np.linalg.norm(axle) + 1e-12))
            rows.append((s.v_lon, data.qvel[5], s.yaw, rear_slip, front_slip))
    return np.degrees(max_roll), np.array(rows)


_CACHE = {}


def _episode(params, model, policy, regime, seed=0):
    key = (regime, seed)
    if key not in _CACHE:
        _CACHE[key] = _fly(params, model, policy, regime, seed)
    return _CACHE[key]


def _rms_mm(x):
    return 1e3 * float(np.sqrt(np.mean(np.square(x))))


# -- closed loop ------------------------------------------------------------

@pytest.mark.parametrize("regime", list(REGIMES))
def test_policy_survives_on_its_own_sensors(params, model, policy, regime):
    max_roll, _ = _episode(params, model, policy, regime)
    assert max_roll < MAX_ROLL_DEG, (
        f"{policy[0]} fell in {regime} on its own sensors (max roll {max_roll:.0f} deg)")


@pytest.mark.parametrize("regime", MOVING)
def test_policy_tracks_the_command_on_its_own_sensors(params, model, policy, regime):
    """Mean speed (and yaw rate, on the circle) over the last 2.4 s within 20%
    of the command. Measured 2026-09-21: 0.66-0.69 m/s against 0.6 over three
    AHRS seeds, and 0.525 m/s / 0.689 rad/s against 0.5 / 0.625 on the circle
    -- it runs consistently fast, inside the bar."""
    speed, yaw_rate, _ = REGIMES[regime]
    _, A = _episode(params, model, policy, regime)
    tail = A[-TAIL:]
    v, w = float(np.mean(tail[:, 0])), float(np.mean(tail[:, 1]))
    assert abs(v - speed) < 0.2 * speed, f"{regime}: mean speed {v:.3f} vs {speed}"
    if yaw_rate:
        assert abs(w - yaw_rate) < 0.2 * yaw_rate, (
            f"{regime}: mean yaw rate {w:.3f} vs {yaw_rate}")


# -- slip tripwire -----------------------------------------------------------

def _median_slip(params, model, policy, regime, col):
    """Median over SLIP_SEEDS of the RMS slip in column `col` [mm/s]."""
    return float(np.median([_rms_mm(_episode(params, model, policy, regime, s)[1][:, col])
                            for s in SLIP_SEEDS]))


def _slip_check(policy, regime, measured, baseline, what):
    bar = SLIP_MARGIN * baseline[regime]
    assert measured < bar, (
        f"{regime}: {what} slip {measured:.1f} mm/s RMS (median of "
        f"{len(SLIP_SEEDS)} AHRS seeds), over {SLIP_MARGIN}x the "
        f"{baseline[regime]:.1f} recorded for {SLIP_BASELINE_POLICY} "
        f"(now flying {policy[0]}). The POLICY is working this wheel harder than "
        f"the baseline did, or the contact moved -- this is not an estimator "
        f"regression. If it is accepted, re-measure and rewrite the baseline.")


@pytest.mark.parametrize("regime", list(REGIMES))
def test_rear_wheel_slip_under_the_shipped_policy(params, model, policy, regime):
    """Hub spin x outer radius against the hub centre's velocity along the
    heading. This IS the longitudinal odometry error: the estimate is hub
    kinematics, so estimate minus truth equals this slip by construction."""
    _slip_check(policy, regime, _median_slip(params, model, policy, regime, 3),
                REAR_SLIP_BASELINE, "rear-wheel")


@pytest.mark.parametrize("regime", list(REGIMES))
def test_front_wheel_lateral_slip_under_the_shipped_policy(params, model, policy,
                                                           regime):
    """The front wheel's velocity along its own axle. The lateral estimate
    assumes this is zero (v_lat = v_lon*tan(theta) - yaw_rate*L); at
    35-50 mm/s it is the same size as the lateral estimation error."""
    _slip_check(policy, regime, _median_slip(params, model, policy, regime, 4),
                FRONT_SLIP_BASELINE, "front-wheel lateral")
