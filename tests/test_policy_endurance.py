"""How long the shipped policy stays up. A POLICY METRIC, not a code test.

Unlike the rest of the suite, this moves when the POLICY moves: a retrain, a
new `control.general_move`, or a change to the sensor models it is flown on.
It is meant to be followed across policies -- MTBF standing still is the
number to watch once good ones start coming out -- so the failure message
always carries the estimate, not just the verdict.

Flown the way teleop and the bike fly it: the configured policy on the
sensors its move yaml says it trained with (velocity estimate + TM151).
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


@pytest.fixture(scope="module")
def policy():
    name = load_params()["control"].get("general_move", "general_rl")
    path = MOVES_DIR / f"{name}.yaml"
    if not path.exists() or not (MOVES_DIR / f"{name}.npz").exists():
        pytest.skip(f"control.general_move names {name}, which is not exported")
    return name, yaml.safe_load(path.read_text()) or {}


# A FALL IS A RANDOM EVENT IN TIME, not a property of a seed. Measured
# 2026-09-21, 64 AHRS seeds x 60 s standing on its own sensors: 32 fell, at
# 2.1 ... 58.4 s; MTBF 79 s (95% CI 56-116); the power-on accel misalignment
# does not predict which (fallers 0.46 deg vs survivors 0.49, p 0.23). The
# rate is highest in the first 10 s (1.44/min) and 0.3-1.0/min after. Teleop
# (`--teleop --swing-linkage`, same sensors by default) fell at ~35 s.
#
# So the verdict is about EXPOSURE. Many short episodes would mostly sample
# the spawn transient, one long one is a single draw; 60 s is past the
# transient and 4x the longest training episode (15 s). The count comes from
# the target, not from taste -- pass if at most MAX_FALLS of N_SEEDS fall:
#
#   target MTBF   P(fall in a 60 s take)   seeds   pass if   P(red|target)   P(green|today)
#     300 s              18%                 31      <= 9        0.038          8.1e-3
#     600 s               9%                 18      <= 4        0.022          9.9e-3   <- used
#    1200 s               5%                 13      <= 2        0.022          7.6e-3
#
# (binomial on per-episode fall probability, "today" = 0.75 falls/min.)
# A policy between today and the target can land either side -- that is the
# grey zone any finite test has. Seeds are fixed, so the result is
# deterministic: it cannot flake, it can only be one draw of the statistics.
ENDURANCE_TARGET_MTBF_S = 600.0
ENDURANCE_SEEDS = 18
ENDURANCE_MAX_FALLS = 4
ENDURANCE_S = 60.0


def _endurance_flight(seed):
    """Seconds until the bike falls standing still on its own sensors, or
    None if it lasts ENDURANCE_S. Top-level so a process pool can pickle it."""
    import warnings
    warnings.filterwarnings("ignore")
    params = load_params()
    model = build_model(params)
    name = params["control"].get("general_move", "general_rl")
    spec = yaml.safe_load((MOVES_DIR / f"{name}.yaml").read_text()) or {}
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
    ctl.set_command_polar(0.0)
    limit = np.deg2rad(MAX_ROLL_DEG)
    for k in range(int(ENDURANCE_S / dt)):
        if odo is not None:
            with odo.estimated(data, dt):
                ctl.step(model, data)
        elif ahrs is not None:
            ahrs.tick(data, dt)
            with ahrs.estimated(data):
                ctl.step(model, data)
        else:
            ctl.step(model, data)
        mujoco.mj_step(model, data)
        if k % 25 == 0 and abs(extract_state(data, np.zeros(3)).roll) > limit:
            return float(data.time)
    return None


def test_policy_stands_for_a_minute_on_its_own_sensors(policy):
    """At most ENDURANCE_MAX_FALLS of ENDURANCE_SEEDS 60 s standing flights
    fall -- see the block above for where the numbers come from."""
    import multiprocessing as mp
    import os
    from concurrent.futures import ProcessPoolExecutor
    workers = max(1, min(ENDURANCE_SEEDS, (os.cpu_count() or 2) // 2))
    with ProcessPoolExecutor(workers, mp_context=mp.get_context("spawn")) as ex:
        falls = dict(zip(range(ENDURANCE_SEEDS),
                         ex.map(_endurance_flight, range(ENDURANCE_SEEDS))))
    fell = {s: t for s, t in falls.items() if t is not None}
    exposure = sum(t if t is not None else ENDURANCE_S for t in falls.values())
    mtbf = exposure / len(fell) if fell else float("inf")
    assert len(fell) <= ENDURANCE_MAX_FALLS, (
        f"{policy[0]} fell standing still in {len(fell)} of {ENDURANCE_SEEDS} "
        f"{ENDURANCE_S:.0f} s flights on its own sensors (pass: <= "
        f"{ENDURANCE_MAX_FALLS}); MTBF ~{mtbf:.0f} s against a "
        f"{ENDURANCE_TARGET_MTBF_S:.0f} s target. Falls at "
        + ", ".join(f"seed {s}: {t:.1f} s" for s, t in sorted(fell.items())))
