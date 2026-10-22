"""Which point does the lateral estimator track: the chassis origin (the rear
axle centre, what the reward tracks) or the rigid-body point under it?

Flies control.general_move on its own sensors and plant, as
tests/test_odometry_in_the_loop._fly does, for 4.8 s per regime, and compares
the estimate with both points. The two differ sideways by roll_rate x the
wheel radius, so a reference-point error would show as a slope of ~51 mm/rad
against roll rate. Recorded in docs/plans/odometry-sharp-steer.md ("Which
point the estimate tracks").

  python analysis/sensing/odo_refpoint.py [--seed N]
"""
import argparse

import mujoco
import numpy as np
import yaml

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import extract_state
from aow_sim.control.drive import DriveController
from aow_sim.control.flick import MOVES_DIR
from aow_sim.control.linearize import settle_upright
from aow_sim.drivetrain_model import KEY, DrivetrainSim, from_env_config
from aow_sim.sim_ahrs import TAU_ORIENT_S, SimAhrs
from aow_sim.sim_odometry import SimOdometry

# (speed [m/s], heading rate [rad/s], shove [m/s]): the in-the-loop test's
REGIMES = {"standstill": (0.0, 0.0, 0.0), "shoved": (0.0, 0.0, 0.12),
           "straight_0.6": (0.6, 0.0, 0.0), "circle_R0.8": (0.5, 0.625, 0.0)}
N_STEPS, WARMUP = 12000, 3000


def fly(model, params, name, spec, speed, yaw_rate, shove, seed):
    dt = model.opt.timestep
    h = float(params["omni_wheel"]["outer_radius"])
    ahrs = SimAhrs(model, params, level=spec["ahrs_level"], seed=seed,
                   tau_orient_s=float(spec.get("ahrs_tau_s", TAU_ORIENT_S)))
    odo = SimOdometry(model, params, mode="front",
                      encoder=str(spec.get("odometry_encoder") or "ideal"), ahrs=ahrs)
    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    mujoco.mj_forward(model, data)
    ctl = DriveController(params, model)
    ctl.reset(model, data)
    ctl._odometry_active = ctl._ahrs_active = True
    ctl._drivetrain_active = KEY in params
    ctl.engage_general(data, name=name)
    ctl.set_command_polar(speed)
    drive = DrivetrainSim.attach(model, params)
    rng = np.random.default_rng(seed)
    rows = []
    R = np.zeros(9)
    for k in range(N_STEPS):
        if yaw_rate:
            ctl.set_command_polar(speed, psi_cmd=ctl._gen_psi_cmd + yaw_rate * dt)
        with odo.estimated(data, dt):
            ctl.step(model, data)
        if shove and k and k % 3000 == 0:
            data.qvel[1] += rng.uniform(-shove, shove)
        if drive is not None:
            drive.pre_step(data)
        mujoco.mj_step(model, data)
        if k < WARMUP:
            continue
        s = extract_state(data, np.zeros(3))
        mujoco.mju_quat2Mat(R, data.qpos[3:7])
        w_world = R.reshape(3, 3) @ data.qvel[3:6]
        v_origin = data.qvel[0:3]
        v_below = v_origin + np.cross(w_world, [0.0, 0.0, -h])
        c, sn = np.cos(s.yaw), np.sin(s.yaw)
        rows.append((odo.est.v_lat, -sn * v_origin[0] + c * v_origin[1],
                     -sn * v_below[0] + c * v_below[1], data.qvel[3]))
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seed", type=int, default=0, help="AHRS / shove seed")
    args = ap.parse_args()
    name = load_params()["control"]["general_move"]
    spec = yaml.safe_load((MOVES_DIR / f"{name}.yaml").read_text())
    params = from_env_config(load_params(), {KEY: spec.get(KEY)})
    model = build_model(params)

    def rms(x):
        return 1e3 * float(np.sqrt(np.mean(x ** 2)))

    print(f"{name}, seed {args.seed}; v_lat RMS error, mm/s")
    print(f"{'regime':14s}{'vs origin':>11s}{'vs below':>10s}"
          f"{'slope mm/rad':>14s}{'corr':>7s}{'roll rate RMS':>15s}")
    for regime, (speed, yr, shove) in REGIMES.items():
        a = fly(model, params, name, spec, speed, yr, shove, args.seed)
        e = a[:, 0] - a[:, 1]
        print(f"{regime:14s}{rms(e):11.1f}{rms(a[:, 0] - a[:, 2]):10.1f}"
              f"{1e3 * np.polyfit(a[:, 3], e, 1)[0]:+14.1f}"
              f"{np.corrcoef(e, a[:, 3])[0, 1]:+7.2f}"
              f"{np.sqrt(np.mean(a[:, 3] ** 2)):13.2f} rad/s")


if __name__ == "__main__":
    main()
