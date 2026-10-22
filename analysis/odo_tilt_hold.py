"""The bench tilt test, in sim: steer held at 90 deg, drives commanded to 0,
the chassis held by a 'hand' (a stiff PD on position, yaw and pitch, and a
roll PD following a tilt profile). The onboard estimator runs on the
simulated TM151 (tm151_filter) and counts encoder, detailed drivetrain --
the policy's training sensors and plant. Reports what the estimate thinks
the bike did against what it did. The bench version is
traces/bike/261021-181828_run_tilt_steer90; both are in
docs/plans/odometry-sharp-steer.md.

  python analysis/odo_tilt_hold.py [steer_deg] [n_seeds]
"""
import sys
import numpy as np
import mujoco
import yaml

from aow_sim.build_model import build_model, load_params
from aow_sim.drivetrain_model import KEY, DrivetrainSim, from_env_config
from aow_sim.control.flick import MOVES_DIR
from aow_sim.control.linearize import settle_upright
from aow_sim.sim_ahrs import SimAhrs
from aow_sim.sim_odometry import SimOdometry

STEER_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
SEEDS = int(sys.argv[2]) if len(sys.argv) > 2 else 5

name = load_params()["control"]["general_move"]
spec = yaml.safe_load((MOVES_DIR / f"{name}.yaml").read_text())
params = from_env_config(load_params(), {KEY: spec.get(KEY)})
model = build_model(params)
dt = model.opt.timestep
cid = model.body("chassis").id

# (duration s, roll at the END deg): ramps between holds, like the bench run
PROFILE = [(2, 0), (1, 20), (3, 20), (1, 0), (1, -20), (3, -20), (1, 0),
           (1, 20), (3, 20), (1, 0), (1, -20), (3, -20), (1, 0), (10, 0)]
T = sum(d for d, _ in PROFILE)


def roll_target(t):
    t0, r0 = 0.0, 0.0
    for d, r in PROFILE:
        if t <= t0 + d:
            return np.deg2rad(r0 + (r - r0) * (t - t0) / d)
        t0, r0 = t0 + d, r
    return np.deg2rad(r0)


def quat_err(q, q_t):
    """World-frame rotation vector taking q to q_t."""
    qi = np.zeros(4); mujoco.mju_negQuat(qi, q)
    qe = np.zeros(4); mujoco.mju_mulQuat(qe, q_t, qi)
    v = np.zeros(3); mujoco.mju_quat2Vel(v, qe, 1.0)
    return v


def fly(seed):
    settled = settle_upright(model)
    data = mujoco.MjData(model)
    data.qpos[:] = settled.qpos
    mujoco.mj_forward(model, data)
    ahrs = SimAhrs(model, params, level=spec["ahrs_level"], seed=seed,
                   tau_orient_s=float(spec.get("ahrs_tau_s", 2.0)))
    odo = SimOdometry(model, params, mode="front",
                      encoder=str(spec.get("odometry_encoder") or "ideal"), ahrs=ahrs)
    drive = DrivetrainSim.attach(model, params)
    a = {model.actuator(i).name: i for i in range(model.nu)}
    data.ctrl[a["drive_a"]] = data.ctrl[a["drive_b"]] = 0.0
    p0 = data.qpos[0:3].copy()
    q_up = data.qpos[3:7].copy()
    m_tot = float(model.body_subtreemass[cid])
    rows = []
    n = int(T / dt)
    for k in range(n):
        t = k * dt
        # steer to STEER_DEG over the first second, then hold
        data.ctrl[a["steer"]] = np.deg2rad(STEER_DEG) * min(1.0, t)
        # the hand: target attitude = upright rotated by the roll profile about
        # the bike's own forward axis
        r = roll_target(t)
        q_roll = np.array([np.cos(r / 2), np.sin(r / 2), 0.0, 0.0])
        q_t = np.zeros(4); mujoco.mju_mulQuat(q_t, q_up, q_roll)
        R = np.zeros(9); mujoco.mju_quat2Mat(R, data.qpos[3:7]); R = R.reshape(3, 3)
        w_world = R @ data.qvel[3:6]
        data.xfrc_applied[cid, 3:6] = 40.0 * quat_err(data.qpos[3:7], q_t) - 1.5 * w_world
        # position held in x, y (a hand), z left to the contact
        data.xfrc_applied[cid, 0:2] = m_tot * (400.0 * (p0[:2] - data.qpos[:2])
                                               - 40.0 * data.qvel[:2])
        v_lon_e, v_lat_e = odo.update(data, dt)
        if drive is not None:
            drive.pre_step(data)
        mujoco.mj_step(model, data)
        if k % 25 == 0:      # 100 Hz
            yaw = np.arctan2(R[1, 0], R[0, 0])
            c, s = np.cos(yaw), np.sin(yaw)
            v = data.qvel[:2]
            rows.append((t, np.degrees(r), v_lon_e, v_lat_e, c * v[0] + s * v[1],
                         -s * v[0] + c * v[1], odo.est.confidence,
                         np.degrees(np.arctan2(R[2, 1], R[2, 2]))))
    return np.array(rows)


print(f"{name}, steer {STEER_DEG:.0f} deg, {T} s, AHRS {spec['ahrs_level']}, "
      f"encoder {spec.get('odometry_encoder')}, detailed drivetrain")
finals, peaks = [], []
for seed in range(SEEDS):
    x = fly(seed)
    t, vlat_e, vlat_t, conf = x[:, 0], x[:, 3], x[:, 5], x[:, 6]
    y_e = np.cumsum(vlat_e) * 0.01
    y_t = np.cumsum(vlat_t) * 0.01
    still = t > T - 10
    print(f"  achieved roll {x[:,7].min():+.1f}..{x[:,7].max():+.1f} deg (target +-20), tracking RMS {np.sqrt(np.mean((x[:,7]-x[:,1])**2)):.1f} deg")
    print(f"seed {seed}: conf {conf[t > 2].mean():.2f}  est lateral travel {y_e[-1]:+.3f} m "
          f"(true {y_t[-1]:+.3f})  peak |v_lat est| {np.abs(vlat_e).max():.3f}  "
          f"v_lat est over the last 10 s still: {vlat_e[still].mean():+.3f} m/s "
          f"(start {vlat_e[still][0]:+.3f}, end {vlat_e[still][-1]:+.3f})")
    finals.append(y_e[-1]); peaks.append(np.abs(vlat_e).max())
print(f"median |est travel| {np.median(np.abs(finals)):.3f} m over {T} s; "
      f"median peak |v_lat| {np.median(peaks):.3f} m/s")
