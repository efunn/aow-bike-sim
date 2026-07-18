"""Reduced lateral-model identification + discrete LQR design.

Why not mjd_transitionFD (tried first, abandoned): the FD Jacobian about the
standstill equilibrium is taken in the *sticking* regime of the friction cone,
and underestimates the drive->lateral response by ~2x at real crawl amplitudes
(measured: dy_vel/d_diff -3.2e-3 predicted vs -6.2e-3 actual over one control
period). An LQR gain designed on that model is unstable on the true plant.

Instead we identify the discrete-time reduced lateral model directly at
operating amplitude: state

    x = [e_lat, roll, yaw, steer, v_lat, roll_rate, yaw_rate, steer_rate]

inputs u = [d, steer_cmd] (d = drive_a - drive_b differential; common mode is
handled by a separate longitudinal P loop, which is decoupled from lateral
balance). Procedure: from the settled upright equilibrium, run many
one-control-period rollouts with random small-but-finite initial states and
constant random inputs, then least-squares fit x' = A x + B u. DLQR on (A, B)
with weights from the YAML control.lqr block.
"""

from __future__ import annotations

import mujoco
import numpy as np
import scipy.linalg

N_STATE = 8
IDX_POS = slice(0, 4)   # e_lat, roll, yaw, steer
IDX_VEL = slice(4, 8)


def settle_upright(model: mujoco.MjModel, duration: float = 0.5) -> mujoco.MjData:
    """Converge contacts with the chassis projected upright each step."""
    data = mujoco.MjData(model)
    for _ in range(int(round(duration / model.opt.timestep))):
        mujoco.mj_step(model, data)
        data.qpos[0:2] = 0.0
        data.qpos[3:7] = (1, 0, 0, 0)
        data.qvel[0:2] = 0.0
        data.qvel[3:6] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    return data


def _reduced_state(model, data) -> np.ndarray:
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, data.qpos[3:7])
    R = R.reshape(3, 3)
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    sj, sd = model.joint("steer_joint").qposadr[0], model.joint("steer_joint").dofadr[0]
    return np.array([
        data.qpos[1], roll, yaw, data.qpos[sj],
        data.qvel[1], data.qvel[3], data.qvel[5], data.qvel[sd],
    ])


def _set_reduced_state(model, data, qpos_eq, x) -> None:
    data.qpos[:] = qpos_eq
    data.qvel[:] = 0.0
    data.qpos[1] = x[0]
    half_r, half_y = x[1] / 2, x[2] / 2
    q_roll = np.array([np.cos(half_r), np.sin(half_r), 0, 0])
    q_yaw = np.array([np.cos(half_y), 0, 0, np.sin(half_y)])
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, q_yaw, q_roll)
    data.qpos[3:7] = quat
    sj, sd = model.joint("steer_joint").qposadr[0], model.joint("steer_joint").dofadr[0]
    data.qpos[sj] = x[3]
    data.qvel[1], data.qvel[3], data.qvel[5], data.qvel[sd] = x[4:8]
    mujoco.mj_forward(model, data)


def identify_lateral_model(
    params: dict,
    model: mujoco.MjModel,
    qpos_eq: np.ndarray,
    n_episodes: int = 400,
    seed: int = 0,
):
    """Least-squares discrete (A, B) over one control period, at finite amplitude."""
    n_lift = max(1, int(round(1.0 / params["control"]["rate_hz"]
                              / model.opt.timestep)))
    rng = np.random.default_rng(seed)
    scale_x = np.array([0.01, 0.02, 0.02, 0.10,    # m, rad, rad, rad
                        0.05, 0.20, 0.10, 0.50])   # m/s, rad/s x3
    scale_u = np.array([6.0, 0.15])                # diff rad/s, steer rad
    data = mujoco.MjData(model)
    aid = {n: model.actuator(n).id for n in ("drive_a", "drive_b", "steer")}

    X, U, Xn = [], [], []
    for _ in range(n_episodes):
        x0 = rng.uniform(-1, 1, N_STATE) * scale_x
        u = rng.uniform(-1, 1, 2) * scale_u
        _set_reduced_state(model, data, qpos_eq, x0)
        data.ctrl[:] = 0.0
        data.ctrl[aid["drive_a"]] = u[0] / 2
        data.ctrl[aid["drive_b"]] = -u[0] / 2
        data.ctrl[aid["steer"]] = u[1]
        for _ in range(n_lift):
            mujoco.mj_step(model, data)
        X.append(x0)
        U.append(u)
        Xn.append(_reduced_state(model, data))
    X, U, Xn = np.array(X), np.array(U), np.array(Xn)

    Z = np.hstack([X, U])
    theta, *_ = np.linalg.lstsq(Z, Xn, rcond=None)
    A, B = theta[:N_STATE].T, theta[N_STATE:].T
    resid = Xn - Z @ theta
    r2 = 1.0 - resid.var(axis=0) / np.maximum(Xn.var(axis=0), 1e-12)
    return A, B, r2


def design_lqr(params: dict, model: mujoco.MjModel):
    """Returns (K over the reduced state, equilibrium qpos, fit R^2 per state)."""
    cfg = params["control"]["lqr"]
    data_eq = settle_upright(model)
    A, B, r2 = identify_lateral_model(params, model, data_eq.qpos)
    Q = np.diag([
        cfg["q_ypos"], cfg["q_roll"], cfg["q_yaw"], cfg["q_steer"],
        cfg["q_yvel"], cfg["q_roll_rate"],
        cfg.get("q_yaw_rate", 0.2 * cfg["q_yaw"]), 0.1 * cfg["q_steer"],
    ])
    R = np.diag([cfg["r_drive"], cfg["r_steer"]])
    X = scipy.linalg.solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ X @ B, B.T @ X @ A)
    if np.max(np.abs(np.linalg.eigvals(A - B @ K))) >= 1.0:
        raise RuntimeError("identified-model LQR is not stabilizing")
    return K, data_eq.qpos.copy(), r2
