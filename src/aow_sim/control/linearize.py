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

from dataclasses import dataclass

import warnings

import mujoco
import numpy as np

from ..build_model import reset_actuator_state

# The artifact lives in a MuJoCo-free module so the bike can load it.
from .lqr_design import LQRDesign  # noqa: F401

# scipy is imported lazily inside _dlqr_checked, not here: hw/state.py imports
# LQRDesign from this module, and the whole point of the deployment bundle is
# that the bike needs neither scipy nor a Riccati solve. A module-level import
# would drag scipy onto the Pi for a dataclass.

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


def settle_rolling(
    model: mujoco.MjModel, params: dict, v: float, duration: float = 0.5
) -> mujoco.MjData:
    """Steady straight rolling at speed v (+X), upright: settle contacts with
    the chassis projected onto the line each step while the drives hold the
    common-mode speed, then snapshot a kinematically consistent state.

    The returned data carries the equilibrium ctrl (common-mode hold) — the
    caller must keep/offset it, not zero it."""
    if v == 0.0:
        return settle_upright(model, duration)
    r_wheel = params["omni_wheel"]["outer_radius"]
    r_front = params["bike"]["front_wheel"]["radius"]
    hub_rate = v / r_wheel
    data = mujoco.MjData(model)
    for name in ("drive_a", "drive_b"):
        data.ctrl[model.actuator(name).id] = hub_rate  # common mode = hub rate
    for _ in range(int(round(duration / model.opt.timestep))):
        mujoco.mj_step(model, data)
        data.qpos[1] = 0.0                 # x advances freely
        data.qpos[3:7] = (1, 0, 0, 0)
        data.qvel[0] = v
        data.qvel[1] = 0.0
        data.qvel[3:6] = 0.0
    # Consistent snapshot: only the rolling DOFs move.
    qvel = np.zeros(model.nv)
    qvel[0] = v
    for joint, rate in (("hub_spin", hub_rate), ("input_a_spin", hub_rate),
                        ("input_b_spin", hub_rate), ("front_spin", v / r_front)):
        qvel[model.joint(joint).dofadr[0]] = rate
    data.qvel[:] = qvel
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


def _set_reduced_state(model, data, eq: mujoco.MjData, x) -> None:
    """Superimpose a reduced-state perturbation on the (possibly rolling)
    equilibrium `eq`."""
    data.qpos[:] = eq.qpos
    data.qvel[:] = eq.qvel
    data.qpos[1] = x[0]
    half_r, half_y = x[1] / 2, x[2] / 2
    q_roll = np.array([np.cos(half_r), np.sin(half_r), 0, 0])
    q_yaw = np.array([np.cos(half_y), 0, 0, np.sin(half_y)])
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, q_yaw, q_roll)
    data.qpos[3:7] = quat
    sj, sd = model.joint("steer_joint").qposadr[0], model.joint("steer_joint").dofadr[0]
    data.qpos[sj] = x[3]
    data.qvel[1] = eq.qvel[1] + x[4]
    data.qvel[3] = eq.qvel[3] + x[5]
    data.qvel[5] = eq.qvel[5] + x[6]
    data.qvel[sd] = eq.qvel[sd] + x[7]
    # The drive integrator (actuators.drive_ki > 0) is state, exactly as qvel
    # is: restore the equilibrium's, or every rollout inherits the previous
    # episode's wind-up as a random unmodelled input. Worst fit R^2 measured
    # 0.7543 without this, 0.9412 with it.
    reset_actuator_state(model, data, eq.act)
    mujoco.mj_forward(model, data)


def identify_lateral_model(
    params: dict,
    model: mujoco.MjModel,
    eq: mujoco.MjData,
    n_episodes: int = 400,
    seed: int = 0,
):
    """Least-squares discrete (A, B) over one control period, at finite
    amplitude, about the (possibly rolling) equilibrium `eq` — whose ctrl
    carries the common-mode hold that rollout inputs are offset from."""
    n_lift = max(1, int(round(1.0 / params["control"]["rate_hz"]
                              / model.opt.timestep)))
    rng = np.random.default_rng(seed)
    scale_x = np.array([0.01, 0.02, 0.02, 0.10,    # m, rad, rad, rad
                        0.05, 0.20, 0.10, 0.50])   # m/s, rad/s x3
    scale_u = np.array([6.0, 0.15])                # diff rad/s, steer rad
    data = mujoco.MjData(model)
    aid = {n: model.actuator(n).id for n in ("drive_a", "drive_b", "steer")}
    base_a, base_b = eq.ctrl[aid["drive_a"]], eq.ctrl[aid["drive_b"]]

    X, U, Xn = [], [], []
    for _ in range(n_episodes):
        x0 = rng.uniform(-1, 1, N_STATE) * scale_x
        u = rng.uniform(-1, 1, 2) * scale_u
        _set_reduced_state(model, data, eq, x0)
        data.ctrl[:] = 0.0
        data.ctrl[aid["drive_a"]] = base_a + u[0] / 2
        data.ctrl[aid["drive_b"]] = base_b - u[0] / 2
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


def _weights(cfg) -> tuple[np.ndarray, np.ndarray]:
    Q = np.diag([
        cfg["q_ypos"], cfg["q_roll"], cfg["q_yaw"], cfg["q_steer"],
        cfg["q_yvel"], cfg["q_roll_rate"],
        cfg.get("q_yaw_rate", 0.2 * cfg["q_yaw"]), 0.1 * cfg["q_steer"],
    ])
    R = np.diag([cfg["r_drive"], cfg["r_steer"]])
    return Q, R


def _dlqr_checked(A, B, Q, R, label: str):
    import scipy.linalg

    X = scipy.linalg.solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ X @ B, B.T @ X @ A)
    specrad = np.max(np.abs(np.linalg.eigvals(A - B @ K)))
    if specrad >= 1.0:
        raise RuntimeError(f"identified-model LQR not stabilizing ({label})")
    return K


MIN_FIT_R2 = 0.98
"""Below this the identified LINEAR model stops describing the plant well.

Worth being clear about what this catches, because it is NOT staleness: the
design is re-identified from `params` on every construction, so it always
matches the current model. A low R^2 means the model is genuinely less linear
than the LQR assumes -- softer contacts, a heavier bike, a changed geometry --
and re-running the identification will not improve it. What has to change is
the plant, the operating point, or the expectation.

Concretely: contact damping of 0.5 (vs 1.0) drops the worst channel to ~0.970.
Nothing warns you at runtime unless this does, and the symptom downstream is
just a slightly worse analytic controller, which is easy to misread as tuning.
"""


def _warn_fit(r2_grid, speeds=None) -> None:
    """One warning per DESIGN, naming the worst operating point.

    Deliberately not one per speed: the gain schedule identifies nine of them,
    so per-speed warnings meant four lines of identical text on every teleop,
    record and analysis startup. That is wallpaper, and wallpaper gets filtered
    out — which is the opposite of what a fit check is for. Summarise instead,
    and point at the speed that is actually the problem.
    """
    r2_grid = np.atleast_2d(r2_grid)
    worst_per_row = r2_grid.min(axis=1)
    if worst_per_row.min() >= MIN_FIT_R2:
        return
    i = int(np.argmin(worst_per_row))
    where = (f"v={speeds[i]:+.2f} m/s" if speeds is not None
             else "the standstill design")
    n_bad = int((worst_per_row < MIN_FIT_R2).sum())
    extra = (f" ({n_bad} of {len(worst_per_row)} scheduled speeds are under it)"
             if len(worst_per_row) > 1 else "")
    warnings.warn(
        f"lateral model fits poorly — worst R^2 {worst_per_row.min():.3f} < "
        f"{MIN_FIT_R2} at {where}{extra}. The design is CURRENT (re-identified "
        "every run); this is the PLANT being less linear than the LQR assumes, "
        "so the fix is sim.contact_*/mass/geometry or a narrower operating "
        "envelope — not re-running the design.",
        RuntimeWarning, stacklevel=3)


def design_lqr(params: dict, model: mujoco.MjModel, v: float = 0.0):
    """Returns (K over the reduced state, equilibrium qpos, fit R^2 per state)."""
    Q, R = _weights(params["control"]["lqr"])
    eq = settle_rolling(model, params, v)
    A, B, r2 = identify_lateral_model(params, model, eq)
    K = _dlqr_checked(A, B, Q, R, f"v={v:.2f}")
    _warn_fit(r2)
    return K, eq.qpos.copy(), r2


def design_gain_schedule(params: dict, model: mujoco.MjModel):
    """(speeds, K_stack, r2_stack) over the mirrored control.drive.speed_grid.

    Runtime gains come from per-element linear interpolation in measured
    forward speed (clamped at the grid ends)."""
    grid = sorted(params["control"]["drive"]["speed_grid"])
    speeds = sorted({-v for v in grid} | set(grid))
    Q, R = _weights(params["control"]["lqr"])
    Ks, r2s = [], []
    for v in speeds:
        eq = settle_rolling(model, params, v)
        A, B, r2 = identify_lateral_model(params, model, eq)
        Ks.append(_dlqr_checked(A, B, Q, R, f"v={v:.2f}"))
        r2s.append(r2)
    _warn_fit(np.stack(r2s), np.array(speeds))
    return np.array(speeds), np.stack(Ks), np.stack(r2s)


def design_all(params: dict, model: mujoco.MjModel) -> LQRDesign:
    """Both designs in one object. ~2 s: the single-point design plus one
    identification per speed in `control.drive.speed_grid`. Cheap enough to
    run at startup, which is what every in-sim path does."""
    K, qpos_eq, fit_r2 = design_lqr(params, model)
    speeds, Ks, r2s = design_gain_schedule(params, model)
    return LQRDesign(K, qpos_eq, fit_r2, speeds, Ks, r2s)
