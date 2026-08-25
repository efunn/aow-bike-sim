"""Baseline stationary balance controllers.

Two controllers behind one interface:
  PDCascade  — transparent reference: roll PD -> rear lateral crawl velocity,
               slow outer loop on lateral drift -> roll setpoint, weak
               longitudinal P -> common-mode drive. Steer held straight.
  LQRBalance — discrete LQR on the numerically linearized full model
               (see linearize.py); free to use steering + crawl together.

Both run at `control.rate_hz` with zero-order hold between updates (physics
steps much faster), use ground-truth simulator state, and saturate to the
actuator ctrlranges. Sensor-only estimation is a later phase.

Conventions (chassis frame: +X forward, +Y left, +Z up):
  roll > 0  = lean right (-Y side down), from ZYX Euler of the chassis quat.
  Differential drive d = drive_a - drive_b; d > 0 crawls the rear contact
  toward -Y (verified empirically; flips with drivetrain.k_roller sign).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .steer import SteerFrame


def mix(common: float, diff: float) -> tuple[float, float]:
    """(common, differential) -> (drive_a, drive_b) input-shaft commands."""
    return common + diff / 2, common - diff / 2


def lat_gain(params: dict) -> float:
    """Rear-contact lateral velocity [m/s] per unit differential d [rad/s].

    ring_rel = d/2, roller spin = k_roller * d/2, contact speed = spin * rho_eff.
    Sign: d > 0 moves the contact toward -Y (matches test_lateral_crawl).
    """
    roller = params["omni_wheel"]["roller"]
    rho_eff = (roller["big_diameter"] + roller["small_diameter"]) / 4
    return -params["drivetrain"]["k_roller"] / 2 * rho_eff


@dataclass
class BikeState:
    roll: float
    roll_rate: float
    yaw: float
    e_lon: float   # fore/aft drift from the reference point, bike-yaw frame
    e_lat: float   # lateral drift, +Y(left) positive
    v_lon: float
    v_lat: float
    # Defaulted so every positional BikeState(...) construction still works.
    pitch: float = 0.0        # +ve = NOSE UP; see extract_state on the sign
    pitch_rate: float = 0.0


def quat_to_mat(q) -> np.ndarray:
    """(w,x,y,z) -> 3x3 rotation matrix, identical to mujoco.mju_quat2Mat.

    Written out rather than called so this module — and therefore the whole
    controller stack — imports without MuJoCo. The bike runs the controllers,
    not the simulator; see params.py and tests/test_hw_no_mujoco.py.
    """
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def extract_state(data, ref_pos: np.ndarray) -> BikeState:
    """Ground-truth state of the chassis freejoint (qpos[0:7], qvel[0:6])."""
    R = quat_to_mat(data.qpos[3:7])
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    roll_rate = data.qvel[3]  # freejoint angular velocity is body-frame
    c, s = np.cos(yaw), np.sin(yaw)
    to_yaw = np.array([[c, s], [-s, c]])
    e_lon, e_lat = to_yaw @ (data.qpos[:2] - ref_pos[:2])
    v_lon, v_lat = to_yaw @ data.qvel[:2]
    # NOTE THE SIGN. The textbook ZYX pitch is asin(-R[2,0]), which is NEGATIVE
    # when the nose rises, because R[2,0] is the world-z component of the body
    # +X (forward) axis. Reported that way, `max(pitch)` picks the nose-DOWN
    # tail and a 23 deg wheelie reads as "pitch never exceeded 0.4 deg".
    # Negated here so the sign matches the word. On hardware both of these
    # come straight off the AHRS (hw/state.set_orientation), so unlike world
    # position they are honestly observable.
    # pitch_rate is -qvel[4] for the same reason: body +Y points LEFT, so a
    # positive rotation about it pitches the nose DOWN. Verified against
    # d(pitch)/dt, which correlates +0.81 with -qvel[4] and -0.81 with +qvel[4].
    pitch = np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    return BikeState(roll, roll_rate, yaw, e_lon, e_lat, v_lon, v_lat,
                     pitch, -data.qvel[4])


class _Base:
    """Shared ZOH scheduling, actuator lookup, and saturation."""

    def __init__(self, params: dict, model):
        self.params = params
        self.dt = 1.0 / params["control"]["rate_hz"]
        self.aid = {n: model.actuator(n).id for n in ("drive_a", "drive_b", "steer")}
        # Present only on a wings model; every wingless model omits it and the
        # general policy's wing channel is simply unavailable there.
        for _opt in ("wings", "swing"):
            # Present only on the model that built that mechanism; every other
            # model omits it and the general policy's channel is simply
            # unavailable there.
            try:
                self.aid[_opt] = model.actuator(_opt).id
            except (KeyError, ValueError):
                pass
        # Saturate only actuators that declare a ctrlrange (steer is unlimited).
        limited = model.actuator_ctrllimited.astype(bool)
        self.lo = np.where(limited, model.actuator_ctrlrange[:, 0], -np.inf)
        self.hi = np.where(limited, model.actuator_ctrlrange[:, 1], np.inf)
        self._ref_pos: np.ndarray | None = None
        self._next_t = 0.0
        self._u = np.zeros(model.nu)

    def reset(self, model, data) -> None:
        self._ref_pos = data.qpos[:3].copy()
        self._next_t = data.time
        self._u = np.zeros(model.nu)

    def step(self, model, data) -> np.ndarray:
        """Call every physics step; writes data.ctrl with ZOH at rate_hz."""
        if self._ref_pos is None or data.time < self._next_t - 2 * self.dt:
            self.reset(model, data)  # first call, or viewer was reset
        if data.time + 1e-12 >= self._next_t:
            u = np.asarray(self._compute(model, data), dtype=float)
            self._u = np.clip(u, self.lo, self.hi)
            self._next_t = data.time + self.dt
        data.ctrl[:] = self._u
        return self._u

    def _compute(self, model, data) -> np.ndarray:  # pragma: no cover
        raise NotImplementedError


class PDCascade(_Base):
    """Roll PD -> crawl velocity; outer lateral P(D) -> roll setpoint;
    weak longitudinal P -> common mode; steer held at zero."""

    def __init__(self, params, model):
        super().__init__(params, model)
        g = params["control"]["pd"]
        self.roll_kp, self.roll_kd = g["roll_kp"], g["roll_kd"]
        self.y_kp, self.y_kd = g["y_kp"], g["y_kd"]
        self.x_kp = g["x_kp"]
        self.max_roll_ref = np.deg2rad(g["max_roll_setpoint_deg"])
        self.lat_per_d = lat_gain(params)

    def _compute(self, model, data):
        s = extract_state(data, self._ref_pos)
        # Drifted left (e_lat > 0): lean right (roll > 0) so the crawl that
        # catches the fall carries the base back to the right.
        roll_ref = np.clip(
            self.y_kp * s.e_lat + self.y_kd * s.v_lat,
            -self.max_roll_ref, self.max_roll_ref,
        )
        # Leaning right beyond setpoint: accelerate the base right (-Y) to get
        # under the CoM. A velocity-source base can't stabilize a pendulum from
        # roll feedback alone (the commanded velocity must be relative to the
        # current base velocity), so this is an acceleration-style law:
        # v_cmd = v_now + roll PD.
        v_lat_cmd = s.v_lat - (
            self.roll_kp * (s.roll - roll_ref) + self.roll_kd * s.roll_rate
        )
        d = v_lat_cmd / self.lat_per_d
        common = -self.x_kp * s.e_lon
        a, b = mix(common, d)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = 0.0
        return u


class LQRBalance(_Base):
    """DLQR on the identified reduced lateral model (see linearize.py).

    [d, steer] = -K x_lat; a separate longitudinal P loop supplies the common
    mode (decoupled from lateral balance)."""

    def __init__(self, params, model, design=None):
        super().__init__(params, model)
        # Re-identified from `params` on EVERY construction, so the design can
        # never go stale relative to the model -- change a mass or a contact
        # parameter and the next run re-derives against it. Measured cost is
        # 0.39 s (design_lqr) / 2.0 s (design_all), not the "minutes" an
        # earlier comment here claimed, so there is no reason to cache it.
        #
        # `design` short-circuits it anyway for the ONE case that cannot run
        # it: the onboard path loads a precomputed LQRDesign from
        # deploy/bundle.npz because the Pi has neither scipy nor MuJoCo. That
        # copy IS cacheable and therefore IS staleness-prone, which is exactly
        # why export_deploy stamps a params_digest and hw.state refuses a
        # bundle that does not match. See docs/plans/untethered-setup.md.
        if design is None:
            from .linearize import design_lqr  # deferred: pulls in scipy
            self.K, self.qpos_eq, self.fit_r2 = design_lqr(params, model)
        else:
            self.K, self.qpos_eq, self.fit_r2 = (
                design.K, design.qpos_eq, design.fit_r2)
        self.x_kp = params["control"]["pd"]["x_kp"]
        # Hard steer clamp: the lateral model is identified at small steer
        # angles; letting the loop command large ones leaves the region where
        # the linear design is valid (and did destabilize it in practice).
        self.steer_limit = np.deg2rad(params["control"]["lqr"]["steer_limit_deg"])
        self._sj = model.joint("steer_joint").qposadr[0]
        self._sd = model.joint("steer_joint").dofadr[0]
        # Wings model only; None everywhere else, which is what the general
        # policy's wing flags are checked against in engage_general.
        self._wj = self._wd = None
        # Whichever mechanism this model was built with -- `wing_right_joint`
        # for the mirrored pair, `swing_right_joint` for the co-rotating one.
        # They are alternatives, so at most one exists.
        #
        # NOTE WHY THIS MATTERS MORE THAN IT LOOKS: a miss leaves _wj as None,
        # and `data.qpos[None]` does NOT raise -- numpy reads None as
        # np.newaxis and hands back a 1xN array, so the failure surfaces later
        # as "only 0-dimensional arrays can be converted to Python scalars"
        # from a float() several frames away, with nothing pointing at the
        # lookup that actually failed.
        for _jn in ("wing_right_joint", "swing_right_joint"):
            try:
                wj = model.joint(_jn)
            except (KeyError, ValueError):
                continue
            self._wj, self._wd = wj.qposadr[0], wj.dofadr[0]
            break
        self._ref_yaw = 0.0
        self.steer_frame = SteerFrame()

    def reset(self, model, data):
        super().reset(model, data)
        s = extract_state(data, self._ref_pos)
        self._ref_yaw = s.yaw
        self.steer_frame.sync(float(data.qpos[self._sj]))

    def _compute(self, model, data):
        s = extract_state(data, self._ref_pos)
        yaw_err = np.arctan2(np.sin(s.yaw - self._ref_yaw),
                             np.cos(s.yaw - self._ref_yaw))
        x = np.array([
            s.e_lat, s.roll, yaw_err,
            self.steer_frame.measured(data.qpos[self._sj]),
            s.v_lat, s.roll_rate, data.qvel[5], data.qvel[self._sd],
        ])
        d, steer = -self.K @ x
        steer = np.clip(steer, -self.steer_limit, self.steer_limit)
        a, b = mix(-self.x_kp * s.e_lon, d)
        u = np.zeros(len(self._u))
        u[self.aid["drive_a"]], u[self.aid["drive_b"]] = a, b
        u[self.aid["steer"]] = self.steer_frame.command(steer)
        return u


def make_controller(name: str, params: dict, model):
    if name == "pd":
        return PDCascade(params, model)
    if name == "lqr":
        return LQRBalance(params, model)
    raise ValueError(f"unknown controller {name!r}; expected 'pd' or 'lqr'")


def run(model, data, controller, duration: float, on_step=None) -> None:
    """Advance the sim `duration` seconds with the controller in the loop."""
    for _ in range(int(round(duration / model.opt.timestep))):
        controller.step(model, data)
        import mujoco       # sim-only helper; not needed on the bike
        mujoco.mj_step(model, data)
        if on_step is not None:
            on_step(data)
