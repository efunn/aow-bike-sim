"""The detailed drivetrain: XC430 firmware loop, differential detent, roller slop.

OPT-IN. `config/drivetrain_model.yaml` is an overlay, loaded only by callers
that ask (`run_drive --drivetrain`, `analysis/drivetrain_eval.py`). Without it
`edit_spec` is a no-op, `DrivetrainSim.attach` returns None, and the model is
the one every policy trained on. With it the params dict carries the overlay
under `KEY`, which moves `plant_digest` -- correctly, since it is a different
plant. The reasoning and the fits: docs/plans/drivetrain-model.md.

Three parts, each switchable, because each answers a different question and
has its own evidence:

  servo        The shipped drive is a native MuJoCo velocity-PI with no delay,
               a flat torque clamp and a GUESSed armature. This replaces it
               with the firmware law fitted to Present PWM on the bench,
                   duty = Kp (v_ref - LPF(w_meas)) + Ki (integral(v_ref) - theta_meas)
               clipped to +-1, driving the motor line
                   torque = stall_torque (duty - w / no_load_speed),
               with the command latency and the loop's own measurement lag.
               `ctrl[drive_a/b]` KEEPS ITS MEANING -- commanded input-shaft
               speed -- so no caller changes; the two drive actuators become
               zero-force command holders and two torque actuators,
               `drive_a_motor` / `drive_b_motor`, are appended LAST so no
               existing actuator index moves.
  detent       A friction BUMP on the ring-vs-hub coordinate once per 7.5 deg.
               Friction, not a conservative hill: at the 1 % creep the extra
               push sits at the same phase in both push directions. Applied as
               ELASTO-PLASTIC friction -- a stiff spring to a sticking anchor,
               which slides once the spring exceeds the local limit -- and put
               onto the two INPUT SHAFTS through the differential's mixing.
               Not MuJoCo's own frictionloss on `ring_spin`: that joint is
               light and the soft friction constraint lets it creep at
               0.13-0.3 rad/s under the full bump at any solref, so a 1 % diff
               creep never stalled at all. Stick-slip is the whole point.
  roller_slop  Each rigid roller equality becomes a tendon
               (roller_spin_i - k_roller * ring_spin) with a weak centring
               spring and hard limits at the ends of the measured play. Pure
               model structure: no per-step Python.

THE HOOK. MuJoCo cannot express the firmware loop natively (a duty clip inside
the sum, delays), so `DrivetrainSim.pre_step(data)` must run immediately
before EVERY `mj_step` -- the env's substep loop and teleop's physics loop both
call it. It detects a rewound `data.time` and resets itself, which covers the
viewer's own Backspace. A loop that steps physics without calling it gets
drives that never move: loud, not subtle.

Not under hw/: this is simulator-only and imports MuJoCo.
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import mujoco
import yaml

from .params import _normalize

KEY = "drivetrain_model"
DEFAULT_PATH = Path(__file__).resolve().parents[2] / "config" / "drivetrain_model.yaml"
PARTS = ("servo", "detent", "roller_slop")


# -- params ------------------------------------------------------------------

def load(path=None) -> dict:
    """The overlay file, `{value, source}` wrappers stripped."""
    with open(path or DEFAULT_PATH) as f:
        return _normalize(yaml.safe_load(f))


def with_drivetrain(params: dict, path=None, without=(), gains=None,
                    overrides=None) -> dict:
    """`params` plus the overlay under KEY. A new dict; `params` is untouched.

    `without` switches parts off by name. `gains` is a firmware (P, I) pair in
    TABLE units, replacing the overlay's `velocity_p_gain` / `velocity_i_gain`.
    `overrides` sets values after loading: a dict value merges into that part
    (`{"servo": {"coulomb_friction": [0.16, 0.21]}}`), anything else replaces
    a top-level key (`{"update_every": 2}`). For sweeps; the file stays the
    reference.
    """
    cfg = load(path)
    for part in without:
        if part not in PARTS:
            raise ValueError(f"unknown drivetrain part {part!r}; one of {PARTS}")
        cfg[part]["enabled"] = False
    if gains is not None:
        p_gain, i_gain = gains
        cfg["servo"]["velocity_p_gain"] = int(p_gain)
        cfg["servo"]["velocity_i_gain"] = int(i_gain)
    for key, value in (overrides or {}).items():
        if isinstance(value, dict):
            cfg[key].update(value)
        else:
            cfg[key] = value
    return {**params, KEY: cfg}


def from_env_config(params: dict, env: dict) -> dict:
    """The plant an RL env config (`cfg["env"]`) asks for.

    Keys: `drivetrain_model` -- true (the default overlay file), a path, or a
    RESOLVED overlay dict as an export records it; `drivetrain_without` -- parts
    to switch off; `servo_gains` -- [P, I] in table units. Absent or false is
    the ideal drivetrain.

    PARAMS THAT ALREADY CARRY AN OVERLAY WIN. That is how a caller forces a
    plant -- analysis/drivetrain_eval.py's variants -- and how the trainer,
    which resolves the overlay once up front so the digest and the export see
    it, stops every env from resolving it again.
    """
    if KEY in params:
        return params
    spec = env.get("drivetrain_model")
    if not spec:
        return params
    if isinstance(spec, dict):
        return {**params, KEY: copy.deepcopy(spec)}
    return with_drivetrain(params, None if spec is True else spec,
                           without=tuple(env.get("drivetrain_without") or ()),
                           gains=env.get("servo_gains"))


def base_params(params: dict) -> dict:
    """`params` without the overlay: the ideal plant. Identity if absent."""
    if KEY not in params:
        return params
    return {k: v for k, v in params.items() if k != KEY}


def enabled(params: dict, part: str) -> bool:
    cfg = params.get(KEY) or {}
    return bool((cfg.get(part) or {}).get("enabled", False))


def describe(params: dict) -> str:
    """One line for banners and tables."""
    if KEY not in params:
        return "ideal drivetrain"
    cfg = params[KEY]
    bits = []
    if enabled(params, "servo"):
        s = cfg["servo"]
        bits.append(f"servo P{s['velocity_p_gain']}/I{s['velocity_i_gain']}")
    if enabled(params, "detent"):
        bits.append("detent")
    if enabled(params, "roller_slop"):
        bits.append(f"roller slop +-{cfg['roller_slop']['half_play_deg']:g} deg")
    return ", ".join(bits) if bits else "overlay loaded, every part off"


# -- model structure ---------------------------------------------------------

def edit_spec(spec: mujoco.MjSpec, p: dict) -> None:
    """Apply the overlay's STRUCTURAL changes. Called last in `build_spec`."""
    cfg = p.get(KEY)
    if not cfg:
        return
    dt = p["drivetrain"]
    belt = float(dt["belt_ratio"])

    if enabled(p, "servo"):
        s = cfg["servo"]
        vmax = float(s["velocity_limit_rad_s"]) * belt
        acts = {a.name: a for a in spec.actuators}
        joints = {j.name: j for j in spec.joints}
        for tag in ("a", "b"):
            # Command holder: ctrl still means commanded input-shaft speed and
            # is still clipped to the Velocity Limit, but produces no force.
            act = acts[f"drive_{tag}"]
            act.dyntype = mujoco.mjtDyn.mjDYN_NONE
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.gainprm[:] = 0.0
            act.biastype = mujoco.mjtBias.mjBIAS_NONE
            act.biasprm[:] = 0.0
            act.ctrllimited = 1
            act.ctrlrange = [-vmax, vmax]
            act.forcelimited = 0
            # The Coulomb friction is NOT a joint frictionloss: see
            # DrivetrainSim, which applies it as a sticking friction.
            joints[f"input_{tag}_spin"].armature = float(s["rotor_inertia"]) / belt ** 2
        for tag in ("a", "b"):
            m = spec.add_actuator(name=f"drive_{tag}_motor")
            m.trntype = mujoco.mjtTrn.mjTRN_JOINT
            m.target = f"input_{tag}_spin"
            m.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            m.gainprm[0] = 1.0
            m.biastype = mujoco.mjtBias.mjBIAS_NONE
            m.ctrllimited = 0
            m.forcelimited = 0

    if enabled(p, "roller_slop"):
        r = cfg["roller_slop"]
        k = float(dt["k_roller"])
        half = math.radians(float(r["half_play_deg"]))
        rigid = [e for e in spec.equalities
                 if e.type == mujoco.mjtEq.mjEQ_JOINT
                 and e.name1.startswith("roller_spin_") and e.name2 == "ring_spin"]
        for e in rigid:
            joint = e.name1
            spec.delete(e)
            t = spec.add_tendon(name=f"slop_{joint}")
            t.wrap_joint(joint, 1.0)
            t.wrap_joint("ring_spin", -k)
            t.limited = 1
            t.range = [-half, half]
            t.springlength = [0.0, 0.0]
            t.stiffness = [float(r["centring_stiffness"]), 0.0, 0.0]
            t.damping = [float(r["damping"]), 0.0, 0.0]
            t.solref_limit = [float(r["wall_timeconst_s"]), 1.0]


# -- runtime -----------------------------------------------------------------

class DrivetrainSim:
    """Per-step state for the parts MuJoCo cannot express: servo and detent.

    Build with `attach`, call `reset(data, rng)` alongside any hand-rolled
    qpos/qvel reset, and `pre_step(data)` before every `mj_step`.
    """

    @classmethod
    def attach(cls, model, params: dict):
        if not (enabled(params, "servo") or enabled(params, "detent")):
            return None
        return cls(model, params)

    def __init__(self, model, params: dict):
        cfg = params[KEY]
        self.model = model
        self.h = float(model.opt.timestep)
        self.belt = float(params["drivetrain"]["belt_ratio"])
        self.servo = enabled(params, "servo")
        self.detent = enabled(params, "detent")
        # The firmware and the frictions run every `update_every` physics
        # steps and HOLD their outputs in between (ctrl and qfrc_applied
        # persist in data). Delays, filter and integral all use the firmware
        # period, so the loop means the same thing at any setting.
        self.every = max(1, int(cfg.get("update_every", 1)))
        self.h_fw = self.h * self.every
        if self.servo:
            s = cfg["servo"]
            srv = params["servos"]["xc430_w150"]
            self.stall = float(srv["stall_torque"])
            self.w0 = float(srv["no_load_rpm"]) * 2 * math.pi / 60
            self.kp = float(s["kp_per_unit"]) * float(s["velocity_p_gain"])
            self.ki = float(s["ki_per_unit"]) * float(s["velocity_i_gain"])
            self.vlim = float(s["velocity_limit_rad_s"])
            tau = float(s["velocity_filter_s"])
            self.alpha = self.h_fw / (tau + self.h_fw) if tau > 0 else 1.0
            # Ring buffers of length delay+1: write at k, read at k+1.
            self.nc = max(0, round(float(s["command_delay_s"]) / self.h_fw)) + 1
            self.nm = max(0, round(float(s["measurement_delay_s"]) / self.h_fw)) + 1
            self.cmd_id = (model.actuator("drive_a").id, model.actuator("drive_b").id)
            self.motor_id = (model.actuator("drive_a_motor").id,
                             model.actuator("drive_b_motor").id)
            self.qadr = (int(model.joint("input_a_spin").qposadr[0]),
                         int(model.joint("input_b_spin").qposadr[0]))
            self.dadr = (int(model.joint("input_a_spin").dofadr[0]),
                         int(model.joint("input_b_spin").dofadr[0]))
            self.duty = [0.0, 0.0]
            # Coulomb friction at each input shaft, elasto-plastic like the
            # detent and for the same reason: a MuJoCo frictionloss on these
            # joints creeps (solreffriction 0.02) and gives no static floor,
            # and without one the detent never stalls a 1 % creep.
            # One number, or [side a, side b]: servo 102's side measures ~25 %
            # stiffer, and a pair is how that asymmetry gets into the plant.
            fc = s["coulomb_friction"]
            fc = (fc, fc) if isinstance(fc, (int, float)) else tuple(fc)
            self.fc_in = (float(fc[0]) / self.belt, float(fc[1]) / self.belt)
            self._fc_nominal = self.fc_in
            # Supply voltage over the 12 V the datasheet quotes; see set_supply.
            self.supply = 1.0
            self.k_in = float(s["friction_stick_stiffness"])
            self.c_in = float(s["friction_stick_damping"])
        if self.detent:
            d = cfg["detent"]
            self.period = math.radians(float(d["period_deg"]))
            self.width = math.radians(float(d["width_deg"]))
            self.fmax = float(d["friction"])
            self.phase = math.radians(float(d.get("phase_deg", 0.0)))
            self.k_stick = float(d["stick_stiffness"])
            self.c_stick = float(d["stick_damping"])
            # ring-vs-hub = ring_abs - hub, both linear in the inputs; the
            # same coefficients map a force on it back onto the inputs.
            dt = params["drivetrain"]
            self.ga = float(dt["mix_ring_a"]) - float(dt["mix_hub_a"])
            self.gb = float(dt["mix_ring_b"]) - float(dt["mix_hub_b"])
            self.detent_force = 0.0
        self.in_q = (int(model.joint("input_a_spin").qposadr[0]),
                     int(model.joint("input_b_spin").qposadr[0]))
        self.in_d = (int(model.joint("input_a_spin").dofadr[0]),
                     int(model.joint("input_b_spin").dofadr[0]))
        self._ready = False
        self._t_last = -math.inf

    def set_supply(self, scale: float) -> None:
        """Battery voltage as a fraction of 12 V. Stall torque and no-load speed
        both scale with it -- torque = stall (scale * duty - w / w0) -- while
        the firmware gains and the Velocity Limit register do not. Held until
        set again; `reset` does not touch it."""
        if self.servo:
            self.supply = float(scale)

    def set_friction_scale(self, scales) -> None:
        """Scale each input shaft's Coulomb friction, [side a, side b], against
        the overlay's value. Held until set again."""
        if self.servo:
            a, b = (float(x) for x in scales)
            self.fc_in = (self._fc_nominal[0] * a, self._fc_nominal[1] * b)

    def reset(self, data, rng=None) -> None:
        """Zero the loop's memory at the CURRENT state: no integral error, no
        pending command, filters settled on the present shaft speed. `rng`
        re-draws the detent phase; None keeps the configured one."""
        if self.servo:
            self._k_c = 0
            self._k_m = 0
            self._ref, self._wf = [0.0, 0.0], [0.0, 0.0]
            self._cmd = [[0.0] * self.nc, [0.0] * self.nc]
            self._th = [[0.0] * self.nm, [0.0] * self.nm]
            self._w = [[0.0] * self.nm, [0.0] * self.nm]
            for i in (0, 1):
                th = float(data.qpos[self.qadr[i]]) / self.belt
                w = float(data.qvel[self.dadr[i]]) / self.belt
                self._ref[i] = th
                self._wf[i] = w
                self._th[i] = [th] * self.nm
                self._w[i] = [w] * self.nm
                data.ctrl[self.motor_id[i]] = 0.0
                self.duty[i] = 0.0
            self._fa = [float(data.qpos[q]) for q in self.in_q]
        if self.detent:
            if rng is not None:
                self.phase = float(rng.uniform(0.0, self.period))
            self._anchor = (self.ga * float(data.qpos[self.in_q[0]])
                            + self.gb * float(data.qpos[self.in_q[1]]))
            self.detent_force = 0.0
        data.qfrc_applied[self.in_d[0]] = 0.0
        data.qfrc_applied[self.in_d[1]] = 0.0
        self._t_last = float(data.time)
        self._ready = True
        self._n = self.every - 1        # the first pre_step updates

    def pre_step(self, data) -> None:
        t = data.time
        if not self._ready or t < self._t_last:
            self.reset(data)
        self._t_last = t
        self._n += 1
        fw = self._n >= self.every        # a firmware tick this step?
        if fw:
            self._n = 0
        fric = [0.0, 0.0]
        if self.servo:
            qpos, qvel = data.qpos, data.qvel
            if fw:
                ctrl = data.ctrl
                h, belt, vlim = self.h_fw, self.belt, self.vlim
                kp, ki, alpha = self.kp, self.ki, self.alpha
                kc, kc1 = self._k_c, (self._k_c + 1) % self.nc
                km, km1 = self._k_m, (self._k_m + 1) % self.nm
                for i in (0, 1):
                    v = float(ctrl[self.cmd_id[i]]) / belt
                    v = vlim if v > vlim else (-vlim if v < -vlim else v)
                    buf = self._cmd[i]
                    buf[kc] = v
                    v_ref = buf[kc1]
                    th = float(qpos[self.qadr[i]]) / belt
                    w = float(qvel[self.dadr[i]]) / belt
                    bt, bw = self._th[i], self._w[i]
                    bt[km] = th
                    bw[km] = w
                    wf = self._wf[i] + alpha * (bw[km1] - self._wf[i])
                    self._wf[i] = wf
                    ref = self._ref[i] + v_ref * h
                    duty = kp * (v_ref - wf) + ki * (ref - bt[km1])
                    # Anti-windup by back-calculation: hold the integral where it
                    # just saturates. The firmware's own rule is unmeasured.
                    if duty > 1.0:
                        if ki > 0.0:
                            ref -= (duty - 1.0) / ki
                        duty = 1.0
                    elif duty < -1.0:
                        if ki > 0.0:
                            ref -= (duty + 1.0) / ki
                        duty = -1.0
                    self._ref[i] = ref
                    self.duty[i] = duty
                    ctrl[self.motor_id[i]] = (self.stall * (self.supply * duty - w / self.w0)
                                              / belt)
                self._k_c, self._k_m = kc1, km1
            # Sticking Coulomb friction on each input shaft, EVERY physics
            # step whatever update_every says: held across steps, the stiff
            # stick spring measurably changed small-amplitude tracking.
            for i in (0, 1):
                x = float(qpos[self.in_q[i]])
                f = -self.k_in * (x - self._fa[i]) - self.c_in * float(qvel[self.in_d[i]])
                fc_i = self.fc_in[i]
                if f > fc_i or f < -fc_i:
                    f = fc_i if f > 0.0 else -fc_i
                    self._fa[i] = x + f / self.k_in
                fric[i] = f
        if self.detent:
            qpos, qvel = data.qpos, data.qvel
            ga, gb = self.ga, self.gb
            phi = ga * float(qpos[self.in_q[0]]) + gb * float(qpos[self.in_q[1]])
            x = (phi - self.phase) % self.period - 0.5 * self.period
            q = 0.0
            if abs(x) < 0.5 * self.width:
                c = math.cos(math.pi * x / self.width)
                limit = self.fmax * c * c
                vphi = ga * float(qvel[self.in_d[0]]) + gb * float(qvel[self.in_d[1]])
                q = -self.k_stick * (phi - self._anchor) - self.c_stick * vphi
                if q > limit or q < -limit:
                    # Slipping: kinetic = static, and the anchor is dragged so
                    # the spring sits exactly at the limit.
                    q = limit if q > 0.0 else -limit
                    self._anchor = phi + q / self.k_stick
            else:
                self._anchor = phi
            self.detent_force = q
            fric[0] += ga * q
            fric[1] += gb * q
        if self.servo or self.detent:
            data.qfrc_applied[self.in_d[0]] = fric[0]
            data.qfrc_applied[self.in_d[1]] = fric[1]
