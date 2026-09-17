"""The bike -> ground-station packet: one schema, both directions, no mujoco.

WHY THIS IS ITS OWN MODULE. The encoder runs on the Pi inside the control
thread and must not import mujoco (`tests/test_hw_no_mujoco.py` pins that); the
decoder runs on the laptop inside a MuJoCo viewer. Putting them in the same
file anyway is deliberate -- the failure this whole module exists to prevent is
the two halves drifting, and a field that one side writes and the other never
reads is a control nobody has. `build` and `apply_pose` sit fifty lines apart
so that drift is visible, and `test_every_telemetry_field_is_read_by_the_mirror`
makes it an error rather than a matter of noticing.

The mujoco-free half is `build` and the constants. `apply_pose` takes an
already-built `model`/`data` from its caller and touches only `qpos`/`qvel`, so
the import stays on the laptop's side of the fence.

WHAT IS MEASURED AND WHAT IS DRAWN. The bike does not know most of its own
pose, and the packet says so rather than presenting a plausible number:

    quat        MEASURED, the AHRS through the mount calibration
    gyro        MEASURED, the AHRS
    steer       MEASURED, the servo's encoder minus steer_zero
    w_shaft     MEASURED, the drive servos, differenced over Realtime Tick
    v_world     ESTIMATED, VelocityEstimator -- odometry plus the front-wheel
                constraint, and `vlat_conf` says how much to believe the
                lateral half
    pos         DEAD-RECKONED and drifts without bound. It is qpos[:2] as the
                onboard controller has it, sent for agreement rather than for
                truth. NOTHING may treat it as position.
    z, wheels,  NOT KNOWN AT ALL. The bike has no ride-height sensor and no
    rollers     encoder on the front wheel or the omni internals.

`apply_pose` fills the last group so the picture is not broken, and every one
of those is a DRAWING, not a reading:

  * ride height is chosen so the LOWER WHEEL sits on the floor. A fixed
    height is wrong the moment the bike pitches: the attitude rotates the
    whole body about its origin, so a nose-down attitude drives the front
    wheel through the ground and a wheelie floats the rear. Both wheels are
    surfaces of revolution about their own axes, so "lowest point" is exactly
    `centre_z - radius` for each, with no bounding-box estimate involved;
  * the rear wheel's hub, ring and eight rollers are a CLOSED FORM of the two
    input-shaft angles -- the gearbox is a pair of linear tendon constraints
    (`build_model._aow_assembly`), so hub = mix_hub_a*a + mix_hub_b*b and
    ring_abs = mix_ring_a*a + mix_ring_b*b exactly, with no solver needed.
    That is why the wheel can be drawn faithfully from two numbers;
  * the input-shaft angles are MEASURED and sent (`shaft`), summed on the bike
    from the same unwrapped per-tick delta the velocity is differenced from.
    The station integrating `w_shaft` instead -- which is what this did first
    -- is lossy three ways: `vel` is filtered so it lags, telemetry does not
    carry every tick, and a dropped packet is a permanently lost increment.
    Integration survives only as the fallback for a bike too old to send it;
  * the front wheel is rolled from forward speed and its own radius.
"""

from __future__ import annotations

# Bump when a field changes MEANING or disappears. Adding a field does not
# need a bump -- `apply_pose` tolerates absence so a newer bike can talk to an
# older station -- but a station reading a renamed field silently gets zero,
# which is exactly the class of bug the version check exists to turn into an
# error at connect time instead of a wrong picture ten minutes in.
SCHEMA_VERSION = 2

# Every key `build` emits. The test walks this, so a field added without a
# reader (or read without being sent) fails rather than rotting.
FIELDS = (
    "v", "t", "state",
    "quat", "gyro", "v_world", "pos", "steer", "w_shaft", "shaft",
    "cmd_v_world", "cmd_psi", "psi",
    "roll", "roll_rate", "volts", "vlat_conf", "qos",
    "jitter_ms", "dt_ms", "cuts", "righting", "righting_current",
)


def build(*, t, state, quat, gyro, v_world, pos, steer, w_shaft, shaft, psi,
          cmd_v_world, cmd_psi, roll, roll_rate, volts, vlat_conf, qos,
          jitter_ms, dt_ms, cuts, righting, righting_current) -> dict:
    """The packet. Pure, keyword-only, and rounded HERE rather than at the
    call site so the wire format is one decision in one place.

    Keyword-only on purpose: this has twenty fields, half of them
    three-element vectors of similar magnitude, and a positional call that
    transposes `gyro` and `v_world` would produce a bike that renders almost
    correctly.

    `roll`/`roll_rate` are derivable from `quat`/`gyro` and are sent anyway --
    the console line and the station's status line both want them every frame,
    and recomputing a number the bike already has in order to print it is the
    kind of duplication that eventually disagrees. They are DISPLAY COPIES: if
    one ever contradicts the quaternion, the quaternion is right.
    """
    return {
        "v": SCHEMA_VERSION,
        "t": round(float(t), 3),
        "state": state,
        # -- pose, for the mirror ------------------------------------------
        "quat": [round(float(x), 6) for x in quat],
        "gyro": [round(float(x), 4) for x in gyro],
        "v_world": [round(float(x), 3) for x in v_world],
        "pos": [round(float(x), 3) for x in pos],
        "steer": round(float(steer), 4),
        "w_shaft": [round(float(x), 3) for x in w_shaft],
        # ABSOLUTE input-shaft angle since the bus opened, summed on the bike
        # from the same per-tick delta the velocity is differenced from. Sent
        # rather than left to the station to integrate, because integrating a
        # FILTERED rate over whatever fraction of ticks happened to arrive is
        # lossy in three ways at once and drifts without bound.
        "shaft": [round(float(x), 4) for x in shaft],
        "psi": round(float(psi), 4),
        # -- what the BIKE believes it was told, not what the station
        #    remembers sending. They differ exactly when a packet was dropped,
        #    which is the moment the difference matters.
        "cmd_v_world": [round(float(x), 3) for x in cmd_v_world],
        "cmd_psi": round(float(cmd_psi), 4),
        # -- health ---------------------------------------------------------
        "roll": round(float(roll), 4),
        "roll_rate": round(float(roll_rate), 3),
        "volts": round(float(volts), 1),
        "vlat_conf": round(float(vlat_conf), 2),
        "qos": None if qos is None else int(qos),
        "jitter_ms": jitter_ms,
        "dt_ms": round(float(dt_ms), 2),
        "cuts": int(cuts),
        "righting": None if righting is None else round(float(righting), 4),
        "righting_current": righting_current,
    }


class SchemaMismatch(RuntimeError):
    """The bike and the station disagree about the packet. Raised at the FIRST
    packet rather than tolerated, because the failure mode of carrying on is a
    mirror that renders a confidently wrong bike."""


def check_version(tel: dict) -> None:
    """Raise unless `tel` is this schema. An EMPTY dict passes: the bike's link
    thread starts before its control loop, so a station can legitimately see
    `{}` first, and calling that a version mismatch sends the reader off to
    re-sync a Pi that is fine. A packet with fields but the wrong `v` is the
    real thing this catches."""
    if not tel:
        return
    got = tel.get("v")
    if got != SCHEMA_VERSION:
        raise SchemaMismatch(
            f"telemetry schema v{got} from the bike, v{SCHEMA_VERSION} here. "
            "Re-sync the Pi (rsync) -- this is a stale deploy, not a bug.")


class PoseAdr:
    """qpos/qvel addresses BY JOINT NAME, resolved once.

    By name because the alternative -- hardcoding qpos indices -- breaks
    silently: `build_model` emits a different `nq` with payload, hockey or a
    righting mechanism attached, and an index that shifted would render the
    steer angle into a roller.
    """

    def __init__(self, model):
        import mujoco
        self.m = model
        self.q, self.d = {}, {}
        for j in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if name:
                self.q[name] = int(model.jnt_qposadr[j])
                self.d[name] = int(model.jnt_dofadr[j])
        self.rollers = sorted(n for n in self.q if n.startswith("roller_spin_"))


def gearbox_mix(params: dict) -> dict:
    """The rear assembly's kinematics, as `build_model` wires them.

    `_aow_assembly` builds two tendon equalities:

        hub_spin            = mix_hub_a  * a + mix_hub_b  * b
        hub_spin + ring_rel = mix_ring_a * a + mix_ring_b * b
        roller_spin_i       = k_roller * ring_rel

    so the whole wheel -- hub, ring and eight rollers -- follows from the two
    input-shaft angles in closed form. Read from params rather than copied,
    because these are the same numbers the model is built from and a second
    copy is a second thing to keep in step.
    """
    dt = params["drivetrain"]
    return {k: float(dt[k]) for k in
            ("mix_hub_a", "mix_hub_b", "mix_ring_a", "mix_ring_b", "k_roller")}


def ground_the_wheels(model, data, params: dict) -> float:
    """Raise/lower the chassis so the LOWER wheel touches z = 0. -> the shift.

    Call after the attitude is set and `mj_kinematics` has run; the caller must
    run kinematics again afterwards, which `apply_pose` does.

    A DRAWING, not a contact solve. It exists because the bike sends no ride
    height, so the mirror has to choose one -- and a constant is wrong as soon
    as the bike pitches, which it does on every wheelie. Using each wheel's own
    centre and radius is exact for a surface of revolution and needs no
    geometry query: the rear omni wheel and the front tyre are both round about
    their axles, whatever the attitude.

    Consequence worth knowing: the rendered bike can never sink into the floor
    and never floats, so it CANNOT show a real wheel lifting off. A wheelie
    reads as a pitch about the rear contact, which is what it is.
    """
    import numpy as np
    r_rear = float(params["omni_wheel"]["outer_radius"])
    r_front = float(params["bike"]["front_wheel"]["radius"])
    low = min(float(data.body("aow_hub").xpos[2]) - r_rear,
              float(data.body("front_wheel").xpos[2]) - r_front)
    data.qpos[2] -= low
    return -low


def apply_pose(model, data, tel: dict, adr: PoseAdr, params: dict,
               shaft_angle=(0.0, 0.0), rest_z: float | None = None,
               dt: float = 0.0) -> tuple:
    """Write one telemetry packet into `data` for RENDERING. -> new shaft angle.

    NOT a state estimator and not a physics step: it sets `qpos`/`qvel` and
    leaves the caller to `mj_forward`. Nothing here is integrated except the
    shaft and front-wheel angles, which the caller carries between frames.

    `dt` IS DISPLAY TIME -- seconds since the last call -- and not the bike's
    control period. Getting that wrong is subtle and was wrong here first
    time: `dt_ms` from the packet is the bike's 10 ms tick, and using it once
    per RENDERED FRAME integrates 60 x 10 ms per second of wall clock, so the
    wheels turned at 60% of reality. It presented as "I spun the rear wheel by
    hand and the render barely moved", which is a long way from an integrator.
    The velocities are instantaneous rates, so the time they are multiplied by
    has to be the time the viewer actually advanced.

    Absent fields are left alone rather than zeroed, so a newer bike talking to
    an older station degrades to a partly-stale picture instead of a bike that
    snaps to the origin every frame.
    """
    import numpy as np
    q, d = adr.q, adr.d
    if "quat" in tel:
        data.qpos[3:7] = tel["quat"]
    if "gyro" in tel:
        data.qvel[3:6] = tel["gyro"]
    if "v_world" in tel:
        data.qvel[0:2] = tel["v_world"]
    if "pos" in tel:
        data.qpos[0:2] = tel["pos"]
    # NOT MEASURED -- the bike has no ride-height sensor. Start from the
    # model's rest height so the first kinematics pass is sane; `ground_the_
    # wheels` below then puts the lower wheel on the floor for real.
    if rest_z is not None:
        data.qpos[2] = rest_z
    if "steer" in tel and "steer_joint" in q:
        data.qpos[q["steer_joint"]] = tel["steer"]

    w_a, w_b = tel.get("w_shaft", (0.0, 0.0))
    dt_s = float(dt)                         # DISPLAY seconds; see the docstring
    sent = tel.get("shaft")
    if sent is not None:
        a, b = float(sent[0]), float(sent[1])     # MEASURED; no drift to carry
    else:                                         # a bike older than schema v2
        a = shaft_angle[0] + w_a * dt_s
        b = shaft_angle[1] + w_b * dt_s
    mix = gearbox_mix(params)
    hub = mix["mix_hub_a"] * a + mix["mix_hub_b"] * b
    ring_abs = mix["mix_ring_a"] * a + mix["mix_ring_b"] * b
    ring_rel = ring_abs - hub                # the joint is RELATIVE to the hub
    hub_w = mix["mix_hub_a"] * w_a + mix["mix_hub_b"] * w_b
    ring_w = (mix["mix_ring_a"] * w_a + mix["mix_ring_b"] * w_b) - hub_w
    for name, val, rate in (("hub_spin", hub, hub_w),
                            ("ring_spin", ring_rel, ring_w)):
        if name in q:
            data.qpos[q[name]] = val
            data.qvel[d[name]] = rate
    for name in adr.rollers:
        data.qpos[q[name]] = mix["k_roller"] * ring_rel
        data.qvel[d[name]] = mix["k_roller"] * ring_w
    for name, val, rate in (("input_a_spin", a, w_a), ("input_b_spin", b, w_b)):
        if name in q:
            data.qpos[q[name]] = val
            data.qvel[d[name]] = rate

    # The front wheel has no encoder. Roll it from forward speed so it does not
    # skid visibly -- a drawing, and the module docstring says so.
    if "front_spin" in q:
        r = float(params["bike"]["front_wheel"]["radius"])
        speed = float(np.linalg.norm(tel.get("v_world", (0.0, 0.0))))
        data.qvel[d["front_spin"]] = speed / r if r else 0.0
        data.qpos[q["front_spin"]] += (speed / r if r else 0.0) * dt_s

    if rest_z is not None:
        import mujoco
        mujoco.mj_kinematics(model, data)     # place the wheels, then settle
        ground_the_wheels(model, data, params)
    return a, b
