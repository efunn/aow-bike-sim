"""The bike <-> ground-station packets: one schema, both directions, no mujoco.

Bike -> station is JSON (`build`, below). Station -> bike is 12 packed bytes
(`encode_command` / `decode_command`).

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
    righting_   MEASURED, the righting servo's encoder. The LINKAGE it drives
    pos         is drawn from the config -- see the swing note below.

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
  * the front wheel is rolled from forward speed and its own radius;
  * the co-rotating four-bar is solved in closed form from the righting
    servo's measured angle (`build_model.SwingLinkageSolver`). A MIXED case,
    and the mix is the point: the angle is a real encoder reading and every
    link hanging off it is geometry. The loop is closed by `mjEQ_CONNECT`,
    which only the constraint solver satisfies and only during a dynamics
    step -- so a mirror that never steps has to write every joint itself or
    watch the mechanism come apart on screen.
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
    "jitter_ms", "dt_ms", "cuts", "righting", "righting_pos",
    "righting_current", "servos", "run", "log",
)

# The effort slot's key names ITS UNIT, because address 126 means different
# things by model (hw/dynamixel.HEALTH_BLOCK): a drive's 0.3 is 30 % of max
# torque and the steer's 0.3 is 300 mA. A single "effort" key would put both
# in one column and invite comparing them.
EFFORT_KEY = {"A": "A", "frac_max_torque": "load"}


def servo_health(health: dict, effort_unit: dict) -> dict:
    """{role: {HEALTH_BLOCK label: value}} -> the packet's `servos` field.

    Short keys, since this is five numbers times four servos every tick:
    `pwm` duty [-1, 1], `A` or `load` (see EFFORT_KEY), `V`, `C` [degC], and
    `err` -- the raw Hardware Error Status bits, 0 when healthy, decoded by
    `hw.dynamixel.describe_hardware_error` where someone reads it.
    """
    out = {}
    for role, h in health.items():
        if not h:
            continue
        row = {"pwm": round(float(h["Present PWM"]), 3)}
        row[EFFORT_KEY.get(effort_unit.get(role), "effort")] = round(
            float(h["effort"]), 3)
        row["V"] = round(float(h["Present Input Voltage"]), 1)
        row["C"] = int(h["Present Temperature"])
        row["err"] = int(h["Hardware Error Status"])
        out[role] = row
    return out


def build(*, t, state, quat, gyro, v_world, pos, steer, w_shaft, shaft, psi,
          cmd_v_world, cmd_psi, roll, roll_rate, volts, vlat_conf, qos,
          jitter_ms, dt_ms, cuts, righting, righting_pos,
          righting_current, servos=None, run=None, log=None) -> dict:
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
        # TWO NUMBERS, AND THE DIFFERENCE IS THE POINT. `righting` is the goal
        # the operator last asked for; `righting_pos` is where the servo
        # actually is. They separate exactly when the mechanism is loaded past
        # what Goal Current allows -- which is the whole reason that servo runs
        # in current-based position mode, and the thing a station tuning
        # `righting_current` needs to see. Rendering the goal would draw a wing
        # that always arrives.
        "righting": None if righting is None else round(float(righting), 4),
        "righting_pos": (None if righting_pos is None
                         else round(float(righting_pos), 4)),
        "righting_current": righting_current,
        # -- per servo, every tick (see `servo_health`). Already rounded.
        "servos": servos or {},
        # The onboard record this tick is being written into -- the name of
        # its directory under traces/bike/ on the Pi, or None when the bike
        # is not recording. What the operator writes down to find it later.
        "run": run,
        # The bike's own messages, see `EventLog`. Written ONLY on the bike.
        "log": log or [],
    }


class EventLog:
    """The bike's messages to the station: `[[seq, kind, text], ...]`.

    WHY. What happens to the bike -- a cut, a lost link, a hardware error, a
    failsafe -- is decided ON THE BIKE, and used to be printed only to the
    bike's own console, which the operator is not looking at. The station
    re-derived some of it by watching `state` change, which is a second copy
    of the decision in the place that is not the authority. Now the bike says
    it once, in words, and the station prints what it is told.

    LOSS-PROOF WITHOUT ACKS. Each message rides in EVERY packet for `hold_s`
    (~500 packets at 100 Hz), and the station prints each `seq` once. 5 s
    rather than 2 because the message that matters most after a dropout is
    "LINK LOST", and at 2 s it only just survived a 3 s outage (measured on
    the Pi): it is made 1 s into the silence and has to outlast the rest. After
    `hold_s` a message drops out, so a quiet bike sends `"log": []` -- the
    steady-state cost is ~11 bytes, and a burst is bounded by `keep`.

    `kind` is a short tag a station can act on (colour, sound) without
    parsing the text: cut, rearm, link, error, failsafe. Plain text rather
    than codes because codes need the same table on both ends, which is the
    two-copies problem this exists to remove. Shorten later if bytes matter.
    """

    def __init__(self, hold_s: float = 5.0, keep: int = 4, clock=None):
        import time as _time
        from collections import deque
        self.hold_s, self.clock = float(hold_s), clock or _time.monotonic
        self.seq = 0
        self._q = deque(maxlen=int(keep))

    def add(self, kind: str, text: str) -> None:
        self.seq += 1
        self._q.append((self.clock(), [self.seq, kind, text]))

    def recent(self) -> list:
        now = self.clock()
        return [m for t, m in self._q if now - t <= self.hold_s]


def new_messages(tel: dict, last_seq: int) -> tuple[list, int]:
    """-> (messages with seq > last_seq, the new last_seq). The station's
    half of `EventLog`: each message printed once, however many packets
    carried it. A bike that RESTARTED counts from 1 again, so a seq far
    below the last one seen resets the count rather than going silent."""
    got = [m for m in (tel.get("log") or []) if isinstance(m, list) and len(m) == 3]
    if got and max(m[0] for m in got) < last_seq:
        last_seq = 0                         # the bike restarted
    fresh = [m for m in got if m[0] > last_seq]
    return fresh, max([last_seq] + [m[0] for m in fresh])


SERVO_ORDER = ("drive_a", "drive_b", "steer", "righting")


def servo_errors(tel: dict) -> dict:
    """{role: "overload, ..."} for every servo reporting a hardware error.
    Empty when all is well -- the common case, and the one to keep quiet."""
    from .dynamixel import describe_hardware_error
    return {role: describe_hardware_error(int(row.get("err", 0)))
            for role, row in (tel.get("servos") or {}).items()
            if row.get("err")}


def status_text(tel: dict) -> tuple[str, str]:
    """The station's readout: (labels, values), one line each, for the
    viewer's two-column text overlay. Pure, so it is tested without a window.

    What is here is what the operator cannot see by looking at the bike: the
    pack, the loop's health, which record this is going into, and per servo
    temperature, effort and duty. A hardware error REPLACES that servo's line,
    because a latched error is the one thing on this list that means stop.
    """
    left, right = [], []

    def row(k, v):
        left.append(k)
        right.append(v)

    row("state", f"{tel.get('state', '?')}   cuts {tel.get('cuts', 0)}")
    q = tel.get("qos")
    row("pack", f"{tel.get('volts', float('nan')):.1f} V   jitter "
                f"{tel.get('jitter_ms') or 0:.2f} ms   qos {'-' if q is None else q}")
    row("record", tel.get("run") or "off")
    log = tel.get("log") or []
    if log:
        row("bike", str(log[-1][2]))
    errs = servo_errors(tel)
    servos = tel.get("servos") or {}
    for role in [r for r in SERVO_ORDER if r in servos] + \
            sorted(set(servos) - set(SERVO_ORDER)):
        r = servos[role]
        if role in errs:
            row(role, f"HARDWARE ERROR: {errs[role]}")
            continue
        eff = (f"{r['A']:+.2f} A" if "A" in r
               else f"load {100 * r['load']:+4.0f}%" if "load" in r else "")
        row(role, f"{r.get('C', 0):3d} C   {eff}   pwm {100 * r.get('pwm', 0):+4.0f}%")
    return "\n".join(left), "\n".join(right)


# ---------------------------------------------------------------------------
# The OTHER direction: station -> bike, as 12 packed bytes.
#
#   byte  0      version (COMMAND_VERSION)
#   byte  1      rearm_n, uint8 -- a press COUNT, wraps at 256; the bike acts
#                on a change, so the wrap is harmless
#   bytes 2-5    v_cmd_world x, y     int16  [mm/s]    +-32.7 m/s
#   bytes 6-7    psi_cmd, WRAPPED     int16  [1e-4 rad]  +-pi fits in +-31416;
#                ABSENT = the station does not know the bike's heading yet
#   bytes 8-9    righting_rad         int16  [1e-3 rad]  +-32.7 rad, ~5 turns
#   bytes 10-11  righting_current     int16  [raw counts]
#
# ABSENT (-32768) in a righting slot means "not commanded": the bike then
# writes no goal at all, which is different from a goal of zero.
#
# WHY PACKED, when this was JSON on purpose: the operator's call (2026-09-18),
# while the protocol is being reworked anyway. JSON was 76-150 B. On wifi the
# saving is small -- 28 B of IP/UDP header and ~30 B of 802.11 framing ride
# every packet whatever the payload -- but the format now has a fixed layout
# and a version byte, which a JSON dict never had.
#
# The heading goes WRAPPED because the bike only ever uses it as
# wrap_pi(psi_cmd - psi) (general_spec.build_obs), so a wrapped command is
# the same command. Resolution: 1 mm/s, 0.006 deg, 0.057 deg for the wing
# (the servo resolves 0.088).
# ---------------------------------------------------------------------------

COMMAND_VERSION = 1
_CMD = __import__("struct").Struct("<BBhhhhh")
COMMAND_BYTES = _CMD.size          # 12
ABSENT = -32768


class CommandFormatError(ValueError):
    """A datagram that is not this command format -- most likely a station
    from before the packed format, still sending JSON."""


def _i16(x: float) -> int:
    return int(max(-32767, min(32767, round(x))))


def encode_command(pkt: dict) -> bytes:
    """`OperatorState.packet()` (a dict) -> the 12-byte datagram."""
    import math
    vx, vy = pkt.get("v_cmd_world", (0.0, 0.0))
    psi = pkt.get("psi_cmd")
    if psi is not None:
        psi = math.atan2(math.sin(float(psi)), math.cos(float(psi)))
    r = pkt.get("righting_rad")
    cur = pkt.get("righting_current")
    return _CMD.pack(COMMAND_VERSION, int(pkt.get("rearm_n", 0)) & 0xFF,
                     _i16(vx * 1e3), _i16(vy * 1e3),
                     ABSENT if psi is None else _i16(psi * 1e4),
                     ABSENT if r is None else _i16(r * 1e3),
                     ABSENT if cur is None else _i16(cur))


def decode_command(data: bytes) -> dict:
    """The 12-byte datagram -> the same dict shape the bike always read, so
    nothing downstream of the socket changed. Raises CommandFormatError."""
    if data[:1] == b"{":
        raise CommandFormatError(
            "the station is sending JSON -- it predates the packed command "
            "format. Update the station (this repo, same commit as the bike).")
    if len(data) != COMMAND_BYTES:
        raise CommandFormatError(f"{len(data)} B, expected {COMMAND_BYTES}")
    ver, n, vx, vy, psi, r, cur = _CMD.unpack(data)
    if ver != COMMAND_VERSION:
        raise CommandFormatError(
            f"command format v{ver}, this bike reads v{COMMAND_VERSION}")
    out = {"v_cmd_world": [vx * 1e-3, vy * 1e-3], "rearm_n": n}
    if psi != ABSENT:
        out["psi_cmd"] = psi * 1e-4
    if r != ABSENT:
        out["righting_rad"] = r * 1e-3
    if cur != ABSENT:
        out["righting_current"] = cur
    return out


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
        # RIGHTING PANELS, for grounding. Box geoms, so unlike the wheels their
        # lowest point is a CORNER and depends on attitude -- see
        # `ground_the_bike`. Collected by name because every righting mechanism
        # names its panels the same way (`wing_left`, `swing_wing_right`), and
        # an empty tuple is the normal case: a bike with no mechanism built.
        self.panels = []
        for g in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            if (name and ("wing" in name)
                    and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX):
                self.panels.append((g, model.geom_size[g].copy()))


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


def ground_the_bike(model, data, params: dict, adr=None) -> float:
    """Raise/lower the chassis so its LOWEST point touches z = 0. -> the shift.

    Call after the attitude is set and `mj_kinematics` has run; the caller must
    run kinematics again afterwards, which `apply_pose` does.

    A DRAWING, not a contact solve. It exists because the bike sends no ride
    height, so the mirror has to choose one -- and a constant is wrong as soon
    as the bike pitches, which it does on every wheelie.

    THE WHEELS are exact and need no geometry query: the rear omni wheel and
    the front tyre are both surfaces of revolution about their own axles, so
    the lowest point is `centre_z - radius` whatever the attitude.

    THE RIGHTING PANELS are not. They are boxes, so the lowest point is a
    CORNER, and which corner it is changes with roll -- which is the whole
    reason they need including. Measured on the co-rotating four-bar: upright,
    a fully deployed wing still clears the wheel contact by 5.7 mm and changes
    nothing here; rolled 61 deg it reaches 78 mm BELOW it, which is what was
    being drawn as a wing sunk through the floor. Transforming eight corners
    per panel is a few microseconds and happens once a frame.

    So a rolled bike with a wing out now rests ON THE WING, which is what the
    mechanism is for. `adr` is optional only so a caller that has not built a
    `PoseAdr` still gets the wheels; pass it to include the panels.

    Consequence worth knowing, and unchanged: the rendered bike can never sink
    into the floor and never floats, so it CANNOT show a wheel lifting off. A
    wheelie reads as a pitch about the rear contact, which is what it is.
    """
    import numpy as np
    r_rear = float(params["omni_wheel"]["outer_radius"])
    r_front = float(params["bike"]["front_wheel"]["radius"])
    low = min(float(data.body("aow_hub").xpos[2]) - r_rear,
              float(data.body("front_wheel").xpos[2]) - r_front)
    for gid, size in (getattr(adr, "panels", None) or ()):
        pos = data.geom_xpos[gid]
        rot = data.geom_xmat[gid].reshape(3, 3)
        # Only the z row of the rotation matters, so the lowest corner is the
        # centre minus the sum of |projection| of each half-extent onto z --
        # no need to enumerate the eight of them.
        low = min(low, float(pos[2]) - float(np.abs(rot[2]) @ size))
    data.qpos[2] -= low
    return -low


# The name this had while it only knew about wheels. Kept because it is the
# kind of helper a bench script imports.
ground_the_wheels = ground_the_bike


def apply_pose(model, data, tel: dict, adr: PoseAdr, params: dict,
               shaft_angle=(0.0, 0.0), rest_z: float | None = None,
               dt: float = 0.0, swing_pose=None) -> tuple:
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

    # -- the righting mechanism ---------------------------------------------
    #
    # `swing_pose` maps a CRANK TRAVEL to every joint in the four-bar. It has
    # to write all of them: the loop is closed by `mjEQ_CONNECT`, and an
    # equality is only satisfied by the constraint solver during a dynamics
    # step. The mirror never steps, so writing the crank alone and calling
    # `mj_forward` would leave the couplers and wings where they were and the
    # mechanism would visibly come apart. See build_model.SwingLinkageSolver.
    #
    # MEASURED where the servo is, DRAWN where the wings are: the angle is a
    # real encoder reading, and the linkage hanging off it is geometry from the
    # config. On a bike whose wings are not built yet, that is a picture of
    # what this servo angle WOULD do.
    if swing_pose is not None and "swing_crank_joint" in q:
        got = tel.get("righting_pos")
        if got is None:
            got = tel.get("righting")         # the goal, if there is no reading
        if got is not None:
            angles = swing_pose(float(got))
            if angles is not None:            # None = past the assembly limit
                for name, val in angles.items():
                    if name in q:
                        data.qpos[q[name]] = val
                        data.qvel[d[name]] = 0.0

    if rest_z is not None:
        import mujoco
        mujoco.mj_kinematics(model, data)     # place the parts, then settle
        ground_the_bike(model, data, params, adr)
    return a, b
