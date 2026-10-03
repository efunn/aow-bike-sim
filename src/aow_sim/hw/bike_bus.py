"""The bike's servos as one device: two drives, the steer, optionally the
righting servo. A `DynamixelBus` (the generic X-series layer) underneath;
everything bike-specific is here.

Per tick: one FastSyncRead and one SyncWrite. Each servo maps its own goal
register (Goal Velocity on the drives, Goal Position on the steer and the
righting servo) to the same indirect write slot.

Timing comes from the servos: the control dt is the first drive's Realtime
Tick delta, not the host's sleep. Velocity is re-differenced from position
through `RateFilter` rather than taken from Present Velocity (~25 ms of lag);
the servo's own estimate is still read and surfaced as `*_reported`.

`BikeBus` owns every conversion between the controller's units (input-shaft
rad/s, steer-joint rad) and register counts, including the belt and steering
ratios and the drive servos' mounting signs.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from ..control.steer import XC330_COUNTS_PER_RAD, clamp_extended
from .control_table import table_by_name
from .dynamixel import (MODE_CURRENT_POSITION, MODE_EXTENDED_POSITION,
                        MODE_VELOCITY, VEL_LSB_RAD_S, VOLT_LSB, DynamixelBus,
                        IndirectMap, describe_hardware_error, pos_delta,
                        signed, tick_delta_ms)
from .rate_filter import RateFilter

# The rate the bike's sense/actuate loop runs at. `RateFilter` quantises its
# window to it and `assert_alias_margin` checks against it; the simulated
# estimator imports it so it ticks at the Pi's rate.
CONTROL_HZ_DEFAULT = 100.0

# What every servo reports each tick, in order; contiguous once indirected.
READ_BLOCK = ("Realtime Tick", "Present Position", "Present Velocity")

# Read alongside, for the record and the ground station; no controller uses
# it. After READ_BLOCK so the control fields keep their offsets. 10 + 8 read
# + 4 write = 22 of block 1's 28 bytes. "effort" is address 126 on both
# models: Present Current [A] on the XC330, Present Load [fraction of max
# torque] on the XC430; the per-id Register carries the unit.
HEALTH_BLOCK = ("Present PWM", "effort", "Present Input Voltage",
                "Present Temperature", "Hardware Error Status")

# The drivetrain overlay's firmware-gain keys and their registers. Duplicated
# from `drivetrain_model.GAIN_KEYS` because that module imports mujoco and
# nothing under hw/ may; a test asserts the two agree.
DRIVETRAIN_KEY = "drivetrain_model"
VELOCITY_GAIN_REGISTERS = {"velocity_p_gain": "Velocity P Gain",
                           "velocity_i_gain": "Velocity I Gain"}


def assert_alias_margin(params: dict, control_hz: float,
                        min_margin: float = 5.0) -> float:
    """Raise if a drive's encoder could alias at top speed; return the margin.

    `pos_delta` takes the shortest path, right only while the shaft turns
    under half a turn per sample. Fails silently otherwise (a velocity of the
    wrong sign), and the margin rests on the belt ratio and the loop rate.
    """
    v_max = float(params["control"]["drive"]["v_max"])
    r = float(params["omni_wheel"]["outer_radius"])
    belt = float(params["drivetrain"]["belt_ratio"])
    rev_s = v_max / (2 * np.pi * r) / belt      # servo revs/s at top speed
    rev_per_sample = rev_s / control_hz
    margin = 0.5 / rev_per_sample if rev_per_sample else float("inf")
    if margin < min_margin:
        raise RuntimeError(
            f"encoder aliasing margin is only {margin:.1f}x at v_max {v_max} "
            f"m/s and {control_hz:g} Hz ({rev_per_sample:.3f} rev/sample "
            f"against a 0.5 rev limit). pos_delta picks the SHORTEST path, so "
            f"below ~1x it silently reports the wrong sign. Raise control_hz, "
            f"raise belt_ratio, or lower v_max.")
    return float(margin)


def policy_servo_gains(name: str, moves_dir="moves") -> dict | None:
    """The firmware velocity gains `moves/NAME.yaml` was trained at, or None
    if it trained on the ideal drive plant (no firmware loop, so no opinion).
    Reads the yaml directly: no weights needed, and `drivetrain_model` cannot
    be imported here."""
    import yaml
    try:
        with open(Path(moves_dir) / f"{name}.yaml") as f:
            rec = (yaml.safe_load(f) or {}).get(DRIVETRAIN_KEY) or {}
    except FileNotFoundError:
        return None
    servo = rec.get("servo") or {}
    if not servo.get("enabled", False):
        return None
    got = {reg: servo[key] for key, reg in VELOCITY_GAIN_REGISTERS.items()
           if key in servo}
    return got or None


def resolve_gains(params: dict, policy: str | None = None,
                  override=None, moves_dir="moves") -> tuple:
    """-> ({role: {register: value}}, note): what to write, and from where.

    An explicit pin beats the policy's own training gain, which beats
    `control.onboard.gains` -- the same order as teleop
    (`drivetrain_model.teleop_overlay`), because a policy is specific to the
    gain it trained at. Print `note`: it says which source won.
    """
    cfg = ((params.get("control") or {}).get("onboard") or {})
    gains = {role: dict(regs) for role, regs in (cfg.get("gains") or {}).items()}

    if override is not None:
        p_gain, i_gain = override
        src = f"pinned P{int(p_gain)}/I{int(i_gain)}"
        drive = {"Velocity P Gain": int(p_gain), "Velocity I Gain": int(i_gain)}
    else:
        drive = policy_servo_gains(policy, moves_dir) if policy else None
        if drive:
            src = (f"{policy}'s own training gain "
                   f"P{drive['Velocity P Gain']}/I{drive['Velocity I Gain']}")
        elif gains.get("drive"):
            d = gains["drive"]
            src = (f"config control.onboard.gains.drive "
                   f"P{d.get('Velocity P Gain')}/I{d.get('Velocity I Gain')}")
            drive = None                     # already in `gains`
        else:
            src = ("NOTHING -- the policy trained on the ideal drive plant and "
                   "no default is configured, so the servos keep their "
                   "power-on values (factory P100/I1920)")
            drive = None
    if drive:
        gains.setdefault("drive", {}).update(drive)
    return gains, f"firmware velocity gains from {src}"


class BikeBus:
    """The bike's three servos, plus the optional righting servo, as one device.

    The righting servo (current-based position mode) shares the steer's Goal
    Position write slot, so it costs one more SyncWrite param and one more
    status packet. Its Goal Current cannot ride in the shared block (the
    XC430 has no such register): it is a separate operator-scale write,
    `set_righting_current`.
    """

    def __init__(self, params: dict, port: str = "/dev/ttyUSB0",
                 baud: int = 3_000_000, ids=(1, 2, 3),
                 righting_id: int | None = None,
                 velocity_source: str = "differenced",
                 window_ms: float = 25.0, taper: float = 0.5,
                 control_hz: float = CONTROL_HZ_DEFAULT,
                 gains: dict | None = None,
                 righting_current: int | None = None,
                 servo_sign=(1.0, 1.0),
                 models: dict | None = None):
        self.params = params
        # Servo-shaft angle the controller calls zero: None captures it at
        # open(), a number pins it.
        self.steer_zero = None
        self.id_a, self.id_b, self.id_steer = ids
        self.id_right = int(righting_id) if righting_id is not None else None
        self.ids = tuple(ids) + ((self.id_right,) if self.id_right else ())
        self.control_hz = float(control_hz)
        self.belt_ratio = float(params["drivetrain"]["belt_ratio"])
        # Which way each drive servo turns for a POSITIVE input shaft: the
        # horns face outboard, so the pair is mirrored, measured [1, -1]
        # (drivetrain-measurements.yaml). Applied here and nowhere else:
        # everything upstream is in the input-shaft frame.
        self.servo_sign = (float(servo_sign[0]), float(servo_sign[1]))
        self.steer_ratio = float(params["bike"]["steering"]["gear_ratio"])
        self.port_name, self.baud = port, baud
        if velocity_source not in ("differenced", "reported"):
            raise ValueError("velocity_source must be 'differenced' or 'reported'")
        self.velocity_source = velocity_source
        self._filters = {i: RateFilter(window_ms, taper, 1000.0 / control_hz)
                         for i in self.ids}
        # Per-servo goal register: what makes one SyncWrite enough.
        self.goal_item = {self.id_a: "Goal Velocity",
                          self.id_b: "Goal Velocity",
                          self.id_steer: "Goal Position"}
        if self.id_right:
            self.goal_item[self.id_right] = "Goal Position"
        # Role-keyed on the way in ("drive" = both hubs), id-keyed here.
        self.gains = self._expand_gains(gains)
        self.gains_applied: dict = {}
        # Righting servo's Goal Current, RAW counts (see set_righting_current);
        # None leaves the power-on value, which is Current Limit: no cap.
        self.righting_current = (None if righting_current is None
                                 else int(righting_current))
        self.righting_current_applied: int | None = None
        # The righting goal [rad, servo shaft], set by the operator and held.
        self._righting_goal = None
        self._prev = {}          # id -> (tick, counts)
        # Single-turn position, so unwrapped: the velocity-mode hubs only.
        self._wraps = {self.id_a, self.id_b}
        # Accumulated shaft angle [rad, servo] for the hubs, from the same
        # unwrapped deltas the velocity uses; zero at open().
        self._turned = {}
        self.last_raw: dict = {}          # the latest frame, raw: the recorder's row
        self.rebooted: dict = {}          # {id: error bits} cleared at open()
        self.last_voltage: float | None = None
        self.last_read_s = float("nan")
        self.last_goal: dict = {}
        # Declared models, so the map can be built (and tested) with no bus;
        # open() raises if the hardware answers differently.
        self.models = dict(models) if models else {
            self.id_a: "xc430_w150", self.id_b: "xc430_w150",
            self.id_steer: "xc330_t181"}
        if self.id_right and self.id_right not in self.models:
            self.models[self.id_right] = "xc330_t181"
        self.tables = {i: table_by_name(m) for i, m in self.models.items()}
        # Seeded with the DECLARED tables; discovery replaces them only once
        # every id has answered. So an open() that fails partway still has
        # addresses for close()'s torque-off (found 2026-10-03 with the steer
        # unplugged: the failed open then raised KeyError instead).
        self._dxl = DynamixelBus(port, baud, ids=self.ids)
        self._dxl.tables = dict(self.tables)

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> None:
        """Port and discovery (with the firmware floor), the declared-model
        check, latched-error reboot, indirect map, modes and gains, steer
        zero. Leaves torque OFF: `arm()` is the explicit step."""
        assert_alias_margin(self.params, self.control_hz)
        self._dxl.open()
        self._check_models()
        self.rebooted = self._clear_latched_errors()

        self.torque(False)                 # per servo, confirmed: the map needs it off
        self._dxl.apply_map(self._build_map())
        self.gains_applied = self._configure_servos()
        self._capture_steer_zero()

    def _capture_steer_zero(self) -> None:
        """Take the steer's present angle as zero, unless pinned.

        The steer is commanded as an absolute multi-turn angle and an XC330
        loses its turn count across a power cycle, so without this the first
        command jumps. Applied both ways (read and command). It does not home
        the bike: it assumes the wheel is centred at power-on.
        """
        if self.steer_zero is not None:            # pinned by the caller
            return
        raw = signed(self.read_raw(self.id_steer, "Present Position"), 4)
        self.steer_zero = raw / XC330_COUNTS_PER_RAD
        print(f"steer zero captured at {np.degrees(self.steer_zero):+.1f} deg "
              f"(servo shaft); commands are relative to it")

    def arm(self, ids=None) -> None:
        """Torque on, then re-read the gains: a torque change is where a gain
        gets quietly restored (control_tables/README.md)."""
        self.torque(True, ids)
        bad = self._verify_gains()
        if bad:
            self.torque(False, ids)
            raise RuntimeError("gains changed across torque enable: "
                               + "; ".join(bad))

    def _build_map(self) -> IndirectMap:
        """The bike's indirect layout. No I/O, so tests can check it."""
        imap = IndirectMap({i: self.tables[i] for i in self.ids})
        for name in READ_BLOCK:
            imap.read(name)
        for label in HEALTH_BLOCK:
            imap.read(self._health_spec(label), label=label)
        imap.write({i: self.goal_item[i] for i in self.ids}, label="goal")
        self.read_addr, self.read_len = imap.read_addr, imap.read_len
        self.read_offsets = imap.read_offsets
        # (label, address, size, Register) per health field, per servo,
        # resolved once: read_state decodes through them.
        self._health_regs = {
            i: tuple((lbl, imap.read_addr + imap.read_offsets[lbl],
                      imap.register(i, lbl).size, imap.register(i, lbl))
                     for lbl in HEALTH_BLOCK)
            for i in self.ids}
        self.write_addr, self.write_len = imap.write_addr, imap.write_len
        self._map = imap
        return imap

    def _health_spec(self, label: str):
        """`HEALTH_BLOCK` label -> an IndirectMap spec; only "effort" differs
        by model."""
        if label != "effort":
            return label
        return {i: ("Present Current" if "Present Current" in self.tables[i]
                    else "Present Load") for i in self.ids}

    @property
    def roles(self) -> dict:
        """id -> the name the station and the record use for it."""
        out = {self.id_a: "drive_a", self.id_b: "drive_b",
               self.id_steer: "steer"}
        if self.id_right:
            out[self.id_right] = "righting"
        return out

    def _check_models(self) -> None:
        """Each id answered discovery as its declared model, or raise: the
        declared tables build the map, so swapped ids would read plausible
        nonsense."""
        wrong = []
        for i in self.ids:
            got = self._dxl.tables[i]
            want = self.tables[i]
            if got.name != want.name:
                wrong.append(f"id {i}: declared {want.name}, answers {got.name}")
        if wrong:
            raise RuntimeError("servo model mismatch (ids swapped, or the "
                               "wrong bus): " + "; ".join(wrong))

    def _clear_latched_errors(self) -> dict:
        """Reboot any servo with a latched hardware error -> {id: bits}.

        A latched servo keeps its torque off whatever is written, so without
        this a restart after a trip would arm a bike with a dead drive. Done
        before configuration, so the RAM a reboot wipes is written next anyway.
        """
        bad = self._dxl.hardware_errors()
        if not bad:
            return {}
        self._dxl.reboot(ids=tuple(bad), settle_s=1.0)
        still = self._dxl.hardware_errors(ids=tuple(bad))
        if still:
            raise RuntimeError(
                "hardware error survives a reboot -- check the servo before "
                "running: " + "; ".join(f"id {i}: {describe_hardware_error(b)}"
                                        for i, b in still.items()))
        return bad

    def _expand_gains(self, gains) -> dict:
        """{role: {register: value}} -> {id: {register: value}}. Roles: drive
        (both hubs), drive_a, drive_b, steer, righting; a role with no servo
        (righting on a three-servo bench) is dropped."""
        by_role = {"drive": (self.id_a, self.id_b), "drive_a": (self.id_a,),
                   "drive_b": (self.id_b,), "steer": (self.id_steer,),
                   "righting": (self.id_right,) if self.id_right else ()}
        out: dict = {}
        for role, regs in (gains or {}).items():
            if role not in by_role:
                raise ValueError(f"unknown gain role {role!r}; "
                                 f"expected one of {sorted(by_role)}")
            for i in by_role[role]:
                out.setdefault(i, {}).update({k: int(v) for k, v in regs.items()})
        return out

    def modes(self) -> dict:
        """{id: Operating Mode} the bike wants. Pure; no I/O."""
        want = {self.id_a: MODE_VELOCITY, self.id_b: MODE_VELOCITY,
                self.id_steer: MODE_EXTENDED_POSITION}
        if self.id_right:
            want[self.id_right] = MODE_CURRENT_POSITION
        return want

    def _configure_servos(self) -> dict:
        """Return Delay 0 and the modes (EEPROM), then gains and the righting
        current cap (RAM). Torque must be off. -> what was written, per id.

        The mode is written only if it differs: writing Operating Mode resets
        the position gains (control_tables/README.md), so this keeps startup
        idempotent. The gains and the cap are RAM, back at power-on defaults
        every power cycle, so they are written every time, after the mode,
        and read back.
        """
        report: dict = {}
        for i in self.ids:
            if self.read_raw(i, "Return Delay Time") != 0:
                self.write_raw(i, "Return Delay Time", 0)
        for i, mode in self.modes().items():
            if self.read_raw(i, "Operating Mode") != mode:
                self.write_raw(i, "Operating Mode", mode)
                if self.read_raw(i, "Operating Mode") != mode:
                    raise RuntimeError(f"id {i} refused Operating Mode {mode}")
                report.setdefault(i, {})["mode_written"] = mode
        for i, regs in self.gains.items():
            for name, value in regs.items():
                self.write_raw(i, name, int(value))
            report.setdefault(i, {}).update(
                {name: self.read_raw(i, name) for name in regs})
        bad = self._verify_gains()
        if bad:
            raise RuntimeError("gains did not stick: " + "; ".join(bad))
        if self.id_right and self.righting_current is not None:
            self.righting_current_applied = self.set_righting_current(
                self.righting_current)
            report.setdefault(self.id_right, {})["righting_current"] = \
                self.righting_current_applied
        return report

    def _verify_gains(self) -> list:
        """-> ["id 1 Velocity P Gain: wanted 400, read 100", ...]; [] if clean."""
        bad = []
        for i, regs in self.gains.items():
            for name, value in regs.items():
                got = self.read_raw(i, name)
                if got != int(value):
                    bad.append(f"id {i} {name}: wanted {int(value)}, read {got}")
        return bad

    def read_raw(self, dxl_id: int, name: str) -> int:
        """One register, raw counts. Startup only -- not on the tick path."""
        return int(self._dxl.read_raw(dxl_id, name))

    def write_raw(self, dxl_id: int, name: str, raw: int) -> None:
        """One register, raw counts. Startup only -- not on the tick path."""
        self._dxl.write_raw(dxl_id, name, int(raw))

    def close(self) -> None:
        """Torque off EVERY servo, then close: the one write that must land.

        The port is recovered first (a signal mid-transaction leaves it
        busy), then each servo is written separately, so a failure is seen
        per servo; failures are retried once and reported. Verified on the
        Pi under SIGTERM, SIGHUP and ctrl-C (pi-bench-bringup.md, 2026-09-18).
        """
        if not self._dxl.is_open:
            return
        self._dxl.recover_port()
        failed = []
        for i in self.ids:
            try:
                self.torque(False, (i,))
            except RuntimeError:
                failed.append(i)
        still = []
        for i in failed:
            try:
                self.torque(False, (i,))
            except RuntimeError as e:
                still.append(f"{i}: {e}")
        self._dxl.close()
        if still:
            raise RuntimeError("torque-off FAILED -- these servos may still be "
                               "energised: " + "; ".join(still))

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    @property
    def policy_ids(self) -> tuple:
        """The servos the balance policy drives. Not the righting servo: it is
        operated by hand and stays live while the other three are cut."""
        return (self.id_a, self.id_b, self.id_steer)

    def torque(self, on: bool, ids=None) -> None:
        """Torque Enable, one confirmed write per servo -- not the generic
        SyncWrite, which returns no status, so a servo that missed it would
        go unnoticed."""
        for i in (self.ids if ids is None else tuple(ids)):
            self.write_raw(i, "Torque Enable", int(on))

    def pack_voltage(self) -> float:
        """Bus voltage [V], one round trip: for the preflight, before the loop.
        Once `read_state` runs, `last_voltage` has it for free."""
        return self.read_raw(self.id_a, "Present Input Voltage") * VOLT_LSB

    # -- per-tick I/O ------------------------------------------------------

    def read_state(self) -> dict:
        """One FastSyncRead -> ``{'dt': s, 'servos': {id: {...}}}``.

        Per servo: `pos` [rad, servo shaft], `vel` [rad/s, differenced or
        reported], `vel_reported`, `turned` (hubs: accumulated since open) and
        decoded `health`. `dt` is drive A's Realtime Tick delta: one servo's
        clock, since the clocks are not synchronised.
        """
        t_read = time.perf_counter()
        frame = self._dxl.read_frame(decode=False)
        self.last_read_s = time.perf_counter() - t_read

        out, dt_s = {}, None
        for i in self.ids:
            row = frame[i]
            tick, pos_raw, vel_raw = (row[n] for n in READ_BLOCK)
            counts = signed(pos_raw, 4)
            vel_rep = signed(vel_raw, 4) * VEL_LSB_RAD_S
            health_raw = tuple(row[lbl] for lbl in HEALTH_BLOCK)
            # Raw, in read-label order: what bench_log stores.
            self.last_raw[i] = (tick, pos_raw, vel_raw) + health_raw

            filt = self._filters[i]
            prev = self._prev.get(i)
            if prev is None:                 # first tick: no interval yet
                vel_diff = filt.peek()
            else:
                dms = tick_delta_ms(tick, prev[0])
                if 0 < dms < 500:            # plausible interval
                    # Unwrapped for the hubs (single-turn position); plain for
                    # the steer, whose multi-turn winding is real.
                    d = (pos_delta(counts, prev[1]) if i in self._wraps
                         else counts - prev[1])
                    raw = (d / XC330_COUNTS_PER_RAD) / (dms * 1e-3)
                    vel_diff = filt.update(raw)
                    # The same delta, summed: integrating `vel` would carry
                    # the filter's lag.
                    self._turned[i] = self._turned.get(i, 0.0) + \
                        d / XC330_COUNTS_PER_RAD
                    if i == self.id_a:
                        dt_s = dms * 1e-3
                else:                        # wrap glitch or stall: hold
                    vel_diff = filt.peek()
            self._prev[i] = (tick, counts)

            out[i] = {
                "pos": counts / XC330_COUNTS_PER_RAD,
                "vel": vel_diff if self.velocity_source == "differenced" else vel_rep,
                "vel_reported": vel_rep,
                "turned": self._turned.get(i, 0.0),
                "health": {lbl: reg.decode(raw) for (lbl, _, _, reg), raw
                           in zip(self._health_regs[i], health_raw)},
            }
        self.last_voltage = out[self.id_a]["health"].get("Present Input Voltage")
        return {"dt": dt_s, "servos": out}

    def to_controller_units(self, state: dict) -> dict:
        """Servo units -> the controllers' and estimator's.

        `w_servo_*` stay at the servo (the estimator applies the belt ratio)
        but are signed into the input-shaft frame; `steer_*` are steer-joint
        rad; `turned_*` are input-shaft rad, for display. `health` is per role,
        in servo units and frame.
        """
        sv = state["servos"]
        return {
            "dt": state["dt"],
            "w_servo_a": sv[self.id_a]["vel"] * self.servo_sign[0],
            "w_servo_b": sv[self.id_b]["vel"] * self.servo_sign[1],
            "turned_a": sv[self.id_a]["turned"] * self.belt_ratio * self.servo_sign[0],
            "turned_b": sv[self.id_b]["turned"] * self.belt_ratio * self.servo_sign[1],
            "steer_pos": ((sv[self.id_steer]["pos"] - (self.steer_zero or 0.0))
                          / self.steer_ratio),
            "steer_vel": sv[self.id_steer]["vel"] / self.steer_ratio,
            "w_servo_a_reported": sv[self.id_a]["vel_reported"],
            "w_servo_b_reported": sv[self.id_b]["vel_reported"],
            "steer_vel_reported": sv[self.id_steer]["vel_reported"] / self.steer_ratio,
            "righting_pos": (sv[self.id_right]["pos"] if self.id_right else None),
            "health": {role: sv[i].get("health", {})
                       for i, role in self.roles.items()},
        }

    def set_righting(self, goal_rad: float | None) -> None:
        """Hold a goal for the righting servo [rad, its shaft]. Goes out in the
        next SyncWrite; None (the startup state) leaves it out entirely."""
        self._righting_goal = None if goal_rad is None else float(goal_rad)

    def set_righting_current(self, counts: int) -> int:
        """Goal Current(102) on the righting servo, its torque cap, in RAW
        counts, clamped to Current Limit; returns the value written.

        Raw because the unit is unsettled: the e-manual gives ~1 mA/LSB, the
        vendored model file a torque unit. Station C (R6,
        first-physical-test.md) settles it.
        """
        if not self.id_right:
            raise RuntimeError("no righting servo configured")
        limit = self.read_raw(self.id_right, "Current Limit")
        value = int(max(-limit, min(limit, int(counts))))
        self.write_raw(self.id_right, "Goal Current", value)
        return value

    def write_commands(self, ctrl, aid: dict) -> None:
        """One SyncWrite from the controller's `ctrl` vector.

        Drive entries are input-shaft rad/s (to the servo: / belt ratio, x its
        sign); the steer entry is an absolute steer-joint angle (x steer ratio,
        + steer zero). The righting goal comes from `set_righting`.
        `last_goal` records what went out, in servo units.
        """
        raw, goal = {}, {}
        for (dxl_id, key), sign in zip(((self.id_a, "drive_a"),
                                        (self.id_b, "drive_b")),
                                       self.servo_sign):
            w_servo = float(ctrl[aid[key]]) / self.belt_ratio * sign
            raw[dxl_id] = int(round(w_servo / VEL_LSB_RAD_S))
            goal[dxl_id] = w_servo

        steer_rad = clamp_extended(float(ctrl[aid["steer"]]) * self.steer_ratio
                                   + (self.steer_zero or 0.0))
        raw[self.id_steer] = int(round(steer_rad * XC330_COUNTS_PER_RAD))
        goal[self.id_steer] = steer_rad

        if self.id_right and self._righting_goal is not None:
            right_rad = clamp_extended(self._righting_goal)
            raw[self.id_right] = int(round(right_rad * XC330_COUNTS_PER_RAD))
            goal[self.id_right] = right_rad
        self.last_goal = goal
        self._dxl.write_frame(raw, encode=False)
