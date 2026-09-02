"""Per-model Dynamixel control tables, loaded from ROBOTIS's own `.model` files.

WHY THIS IS A DOCUMENT AND NOT A DICT IN THE BUS MODULE. `dynamixel.py` used to
carry a nine-entry hand-typed `CT` with a comment claiming the XC430-W150 and
XC330-T181 "have identical control tables for everything used here". That is
true of those nine registers and stops being true the moment a bench test wants
a tenth — address 126 is `Present Load` on one and `Present Current` on the
other, same width, same slot in any indirect map, different meaning and
different units. A hand-typed subset cannot express that; a per-model table can.

The tables in `control_tables/*.model` are vendored VERBATIM from
`ROBOTIS-GIT/DynamixelSDK` so they stay diffable against upstream — see that
directory's README for provenance, checksums, and the two model differences
that were verified against the hardware rather than inferred.

Anything upstream does not carry lives in `_OVERRIDES` below, annotated with a
`source:` in the same spirit as `config/bike_params.yaml`.

USAGE

    from .control_table import table_for, table_by_name

    ct = table_for(1070)              # by Model Number(0), as read off the bus
    ct = table_by_name("xc430_w150")  # or by file name
    ct["Present Position"].address    # 132
    ct["Present Velocity"].unit       # rad/s per LSB
    ct.decode("Present Current", raw) # -> physical units, sign applied

`table_for` is the one to use at runtime: read register 0 from each servo and
let the bus discover what it is talking to, rather than being told.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

TABLE_DIR = Path(__file__).parent / "control_tables"

# Model Number(0) -> table file stem. Read off the hardware, not the datasheet:
# IDs 101/102 report 1070 and 103/104 report 1210 (2026-09-01, fw 50/53).
MODEL_NUMBERS = {
    1070: "xc430_w150",
    1200: "xl330_m288",
    1210: "xc330_t181",
}

# Indirect block 1, present on every X-series part and the only one the XC330
# has. 28 entries: `Indirect Address N` is 2 bytes at 168 + 2*(N-1) and holds
# the address of ONE byte; `Indirect Data N` is that byte, at 224 + (N-1).
#
# Upstream's `Indirect Address Write` / `Indirect Address Read` entries are a
# naming convention rather than a hardware partition -- they are two pointers
# into the same general-purpose pool, and any entry may carry a goal or a
# feedback byte. Allocation is `IndirectMap`'s business, not the table's.
INDIRECT_ADDRESS_1 = 168
INDIRECT_DATA_1 = 224
N_INDIRECT = 28

# Extra unit/sign facts upstream's [unit info] does not carry, per model.
# `None` for a unit means "raw counts, no conversion".
#
#   model stem -> register name -> (per-LSB value, unit name, signed, source)
_COMMON = {
    # 4096 counts/rev, 12-bit absolute encoder. Same constant control/steer.py
    # uses as XC330_COUNTS_PER_RAD. source: datasheet
    "Present Position":      (2 * 3.141592653589793 / 4096, "rad", True),
    "Goal Position":         (2 * 3.141592653589793 / 4096, "rad", True),
    # Present PWM is 0.113 % per LSB against a PWM Limit(36) of 885 = 100 %.
    # Expressed here as DUTY in [-1, 1], which is what the actuator models
    # want. source: datasheet (885 counts full scale)
    "Present PWM":           (1.0 / 885.0, "duty", True),
    "Goal PWM":              (1.0 / 885.0, "duty", True),
    "PWM Limit":             (1.0 / 885.0, "duty", False),
    # Same LSB upstream gives for Present/Goal Velocity, which its [unit info]
    # covers and this register it does not. Without it `Velocity Limit` decodes
    # as raw counts and reads ~460 where the truth is ~11 rad/s -- which is a
    # safety clamp that does not clamp. Asserted equal to the upstream value in
    # tests/test_hw_dynamixel.py rather than trusted here.
    # source: datasheet
    "Velocity Limit":        (0.0239691227, "rad/s", False),
    # source: datasheet
    "Present Input Voltage": (0.1, "V", False),
    "Present Temperature":   (1.0, "degC", False),
    "Realtime Tick":         (1.0, "ms", False),
}
_OVERRIDES = {
    "xc430_w150": {
        **_COMMON,
        # NO current register on this part -- zero mentions of Present Current
        # or Current Limit in its table. 126 is Present Load, a duty-derived
        # INFERENCE of "about 50% of the maximum torque", +-1000 = +-100%.
        # Speed-dependent, because duty at a given torque depends on back-EMF;
        # calibrate against a known hanging load at steady speeds before
        # trusting anything dynamic. source: datasheet + docs/plans/eval-score-rewrite.md
        "Present Load":      (0.001, "frac_max_torque", True),
    },
    # XL330-M288-T. Register map is IDENTICAL to the XC330-T181 -- the vendored
    # files differ only in that upstream declines to give a current unit for
    # this part ("raw", 1.0) where it gives the XC330 a value in N.m. It is the
    # same 1.0 mA/LSB; the N.m figure upstream carries for the XC330 is that
    # multiplied by an assumed torque constant, which is exactly why this file
    # keeps the electrical unit and leaves kt to bike_params.yaml.
    #
    # Not a bike part. Present because BAM (Rhoban/bam) identifies this exact
    # actuator, so it is the one servo where our measurements and theirs can be
    # compared directly. Runs at 5 V, not the bike's 12 -- read Present Input
    # Voltage rather than assuming.
    "xl330_m288": {
        **_COMMON,
        "Present Current":   (0.001, "A", True),
        "Goal Current":      (0.001, "A", True),
        "Current Limit":     (0.001, "A", False),
    },
    "xc330_t181": {
        **_COMMON,
        # MEASURED milliamps, unlike the XC430's inferred percentage.
        # Upstream's [unit info] gives this in N.m, i.e. already multiplied by
        # an assumed torque constant; the raw electrical unit is kept here and
        # kt is left to bike_params.yaml, which annotates its provenance.
        # source: datasheet (1.0 mA/LSB)
        "Present Current":   (0.001, "A", True),
        "Goal Current":      (0.001, "A", True),
        "Current Limit":     (0.001, "A", False),
    },
}

# Registers whose raw field is two's complement inside an unsigned width, over
# and above whatever [unit info] declares. Position is signed because Extended
# Position Control Mode winds negative.
_SIGNED = {"Present Position", "Goal Position", "Homing Offset",
           "Present Velocity", "Goal Velocity", "Present PWM", "Goal PWM",
           "Present Load", "Present Current", "Goal Current",
           "Velocity Trajectory", "Position Trajectory"}


@dataclass(frozen=True)
class Register:
    """One control-table entry: where it is, how wide, and what it means."""

    name: str
    address: int
    size: int
    unit: float | None = None       # physical units per LSB, None = raw counts
    unit_name: str | None = None
    signed: bool = False

    def decode(self, raw: int) -> float:
        """Raw register field -> physical units (or raw counts if no unit)."""
        v = _signed(raw, self.size) if self.signed else raw
        return v * self.unit if self.unit is not None else float(v)

    def encode(self, value: float) -> int:
        """Physical units -> raw register field, as an unsigned width."""
        v = int(round(value / self.unit)) if self.unit is not None else int(round(value))
        return v & ((1 << (8 * self.size)) - 1)


def _signed(v: int, nbytes: int) -> int:
    """Registers are two's complement inside an unsigned field."""
    bits = 8 * nbytes
    return v - (1 << bits) if v >= (1 << (bits - 1)) else v


class ControlTable:
    """One servo model's register map, plus its indirect block geometry."""

    #: Indirect block 1 -- identical on every model here, see the constants.
    indirect_address = INDIRECT_ADDRESS_1
    indirect_data = INDIRECT_DATA_1
    n_indirect = N_INDIRECT

    def __init__(self, name: str, registers: dict, type_info: dict,
                 model_number: int | None = None):
        self.name = name
        self.registers = registers
        self.type_info = type_info
        self.model_number = model_number

    def __getitem__(self, name: str) -> Register:
        try:
            return self.registers[name]
        except KeyError:
            raise KeyError(
                f"{self.name} has no register {name!r}. Nearest: "
                f"{sorted(n for n in self.registers if name.split()[-1] in n)}"
            ) from None

    def __contains__(self, name: str) -> bool:
        return name in self.registers

    def __repr__(self) -> str:
        return (f"<ControlTable {self.name} model={self.model_number} "
                f"{len(self.registers)} registers>")

    def addr(self, name: str) -> int:
        return self[name].address

    def size(self, name: str) -> int:
        return self[name].size

    def decode(self, name: str, raw: int) -> float:
        return self[name].decode(raw)

    def encode(self, name: str, value: float) -> int:
        return self[name].encode(value)


def _parse_model_file(path: Path) -> tuple[dict, dict, dict]:
    """Parse a ROBOTIS `.model` file into (control_table, units, type_info).

    Format is tab-separated with `[section]` headers and one header row per
    section. Duplicate ADDRESSES are expected and harmless -- upstream lists
    168 as both `Indirect Address 1` and `Indirect Address Write` -- because
    the map is keyed by NAME.
    """
    ct, units, tinfo, section, header = {}, {}, {}, None, False
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            section, header = line[1:-1].strip().lower(), True
            continue
        if header:                      # the column-name row of each section
            header = False
            continue
        parts = [p.strip() for p in line.split("\t") if p.strip()]
        if section == "control table" and len(parts) >= 3:
            ct[" ".join(parts[2:])] = (int(parts[0]), int(parts[1]))
        elif section == "unit info" and len(parts) >= 3:
            units[parts[0]] = (float(parts[1]), parts[2],
                               len(parts) > 3 and parts[3] == "signed")
        elif section == "type info" and len(parts) >= 2:
            tinfo[parts[0]] = parts[1]
    if not ct:
        raise ValueError(f"{path} has no [control table] section")
    return ct, units, tinfo


def _build(stem: str) -> ControlTable:
    path = TABLE_DIR / f"{stem}.model"
    if not path.exists():
        raise FileNotFoundError(
            f"no control table {path}. Known: "
            f"{sorted(p.stem for p in TABLE_DIR.glob('*.model'))}")
    raw_ct, upstream_units, tinfo = _parse_model_file(path)
    over = _OVERRIDES.get(stem, {})

    registers = {}
    for name, (address, size) in raw_ct.items():
        unit = unit_name = None
        signed = name in _SIGNED
        if name in over:                        # ours wins over upstream's
            unit, unit_name, signed = over[name]
        elif name in upstream_units:
            unit, unit_name, u_signed = upstream_units[name]
            signed = signed or u_signed
        registers[name] = Register(name, address, size, unit, unit_name, signed)

    missing = sorted(set(over) - set(raw_ct))
    if missing:
        raise ValueError(
            f"{stem}: _OVERRIDES names registers absent from the vendored "
            f"table: {missing}. Either the table was re-fetched and changed, "
            f"or the override is for a different model.")

    model_number = next((n for n, s in MODEL_NUMBERS.items() if s == stem), None)
    return ControlTable(stem, registers, tinfo, model_number)


_CACHE: dict[str, ControlTable] = {}


def table_by_name(stem: str) -> ControlTable:
    """Load a control table by file stem, e.g. ``"xc430_w150"``. Cached."""
    if stem not in _CACHE:
        _CACHE[stem] = _build(stem)
    return _CACHE[stem]


def table_for(model_number: int) -> ControlTable:
    """Load the table for a Model Number(0) as read off the bus.

    Discovery beats configuration here: a servo that has been swapped, or a
    bench rig wired up in a different order, announces what it is.
    """
    try:
        stem = MODEL_NUMBERS[model_number]
    except KeyError:
        raise KeyError(
            f"unknown Model Number {model_number}. Known: "
            f"{ {n: s for n, s in MODEL_NUMBERS.items()} }. Add the model's "
            f".model file from ROBOTIS-GIT/DynamixelSDK to "
            f"{TABLE_DIR.name}/ and register it in MODEL_NUMBERS."
        ) from None
    return table_by_name(stem)


def known_tables() -> list[str]:
    """Every vendored table's file stem."""
    return sorted(p.stem for p in TABLE_DIR.glob("*.model"))
