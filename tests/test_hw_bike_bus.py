"""The bike's servo bus (`hw/bike_bus.py`): its indirect layout, the
righting servo, the firmware gains and the servo signs.

No hardware and no dynamixel_sdk: the map is built without a bus and checked
through `IndirectMap.address_bytes`, which IS the setup SyncWrite's payload.
If the layout is wrong, every read and write after startup silently
addresses the wrong registers.
"""

import numpy as np
import pytest

from aow_sim.build_model import load_params
from aow_sim.control.steer import XC330_COUNTS_PER_RAD
from aow_sim.hw.control_table import table_by_name
from aow_sim.hw.dynamixel import (INDIRECT_DATA_1, MODE_CURRENT_POSITION,
                                  MODE_EXTENDED_POSITION, MODE_VELOCITY)
from aow_sim.hw.bike_bus import (HEALTH_BLOCK, READ_BLOCK,
                                  VELOCITY_GAIN_REGISTERS, BikeBus,
                                  assert_alias_margin, resolve_gains)

pytestmark = pytest.mark.pure


@pytest.fixture
def bus():
    """A bus with its indirect map built but nothing on a wire.

    `_build_map` is deliberately pure — the map is now installed in ONE
    SyncWrite, so intercepting individual register writes would test the
    transport instead of the layout. `IndirectMap.address_bytes` IS the
    payload, so asserting on it checks the same bytes more directly.
    """
    b = BikeBus(load_params(), ids=(1, 2, 3))
    b._build_map()
    return b


def _indirect_map(bus, dxl_id):
    """-> {indirect data address: source register address} for one servo.

    Decoded straight out of the SyncWrite payload: entry k is a little-endian
    source address at 168 + 2k, and surfaces as one byte at 224 + k.
    """
    payload = bus._map.address_bytes(dxl_id)
    return {INDIRECT_DATA_1 + k: payload[2 * k] | (payload[2 * k + 1] << 8)
            for k in range(len(payload) // 2)}


def test_read_block_maps_the_intended_registers_contiguously(bus):
    """Realtime Tick / Present Position / Present Velocity are scattered in
    the real control table; indirection must make them one contiguous run."""
    for dxl_id in bus.ids:
        expected, addr = {}, INDIRECT_DATA_1
        for name in READ_BLOCK:
            reg = bus.tables[dxl_id][name]
            for k in range(reg.size):
                expected[addr] = reg.address + k
                addr += 1
        got = _indirect_map(bus, dxl_id)
        for a, src in expected.items():
            assert got[a] == src, f"id {dxl_id}: indirect {a} -> {got[a]}, want {src}"

    assert bus.read_addr == INDIRECT_DATA_1
    # The control fields first and unmoved; the health block (8 bytes) after.
    assert sum(bus.tables[bus.id_a][n].size for n in READ_BLOCK) == 10
    assert bus.read_len == 10 + 8


def test_the_effort_slot_is_current_on_one_model_and_load_on_the_other():
    """Address 126 is Present Current [A] on the XC330 and Present Load
    [fraction of max torque] on the XC430, which has no current sensor. One
    indirect slot carries both -- same address, same width -- and each id
    decodes by ITS OWN register, so a drive's 0.3 is 30 % load and the
    steer's 0.3 is 300 mA. Swap them and the numbers stay plausible."""
    b = BikeBus(load_params(), ids=(1, 2, 3), righting_id=4)
    b._build_map()
    got = {i: b._map.register(i, "effort") for i in b.ids}
    assert {r.address for r in got.values()} == {126}
    assert got[1].name == got[2].name == "Present Load"
    assert got[3].name == got[4].name == "Present Current"
    assert got[1].unit_name != got[3].unit_name
    # and every servo's slot maps to 126, not just the first model's
    off = b.read_offsets["effort"]
    for i in b.ids:
        m = _indirect_map(b, i)
        assert (m[INDIRECT_DATA_1 + off], m[INDIRECT_DATA_1 + off + 1]) == (126, 127)
    assert b._map.read_labels == READ_BLOCK + HEALTH_BLOCK


def test_read_offsets_locate_each_field(bus):
    off = bus.read_offsets
    assert off["Realtime Tick"] == 0
    assert off["Present Position"] == 2
    assert off["Present Velocity"] == 6


def test_one_syncwrite_hits_different_registers_per_servo(bus):
    """The reason indirection is used at all.

    Drives must receive Goal Velocity and the steer Goal Position, from the
    SAME 4-byte SyncWrite at the SAME indirect address. Written directly this
    would need a BulkWrite.
    """
    assert bus.write_len == 4
    assert bus.write_addr == INDIRECT_DATA_1 + bus.read_len

    for dxl_id, item in ((bus.id_a, "Goal Velocity"),
                         (bus.id_b, "Goal Velocity"),
                         (bus.id_steer, "Goal Position")):
        src = bus.tables[dxl_id][item].address
        got = _indirect_map(bus, dxl_id)
        for k in range(4):
            a = bus.write_addr + k
            assert got[a] == src + k, (
                f"id {dxl_id}: indirect {a} -> {got[a]}, want {item}+{k}={src+k}")

    # ... and the drives and steer really do differ, or the test above is vacuous
    t = bus.tables[bus.id_a]
    assert t["Goal Velocity"].address != t["Goal Position"].address


def test_indirect_block_fits(bus):
    """28 indirect entries exist; the layout must not silently overrun into
    territory that does not exist on every model."""
    used = bus.read_len + bus.write_len
    assert used <= 28, f"indirect block 1 overrun: {used} bytes"


def test_position_counts_round_trip():
    """The bus and the simulator's steering frame must share one constant."""
    for rad in (0.0, 0.5, -1.25, 37.0):
        counts = round(rad * XC330_COUNTS_PER_RAD)
        assert abs(counts / XC330_COUNTS_PER_RAD - rad) < 1e-3
    assert np.isclose(XC330_COUNTS_PER_RAD * 2 * np.pi, 4096)


def test_velocity_source_is_validated():
    with pytest.raises(ValueError, match="velocity_source"):
        BikeBus(load_params(), velocity_source="whatever")


def test_bus_builds_one_filter_per_servo():
    b = BikeBus(load_params(), ids=(1, 2, 3), window_ms=25.0, taper=0.5,
                 control_hz=100.0)
    assert set(b._filters) == {1, 2, 3}
    assert all(f.n_taps == 2 for f in b._filters.values())
    assert b._filters[1] is not b._filters[2], "servos must not share a buffer"


def test_alias_margin_passes_at_the_rates_we_run_and_fails_when_too_slow():
    """The margin is bought by belt_ratio and the sample rate, and vanishes
    silently if either moves -- the symptom is a wrong-SIGN velocity at speed,
    not an exception. So it is asserted rather than assumed."""
    params = load_params()
    assert assert_alias_margin(params, 100.0) > 40.0
    assert assert_alias_margin(params, 50.0) > 20.0
    # 2 Hz puts more than half a turn between samples: unrecoverable.
    with pytest.raises(RuntimeError, match="aliasing margin"):
        assert_alias_margin(params, 2.0)


def test_only_the_velocity_mode_servos_are_unwrapped():
    """The steer is Extended Position over +-256 turns and control/steer.py
    winds it pi per flick ON PURPOSE. Unwrapping it would discard turns."""
    params = load_params()
    bus = BikeBus(params, ids=(1, 2, 3))
    assert bus._wraps == {bus.id_a, bus.id_b}
    assert bus.id_steer not in bus._wraps


def test_the_righting_servo_joins_the_same_write_slot():
    """Current-based position mode takes its goal in Goal Position(116), the
    same register the steer uses — which is the whole reason a fourth servo is
    free: it maps to the SAME indirect write address and needs no BulkWrite."""
    b = BikeBus(load_params(), ids=(1, 2, 3), righting_id=4)
    b._build_map()
    assert b.ids == (1, 2, 3, 4)
    assert b.goal_item[4] == "Goal Position"
    assert b.write_addr == BikeBus(load_params(), ids=(1, 2, 3))._build_map().write_addr
    assert b.write_len == 4, "one 4-byte goal per servo, whatever the goal means"


def test_the_righting_servo_is_not_unwrapped():
    """It runs current-based position, where the winding is meaningful — the
    same argument as the steer, and the opposite of the two hubs."""
    b = BikeBus(load_params(), ids=(1, 2, 3), righting_id=4)
    assert b._wraps == {b.id_a, b.id_b}


def test_the_righting_servo_gets_current_based_position_mode():
    b = BikeBus(load_params(), ids=(1, 2, 3), righting_id=4)
    assert b.modes() == {1: MODE_VELOCITY, 2: MODE_VELOCITY,
                         3: MODE_EXTENDED_POSITION, 4: MODE_CURRENT_POSITION}
    assert 4 not in BikeBus(load_params(), ids=(1, 2, 3)).modes()


def test_the_righting_current_is_carried_but_not_written_without_a_servo():
    """The startup torque cap. Pure -- no bus, so this pins the PLUMBING:
    that the value survives construction and that a three-servo bench carries
    it harmlessly rather than raising."""
    b = BikeBus(load_params(), ids=(1, 2, 3), righting_id=4,
                 righting_current=300)
    assert b.righting_current == 300
    assert b.righting_current_applied is None      # nothing written until open()
    three = BikeBus(load_params(), ids=(1, 2, 3), righting_current=300)
    assert three.id_right is None
    assert three.righting_current == 300           # carried, never applied


def test_no_righting_current_means_no_cap_is_written():
    """None must not become 0. A Goal Current of 0 is a servo that produces no
    torque at all, which would look exactly like a dead righting mechanism."""
    b = BikeBus(load_params(), ids=(1, 2, 3), righting_id=4)
    assert b.righting_current is None


def test_the_configured_ids_match_the_configured_righting_servo():
    """101-103 plus 104, not 1-3 plus 4. A Dynamixel ships as ID 1, so ID 1 on
    a bus means an unconfigured servo; and `righting_id` sat at 4 against a
    bench numbered 101-104 until 2026-09-16, which no test would have caught
    because nothing read the two together."""
    cfg = load_params()["control"]["onboard"]
    ids = tuple(cfg["servo_ids"])
    assert len(ids) == 3 and 1 not in ids
    assert cfg["righting_id"] not in ids
    assert cfg["righting_id"] == max(ids) + 1, (
        "the righting servo is the fourth on the same chain; if it is not "
        "adjacent to the other three, say why here")


def test_gain_roles_expand_to_ids():
    b = BikeBus(load_params(), ids=(1, 2, 3), righting_id=4,
                 gains={"drive": {"Velocity P Gain": 400},
                        "steer": {"Position P Gain": 800}})
    assert b.gains == {1: {"Velocity P Gain": 400}, 2: {"Velocity P Gain": 400},
                       3: {"Position P Gain": 800}}


def test_a_righting_gain_is_dropped_on_a_three_servo_bench():
    """One config has to serve both benches, so a role with no servo behind it
    is not an error."""
    b = BikeBus(load_params(), ids=(1, 2, 3),
                 gains={"righting": {"Position P Gain": 700}})
    assert b.gains == {}
    with pytest.raises(ValueError, match="unknown gain role"):
        BikeBus(load_params(), ids=(1, 2, 3), gains={"wheels": {}})


def test_the_gain_keys_agree_with_the_drivetrain_overlay():
    """`hw/` cannot import drivetrain_model (it imports mujoco at module
    scope), so the key names are duplicated. Duplicated AND CHECKED — the same
    device as the shared-slot check below."""
    from aow_sim.drivetrain_model import GAIN_KEYS
    assert tuple(VELOCITY_GAIN_REGISTERS) == GAIN_KEYS


def test_the_policys_own_training_gain_is_what_gets_written():
    """A policy is specific to the firmware gain it trained at — the P 100
    seeds score 0.741–0.751 on their own gain and 0.180–0.503 on P 400 — so
    the record's gain beats any configured default."""
    gains, note = resolve_gains(load_params(), "general_rl_drivetrain_p400_1")
    assert gains["drive"] == {"Velocity P Gain": 400, "Velocity I Gain": 1920}
    assert "general_rl_drivetrain_p400_1" in note


def test_a_pinned_gain_beats_the_policys_own():
    gains, note = resolve_gains(load_params(), "general_rl_drivetrain_p400_1",
                                override=(100, 1920))
    assert gains["drive"]["Velocity P Gain"] == 100
    assert "pinned" in note


def test_an_ideal_plant_policy_has_no_opinion_and_says_so():
    """`general_rl_cmd_curriculum2b` trained without the drivetrain overlay, so
    there is no firmware loop in its plant and no gain to inherit. The startup
    must say that rather than inventing one."""
    gains, note = resolve_gains(load_params(), "general_rl_cmd_curriculum2b")
    assert "drive" not in gains
    assert "NOTHING" in note


def test_a_missing_move_file_is_not_a_crash():
    gains, note = resolve_gains(load_params(), "no_such_policy_at_all")
    assert "drive" not in gains and "NOTHING" in note


def test_turned_is_surfaced_at_the_input_shaft_not_the_servo():
    """The accumulated angle has to arrive in the frame the MODEL's joints are
    in. `w_servo_*` deliberately stay in servo units because VelocityEstimator
    applies the belt ratio itself; `turned_*` do not have that excuse -- no
    controller reads them, so they are converted here or nowhere."""
    b = BikeBus(load_params(), ids=(1, 2, 3))
    state = {"dt": 0.01, "servos": {
        1: {"pos": 0.0, "vel": 1.0, "vel_reported": 1.0, "turned": 2.0},
        2: {"pos": 0.0, "vel": -1.0, "vel_reported": -1.0, "turned": -3.0},
        3: {"pos": 0.0, "vel": 0.0, "vel_reported": 0.0, "turned": 0.0}}}
    got = b.to_controller_units(state)
    assert got["turned_a"] == pytest.approx(2.0 * b.belt_ratio)
    assert got["turned_b"] == pytest.approx(-3.0 * b.belt_ratio)
    # and the rates are NOT converted, which is the asymmetry worth pinning
    assert got["w_servo_a"] == pytest.approx(1.0)


def test_the_accumulator_starts_empty_and_is_per_servo():
    b = BikeBus(load_params(), ids=(1, 2, 3), righting_id=4)
    assert b._turned == {}


def test_the_servo_sign_is_applied_on_read_and_on_write():
    """MEASURED [1, -1] (docs/measurements/drivetrain-measurements.yaml,
    `signs.servo_sign_turning_hub_forward`): both horns face outboard, so B is
    mirrored. It has to land on BOTH paths or the two disagree -- and it must
    land in exactly one place per path, because everything upstream (the
    estimator's hub mix, the ctrl vector, the model's joints) is written in
    the input-shaft frame and must not know servos exist.
    """
    b = BikeBus(load_params(), ids=(1, 2, 3), servo_sign=(1, -1))
    state = {"dt": 0.01, "servos": {
        1: {"pos": 0.0, "vel": 2.0, "vel_reported": 2.0, "turned": 1.0},
        2: {"pos": 0.0, "vel": -2.0, "vel_reported": -2.0, "turned": -1.0},
        3: {"pos": 0.0, "vel": 0.0, "vel_reported": 0.0, "turned": 0.0}}}
    got = b.to_controller_units(state)
    # a real forward roll: servos equal and OPPOSITE, input shafts TOGETHER
    assert got["w_servo_a"] == pytest.approx(2.0)
    assert got["w_servo_b"] == pytest.approx(2.0)
    assert got["turned_a"] == pytest.approx(1.0 * b.belt_ratio)
    assert got["turned_b"] == pytest.approx(1.0 * b.belt_ratio)


def test_the_default_sign_is_inert():
    """A three-servo bench with no config keeps the old behaviour rather than
    silently adopting this bike's mirroring."""
    b = BikeBus(load_params(), ids=(1, 2, 3))
    assert b.servo_sign == (1.0, 1.0)


def test_the_configured_sign_is_the_measured_one():
    cfg = load_params()["control"]["onboard"]
    assert list(cfg["servo_sign"]) == [1, -1], (
        "if the build changes, re-measure with `analysis/drivetrain_bench.py "
        "jog` and update BOTH this and drivetrain-measurements.yaml")


def test_a_commanded_common_mode_reaches_the_servos_as_opposite_goals():
    """The failure the sign prevents, stated as behaviour: commanding both
    input shafts forward must turn the two MIRRORED servos opposite ways. With
    the sign missing the bike crabs when told to drive."""
    b = BikeBus(load_params(), ids=(1, 2, 3), servo_sign=(1, -1))
    sent = {}

    class FakeWriter:
        def clearParam(self): sent.clear()
        def addParam(self, i, data):
            sent[i] = int.from_bytes(bytes(data), "little", signed=False)
            return True
        def txPacket(self): return 0

    b._dxl._map, b._dxl._writer = b._build_map(), FakeWriter()   # the real packing
    b.steer_zero = 0.0
    aid = {"drive_a": 0, "drive_b": 1, "steer": 2}
    b.write_commands([6.0, 6.0, 0.0], aid)          # both input shafts forward

    def assigned(v):
        return v - (1 << 32) if v >= (1 << 31) else v
    a, bb = assigned(sent[1]), assigned(sent[2])
    assert a > 0 and bb < 0, f"servo goals {a}, {bb} -- must be opposite"
    assert a == pytest.approx(-bb, rel=1e-9)


def test_the_shared_slots_mean_the_same_register_on_both_models():
    """Every register BikeBus puts in a slot shared by all servos (the
    READ_BLOCK, Torque Enable, Present Input Voltage) has one address and width
    on both models, or one slot would read different fields."""
    a, b = table_by_name("xc430_w150"), table_by_name("xc330_t181")
    for name in READ_BLOCK + ("Torque Enable", "Present Input Voltage",
                              "Goal Velocity", "Goal Position"):
        assert (a[name].address, a[name].size) == (b[name].address, b[name].size), name
