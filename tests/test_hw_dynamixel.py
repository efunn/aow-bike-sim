"""Dynamixel bus layer: indirect-address mapping, unit conversions, tick math.

No hardware and no dynamixel_sdk needed — the bus is driven through a fake
packet handler that records every register write, so the indirect setup can be
checked byte by byte. That setup is the part worth testing: it is written once
at startup, it is invisible afterwards, and if it is wrong every subsequent
read and write silently addresses the wrong registers.
"""

import numpy as np
import pytest

from aow_sim.build_model import load_params
from aow_sim.control.steer import XC330_COUNTS_PER_RAD
from aow_sim.hw.control_table import (MODEL_NUMBERS, table_by_name,
                                      table_for)
from aow_sim.hw.dynamixel import (CT, INDIRECT_ADDRESS_1, INDIRECT_DATA_1,
                                  N_INDIRECT, POS_WRAP, READ_BLOCK,
                                  TICK_WRAP, VEL_LSB_RAD_S, IndirectMap,
                                  RateFilter, ServoBus, assert_alias_margin,
                                  _pos_delta, _signed, _tick_delta_ms)

# Register maps, unit conversions and tick math; no bike model.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.pure


@pytest.fixture
def bus():
    """A bus with its indirect map built but nothing on a wire.

    `_build_map` is deliberately pure — the map is now installed in ONE
    SyncWrite, so intercepting individual register writes would test the
    transport instead of the layout. `IndirectMap.address_bytes` IS the
    payload, so asserting on it checks the same bytes more directly.
    """
    b = ServoBus(load_params(), ids=(1, 2, 3))
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
    expected, addr = {}, INDIRECT_DATA_1
    for name in READ_BLOCK:
        src, size = CT[name]
        for k in range(size):
            expected[addr] = src + k
            addr += 1

    for dxl_id in bus.ids:
        got = _indirect_map(bus, dxl_id)
        for a, src in expected.items():
            assert got[a] == src, f"id {dxl_id}: indirect {a} -> {got[a]}, want {src}"

    assert bus.read_addr == INDIRECT_DATA_1
    assert bus.read_len == sum(CT[n][1] for n in READ_BLOCK) == 10


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
        src, _ = CT[item]
        got = _indirect_map(bus, dxl_id)
        for k in range(4):
            a = bus.write_addr + k
            assert got[a] == src + k, (
                f"id {dxl_id}: indirect {a} -> {got[a]}, want {item}+{k}={src+k}")

    # ... and the drives and steer really do differ, or the test above is vacuous
    assert CT["Goal Velocity"][0] != CT["Goal Position"][0]


def test_indirect_block_fits(bus):
    """28 indirect entries exist; the layout must not silently overrun into
    territory that does not exist on every model."""
    used = bus.read_len + bus.write_len
    assert used <= 28, f"indirect block 1 overrun: {used} bytes"


def test_signed_decoding():
    """Position and velocity are two's complement inside unsigned fields."""
    assert _signed(0, 4) == 0
    assert _signed(1000, 4) == 1000
    assert _signed(0xFFFFFFFF, 4) == -1
    assert _signed(0xFFFFFFFF - 999, 4) == -1000
    assert _signed(0x7FFFFFFF, 4) == 2 ** 31 - 1
    assert _signed(0x80000000, 4) == -2 ** 31
    assert _signed(0xFFFF, 2) == -1


def test_velocity_lsb_matches_datasheet():
    """0.229 rev/min per LSB, both models."""
    one_rev_per_s = 2 * np.pi
    assert np.isclose(VEL_LSB_RAD_S, 0.229 * one_rev_per_s / 60.0)
    # 100 raw units -> 22.9 rev/min
    assert np.isclose(100 * VEL_LSB_RAD_S * 60 / one_rev_per_s, 22.9)


def test_tick_delta_handles_the_32768_wrap():
    """Realtime Tick is 0..32767 ms and wraps every ~32.8 s. A naive
    subtraction would hand the estimator a -32.7 s dt once per wrap."""
    assert _tick_delta_ms(1100, 1000) == 100
    assert _tick_delta_ms(5, TICK_WRAP - 5) == 10        # across the wrap
    assert _tick_delta_ms(0, TICK_WRAP - 1) == 1
    assert _tick_delta_ms(1000, 1000) == 0
    for prev in (0, 17, 32000, TICK_WRAP - 1):
        for step in (1, 10, 100):
            assert _tick_delta_ms((prev + step) % TICK_WRAP, prev) == step


def test_position_counts_round_trip():
    """The bus and the simulator's steering frame must share one constant."""
    for rad in (0.0, 0.5, -1.25, 37.0):
        counts = round(rad * XC330_COUNTS_PER_RAD)
        assert abs(counts / XC330_COUNTS_PER_RAD - rad) < 1e-3
    assert np.isclose(XC330_COUNTS_PER_RAD * 2 * np.pi, 4096)


def test_velocity_source_is_validated():
    with pytest.raises(ValueError, match="velocity_source"):
        ServoBus(load_params(), velocity_source="whatever")


# --- RateFilter: the velocity smoothing between raw differencing and the
# servo's own ~50 ms boxcar. Defaults were chosen by sweep (see the class
# docstring); these lock in the properties that made them the right choice.


def test_weights_are_normalized_and_recency_ordered():
    for taper in (0.0, 0.25, 0.5, 1.0):
        f = RateFilter(50.0, taper, nominal_dt_ms=10.0)
        assert np.isclose(f.weights.sum(), 1.0)
        assert np.all(np.diff(f.weights) <= 1e-12), "must not favour older samples"
    assert np.allclose(RateFilter(50.0, 1.0, 10.0).weights, 0.2)   # uniform


def test_window_quantizes_to_whole_ticks():
    """At 100 Hz a 20 ms and a 25 ms request are the same 2-tap filter —
    surfaced via n_taps so callers do not assume otherwise."""
    assert RateFilter(20.0, 0.5, 10.0).n_taps == 2
    assert RateFilter(25.0, 0.5, 10.0).n_taps == 2
    assert RateFilter(50.0, 0.5, 10.0).n_taps == 5
    assert RateFilter(4.0, 0.5, 10.0).n_taps == 1        # never degenerate to 0


def test_group_delay_ordering_and_values():
    """Less taper = less lag; a single difference is half a tick behind."""
    lags = [RateFilter(25.0, t, 10.0).group_delay_ms for t in (0.0, 0.5, 1.0)]
    assert lags == sorted(lags)
    assert np.isclose(RateFilter(10.0, 0.5, 10.0).group_delay_ms, 5.0)
    assert np.isclose(RateFilter(25.0, 0.5, 10.0).group_delay_ms, 25.0 / 3)
    # The default must stay well under the servo's own ~25 ms of lag.
    assert RateFilter().group_delay_ms < 12.0


def test_constant_input_passes_through_unchanged():
    """No taper choice may bias steady-state velocity."""
    for taper in (0.0, 0.5, 1.0):
        f = RateFilter(50.0, taper, 10.0)
        for _ in range(10):
            out = f.update(3.25)
        assert np.isclose(out, 3.25)


def test_uniform_taper_is_a_span_difference():
    """taper=1.0 telescopes: the mean of consecutive differences is exactly
    (newest - oldest) / window. This is why it is the quietest and laggiest."""
    f = RateFilter(50.0, 1.0, 10.0)
    rates = [1.0, 4.0, 2.0, 8.0, 5.0]
    for r in rates:
        out = f.update(r)
    assert np.isclose(out, np.mean(rates))


def test_filter_smooths_quantization_noise():
    """The point of the thing: averaging a noisy rate beats not averaging."""
    rng = np.random.default_rng(0)
    truth = 5.0
    noisy = truth + rng.normal(0, 1.0, 400)
    f = RateFilter(50.0, 0.5, 10.0)
    out = np.array([f.update(v) for v in noisy])[20:]
    assert out.std() < noisy.std() / 1.5


def test_peek_holds_last_estimate_without_new_sample():
    """A dropped or implausible tick must hold, not inject a fake zero."""
    f = RateFilter(25.0, 0.5, 10.0)
    f.update(2.0)
    held = f.update(2.0)
    assert np.isclose(f.peek(), held)
    assert np.isclose(f.peek(), held), "peek must not mutate state"
    assert RateFilter().peek() == 0.0        # empty buffer is well defined


def test_taper_is_validated():
    for bad in (-0.1, 1.5):
        with pytest.raises(ValueError, match="taper"):
            RateFilter(25.0, bad, 10.0)


def test_bus_builds_one_filter_per_servo():
    b = ServoBus(load_params(), ids=(1, 2, 3), window_ms=25.0, taper=0.5,
                 control_hz=100.0)
    assert set(b._filters) == {1, 2, 3}
    assert all(f.n_taps == 2 for f in b._filters.values())
    assert b._filters[1] is not b._filters[2], "servos must not share a buffer"


def test_span_not_tap_count_sets_the_smoothing():
    """Two filters covering the same TIME span are near-equivalent regardless
    of how many samples fall inside it; two with the same tap count at
    different rates are not. This is why oversampling the servos buys nothing.
    """
    span_25ms = [RateFilter(25.0, 1.0, dt).group_delay_ms
                 for dt in (10.0, 5.0, 2.0)]
    assert max(span_25ms) - min(span_25ms) < 4.0, span_25ms

    same_taps = [RateFilter(2 * dt, 1.0, dt).group_delay_ms
                 for dt in (10.0, 5.0, 2.0)]
    assert max(same_taps) / min(same_taps) > 4.0, same_taps


def test_more_taps_at_one_rate_means_more_lag():
    """At a fixed rate, 'more samples' is a longer span and costs lag
    proportionally — it is not free averaging."""
    lags = [RateFilter(n * 10.0, 0.5, 10.0).group_delay_ms for n in (1, 2, 4, 8)]
    assert lags == sorted(lags)
    assert lags[-1] > 3 * lags[0]


# --- single-turn position unwrap (velocity-mode hubs) ----------------------

def test_pos_delta_unwraps_the_single_turn_rollover():
    """The hubs run in Velocity Control Mode, where Present Position is
    0..4095 over ONE rotation, so a turning wheel rolls over about once a
    second at speed. Plain subtraction reads that as a full turn backwards."""
    assert _pos_delta(10, 5) == 5              # ordinary forward
    assert _pos_delta(5, 10) == -5             # ordinary reverse
    assert _pos_delta(0, 4095) == 1            # forward THROUGH the rollover
    assert _pos_delta(4095, 0) == -1           # reverse through it
    assert _pos_delta(100, 4000) == 196
    assert _pos_delta(4000, 100) == -196
    # What the bug looked like: unwrapped, one rollover is a whole revolution
    # of phantom motion on a single sample.
    assert 4095 - 0 == 4095 and _pos_delta(4095, 0) == -1


def test_pos_delta_is_exact_over_a_full_sweep():
    """Integrating the unwrapped deltas must reproduce the true travel, for
    both directions and across many rollovers."""
    for step in (1, 7, 37, -1, -13):
        pos, total = 0, 0
        for _ in range(600):
            nxt = (pos + step) % POS_WRAP
            total += _pos_delta(nxt, pos)
            pos = nxt
        assert total == step * 600, f"step {step}: {total} != {step * 600}"


def test_half_a_turn_is_the_ambiguous_case_and_is_far_from_operation():
    """Exactly POS_WRAP/2 is genuinely undecidable -- +2048 and -2048 are the
    same point -- so the sign there is a convention, not a result. This is the
    limit `assert_alias_margin` keeps the bike 40x away from; the test pins
    that the boundary is handled without raising, not which way it falls."""
    assert abs(_pos_delta(2048, 0)) == POS_WRAP // 2
    assert _pos_delta(2047, 0) == 2047         # just inside: unambiguous
    assert _pos_delta(-2047 % POS_WRAP, 0) == -2047


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
    bus = ServoBus(params, ids=(1, 2, 3))
    assert bus._wraps == {bus.id_a, bus.id_b}
    assert bus.id_steer not in bus._wraps


# --- per-model control tables -------------------------------------------
# Added 2026-09-01 with the bench rigs. The old hand-typed CT could not
# express a register that means different things on the two models, and 126
# is exactly that.

def test_the_shared_subset_really_is_shared():
    """`CT` is derived from the XC430 table and asserted equal to the XC330's.

    If a re-fetch of the vendored tables ever breaks that, the import fails
    loudly rather than the bike addressing the wrong register.
    """
    a, b = table_by_name("xc430_w150"), table_by_name("xc330_t181")
    for name, (addr, size) in CT.items():
        assert (a[name].address, a[name].size) == (addr, size)
        assert (b[name].address, b[name].size) == (addr, size)


def test_address_126_diverges_between_the_models():
    """Same address, same width, different meaning and different units."""
    a, b = table_by_name("xc430_w150"), table_by_name("xc330_t181")
    assert a["Present Load"].address == b["Present Current"].address == 126
    assert a["Present Load"].size == b["Present Current"].size == 2
    assert a["Present Load"].unit_name == "frac_max_torque"
    assert b["Present Current"].unit_name == "A"
    # The XC430 genuinely has no current registers at all.
    assert "Present Current" not in a and "Current Limit" not in a
    assert "Current Limit" in b


def test_model_numbers_match_the_hardware_probe():
    """Read off IDs 101-104 on 2026-09-01, not copied from a datasheet."""
    assert MODEL_NUMBERS == {1070: "xc430_w150", 1210: "xc330_t181"}
    assert table_for(1070).name == "xc430_w150"
    assert table_for(1210).name == "xc330_t181"


def test_unknown_model_number_names_the_fix():
    with pytest.raises(KeyError, match="unknown Model Number"):
        table_for(9999)


def test_signed_registers_decode_negative():
    ct = table_by_name("xc330_t181")
    assert ct.decode("Present Current", 0xFFFF) == pytest.approx(-0.001)
    assert ct["Present Position"].decode(0xFFFFFFFF) < 0


# --- IndirectMap ---------------------------------------------------------

def _tables():
    return {101: table_by_name("xc430_w150"), 103: table_by_name("xc330_t181")}


def test_one_slot_can_be_a_different_register_per_model():
    """The reason the map takes a dict: 126 is Load on one and Current on the
    other, so the slot is byte-identical and the DECODE is not."""
    m = (IndirectMap(_tables()).read("Realtime Tick")
         .read({101: "Present Load", 103: "Present Current"}, label="torque"))
    assert m.register(101, "torque").name == "Present Load"
    assert m.register(103, "torque").name == "Present Current"
    # Same offset, same source address, on both servos.
    assert m.address_bytes(101)[4:] == m.address_bytes(103)[4:] == [126, 0, 127, 0]
    assert m.read_offsets["torque"] == 2


def test_mixed_widths_are_refused():
    """A slot whose width differs across servos would put different fields at
    the same offset, and every later decode would be quietly wrong."""
    with pytest.raises(ValueError, match="differ in width"):
        IndirectMap(_tables()).read({101: "Present PWM",
                                     103: "Present Position"})


def test_a_spec_missing_a_servo_is_refused():
    with pytest.raises(KeyError, match="does not name a register"):
        IndirectMap(_tables()).read({101: "Present Load"})


def test_block_budget_is_enforced_and_names_the_overflow():
    """28 bytes for reads and writes together — the XC330 has no second block,
    so a mixed bus cannot spill into 578/634."""
    m = IndirectMap(_tables())
    for name in ("Present Position", "Present Velocity", "Velocity Trajectory",
                 "Position Trajectory", "Goal Position", "Goal Velocity",
                 "Profile Velocity", "Profile Acceleration"):
        m.read(name)
    assert m.n_bytes > N_INDIRECT
    with pytest.raises(RuntimeError, match="second block"):
        m.apply(None, None)


def test_reads_then_writes_are_each_one_contiguous_span():
    m = (IndirectMap(_tables()).read("Realtime Tick").read("Present Position")
         .write({101: "Goal Velocity", 103: "Goal Position"}, label="goal"))
    assert m.read_addr == INDIRECT_DATA_1
    assert m.read_len == 6 and m.write_len == 4
    assert m.write_addr == INDIRECT_DATA_1 + m.read_len
    assert m.n_bytes == 10


def test_velocity_limit_shares_the_velocity_lsb():
    """It is a velocity register but upstream's [unit info] does not cover it.

    Left as raw counts it decodes to ~460 where the truth is ~11 rad/s, which
    silently defeats any amplitude clamp written against it — analysis/
    servo_reversal.py had exactly that bug before torque was ever enabled.
    """
    for stem in ("xc430_w150", "xc330_t181"):
        ct = table_by_name(stem)
        assert ct["Velocity Limit"].unit == ct["Present Velocity"].unit
        assert ct["Velocity Limit"].unit_name == "rad/s"
