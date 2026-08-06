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
from aow_sim.hw.dynamixel import (CT, INDIRECT_ADDRESS_1, INDIRECT_DATA_1,
                                  READ_BLOCK, TICK_WRAP, VEL_LSB_RAD_S,
                                  RateFilter, ServoBus, _signed, _tick_delta_ms)


class FakePacket:
    """Records register writes instead of putting them on a wire."""

    def __init__(self):
        self.writes = []          # (dxl_id, address, width, value)

    def _w(self, width):
        def fn(port, dxl_id, address, value):
            self.writes.append((dxl_id, address, width, value))
            return 0, 0
        return fn

    def __getattr__(self, name):
        for width in (1, 2, 4):
            if name == f"write{width}ByteTxRx":
                return self._w(width)
        raise AttributeError(name)


@pytest.fixture
def bus():
    b = ServoBus(load_params(), ids=(1, 2, 3))
    b._packet = FakePacket()
    b._port = object()
    b._setup_indirect()
    return b


def _indirect_map(bus, dxl_id):
    """-> {indirect data address: source register address} for one servo."""
    out = {}
    for i, addr, width, value in bus._packet.writes:
        if i != dxl_id or not (INDIRECT_ADDRESS_1 <= addr < INDIRECT_DATA_1):
            continue
        slot = (addr - INDIRECT_ADDRESS_1) // 2       # 0-based indirect entry
        out[INDIRECT_DATA_1 + slot] = value
    return out


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
