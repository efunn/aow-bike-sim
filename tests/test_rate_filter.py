"""`hw/rate_filter.RateFilter`: the velocity smoothing between raw
differencing and the servo's own ~50 ms boxcar. These pin the properties that
made the defaults the right choice."""

import numpy as np
import pytest

from aow_sim.hw.rate_filter import RateFilter

pytestmark = pytest.mark.pure


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
