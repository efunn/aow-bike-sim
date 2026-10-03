"""A short weighted moving average for rates differenced from an encoder.

Between the servo's own Present Velocity (~a 50 ms boxcar, ~25 ms of lag) and
raw single-step differencing (no lag, but one count of quantisation lands on
one sample). Numpy only, no bus: the simulator's estimator uses it too.
"""

from __future__ import annotations

from collections import deque

import numpy as np


class RateFilter:
    """Weighted moving average over recent difference-derived rates.

    `taper` is the OLDEST sample's weight relative to the newest (1.0):
    1.0 is a boxcar (exactly a span difference: least noise, most lag), 0.0
    a ramp to zero (least lag, most noise). Rates are pushed, not counts, so
    a jittery dt is handled per sample.

    `window_ms` is quantised to whole ticks: read `n_taps` and
    `group_delay_ms` rather than assume it was honoured. The default (25 ms,
    0.5) was swept in sim against ground truth; what sets the smoothing is the
    time span, not the tap count, and a deadband on small differences only
    adds bias (docs/plans/odometry-rewrite.md, "RateFilter: the sweep").
    """

    def __init__(self, window_ms: float = 25.0, taper: float = 0.5,
                 nominal_dt_ms: float = 10.0):
        if not 0.0 <= taper <= 1.0:
            raise ValueError(f"taper must be in [0, 1], got {taper}")
        n = max(1, int(round(window_ms / nominal_dt_ms)))
        w = np.linspace(1.0, taper, n) if n > 1 else np.ones(1)
        self.weights = w / w.sum()          # index 0 = most recent
        self.n_taps = n                     # window_ms rounded to whole ticks
        self.window_ms, self.taper = window_ms, taper
        self.nominal_dt_ms = nominal_dt_ms
        self._buf: deque = deque(maxlen=n)

    def update(self, rate: float) -> float:
        """Push one difference-derived rate; return the filtered estimate."""
        self._buf.appendleft(rate)
        w = self.weights[:len(self._buf)]
        return float(np.dot(w, self._buf) / w.sum())

    def peek(self) -> float:
        """The estimate without a new sample: a skipped tick holds the last
        value rather than injecting a zero."""
        if not self._buf:
            return 0.0
        w = self.weights[:len(self._buf)]
        return float(np.dot(w, self._buf) / w.sum())

    def reset(self) -> None:
        self._buf.clear()

    @property
    def group_delay_ms(self) -> float:
        """Lag behind the truth at DC: half a sample for the difference itself,
        plus the taps' weighted mean age."""
        w = self.weights
        return (0.5 + float(np.dot(w, np.arange(len(w))))) * self.nominal_dt_ms
