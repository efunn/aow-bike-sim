"""`control.lqr.q_steer_standstill`: a steer weight for the v = 0 designs.

Invalidated by `linearize._weights`.
"""

import numpy as np
import pytest

from aow_sim.build_model import load_params
from aow_sim.control import linearize as lz


@pytest.mark.pure
def test_standstill_swaps_in_only_the_steer_weights():
    cfg = dict(load_params()["control"]["lqr"], q_steer=5.0, q_steer_standstill=50.0)
    Q, R = lz._weights(cfg)
    Q0, R0 = lz._weights(cfg, standstill=True)
    steer, steer_rate = lz.STATE_NAMES.index("steer"), lz.STATE_NAMES.index("steer_rate")
    assert (Q[steer, steer], Q0[steer, steer]) == (5.0, 50.0)
    assert (Q[steer_rate, steer_rate], Q0[steer_rate, steer_rate]) == pytest.approx((0.5, 5.0))
    others = [i for i in range(len(Q)) if i not in (steer, steer_rate)]
    assert np.array_equal(np.diag(Q)[others], np.diag(Q0)[others])
    assert np.array_equal(R, R0)
    # Unset, standstill is the plain weight.
    plain = {k: v for k, v in cfg.items() if k != "q_steer_standstill"}
    assert np.array_equal(lz._weights(plain, standstill=True)[0], lz._weights(plain)[0])

