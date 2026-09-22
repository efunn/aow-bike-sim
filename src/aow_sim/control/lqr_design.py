"""The LQR design ARTIFACT, separated from the machinery that produces it.

Deliberately a module of its own, importing nothing but numpy. Producing an
LQRDesign means numerically linearizing the MuJoCo model at every grid speed
(control/linearize.py, ~2 s of rollouts + scipy); *consuming* one is a matrix
lookup. The bike only ever consumes, so the type it consumes must not drag a
physics engine onto the Pi.

`linearize.py` re-exports LQRDesign, so existing imports keep working.
See tests/test_hw_no_mujoco.py.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# The reduced state, in order -- linearize.py documents each entry. Here, not
# there, because the bike needs the names and must not import linearize
# (MuJoCo). The last two are the drive servos (balance.CrawlSensor).
STATE_NAMES = ("e_lat", "roll", "yaw", "steer",
               "v_lat", "roll_rate", "yaw_rate", "steer_rate",
               "crawl_rate", "crawl_lag")


@dataclass
class LQRDesign:
    """Everything LQRBalance/DriveController need from the linearization.

    `aow_sim.export_deploy` runs `design_all()` on the laptop and ships this;
    see docs/plans/untethered-setup.md.
    """
    K: np.ndarray            # standstill gain (LQRBalance)
    qpos_eq: np.ndarray      # upright equilibrium pose
    fit_r2: np.ndarray       # per-state fit quality at standstill
    speeds: np.ndarray       # gain-schedule breakpoints [m/s]
    Ks: np.ndarray           # (n_speeds, 2, 10) scheduled gains
    fit_r2_grid: np.ndarray  # per-speed fit quality
    # (2, 8) standstill gain on the pre-crawl-state design, for the
    # crawl-balance fallback only (DriveController._K0). None on a bundle
    # exported before 2026-09-22.
    K0_legacy: np.ndarray | None = None
    # The control rate [Hz] the gains were designed at. A discrete design is
    # only the design at its own rate: 200 Hz gains held for 10 ms are a
    # different, untested controller. None on a bundle that did not say.
    rate_hz: float | None = None
