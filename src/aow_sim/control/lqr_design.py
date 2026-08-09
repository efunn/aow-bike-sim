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
    Ks: np.ndarray           # (n_speeds, 2, 8) scheduled gains
    fit_r2_grid: np.ndarray  # per-speed fit quality
