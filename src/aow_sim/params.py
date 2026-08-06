"""Loading bike_params.yaml — deliberately free of MuJoCo.

Split out of build_model.py so the onboard code can read the parameter file
without dragging in MuJoCo. `build_model` re-exports both names, so every
existing `from .build_model import load_params` keeps working.

This is the same reason `hw/state.py` exists: the bike runs the controllers,
not the simulator, and the Pi should not need a physics engine installed to
balance. See tests/test_hw_no_mujoco.py, which enforces it.
"""

from __future__ import annotations

from pathlib import Path

import yaml

DEFAULT_PARAMS = Path(__file__).resolve().parents[2] / "config" / "bike_params.yaml"


def _normalize(node):
    """Strip {value:, source:} wrappers, leaving plain values."""
    if isinstance(node, dict):
        if "value" in node and "source" in node:
            return node["value"]
        return {k: _normalize(v) for k, v in node.items()}
    return node


def load_params(path: str | Path | None = None) -> dict:
    with open(path or DEFAULT_PARAMS) as f:
        return _normalize(yaml.safe_load(f))
