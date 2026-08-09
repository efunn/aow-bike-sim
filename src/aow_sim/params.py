"""Loading bike_params.yaml — deliberately free of MuJoCo.

Split out of build_model.py so the onboard code can read the parameter file
without dragging in MuJoCo. `build_model` re-exports both names, so every
existing `from .build_model import load_params` keeps working.

This is the same reason `hw/state.py` exists: the bike runs the controllers,
not the simulator, and the Pi should not need a physics engine installed to
balance. See tests/test_hw_no_mujoco.py, which enforces it.
"""

from __future__ import annotations

import hashlib
import json
import math
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


def derive_righting(p: dict) -> dict:
    """Fill in the righting dimensions that are consequences, not choices.

    The roof and the stowed wings are ONE envelope, not two parts that happen
    to fit: the roof is the circle circumscribing the stowed wing tips. Make
    the roof radius the stow half-span and put its axis at the wing-tip height
    and the tips sit exactly ON the roof surface -- so upside down they are
    tangent to the rolling envelope and can never prop the bike up. Getting
    that wrong is what left it stuck at 154 deg (see part 5 of
    docs/plans/self-righting.md); it is a geometric identity, so it should be
    enforced by construction rather than rediscovered by sweeping.

    Two drivers, both a metre-stick measurement of the finished bike:

        bike_width   wing tip to wing tip, stowed  = the roof DIAMETER
        bike_height  top of the roof, above the rear axle

    from which:

        roof.radius   = bike_width / 2
        roof.height   = bike_height - roof.radius      (axis = the wing tips)
        crank_length  = (bike_width / 2 - pivot_y) / sin(crank_deg)
        wings.length  = roof.height - pivot_z - crank_length * cos(crank_deg)

    Mutates and returns `p`. A missing `righting` block, missing drivers, or a
    pre-set value all leave things alone, so a sweep can still override any
    single dimension after loading.
    """
    rg = p.get("righting")
    if not isinstance(rg, dict):
        return p
    width, height = rg.get("bike_width"), rg.get("bike_height")
    if width is None or height is None:
        return p
    half = width / 2.0

    roof = rg.get("roof")
    if isinstance(roof, dict):
        roof.setdefault("radius", half)
        roof.setdefault("height", height - half)

    w = rg.get("wings")
    if isinstance(w, dict):
        # The crank sets how far outboard the leg sits; the leg then reaches
        # from there up to the roof axis. Both fall out of the envelope.
        sin = math.sin(math.radians(w["crank_deg"]))
        if abs(sin) < 1e-9:
            raise ValueError(
                "righting.wings.crank_deg near 0 cranks the leg straight up, "
                "so bike_width cannot set the crank length; give crank_length "
                "explicitly or crank the wing outboard")
        w.setdefault("crank_length", (half - w["pivot"][1]) / sin)
        w.setdefault(
            "length",
            (height - half) - w["pivot"][2]
            - w["crank_length"] * math.cos(math.radians(w["crank_deg"])))
    return p


def load_params(path: str | Path | None = None) -> dict:
    with open(path or DEFAULT_PARAMS) as f:
        return derive_righting(_normalize(yaml.safe_load(f)))


def params_digest(params: dict) -> str:
    """Stable hash of the parameter set an artifact was designed or trained for.

    Lives HERE, not in export_deploy, for the reason in this module's
    docstring: `hw/state.py` checks a deploy bundle's digest on load, and
    importing it from export_deploy would drag build_model -> MuJoCo onto the
    Pi to do it. Same argument now applies twice over, because trained moves
    carry the digest too and `control/flick.py::load_move` is on the
    numpy-only replay path.
    """
    blob = json.dumps(params, sort_keys=True, default=float).encode()
    return hashlib.sha256(blob).hexdigest()[:16]
