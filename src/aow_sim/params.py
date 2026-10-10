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


# libyaml's parser when the install has it: bike_params.yaml parses in 1.7 ms
# against 23.5 ms, and the suite calls this dozens of times. Same safe
# constructor, same resolver -- every config/ and moves/ yaml loaded to an
# equal dict and hash either way (checked 2026-09-29). A build without libyaml
# (the Pi, possibly) falls back to the pure-Python one.
_Loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)


ROOT = DEFAULT_PARAMS.parents[1]


def scaled_linkage(cad_cfg: str | Path) -> dict:
    """The four-bar a righting CAD config builds: its `linkage.config` with
    the links at `linkage.scale` about the wing rod -- the three lengths
    times k, the servo's height pulled toward the rod's by the same k -- as
    `cad_righting.load` and `analysis/swing_stepthrough.py` scale it. Angles,
    the panel and the stroke are left as the file has them."""
    cad = _normalize(yaml.load((ROOT / cad_cfg).read_text(), Loader=_Loader))
    cfg = yaml.load((ROOT / cad["linkage"]["config"]).read_text(), Loader=_Loader)
    k = float(cad["linkage"]["scale"])
    m = cfg["mechanism"]
    for n in ("crank_length", "coupler_length", "rocker_length"):
        m[n] = float(m[n]) * k
    pz = float(m["wing_pivot_z"])
    m["servo_offset"] = pz + k * (float(m["servo_offset"]) - pz)
    cfg["_source"] = {"cad": str(cad_cfg), "config": str(cad["linkage"]["config"]),
                      "scale": k}
    return cfg


def resolve_righting_module(p: dict) -> dict:
    """Inline the righting module's files into `p` (bike_params.yaml
    `righting.module`): `linkage` (the scaled four-bar) and `blade_outline` /
    `blade_y` (the blade's front section). Inlined rather than read at build
    time so that plant_digest, a hash of this dict, moves when the CAD's
    linkage, its scale or the blade does. Mutates and returns `p`; a params
    file without the block is left alone."""
    mod = (p.get("righting") or {}).get("module")
    if not isinstance(mod, dict) or "linkage" in mod:
        return p
    mod["linkage"] = scaled_linkage(mod["linkage_cad"])
    blade = yaml.load((ROOT / mod["blade_file"]).read_text(), Loader=_Loader)
    mod["blade_outline"] = blade_points(blade["outline"])
    mod["blade_y"] = [float(v) for v in blade["y"]]
    return p


BLADE_CURVE_SEGMENTS = 40


def blade_points(outline, segments: int = BLADE_CURVE_SEGMENTS) -> list:
    """A blade file's `outline` as plain [x, y] points: the RIGHT wing seen
    from BEHIND, mm, x out from the midline (to the right), y up from the
    floor. Angles are counter-clockwise in that view. Each entry is one of:

      [x, y]                         a point.
      {dir_deg: a, len: L}           a straight run of L mm from the previous
                                     point, at a deg FROM LEVEL: 0 level
                                     outward, 90 straight up, 180 level
                                     inward. E.g. the outer face, its angle
                                     one number.
      {curve: {to: [x, y],           a CUBIC Bezier from the previous point to
               handles: [h0, h1],    `to`, tangent at both ends to the
               start_deg: 0,         straight sections either side by
               end_deg: 0}}          default; start_deg / end_deg (optional)
                                     turn an end off tangent, relative to its
                                     neighbour, for a deliberate kink.
                                     `handles` [mm]: how far each end runs
                                     along its tangent before turning -- the
                                     tuning knobs.

    The loop closes itself, last point back to the first.

    Curves are cut into `segments` straight pieces here. Two curves may not
    share a point (each one's tangent is relative to a STRAIGHT neighbour).
    An outline of plain points comes back as floats, unchanged."""
    import numpy as np
    nodes = []                              # (position, the curve arriving there)
    for e in outline:
        if isinstance(e, dict) and "curve" in e:
            nodes.append((np.asarray(e["curve"]["to"], float), e["curve"]))
        elif isinstance(e, dict):
            if not nodes:
                raise ValueError("blade outline starts with a run; give a point first")
            a = math.radians(float(e["dir_deg"]))
            nodes.append((nodes[-1][0] + float(e["len"]) * np.array([math.cos(a), math.sin(a)]),
                          None))
        else:
            nodes.append((np.asarray(e, float), None))
    n = len(nodes)

    def unit(v):
        return v / np.linalg.norm(v)

    def turn(v, deg):
        c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        return np.array([c * v[0] - s_ * v[1], s_ * v[0] + c * v[1]])
    out = []
    for i, (p, c) in enumerate(nodes):
        if c is not None:
            if nodes[i - 1][1] is not None or nodes[(i + 1) % n][1] is not None:
                raise ValueError("two blade curves share a point; put a straight section between")
            a, before, after = nodes[i - 1][0], nodes[i - 2][0], nodes[(i + 1) % n][0]
            t0 = turn(unit(a - before), float(c.get("start_deg", 0.0)))
            t1 = turn(unit(after - p), float(c.get("end_deg", 0.0)))
            h0, h1 = (float(v) for v in c["handles"])
            P = [a, a + h0 * t0, p - h1 * t1, p]
            for k in range(1, segments):
                t = k / segments
                w = [(1 - t) ** 3, 3 * (1 - t) ** 2 * t, 3 * (1 - t) * t ** 2, t ** 3]
                out.append([float(sum(wi * q[0] for wi, q in zip(w, P))),
                            float(sum(wi * q[1] for wi, q in zip(w, P)))])
        out.append([float(p[0]), float(p[1])])
    return out


def load_params(path: str | Path | None = None) -> dict:
    with open(path or DEFAULT_PARAMS) as f:
        return resolve_righting_module(
            derive_righting(_normalize(yaml.load(f, Loader=_Loader))))


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


def _hash(obj) -> str:
    return hashlib.sha256(
        json.dumps(obj, sort_keys=True, default=float).encode()).hexdigest()[:16]


def plant_digest(params: dict) -> str:
    """Hash of the BIKE: every top-level key except `control`.

    Answers "was this trained or derived against the machine I am now running?"
    -- the only question a trained policy can be asked, because nothing under
    `train_*.py` or `*_env.py` reads `params["control"]` at all. The env takes
    its control rate from the RL config, not from here.

    Splitting this out of `params_digest` was not tidying. Measured 2026-08-25:
    32 of the 44 `control` leaves are read by NOTHING that carries a digest, so
    editing a PD gain -- or `general_move`, which is a policy name -- invalidated
    the deploy bundle and all 39 exports. Under this hash,
    `general_rl_smooth_diff_pi` is valid again: it differs from the current
    parameters in exactly two leaves, `control.lqr.q_roll_rate` and
    `control.lqr.q_steer`, and it cannot read either. Five exports recover this
    way, and they are the current ones.

    See docs/plans/params-digest-split.md.
    """
    return _hash({k: v for k, v in params.items() if k != "control"})


# The control fields the LQR gain design actually reads -- linearize.py:145
# (rate_hz), :271 and :286 (the lqr weights), :284 (the speed grid). Twelve
# leaves of the forty-four. If linearize starts reading another one, it belongs
# here, and a stale bundle will otherwise go unnoticed.
DESIGN_FIELDS = ("rate_hz", "lqr", "drive.speed_grid")


def design_digest(params: dict) -> str:
    """Hash of the LQR DESIGN INPUTS ONLY -- deliberately not the plant.

    Answers "were these gains designed against the weights I am now running?"
    Independent of `plant_digest` on purpose: checking the two separately is
    what lets a mismatch say WHICH half moved, instead of printing two hex
    strings and leaving the reader to guess. A combined hash cannot do that.

    Nothing an RL policy does depends on this, which is the point -- a stale
    gain schedule must not be able to stop an RL run. See `hw/state.py`.
    """
    c = params.get("control") or {}
    picked = {}
    for f in DESIGN_FIELDS:
        node, key = c, f
        if "." in f:
            head, key = f.split(".", 1)
            node = c.get(head) or {}
        picked[f] = node.get(key)
    return _hash(picked)
