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
# The bike's onboard settings (servo ids, devices, registers), merged in at
# control.onboard; and the studies' blocks (analytic controllers, never-built
# righting mechanisms, test rigs, the ball move), merged in at their own
# paths. Neither is the bike in its current form, which is all
# bike_params.yaml holds since 2026-10-21. Both sit beside the DEFAULT file,
# and are merged into any params file loaded.
ONBOARD_PARAMS = DEFAULT_PARAMS.parent / "onboard.yaml"
STUDIES_DIR = DEFAULT_PARAMS.parent / "studies"
# Every path the studies files hold, outside `control` (already out of the
# plant): plant_digest drops them, so editing a study never reads as a
# different bike. tests/test_params_files.py pins this against the files.
STUDY_PATHS = (("testbed",), ("training_wheels",), ("hockey",),
               ("righting", "bike_width"), ("righting", "bike_height"),
               ("righting", "roof"), ("righting", "bumper"),
               ("righting", "arm"), ("righting", "wings"))


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
    `cad.righting.load` and `analysis/mechanism/swing_stepthrough.py` scale it. Angles,
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


def resolve_righting_module(p: dict, blade_file: str | Path | None = None) -> dict:
    """Inline the righting module's files into `p` (bike_params.yaml
    `righting.module`): `linkage` (the scaled four-bar) and `blade_outline` /
    `blade_y` (the blade's front section). Inlined rather than read at build
    time so that plant_digest, a hash of this dict, moves when the CAD's
    linkage, its scale or the blade does. Mutates and returns `p`; a params
    file without the block is left alone.

    `blade_file` swaps in another blade (teleop's --blade). A blade file may
    also carry the two things that follow the blade rather than the linkage,
    and they then win over bike_params / the linkage config: `mass` (each
    blade) and `crank_travel_deg`, the commanded stroke -- the blade's own
    level point, which differs per blade (V2 129.3, V3 116.9) and which
    teleop, the actuator's range and the ground station all read from here."""
    mod = (p.get("righting") or {}).get("module")
    if not isinstance(mod, dict) or "linkage" in mod:
        return p
    if blade_file is not None:
        # from the caller's directory, or else the repo's (as blade_file is)
        bf = Path(blade_file)
        if not bf.is_absolute():
            bf = Path.cwd() / bf if (Path.cwd() / bf).exists() else ROOT / bf
        bf = bf.resolve()
        # relative to the repo when inside it: the path is hashed into the
        # digest, and an absolute one would differ per machine
        mod["blade_file"] = (bf.relative_to(ROOT).as_posix()
                             if bf.is_relative_to(ROOT) else str(bf))
    mod["linkage"] = scaled_linkage(mod["linkage_cad"])
    blade = yaml.load((ROOT / mod["blade_file"]).read_text(), Loader=_Loader)
    mod["blade_outline"] = blade_points(blade["outline"])
    mod["blade_y"] = [float(v) for v in blade["y"]]
    if "mass" in blade:
        m = blade["mass"]
        mod["blade"]["mass"] = float(m["value"] if isinstance(m, dict) else m)
    if "crank_travel_deg" in blade:
        mod["linkage"]["stroke"]["crank_travel_deg"] = float(blade["crank_travel_deg"])
    return p


BLADE_CURVE_SEGMENTS = 40


def blade_segments(outline, _raw: bool = False) -> list[dict]:
    """A blade file's `outline` as its EXACT pieces, in order round the loop:
    {"line": [p0, p1]} or {"bezier": [P0, P1, P2, P3]} (a cubic's control
    points), each a plain [x, y] in the frame `blade_points` describes. The
    CAD sketches these (a real spline); `blade_points` flattens them."""
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

    def f(q):
        # _raw: numpy arrays, as blade_points evaluates them (bit-identical
        # to its flattening before the split, which the plant digest hashes)
        return q if _raw else [float(q[0]), float(q[1])]
    out = []
    for i, (p, c) in enumerate(nodes):
        a = nodes[i - 1][0]
        if c is not None:
            if nodes[i - 1][1] is not None or nodes[(i + 1) % n][1] is not None:
                raise ValueError("two blade curves share a point; put a straight section between")
            before, after = nodes[i - 2][0], nodes[(i + 1) % n][0]
            t0 = turn(unit(a - before), float(c.get("start_deg", 0.0)))
            t1 = turn(unit(after - p), float(c.get("end_deg", 0.0)))
            h0, h1 = (float(v) for v in c["handles"])
            out.append({"bezier": [f(a), f(a + h0 * t0), f(p - h1 * t1), f(p)]})
        else:
            out.append({"line": [f(a), f(p)]})
    # the loop starts at the first node: its closing line comes last
    return out[1:] + out[:1]


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

    Curves are cut into `segments` straight pieces here (`blade_segments`
    keeps them as Beziers, for the CAD). Two curves may not
    share a point (each one's tangent is relative to a STRAIGHT neighbour).
    An outline of plain points comes back as floats, unchanged."""
    out = []
    for s in blade_segments(outline, _raw=True):
        if "bezier" in s:
            P = s["bezier"]
            for k in range(1, segments):
                t = k / segments
                w = [(1 - t) ** 3, 3 * (1 - t) ** 2 * t, 3 * (1 - t) * t ** 2, t ** 3]
                out.append([float(sum(wi * q[0] for wi, q in zip(w, P))),
                            float(sum(wi * q[1] for wi, q in zip(w, P)))])
            out.append([float(P[3][0]), float(P[3][1])])
        else:
            out.append([float(s["line"][1][0]), float(s["line"][1][1])])
    # the closing line ends on the first point: it goes first, as given
    return out[-1:] + out[:-1]


def _merge(base: dict, extra: dict) -> dict:
    """Deep-merge `extra` into `base` (in place); `extra` wins on a leaf."""
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v
    return base


# The XC330's mode-5 position PID, read as milliamps: A per (count x rad) of
# Position P Gain, derived (4096/2pi)/(256*1000), 2.525e-3 measured on an
# X330; and A s/rad per unit of Position D Gain, taking the D term per 1 ms
# loop tick (a bare-shaft fit gives 0.25-0.7x it). righting_servo.py uses the
# same two.
XC330_K5 = (4096 / (2 * math.pi)) / (256 * 1000)
XC330_KD_PER_UNIT = (1 / 16) * (256 * XC330_K5) * 1e-3


def derive_servo_gains(p: dict) -> dict:
    """The sim's servo gains from the firmware registers the bike writes:
    actuators.steer_kp from actuators.steer_gains, and righting.module
    servo_kp / servo_kv from righting.module.gains. A value already set (a
    params file from before 2026-10-21, a sweep) is left alone. Mutates and
    returns `p`."""
    act = p.get("actuators") or {}
    if "steer_gains" in act and "steer_kp" not in act:
        act["steer_kp"] = (float(act["steer_gains"]["Position P Gain"])
                           * float(act["steer_kp_per_unit"]))
    mod = (p.get("righting") or {}).get("module")
    if isinstance(mod, dict) and "gains" in mod and "servo_kp" not in mod:
        k = float(p["servos"]["xc330_t181"]["current_torque_gain"])
        mod["servo_kp"] = float(mod["gains"]["Position P Gain"]) * XC330_K5 * k
        mod["servo_kv"] = (float(mod["gains"]["Position D Gain"])
                           * XC330_KD_PER_UNIT * k)
    return p


# The printed parts whose mass is a CAD estimate (cad.servo_mount.printed_mass),
# as paths to their `mass`: what bike.printed_mass_scale multiplies. Not the
# rollcage rings or the blade (their files carry the slicer's), the steel wing
# rod, or bike.fork_mass (its split of a weighed 86 g with the front wheel). tests/test_params_files.py pins this
# against every `source: estimate` mass under those two blocks.
PRINTED_PARTS = tuple(
    [("bike", "frame_parts", k, "mass") for k in (
        "drive_case_left", "drive_case_right", "drive_chainstay_left",
        "drive_chainstay_right", "drive_pulley_left", "drive_pulley_right",
        "steer_case_lower", "steer_case_upper", "pi_carrier", "power_holder")]
    + [("righting", "module", k, "mass") for k in ("crank", "coupler", "wing_hub")]
    # upper_case left out since it was weighed (12 g, 11-07): a scale fitted
    # to estimates must not move a measured mass.
    + [("righting", "module", "fixed", k, "mass") for k in (
        "bulkhead", "bridge", "lower_case")])


def wing_hub_mass(mod: dict) -> float:
    """Each wing's body past its blade, the one point mass at wing_hub.at:
    rocker + knuckle, the steel pin, the stub and the wing's screws (each
    absent: 0, as an older params file has only `wing_hub`)."""
    return float(mod["wing_hub"]["mass"]) + sum(
        float(mod.get(k, {}).get("mass", 0.0))
        for k in ("wing_pin", "wing_stub", "wing_screws"))


def scale_printed_masses(p: dict) -> dict:
    """Every PRINTED_PARTS mass times bike.printed_mass_scale (absent: 1).
    A part the bike does not have (a variant's own layout, `!replace`) is
    skipped: its masses are its own."""
    k = float(p["bike"].get("printed_mass_scale", 1.0))
    if k == 1.0:
        return p
    for path in PRINTED_PARTS:
        node = p
        for key in path[:-1]:
            node = node.get(key) if isinstance(node, dict) else None
        if not isinstance(node, dict) or path[-1] not in node:
            continue
        node[path[-1]] = node[path[-1]] * k
    return p


def load_params(path: str | Path | None = None,
                blade: str | Path | None = None,
                studies: bool = True) -> dict:
    """`blade`: another righting blade file in place of the module's own
    (resolve_righting_module) -- a different plant, so a different digest.

    The onboard file is merged in at control.onboard and, unless
    `studies=False`, every config/studies/*.yaml at its own paths. False is
    the bike alone: what the default model must build from."""
    return derive_servo_gains(scale_printed_masses(resolve_righting_module(
        derive_righting(_normalize(load_raw_params(path, studies))), blade)))


def with_wheel_play(params: dict, spec) -> dict:
    """`params` with the `wheel_play:` block (build_model's optional play)
    that `spec` names: ABSENT or None leaves `params` as it is (bike_params'
    own play since 2026-10-31); False strips it; a path is that file's
    `wheel_play:` block (or the whole file), its {value:, source:} wrappers
    stripped so bike_params.yaml itself works; a dict
    is itself, as an export records it. Idempotent once resolved, so the
    trainer and each env can resolve the same config. Relative paths are
    from the repo root (traces/ is not in the GPU pod's tarball: put the
    block inline in a config that goes there)."""
    if spec is None:
        return params
    if spec is False:
        return {k: v for k, v in params.items() if k != "wheel_play"}
    if isinstance(spec, (str, Path)):
        f = Path(spec)
        if not f.is_absolute() and not f.exists():
            f = DEFAULT_PARAMS.parents[1] / f
        doc = yaml.safe_load(f.read_text()) or {}
        spec = _normalize(doc.get("wheel_play", doc))
    if not isinstance(spec, dict):
        raise ValueError(f"wheel_play: a path, a dict, false or absent, not {spec!r}")
    return {**params, "wheel_play": json.loads(json.dumps(spec))}


# What an RL config's `floor:` may set: the sim's contact, which in this model
# is every wheel on the floor (and everything else that touches). No floor is
# measured yet (docs/measurements/floor-measurements.yaml); label a setting by
# its sink at bike weight, never by solref's names (CLAUDE.md, the dampratio
# trap).
FLOOR_KEYS = ("contact_solref", "contact_solimp", "friction_sliding",
              "friction_torsional")


def with_contact_solref(params: dict, solref) -> dict:
    """`params` with `sim.contact_solref` set AND every `sim.contact_parts`
    solref dropped, so the one contact really is every contact. Since
    2026-10-31 the wheels carry their own solref, which dictates over the
    global (build_model._part_contact): setting the global alone would leave
    the wheels where they were. Everything that sets the contact for a study,
    a floor overlay or a teleop flag goes through here."""
    parts = {}
    for name, over in (params["sim"].get("contact_parts") or {}).items():
        rest = {k: v for k, v in over.items() if k != "solref"}
        if rest:
            parts[name] = rest
    return {**params, "sim": {**params["sim"], "contact_solref": list(solref),
                              "contact_parts": parts}}


def with_floor(params: dict, spec) -> dict:
    """`params` with an RL config's `floor:` overlay on `sim`: ABSENT or None
    leaves it, a dict of FLOOR_KEYS replaces those. An export records the
    dict, so replay rebuilds the floor it trained on. A floor's contact_solref
    is every contact's, the wheels' own included (with_contact_solref): what
    the soft-floor exports (v5 s2-s4) trained on, before the wheels had one."""
    if spec is None:
        return params
    if not isinstance(spec, dict) or set(spec) - set(FLOOR_KEYS):
        raise ValueError(f"floor: a dict of {FLOOR_KEYS}, not {spec!r}")
    spec = json.loads(json.dumps(spec))
    if "contact_solref" in spec:
        params = with_contact_solref(params, spec["contact_solref"])
    return {**params, "sim": {**params["sim"], **spec}}


# What a variant file may ADD rather than change: a new servo model's block
# (any key under `servos`), and the key that picks the drive servo, absent
# from bike_params.yaml so its digest stays where it was (drive_servo()).
VARIANT_NEW_PARENTS = (("servos",),)
VARIANT_NEW_KEYS = (("drivetrain", "drive_servo"),)


class _Replace(dict):
    """A variant block tagged `!replace`: it takes the base's block's place
    whole instead of merging into it, so it may add and drop keys. For the
    geometry a new layout owns (bike.frame_parts, bike.payload); everything
    not replaced is still inherited."""


class _VariantLoader(_Loader):
    pass


_VariantLoader.add_constructor(
    "!replace", lambda loader, node: _Replace(loader.construct_mapping(node, deep=True)))


def _plain(node):
    """`node` with every _Replace turned back into a dict."""
    if isinstance(node, dict):
        return {k: _plain(v) for k, v in node.items()}
    if isinstance(node, list):
        return [_plain(v) for v in node]
    return node


def _merge_variant(base: dict, extra: dict, where: str, at: tuple = ()) -> dict:
    """_merge, but a key `base` does not already hold is an error -- a typo in
    a variant would otherwise be a silent no-op -- except VARIANT_NEW_*.
    A `!replace` block replaces the base's whole (which must exist)."""
    for k, v in extra.items():
        path = at + (k,)
        if k not in base and at not in VARIANT_NEW_PARENTS \
                and path not in VARIANT_NEW_KEYS:
            raise KeyError(f"{where}: {'.'.join(path)} is not in its base; "
                           "a variant changes keys, it does not add them")
        if isinstance(v, _Replace):
            base[k] = _plain(v)
        elif isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge_variant(base[k], v, where, path)
        else:
            base[k] = v
    return base


def _read_bike(path: Path) -> dict:
    """One bike file, as written. A file with a top-level `variant_of:` is the
    differences from that file (relative to its own folder), laid over it;
    a base may itself be a variant. config/variants/README.md."""
    p = yaml.load(path.read_text(), Loader=_VariantLoader) or {}
    base = p.pop("variant_of", None)
    if base is None:
        return _plain(p)
    return _plain(_merge_variant(_read_bike((path.parent / base).resolve()), p,
                                 str(path)))


def drive_servo(p: dict) -> dict:
    """The drive servos' `servos:` block: drivetrain.drive_servo names it,
    XC430-W150 (the bike as built) when absent."""
    return p["servos"][p["drivetrain"].get("drive_servo", "xc430_w150")]


def load_raw_params(path: str | Path | None = None, studies: bool = True) -> dict:
    """The merged tree as written -- `{value:, source:}` wrappers kept, nothing
    derived -- for readers that want a parameter's provenance or the onboard
    registers verbatim."""
    p = _read_bike(Path(path or DEFAULT_PARAMS))
    if ONBOARD_PARAMS.exists():
        p.setdefault("control", {})["onboard"] = yaml.load(
            ONBOARD_PARAMS.read_text(), Loader=_Loader)
    if studies:
        for f in sorted(STUDIES_DIR.glob("*.yaml")):
            _merge(p, yaml.load(f.read_text(), Loader=_Loader) or {})
    return p


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
    """Hash of the BIKE: every top-level key except `control`, and less the
    studies' blocks (STUDY_PATHS) since 2026-10-21.

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
    bike = {k: v for k, v in params.items() if k != "control"}
    for path in STUDY_PATHS:              # the studies' blocks: not the bike
        node = bike
        for k in path[:-1]:
            node = node.get(k) if isinstance(node, dict) else None
        if isinstance(node, dict) and path[-1] in node:
            if node is bike:
                bike.pop(path[-1])
            else:                         # copy on the way down, never mutate
                bike[path[0]] = node = {**bike[path[0]]}
                node.pop(path[-1])
    # A 0 g ballast is not fitted (build_model adds no geom): the same bike.
    mod = bike.get("righting", {}).get("module", {})
    if isinstance(mod, dict) and mod.get("ballast", {}).get("mass", None) == 0.0:
        bike["righting"] = {**bike["righting"],
                            "module": {k: v for k, v in mod.items() if k != "ballast"}}
    return _hash(bike)


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
