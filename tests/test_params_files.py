"""bike_params.yaml is the bike; the onboard settings and the studies sit beside it.

WHAT INVALIDATES THIS FILE: `params.load_params` / `load_raw_params`,
`params.STUDY_PATHS` or `plant_digest`, or moving a block between
config/bike_params.yaml, config/onboard.yaml and config/studies/.

Since 2026-10-21 bike_params.yaml holds only what trains and renders the bike
in its current form. `load_params` merges config/onboard.yaml in at
control.onboard and every config/studies/*.yaml at its own paths, so the old
`params[...]` paths still resolve; these tests pin that the split is a split
and not a change: the default bike needs no study block, and no study edit
can move `plant_digest`.
"""
from pathlib import Path

import numpy as np
import pytest
import yaml

from aow_sim.params import (ONBOARD_PARAMS, PRINTED_PARTS, STUDIES_DIR,
                            STUDY_PATHS, load_params, load_raw_params,
                            plant_digest)

pytestmark = pytest.mark.geometry


def _paths(tree: dict) -> set:
    """Each study file's paths outside `control`: a top-level key, or one level
    down under `righting` (the only block that holds both the bike's parts
    and the studies')."""
    out = set()
    for k, v in tree.items():
        if k == "control":
            continue
        if k == "righting":
            out |= {(k, sub) for sub in v}
        else:
            out.add((k,))
    return out


@pytest.mark.pure
def test_study_paths_are_exactly_what_the_studies_files_hold():
    held = set()
    for f in STUDIES_DIR.glob("*.yaml"):
        held |= _paths(yaml.safe_load(f.read_text()) or {})
    assert held == set(STUDY_PATHS)


@pytest.mark.pure
def test_bike_params_holds_no_study_block_and_no_onboard():
    raw = yaml.safe_load((STUDIES_DIR.parent / "bike_params.yaml").read_text())
    for path in STUDY_PATHS:
        node = raw
        for k in path[:-1]:
            node = node.get(k, {})
        assert path[-1] not in node, path
    assert "onboard" not in raw["control"]


@pytest.mark.pure
def test_onboard_is_merged_in_verbatim_at_control_onboard():
    onboard = yaml.safe_load(ONBOARD_PARAMS.read_text())
    assert load_raw_params()["control"]["onboard"] == onboard
    assert load_params()["control"]["onboard"]["servo_ids"] == onboard["servo_ids"]


@pytest.mark.pure
def test_no_study_and_no_onboard_setting_moves_the_plant_digest():
    p = load_params()
    base = plant_digest(p)
    assert plant_digest(load_params(studies=False)) == base
    p["righting"]["wings"]["mass"] = 99.0
    p["hockey"]["ball"]["mass"] = 99.0
    p["control"]["onboard"]["servo_ids"] = [1, 2, 3]
    assert plant_digest(p) == base
    p["righting"]["module"]["servo_kp"] = 99.0     # the bike's own: it does
    assert plant_digest(p) != base


def test_the_default_bike_builds_from_bike_params_alone_and_identically():
    from aow_sim.build_model import build_model
    a = build_model(load_params())
    b = build_model(load_params(studies=False))
    assert (a.nq, a.nv, a.nu, a.nbody, a.ngeom) == (b.nq, b.nv, b.nu, b.nbody, b.ngeom)
    for f in ("body_mass", "body_inertia", "body_ipos", "geom_size", "geom_pos",
              "actuator_gainprm", "actuator_biasprm", "jnt_range", "geom_friction"):
        assert np.array_equal(getattr(a, f), getattr(b, f)), f


def test_printed_parts_are_every_estimated_mass_but_the_steel():
    """bike.printed_mass_scale reaches every `source: estimate` mass under
    bike.frame_parts and righting.module -- a new printed part cannot quietly
    escape it -- except the steel (the wing rod, the wings' screws), and the blade, whose file's mass
    (the slicer's) replaces the entry here."""
    raw = load_raw_params(studies=False)
    est = {("bike", "frame_parts", k, "mass")
           for k, v in raw["bike"]["frame_parts"].items()
           if isinstance(v.get("mass"), dict) and v["mass"]["source"] == "estimate"}
    mod = raw["righting"]["module"]
    est |= {("righting", "module", k, "mass") for k, v in mod.items()
            if isinstance(v, dict) and isinstance(v.get("mass"), dict)
            and v["mass"]["source"] == "estimate"}
    est |= {("righting", "module", "fixed", k, "mass")
            for k, v in mod["fixed"].items() if v["mass"]["source"] == "estimate"}
    est.discard(("righting", "module", "fixed", "wing_rod", "mass"))   # steel
    est.discard(("righting", "module", "blade", "mass"))   # V2's; the file's wins
    for k in ("wing_screws", "wing_pin"):                       # steel
        est.discard(("righting", "module", k, "mass"))
    est.discard(("righting", "module", "wing_stub", "mass"))    # the slicer's
    assert est == set(PRINTED_PARTS)


def test_the_printed_scale_multiplies_exactly_those_masses(tmp_path):
    """The scale moves the PRINTED_PARTS masses by itself and nothing else."""
    from aow_sim.params import DEFAULT_PARAMS
    text = DEFAULT_PARAMS.read_text()
    line = next(l for l in text.splitlines()
                if l.strip().startswith("printed_mass_scale:"))
    f = tmp_path / "bike_params.yaml"
    f.write_text(text.replace(line, "  printed_mass_scale: {value: 1.0, source: estimate}"))
    p, one = load_params(), load_params(f)
    k = p["bike"]["printed_mass_scale"]
    assert k != 1.0
    for path in PRINTED_PARTS:
        a, b = p, one
        for key in path:
            a, b = a[key], b[key]
        assert a == pytest.approx(b * k), path
    for path in (("righting", "module", "fixed", "wing_rod", "mass"),
                 ("righting", "rollcage", "mass", "cages"), ("bike", "fork_mass"),
                 ("bike", "frame_parts", "cables", "mass")):
        a, b = p, one
        for key in path:
            a, b = a[key], b[key]
        assert a == b, path
