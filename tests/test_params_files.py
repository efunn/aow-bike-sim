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

from aow_sim.params import (ONBOARD_PARAMS, STUDIES_DIR, STUDY_PATHS,
                            load_params, load_raw_params, plant_digest)

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
