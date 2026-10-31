"""An RL config's `floor:` (params.with_floor): the sim's contact settings for
one training run, recorded in its move so replay rebuilds the floor it
trained on. What invalidates these: params.with_floor, the move yaml's
`floor` key, policy_env_overrides, how build_model applies sim contact.
"""
from types import SimpleNamespace

import pytest
import yaml

from aow_sim.control import general_spec as gs
from aow_sim.params import load_params, with_floor

pytestmark = pytest.mark.spec

CFG = "config/rl/rl_general_v5_floor_softslip.yaml"


def test_overlay_replaces_only_the_named_keys():
    p = load_params()
    mu0 = p["sim"]["friction_sliding"]
    q = with_floor(p, {"friction_sliding": 0.35})
    assert q["sim"]["friction_sliding"] == 0.35
    assert q["sim"]["contact_solref"] == p["sim"]["contact_solref"]
    assert p["sim"]["friction_sliding"] == mu0            # not mutated
    assert with_floor(p, None) is p


def test_a_floors_solref_is_every_contacts_the_wheels_own_included():
    """Since 2026-10-31 the wheels carry their own solref (sim.contact_parts),
    which dictates over the global. A floor's contact_solref must still reach
    them, or the soft-floor exports (v5 s2-s4) replay on the wrong contact."""
    from aow_sim.build_model import CONTACT_PARTS, _part_contact
    p = load_params()
    q = with_floor(p, {"contact_solref": [0.02, 1.0]})
    for part in CONTACT_PARTS:
        assert _part_contact(q["sim"], part)["solref"] == [0.02, 1.0]
    assert p["sim"]["contact_parts"], "load_params' own block was mutated"


def test_unknown_keys_are_refused():
    with pytest.raises(ValueError):
        with_floor(load_params(), {"timestep": 1e-3})


def test_env_builds_and_the_move_records_it():
    from aow_sim.control.general_env import GeneralEnv
    from aow_sim.train_general_rl import move_doc
    with open(CFG) as f:
        cfg = yaml.safe_load(f)
    e = GeneralEnv(None, cfg, seed=0)
    g = e.model.geom("floor").id
    assert e.model.geom_friction[g][0] == pytest.approx(0.35)
    assert list(e.model.geom_solref[g]) == pytest.approx([0.02, 1.0])
    doc = move_doc("general_floor_smoke", load_params(), cfg, {})
    assert doc["floor"] == {"contact_solref": [0.02, 1.0], "friction_sliding": 0.35}
    pol = SimpleNamespace(floor=doc["floor"])
    assert gs.policy_env_overrides(pol)["floor"] == doc["floor"]
    assert gs.policy_env_overrides(SimpleNamespace())["floor"] is None
