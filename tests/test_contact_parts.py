"""The three ground-touching parts can carry their own contact parameters.

WHAT INVALIDATES THIS FILE: `build_model._part_contact`, `CONTACT_PARTS`, the
`sim.contact_parts` override block, or any change to which geoms are treated as
righting surfaces.

The point of the split is that MuJoCo's combination rules cannot express a soft
part against a hard one. `solref`/`solimp` combine by solmix-weighted average
and `friction` by elementwise max, so a compliant TPU roller landing on rigid
wood comes out as the AVERAGE, not as the roller. `geom_priority` is the only
knob that overrides both: the higher-priority geom dictates every contact
parameter and no combination happens.

It ships as a bit-exact no-op -- every part resolves to the same globals, where
dictation and combination agree -- so that turning it on did not move
`plant_digest` and did not make a single export in `moves/` provisional. These
tests pin that, and pin that the override actually bites when someone means it.
"""
import numpy as np
import pytest
import mujoco

from aow_sim.build_model import (CONTACT_PARTS, _part_contact, build_model,
                                 load_params)
from aow_sim.params import plant_digest

pytestmark = pytest.mark.contact


def _ground_contacts(model, settle=500):
    """{geom name touching the floor: (solref, mu)} after settling."""
    data = mujoco.MjData(model)
    for _ in range(settle):
        mujoco.mj_step(model, data)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(model.ngeom)]
    floor = names.index("floor")
    out = {}
    for c in range(data.ncon):
        g1, g2 = data.contact.geom1[c], data.contact.geom2[c]
        if floor not in (g1, g2):
            continue
        other = g2 if g1 == floor else g1
        out[names[other]] = (tuple(np.round(data.contact.solref[c], 9)),
                             round(float(data.contact.friction[c][0]), 9))
    return out


def test_shipped_config_is_a_bit_exact_no_op():
    """No `sim.contact_parts` block -> every part gets the globals verbatim.

    This is the guard that lets the split ship without re-baselining anything.
    If it fails, the physics moved and every policy in `moves/` is provisional.
    """
    p = load_params()
    assert "contact_parts" not in p["sim"], (
        "bike_params.yaml has grown a contact_parts block -- that is a "
        "deliberate physics change and moves plant_digest; update this test "
        "and re-export rather than deleting the assertion")
    sim = p["sim"]
    for part in CONTACT_PARTS:
        kw = _part_contact(sim, part)
        assert kw["solref"] == list(sim["contact_solref"])
        assert kw["friction"][0] == sim["friction_sliding"]
        assert kw["friction"][1] == sim["friction_torsional"]


def test_the_split_did_not_move_the_plant_digest():
    """The whole reason the overrides are absent from the YAML.

    RE-BASELINED, not loosened. This pins a literal, so it goes red whenever
    the plant moves for ANY reason -- which is the point: it is the tripwire
    that asks "did you mean to move the plant?". Every re-baseline belongs in
    the log below with what moved it, so the history reads as a list of
    deliberate plant changes rather than as a number somebody kept bumping.

        e1ec36bfa670217e  the contact-parts split (the claim this test makes)
        eda849e7afaaca0f  2026-09-16  actuators.steer_kv 0.05 -> 0.0676 (the
                          XC330's back-EMF droop, derived, replacing a guess)
                          and righting.{arm,wings}.servo_kp/kv from tuning
                          knobs to the servo's own firmware gains
    """
    assert plant_digest(load_params()) == "eda849e7afaaca0f"


def test_every_part_gets_priority_so_it_dictates_its_contact():
    """Priority 1 against the floor's 0. Without it an override would AVERAGE
    with the floor and a soft part could never win."""
    for part in CONTACT_PARTS:
        assert _part_contact(load_params()["sim"], part)["priority"] == 1


def test_unknown_part_name_raises():
    """A typo must not silently fall back to the globals and look like it
    worked -- that is the failure mode this whole file exists to prevent."""
    with pytest.raises(KeyError):
        _part_contact(load_params()["sim"], "fornt_tire")


def test_ground_contacts_are_identical_with_and_without_the_split():
    """End to end: the contacts MuJoCo actually forms are unchanged.

    Compares the built model against one whose priorities are zeroed, i.e. the
    pre-split combination path, on the same settled pose.
    """
    p = load_params()
    with_split = build_model(p)
    without = build_model(p)
    without.geom_priority[:] = 0

    a = _ground_contacts(with_split)
    b = _ground_contacts(without)
    assert a and b, "nothing settled onto the floor"
    assert a == b, f"the split changed the contacts:\n  split={a}\n  plain={b}"


def test_an_override_actually_bites():
    """A front-tire override must reach the contact and NOT be averaged away.

    The floor stays at the global solref; averaging would give the midpoint, so
    reading back the override exactly is what proves priority is doing the work.
    """
    p = load_params()
    p["sim"]["contact_parts"] = {
        "front_tire": {"solref": [0.002, 1.0], "friction_sliding": 1.3}}
    kw = _part_contact(p["sim"], "front_tire")
    assert kw["solref"] == [0.002, 1.0]
    assert kw["friction"][0] == 1.3
    # torsional was not overridden and must still come from the globals
    assert kw["friction"][1] == p["sim"]["friction_torsional"]

    got = _ground_contacts(build_model(p))
    assert "front_tire" in got, "front tire never touched the floor"
    solref, mu = got["front_tire"]
    assert solref == pytest.approx((0.002, 1.0)), (
        f"front tire contact was {solref}; averaging with the floor would give "
        f"the midpoint, so priority is not being applied")
    assert mu == pytest.approx(1.3)


def test_an_override_on_one_part_leaves_the_others_alone():
    p = load_params()
    p["sim"]["contact_parts"] = {"front_tire": {"solref": [0.002, 1.0]}}
    assert _part_contact(p["sim"], "roller")["solref"] == list(
        p["sim"]["contact_solref"])
    assert _part_contact(p["sim"], "righting")["solref"] == list(
        p["sim"]["contact_solref"])


def _driven_rollout(model, n=3000):
    """A deterministic driven rollout -- same ctrl every call, no randomness.

    Driven rather than passive on purpose: the contact has to be LOADED for a
    contact change to show up, and a bike settling under gravity barely loads
    it. The sinusoids are arbitrary but fixed.
    """
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    out = []
    for k in range(n):
        t = k * model.opt.timestep
        data.ctrl[:] = 0.0
        if model.nu >= 3:
            data.ctrl[0] = 3.0 * np.sin(6.0 * t)
            data.ctrl[1] = 3.0 * np.sin(6.0 * t + 0.4)
            data.ctrl[2] = 0.3 * np.sin(2.0 * t)
        mujoco.mj_step(model, data)
        if k % 500 == 499:
            out.append(data.qpos.copy())
    return np.array(out)


def test_trajectory_is_BITWISE_identical_to_the_pre_split_model():
    """The claim the whole no-op rests on, at the level that matters.

    Contact parameters agreeing is necessary but not sufficient -- what has to
    be unchanged is the TRAJECTORY. Compared against the same model with
    priorities zeroed, which is exactly the pre-split combination path.
    """
    p = load_params()
    split = build_model(p)
    pre = build_model(p)
    pre.geom_priority[:] = 0
    assert set(np.unique(split.geom_priority)) == {0, 1}, "split not applied"

    a, b = _driven_rollout(split), _driven_rollout(pre)
    assert np.array_equal(a, b), (
        f"the split moved the trajectory by {np.abs(a - b).max():.3e} -- it is "
        f"no longer a no-op, and every export in moves/ is provisional")


def test_an_override_actually_changes_the_trajectory():
    """The other half: the split is not merely inert plumbing.

    Without this, a bug that ignored `contact_parts` entirely would pass every
    other test in this file by being a perfect no-op.
    """
    p = load_params()
    base = _driven_rollout(build_model(p))

    q = load_params()
    q["sim"]["contact_parts"] = {
        "front_tire": {"solref": [0.002, 1.0], "friction_sliding": 1.3}}
    changed = _driven_rollout(build_model(q))

    assert not np.array_equal(base, changed), (
        "a front_tire override produced an identical trajectory -- the "
        "override is being dropped somewhere between config and geom")
    assert np.abs(base - changed).max() > 1e-3
