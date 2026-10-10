"""The bike's righting module, V2, as the simulator builds it by default.

V2 is the diamond four-bar `cad_righting` draws (config/righting_cad.yaml:
swing_linkage_shared_rod.yaml at linkage.scale) with the blades of
config/righting_blade.yaml. These pin the sim to the CAD: the same links, the
blade where the file draws it, the toes stopping the stroke past the
commanded travel and short of the knuckles, and the mass the module adds.
Invalidated by a change to the module's files, `righting.module` in
bike_params.yaml, or build_model's `_add_swing_linkage`.

BLADE-AGNOSTIC (user, 2026-10-10): whatever blade the module carries, these
read its numbers from the resolved params, not V2's -- so a new blade passes
unless it actually breaks something. V2's own numbers are pinned only by the
tests that name V2's file or V3's.
"""

from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from aow_sim.build_model import SWING_LINKAGE_CFG, build_model, load_params
from aow_sim.params import ROOT, plant_digest, resolve_righting_module

pytestmark = pytest.mark.righting


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return build_model(params)


V3 = "config/swing_explore/blade_v3.yaml"


def _blade_geoms(model, tag):
    """One wing's blade pieces: the blade, `_toe`, and `_p{n}` for any more
    the convex split of its outline makes."""
    out = []
    for g in range(model.ngeom):
        n = model.geom(g).name
        rest = n[len(f"swing_wing_{tag}"):] if n.startswith(f"swing_wing_{tag}") else None
        if rest is not None and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and \
                (rest in ("", "_toe") or (rest.startswith("_p") and rest[2:].isdigit())):
            out.append(g)
    return out


def _stroke(params):
    return params["righting"]["module"]["linkage"]["stroke"]["crank_travel_deg"]


def test_the_default_bike_carries_v2(params, model):
    names = {model.geom(g).name for g in range(model.ngeom)}
    assert {"swing_wing_right_toe", "swing_wing_left_toe",
            "righting_fixed", "roof"} <= names
    assert not {"bumper_left", "bumper_right"} & names   # user: no bumpers
    lo, hi = np.degrees(model.actuator_ctrlrange[model.actuator("swing").id])
    # the commanded stroke: the linkage's, or the blade file's own level point
    assert (lo, hi) == pytest.approx((-_stroke(params), _stroke(params)))


def test_the_links_are_the_cads(params):
    """The sim's four-bar is cad_righting's: shared_rod x linkage.scale."""
    cad = yaml.safe_load((ROOT / "config/righting_cad.yaml").read_text())
    k = cad["linkage"]["scale"]["value"]
    raw = yaml.safe_load((ROOT / cad["linkage"]["config"]["value"]).read_text())
    m = params["righting"]["module"]["linkage"]["mechanism"]
    for n in ("crank_length", "coupler_length", "rocker_length"):
        assert m[n] == pytest.approx(raw["mechanism"][n] * k)
    from aow_sim import cad_righting as cr
    lk = cr.load()["lk"]
    assert (lk.crank, lk.coupler, lk.rocker) == pytest.approx(
        (m["crank_length"], m["coupler_length"], m["rocker_length"]))


def test_the_blade_sits_where_the_file_draws_it(params, model):
    """At qpos0 the bike stands on the floor exactly (chassis at the rear
    radius), crank 0 is stow, and every blade vertex is an outline point."""
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    ol = params["righting"]["module"]["blade_outline"]
    want = {(round(o, 1), round(u, 1)) for o, u in ol}
    for tag, side in (("right", -1), ("left", 1)):
        got = set()
        for g in _blade_geoms(model, tag):
            mid = model.geom_dataid[g]
            V = model.mesh_vert[model.mesh_vertadr[mid]:
                                model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
            W = V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            got |= {(round(side * y * 1000, 1), round(z * 1000, 1))
                    for y, z in W[:, 1:]}
        assert got == want, tag


def _drive_crank(model, target_deg, seconds=4.0, rate_dps=40.0):
    """Ramp the crank (bare actuator, range widened) with the chassis held in
    the air; return the crank angle where the blades first touch and where it
    ends."""
    m = model
    aid = m.actuator("swing").id
    m.actuator_ctrlrange[aid] = np.deg2rad([-170, 170])
    qa = m.joint("swing_crank_joint").qposadr[0]
    blades = set(_blade_geoms(m, "right") + _blade_geoms(m, "left"))
    d = mujoco.MjData(m)
    q0 = d.qpos.copy()
    q0[2] += 0.15
    d.qpos[:] = q0
    first = None
    for _ in range(int(seconds / m.opt.timestep)):
        d.ctrl[aid] = np.sign(target_deg) * min(np.deg2rad(rate_dps) * d.time,
                                                np.deg2rad(abs(target_deg)))
        mujoco.mj_step(m, d)
        d.qpos[:7] = q0[:7]
        d.qvel[:6] = 0
        if first is None and any(c.geom1 in blades and c.geom2 in blades
                                 for c in d.contact[:d.ncon]):
            first = float(np.degrees(d.qpos[qa]))
    return first, float(np.degrees(d.qpos[qa]))


@pytest.mark.parametrize("sign", (1, -1))
def test_the_toes_are_the_end_stop(params, sign):
    """Driven past the commanded stroke, the blades meet and stop the crank,
    both ways -- AFTER the stroke the wing keys command, and BEFORE each
    knuckle reaches the other wing's coupler (user, 2026-10-09): held under
    the full 0.55 N.m it must stay inside the 131 deg that test_cad_righting
    clears the knuckle to. (V2's toes meet at 129.75-130.0, the stepthrough's
    geometry; any blade on this linkage must land in the same window.)"""
    _check_end_stop(params, sign)


def _check_end_stop(params, sign):
    first, end = _drive_crank(build_model(params), sign * 150.0)
    assert first is not None, "the blades never met"
    assert _stroke(params) < abs(first) < 131.0
    assert abs(end) < 131.0


def test_the_bare_actuator_holds_without_ringing(params):
    """Regression: with no reflected rotor inertia the V2 crank (2.5e-6 kg m2
    on its own) rang at 1200+ deg/s holding any angle but stow on the bare
    position actuator. `crank_armature` is what stops it."""
    m = build_model(params)
    aid = m.actuator("swing").id
    j = m.joint("swing_crank_joint")
    assert m.dof_armature[j.dofadr[0]] == params["righting"]["module"]["crank_armature"]
    d = mujoco.MjData(m)
    q0 = d.qpos.copy()
    q0[2] += 0.15
    d.qpos[:] = q0
    peak = 0.0
    for _ in range(int(2.0 / m.opt.timestep)):
        d.ctrl[aid] = np.deg2rad(60) * min(1.0, d.time)
        mujoco.mj_step(m, d)
        d.qpos[:7] = q0[:7]
        d.qvel[:6] = 0
        if d.time > 1.5:
            peak = max(peak, abs(d.qvel[j.dofadr[0]]))
    assert np.degrees(peak) < 1.0


def test_the_module_adds_its_listed_mass(params, model):
    """Every gram of the module is a GUESS from CAD volumes; this pins that
    the builder puts exactly the listed ones on the bike (and the roof)."""
    mod, rg = params["righting"]["module"], params["righting"]
    bare = build_model(params, swing_linkage=False)
    listed = (mod["crank"]["mass"] + 2 * mod["coupler"]["mass"]
              + 2 * (mod["wing_hub"]["mass"] + mod["blade"]["mass"])
              + mod["fixed"]["mass"] + rg["roof"]["mass"])
    assert model.body_subtreemass[1] - bare.body_subtreemass[1] == \
        pytest.approx(listed, abs=1e-9)


def test_the_digest_sees_the_blade_file(params, tmp_path):
    """The blade is inlined into params, so a changed blade file moves the
    plant digest even though bike_params.yaml did not change."""
    blade = yaml.safe_load((ROOT / "config/righting_blade.yaml").read_text())
    blade["outline"][2] = [17.0, 24.0]
    f = tmp_path / "blade.yaml"
    f.write_text(yaml.safe_dump(blade))
    p = load_params()
    mod = p["righting"]["module"]
    for k in ("linkage", "blade_outline", "blade_y"):
        mod.pop(k)
    mod["blade_file"] = str(f)
    assert plant_digest(resolve_righting_module(p)) != plant_digest(params)


@pytest.mark.parametrize("kw", (
    dict(swing_linkage=False), dict(wings=True), dict(righting=True),
    dict(swing=True), dict(swing_linkage=True, swing_linkage_cfg=SWING_LINKAGE_CFG)))
def test_the_other_mechanisms_still_build_instead(params, kw):
    m = build_model(params, **kw)
    names = {m.geom(g).name for g in range(m.ngeom)}
    assert "swing_wing_right_toe" not in names and "righting_fixed" not in names


def test_the_ground_station_strokes_the_module(params):
    """The real station's default stroke is the module's -- with its blade's
    own `crank_travel_deg` if the file gives one -- not V1's 136.6, which on
    V2 hardware drives the toes into each other."""
    from aow_sim.hw.ground import module_travel_deg
    v1 = yaml.safe_load(SWING_LINKAGE_CFG.read_text())["stroke"]["crank_travel_deg"]
    assert module_travel_deg() == pytest.approx(_stroke(params))
    assert module_travel_deg() != pytest.approx(v1)


# ---------------------------------------------------------------- blade outlines
# params.blade_points (the file format: points, runs, cubic curves) and
# build_model._convex_pieces (the sim's split of any outline). 2026-10-10.

def _convex(P) -> bool:
    e = np.roll(P, -1, axis=0) - P
    t = e[:, 0] * np.roll(e[:, 1], -1) - e[:, 1] * np.roll(e[:, 0], -1)
    return bool((t >= -1e-9).all() or (t <= 1e-9).all())


# V2's six points as built 2026-10-09, frozen here: the splitter's
# regression case, whatever config/righting_blade.yaml later becomes.
V2_OUTLINE = [[63.9, 147.4], [38.5, 50.6], [15.69, 22.32], [19.57, 19.24],
              [43.3, 49.4], [68.7, 146.1]]


def test_a_plain_outline_is_unchanged_and_v2_keeps_its_two_pieces():
    from aow_sim.build_model import _convex_pieces
    from aow_sim.params import blade_points
    ol = V2_OUTLINE
    assert blade_points(ol) == [[float(a), float(b)] for a, b in ol]
    assert _convex_pieces(np.array(ol, float)) == [[0, 1, 4, 5], [1, 2, 3, 4]]


@pytest.mark.parametrize("end_deg", (0.0, 15.0))
def test_a_curve_is_tangent_to_its_neighbours_by_angle(end_deg):
    """start_deg / end_deg are relative to the straight sections either side;
    a run's dir_deg is from level, counter-clockwise: 90 is straight up."""
    from aow_sim.params import blade_points
    ol = [[0, 0], [10, 0],
          {"curve": {"to": [30, 20], "handles": [8, 8], "start_deg": 0, "end_deg": end_deg}},
          {"dir_deg": 90, "len": 10}]
    P = np.array(blade_points(ol, segments=4000))   # the last facet lags the tangent by half its turn
    assert P[-1] == pytest.approx([30, 30])                  # the run: straight up
    d0, d1 = P[2] - P[1], P[-2] - P[-3]                      # first / last facet of the curve
    assert np.degrees(np.arctan2(d0[1], d0[0])) == pytest.approx(0.0, abs=0.5)
    assert np.degrees(np.arctan2(d1[1], d1[0])) == pytest.approx(90.0 + end_deg, abs=0.5)


@pytest.mark.parametrize("outline", (
    # a curve straight after point 2, into the toe
    [[63.9, 147.4], [38.5, 50.6],
     {"curve": {"to": [15.69, 22.32], "handles": [10, 10], "start_deg": -10}},
     [19.57, 19.24], [43.3, 49.4], [68.7, 146.1]],
    # an inward dent in the outer face (inside the 5 mm blade: the inner face
    # is at 48.8 there, the straight outer face at 54.0)
    [[63.9, 147.4], [38.5, 50.6], [15.69, 22.32], [19.57, 19.24], [43.3, 49.4],
     [51.5, 90.0], [68.7, 146.1]],
    "config/swing_explore/blade_v3.yaml",
))
def test_any_outline_is_cut_into_convex_pieces_that_tile_it(outline):
    from aow_sim.build_model import _convex_pieces, _shoelace
    from aow_sim.params import blade_points
    if isinstance(outline, str):
        outline = yaml.safe_load((ROOT / outline).read_text())["outline"]
    P = np.array(blade_points(outline))
    cut = _convex_pieces(P)
    assert all(_convex(P[q]) for q in cut)
    assert sum(abs(_shoelace(P[q])) for q in cut) == pytest.approx(abs(_shoelace(P)))
    assert any(0 in q for q in cut) and any(2 in q for q in cut)


def test_runs_alone_draw_a_polygon_that_closes_itself():
    from aow_sim.params import blade_points
    sq = [[40, 60], {"dir_deg": 90, "len": 20}, {"dir_deg": 0, "len": 20},
          {"dir_deg": 270, "len": 20}]
    assert np.array(blade_points(sq)) == pytest.approx(np.array([[40, 60], [40, 80], [60, 80], [60, 60]]))


def test_v3_builds_with_its_toe_stop():
    """The exploring V3 blade swapped onto the module, as teleop's --blade
    does: its outline, and its file's mass and stroke (its own level point,
    116.9 -- not the linkage's 129.3, which would drive it 12 deg past
    level). Its toes, V2's, still stop the crank past that and short of the
    knuckles."""
    p = load_params(blade=V3)
    spec = yaml.safe_load((ROOT / V3).read_text())
    mod = p["righting"]["module"]
    assert _stroke(p) == spec["crank_travel_deg"] == pytest.approx(116.9)
    assert mod["blade"]["mass"] == spec["mass"]["value"]
    m = build_model(p)
    assert np.degrees(m.actuator_ctrlrange[m.actuator("swing").id, 1]) == pytest.approx(116.9)
    assert len(_blade_geoms(m, "right")) == len(_blade_geoms(m, "left")) >= 2
    _check_end_stop(p, 1.0)


def test_a_swapped_blade_is_another_plant(params):
    """--blade is a different bike: the digest moves, and it moves the SAME
    whether the path is given relative or absolute (the path is hashed, and
    an absolute one would differ per machine). Naming the module's own file
    is no change at all."""
    own = params["righting"]["module"]["blade_file"]
    other = V3 if Path(own) != Path(V3) else "config/righting_blade.yaml"
    a, b = load_params(blade=other), load_params(blade=ROOT / other)
    assert plant_digest(a) == plant_digest(b) != plant_digest(params)
    assert plant_digest(load_params(blade=own)) == plant_digest(params)
