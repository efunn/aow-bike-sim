"""The front tire's cross-section: the plain crown and the flat-tread variant.

Invalidated by `geometry.crowned_wheel_vertices` or `run_drive`'s
`--front-wheel` override.
"""

import numpy as np
import pytest

from aow_sim import geometry
from aow_sim.run_drive import FRONT_WHEEL_PRESETS, _front_wheel_override

pytestmark = pytest.mark.pure


def _profile(v):
    """(z, r) per vertex, the two axis points dropped."""
    r = np.hypot(v[:, 0], v[:, 1])
    keep = r > 0
    return v[keep, 2], r[keep]


def test_flat_zero_is_the_plain_crown_vertex_for_vertex():
    # Written out as the function was before flat_width existed, so a change
    # to the default path cannot hide behind a shared helper.
    radius, width, crown, seg = 0.05, 0.028, 0.014, 64
    z = np.linspace(-width / 2, width / 2, 9)
    r = radius - crown + np.sqrt(crown**2 - z**2)
    th = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    want = np.vstack([np.column_stack([ri * np.cos(th), ri * np.sin(th),
                                       np.full(seg, zi)]) for zi, ri in zip(z, r)]
                     + [np.array([[0, 0, -width / 2], [0, 0, width / 2]])])
    got = geometry.crowned_wheel_vertices(radius, width, crown, seg)
    assert np.array_equal(got, want)


def _on_arc(z, r, cz, cr, rad):
    return np.allclose(np.hypot(z - cz, r - cr), rad)


def test_flat_preset_is_a_flat_tread_then_quarter_round_shoulders():
    d, w, sh, _ = (x / 1000 for x in FRONT_WHEEL_PRESETS["flat"])
    z, r = _profile(geometry.crowned_wheel_vertices(
        d / 2, w, np.inf, 64, shoulder_radius=sh))
    flat = w / 2 - sh
    on_flat = np.abs(z) <= flat + 1e-12
    assert np.allclose(r[on_flat], d / 2)
    assert np.isclose(np.abs(z[on_flat]).max(), 0.005)   # 24 - 2 x 7 = 10 mm
    assert _on_arc(np.abs(z[~on_flat]), r[~on_flat], flat, d / 2 - sh, sh)
    # The shoulder comes round to vertical at the sidewall.
    assert np.isclose(np.abs(z).max(), w / 2)
    assert np.isclose(r[np.isclose(np.abs(z), w / 2)].min(), d / 2 - sh)


@pytest.mark.parametrize("crown", [0.013, 0.03, 0.06, 0.5])
def test_crowned_tread_meets_its_shoulders_tangent(crown):
    R, w, sh = 0.05125, 0.024, 0.007
    z, r = _profile(geometry.crowned_wheel_vertices(
        R, w, crown, 64, shoulder_radius=sh))
    alpha = np.arcsin((w / 2 - sh) / (crown - sh))
    zc, rc0 = (crown - sh) * np.sin(alpha), R - crown + (crown - sh) * np.cos(alpha)
    tread = np.abs(z) <= crown * np.sin(alpha) + 1e-12   # the tangent point
    assert _on_arc(z[tread], r[tread], 0.0, R - crown, crown)
    assert _on_arc(np.abs(z[~tread]), r[~tread], zc, rc0, sh)
    # Tangent: the shoulder centre lies on the crown's radius, Rc - Rs in.
    assert np.isclose(np.hypot(zc, rc0 - (R - crown)), crown - sh)
    assert np.isclose(r.max(), R) and np.isclose(np.abs(z).max(), w / 2)


def test_rejects_impossible_shapes():
    for crown, sh in ((0.010, 0.007), (np.inf, 0.013), (np.inf, 0.0)):
        with pytest.raises(ValueError):
            geometry.crowned_wheel_vertices(0.05, 0.024, crown, shoulder_radius=sh)


def test_override_replaces_shape_only():
    params = {"bike": {"wheelbase": 0.2, "front_wheel": {
        "radius": 0.05125, "width": 0.024, "shoulder_radius": 0.007,
        "crown_radius": 0.08, "mass": 0.06}}}
    fw = _front_wheel_override(params, "flat")["bike"]["front_wheel"]
    assert (fw["radius"], fw["width"], fw["shoulder_radius"]) \
        == pytest.approx((0.05125, 0.024, 0.007))
    assert np.isinf(fw["crown_radius"]) and fw["mass"] == 0.06
    old = _front_wheel_override(params, "old")["bike"]["front_wheel"]
    assert "shoulder_radius" not in old
    assert (old["radius"], old["width"], old["crown_radius"]) \
        == pytest.approx((0.05, 0.028, 0.014))
    assert params["bike"]["front_wheel"]["crown_radius"] == 0.08   # untouched
    assert _front_wheel_override(params, "102.5,24,7,40")["bike"]["front_wheel"] \
        ["crown_radius"] == pytest.approx(0.04)
    for bad in ("100,28", "round", "100,24,13,inf"):
        with pytest.raises(SystemExit):
            _front_wheel_override(params, bad)
