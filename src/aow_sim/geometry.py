"""Procedural mesh generation for contact geometry.

All meshes are vertex clouds; MuJoCo computes the convex hull, which is exact
for these shapes (truncated cone, crowned disc) up to tessellation.
"""

from __future__ import annotations

import numpy as np


def truncated_cone_vertices(
    big_radius: float, small_radius: float, length: float, segments: int = 32
) -> np.ndarray:
    """Truncated cone along local +Z, centered at its midpoint.

    Big end at z = -length/2, small end at z = +length/2. Returns (N, 3) vertices.
    """
    if not (big_radius >= small_radius > 0 and length > 0):
        raise ValueError("expected big_radius >= small_radius > 0 and length > 0")
    theta = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    ring = np.stack([np.cos(theta), np.sin(theta)], axis=1)
    big = np.column_stack([big_radius * ring, np.full(segments, -length / 2)])
    small = np.column_stack([small_radius * ring, np.full(segments, length / 2)])
    caps = np.array([[0.0, 0.0, -length / 2], [0.0, 0.0, length / 2]])
    return np.vstack([big, small, caps])


def crowned_wheel_vertices(
    radius: float,
    width: float,
    crown_radius: float,
    segments: int = 32,
    profile_points: int = 9,
) -> np.ndarray:
    """Solid of revolution about local Z: a disc with a circular-arc crowned rim.

    Cross-section radius at axial offset z: r(z) = radius - crown_radius
    + sqrt(crown_radius^2 - z^2). Requires width/2 <= crown_radius. The convex
    hull closes the flat sides via the two axis points. Returns (N, 3) vertices.
    """
    half = width / 2
    if half > crown_radius:
        raise ValueError("width/2 must be <= crown_radius for a circular-arc crown")
    z = np.linspace(-half, half, profile_points)
    r = radius - crown_radius + np.sqrt(crown_radius**2 - z**2)
    theta = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    rings = [
        np.column_stack([ri * np.cos(theta), ri * np.sin(theta), np.full(segments, zi)])
        for zi, ri in zip(z, r)
    ]
    caps = np.array([[0.0, 0.0, -half], [0.0, 0.0, half]])
    return np.vstack(rings + [caps])


def roller_radius_at(s: np.ndarray, roller: dict) -> np.ndarray:
    """Roller surface radius at axial offset |s| from the axle midpoint.

    The two cones span |s| in [pair_gap/2, pair_gap/2 + length]; radius tapers
    linearly from big to small (big_end_inward=True) or the reverse. NaN outside.
    """
    s = np.abs(np.asarray(s, dtype=float))
    s0 = roller["pair_gap"] / 2
    s1 = s0 + roller["length"]
    frac = (s - s0) / roller["length"]
    r_in, r_out = roller["big_diameter"] / 2, roller["small_diameter"] / 2
    if not roller.get("big_end_inward", True):
        r_in, r_out = r_out, r_in
    r = r_in + (r_out - r_in) * frac
    return np.where((s >= s0) & (s <= s1), r, np.nan)


def envelope_radius(s: np.ndarray, axle_mount_radius: float, roller: dict) -> np.ndarray:
    """Approximate wheel-envelope radius at roller axial offset s.

    Distance from the wheel axis to the roller surface point radially outward at
    station s: sqrt(mount_radius^2 + s^2) + roller_radius(s). Slight overestimate
    (ignores the tilt of the radial direction relative to the cone cross-section);
    good enough to validate measured cone dimensions against the measured
    envelope, not a contact computation.
    """
    s = np.asarray(s, dtype=float)
    return np.sqrt(axle_mount_radius**2 + s**2) + roller_radius_at(s, roller)


def envelope_deviation(outer_radius: float, axle_mount_radius: float, roller: dict) -> float:
    """Max |envelope - outer_radius| over the roller span (meters)."""
    s0 = roller["pair_gap"] / 2
    s1 = s0 + roller["length"]
    s = np.linspace(s0, s1, 50)
    env = envelope_radius(s, axle_mount_radius, roller)
    return float(np.max(np.abs(env - outer_radius)))
