"""Roller stripes: the decoration that makes rear-wheel rotation readable.

The omni wheel's rollers ship near-black and are surfaces of revolution, so a
spinning roller and a stationary one look identical. Painting stripes along
each cone gives the eye something to track, which is the whole reason
`analysis/wheel_slowmo.py` is legible at all.

This lives in `src/` rather than in that script because teleop draws it too
(run_drive's `wheel` camera). One implementation, so a stripe count or a
colour changed for one is changed for both -- the alternative is two drawings
that drift and stop being comparable, which defeats the point of having the
view in teleop at all.

Both halves are pure scene decoration: they append to an `mjvScene` and touch
neither model nor data.
"""

from __future__ import annotations

import mujoco
import numpy as np

N_AXLES = 8
STRIPES = 4                          # per cone, every 90 deg
C_STRIPE = (0.96, 0.97, 1.00, 1.0)   # bright: the rollers themselves are dark
C_HOT = (1.00, 0.45, 0.10, 1.0)      # the roller currently on the floor
# Capsule radius [m]. wheel_slowmo frames the wheel across most of a panel and
# 0.6 mm is a clean hairline there; teleop's `wheel` camera stands the same
# 0.45 m off but renders into a shared viewport, so the same stripe lands under
# a pixel wide on a near-black roller and vanishes. Hence a parameter with the
# slowmo value as the default -- the clips stay byte-identical.
STRIPE_RADIUS = 6e-4
STRIPE_RADIUS_TELEOP = 1.6e-3


def stripe_frames(model, p):
    """Where the stripes sit on each cone, in axle-body coordinates.

    Each roller axle body spins about its own joint axis, so a point fixed in
    that body's frame rides the roller. The stripes follow the cone taper (big
    end inboard) rather than sitting at one radius, so they lie ON the surface
    instead of floating off the small end.

    Computed once against the model; the result is static and gets transformed
    by the live body pose every frame in `add_stripes`.
    """
    r = p["omni_wheel"]["roller"]          # load_params has already resolved
    r_big = r["big_diameter"] / 2          #   the {value, source} leaves
    r_small = r["small_diameter"] / 2
    length, gap = r["length"], r["pair_gap"]
    eps = 3e-4                                   # lift clear of the surface
    out = []
    for i in range(N_AXLES):
        axis = np.array(model.joint(f"roller_spin_{i}").axis, float)
        axis /= np.linalg.norm(axis)
        # any two unit vectors spanning the plane normal to the axle
        tmp = np.array([0.0, 1.0, 0.0])
        if abs(tmp @ axis) > 0.9:
            tmp = np.array([1.0, 0.0, 0.0])
        u = np.cross(axis, tmp); u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        segs = []
        for sign in (+1, -1):                    # the two cones of the pair
            z_in, z_out = sign * gap / 2, sign * (gap / 2 + length)
            for k in range(STRIPES):
                th = 2 * np.pi * k / STRIPES
                dirv = np.cos(th) * u + np.sin(th) * v
                segs.append(((r_big + eps) * dirv + z_in * axis,
                             (r_small + eps) * dirv + z_out * axis))
        out.append((model.body(f"roller_axle_{i}").id, segs))
    return out


def add_stripes(scene, data, frames, hot=(), radius=STRIPE_RADIUS):
    """Draw the stripes at the current pose. `hot` = body ids to highlight,
    normally whichever roller is on the floor."""
    for bid, segs in frames:
        pos, R = data.xpos[bid], data.xmat[bid].reshape(3, 3)
        rgba = np.asarray(C_HOT if bid in hot else C_STRIPE, np.float32)
        for p0, p1 in segs:
            if scene.ngeom >= scene.maxgeom:
                return
            g = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                                np.zeros(3), np.zeros(9), rgba)
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                 radius, pos + R @ p0, pos + R @ p1)
            scene.ngeom += 1
