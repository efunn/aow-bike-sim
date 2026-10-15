"""Keep the drivetrain's free-spinning joint angles small in float32.

MJWarp's qpos is float32. The input shafts reach ~100 rad in 5 s of v3_s0
and ~1000 rad in 15 s at full speed, where float32's spacing times the stick
springs (250 and 1000 N m/rad) is comparable to the stick friction itself
(docs/plans/gpu-training.md, phase 2). This shifts every coupled angle back
by an amount that leaves the plant EXACTLY as it was, once per policy step.

What "exactly" requires (the tendons: hub = (a+b)/2, ring = a - hub =
(a-b)/2, rollers = 2.4 x ring; the detent on phi = (a-b)/2, period 7.5 deg;
the encoders 4096/rev a 3:1 belt behind the input shafts):

  da = 6*pi*i, db = 6*pi*j  (a whole servo revolution each)
  hub   += (da+db)/2 = 3*pi*(i+j)   -> whole turns iff i+j even
  ring  += (da-db)/2 = 3*pi*(i-j)   -> whole turns iff i-j even
  roller+= 2.4 x ring               -> whole turns iff (i-j) % 5 == 0
  phi   += 3*pi*(i-j)               -> whole 7.5 deg periods

so i - j a multiple of 10: the COMMON mode moves in 6*pi steps, the
DIFFERENTIAL in 60*pi (30*pi of phi). Afterwards |a|, |b| <~ 56 rad.
The front wheel's spin is independent and wraps by 2*pi.

The state the hooks keep in the same angles moves with it: the servo's
reference and measurement buffers (servo side, /belt), the stick anchors,
the detent anchor, the odometry's last encoder counts (4096 per servo turn).
"""
from __future__ import annotations

import numpy as np
import warp as wp

TWO_PI = 2.0 * np.pi


@wp.struct
class RebaseConst:
    qa: int
    qb: int
    hub: int
    ring: int
    roller0: int
    nroller: int
    k_roller: float
    front: int
    belt: float
    counts_per_turn: int


def const_from(env) -> RebaseConst:
    m = env.model
    q = lambda n: int(m.joint(n).qposadr[0])
    rollers = [q(f"roller_spin_{i}") for i in range(8)]
    if rollers != list(range(rollers[0], rollers[0] + 8)):
        raise ValueError("roller joints are not contiguous in qpos")
    dt = env.p["drivetrain"]
    if (float(dt["mix_hub_a"]), float(dt["mix_hub_b"]), float(dt["mix_ring_a"]),
            float(dt["mix_ring_b"]), float(dt["k_roller"])) != (0.5, 0.5, 1.0, 0.0, 2.4):
        raise NotImplementedError("the multiples are worked out for this gearing")
    c = RebaseConst()
    c.qa, c.qb = q("input_a_spin"), q("input_b_spin")
    c.hub, c.ring = q("hub_spin"), q("ring_spin")
    c.roller0, c.nroller, c.k_roller = rollers[0], 8, 2.4
    c.front = q("front_spin")
    c.belt = float(env._drive.belt)
    from ..hw.dynamixel import POS_WRAP
    c.counts_per_turn = int(POS_WRAP)
    return c


@wp.kernel
def rebase(c: RebaseConst, qpos: wp.array2d(dtype=float),
           s_ref: wp.array2d(dtype=float), s_th: wp.array3d(dtype=float),
           s_fa: wp.array2d(dtype=float), anchor: wp.array(dtype=float),
           o_prev: wp.array2d(dtype=int), shifts: wp.array2d(dtype=int)):
    world = wp.tid()
    unit = 6.0 * wp.pi                          # one servo turn of input
    a = qpos[world, c.qa]
    b = qpos[world, c.qb]
    s = 0.5 * (a + b)
    dlt = 0.5 * (a - b)
    n = int(wp.round(dlt / (5.0 * unit)))       # differential: 30*pi of phi
    j = int(wp.round((s - 5.0 * unit * float(n)) / unit))
    i = j + 10 * n
    shifts[world, 0] = i
    shifts[world, 1] = j
    if i != 0 or j != 0:
        da = unit * float(i)
        db = unit * float(j)
        dhub = 0.5 * (da + db)
        dring = 0.5 * (da - db)
        qpos[world, c.qa] = a - da
        qpos[world, c.qb] = b - db
        qpos[world, c.hub] = qpos[world, c.hub] - dhub
        qpos[world, c.ring] = qpos[world, c.ring] - dring
        for r in range(c.nroller):
            qpos[world, c.roller0 + r] = qpos[world, c.roller0 + r] - c.k_roller * dring
        s_ref[world, 0] = s_ref[world, 0] - da / c.belt
        s_ref[world, 1] = s_ref[world, 1] - db / c.belt
        for k in range(s_th.shape[2]):
            s_th[world, 0, k] = s_th[world, 0, k] - da / c.belt
            s_th[world, 1, k] = s_th[world, 1, k] - db / c.belt
        s_fa[world, 0] = s_fa[world, 0] - da
        s_fa[world, 1] = s_fa[world, 1] - db
        anchor[world] = anchor[world] - dring       # phi = (a - b)/2
        o_prev[world, 0] = o_prev[world, 0] - c.counts_per_turn * i
        o_prev[world, 1] = o_prev[world, 1] - c.counts_per_turn * j
    f = qpos[world, c.front]
    qpos[world, c.front] = f - 2.0 * wp.pi * wp.round(f / (2.0 * wp.pi))
