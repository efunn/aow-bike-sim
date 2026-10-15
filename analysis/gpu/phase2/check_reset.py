"""The reset (aow_sim.gpu.reset, through GpuGeneralEnv) against
GeneralEnv.reset.

  1. randomization OFF: the start state is deterministic (but for the AHRS
     realisation and the first command), so compare it field by field.
  2. randomization ON: the drawn quantities' ranges and means over many
     worlds against the config's.

  python analysis/gpu/phase2/check_reset.py
"""
import argparse

import numpy as np
import warp as wp

from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.gpu.env import GpuGeneralEnv

CONFIG = "config/rl_general_v3.yaml"


def _rand_off(cfg):
    cfg["randomization"] = {**cfg["randomization"], "enabled": False}
    return cfg


def deterministic():
    g = GpuGeneralEnv(CONFIG, nworld=4, seed=0, cfg_override=_rand_off)
    obs = g.reset().numpy()
    cpu = GeneralEnv(None, _rand_off(_load_rl_config(CONFIG)), seed=0)
    o_py, _ = cpu.reset(seed=0)
    d = cpu.data
    q = g.d.qpos.numpy()[0]
    print("randomization off, world 0 vs GeneralEnv.reset:")
    print(f"  qpos max |diff| {np.abs(q - d.qpos).max():.3g}   qvel "
          f"{np.abs(g.d.qvel.numpy()[0] - d.qvel).max():.3g}   act "
          f"{np.abs(g.d.act.numpy()[0] - d.act).max():.3g}")
    print(f"  steer delay line (vs a fresh line at the env's clock): "
          f"values {np.abs(g.d.history.numpy()[0][13:] - d.history[13:]).max():.3g}")
    s = g.s_drv
    dr = cpu._drive
    print(f"  drive ref {np.abs(s.ref.numpy()[0] - dr._ref).max():.3g}  wf "
          f"{np.abs(s.wf.numpy()[0] - dr._wf).max():.3g}  fa "
          f"{np.abs(s.fa.numpy()[0] - dr._fa).max():.3g}  anchor "
          f"{abs(s.anchor.numpy()[0] - dr._anchor):.3g}  th buffers "
          f"{np.abs(s.th.numpy()[0] - np.array(dr._th)).max():.3g}")
    fl = g.m.dof_frictionloss.numpy()[0, g.c_gb.dof]
    print(f"  steer frictionloss {fl:.6f} vs {cpu.model.dof_frictionloss[g.c_gb.dof]:.6f}")
    print(f"  psi {g.psi.numpy()[0]:.3g} vs {cpu._psi:.3g}; steer target "
          f"{g.steer_target.numpy()[0]:.3g} vs {cpu._steer:.3g}; step "
          f"{g.step_i.numpy()[0]} vs {cpu._step}")
    print("  obs entries, world 0 vs python (5, 6, 7, 12-14 deterministic; "
          "0-2 and 15-16 carry the AHRS draw; 8-11 the command draw):")
    print("   ", np.array2string(np.abs(obs[0] - o_py), precision=4))
    sd = np.abs(g.d.sensordata.numpy()[0] - d.sensordata).max()
    print(f"  sensordata after the guarded forward {sd:.3g}; nefc {g.d.nefc.numpy()[0]} vs {d.nefc}")


def distributions(n):
    g = GpuGeneralEnv(CONFIG, nworld=n, seed=1)
    g.reset()
    cpu = g.cpu
    r = cpu.rand
    mass = g.m.body_mass.numpy() / cpu._rand._mass0[None]
    fr = g.m.geom_friction.numpy()[:, :, 0] / cpu._rand._friction0[None, :, 0]
    fr0 = cpu._rand._forcerange0
    nz = fr0[:, 1] != 0
    sup = g.m.actuator_forcerange.numpy()[:, nz, 1] / fr0[nz, 1]
    q = g.d.qpos.numpy()
    roll = np.degrees(np.arctan2(2 * (q[:, 3] * q[:, 4] + q[:, 5] * q[:, 6]),
                                 1 - 2 * (q[:, 4] ** 2 + q[:, 5] ** 2)))
    ph = g.s_drv.phase.numpy()
    print(f"\nrandomization on, {n} worlds (expected: uniform ranges from the config):")
    rows = (("mass scale", mass[:, cpu._rand._mass0 > 0].ravel(), 1 - r["mass_frac"], 1 + r["mass_frac"]),
            ("friction scale (one per world)", fr[:, 0], 1 - r["friction_frac"], 1 + r["friction_frac"]),
            ("supply scale", sup[:, 0], 1 - r["actuator_frac"], 1 + r["actuator_frac"]),
            ("init roll deg", roll, -r["init_roll_deg"], r["init_roll_deg"]),
            ("detent phase", ph, 0.0, g.c_drv.period))
    for name, x, lo, hi in rows:
        print(f"  {name:30s} min {x.min():8.4f} max {x.max():8.4f} mean {x.mean():8.4f}"
              f"   (want [{lo:.4f}, {hi:.4f}], mean {(lo + hi) / 2:.4f})")
    same = np.allclose(fr, fr[:, :1]), np.allclose(sup, sup[:, :1])
    print(f"  one friction draw per world: {same[0]}; one supply draw per world: {same[1]}")
    ctrl = g.m.actuator_ctrlrange.numpy()[:, :, 1]
    c0 = cpu._rand._ctrlrange0[:, 1]
    k = np.flatnonzero(c0)
    print(f"  ctrlrange scales with the same supply draw: "
          f"{np.allclose(ctrl[:, k] / c0[k], sup[:, :1], atol=1e-6)}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=2000)
    a = ap.parse_args()
    wp.config.quiet = True
    wp.init()
    deterministic()
    distributions(a.n)
