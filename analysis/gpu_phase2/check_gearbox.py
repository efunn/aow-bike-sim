"""The steer's gearbox-friction kernel (aow_sim.gpu.gearbox) against
GeneralEnv's GearboxFriction, on the recorded reference. The constraint rows
are rebuilt in MJWarp's form (type, id per row) from C MuJoCo's fixed layout
-- rows ne.. are the dof-friction rows, one per dof with frictionloss > 0, in
dof order -- and then SHUFFLED per substep, since MJWarp's order is not fixed.

  python analysis/gpu_phase2/check_gearbox.py
"""
import argparse

import numpy as np
import warp as wp

from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.gpu import gearbox as gg

NROW = 64


def main(ref_path):
    wp.config.quiet = True
    wp.init()
    r = np.load(ref_path, allow_pickle=True)
    env = GeneralEnv(None, _load_rl_config(str(r["config"])))
    c = gg.const_from(env._gearbox[0])
    n = len(r["sub_pre_time"])
    rng = np.random.default_rng(0)

    qvel = r["sub_pre_qvel"].astype(np.float32)
    qfa = r["sub_pre_qfrc_actuator"].astype(np.float32)
    sd = r["sub_pre_sensordata"].astype(np.float32)
    fl = r["sub_pre_frictionloss"]
    ne, nefc, force = r["sub_pre_ne"], r["sub_pre_nefc"], r["sub_pre_efc_force"]
    want = r["sub_hooked_frictionloss"][:, c.dof]

    etype = np.full((n, NROW), -1, np.int32)
    eid = np.full((n, NROW), -1, np.int32)
    eforce = np.zeros((n, NROW), np.float32)
    for t in range(n):
        dofs = np.flatnonzero(fl[t] > 0)
        rows = ne[t] + np.arange(len(dofs))
        assert rows[-1] < NROW and nefc[t] <= NROW
        perm = rng.permutation(nefc[t])          # MJWarp: any order
        for k, (row, dof) in enumerate(zip(rows, dofs)):
            etype[t, perm[row]] = gg.FRICTION_DOF
            eid[t, perm[row]] = dof
            eforce[t, perm[row]] = force[t, row]

    # every substep is independent (no state): one launch, substeps as worlds
    out = wp.array(np.zeros((n, qvel.shape[1]), np.float32), dtype=float)
    gg.launch_pre_step(
        c, wp.array(qvel, dtype=float), wp.array(qfa, dtype=float),
        wp.array(sd, dtype=float), wp.array(etype, dtype=int),
        wp.array(eid, dtype=int), wp.array(eforce, dtype=float),
        wp.array(nefc.astype(np.int32), dtype=int), out)
    got = out.numpy()[:, c.dof]
    err = np.abs(got - want)
    print(f"steer frictionloss: max |err| {err.max():.3g} N m (ref {want.min():.4f}"
          f"-{want.max():.4f}), rel {np.max(err / want):.2g}; substeps {n}")
    # how often each branch ran, so a pass is not one branch only
    w = r["sub_pre_qvel"][:, c.dof]
    print(f"at rest (|w| < {c.w_stick:.3g}): {np.mean(np.abs(w) < c.w_stick):.1%} of substeps")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="traces/gpu_phase2/reference.npz")
    main(ap.parse_args().ref)
