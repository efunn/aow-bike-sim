"""The drivetrain kernel's DETENT (aow_sim.gpu.drivetrain) against the Python
DrivetrainSim, on an overlay with every detent option on -- by default the
10-24 sawtooth (flat friction, sawtooth push, kinetic fraction 0.15).

No recorded flight: each world is one Python DrivetrainSim on the same model,
both driven through the SAME synthetic shaft trajectory -- the differential
creeping back and forth across many 7.5 deg notches with a wobble on top, so
the friction sticks, slips and re-sticks and the sawtooth flips -- with random
velocity commands. Each step the Python class and the kernel see identical
qpos/qvel/ctrl and their outputs (motor torques, applied forces) are compared;
the state each carries (anchor, sliding, servo buffers) evolves on its own.
check_drivetrain.py does the same on a recorded GeneralEnv flight, but only
for the overlay that run trained on.

  PYTHONPATH=src ~/miniforge3/envs/aow-sim-gpu/bin/python analysis/gpu/phase2/check_detent.py
  ... --set detent.hill_flip_deg=1.5 --set detent.friction_minus_scale=1.15
  ... --wobble 0 --cycles 0.3 0.6 --steps 20000      # slow: the friction sticks too
"""
import argparse

import mujoco
import numpy as np
import warp as wp
import yaml

from aow_sim import drivetrain_model as dm
from aow_sim.build_model import build_model, load_params
from aow_sim.gpu import drivetrain as gd


def trajectory(rng, n, h, sim, wobble=0.3, cycles=(1.5, 3.0)):
    """Shaft angles and speeds for one world: phi = ga a + gb b creeping
    +-4 notches and back over the run, a 0.3-notch 6 Hz wobble, common mode
    drifting."""
    t = np.arange(n) * h
    P = sim.period
    creep = 4 * P * np.sin(2 * np.pi * t / (n * h) * rng.uniform(*cycles) + rng.uniform(0, 6.3))
    wob = wobble * P * np.sin(2 * np.pi * 6.0 * t + rng.uniform(0, 6.3))
    phi = creep + wob
    common = rng.uniform(-1, 1) * t
    # phi = ga a + gb b, common = (a + b) / 2  ->  solve for a, b
    A = np.array([[sim.ga, sim.gb], [0.5, 0.5]])
    a, b = np.linalg.solve(A, np.vstack([phi, common]))
    return a, b, np.gradient(a, h), np.gradient(b, h)


def main(overlay, sets, nworld, steps, seed, wobble=0.3, cycles=(1.5, 3.0)):
    wp.config.quiet = True
    wp.init()
    params = load_params()
    # "default": bike_params' own block (the friction bump every run trained on)
    cfg = dict(params[dm.KEY]) if overlay == "default" else dm.load(overlay)
    for kv in sets:
        path, val = kv.split("=", 1)
        part, key = path.split(".", 1)
        cfg[part] = {**cfg[part], key: yaml.safe_load(val)}
    params = {**params, dm.KEY: cfg}
    rng = np.random.default_rng(seed)
    sims, datas, trajs = [], [], []
    for w in range(nworld):
        model = build_model(params)
        sim = dm.DrivetrainSim.attach(model, params)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        sim.reset(data, rng=np.random.default_rng(seed + w))
        sims.append(sim), datas.append(data)
        trajs.append(trajectory(rng, steps, sim.h, sim, wobble, cycles))
    sim0 = sims[0]
    c = gd.const_from(sim0)
    st = gd.DriveState(nworld, c.nc, c.nm)
    for w, sim in enumerate(sims):
        gd.load_state(st, w, vars(sim))
    nq, nv, nu = datas[0].qpos.size, datas[0].qvel.size, datas[0].ctrl.size
    d_qpos = wp.zeros((nworld, nq), dtype=float)
    d_qvel = wp.zeros((nworld, nv), dtype=float)
    d_ctrl = wp.zeros((nworld, nu), dtype=float)
    d_frc = wp.zeros((nworld, nv), dtype=float)
    qa, qb, da, db = sim0.in_q[0], sim0.in_q[1], sim0.in_d[0], sim0.in_d[1]
    got, ref = np.zeros((steps, nworld, 4)), np.zeros((steps, nworld, 4))
    slid = np.zeros((steps, nworld), dtype=int)
    hill_on = np.zeros((steps, nworld))
    for k in range(steps):
        Q, V, C = np.zeros((nworld, nq)), np.zeros((nworld, nv)), np.zeros((nworld, nu))
        for w, (sim, data) in enumerate(zip(sims, datas)):
            a, b, va, vb = (x[k] for x in trajs[w])
            data.time = k * sim.h
            data.qpos[qa], data.qpos[qb] = a, b
            data.qvel[da], data.qvel[db] = va, vb
            for i in sim.cmd_id:
                data.ctrl[i] = rng.uniform(-20, 20)
            Q[w], V[w], C[w] = data.qpos, data.qvel, data.ctrl
            sim.pre_step(data)
            ref[k, w] = (data.ctrl[sim.motor_id[0]], data.ctrl[sim.motor_id[1]],
                         data.qfrc_applied[da], data.qfrc_applied[db])
            slid[k, w] = int(sim._sliding)
            hill_on[k, w] = sim.hill_force
        d_qpos.assign(Q.astype(np.float32))
        d_qvel.assign(V.astype(np.float32))
        d_ctrl.assign(C.astype(np.float32))
        gd.launch_pre_step(c, st, d_qpos, d_qvel, d_ctrl, d_frc)
        cg, fg = d_ctrl.numpy(), d_frc.numpy()
        got[k] = np.stack([cg[:, sim0.motor_id[0]], cg[:, sim0.motor_id[1]],
                           fg[:, da], fg[:, db]], 1)
    print(f"overlay {overlay} {' '.join(sets)}: {nworld} worlds x {steps} steps; "
          f"sliding on {slid.mean():.0%} of steps, |hill| up to {np.abs(hill_on).max():.3f} N m")
    for j, name in enumerate(("motor a", "motor b", "applied a", "applied b")):
        err = np.abs(got[..., j] - ref[..., j])
        scale = np.abs(ref[..., j]).max()
        print(f"  {name:10s} max |err| {err.max():.3g} (scale {scale:.3g}, rel "
              f"{err.max() / scale:.2g}); p99 {np.percentile(err, 99):.3g}")
    return got, ref


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--overlay", default="traces/drivetrain/drivetrain_sawtooth_1024.yaml")
    ap.add_argument("--set", dest="sets", action="append", default=[], metavar="PART.KEY=VALUE")
    ap.add_argument("--nworld", type=int, default=4)
    ap.add_argument("--steps", type=int, default=6000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--wobble", type=float, default=0.3, help="6 Hz wobble, notches")
    ap.add_argument("--cycles", type=float, nargs=2, default=(1.5, 3.0),
                    help="creep cycles over the run (a range, drawn per world)")
    a = ap.parse_args()
    main(a.overlay, a.sets, a.nworld, a.steps, a.seed, a.wobble, tuple(a.cycles))
