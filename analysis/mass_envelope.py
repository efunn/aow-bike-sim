"""How heavy can this bike be, and does mass quiet the chatter?

Two questions that need two different controllers, which is the whole design of
this script:

  CAN it stay up?      Ask the LQR. It is RE-IDENTIFIED against whatever model
                       it is handed (control/linearize.py runs finite-amplitude
                       rollouts every time), so at 2x mass it designs gains for
                       a 2x bike. A failure here is the PLANT running out of
                       actuator authority.
  Does the POLICY cope? Ask a trained export. Its weights are frozen at the
                       nominal mass, so a failure here is off-distribution, not
                       impossible. Reading one for the other is the trap: an RL
                       fall at 2x says nothing about whether 2x is flyable.

MASS AND INERTIA SCALE TOGETHER. control/randomize.py scales `body_mass` alone,
which is defensible at its +-10% but not here -- a body with 3x the mass and
1x the inertia is not a heavier bike, it is a physically impossible one. This
scales `body_inertia` by the same factor, i.e. uniform density at fixed
geometry. Contact stiffness is NOT scaled, so a heavy bike really does sink
further, which is the honest consequence of loading the same wheel.

  python analysis/mass_envelope.py --study scale
  python analysis/mass_envelope.py --study com --policy general_rl_glide_pitch_hub2

Read-only: builds its own models, never writes a config or a move.
"""
from __future__ import annotations

import argparse
import warnings

import mujoco
import numpy as np

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import extract_state, make_controller
from aow_sim.control.general_env import _load_rl_config
from aow_sim.control.linearize import settle_upright
from rsa_policies import env_for, load_general, REPO

CONFIG_FOR = {
    "general_rl_glide_pitch_dt4e4": "rl_general_glide_pitch.yaml",
    "general_rl_glide_pitch_hub":   "rl_general_glide_pitch_hub.yaml",
    "general_rl_glide_pitch_hub2":  "rl_general_glide_pitch_hub2.yaml",
    "general_rl_glide_pitch_hub3":  "rl_general_glide_pitch_hub3.yaml",
}


def scale_mass(model, k, com_shift=0.0):
    """Uniform density scale, plus an optional CoM height shift [m] on the
    chassis. Inertia moves with mass; geometry does not move at all."""
    model.body_mass[:] *= k
    model.body_inertia[:] *= k
    if com_shift:
        model.body("chassis").ipos[2] += com_shift


def lqr_trial(k, com_shift, tilt_deg=3.0, seconds=6.0):
    p = load_params()
    m = build_model(p)
    scale_mass(m, k, com_shift)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        eq = settle_upright(m)
        c = make_controller("lqr", p, m)
    d = mujoco.MjData(m)
    d.qpos[:] = eq.qpos
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array([1.0, 0, 0]), np.deg2rad(tilt_deg))
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, q, d.qpos[3:7])
    d.qpos[3:7] = out
    mujoco.mj_forward(m, d)
    c.reset(m, d)
    every = max(1, round((1.0 / p["control"]["rate_hz"]) / m.opt.timestep))
    roll = []
    for i in range(int(seconds / m.opt.timestep)):
        if i % every == 0:
            c.step(m, d)
        mujoco.mj_step(m, d)
        roll.append(abs(extract_state(d, eq.qpos[:2]).roll))
    roll = np.degrees(roll)
    return roll.max() < 25.0, float(np.min(c.fit_r2)), float(roll[-2500:].std())


def rl_trial(name, k, com_shift, seconds=12.0):
    p = load_params()
    cfg = _load_rl_config(REPO / "config" / CONFIG_FOR.get(name, "rl_general.yaml"))
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        pol = load_general(name)
        env = env_for(pol, p, cfg)
    scale_mass(env.model, k, com_shift)
    env._eq = settle_upright(env.model).qpos.copy()   # heavier sinks further
    sc = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    sc = np.where(sc > 0, sc, 1.0)
    hub = env.model.joint("hub_spin").dofadr[0]
    obs, _ = env.reset(seed=7, options={"v_cmd": (0.0, 0.0),
                                        "psi_cmd_rel": 0.0, "difficulty": 1.0})
    env.data.qacc_warmstart[:] = 0.0
    H, fell = [], False
    for _ in range(int(seconds / env.ctrl_dt)):
        a = (np.asarray(pol.action(obs), float) / sc)[:env.action_space.shape[0]]
        obs, _r, term, _tr, _i = env.step(a)
        H.append(env.data.qvel[hub])
        if term:
            fell = True
            break
    H = np.array(H)
    R = load_params()["omni_wheel"]["outer_radius"]
    return (not fell), np.abs(H).mean() * 60 / (2 * np.pi), np.abs(H).mean() * R * seconds


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--study", choices=["scale", "com"], default="scale")
    ap.add_argument("--policy", default="general_rl_glide_pitch_dt4e4")
    args = ap.parse_args()
    base = float(build_model(load_params()).body_mass.sum())

    if args.study == "scale":
        print(f"uniform mass scale (nominal {base*1000:.0f} g), 3 deg tilt for the LQR, "
              f"12 s hold for {args.policy}\n")
        print(f"{'x':>5}{'total g':>9}{'LQR up':>8}{'fit R2':>8}{'tail rms':>10}"
              f"{'RL up':>7}{'hub rpm':>9}{'rim':>8}")
        for k in (0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0):
            up, r2, rms = lqr_trial(k, 0.0)
            rup, rpm, rim = rl_trial(args.policy, k, 0.0)
            print(f"{k:5.2f}{base*k*1000:9.0f}{'yes' if up else 'FELL':>8}{r2:8.3f}"
                  f"{rms:10.3f}{'yes' if rup else 'FELL':>7}{rpm:9.1f}{rim:7.2f}m",
                  flush=True)
    else:
        print(f"chassis CoM height shift at nominal mass, same trials\n")
        print(f"{'shift mm':>9}{'LQR up':>8}{'fit R2':>8}{'tail rms':>10}"
              f"{'RL up':>7}{'hub rpm':>9}{'rim':>8}")
        for mm in (-40, -20, 0, 20, 40, 60):
            up, r2, rms = lqr_trial(1.0, mm / 1000.0)
            rup, rpm, rim = rl_trial(args.policy, 1.0, mm / 1000.0)
            print(f"{mm:9.0f}{'yes' if up else 'FELL':>8}{r2:8.3f}{rms:10.3f}"
                  f"{'yes' if rup else 'FELL':>7}{rpm:9.1f}{rim:7.2f}m", flush=True)


if __name__ == "__main__":
    main()
