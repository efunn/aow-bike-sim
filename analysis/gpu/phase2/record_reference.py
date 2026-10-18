"""Record GeneralEnv's hooks, substep by substep, as the reference the Warp
port is tested against (docs/plans/gpu-training.md, phase 2).

Whole trajectories cannot be compared across engines -- MJWarp's contacts put
the chassis 0.7 mm / 0.24 deg off C MuJoCo within 100 ms (phase 1) -- so the
port is checked PIECE BY PIECE: each Warp kernel gets the inputs recorded
here and must reproduce the outputs recorded here, carrying its own state.

Per physics substep, three snapshots of the fields the hooks read or write:

  pre     before the drivetrain hook (what it reads)
  hooked  after every pre-step hook, before mj_step (what they wrote)
  post    after mj_step and the odometry/AHRS tick (what they read next)

plus, per policy step, the action, obs, reward, flags and the env's own
scalars (command, heading, steer target, difficulty), and the hooks' internal
state right after each reset. The v3 recipe only (config/rl_general_v3.yaml),
driven by the v3_s0 policy so the bike does what a trained bike does.

  python analysis/gpu/phase2/record_reference.py --episodes 2 --steps 250
"""
import argparse
import copy
import time
from pathlib import Path

import numpy as np

from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.policy import load_policy_npz

SNAP = ("time", "qpos", "qvel", "ctrl", "act", "qfrc_applied", "qfrc_actuator",
        "sensordata")


def _snap(env, tag, out):
    d = env.data
    for f in SNAP:
        out.setdefault(f"{tag}_{f}", []).append(np.array(getattr(d, f), np.float64, copy=True))
    out.setdefault(f"{tag}_frictionloss", []).append(env.model.dof_frictionloss.copy())
    out.setdefault(f"{tag}_nefc", []).append(int(d.nefc))
    out.setdefault(f"{tag}_ne", []).append(int(d.ne))
    out.setdefault(f"{tag}_efc_force", []).append(
        np.pad(np.array(d.efc_force[:d.nefc]), (0, max(0, 64 - d.nefc)))[:64])


def _hook_state(obj):
    """The plain-number state of a hook object: floats, ints, lists, tuples."""
    keep = {}
    for k, v in vars(obj).items():
        if isinstance(v, (bool, int, float)):
            keep[k] = v
        elif isinstance(v, (list, tuple)) and all(
                isinstance(x, (int, float, list)) for x in v):
            keep[k] = copy.deepcopy(v)
        elif isinstance(v, np.ndarray) and v.size < 64:
            keep[k] = v.copy()
    return keep


def record(episodes, steps, seed, difficulty, policy):
    cfg = _load_rl_config("config/rl_general_v3.yaml")
    env = GeneralEnv(None, cfg, seed=seed)
    pol = load_policy_npz(policy)
    sub, pol_rows, starts = {}, {}, []

    drive_pre = env._drive.pre_step
    def drive_wrap(data):
        _snap(env, "pre", sub)
        drive_pre(data)
    env._drive.pre_step = drive_wrap
    gb = env._gearbox[-1]
    gb_pre = gb.pre_step
    def gb_wrap(data):
        gb_pre(data)
        _snap(env, "hooked", sub)
    gb.pre_step = gb_wrap
    odo_up = env._odo.update
    def odo_wrap(data, dt):
        r = odo_up(data, dt)
        if dt > 0.0:                       # the substep call, not _obs()'s read
            _snap(env, "post", sub)
            sub.setdefault("odo_out", []).append(np.array(r, float))
            c = env._ahrs._cache
            sub.setdefault("ahrs_quat", []).append(np.array(c["quat"], float))
            sub.setdefault("ahrs_gyro", []).append(np.array(c["gyro"], float))
        return r
    env._odo.update = odo_wrap

    # Every AHRS sample: which substep it saw (it runs inside odo.update,
    # after that substep's mj_step), its 12 normal draws, and its output --
    # so the kernel can be fed numpy's own noise and checked exactly.
    ahrs = env._ahrs
    a_sample = ahrs.sample
    class _Tap:
        def __init__(self, rng):
            self.rng = rng
        def standard_normal(self, *a, **k):
            z = self.rng.standard_normal(*a, **k)
            sub.setdefault("ahrs_z", []).append(np.array(z, float))
            return z
        def __getattr__(self, name):
            return getattr(self.rng, name)
    def sample_wrap(data, dt):
        rng = ahrs.rng
        ahrs.rng = _Tap(rng)
        try:
            out = a_sample(data, dt)
        finally:
            ahrs.rng = rng
        sub.setdefault("ahrs_at", []).append(len(sub.get("post_time", [])))
        sub.setdefault("ahrs_sd", []).append(np.array(data.sensordata, float))
        sub.setdefault("ahrs_dt", []).append(float(dt))
        for k in ("quat", "gyro", "accel"):
            sub.setdefault(f"ahrs_out_{k}", []).append(np.array(out[k], float))
        return out
    ahrs.sample = sample_wrap

    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep, options={"difficulty": difficulty})
        pol.reset()
        starts.append(dict(
            substep=len(sub.get("pre_time", [])), step=len(pol_rows.get("obs", [])),
            # AHRS samples so far, the reset's own priming sample included:
            # this episode's ticks start here
            ahrs_k=len(sub.get("ahrs_at", [])),
            drive=_hook_state(env._drive), gearbox=_hook_state(gb),
            odo=_hook_state(env._odo), ahrs=_hook_state(env._ahrs),
            env=dict(v_cmd_w=env._v_cmd_w.copy(), psi_cmd=env._psi_cmd,
                     psi=env._psi, raw_prev=env._raw_prev, steer=env._steer,
                     diff=env._diff, p0=env._p0.copy(),
                     next_resample=env._next_resample),
            model=dict(body_mass=env.model.body_mass.copy(),
                       geom_friction=env.model.geom_friction.copy(),
                       actuator_gainprm=env.model.actuator_gainprm.copy(),
                       actuator_biasprm=env.model.actuator_biasprm.copy(),
                       actuator_forcerange=env.model.actuator_forcerange.copy())))
        for k in range(steps):
            a = np.clip(pol.mean(obs), -1, 1).astype(np.float32)
            pol_rows.setdefault("obs", []).append(np.array(obs, float))
            obs, r, term, trunc, info = env.step(a)
            for key, val in (("action", a), ("reward", r), ("terminated", term),
                             ("truncated", trunc), ("v_cmd_w", env._v_cmd_w),
                             ("psi_cmd", env._psi_cmd), ("psi", env._psi),
                             ("raw_prev", env._raw_prev), ("step", env._step),
                             ("next_resample", env._next_resample),
                             ("steer_target", env._steer), ("diff", env._diff),
                             ("prev_a", env._prev_a), ("track", info["track"]),
                             ("episode", ep)):
                pol_rows.setdefault(key, []).append(np.array(val, float))
            if term or trunc:
                break
    return env, sub, pol_rows, starts


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--steps", type=int, default=250)       # 5 s at 50 Hz
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--difficulty", type=float, default=1.0)
    ap.add_argument("--policy", default="moves/archive/general_rl_v3_s0.npz")
    ap.add_argument("--out", default="traces/gpu_phase2/reference.npz")
    a = ap.parse_args()
    t = time.time()
    env, sub, rows, starts = record(a.episodes, a.steps, a.seed, a.difficulty, a.policy)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        a.out, **{f"sub_{k}": np.asarray(v) for k, v in sub.items()},
        **{f"pol_{k}": np.asarray(v) for k, v in rows.items()},
        starts=np.array(starts, dtype=object), config="config/rl_general_v3.yaml",
        policy=a.policy)
    n = len(sub["pre_time"])
    print(f"{a.out}: {len(rows['obs'])} policy steps, {n} substeps, "
          f"falls {int(np.sum(rows['terminated']))}, {time.time() - t:.1f} s")
