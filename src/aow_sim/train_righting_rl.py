"""Train the RL get-up on the swing module: control/righting_env.py, the
32-env CPU (SB3) version. Requires the `[rl]` extra.

  python -m aow_sim.train_righting_rl                    # config/rl/rl_righting.yaml
  python -m aow_sim.train_righting_rl --resume
  python -m aow_sim.train_righting_rl --export-from 1000000 --export-name righting_rl_1m
  python -m aow_sim.train_righting_rl --eval moves/righting_rl.npz

On finish it exports the deterministic policy to moves/<name>.{npz,yaml}
(control/policy.py's npz, the bounds field holding RightingBounds), which
righting_spec.RightingPolicy replays with numpy alone. Nothing flies it yet:
not teleop, not run_bike.
"""

from __future__ import annotations

import argparse
import copy
import pickle
from pathlib import Path

import numpy as np
import torch
import yaml
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv

from .build_model import load_params
from .control.flick import MOVES_DIR, reserve_move_name
from .control.policy import save_policy_npz
from .control.righting_env import RightingEnv, _load_rl_config, scripted_ramp
from .control.righting_spec import OBS_DIM, RightingBounds, RightingPolicy
from .params import params_digest, plant_digest
from .train_flick_rl import _resume_vecnormalize

RUN_DIR = Path(__file__).resolve().parents[2] / "runs" / "righting_rl"
#: Servo strengths the eval grid flies, both sides each: the scripted
#: 120 deg/s full-stroke ramp goes over at 1.0 and 0.85, props at 0.7 and
#: stalls at 0.6 (this env, 2026-11-03).
EVAL_STRENGTHS = (1.0, 0.85, 0.7, 0.6)
_LETTER = {"success": "S", "over": "O", "overload": "H", "timeout": "T"}


def _make_vecenv(params, cfg, n_envs, seed):
    return SubprocVecEnv([
        (lambda i=i: Monitor(RightingEnv(params, cfg, seed=seed + i)))
        for i in range(n_envs)])


def _export(model, vecnorm, cfg, path_npz: Path):
    policy = model.policy
    layers = [(m.weight.detach().cpu().numpy(), m.bias.detach().cpu().numpy())
              for m in policy.mlp_extractor.policy_net
              if isinstance(m, torch.nn.Linear)]
    an = policy.action_net
    layers.append((an.weight.detach().cpu().numpy(), an.bias.detach().cpu().numpy()))
    save_policy_npz(path_npz, layers, "tanh", vecnorm.obs_rms.mean,
                    vecnorm.obs_rms.var, RightingBounds(**cfg["env"]["action_bounds"]),
                    obs_clip=float(vecnorm.clip_obs))


def _verify_export(model, vecnorm, npz):
    """Max |raw action| difference between the numpy replay and SB3."""
    pol = RightingPolicy(npz)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(64):
        obs = rng.standard_normal(OBS_DIM).astype(np.float32)
        sb3 = model.policy.predict(vecnorm.normalize_obs(obs), deterministic=True)[0]
        worst = max(worst, float(np.max(np.abs(
            np.clip(pol.net.mean(obs), -1, 1) - sb3))))
    return worst


def evaluate(params, cfg, act, seeds=3):
    """`act(obs, side)` -> normalized action, flown on the EVAL_STRENGTHS grid
    (supply 1, masses and friction nominal), both sides, from rest and from
    mid-fall. -> per-(strength, start) outcome strings and the totals."""
    rows, tot = {}, {"success": 0, "over": 0, "overload": 0, "timeout": 0}
    for s in EVAL_STRENGTHS:
        c = copy.deepcopy(cfg)
        r = c["randomization"]
        r.update(servo_strength=[s, s], supply=[1.0, 1.0], mass_frac=0.0,
                 friction_frac=0.0)
        env = RightingEnv(params, c, seed=7)
        for start in ("rest", "falling"):
          out = []
          for side in (1, -1):
            for k in range(seeds):
                obs, _ = env.reset(seed=100 + k,
                                   options={"side": side, "start": start})
                done, t = False, 0
                while not done:
                    obs, _r, te, tr, info = (
                        env.step(None, command=act(obs, side))
                        if getattr(act, "raw", False) else env.step(act(obs, side)))
                    done, t = te or tr, t + 1
                key = ("success" if info["success"] else "over" if info["over"]
                       else "overload" if info["overload"] else "timeout")
                tot[key] += 1
                out.append(f"{_LETTER[key]}{t * env.ctrl_dt:.1f}")
          rows[(s, start)] = out
    return rows, tot


def _print_eval(rows, tot):
    for (s, start), out in rows.items():
        print(f"  strength {s:.2f} {start:>7}: {' '.join(out)}")
    n = sum(tot.values())
    print(f"  {tot['success']}/{n} propped, {tot['over']} over, "
          f"{tot['overload']} overload, {tot['timeout']} timeout "
          "(S/O/H/T + seconds; R x3 then L x3)")


def _npz_act(npz):
    pol = RightingPolicy(npz)
    return lambda obs, side: np.clip(pol.net.mean(obs), -1.0, 1.0)


def _finish(model, vecnorm, params, cfg, total, name, source=None):
    chosen = reserve_move_name(name)
    if chosen != name:
        print(f"moves/{name} already exists -- exporting as {chosen} instead")
        name = chosen
    npz = MOVES_DIR / f"{name}.npz"
    _export(model, vecnorm, cfg, npz)
    err = _verify_export(model, vecnorm, npz)
    print(f"numpy export vs trained net: max action diff {err:.2e} "
          f"({'OK' if err < 1e-4 else 'WARNING -- export mismatch'})")
    rows, tot = evaluate(params, cfg, _npz_act(npz))
    _print_eval(rows, tot)
    a = cfg["algo"]
    doc = {"name": name, "type": "rl", "move": "righting",
           "policy_file": f"{name}.npz",
           "params_digest": params_digest(params),
           "plant_digest": plant_digest(params),
           "env": cfg["env"], "randomization": cfg["randomization"],
           "trained": {"algo": a["algorithm"], "timesteps": int(total),
                       "net_arch": list(a["net_arch"]), "seed": a["seed"],
                       "export_max_diff": err,
                       "eval": {"strengths": list(EVAL_STRENGTHS), **tot}}}
    if source:
        doc["trained"]["exported_from"] = source
    with open(MOVES_DIR / f"{name}.yaml", "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"wrote {MOVES_DIR / f'{name}.yaml'} and {npz}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--timesteps", type=int, default=None, help="override total")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--run-dir", type=Path, default=None)
    ap.add_argument("--export-name", default="righting_rl", metavar="NAME")
    ap.add_argument("--export-from", default=None, metavar="STEPS|PATH",
                    help="export a saved checkpoint instead of training")
    ap.add_argument("--eval", default=None, metavar="NPZ|scripted",
                    help="fly an exported policy (or the scripted 120 deg/s "
                         "ramp) on the eval grid, then exit")
    args = ap.parse_args()

    params = load_params()
    cfg = _load_rl_config(args.config)
    a = cfg["algo"]
    if args.seed is not None:
        a["seed"] = args.seed
    run_dir = args.run_dir or RUN_DIR
    ckpt = run_dir / "checkpoints"

    if args.eval:
        if args.eval == "scripted":
            act = scripted_ramp(cfg)
        else:
            act = _npz_act(args.eval)
        _print_eval(*evaluate(params, cfg, act))
        return
    if args.export_from:
        src = Path(args.export_from)
        if not src.exists():
            src = ckpt / f"ppo_{int(args.export_from)}_steps.zip"
        vn = src.with_name(src.name.replace("ppo_", "ppo_vecnormalize_")
                           .replace(".zip", ".pkl"))
        with open(vn, "rb") as f:
            vecnorm = pickle.load(f)
        model = PPO.load(str(src), device="cpu")
        _finish(model, vecnorm, params, cfg, int(src.stem.split("_")[1]),
                args.export_name, source=src.name)
        return

    run_dir.mkdir(parents=True, exist_ok=True)
    with open(run_dir / "config.yaml", "w") as f:       # the config actually used
        yaml.safe_dump(cfg, f, sort_keys=False)
    venv = _make_vecenv(params, cfg, a["n_envs"], a["seed"])
    last = (sorted(ckpt.glob("ppo_*_steps.zip"), key=lambda p: int(p.stem.split("_")[1]))
            if ckpt.exists() else [])
    vn_path = run_dir / "vecnormalize.pkl"
    venv = _resume_vecnormalize(venv, args.resume, last, vn_path)
    if args.resume and last:
        model = PPO.load(str(last[-1]), env=venv)
        print(f"resumed from {last[-1].name}")
    else:
        model = PPO("MlpPolicy", venv, learning_rate=a["learning_rate"],
                    n_steps=a["n_steps"], batch_size=a["batch_size"],
                    n_epochs=a["n_epochs"], gamma=a["gamma"],
                    gae_lambda=a["gae_lambda"], clip_range=a["clip_range"],
                    ent_coef=a["ent_coef"], seed=a["seed"],
                    policy_kwargs=dict(net_arch=list(a["net_arch"]),
                                       activation_fn=torch.nn.Tanh,
                                       log_std_init=a.get("log_std_init", 0.0)),
                    tensorboard_log=str(run_dir), verbose=1)
    cb = CheckpointCallback(save_freq=max(1, 100_000 // a["n_envs"]),
                            save_path=str(ckpt), name_prefix="ppo",
                            save_vecnormalize=True)
    total = args.timesteps or a["total_timesteps"]
    model.learn(total_timesteps=total, callback=cb,
                reset_num_timesteps=not args.resume, progress_bar=True)
    venv.save(str(vn_path))
    _finish(model, venv, params, cfg, total, args.export_name)


if __name__ == "__main__":
    main()
