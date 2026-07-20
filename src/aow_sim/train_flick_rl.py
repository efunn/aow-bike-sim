"""Train an RL policy for the two-arc 180 flick — the closed-loop alternative to
the scipy trajectory optimization. Requires the `[rl]` extra (gymnasium, SB3,
torch, tensorboard); the base install needs none of these to *replay* the result.

  pip install -e '.[rl]'
  python -m aow_sim.train_flick_rl                 # reads config/rl_flick.yaml
  python -m aow_sim.train_flick_rl --resume        # continue from last checkpoint
  tensorboard --logdir runs/flick_rl               # watch learning curves

On finish it exports the deterministic policy (MLP weights + VecNormalize obs
stats) to `moves/flick_rl.npz` and writes `moves/flick_rl.yaml` (provenance +
metrics from a deterministic eval), which `DriveController.command_flick(
"flick_rl")` replays with numpy alone. It never touches bike_params.yaml or the
scipy path. Training is long — run it on whatever machine you like; the artifact
is portable.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch
import yaml
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

from .build_model import load_params
from .control.flick import MOVES_DIR
from .control.flick_env import FlickEnv, _load_rl_config
from .control.flick_spec import ActionBounds
from .control.policy import MLPPolicy, save_policy_npz

RUN_DIR = Path(__file__).resolve().parents[2] / "runs" / "flick_rl"


def _make_vecenv(params, cfg, n_envs, seed):
    # Monitor wraps each env so SB3 can log rollout/ep_rew_mean, ep_len_mean,
    # and (from the env's is_success info) rollout/success_rate.
    return SubprocVecEnv([
        (lambda i=i: Monitor(FlickEnv(params, cfg, seed=seed + i)))
        for i in range(n_envs)
    ])


def _export(model, vecnorm, cfg, path_npz: Path):
    """Pull the deterministic policy MLP + VecNormalize obs stats out of SB3 and
    save as a numpy .npz (see control/policy.py for the replay side)."""
    policy = model.policy
    layers = []
    for m in policy.mlp_extractor.policy_net:          # Linear/Tanh sequence
        if isinstance(m, torch.nn.Linear):
            layers.append((m.weight.detach().cpu().numpy(),
                           m.bias.detach().cpu().numpy()))
    an = policy.action_net                             # final mean layer
    layers.append((an.weight.detach().cpu().numpy(), an.bias.detach().cpu().numpy()))
    obs_mean = vecnorm.obs_rms.mean.astype(np.float32)
    obs_var = vecnorm.obs_rms.var.astype(np.float32)
    bounds = ActionBounds(**cfg["env"]["action_bounds"])
    save_policy_npz(path_npz, layers, "tanh", obs_mean, obs_var, bounds,
                    obs_clip=float(vecnorm.clip_obs))
    return layers, obs_mean, obs_var, bounds


def _verify_export(model, vecnorm, npz_path):
    """The whole point of the numpy export is that it matches the trained net.
    Check a handful of observations agree before trusting the artifact."""
    from .control.policy import load_policy_npz
    pol = load_policy_npz(npz_path)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(64):
        obs = rng.standard_normal(pol.obs_mean.shape[0]).astype(np.float32)
        norm = vecnorm.normalize_obs(obs)
        with torch.no_grad():
            t = torch.as_tensor(norm).float().unsqueeze(0)
            sb3_mean = model.policy.predict(norm, deterministic=True)[0]
        # numpy path returns scaled action; unscale to compare the raw mean
        sr, hub, diff = pol.action(obs)
        raw = np.array([sr / pol.bounds.steer_rate_max, hub / pol.bounds.hub_max,
                        diff / pol.bounds.diff_max])[:len(sb3_mean)]
        worst = max(worst, float(np.max(np.abs(raw - sb3_mean))))
    return worst


def _eval(params, cfg, npz_path, n=8):
    """Deterministic eval of the exported numpy policy in a no-randomization env
    -> metrics for the move file."""
    from .control.policy import load_policy_npz
    pol = load_policy_npz(npz_path)
    pol.target = np.deg2rad(cfg["env"]["yaw_target_deg"])
    pol.horizon = cfg["env"]["max_episode_s"]
    ecfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = FlickEnv(params, ecfg)
    rolls, yaws, lats, succ = [], [], [], 0
    for k in range(n):
        obs, _ = env.reset(seed=1000 + k)
        done = False
        mr = 0.0
        while not done:
            a = pol.action(obs)
            # env expects normalized action; invert scale
            na = np.array([a[0] / pol.bounds.steer_rate_max,
                           a[1] / pol.bounds.hub_max,
                           a[2] / pol.bounds.diff_max])[:env.action_space.shape[0]]
            obs, r, term, trunc, info = env.step(na)
            mr = max(mr, abs(np.degrees(np.arcsin(np.clip(obs[2], -1, 1)))))
            done = term or trunc
        rolls.append(mr)
        yaws.append(info["yaw_err_deg"])
        lats.append(abs(info["e_lat"]))
        succ += int(info["success"])
    L = params["bike"]["wheelbase"]
    return {"success_rate": succ / n,
            "final_yaw_err_deg": round(float(np.mean(np.abs(yaws))), 1),
            "lateral_env_L": round(float(np.max(lats)) / L, 3),
            "n_eval": n}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--timesteps", type=int, default=None, help="override total")
    args = ap.parse_args()

    params = load_params()
    cfg = _load_rl_config(args.config)
    a = cfg["algo"]
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    ckpt = RUN_DIR / "checkpoints"

    venv = _make_vecenv(params, cfg, a["n_envs"], a["seed"])
    vn_path = RUN_DIR / "vecnormalize.pkl"
    if args.resume and vn_path.exists():
        venv = VecNormalize.load(str(vn_path), venv)
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)

    policy_kwargs = dict(net_arch=list(a["net_arch"]), activation_fn=torch.nn.Tanh)
    last_ckpt = sorted(ckpt.glob("*.zip")) if ckpt.exists() else []
    if args.resume and last_ckpt:
        model = PPO.load(str(last_ckpt[-1]), env=venv)
        print(f"resumed from {last_ckpt[-1].name}")
    else:
        model = PPO("MlpPolicy", venv, learning_rate=a["learning_rate"],
                    n_steps=a["n_steps"], batch_size=a["batch_size"],
                    n_epochs=a["n_epochs"], gamma=a["gamma"],
                    gae_lambda=a["gae_lambda"], clip_range=a["clip_range"],
                    ent_coef=a["ent_coef"], policy_kwargs=policy_kwargs,
                    seed=a["seed"], tensorboard_log=str(RUN_DIR), verbose=1)

    cb = CheckpointCallback(save_freq=max(1, 100_000 // a["n_envs"]),
                            save_path=str(ckpt), name_prefix="ppo",
                            save_vecnormalize=True)
    total = args.timesteps or a["total_timesteps"]
    model.learn(total_timesteps=total, callback=cb, reset_num_timesteps=not args.resume,
                progress_bar=True)
    venv.save(str(vn_path))

    npz = MOVES_DIR / "flick_rl.npz"
    _export(model, venv, cfg, npz)
    err = _verify_export(model, venv, npz)
    print(f"numpy-export vs trained net: max action diff {err:.2e} "
          f"({'OK' if err < 1e-4 else 'WARNING — export mismatch'})")
    metrics = _eval(params, cfg, npz)
    print("deterministic eval:", metrics)

    doc = {"name": "flick_rl", "type": "rl", "policy_file": "flick_rl.npz",
           "yaw_target_deg": cfg["env"]["yaw_target_deg"],
           "max_episode_s": cfg["env"]["max_episode_s"],
           "action_space": cfg["env"]["action_space"],
           "trained": {"algo": a["algorithm"], "timesteps": int(total),
                       "net_arch": list(a["net_arch"]),
                       "export_max_diff": float(err), "metrics": metrics}}
    with open(MOVES_DIR / "flick_rl.yaml", "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"wrote {MOVES_DIR / 'flick_rl.yaml'} and {npz}")


if __name__ == "__main__":
    main()
