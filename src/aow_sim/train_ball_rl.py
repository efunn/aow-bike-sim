"""Train an RL policy for the ball-shot move (docs/plans/ball-shot-move.md).
Requires the `[rl]` extra (gymnasium, SB3, torch, tensorboard); the base install
needs none of these to *replay* the result. Mirrors train_flick_rl.py.

  pip install -e '.[rl]'
  python -m aow_sim.train_ball_rl                  # reads config/rl_ball.yaml
  python -m aow_sim.train_ball_rl --timesteps 50000  # short smoke run
  python -m aow_sim.train_ball_rl --resume         # continue from last checkpoint
  tensorboard --logdir runs/ball_rl                # watch learning curves

On finish it exports the deterministic policy (MLP weights + VecNormalize obs
stats) to `moves/ball_rl.npz` and writes `moves/ball_rl.yaml` (provenance +
metrics from a deterministic eval), which `DriveController.command_ball(
"ball_rl")` replays with numpy alone. It never touches bike_params.yaml.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch
import yaml
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

from .build_model import load_params
from .control.ball_env import BallEnv, _load_rl_config
from .control.ball_spec import ActionBounds
from .control.flick import MOVES_DIR
from .control.policy import save_policy_npz

RUN_DIR = Path(__file__).resolve().parents[2] / "runs" / "ball_rl"


def _make_vecenv(params, cfg, n_envs, seed):
    # Monitor wraps each env so SB3 logs rollout/ep_rew_mean, ep_len_mean, and
    # (from the env's is_success info) rollout/success_rate.
    return SubprocVecEnv([
        (lambda i=i: Monitor(BallEnv(params, cfg, seed=seed + i)))
        for i in range(n_envs)
    ])


class BestBySuccess(BaseCallback):
    """Periodically run a deterministic, no-randomization eval and keep the
    best-scoring snapshot — selecting on **success rate**, not mean reward.

    Reward and success can diverge (a shaping exploit pays better than the task),
    so the last update is not necessarily the best policy and reward is not a
    safe selection criterion. Saves best_model.zip + best_vecnormalize.pkl.
    """

    def __init__(self, params, cfg, eval_freq, n_episodes, save_path, verbose=0):
        super().__init__(verbose)
        self.params, self.cfg = params, cfg
        self.eval_freq, self.n_episodes = eval_freq, n_episodes
        self.save_path = Path(save_path)
        self.best = -1.0
        self._env = None

    def _on_step(self) -> bool:
        if self.eval_freq <= 0 or self.n_calls % self.eval_freq != 0:
            return True
        if self._env is None:      # lazily built; randomization off for a clean signal
            ecfg = {**self.cfg,
                    "randomization": {**self.cfg["randomization"], "enabled": False}}
            self._env = BallEnv(self.params, ecfg)
        vn = self.model.get_vec_normalize_env()
        succ = speeds = 0
        for k in range(self.n_episodes):
            obs, _ = self._env.reset(seed=10_000 + k)
            done, info = False, {}
            while not done:
                o = vn.normalize_obs(obs) if vn is not None else obs
                act, _ = self.model.predict(o, deterministic=True)
                obs, _r, term, trunc, info = self._env.step(act)
                done = term or trunc
            succ += int(info.get("is_success", False))
            speeds += float(info.get("ball_speed", 0.0))
        rate = succ / self.n_episodes
        self.logger.record("eval/success_rate", rate)
        self.logger.record("eval/mean_ball_speed", speeds / self.n_episodes)
        if rate > self.best:
            self.best = rate
            self.save_path.mkdir(parents=True, exist_ok=True)
            self.model.save(str(self.save_path / "best_model"))
            if vn is not None:
                vn.save(str(self.save_path / "best_vecnormalize.pkl"))
            if self.verbose:
                print(f"  new best eval success {rate:.2f} @ {self.num_timesteps} steps")
        return True


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
    """The numpy export must match the trained net. Check a batch of random
    observations agree before trusting the artifact."""
    from .control.policy import load_policy_npz
    pol = load_policy_npz(npz_path)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(64):
        obs = rng.standard_normal(pol.obs_mean.shape[0]).astype(np.float32)
        norm = vecnorm.normalize_obs(obs)
        with torch.no_grad():
            sb3_mean = model.policy.predict(norm, deterministic=True)[0]
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
    ecfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = BallEnv(params, ecfg)
    speeds, aligns, succ, stick, wheel = [], [], 0, 0, 0
    for k in range(n):
        obs, _ = env.reset(seed=1000 + k)
        done = False
        info = {}
        while not done:
            a = pol.action(obs)
            na = np.array([a[0] / pol.bounds.steer_rate_max,
                           a[1] / pol.bounds.hub_max,
                           a[2] / pol.bounds.diff_max])[:env.action_space.shape[0]]
            obs, r, term, trunc, info = env.step(na)
            done = term or trunc
        speeds.append(info["ball_speed"])
        aligns.append(info["launch_deg"])
        succ += int(info["success"])
        stick += int(info["hit_stick"])
        wheel += int(info["hit_wheel"])
    return {"success_rate": succ / n,
            "mean_launch_speed": round(float(np.mean(speeds)), 3),
            "mean_launch_deg": round(float(np.mean(aligns)), 1),
            "stick_hit_rate": stick / n, "wheel_hit_rate": wheel / n,
            "n_eval": n}


def _finish(model, vecnorm, params, cfg, total, source=None):
    """Export -> verify -> eval -> write the move file. Shared by a finished
    training run and by --export-from."""
    a = cfg["algo"]
    npz = MOVES_DIR / "ball_rl.npz"
    _export(model, vecnorm, cfg, npz)
    err = _verify_export(model, vecnorm, npz)
    print(f"numpy-export vs trained net: max action diff {err:.2e} "
          f"({'OK' if err < 1e-4 else 'WARNING — export mismatch'})")
    metrics = _eval(params, cfg, npz)
    print("deterministic eval:", metrics)

    trained = {"algo": a["algorithm"], "timesteps": int(total),
               "net_arch": list(a["net_arch"]),
               "export_max_diff": float(err), "metrics": metrics}
    if source:
        trained["exported_from"] = source
    doc = {"name": "ball_rl", "type": "rl", "policy_file": "ball_rl.npz",
           "max_episode_s": cfg["env"]["max_episode_s"],
           "ball_start": list(cfg["env"]["ball_start"]),
           "launch_target_deg": cfg["env"]["launch_target_deg"],
           "action_space": cfg["env"]["action_space"],
           "trained": trained}
    with open(MOVES_DIR / "ball_rl.yaml", "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"wrote {MOVES_DIR / 'ball_rl.yaml'} and {npz}")


def _export_from(spec: str, params, cfg):
    """Export a saved checkpoint instead of training — e.g. to recover the best
    policy when training later regressed. `spec` is a step count or a .zip path;
    the matching ppo_vecnormalize_*.pkl is loaded alongside it."""
    import pickle
    src = Path(spec)
    if not src.exists():
        src = RUN_DIR / "checkpoints" / f"ppo_{int(spec)}_steps.zip"
    if not src.exists():
        raise SystemExit(f"no such checkpoint: {src}")
    vn = src.with_name(src.name.replace("ppo_", "ppo_vecnormalize_")
                       .replace(".zip", ".pkl"))
    if not vn.exists():
        raise SystemExit(f"no VecNormalize stats beside the checkpoint: {vn}")
    with open(vn, "rb") as f:
        vecnorm = pickle.load(f)          # obs_rms/clip_obs only; no venv needed
    model = PPO.load(str(src), device="cpu")
    steps = int(src.stem.split("_")[1])
    print(f"exporting {src.name} (+ {vn.name}) without training")
    _finish(model, vecnorm, params, cfg, steps, source=src.name)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--timesteps", type=int, default=None, help="override total")
    ap.add_argument("--export-from", default=None, metavar="STEPS|PATH",
                    help="export an existing checkpoint instead of training "
                         "(e.g. --export-from 3000000)")
    args = ap.parse_args()

    params = load_params()
    cfg = _load_rl_config(args.config)
    a = cfg["algo"]
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    ckpt = RUN_DIR / "checkpoints"

    if args.export_from:
        _export_from(args.export_from, params, cfg)
        return

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

    cb = [CheckpointCallback(save_freq=max(1, 100_000 // a["n_envs"]),
                             save_path=str(ckpt), name_prefix="ppo",
                             save_vecnormalize=True),
          BestBySuccess(params, cfg,
                        eval_freq=max(1, a.get("eval_every", 100_000) // a["n_envs"]),
                        n_episodes=a.get("eval_episodes", 10),
                        save_path=RUN_DIR, verbose=1)]
    total = args.timesteps or a["total_timesteps"]
    model.learn(total_timesteps=total, callback=cb, reset_num_timesteps=not args.resume,
                progress_bar=True)
    venv.save(str(vn_path))

    # Export the best-by-success snapshot, not whatever the last update produced.
    best_zip = RUN_DIR / "best_model.zip"
    best_vn = RUN_DIR / "best_vecnormalize.pkl"
    if best_zip.exists() and best_vn.exists():
        import pickle
        with open(best_vn, "rb") as f:
            best_norm = pickle.load(f)
        print(f"exporting best-by-success snapshot ({cb[1].best:.2f}) "
              f"rather than the final policy")
        _finish(PPO.load(str(best_zip), device="cpu"), best_norm, params, cfg,
                total, source="best_model.zip")
    else:
        _finish(model, venv, params, cfg, total)


if __name__ == "__main__":
    main()
