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
from .params import params_digest, plant_digest
from .control.ball_env import BallEnv, _load_rl_config
from .control.ball_spec import ActionBounds
from .control.flick import MOVES_DIR, reserve_move_name
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
    """Periodically run a deterministic eval and keep the best-scoring snapshot.

    Selection is on a *composite* score, not mean reward and not success alone:

        score = success_rate + mean_ball_speed / speed_ref

    Reward and success can diverge (a shaping exploit pays better than the task),
    so reward is not a safe criterion. But success alone is worse: it saturates.
    Deterministic eval episodes are near-identical, so success_rate is nearly a
    single bit, and once it first reaches 1.0 a strict `>` test can never be beaten
    again — the snapshot froze at 400k steps on a 5M run while launch speed went on
    climbing 0.25 -> 0.61 m/s. The speed term breaks that tie and keeps tracking the
    thing we actually want more of.
    """

    def __init__(self, params, cfg, eval_freq, n_episodes, save_path, verbose=0):
        super().__init__(verbose)
        self.params, self.cfg = params, cfg
        self.eval_freq, self.n_episodes = eval_freq, n_episodes
        self.save_path = Path(save_path)
        self.best = -1.0
        self.best_info = {}
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
        mean_speed = speeds / self.n_episodes
        ref = max(1e-6, self.cfg["reward"].get("bonus_speed_ref", 0.6))
        score = rate + mean_speed / ref
        self.logger.record("eval/success_rate", rate)
        self.logger.record("eval/mean_ball_speed", mean_speed)
        self.logger.record("eval/score", score)
        if score > self.best:
            self.best = score
            self.best_info = {"score": score, "success_rate": rate,
                              "mean_ball_speed": mean_speed,
                              "steps": int(self.num_timesteps)}
            self.save_path.mkdir(parents=True, exist_ok=True)
            self.model.save(str(self.save_path / "best_model"))
            if vn is not None:
                vn.save(str(self.save_path / "best_vecnormalize.pkl"))
            if self.verbose:
                print(f"  new best score {score:.3f} (success {rate:.2f}, "
                      f"ball {mean_speed:.3f} m/s) @ {self.num_timesteps} steps")
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


def _finish(model, vecnorm, params, cfg, total, source=None, name="ball_rl"):
    """Export -> verify -> eval -> write the move file. Shared by a finished
    training run and by --export-from."""
    # Never clobber an existing export. See reserve_move_name.
    chosen = reserve_move_name(name)
    if chosen != name:
        print(f"moves/{name} already exists — exporting as {chosen} "
              "instead (nothing was overwritten)")
        name = chosen
    a = cfg["algo"]
    npz = MOVES_DIR / f"{name}.npz"
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
    doc = {"name": name, "type": "rl", "policy_file": f"{name}.npz",
           # The parameter set this policy was TRAINED against.
           # Replay warns on a mismatch (control/flick.py::
           # check_move_digest) — a policy is an artifact of the
           # plant it saw, and nothing else records which that was.
           "params_digest": params_digest(params),
           # What a policy is actually judged on: the BIKE, not the
           # controller settings it never reads. See params.py.
           "plant_digest": plant_digest(params),
           "max_episode_s": cfg["env"]["max_episode_s"],
           "ball_start": list(cfg["env"]["ball_start"]),
           "launch_target_deg": cfg["env"]["launch_target_deg"],
           "action_space": cfg["env"]["action_space"],
           "trained": trained}
    with open(MOVES_DIR / f"{name}.yaml", "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"wrote {MOVES_DIR / f'{name}.yaml'} and {npz}")


def _resume_vecnormalize(venv, resume: bool, ckpts: list[Path], vn_path: Path):
    """Wrap `venv` in VecNormalize, restoring stats that MATCH the checkpoint
    being resumed from.

    CheckpointCallback(save_vecnormalize=True) writes a paired
    ppo_vecnormalize_<steps>.pkl beside every checkpoint. The top-level
    vecnormalize.pkl is only written once learn() returns, so it is missing
    after a killed run and stale after an earlier, longer one -- either way
    the wrong file to pair with the newest checkpoint. Stats that disagree
    with the weights mean the policy is fed a different observation
    distribution than it trained on until the running stats recover.
    """
    src = None
    if resume:
        if ckpts:
            p = ckpts[-1].with_name(ckpts[-1].name
                                    .replace("ppo_", "ppo_vecnormalize_")
                                    .replace(".zip", ".pkl"))
            src = p if p.exists() else None
        if src is None and vn_path.exists():
            src = vn_path                  # end-of-run stats: better than none
    if src is not None:
        print(f"resumed obs/reward stats from {src.name}")
        return VecNormalize.load(str(src), venv)
    if resume:
        print("WARNING: no VecNormalize stats to resume from — trained weights "
              "will run on unnormalized observations until the running stats "
              "recover")
    return VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)


def _scan_checkpoints(params, cfg, n_episodes=6, every=1):
    """Evaluate every saved checkpoint and report success + launch speed, so a
    good mid-run policy can be found and exported after the fact. Training
    progress is not monotonic and the live selector only sees its own metric —
    this shows the whole run so you can pick by eye."""
    import pickle
    ckpt = RUN_DIR / "checkpoints"
    zips = sorted(ckpt.glob("ppo_*_steps.zip"),
                  key=lambda p: int(p.stem.split("_")[1]))[::every]
    if not zips:
        raise SystemExit(f"no checkpoints in {ckpt}")
    ecfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = BallEnv(params, ecfg)
    print(f"{'steps':>9} {'success':>8} {'ball_v':>8} {'strike_v':>9} {'stick':>6}")
    rows = []
    for z in zips:
        vn_p = z.with_name(z.name.replace("ppo_", "ppo_vecnormalize_")
                           .replace(".zip", ".pkl"))
        if not vn_p.exists():
            continue
        with open(vn_p, "rb") as f:
            vn = pickle.load(f)
        model = PPO.load(str(z), device="cpu")
        if model.observation_space.shape != env.observation_space.shape:
            steps = int(z.stem.split("_")[1])
            print(f"{steps:>9}  (stale obs spec — skipped; retrain)")
            continue
        succ = spd = stv = stick = 0.0
        for k in range(n_episodes):
            obs, _ = env.reset(seed=10_000 + k)
            done, info = False, {}
            while not done:
                act, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
                obs, _r, te, tr, info = env.step(act)
                done = te or tr
            succ += int(info.get("is_success", False))
            spd += info.get("ball_speed", 0.0)
            stv += info.get("strike_speed", 0.0)
            stick += int(info.get("hit_stick", False))
        n = n_episodes
        steps = int(z.stem.split("_")[1])
        rows.append((steps, succ / n, spd / n, stv / n, stick / n))
        print(f"{steps:>9} {succ/n:>8.2f} {spd/n:>8.3f} {stv/n:>9.3f} {stick/n:>6.2f}")
    if rows:
        best = max(rows, key=lambda r: r[2])          # by launch speed
        print(f"\nfastest launch: {best[2]:.3f} m/s at {best[0]} steps "
              f"(success {best[1]:.2f})")
        print(f"export it with:  python -m aow_sim.train_ball_rl "
              f"--export-from {best[0]}")


def _export_from(spec: str, params, cfg, name="ball_rl"):
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
    from .control.ball_spec import OBS_DIM
    got = int(model.observation_space.shape[0])
    if got != OBS_DIM:
        raise SystemExit(
            f"{src.name} was trained with obs_dim {got}; the current spec is "
            f"{OBS_DIM} — this checkpoint predates the observation-spec change "
            "and cannot replay. Retrain first.")
    steps = int(src.stem.split("_")[1])
    print(f"exporting {src.name} (+ {vn.name}) without training")
    _finish(model, vecnorm, params, cfg, steps, source=src.name, name=name)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--timesteps", type=int, default=None, help="override total")
    ap.add_argument("--export-from", default=None, metavar="STEPS|PATH",
                    help="export an existing checkpoint instead of training "
                         "(e.g. --export-from 3000000)")
    ap.add_argument("--scan-checkpoints", action="store_true",
                    help="evaluate every saved checkpoint (success + launch "
                         "speed) to pick a good one post-hoc, then exit")
    ap.add_argument("--scan-every", type=int, default=1,
                    help="with --scan-checkpoints, evaluate every Nth checkpoint")
    ap.add_argument("--export-name", default="ball_rl", metavar="NAME",
                    help="move name for --export-from (e.g. ball_rl_500k, so "
                         "the primary moves/ball_rl is left alone)")
    ap.add_argument("--trace", type=Path, default=None, metavar="DIR",
                    help="after exporting, roll the move out and write "
                         "<name>_trace.csv/.png here (see rollout_move.py)")
    args = ap.parse_args()

    params = load_params()
    cfg = _load_rl_config(args.config)
    a = cfg["algo"]
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    ckpt = RUN_DIR / "checkpoints"

    if args.scan_checkpoints:
        _scan_checkpoints(params, cfg, every=max(1, args.scan_every))
        return
    if args.export_from:
        _export_from(args.export_from, params, cfg, name=args.export_name)
        if args.trace:
            from .train_flick_rl import _trace
            _trace(args.export_name, args.trace)
        return

    venv = _make_vecenv(params, cfg, a["n_envs"], a["seed"])

    # Numeric key, as in _scan_checkpoints: a plain sorted() is lexicographic,
    # which puts ppo_900000_steps last and silently resumes millions of steps
    # back. Globbing ppo_*_steps.zip also skips any stray zip in the dir.
    last_ckpt = (sorted(ckpt.glob("ppo_*_steps.zip"),
                        key=lambda p: int(p.stem.split("_")[1]))
                 if ckpt.exists() else [])
    vn_path = RUN_DIR / "vecnormalize.pkl"       # save target, written at the end
    venv = _resume_vecnormalize(venv, args.resume, last_ckpt, vn_path)

    policy_kwargs = dict(net_arch=list(a["net_arch"]), activation_fn=torch.nn.Tanh)
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
        bi = cb[1].best_info
        print(f"exporting best snapshot: score {bi.get('score', 0):.3f} "
              f"(success {bi.get('success_rate', 0):.2f}, "
              f"ball {bi.get('mean_ball_speed', 0):.3f} m/s) "
              f"from {bi.get('steps', '?')} steps")
        # Record the snapshot's OWN step count, not the run total — the best
        # policy is usually from mid-run and mislabelling it hides that.
        _finish(PPO.load(str(best_zip), device="cpu"), best_norm, params, cfg,
                bi.get("steps", total), source="best_model.zip")
    else:
        _finish(model, venv, params, cfg, total)


if __name__ == "__main__":
    main()
