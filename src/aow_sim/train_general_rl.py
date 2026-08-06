"""Train the GENERAL command-conditioned policy: one always-on controller
tracking a live (velocity vector, heading) command (see general_env.py).
Requires the `[rl]` extra; the base install replays the result with numpy.

  pip install -e '.[rl]'
  python -m aow_sim.train_general_rl               # reads config/rl_general.yaml
  python -m aow_sim.train_general_rl --resume      # continue from last checkpoint
  tensorboard --logdir runs/general_rl             # watch learning curves

Mid-run policies: CheckpointCallback snapshots every ~100k steps into
runs/general_rl/checkpoints/. To find and inspect an interesting one:

  python -m aow_sim.train_general_rl --scan-checkpoints
  python -m aow_sim.train_general_rl --export-from 2000000 \\
      --export-name general_rl_2m

Unlike the move trainers there is no "success" to select on — an always-on
controller either survives while tracking or it doesn't — so the snapshot
score is survival x tracking quality (see `_score`).
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
from .control.flick import MOVES_DIR
from .control.general_env import GeneralEnv, _load_rl_config
from .control.general_spec import ActionBounds
from .control.policy import save_policy_npz

RUN_DIR = Path(__file__).resolve().parents[2] / "runs" / "general_rl"

# Deterministic command grid for every eval path, as (v_lon, v_lat, heading
# step [deg]) with the velocity fractions relative to env.v_max. Mirrors the
# analytic controller's own benchmarks (straight sprint, reverse, 90/180 deg
# turns at speed) so the two are directly comparable — see the Stage-2 gate.
_EVAL_CMDS = [
    (0.0, 0.0, 0.0),      # hold station
    (0.0, 0.0, 90.0),     # standstill quarter turn
    (0.0, 0.0, 180.0),    # standstill about-face
    (0.67, 0.0, 0.0),     # straight sprint (~0.8 m/s at v_max 1.2)
    (0.67, 0.0, 90.0),    # turn at speed
    (0.67, 0.0, 180.0),   # U-turn at speed
    (1.0, 0.0, 0.0),      # top speed straight
    (-0.42, 0.0, 0.0),    # reverse (~-0.5 m/s)
    (-0.42, 0.0, 90.0),   # reverse turn
    (0.0, 0.33, 0.0),     # pure lateral crab (rear omni)
    (0.5, 0.0, 45.0),
    (0.5, 0.33, 180.0),   # glide + about-face = the pivot move
]


def eval_cmds(v_max: float) -> list[tuple[float, float, float]]:
    """Absolute (v_lon, v_lat, heading-step rad) commands for the eval grid,
    scaled to the configured envelope."""
    return [(round(a * v_max, 3), round(b * v_max, 3), np.deg2rad(d))
            for a, b, d in _EVAL_CMDS]


def _score(m: dict) -> float:
    """Snapshot-selection score (higher is better): survival x tracking.

    An always-on controller has no task success, so `survive_rate` is the
    fraction of eval episodes that did not fall, and the tracking term is
    the env's own bounded [0,1] reward average. Multiplying rather than
    adding is deliberate: a policy that tracks beautifully for 2 s and then
    falls must not outrank one that survives the whole episode."""
    return m["survive_rate"] * m["track"]


def _make_vecenv(params, cfg, n_envs, seed):
    return SubprocVecEnv([
        (lambda i=i: Monitor(GeneralEnv(params, cfg, seed=seed + i)))
        for i in range(n_envs)
    ])


def _eval_episodes(env, act_fn, cmds):
    """One episode per command point; aggregate metrics. The eval env has
    randomization disabled, so repeating a command adds nothing."""
    n = len(cmds)
    survived, tracks, verrs, herrs = 0, [], [], []
    for k, (v_lon, v_lat, dpsi) in enumerate(cmds):
        obs, _ = env.reset(seed=10_000 + k, options={
            "v_cmd": (v_lon, v_lat), "psi_cmd_rel": dpsi, "difficulty": 1.0})
        done, info = False, {}
        while not done:
            obs, _r, term, trunc, info = env.step(act_fn(obs))
            done = term or trunc
        survived += int(not info.get("fell", True))
        tracks.append(info.get("track", 0.0))
        verrs.append(info.get("vel_err", 9.9))
        herrs.append(info.get("head_err_deg", 180.0))
    return {"survive_rate": survived / n,
            "track": round(float(np.mean(tracks)), 3),
            "vel_err": round(float(np.mean(verrs)), 3),
            "head_err_deg": round(float(np.mean(herrs)), 1),
            "n_eval": n}


class BestByScore(BaseCallback):
    """Keep the best-scoring snapshot from a periodic deterministic eval over
    the command grid. See `_score` for the criterion."""

    def __init__(self, params, cfg, eval_freq, save_path, verbose=0):
        super().__init__(verbose)
        self.params, self.cfg = params, cfg
        self.eval_freq = eval_freq
        self.cmds = eval_cmds(cfg["env"]["v_max"])
        self.save_path = Path(save_path)
        self.best = -1.0
        self.best_info = {}
        self._env = None

    def _on_step(self) -> bool:
        if self.eval_freq <= 0 or self.n_calls % self.eval_freq != 0:
            return True
        if self._env is None:      # lazily built; randomization off
            ecfg = {**self.cfg,
                    "randomization": {**self.cfg["randomization"], "enabled": False}}
            self._env = GeneralEnv(self.params, ecfg)
        vn = self.model.get_vec_normalize_env()

        def act(obs):
            o = vn.normalize_obs(obs) if vn is not None else obs
            return self.model.predict(o, deterministic=True)[0]

        m = _eval_episodes(self._env, act, self.cmds)
        score = _score(m)
        self.logger.record("eval/survive_rate", m["survive_rate"])
        self.logger.record("eval/track", m["track"])
        self.logger.record("eval/vel_err", m["vel_err"])
        self.logger.record("eval/head_err_deg", m["head_err_deg"])
        self.logger.record("eval/score", score)
        if score > self.best:
            self.best = score
            self.best_info = {"score": score, **m,
                              "steps": int(self.num_timesteps)}
            self.save_path.mkdir(parents=True, exist_ok=True)
            self.model.save(str(self.save_path / "best_model"))
            if vn is not None:
                vn.save(str(self.save_path / "best_vecnormalize.pkl"))
            if self.verbose:
                print(f"  new best score {score:.3f} (survive "
                      f"{m['survive_rate']:.2f}, track {m['track']:.2f}, "
                      f"head err {m['head_err_deg']:.0f} deg) @ "
                      f"{self.num_timesteps} steps")
        return True


class DifficultyLog(BaseCallback):
    """Log the curriculum level, so a dip in ep_len_mean can be read as either
    'the policy got worse' or 'the task got harder' -- opposite conclusions
    from the same curve.

    Each SubprocVecEnv worker owns its own `_diff` counter and advances it on
    its own episode outcomes (GeneralEnv._advance_curriculum), so there is no
    single global level; min/max show how far the 32 envs have drifted apart.
    Difficulty moves by at most 0.02 per episode end, so recording the value
    seen at dump time rather than a running mean loses nothing.
    """

    def _on_step(self) -> bool:
        d = [i["difficulty"] for i in self.locals.get("infos", ())
             if "difficulty" in i]
        if d:
            self.logger.record("curriculum/difficulty", float(np.mean(d)))
            self.logger.record("curriculum/difficulty_min", float(np.min(d)))
            self.logger.record("curriculum/difficulty_max", float(np.max(d)))
        return True


def _export(model, vecnorm, cfg, path_npz: Path):
    """Pull the deterministic policy MLP + VecNormalize obs stats out of SB3
    and save as a numpy .npz (see control/policy.py for the replay side)."""
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


def _eval(params, cfg, npz_path):
    """Deterministic eval of the exported numpy policy over the command grid
    -> metrics for the move file."""
    from .control.policy import load_policy_npz
    pol = load_policy_npz(npz_path)
    ecfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = GeneralEnv(params, ecfg)

    def act(obs):
        a = pol.action(obs)
        return np.array([a[0] / pol.bounds.steer_rate_max,
                         a[1] / pol.bounds.hub_max,
                         a[2] / pol.bounds.diff_max])[:env.action_space.shape[0]]

    return _eval_episodes(env, act, eval_cmds(cfg["env"]["v_max"]))


def _finish(model, vecnorm, params, cfg, total, source=None, name="general_rl"):
    """Export -> verify -> eval -> write the move file."""
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
    doc = {"name": name, "type": "rl", "kind": "general",
           "policy_file": f"{name}.npz",
           # control_rate_hz is part of the contract: replay must query the
           # policy at the rate it was trained at, not the controller rate.
           "control_rate_hz": cfg["env"]["control_rate_hz"],
           "v_max": cfg["env"]["v_max"],
           "action_space": cfg["env"]["action_space"],
           "trained": trained}
    with open(MOVES_DIR / f"{name}.yaml", "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"wrote {MOVES_DIR / f'{name}.yaml'} and {npz}")


def _scan_checkpoints(params, cfg, every=1):
    """Evaluate every saved checkpoint over the command grid, so an
    interesting mid-run policy can be found and exported after the fact."""
    import pickle
    cmds = eval_cmds(cfg["env"]["v_max"])
    ckpt = RUN_DIR / "checkpoints"
    zips = sorted(ckpt.glob("ppo_*_steps.zip"),
                  key=lambda p: int(p.stem.split("_")[1]))[::every]
    if not zips:
        raise SystemExit(f"no checkpoints in {ckpt}")
    ecfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = GeneralEnv(params, ecfg)
    print(f"{'steps':>9} {'survive':>8} {'track':>7} {'v_err':>7} "
          f"{'head_err':>9} {'score':>6}")
    rows = []
    for z in zips:
        vn_p = z.with_name(z.name.replace("ppo_", "ppo_vecnormalize_")
                           .replace(".zip", ".pkl"))
        if not vn_p.exists():
            continue
        with open(vn_p, "rb") as f:
            vn = pickle.load(f)
        model = PPO.load(str(z), device="cpu")
        steps = int(z.stem.split("_")[1])
        if model.observation_space.shape != env.observation_space.shape:
            print(f"{steps:>9}  (stale obs spec — skipped; retrain)")
            continue

        def act(obs):
            return model.predict(vn.normalize_obs(obs), deterministic=True)[0]

        m = _eval_episodes(env, act, cmds)
        score = _score(m)
        rows.append((steps, score, m))
        print(f"{steps:>9} {m['survive_rate']:>8.2f} {m['track']:>7.3f} "
              f"{m['vel_err']:>7.3f} {m['head_err_deg']:>9.1f} {score:>6.3f}")
    if rows:
        best = max(rows, key=lambda r: r[1])
        print(f"\nbest score {best[1]:.3f} at {best[0]} steps "
              f"(survive {best[2]['survive_rate']:.2f}, "
              f"track {best[2]['track']:.2f})")
        print(f"export it with:  python -m aow_sim.train_general_rl "
              f"--export-from {best[0]}")


def _export_from(spec: str, params, cfg, name="general_rl"):
    """Export a saved checkpoint instead of training. `spec` is a step count
    or a .zip path; the matching ppo_vecnormalize_*.pkl is loaded alongside."""
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
    from .control.general_spec import OBS_DIM
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
                    help="export an existing checkpoint instead of training")
    ap.add_argument("--export-name", default="general_rl", metavar="NAME",
                    help="move name for --export-from (e.g. general_rl_2m)")
    ap.add_argument("--scan-checkpoints", action="store_true",
                    help="evaluate every saved checkpoint, then exit")
    ap.add_argument("--scan-every", type=int, default=1,
                    help="with --scan-checkpoints, evaluate every Nth checkpoint")
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
        return

    venv = _make_vecenv(params, cfg, a["n_envs"], a["seed"])
    vn_path = RUN_DIR / "vecnormalize.pkl"
    if args.resume and vn_path.exists():
        venv = VecNormalize.load(str(vn_path), venv)
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)

    policy_kwargs = dict(net_arch=list(a["net_arch"]), activation_fn=torch.nn.Tanh)
    # Numeric key, as in _scan_checkpoints: a plain sorted() is lexicographic,
    # which puts ppo_900000_steps last and silently resumes millions of steps
    # back. Globbing ppo_*_steps.zip also skips any stray zip in the dir.
    last_ckpt = (sorted(ckpt.glob("ppo_*_steps.zip"),
                        key=lambda p: int(p.stem.split("_")[1]))
                 if ckpt.exists() else [])
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
          BestByScore(params, cfg,
                      eval_freq=max(1, a.get("eval_every", 200_000) // a["n_envs"]),
                      save_path=RUN_DIR, verbose=1),
          DifficultyLog()]
    total = args.timesteps or a["total_timesteps"]
    model.learn(total_timesteps=total, callback=cb,
                reset_num_timesteps=not args.resume, progress_bar=True)
    venv.save(str(vn_path))

    best_zip = RUN_DIR / "best_model.zip"
    best_vn = RUN_DIR / "best_vecnormalize.pkl"
    bi = cb[1].best_info
    if best_zip.exists() and bi:
        import pickle
        print(f"exporting best snapshot: score {bi['score']:.3f} "
              f"(survive {bi['survive_rate']:.2f}, track {bi['track']:.2f}) "
              f"from {bi['steps']} steps")
        best = PPO.load(str(best_zip), device="cpu")
        with open(best_vn, "rb") as f:
            bvn = pickle.load(f)
        _finish(best, bvn, params, cfg, bi["steps"],
                source="best_model.zip", name="general_rl")
    else:
        _finish(model, venv, params, cfg, total, name="general_rl")


if __name__ == "__main__":
    main()
