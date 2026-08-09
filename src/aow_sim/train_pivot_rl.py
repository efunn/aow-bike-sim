"""Train the RL pivot: 180-deg chassis yaw with the front wheel holding its
global ground heading, from standstill or a glide (see pivot_env.py).
Requires the `[rl]` extra; the base install replays the result with numpy.

  pip install -e '.[rl]'
  python -m aow_sim.train_pivot_rl                 # reads config/rl_pivot.yaml
  python -m aow_sim.train_pivot_rl --resume        # continue from last checkpoint
  tensorboard --logdir runs/pivot_rl               # watch learning curves

Mid-run policies: CheckpointCallback snapshots every ~100k steps into
runs/pivot_rl/checkpoints/. To find and inspect an interesting one:

  python -m aow_sim.train_pivot_rl --scan-checkpoints
  python -m aow_sim.train_pivot_rl --export-from 500000 \\
      --export-name pivot_rl_500k --trace traces/

On finish it exports the best-by-score snapshot to `moves/pivot_rl.npz` +
`moves/pivot_rl.yaml`, replayed by `DriveController.command_pivot_rl`.
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
from .params import params_digest
from .control.flick import MOVES_DIR, reserve_move_name
from .control.pivot_env import PivotEnv, _load_rl_config
from .control.pivot_spec import ActionBounds
from .control.policy import save_policy_npz

RUN_DIR = Path(__file__).resolve().parents[2] / "runs" / "pivot_rl"

# Deterministic (v_start, v_end) eval points as FRACTIONS of env.v_max:
# stationary, spin-up, spin-down, and steady glides. Kept relative so the
# eval only ever scores what the run actually trains — see eval_grid.
_EVAL_FRACS = [(0.0, 0.0), (0.0, 0.5), (0.0, 1.0), (0.5, 0.0), (1.0, 0.0),
               (0.5, 0.5), (1.0, 1.0), (1.0, 0.5), (0.5, 1.0), (0.75, 0.25)]
HOLD_REF_DEG = 20.0   # hold-RMS of a sloppy-but-passing policy (score ref)


def eval_grid(v_max: float) -> list[tuple[float, float]]:
    """Deterministic (v_start, v_end) points for every eval path
    (BestByScore, _eval, _scan_checkpoints), scaled to the configured
    envelope and deduplicated.

    Relative, not absolute: with `v_max: 0.0` (stationary-only training) an
    absolute grid would score the policy at speeds it never sees, capping
    success_rate at 1/len(grid) and handing snapshot selection to
    out-of-distribution failures. Here v_max = 0 collapses to the single
    stationary point, so the run is scored on exactly what it trains."""
    seen: dict[tuple[float, float], None] = {}
    for a, b in _EVAL_FRACS:
        seen.setdefault((round(a * v_max, 4), round(b * v_max, 4)), None)
    return list(seen)


def _score(m: dict) -> float:
    """Composite snapshot-selection score (higher is better).

        success_rate + yaw progress + wheel-hold quality

    Success alone saturates on deterministic eval (the ball_rl lesson: the
    snapshot froze at 400k of a 5M run). Hold quality is the move's
    signature, but it CANNOT stand alone as the tie-break: a policy that
    never turns holds its heading perfectly, so do-nothing scores ~0.9
    while a policy that nearly completes the turn scores ~0.5 and would
    lose. The yaw-progress term restores the ordering, and success still
    dominates both (any success beats any non-success)."""
    prog = max(0.0, 1.0 - m["final_yaw_err_deg"] / 180.0)
    hold = max(0.0, 1.0 - m["hold_rms_deg"] / HOLD_REF_DEG)
    return m["success_rate"] + prog + hold


def _make_vecenv(params, cfg, n_envs, seed):
    return SubprocVecEnv([
        (lambda i=i: Monitor(PivotEnv(params, cfg, seed=seed + i)))
        for i in range(n_envs)
    ])


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


def _eval_episodes(env, act_fn, grid):
    """Run one episode per (v_start, v_end) point; aggregate metrics.

    One episode per point, not a fixed count: the eval env has
    randomization disabled, so two episodes at the same (v_start, v_end)
    are bit-identical and would only pad the average."""
    n = len(grid)
    succ, holds, yerrs, verrs = 0, [], [], []
    for k, (vs, ve) in enumerate(grid):
        obs, _ = env.reset(seed=10_000 + k, options={"v_start": vs, "v_end": ve})
        done, info = False, {}
        while not done:
            obs, _r, term, trunc, info = env.step(act_fn(obs))
            done = term or trunc
        succ += int(info.get("is_success", False))
        holds.append(info.get("hold_rms_deg", 90.0))
        yerrs.append(abs(info.get("yaw_err_deg", 180.0)))
        verrs.append(abs(info.get("v_err_end", 1.0)))
    return {"success_rate": succ / n,
            "hold_rms_deg": round(float(np.mean(holds)), 1),
            "final_yaw_err_deg": round(float(np.mean(yerrs)), 1),
            "final_v_err": round(float(np.mean(verrs)), 3),
            "n_eval": n}


class BestByScore(BaseCallback):
    """Keep the best-scoring snapshot from a periodic deterministic eval
    over the (v_start, v_end) grid. See `_score` for the criterion and why
    neither success nor hold quality works alone."""

    def __init__(self, params, cfg, eval_freq, save_path, verbose=0):
        super().__init__(verbose)
        self.params, self.cfg = params, cfg
        self.eval_freq = eval_freq
        self.grid = eval_grid(cfg["env"]["v_max"])
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
            self._env = PivotEnv(self.params, ecfg)
        vn = self.model.get_vec_normalize_env()

        def act(obs):
            o = vn.normalize_obs(obs) if vn is not None else obs
            return self.model.predict(o, deterministic=True)[0]

        m = _eval_episodes(self._env, act, self.grid)
        score = _score(m)
        self.logger.record("eval/success_rate", m["success_rate"])
        self.logger.record("eval/hold_rms_deg", m["hold_rms_deg"])
        self.logger.record("eval/yaw_err_deg", m["final_yaw_err_deg"])
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
                print(f"  new best score {score:.3f} (success "
                      f"{m['success_rate']:.2f}, yaw err "
                      f"{m['final_yaw_err_deg']:.0f} deg, hold "
                      f"{m['hold_rms_deg']:.1f} deg) @ {self.num_timesteps} steps")
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
    """Deterministic eval of the exported numpy policy over the (v_start,
    v_end) grid -> metrics for the move file."""
    from .control.policy import load_policy_npz
    pol = load_policy_npz(npz_path)
    ecfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = PivotEnv(params, ecfg)

    def act(obs):
        a = pol.action(obs)
        return np.array([a[0] / pol.bounds.steer_rate_max,
                         a[1] / pol.bounds.hub_max,
                         a[2] / pol.bounds.diff_max])[:env.action_space.shape[0]]

    return _eval_episodes(env, act, eval_grid(cfg["env"]["v_max"]))


def _finish(model, vecnorm, params, cfg, total, source=None, name="pivot_rl"):
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
           "yaw_target_deg": cfg["env"]["yaw_target_deg"],
           "max_episode_s": cfg["env"]["max_episode_s"],
           "v_max": cfg["env"]["v_max"],
           "action_space": cfg["env"]["action_space"],
           "trained": trained}
    with open(MOVES_DIR / f"{name}.yaml", "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"wrote {MOVES_DIR / f'{name}.yaml'} and {npz}")


def _scan_checkpoints(params, cfg, every=1):
    """Evaluate every saved checkpoint over the (v_start, v_end) grid, so an
    interesting mid-run policy can be found and exported after the fact."""
    import pickle
    grid = eval_grid(cfg["env"]["v_max"])
    ckpt = RUN_DIR / "checkpoints"
    zips = sorted(ckpt.glob("ppo_*_steps.zip"),
                  key=lambda p: int(p.stem.split("_")[1]))[::every]
    if not zips:
        raise SystemExit(f"no checkpoints in {ckpt}")
    ecfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = PivotEnv(params, ecfg)
    print(f"{'steps':>9} {'success':>8} {'hold_rms':>9} {'yaw_err':>8} "
          f"{'v_err':>6} {'score':>6}")
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

        m = _eval_episodes(env, act, grid)
        score = _score(m)
        rows.append((steps, score, m))
        print(f"{steps:>9} {m['success_rate']:>8.2f} {m['hold_rms_deg']:>9.1f} "
              f"{m['final_yaw_err_deg']:>8.1f} {m['final_v_err']:>6.2f} "
              f"{score:>6.3f}")
    if rows:
        best = max(rows, key=lambda r: r[1])
        print(f"\nbest score {best[1]:.3f} at {best[0]} steps "
              f"(success {best[2]['success_rate']:.2f}, hold "
              f"{best[2]['hold_rms_deg']:.1f} deg)")
        print(f"export it with:  python -m aow_sim.train_pivot_rl "
              f"--export-from {best[0]}")


def _export_from(spec: str, params, cfg, name="pivot_rl"):
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
    from .control.pivot_spec import OBS_DIM
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
    ap.add_argument("--export-name", default="pivot_rl", metavar="NAME",
                    help="move name for --export-from (e.g. pivot_rl_500k)")
    ap.add_argument("--scan-checkpoints", action="store_true",
                    help="evaluate every saved checkpoint, then exit")
    ap.add_argument("--scan-every", type=int, default=1,
                    help="with --scan-checkpoints, evaluate every Nth checkpoint")
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
          BestByScore(params, cfg,
                      eval_freq=max(1, a.get("eval_every", 100_000) // a["n_envs"]),
                      save_path=RUN_DIR, verbose=1)]
    total = args.timesteps or a["total_timesteps"]
    model.learn(total_timesteps=total, callback=cb, reset_num_timesteps=not args.resume,
                progress_bar=True)
    venv.save(str(vn_path))

    # Export the best-by-score snapshot, not whatever the last update produced.
    best_zip = RUN_DIR / "best_model.zip"
    best_vn = RUN_DIR / "best_vecnormalize.pkl"
    if best_zip.exists() and best_vn.exists():
        import pickle
        with open(best_vn, "rb") as f:
            best_norm = pickle.load(f)
        bi = cb[1].best_info
        print(f"exporting best snapshot: score {bi.get('score', 0):.3f} "
              f"(success {bi.get('success_rate', 0):.2f}, hold "
              f"{bi.get('hold_rms_deg', 0):.1f} deg) "
              f"from {bi.get('steps', '?')} steps")
        _finish(PPO.load(str(best_zip), device="cpu"), best_norm, params, cfg,
                bi.get("steps", total), source="best_model.zip")
    else:
        _finish(model, venv, params, cfg, total)
    if args.trace:
        from .train_flick_rl import _trace
        _trace("pivot_rl", args.trace)


if __name__ == "__main__":
    main()
