"""Train an RL policy for the two-arc 180 flick — the closed-loop alternative to
the scipy trajectory optimization. Requires the `[rl]` extra (gymnasium, SB3,
torch, tensorboard); the base install needs none of these to *replay* the result.

  pip install -e '.[rl]'
  python -m aow_sim.train_flick_rl                 # reads config/rl_flick.yaml
  python -m aow_sim.train_flick_rl --resume        # continue from last checkpoint
  tensorboard --logdir runs/flick_rl               # watch learning curves

Mid-run policies: CheckpointCallback snapshots every ~100k steps into
runs/flick_rl/checkpoints/. To find and inspect an interesting one (e.g. a
peak on the learning curve) without retraining:

  python -m aow_sim.train_flick_rl --scan-checkpoints
  python -m aow_sim.train_flick_rl --export-from 500000 \\
      --export-name flick_rl_500k --trace traces/

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
from .params import params_digest
from .control.flick import MOVES_DIR, reserve_move_name
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


def _finish(model, vecnorm, params, cfg, total, source=None, name="flick_rl"):
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
    """Evaluate every saved checkpoint (deterministic, no randomization) and
    report success / yaw error / lateral envelope, so an interesting mid-run
    policy — training progress is not monotonic — can be found and exported
    after the fact with --export-from."""
    import pickle
    ckpt = RUN_DIR / "checkpoints"
    zips = sorted(ckpt.glob("ppo_*_steps.zip"),
                  key=lambda p: int(p.stem.split("_")[1]))[::every]
    if not zips:
        raise SystemExit(f"no checkpoints in {ckpt}")
    ecfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = FlickEnv(params, ecfg)
    L = params["bike"]["wheelbase"]
    print(f"{'steps':>9} {'success':>8} {'yaw_err':>8} {'lat[L]':>7}")
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
        succ, yerr, lat = 0, 0.0, 0.0
        for k in range(n_episodes):
            obs, _ = env.reset(seed=10_000 + k)
            done, info = False, {}
            while not done:
                act, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
                obs, _r, te, tr, info = env.step(act)
                done = te or tr
            succ += int(info.get("is_success", False))
            yerr += abs(info.get("yaw_err_deg", 180.0))
            lat = max(lat, abs(info.get("e_lat", 0.0)) / L)
        n = n_episodes
        rows.append((steps, succ / n, yerr / n, lat))
        print(f"{steps:>9} {succ/n:>8.2f} {yerr/n:>8.1f} {lat:>7.2f}")
    if rows:
        best = max(rows, key=lambda r: (r[1], -r[2]))   # success, then yaw err
        print(f"\nbest: success {best[1]:.2f}, yaw err {best[2]:.1f} deg at "
              f"{best[0]} steps")
        print(f"export it with:  python -m aow_sim.train_flick_rl "
              f"--export-from {best[0]}")


def _export_from(spec: str, params, cfg, name="flick_rl"):
    """Export a saved checkpoint instead of training — e.g. to grab an
    interesting mid-run policy. `spec` is a step count or a .zip path; the
    matching ppo_vecnormalize_*.pkl is loaded alongside it."""
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
    from .control.flick_spec import OBS_DIM
    got = int(model.observation_space.shape[0])
    if got != OBS_DIM:
        raise SystemExit(
            f"{src.name} was trained with obs_dim {got}; the current spec is "
            f"{OBS_DIM} — this checkpoint predates the observation-spec change "
            "and cannot replay. Retrain first.")
    steps = int(src.stem.split("_")[1])
    print(f"exporting {src.name} (+ {vn.name}) without training")
    _finish(model, vecnorm, params, cfg, steps, source=src.name, name=name)


def _trace(name: str, out_dir: Path) -> None:
    """Roll the freshly exported move out headless and write CSV + plot
    (see rollout_move.py)."""
    from .rollout_move import (_no_plot_hint, plot, rollout, rollout_general,
                               summarize, write_csv)
    # An always-on policy has no horizon to replay; it gets a command script.
    tr = rollout_general(name) if name.startswith("general") else rollout(name)
    print(summarize(tr))
    out_dir.mkdir(parents=True, exist_ok=True)
    csv = out_dir / f"{name}_trace.csv"
    write_csv(tr, csv)
    print(f"  wrote {csv}")
    png = out_dir / f"{name}_trace.png"
    if plot(tr, png):
        print(f"  wrote {png}")
    else:
        print(_no_plot_hint())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--timesteps", type=int, default=None, help="override total")
    ap.add_argument("--export-from", default=None, metavar="STEPS|PATH",
                    help="export an existing checkpoint instead of training "
                         "(e.g. --export-from 500000)")
    ap.add_argument("--export-name", default="flick_rl", metavar="NAME",
                    help="move name for --export-from (e.g. flick_rl_500k, so "
                         "the primary moves/flick_rl is left alone)")
    ap.add_argument("--scan-checkpoints", action="store_true",
                    help="evaluate every saved checkpoint (success/yaw err) "
                         "to pick a good one post-hoc, then exit")
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

    cb = CheckpointCallback(save_freq=max(1, 100_000 // a["n_envs"]),
                            save_path=str(ckpt), name_prefix="ppo",
                            save_vecnormalize=True)
    total = args.timesteps or a["total_timesteps"]
    model.learn(total_timesteps=total, callback=cb, reset_num_timesteps=not args.resume,
                progress_bar=True)
    venv.save(str(vn_path))

    _finish(model, venv, params, cfg, total)
    if args.trace:
        _trace("flick_rl", args.trace)


if __name__ == "__main__":
    main()
