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
from .control.balance import extract_state
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
    # Directional large turns. The +-180 rows above CANNOT measure handedness:
    # command_to_body uses wrap_pi(psi_cmd - psi), so +180 and -180 are the
    # same observation and which way the bike spins is decided by yaw noise.
    # 170 is far enough round to be an about-face and still unambiguous.
    (0.0, 0.0, 170.0),
    (0.0, 0.0, -170.0),
]


def _mirror(c: tuple[float, float, float]) -> tuple[float, float, float]:
    """Left-right reflection of a command: flip lateral velocity and heading
    step. The plant is mirror-symmetric (axle_cant_deg 0, cone pairs mirrored
    about the wheel mid-plane), so the reflected command is exactly as
    achievable as the original -- any difference in score is the policy's own
    handedness."""
    v_lon, v_lat, dpsi = c
    # `+ 0.0` normalizes the -0.0 that negating 0.0 produces, and a mirrored
    # +-180 stays +180 (it wraps to the same target either way) so the command
    # labels don't imply a direction the observation cannot express.
    dpsi = 180.0 if abs(dpsi) == 180.0 else -dpsi + 0.0
    return (v_lon, -v_lat + 0.0, dpsi)


def _is_self_mirror(c: tuple[float, float, float]) -> bool:
    """True when reflecting `c` yields the same commanded behaviour, so adding
    the mirror would only duplicate an episode. Straight-ahead commands
    (v_lat 0, dpsi 0) are self-mirror, and so is a pure +-180 heading step --
    psi_err wraps, making +180 and -180 identical."""
    _, v_lat, dpsi = c
    return v_lat == 0.0 and (dpsi == 0.0 or abs(dpsi) == 180.0)


def _mirrored_grid() -> list[tuple[float, float, float]]:
    """`_EVAL_CMDS` plus the reflection of every command that has a distinct
    one. Without this the grid only ever turns right and only ever crabs left,
    so a one-handed policy scores flawlessly and snapshot selection cannot see
    it."""
    out = list(_EVAL_CMDS)
    for c in _EVAL_CMDS:
        if not _is_self_mirror(c):
            m = _mirror(c)
            if m not in out:
                out.append(m)
    return out


def eval_cmds(v_max: float) -> list[tuple[float, float, float]]:
    """Absolute (v_lon, v_lat, heading-step rad) commands for the eval grid,
    scaled to the configured envelope."""
    return [(round(a * v_max, 3), round(b * v_max, 3), np.deg2rad(d))
            for a, b, d in _mirrored_grid()]


def _score(m: dict) -> float:
    """Snapshot-selection score (higher is better): survival x tracking.

    An always-on controller has no task success, so `survive_rate` is the
    fraction of eval episodes that did not fall, and the tracking term is
    the env's own bounded [0,1] reward average. Multiplying rather than
    adding is deliberate: a policy that tracks beautifully for 2 s and then
    falls must not outrank one that survives the whole episode.

    The tracking term is the GEOMETRIC mean over commands, not the arithmetic
    one: identical when every command scores alike, but a command the policy
    has given up on (track -> 0) drags the whole score down instead of being
    averaged away. The arithmetic mean is what let a policy that refuses to
    reverse outrank one that reverses -- it abandoned 2 of 12 commands and
    banked the survival it bought. Falls back to `track` for metrics dicts
    written before track_geo existed."""
    return m["survive_rate"] * m.get("track_geo", m["track"])


def _make_vecenv(params, cfg, n_envs, seed):
    return SubprocVecEnv([
        (lambda i=i: Monitor(GeneralEnv(params, cfg, seed=seed + i)))
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


_TAIL_S = 2.0        # steady-state window at the end of an episode [s]
_HEAD_TOL_DEG = 10.0  # "the turn is done" threshold for turn timing


def _eval_episodes(env, act_fn, cmds):
    """One episode per command point; per-command rows plus aggregates. The
    eval env has randomization disabled, so repeating a command adds nothing.

    Behavioural quantities (drift, steer-at-rest, achieved speed, time to
    heading) are read straight off the env rather than the info dict, so this
    stays a pure measurement change -- general_env.py is untouched and no
    trained policy can be affected by it. Each eval episode holds ONE command
    for its whole duration (reset sets `_next_resample` to infinity), so
    per-episode aggregates are well defined.
    """
    n = len(cmds)
    tail = max(1, int(_TAIL_S / env.ctrl_dt))
    rows = []
    for k, (v_lon, v_lat, dpsi) in enumerate(cmds):
        obs, _ = env.reset(seed=10_000 + k, options={
            "v_cmd": (v_lon, v_lat), "psi_cmd_rel": dpsi, "difficulty": 1.0})
        done, info = False, {}
        steps = 0
        t_head = None                      # first step inside _HEAD_TOL_DEG
        v_lons, v_lats, steers = [], [], []  # tail windows, refilled as we go
        while not done:
            obs, _r, term, trunc, info = env.step(act_fn(obs))
            done = term or trunc
            steps += 1
            if t_head is None and info.get("head_err_deg", 180.0) < _HEAD_TOL_DEG:
                t_head = steps
            s = extract_state(env.data, env._p0)
            v_lons.append(float(s.v_lon))
            v_lats.append(float(s.v_lat))
            steers.append(abs(float(env.data.qpos[env._sj])))
            del v_lons[:-tail], v_lats[:-tail], steers[:-tail]
        s = extract_state(env.data, env._p0)
        fell = bool(info.get("fell", True))
        rows.append({
            "cmd": (round(v_lon, 3), round(v_lat, 3), round(np.degrees(dpsi))),
            "track": float(info.get("track", 0.0)),
            "fell": fell,
            "vel_err": float(info.get("vel_err", 9.9)),
            "head_err_deg": float(info.get("head_err_deg", 180.0)),
            "drift_m": float(np.hypot(s.e_lon, s.e_lat)),
            "steer_deg": float(np.degrees(np.mean(steers))) if steers else 0.0,
            "v_ach": float(np.mean(v_lons)) if v_lons else 0.0,
            # Achieved LATERAL speed. Without it a crab command is scored only
            # through v_ach (longitudinal), which reads ~0 whether the bike
            # crabbed properly, yawed away and slid, or simply sat there.
            "v_lat_ach": float(np.mean(v_lats)) if v_lats else 0.0,
            # A turn that never got inside tolerance is censored at the episode
            # length, not dropped -- "never finished" must cost more than slow.
            "t_head_s": (t_head if t_head is not None else steps) * env.ctrl_dt,
            "steps": steps,
        })

    tracks = [r["track"] for r in rows]
    m = {"survive_rate": sum(not r["fell"] for r in rows) / n,
         "track": round(float(np.mean(tracks)), 3),
         # Geometric mean: equals the arithmetic mean when commands score
         # alike, but any ABANDONED command (track -> 0) drags it down. The
         # arithmetic mean let a policy that refuses to reverse hide behind
         # ten other commands it does well.
         "track_geo": round(float(np.exp(np.mean(np.log(
             np.clip(tracks, 1e-3, 1.0))))), 3),
         "vel_err": round(float(np.mean([r["vel_err"] for r in rows])), 3),
         "head_err_deg": round(float(np.mean([r["head_err_deg"] for r in rows])), 1),
         "n_eval": n}
    m.update(_behaviour_metrics(rows))
    return m, rows


def _behaviour_metrics(rows: list[dict]) -> dict:
    """Diagnostic behaviour summary -- reported and logged, deliberately NOT
    in `_score`. Drift and steer-at-rest are free in the *reward* (the action
    is steer rate, so a cocked wheel costs nothing to hold), so making
    selection chase them without fixing the reward would just relocate the
    underdetermination rather than remove it."""
    def by(pred):
        return [r for r in rows if pred(r["cmd"])]

    hold = by(lambda c: c[0] == 0 and c[1] == 0 and c[2] == 0)
    fwd = by(lambda c: c[0] > 0.1)
    rev = by(lambda c: c[0] < -0.1)
    crab_l = by(lambda c: c[1] > 0.1)
    crab_r = by(lambda c: c[1] < -0.1)
    crab = crab_l + crab_r

    def ratio(sel):     # achieved / commanded longitudinal speed
        if not sel:
            return float("nan")
        return float(np.mean([np.clip(r["v_ach"] / r["cmd"][0], 0.0, 1.5)
                              for r in sel]))

    def crab_ratio(sel):  # achieved / commanded LATERAL speed, per side
        if not sel:
            return float("nan")
        return float(np.mean([np.clip(r["v_lat_ach"] / r["cmd"][1], 0.0, 1.5)
                              for r in sel]))

    # Handedness: pair each turning command with its reflection and compare
    # how long each took to arrive. Symmetric plant => a symmetric policy
    # scores 0 here; 1.0 means one side never arrived at all.
    times = {r["cmd"]: r["t_head_s"] for r in rows}
    asyms, seen = [], set()
    for c, t in times.items():
        # Only commands that actually demand a heading change, and only
        # those with a distinct mirror. A `c[2] > 0` filter looks like it
        # counts each pair once but does not: -180 normalizes to +180, so
        # BOTH members of a 180 pair pass it (double-counted) and a
        # self-mirror 180 pairs with itself, contributing a spurious 0 that
        # dilutes the average.
        mc = _mirror(c)
        key = frozenset((c, mc))
        if _is_self_mirror(c) or abs(c[2]) < 1e-9 or mc not in times or key in seen:
            continue
        seen.add(key)
        t2 = times[mc]
        if max(t, t2) > 0:
            asyms.append(abs(t - t2) / max(t, t2))
    return {
        "drift_m": round(float(np.mean([r["drift_m"] for r in hold])), 3)
                   if hold else float("nan"),
        "steer_rest_deg": round(float(np.mean([r["steer_deg"] for r in hold])), 1)
                          if hold else float("nan"),
        "speed_ratio_fwd": round(ratio(fwd), 3),
        "speed_ratio_rev": round(ratio(rev), 3),
        # Crab, per side: a policy that only crabs toward its preferred
        # steering direction shows one good ratio and one near zero.
        "crab_ratio_left": round(crab_ratio(crab_l), 3),
        "crab_ratio_right": round(crab_ratio(crab_r), 3),
        # Heading held DURING a crab. A crab is the one command where holding
        # heading and reaching the commanded velocity are simultaneously
        # demanding, and sigma_psi_deg 25 is loose enough that yawing ~15 deg
        # toward the travel direction is a cheap substitute for crabbing.
        "crab_head_err": round(float(np.mean([r["head_err_deg"] for r in crab])), 1)
                         if crab else float("nan"),
        "turn_asym": round(float(np.mean(asyms)), 3) if asyms else float("nan"),
    }


def _print_rows(rows: list[dict]) -> None:
    """Per-command table. The whole point: two abandoned commands are
    invisible in a mean, and obvious here."""
    # `dist` is displacement from the start pose: it is only "drift" on the
    # hold-station row (which is the only row the drift_m aggregate uses);
    # on a sprint row it is just how far the bike went.
    print(f"    {'v_lon':>6} {'v_lat':>6} {'dpsi':>5} | {'track':>6} {'v_ach':>6} "
          f"{'t_head':>6} {'dist':>6} {'steer':>6}  fell")
    for r in rows:
        v_lon, v_lat, dpsi = r["cmd"]
        print(f"    {v_lon:>6.2f} {v_lat:>6.2f} {dpsi:>5.0f} | {r['track']:>6.3f} "
              f"{r['v_ach']:>6.2f} {r['v_lat_ach']:>7.2f} "
              f"{r['head_err_deg']:>5.1f} {r['t_head_s']:>6.2f} "
              f"{r['drift_m']:>6.2f} {r['steer_deg']:>6.1f}  "
              f"{'X' if r['fell'] else ''}")


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

        m, rows = _eval_episodes(self._env, act, self.cmds)
        score = _score(m)
        for k in ("survive_rate", "track", "track_geo", "vel_err",
                  "head_err_deg", "drift_m", "steer_rest_deg",
                  "speed_ratio_fwd", "speed_ratio_rev", "turn_asym"):
            self.logger.record(f"eval/{k}", m[k])
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
                      f"{m['survive_rate']:.2f}, track_geo {m['track_geo']:.2f}, "
                      f"fwd/rev {m['speed_ratio_fwd']:.2f}/"
                      f"{m['speed_ratio_rev']:.2f}, asym {m['turn_asym']:.2f}) @ "
                      f"{self.num_timesteps} steps")
                _print_rows(rows)
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

    m, rows = _eval_episodes(env, act, eval_cmds(cfg["env"]["v_max"]))
    _print_rows(rows)
    return m


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
           # Lateral (crab) envelope as a fraction of v_max -- teleop clamps
           # its crab command to it, so the operator cannot command a sideways
           # speed outside what this policy trained on.
           "v_lat_frac": cfg["env"]["v_lat_frac"],
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
    # track_geo / fwd / rev / asym are the columns that separate checkpoints the
    # blended `track` ranked equally: a policy that abandons reverse or turns
    # one-handed shows up here and nowhere else.
    print(f"{'steps':>9} {'survive':>8} {'track':>7} {'geo':>7} {'fwd':>5} "
          f"{'rev':>5} {'crabL':>6} {'crabR':>6} {'cHerr':>6} {'asym':>5} "
          f"{'drift':>6} {'steer':>6} {'score':>6}")
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

        m, _ = _eval_episodes(env, act, cmds)
        score = _score(m)
        rows.append((steps, score, m))
        print(f"{steps:>9} {m['survive_rate']:>8.2f} {m['track']:>7.3f} "
              f"{m['track_geo']:>7.3f} {m['speed_ratio_fwd']:>5.2f} "
              f"{m['speed_ratio_rev']:>5.2f} {m['crab_ratio_left']:>6.2f} "
              f"{m['crab_ratio_right']:>6.2f} {m['crab_head_err']:>6.1f} "
              f"{m['turn_asym']:>5.2f} "
              f"{m['drift_m']:>6.2f} {m['steer_rest_deg']:>6.1f} {score:>6.3f}")
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
                    help="move file to write, for BOTH --export-from and the "
                         "export at the end of training (e.g. general_rl_2m). "
                         "Set it when training a variant, or the run will "
                         "overwrite moves/general_rl on completion.")
    ap.add_argument("--scan-checkpoints", action="store_true",
                    help="evaluate every saved checkpoint, then exit")
    ap.add_argument("--scan-every", type=int, default=1,
                    help="with --scan-checkpoints, evaluate every Nth checkpoint")
    ap.add_argument("--run-dir", default=None, metavar="PATH",
                    help="read/write this run directory instead of runs/general_rl "
                         "(e.g. re-score an archived run without moving it)")
    args = ap.parse_args()

    # Rebound before anything reads it: _scan_checkpoints and _export_from
    # resolve the module global, so this redirects checkpoints, tensorboard
    # logs and best_model together rather than half of them.
    global RUN_DIR
    if args.run_dir:
        RUN_DIR = Path(args.run_dir).resolve()
        print(f"run dir: {RUN_DIR}")

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
                source="best_model.zip", name=args.export_name)
    else:
        _finish(model, venv, params, cfg, total, name=args.export_name)


if __name__ == "__main__":
    main()
