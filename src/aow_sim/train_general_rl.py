"""Train the GENERAL command-conditioned policy: one always-on controller
tracking a live (velocity vector, heading) command (see general_env.py).
Requires the `[rl]` extra; the base install replays the result with numpy.

  pip install -e '.[rl]'
  python -m aow_sim.train_general_rl               # reads config/rl/rl_general.yaml
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
from .params import params_digest, plant_digest
from .smooth_ppo import SmoothPPO, parse_smooth
from .recurrent_ppo import MemoryPolicy, RecurrentSmoothPPO, parse_memory, sb3_actor
from .control.flick import MOVES_DIR, reserve_move_name
from .control.general_env import GeneralEnv, _load_rl_config
from .control.general_spec import ActionBounds, obs_layout, righting_module_for
from .control.policy import save_policy_npz
# The eval (grid, metrics, eval config) lives in eval_general, torch-free;
# re-exported here for every existing `from aow_sim.train_general_rl import`.
from .eval_general import (FAMILIES, _EVAL_CMDS, _EVAL_EPISODE_S,  # noqa: F401
                           _HEAD_TOL_DEG, _TAIL_S, _behaviour_metrics, _by_family,
                           _eval_cfg, _eval_episodes, _is_self_mirror, _med, _mirror,
                           _mirrored_grid, _print_rows, _score, eval_cmds)
from .sim_ahrs import level_tau_orient_s

RUN_DIR = Path(__file__).resolve().parents[2] / "runs" / "general_rl"

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
            self._env = GeneralEnv(self.params, _eval_cfg(self.cfg))
        vn = self.model.get_vec_normalize_env()
        act = sb3_actor(self.model, vn)        # stateful for a memory policy
        m, rows = _eval_episodes(self._env, act, self.cmds)
        score = _score(m)
        keys = ["survive_rate", "track", "track_geo", "vel_err",
                "head_err_deg", "drift_m", "drift_lon", "drift_lat",
                "steer_rest_deg",
                "speed_ratio_fwd", "speed_ratio_rev", "turn_asym"]
        # Only on an arm that HAS a mechanism -- a wingless run would just get
        # two flat zero traces. Both fields are always present in `m`
        # (_behaviour_metrics uses .get), so this gates the RECORDING, not the
        # computation. Without it the numbers were computed every eval and
        # thrown away here, which is why a wings arm's board had no wing trace.
        if getattr(self._env, "wings", False) or getattr(self._env, "swing", False):
            keys += ["wing_deg", "wing_duty"]
        for k in keys:
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
        f = [i["diff_scale"] for i in self.locals.get("infos", ())
             if "diff_scale" in i]
        if f and min(f) < 1.0:
            self.logger.record("curriculum/diff_scale", float(np.mean(f)))
        return True


def export_bounds(cfg) -> dict:
    """The action bounds a policy is EXPORTED with: the config's, except that a
    differential faded out by the curriculum (`env.diff_fade`) is written as
    diff_max 0. The bounds travel in the .npz and replay scales by them, so
    exporting the training bound would let teleop and the Pi apply a channel
    the finished policy was never scored with."""
    b = dict(cfg["env"]["action_bounds"])
    lo, hi = cfg["env"].get("diff_fade", (0.0, 0.0))
    if hi > lo:
        b["diff_max"] = 0.0
    return b


def _export(model, vecnorm, cfg, path_npz: Path):
    """Pull the deterministic policy MLP + VecNormalize obs stats out of SB3
    and save as a numpy .npz (see control/policy.py for the replay side).
    A MemoryPolicy also writes its encoder and MinGRU layer."""
    policy = model.policy
    memory = None
    if isinstance(policy, MemoryPolicy):
        def wb(m):
            return (m.weight.detach().cpu().numpy(), m.bias.detach().cpu().numpy())
        memory = (wb(policy.encoder), wb(policy.memory.lin))
    layers = []
    for m in policy.mlp_extractor.policy_net:          # Linear/Tanh sequence
        if isinstance(m, torch.nn.Linear):
            layers.append((m.weight.detach().cpu().numpy(),
                           m.bias.detach().cpu().numpy()))
    an = policy.action_net                             # final mean layer
    layers.append((an.weight.detach().cpu().numpy(), an.bias.detach().cpu().numpy()))
    obs_mean = vecnorm.obs_rms.mean.astype(np.float32)
    obs_var = vecnorm.obs_rms.var.astype(np.float32)
    bounds = ActionBounds(**export_bounds(cfg))
    save_policy_npz(path_npz, layers, "tanh", obs_mean, obs_var, bounds,
                    obs_clip=float(vecnorm.clip_obs), memory=memory)
    return layers, obs_mean, obs_var, bounds


def _verify_export(model, vecnorm, npz_path):
    """The numpy export must match the trained net. Check a batch of random
    observations agree before trusting the artifact. Fed as ONE SEQUENCE
    from an empty memory, so a memory policy's recurrence is checked too
    (a stateless MLP is unaffected by the order)."""
    from .control.policy import load_policy_npz
    pol = load_policy_npz(npz_path)
    rng = np.random.default_rng(0)
    worst = 0.0
    trained = sb3_actor(model, vecnorm)
    trained.reset()
    pol.reset()
    for _ in range(64):
        obs = rng.standard_normal(pol.obs_mean.shape[0]).astype(np.float32)
        with torch.no_grad():
            sb3_mean = trained(obs)
        act = pol.action(obs)
        # Arity follows the policy's output width (3, or 4 with the wing
        # channel), so unpack positionally rather than by fixed name count.
        scales = [pol.bounds.steer_rate_max, pol.bounds.hub_max,
                  pol.bounds.diff_max, pol.bounds.wing_rate_max]
        # A bound of 0 DISABLES that channel (scale_action multiplies by it),
        # which is a legitimate config -- action_bounds.hub_max: 0.0 is how the
        # hold-without-the-hub diagnostic pins the wheel. Dividing by it raised
        # ZeroDivisionError and killed the run AFTER training finished and the
        # .npz was written, leaving an export with no .yaml beside it.
        #
        # A disabled channel is EXCLUDED rather than unit-scaled. `pol.action`
        # returns 0 for it (scaled by the zero bound) while sb3_mean still
        # carries the net's raw output, so comparing them reports a mismatch of
        # up to 1.0 on a channel that cannot affect the robot -- a false alarm
        # on exactly the check that is supposed to catch real export bugs.
        keep = [i for i, s in enumerate(scales[:len(sb3_mean)]) if s]
        raw = np.array([act[i] / scales[i] for i in keep])
        sb3_mean = np.asarray(sb3_mean)[keep]
        worst = max(worst, float(np.max(np.abs(raw - sb3_mean))))
    return worst


def _eval(params, cfg, npz_path):
    """Deterministic eval of the exported numpy policy over the command grid
    -> metrics for the move file."""
    from .control.policy import load_policy_npz
    pol = load_policy_npz(npz_path)
    env = GeneralEnv(params, _eval_cfg(cfg))

    # Scales in channel order; the 4th is present only for a wing policy and
    # is sliced off below for any env that does not have the channel.
    scales = np.array([pol.bounds.steer_rate_max, pol.bounds.hub_max,
                       pol.bounds.diff_max,
                       max(pol.bounds.wing_rate_max, 1e-9)])

    # A zero bound is a disabled channel (pol.action returns 0 for it);
    # dividing by it gave 0/0 = NaN, so divide by 1 as ActionBounds.normalize does.
    scales = np.where(scales == 0.0, 1.0, scales)

    def act(obs):
        a = np.asarray(pol.action(obs), float)
        return (a / scales[:len(a)])[:env.action_space.shape[0]]

    act.reset = pol.reset
    m, rows = _eval_episodes(env, act, eval_cmds(cfg["env"]["v_max"]))
    _print_rows(rows)
    return m


def move_doc(name: str, params, cfg, trained: dict) -> dict:
    """The move yaml for a general policy: what replay needs to rebuild its
    plant, sensors and observation, plus `trained`. Shared by `_finish`
    (SB3) and gpu/puffer/export_policy.py (PufferLib checkpoints)."""
    return {"name": name, "type": "rl", "kind": "general",
           "policy_file": f"{name}.npz",
           # The parameter set this policy was TRAINED against.
           # Replay warns on a mismatch (control/flick.py::
           # check_move_digest) — a policy is an artifact of the
           # plant it saw, and nothing else records which that was.
           "params_digest": params_digest(params),
           # What a policy is actually judged on: the BIKE, not the
           # controller settings it never reads. See params.py.
           "plant_digest": plant_digest(params),
           # control_rate_hz is part of the contract: replay must query the
           # policy at the rate it was trained at, not the controller rate.
           "control_rate_hz": cfg["env"]["control_rate_hz"],
           "v_max": cfg["env"]["v_max"],
           # Lateral (crab) envelope as a fraction of v_max -- teleop clamps
           # its crab command to it, so the operator cannot command a sideways
           # speed outside what this policy trained on.
           "v_lat_frac": cfg["env"]["v_lat_frac"],
           # Velocity-window time constant [s]. Part of the OBSERVATION
           # CONTRACT, not a preference: it decides whether this policy is 15
           # or 17 wide, and replay must low-pass with the same constant.
           "vel_window_s": cfg["env"].get("vel_window_s", 0.0),
           # Zeroed lateral velocity. MUST be written even when false, for
           # the same reason obs_swing must: replay rebuilds the observation
           # from these flags, and one that is merely absent reads as False.
           # It does NOT change the width, so obs_layout cannot catch a
           # mismatch here -- this field is the only record.
           "obs_zero_lat": bool(cfg["env"].get("obs_zero_lat", False)),
           # Trained with the ONBOARD ESTIMATE in the observation rather than
           # MuJoCo truth. On hardware this needs nothing: HardwareData.qvel
           # already carries the estimate. In SIM REPLAY it does -- teleop must
           # be run with `--odometry`, or the policy is handed a cleaner signal
           # than it was trained on. Width is unchanged, so obs_layout cannot
           # catch the mismatch; this field is the only record.
           "obs_odometry": bool(cfg["env"].get("obs_odometry", False)),
           # Which ENCODER MODEL the estimate came through: "ideal"
           # (instantaneous joint velocity) or "counts" (4096/rev quantised,
           # differenced, RateFilter -- the hardware path). Only meaningful
           # with obs_odometry. Like it, invisible to obs_layout.
           "odometry_encoder": str(cfg["env"].get("odometry_encoder", "ideal")),
           # How the STEER RATE (obs[5]) was formed: "joint" (exact MuJoCo
           # velocity, every export before 2026-10-17) or "counts" (the Pi's
           # differenced, RateFilter'd servo counts -- sim_odometry.SimSteerRate).
           # Written even at its default: replay and teleop rebuild the
           # observation from it, and the width does not change.
           "steer_rate_encoder": str(cfg["env"].get("steer_rate_encoder", "joint")),
           # THE ATTITUDE ERROR MODEL, and the exact counterpart of the two
           # fields above -- it corrupts roll, roll_rate and yaw_rate in
           # place, so obs_layout cannot catch a mismatch and this is the only
           # record. Unrecorded until 2026-09-11, which is how every analysis
           # script that did not patch its own cfg evaluated an AHRS-trained
           # policy on MuJoCo attitude. `ahrs_tau_s` is written EXPLICITLY
           # even when the config left it defaulted, so the export pins the
           # number rather than inheriting whatever `sim_ahrs.TAU_ORIENT_S`
           # happens to be later -- it has already moved once, 2.0 -> 0.19.
           # The default is the LEVEL's own, as SimAhrs takes it: for
           # tm151_filter that is the wander time (2.0), not 0.19.
           "ahrs_level": str(cfg["env"].get("ahrs_level", "none")),
           "ahrs_tau_s": float(cfg["env"].get(
               "ahrs_tau_s", level_tau_orient_s(
                   str(cfg["env"].get("ahrs_level", "none"))))),
           "ahrs_channels": str(cfg["env"].get("ahrs_channels", "both")),
           # THE DRIVETRAIN, resolved: the whole overlay dict, or null for the
           # ideal drives. The dict rather than a path, so the policy replays
           # on the plant it trained on after config/drivetrain_model.yaml has
           # moved on. Invisible to obs_layout, like the sensors above.
           "drivetrain_model": params.get("drivetrain_model"),
           # The wheel play it trained with (build_model's optional
           # `wheel_play:`), resolved; null = none, every export before
           # 2026-10-26. Same reason as the drivetrain: replay rebuilds it.
           "wheel_play": params.get("wheel_play"),
           # The floor it trained on (`floor:`, params.with_floor), as the
           # config gave it; null = bike_params' sim contact.
           "floor": cfg["env"].get("floor"),
           "obs_pitch": bool(cfg["env"].get("obs_pitch", False)),
           "obs_wings": bool(cfg["env"].get("obs_wings", False)),
           "act_wings": bool(cfg["env"].get("act_wings", False)),
           # The co-rotating pair. MUST be written even when false: replay
           # reads these off the move to rebuild the observation, and a flag
           # that is merely absent reads as False -- which is how a swing
           # policy came out declaring the 15-wide base layout while its net
           # was 17, and failed to load with a message blaming a spec change.
           "obs_swing": bool(cfg["env"].get("obs_swing", False)),
           "act_swing": bool(cfg["env"].get("act_swing", False)),
           "wing_max_deg": float(cfg["env"].get("wing_max_deg", 90.0)),
           # The style bit (general_spec.wheel_err): three more obs
           # entries, so it is in obs_layout too; written even when false, like
           # obs_swing. style_p / w_style are training-only and not replayed.
           "obs_style": bool(cfg["env"].get("obs_style", False)),
           # The detent phase and the measured diff rate (general_spec.
           # detent_angle): three entries; the bike reads its flip off
           # onboard.yaml's detent_flip.
           "obs_detent": bool(cfg["env"].get("obs_detent", False)),
           # WHICH swing mechanism, and what is on it. The layout is the same
           # for the geared stand-in and the four-bar (one crank channel), so
           # obs_layout cannot tell them apart; these are the only record.
           "swing_linkage": bool(cfg["env"].get("swing_linkage", False)),
           # The righting module the bike carried in training (v2 / v1 /
           # none), driven or not: a non-swing policy still trained with
           # its mass and its blades.
           "righting_module": righting_module_for(cfg["env"]),
           # ...and whether it was WELDED at stow (no joints, no servo). The
           # eval rebuilds it that way; teleop and the bike fly it jointed.
           "righting_rigid": bool(cfg["env"].get("righting_rigid", False)),
           # Training DR, never replayed (the eval builds no ball), but it
           # also put 2 x 20 g of hockey-stick panels on the chassis: the
           # only record of whether this policy trained with them.
           "ball_prob": float(cfg["env"].get("ball_prob", 0.0)),
           "swing_tip_mass_kg": float(cfg["env"].get("swing_tip_mass_kg", 0.0)),
           "wing_touch_fails": bool(cfg["env"].get("wing_touch_fails", False)),
           # The resulting entry names, recorded explicitly. Two optional
           # 2-entry blocks make WIDTH ambiguous (a windowed policy and a
           # pitch-observing one are both 17), so replay compares this list
           # rather than a length -- see drive.engage_general.
           "obs_layout": list(obs_layout(
               cfg["env"].get("vel_window_s", 0.0),
               cfg["env"].get("obs_pitch", False),
               cfg["env"].get("obs_wings", False),
               cfg["env"].get("obs_swing", False),
               cfg["env"].get("obs_style", False),
               cfg["env"].get("obs_detent", False))),
           "action_space": cfg["env"]["action_space"],
           "trained": trained}


def _finish(model, vecnorm, params, cfg, total, source=None, name="general_rl"):
    """Export -> verify -> eval -> write the move file."""
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
    smooth = parse_smooth(a.get("smooth"))
    if any(smooth.values()):
        # The loss it trained under; nothing in the weights records it.
        trained["smooth"] = smooth
    memory = parse_memory(a.get("memory"))
    if memory:
        # The .npz says it is recurrent (arch: mingru); this records how it
        # was trained (state width, truncated-backprop length).
        trained["memory"] = memory
    if source:
        trained["exported_from"] = source
    doc = move_doc(name, params, cfg, trained)
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
    env = GeneralEnv(params, _eval_cfg(cfg))
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

        act = sb3_actor(model, vn)
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
    # Against the width THIS CONFIG implies, not the bare OBS_DIM constant.
    # The optional blocks widen the observation -- vel_window_s appends v_bar,
    # obs_pitch appends (pitch, pitch_rate) -- so a glide_pitch checkpoint is
    # 19 wide while OBS_DIM is 15, and comparing to the constant rejected every
    # such checkpoint with a message blaming a spec change that never happened.
    # Same class of bug as the call sites general_spec.policy_flags documents:
    # a consumer that forgot the optional blocks.
    from .control.general_spec import obs_dim_for
    env_cfg = cfg["env"]
    want = obs_dim_for(vel_window_s=float(env_cfg.get("vel_window_s", 0.0)),
                       obs_pitch=bool(env_cfg.get("obs_pitch", False)),
                       obs_wings=bool(env_cfg.get("obs_wings", False)),
                       obs_swing=bool(env_cfg.get("obs_swing", False)),
                       obs_style=bool(env_cfg.get("obs_style", False)),
                       obs_detent=bool(env_cfg.get("obs_detent", False)))
    got = int(model.observation_space.shape[0])
    if got != want:
        raise SystemExit(
            f"{src.name} was trained with obs_dim {got}; this config implies "
            f"{want} — the checkpoint and the config disagree "
            "and cannot replay. Retrain first.")
    steps = int(src.stem.split("_")[1])
    print(f"exporting {src.name} (+ {vn.name}) without training")
    _finish(model, vecnorm, params, cfg, steps, source=src.name, name=name)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=None)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--timesteps", type=int, default=None, help="override total")
    # SEED OVERRIDE, so a seed sweep needs no per-seed config copy. It reaches
    # `_make_vecenv` (the env reset streams) and PPO itself -- exactly what
    # `algo.seed` feeds; this only changes where the number comes from.
    #
    # WHY IT IS WORTH HAVING. The turn personality -- which way a policy
    # resolves a turn, body-forward or reversed out -- is set by the SEED and
    # not by the config: rl_general_cmd_curriculum2 and _2b differ only in
    # `algo.seed` and resolve 33% vs 71% of their turns forward. Sweeping that
    # wants N launches of ONE config, not N configs. See
    # docs/plans/eval-score-rewrite.md "The turn personality", and
    # `scripts/rl.sh seeds`.
    ap.add_argument("--seed", type=int, default=None, metavar="N",
                    help="override algo.seed for this run")
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
    # The opt-in detailed drivetrain (env.drivetrain_model). Resolved HERE, not
    # only inside each env, so plant_digest and the move yaml record the plant
    # the policy actually trained on.
    from .drivetrain_model import describe, from_env_config
    params = from_env_config(params, cfg["env"])
    print(f"plant: {describe(params)}")
    a = cfg["algo"]
    if args.seed is not None:
        # Rebound before either consumer reads it: `_make_vecenv` below and
        # PPO's own `seed=` further down both take `a["seed"]`.
        a = {**a, "seed": args.seed}
        print(f"seed override: algo.seed -> {args.seed}")
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
    # Smoothness terms on the policy mean (smooth_ppo.py). Stock PPO unless a
    # weight is set, so every config without `algo.smooth` trains exactly as
    # before -- not merely equivalently.
    smooth = parse_smooth(a.get("smooth"))
    algo_cls, extra = ((SmoothPPO, {"smooth": smooth}) if any(smooth.values())
                       else (PPO, {}))
    if any(smooth.values()):
        print(f"smoothness loss: {smooth}")
    # MEMORY in the actor (recurrent_ppo.py): absent `algo.memory` leaves the
    # MLP path above untouched.
    memory = parse_memory(a.get("memory"))
    policy_cls = "MlpPolicy"
    if memory:
        algo_cls, extra = RecurrentSmoothPPO, {"smooth": smooth, "bptt": memory["bptt"]}
        policy_cls = MemoryPolicy
        policy_kwargs["memory_hidden"] = memory["hidden"]
        print(f"memory: MinGRU {memory['hidden']} in the actor, "
              f"backprop over {memory['bptt']}-step chunks")
    if args.resume and last_ckpt:
        model = algo_cls.load(str(last_ckpt[-1]), env=venv)
        if extra:
            model.smooth = smooth        # the config wins over the checkpoint
        print(f"resumed from {last_ckpt[-1].name}")
    else:
        model = algo_cls(policy_cls, venv, learning_rate=a["learning_rate"],
                         n_steps=a["n_steps"], batch_size=a["batch_size"],
                         n_epochs=a["n_epochs"], gamma=a["gamma"],
                         gae_lambda=a["gae_lambda"], clip_range=a["clip_range"],
                         ent_coef=a["ent_coef"], policy_kwargs=policy_kwargs,
                         seed=a["seed"], tensorboard_log=str(RUN_DIR), verbose=1,
                         **extra)

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
