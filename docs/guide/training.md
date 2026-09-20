# Training a policy

How to train, export, and inspect driving policies.

Hyperparameters and reward weights live in `config/rl_general*.yaml`.

Training needs the `rl` extra:

```sh
pip install -e '.[rl]'
```

An exported policy is just numpy weights, so the base install is sufficient to drive policies for replay and analysis.

---

## The general policy

`general_rl` is an **always-on controller**, not a canned maneuver. It has no horizon, no hand-back and no start-pose frame. It is engaged once and then driven by `set_command(v_cmd_world, psi_cmd)`: a **velocity vector** rather than (course, speed). Its observation is *stationary* (no phase input) which lets it run indefinitely.

It trains at 50 Hz with mid-episode step-change commands and a command curriculum; replay holds each action for the right number of controller ticks automatically. Exports land in `moves/` as a `*.yaml` describing the policy plus a `*.npz` of weights.

The contract both sides read is [`control/general_spec.py`](../../src/aow_sim/control/general_spec.py). **Change it and training and replay must move together**. It is the single observation/action layout they share.

What it trains *against*: the parametric model, the procedural omni-wheel contact, and decisions on their construction in [mujoco-modeling-decisions.md](../plans/mujoco-modeling-decisions.md).

## Training

```sh
python -m aow_sim.train_general_rl              # -> moves/general_rl.{yaml,npz}
```

Training is **not necessarily monotonic**, so the trainer keeps the best-scoring snapshot from a periodic deterministic eval and exports *that*, rather than whatever the last gradient update happened to produce.

| flag | what it does |
|---|---|
| `--config PATH` | which `config/rl_general*.yaml` to use |
| `--timesteps N` | override the config's total |
| `--resume` | continue from the last checkpoint |
| `--seed N` | seed the run |
| `--run-dir PATH` | where checkpoints and tensorboard logs go |
| `--scan-checkpoints` | score every saved checkpoint, to pick a good mid-run policy after the fact |
| `--scan-every N` | only score every Nth one |
| `--export-from STEPS\|PATH` | export that checkpoint instead of training |
| `--export-name NAME` | write `moves/NAME.{yaml,npz}` instead of the default |

Watch learning curves with `tensorboard --logdir runs/general_rl`.

### Banking a mid-run policy

Score the checkpoints, export the one you want under its own name, then compare them headless or side by side in teleop:

```sh
python -m aow_sim.train_general_rl --scan-checkpoints              # score them all
python -m aow_sim.train_general_rl --export-from 8000000 \
    --export-name general_rl_8m                                    # -> moves/general_rl_8m.*
python -m aow_sim.rollout_move general_rl_8m --out traces/         # CSV + PNG
mjpython -m aow_sim.run_drive --teleop --general general_rl_8m     # drive it
```

### Give every run its own `--run-dir`

Checkpoints, tensorboard logs and `best_model.zip` all live under `--run-dir`, so runs that name their own directory never touch each other:

```sh
python -m aow_sim.train_general_rl --run-dir runs/my_experiment \
    --export-name my_experiment
```

`scripts/rl.sh` passes a per-run directory for you.

### Which policy teleop drives

`control.general_move` in `config/bike_params.yaml` (default `general_rl`), overridden per session by `run_drive --general NAME`. The name selects `moves/NAME.yaml`, whose `policy_file:` field points at the weights, so two move files can share one `.npz`, and comparing exports never means renaming anything.

---

## Inspecting a policy

There are two different things you might mean by "run it and look": `rollout_move` drives **one scripted sequence** and plots the trace; `analysis/per_command.py` runs **the eval grid**: the twenty commands the trainer scores itself on. The grid is the one to reach for when comparing policies.

### `rollout_move`: one sequence, in detail

Replays a policy headless from a settled standstill, recording commanded and measured steer (multi-turn radians), unwrapped yaw, roll and speed at every physics step, and marking each command instant:

```sh
python -m aow_sim.rollout_move general_rl --out traces/    # summary + CSV + PNG
python -m aow_sim.rollout_move general_rl_8m --out traces/
```

There is no horizon to replay, so it drives a scripted sequence of step commands (drive off, turn at speed, stop, reverse, about-face) and marks each command change on the plot. **That sequence is not the eval grid**, so the numbers it prints are not the numbers a policy is scored on.

### `record`: something to look at

`rollout_move` gives you numbers; `aow_sim.record` gives you video. It renders **offscreen** (no viewer, no `mjpython`) so it runs over ssh or on the training box, and writes two artifacts per run: an `.mp4`, and a contact-sheet `.png` with one captioned tile per scripted event, so the whole run reads as a single still.

```sh
python -m aow_sim.record --script crab --general general_rl_1k   # crab, or curve?
python -m aow_sim.record --script drive --analytic               # the LQR instead
python -m aow_sim.record --script o --camera chase --fps 60
python -m aow_sim.record --script right                          # self-righting
```

| flag | what it does |
|---|---|
| `--script` | `eval_*` are single eval-grid commands (see [media.md](media.md)); `crab` / `drive` diagnose a controller; `o` / `s` / `t` draw letters; `right` starts fallen and films the righting stroke |
| `--general NAME` / `--analytic` | which controller drives |
| `--camera` | `top` fixed world view (shows translation, the default); `corner` 3/4, the one that shows pitch *and* roll; `chase` / `rear` / `front` / `side` locked to the bike (shows attitude). `--script right` defaults to `rear` |
| `--distance` / `--elevation` / `--azimuth` | framing. `top` auto-fits the distance to the path actually taken |
| `--seconds`, `--fps`, `--width` / `--height`, `--out` | length, frame rate, resolution, output path |
| `--hockey` | the ball scene |

Every frame carries the ground dial and a **world-frame red trail** of where the bike has actually been. The trail helps visualize the bike's movement over time, and is especially helpful for visualizing drift.

`--script right` films from behind, which is the only view a roll-plane mechanism reads in.

Needs the `viz` extra (`imageio` + `imageio-ffmpeg`); `dev` includes it.

### Comparing policies

**The `metrics:` block in a `moves/*.yaml` is not comparable across policies.** Those numbers were written by training runs spanning different eval grids, different episode lengths and different code. Reading down a column across policies may compare different quantities.

Re-run the comparison in one process instead:

```sh
python analysis/per_command.py --policies A B C     # the eval grid, one bar per policy
python analysis/eval_video.py --policies A B        # the same grid, as video
```

**The eval grid is twenty commands**: hold, standstill quarter-turn and about-face, sprint, turn at speed, top speed, reverse, reverse turn, pure crab, glide-and-about-face, and the mirror image of every mirrorable command. It lives in `train_general_rl._EVAL_CMDS`, and the mirroring matters, because policies can (and do) learn handedness.

`per_command.py` plots the commands **individually**, grouped by family with mirrored pairs adjacent, so handedness reads straight off the chart. A command the policy fell on is hatched (and its results suspect).

`analysis/` holds one-off studies that ask a question of existing artifacts and change nothing. A figure worth keeping goes in `analysis/plots/`, and must be reproducible at the name it is tracked under.

## Training somewhere else

Training likely runs on a separate computer, not your laptop. `scripts/rl.sh` is the launcher: it starts tensorboard *and* training under `nohup`/`setsid` so both outlive the ssh session, writes per-launch timestamped logs, and records pids under `runs/`.

```sh
./scripts/rl.sh up general --config config/<cfg>.yaml \
    --run-dir runs/<name> --export-name <name>
./scripts/rl.sh status | eta general | logs general
```

Everything after the move name goes straight through to `train_<move>_rl`. Ports are per-move, so two moves can train and be watched at once.

A finished run leaves `moves/<name>.{npz,yaml}` untracked on the remote; `./scripts/rl.sh sync` fast-forwards that checkout without destroying them. Pull results back with plain `rsync`; `runs/` is gitignored, so git will not bring it.

**PPO's rollout buffer is `n_envs × n_steps`.** Change one and change the other
in the same edit, or you have silently changed the algorithm rather than the
parallelism.
