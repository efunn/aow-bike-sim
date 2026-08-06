# aow-bike-sim

MuJoCo simulator for an active omni wheel (AOW) RC two-wheeler.

The rear wheel is an active omni wheel (8 driven axles, 16 truncated-cone
rollers) scavenged from a HC-802 RC bike. Two Dynamixel
XC430-W150 servos drive the AOW differential, an XC330-T181 steers
(continuous 360°+), and a TM151 AHRS measures orientation.

A general RL policy is trained to balance and control the bike through heading+velocity commands. Various other control methods are also optional. For example, an LQR balance controller, short-lived RL 'moves' (2-4 second moves that take over for the LQR, then hand control back), and trajectory optimizations wrapped around the LQR.

## Quickstart

```sh
conda create -n aow-sim python=3.12 -y && conda activate aow-sim
pip install -e '.[dev]'

pytest                                          # model correctness tests
python -m aow_sim.view --training-wheels        # interactive viewer (open loop)
python -m aow_sim.view --variant testbed        # wheel-only system-ID rig
python -m aow_sim.build_model -o model.xml      # dump MJCF
```

In the open-loop viewer's Control panel: equal `drive_a`/`drive_b` rolls the
bike, differential input crawls the rear wheel sideways, `steer` is continuous.

## Controllers

```sh
python -m aow_sim.run_balance                   # balance metrics (LQR baseline)
python -m aow_sim.run_balance --controller pd   # PD cascade reference (legacy)
python -m aow_sim.run_balance --view            # watch it balance; shove it with
                                                #   double-click + Ctrl+right-drag

python -m aow_sim.run_pivot                     # crawl-pivot metrics + rate sweep
python -m aow_sim.run_pivot --view              # +180° / −180° pivot demo
mjpython -m aow_sim.run_pivot --teleop          # ←/→ ±30°, 6/7 ±90°, 8/9 ±180°

python -m aow_sim.run_drive                     # sprints, accel sweep, circle/flip/flick envelopes
python -m aow_sim.run_drive --view              # sprint + circle + stop + flip + flick demo
python -m aow_sim.run_drive --hockey            # + the ball/stick scene
mjpython -m aow_sim.run_drive --teleop          # RC-style driving (keys below)
```

Teleop bindings are digits, arrows, and `.` — MuJoCo's viewer owns every
letter A–Z:

Driving is game-style: `↑`/`↓` are throttle and brake-into-reverse (the
opposite key brakes hard through zero), and releasing both coasts the speed
target back to zero. `←`/`→` slew the heading continuously while held; heading
is a setpoint, so it stays where you leave it.

| key | action | key | action |
|---|---|---|---|
| ↑ / ↓ | throttle / brake→reverse | 8 / 9 | flick 180° (trajopt reverse-/forward-first) |
| ← / → | hold to turn | 3 | flick 180° (RL policy) |
| 6 / 7 | circle left / right | `.` | pivot 180°, front wheel holds its line (RL) |
| 4 | crawl front-pivot 180° | 1 / 0 | ball shot (RL) / re-park ball (`--hockey`) |
| 5 | stop now | `,` | general RL policy on / off |
| `/` | re-zero the command | 2 | toggle the ground dial |

The general RL policy drives by default (whenever `moves/general_rl` exists
and matches the current spec). It is a modal layer: the arrows command it,
the maneuver keys in the right-hand column are shadowed, and `6`/`7`/`8` snap
the heading 90°L / 90°R / 180°. `,` switches to the analytic controller and
back; either way the command is zeroed on the switch, so nothing inherits a
stale setpoint. The controller choice survives a viewer reset, rewinding time makes the
controller fall back to line mode internally, and teleop re-engages the policy
unless you turned it off.

A ground dial under the bike shows **heading** as a tick on the rim (green = commanded, cyan =
actual). Turns can be tapped or held. A *held* turn is clamped to 35° of lead over the actual heading, so it
can't wind up and leave the bike spinning after release — heading snaps
deliberately bypass that clamp. For **velocity**, an arrow is shown in the inner gauge whose full scale is
`v_max` (orange = commanded, yellow = actual). To control velocity, a fresh press steps the command by
 0.25 m/s, a held command continuously increases/decreases velocity, and the target coasts to zero once you release all buttons. 

## Moves

Maneuvers are authored offline into `moves/` and replayed by `DriveController`.
Two kinds: **trajectory optimization** (scipy; open-loop feedforward knots in
the yaml) and **RL policies** (closed-loop; exported as numpy weights, so
replay needs no torch — the base install runs every move).

```sh
python -m aow_sim.optimize_flick --reverse-first   # -> moves/flick.yaml
python -m aow_sim.optimize_flick --name flick_fwd  #    (never touches config)

pip install -e '.[rl]'                             # gymnasium + SB3 + torch, TRAINING only
python -m aow_sim.train_flick_rl                   # -> moves/flick_rl.{yaml,npz}
python -m aow_sim.train_ball_rl                    # -> moves/ball_rl.{yaml,npz}
python -m aow_sim.train_pivot_rl                   # -> moves/pivot_rl.{yaml,npz}
python -m aow_sim.train_general_rl                 # -> moves/general_rl.{yaml,npz}
```

| move | what it does |
|---|---|
| `flick` / `flick_fwd` | two-arc 180° flick from standstill (trajopt) |
| `flick_rl` | the same 180°, closed-loop and disturbance-robust |
| `ball_rl` | strike the ball with the side stick toward a world-frame target |
| `pivot_rl` | 180° chassis yaw while the front wheel holds its global heading — stationary or gliding (`v_start`/`v_end` up to 0.6 m/s), robust to a ball-hit impulse mid-turn |
| `general_rl` | **not a move** — an always-on controller tracking a live (velocity vector, heading) command; see below |

### The general policy

`general_rl` is a different kind of artifact: no horizon, no hand-back, no
start-pose frame. It is engaged once (`DriveController.engage_general`) and
then driven by `set_command(v_cmd_world, psi_cmd)` — a **velocity vector**
rather than (course, speed), so "stop" and "reverse" are ordinary points
instead of singularities. Its observation is *stationary* (no `phase`), which
is what lets it run indefinitely. It trains at 50 Hz with mid-episode
step-change commands and a command curriculum; replay automatically holds each
action for the right number of controller ticks. See
[general_spec.py](src/aow_sim/control/general_spec.py) for the contract.

All three trainers share a CLI. Training is not monotonic, so each keeps the
best-scoring snapshot from a periodic deterministic eval and exports *that*
rather than whatever the last update produced:

| flag | what it does |
|---|---|
| `--timesteps N` | override the config's total |
| `--resume` | continue from the last checkpoint |
| `--scan-checkpoints` | score every saved checkpoint, to pick a good mid-run policy post hoc |
| `--export-from STEPS\|PATH` | export that checkpoint instead of training |
| `--export-name NAME` | write `moves/NAME.{yaml,npz}` instead of the default |
| `--trace DIR` | roll the exported policy out and write a CSV + PNG |

Watch learning curves with `tensorboard --logdir runs/<move>`. Hyperparameters,
reward weights, and domain randomization live in `config/rl_<move>.yaml`.

## Inspecting a move

RL moves are closed-loop policies. `rollout_move` replays a move headless from a settled standstill and
records commanded/measured steer (multi-turn radians), unwrapped yaw, roll, and
speed at every physics step, marking the command and hand-back instants:

```sh
python -m aow_sim.rollout_move flick_rl --out traces/      # summary + CSV + PNG
python -m aow_sim.rollout_move flick --direction -1        # mirrored trajopt flick
python -m aow_sim.rollout_move pivot_rl --v-start 0.4 --v-end 0.4 --out traces/
python -m aow_sim.rollout_move general_rl --out traces/    # scripted command sequence
```

For `general_rl` there is no horizon to replay, so the tool drives a scripted
sequence of step commands (drive off, turn at speed, stop, reverse, about-face)
and marks each command change on the plot.

## Layout

- `config/bike_params.yaml` — physical bike parameters: every measurement,
  with units and provenance (`measured` / `tooth-count` / `datasheet` / `GUESS`).
  `GUESS` are parameters that still need to be identified.
- `config/rl_*.yaml` — per-move RL training configs (algo/env/reward/randomization).
- `src/aow_sim/` — parametric model builder (`mjSpec`), procedural contact
  meshes, viewer, runners, offline optimizers/trainers.
- `src/aow_sim/control/` — controllers (`balance`, `pivot`, `drive`), the
  multi-turn steering frame (`steer.py`, incl. the XC330 extended-position
  contract), and one `*_spec.py` + `*_env.py` per RL move — the spec is the
  single observation/action contract shared by training and replay.
- `src/aow_sim/hw/` — the physical bike. A `HardwareData` shim that quacks like
  `mjData`, so `DriveController` runs on the robot **unmodified**; the
  Dynamixel bus, the TM151 reader, the velocity estimator, and the onboard
  loop. `export_deploy.py` ships the LQR gain schedule to it so the Pi needs
  no MuJoCo model and no scipy. See `docs/plans/untethered-setup.md`.
- `moves/` — authored maneuvers (`*.yaml`, plus `*.npz` policy weights for RL).
- `runs/` — training checkpoints and tensorboard logs (gitignored).
- `docs/measurements/omni-wheel-protocol.md` — what to measure and how,
  including the testbed calibration experiments.
- `docs/plans/mujoco-modeling-decisions.md` — why the model is built this way.
- `tests/` — compilation, coupling-ratio, envelope, and behavior tests.
- `traces/` — diagnostics and plots for RL policies (gitignored).
