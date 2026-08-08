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
| 5 | stop now (incl. crab) | `,` | general RL policy on / off |
| `/` | re-zero the command | 2 | toggle the ground dial |
| 1 / 3 | crab left / right (general mode only) | F5 | fullscreen (Backspace resets) |
| `[` / `]` | trail: pen-up / 2s / 4s / 10s / inf | `` ` `` | camera: free → follow → overhead |

The general RL policy drives by default (whenever `moves/general_rl` exists
and matches the current spec). It is a modal layer: the arrows command it,
the maneuver keys in the right-hand column are shadowed, `6`/`7`/`8` snap
the heading 90°L / 90°R / 180°, and **`1`/`3` crab left / right** — sideways
travel with the heading held, which the rear omni makes physical. Crab is
general-mode only (the analytic controller has no lateral command, and there
`1`/`3` keep firing the ball shot and the RL flick). It behaves like the
throttle — tap to step, hold to build, release to coast back to zero — and is
clamped to the lateral envelope the policy trained on (`v_lat_frac` in its
move file, 0.4 × `v_max` ≈ 0.48 m/s). `,` switches to the analytic controller and
back; either way the command is zeroed on the switch, so nothing inherits a
stale setpoint.

```sh
python -m aow_sim.record --script o --general general_rl_1k --camera top
python -m aow_sim.record --script s   # two mirrored arcs; needs a symmetric policy
python -m aow_sim.record --script t   # crossbar, pen up, reposition, pen down, stem
```

(The recorder itself — flags, outputs, why the camera is fixed by default — is
documented under [Recording a run](#recording-a-run).)

Those drawings are driven through `control/gamepad.py`, a **virtual gamepad**:
axes in, a velocity vector plus a heading rate out. The keyboard and a future
controller are two front-ends onto the same mapping, so any shape a script can
draw is reachable by hand.

| stick / key | command |
|---|---|
| LEFT stick Y — `↑`/`↓` | longitudinal velocity (± `v_max`) |
| LEFT stick X — `1`/`3` | lateral velocity (± `v_lat_frac · v_max`) |
| RIGHT stick X — `←`/`→` | heading **rate** (integrates, so releasing holds the heading) |
| A / B / X / Y — `5` `/` `7` `6` | stop · re-zero · snap 90° L/R |
| LB / RB — `]` / `[` | trail longer / shorter |

`aow_sim.record` burns a **gamepad input overlay** into every video frame —
stick gates, pen state and the snap button — so a recording shows what was
commanded, not just what happened. The live viewer does **not** have it yet,
and deliberately so: the keyboard path has no continuous axes to display (a
held key is a ramp, not a deflection), so the gates would read as a square
wave. It becomes worth porting once `control/gamepad.py` is fed by a real
controller — the drawing function `_hud` in `record.py` already takes a `Pad`
and would move across as-is.

The controller choice survives a viewer reset, rewinding time makes the
controller fall back to line mode internally, and teleop re-engages the policy
unless you turned it off.

Pressing `` ` `` cycles the camera: **free** (mouse-driven, the viewer's own),
**follow** (chase, azimuth tracking the bike's heading) and **overhead** (plan
view). Both tracked modes hold the bike still in frame, so they also switch on
a 0.5 m **floor grid** — without a world-fixed reference a follow camera makes
a moving bike look parked. A **red trail** marks where the bike has actually
been: the last 2 s solid, then fading to clear over 0.5 s. `[` and `]` step
that history through **pen-up / 2s / 4s / 10s / inf**. Pen-up keeps what is
already drawn and stops adding, so the bike can be repositioned invisibly and a
disconnected shape drawn — that is how the `t` drawing puts a stem under a
crossbar without retracing it.

A ground dial under the bike shows **heading** as a tick on the rim (green = commanded, cyan =
actual). Turns can be tapped or held. A *held* turn is clamped to 35° of lead over the actual heading, so it
can't wind up and leave the bike spinning after release — heading snaps
deliberately bypass that clamp. For **velocity**, an arrow is shown in the inner gauge whose full scale is
`v_max` (orange = commanded, yellow = actual). To control velocity, a fresh press steps the command by
 0.25 m/s, a held command continuously increases/decreases velocity, and the target coasts to zero once you release all buttons. 

## Falling over

Two studies of what happens when balance is lost, and of a possible fourth
servo to stand the bike back up — see
[self-righting.md](docs/plans/self-righting.md) for the numbers and the
recommendation. Nothing is decided; the tools exist to re-sweep the geometry.

```sh
python analysis/no_return.py                 # the recoverable set in (roll, roll rate)
python analysis/no_return.py --controller lqr
python analysis/self_righting.py profile --sweep   # side geometry -> resting attitude
python analysis/self_righting.py rest              # ...checked against real falls
python analysis/self_righting.py lift --sweep      # arm length/pivot -> servo torque
python analysis/self_righting.py sequence          # fall -> right -> hand off -> retract
```

The headline: there is no tipping *angle* — the boundary is a curve in
(roll, roll rate) that moves with speed — and from the moment a fall is
visible the bike is flat in ~0.3 s. Nothing catches that, so the mechanism is
a righting mechanism, not a catch. `build_model(..., righting=True)` makes the
chassis lumps collidable and adds the study's bumper pads and arm; it is off
everywhere else, so training, teleop and deployment see the model they always
did.

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

Which policy teleop drives with is `control.general_move` in
`config/bike_params.yaml` (default `general_rl`), overridden per session by
`run_drive --general NAME`. The name selects `moves/NAME.yaml`, whose
`policy_file:` field points at the weights — so two move files can share one
`.npz`, and comparing exports never means renaming anything.

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

Banking a mid-run policy — score the checkpoints, export the one you want under
its own name, then compare them headless or side by side in teleop:

```sh
python -m aow_sim.train_general_rl --scan-checkpoints              # score them all
python -m aow_sim.train_general_rl --export-from 8000000 \
    --export-name general_rl_8m                                    # -> moves/general_rl_8m.{yaml,npz}
python -m aow_sim.rollout_move general_rl_8m --out traces/         # CSV + PNG
mjpython -m aow_sim.run_drive --teleop --general general_rl_8m     # drive it
```

**Checkpoints do not survive the next run.** A fresh run restarts the step
counter, so it overwrites `runs/<move>/checkpoints/ppo_<steps>_steps.zip` as it
passes each mark, and `best_model.zip` is replaced at its first eval. Only
checkpoints *beyond* where the new run reaches survive — mixed in with the new
ones, which makes `--scan-checkpoints` compare two different lineages. Export
what you want to `moves/` first (as above), or archive the whole run:
`cp -a runs/general_rl runs/general_rl.$(date +%F)`. Tensorboard logs are safe:
each run gets its own `PPO_<n>/` subdirectory.

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

### Recording a run

`rollout_move` gives you numbers; `aow_sim.record` gives you something to look
at. It renders **offscreen** — no viewer, no `mjpython` — so it runs over ssh
or on the training box, and writes two artifacts per run: an `.mp4`, and a
contact-sheet `.png` with one captioned tile per scripted event, so the whole
run reads as a single still.

```sh
python -m aow_sim.record --script crab --general general_rl_1k  # crab, or curve?
python -m aow_sim.record --script drive --analytic              # the LQR instead
python -m aow_sim.record --script o --camera chase --fps 60
```

| flag | what it does |
|---|---|
| `--script` | `crab` / `drive` diagnose a controller; `o` / `s` / `t` draw letters (see teleop above) |
| `--general NAME` / `--analytic` | which controller drives |
| `--camera top\|chase` | fixed world view (default) or locked to the bike |
| `--distance/--elevation/--azimuth` | framing; `top` auto-fits the distance to the path actually taken |
| `--out`, `--width/--height/--fps`, `--hockey` | output path, resolution, frame rate, ball scene |

Every frame carries the ground dial (teleop's `2` overlay) plus a **world-frame
red trail** of where the bike has actually been. The trail is the point: the
dial is drawn under the bike and travels with it, so it cannot show
displacement — it was the trail that revealed a commanded "crab" was really a
long curved drive. The camera defaults to fixed for the same reason, since a
chase cam pins the bike to the centre of frame and hides exactly what you came
to see.

The drawing scripts additionally burn in the gamepad input overlay; `crab` and
`drive` don't, because they are scripted as controller calls rather than pad
axes, so there are no stick positions to draw.

Needs the `[viz]` extra (`imageio` + `imageio-ffmpeg`) — `[dev]` includes it.

## Layout

- `config/bike_params.yaml` — physical bike parameters: every measurement,
  with units and provenance (`measured` / `tooth-count` / `datasheet` / `GUESS`).
  `GUESS` are parameters that still need to be identified.
- `config/rl_*.yaml` — per-move RL training configs (algo/env/reward/randomization).
- `src/aow_sim/` — parametric model builder (`mjSpec`), procedural contact
  meshes, viewer, runners, offline optimizers/trainers.
- `src/aow_sim/record.py` — offscreen recorder: drives a scripted run and
  renders it to MP4 + a contact sheet, with the ground dial and a world-frame
  trail. Needs no viewer, so it works headless — see *Recording a run*.
- `src/aow_sim/hw/` — the onboard stack for the physical bike (servo bus, AHRS,
  velocity estimation, control loop). Imports **without MuJoCo, scipy or
  torch** — see `docs/plans/untethered-setup.md`.
- `src/aow_sim/control/` — controllers (`balance`, `pivot`, `drive`), the
  multi-turn steering frame (`steer.py`, incl. the XC330 extended-position
  contract), and one `*_spec.py` + `*_env.py` per RL move — the spec is the
  single observation/action contract shared by training and replay.
- `src/aow_sim/hw/` — the physical bike. A `HardwareData` shim that quacks like
  `mjData`, so `DriveController` runs on the robot **unmodified**; the
  Dynamixel bus, the TM151 reader, the velocity estimator, and the onboard
  loop. `export_deploy.py` ships the LQR gain schedule to it so the Pi needs
  no MuJoCo model and no scipy. See `docs/plans/untethered-setup.md`.
- `analysis/` — one-off studies that ask a question of the trained artifacts
  rather than producing one. Each writes a PNG next to itself and changes
  nothing.
- `moves/` — authored maneuvers (`*.yaml`, plus `*.npz` policy weights for RL).
- `runs/` — training checkpoints and tensorboard logs (gitignored).
- `docs/measurements/omni-wheel-protocol.md` — what to measure and how,
  including the testbed calibration experiments.
- `docs/plans/mujoco-modeling-decisions.md` — why the model is built this way.
- `docs/plans/untethered-setup.md` — the physical bike: parts, power, onboard
  architecture, Pi setup and deployment, bring-up order.
- `docs/plans/self-righting.md` — where recovery stops being possible, what a
  fallen bike rests on, and what a fourth servo would have to be to stand it
  back up.
- `tests/` — compilation, coupling-ratio, envelope, and behavior tests.
- `traces/` — diagnostics and plots for RL policies (gitignored).
