# Getting started

Installing the simulator, driving the bike, and repo directions.

## Install

Python 3.11+.

```sh
python -m venv .venv && source .venv/bin/activate
pip install -e '.[dev]'
```

`[dev]` is enough for working on the simulator. The other extras:

| extra | pulls in | when you need it |
|---|---|---|
| *(base)* | mujoco, numpy, pyyaml, scipy | run the sim, replay a trained policy |
| `dev` | pytest, xdist, matplotlib, imageio | everything below except `rl` and `onboard` |
| `viz` | matplotlib, imageio, imageio-ffmpeg | `aow_sim.record`, and the PNGs the tools write beside their CSVs |
| `teleop` | pyobjc (macOS only) | **hold-to-drive.** Without it the arrow keys still work, but only tap-by-tap — MuJoCo's viewer reports key-down and nothing else, so real key state has to come from the OS |
| `rl` | gymnasium, SB3, torch, tensorboard | *CPU training only.* Replaying an exported policy needs numpy alone |
| `onboard` | pyserial, dynamixel-sdk | talking to real servos and the AHRS, from a laptop. **Not** how you install on the bike: the base dependencies are unconditional, so this drags MuJoCo along with it |

## Drive it

```sh
mjpython -m aow_sim.run_drive --teleop
```

**`mjpython`, not `python`, for anything with an interactive viewer** (macOS).
Headless tools (`record`, `rollout_move`, the trainers) run under plain
`python` anywhere, including over ssh.

It drives like an RC bike:

| key | what it does |
|---|---|
| `↑` / `↓` | throttle / brake into reverse. Tap to step 0.25 m/s, hold to build, release to coast back to zero |
| `←` / `→` | turn. Heading is a *setpoint* — it stays where you leave it |
| `1` / `3` | **crab left / right** — sideways, heading held |
| `6` / `7` / `8` | snap the heading 90° left / 90° right / 180° |
| `5` | stop now, laterally as well |
| `,` | policy menu — pick which trained policy is driving, live |
| `\` | camera: free → follow → overhead → rear wheel |

Bindings are digits, arrows and punctuation to stay clear of the MuJoCo
viewer's own A–Z defaults. Every mode, the ground dial, the policy menu, slow
motion, a game controller and the self-righting layer:
**[teleop.md](teleop.md)**.

**The real bike takes the same commands.** The operator sends a whole command
(velocity vector, heading, mode) over UDP:

```sh
mjpython -m aow_sim.run_drive --mirror aowbike.local
```

Same key map, no simulated physics: every pose in the viewer is telemetry
coming back from the bike. Bring-up and the onboard side are in
[physical-build.md](physical-build.md).

Other entry points:

```sh
python -m aow_sim.view --training-wheels    # open-loop viewer, bike propped up
python -m aow_sim.run_drive                 # headless: sprints, circles, envelopes
python -m aow_sim.record --help             # render a policy to video
```

## Train a policy

The bike's control policies train on GPU: MuJoCo Warp and
PufferLib 5.0, 72-80M steps in 12-90 minutes depending on the card.
A sample script in `scripts/gpu.sh` runs it end to end.
Recommended: 5090 (3090 and 4090 ok too; we need speed, not VRAM).

Earlier policies were trained on CPU (Stable-Baselines3 PPO):

```sh
pip install -e '.[rl]'
python -m aow_sim.train_general_rl          # -> moves/general_rl.{yaml,npz}
```

**[training.md](training.md)** covers exporting a mid-run checkpoint,
inspecting a policy, and comparing policies on one grid.

## Repo map

| path | what is in it |
|---|---|
| `config/bike_params.yaml` | the bike: every physical measurement, with units and provenance (`measured` / `tooth-count` / `datasheet` / `design` / `estimate` / `model` / `GUESS`). `GUESS` means "placeholder, still to be identified" |
| `config/onboard.yaml` | the Pi's devices, servo ids and registers |
| `config/rl/` | training configs: algorithm, env, reward weights, domain randomisation |
| `src/aow_sim/` | the parametric model builder (`mjSpec`), procedural contact meshes, viewer, runners and trainers |
| `src/aow_sim/control/` | `drive.py` is the controller that runs everywhere, in sim and on the bike; `general_spec.py` is the observation/action contract training and replay share |
| `src/aow_sim/gpu/` | the MuJoCo Warp / PufferLib training path and its export back to numpy |
| `src/aow_sim/hw/` | the onboard stack: Dynamixel bus, TM151 reader, velocity estimation, the 100 Hz loop, the ground station. Imports without MuJoCo, scipy or torch |
| `src/aow_sim/cad/` | the FeatureScript generators for the Onshape model |
| `src/aow_sim/web/` | the browser demo: the policy in MuJoCo's WASM build |
| `analysis/` | one-off studies that ask a question of existing artifacts and change nothing, grouped by kind; tracked figures land in each folder's `plots/` |
| `moves/` | trained policies: `*.yaml` plus `*.npz` weights |
| `scripts/` | launchers and generators, including the one behind the README's clips ([media.md](media.md)) |
| `tests/` | every file carries a marker saying what invalidates it — `pytest --markers` |
| `docs/guide/` | this page and the others like it |
| `docs/plans/` | design records: the reasoning, the measurements, the options rejected and why |
| `docs/measurements/` | protocol/data pairs — what to measure, how, and the numbers that came back |

`runs/` (training checkpoints and tensorboard logs) and `traces/` (diagnostic
plots and video) are gitignored.
