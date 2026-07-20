# aow-bike-sim

MuJoCo simulator for an active omni wheel RC two-wheeler.

The rear wheel is an active omni wheel (8 driven axles, 16 truncated-cone
rollers) scavenged from a Hui Can HC-802 / Ducati Upriser; two Dynamixel
XC430-W150 servos drive it through the toy's gearbox, an XC330-T181 steers
(continuous 360°+), and a TM151 AHRS measures orientation. Goal: balance →
maneuvers → tricks.

## Quickstart

```sh
conda create -n aow-sim python=3.12 -y && conda activate aow-sim
pip install -e '.[dev]'

pytest                                          # model correctness tests
python -m aow_sim.view --training-wheels        # interactive viewer (open loop)
python -m aow_sim.view --variant testbed        # wheel-only system-ID rig
python -m aow_sim.build_model -o model.xml      # dump MJCF

python -m aow_sim.run_balance                   # balance metrics (LQR baseline)
python -m aow_sim.run_balance --controller pd   # PD cascade reference (legacy)
python -m aow_sim.run_balance --view            # watch it balance; shove it with
                                                #   double-click + Ctrl+right-drag

python -m aow_sim.run_pivot                     # crawl-pivot metrics + rate sweep
python -m aow_sim.run_pivot --view              # +180° / −180° pivot demo
mjpython -m aow_sim.run_pivot --teleop          # drive the heading with the keyboard

python -m aow_sim.run_drive                     # sprints, accel sweep, circle+flip+flick envelopes
python -m aow_sim.run_drive --view              # sprint + circle + stop + flip + flick demo
mjpython -m aow_sim.run_drive --teleop          # RC-style driving (number keys — MuJoCo's viewer
                                                #   owns the letters): ↑/↓ speed, ←/→ heading,
                                                #   6/7 circle, 8/9 flick, 4 flip, 5 stop, 2 overlay

python -m aow_sim.optimize_flick --reverse-first          # (offline) optimize the two-arc 180 flick
python -m aow_sim.optimize_flick --name flick_fwd         #   -> moves/<name>.yaml; never touches config

pip install -e '.[rl]' && python -m aow_sim.train_flick_rl   # (offline) RL policy for the flick,
                                                             #   an alternative to the scipy optimizer
                                                             #   -> moves/flick_rl.{yaml,npz}; replay is
                                                             #   numpy-only (no torch needed to run it)
                                                #   (←/→ ±30°, J/L ±90°, U/O ±180°)
```

In the open-loop viewer's Control panel: equal `drive_a`/`drive_b` rolls the
bike, differential input crawls the rear wheel sideways, `steer` is continuous.

Balance baseline (placeholder chassis params): both controllers hold the bike
indefinitely from a 3° lean with <0.1° RMS wobble, <10 cm drift, and recover a
4 N × 0.1 s lateral shove. Gains/weights live in `config/bike_params.yaml`
under `control:`.

## Layout

- `config/bike_params.yaml` — **single source of truth**: every measurement,
  with units and provenance (`measured` / `tooth-count` / `datasheet` / `GUESS`).
  Current values are placeholders; replacing them is the whole measurement task.
- `src/aow_sim/` — parametric model builder (`mjSpec`), procedural contact
  meshes, viewer.
- `docs/measurements/omni-wheel-protocol.md` — what to measure and how,
  including the testbed calibration experiments.
- `docs/plans/mujoco-modeling-decisions.md` — why the model is built this way.
- `tests/` — compilation, coupling-ratio, envelope, and behavior tests.
