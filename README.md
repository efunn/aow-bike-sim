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
python -m aow_sim.view --training-wheels        # interactive viewer
python -m aow_sim.view --variant testbed        # wheel-only system-ID rig
python -m aow_sim.build_model -o model.xml      # dump MJCF
```

In the viewer's Control panel: equal `drive_a`/`drive_b` rolls the bike,
differential input crawls the rear wheel sideways, `steer` is continuous.

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
