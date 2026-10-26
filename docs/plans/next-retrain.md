# The next training run: every model change since the pointer

Started 2026-10-26. The point (user): put ALL of the recent modelling into
the next run -- the drivetrain as re-measured, the roller modelling, and the
wheel play that reproduces the bike's 9-11 Hz floor shake -- and see whether
a policy trained on it stops shaking on the bike. Evidence:
`physical-bike-tests.md` (tests 1-3, item 8 "Queued for the next retrain",
item 9), and test 2's "Wheel play reproduces it" / "Rough fit".

The run goes on the GPU stack (`gpu-training.md`, phase 3; `scripts/gpu.sh`)
and is judged here in C MuJoCo, as every GPU run is.

## The changes, and where each stands on the GPU

| change | in the model | on the GPU | to do |
|---|---|---|---|
| v2 bike, `steer_rate_encoder: counts` | bike_params; `rl_general_v3.yaml` | trained (`general_puffer_rake20_*`) | nothing |
| **sawtooth detent**: flat friction 0.12 N m, sawtooth push 0.12, `kinetic_fraction` 0.15 (fitted GUESS) | `drivetrain_model` options (ea12a85), `traces/drivetrain/drivetrain_sawtooth_1024.yaml` | **ported 2026-10-26** (`gpu/drivetrain.py`: hill sine/sawtooth, flip ramp, offset, kinetic, handed scale; per-world `sliding` state, zeroed at reset) | put the overlay INLINE in the config (traces/ is not in the pod tarball) |
| **roller slop with the measured spring** (~1e-2 N m/rad from the flick video, vs the 1e-3 GUESS) | `roller_slop` tendons; the sawtooth file carries 0.01 | carried: tendons and limits are model structure, `put_model` takes them | same inline overlay; check chatter after (x10 cut `smooth_temporal`'s chatter score 0.818 -> 0.598 in `slop_sensitivity.py`) |
| drivetrain Coulomb as re-measured (coast: ~0.16-0.18 common, 0.114 diff per servo, sides equal) | the overlay still says `coulomb_friction` 0.15 | carried (a constant) | decide the value; edit the inline overlay |
| `k_roller: -2.4` with the `onboard.yaml` A/B swap reverted, in ONE change | bike_params + onboard | carried (a constant) | do it; either half alone breaks the sideways sign. Moves plant_digest, so `export_deploy` + `bike.sh sync` |
| **wheel play** (front axle 1.0 mm, headset 1.5 mm, rear axle 2.5 mm, 4 N m/rad; `traces/drivetrain/wheel_play_fit_1026.yaml`) | `build_model`'s optional `wheel_play:` (absent from bike_params) | carried by `put_model` (limited hinges with springs; no kernel) -- by reading, not yet run | the env key and the export record (below); check `njmax` |
| AHRS mount from test 5 | not run | -- | skip unless test 5 runs |

**Checks run for the detent port** (Warp 1.18 on CPU, `aow-sim-gpu`):

- `analysis/gpu/phase2/check_detent.py` -- new: the kernel against the Python
  `DrivetrainSim` on the same synthetic shaft trajectories, 4 worlds:

  | overlay | sliding | motor torque, max rel err | applied force, max rel err |
  |---|---|---|---|
  | bike_params' friction bump (`--overlay default`) | 33 % | 2.2e-6 | 4.3e-4 |
  | sawtooth 1024 | 99 % | 2.2e-6 | 4.9e-4 |
  | sawtooth, slow (`--wobble 0 --cycles 0.3 0.6`, 8 s) | 99 % | 8.4e-6 | 1.5e-3 |
  | + flip 1.5 deg, minus scale 1.15, offset 0.02 N m | 99 % | 8.4e-6 | 2.2e-3 |

  The flat friction with kinetic 0.15 slips nearly always once moving;
  the 1 % that sticks is the reversals. Float32 against float64.
- `check_drivetrain.py` on the recorded phase-2 flight (friction bump):
  motor 0.16 %, applied 3 % -- the phase-2 numbers (`gpu-training.md`,
  "float32, below"), so the bump path is unchanged. The six phase-2 checks
  needed their recorded config path mapped to `config/rl/` (moved 10-21).
- `check_reset.py`: unchanged with the new `sliding` argument.
- The whole `GpuGeneralEnv` with the sawtooth overlay, 4 worlds, 20 steps:
  finite observations, all four worlds' friction slipping.

## Wheel play: what is left

1. **An env key, mirrored on `drivetrain_model`.** `env.wheel_play` in an
   RL config: absent = none (every config so far reproduces bit for bit),
   a path = that file's `wheel_play:` block, a dict = a resolved record.
   Resolved in `GeneralEnv.__init__` beside `drivetrain_model.from_env_config`,
   so the CPU trainer and the GPU env both get it. Test: absent leaves the
   model and the RNG stream identical (the pattern of
   `test_absent_ranges_leave_the_rng_stream_untouched`).
2. **Record it in the export.** `train_general_rl.py` (~line 346) and
   `gpu/export_policy.py` write `drivetrain_model` into the move yaml; write
   `wheel_play` beside it, and have `run_vs_sim.sim`, teleop's startup plant
   and `drivetrain_trace.py` read the move's own record, so a policy is
   always replayed on the plant it trained on.
3. **CPU smoke**: `GeneralEnv` with the fit, a few thousand steps, play
   joints inside their limits; peak `nefc` / `ncon` against the GPU's
   `njmax 128` / `nconmax 32` (each hinge at a wall is a limit row).
4. **Warp-on-CPU smoke**, as above for the detent.

What does not carry: per-world RANDOMISATION of the play (`BATCHED` in
`gpu/env.py` holds mass, friction, force/ctrl range, frictionloss only;
`jnt_range` / `jnt_stiffness` would need batching and a reset draw).
Fixed play first. Speed: nv 25 -> 30, expect a few %; measure.

## The run

1. One config, `config/rl/rl_general_v4.yaml` (say): `rl_general_v3.yaml`
   plus, inline, the sawtooth overlay with the chosen Coulomb and the
   measured roller spring, and the wheel play fit. `k_roller` and the
   onboard revert landed first, with `export_deploy`.
2. CPU and Warp-on-CPU smokes on THAT config.
3. ONE 72M pod run (`scripts/gpu.sh`). Confirm GPU type and $/h first.
4. Judge here, with the same plant switched on: `per_command.py --summary
   --ahrs tm151_filter`, `standstill.py`, `chatter.py`, and `run_vs_sim.py`
   against 174349 -- the 7-12 Hz band and the badges on
   `drivetrain_traces.html`. The bar: shakes markedly less than the pointer
   does on the same plant (15.1 deg/s roll rate 7-12 Hz RMS with the play
   fit), without giving up falls or standstill.
5. Fly it on the bike against the pointer, no inputs, 50 s.

## Choices on the way

- **Where the wheel play lives once adopted.** Today an env key and a
  traces file. If a play-trained policy becomes the pointer, the play is
  part of the bike and belongs in `bike_params.yaml` (`source: model`,
  "fit to floor runs 174349 / 225207"): plant_digest moves, `export_deploy`,
  and the qpos addresses `DeployModel` carries shift (the re-export covers
  it; check `hw/state.py`'s nq/nv against the new bundle).
- **The chainstays** (`rear_stays`) stay out: they do not flex that much
  (user). Back in only for a suspension study, mostly `travel_mm_vertical`.

## Open

- The steer servo's own shaft moves 2x the sim's at 7-12 Hz (3.5x above
  25 Hz) for a matching target (`physical-bike-tests.md`, "Logged alike").
  (An earlier "the sim's roller rate peaks at 4.3 Hz" was a logging
  artifact, withdrawn.)
- The bike sways ~2x more than the sim below 3 Hz (roll rate, diff, steer
  target), unexplained by any play tried.
- The sawtooth's kinetic fraction 0.15 is a fitted GUESS; the slow check
  above shows it keeps the flat friction slipping almost always once moving.
