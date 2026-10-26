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
| **wheel play** (front axle 1.0 mm, headset 1.5 mm, rear axle 2.5 mm, 4 N m/rad; `traces/drivetrain/wheel_play_fit_1026.yaml`) | `build_model`'s optional `wheel_play:` (absent from bike_params) | carried by `put_model` (limited hinges with springs; no kernel); **run on Warp-CPU 2026-10-26** | done: `env.wheel_play`, recorded in the export (below) |
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

## Wheel play: done (2026-10-26)

1. **`env.wheel_play`**, resolved inside `drivetrain_model.from_env_config`
   (`params.with_wheel_play`), so the trainer, `GeneralEnv`, the GPU env
   (built from a `GeneralEnv`) and `export_policy` all take it. Absent or
   null leaves the params' own (none: v3 still hashes to 6446d8fa); a path
   is that file's block; a dict is itself; false strips it.
2. **Recorded**: `move_doc` writes `wheel_play` beside `drivetrain_model`;
   a loaded policy carries it (`pol.wheel_play`) into
   `policy_env_overrides`; `run_vs_sim.sim` (and so `drivetrain_trace.py`)
   and teleop's startup plant build it from the move. `--set wheel_play=...`
   and `--wheel-play` still win. A policy without a record is judged on the
   caller's plant, so the pointer can still be flown with play switched on.
3. **CPU smoke**, `GeneralEnv`, 3000 steps, actions random / zero
   alternating (~130 falls each), one run each:

   | config | nq / nv | peak nefc | peak ncon | limit rows | ms / env step |
   |---|---|---|---|---|---|
   | v3 | 21 / 20 | 39 | 4 | 0 | 1.66 |
   | v4 | 26 / 25 | 39 | 5 | 4 | 1.95 |

   Under those falls the play hinges pass their walls by 50-80 % (headset
   +-0.30 deg range, seen -0.47..+0.41; rear axle +-1.40, seen +-1.6): the
   2 ms walls are soft. Not checked under a policy that stays up.
4. **Warp-on-CPU smoke** (`fly_policy.py --config config/rl/rl_general_v4.yaml`,
   v3_s0, 8 worlds x 250 steps): finite, 1 fall (C MuJoCo 0), reward/step
   2.60 (C 2.69); peak nefc 44 of `njmax` 128, 57 contacts in all for 8
   worlds. 24 policy steps/s against v3's 35 on the same device: Warp-on-CPU
   is not the GPU, so measure the pod's rate before reading anything into it.

## The run

1. **Written**: `config/rl/rl_general_v4.yaml`, `rl_general_v3.yaml` plus,
   inline, the sawtooth overlay (Coulomb 0.15 as fitted, roller spring 0.01)
   and the wheel play fit. `k_roller` and the onboard revert LEFT (user,
   10-26): today's pair, +2.4 with drive_a on servo 102, is consistent on
   the bike (`onboard.yaml`, measured 10-24), and every policy so far
   trained on +2.4, so flipping it now would mean re-flipping onboard.yaml
   to fly old and new policies back to back. Coulomb stays 0.15 (user).
2. **Done**: CPU and Warp-on-CPU smokes on that config (above).
3. **Ran 2026-10-26**: one 80M run on a secure A40 (no 4090 / 5090 to be
   had; 4 seeds under MPS were ~3.3 h each and were stopped):
   `general_puffer_a40_v4_{52,72,80}M`, `results_a40_v4/`, details in
   `gpu-training.md`. Training ended at fall 0.10 (the v2 run: 0.03).
4. Judge here, with the same plant switched on: `per_command.py --summary
   --ahrs tm151_filter` and `standstill.py` DONE (as good as the pointer;
   standstill drifts less than it on either plant, and about the same
   with the play as without), `chatter.py` and `run_vs_sim.py`
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
