# config/

What the simulator, the trainers and the CAD generators read. Each table is
sorted newest first by the date the file was added. `git log -- <file>` has
the full history; this index only says what each file is FOR.

| folder | holds |
|---|---|
| `config/` | the plant: `bike_params.yaml` (the bike in its current form, what trains and renders it; `source:` on every physical number), `ahrs_mount.yaml`; `onboard.yaml` (the Pi's devices, servo ids and registers; merged in at `control.onboard`) |
| `studies/` | blocks that are not the current bike, merged in by `load_params` at their old paths and left out of `plant_digest`: `analytic_control.yaml` (PD, LQR, its drive, flip, pivot), `righting_mechanisms.yaml` (the never-built roof, bumpers, arm, wing pairs), `test_rigs.yaml` (testbed, training wheels), `ball.yaml` (the ball move) |
| `rl/` | training configs, one per run lineage; `rl_<move>.yaml` is that move's default (`scripts/rl.sh`, the envs) |
| `cad/` | inputs to `python -m aow_sim.cad.<name>`, and `onshape.yaml` (document and tab ids) |
| `linkage/` | righting-mechanism geometry the model, teleop and CAD load |
| `bench/` | bench and test-rig geometry |
| `plays/` | command sequences for `control/command_play.py` |
| `*/archive/` | superseded, kept so old runs, moves and figures stay reproducible |

**What "current" means.** The same rule as `moves/archive/`: a config stays out
of `archive/` if a move at the top level of `moves/` was trained on it, if code
loads it by default, or if it is the live GPU lineage. Everything else is
archived — moved, never deleted, and every file keeps its name.

**Old paths.** Until 2026-10-21 every file here sat flat in `config/` (and two
in `config/swing_explore/`). A path written down before then — `graph.config`
in a `moves/*.yaml`, a GPU checkpoint's `aow_bike_*.json`, a `runs/*/logs`
line, a DEVLOG entry or a commit message — names the same file, now in one of
the folders above. File names are unique across `config/`, so
`find config -name <file>` finds it. `python -m aow_sim.gpu.export_policy`
takes `--config` for a graph exported before the move.

The `rl/` and `rl/archive/` yaml files were moved byte-for-byte, so a
`config_sha256` recorded in a GPU move compares exactly as it did before the
move. (The `_style` and `_style_mult` moves match their files. The
`rl_general_v3` moves do not, because the file was edited on 2026-10-17 after
they were exported.) The header comments in these files still name their
siblings by the OLD `config/<file>` paths, on purpose.

## rl/

| file | added | produced (moves) |
|---|---|---|
| `rl_general_v3_style_mult.yaml` | 2026-10-18 | `general_puffer_mult_*` (archive) |
| `rl_general_v3_style.yaml` | 2026-10-17 | `general_puffer_style_*` (archive) |
| `rl_general_v3.yaml` | 2026-10-14 | `general_puffer_{fixes,next,seed1}_*` (`control.general_move` is `general_puffer_fixes_72M`), `general_rl_v3_s1` |
| `rl_general_v3_mingru.yaml` | 2026-10-14 | `general_rl_v3_mingru_s0` |
| `queue_general_v3.txt`, `queue_general_v3_mingru.txt` | 2026-10-14 | the v3 seed queues (`rl.sh queue general <file>`) |
| `rl_general_smooth_temporal.yaml` | 2026-10-01 | `general_rl_smooth_temporal` |
| `rl_general.yaml` | 2026-08-02 | the `general` default: `GeneralEnv`, `train_general_rl`, most of `analysis/` |
| `rl_ball.yaml`, `rl_pivot.yaml` | 2026-07-21, 08-02 | `ball_rl`, `pivot_rl` |
| `rl_flick.yaml` | 2026-07-20 | `flick_rl` |

## rl/archive/

| file | added | produced (moves, all in `moves/archive/` or never exported) |
|---|---|---|
| `rl_general_nodiff.yaml` | 2026-10-09 | `general_rl_nodiff_2` (a one-off probe) |
| `rl_general_smooth_{base,bound,second}.yaml`, `queue_smooth_loss.txt` | 2026-10-01 | the smooth-loss arms |
| `rl_general_drivetrain_p{100,400}.yaml`, `queue_drivetrain_gains.txt` | 2026-09-14 | `general_rl_drivetrain_p{100,400}_{0,1,2}` |
| `rl_general_cmd_curriculum{,2,2b}.yaml` | 2026-08-30 | `general_rl_cmd_curriculum{,2,2b}` |
| `rl_general_odo_ahrs_pitch{,_w}.yaml` | 2026-08-29 | `general_rl_odo_ahrs_pitch{,_w}` |
| `rl_general_odo_ahrs_{rand,rand2,tau019}.yaml` | 2026-08-28 | `general_rl_odo_ahrs_{rand,rand2,tau019}` |
| `rl_general_odo.yaml`, `rl_general_odo_ahrs.yaml` | 2026-08-26 | `general_rl_odo`, `general_rl_odo_ahrs` |
| `rl_general_smooth_diff_nolat.yaml` | 2026-08-26 | — |
| `rl_general_swing{,_open}.yaml` | 2026-08-25 | `runs/general_swing{,_open}` |
| `rl_general_pitch_smooth_diff.yaml` | 2026-08-22 | `general_rl_pitch_smooth_diff_pi` |
| `rl_general_glide_pitch_{hub,hub2,hub3,smooth}.yaml`, `rl_general_hold{,_nohub}.yaml` | 2026-08-21 | the glide/hold series |
| `rl_general_wings.yaml` | 2026-08-15 | `runs/general_wings*` |
| `rl_general_glide{,_pitch}.yaml` | 2026-08-10 | `general_rl_glide_og`, `general_rl_glide_pitch_{og,dt4e4}` |
| `rl_general_smooth_diff.yaml` | 2026-08-08 | `general_rl_smooth_diff_{og,pi}` |
| `rl_general_smooth.yaml` | 2026-08-07 | the first smoothness runs |
| `rl_ball_unhacked.yaml`, `rl_flick_oldsteer.yaml` | 2026-08-02 | — |
| `rl_ball_base.yaml` | 2026-07-21 | — |

## cad/

| file | added | read by |
|---|---|---|
| `bike_v2_cad.yaml` | 2026-10-11 | `cad.bike_v2`, `cad.power`, `cad.pi_carrier` (the current bike) |
| `drop_rig_cad.yaml` | 2026-10-02 | `cad.drop_rig` |
| `bike_cad.yaml` | 2026-09-30 | `cad.bike` (v1 whole bike) |
| `righting_cad.yaml` | 2026-09-29 | `cad.righting`; `bike_params.righting.linkage_cad` |
| `wing_linkage_metal_cad.yaml` | 2026-09-29 | `cad.wing_linkage_metal` |
| `drive_cad.yaml`, `steering_cad.yaml` | 2026-09-28 | `cad.drive`, `cad.steering` |
| `x330_fixture_cad.yaml` | 2026-09-25 | `cad.x330_fixture` |
| `ahrs_fixture_cad.yaml` | 2026-09-23 | `cad.ahrs_fixture` |
| `servo_mounts.yaml` | 2026-08-25 | `cad.servo_mount` and every generator that mounts a servo |
| `onshape.yaml` | 2026-08-21 | `cad.onshape`: document, workspace and tab ids |
| `bike_params_cad.yaml` | 2026-08-18 | `cad.layout` and the generators: CAD-only dimensions |

## linkage/

| file | added | read by |
|---|---|---|
| `blade_v3.yaml` | 2026-10-10 | `bike_params.righting.blade_file` (the bike's blade) |
| `righting_blade.yaml` | 2026-10-09 | the V2 blade; `analysis/mechanism/swing_stepthrough.py` |
| `swing_linkage_shared_rod.yaml` | 2026-09-27 | `cad/righting_cad.yaml`, `cad/wing_linkage_metal_cad.yaml` (the V2 diamond) |
| `swing_linkage_smaller.yaml`, `swing_linkage_flat.yaml` | 2026-09-03 | `build_model` / teleop / `hw.ground` (V1); `cad.swing_linkage` example |
| `swing_linkage.yaml`, `swing_wings.yaml` | 2026-08-25 | `analysis/mechanism/swing_linkage.py` default; `build_model` swing wings |
| `wing_linkage_w75.yaml` | 2026-08-18 | `record`, `run_drive` |
| `wing_linkage.yaml`, `wing_linkage_locking.yaml` | 2026-08-15 | `analysis/mechanism/wing_linkage.py`; `build_model.LINKAGE_CFG` |

`linkage/archive/`: the swing-linkage variants from the September design
rounds (`_compact`, `_constraints*`, `_margin`, `_opt`, `_smaller_v2`,
`_stagger*`, `_tight`, `_vertical`), `wing_linkage_optimized.yaml` and
`diamond_bothout.yaml`. Several still regenerate tagged figures under
`analysis/mechanism/plots/` (`--config config/linkage/archive/<file> --tag _<x>`).

## bench/, archive/

| file | added | read by |
|---|---|---|
| `bench/drop_rig_bench.yaml` | 2026-10-03 | `bench/force_drop.py`, `bench/wave_run.py`, `analysis/contact/drop_rig_sim.py` |
| `bench/floor_rig.yaml` | 2026-09-25 | `floor_rig.py`, `run_drive` |
| `archive/flywheel.yaml` | 2026-08-25 | `analysis/mechanism/flywheel.py` only (`build_model(flywheel=True, flywheel_cfg=...)`) |
