# moves/: what it holds, who reads it, and a layout that says which is which

> **Status, 2026-10-16: done, layout C with the archive TRACKED.** §1-3
> are the survey as it was before the move; §4 the three layouts proposed;
> §5 what was decided and built. The proposal recommended A; the user
> picked C.

Evidence tags as in `general-rl-improvements.md`: **[measured]** (counted
or run), **[read]** (code read), **[reasoned]**.

---

## 1. What is there (2026-10-16) [measured]

92 moves (91 `.npz` + `.yaml` pairs, `flick` is a trajectory yaml), 9.4 MB,
flat, plus `moves/stale/` (one `ball_rl` from 2026-07-21). 147 files
tracked, 38 untracked.

| kind | count | tracked | plant_digest | examples |
|---|---|---|---|---|
| early moves, before the digest field | 34 | yes | none | `flick_rl*`, `pivot_rl*`, `ball_rl`, `general_rl`, `general_rl_glide_og`, `general_wings*_rl` |
| general arms, Aug-Oct | 27 | yes | 5 older digests | `general_rl_odo_ahrs*`, `general_rl_cmd_curriculum*`, `general_rl_smooth_*`, `general_rl_drivetrain_p{100,400}_{0,1,2}` |
| one study's 12 seeds | 12 | yes | e1ec36bf | `personality0..11` |
| CPU v3 seeds + MinGRU arm | 6 | **no** | **bc66ca20 (today's)** | `general_rl_v3_s0..s4`, `general_rl_v3_mingru_s0` |
| GPU checkpoints | 13 | **no** | **bc66ca20** | `general_puffer_{1h,next,fixes,seed1}_{13..72}M` |

**Every move trained against today's plant is untracked, and every tracked
one is from an older plant.** The teleop default
(`config/bike_params.yaml: control.general_move: general_rl_smooth_temporal`)
carries digest 9953690d, not bc66ca20. [measured: `params.plant_digest`]

## 2. Who reads it [read]

One loader, so a layout change has a narrow waist:

- `control/flick.py`: `MOVES_DIR = <repo>/moves`, `load_move(name)` reads
  `MOVES_DIR/<name>.yaml`, then `policy_file` **relative to that yaml's
  folder**. So `load_move("general/v3/s0")` already works with
  subfolders; nothing assumes flat except the callers below.
- `policy_menu.py` (teleop's `,` menu): `MOVES_DIR.glob("*.yaml")`,
  `kind: general` only, newest mtime first, one line from the yaml's
  `trained.metrics` -- **the incomparable numbers** (CLAUDE.md). Flat only.
- `hw/bike_bus.py`: `policy_servo_gains(name, moves_dir="moves")` reads
  `<name>.yaml` on the Pi -- a copied tree, so whatever is in `moves/`
  is shipped (9.4 MB now).
- `drivetrain_model.policy_record/policy_params`: by name, `moves_dir`.
- Trainers (`train_{general,flick,pivot,ball}_rl.py`): `--export-name NAME`
  writes `moves/NAME.{npz,yaml}`; `gpu/export_policy.py --name` the same.
- `scripts/rl.sh`: refuses a queue entry whose `moves/<name>.yaml`
  exists; `sync` byte-checks untracked moves against the incoming commit
  (it exists because finished runs leave untracked moves on the box).
- By name in tests and configs: ~80 files mention `moves/` or MOVES_DIR;
  the most-pinned names are `flick` / `ball_rl` / `general_rl` /
  `general_rl_odo_ahrs` / `general_rl_smooth_diff_pi` /
  `general_rl_cmd_curriculum2b` (10-48 files each, counts inflated by
  word matches). `tests/test_general_rl.py` alone uses MOVES_DIR 12 times.

## 3. What goes wrong with it now

1. **A name is the only metadata.** Seed, arm, checkpoint step, CPU or GPU,
   trainer, whether it was meant for the bike -- all in an ad-hoc suffix
   (`_s3`, `_52M`, `_og`, `_2`, `_prev`). Nothing tells a curve-only
   checkpoint from a candidate.
2. **"Current" is invisible.** Which policies match today's plant takes a
   digest check per file; the answer (19, all untracked) is not written
   anywhere, and the teleop default is from an older plant.
3. **The comparable result lives nowhere next to the move.** `per_command`
   and `chatter` numbers are in commit messages and plan docs; the yaml's
   own `metrics:` block is the one CLAUDE.md says not to compare -- and it
   is what the teleop menu shows.
4. **Provenance is split.** A GPU move's config, graph meta and PufferLib
   ini are in `analysis/gpu/phase3/results_*/checkpoints/`; a CPU move's
   run dir is in `runs/` (git-ignored). The yaml has a config path and
   hash but no pointer to either.
5. **Tracking is undecided per file.** Untracked moves are "your call"
   in every commit draft; 38 are waiting. The rule that decides it does
   not exist yet.
6. **Everything ships to the Pi**, including 12 personality seeds and 13
   GPU checkpoints.

## 4. Three layouts

**A. Flat + an index (smallest change).** Keep `moves/<name>.*`. Add
`moves/index.yaml`, one entry per move:

```yaml
general_puffer_seed1_72M:
  status: candidate          # current | candidate | reference | curve | archive
  family: general
  trainer: puffer            # sb3 | puffer | trajopt
  run: puffer_seed1          # groups checkpoints and seeds
  step: 72M
  provenance: analysis/gpu/phase3/results_a40_seed1/checkpoints/
  grid:                      # the COMPARABLE numbers, with how they were made
    per_command: {turns_fwd: 7/9, straight: 2/2, fell: 0}
    chatter: {score: 0.875, d_action: 0.008}
    sensors: tm151_filter tau 2.0, counts
    date: 2026-10-16
```

`analysis/move_index.py` (new) rebuilds the digest column, flags entries
whose digest is not today's, and `--grid NAMES` re-runs `per_command` +
`chatter` and writes their numbers in -- so the index is regenerated, not
hand-kept. The teleop menu reads `status` and `grid` instead of
`trained.metrics`. Old names keep working everywhere. Costs: one file and
one script; touches `policy_menu.py`. Does not fix "everything ships".

**B. Per run folders.** `moves/<family>/<run>/<step or seed>.{npz,yaml}`
plus `run.yaml` (config, commit, seeds, where the run dir / checkpoints
are), e.g. `moves/general/puffer_fixes/52M`, `moves/general/v3/s0`.
`load_move("general/v3/s0")` works today [read]; needed: `policy_menu`
to `rglob`, the trainers' `--export-name` to accept a path, `rl.sh`'s
exists-check unchanged (it builds the same path), and **every pinned name
in tests/configs renamed** -- or an alias table for the old flat names.
Costs most; old names in DEVLOG, commit messages and plan docs go stale
unless the alias table stays forever.

**C. Split by purpose, on disk.** `moves/` = tracked, shipped, only
`current` + `candidate` + a few `reference`; everything else
(curve checkpoints, seed sweeps, archive) to a git-ignored, Dropbox-synced
`moves_archive/` (like `runs/`), loadable by passing `moves_dir`.
Fixes the Pi payload and the "your call" pile; a name moving between the
two folders is a move, not a rename.

## 5. Decided and built (2026-10-16)

**A was dropped.** The user's objection: an index rehashes what every move
yaml already carries (digest, config, trainer, metrics), so it is a second
copy to keep in step, not new information. The one thing it would have
added, a comparable grid number, is a re-run of `per_command.py` either
way.

**C, but tracked.** `moves/archive/` is in git like the top level (8.0 MB of
9.4); untrack it later if it balloons. So nothing about tracking or the Pi
payload changed. What changed is what the top level and the teleop menu show.

- **Top level, 12 moves.** The pointer `general_puffer_fixes_72M` (also new
  on 10-16), today's GPU finals `fixes_52M` / `seed1_72M` / `next_72M`,
  `general_rl_v3_s1`, the previous pointer `general_rl_smooth_temporal`, and
  the names teleop's keys and the tests load by path: `flick`, `flick_fwd`,
  `flick_rl`, `ball_rl`, `pivot_rl`, `general_rl`.
- **Then the CPU MinGRU arm (10-16, 13 on top).** `queue_general_v3_mingru`
  finished on the box: `general_rl_v3_mingru_s0` on top as the arm's
  representative, `s1..s4` in the archive. Picked on its own export metrics,
  which are comparable within the arm (same queue, code and grid): s0 is the
  only seed with low drift (0.07 m), head error 25 deg AND crab both ways.
  s4 tracks best (0.708, 12.6 deg) but barely crabs (right 0.014). On one
  common `per_command.py --summary` grid (tm151_filter, counts) all five and
  `v3_s1` fell 0/20; turns resolved forward 5-7/9 vs `v3_s1` 8/9.
- **Archive, the other 80** (161 files, plus the orphan
  `general_rl_hold_nohub.npz`). `moves/stale/` is untouched.
- **Names still work everywhere.** `control.flick.move_path` falls back to
  `archive/` when `moves/NAME.yaml` is missing. `load_move`,
  `drivetrain_model.policy_record`, `record.py`, teleop's startup read and
  `hw/bike_bus.policy_servo_gains` all go through it (the last inline, since
  `hw/` may not import `control/`). Every name in configs, analysis scripts,
  commits and docs stays valid.
- **New exports can't take an archived name.** `reserve_move_name` and
  `rl.sh`'s queue check both look in `archive/` too.
- **`rl.sh sync`** compares an untracked `moves/X` on the box against
  `moves/archive/X` in the incoming commit, so a move trained on the box and
  archived here is reconciled rather than left as a stray.
- **Tests** that sweep every move (obs width, AHRS declaration) glob both
  folders.

Retiring or reviving a move is now a `mv` (then the commit), never a rename.
