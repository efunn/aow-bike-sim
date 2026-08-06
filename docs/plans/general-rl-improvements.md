# Ideas for general RL policy improvements

Candidate work on `general_rl` — the always-on command-conditioned controller
(`control/general_spec.py`, `control/general_env.py`, `train_general_rl.py`).
Collected 2026-08-05/06 while benchmarking a 10M-step run on the 32-thread
remote box. Nothing here is committed to; the training loop works and the
next step is benchmarking it, not rewriting it.

Each item is tagged with how much evidence is behind it:

- **[measured]** — a number or a code read backs it
- **[reasoned]** — follows from measured facts, not directly observed
- **[speculative]** — a hunch worth testing before acting on

## Context: where the policy stands

The current export (`moves/general_rl`, snapshot at 6M of 10M steps, selected
by `BestByScore`) survives every eval command and holds heading tightly, but
**will not drive backwards** in teleop. Its predecessor (`general_rl_og`,
final 10M checkpoint) reversed but tracked heading four times worse.

| export | vel_err | head_err | survive | track | score |
|---|---|---|---|---|---|
| `general_rl` (6M snapshot) | 0.291 | 2.5° | 1.00 | 0.763 | 0.763 |
| `general_rl_og` (10M final) | 0.225 | 9.9° | 0.92 | 0.702 | 0.644 |

Velocity tracking got *worse* while heading and survival improved — the
signature of a policy that stopped attempting risky commands, and `_score =
survive_rate × track` preferred exactly that trade.

## 1. Reverse refusal

### Why it happens

**Not the curriculum** **[measured]** — `_sample_command` draws `v_lon_w` from
`uniform(-v_lim, v_lim)` at *every* difficulty, so ~40% of commands are
reverse from `_diff = 0` onward. Only the magnitude is gated:
`v_lim = v_max·(0.25 + 0.75·d)`, so the eval's −0.5 m/s reverse only enters
the training distribution once `d ≳ 0.22`. Gentle reverse is always trained.

**The reward makes refusing rational** **[reasoned]**. The velocity term is
`w_vel · exp(−Δv²/σ_v²)` with `σ_v = 0.35`, so ignoring a −0.5 m/s command
still collects `exp(−2.04) ≈ 13%` of it — refusing forfeits ~1.3 reward/step,
and only while a reverse command is active. Falling costs `penalty_fall: 50`
*plus* every remaining step of a 15 s episode, which at ~2.5/step exceeds
1000. Break-even is around a **15% fall probability**: if attempting reverse
drops the bike more than about one time in seven, declining scores higher.

Note the implicit asymmetry — the dominant cost of falling is the forfeited
episode, not `penalty_fall`. The explicit knob is ~5% of the real penalty.

**Reverse is genuinely harder** **[reasoned]**. Rolling backwards inverts the
sign of the steer→lateral-acceleration relation that stabilises a two-wheeler.
`sharper-turns-stage-1.md` independently reaches the same conclusion from the
analytic side ("reversed caster degrades the straight-line model fastest").
The rear omni offers an escape a real bike lacks, but the front caster still
flips. A policy that avoids reverse has learned something true about the plant.

### Candidate fixes, cheapest first

1. **Per-command eval breakdown** (see §2) — before changing any reward,
   find out whether the whole run refuses reverse or only the 6M snapshot.
2. **Score per-command competence**, not a blended mean: e.g. `min` over
   commands, or `survive × track × (worst-command track / mean track)`. Stops
   selection from rewarding a policy that abandons 2 of 12 commands.
3. **Raise `w_vel` relative to `w_head`** (currently 1.5 / 1.0). Tilts the
   trade back toward tracking the command it was given.
4. **Widen `σ_v`'s implicit tolerance asymmetry** — or add a small explicit
   term for *sign agreement* between `v_cmd` and `v`, so "moving the wrong
   way / not at all" is distinguishable from "moving slightly too slow".
5. **Reverse-specific curriculum**: decouple the reverse range from the
   forward range so reverse magnitude ramps on its own schedule.
6. Accept it, and let the analytic controller own reverse — the modal layer
   already exists (`,` toggles). **[speculative]** but a legitimate answer if
   reverse balance turns out to be genuinely marginal in this plant.

## 2. Eval and snapshot selection

- **Per-command breakdown in `_eval_episodes`** **[measured gap]**. It
  aggregates 12 commands into four means, so two totally failed commands hide
  inside `vel_err = 0.291`. Printing (and returning) per-command rows would
  have surfaced the reverse failure immediately, and would make
  `--scan-checkpoints` far more useful — scan for a checkpoint that reverses,
  not one with a good mean. **This is the single highest-value change here.**
- **The eval grid is thin where the policy is weakest** **[measured]**:
  12 commands, of which 2 are reverse and 1 is lateral crab. A failure mode
  confined to reverse moves the mean by very little.
- **Eval runs with randomization disabled** **[measured]** — so selection
  optimises nominal-model performance, while untethered transfer depends on
  `randomization.actuator_frac` et al. Consider scoring a randomized eval
  (or both), given that sim-to-real is the point of the domain randomization.
- **Eval cost grows with competence** **[measured]**: episodes run to
  termination, so a good policy costs the full 12 × 750 steps where a falling
  one exits early. Measured at 35.5 s per eval at ~1M steps. With
  `eval_every: 1000000` that amortizes to ~0.6 s/rollout — fine — but it will
  grow as the policy improves.
- **Single seed** **[measured]**: `seed: 0`, one run. Before concluding that
  any hyperparameter change helped, run 2-3 seeds — the reverse/heading trade
  above could plausibly be seed noise rather than a systematic effect.

## 3. Curriculum

- **It is per-env, not global** **[measured]**. Each `SubprocVecEnv` worker
  holds its own `_diff` and advances it on its own episode outcomes, so 32
  bikes sit at 32 different difficulties with no synchronization.
- **It ratchets per episode, so its rate is coupled to `n_envs`**
  **[reasoned]**. At fixed global timesteps, 32 envs end a quarter as many
  episodes per env as 8 did, so difficulty climbs ~4× slower against the
  tensorboard x-axis. Training curves looked similar across the 8→32 change,
  so the practical magnitude may be small — `curriculum/difficulty` (now
  logged) will settle it on the next run.
- **It never regresses** **[measured]** — `_advance_curriculum` only
  increments. A policy that degrades keeps the difficulty it earned.
- Ideas: share difficulty across envs (or advance on a rolling window of
  outcomes rather than a single episode); allow regression; drive difficulty
  from the periodic eval score instead of per-episode score. **[speculative]**

## 4. Resume correctness

*(Both known resume bugs are now fixed — the lexicographic checkpoint sort that
picked `ppo_900000` over `ppo_5000000`, and the `VecNormalize` pairing below.
Kept here as a record of the failure modes.)*

**`VecNormalize` stats were not paired with the checkpoint being resumed.**
`--resume` loaded the newest checkpoint from `checkpoints/ppo_*_steps.zip` but
took obs/reward stats from `runs/<move>/vecnormalize.pkl`, which is only
written *after* `learn()` completes. So resuming a **killed** run found no
top-level pkl and built fresh zero stats under trained weights, while resuming
after an **earlier, longer** run found a *stale* pkl that did not correspond
to the checkpoint at all. Either way the policy was fed a different
observation distribution than it trained on until the running stats recovered
— silent, and easily mistaken for the resume itself being lossy.

`_resume_vecnormalize` now prefers the matched
`ppo_vecnormalize_<steps>.pkl` that `CheckpointCallback(save_vecnormalize=True)`
writes beside every checkpoint, falls back to the top-level file, and warns
loudly when neither exists.

Also on resume: SB3 *adds* the budget to steps already done
(`base_class._setup_learn`), so `--timesteps` is "how many more", not "up to".
Resuming a 1.2M-step run with the config's 10M targets 11.2M.

*(The lexicographic checkpoint sort that made `--resume` pick `ppo_900000`
over `ppo_5000000` is fixed as of `c958ba4`.)*

## 5. Throughput — mostly closed

Measured on the remote box (Threadripper 2950X, 16C/32T, no CPU quota):

| change | effect |
|---|---|
| `n_envs: 8 → 32` (+ `n_steps: 2048 → 512`) | 1077 → 1556 steps/s **[measured]** |
| `eval_every: 200000 → 1000000` | 1556 → 1839 steps/s **[measured]** |
| `batch_size: 256 → 1024` | ~+13% projected (update 1.89 s → 0.93 s) **[measured, untried]** |

Dead ends, all tested: `OMP_NUM_THREADS` (1531 vs 1556 — noise), MKL AVX2
forcing, CPU quota/affinity (none), and the GPU — the installed torch is built
for `sm_75+` and the card is a Pascal 1050 (`sm_61`), so it has no kernels for
it regardless.

**The useful structural fact** **[reasoned]**: collection time and update time
both scale with the rollout buffer, so

```
update / collection  depends only on  n_epochs and batch_size
                     — independent of n_envs and n_steps
```

which is why the `n_envs` change hit an Amdahl wall exactly where it did.

**Current split** **[measured]**: 8.91 s per rollout = 77% collection, 23%
update. Collection reaches 2629 steps/s against a 16-core ideal of 3600
(73%) — a healthy number for 32 workers on 16 physical cores with a
synchronization barrier, with no cliff in the scaling curve. `n_envs: 32` is
already optimal; 24 is within 3%, 16 costs 16%.

**The floor is single-core physics** **[measured]**: `timestep: 2.0e-4` means
100 `mj_step` per 50 Hz action; one env runs 225 steps/s = 4.5× realtime on
the 2950X (9.8× on an M-series laptop, matching the ~11× recorded in
`mujoco-modeling-decisions.md`). Per-bike throughput is a per-core property
that no amount of parallelism changes.

Remaining levers, both costly:

- **Async collection** (APPO / sample-factory style) to overlap stepping with
  the update, reclaiming the 23%. PPO is on-policy, so this makes the data
  slightly off-policy — a different algorithm, not a setting.
- **Cheaper physics** (`timestep`, `mesh_segments: 32`, `condim: 4`,
  `impratio: 10`) — validated modeling decisions with fidelity consequences.
  Out of scope for a throughput exercise.

Tools: `scripts/bench_collect.py` (env scaling sweep) and
`scripts/bench_update.py` (gradient-phase timing). `scripts/rl.sh eta` reports
device, throughput, and projected finish from a live run's log.

## 6. Measurement hygiene (learned the hard way)

- **Benchmark on an idle box.** Both bench scripts write nothing, so they
  can't corrupt a run — but a run corrupts *them*. Timings taken beside live
  training came out 2-8× too slow and non-monotonic across a thread sweep,
  which produced two wrong conclusions before it was caught.
- **This container's `/proc` lies.** `ELAPSED` reads as ~441M days because
  `btime` is bogus, so `ps %cpu` (cputime ÷ elapsed) reports 0.0 for every
  process and `vmstat`'s cpu columns are equally meaningless. Use cgroup v2
  instead: `usage_usec` from `/sys/fs/cgroup/cpu.stat` is a monotonic counter,
  and a delta over a known wall interval gives cores-busy honestly.
- **`nproc` is not core count** in a container (it ignores cgroup quota), and
  `inxi` misreported the 2950X as 8-core.
- **Any new run overwrites `runs/<move>/` artifacts, not just benchmarks.**
  The step counter restarts, so checkpoints are rewritten as the new run
  passes each 100k mark and `best_model.zip` is replaced at its first eval.
  Checkpoints *beyond* the new run's length survive from the old one and mix
  in, which quietly makes `--scan-checkpoints` compare two lineages. Export
  anything interesting to `moves/` with `--export-from/--export-name` first,
  or archive the dir (~25 MB). Tensorboard logs are safe — one `PPO_<n>/` per
  run. A `--run-dir` flag would remove the hazard entirely. **[speculative]**

## 7. Not investigated

- **`gamma: 0.99` at 50 Hz = 2.0 s lookahead vs `resample_s: [1.5, 4.0]`**
  — the discount horizon and the command hold time are the same order, so the
  policy can barely see past the current command. Probably intentional; worth
  a deliberate look. **[speculative]**
- **`p_v_zero: 0.2` plus `w_alive: 0.5`** — 20% of commands *are* "stop", and
  standing still collects the alive bonus. Combined with §1, there may be a
  do-nothing attractor that is over-rewarded. **[speculative]**
- **`n_steps: 512` vs `max_steps` 750** — no episode fits in one rollout, so
  every episode spans a boundary and relies on GAE bootstrapping at
  truncation. Standard, but never verified here that truncation vs
  termination is being distinguished correctly end-to-end. **[speculative]**
- **8 pre-existing test failures** in `tests/test_drive.py` (all flick/flip,
  "fell during flick" / "servo snapped") on a clean tree. Unrelated to the
  general policy — the trajopt feedforward likely predates the payload mass
  now in `bike_params.yaml`. Re-run `optimize_flick` when those matter again.
