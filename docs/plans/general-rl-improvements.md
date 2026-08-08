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

1. **Per-command eval breakdown** (see §3) — before changing any reward,
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

## 2. Behaviour the objective does not pin down

Three policies, identical reward and curriculum, differing only in parallelism
and batch size, scored the same and drive visibly differently:

| | (1) `general_rl_og` 8x2048 | (2) `general_rl` 32x512 | (3) `general_rl_1k` +batch 1024 |
|---|---|---|---|
| steer at rest | offset to one side | offset to one side | ~straight |
| zero command | drifts toward the front wheel | drifts toward the front wheel | drifts backwards |
| turn handedness | fast both ways | strongly prefers one | — |
| reverse | ok | refuses | good |
| 180 deg | fast | fast | slower |

**Attribution caveat**: n = 1 per config, and `n_envs` changes the per-env
seeds (`seed + i`), so the hyperparameter and the random seed are confounded —
these differences are as consistent with run-to-run variance as with batch
size. Any claim that a hyperparameter caused a behaviour needs 2-3 seeds per
arm. What the comparison *does* establish is that **metric variance is low
while behaviour variance is high**: the score does not span what the operator
cares about. Four concrete gaps.

### 2.1 The eval never turns left **[measured]**

`_EVAL_CMDS` has 12 commands whose heading steps are `{0, +45, +90, +180}` and
whose lateral commands are `{0, +0.33}` — **not one negative heading step and
not one left crab**. A policy that turns beautifully one way and badly the
other scores a flawless eval, which is exactly observation (2). Snapshot
selection cannot see handedness, so it never selects against it.

Fix: mirror the grid — add the sign-flipped twin of every asymmetric command
(~20 commands total). Cheap, and it makes handedness both visible and
selectable-against. Note `+180` is its own mirror in heading terms but not in
*execution*, so the direction the policy chooses to rotate is still worth
recording.

### 2.2 Steer angle at rest is free **[measured]**

The action is steer *rate*, and the only action costs are `w_effort: 0.001`
and `w_smooth: 0.005` — both on the action, not the state. Once the wheel is
cocked, holding it there costs exactly zero. Nothing in the reward prefers a
centred wheel, so "wheel offset at rest" is an entire family of equally
optimal policies.

**Observe it; do not pin it.** A cocked front wheel at standstill is how a
trackstand works on a real bicycle — a legitimate strategy, not a defect — and
a `|steer|` penalty would forbid a technique the bike may genuinely need. So
`steer_rest_deg` is an eval diagnostic for comparing what different policies
settle on, and nothing more. (Earlier drafts of this section proposed a reward
penalty; that was wrong.)

The one case worth acting on is §2.7, where the offset grows to 24° late in
training — but the cause there is exploration pressure outlasting the
curriculum, so the fix is the training budget (already applied), not a reward
term.

### 2.3 Slow drift is nearly free **[measured]**

`sigma_v: 0.35` is generous near zero: drifting at 0.15 m/s against a
"hold station" command still collects `exp(-(0.15/0.35)^2) = 83%` of the
velocity reward, while wandering >2 m over a 15 s episode. To a rider that is
a broken stationary hold; to the objective it is a rounding error. And because
the command is a *velocity* vector there is no position term anywhere — drift
is only ever penalised through its (small) velocity error.

**And the policy cannot see it.** `build_obs` (`control/general_spec.py:60`)
carries no position or integrated error — only velocities. The bike has no way
to know it has drifted 2 m, so a reward term on displacement would ask it to
minimise something invisible, the exact error the spec docstring calls out for
`prev_action` ("the policy is asked to minimize something it cannot see").

That makes drift the most expensive item in this document to fix properly:

- a station-keeping reward alone would not work — it needs position, or an
  integrated velocity error, added to the observation;
- that changes `OBS_DIM`, which **invalidates every existing policy** (replay
  checks `obs_dim` against the spec and refuses to load) and forces a retrain
  of all three;
- the cheap alternative is to make the velocity tolerance **relative instead of
  absolute** — replace the constant `sigma_v: 0.35` with something like
  `sigma = 0.1 + 0.25 * abs(v_cmd) / v_max`, so a "stop" command demands
  near-zero velocity (0.15 m/s of drift drops from 83% of the reward to ~11%)
  while a full-speed command keeps today's slack. No new state, no obs change.

  Be clear about what that buys: it attacks the drift *rate*, not accumulated
  displacement. The bike gets stiller; it still cannot return to where it
  started, because it cannot tell that it left.

**Why the LQR does hold station.** The analytic controller regulates position
explicitly — `linearize.py:12` has `x = [e_lat, roll, yaw, steer, v_lat,
roll_rate, yaw_rate, steer_rate]`, with `q_ypos: 3.0` weighting that first
term. Lateral drift is a state it can see and is penalised for. (Only lateral;
there is no `e_lon`, which is fine — fore/aft is the benign direction.) That
is the whole difference: the LQR was given the one signal `build_obs`
structurally lacks.

Measure first: `drift_m` is in the eval now, so the size of the problem across
checkpoints is knowable before paying for an obs change.

### 2.4 The plant is mirror-symmetric, so handedness is an artifact **[measured]**

`axle_cant_deg: 0.0` ("axles are purely tangential") and each axle carries two
truncated cones mirrored about the wheel mid-plane, so the AOW as modelled has
no chirality; the randomization draws are symmetric too. There is therefore no
physical reason for a left/right preference in sim — it is learned symmetry
breaking, and mirror augmentation (reflect obs and action about the sagittal
plane) is a *valid* structural fix rather than a bias. Worth confirming the
real wheel has no handedness the model omits before relying on it.

### 2.5 Behavioural metrics worth adding to the eval

The four things the operator noticed are all measurable, and none are
currently measured:

- zero-command drift [m over 15 s]
- mean `|steer|` at rest [deg]
- left-vs-right completion time for the same turn [ratio]
- achieved-vs-commanded speed in reverse [m/s]

Turning teleop impressions into eval columns is what would let snapshot
selection choose the bike you actually want, instead of the one with the best
blended mean.

**SHIPPED 2026-08-06** — grid mirrored (12 → 20 commands, plus directional
`±170` since `±180` wraps and cannot express handedness), per-command table,
all four behavioural metrics, and a geometric-mean score. Measurement only:
`general_env.py` untouched. Measured on the two exported policies:

| | `general_rl` (2) | `general_rl_og` (1) |
|---|---|---|
| `speed_ratio_fwd` | 1.04 | 1.18 |
| `speed_ratio_rev` | **0.00** | 1.18 |
| `turn_asym` | 0.316 | 0.337 |
| `drift_m` | 1.34 | 2.30 |
| `steer_rest_deg` | **53.5** | 21.9 |

Every teleop impression is now a number: the reverse refusal reads 0.00
(commanded −0.5 m/s, achieved +0.03), and the cocked wheel reads 53.5°.

### 2.6 The geometric mean does NOT fix reverse **[measured]**

The plan assumed a geometric mean would penalise the reverse refusal. It does
not, and the reason matters: `track = ½(r_vel + r_head)`, so a policy that
ignores a velocity command while holding heading perfectly still scores
**~0.5**, not ~0. The geometric mean only bites when a command approaches
zero, which needs *both* terms to fail.

Measured: refusing both reverse commands moved the score 0.765 → 0.754.
Meanwhile `general_rl_og`, which reverses correctly, **fell** on the
turn-at-speed command (track 0.001) and dropped 0.694 → 0.522. So the new
score works exactly as designed for falls and hard failures — and under it the
reverse-refusing policy still wins, 0.754 to 0.522.

The mechanism is sound; the assumption that `track` bottoms out for an ignored
command was wrong. To make *selection* prefer a reversing policy, one of:

1. score the velocity component per command (geometric mean of `r_vel`), so an
   ignored velocity command really does approach zero — keeps behavioural
   metrics out of the score, smallest change;
2. admit `speed_ratio_fwd`/`speed_ratio_rev` into the score, reversing the
   diagnostic-only decision;
3. fix the reward instead (§1), and let the eval keep score honestly.

### 2.7 The last 4M steps make the policy worse **[measured]**

Scanning all 20 checkpoints of `general_rl_1k` against the new eval, alongside
`train/std` and `curriculum/difficulty` from its tensorboard log:

| step | difficulty | `train/std` | fwd | steer_rest | score |
|---|---|---|---|---|---|
| 1.6M | 0.71 | 0.561 | 0.64 | 9.7° | 0.618 |
| 3.1M | 1.00 | **0.551** | 0.89 | 7.7° | 0.772 |
| 4.6M | 1.00 | 0.565 | **0.93** | 5.3° | 0.793 |
| 6.1M | 1.00 | 0.644 | 0.90 | **4.6°** | **0.798** |
| 7.6M | 1.00 | 0.680 | 0.74 | 16.1° | 0.747 |
| 9.6M | 1.00 | 0.731 | 0.67 | **24.1°** | 0.717 |

The curriculum saturates at **3.21M** (all 32 envs at 1.0). Action `std`
bottoms out right there at 0.551 — and then *inflates* to 0.731, a 33% rise,
with entropy climbing to match. That is entropy **expansion**, not collapse:
`ent_coef: 0.005` keeps pushing exploration up, and once difficulty stops
rising at 3.2M nothing pushes back.

Everything downstream tracks it. Forward speed falls 0.93 → 0.67 and the
resting steer offset explodes 4.6° → 24.1° over the same span. Causation is not
proven, but the mechanism is plausible and specific: PPO optimises expected
return *under its own action noise*, so an inflating `std` moves the optimal
mean — and the parameters that drift furthest are the ones with no gradient
holding them, which is exactly the free steer angle of §2.2.

Practical consequences:

- **`total_timesteps: 10000000` is ~4M too long for this config.** The run
  peaks at 4.6-6.1M. Stopping there is both a better policy and ~35 minutes off
  a 90-minute run.
- Snapshot selection earned its keep: the export came from `best_model.zip` at
  **6.0M**, the peak. The final-weights policy would have scored 0.717 vs 0.798.
- If longer runs are wanted, decay the exploration pressure rather than
  extending it — SB3 takes `learning_rate` and `clip_range` as callables (a
  linear decay is a one-liner); `ent_coef` is a plain float and would need a
  small callback to anneal after the curriculum saturates.

Also visible across all 20 checkpoints: `turn_asym` sits at 0.21-0.39 with no
trend for the whole run. Handedness never improves because nothing ever
penalises it (§2.4), and `speed_ratio_rev` is healthy (0.88-1.21) from 1.1M
onward — this run never had run (2)'s reverse pathology.

## 3. Eval and snapshot selection

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

## 4. Curriculum

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

## 5. Resume correctness

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

## 6. Throughput — mostly closed

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

## 7. Measurement hygiene (learned the hard way)

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

## 8. Not investigated

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
