# Fixing the eval score

> **Status: OPEN — half landed, 2026-09-08.** The geometric mean over commands
> and the per-family reporting shipped; the **directional gate did not**.
> `train_general_rl._score` is still `survive_rate * track_geo`
> (`train_general_rl.py:243`), and `speed_ratio_fwd` is computed but marked
> "Diagnostic only, deliberately not in `_score`". This is critical-path item 3
> and ranked risk #2 in `docs/status.md`: every future long run is exposed to a
> policy trading away a direction, which has cost 12M steps once already.

The selection score is the thing that decides which checkpoint becomes a
`moves/*.npz`, and which of two runs was better. It is currently too narrow to
do either job well. This doc collects what is measured, what the mechanism
turns out to be, and what the candidate changes cost — because the cost, not
the design, is what has kept this open.

**Scope.** `_score`, `track`, the eval grid, and `BestByScore` — i.e.
`train_general_rl.py`. NOT the reward (`general_env.step`), which is a
different object that happens to share weights' worth of vocabulary. See
"Two things called the eval" below; conflating them has already cost time.

**Prior art, not restated here.** `general-rl-improvements.md` §2.5 (the four
behavioural metrics, SHIPPED 2026-08-06), §2.6 (why the geometric mean does not
fix reverse), §3 (per-command breakdown, grid thinness, randomization,
single seed). Read those first. This doc is the layer that decides what to do.

---

## Two things called "the eval"

They are routinely conflated in conversation and they contain different terms.

| | the REWARD | the SCORE |
|---|---|---|
| where | `general_env.step` | `train_general_rl._score` |
| what it drives | the gradient | checkpoint selection, and every comparison table |
| velocity tracking | yes, `w_vel` | yes, via `track` |
| heading tracking | yes, `w_head` | yes, via `track` |
| roll magnitude | yes, `w_upright` | **no** |
| pitch | yes, `w_pitch` | **no** |
| action smoothness | yes, `w_smooth` | **no** |
| hub magnitude at rest | yes, `w_hub_idle` | **no** |
| effort | yes, `w_effort` | **no** |
| falling | yes, `penalty_fall` | yes, `survive_rate` |

So a change that makes the bike visibly calmer is priced by the reward and
invisible to the score. That asymmetry is the whole problem.

## What the score is, exactly

    _score = survive_rate * track_geo

over the 20-command grid, 15 s per command, **randomization off**, and

    track = mean over steps of  0.5 * (r_vel + r_head)

`track_geo` is the geometric mean of per-command `track`; `survive_rate` is the
fraction of commands that did not exceed `fall_roll_deg` 60.

## Why it is too narrow

### The measured evidence

Four glide arms, selection score 0.886 / 0.898 / 0.915 / 0.902 — a spread of
0.03 — while contact load moved 4x and the kick-recovery envelope moved from
1/8 to 8/8. The single largest behavioural improvement in the period,
`general_rl_glide_pitch_hub3`, reads:

| | baseline | hub3 |
|---|---|---|
| rim travel over a 15 s hold | 7.58 m | **3.35 m** |
| airborne fraction | 58% | **13%** |
| peak contact force | 7.23x weight | **3.66x** |
| kick recovery at dv 0.35 | 7/8 | **8/8** |
| **selection score** | — | **moved 0.03** |

"Holds station" and "holds station by sawing the wheel seven metres" are the
same number today.

### The survival factor saturates IN-DISTRIBUTION, and only there

**This section has been wrong twice; the corrections are kept because each
looked right.** First it said "survival carries no information". Then it said
that holds "on truth". Both are too loose. The actual condition is
IN-DISTRIBUTION: a CONVERGED policy evaluated in the mode it was TRAINED for
survives 1.00, whatever that mode is.

Read the standings column header — it is "trained-on", not "on truth":

    policy               evaluated in its own training mode      surv
    smooth_diff_pi       truth                                   1.00
    odo                  the odometry estimate                   1.00
    nolat                truth, no v_lat                         1.00
    odo_ahrs             estimate + tm151                        1.00

Every collapse in those tables -- 0.044 / 0.15, 0.110 / 0.20, 0.537 / 0.90 --
is an OUT-OF-DISTRIBUTION evaluation.

Where the factor does real work:

  * OUT OF DISTRIBUTION, e.g. a truth-trained policy in sensor mode.
  * DURING TRAINING, at any difficulty not yet mastered. Measured live on
    `general_rl_odo_ahrs_rand`, eval at the nominal point:

        steps   survive  track_geo   score
        1M         0.10      0.244   0.024
        2M         0.45      0.354   0.159
        3M         0.35      0.314   0.110

    Survival dominates throughout and the 2M -> 3M regression is survival-led.
    `BestByScore` correctly declined 3M.

**WHY THIS MATTERS FOR THE FIX, and it is not a detail.** "Make the grid
harder" is too loose to act on, because an out-of-distribution evaluation is a
TRANSFER TEST, NOT A SELECTION CRITERION. Selecting checkpoints against a
distribution makes that distribution in-distribution by construction, and
survival saturates again — this is precisely what happened when `odo_ahrs`
trained against the TM151 and then scored 1.00 against it.

What is needed is something hard INSIDE the training distribution. There is
already an exact instance of that, unexploited:

    randomization.disturb_prob      0.01
    randomization.disturb_force_N   2.0

The policy trains with disturbance pokes. The eval runs with randomization
off. **So it is never once evaluated on a disturbance it was trained to
handle.** That is option B below, and this is the argument for putting it above
the others.

### The mechanism behind the reverse trap (from §2.6, and it generalises)

`track = 0.5 * (r_vel + r_head)`, so a policy that **ignores a velocity command
while holding heading perfectly** still scores ~0.5 on that command, not ~0.
The geometric mean only bites when a command approaches zero, which needs BOTH
terms to fail.

Measured: refusing both reverse commands moved the score 0.765 -> 0.754.
`general_rl_og`, which reverses correctly, FELL on turn-at-speed and dropped
0.694 -> 0.522. Under the current score the reverse-refusing policy wins.

This is the same mechanism as critical-path item 3, where `_score` rose
monotonically across the span in which a 12M run lost forward drive. It is not
a second bug. **Any fix that leaves `track` as a mean of two terms inherits the
0.5 floor.**

## Candidate changes, ordered by value per unit of disruption

### A. Score the velocity component per command — geometric mean of `r_vel`

From §2.6 option 1. Removes the 0.5 floor at its source: an ignored velocity
command really does approach zero. Keeps behavioural metrics out of the score,
which preserves the separation between "what selects" and "what diagnoses".

  * cost: re-bases every score. See "The re-basing decision".
  * does NOT address contact, disturbance, or hesitancy.
  * smallest change that fixes a failure we have actually been bitten by twice.

### B. A disturbance arm  — REORDERED TO FIRST after the in-distribution finding

`analysis/kick_recovery.py` already measures it. It needs to become a SELECTION
criterion rather than a report.

This is now the leading candidate, not merely the highest-value one. It is the
only proposal here that makes the score discriminate WITHOUT relying on an
out-of-distribution evaluation — the policy already trains against pokes
(`disturb_prob` 0.01, `disturb_force_N` 2.0) and is never evaluated on one. So
it asks a question the policy was trained to answer and is not currently asked,
which is exactly the shape a selection criterion needs and a transfer test does
not have.

  * cost: eval time. Episodes already run to termination and a competent policy
    costs the full grid; adding a disturbance arm adds commands.
  * requires deciding whether the disturbed episodes join `track_geo` or form a
    separate factor. A separate factor is probably right — a policy should not
    be able to trade nominal tracking for recovery or the reverse.

### C. A contact-load term

Since "holds station" and "holds station by sawing the wheel" score
identically. Needs a normalisation nobody has chosen yet: rim travel, peak
force and airborne fraction all move together in the hub3 measurement, so one
of them may stand in for the set.

  * cost: re-bases every score, AND it is a term whose units are not obviously
    commensurable with a bounded [0,1] tracking reward.

**SERVO LOAD IS A FOURTH CANDIDATE, and the only one measurable on hardware.**
Noted 2026-08-28. The XC430/XC330 control table carries:

    addr  XC430-W150 (drive x2)          XC330-T181
    124   Present PWM                    Present PWM, 0.113 %
    126   Present Load, -1000..1000,     Present Current, 1.0 mA
          0.1 % of max torque

**THE CONTROL TABLES DIFFER AT 126, AND THE TWO SIGNALS ARE DIFFERENT IN KIND**
-- corrected 2026-08-28, an earlier version of this section quoted
`Present Load` for both. The XC430 doc has ZERO mentions of `Present Current`
or `Current Limit` against the XC330's 15, so the XC330 has real current
sensing and current-based control while the XC430 does not. Its "Load" is a
DERIVED ESTIMATE -- a scale-free percentage of max torque, likely computed from
PWM -- where the XC330 reports MEASURED milliamps, convertible to torque
through the motor constant. That asymmetry is why the internal filter is hard
to pin on the drive servo and largely moot on the steering one, and it is not a
naming difference.

124/126/128/132 are contiguous, so the indirect block goes from 10 bytes to 14
of `N_INDIRECT` 28 -- same single SyncRead, no extra round trip, no added lag.
**It does NOT break `READ_BLOCK`'s uniformity** -- corrected 2026-08-28, an
earlier version of this paragraph said it would. 126 is the SAME ADDRESS AND
WIDTH on both models, so the indirect map is byte-identical and one SyncRead
still serves every servo; the per-model part is a SCALE CONSTANT at decode
time, exactly as `VEL_LSB_RAD_S` already is. Indirect addressing would also let
a dummy register pad a genuine misalignment into a common virtual layout, so
even a real divergence here is cheap.

Why it matters more than the sim-only candidates: rim travel, peak contact
force and airborne fraction cannot be measured on the bike. Servo load can. So
it is the only proposal here that could VALIDATE the sim's contact numbers
against hardware rather than merely score them -- and the contact model is the
least-measured parameter in the sim.

What it is NOT: a contact force. It measures ACTUATOR EFFORT, so it is a good
proxy for "holds station by sawing the wheel seven metres" (which is what C is
actually about) and a poor one for peak contact load.

**The internal filter is unknown and that is a solved genre of problem here,
twice over.** `hw/dynamixel.py` already declines to trust `Present
Velocity(128)` because someone characterised it as "roughly a 50 ms boxcar" and
re-estimated velocity from position instead; and `analysis/tm151_check.py`
turned a guessed 2.0 s orientation tau into a measured 0.19 (r2 0.999) from a
300 s capture. Same move: command a known step, log Load at 100 Hz, fit.
Recording `Present PWM` in the same block is the cheap control -- if Load is
merely a filtered PWM, one regression says so and hands over the filter.

### D. Randomization on, or a second randomized score

From §3. Selection currently optimises nominal-model performance while
untethered transfer depends on `randomization.actuator_frac` et al.

  * cost: the eval stops being an exactly reproducible point measurement, which
    is the property that makes numbers comparable across months. Probably wants
    to be a SECOND score reported alongside, not a replacement.

### E. Surface `t_head_s` rather than averaging it away — DONE 2026-08-28

Already recorded per command (`_HEAD_TOL_DEG` 10.0, saturating at episode
length when never reached). Paired with `fell` it separates the three failure
modes an operator can see by hand at teleop `[8]`:

| | fell | `t_head_s` | `head_err_deg` |
|---|---|---|---|
| fell over | yes | — | — |
| decisive | no | low | low |
| hesitant | no | high, below episode length | low |
| never tried | no | pinned at episode length | high |

  * cost: nearly nothing — it is a reporting change, and the data already
    exists. Does not re-base anything.
  * **Do this one first regardless of the rest**, because it is free and it
    turns a teleop impression into a number, which is the move that §2.5
    already proved out once.

**DONE, and it paid immediately.** `t_head_s` is now in the metrics dict, and
it INVERTS the ordering the other numbers give: `general_rl_odo` arrives
fastest (0.87 s) and then holds worst (28.7° in the settled window), while
`general_rl_odo_ahrs` arrives slowest but one (1.40 s) and holds tightest
(14.2°). Arrive-fast-then-wander against arrive-slow-hold-tight is a
behavioural split no single number was showing.

It also cannot be aggregated naively, which is what led to **F** below: 6 of
the 20 commands have `dpsi 0`, so `t_head` fires on the first step and a
median over all 20 is really the 4th-5th smallest of the 14 turners. It is now
reported over turning commands only, split small (`t_head_s`, 45/90) and large
(`t_head_big_s`, ≥170) because those are bimodal — 0.5–3 s against 5–15 s or
never — and one median sits in the empty middle between them.

### F. A PER-FAMILY metrics block — the aggregate is blind by construction — DONE 2026-08-29

Found while doing **E**, and it is the same defect one level up. **A median
over 20 commands cannot see a failure affecting fewer than 10 of them.** The
grid has exactly 6 large turns, so no whole-grid median can EVER report a
large-turn failure. Measured, `head_err_med` at tau 0.19:

| policy | hold (1) | straight (2) | crab (2) | turn≤90 (8) | turn≥170 (6) | ALL (20) |
|---|---|---|---|---|---|---|
| `odo_ahrs` | 4.8 | 6.8 | 16.3 | 5.4 | **65.7** | 8.1 |
| `odo_ahrs_rand` | 12.1 | 1.6 | 32.9 | 5.3 | **100.8** | 10.9 |
| `odo_ahrs_tau019` | 9.9 | 4.1 | 10.3 | 3.7 | **156.7** | 11.1 |
| `odo_ahrs_rand2` | 8.2 | 2.7 | 8.0 | 3.6 | **152.1** | **7.4** |

The ALL column INVERTS the ranking on the axis that matters: `rand2` has the
best whole-grid median and is the second worst on large turns. Every
conclusion drawn from "rand2 holds heading better" came from that column.

**Shipped** as `train_general_rl.FAMILIES` and `_by_family`: a `by_family`
block under `metrics`, five families that PARTITION the grid, ~20 numbers
against the ~160 of the full metric x command matrix. Top-level `n_eval`,
`survive_rate`, `track`, `track_geo` are untouched, so **this did not re-base
`_score` and was not blocked on the decision below.** `analysis/chatter.py`
prints one table per family; every future export carries the block, and
policies exported before it get one by re-running the eval.

Families: `hold` (1), `spin` (2, in-place ±90), `cruise` (9), `crab` (2),
`turn_big` (6). `spin` exists because it fell through every predicate in the
first draft — and it is the family where `rand2` reads 52° against ~2° on the
same turn while moving, so the gap it was hiding was the largest one.

Which metrics survive the reduction, and why the others do not:

  * `head_err_med` — DROP. On turns it restates `t_head_s`; on hold/cruise it
    tracks `head_err_tail` at a steady ~2–2.5x for every policy.
  * `vel_err_tail` — DROP. 19% spread across policies against `vel_err_med`'s
    67%; the tail is dominated by the worst single sample.
  * `drift_max`, `drift_sd` — DROP, REPLACED by `drift_overshoot` = peak minus
    final. `drift_max` equalled `drift_m` exactly in 3 of 4 policies (drift
    grows monotonically), and `drift_sd` sat at 0.267–0.315 x `drift_m` in all
    4 — the ratio for a linear ramp. The overshoot is the same information
    with a null value of zero.
  * `t_head_s` and `head_err_tail` are BOTH kept, and look redundant but are
    not. `odo_ahrs` and `rand` arrive at large turns at 6.73 s and 6.78 s —
    indistinguishable — while holding them at 14.7° and 105.0°. `rand`
    touches 10° once and wanders back off.

Two invariants worth asserting, each catching a different bug: `sum(n_f) ==
n_eval` (a command falling through every predicate is invisible — this fired
once already), and `sum(survive_rate_f * n_f) / n_eval == survive_rate` (a
predicate that double-counts, or a rate on the wrong denominator).

**Survival is a RATE per family, not a count.** Families are sizes 1, 2, 9, 2,
6, so `fell: 1` would mean a 6x different thing in `turn_big` than in `crab`.

**READ `survive_rate` WITH `t_head_s`, NEVER ALONE**, because a high family
survival means competence OR refusal. On `turn_big`, `odo_ahrs` attempts the
turn, arrives in 6.73 s and falls on 1 of 6; `rand2` never turns at all —
it drives BACKWARDS at ~0.8 m/s (`vel_err_med` 0.091 with a heading error of
175.6°), which satisfies the world-frame velocity command exactly while
abandoning the heading, and therefore never falls. That is survival bought by
refusal — the pattern `track_geo`'s geometric mean was introduced to defeat at
the grid level, reappearing inside a family. Nothing in the schema catches it
except reading the two together.

  * cost, as built: a reduction over rows `_eval_episodes` already builds. No
    extra simulation, no re-basing, no change to `_score`.
  * the invariants are on INTEGER COUNTS, not the stored rates. Checking the
    rounded `survive_rate` fired a false positive on the first run —
    `turn_big`'s 5/6 rounds to 0.833 and weights back to 0.9499 against
    0.9500.
  * CLOSED 2026-08-29: `cruise` computes `t_head_s` over 6 of its 9 members
    (the straight members reach 10° on the first step), so one cell has a
    different denominator from its row. `t_head_n` is reported beside it. A
    sixth family splitting straight from moving-turn was considered and is NOT
    warranted — within `cruise`, straight against turning gives `vel_err_med`
    ratios of 1.02 / 1.15 / 0.81 / 0.94 and `head_err_tail` ratios of
    0.69 / 1.62 / 0.86 / 0.99 across the four AHRS policies. **The direction is
    not consistent**, which is the test: every split that earned its place —
    `turn_big` against `cruise`, `crab` against `cruise`, `spin` against moving
    turns — had the same sign for every policy. These do not, so it is noise.
    Survival stays over all 9, which is the number that has to be 1.00 anyway;
    a break there is findable per command.

## The re-basing decision — this is the actual blocker

Any change to `_score` re-bases every score in `docs/` and in every
`moves/*.yaml` `trained.metrics` block. Those numbers are how runs months apart
are compared, and there are 42 exports.

Three options, and the choice has not been made:

1. **Re-measure.** Run the new score over every export that still matches the
   plant. Bounded (8 of 42 as of 2026-08-26) and gives one comparable table.
   Exports that no longer match the plant cannot be re-measured meaningfully
   anyway, which caps the work.
2. **Annotate.** Leave old numbers, mark the score version alongside them. Cheap
   and permanently confusing — two numbers called "score" in one table is the
   failure mode this repo has hit before with `params_digest`.
3. **Version the field.** `score` stays, `score_v2` is added. Honest, and it
   makes `BestByScore` ambiguous unless one is named as authoritative.

**Recommendation: (1), scoped to the 8 plant-matching exports**, with the old
column dropped rather than kept. A score that cannot be recomputed for an
artifact is a score that artifact should not carry.

## What is NOT in scope

- The reward. Changing `w_*` is a retraining decision with its own costs, and
  §1 of `general-rl-improvements.md` already argues the reverse case there.
- `BestByScore`'s cadence (`eval_every`). Orthogonal.
- The sensor modes. Those are settled — see `test_sensor_modes.py` and the
  sensor sections of `docs/status.md`.

## Order of work, if this gets picked up

1. ~~**E** — free, no re-basing, immediately useful for the 180-degree flip
   question.~~ DONE 2026-08-28.
2. ~~**F** — also free and also unblocked, and what makes every other
   comparison in this doc readable.~~ DONE 2026-08-29, `cruise` denominator
   question included: measured and closed, no sixth family. **Nothing under F
   is outstanding.**
3. Decide the re-basing question above. Everything else is blocked on it and
   nothing else should start first.
4. **B** — the only candidate that discriminates in-distribution. Largest
   design question (separate factor vs folded into `track_geo`), but it is the
   one that changes what selection can see.
5. **A** — smallest real fix, addresses a failure seen twice.
6. **C** and **D** — both want a measurement or a convention that does not
   exist yet. Note that **D is a transfer test, not a selection criterion**,
   by the argument above; it belongs alongside the score, never inside it.

---

## The command distribution, 2026-08-30 — moved here from docs/status.md

Moved verbatim 2026-09-08. The score and the sampler are the same problem
seen from two ends: this measures what training was ASKED to do, while the
rest of this document is about how the result was SCORED.

### The command distribution — what training never sampled (2026-08-30)

Every sensor arm changed what the policy *sees*. Nobody had looked at what it
is *asked to do*. Measured against `general_rl_odo_ahrs_pitch_w`'s own 20M-step
run, the legacy sampler draws `v_lon` (20% zeroed), a lateral term and a
heading step, each from its own difficulty-scaled range — so the family a
command lands in is a **side effect of three ramps interacting**:

| share of draws | d=0.15 | d=0.5 | d=1.0 | eval grid |
|---|---|---|---|---|
| in place, ~no turn | 14.1% | 5.6% | 1.4% | 5% |
| in place, turning | 24.1% | 13.4% | 3.6% | 10% |
| in place, about-face | 0.0% | 0.0% | 0.2% | **15%** |
| moving straight | 61.8% | 47.7% | 14.9% | 45% |
| moving with lateral | 0.0% | 33.3% | **75.7%** | 10% |
| moving, about-face | 0.0% | 0.0% | 4.1% | 15% |

**The mix inverts.** Straight-line cruise goes 62% → 15% while anything with a
lateral component goes 0% → 76%, because `v_lat` is a *dither on every draw*
rather than an occasional crab command — at `v_lat_frac: 0.4` it is ±0.48 m/s
at full difficulty. **And the in-place families starve** for the same reason:
hold, spin and about-face all need `v_lat ≈ 0`, which is exactly what the
sampler stops producing. The eval's `hold` command — every component exactly
zero, and the only place `drift_m` is defined at all — is **5% of the score and
0% of the experience**, at every difficulty. `p_v_zero: 0.2` does not reach it:
it zeroes `v_lon` alone, so what it actually samples is "stop, crab sideways,
and turn to face somewhere else".

**An earlier reading of this was wrong and is retracted.** `hold_spectrum.py`'s
airborne columns were used to argue holding is intrinsically hard; those columns
are the ones `liftoff.py` supersedes as *"actively misleading"* (the rear omni's
0.6 mm envelope ripple reads as a wheelie). The LQR holds standstill at 1.17°
peak roll over 40 s and `drive.speed_grid` starts at 0.0, so holding is a solved
linear problem. The sawing every policy does under a hold command is an
out-of-distribution symptom, not evidence of difficulty.

**Episode lengths, both ends.** `gamma: 0.99` at 50 Hz is 2.0 s of lookahead
(1/(1−γ) = 100 steps), and the discounted mass a critic has seen by time T is
1 − γ^(T/dt):

| segment | horizons | mass seen |
|---|---|---|
| 1.5 s (old `resample_s` floor) | 0.75 | **53%** |
| 4.0 s (old ceiling) | 2.0 | 87% |
| 6.0 s | 3.0 | 95% |
| 15 s (old eval) | 7.5 | 99.95% |

The old floor spilled the critic's window past the command change for nearly
half its mass while the policy was told the command is stationary. And the eval
held one command for 15 s — 7 horizons of a regime no training command reaches
— with `_TAIL_S` reading seconds 13–15 specifically.

**Changed.** Eval episodes are now 5 s (`_EVAL_EPISODE_S`, override with
`env.eval_episode_s`), so the tail window lands at 3–5 s. **Outstanding: every
`eval_*` in an existing `moves/*.yaml` was measured at 15 s and is not
comparable — re-run rather than mix.** The saving is incidental: eval is a
single `GeneralEnv` stepped serially, ~11 s of sim per pass measured at 1,311
steps/s, 20 passes a run.

**Also measured, not yet acted on:** training resets are ±2° roll / ±5° yaw
with full randomization and 59% of episodes end in a fall (`success_rate` 0.41
at 20M); the eval resets clean at 0.5° with randomization off and survives 90%.
Both numbers are right; they are not the same bike. Read `eval/survive_rate` as
a clean-room figure.

**The arm.** `config/rl_general_cmd_curriculum.yaml` replaces the mix with four
explicit weights (`cmd_families`), drops the lateral command entirely
(`v_lat_frac: 0.0` — crab stays in the eval, deliberately unrepresented),
widens `resample_s` to `[2.0, 6.0]`, and starts the curriculum at 0 with a pure
hold stage that ramps out. Opt-in: absent `cmd_families`, the sampler is the
legacy one bit for bit. Watch `curriculum/difficulty` before reward.

#### Arm 1 ran, and it failed: the pure-hold stage kills forward motion

20M steps, difficulty reached 1.0 by 13M, survival healthy (`eval/survive_rate`
0.85–0.95). **`eval/speed_ratio_fwd` was 0.000 at all twenty evals** while
`speed_ratio_rev` sat at 0.85–1.28. In teleop the bike ignores forward and
reverses. On the export, the hub action is negative under *every* command:

| command | hub action | v_lon | pen: net travel over 5 s |
|---|---|---|---|
| hold | −0.487 | −0.192 | 0.93 m |
| forward +0.80 | **−0.258** | **−0.334** | **1.67 m BACKWARD**, `wander` 1.2 |
| reverse −0.50 | −0.651 | −0.644 | 3.23 m |

`general_rl_odo_ahrs_pitch_w` under the same commands: −0.223 / **+0.219** /
−0.502, holding at −0.037 m/s. Signed per-command ratios put arm 1 negative on
**all nine** forward commands, including the two pure straight-line ones where
no turn can be blamed; pitch_w is positive on five and negative on four (its
four are the `turn_big` reversals documented above).

**It is not the plumbing, and that was tested rather than assumed.** The
command chain is symmetric (35.1% forward / 35.3% reverse, timestep-weighted);
obs slot 8 carries the right sign; and a four-way probe of command sign against
actual velocity gives `vel_err` 0.086 when they agree and 0.83–0.91 when they
oppose — **byte-identical numbers for arm 1's config and pitch_w's**. Same code
trained both, and one drives forward.

**The mechanism is the advance gate.** Arm 1's hold drifts backwards at
0.192 m/s, which at `sigma_v` 0.35 scores `r_vel = 0.74`; with heading held the
episode scores `track = 0.87` against `advance_score: 0.6`. A mediocre hold
clears the bar with 45% margin, so the curriculum certified it and every later
stage inherited the bias. Escape is then impossible: under a +0.8 command at
v = −0.33 the velocity reward is `exp(−(1.13/0.35)²) ≈ 3e-5` against 0.0055 for
standing still — 0.005 of difference on an episode return near 900.

**Note the stage did not even buy holding.** Arm 1 holds at 0.192 m/s drift
against pitch_w's 0.037 — 5.2× worse, over the same 5 s pen run — and pitch_w
was never given a hold command at all.

**Outstanding — arm 1 moved six variables at once** (family sampler,
`curriculum.start` 0.15→0, `v_lat_frac` 0.4→0, `resample_s`, `ahrs_tau_s`
2.0→0.19, `obs_pitch` without `w_pitch`), so one run cannot attribute the
failure. The mechanism above fits the arithmetic but is not isolated.
**Arm 2 (`config/rl_general_cmd_curriculum2.yaml`) is the one-line
discriminator:** `hold_max: 0.40` makes difficulty 0 a 40% hold / 60% straight
mix, so clearing the gate requires hub in both directions. If forward returns
it is the hold stage; if not, the next cut is `curriculum.start` back to 0.15
with the sampler kept.

**Also queued, independent of which rung wins:** the velocity reward has no
gradient outside about ±0.7 m/s, which is what makes this failure unrecoverable
rather than slow. A bounded linear term would trap no policy this way.

#### KNOWN ISSUE: handedness on swept turns (2026-08-30)

`general_rl_cmd_curriculum2` executes `(0.804, 0, +90)` by turning left and
driving forward, and `(0.804, 0, −90)` by turning **left anyway** (+87°) and
reversing at −0.697 — which lands its world velocity within 3° of the command,
so the velocity half is satisfied exactly and only the heading is abandoned.
`turn_asym` 0.369 against `odo_ahrs_pitch_w`'s 0.168 is the same fact.

**Confirmed in teleop, and a ramped heading command does not fix it** — the
preference survives a swept turn as well as a step. Accepted for now: it is a
training problem, not an eval one, and no arm is queued against it.

**Queued: `config/rl_general_cmd_curriculum2b.yaml`** — `algo.seed` 0 → 1 and
nothing else. Every conclusion on this line has come from one run per arm, and
`asymmetric-actor-critic.md` §7 argues the seed floor is the binding constraint
on judging any change at all: if the spread across seeds is the size of the
effect, no single-run comparison means anything. `speed_ratio_fwd` −0.426 →
+0.841 is either `hold_max` working or one lucky draw, and nothing so far
separates those. It also exercises the new eval instrumentation end to end.

```sh
./scripts/rl.sh up general --config config/rl_general_cmd_curriculum2b.yaml \
    --run-dir runs/general_rl_cmd_curriculum2b \
    --export-name general_rl_cmd_curriculum2b
```

**Read it against a RE-SCORED seed-0 export, not against the block recorded in
`moves/general_rl_cmd_curriculum2.yaml`** — that one was measured with the ball
in and the wide ratio selector.

**What WAS fixed is the eval reporting it as something else.** A command is a
world velocity plus a heading, so the velocity half is satisfiable body-forward
(yaw = `psi_cmd`) or body-backward (`psi_cmd` + 180°) — `v_ach`'s sign is
decided by which way the policy turned, not by whether it will drive forward.
`speed_ratio_fwd`/`rev` now select only `|dpsi| < 1°` (2 forward rows, 1
reverse; the 9 turning rows are excluded and covered by `turn_asym` and
`by_family`). Before the change, six rows at +0.79…+0.97 and three at
−0.98…−1.16 cancelled to **+0.027** — a policy that drives forward fine,
reported as one that does not.

**And the eval had a ball in it.** `ball_prob: 0.25` is training DR that was
never masked for evaluation, so five of twenty commands were scored against an
obstacle the metrics never mention — deterministic per command, arbitrary as to
which. Now zeroed in both `train_general_rl._eval_cfg` and
`analysis/per_command._cfg_for` (shared by `eval_video.py`, so the clips and
the bar charts match the metrics). **Outstanding: every eval number recorded
before today includes it.**

**`speed_ratio_fwd` is now SIGNED** (clip floor 0.0 → −1.5). The floor read
every wrong-direction policy as exactly 0.000 and threw away the one number
that says what it is doing instead. It also flattered the metric generally:
pitch_w's four reversing rows used to count as zero rather than as negatives,
so its published 0.642 was inflated. **Outstanding: `speed_ratio_fwd` numbers
recorded before this are not comparable, and neither are those from before the
15 s → 5 s change.**

---
