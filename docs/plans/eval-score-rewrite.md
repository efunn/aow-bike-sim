# Fixing the eval score

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
