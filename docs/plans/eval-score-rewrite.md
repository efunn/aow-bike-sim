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

### The survival factor mostly carries no information

On truth, essentially every policy in the standings survives 1.00, so `_score`
collapses to `track_geo` alone. The multiplication only starts doing work once
something is already visibly broken.

**The exception, and it is instructive:** in SENSOR mode survival discriminates
strongly — 0.15 / 0.20 / 0.90 / 0.95 / 1.00 across the policies measured
2026-08-27/28. The factor is not badly designed; the nominal-conditions grid
simply never puts anything under enough stress to exercise it. That is an
argument for a harder grid, not for a different formula.

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

### B. A disturbance arm

`analysis/kick_recovery.py` already measures it. It needs to become a
SELECTION criterion rather than a report. This is the highest-value addition on
the critical path, because it is the only one that tests the property the bike
is actually for.

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

### D. Randomization on, or a second randomized score

From §3. Selection currently optimises nominal-model performance while
untethered transfer depends on `randomization.actuator_frac` et al.

  * cost: the eval stops being an exactly reproducible point measurement, which
    is the property that makes numbers comparable across months. Probably wants
    to be a SECOND score reported alongside, not a replacement.

### E. Surface `t_head_s` rather than averaging it away

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

1. **E** — free, no re-basing, immediately useful for the 180-degree flip
   question.
2. Decide the re-basing question above. Everything else is blocked on it and
   nothing else should start first.
3. **A** — smallest real fix, addresses a failure seen twice.
4. **B** — highest value, largest design question (separate factor vs folded in).
5. **C** and **D** — both want a measurement or a convention that does not
   exist yet.
