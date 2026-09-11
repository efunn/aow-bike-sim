# The 12-seed sweep: what varies, what breaks, and what to do about it

> **Status: FINDINGS ONLY — nothing here is implemented, and deliberately so.**
> Recorded 2026-09-10 from a 12-seed sweep of `rl_general_cmd_curriculum2.yaml`
> (seeds 0-11, 20M steps each, 26.9 h serial on the 16C/32T box). Every option
> in §6 is a note for later, not a queued change: the physical parameters are
> about to move once the bike is measured, and small reward or eval tweaks made
> now would be re-derived from scratch afterwards.
>
> Evidence tags as in `general-rl-improvements.md`: **[measured]** a number or
> code read backs it, **[reasoned]** follows from measured facts, and
> **[speculative]** a hunch worth testing.

Related: `eval-score-rewrite.md` (the score itself and the turn-personality
axis), `general-rl-improvements.md` (prior RL findings), `docs/status.md`,
`sensor-workstream.md` (the AHRS these twelve trained against — and §9 below,
for why this document's numbers survived a 09-11 bug that hit others).

---

## 1. Training is exactly reproducible [measured]

Seeds 0 and 1 re-ran the configs behind `general_rl_cmd_curriculum2` and
`_2b`. Both came back **bit-identical, 12/12 arrays**, across ten days, an
intervening `sync`, and everything committed since.

So every difference in this document is attributable to `algo.seed` and nothing
else. That is what makes the rest of it readable.

Two caveats. It was checked on ONE box: SB3 seeds python/numpy/torch but never
calls `torch.use_deterministic_algorithms`, so a different thread count could
reorder float reductions. And the exports matched partly because the eval last
changed in the same commit batch that produced `curriculum2` (`3193637` and
`034b0ae`, both 08-30 14:39) -- there was no selection change for the
reproduction to expose.

## 2. A third of runs fail, and the failure is one specific thing [measured]

Not "worse" -- **broken**: commanded forward, drives backward.

| | seeds | share |
|---|---|---|
| competent (drives forward when told) | 0, 1, 3, 6, 7, 8, 11 | 7/12 |
| **broken** (`speed_ratio_fwd` < 0) | 2, 4, 9, 10 | 4/12 |
| partial | 5 | 1/12 |

`hold_max: 0.40` reduced this failure mode -- arm 1 at `hold_max 1.0` was
`speed_ratio_fwd` -0.421 -- but did not remove it. **A third of runs on this
config still land there.**

Two independent measurements agree exactly on which: the recorded training
metric (`speed_ratio_fwd < 0.1`) and a fresh eval of straight-line competence
both pick out {2, 4, 5, 9, 10}, no false positives either way.

## 3. The failure taxonomy, with exemplars [measured]

Three attractors, and the two failures are *different* failures:

| | forward drive | heading commands | exemplar |
|---|---|---|---|
| **ideal** | 1.10 fwd / 1.11 rev | **12/12 arrived, 0 abandoned** | **`personality1`** |
| **LM1** — heading yes, direction no | **-0.42** fwd | 14/14 arrived, 0 abandoned, 0 falls | **`personality4`** |
| **LM2** — direction yes, heading no | 0.57 fwd | 8/14, **6 abandoned**, 0 falls | **`personality11`** |

**LM1 looks PERFECT on time-to-heading.** `personality4` completes every
heading command, faster than the ideal policy does, and never falls -- while
driving backwards throughout. It is not turning around to do that: it holds the
commanded heading and reverses, so it spends NOTHING on the velocity term and
optimises heading alone (§5). Time-to-heading cannot see direction; that is what
`v_ach` is for. Any selection built on manoeuvre time alone would pick LM1 --
and so would one built on pivot speed (§5b).

**LM2 never falls either**, and for the same reason LM1 doesn't: declining a
turn is safe. Survival bought by refusal, one level down from where
`track_geo`'s geometric mean was introduced to defeat it.

`personality1` is the only policy in the set that completes **every** heading
command it is given. Every other viable seed abandons 2-6 of them. That is what
is felt in teleop -- not faster turns, but no refusals.

## 4. Reverse is genuinely easier, and it is not the grid [measured]

The stock eval grid commands forward at 0.6 / 0.804 / **1.2** m/s and reverse
only at **-0.504** -- 42% of max forward -- across 9 forward and 3 reverse
commands. That looked like enough to explain "every policy reverses well".

**It is not.** Mirroring every forward command into reverse at the SAME speed
and re-scoring all twelve:

| policy | fwd@matched | rev@matched | rev - fwd |
|---|---|---|---|
| `personality1` | 1.09 | 1.09 | **-0.00** |
| `personality6` | 1.05 | 1.07 | 0.02 |
| `personality8` | 0.93 | 1.04 | 0.12 |
| `personality0` | 0.83 | 0.96 | 0.13 |
| `personality7` | 0.84 | 0.99 | 0.15 |
| `personality3` | 0.76 | 1.02 | 0.27 |
| `personality11` | 0.53 | 1.02 | 0.49 |
| `personality5` | 0.06 | 1.03 | 0.98 |
| `personality2` | -0.16 | 0.95 | 1.11 |
| `personality10` | -0.35 | 0.99 | 1.34 |
| `personality4` | -0.42 | 1.07 | 1.48 |
| `personality9` | -0.49 | 1.11 | **1.61** |

**Even at -1.2 m/s, every policy reverses at >= 0.95** -- including the four
that cannot drive forward at all. Forward drive is the thing that varies;
reverse is free. The gap is a clean severity axis, near zero for the two best
policies and rising monotonically to 1.61 for the worst.

**Training commands are symmetric** -- `general_env.py:540`,
`v_lon_w = rng.uniform(-v_lim, v_lim)` -- so command exposure cannot explain
the preference either. The asymmetry is in the eval grid only.

### The working hypothesis [speculative]

**A linearised model says forward is easier; the real geometry may be easier in
reverse.** Driven backwards the bike is front-wheel-drive with rear caster
steering, which is a different and possibly more tractable plant than
rear-wheel-drive with front steering.

The LQR points the same way, and it is not an artefact of how it was fit:
`design_gain_schedule` mirrors `control.drive.speed_grid` to negatives and
calls `settle_rolling(model, params, v)` per speed, so each reverse gain is
identified at its OWN reverse trim rather than extrapolated from forward. Its
worst fit is **R^2 0.905 at v = -0.50**, i.e. reverse is LESS linear -- which is
consistent with "the linear design struggles in reverse while the nonlinear
plant is easier there", and inconsistent with reverse simply being benign.

Not established. What would settle it is a plant-level measurement with no
policy in the loop -- recoverable roll envelope, or steer-to-lean transfer, at
+v and -v.

## 5. ONE behaviour, two different bills [measured]

Corrected 2026-09-10 after measuring it. An earlier version of this section
claimed the failing policies exploit the world-frame command by turning around.
**They do not turn at all.**

### What they actually do

Hold the commanded heading -- or rather, hold whatever heading they had -- and
drive backwards. That is the whole behaviour. Whether it scores well depends
entirely on what heading change was asked for, because `command_to_body`
(`general_spec.py:400`) rotates the world command into the **CURRENT** body
frame and scores the heading separately:

```python
v_lon, v_lat = rotate_to_body(v_cmd_world[0], v_cmd_world[1], psi)   # CURRENT yaw
psi_err      = wrap_pi(psi_cmd - psi)
```

So for a command of magnitude V:

| commanded `dpsi` | `v_cl` | bike's `vb_lon` | velocity term | heading term |
|---|---|---|---|---|
| **0** (straight) | `+V` | `-V` | `v_err = 2V` -> **LOST** | kept |
| **+-180** | `-V` | `-V` | `v_err = 0` -> **KEPT** | lost |

**Identical action, opposite bookkeeping.** Measured, `personality4`:

| command | v_ach | head_err_tail | vel_err_med |
|---|---|---|---|
| `fwd 0.80, dpsi 0` | -0.38 | **6.3 deg** | **1.20** |
| `fwd 1.20, dpsi 0` | -0.43 | 9.5 deg | 1.63 |

`head_err` of 6-10 deg is the policy pointing exactly where it was told, while
driving the wrong way -- so its world velocity is OPPOSITE the command and
`r_vel = exp(-1.2^2/0.35^2) ~ 8e-6`. It banks `w_head` and forfeits `w_vel`.

**A `head_err` near 180 does NOT mean the bike turned around.** It means the
bike stayed put while the COMMAND moved 180 deg away. From the bike's own frame
nothing happened; it just started reversing.

### On a +-180 command that reversal is nearly free [measured]

`fwd 0.80, dpsi +180`, eight policies:

| policy | v_ach | head_err | vel_err | what it did |
|---|---|---|---|---|
| **`personality1`** | **+0.94** | **4.9** | 0.17 | **turned, drove as told** |
| `personality0` | -0.81 | 177.3 | 0.13 | ignored the heading, reversed |
| `personality3` | -0.84 | 180.0 | 0.10 | ignored the heading, reversed |
| `personality6` | -0.96 | 178.7 | 0.20 | ignored the heading, reversed |
| `personality7` | -0.74 | 178.6 | 0.15 | ignored the heading, reversed |
| `personality8` | -0.90 | 180.0 | 0.12 | ignored the heading, reversed |
| `personality11` | -0.85 | 180.0 | 0.07 | ignored the heading, reversed |
| `personality4` | -0.38 | 10.3 | **1.20** | held heading, lost the velocity |

`vel_err` 0.07-0.20 means full `w_vel` for essentially nothing. **Seven of eight
take it. `personality1` is the only one that turns.**

### What the weights would do, on the +-180 case only [measured]

At a 180 deg error the heading term is annihilated rather than reduced --
`exp(-(pi/0.436)^2) = 3e-23` -- so the trade is `w_vel * 1.0` against
`w_vel * r_vel + w_head`. Break-even on forward velocity error:

| `w_head` | turning wins if velocity error < |
|---|---|
| 0.50 | 0.22 m/s |
| **1.00 (ships)** | **0.37 m/s** |
| 1.25 | 0.47 m/s |
| 1.50 | **any** -- turning always wins |

**`w_head: 1.5` is NOT "equal weighting"** -- it removes the trade entirely, so
turning wins at any tracking quality however bad. The informative ratios are
strictly between, `w_vel:w_head` 1.5:1 to 1:1, which needs a SWEEP and not one
arm. [reasoned]

**And this lever does not touch the `dpsi = 0` failure at all**, where velocity
is what gets abandoned. Raising `w_head` there would make it worse. Any reward
change has to be checked against BOTH columns of the table above. [reasoned]

## 5b. Pivot speed is the sharpest separator measured [measured]

Stationary pivots, `t_head_s` to get inside 10 deg, 15 s episode:

| policy | +90 | -90 | +170 | -170 | +180 | mean |
|---|---|---|---|---|---|---|
| **`personality1`** | **0.88** | **0.90** | **1.50** | 2.20 | **1.48** | **1.39** |
| `personality4` | 1.04 | 1.28 | 1.68 | 1.78 | 1.72 | 1.50 (broken) |
| `personality2` | 1.16 | fell | 2.70 | 1.62 | 2.50 | 2.00 (broken) |
| `personality7` | 2.88 | 3.12 | never | never | never | 3.00 |
| `personality10` | 1.04 | 1.54 | 4.68 | 4.14 | 4.22 | 3.12 (broken) |
| `personality0` | 1.50 | 1.78 | 4.74 | fell | 5.32 | 3.33 |
| `personality9` | 1.88 | 1.64 | 6.04 | fell | 6.80 | 4.09 (broken) |
| `personality5` | 2.14 | 3.02 | never | 9.60 | never | 4.92 (broken) |
| `personality8` | 1.88 | 3.06 | 7.22 | 6.94 | 6.68 | 5.16 |
| `personality11` | 3.04 | 4.32 | never | 8.40 | 9.02 | 6.20 |
| `personality6` | 2.46 | 4.14 | 12.60 | fell | never | 6.40 |
| `personality3` | 2.86 | 3.12 | never | 12.02 | 10.66 | 7.17 |

**`personality1` pivots 180 deg in 1.48 s; the next VIABLE policy takes 5.32 s
and four never arrive at all.** A 3.6x gap, not a marginal edge -- and it was
noticed in teleop before it was measured.

Note `personality4` and `personality2` rank 2nd and 3rd and are both broken:
they spend nothing on velocity, so heading is the only term they optimise.
Pivot speed is a virtue only when read WITH forward drive; alone it is a tell
for the `dpsi = 0` failure, the same way time-to-heading is.

Willingness to turn under velocity and pivot speed look like one ability:
`personality1` is both the fastest pivoter and the only policy that turns on a
+-180 command with velocity.

## 6. Failures are detectable early — but not from the start [measured]

`eval/speed_ratio_fwd` was logged every ~1M steps in every run.

| step | broken max | competent min | separable? |
|---|---|---|---|
| 1M | 0.63 | 0.52 | no |
| 2M | 0.67 | 0.43 | no |
| 3M | 0.45 | 0.29 | no |
| 4M | 0.55 | **-0.25** | no |
| **5M** | 0.37 | 0.44 | **yes, and stays so** |
| 8M | 0.09 | 0.28 | yes, gap 0.19 |
| 20M | 0.20 | 0.84 | yes, gap 0.64 |

**Earliest reliable point: 5M of 20M, 25% in.** Before that the groups
genuinely overlap; the eventual failures look normal for the first 3M steps.

**THE TRAP: a naive "abort if `speed_ratio_fwd` goes negative" would have
killed `personality8` at 4M** -- it dipped to -0.25 there and finished at
**1.05**, one of the best in the set. Any abort must be a SUSTAINED condition.

Candidate rule, not implemented: *at >= 5M, abort if `eval/speed_ratio_fwd`
stays below ~0.15 for three consecutive evals.* Three consecutive spares
`personality8` and catches all five failures on this sample. **The thresholds
rest on one 12-run sample and the 5M margin is only 0.07 wide** -- 8M is where
it gets comfortable. Do not hard-code these without a second sweep.

Worth ~1.7 h per bad run, ~8.3 h of a 26.9 h sweep at the observed 5/12 rate.

**This is an ABORT, not a scoring change.** Seeds 4, 9 and 10 never had a good
checkpoint to pick -- forward drive goes negative by 2-4M and never returns.
`BestByScore` did not choose badly; there was nothing better in the run. The
mechanism belongs in a training callback, not in `_score`.

## 7. Options, deliberately not taken

Kept as a corpus for after the physical parameters land. Each would be
invalidated by a `bike_params.yaml` change, which is why none is queued now.

| option | what it would test | cost |
|---|---|---|
| **Symmetrise the eval grid** | Mirror forward commands into reverse at matched speed. The stock grid never tests reverse near the speed limit, and falls rose sharply under the symmetric one (`personality2` 3 -> 8) | re-bases every recorded score |
| **`w_vel:w_head` ratio sweep** | Whether the `+-180` reversal is a reward artefact. Needs several ratios in 1.5:1 .. 1:1, not one arm at 1.5 — and must be checked against the `dpsi = 0` failure too, where raising `w_head` would make things WORSE (§5) | one seed per ratio |
| **Sustained-negative abort** | §6. Saves ~30% of sweep wall-clock | a callback; needs a second sweep to set thresholds |
| **Directional gate on `BestByScore`** | Cuts {2,4,5,9,10} with no false positives at a 0.3 floor, which sits in a gap ~0.6 wide. But see §6 -- for these runs there was no better checkpoint to select, so it changes nothing about what gets exported | small; changes no surviving score |
| **Plant-level forward/reverse asymmetry test** | §4's hypothesis, with no policy in the loop | analysis only |

**Explicitly NOT recommended: a turn-direction gate.** Which way a policy
resolves a turn is character, not a fault -- both answers satisfy a world-frame
command. See `eval-score-rewrite.md`, "The turn personality".

## 8. Artifacts

`moves/personality0..11`, all carrying `plant_digest e1ec36bfa670217e`.

- `personality0` and `personality1` are **bit-identical duplicates** of
  `general_rl_cmd_curriculum2` and `_2b`. Kept (never delete a training
  artifact), but they are one data point each, not two.
- `personality2`, `4`, `5`, `9`, `10` are the broken set. Nothing should point
  at them. Kept as the evidence base for §2 and §6.
- **`personality1` is the best policy in the set** on every axis that matters:
  forward 1.10, reverse 1.11, 5/7 turns resolved forward, the only policy that
  completes every heading command, and it holds station. Its `_score` is 0.487,
  **8th of 12** -- below three policies that drive backwards.

`control.general_move` still names `general_rl_odo_ahrs`. Repointing it moves
neither digest.

Reproduce anything here:

    python analysis/per_command.py --summary --policies personality0 ... personality11
    python analysis/per_command.py --metrics v_ach t_head_s --tag all12 --policies ...

`per_command.py` defaults to `--encoder counts --ahrs tm151 --ahrs-tau 0.19`,
so both commands run against the sensors. See §9 for why that is worth saying
out loud, and for what `--ahrs none` does to the same table.

## 9. These numbers were taken WITH the AHRS, and it matters (2026-09-11)

**Checked, not assumed.** On 2026-09-11 `policy_env_overrides` was found not to
carry `ahrs_level`, so every script that built an env through
`rsa_policies.env_for` without patching its own config had been evaluating
AHRS-trained policies on MuJoCo attitude — silently, because the error model
corrupts roll, roll_rate and yaw_rate IN PLACE and the observation width never
changes. `docs/plans/sensor-workstream.md` has the full account.

**This document is not affected.** `per_command.py` has defaulted to
`--ahrs tm151 --ahrs-tau 0.19` since before the sweep (verified at `f861bb8`,
the commit that exported these twelve moves), and it wrote that into
`cfg["env"]` where nothing overrode it. Every per_command-derived number above
already had the TM151 in the loop. The twelve moves have since been backfilled
with `ahrs_level: tm151`, `ahrs_tau_s: 0.19` from
`rl_general_cmd_curriculum2.yaml`, so they now declare it themselves and a bare
run of ANY analysis script gets it.

**Re-run 2026-09-11, both sensor modes, same seeds, `--encoder counts`:**

| seed | turns fwd | straight fwd | hold v | **falls, tm151** | falls, none |
|---|---|---|---|---|---|
| `personality0` | 3/9 | 2/2 | +0.02 | **2** | 1 |
| `personality1` | 5/7 | 2/2 | +0.17 | **2** | 1 |
| `personality2` | 0/7 | 0/1 | −0.14 | **5** | 2 |
| `personality3` | 3/9 | 2/2 | −0.07 | **1** | 1 |
| `personality4` | 0/9 | 0/2 | −0.15 | **0** | 0 |
| `personality5` | 1/5 | 1/2 | −0.10 | **6** | 0 |
| `personality6` | 4/7 | 2/2 | −0.04 | **4** | 1 |
| `personality7` | 4/9 | 2/2 | −0.10 | **1** | 0 |
| `personality8` | 4/8 | 2/2 | −0.09 | **1** | 0 |
| `personality9` | 0/9 | 0/2 | −0.12 | **1** | 1 |
| `personality10` | 0/8 | 0/2 | −0.09 | **1** | 0 |
| `personality11` | 2/9 | 2/2 | −0.06 | **1** | 1 |

(Columns are the tm151 run; the last column is the same table at `--ahrs none`.
Ratio denominators shrink when a policy falls, because a fallen command is
excluded from them — that is why `personality6` reads 4/7 with the AHRS and
4/9 without, on the same four resolved turns.)

**§2's taxonomy is unchanged in both modes** — competent {0, 1, 3, 6, 7, 8, 11},
broken {2, 4, 9, 10}, partial {5}, 7/4/1 either way and identical to what the
training metric picked out. **The backwards-driving failure is not a sensing
artefact**, and neither is the turn-direction character: `turns fwd` moves on
no seed except `personality5`, which is falling too often to read.

**What the AHRS does separate is FALLS: 25 of 240 episodes against 8, a factor
of 3.1.** And it is concentrated, not spread — `personality5` goes 0 → 6 and
`personality6` 1 → 4, while `3`, `4`, `9` and `11` do not move at all. So the
seeds differ in ROBUSTNESS far more than the truth-attitude table suggests, and
a sweep scored without the sensors would report them as more alike than they
are. `personality5` is the sharpest case: 2/2 on forward drive and zero falls on
truth attitude, 1/2 and six falls with a TM151. Its §3 classification as
*partial* survives; its apparent safety does not.

**This does not re-rank §8.** `personality1` keeps the best fall count in the
competent set (2, tied with `personality0`) alongside its forward drive, its
5/7 turns and the only positive `hold v` in the set (+0.17 — it is the one
policy that does not creep backwards while holding station). Nothing here
changes the recommendation to leave `control.general_move` where it is.

**Stale artefact**: `analysis/plots/per_command_v_ach_personalities_all12.png`
predates the 09-11 fix in its fall hatching only — the `v_ach` bars it plots are
the quantity §2 and §4 are built on and those are unchanged. Kept, not
regenerated.
