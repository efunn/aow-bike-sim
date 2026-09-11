# The sensor workstream — what the bike can actually feel

> **Status: LARGELY DONE, and it is the reason the current policies work.**
> The simulator models the onboard velocity estimate, encoder quantisation and
> the TM151's attitude error, in the loop and in training. A policy trained on
> MuJoCo truth survives 0.20 of the eval grid against 1.00 for one trained
> against the sensors. Open: the DYNAMIC attitude accuracy, which the datasheet
> bounds rather than measures, and which needs a moving unit with independent
> attitude truth — i.e. a bike.

Assembled 2026-09-08 from `docs/status.md`, which had been the only record of
this arc. Everything below is verbatim and carries its own dates; nothing here
was re-measured on the day it moved.

Related: `odometry-rewrite.md` (the estimator itself, and why it was not
rewritten), `eval-score-rewrite.md` (how these policies were scored, and why
the early numbers hid a failure), `general-rl-improvements.md` (the RL findings
these build on).

---

### The pitch workstream — why crab does not work

This is the substantive control finding of the period, and it is a negative
result that closed a hypothesis rather than a win.

**The hypothesis.** The differential is the only actuator for both roll balance
and lateral crawl, and under a hold command it is already p95-saturated by
balance alone. A sustained pure crab is therefore not physically available at
standstill. The only manoeuvre that produces net lateral motion is an
oscillatory wriggle — the parallel-park bracket — but that has instantaneous
speed far above commanded speed, and the baseline reward scores *instantaneous*
velocity error every step. Measured: at a 0.144 m/s crab command, a perfect
wriggle scores *worse than standing still*. So the reward forbade the one
manoeuvre that could satisfy the command.

**Arm 1 — `general_rl_glide_og`** low-passed the measured velocity
(`vel_window_s`) so the gait's time average is what gets tracked. It worked, in
the sense that the policy did start oscillating — 2× more than any other policy
against a common 1 s reference. **But the oscillation was pitch, not the
bracket**, and pitching produces no lateral travel: net `v_lat` under a crab
command came out at 0.064 m/s, *below* `general_rl_smooth_diff_og`'s 0.082 from
plain per-channel smoothing. It also cost hold quality (drift 0.20 → 1.46 m).

**Why pitch was free.** `w_upright` priced roll and *nothing priced pitch*, and
a time-averaged velocity reward makes a pitch oscillation free twice over —
a wheelie that comes back down averages to nothing. Arm 1 reached **23° nose-up
with the front wheel 79 mm off the floor** on a plain accelerate command.

**Two tools were built to see this, and both earn their keep.**

- `analysis/liftoff.py` — supersedes the airborne-*percentage* columns in
  `hold_spectrum.py` and `chatter.py`, which are actively misleading. Binary
  "is the wheel touching" counts the rear omni's own ~0.6 mm envelope ripple
  identically to a visible wheelie: the rear reads 40–60% "airborne" while
  never clearing 5 mm, the front reads about the same while clearing 79 mm.
  Same percentage, two orders of magnitude apart in behaviour. It reports the
  gap *distribution* in mm, and separates pitch from hop by comparing measured
  pitch, implied pitch, and common lift. `corr(front gap, pitch) = 0.999`
  confirmed arm 1's liftoff was pitch, not hopping.
- `analysis/pen.py` — the top-down ground track per command, which is what the
  operator actually sees. A mean velocity cannot represent "it draws a slow
  squiggly repeated S"; a squiggle whose lobes cancel and a straight line have
  the same mean.

**Arm 2 — `general_rl_glide_pitch_og`** changes three things, each aimed at one
link in the chain: `v_lat_frac` 0.4 → 0.12 (at 0.4 the crab command reaches
102% of the differential's kinematic ceiling and **8.1% of sampled commands are
impossible for any controller**; at 0.12 that is 0.0%, and the optimal response
to a mostly-impossible channel is to ignore it, which is what every policy so
far has done), `w_pitch` 0.0 → 2.0 (a 23° wheelie now costs ~11% of the
per-step maximum — priced, not forbidden), and `obs_pitch: true` (required by
the second: charging for state the policy cannot see is a known defect).
Observation is 19 wide, and **width is not the contract** — the move yaml
records `obs_layout` and replay compares it element-wise.

**The exit criterion is already written down, and it matters:** if crab is
still flat *and* pitch is now controlled, the wriggle hypothesis is dead and
the honest next step is the open-loop gait sweep, not another arm.

---


#### Policy exports, current standings

| export | survive | track | vel_err | head_err | reverse | notes |
|---|---|---|---|---|---|---|
| `general_rl_og` | 0.92 | 0.702 | 0.225 | 9.9° | yes | 10M final |
| `general_rl` ← **config default** | 1.00 | 0.763 | 0.291 | 2.5° | **no** | 6M snapshot, pre-contact-change |
| `general_rl_1k` | 1.00 | 0.816 | 0.163 | 3.1° | — | loops out on reverse+180 (up_z −0.99) |
| `general_rl_smooth_og` | 1.00 | 0.816 | 0.167 | 3.6° | 1.006 | crab 0.21 / 0.30 |
| `general_rl_smooth_diff_og` | 1.00 | 0.800 | 0.166 | 4.3° | 0.983 | crab 0.21 / 0.28; best net v_lat (0.082) |
| `general_rl_smooth_stiff` | 1.00 | 0.805 | 0.191 | 3.0° | 1.122 | crab 0.39 / **0.02** |
| `general_rl_glide_og` | — | — | — | — | — | arm 1: oscillates 2×, but it is **pitch**; 23° nose-up |
| `general_rl_glide_pitch_og` | — | — | — | — | — | arm 2: `v_lat_frac` 0.12, `w_pitch` 2.0, 19-wide obs |

**DONE (`b2580fd`): `control.general_move` now points at
`general_rl_smooth_diff_pi`.** It was outstanding across four snapshots. The
and it cost NOTHING. **Correcting a claim made here on 2026-08-26**: this
said the pointer "lives in `bike_params.yaml`, so `design_digest` moved for a
change with no physical content". That is false, and it was carried from
CLAUDE.md's *Current state* section, which predates the plant/design split in
`fd10bc4`. `plant_digest` excludes the whole `control` subtree
(`params.py:114`) and `design_digest` covers only `rate_hz`, `lqr` and
`drive.speed_grid` (`DESIGN_FIELDS`, `params.py:139`). `control.general_move`
is in neither. Verified by measurement: repointing to `general_rl_odo` leaves
both at `e1ec36bfa670217e` / `2db6c647ff3a2d59`. The comment at
`bike_params.yaml:288` already said so.

**AND IT WAS ALREADY THE WRONG DEFAULT, for a reason that has nothing to do
with driving quality** — see the odometry section. `smooth_diff_pi` is trained
on MuJoCo truth and scores 0.808 on the eval grid; handed the onboard velocity
estimate, which is what the Pi will give it, it scores **0.044 and survives
0.15 of the grid**. Do not push it to hardware.

**SUPERSEDED 2026-08-28: `control.general_move` now names
`general_rl_odo_ahrs`**, which trained against both the velocity estimate and
the TM151 attitude and survives 1.00 of the grid with them in. Digests
re-checked after the edit and unmoved. `smooth_diff_pi` stays as the ceiling
reference — 0.808 / 1.00 on truth is what perfect sensing is worth — reachable
with `--general` for one session. Keep the number, retire the pointer.

#### Which exports match the plant (2026-08-26, re-counted 08-28)

**11 of 45 exports carry the current `plant_digest` `e1ec36bfa670217e`**
(re-counted 2026-08-28, after `..._tau019` and `..._rand2` landed):
`general_rl_smooth_diff_pi`, `general_rl_pitch_smooth_diff_pi`,
`general_rl_glide_pitch_smooth_pi`, `general_rl_odo`, `general_rl_odo_ahrs`,
`general_rl_odo_ahrs_rand`, `general_rl_odo_ahrs_tau019`,
`general_rl_odo_ahrs_rand2`, `general_rl_nolat`, `general_swing_rl`,
`general_swing_open_rl`. Another 34
have no `plant_digest` field at all and predate it.

This supersedes the old reading that "matches the digest" and "drives well"
pick out different policies — that was true when only the two `swing` exports
matched, and the plant/design split plus a backfill recovered the rest. The
good drivers now match. Judge by the export's own field, never by filename or
mtime.

#### Trained against the ONBOARD ESTIMATE (2026-08-26)

| export | trained-on: score / surv | on the ESTIMATE: score / surv | notes |
|---|---|---|---|
| `general_rl_smooth_diff_pi` | 0.808 / 1.00 | **0.044 / 0.15** | truth-trained; does not survive deployment |
| `general_rl_odo` | 0.772 / 1.00 | **0.772 / 1.00** | observes the estimate during training |
| `general_rl_nolat` | 0.780 / 1.00 | 0.742 / 1.00 | never observes `v_lat`; 12M, remote |

One grid, 20 commands, identical seeds, randomization off
(`analysis/chatter.py --force-odometry`). Both digests current
(`plant_digest e1ec36bfa670217e`).

`odo` gives up 0.036 of score on truth and keeps all of it on the estimate. It
is also **6x quieter** — summed per-step action change 0.251 against 1.566 and
1.548, pinned to an actuator bound 1.2% of steps against 50.6% and 37.2%, and
at rest 0.003 against 2.049. What it costs: drift 0.956 m against 0.296, and
crab partly given up (0.35/0.52 against 0.49/0.58).

`nolat`'s crab ratios look excellent (0.92/1.11) and are an artefact —
`crab_head_err` 98.6° means it turned to face the crab direction and drove
forward. It is also **not immune to the estimator**: it still takes `v_lon`
from it, which costs 0.780 → 0.742 and 7x the drift.

#### Trained against the AHRS too — `general_rl_odo_ahrs` (2026-08-27)

**The hypothesis held.** `config/rl_general_odo_ahrs.yaml` added the encoder
model and a TM151 attitude error to `rl_general_odo.yaml` and committed in
advance to what would count. Eval grid at `--encoder counts --ahrs tm151`:

| | `general_rl_odo_ahrs` | `general_rl_odo` |
|---|---|---|
| score / **survival** | 0.672 / **1.00** | 0.526 / **0.90** |
| track_geo | 0.672 | 0.585 |
| vel_err | 0.198 | 0.272 |
| **head_err** | **7.8°** | 34.5° |
| drift_m | 1.287 | 1.606 |
| steer_rest | 2.2° | 4.9° |

Training on a wrong-but-correlated attitude does for roll what training on a
wrong-but-correlated velocity did for `v_lat`. Heading error is the headline:
**34.5° → 7.8°**, better than any estimate-trained policy has managed.

Confirmed in teleop: it holds in common scenarios where `general_rl_odo` now
falls with the AHRS in. **Known edge case: forward travel plus a 180° heading
flip can still drop it.**

**What it cost.** Chatter roughly doubled — summed per-step action change 1.160
against 0.566, saturation 30.9% against 18.3%, and at rest 1.852 against 0.631.
It answers attitude noise by working harder, which is the opposite of what
`odo` did to velocity noise. And **crab left collapsed to 0.12** (from 0.44)
while crab right holds at 1.09 — though `crab_head_err` 5.1° against 86.8° says
it is now genuinely crabbing rather than turning to face, so the two numbers
have to be read together. Cross-axis leakage went the other way too: crab now
scoots forward at +0.366 m/s against −0.214.

#### The real TM151, measured (2026-08-27)

300 s stationary on a desk at 50 Hz, via `analysis/tm151_record.py` →
`tm151_check.py`. First real-hardware numbers in the project:

| quantity | measured | `sim_ahrs` | |
|---|---|---|---|
| roll RMS at rest | **0.0142°** | 0.5° | 35x better than modelled |
| pitch RMS at rest | 0.0159° | 0.5° | 31x better |
| orientation tau | **0.19 ± 0.01 s** | 2.0 s (guess) | 10x faster, r² 0.999 |
| gyro σ | 0.128 °/s | 0.083 | model optimistic |
| gyro p-p over 1 s | 0.575 °/s | ≤0.5 spec | over spec |
| yaw drift | −0.031 °/min | 0.12 (inertial) | the compass is working |
| gyro bias | −0.00043 °/s | — | negligible |

**Do not lower the model on the 35x.** One unit, room temperature, dead still;
the datasheet bound presumably covers unit spread, temperature and mounting.
And it says nothing about the DYNAMIC row, which is what every eval result
above turns on. **Do not conclude the gyro is out of spec either** — a desk is
not vibration-free, and building and fan motion is real rotation the gyro is
right to report. Re-measure on foam before believing 0.575 > 0.5 is the part.

Gauss-Markov was the right SHAPE (r² 0.999); only the timescale was wrong.

#### THE TAU IT TRAINED ON IS 10x WRONG, AND FOR THIS POLICY THAT MATTERS

**Superseded in part — see "The tau question, answered" below.** The table
here is still correct; the CONCLUSION drawn from it ("for this policy that
matters") turned out to overstate a 1-in-20 effect. Kept as written because it
is the record of a correction, and the correction was itself corrected.

`sim_ahrs.TAU_ORIENT_S` is 2.0 s, a guess. The real part measures **0.19 s**
(300 s at rest, exponential fit r² 0.999 — see `analysis/tm151_check.py`).
Re-evaluating at the measured value:

| policy | tau 2.0 (guess) | tau 0.19 (measured) |
|---|---|---|
| `general_rl_odo_ahrs` | 0.672 / **1.00** | 0.570 / **0.95** |
| `general_rl_odo` | 0.526 / 0.90 | 0.506 / 0.90 |

**This corrects a claim made here on 2026-08-27** that the tau guess "changed
nothing". That was measured on `general_rl_odo`, which never trained against
the AHRS — and it is still true for it (0.526 → 0.506, inside seed noise). It
is FALSE for `odo_ahrs`, which trained at tau 2.0 and partly learned that
timescale: at the real tau it gives back half its gain, 1.00 → 0.95 survival.

It still beats `odo` at the measured tau (0.570 / 0.95 against 0.506 / 0.90),
so the run was worth making. But **the headline 1.00 is partly an artefact of
training and evaluating at the same wrong number**, and the next run should
RANDOMISE tau rather than pick one — the dynamic tau is unmeasured anyway, and
0.19 s is a resting figure.

#### Pricing the TM171 — the part ordering FLIPS SIGN with the unmeasured tau (2026-08-28)

The existing TM171 row (0.689 against 0.537) was measured on `general_rl_odo`,
which never trained against an AHRS — it prices the part for a policy caught
unprepared. Priced instead for the policy we would actually fly, on the full
2×2 (`analysis/chatter.py --policies general_rl_odo_ahrs --encoder counts`):

| `general_rl_odo_ahrs` | tau 2.0 (**trained on**) | tau 0.19 (**measured**) |
|---|---|---|
| `--ahrs tm151` (1.5°, what we own) | **0.672 / 1.00** | 0.570 / 0.95 |
| `--ahrs tm171` (1.0°, the upgrade) | 0.632 / 1.00 | **0.696 / 1.00** |

**The better part scores WORSE at the tau the policy trained on**, and better at
the measured one. Survival is 1.00 in three of four cells; the single 0.95 is
`tm151` at the measured tau, and the TM171 is what recovers it. Heading error
moves the same way and no further: 7.8° → 12.9° at tau 2.0, 17.3° → 15.4° at
0.19.

**The purchasing answer is therefore "not yet, and not for this reason".** The
0.04 gap at tau 2.0 is barely outside the ±0.02 seed-noise floor, but the
0.126 gap at 0.19 is well outside it — so the effect is real and its SIGN
depends on a constant nobody has measured in motion. A part-quality ordering
that inverts under an unmeasured parameter is not a result you can spend money
on. `TAU_ORIENT_S` was a training contract, and it turns out to be one the
sensor ablations are also conditioned on.

What this does answer, and it is the useful half: **the fix for sensor error
was training against it, not spending money.** Every cell here survives at
0.95 or better, against 0.20 for the truth-trained default. The residual the
TM171 could buy is `track_geo`, and it is worth 0.02–0.13 depending on a number
we do not know.

**Re-ask this after `general_rl_odo_ahrs_rand`** — a policy trained over the
tau range should give a monotone ordering, and a monotone ordering is one you
can price. (Ablated earlier and not re-derived: orientation RMS 1.5° → 1.0°
carries essentially all of the TM171 difference; yaw drift and misalignment do
nothing. So this is a one-parameter question wearing a part-number label,
which is exactly why randomising that one parameter is the prerequisite.)

#### Randomising the AHRS error parameters — TRIED, AND IT FAILED (2026-08-28)

**The mechanism works and the run lost.** Keep the mechanism; do not repeat the
ranges.

`sim_ahrs.SimAhrs.set_error_params` re-points the two unmeasured parameters,
and `general_env.reset` draws them log-uniform per episode when
`randomization.ahrs_orient_rms_deg_range` and `ahrs_tau_s_range` are set. Note
what was and was not already random, because that distinction is the entire
content of the change:

| | randomised before? |
|---|---|
| error REALISATION — noise trajectory, misalignment draw, orientation walk | **yes**, fresh seed per episode |
| the PARAMETERS — orientation RMS, `ahrs_tau_s` | **no**, one fixed value every episode |

**The verdict.** `general_rl_odo_ahrs_rand`, 12M steps against
`config/rl_general_odo_ahrs_rand.yaml` (RMS 0.05–2.0°, tau 0.1–3.0 s). One
grid, identical seeds, randomization off, `--encoder counts --ahrs tm151`:

| | tau 2.0 | tau 0.19 |
|---|---|---|
| `general_rl_odo_ahrs` | **0.672 / 1.00** | **0.570 / 0.95** |
| `general_rl_odo_ahrs_rand` | 0.480 / 0.85 | 0.457 / 0.75 |

Worse at both taus and worse on SURVIVAL at both, on twice the steps. It *did*
flatten the score across tau — spread 0.102 → 0.023 — so it is genuinely less
tau-sensitive, **by being uniformly worse rather than by being robust.**
Survival degrades more in relative terms (0.85 → 0.75) than the specialised
policy's (1.00 → 0.95).

**Why, and it was predicted in the config header rather than diagnosed after.**
Log-uniform over a 40x band puts the MEDIAN episode at **0.32°** and only
**7.8%** at or above the 1.5° design case. It trained mostly on a sensor
cleaner than the one it flies. The old header said: *"if the policy comes back
robust across tau but worse than `general_rl_odo_ahrs` at `tm151` itself, raise
the floor to ~0.3° rather than abandoning the log."* That is exactly what
happened.

**And `_score` could not see the damage.** It rose monotonically to the final
checkpoint — 0.302, 0.410, 0.444, 0.541, 0.579 — while steering chatter
doubled (`dsteer²` 1.183 against 0.535). `_score` is survival × tracking and
prices neither chatter nor rest behaviour, so `BestByScore` exported the last
checkpoint with no way to know. This is the `eval-score-rewrite.md` argument
reproducing itself on a fresh run, and it is the strongest evidence for it yet.

**A caveat on `steer_rest_deg`, because it misled once already.** It reads 41.7°
for `_rand` against 2.2°, but it is `mean(|steer|)` over the tail of a SINGLE
hold episode — the 20-command grid contains exactly one `(0,0,0)`. It therefore
cannot distinguish a fixed offset from a symmetric saw. Teleop shows the wheel
mostly pointing straight, which together with the doubled `dsteer²` says
OSCILLATION, not a cocked wheel. **Read it with `dsteer²`, never alone**, and do
not quote it as evidence of a resting offset.

**What survives the failure, and is worth keeping:** the ranges live under
`randomization:`, not `env:` — a training-time distribution rather than part of
the observation contract, so an eval with randomization off runs at the nominal
point and every table above stays comparable. They are read by `general_env`,
not `DomainRandomizer`, which perturbs the MuJoCo model and the sensor is not
in the model. Absent keys consume no random numbers, so every config predating
this reproduces bit for bit (asserted,
`test_absent_ranges_leave_the_rng_stream_untouched`).

#### The tau question, answered (2026-08-28)

Both configured follow-ups landed. One grid for all five policies —
`analysis/ahrs_tau.py`, 20 commands, identical seeds, randomization off,
`--encoder counts --ahrs tm151`:

| policy | trained at tau | eval tau 2.0 | eval tau 0.19 | spread |
|---|---|---|---|---|
| `odo_ahrs` | 2.0 (guess) | **0.672 / 1.00** | 0.570 / 0.95 | 0.102 |
| `odo_ahrs_tau019` | 0.19 (measured) | 0.563 / 1.00 | 0.551 / 1.00 | 0.012 |
| `odo_ahrs_rand` | 0.1–3.0 (wide) | 0.480 / 0.85 | 0.457 / 0.75 | 0.023 |
| `odo_ahrs_rand2` | 0.1–0.6 (narrow) | 0.654 / 1.00 | **0.663 / 1.00** | **0.009** |
| `odo` | never saw an AHRS | 0.526 / 0.90 | 0.506 / 0.90 | 0.020 |

**Against the bars written before the runs.** `tau019`'s bar was *beat 0.570 /
0.95 at tau 0.19*: it returned **0.551 / 1.00** — missed on score, met on
survival. Pinning the constant at its measured value bought robustness and
paid for it in tracking; it is not an improvement. `rand2`'s bar was the
specialised policy at both taus: it returned **0.654 / 1.00** and **0.663 /
1.00** — just under at tau 2.0, comfortably over at 0.19, which is the number
the bike has. **`rand2` is the run that worked**, and it had the better
training eval of the two as well (its own export block: `track_geo` 0.659
against `tau019`'s 0.542, `vel_err` 0.175 against 0.224).

**THE CONCLUSION: the AHRS error's SIZE matters enormously and its
CORRELATION TIME does not.** Training against the error at all is worth ~0.15
of score and 27° of heading (`odo_ahrs` against `odo`). Moving tau by 10x,
across four policies, moves score by at most 0.102 and survival by one episode
in twenty. Four independent reasons this is not an artefact of the grid:

1. **tau cannot change the error's size.** `_gm_step` is a *stationary*
   Gauss-Markov process — the innovation carries `sqrt(1-a²)` precisely so the
   standing deviation stays at the level's RMS for every tau. Measured, held
   still, 60 s at 50 Hz: RMS is flat at ~1.45° from tau 1e-5 to 60 s while the
   mean step-to-step change goes 1.67° → 0.03°. tau sets the wander rate, full
   stop.
2. **Both ends of the range are benign**, as `sim_ahrs.py:236` predicted before
   any of this was measured: white noise the loop averages away at one end, an
   offset it trims out at the other. The cost is a shallow interior bump, not a
   monotone.
3. **The policy that never saw an AHRS flips MORE episodes than the one
   accused of specialising.** Per-command, `odo_ahrs` disagrees with itself
   across tau on exactly one command (cmd 19, a `dpsi 180`) — while
   `general_rl_odo`, which never trained against an AHRS at all and therefore
   cannot have learned a timescale, flips **four**, and `odo_ahrs_rand` flips
   **six**. If these flips measured learned tau specialisation, that ordering
   would be impossible. They measure marginal episodes moving under a
   different noise realisation.

   And `odo_ahrs`'s one flip is a `dpsi 180` command, which is already
   recorded below as an edge case it can fail independent of tau — it lands
   exactly where the policy was already marginal.
4. **The heading gap is an artefact of the metric, and it INVERTS when the
   metric is fixed.** `odo_ahrs` reads 7.8° at tau 2.0 and 17.3° at 0.19 — but
   `head_err_deg` is the error at the LAST STEP of an episode, so a
   20-command mean of it is a mean of twenty single samples, and one abandoned
   command moves it by ~9°. On the whole-episode median (`head_err_med`) the
   same policy reads **10.1° at tau 2.0 against 8.1° at 0.19** — slightly
   BETTER at the measured value. The mean is moved by one episode that never
   turned around at all (172° final error) plus the fall.

   (An earlier version of this section quoted "median 5.5° vs 5.3°". That was
   the median ACROSS COMMANDS of the same last-step samples — a robust
   aggregate of the wrong quantity. The 10.1/8.1 above is the median over the
   episodes themselves. The conclusion did not change; the numbers did.)

   **That abandoned episode is cmd 5 (`v_lon 0.80, dpsi 180`), and it is the
   worst episode for EVERY policy in the table at BOTH taus** — 171–180° of
   final error in all ten cells. It is a shared blind spot of the whole
   SENSOR-TRAINED family, not a tau effect and not specific to `odo_ahrs`.

   **It is not a limit of the bike, and an earlier version of this line said
   "nothing turns around from 0.8 m/s on a 180° command", which is wrong.**
   `general_rl_pitch_smooth_diff_pi`, on MuJoCo truth, completes cmd 5 at a
   **2.1° median** and both `fwd + crab + 180` commands at ~2°. The manoeuvre
   is available in the plant; the AHRS-trained policies have lost it. See "The
   ideal-sensor reference" below.

**The model stays and `TAU_ORIENT_S` moves to the measurement: 2.0 → 0.19.**
An error that jumps independently every control step is not what a fusion
filter does, the model costs nothing, and 0.19 s has an r² of 0.999 behind it.
The constant was held at 2.0 as a TRAINING CONTRACT — moving it silently
reprices `odo_ahrs` — and that repricing is now measured rather than feared:
0.672 / 1.00 → 0.570 / 0.95 by default, the largest tau sensitivity in the set
and still one episode in twenty. tau was carried as a live risk to every
sensor result and it is not one, so the constant is now simply the number the
part has.

**Nothing was retrained for it.** The four AHRS policies stand as exported and
are re-scored at the new default. Three consequences to know:

- `config/rl_general_odo_ahrs.yaml` now **pins `ahrs_tau_s: 2.0` explicitly**.
  It previously relied on the fallback, so moving the default would have
  silently changed what a re-run of that config trains — the export would no
  longer be reproducible from its own config. The randomised configs draw from
  a range and never read the default at all.
- `general_env` no longer hardcodes `2.0` as its fallback; it reads
  `TAU_ORIENT_S`.
- `analysis/chatter.py --ahrs-tau` defaults to `TAU_ORIENT_S` too, so **a
  chatter table taken after 2026-08-28 is at tau 0.19 unless it says
  otherwise, and every table in this file dated before that is at 2.0.**
  Where both are quoted the column headers say which.

**What none of this rests on: a hand-flown demonstration. There isn't one.**
Every number above is fixed-seed episodes on the eval grid. Attempts to find a
teleop case failed — the commands that separate the taus are the same ones
these policies drop at either tau, and `--ahrs-tau 0.19` versus `2.0` is not
distinguishable by feel. Treat any single flipped episode as an anecdote; the
seed-noise floor on `score` is about ±0.02, and one episode in twenty is 0.05
of survival.

**A trap this run walked into, recorded so the next one does not.**
`general_rl_odo` predates the `odometry_encoder` field, so a grid that does
not force the encoder silently scores it on `ideal` and it comes out at 0.649
/ 1.00 at tau 0.19 — apparently the best policy in the set. It is being run on
a different bike. `analysis/ahrs_tau.py` takes `--encoder` and defaults it to
`counts` for exactly this reason, and prints which encoder produced the table.

**Still open, and not answered by any of this:** 0.19 s is a RESTING figure.
The dynamic correlation time is unmeasured, as is the dynamic RMS — and the
RMS is the half that matters. `rand2`'s narrow span (RMS 0.3–2.0°, tau
0.1–0.6 s) is the current answer to that ignorance and is the reason it
generalises across tau at no cost.

#### The eval metrics were single samples, then they were blind (2026-08-28)

Two defects, found one after the other, both in REPORTING only — the reward
reads `psi_err` every step, and `_score` is `survive_rate × track_geo` where
`track` is already an episode mean (`general_env.py:775`). Selection was never
affected. Every human decision read off these tables was.

**One: the values were last-step samples.** `head_err_deg`, `vel_err` and
`drift_m` on a row are the values at the LAST STEP of the episode, so a grid
aggregate is a mean of twenty single samples and any one abandoned command
hijacks it. `_eval_episodes` now carries the whole-episode series.

**Two, and worse: the whole-grid median is blind by construction.** A median
over 20 commands cannot see a failure affecting fewer than 10 of them, because
the middle two never reach the tail of the sorted list. The grid holds exactly
6 large turns, so **no whole-grid median can ever report a large-turn
failure** — and none did:

| policy | hold (1) | straight (2) | crab (2) | turn≤90 (8) | turn≥170 (6) | ALL (20) |
|---|---|---|---|---|---|---|
| `odo_ahrs` | 4.8° | 6.8° | 16.3° | 5.4° | **65.7°** | 8.1° |
| `odo_ahrs_rand` | 12.1° | 1.6° | 32.9° | 5.3° | **100.8°** | 10.9° |
| `odo_ahrs_tau019` | 9.9° | 4.1° | 10.3° | 3.7° | **156.7°** | 11.1° |
| `odo_ahrs_rand2` | 8.2° | 2.7° | 8.0° | 3.6° | **152.1°** | **7.4°** |

The ALL column INVERTS the ranking on the axis that matters: `rand2` has the
best whole-grid median and is second worst on large turns.

**The fix is a `by_family` block** beside the whole-grid scalars, not in place
of them — `n_eval`, `survive_rate`, `track` and `track_geo` are untouched, so
this re-bases nothing and every metrics block already in `moves/*.yaml` stays
comparable. Five families that PARTITION the grid, ~20 numbers against the
~160 of the full metric × command matrix. **Implemented** —
`train_general_rl.FAMILIES` and `_by_family`, reported by
`analysis/chatter.py` and written into every future export's metrics block.
The argument is item **F** of `docs/plans/eval-score-rewrite.md`.

| family | n | what it is |
|---|---|---|
| `hold` | 1 | v = 0, dpsi = 0 — the only place drift is defined |
| `spin` | 2 | v = 0, in-place ±90 |
| `cruise` | 9 | moving, \|dpsi\| < 170 |
| `crab` | 2 | \|v_lat\| > 0.1 |
| `turn_big` | 6 | \|dpsi\| ≥ 170 |

Per family: `survive_rate`, `t_head_s` (over the members that ask for a
heading change), `head_err_tail`, `vel_err_med`; plus `drift_m` and
`drift_overshoot` on `hold`.

**`spin` exists because it fell through every predicate in the first draft**
and was invisible — and it is the family where the policies differ most
(`rand2` reads 52° there against ~2° on the same 90° turn while moving). Two
invariants now guard that, on integer counts rather than the rounded rates:
families must cover every command exactly once, and their surviving-episode
counts must sum to the grid's.

**What survives the reduction, and what does not.** `head_err_med` restates
`t_head_s` on turns and tracks `head_err_tail` at a steady 2–2.5× elsewhere.
`vel_err_tail` spreads 19% across policies against `vel_err_med`'s 67%.
`drift_max` equalled `drift_m` exactly in 3 of 4 policies and `drift_sd` sat
at 0.267–0.315 × `drift_m` in all four — the ratio for a linear ramp — so both
are replaced by `drift_overshoot` (peak minus final), the same information
with a null value of zero. `t_head_s` and `head_err_tail` look redundant and
are not: `odo_ahrs` and `rand` arrive at large turns at 6.73 s and 6.78 s
while holding them at 14.7° and 105.0° — `rand` touches 10° once and wanders
back off.

**READ `survive_rate` WITH `t_head_s`, NEVER ALONE.** On `turn_big`,
`odo_ahrs` reads 0.83 / 6.73 s / 14.7° and `rand2` reads 1.00 / 15.00 s /
152.3°: `odo_ahrs` attempts the turn, completes it, and falls on 1 of 6;
`rand2` never turns at all — it drives BACKWARDS at ~0.8 m/s, which
satisfies the world-frame velocity command exactly (`vel_err_med` 0.091)
while abandoning the heading (175.6°), and therefore never falls.
**Survival bought by refusal** — the pattern `track_geo`'s geometric mean was
introduced to defeat at the grid level, reappearing inside a family.

**Consequence for `control.general_move`.** It names `odo_ahrs`, and the case
for that has now been read three ways in one day: the last-step mean said
`odo_ahrs` held heading far better (17.3° against 33.8°); the whole-episode
median said `rand2` swept everything but chatter (8.1° against 7.4°); per
family it is a trade again, and a different one from the first.
Indistinguishable on `cruise` and `crab`; `odo_ahrs` clearly better on `spin`
(2.40 s against 5.17 s) and the only one that completes a large turn
(`head_err_tail` 14.7° against 152.3°); `rand2` holds less than half the drift
(0.664 m against 1.584 m) and never falls. On `turn_big` the question is
whether a 1-in-6 fall while turning around is worse than never turning around,
which is a decision and not a metric. **OUTSTANDING: whether
`control.general_move` should name `rand2` instead.** It is a one-line config
edit and moves neither digest.

**The 180° turnaround is not a limit of the bike**, and an earlier version of
this file said it was — see "The ideal-sensor reference" below.

Eight figures per arm, one per metric, in `analysis/plots/`:
[heading median](../analysis/plots/per_command_head_err_med.png) and
[time to heading](../analysis/plots/per_command_t_head_s.png) for the four
AHRS policies; the same two with `_ideal` for the truth pair
([heading](../analysis/plots/per_command_head_err_med_ideal.png),
[t_head](../analysis/plots/per_command_t_head_s_ideal.png)). Every figure
stamps its own sensor configuration, because two runs differing only in
`--ahrs` are otherwise identical in every label. **The charts are what made
the family structure visible** — the aggregates could not have shown it.

#### The odo/ahrs line cannot turn around, and it is `obs_pitch` (2026-08-29)

**The finding.** Every policy on the sensor line — `odo`, `odo_ahrs`,
`tau019`, `rand`, `rand2` — fails the three moving 180° commands, and **not by
falling**. It drives BACKWARDS at ~0.8 m/s and declines to turn. `rand2` on
`(0.804, 0, 180)`: `vel_err_med` **0.091** with a heading error of **175.6°**
and `v_ach` **−0.843**. The command is a world-frame velocity vector plus a
heading, so reversing satisfies the velocity half *exactly* while abandoning
the other — and `track` is `0.5 × (r_vel + r_head)`, so that is half marks
with none of the risk of a 180° turn. It is the mirror of the failure
`general-rl-improvements.md` §2.6 records, a policy banking what it can get
and skipping what it cannot.

**It is not the sensors.** `rand2` run on MuJoCo truth behaves identically on
`turn_big` — `t_head` 15.00 s and `head_err_tail` 152.8°, against 15.00 s and
152.3° with the TM151 — while every other family improves as expected. Perfect
sensors change nothing about this manoeuvre.

**It is not the training budget either.** `turn_big` commands completed
(reached 10° and survived), all on truth:

| policy | `obs_pitch` | steps | completed | `v_ach` on the 3 moving |
|---|---|---|---|---|
| `smooth_diff_pi` | false | 11M | 3/6 | −0.85, −0.76, −0.69 |
| `pitch_smooth_diff_pi` | **true** | 12M | **6/6** | +0.79, +0.70, +0.65 |
| `glide_pitch_smooth_pi` | **true** | **5M** | **6/6** | +0.81, +0.58, +0.60 |
| `nolat` | false | 9M | 3/6 | −0.82, −0.70, −0.76 |

Perfect separation on `obs_pitch`, and **the sign of `v_ach` is the tell**:
pitch policies turn round and drive forward, pitchless ones reverse. 5M steps
with pitch beat 11M without.

**This is a rediscovery, not a discovery** — pitch is what solved the flicks on
the old policies, and eight configs in the `glide_pitch` / `pitch_smooth_diff`
family carry `obs_pitch: true`.

**THE MECHANISM: without pitch, the manoeuvre ends the episode.**
`analysis/reverse_flip.py` sets up the case the eval grid cannot — reverse
until the speed is established, then snap the heading 180°, which is what the
teleop `8` key does. Peak pitch over the manoeuvre:

| policy | −0.50 | −0.84 | −1.02 | −1.20 m/s |
|---|---|---|---|---|
| `odo_ahrs` | 13.0° | **86.9° FELL** | **85.7° FELL** | **81.8° FELL** |
| `odo_ahrs_rand2` | 7.2° | 31.9° | **87.6° FELL** | fell before the snap |
| `smooth_diff_pi` | 16.8° | 50.0° | **82.7° FELL** | fell, 61° roll |
| `pitch_smooth_diff_pi` | 7.8° | **9.1°** | **14.5°** | **23.0°** — never falls |

**80–90° of pitch is the bike going over backwards.** Every policy without
`obs_pitch` flips above ~0.84 m/s of reverse; the one with it never exceeds
23° at any reverse speed up to `v_max`. So the pitchless policies are not
failing to turn — **they have correctly learned that attempting it from a
reverse ends the episode**, and the reversing-with-a-175°-heading-error
behaviour the eval reports is the AVOIDANCE, not the failure.

The eval grid cannot see this: its 180° commands start from REST, so the bike
never reaches the reversing state that triggers the flip. It was found in
teleop — hold reverse, press `8` — and only then reproduced headlessly.

**How it was lost.** The sensor workstream forked off `rl_general.yaml`, the
pre-pitch trunk, rather than off the pitch line, and inherited
`obs_pitch: false` BY OMISSION — `rl_general_odo.yaml`,
`rl_general_odo_ahrs.yaml` and `rl_general_odo_ahrs_rand2.yaml` never mention
it. **Nothing flagged it for the whole sensor workstream**, because the
whole-grid eval median cannot see a failure in 3 of 20 commands. That is the
same blindness "The eval metrics were single samples" is about, and this is
what it cost.

**A blocker found while queueing the run: `obs_pitch` was reading MuJoCo
truth.** `_obs` decoded the corrupted AHRS quaternion, **discarded its pitch
component**, and fed the policy `s.pitch` from the model — so an `obs_pitch`
policy read the TM151 for roll and perfect truth for pitch. The gyro's middle
axis is the pitch rate and was likewise never read. Measured before the fix:
obs roll 0.000° from the AHRS and 15.07° from truth; obs pitch **0.000° from
truth** and 0.827° from the AHRS. Fixed 2026-08-29 — both now come from the
sensor, `obs_layout` is unchanged so it is not a spec change, and the reward
keeps `s` as it always did.

**Nothing measured changes**: every `obs_pitch` policy in the standings trained
and was evaluated at `ahrs_level: none`, and the whole odo/ahrs line has
`obs_pitch: false`. What it changes is the queued run, which would otherwise
have trained against a perfect pitch signal the bike does not have — in the one
workstream whose entire point is removing those. Guarded by
`test_every_orientation_channel_in_the_obs_comes_from_the_sensor`, which
asserts each orientation entry is closer to the sensor than to truth, so it
cannot be defeated by the error model getting quieter.

**RAN 2026-08-29, AND IT DID NOT WORK.** `general_rl_odo_ahrs_pitch`,
`obs_pitch: true` alone against `rand2`, 18M steps: **survive_rate 0.60**
against 1.00, `track_geo` 0.432 against 0.663, and `by_family.turn_big`
`t_head_s` still **15.0** — it never turns around. Worse everywhere, not
merely unhelpful.

**Read it as undertraining, not a verdict.** Steps to difficulty 0.95:

| `rand2` (15 obs) | 7.23M of 12M | 4.8M at full difficulty |
|---|---|---|
| `..._pitch` (17 obs) | **~16M of 18M** | **~2M at full difficulty** |
| truth-trained runs | 1.6–2.6M | ~10M |

Two extra observation entries **more than doubled the ramp**, and `eval/score`
was still climbing at the final eval — the exported checkpoint is 18.0M, the
last one. The 18M budget was derived assuming a ~7.5M ramp; that assumption
was wrong.

**A sign bug was found and fixed, and it is NOT why the arm failed.**
`general_env` fed `obs` pitch from `rpy_from_quat`, which is MINUS
`extract_state`'s convention (balance.py:84), so pitch and pitch_rate were
both inverted. **Both together**, so the pair stayed coherent — the rate is
still the derivative of the angle — and a network absorbs a consistent sign
flip in its first layer. The arm was a fair test in sim. It was a DEPLOYMENT
bug: `hw/state.set_orientation` writes the AHRS quaternion into `qpos` and
lets `extract_state` negate it, so the bike would have produced the opposite
sign to what the policy trained on. Guarded now by
`test_every_orientation_channel_in_the_obs_comes_from_the_sensor`, which
asserts the SIGN as well as the source — the original version only checked
"closer to the sensor than to truth", which a negated channel passes trivially
by being far from both.

#### What the pitch channel is actually for (2026-08-29)

`analysis/pitch_ablation.py` takes the one policy that CAN do the reversal and
perturbs only its pitch pair, everything else at MuJoCo truth:

| pitch pair | −0.50 | −0.84 | −1.02 | −1.20 m/s | eval grid |
|---|---|---|---|---|---|
| normal | 7.8° | 9.1° | 14.5° | 23.0° | 0.866 / 1.00 |
| **blanked** | 31.4° | **86.4° FELL** | **85.4° FELL** | **84.3° FELL** | 0.816 / 0.95 |
| **noisy (1.42°)** | 6.9° | 7.7° | 16.4° | 22.1° | 0.676 / 0.95 |

**The `eval grid` column is optimistic, and the comparison is not.** Those
three scores were taken before 2026-09-11, when `pitch_ablation.py` could not
supply the AHRS at all (it has no flag, and `policy_env_overrides` did not
carry the field) — so an AHRS-trained policy was scored on truth attitude.
Every row shares that baseline, which is what the ablation needs; only the
absolute level is flattering. Not re-run: the finding is the 84–86° column.

**Blanking brings the backflip back** — same weights, same sensors, one
channel zeroed, and the same 84–86° as the policies that never had pitch.
The eval grid barely notices (0.866 → 0.816), which is the signature of a
NARROW purpose: blanking a general-regulation input would wreck ordinary
driving. **Noise does not** bring it back; it costs general performance and
not the flip protection.

**The operative band is 2–8°, not the 80–90° of the flip.** Asking the policy
for its action twice at every step, once with the real pitch pair and once
blanked, and binning by the pitch at that step:

| \|pitch\| | 0–1° | 1–2° | 3–5° | 5–8° | 8–20° |
|---|---|---|---|---|---|
| mean \|Δaction\| | 0.154 | 1.10 | 1.14 | 1.31 | 1.46 |

Saturated at the full action range by 5–8°, and the policy never lets pitch
exceed ~22°. **It is catching the front wheel on the way up.** 97% of steps
sit in the idle band, which is why removing pitch costs so little on the grid.

**That band is where the sensor is worst.** True pitch RMS is 0.481° against
a 1.418° TM151 error, so the idle band is 3× more noise than signal and the
onset band is an SNR of 1.4–5.6 — not the ~60 the 80–90° framing implies.
Detection survives it measurably, because it is a repeated-sample decision
over the ~0.1–0.5 s the onset takes and the response saturates by 5–8°, so the
policy only needs "above ~3°". **A low-pass filter on pitch would be actively
harmful** — it would clean the idle band at the cost of lagging the onset,
which is the one thing the channel is for.

**Queued: `config/rl_general_odo_ahrs_pitch_w.yaml`** — `obs_pitch: true` AND
`w_pitch: 2.0`, 20M steps, matching what both truth-trained flicking configs
carry. The penalty is now chosen on evidence rather than by matching: it
biases the policy OUT of the 2–8° band rather than requiring it to resolve
pitch inside it at SNR ~2, which is a different and more robust mechanism than
the first arm had.

```sh
./scripts/rl.sh up general --config config/rl_general_odo_ahrs_pitch_w.yaml \
    --run-dir runs/general_rl_odo_ahrs_pitch_w \
    --export-name general_rl_odo_ahrs_pitch_w
```

**Read `curriculum/difficulty` before the score.** If the ramp repeats at ~16M,
20M buys only 4M at full difficulty and the result is inconclusive whatever
`w_pitch` did. Bar is unchanged: `rand2` and the first arm both complete 0 of
the 3 moving reversals, so anything above 0 confirms it — read
`by_family.turn_big`, and read `survive_rate` WITH `t_head_s`.

**What is now given up:** the first arm was meant to be the single-variable
test of `obs_pitch` alone. It failed for budget reasons, so that test was
never really run, and this arm changes two things at once. If it works we will
not know whether observing alone would have sufficed.

#### The ideal-sensor reference — what the eval looks like with no sensors

`analysis/per_command.py --policies general_rl_smooth_diff_pi
general_rl_pitch_smooth_diff_pi --ahrs none --encoder ideal --tag ideal`. Both
are truth-trained, both carry the current `plant_digest`, and with `--ahrs
none` and no `obs_odometry` the controller reads MuJoCo truth for velocity AND
attitude. **A ceiling, not a hardware-achievable number.** By family, against
the best sensor-trained policy:

| family | metric | `smooth_diff_pi` | `pitch_smooth_diff_pi` | `rand2` (sensors) |
|---|---|---|---|---|
| cruise | `t_head` / `head_tail` | 0.47 s / 2.4° | 0.58 s / 3.6° | 0.80 s / 8.1° |
| crab | `vel_err_med` | 0.348 | 0.345 | 0.404 |
| turn_big | `t_head` / `head_tail` | 8.09 s / 80.5° | **1.60 s / 5.1°** | 15.00 s / 152.3° |
| hold | `drift_m` | **0.296 m** | 1.267 m | 0.664 m |

The sensor cost on `cruise` is real but modest — roughly 1.7× on time-to-
heading and 3× on held heading. **On `turn_big` it is not a sensor cost at
all**, per the section above.

Note the two truth policies are not ranked, they trade: `smooth_diff_pi` holds
a quarter of the other's drift while `pitch_smooth_diff_pi` is the one that
can reverse.

#### Teleop: an AHRS tau flag, and a reset bug (2026-08-28)

**`run_drive --teleop --ahrs-tau SECONDS`.** Teleop built its `SimAhrs` with
the module default, so a policy trained at 0.19 was flown at 2.0 with nothing
saying so. The flag threads a value through `_teleop` into the constructor and
the startup banner now prints the tau ACTUALLY IN FORCE, calling it a guess
only when it is the default — a banner that names the wrong constant is worse
than no banner. `--ahrs-tau 0` is rejected at the parser (`_gm_step` computes
`exp(-dt/tau)` in plain floats and raises `ZeroDivisionError`); the message
points at 1e-5 for the white-noise limit. `record.py` builds no `SimAhrs` and
needed nothing.

The flag exists so the banner can be honest, NOT because tau is a live risk —
see the section above. Expect no felt difference between 0.19 and 2.0.

**A viewer reset left the heading command standing — in ANALYTIC mode.**
Backspace rewinds the pose, but `state["psi"]` is operator intent and survives
it, so the leftover `state["psi"] - state["psi_sent"]` delta reached
`command_heading` on the next frame and put the analytic controller straight
into an ARC — a bike that should drive forward from the start pose curved away
instead. `ensure_mode` now re-anchors the command on the same rewind test the
trail uses, deferred ONE FRAME because `c._psi` is only re-read from the
rewound data when `_Base.step` notices the rewind (`balance.py:131`); zeroing
on the frame the rewind is seen would anchor to the pre-reset heading, which
is the bug rather than the fix. Covered by
`test_viewer_reset_re_anchors_the_heading_command`, which fails without it.

**And the one that was actually reported, in GENERAL mode: `c._psi` was
stale.** Four live captures with `AOW_RESET_DEBUG=1`, every one the same
shape — bike back at `+0.0°`, the controller still holding the heading it had
when Backspace was pressed, and `c.mode` still `general`:

| capture | bike_yaw | `c._psi` | `_gen_psi_cmd` | `state["psi"]` |
|---|---|---|---|---|
| 1 | +0.0° | **+50.9°** | +171.3° | +171.3° |
| 2 | +0.0° | **+55.2° → +55.7°** | +50.9° | +50.9° |
| 3 | +0.0° | **−112.2°** | +190.0° | +190.0° |

`c._psi` is what everything else is anchored to, and it is only re-read from
the rewound data by `_Base.step`'s own check (`balance.py:131`). On the real
viewer that had not run by the time `ensure_mode` did — `mode` is still
`general` in all three, so the re-engage path never fired either. Capture 2 is
the mechanism in one line: `c._psi` moves +55.2 → +55.7 across two frames,
tracking the rewound bike incrementally while carrying a constant offset. It
is an unwrapped accumulator absorbing the rewind as a permanent bias rather
than resetting through it. So the command came back pointing wherever the bike
happened to be facing — which is why arrows recover roughly forward and the
`6`/`7`/`8` snaps flip it to reverse.

`ensure_mode` now calls `c.reset(m, d)` itself on the rewind it detects,
before anchoring. Covered by
`test_viewer_reset_re_reads_the_heading_from_the_rewound_data`, which asserts
after a SINGLE frame — the harness resets synchronously, so `_Base.step` gets
its chance on the next frame and the command comes right by itself. **That is
how two earlier attempts at this test passed against broken code**, and why
the first fix (deferring the re-anchor by one frame to let `_Base.step` do the
re-read) was exactly backwards: correct in the harness, wrong in the viewer.

**Confirmed fixed on a live session** — four resets, including two with speed
still on, every one reading `c._psi +0.0  gen_cmd +0.0  state_psi +0.0
v +0.00` with the bike at `+0.0`. The `AOW_RESET_DEBUG` instrumentation that
found it has been removed; it was three lines gated on an env var, and the
captures it produced are the tables above.

#### The sensor modes are now a test (2026-08-28)

`tests/test_sensor_modes.py`, marker `contact`, 5 tests in ~19 s serial. Until
now nothing in the suite ran a policy against the sensors the bike will
actually have, so the 0.808 → 0.044 collapse was invisible to a green run — and
a repoint of `control.general_move` back to a truth-trained export would have
been a completely green change.

Four commands (hold, forward, reverse, 90° turn), not the full twenty: this is
a regression guard, not a measurement. The measurement stays `chatter.py` and
the tables above. Measured on the subset, with the asserted floor beside it:

| mode / policy | surv | track_geo | asserted |
|---|---|---|---|
| IDEAL `smooth_diff_pi` on truth | 1.00 | 0.951 | ≥ 0.90 |
| SENSOR `odo_ahrs` tm151 tau 2.0 | 1.00 | 0.859 | ≥ 0.78 |
| SENSOR `odo_ahrs` tm151 tau 0.19 | 1.00 | 0.849 | ≥ 0.78 |
| **GUARD** `smooth_diff_pi` in sensor mode | **0.25** | 0.493 | surv **≤ 0.50** |

The guard row is asserted as a CEILING, not a floor: the claim is that sensor
mode is genuinely hard, so a version of these models a truth-trained policy
sails through is a version that is not modelling the sensors. A fifth test
asserts only that `control.general_move` names an export declaring
`obs_odometry` — a property of the export, not a name, so a better
sensor-trained policy can replace it freely.

**The assertion is SURVIVAL, not RMS against truth.** That was measured the
wrong way round once already: an open-loop accuracy objective selected a WORSE
estimator, because an estimator can be more accurate on average and worse
exactly where the controller needs it.

**What the subset does NOT resolve**: both taus survive 1.00 here, so this
cannot see the specialisation that costs the policy 1.00 → 0.95 on the full
grid. That is the honest limit of a 4-command guard, and it is what
`general_rl_odo_ahrs_rand` addresses rather than this file.

**Still deferred**, per the 08-27 decision: nothing here disturbs the bike or
looks at the contact, so it inherits the eval grid's blind spots wholesale.

#### The move yaml never recorded the AHRS, so most scripts evaluated without it (2026-09-11)

**`policy_env_overrides` carried `obs_odometry` and `odometry_encoder` and not
`ahrs_level`.** Its own docstring already named this failure mode — "an `odo`
policy evaluated without it is handed MuJoCo truth instead of the onboard
estimate, i.e. a cleaner signal than it ever trained on, and scores better than
it deserves" — and the AHRS was the one sensor not on the list. The attitude
error corrupts roll, roll_rate and yaw_rate IN PLACE, so the observation width
never changes and nothing raises:

    general_rl_cmd_curriculum2b      ahrs=None
    general_rl_odo_ahrs              ahrs=None

**How much it was worth**, 20-command grid, identical seeds, randomization off,
encoder `counts`, only the error model differing:

| policy | no AHRS | tm151 τ 0.19 | tm151 τ 2.0 |
|---|---|---|---|
| `general_rl_odo_ahrs` | 0.663 / 1.00 | 0.644 / 0.95 | 0.686 / 1.00 |
| `general_rl_cmd_curriculum2b` | **0.675** / 0.95 | 0.595 / 0.90 | 0.530 / 0.90 |

`odo_ahrs` barely cares — +0.019 against a ±0.02 seed-noise floor, and at its
own training tau of 2.0 the no-AHRS run is WORSE. The curriculum arm is the
casualty at +0.080 and +0.05 of survival, and it is the generation whose
comparisons are live. **The policy named for the AHRS was the least
contaminated one in the set**, which is why nobody caught this by reading a
table.

**Not every script was affected.** `chatter.py`, `per_command.py`,
`ahrs_tau.py`, `eval_video.py` and `reverse_flip.py` all patched
`cfg["env"]["ahrs_level"]` themselves off a `--ahrs` flag, and `per_command`
defaults it to `tm151`. Affected were the ones with no flag at all:
`floor_sweep.py`, `pen.py`, `liftoff.py`, `hold_spectrum.py`,
`move_confusion.py`, `mass_envelope.py`, `kick_recovery.py`,
`contact_surrogates.py`, `wheel_slowmo.py` and `pitch_ablation.py` — the last
of which discusses the TM151 pitch error in its comments while running without
one.

**The fix, and the precedence rule it forced.** The exporter now writes
`ahrs_level` / `ahrs_tau_s` / `ahrs_channels`; `load_move` carries them onto
the policy; `policy_env_overrides` returns them. Because those overrides are
overlaid ON TOP OF `cfg["env"]`, a cfg-level AHRS is now overridden back by any
policy that declares one — so **a caller forcing a sensor mode sets it on the
POLICY**, which is the route `--encoder` has always taken. All six cfg-patching
call sites were converted. Getting this backwards would break `ahrs_tau.py`
worst and least visibly: it sweeps tau, and a policy-wins-always rule applied
to a cfg-level sweep pins every cell to the training tau and prints a flat
table as a finding. `test_the_policys_ahrs_wins_over_the_config` pins it.

**21 already-exported moves were backfilled** — the nine named `odo_ahrs` /
`cmd_curriculum` arms and all twelve `personality` seeds. Each move was matched
to its training config by fingerprinting the fourteen env fields the yaml
already records (`v_max`, `v_lat_frac`, `vel_window_s`, `obs_*`, `act_*`,
`odometry_encoder`, `action_space`, …); a mismatch was a hard error rather
than a warning, since writing the wrong sensor model into the contract is the
exact mistake being removed. `ahrs_tau_s` is written EXPLICITLY even where the
config left it defaulted, so an export pins the number instead of inheriting
whatever `sim_ahrs.TAU_ORIENT_S` is later — it has already moved once, 2.0 →
0.19.

**Two fixes fell out alongside.** `GeneralEnv` passed its raw `params`
argument, not the resolved `self.p`, to both `SimAhrs` and `SimOdometry` — so
an env built from an rl_cfg alone (as every test in `test_general_rl.py` builds
one) crashed on that path. And teleop replay warned about an odometry-trained
policy running on truth but had no equivalent AHRS warning; it does now, because
until this change there was no field to test.

**Still not covered**: `run_drive.py` teleop takes `--ahrs` explicitly and does
not read the policy's declaration — it only warns. That is a separate
construction path from `GeneralEnv` and was left alone.

#### Live training run

Last read 2026-08-14 at 6.01 M steps (`rl_general.yaml` with `v_lat_frac` 0.12
and contact damping 0.5): curriculum topped out at 4.997 M (83% through, vs
3.59 M for the previous run — the changes made it markedly harder to
bootstrap), reward plateaued at +3.9% over the final tenth. Against the
previous run it is **behind on every comparable axis** — score 0.685 vs 0.767,
survive 0.950 vs 1.000, `turn_asym` 0.319 vs 0.265 — with one genuine win,
`steer_rest_deg` 10.2 vs 19.2, which improved *within* the run (83 → 10) so it
is a real trend. Caveat: `v_lat_frac` changed the command distribution and the
plant changed too, so this is not a clean A/B. The training box was unreachable
at the time of writing, so this is the last reading, not the current state.

`turn_asym` has now sat at 0.2–0.32 across every run and has never improved
with more steps. It is not a training-length problem.

#### Wings in the policy — tried four ways, answer is no (2026-08-15)

Four 5–6 M runs asked whether the general policy should observe and drive the
righting wings. It should not, and the reason is structural rather than a
tuning failure.

| run | wings available | `w_wing` | outcome |
|---|---|---|---|
| wings1 | from step 0 | flat 0.05 | total crutch: 89°, duty 1.0, **20/20 falls** forced stowed |
| wings2 | from step 0 | ramp → 1.0 | worse: deployed less but let roll reach 43° and got caught harder (24% of weight on the feet vs 9%) |
| wings3 | gated open 0.5–0.8 | ramped on `_diff` | never used — the two schedules collided, no free window |
| wings4 | gated open 0.4–0.6 | ramped 0.85–1.0 | **never used**, with a verified 0.57 M-step window where they were open and nearly free |

Available early and it never learns to balance; available late and it has no
use for them. **The cause is that episodes terminate at `fall_roll_deg` 60, so
the fallen state the wings exist for is outside the training distribution by
construction** — see "Still open" in `docs/plans/self-righting.md` for what
changing that would require. Do not re-open this by tuning the reward.

Two results worth keeping out of the detour:

* **`general_wings3_rl` does a genuine flick** — 211° of steer sweep in the
  BODY frame against only 94° in the WORLD frame, i.e. it rotates the bike
  around a wheel whose ground heading barely moves. Every other policy is the
  inverse (65–80° body, 206–221° world): it cranks the wheel and drives round.
  First policy in the repo to do the manoeuvre the flick move was authored for.
* **The crab ceiling is roll headroom, confirmed.** `crab_ratio` hit 0.65/0.70
  with the wings pinning roll to ~5°, and fell straight back to ~0.27 as they
  withdrew. Crab is bounded by the differential being spent on balance, not by
  the policy failing to learn it.

**Default is unchanged: the wings are operated separately from the policy.**
The scaffolding stays and is inert when off — `obs_wings`/`act_wings` default
false, `ActionBounds.wing_rate_max` defaults to 0.0 so every existing 3-arg
move yaml still constructs, and `rl_general.yaml` still yields obs 15 / act 3 /
`nu` 3 with no wings in the model. flick/pivot/ball are untouched.

---
