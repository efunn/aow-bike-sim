# A privileged critic for the general policy

Candidate change to `train_general_rl.py` and `control/general_env.py`: give
PPO's **value function** the simulator's true state, while the **policy** keeps
seeing exactly what the Pi will hand it. Collected 2026-08-29 after asking
whether stable-baselines3 PPO is still the right tool given the compute on
hand. The answer was yes — this is the one gap in SB3 that lines up with the
sensor workstream, and it is worth writing down before it is worth doing.

**Nothing here is committed to.** No config carries the flag, no run has been
launched, and §7 argues it *cannot be evaluated at all* until the seed floor is
measured — which is a prerequisite worth doing whether or not this lands.

Evidence tags as in `general-rl-improvements.md`:

- **[measured]** — a number or a code read backs it
- **[reasoned]** — follows from measured facts, not directly observed
- **[speculative]** — a hunch worth testing before acting on

---

## 1. Two things called "asymmetric"

They get conflated, and one of them we already have. Conflating them is how
this looked like a solved problem for the length of a conversation.

| | the REWARD | the CRITIC INPUT |
|---|---|---|
| where | `general_env.step` | `model.policy.mlp_extractor.value_net` |
| privileged today? | **yes** | **no** |
| what it decides | what counts as good | how much of the return is credited to the action |
| documented at | `_obs`, "THE OBSERVATION gets the estimate; `s` stays TRUTH" | nowhere — it is SB3's default |

**We have the first.** `general_env._obs` computes `s = extract_state(...)` from
MuJoCo and hands the reward `s.roll`, `s.pitch`, `s.v_lon`, `s.v_lat` while the
observation gets `o_roll` off `SimAhrs` and `o_lon`/`o_lat` off `SimOdometry`.
That separation is deliberate and load-bearing: feed the estimate to both and
the policy can be rewarded for fooling its own estimator. Keep it. **[measured]**

**We do not have the second.** `PPO("MlpPolicy", venv, ...)` builds one input
tensor and feeds it to both heads, so the value function sees the same 15-wide
corrupted vector the actor does.

## 2. Why the first makes the second necessary

Because the reward is scored on truth, the critic is asked to predict the
discounted sum of a quantity its input does not contain. That residual is
irreducible — no amount of training removes it — and it enters the advantage

    A_t = r_t + γ·V(o_{t+1}) − V(o_t)

as noise. The gradient is not biased by it; it is *noisier*, and the noise is
**correlated within an episode**, because these are drawn once at `reset` and
are invisible to the actor for the whole 750 steps: **[measured]**

| draw | where | span in `rl_general_odo_ahrs_rand2` |
|---|---|---|
| per-body mass scale | `randomize.apply` | ±15% |
| sliding friction scale | `randomize.apply` | ±20% |
| actuator strength (supply voltage) | `randomize.apply` | ±15% |
| AHRS orientation RMS | `general_env.reset` | 0.3–2.0 deg, log-uniform |
| AHRS correlation time | `general_env.reset` | 0.1–0.6 s, log-uniform |
| contact solref / dampratio | `randomize.apply` | **not yet enabled** — queued |

An episode that scored badly because it drew 2.0 deg of attitude error and a
0.85× actuator looks, to the learner, exactly like an episode where the policy
chose badly. A critic that can see the draw explains that offset away and the
advantage is left carrying only what the actions did. **[reasoned]**

This is also the regime where the literature reports the effect is largest —
wide domain randomization over quantities the actor cannot observe — which is
precisely the direction the contact workstream is heading. `solref_frac` and
`dampratio_range` are commented out in every config today with the note
"UNCOMMENT FOR THE NEXT RUN", and contact is the parameter the eval is most
brittle to. Turning those on adds two more episode-constant invisible draws,
i.e. makes this problem worse, not better.

## 3. The honest counter-case

Written first, because it is the part that would get skipped.

- **The actor's observation is not that impoverished.** 15 entries including
  `prev_action`, and the AHRS/odometry corruption is a filtered, correlated
  error rather than sparse or occluded sensing. The gap between `V(o)` and
  `V(s,o)` may be small. **[speculative]**
- **The in-distribution score is already saturated.** `eval-score-rewrite.md`
  establishes that a converged policy evaluated in the mode it trained for
  survives 1.00 whatever that mode is, and `rand2` does. So the headline score
  has limited room, and any gain will show up in `track`, `vel_err`,
  `turn_asym` and time-to-converge — not in `survive_rate`. **[measured]**
- **It cannot be read off one run per arm.** See §7. This is the binding
  constraint, and it is not specific to this change.
- **It is a training-time-only change with a real code cost.** ~120 lines of
  custom SB3 policy (§5), in the one file where a mistake is expensive because
  a run is 1.4 h.

If §7's seed floor comes back wide — say the spread across seeds is itself
0.05 — then this change is not measurable at 3 seeds either, and the honest
conclusion is that the whole comparison table needs more runs before *any*
algorithmic change can be judged.

## 4. Design

### 4.1 Where the flag lives, and why

**`algo.privileged_critic: bool`, defaulting to `false`.** Under `algo:`, not
`env:`, and the rule the repo already uses decides it: `env:` holds the fields
that get exported into the move yaml and that define the observation contract
(`vel_window_s`'s docstring spells this out). This changes neither. The actor's
input is untouched, `obs_layout` is unchanged, and the critic is discarded at
export — so nothing downstream of `moves/*.npz` can tell a privileged run from
an ordinary one.

But the *env* still has to know, because it emits the wider vector. Pass it as
a **constructor kwarg** — `GeneralEnv(params, cfg, privileged=False)` — rather
than having the env read `cfg["algo"]`:

- every existing call site is untouched (`analysis/chatter.py`,
  `analysis/ahrs_tau.py`, `_eval`, `drive.py`'s replay path);
- `algo:` stays a section the env never reads, which is true today;
- the eval and replay paths keep the flat 15-wide env by construction, which
  is what they want.

### 4.2 The observation becomes a PREFIX, not a dict

The privileged block is **appended**, and the actor slices `[:obs_dim]`. The
observation space stays a flat `Box`. This is the same trick `general_spec`
already documents for the optional blocks — "the 15-entry layout is a strict
PREFIX of the 17-entry one, so every positional index into an observation
stays valid across both" — applied one level out.

The alternative, a `Dict` observation space with `MultiInputPolicy`, is
**rejected**: it turns `VecNormalize.obs_rms` into a dict of `RunningMeanStd`,
which breaks `_export`, `_verify_export`, `_resume_vecnormalize` and the
`vecnormalize.pkl` format, for no capability the prefix form lacks.

The prefix form has one property that matters a lot and is easy to miss:
**`VecNormalize` normalizes per element**, so the running statistics of the
first 15 entries are unaffected by whatever is appended after them. The
exported `obs_mean`/`obs_var` are therefore just `obs_rms.mean[:obs_dim]` and
`obs_rms.var[:obs_dim]` — numerically the same object a non-privileged run
would have produced. Export is a slice. **[reasoned — worth asserting in a test]**

### 4.3 What goes in the privileged block

Appended in this order. Widths for the `rand2` configuration (actor obs 15):

| block | entries | w | why the actor cannot see it |
|---|---|---|---|
| true fast state | `roll`, `roll_rate`, `yaw_rate`, `pitch`, `pitch_rate` | 5 | actor reads these through `SimAhrs` |
| true body velocity | `v_lon`, `v_lat` | 2 | actor reads these through `SimOdometry` |
| true heading error | `sin_psi_err`, `cos_psi_err` | 2 | actor's `_psi` is AHRS-integrated and drifts unbounded |
| episode draws | `mass_scale`, `friction_scale`, `actuator_scale`, `ahrs_rms_deg`, `ahrs_tau_s` | 5 | constant within an episode, never observed |
| curriculum | `difficulty` | 1 | sets the command distribution, hence the achievable return |
| **total** | | **15** | → critic input 30 |

Second tier, deliberately **not** in the first version: per-wheel contact
normal force and an airborne flag. They are the terms most likely to help a
contact-randomized run, and they are also the ones whose extraction from
`data.contact` is fiddly. The block is a suffix, so adding them later is
additive and does not disturb anything already trained. **[speculative]**

`randomize.DomainRandomizer.apply` does not currently *record* its draws — it
applies them to the model and returns. It needs a `self.last: dict` written in
`apply` and `reset_nominal`, which is the only change outside the two files
named above.

With `randomization.enabled: false` the draw entries are constants with zero
variance. `VecNormalize`'s epsilon handles the division; and no eval path uses
the critic at all, so this is cosmetic.

### 4.4 The critic sees truth AND the actor's observation

`V(s, o)`, not `V(s)`. Two reasons, one practical and one about what the value
function is for:

1. **It strictly nests today's critic.** With the suffix ignored the network
   can represent exactly the current `V(o)`, so the flag cannot make the
   critic *less* capable — only the optimization can go wrong.
2. **The agent's confusion is part of the value.** What happens next depends on
   what the *actor* will do, and the actor acts on `o`. A critic given only
   `s` cannot represent "the true roll is fine but the AHRS is currently
   reading 2 degrees off, so the policy is about to do something wrong."

The unbiased asymmetric actor-critic in the literature (Baisero & Amato) makes
the same argument for conditioning on state *and* history rather than state
alone; the form here is the memoryless version of it. Worth reading the paper
before finalising rather than taking this sentence as the citation.
**[reasoned]**

## 5. Implementation

Five edits. Nothing in `control/general_spec.py`, nothing in `hw/`, nothing in
`deploy/`.

**`control/randomize.py`** — record the draws. `apply` writes
`self.last = {"mass_scale": ..., "friction_scale": ..., "actuator_scale": ...,
"solref_scale": ..., "dampratio": ...}`; `reset_nominal` writes the all-ones
version. ~10 lines.

**`control/general_env.py`** — `__init__` takes `privileged=False`, and when
set widens `observation_space` to `obs_dim + PRIV_DIM`. `_obs` already computes
everything needed: `s` is in hand, `self._ahrs`'s parameters are readable, and
the draws come off `DomainRandomizer.last`. One `np.concatenate`. ~30 lines
including the block's own name tuple, which should live beside
`OBS_NAMES_BASE`'s idiom so it is greppable.

**A custom SB3 policy** — the fiddly part, ~80 lines. Subclass
`ActorCriticPolicy` with `share_features_extractor=False`, override
`extract_features` to return `(obs[:, :obs_dim], obs)`, and override
`_build_mlp_extractor` to build a `MlpExtractor`-shaped module whose
`policy_net` takes `obs_dim` and whose `value_net` takes the full width.
**Keep the attribute names `mlp_extractor.policy_net` and `action_net`** — that
is the contract `_export` reads, and holding it means the export path needs no
change beyond the slice below.

**`train_general_rl.py`** — read the flag; pass `privileged=` into `_make_env`;
select the custom policy class instead of `"MlpPolicy"`; slice `obs_rms` in
`_export`; and two small consequences:

- `_verify_export` builds a random observation of `pol.obs_mean.shape[0]` and
  feeds it to `model.policy.predict`. Under the flag those widths differ. Draw
  the full width, normalize, predict, and hand the *prefix* to `pol.action`.
- `BestByScore._on_step` builds a plain `GeneralEnv` and calls
  `self.model.predict` — 15 in, 30 expected. Build that env with
  `privileged=True` too, purely for width. This is provably eval-neutral: the
  actor slices the prefix, so nothing in the suffix can reach the action. The
  suffix is nominal there anyway, since the eval env sets
  `randomization.enabled: false`.

**`--resume` across the flag is impossible** and should say so. The observation
width changes, so the checkpoint's first critic layer and the
`vecnormalize.pkl` shapes both mismatch. Raise a named error rather than
letting torch report a shape.

### Reproducibility criterion

**The flag must not touch the rng stream.** Every value in the privileged block
is already drawn — by `reset`, by `DomainRandomizer.apply`, by `mj_step` — so
building the suffix consumes no randomness. That is the same criterion the AHRS
reset comment states ("Inside the `if`, so the default path's rng stream is
untouched"), and it is what makes `privileged_critic: false` bit-identical to
today rather than merely equivalent. It should be a `pure` test: same seed,
same config, flag off, obs identical to the pre-change env.

## 6. What it costs

### Measured baselines

| run | box | steps | wall | fps |
|---|---|---|---|---|
| `general_rl_odo_ahrs_rand2` | remote (2950X, 32 env) | 12.0 M | **1.43 h** | 3299 → 2334 |
| `general_rl_glide_pitch_dt4e4` | M4 (32 env) | 6.0 M | **0:34** | ~2969 |

fps decays over a run as the curriculum lengthens surviving episodes; the mean
on the remote is ~2340, i.e. one 16384-step rollout every **7.0 s**. **[measured]**

### Estimated overhead

| term | now | with the flag | delta |
|---|---|---|---|
| actor input width | 15 | 15 | **0** |
| critic input width | 15 | 30 | +15 |
| critic parameters | 18,689 | 20,609 | **+10.3%** |
| total parameters | ~37,600 | ~39,500 | +5.1% |
| obs bytes / env-step over the `SubprocVecEnv` pipe | 60 B | 120 B | +0.14 MB/s at 2340 fps |
| rollout buffer | 1.0 MB | 2.0 MB | +1 MB |
| checkpoint zip | — | — | +8 KB |
| env work per policy step | 50 × `mj_step` | 50 × `mj_step` + one 30-float concat | ~1e-5 |

Only `net_arch`'s first critic layer grows; `[128, 128]` in SB3 2.x means
separate `pi` and `vf` trunks with no shared layers, so the actor is untouched
bit for bit. The extra flops land entirely in the update phase — 160 minibatch
updates of 1024 per rollout — which the arithmetic puts at well under 1 s of
the 7.0 s rollout on 32 threads.

**Predicted wall-clock cost: under 1%, i.e. under a minute on a 1.4 h run.**
**[reasoned]** — and it should not be believed on the arithmetic. Measure it,
with `rl_general_pc_smoke.yaml` a copy of `rl_general_odo_ahrs_rand2.yaml`
carrying the flag:

```bash
python -m aow_sim.train_general_rl --config config/rl_general_pc_smoke.yaml --run-dir runs/pc_smoke --export-name pc_smoke --timesteps 100000
```

then read `time/fps` off the board against the same 100k with the flag off.
100k steps is ~45 s each. (`smoke` in both names, per the deletion rule — these
are the throwaways.)

### What it does not cost

| artifact | affected? |
|---|---|
| `obs_layout` / `general_spec.py` contract | **no** |
| `moves/*.npz` structure, `moves/*.yaml` fields | **no** |
| `control/policy.py` numpy replay | **no** |
| `control/drive.py`, `hw/`, `deploy/bundle.npz` | **no** |
| `plant_digest`, `design_digest` | **no** — nothing under `control:` or the plant moves |
| `--resume` from an existing checkpoint | **yes, blocked** — see §5 |
| tests | `pure` (new reproducibility test), `policy` (export round-trip). Full suite before handing back — it crosses training and export. |

## 7. How to evaluate it — and the prerequisite

**This cannot be judged from one run per arm, and neither can anything else in
`docs/status.md`'s standings table.** Every `config/rl_general_*.yaml` carries
`seed: 0`. The eval grid is deterministic — `_eval_episodes` fixes seeds per
command, `chatter.py` runs with randomization off, 20 commands, one episode
each — so there is no eval sampling noise. What has never been measured is the
**training** seed spread: what the *same config* trained twice scores on that
same fixed grid. `analysis/ahrs_tau.py`'s header already names "the seed-noise
floor" as the number it would want. **[measured]**

Differences of 0.05–0.10 in `score` are read as real throughout the standings
table. Whether they are cannot currently be answered.

**Step 0 — the seed floor. Do this whether or not the rest happens.**
`rl_general_odo_ahrs_rand2.yaml` unchanged, `seed: 0, 1, 2`. 3 × 1.43 h ≈
**4.3 h**. Report the spread in `score`, `track`, `vel_err`, `turn_asym`.

**Step 1 — the arm.** Same config plus `privileged_critic: true`, same three
seeds. Another **4.3 h**. One overnight for both steps.

**Decision rule, written before the runs:** the mean on/off gap must exceed the
step-0 spread. If it does not, the change is not measurable at this sample size
and the answer is "no effect detected at n=3", not "no effect".

Run `rand2` rather than the current best-by-pointer `odo_ahrs`, because `rand2`
randomizes the AHRS error parameters — it has the most invisible-to-the-actor
episode structure, which is exactly the mechanism under test. If the effect
exists anywhere it should be largest there.

Launch, per the hard rule, as `scripts/rl.sh` and not a bare module:

```bash
ssh 192.168.1.101 'cd ~/aow-bike-sim && ./scripts/rl.sh up general --config config/rl_general_odo_ahrs_rand2_pc.yaml --run-dir runs/general_rl_pc_s0 --export-name general_rl_pc_s0'
```

## 8. Rejected and deferred

**`V(s)` alone, dropping the actor's observation from the critic** — rejected.
Strictly less information than `V(s, o)`, cannot represent the actor's own
confusion, and does not nest the current critic, so a regression would be
ambiguous between "privileged information does not help" and "we removed
something that did". §4.4.

**`Dict` observation space with `MultiInputPolicy`** — rejected on plumbing
cost for no capability. §4.2.

**Moving the REWARD onto the sensed view instead** — rejected, and it is the
obvious cheap alternative to §5's ~120 lines, so it is written down before
someone re-proposes it. It attacks the same problem from §2 — the critic is
asked to predict a return its input does not contain — by closing the gap from
the other end: score the reward on `o_roll`/`o_lon`/`o_lat` rather than on `s`,
and the residual disappears with no code beyond a few lines in
`general_env.step`.

`pollen-robotics/microduck_rl` states it as a hard invariant (`AGENTS.md`): *"If
an obs is remapped to a sensor view (backlash encoder, bias), any tracking
REWARD on the same quantity must measure the same view — otherwise the policy is
punished for correcting what it sees."* They apply it to encoder bias and to a
backlash hinge the real encoder reads through, and feed the *unbiased* joint
position to the critic only.

**It is right for their case and wrong for ours, and the discriminator is
whether the obs↔truth gap is correctable by the policy.** Their two examples are
static unobservable offsets: a per-joint encoder bias and gear play are constants
no policy can perceive or undo, so reward-on-truth is pure advantage noise with
no upside — nothing is being protected. Ours is a *dynamic estimator* with its
own lag and correlated error, and there the gap is partly a function of what the
policy does: an AHRS-scored reward can be satisfied by exciting the estimator
rather than by balancing the bike. That is the failure `_obs` was written to
prevent ("THE OBSERVATION gets the estimate; `s` stays TRUTH"), and it is
load-bearing. §1.

So both changes remove the same noise term, and only one of them keeps the
anti-gaming property. That is the whole argument for paying §5's code cost
rather than taking the free version.

The rule does yield one CHECK worth running, though, in the narrow form where
their case and ours coincide: is there any quantity our reward scores that
differs from the actor's observation by a per-episode CONSTANT the policy can
neither perceive nor correct? §2's table is exactly that list — mass, friction
and actuator-strength scales, AHRS orientation RMS and correlation time — and
those are the terms this plan hands to the critic. An unobservable *constant*
has no dynamic component for the policy to game, so for those specifically the
two approaches are equivalent in safety and the critic route wins only on not
touching the reward. Where the two diverge is the AHRS's dynamic error, which is
the part we must keep scoring on truth.

**An observation HISTORY for the actor** (stack the last 5–10 frames) —
deferred, and it is a genuinely different change: it moves `obs_layout`, the
move yaml, `control/policy.py`'s replay, `drive.py` and the hardware path,
because the actor's input changes. It also plausibly subsumes part of the gain
here, since lag compensation is what a history buys and AHRS lag is a large
share of the corruption. Cheaper to *try* and far more expensive to *ship*.
The two are not alternatives — one is a training-loop change and the other is
a contract change — but if only one gets tested first, the history is the one
with the larger plausible effect and the privileged critic is the one that
cannot break deployment.

**`RecurrentPPO` (sb3-contrib)** — deferred. Same contract problem as the
history, plus the numpy replay in `control/policy.py` is a feedforward MLP
evaluator and would have to carry and reset LSTM state on the Pi.

**Teacher–student / RMA distillation** — the larger version of this idea: train
a privileged *policy*, then distill into a sensor-only student. The privileged
critic is a prerequisite step toward it, not a competitor. Out of scope until
step 1 says the privileged information is worth anything at all here.

**Swapping the algorithm** (SAC/TD3, MJX/Brax on GPU) — out of scope. Briefly:
off-policy buys sample efficiency the cheap CPU sim does not need and pairs
badly with wide randomization; MJX is the real throughput lever but needs a GPU
the remote does not have (a driverless GTX 1050 Ti) and would put the 64-segment
procedural omni contact meshes through the part of MJX that handles them worst.

## 9. Open questions

- Does the gap between `V(o)` and `V(s, o)` matter at all when the actor's
  observation is already 15 entries of correlated, filtered estimate rather
  than sparse or occluded sensing? This is the whole bet. **[speculative]**
- Should `difficulty` be in the block? It lets the value function track the
  curriculum instead of chasing a moving target — but it also means the critic
  learns a function of training bookkeeping, and the curriculum's own advance
  criterion reads `0.5 * (r_vel + r_head)`, not the value. Include it, flag it,
  and it is one column to ablate.
- Does the benefit grow when `solref_frac` / `dampratio_range` are finally
  enabled? The mechanism says yes — two more episode-constant invisible draws —
  and that is the run worth re-testing on. **[reasoned]**
- Does `norm_reward=True` interact? The reward normalizer's running variance
  is inflated by exactly the return variance the privileged critic is meant to
  explain away, so the effective learning rate may shift as a side effect.
  Worth watching `train/value_loss` and `train/explained_variance` on the board
  rather than reasoning about it — `explained_variance` is in fact the single
  most direct readout of whether this is working at all.
