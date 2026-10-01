# Smoothness as a loss on the policy (CAPS family)

> **Status: IMPLEMENTED 2026-10-01, NOT YET RUN.** `src/aow_sim/smooth_ppo.py`
> (`SmoothPPO`, terms `bound` / `temporal` / `second`; the spatial term of §4
> is NOT built). Four runs queued in `config/queue_smooth_loss.txt`: a base arm
> and the three arms of §5, one seed each. `reward.w_copper` (copper loss at
> the physics rate, `general_env.py`) exists too and is parked at 0.

Collected 2026-10-01 while asking why every `general_rl*` policy chatters in
steer and rear drive. The question was which levers exist beyond the reward
weights already tried (`w_smooth`, `w_hub_idle`), and what the literature has
built on or criticised since CAPS.

Evidence tags as in `general-rl-improvements.md`:

- **[measured]** — a number or a code read backs it
- **[reasoned]** — follows from measured facts, not directly observed
- **[read]** — from a paper's abstract or a search summary; the papers were
  NOT read in full

---

## 1. What the chatter is, measured 2026-10-01

Eval grid as `analysis/chatter.py` (20 commands, identical seeds,
randomization off), each policy on the sensors it declares. §1.1 is a
`chatter.py` table. §1.2 and §1.3 came from scratch probes that are **not
tracked**.

### 1.1 The network puts its mean past the action bound **[measured]**

SB3 PPO's default Gaussian is unsquashed and clipped to [-1, 1]
(`policy_kwargs` in `train_general_rl.py` passes only `net_arch` and the
activation). The mean |μ| before the clip, in units of each channel's bound,
over the whole grid — `chatter.py`'s "network mean BEFORE the clip" table,
read through `MLPPolicy.mean`:

| policy | steer >1 / >2 | hub >1 / >2 | diff >1 / >2 |
|---|---|---|---|
| `cmd_curriculum2b` | 44% / 14% | 17% / 0% | 15% / 2% |
| `odo_ahrs_rand2` | 29% / 5% | 7% / 0% | 8% / 0% |
| `glide_pitch_hub3` | 52% / 25% | 9% / 0% | 67% / 36% |
| `smooth_diff_og` | 22% / 3% | 7% / 0% | 39% / 13% |

Partly bang-bang, not purely: the median |μ| is under 1 on most channels.
`smooth_diff_og` prices diff at `w_smooth` 0.25 and still holds its diff mean
past the bound 39% of the time.

### 1.2 The pointer's hub reacts against its own last command **[measured]**

The observation includes the policy's previous normalized action (obs 12-14:
`prev_steer_rate`, `prev_hub`, `prev_diff`), so each new command can depend on
the last one directly, not only through the bike. The probe: at each recorded
step, keep the rest of the observation fixed, nudge `prev_hub` by a small
amount, and see how far the new hub mean moves. That ratio is the hub's gain
on its own last command. Clipped channels count as zero, since a nudge cannot
move a clipped output. Hold steps:

| policy | hub gain on its own last hub, median [p10] | largest gain of the 3×3 loop, median |
|---|---|---|
| `cmd_curriculum2b` | −0.91 [−1.50] | 1.08 |
| `odo_ahrs_rand2` | −0.83 [−1.34] | 0.97 |
| `glide_pitch_hub3` | 0.06 | 0.34 |
| `smooth_diff_og` | −0.35 | 0.39 |

Read the pointer's −0.91 as: if the last hub command was +0.5, this one is
pushed about −0.45 by that alone. Repeat it and the hub flips sign every step,
a 25 Hz square wave, shrinking only 9% a step. At the 10% of steps where the
gain is past −1 it GROWS instead, until the ±1 clip stops it. The last column
is the same question asked of all three channels together, cross-terms
included: below 1 the flip-flop dies away on its own, at 1 it never does. The
pointer sits at 1.08; the other policies at 0.3-0.4, where it is gone within a
couple of steps.

Two limits on what this shows:
- **A linearization at frozen state.** The bike's own response also feeds
  back, so this is a tendency the network carries, not a demonstrated
  oscillator.
- **It does not explain the whole hub chatter.** Hub sign flips run at
  12-14/s on EVERY policy, whatever its self-gain. Per-step hub change (|Δa|²)
  is highest on the two high-gain policies (0.37 and 0.24, against 0.17-0.24),
  which is a weak link at best.

Steer's self-gain is ~0 on every policy, so steer chatter goes through the
bike or the sensors, not this loop.

### 1.3 Most of the actuator power is in the chatter **[measured]**

"Actuator power" here is the mechanical power each motor delivers at its
joint: torque times speed, summed over steer and both drives, taken as an
absolute value so braking counts too, and sampled every physics step (2500/s)
while holding still. The RL policies spend 1.7-5.3 W on it; the analytic LQR
on truth spends 0.002 W. 52-63% of the RL figure is fluctuation faster than
~10 Hz (|P| minus its 100 ms moving average). Part of it is above the 25 Hz
control Nyquist — the pointer's tread speed carries 18 and 14 mm/s RMS in the
32-64 Hz and 64+ Hz bands, `chatter.py`'s band table — so it is the edges of
the held commands ringing the servos and the contact, which no per-step action
penalty can see. This measurement is why `w_copper` exists.

---

## 2. Why a reward term is the weak lever **[reasoned]**

- **It reaches the deployed mean only through returns.** The reward is charged
  on SAMPLED actions; the Pi flies the deterministic mean. The gradient on μ is
  a high-variance policy-gradient estimate, not a direct one.
- **Exploration noise pays most of it.** White noise adds ~2σ² to E|Δa|²
  whatever μ does, so part of `w_smooth` is a tax on σ rather than a price on
  the chatter.
- **It moves the violence.** `rl_general_glide_pitch_smooth` raised `w_smooth`
  and contact load went UP: peak 7.23 → 9.48× weight **[measured]**, in that
  config's header.

`w_copper` keeps the first two weaknesses but fixes the third by pricing the
consequence at physics rate. A loss term fixes the first two.

---

## 3. The family **[read]**

| method | what it adds | relevance here |
|---|---|---|
| **CAPS** (Mysore et al., ICRA 2021) | loss on μ: temporal ‖μ(s_t)−μ(s_{t+1})‖ + spatial ‖μ(s)−μ(s+ε)‖ | the baseline |
| **L2C2** (Kobayashi, 2022) | spatial neighbourhood adapted to how the state is moving, not a fixed radius | only if a fixed σ proves wrong |
| **Grad-CAPS** (2024) | penalises the SECOND difference, μ(s_{t+1})−2μ(s_t)+μ(s_{t−1}); argues CAPS over-smooths and costs agility | **our pattern exactly**: sign flips every other step. A ramp is free, a zig-zag is not |
| **ASAP** (AAAI 2026) | "similar states" from real transitions, not synthetic noise; criticises CAPS and L2C2 for invented neighbours | if the spatial term is used |
| **Kobayashi & Yamanaka** (2026) | redesigned regulariser; CAPS over-smooths, non-monotonic in its weight | a warning on weight sweeps |
| **LipsNet** (ICML 2023) | Lipschitz bound in the architecture | changes the export path — see §5 |
| **LipsNet++** (ICML 2025) | adds a learned Fourier filter over an observation history; separates observation noise from policy roughness | relevant to the AHRS line |
| spectral normalisation | cruder Lipschitz bound | — |
| **Christmann et al.** (IROS 2024) | benchmark of the above; LipsNet + CAPS / L2C2 best | Gym tasks, not hardware |

Adjacent, not losses:
- **gSDE** (Raffin et al., CoRL 2021): smooth exploration noise, `use_sde=True`
  in SB3. Considered and set aside 2026-10-01 (user: wants something more
  different).
- **Seyde et al.** (NeurIPS 2021): RL drifts to bang-bang and it is often
  near-optimal on benchmarks. Pushback: some saturation may be correct for the
  reward.
- **Chou et al.** (2017): Beta distribution, so the action cannot leave its
  bounds.
- *Where Entropy Is Measured Matters* (arXiv, 2026-08): clipped-Gaussian PPO
  means outside the interval on 82% of states — the §1.1 phenomenon. Only the
  search snippet was seen.

---

## 4. Implementation here **[reasoned]**, written against SB3 2.9.0

### 4.1 Where the loss goes

SB3 has no hook for extra loss terms. The whole loss is one line of
`PPO.train()`, `stable_baselines3/ppo/ppo.py:256`:

```python
loss = policy_loss + self.ent_coef * entropy_loss + self.vf_coef * value_loss
```

Two routes:

1. **Copy `train()` into a `SmoothPPO(PPO)` subclass** and add `+ reg` there.
   ~110 copied lines, pinned to the SB3 version. Guard it with a test: every
   λ = 0 must give one update bit-identical to stock `PPO.train()`, so an SB3
   upgrade that changes the original fails loudly. **The route to use.**
2. `super().train()`, then K gradient steps on the regulariser alone. ~30
   lines and upgrade-proof, but the objectives alternate and PPO's next update
   can undo the smoothing. A prototype only.

### 4.2 Temporal pairs: build them before the buffer is shuffled

`rollout_buffer.get()` flattens and shuffles, which destroys adjacency. At the
top of `train()`, before the first `get()`, `rollout_buffer.observations` is
`(n_steps, n_envs, obs_dim)` and already VecNormalize-normalized: 512 × 32
here. A pair is valid where `episode_starts[t+1] == 0`.

```python
def _mu(self, obs):                       # pre-clip mean: what the Pi flies
    return self.policy.get_distribution(obs).distribution.mean

def _pairs(self):
    rb = self.rollout_buffer
    o = th.as_tensor(rb.observations, device=self.device)               # (T, E, D)
    cont = th.as_tensor(rb.episode_starts[1:] == 0, device=self.device) # (T-1, E)
    return o[:-1][cont], o[1:][cont]

def _reg(self, s, s1, obs):
    sm, reg = self.smooth, 0.0
    if sm.get("lam_t"):                   # temporal
        i = th.randint(len(s), (len(obs),), device=self.device)
        reg = reg + sm["lam_t"] * (self._mu(s[i]) - self._mu(s1[i])).pow(2).sum(-1).mean()
    mu = self._mu(obs)
    if sm.get("lam_s"):                   # spatial
        eps = th.randn_like(obs) * self._sigma
        reg = reg + sm["lam_s"] * (mu - self._mu(obs + eps)).pow(2).sum(-1).mean()
    if sm.get("lam_bound"):               # keep the mean inside the box
        reg = reg + sm["lam_bound"] * th.relu(mu.abs() - 1).pow(2).sum(-1).mean()
    return reg
```

Grad-CAPS is the same with triples (t−1, t, t+1), all three in one episode.

### 4.3 Choices specific to this bike

- **`lam_bound`** goes straight at §1.1. One line; include it in every arm.
- **The temporal term** prices what §1.2's hub loop does, step to step.
- **Spatial σ from the sensor model, not a guess** (CAPS uses one generic σ).
  Raw noise per feature — AHRS roll and gyro from `sim_ahrs`, velocity from
  the encoder model — divided by `sqrt(obs_rms.var)` from
  `self.get_vec_normalize_env()`. Leave the command dims unperturbed.
  - Or perturb ONLY `prev_action` (obs 12-14): that penalises ∂μ/∂prev_a,
    the §1.2 self-loop gain, directly.
  - The AHRS error is slow (τ 0.19 s), not white, so this is a scale, not a
    noise model.
- **Not changed:** the network, the npz export, `MLPPolicy`, `chatter.py`,
  the deploy bundle and the Pi. The regulariser exists only in training.

### 4.4 Touch points in this repo

| where | change |
|---|---|
| `train_general_rl.py:1107` (construct), `:1104` (resume) | `PPO` → `SmoothPPO`, passing `smooth=` |
| `train_general_rl.py:958`, `:1000`, `:1134` | `PPO.load` for eval and export; a subclass zip is expected to load there, UNVERIFIED |
| config | an `algo.smooth:` block (`lam_t`, `lam_s`, `lam_bound`, σ rule), stamped into the move yaml's `trained:` record |
| logging | `train/smooth_reg` beside `train/loss` |
| tests | the λ = 0 equivalence test, on a cheap gym env (no model build) |

Cost: two or three extra forward passes of a 128×128 MLP per 1024-sample
minibatch, expected negligible next to MuJoCo stepping. Not measured.

---

### 4.5 As built

The sketch above became `smooth_ppo.py` with three changes: the keys are
`algo.smooth: {bound, temporal, second}` (a misspelt key is an error, not a
silent 0); stock `PPO` is used whenever every weight is 0, so old configs train
exactly as before; and the per-term values (unweighted) are logged as
`train/smooth_*`. `tests/test_smooth_ppo.py` (marker `trainer`) holds the λ = 0
bit-identity, the episode-boundary windowing, and both load paths.

**Weights, calibrated 2026-10-01** on the pointer's trained actor
(`runs/general_rl_cmd_curriculum2b/best_model.zip`, one 16 × 512 rollout).
Gradient norms on the actor:

| PPO's policy loss | bound at λ 1 | temporal at λ 1 | second at λ 1 |
|---|---|---|---|
| 1.57 | 4.2 | 10.7 | 27.6 |

Each weight puts its term at a quarter of PPO's gradient there: **bound 0.1,
temporal 0.04, second 0.015**. One calibration point, on a policy that already
chatters; trained from scratch the terms start near zero and grow only with
the chatter.

## 5. Arms

Three training runs, each the current base config plus one loss setting, all
compared against a base run with no loss term (`rl_general_smooth_base.yaml`:
the pointer's config plus the detailed drivetrain at P 100):

1. **Bound only** (`lam_bound`): charge the network for any mean past ±1.
   Nothing else changes. Answers whether the bang-bang saturation of §1.1 is
   itself driving the chatter.
2. **Bound + temporal** (CAPS's temporal term): also charge any change in the
   mean from one step to the next. Smooths everything, ramps included.
3. **Bound + second difference** (Grad-CAPS): charge only a change in the
   CHANGE — a zig-zag costs, a steady ramp is free. Aimed at the
   every-other-step sign flipping, without slowing deliberate moves.

Arm 1 is in both others so that 2 and 3 differ from it in one term each.

What to read off each: `chatter.py` (saturation, the pre-clip table,
flips/s, |Δa|² per family, the wheel bands), the prev-action gain (§1.2) and
actuator power at hold (§1.3). The last two are scratch probes today and
belong in `analysis/` before the first arm.

LipsNet / LipsNet++ change the network, so they change the npz export and
`MLPPolicy` on the Pi. They are a bigger step than any of the above.

## 6. Open

- Whether `w_copper` alone removes the zig-zag. If it does, this stays parked.
- The servos' own current and load as observations, if the copper term
  matters: XC330 Present Current reads as signed BUS current (XL330 bench fit,
  `bam-current-position.md`); the XC430 drives have no current register, only
  duty-derived Present Load.
- Whether the AHRS error drives part of the chatter on the sensor-trained
  policies. Truth-trained `smooth_diff_og` chatters too, so it is not all of it.

## Sources

- [CAPS](https://arxiv.org/pdf/2012.06644) ·
  [L2C2](https://arxiv.org/pdf/2202.07152) ·
  [Grad-CAPS](https://arxiv.org/abs/2407.04315) ·
  [ASAP](https://arxiv.org/pdf/2601.18479) ·
  [Redesigning Regularization](https://arxiv.org/pdf/2606.13169)
- [LipsNet](https://proceedings.mlr.press/v202/song23b/song23b.pdf) ·
  [LipsNet++](https://xjsong99.github.io/LipsNet_v2/) ·
  [Benchmarking Smoothness](https://arxiv.org/pdf/2410.16632)
- [gSDE](https://arxiv.org/pdf/2005.05719) ·
  [Bang-bang](https://arxiv.org/pdf/2111.02552) ·
  [Where Entropy Is Measured Matters](https://arxiv.org/html/2608.24488)
