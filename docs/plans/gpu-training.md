# GPU training: MuJoCo Warp + PufferLib 5.0

> **Status: PLAN, 2026-10-14. Nothing built.** Goal (user): the whole
> physics + RL stack on one NVIDIA GPU -- MuJoCo Warp for the bike,
> PufferLib 5.0's PPO for the learner. Phases are gated: each one ends in a
> number that decides whether the next is worth starting.

Evidence tags as in `general-rl-improvements.md`: **[measured]** (a number or
a code read backs it), **[read]** (docs or source read, not run),
**[reasoned]**.

---

## 0. Where we start

| | today (CPU) | source |
|---|---|---|
| box throughput, whole training | ~2,100 policy steps/s = ~105k physics steps/s (32 envs, rigid module, ~7.8 s per 16,384-step update) | [measured] 2026-10-13/14 |
| `mj_step` alone, per core | 12.4 us (M4, rigid, ideal drive); ~47 us on the box | [measured] |
| env step, one box process | 3.76 ms: `mj_step` 63 %, drivetrain 13 %, odometry 8 %, gearbox 5 % | [measured] |
| PPO `train()` | ~1.9 s of each update | [measured] |

**The bar a GPU stack must clear:** end to end, ~105k physics steps/s
today, every hook, the obs and the learner included. User (2026-10-14):
"still interesting at like 2x", more worth it at 5-10x. So:

| end to end | physics steps/s | policy steps/s |
|---|---|---|
| 2x -- worth finishing | ~210k | ~4k |
| 5x | ~525k | ~10k |
| 10x | ~1M | ~20k |

What MJX on the Mac CPU already told us (`aow-sim-gpu` env, MuJoCo 3.10.0):
- The rigid training model loads into MJX unmodified (after 261014-03 removed
  the wings' contact sensors) and tracks C MuJoCo to ~0.1 mm over 100 ms.
  [measured]
- ~1.06 MB per world (XLA's own accounting at 64-1024 worlds): ~20k worlds
  in 24 GB. [measured on CPU; GPU unverified]
- MJX's JAX backend checks every candidate contact pair every step; the 16
  roller meshes are ~60 % of its step, the blades 12 %, the 3,138-vertex
  tyre 3 %. C MuJoCo skips far pairs (lifting the bike: -3 %). MJWarp is
  documented to have a broadphase. [measured / read]

---

## 1. Background for phases 3-4: PufferLib 5.0 is CUDA C

Not a phase. Phase 1 benchmarks physics only; this section is here because
it decides how the env has to be built (phase 2) and bridged (phase 3), and
phase 1 runs the cheap probes of its risks (below) while the GPU is rented.

Read from branch `5.0` at `ea7f989` (2026-10-14) [read]:

- The trainer is ~10k lines of CUDA C (`src/pufferl.cu`, `src/algo.cu`).
  **No PyTorch, no Python in the loop.** Custom losses mean editing
  `algo.cu` ("fork the project and edit directly", puffer.ai/docs).
- Envs are C (CPU) or CUDA (GPU). The GPU contract (`src/pufferenv.h`,
  example `ocean/robot_arm/robot_arm.cu`):
  `puf_vec_create(n, kwargs, obs*, actions*, rewards*, terminals*)` hands
  the env DEVICE pointers once; `puf_step` launches kernels on the stream
  given by `puf_bind_stream`. The env steps the whole batch.
- Default network PufferNet: MinGRU (a recurrent layer) + highway nets,
  `hidden_size` 128, `num_layers` 4. Continuous actions with a learned
  `logstd`: yes. Optimizer: Muon (default lr 0.015). Precision bf16
  default, `--float` for fp32. Training is GPU only; CPU is eval only.
- Defaults vs ours: `total_agents` 4096, `horizon` 64, `minibatch_size`
  8192, `gamma` 0.995, `gae_lambda` 0.90, `vf_coef` 2.0, async (actor one
  epoch stale). robot_arm (their continuous-control example): 4096 agents,
  horizon 128, lr 3e-4, gamma 0.99, lambda 0.95.
- Their GPU robot arm wrote its OWN rigid-body physics in CUDA (3.7k
  lines). No example drives MuJoCo or any Python simulator.

**So the bridge is the hard part.** There is no C/CUDA MuJoCo for the
GPU: MJWarp is Python-launched Warp kernels, which Warp JIT-compiles to
CUDA from Python source, specialised to the model at load time [read].
PufferLib calls a C function per step. Once captured, though, a CUDA graph
IS the pure-C form: launching it needs no Python, only the setup does.
The design that keeps Python out of the per-step path [reasoned, each step to be verified in phase 1]:

1. `puf_vec_create` embeds CPython once, imports our env module, and passes
   it PufferLib's device pointers.
2. Python wraps those pointers as Warp arrays (no copy), builds MJWarp with
   `nworld = n`, and **captures one whole policy step as a CUDA graph**:
   action -> ctrl kernel, 50 x (hook kernels + `mjw.step`), obs / reward /
   done / auto-reset kernels.
3. It returns the graph's executable handle. `puf_step` is then one
   `cudaGraphLaunch` on PufferLib's stream.

Risks in that chain: PufferLib captures its own rollout into a CUDA graph
(`cudagraphs = 1`) -- whether a launch of OUR graph nests inside theirs;
Warp's graph handle being reachable from C; `num_buffers` 2 means two env
batches on two streams (use 1, as robot_arm does).

**When Python runs, in this design** [reasoned]:

| when | Python? | what |
|---|---|---|
| startup | yes, once | build the model, wrap PufferLib's buffers, capture the step graph |
| every policy step (50 substeps, hooks, obs, reward, resets, randomisation) | **no** | one `cudaGraphLaunch` from PufferLib's C loop |
| every PPO update | **no** | PufferLib's CUDA C learner |
| curriculum difficulty | no, if kept on the GPU | today `GeneralEnv` advances it per episode; in the port, a kernel (or C) reads episode scores and moves the one scalar the command kernel reads |
| periodic eval (`BestByScore` today, every 1M steps) | optionally, rarely | either a Python callback every N updates on a spare world batch, or none: save checkpoints and score them on the CPU stack afterwards |
| export to `moves/*.npz` + yaml | yes, offline | after the run, from PufferLib's flat fp32 checkpoint |

So mid-run Python is at most a rare callback, never in the step loop.

**The plain-PyTorch fallback**, decided at choice B below: MJWarp -> torch
tensors (zero-copy through Warp's interop) -> a torch PPO on the GPU. It
keeps `SmoothPPO` and our MLP as they are. It gives up PufferLib's trainer
speed -- which matters less here than for their envs, because our step is
physics-heavy (50 substeps) and the learner was ~25 % of a box update.
[reasoned]

---

## 2. Phases and gates

### Phase 1 -- physics only on a rented GPU (~1-2 h, a few $)

A prepaid GPU host (user, 2026-10-14: RunPod or similar, so a forgotten
instance can only burn the balance), one L4 / A10-class card (24 GB) or
better. Pin versions: MuJoCo 3.10.0 + the matching `mujoco_warp`, Warp,
CUDA 12.x. Write and dry-run every script on the Mac first (Warp runs on
CPU), so the rented hours are runs, not debugging.

- Load the training model (ideal drivetrain, rigid V3) into MJWarp; list
  what it refuses. Watch: elliptic cones, the Newton solver, mesh-mesh
  pairs, joint/tendon equalities, tendon limits (roller slop, detailed
  drive only).
- Trajectory check against C, as done for MJX.
- `nworld` sweep 1k / 4k / 16k / 32k: physics steps/s and memory, CUDA
  graph captured. Same with MJX-on-GPU for reference.
- Per-world model parameters: domain randomization scales mass, friction
  and actuator gains per episode, and the gearbox friction rewrites
  `dof_frictionloss` every substep. **Can MJWarp vary model fields per
  world?** ("Heterogeneous worlds -- per-world meshes or number of DOF --
  are not available"; batched parameters unclear.) [read]

**Bridge probes, same instance (~1 h more).** Each is a small script, and
each answers a risk in section 1 before phases 2-3 are built around it:

| probe | answers |
|---|---|
| build PufferLib 5.0, train `robot_arm` for ~5 min | it builds on this CUDA/driver; its end-to-end steps/s and learner share = the ceiling our env adds to |
| `wp.array(ptr=..., ...)` over a `cudaMalloc`'d buffer from a ctypes C stub | zero-copy wrapping of PufferLib's buffers |
| capture `mjw.step` x50 as a Warp graph, launch its exec handle from C on a foreign stream | Python-free stepping (the core of the bridge) |
| that launch inside a C `cudaStreamBeginCapture` | nesting in PufferLib's own rollout graph (open question 2) |
| PufferLib with `num_layers = 0` | network option 1 in phase 4: an MLP-only policy without editing `algo.cu` |

None of these is a stop gate; a failure moves phase 3 to the torch
fallback, or phase 4 to option 2.

**Gate A:** per-world parameters possible, AND physics alone fast enough
that 2x end to end is plausible once the hooks and the learner take their
share: >= ~0.5M physics steps/s to continue, >= ~2M to expect 5-10x.
Below ~0.5M, stop: the CPU stack stays. (The hook and learner share on GPU
is a guess until phase 3; on the box they are ~37 % of an env step plus
~25 % of an update.) [reasoned]

### Phase 2 -- the env in Warp kernels (most of the work)

Each Python piece of `GeneralEnv` becomes a kernel over worlds. Develop and
unit-test on the Mac -- Warp runs kernels on CPU -- against the Python
reference, float32 tolerance:

| piece | per | note |
|---|---|---|
| action scale + `mix` + steer target integration | policy step | trivial |
| drivetrain servo PI, delays, filter, stick friction | substep | `drivetrain_model.DrivetrainSim.pre_step` |
| detent | substep | same |
| gearbox friction (load-proportional limit) | substep | writes a model field -- see gate A |
| AHRS `tm151_filter` | 100 Hz | needs a per-world RNG |
| odometry (counts, RateFilter, `front` mode) | 100 Hz | `hw/odometry.py` is the Pi's own; the kernel is a port, the reference stays |
| obs, reward, fall, truncation | policy step | |
| command families + curriculum, resample | policy step | the curriculum is a global scalar today |
| domain randomization + reset | episode | auto-reset inside the graph |

**Verification:** replay a recorded action sequence through both envs,
episode by episode, compare obs and rewards to tolerance. A digest-style
check that the GPU env reads the same params as `GeneralEnv`.

### Phase 3 -- the PufferLib bridge

`ocean/aow_bike/` in a PufferLib fork: the embedding `.cu` (section 1), an
`aow_bike.ini`. Smoke: a random policy for 1M steps, rewards and resets
sane. Measure end-to-end steps/s.

**Choice B (not a stop gate):** past gate A the env is on the GPU either
way, so the only question here is which learner sits on it -- PufferLib
through the bridge, or the torch fallback. User (2026-10-14): even ~10 %
from PufferLib's trainer is worth having on a rented GPU. Take PufferLib if
the bridge works; fall back to torch if it fights PufferLib's graphs or
streams for long.

### Phase 4 -- algorithm parity

- **Network.** 5.0 has NO built-in MLP [read, `algo.cu` @ ea7f989]:
  `build_arch` always wires encoder (one linear, no activation; an env may
  replace it via `create_custom_encoder`) -> `num_layers` MinGRU layers ->
  decoder (one linear giving the action means AND the value: a shared
  trunk, where SB3 gives us separate 128x128 tanh actor and critic). Each
  MinGRU layer: `c = W x` (3H outputs), `z = sigmoid(c_gate)`,
  `h~ = c_h >= 0 ? c_h + 0.5 : sigmoid(c_h)`, `h = lerp(h_prev, h~, z)`,
  output `sigmoid(c_proj) * h + (1 - sigmoid(c_proj)) * x`; `h` per layer is
  the carried state, reset on terminal. Options, cheapest first:
  1. `num_layers = 0` + our MLP as the env's custom encoder: the MinGRU
     loop then does not run. Untested -- zero-size buffers may assert.
  2. An MLP `Network` in `algo.cu` beside MinGRU (forward, backward, init:
     GEMMs exist as `puf_mm`, so tanh kernels and their backward).
  3. Keep MinGRU and port it to replay: the Pi's runtime, `general_spec`,
     the export, and every tool that calls `pol.action(o)` statelessly
     (`per_command`, `chatter`, teleop) gain a hidden state to carry and to
     reset on engage. The arithmetic is trivial in numpy (4 x 128x384
     matmuls, ~6x our MLP's, nothing for the Pi); the state is the work.
  Prefer 1, then 2. Memory (option 3) may be worth having on its own
  merits -- it could stand in for the velocity filters -- and gets tested
  on the CPU stack first (`MinGRU` in SB3 vs today's MLP, same config):
  if it wins there, option 3 becomes the default rather than a fallback.
- **Smoothness terms** (bound, temporal, second) in `algo.cu`'s loss:
  temporal needs the policy mean at consecutive steps, which the
  [agents x horizon] buffer has.
- **Export** to `moves/*.npz` + yaml in today's format, so `per_command.py`,
  teleop and the Pi replay it on the CPU plant untouched -- the real test of
  a GPU-trained policy is the C MuJoCo eval.

### Phase 5 -- first comparison

Five seeds, GPU-style hyperparameters (thousands of envs, short horizon;
PPO is a different algorithm at that shape -- see the 10-14 discussion),
against `queue_general_v3`'s five on one `per_command.py` grid. Report
wall time to equal score, not steps/s.

---

## 3. Housekeeping

- **Branch, in a worktree.** The user OK'd a branch (2026-10-14). Other
  sessions share this checkout, so checking a branch out HERE would swap
  their files: use `git worktree add` (and `PYTHONPATH=src` inside it, see
  CLAUDE.md). The user still commits.
- **Envs:** `aow-sim-gpu` (local, made 2026-10-14: MuJoCo/MJX 3.10.0, JAX
  0.11.2; reads the repo through `PYTHONPATH=src`, nothing installed into
  it). The GPU machine gets its own.
- **The box's GTX 1050** (Pascal, 2 GB) is not a target: below what bf16
  kernels and modern MJWarp builds expect. [reasoned, not tested]
- **Do not touch the CPU stack.** `GeneralEnv`, `SmoothPPO` and the
  configs stay the reference the GPU env is checked against.

## 4. Open questions

1. Per-world model parameters in MJWarp (gate A).
2. Nesting our captured graph inside PufferLib's (gate B).
3. MLP in PufferLib vs MinGRU on the Pi.
4. Whether bf16 training holds up for a balance controller, or `--float`.
5. Hyperparameters at 4k-16k envs: PufferLib's defaults are tuned for
   their envs, not a 50-substep physics step.

Sources: puffer.ai/docs.html; PufferAI/PufferLib branch 5.0 @ ea7f989
(`src/pufferenv.h`, `config/default.ini`, `config/robot_arm.ini`,
`ocean/robot_arm/robot_arm.cu`); mujoco.readthedocs.io MJWarp pages.
