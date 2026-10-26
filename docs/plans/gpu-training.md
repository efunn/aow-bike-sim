# GPU training: MuJoCo Warp + PufferLib 5.0

> **Status, 2026-10-14: phase 1 done, gate A passed** (2.36M physics
> steps/s on an A40; the bridge cannot nest graphs, so phase 3 uses two
> launches per step -- see phase 1's run). **Phase 2: the env runs on
> CUDA as one captured graph per policy step, 30k policy steps/s at 16-32k
> worlds with v3_s0 driving (0 falls) -- ~14x the box.** The eval grid
> matches GeneralEnv, and the env costs x1.05 of bare MJWarp on the same
> model. **Phase 3: PufferLib trains on it end to end on an A40 and learns
> to ride: 20M steps in 14 min (21-25k SPS, ~10x the box), episodes from
> 0.4 s to 13.4 s of 15, ~72 % ending without a fall** (training-time
> numbers, not v3's eval grid; the first run's settings learned nothing --
> see phase 3). **2026-10-15: a 1 h A40 run (~$0.71) trains a policy
> that matches v3's CPU seeds on the common grid** -- 0/20 falls, both
> straight commands forward, 7/9 turns forward (v3 seeds 4-8/9), holds
> still -- after fixing PufferLib's [-1, 1] reward clamp (velocity
> tracking earned nothing), raw advantages, the curriculum's pace, and
> adding SmoothPPO's temporal term (phase 3, "It worked"). Checkpoints
> convert to moves (`gpu/export_policy.py`). **2026-10-16: a second seed
> makes the same direction choice on all 20 grid commands, and turns
> faster; on an RTX
> 5090 a 72M-step run takes ~12 min (~$0.23) at 32-64k worlds, 4090 ~17
> min, A40 38; the export's GPU memory went from 0.72 MB a world to
> ~0.07** (phase 3, "Which card"). Phases 4-5 not started.
> (Updated 2026-10-16.)
> Goal (user): the whole
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

**Starting phase 1 from a fresh session** (written 2026-10-14):

- **The model, not the repo.** Phase 1 needs only the compiled model.
  Export it here and upload the file; nothing from `aow_sim` has to run
  on the pod:

  ```python
  import mujoco
  from aow_sim.build_model import load_params
  from aow_sim.control.general_env import GeneralEnv, _load_rl_config
  from aow_sim.drivetrain_model import from_env_config
  cfg = _load_rl_config("config/rl/rl_general_v3.yaml")
  env = GeneralEnv(from_env_config(load_params(), cfg["env"]), cfg)
  mujoco.mj_saveModel(env.model, "v3_rigid.mjb")
  # the probes' start state (analysis/gpu/phase1/): one reset, seed 0
  import numpy as np
  env.reset(seed=0)
  np.savez("v3_init_state.npz", qpos=env.data.qpos, qvel=env.data.qvel,
           ctrl=env.data.ctrl)
  ```

  4.5 MB; nq 21, nv 20, nu 5, 127 geoms, 74 meshes, timestep 4e-4,
  elliptic cones, Newton. Reloading the .mjb and stepping 500 steps gives
  0.0 qpos difference against the original. [measured, 2026-10-14]
  A `.mjb` is tied to its MuJoCo version: pin 3.10.0 on the pod too. The
  drivetrain, odometry and AHRS models are Python hooks, not in the file:
  with or without the drivetrain overlay the compiled model has the same
  actuators, gains, damping, armature and friction loss. [measured] The
  hooks are phase 2's problem, not phase 1's.
- **Code onto the pod:** copy files (`runpodctl send`, or scp to the pod's
  SSH address). Never put a GitHub token or key on a rented machine.
- **Waiting for a pod** (lesson of 2026-10-14): `get-pod` right after
  create, again until `runtime.ports` shows the 22/tcp mapping; then start
  `analysis/gpu/phase1/wait_ssh.sh <ip> <port> <createdAt epoch>` IN THE
  BACKGROUND and work on while it waits. It logs TCP-open and ssh-ok times
  from createdAt. Never a foreground loop, and never the `ssh.runpod.io`
  proxy (needs a pty; the first run's loop over it blocked the session
  until the user's message pushed it aside).
- **Waiting for a pod job:** launch it detached (`nohup ... &`, with
  `< /dev/null` too, or the launching ssh stays open until the job ends),
  writing `results/DONE` last. Then ONE background ssh that blocks on the
  pod -- `while [ ! -f results/DONE ]; do sleep 5; done; <print results>` --
  so its exit is the notification. Not timed sleeps: on 2026-10-14 those
  under-guessed a 14 min run twice.
- **Choosing the GPU** [reasoned from the A40 run]: memory is not the
  limit (~2 GB at 32k worlds); compute is (throughput flattens past ~16k
  worlds). So rank cards by compute per dollar, 16-24 GB is plenty for
  physics plus a small net. Whether MJWarp's step scales with FP32 compute
  or with memory latency is untested: a 4090 (~2x the A40's FP32, $0.89/h
  secure, $0.34 community) on the same `run_all.sh` would show it.
- **Warp's kernel cache comes back with results/** (`warp-cache.tgz`) for
  the tentative `analysis/gpu/phase1/Dockerfile` (not built; where it would
  live and what is open is in its header). Phase 2's, with the env's
  kernels compiled for sm_86: `analysis/gpu/phase2/results_a40/
  warp-cache.tgz` (8.8 MB; cold vs warm `fly_policy` 174 vs 117 s, so
  ~1 min of compile it would save).
- **Spending.** Confirm with the user before creating a pod (GPU type and
  $/h from the catalog read). TERMINATE it at the end of the session.
  Stopping leaves its disk billing. Write the measured numbers back into
  this section with `[measured]` tags before ending.
- **RunPod access** (2026-10-14): the plugin is installed at user scope.
  Signing in through the desktop app's `/mcp` failed (`redirect_uri is not
  allowed`), and the terminal CLI's sign-in did not reach the desktop
  session either. What worked (user): desktop app Settings > Connectors,
  delete the RunPod server, re-add it by hand, then the browser sign-in
  link; the connector came up in the SAME session, no restart. The account
  has no SSH keys registered: pass `PUBLIC_KEY` (contents of
  `~/.ssh/id_ed25519.pub`) in the pod's env instead of changing account
  settings.

**Mac dry run, 2026-10-14** (`analysis/gpu/phase1/`: `probe_physics.py`,
`pod_setup.sh`, the exported `v3_rigid.mjb` + `v3_init_state.npz`;
`mujoco-warp 3.10.0.3`, Warp 1.18.0, CPU device):

- **Loads unmodified.** `put_model` refuses nothing: elliptic cones,
  Newton, the joint/tendon equalities, tendon limits and the meshes all
  compile and step. [measured]
- **Per-world parameters: yes, in the API.** `put_model(mjm,
  batch_sizes={field: n})` gives a field a per-world leading dimension;
  kernels index it `worldid % n`. `body_mass`, `body_inertia`,
  `geom_friction`, `geom_solref`, `actuator_gainprm/biasprm/forcerange`,
  `dof_frictionloss/damping/armature` are all batchable [read, io.py and
  types.py]. Four worlds at scales 1/0.8/1.2/1.5 of mass, friction, gains,
  friction loss and damping: world 0 equals the unbatched run, the others
  move away from it, all finite. [measured] Gate A's first half is met;
  rewriting `dof_frictionloss` each substep (the gearbox) is an array write
  and is phase 2's.
- **Tracking vs C differs, and MJX differs the same way.** Open loop,
  smooth random steer and motor commands, 100 ms: chassis 0.73-1.0 mm and
  0.26-0.31 deg roll off C. MJX on the same run: 0.92 mm, 0.10 deg roll, and
  float32 = float64 to 0.001 deg, so precision is not the cause. Both GPU
  engines lay up to 4 contact points on a mesh resting on the plane where C
  lays 1 here (Warp: 6-7 contacts, C: 2), and the floor-touching roller's
  spin is the largest joint difference (0.1 rad by 100 ms). A C run nudged
  1e-7 m stays within 0.001 mm over the same 100 ms. [measured] The earlier
  "MJX ~0.1 mm over 100 ms" was a gentler case; this is the harder one.
  Consequence: a GPU-trained policy needs a sim-to-sim check in C before
  anything else (phase 5), not only a replay match in phase 2.
- **A Python-free bridge may already exist: Warp's APIC.** Warp 1.18
  saves a captured graph to a `.wrp` file (`ScopedCapture(apic=True)`,
  `capture_save`) and libwarp exports a C API to load and run it:
  `wp_apic_load_graph`, `wp_apic_get_param_ptr` (device pointer of a bound
  array), `wp_apic_get_cuda_graph(_exec)` (a plain `cudaGraph_t` /
  `cudaGraphExec_t`), header comment "standalone, no Python needed" for
  CUDA [read, `warp/native/apic.h`]. On the Mac CPU: 50 `mjw.step`s x 8
  worlds captured, saved, reloaded in a fresh process: 0.0 difference
  from the live graph; restoring qpos/qvel/act/ctrl/qacc_warmstart/time
  through the bound params and relaunching: 0.0 again. [measured] (Bind
  `history` too: the steer delay line, this morning's reset bug.) A CPU
  load still needs Python to link the kernels, so the C-only test is
  CUDA-only: `bridge_probe.cu` (T1-T5: load from C, replay on a C stream,
  restore from C, rate, then the graph inside a C stream capture two ways --
  `cudaGraphLaunch` while capturing, and a child-graph node). If T5 passes,
  phase 3's core is mostly done; probes 2-4 of the table above are folded
  into it. Cost: the `.wrp` snapshots every buffer, ~0.7 MB per world
  (15.9 MB at 8, 55.5 MB at 64) [measured], so the probe runs 1k and 4k.
- **Run order on the pod:** `pod_setup.sh`, then `run_all.sh` (physics
  probes, graph export + C probe, then `puffer_probe.sh`: PufferLib at
  ea7f989, robot_arm 5 min, `num_layers=0` 2 min). All of it writes JSON
  lines to `results/results.jsonl`.
**Phase 1 run, 2026-10-14, RunPod A40 48 GB** (secure, CA-MTL-1, $0.59/h;
`runpod/pytorch:1.0.2-cu1281-torch280-ubuntu2404`, driver 580 / CUDA 13.0,
toolkit 12.8; pod up 23 min, ~$0.23, terminated). Raw output:
`analysis/gpu/phase1/results_a40/`. All [measured], one run each.

| | result |
|---|---|
| per-world params on GPU | all 5 fields: world 0 = unbatched, others diverge, finite |
| C tracking on GPU | same as the CPU dry run (0.67 mm, 0.24 deg roll at 100 ms) |
| MJWarp, CUDA graph of 50 steps | 1.10M / 1.99M / 2.30M / **2.36M** physics steps/s at 1k / 4k / 16k / 32k worlds |
| memory | ~2 GB used at 32k worlds (free 42.3 of 44.4 GB) |
| MJX, same GPU | 15.6k / 14.8k steps/s at 1k / 4k: ~150x slower, as the dense contact pairs predicted |
| graph launched from C (T4) | 1.23M at 1k, **2.20M at 4k** (Python launch 1.18M / 2.03M) |
| C load / replay / restore (T1-T3) | ok; qpos 7-9e-9 off the Python reference |
| graph nested in a stream capture (T5a, T5b) | **fails**: `cudaGraphLaunch` while capturing is not permitted; child-graph node "operation not supported" |
| child node with no capture (T6) | also "not supported": this graph cannot be a child at all. Nodes: 5350 kernel, 250 memcpy (all D2D), 1000 memset. Cause unknown |
| PufferLib ea7f989 build | ok after two fixes (below), 125 s |
| robot_arm, 5 min | 75M steps; 475k SPS early, 184k by the end (rollout 90 -> 97 %) |
| `policy.num_layers = 0` | crashes: CUDA assert in robot_arm's `puf_step`. Phase 4 option 1 is out; option 2 (edit `algo.cu`) |

**Gate A: passed.** Per-world parameters work, and physics alone is 2.36M
steps/s, above the >= ~2M line, ~22x the box's whole-pipeline 105k.
Caveat: the probe runs open loop, so the bikes have fallen within ~0.3 s
and lie on their side (~71 deg roll), still in contact with the floor.
The step cost hardly depends on it: same run, one world, MJWarp on the
Mac CPU, Newton iterations per step 2.8 upright (0-0.1 s, 3.7 contacts)
vs 2.4 lying (0.3-0.5 s, 2.7 contacts) [measured]. So the sweep is at
most ~10-20 % optimistic [reasoned]. Throughput is flattening by 16k-32k
(2.30M -> 2.36M) with memory to spare: past ~16k worlds the GPU is busy,
more worlds add little. Nothing above 32k was tried.

In policy steps: one 20 ms control step is 50 substeps, so 4k worlds at
2.2M physics steps/s is ~93 ms per control step, ~44k policy steps/s,
against the box's ~2.1k. [reasoned from the measured rates; no hooks,
obs or learner yet]

**Gate B's open question, answered no:** the Warp graph does not nest in
PufferLib's capture. Phase 3 instead launches two graphs per control step
on one stream (PufferLib's, then the Warp exec loaded from C), which T2-T4
show works with no Python. **The seam** [read, `src/pufferl.cu`
`rollout_start`, ea7f989]: for a GPU env PufferLib captures the WHOLE
horizon (`horizon` = 64-128 steps of `pufferl_forward_step` + `puf_step`)
as one graph on its first rollout, and our `puf_step` would run inside
that capture, which is exactly what fails. Two ways out, both small:
`base.cudagraphs = -1` runs the same loop eagerly (our `puf_step` then
launches the Warp exec itself; the forward pass loses its graph), or an
edit of ~20 lines there to capture the forward pass per step and launch
it and the Warp graph back to back. Cost of the extra launches: a graph
launch is microseconds; a control step at 4k worlds is ~93 ms of physics,
so even 2 launches x 128 steps a horizon is ~0.01 % [reasoned; launch
overhead not measured here]. PufferLib's one-graph-per-horizon design
pays off for its own envs, whose steps take microseconds; ours does not
need it.

**Setup timings** (for a Docker image; user asked): pod create -> SSH
~156 s is MY polling, not RunPod's boot: the API said RUNNING at create,
the container's uptime was 136 s at the read where the SSH port first
showed, and my wait loop sampled every 10+ s. The real boot was not
measured; time it with a tight loop next rental. apt 23 s (+ libGL 2 s); pip 38 s (797 MB pip cache); Warp's
first-step kernel compile **45 s** (19 MB cache in `~/.cache/warp`); MJX
JIT inside its 234 s sweep; PufferLib clone 26 s, compile 125 s (ccache
did not hit: 8 KB). Baking the pip installs, the Warp kernel cache and
the PufferLib binary into an image would save ~4 min a rental.

**Fixes found on the pod** (in the scripts now): the image has CUDA 12.8 at
`/usr/local/cuda/bin` but not on PATH; Warp's Linux library is `warp.so`
(`libwarp.dylib` on macOS); PufferLib's `build.sh robot_arm` greps
`robot_arm.cu` for the `obs_t` typedef that lives in `robot_arm.h`, so it
refuses before compiling (worked around with a sed); raylib links `-lGL`
(`libgl-dev`). `save_graph.py --no-mempool` fails: a Warp capture needs
the mempool. The `.wrp` on GPU is ~45 KB per world (61 MB at 1k, 190 MB
at 4k), not the CPU run's 0.7 MB.

- **The probe's controls.** `drive_a`/`drive_b` are zero-gain integrators
  the drivetrain hook feeds; `steer` and `drive_*_motor` have no
  ctrlrange, so a ctrlrange-based generator commands them 0. The probe
  drives the unlimited ones at +-0.3 (rad, N m).

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

**Verification:** ~~replay a recorded action sequence through both envs,
episode by episode, compare obs and rewards to tolerance~~ -- cannot work
against C MuJoCo: phase 1 put MJWarp's contacts 0.7 mm / 0.24 deg off C in
100 ms, and a balancing bike amplifies it. Instead (2026-10-14):

1. **Per piece, exact.** `analysis/gpu/phase2/record_reference.py` runs
   GeneralEnv (v3 config, v3_s0 driving) and snapshots MjData around every
   hook on every substep (`traces/gpu_phase2/reference.npz`). Each kernel
   gets the recorded inputs, carries its own state, and must reproduce the
   recorded outputs (`check_<piece>.py`). No physics in the loop.
2. **End to end, same engine** (user's suggestion): a hybrid reference --
   GeneralEnv's Python hooks, obs and reward on MJWarp physics (one world,
   MjData copied across each substep) -- against the all-Warp env. One
   engine, so only the port differs. Chaos still grows float32-vs-float64
   hook rounding; measure the horizon over which they agree.
3. **Behaviour.** v3_s0 flown in both on the per_command grid: fall rate,
   tracking. Random draws (randomization, commands, AHRS noise) compared as
   distributions, since Warp's RNG cannot reproduce numpy's stream.

A digest-style check that the GPU env reads the same params as `GeneralEnv`
still stands: every kernel's constants are built FROM a live GeneralEnv's
hook objects (`const_from`), never from the yaml separately.

**Progress** (2026-10-14; code in `src/aow_sim/gpu/`, checks in
`analysis/gpu/phase2/`; reference 2 episodes x 5 s, 25,000 substeps):

| piece | kernel | vs reference [measured] |
|---|---|---|
| drivetrain servo, delays, stick friction, detent | `drivetrain.pre_step` | motor torque 0.15 % rel; applied force 3 % (float32, below) |
| steer gearbox + headset friction | `gearbox.pre_step` | 1e-6 rel, rows shuffled each substep; rest branch 9.2 % of substeps |
| action scale, steer target, mix | `action.apply_action` | 3e-6 / 3e-8 |
| AHRS tm151_filter (fed numpy's own draws) | `ahrs.sample` | quat 1.3e-7, gyro 3e-7, accel 6e-6; no growth over 5 s |
| odometry counts / front | `odometry.advance` | v_lon 2.6e-7, v_lat 1e-7 m/s; 0 of 998 ticks off by > 1 mm/s |
| heading, obs (17), reward, fall, truncation | `obs_reward.obs_reward` | obs <= 1.2e-7, reward 6e-7, flags identical over 500 steps |
| command families, resample schedule, curriculum (per world) | `commands.after_step` | 100k draws a side at d 0/0.3/0.6/1: family shares within sampling noise of each other and of `_family_weights`; |v|, |step|, interval quantiles agree |
| reset: randomization, pose, actuator state, every hook, first command, AHRS prime, first obs | `reset.reset_worlds` + `ForwardGuard` + `obs_reward.reset_obs` | randomization off: qpos 1.8e-8, delay line / qvel / act exact, drive buffers 1e-9, deterministic obs entries exact; on: ranges, means, one friction and one supply draw per world |
| assembly | `env.GpuGeneralEnv` | v3_s0, 8 worlds x 5 s, training randomization: 0 falls, 2.846 reward/step (GeneralEnv: 0, 2.852). Past 750 steps: all worlds truncate, reset, curriculum 0 -> 0.02, fly on |
| drivetrain angle rebase (once per policy step) | `rebase.rebase` | a consistent upward shift (~270 rad) then rebase vs untouched: obs diff 0.0014 / 0.0005 / 0.0014 / 0.086 after 1 / 2 / 5 / 10 steps, as a 1e-6 rad nudge (0 / 0.0005 / 0.0012 / 0.016); input a alone (broken): 5 from step 1 |
| hybrid end to end (user's suggestion) | `analysis/gpu/phase2/hybrid.py` | below |
| CUDA, the whole step (policy + env + auto-reset) as one graph | `analysis/gpu/phase2/bench_env.py` | A40, below |
| eval grid in both envs | `analysis/gpu/phase2/eval_grid.py` | built, Mac smoke only |

**The hybrid** -- GeneralEnv's own Python hooks, obs and reward on MJWarp
physics (one world) against GpuGeneralEnv, so one engine and only the port
differs. v3_s0, 0.5 m/s, randomization off, the Warp env fed the hybrid's
numpy draws [measured, 2026-10-14]:

| after | obs diff, open loop | 1e-6 rad nudge (Warp vs itself) |
|---|---|---|
| reset | 2.3e-10 | 0 |
| 1 step | 7e-9 | 4.5e-7 |
| 5 steps | 4.3e-4 | 0.033 |
| 0.5 s | 0.0029 | 0.033 |
| 1 s | 0.11 | 0.4 |

The port differs from the Python by less than a 1e-6 rad nudge's growth.
Open loop (the Warp env replaying the hybrid's actions) it fell at 1.6 s:
a balancing bike fed corrections chosen for a state chaos has moved away
from does. Closed loop (each env flown by the policy on its own obs), 5 s:
neither falls, mean reward/step 2.876 (hybrid) vs 2.888 (Warp).

Three departures in the hybrid, each so the two are driven alike: the
gearbox row found by (type, id); the odometry ticking every 25th substep by
count; the Warp env given the hybrid's AHRS draws, detent phase,
misalignment, drift sign and command.

**The pod run** (2026-10-14, A40 secure EU-SE-1, $0.59/h, ~23 min, ~$0.23).
Everything above ran on the Mac's CPU; this is CUDA. Copied (no git, no
token): `src/`, `config/`, `moves/general_rl_v3_s0.*`, `analysis/gpu/phase2/`,
the two phase 1 pod scripts, `pyproject.toml`. Then
`analysis/gpu/phase1/pod_setup.sh` and `analysis/gpu/phase2/run_pod.sh`;
logs in `analysis/gpu/phase2/results_a40/`. [measured]

1. **It flies on CUDA.** `fly_policy.py`, v3_s0, 8 worlds x 250 steps:
   0 falls, 2.864 reward/step (GeneralEnv 2.861); warm rerun 0, 2.859.
2. **The whole step captures** -- policy, env and auto-reset in one CUDA
   graph, one launch per policy step. The MLP is Warp kernels
   (`bench_env.WarpMLP`, 2.4e-7 from numpy), so nothing leaves the GPU.
   `bench_env.py --check`, 64 worlds x 250 steps: eager 0 falls 2.872,
   graph 0 falls 2.870 reward/step.
3. **Speed** -- the whole env, 200 graph launches per row; random actions
   fall every ~22 steps, so their rows reset constantly:

| worlds | random: policy steps/s | v3_s0: policy steps/s | v3_s0 physics steps/s | eager (no graph), v3_s0 | GPU mem |
|---|---|---|---|---|---|
| 1,024 | 12.3k | 12.2k | 0.61M | 2.1k | 455 MB |
| 4,096 | 24.7k | 23.6k | 1.18M | 7.4k | 615 MB |
| 16,384 | 31.6k | 29.5k | 1.47M | 22.5k | 1.3 GB |
| 32,768 | 33.1k | 30.4k | 1.52M | 29.4k | 2.2 GB |

   Against the box's ~2.1k policy steps/s end to end: **~14x at 16k
   worlds**, before any learner. The hooks, reset and obs cost ~35 % of
   bare MJWarp (2.36M physics steps/s at 32k, phase 1); the policy net
   5-7 %. v3_s0 fell 0 times in every row. Without the graph the Python
   launch loop (~0.4 s a policy step) dominates below ~16k worlds.
4. **Setup times:** boot to ssh 75 s (phase 1 156 s), apt 17 s, pip 36 s,
   Warp kernel compile ~1 min (cold vs warm `fly_policy`). The tarball's
   scp: seconds.

**The eval grid on CUDA, 2026-10-14** (A40 EU-SE-1, the phase 3 rental;
`analysis/gpu/phase3/results_a40/eval_grid.{log,json}`) [measured]: 640
worlds x 250 steps in 22.8 s (GeneralEnv's 20 episodes: 15.2 s). Falls
1/640 (one seed of `(0, -0.396, 0)`) vs 0/20. GeneralEnv's value lies
inside the 32 GPU seeds' [min, max] for 122 of 140 (command, metric)
pairs: head_err_med 17/20, vel_err_med 18, head_err_tail 19, vel_err_tail
18, t_head_s 18, v_ach 18, **drift_m 14**. One draw from the same
distribution lands outside 32 others' range with p = 2/33, so ~8.5 misses
were expected and there are 18 -- the engines agree closely, not exactly;
drift (CPU 2-5 % above the GPU's range on four commands) is the one to
watch. [reasoned on the p; the cause untested]

How it is built: `analysis/gpu/phase2/eval_grid.py` flies the 20-command grid both ways:
GeneralEnv through `eval_general._eval_episodes` itself (the eval
config, one episode a command, seeds 10_000 + k), and the GPU env with
every command x 32 seeds as worlds at once, each holding its command
(the reset options' `v_cmd` / `psi_cmd_rel`, set through `on_reset`), the
metrics rebuilt from device state under the same names. Per command:
GeneralEnv's value against the GPU seeds' median and range. Smoke-tested
on the Mac (2 seeds, 0.4 s each side): plumbing right, heading and speed
errors within ~1 deg / 0.01 m/s that early. Run:
`bash analysis/gpu/phase2/run_pod.sh eval`. The metrics
come from `aow_sim.eval_general`, torch-free since 2026-10-14.

**Where the time goes, measured 2026-10-14** (same rental,
`results_a40/profile.{log,jsonl}`) [measured]:

| worlds | env graph ms / policy step | bare MJWarp, this model, graph | env / bare | bare physics steps/s |
|---|---|---|---|---|
| 1024 | 83.4 | 80.4 | x1.04 | 0.64M |
| 4096 | 173.0 | 165.2 | x1.05 | 1.24M |
| 16384 | 552.8 | 527.0 | x1.05 | 1.55M |
| 32768 | 1075.6 | 1022.1 | x1.05 | 1.60M |

Of summed GPU time: MJWarp's step 96-97 %, the reset path ~2 %, our
per-substep hooks 0.6-1.2 %, the policy MLP <= 0.4 %. **So the ~35 % gap
was the model, not our code:** phase 1's rigid export runs 2.36M physics
steps/s, this model 1.60M. Our hooks and resets cost ~5 %, and fusing or
skipping them (below) is worth at most that. Inside `mjw.step` the top
kernels are the constraint solver's dense paths (`_update_gradient_JTCJ_dense`
39 ms of 202 at 4096, the linesearch, the Cholesky) and the primitive
narrowphase -- where any real speedup would be (solver options, contact
count), not in the env. (Summed per-call GPU time exceeds the graph's
wall: timing each call adds overhead, so "gaps" print negative.)

The plan before the measurement: the whole env is ~35 % below phase
1's bare 2.36M physics steps/s, but phase 1 timed a different (rigid
export) model, and the Mac's CPU puts MJWarp at 98 % of a step with every
hook under 2 %. `analysis/gpu/phase2/profile_env.py` times each call of a
real step on the GPU (Warp's activity timing: kernels, memcpy, memset, per
section: MJWarp step, each hook, sense ticks, the reset path, the policy
net), the same step as a graph (graph minus summed GPU time = launch gaps),
and bare physics on THIS model as a graph -- the like-for-like baseline.
`run_pod.sh profile` also takes an Nsight Systems trace if `nsys` exists.
Optimise nothing before it has run: candidates were the per-step reset
(~1/50 of the physics by arithmetic, `wp.capture_if` could skip it) and
fusing the per-substep hooks (100 launches a policy step into 50).

Found on the way:
- **MJWarp's constraint rows have no fixed order** (`_friction_dof` takes
  its row from `wp.atomic_add`). The Python gearbox hook reads the
  friction sign at a fixed row; the kernel finds the row by (type, id).
- **GeneralEnv's prev_action is one step stale, drive.py's is not.**
  `step()` builds the obs (line ~1098) before `self._prev_a = action`
  (~1175), so the obs that picks a_{k+1} carries a_{k-1} (recorded: exactly
  a_{k-1}, never a_k); `drive.py` feeds a_k. A train/replay mismatch on
  obs[12:15] for every general policy. Flagged as its own task; the port
  copies the env, for parity.
- **MJWarp's `forward` cannot be masked to some worlds.** The reset needs
  one (sensors for the AHRS prime and first obs); on a world that did not
  reset it would overwrite what the next substep's hooks read, which the
  Python env never does mid-episode. `ForwardGuard` saves and restores
  sensordata, qfrc_actuator, actuator_force, the efc rows and counts, and
  the warm start for those worlds. [built; the hybrid check will test it]
- **The first accelerometer reading after a reset differs by engine:** z
  12.0 (MJWarp) vs 10.4 m/s^2 (C) at the settled start pose, x -0.47 vs
  -1.31; the headset force by 0.2 N. MJWarp's extra contact points push
  differently in the first forward. Every kinematic sensor matches to 1e-9.
- **GeneralEnv's reset obs read MuJoCo truth** for roll, the rates and pitch:
  `_obs` read the AHRS before the odometry's call primed it, so the cache
  was empty on an episode's first read. One truth observation per episode.
  Fixed in GeneralEnv (prime first); found as the hybrid's one reset-obs
  difference (0.0027 rad/s on yaw rate).
- **Float32 encoder counts flip at half-count boundaries** now and then:
  6 of 998 ticks in one recording (0 in another) off by one count, 7.9 mm/s
  of v_lon through the 2-tap filter. The kind of error a real encoder makes.
- **The odometry's tick drifts in the Python env.** `SimOdometry` counts
  time in float (`_acc += 0.0004`), so 5 s gave 499 ticks, not 500. The
  port ticks every 25 substeps by integer count. [measured; effect not]
- **Float32 shaft angles vs the stick springs.** The input shafts reach
  ~106 rad in 5 s of v3_s0 (~1000 rad at 1.2 m/s for 15 s). float32's
  spacing there times k_in 250 / k_stick 1000 N m/rad is the 3 % above,
  and ~0.015-0.06 N m at 1000 rad against a 0.05 N m stick friction and a
  0.096 N m detent. MJWarp's own qpos has the same resolution, so no hook
  fix exists. Planned: a rebase kernel once per policy step shifting each
  input shaft by a multiple of 10*pi -- ring +da, rollers +2.4 x ring,
  hub +(da+db)/2 (needs na+nb even), detent phi by whole 7.5 deg periods --
  with the servo buffers, stick anchors and the odometry's last count
  shifted alike. The encoder counts add one more condition: a whole servo
  revolution behind the 3:1 belt (6*pi of input), so the step is 30*pi.
  Built as `rebase.py` (6*pi common, 60*pi differential; checked in the
  progress table).

### Phase 3 -- the PufferLib bridge

`ocean/aow_bike/` in a PufferLib fork: the embedding `.cu` (section 1), an
`aow_bike.ini`. Smoke: a random policy for 1M steps, rewards and resets
sane. Measure end-to-end steps/s.

**Built 2026-10-14, run the same day (results below the table).** No
embedded Python after all: phase 1's APIC route (T1-T4) carries the whole
env.

| piece | what |
|---|---|
| `src/aow_sim/gpu/export_graph.py` | builds GpuGeneralEnv for N worlds (training randomization), resets, captures ONE `step` with `apic=True`, `capture_save`s it with named buffers: `act_in` in; `obs`, `reward`, `terminated`, `truncated`, `ep_end`, `last_ret/len/track/fell/diff` out; plus `<stem>.json` (shapes, config hash, versions). Run on the training GPU: the modules are compiled for its arch |
| `env.py` `_episode_stats` | new kernel before the in-step reset: the finished episode's return, length, track, fall, difficulty (the reset would wipe them), for the trainer's logs |
| `analysis/gpu/phase3/check_graph.py` | reloads the `.wrp` as C will (`capture_load`, buffers by name through their pointers) and flies v3_s0 or random actions through it |
| `src/aow_sim/gpu/puffer/aow_bike.cu` | the PufferLib env: `dlopen`s Warp's `warp.so`, `wp_apic_load_graph`, binds the buffers by name (sizes checked against `vec.total_agents`); `puf_step` = actions D2D -> `cudaGraphLaunch` -> one kernel writing obs (bf16 or float), reward, terminal, and finished episodes into each world's `Log` |
| `src/aow_sim/gpu/puffer/aow_bike.ini` | `cudagraphs = -1` (phase 1: our graph cannot launch inside PufferLib's capture); v3's PPO numbers where PufferLib has the knob |
| `src/aow_sim/gpu/puffer/install.sh` | clones PufferLib at ea7f989 if absent, copies the env in, fills the ini's paths, `./build.sh aow_bike --cu` |
| `analysis/gpu/phase3/run_pod.sh` | export -> check (v3_s0 300 steps, random 100) -> build -> `./puffer train --headless` for 10 min -> warp cache |

Mac checks (CPU device, 8 worlds): the reloaded graph flies v3_s0 20
steps, 0 falls, 2.61 reward/step; random actions, 60 steps: 15 falls, each
reported once through `ep_end` with length 23.6, return -1.95 (the 50
penalty), track 0.76, and the world back at a fresh episode. [measured]

**Differences from the CPU stack in this smoke** (phase 4's list, all
deliberate here): PufferLib's MinGRU net (1 layer) not our MLP; Muon, its
only optimizer; bf16 unless built `--float`; **no observation
normalization** (SB3 ran VecNormalize; our obs are O(1) by construction,
but rates and velocities are not unit-scaled); **time limits are
terminals** (PufferLib 5.0 has no truncation signal, SB3 bootstraps them);
no smoothness terms; `puf_reset` only re-emits the saved reset state (fine
at startup, wrong if PufferLib resets mid-run).

**The next run (written 2026-10-15; nothing below has run yet).** The 1 h
run's setup with both collapse fixes: actions clipped in `aow_bike.cu`
(`ab_in`; syntax-checked, never compiled), `learning_rate 0.005` cosine to
0.001 over `total_timesteps 80M`, minibatch 8192 x replay 4 (128 gradient
steps an epoch), horizon 64, checkpoints every 25 epochs (~6.5M steps),
the shared curriculum (K 32, `export_graph`'s default), 4096 worlds. The
question it answers: with the collapse fixed, does it keep improving past
~30M, does difficulty climb, and do its checkpoints STAND STILL?
A40 secure, EU-SE-1, $0.59/h; ~65 min for 1 h of training, ~$0.65.

1. **Confirm with the user** GPU type and $/h. Then `create-pod`: image
   `runpod/pytorch:1.0.2-cu1281-torch280-ubuntu2404`, cloud SECURE, gpu
   `NVIDIA A40` x1, `dataCenterIds ["EU-SE-1"]` (CA-MTL-1 gave a dead
   CUDA driver once), disk 40, ports `["22/tcp"]`, env `PUBLIC_KEY` = the
   contents of `~/.ssh/id_ed25519.pub`. **Any other card: pass
   `gpu.minCudaVersion "13.0"`** -- Warp 1.18's wheel is built against
   CUDA 13 and falls back to the CPU on a 12.8 driver (a secure 5090,
   driver 570, 2026-10-16; `pod_setup.sh`'s assert caught it). The same
   day, two community 4090s on one host failed every CUDA call and one
   secure 5090 had no internet (apt hung), so check `torch.zeros(1).cuda()`
   AND `curl pypi.org` before copying anything. The 4090s' symptom --
   `nvidia-smi` fine, every CUDA call "unknown error" (999) -- is most
   often the host's `nvidia_uvm` module not loaded (JuliaGPU/CUDAdrv.jl#50),
   and otherwise a suspend/resume or power fault (openmm/openmm#1962); all
   need the host's owner. `ls /dev/nvidia*` in the failing pod tells the
   first apart: no `nvidia-uvm`. Not checked on 2026-10-16.
2. **Bundle** (meanwhile): `bash analysis/gpu/bundle.sh <scratch>/bundle.tgz`.
   Newest Warp cache (sm_86; only the env module recompiles, ~8 s):
   `analysis/gpu/phase3/results_a40_run2/warp-cache.tgz` (git-ignored).
3. **Wait for SSH:** `get-pod` until `runtime.ports` has the 22/tcp
   mapping, then in the BACKGROUND: `bash analysis/gpu/phase1/wait_ssh.sh
   <ip> <port> <createdAt epoch>` (~75 s from create). Then check the GPU
   works before copying anything: `ssh ... 'python3 -c "import torch;
   torch.zeros(1).cuda()"'` (setup also asserts it, but this costs nothing).
4. **Copy and launch** (`-o BatchMode=yes -p <port> root@<ip>`):
   `scp bundle.tgz warp-cache.tgz root@ip:/root/`, then
   `ssh ... 'mkdir -p ~/aow && cd ~/aow && tar xzf ~/bundle.tgz &&
   (TRAIN_S=3600 nohup bash analysis/gpu/phase3/pod_launch.sh > launch.log
   2>&1 < /dev/null &)'`.
5. **Watch,** all in the background. Since 10-17 `pod_launch.sh` starts
   `watch.py` itself: one row per checkpoint plus its alarms in
   `results/watch.log`. Stream it HERE in the background too, ending on
   ALL_DONE: `ssh ... 'cd ~/aow; until [ -f results/watch.log ]; do sleep
   2; done; tail -n +1 -f results/watch.log & T=$!; while [ ! -f
   results/ALL_DONE ]; do sleep 10; done; sleep 8; kill $T'`. Stop a run
   with `pkill -x puffer`, never `pkill -f "puffer train"`: -f matched the
   ssh command line that carried it and killed that session.
   - ~5 min: `results/timings.jsonl` (export compiled 1 / cached 42) and
     the last training panel -- SPS ~24k, kl small.
   - halfway (user's habit): `scp` `results/train.log`, then
     `python analysis/gpu/phase3/train_curve.py train.log`; list
     `~/puffer/checkpoints/aow_bike/*/` and pull them. **If kl stays
     >> 0.1 or entropy falls under ~0.2, say so; if the return goes
     astronomic or kl/clipfrac drop to 0 with entropy pinned at 4.257,
     it is NaN -- stop and report** (2026-10-15's run spent 25 min dead).
   - the end: ONE ssh blocking on `results/ALL_DONE` (not `DONE`: run_pod.sh
     writes that earlier).
6. **Pull** `rsync -a -e "ssh -p <port>" root@ip:aow/results/
   analysis/gpu/phase3/results_<tag>/` plus `aow/graphs/aow_bike_4096.json`
   and `puffer/config/aow_bike.ini` into its `checkpoints/`. Then
   **TERMINATE** (`delete-pod`) and confirm with `list-pods`.
7. **Judge it here:** `python analysis/gpu/phase3/train_curve.py
   .../train.log`; convert two or three checkpoints (`python -m
   aow_sim.gpu.export_policy CKPT.bin --graph-meta .../aow_bike_4096.json
   --ini .../aow_bike.ini --name general_puffer_<tag>_<steps>`, which also
   runs the 20-command eval into the yaml); `python
   analysis/gpu/phase3/standstill.py general_rl_v3_s0 <those names>`. Then
   teleop. Write the numbers into this section, `[measured]`.

What would say it worked: no NaN; kl mostly < ~0.1; fall_rate well under
0.3 by 30-40M and still falling; difficulty clearly above 0.03; a
checkpoint whose standstill max |v| is near v3_s0's 0.10-0.17 m/s. If it
learns but never stands still, the next levers are reward-side (phase 4),
not the optimizer.

**It ran, 2026-10-15** (A40 EU-SE-1, ~70 min, ~$0.69;
`analysis/gpu/phase3/results_a40_next/`, rows from
`analysis/gpu/phase3/watch.py`, which now runs on the pod and prints one
row per checkpoint) [measured]:

| epoch | steps | len | fall | difficulty | entropy | kl | clipfrac |
|---|---|---|---|---|---|---|---|
| 25 | 6.6M | 63 | 1.000 | 0.000 | 2.67 | 0.056 | 0.34 |
| 50 | 13.1M | 609 | 0.290 | 0.138 | 1.46 | 0.173 | 0.42 |
| 75 | 19.7M | 523 | 0.621 | 1.000 | 1.57 | 0.475 | 0.64 |
| 125 | 32.8M | 663 | 0.273 | 1.000 | 2.82 | 0.352 | 0.60 |
| 200 | 52.4M | 649 | 0.289 | 1.000 | 3.58 | 0.273 | 0.59 |
| 287 | 75.2M | 694 | 0.194 | 1.000 | 4.34 | 0.303 | 0.61 |

**No NaN** past the 1 h run's 34.6M: the action clip held. The shared
curriculum went 0.14 -> 1.0 in ~6.5M steps (25 epochs, ~3200 gradient
steps; SB3 takes ~24k gradient steps over v3's ramp), and its credit is
~3M steps stale at 4096 worlds -- a good episode counts at its end, at
the difficulty taken 15 s earlier [reasoned]. kl never came under 0.1.
Entropy rose from epoch 75 on, all of it the STEER channel's logstd
(std 0.72 at 26M -> 1.66 at 46M; hub and diff ~0.5): past std 1 most
steer samples clip to +-1, so widening it costs little while `ent_coef
0.005` pays for it [reasoned].

**And it does not stand still -- it floors the hub.** Converted 32.8M,
45.9M, 72.1M (`general_puffer_next_<N>M`):

| policy | eval survive | track | vel_err | hold drift | standstill max \|v\| | mean \|hub\| |
|---|---|---|---|---|---|---|
| general_rl_v3_s0 | 1.00 | 0.72 | 0.09 | 0.08 m | 0.10-0.16 m/s | 0.02-0.03 |
| next_32M | 1.00 | 0.29 | 0.96 | 6.0 m | 1.37-1.43 | 0.96-0.97 |
| next_45M | 0.85 | 0.35 | 1.02 | 5.2 m | 1.20-1.30 | 0.94-0.96 |
| next_72M | 1.00 | 0.40 | 0.89 | 5.4 m | 1.28-1.31 | 0.96-0.99 |

(Eval columns: each yaml's 20-command grid, same code as v3_s0's; standstill:
`standstill.py`, 4 seeds, zero command.) Every checkpoint drives at
~1.3 m/s from 0.1 s after reset, whatever the command; it tracks HEADING
(72M head_err 10 deg) and ignores SPEED. So "survives 15 s" at
difficulty 1 was earned by moving -- the training panels' track ~0.43
said the same. Not established which of phase 4's differences causes it
(no `bound` / `temporal` loss terms, so the mean is free past the clip;
no obs normalization; time limits as terminals).

The 13.1M checkpoint (difficulty 0.14 then) DID hold still: standstill
max |v| 0.09-0.14 m/s, mean |hub| 0.01-0.03, as v3_s0; eval survive 0.40.
So hold was learned, then lost when the curriculum jumped to 1.0 (hold
share ~30% -> 10%, speeds to 1.2 m/s, moving turns open) [measured]. The
command sampler is not it: reverse is 50.1% of moving commands at every
difficulty, Warp and numpy alike (the phase 2 parity check compared |v|,
never the sign; checked now). The bound term is NOT the lever: on the CPU
stack it moved the saturation and nothing else
(`policy-smoothness-losses.md` §5.1); temporal was.

**PufferLib clamps the reward it trains on to [-1, 1]** [read:
`src/pufferl.cu:1487` @ ea7f989, unconditional, before the advantages].
Our step reward reaches 3.0 (w_vel 1.5 + w_head 1.0 + w_alive 0.5), so
once the heading error is under ~21 deg (r_head >= 0.5) heading + alive
alone hit the clamp and **velocity tracking earned nothing**; a fall's -50
was -1. Every PufferLib run to date trained that way, and the next run's
checkpoints did exactly what that rewards: heading to 10 deg, speed
ignored [reasoned -- it fits, it is not isolated]. SB3 never saw it
(VecNormalize scales the reward, `norm_reward=True`). Nor does PufferLib
normalise advantages (`puff_advantage`, `ppo_loss_compute` use them raw),
so `ent_coef` and any loss weight meant whatever the reward scale made
them; SB3 normalises per minibatch.

**The run after (written 2026-10-15; it ran -- below).** Same runbook, four
changes, all on by default -- one run cannot separate them. Three are in
PufferLib's trainer, via `src/aow_sim/gpu/puffer/puffer_aow.patch`
(`install.sh` resets `src/algo.cu` and `src/pufferl.cu` and applies it),
each behind an `AOW_*` switch that `aow_bike.cu` defines -- so only this
env's build gets them. The patched trainer has NOT been compiled (no CUDA
here): the pod's build step is its first nvcc.

- **Reward scale** (`AOW_REWARD_SCALE 0.02`, `ab_out`): the reward
  PufferLib trains on x 0.02, a fall ~-1, nothing clamped, every ratio
  kept. The dashboard's `episode_return` is the env's own `last_ret`,
  unscaled, so it stays comparable with the runs above.
- **Advantages normalised per minibatch** (`AOW_NORM_ADV`, kernel
  `aow_norm_adv` after `puff_advantage`): (A - mean) / (std + 1e-8), as SB3.
- **SmoothPPO's temporal term** (`AOW_SMOOTH_TEMPORAL 0.04`):
  `smooth_temporal.h`, lam (mu_{t+1} - mu_t)^2 per pair inside one episode,
  on the mean before the clip, called from `ppo_loss_compute`. 0.04 is the
  CPU weight -- chosen, not swept: one gradient-ratio point on the CPU
  stack's trained MLP actor put the term at ~1/4 of PPO's policy gradient
  (`policy-smoothness-losses.md` §4); not re-measured on PufferLib's MinGRU
  or from scratch. `analysis/gpu/phase3/check_smooth.py`: loss against a
  numpy reference 1.6e-7, gradient against finite differences 5e-10, with
  terminals.
- **Its gradient, logged** (`AOW_GRAD_LOG`): one `AOW_GRAD pg .. smooth ..
  ratio ..` line per epoch -- the two terms' gradient norms on the policy
  mean; `train_curve.py` / `watch.py` show them as `pg_grad`, `s/pg`. The
  CPU's quarter was in PARAMETER space, so the ratio here is the same idea
  at the network's output, not the same number.
- **A paced curriculum**: `export_graph --curriculum-lead 0.1` (the
  default; 0 = the rule above). A good episode counts only if it was
  played within 0.1 of the shared value as of the last step, so the value
  rises at most ~0.1 per episode length of training (~3M steps at 4096
  worlds): 0 -> 1 in >= ~30M steps rather than 0.14 -> 1 in 6.5M
  [reasoned]. Checked on the Mac (Warp CPU): the cutoff credits 0.40 and
  refuses 0.399 at a shared 0.50; a 16-world smoke run is identical with
  and without it while it does not bind.

Judge it as above, plus: does difficulty climb steadily instead of
jumping; do checkpoints past difficulty ~0.5 still hold still and reverse
(`standstill.py`; the eval yaml's `speed_ratio_rev`); `s/pg` in the first
epochs (if far from ~0.25, the weight is the next thing to move). If the
build fails, the patch is the suspect: `results/build.log`.

**It worked, 2026-10-15** (A40 EU-SE-1, ~72 min, ~$0.71;
`analysis/gpu/phase3/results_a40_fixes/`) [measured]. The patched trainer
built on its first nvcc (45 s); 72.6M steps in the hour, 19-25k SPS.

| epoch | steps | len | fall | track | difficulty | entropy | kl | s/pg |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.26M | 21 | 1.000 | 0.73 | 0.000 | 4.24 | 0.008 | 0.016 |
| 25 | 6.6M | 375 | 0.867 | 0.76 | 0.000 | 3.31 | 0.112 | 0.024 |
| 50 | 13.1M | 734 | 0.048 | 0.87 | 0.175 | 2.74 | 0.032 | 0.013 |
| 75 | 19.7M | 705 | 0.162 | 0.83 | 0.404 | 2.34 | 0.045 | 0.010 |
| 100 | 26.2M | 684 | 0.227 | 0.79 | 0.608 | 2.01 | 0.043 | 0.008 |
| 125 | 32.8M | 646 | 0.295 | 0.76 | 0.818 | 1.85 | 0.068 | 0.007 |
| 150 | 39.3M | 694 | 0.212 | 0.74 | 1.000 | 1.63 | 0.034 | 0.007 |
| 200 | 52.4M | 744 | 0.030 | 0.77 | 1.000 | 1.42 | 0.029 | 0.006 |
| 277 | 72.6M | 743 | 0.030 | 0.80 | 1.000 | 1.13 | 0.031 | -- |

Difficulty climbed ~0.2 per 6.5M steps as the lead allows, reached 1.0
at ~39M, and the fall rate turned DOWN there (the previous run's jump
sent it to 0.62). kl under 0.1 from epoch 50 on; entropy fell all run.
**The temporal term stayed small: s/pg 1.6 % at the start, ~0.5 % by the
end** -- at the policy mean, not the CPU's parameter-space quarter, so
not the same number, but whatever smoothing this run has is mostly not
from it. Raising the weight is the open lever.

Converted checkpoints: every one from 13.1M on holds still (standstill
max |v| 0.09-0.18 m/s, end 0.02-0.12 m; v3_s0 0.10-0.16, 0.05-0.10).
Each yaml's eval (same code, same grid as v3_s0's):

| policy | survive | track | vel_err | head_err | fwd | rev | turn_big survive / tail |
|---|---|---|---|---|---|---|---|
| general_rl_v3_s0 | 1.00 | 0.723 | 0.091 | 47.7 | 1.05 | 0.94 | 1.0 / 164 deg |
| fixes_19M | 0.70 | 0.599 | 0.246 | 43.1 | 0.96 | 1.02 | 0.0 / 180 deg |
| fixes_39M | 0.95 | 0.728 | 0.115 | 13.2 | 0.96 | 1.02 | 0.83 / 14 deg |
| fixes_52M | 1.00 | 0.784 | 0.080 | 5.6 | 1.00 | 1.07 | 1.0 / 4 deg |
| fixes_72M | 1.00 | 0.813 | 0.093 | 5.6 | 0.99 | 1.14 | 1.0 / 6 deg |

The common grid (`per_command.py --summary`, one process, identical
seeds), at the training sensor (`--ahrs tm151_filter --ahrs-tau 2.0`):

| policy | turns fwd | straight fwd | hold v | fell |
|---|---|---|---|---|
| general_rl_v3_s0 | 5/9 | 2/2 | -0.01 | 0 |
| general_rl_v3_s1 | 8/9 | 2/2 | -0.00 | 0 |
| general_rl_v3_s2 | 4/9 | 2/2 | -0.06 | 0 |
| general_puffer_fixes_52M | 7/9 | 2/2 | -0.01 | 0 |
| general_puffer_fixes_72M | 7/9 | 2/2 | -0.03 | 0 |

At `per_command.py`'s default `--ahrs tm151` (not what either stack
trains on): v3_s0 1 fall, v3_s1 2, both GPU checkpoints 0.

So ~55 min of A40 training to 52M reaches what a v3 seed reaches in
~3 h on the box (20M steps), on this grid. ONE GPU seed against three CPU
seeds; four changes at once, so which of them mattered is not separable
from this run.

**Chatter** (`analysis/policy/chatter.py`, one grid, identical seeds) [measured]:

| | v3_s0 | v3_s1 | fixes_52M | fixes_72M |
|---|---|---|---|---|
| score | 0.728 | 0.818 | 0.851 | 0.863 |
| per-step action change, total | 0.035 | 0.048 | 0.008 | 0.008 |
| steer sign flips /s | 14.9 | 12.2 | 9.7 | 8.1 |
| steps pinned to a bound | 1.0 % | 4.5 % | 0.4 % | 0.3 % |
| tread jitter 2-8 Hz, mm/s | 43.9 | 52.2 | 29.6 | 33.3 |
| hold drift (one episode) | 0.18 m | 0.07 m | 0.20 m | 0.39 m |
| crab left / right | 0.96 / 0.51 | 0.71 / 0.56 | 0.41 / 0.59 | 0.45 / 0.63 |

4-6x less per-step action change; the mean never past the clip. NOT
attributable to the temporal term (~0.5 % of PPO's gradient): the MinGRU,
Muon, the batch size and the advantage normalisation are all candidates.
Crab was never trained (`v_lat_frac 0`). Teleop (user, 2026-10-15): "looks
pretty smooth". Not yet run: the standing-endurance `prospective` test.

**What the MinGRU memory does** (`analysis/policy/mingru_memory.py`, the eval
grid, 5000 steps) [measured], beside the CPU arm's first seed
(`general_rl_v3_mingru_s0`, `config/rl/rl_general_v3_mingru.yaml`):

| | CPU mingru_s0 | GPU fixes_52M |
|---|---|---|
| time constant at each unit's mean gate | median 35 ms, max 80 ms | median 35 ms, max 0.37 s |
| units slower than 0.5 s in their slowest 5 % of steps | 86 of 128 (up to 23 s) | 9 (one ~558 s: latched) |
| head input coming through h | 76 % | 56 % |
| grid as trained: surv / track / head_err | 1.00 / 0.701 / 23.6 | 1.00 / 0.784 / 5.6 |
| NO GATE (h = h~, memory cut): same | 1.00 / 0.664 / 22.3 | 0.85 / 0.622 / 27.3 |

Mostly a ~2-step low-pass (dt 20 ms) -- not the slow estimation (mass,
friction) the arm was set up to look for -- but with gates that CLOSE in
some states and then hold for seconds. Which states (the script's LATCHING
block, plus a per-command table of the latching units):

- **GPU 52M: registers set at reset.** 7 of its 9 latching units are
  latched 60-100 % of EVERY episode, hold included: their h is written in
  the first steps after the reset and then held (unit 20: +1.1 -> +1.3
  over the hold episode). They open mainly on the 170-180 deg commands
  (latched ~60-80 % there). Unit 102 latches ~100 % on the forward commands
  and stays open on hold, spin and reverse: a driving-forward flag. A
  bogus first observation after a reset is therefore written into units
  that keep it for the episode -- the teleop reset report's likely path
  [reasoned].
- **CPU mingru_s0: direction modes.** 5 units latch 80-100 % on the
  forward commands at any heading and stay open on hold and reverse; unit
  48 latches ~100 % on reverse, including `0.8 m/s, 180 deg`, which it
  drives backwards. Latching rises with motion: 0.2 units on the hold, 12
  at speed, 18.6 at yaw rate > 1 rad/s, 23 at |roll| > 0.1 rad.
Cutting it costs the GPU policy its heading and a fifth of its survival,
the CPU seed little (drift 0.06 -> 0.30 m on the hold). One seed each.

**The pod run, 2026-10-14** (A40 secure EU-SE-1, $0.59/h, ~21 min,
~$0.21; results in `analysis/gpu/phase3/results_a40/`) [measured]:

| step | result | wall |
|---|---|---|
| export, 4096 worlds, on `cuda:0` | `.wrp` 188 MB (~46 KB/world, as phase 1), modules 9.2 MB | 60 s |
| check_graph, v3_s0, 300 steps | 0 falls, 0 episode ends, 2.886 reward/step | 56 s |
| check_graph, random, 100 steps | 16910 falls, each reported once; length 21.4, return -6.57, track 0.77 | 19 s |
| `install.sh` (nvcc build of `aow_bike.cu`) | built first try, `.wrp` loaded, 4096 worlds | 70 s |
| `./puffer train --headless` | **24.7-24.8k SPS** steady, env 99 % of the time; 14.4M steps, 55 epochs | 600 s (timeout) |

**It runs; it did not learn.** Over 55 epochs episode length 20.6 ->
21.9 (random: 21.4), return -20.4 -> -15.4, fall_rate 1.000 throughout,
entropy 4.256 -> 4.145 (std ~1 on all three actions: 3 x 1.42),
`kl` and `clipfrac` 0.000, train 26 ms an epoch. Why [read, `src/pufferl.cu`
@ ea7f989 :1504]: `total_minibatches = replay_ratio * batch /
minibatch_size` = 1.0 x 262144 / 32768 = **8 gradient steps an epoch**,
440 in the run, where SB3 (10 epochs x 16 minibatches per 16384) takes
~140k over the same 14.4M samples. And `learning_rate = 0.0003` is v3's
Adam value; PufferLib's default for Muon is 0.015 (`config/default.ini`;
its envs' inis use 0.001-0.015).

**Second run, same day: it learns** (A40 EU-SE-1, ~22 min, ~$0.21;
`analysis/gpu/phase3/results_a40_run2/`) [measured]. `aow_bike.ini` now
has `learning_rate = 0.015`, `minibatch_size = 8192`, `replay_ratio =
4.0` (128 gradient steps an epoch, 230 ms of each ~11 s epoch); 15 min of
training so the 20M budget (and the lr anneal) completes:

| epoch | steps | SPS | ep. length | return | fall_rate | entropy |
|---|---|---|---|---|---|---|
| 1 | 0.26M | 12.2k | 20.6 | -20 | 1.000 | 4.10 |
| 17 | 4.5M | 24.1k | 31.7 | 11 | 1.000 | 3.02 |
| 33 | 8.7M | 23.5k | 105 | 126 | 1.000 | 2.06 |
| 49 | 12.8M | 22.7k | 402 | 622 | 0.893 | 1.22 |
| 57 | 14.9M | 22.1k | 565 | 972 | 0.547 | 0.96 |
| 75 | 19.7M | 21.2k | 668 | 1094 | 0.284 | 0.59 |

Max length is 750 (15 s). These are training-time episodes under the
training randomization, with curriculum difficulty ~0.013 throughout --
NOT comparable to v3's eval grid yet (phase 4: export the policy and run
`per_command.py`). SPS falls 24.6k -> 21.2k as episodes lengthen [cause
not looked at]. The log's last panels ("To go 0ms", difficulty 0.000) read
length ~50 and fall_rate 1.000: PufferLib's end-of-run evaluation, which
by its look averages only episodes that ended inside a short window -- so
falls only [reasoned, not checked in the source]. Read the training panels.

Against v3_s0's own curve (the box's tensorboard) [measured]: ep_len_mean
583 at 0.7M, 666 at 5M, 698 at 20M; difficulty 0.17 at 0.7M, 1.0 by 5M.
So run 2 at 20M is about where SB3 was at 0.4-0.7M: ~28x the samples,
~2.5x the wall clock (6 vs 14 min), at a difficulty SB3 passed at ~0.3M.
Gradient steps per wall-second are about equal (SB3 ~20, this ~12): the
11x of data goes into bigger minibatches, not more updates [reasoned].

**The curriculum was per world, so it could not move.** GeneralEnv's rule
(+0.02 per good episode, in that env) needs nworld x 50 good episodes to
reach 1.0: 1600 for 32 envs (~1.2M steps at best), ~205k at 4096 worlds
(>= 154M steps). At 20M steps no world could pass ~0.13. Fixed
2026-10-15: `GpuGeneralEnv(shared_curriculum_envs=K)` keeps ONE difficulty,
+0.02/K per good episode in any world, taken by each world at its reset --
the mean moves per good episode as K per-env curricula's mean does, at any
world count; `export_graph` uses K = 32 (v3's SB3 run; `--curriculum-envs
0` = per world, the default of the env itself so the parity checks keep
GeneralEnv's rule). Mac check, 16 worlds of 0.6 s episodes under v3_s0:
per world unchanged (each = good x 0.02); shared, 31 good -> 0.01937 =
31 x 0.02/32 exactly, every resetting world on it, `last_diff` the value
the episode ran at. [measured, CPU device] Lost: SB3's spread between
envs (its difficulty_min/max); drawing each world's difficulty below the
shared one at reset would restore some, not built.

**The 1 h run, 2026-10-15: collapsed at ~33M** (A40 EU-SE-1, ~39 min,
~$0.38, stopped early; `analysis/gpu/phase3/results_a40_1h/`: the log to
44M and checkpoints at 13M, 26M, 39M) [measured]. Run 2's update settings,
the shared curriculum (K 32), lr 0.015 cosine to a 0.2 floor over 80M.

| epoch | steps | ep. length | fall_rate | difficulty | entropy | kl | clipfrac |
|---|---|---|---|---|---|---|---|
| 31 | 8.1M | 82 | 1.000 | 0.000 | 2.24 | 0.10 | 0.36 |
| 61 | 16.0M | 347 | 0.922 | 0.001 | 0.75 | 0.95 | 0.62 |
| 101 | 26.5M | 608 | 0.390 | 0.020 | 0.16 | 1.04 | 0.75 |
| 121 | 31.7M | 606 | 0.398 | 0.032 | 0.51 | 0.70 | 0.69 |
| 131 | 34.3M | 20 | 1.000 | 0.032 | 4.55 | 1.1e6 | 1.00 |

Return -91M at epoch 131; the 39M checkpoint is all NaN; after it the net
emits nothing (kl 0, entropy fixed, random-action episode lengths) while
the GPU stays busy. Two causes:
- **Unclipped actions (a port bug).** PufferLib passes the raw Gaussian
  sample (clamped at +-1e6); SB3 clips to [-1, 1] before `env.step`, and
  GeneralEnv relies on it: the physics clips again, but the effort and
  smoothness penalties and obs[12:15] read the action as given. Fixed in
  `aow_bike.cu` (`ab_in`: clip, then into the graph). The smoothness term
  on raw sampled noise also rewards a small std, which may be why entropy
  fell to 0.12 [reasoned].
- **lr too hot**: kl 0.7-2.5, clipfrac 0.6-0.75 from ~10M steps on. Now
  `learning_rate = 0.005` (cosine to 0.001), checkpoints every 25 epochs.
  Untested.

**Standing still** (user, teleop: `general_puffer_1h_13M` "launches off
from standstill"). Reproduced without teleop, zero command from reset, 6
seeds, 5 s [measured]: v3_s0 max |v| 0.10-0.17 m/s, never above 0.3;
puffer 13M 0.30-0.58 m/s, moving off at 1.1-1.8 s, up to 1.3 m; puffer 26M
0.64-0.70 m/s from 0.1 s, rocking (0.2-0.6 m net). The same in the GPU env
it trained in (zero-command worlds 0.38-0.45 m/s; bf16-rounded obs change
nothing), so it is the policy, not teleop, reset or the conversion. Rolling
is easier to balance than standing; these policies bought survival with
motion [reasoned]. Their eval: 45 % survive, 1 of 9 cruises.
(`rl_general_v3.yaml`'s "difficulty 0 is a PURE HOLD stage" is stale:
GeneralEnv commands 0 in 39 % of resets there, the rest straight at median
0.16, max 0.30 m/s -- both stacks alike.)

**Replaying a PufferLib policy:** `python -m aow_sim.gpu.export_policy
CKPT.bin --graph-meta G.json --ini aow_bike.ini --name NAME` writes a move
that replays as `MinGRUPolicy` -- eval, per_command.py, teleop, the Pi --
with the trainer's yaml (`train_general_rl.move_doc`) and metrics block.
The `.bin` layout, read from ea7f989's source: flat fp32, tensors on
16-byte boundaries (8 elements in bf16): encoder (H, obs), decoder (A+1,
H; row A is the value), logstd, MinGRU (3H, H) [c|g|p]; no biases. The
dump covers `total_elems`, which excludes the padding, so the last tensor
loses as many trailing weights as were padded (5 of 49152 here; written as
0, counted in the yaml). Checked: a synthetic round trip against
PufferLib's forward written out (5e-8), and every real checkpoint is
exactly 51843 floats = 207372 bytes.

**For a Docker image, both caches tested on that rental** [measured]:
- Warp's kernel cache from the first run (`warp-cache.tgz`, sm_86),
  untarred to `~/.cache` before anything compiled: 0 modules compiled,
  43 cached (first run: 43 / 0); export 60 s -> 6.5 s. The cache saved
  after the run was byte-identical, so check_v3s0's ~55 s is flight, not
  compile.
- The built `puffer` binary (1.7 MB) copied into a FRESH unbuilt
  checkout at ea7f989 with only `config/aow_bike.ini`, the built tree
  moved aside: loads the graph, trains at 24.7k SPS. It reads the ini
  (`src/ini.h:510`, `config/<env>.ini` from the cwd) and the graph path at
  run time and links CUDA's shared libraries from the image, so one build
  serves any world count, graph or hyperparameters. Rebuild only for a new
  `aow_bike.cu` or PufferLib commit. `build.sh` compiles with
  `-arch=${NVCC_ARCH:-native}`: set NVCC_ARCH for an image meant for
  other cards (an sm_86 binary on another arch: untested).
- Left per rental after both: apt + pip ~51 s, the PufferLib build ~1 min
  (now avoidable), boot ~75 s.

**A faulty host first** (CA-MTL-1, same day): `nvidia-smi` listed the A40,
but `cuInit` returned 999 (Warp and torch both), and Warp fell back to the
CPU WITHOUT an error -- the export ran there (a 2.8 GB graph) for 6 min at
0 % GPU before it was noticed. Terminated after ~9 min (~$0.09).
`pod_setup.sh`'s `warp_init` now asserts `wp.is_cuda_available()`, so
such a pod fails in its first minute.

**Choice B (not a stop gate):** past gate A the env is on the GPU either
way, so the only question here is which learner sits on it -- PufferLib
through the bridge, or the torch fallback. User (2026-10-14): even ~10 %
from PufferLib's trainer is worth having on a rented GPU. Take PufferLib if
the bridge works; fall back to torch if it fights PufferLib's graphs or
streams for long.

**A second seed, 2026-10-16** (A40 EU-SE-1, ~70 min; env seed 1,
PufferLib seed 43, everything else as "It worked";
`analysis/gpu/phase3/results_a40_seed1/`) [measured]. The curve tracked
seed 0's within a few hundredths at every checkpoint (difficulty 1.0 by
~39M, fall rate 0.03 at 70M). On the common grid (`per_command.py
--ahrs tm151_filter --ahrs-tau 2.0`) its 52M and 72M are 7/9 turns
forward, 2/2 straight, 0 falls -- seed 0's numbers -- and `v_ach` has
the SAME SIGN on all 20 commands (both reverse through `rev 0.5 +-90`,
which v3_s1 resolves forward). Chatter: score 0.863 / 0.875 (72M is the
best so far), per-step action change 0.008-0.009, big turns 1.1-1.2 s
to heading. What two GPU seeds share is the DIRECTION choice on every
command (three CPU seeds gave three); the details differ -- seed 1 turns
1.5-2x faster than seed 0 (1.18 / 1.08 s against 2.08 / 1.53), more
evenly (asymmetry 0.21 against 0.25 / 0.38), crab bias mirrored. Turn
FOOTPRINT, read off per_command's drift plots (72M vs 72M vs v3_s1, by
eye, not tabulated): in-place spins (+-170, +180) peak ~0.6-0.75 m from
the start for seed 1, ~0.6 seed 0, ~0.2-0.35 v3_s1 -- all creep forward
while rotating, and seeds 0 and 1 slide to opposite sides. On the moving
`fwd 0.8 +180` the final lateral offset (~ U-turn width) is ~1.2 m seed
0, ~0.8 seed 1, ~0.45 v3_s1, and its sign says seed 1 turned left (the
commanded side) where the other two went right. That reading is the
measure, not a stand-in: after a U-turn the bike runs back parallel,
offset by the turn's diameter, and nothing pulls it back (the reward
tracks velocity and heading, not position), so `drift_lat` on a moving
180 IS the turn width, sign = side (- = left). Clean on `fwd 0.8 +180`
only -- the crab+180 commands add their sideways motion -- and any
sideways creep on the straight part adds to it. `drift_m`/`drift_max` on
moving commands are mostly distance driven (18 m on `fwd 1.2`); read
`drift_lat` there instead.
Why is reasoned, not tested (each update averages 4096 worlds' commands
instead of 32 envs'), and two seeds are evidence, not proof. Neither run
is repeatable to the bit: the shared curriculum's float atomics and
MJWarp's atomically numbered constraint rows vary with scheduling
[reasoned from the code]; even two exports of one seed differ at step 1.

**Which card, 2026-10-16** (`analysis/gpu/phase3/bench_card.sh`; results
in `results_{a40,4090,5090}_bench/`) [measured]. Env only, the trained
52M MinGRU driving (`bench_env.py`, now with a Warp MinGRU), policy
steps/s:

| card | 1k | 4k | 16k | 32k | 64k |
|---|---|---|---|---|---|
| A40 | 12.3k | 23.2k | 27.7k | 28.3k | 28.4k |
| RTX 4090 | 21.5k | 49.4k | 66.1k | 62.9k | 61.8k |
| RTX 5090 | 22.9k* | 57.2k* | 93.2k* | 93.5k* | 92.4k* |

(* v3_s0 driving; 52M loads the physics within 3 % of it on every card.)
At 1k worlds the 4090 and 5090 are equal -- both ~3.1 GHz, and there the
step is the per-substep latency floor; at 16k+ the 5090's extra cores
count. Real `puffer train`, median panel after warm-up, and its VRAM:

| card ($/h secure) | 4k | 16k | 32k | 64k | 72M steps at the best |
|---|---|---|---|---|---|
| A40 (0.59) | 19.9k, 3.7 GB | 31.4k, 12.3 GB | 33.0k, 24.0 GB | no (memory) | 38 min, $0.38 (16k) |
| 4090 (0.89) | 44.6k, 0.9 GB | 69.9k, 1.7 GB | 71.5k, 2.9 GB | 71.3k, 5.1 GB | 17 min, $0.25 (32k) |
| 5090 (1.19) | 50.6k, 1.0 GB | 94.6k, 1.8 GB | 102.8k, 3.0 GB | 104.4k, 5.2 GB | 12 min, $0.23 (64k) |

(A40 rows on the old export, before the memory fix below.) The 5090
drew 395 W mean, 401 max, training at 16k (575 W limit;
`results_5090_bench/power_16384.csv`): ~4 mJ a step. The 4090 was not
measured. On secure cloud the 4090 and 5090 cost the same per run and
the 5090 is ~30 % sooner; a working community 4090 ($0.34) would be
~$0.10 a run. The learner is 1 % of a step at 4k, 5-6 % at 16k+
(PufferLib's own timer), so a card is chosen for MJWarp: FP32 cores and
clock, not tensor cores.

**The export's GPU memory, fixed 2026-10-16** (`gpu/scratch.py`;
`results_a40_bench/mem_probe_16384.txt`). Training used 0.72 MB a world
-- 12.3 GB at 16k, and 64k did not load on the 44 GB A40. Not the env
(1.25 GB at 16k) and not PufferLib: Warp's APIC capture, which keeps
every MJWarp call's working memory for good where an ordinary capture
reuses it (one step: 2341 allocations, 10.3 GB at 16k). MjwScratch
serves the i-th allocation of every MJWarp call from slot i; 46 slots,
218 MB at 16k. Loaded as PufferLib loads it, on the 4090: 1.44 / 2.41 /
4.24 GB at 16k / 32k / 64k. Proof it changes nothing: NaN in every slot
after every MJWarp call (`check_scratch.py --poison`, 200 random-action
steps x 512 worlds, ~4450 falls) -- no non-finite value on the 4090 or
the 5090. Comparing exports cannot show it; they differ at step 1 anyway.

**Setup time** (fresh 5090, `timings.jsonl`): ~4 min before the first
step -- boot ~30 s, apt 20-44, pip mujoco-warp 17, pip mjx+jax 38, the
first Warp compile on a new architecture ~55, PufferLib clone + build
43. Now: jax only with `WITH_MJX=1` (nothing imports it); PufferLib
fetched as one commit (`puffer/fetch.sh`: 8.0 s against 46.7 for the
928 MB clone, measured); apt and that fetch run during pip; the build
during the graph checks; one Warp cache for sm_86/89/120,
`analysis/gpu/warp-cache.tgz` (git-ignored, 25 MB; copy it up as
`~/warp-cache.tgz`). Expected ~1.5 min -- not yet measured. A network
volume was considered and dropped: it would tie every run to one data
center's stock (EUR-IS-2, where both cards worked, has no volumes).

**The next run: the style bit (2026-10-17; scripts changed, nothing run).**
`CONFIG=config/rl/rl_general_v3_style.yaml`, 4096 worlds, like every policy trained
so far: on a 5090 (50.6k steps/s at 4k) ~26 min for the ini's 80M steps
plus ~4 min setup. No plain v3 control run (the user's call: the existing
policies drove on the updated plant).
- **`CONFIG=`** passes through `pod_launch.sh` -> `run_pod.sh` ->
  `export_graph --config`. It was hard-wired to `rl_general_v3.yaml`.
  One launch is one config. Both write `results/` and the same graph stem,
  so a second launch on the same pod needs `results/` moved aside first;
  that is not scripted.
  Since 10-17 that config trains the Pi-style steer rate
  (`steer_rate_encoder: counts`), whose Warp kernel has run only on Warp's
  CPU device here. `config/rl/rl_general_v3_style.yaml` adds the style bit
  (`wishlist.md`, "pivots").
- **Obs width.** The style config's graph is 20 wide. `install.sh` writes
  `OBS_SIZE` into `aow_bike.cu` from the graph's json. `aow_bike.cu` now
  refuses an exact mismatch: its buffer check only tested "at least", so a
  20-wide graph under 17 would have trained without ever showing the bit.
  `export_policy` already reads the width from the json. The weights the
  `.bin` does not hold stay at 5 at obs 20.
- **Checks.** `check_graph` flies v3_s0 (17 wide) on a 20-wide graph by
  reading the first 17 columns. `general_rl_v3_s0` moved to
  `moves/archive/` on 10-16, and `bundle.sh` plus nine scripts named it by
  path, so the bundle would have failed. They point there now. The bundle
  also swept in `analysis/gpu/warp-cache.tgz` (25 MB), which is copied up
  separately anyway. It is excluded now: 1.4 MB, not 26.

**It ran, 2026-10-17** (secure 5090, driver 595 / CUDA 13.2, ~34 min pod,
~$0.68; `analysis/gpu/phase3/results_5090_style/`) [measured]. Setup to
training ~2 min with the sm_120 cache (export 19 s, checks 36 s, build in
parallel); training 80M steps in 1754 s, 49-53k steps/s. The graph and
the build both said obs 20. watch.py, every 25 epochs: no alarm; fall 0.99
-> 0.21 by 13M, 0.03-0.05 from 59M; difficulty 1.0 from 39M; entropy 4.2
-> 1.1; kl 0.02-0.06. Converted 52M, 72M, 80M (`general_puffer_style_*`):
all 20/20 on the eval grid, standstill max |v| 0.11-0.19 m/s (fixes_72M
0.15-0.17). With the bit on they mostly refuse in-place turns: see
`wishlist.md`, "pivots". `standstill.py` needs `--config` for a 20-wide
policy, and `per_command --style` refuses a policy without the bit, even
at 0.

**Run 2: the multiplier, at 16k worlds (ran 10-18).**
`CONFIG=config/rl/rl_general_v3_style_mult.yaml NW=16384 HORIZON=16`.
- **Why horizon 16.** In the 5090 benchmark (`results_5090_bench/`, 240 s
  each, same ini) learning tracked EPOCHS, not steps: return 268 at epoch
  21 / 5.5M steps on 4k worlds, 398 at epoch 20 / 21.0M on 16k. An epoch
  is horizon x worlds samples, and 16k runs only 1.87x faster (94.6k vs
  50.6k steps/s), so 16k at horizon 64 is SLOWER per epoch in wall time.
  16k x 16 keeps the batch (262144 samples, 128 gradient steps an epoch,
  the checkpoint every ~6.5M steps): an epoch ~2.8 s against ~5.2 s, 80M
  steps in ~14 min. Cost: rollout pieces of 0.32 s (from 1.28), so the
  advantage bootstraps sooner and the MinGRU trains on shorter sequences.
  The hidden STATE is not cut [read, `src/pufferl.cu` @ ea7f989, and both
  runs' saved ini]: `reset_every_horizon = 0` (default.ini; not set by
  aow_bike.ini), so the state carries across horizons and is zeroed only
  on a terminal (`zero_term_state`; our bridge sends time limits as
  terminals, so every episode end). Training starts each segment from the
  state snapshotted at its first step (`snapshot_state` ->
  `initial_states` -> `train_state`): the GRADIENT is truncated at the
  horizon, the state is not. Also `async = 1`: rollouts act with a
  one-epoch-stale copy of the policy.
  32k / 64k would need horizon 8 / 4 for ~10 % more; not tried.
- **`HORIZON=`** seds `[train] horizon` after `install.sh`, like
  `PUFFER_SEED`; `run_pod.sh` logs it beside `total_agents` in build.log.
- **Two changes in one run** (the user's call): bit-off behaviour against
  run 1 mostly isolates the world count, the spins mostly the reward.

**It ran, 2026-10-18** [measured]. Two secure 5090s on 74.2.96.x pulled
pypi at ~200-300 KB/s (github 90 KB/s) and were terminated before any
copy; EU-CZ-1 (`dataCenterIds`) gave 224 MB/s. The FIRST attempt (flat
`--curriculum-lead 0.1`) was stopped at 464 s / ~35M steps: difficulty
0.013 at 19.7M and 0.117 at 32.8M, against run 1's 0.38 and 0.81. The lead
caps the rise at ~lead per episode length, i.e. per 750 x NW samples, 4x
more samples at 16k; `run_pod.sh` now defaults it to 0.1 x NW/4096
(`results_5090_mult_lead01/`). The relaunch, lead 0.4
(`results_5090_mult/`): 80M steps in 1085 s of training (82-90k steps/s;
1754 s at 4k), difficulty 0.17 (19.7M), 0.61 (32.8M), 1.0 from 52M; fall
0.02-0.08 from 59M; two KL HIGH panels (0.171 at 6.6M, 0.354 at 19.7M),
kl 0.03-0.05 after. Converted `general_puffer_mult_{52,72,80}M`: 0/20
falls each way, 7/9 turns forward, 2/2 straight, but the bit-off HOLD
creeps (-0.02..-0.05 m/s; standstill 0.13-0.24 m in 5 s, run 1 0.02-0.11;
eval-grid drift 0.15-0.20 m, run 1 0.015-0.025). The style result is in
`wishlist.md`. Pod ~32 min + two bad ones, ~$0.75.

Curves, the two runs on steps, training wall time and bike time per world
(steps / worlds x 0.02 s, in 15 s episodes):
`python analysis/gpu/phase3/compare_curves.py --tag style_4k_vs_16k
"run 1: 4k x 64, add=analysis/gpu/phase3/results_5090_style/train.log"
"run 2: 16k x 16, mult, lead 0.4=analysis/gpu/phase3/results_5090_mult/train.log"`
-> `analysis/gpu/plots/gpu_curves_style_4k_vs_16k.png`. The panel's
"difficulty" is the mean difficulty that episodes ENDING that epoch were
PLAYED at (`last_diff`), so it lags the shared value by up to one episode
(750 steps x worlds: 3.1M samples at 4k, 12.3M at 16k) and is not
monotonic -- run 2's dip 0.4 -> 0.13 at ~27M is the cohort started near
15M (at ~0.13) truncating together after the collapse [reasoned; the
shared value itself only rises: atomic adds, capped at 1, and is not
logged]. On that lagged panel, 0.9 at 35.1M / 11.2 min (4k) and 52.2M /
10.2 min (16k). 16k fell back to fall ~1.0 twice (~20-25M, ~33-38M) with kl
up to 0.96. In BIKE time run 2's bikes lived ~6.5 episodes each (run 1
~26), and run 1's difficulty climbs in stairs ~0.1 an episode apart (the
lead cap, visible) where run 2's are ~0.4 (lead 0.4): same pace per
sample, 4x coarser per episode. The second collapse follows the 0.4 ->
0.8 stair; the FIRST is not the stair -- the lead-0.1 launch (stopped)
collapsed at the same ~1.6 episodes / ~20-25M with difficulty ~0.05.
Cause unknown. 16k's entropy sits lower throughout; the lead-0.1 launch's
went to ~0.1 by 40M. One seed each. Underlying cost of more worlds: the
curriculum is paced per bike lifetime, and at 16k ten 0.1 stairs are
~123M steps.

**The v2 plant, no style bit (ran 2026-10-23)** [measured]. The first
policy trained on the bike as built: plant 6446d8fa (rake 20, CAD masses
1157 g, the measured steer kp 2.3 + rotor inertia, the AHRS 45 deg
nose-down), against the pointer's bc66ca20. Default `CONFIG`
(`rl_general_v3.yaml`, 17 wide, Pi-style steer rate), 4096 worlds x
horizon 64, `TRAIN_S=3600` (the ini's 80M ends it).
`results_5090_rake20/`. A secure 5090 in EUR-NO-1 pulled pythonhosted at
~280 KB/s and github at 40 KB/s (curl, 40 s) and was terminated before
any copy (~$0.08); EU-RO-1 gave 53 MB/s. Setup to training ~1.5 min with
the sm_120 cache; 80M in 1724 s at 50-54k steps/s.

| epoch | steps | len | fall | difficulty | entropy | kl |
|---|---|---|---|---|---|---|
| 25 | 6.6M | 338 | 0.937 | 0.000 | 3.35 | 0.175 (KL HIGH) |
| 50 | 13.1M | 714 | 0.128 | 0.154 | 2.81 | 0.022 |
| 125 | 32.8M | 685 | 0.191 | 0.819 | 2.00 | 0.035 |
| 200 | 52.4M | 741 | 0.037 | 1.000 | 1.66 | 0.039 |
| 305 | 80.0M | 742 | 0.028 | 0.667 | 1.27 | 0.033 |

Converted 52M, 72M, 80M (`general_puffer_rake20_*`) and judged on the v2
plant with the pointer beside them (`per_command.txt`, `standstill.txt`,
`steer_wobble.txt` in the results dir): all four 0/20 falls, 7/9 turns
forward, 2/2 straight, hold -0.02..-0.03 m/s, in-place spins steered
(wheel/yaw 0.95-1.10). Standstill max |v|: fixes_72M 0.17-0.21 m/s,
52M 0.13-0.16, 72M 0.14-0.17, 80M 0.18-0.21. Steer wobble amplitude
[deg] (`analysis/policy/steer_wobble.py`, 3 seeds):

| | fixes_72M | rake20_52M | rake20_72M | rake20_80M |
|---|---|---|---|---|
| hold | 6.7 | 4.9 | 5.4 | 6.4 |
| straight +0.8 | 1.3 | 4.3 | 4.9 | 5.2 |
| straight +1.2 | 2.5 | 17.1 | 9.9 | 5.9 |
| turn +1.2, 1.2 rad/s | 1.9 | 16.9 | 3.1 | 1.3 |

So the retrained policy is NOT smoother: at top speed it wobbles more
than the pointer flown on the same plant, shrinking with training (17 ->
10 -> 6 deg). It also overshoots 1.2 m/s (achieved 1.24-1.26; the
pointer 1.20). One seed: whether that is the plant or this seed is open.
The user, in teleop: "seems fine, don't see a huge difference".

**Several runs on one card: MPS (bench, same pod, 2026-10-23)**
[measured]. `analysis/gpu/phase3/bench_seeds.sh`: N independent 4096-world
runs at once, each with its own graph (`export_graph --seed k`) and its
own copy of `~/puffer` (ini seed 42+k), 120 s each, steps/s from 45 s on
(`results_5090_rake20/bench_seeds/`):

| at once | total steps/s | each |
|---|---|---|
| 1 (the run above) | ~51k | 51k |
| 2 / 4 / 8, plain | 57k / 54k / 55k | 28k / 14k / 7k |
| 4, MPS | 116k | 29k |
| 8, MPS | 127k | 16k |

Plain processes time-slice the card, so they gain nothing. Under MPS
(`nvidia-cuda-mps-control -d`, started inside the container without
trouble) kernels from different runs share the SMs: 4 runs give 2.3x a
single run's throughput, 8 only ~10 % more than 4. So **4 runs under MPS
is the sweet spot**: 4 x 80M in ~46 min against 29 min for one. Each run
has its own graph and ini, so the four may differ in anything those
carry -- the env config (reward weights, randomization, curriculum) and
the trainer ini (seed, learning rate, entropy, horizon). A config that
changes the obs width needs its own `install.sh` build (OBS_SIZE is
compiled in); the copies share one build.

**The runbook as a script (2026-10-23; first run end to end 10-26, below).**
`scripts/gpu.sh` drives steps 2-7 above from this checkout against any
ssh-reachable root shell with an NVIDIA GPU (CUDA >= 13 driver; only the
RunPod image is tested): `check HOST PORT` (CUDA, pypi and github speed),
`up HOST PORT [SEEDS=4 CONFIG=... ...]` (bundle, the newest Warp cache,
launch detached), `watch HOST PORT` (every run's checkpoint rows until
ALL_DONE), `pull HOST PORT TAG` (to `results_TAG/`), `export TAG [52 72
80]` (the checkpoints nearest those millions of steps, to
`general_puffer_TAG[_s<k>]_<N>M`; an existing name is skipped). Creating
and terminating the pod stay outside it. `SEEDS=N` (N > 1) makes
`pod_launch.sh` run `run_seeds.sh` instead of `run_pod.sh`: N graphs
(env seed ENV_SEED+k-1), N trainer copies (PufferLib seed
PUFFER_SEED+k-1, so run 1 is a one-run launch's pair), MPS on, each
trained to the ini's 80M, `results/seed_<k>/` each with its checkpoints,
ini, graph json and watch.log. A one-run launch now also leaves the graph
json and ini in `results/checkpoints/`, so `export` reads both layouts.
Checked here: syntax, the argument guard, and `export` on this run's
results in both layouts (the 80M weights byte-identical to the export
above; the test exports were `*smoke*` and are deleted). `run_seeds.sh`
had not run on a GPU then. Two bugs found on the 10-26 run, fixed: `watch`
waited forever on a one-run launch (`ls a b` fails when either is
missing, and a one-run launch has no `seed_*/`), and `export` took the
first run folder under `puffer_checkpoints/aow_bike/`, which after a
relaunch on the same pod is the stopped run's empty one (it now takes the
newest that holds checkpoints).

**The v4 plant: sawtooth detent + wheel play, on an A40 (ran 2026-10-26)**
[measured]. `config/rl/rl_general_v4.yaml` (v3 + the sawtooth drivetrain
and the play fit inline; `next-retrain.md`), plant 54fdba72, seeds 0 / 42,
4096 x 64. No secure or community 4090 / 5090 at any CUDA version; a
secure A40 ($0.59/h, CA-MTL-1; github's test URL read 0 B/s, github.com
itself answered, setup never fetches from it). Results:
`results_a40_v4/`, the two stopped attempts beside it.

| on the A40, v4 | steps/s | 80M takes |
|---|---|---|
| `SEEDS=4`, MPS | 27k in all, 6.8k each | ~3.3 h each |
| one run | 19.4-19.7k early, 16.0k by the end | 5361 s (89 min) |

So MPS gave 1.4x on the A40 against 2.3x on the 5090; the 4-seed launch
was stopped at 222 s (`x4_a40_aborted/`). `run_seeds.sh` worked end to
end otherwise: four graphs, four inis (seeds 42-45), MPS on, checkpoints,
the Warp cache, ALL_DONE. The first one-run relaunch was stopped at ~3 min
(`x1_a40_capped/`): the one-run `TRAIN_S` default of 3600 s would have cut
it near 70M; `TRAIN_S=5400` finished on the ini's 80M with 39 s to spare.
The rate fell ~20 % over the run (19.7k -> 16.0k), as on no 5090 run.

Training curve against the v2-plant run (5090, same seeds) -- one seed
each, not repeatable:

| steps | fall v3 / v4 | difficulty | entropy |
|---|---|---|---|
| 13.1M | 0.13 / 0.26 | 0.15 / 0.11 | 2.81 / 2.48 |
| 32.8M | 0.19 / 0.64 | 0.82 / 0.76 | 2.00 / 1.52 |
| 45.9M | 0.06 / 0.40 | 1.00 / 1.00 | 1.70 / 1.27 |
| 80.0M | 0.03 / 0.10 | 0.67 / 0.67 | 1.27 / 0.59 |

Difficulty (paced by tracking, not falls) climbed as fast; falls peaked
later and higher and came down; entropy fell faster. One KL HIGH (0.131
at 26.2M). Converted `general_puffer_a40_v4_{52,72,80}M` (each records
`wheel_play` and the sawtooth overlay). Judged here, `per_command.py
--summary` (each on its OWN plant, from its record): all five 0/20 falls,
7/9 forward, 2/2 straight, hold -0.01..-0.03 m/s, spins steered (wheel/yaw
0.96-1.05) -- the same as `fixes_72M` and `rake20_80M`. `standstill.py`
(6 seeds, 5 s) on the v4 plant for all five, max |v| / end distance:
pointer 0.19-0.23 m/s / 0.12-0.15 m, rake20_80M 0.18-0.26 / 0.15-0.17,
v4 52M 0.14-0.21 / 0.08-0.12, 72M 0.14-0.17 / 0.07-0.08, 80M 0.12-0.19 /
0.05-0.11; no falls. On the v3 plant (`standstill_v3plant.txt`) the same
order (pointer end 0.10-0.12 m, v4 0.05-0.11): the play changes little
for any of them in 5 s. NOTE `standstill.py` flies every policy on its
`--config` plant, ignoring the move's record (default v3: no play).
Not yet: the 7-12 Hz shake with the play (`run_vs_sim.py` against 174349),
the steer wobble, and the bike.

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
