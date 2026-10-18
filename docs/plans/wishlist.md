# Cool moves

> **Status, 2026-10-16 (third pass: lie-down fixed on the detailed drive,
> close-range plays with their own ball and crank, floor tilt in training).**
> Second pass: The showcase moves, each with what
> exists, what is missing, and the first tests -- most of which have now RUN,
> results inline under each section. New this pass: the ramp test
> (`analysis/ramp_test.py`), timed command plays in teleop
> (`control/command_play.py`, `config/plays/`, `analysis/ball_play.py`), lie
> down / get up / auto-up on the swing module (`control/swing_sit.py`,
> teleop PageDown / PageUp, `analysis/swing_sit.py`), and the wheelie launch
> (`analysis/wheelie_launch.py`). Ordered easiest first. Evidence tags as in
> `general-rl-improvements.md`: **[measured]**, **[read]** (code read),
> **[reasoned]**. Open questions for the user are marked **(user)**.

**Blockers for the physical build: none found.** Nothing here asks for a
change to the bike as designed:
- Ramps, plays and lie-down / get-up run on today's plant.
- The wing-edge shot and the wheelie's side support both use the blade as
  it is.
- The blade is a swappable part anyway (user, 2026-10-16), so a move that
  wants a different edge or toe is a reprint, not a redesign.
- The open items are sim work (training the style bit, a tilt arm) or measurements
  only the built bike gives: roller mu for the wheelie, the real ramp, the
  lie-down on hardware.

| move | what exists | what is missing | difficulty |
|---|---|---|---|
| ramps | the ramp test (both GPU policies to 15 deg); floor tilt in training, CPU + GPU, off by default | the real ramp; a tilt arm if the bike disagrees | easy |
| ball shots | close-range plays in teleop (End): ram 1.27, wing edge 1.37, wheel swing 0.88 m/s | wheel guard (CAD); the wing edge on the bike | easy-medium |
| 180 past 90 deg steer | obs and action allow it; the style bit built (10-17), untrained | a GPU run | medium |
| getting up and down | lie down / get up / auto-up in teleop (PageDown / PageUp), fixed on the detailed drive | the blades on the bike; a hardware run | medium, hardware-gated |
| wheelie | launch: lifts above floor mu ~0.75; a blade can carry the roll | a pitch balance controller; the real roller mu | hard |

---

## ramps (easy demo)

Your notes:
- current teleop seems resilient to slopes on `general_puffer_fixes_52M`
- need to triple check the slopes sim2real

**What exists. [read]** Teleop compiles floors at 0 / 2 / 5 / 10 deg
(`run_drive.FLOOR_TILT_STEPS`, `--slope-bearing`). It tilts the FLOOR, not
gravity, so world Z stays vertical and the policy sees the standing roll offset
a real AHRS would on a cross-slope (`build_model.py`, the floor block).
`analysis/pen_slope.py` tilts gravity instead, so its SENSING is wrong on a
cross-slope (its own docstring says so). Use teleop or the floor tilt for
anything about the policy's behaviour.
- yea if the normal bike drives fine on level ground, I'll try a slope and
  if it doesn't agree with the sim then we fix the sim

**What training never sees. [read]** No config sets `floor_tilt_*`, so every
policy has trained on level ground only. Slope behaviour is whatever
generalises.
- yep would train on floor tilts if needed, does that just plug in?

  **Not quite: four pieces, none of them large. [read]**
  1. **Tilting the floor per episode.**
     - CPU: set the floor's `model.geom_quat` at reset, as
       `run_drive.tilt_floor` already does.
     - GPU: see the MJWarp answer below.
  2. **Spawning on the plane.** Every reset puts the bike level at the origin,
     but the tilted plane passes through the origin. At 10 deg the front
     contact is then 35 mm off the plane (0.20 m x tan 10). The spawn pose has
     to be rotated onto the plane: a few lines in `general_env.reset` and in
     `gpu/reset.py`.
  3. **The reward.** v3 observes pitch (`obs_pitch: true`) and has no
     `w_pitch` penalty, so nothing charges the policy for the slope's own
     pitch. Any arm that turns `w_pitch` on would need it measured against
     the floor normal.
  4. **The curriculum.** Tilt and bearing as a randomized range, or as a
     curriculum axis like the command difficulty.
  - (yes: no `w_pitch` ever; a curriculum angle, not critical yet, but as
    an option, default 0)

  **Built, 2026-10-16, OFF by default.** Two env keys:
  - `floor_tilt_max_deg` (0 = off): the tilt is uniform in [0, max], with
    any downhill bearing.
  - `floor_tilt_curriculum` (true): multiplies the tilt by the difficulty.

  Both CPU (`GeneralEnv._tilt_floor`) and GPU (`gpu/reset.py`). Off, no
  draw is made, so every existing run reproduces bit for bit. The bike
  spawns level to the ground, as teleop's `place_bike` does. `rl_general_v3.yaml` is
  NOT edited: every GPU export records its sha256, so a tilt arm is its
  own copy with the key set. `tests/test_floor_tilt.py`.
  - **A trap found on the way. [measured]** CPU MuJoCo ignores
    `geom_quat` for a geom compiled with `geom_sameframe = 1`, and the
    level floor is one. Turning only the quat left the floor level, and
    the bike spawned 28.9 mm into it at 9.7 deg. The env clears the flag.
    `run_drive.tilt_floor` has the same bug; nothing calls it.
  - **The GPU path. [measured on Warp's CPU device, 8 worlds]** Each world
    tilted differently (0.8-11.4 deg), each axle spawned 51.04 mm above
    its own plane, and none sank through it. Not yet run on a GPU or in
    training.
  - **What the level-trained policies do on it. [measured: v3 env, 20
    seeded episodes at full difficulty]**

        tilt up to        0      5     10     15 deg
        fixes_52M      0/20   9/20  10/20  13/20   fell
        v3_s1          3/20  10/20  12/20  14/20

    Same draws in every column (the "0" column is 0.001 deg, so it makes
    the same draws). With truth sensors fixes_52M is still 0/20 vs 8/20,
    so the sensor models are not the cause. These are full-difficulty
    commands (reverse at 1.1 m/s), not the gentle ramp course above.

**The sim2real checklist:**
1. **Ramp edges.** The sim tilts a whole infinite plane through the origin. A
   real ramp has a kink at the bottom and the top, and the 0.20 m wheelbase
   crosses each one. Not modelled anywhere. The cheapest fix is a box geom
   ramp in a scene file; a heightfield is the general one.
   - not really worried about that part, the ramp edges are just varying slopes
     to the bike (short of bottoming out or whatever)
2. **Sensing.** The policy reads roll from the AHRS filter (`tm151_filter`,
   tau 2.0 in the evals). On a slope the filter's gravity reference is right,
   but the transient when the bike rolls onto the ramp passes through it. Fly
   the ramp in teleop with the AHRS model on, not truth.
   - yea should always have AHRS model
3. **Traction and torque. [reasoned]** Climbing 10 deg takes
   m g sin(10 deg) = 2.1 N against ~21 N of rear stall force (2 x 0.533 N m
   at the inputs / 0.0512 m), so torque is not the limit. Roller friction on
   the real surface may be.
4. **Measure the real ramp.** Angle, surface and edge shape, then put those
   in the scene and not the 2 / 5 / 10 dial.
   - yea can make a real ramp but the RL training (if any) should generalize from
     the various angled surfaces trained on

**If it needs training:** a per-episode floor tilt in `randomization:` on the
CPU env is a config-scale change. On the GPU env, whether MJWarp can tilt a
geom per world has NOT been checked.
- check whether MJWarp can tilt a geom per world (we just render all the floors
  in teleop and only show one at a time, not sure if that's valid or efficient here)

  **It can, with one catch. [read: mujoco_warp `io.put_model`, `smooth.py`]**
  - `geom_quat` is a batchable model field (`batch_sizes={"geom_quat": n}`,
    the same mechanism `gpu/env.py` uses for `body_mass` and
    `geom_friction`).
  - **The catch.** MJWarp treats geoms on the world body as static: their
    `geom_xpos`/`geom_xmat` are computed ONCE, in `make_data`
    (`_geom_local_to_global` returns early for them). Changing `geom_quat`
    at reset therefore moves nothing. The reset kernel has to write the
    floor's `d.geom_xmat[world, floor]` directly. The plane passes through
    the origin, so `geom_xpos` stays 0. That write is a per-world Data
    field, as cheap as the other reset writes.
  - **The teleop way, corrected.** MJWarp builds its collision pair list
    once, in `put_model`, from contype/conaffinity. Floors switched off
    before that cost nothing; only floors left ON add a pair per geom per
    world. My "plane-vs-everything per floor" held only for that case. It
    is moot anyway: one floor with a per-world pose is what was built
    (above).
  - **Tested** on Warp's CPU device (above), not yet on a GPU.

**First test:** a box ramp at the measured angle in teleop with
`general_puffer_seed1_72M` and `fixes_52M`, AHRS model on, up and down and
across.
- yeah you can formalize this test

**Formalized: `analysis/ramp_test.py`. [measured, 2026-10-16]**
- **The course.** A box ramp rising from the flat floor at x = 1 m, so it has
  a real bottom edge. One play: drive up at 0.5 m/s, hold facing up (4 s),
  turn across and hold (4 s), turn to face down and hold (3 s), drive down
  and off, stop.
- **The sensors.** AHRS tm151_filter at tau 2 s, plus teleop's odometry
  (front, encoder counts).
- **Result.** Both policies do the whole course at 5, 10 and 15 deg, and
  both fall at 20: seed 1 while holding facing up, fixes_52M driving down.

    moved during the holds [m]       5 deg   10 deg   15 deg   (flat)
    seed1_72M   facing up            0.07    0.15     0.25     0.02
                across               0.36    0.58     0.66     0.30
                facing down          0.32    0.57     1.15     0.31
    fixes_52M   facing up            0.07    0.14     0.24     0.03
                across               0.28    0.36     0.37     0.34
                facing down          0.40    0.34     1.03     0.33

  - "Across" and "facing down" include the turn itself (~0.3 m on the
    flat). The growth over the flat is the slope pulling the bike downhill
    while it is told to stand still: 0.7-0.85 m in 3 s facing down at 15 deg.
  - **Worst |roll| in the across turn.** seed1_72M 14-17 deg (12 on the
    flat: its fast turn, not the slope). fixes_52M 3-6 deg.
- **Not modelled.** The real ramp's surface friction (it is the floor's
  0.9 GUESS), and the drivetrain detail (ideal drive).

---

## ball shots (makes it more fun)

Your notes:
- current teleop seems resilient to 5.5 m/s on `general_puffer_fixes_52M`, I
  think this should just be commanded with a preset heading/velocity
  trajectory on top of the policy
- front wheel needs a guard (from fork pieces)
- should also just be able to take some hits, and seems fine on that in the
  latest

**What exists. [read]**
- `run_drive --ball` adds the stick panels and the road-hockey ball. Key `1`
  re-parks the ball and fires `ball_rl`, the dedicated shot policy.
  - oh I've been using `--hockey`, you press space bar to shoot a random ball
    at the bike, `[` and `]` change the ball speed
  - (Right: `--hockey` is the flag; `--ball` was my slip. Key `1` firing
    `ball_rl` is analytic-mode only.)
- `ball_rl` predates the plant digest, so it was trained on an old plant.
- `docs/plans/ball-shot-move.md` has the concept (stick panels) and the
  hardware clips from 2026-08-24, "the wheel shot" (`traces/ball_shot_traces/`).

**Your plan, a scripted command sequence on the general policy, is the
cheap route.** No new policy, and no stick: the shot is the wheel shot.
A sequence would be:
1. Approach along a line offset from the ball.
2. Step the heading command at a set distance, so the swing carries the
   front wheel through the ball.
3. Hold, then stop.

This needs a timed command player in teleop (key -> list of
`(t, v_lon, v_lat, psi)`), plus a sweep over offset, speed and step angle in
sim, scored by ball speed and direction.
- this seems worth implementing in general

**Limits. [read]**
- The general policy trained with `v_max 1.2` m/s, so any command above that
  is outside training.
- **(user)** What the 5.5 m/s in your note is: the bike's speed, or the
  ball's into the bike?
  - that is the velocity of the ball on `--hockey` (adjusted with `[` and `]`
    to 5.5m/s)

**Taking hits.**
- `rl_general_v3.yaml` has `disturb_prob: 0.0` ("pushes off for now"), so
  the current policies never trained against a push. Any resilience
  generalised from elsewhere.
- Turning pushes back on is one config line, at the cost of a new run.
- Ball strikes in teleop are the test.

**The guard** is CAD (fork pieces). Its mass and inertia go into the front
assembly, which changes the plant digest (a small re-export, and the policies
become provisional).

**First test:** the command player plus one hand-tuned shot in teleop, then
the sweep.
- agree command player good here

**Built and swept, 2026-10-16.**
- **The player.** `control/command_play.py`: keyframes of the same command
  the keyboard sends, `[t, v, v_lat, psi_rel_deg]`, held or ramped.
  - Teleop: `--play config/plays/<name>.yaml` (repeatable). End (fn+right)
    plays the next one; any driving key (arrows, 1/3, 5, /) aborts.
  - Shipped: `config/plays/ram.yaml` and `wheel_swing.yaml`.
- **The sweep. [measured: `analysis/ball_play.py`, seed1_72M, truth,
  `--hockey` model, ball 3 m ahead, bike from rest]**
  - **A straight ram is the strongest shot.** The ball leaves at 1.20 m/s
    (the bike's speed), with the bike upright.
  - **A heading step that swings the front wheel through the ball** (+-45 /
    90 deg, at 0-20 cm before it) never beat that: 0.4-0.87 m/s. The
    hardest ones FELL.
  - **Aim is the limit.** The ram vs the ball's offset from the path:

    ball y [cm]    -4    -3    -2    -1     0    +1    +2    +3    +4
    ball m/s     0.68  0.83  0.87  1.11  1.20  1.21  1.11  0.85  0.70
    fell                  yes                                   yes

  - **So:** at the bike's 1.2 m/s ceiling the shot is at most ~1.2 m/s, and
    open loop over 3 m the bike's own wander (several cm) eats it. Aim by
    hand, close in, then play. A harder shot needs more bike speed than
    training's `v_max` (off-distribution) or a stiffer, bouncier contact
    than the ball's GUESS parameters.
- **5.5 m/s is the `--hockey` ball's speed into the bike**, so the bike
  taking a 60 g ball at 5.5 m/s is the robustness number.

**Second pass, 2026-10-16: close range, your wing-edge idea, a full
teleop example.**
- **3 m was too far** (you: hit the button "way closer"). The bike is at
  1.2 m/s within 0.3 s and 0.25 m, and over 3 m a standing bike's heading
  wander (a few degrees) puts it 10-15 cm off the ball. The plays now start
  0.3-0.6 m from the ball.
- **Plays can carry their own ball and crank.**
  - `ball: [x, y]` parks the `--hockey` ball in the bike's frame when the
    play starts.
  - An optional 5th key column is the crank goal in degrees.
- **The three shots. [measured, `analysis/ball_play.py --play`, seed1_72M,
  truth, ideal drive]**

    play              ball at (m)       ball m/s   notes
    ram               0.6 ahead, 0      1.27       2 cm off: 1.18
    wing_edge         0.6 ahead, 0.10R  1.37       blade 105 deg; 12 cm 1.27,
                                                   14 cm FELL
    wheel_swing       0.30, 0.16R       0.88       spin -120 deg from rest;
                                                   ball off ~75 deg right

  - **The wing edge (your idea) is the best shot.** It is faster than the
    bike (the ram cannot be), and its aim window is ~3 cm against the
    ram's ~+-1. It is only a crank command on top of ordinary driving.
  - **The wheel swing only works from beside the ball.** At 0.6 m a
    heading step turns into a ram before the wheel swings.
- **In teleop, as launched** (default policy, its drivetrain, odometry;
  three End presses each): ram 0.79-1.30, wing_edge 1.34-1.40, wheel_swing
  0.40-0.54 m/s. Every press hit.
- **Full example:**
  `mjpython -m aow_sim.run_drive --teleop --hockey --play config/plays/wing_edge.yaml`,
  then End. Each play file has its own example and numbers in its header.

---

## using the steering past 90deg (e.g. the 180 flick; using the full steer capability, sin/cos on the control policy)

Your note:
- you can do the 180 degree turn with a straight wheel in world coordinates
  but I only got it randomly in training a couple of times

**The general policy is not blocked from it. [read]**
- The steer is observed as sin/cos(2 delta) (`general_spec.build_obs`). That
  is winding-invariant and smooth through 180, because the wheel is
  front-back symmetric.
- The action is a steer RATE integrated into a multi-turn target, so it can
  sweep through 90 and 180 without wrapping.
- The "sin/cos on the control policy" part is already done.

**The move already exists as its own policy. [read]**
- `control/pivot_spec.py` describes exactly this: "the chassis yaws to a
  target (180 deg for now) while the FRONT WHEEL HOLDS ITS GLOBAL GROUND
  HEADING".
- It works from a standstill or a glide, and finishes gliding backward.
- `pivot_rl` / `_v2` / `_og` exist, trained on an old plant (no digest field).
- `flick_rl` is the steer-sweep version.

**Why the general policy rarely does it. [reasoned]** Nothing asks for it. The
heading reward pays for getting `psi` there fast, and a plain spin does that
with less risk. A pivot happens only when it happens to score better.
- yeah it's probably not most efficient, but it did come out of RL before

**Three routes:**
1. **Handoff (recommended first).** Retrain `pivot_rl` on today's plant (its
   CPU env still exists), then wire `drive.py` to go general -> pivot ->
   general.
   - Today the moves hand back to the balance underneath, which is the
     deprecated LQR.
   - Reuses the most code. Costs a CPU run, plus the handoff state check
     (`recovery.ready_for_policy`).
2. **A style bit in the command.** Add a "pivot" flag to the obs and reward
   the front-wheel world heading while it is set. One policy does both, but
   it changes the obs layout (new runs, GPU env and `export_graph` included).
3. **Reward-only nudge.** Cheapest to write, but it changes every other
   behaviour too. Not recommended.

**First test:** fly the existing `pivot_rl` on today's plant in teleop to see
how far off it is before retraining. A stale plant may still mostly work.
- HOLY this is an old one but yes take a stab at running it
- style bit would be cool as a separate input, it's just a bit or can be a knob?

**Ran it, 2026-10-16: every pivot policy falls on today's plant.
[measured: `rollout_move`, default params]**
- **The yamls are not what their names say.** `pivot_rl_og.yaml` and
  `pivot_rl_v2.yaml` both say `name: pivot_rl` and
  `policy_file: pivot_rl.npz`, so loading any of the three flies
  `pivot_rl.npz`. The `_og` and `_v2` weights were flown through temporary
  `*_smoke` yamls (deleted afterwards).
- **How they fall.** From a standstill, `pivot_rl` tips over 1.9 s in,
  0.9 s into the move, with the yaw only at ~90 deg.

    from rest      max |roll|   yaw turned
      pivot_rl     126 deg      +99 deg
      _og          121          +171
      _v2          180          +196

    at 0.4 m/s     max |roll|   yaw turned
      _og          115          +102
      _v2          165          +120

- **Not a surprise.** These were trained in August on a ~1.0 kg bike
  without the righting module. Today's is 1.25 kg, with a different steer
  servo model, contact and drivetrain. Route 1 starts with a retrain.

**Bit or knob. [reasoned]** Either can be a separate input. A knob is only
meaningful if the reward is a continuous blend, e.g. the front-wheel hold
term weighted by the knob. Then 0 is today's turn, 1 is the pivot, and
in-between is whatever PPO finds for a half-weighted hold, which may not
look like anything in particular. A bit is the cleaner first version; a
knob can come later without changing the obs layout (same slot, 0..1
instead of 0/1).
- The bit a like a global brake for the front wheel; it makes it try to stay
  at the same global heading while it's on; only becomes relevant when heading
  moves are introduced (bit probably flips randomly on the same schedule as commands,
  scale the bit ratio with difficulty+curriculum)
- (you: add the style bit, not a pivot_rl retrain)

**Built, 2026-10-17; not trained. [measured where marked]** What changed
from the design below, mostly after your review:
- **An angle, latched, observed.** While active, the reward is
  `w_style x exp(-(e / sigma)^2)`, with e the front wheel's world heading
  minus where it was when the bit came on, mod 360 (`general_spec.wheel_err`).
  It is in the obs as sin/cos, beside the bit (17 -> 20 wide). The bike forms
  it the same way: its own heading plus the steer angle (`drive.py`).
  - A first version paid for a still wheel (zero RATE) instead. You caught
    the flaw: its cost per step is bounded, so one fast steered flick paid
    ~0.6 s of it and nothing after, and a slow drift paid almost nothing.
  - Mod 360, not 180: mod 180, a steered 180 deg spin would come back to its
    latch for free.
- **Active only at a zero speed command** (a hold or an in-place turn), in
  training and replay alike (`style_active`).
  - The front wheel rolls only along its own heading, so a held wheel and
    travel along any other line cannot both be met. 90 deg + forward is
    impossible, not hard.
  - On a straight the bit would only fight the course, so straight is out
    too.
  - On a hold it means "stop steering": balance from the differential alone.
  - The latch is taken on the rising edge, so a chain of in-place turns holds
    one wheel heading.
- **Moving turns: the crab is the dual, not the bit.** While moving, the wheel
  has to come round to the new course; what can stay held is the CHASSIS.
  That is a crab, already a command (lateral velocity, heading kept), never
  trained since `v_lat_frac: 0`. Its own experiment if wanted.
- **Where:** `config/rl_general_v3_style.yaml` (v3 plus `obs_style`,
  `style_p` 0.5, `w_style` 0.75, `sigma_style_deg` 25; the header says why);
  GeneralEnv and the GPU kernels (Warp CPU device checked against the spec);
  the move yaml; `drive.set_style`; teleop **Home** (fn+left); `per_command
  --style 1` and a wheel/yaw table under `--summary`.
- **Override, for after training:** teleop `--style-ungated`
  (`DriveController.style_ungated`) lets the bit act at a speed command too.
  It asks the policy something it never trained on, so it's an experiment.
- **Baseline [measured, bit off, tm151_filter tau 2.0].** No current policy
  pivots. The front wheel turns 0.92-1.05 deg per deg of chassis yaw on all
  five in-place spins (+-90, +-170, 180), for fixes_72M, fixes_52M,
  seed1_72M, v3_s0 and v3_s1. A pivot reads ~0.
- **Trained, 2026-10-17 [measured]** (5090, 80M steps,
  `general_puffer_style_{52,72,80}M`): **with the bit on it mostly does not
  turn.** On the in-place spins the chassis yaws 0-17 deg against a 90-180
  deg command and holds the wheel (head error 75-180 deg at the end) in
  9 of 15 spin episodes across the three checkpoints, 72M and 80M on
  every spin but +90. 52M still turned steered on three (wheel/yaw
  0.8-1.07) and stopped at -33 on -90. 80M on +90 did pivot (yaw 149,
  wheel -7) but overshot by 60 deg; 72M fell on +90. Bit off: steered
  spins as before (wheel/yaw 0.91-1.04), 0/20 falls, stands still. So the
  bit outbids the turn, the failure the weights were meant to rule out.
  Not the reward clamp (scale 0.02, a step is ~0.08).
  [reasoned] Per step, not turning pays w_style 0.75 at once; a finished
  steered turn pays w_head 1.0. The margin, 0.25 a step, starts only after
  the turn, so it is discounted, and the spin itself costs velocity,
  upright and smoothness terms. The weight argument compared a pivot with a
  steered spin and never with refusing.
- **By hand, teleop (user, 10-17).** 72M is the most promising. A held
  left key at zero speed (the command slews 1.2 rad/s and leads the bike
  by at most 35 deg, `run_drive._LEAD_MAX`) pivots as wanted -- on 1 of the
  3 checkpoints, one direction only. With `--style-ungated` the wheel stays
  pointed while driving, visibly more than gated (where it frees the
  moment a speed is commanded); untrained, and what it is doing is an open
  question. Hold: bit off holds still; bit on creeps backwards with less
  roll and steer. Headless, 15 s, 72M / 80M, 3 seeds [measured]: bit off
  -0.1..+0.1 cm/s, roll sd 0.7-1.0 deg, steer sd 3.3-5.0 deg; bit on
  -1.5..-2.3 cm/s, roll sd 0.46-0.61, steer sd 1.8-2.6.
- **The 90 deg point (user).** Pivoting about a held wheel, steer passes
  90 deg relative to the chassis at a 90 deg turn, an unstable point;
  big turns need it to learn to go past that rather than stop short of it.
- **Next (not decided):** a form in which refusing earns nothing, e.g. the
  bit as a multiplier on the heading term, w_head x r_head x (1 + k x
  r_style): pivot > steered > refuse at any discount.

**The style bit: the design, as written before it was built. [reasoned]**
- **Obs.** A new optional block `obs_style: ("style",)`, appended through
  `general_spec._OPTIONAL_BLOCKS` like `obs_pitch`. Every existing policy
  keeps its layout. The value is 0/1 now, with the same slot for a knob
  later.
- **Command.** Drawn with each resample (`_sample_command`; on the GPU
  `commands.sample_family`). It is on with probability
  `p_style x difficulty`, so it arrives with the curriculum. When it turns
  on, the front wheel's world heading is latched:
  theta0 = psi + wheel_heading(steer), mod pi (`steer.wheel_heading`).
- **Reward, while on.** `w_style x exp(-(wrap_halfpi(theta - theta0) /
  sigma)^2)`: your "global brake for the front wheel". Off, the term is
  absent, so the bit costs nothing when it is 0.
  - **A heading command still decides where the chassis goes.** The bit
    only says HOW: around a held wheel (the pivot) instead of a steered
    turn.
  - **Open question.** Should the bit also be on for straight commands?
    Then the wheel heading is simply held, which the policy already does.
    That costs nothing and gives the bit a meaning in every state.
- **Where it touches.**
  - `general_spec`: block, names.
  - `general_env`: draw, latch, reward, info.
  - GPU: `gpu/commands.py`, `gpu/obs_reward.py`, `gpu/reset.py`, and the
    obs width in `export_graph` / `export_policy`.
  - Replay: `drive.py` puts the bit in the obs, and teleop gets a key to
    toggle it.
  - Eval: per_command gets "spin +180 with the bit" rows.
  - The `spec` and `policy` tests. A new GPU run, with the bit on.
- **Measured before building:** nothing. The one number worth taking first
  is how often today's policies already pivot on spin+180 (front-wheel
  heading change during the turn). That is a baseline the bit must beat.

---

## getting up and down cleanly (showing the extra axis)

Your notes:
- drop to the ground intentionally and turn off (main) policy (fast+smooth)
- get back up and engage (main) policy (fast+smooth)

**Up exists in sim. [read]** `control/righting.py`'s `RightingSequencer` runs
three steps:
1. Lift: slew the module into the floor until the bike rolls up.
2. Balance: hand to the general policy once the roll is inside its
   recoverable set, holding the module where it is.
3. Retract: only after the policy has held it for a while.

`analysis/self_righting.py` and `record.py` drive it. How fast and smooth it
is with V3 and today's policy has not been measured here. That is the first
number to take, and "fast+smooth" needs a target against it **(user)**.
- Yeah I don't have this in teleop at the moment (auto up), was doing manual
  but I think it's time to put the auto up in

**Down is new.** It is the reverse sequence, and nothing implements it:
1. Hold to a stop.
2. Deploy to the fall-catcher angle (~90 deg; the general spec's wing cap
   keeps the bike on its wheels there, while above ~96 deg it jacks the bike
   up).
3. Cut the policy and lean the bike onto one blade.
4. Lower it with the crank.

The side it falls to has to be chosen. Two ways:
- a small roll command before the cut, or
- a deliberate steer and diff kick.

**Scripted or learned:**
- **Scripted (first).** Phases on top of `RightingSequencer`, tuned in sim.
- **Learned.** The general policy can own the wings already: `act_wings` is
  a 4th channel. It is still an experiment, and the 10-09 note says the
  swing actuator was not commanded in training until then. A "sit" command
  would be new.
  - yeah scripted for now. as gentle of a fall as possible please

**Gated on hardware:** the blades on the built module (`docs/status.md` still lists the print as a next step; it is out of date).
- I printed and built the righting module, just haven't put the blades on it yet

**Built, 2026-10-16: `control/swing_sit.py`, in teleop.**
- **Correction to "Up exists in sim" above.** `RightingSequencer` drives the
  OLD mechanisms (arm, geared wings, the first linkage), not the bike's swing
  four-bar. For the swing there was only teleop's manual 9/4.
- **Teleop.**
  - PageDown (fn+down) lies down onto the side the bike already leans
    toward, and it stays down.
  - PageUp (fn+up) gets up.
  - After a fall it gets up by itself: down past 60 deg, nearly still, for
    0.5 s. `--no-auto-up` turns that off.
  - Plain numpy, so the same sequence can run on the Pi.
- **Down, as gentle as it gets in sim:**
  1. Crank to 110 deg on the chosen side with the policy still balancing.
  2. Policy off, drives held; a 10 rad/s, 0.2 s rear-differential kick
     leans the bike onto the blade, and it rests there at ~6 deg.
  3. The crank goes back to centre at 30 deg/s, rolling the bike down.
  4. The Goal Current ramps to 0 over 1 s.
- **Up:**
  1. Crank goal ramped at 120 deg/s to the full stroke on the side it lies
     on, policy off.
  2. The policy takes over inside `recovery.ready_for_policy` (12 deg,
     3 rad/s).
  3. The blade retracts 1 s later.
- **Measured [`analysis/swing_sit.py`; seed1_72M and fixes_52M; truth and
  tm151_filter tau 2 alike]:**

    down        5.9 s, peak 97-100 deg/s, chassis 7-12 N, still at 79-80 deg
    up          hand-off 1.5 s, riding 2.7 s, worst |roll| after 11.7-12.0
    auto-up     detected 3.7 s after a shove, hand-off 1.0 s, riding 2.2 s
    (a FALL, for scale: 720-780 deg/s, 150-200 N on the chassis)

- **What the sweeps ruled out:**
  - **Leaning with the blade at the full stroke.** The blade already
    touches the floor upright and props the bike over the OTHER way, every
    trial.
  - **Holding the crank at full current once down.** The bike rocks
    +-4 deg at ~35 deg/s forever.
  - **Going limp at once.** It drops the last few degrees: 152 deg/s, 40 N.
  - **A lift that steps the crank.** It throws the bike through upright at
    ~300 deg/s, outside the hand-off gate, and over onto its other side.
  - **The policy on during the lift.** Failed 2 of 3.
- **Not tested.** The detailed drivetrain, the left/right difference on the
  real bike, a battery below 12 V (the Goal Current is teleop's 785 counts).
  Everything here is sim.
  - (you: PgUp/PgDn in teleop is janky; sometimes it falls to the wrong
    side, and then gets up wrong too)

**Fixed, 2026-10-16: it was the detailed drivetrain.**
- **Reproduced** headless with teleop's own `main()`: fixes_52M on its
  detailed XC430 drive, with odometry. One wrong-side fall and one bad
  get-up in 6 tries. The tuning above had been done on the ideal drive.
- **Wrong side.** The rear kick's first effect is a brief roll the wrong
  way, and a fixed 0.2 s pulse sometimes ended with the bike still leaning
  away (the policy had parked it there).
  - Now the kick is held (20 rad/s) until the bike leans 3 deg onto the
    blade, up to 1.5 s.
  - If it goes 8 deg the other way instead, the policy gets the bike back
    and the blade folds ("lie-down ABORTED").
- **Got up, then fell over.** After the hand-off the policy leaned ~5 deg
  ON the still-planted blade, and the retract snatched it away.
  - Now the blade eases back to 95 deg at 30 deg/s.
  - It retracts (at 120 deg/s) only once the bike is within 3 deg of
    upright.
- **Stop first.** PageDown while moving now lets the policy stop the
  bike before any blade moves.
- **Teleop, as launched, after the fix:** 24/24 down on the chosen side
  and back up. That is 8 tries each for fixes_52M truth, fixes_52M under
  the AHRS model and seed1_72M under the AHRS model, half of them pressed
  while driving and turning.
  - `analysis/swing_sit.py` (now on each policy's own drivetrain): down
    5.8 s at <= 102 deg/s, 7-12 N; up, riding 3.3 s; auto-up 2.8 s.
  - `test_teleop` gained the fixes_52M detailed-drive case. That case
    passes, but the old failure was 2 in 6, so one passing run would
    not have proven it caught the bug.
  - (you: slow but functional)

**Where the time goes, if it should be quicker. [from the defaults in
`swing_sit.py` and the measured totals; no speed-up tried yet]**
- **Down, 5.8 s.**
  - The lower (110 deg of crank at `lower_dps` 30) is ~3.7 s of it.
  - The rest: `limp_s` 1.0 s, `deploy_s` 0.6, `settle_s` 0.4, and the
    kick and the stop.
  - The lower is the knob. On the ideal drive 60 deg/s raised the peak
    roll rate 97 -> 129 deg/s, and 120 deg/s raised it to 211, against
    ~750 for a fall.
- **Up, 3.3 s.**
  - The lift is ~1.5 s: it ramps at `lift_dps` 120 from where the limp
    left the crank (about -70 deg, the far side), so a third of the
    stroke is spent getting to centre.
  - Then the eased fold (`release_dps` 30 to 95 deg, retract only once
    upright).
  - **Cheaper first try:** step the crank to centre at the start of the
    lift, then ramp only the loaded part. 90-150 deg/s all came up; 180
    failed once.

---

## wheelie (someone will ask for it)

Your note:
- original toy did wheelies but only if you held it upright in wheelie
  position for like 3sec before their control policy engaged, this one can
  probably do it with the righting mechanism? I think the rear wheel can get
  enough torque but it won't have the stability on its own (reasoned based on
  the original toy's different modes)

**The numbers on today's model. [measured: `build_model`, default params]**
- Bike mass: 1.248 kg.
- CoM: 0.0868 m ahead of the rear axle and 0.0664 m above it (0.1176 m off
  the floor).
- Balance point: the CoM sits over the rear axle at **~53 deg nose-up**.
- Pendulum: 0.109 m from the axle, so sqrt(L/g) ~ 0.1 s. Fast.
- Clearance: a crude bounding-sphere check found nothing on the chassis
  reaching the floor at 30-60 deg. Check it with real contacts before
  believing it.

**Lifting the nose. [reasoned, rigid wheel, no slip]**
- **Acceleration.** The front lifts when the forward acceleration exceeds
  g x 0.0868 / 0.1176 = **7.2 m/s^2**. That needs 9.0 N of traction (0.46 N m
  at the wheel, before wheel inertia) and mu >= 0.74 on the rollers. **The
  roller friction is the doubtful part.**
- **Torque.** Both drive servos give ~1.07 N m at stall, falling linearly
  to zero at 1.7 m/s. So the torque holds above 0.46 N m only below
  ~1.0 m/s, which is a window of roughly 0.1-0.15 s.
- **A second entry.** Reverse, then brake hard. Decelerating a reversing
  bike is a forward acceleration, and it uses the stored speed instead of
  stall torque.

**Holding it.** It is a unicycle on the omni wheel:
- **Pitch** comes from the hub. This is the 0.1 s pendulum, so the
  drivetrain's slop and detent (the opt-in overlay) matter more here than
  anywhere else.
- **Roll** comes from the diff, the same channel that catches roll now. The
  steer does almost nothing with the front wheel in the air.
- **Pitch sensing.** The policy needs pitch in its obs, which exists as an
  optional block.
- **The policy.** Probably its own policy and env, with the general policy
  before and after: handoff, as in the 180 route 1.

**The righting module cannot lift the nose. [read]** The blades swing
sideways (a roll mechanism). It can carry the roll, though: see "The wing as
a side support" below.

**First test (cheap, no training):** an open-loop launch in sim (full common
drive from rest, then reverse-and-brake) to see whether the nose lifts at
all, with the detailed drivetrain on. If it does not lift, the wheelie needs
hardware changes (traction, gearing, CoM) before any policy.
- yeah but pin the bike's roll so it can't fall over that way
- and does making the bike heavier on the rear help (surely)?

**Ran it, 2026-10-16: the nose lifts, and traction decides it.
[measured: `analysis/wheelie_launch.py`, default params, ideal drivetrain]**
- **Setup.** Roll is pinned by projecting the chassis onto the pitch plane
  each step (y, roll and yaw zeroed). Both drive inputs are commanded to full
  speed, open loop.
- **Correction to the estimate above.** Torque is NOT the limit. The servos
  out-pull the floor: rear friction use (|Ft|/Fn) sits AT the floor's mu the
  whole way. The launch is traction-limited.
- **Full drive overshoots.** At mu 0.9 (`sim.friction_sliding`, a GUESS) the
  nose passes the 53 deg balance point at 0.24 s (launch) or 0.17 s
  (reverse at 0.8 m/s, then brake), and keeps going past vertical. Getting
  up is easy. Stopping AT the balance point is the controller's job.
- **The floor friction threshold** matches the rigid estimate
  (mu >= x/h = 0.74):

    floor mu     launch from rest     reverse then brake
    0.60         no (0.2 deg)         no (0.3 deg)
    0.70         no (2 deg)           no (46 deg)
    0.75         no (45 deg)          yes, 0.25 s
    0.80         yes, 0.31 s          yes, 0.20 s

**Heavier at the rear: yes, through x/h, not through weight.**
- **What matters is the ratio.** The mu a launch needs is (CoM ahead of the
  axle) / (CoM height), so ballast helps only by shrinking that ratio.
- **Location matters more than mass.** 100 g at the axle moves it
  0.74 -> 0.71. The same 100 g 40 mm behind and 40 mm above the axle moves
  it to 0.67, and the launch from rest then succeeds at mu 0.70 (0.28 s),
  where the stock bike fails.
- **Up and back is best.** Height lengthens the pendulum (slower to fall,
  easier to hold), and behind the axle lowers the balance angle
  (53 -> 50 deg).

**The wing as a side support (your answer). [measured: blade-to-floor
distance, `mj_geomDistance`, chassis posed by hand]** Pitching the nose up
lifts the blades, because they sit 0.12 m ahead of the rear axle:

    pitch             0     20    40    53 deg (balance)
    upright, stroke   12    35    59    75 mm  (90 deg stroke)
    upright, stroke    0    23    50    67 mm  (116.9, full)
    lean to touch    0.0  16.0  23.0  25.5 deg (full stroke)
    lean to touch   16.5  28.5  33.5  33.5 deg (90 deg stroke)

- **It works.** At the 53 deg balance point, a bike leaning 25.5 deg onto a
  fully deployed blade stands on the rear wheel plus that blade's toe. The
  blade carries roll. Pitch still needs active balance, about the line
  through the two contacts, which is the same planar problem
  `wheelie_launch.py` flies.
- **Not checked.** Whether anything else on the chassis touches first at
  that lean (only the blades were measured), and the blade dragging on the
  floor if the wheel moves while it is planted.
- **The obvious first wheelie.** Stand the bike up onto blade + rear wheel
  (your toy's "hold it in position"), then let a pitch-only controller take
  it, then lift off the blade with the diff.
  - probably a separate training run for the actual wheelie stability,
    not critical to train now tho, I know the plant can do it
