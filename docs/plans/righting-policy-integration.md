# Righting policy integration: a learned get-up beside the drive policy

STATUS (2026-11-03): PLAN. The policy exists and is trained in sim
(`righting_rl_v2`); nothing flies it -- not teleop, not `run_bike`. This
doc is how it gets there, in order, and what to measure on the bench on
the way.

## The design (user, 2026-11-03)

Two policies, each owning its own servos, switched by roll with
hysteresis. Not blended: blending averages two commands neither was
trained on, and the general policy was established (self-righting.md,
four wings runs) not to drive the wings.

| | drive policy (`general_move`) | righting policy (`righting_rl_*`) |
|---|---|---|
| owns | steer, both drives | the crank: Goal Position + Goal Current |
| inputs | its 20 (v6 layout) | 9, mirrored onto the falling side (below) |
| on | riding; back on at a still prop | \|roll\| past `TAKEOVER_DEG` (60) |
| off | at the takeover | a prop held inside Enter's window |

While the righting policy runs, the drives are HELD (zero speed) and the
steer is held where it was when the drive policy let go -- exactly as
swing_sit's lift does now, and as the policy trained. The 60 deg / 15 deg
gap is the get-up itself, so the two never fight over a bike near a
threshold.

The drive policy's recovery limit is 12-24 deg (self-righting.md part 1),
so by 60 the fall is lost; taking over there and not at the bottom is
what the mid-fall starts (below) are for.

## What exists

- `control/righting_spec.py` -- the contract, numpy only (Pi-safe).
  `RightingPolicy(npz).action(obs, side)`.
- `control/righting_env.py`, `train_righting_rl.py`, `config/rl/rl_righting.yaml`.
- `moves/righting_rl_v2.{npz,yaml}` -- 3M steps, starts at rest only.
  `righting_rl_v3` (mid-fall starts, 60 % of episodes) is the next run.

Observation, side frame (`side` = sign of roll at takeover, fixed for the
get-up; every signed entry x side):

| # | entry | sim | bike source |
|---|---|---|---|
| 0 | roll [rad] | truth | AHRS roll (`_roll`) |
| 1 | roll rate [rad/s] | truth | AHRS (`_roll_rate`) |
| 2 | crank [rad] | qpos | Present Position via `righting_pos` -> crank |
| 3 | crank rate [rad/s] | qvel | Present Velocity -- READ, NOT PLUMBED |
| 4 | goal - crank [rad] | stored goal | last Goal Position written |
| 5 | current [A] | servo model | Present Current ("effort") -- READ, NOT PLUMBED |
| 6 | Goal Current / 785 | written | last value written |
| 7 | overload estimate / trip | proxy | NEW: the same proxy on the Pi |
| 8 | elapsed / 4 s | episode step | NEW: clock from takeover |

Action, each in [-1, 1]: crank goal 110 + 70 a deg on the falling side
(clipped to the 116.9 stroke, then `crank_lead.lead_goal` 40 deg);
Goal Current 785 min(1, a + 1); rear differential x 0 (unused).

Sim numbers so far (eval grid, 3 per side, nominal mass, supply 1):

| servo strength | 1.0 | 0.85 | 0.7 | 0.6 |
|---|---|---|---|---|
| scripted lift, rest | over 6 | over 6 | propped 6 | overload 6 |
| scripted lift, mid-fall | over 6 | over 5 | propped 4 | overload 6 |
| v2, rest | propped 6 (1.0 s) | 6 (1.0) | 6 (1.3) | overload 6 |
| v2, mid-fall | propped 6 (0.7-1.2) | 6 | 6 | overload 6 |

v2 per get-up at strength 0.85 (8 rest, 8 mid-fall): 3-25 Goal Current
changes at 5 % steps, Goal Current 427-785, the goal at the 40 deg lead
cap throughout, overload estimate peak 0.15-0.51; one mid-fall start
timed out at 4 s.

## Steps

**1. A pure runtime: `control/righting_policy.py`.** Pi-safe (numpy, no
mujoco), shared by teleop and `run_bike` the way `crank_lead.py` is.
Holds what the env held: `side` (latched at takeover), the takeover
clock, the last goal and Goal Current written, the overload estimate
(`_heat_step`, moved out of the env so both run ONE copy). Per tick:
`step(t, roll, roll_rate, crank, crank_rate, current) -> (goal, counts,
done)` with `done` in {None, "propped", "over", "overload", "timeout"}.
The env then calls it too, so training and flight share the obs code.
Test: the env's episode replayed through it gives the identical obs.
A BACKSTOP outside the policy: the same overload estimate, and at 0.8 of
a trip Goal Current is capped at 400 mA -- under B2's 413 mA floor, where
it never trips -- until it falls back. Training ends an episode at a trip
(penalty as going over), which teaches avoiding it, not a guarantee; on
the bike a trip is torque off, the crank limp and the bike dropped.

**2. swing_sit: a learned lift.** `SwingSit(lifter=...)`: in `lift`, the
goal and current come from the lifter instead of the ramp; "propped"
from the lifter enters `propped` as the scripted lift does, and
everything after it -- Enter, `balance`, the eased release, `retract` --
is unchanged. "over" / "overload" / "timeout" go to `lower` as a failed
lift does. The BLOCKED rule stays as the backstop (a crank dead behind
its goal for 0.5 s): replayed over run 261026-175020 it fired on none
of the 13 battery lifts and caught the wall in 0.3 s (status.md), and the
policy's lead is capped at 40 deg, so it sees what the script's ramp saw. Goal Current writes quantised to 5 %, as now.

**3. The takeover trigger.** New in SwingSit: `takeover(t, roll,
roll_rate)` -- |roll| past `TAKEOVER_DEG` while riding, no stillness
wait -- starts a lift at once when a lifter is set. Auto-up's existing
rule (past 60, still for 0.5 s) stays for the scripted lift.

**4. Teleop.** `run_drive --teleop --righting-policy righting_rl_v2`;
PageUp and auto-up use it, `--righting-policy none` keeps the script. HUD:
lifter active, side, overload estimate, done reason. Teleop flies the
detailed drivetrain where the policy trained on the ideal one; the drives
are held throughout, so this should not matter -- check it does not.
Hand-back: teleop may engage the drive policy AT the prop automatically
(SwingSit `hand_off=True`'s `balance`) or wait for Enter (the bike's
`hand_off=False`); the user's design is automatic, so teleop does that
first.

**5. Offline replay of bike runs (no flight).** Feed the 11-02 and 10-26
records -- AHRS roll and rate, crank, Present Current where recorded --
through the runtime and plot what it WOULD have commanded beside what the
script did. Catches sign, unit and side-latch errors before a motor moves.
The records carry the righting servo's Present Current and PWM (the
11-02 overload analysis read them from there).

**6. run_bike.** GOAL CURRENT INTO THE SYNCWRITE first. Today
`set_righting_current` is its own request/response write and reads
Current Limit before it -- two replies per change, and a lost reply
raises an error no caller in run_bike catches (reads ride out up to 3
bad frames, `_read_servos`; Goal Position goes out in the SyncWrite,
which has no reply). The bench lost one at 400 mA (B2, rc -3001 then
-3002); the bike logged -3002 on 10-24. The policy writes Goal Current 3-25
times a get-up. The block has 6 bytes spare (22 of 28); Goal Current
needs 2, and then it goes every tick with no reply to lose.
Then plumb Present Velocity and Present Current for the
righting servo into the state (`bike_bus.read_state` already decodes
both; only `righting_pos` reaches `run_bike`). `onboard.yaml`
`righting_policy:` names the export; `scripts/bike.sh sync` carries the
npz. Telemetry: lifter active, side, overload estimate, policy goal and
current. Keep Enter as the hand-back on the bike first (status quo);
automatic once flown. The fault path is unchanged: a servo fault ends the
sequence and cuts.

**Auto-up and the controls (user, 11-03: auto-up with the takeover on the
bike).** Today the bike never gets up by itself (no auto-retry, status.md);
with a lifter it would: past `TAKEOVER_DEG` while engaged -> the lifter ->
propped -> the drive policy (Enter at first, automatic once flown). The
station's controls as they stand (hw/ground.py, the mirror's pad):

| | key | pad | does |
|---|---|---|---|
| engage / re-arm | Enter (r) | Y | the drive policy takes an upright, still bike; re-arms after a cut |
| disengage | Backspace | B | the drive policy out, its three servos limp -- the RIGHTING SERVO KEEPS HOLDING |
| get up | u / PageUp | Start, d-pad up | from lying or any cut / standby |
| lie down / park | z / PageDown, p / Home | X / d-pad down, d-pad left | |
| quit | q | | heartbeat stops; 1 s later the bike cuts and waits |

THERE IS NO KILL FOR EVERYTHING: nothing on the station takes the
righting servo's torque off (disengage, quit, link loss, a pack cut and a
servo fault all leave the crank holding, by design -- a prop drops if it
goes limp). With an auto-up that can start a lift by itself, add one: a
KILL that also sets the righting Goal Current to 0 and disarms auto-up,
from which Enter (an upright bike) or get-up (lying) re-engages as from
standby. Its key, and whether disengage itself should disarm auto-up,
are the user's call.

**7. On the bike.** In order: lie-down + learned get-up from rest
(PageDown, PageUp), both sides, battery and power brick; then mid-fall
takeover from a hand-held lean released past 60; then auto hand-back.
Compare to the scripted lift's 11-02 record side by side.

## Bench tests -- the module on the Mac (id 104, XC330, fw 53, mode 5)

Read 2026-11-03, torque off: 12.0 V, 30 C, no hardware error, P700 D1400,
Goal Current 910 (the power-on default; the bike writes 785).

**B1. Stall: duty and current at Goal Current steps.** Crank blocked
(blade against a fixed block) with the goal 40 deg past it, as the
policy's lead holds it; Goal Current 200 -> 785 in steps, ~0.5 s each,
cooling between; log Present PWM, Present Current, voltage, temperature.
ANSWERS: the model says duty ~0.8 at a 785 mA stall; the bike pinned PWM
885 at 670-750 mA. Which is right sets where `servo_strength` 1.0
actually sits, i.e. which column of the table above is the bike. Not
destructive: well short of the overload trips (1.7 s+).

**B2. Overload: time to trip.** Same stall held at 785 (then 600) until
Hardware Error bit 5 (overload); log time, current, temperature; reboot.
ANSWERS: input 7 is a GUESS shaped on three bike trips; this replaces it
with a measured curve. Costs a trip and heat per point; one point per
level, cooled to < 35 C between.

**B2 RESULT (2026-11-03, `analysis/servo/righting_overload.py`, run
traces/servo_stall/righting_b2_261103-025839 and one whose log was lost
to a dropped reply; - side, held between the servo and the left blade,
40 deg lead, 11.8-11.9 V, 34-39 C; every trip reported as OVERLOAD, bit
5 -- not temperature, whose limit is 70 C).**

| Goal Current [mA] | 785 | 700 | 600 | 500 | 450 |
|---|---|---|---|---|---|
| current reached [mA] | 730 | 700 | 600 | 500 | 450 |
| PWM [%] | 100 | 99 | 92 | 87 | 84 |
| time to trip [s] | 1.98 | 2.30 | 3.62 | 6.98 | slipped first |

**t_trip = 0.645 A s / (I - 0.413 A)**, within 0.5 s at every point; it
puts the bike's manual trips (670-750 mA) at 1.9-2.5 s against the
measured 1.7 and 2.3. It is CURRENT and time, not PWM: 600 and 500 mA
tripped at 87-92 % PWM. Below ~410 mA it should never trip -- untested
under 500 (450: ~18 s by the fit, 420: ~95). Not measured: how fast it
recovers between pushes (every trip was cleared by a reboot), which is
what pumping and back-to-back get-ups need. The env's overload estimate
(input 7) now uses this law; its recovery (`cool_s` 10) is still a
GUESS. Three 785 holds slipped by hand inside 0.06 s.

**B2b RESULT (2026-11-03, `analysis/servo/righting_recovery.py`, run
traces/servo_stall/righting_b2b_261103-032247; - side, 36-39 C).** 700 mA
for 1.5 s (2/3 of a trip by the law), then held still at <= 300 mA (it
drew ~80-85 mA), then 700 mA until it tripped:

| gap [s] | 1 | 2 | 4 | 8 |
|---|---|---|---|---|
| second trip [s] | 2.34 | 2.58 | 2.62 | 2.61 |

With no recovery it would trip at 0.75 s; fresh it trips at 2.30. So the
charge is ALL gone within 1 s below the floor -- the overload behaves
like a timer that resets, not a slow leak. The env's `cool_s` is now 1.0
(an upper bound). Gaps under 1 s defeated a hand hold (runs -033035,
-033246: the crank ran 16-33 deg in the preload and slipped at the push)
-- so the BIKE'S OWN RECORDS answer it instead. Replayed over the righting
servo's recorded current in all 77 bike runs, the trip law with a drain
of a full trip per `cool_s`:

| cool_s | real overloads predicted (of 7) | early by | false | missed |
|---|---|---|---|---|
| 0.25 / 0.5 | 7 | 0.04-0.14 s | 1 | 0 |
| 1.0 | 7 | 0.04-0.39 s | 1 | 0 |

The 10-26 pumping run is the one that separates them (0.5: 0.14 s early;
1.0: 0.39; 2.0: 0.9; no drain: 15 s). The false one is 261026-191739,
the run that rocked on the wing at ~650 mA; it errs safe. The env's
`cool_s` is 0.5. A bench law that predicts every bike trip to 0.14 s is
the strongest check this module has had. A first version of this test
went LIMP in the gap with a hand still on it and measured nothing (run
-031753: the crank sprang 16-25 deg each time).

**B3. The runtime against the real servo, no bike.** Step 1's runtime
with a SYNTHETIC roll trace (a recorded fall, played back), driving the
real crank: checks signs, the side latch, units of Present Current and
Velocity, and the 5 % Goal Current writes end to end. The blades swing
free, so nothing lifts; what it shows is that the commands are the ones
the sim would send.

**B4. Bus timing.** Goal Position + Goal Current writes and the read
block at 100 Hz for 10 s on the U2D2: a lower bound (one servo here, four
on the bike). Only if B3 shows slips.

B1 first: it is short and it moves the most.

**B1 RESULT (2026-11-03, `analysis/servo/righting_stall.py`, runs
traces/servo_stall/righting_b1_261103-023932, -024459 and -025143; held
by hand, 100 mA preload, 40 deg lead, 11.8-11.9 V, 33-35 C).** The +
side with both blades pinched; the - side slipped that way at 400-500
mA (each direction loads mostly one blade), and reached 785 held between
the servo and the left blade (-025143).

| Goal Current [mA] | 200 | 300 | 400 | 500 | 600 | 700 | 785 |
|---|---|---|---|---|---|---|---|
| Present PWM, + | 42 | 54-57 | 70-71 | 77-83 | 89 | 96 | **100** |
| Present Current, + | 200 | 300 | 400 | 500 | 600 | 700 | **756** |
| Present PWM, - | 43 | 58 | 67-70 | 82 | 89 | 96 | **100** |
| Present Current, - | 200 | 300 | 400 | 500 | 600 | 700 | **756** |
| model duty | 19 | 30 | 40 | 51 | 62 | 72 | 78 |

The two directions agree to 1-4 points of PWM at every level: the
servo pushes the same both ways, so the bike's weaker LEFT lifts are not
the servo. PWM pins at 100 % before the current reaches a 785 cap: 756 mA at
11.8 V, the bike's 670-750 mA at 11.3-12.2 V. So above ~750 Goal Current
buys nothing at stall on this pack. Fit (15 unsaturated points, PWM x
V/12 against current): slope 1.10 /A against the model's 1.04 (k/ts), but
an OFFSET of +0.22 where the model has -0.02; it bends over at the top
(the fit puts 100 % at 691 mA). Unresolved: whether that offset is real
duty or how Present PWM reports in mode 5 (righting_servo.py: it does not
track the applied duty there). SETTLED BY THE BIKE'S OWN RECORDS (84 runs,
10-2x to 11-0x, 1.06 M ticks): PWM x V/12 = 0.11 + 1.58 (tau/ts) + 0.49
(w/w0) where the model has 0 + 1 + 1 -- about double the model at stall,
HALF its rise with speed. No winding does both, and the no-load top speed
matches the datasheet: the register is not the duty in this mode. Not a
modelling question; Present PWM in mode 5 is uncalibrated, and nothing
should key on it (the overload does not: B2). CORRECTED: an earlier
draft tied this to the sim's ~0.15 N m lift shortfall; that shortfall is
measured through CURRENT (status.md row 4), the load side, and is
unrelated.

## Open

- **The weakest servo.** Nothing lifts at strength 0.6 in sim, scripted
  or learned. Whether the bike is there is B1's question.
- **The lying pose.** The sim lies at ~70 deg roll, crank ~66 past
  centre; the bike at 74 and 26.5-28 (status.md row 4). The policy's rest
  starts are the sim's.
- **Over the top on the bike.** The policy never went over in sim; the
  scripted lift went over in sim and not on the bike. Over-the-top on
  the bike ends the episode as a fall the OTHER way, which the takeover
  then catches -- worth deciding whether a second get-up is allowed.
- **The 22-wide alternative** (one policy, v6's observation plus the
  crank, all four channels, fall to riding) was built and set aside: 0 of
  40 random episodes got up, and it would learn balance from scratch.
