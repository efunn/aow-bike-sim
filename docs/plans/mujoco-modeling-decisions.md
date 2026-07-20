# MuJoCo Modeling Decisions

Answers to the open questions in `prelim-architecture.md`, and the rationale
behind the builder in `src/aow_sim/build_model.py`. Status: implemented and
tested with placeholder parameters (2026-07).

## Approach: parameter file + procedural builder (option 3)

- **Not OnShape→URDF.** URDF cannot express joint/tendon equality couplings
  (the gear train), contact parameters, or actuators — all essential here.
  CAD-modeling gears would also be wasted effort: gear teeth are never
  contact-simulated; the entire drivetrain reduces to tooth-count ratios
  enforced by constraints. CAD remains optional later, purely for visual meshes.
- **Not prose→hand-written MJCF.** 8 coupled roller bodies hand-authored in XML
  is unmaintainable across measurement revisions.
- **Chosen:** `config/bike_params.yaml` (every parameter tagged with units and
  source) → Python builder on MuJoCo's `mjSpec` API → model. Contact meshes
  (truncated cones, crowned tire) are generated procedurally as vertex clouds;
  MuJoCo's convex hull is exact for these shapes up to tessellation.

## Kinematic structure

- 8 roller-axle bodies on the hub, each with one hinge (tangential axis) and
  two truncated-cone mesh geoms (big ends facing each other across the gap).
- `roller_ring` is a **child of the hub** with a coaxial hinge, so its joint
  angle is the ring-vs-hub *relative* angle — which makes each roller coupling
  a simple two-joint equality: `roller_i = k_roller × ring_rel`. Rigid gearing
  assumed (backlash ignored); "all 8 rollers rotate identically" is a test.
- The toy gearbox is pure kinematics: two input-shaft joints, and two **fixed
  tendons with tendon-equality constraints** mapping them to hub and absolute
  ring rotation through a 2×2 mixing matrix.
- **Teardown (2026-07-17) resolved the mixing: the wheel is a bevel
  differential.** Two side ("ring") gears mesh all 8 planet axles mounted in
  the carrier (= hub); the belts drive the two ring-gear shafts directly.
  Hence hub = (ring1 + ring2)/2, i.e. `mix_hub_* = 0.5/0.5`,
  `mix_ring_* = 1.0/0.0` (the model's ring body is ring gear 1; ring gear 2's
  mass lumps into the hub, its counter-spin inertia ~2e-6 kg·m² neglected).
  Equal inputs → pure roll; opposed inputs → pure lateral crawl.
  `k_roller = (48/12)·(12/20) = 2.4`; sign flips if the wheel mounts flipped —
  confirm at final assembly.
- No gear geometry exists anywhere in the model.

## Contact and solver

| Question from prelim-architecture | Decision |
|---|---|
| Truncated cone geom? | MuJoCo has **no cone primitive** — procedurally generated convex revolved meshes (32 segments, configurable). |
| `condim=6` rolling friction? | **No.** Bearing/drivetrain losses live in the joints (`frictionloss` + `damping`, calibrated by spin-down tests), not the contact. `condim=4` (torsional) is **on by default**: more physical for every maneuver, one extra constraint row. Its coefficient is the calibration risk — overestimates add fake yaw resistance — so it's a YAML parameter. |
| Friction cones/solver? | Elliptic cones, Newton solver, `impratio=10`. |
| Integrator/timestep | `implicitfast`, 2e-4 s (placeholder-validated: stable at rest, no jitter, ~0.1 mm penetration; ~11× realtime single-threaded on an M-series laptop). |
| Contact filtering | Only {rollers, tire, training wheels} ↔ floor collide: floor is contype 1/conaffinity 2, dynamic geoms 2/1, all else 0/0 — dynamic geoms can never collide with each other. |
| Inertias | Weigh parts; geoms carry measured masses and MuJoCo derives inertia from shape. Non-contact parts are primitives at correct poses — no CAD needed. |
| Light-roller stiffness risk | `armature` on input shafts (physically: reflected rotor/gear inertia) plus equality `solref = (0.005, 1)`; stable in tests. |

## Actuators and sensors

- Drive (2× XC430-W150 @ 12 V): velocity actuators on the input shafts;
  `ctrlrange` ±(no-load speed × belt ratio), `forcerange` ±(stall ÷ belt
  ratio). Placeholder for Dynamixel velocity-PI emulation at the real control
  rate (later phase, after testbed system-ID).
- Steering (XC330-T181): position actuator, `forcerange` from stall × gear
  ratio, **joint unlimited and no ctrlrange** — continuous 360°+ steering is a
  design requirement (minimum-diameter turns, tricks).
- Sensors: gyro + accelerometer + framequat at the AHRS site (TM151 mirror),
  jointpos/jointvel on servo joints (encoder mirrors). Noise/latency: later
  phase.

## Known simplifications (revisit when they matter)

- Umbilical cable forces are not modeled.
- Dynamixel firmware control loops approximated by ideal MuJoCo servo actuators.
- Gear backlash and belt compliance ignored (rigid couplings).
- Cone edges are sharp; real TPU bumpers may be rounded (mesh generator can add
  a chamfer if teardown shows one).
- Fast approximation models (anisotropic-friction capsule à la Ekumen's LeKiwi
  omni-base, or a ballbot-style reduction) deferred until this reference model
  is validated — note the capsule trick only handles *passive* rollers, so an
  active-roller approximation will need actuated lateral contact velocity.

## Verified behaviors (placeholder parameters, `tests/test_model.py`)

Equal drive → straight roll near rigid-rolling speed; differential drive →
lateral rear-wheel crawl (the AOW signature); no support → falls like a bike;
at rest with training wheels → no jitter/drift; all 8 rollers phase-locked at
exactly `k_roller` × ring angle.

## Balance controller (baseline, 2026-07-17)

Two stationary balance controllers in `src/aow_sim/control/`, both at 200 Hz
with zero-order hold (`control.rate_hz`), ground-truth state (sensor-only
estimation is a later phase), saturating to actuator ctrlranges:

- **PD cascade** (`PDCascade`): roll PD → rear-crawl velocity, slow outer loop
  on lateral drift → roll setpoint, weak longitudinal P → common mode, steer
  held straight. Key structural lesson: a velocity-source base cannot stabilize
  the pendulum from roll feedback alone — the crawl command must be
  **relative to the current base velocity** (acceleration-style law,
  `v_cmd = v_lat + PD(roll)`).
- **LQR** (`LQRBalance`): DLQR on an **identified** reduced lateral model
  (`[e_lat, roll, yaw, steer]` + rates; inputs `[differential, steer]`),
  fit by least squares over one-control-period rollouts at finite amplitude
  (R² > 0.997 on all states). It uses steering for balance on its own — the
  steer/rear-crawl coordination observed in the toy emerges from the optimum.

**Why not `mjd_transitionFD`**: the infinitesimal FD Jacobian at standstill
linearizes the friction cone in its *sticking* regime and underestimates the
drive→lateral-velocity response by ~2× at real crawl amplitudes; LQR gains
designed on it are unstable on the true plant. Finite-amplitude system ID is
the standing approach for contact-dominated linearizations in this project.

Baseline metrics (placeholder chassis params, `run_balance.py`): from a 3°
lean both controllers settle to <0.1° RMS wobble with <10 cm total drift and
recover a ~3–4 N × 0.1 s lateral push. Regression-tested in
`tests/test_balance.py`, including "LQR actually uses steering".

**Geometry-change lesson (2026-07-17, rake 24°→15° / fork offset →0, i.e.
trail 5.9→13.4 mm)**: the LQR redesigns itself automatically, but with more
trail its closed loop drifted out of the identified region (47° steer, 64° yaw
excursions) and limit-cycled. Two structural fixes, now defaults: a hard steer
clamp (`control.lqr.steer_limit_deg`, keeps the loop inside the region where
the linear model is valid) and a real yaw weight (`q_yaw` — crawl-induced yaw
must be regulated, not left to wander; it spirals with the lateral loop).
Multi-period/finite-state system ID was tried and is *worse* (R² collapses on
velocity states — dynamic contact regimes don't fit one linear model). After a
significant geometry change, re-run `run_balance.py` for both controllers and
re-check the effort weights (`r_drive` trades authority vs. saturation margin;
its optimum moved with the geometry).

**Oscillatory-settling lesson (2026-07-17)**: heading swings are *intrinsic*
to rear-crawl station-keeping (yaw ≈ crawl distance / wheelbase, since the
bike pivots about the front contact) — heavily weighting yaw *position* makes
the loop fight unavoidable swings, ride the steer clamp (~40% engagement), and
ring for many oscillations. The fix that matched the toy's 1–2-oscillation
settling: **light `q_yaw`, heavy `q_yaw_rate`** — damp rotation,
accept where it ends up pointing. Settling went from 6 oscillations / 7 s to
1 oscillation / 3.2 s at unchanged push robustness. Also tried and rejected:
10-state model with common-mode input (ID quality drops, wanders) and loose
`q_ypos` (slower, more oscillation). PD cascade is legacy — tuning effort
targets the LQR only. (Weights later nudged to `q_yaw` 8 / `q_yaw_rate` 12 for
pivot tracking accuracy; push settling stays at ~2 oscillations. `q_yaw` ≳ 15
is where ringing returns.)

## Crawl pivot (`control/pivot.py`, 2026-07-17)

Heading control is **reference tracking on the balance LQR**, not new feedback:
a trapezoidal yaw profile (`control.pivot.yaw_rate/yaw_accel`) generates a
feasible moving reference — yaw ramp, rear-contact position on the arc around
the front contact, matching crawl velocity — plus differential feedforward
(`d_ff = v_lat_ref / lat_gain`). The balance gain matrix only corrects
residuals, which is what preserves the anti-ringing weight design. Between
pivots the references are constant, so `PivotController` *is* the stationary
balance controller.

Physics note: for a pivot about the front contact, the CoM's *centripetal*
acceleration points along body-X (longitudinal — no lean needed); only the
*tangential* acceleration `psi_ddot · r_com` is lateral, so the lean
feedforward (`lean_ff`) acts during profile ramps and is zero at cruise.

Measured envelope (placeholder chassis): ±90° in 1.4 s, 180° in 2.5 s;
heading error < 1° after settle, front-contact wander **< 1 cm** at every rate
up to the ~4 rad/s crawl ceiling; max lean ~4°. `tests/test_pivot.py` guards
profile correctness and closed-loop pivots. Recorded follow-ups: steer-90°
pivot about the rear contact (minimum-diameter mode, needs large-steer
modeling), teleop-style continuous heading commands, pivot + forward drive.

## Driving: gain-scheduled LQR (`control/drive.py`, 2026-07-18)

Balance at speed is steering-dominated and speed-dependent, so the
finite-amplitude identification runs at a mirrored grid of forward speeds
(`control.drive.speed_grid`, ±1.2 m/s max ≈ 70% of the no-load ceiling) and
gains are linearly interpolated by measured speed. v = 0 recovers the
stationary controller; negative speeds capture the reversed-caster regime —
the identified steer/roll gain flips sign backward, with zero hand-modeling.
Line and circle path modes layer references + feedforward on top, pivot-style.

Hard-won lessons, in the order they bit:

1. **The model's lateral-velocity state is the cross-track *rate*** (world-
   frame v_y in the ID frame, containing v·sin(heading error)) — not the
   body-frame lateral slip velocity. They coincide at standstill (why balance
   and pivot never noticed); feeding the body-frame value at 0.8 m/s loses
   the dominant v·ψ term and destabilizes cruise. Symptom: slow (~0.7 s
   doubling) roll divergence with steering pinned at the clamp.
2. **Stopping must re-anchor where the bike halts** (`command_stop`), at the
   moment v_ref reaches zero — anchoring at stop initiation pulls the bike
   back by its braking distance; not re-anchoring at all sends it sprinting
   back to the old anchor.
3. **At balance, roll — not steering — sets the turning radius**
   (R = v²/(g·tanφ), so 0.2° of roll residual ≈ 10% of radius at 0.5 m/s).
   A steer-side integral proved this by doing nothing. Two-part fix:
   `lean_ff = 0.85` (the ideal-bicycle lean formula over-leans because the
   crowned tire's contact patch shifts into the turn) plus a slow integral
   lean-trim on cross-track error (`ki_lat = 0.05`; larger values hunt).
   Result: radius bias ≤ 1 cm at R ∈ {0.5, 0.8, 1.0}, both directions.
4. Steer clamp in circle mode applies to the feedback correction *around*
   the kinematic feedforward `atan(L/R)` — tight circles need large absolute
   steer; model validity bounds the deviation from equilibrium.

Baselines (placeholder chassis, `run_drive.py` / `tests/test_drive.py`):
straight sprints to ±1.2 m/s with <6 cm cross-track and ~0.5 m braking
distance; envelope numbers (tightest circle, tightest stop-from-circle, max
accel, fastest circle) reported by the harness — see its output, they will
move as the model gets calibrated. Teleop: `mjpython -m aow_sim.run_drive
--teleop`.

## Teleop turning: `command_heading` (2026-07-18)

The first teleop mapped ←/→ to instantaneous ±15° heading-reference steps —
unusable (jolts at speed, near-inert at standstill). `command_heading` now
picks the mechanism by speed:

- **|v| < 0.3 (arc mode)**: the pivot recipe — positional reference on the arc
  around the front contact. The *position* feedback is what brakes yaw
  momentum at the end of a turn; a heading-only reference lets the bike spin
  20° past the stop and diverge (yaw–crawl positive feedback outside the
  identified region). Tight lag governor (8°); handoff back to line mode only
  once yaw momentum is spent.
- **At speed (rotating carrot)**: the line heading slews under the bike
  (trapezoid, `yaw_slew`/`yaw_accel`), re-anchoring each tick, with lean
  feedforward `v·ψ̇/g` and **kinematic steer feedforward** `atan(ψ̇L/v)` —
  whose sign flips in reverse; without it, backing turns diverge because weak
  yaw feedback never finds the opposite-signed steer. Turn-rate ceiling =
  what the steer clamp can kinematically deliver, `0.7·v·tan(steer_limit)/L`
  (commanding more over-leans the bike relative to the achievable arc and it
  falls inward).
- A line-mode turn that decays below 0.25 m/s hands off to arc mode
  mid-slew (stop-while-turning case).

Verified (tests + scenarios): standstill ±90° and chained 4×90° lap, ±90° at
0.8 m/s, ±30° slalom, ±45° in reverse, accelerate-mid-turn, stop-mid-turn —
all upright with ≤4° heading error.

## Sharper turns beyond the linear range (stage 1, 2026-07-18)

`command_heading` at speed is now **feedforward-carried** like circle mode:
the kinematic steer `atan(ψ̇L/v)` may grow to `steer_ff_max_deg` (45°) — it
moves the operating point; the ±15° feedback clamp bounds only the *deviation*
from it, which is what the identified model's validity actually constrains.
Lean reference in atan form; turn-rate ceiling = `turn_rate_margin ×
v·tan(steer_ff_max)/L`.

Measured envelope (placeholder chassis, `run_drive.py`): 90° turns at up to
**3.9 rad/s** at 0.8–1.2 m/s (turn radius ≈ 0.2–0.3 m, matching the circle
envelope — the production cap keeps ~30% margin under that); U-turn at
0.8 m/s sweeps only **0.64 m**; reverse circles at −0.5 m/s track down to
**R = 0.21 m** (tighter than forward!).

**Reverse keep-out band** (the important finding): straight cruise diverges in
[−0.88, −0.78] m/s — a slow, monotonic caster/weave coalescence pocket that
*no* gain choice stabilizes at our steer/crawl authority (verified: K from
other speeds, re-designs with different weights, stiffer steer servos — all
fail; neighbors ±0.1 m/s are clean). Turning is unreliable across the wider
(−1.15, −0.72); only identified grid speeds outside it (−0.6, −1.2) turn
cleanly. `control.drive.reverse_avoid_band` snaps dwell targets past the band
(transiting during ramps is fine). Expect the physical bike to have such a
pocket too, elsewhere (it scales with steering stiffness — the real XC330 is
much stiffer than the modeled servo); recalibrate on hardware.

**Stage-2 triggers** (turning-equilibrium 2D identification): met *only inside
the reverse band* — forward and outer-reverse turning matches the circle
envelope, so stage 2 is deferred until the band matters in practice or
hardware calibration moves it somewhere inconvenient. Sketch when needed:
`settle_circling(v, κ)` projection settling, path-frame ID about the turning
equilibrium, 2D gain + equilibrium-map interpolation replacing analytic
feedforwards.

## Agility: 180-degree swap-ends flip ("flip" mode, 2026-07-19)

Goal (`docs/plans/agility-turn-180-move.md`): rotate 180 about the *midline*,
lateral deviation <= 0.5 L "at any point". Implemented as a `DriveController`
mode: pre-steer the front to ~90 deg (frees it to roll laterally), hold while
the rear crawl feedback tracks a radius-L/2 circle about the captured center
and balances roll, hub closes a slow longitudinal center loop, then hand back
to line mode. Result: stable (max roll ~2 deg), completes 180 (+/-1 deg), ends
within ~0.3 L of the start, settles clean. Both directions; teleop key **F**.

**Key finding — exact center-spin is a kinematic singularity.** A non-scrubbing
front wheel forces the instantaneous center of rotation onto the front-axle
line; that line passes through the midpoint *only at steer = 90 deg exactly*
(at 45 deg the ICR is still 0.35 L from center, near the front). So for any
steer < 90 the bike pivots near the **front contact**, and the passive front
wheel's spin-up inertia collapses even the delta=90 case back to a front-pivot
transient. Consequence: the flip **bulges out to ~1 L mid-spin** (front-pivot)
and the hub loop only reels the center back by the end. This also kills the
concept doc's *swept* reverse-flick (steer 0->180): sweeping through low steer
angles is a front-pivot for most of the move. The peak-excursion goal (<=0.5 L
throughout) is therefore **not reachable with feedback tracking**; it needs
**trajectory optimization** (co-optimized steer/hub/crawl trajectory + TVLQR)
to actively drive the front around in sync — recorded as the escalation, not
built. What ships is a genuinely useful fast in-place 180 that ends centered;
`tests/test_drive.py::test_flip_completes` guards upright + completion + final
centering (peak excursion intentionally not asserted).
