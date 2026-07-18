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
