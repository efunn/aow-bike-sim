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
