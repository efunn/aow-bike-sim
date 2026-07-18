# Omni Wheel Measurement Protocol

Companion data-entry sheet: `omni-wheel-measurements.yaml` (same section
numbering — open it side by side and fill in raw values in mm/g as you measure).

Every number below goes into `config/bike_params.yaml` (replace the matching
`source: GUESS` entry and update its `source:` tag). Rebuild and re-run tests
after entering values: `python -m pytest` — `test_envelope_matches_outer_radius`
cross-checks your cone measurements against your envelope measurement.

Tools: digital calipers, scale with 0.1 g resolution, camera (orthogonal photos
with a ruler in frame), patience for counting gear teeth.

## 1. Wheel envelope (before teardown)

Measure the caliper diameter at three roll phases (with 8 axles, diametrically
opposite contact points are at the same phase, so caliper reading = 2·R(phase)):

| Measurement | YAML key | Value | Notes |
|---|---|---|---|
| Diameter, big-end tip of one roller in contact (max) | — (cross-check) | 102.35 mm | measured 2026-07-16, unloaded |
| Diameter, mid-roller | — (cross-check) | ~102.05 mm | " |
| Diameter, exactly two rollers sharing contact (min) | — (cross-check) | 101.75 mm | " |
| Overall wheel width | `omni_wheel.width` | | |

With roller axles tangential at mounting radius `r_m`, the envelope radius at
axial station `x` along a roller of local radius `ρ(x)` is
`R(x) = sqrt((r_m + ρ(x))² + x²)`. The measured cone taper is consistent with a
*constant-envelope* design (`R(x) ≈ const` along the cone face); the ripple
comes from the coverage gaps instead: at the two-roller phases the rigid wheel
rests on two roller corners **bridging a gap** (chord geometry), which is what
the min-diameter readings measure. Rolling "bumpiness" is real physics the sim
reproduces automatically via the mesh geometry.

**Teardown epilogue (2026-07-17)**: direct measurements settled it —
`r_m = 40.0 mm`, big/small Ø 22.0/19.0, full cone length 7.5 mm. The ridge
check `sqrt((r_m + ρ_big)² + x_big²)` reproduces Ø102.35 within the rollers'
axial play (pair gap floats 8.5–10 mm). The pre-teardown fits (r_m ≈ 38.9,
length ≈ 9.4, little Ø ≈ 20.4) chased a rough "Ø ~24" big-end estimate —
lesson: the envelope solve is only as good as its sloppiest input. Keep it as
a pre-teardown estimator and sanity alarm; trust calipers on the bare part
once available.

### Reconciling envelope vs. component measurements

The envelope values are **cross-check constraints, not model inputs** — the
model is generated from the primitive parameters (mounting radius, cone
dimensions, layout), and the envelope is derived geometry, so small
disagreement with §2–§3 measurements is expected and useful:

1. Cone dimensions (§3) are measurable externally, without teardown.
2. With those plus the envelope values, solve `R_max` = `sqrt((r_m + ρ_big)² +
   x_big²)` for `r_m` — an estimate of the mounting radius **before opening the
   wheel**; the §2 teardown measurement then confirms it.
3. After best-fitting `r_m`, residuals > ~0.1–0.2 mm against all three envelope
   values mean a layout assumption is wrong (axle cant, big-end orientation,
   contact on a cone edge instead of its face). Treat that as a diagnostic to
   resolve, not noise to average away.
4. Calipers measure the wheel unloaded — correct for the rigid-geometry model
   (contact softness handles TPU squish). Effective rolling radius under load
   is separately calibrated in §7 (distance per hub revolution, loaded).

`omni_wheel.outer_radius` in the YAML is the *nominal* radius used for scene
sizing only; set it to the mid-roller value.

## 2. Roller axles (teardown)

| Measurement | YAML key | Value | Notes |
|---|---|---|---|
| Number of axles | `omni_wheel.n_axles` | 8 | confirmed by user |
| Mounting radius (wheel axis → axle axis) | `omni_wheel.axle_mount_radius` | | measure hub center to axle center, or derive from two axle-to-axle distances |
| Axle cant out of tangential direction | `omni_wheel.axle_cant_deg` | | 0 if axles are purely tangential; check with a straightedge/photo |
| Axle diameter | — (record here) | | not used by the sim; useful for spare-part sourcing |

## 3. Truncated cones (measure 2–3, assume identical)

| Measurement | YAML key | Value | Notes |
|---|---|---|---|
| Large-end diameter | `omni_wheel.roller.big_diameter` | ~24 mm | approx (wheel intact); remeasure at teardown |
| Small-end diameter | `omni_wheel.roller.small_diameter` | ~20 mm | approx (wheel intact); remeasure at teardown |
| Main cone length (along axle) | `omni_wheel.roller.length` | 7.5 mm | one cone, not the pair |
| Secondary cone length | — (record; model later) | | back-taper after the big end |
| Secondary cone end diameter | — (record; model later) | | |
| Gap between the pair | `omni_wheel.roller.pair_gap` | | |
| Big ends face each other across the gap? | `omni_wheel.roller.big_end_inward` | | expected yes for a round envelope |
| Mass: one axle + both cones | `omni_wheel.roller.pair_mass` | | |

**Secondary cone — ignore for contact geometry.** The main cone ends in a
near-sharp ridge at max diameter, then a secondary cone tapers back down
steeply and (confirmed by observation) never touches the ground unloaded.
Roller-to-roller handover happens by two-point *bridging* across the coverage
gaps (see §1), entirely on main-cone surfaces/corners, so the rigid model only
needs the main cone. The secondary cone matters only for material deflection
at the ridge (it makes the corner more compliant) — that stays unmodeled;
MuJoCo's soft contact absorbs some of it and the §7 loaded-rolling-radius test
checks whether that's good enough. Expect the rigid model to slightly
overstate the bump at ridge phases. Its mass is captured by the pair weighing.

Practical annoyances observed: rollers have ~0.7 mm of axial play on their
axles (makes length/gap measurement fuzzy; not modeled), and only ~80% of each
roller extends beyond the casing, so intact-wheel diameter readings are
approximate — final diameters come from teardown.

Also note TPU/rubber hardness impression while handling the cones.

## 4. Gear train — count teeth, don't measure angles

Photograph the gearbox at each disassembly step. Gear meshes are never
contact-simulated; every leg of the train reduces to an exact ratio from tooth
counts. Record the raw counts here (provenance + re-derivation), enter only the
derived ratios in the YAML.

**Ring → roller leg** (ring gear → bevel axle → roller):

| Gear | Teeth |
|---|---|
| Ring gear | |
| Bevel gear, ring-gear side | |
| Bevel gear, roller side | |
| Roller gear | |

Derived: `drivetrain.k_roller` = (ring ÷ bevel-ring-side) × (bevel-roller-side ÷ roller).

**Sign is not in the tooth counts** (each bevel mesh can flip direction):
hold the hub, turn the ring gear by hand, note the roller spin direction, and
set the sign of `k_roller` so that positive differential drive crawls in your
chosen positive lateral direction. Mesh friction/efficiency is deliberately not
modeled per-gear — it's lumped into the joint `frictionloss`/`damping` that the
spin-down tests calibrate.

**Input → hub/ring leg — RESOLVED at teardown: the wheel is a bevel
differential.** Two ring gears (side gears), one per side, each mesh all
8 planet ("double bevel") axles mounted in the carrier — which *is* the hub
frame. Rings turning together carry the wheel around (rollers still); rings
turning differentially spin the planets and hence the rollers. The toy's motor
pinions lived outside the wheel and are discarded: the belts drive the two
ring-gear shafts directly, so there are no input-leg gears to count — only the
belt pulleys.

Derived mixing (input A/B ≡ ring gear 1/2 shafts, 1:1; the model's "ring" body
is ring gear 1):

| Quantity | YAML key | Value |
|---|---|---|
| hub rev per input-A rev | `drivetrain.mix_hub_a` | 0.5 (carrier = mean of side gears) |
| hub rev per input-B rev | `drivetrain.mix_hub_b` | 0.5 |
| ring rev per input-A rev | `drivetrain.mix_ring_a` | 1.0 |
| ring rev per input-B rev | `drivetrain.mix_ring_b` | 0.0 |

Signs matter: pick positive = wheel rolling forward / rollers crawling left,
and keep it consistent. The kinematic convention in the sim:
`hub = mix_hub_a·inputA + mix_hub_b·inputB`, same for absolute ring rotation.
Roller-spin sign flips if the wheel is mounted flipped — set
`k_roller`'s sign from the hand-turn test after final assembly.

Belt pulleys (your custom drive): tooth counts of servo-side and wheel-side
pulleys → `drivetrain.belt_ratio` (wheel-side speed ÷ servo speed).

## 5. Masses

Measured per part (better than assembly-level: lumps can be regrouped if the
model topology changes). Model-body lumps are derived by *what rotates
together*:

| Model body | Composition | Mass | YAML key |
|---|---|---|---|
| Roller pair (×8) | 2 cones + roller axle w/ gear | 4.372 g | `omni_wheel.roller.pair_mass` |
| Ring (= ring gear 1) | one side gear | 4.570 g | `omni_wheel.ring.mass` |
| Hub (carrier) | frames + casing + 8 planet axles + ring gear 2 + screws | 75.630 g | `omni_wheel.hub.mass` |

Sanity: 8×4.372 + 4.570 + 75.630 = 115.176 g = measured whole wheel ✓.
Ring gear 2 lumps into the hub (it translates with the wheel); its counter-spin
inertia (~2·10⁻⁶ kg·m²) is neglected. Planet-axle spin inertia likewise lives
nowhere explicit — if the roller flick spin-down (§7) shows more inertia than
the pair alone, add it as roller-joint `armature`.

## 6. Bike as-built (once the chassis exists)

`bike.wheelbase`, `bike.rake_deg`, `bike.fork_offset`, `bike.steering.gear_ratio`,
front wheel radius/width/crown radius/mass, and the position + mass of every
component heavier than ~5 g (servos can use datasheet masses). Positions are in
the chassis frame: origin at the rear axle center, +X forward, +Z up.

## 7. Empirical calibration (wheel-only testbed)

These need the physical rig; the sim's `--variant testbed` model is its mirror,
so results are compared like-for-like. Do them in this order:

1. **Roller flick spin-down** — with the gear train disengaged or a spare
   roller on its axle: flick it, video the decay (slow-mo), fit exponential-ish
   decay → `drivetrain.roller_joint_damping` / `roller_joint_frictionloss`.
   (Constant deceleration ⇒ frictionloss dominates; speed-proportional ⇒ damping.)
2. **Driven-wheel spin-down** — servo spins the hub to a known speed (read the
   Dynamixel encoder), cut torque (torque-disable), log encoder decay →
   hub damping/frictionloss + total drivetrain reflected inertia
   (`drivetrain.input_armature`).
3. **Servo step/ramp response** — command velocity steps through the belt, log
   encoder velocity → validates/replaces the placeholder `actuators.drive_kv`
   and the torque/speed limits.
4. **Incline slide test** — place the (unpowered, locked) wheel sideways on a
   board on your target floor material, tilt until it slides; μ = tan(angle) →
   `sim.friction_sliding`.
5. **Whole-bike CoM** — balance the assembled bike on a straightedge in two
   orientations, or put each wheel on a scale and use moment balance →
   cross-checks the component-position bookkeeping in the YAML.
6. **Loaded rolling radius** — with the wheel bearing roughly its in-bike load,
   roll it exactly one hub revolution and measure distance traveled ÷ 2π. The
   §1 envelope calipering is unloaded; TPU squish makes the effective radius
   under load slightly smaller. Compare against the sim's rolled distance to
   tune contact softness (`solref`/`solimp`) if the gap matters.
