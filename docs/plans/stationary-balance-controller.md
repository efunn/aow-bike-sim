# Stationary Balance Controller (baseline)

## Context

The AOW bike model is geometry-complete with measured wheel parameters, a bench-verified differential drivetrain (hub = mean of ring gears; opposed inputs → pure lateral crawl at the rear contact), and a passing test suite. Next milestone from the performance goals: **balance while standing still, minimizing rear-wheel drift**.

Physics: a stationary upright bike is an inverted pendulum in roll (pendulum frequency ≈ √(g/h) ≈ 1.5 Hz for CoM height ~0.1 m). The rear wheel's driven lateral crawl is the "cart" that moves the support line under the CoM; steering repositions the front contact (the toy visibly coordinates both — steer disturbances get caught by rear crawl, pivots "fall into" the rear's motion). User decisions: **LQR primary + PD cascade reference**, **ground-truth state** (sensor-only estimation is a later phase), **200 Hz control** with zero-order hold (YAML parameter; hardware can reach ~500 Hz later).

## Control mapping (shared by both controllers)

`ctrl = [drive_a, drive_b, steer]`. Decompose drives into common mode `c = (a+b)/2` → hub spin (fore/aft roll) and differential `d = a − b` → `ring_rel = d/2` → roller spin `= k_roller·d/2 = 1.2·d` → lateral contact velocity `≈ 1.2·d·ρ_eff` with `ρ_eff ≈ 10.25 mm` (mean cone radius, derived from `omni_wheel.roller` params, not hardcoded). Helper `mix(common, diff) -> (a, b)` lives with the controllers.

## Implementation

New package `src/aow_sim/control/`; script `src/aow_sim/run_balance.py`; config block `control:` in `config/bike_params.yaml`; scipy added to deps.

### 1. `control/balance.py` — two controllers, one interface

Common protocol: `reset(model, data)`, `step(model, data) -> ctrl` called every physics step; internally updates only every `1/rate_hz` (ZOH between updates); saturates to actuator ctrlranges. Ground-truth state extraction: roll & roll rate from chassis quat/qvel, rear-contact lateral/longitudinal position & velocity from chassis freejoint (rear axle ≈ chassis origin), steer angle/rate from joints.

**PDCascade** (transparent reference, rear-crawl only):
- Inner: roll PD → commanded lateral contact velocity `v_y` → `d = v_y / (1.2·ρ_eff)`.
- Outer (slow): rear lateral position P → roll setpoint bias, clamped to a few degrees (`max_roll_setpoint_deg`).
- Weak longitudinal P on x → common mode (stops fore/aft wander).
- Steer held at 0 by its position actuator.
- Starting gains derived from the pendulum model (ω ≈ 9–10 rad/s), tuned in sim.

**LQRBalance** (primary; discovers steer+crawl coordination on its own):
- Gain `K` from `control/linearize.py` (below); feedback `u = −K·dx` where `dx = [mj_differentiatePos(qpos, qpos_eq); qvel]` (handles the freejoint quaternion correctly), applied at 200 Hz.

### 2. `control/linearize.py` — numeric linearization + DLQR

- Settle the full model upright for ~0.5 s (contacts converge; training wheels OFF), take that as equilibrium (`qpos_eq`, ctrl = 0).
- `mujoco.mjd_transitionFD` (centered) → single-physics-step `A, B` (40×40, 40×3 for nv = 20).
- Lift to the control period (n = timestep ratio ≈ 25): `A_h = Aⁿ`, `B_h = Σ AⁱB`.
- Discrete LQR via `scipy.linalg.solve_discrete_are`. Q built by joint/DOF name (heavy: roll, roll rate; light: lateral pos/vel, yaw, steer; ~zero: everything else), R small on drives, moderate on steer. Weights live in the YAML `control.lqr` block.
- Contingency (known risk): FD around a contact-rich equilibrium can be noisy → if K misbehaves, project to a reduced state (roll, roll rate, y, ẏ, steer, steer rate) before solving. Note in code, only implement if needed.

### 3. `run_balance.py` — CLI harness

- `--controller lqr|pd`, `--view`, `--params`.
- Headless (default): scenario suite — settle from 2–3° initial roll, lateral push impulses at the CoM (both signs, increasing magnitude), report roll RMS, max |y| drift, survival time, and the largest recovered push. Prints a metrics table; this is the regression baseline for later controllers.
- `--view`: interactive viewer via `mujoco.set_mjcb_control` (works with the managed viewer on macOS; ZOH handled inside the callback) — drag-perturb the bike and watch it catch itself.

### 4. Config additions (`config/bike_params.yaml`)

```yaml
control:
  rate_hz: 200
  pd: {roll_kp, roll_kd, y_kp, x_kp, max_roll_setpoint_deg}
  lqr: {q_roll, q_roll_rate, q_ypos, q_yvel, q_yaw, q_steer, r_drive, r_steer}
```
(plain scalars — tuning knobs, not measurements)

### 5. Tests (`tests/test_balance.py`)

For each controller: (a) from 2° roll, stays |roll| < 15° for 10 s and ends near upright; (b) rear lateral drift bounded (|y| < ~0.15 m over the run); (c) recovers a calibrated lateral push (magnitude chosen with margin during implementation); (d) LQR-specific: the gain matrix actually uses the steer column (nonzero, sane sign) — the toy-like coordination emerging is a designed-for outcome, verify it. Keep total test sim time ~30 s (runs in seconds at 5 kHz).

### 6. Docs

Short "Balance controller" section in README (how to run) and a decisions note in `docs/plans/mujoco-modeling-decisions.md` (architecture, control rate, ground-truth caveat, the roll-ripple disturbance from the 16-cone envelope being a realism feature the controller must tolerate).

## Verification

1. `pytest` — new balance tests plus the existing 10 stay green.
2. `python -m aow_sim.run_balance` (headless) — metrics table shows: settles from 3° tilt, survives ≥ 10 s with sub-cm RMS roll wobble, bounded drift, recovers a nontrivial push; LQR ≥ PD on every metric.
3. `python -m aow_sim.run_balance --controller lqr --view` — drag-perturb in the viewer: bike visibly crawls the rear wheel to catch itself; verify steering participates (small counter-steer motions) for the LQR.
4. Inspect printed K rows for steer to confirm the steering/rear conjunction the user described in the toy.

## Out of scope (recorded as later phases)

Sensor-only estimation (AHRS + encoder integration), Dynamixel control-loop emulation with latency/noise, pivot-in-place maneuver controller (steer setpoint + yaw tracking on top of LQR), the remaining performance goals (drive straight, circles), RL.