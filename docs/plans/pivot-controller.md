# Pivot Controller (crawl pivot about the front contact)

## Context

The stationary LQR balance controller is committed and settles toy-like (1 oscillation) after the yaw-rate-damping fix. Next performance goal: **pivot in place** — turn the bike's heading while balanced, the toy's "steer, then fall into the pivot with rear-wheel movement." User decision: implement the **crawl pivot** (rear wheel crawls an arc around the stationary front contact; differential drive does the work; steering stays small) — the direct extension of the current machinery. The steer-90° pivot (about the rear contact, minimum-diameter trick) is a recorded follow-up once heading command exists.

This introduces heading control as *reference tracking* on the existing LQR — not new feedback design. Key lesson to preserve (decisions doc, 2026-07-17): yaw-position feedback must stay light or the loop rings against the steer clamp; the pivot therefore generates a *feasible* moving reference with feedforward, and the existing gain matrix only cleans up residuals.

## Design

### `src/aow_sim/control/pivot.py`

**`YawProfile`** — trapezoidal yaw-rate profile: given Δψ, `yaw_rate`, `yaw_accel` (config), produces `(psi_ref(t), psi_dot_ref(t))`, accel-limited ramp up/cruise/ramp down. Handles either sign; supports |Δψ| > 180° (unwrapped).

**`PivotController(LQRBalance)`** — reuses the identified model, gain matrix K, steer clamp, ZOH base, and `lat_gain` from `control/balance.py` unchanged.

- `command_pivot(delta_yaw)` — sets up a profile starting from the current unwrapped yaw and captures the pivot center: `p_front = p_rear + R(psi)·[wheelbase, 0]` (world frame, from chassis freejoint + `bike.wheelbase`).
- Reference at time t: `psi_ref`, `psi_dot_ref` from the profile; rear-contact position ref `p_ref = p_front − R(psi_ref)·[wheelbase, 0]`; body-frame lateral velocity ref `v_lat_ref = wheelbase · psi_dot_ref`; optional lean feedforward `roll_ref = lean_ff · psi_dot_ref² · r_com / g` toward the pivot center (config gain, default on; r_com ≈ CoM distance from front contact — compute from the model's subtree CoM once at init).
- Feedback: build the same 8-state vector as `LQRBalance._compute` but as *errors*: `e_lat` from `p_ref` (rotated into the current yaw frame, as today), `roll − roll_ref`, unwrapped `psi − psi_ref`, `v_lat − v_lat_ref`, `psi_dot − psi_dot_ref`; then `u = −K x_err + [d_ff, 0]` with `d_ff = v_lat_ref / lat_gain` and a config `ff_gain` (default 1.0, tunable — front-patch torsional friction may need a bit more or less than kinematic).
- Unwrapped yaw: accumulate raw yaw increments each tick (no atan2 wrap jumps); supports multi-turn commands.
- Profile complete → automatically degenerates to station-keeping (references constant at the final pose) — the controller *is* the balance controller between pivots.

### Config (`config/bike_params.yaml`)

```yaml
control:
  pivot:
    yaw_rate: 1.5      # rad/s cruise (crawl-speed ceiling is ~4 rad/s; balance-limited in practice)
    yaw_accel: 4.0     # rad/s^2
    lean_ff: 1.0       # centripetal lean feedforward gain (0 disables)
    ff_gain: 1.0       # crawl feedforward scale
```

### `src/aow_sim/run_pivot.py` (CLI harness, mirrors run_balance.py)

- Headless default: scenario table — pivot +90°, −90°, +180° at config rate; then a yaw-rate sweep (0.5 → 4 rad/s) on ±90° until failure. Metrics per scenario: completion time, heading error 1 s after profile end, max |roll|, **pivot-center wander** (max distance of the front contact from its start — the "in place" measure), survived.
- `--view`: scripted demo (+180°, hold 2 s, −180°) in the viewer via `mujoco.set_mjcb_control`.

### Tests (`tests/test_pivot.py`)

- ±90° pivot at default rate: survives, final heading within 5°, front-contact wander < ~8 cm, ends balanced (roll settles < 1° RMS in the last second).
- 180° pivot: same criteria, looser wander bound.
- Profile unit check: YawProfile reaches Δψ exactly, rate/accel limits respected.
- Existing 15 tests stay green.

### Docs

README: `run_pivot` lines in quickstart. Decisions doc: short section — pivot = reference tracking + feedforward on the balance LQR, why feasible references preserve the light-yaw-weight lesson; steer-90 pivot recorded as follow-up.

## Files

- New: `src/aow_sim/control/pivot.py`, `src/aow_sim/run_pivot.py`, `tests/test_pivot.py`
- Touched: `config/bike_params.yaml` (pivot block), `src/aow_sim/control/__init__.py` (export), README, `docs/plans/mujoco-modeling-decisions.md`

## Verification

1. `pytest` — new pivot tests + existing 15.
2. `python -m aow_sim.run_pivot` — metrics table: ±90° and 180° complete with small wander; rate sweep reports the achievable envelope (expect balance-limited well below the 4 rad/s crawl ceiling).
3. `python -m aow_sim.run_pivot --view` — watch the +180/−180 demo: front wheel roughly planted, rear sweeping, bike leaning slightly into the pivot, clean settle at each end.
4. Tuning pass if needed: `ff_gain` first (feedforward accuracy), then `yaw_accel` (aggressiveness), leaving the balance weights untouched.

## Out of scope (recorded)

Steer-90° pivot about the rear contact (minimum-diameter mode — needs large-steer modeling work); teleop-style continuous heading/rate command interface; combining pivot with forward drive (drift/arc maneuvers); PD cascade updates (legacy, per user).