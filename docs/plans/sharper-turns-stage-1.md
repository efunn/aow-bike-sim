# Sharper Turns Beyond the Linear Range (stage 1: feedforward-carried turns)

## Context

Teleop/command turns are stable but gentle: line-mode turns clamp *absolute* steer at 15° and cap turn rate at `0.7·v·tan(15°)/L` (≈0.86 rad/s at 0.8 m/s → R ≈ 0.93 m). Meanwhile **circle mode already operates far outside the linear range** — the envelope search tracked R ≈ 0.30 m at 0.5 m/s (≈34° absolute steer) — because it clamps only the ±15° feedback *correction around* the kinematic feedforward, keeping the *deviation from equilibrium* inside the identified model's validity while the equilibrium itself moves. Stage 1 unifies heading turns with that proven structure and measures the new envelope, forward **and reverse** (user: reverse matters equally). Stage 2 — identification about *turning* equilibria (2D speed × curvature schedule) — is recorded with explicit trigger criteria, expected to be needed mainly for sharp reverse (reversed caster degrades the straight-line model fastest).

## Design (stage 1)

### 1. `control/drive.py` — line-mode turn changes (`command_heading` at speed)

- **Steer feedforward unclamped from 15°**: `steer_ff = steer_ff_gain · atan(ψ̇_ref·L / v_lon)` clipped to new `steer_ff_max_deg` (default 45°) instead of `steer_limit`. Feedback stays clamped ±`steer_limit_deg` *around* it — exactly circle mode's structure.
- **Lean reference in atan form** for large-lean consistency with circle mode: `roll_ref = −lean_ff · atan(v_lon·ψ̇_ref / g)`.
- **Turn-rate ceiling raised** to `turn_rate_margin · |v|·tan(steer_ff_max)/L` (margin default 0.7; at 0.8 m/s → ≈2.8 rad/s, R ≈ 0.29 m — matching the circle envelope). Keep `min(yaw_slew_sharp, …)` with a new `yaw_slew_sharp` (default 3.0 rad/s) so standstill/crawl blending is unchanged.
- **Reverse scaling knob**: `reverse_turn_scale` (default 1.0) multiplying the cap when `v_lon < 0` — the tuning handle if validation shows reverse can't run as hard; findings recorded either way.
- Unchanged: trapezoid slew + lag governor, arc mode at low speed, line→arc decay handoff, `_int_lat`.

### 2. `run_drive.py` — envelope measurement (the stage-2 gate)

- **Turn-rate envelope sweep**: for v ∈ {0.4, 0.8, 1.2, −0.4, −0.8}: binary-search the max clean rate for a ±90° `command_heading` turn (success = upright + |heading err| < 10° after settle) → table of ψ̇_max(v) and equivalent turn radius v/ψ̇. This quantifies exactly what stage 1 bought and where it stops.
- **Reverse circle envelopes**: run `tightest_search` (tracking + stop-from-circle) at v = −0.5 both directions — `circle_ok`/`command_circle` are sign-generic in principle; validate and fix if not.
- **U-turn scenario**: 180° at 0.8 m/s — report swept lateral width (the practical sharpness metric).

### 3. Tests (`tests/test_drive.py`) — conservative fixed points

- Forward sharp turn: 90° at 0.8 m/s with the cap allowing ≥2.0 rad/s — completes upright, heading err < 6°.
- Reverse turn: 90° at −0.5 m/s at whatever rate validates with margin (set during implementation).
- Reverse circle: R = 0.8 m at −0.5 m/s tracks (radius err < 15%).
- Existing 33 tests stay green.

### 4. Config (`control.drive`)

```yaml
steer_ff_max_deg: 45.0   # feedforward steer ceiling (feedback still ±steer_limit around it)
turn_rate_margin: 0.7    # fraction of the kinematic ceiling used by command_heading
yaw_slew_sharp: 3.0      # rad/s absolute ceiling for at-speed turns
reverse_turn_scale: 1.0  # extra cap factor for v < 0 (tune down if reverse validates worse)
```

### 5. Docs — decisions doc section

Stage-1 rationale (equilibrium-following with clamped deviation), measured envelope table, and **stage-2 trigger criteria**: build the (speed × curvature) turning-equilibrium schedule if (a) reverse ψ̇_max < ~50% of forward at the same |v|, or (b) forward turn radius stays >1.5× the circle-mode envelope at matched speed, or (c) teleop feel still inadequate. Stage-2 sketch recorded: `settle_circling(v, κ)` (project onto the leaned, yawing equilibrium each step), ID in the path frame, 2D gain + equilibrium-map interpolation replacing the analytic feedforwards.

## Files

- `src/aow_sim/control/drive.py` — line-turn ff/clamp/cap changes
- `src/aow_sim/run_drive.py` — turn-rate envelope sweep, reverse circle searches, U-turn metric
- `tests/test_drive.py` — 3 new fixed-point tests
- `config/bike_params.yaml` — 4 new `control.drive` keys
- `docs/plans/mujoco-modeling-decisions.md` — stage-1 section + stage-2 triggers

## Verification

1. `pytest` — 33 existing + 3 new.
2. `python -m aow_sim.run_drive` — envelope tables: expect forward ψ̇_max ≈ 2–3 rad/s at 0.8 m/s (R ≈ 0.3–0.4 m, near circle-mode); reverse numbers whatever they are, honestly reported against the stage-2 triggers.
3. `mjpython -m aow_sim.run_drive --teleop` — the felt test: ←/→ turns should now visibly carve at speed; J/L U-turns tight; reverse turns snappier or (if capped) at least clean.

## Out of scope (recorded)

Stage 2 turning-equilibrium identification (triggered by the criteria above); crawl-assisted drift turns (differential + steer combined beyond kinematic arcs — agility phase); steer-90° minimum-diameter mode.