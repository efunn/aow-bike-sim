"""Drive harness: python -m aow_sim.run_drive [--view | --teleop].

Headless (default):
  1. Straight sprints (fwd/back, config accel): cruise quality, braking
     distance, cross-track, survival.
  2. Accel sweep at v_max until failure -> max clean accel/decel.
  3. Binary-search envelopes (per the target baselines): tightest tracked
     circle and tightest stop-from-circle, both directions, at 0.5 m/s.
  4. Fastest-circle sweep at the tightest radius (+ margin).

--view: scripted demo (sprint, one circle lap, stop).
--teleop (macOS: mjpython): ↑/↓ speed ±0.25 m/s (through zero into reverse),
  ←/→ heading nudge ±15°, C / V circle left/right (R=0.8 m), Space stop.
"""

from __future__ import annotations

import argparse
import copy

import mujoco
import numpy as np

from .build_model import build_model, load_params
from .control import DriveController, run
from .control.balance import extract_state
from .control.linearize import settle_upright

UPRIGHT_LIMIT_DEG = 60.0


def _fresh(model, eq_qpos):
    data = mujoco.MjData(model)
    data.qpos[:] = eq_qpos
    a = np.deg2rad(0.5)
    data.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(model, data)
    return data


class _Roll:
    def __init__(self, c):
        self.c, self.deg = c, []

    def __call__(self, dd):
        self.deg.append(np.degrees(extract_state(dd, self.c._ref_pos).roll))

    @property
    def ok(self):
        r = np.abs(self.deg)
        return bool(np.all(np.isfinite(r)) and r.max() < UPRIGHT_LIMIT_DEG)


def sprint_scenario(model, params, eq_qpos, v_target: float) -> dict:
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    c.set_speed(v_target)
    roll = _Roll(c)
    ys, vs = [], []

    def rec(dd):
        roll(dd)
        s = extract_state(dd, c._ref_pos)
        ys.append(dd.qpos[1])
        vs.append(s.v_lon)

    t_ramp = abs(v_target) / params["control"]["drive"]["accel"]
    run(model, data, c, t_ramp + 2.0, on_step=rec)
    cruise_v = float(vs[-1])          # sampled at end of cruise, pre-brake
    x_brake = data.qpos[0]
    c.command_stop()
    run(model, data, c, t_ramp + 2.5, on_step=rec)
    return {
        "v_target": v_target,
        "cruise v": round(cruise_v, 3),
        "max |roll| [deg]": round(float(np.max(np.abs(roll.deg))), 2),
        "max cross-track [m]": round(float(np.max(np.abs(ys))), 3),
        "brake+settle [m]": round(abs(float(data.qpos[0] - x_brake)), 3),
        "final v": round(float(vs[-1]), 3),
        "survived": roll.ok,
    }


def accel_sweep(model, params, eq_qpos, accels=(1.5, 2.5, 4.0, 6.0, 9.0)):
    print("\naccel sweep (0 -> v_max -> 0):")
    best = 0.0
    for a in accels:
        p = copy.deepcopy(params)
        p["control"]["drive"]["accel"] = a
        res = sprint_scenario(model, p, eq_qpos, p["control"]["drive"]["v_max"])
        ok = res["survived"] and res["max |roll| [deg]"] < 20
        print(f"  {a:4.1f} m/s^2: roll={res['max |roll| [deg]']:5.2f}  "
              f"cross={res['max cross-track [m]']:.3f}  "
              f"brake={res['brake+settle [m]']:.2f} m  {'ok' if ok else 'FAIL'}")
        if ok:
            best = a
        else:
            break
    print(f"  -> max clean accel/decel: {best:.1f} m/s^2")
    return best


def circle_ok(model, params, eq_qpos, radius, direction, v=0.5,
              stop_test=False) -> tuple[bool, float]:
    """One ramped lap on the circle; optionally command a stop mid-circle.
    Returns (success, mean radius error)."""
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    c.command_circle(data, radius, direction)
    c.set_speed(v)
    roll = _Roll(c)
    radii = []

    def rec(dd):
        roll(dd)
        radii.append(float(np.linalg.norm(dd.qpos[:2] - c._center)))

    lap_t = (abs(v) / params["control"]["drive"]["accel"]
             + 2 * np.pi * radius / abs(v))
    run(model, data, c, lap_t, on_step=rec)
    tail = np.array(radii[len(radii) // 3:])
    err = float(np.mean(np.abs(tail - radius)))
    ok = roll.ok and err < max(0.10 * radius, 0.03)
    if ok and stop_test:
        c.command_stop()
        roll2 = _Roll(c)
        run(model, data, c,
            abs(v) / params["control"]["drive"]["accel"] + 3.0,
            on_step=roll2)
        ok = roll2.ok and abs(roll2.deg[-1]) < 5.0
    return ok, err


def tightest_search(model, params, eq_qpos, direction, stop_test=False,
                    lo=0.2, hi=1.0, tol=0.02, v=0.5) -> float:
    """Binary search the smallest radius that succeeds (assumes monotone)."""
    ok, _ = circle_ok(model, params, eq_qpos, hi, direction, v=v,
                      stop_test=stop_test)
    if not ok:
        return float("nan")
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        ok, _ = circle_ok(model, params, eq_qpos, mid, direction, v=v,
                          stop_test=stop_test)
        if ok:
            hi = mid
        else:
            lo = mid
    return hi


def turn_ok(model, params, eq_qpos, v, rate, delta_deg=90.0) -> bool:
    """command_heading turn at a forced slew rate: upright + tracks."""
    p = copy.deepcopy(params)
    p["control"]["drive"]["yaw_slew_sharp"] = rate
    p["control"]["drive"]["turn_rate_margin"] = 10.0   # cap = rate, not margin
    data = _fresh(model, eq_qpos)
    c = DriveController(p, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    c.set_speed(v)
    run(model, data, c, 2.0)
    psi0 = c._psi
    c.command_heading(data, np.deg2rad(delta_deg))
    roll = _Roll(c)
    run(model, data, c, np.deg2rad(abs(delta_deg)) / rate + 3.0, on_step=roll)
    err = abs(np.degrees(c._psi - psi0) - delta_deg)
    return roll.ok and err < 10.0


def turn_rate_envelope(model, params, eq_qpos,
                       speeds=(0.4, 0.8, 1.2, -0.4, -0.6, -1.0, -1.2)):
    """Binary-search the max clean 90-degree turn rate per speed."""
    print("\nturn-rate envelope (90-degree command_heading, tol 0.1 rad/s):")
    for v in speeds:
        lo, hi = 0.3, 4.0
        if not turn_ok(model, params, eq_qpos, v, lo):
            print(f"  v={v:+.1f}: < {lo} rad/s (FAIL at floor)")
            continue
        while hi - lo > 0.1:
            mid = 0.5 * (lo + hi)
            if turn_ok(model, params, eq_qpos, v, mid):
                lo = mid
            else:
                hi = mid
        r_turn = abs(v) / lo
        print(f"  v={v:+.1f}: max rate {lo:.2f} rad/s  (turn radius ~{r_turn:.2f} m)")


def uturn_width(model, params, eq_qpos, v=0.8) -> float:
    """180-degree turn at speed: swept lateral width (practical sharpness)."""
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    c.set_speed(v)
    run(model, data, c, 2.0)
    c.command_heading(data, np.deg2rad(180))
    roll = _Roll(c)
    ys = []

    def rec(dd):
        roll(dd)
        ys.append(dd.qpos[1])

    run(model, data, c, 6.0, on_step=rec)
    return float(np.ptp(ys)) if roll.ok else float("nan")


def flip_scenario(model, params, eq_qpos, direction=1) -> dict:
    """180-degree swap-ends flip from standstill. Reports upright, final yaw
    error, the peak and final center excursion (in wheelbases), and settle."""
    L = params["bike"]["wheelbase"]
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    psi0 = c._psi
    T = c.command_flip(data, direction)
    C0 = c._flip_center.copy()
    roll = _Roll(c)
    devs = []

    def rec(dd):
        roll(dd)
        cc, ss = np.cos(c._psi), np.sin(c._psi)
        devs.append(float(np.linalg.norm(
            dd.qpos[:2] + (L / 2) * np.array([cc, ss]) - C0)))

    run(model, data, c, T + 5.0, on_step=rec)
    tail = np.abs(roll.deg)[-int(0.5 / model.opt.timestep):]  # roll.deg already in deg
    return {
        "direction": direction,
        "duration [s]": round(T, 2),
        "yaw err [deg]": round(np.degrees(c._psi - psi0) - 180 * np.sign(direction), 1),
        "peak excursion [L]": round(max(devs) / L, 2),
        "final excursion [L]": round(devs[-1] / L, 2),
        "max |roll| [deg]": round(float(np.max(np.abs(roll.deg))), 2),
        "settled RMS [deg]": round(float(np.sqrt(np.mean(tail**2))), 2),
        "survived": roll.ok,
    }


def flick_scenario(model, params, eq_qpos, direction=1, name="flick") -> dict:
    """Optimized two-arc 180 flick from standstill. Reports upright, final yaw,
    the side-to-side lateral envelope (the bounded axis; x is free), x-shift,
    and settle. Requires moves/<name>.yaml (run optimize_flick.py first)."""
    L = params["bike"]["wheelbase"]
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    psi0 = c._psi
    p0 = data.qpos[:2].copy()
    yaw0 = psi0
    T = c.command_flick(data, direction, name=name)
    roll = _Roll(c)
    lats = []

    def rec(dd):
        roll(dd)
        d = dd.qpos[:2] - p0
        lats.append(abs(-np.sin(yaw0) * d[0] + np.cos(yaw0) * d[1]))

    run(model, data, c, T + 4.0, on_step=rec)
    d = data.qpos[:2] - p0
    x_shift = float(np.cos(yaw0) * d[0] + np.sin(yaw0) * d[1])
    tail = np.abs(roll.deg)[-int(0.5 / model.opt.timestep):]
    return {
        "move": name,
        "direction": direction,
        "duration [s]": round(T, 2),
        "yaw err [deg]": round(np.degrees(c._psi - psi0) - 180 * np.sign(direction), 1),
        "lateral env [L]": round(max(lats) / L, 2),
        "x shift [L]": round(x_shift / L, 2),
        "max |roll| [deg]": round(float(np.max(np.abs(roll.deg))), 2),
        "settled RMS [deg]": round(float(np.sqrt(np.mean(tail**2))), 2),
        "survived": roll.ok,
    }


def fastest_circle(model, params, eq_qpos, radius,
                   vs=(0.5, 0.75, 1.0, 1.2)) -> float:
    best = 0.0
    for v in vs:
        ok, _ = circle_ok(model, params, eq_qpos, radius, +1, v=v)
        if ok:
            best = v
        else:
            break
    return best


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=None)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--teleop", action="store_true")
    ap.add_argument("--hockey", action="store_true",
                    help="add the ball-shot stick panels + ball (teleop key 1 fires it)")
    args = ap.parse_args()
    params = load_params(args.params)
    model = build_model(params, variant="full", hockey=args.hockey)
    eq = settle_upright(model)

    if args.teleop:
        _teleop(model, params, eq.qpos, hockey=args.hockey)
        return
    if args.view:
        _view_demo(model, params, eq.qpos, hockey=args.hockey)
        return

    v_max = params["control"]["drive"]["v_max"]
    print("straight sprints:")
    for vt in (0.8, v_max, -0.5, -v_max):
        res = sprint_scenario(model, params, eq.qpos, vt)
        print("  " + "  ".join(f"{k}={v}" for k, v in res.items()))
    max_acc = accel_sweep(model, params, eq.qpos)

    turn_rate_envelope(model, params, eq.qpos)
    w = uturn_width(model, params, eq.qpos)
    print(f"\nU-turn at 0.8 m/s: swept width {w:.2f} m")

    print("\ncircle envelopes at 0.5 m/s (binary search, tol 2 cm):")
    for direction, tag in ((+1, "CCW"), (-1, "CW")):
        r_track = tightest_search(model, params, eq.qpos, direction)
        r_stop = tightest_search(model, params, eq.qpos, direction,
                                 stop_test=True)
        print(f"  {tag}: tightest tracked R = {r_track:.2f} m; "
              f"tightest stop-from-circle R = {r_stop:.2f} m")
    print("\nreverse circle envelopes at -0.5 m/s:")
    for direction, tag in ((+1, "CCW"), (-1, "CW")):
        r_track = tightest_search(model, params, eq.qpos, direction, v=-0.5)
        print(f"  {tag}: tightest tracked R = {r_track:.2f} m")
    r_ref = 0.5 if np.isnan(r_track) else max(r_track + 0.1, 0.4)
    v_best = fastest_circle(model, params, eq.qpos, r_ref)
    print(f"\nfastest circle at R = {r_ref:.2f} m: {v_best:.2f} m/s")

    print("\n180-degree swap-ends flip (standstill, crawl front-pivot):")
    for direction in (+1, -1):
        res = flip_scenario(model, params, eq.qpos, direction)
        print("  " + "  ".join(f"{k}={v}" for k, v in res.items()))
    print("  (peak excursion ~1 L is intrinsic: exact center-spin is a delta=90"
          " singularity)")

    from .control.flick import MOVES_DIR
    print("\n180-degree two-arc flick (lateral bounded, x free):")
    variants = (("flick", "trajopt reverse-first"),
                ("flick_fwd", "trajopt forward-first"),
                ("flick_rl", "RL policy (closed-loop)"))
    for move, label in variants:
        if (MOVES_DIR / f"{move}.yaml").exists():
            res = flick_scenario(model, params, eq.qpos, +1, name=move)
            print(f"  [{label}] " + "  ".join(f"{k}={v}" for k, v in res.items()))
        else:
            how = ("python -m aow_sim.train_flick_rl" if move == "flick_rl"
                   else f"python -m aow_sim.optimize_flick"
                        f"{' --reverse-first' if move == 'flick' else f' --name {move}'}")
            print(f"  [{label}] no moves/{move}.yaml — run `{how}`")
    print(f"\nsummary: v_max ±{v_max} m/s straight OK, max accel {max_acc:.1f} m/s^2")


def _view_demo(model, params, eq_qpos, hockey=False):
    # Uses the passive viewer (launch_passive via teleop_loop), not the managed
    # mujoco.viewer.launch app — the latter spins up its own _Simulate and is
    # unreliable under mjpython on macOS ("_Simulate ... unknown exception").
    from .interactive import teleop_loop
    from .control.flick import MOVES_DIR
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    if hockey:
        if not (MOVES_DIR / "ball_rl.yaml").exists():
            raise SystemExit("no moves/ball_rl.yaml — run "
                             "`python -m aow_sim.train_ball_rl`")
        # Ball-shot-only demo: fire the RL ball move from standstill.
        plan = [(1.0, lambda d: (_reset_ball(model, d, params),
                                 c.command_ball(d, name="ball_rl")))]
        intro = "viewer demo: ball-shot (RL) from standstill"
    else:
        plan = [
            (1.0, lambda d: c.set_speed(0.8)),
            (4.0, lambda d: c.command_circle(d, 0.8, +1)),
            (14.0, lambda d: c.command_stop()),
            (17.0, lambda d: c.command_flip(d, +1)),
        ]
        if (MOVES_DIR / "flick.yaml").exists():
            plan.append((22.0, lambda d: c.command_flick(d, +1)))
        intro = "viewer demo: sprint 0.8 m/s, circle R=0.8, stop, flip, flick"

    stage = {"i": 0}
    overlay_on = [True]

    def step(m, d):
        if stage["i"] < len(plan) and d.time >= plan[stage["i"]][0]:
            plan[stage["i"]][1](d)
            stage["i"] += 1
        c.step(m, d)

    teleop_loop(model, data, step, lambda k: None, intro, "aow_sim.run_drive",
                draw=lambda scn, m, d: _overlay(scn, m, d, c, overlay_on))


def _overlay(scn, model, data, c, on):
    """Draw heading (cyan), velocity (yellow, length ∝ speed) and the
    controller reference (green, heading + length ∝ target speed) as arrows
    above the chassis, using MuJoCo user_scn geometry. Toggled by `on`."""
    scn.ngeom = 0
    if not on[0]:
        return
    base = data.body("chassis").xpos.copy()
    base[2] += 0.12
    vscale = 0.15   # meters of arrow per m/s

    def arrow(heading, length, rgba):
        if scn.ngeom >= scn.maxgeom:
            return
        tip = base + np.array([np.cos(heading) * length,
                               np.sin(heading) * length, 0.0])
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW,
                            np.zeros(3), np.zeros(3), np.zeros(9),
                            np.asarray(rgba, np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.006, base, tip)
        scn.ngeom += 1

    R = data.body("chassis").xmat.reshape(3, 3)
    heading = float(np.arctan2(R[1, 0], R[0, 0]))
    v = data.qvel[:2]
    speed = float(np.linalg.norm(v))
    arrow(heading, 0.15, [0.2, 0.8, 1.0, 1.0])                            # heading
    if speed > 1e-3:
        arrow(float(np.arctan2(v[1], v[0])), vscale * speed,
              [1.0, 0.9, 0.1, 1.0])                                       # velocity
    h_ref, s_ref = c.viz_reference(data)
    arrow(h_ref, 0.10 + vscale * abs(s_ref), [0.2, 1.0, 0.3, 1.0])        # reference


def _reset_ball(model, data, params):
    """Re-park the ball at its bike-frame start pose (hockey model only), so a
    fresh shot can be attempted. No-op if the model has no ball."""
    try:
        jid = int(model.body("ball").jntadr[0])
    except Exception:
        return
    q, v = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
    ball = params["hockey"]["ball"]
    data.qpos[q:q + 2] = ball["start"]
    data.qpos[q + 2] = ball["radius"]
    data.qpos[q + 3:q + 7] = [1, 0, 0, 0]
    data.qvel[v:v + 6] = 0.0


def _teleop(model, params, eq_qpos, hockey=False):
    from .interactive import teleop_loop

    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    pending = []
    state = {"v": 0.0}
    overlay_on = [True]

    def on_key(keycode):
        pending.append(keycode)

    def step(m, d):
        while pending:
            k = pending.pop(0)
            if k == 265:      # up arrow
                state["v"] = min(state["v"] + 0.25, c.profile.v_max)
                c.set_speed(state["v"])
            elif k == 264:    # down arrow
                state["v"] = max(state["v"] - 0.25, -c.profile.v_max)
                c.set_speed(state["v"])
            elif k in (263, 262):   # left / right arrow: slewed turn (any speed)
                c.command_heading(d, np.deg2rad(15.0 if k == 263 else -15.0))
            elif k in (ord("6"), ord("7")):   # circle left / right
                c.command_circle(d, 0.8, +1 if k == ord("6") else -1)
            elif k in (ord("8"), ord("9"), ord("3")):
                # flick: 8 trajopt reverse-first, 9 trajopt forward-first, 3 RL
                state["v"] = 0.0
                move = {ord("8"): "flick", ord("9"): "flick_fwd",
                        ord("3"): "flick_rl"}[k]
                try:
                    c.command_flick(d, +1, name=move)
                except FileNotFoundError:
                    print(f"no moves/{move}.yaml yet")
            elif k == ord("1"):     # ball-shot (RL): reset the ball, then fire
                state["v"] = 0.0
                _reset_ball(m, d, params)
                try:
                    c.command_ball(d, name="ball_rl")
                except FileNotFoundError:
                    print("no moves/ball_rl.yaml yet — run "
                          "`python -m aow_sim.train_ball_rl`")
            elif k == ord("0"):     # re-park the ball at its start pose
                _reset_ball(m, d, params)
            elif k == ord("4"):     # crawl front-pivot 180 (in-place variant)
                state["v"] = 0.0
                c.command_flip(d, +1)
            elif k == ord("5"):     # stop
                state["v"] = 0.0
                c.command_stop()
            elif k == ord("2"):     # toggle reference overlay
                overlay_on[0] = not overlay_on[0]
        c.step(m, d)

    ball_help = "\n  1 ball-shot (RL)   0 reset ball" if hockey else ""
    # Number keys + arrows: MuJoCo's viewer binds every letter A-Z (F=force
    # display, etc.), so letters would double up. Number keys 0-9 are free; 4/5
    # toggle (empty) geom groups harmlessly; arrows are free while unpaused.
    teleop_loop(model, data, step, on_key,
                "teleop (number keys — MuJoCo's viewer owns the letters):\n"
                "  ↑/↓ speed ±0.25   ←/→ turn ±15°   6/7 circle L/R\n"
                "  8/9 flick (trajopt rev/fwd)   3 flick (RL)   4 flip   "
                "5 stop   2 toggle overlay" + ball_help,
                "aow_sim.run_drive",
                draw=lambda scn, m, d: _overlay(scn, m, d, c, overlay_on))


if __name__ == "__main__":
    import mujoco.viewer  # noqa: F401

    main()
