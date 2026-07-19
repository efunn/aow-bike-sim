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

    lap_t = v / params["control"]["drive"]["accel"] + 2 * np.pi * radius / v
    run(model, data, c, lap_t, on_step=rec)
    tail = np.array(radii[len(radii) // 3:])
    err = float(np.mean(np.abs(tail - radius)))
    ok = roll.ok and err < max(0.10 * radius, 0.03)
    if ok and stop_test:
        c.command_stop()
        roll2 = _Roll(c)
        run(model, data, c, v / params["control"]["drive"]["accel"] + 3.0,
            on_step=roll2)
        ok = roll2.ok and abs(roll2.deg[-1]) < 5.0
    return ok, err


def tightest_search(model, params, eq_qpos, direction, stop_test=False,
                    lo=0.2, hi=1.0, tol=0.02) -> float:
    """Binary search the smallest radius that succeeds (assumes monotone)."""
    ok, _ = circle_ok(model, params, eq_qpos, hi, direction, stop_test=stop_test)
    if not ok:
        return float("nan")
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        ok, _ = circle_ok(model, params, eq_qpos, mid, direction,
                          stop_test=stop_test)
        if ok:
            hi = mid
        else:
            lo = mid
    return hi


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
    args = ap.parse_args()
    params = load_params(args.params)
    model = build_model(params, variant="full")
    eq = settle_upright(model)

    if args.teleop:
        _teleop(model, params, eq.qpos)
        return
    if args.view:
        _view_demo(model, params, eq.qpos)
        return

    v_max = params["control"]["drive"]["v_max"]
    print("straight sprints:")
    for vt in (0.8, v_max, -0.5, -v_max):
        res = sprint_scenario(model, params, eq.qpos, vt)
        print("  " + "  ".join(f"{k}={v}" for k, v in res.items()))
    max_acc = accel_sweep(model, params, eq.qpos)

    print("\ncircle envelopes at 0.5 m/s (binary search, tol 2 cm):")
    for direction, tag in ((+1, "CCW"), (-1, "CW")):
        r_track = tightest_search(model, params, eq.qpos, direction)
        r_stop = tightest_search(model, params, eq.qpos, direction,
                                 stop_test=True)
        print(f"  {tag}: tightest tracked R = {r_track:.2f} m; "
              f"tightest stop-from-circle R = {r_stop:.2f} m")
    r_ref = 0.5 if np.isnan(r_track) else max(r_track + 0.1, 0.4)
    v_best = fastest_circle(model, params, eq.qpos, r_ref)
    print(f"\nfastest circle at R = {r_ref:.2f} m: {v_best:.2f} m/s")
    print(f"summary: v_max ±{v_max} m/s straight OK, max accel {max_acc:.1f} m/s^2")


def _view_demo(model, params, eq_qpos):
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    stage = {"i": 0}
    plan = [
        (1.0, lambda d: c.set_speed(0.8)),
        (4.0, lambda d: c.command_circle(d, 0.8, +1)),
        (14.0, lambda d: c.command_stop()),
    ]

    def cb(m, d):
        c.step(m, d)
        if stage["i"] < len(plan) and d.time >= plan[stage["i"]][0]:
            plan[stage["i"]][1](d)
            stage["i"] += 1
    mujoco.set_mjcb_control(cb)
    print("viewer: sprint to 0.8 m/s, circle R=0.8 CCW, stop")
    try:
        mujoco.viewer.launch(model, data)
    finally:
        mujoco.set_mjcb_control(None)


def _teleop(model, params, eq_qpos):
    from .interactive import teleop_loop

    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    pending = []
    state = {"v": 0.0}

    def on_key(keycode):
        pending.append(keycode)

    def step(m, d):
        while pending:
            k = pending.pop(0)
            if k == 265:      # up
                state["v"] = min(state["v"] + 0.25, c.profile.v_max)
                c.set_speed(state["v"])
            elif k == 264:    # down
                state["v"] = max(state["v"] - 0.25, -c.profile.v_max)
                c.set_speed(state["v"])
            elif k in (263, 262):   # left / right: slewed turn (works at any speed)
                c.command_heading(d, np.deg2rad(15.0 if k == 263 else -15.0))
            elif k in (ord("J"), ord("L")):   # big turns, pivot-teleop style
                c.command_heading(d, np.deg2rad(90.0 if k == ord("J") else -90.0))
            elif k == ord("C"):
                c.command_circle(d, 0.8, +1)
            elif k == ord("V"):
                c.command_circle(d, 0.8, -1)
            elif k == ord(" "):
                state["v"] = 0.0
                c.command_stop()
        c.step(m, d)

    teleop_loop(model, data, step, on_key,
                "teleop: ↑/↓ speed ±0.25   ←/→ turn ±15°   J/L turn ±90°   "
                "C/V circle L/R   Space stop   (Esc quits)",
                "aow_sim.run_drive")


if __name__ == "__main__":
    import mujoco.viewer  # noqa: F401

    main()
