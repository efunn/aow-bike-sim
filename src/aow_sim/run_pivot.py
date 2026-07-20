"""Crawl-pivot harness: python -m aow_sim.run_pivot [--view].

Headless (default): scenario table — pivots at the configured profile, then a
yaw-rate sweep to find the achievable envelope. Metrics per scenario:
  duration        — profile time (accel-limited trapezoid)
  err@1s / err@4s — heading error after the profile ends [deg]
  max |roll|      — during pivot + hold [deg]
  wander          — max distance of the front contact from its start [cm];
                    the "pivot in place" measure
--view: scripted demo (+180°, hold, −180°) with the controller in the loop.
--teleop: interactive heading control (macOS: run under `mjpython`):
    left/right arrows  ±30°      J / L  ±90°      U / O  ±180°
Commands re-anchor immediately, so mashing keys mid-pivot is allowed.
"""

from __future__ import annotations

import argparse

import mujoco
import numpy as np

from .build_model import build_model, load_params
from .control import PivotController, run
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


def pivot_scenario(model, params, eq_qpos, delta_deg: float) -> dict:
    data = _fresh(model, eq_qpos)
    c = PivotController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.5)
    psi0 = c._psi
    duration = c.command_pivot(data, np.deg2rad(delta_deg))
    pf0 = c._p_front.copy()
    rolls, wander = [], []

    def rec(dd):
        s = extract_state(dd, c._ref_pos)
        rolls.append(np.degrees(s.roll))
        pf = dd.qpos[:2] + c.wheelbase * np.array([np.cos(s.yaw), np.sin(s.yaw)])
        wander.append(float(np.linalg.norm(pf - pf0)))

    run(model, data, c, duration + 1.0, on_step=rec)
    err1 = np.degrees(c._psi - psi0) - delta_deg
    run(model, data, c, 3.0, on_step=rec)
    err4 = np.degrees(c._psi - psi0) - delta_deg
    max_roll = float(np.max(np.abs(rolls)))
    return {
        "delta [deg]": delta_deg,
        "duration [s]": round(duration, 2),
        "err@1s [deg]": round(err1, 2),
        "err@4s [deg]": round(err4, 2),
        "max |roll| [deg]": round(max_roll, 2),
        "wander [cm]": round(100 * np.max(wander), 1),
        "survived": bool(np.all(np.isfinite(rolls)) and max_roll < UPRIGHT_LIMIT_DEG),
    }


def rate_sweep(model, params, eq_qpos, rates=(0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0)):
    import copy

    print("\nyaw-rate sweep (±90°):")
    best = 0.0
    for rate in rates:
        p = copy.deepcopy(params)
        p["control"]["pivot"]["yaw_rate"] = rate
        res = pivot_scenario(model, p, eq_qpos, 90.0)
        ok = res["survived"] and abs(res["err@4s [deg]"]) < 10.0
        print(f"  {rate:4.1f} rad/s: err@4s={res['err@4s [deg]']:+6.2f}  "
              f"roll={res['max |roll| [deg]']:5.2f}  wander={res['wander [cm]']:5.1f} cm  "
              f"{'ok' if ok else 'FAIL'}")
        if ok:
            best = rate
        else:
            break
    print(f"  -> largest clean pivot rate: {best:.1f} rad/s")


# Number keys + arrows: MuJoCo's viewer binds every letter, so letters double
# up with its shortcuts (F=force display, etc.). 6-9 are free.
KEY_DELTAS_DEG = {
    263: +30.0, 262: -30.0,            # GLFW left / right arrow
    ord("6"): +90.0, ord("7"): -90.0,
    ord("8"): +180.0, ord("9"): -180.0,
}


def teleop(model, params, eq_qpos) -> None:
    from .interactive import teleop_loop

    data = _fresh(model, eq_qpos)
    c = PivotController(params, model)
    c.reset(model, data)
    pending: list[float] = []

    def on_key(keycode):
        if keycode in KEY_DELTAS_DEG:
            pending.append(np.deg2rad(KEY_DELTAS_DEG[keycode]))

    def step(m, d):
        if pending:
            c.command_pivot(d, pending.pop(0))
        c.step(m, d)

    teleop_loop(model, data, step, on_key,
                "teleop: ←/→ ±30°   6/7 ±90°   8/9 ±180°   (Esc quits)",
                "aow_sim.run_pivot")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=None)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--teleop", action="store_true",
                    help="interactive heading control (macOS: use mjpython)")
    args = ap.parse_args()
    params = load_params(args.params)
    model = build_model(params, variant="full")
    eq = settle_upright(model)

    if args.teleop:
        teleop(model, params, eq.qpos)
        return
    if args.view:
        data = _fresh(model, eq.qpos)
        c = PivotController(params, model)
        c.reset(model, data)
        seq = [(2.0, np.deg2rad(180)), (None, np.deg2rad(-180))]  # (start time, delta)
        state = {"i": 0, "t_next": 2.0}

        def cb(m, d):
            c.step(m, d)
            if state["i"] < len(seq) and d.time >= state["t_next"]:
                T = c.command_pivot(d, seq[state["i"]][1])
                state["i"] += 1
                state["t_next"] = d.time + T + 2.0
        mujoco.set_mjcb_control(cb)
        print("viewer: +180° pivot, 2 s hold, −180° back — then station-keeping")
        try:
            mujoco.viewer.launch(model, data)
        finally:
            mujoco.set_mjcb_control(None)
        return

    print("crawl pivot scenarios (config profile):")
    for delta in (90.0, -90.0, 180.0):
        res = pivot_scenario(model, params, eq.qpos, delta)
        print("  " + "  ".join(f"{k}={v}" for k, v in res.items()))
    rate_sweep(model, params, eq.qpos)


if __name__ == "__main__":
    import mujoco.viewer  # noqa: F401

    main()
