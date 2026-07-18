"""Stationary balance harness: python -m aow_sim.run_balance [--controller lqr|pd].

Headless (default): scenario suite against the full bike (no training wheels):
  1. Recovery from an initial roll tilt — roll RMS, drift, survival.
  2. Lateral push sweep — 0.1 s force pulses at the chassis, increasing
     magnitude; reports the largest recovered push.
The printed table is the regression baseline for future controllers.

--view: interactive viewer with the controller in the loop (drag-perturb the
bike with double-click + Ctrl+drag and watch it catch itself).
"""

from __future__ import annotations

import argparse

import mujoco
import numpy as np

from .build_model import build_model, load_params
from .control.balance import extract_state, make_controller, run
from .control.linearize import settle_upright

UPRIGHT_LIMIT_DEG = 60.0


def _tilted_data(model, eq_qpos, tilt_deg: float) -> mujoco.MjData:
    data = mujoco.MjData(model)
    data.qpos[:] = eq_qpos
    a = np.deg2rad(tilt_deg)
    data.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(model, data)
    return data


class _Recorder:
    def __init__(self, controller):
        self.c = controller
        self.rolls: list[float] = []
        self.drifts: list[float] = []

    def __call__(self, data):
        s = extract_state(data, self.c._ref_pos)
        self.rolls.append(s.roll)
        self.drifts.append(float(np.hypot(*(data.qpos[:2] - self.c._ref_pos[:2]))))

    @property
    def survived(self) -> bool:
        r = np.degrees(np.abs(self.rolls))
        return bool(np.all(np.isfinite(r)) and np.max(r) < UPRIGHT_LIMIT_DEG)


def tilt_scenario(model, params, name: str, eq_qpos, tilt_deg=3.0, duration=10.0):
    data = _tilted_data(model, eq_qpos, tilt_deg)
    ctrl = make_controller(name, params, model)
    ctrl.reset(model, data)
    rec = _Recorder(ctrl)
    run(model, data, ctrl, duration, on_step=rec)
    rolls = np.degrees(rec.rolls)
    tail = rolls[len(rolls) // 2:]
    return {
        "survived": rec.survived,
        "tail roll RMS [deg]": float(np.sqrt(np.mean(tail**2))),
        "max |roll| [deg]": float(np.max(np.abs(rolls))),
        "max drift [m]": float(np.max(rec.drifts)),
        "final drift [m]": rec.drifts[-1],
    }


def push_scenario(model, params, name: str, eq_qpos, force: float,
                  settle=2.0, hold=5.0, pulse=0.1) -> bool:
    data = _tilted_data(model, eq_qpos, 0.5)
    ctrl = make_controller(name, params, model)
    ctrl.reset(model, data)
    rec = _Recorder(ctrl)
    chassis = model.body("chassis").id
    run(model, data, ctrl, settle, on_step=rec)
    data.xfrc_applied[chassis, 1] = force
    run(model, data, ctrl, pulse, on_step=rec)
    data.xfrc_applied[chassis, 1] = 0.0
    run(model, data, ctrl, hold, on_step=rec)
    return rec.survived


def push_sweep(model, params, name: str, eq_qpos, forces=None):
    forces = forces or [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0]
    best = 0.0
    for f in forces:
        if push_scenario(model, params, name, eq_qpos, f):
            best = f
        else:
            break
    return best


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--controller", choices=["lqr", "pd"], default="lqr")
    ap.add_argument("--params", default=None)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--tilt-deg", type=float, default=3.0)
    args = ap.parse_args()

    params = load_params(args.params)
    model = build_model(params, variant="full")
    eq = settle_upright(model)

    if args.view:
        data = _tilted_data(model, eq.qpos, args.tilt_deg)
        ctrl = make_controller(args.controller, params, model)
        ctrl.reset(model, data)
        mujoco.set_mjcb_control(lambda m, d: ctrl.step(m, d))
        print(f"viewer: {args.controller} controller active — "
              "double-click the chassis, Ctrl+right-drag to shove it")
        try:
            mujoco.viewer.launch(model, data)
        finally:
            mujoco.set_mjcb_control(None)
        return

    print(f"controller: {args.controller}")
    if args.controller == "lqr":
        probe = make_controller("lqr", params, model)
        print("identified-model fit R^2:", np.round(probe.fit_r2, 4))
    m1 = tilt_scenario(model, params, args.controller, eq.qpos, args.tilt_deg)
    print(f"\ntilt recovery ({args.tilt_deg:.1f} deg, 10 s):")
    for k, v in m1.items():
        print(f"  {k:22s} {v if isinstance(v, bool) else round(v, 4)}")
    best = push_sweep(model, params, args.controller, eq.qpos)
    print(f"\nlargest recovered lateral push: {best:.1f} N x 0.1 s "
          f"({best * 0.1:.2f} N*s impulse)")


if __name__ == "__main__":
    import mujoco.viewer  # noqa: F401  (registers the viewer module)

    main()
