"""Two-arc 180-degree flick: trajectory representation, rollout, cost, I/O.

The flick is a reverse-arc then forward-arc, with the front wheel sweeping
continuously 0->180 deg in one direction while the drive flips reverse->forward
(so the same steer direction yields same-sign yaw across both arcs). It is a
hard coordination problem, so the feedforward trajectory (steer + hub) is found
offline by trajectory optimization (see optimize_flick.py); a roll->crawl
balance runs underneath during the rollout.

This module holds the pieces the optimizer and the replay controller share:
  FlickTrajectory  — the parameterized feedforward schedules.
  rollout / cost   — simulate a candidate and score it.
  load_move/save   — read/write moves/<name>.yaml (the optimizer's output).
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import date
from pathlib import Path

import mujoco
import numpy as np
import yaml

from .balance import extract_state, mix

GRAVITY = 9.81
MOVES_DIR = Path(__file__).resolve().parents[3] / "moves"  # repo-root/moves

# Parameter vector layout (7): [T, s1, s2, s3, v1, v2, v3]
#   steer knots at fractions 1/4,2/4,3/4 of T, endpoints 0 and pi (sorted).
#   hub knots at the same interior fractions, endpoints 0 and 0.
N_PARAM = 7
PARAM_BOUNDS = [
    (1.5, 4.0),                    # T horizon [s]
    (0.0, np.pi), (0.0, np.pi), (0.0, np.pi),   # steer knots [rad]
    (-0.6, 0.6), (-0.6, 0.6), (-0.6, 0.6),      # hub knots [m/s]
]


@dataclass
class FlickTrajectory:
    T: float
    steer_knots: np.ndarray   # 3 interior knots [rad]; profile is [0,*,*,*,pi]
    hub_knots: np.ndarray     # 3 interior knots [m/s]; profile is [0,*,*,*,0]
    direction: int = 1        # +1 as-authored; -1 mirrors steer & keeps hub

    @classmethod
    def from_params(cls, p, direction: int = 1) -> "FlickTrajectory":
        return cls(float(p[0]), np.sort(np.asarray(p[1:4], float)),
                   np.asarray(p[4:7], float), direction)

    def _knot_times(self) -> np.ndarray:
        return np.array([0.0, 0.25, 0.5, 0.75, 1.0]) * self.T

    def steer(self, t: float) -> float:
        vals = np.concatenate([[0.0], self.steer_knots, [np.pi]])
        return self.direction * float(np.interp(t, self._knot_times(), vals))

    def hub(self, t: float) -> float:
        vals = np.concatenate([[0.0], self.hub_knots, [0.0]])
        return float(np.interp(t, self._knot_times(), vals))

    def to_dict(self) -> dict:
        return {"horizon": round(self.T, 4),
                "steer_knots": [round(float(x), 4) for x in self.steer_knots],
                "hub_knots": [round(float(x), 4) for x in self.hub_knots]}


def _fresh(model, eq_qpos):
    data = mujoco.MjData(model)
    data.qpos[:] = eq_qpos
    a = np.deg2rad(0.5)                      # tiny lean so balance is exercised
    data.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(model, data)
    return data


def rollout(model, params, eq_qpos, K0, flick: FlickTrajectory,
            settle: float = 2.0) -> dict:
    """Simulate one flick with feedforward steer+hub and roll->crawl balance.
    Returns metrics: max roll, max lateral |y|, final yaw/rates, x shift, fell."""
    aid = {n: model.actuator(n).id for n in ("drive_a", "drive_b", "steer")}
    rate_hz = params["control"]["rate_hz"]
    ctrl_dt = 1.0 / rate_hz
    r_wheel = params["omni_wheel"]["outer_radius"]

    data = _fresh(model, eq_qpos)
    p0 = data.qpos[:2].copy()
    yaw0 = extract_state(data, p0).yaw
    psi = yaw0
    raw_prev = yaw0
    next_t = 0.0
    u = np.zeros(model.nu)
    total = flick.T + settle
    max_roll = 0.0
    max_roll_rate = 0.0
    max_lat = 0.0

    for _ in range(int(round(total / model.opt.timestep))):
        if data.time + 1e-12 >= next_t:
            s = extract_state(data, p0)
            psi += np.arctan2(np.sin(s.yaw - raw_prev), np.cos(s.yaw - raw_prev))
            raw_prev = s.yaw
            tau = min(data.time, flick.T)
            steer_ff = flick.steer(tau)
            hub = flick.hub(tau) if data.time < flick.T else 0.0
            # roll/lateral -> crawl balance (LQR-derived gains, crawl channel
            # only; steer & yaw states excluded — steer is committed, yaw is
            # the maneuver). Keeps the bike upright AND laterally bounded.
            d_bal = float(-K0[0] @ np.array([
                s.e_lat, s.roll, 0.0, 0.0, s.v_lat, s.roll_rate, 0.0, 0.0]))
            common = hub / r_wheel
            a, b = mix(common, d_bal)
            u[aid["drive_a"]], u[aid["drive_b"]] = a, b
            u[aid["steer"]] = steer_ff
            next_t = data.time + ctrl_dt
        data.ctrl[:] = u
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)):
            return {"fell": True, "max_roll": np.pi, "max_roll_rate": 99.0,
                    "max_lat": 9.9,
                    "yaw_final": psi - yaw0, "roll_f": np.pi, "roll_rate_f": 9.9,
                    "yaw_rate_f": 9.9, "v_f": 9.9, "x_shift": 0.0}
        sr = extract_state(data, p0)
        max_roll = max(max_roll, abs(sr.roll))
        max_roll_rate = max(max_roll_rate, abs(sr.roll_rate))
        # lateral offset in the START yaw frame (y perpendicular to entry heading)
        d = data.qpos[:2] - p0
        lat = -np.sin(yaw0) * d[0] + np.cos(yaw0) * d[1]
        max_lat = max(max_lat, abs(lat))

    s = extract_state(data, p0)
    psi += np.arctan2(np.sin(s.yaw - raw_prev), np.cos(s.yaw - raw_prev))
    d = data.qpos[:2] - p0
    x_shift = np.cos(yaw0) * d[0] + np.sin(yaw0) * d[1]
    return {
        "fell": bool(max_roll > np.deg2rad(60)),
        "max_roll": float(max_roll),
        "max_roll_rate": float(max_roll_rate),
        "max_lat": float(max_lat),
        "yaw_final": float(psi - yaw0),
        "roll_f": float(abs(s.roll)),
        "roll_rate_f": float(abs(s.roll_rate)),
        "yaw_rate_f": float(abs(data.qvel[5])),
        "v_f": float(abs(s.v_lon)),
        "x_shift": float(x_shift),
        "T": float(flick.T),
    }


# Cost weights (kept here so the move file can record them).
COST_WEIGHTS = dict(fell=1000.0, roll=3.0, rollrate=0.6, yaw=8.0, lat=15.0,
                    settle=2.0, time=0.5)


def cost(m: dict, weights: dict = COST_WEIGHTS) -> float:
    w = weights
    settle = m["roll_f"] + m["roll_rate_f"] + m["yaw_rate_f"] + m["v_f"]
    # Target |yaw| = 180 deg: a flick either direction is valid (mirror later).
    yaw_err2 = min((m["yaw_final"] - np.pi) ** 2, (m["yaw_final"] + np.pi) ** 2)
    return (w["fell"] * m["fell"]
            + w["roll"] * m["max_roll"]
            + w.get("rollrate", 0.0) * m.get("max_roll_rate", 0.0)  # damp oscillation
            + w["yaw"] * yaw_err2
            + w["lat"] * m["max_lat"]
            + w["settle"] * settle
            + w["time"] * m.get("T", 0.0))


# -- move file I/O ---------------------------------------------------------

def load_move(name: str, moves_dir: Path | str | None = None) -> FlickTrajectory:
    path = Path(moves_dir or MOVES_DIR) / f"{name}.yaml"
    with open(path) as f:
        d = yaml.safe_load(f)
    return FlickTrajectory(float(d["horizon"]),
                           np.asarray(d["steer_knots"], float),
                           np.asarray(d["hub_knots"], float))


def save_move(name: str, flick: FlickTrajectory, metrics: dict,
              weights: dict, moves_dir: Path | str | None = None) -> Path:
    d = Path(moves_dir or MOVES_DIR)
    d.mkdir(exist_ok=True)
    path = d / f"{name}.yaml"
    doc = {"name": name, **flick.to_dict(),
           "optimized": {
               "date": date.today().isoformat(),
               "cost_weights": {k: float(v) for k, v in weights.items()},
               "metrics": {
                   "lateral_envelope_m": round(float(metrics["max_lat"]), 4),
                   "lateral_envelope_L": None,   # filled by caller if wanted
                   "final_yaw_deg": round(float(np.degrees(metrics["yaw_final"])), 2),
                   "max_roll_deg": round(float(np.degrees(metrics["max_roll"])), 2),
                   "max_roll_rate_deg_s": round(
                       float(np.degrees(metrics.get("max_roll_rate", 0.0))), 1),
                   "x_shift_m": round(float(metrics["x_shift"]), 3),
                   "duration_s": round(float(flick.T), 3),
               }}}
    with open(path, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    return path
