"""Does a realistically-placed reaction wheel actually buy the bike anything?

Asks the question at the PLANT level, with the analytic LQR as the controller,
because that needs no training and therefore no remote box. What it can answer:
"does the bike become easier to save / easier to turn". What it CANNOT answer:
"would a policy trained with a flywheel find a use for it we did not think of".
Read the numbers as a floor on the benefit, not a ceiling.

THE ONE NUMBER THAT MATTERS is momentum, not torque. A reaction wheel is a
momentum store: it exerts torque only while accelerating, and once it hits its
no-load speed it has nothing left. So its whole authority is

    H = I_wheel * w_noload

and everything else -- stall torque, gear ratio, how quickly it spins up -- only
says how fast H gets spent. Both servo candidates are torque-rich and
momentum-poor by three orders of magnitude, which is why `gear_ratio` in
config/flywheel.yaml is a step-UP: the servo gearbox already traded away the
speed that H is made of.

    python analysis/flywheel.py                    # budget + both axes
    python analysis/flywheel.py --axis roll        # just the balance question
    python analysis/flywheel.py --kicks 0.15 0.35  # bisection bracket

Read-only apart from an optional --csv. Builds its own models; touches no
policy, no bundle, and no bike_params.yaml -- the flywheel geometry lives in
config/flywheel.yaml precisely so this study cannot move the params digest.
"""
from __future__ import annotations

import argparse
import copy
import csv
import os
import tempfile
from pathlib import Path

import mujoco
import numpy as np
import yaml

from aow_sim.build_model import (FLYWHEEL_CFG, build_model, flywheel_budget,
                                 load_params)
from aow_sim.control.balance import extract_state
from aow_sim.control.drive import DriveController
from aow_sim.control.linearize import design_all, settle_rolling

FALL_DEG = 75.0
RECOVER_DEG = 20.0
HORIZON_S = 6.0
TAIL_S = 1.0
BISECT_TOL = 0.005      # m/s, resolution of the kick search
GRAVITY = 9.80665


def _cfg(axis: str, **over) -> dict:
    cfg = yaml.safe_load(FLYWHEEL_CFG.read_text())
    cfg["axis"] = axis
    cfg.update(over)
    return cfg


def _build(params, cfg: dict | None):
    """A model with (or without) the wheel. `cfg` is written to a temp file
    rather than passed as a dict because build_spec takes a PATH -- keeping one
    code path for how the study and a normal build read the same config."""
    if cfg is None:
        return build_model(params, variant="full")
    fd, path = tempfile.mkstemp(suffix=".yaml")
    try:
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(cfg, f)
        return build_model(params, variant="full", flywheel=True, flywheel_cfg=path)
    finally:
        os.unlink(path)


# --------------------------------------------------------------------------
# plant-level numbers, no controller involved


def roll_inertia(model, params) -> tuple[float, float]:
    """(CoM height above the floor, roll inertia about the contact line).

    Measured off the compiled model rather than re-derived from the yaml, so
    it stays right when the flywheel's own mass moves the CoM."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    total = model.body_subtreemass[model.body("chassis").id]
    com = data.subtree_com[model.body("chassis").id].copy()
    h = float(com[2])
    # Parallel axis from the whole-bike inertia about the CoM. mj_fullM would
    # give the joint-space mass matrix; for a free body the top-left 3x3 of the
    # rotational block is what we want, so take it from the composite inertia.
    mujoco.mj_forward(model, data)
    i_com = float(data.cinert[model.body("chassis").id][0])  # about CoM, xx
    return h, total * h * h + max(i_com, 0.0)


def budget_report(params, cfg: dict) -> dict:
    b = flywheel_budget(params, cfg)
    m_fw = _build(params, cfg)
    m_base = _build(params, None)
    h_fw, i_roll_fw = roll_inertia(m_fw, params)
    h_base, i_roll_base = roll_inertia(m_base, params)
    i_yaw = float(mujoco.MjData(m_fw).cinert[m_fw.body("chassis").id][2]) or None
    mass_base = float(m_base.body_subtreemass[m_base.body("chassis").id])
    mass_fw = float(m_fw.body_subtreemass[m_fw.body("chassis").id])
    return {
        **b,
        "mass_base": mass_base,
        "mass_fw": mass_fw,
        "com_base": h_base,
        "com_fw": h_fw,
        "i_roll": i_roll_fw,
        # The rate step the wheel can absorb in one shot, which is the
        # honest statement of "how big a save does this buy".
        "d_roll_rate": b["momentum"] / i_roll_fw,
        "grav_torque_10deg": mass_fw * GRAVITY * h_fw * np.sin(np.deg2rad(10)),
    }


# --------------------------------------------------------------------------
# closed loop: LQR for drive+steer, a hand-written law for the wheel


class FlywheelLaw:
    """Roll-rate-dominant feedback with a slow unwind toward zero speed.

    NOT tuned hard, and deliberately so. The question is whether the ACTUATOR
    is worth having, and a heavily tuned law would confound "the wheel helps"
    with "these three gains help". kd carries the work because a momentum store
    is a damper: it can absorb rate, and it cannot hold a static lean at all.

    `unwind` bleeds wheel speed back toward zero so the store is recharged for
    the next disturbance. It fights the damping term, so it is small; without
    it the wheel parks at saturation after the first save and every later one
    gets nothing."""

    def __init__(self, model, tau_max: float, kp=0.0, kd=0.35, unwind=0.004):
        self.aid = model.actuator("flywheel").id
        self.jid = model.joint("flywheel_joint").id
        self.dof = model.jnt_dofadr[self.jid]
        self.tau_max, self.kp, self.kd, self.unwind = tau_max, kp, kd, unwind

    def __call__(self, data, roll, roll_rate) -> float:
        w = float(data.qvel[self.dof])
        tau = -self.kp * roll - self.kd * roll_rate + self.unwind * w
        return float(np.clip(tau, -self.tau_max, self.tau_max))


def kick_rollout(model, params, design, dv: float, cfg: dict | None,
                 law: FlywheelLaw | None, v0: float = 0.0):
    """Lateral velocity step on the chassis, then does the LQR keep it up.

    Matches tests/test_hw_odometry and analysis/kick_recovery: the disturbance
    is `qvel[1] += dv`, NOT xfrc_applied, because several code paths zero
    xfrc at the top of a step and a force written from outside then silently
    does nothing -- which reads as a miraculously robust bike."""
    eq = settle_rolling(model, params, float(v0))
    data = mujoco.MjData(model)
    data.qpos[:] = eq.qpos
    data.qvel[:] = eq.qvel
    data.ctrl[:] = eq.ctrl
    data.qacc_warmstart[:] = 0.0      # see kick_recovery.py: this leaks
    mujoco.mj_forward(model, data)

    ctrl = DriveController(params, model, design=design)
    ctrl.reset(model, data)
    ctrl.profile.v_ref = v0
    ctrl.set_speed(v0)

    data.qvel[1] += dv
    dt = model.opt.timestep
    n = int(round(HORIZON_S / dt))
    tail_start = n - int(round(TAIL_S / dt))
    tail_max, peak_w, peak_roll = 0.0, 0.0, 0.0
    for i in range(n):
        ctrl.step(model, data)
        s = extract_state(data, ctrl._ref_pos)
        if law is not None:
            data.ctrl[law.aid] = law(data, s.roll, s.roll_rate)
            peak_w = max(peak_w, abs(float(data.qvel[law.dof])))
        mujoco.mj_step(model, data)
        if not np.isfinite(s.roll) or abs(s.roll) > np.deg2rad(FALL_DEG):
            return False, np.degrees(abs(s.roll)), peak_w
        peak_roll = max(peak_roll, abs(s.roll))
        if i >= tail_start:
            tail_max = max(tail_max, abs(s.roll))
    return tail_max < np.deg2rad(RECOVER_DEG), np.degrees(peak_roll), peak_w


def max_kick(model, params, design, cfg, law, v0, lo, hi) -> tuple[float, float]:
    """Largest recovered lateral kick [m/s], by bisection. Returns (dv, peak
    wheel speed as a fraction of saturation at that dv)."""
    ok_lo, _, w_lo = kick_rollout(model, params, design, lo, cfg, law, v0)
    if not ok_lo:
        return 0.0, 0.0
    ok_hi, _, _ = kick_rollout(model, params, design, hi, cfg, law, v0)
    if ok_hi:
        return hi, w_lo
    best, best_w = lo, w_lo
    while hi - lo > BISECT_TOL:
        mid = 0.5 * (lo + hi)
        ok, _, w = kick_rollout(model, params, design, mid, cfg, law, v0)
        if ok:
            lo, best, best_w = mid, mid, w
        else:
            hi = mid
    return best, best_w


# --------------------------------------------------------------------------
# yaw: what the wheel can do to heading, against what steering already does


def yaw_authority(model, params, design, cfg, tau_max: float, v0: float = 0.0):
    """Full flywheel torque from rest at a hold, with the LQR still balancing.

    Reports the heading actually swung before the wheel saturates. That is the
    honest agility number: a reaction wheel gives you a FINITE ANGLE, not a
    rate you can hold."""
    eq = settle_rolling(model, params, float(v0))
    data = mujoco.MjData(model)
    data.qpos[:] = eq.qpos
    data.qvel[:] = eq.qvel
    data.ctrl[:] = eq.ctrl
    data.qacc_warmstart[:] = 0.0
    mujoco.mj_forward(model, data)
    ctrl = DriveController(params, model, design=design)
    ctrl.reset(model, data)
    ctrl.profile.v_ref = v0
    ctrl.set_speed(v0)

    aid = model.actuator("flywheel").id
    dof = model.jnt_dofadr[model.joint("flywheel_joint").id]
    dt = model.opt.timestep
    yaw0 = extract_state(data, ctrl._ref_pos).yaw
    # MEASURE ONLY UP TO SATURATION. Past it the wheel has nothing left, the
    # commanded torque is fiction, and the LQR rings against it -- an earlier
    # version of this reported max|yaw| and peak|yaw rate| over the whole 2 s
    # and got 5.2 deg / 305 deg/s, which is the AMPLITUDE OF THAT RINGING and
    # not authority at all. The honest number is the net heading change while
    # the wheel still has momentum to give.
    yaw_at_sat, t_sat, fell = 0.0, None, False
    roll_amp = 0.0
    for i in range(int(round(2.0 / dt))):
        ctrl.step(model, data)
        data.ctrl[aid] = tau_max
        mujoco.mj_step(model, data)
        s = extract_state(data, ctrl._ref_pos)
        if not np.isfinite(s.roll) or abs(s.roll) > np.deg2rad(FALL_DEG):
            fell = True
            break
        if t_sat is None:
            yaw_at_sat = np.degrees(s.yaw - yaw0)
            roll_amp = max(roll_amp, abs(np.degrees(s.roll)))
            if abs(float(data.qvel[dof])) > 0.95 * cfg["_w_max"]:
                t_sat = i * dt
    return {"yaw_at_saturation_deg": yaw_at_sat, "t_saturate_s": t_sat,
            "roll_amp_deg": roll_amp, "fell": fell}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--axis", choices=["roll", "yaw", "both"], default="both")
    ap.add_argument("--speeds", type=float, nargs="+", default=[0.0, 0.4])
    ap.add_argument("--kicks", type=float, nargs=2, default=[0.05, 0.60],
                    metavar=("LO", "HI"), help="bisection bracket [m/s]")
    ap.add_argument("--kd", type=float, default=0.35)
    ap.add_argument("--mass", type=float, default=None,
                    help="override flywheel mass [kg], for a sensitivity run")
    ap.add_argument("--gear-ratio", type=float, default=None)
    ap.add_argument("--csv", type=Path, default=None)
    a = ap.parse_args()

    params = load_params()
    over = {}
    if a.mass is not None:
        over["mass"] = a.mass
    if a.gear_ratio is not None:
        over["gear_ratio"] = a.gear_ratio

    rows = []
    print("=" * 74)
    print("MOMENTUM BUDGET  (config/flywheel.yaml"
          + (f", overrides {over}" if over else "") + ")")
    print("=" * 74)
    ref = _cfg("roll", **over)
    b = budget_report(params, ref)
    print(f"  wheel                {b['inertia']*1e5:8.2f}e-5 kg.m^2"
          f"   ({ref['mass']*1e3:.0f} g at {ref['radius']*1e3:.0f} mm,"
          f" rim_fraction {ref['rim_fraction']})")
    print(f"  gear ratio           {ref['gear_ratio']:8.2f} : 1  (step-up)")
    print(f"  flywheel torque      {b['tau_max']:8.3f} N.m")
    print(f"  flywheel top speed   {b['w_max']:8.1f} rad/s"
          f"  ({b['w_max']*60/(2*np.pi):.0f} rpm)")
    print(f"  MOMENTUM STORE H     {b['momentum']*1e3:8.2f} mN.m.s"
          f"   <- the whole authority")
    print(f"  spin-up to full      {b['spin_up_s']*1e3:8.0f} ms")
    print()
    print(f"  bike mass            {b['mass_base']:8.3f} -> {b['mass_fw']:.3f} kg"
          f"  (+{100*(b['mass_fw']/b['mass_base']-1):.0f}%)")
    print(f"  CoM height           {b['com_base']*1e3:8.1f} -> {b['com_fw']*1e3:.1f} mm")
    print(f"  roll inertia         {b['i_roll']*1e3:8.2f}e-3 kg.m^2 (about contact line)")
    print(f"  gravity torque @10deg{b['grav_torque_10deg']:8.3f} N.m"
          f"   vs flywheel {b['tau_max']:.3f} N.m")
    print()
    print(f"  ONE-SHOT ROLL RATE IT CAN NULL:  H / I_roll ="
          f" {b['d_roll_rate']:.3f} rad/s = {np.degrees(b['d_roll_rate']):.1f} deg/s")
    print("  (once. then it is saturated and the differential has to dump it.)")

    axes = ["roll", "yaw"] if a.axis == "both" else [a.axis]

    if "roll" in axes:
        print()
        print("=" * 74)
        print("ROLL AXIS -- largest lateral kick the LQR still recovers [m/s]")
        print("=" * 74)
        print("THE BALLAST COLUMN IS THE POINT. A 130 g wheel mounted low drops")
        print("the CoM by 6 mm, and a lower CoM improves the kick envelope all by")
        print("itself. Without an equal-mass dead-weight control, 'the reaction")
        print("wheel helped' is indistinguishable from 'the ballast helped'.")
        print()
        print(f"{'speed':>7} {'baseline':>10} {'ballast':>10} {'flywheel':>10}"
              f" {'fw vs ball':>11} {'wheel used':>11}")
        cfg = _cfg("roll", **over)
        # Same total added mass, same place, but the rotor's inertia is
        # negligible -- so it can store no momentum and act as no actuator.
        ball = _cfg("roll", **over)
        ball["bracket_mass"] = ball["mass"] + ball["bracket_mass"]
        ball["mass"] = 1e-4
        m_base = _build(params, None)
        d_base = design_all(params, m_base)
        m_ball = _build(params, ball)
        d_ball = design_all(params, m_ball)
        m_fw = _build(params, cfg)
        d_fw = design_all(params, m_fw)
        law = FlywheelLaw(m_fw, b["tau_max"], kd=a.kd)
        for v0 in a.speeds:
            k0, _ = max_kick(m_base, params, d_base, None, None, v0, *a.kicks)
            kb, _ = max_kick(m_ball, params, d_ball, ball, None, v0, *a.kicks)
            k1, w = max_kick(m_fw, params, d_fw, cfg, law, v0, *a.kicks)
            frac = 100 * w / b["w_max"] if b["w_max"] else 0.0
            d = f"{100*(k1/kb-1):+.0f}%" if kb else "n/a"
            print(f"{v0:7.2f} {k0:10.3f} {kb:10.3f} {k1:10.3f} {d:>11}"
                  f" {frac:10.0f}%")
            rows.append({"axis": "roll", "speed": v0, "baseline": k0,
                         "ballast": kb, "flywheel": k1, "wheel_used_pct": frac})

    if "yaw" in axes:
        print()
        print("=" * 74)
        print("YAW AXIS -- full torque from a hold, what heading does it buy")
        print("=" * 74)
        cfg = _cfg("yaw", **over)
        cfg["_w_max"] = b["w_max"]
        m_fw = _build(params, {k: v for k, v in cfg.items() if not k.startswith("_")})
        d_fw = design_all(params, m_fw)
        r = yaw_authority(m_fw, params, d_fw, cfg, b["tau_max"])
        print(f"  NET HEADING at saturation {r['yaw_at_saturation_deg']:7.2f} deg"
              f"   <- the whole agility benefit")
        print(f"  wheel saturated at        "
              + (f"{r['t_saturate_s']*1e3:7.0f} ms" if r["t_saturate_s"] else "    never"))
        print(f"  roll excursion meanwhile  {r['roll_amp_deg']:7.2f} deg"
              f"   (cost, not benefit)")
        print(f"  bike fell                 {str(r['fell']):>7}")
        print()
        print("  For scale: the pivot and flick moves turn the bike 180 deg using")
        print("  the differential against the ground. The ground is an infinite")
        print("  momentum sink; the wheel has 9.4 mN.m.s and then it is done.")
        rows.append({"axis": "yaw", "speed": 0.0, "baseline": "",
                     "flywheel": r["yaw_at_saturation_deg"], "wheel_used_pct": ""})

    if a.csv and rows:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {a.csv}")


if __name__ == "__main__":
    main()
