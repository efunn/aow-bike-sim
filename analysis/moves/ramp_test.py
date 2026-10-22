"""The ramp test: up a ramp, hold on it, turn across it, come back down.

docs/plans/wishlist.md, "ramps". No policy has trained on a slope (no config
sets `floor_tilt_*`), so this asks how far the level-ground policies
generalise, the way the bike will meet a ramp: from FLAT floor, over the edge
and onto it.

THE SCENE. The usual level floor, plus a box whose top face rises from the
floor at x = 1 m at --angle deg, 3 m long, full width: a ramp with a real
bottom edge. The bike starts at rest at the origin facing it.

THE COURSE (one play, config-free, `COURSE` below): settle; drive 0.5 m/s
up onto the ramp (1 m of flat, then ~1.5 m of slope); HOLD on the slope
facing uphill; turn +90 deg and HOLD across it; turn to face downhill and
HOLD; drive down and off; stop on the flat.

THE SENSORS are the bike's: the AHRS model (--ahrs, default tm151_filter at
tau 2.0 s, what the GPU policies trained against) and the onboard odometry
(front, encoder counts), as teleop flies by default. Truth with
`--ahrs none --no-odometry`.

Per phase: the worst |roll| (truth), how far the bike moved during a hold
(a slide down the slope shows here), the mean speed while driving, and the
heading error at the end of a turn. FELL is |roll| > 60 deg.

    python analysis/moves/ramp_test.py
    python analysis/moves/ramp_test.py --angle 5 10 15 20 --policies general_puffer_seed1_72M

Ideal drivetrain. Ramp friction = the floor's (`sim.friction_sliding`).
"""
from __future__ import annotations

import argparse
import contextlib
import io

import mujoco
import numpy as np

from aow_sim import build_model as bm
from aow_sim import drivetrain_model
from aow_sim.build_model import FLOOR_CONAFF, FLOOR_CONTYPE, _contact_friction, load_params
from aow_sim.control.command_play import Play
from aow_sim.control.drive import DriveController
from aow_sim.control.linearize import settle_upright
from aow_sim.control.righting import roll_pitch

RAMP_X0, RAMP_LEN, RAMP_W, RAMP_T = 1.0, 3.0, 3.0, 0.2

#   t s    v m/s  v_lat  psi deg   phase
COURSE = [
    (0.0,  0.0, 0.0,   0, "settle"),
    (1.0,  0.5, 0.0,   0, "drive up"),
    (6.0,  0.0, 0.0,   0, "hold, facing up"),
    (10.0, 0.0, 0.0,  90, "hold, across"),
    (14.0, 0.0, 0.0, 180, "hold, facing down"),
    (17.0, 0.5, 0.0, 180, "drive down"),
    (22.0, 0.0, 0.0, 180, "stop on the flat"),
    (25.0, 0.0, 0.0, 180, "end"),
]
DRIVE = ("drive up", "drive down")
TURN = ("hold, across", "hold, facing down")


def ramp_model(params, angle_deg):
    """The default bike with a ramp box added to its spec."""
    spec = bm.build_spec(params)
    th = np.deg2rad(angle_deg)
    up = np.array([np.cos(th), 0.0, np.sin(th)])        # along the slope
    nrm = np.array([-np.sin(th), 0.0, np.cos(th)])      # its normal
    centre = np.array([RAMP_X0, 0.0, 0.0]) + up * RAMP_LEN / 2 - nrm * RAMP_T / 2
    spec.worldbody.add_geom(
        name="ramp", type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[RAMP_LEN / 2, RAMP_W / 2, RAMP_T / 2], pos=centre,
        quat=[np.cos(-th / 2), 0.0, np.sin(-th / 2), 0.0],
        contype=FLOOR_CONTYPE, conaffinity=FLOOR_CONAFF,
        condim=params["sim"]["condim"], friction=_contact_friction(params["sim"]),
        rgba=[0.6, 0.7, 0.8, 1.0])
    m = spec.compile()
    # Registered as the default bike, so the controller's LQR twin (built
    # from these arguments, on level ground) is the ordinary one.
    bm._BUILD_ARGS[m] = bm.build_args(bm.build_model(params))
    return m


def run(params, policy, angle, ahrs, ahrs_tau, odometry):
    m = ramp_model(params, angle)
    d = mujoco.MjData(m)
    d.qpos[:] = settle_upright(m).qpos
    mujoco.mj_forward(m, d)
    with contextlib.redirect_stdout(io.StringIO()):
        c = DriveController(params, m)
        c.reset(m, d)
        c.engage_general(d, name=policy)
    hooks = drivetrain_model.attach_hooks(m, params)
    sens = odo = None
    if ahrs != "none":
        from aow_sim.sim_ahrs import SimAhrs, level_tau_orient_s
        sens = SimAhrs(m, params, level=ahrs, tau_orient_s=(
            level_tau_orient_s(ahrs) if ahrs_tau is None else ahrs_tau))
    if odometry:
        from aow_sim.sim_odometry import SimOdometry
        odo = SimOdometry(m, params, mode="front", encoder="counts", ahrs=sens)
    play = Play.from_keys([k[:4] for k in COURSE])
    names = [k[4] for k in COURSE]
    psi0 = c._psi
    out = {n: dict(roll=0.0, xy=[], v=[], head=np.nan) for n in names}
    fell_in = None
    dt = m.opt.timestep
    while d.time < play.duration:
        i = int(np.searchsorted(play.t, d.time, side="right")) - 1
        ph = names[i]
        v, vl, dpsi = play.at(d.time)
        c.set_command_polar(float(np.hypot(v, vl)), float(np.arctan2(vl, v)),
                            psi_cmd=psi0 + dpsi)
        if odo is not None:
            with odo.estimated(d, dt):
                c.step(m, d)
        elif sens is not None:
            sens.tick(d, dt)
            with sens.estimated(d):
                c.step(m, d)
        else:
            c.step(m, d)
        for h in hooks:
            h.pre_step(d)
        mujoco.mj_step(m, d)
        roll, _ = roll_pitch(d.qpos[3:7])
        o = out[ph]
        o["roll"] = max(o["roll"], abs(roll))
        o["xy"].append(d.qpos[:2].copy())
        o["v"].append(float(np.hypot(*d.qvel[:2])))
        w, z = d.qpos[3], d.qpos[6]
        yaw = 2 * np.arctan2(z, w)       # small roll/pitch: good enough here
        o["head"] = float(np.degrees(np.angle(np.exp(1j * (yaw - psi0 - np.deg2rad(COURSE[i][3]))))))
        if abs(roll) > 60 and fell_in is None:
            fell_in = ph
            break
    return out, names, fell_in


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--policies", nargs="+",
                    default=["general_puffer_seed1_72M", "general_puffer_fixes_52M"])
    ap.add_argument("--angle", type=float, nargs="+", default=[5.0, 10.0, 15.0])
    ap.add_argument("--ahrs", default="tm151_filter")
    ap.add_argument("--ahrs-tau", type=float, default=2.0)
    ap.add_argument("--no-odometry", action="store_true")
    a = ap.parse_args()
    p = load_params()
    sens = (f"AHRS {a.ahrs}" + (f" tau {a.ahrs_tau:g} s" if a.ahrs != "none" else "")
            + ("" if a.no_odometry else ", odometry front/counts"))
    print(f"ramp from x = {RAMP_X0} m, {RAMP_LEN} m long; {sens}")
    for pol in a.policies:
        for ang in a.angle:
            out, names, fell = run(p, pol, ang, a.ahrs, a.ahrs_tau, not a.no_odometry)
            print(f"\n{pol}, {ang:g} deg" + (f": FELL during '{fell}'" if fell else ""))
            print("    phase                 max|roll|  moved m  mean v m/s  heading err")
            for n in names[:-1]:
                o = out[n]
                if not o["xy"]:
                    continue
                moved = float(np.linalg.norm(o["xy"][-1] - o["xy"][0]))
                print(f"    {n:20s}  {o['roll']:7.1f}  {moved:7.2f}"
                      f"  {np.mean(o['v']) if n in DRIVE else float('nan'):9.2f}"
                      f"  {o['head'] if n in TURN else float('nan'):+9.1f}")


if __name__ == "__main__":
    main()
