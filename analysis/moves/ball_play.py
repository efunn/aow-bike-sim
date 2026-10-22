"""The wheel shot as a command sequence on the general policy: sweep it, or
fly a play file.

docs/plans/wishlist.md, "ball shots". No shot policy: the general policy
drives an approach and a heading step (control/command_play.py), and the
front wheel swings through the ball -- the "wheel shot" seen on hardware
(docs/plans/ball-shot-move.md).

SWEEP (default). The bike starts at rest at the origin facing +x; the ball
is parked at (--ball-x, offset) -- keep it close (0.6 m): the bike is at
1.2 m/s within 0.25 m, and over metres its own heading wander decides the
aim. Command v straight ahead; when the front
contact is `lead` m short of the ball in x, step the heading by `angle`;
1.5 s later command a stop. Scored by the ball's peak speed, its direction
at that moment (0 = straight ahead, + left), what touched it first, and
whether the bike stayed up. The trigger time printed for each row is what to
put in a play file's second key.

    python analysis/moves/ball_play.py
    python analysis/moves/ball_play.py --v 0.8 1.2 --offset -0.05 -0.1 --angle -60 60
    python analysis/moves/ball_play.py --play config/plays/wheel_shot.yaml --ball-y -0.08

The hockey model is teleop's `--hockey`: the ball AND the side stick panels
(build_model._add_hockey; +40 g). Truth state unless --ahrs. Ideal
drivetrain.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import itertools

import mujoco
import numpy as np

from aow_sim import drivetrain_model
from aow_sim.build_model import build_model, load_params
from aow_sim.control.command_play import load_play
from aow_sim.control.drive import DriveController
from aow_sim.control.linearize import settle_upright
from aow_sim.control.righting import roll_pitch


def setup(params, policy, ball_xy, ahrs="none", ahrs_tau=None):
    m = build_model(params, variant="full", hockey=True)
    d = mujoco.MjData(m)
    d.qpos[:] = settle_upright(m).qpos
    jb = m.body("ball").jntadr[0]
    qb, vb = m.jnt_qposadr[jb], m.jnt_dofadr[jb]
    d.qpos[qb:qb + 3] = [ball_xy[0], ball_xy[1], params["hockey"]["ball"]["radius"]]
    d.qpos[qb + 3:qb + 7] = [1, 0, 0, 0]
    d.qvel[vb:vb + 6] = 0
    mujoco.mj_forward(m, d)
    with contextlib.redirect_stdout(io.StringIO()):
        c = DriveController(params, m)
        c.reset(m, d)
        c.engage_general(d, name=policy)
    c.set_command_polar(0.0, psi_cmd=c._psi)
    hooks = drivetrain_model.attach_hooks(m, params)
    sens = None
    if ahrs != "none":
        from aow_sim.sim_ahrs import SimAhrs, level_tau_orient_s
        sens = SimAhrs(m, params, level=ahrs, tau_orient_s=(
            level_tau_orient_s(ahrs) if ahrs_tau is None else ahrs_tau))
    return m, d, c, hooks, sens, (qb, vb)


def fly(params, policy, ball_xy, command, T=4.0, ahrs="none", ahrs_tau=None,
        crank=None):
    """`command(t, front_x, ball_touched)` -> (v, v_lat, psi) or None to keep.
    `crank(t)` -> the righting crank's goal [rad] (None: centred).
    Returns the shot's score."""
    m, d, c, hooks, sens, (qb, vb) = setup(params, policy, ball_xy, ahrs, ahrs_tau)
    swing = m.actuator("swing").id
    wb = params["bike"]["wheelbase"]
    ball_g = m.geom("ball").id
    floor = m.geom("floor").id
    first, peak, peak_dir, fell = None, 0.0, np.nan, False
    while d.time < T:
        psi = c._psi
        front_x = d.qpos[0] + wb * np.cos(psi)
        cmd = command(d.time, front_x, first is not None)
        if cmd is not None:
            v, vl, ps = cmd
            c.set_command_polar(float(np.hypot(v, vl)), float(np.arctan2(vl, v)), psi_cmd=ps)
        if sens is not None:
            sens.tick(d, m.opt.timestep)
            with sens.estimated(d):
                c.step(m, d)
        else:
            c.step(m, d)
        # After c.step, which writes the whole ctrl vector. The bare
        # position actuator, not teleop's mode-5 servo model: a sweep.
        d.ctrl[swing] = 0.0 if crank is None else crank(d.time)
        for h in hooks:
            h.pre_step(d)
        mujoco.mj_step(m, d)
        if first is None:
            for i in range(d.ncon):
                con = d.contact[i]
                if ball_g in (con.geom1, con.geom2):
                    other = con.geom2 if con.geom1 == ball_g else con.geom1
                    if other != floor:
                        first = m.body(m.geom_bodyid[other]).name
                        break
        sp = float(np.hypot(*d.qvel[vb:vb + 2]))
        if sp > peak:
            peak, peak_dir = sp, float(np.degrees(np.arctan2(d.qvel[vb + 1], d.qvel[vb])))
        if abs(roll_pitch(d.qpos[3:7])[0]) > 60:
            fell = True
    return dict(ball_speed=peak, ball_dir=peak_dir, hit_by=first or "-", fell=fell)


def sweep(a, p):
    rows = []
    for v, off, ang, lead, ck in itertools.product(a.v, a.offset, a.angle, a.lead,
                                                    a.crank):
        trig = {}

        def command(t, front_x, touched, v=v, ang=ang, lead=lead):
            if "t" not in trig and front_x >= a.ball_x - lead:
                trig["t"] = t
            if "t" not in trig:
                return v, 0.0, 0.0
            if t - trig["t"] < 1.5:
                return v, 0.0, np.deg2rad(ang)
            return 0.0, 0.0, np.deg2rad(ang)

        # The blade goes down from the start: the policy drives with it out.
        r = fly(p, a.policy, (a.ball_x, off), command, ahrs=a.ahrs, ahrs_tau=a.ahrs_tau,
                crank=(lambda t, ck=ck: np.deg2rad(ck)))
        r.update(v=v, offset=off, angle=ang, lead=lead, crank=ck,
                 t_step=trig.get("t", np.nan))
        rows.append(r)
    rows.sort(key=lambda r: -r["ball_speed"])
    print(f"policy {a.policy}, ball at x = {a.ball_x} m; the bike starts at rest at x = 0")
    print("  v m/s  offset m  angle deg  lead m  crank  step at s   ball m/s  dir deg  hit by        fell")
    for r in rows:
        print(f"  {r['v']:5.2f}  {r['offset']:+7.3f}  {r['angle']:+8.0f}  {r['lead']:6.2f}"
              f"  {r['crank']:+5.0f}"
              f"  {r['t_step']:8.2f}   {r['ball_speed']:7.2f}  {r['ball_dir']:+7.1f}  "
              f"{r['hit_by']:12s}  {'FELL' if r['fell'] else ''}")


def play(a, p):
    pl = load_play(a.play)

    def command(t, front_x, touched):
        return pl.at(t)

    # The play's own ball wins: the bike starts at the origin facing +x, so
    # its bike-frame position IS the world position here.
    balls = ([pl.ball] if pl.ball is not None and not a.ball_ys
             else [(a.ball_x, y) for y in (a.ball_ys or [a.ball_y])])
    for xy in balls:
        r = fly(p, a.policy, xy, command, T=pl.duration + 1.5,
                ahrs=a.ahrs, ahrs_tau=a.ahrs_tau,
                crank=None if pl.crank is None else pl.crank_at)
        print(f"{pl.name}, ball at ({xy[0]}, {xy[1]:+.3f}): ball "
              f"{r['ball_speed']:.2f} m/s at {r['ball_dir']:+.1f} deg, hit by {r['hit_by']}"
              + (", FELL" if r["fell"] else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--policy", default="general_puffer_seed1_72M")
    ap.add_argument("--ball-x", type=float, default=1.0)
    ap.add_argument("--ball-y", type=float, default=-0.08, help="--play only")
    ap.add_argument("--ball-ys", type=float, nargs="+", help="--play: several ball y")
    ap.add_argument("--v", type=float, nargs="+", default=[0.8, 1.2])
    ap.add_argument("--offset", type=float, nargs="+", default=[-0.05, -0.10, -0.15],
                    help="ball y [m]; - = right of the path")
    ap.add_argument("--angle", type=float, nargs="+", default=[-60.0, 60.0])
    ap.add_argument("--lead", type=float, nargs="+", default=[0.05, 0.15])
    ap.add_argument("--crank", type=float, nargs="+", default=[0.0],
                    help="righting crank deg, held from the start (+ = right blade down)")
    ap.add_argument("--play", help="fly this play file instead of the sweep")
    ap.add_argument("--ahrs", default="none")
    ap.add_argument("--ahrs-tau", type=float)
    a = ap.parse_args()
    p = load_params()
    play(a, p) if a.play else sweep(a, p)


if __name__ == "__main__":
    main()
