"""Lie down and get up on the swing module: control/swing_sit.py, flown.

docs/plans/wishlist.md, "getting up and down cleanly". The bike starts
upright under the general policy, lies down onto one blade, lies still, gets
back up and rides. Then the same from a FALL: the policy cut with the bike
leaning, as auto-up in teleop sees it.

Reported, per run:
  down   time to lying, peak |roll rate| from the policy cut to lying, peak
         floor force on the CHASSIS (anything not a wheel or a blade) and on
         the blades, and whether it ended still
  up     time from the start of the lift to the hand-off and to riding, and
         the worst |roll| in the 2 s after the hand-off

For scale: a FALL from upright onto the side -- the first lie-down trials,
cut with the blade at the full stroke, which props the bike over the other
way -- read ~720-780 deg/s and ~150-200 N on the chassis the same way.

MEASURED 2026-10-16, second version (closed-loop kick, eased hand-off,
stop first), each policy on ITS OWN drivetrain (the detailed XC430 loop
for these two, as teleop compiles it); seed1_72M and fixes_52M, truth and
--ahrs tm151_filter --ahrs-tau 2.0 all within these ranges:

  down, either side   lying after 5.8 s, peak 96-102 deg/s, chassis
                      6.7-11.7 N, blade ~14.5 N; still at 79-80 deg
  up from lying       hand-off 1.5 s, riding 3.3 s, worst |roll| after
                      the hand-off 11.2-12.0 deg (the hand-off angle)
  auto-up, a fall     detected 3.7 s after the shove; hand-off 1.0 s,
                      riding 2.8 s, worst 10.6-12.0 deg
  teleop, as launched (PageDown, wait, PageUp; 8 tries each; fixes_52M
  truth, fixes_52M and seed1_72M under the AHRS model; half of them
  pressed while driving): 24/24 down on the chosen side and back up.

THE FIRST VERSION, tuned on the IDEAL drive, went wrong in teleop on the
detailed one (the user's report, reproduced 2/6): a fixed 0.2 s kick
sometimes left the bike leaning away -- the kick's first effect is a
brief roll the wrong way -- so it fell onto the bladeless side; and the
policy leaned ~5 deg onto the still-planted blade after the hand-off, so
folding it at once dropped the bike onto that side. Hence the closed-loop
kick (with an abort), the eased release, and --ideal for the old plant.

How the defaults were chosen (sweeps, truth, seed1_72M):
  lean crank 110      116.9 (full) props the bike over the other way in
                      every trial; 100 and 110 with a small kick also
                      fell the wrong way (ideal drive)
  kick 20, closed     10 rad/s aborted 2/8 on the detailed drive (seed1,
                      AHRS); 20 rad/s held up to 1.5 s: 24/24
  lower 30 deg/s      60: 129 deg/s peak; 120: 211
  limp over 1 s       held at full current: rocks +-4 deg at ~35 deg/s
                      indefinitely; released at once: 152 deg/s, 40 N
  lift ramp 120/s     90-150 all four cases up; 180 failed one (over the
                      top); a STEP never came up (through upright at
                      ~300 deg/s, outside the hand-off gate); the policy
                      ON during the lift failed 2 of 3

    python analysis/moves/swing_sit.py
    python analysis/moves/swing_sit.py --policy general_puffer_fixes_52M --side -1
    python analysis/moves/swing_sit.py --lean 110 --kick 20 --lower-dps 30 --limp-s 1

The policy's own drivetrain (--ideal for the ideal one); truth state
unless --ahrs (then the policy AND the
sequence read the AHRS model's roll, as on the bike); the crank on
CurrentBasedPositionServo at teleop's starting Goal Current.
"""
from __future__ import annotations

import argparse
import contextlib
import io

import mujoco
import numpy as np

from aow_sim import drivetrain_model
from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import mix
from aow_sim.control.drive import DriveController
from aow_sim.control.linearize import settle_upright
from aow_sim.control.righting import roll_pitch
from aow_sim.control.swing_sit import SwingSit
from aow_sim.righting_servo import CurrentBasedPositionServo


class Sim:
    """The bike, its crank servo and the general policy, stepped together."""

    def __init__(self, params, policy, ahrs="none", ahrs_tau=None, ideal=False):
        # The policy's OWN drivetrain, as teleop compiles it: the detailed
        # XC430 loop, slop and detent for the GPU policies. The first tuning
        # (ideal drive) went wrong-side and dropped the bike after hand-off
        # on this plant in teleop.
        if not ideal:
            params = drivetrain_model.policy_params(params, policy)
        self.p, self.policy = params, policy
        m = self.m = build_model(params, variant="full")
        d = self.d = mujoco.MjData(m)
        d.qpos[:] = settle_upright(m).qpos
        mujoco.mj_forward(m, d)
        self.aid = m.actuator("swing").id
        self.crank_adr = m.joint("swing_crank_joint").qposadr[0]
        self.srv = CurrentBasedPositionServo.attach(
            m, params, torque_nm=float(m.actuator_forcerange[self.aid, 1]), at_output=True)
        self.gc = self.srv.goal_current
        self.hooks = drivetrain_model.attach_hooks(m, params)
        self.ahrs = None
        if ahrs != "none":
            from aow_sim.sim_ahrs import SimAhrs, level_tau_orient_s
            self.ahrs = SimAhrs(m, params, level=ahrs, tau_orient_s=(
                level_tau_orient_s(ahrs) if ahrs_tau is None else ahrs_tau))
        with contextlib.redirect_stdout(io.StringIO()):
            self.c = DriveController(params, m)
            self.c.reset(m, d)
        self.engage()
        self.crank_goal = 0.0
        self.steer_hold = None
        floor = m.geom("floor").id
        blade = {g for g in range(m.ngeom)
                 if m.body(m.geom_bodyid[g]).name.startswith("swing_wing")}
        wheel = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name not in
                 ("chassis",) and g not in blade}
        self.floor, self.blade, self.wheel = floor, blade, wheel

    def engage(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.c.engage_general(self.d, name=self.policy)
        self.c.set_command_polar(0.0, psi_cmd=self.c._psi)

    def seen(self):
        """The AHRS's view of the orientation (truth without --ahrs)."""
        return (contextlib.nullcontext() if self.ahrs is None
                else self.ahrs.estimated(self.d))

    def state(self):
        """(roll, roll rate, crank) as the bike would measure them."""
        with self.seen():
            roll, _ = roll_pitch(self.d.qpos[3:7])
            rate = float(self.d.qvel[3])
        return np.deg2rad(roll), rate, float(self.d.qpos[self.crank_adr])

    def step(self, cmd=None, policy_was=True):
        """Advance one physics step under `cmd` (None: policy, crank held)."""
        m, d, c = self.m, self.d, self.c
        on = cmd is None or cmd.policy
        if on and not policy_was:
            self.engage()
        if not on and self.steer_hold is None:
            self.steer_hold = float(d.ctrl[c.aid["steer"]])
        if on:
            self.steer_hold = None
        if self.ahrs is not None:
            self.ahrs.tick(d, m.opt.timestep)
        with self.seen():
            c.step(m, d)
        if not on:
            a, b = mix(0.0, cmd.diff)
            d.ctrl[c.aid["drive_a"]], d.ctrl[c.aid["drive_b"]] = a, b
            d.ctrl[c.aid["steer"]] = self.steer_hold
        if cmd is not None:
            self.crank_goal = cmd.crank
            self.srv.set_goal_current(round(self.gc * cmd.current))
        d.ctrl[self.aid] = self.crank_goal
        self.srv.pre_step(d)
        for h in self.hooks:
            h.pre_step(d)
        mujoco.mj_step(m, d)
        return on

    def floor_forces(self):
        """(chassis, blade) floor normal force this step [N]."""
        ch = bl = 0.0
        for i in range(self.d.ncon):
            con = self.d.contact[i]
            if self.floor not in (con.geom1, con.geom2):
                continue
            g = con.geom2 if con.geom1 == self.floor else con.geom1
            f = np.zeros(6)
            mujoco.mj_contactForce(self.m, self.d, i, f)
            if g in self.blade:
                bl += f[0]
            elif g not in self.wheel:
                ch += f[0]
        return ch, bl


def fly(sim, seq, until, T, start=None, record=None):
    """Run `seq` from `start(t)` until its phase is `until` (or T s)."""
    d, dt = sim.d, sim.m.opt.timestep
    t0 = d.time
    if start is not None:
        start(d.time)
    on = True
    out = dict(peak_rate=0.0, chassis=0.0, blade=0.0, t_end=None, hand=None, rolls=[])
    while d.time - t0 < T:
        roll, rate, crank = sim.state()
        cmd = seq.step(d.time, roll, rate, crank)
        if seq.phase == "balance" and out["hand"] is None:
            out["hand"] = d.time - t0
        on = sim.step(cmd, on)
        if not on:
            out["peak_rate"] = max(out["peak_rate"], abs(np.degrees(rate)))
            ch, bl = sim.floor_forces()
            out["chassis"], out["blade"] = max(out["chassis"], ch), max(out["blade"], bl)
        if out["hand"] is not None:
            out["rolls"].append((d.time - t0 - out["hand"], abs(np.degrees(roll))))
        if seq.phase == until:
            out["t_end"] = d.time - t0
            break
    return out


def hold(sim, seconds, cmd=None):
    on = cmd is None or cmd.policy
    for _ in range(int(seconds / sim.m.opt.timestep)):
        on = sim.step(cmd, on)


def make_seq(sim, a):
    return SwingSit(float(sim.m.actuator_ctrlrange[sim.aid, 1]), lean_deg=a.lean,
                    kick=a.kick, lower_dps=a.lower_dps, limp_s=a.limp_s,
                    lift_dps=a.lift_dps, lift_policy=a.lift_policy)


def report_down(tag, o, sim):
    roll, rate, _ = sim.state()
    print(f"{tag}: lying after {o['t_end'] or float('nan'):.2f} s, peak |roll rate| "
          f"{o['peak_rate']:.0f} deg/s, floor force chassis {o['chassis']:.1f} N / blade "
          f"{o['blade']:.1f} N; at rest roll {np.degrees(roll):+.1f} deg, "
          f"rate {np.degrees(rate):+.1f} deg/s")


def report_up(tag, o):
    if o["hand"] is None:
        print(f"{tag}: DID NOT COME UP")
        return
    after = [r for t, r in o["rolls"] if t <= 2.0]
    print(f"{tag}: hand-off after {o['hand']:.2f} s, riding after "
          f"{o['t_end'] or float('nan'):.2f} s, worst |roll| in the 2 s after "
          f"hand-off {max(after):.1f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--policy", default="general_puffer_seed1_72M")
    ap.add_argument("--side", type=float, nargs="*", default=[1.0, -1.0])
    ap.add_argument("--lean", type=float, default=110.0, help="crank deg for the lean")
    ap.add_argument("--kick", type=float, default=20.0,
                    help="diff kick rad/s, held until the bike leans 3 deg onto the blade")
    ap.add_argument("--lower-dps", type=float, default=30.0)
    ap.add_argument("--limp-s", type=float, default=1.0)
    ap.add_argument("--lift-dps", type=float, default=120.0,
                    help="crank goal ramp for the lift, deg/s; 0 = a step")
    ap.add_argument("--ahrs", default="none", help="e.g. tm151_filter")
    ap.add_argument("--ideal", action="store_true",
                    help="ideal drivetrain instead of the policy's own")
    ap.add_argument("--ahrs-tau", type=float)
    ap.add_argument("--lift-policy", action="store_true",
                    help="keep the policy driving during the lift")
    a = ap.parse_args()
    a.lift_dps = a.lift_dps or None
    p = load_params()
    for side in a.side:
        name = "right" if side > 0 else "left"
        sim = Sim(p, a.policy, a.ahrs, a.ahrs_tau, a.ideal)
        seq = make_seq(sim, a)
        hold(sim, 0.5)
        o = fly(sim, seq, "lying", 12.0,
                start=lambda t: seq.lie_down(t, side=side))
        hold(sim, 1.0, seq.step(sim.d.time, *sim.state()))
        report_down(f"down onto the {name} blade", o, sim)
        roll = sim.state()[0]
        o = fly(sim, seq, "riding", 8.0, start=lambda t: seq.get_up(t, roll))
        report_up(f"  up from the {name} side", o)

    # A FALL, as auto-up sees it: lean the bike and cut everything.
    for side in a.side:
        name = "right" if side > 0 else "left"
        sim = Sim(p, a.policy, a.ahrs, a.ahrs_tau, a.ideal)
        hold(sim, 0.5)
        seq = make_seq(sim, a)
        sim.d.qvel[3] = side * 2.0                 # a shove: 2 rad/s of roll
        from aow_sim.control.swing_sit import Command
        t = sim.d.time
        while sim.d.time - t < 4.0:                # policy off: drives held
            r, rr, _ = sim.state()
            if seq.fallen(sim.d.time, r, rr):
                break
            sim.step(Command(0.0, policy=False), False)
        r, rr, _ = sim.state()
        print(f"fell {name}: roll {np.degrees(r):+.1f} deg after {sim.d.time - t:.2f} s")
        o = fly(sim, seq, "riding", 8.0, start=lambda tt: seq.get_up(tt, r))
        report_up(f"  auto-up from the {name} side", o)


if __name__ == "__main__":
    main()
