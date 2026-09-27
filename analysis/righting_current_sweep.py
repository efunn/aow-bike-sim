"""What Goal Current does the swing linkage need to right the bike?

The number teleop found by hand (2026-09-26, 12 V: 525 counts lifts a little,
605 partway, 625 rights it), made reproducible. The design figure,
`limits.torque_nm` converted to counts, is a BUDGET the optimiser allowed --
not what the stroke needs -- so it is not a prediction of this; running the
stroke is.

Each trial, as teleop does it: the bike dropped onto its side and settled with
the crank held at centre, then the crank goal stepped straight to
+-`crank_travel_deg` (teleop's 9 / 4; no slew -- mode 5 has no profile), the
crank driven by CurrentBasedPositionServo at the trial's Goal Current, the steer
with its gearbox friction. Wheels passive: no policy, so this is the
mechanism's own lift. RIGHTED = the chassis comes within `--level-deg` of
upright at some point: getting it there is the mechanism's job, catching it
is the policy's (with passive wheels it then tips over the far side, which
`went over` reports). Peak motor torque is over the LIFT only, up to the
least roll -- after that the falling bike back-drives the crank past its
no-load speed and the number means something else.

Checked against teleop, 12 V (2026-09-26): sim 525 counts 80 -> 75 deg, 600
-> 64, first upright at 630 (10-count steps); by hand 525 lifted a little, 605
partway, 625 righted it.

    python analysis/righting_current_sweep.py
    python analysis/righting_current_sweep.py --supply 11.1 --counts 500 900 20
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from aow_sim import gearbox_friction  # noqa: E402
from aow_sim.build_model import SWING_LINKAGE_CFG, build_model, load_params  # noqa: E402
from aow_sim.control.linearize import settle_upright  # noqa: E402
from aow_sim.control.righting import roll_pitch  # noqa: E402
from aow_sim.righting_servo import CURRENT_LIMIT, CurrentBasedPositionServo  # noqa: E402


def fallen(params, side: float, settle_s: float):
    """Model, data and servo with the bike settled on its side, crank at centre."""
    m = build_model(params, righting=True, swing_linkage=True)
    d = mujoco.MjData(m)
    d.qpos[:] = settle_upright(m).qpos
    a = np.deg2rad(100.0 * side) / 2
    d.qpos[3:7] = [np.cos(a), np.sin(a), 0.0, 0.0]
    d.qpos[2] += 0.02
    mujoco.mj_forward(m, d)
    srv = CurrentBasedPositionServo.attach(m, params)
    srv.set_goal_current(CURRENT_LIMIT)
    hooks = [srv, gearbox_friction.attach_steer(m, params)]
    d.ctrl[m.actuator("swing").id] = 0.0
    step(m, d, hooks, settle_s)
    return m, d, srv, hooks


def step(m, d, hooks, seconds, on=None):
    for _ in range(int(round(seconds / m.opt.timestep))):
        for h in hooks:
            h.pre_step(d)
        mujoco.mj_step(m, d)
        if on is not None:
            on(d)


def trial(params, side, goal, counts, supply, seconds, level_deg, start, slew=None):
    m, d, srv, hooks = start
    d = mujoco.MjData(m)
    d.qpos[:], d.qvel[:] = start[1].qpos, start[1].qvel
    mujoco.mj_forward(m, d)
    srv.set_supply(supply)
    srv.set_goal_current(counts)
    aid = m.actuator("swing").id
    d.ctrl[aid] = goal if slew is None else 0.0
    rolls, taus = [], []

    def on(dd):
        if slew is not None:         # a ramped goal: no momentum to spend
            dd.ctrl[aid] = float(np.clip(dd.time * slew * np.sign(goal),
                                         -abs(goal), abs(goal)))
        rolls.append(roll_pitch(dd.qpos[3:7])[0])
        taus.append(srv.torque)
    step(m, d, hooks, seconds, on)
    r = np.abs(np.array(rolls))
    k = int(r.argmin())
    return {"min_roll": float(r[k]), "end_roll": float(r[-1]),
            "righted": bool(r[k] < level_deg),
            "over": bool(r[k] < level_deg and r[-1] > level_deg),
            "peak_tau": float(np.abs(taus[:k + 1]).max())}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--counts", type=int, nargs=3, default=[400, 900, 20],
                    metavar=("FROM", "TO", "STEP"))
    ap.add_argument("--supply", type=float, default=12.0, help="pack voltage [V]")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--settle-s", type=float, default=2.0)
    ap.add_argument("--level-deg", type=float, default=5.0)
    ap.add_argument("--slew-dps", type=float, default=None,
                    help="ramp the crank goal at this rate instead of stepping it: "
                         "slow enough and the lift is quasi-static, so the "
                         "threshold is the stroke's peak torque, with no swing "
                         "momentum carrying it over the hump (pair with a longer "
                         "--seconds)")
    args = ap.parse_args()

    params = load_params()
    dep = np.deg2rad(float(yaml.safe_load(SWING_LINKAGE_CFG.read_text())
                           ["stroke"]["crank_travel_deg"]))
    counts = list(range(args.counts[0], args.counts[1] + 1, args.counts[2]))
    s = args.supply / 12.0
    print(f"swing linkage, {args.supply:g} V, crank goal +-{np.degrees(dep):.0f} deg, "
          f"{args.seconds:g} s per trial, wheels passive; righted = reached within "
          f"{args.level_deg:g} deg of upright\n")
    for side in (+1.0, -1.0):
        start = fallen(params, side, args.settle_s)
        r0 = roll_pitch(start[1].qpos[3:7])[0]
        # The goal that pushes the bike up: whichever sign reduces the roll
        # more at the Current Limit. The other one pushes into the floor.
        best = {g: trial(params, side, g, CURRENT_LIMIT, s, args.seconds,
                         args.level_deg, start)["min_roll"] for g in (dep, -dep)}
        goal = min(best, key=best.get)
        print(f"fallen at {r0:+.0f} deg roll, crank goal {np.degrees(goal):+.0f} deg")
        print(f"  {'counts':>6} {'min |roll|':>10} {'lift peak tau':>13}  righted")
        first = None
        for c in counts:
            t = trial(params, side, goal, c, s, args.seconds, args.level_deg, start,
                      None if args.slew_dps is None else np.deg2rad(args.slew_dps))
            print(f"  {c:>6} {t['min_roll']:>9.1f}° {t['peak_tau']:>10.3f} N m  "
                  + ("YES" + (" (went over)" if t["over"] else "") if t["righted"] else ""))
            if t["righted"] and first is None:
                first = c
        print(f"  -> first to right it: {first if first is not None else 'none'} counts\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
