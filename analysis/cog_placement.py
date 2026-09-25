"""Where should the bike's CoG be? Plant-only numbers over a grid of CoG positions.

Three questions, each answered without a controller in the loop:

  FALL      time for the uncontrolled bike to fall from a small lean. The
            roll axis is the line through both contacts; I theta'' =
            m g h sin(theta), integrated from 1 deg to 30 deg and to 60 deg
            (`fall_roll_deg`, where training episodes end).
  TURN      yaw inertia, which is what every turn, pivot and flick has to
            accelerate: about the CoG, about the rear contact and about the
            front contact.
  TRACTION  the static front/rear split, the forward acceleration that
            unloads the front wheel (steering authority gone), and the
            rear-drive acceleration and braking limits including load
            transfer. Braking is the weaker, and a hold-station reversal is
            half braking.

THE INERTIA ABOUT THE CoG IS HELD FIXED while the CoG moves: the whole bike's
radius of gyration is taken from the current model and the parallel-axis
terms are recomputed at each grid point. That is "move the heavy parts
without changing what they are", an approximation. The current model's own
CoG rests on `chassis.mass 0.45 GUESS`, 44 % of the bike; mu is
`friction_sliding`, also a GUESS until F3 runs.

    python analysis/cog_placement.py
    python analysis/cog_placement.py --heights 80 100 124 150 --ahead 60 83 100

Read-only: builds the model, changes nothing.
"""
from __future__ import annotations

import argparse
import math

import mujoco
import numpy as np

from aow_sim.build_model import build_model, load_params
from aow_sim.control.linearize import settle_upright

G = 9.81


def whole_body(model, data):
    """Mass, CoG and the inertia tensor about the CoG, world frame."""
    m = float(model.body_mass[1:].sum())
    c = data.subtree_com[0].copy()
    inertia = np.zeros((3, 3))
    for i in range(1, model.nbody):
        mi = model.body_mass[i]
        R = data.ximat[i].reshape(3, 3)
        d = data.xipos[i] - c
        inertia += R @ np.diag(model.body_inertia[i]) @ R.T
        inertia += mi * (d @ d * np.eye(3) - np.outer(d, d))
    return m, c, inertia


def fall_time(inertia_contact, m, h, deg_to):
    """Seconds from 1 deg to `deg_to`, starting at rest. Plant only."""
    th, w, t, dt = math.radians(1.0), 0.0, 0.0, 1e-5
    k = m * G * h / inertia_contact
    while th < math.radians(deg_to):
        w += k * math.sin(th) * dt
        th += w * dt
        t += dt
    return t


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--heights", type=float, nargs="+",
                    default=[80, 100, 124, 150, 180], help="CoG height above floor, mm")
    ap.add_argument("--ahead", type=float, nargs="+",
                    default=[50, 83, 100, 130], help="CoG ahead of the rear contact, mm")
    args = ap.parse_args()

    p = load_params()
    model = build_model(p)
    data = settle_upright(model)
    m, c, Ic = whole_body(model, data)
    wb = p["bike"]["wheelbase"]
    mu = p["sim"]["friction_sliding"]
    # Rear contact: directly under the rear hub, on the floor (z = 0).
    x_rear = data.xpos[model.body("aow_hub").id][0]
    ahead0, h0 = (c[0] - x_rear) * 1000, c[2] * 1000

    print(f"model: {m*1000:.0f} g, CoG {ahead0:.0f} mm ahead of the rear contact, "
          f"{h0:.0f} mm up (chassis mass is a GUESS)")
    print(f"inertia about CoG [g m^2]: roll {Ic[0,0]*1e3:.3f}  pitch {Ic[1,1]*1e3:.3f}  "
          f"yaw {Ic[2,2]*1e3:.3f};  wheelbase {wb*1000:.0f} mm, mu {mu}\n")

    print("FALL (plant only, from 1 deg; roll about the contact line)")
    print(f"{'h mm':>6}{'tau ms':>8}{'->30deg ms':>12}{'->60deg ms':>12}")
    for h in args.heights:
        hm = h / 1000
        I = Ic[0, 0] + m * hm * hm
        tau = math.sqrt(I / (m * G * hm))
        print(f"{h:6.0f}{tau*1000:8.1f}{fall_time(I, m, hm, 30)*1000:12.0f}"
              f"{fall_time(I, m, hm, 60)*1000:12.0f}")

    print("\nTURN (yaw inertia, g m^2)")
    print(f"{'ahead mm':>9}{'about CoG':>11}{'about rear':>12}{'about front':>13}")
    for a in args.ahead:
        am = a / 1000
        print(f"{a:9.0f}{Ic[2,2]*1e3:11.3f}{(Ic[2,2] + m*am*am)*1e3:12.3f}"
              f"{(Ic[2,2] + m*(wb-am)**2)*1e3:13.3f}")

    print(f"\nTRACTION (mu {mu}; accelerations in g)")
    print(f"{'ahead mm':>9}{'h mm':>6}{'rear %':>8}{'front-lift':>12}"
          f"{'rear accel':>12}{'rear brake':>12}")
    for a in args.ahead:
        for h in args.heights:
            am, hm = a / 1000, h / 1000
            rear = (wb - am) / wb
            lift = am / hm                                  # a at which N_front -> 0
            acc = mu * rear / (1 - mu * hm / wb) if mu * hm < wb else float("inf")
            brk = mu * rear / (1 + mu * hm / wb)
            print(f"{a:9.0f}{h:6.0f}{rear*100:8.0f}{lift:12.2f}"
                  f"{min(acc, lift):12.2f}{brk:12.2f}")
    print("\nrear accel is capped at front-lift: past it the front wheel is off the floor.")



if __name__ == "__main__":
    main()
