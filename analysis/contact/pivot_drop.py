"""The pivot drop: one end of the bike lifted about the other wheel's contact
and let go, read off the bike's own AHRS, and the contact's damping fitted
per wheel against the same drop in the sim.

    python analysis/contact/pivot_drop.py traces/ahrs/261031-134407_pivot_drops.csv
    python analysis/contact/pivot_drop.py <csv> --plot analysis/contact/plots/pivot_drop.png

Why (2026-10-31): the drop rig's contact was 2-4x softer than the hand drops
and the dial on the same tire, so its fit carried the rig's own give. This
drops the real bike on the real floor: the real tire, the real rollers, the
mass and inertia each contact actually carries, and no rig.

RECORDING: `bench/ahrs_stream.py --record pivot_drops` on the Pi, motors off,
wings stowed; each drop held still ~1 s, released without a push, ~2 s still
after. The TM151 sits pitched ~45 deg on the bike, so its y gyro is the
bike's pitch rate.

WHAT A DROP GIVES. Before release, the held pitch against the settled pitch
after it is the lift angle, so the drop height needs no ruler: h0 = L sin(th),
L the wheelbase. Then the pitch rate at 200 Hz: the fall's peak just before
the impact and the rebound's peak just after, the ratio of the two (`e` here)
read the same way off the sim's chassis rate sampled at the same 200 Hz.
Matching that ratio, not a restitution computed from it, is the point: the
sampling's own bias cancels. The sim's fall rate at the same h0 checks the
drop (a push or a drag at release shows as a fall rate off the sim's) and,
across drops, the bike's pitch inertia.

WHAT IT CANNOT GIVE: the stiffness. The contact lasts a few ms, under one
200 Hz sample; timeconst 0.005 vs 0.01 moved the simulated rebound 2 deg/s.
So the contact is written in MuJoCo's NEGATIVE solref, (-stiffness,
-damping), with the stiffness held at the shipped [0.005, 1.0]'s own (k =
1/(dmax tc)^2, b = 2/(dmax tc) at dampratio 1) and the damping alone fitted,
one per wheel through `sim.contact_parts` (front_tire, roller). Nothing is
written to bike_params.yaml.

`--check` confirms the negative form at the shipped damping reproduces the
shipped positive solref's drop.
"""
from __future__ import annotations

import argparse
import copy
import csv
import sys

import mujoco
import numpy as np

from aow_sim.build_model import build_model, load_params
from aow_sim.params import with_contact_solref

HZ = 200.0
HOLD_S = 0.3            # still for this long before release
HOLD_STD_DEG = 0.15
MIN_LIFT_DEG = 1.5
B_GRID_FRAC = (0.15, 0.25, 0.35, 0.5, 0.7, 1.0)   # damping, fractions of the shipped (critical)


def shipped_kb(p: dict) -> tuple[float, float]:
    """The shipped positive solref as (stiffness, damping) at the solimp's dmax."""
    tc, dr = p["sim"]["contact_solref"]
    dmax = p["sim"]["contact_solimp"][1]
    return 1.0 / (dmax * tc * dr) ** 2, 2.0 / (dmax * tc)


def wheelbase(p: dict) -> float:
    w = p["bike"]["wheelbase"]
    return float(w["value"] if isinstance(w, dict) else w)


# -- the recording ------------------------------------------------------------

def load(path: str):
    t, pitch, roll, gy = [], [], [], []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            if r["kind"] != "combo":
                continue
            t.append(int(r["t_us"]) * 1e-6)
            pitch.append(float(r["pitch_deg"]))
            roll.append(float(r["roll_deg"]))
            gy.append(float(r["gy_rad_s"]))
    t = np.array(t)
    return t - t[0], np.array(pitch), np.array(roll), np.array(gy)


def fall_rebound(w) -> tuple[float, float, int]:
    """(fall peak, rebound peak, impact index) of a pitch-rate trace from the
    release: the fall is the largest rate, the impact the first sample after
    it whose sign has turned, the rebound the largest rate the other way in
    the 60 ms after that."""
    i_pk = int(np.argmax(np.abs(w)))
    s = np.sign(w[i_pk])
    turned = np.nonzero(s * w[i_pk:] <= 0)[0]
    if not len(turned):
        return abs(w[i_pk]), 0.0, len(w)
    k = i_pk + int(turned[0])
    reb = -s * w[k:k + int(0.06 * HZ)]
    return abs(w[i_pk]), max(float(reb.max()), 0.0), k


def rest_pitch(pitch, gy) -> float:
    """The pitch the bike sits at on both wheels: the commonest still pitch."""
    still = np.abs(gy) < 0.02
    h, edges = np.histogram(pitch[still], bins=np.arange(pitch.min(), pitch.max() + 0.25, 0.25))
    return float(edges[np.argmax(h)] + 0.125)


def find_drops(t, pitch, roll, gy):
    """Each release from a still hold away from rest, to the rebound. A hold
    must be MIN_LIFT_DEG off the bike's rest pitch and the pitch must settle
    back within 1 deg of it, which rejects the lift itself (a still stretch at
    rest, then a move away from it)."""
    n_hold = int(HOLD_S * HZ)
    rest0 = rest_pitch(pitch, gy)
    drops, i = [], n_hold
    while i < len(t) - int(1.5 * HZ):
        w = pitch[i - n_hold:i]
        if w.std() > HOLD_STD_DEG or np.abs(gy[i - n_hold:i]).max() > 0.05:
            i += 1
            continue
        # released: the rate leaves the hold
        j = i
        while j < len(t) - 1 and abs(gy[j]) < 0.15 and abs(pitch[j] - w.mean()) < 0.3:
            j += 1
        if j >= len(t) - int(1.0 * HZ):
            break
        if abs(w.mean() - rest0) < MIN_LIFT_DEG:
            i = j + 1           # still at rest: what follows is a lift, not a drop
            continue
        # the settled pitch after: 0.5-1.0 s past the release
        after = pitch[j + int(0.5 * HZ):j + int(1.0 * HZ)]
        rest = np.median(after)
        lift = w.mean() - rest
        if abs(rest - rest0) > 1.0 or np.abs(after - rest).max() > 0.5:
            i = j + 1
            continue
        fall, rebound, k = fall_rebound(gy[j:j + int(0.4 * HZ)])
        drops.append(dict(t=t[j], lift_deg=lift, roll_deg=roll[j], fall=fall,
                          rebound=rebound, impact_ms=1000 * k / HZ))
        i = j + int(1.0 * HZ)
    return drops


# -- the sim ------------------------------------------------------------------

def place(m, d, p: dict, end: str, h0_mm: float) -> None:
    """Pose the bike with `end` lifted h0_mm about the other wheel's contact,
    still, and run mj_forward: the instant of release."""
    L = wheelbase(p)
    mujoco.mj_forward(m, d)
    z0 = d.qpos[2]                                       # the rear axle's height at qpos0
    th = np.arcsin(h0_mm * 1e-3 / L)
    piv = np.array([0.0, 0, 0]) if end == "front" else np.array([L, 0, 0])
    ang = -th if end == "front" else th                  # nose up / tail up
    q = np.array([np.cos(ang / 2), 0, np.sin(ang / 2), 0])
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    d.qpos[3:7] = q
    d.qpos[0:3] = piv + R.reshape(3, 3) @ (np.array([0, 0, z0]) - piv)
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)


def sim_drop(p: dict, end: str, h0_mm: float, part_solref=None, global_solref=None,
             seconds: float = 0.5, lowpass_s: float = 0.0):
    """Lift `end` h0_mm about the other wheel's contact, release, and return the
    chassis pitch rate sampled at HZ, as (fall peak, rebound peak) [rad/s]."""
    p = copy.deepcopy(p)
    if global_solref is not None:
        p = with_contact_solref(p, global_solref)
    if part_solref is not None:          # the other wheel keeps bike_params' own
        part = "front_tire" if end == "front" else "roller"
        p["sim"]["contact_parts"] = {**(p["sim"].get("contact_parts") or {}),
                                     part: {"solref": list(part_solref)}}
    r = build_model(p)
    m = r[0] if isinstance(r, tuple) else r
    d = mujoco.MjData(m)
    place(m, d, p, end, h0_mm)
    every = max(1, round(1.0 / HZ / m.opt.timestep))
    w = []
    a = 1.0 if lowpass_s <= 0 else 1.0 - np.exp(-m.opt.timestep / lowpass_s)
    rate = 0.0
    for i in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)
        rate += a * (d.qvel[4] - rate)              # the gyro's own smoothing, if any
        if i % every == 0:
            w.append(rate)
    fall, rebound, _k = fall_rebound(np.array(w[:int(0.4 * HZ)]))
    return fall, rebound


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv", help="bench/ahrs_stream.py --record output")
    ap.add_argument("--check", action="store_true",
                    help="the negative form at the shipped damping against the shipped solref")
    ap.add_argument("--plot", default=None, help="write a figure here")
    ap.add_argument("--gyro-lowpass-ms", type=float, default=0.0,
                    help="first-order smoothing applied to the sim's rate before sampling. "
                         "Default 0: the 10-07 drop rig runs bound the TM151's own at ~3 ms "
                         "(impact peaks at or above free fall's 28-63 deg/s; 10 ms would cut "
                         "them by ~23). The bike's slower falls here are the hand's release")
    ap.add_argument("--clean", type=float, default=0.3,
                    help="keep drops whose fall peak is within this fraction of the sim's")
    a = ap.parse_args()

    p = load_params()
    k0, b0 = shipped_kb(p)
    L = wheelbase(p)
    if a.check:
        for end in ("front", "rear"):
            pos = sim_drop(p, end, 15.0)
            neg = sim_drop(p, end, 15.0, global_solref=[-k0, -b0])
            print(f"check {end}: shipped solref fall/rebound {np.degrees(pos[0]):.1f}/{np.degrees(pos[1]):.1f} "
                  f"deg/s, negative ({-k0:.0f}, {-b0:.1f}) {np.degrees(neg[0]):.1f}/{np.degrees(neg[1]):.1f}")

    t, pitch, roll, gy = load(a.csv)
    drops = find_drops(t, pitch, roll, gy)
    # front lifted = pitch rises nose-up; the TM151 reads that as pitch going UP
    # (less negative here) -- decided from the data's two halves, then checked:
    # the front phase is the first half of the recording.
    for dr in drops:
        dr["end"] = "front" if dr["lift_deg"] > 0 else "rear"
        dr["h0_mm"] = 1000 * L * np.sin(np.radians(abs(dr["lift_deg"])))
    print(f"{a.csv}: {len(drops)} drops found, "
          f"{sum(d['end'] == 'front' for d in drops)} front, {sum(d['end'] == 'rear' for d in drops)} rear")

    bs = [f * b0 for f in B_GRID_FRAC]
    rows = []
    for dr in drops:
        sims = [sim_drop(p, dr["end"], dr["h0_mm"], part_solref=[-k0, -b],
                         lowpass_s=a.gyro_lowpass_ms * 1e-3) for b in bs]
        e_sim = np.array([r / f for f, r in sims])
        e_meas = dr["rebound"] / dr["fall"]
        # e falls as damping rises: interpolate the damping that gives e_meas
        order = np.argsort(e_sim)
        b_fit = float(np.interp(e_meas, e_sim[order], np.array(bs)[order], left=np.nan, right=np.nan))
        fall_sim = np.mean([f for f, _ in sims])
        rows.append((dr, e_meas, b_fit, fall_sim, e_sim))
        print(f"  t {dr['t']:6.1f} s  {dr['end']:5s} lift {abs(dr['lift_deg']):4.1f} deg = {dr['h0_mm']:4.1f} mm  "
              f"roll {dr['roll_deg']:+5.1f}  fall {np.degrees(dr['fall']):4.0f} deg/s (sim {np.degrees(fall_sim):4.0f})  "
              f"rebound {np.degrees(dr['rebound']):3.0f}  e {e_meas:.2f}  "
              f"-> damping {b_fit / b0 if b_fit == b_fit else float('nan'):.2f} x shipped "
              f"(sim e {e_sim.min():.2f}-{e_sim.max():.2f} over the grid)")

    print(f"\nshipped contact: stiffness {k0:.0f}, damping {b0:.1f} (negative-solref units); "
          f"damping below as a fraction of that (1.0 = the shipped, critically damped)")
    for end, part in (("front", "front_tire"), ("rear", "roller")):
        sel = [r for r in rows if r[0]["end"] == end]
        ok = [r for r in sel if r[2] == r[2] and abs(r[0]["fall"] / r[3] - 1) < a.clean]
        if not sel:
            continue
        fr = np.array([r[2] / b0 for r in ok])
        msg = (f"median {np.median(fr):.2f} (IQR {np.percentile(fr, 25):.2f}-{np.percentile(fr, 75):.2f}) "
               f"-> sim.contact_parts.{part}.solref [{-k0:.0f}, {-np.median(fr) * b0:.1f}]") if len(fr) else "none usable"
        print(f"{end:5s}: {len(sel)} drops, {len(ok)} with a clean release (fall within {a.clean:.0%} of the sim's) "
              f"and inside the grid: damping {msg}")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(10, 4))
        for c, end in ((0, "front"), (1, "rear")):
            sel = [r for r in rows if r[0]["end"] == end]
            ax[c].scatter([r[0]["h0_mm"] for r in sel], [r[1] for r in sel], c="k", s=14, label="bike")
            for i, f in enumerate(B_GRID_FRAC):
                ax[c].plot([r[0]["h0_mm"] for r in sel], [r[4][i] for r in sel], ".", ms=4,
                           label=f"sim, damping {f:g} x")
            ax[c].set_title(f"{end} lifted")
            ax[c].set_xlabel("lift at the wheel [mm]")
            ax[c].set_ylabel("rebound / fall pitch rate, 200 Hz")
        ax[1].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print(f"wrote {a.plot}")


if __name__ == "__main__":
    main()
