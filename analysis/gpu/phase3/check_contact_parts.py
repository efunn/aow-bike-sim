"""Do MJWarp and C MuJoCo agree on the wheels' own contacts?

    PYTHONPATH=src python analysis/gpu/phase3/check_contact_parts.py            # CUDA if there is one
    PYTHONPATH=src python analysis/gpu/phase3/check_contact_parts.py --device cpu

Since 2026-10-31 bike_params gives the front tire and the rollers their own
solref in MuJoCo's NEGATIVE form (-stiffness, -damping; build_model's
sim.contact_parts), dictated by geom priority over the floor's positive pair.
No GPU run has trained on either. This flies the pivot drop
(analysis/contact/pivot_drop.py: one end lifted 15 mm about the other
wheel's contact, let go) in both engines from the same state and compares the
chassis pitch rate at 200 Hz -- the fall, the impact and the rebound, which
is where the contact's damping shows. The same drop with the shipped
positive pair on every contact is the reference: the two engines' gap there
is the floor this one has to sit inside.
"""
import argparse
import copy
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "analysis" / "contact"))


def trace_c(m, d0, n, every):
    d = copy.deepcopy(d0)
    w = []
    for i in range(n):
        mujoco.mj_step(m, d)
        if i % every == 0:
            w.append(d.qvel[4])
    return np.array(w)


def trace_warp(m, d0, n, every, device):
    import warp as wp
    import mujoco_warp as mjw
    with wp.ScopedDevice(device):
        mw = mjw.put_model(m)
        dw = mjw.put_data(m, d0, nconmax=64, njmax=256)
        w = []
        for i in range(n):
            mjw.step(mw, dw)
            if i % every == 0:
                w.append(float(dw.qvel.numpy()[0, 4]))
    return np.array(w)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--device", default=None, help="warp device (default: CUDA if present)")
    ap.add_argument("--h0-mm", type=float, default=15.0)
    ap.add_argument("--seconds", type=float, default=0.3)
    a = ap.parse_args()
    import warp as wp
    wp.config.quiet = True
    wp.init()
    device = a.device or ("cuda:0" if wp.is_cuda_available() else "cpu")

    from aow_sim.build_model import build_model, load_params
    from aow_sim.params import with_contact_solref
    from pivot_drop import HZ, fall_rebound, place

    p0 = load_params()
    cases = {"fitted (bike_params)": p0,
             "shipped [0.005, 1.0] everywhere": with_contact_solref(p0, p0["sim"]["contact_solref"])}
    print(f"MJWarp on {device}, pivot drop {a.h0_mm:g} mm, chassis pitch rate at {HZ:g} Hz")
    for label, p in cases.items():
        for end in ("front", "rear"):
            r = build_model(p)
            m = r[0] if isinstance(r, tuple) else r
            d = mujoco.MjData(m)
            place(m, d, p, end, a.h0_mm)
            n, every = int(a.seconds / m.opt.timestep), max(1, round(1 / HZ / m.opt.timestep))
            wc, ww = trace_c(m, d, n, every), trace_warp(m, d, n, every, device)
            fc, rc, _ = fall_rebound(wc[:int(0.4 * HZ)])
            fw, rw, _ = fall_rebound(ww[:int(0.4 * HZ)])
            print(f"  {label:32s} {end:5s}: C fall/rebound {np.degrees(fc):6.1f}/{np.degrees(rc):5.1f} deg/s, "
                  f"Warp {np.degrees(fw):6.1f}/{np.degrees(rw):5.1f}; rebound/fall C {rc / fc:.3f} "
                  f"Warp {rw / fw:.3f}; max |diff| {np.degrees(np.abs(wc - ww).max()):.1f} deg/s")


if __name__ == "__main__":
    main()
