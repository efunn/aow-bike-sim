"""Export everything the onboard controller needs, so the bike carries no
MuJoCo model and no scipy.

Run on the laptop:

    python -m aow_sim.export_deploy                 # -> deploy/bundle.npz
    python -m aow_sim.export_deploy --no-payload    # the tethered bike

Building a DriveController normally costs two numerical linearizations of the
MuJoCo model (design_lqr + design_gain_schedule) — minutes of rollouts, and
scipy for the Riccati solve. That is the wrong thing to ask of a Pi Zero 2 W
at every boot, and scipy on ARM is a nuisance to install besides.

So the laptop does it once and ships the answer: the gain schedule, the
equilibrium pose, and the handful of model constants the controllers read
(actuator ids, ctrlranges, joint addresses, nq/nv/nu). `aow_sim.hw.state`
loads the result into a DeployModel + LQRDesign and constructs the SAME
DriveController class the simulator uses.

This is the same trick moves/*.npz already plays for torch: training needs
it, replay does not. See docs/plans/untethered-setup.md.

The bundle is tied to one model. `params_digest` pins the parameters it was
built from, and hw.state refuses a bundle whose digest does not match the
bike_params.yaml on the robot — a silently stale gain schedule on a balancing
robot is a fall, not a warning.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from .build_model import build_model, load_params
from .control.drive import DriveController
from .control.linearize import design_all

# Actuators and joints the controllers look up by name.
ACTUATORS = ("drive_a", "drive_b", "steer")
STEER_JOINT = "steer_joint"


def params_digest(params: dict) -> str:
    """Stable hash of the parameter set a bundle was designed for."""
    blob = json.dumps(params, sort_keys=True, default=float).encode()
    return hashlib.sha256(blob).hexdigest()[:16]


def build_bundle(params: dict, payload: bool = True) -> dict:
    model = build_model(params, variant="full", payload=payload)
    design = design_all(params, model)
    sj = model.joint(STEER_JOINT)
    return {
        # -- LQR design (the expensive part) --
        "K": design.K,
        "qpos_eq": design.qpos_eq,
        "fit_r2": design.fit_r2,
        "speeds": design.speeds,
        "Ks": design.Ks,
        "fit_r2_grid": design.fit_r2_grid,
        # -- model constants the controllers read --
        "nq": np.array(model.nq),
        "nv": np.array(model.nv),
        "nu": np.array(model.nu),
        "actuator_ctrlrange": model.actuator_ctrlrange.copy(),
        "actuator_ctrllimited": model.actuator_ctrllimited.copy(),
        "actuator_names": np.array(ACTUATORS),
        "actuator_ids": np.array([model.actuator(n).id for n in ACTUATORS]),
        "steer_qposadr": np.array(sj.qposadr[0]),
        "steer_dofadr": np.array(sj.dofadr[0]),
        # -- provenance --
        "params_digest": np.array(params_digest(params)),
        "payload": np.array(payload),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=None, help="path to bike_params.yaml")
    ap.add_argument("--no-payload", action="store_true",
                    help="design for the tethered bike (no battery/electronics)")
    ap.add_argument("-o", "--output", default="deploy/bundle.npz")
    args = ap.parse_args()

    params = load_params(args.params)
    print(f"designing (payload={not args.no_payload}) — this takes a few minutes...")
    bundle = build_bundle(params, payload=not args.no_payload)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, **bundle)

    r2 = bundle["fit_r2_grid"]
    print(f"wrote {out}")
    print(f"  digest      {bundle['params_digest']}")
    print(f"  speeds      {np.round(bundle['speeds'], 2)}")
    print(f"  gains       {bundle['Ks'].shape}")
    print(f"  worst fit   R^2 {r2.min():.3f}  (per-state min over the grid)")

    # A bundle that cannot rebuild the controller is worthless; prove it here
    # rather than discovering it on the bike.
    from .hw.state import DeployModel, load_bundle
    design, dm = load_bundle(out, params)
    DriveController(params, dm, design)
    print("  rebuild     OK (DriveController constructed from the bundle alone)")


if __name__ == "__main__":
    main()
