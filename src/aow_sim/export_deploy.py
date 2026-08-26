"""Export everything the onboard controller needs, so the bike carries no
MuJoCo model and no scipy.

Run on the laptop:

    python -m aow_sim.export_deploy                 # -> deploy/bundle.npz
    python -m aow_sim.export_deploy --no-payload    # the tethered bike

Building a DriveController normally costs two numerical linearizations of the
MuJoCo model (design_lqr + design_gain_schedule) — a couple of seconds of
rollouts, but it needs scipy and MuJoCo, and
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
from pathlib import Path

import numpy as np

from .build_model import build_model, load_params
from .control.drive import DriveController
from .control.linearize import design_all
# Re-exported: params_digest moved to params.py (MuJoCo-free) so that the
# onboard bundle check and the move-file check can both reach it without
# importing a physics engine. Kept importable from here — it is the module
# people associate with the digest, and hw/state.py used to import it here.
from .params import params_digest, plant_digest, design_digest  # noqa: F401

# Actuators and joints the controllers look up by name.
ACTUATORS = ("drive_a", "drive_b", "steer")
STEER_JOINT = "steer_joint"


DEFAULT_AHRS_MOUNT = "config/ahrs_mount.yaml"


def load_ahrs_mount(path: str | None = None) -> tuple[np.ndarray, str]:
    """Read the AHRS mounting quaternion -> (q_mount, source).

    Deliberately NOT part of `bike_params.yaml`, and so not part of
    `params_digest`: this describes where the sensor sits on the chassis, not
    the bike's physics. It cannot change a gain schedule or a trained policy,
    so making it invalidate both on every re-seat of the sensor would be a
    pure cost. It rides in the bundle because that is what the bike loads.

    A missing file is not an error — it means "not calibrated yet" and yields
    identity, the same thing the code did before this was persisted at all.
    """
    import yaml

    p = Path(path or DEFAULT_AHRS_MOUNT)
    if not p.exists():
        return np.array([1.0, 0.0, 0.0, 0.0]), "absent"
    d = yaml.safe_load(p.read_text()) or {}
    q = np.asarray(d.get("q_mount", [1.0, 0.0, 0.0, 0.0]), dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        raise ValueError(f"{p}: q_mount is the zero quaternion")
    return q / n, str(d.get("source", "GUESS"))


def build_bundle(params: dict, payload: bool = True,
                 ahrs_mount: tuple[np.ndarray, str] | None = None) -> dict:
    model = build_model(params, variant="full", payload=payload)
    design = design_all(params, model)
    sj = model.joint(STEER_JOINT)
    q_mount, mount_source = ahrs_mount or (np.array([1.0, 0.0, 0.0, 0.0]), "absent")
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
        # -- AHRS mounting calibration (see load_ahrs_mount) --
        "ahrs_q_mount": q_mount,
        "ahrs_mount_source": np.array(mount_source),
        # -- provenance --
        # THREE digests, because the bundle is two artifacts. `plant_digest`
        # covers DeployModel -- actuator ids, joint addresses, qpos_eq -- which
        # the RL path needs and a mismatch on which is a fall. `design_digest`
        # covers only the LQR gain-design inputs, so a stale gain schedule can
        # warn without stopping an RL run. `params_digest` stays for tools that
        # already read it. See docs/plans/params-digest-split.md.
        "params_digest": np.array(params_digest(params)),
        "plant_digest": np.array(plant_digest(params)),
        "design_digest": np.array(design_digest(params)),
        "payload": np.array(payload),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=None, help="path to bike_params.yaml")
    ap.add_argument("--no-payload", action="store_true",
                    help="design for the tethered bike (no battery/electronics)")
    ap.add_argument("-o", "--output", default="deploy/bundle.npz")
    ap.add_argument("--ahrs-mount", default=None,
                    help=f"AHRS mounting calibration (default {DEFAULT_AHRS_MOUNT})")
    args = ap.parse_args()

    params = load_params(args.params)
    mount = load_ahrs_mount(args.ahrs_mount)
    print(f"designing (payload={not args.no_payload})...")
    bundle = build_bundle(params, payload=not args.no_payload, ahrs_mount=mount)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, **bundle)

    r2 = bundle["fit_r2_grid"]
    print(f"wrote {out}")
    print(f"  digest      {bundle['params_digest']}")
    print(f"  speeds      {np.round(bundle['speeds'], 2)}")
    print(f"  gains       {bundle['Ks'].shape}")
    print(f"  worst fit   R^2 {r2.min():.3f}  (per-state min over the grid)")
    q, src = mount
    print(f"  ahrs mount  {np.round(q, 4)}  (source: {src})")
    if src != "measured":
        print("              ^ NOT calibrated — the bike will assume the sensor "
              "is perfectly\n                aligned with the chassis. A few degrees "
              "of tilt is a permanent\n                roll bias the controller trims "
              f"against forever. See {DEFAULT_AHRS_MOUNT}.")

    # A bundle that cannot rebuild the controller is worthless; prove it here
    # rather than discovering it on the bike.
    from .hw.state import DeployModel, load_bundle
    design, dm = load_bundle(out, params)
    DriveController(params, dm, design)
    print("  rebuild     OK (DriveController constructed from the bundle alone)")


if __name__ == "__main__":
    main()
