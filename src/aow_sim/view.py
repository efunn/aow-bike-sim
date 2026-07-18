"""Interactive viewer: python -m aow_sim.view [--variant full|testbed] [--training-wheels].

Use the viewer's Control panel to command the actuators:
  drive_a / drive_b — input-shaft velocity [rad/s]; equal = roll, differential = lateral crawl
  steer             — steer angle [rad], continuous (full variant only)
"""

from __future__ import annotations

import argparse

import mujoco
import mujoco.viewer

from .build_model import build_model, load_params


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--variant", choices=["full", "testbed"], default="full")
    ap.add_argument("--params", default=None, help="path to bike_params.yaml")
    ap.add_argument("--training-wheels", action="store_true")
    args = ap.parse_args()
    model = build_model(load_params(args.params), args.variant, args.training_wheels)
    print(__doc__)
    mujoco.viewer.launch(model)


if __name__ == "__main__":
    main()
