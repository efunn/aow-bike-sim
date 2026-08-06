"""Hardware backend for the physical bike.

Nothing here is imported by the simulator. The point of this package is that
the CONTROLLERS are not modified at all: `DriveController.step(model, data)`
runs verbatim against a `HardwareData` that quacks like `mjData`.

See docs/plans/untethered-setup.md for the wiring, the parts, and why the
onboard controller is `general_rl` rather than the LQR.
"""
