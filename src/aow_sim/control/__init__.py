"""Controllers, moves, and the shared observation/action specs.

Re-exports are LAZY. Eagerly importing `.balance` and `.drive` here meant that
`from aow_sim.control.steer import ...` — a numpy-only module — pulled in the
whole controller stack, and with it MuJoCo. That broke the onboard code, which
must run on a Pi with no physics engine installed. A module-level `__getattr__`
(PEP 562) keeps `from aow_sim.control import DriveController` working while
letting a submodule be imported on its own.

See tests/test_hw_no_mujoco.py, which enforces the property.
"""

_EXPORTS = {
    "PDCascade": ".balance",
    "LQRBalance": ".balance",
    "make_controller": ".balance",
    "run": ".balance",
    "DriveController": ".drive",
    "SpeedProfile": ".drive",
    "FlickTrajectory": ".flick",
    "load_move": ".flick",
    "PivotController": ".pivot",
    "YawProfile": ".pivot",
}

__all__ = list(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from importlib import import_module
    return getattr(import_module(_EXPORTS[name], __name__), name)


def __dir__():
    return sorted(__all__)
