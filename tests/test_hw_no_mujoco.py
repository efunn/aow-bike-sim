"""The onboard stack must import with no MuJoCo, scipy, or torch installed.

This is the property the whole deployment story rests on: the Pi runs the
controllers, not the simulator, so `pip install` on the bike is numpy +
pyserial + dynamixel-sdk and nothing else.

It is also easy to break by accident and impossible to notice on a laptop
where everything is installed. It HAS broken: `control/__init__.py` used to
eagerly import `.balance`, so `from aow_sim.control.steer import ...` — a
numpy-only module — pulled in MuJoCo transitively.

The blocker asserts on itself first. An earlier version of this test used the
`find_module` API that Python 3.12 removed, so it blocked nothing and passed
everything.
"""

import importlib
import sys

import pytest

# Everything a laptop has and the bike must not need.
LAPTOP_ONLY = {"mujoco", "scipy", "torch", "gymnasium", "stable_baselines3",
               "matplotlib", "tensorboard"}

ONBOARD_MODULES = [
    "aow_sim.params",
    "aow_sim.control.steer",
    "aow_sim.control.policy",
    "aow_sim.control.lqr_design",
    "aow_sim.control.flick_spec",
    "aow_sim.control.general_spec",
    "aow_sim.control.balance",
    "aow_sim.control.pivot",
    "aow_sim.control.flick",
    "aow_sim.control.drive",
    "aow_sim.hw.ahrs",
    "aow_sim.hw.dynamixel",
    "aow_sim.hw.odometry",
    "aow_sim.hw.state",
    "aow_sim.hw.run_bike",
]


class _Blocker:
    """Make the laptop-only packages look absent, however they are reached."""

    def find_spec(self, name, path=None, target=None):
        if name.split(".")[0] in LAPTOP_ONLY:
            raise ImportError(f"BLOCKED (not installed on the Pi): {name}")
        return None


@pytest.fixture
def without_laptop_packages():
    blocker = _Blocker()
    saved = dict(sys.modules)
    for name in list(sys.modules):
        if name.split(".")[0] in LAPTOP_ONLY or name.startswith("aow_sim"):
            del sys.modules[name]
    sys.meta_path.insert(0, blocker)
    try:
        yield
    finally:
        sys.meta_path.remove(blocker)
        sys.modules.clear()
        sys.modules.update(saved)


def test_the_blocker_actually_blocks(without_laptop_packages):
    """Guard the guard: if this passes vacuously, every test below is a lie."""
    for name in ("mujoco", "scipy", "torch"):
        with pytest.raises(ImportError, match="BLOCKED"):
            importlib.import_module(name)


@pytest.mark.parametrize("module", ONBOARD_MODULES)
def test_onboard_module_imports_without_laptop_packages(without_laptop_packages,
                                                        module):
    importlib.import_module(module)


def test_controller_stack_is_reachable_without_mujoco(without_laptop_packages):
    """Not just importable — the classes the bike actually constructs."""
    from aow_sim.control.drive import DriveController      # noqa: F401
    from aow_sim.control.lqr_design import LQRDesign       # noqa: F401
    from aow_sim.control.policy import load_policy_npz     # noqa: F401
    from aow_sim.hw.run_bike import BikeRunner             # noqa: F401
    from aow_sim.params import load_params

    params = load_params()                    # reads the real YAML
    assert params["bike"]["wheelbase"] > 0


def test_lazy_control_reexports_still_work():
    """The lazy __getattr__ in control/__init__ must not break the sim-side
    `from aow_sim.control import DriveController` spelling."""
    import aow_sim.control as c
    assert c.DriveController is not None
    assert c.LQRBalance is not None
    assert "DriveController" in dir(c)
    with pytest.raises(AttributeError):
        c.NoSuchThing


def test_quat_to_mat_matches_mujoco():
    """balance.quat_to_mat replaced mujoco.mju_quat2Mat to break the import
    dependency; it must stay numerically identical."""
    import numpy as np
    import mujoco
    from aow_sim.control.balance import quat_to_mat

    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        expected = np.zeros(9)
        mujoco.mju_quat2Mat(expected, q)
        assert np.allclose(expected.reshape(3, 3), quat_to_mat(q), atol=1e-12)
