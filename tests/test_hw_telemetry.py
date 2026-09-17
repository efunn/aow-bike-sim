"""The bike -> station packet, both directions.

WHAT INVALIDATES THIS FILE: `hw/telemetry.py`'s FIELDS or `build` signature,
the drivetrain mix coefficients, or the joint names `apply_pose` writes.

The point of the file is the SEAM. `build` runs on the Pi and `apply_pose` on
the laptop, and the failure they are both exposed to is drifting apart: a
field renamed on one side is read as a `.get()` default on the other, which
does not raise and does not look wrong -- it renders a confidently incorrect
bike. So these tests walk the field list rather than spot-checking, and the
gearbox test compares against MuJoCo's own constraint solve rather than
against the arithmetic in the module.
"""
import json

import numpy as np
import pytest

import mujoco

from aow_sim.build_model import build_model, load_params
from aow_sim.hw import telemetry as T

pytestmark = pytest.mark.geometry


def sample(**over):
    kw = dict(t=1.0, state="engaged", quat=[0.996, 0.087, 0.0, 0.0],
              gyro=[0.1, 0.0, 0.2], v_world=[0.6, -0.05], pos=[1.2, 0.3],
              steer=-0.21, w_shaft=[24.0, -16.0], shaft=[0.0, 0.0], psi=0.3,
              cmd_v_world=[0.7, 0.0], cmd_psi=0.35, roll=0.174, roll_rate=0.1,
              volts=11.9, vlat_conf=0.87, qos=5, jitter_ms=0.14, dt_ms=10.0,
              cuts=0, righting=None, righting_current=300)
    kw.update(over)
    return T.build(**kw)


def test_build_emits_exactly_the_declared_fields():
    """FIELDS is what the test walks, so it must BE the packet -- not a
    hand-maintained list beside it that quietly falls behind."""
    assert set(sample()) == set(T.FIELDS)


def test_the_packet_is_json_and_small_enough_to_not_think_about():
    """Measured link ceiling is >1.5 MB/s with zero loss (pi-bench-bringup);
    this asserts the packet stays in the regime where that is obviously true,
    not that it is optimal."""
    b = json.dumps(sample()).encode()
    assert len(b) < 1200, f"{len(b)} B -- fine on the radio, but say why here"


def test_a_schema_mismatch_raises_rather_than_rendering_a_wrong_bike():
    with pytest.raises(T.SchemaMismatch, match="stale deploy"):
        T.check_version({"v": T.SCHEMA_VERSION - 1})
    with pytest.raises(T.SchemaMismatch):
        T.check_version({"state": "engaged"})     # fields, no version
    T.check_version(sample())                     # the live one does not raise


def test_an_empty_packet_is_not_a_version_mismatch():
    """The bike's link thread starts before its control loop, so the first
    packets on the wire are `{}`. Found by running the mirror against the real
    bike: it raised "schema vNone ... re-sync the Pi" against a Pi that was
    correct, which sends the reader after the wrong thing entirely. The bike
    no longer transmits an empty dict AND this tolerates one -- both, because
    an older bike is exactly the case the version check exists for."""
    T.check_version({})


def test_build_is_keyword_only():
    """Twenty fields, half of them same-magnitude vectors: a positional call
    that swaps gyro and v_world renders a bike that looks almost right."""
    with pytest.raises(TypeError):
        T.build(1.0, "engaged")


# --- the mirror --------------------------------------------------------------

@pytest.fixture(scope="module")
def built():
    p = load_params()
    m = build_model(p)
    return p, m


def test_the_gearbox_closed_form_matches_mujocos_own_constraint(built):
    """`apply_pose` computes hub/ring/rollers from the two input shafts in
    closed form, because a render has no solver. This checks that against the
    tendon equalities MuJoCo actually solves, driven with real physics -- so
    if `_aow_assembly`'s mixes change, this fails rather than the mirror
    quietly drawing a wheel that turns at the wrong rate.
    """
    p, m = built
    d = mujoco.MjData(m)
    d.ctrl[:2] = (14.0, -9.0)                # spin the inputs opposite ways
    for _ in range(1500):
        mujoco.mj_step(m, d)

    adr = T.PoseAdr(m)
    a = float(d.qpos[adr.q["input_a_spin"]])
    b = float(d.qpos[adr.q["input_b_spin"]])
    mix = T.gearbox_mix(p)
    hub = mix["mix_hub_a"] * a + mix["mix_hub_b"] * b
    ring_rel = (mix["mix_ring_a"] * a + mix["mix_ring_b"] * b) - hub

    assert float(d.qpos[adr.q["hub_spin"]]) == pytest.approx(hub, abs=2e-3)
    assert float(d.qpos[adr.q["ring_spin"]]) == pytest.approx(ring_rel, abs=2e-3)
    for name in adr.rollers:
        assert float(d.qpos[adr.q[name]]) == pytest.approx(
            mix["k_roller"] * ring_rel, abs=5e-3), name


def test_apply_pose_puts_every_measured_field_somewhere_visible(built):
    """The seam test. Every field `build` sends that describes POSE must move
    something in `data`; a field nothing reads is a wire nobody connected."""
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    tel = sample()
    T.apply_pose(m, d, tel, adr, p, rest_z=0.05093)
    assert np.allclose(d.qpos[3:7], tel["quat"])
    assert np.allclose(d.qvel[3:6], tel["gyro"])
    assert np.allclose(d.qvel[0:2], tel["v_world"])
    assert np.allclose(d.qpos[0:2], tel["pos"])
    # NOT a fixed height any more: `rest_z` only seeds the first kinematics
    # pass, and `ground_the_wheels` then puts the lower wheel on the floor.
    assert d.qpos[2] > 0.0
    assert d.qpos[adr.q["steer_joint"]] == pytest.approx(tel["steer"])
    # w_shaft drives the wheel through the integrator, so it shows up as RATE
    # on the first frame even though the angle is still ~0.
    assert d.qvel[adr.d["input_a_spin"]] == pytest.approx(tel["w_shaft"][0])
    assert d.qvel[adr.d["input_b_spin"]] == pytest.approx(tel["w_shaft"][1])
    assert d.qvel[adr.d["hub_spin"]] != 0.0
    assert d.qvel[adr.d["roller_spin_0"]] != 0.0


def test_the_shaft_angle_integrates_across_frames(built):
    """The FALLBACK path, for a bike too old to send `shaft`. Kept because
    that is exactly the case the schema version exists to survive."""
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    tel = sample(w_shaft=[10.0, 0.0])
    del tel["shaft"]                       # force the fallback path
    shaft = (0.0, 0.0)
    for _ in range(100):
        shaft = T.apply_pose(m, d, tel, adr, p, shaft_angle=shaft, dt=0.01)
    assert shaft[0] == pytest.approx(10.0)           # 10 rad/s for 1.0 s
    assert shaft[1] == pytest.approx(0.0)
    assert d.qpos[adr.q["input_a_spin"]] == pytest.approx(10.0)


def test_a_missing_field_leaves_the_pose_alone_rather_than_zeroing_it(built):
    """A newer bike talking to an older station should degrade to a partly
    stale picture, not to a bike that snaps to the origin every frame."""
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    T.apply_pose(m, d, sample(), adr, p, rest_z=0.05)
    before = d.qpos.copy()
    T.apply_pose(m, d, {"v": T.SCHEMA_VERSION, "dt_ms": 10.0}, adr, p)
    assert np.allclose(d.qpos[0:2], before[0:2])
    assert np.allclose(d.qpos[3:7], before[3:7])
    assert d.qpos[adr.q["steer_joint"]] == pytest.approx(before[adr.q["steer_joint"]])


def test_pose_adr_resolves_by_name_not_by_index(built):
    """Hardcoded qpos indices break silently: `build_model` emits a different
    nq with payload/hockey/righting attached."""
    p, m = built
    adr = T.PoseAdr(m)
    assert adr.q["steer_joint"] == int(
        m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "steer_joint")])
    assert len(adr.rollers) == 8
    assert adr.rollers == sorted(adr.rollers)


def test_neither_wheel_ever_goes_below_the_floor(built):
    """A fixed ride height is wrong the moment the bike pitches: the attitude
    rotates the body about its origin, so nose-down drove the front wheel
    through the ground and a wheelie floated the rear. Found by looking at the
    mirror, not by a test -- so here is the test."""
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    r_rear = float(p["omni_wheel"]["outer_radius"])
    r_front = float(p["bike"]["front_wheel"]["radius"])
    for deg in (-30, -25, -10, 0, 10, 25, 30):
        a = np.deg2rad(deg) / 2
        tel = sample(quat=[np.cos(a), 0.0, np.sin(a), 0.0])
        T.apply_pose(m, d, tel, adr, p, rest_z=0.0509)
        mujoco.mj_forward(m, d)
        rear = float(d.body("aow_hub").xpos[2]) - r_rear
        front = float(d.body("front_wheel").xpos[2]) - r_front
        assert min(rear, front) == pytest.approx(0.0, abs=1e-6), (
            f"pitch {deg}: rear {rear:.5f} front {front:.5f} -- one wheel must "
            f"touch")
        assert rear > -1e-6 and front > -1e-6, f"pitch {deg}: below the floor"


def test_a_roll_also_keeps_the_wheels_on_the_floor(built):
    """Roll is the axis the bike actually lives on, and it rotates the wheels
    about their own contact line rather than about the wheelbase -- so it is a
    weaker test than pitch, and worth having precisely because it is easy to
    assume it is covered."""
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    r_rear = float(p["omni_wheel"]["outer_radius"])
    r_front = float(p["bike"]["front_wheel"]["radius"])
    for deg in (-40, -15, 0, 15, 40):
        a = np.deg2rad(deg) / 2
        T.apply_pose(m, d, sample(quat=[np.cos(a), np.sin(a), 0.0, 0.0]),
                     adr, p, rest_z=0.0509)
        mujoco.mj_forward(m, d)
        rear = float(d.body("aow_hub").xpos[2]) - r_rear
        front = float(d.body("front_wheel").xpos[2]) - r_front
        assert rear > -1e-6 and front > -1e-6, f"roll {deg}: below the floor"


def test_the_shaft_integrates_over_DISPLAY_time_not_the_bikes_tick(built):
    """The bug that made a hand-spun rear wheel barely move in the mirror.

    `dt_ms` in the packet is the BIKE's control period (10 ms). Using it once
    per rendered frame integrates 60 x 10 ms per second of wall clock, i.e.
    the wheels turn at 60% of reality -- and at any other frame rate, at some
    other arbitrary fraction. The rate is instantaneous; the time it is
    multiplied by must be the time the VIEWER advanced.
    """
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    tel = sample(w_shaft=[10.0, 0.0], dt_ms=10.0)    # bike ticks at 100 Hz
    del tel["shaft"]                                 # force the fallback path
    shaft = (0.0, 0.0)
    for _ in range(60):                              # one second at 60 fps
        shaft = T.apply_pose(m, d, tel, adr, p, shaft_angle=shaft, dt=1 / 60)
    assert shaft[0] == pytest.approx(10.0, rel=1e-6), (
        "one second of display at 10 rad/s is 10 rad, whatever the frame rate "
        "and whatever the bike's tick")


def test_a_sent_shaft_angle_beats_integrating_the_rate(built):
    """`shaft` is summed on the bike from the unwrapped per-tick delta.
    Integrating `w_shaft` on the station instead is lossy three ways -- `vel`
    is filtered so it lags, telemetry does not carry every tick, and a dropped
    packet is a lost increment that never comes back."""
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    tel = sample(shaft=[3.0, -2.0], w_shaft=[999.0, 999.0])
    got = T.apply_pose(m, d, tel, adr, p, shaft_angle=(50.0, 50.0), dt=1.0)
    assert got == pytest.approx((3.0, -2.0)), "the sent angle must win outright"
    assert d.qpos[adr.q["input_a_spin"]] == pytest.approx(3.0)


def test_integration_survives_only_for_a_bike_that_cannot_send_it(built):
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    tel = sample(w_shaft=[10.0, 0.0])
    del tel["shaft"]
    got = T.apply_pose(m, d, tel, adr, p, shaft_angle=(0.0, 0.0), dt=0.5)
    assert got[0] == pytest.approx(5.0)
