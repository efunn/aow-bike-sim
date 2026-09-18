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
              cuts=0, righting=None, righting_pos=None,
              righting_current=300)
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


HEALTH = {  # hw/dynamixel.HEALTH_BLOCK labels, decoded, per role
    "drive_a": {"Present PWM": 0.12, "effort": 0.3, "Present Input Voltage": 11.9,
                "Present Temperature": 38.0, "Hardware Error Status": 0.0},
    "drive_b": {"Present PWM": -0.1, "effort": -0.25, "Present Input Voltage": 11.9,
                "Present Temperature": 37.0, "Hardware Error Status": 0.0},
    "steer": {"Present PWM": 0.05, "effort": 0.3, "Present Input Voltage": 12.0,
              "Present Temperature": 30.0, "Hardware Error Status": 0.0},
    "righting": {"Present PWM": 0.0, "effort": 0.0, "Present Input Voltage": 12.0,
                 "Present Temperature": 29.0, "Hardware Error Status": 36.0},
}
UNITS = {"drive_a": "frac_max_torque", "drive_b": "frac_max_torque",
         "steer": "A", "righting": "A"}


def test_the_effort_key_names_its_unit_so_load_and_amps_never_share_a_column():
    """Address 126 is Present Load on the drives and Present Current on the
    XC330s. The SAME 0.3 is 30 % of max torque on one and 300 mA on the other;
    a shared key would invite adding them up."""
    got = T.servo_health(HEALTH, UNITS)
    assert got["drive_a"]["load"] == 0.3 and "A" not in got["drive_a"]
    assert got["steer"]["A"] == 0.3 and "load" not in got["steer"]
    assert got["righting"]["err"] == 36 and got["drive_a"]["C"] == 38


def test_the_full_packet_with_four_servos_stays_small():
    """The number that matters is WITH the servos in -- Phase 2's packet."""
    b = json.dumps(sample(servos=T.servo_health(HEALTH, UNITS),
                          run="260917-221500_run_bench")).encode()
    assert len(b) < 1200, f"{len(b)} B"


def test_the_readout_shows_each_servo_and_puts_an_error_in_place_of_its_line():
    tel = sample(servos=T.servo_health(HEALTH, UNITS), run="260917-221500_run")
    left, right = T.status_text(tel)
    labels, values = left.split("\n"), right.split("\n")
    assert len(labels) == len(values)
    assert labels[3:] == ["drive_a", "drive_b", "steer", "righting"]
    row = dict(zip(labels, values))
    assert "load  +30%" in row["drive_a"] and "+0.30 A" in row["steer"]
    assert row["record"] == "260917-221500_run"
    # 36 = bits 2 and 5: overheating and electrical shock
    assert row["righting"].startswith("HARDWARE ERROR")
    assert T.servo_errors(tel) == {"righting": row["righting"].split(": ", 1)[1]}
    assert T.status_text(sample())[1].split("\n")[2] == "off"   # not recording


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


# --- the righting mechanism ---------------------------------------------------

@pytest.fixture(scope="module")
def swung():
    """A bike WITH the co-rotating four-bar. Separate from `built` because
    nearly every other test here wants the plain model, and the mechanism adds
    six joints that shift nothing else only because `PoseAdr` works by name."""
    import yaml

    from aow_sim.build_model import SWING_LINKAGE_CFG, SwingLinkageSolver
    p = load_params()
    m = build_model(p, variant="full", righting=True, swing_linkage=True)
    onb = p["control"]["onboard"]
    stow = np.deg2rad(float(onb["righting_stow_deg"]))
    sign = float(onb["righting_sign"])
    with open(SWING_LINKAGE_CFG) as fh:
        solver = SwingLinkageSolver(yaml.safe_load(fh),
                                    p["omni_wheel"]["outer_radius"])
    return p, m, solver, (lambda rad: solver.pose((float(rad) - stow) * sign)), stow


def _eq_gaps(m, d):
    """Distance between each `mjEQ_CONNECT` site pair [m]. Zero means the
    four-bar is assembled; anything else means it is drawn coming apart."""
    return [float(np.linalg.norm(d.site(f"swing_coupler_{t}_end").xpos
                                 - d.site(f"swing_wing_{t}_attach").xpos))
            for t in ("right", "left")]


def test_the_rendered_four_bar_stays_assembled_without_a_physics_step(swung):
    """THE REASON `SwingLinkageSolver` EXISTS. The loop is closed by an
    equality constraint, and an equality is only satisfied by the solver during
    a dynamics step -- `mj_forward` computes constraint forces, not positions.
    The mirror never steps, so if `apply_pose` wrote only the crank the
    mechanism would visibly tear apart.

    Checked against MuJoCo's own constraint rather than against the arithmetic
    in the module, for the same reason the gearbox test is."""
    p, m, solver, pose, stow = swung
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    trav = solver.travel_max
    assert trav > 0, "the config must declare stroke.crank_travel_deg"
    for t in np.linspace(-trav, trav, 21):
        mujoco.mj_resetData(m, d)
        T.apply_pose(m, d, sample(righting_pos=float(stow + t)), adr, p,
                     rest_z=0.05, dt=1 / 60, swing_pose=pose)
        mujoco.mj_forward(m, d)
        assert max(_eq_gaps(m, d)) < 1e-9, f"comes apart at {np.rad2deg(t):.1f} deg"


def test_the_solvers_rest_pose_is_the_pose_the_builder_laid_out(swung):
    """The contract between the solver's two callers. `_add_swing_linkage`
    draws every link in its rest direction, so a joint at zero IS the rest
    pose -- and the solver must agree, or the mechanism would jump the moment
    the first packet arrived and then be wrong by that offset forever.

    Both halves are checked: the solver says all-zero at zero travel, and the
    model as compiled (nothing written into it at all) already satisfies its
    own equality constraints."""
    p, m, solver, pose, _stow = swung
    at_rest = solver.pose(0.0)
    assert set(at_rest) == {"swing_crank_joint",
                            "swing_coupler_left_joint", "swing_wing_left_joint",
                            "swing_coupler_right_joint", "swing_wing_right_joint"}
    assert all(v == pytest.approx(0.0, abs=1e-12) for v in at_rest.values()), \
        at_rest
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert max(_eq_gaps(m, d)) < 1e-9, "the compiled model is not at rest"


def test_the_wings_co_rotate_rather_than_mirror(swung):
    """The whole point of this mechanism over the geared pair: one wing swings
    down and out while the other comes up and in. A mirrored pair would give
    the two wings equal and OPPOSITE angles; this one drives them the same way,
    which is what lets it present one face on one side."""
    p, m, solver, pose, stow = swung
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    q = adr.q
    got = {}
    for tag, t in (("plus", 0.6 * solver.travel_max),
                   ("minus", -0.6 * solver.travel_max)):
        mujoco.mj_resetData(m, d)
        T.apply_pose(m, d, sample(righting_pos=float(stow + t)), adr, p,
                     rest_z=0.05, dt=1 / 60, swing_pose=pose)
        got[tag] = (float(d.qpos[q["swing_wing_right_joint"]]),
                    float(d.qpos[q["swing_wing_left_joint"]]))
    for r, l in got.values():
        assert np.sign(r) == np.sign(l), (r, l)      # same way: co-rotating
        assert abs(r) > 2 * abs(l) or abs(l) > 2 * abs(r), (
            "one side should swing far while the other barely moves")
    # And the two crank directions are each other's mirror image, which is what
    # makes the rest pose symmetric by construction rather than by tuning.
    assert got["plus"][0] == pytest.approx(-got["minus"][1], abs=1e-4)


def test_the_measured_angle_beats_the_commanded_one(swung):
    """A wing held back by its Goal Current must be DRAWN held back. Rendering
    the goal would give a mechanism that always arrives, which is exactly the
    thing the operator is watching for while tuning `righting_current`."""
    p, m, solver, pose, stow = swung
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    half = 0.5 * solver.travel_max
    T.apply_pose(m, d, sample(righting=float(stow + solver.travel_max),
                              righting_pos=float(stow + half)),
                 adr, p, rest_z=0.05, dt=1 / 60, swing_pose=pose)
    assert d.qpos[adr.q["swing_crank_joint"]] == pytest.approx(half, abs=1e-4)


def test_a_bike_that_sends_no_reading_falls_back_to_the_goal(swung):
    """An older bike, or one whose righting servo is absent from the chain."""
    p, m, solver, pose, stow = swung
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    half = 0.5 * solver.travel_max
    T.apply_pose(m, d, sample(righting=float(stow + half)), adr, p,
                 rest_z=0.05, dt=1 / 60, swing_pose=pose)
    # abs=1e-4 because `build` ROUNDS the angle to four decimals on the way
    # out. That is the wire's resolution -- 6e-5 rad, or 0.003 deg at the
    # servo -- not slop in the solve.
    assert d.qpos[adr.q["swing_crank_joint"]] == pytest.approx(half, abs=1e-4)
    mujoco.mj_resetData(m, d)
    T.apply_pose(m, d, sample(), adr, p, rest_z=0.05, dt=1 / 60, swing_pose=pose)
    assert d.qpos[adr.q["swing_crank_joint"]] == 0.0, "no command, no movement"


def test_a_model_without_the_mechanism_ignores_the_solver(built):
    """The station builds the solver from a config; the model may not have the
    mechanism at all. That combination must render a plain bike, not raise."""
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    T.apply_pose(m, d, sample(righting_pos=3.4), adr, p, rest_z=0.05,
                 dt=1 / 60, swing_pose=lambda _: {"swing_crank_joint": 1.0})
    assert "swing_crank_joint" not in adr.q


def test_this_geometry_turns_all_the_way_round(swung):
    """MEASURED, and not what the config's name suggests. `crank_travel_deg`
    136.6 is the DESIGN stroke -- what reach and clearance ask for -- not a
    kinematic limit: this crank closes the loop at every angle in 360 deg.

    Worth pinning because the guard below looks like dead code otherwise, and
    because "the mechanism stopped at the end of its travel" would be the wrong
    explanation for a station that froze at 136 deg."""
    _, _, solver, pose, stow = swung
    assert solver.travel_max == pytest.approx(np.deg2rad(136.6), abs=1e-3)
    assert all(pose(stow + np.deg2rad(d)) is not None
               for d in range(-180, 181)), "expected a full-rotation crank"


def test_past_the_assembly_limit_the_mechanism_holds_rather_than_jumps(swung):
    """`pose` returns None where the loop cannot close. That is an answer, not
    an error -- and the station must leave the last good pose on screen rather
    than snapping every joint to zero.

    Driven with a LENGTHENED coupler, which sounds like the wrong direction
    and is not. A circle-circle solve fails two ways, and this one fails on the
    INNER bound: the loop needs |rocker - coupler| <= L <= rocker + coupler,
    and stretching the coupler from 59.6 to 77.5 mm lifts that lower bound from
    11.6 to 29.5 mm while the crank tip still swings to within 27.8 mm of the
    wing pivot. Nothing is out of REACH -- 52 of 361 whole degrees are out of
    FOLD, because the two links would have to overlap to close that tightly.

    Chosen over a shortened one because x0.7 fails in the constructor instead
    (it cannot close at rest either), which tests a different thing. Not
    hypothetical: `crank_length` and `coupler_length` are free variables of the
    optimiser in analysis/swing_linkage.py, and `assembly_limit` exists there
    to find exactly this."""
    import copy

    import yaml

    from aow_sim.build_model import SWING_LINKAGE_CFG, SwingLinkageSolver
    p, m, solver, pose, stow = swung
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    with open(SWING_LINKAGE_CFG) as fh:
        cfg = copy.deepcopy(yaml.safe_load(fh))
    cfg["mechanism"]["coupler_length"] *= 1.3
    short = SwingLinkageSolver(cfg, p["omni_wheel"]["outer_radius"])
    blocked = [dg for dg in range(-180, 181)
               if short.pose(np.deg2rad(dg)) is None]
    assert blocked, "a longer coupler should put some angles out of reach"

    half = 0.5 * solver.travel_max
    T.apply_pose(m, d, sample(righting_pos=float(stow + half)), adr, p,
                 rest_z=0.05, dt=1 / 60, swing_pose=pose)
    was = float(d.qpos[adr.q["swing_crank_joint"]])
    T.apply_pose(m, d, sample(righting_pos=999.0), adr, p, rest_z=0.05,
                 dt=1 / 60,
                 swing_pose=lambda rad: short.pose(np.deg2rad(blocked[0])))
    assert d.qpos[adr.q["swing_crank_joint"]] == pytest.approx(was), (
        "an unreachable angle must leave the last good pose alone")


# --- the seam ----------------------------------------------------------------

# Every field, split by what it is FOR. A field that describes the bike's
# shape must move `data`; a field that is a number on the status line must not
# be expected to. The split is written out rather than inferred so that adding
# a field forces a decision about which it is -- which is the whole mechanism
# below.
POSE_FIELDS = {"quat", "gyro", "v_world", "pos", "steer", "w_shaft", "shaft",
               "righting", "righting_pos"}
DISPLAY_FIELDS = {"v", "t", "state", "cmd_v_world", "cmd_psi", "psi", "roll",
                  "roll_rate", "volts", "vlat_conf", "qos", "jitter_ms",
                  "dt_ms", "cuts", "righting_current", "servos", "run", "log",
                  "hold", "warn"}

# A perturbation per pose field, big enough to be unmistakable.
_NUDGE = {"quat": [0.966, 0.259, 0.0, 0.0], "gyro": [1.0, -2.0, 3.0],
          "v_world": [-1.1, 0.7], "pos": [9.0, -4.0], "steer": 0.4,
          "w_shaft": [-30.0, 44.0], "shaft": [7.0, -5.0],
          "righting_pos": 3.6, "righting": 3.6}


def test_every_telemetry_field_is_read_by_the_mirror(swung):
    """THE SEAM TEST, and the one this module's docstring has always claimed
    existed. `build` runs on the Pi and `apply_pose` on the laptop; the failure
    they are both exposed to is a field that one side writes and the other
    never reads, which does not raise and does not look wrong.

    `test_build_emits_exactly_the_declared_fields` pins the SENDING half.
    This is the receiving half: every pose field, perturbed on its own, must
    move `qpos` or `qvel`. A field added with no reader lands in neither set
    and fails on the partition below before it gets here.
    """
    p, m, _solver, pose, _stow = swung
    assert POSE_FIELDS | DISPLAY_FIELDS == set(T.FIELDS), (
        "a new field must be classified as pose or display -- "
        f"unclassified: {set(T.FIELDS) - POSE_FIELDS - DISPLAY_FIELDS}")
    assert not POSE_FIELDS & DISPLAY_FIELDS

    def rendered(tel):
        d = mujoco.MjData(m)
        T.apply_pose(m, d, tel, T.PoseAdr(m), p, rest_z=0.05, dt=1 / 60,
                     swing_pose=pose)
        return np.concatenate([d.qpos.copy(), d.qvel.copy()])

    for name in sorted(POSE_FIELDS):
        base = sample(righting_pos=2.9)
        # `righting` is the FALLBACK for a bike that sends no reading, so it
        # can only be shown to be read when there is no reading to beat it.
        if name == "righting":
            base = sample()
        moved = rendered({**base, name: _NUDGE[name]})
        assert not np.allclose(rendered(base), moved), (
            f"{name!r} is sent and nothing renders it -- a wire nobody "
            "connected")


def test_a_deployed_wing_never_sinks_through_the_floor(swung):
    """The panels are BOXES, so their lowest point is a corner and which
    corner it is changes with roll -- the reason the wheels' `centre - radius`
    shortcut cannot cover them.

    Upright this changes nothing (a fully deployed wing still clears the wheel
    contact by 5.7 mm); rolled it is the whole point, because the wing reaches
    78 mm below the wheels at 61 deg and was being drawn sunk through the
    ground."""
    p, m, solver, pose, stow = swung
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    assert adr.panels, "the mechanism's panels must be found by name"

    def lowest(roll_deg, servo_deg):
        a = np.deg2rad(roll_deg) / 2
        tel = sample(quat=[float(np.cos(a)), float(np.sin(a)), 0.0, 0.0],
                     righting_pos=float(stow + np.deg2rad(servo_deg)))
        mujoco.mj_resetData(m, d)
        T.apply_pose(m, d, tel, adr, p, rest_z=0.05, dt=1 / 60, swing_pose=pose)
        mujoco.mj_forward(m, d)
        floors = [float(d.body("aow_hub").xpos[2])
                  - float(p["omni_wheel"]["outer_radius"]),
                  float(d.body("front_wheel").xpos[2])
                  - float(p["bike"]["front_wheel"]["radius"])]
        for gid, size in adr.panels:
            rot = d.geom_xmat[gid].reshape(3, 3)
            floors.append(float(d.geom_xpos[gid][2])
                          - float(np.abs(rot[2]) @ size))
        return min(floors)

    for roll in (0, 30, 61, 90):
        for servo in (0, 68, 136):
            assert lowest(roll, servo) > -1e-9, (
                f"something is through the floor at roll {roll}, "
                f"servo {servo}")


def test_grounding_on_a_wing_lifts_the_wheels_rather_than_the_other_way(swung):
    """A rolled bike with a wing out rests ON THE WING -- which is what the
    mechanism is for, and is the observable difference from grounding on the
    wheels alone."""
    p, m, solver, pose, stow = swung
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    a = np.deg2rad(61.0) / 2
    tel = sample(quat=[float(np.cos(a)), float(np.sin(a)), 0.0, 0.0],
                 righting_pos=float(stow + np.deg2rad(136.0)))
    T.apply_pose(m, d, tel, adr, p, rest_z=0.05, dt=1 / 60, swing_pose=pose)
    mujoco.mj_forward(m, d)
    rear = (float(d.body("aow_hub").xpos[2])
            - float(p["omni_wheel"]["outer_radius"]))
    assert rear > 0.05, (
        f"the rear wheel should be lifted clear, is {rear*1e3:.1f} mm")


def test_a_bike_with_no_mechanism_grounds_on_its_wheels_exactly_as_before(built):
    """The generalisation must not move the plain bike. `panels` is empty
    there, so the wheel arithmetic is reached unchanged."""
    p, m = built
    adr, d = T.PoseAdr(m), mujoco.MjData(m)
    assert adr.panels == []
    T.apply_pose(m, d, sample(), adr, p, rest_z=0.05, dt=1 / 60)
    mujoco.mj_forward(m, d)          # apply_pose leaves this to its caller
    rear = (float(d.body("aow_hub").xpos[2])
            - float(p["omni_wheel"]["outer_radius"]))
    front = (float(d.body("front_wheel").xpos[2])
             - float(p["bike"]["front_wheel"]["radius"]))
    assert min(rear, front) == pytest.approx(0.0, abs=1e-12)


def test_a_bike_message_rides_every_packet_for_a_while_then_drops_out():
    """Loss-proof without acks: each message is repeated for `hold_s`, and a
    quiet bike sends an empty list -- the steady-state cost is ~11 bytes."""
    now = [0.0]
    log = T.EventLog(hold_s=2.0, keep=4, clock=lambda: now[0])
    assert log.recent() == []
    log.add("cut", "CUT: roll 61 deg")
    now[0] = 1.9
    assert log.recent() == [[1, "cut", "CUT: roll 61 deg"]]
    now[0] = 2.1
    assert log.recent() == []
    for k in range(6):                        # a burst is bounded by `keep`
        log.add("link", f"m{k}")
    assert [m[0] for m in log.recent()] == [4, 5, 6, 7]


def test_the_station_prints_each_message_once_whatever_the_packet_loss():
    seen = 0
    printed = []
    packets = [{"log": [[1, "cut", "a"]]}, {"log": [[1, "cut", "a"]]}, {},
               {"log": [[1, "cut", "a"], [2, "rearm", "b"]]},
               {"log": [[2, "rearm", "b"]]}]
    for tel in packets:
        msgs, seen = T.new_messages(tel, seen)
        printed += [m[2] for m in msgs]
    assert printed == ["a", "b"]
    # a RESTARTED bike counts from 1 again; that must not go silent
    msgs, seen = T.new_messages({"log": [[1, "link", "c"]]}, seen)
    assert [m[2] for m in msgs] == ["c"]


def test_the_readout_shows_the_bikes_latest_message():
    tel = sample(log=[[3, "link", "LINK BACK from 192.168.0.114"]])
    left, right = T.status_text(tel)
    assert dict(zip(left.split("\n"), right.split("\n")))["bike"] == \
        "LINK BACK from 192.168.0.114"


def test_a_hold_is_a_row_on_the_readout_while_it_lasts():
    """The level, not the message: a station that connects after the cut
    still sees why `r` does nothing."""
    left, right = T.status_text(sample(state="cut", hold="pack at 9.8 V"))
    assert "HELD" in left.split("\n") and "pack at 9.8 V" in right
    left, _ = T.status_text(sample())
    assert "HELD" not in left and "WARN" not in left
    left, right = T.status_text(sample(warn="pack low: 10.4 V"))
    assert "WARN" in left.split("\n") and "10.4 V" in right
