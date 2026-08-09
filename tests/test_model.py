"""Model correctness tests: compilation, couplings, geometry, and behavior."""

import copy

import mujoco
import numpy as np
import pytest

from aow_sim import geometry
from aow_sim.build_model import build_model, load_params


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def full_model(params):
    return build_model(params, variant="full")


@pytest.fixture(scope="module")
def testbed_model(params):
    return build_model(params, variant="testbed")


def _step_for(model, data, seconds):
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qacc)), "simulation blew up"


def test_variants_compile_with_expected_dofs(full_model, testbed_model):
    # full: free(6) + steer + front + hub + ring + 8 rollers + 2 inputs = 20
    assert full_model.nv == 20
    assert full_model.nu == 3  # drive_a, drive_b, steer
    # testbed: hub + ring + 8 rollers + 2 inputs = 12
    assert testbed_model.nv == 12
    assert testbed_model.nu == 2
    # 8 roller couplings + 2 gearbox tendon constraints
    assert full_model.neq == 10


def test_steering_joint_unlimited(full_model):
    j = full_model.joint("steer_joint")
    assert not j.limited[0], "steering must allow continuous 360°+ rotation"


def test_envelope_matches_outer_radius(params):
    ow = params["omni_wheel"]
    dev = geometry.envelope_deviation(
        ow["outer_radius"], ow["axle_mount_radius"], ow["roller"]
    )
    assert dev < 0.001, f"cone envelope deviates {dev*1000:.2f} mm from wheel outer radius"


def test_roller_coupling_ratio(params, testbed_model):
    """Drive the ring input: every roller spins by the identical angle, k x ring."""
    m = testbed_model
    d = mujoco.MjData(m)
    d.ctrl[m.actuator("drive_b").id] = 5.0
    _step_for(m, d, 1.0)
    ring = d.qpos[m.joint("ring_spin").qposadr[0]]
    rollers = np.array(
        [d.qpos[m.joint(f"roller_spin_{i}").qposadr[0]] for i in range(8)]
    )
    assert abs(ring) > 1.0, "ring did not spin"
    assert np.ptp(rollers) < 1e-9, "rollers out of sync (rigid gearing violated)"
    k = params["drivetrain"]["k_roller"]
    assert rollers[0] / ring == pytest.approx(k, rel=0.01)


def test_gearbox_mixing(params, testbed_model):
    """Differential: hub (carrier) tracks mix_hub_a * input A when B is held."""
    m = testbed_model
    d = mujoco.MjData(m)
    d.ctrl[m.actuator("drive_a").id] = 4.0
    _step_for(m, d, 1.0)
    hub_v = d.qvel[m.joint("hub_spin").dofadr[0]]
    in_v = d.qvel[m.joint("input_a_spin").dofadr[0]]
    assert hub_v == pytest.approx(in_v * params["drivetrain"]["mix_hub_a"], rel=0.02)


def test_rest_stability(params):
    """With training wheels, the bike stands still: no jitter, drift, or sinking."""
    m = build_model(params, variant="full", training_wheels=True)
    d = mujoco.MjData(m)
    _step_for(m, d, 2.0)
    assert np.linalg.norm(d.qpos[:2]) < 0.005, "bike drifted at rest"
    r_rear = params["omni_wheel"]["outer_radius"]
    assert d.qpos[2] > r_rear - 0.002, "bike sank into the floor"
    assert np.abs(d.qvel).max() < 0.05, "bike jitters at rest"


def test_falls_without_support(params, full_model):
    """No training wheels, no control: the bike tips over like a real bike."""
    d = mujoco.MjData(full_model)
    d.qvel[3] = 0.1  # small roll-rate nudge off the unstable equilibrium
    _step_for(full_model, d, 2.0)
    up_z = np.zeros(9)
    mujoco.mju_quat2Mat(up_z, d.qpos[3:7])
    assert up_z[8] < 0.5, "bike should have fallen over (chassis z-axis tilted > 60°)"


def test_forward_roll(params):
    """Equal drive input rolls the bike forward near the rigid-rolling speed."""
    m = build_model(params, variant="full", training_wheels=True)
    d = mujoco.MjData(m)
    for tag in ("drive_a", "drive_b"):
        d.ctrl[m.actuator(tag).id] = 6.0
    _step_for(m, d, 2.0)
    assert d.qpos[0] > 0.4, f"only advanced {d.qpos[0]:.3f} m"
    assert abs(d.qpos[1]) < 0.05, "veered sideways under symmetric drive"


def test_lateral_crawl(params):
    """Opposed inputs = pure differential: rollers spin, rear wheel crawls
    sideways with (almost) no forward roll (hub = mean of the ring gears = 0)."""
    m = build_model(params, variant="full", training_wheels=True)
    d = mujoco.MjData(m)
    d.ctrl[m.actuator("drive_a").id] = 4.0
    d.ctrl[m.actuator("drive_b").id] = -4.0
    _step_for(m, d, 2.0)
    assert abs(d.qpos[1]) > 0.05, "no lateral crawl (AOW signature behavior missing)"
    # Trail makes the front end self-steer slightly during crawl, so some
    # forward arc is physical; it must just not dominate the lateral motion.
    assert abs(d.qpos[0]) < 0.7 * abs(d.qpos[1]), "opposed drive should mostly crawl, not roll"


def test_sensors_present(full_model):
    for name in ("ahrs_gyro", "ahrs_accel", "ahrs_quat",
                 "steer_pos", "steer_vel",
                 "input_a_pos", "input_a_vel", "input_b_pos", "input_b_vel"):
        assert full_model.sensor(name) is not None


# --------------------------------------------------------------------------
# The self-righting mechanisms (docs/plans/self-righting.md). Both are opt-in
# and mutually exclusive; neither may touch the bike every other test uses.


@pytest.fixture(scope="module")
def wing_model(params):
    return build_model(params, variant="full", righting=True, wings=True)


def test_righting_mechanisms_are_exclusive(params):
    """`righting=True` is the single arm, `wings=True` swaps it for the pair,
    and neither leaks into the default bike. The arm's mass would otherwise
    land in every wing torque reading and make the two studies incomparable."""
    from aow_sim.build_model import build_spec

    plain = build_spec(params).to_xml()
    arm = build_spec(params, righting=True).to_xml()
    wings = build_spec(params, righting=True, wings=True).to_xml()
    assert "wing" not in plain and "righting" not in plain
    assert "wing" not in arm and "righting_arm" in arm
    assert "righting_arm" not in wings and "wing_left_joint" in wings
    # Both mechanisms share the bumper pads that set the resting attitude.
    assert "bumper_left" in arm and "bumper_left" in wings


def test_wings_build(wing_model, params):
    w = params["righting"]["wings"]
    for name in ("wing_left_joint", "wing_right_joint"):
        assert wing_model.joint(name) is not None
    for name in ("wings_pos", "wings_vel"):
        assert wing_model.sensor(name) is not None
    # One actuator for the PAIR: the gear train is what makes a single servo
    # enough, so a second actuator here would be modelling a second servo.
    act = wing_model.actuator("wings")
    tau = params["servos"]["xc330_t181"]["stall_torque"] * w["gear_ratio"]
    assert act.forcerange == pytest.approx([-tau, tau])
    assert wing_model.joint(act.trnid[0]).name == "wing_right_joint"
    # Mechanical stops, per side: the pair can never be driven backwards
    # through stow into the chassis.
    assert np.degrees(wing_model.joint("wing_right_joint").range) == \
        pytest.approx([w["stow_deg"], w["deploy_deg"]])
    assert np.degrees(wing_model.joint("wing_left_joint").range) == \
        pytest.approx([-w["deploy_deg"], w["stow_deg"]])


def test_wings_mirror_through_the_reversal_gear(wing_model, params):
    """The equality constraint IS the reversal gear: driving the right wing
    must carry the left one through an equal and opposite angle.

    Run on a chassis pinned in place, so what is measured is the coupling and
    not a bike toppling out from under it. Free-flying, the pair still mirrors
    to ~0.7 deg once one wing is loaded against the floor and the other is in
    the air -- soft-constraint compliance, which is roughly what real gear
    backlash would look like anyway.
    """
    w = params["righting"]["wings"]
    d = mujoco.MjData(wing_model)
    mujoco.mj_forward(wing_model, d)
    pose, vel = d.qpos[:7].copy(), np.zeros(6)
    aid = wing_model.actuator("wings").id
    right = wing_model.joint("wing_right_joint").qposadr[0]
    left = wing_model.joint("wing_left_joint").qposadr[0]
    cmd, worst = 0.0, 0.0
    for _ in range(int(2.0 / wing_model.opt.timestep)):
        cmd = min(cmd + 0.7 * wing_model.opt.timestep, np.deg2rad(w["deploy_deg"]))
        d.ctrl[aid] = cmd
        mujoco.mj_step(wing_model, d)
        d.qpos[:7], d.qvel[:6] = pose, vel      # pin the chassis
        worst = max(worst, abs(d.qpos[left] - w["mirror"] * d.qpos[right]))
    assert np.degrees(d.qpos[right]) > 60, "the pair never deployed"
    # Loose on purpose: this catches a coupling that is MISSING or has the
    # wrong sign/coefficient, which is wrong by tens of degrees. Tightening it
    # to fractions of a degree only makes it brittle to geometry sweeps -- a
    # longer crank swings more inertia and loads the soft equality harder.
    assert np.degrees(worst) < 0.5, \
        f"wings drifted apart by {np.degrees(worst):.3f} deg"


def test_stowed_wings_clear_the_recoverable_set(params, wing_model):
    """Stowed, the wings are the lowest outboard thing on the bike, so they
    decide the lean at which something other than a tyre touches down. That
    has to stay outside the policy's recoverable set (up to 30.9 deg, see
    analysis/no_return.py) or a savable lean turns into a scrape.

    Same construction as analysis/self_righting.py's `landscape`: hold the bike
    clear of the floor at each roll angle and ask which geom is nearest it,
    rather than rotating about a fixed axle and hoping the wheel stays put."""
    d = mujoco.MjData(wing_model)
    floor = wing_model.geom("floor").id
    dyn = [i for i in range(wing_model.ngeom)
           if wing_model.geom_contype[i] and i != floor]
    wheels = {"front_tire"} | {f"roller_{i}_{s}" for i in range(8) for s in "ab"}
    touch = None
    for deg in range(0, 61):
        a = np.deg2rad(deg) / 2
        d.qpos[:] = 0.0
        d.qpos[2] = 0.5                          # clear of the floor
        d.qpos[3:7] = [np.cos(a), np.sin(a), 0.0, 0.0]
        mujoco.mj_forward(wing_model, d)
        gaps = [mujoco.mj_geomDistance(wing_model, d, floor, g, 2.0, None)
                for g in dyn]
        nearest = mujoco.mj_id2name(wing_model, mujoco.mjtObj.mjOBJ_GEOM,
                                    dyn[int(np.argmin(gaps))])
        if nearest not in wheels:
            touch = (deg, nearest)
            break
    assert touch is not None and touch[0] > 35, \
        f"stowed mechanism touches down at {touch} of roll"


def test_stowed_wings_park_outboard_of_the_drive_servos(wing_model, params):
    """The dogleg must crank OUTBOARD, so the stowed leg parks alongside the
    bike rather than converging on the centreline.

    This is the one wing constraint the physics cannot enforce: the wings
    collide with the floor and with nothing else, so an inboard crank puts the
    stowed leg straight through the drive servos (|y| = 15.75..44.25 mm) and
    the simulation runs perfectly happily."""
    d = mujoco.MjData(wing_model)
    mujoco.mj_forward(wing_model, d)
    servo = params["servos"]["xc430_w150"]
    face = abs(servo["pos_right"][1]) + servo["box_size"][1] / 2
    for tag in ("left", "right"):
        for part in ("leg", "foot"):
            y = abs(float(d.geom_xpos[wing_model.geom(f"wing_{tag}_{part}").id][1]))
            assert y >= face, \
                f"stowed wing_{tag}_{part} at |y|={y * 1000:.1f} mm is inboard " \
                f"of the drive servo face at {face * 1000:.1f} mm"


def test_wing_gear_train_fits(params):
    """The two wing gears MESH EACH OTHER and the servo drives one of them, so
    each disc is half the pivot spacing and the ratio only sizes the pinion.

    The property worth pinning is the DECOUPLING: with a central pinion the
    disc grew with the ratio, so buying torque widened the bike. Here the
    envelope is ratio-independent."""
    from aow_sim.build_model import wing_fit

    w = params["righting"]["wings"]
    f = wing_fit(params)
    # Equal discs on pivots 2*half_span apart -> each radius IS the half-span.
    assert f["disc_radius"] == pytest.approx(w["pivot"][1])
    assert f["disc_radius"] / f["pinion_radius"] == pytest.approx(w["gear_ratio"])
    assert not f["pinion_too_small"], "configured ratio needs an unprintable pinion"
    assert not f["grounds_out"], "the driven disc is bigger than the pivot height"
    assert w["gear_ratio"] <= f["max_ratio"]

    # More reduction still means a smaller pinion, and that is now the ONLY
    # ceiling on the ratio...
    lo, hi = copy.deepcopy(params), copy.deepcopy(params)
    lo["righting"]["wings"]["gear_ratio"] = 2.0
    hi["righting"]["wings"]["gear_ratio"] = 8.0
    assert wing_fit(hi)["pinion_radius"] < wing_fit(lo)["pinion_radius"]
    assert wing_fit(hi)["pinion_too_small"]
    # ...while the disc, and therefore the whole envelope, does not move at all.
    assert wing_fit(hi)["disc_radius"] == pytest.approx(wing_fit(lo)["disc_radius"])
    assert wing_fit(hi)["min_bike_width"] == pytest.approx(wing_fit(lo)["min_bike_width"])
    # The envelope is wide enough for the crank to clear its own disc.
    assert params["righting"]["bike_width"] >= f["min_bike_width"] - 1e-9


def test_wing_crank_clears_the_driven_disc(params):
    """The crank exists to carry the leg past the rim of its own gear. Below
    the disc radius the wing stands on the gear instead of on the floor --
    which the physics will not catch, since the discs are non-colliding."""
    from aow_sim.build_model import wing_fit

    short, long = copy.deepcopy(params), copy.deepcopy(params)
    disc = wing_fit(params)["disc_radius"]
    # Crank reach is the OUTBOARD component, so it depends on angle as well as
    # length -- swinging the crank inboard of 90 deg shortens the reach.
    short["righting"]["wings"].update(crank_length=disc * 0.5, crank_deg=90.0)
    long["righting"]["wings"].update(crank_length=disc * 1.5, crank_deg=90.0)
    assert wing_fit(short)["leg_stands_on_gear"]
    assert not wing_fit(long)["leg_stands_on_gear"]
    # Same length, cranked inboard -> reach shrinks by sin, and it can fail
    # even when the length alone would have cleared.
    tilted = copy.deepcopy(long)
    tilted["righting"]["wings"]["crank_deg"] = 30.0
    assert wing_fit(tilted)["crank_reach"] < wing_fit(long)["crank_reach"]
    assert wing_fit(tilted)["leg_stands_on_gear"]


def test_righting_envelope_is_derived_and_tangent(params):
    """The roof and the stowed wings are ONE envelope: the roof is the circle
    circumscribing the stowed wing tips, so the tips lie exactly on it.

    That tangency is the whole point -- upside down, tips ON the rolling
    surface cannot prop the bike up, while tips OUTSIDE it become outriggers
    and catch the bike part-way over (it stuck at 154 deg before the two were
    coupled). It is a geometric identity, so it is pinned here rather than
    left to a sweep to rediscover."""
    from aow_sim.params import derive_righting

    rg = params["righting"]
    w, roof = rg["wings"], rg["roof"]
    tip_y = w["pivot"][1] + w["crank_length"] * np.sin(np.deg2rad(w["crank_deg"]))
    tip_z = (w["pivot"][2] + w["crank_length"] * np.cos(np.deg2rad(w["crank_deg"]))
             + w["length"])
    assert tip_y == pytest.approx(rg["bike_width"] / 2)
    assert tip_z + roof["radius"] == pytest.approx(rg["bike_height"])
    assert np.hypot(tip_y, tip_z - roof["height"]) == pytest.approx(roof["radius"])

    # Driving the envelope moves all four derived dimensions together.
    wider = copy.deepcopy(params)
    for k in ("radius", "height"):
        wider["righting"]["roof"].pop(k)
    for k in ("crank_length", "length"):
        wider["righting"]["wings"].pop(k)
    wider["righting"]["bike_width"] = 0.150
    derive_righting(wider)
    w2, r2 = wider["righting"]["wings"], wider["righting"]["roof"]
    assert r2["radius"] == pytest.approx(0.075)
    assert w2["crank_length"] > w["crank_length"]      # tip pushed outboard
    tip2_y = w2["pivot"][1] + w2["crank_length"] * np.sin(np.deg2rad(w2["crank_deg"]))
    tip2_z = (w2["pivot"][2] + w2["crank_length"] * np.cos(np.deg2rad(w2["crank_deg"]))
              + w2["length"])
    assert np.hypot(tip2_y, tip2_z - r2["height"]) == pytest.approx(r2["radius"])
