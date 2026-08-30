"""General command-conditioned policy tests.

The spec tests are dependency-free and always run; env tests need gymnasium;
the trainer test needs stable-baselines3; the replay test skips until a
policy has been trained.
"""

import numpy as np
import pytest

from aow_sim.control.general_spec import (ACT_DIM, OBS_DIM, ActionBounds,
                                          build_obs, command_to_body,
                                          scale_action)

# general_spec's obs/action layout, shared by the trainer and the replay.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.spec


def _obs(**kw):
    """build_obs with a nominal argument set; override by keyword."""
    a = dict(roll=0.02, roll_rate=-0.1, yaw_rate=0.3, steer=0.4,
             steer_rate=0.5, v_lon=0.8, v_lat=-0.05,
             v_cmd_lon=1.0, v_cmd_lat=0.0, psi_err=0.25,
             prev_action=[0.1, -0.2, 0.3])
    a.update(kw)
    return build_obs(**a)


def test_obs_spec_shape_and_encodings():
    obs = _obs()
    assert obs.shape == (OBS_DIM,)
    assert np.all(np.isfinite(obs))
    assert ACT_DIM == 3
    # sin/cos pairs live on the unit circle: 2*steer at (3,4), psi_err at (10,11)
    for i, j in ((3, 4), (10, 11)):
        assert obs[i]**2 + obs[j]**2 == pytest.approx(1.0, abs=1e-6)
    # prev_action is fed back (the last ACT_DIM slots)
    # prev_action occupies 12..14. It is NOT `obs[-3:]` any more: a policy
    # trained with a velocity window appends v_bar after it, deliberately, so
    # that the un-windowed layout stays a strict PREFIX of the windowed one.
    assert obs[12:15] == pytest.approx([0.1, -0.2, 0.3], abs=1e-6)


def test_obs_is_stationary():
    """The defining property of a general policy: the observation carries no
    notion of elapsed time or of when the policy was engaged. Identical state
    + identical command must give an identical obs, always.

    This is what `phase` (present in every finite-horizon move spec) breaks,
    and why those specs cannot be reused for an always-on controller."""
    import inspect
    from aow_sim.control import general_spec

    src = inspect.getsource(general_spec.build_obs)
    assert "phase" not in src, "general obs must not carry an episode clock"
    # calling twice with the same arguments is identical by construction;
    # the real content of this test is that build_obs takes no time/step
    # argument at all
    sig = inspect.signature(general_spec.build_obs).parameters
    for banned in ("phase", "t", "tau", "step", "elapsed", "horizon"):
        assert banned not in sig, f"build_obs must not take `{banned}`"
    assert _obs() == pytest.approx(_obs(), abs=0.0)


def test_steer_encoding_is_winding_invariant():
    """sin/cos(2*steer): the wheel is front-back symmetric, so a pi shift and
    any multi-turn winding must encode identically — this is what lets the
    general policy read raw multi-turn qpos with no pi-park rebasing."""
    base = _obs(steer=0.4)
    for shift in (np.pi, -np.pi, 2 * np.pi, 6 * np.pi):
        assert _obs(steer=0.4 + shift)[3:5] == pytest.approx(base[3:5], abs=1e-4)


def test_heading_is_mod_two_pi():
    """Unlike steer, the chassis has a front: heading must NOT be pi-symmetric
    (facing backward is not the same as facing forward)."""
    fwd, back = _obs(psi_err=0.0), _obs(psi_err=np.pi)
    assert not np.allclose(fwd[10:12], back[10:12], atol=1e-3)
    assert _obs(psi_err=2 * np.pi)[10:12] == pytest.approx(fwd[10:12], abs=1e-5)


def test_command_to_body_frame():
    """The command is a velocity VECTOR in the body frame, so stop and
    reverse are continuous rather than singular (a polar course/speed command
    is undefined at zero speed and flips discontinuously on reverse)."""
    # facing +x, commanded +x at 1 m/s -> pure longitudinal
    v_lon, v_lat, err = command_to_body((1.0, 0.0), 0.0, 0.0)
    assert (v_lon, v_lat) == pytest.approx((1.0, 0.0), abs=1e-9)
    assert err == pytest.approx(0.0)
    # facing +x, commanded +y -> pure lateral (+Y is left)
    v_lon, v_lat, _ = command_to_body((0.0, 1.0), 0.0, 0.0)
    assert (v_lon, v_lat) == pytest.approx((0.0, 1.0), abs=1e-9)
    # yaw-equivariance: rotate bike and command together -> same body command
    for psi in (0.3, 1.9, -2.7):
        c, s = np.cos(psi), np.sin(psi)
        got = command_to_body((c * 0.7 - s * 0.2, s * 0.7 + c * 0.2), psi, psi)
        assert got[:2] == pytest.approx((0.7, 0.2), abs=1e-9)
    # stop and reverse are ordinary points, not singularities
    assert command_to_body((0.0, 0.0), 0.0, 0.0)[:2] == pytest.approx((0.0, 0.0))
    assert command_to_body((-0.6, 0.0), 0.0, 0.0)[:2] == pytest.approx((-0.6, 0.0))
    # heading error wraps to (-pi, pi]
    assert command_to_body((0, 0), 3.0 * np.pi, 0.0)[2] == pytest.approx(np.pi)
    assert abs(command_to_body((0, 0), 0.0, 3.0)[2]) <= np.pi + 1e-9


@pytest.mark.policy
def test_action_contract_shared_with_moves():
    b = ActionBounds(8.0, 1.4, 40.0)
    sr, hub, diff = scale_action([2.0, -2.0, 0.5], b)      # clips to [-1, 1]
    assert (sr, hub, diff) == pytest.approx((8.0, -1.4, 20.0))
    assert scale_action([0.5, 0.5], b)[2] == 0.0           # feedforward form


# -- env ------------------------------------------------------------------

def _env(**env_over):
    pytest.importorskip("gymnasium")
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    cfg = _load_rl_config()
    cfg = {**cfg, "env": {**cfg["env"], "ball_prob": 0.0, **env_over}}
    return GeneralEnv(rl_cfg=cfg, seed=0)


def test_env_reset_step():
    env = _env()
    assert env.observation_space.shape == (OBS_DIM,)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (OBS_DIM,) and np.all(np.isfinite(obs))
    rng = np.random.default_rng(1)
    term = trunc = False
    steps = 0
    while not (term or trunc) and steps < env.max_steps + 1:
        obs, r, term, trunc, info = env.step(rng.uniform(-1, 1, 3))
        assert np.all(np.isfinite(obs)) and np.isfinite(r)
        steps += 1
    assert term or trunc
    # determinism
    o1, _ = _env().reset(seed=5)
    o2, _ = _env().reset(seed=5)
    assert np.allclose(o1, o2)


def test_terminates_only_on_fall():
    """An always-on controller has no success state to stop at — the only
    early exit is falling over."""
    env = _env()
    env.reset(seed=0)
    term = trunc = False
    info = {}
    for _ in range(env.max_steps + 1):
        _o, _r, term, trunc, info = env.step(np.zeros(3))
        if term or trunc:
            break
    assert not (term and not info["fell"]), "terminated without falling"
    if term:
        assert info["fell"]


def test_command_resamples_mid_episode_as_step_change():
    """The operator can stop or reverse instantly, so commands are redrawn as
    step changes mid-episode — not ramped.

    Resampling is forced fast here (the shipped 1.5-4.0 s is longer than an
    untrained policy survives) so the mechanism is exercised on the real
    step() path rather than by poking internals."""
    env = _env(resample_s=[0.06, 0.08])
    env.reset(seed=3)
    env.set_difficulty(1.0)
    jumps, sizes = 0, []
    prev = env._v_cmd_w.copy()
    for _ in range(env.max_steps):
        _o, _r, term, trunc, _i = env.step(np.zeros(3))
        if not np.allclose(env._v_cmd_w, prev):
            jumps += 1
            sizes.append(float(np.linalg.norm(env._v_cmd_w - prev)))
            prev = env._v_cmd_w.copy()
        if term or trunc:
            break
    assert jumps >= 1, "command never resampled"
    # a step change, not a ramp: each jump is a finite discontinuity
    assert max(sizes) > 1e-2


def test_eval_options_hold_a_fixed_command():
    """The deterministic eval grid pins one command for a whole episode."""
    env = _env()
    env.reset(seed=0, options={"v_cmd": (0.5, 0.0), "psi_cmd_rel": np.pi / 2,
                               "difficulty": 1.0})
    v0, psi0 = env._v_cmd_w.copy(), env._psi_cmd
    for _ in range(40):
        _o, _r, term, trunc, _i = env.step(np.zeros(3))
        if term or trunc:
            break
    assert np.allclose(env._v_cmd_w, v0) and env._psi_cmd == pytest.approx(psi0)


def _rw_env(w_smooth, **env_over):
    pytest.importorskip("gymnasium")
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    cfg = _load_rl_config()
    cfg = {**cfg,
           "env": {**cfg["env"], "ball_prob": 0.0, **env_over},
           "reward": {**cfg["reward"], "w_smooth": w_smooth},
           "randomization": {**cfg["randomization"], "enabled": False}}
    return GeneralEnv(rl_cfg=cfg, seed=0)


def test_per_channel_w_smooth_reduces_to_the_scalar():
    """The three action channels are not interchangeable -- under a uniform
    weight `steer` and `hub` gave up their chatter and `diff` did not -- so
    w_smooth may be given per channel. The list form must be a strict
    generalisation: all three entries equal has to reproduce the scalar
    reward EXACTLY, or every config predating the change silently moves.
    """
    rng = np.random.default_rng(0)
    acts = rng.uniform(-1, 1, (60, 3))

    def rewards(w):
        env = _rw_env(w)
        env.reset(seed=11)
        env.set_difficulty(1.0)
        out = []
        for a in acts:
            _o, r, term, trunc, _i = env.step(a)
            out.append(r)
            if term or trunc:
                break
        return np.array(out)

    scalar = rewards(0.05)
    assert rewards([0.05, 0.05, 0.05]) == pytest.approx(scalar, abs=0.0)
    # ... and pricing one channel differently actually changes the reward, in
    # the direction of a bigger penalty on a channel that is moving.
    diff_priced = rewards([0.05, 0.05, 0.25])
    assert not np.allclose(diff_priced, scalar)
    assert np.all(diff_priced <= scalar + 1e-12)


def test_per_channel_w_smooth_rejects_a_short_list():
    """A 2-list against 3 channels would quietly leave `diff` unpriced --
    which is exactly the experiment being run, so it must never happen by
    accident."""
    from aow_sim.control.general_env import _per_channel

    assert _per_channel(0.05, 3, "w") == pytest.approx([0.05] * 3)
    assert _per_channel([0.05, 0.05, 0.25], 2, "w") == pytest.approx([0.05] * 2)
    with pytest.raises(ValueError, match="action channels"):
        _per_channel([0.05, 0.05], 3, "w")


def test_curriculum_widens_command_range():
    """Difficulty scales the sampled command envelope: gentle at 0, full at 1."""
    env = _env()
    env.reset(seed=0)
    rng = np.random.default_rng(0)

    def spread(diff, n=200):
        env.set_difficulty(diff)
        env._psi = 0.0
        env._step = 0
        speeds, heads = [], []
        for _ in range(n):
            env._sample_command(rng)
            speeds.append(np.linalg.norm(env._v_cmd_w))
            heads.append(abs(env._psi_cmd))
        return max(speeds), max(heads)

    v_lo, h_lo = spread(0.0)
    v_hi, h_hi = spread(1.0)
    assert v_hi > v_lo and h_hi > h_lo
    assert v_hi <= env.v_max * 1.5 + 1e-6      # within the configured envelope
    assert h_lo <= np.deg2rad(30.0) + 1e-6     # gentle heading steps at diff 0


def test_p_v_zero_alone_cannot_produce_the_eval_grids_hold():
    """`p_v_zero` zeroes v_lon only, so its draws are not the HOLD command.

    train_general_rl's eval grid carries a `hold` family -- v_lon, v_lat and
    the heading step all exactly zero -- and it is the only family where drift
    is defined. This pins WHY that command needs its own draw: at any live
    difficulty the lateral term is a continuous variate and the heading step
    is another, so a zero-forward draw still crabs and still turns.
    """
    env = _env(p_v_zero=1.0, p_cmd_hold=0.0)
    env.reset(seed=0)
    env.set_difficulty(0.15)      # the shipped curriculum `start`
    rng = np.random.default_rng(0)
    for _ in range(200):
        env._psi = 0.0
        env._sample_command(rng)
        assert np.linalg.norm(env._v_cmd_w) > 0.0, (
            "a zero-forward draw came out as a full hold, which would make "
            "p_cmd_hold redundant -- check v_lat_frac and the difficulty")


def test_p_cmd_hold_draws_the_eval_grids_hold_command():
    """`p_cmd_hold` produces exactly (0, 0) velocity and freezes the heading.

    Freezing rather than re-anchoring is the contract: `drive.set_command`
    leaves `_gen_psi_cmd` alone when the operator releases the stick, so the
    trained idle state must be "converge to the last commanded heading and
    stay there", not "adopt whatever heading I happen to have".
    """
    env = _env(p_cmd_hold=1.0)
    env.reset(seed=0)
    env.set_difficulty(1.0)       # hardest case: crab and +-pi both wide open
    rng = np.random.default_rng(0)

    env._psi_cmd = 0.7            # a command the bike has not reached yet
    for psi in (0.0, 0.3, -1.2):
        env._psi = psi
        env._sample_command(rng)
        assert np.all(env._v_cmd_w == 0.0)
        assert env._psi_cmd == 0.7, "the heading command must not be re-anchored"

    # At reset there is no previous command to freeze, so it anchors.
    env._psi = 0.42
    env._sample_command(rng, first=True)
    assert np.all(env._v_cmd_w == 0.0)
    assert env._psi_cmd == pytest.approx(0.42)


def test_p_cmd_hold_defaults_off_so_existing_configs_are_unchanged():
    """Every config written before the knob must describe the run it trained."""
    env = _env()
    assert env.p_cmd_hold == 0.0
    assert env.families is None, "the family sampler must be opt-in too"


# -- the family sampler (cmd_families) ------------------------------------

_FAMILIES = {"hold_min": 0.10, "hold_decay": 3.0,
             "straight":      {"onset": 0.00, "full": 0.30, "weight": 1.5},
             "turn_in_place": {"onset": 0.05, "full": 0.45, "weight": 1.0},
             "moving_turn":   {"onset": 0.20, "full": 0.70, "weight": 2.0}}


def _fam_env(**over):
    return _env(cmd_families=_FAMILIES, v_lat_frac=0.0, **over)


def _draw_families(env, d, n=8000, seed=0):
    """Classify n draws at difficulty `d` into the four cells."""
    rng = np.random.default_rng(seed)
    env.set_difficulty(d)
    counts = dict(hold=0, in_place=0, straight=0, moving_turn=0)
    for _ in range(n):
        env._psi = env._psi_cmd = 0.0
        env._sample_family_command(rng)
        moving = np.linalg.norm(env._v_cmd_w) > 0.0
        turning = env._psi_cmd != 0.0
        counts["moving_turn" if (moving and turning) else "straight" if moving
               else "in_place" if turning else "hold"] += 1
    return {k: v / n for k, v in counts.items()}


def test_the_four_families_partition_the_command_space():
    """With the lateral command dropped, every draw is exactly one family.

    That is what makes the weights readable against the eval grid: its
    sixteen non-crab commands split 1 / 5 / 3 / 7 across these same cells.
    A non-zero `v_lat_frac` would break the partition silently.
    """
    env = _fam_env()
    env.reset(seed=0)
    for d in (0.0, 0.3, 0.7, 1.0):
        assert sum(_draw_families(env, d).values()) == pytest.approx(1.0)


def test_hold_ramps_out_and_the_others_ramp_in():
    """Difficulty 0 is a PURE hold stage; hold decays to its floor at d=1."""
    env = _fam_env()
    env.reset(seed=0)
    lo, hi = _draw_families(env, 0.0), _draw_families(env, 1.0)

    assert lo["hold"] == 1.0, "difficulty 0 must be nothing but holds"
    assert hi["hold"] == pytest.approx(_FAMILIES["hold_min"], abs=0.02)
    # Each of the other three is absent at d=0 and present at d=1.
    for k in ("straight", "in_place", "moving_turn"):
        assert lo[k] == 0.0 and hi[k] > 0.1, k
    # `straight` opens first and `moving_turn` last -- the onsets, observed.
    mid = _draw_families(env, 0.15)
    assert mid["straight"] > 0.0 and mid["moving_turn"] == 0.0


def test_family_magnitudes_respect_their_floors():
    """A non-zero draw is never small enough to impersonate its neighbour.

    Without floors a 2 deg "turn in place" is a hold and a 0.01 m/s "straight"
    is one too, which would put the families back to overlapping -- the exact
    failure the explicit mix exists to remove.
    """
    env = _fam_env()
    env.reset(seed=0)
    env.set_difficulty(1.0)
    rng = np.random.default_rng(1)
    for _ in range(4000):
        env._psi = env._psi_cmd = 0.0
        env._sample_family_command(rng)
        speed = float(np.linalg.norm(env._v_cmd_w))
        step = abs(env._psi_cmd)
        assert speed == 0.0 or speed >= env.v_lo - 1e-9
        assert step == 0.0 or step >= env.step_lo - 1e-9


def test_signed_magnitude_is_two_sided_and_shaped():
    """Sign is a coin flip; the density leans on the exponent, not the range.

    Exponent 0 is the flat draw the legacy sampler used. Raising it must move
    mass toward the ceiling -- that is the whole reason it exists, since a
    widening uniform dilutes its own large-turn tail as the span grows.
    """
    from aow_sim.control.general_env import GeneralEnv
    rng = np.random.default_rng(0)
    flat = np.array([GeneralEnv._signed_magnitude(rng, 10.0, 180.0, 0.0)
                     for _ in range(20000)])
    lean = np.array([GeneralEnv._signed_magnitude(rng, 10.0, 180.0, 1.0)
                     for _ in range(20000)])
    for x in (flat, lean):
        assert np.mean(x > 0) == pytest.approx(0.5, abs=0.02)
        assert np.all(np.abs(x) >= 10.0 - 1e-9) and np.all(np.abs(x) <= 180.0)
    assert np.mean(np.abs(lean)) > np.mean(np.abs(flat)) + 15.0


def test_hold_max_keeps_a_motion_command_in_the_mix_at_difficulty_zero():
    """`hold_max` < 1 is what stops difficulty 0 being a pure standstill.

    Arm 1 ran at 100% hold there, learned a hold drifting backwards at
    0.192 m/s, and that scores `0.5*(exp(-(0.192/0.35)^2) + 1) = 0.87` against
    `advance_score: 0.6` -- so the gate certified it and forward motion never
    appeared in 20M steps. With `straight` in the mix from step 0, clearing the
    gate needs hub in both directions.
    """
    fam = {**_FAMILIES, "hold_max": 0.4,
           "straight": {"onset": 0.0, "full": 0.0, "weight": 1.5}}
    env = _env(cmd_families=fam, v_lat_frac=0.0)
    env.reset(seed=0)
    at_zero = _draw_families(env, 0.0)
    assert at_zero["hold"] == pytest.approx(0.4, abs=0.02)
    assert at_zero["straight"] == pytest.approx(0.6, abs=0.02)
    # and the d=1 end is untouched by the new knob
    assert _draw_families(env, 1.0)["hold"] == pytest.approx(0.1, abs=0.02)


def test_hold_max_defaults_to_one_so_arm_one_is_unchanged():
    env = _env(cmd_families=_FAMILIES, v_lat_frac=0.0)
    env.reset(seed=0)
    assert env.families["hold_max"] == 1.0
    assert _draw_families(env, 0.0)["hold"] == 1.0


def test_speed_ratio_is_signed_so_a_reversing_policy_is_legible():
    """The floor at 0.0 made every wrong-direction policy read exactly 0.000.

    `general_rl_cmd_curriculum` reported `speed_ratio_fwd` 0.000 at all twenty
    evals; underneath it was reversing at ~40% of the commanded forward speed.
    "Achieves nothing" and "drives the other way" need different fixes.
    """
    from aow_sim.train_general_rl import _behaviour_metrics

    def row(cmd, v_ach):
        return {"cmd": cmd, "v_ach": v_ach, "t_head_s": 1.0, "drift_m": 0.0,
                "steer_deg": 0.0, "track": 0.5, "fell": False,
                "vel_err": 0.0, "head_err_deg": 0.0, "steps": 750}

    m = _behaviour_metrics([
        row((0.0, 0.0, 0), 0.0),
        row((0.8, 0.0, 0), -0.32),   # told forward, drives BACKWARD
        row((-0.5, 0.0, 0), -0.5),   # reverse on target
    ])
    assert m["speed_ratio_fwd"] == pytest.approx(-0.4)
    assert m["speed_ratio_rev"] == pytest.approx(1.0)


def test_the_family_arm_config_is_wired_end_to_end():
    """The shipped arm must actually select the family sampler."""
    from aow_sim.build_model import load_params
    from aow_sim.control.general_env import GeneralEnv, _load_rl_config
    cfg = _load_rl_config("config/rl_general_cmd_curriculum.yaml")
    assert cfg["env"]["v_lat_frac"] == 0.0, "the partition needs no lateral"
    assert cfg["curriculum"]["start"] == 0.0, "d=0 is the pure-hold stage"
    assert "w_pitch" not in cfg["reward"] and cfg["env"]["obs_pitch"] is True
    # This arm reads the onboard estimate, so the env needs real params.
    env = GeneralEnv(load_params(), rl_cfg=cfg, seed=0)
    assert env.families is not None
    assert env.resample_s == (2.0, 6.0)


# -- trainer / replay -----------------------------------------------------

def test_eval_cmds_scale_with_v_max():
    pytest.importorskip("stable_baselines3")
    from aow_sim.train_general_rl import eval_cmds

    cmds = eval_cmds(1.2)
    assert len(cmds) >= 8
    assert max(abs(v) for v, _, _ in cmds) == pytest.approx(1.2)
    assert max(abs(d) for _, _, d in cmds) == pytest.approx(np.pi)
    # scales rather than being pinned
    assert max(abs(v) for v, _, _ in eval_cmds(0.6)) == pytest.approx(0.6)


def test_eval_grid_is_mirrored():
    """The grid must exercise both turn directions and both crab directions.
    It used to only ever turn right and crab left, so a one-handed policy
    scored flawlessly and snapshot selection could not see the asymmetry."""
    pytest.importorskip("stable_baselines3")
    from aow_sim.train_general_rl import _is_self_mirror, _mirror, _mirrored_grid

    grid = _mirrored_grid()
    assert len(grid) == len(set(grid)), "duplicate commands in the eval grid"
    for c in grid:
        if not _is_self_mirror(c):
            assert _mirror(c) in grid, f"{c} has no mirror in the grid"
    # both signs actually present, not just self-mirror rows
    assert any(d < 0 for _, _, d in grid) and any(d > 0 for _, _, d in grid)
    assert any(b < 0 for _, b, _ in grid) and any(b > 0 for _, b, _ in grid)
    # +-180 wraps to the same psi_err, so it cannot measure handedness; the
    # directional large turn does.
    assert (0.0, 0.0, 170.0) in grid and (0.0, 0.0, -170.0) in grid
    assert _is_self_mirror((0.0, 0.0, 180.0))


def test_score_penalises_an_abandoned_command():
    """Geometric mean, not arithmetic: a policy that gives up on one command
    (e.g. refuses to reverse) must not hide behind the ones it does well."""
    pytest.importorskip("stable_baselines3")
    from aow_sim.train_general_rl import _score

    uniform = {"survive_rate": 1.0, "track": 0.8, "track_geo": 0.8}
    assert _score(uniform) == pytest.approx(0.8)

    # Ten commands done well, two abandoned. The arithmetic mean barely
    # notices (0.8 -> 0.67); the geometric mean collapses (0.8 -> 0.26).
    import numpy as np
    abandoned = [0.8] * 10 + [0.0, 0.0]
    geo = lambda t: float(np.exp(np.mean(np.log(np.clip(t, 1e-3, 1.0)))))
    arith, g = float(np.mean(abandoned)), geo(abandoned)
    assert arith > 0.65, "arithmetic mean is what let this hide"
    assert g < 0.5 * arith, "geometric mean must not let it hide"
    assert _score({"survive_rate": 1.0, "track": arith, "track_geo": g}) < 0.3

    # metrics dicts written before track_geo existed still score
    assert _score({"survive_rate": 0.5, "track": 0.6}) == pytest.approx(0.3)


def test_behaviour_metrics_report_both_speed_directions():
    """A policy good at reverse but sluggish forward must be as visible as the
    reverse-refusing one -- a single 'reverse speed' number would reward
    exactly the failure mode being hunted."""
    pytest.importorskip("stable_baselines3")
    from aow_sim.train_general_rl import _behaviour_metrics

    def row(cmd, v_ach, t_head=1.0, drift=0.0, steer=0.0):
        return {"cmd": cmd, "v_ach": v_ach, "t_head_s": t_head, "drift_m": drift,
                "steer_deg": steer, "track": 0.5, "fell": False,
                "vel_err": 0.0, "head_err_deg": 0.0, "steps": 750}

    m = _behaviour_metrics([
        row((0.0, 0.0, 0), 0.0, drift=1.4, steer=12.0),   # hold station
        row((0.8, 0.0, 0), 0.2),                          # forward: sluggish
        row((-0.5, 0.0, 0), -0.5),                        # reverse: on target
        row((0.0, 0.0, 90), 0.0, t_head=2.0),             # right turn: quick
        row((0.0, 0.0, -90), 0.0, t_head=6.0),            # left turn: slow
    ])
    assert m["speed_ratio_fwd"] == pytest.approx(0.25)
    assert m["speed_ratio_rev"] == pytest.approx(1.0)
    assert m["drift_m"] == pytest.approx(1.4)
    assert m["steer_rest_deg"] == pytest.approx(12.0)
    assert m["turn_asym"] == pytest.approx((6.0 - 2.0) / 6.0, abs=1e-3)  # 3dp


def test_plot_command_branch_is_non_directional():
    """A heading command is only meaningful mod 2*pi (the policy sees
    sin/cos of the error), so a 180 deg snap is deliberately non-directional.
    The trace plot must resolve the command onto the turn the bike actually
    made instead of inventing a 360 deg tracking error."""
    from aow_sim.rollout_move import _cmd_branch

    # commanded +90 then +270; the bike satisfies the second the short way
    cmd = np.concatenate([np.full(50, np.pi / 2), np.full(50, 3 * np.pi / 2)])
    act = np.concatenate([np.full(50, np.pi / 2),
                          np.linspace(np.pi / 2, -np.pi / 2, 50)])
    out = _cmd_branch(cmd, act)
    assert out[:50] == pytest.approx(np.pi / 2)
    assert out[-1] == pytest.approx(-np.pi / 2), "should show -90, not +270"
    # one branch per held command (no mid-turn jump), and still the same
    # command modulo a whole turn
    assert np.allclose(out[50:], out[50])
    assert np.allclose(np.remainder(out - cmd, 2 * np.pi), 0.0, atol=1e-9)
    # a command already on the nearest branch is untouched
    same = _cmd_branch(np.full(10, 0.3), np.full(10, 0.25))
    assert same == pytest.approx(0.3)


@pytest.mark.policy
def test_general_move_replays():
    """THE CONFIGURED move loads and replays without NaN.

    Named for `control.general_move` and now actually reading it. It used to
    load `general_rl` by hand, so it passed whatever the config pointed at --
    including at a policy that did not exist.
    """
    from aow_sim.build_model import load_params
    from aow_sim.control.flick import MOVES_DIR
    from aow_sim.control.policy import load_policy_npz
    name = load_params()["control"].get("general_move", "general_rl")
    if not (MOVES_DIR / f"{name}.npz").exists():
        pytest.skip(f"control.general_move names {name}, which is not exported")
    pol = load_policy_npz(MOVES_DIR / f"{name}.npz")
    if pol.obs_dim != OBS_DIM:
        pytest.skip(f"moves/{name} predates the current obs spec — retrain")
    act = pol.action(_obs())
    assert len(act) == 3 and np.all(np.isfinite(act))


@pytest.mark.policy
def test_move_file_carries_the_lateral_envelope(tmp_path):
    """v_lat_frac rides in the move yaml so teleop clamps the crab command to
    what THIS policy trained on; files written before the field default to
    0.4, which is what every existing general_rl was trained with."""
    import yaml
    from aow_sim.control.flick import MOVES_DIR, load_move
    if not (MOVES_DIR / "general_rl.npz").exists():
        pytest.skip("run `python -m aow_sim.train_general_rl` first")

    doc = yaml.safe_load((MOVES_DIR / "general_rl.yaml").read_text())
    for frac, expect in ((None, 0.4), (0.25, 0.25)):
        d = dict(doc, policy_file=str(MOVES_DIR / "general_rl.npz"))
        d.pop("v_lat_frac", None)
        if frac is not None:
            d["v_lat_frac"] = frac
        p = tmp_path / "m.yaml"
        p.write_text(yaml.safe_dump(d))
        assert load_move("m", moves_dir=tmp_path).v_lat_frac == expect


def test_crab_command_is_perpendicular_to_heading():
    """A course_rel of +-pi/2 is a pure crab: the commanded world velocity is
    perpendicular to the commanded heading, and the heading itself is
    untouched. This is the whole basis of the teleop crab keys."""
    mujoco = pytest.importorskip("mujoco")  # noqa: F841
    from aow_sim.build_model import build_model, load_params
    from aow_sim.control import DriveController
    from aow_sim.control.flick import MOVES_DIR
    from aow_sim.control.linearize import settle_upright
    from aow_sim.run_drive import _command_ref, _fresh
    if not (MOVES_DIR / "general_rl.npz").exists():
        pytest.skip("run `python -m aow_sim.train_general_rl` first")

    params = load_params()
    model = build_model(params, variant="full")
    data = _fresh(model, settle_upright(model).qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    try:
        c.engage_general(data, name="general_rl")
    except ValueError:
        pytest.skip("moves/general_rl predates the current obs spec — retrain")

    for psi, course, want in (
            (0.0, np.pi / 2, (0.0, 0.3)),        # facing +X, crab left -> +Y
            (0.0, -np.pi / 2, (0.0, -0.3)),      # ... crab right      -> -Y
            (np.pi / 2, np.pi / 2, (-0.3, 0.0)),  # facing +Y, left     -> -X
    ):
        c.set_command_polar(0.3, course, psi_cmd=psi)
        h, v = _command_ref(c, data)
        assert np.allclose(v, want, atol=1e-9), (psi, course, v)
        assert h == pytest.approx(psi)

    # Combined: teleop sends one vector as (hypot, atan2) off the heading.
    v_lon, v_lat = 0.5, 0.3
    c.set_command_polar(float(np.hypot(v_lon, v_lat)),
                        float(np.arctan2(v_lat, v_lon)), psi_cmd=0.0)
    _, v = _command_ref(c, data)
    assert np.allclose(v, (v_lon, v_lat), atol=1e-9)


# -- velocity window (windowed velocity tracking) --------------------------

def test_vel_filter_alpha_is_rate_independent():
    """The env filters at 50 Hz and drive.py at the controller rate, so alpha
    must encode a CONTINUOUS time constant. `1 - exp(-dt/tau)` does; the naive
    `dt/tau` does not, and the two paths would then disagree about what the
    policy was trained on."""
    from aow_sim.control.general_spec import vel_filter_alpha, vel_filter_step

    for hz in (50, 200, 1000):
        v = np.zeros(2)
        a = vel_filter_alpha(1.0 / hz, 1.0)
        for _ in range(hz):                    # exactly one time constant
            v = vel_filter_step(v, [1.0, 0.0], a)
        assert v[0] == pytest.approx(1 - 1 / np.e, abs=1e-4)
    # a non-positive window is EXACTLY 1.0, not approximately: that is what
    # makes the un-windowed path bit-for-bit identical
    assert vel_filter_alpha(0.02, 0.0) == 1.0
    assert vel_filter_alpha(0.02, -1.0) == 1.0


def test_base_layout_is_a_prefix_of_every_optional_layout():
    """Every positional index into the base observation stays valid whatever
    optional blocks are on. analysis/mirror_equivariance.py slices FLIP_OBS on
    exactly this."""
    from aow_sim.control.general_spec import (OBS_MIRROR_PARITY, OBS_NAMES,
                                              obs_dim_for, obs_layout)
    base = _obs()
    assert base.shape == (OBS_DIM,)
    for kw, flags in ((dict(v_bar=(0.7, -0.04)), (1.0, False, False)),
                      (dict(pitch=(0.1, 0.4)), (0.0, True, False)),
                      (dict(wings=(1.2, 0.3)), (0.0, False, True)),
                      (dict(v_bar=(0.7, -0.04), pitch=(0.1, 0.4)),
                       (1.0, True, False)),
                      (dict(v_bar=(0.7, -0.04), pitch=(0.1, 0.4),
                            wings=(1.2, 0.3)), (1.0, True, True))):
        o = _obs(**kw)
        assert o.shape == (obs_dim_for(*flags),)
        assert o[:OBS_DIM] == pytest.approx(base, abs=0.0)
        assert obs_layout(*flags)[:OBS_DIM] == OBS_NAMES[:OBS_DIM]
    assert len(OBS_NAMES) == len(OBS_MIRROR_PARITY)
    # A filter is linear and time-invariant, so a filtered quantity mirrors
    # exactly like its source. Getting this wrong does not raise -- it
    # silently corrupts every handedness number.
    i = OBS_NAMES.index
    assert OBS_MIRROR_PARITY[i("v_bar_lon")] == OBS_MIRROR_PARITY[i("v_lon")]
    assert OBS_MIRROR_PARITY[i("v_bar_lat")] == OBS_MIRROR_PARITY[i("v_lat")]
    # Pitch is a SAGITTAL quantity: the mirror leaves it alone, unlike roll.
    assert OBS_MIRROR_PARITY[i("pitch")] == +1
    assert OBS_MIRROR_PARITY[i("pitch_rate")] == +1
    # Wings are +1 for a DIFFERENT reason: one actuator drives the pair
    # through an equality with mirror = -1, so left = -right and every
    # reachable wing state is symmetric. Only true while they are coupled.
    assert OBS_MIRROR_PARITY[i("wing_angle")] == +1
    assert OBS_MIRROR_PARITY[i("wing_rate")] == +1


def test_observation_width_is_ambiguous_so_layout_is_the_contract():
    """A velocity-windowed policy and a pitch-observing one are BOTH 17 wide
    with different meanings in slots 15-16. A width check would load either as
    the other and feed the net nonsense with nothing raised, so replay
    compares the layout element-wise."""
    from aow_sim.control.general_spec import obs_dim_for, obs_layout
    # THREE distinct 17-wide layouts now, which is the whole point.
    assert (obs_dim_for(1.0, False, False) == obs_dim_for(0.0, True, False)
            == obs_dim_for(0.0, False, True) == 17)
    assert len({obs_layout(1.0, False, False), obs_layout(0.0, True, False),
                obs_layout(0.0, False, True)}) == 3


def test_pitch_sign_is_nose_up():
    """extract_state negates the textbook ZYX pitch so +ve means nose up.
    Reported raw, `max(pitch)` picks the nose-DOWN tail and a 23 deg wheelie
    reads as 0.4 deg -- which is exactly what it did."""
    mujoco = pytest.importorskip("mujoco")
    from aow_sim.control.balance import extract_state

    class _D:
        qpos = np.zeros(7)
        qvel = np.zeros(6)
    d = _D()
    d.qpos = np.zeros(7)
    ang = np.deg2rad(20.0)                    # nose up by 20 deg
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array([0.0, -1.0, 0.0]), ang)
    d.qpos[3:7] = q
    d.qvel = np.zeros(6)
    d.qvel[4] = -1.0                          # nose rising
    s = extract_state(d, np.zeros(2))
    assert np.degrees(s.pitch) == pytest.approx(20.0, abs=0.5)
    assert s.pitch_rate > 0


def test_vel_window_zero_reproduces_the_instantaneous_reward():
    """The back-compat contract: a config without the field, or with 0.0, is
    the pre-window env exactly -- same width, same reward argument."""
    env = _env(vel_window_s=0.0)
    assert env.obs_dim == OBS_DIM
    obs, _ = env.reset(seed=1)
    rng = np.random.default_rng(0)
    for _ in range(40):
        obs, _r, term, trunc, info = env.step(rng.uniform(-1, 1, 3))
        # v_bar IS v, so the windowed and instantaneous errors coincide
        assert info["vel_err_win"] == pytest.approx(info["vel_err"], abs=0.0)
        if term or trunc:
            break


def test_windowed_reward_admits_an_oscillating_gait():
    """THE point of the whole change.

    A wriggle produces net lateral motion by oscillating fore/aft, so its
    instantaneous speed is much larger than the commanded speed. Scored
    instant by instant it loses to standing still; scored on its time average
    it does not. Pure arithmetic on the filter so it cannot go flaky."""
    from aow_sim.control.general_spec import vel_filter_alpha, vel_filter_step

    dt, sigma, cmd_lat = 0.02, 0.35, 0.144
    t = np.arange(0, 8.0, dt)
    # net lateral drift at exactly the commanded speed, carried by a 2 Hz
    # fore/aft oscillation of +-0.45 m/s
    v = np.stack([0.45 * np.sin(2 * np.pi * 2.0 * t),
                  np.full_like(t, cmd_lat)], axis=1)

    def mean_r(window):
        a = vel_filter_alpha(dt, window)
        vb = v[0].copy()
        out = []
        for k in range(len(t)):
            vb = vel_filter_step(vb, v[k], a)
            e2 = (0.0 - vb[0]) ** 2 + (cmd_lat - vb[1]) ** 2
            out.append(np.exp(-e2 / sigma ** 2))
        return float(np.mean(out[len(t) // 2:]))       # after the flush

    idle = np.exp(-(cmd_lat ** 2) / sigma ** 2)        # ignore the command
    assert mean_r(0.0) < idle, "instantaneous reward should punish the gait"
    assert mean_r(1.0) > idle, "windowed reward should admit it"
    assert mean_r(1.0) > 0.9


def test_velocity_filter_survives_a_command_resample():
    """The filter is a property of the bike's motion, not of the command.
    Resetting it on resample would be a hidden state jump the policy cannot
    see, and hardware has no resample event to match."""
    env = _env(vel_window_s=1.0, resample_s=[0.06, 0.08])
    env.reset(seed=3)
    env.set_difficulty(1.0)
    prev_cmd = env._v_cmd_w.copy()
    jumped = False
    for _ in range(env.max_steps):
        before = env._v_bar_w.copy()
        _o, _r, term, trunc, _i = env.step(np.zeros(3))
        if not np.allclose(env._v_cmd_w, prev_cmd):
            jumped = True
            # one filter tick of movement, not a reset to zero or to v
            assert np.linalg.norm(env._v_bar_w - before) < 0.05
            prev_cmd = env._v_cmd_w.copy()
        if term or trunc:
            break
    assert jumped, "command never resampled"


def test_shipped_general_policy_matches_its_declared_width():
    """Fails rather than skips. Every other general-policy test skips on an
    obs-dim mismatch, so after a spec change the whole suite would go green by
    absence -- this is the one that goes red instead."""
    from aow_sim.control.flick import MOVES_DIR, load_move
    from aow_sim.control.general_spec import obs_layout_for
    names = sorted(p.stem for p in MOVES_DIR.glob("general_*.yaml"))
    if not names:
        pytest.skip("no general policies exported yet")
    for name in names:
        pol = load_move(name)
        want = obs_layout_for(pol)
        assert pol.obs_dim == len(want), (
            f"moves/{name}: obs_dim {pol.obs_dim} but its declared flags "
            f"imply {len(want)}: {want}")
        declared = tuple(getattr(pol, "obs_layout", ()) or ())
        assert not declared or declared == want, (
            f"moves/{name}: declared layout disagrees with its own flags")


def test_wing_channel_is_optional_and_appended():
    """ACT_DIM stays the SHARED move contract at 3; the wing channel is a
    general-policy-only fourth entry. scale_action's return arity follows its
    input arity, so every move policy still unpacks three."""
    from aow_sim.control.general_spec import ACT_DIM, act_dim_for
    from aow_sim.control.flick_spec import ActionBounds, scale_action

    assert ACT_DIM == 3
    assert act_dim_for(False) == 3 and act_dim_for(True) == 4
    b = ActionBounds(8.0, 1.4, 40.0)
    assert b.wing_rate_max == 0.0            # defaulted: moves are unchanged
    assert len(scale_action([0.5, 0.5], b)) == 3
    assert len(scale_action([1.0, -1.0, 0.5], b)) == 3
    bw = ActionBounds(8.0, 1.4, 40.0, 4.0)
    assert scale_action([1.0, -1.0, 0.5, 0.25], bw) == pytest.approx(
        (8.0, -1.4, 20.0, 1.0))
    # a bounds list written before the wing channel existed still loads
    assert ActionBounds.from_list([8.0, 1.4, 40.0]).wing_rate_max == 0.0
    assert ActionBounds.from_list([8.0, 1.4, 40.0, 4.0]).wing_rate_max == 4.0


def test_wing_command_is_clipped_to_the_no_lift_cap():
    """Past ~96 deg the deployed foot goes under the floor and jacks the bike
    off its wheels. The integrator must clamp at the cap and never below 0."""
    env = _env(obs_wings=True, act_wings=True, wing_max_deg=90.0,
               action_bounds=dict(steer_rate_max=8.0, hub_max=1.4,
                                  diff_max=40.0, wing_rate_max=4.0))
    assert env.action_space.shape == (4,)
    env.reset(seed=1)
    a = np.zeros(4)
    a[3] = 1.0                                # drive it hard open
    for _ in range(80):
        _o, _r, term, trunc, info = env.step(a)
        assert info["wing_deg"] <= 90.0 + 1e-6
        if term or trunc:
            break
    assert info["wing_deg"] == pytest.approx(90.0, abs=1e-6)
    a[3] = -1.0                               # ...and hard shut
    for _ in range(80):
        _o, _r, term, trunc, info = env.step(a)
        assert info["wing_deg"] >= -1e-9
        if term or trunc:
            break


def test_wings_off_builds_the_wingless_model():
    """The gating guarantee: without the flags this is today's robot, with no
    wing actuator and a 3-channel action."""
    env = _env()
    assert env.obs_dim == OBS_DIM and env.action_space.shape == (3,)
    assert not env.wings and env.model.nu == 3
    assert all("wing" not in env.model.joint(i).name
               for i in range(env.model.njnt))
