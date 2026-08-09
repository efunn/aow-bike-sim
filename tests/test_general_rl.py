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
    assert obs[-3:] == pytest.approx([0.1, -0.2, 0.3], abs=1e-6)


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


def test_general_move_replays():
    """If a trained general policy exists, it loads and replays without NaN."""
    from aow_sim.control.flick import MOVES_DIR
    if not (MOVES_DIR / "general_rl.npz").exists():
        pytest.skip("run `python -m aow_sim.train_general_rl` first")
    from aow_sim.control.policy import load_policy_npz
    pol = load_policy_npz(MOVES_DIR / "general_rl.npz")
    if pol.obs_dim != OBS_DIM:
        pytest.skip("moves/general_rl predates the current obs spec — retrain")
    act = pol.action(_obs())
    assert len(act) == 3 and np.all(np.isfinite(act))


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
