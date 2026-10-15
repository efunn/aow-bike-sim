"""The general policy's eval: the command grid, the per-episode metrics and
their aggregates, and the eval config. Moved out of `train_general_rl` on
2026-10-14 so the metrics can be imported without torch or SB3 -- the GPU
env's eval (`analysis/gpu/phase2/eval_grid.py`), the analysis scripts and
the tests need the definitions, not the trainer. `train_general_rl`
re-exports every name, so `from aow_sim.train_general_rl import
_eval_episodes` still works; new code imports from here.
"""
from __future__ import annotations

import numpy as np

from .control.balance import extract_state

# Deterministic command grid for every eval path, as (v_lon, v_lat, heading
# step [deg]) with the velocity fractions relative to env.v_max. Mirrors the
# analytic controller's own benchmarks (straight sprint, reverse, 90/180 deg
# turns at speed) so the two are directly comparable — see the Stage-2 gate.
_EVAL_CMDS = [
    (0.0, 0.0, 0.0),      # hold station
    (0.0, 0.0, 90.0),     # standstill quarter turn
    (0.0, 0.0, 180.0),    # standstill about-face
    (0.67, 0.0, 0.0),     # straight sprint (~0.8 m/s at v_max 1.2)
    (0.67, 0.0, 90.0),    # turn at speed
    (0.67, 0.0, 180.0),   # U-turn at speed
    (1.0, 0.0, 0.0),      # top speed straight
    (-0.42, 0.0, 0.0),    # reverse (~-0.5 m/s)
    (-0.42, 0.0, 90.0),   # reverse turn
    (0.0, 0.33, 0.0),     # pure lateral crab (rear omni)
    (0.5, 0.0, 45.0),
    (0.5, 0.33, 180.0),   # glide + about-face = the pivot move
    # Directional large turns. The +-180 rows above CANNOT measure handedness:
    # command_to_body uses wrap_pi(psi_cmd - psi), so +180 and -180 are the
    # same observation and which way the bike spins is decided by yaw noise.
    # 170 is far enough round to be an about-face and still unambiguous.
    (0.0, 0.0, 170.0),
    (0.0, 0.0, -170.0),
]


def _mirror(c: tuple[float, float, float]) -> tuple[float, float, float]:
    """Left-right reflection of a command: flip lateral velocity and heading
    step. The plant is mirror-symmetric (axle_cant_deg 0, cone pairs mirrored
    about the wheel mid-plane), so the reflected command is exactly as
    achievable as the original -- any difference in score is the policy's own
    handedness."""
    v_lon, v_lat, dpsi = c
    # `+ 0.0` normalizes the -0.0 that negating 0.0 produces, and a mirrored
    # +-180 stays +180 (it wraps to the same target either way) so the command
    # labels don't imply a direction the observation cannot express.
    dpsi = 180.0 if abs(dpsi) == 180.0 else -dpsi + 0.0
    return (v_lon, -v_lat + 0.0, dpsi)


def _is_self_mirror(c: tuple[float, float, float]) -> bool:
    """True when reflecting `c` yields the same commanded behaviour, so adding
    the mirror would only duplicate an episode. Straight-ahead commands
    (v_lat 0, dpsi 0) are self-mirror, and so is a pure +-180 heading step --
    psi_err wraps, making +180 and -180 identical."""
    _, v_lat, dpsi = c
    return v_lat == 0.0 and (dpsi == 0.0 or abs(dpsi) == 180.0)


def _mirrored_grid() -> list[tuple[float, float, float]]:
    """`_EVAL_CMDS` plus the reflection of every command that has a distinct
    one. Without this the grid only ever turns right and only ever crabs left,
    so a one-handed policy scores flawlessly and snapshot selection cannot see
    it."""
    out = list(_EVAL_CMDS)
    for c in _EVAL_CMDS:
        if not _is_self_mirror(c):
            m = _mirror(c)
            if m not in out:
                out.append(m)
    return out


def eval_cmds(v_max: float) -> list[tuple[float, float, float]]:
    """Absolute (v_lon, v_lat, heading-step rad) commands for the eval grid,
    scaled to the configured envelope."""
    return [(round(a * v_max, 3), round(b * v_max, 3), np.deg2rad(d))
            for a, b, d in _mirrored_grid()]


# The command families the per-family metrics block is reported over. They
# must PARTITION the grid -- every command in exactly one, n's summing to
# n_eval -- and that is asserted, not assumed: in the first draft `spin` did
# not exist and the in-place +-90 commands fell through every predicate,
# invisible. That is the family where the policies differ most.
#
# The predicate argument is row["cmd"] = (v_lon, v_lat, dpsi_DEGREES). Order
# of the keys is display order; the predicates are mutually exclusive on their
# own, so nothing depends on evaluation order.
#
# WHY THESE FIVE. A family earns its place by separating policies that the
# neighbouring families do not: turn_big against everything (65.7 deg against
# 5.4 for general_rl_odo_ahrs), crab against cruise (3x on velocity error),
# spin against moving turns (52 deg against ~2 on the same 90 deg turn for
# general_rl_odo_ahrs_rand2), and hold because it is the only place drift is
# defined at all. Straight-line and moving-turn commands are NOT separated:
# they measured 6.8 against 5.4 deg and 0.124 against 0.147 m/s across four
# policies, which is no separation.
FAMILIES = {
    "hold":     lambda c: c[0] == 0 and c[1] == 0 and abs(c[2]) <= 1,
    "spin":     lambda c: c[0] == 0 and c[1] == 0 and 1 < abs(c[2]) < 170,
    "cruise":   lambda c: abs(c[0]) > 0.1 and abs(c[1]) < 0.1 and abs(c[2]) < 170,
    "crab":     lambda c: abs(c[1]) > 0.1 and abs(c[2]) < 170,
    "turn_big": lambda c: abs(c[2]) >= 170,
}


def _by_family(rows: list[dict]) -> dict:
    """Per-family metrics. THE POINT IS THAT THE WHOLE-GRID MEDIAN IS BLIND.

    A median over 20 commands cannot see a failure affecting fewer than 10 of
    them, because the middle two never reach the tail of the sorted list. The
    grid holds exactly 6 large turns, so no whole-grid median can EVER report
    a large-turn failure -- and one did not: `general_rl_odo_ahrs_rand2` has
    the BEST whole-grid heading median of four policies (7.4 deg) and the
    second WORST on large turns (152.1). See docs/plans/eval-score-rewrite.md
    section F.

    Survival is a RATE here, not a count: the families are sizes 1, 2, 9, 2
    and 6, so a count would mean a 6x different thing in `turn_big` than in
    `crab`. It reuses the top-level key name rather than inventing
    `fell_rate`, which makes the aggregate a checkable identity -- asserted
    below.

    READ `survive_rate` WITH `t_head_s`, NEVER ALONE. A high family survival
    means competence OR refusal, and only the arrival time separates them: on
    `turn_big`, `odo_ahrs` attempts the turn, arrives in 6.73 s and falls on 1
    of 6, while `rand2` never turns at all -- it drives BACKWARDS at
    ~0.8 m/s, which satisfies the world-frame velocity command exactly while
    abandoning the heading, and therefore never falls. Survival bought by refusal is the pattern
    `track_geo`'s geometric mean was introduced to defeat at the grid level,
    reappearing one level down.
    """
    out, covered, survivors = {}, 0, 0
    for name, pred in FAMILIES.items():
        sel = [r for r in rows if pred(r["cmd"])]
        if not sel:
            continue
        covered += len(sel)
        survivors += sum(not r["fell"] for r in sel)
        fam = {
            "n": len(sel),
            "survive_rate": round(sum(not r["fell"] for r in sel) / len(sel), 3),
        }
        # Time-to-heading only exists where a heading change was ASKED FOR.
        # In `cruise` that is a subset of the family (the moving turns), so
        # this cell has a different denominator from its row -- deliberate,
        # and the alternative is a sixth family with n=2.
        turners = [r for r in sel if abs(r["cmd"][2]) > 1]
        if turners:
            fam["t_head_s"] = _med([r["t_head_s"] for r in turners], 2)
            fam["t_head_n"] = len(turners)
        fam["head_err_tail"] = _med([r["head_err_tail"] for r in sel], 1)
        fam["vel_err_med"] = _med([r["vel_err_med"] for r in sel], 3)
        if name == "hold":
            # Drift is only meaningful against a stationary command, and the
            # grid holds exactly one -- so these are ONE episode, not a
            # median. `drift_overshoot` replaces the `drift_max` and
            # `drift_sd` this block briefly carried: max equalled the final
            # value exactly in 3 of 4 policies (drift grows monotonically),
            # and sd sat at 0.267-0.315 x final in all four, the ratio for a
            # linear ramp. Peak-minus-final is the same information with a
            # null value of zero, so a non-zero reading MEANS something:
            # the bike wandered out and came back rather than leaving.
            fam["drift_m"] = round(float(sel[0]["drift_m"]), 3)
            fam["drift_overshoot"] = round(
                float(sel[0].get("drift_max", sel[0]["drift_m"])
                      - sel[0]["drift_m"]), 3)
        out[name] = fam

    # The two invariants, ON INTEGER COUNTS. Checking the stored
    # `survive_rate` instead is a false positive waiting to happen: it is
    # rounded to 3 dp, so turn_big's 5/6 -> 0.833 and the weighted sum lands
    # at 0.9499 against a grid 0.9500. Counts do not round.
    if covered != len(rows):
        raise AssertionError(
            f"command families cover {covered} of {len(rows)} commands -- one "
            f"falls through every predicate in FAMILIES and is invisible")
    total = sum(not r["fell"] for r in rows)
    if survivors != total:
        raise AssertionError(
            f"families count {survivors} surviving episodes against {total} "
            f"in the grid -- a predicate is double-counting")
    return out


def _med(values, ndigits):
    """Median of a possibly-empty selection. NaN rather than an exception, so
    a caller running a reduced grid gets a blank cell instead of a crash."""
    return round(float(np.median(values)), ndigits) if values else float("nan")


def _score(m: dict) -> float:
    """Snapshot-selection score (higher is better): survival x tracking.

    An always-on controller has no task success, so `survive_rate` is the
    fraction of eval episodes that did not fall, and the tracking term is
    the env's own bounded [0,1] reward average. Multiplying rather than
    adding is deliberate: a policy that tracks beautifully for 2 s and then
    falls must not outrank one that survives the whole episode.

    The tracking term is the GEOMETRIC mean over commands, not the arithmetic
    one: identical when every command scores alike, but a command the policy
    has given up on (track -> 0) drags the whole score down instead of being
    averaged away. The arithmetic mean is what let a policy that refuses to
    reverse outrank one that reverses -- it abandoned 2 of 12 commands and
    banked the survival it bought. Falls back to `track` for metrics dicts
    written before track_geo existed."""
    return m["survive_rate"] * m.get("track_geo", m["track"])


_TAIL_S = 2.0        # steady-state window at the end of an episode [s]
_HEAD_TOL_DEG = 10.0  # "the turn is done" threshold for turn timing

# Eval episodes are SHORTER than training ones, and the reason is the discount
# horizon, not cost. `gamma: 0.99` at 50 Hz is 1/(1-g) = 100 steps = 2.0 s of
# lookahead, and no training command outlives `resample_s`'s ceiling -- so a
# 15 s eval held one command for ~7 horizons of a regime training never
# presents, and `_TAIL_S` read the LAST 2 s of exactly that. At 5 s the tail
# window lands at 3-5 s, inside the training distribution.
#
# NOT COMPARABLE ACROSS THE CHANGE: every `eval_*` number recorded in an
# existing moves/*.yaml was measured over 15 s. Re-run rather than compare.
# `env.eval_episode_s` overrides, so 15.0 restores the old measurement.
_EVAL_EPISODE_S = 5.0


def _eval_cfg(cfg: dict) -> dict:
    """The training config as the EVAL env wants it.

    Two changes, in one place because three call sites want both and a fourth
    will: randomization off (so repeating a command adds nothing), and the
    shorter episode above. The training `max_episode_s` is deliberately NOT
    the eval's -- they answer different questions, and sharing the key made
    the eval silently inherit whatever the curriculum wanted.
    """
    env = {**cfg["env"],
           "max_episode_s": float(cfg["env"].get("eval_episode_s",
                                                 _EVAL_EPISODE_S)),
           # NO BALL. `ball_prob` is contact-robustness DR for training, and
           # it had been leaking into the eval: a quarter of episodes had a
           # ball parked within `ball_place_radius`, deterministic per command
           # (the seed is `10_000 + k`) but arbitrary as to WHICH commands got
           # one. So five of twenty rows were scored against an obstacle the
           # metrics do not mention -- visible only when one rolled through
           # the `spin +170` panel of an eval video.
           "ball_prob": 0.0}
    return {**cfg, "env": env,
            "randomization": {**cfg["randomization"], "enabled": False}}


def _eval_episodes(env, act_fn, cmds):
    """One episode per command point; per-command rows plus aggregates. The
    eval env has randomization disabled, so repeating a command adds nothing.

    Behavioural quantities (drift, steer-at-rest, achieved speed, time to
    heading) are read straight off the env rather than the info dict, so this
    stays a pure measurement change -- general_env.py is untouched and no
    trained policy can be affected by it. Each eval episode holds ONE command
    for its whole duration (reset sets `_next_resample` to infinity), so
    per-episode aggregates are well defined.
    """
    n = len(cmds)
    tail = max(1, int(_TAIL_S / env.ctrl_dt))
    rows = []
    for k, (v_lon, v_lat, dpsi) in enumerate(cmds):
        obs, _ = env.reset(seed=10_000 + k, options={
            "v_cmd": (v_lon, v_lat), "psi_cmd_rel": dpsi, "difficulty": 1.0})
        # A policy with memory starts every episode empty. `act_fn.reset` is
        # set by sb3_actor and by the replay paths below; a plain lambda
        # around an MLP needs none.
        if hasattr(act_fn, "reset"):
            act_fn.reset()
        done, info = False, {}
        steps = 0
        t_head = None                      # first step inside _HEAD_TOL_DEG
        v_lons, v_lats, steers = [], [], []  # tail windows, refilled as we go
        xys, oscs, wings = [], [], []
        # WHOLE-EPISODE series, deliberately NOT truncated to the tail like
        # the windows above. The `head_err_deg` / `vel_err` / `drift_m` a row
        # has always carried are the values at the LAST STEP -- one sample --
        # so a 20-command mean of them is a mean of twenty single samples, and
        # one command the policy abandoned moves it by ~9 deg. These are what
        # let the same rows report a median and a worst case instead. 15 s at
        # 50 Hz is 750 floats a series; the cost is nothing.
        heads, vels, drifts = [], [], []
        # Static cap, not the curriculum-scheduled one: `wing_duty` has to mean
        # the same thing at every difficulty or the trace is not comparable
        # across a run.
        wing_cap_deg = float(np.degrees(getattr(env, "wing_max", 0.0)))
        # World-frame axis the LATERAL command points along, fixed for the
        # episode because psi_cmd is.
        lat_axis = np.array([-np.sin(env._psi_cmd), np.cos(env._psi_cmd)])
        lon_axis = np.array([np.cos(env._psi_cmd), np.sin(env._psi_cmd)])
        while not done:
            obs, _r, term, trunc, info = env.step(act_fn(obs))
            done = term or trunc
            steps += 1
            if t_head is None and info.get("head_err_deg", 180.0) < _HEAD_TOL_DEG:
                t_head = steps
            s = extract_state(env.data, env._p0)
            v_lons.append(float(s.v_lon))
            v_lats.append(float(s.v_lat))
            steers.append(abs(float(env.data.qpos[env._sj])))
            xys.append(env.data.qpos[:2].copy())
            # How much of the motion is oscillation the window averages away.
            # Distinguishes a real wriggle from a smooth crab; 0 when the
            # policy has no velocity window.
            oscs.append(float(np.hypot(s.v_lon - info.get("v_bar_lon", s.v_lon),
                                       s.v_lat - info.get("v_bar_lat", s.v_lat))))
            wings.append(float(info.get("wing_deg", 0.0)))
            heads.append(float(info.get("head_err_deg", 180.0)))
            vels.append(float(info.get("vel_err", 9.9)))
            drifts.append(float(np.hypot(s.e_lon, s.e_lat)))
            del v_lons[:-tail], v_lats[:-tail], steers[:-tail]
            del xys[:-tail], oscs[:-tail], wings[:-tail]
        s = extract_state(env.data, env._p0)
        # NET lateral speed: world displacement over the tail projected on the
        # commanded lateral axis. NOT the mean of body-frame v_lat -- under a
        # wriggle the body frame rotates within the window, so that mean is
        # not a displacement rate and reads ~0 for a gait that is crabbing
        # correctly. This is what crab_ratio_* is scored from.
        v_lat_net = 0.0
        if len(xys) >= 2:
            v_lat_net = float((xys[-1] - xys[0]) @ lat_axis
                              / ((len(xys) - 1) * env.ctrl_dt))
        fell = bool(info.get("fell", True))
        rows.append({
            "cmd": (round(v_lon, 3), round(v_lat, 3), round(np.degrees(dpsi))),
            "track": float(info.get("track", 0.0)),
            "fell": fell,
            # A fall BY A WING on the floor (wing_touch_fails), not by roll.
            "wing_touch": bool(info.get("wing_touch", False)),
            "vel_err": float(info.get("vel_err", 9.9)),
            "head_err_deg": float(info.get("head_err_deg", 180.0)),
            "drift_m": float(np.hypot(s.e_lon, s.e_lat)),
            # WHERE it drifted to, not just how far. `drift_m` is a hypot, so
            # it throws away the one thing that says WHICH FAILURE this is: a
            # bike backing out from under itself and one sliding sideways read
            # identically. `general_rl_cmd_curriculum` holds at 0.93 m and
            # `..._pitch_w` at 0.18 m, and the magnitudes alone do not say that
            # the first is almost pure reverse.
            #
            # Resolved in the COMMANDED-heading frame, which is fixed for the
            # episode -- NOT `s.e_lon` / `s.e_lat`, which are in the live
            # bike-yaw frame. Under a hold the bike is supposed to keep its
            # heading, but a decomposition that rotates with the thing it is
            # measuring cannot report the case where it does not. Same axis
            # convention as `v_lat_net` above: +lon along the command,
            # +lat to its left.
            "drift_lon": float((env.data.qpos[:2] - env._p0) @ lon_axis),
            "drift_lat": float((env.data.qpos[:2] - env._p0) @ lat_axis),
            # The same three quantities over the WHOLE episode. The keys
            # above stay last-step so every metrics block already written
            # into moves/*.yaml remains comparable -- same convention as
            # `vel_err` vs `vel_err_win` in general_env.
            "head_err_med": float(np.median(heads)) if heads else 180.0,
            "vel_err_med": float(np.median(vels)) if vels else 9.9,
            # WORST EXCURSION ONCE SETTLED, over the same _TAIL_S window the
            # speed and steer numbers use. NOT a whole-episode max: both
            # commands are STEPS applied at t=0, so head_err starts at |dpsi|
            # (180 on the 180 commands) and vel_err starts at |v_cmd| (up to
            # 1.2) no matter how good the policy is. A whole-episode max
            # measures the command, not the response -- measured at 180.0 for
            # every policy in the set, which is what retired it.
            "head_err_tail": float(np.max(heads[-tail:])) if heads else 180.0,
            "vel_err_tail": float(np.max(vels[-tail:])) if vels else 9.9,
            "drift_max": float(np.max(drifts)) if drifts else 0.0,
            # SD about the episode's own mean. Read WITH drift_max/drift_m:
            # a bike rocking back and forth has drift_max well above its
            # final drift, while one leaving steadily has the two nearly
            # equal, and the SD says how big the excursion is either way.
            "drift_sd": float(np.std(drifts)) if drifts else 0.0,
            "steer_deg": float(np.degrees(np.mean(steers))) if steers else 0.0,
            "v_ach": float(np.mean(v_lons)) if v_lons else 0.0,
            # Achieved LATERAL speed. Without it a crab command is scored only
            # through v_ach (longitudinal), which reads ~0 whether the bike
            # crabbed properly, yawed away and slid, or simply sat there.
            "v_lat_ach": float(np.mean(v_lats)) if v_lats else 0.0,
            # ...and the displacement-based version that crab_ratio_* uses.
            "v_lat_net": v_lat_net,
            "v_osc_rms": float(np.sqrt(np.mean(np.square(oscs)))) if oscs else 0.0,
            # Wings. `wing_duty` is the fraction of the tail spent deployed
            # far enough to be near the floor -- READ THIS BEFORE THE SCORE on
            # a wings arm. A policy that discovers "deploy and lean" scores
            # well while quietly turning a righting mechanism into landing
            # gear, and the score alone cannot tell you that happened.
            # MAGNITUDE, not the signed mean. On the mirrored pair the wing
            # command is 0..cap so the two are identical -- this is a no-op for
            # every existing arm. On the CO-ROTATING pair (act_swing) the
            # command is signed, -cap..+cap, and a policy that deploys left as
            # often as right averages to ~0: indistinguishable from never
            # touching the wings, which is the one thing this metric exists to
            # detect.
            "wing_deg": float(np.mean(np.abs(wings))) if wings else 0.0,
            # Threshold SCALED to the cap rather than hardcoded. 60 of a 90 deg
            # stroke is two thirds, and two thirds is the thing meant ("far
            # enough to be near the floor"); the literal 60 silently becomes
            # unreachable on a mechanism capped at 45, pinning duty at 0.000
            # whatever the policy does.
            "wing_duty": (float(np.mean(np.abs(wings) > (2 / 3) * wing_cap_deg))
                          if wings and wing_cap_deg > 0 else 0.0),
            # A turn that never got inside tolerance is censored at the episode
            # length, not dropped -- "never finished" must cost more than slow.
            "t_head_s": (t_head if t_head is not None else steps) * env.ctrl_dt,
            "steps": steps,
        })

    tracks = [r["track"] for r in rows]
    m = {"survive_rate": sum(not r["fell"] for r in rows) / n,
         # Of the falls, how many were a wing touching down.
         "wing_touch_rate": sum(r.get("wing_touch", False) for r in rows) / n,
         "track": round(float(np.mean(tracks)), 3),
         # Geometric mean: equals the arithmetic mean when commands score
         # alike, but any ABANDONED command (track -> 0) drags it down. The
         # arithmetic mean let a policy that refuses to reverse hide behind
         # ten other commands it does well.
         "track_geo": round(float(np.exp(np.mean(np.log(
             np.clip(tracks, 1e-3, 1.0))))), 3),
         "vel_err": round(float(np.mean([r["vel_err"] for r in rows])), 3),
         "head_err_deg": round(float(np.mean([r["head_err_deg"] for r in rows])), 1),
         "n_eval": n}
    m.update(_behaviour_metrics(rows))
    # PER-FAMILY, beside the whole-grid scalars rather than in place of them:
    # `_score` reads survive_rate and track_geo, and re-basing it is a
    # separate and currently blocked decision. Everything above is unchanged,
    # so every metrics block already written into moves/*.yaml stays
    # comparable.
    m["by_family"] = _by_family(rows)

    return m, rows


def _behaviour_metrics(rows: list[dict]) -> dict:
    """Diagnostic behaviour summary -- reported and logged, deliberately NOT
    in `_score`. Drift and steer-at-rest are free in the *reward* (the action
    is steer rate, so a cocked wheel costs nothing to hold), so making
    selection chase them without fixing the reward would just relocate the
    underdetermination rather than remove it."""
    def by(pred):
        return [r for r in rows if pred(r["cmd"])]

    hold = by(lambda c: c[0] == 0 and c[1] == 0 and c[2] == 0)
    # DRIVE DIRECTION IS ONLY MEASURABLE WHERE NO TURN IS COMMANDED. A command
    # is a world-frame velocity plus a heading, and the velocity half is
    # satisfied either body-forward (yaw = psi_cmd) or body-backward
    # (yaw = psi_cmd + 180) -- so `v_ach`'s sign is decided by which way the
    # policy TURNED, not by whether it will drive forward.
    #
    # Measured on `general_rl_cmd_curriculum2`, command (0.804, 0, -90): it
    # turned LEFT +87 deg and reversed at -0.697, putting its world velocity
    # within 3 deg of the command. That is a heading failure, and the old
    # selector scored it -1.009 as though the policy refused to drive forward.
    # Six of its nine forward rows read +0.79..+0.97 and three read
    # -0.98..-1.16, which cancelled to `speed_ratio_fwd` +0.027 -- a policy
    # that drives forward fine, reported as one that does not.
    #
    # `abs(c[2]) < 1` keeps only the straight-ahead commands, where forward is
    # the cheap answer and reversing costs a 180 deg heading error for no
    # saving. Handedness on the turning commands is `turn_asym`'s job, and the
    # +-180 group's is `by_family.turn_big`.
    fwd = by(lambda c: c[0] > 0.1 and abs(c[2]) < 1)
    rev = by(lambda c: c[0] < -0.1 and abs(c[2]) < 1)
    crab_l = by(lambda c: c[1] > 0.1)
    crab_r = by(lambda c: c[1] < -0.1)
    crab = crab_l + crab_r

    def ratio(sel):     # achieved / commanded longitudinal speed, SIGNED
        """Signed fraction of the commanded speed achieved.

        The lower clip was 0.0, which floored every wrong-direction policy at
        exactly 0.000 and threw away the one number that says WHAT it is doing
        instead. `general_rl_cmd_curriculum` read `speed_ratio_fwd` 0.000 at
        all twenty of its evals; the value underneath was about -0.4, i.e. it
        reverses at 40% of the speed it was told to go forward. "Achieves
        nothing" and "drives the other way at half throttle" are different
        failures with different fixes, and the floor made them identical.

        Still clipped, at +-1.5: a near-zero command in the denominator would
        otherwise let one row dominate the mean. Negative now means WRONG
        DIRECTION, which is why `policy_menu`'s `r > 0.5` test still reads
        correctly -- it prints the number beside REFUSES, and that number is
        now informative.
        """
        if not sel:
            return float("nan")
        return float(np.mean([np.clip(r["v_ach"] / r["cmd"][0], -1.5, 1.5)
                              for r in sel]))

    def crab_ratio(sel):  # achieved / commanded LATERAL speed, per side
        """Scored from `v_lat_net` (displacement), NOT `v_lat_ach` (the mean
        of instantaneous body-frame v_lat). The two agree while the bike holds
        heading, and diverge exactly where it matters: a wriggle gait rotates
        the body frame within the averaging window, so the mean of body-frame
        v_lat is not a displacement rate and reads ~0 for a policy that is in
        fact crabbing. `v_lat_ach` stays in the rows so older numbers remain
        readable."""
        if not sel:
            return float("nan")
        return float(np.mean([np.clip(r["v_lat_net"] / r["cmd"][1], 0.0, 1.5)
                              for r in sel]))

    # Handedness: pair each turning command with its reflection and compare
    # how long each took to arrive. Symmetric plant => a symmetric policy
    # scores 0 here; 1.0 means one side never arrived at all.
    times = {r["cmd"]: r["t_head_s"] for r in rows}
    asyms, seen = [], set()
    for c, t in times.items():
        # Only commands that actually demand a heading change, and only
        # those with a distinct mirror. A `c[2] > 0` filter looks like it
        # counts each pair once but does not: -180 normalizes to +180, so
        # BOTH members of a 180 pair pass it (double-counted) and a
        # self-mirror 180 pairs with itself, contributing a spurious 0 that
        # dilutes the average.
        mc = _mirror(c)
        key = frozenset((c, mc))
        if _is_self_mirror(c) or abs(c[2]) < 1e-9 or mc not in times or key in seen:
            continue
        seen.add(key)
        t2 = times[mc]
        if max(t, t2) > 0:
            asyms.append(abs(t - t2) / max(t, t2))
    return {
        "drift_m": round(float(np.mean([r["drift_m"] for r in hold])), 3)
                   if hold else float("nan"),
        # WHICH WAY it drifted, in the commanded-heading frame: +lon is the
        # direction the bike was pointed, +lat is to its left. `drift_m` is a
        # hypot and cannot distinguish backing out from sliding sideways --
        # two different failures with two different fixes. `drift_bearing_deg`
        # is the same information as one angle: 0 straight ahead, +-180
        # straight back, +90 to the left.
        "drift_lon": round(float(np.mean([r.get("drift_lon", 0.0)
                                          for r in hold])), 3)
                     if hold else float("nan"),
        "drift_lat": round(float(np.mean([r.get("drift_lat", 0.0)
                                          for r in hold])), 3)
                     if hold else float("nan"),
        "drift_bearing_deg": round(float(np.degrees(np.arctan2(
            np.mean([r.get("drift_lat", 0.0) for r in hold]),
            np.mean([r.get("drift_lon", 0.0) for r in hold])))), 1)
                             if hold else float("nan"),
        # Drift is measured on the HOLD command only, and the 20-command grid
        # contains exactly one -- so these are one episode, not twenty, and
        # the time series is the only thing that can say anything about
        # consistency. `drift_max` well above `drift_m` means it wandered out
        # and came back; the two nearly equal means it left and kept going.
        # .get, for the same reason the wing fields below use it: this is a
        # pure function and callers hand it minimal row dicts.
        "drift_max": round(float(np.mean([r.get("drift_max", r["drift_m"])
                                          for r in hold])), 3)
                     if hold else float("nan"),
        "drift_sd": round(float(np.mean([r.get("drift_sd", 0.0)
                                         for r in hold])), 3)
                    if hold else float("nan"),
        "steer_rest_deg": round(float(np.mean([r["steer_deg"] for r in hold])), 1)
                          if hold else float("nan"),
        "speed_ratio_fwd": round(ratio(fwd), 3),
        "speed_ratio_rev": round(ratio(rev), 3),
        # Crab, per side: a policy that only crabs toward its preferred
        # steering direction shows one good ratio and one near zero.
        "crab_ratio_left": round(crab_ratio(crab_l), 3),
        "crab_ratio_right": round(crab_ratio(crab_r), 3),
        # Heading held DURING a crab. A crab is the one command where holding
        # heading and reaching the commanded velocity are simultaneously
        # demanding, and sigma_psi_deg 25 is loose enough that yawing ~15 deg
        # toward the travel direction is a cheap substitute for crabbing.
        "crab_head_err": round(float(np.mean([r["head_err_deg"] for r in crab])), 1)
                         if crab else float("nan"),
        # RMS of the motion the velocity window averages away, on crab rows.
        # This is what separates the two ways a crab_ratio can rise: a genuine
        # wriggle gait (high crab_osc AND high crab_ratio) from a smooth crab
        # that got easier for some unrelated reason (crab_osc near zero).
        # Diagnostic only, deliberately not in _score.
        "crab_osc": round(float(np.mean([r["v_osc_rms"] for r in crab])), 3)
                    if crab else float("nan"),
        # Averaged over EVERY command, not just one family: the question is
        # whether the policy leans on the wings at all, anywhere.
        # .get, not [], because this averages over EVERY row and
        # _behaviour_metrics is a pure function callers hand minimal row dicts
        # to; a wingless run legitimately has no wing fields.
        "wing_deg": round(float(np.mean([r.get("wing_deg", 0.0)
                                         for r in rows])), 1),
        "wing_duty": round(float(np.mean([r.get("wing_duty", 0.0)
                                          for r in rows])), 3),
        "turn_asym": round(float(np.mean(asyms)), 3) if asyms else float("nan"),
    }


def _print_rows(rows: list[dict]) -> None:
    """Per-command table. The whole point: two abandoned commands are
    invisible in a mean, and obvious here."""
    # `dist` is displacement from the start pose: it is only "drift" on the
    # hold-station row (which is the only row the drift_m aggregate uses);
    # on a sprint row it is just how far the bike went.
    print(f"    {'v_lon':>6} {'v_lat':>6} {'dpsi':>5} | {'track':>6} {'v_ach':>6} "
          f"{'t_head':>6} {'dist':>6} {'steer':>6}  fell")
    for r in rows:
        v_lon, v_lat, dpsi = r["cmd"]
        print(f"    {v_lon:>6.2f} {v_lat:>6.2f} {dpsi:>5.0f} | {r['track']:>6.3f} "
              f"{r['v_ach']:>6.2f} {r['v_lat_ach']:>7.2f} "
              f"{r['head_err_deg']:>5.1f} {r['t_head_s']:>6.2f} "
              f"{r['drift_m']:>6.2f} {r['steer_deg']:>6.1f}  "
              f"{'X' if r['fell'] else ''}")
