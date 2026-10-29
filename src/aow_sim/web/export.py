"""Export a general policy and the plant it flies on for the browser demo.

    python -m aow_sim.web.export                  # control.general_move
    python -m aow_sim.web.export --policy NAME --trace traces/web_trace.json

Writes `site/assets/`:

  bike.mjb      the COMPILED model, as GeneralEnv holds it after construction:
                the drivetrain switched to the detailed plant and the gearbox
                friction attached. Binary, not XML -- an XML round trip of the
                same model lost ~1e-7 of precision on every float, re-processed
                the meshes, and dropped `eq_active0` / `tendon_limited`, so the
                rollers came back rigid. An .mjb is byte-exact, and the price is
                that it loads only in the SAME MuJoCo version: the page reads
                `mujoco_version` from config.json and loads exactly that build.
  policy.bin    the MinGRU weights, float32, laid out by config.json's manifest
  config.json   every constant the page's per-step code needs, read off the
                live Python objects (DrivetrainSim, GearboxFriction,
                SimOdometry) rather than re-derived from bike_params, so the two
                sides cannot disagree about a number.

THE PLANT IS THE POLICY'S OWN EVAL ENV with one sensor swapped: the AHRS error
model is off (`ahrs_level: none`), so attitude and heading are MuJoCo truth.
Felt in teleop (2026-10-22): no visible difference for general_puffer_fixes_72M.
Everything else it trained with stays, as its move yaml records it -- the XC430
firmware loop, the detent (sawtooth hill and kinetic friction where it has
them), roller slop, wheel play (joints in the compiled model, nothing per
step), steer gearbox friction, the onboard velocity estimate from quantised
encoder counts, and the steer rate from the servo's counts where it trained
on that. The firmware loop was the one that mattered by hand.

Three changes make it teleop's bike rather than the eval's, all for the demo:

  * the righting module is JOINTED (`righting_rigid: false`), with the crank
    on its current-based position servo model, as teleop flies it -- the
    policy trained on the welded module, and holds the crank at stow either
    way. This is what lets the page lie down, get up and swing a blade.
  * a ball (`hockey`), parked off-scene at reset as GeneralEnv parks it. Two
    edits to the build, here only: the ball-shot move's stick panels are
    left off (20 g each, on the chassis -- not the bike), and the ball gets
    condim 6 with rolling friction. At the sim's condim 4 MuJoCo ignores
    rolling friction entirely, and the ball rolled on for good: 53 m from
    4 m/s and still going at 20 s. It is also bigger, lighter and bouncier
    than the study's (BALL_RADIUS and the rest, below, with the sweep).
    `config/studies/ball.yaml` is unchanged.
  * lie down / get up as the bike runs them (`sit_config`), not teleop's.

`--trace` also runs the env for `--seconds` under a fixed command schedule
and records every step, for `check.mjs`.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import mujoco
import numpy as np

SITE = Path(__file__).resolve().parent / "site"
AHRS_LEVEL = "none"
# The page's ball: a light, bouncy play ball the bike can push around with a
# wheel or a blade without falling over (user, 2026-10-23: "like a beach ball
# or balloon"). Swept in the page's own sim the same day, 6 drives each into
# and past it (ahead at 0.6 and 1.0 m/s; a blade out at 91 deg, each side, at
# 0.3 and 0.6), 4 detent phases:
#
#     ball                          bike falls   ball pushed (median)
#     67 mm, 20 g, grippy           2 / 6        --   (one phase)
#     67 mm, 10 g, bouncy           1 / 6        --   (one phase)
#     120 mm, 10 g, bouncy          8 / 24       0.48 m
#     160 mm, 10 g, bouncy          0 / 24       0.30 m
#     160 mm, 10 g, softer          0 / 24       0.37 m   <- this
#
# The small ball's falls were the blade riding up onto it and propping the
# bike over -- a kickstand, whatever its mass, since the floor holds the ball
# up. A ball wider than the blade's reach meets it side-on instead. SIZE is
# what fixed it, not mass.
#
# STIFFNESS is what stops it jittering. Pushed by the front wheel at 0.5 m/s,
# a soft bouncy contact chatters between tyre and floor (same sweep):
#
#     contact (stiffness, damping)   rebound   pushed: hop span, airborne
#     2.5e3, 10                      0.62      21.5 mm, 14 % of the time
#     1e4,   20                      0.62       8.0 mm,  3 %
#     3e4,   70                      0.48       1.7 mm,  0 %   <- this
#     3e4,   110                     0.36       1.2 mm,  0 %
#
# Re-run with this contact: still 0 / 24 falls, ball pushed 0.32 m median.
# Rolls ~0.5 m from 1 m/s with the air drag. At rest it never moved (5 um).
#
# 2026-10-27, the user: a real pickleball's size, 74 mm, accepting the blade
# riding up onto it that the sweep above found at 67 mm. Not re-swept; the
# mass and contact stay the 160 mm ball's.
BALL_RADIUS = 0.037
BALL_MASS, BALL_SLIDING, BALL_ROLLING = 0.010, 0.4, 0.002
BALL_SOLREF = (-3e4, -70.0)     # direct form: stiffness, damping (mass-normalised)
BALL_DRAG_TAU = 0.8             # s; air drag, applied by the page
# GeneralEnv builds its ball scene only when it may place a ball. This
# probability never fires, so every reset parks it, as an eval reset does.
BALL_PROB_PARKED = 1e-12
# Teleop's shove (`.`) is 8 N for 0.35 s, sideways on the chassis. The page
# has two: a nudge the policy catches and a knockdown for the auto-up. Swept
# standing, both sides, 4 detent phases each (8 shoves a force):
#
#     plant                   caught                       over
#     fixes_72M (10-23)       0.5-1.5 N, peak roll 3-18    2 N and up, 8 / 8
#     a40_v4_80M (10-26)      0.25-0.9 N, peak roll 3-7    1.0 N 1 / 8, 1.25 N 7 / 8,
#                                                          1.5 N and up 8 / 8
#
# The v4 plant (sawtooth detent, wheel play) tips at about two thirds of the
# old force, so both keys came down (user, 10-26: "a bit less stable"): the
# nudge to 0.75 N, below the first fall; the knockdown to 2 N, 8 / 8 over.
SHOVE_N, NUDGE_N, SHOVE_S = 2.0, 0.75, 0.35
# Z / C swing the blades to this crank angle, one side or the other: 70 % of
# the stroke, from the mechanism sim (user, 2026-10-23). Teleop's 9 / 4 go to
# the full stroke, where the blade meets the floor.
BLADE_CRANK_DEG = 91.0
# Lie down / get up are the bike's (sit_config). Where the bike waits for
# the operator, the page goes on by itself: a get-up that stood the bike on
# a blade hands it to the policy after PROP_HOLD_S, and one that failed
# tries again after RETRY_S. The bike has no retry on purpose -- lifting a
# stalled crank again and again overloaded it -- and the sim's crank has no
# overload. PROP_HOLD_S is 0 because the SIM does not stay propped: its lift
# reaches the prop still rolling at ~44 deg/s, inside the hand-off gate, and
# with nobody balancing it coasts through upright onto the side with no
# blade (2026-10-26, a40_v4_80M: 6 of 6 with the drives limp, and the one
# run with them held). Handed over at once: 6 / 6 lie-downs up in 3.1 s, both sides; 20 /
# 20 knockdowns in 4.3-5.3 s, no lift failing. 0.8 s was the first value.
PROP_HOLD_S = 0.0
RETRY_S = 1.0
# Teleop's pelt (`--hockey`, Space): rolled in from this far off the flank.
# Teleop throws at 4 m/s; the page's lighter ball at 2.5.
PELT_SPEED, PELT_STANDOFF = 2.5, 0.45

# The trace's command schedule: (start s, forward m/s, heading change deg).
# Hold, drive, turn while driving, reverse, and turn in place -- one of each
# thing the page's keys can ask for.
SCHEDULE = ((0.0, 0.0, 0.0), (1.5, 0.6, 0.0), (4.0, 0.6, 60.0),
            (6.5, -0.4, 0.0), (9.0, 0.0, -90.0), (11.5, 0.0, 0.0))


def _web_build_model(params=None, variant="full", **kw):
    """`build_model` with the ball edits (module docstring): no stick panels,
    a ball that rolls to a stop. Identical to build_model without `hockey`."""
    from ..build_model import build_spec
    spec = build_spec(params, variant, **kw)
    if kw.get("hockey"):
        for name in ("stick_left", "stick_right"):
            spec.delete(spec.geom(name))
        ball = spec.geom("ball")
        ball.condim = 6
        if BALL_RADIUS is not None:
            ball.size = [BALL_RADIUS, 0.0, 0.0]
        if BALL_SOLREF is not None:
            # Direct form: stiffness and damping, mass-normalised. solmix
            # lets the ball's own pair win the mix with whatever it touches,
            # so this is the ball's contact and nothing else's.
            ball.solref = list(BALL_SOLREF)
            ball.solmix = 1000.0
        ball.mass = BALL_MASS
        ball.friction = [BALL_SLIDING, ball.friction[1], BALL_ROLLING]
    return spec.compile()


def build_env(name: str):
    """(env, pol): the policy's eval env, as per_command.py builds it, with
    the AHRS model off, the righting module jointed and a parked ball.
    Randomisation off."""
    from ..control import general_env
    from ..control.flick import load_move
    from ..control.general_spec import policy_env_overrides
    from ..params import load_params
    pol = load_move(name)
    pol.ahrs_level = AHRS_LEVEL
    cfg = general_env._load_rl_config()
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    cfg = {**cfg, "env": {**cfg["env"], **policy_env_overrides(pol),
                          "righting_rigid": False,
                          "ball_prob": BALL_PROB_PARKED}}
    real = general_env.build_model
    general_env.build_model = _web_build_model
    try:
        env = general_env.GeneralEnv(load_params(), cfg)
    finally:
        general_env.build_model = real
    _check_supported(env, pol)
    return env, pol


def _check_supported(env, pol) -> None:
    """The page implements the paths this policy takes and no others. Refuse
    the rest here, where the reason can be said, instead of exporting a page
    that silently flies a different observation."""
    if env._ahrs is not None:
        raise SystemExit("the page reads attitude as truth")
    if env._steer_rate is not None and env._steer_rate.encoder != "counts":
        raise SystemExit("the page implements the joint and counts steer rates only")
    if env._odo is None or env._odo.encoder != "counts" or env._odo.fusion != "front":
        raise SystemExit("the page implements only the counts encoder, front fusion")
    if env._crank is None or not env.hockey:
        raise SystemExit("the page expects the jointed righting module and the ball")
    if env.wings or env.swing or env.obs_style:
        raise SystemExit("the page has no policy-driven wing or style channels")
    if env.vel_window_s > 0.0 or env.zero_lat or not env.full:
        raise SystemExit("the page implements the un-windowed full action space")
    if not hasattr(pol, "enc"):
        raise SystemExit("the page implements the MinGRU policy only")
    if env._drive is None or not env._drive.servo:
        raise SystemExit("the page implements the detailed drivetrain only")


def _policy_arrays(pol) -> dict:
    out = {"obs_mean": pol.obs_mean, "obs_var": pol.obs_var,
           "enc_W": pol.enc[0], "enc_b": pol.enc[1],
           "mem_W": pol.mem[0], "mem_b": pol.mem[1]}
    for i, (W, b) in enumerate(pol.layers):
        out[f"W{i}"], out[f"b{i}"] = W, b
    return out


def write_policy(pol, path: Path) -> dict:
    """The weights as one float32 blob; returns the manifest. The npz stores
    float32, and the loader only widens it, so this is exact."""
    manifest, chunks, off = {}, [], 0
    for key, arr in _policy_arrays(pol).items():
        a = np.ascontiguousarray(arr, dtype=np.float32)
        assert np.array_equal(a.astype(np.float64), np.asarray(arr, np.float64)), key
        manifest[key] = {"offset": off, "shape": list(a.shape)}
        chunks.append(a.ravel())
        off += a.size
    np.concatenate(chunks).astype("<f4").tofile(path)
    return manifest


# The page's look where it differs from teleop's, by geom name (user,
# 2026-10-27): the roll cages solid, both blades orange, and the front tyre
# in the rear rollers' two greys rather than teleop's blue / orange quarters
# (those show its spin; the greys still do).
_ORANGE = (0.96, 0.46, 0.09, 1.0)
_GREY_LIGHT, _GREY_DARK = (0.70, 0.71, 0.73, 1.0), (0.38, 0.39, 0.41, 1.0)
PAGE_COLOURS = {
    "rollcage_rear": _ORANGE, "rollcage_front": _ORANGE,
    "swing_wing_left": _ORANGE, "swing_wing_left_toe": _ORANGE,
    "swing_wing_right": _ORANGE, "swing_wing_right_toe": _ORANGE,
    "front_tire_q0": _GREY_LIGHT, "front_tire_q1": _GREY_DARK,
    "front_tire_q2": _GREY_LIGHT, "front_tire_q3": _GREY_DARK,
}


def visible_geoms(model) -> list:
    """[geom id, r, g, b, a] for every bike geom teleop's bare look draws,
    from `strip_visuals` on a COPY: the exported model keeps its colours.
    The floor (body 0) is left out; the page draws its own ground.
    PAGE_COLOURS overrides by name."""
    from ..build_model import strip_visuals
    shown = copy.deepcopy(model)
    strip_visuals(shown)
    missing = set(PAGE_COLOURS) - {shown.geom(g).name for g in range(shown.ngeom)}
    assert not missing, f"PAGE_COLOURS names no geom: {sorted(missing)}"
    return [[g, *map(float, PAGE_COLOURS.get(shown.geom(g).name, shown.geom_rgba[g]))]
            for g in range(shown.ngeom)
            if shown.geom_bodyid[g] != 0 and shown.geom_rgba[g][3] > 0.0]


def sit_config() -> dict:
    """Lie down / get up as the BIKE runs it (hw/run_bike.py's SwingSit and
    its steer hold), read off run_bike so the two cannot drift; plus the
    page's two waits where the bike waits for the operator."""
    from ..hw import run_bike as rb
    return {
        "swing_sit": {"handOff": False, "restDeg": rb.LIE_REST_DEG,
                      "liftDps": rb.SIT_DPS, "lowerDps": rb.SIT_DPS,
                      "lowerMap": [list(a) for a in rb.LIE_MAP],
                      "lowerRollDps": rb.LIE_ROLL_DPS,
                      "lowerRollAcc": rb.LIE_ROLL_ACC,
                      "retryS": None, "pumpS": rb.PUMP_S},
        "steer_deg": rb.SIT_STEER_DEG, "steer_dps": rb.SIT_STEER_DPS,
        "prop_hold_s": PROP_HOLD_S, "retry_s": RETRY_S,
    }


def config(env, pol, manifest: dict) -> dict:
    m, drv, odo = env.model, env._drive, env._odo
    est = odo.est

    def act(n):
        return int(m.actuator(n).id)

    def jnt(n):
        j = m.joint(n)
        return {"qpos": int(j.qposadr[0]), "dof": int(j.dofadr[0])}

    def friction(g):
        return {"dof": g.dof, "qadr": g.qadr, "base": g.base,
                "base_q2": g.base_q2, "s0": g.s0, "sc": g.sc,
                "r0": g.r0, "rc": g.rc, "w_stick": g.w_stick,
                "damp": g._damp, "touch": g._touch,
                "ha": getattr(g, "_ha", 0.0), "hb": getattr(g, "_hb", 0.0)}

    gb = [friction(g) for g in env._gearbox]
    sr = env._steer_rate
    cs = env._crank
    from ..control.general_env import _PARKED
    from ..control.recovery import HANDOFF_RATE, RECOVER_DEG
    from ..righting_servo import AMPS_PER_COUNT
    ball_r = float(m.geom_size[m.geom("ball").id, 0])
    from ..control.general_spec import obs_layout_for
    from ..control.steer import STEER_LEAD_MAX, XC330_COUNTS_PER_RAD
    from ..hw.dynamixel import POS_WRAP
    from ..hw.odometry import COS_MIN, GRAVITY
    return {
        "policy": {"name": pol.name if hasattr(pol, "name") else None,
                   "manifest": manifest, "obs_clip": pol.obs_clip,
                   "bounds": pol.bounds.to_list()[:pol.act_dim]},
        "mujoco_version": mujoco.__version__,
        "plant_digest": getattr(pol, "plant_digest", None),
        "obs_layout": list(obs_layout_for(pol)),
        "ahrs_level": AHRS_LEVEL,
        "timestep": float(m.opt.timestep),
        "ctrl_dt": env.ctrl_dt,
        "substeps": env.substeps,
        "bounds": env.bounds.to_list()[:3],
        "v_max": env.v_max,
        "fall_roll": float(env.fall),
        "r_rear": float(env._r_rear),
        "steer_lead_max": STEER_LEAD_MAX,
        "eq_qpos": env._eq.tolist(),
        "init_roll": float(np.deg2rad(0.5)),
        "chassis_body": int(env._chassis),
        "actuators": {n: act(n) for n in
                      ("drive_a", "drive_b", "steer", "drive_a_motor",
                       "drive_b_motor")},
        "joints": {n: jnt(n) for n in
                   ("steer_joint", "input_a_spin", "input_b_spin")},
        "drivetrain": {
            "every": drv.every, "belt": drv.belt, "h_fw": drv.h_fw,
            "kp": drv.kp, "ki": drv.ki, "vlim": drv.vlim, "alpha": drv.alpha,
            "nc": drv.nc, "nm": drv.nm, "stall": drv.stall, "w0": drv.w0,
            "fc_in": list(drv.fc_in), "k_in": drv.k_in, "c_in": drv.c_in,
            "supply": drv.supply,
            "detent": None if not drv.detent else {
                "period": drv.period, "width": drv.width, "fmax": drv.fmax,
                "k_stick": drv.k_stick, "c_stick": drv.c_stick,
                "ga": drv.ga, "gb": drv.gb,
                "hill": drv.hill, "hill_saw": drv.hill_saw,
                "hill_phase": drv.hill_phase, "hill_flip": drv.hill_flip,
                "hill_offset": drv.hill_offset, "kinetic": drv.kinetic,
                "minus_scale": drv.minus_scale},
        },
        "gearbox": gb,
        "odometry": {
            "odo_dt": odo.odo_dt, "window_ms": odo.window_ms,
            "taper": odo.taper,
            "sensors": {k: list(v) for k, v in odo.adr.items()},
            "belt_ratio": est.belt_ratio, "mix_hub_a": est.mix_hub_a,
            "mix_hub_b": est.mix_hub_b, "r_wheel": est.r_wheel,
            "wheelbase": est.wheelbase, "rake": float(est.rake),
            "lon_smooth": est.lon_smooth,
            "steer_gear_ratio": float(
                odo.params["bike"]["steering"]["gear_ratio"]),
            "counts_per_rad": XC330_COUNTS_PER_RAD, "pos_wrap": POS_WRAP,
            "cos_min": COS_MIN, "gravity": GRAVITY,
        },
        "steer_rate": None if env._steer_rate is None else {
            "adr": sr.adr, "ratio": sr.ratio, "cpr": sr.cpr,
            "odo_dt": sr.odo_dt, "window_ms": sr.window_ms, "taper": sr.taper},
        "sit": sit_config(),
        "crank": {
            "aid": cs.aid, "qadr": cs.qadr, "dadr": cs.dadr,
            "ts": cs.ts, "w0": cs.w0, "k": cs.k, "i0": cs.i0,
            "kp_a": cs.kp_a, "kd_a": cs.kd_a, "nc": cs.nc,
            "goal_current": cs.goal_current, "supply": cs.supply,
            "amps_per_count": AMPS_PER_COUNT,
            "friction": friction(cs.friction),
            "deploy": float(m.actuator_ctrlrange[cs.aid, 1]),
            "blade": float(np.deg2rad(BLADE_CRANK_DEG)),
            "recover_deg": RECOVER_DEG, "handoff_rate": HANDOFF_RATE,
        },
        "ball": {"qadr": int(env._ball_q), "dadr": int(env._ball_v),
                 "body": int(m.body("ball").id), "mass": BALL_MASS,
                 "drag_tau": BALL_DRAG_TAU,
                 "radius": ball_r, "parked": _PARKED.tolist(),
                 # GeneralEnv parks it at the STUDY's radius, whatever this one is
                 "parked_z": float(env._ball_r),
                 # where the page puts it at reset, bike frame [x fwd, y left]
                 "start": [float(x) for x in env.p["hockey"]["ball"]["start"]],
                 "pelt_speed": PELT_SPEED, "pelt_standoff": PELT_STANDOFF},
        "shove": {"force": SHOVE_N, "nudge": NUDGE_N, "seconds": SHOVE_S},
        "render": {"geoms": visible_geoms(m), "ball_geom": int(m.geom("ball").id),
                   # the rear wheel, whose track the page draws
                   "trail_body": int(m.body("aow_hub").id)},
    }


def write_trace(env, pol, path: Path, seconds: float) -> None:
    """Fly the policy under SCHEDULE and record each step: the command, the
    observation the policy saw, the action it took and the state after.

    The command is written straight into the env between steps, as teleop's
    keys write it into DriveController, and is stored per step so the replay
    needs no copy of SCHEDULE."""
    obs, _ = env.reset(seed=0, options={"v_cmd": (0.0, 0.0)})
    pol.reset()
    psi0 = env._psi
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    rows = []
    for k in range(int(round(seconds / env.ctrl_dt))):
        t = k * env.ctrl_dt
        _, v, dpsi = [r for r in SCHEDULE if r[0] <= t + 1e-9][-1]
        if k == 0 or rows[-1]["sched"] != [v, dpsi]:
            env._psi_cmd = env._psi_cmd + np.deg2rad(dpsi) if dpsi else env._psi_cmd
        c, s = np.cos(env._psi_cmd), np.sin(env._psi_cmd)
        env._v_cmd_w = np.array([c * v, s * v])
        a = np.asarray(pol.action(obs), float) / scale
        obs_in = obs
        obs, _r, term, _trunc, _ = env.step(a)
        rows.append({"sched": [v, dpsi],
                     "v_cmd_w": env._v_cmd_w.tolist(), "psi_cmd": env._psi_cmd,
                     "obs": np.asarray(obs_in, float).tolist(),
                     "action": np.asarray(a, float).tolist(),
                     "qpos": env.data.qpos.tolist(),
                     "qvel": env.data.qvel.tolist(),
                     "fell": bool(term)})
        if term:
            break
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"psi0": psi0,
                                "detent_phase": env._drive.phase,
                                "steps": rows}))
    print(f"trace: {len(rows)} steps -> {path}")


def main() -> None:
    from ..params import load_params
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--policy", default=None,
                    help="move name (default: control.general_move)")
    ap.add_argument("--out", type=Path, default=SITE / "assets")
    ap.add_argument("--trace", type=Path, default=None,
                    help="also write a reference trace for check.mjs")
    ap.add_argument("--seconds", type=float, default=14.0)
    args = ap.parse_args()
    name = args.policy or load_params()["control"]["general_move"]
    env, pol = build_env(name)
    pol.name = name
    args.out.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveModel(env.model, str(args.out / "bike.mjb"), None)
    manifest = write_policy(pol, args.out / "policy.bin")
    (args.out / "config.json").write_text(
        json.dumps(config(env, pol, manifest), indent=1))
    for f in ("bike.mjb", "policy.bin", "config.json"):
        print(f"{args.out / f}  {(args.out / f).stat().st_size / 1e6:.2f} MB")
    if args.trace:
        write_trace(env, pol, args.trace, args.seconds)


if __name__ == "__main__":
    main()
