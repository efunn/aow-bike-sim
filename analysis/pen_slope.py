"""The pen, on a floor that is not level -- MuJoCo plan view, one panel per slope.

`pen.py` draws the ground track in matplotlib. This is the same question asked
of the RENDERER: the actual top-down MuJoCo view with the red world trail from
`record.py`, so the pen stroke sits on the checkered floor the operator sees
rather than on a pair of axes. The axis swept is FLOOR INCLINE.

WHY THIS EXISTS. A garage slab is poured to drain, typically 1-2% (0.57-1.15
deg). On a 1.02 kg bike the along-slope gravity component is unopposed, and
over a 5 s episode that is ~1.2 m of free drift at 1% and ~2.4 m at 2% --
against a measured `hold` drift span of 1.685-2.275 m across mu 0.5-2.0
(docs/plans/floors-and-the-contact-model.md section 3). So a code-legal slope
contributes as much wander as the ENTIRE friction axis, and any station-keeping
number taken on a garage floor is partly a slope reading. This draws that.

SLOPE IS IMPLEMENTED AS TILTED GRAVITY, NOT A TILTED FLOOR. For an infinite
plane the two are exactly equivalent -- the bike cannot tell them apart, only
the camera can -- and tilting gravity costs one line while tilting the floor
geom would invalidate every height in `bike_params.yaml` that is measured from
a level floor (`stand_clearance`, the wings' `clearance`, the righting reach
calculations, "the axle sits 51.2 mm off the floor"). Keeping the floor flat is
also what lets the red trail be drawn at a fixed z with no transformation, and
what keeps the checker grid readable as a distance scale.

WHAT TILTED GRAVITY DOES NOT REPRODUCE -- READ THIS BEFORE TRUSTING A NUMBER.
The MECHANICS are exact: the bike is pulled down the slope exactly as it would
be on a real one. The SENSING is not. `extract_state` takes roll from the body
quaternion against WORLD Z (balance.py:77), and world Z here is the FLOOR
NORMAL, because the floor never moved. A real AHRS references GRAVITY, so on a
real cross-slope it reports a standing roll offset of the slope angle and the
policy is fed a biased roll forever; in here it is fed a clean one.

That gap is not second order. The policy holds max roll between 0.2 and 3.3 deg
across the eval grid (sim_ahrs.py), and the TM151's own dynamic roll accuracy
is <1.5 deg RMS -- so a 1.15 deg bias from a 2% cross slope is the size of the
whole working range AND of the sensor's error budget. THE REAL BIKE SHOULD BE
EXPECTED TO DO WORSE ON A CROSS SLOPE THAN THESE PANELS SHOW, and the numbers
here are the mechanical part of the problem only.

Fixing it means either rotating the floor geom after all (world Z then stays
true vertical, at the cost of every floor-referenced height in
bike_params.yaml) or referencing the observed roll to the gravity vector, which
is a change to the observation contract and so to `general_spec`. Neither is
free, and neither is done.

The MAGNETOMETER is a non-issue for the opposite reason: there isn't one.
sim_ahrs.py:109 -- yaw comes from the true quaternion plus a drift term, so
tilting gravity cannot perturb a magnetometer that is not there. Worth knowing
if one is ever added: Earth's field is fixed in the WORLD, so a gravity-tilt
model would have to rotate the field by the same rotation, or the dip angle
between gravity and field becomes one that exists nowhere on Earth and any
real fusion filter fed it would produce nonsense.

DEGREES, NOT PERCENT -- they differ by ~2x in the range that matters.
tan(1 deg) = 1.75%, so a 1 deg default is already steeper than most garages;
0.57 deg is 1%. The default sweep spans both readings of "a slightly sloped
floor" so neither has to be argued about.

READ THE PANELS AGAINST ONE ANOTHER, NOT INDIVIDUALLY. Every panel shares ONE
camera, framed on the union of all paths, so a 4 m runaway and a 0.2 m hold are
drawn at the same scale and cannot be confused. The floor checker is 0.25 m per
square (`sim.floor_grid_m`) and is the scale bar.

    python analysis/pen_slope.py
    python analysis/pen_slope.py --slopes 0 0.57 1.15 2.0 --commands hold
    python analysis/pen_slope.py --across --tag cross    # tilt sideways instead
    python analysis/pen_slope.py --drivetrain --tag drivetrain_p100   # detailed drive

Writes analysis/plots/pen_slope_<policy>[_<tag>].png and prints the table.
Read-only: loads moves/*.npz and config, overrides gravity and floor extent IN
MEMORY ONLY, and writes nothing but the PNG.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aow_sim import drivetrain_model as dm
from aow_sim.build_model import load_params, tune_lighting
from aow_sim.control.balance import extract_state
from aow_sim.control.general_env import _load_rl_config
from aow_sim.record import _camera, _trail
# The CANONICAL tilt lives in run_drive so teleop feel and these numbers
# cannot drift apart -- press ` in a teleop session and it is this function.
from aow_sim.run_drive import (floor_height, slope_components,
                               tilt_gravity)
from rsa_policies import REPO, env_for, load_general

G = 9.81
UPRIGHT_LIMIT_DEG = 60.0        # run_drive.UPRIGHT_LIMIT_DEG

# (label, (v_lon, v_lat), dpsi_deg) -- the pen.py shape, minus the held turns.
# `hold` first because it is the command the slope actually destroys: it is the
# one with nothing commanded to mask the drift.
COMMANDS = {
    "hold":  ((0.0, 0.0), 0),
    "fwd":   ((0.8, 0.0), 0),
    "crabL": ((0.0, 0.396), 0),
    "turnL": ((0.0, 0.0), 90),
}


def _plots_dir():
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _soften_floor(model, fade: float, name: str = "floor_grid"):
    """Wash the checker out toward its own mean, in place.

    The checker is a build-time texture (`rgb1`/`rgb2` in build_model), not a
    parameter, so there is nothing to pass -- but the baked bytes are editable
    and pulling both tones toward their mean keeps the grid readable as a
    distance scale while stopping it competing with the pen. fade=1 is the
    original, 0 is flat grey.
    """
    tid = model.texture(name).id
    a = model.tex_adr[tid]
    n = model.tex_height[tid] * model.tex_width[tid] * model.tex_nchannel[tid]
    d = model.tex_data[a:a + n].astype(float)
    model.tex_data[a:a + n] = np.clip(
        d.mean() + (d - d.mean()) * fade, 0, 255).astype(np.uint8)


def _downhill_label(gxy, slope_deg: float) -> str:
    """The downhill indicator, as an AXIS LABEL rather than an in-panel badge.

    It started as a badge in the corner and covered the pen: the panels that
    matter most are the tightly-framed ones, which is exactly where a fixed
    corner box eats the most of the picture. An axis label cannot occlude
    anything. The arrow glyph points the way it points ON SCREEN -- +Y renders
    upward at azimuth 90 -- so the label cannot be read backwards off the
    figure the way body-axis wording can.
    """
    n = float(np.linalg.norm(gxy[:2]))
    if n < 1e-9:
        return "level floor"
    gx, gy = gxy[0], gxy[1]
    arrow = ("\u2192" if gx > 0 else "\u2190") if abs(gx) >= abs(gy) else \
            ("\u2191" if gy > 0 else "\u2193")
    return f"downhill {arrow}  {slope_deg:.2f}\u00b0"


def total_tilt_deg(gravity) -> float:
    """The single slope angle the two components add up to."""
    return float(np.degrees(np.arccos(min(1.0, abs(gravity[2]) / G))))


def roll_out(pol, env, v_cmd, dpsi_deg, seconds, seed: int = 7):
    """One held command on the currently-tilted floor.

    Returns (xy path, final qpos, final qvel, fell). The final state is kept
    because the render is a SINGLE frame at the end carrying the whole trail --
    there is no point animating a pen that is only read as a finished stroke.
    """
    b = pol.bounds
    scale = np.array([b.steer_rate_max, b.hub_max, b.diff_max])
    obs, _ = env.reset(seed=seed, options={"v_cmd": v_cmd,
                                        "psi_cmd_rel": np.deg2rad(dpsi_deg),
                                        "difficulty": 1.0})
    xy, fell = [], False
    for _ in range(int(seconds / env.ctrl_dt)):
        a = np.asarray(pol.action(obs), float) / scale
        obs, _r, term, trunc, _i = env.step(a[:env.action_space.shape[0]])
        s = extract_state(env.data, env._p0)
        xy.append(env.data.qpos[:2].copy())
        if abs(np.degrees(s.roll)) > UPRIGHT_LIMIT_DEG:
            fell = True
            break
        if term or trunc:
            break
    return (np.array(xy), env.data.qpos.copy(), env.data.qvel.copy(), fell)


# The rose, laid out as it is drawn. Bearing is from +X toward +Y, which on
# screen is from RIGHT toward UP, so the glyph in each cell points where the
# panel's pen will run. Centre is level and is also the grey baseline for all
# eight.
_ROSE = [
    [(135, "\u2196 NW"), ( 90, "\u2191 N"), ( 45, "\u2197 NE")],
    [(180, "\u2190 W"),  (None, "level"),    (  0, "\u2192 E")],
    [(225, "\u2199 SW"), (270, "\u2193 S"), (315, "\u2198 SE")],
]


def _cell_env(args, pol, params, cfg, env, tilt_deg, bearing_deg):
    """(env, model, data, normal) for one rose cell, by whichever method.

    GRAVITY mode reuses one env and repoints gravity -- cheap, and the floor
    stays flat so world Z is the floor normal. FLOOR mode has to REBUILD,
    because the tilt is baked at spec time (see build_model), and in exchange
    world Z stays vertical and the roll the policy is fed carries the standing
    offset a real AHRS would see on that slope.
    """
    if args.tilt_gravity:
        tilt_gravity(env.model, *((0.0, 0.0) if tilt_deg == 0.0 else
                                  slope_components(tilt_deg, bearing_deg)))
        return env, env.model, env.data, np.array([0.0, 0.0, 1.0])
    q = {**params, "sim": {**params["sim"],
                           "floor_tilt_deg": float(tilt_deg),
                           "floor_tilt_bearing_deg": float(bearing_deg)}}
    e = env_for(pol, q, cfg)
    th, br = np.deg2rad(tilt_deg), np.deg2rad(bearing_deg)
    n = np.array([np.sin(th) * np.cos(br), np.sin(th) * np.sin(br), np.cos(th)])
    return e, e.model, e.data, n


def _on_floor(xy, n):
    """Trail points lifted onto the sloping surface, as Nx3."""
    z = floor_height(n, xy) + 0.004
    return np.column_stack([xy[:, 0], xy[:, 1], z])


def run_rose(args, pol, env, model, data, plt, params=None, cfg=None):
    """3x3: one slope magnitude, eight downhill directions, level in the middle.

    Reads as a POLAR PLOT of the policy's disturbance rejection. The plant is
    mirror-symmetric about its roll plane (`axle_cant_deg` 0, measured), so N
    and S must match for an unbiased policy and any difference between them is
    the POLICY's handedness -- the same argument steer_stability.py makes for
    sweeping both signs. E and W are NOT expected to match: forward and reverse
    are genuinely different for this bike.
    """
    # MULTIPLE SEEDS, because one is not enough here and that is measured,
    # not assumed. Hold drift is a squiggle whose endpoint is sensitive to
    # initial conditions: at a single seed this policy's SW cell read 0.28x its
    # linear prediction, which looked like strong sub-additivity and was simply
    # scatter -- four other seeds put it at 0.80-0.95x. A one-seed rose invites
    # exactly that false reading, so every cell is run `--seeds` times, all of
    # them are drawn, and the table carries the spread beside the mean.
    seeds = [7 + 4 * i for i in range(max(1, args.seeds))]
    cells = {}
    for r, row in enumerate(_ROSE):
        for c, (bearing, label) in enumerate(row):
            e, m_, d_, n = _cell_env(args, pol, params, cfg, env,
                                     0.0 if bearing is None else args.rose_slope,
                                     0.0 if bearing is None else bearing)
            v_cmd, dpsi = COMMANDS[args.rose_command]
            runs = [roll_out(pol, e, v_cmd, dpsi, args.seconds, seed=s)
                    for s in seeds]
            cells[(r, c)] = (bearing, label, runs, e, m_, d_, n)

    base_runs = cells[(1, 1)][2]
    base_n = cells[(1, 1)][6]
    d0 = np.array([r[0][-1] - r[0][0] for r in base_runs])
    free = 0.5 * G * np.sin(np.deg2rad(args.rose_slope)) * args.seconds ** 2
    print(f"\n{args.policy}   slope rose, {args.rose_slope:.2f}\u00b0 "
          f"({np.tan(np.deg2rad(args.rose_slope)) * 100:.2f}%), "
          f"command '{args.rose_command}', {args.seconds:.0f} s, "
          f"{len(seeds)} seed(s)")
    print(f"{'dir':>7}{'bearing':>9}{'vs level m':>12}{'sd':>7}{'min':>8}"
          f"{'max':>8}{'rejected':>10}{'fell':>6}")
    means = {}
    for r, row in enumerate(_ROSE):
        for c, _ in enumerate(row):
            bearing, label, runs = cells[(r, c)][:3]
            fell = sum(1 for x in runs if x[3])
            if bearing is None:
                nets = [float(np.linalg.norm(x[0][-1] - x[0][0])) for x in runs]
                print(f"{'level':>7}{'--':>9}{'--':>12}{'--':>7}{'--':>8}{'--':>8}"
                      f"{'--':>10}{fell:>6}"
                      f"    (level drift itself {np.mean(nets):.2f} m)")
                continue
            u = np.array([np.cos(np.deg2rad(bearing)), np.sin(np.deg2rad(bearing))])
            rel = np.array([(x[0][-1] - x[0][0] - b) @ u
                            for x, b in zip(runs, d0)])
            means[label.split()[-1]] = rel
            print(f"{label.split()[-1]:>7}{bearing:>9.0f}{rel.mean():>12.2f}"
                  f"{rel.std():>7.2f}{rel.min():>8.2f}{rel.max():>8.2f}"
                  f"{100 * (1 - rel.mean() / free):>9.0f}%{fell:>6}")
    print(f"  free-drift at this tilt = {free:.2f} m. Every cell is the SAME "
          f"total tilt: the diagonals are")
    print("  a rotated version of the same hill, not a steeper one "
          "(run_drive.slope_components).")
    if len(seeds) > 1:
        d = means["N"] - means["S"]
        agree = int(np.sum(np.sign(d) == np.sign(d.mean())))
        print(f"  handedness: N {means['N'].mean():+.2f} vs S "
              f"{means['S'].mean():+.2f}, difference {d.mean():+.2f} "
              f"(sd {d.std():.2f}, same sign in {agree}/{len(d)} seeds).")
        print("  The plant is mirror-symmetric, so a real gap here is the "
              "POLICY, not the bike.")

    # one camera for all nine -- directions have to be comparable to each other
    allxy = np.concatenate([x[0] for c in cells.values() for x in c[2]])
    lookat = np.array([allxy[:, 0].mean(), allxy[:, 1].mean(), 0.0])
    span = float(max(np.ptp(allxy[:, 0]), np.ptp(allxy[:, 1])))
    distance = max(span, args.min_span) * 1.5
    cam = _camera(model, "top", distance, -89.0, 90.0, lookat)
    r_red, r_grey = 0.0045 * distance, 0.0028 * distance
    fig, axes = plt.subplots(3, 3, figsize=(8.7, 7.3), squeeze=False)
    step = max(1, args.trail_every)
    for r, row in enumerate(_ROSE):
        for c, _ in enumerate(row):
            bearing, label, runs, _e, m_, d_, n = cells[(r, c)]
            # Each cell may own its own MODEL in floor-tilt mode, so the
            # renderer is per cell rather than per figure.
            m_.vis.global_.offwidth = max(args.px, m_.vis.global_.offwidth)
            m_.vis.global_.offheight = max(args.px, m_.vis.global_.offheight)
            tune_lighting(m_)
            _soften_floor(m_, args.grid_fade)
            renderer = mujoco.Renderer(m_, int(args.px * 0.62), args.px,
                                       max_geom=20000)
            # Pose the bike at the FIRST seed's end state; every seed's pen is
            # drawn. The spread between the red strokes is the run-to-run
            # scatter, which is the honest way to show a quantity this
            # sensitive -- a single clean stroke would overstate the result.
            d_.qpos[:], d_.qvel[:] = runs[0][1], runs[0][2]
            mujoco.mj_forward(m_, d_)
            renderer.update_scene(d_, camera=cam)
            if bearing is not None:
                for bx in base_runs:
                    _trail(renderer.scene, _on_floor(bx[0][::step], base_n),
                           rgba=(0.25, 0.25, 0.30, 1.0), radius=r_grey)
            for x in runs:
                _trail(renderer.scene, _on_floor(x[0][::step], n), radius=r_red)
            ax = axes[r][c]
            ax.imshow(renderer.render().copy())
            renderer.close()
            ax.set_anchor("N")
            ax.set_xticks([]); ax.set_yticks([])
            # The fall count goes in the X-LABEL, not a title: a title sits
            # above the panel and therefore on top of the previous row's
            # label. One line per cell, and its colour carries the verdict.
            nf = sum(1 for x in runs if x[3])
            lab = label if bearing is None else (
                f"{label}  {means[label.split()[-1]].mean():+.2f} m")
            if nf:
                lab += f"   FELL {nf}/{len(runs)}"
            ax.set_xlabel(lab, fontsize=9.5,
                          color=("#b22222" if nf else
                                 "0.35" if bearing is None else "#1a5fb4"))
    fig.suptitle(f"Slope rose ({'GRAVITY tilted' if args.tilt_gravity else 'floor tilted'}), "
                 f"{args.rose_slope:.2f}\u00b0 "
                 f"({np.tan(np.deg2rad(args.rose_slope)) * 100:.1f}%), "
                 f"'{args.rose_command}' \u2014 {args.policy}", fontsize=13)
    caption = (
        f"Each panel tilts the floor downhill the way its arrow points; the "
        f"centre is level.   {args.seconds:.0f} s per panel.\n"
        f"Red = that direction, one stroke per seed; the thinner dark lines "
        f"are the level runs. Labels are mean downhill gain vs level.\n"
        f"All eight are the SAME total tilt \u2014 the diagonals are not "
        f"steeper. One camera for all nine. Checker = {args.grid_m:g} m.")
    # The bottom row carries x-labels of its own, so the caption needs a
    # reserved band rather than the sliver a wide figure gets away with.
    # h_pad, and every image anchored to the top of its axes: imshow keeps a
    # fixed aspect, so a wide frame in a squarer axes leaves vertical slack
    # that matplotlib splits evenly -- which put each row's x-label underneath
    # the row below it.
    # The caption band is a fixed number of INCHES, converted to a fraction --
    # a fixed fraction reserves a quarter of a two-row figure and a sliver of
    # a six-row one.
    band = 0.70 / 7.3          # the rose's figure height, in inches
    fig.tight_layout(rect=(0, band, 1, 1), h_pad=1.4)
    fig.text(0.5, 0.006, caption, ha="center", va="bottom", fontsize=8.5,
             color="0.25", linespacing=1.6)
    tag = f"_{args.tag}" if args.tag else ""
    out = _plots_dir() / f"pen_rose_{args.policy}{tag}.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"\n  wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", default="general_rl_cmd_curriculum2b",
                    help="move name under moves/")
    ap.add_argument("--slopes", type=float, nargs="*",
                    default=[0.0, 0.57, 1.15, 2.0],
                    help="floor incline in DEGREES (0.57 = 1%%, 1.15 = 2%%)")
    ap.add_argument("--commands", nargs="*", default=["hold", "fwd"],
                    choices=list(COMMANDS))
    ap.add_argument("--across", action="store_true",
                    help="tilt sideways (downhill toward +Y) instead of ahead")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--trail-every", type=int, default=3,
                    help="draw every Nth path point as a trail bead")
    ap.add_argument("--px", type=int, default=520, help="panel size in pixels")
    ap.add_argument("--floor-size", type=float, default=30.0,
                    help="render extent of the floor plane, half-width in m. Must outlast the CAMERA, which is framed wider than the path.")
    ap.add_argument("--min-span", type=float, default=0.9,
                    help="closest the camera may frame, in m")
    ap.add_argument("--grid-m", type=float, default=0.5,
                    help="floor checker pitch in m")
    ap.add_argument("--grid-fade", type=float, default=0.45,
                    help="checker contrast, 1 = as built, 0 = flat grey")
    ap.add_argument("--tilt-gravity", action="store_true",
                    help="tilt GRAVITY instead of the floor. Mechanically "
                         "equivalent but world Z becomes the floor normal, so "
                         "the policy's roll loses its gravity reference and "
                         "the bike coasts downhill uncorrected -- measured 5-30x "
                         "more drift than the floor-tilted truth. Kept only to "
                         "reproduce that comparison.")
    ap.add_argument("--rose", action="store_true",
                    help="3x3: eight downhill directions at one tilt, "
                         "level in the middle")
    ap.add_argument("--rose-slope", type=float, default=1.15,
                    help="tilt for --rose in DEGREES (1.15 = 2%%)")
    ap.add_argument("--rose-command", default="hold",
                    choices=list(COMMANDS))
    ap.add_argument("--seeds", type=int, default=5,
                    help="rollouts per rose cell; 1 is not enough, see "
                         "the note in run_rose")
    ap.add_argument("--tag", default="",
                    help="suffix for the PNG, so variants never overwrite each "
                         "other at a tracked name")
    ap.add_argument("--drivetrain", nargs="?", const=True, default=None,
                    metavar="PATH",
                    help="the opt-in detailed drivetrain "
                         "(config/drivetrain_model.yaml, or PATH); same flags "
                         "as run_drive. Pass --tag with it")
    ap.add_argument("--drivetrain-without", nargs="+", default=(),
                    choices=dm.PARTS, metavar="PART")
    ap.add_argument("--servo-gains", default=None, metavar="P:I",
                    help="firmware Velocity P:I in table units under "
                         "--drivetrain (default the overlay's 100:1920)")
    args = ap.parse_args()

    params = load_params()
    if args.drivetrain or args.drivetrain_without or args.servo_gains:
        gains = (tuple(int(x) for x in args.servo_gains.split(":"))
                 if args.servo_gains else None)
        params = dm.with_drivetrain(
            params, None if args.drivetrain in (None, True) else args.drivetrain,
            without=args.drivetrain_without, gains=gains)
    # In-memory only. A bigger plane keeps a runaway on a drawn floor instead of
    # off the edge into grey; the checker is texuniform so squares stay 0.25 m
    # whatever this is, and the scale reading is unaffected.
    params["sim"] = {**params["sim"], "floor_size": args.floor_size,
                     "floor_grid_m": args.grid_m}
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}

    pol = load_general(args.policy)
    env = env_for(pol, params, cfg)
    model, data = env.model, env.data

    if args.rose:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        run_rose(args, pol, env, model, data, plt, params, cfg)
        return

    # ---- PASS 1. Floor-tilt mode needs a REBUILT model per slope (the tilt is
    # baked at spec time), so envs are cached by angle rather than one env
    # being repointed. Seeds for the same reason the rose has them: a single
    # rollout of a drifting hold is not a measurement.
    seeds = [7 + 4 * k for k in range(max(1, args.seeds))]
    bearing = 90.0 if args.across else 0.0
    cache, runs, downs = {}, {}, {}
    for s_deg in args.slopes:
        cache[s_deg] = _cell_env(args, pol, params, cfg, env, s_deg, bearing)
    for cmd in args.commands:
        v_cmd, dpsi = COMMANDS[cmd]
        for s_deg in args.slopes:
            e, m_, d_, n = cache[s_deg]
            runs[(cmd, s_deg)] = [roll_out(pol, e, v_cmd, dpsi, args.seconds,
                                           seed=s) for s in seeds]
            # Horizontal downhill direction, however the tilt was applied: the
            # floor normal's xy in floor mode, gravity's xy in gravity mode.
            downs[(cmd, s_deg)] = (np.array(m_.opt.gravity[:2])
                                   if args.tilt_gravity else n[:2])

    base = {c: np.array([r[0][-1] - r[0][0] for r in runs[(c, 0.0)]])
            for c in args.commands} if 0.0 in args.slopes else {}
    print(f"\n{args.policy}   {dm.describe(params)}   "
          f"{'across' if args.across else 'along'}-slope, "
          f"{'GRAVITY tilted' if args.tilt_gravity else 'floor tilted'}, "
          f"{args.seconds:.0f} s, {len(seeds)} seed(s)")
    if not base:
        print("  no 0.0 in --slopes: no level baseline, delta column is absolute.")
    print(f"{'cmd':>7}{'slope':>8}{'as %':>7}{'vs level m':>12}{'sd':>7}"
          f"{'free-drift m':>14}{'rejected':>10}{'fell':>6}")
    for cmd in args.commands:
        for s_deg in args.slopes:
            rs = runs[(cmd, s_deg)]
            nf = sum(1 for r in rs if r[3])
            free = 0.5 * G * np.sin(np.deg2rad(s_deg)) * args.seconds ** 2
            u = downs[(cmd, s_deg)]
            nn = float(np.linalg.norm(u))
            pct = np.tan(np.deg2rad(s_deg)) * 100
            if nn < 1e-9 or not base:
                print(f"{cmd:>7}{s_deg:>8.2f}{pct:>7.2f}{'--':>12}{'--':>7}"
                      f"{free:>14.2f}{'--':>10}{nf:>6}")
                continue
            u = u / nn
            rel = np.array([(r[0][-1] - r[0][0] - b) @ u
                            for r, b in zip(rs, base[cmd])])
            print(f"{cmd:>7}{s_deg:>8.2f}{pct:>7.2f}{rel.mean():>12.2f}"
                  f"{rel.std():>7.2f}{free:>14.2f}"
                  f"{100 * (1 - rel.mean() / free):>9.0f}%{nf:>6}")
    print("  vs level   = downhill displacement minus the same command on a "
          "level floor.")
    print("  rejected   = how much of the free drift the controller refused.")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nno PNG: matplotlib is not installed in this interpreter")
        return

    nr, nc = len(args.commands), len(args.slopes)
    # Row height matched to the IMAGE aspect plus its title and label. The
    # images are anchored north (so a label cannot collide with the row below),
    # which means any slack in the axes box shows as dead white space under
    # each row -- at 2 rows that was a quarter of the figure.
    pw, ph_ = 2.9, 2.9 * 0.62
    fig, axes = plt.subplots(nr, nc, figsize=(pw * nc, (ph_ + 0.62) * nr),
                             squeeze=False)
    step = max(1, args.trail_every)
    for i, cmd in enumerate(args.commands):
        rowxy = np.concatenate([r[0] for s in args.slopes
                                for r in runs[(cmd, s)]])
        lookat = np.array([rowxy[:, 0].mean(), rowxy[:, 1].mean(), 0.0])
        span = float(max(np.ptp(rowxy[:, 0]), np.ptp(rowxy[:, 1])))
        distance = max(span, args.min_span) * 1.5
        r_red, r_grey = 0.0045 * distance, 0.0028 * distance
        for j, s_deg in enumerate(args.slopes):
            e, m_, d_, n = cache[s_deg]
            cam = _camera(m_, "top", distance, -89.0, 90.0, lookat)
            m_.vis.global_.offwidth = max(args.px, m_.vis.global_.offwidth)
            m_.vis.global_.offheight = max(args.px, m_.vis.global_.offheight)
            tune_lighting(m_)
            _soften_floor(m_, args.grid_fade)
            renderer = mujoco.Renderer(m_, int(args.px * 0.62), args.px,
                                       max_geom=20000)
            rs = runs[(cmd, s_deg)]
            d_.qpos[:], d_.qvel[:] = rs[0][1], rs[0][2]
            mujoco.mj_forward(m_, d_)
            renderer.update_scene(d_, camera=cam)
            if base and s_deg != 0.0:
                bn = cache[0.0][3]
                for br in runs[(cmd, 0.0)]:
                    _trail(renderer.scene, _on_floor(br[0][::step], bn),
                           rgba=(0.25, 0.25, 0.30, 1.0), radius=r_grey)
            for r in rs:
                _trail(renderer.scene, _on_floor(r[0][::step], n), radius=r_red)
            ax = axes[i][j]
            ax.imshow(renderer.render().copy())
            renderer.close()
            ax.set_anchor("N")
            ax.set_xticks([]); ax.set_yticks([])
            nf = sum(1 for r in rs if r[3])
            ax.set_title(f"{s_deg:.2f}\u00b0  ({np.tan(np.deg2rad(s_deg)) * 100:.2f}%)",
                         fontsize=9)
            lab = _downhill_label(downs[(cmd, s_deg)], s_deg)
            if nf:
                lab += f"   FELL {nf}/{len(rs)}"
            ax.set_xlabel(lab, fontsize=8.5,
                          color=("#b22222" if nf else
                                 "0.35" if s_deg == 0.0 else "#1a5fb4"))
            if j == 0:
                ax.set_ylabel(cmd, fontsize=11)

    axis_short = "cross-slope" if args.across else "along-slope"
    how = "GRAVITY tilted" if args.tilt_gravity else "floor tilted"
    fig.suptitle(f"Pen on a sloping floor, {axis_short} ({how}) "
                 f"\u2014 {args.policy}, {dm.describe(params)}", fontsize=13)
    where = ("+Y, the bike's left, UP in these panels" if args.across
             else "+X, straight ahead, RIGHT in these panels")
    caption = (
        f"Downhill is {where} \u2014 see the label under each panel.   "
        f"{args.seconds:.0f} s per panel, {len(seeds)} seed(s).\n"
        f"Red = this slope, one stroke per seed; the thinner dark lines are "
        f"the level runs. Where only red shows, the slope changed nothing.\n"
        f"The camera is shared across a ROW (one command), not across rows: "
        f"slopes are comparable, commands are not.\n"
        f"Floor checker = {args.grid_m:g} m per square."
        + ("  Gravity is tilted and the floor left flat, so the policy's roll "
           "loses its gravity reference \u2014 this OVERSTATES drift."
           if args.tilt_gravity else
           "  The floor geom is rotated, so world Z stays vertical and the "
           "roll the policy reads is the physical one."))
    # The caption band is a fixed number of INCHES, converted to a fraction --
    # a fixed fraction reserves a quarter of a two-row figure and a sliver of
    # a six-row one.
    band = 0.70 / ((ph_ + 0.62) * nr)
    fig.tight_layout(rect=(0, band, 1, 1), h_pad=1.4)
    fig.text(0.5, 0.006, caption, ha="center", va="bottom", fontsize=8.5,
             color="0.25", linespacing=1.6)
    tag = f"_{args.tag}" if args.tag else ""
    out = _plots_dir() / f"pen_slope_{args.policy}{tag}.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"\n  wrote {out}")


if __name__ == "__main__":
    main()
