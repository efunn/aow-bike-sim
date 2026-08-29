"""Two policies side by side, driving the whole eval grid. MuJoCo, top-down.

The per-command bars say WHICH commands a policy is bad at. They cannot say
what it does instead -- and "drives backwards at 0.8 m/s and declines to turn"
is a statement about the PATH, which took a teleop session and a probe script
to establish. This is that, for every command, for two policies at once.

The view is teleop's overhead: the bike from above with the pen trail behind
it. Both panels share one camera distance per command, so the two paths are
directly comparable rather than each auto-zoomed to its own extent.

SAME SEEDS AS THE EVAL (`10_000 + k`, `_eval_episodes`' own rule), so a clip is
the same episode the metrics blocks and the bar charts describe. If the table
says a policy fell on `fwd 0.6 crabR 0.4 +180`, this is that fall.

REAL TIME, the first `--seconds-per-move` of each episode. Not the whole
episode compressed, which would give every clip a different speed and hide how
fast anything happens. The rollout still runs to the end, so `fell` is the same
fact the tables report: a fall inside the window is shown, and one after the
window is captioned rather than silently dropped.

TWO PASSES, as aow_sim.record does. The camera cannot be framed until the path
is known, and a fixed overhead camera the bike drives out of shows nothing.

  python analysis/eval_video.py
  python analysis/eval_video.py --policies general_rl_odo_ahrs \
      general_rl_odo_ahrs_pitch --tag pitch_arm

Writes traces/eval_video/<a>_vs_<b><tag>.mp4 -- traces/ is gitignored, as
aow_sim.record and wheel_slowmo already do for video.
"""

from __future__ import annotations

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aow_sim.build_model import build_model, load_params, tune_lighting
from aow_sim.run_drive import _overlay
from aow_sim.train_general_rl import eval_cmds
from per_command import COLORS, _cfg_for, label_for, order_and_labels
from rsa_policies import env_for, load_general

OUT_DIR = Path(__file__).resolve().parents[1] / "traces" / "eval_video"


def _rollout(job):
    """Every episode of the grid for ONE policy, in a worker process.

    Returns qpos for the frames that will be SHOWN plus the fall time, which
    comes from the full episode -- so the caption can say "falls at 8.2 s,
    after this clip" rather than showing a clean-looking clip for a command
    the tables count as a fall.

    Built inside the worker because a MuJoCo model does not pickle; see
    per_command.run for why this is parallel over policies, not commands.
    """
    name, encoder, ahrs, tau, window = job
    params = load_params()
    cfg = _cfg_for(ahrs, tau)
    pol = load_general(name)
    if encoder:
        pol.odometry_encoder = encoder
    env = env_for(pol, params, cfg)
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n_act = env.action_space.shape[0]
    out = []
    for k, (v_lon, v_lat, dpsi) in enumerate(eval_cmds(cfg["env"]["v_max"])):
        obs, _ = env.reset(seed=10_000 + k, options={
            "v_cmd": (v_lon, v_lat), "psi_cmd_rel": dpsi, "difficulty": 1.0})
        n_show = int(window / env.ctrl_dt) + 1
        qpos, steps, fell = [env.data.qpos.copy()], 0, False
        while True:
            a = (np.asarray(pol.action(obs), float) / scale)[:n_act]
            obs, _r, term, trunc, _i = env.step(a)
            steps += 1
            if len(qpos) < n_show:
                qpos.append(env.data.qpos.copy())
            if term or trunc:
                fell = bool(term)
                break
        # The dial's COMMAND half. `_overlay` takes (heading, world velocity)
        # and only falls back to reading a live DriveController when this is
        # None -- which is what lets the overlay run here with c=None. Both
        # are constant: the eval holds one command for the episode.
        out.append({"qpos": np.array(qpos), "fell": fell,
                    "t_end": steps * env.ctrl_dt, "dt": env.ctrl_dt,
                    "psi_cmd": float(env._psi_cmd),
                    "v_cmd_w": np.asarray(env._v_cmd_w, float).copy()})
    return name, out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policies", nargs=2, metavar="NAME",
                    default=["general_rl_odo_ahrs", "general_rl_odo_ahrs_rand2"])
    ap.add_argument("--encoder", default="counts",
                    choices=("counts", "ideal", "reported", ""))
    ap.add_argument("--ahrs", default="tm151",
                    choices=("none", "tm151_static", "tm151", "tm171"))
    ap.add_argument("--ahrs-tau", type=float, default=0.19)
    ap.add_argument("--seconds-per-move", type=float, default=5.0)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--size", type=int, default=560, help="pixels per panel")
    ap.add_argument("--distance", type=float, default=2.6,
                    help="overhead camera distance [m]; 2.6 is teleop's own")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()
    tag = f"_{args.tag}" if args.tag and not args.tag.startswith("_") else args.tag

    import imageio.v2 as imageio

    v_max = _cfg_for(args.ahrs, args.ahrs_tau)["env"]["v_max"]
    cmds = eval_cmds(v_max)
    print(f"pass 1: rolling out 2 policies x {len(cmds)} commands, in parallel")
    jobs = [(n, args.encoder, args.ahrs, args.ahrs_tau, args.seconds_per_move)
            for n in args.policies]
    with ProcessPoolExecutor(max_workers=2) as ex:
        got = dict(ex.map(_rollout, jobs))
    runs = [got[n] for n in args.policies]

    # One model for rendering: both policies fly the same bike, and only the
    # observation config differs between their envs. It must be built the way
    # GeneralEnv builds its own (general_env.py:127) -- a bare
    # build_model(params) is 21 qpos against the env's 28, and replaying a
    # recorded qpos into it fails on the shape.
    params = load_params()
    env_cfg = _cfg_for(args.ahrs, args.ahrs_tau)["env"]
    # `hockey` is not a flag in the config -- GeneralEnv derives it from
    # ball_prob (general_env.py:108), and the ball is a FREE JOINT, so getting
    # it wrong is 7 qpos and a broadcast error rather than a missing prop.
    hockey = bool(env_cfg.get("ball_prob", 0.0) > 0.0)
    wings = bool(env_cfg.get("act_wings", False) or env_cfg.get("obs_wings", False))
    swing = bool(env_cfg.get("act_swing", False) or env_cfg.get("obs_swing", False))
    model = build_model(params, variant="full", hockey=hockey,
                        righting=wings or swing, wings=wings, swing=swing)
    px = args.size
    model.vis.global_.offwidth = max(px, model.vis.global_.offwidth)
    model.vis.global_.offheight = max(px, model.vis.global_.offheight)
    # WIDEN THE FLOOR, THEN LIGHT IT, THEN BUILD THE RENDERER -- in that
    # order, and no wider than needed.
    #
    # The stock floor is a 6 m plane and a 5 s run at 0.8 m/s reaches its
    # edge, which renders as black void. But `tune_lighting` sizes
    # `vis.map.shadowclip` FROM THE FLOOR (build_model.py:1764), so widening
    # it afterwards leaves the shadow box at the old +-3 m and the bike stops
    # casting a shadow once it drives out -- the same defect that fix exists
    # to cure, reintroduced from the other side. Calling tune_lighting after a
    # 60 m floor is no better: shadowclip ~25 spreads the same 8192 texels
    # over a vast square and washes the shadow out, which its docstring warns
    # about directly.
    #
    # So size the floor from the GROUND ACTUALLY COVERED -- the rollouts are
    # already in hand from pass 1 -- plus what the camera can see past the
    # bike. That keeps shadowclip near its tuned value.
    reach = max(float(np.abs(e["qpos"][:, :2]).max())
                for run in runs for e in run)
    floor_half = reach + args.distance * 0.75 + 0.5
    model.geom_size[model.geom("floor").id, :2] = floor_half
    tune_lighting(model)
    print(f"floor half-extent {floor_half:.1f} m (paths reach "
          f"{reach:.1f} m), shadowclip {model.vis.map.shadowclip:.2f}")
    renderer = mujoco.Renderer(model, px, px)
    data = mujoco.MjData(model)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.elevation, cam.azimuth = -89.0, 90.0     # straight down, +X up-screen
    cam.distance = args.distance

    # EVEN pixel dimensions, both axes: h264 with yuv420p rejects an odd one,
    # and the failure is an ffmpeg stderr dump rather than a python error.
    dpi = 100
    w_px, h_px = 2 * px + 24, px + 132
    w_px += w_px % 2
    h_px += h_px % 2
    fig, axes = plt.subplots(1, 2, figsize=(w_px / dpi, h_px / dpi), dpi=dpi)
    fig.subplots_adjust(top=0.90, bottom=0.055, left=0.005, right=0.995,
                        wspace=0.02)
    # ONE header row: policy name, move banner, policy name. The names are
    # fig.text at the panel centres rather than ax.set_title, because a title
    # sits at whatever height matplotlib chooses and the banner would not line
    # up with it. The explanatory line goes at the BOTTOM, out of the way.
    ims, notes = [], []
    blank = np.zeros((px, px, 3), np.uint8)
    y_head = 0.945
    for j, (ax, name) in enumerate(zip(axes, args.policies)):
        pos = ax.get_position()
        fig.text(pos.x0 + pos.width / 2, y_head, name, ha="center",
                 va="center", fontsize=12.5, color=COLORS[j],
                 fontweight="bold")
        ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_edgecolor(COLORS[j]); sp.set_linewidth(2.0)
        ims.append(ax.imshow(blank))
        notes.append(ax.text(0.5, 0.035, "", transform=ax.transAxes,
                             ha="center", fontsize=12, fontweight="bold",
                             color="white",
                             bbox=dict(boxstyle="round,pad=0.3",
                                       fc="#bb2222", ec="none")))

    banner = fig.text(0.5, y_head, "", ha="center", va="center", fontsize=19,
                      fontweight="bold", color="white",
                      bbox=dict(boxstyle="round,pad=0.45", fc="#333333",
                                ec="none"))
    sub = fig.text(0.5, 0.022, "", ha="center", va="center", fontsize=9.5,
                   color="0.45")

    n_frames = int(args.seconds_per_move * args.fps)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"{args.policies[0]}_vs_{args.policies[1]}{tag}.mp4"
    writer = imageio.get_writer(out, fps=args.fps, macro_block_size=1)
    print(f"pass 2: rendering {len(cmds) * n_frames} frames")

    # SAME ORDER AS THE BAR CHARTS -- grouped by family, cruise split
    # forward/reverse and turn_big in place/moving, mirrored pairs adjacent.
    # Imported rather than restated, so the video and the figures cannot drift
    # into different orders for the same 20 commands.
    order, labels, spans = order_and_labels(cmds)
    fam_of = {}
    for name, a_, b_ in spans:
        for i in range(a_, b_):
            fam_of[i] = name.replace("\n", " / ")

    for pos, k in enumerate(order):
        lab = labels[pos]
        banner.set_text(f"#{k}   {lab}")
        sub.set_text(f"{fam_of[pos]}   |   {pos + 1} of {len(order)}   |   "
                     f"first {args.seconds_per_move:g} s, real time   |   "
                     f"dial: green/cyan = commanded/actual heading, "
                     f"orange/yellow = velocity")
        eps = [r[k] for r in runs]
        # ONE distance for both panels, from whichever policy covers more
        # ground, so the two paths are comparable rather than each auto-zoomed.
        # Teleop's own overhead: FOLLOW the bike at a fixed distance, so it
        # stays the same readable size on every command. Framing the whole
        # path instead was tried and is worse -- a 3 m run shrinks the bike to
        # a smudge, and the fixed frame runs off the 6 m floor. The pen trail
        # and the moving grid carry the displacement instead. Azimuth is held
        # world-fixed rather than teleop's `= yaw`, because a view that
        # rotates with the bike makes two panels impossible to compare.
        for note, e in zip(notes, eps):
            note.set_text("" if not e["fell"] or
                          e["t_end"] <= args.seconds_per_move else
                          f"falls at {e['t_end']:.1f} s, after this clip")

        for i in range(n_frames):
            t = i / args.fps
            for im, note, e in zip(ims, notes, eps):
                n = len(e["qpos"])
                j_ = min(n - 1, int(round(t / e["dt"])))
                data.qpos[:] = e["qpos"][j_]
                data.qvel[:] = 0.0
                # data.time must be the time of the STEP being drawn, not
                # of the frame. `_overlay` treats `now < trail[-1][0]` as a
                # viewer rewind and CLEARS the pen -- and j_ rounds up, so
                # frame time is behind the last trail point about two frames
                # in three at 30 fps against a 50 Hz control rate. That was
                # the flicker, identical in both panels because both index
                # the same way.
                data.time = j_ * e["dt"]
                mujoco.mj_forward(model, data)
                cam.lookat[:] = [data.qpos[0], data.qpos[1], 0.0]
                renderer.update_scene(data, camera=cam)
                # The teleop overlay itself -- pen trail, floor grid, and the
                # command dial: green tick commanded heading, cyan actual,
                # orange ray commanded velocity, yellow actual. reset=False
                # appends onto the model geoms already in the scene, as
                # aow_sim.record does. c=None is safe ONLY because `command`
                # is supplied; without it the overlay reads a live controller.
                trail = [(m * e["dt"], float(q[0]), float(q[1]))
                         for m, q in enumerate(e["qpos"][:j_ + 1])]
                _overlay(renderer.scene, model, data, None, [True], v_max,
                         reset=False, command=(e["psi_cmd"], e["v_cmd_w"]),
                         trail=trail, grid=True,
                         trail_level=args.seconds_per_move)
                im.set_data(renderer.render())
                if e["fell"] and e["t_end"] <= args.seconds_per_move \
                        and j_ >= n - 1:
                    note.set_text(f"FELL at {e['t_end']:.1f} s")
            fig.canvas.draw()
            writer.append_data(np.asarray(fig.canvas.buffer_rgba())[..., :3])
        print(f"  {k:>2}  {lab}", flush=True)

    writer.close()
    plt.close(fig)
    print(f"\nwrote {out}   "
          f"({len(cmds) * args.seconds_per_move:.0f} s at {args.fps} fps)")


if __name__ == "__main__":
    main()
