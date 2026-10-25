"""wheel_slowmo.py's rear-wheel clip, for one policy on two drivetrains.

    python analysis/policy/drivetrain_slowmo.py \
        --new traces/drivetrain/drivetrain_sawtooth_1024.yaml
    python analysis/policy/drivetrain_slowmo.py --new <overlay> \
        --policy general_puffer_rake20_80M --start 6 --seconds 0.6 --slowmo 40
    python analysis/policy/drivetrain_slowmo.py --new <overlay> --parts detent \
        --set sim.contact_solref='[0.004, 0.3]'          # both rows

EVERYTHING ON SCREEN IS analysis/contact/wheel_slowmo.py's, imported, not
copied: the waterline camera on z=0 with the floor drawn as a line and the
sub-floor tinted, the side / rear / magnified contact panels with their
px/mm captions, the ghosted bike and lightened rollers, the roller stripes
with the loaded roller hot, the world-fixed ground ticks that make slip
visible, the HUD (signed clearance, contact force or AIRBORNE, roll, v_lat,
the action bars, the clearance history on a scale shared by both rows),
and the viability table printed at the end. Read that module's docstring
for why each is the way it is.

WHAT DIFFERS is the flight. wheel_slowmo flies a CPU policy through a
re-created GeneralEnv step on the ideal drive; that loader predates the
MinGRU puffer policies, and the env step has no drivetrain hook. Here the
flight is run_vs_sim.sim() itself (its `on_step` hook), the floor twin the
spectra come from: the policy's own training drivetrain record, the Pi's
counts steer rate, its AHRS / odometry levels, headset friction, a zero
command. Row 1 is that; row 2 swaps in `--new`'s `detent` and `roller_slop`
blocks (`--parts` picks which), servo gains and the rest staying the
policy's. Nothing is re-implemented, so there is no replication to verify.

Added under the two rows: roll rate and ring-vs-hub angle (mod one 7.5 deg
notch) of both runs, with a cursor -- the detent's own coordinate.

Same seed both rows, but NOT the same trajectory: a different plant
diverges within a step. Compare behaviour, not frames.

Writes traces/drivetrain_slowmo/<policy><tag>.mp4 (traces/ is gitignored).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "contact"))
import run_vs_sim as R  # noqa: E402
import wheel_slowmo as W  # noqa: E402

from aow_sim import drivetrain_model as dm  # noqa: E402
from aow_sim.build_model import load_params  # noqa: E402
from aow_sim.wheel_overlay import stripe_frames  # noqa: E402

from PIL import Image, ImageDraw  # noqa: E402  (wheel_slowmo checked both)
import imageio.v2 as imageio  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
COLORS = [(110, 170, 240), (245, 120, 110)]      # row 1, row 2 in the traces


class Recorder:
    """on_step hook: wheel_slowmo's per-frame sample on a sim-time clock from
    `start`, plus a dense roll-rate / ring-vs-hub trace for the strip."""

    def __init__(self, start, seconds, frame_dt):
        self.t0, self.t1, self.frame_dt = start, start + seconds, frame_dt
        self.next_t = start
        self.frames, self.trace = [], []
        self.model = None

    def _setup(self, model, data):
        self.model = model
        names = [model.geom(i).name for i in range(model.ngeom)]
        self.floor = names.index("floor")
        self.rear = {i for i, n in enumerate(names) if n.startswith("roller_")}
        self.verts = W.wheel_vertices(model, self.rear)
        self.weight = float(model.body_subtreemass[model.body("chassis").id] * 9.81)
        self.buf = np.zeros(6)
        self.ring = (model.joint("input_a_spin").qposadr[0],
                     model.joint("input_b_spin").qposadr[0])
        p = load_params()["drivetrain"]
        self.g = (float(p["mix_ring_a"]) - float(p["mix_hub_a"]),
                  float(p["mix_ring_b"]) - float(p["mix_hub_b"]))

    def __call__(self, model, data, ctl, drive=None):
        if self.model is None:
            self._setup(model, data)
        t = data.time
        if self.t0 - 0.25 <= t < self.t1:
            phi = math.degrees(self.g[0] * data.qpos[self.ring[0]]
                               + self.g[1] * data.qpos[self.ring[1]])
            self.trace.append((t, math.degrees(data.qvel[3]), phi))
        if t < self.next_t or t >= self.t1:
            return
        self.next_t += self.frame_dt
        # _p0 only sets the position error, which the HUD does not show
        env = SimpleNamespace(model=model, data=data, _p0=data.qpos[:2].copy())
        act = np.asarray(ctl._gen_prev_a, float)[:3]
        self.frames.append(W._sample(env, act, self.rear, self.floor,
                                     self.weight, self.buf, self.verts))


def fly(policy, overrides, start, seconds, frame_dt, seed):
    rec = Recorder(start, seconds, frame_dt)
    R.sim(policy, start + seconds + 0.02, seed, overrides, on_step=rec)
    return rec


def _strip(recs, t_now, width, font):
    """Roll rate and ring-vs-hub mod one notch for both rows, with a cursor."""
    h = 230
    im = Image.new("RGB", (width, h), (14, 16, 20))
    d = ImageDraw.Draw(im)
    ta, tb = recs[0].t0, recs[0].t1
    roll = np.concatenate([np.array(r.trace)[:, 1] for r in recs])
    rlim = max(30.0, float(np.percentile(np.abs(roll), 99.5)))
    boxes = [((12, 6, width - 12, 108), 1, -rlim, rlim, f"roll rate, +-{rlim:.0f} deg/s"),
             ((12, 118, width - 12, 222), 2, 0.0, 7.5, "ring-vs-hub mod 7.5 deg (one notch)")]
    for (x0, y0, x1, y1), key, lo, hi, label in boxes:
        d.rectangle((x0, y0, x1, y1), fill=(20, 24, 32), outline=(60, 66, 80))

        def xy(t, v):
            return (x0 + (t - ta) / (tb - ta) * (x1 - x0),
                    y1 - (np.clip(v, lo, hi) - lo) / (hi - lo) * (y1 - y0))
        if lo < 0 < hi:
            d.line([xy(ta, 0), xy(tb, 0)], fill=(60, 66, 80))
        for rec, col in zip(recs, COLORS):
            tr = np.array(rec.trace)
            m = (tr[:, 0] >= ta) & (tr[:, 0] <= tb)
            tt, vv = tr[m, 0], tr[m, key]
            if key == 2:
                vv = vv % 7.5
            step = max(1, len(tt) // 2000)
            pts = [xy(a, b) for a, b in zip(tt[::step], vv[::step])]
            for a, b in zip(pts[:-1], pts[1:]):     # no line across the wrap
                if key != 2 or abs(a[1] - b[1]) < (y1 - y0) / 2:
                    d.line([a, b], fill=col, width=2)
        tw = d.textlength(label, font=font)
        d.rectangle((x0 + 2, y0 + 2, x0 + 10 + tw, y0 + 20), fill=(20, 24, 32))
        d.text((x0 + 6, y0 + 3), label, fill=(150, 158, 172), font=font)
        cx = xy(t_now, 0)[0]
        d.line([(cx, y0), (cx, y1)], fill=(245, 160, 90), width=1)
    return np.asarray(im)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--new", type=Path, required=True, metavar="OVERLAY",
                    help="a whole drivetrain_model file (drivetrain_model.load)")
    ap.add_argument("--parts", nargs="+", default=["detent", "roller_slop"],
                    choices=dm.PARTS, help="which blocks of --new go into row 2")
    ap.add_argument("--policy", default=None,
                    help="default: control.general_move in bike_params.yaml")
    ap.add_argument("--set", dest="overrides", action="append", default=[],
                    metavar="PATH=VALUE",
                    help="override a sim parameter in BOTH rows, as run_vs_sim "
                         "--set (sim.contact_solref, sim.timestep, ...)")
    ap.add_argument("--start", type=float, default=4.0,
                    help="sim s flown before recording (the policy settles)")
    ap.add_argument("--seconds", type=float, default=0.8,
                    help="SIMULATED seconds recorded (video = this x slowmo)")
    ap.add_argument("--slowmo", type=float, default=40.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--views", nargs="*", default=list(W.VIEWS), choices=list(W.VIEWS))
    ap.add_argument("--above", type=float, default=None,
                    help="metres shown above the ground line, side/rear panels")
    ap.add_argument("--below", type=float, default=None,
                    help="metres shown below the ground line, side/rear panels")
    ap.add_argument("--contact-mm", type=float, default=None,
                    help="half-span of the magnified contact panel [mm]")
    ap.add_argument("--width", type=int, default=1500, help="output width [px]")
    ap.add_argument("--no-video", action="store_true",
                    help="flights and the viability table only")
    ap.add_argument("--tag", default="")
    ap.add_argument("--out-dir", type=Path, default=REPO / "traces" / "drivetrain_slowmo")
    a = ap.parse_args()

    import yaml
    policy = a.policy or load_params()["control"]["general_move"]
    shared = []
    for o in a.overrides:
        path, eq, value = o.partition("=")
        if not eq:
            ap.error(f"--set wants PATH=VALUE, got {o!r}")
        shared.append((path, value))
    new = dm.load(a.new)
    swap = [(f"{dm.KEY}.{part}", yaml.safe_dump(new[part])) for part in a.parts]
    rows = [("its training drivetrain", shared),
            (f"{'+'.join(a.parts)} from {a.new.stem}", shared + swap)]

    frame_dt = 1.0 / (W.FPS * a.slowmo)
    recs = []
    for label, ov in rows:
        rec = fly(policy, ov, a.start, a.seconds, frame_dt, a.seed)
        dt = rec.model.opt.timestep
        if frame_dt < dt:           # wheel_slowmo's floor: one frame per step
            raise SystemExit(f"--slowmo {a.slowmo:g} needs {frame_dt:.2e} s per "
                             f"frame, under the {dt:.2e} s timestep; at most "
                             f"{1 / (W.FPS * dt):.0f}x")
        recs.append(rec)
    m0 = recs[0].model
    tc = float(m0.geom_solref[0, 0])
    print(f"{policy}: {a.seconds:g} s from t={a.start:g} s, {a.slowmo:g}x slow at "
          f"{W.FPS} fps -> {a.seconds * a.slowmo:.0f} s of video; contact solref "
          f"{np.round(m0.geom_solref[0], 5).tolist()}, timestep {m0.opt.timestep:g}"
          + (f", refsafe margin 2*dt/timeconst {2 * m0.opt.timestep / tc:.2f}"
             if tc > 0 else ""))

    params = load_params()
    roller_r_mm = params["omni_wheel"]["roller"]["big_diameter"] / 2 * 1e3
    table = {f"{policy}, {lab}": W.viability(r.frames, a.seconds, roller_r_mm)
             for (lab, _), r in zip(rows, recs)}

    if not a.no_video:
        panel_h, panel_w = 560, 620
        rens, framings = [], []
        for rec in recs:
            m = rec.model           # done stepping: free to restyle for render
            W.ghost_everything_but_the_wheel(m)
            m.vis.global_.offwidth = max(panel_w, m.vis.global_.offwidth)
            m.vis.global_.offheight = max(panel_h, m.vis.global_.offheight)
            m.vis.headlight.ambient[:] = 0.62
            m.vis.headlight.diffuse[:] = 0.55
            m.vis.headlight.specular[:] = 0.0
            rens.append((mujoco.Renderer(m, panel_h, panel_w), mujoco.MjData(m),
                         stripe_frames(m, params)))
            fr = {}
            for v in a.views:
                spec = dict(W.VIEWS[v])
                if v == "contact" and a.contact_mm:
                    spec["above"], spec["below"] = a.contact_mm * 1e-3, a.contact_mm * 5e-4
                elif v != "contact":
                    spec["above"] = a.above or spec["above"]
                    spec["below"] = a.below or spec["below"]
                fr[v] = W.framing(m, spec["above"], spec["below"], panel_h)
            framings.append(fr)
        for v in a.views:
            fovy, _k, ppmm, _b = framings[0][v]
            print(f"  {v:8} fovy {fovy:5.1f} deg at {W.STANDOFF * 100:.0f} cm, {ppmm:.1f} px/mm")
        font, font_sm = W._font(19), W._font(14)
        # The HUD's name line is one line wide before the clearance column.
        short = policy.removeprefix("general_puffer_").removeprefix("general_rl_")
        huds = ["old drivetrain", "new: " + a.new.stem.removeprefix("drivetrain_")]
        clears = [[f["clear_mm"] for f in r.frames] for r in recs]
        span = max(0.05, max(abs(v) for c in clears for v in c))   # shared scale
        out = a.out_dir / f"{policy}{a.tag}.mp4"
        out.parent.mkdir(parents=True, exist_ok=True)
        n_out = min(len(r.frames) for r in recs)
        with imageio.get_writer(out, fps=W.FPS, macro_block_size=1, quality=8) as wr:
            for n in range(n_out):
                bands = []
                for hud, rec, (ren, d, stripes), fr, c in zip(
                        huds, recs, rens, framings, clears):
                    f = rec.frames[n]
                    d.qpos[:], d.qvel[:] = f["qpos"], f["qvel"]
                    mujoco.mj_forward(rec.model, d)
                    panels = [W.render_panel(ren, rec.model, d, v, f, stripes, fr, font_sm)
                              for v in a.views]
                    bands.append(W.compose(panels, f, f"{short}  |  {hud}", a.slowmo,
                                           c[:n + 1], font, font_sm, a.width, span=span))
                w = min(b.shape[1] for b in bands)
                strip = _strip(recs, recs[0].frames[n]["t"], w, font_sm)
                frame = np.vstack([b[:, :w] for b in bands] + [strip])
                h, w = frame.shape[:2]
                wr.append_data(frame[:h - h % 2, :w - w % 2])
        print(f"  wrote {out}  ({n_out} frames)")

    print(f"\nwhat the hardware would have to survive "
          f"({a.seconds:.2f} s standing, roller radius {roller_r_mm:.1f} mm)")
    kw = max([26] + [len(k) + 2 for k in table])
    print(f"{'run':{kw}}{'pen pk':>8}{'of roller':>11}{'force pk':>10}"
          f"{'airborne':>10}{'landings':>10}{'roller':>9}{'hub':>8}")
    print(f"{'':{kw}}{'[mm]':>8}{'[%]':>11}{'[x weight]':>10}{'[%]':>10}"
          f"{'[/s]':>10}{'[rpm]':>9}{'[rpm]':>8}")
    for key, r in table.items():
        print(f"{key:{kw}}{r['pen_peak']:>8.2f}{100 * r['pen_frac']:>11.1f}"
              f"{r['force_peak']:>10.2f}{100 * r['air_frac']:>10.0f}"
              f"{r['landings_hz']:>10.1f}{r['roller_rpm']:>9.0f}"
              f"{r['hub_rpm']:>8.0f}")


if __name__ == "__main__":
    main()
