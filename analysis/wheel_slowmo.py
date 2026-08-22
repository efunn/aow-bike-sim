"""Giga-slow-motion detail shot of the rear omni wheel holding station.

WHAT THIS IS FOR. `hold` asks the bike to stand still, and every general_rl
policy answers by buzzing the wheel against the floor instead (see
analysis/hold_spectrum.py and analysis/chatter.py: the rear wheel is airborne
27-48% of the time and the differential channel chatters at 10-15 Hz). Those
scripts measure it. This one SHOWS it, at the timescale it actually happens
on, so the motion can be judged against a real wheel that has to survive it.

  python analysis/wheel_slowmo.py                          # every policy
  python analysis/wheel_slowmo.py --policies general_rl_smooth_diff_og
  python analysis/wheel_slowmo.py --seconds 0.5 --slowmo 83 --width 900

Writes traces/wheel_slowmo/<policy><tag>.mp4 -- traces/ is gitignored, and at
~7 MB a clip these do not belong beside the committed PNGs in analysis/.
Read-only otherwise: it loads moves/*.npz and touches nothing else.

THE WATERLINE. MuJoCo's contacts are soft, so the rollers really do sink
below z=0 -- that penetration is not a rendering artefact, it IS the contact
model, and hiding it would hide the thing most worth looking at. So the floor
geom is made invisible and the ground is drawn as a line instead, with the
sub-floor half of the frame tinted: the wheel is rendered whole and crosses
the line like a hull crossing a waterline.

The line is exact, not eyeballed. A perspective camera placed exactly ON the
plane z=0 and pointed horizontally (elevation 0) has that plane passing
through its optical centre, so every point of z=0 -- at any depth, any
distance -- projects onto the single row through the principal point. The
camera therefore tracks the wheel in x and y but never in z, and `elevation`
is deliberately not exposed as a flag: tilt the camera and the ground stops
being a line and becomes a region below a horizon, and the readout is a lie.
Azimuth is free, because rotating within the plane keeps the camera in it.

THREE VIEWS, because no one of them is enough for this wheel:

  side     camera along the axle (+Y), the wheel as a disc. Shows wheel spin,
           the bob of the whole bike, and the gap at the contact.
  rear     camera along the bike's -X, the wheel edge-on. This is the one that
           shows LATERAL crawl -- the roller axes at the bottom of the wheel
           point along x, so a contact roller spinning is an in-plane rotation
           here and a straight-at-you rotation in the side view.
  contact  the same camera at 13x the magnification. Penetration runs a few
           tenths of a millimetre and the wide framing puts one millimetre at
           2.4 px, so the quantity the whole video exists to show is invisible
           there; at 31 px/mm it is not.

All three put z=0 on the same row, so the ground line runs unbroken across the
strip even though the panels are at different scales. Zoom is done by
narrowing the field of view at a fixed 45 cm standoff, not by moving in: 9 mm
of framing would otherwise put the camera inside the wheel.

ROLLER STRIPES. The rollers are smooth truncated cones; nothing about their
own rotation is visible on an unmarked surface, and their rotation is exactly
what turns into sideways motion. Four stripes are drawn every 90 deg around
each cone, following the cone taper (the surface goes 11.0 mm radius at the
inner end to 9.5 mm at the outer). The pair whose roller is currently touching
the floor is drawn hot, so "which roller is carrying the bike" is readable
frame by frame.

SUBSTEP RESOLUTION. The control loop runs at 50 Hz but the physics runs at
2.5 kHz, and everything interesting here lives between control ticks. So the
rollout re-creates GeneralEnv.step's actuator write and then single-steps the
physics itself. That duplication is checked rather than trusted:
`_verify_replication` asserts this loop reproduces GeneralEnv.step's qpos to
machine precision before any frame is rendered. The env is never modified --
as with the rest of analysis/, no trained policy can be affected by a
measurement.

PLAYBACK RATE IS A REQUEST, NOT AN OUTCOME. Frames are sampled on a SIM-TIME
clock, so `--slowmo` is honoured exactly at any timestep. (It used to sample
every N physics steps, which tied the rate to the timestep: one `--slowmo 83`
came out at 83x under a 2e-4 step and 5.6x under 3e-3, and the two clips could
not be compared.) The one hard floor is the timestep itself -- no run emits
frames faster than it integrates -- and asking for more says so and caps.

  python analysis/wheel_slowmo.py --slowmo 1 --seconds 4     # real time
  python analysis/wheel_slowmo.py --compare 4e-4,3e-3 --slowmo 5

`--compare` stacks one run per timestep into a single video on that shared
clock, with a shared clearance scale. They are NOT the same trajectory: same
policy, same seed, but a different timestep diverges within a few steps
because the hold is a buzz, not a fixed point. Read a stacked clip as two
samples of the same behaviour, never as one drifting from the other.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.balance import extract_state, mix
from aow_sim.control.flick import MOVES_DIR
import aow_sim.control.general_env as ge
from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.general_spec import scale_action
from aow_sim.control.policy import load_policy_npz
# load_general, NOT load_policy_npz: the latter reads only the weights, so the
# result has no vel_window_s / obs_pitch and `policy_flags` sees an empty set --
# which silently builds a 15-wide env for a 19-wide policy. See load_general.
from rsa_policies import POLICIES, REPO, load_general

try:
    import imageio.v2 as imageio
except ImportError as e:
    raise SystemExit("needs imageio + ffmpeg plugin: pip install -e '.[viz]'") from e
try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as e:
    raise SystemExit("needs pillow: pip install pillow") from e

FPS = 60                      # output frame rate; slowmo is relative to this
N_AXLES = 8
STRIPES = 4                   # per cone, every 90 deg

# Colours (RGBA, 0-1). Kept dark-on-light so the tinted sub-floor half reads
# as "below ground" rather than as a shadow.
C_STRIPE = (0.96, 0.97, 1.00, 1.0)   # bright: the rollers themselves are dark
C_HOT = (1.00, 0.45, 0.10, 1.0)      # the roller currently on the floor
C_GHOST = 0.13                        # alpha for everything that is not the wheel


# -- rollout ---------------------------------------------------------------

def _apply_control(env, action):
    """GeneralEnv.step's actuator write, without the substep loop.

    Mirrors control/general_env.py step() exactly; `_verify_replication`
    below is what keeps the two from drifting apart silently.
    """
    action = np.asarray(action, np.float32)
    steer_rate, hub, diff = scale_action(action, env.bounds)
    env._steer += steer_rate * env.ctrl_dt
    a, b = mix(hub / env._r_rear, diff)
    env.data.ctrl[env._aid["drive_a"]] = a
    env.data.ctrl[env._aid["drive_b"]] = b
    env.data.ctrl[env._aid["steer"]] = env._steer
    env.data.xfrc_applied[env._chassis, :] = 0.0


def drive_lag(base_params, tau):
    """A `build_model` that gives the drive actuators a first-order lag.

    Has to be done on the SPEC, before compile: MuJoCo allocates the actuator
    activation state (`na`) at compile time, so setting `actuator_dyntype` on a
    live model changes nothing and fails silently.
    """
    from aow_sim.build_model import build_spec

    def f(params=None, variant="full", training_wheels=False, hockey=False,
          payload=True, righting=False, wings=False):
        spec = build_spec(params or base_params, variant, training_wheels,
                          hockey, payload, righting, wings)
        for a in spec.actuators:
            if a.name.startswith("drive_"):
                a.dyntype = mujoco.mjtDyn.mjDYN_FILTEREXACT
                a.dynprm[0] = tau
        return spec.compile()
    return f


def override_solref(model, timeconst, dampratio):
    """Retune the contact on an already-compiled model.

    Sweeping `dampratio` is the whole point of this knob: the shipped 1.0 is
    CRITICAL damping and cannot bounce, while the physical wheel audibly
    bounces two or three times, so the clips at 1.0 are showing a floor that
    absorbs every impact. Overriding here rather than editing
    config/bike_params.yaml keeps the sweep from disturbing a training run
    that is reading the config.
    """
    if timeconst is not None:
        model.geom_solref[:, 0] = timeconst
    if dampratio is not None:
        model.geom_solref[:, 1] = dampratio


def _verify_replication(params, cfg, pol, solref=(None, None), n_ctrl=5,
                        pol_env=False):
    """Assert the hand-rolled substep loop equals GeneralEnv.step().

    Cheap insurance against the duplication in `_apply_control` rotting: if
    the env's control law changes and this file does not, the numbers and the
    video would quietly stop being of the same policy.
    """
    if pol_env:                     # match the env to the policy's obs blocks
        from aow_sim.control.general_spec import policy_flags
        cfg = {**cfg, "env": {**cfg["env"], **dict(
            policy_flags(pol),
            act_wings=bool(getattr(pol, "act_wings", False)),
            wing_max_deg=float(getattr(pol, "wing_max_deg", 90.0)))}}
    ref, mine = GeneralEnv(params, cfg), GeneralEnv(params, cfg)
    for e in (ref, mine):                 # validate under the physics in use
        override_solref(e.model, *solref)
    o_r, _ = ref.reset(seed=7, options=_HOLD)
    o_m, _ = mine.reset(seed=7, options=_HOLD)
    for _ in range(n_ctrl):
        a = _norm_action(pol, o_r)
        o_r, *_ = ref.step(a)
        _apply_control(mine, a)
        for _ in range(mine.substeps):
            mujoco.mj_step(mine.model, mine.data)
        o_m, *_ = mine._obs()
    err = float(np.abs(ref.data.qpos - mine.data.qpos).max())
    assert err < 1e-12, (
        f"substep replication drifted from GeneralEnv.step by {err:.2e} -- "
        "_apply_control is out of date with control/general_env.py step()")
    return err


_HOLD = {"v_cmd": (0.0, 0.0), "psi_cmd_rel": 0.0, "difficulty": 1.0}


def _norm_action(pol, obs):
    """Physical action -> fraction of each channel's bound.

    A bound of 0 DISABLES that channel (action_bounds.hub_max: 0.0 is how the
    hold-without-the-hub diagnostic pins the wheel), and 0/0 is NaN, which
    reaches the HUD as `int(NaN)` and kills the render several hundred frames
    in. A disabled channel is already 0 in physical units, so dividing it by 1
    leaves it 0 -- which is exactly what the bar should draw.
    """
    b = pol.bounds
    scale = np.array([b.steer_rate_max, b.hub_max, b.diff_max])
    return np.asarray(pol.action(obs), float) / np.where(scale > 0, scale, 1.0)


def rollout(pol, env, seconds, frame_dt, rear_geoms, floor):
    """Hold command, sampled on a SIM-TIME clock: one frame per `frame_dt`.

    Not "every N physics steps". A stride ties the playback rate to the
    timestep -- 83x at 2e-4 and 5.6x at 3e-3 for the same request -- so two
    runs at different timesteps could not be played against each other, which
    is the whole point of `--compare`. Sampling on sim time makes the output
    rate exactly what was asked for at any timestep, at the cost of the sample
    landing on the first physics step at or after each frame time (within one
    timestep, i.e. under half a pixel of wheel travel here).

    `frame_dt` must be >= the timestep; the caller clamps and says so.

    Returns a list of frames; each carries the full simulator state needed to
    render it plus the scalars the HUD reads, so rendering never re-simulates.
    """
    obs, _ = env.reset(seed=7, options=_HOLD)
    weight = float(env.model.body_subtreemass[env.model.body("chassis").id] * 9.81)
    verts = wheel_vertices(env.model, rear_geoms)
    dt = env.model.opt.timestep
    n_ctrl = int(round(seconds / env.ctrl_dt))
    out, buf = [], np.zeros(6)
    action = np.zeros(3)
    k, next_t = 0, 0.0
    for _ in range(n_ctrl):
        action = _norm_action(pol, obs)
        _apply_control(env, action)
        for _ in range(env.substeps):
            mujoco.mj_step(env.model, env.data)
            k += 1
            if k * dt >= next_t:
                out.append(_sample(env, action, rear_geoms, floor, weight,
                                   buf, verts))
                next_t += frame_dt
        obs, *_ = env._obs()
    return out


def _sample(env, action, rear_geoms, floor, weight, buf, verts):
    """One frame's worth of state. `pen` is the deepest roller penetration in
    mm -- with soft contacts that is the honest measure of how hard the wheel
    is being driven into the floor, and it is the number a real wheel would
    have to answer for."""
    m, d = env.model, env.data
    pen, force, hot = 0.0, 0.0, set()
    for i in range(d.ncon):
        c = d.contact[i]
        if floor not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == floor else c.geom1
        if other not in rear_geoms:
            continue
        mujoco.mj_contactForce(m, d, i, buf)
        force += float(buf[0])
        pen = max(pen, -float(c.dist))
        hot.add(int(m.geom_bodyid[other]))
    s = extract_state(d, env._p0)
    roller = np.abs([d.qvel[m.joint(f"roller_spin_{i}").dofadr[0]]
                     for i in range(N_AXLES)])
    return {
        # Signed, so the flight half of the cycle is a quantity and not just
        # the absence of one. Negative = sunk in, positive = off the floor.
        "clear_mm": clearance_mm(d, verts),
        "roller_rpm": float(roller.max() * 60 / (2 * np.pi)),
        "hub_rpm": float(abs(d.qvel[m.joint("hub_spin").dofadr[0]])
                         * 60 / (2 * np.pi)),
        "qpos": d.qpos.copy(), "qvel": d.qvel.copy(), "t": float(d.time),
        "act": np.asarray(action, float).copy(),
        "pen_mm": pen * 1e3, "force_w": force / weight, "hot": hot,
        "roll_deg": float(np.degrees(s.roll)),
        "v_lat": float(s.v_lat), "v_lon": float(s.v_lon),
        "airborne": force <= 1e-9,
    }


def wheel_vertices(model, rear_geoms):
    """Per-geom (mesh vertex array, geom id) for every rear-wheel geom.

    Used to get SIGNED clearance. Contacts cannot give it: MuJoCo only creates
    a contact once the geoms overlap, so `contact.dist` reports penetration and
    then simply stops existing when the wheel lifts -- the flight half of the
    cycle is exactly the half a contact-based measure is blind to. (`margin`
    would extend detection, but it is a model field, and changing the model to
    measure it is the one thing analysis/ does not do.)

    The rollers are convex meshes, so the lowest point of the collision
    geometry IS a vertex of the hull, and min z over the vertices is not an
    approximation of the clearance -- it is the same quantity MuJoCo's
    collision would find, computed without touching the model.
    """
    out = []
    for g in sorted(rear_geoms):
        did = int(model.geom_dataid[g])
        if did < 0:                       # not a mesh; skip (none today)
            continue
        a = int(model.mesh_vertadr[did])
        n = int(model.mesh_vertnum[did])
        out.append((g, model.mesh_vert[a:a + n].astype(float)))
    return out


def clearance_mm(data, verts):
    """Signed wheel-to-floor clearance [mm]: negative = sunk into the floor,
    positive = flying. The floor is the plane z=0, so this is just the lowest
    vertex."""
    lo = min(float((data.geom_xpos[g]
                    + v @ data.geom_xmat[g].reshape(3, 3).T)[:, 2].min())
             for g, v in verts)
    return lo * 1e3


# -- scene decoration ------------------------------------------------------

def stripe_frames(model, p):
    """Where the four stripes sit on each cone, in axle-body coordinates.

    Each roller axle body spins about its own joint axis, so a point fixed in
    that body's frame rides the roller. The stripes follow the cone taper
    (big end inboard) rather than sitting at one radius, so they lie ON the
    surface instead of floating off the small end.
    """
    r = p["omni_wheel"]["roller"]          # load_params has already resolved
    r_big = r["big_diameter"] / 2          #   the {value, source} leaves
    r_small = r["small_diameter"] / 2
    length, gap = r["length"], r["pair_gap"]
    eps = 3e-4                                   # lift clear of the surface
    out = []
    for i in range(N_AXLES):
        axis = np.array(model.joint(f"roller_spin_{i}").axis, float)
        axis /= np.linalg.norm(axis)
        # any two unit vectors spanning the plane normal to the axle
        tmp = np.array([0.0, 1.0, 0.0])
        if abs(tmp @ axis) > 0.9:
            tmp = np.array([1.0, 0.0, 0.0])
        u = np.cross(axis, tmp); u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        segs = []
        for sign in (+1, -1):                    # the two cones of the pair
            z_in, z_out = sign * gap / 2, sign * (gap / 2 + length)
            for k in range(STRIPES):
                th = 2 * np.pi * k / STRIPES
                dirv = np.cos(th) * u + np.sin(th) * v
                segs.append(((r_big + eps) * dirv + z_in * axis,
                             (r_small + eps) * dirv + z_out * axis))
        out.append((model.body(f"roller_axle_{i}").id, segs))
    return out


def add_stripes(scene, data, frames, hot):
    for bid, segs in frames:
        pos, R = data.xpos[bid], data.xmat[bid].reshape(3, 3)
        rgba = np.asarray(C_HOT if bid in hot else C_STRIPE, np.float32)
        for p0, p1 in segs:
            if scene.ngeom >= scene.maxgeom:
                return
            g = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                                np.zeros(3), np.zeros(9), rgba)
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 6e-4,
                                 pos + R @ p0, pos + R @ p1)
            scene.ngeom += 1


def add_ground_ticks(scene, view, lookat, span_mm, below_m):
    """A ruler painted on the floor, in WORLD coordinates.

    The camera tracks the wheel, so anything fixed to the ground scrolls
    through frame. That is the point: the roller stripes say how much the
    wheel turned and these ticks say how far the ground actually went past, so
    a wheel spinning against stationary ticks is slip, visible directly rather
    than inferred from the hub-revolutions-vs-travel arithmetic in
    hold_spectrum.py.

    Drawn hanging BELOW z=0 into the tinted half, so they never compete with
    the ground line itself.
    """
    v = VIEWS[view]
    ax, step = v["axis"], v["tick_mm"] * 1e-3
    near = np.asarray(v["near"], float)
    centre = float(lookat[ax])
    n = int(span_mm / v["tick_mm"] / 2) + 2
    k0 = int(round(centre / step))
    # Tick weight is keyed to how much of the sub-floor the panel shows, so a
    # tick occupies the same fraction of the frame in every panel. Keying it to
    # the SPACING instead leaves the contact panel's marks 10 px tall, and a
    # fixed size makes them a hairline in `side` and a fence post in `contact`
    # -- the two are 16x apart in scale.
    short, radius = below_m * 0.35, below_m * 0.02
    for k in range(k0 - n, k0 + n + 1):
        if scene.ngeom >= scene.maxgeom:
            return
        # every 4th tick is longer, so the spacing is countable while moving
        depth = short * (1.7 if k % 4 == 0 else 1.0)
        p0 = np.array(lookat, float) + near
        p0[ax] = k * step
        p0[2] = 0.0
        p1 = p0.copy()
        p1[2] = -depth
        g = scene.geoms[scene.ngeom]
        rgba = np.asarray((0.55, 0.80, 1.0, 1.0) if k % 4 == 0 else
                          (0.36, 0.52, 0.68, 1.0), np.float32)
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                            np.zeros(3), np.zeros(9), rgba)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, p0, p1)
        scene.ngeom += 1


def ghost_everything_but_the_wheel(model, keep_prefixes=("roller_", "ring_body",
                                                         "hub_body")):
    """Fade the chassis, fork and front wheel, and lift the rollers out of the
    dark.

    The rollers ship near-black, which is fine in a wide shot and useless in
    the contact panel: the silhouette against the tinted sub-floor is the
    whole measurement there, and a black shape on a dark ground has no edge to
    read. Everything that is not the wheel is kept faintly rather than hidden,
    so the wheel is still read as part of a bike.
    """
    for i in range(model.ngeom):
        name = model.geom(i).name
        if name == "floor":
            model.geom_rgba[i, 3] = 0.0          # replaced by the drawn line
        elif name.startswith("roller_"):
            model.geom_rgba[i] = (0.62, 0.65, 0.70, 1.0)
        elif name.startswith(("ring_body", "hub_body")):
            model.geom_rgba[i] = (0.34, 0.37, 0.43, 1.0)
        else:
            model.geom_rgba[i, 3] = C_GHOST


# -- rendering -------------------------------------------------------------

# azimuth [deg], and how much of the world to show above / below z=0 [m].
# `contact` exists because of a scale problem: penetration runs a few tenths
# of a millimetre, and at the 115 mm framing that fits the whole wheel one
# millimetre is 2.4 pixels -- the very thing the video is for is invisible.
# Zoomed to 16 mm it is 17 px/mm and unmistakable.
# `axis` is the world axis that runs HORIZONTALLY in that view, and is the one
# the ground ticks are laid along; `near` shifts the ticks along the viewing
# axis so the wheel does not hide them. `tick_mm` is chosen per view because
# the scales differ 13x: 25 mm ticks would give the contact panel less than
# one.
VIEWS = {
    "side":    dict(azimuth=90.0,  above=0.115, below=0.022,
                    axis=0, near=(0.0, 0.055, 0.0), tick_mm=25.0),
    "rear":    dict(azimuth=180.0, above=0.115, below=0.022,
                    axis=1, near=(-0.075, 0.0, 0.0), tick_mm=25.0),
    "contact": dict(azimuth=180.0, above=0.009, below=0.0045,
                    axis=1, near=(-0.055, 0.0, 0.0), tick_mm=2.0),
}


def _camera(view, lookat):
    cam = mujoco.MjvCamera()
    # z pinned to 0, NOT to the wheel: this is what makes the ground project
    # to a single row (see the module docstring).
    cam.lookat = np.array([lookat[0], lookat[1], 0.0])
    cam.distance = STANDOFF
    cam.azimuth = VIEWS[view]["azimuth"]
    cam.elevation = 0.0
    return cam


STANDOFF = 0.45      # camera distance [m], the same for every panel


def framing(model, above, below, panel_h):
    """Turn "show this many metres above and below the ground" into a field of
    view and a crop row.

    Zooming by moving the camera IN does not work at the contact scale: 16 mm
    of view puts the camera 3.9 cm from the contact patch, which is inside the
    wheel, and what renders is the inside of a roller. So every panel keeps
    the same generous standoff and zooms by narrowing `fovy` instead -- a
    telephoto, near-orthographic at the contact end (5 deg), which is also the
    right projection for judging a gap.

    Because the camera sits ON z=0 the view is symmetric about the ground
    line, so `above` alone sets the fov and `below` is purely a crop.
    """
    fovy = float(np.degrees(2 * np.arctan(above / STANDOFF)))
    row0 = panel_h // 2                            # world z=0
    keep = row0 + int(round(below / above * row0))
    px_per_mm = row0 / (above * 1e3)
    return fovy, min(keep, panel_h), px_per_mm, below


def render_panel(renderer, model, data, view, frame, frames_geom, framings,
                 font):
    fovy, keep_rows, px_per_mm, below_m = framings[view]
    hub = data.xpos[model.body("aow_hub").id]
    cam = _camera(view, hub)
    model.vis.global_.fovy = fovy       # the free camera reads fovy from here
    renderer.update_scene(data, camera=cam)
    add_stripes(renderer.scene, data, frames_geom, frame["hot"])
    add_ground_ticks(renderer.scene, view, hub,
                     renderer.width / px_per_mm, below_m)
    img = renderer.render()
    img = _draw_waterline(img, img.shape[0] // 2)   # world z=0, exactly
    img = img[:keep_rows]
    return _caption(img, view, px_per_mm, font)


def _caption(img, view, px_per_mm, font):
    """Name the panel and state its scale, so the zoomed one can never be
    mistaken for the wide one."""
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    # The contact panel is mostly light roller, so light-on-dark captions
    # vanish exactly where the zoom matters most. Back them with a plate.
    d.rectangle([0, 0, 166, 60], fill=(12, 14, 18))
    d.text((10, 6), view, font=font, fill=(226, 232, 244))
    d.text((10, 24), f"{px_per_mm:.1f} px/mm", font=font, fill=(140, 148, 165))
    d.text((10, 42), f"ticks {VIEWS[view]['tick_mm']:.0f} mm",
           font=font, fill=(120, 175, 225))
    return np.asarray(im)


def _draw_waterline(img, row0):
    """Tint everything below z=0 and draw the ground line on top."""
    img = img.copy()
    below = img[row0:].astype(np.float32)
    tint = np.array([38.0, 62.0, 92.0], np.float32)     # cool "underwater"
    img[row0:] = (below * 0.45 + tint * 0.55).astype(np.uint8)
    img[row0:row0 + 2] = np.array([120, 190, 255], np.uint8)
    return img


def _font(size):
    for p in ("/System/Library/Fonts/SFNSMono.ttf",
              "/System/Library/Fonts/Supplemental/Andale Mono.ttf",
              "/Library/Fonts/Arial.ttf"):
        try:
            return ImageFont.truetype(p, size)
        except OSError:
            continue
    return ImageFont.load_default()


_TINT_FLOOR = np.array([21, 34, 51], np.uint8)     # 0.55 * tint, over black


def compose(panels, frame, policy, slowmo, hist, font, font_sm, width,
            span=None):
    """Panels side by side, HUD underneath.

    Every panel puts world z=0 on the same row (it is the panel's vertical
    centre, by construction), so padding them to a common height with the
    sub-floor colour makes the ground line run unbroken across all three --
    even though the zoomed panel is at 7x the scale of its neighbours.
    """
    rows = max(p.shape[0] for p in panels)
    panels = [p if p.shape[0] == rows else
              np.vstack([p, np.broadcast_to(
                  _TINT_FLOOR, (rows - p.shape[0], p.shape[1], 3))])
              for p in panels]
    strip = np.hstack(panels)
    im = Image.fromarray(strip)
    if im.width != width:
        im = im.resize((width, int(im.height * width / im.width)), Image.LANCZOS)
    hud_h = 132
    out = Image.new("RGB", (im.width, im.height + hud_h), (14, 16, 20))
    out.paste(im, (0, 0))
    d = ImageDraw.Draw(out)
    y0 = im.height + 8

    air = "AIRBORNE" if frame["airborne"] else f"{frame['force_w']:.2f} x weight"
    c = frame["clear_mm"]
    d.text((12, y0), f"{policy}", font=font, fill=(235, 238, 245))
    d.text((12, y0 + 22), f"t = {frame['t']:6.3f} s   {slowmo:g}x slow",
           font=font_sm, fill=(150, 158, 172))
    col2 = int(im.width * 0.34)
    d.text((col2, y0),
           f"{'lift' if c > 0 else 'penetration'}   {abs(c):5.3f} mm",
           font=font_sm,
           fill=(245, 160, 90) if c > 0 else (120, 190, 255))
    d.text((col2, y0 + 20), f"contact     {air}", font=font_sm,
           fill=(245, 160, 90) if frame["airborne"] else (200, 208, 220))
    d.text((col2, y0 + 40), f"roll      {frame['roll_deg']:+6.2f} deg",
           font=font_sm, fill=(200, 208, 220))
    d.text((col2, y0 + 60), f"v_lat    {frame['v_lat']:+6.3f} m/s",
           font=font_sm, fill=(200, 208, 220))

    # action bars, normalised to the bounds the net emits against
    col3 = int(im.width * 0.60)
    for i, name in enumerate(("steer", "hub", "diff")):
        yy = y0 + i * 20
        d.text((col3, yy), f"{name:>5}", font=font_sm, fill=(150, 158, 172))
        x0, w = col3 + 52, 120
        d.rectangle([x0, yy + 4, x0 + w, yy + 12], outline=(70, 76, 90))
        mid = x0 + w // 2
        val = float(np.clip(frame["act"][i], -1, 1))
        x_lo, x_hi = sorted((mid, mid + int(val * w / 2)))
        d.rectangle([x_lo, yy + 5, max(x_hi, x_lo + 1), yy + 11],
                    fill=(245, 160, 90) if abs(val) > 0.98 else (110, 170, 240))
        d.line([mid, yy + 2, mid, yy + 14], fill=(120, 128, 145))

    # Clearance history: the whole clip at once, signed about a ground line
    # that matches the one in the panels above -- lift up, penetration down,
    # on ONE symmetric scale so the two halves are directly comparable.
    if len(hist) > 1:
        hx0, hy0 = int(im.width * 0.60) + 190, y0 + 2
        hw, hh = im.width - hx0 - 14, 74
        if hw > 40:
            # `span` is passed in by --compare so the stacked runs share one
            # y-scale. Left to autoscale, the coarse run's ±2.7 mm hop and the
            # fine run's ±0.5 mm buzz draw as the same-sized squiggle, which
            # inverts the comparison the video exists to make.
            span = span or max(0.05, float(np.abs(hist).max()))
            mid = hy0 + hh / 2
            sc = (hh / 2) / span
            d.rectangle([hx0, mid, hx0 + hw, hy0 + hh], fill=(20, 31, 46))
            pts = [(hx0 + hw * i / (len(hist) - 1), mid - v * sc)
                   for i, v in enumerate(hist)]
            d.line(pts, fill=(150, 205, 255), width=1)
            d.line([hx0, mid, hx0 + hw, mid], fill=(120, 190, 255))
            d.line([hx0 + hw, hy0, hx0 + hw, hy0 + hh], fill=(245, 160, 90))
            d.text((hx0, hy0 - 2), f"+{span:.2f}", font=font_sm,
                   fill=(110, 118, 132))
            d.text((hx0, hy0 + hh - 14), f"-{span:.2f}", font=font_sm,
                   fill=(110, 118, 132))
            d.text((hx0 + 52, hy0 + hh + 1), "clearance mm: +lift / -sink",
                   font=font_sm, fill=(110, 118, 132))
    # h264 + yuv420p needs even dimensions, and macro_block_size=1 means
    # imageio will not pad for us -- an odd height is a broken pipe, not a
    # warning.
    arr = np.asarray(out)
    h, w = arr.shape[:2]
    return arr[:h - h % 2, :w - w % 2]


# -- what the hardware would have to survive -------------------------------

def viability(frames, seconds, roller_r_mm):
    """The numbers a physical wheel has to answer for, not the control ones.

    Penetration is reported against the ROLLER radius rather than in absolute
    terms: MuJoCo's soft contact lets a geom sink until the restoring force
    balances, so the depth is a statement about contact stiffness, and a
    penetration that is a noticeable fraction of the roller is a sign the
    contact model -- not the policy -- is setting the behaviour.

    `touchdowns` is the one to watch for hardware. Each is a rear wheel
    landing under load, and the rollers are 4.4 g cones on small axles: it is
    a fatigue and a bearing-noise number long before it is a control number.
    """
    pen = np.array([f["pen_mm"] for f in frames])
    force = np.array([f["force_w"] for f in frames])
    air = np.array([f["airborne"] for f in frames])
    lands = int(np.sum(air[:-1] & ~air[1:]))
    return {
        "pen_peak": float(pen.max()),
        "pen_frac": float(pen.max() / roller_r_mm),
        "force_peak": float(force.max()),
        "air_frac": float(air.mean()),
        "landings_hz": lands / seconds,
        "roller_rpm": max(f["roller_rpm"] for f in frames),
        "hub_rpm": max(f["hub_rpm"] for f in frames),
    }


# -- main ------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policies", nargs="*", default=list(POLICIES))
    ap.add_argument("--seconds", type=float, default=0.8,
                    help="SIMULATED seconds (output length = this x slowmo)")
    ap.add_argument("--slowmo", type=float, default=40.0,
                    help=f"playback slowdown; {5000 / FPS:.0f}x shows every "
                         "physics step and is the ceiling worth asking for")
    ap.add_argument("--views", nargs="*", default=list(VIEWS),
                    choices=list(VIEWS))
    ap.add_argument("--above", type=float, default=None,
                    help="override metres shown ABOVE the ground line for the "
                         "side/rear panels (defaults per view, see VIEWS)")
    ap.add_argument("--below", type=float, default=None,
                    help="override metres shown BELOW the ground line")
    ap.add_argument("--contact-mm", type=float, default=None,
                    help="half-span of the magnified contact panel [mm above "
                         "the ground line]; smaller = more px/mm")
    ap.add_argument("--width", type=int, default=1500, help="output width [px]")
    ap.add_argument("--no-video", action="store_true",
                    help="rollouts and the viability table only, no rendering")
    ap.add_argument("--timeconst", type=float, default=None,
                    help="override contact solref timeconst [s] for this "
                         "render only; config/bike_params.yaml is untouched")
    ap.add_argument("--compare", default=None, metavar="DT,DT,...",
                    help="stack one run PER TIMESTEP in a single video, on a "
                         "shared frame clock and a shared clearance scale, so "
                         "they play against each other. e.g. --compare "
                         "4e-4,3e-3. The runs are NOT the same trajectory: "
                         "same policy and seed, but a different timestep "
                         "diverges within a few steps, so read it as two "
                         "samples of the same behaviour, not as drift")
    ap.add_argument("--drive-tau", type=float, default=None, metavar="S",
                    help="first-order lag [s] on the DRIVE actuators, for this "
                         "render only. The shipped model has none: its velocity "
                         "loop settles in J/kv = 0.6 ms, against the XC430's own "
                         "electromechanical time constant of ~19 ms, so the "
                         "modelled servo reverses ~31x faster than the motor can. "
                         "See docs/plans/aow-contact-approximations.md.")
    ap.add_argument("--timestep", type=float, default=None,
                    help="override sim.timestep [s] for this render only; "
                         "config/bike_params.yaml is untouched. Applied "
                         "BEFORE the env is built, so the control substep "
                         "count follows it. Watch the refsafe margin the "
                         "header prints: once 2*timestep exceeds the solref "
                         "timeconst MuJoCo silently substitutes 2*timestep "
                         "and the clip is of a softer contact than the one "
                         "asked for")
    ap.add_argument("--dampratio", type=float, default=None,
                    help="override contact solref dampratio; 1.0 cannot "
                         "bounce, the physical wheel does -- see "
                         "analysis/contact_calibration.py")
    ap.add_argument("--tag", default="",
                    help="suffix for the output filename, to keep a sweep's "
                         "clips apart")
    ap.add_argument("--out-dir", type=Path, default=REPO / "traces" / "wheel_slowmo",
                    help="videos land in traces/ (gitignored, as "
                         "aow_sim.record already does) rather than "
                         "in analysis/ beside the committed PNGs")
    args = ap.parse_args()

    base = load_params()
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    solref = (args.timeconst, args.dampratio)
    if args.drive_tau:
        # Rebinding the name in general_env's namespace is what takes effect:
        # it does `from ..build_model import build_model`, so patching the
        # source module would be a no-op here.
        ge.build_model = drive_lag(base, args.drive_tau)
        print(f"drive actuators: first-order lag tau = {args.drive_tau * 1e3:.0f} ms "
              f"(the shipped model has none; the XC430's own electromechanical "
              f"time constant is ~19 ms)")

    def env_at(timestep, pol=None):
        """An env for THIS timestep and THIS policy.

        The policy argument is not optional in spirit: observation width is a
        property of the policy, not of the config. A `glide_pitch` export
        carries `vel_window_s` and `obs_pitch` and reads a 19-wide observation,
        while a bare `rl_general.yaml` env builds a 15-wide one — and the
        failure is `pol.action` raising on a length mismatch, which reads like
        a corrupt export rather than a mismatched env. `policy_flags` is the
        one place that maps a loaded policy to those blocks; see its docstring
        for the call sites that each forgot a different one.
        """
        from aow_sim.control.general_spec import policy_flags
        p = base if not timestep else {
            **base, "sim": {**base["sim"], "timestep": timestep}}
        c = cfg
        if pol is not None:
            over = dict(policy_flags(pol),
                        act_wings=bool(getattr(pol, "act_wings", False)),
                        wing_max_deg=float(getattr(pol, "wing_max_deg", 90.0)))
            c = {**cfg, "env": {**cfg["env"], **over}}
        e = GeneralEnv(p, c)
        override_solref(e.model, *solref)
        return p, e

    steps = ([float(x) for x in args.compare.split(",")] if args.compare
             else [args.timestep])
    _probe = (load_general(args.policies[0])
              if args.policies and (MOVES_DIR / f"{args.policies[0]}.npz").exists()
              else None)
    runs = [dict(zip(("params", "env"), env_at(s, _probe))) for s in steps]
    for r, s in zip(runs, steps):
        r["label"] = f"dt {r['env'].model.opt.timestep:g}" if len(steps) > 1 else ""
    params, env = runs[0]["params"], runs[0]["env"]
    # One renderer for every run: the models differ ONLY in opt.timestep, so
    # any run's state can be posed in any run's model. Asserted rather than
    # assumed, because a divergence here would render one run's wheel using
    # another run's geometry and look perfectly plausible.
    assert len({(r["env"].model.nq, r["env"].model.nv, r["env"].model.ngeom)
                for r in runs}) == 1, "compared models are not structurally identical"
    model = env.model
    print(f"contact solref {np.round(model.geom_solref[0], 5).tolist()}"
          + ("  (overridden)" if any(v is not None for v in solref) else ""))

    # MuJoCo's `refsafe` (on unless mjDSBL_REFSAFE is set) raises a POSITIVE
    # solref timeconst to 2*timestep without saying so, so past that point the
    # clip stops being of the contact that was configured. Negative solref gets
    # no such guard -- it just diverges. Either way the ratio is worth seeing.
    tc = float(model.geom_solref[0, 0])
    for r in runs:
        step = r["env"].model.opt.timestep
        if tc > 0:
            margin = 2 * step / tc
            note = (f"  ** CLAMPED: refsafe is using timeconst {2 * step:g} s, "
                    f"not {tc:g}" if margin >= 1.0 else "")
            print(f"dt {step:g}: refsafe margin 2*dt/timeconst = {margin:.2f} "
                  f"({tc / step:.1f} steps per contact time constant)" + note)

    # The frame clock is the SAME sim-time interval for every run, which is
    # what makes stacked runs comparable. Its one hard floor is the coarsest
    # timestep: no run can emit frames faster than it integrates.
    coarsest = max(r["env"].model.opt.timestep for r in runs)
    frame_dt = 1.0 / (FPS * args.slowmo)
    if frame_dt < coarsest:
        print(f"  slowmo {args.slowmo:g}x needs {frame_dt:.2e} s per frame but the "
              f"coarsest timestep is {coarsest:.2e} s -- capped at "
              f"{1 / (FPS * coarsest):.1f}x")
        frame_dt = coarsest
    eff = 1.0 / (FPS * frame_dt)
    print(f"frame clock {frame_dt:.2e} s of sim per frame -> {eff:.1f}x slow "
          f"at {FPS} fps, {args.seconds * eff:.1f} s of video per policy"
          + (f", {len(runs)} runs stacked" if len(runs) > 1 else ""))

    gname = [model.geom(i).name for i in range(model.ngeom)]
    floor = gname.index("floor")
    rear = {i for i, n in enumerate(gname) if n.startswith("roller_")}
    frames_geom = stripe_frames(model, params)

    ghost_everything_but_the_wheel(model)
    panel_h, panel_w = 560, 620
    # The offscreen framebuffer is declared in the MJCF (640x480) and Renderer
    # refuses anything larger -- same lift record.py does.
    model.vis.global_.offwidth = max(panel_w, model.vis.global_.offwidth)
    model.vis.global_.offheight = max(panel_h, model.vis.global_.offheight)
    model.vis.headlight.ambient[:] = 0.62
    model.vis.headlight.diffuse[:] = 0.55
    model.vis.headlight.specular[:] = 0.0
    renderer = mujoco.Renderer(model, panel_h, panel_w)
    framings = {}
    for v in args.views:
        spec = dict(VIEWS[v])
        if v == "contact" and args.contact_mm:
            spec["above"] = args.contact_mm * 1e-3
            spec["below"] = args.contact_mm * 1e-3 / 2
        elif v != "contact":
            spec["above"] = args.above or spec["above"]
            spec["below"] = args.below or spec["below"]
        framings[v] = framing(model, spec["above"], spec["below"], panel_h)
        fovy, _k, ppmm, _b = framings[v]
        print(f"  {v:8} fovy {fovy:5.1f} deg at {STANDOFF * 100:.0f} cm, "
              f"{spec['above'] * 1e3:5.1f} mm above / "
              f"{spec['below'] * 1e3:4.1f} mm below, {ppmm:.1f} px/mm")
    font, font_sm = _font(19), _font(14)
    roller_r_mm = params["omni_wheel"]["roller"]["big_diameter"] / 2 * 1e3
    rows = {}

    for key in args.policies:
        npz = MOVES_DIR / f"{key}.npz"
        if not npz.exists():
            print(f"  skip {key}: no {npz.name}")
            continue
        pol = load_general(key)
        errs, takes = [], []
        for r in runs:
            # Rebuild the env for THIS policy: observation width is a property
            # of the policy, so a run of mixed policies cannot share one env.
            # (`_probe` above already sized the render model off the first, and
            # the assert there is what catches a genuinely different MODEL --
            # a wings policy, say -- as opposed to a different obs width.)
            r["params"], r["env"] = env_at(
                r["env"].model.opt.timestep if len(runs) > 1 else args.timestep,
                pol)
            errs.append(_verify_replication(r["params"], cfg, pol, solref, pol_env=True))
            takes.append(rollout(pol, r["env"], args.seconds, frame_dt,
                                 rear, floor))
        # SIGNED clearance, not pen_mm: pen_mm is >= 0 by construction, so
        # feeding it here drew the lift half of every cycle as a flat line.
        clears = [[f["clear_mm"] for f in t] for t in takes]
        # One clearance scale across the stack, set by whichever run swings
        # furthest -- see the note in compose().
        span = (max(0.05, max(abs(v) for c in clears for v in c))
                if len(runs) > 1 else None)
        for r, t in zip(runs, takes):
            tag = f"{key} {r['label']}".strip()
            rows[tag] = viability(t, args.seconds, roller_r_mm)
        if not args.no_video:
            out = args.out_dir / f"wheel_slowmo_{key}{args.tag}.mp4"
            out.parent.mkdir(parents=True, exist_ok=True)
            writer = imageio.get_writer(out, fps=FPS, macro_block_size=1,
                                        quality=8)
            # Runs can differ by a frame from the rounding in the clock, and a
            # stacked video has to end when the shortest one does.
            n_out = min(len(t) for t in takes)
            for n in range(n_out):
                bands = []
                for r, t, c in zip(runs, takes, clears):
                    fr = t[n]
                    env.data.qpos[:] = fr["qpos"]
                    env.data.qvel[:] = fr["qvel"]
                    mujoco.mj_forward(model, env.data)
                    panels = [render_panel(renderer, model, env.data, v, fr,
                                           frames_geom, framings, font_sm)
                              for v in args.views]
                    bands.append(compose(
                        panels, fr, f"{key}  {r['label']}".strip(), eff,
                        c[:n + 1], font, font_sm, args.width, span=span))
                frame = (bands[0] if len(bands) == 1
                         else np.vstack([b[:, :min(x.shape[1] for x in bands)]
                                         for b in bands]))
                h, w = frame.shape[:2]          # even dims again after vstack
                writer.append_data(frame[:h - h % 2, :w - w % 2])
            writer.close()
            print(f"  wrote {out}  ({n_out} frames, replication err "
                  + ", ".join(f"{e:.1e}" for e in errs) + ")")

    print(f"\nwhat the hardware would have to survive "
          f"({args.seconds:.2f} s of hold, roller radius {roller_r_mm:.1f} mm)")
    # Width follows the labels: --compare appends "dt 0.0004" to each, which
    # overflowed the old fixed 26 and pushed every number out of its column.
    kw = max([26] + [len(k) + 2 for k in rows])
    print(f"{'policy':{kw}}{'pen pk':>8}{'of roller':>11}{'force pk':>10}"
          f"{'airborne':>10}{'landings':>10}{'roller':>9}{'hub':>8}")
    print(f"{'':{kw}}{'[mm]':>8}{'[%]':>11}{'[x weight]':>10}{'[%]':>10}"
          f"{'[/s]':>10}{'[rpm]':>9}{'[rpm]':>8}")
    for key, r in rows.items():
        print(f"{key:{kw}}{r['pen_peak']:>8.2f}{100 * r['pen_frac']:>11.1f}"
              f"{r['force_peak']:>10.2f}{100 * r['air_frac']:>10.0f}"
              f"{r['landings_hz']:>10.1f}{r['roller_rpm']:>9.0f}"
              f"{r['hub_rpm']:>8.0f}")


if __name__ == "__main__":
    main()
