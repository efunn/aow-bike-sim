# Media in the README

What the README's clips are, the command behind each, and how to add more.

There are two sets, both **committed animated WebPs** in `docs/media/`: four
clips of the real bike (`bike_*.webp`), which the README shows at the top, and
four sim renders (`eval_*.webp`, ~0.9 MB), which it does not show at the
moment (2026-10-27: they are of an older policy). They loop on their own,
they render in a clone and with no network, and they are small enough to live
in the repo.

They are also **tracked figures**, so the repo's rule applies: each must be
reproducible at the name it is tracked under. `scripts/readme_media.py` makes
both sets.

---

## The bike footage

```sh
python scripts/readme_media.py --footage                # all four
python scripts/readme_media.py --footage --only loop    # just one
```

The sources are the site's clips, `site/aow/media/{updown,disturbed,loop,ballhit}.mp4`:
phone footage (1080p60), cut to 1280 px for the site by the command in
`site/aow/index.html`'s header. The script decimates to 15 fps, scales to 480
px wide and writes `docs/media/bike_<name>.webp`.

| file | KB (2026-10-28) |
|---|---|
| `bike_updown.webp` | 1305 |
| `bike_disturbed.webp` | 704 |
| `bike_loop.webp` | 526 |
| `bike_ballhit.webp` | 450 |

**They are HDR, and are tone-mapped.** The phone records BT.2020 primaries
with the HLG transfer (`arib-std-b67`); the mp4s keep those tags, and a
browser playing them honours them. A plain conversion to WebP does not: it
reads the pixels as SDR, and the result is washed out with the orange shifted
toward yellow. The script reads the transfer off ffmpeg's banner and, for HLG
or PQ, converts to BT.709 first through `zscale` and `tonemap=hable` (against
`mobius`, which came out blown out, and `clip`; picked by eye on one frame).
That needs `zscale` and `libwebp`, which imageio-ffmpeg's bundled binary has
and Homebrew's 9.0.2 does not, so the script uses the bundled one.

**Quality 80, not the sim set's 60** (`FOOTAGE_QUALITY`): at 60 the dark floor
tiles break into visible blocks.

---

## The teardown and the linkage

```sh
python scripts/readme_media.py --teardown    # docs/media/omni_teardown.webp
python scripts/readme_media.py --linkage     # docs/media/swing_linkage.webp
```

| file | from | KB (2026-10-28) |
|---|---|---|
| `omni_teardown.webp` | `docs/measurements/wheel-dimensions/01-06*.jpg` | 431 |
| `swing_linkage.webp` | `analysis/mechanism/plots/swing_stepthrough.html` | 1135 |

**The teardown** is the six photos of the 2026-07-17 teardown, outside in,
cropped to the notebook page (`TEARDOWN_CROP`), each held 0.75 s with a
0.25 s crossfade to the next. Each photo is white-balanced on its own: a bare
patch of the page (`TEARDOWN_PAGE_BOX`) is scaled to a slightly warm white
(`TEARDOWN_PAGE_WHITE`, blue/green 0.96). As shot, the page and the hub's
white plastic both read yellow, the plastic ~4% bluer than the page, so that
target leaves the plastic about neutral and the notebook still a touch warm
against it (photo 03's hub: (195, 188, 167) as shot, (222, 221, 220) after). Every frame is a keyframe (`kmin=kmax=1`): left to
itself the animated-WebP encoder re-codes only the blocks it judges changed,
and across a crossfade it kept stale ones, a grid of white patches over the
third photo.

**The linkage** is the stepthrough page's two views for V3, the current
blade, side by side and in sync: "Mechanism, bike frame" and "Bike on the
floor", with their crank and roll readouts. Captured with headless Google
Chrome at 1000 x 540 CSS px (over the page's 900 px breakpoint, so the views
sit side by side), scaled to 720 px. The script appends a few lines to a
scratch copy of the tracked page: they grab the slider (which pauses the
autoplay), pin that pane's views to the window, and set the slider from the
URL's `#fragment`. The slider's far end runs past V3's 117° stroke (the
readout says "130° of 117°"), as it does on the page. 61
positions across the stroke, then the same frames back, 4 s a sweep, as the
page's own Play does. It shoots the page as it stands, so re-render the page
first (`analysis/mechanism/swing_stepthrough.py --sim`) if a design changed.
It takes the system's colour scheme; this one was shot in dark mode.

---

## The two diagrams

```sh
python scripts/readme_diagrams.py                 # the pointer's
python scripts/readme_diagrams.py --policy NAME   # another export's
```

`docs/media/control_scheme.svg` and `docs/media/curriculum.svg`, drawn from
the data, not typed: the observation layout and the network from the move
yaml (`obs_layout`, `trained.arch`), the action bounds and the command
sampler's constants from the training config the move names. The config's
sha256 is checked against the one the move recorded, and the script refuses
on a mismatch, so a config edited since the run is never drawn as the one it
trained on. The family mix is the sampler's own formula
(`gpu/commands.py`), evaluated at difficulty 0, 0.25, 0.5, 0.75 and 1; it
agrees with the mix measured in `config/rl/rl_general_v4.yaml`'s comment
(40/0/60/0 at 0, 10/20/30/40 at 1). Every observation entry must fall in one
of the script's groups, or it stops.

The bike icons are the drivetrain-traces page's (`BACK`, `TOP`, the arc
arrow), at the same 40-unit scale. **No wings in any drawing**: the policy
neither observes nor drives them. Both files carry a `prefers-color-scheme`
block, checked rendered as `<img>` in headless Chrome in both schemes.

---

## The sim renders

### Which policy

**`general_rl_smooth_temporal`** — what `control.general_move` in
`config/bike_params.yaml` points at — on the `tm151_filter` AHRS, teleop's
default. So the tiles are what a bare `mjpython -m aow_sim.run_drive --teleop`
drives.

| | |
|---|---|
| `plant_digest` | `9953690d05e33ab6` |
| current plant | `12d9639e3ce90f36` (trained before the latest plant change) |
| drivetrain | the detailed one (XC430 firmware loop), as trained — it is the default plant |
| `v_max` | 1.2 m/s |
| `v_lat_frac` | **0.0** — this export has no crab at all, which is why no tile shows one |
| sensors | odometry on (`counts` encoder); AHRS **`tm151_filter`** (it trained on `tm151`); pitch in the observation |

### Which commands

Each clip is **one command out of the eval grid**
(`train_general_rl._EVAL_CMDS`), applied as a step at t=0 and held — which is
what an eval episode does. They are `_SCRIPTS` entries in
[`record.py`](../../src/aow_sim/record.py):

| file | `--script` | eval command `(v_lon, v_lat, dpsi)` | family |
|---|---|---|---|
| `eval_hold.webp` | `eval_hold` | `(0, 0, 0°)` | `hold` |
| `eval_turn_fwd.webp` | `eval_turn_fwd` | `(0.804, 0, +90°)` | `cruise` |
| `eval_turn_rev.webp` | `eval_turn_rev` | `(-0.504, 0, +90°)` | `cruise` |
| `eval_spin180.webp` | `eval_spin180` | `(0, 0, 180°)` | `turn_big` |

The velocities are the grid's own fractions — 0.67 and −0.42 of `v_max` —
evaluated at run time, so a tile stays the command the metrics describe if
`v_max` moves. They are duplicated in `record.py` rather than imported because
`train_general_rl` pulls in gymnasium, SB3 and torch, and recording a video
must not need any of them.

**These are not seeded to match `_eval_episodes`** (which uses `10_000 + k`), so
a tile is the eval *command* from a settled standstill, not byte-for-byte the
scored episode. `analysis/policy/eval_video.py` is the tool that reproduces the scored
episode exactly.

One caveat on the 180: `command_to_body` uses `wrap_pi(psi_cmd - psi)`, so +180
and −180 are the same observation and which way the bike spins is decided by
yaw noise. The grid carries `±170` rows for exactly that reason. The tile shows
a real about-face; it just cannot be read as evidence about handedness.

### Regenerating them

**One script owns the settings:**

```sh
python scripts/readme_media.py                    # all four
python scripts/readme_media.py --only eval_hold   # just one
python scripts/readme_media.py --policy general_rl_8m
```

It renders each clip with `aow_sim.record`, converts it to WebP, writes
`docs/media/<name>.webp`, and keeps the source mp4 in `traces/`. It defaults to
whatever `control.general_move` points at, on teleop's default AHRS
(`tm151_filter`), so the tiles and the default driving
experience cannot drift apart, and it prints the plant-digest comparison above
every time it runs.

The framing it uses, and why:

- **`--camera chase`**, not the `corner` preset. `_CAM_PRESETS` **overrides**
  `--elevation`/`--azimuth` for `corner`/`rear`/`front`/`side`, so those four
  cannot be re-aimed from the CLI at all. `chase` takes what it is given.
- **`--distance 0.80`** frames the 0.30 m ground dial with a little margin,
  about as tight as it goes before the rim clips. The record default of 3.0
  renders the bike as a few pixels.
- **`--elevation -28`**, a 3/4 view: shallow enough to read roll and pitch,
  steep enough to read the path.
- **`--seconds 4`**, long enough for each command to resolve, short enough to
  loop without feeling like a video.
- **`--trail-seconds 1.0`** — see [the trail](#the-trail) below.

### No black, and where it came from

`record.py` renders a **20 m floor** (`_RENDER_FLOOR`, overridable with
`--floor`). The config's `sim.floor_size` is 3.0, which is sized for a viewer
sitting at the origin; a tracking camera following a bike that has driven three
metres puts the edge of the world in frame, and past that edge the render is
black. A MuJoCo plane's collision is **infinite** — `floor_size` only sets what
gets drawn — so this is identical physics and a bigger picture. The same trick
was already in the `demo` path for the same reason.

**It is done in `record.py`, never in `bike_params.yaml`.** That file is hashed
into `plant_digest`; a purely cosmetic change to it would mark every trained
policy stale.

There is consequently **no crop** in the WebP conversion any more. An earlier
version cropped 20 rows off the top to cut a black sky band; with the bigger
floor there is nothing to cut. Verified by measuring the frame border across
every frame of all four clips: 0.00%.

### The WebP conversion

```sh
ffmpeg -i traces/<script>.mp4 \
    -vf "fps=12,scale=480:-1:flags=lanczos" \
    -loop 0 -quality 60 -compression_level 6 docs/media/<script>.webp
```

480×360, 48 frames, 167–267 KB per clip, 883 KB for the set (2026-10-09). No system
`ffmpeg` is needed — `imageio-ffmpeg` (in the `viz` and `dev` extras) ships a
binary and the script finds it.

### The trail

`--trail-seconds 1.0` draws **teleop's own trail** — the fading red line the
viewer draws, aged over the given window. It is not the recorder's other trail:
the `o`/`s`/`t` drawings need a pen, so they keep accumulating spheres forever
and a pen-up gap stays a gap. For a short clip that bead chain reads as clutter
and buries the bike the tile is of.

### Sensors

By default the clips run with the **sensors the policy trained against**
— for this one the onboard velocity estimate and a TM151 attitude error
model at `ahrs_tau_s` 0.19 — not MuJoCo ground truth. `record.py` derives
that from the move file and prints what it resolved on every run. The current
tiles override the AHRS:

    sensors: ahrs=tm151_filter tau=2s  odometry=front encoder=counts

This matters more than it sounds. A sensor-trained policy replayed on truth is
getting a cleaner signal than it ever saw, so it looks better than it is, and
the eval that scores it does not run that way either. `--sensors truth`
restores the old behaviour for comparison; `--ahrs`, `--ahrs-tau` and
`--odometry` override individually. Under `tm151_filter` the printed tau is
only the time of its 0.1 deg residual wander; the filter's own time constants
are fixed in `sim_ahrs`.

---

## Adding a longer clip

Anything too long to commit goes to **GitHub's attachment CDN** instead. Drop
the file into the comment box of any issue or pull request — do not submit the
comment — and GitHub hands back a URL:

    https://github.com/user-attachments/assets/<id>

Nothing is committed and nothing enters anyone's clone. Inside a `<table>` the
URL has to go in an element, because markdown is not parsed inside block HTML
and a bare URL there stays text:

```html
<video src="https://github.com/user-attachments/assets/<id>" controls muted></video>
```

**Verified against a rendered README** (2026-09-19, reading the DOM of a public
repo that uses this): the `<video>` element survives, carrying `src`,
`controls` and `muted`. GitHub adds its own class and
`style="max-height:640px; min-height:200px"`.

Three things to know:

- **No autoplay and no loop.** Those attributes are stripped, so a video waits
  for a click. That is the whole reason the four tiles above are WebP: a
  click-to-play tile is dead weight for a silent four-second sim render.
- **`width` is stripped too.** GitHub imposes its own sizing, so `width="100%"`
  on a `<video>` is inert. Harmless, but do not expect it to do anything.
- **`POST https://api.github.com/markdown` is not the same renderer.** It
  strips `<video>` entirely, in both `markdown` and `gfm` modes, while `<img>`
  survives. Do not use that endpoint to test video markup — it will tell you
  the tag does not work, and it is wrong about the README.

At render time GitHub rewrites the `src` to a signed
`private-user-images.githubusercontent.com/...?jwt=...` URL that expires in
five minutes. The `user-attachments` URL is the stable thing you write. So
these cannot be hotlinked anywhere else, and they are blank in a clone or with
no network — which is the trade for not committing them.

### Candidates worth a longer clip

| what | command |
|---|---|
| Two policies side by side over the whole eval grid | `python analysis/policy/eval_video.py --policies A B --tag <name>` |
| Crab — sideways travel with the heading held | `python -m aow_sim.record --script crab --general <policy>` |
| Self-righting, from a fallen start | `python -m aow_sim.record --script right` |
| Pen drawings — shows commanded control legibly | `python -m aow_sim.record --script o` (also `s`, `t`) |

`eval_video.py` uses the **same seeds as the eval**, so a clip is the same
episode the metric tables describe. That makes it the honest choice for
anything comparative — but it is a 6–7 MB grid sweep, a documentation asset
rather than a tile.

Every `record` run also writes a contact sheet `.png` beside the `.mp4`: one
captioned tile per scripted event, so a whole run reads as a single still.
