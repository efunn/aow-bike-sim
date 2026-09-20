# Media in the README

What the README's clips are, the command behind each, and how to add more.

The four clips are **committed animated WebPs** in `docs/media/`. They loop on
their own, they render in a clone and with no network, and at ~1.2 MB for the
set they are small enough to live in the repo.

They are also **tracked figures**, so the repo's rule applies: each must be
reproducible at the name it is tracked under. That is what the table below is
for.

---

## Provenance

### Which policy

**`general_rl_cmd_curriculum2b`** — what `control.general_move` in
`config/bike_params.yaml` points at, so it is what teleop drives by default.

| | |
|---|---|
| `plant_digest` | `e1ec36bfa670217e` |
| current plant | `eda849e7afaaca0f` — **they do not match** |
| `v_max` | 1.2 m/s |
| `v_lat_frac` | **0.0** — this export has no crab at all, which is why no tile shows one |
| control rate | 50 Hz |
| sensors | odometry on (`counts` encoder), TM151 AHRS at `ahrs_tau_s` 0.19, pitch in the observation |

**The digest mismatch is not specific to this policy.** No export in `moves/`
matches the current plant: 20 carry a stale digest and 19 predate the field.
The plant moved at commit `5fe1ff9` ("sim: a reduction squares the gain, and
update servo gains"); everything in `moves/` was trained before it. So every
policy is provisional against the bike this repo now describes, and these clips
show one of them driving a plant it was not trained on. It balances and tracks
fine — but do not read a number off these.

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
scored episode. `analysis/eval_video.py` is the tool that reproduces the scored
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
whatever `control.general_move` points at, so the tiles and the default driving
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

480×360, 48 frames, 151–252 KB per clip, 814 KB for the set. No system
`ffmpeg` is needed — `imageio-ffmpeg` (in the `viz` and `dev` extras) ships a
binary and the script finds it.

### The trail

`--trail-seconds 1.0` draws **teleop's own trail** — the fading red line the
viewer draws, aged over the given window. It is not the recorder's other trail:
the `o`/`s`/`t` drawings need a pen, so they keep accumulating spheres forever
and a pen-up gap stays a gap. For a short clip that bead chain reads as clutter
and buries the bike the tile is of.

### Sensors

The clips run with the **sensors the policy trained against** — here the
onboard velocity estimate and a TM151 attitude error model at
`ahrs_tau_s` 0.19 — not MuJoCo ground truth. `record.py` derives that from the
move file and prints what it resolved on every run:

    sensors: ahrs=tm151 tau=0.19s  odometry=front encoder=counts

This matters more than it sounds. A sensor-trained policy replayed on truth is
getting a cleaner signal than it ever saw, so it looks better than it is, and
the eval that scores it does not run that way either. `--sensors truth`
restores the old behaviour for comparison; `--ahrs`, `--ahrs-tau` and
`--odometry` override individually.

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
| Two policies side by side over the whole eval grid | `python analysis/eval_video.py --policies A B --tag <name>` |
| Crab — sideways travel with the heading held | `python -m aow_sim.record --script crab --general <policy>` |
| Self-righting, from a fallen start | `python -m aow_sim.record --script right` |
| Pen drawings — shows commanded control legibly | `python -m aow_sim.record --script o` (also `s`, `t`) |

`eval_video.py` uses the **same seeds as the eval**, so a clip is the same
episode the metric tables describe. That makes it the honest choice for
anything comparative — but it is a 6–7 MB grid sweep, a documentation asset
rather than a tile.

Every `record` run also writes a contact sheet `.png` beside the `.mp4`: one
captioned tile per scripted event, so a whole run reads as a single still.
