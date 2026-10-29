#!/usr/bin/env python3
"""Regenerate the README's looping clips, end to end.

    python scripts/readme_media.py                    # the four sim clips
    python scripts/readme_media.py --only eval_hold   # just one
    python scripts/readme_media.py --policy general_rl_8m
    python scripts/readme_media.py --footage          # the four bike clips
    python scripts/readme_media.py --teardown         # the omni wheel, taken apart
    python scripts/readme_media.py --linkage          # the swing linkage (Chrome)

Renders each sim clip with `aow_sim.record`, converts it to an animated WebP,
and writes it into docs/media/. The mp4s land in traces/ (gitignored) and are
kept: they are the higher-quality source if a tile ever needs re-cropping.

`--footage` renders nothing: it converts the site's phone clips
(site/aow/media/*.mp4) into docs/media/bike_<name>.webp. Those are iPhone HDR
-- BT.2020 primaries, HLG transfer -- and decoded as if they were SDR they come
out desaturated with the orange shifted toward yellow, so they are tone-mapped
to BT.709 first (FOOTAGE_TONEMAP).

WHY A SCRIPT AND NOT A COMMAND IN THE DOCS. Every tracked figure in this repo
has to be reproducible at the name it is tracked under, and a README tile is no
different. A wrapped ffmpeg line in a markdown file is a recipe nobody can run
in one step and that silently goes stale; this is the one place the settings
live. docs/guide/media.md explains WHAT each clip is and which policy it used --
this decides HOW.

The policy defaults to `control.general_move` in config/bike_params.yaml, and
the AHRS to tm151_filter, which is what teleop drives with, so the tiles and the
default driving experience do not drift apart. Pass --policy / --ahrs to
override.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from aow_sim.params import load_params, plant_digest      # noqa: E402

OUT_DIR = ROOT / "docs" / "media"
SRC_DIR = ROOT / "traces"

# The four tiles, in README reading order. Each name is a `--script` in
# record.py's _SCRIPTS and a single command off the eval grid; see docs/guide/media.md.
CLIPS = ["eval_hold", "eval_turn_fwd", "eval_turn_rev", "eval_spin180"]

# --- framing -------------------------------------------------------------
# `chase` rather than the `corner` preset because _CAM_PRESETS OVERRIDES
# --elevation/--azimuth for corner/rear/front/side, so those four cannot be
# re-aimed from the CLI at all. chase takes what it is given.
#
# distance 0.80 frames the 0.30 m ground dial with a little margin, which is
# about as tight as it goes before the rim starts clipping. The record default
# of 3.0 renders the bike as a few pixels.
#
# elevation -28 is a 3/4 view: shallow enough to read roll and pitch, steep
# enough to read the path. The `corner` preset's -18 sees further down-range
# for no benefit here.
CAMERA = ["--camera", "chase", "--distance", "0.80",
          "--elevation", "-28", "--azimuth", "135"]
# 4 s is long enough for every one of these commands to resolve and short
# enough to loop without feeling like a video. 30 fps into the mp4; the webp
# is decimated below.
SECONDS, FPS = "4", "30"
# Teleop's own fading trail. NOT the recorder's pen trail, which accumulates
# spheres forever -- right for the o/s/t drawings, clutter on a 4 s loop.
TRAIL_S = "1.0"

# --- webp ----------------------------------------------------------------
# No crop. There used to be one, to cut the black sky band above the horizon;
# record.py now renders a 20 m floor (visual only -- plane collision is
# infinite) and there is no out-of-bounds black left to remove. Verified by
# measuring the frame border over every frame of all four clips.
WEBP_FPS = 12
WEBP_WIDTH = 480
WEBP_QUALITY = 60

# --- footage -------------------------------------------------------------
# The site's clips, in README reading order. Phone footage, 1080p60 cut to
# 1280 px for the site (the command is in site/aow/index.html's header).
FOOTAGE = ["updown", "disturbed", "loop", "ballhit"]
FOOTAGE_DIR = ROOT / "site" / "aow" / "media"
# 15, not the sim's 12: real motion is less regular than a 4 s sim loop, and
# 60 divides by it.
FOOTAGE_FPS = 15
# 80, not the sim's 60: at 60 the dark floor tiles break into visible blocks;
# a real floor's texture is what the sim's flat colours never had.
FOOTAGE_QUALITY = 80
# HLG -> linear -> BT.709 primaries -> tone map -> BT.709. hable, picked by
# the user from one frame of disturbed.mp4 against mobius (blown out) and clip
# (worse). npl 203 is HLG's reference white.
FOOTAGE_TONEMAP = ("zscale=t=linear:npl=203,format=gbrpf32le,zscale=p=bt709,"
                   "tonemap=hable:desat=0,zscale=t=bt709:m=bt709:r=tv,"
                   "format=yuv420p")
HDR_TRANSFERS = ("arib-std-b67", "smpte2084")


def ffmpeg() -> str:
    """imageio-ffmpeg's bundled binary first: it has libwebp and zscale.
    Homebrew's (9.0.2, 2026-10-27) has neither, so a PATH lookup first found
    an ffmpeg that cannot write the output at all."""
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        pass
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    sys.exit("need ffmpeg, or the viz extra:  pip install -e '.[viz]'")


def is_hdr(exe: str, src: Path) -> bool:
    """Read the transfer tag off ffmpeg's stream banner (the bundled binary
    ships no ffprobe)."""
    r = subprocess.run([exe, "-hide_banner", "-i", str(src)],
                       capture_output=True, text=True)
    return any(t in r.stderr for t in HDR_TRANSFERS)


def footage(names: list[str]) -> None:
    exe = ffmpeg()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name in names:
        src = FOOTAGE_DIR / f"{name}.mp4"
        if not src.exists():
            sys.exit(f"no such clip: {src}")
        webp = OUT_DIR / f"bike_{name}.webp"
        hdr = is_hdr(exe, src)
        vf = ",".join(([FOOTAGE_TONEMAP] if hdr else []) +
                      [f"fps={FOOTAGE_FPS}",
                       f"scale={WEBP_WIDTH}:-2:flags=lanczos"])
        subprocess.run(
            [exe, "-y", "-i", str(src), "-an", "-vf", vf,
             "-loop", "0", "-quality", str(FOOTAGE_QUALITY),
             "-compression_level", "6", str(webp)],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"{name:10s} {'HLG -> 709' if hdr else 'SDR':10s} -> "
              f"{webp.relative_to(ROOT)}  {webp.stat().st_size // 1024} KB")


# --- teardown ------------------------------------------------------------
# The omni wheel's teardown photos (2026-07-17), outside in, as a slow
# slideshow. All six were shot on the same notebook page from the same spot,
# so a crossfade reads as the wheel coming apart in place.
TEARDOWN = sorted((ROOT / "docs" / "measurements" / "wheel-dimensions")
                  .glob("0[1-6]-*.jpg"))
# The notebook page, cutting the dark mat either side (4080 x 3072 sources).
TEARDOWN_CROP = (300, 150, 3780, 3072)
TEARDOWN_HOLD_MS, TEARDOWN_FADE_MS, TEARDOWN_FADE_N = 750, 250, 4
# White balance, per photo: a bare patch of the page (source px, top left,
# clear of the wheel and the handwriting in all six) is scaled to a slightly
# warm white. As shot, the page reads ~(210, 199, 169) and the hub's white
# plastic ~(195, 188, 167) -- the plastic's blue/green is ~4% above the
# page's -- so a page at blue/green 0.96 leaves the plastic about neutral:
# the notebook still a touch yellow against it. Per photo because the six
# differ in exposure (page green 197-228) and in warmth (blue/green
# 0.80-0.86).
TEARDOWN_PAGE_BOX = (700, 300, 1100, 700)
TEARDOWN_PAGE_WHITE = (236, 232, 222)


def white_balance(im):
    import numpy as np
    from PIL import Image
    a = np.asarray(im).astype(np.float32)
    x0, y0, x1, y1 = TEARDOWN_PAGE_BOX
    page = np.median(a[y0:y1, x0:x1].reshape(-1, 3), axis=0)
    a *= np.asarray(TEARDOWN_PAGE_WHITE, np.float32) / page
    return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))


def teardown() -> None:
    from PIL import Image, ImageOps
    stills = []
    for f in TEARDOWN:
        im = ImageOps.exif_transpose(Image.open(f)).convert("RGB")
        im = white_balance(im).crop(TEARDOWN_CROP)
        stills.append(im.resize((WEBP_WIDTH, round(im.height * WEBP_WIDTH / im.width)),
                                Image.LANCZOS))
    frames, ms = [], []
    for i, im in enumerate(stills):
        nxt = stills[(i + 1) % len(stills)]
        frames.append(im); ms.append(TEARDOWN_HOLD_MS)
        for k in range(1, TEARDOWN_FADE_N + 1):
            frames.append(Image.blend(im, nxt, k / (TEARDOWN_FADE_N + 1)))
            ms.append(TEARDOWN_FADE_MS // TEARDOWN_FADE_N)
    webp = OUT_DIR / "omni_teardown.webp"
    # Every frame a keyframe. Left to itself the encoder re-codes only the
    # blocks it judges changed, and across a crossfade it keeps stale ones:
    # the first try had a grid of white patches over the third photo.
    frames[0].save(webp, save_all=True, append_images=frames[1:], duration=ms,
                   loop=0, quality=FOOTAGE_QUALITY, method=6, kmin=1, kmax=1)
    print(f"teardown   {len(stills)} photos -> {webp.relative_to(ROOT)}  "
          f"{webp.stat().st_size // 1024} KB")


# --- linkage -------------------------------------------------------------
# The stepthrough page's two views for the current blade (V3, the right
# pane's default) -- "Mechanism, bike frame" and "Bike on the floor", drawn in
# sync -- stepped through the stroke and back the way the page's own Play
# does. Captured with headless Chrome from a scratch copy of the tracked page
# with a few lines appended: grab the slider (which pauses the autoplay), pin
# that pane's two views, captions and all, to the window, and set the slider
# from the URL's #fragment. The window is over the page's 900 px breakpoint,
# so the two views sit side by side. The page itself is not touched;
# re-render it first (analysis/mechanism/swing_stepthrough.py) if a design
# changed.
STEPTHROUGH = ROOT / "analysis" / "mechanism" / "plots" / "swing_stepthrough.html"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
LINKAGE_STEPS = 60          # out, then the same frames back
LINKAGE_FRAME_MS = 67       # 4 s a sweep, the page's own pace
LINKAGE_HOLD_MS = 600       # at each end of the stroke
LINKAGE_WINDOW = (1000, 540)   # CSS px: two square canvases, captions, padding
LINKAGE_WIDTH = 720
_CAPTURE_JS = """
<style>body *{visibility:hidden}
.cap-target, .cap-target *{visibility:visible!important}
.cap-target{position:fixed;inset:0;padding:12px;z-index:99;background:var(--bg)}</style>
<script>window.addEventListener("load", () => {
  const s = document.getElementById("frac");
  s.dispatchEvent(new Event("pointerdown"));
  document.querySelectorAll(".pane .views")[1].classList.add("cap-target");
  requestAnimationFrame(() => { s.value = +location.hash.slice(1);
                                s.dispatchEvent(new Event("input")); });
});</script>
"""


def linkage() -> None:
    import tempfile
    from PIL import Image
    chrome = CHROME if Path(CHROME).exists() else shutil.which("google-chrome")
    if not chrome:
        sys.exit("need Google Chrome for the linkage capture")
    with tempfile.TemporaryDirectory() as tmp:
        page = Path(tmp) / "capture.html"
        page.write_text(STEPTHROUGH.read_text() + _CAPTURE_JS)
        frames = []
        for k in range(LINKAGE_STEPS + 1):
            png = Path(tmp) / f"{k:03d}.png"
            subprocess.run(
                [chrome, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                 "--window-size={},{}".format(*LINKAGE_WINDOW),
                 "--force-device-scale-factor=2",
                 "--virtual-time-budget=3000", f"--screenshot={png}",
                 f"{page.as_uri()}#{round(1000 * k / LINKAGE_STEPS)}"],
                check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            im = Image.open(png).convert("RGB")
            frames.append(im.resize((LINKAGE_WIDTH,
                                     round(im.height * LINKAGE_WIDTH / im.width)),
                                    Image.LANCZOS))
    seq = frames + frames[-2:0:-1]
    ms = [LINKAGE_FRAME_MS] * len(seq)
    ms[0] = ms[LINKAGE_STEPS] = LINKAGE_HOLD_MS
    webp = OUT_DIR / "swing_linkage.webp"
    seq[0].save(webp, save_all=True, append_images=seq[1:], duration=ms,
                loop=0, quality=FOOTAGE_QUALITY, method=6)
    print(f"linkage    {len(seq)} frames -> {webp.relative_to(ROOT)}  "
          f"{webp.stat().st_size // 1024} KB")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", default=None,
                    help="moves/NAME. Default: control.general_move")
    ap.add_argument("--ahrs", default="tm151_filter",
                    help="passed to aow_sim.record --ahrs. Default: teleop's "
                         "own default (run_drive --ahrs), so the tiles are "
                         "what teleop drives")
    ap.add_argument("--only", action="append", choices=CLIPS + FOOTAGE,
                    default=None,
                    help="regenerate just this clip (repeatable)")
    ap.add_argument("--footage", action="store_true",
                    help="convert the bike footage instead of rendering the "
                         "sim clips")
    ap.add_argument("--teardown", action="store_true",
                    help="the omni wheel's teardown photos as a slideshow")
    ap.add_argument("--linkage", action="store_true",
                    help="the swing linkage's stroke, from the stepthrough "
                         "page (needs Google Chrome)")
    ap.add_argument("--keep-mp4", action="store_true", default=True,
                    help="(default) leave the source mp4s in traces/")
    a = ap.parse_args()

    if a.footage or a.teardown or a.linkage:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        if a.footage:
            footage([n for n in (a.only or FOOTAGE) if n in FOOTAGE])
        if a.teardown:
            teardown()
        if a.linkage:
            linkage()
        return

    params = load_params()
    policy = a.policy or params["control"]["general_move"]
    clips = a.only or CLIPS

    # Say out loud whether the policy matches the bike the repo now describes.
    # A tile rendered by a policy trained against a different plant is still a
    # fine picture, but nobody should read a number off it.
    import yaml
    move = ROOT / "moves" / f"{policy}.yaml"
    if not move.exists():
        sys.exit(f"no such policy: {move}")
    pd = (yaml.safe_load(move.read_text()) or {}).get("plant_digest")
    cur = plant_digest(params)
    print(f"policy       {policy}", flush=True)
    print(f"plant_digest {pd}  (current {cur})"
          f"  {'MATCH' if pd == cur else '** STALE -- trained on a different plant **'}",
          flush=True)
    print(flush=True)

    exe = ffmpeg()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    SRC_DIR.mkdir(parents=True, exist_ok=True)

    for name in clips:
        mp4 = SRC_DIR / f"{name}.mp4"
        webp = OUT_DIR / f"{name}.webp"
        print(f"--- {name}", flush=True)
        subprocess.run(
            [sys.executable, "-m", "aow_sim.record", "--script", name,
             "--general", policy, *CAMERA, "--seconds", SECONDS, "--fps", FPS,
             "--trail-seconds", TRAIL_S, "--out", str(mp4),
             *(["--ahrs", a.ahrs] if a.ahrs else [])],
            cwd=ROOT, check=True)
        subprocess.run(
            [exe, "-y", "-i", str(mp4),
             "-vf", f"fps={WEBP_FPS},scale={WEBP_WIDTH}:-1:flags=lanczos",
             "-loop", "0", "-quality", str(WEBP_QUALITY),
             "-compression_level", "6", str(webp)],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"    -> {webp.relative_to(ROOT)}  {webp.stat().st_size // 1024} KB")

    total = sum(f.stat().st_size for f in OUT_DIR.glob("*.webp"))
    print(f"\ndocs/media/ total {total // 1024} KB")


if __name__ == "__main__":
    main()
