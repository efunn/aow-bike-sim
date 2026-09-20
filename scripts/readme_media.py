#!/usr/bin/env python3
"""Regenerate the README's four looping clips, end to end.

    python scripts/readme_media.py                    # all four
    python scripts/readme_media.py --only eval_hold   # just one
    python scripts/readme_media.py --policy general_rl_8m

Renders each clip with `aow_sim.record`, converts it to an animated WebP, and
writes it into docs/media/. The mp4s land in traces/ (gitignored) and are kept:
they are the higher-quality source if a tile ever needs re-cropping.

WHY A SCRIPT AND NOT A COMMAND IN THE DOCS. Every tracked figure in this repo
has to be reproducible at the name it is tracked under, and a README tile is no
different. A wrapped ffmpeg line in a markdown file is a recipe nobody can run
in one step and that silently goes stale; this is the one place the settings
live. docs/guide/media.md explains WHAT each clip is and which policy it used --
this decides HOW.

The policy defaults to `control.general_move` in config/bike_params.yaml, which
is what teleop drives with, so the tiles and the default driving experience do
not drift apart. Pass --policy to override.
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


def ffmpeg() -> str:
    """The bundled binary, so no system ffmpeg is needed."""
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    try:
        import imageio_ffmpeg
    except ImportError:
        sys.exit("need ffmpeg, or the viz extra:  pip install -e '.[viz]'")
    return imageio_ffmpeg.get_ffmpeg_exe()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", default=None,
                    help="moves/NAME. Default: control.general_move")
    ap.add_argument("--only", action="append", choices=CLIPS, default=None,
                    help="regenerate just this clip (repeatable)")
    ap.add_argument("--keep-mp4", action="store_true", default=True,
                    help="(default) leave the source mp4s in traces/")
    a = ap.parse_args()

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
             "--trail-seconds", TRAIL_S, "--out", str(mp4)],
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
