"""The browser demo: the general policy driving the bike in MuJoCo's WASM build.

`export.py` writes what the page loads into `site/assets/`; `site/` is a
static site (no build step) that Cloudflare Pages or any file server can host,
and that a personal page embeds with an <iframe>. `check.mjs` replays a Python
trace through the page's own simulation code in node, so a port that drifts
from `GeneralEnv` shows up as numbers rather than as a bike that falls.

Nothing here is imported by training, replay or the Pi.
"""
