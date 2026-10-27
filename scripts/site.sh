#!/usr/bin/env bash
# spiritlake.xyz: assemble the site from this repo's pages, look at it, upload it.
#
#   scripts/site.sh build     site/build/, from the pages as they stand (seconds)
#   scripts/site.sh serve     build, then http://localhost:8770
#   scripts/site.sh deploy    build, then upload site/build to Cloudflare Pages
#
# What goes where (Cloudflare serves <dir>/index.html at /<dir>/):
#
#   /            site/index.html                             by hand
#   /videos/     site/videos/index.html + media/*.webp       by hand
#   /sim/        src/aow_sim/web/site/                       python -m aow_sim.web.export
#   /linkage/    analysis/mechanism/plots/swing_stepthrough.html
#   /sim2real/   analysis/policy/plots/drivetrain_traces.html
#
# The two analysis pages are re-filled from their templates on every build
# (--from-cache: no sim), so a wording edit in
#   analysis/mechanism/templates/swing_stepthrough.html
#   analysis/policy/templates/drivetrain_traces.html
# or a name in site/labels.yaml shows up on the next build. New DATA (a new
# run, a changed design) needs the generator run in full; see its docstring.
#
# deploy needs `npx wrangler login` once, and a Pages project named
# $SITE_PROJECT (default spiritlake) attached to the domain in the dashboard.
set -euo pipefail
cd "$(dirname "$0")/.."

B=site/build
PORT=${PORT:-8770}
PROJECT=${SITE_PROJECT:-spiritlake}
SIM=src/aow_sim/web/site
CLIPS=(hold turn_fwd turn_rev spin180)

build() {
    for f in bike.mjb policy.bin config.json; do
        [[ -f $SIM/assets/$f ]] || { echo "missing $SIM/assets/$f: run python -m aow_sim.web.export" >&2; exit 1; }
    done
    python analysis/policy/drivetrain_trace.py --page --from-cache
    python analysis/mechanism/swing_stepthrough.py --from-cache

    rm -rf "$B"     # only ever this script's output (gitignored)
    mkdir -p "$B"/{videos/media,linkage,sim2real}
    cp site/index.html "$B"/
    cp site/videos/index.html "$B"/videos/
    for c in "${CLIPS[@]}"; do
        if [[ -f site/videos/media/$c.webp ]]; then
            cp "site/videos/media/$c.webp" "$B/videos/media/"
        else
            echo "videos: no site/videos/media/$c.webp yet; using the sim clip docs/media/eval_$c.webp"
            cp "docs/media/eval_$c.webp" "$B/videos/media/$c.webp"
        fi
    done
    cp -R "$SIM" "$B"/sim
    cp analysis/mechanism/plots/swing_stepthrough.html "$B"/linkage/index.html
    cp analysis/policy/plots/drivetrain_traces.html "$B"/sim2real/index.html
    echo "built $B ($(du -sh "$B" | cut -f1))"
}

case "${1:-}" in
    build)  build ;;
    serve)  build; echo "http://localhost:$PORT"; exec python -m http.server "$PORT" --bind 127.0.0.1 --directory "$B" ;;
    deploy) build; npx wrangler pages deploy "$B" --project-name "$PROJECT" ;;
    *)      sed -n '2,7p' "$0"; exit 2 ;;
esac
