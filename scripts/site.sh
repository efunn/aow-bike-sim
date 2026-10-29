#!/usr/bin/env bash
# The static site: assemble it from this repo's pages, look at it, upload it.
#
#   scripts/site.sh build     site/build/, from the pages as they stand (seconds)
#   scripts/site.sh serve     build, then http://localhost:8770
#   scripts/site.sh deploy    build, then upload site/build to a Cloudflare Worker
#
# What goes where (Cloudflare serves <dir>/index.html at /<dir>/):
#
#   /               site/index.html                  by hand (the ice-top game)
#   /aow/           site/aow/index.html + media/     by hand (the four clips; the sim below them, framed)
#   /aow/sim/       src/aow_sim/web/site/            python -m aow_sim.web.export
#   /aow/tech/      site/aow/tech/ (+ bike.webp)     by hand (a summary; links the source and the two below)
#   /aow/linkage/   analysis/mechanism/plots/swing_stepthrough.html   (linked from /aow/tech/)
#   /aow/sim2real/  analysis/policy/plots/drivetrain_traces.html      (linked from /aow/tech/)
#
# The two analysis pages are re-filled from their templates on every build
# (--from-cache: no sim), so a wording edit in
#   analysis/mechanism/templates/swing_stepthrough.html
#   analysis/policy/templates/drivetrain_traces.html
# or a name in site/labels.yaml shows up on the next build. New DATA (a new
# run, a changed design) needs the generator run in full; see its docstring.
#
# deploy needs `npx wrangler login` once, and SITE_WORKER set to the name of
# the Worker the domain is attached to (Workers & Pages in the dashboard). It
# is an assets-only Worker: no script, Cloudflare serves the files as they are.
# A deploy REPLACES every file the Worker serves with site/build, so anything
# uploaded by hand and not in this repo is gone afterwards. A custom domain set
# in the dashboard stays as long as no wrangler config holds a `routes` key
# (Cloudflare's docs: the config's routes override the dashboard's), so keep
# this config-free, or keep `routes` out of any wrangler.jsonc that appears.
# DRY=1 builds and checks without uploading.
set -euo pipefail
cd "$(dirname "$0")/.."

B=site/build
A=$B/aow
PORT=${PORT:-8770}
SIM=src/aow_sim/web/site

build() {
    for f in bike.mjb policy.bin config.json; do
        [[ -f $SIM/assets/$f ]] || { echo "missing $SIM/assets/$f: run python -m aow_sim.web.export" >&2; exit 1; }
    done
    python analysis/policy/drivetrain_trace.py --page --from-cache
    python analysis/mechanism/swing_stepthrough.py --from-cache

    rm -rf "$B"     # only ever this script's output (gitignored)
    mkdir -p "$A"/{media,tech,linkage,sim2real}
    cp site/index.html site/favicon.svg "$B"/
    cp site/aow/index.html site/aow/favicon.svg "$A"/
    cp site/aow/tech/{index.html,bike.webp} "$A"/tech/
    cp site/aow/media/*.mp4 "$A"/media/
    cp -R "$SIM" "$A"/sim
    cp analysis/mechanism/plots/swing_stepthrough.html "$A"/linkage/index.html
    cp analysis/policy/plots/drivetrain_traces.html "$A"/sim2real/index.html
    echo "built $B ($(du -sh "$B" | cut -f1))"
}

case "${1:-}" in
    build)  build ;;
    serve)  build; echo "http://localhost:$PORT"; exec python -m http.server "$PORT" --bind 127.0.0.1 --directory "$B" ;;
    deploy) : "${SITE_WORKER:?set SITE_WORKER to the name of the Cloudflare Worker}"
            build
            npx wrangler deploy --assets "$B" --name "$SITE_WORKER" \
                --compatibility-date 2026-10-01 ${DRY:+--dry-run} ;;
    *)      sed -n '2,6p' "$0"; exit 2 ;;
esac
