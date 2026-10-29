#!/usr/bin/env bash
# The run on a pod as one table: a row per seed with its config, the latest
# checkpoint row from its watch.log, and an ETA to the ini's total_timesteps.
# Runs ON THE POD from ~/aow; scripts/gpu.sh pipes it over ssh, so a pod
# launched from an older bundle can still be read:
#
#   bash status.sh            # one table, now
#   bash status.sh --follow   # the table again whenever any seed checkpoints,
#                             # until results/ALL_DONE
#
# Rows are status, not results: judge a run after export (CLAUDE.md).
cd ~/aow || exit 1

table() {
    local total dirs
    total=$(grep -h '^total_timesteps' ~/puffer_s1/config/aow_bike.ini \
                ~/puffer/config/aow_bike.ini 2>/dev/null | head -1 | tr -d ' _' | cut -d= -f2)
    if ls -d results/seed_* >/dev/null 2>&1; then dirs=$(ls -d results/seed_*); else dirs=results; fi
    echo "== $(date -u +%H:%M:%S) UTC"
    for d in $dirs; do
        k=${d##*seed_}; [ "$d" = results ] && k=1
        # ls sorts its arguments together, so ask for the seed's own first
        g=$(ls graphs/aow_bike_*_s$k.json 2>/dev/null | head -1)
        [ -n "$g" ] || g=$(ls graphs/aow_bike_*.json 2>/dev/null | head -1)
        cfg=$(grep -o '"config": *"[^"]*"' "$g" 2>/dev/null | head -1 | sed 's/.*"\([^"]*\)"$/\1/')
        cfg=$(basename "${cfg:-?}" .yaml); cfg=${cfg#rl_general_}
        row=$(awk '$1 ~ /^[0-9]+$/' "$d/watch.log" 2>/dev/null | tail -1)
        done_=""; [ -f "$d/DONE" ] && done_=done
        echo "$k $cfg $total $done_ | $row"
    done | awk '
        function num(s,  m) { m = 1; if (s ~ /K$/) m = 1e3; if (s ~ /M$/) m = 1e6
                              sub(/[KM]$/, "", s); return s * m }
        BEGIN { printf "%-4s %-18s %6s %8s %7s %8s %9s %6s %6s %6s %6s %6s  %-8s %s\n",
                "seed","config","epoch","steps","sps","len","ret","fall","track","diff","ent","kl","eta","verdict" }
        { split($0, h, " [|] "); split(h[1], a, " "); n = split(h[2], r, " ")
          if (n < 14) { printf "%-4s %-18s  (no checkpoint row yet)\n", a[1], a[2]; next }
          left = a[3] - num(r[2]); eta = (a[4] == "done") ? "done" : \
                 (num(r[3]) > 0 ? sprintf("%dm", left / num(r[3]) / 60 + 0.5) : "?")
          v = r[14]; for (i = 15; i <= n; i++) v = v " " r[i]
          printf "%-4s %-18s %6s %8s %7s %8s %9s %6s %6s %6s %6s %6s  %-8s %s\n",
                 a[1], a[2], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], eta, v }'
}

if [ "${1:-}" = --follow ]; then
    until ls results/watch.log results/seed_*/watch.log 2>/dev/null | grep -q .; do sleep 2; done
    last=""
    while :; do
        # once per ROUND: when the slowest seed checkpoints (or finishes), not
        # at every seed's row -- four rows a round printed four tables
        now=$(for w in $(ls results/watch.log results/seed_*/watch.log 2>/dev/null); do
                  [ -f "$(dirname $w)/DONE" ] && echo 1000000 && continue
                  awk '$1 ~ /^[0-9]+$/' $w | wc -l; done | sort -n | head -1)
        [ "$now" != "$last" ] && { table; echo; last=$now; }
        [ -f results/ALL_DONE ] && break
        sleep 10
    done
    table; echo "== ALL_DONE"; tail -n 4 results/timings.jsonl
else
    table
fi
