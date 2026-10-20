#!/usr/bin/env bash
# The bike on the Pi: ship the code, start run_bike, open the mirror.
#
#   ./scripts/bike.sh sync                 # code + deploy/ to the Pi; deletes nothing there
#   ./scripts/bike.sh bench                # run_bike --bench: torque OFF, mount accepted
#   ./scripts/bike.sh run [run_bike args]  # run_bike with whatever flags you give it
#   ./scripts/bike.sh mirror [args]        # the MuJoCo station, here (run in your own Terminal)
#   ./scripts/bike.sh status               # is run_bike up, which ports, is the bundle in step
#
# Typical bench session, two terminals:
#   ./scripts/bike.sh sync && ./scripts/bike.sh bench
#   ./scripts/bike.sh mirror
#
# `bench`/`run` stay in the FOREGROUND over `ssh -t`, so Ctrl-C there is
# run_bike's own clean shutdown (torque off on the way out) -- not detached on
# purpose: a bike process you cannot see is one you cannot stop. Extra args
# pass straight through: `bench --torque`, `run --no-link --seconds 60`,
# `bench --tag steer_check`.
#
# THE U2D2 PORT. run_bike defaults to control.onboard.dxl_port, a by-id path
# naming ONE adapter's serial. If that adapter is not the one plugged in and
# exactly one FTDI adapter is, this passes that one as --port and says so.
# Two adapters, or none, and it leaves run_bike to complain. `--port` given
# by hand always wins.
#
# IF run_bike KEEPS "waiting for a command" WITH THE MIRROR OPEN, the
# mirror's UDP is not arriving -- and the mirror drops send errors, so it
# will not say so. `status` tests exactly that send from this terminal.
# Seen once (2026-10-19, from the Claude desktop app's terminal): python got
# `No route to host` to the Pi while ssh got through, and the same send
# worked minutes later. Cause NOT established -- macOS Local Network
# privacy (which exempts Apple binaries like ssh) fits, as does a
# not-yet-resolved neighbour -- so: run `status`, then the mirror from
# Terminal.app if it says BLOCKED.
#
# `sync` is the runbook's rsync (pi-bench-bringup.md section 7) WITHOUT
# --delete: the Pi's tree has policy files and bench logs that exist only
# there, and --delete would remove them. Stale files left on the Pi are
# harmless; the bundle's digest check catches a stale bundle.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

PI="${PI:-efun@aowbike.local}"
PI_REPO="aow-bike-sim"          # relative to the Pi's home
PI_PY="~/venv/bin/python"       # NOT the system python

usage() { sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

has_port_arg() {
  for a in "$@"; do [[ "$a" == --port || "$a" == --port=* ]] && return 0; done
  return 1
}

# -> "--port <path>" when the configured adapter is absent and exactly one
#    FTDI adapter is present; nothing otherwise.
port_fallback() {
  local want got
  want="$(python - <<'PY'
import yaml
p = yaml.safe_load(open("config/bike_params.yaml"))
print(p["control"].get("onboard", {}).get("dxl_port", ""))
PY
)"
  got="$(ssh "$PI" "test -e '$want' && echo present; ls /dev/serial/by-id/ 2>/dev/null | grep -i ftdi || true")"
  if grep -q '^present$' <<<"$got"; then return 0; fi
  local ftdi; ftdi="$(grep -iv '^present$' <<<"$got" | grep -i ftdi || true)"
  if [[ -n "$ftdi" && "$(wc -l <<<"$ftdi")" -eq 1 ]]; then
    echo "note: configured U2D2 ($(basename "$want")) is not plugged in;" \
         "using the one that is: $ftdi" >&2
    echo "--port /dev/serial/by-id/$ftdi"
  fi
}

run_bike() {
  local extra=""
  if ! has_port_arg "$@"; then extra="$(port_fallback)"; fi
  local args; args="$(printf ' %q' "$@")"
  exec ssh -t "$PI" "cd ~/$PI_REPO && $PI_PY -m aow_sim.hw.run_bike $extra$args"
}

cmd="${1:-}"; [[ $# -gt 0 ]] && shift
case "$cmd" in
  sync)
    rsync -a --exclude .git --exclude runs --exclude traces \
      --exclude __pycache__ --exclude .claude --exclude .pytest_cache \
      ./ "$PI:~/$PI_REPO/"
    [[ -f deploy/bundle.npz ]] || echo "warning: no local deploy/bundle.npz --" \
      "run python -m aow_sim.export_deploy" >&2
    "$0" status
    ;;
  bench) run_bike --bench "$@" ;;
  run)   run_bike "$@" ;;
  mirror) exec mjpython -m aow_sim.run_drive --mirror "${PI#*@}" "$@" ;;
  status)
    local_md5="$(md5 -q deploy/bundle.npz 2>/dev/null || echo none)"
    ssh "$PI" "cd ~/$PI_REPO && echo \"run_bike: \$(pgrep -f [a]ow_sim[.]hw[.]run_bike >/dev/null && echo RUNNING || echo not running)\"; \
      echo \"bundle:   \$(md5sum deploy/bundle.npz 2>/dev/null | cut -d' ' -f1)\"; \
      ls /dev/serial/by-id/ 2>/dev/null | sed 's/^/port:     /'"
    echo "local:    $local_md5"
    # The mirror's own failure mode, tested from THIS terminal.
    python - "${PI#*@}" <<'PY'
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.sendto(b"", (socket.gethostbyname(sys.argv[1]), 9))
    print("udp:      this terminal can reach the Pi")
except OSError as e:
    print(f"udp:      BLOCKED from this terminal ({e}) -- the mirror will not "
          "connect; run it from Terminal.app")
PY
    ;;
  ""|-h|--help|help) usage ;;
  *) echo "unknown command: $cmd" >&2; usage 1 ;;
esac
