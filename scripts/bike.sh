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
# THE U2D2 PORT. run_bike defaults to config/onboard.yaml's dxl_port, a by-id path
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
# WAITING FOR THE PI. `bench`/`run` started while the Pi boots used to sit
# silent: ssh's connect retries back off (about 1, 3, 7, 15, 31 s), so a Pi
# that came up just after one waited for the next, and every ssh by name paid
# ~1 s of mDNS (1.11 s against 0.12 s by IP, measured 2026-10-20). So they
# first poll the Pi's ssh port every 0.25 s -- at the last address it had
# (~/.cache/aow/pi-ip) and at whatever the name resolves to -- print how long
# it took, then connect BY THAT IP with HostKeyAlias set to the name: the
# known_hosts key for aowbike.local still has to match, so a stale address
# now held by some other machine fails loudly rather than connecting.
#
# `sync` is the runbook's rsync (pi-bench-bringup.md section 7) WITHOUT
# --delete: the Pi's tree has policy files and bench logs that exist only
# there, and --delete would remove them. Stale files left on the Pi are
# harmless; the bundle's digest check catches a stale bundle.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

PI="${PI:-efun@aowbike.local}"
PI_NAME="${PI#*@}"
SSH_OPTS=()
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
p = yaml.safe_load(open("config/onboard.yaml"))
print(p.get("dxl_port", ""))
PY
)"
  got="$(ssh ${SSH_OPTS[@]+"${SSH_OPTS[@]}"} "$PI" "test -e '$want' && echo present; ls /dev/serial/by-id/ 2>/dev/null | grep -i ftdi || true")"
  if grep -q '^present$' <<<"$got"; then return 0; fi
  local ftdi; ftdi="$(grep -iv '^present$' <<<"$got" | grep -i ftdi || true)"
  if [[ -n "$ftdi" && "$(wc -l <<<"$ftdi")" -eq 1 ]]; then
    echo "note: configured U2D2 ($(basename "$want")) is not plugged in;" \
         "using the one that is: $ftdi" >&2
    echo "--port /dev/serial/by-id/$ftdi"
  fi
}

# -> the Pi's IPv4 address once its ssh port answers (progress on stderr).
wait_pi() {
  python - "${PI#*@}" "$HOME/.cache/aow/pi-ip" <<'PY'
import errno, os, socket, sys, threading, time
host, cache = sys.argv[1], sys.argv[2]
t0, found = time.monotonic(), [None]
try:
    cached = open(cache).read().strip() or None
except OSError:
    cached = None

def resolve():        # getaddrinfo blocks ~5 s per miss on .local: off-loop
    while True:
        try:
            found[0] = socket.getaddrinfo(host, 22, socket.AF_INET,
                                          socket.SOCK_STREAM)[0][4][0]
        except OSError:
            pass
        time.sleep(0.5)
threading.Thread(target=resolve, daemon=True).start()

# What each address answered, as runs: "timeout 0-5 s, EHOSTDOWN 5-24 s".
# EHOSTDOWN is macOS refusing locally -- after ~5 s of unanswered ARP it
# marks the address down for net.link.ether.inet.host_down_time (20 s)
# and stops asking -- so a Pi that came up inside that window waits it out.
runs = {}

def ssh_open(ip):
    s = socket.socket()
    s.settimeout(0.25)
    try:
        s.connect((ip, 22))
        r = "open"
    except socket.timeout:
        r = "timeout"
    except OSError as e:
        r = errno.errorcode.get(e.errno, str(e.errno))
    finally:
        s.close()
    t = time.monotonic() - t0
    seq = runs.setdefault(ip, [])
    if seq and seq[-1][0] == r:
        seq[-1][2] = t
    else:
        seq.append([r, t, t])
    return r == "open"

shown = False
while time.monotonic() - t0 < 180:
    for ip in dict.fromkeys(x for x in (found[0], cached) if x):
        if ssh_open(ip):
            if shown:
                print(f"\r{host} up after {time.monotonic() - t0:.1f} s ({ip})   ",
                      file=sys.stderr)
                for a, seq in runs.items():
                    print(f"  {a}: " + ", ".join(f"{r} {b:.1f}-{e:.1f} s"
                          for r, b, e in seq[:-1]), file=sys.stderr)
            os.makedirs(os.path.dirname(cache), exist_ok=True)
            open(cache, "w").write(ip)
            print(ip)
            sys.exit(0)
    if time.monotonic() - t0 > 0.5:
        dots = "." * (1 + int((time.monotonic() - t0) * 2) % 3)
        print(f"\rwaiting for {host} {time.monotonic() - t0:4.0f} s{dots:<3}",
              end="", file=sys.stderr, flush=True)
        shown = True
    time.sleep(0.25)
print(f"\n{host}: no ssh after 180 s", file=sys.stderr)
sys.exit(1)
PY
}

run_bike() {
  local ip; ip="$(wait_pi)"
  # by address, checked against the name's host key (see WAITING FOR THE PI)
  PI="${PI%@*}@$ip"; SSH_OPTS=(-o "HostKeyAlias=$PI_NAME")
  local extra=""
  if ! has_port_arg "$@"; then extra="$(port_fallback)"; fi
  local args; args="$(printf ' %q' "$@")"
  exec ssh -t "${SSH_OPTS[@]}" "$PI" "cd ~/$PI_REPO && $PI_PY -m aow_sim.hw.run_bike $extra$args"
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
  mirror) exec mjpython -m aow_sim.run_drive --mirror "$PI_NAME" "$@" ;;
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
