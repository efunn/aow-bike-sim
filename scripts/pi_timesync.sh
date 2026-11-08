#!/usr/bin/env bash
# The Pi's first clock sync, ~30 s sooner. Runs ON the Pi, as root, from here:
#
#   ssh efun@aowbike.local 'sudo bash -s' < scripts/pi_timesync.sh            # apply
#   ssh efun@aowbike.local 'sudo bash -s' < scripts/pi_timesync.sh rollback   # undo
#
# WHY (2026-11-07, journal of two boots, seconds from kernel start): wifi has
# its DHCP address at 10.8 s, ssh is in at 12 s, and timesyncd's first sync
# lands at 42 s both times. It tried at 7 s, before the network, failed, and
# then waits ConnectionRetrySec (default 30 s) before trying a server again;
# networkd's "configuration changed" at 10.8 s does not cut that short. A
# run started in between is stamped with the clock restored from the last
# shutdown (261105-143426_run: 15 s after boot, 47 h early; the clock it
# restored is only saved every 5 min, so it is never right after a reboot).
#
# 2 s: a retry every 2 s until the first server answers. Takes effect on the
# restart below; nothing else about timesyncd changes.
set -euo pipefail
[[ $(id -u) -eq 0 ]] || { echo "run as root (sudo bash -s)" >&2; exit 1; }
CONF=/etc/systemd/timesyncd.conf.d/10-aow-retry.conf

if [[ "${1:-}" == rollback ]]; then
  rm -f "$CONF"
  systemctl restart systemd-timesyncd
  echo "removed $CONF; timesyncd back to its 30 s retry"
  exit 0
fi

mkdir -p "$(dirname "$CONF")"
cat > "$CONF" <<'EOF'
# scripts/pi_timesync.sh: retry a server every 2 s, not 30, so the first
# sync follows the wifi by seconds after a boot.
[Time]
ConnectionRetrySec=2
EOF
systemctl restart systemd-timesyncd
echo "wrote $CONF; timesyncd restarted"
