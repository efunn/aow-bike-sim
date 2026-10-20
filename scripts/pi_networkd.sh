#!/usr/bin/env bash
# The Pi's WiFi without NetworkManager: systemd-networkd + wpa_supplicant.
# Runs ON the Pi, as root, from here:
#
#   ssh efun@aowbike.local 'sudo bash -s' < scripts/pi_networkd.sh            # switch
#   ssh efun@aowbike.local 'sudo bash -s' < scripts/pi_networkd.sh rollback   # back to NM
#   ssh efun@aowbike.local 'sudo WIFI_FREQ=2417 bash -s' < scripts/pi_networkd.sh  # + channel pin
#
# WIFI_FREQ (MHz) pins wpa_supplicant's scan to the access point's channel:
# it skips scanning every other channel, and NEVER FINDS the network if the
# router moves -- so only with the router's channel fixed too (it is: 2,
# 2417 MHz, set by hand 2026-10-20; away from a microwave's ~2.45 GHz).
#
# then `sudo reboot`. Nothing changes until the reboot: NetworkManager keeps
# the link this ssh is on, so the switch cannot cut its own session.
#
# WHY (2026-10-20, journal on the Pi 3B+, seconds from kernel start):
# NetworkManager started at 9.1 s, then sat 2.9 s on its own netplan round
# trip (`netplan generate`, then a systemd reload: 1.48 s), then 2.5 s more
# with the radio ready before it began to connect. WiFi had its address at
# 16.75 s. wpa_supplicant and networkd do the same job without either.
#
# The WiFi credentials come from NetworkManager's own saved connection
# (ATA.nmconnection), hashed by wpa_passphrase into a root-only file. The
# passphrase never leaves the Pi and is never printed.
#
# Also:
#  * WiFi POWER SAVE. NetworkManager was what turned it off (the
#    connection's powersave=2); brcmfmac defaults it ON, which adds latency
#    to the control link. A oneshot unit turns it off once wpa_supplicant is
#    up. Check after the reboot: `/usr/sbin/iw dev wlan0 get power_save`.
#  * eth0 gets DHCP AND link-local, so a cable straight to the Mac still
#    reaches `aowbike.local` if WiFi never comes up.
#  * systemd-networkd-wait-online is left disabled: with eth0 unplugged it
#    would wait out its timeout for anything ordered after network-online.
#  * NetworkManager's files are left in place; `rollback` re-enables it.
set -euo pipefail

[[ $(id -u) -eq 0 ]] || { echo "run as root (sudo bash -s)" >&2; exit 1; }
KEYFILE=/etc/NetworkManager/system-connections/ATA.nmconnection
WPA=/etc/wpa_supplicant/wpa_supplicant-wlan0.conf
NET=/etc/systemd/network
PS=/etc/systemd/system/wifi-powersave-off.service

if [[ "${1:-}" == rollback ]]; then
  systemctl disable systemd-networkd systemd-networkd.socket \
    wpa_supplicant@wlan0 wifi-powersave-off 2>/dev/null || true
  systemctl enable NetworkManager NetworkManager-wait-online \
    NetworkManager-dispatcher wpa_supplicant
  echo "rolled back to NetworkManager; reboot to apply"
  exit 0
fi

[[ -f $KEYFILE ]] || { echo "no $KEYFILE" >&2; exit 1; }
[[ -x /usr/sbin/iw ]] || { echo "no /usr/sbin/iw" >&2; exit 1; }
ssid="$(sed -n 's/^ssid=//p' "$KEYFILE" | head -1)"
grep -q '^key-mgmt=wpa-psk' "$KEYFILE" || { echo "not WPA-PSK" >&2; exit 1; }
grep -q '^psk=' "$KEYFILE" || { echo "no saved psk" >&2; exit 1; }

# the network block, hashed; passphrase on stdin, the plaintext comment dropped
block="$(sed -n 's/^psk=//p' "$KEYFILE" | head -1 | wpa_passphrase "$ssid" \
  | grep -v '^[[:space:]]*#psk=')"
if [[ -n "${WIFI_FREQ:-}" ]]; then
  [[ "$WIFI_FREQ" =~ ^[0-9]{4}$ ]] || { echo "WIFI_FREQ is MHz, e.g. 2417" >&2; exit 1; }
  block="${block%\}}	scan_freq=$WIFI_FREQ
	freq_list=$WIFI_FREQ
}"
fi
umask 077
cat > "$WPA" <<EOF
ctrl_interface=DIR=/run/wpa_supplicant GROUP=netdev
update_config=0
country=CA
$block
EOF
chmod 600 "$WPA"
umask 022

mkdir -p "$NET"
cat > "$NET/25-wlan0.network" <<'EOF'
[Match]
Name=wlan0

[Network]
DHCP=ipv4
EOF
cat > "$NET/20-eth0.network" <<'EOF'
# Recovery: a cable to the router (DHCP) or straight to the Mac (link-local).
[Match]
Name=eth0

[Network]
DHCP=ipv4
LinkLocalAddressing=yes
EOF

cat > "$PS" <<'EOF'
[Unit]
Description=WiFi power save off (brcmfmac defaults it on)
After=wpa_supplicant@wlan0.service sys-subsystem-net-devices-wlan0.device
BindsTo=sys-subsystem-net-devices-wlan0.device

[Service]
Type=oneshot
ExecStart=/usr/sbin/iw dev wlan0 set power_save off

[Install]
WantedBy=multi-user.target
EOF

# DNS. Without systemd-resolved (not installed) networkd does not write
# resolv.conf, and NetworkManager leaves it holding only its header. So a
# static one: this LAN's router (from the gateway on the current route),
# then a public fallback. Only apt / pip need it; the bike runs on IPs.
gw="$(ip -4 route show default | awk '{print $3; exit}')"
printf '# static (scripts/pi_networkd.sh): networkd without resolved\nnameserver %s\nnameserver 1.1.1.1\n' \
  "${gw:-192.168.0.1}" > /etc/resolv.conf

# test (1)'s channel pin, undone, so a rollback gets the connection as it was
nmcli con mod "$(basename "$KEYFILE" .nmconnection)" \
  802-11-wireless.channel "" 802-11-wireless.band "" || true

systemctl daemon-reload
systemctl disable NetworkManager NetworkManager-wait-online \
  NetworkManager-dispatcher wpa_supplicant
systemctl enable systemd-networkd wpa_supplicant@wlan0 wifi-powersave-off
# AFTER the enable: enabling networkd pulls its wait-online in with it
systemctl disable systemd-networkd-wait-online
echo "switched to systemd-networkd + wpa_supplicant@wlan0; reboot to apply"
