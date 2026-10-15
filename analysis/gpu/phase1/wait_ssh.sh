#!/bin/bash
# Wait for a new pod's DIRECT ssh and log when each stage came up, measured
# from the pod's createdAt. Run it in the BACKGROUND as soon as get-pod shows
# the 22/tcp mapping (ip + public port), so the wait never blocks the session.
#
#   wait_ssh.sh <ip> <port> <createdAt epoch s> [out.jsonl]
#
# Why not the proxy (ssh.runpod.io): it wants an interactive pty (-tt) and
# did not return cleanly to a scripted command on 2026-10-14; the loop over
# it sat until a user message pushed it to the background. Direct only.
IP=$1; PORT=$2; T0=$3; OUT=${4:-/dev/stdout}
now() { date +%s; }
until nc -z -w 2 "$IP" "$PORT" 2>/dev/null; do sleep 1; done
t_tcp=$(now)
until ssh -o BatchMode=yes -o ConnectTimeout=3 -o StrictHostKeyChecking=accept-new \
        -p "$PORT" root@"$IP" true 2>/dev/null; do sleep 1; done
t_ssh=$(now)
echo "{\"step\":\"boot\",\"tcp_open_s\":$((t_tcp - T0)),\"ssh_ok_s\":$((t_ssh - T0))}" | tee -a "$OUT"
