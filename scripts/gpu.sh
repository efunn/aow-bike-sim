#!/usr/bin/env bash
# GPU training on a remote machine, from this checkout: the runbook in
# docs/plans/gpu-training.md (phase 3, "The next run") as commands. The
# machine is anything reachable by ssh as root with an NVIDIA GPU: a rented
# RunPod pod (image runpod/pytorch:1.0.2-cu1281-torch280-ubuntu2404, the
# only setup tested) or your own box. Its driver must offer CUDA >= 13.0 --
# Warp 1.18 otherwise falls back to the CPU (pod_setup.sh asserts it).
#
#   scripts/gpu.sh check  HOST PORT                 # CUDA works? pypi / github speed?
#   scripts/gpu.sh up     HOST PORT [VAR=VAL ...]   # bundle, copy, launch (detached)
#   scripts/gpu.sh status HOST PORT                 # one table: a row per seed, config, ETA
#   scripts/gpu.sh watch  HOST PORT                 # that table at each checkpoint, until done
#   scripts/gpu.sh pull   HOST PORT TAG             # results -> analysis/gpu/phase3/results_TAG
#   scripts/gpu.sh export TAG [STEPS_M ...]         # checkpoints -> moves/general_puffer_TAG[_s<k>]_<N>M
#
# VAR=VAL for `up` (all optional; the defaults are the last good run's):
#   SEEDS=4          four full runs at once under MPS (default 1). On a 5090:
#                    1 run ~29 min for 80M at ~51k steps/s; 4 runs ~46 min
#                    at ~29k each (116k in all); 8 runs 127k in all, 16k each
#   CONFIG=config/rl/rl_general_v3.yaml   the env config (every seed)
#   CONFIGS=a.yaml,b.yaml,...             one config per seed instead (SEEDS
#                    of them, comma-separated, one obs width)
#   NW=4096 HORIZON=64 CURRICULUM_LEAD ENV_SEED=0 PUFFER_SEED=42 TRAIN_S MPS=1
#   TRAIN_S is a wall cap (3600 s one run, 7200 several) that cuts a run
#   short of the ini's 80M on a card under ~23k steps/s: an A40 on the v4
#   plant needed TRAIN_S=5400 (2026-10-26)
#
# NOT here: creating and terminating the pod. That is the RunPod console or
# its API (no key on this machine); see the runbook for the pod spec. Rules
# that cost money when skipped: confirm the card and $/h first; `check`
# before `up` and replace a slow host (seen: ~280 KB/s to pypi, EUR-NO-1,
# 2026-10-23); TERMINATE, never stop, when done.
#
# A run is NOT repeatable even at one seed (float atomics, MJWarp's
# constraint rows); a second launch on the same machine needs its results/
# moved aside first.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
P3=analysis/gpu/phase3
PASS="SEEDS CONFIG CONFIGS NW HORIZON CURRICULUM_LEAD ENV_SEED PUFFER_SEED TRAIN_S MPS SKIP_CHECKS"

usage() { sed -n '2,36p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
remote() {  # remote HOST PORT CMD
    ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 \
        -p "$2" root@"$1" "$3"
}

cmd=${1:-}; shift || true
case "$cmd" in
check)
    [ $# -ge 2 ] || usage
    remote "$1" "$2" '
        nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
        python3 -c "import torch; torch.zeros(1).cuda(); print(\"cuda ok\")" \
            || echo "CUDA UNUSABLE -- replace the machine"
        U=$(python3 -c "import json,urllib.request;d=json.load(urllib.request.urlopen(\"https://pypi.org/pypi/numpy/2.1.0/json\"));print([u[\"url\"] for u in d[\"urls\"] if \"cp312-cp312-manylinux_2_17_x86\" in u[\"filename\"]][0])")
        curl -s -o /dev/null -m 20 -w "pythonhosted %{speed_download} B/s\n" "$U" || true
        curl -s -o /dev/null -m 15 -L -w "github %{speed_download} B/s\n" \
            https://github.com/NVIDIA/warp/archive/refs/tags/v1.8.0.tar.gz || true'
    echo "(good: pythonhosted tens of MB/s; under ~1 MB/s, replace the host)"
    ;;
up)
    [ $# -ge 2 ] || usage
    HOST=$1 PORT=$2; shift 2
    ENVS=""            # TRAIN_S defaults on the machine: 3600 s one run, 7200 several
    for kv in "$@"; do
        k=${kv%%=*}
        [[ " $PASS " == *" $k "* ]] || { echo "unknown VAR $k (one of: $PASS)"; exit 1; }
        ENVS="$ENVS $kv"
    done
    TMP=$(mktemp -d)
    bash analysis/gpu/bundle.sh "$TMP/bundle.tgz"
    # the newest Warp kernel cache pulled from any run: a card of another
    # architecture just compiles once (~1 min)
    CACHE=$(ls -t $P3/results_*/warp-cache.tgz 2>/dev/null | head -1 || true)
    scp -q -o BatchMode=yes -P "$PORT" "$TMP/bundle.tgz" ${CACHE:+"$CACHE"} root@"$HOST":/root/
    [ -n "$CACHE" ] && [ "$(basename "$CACHE")" != warp-cache.tgz ] && \
        remote "$HOST" "$PORT" "mv ~/$(basename "$CACHE") ~/warp-cache.tgz"
    remote "$HOST" "$PORT" "[ ! -e ~/aow/results ] || { echo 'results/ exists: move it aside first'; exit 1; }
        mkdir -p ~/aow && cd ~/aow && tar xzf ~/bundle.tgz 2>/dev/null
        ($ENVS nohup bash $P3/pod_launch.sh > launch.log 2>&1 < /dev/null &)
        echo launched: ${ENVS:-defaults}"
    rm -rf "$TMP"
    echo "next: scripts/gpu.sh watch $HOST $PORT"
    ;;
status|watch)
    [ $# -ge 2 ] || usage
    # piped, not run from the bundle, so a pod launched before status.sh
    # existed reads the same way
    F=""; [ "$cmd" = watch ] && F=--follow
    ssh -o BatchMode=yes -o ConnectTimeout=10 -p "$2" root@"$1" "bash -s -- $F" \
        < $P3/status.sh
    ;;
pull)
    [ $# -ge 3 ] || usage
    D=$P3/results_$3
    [ ! -e "$D" ] || { echo "$D exists; pick another TAG"; exit 1; }
    mkdir -p "$D"
    rsync -a -e "ssh -o BatchMode=yes -p $2" root@"$1":aow/results/ "$D"/
    echo "pulled to $D"; du -sh "$D"
    echo "next: scripts/gpu.sh export $3   (then TERMINATE the pod)"
    ;;
export)
    [ $# -ge 1 ] || usage
    TAG=$1; shift
    STEPS=${*:-52 72 80}
    D=$P3/results_$TAG
    [ -d "$D" ] || { echo "no $D (scripts/gpu.sh pull first)"; exit 1; }
    # one run: results_TAG/{puffer_checkpoints,checkpoints}; several:
    # results_TAG/seed_<k>/ with the ini and graph json beside the checkpoints
    if ls -d "$D"/seed_* >/dev/null 2>&1; then RUNS=$(ls -d "$D"/seed_*); else RUNS=$D; fi
    for R in $RUNS; do
        # the NEWEST run folder holding checkpoints: a relaunch on the same
        # machine leaves the stopped run's (often empty) folder beside it
        CK=$(for d in $(ls -d "$R"/puffer_checkpoints/aow_bike/*/ 2>/dev/null | sort -r); do
                 ls "$d"*.bin >/dev/null 2>&1 && { echo "$d"; break; }; done)
        [ -n "$CK" ] || { echo "$R: no checkpoints"; continue; }
        if [ "$R" = "$D" ]; then
            META=$(ls "$D"/checkpoints/aow_bike_*.json | head -1); INI=$D/checkpoints/aow_bike.ini; SFX=""
        else
            META=$(ls "$R"/aow_bike_*.json | head -1); INI=$R/aow_bike.ini; SFX="_s${R##*seed_}"
        fi
        for M in $STEPS; do
            # the checkpoint whose step count is nearest M million
            BIN=$(ls "$CK"*.bin | awk -v m="$M" '{n=$0; sub(/.*\//,"",n); sub(/\.bin/,"",n);
                     d=n/1e6-m; if(d<0)d=-d; print d, $0}' | sort -n | head -1 | cut -d' ' -f2)
            NAME=general_puffer_${TAG}${SFX}_${M}M
            if ls moves/$NAME.npz moves/archive/$NAME.npz >/dev/null 2>&1; then
                echo "$NAME exists (moves/ or moves/archive/): skipped"; continue
            fi
            echo "== $NAME <- $(basename "$BIN")"
            python -m aow_sim.gpu.export_policy "$BIN" --graph-meta "$META" --ini "$INI" \
                --name "$NAME" | tail -1 | cut -c1-160
        done
    done
    echo "judge: python analysis/policy/per_command.py --summary --policies general_puffer_fixes_72M <names>"
    echo "       python analysis/gpu/phase3/standstill.py <names>"
    ;;
*) usage ;;
esac
