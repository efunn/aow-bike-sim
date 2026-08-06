#!/usr/bin/env bash
# Detached RL training + tensorboard. Nothing here touches the RL code -- it
# only launches `python -m aow_sim.train_<move>_rl` and `tensorboard`, records
# their pids under runs/, and kills them.
#
#   ./scripts/rl.sh up general              # board + training, then disconnect
#   ./scripts/rl.sh board general           # dashboard only (outlives training)
#   ./scripts/rl.sh train general --resume  # extra args go straight to the trainer
#   ./scripts/rl.sh status                  # what is up, and on which port
#   ./scripts/rl.sh logs general            # tail -f the live training log
#   ./scripts/rl.sh stop general            # cancel training, leave the board up
#   ./scripts/rl.sh stop-board general      # kill the board, leave training up
#
# Both halves are started with nohup (+ setsid where available), so they outlive
# the ssh session and each other. Logs are timestamped per launch and never
# overwritten; this script deletes nothing under runs/.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

CONDA_ENV="${CONDA_ENV:-aow-sim}"
MOVES="general flick pivot ball"

# Per-move dashboard port, so two moves can be watched at once.
# Override with:  PORT=6010 ./scripts/rl.sh board general
port_for() {
  case "$1" in
    general) echo 6006 ;;
    flick)   echo 6007 ;;
    pivot)   echo 6008 ;;
    ball)    echo 6009 ;;
  esac
}

die() { echo "error: $*" >&2; exit 1; }

check_move() {
  [[ -n "${1:-}" ]] || die "usage: $0 $SUB <${MOVES// /|}> [args...]"
  [[ -n "$(port_for "$1")" ]] || die "unknown move '$1' (want: $MOVES)"
}

run_dir()  { echo "$REPO/runs/$1_rl"; }
pid_file() { echo "$(run_dir "$1")/.$2.pid"; }   # $2 = train | board

activate_env() {
  [[ "${CONDA_DEFAULT_ENV:-}" == "$CONDA_ENV" ]] && return 0
  local base
  if base="$(conda info --base 2>/dev/null)" && [[ -f "$base/etc/profile.d/conda.sh" ]]; then
    # shellcheck disable=SC1091
    source "$base/etc/profile.d/conda.sh"
    conda activate "$CONDA_ENV" || die "conda env '$CONDA_ENV' not found (override with CONDA_ENV=...)"
  else
    echo "note: conda env '$CONDA_ENV' not activated, using the current python" >&2
  fi
}

# The pidfile holds "<mode> <pid>". mode=group means the process is its own
# process-group leader (setsid), so one signal reaches SB3's SubprocVecEnv
# workers too; mode=tree means we walk the children ourselves instead.
alive() {   # alive <mode> <pid>
  case "$1" in
    group) kill -0 -- "-$2" 2>/dev/null ;;
    *)     kill -0 "$2" 2>/dev/null ;;
  esac
}

# Echoes "<mode> <pid>" if that half is still running; clears a stale pidfile.
running() {
  local f; f="$(pid_file "$1" "$2")"
  [[ -f "$f" ]] || return 1
  local mode pid; read -r mode pid < "$f"
  [[ -n "${pid:-}" ]] || { rm -f "$f"; return 1; }
  if alive "$mode" "$pid"; then echo "$mode $pid"; return 0; fi
  rm -f "$f"; return 1
}

pid_of() { running "$1" "$2" | awk '{print $2}'; }

signal_tree() {   # signal_tree <sig> <pid> -- children first, then the parent
  local sig=$1 pid=$2 child
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do signal_tree "$sig" "$child"; done
  kill -"$sig" "$pid" 2>/dev/null || true
}

signal_job() {    # signal_job <sig> <mode> <pid>
  case "$2" in
    group) kill -"$1" -- "-$3" 2>/dev/null || true ;;
    *)     signal_tree "$1" "$3" ;;
  esac
}

# start <move> <kind> <logname> -- <command...>   (echoes the logfile path)
start() {
  local move=$1 kind=$2 logname=$3; shift 4      # the 4th arg is the literal --
  local dir log mode pid
  dir="$(run_dir "$move")"
  log="$dir/logs/$logname-$(date +%Y%m%d-%H%M%S).log"
  mkdir -p "$dir/logs"

  if running "$move" "$kind" >/dev/null; then
    die "$kind for '$move' already running (pid $(pid_of "$move" "$kind")); stop it first"
  fi

  if command -v setsid >/dev/null 2>&1; then
    setsid nohup "$@" >"$log" 2>&1 < /dev/null &
    mode=group
  else
    nohup "$@" >"$log" 2>&1 < /dev/null &
    mode=tree
  fi
  pid=$!
  echo "$mode $pid" > "$(pid_file "$move" "$kind")"
  ln -sfn "$log" "$dir/logs/$logname-latest.log"

  sleep 2
  if ! alive "$mode" "$pid"; then
    echo "--- $kind died on startup ---" >&2
    tail -20 "$log" >&2
    rm -f "$(pid_file "$move" "$kind")"
    die "see $log"
  fi
  echo "$log"
}

board_url() {
  local ip
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')" || true
  [[ -n "${ip:-}" ]] || ip="$(hostname)"
  echo "http://$ip:$1"
}

cmd_board() {
  local move=$1 port log
  port="${PORT:-$(port_for "$move")}"
  if running "$move" board >/dev/null; then
    echo "tensorboard  -> already up at $(board_url "$port")  (port $port)"
    return 0
  fi
  activate_env
  log="$(start "$move" board tensorboard -- \
    tensorboard --logdir "$(run_dir "$move")" --bind_all --port "$port")"
  echo "tensorboard  -> $(board_url "$port")        <-- port $port"
  echo "                or tunnel it: ssh -N -L $port:localhost:$port <this-host>"
  echo "                log: $log     stop: ./scripts/rl.sh stop-board $move"
}

cmd_train() {
  local move=$1; shift
  activate_env
  local log
  log="$(start "$move" train train -- python -u -m "aow_sim.train_${move}_rl" "$@")"
  echo "training     -> aow_sim.train_${move}_rl ${*:-(config defaults)}, pid $(pid_of "$move" train)"
  echo "                log: $log"
  echo "                tail: ./scripts/rl.sh logs $move    cancel: ./scripts/rl.sh stop $move"
}

cmd_up() {
  local move=$1; shift
  cmd_board "$move"
  cmd_train "$move" "$@"
}

# stop <move> <kind>: TERM the job, escalate to KILL after ~10s.
stop() {
  local move=$1 kind=$2 mode pid job i
  if ! job="$(running "$move" "$kind")"; then
    echo "no $kind running for '$move'"; return 0
  fi
  read -r mode pid <<< "$job"
  signal_job TERM "$mode" "$pid"
  for i in $(seq 20); do
    alive "$mode" "$pid" || break
    sleep 0.5
  done
  if alive "$mode" "$pid"; then
    echo "$kind (pid $pid) ignored SIGTERM, sending SIGKILL"
    signal_job KILL "$mode" "$pid"
    sleep 1
  fi
  rm -f "$(pid_file "$move" "$kind")"
  echo "stopped $kind for '$move' (pid $pid)"
}

cmd_status() {
  local move kind job pid detail any=0
  printf '%-9s %-6s %-8s %s\n' MOVE WHAT PID DETAIL
  for move in $MOVES; do
    for kind in train board; do
      if job="$(running "$move" "$kind")"; then
        any=1
        pid="$(echo "$job" | awk '{print $2}')"
        if [[ $kind == board ]]; then
          detail="$(board_url "$(port_for "$move")")"
        else
          detail="$(run_dir "$move")/logs/train-latest.log"
        fi
        printf '%-9s %-6s %-8s %s\n' "$move" "$kind" "$pid" "$detail"
      fi
    done
  done
  [[ $any == 1 ]] || echo "(nothing running)"
}

SUB="${1:-}"; shift || true
case "$SUB" in
  up)         check_move "${1:-}"; cmd_up "$@" ;;
  train)      check_move "${1:-}"; cmd_train "$@" ;;
  board)      check_move "${1:-}"; cmd_board "$1" ;;
  stop)       check_move "${1:-}"; stop "$1" train ;;
  stop-board) check_move "${1:-}"; stop "$1" board ;;
  logs)       check_move "${1:-}"; tail -f "$(run_dir "$1")/logs/train-latest.log" ;;
  status)     cmd_status ;;
  *)          awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0" >&2; exit 1 ;;
esac
