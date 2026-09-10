#!/usr/bin/env bash
# Detached RL training + tensorboard. Nothing here touches the RL code -- it
# only launches `python -m aow_sim.train_<move>_rl` and `tensorboard`, records
# their pids under runs/, and kills them.
#
#   ./scripts/rl.sh up general              # board + training, then disconnect
#   ./scripts/rl.sh seeds general 4         # seeds 0-3, serial, one board
#   ./scripts/rl.sh seeds general 2-11      # seeds 2..11 (skip ones already run)
#   ./scripts/rl.sh board general           # dashboard only (outlives training)
#   ./scripts/rl.sh train general --resume  # extra args go straight to the trainer
#   ./scripts/rl.sh status                  # what is up, and on which port
#   ./scripts/rl.sh eta general             # cpu/gpu, fps, % done, time left
#   ./scripts/rl.sh logs general            # tail -f the live training log
#   ./scripts/rl.sh stop general            # cancel training, leave the board up
#   ./scripts/rl.sh stop-board general      # kill the board, leave training up
#   ./scripts/rl.sh sync                    # fast-forward to origin, keeping artifacts
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

# Who is listening on a TCP port, if anyone. lsof works on both macOS and
# Linux; ss is the Linux fallback for containers without lsof.
port_pids() {
  if command -v lsof >/dev/null 2>&1; then
    lsof -ti :"$1" -sTCP:LISTEN 2>/dev/null || true
  elif command -v ss >/dev/null 2>&1; then
    ss -lptnH "sport = :$1" 2>/dev/null |
      grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u || true
  fi
}

check_move() {
  [[ -n "${1:-}" ]] || die "usage: $0 $SUB <${MOVES// /|}> [args...]"
  [[ -n "$(port_for "$1")" ]] || die "unknown move '$1' (want: $MOVES)"
}

# RUN_TAG namespaces the BOOKKEEPING dir (pidfiles + launch logs) so several
# runs of the same move can be up at once -- which is what `seeds` needs. It is
# NOT the trainer's --run-dir; those have always been separate (the trainer
# writes wherever --run-dir says, this is only where the pid and the launch log
# live). Every subcommand honours it:
#
#     RUN_TAG=seed0 ./scripts/rl.sh logs general
#     RUN_TAG=seed0 ./scripts/rl.sh stop general
run_dir()  { echo "$REPO/runs/$1_rl${RUN_TAG:+_$RUN_TAG}"; }
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

  # Record the argv, so `eta` can see a --timesteps override after the fact.
  echo "# launched $(date '+%Y-%m-%d %H:%M:%S') :: $*" > "$log"
  if command -v setsid >/dev/null 2>&1; then
    setsid nohup "$@" >>"$log" 2>&1 < /dev/null &
    mode=group
  else
    nohup "$@" >>"$log" 2>&1 < /dev/null &
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

# Where the trainer is ACTUALLY writing tensorboard events. `--run-dir`
# redirects that, and this script passes it straight through to the trainer,
# so deriving the logdir from the move name alone points the board at an empty
# directory ("No dashboards are active for the current data set"). The launch
# argv recorded in the train log is the source of truth. LOGDIR overrides.
# Canonical absolute path, so the two sides of the stale-board comparison are
# always in the same form. `realpath` is not on every macOS by default and the
# directory may not exist yet (the board can start before the first rollout),
# so this resolves textually rather than requiring the path to exist.
canon_dir() {
  local d="$1"
  [[ -n "$d" ]] || return 0
  [[ "$d" = /* ]] || d="$REPO/$d"
  printf '%s\n' "${d%/}"
}

board_logdir() {
  local move=$1 log rd
  if [[ -n "${LOGDIR:-}" ]]; then canon_dir "$LOGDIR"; return; fi
  log="$(run_dir "$move")/logs/train-latest.log"
  if [[ -f "$log" ]]; then
    rd="$(sed -n 's/.*--run-dir[= ]\{1,\}\([^ ]*\).*/\1/p' "$log" | head -1)"
    if [[ -n "$rd" ]]; then canon_dir "$rd"; return; fi
  fi
  run_dir "$move"
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
  local want; want="$(board_logdir "$move")"
  local job
  if job="$(running "$move" board)"; then
    # `up` after a run with a new --run-dir leaves a board serving the OLD
    # directory, and it looks healthy -- a full dashboard of the wrong run.
    # Compare what it was actually launched with and restart on a mismatch.
    local bpid cur
    bpid="${job##* }"
    cur="$(canon_dir "$(ps -o command= -p "$bpid" 2>/dev/null |
           sed -n 's/.*--logdir[= ]\{1,\}\([^ ]*\).*/\1/p')")"
    if [[ -n "$cur" && "$cur" != "$want" ]]; then
      echo "tensorboard  -> logdir changed, restarting"
      echo "                was: $cur"
      echo "                now: $want"
      stop "$move" board >/dev/null 2>&1 || true
    else
      echo "tensorboard  -> already up at $(board_url "$port")  (port $port)"
      [[ -n "$cur" ]] && echo "                logdir: $cur"
      return 0
    fi
  fi
  # Someone is on the port but this script has no pidfile for it -- usually a
  # board that outlived its pidfile (the file is removed as soon as a liveness
  # check fails, and never restored). Starting a second one just dies with
  # "could not bind", which reads like a script bug rather than success.
  local held; held="$(port_pids "$port" | tr "\n" " ")"
  if [[ -n "${held// /}" ]]; then
    echo "tensorboard  -> ALREADY SERVING on port $port (pid ${held% })"
    echo "                $(board_url "$port")  -- try that first; it is probably fine"
    echo "                untracked by this script, so 'stop-board' cannot see it."
    echo "                take the port back:  kill ${held% }"
    echo "                or use another:      PORT=$((port + 1)) $0 board $move"
    return 0
  fi
  activate_env
  local dir="$want"
  log="$(start "$move" board tensorboard -- \
    tensorboard --logdir "$dir" --bind_all --port "$port")"
  echo "tensorboard  -> $(board_url "$port")        <-- port $port"
  echo "                logdir: $dir"
  compgen -G "$dir/*/events.out.tfevents.*" >/dev/null 2>&1 ||
    echo "                NOTE: no event files there yet -- the board will show" \
         "'No dashboards are active' until the first rollout finishes"
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
  echo "                eta: ./scripts/rl.sh eta $move     cancel: ./scripts/rl.sh stop $move"
}

# Launch the SAME config at N different seeds, ONE AFTER ANOTHER, under one
# board. Serial by design.
#
# WHY SERIAL. One PPO run already fits a box, because its two phases do not
# overlap: during COLLECTION the n_envs SubprocVecEnv workers are busy and the
# main process idles; during the UPDATE the main process is in torch and every
# worker is blocked at the barrier (`scripts/bench_update.py` documents this).
# Run two seeds at once and the phases are unsynchronised, so one seed's torch
# update lands on top of the other's MuJoCo workers -- bench_update.py measures
# that collision at 2-8x slowdown, non-monotonic across a thread sweep.
#
# Serial avoids all of it, and avoids the thing that would otherwise have to
# come with it: PPO's rollout buffer is `n_envs x n_steps` and every config here
# is built around 16384, so sharing a box between seeds would mean editing BOTH
# in the same edit (32/512 -> 8/2048 for four seeds) -- changing the algorithm
# to buy parallelism. Serial keeps every seed on the exact shape the configs
# were tuned at, which is also the only way the runs stay comparable to each
# other and to everything already in moves/.
#
# The cost is wall-clock: N seeds take N x one run. That is the trade.
#
# ONE BOARD FOR ALL SEEDS. tensorboard serves a PARENT directory and renders
# each subdirectory as its own run, so pointing it at <base>/ overlays every
# seed on one dashboard -- including the ones that have not started yet, which
# appear as they go.
#
# WHY SEEDS AT ALL. Directional personality -- which way a policy resolves a
# turn -- is set by the seed, not the config: rl_general_cmd_curriculum2 and
# _2b differ only in `algo.seed` and resolve 33% vs 71% of their turns forward.
# One run per config cannot tell a real effect from a lucky draw. See
# docs/plans/eval-score-rewrite.md, "The turn personality".
cmd_seeds() {
  local move=$1 spec=$2; shift 2
  # <N> means seeds 0..N-1; <A-B> means seeds A..B inclusive. The range form
  # exists because seeds already spent are not worth re-spending: seeds 0 and 1
  # are `general_rl_cmd_curriculum2` and `_2b`, so a follow-up sweep starts at 2
  # and its exports do not collide with them.
  local lo hi
  if [[ "$spec" =~ ^([0-9]+)-([0-9]+)$ ]]; then
    lo="${BASH_REMATCH[1]}"; hi="${BASH_REMATCH[2]}"
    (( hi >= lo )) || die "empty seed range '$spec'"
  elif [[ "$spec" =~ ^[0-9]+$ ]] && (( spec >= 1 )); then
    lo=0; hi=$(( spec - 1 ))
  else
    die "usage: $0 seeds <move> <N|A-B> [--config ...] [--run-dir BASE] [--export-name NAME]"
  fi
  local n=$(( hi - lo + 1 ))

  # --run-dir and --export-name become the BASE for per-seed names; anything
  # else passes through to the trainer untouched.
  local base="" name="" ; local -a rest=()
  while (( $# )); do
    case "$1" in
      --run-dir)       base="$2"; shift 2 ;;
      --run-dir=*)     base="${1#--run-dir=}"; shift ;;
      --export-name)   name="$2"; shift 2 ;;
      --export-name=*) name="${1#--export-name=}"; shift ;;
      --seed|--seed=*) die "seeds sets --seed itself; drop it" ;;
      *) rest+=("$1"); shift ;;
    esac
  done
  base="${base:-runs/${move}_seeds}"
  name="${name:-${move}_seed}"
  local tag; tag="$(basename "$base")"

  # Which config the trainer will actually read, for the ETA below.
  local cfg_file="config/rl_${move}.yaml" j
  local -a argv=("${rest[@]+"${rest[@]}"}")
  for ((j = 0; j < ${#argv[@]}; j++)); do
    case "${argv[j]}" in
      --config)   cfg_file="${argv[j+1]:-$cfg_file}" ;;
      --config=*) cfg_file="${argv[j]#--config=}" ;;
    esac
  done

  activate_env
  # Board FIRST and on the PARENT, before any run dir exists -- tensorboard
  # picks up subdirectories as they appear, so it need not wait for seed 0.
  LOGDIR="$base" cmd_board "$move"
  echo

  # ONE detached supervisor runs the whole chain, so the sweep survives the ssh
  # session as a unit and `stop` cancels the REST of it rather than just the
  # seed currently running. Built with printf %q so a config path with a space
  # cannot split.
  local q_move q_base q_name q_rest="" a
  printf -v q_move '%q' "$move"; printf -v q_base '%q' "$base"
  printf -v q_name '%q' "$name"
  for a in ${rest[@]+"${rest[@]}"}; do printf -v a '%q' "$a"; q_rest+=" $a"; done

  # ONE LINE, no embedded newlines. `start` records the argv as the log's FIRST
  # LINE and `cmd_eta` resolves --config and --timesteps by reading exactly that
  # line. A multi-line chain puts them on line 2+, so eta silently falls back to
  # config/rl_<move>.yaml and reports the percentage against the WRONG budget --
  # which is the quiet failure cmd_eta's own comment documents (a 12M run shown
  # as 6M, 2026-08-09). Observed again here before this was collapsed.
  # TRAP + BACKGROUND-AND-WAIT, not a plain foreground call. Two reasons, and
  # the second is a bug this cost:
  #
  #  1. bash DEFERS a trap while a FOREGROUND child runs, so a plain
  #     `python ...` would swallow the signal until the seed finished -- up to
  #     2.2 h late. `cmd & wait $!` lets the trap fire immediately.
  #  2. Without the trap, killing the current python makes the LOOP ADVANCE and
  #     spawn the next seed before bash itself is signalled -- orphaning it.
  #     Observed on macOS, where `setsid` does not exist so `stop` falls back to
  #     signal_tree (children first, then the parent) and loses that race. On
  #     Linux setsid gives a real process group and the group kill is atomic, so
  #     it would not have shown up on the training box at all.
  #
  # `exit 130` marks it cancelled rather than completed.
  local chain="trap 'kill \"\$child\" 2>/dev/null; exit 130' TERM INT; for i in \$(seq $lo $hi); do echo \"=== seed \$i of $lo..$hi : \$(date '+%F %T') ===\"; python -u -m aow_sim.train_${move}_rl --seed \"\$i\" --run-dir $q_base/seed\$i --export-name ${q_name}\$i$q_rest & child=\$!; wait \"\$child\" || echo \"seed \$i FAILED or cancelled -- code \$?\" >&2; done; echo \"=== sweep done : \$(date '+%F %T') ===\""

  # LOGNAME IS `train`, NOT `seeds`. Four places hardcode `train-latest.log` --
  # cmd_eta, cmd_status's detail line, board_logdir and the `logs` subcommand --
  # so any other name silently breaks all of them for a sweep. The supervisor's
  # output goes to the same place a single run's would, and every subcommand
  # keeps working unchanged.
  #
  # WHAT `eta` REPORTS FOR A SWEEP: the seed currently running, not the sweep.
  # The chain restarts SB3 per seed, so `total_timesteps` resets each time and
  # the percentage is the current seed's. Multiply by the seeds remaining, or
  # read the total off the launch banner above.
  local log
  log="$(RUN_TAG="$tag" start "$move" train "train" -- bash -c "$chain")"

  # ETA. 2500 steps/s is MEASURED, not assumed: derived from checkpoint mtimes
  # across general_rl_cmd_curriculum, _2 and _2b on the Threadripper (16C/32T),
  # which agree at 2514 / 2496 / 2518 steps/s -- 20M steps = 2.2 h per run. It
  # is a per-box constant; on a different machine read `./scripts/rl.sh eta`
  # once seed $lo is under way and rescale.
  # An explicit --timesteps BEATS the config, exactly as cmd_eta resolves it.
  # Reading only the config would print a budget the run is not using -- the
  # same wrong-denominator failure, one line higher up.
  local steps eta_h k
  for ((k = 0; k < ${#argv[@]}; k++)); do
    case "${argv[k]}" in
      --timesteps)   steps="${argv[k+1]:-}" ;;
      --timesteps=*) steps="${argv[k]#--timesteps=}" ;;
    esac
  done
  [[ -n "${steps:-}" ]] || steps="$(awk '/^algo:/{a=1} a&&/^[ \t]+total_timesteps:/{print $2; exit}' "$cfg_file" 2>/dev/null)"
  if [[ -n "$steps" ]]; then
    eta_h="$(awk -v s="$steps" -v n="$n" 'BEGIN{printf "%.1f", s*n/2500/3600}')"
    echo "sweep        -> $n seeds, SERIAL, seed $lo..$hi"
    echo "                ${steps} steps each, ~$(awk -v s="$steps" 'BEGIN{printf "%.1f", s/2500/3600}') h per seed"
    echo "                ~${eta_h} h total at 2500 steps/s (measured on the 16C/32T box)"
  else
    echo "sweep        -> $n seeds, SERIAL, seed $lo..$hi"
  fi
  echo "                each: $base/seed<i>, export ${name}<i>"
  echo "                pid $(RUN_TAG="$tag" pid_of "$move" train)  (one supervisor for the chain)"
  echo "                log: $log"
  echo
  echo "board        -> $(board_url "${PORT:-$(port_for "$move")}")  (all seeds, one dashboard)"
  echo "watch        -> RUN_TAG=$tag ./scripts/rl.sh logs $move"
  echo "progress     -> RUN_TAG=$tag ./scripts/rl.sh eta $move   (CURRENT seed, not the sweep)"
  echo "cancel ALL   -> RUN_TAG=$tag ./scripts/rl.sh stop $move"
  echo "compare      -> python analysis/per_command.py --metrics v_ach \\"
  echo "                  --policies ${name}${lo} ${name}$(( lo + 1 )) --tag seedsweep"
}

cmd_up() {
  local move=$1; shift
  # Resolve the board's logdir from the argv we are ABOUT TO LAUNCH, not from
  # the last run's train log. board_logdir parses --run-dir out of
  # train-latest.log, which at this instant still describes the PREVIOUS run --
  # so `up` used to point the board at the previous run's directory. That
  # failure is quiet and convincing: you get a full dashboard of the wrong run,
  # not the obvious "No dashboards are active".
  if [[ -z "${LOGDIR:-}" ]]; then
    local rd=""
    local -a argv=("$@")
    local i
    for ((i = 0; i < ${#argv[@]}; i++)); do
      case "${argv[i]}" in
        --run-dir=*) rd="${argv[i]#--run-dir=}" ;;
        --run-dir)   rd="${argv[i+1]:-}" ;;
      esac
    done
    [[ -n "$rd" ]] && export LOGDIR="$rd"
  fi
  cmd_board "$move"
  cmd_train "$move" "$@"
}

# stop <move> <kind>: TERM the job, escalate to KILL after ~10s.
stop() {
  local move=$1 kind=$2 mode pid job i
  if ! job="$(running "$move" "$kind")"; then
    echo "no $kind running for '$move'"
    # A board can outlive its pidfile (removed the moment a liveness check
    # fails), and then it is invisible here while still holding the port --
    # which shows up later as tensorboard "could not bind".
    if [[ "$kind" == board ]]; then
      local p held
      p="${PORT:-$(port_for "$move")}"
      held="$(port_pids "$p" | tr '\n' ' ')"
      [[ -n "${held// /}" ]] && echo "  ...but port $p is held by pid ${held% }" \
        "(untracked) -- kill ${held% } to free it"
    fi
    return 0
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

# Pull "<key> <value>" out of the last block SB3's verbose=1 logger printed:
#   | time/              |          |
#   |    fps             | 1077     |
# The `|| true` matters: under `set -o pipefail` a no-match grep would otherwise
# fail the assignment and `set -e` would kill the script before it can say that
# the first rollout simply hasn't landed yet.
last_metric() {   # last_metric <log> <key>
  tr -d ' ' < "$1" | grep "^|$2|" | tail -1 | cut -d'|' -f3 || true
}
first_metric() {
  tr -d ' ' < "$1" | grep "^|$2|" | head -1 | cut -d'|' -f3 || true
}
yaml_num() {      # yaml_num <file> <key> -- first scalar, comments/underscores stripped
  # [0-9][0-9]* rather than [0-9]\+ -- BSD sed's BRE has no \+
  sed -n "s/^[[:space:]]*$2:[[:space:]]*\([0-9_][0-9_]*\).*/\1/p" "$1" | head -1 | tr -d _ || true
}

cmd_eta() {
  local move=$1 log dev fps steps first budget hdr cfg rollout
  log="$(run_dir "$move")/logs/train-latest.log"
  [[ -f "$log" ]] || die "no training log for '$move' yet -- has it been started?"

  # The config the run was LAUNCHED with, not the one this move is NAMED
  # after. `train general --config config/rl_general_smooth_diff.yaml` is an
  # ordinary thing to do -- that is how the smooth/diff variants are trained --
  # and reading config/rl_general.yaml instead silently reports a DIFFERENT
  # run's budget. It is a quiet failure: you get a percentage and an ETA that
  # look fine and are computed against the wrong denominator (observed
  # 2026-08-09, a 12M run reported as 6M because rl_general.yaml still said
  # 6M). Fall back to the by-name config for logs written before the launch
  # line was recorded.
  hdr="$(head -1 "$log")"
  cfg="$(sed -n 's/.*--config[= ][= ]*\([^ ]*\).*/\1/p' <<< "$hdr")"
  [[ -n "$cfg" && "$cfg" != /* ]] && cfg="$REPO/$cfg"
  [[ -n "$cfg" && -f "$cfg" ]] || cfg="$REPO/config/rl_$move.yaml"
  echo "config       ${cfg#$REPO/}"

  dev="$(grep -m1 -o 'Using [^ ]* device' "$log" || true)"
  echo "device       ${dev:-unknown (no SB3 banner yet)}"
  if grep -q 'primarily intended to run on the CPU' "$log"; then
    echo "             ^ SB3 warns: PPO + MlpPolicy is CPU work; the GPU buys little here"
  fi

  fps="$(last_metric "$log" fps)"
  steps="$(last_metric "$log" total_timesteps)"
  if [[ -z "$fps" || -z "$steps" ]]; then
    echo "progress     no rollout table logged yet (first one lands after n_steps x n_envs steps)"
    return 0
  fi

  # Budget: an explicit --timesteps beats the config. On --resume SB3 adds the
  # already-done steps to the budget (base_class._setup_learn), so the target
  # sits that much higher; approximate the offset with the first logged count.
  # (`hdr` is read at the top, where it also resolves --config.)
  budget="$(sed -n 's/.*--timesteps[= ][= ]*\([0-9][0-9]*\).*/\1/p' <<< "$hdr")"
  [[ -n "$budget" ]] || budget="$(yaml_num "$cfg" total_timesteps)"
  [[ -n "$budget" ]] || die "no total_timesteps in $cfg and no --timesteps in the launch line"
  if grep -q -- '--resume' <<< "$hdr"; then
    first="$(first_metric "$log" total_timesteps)"
    rollout=$(( $(yaml_num "$cfg" n_steps) * $(yaml_num "$cfg" n_envs) ))
    budget=$(( budget + first - rollout ))
    echo "note         resumed run: target is config total + steps already done"
  fi

  local left
  left="$(awk -v f="$fps" -v s="$steps" -v b="$budget" 'BEGIN {printf "%d", (b - s) / f}')"

  awk -v fps="$fps" -v steps="$steps" -v budget="$budget" -v left="$left" \
      -v elapsed="$(last_metric "$log" time_elapsed)" 'BEGIN {
    fmt = "%dh%02dm";
    printf "throughput   %.0f steps/s\n", fps;
    printf "progress     %s / %s  (%.1f%%)\n", steps, budget, 100 * steps / budget;
    if (elapsed != "") printf "elapsed      " fmt "\n", elapsed / 3600, (elapsed % 3600) / 60;
    if (left > 0)      printf "remaining    " fmt "  at the current rate\n", left / 3600, (left % 3600) / 60;
    else               printf "remaining    budget reached (export/eval may still be running)\n";
  }'

  if [[ "$left" -gt 0 ]]; then
    local finish
    finish="$(date -d "+$left seconds" '+%a %H:%M' 2>/dev/null \
              || date -v"+${left}S" '+%a %H:%M' 2>/dev/null || true)"
    [[ -n "$finish" ]] && echo "finishes     ~$finish"
  fi
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

cmd_sync() {
  # Fast-forward this checkout to origin/main WITHOUT destroying artifacts.
  #
  # THE PROBLEM THIS EXISTS FOR. Training happens here, so a finished run leaves
  # moves/<name>.{npz,yaml} UNTRACKED in this working tree. Those files get
  # rsynced to the laptop, committed and pushed -- and then `git pull` here
  # refuses to move, because checking out the new commit "would overwrite
  # untracked working tree files". It is right to refuse in general; it is
  # pointless here, because the incoming blobs ARE these files. They came from
  # this machine in the first place.
  #
  # So: verify byte-identity against the incoming commit, delete only the ones
  # that match, and leave anything that differs alone -- a file with the same
  # name and different contents is a DIFFERENT training run and is exactly what
  # the refusal is protecting.
  git fetch origin -q || die "git fetch failed"
  local target="origin/main" removed=0 kept=0
  git rev-parse --verify -q "$target" >/dev/null || die "no $target"

  while IFS= read -r f; do
    [[ -n "$f" ]] || continue
    git cat-file -e "$target:$f" 2>/dev/null || continue   # not incoming; ignore
    if [[ "$(git hash-object "$f")" == "$(git rev-parse "$target:$f")" ]]; then
      rm -f "$f"; removed=$((removed + 1))
      echo "  identical to incoming, removed   $f"
    else
      kept=$((kept + 1))
      echo "  DIFFERS from incoming, KEPT      $f"
    fi
  done < <(git ls-files --others --exclude-standard)

  # Same argument for TRACKED files that are merely modified: if the working
  # copy already equals the incoming blob, the edit has arrived by another
  # route (an rsync of the same change) and discarding it loses nothing. This
  # is the case when the fix itself is what is being pulled -- this script.
  while IFS= read -r f; do
    [[ -n "$f" ]] || continue
    git cat-file -e "$target:$f" 2>/dev/null || continue
    if [[ "$(git hash-object "$f")" == "$(git rev-parse "$target:$f")" ]]; then
      git checkout -q -- "$f"; removed=$((removed + 1))
      echo "  modified but identical, reverted  $f"
    else
      kept=$((kept + 1))
      echo "  MODIFIED and differs, KEPT        $f"
    fi
  done < <(git diff --name-only)

  if (( kept > 0 )); then
    echo
    echo "$kept file(s) differ from what is being pulled -- a different run or"
    echo "a real local edit under the same name. Move them aside or commit them"
    echo "by hand, then re-run. Nothing was deleted."
    return 1
  fi
  echo "  reconciled $removed file(s); fast-forwarding"
  git merge --ff-only "$target" >/dev/null || die "not a fast-forward -- resolve by hand"
  echo "  now at $(git rev-parse --short HEAD)"
}

SUB="${1:-}"; shift || true
case "$SUB" in
  up)         check_move "${1:-}"; cmd_up "$@" ;;
  seeds)      check_move "${1:-}"; cmd_seeds "$@" ;;
  train)      check_move "${1:-}"; cmd_train "$@" ;;
  board)      check_move "${1:-}"; cmd_board "$1" ;;
  stop)       check_move "${1:-}"; stop "$1" train ;;
  stop-board) check_move "${1:-}"; stop "$1" board ;;
  eta)        check_move "${1:-}"; cmd_eta "$1" ;;
  logs)       check_move "${1:-}"; tail -f "$(run_dir "$1")/logs/train-latest.log" ;;
  status)     cmd_status ;;
  sync)       cmd_sync ;;
  *)          awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0" >&2; exit 1 ;;
esac
