#!/usr/bin/env bash
# Shared helpers for build scripts. This file is sourced; it does not run a body.

openarm_lock_dir() {
  local candidate=${XDG_RUNTIME_DIR:-}
  if [[ -z "$candidate" || ! -d "$candidate" || ! -w "$candidate" ]]; then
    candidate="/tmp/openarmik-build-locks-$UID"
    if [[ -L "$candidate" || ( -e "$candidate" && ! -O "$candidate" ) ]]; then
      printf 'Refusing unsafe build lock directory: %s\n' "$candidate" >&2
      return 2
    fi
    mkdir -m 700 -p -- "$candidate"
    chmod 700 -- "$candidate"
  else
    if [[ -L "$candidate" || ! -O "$candidate" ]]; then
      printf 'Refusing unsafe XDG runtime directory for build locks: %s\n' "$candidate" >&2
      return 2
    fi
    candidate="$candidate/openarmik-build-locks-$UID"
    if [[ -L "$candidate" || ( -e "$candidate" && ! -O "$candidate" ) ]]; then
      printf 'Refusing unsafe build lock directory: %s\n' "$candidate" >&2
      return 2
    fi
    mkdir -m 700 -p -- "$candidate"
    chmod 700 -- "$candidate"
  fi
  printf '%s\n' "$candidate"
}

openarm_lock_file() {
  local lock_dir=$1 resource=$2 digest
  digest=$(printf '%s\0' "$resource" | sha256sum | awk '{print $1}')
  printf '%s/%s.lock\n' "$lock_dir" "$digest"
}

openarm_lock_forward_signal() {
  local signal=$1 status=$2
  if ((openarm_lock_signal_status == 0)); then
    openarm_lock_signal_status=$status
  fi
  if [[ -n "$openarm_lock_callback_pgid" ]]; then
    kill -s "$signal" -- "-$openarm_lock_callback_pgid" 2>/dev/null || true
  else
    openarm_lock_pending_signals+=("$signal")
  fi
}

openarm_lock_restore_trap() {
  local signal=$1 saved=$2
  if [[ -n "$saved" ]]; then
    # `trap -p` emits shell-quoted input specifically suitable for restoration.
    source /dev/stdin <<<"$saved"
  else
    trap - "$signal"
  fi
}

openarm_run_with_locks() {
  local argument callback lock_dir lock_file lock_fd resource signal
  local callback_status=0 wait_status=0 leader_reaped=0 killed_group=0
  local had_errexit=0 had_monitor=0 escalation_deadline=0
  local saved_hup saved_int saved_term
  local -a resources=() callback_args=() lock_fds=()
  local -a openarm_lock_pending_signals=()
  local -A seen_resources=()
  local openarm_lock_callback_pgid=
  local openarm_lock_signal_status=0

  while (($#)) && [[ "$1" != -- ]]; do
    [[ -n "$1" ]] || {
      printf '%s\n' 'Internal build-lock invocation is malformed.' >&2
      return 2
    }
    if [[ -z "${seen_resources[$1]+present}" ]]; then
      resources+=("$1")
      seen_resources[$1]=1
    fi
    shift
  done
  (($#)) || {
    printf '%s\n' 'Internal build-lock invocation is malformed.' >&2
    return 2
  }
  shift
  ((${#resources[@]} && $#)) || {
    printf '%s\n' 'Internal build-lock invocation is malformed.' >&2
    return 2
  }
  callback=$1
  shift
  declare -F "$callback" >/dev/null || {
    printf 'Internal build-lock callback is not a function: %s\n' "$callback" >&2
    return 2
  }
  callback_args=("$callback" "$@")

  lock_dir=$(openarm_lock_dir) || return $?
  for resource in "${resources[@]}"; do
    lock_file=$(openarm_lock_file "$lock_dir" "$resource")
    if ! exec {lock_fd}>"$lock_file"; then
      for lock_fd in "${lock_fds[@]}"; do
        exec {lock_fd}>&-
      done
      return 1
    fi
    lock_fds+=("$lock_fd")
    if ! flock -n -E 75 "$lock_fd"; then
      printf 'Build resource is already being built: %s\n' "$resource" >&2
      for lock_fd in "${lock_fds[@]}"; do
        exec {lock_fd}>&-
      done
      return 3
    fi
  done

  [[ $- == *e* ]] && had_errexit=1
  [[ $- == *m* ]] && had_monitor=1
  saved_hup=$(trap -p HUP)
  saved_int=$(trap -p INT)
  saved_term=$(trap -p TERM)
  trap 'openarm_lock_forward_signal HUP 129' HUP
  trap 'openarm_lock_forward_signal INT 130' INT
  trap 'openarm_lock_forward_signal TERM 143' TERM

  set -m
  (
    trap - HUP INT TERM
    set +m
    for lock_fd in "${lock_fds[@]}"; do
      exec {lock_fd}>&-
    done
    unset lock_fd lock_fds
    set -euo pipefail
    "${callback_args[@]}"
  ) &
  openarm_lock_callback_pgid=$!
  if ((!had_monitor)); then
    set +m
  fi
  for signal in "${openarm_lock_pending_signals[@]}"; do
    kill -s "$signal" -- "-$openarm_lock_callback_pgid" 2>/dev/null || true
  done
  openarm_lock_pending_signals=()

  while ((!leader_reaped)); do
    if ((openarm_lock_signal_status == 0)); then
      set +e
      wait "$openarm_lock_callback_pgid" 2>/dev/null
      wait_status=$?
      ((had_errexit)) && set -e
      if ! kill -0 "$openarm_lock_callback_pgid" 2>/dev/null; then
        callback_status=$wait_status
        leader_reaped=1
      fi
      continue
    fi

    ((escalation_deadline)) || escalation_deadline=$((SECONDS + 5))
    if [[ ! -r "/proc/$openarm_lock_callback_pgid/stat" ]] ||
       [[ $(awk '{print $3}' "/proc/$openarm_lock_callback_pgid/stat" 2>/dev/null) == Z ]]; then
      set +e
      wait "$openarm_lock_callback_pgid" 2>/dev/null
      callback_status=$?
      ((had_errexit)) && set -e
      leader_reaped=1
      continue
    fi
    if ((SECONDS >= escalation_deadline && !killed_group)); then
      kill -KILL -- "-$openarm_lock_callback_pgid" 2>/dev/null || true
      killed_group=1
    fi
    sleep 0.05
  done

  while kill -0 -- "-$openarm_lock_callback_pgid" 2>/dev/null; do
    if ((openarm_lock_signal_status)); then
      ((escalation_deadline)) || escalation_deadline=$((SECONDS + 5))
      if ((SECONDS >= escalation_deadline && !killed_group)); then
        kill -KILL -- "-$openarm_lock_callback_pgid" 2>/dev/null || true
        killed_group=1
      fi
    fi
    sleep 0.05
  done

  for lock_fd in "${lock_fds[@]}"; do
    exec {lock_fd}>&-
  done
  openarm_lock_restore_trap HUP "$saved_hup"
  openarm_lock_restore_trap INT "$saved_int"
  openarm_lock_restore_trap TERM "$saved_term"
  if ((had_monitor)); then
    set -m
  else
    set +m
  fi

  if ((openarm_lock_signal_status)); then
    return "$openarm_lock_signal_status"
  fi
  return "$callback_status"
}
