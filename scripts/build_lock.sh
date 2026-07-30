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

openarm_run_with_locks() {
  local separator=0 argument resource lock_dir lock_file lock_fd records=
  local -a resources=() sorted=()
  for argument in "$@"; do
    if ((separator)); then
      sorted+=("$argument")
    elif [[ "$argument" == -- ]]; then
      separator=1
    else
      resources+=("$argument")
    fi
  done
  ((separator)) && ((${#sorted[@]})) || {
    printf '%s\n' 'Internal build-lock invocation is malformed.' >&2
    return 2
  }
  lock_dir=$(openarm_lock_dir) || return $?
  mapfile -t resources < <(printf '%s\n' "${resources[@]}" | sort -u)
  for resource in "${resources[@]}"; do
    lock_file=$(openarm_lock_file "$lock_dir" "$resource")
    exec {lock_fd}>"$lock_file"
    if ! flock -n -E 75 "$lock_fd"; then
      printf 'Build resource is already being built: %s\n' "$resource" >&2
      return 75
    fi
    records+="${records:+,}${lock_fd}:${lock_file}"
  done
  OPENARM_BUILD_LOCK_FDS=$records exec "${sorted[@]}"
}

openarm_validate_locks() {
  local lock_dir records=$OPENARM_BUILD_LOCK_FDS record fd lock_file expected actual
  local -a expected_files=()
  lock_dir=$(openarm_lock_dir) || return $?
  [[ -n "$records" ]] || {
    printf '%s\n' 'Build body was not started by the locked runner.' >&2
    return 2
  }
  for expected in "$@"; do
    expected_files+=("$(openarm_lock_file "$lock_dir" "$expected")")
  done
  for expected in "${expected_files[@]}"; do
    record=
    IFS=',' read -r -a lock_records <<<"$records"
    for record in "${lock_records[@]}"; do
      fd=${record%%:*}
      lock_file=${record#*:}
      [[ "$fd" =~ ^[0-9]+$ && "$lock_file" == "$expected" ]] || continue
      actual=$(readlink -f "/proc/self/fd/$fd" 2>/dev/null || true)
      [[ "$actual" == "$(realpath -m -- "$expected")" ]] || continue
      flock -n -E 75 "$fd" || {
        printf 'Build resource is already being built: %s\n' "$expected" >&2
        return 75
      }
      record=validated
      break
    done
    [[ "$record" == validated ]] || {
      printf 'Build body is missing a validated lock for: %s\n' "$expected" >&2
      return 2
    }
  done
}
