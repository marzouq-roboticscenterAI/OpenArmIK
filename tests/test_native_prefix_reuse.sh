#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)); then
  printf 'Usage: %s EMPTY_WORK_ROOT\n' "$0" >&2
  exit 2
fi

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
work_root=$1
if [[ "$work_root" != /* ]]; then
  work_root="$PWD/$work_root"
fi
work_root=$(realpath -m -- "$work_root")
if [[ -e "$work_root" ]]; then
  [[ -d "$work_root" && -z "$(find "$work_root" -mindepth 1 -print -quit)" ]] || {
    printf 'Work root must not exist or must be empty: %s\n' "$work_root" >&2
    exit 2
  }
fi

build_root="$work_root/native-build"
prefix_a="$work_root/prefix-a"
prefix_b="$work_root/prefix-b"
mkdir -p "$work_root"

"$root_dir/scripts/build_native.sh" \
  --build-root "$build_root" \
  --install-prefix "$prefix_a" \
  --build-type Release \
  --tests

"$root_dir/scripts/build_native.sh" \
  --build-root "$build_root" \
  --install-prefix "$prefix_b" \
  --build-type Release \
  --tests

assert_cache_value() {
  local cache_file=$1 key=$2 expected=$3 line value=
  while IFS= read -r line; do
    case "$line" in
      "$key":*=*)
        value=${line#*=}
        break
        ;;
    esac
  done < "$cache_file"
  [[ "$value" == "$expected" ]] || {
    printf 'Stale %s in %s: %s\n' "$key" "$cache_file" "${value:-unset}" >&2
    exit 1
  }
}

assert_cache_value "$build_root/transport/CMakeCache.txt" \
  CMAKE_PREFIX_PATH "$prefix_b"
assert_cache_value "$build_root/transport/CMakeCache.txt" \
  OpenArmCan_DIR "$prefix_b/lib/cmake/OpenArmCan"
assert_cache_value "$build_root/control/CMakeCache.txt" \
  CMAKE_PREFIX_PATH "$prefix_b"
assert_cache_value "$build_root/control/CMakeCache.txt" \
  openarm_model_DIR "$prefix_b/lib/cmake/openarm_model"

transport_verbose=$(cmake --build "$build_root/transport" \
  --target openarm_transport_tests --parallel --clean-first --verbose 2>&1)
control_verbose=$(cmake --build "$build_root/control" \
  --target openarm_control_tests --parallel --clean-first --verbose 2>&1)
linked_output="$transport_verbose"$'\n'"$control_verbose"
if [[ "$linked_output" == *"$prefix_a"* ||
      "$linked_output" != *"$prefix_b/include"* ||
      "$linked_output" != *"$prefix_b/lib/libopenarm_can.a"* ||
      "$linked_output" != *"$prefix_b/lib/libopenarm_model.a"* ]]; then
  printf 'Verbose rebuild did not link exclusively against prefix B\n' >&2
  exit 1
fi

[[ -f "$prefix_b/lib/cmake/OpenArmCan/OpenArmCanTargets.cmake" ]]
[[ -f "$prefix_b/lib/cmake/openarm_model/openarm_modelTargets.cmake" ]]
printf 'Native prefix-reuse regression passed: %s -> %s\n' \
  "$prefix_a" "$prefix_b"
