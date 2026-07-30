#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)); then
  printf 'Usage: %s EMPTY_WORK_ROOT\n' "$0" >&2
  exit 2
fi

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
jobs=${OPENARM_BUILD_JOBS-2}
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || {
  printf 'OPENARM_BUILD_JOBS must be a positive integer: %s\n' "$jobs" >&2
  exit 2
}
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
  --jobs "$jobs" \
  --tests

"$root_dir/scripts/build_native.sh" \
  --build-root "$build_root" \
  --install-prefix "$prefix_b" \
  --build-type Release \
  --jobs "$jobs" \
  --reuse-build-trees \
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
consumer_cache="$build_root/installed_native_consumer/CMakeCache.txt"
assert_cache_value "$consumer_cache" CMAKE_PREFIX_PATH "$prefix_b"
assert_cache_value "$consumer_cache" \
  OpenArmCan_DIR "$prefix_b/lib/cmake/OpenArmCan"
assert_cache_value "$consumer_cache" \
  openarm_model_DIR "$prefix_b/lib/cmake/openarm_model"
assert_cache_value "$consumer_cache" \
  openarm_commission_DIR "$prefix_b/lib/cmake/openarm_commission"
assert_cache_value "$consumer_cache" \
  OpenArmTransport_DIR "$prefix_b/lib/cmake/OpenArmTransport"
assert_cache_value "$consumer_cache" \
  openarm_control_DIR "$prefix_b/lib/cmake/openarm_control"

control_consumer="$build_root/control/public-header-install-consumer"
assert_cache_value "$control_consumer/CMakeCache.txt" \
  CMAKE_PREFIX_PATH \
  "$build_root/control/public-header-install;$prefix_b"
assert_cache_value "$control_consumer/CMakeCache.txt" \
  openarm_control_DIR \
  "$build_root/control/public-header-install/lib/cmake/openarm_control"
assert_cache_value "$control_consumer/CMakeCache.txt" \
  openarm_model_DIR "$prefix_b/lib/cmake/openarm_model"

transport_consumer="$build_root/transport/install-consumer-build"
assert_cache_value "$transport_consumer/CMakeCache.txt" \
  CMAKE_PREFIX_PATH \
  "$build_root/transport/install-consumer-prefix;$prefix_b"
assert_cache_value "$transport_consumer/CMakeCache.txt" \
  OpenArmTransport_DIR \
  "$build_root/transport/install-consumer-prefix/lib/cmake/OpenArmTransport"
assert_cache_value "$transport_consumer/CMakeCache.txt" \
  OpenArmCan_DIR "$prefix_b/lib/cmake/OpenArmCan"

while IFS= read -r cache_file; do
  if grep -Fq "$prefix_a" "$cache_file"; then
    printf 'Stale prefix A path in subordinate cache: %s\n' "$cache_file" >&2
    exit 1
  fi
done < <(find "$build_root" -name CMakeCache.txt -type f -print)

transport_verbose=$(cmake --build "$build_root/transport" \
  --target openarm_transport_tests --parallel "$jobs" --clean-first --verbose 2>&1)
control_verbose=$(cmake --build "$build_root/control" \
  --target openarm_control_tests --parallel "$jobs" --clean-first --verbose 2>&1)
linked_output="$transport_verbose"$'\n'"$control_verbose"
if [[ "$linked_output" == *"$prefix_a"* ||
      "$linked_output" != *"$prefix_b/include"* ||
      "$linked_output" != *"$prefix_b/lib/libopenarm_can.a"* ||
      "$linked_output" != *"$prefix_b/lib/libopenarm_model.a"* ]]; then
  printf 'Verbose rebuild did not link exclusively against prefix B\n' >&2
  exit 1
fi

cmake --build "$build_root/installed_native_consumer" \
  --target openarm_installed_c11 openarm_installed_cxx17 \
  --parallel "$jobs" --clean-first --verbose >/dev/null
for consumer in openarm_installed_c11 openarm_installed_cxx17; do
  consumer_dir="$build_root/installed_native_consumer/CMakeFiles/$consumer.dir"
  flags_file="$consumer_dir/flags.make"
  link_file="$consumer_dir/link.txt"
  if grep -Fq "$prefix_a" "$flags_file" ||
     grep -Fq "$prefix_a" "$link_file" ||
     ! grep -Fq "$prefix_b/include" "$flags_file"; then
    printf '%s compile/link commands do not exclusively use prefix B\n' \
      "$consumer" >&2
    exit 1
  fi
  for archive in \
    libopenarm_can.a \
    libopenarm_model.a \
    libopenarm_commission.a \
    libopenarm_transport.a \
    libopenarm_control.a; do
    if ! grep -Fq "$prefix_b/lib/$archive" "$link_file"; then
      printf '%s does not link prefix B %s\n' "$consumer" "$archive" >&2
      exit 1
    fi
  done
  "$build_root/installed_native_consumer/$consumer"
done

[[ -f "$prefix_b/lib/cmake/OpenArmCan/OpenArmCanTargets.cmake" ]]
[[ -f "$prefix_b/lib/cmake/openarm_model/openarm_modelTargets.cmake" ]]
printf 'Native prefix-reuse regression passed: %s -> %s\n' \
  "$prefix_a" "$prefix_b"
