#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
for argument in "$@"; do
  case "$argument" in
    -h|--help)
      exec "$root_dir/scripts/launch_web_portal.sh" --help
      ;;
  esac
done
output_root="$root_dir/ros2_ws"
jobs=
build_mode=auto
arguments=("$@")
for ((index = 0; index < ${#arguments[@]}; index++)); do
  case "${arguments[index]}" in
    --output-root)
      ((index + 1 < ${#arguments[@]})) || {
        printf '%s requires a path\n' --output-root >&2
        exit 2
      }
      output_root=${arguments[index + 1]}
      ((index += 1))
      ;;
    --jobs)
      ((index + 1 < ${#arguments[@]})) || {
        printf '%s requires a value\n' --jobs >&2
        exit 2
      }
      jobs=${arguments[index + 1]}
      ((index += 1))
      ;;
    --build)
      build_mode=always
      ;;
    --no-build)
      build_mode=never
      ;;
  esac
done

"$root_dir/scripts/install_all_dependencies.sh" --verify >/dev/null
command -v firefox >/dev/null 2>&1 || {
  printf '%s\n' 'Firefox is not installed or is not on PATH.' >&2
  exit 1
}

build_arguments=(--incremental --output-root "$output_root")
[[ -z "$jobs" ]] || build_arguments+=(--jobs "$jobs")
if [[ "$build_mode" != never ]]; then
  "$root_dir/scripts/build.sh" "${build_arguments[@]}"
fi
exec "$root_dir/scripts/launch_web_portal.sh" --firefox "$@" --no-build
