#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root_dir/scripts/build_lock.sh"
source "$root_dir/scripts/lib/build_native_body.sh"
build_root=
install_prefix=
build_type=Release
run_tests=0
reuse_build_trees=0
jobs=${OPENARM_BUILD_JOBS-2}

usage() {
  cat <<'EOF'
Usage: scripts/build_native.sh --build-root PATH --install-prefix PATH [OPTIONS]

Build and install CAN, model, commission, transport, control, and runtime in dependency
order. Build and install paths must be supplied explicitly. Reusing a build
root with a different install prefix deterministically refreshes dependencies.

Options:
  --build-type TYPE  CMake build type (default: Release)
  --jobs JOBS        Maximum concurrent build jobs (default: OPENARM_BUILD_JOBS or 2)
  --reuse-build-trees  Reconfigure and reuse compatible component build trees
  --tests            Build and run every registered hardware-free native CTest
  -h, --help         Show this help
EOF
}

while (($#)); do
  case "$1" in
    --build-root)
      (($# >= 2)) || { printf '%s requires a path\n' "$1" >&2; exit 2; }
      build_root=$2
      shift 2
      ;;
    --install-prefix)
      (($# >= 2)) || { printf '%s requires a path\n' "$1" >&2; exit 2; }
      install_prefix=$2
      shift 2
      ;;
    --build-type)
      (($# >= 2)) || { printf '%s requires a value\n' "$1" >&2; exit 2; }
      build_type=$2
      shift 2
      ;;
    --tests)
      run_tests=1
      shift
      ;;
    --jobs)
      (($# >= 2)) && [[ -n "$2" ]] || {
        printf '%s requires a positive integer\n' "$1" >&2
        exit 2
      }
      jobs=$2
      shift 2
      ;;
    --reuse-build-trees)
      reuse_build_trees=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || {
  printf 'OPENARM_BUILD_JOBS/--jobs must be a positive integer: %s\n' "$jobs" >&2
  exit 2
}
export OPENARM_BUILD_JOBS="$jobs"
export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
export CTEST_PARALLEL_LEVEL="$jobs"

[[ -n "$build_root" ]] || { printf 'Missing --build-root\n' >&2; exit 2; }
[[ -n "$install_prefix" ]] || { printf 'Missing --install-prefix\n' >&2; exit 2; }
[[ "$build_root" == /* ]] || {
  printf '%s\n' '--build-root must be absolute' >&2
  exit 2
}
[[ "$install_prefix" == /* ]] || {
  printf '%s\n' '--install-prefix must be absolute' >&2
  exit 2
}
build_root=$(realpath -m -- "$build_root")
install_prefix=$(realpath -m -- "$install_prefix")
for path in "$build_root" "$install_prefix"; do
  if [[ "$path" == *:* || "$path" == *\;* ]]; then
    printf 'Build/install paths containing : or ; are unsupported: %s\n' "$path" >&2
    exit 2
  fi
done
root_real=$(realpath -e -- "$root_dir")
home_real=$(realpath -m -- "${HOME:-/nonexistent}")
for path in "$build_root" "$install_prefix"; do
  case "$path" in
    /|/etc|/home|/opt|/root|/tmp|/usr|/var|\
    "$root_real"|"$(dirname "$root_real")"|"$home_real")
      printf 'Refusing unsafe build/install path: %s\n' "$path" >&2
      exit 2
      ;;
  esac
done
[[ "$build_type" =~ ^[A-Za-z0-9_+-]+$ ]] || {
  printf 'Invalid --build-type: %s\n' "$build_type" >&2
  exit 2
}
[[ "$run_tests" == 0 || "$run_tests" == 1 ]] || {
  printf 'Invalid native test flag: %s\n' "$run_tests" >&2
  exit 2
}
[[ "$reuse_build_trees" == 0 || "$reuse_build_trees" == 1 ]] || {
  printf 'Invalid native reuse flag: %s\n' "$reuse_build_trees" >&2
  exit 2
}

description_dir="$root_dir/upstream/openarm_description"
if [[ ! -f "$description_dir/package.xml" ]]; then
  printf 'Missing pinned upstream. Run %s/scripts/fetch_upstreams.sh first.\n' "$root_dir" >&2
  exit 1
fi
[[ -x /opt/ros/lyrical/bin/xacro ]] || {
  printf 'Missing ROS xacro executable: %s\n' /opt/ros/lyrical/bin/xacro >&2
  exit 1
}
[[ -d /opt/ros/lyrical/lib/python3.14/site-packages ]] || {
  printf 'Missing ROS xacro Python package path: %s\n' \
    /opt/ros/lyrical/lib/python3.14/site-packages >&2
  exit 1
}

openarm_run_with_locks "$build_root" "$install_prefix" -- \
  openarm_build_native_body "$root_dir" "$build_root" "$install_prefix" \
  "$build_type" "$run_tests" "$reuse_build_trees" "$jobs"
