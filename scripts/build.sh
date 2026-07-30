#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
output_root="$root_dir/ros2_ws"
build_type=Release
run_tests=0
clean=1
jobs=${OPENARM_BUILD_JOBS-2}
original_args=("$@")

usage() {
  cat <<'EOF'
Usage: scripts/build.sh [OPTIONS]

Cleanly build and install every native component, then build the three ROS
packages. The default output root remains ros2_ws for launch compatibility.

Options:
  --tests             Run native CTests and verify all ROS tests are registered
  --incremental       Reuse existing build and install directories
  --output-root PATH  Put native, ROS, log, and install output under PATH
  --build-type TYPE   CMake build type (default: Release)
  --jobs JOBS         Maximum concurrent build jobs (default: OPENARM_BUILD_JOBS or 2)
  -h, --help          Show this help
EOF
}

while (($#)); do
  case "$1" in
    --tests)
      run_tests=1
      shift
      ;;
    --incremental)
      clean=0
      shift
      ;;
    --output-root)
      (($# >= 2)) || { printf '%s requires a path\n' "$1" >&2; exit 2; }
      output_root=$2
      shift 2
      ;;
    --build-type)
      (($# >= 2)) || { printf '%s requires a value\n' "$1" >&2; exit 2; }
      build_type=$2
      shift 2
      ;;
    --jobs)
      (($# >= 2)) && [[ -n "$2" ]] || {
        printf '%s requires a positive integer\n' "$1" >&2
        exit 2
      }
      jobs=$2
      shift 2
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

if [[ "$output_root" != /* ]]; then
  output_root="$PWD/$output_root"
fi
output_root=$(realpath -m -- "$output_root")
if [[ "$output_root" == *:* || "$output_root" == *\;* ]]; then
  printf 'Output roots containing : or ; are unsupported: %s\n' "$output_root" >&2
  exit 2
fi
root_real=$(realpath -e -- "$root_dir")
home_real=$(realpath -m -- "${HOME:-/nonexistent}")
case "$output_root" in
  /|/etc|/home|/opt|/root|/tmp|/usr|/var|\
  "$root_real"|"$(dirname "$root_real")"|"$home_real")
    printf 'Refusing unsafe output root: %s\n' "$output_root" >&2
    exit 2
    ;;
esac
[[ "$build_type" =~ ^[A-Za-z0-9_+-]+$ ]] || {
  printf 'Invalid --build-type: %s\n' "$build_type" >&2
  exit 2
}

description_dir="$root_dir/upstream/openarm_description"
if [[ ! -f "$description_dir/package.xml" ]]; then
  printf 'Missing pinned upstream. Run %s/scripts/fetch_upstreams.sh first.\n' "$root_dir" >&2
  exit 1
fi
[[ -f /opt/ros/lyrical/setup.bash ]] || {
  printf 'Missing ROS setup: /opt/ros/lyrical/setup.bash\n' >&2
  exit 1
}

native_build="$output_root/native_build"
ros_build="$output_root/build"
install_prefix="$output_root/install"
ros_log="$output_root/log"

mkdir -p "$output_root"
lock_file="$output_root/.openarmik-build.lock"
if [[ ${OPENARM_BUILD_LOCK_HELD:-0} != 1 ]]; then
  set +e
  flock -n -E 75 --close "$lock_file" \
    env OPENARM_BUILD_LOCK_HELD=1 "$0" "${original_args[@]}"
  lock_status=$?
  set -e
  if ((lock_status == 75)); then
    printf 'Build output root is already being built: %s\n' "$output_root" >&2
    exit 3
  fi
  exit "$lock_status"
fi

clean_child() {
  local requested=$1
  local resolved
  resolved=$(realpath -m -- "$requested")
  case "$resolved" in
    "$output_root"/*) ;;
    *)
      printf 'Refusing cleanup outside output root: %s\n' "$resolved" >&2
      exit 2
      ;;
  esac
  [[ "$resolved" != "$output_root" ]] || {
    printf 'Refusing to clean the output root itself: %s\n' "$resolved" >&2
    exit 2
  }
  rm -rf --one-file-system -- "$resolved"
}

if ((clean)); then
  clean_child "$native_build"
  clean_child "$ros_build"
  clean_child "$install_prefix"
  clean_child "$ros_log"
fi

native_args=(
  --build-root "$native_build"
  --install-prefix "$install_prefix"
  --build-type "$build_type"
  --jobs "$jobs"
)
if ((!clean)); then
  native_args+=(--reuse-build-trees)
fi
if ((run_tests)); then
  native_args+=(--tests)
fi
OPENARM_NATIVE_BUILD_LOCK_HELD=1 \
  "$root_dir/scripts/build_native.sh" "${native_args[@]}"

set +u
source /opt/ros/lyrical/setup.bash
set -u
export CMAKE_PREFIX_PATH="$install_prefix:/opt/ros/lyrical"
export AMENT_PREFIX_PATH=/opt/ros/lyrical
unset COLCON_PREFIX_PATH

coverage_args=()
if [[ "${OPENARM_IK_ROS_COVERAGE:-0}" == 1 ]]; then
  coverage_args=(-DOPENARM_IK_ROS_COVERAGE=ON)
fi
tests_flag=OFF
if ((run_tests)); then
  tests_flag=ON
fi

colcon_args=(
  --log-base "$ros_log" build
  --executor sequential
  --base-paths "$description_dir" "$root_dir/ros2_ws/src"
  --packages-select openarm_description openarm_control_msgs openarm_ik_ros
  --build-base "$ros_build"
  --install-base "$install_prefix"
  --event-handlers console_direct+
)
if ((clean)); then
  colcon_args+=(--cmake-clean-cache)
fi
colcon_args+=(--cmake-args
  -DCMAKE_BUILD_TYPE="$build_type"
  -DBUILD_TESTING="$tests_flag"
  -DCMAKE_WARN_DEPRECATED=OFF
  -DPython3_EXECUTABLE=/usr/bin/python3
  "${coverage_args[@]}")

MAKEFLAGS="-j$jobs" CMAKE_BUILD_PARALLEL_LEVEL="$jobs" \
  colcon "${colcon_args[@]}"

session_archive="$ros_build/openarm_ik_ros/libopenarm_virtual_control_session.a"
[[ -f "$session_archive" ]] || {
  printf 'Missing production ROS session archive: %s\n' "$session_archive" >&2
  exit 1
}
session_undefined=$(nm -u "$session_archive")
if grep -Eq ' U oa_(controller_|motion_plan_|manifest_)' <<<"$session_undefined"; then
  printf '%s\n' 'Production ROS session bypasses OpenArm::Runtime' >&2
  exit 1
fi
if ! grep -q ' U oa_runtime_create' <<<"$session_undefined"; then
  printf '%s\n' 'Production ROS session does not consume OpenArm::Runtime' >&2
  exit 1
fi

if ((run_tests)); then
  ros_test_listing=$(ctest --test-dir "$ros_build/openarm_ik_ros" -N)
  printf '%s\n' "$ros_test_listing"
  registered_ros_tests=$(awk '/Total Tests:/ {print $3}' <<<"$ros_test_listing")
  if [[ "$registered_ros_tests" != 13 ]]; then
    printf 'Expected 13 openarm_ik_ros tests, found %s\n' \
      "${registered_ros_tests:-none}" >&2
    exit 1
  fi
fi

printf 'OpenArm build complete. Source %s/setup.bash\n' "$install_prefix"
