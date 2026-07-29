#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
output_root="$root_dir/ros2_ws"
build_type=Release
run_tests=0
clean=1

usage() {
  cat <<'EOF'
Usage: scripts/build.sh [OPTIONS]

Cleanly build and install every native component, then build the two ROS
packages. The default output root remains ros2_ws for launch compatibility.

Options:
  --tests             Run native CTests and verify all ROS tests are registered
  --incremental       Reuse existing build and install directories
  --output-root PATH  Put native, ROS, log, and install output under PATH
  --build-type TYPE   CMake build type (default: Release)
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
mkdir -p "$output_root"

native_args=(
  --build-root "$native_build"
  --install-prefix "$install_prefix"
  --build-type "$build_type"
)
if ((run_tests)); then
  native_args+=(--tests)
fi
"$root_dir/scripts/build_native.sh" "${native_args[@]}"

set +u
source /opt/ros/lyrical/setup.bash
set -u
export CMAKE_PREFIX_PATH="$install_prefix:/opt/ros/lyrical"
export AMENT_PREFIX_PATH=/opt/ros/lyrical
unset COLCON_PREFIX_PATH

coverage_args=(-DOPENARM_IK_ROS_COVERAGE=OFF)
if [[ "${OPENARM_IK_ROS_COVERAGE:-0}" == 1 ]]; then
  coverage_args=(-DOPENARM_IK_ROS_COVERAGE=ON)
fi
tests_flag=OFF
if ((run_tests)); then
  tests_flag=ON
fi

colcon --log-base "$ros_log" build \
  --base-paths "$description_dir" "$root_dir/ros2_ws/src" \
  --packages-select openarm_description openarm_ik_ros \
  --build-base "$ros_build" \
  --install-base "$install_prefix" \
  --event-handlers console_direct+ \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE="$build_type" \
    -DBUILD_TESTING="$tests_flag" \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    "${coverage_args[@]}"

if ((run_tests)); then
  ros_test_listing=$(ctest --test-dir "$ros_build/openarm_ik_ros" -N)
  printf '%s\n' "$ros_test_listing"
  registered_ros_tests=$(awk '/Total Tests:/ {print $3}' <<<"$ros_test_listing")
  if [[ "$registered_ros_tests" != 8 ]]; then
    printf 'Expected 8 openarm_ik_ros tests, found %s\n' \
      "${registered_ros_tests:-none}" >&2
    exit 1
  fi
fi

printf 'OpenArm build complete. Source %s/setup.bash\n' "$install_prefix"
