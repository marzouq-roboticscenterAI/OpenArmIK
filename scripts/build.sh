#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root_dir/scripts/build_lock.sh"
source "$root_dir/scripts/lib/build_native_body.sh"
source "$root_dir/scripts/lib/description_pin.sh"
source "$root_dir/scripts/lib/launch_integrity.sh"
output_root="$root_dir/ros2_ws"
build_type=Release
run_tests=0
clean=1
jobs=${OPENARM_BUILD_JOBS-2}

usage() {
  cat <<'EOF'
Usage: scripts/build.sh [OPTIONS]

Cleanly build and install every native component, then build the three ROS
packages. The default output root remains ros2_ws for launch compatibility.

Options:
  --tests             Run native CTests and verify all ROS tests are registered
  --incremental       Reuse build caches; recreate the complete install tree
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
[[ "$run_tests" == 0 || "$run_tests" == 1 ]] || {
  printf 'Invalid test flag: %s\n' "$run_tests" >&2
  exit 2
}
[[ "$clean" == 0 || "$clean" == 1 ]] || {
  printf 'Invalid clean flag: %s\n' "$clean" >&2
  exit 2
}

if [[ "$output_root" != /* ]]; then
  output_root="$PWD/$output_root"
fi
output_root=$(realpath -ms -- "$output_root")
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
openarm_validate_description_pin "$description_dir" || {
  printf 'Pinned description validation failed. Run %s/scripts/fetch_upstreams.sh when online.\n' \
    "$root_dir" >&2
  exit 1
}
[[ -f /opt/ros/lyrical/setup.bash ]] || {
  printf 'Missing ROS setup: /opt/ros/lyrical/setup.bash\n' >&2
  exit 1
}
[[ -x /opt/ros/lyrical/bin/xacro ]] || {
  printf 'Missing ROS xacro executable: %s\n' /opt/ros/lyrical/bin/xacro >&2
  exit 1
}
[[ -d /opt/ros/lyrical/lib/python3.14/site-packages ]] || {
  printf 'Missing ROS xacro Python package path: %s\n' \
    /opt/ros/lyrical/lib/python3.14/site-packages >&2
  exit 1
}
input_fingerprint=$(openarm_compute_launch_source_fingerprint \
  "$root_dir" "$description_dir" "$build_type" "$run_tests")

native_build="$output_root/native_build"
install_prefix="$output_root/install"
export OPENARM_BUILD_JOBS="$jobs"
export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
export CTEST_PARALLEL_LEVEL="$jobs"

openarm_build_all_body() {
  local root_dir=$1 output_root=$2 build_type=$3 run_tests=$4 clean=$5 jobs=$6
  local input_fingerprint=$7
  local native_build="$output_root/native_build"
  local ros_build="$output_root/build"
  local install_prefix="$output_root/install"
  local ros_log="$output_root/log"
  local description_dir="$root_dir/upstream/openarm_description"
  local reuse_build_trees=0 tests_flag=OFF session_archive session_undefined
  local ros_test_listing registered_ros_tests native_request native_request_after
  local ros_request ros_request_after coverage_mode=${OPENARM_IK_ROS_COVERAGE:-0}
  local -a coverage_args=() colcon_args=()
  local -a native_components=(can model commission transport control runtime)
  ((run_tests == 0)) || native_components+=(installed_native_consumer)

  openarm_build_state_validate_output_root "$root_dir" "$output_root" || {
    printf 'Refusing unsafe output root: %s\n' "$output_root" >&2
    return 2
  }
  mkdir -p -- "$output_root"
  rm -f -- "$output_root/$OPENARM_LAUNCH_STAMP_NAME" \
    "$output_root/$OPENARM_LEGACY_LAUNCH_STAMP_NAME"

  native_request=$(openarm_build_state_requested_digest native "$build_type" \
    "$run_tests" 0 "$install_prefix") || return 1
  ros_request=$(openarm_build_state_requested_digest ros "$build_type" \
    "$run_tests" "$coverage_mode" "$install_prefix") || return 1
  if ((!clean)) && openarm_build_state_read_completed "$native_build" \
      "$native_request" "${native_components[@]}" \
      >/dev/null 2>&1 && openarm_build_state_read_completed "$ros_build" \
      "$ros_request" openarm_control_msgs openarm_description openarm_ik_ros \
      >/dev/null 2>&1; then
    reuse_build_trees=1
  else
    openarm_build_state_remove_output_child "$root_dir" "$output_root" \
      native_build || return
    openarm_build_state_remove_output_child "$root_dir" "$output_root" \
      build || return
    openarm_build_state_remove_output_child "$root_dir" "$output_root" \
      log || return
  fi
  # Install trees are launch authority, not caches. Recreate the unified native
  # and ROS install prefix under the exclusive output/native/install leases so
  # removed install rules cannot survive an otherwise incremental rebuild.
  openarm_build_state_remove_output_child "$root_dir" "$output_root" \
    install || return
  mkdir -p -- "$install_prefix" || return
  printf '%s\n' OPENARM_INSTALL_ROOT_V1 > \
    "$install_prefix/$OPENARM_INSTALL_STATE_FILE" || return
  openarm_build_state_write "$native_build" pending "$native_request" || return 1
  openarm_build_state_write "$ros_build" pending "$ros_request" || return 1

  openarm_build_native_body "$root_dir" "$native_build" "$install_prefix" \
    "$build_type" "$run_tests" "$reuse_build_trees" "$jobs"
  native_request_after=$(openarm_build_state_requested_digest native "$build_type" \
    "$run_tests" 0 "$install_prefix") || return 1
  [[ "$native_request_after" == "$native_request" ]] || {
    printf 'Requested native toolchain changed during the build\n' >&2
    return 1
  }
  openarm_build_state_publish_completed "$native_build" "$native_request" \
    "${native_components[@]}" || return 1

  set +u
  source /opt/ros/lyrical/setup.bash
  set -u
  export CMAKE_PREFIX_PATH="$install_prefix:/opt/ros/lyrical"
  export AMENT_PREFIX_PATH=/opt/ros/lyrical
  unset COLCON_PREFIX_PATH

  if [[ "${OPENARM_IK_ROS_COVERAGE:-0}" == 1 ]]; then
    coverage_args=(-DOPENARM_IK_ROS_COVERAGE=ON)
  fi
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
    return 1
  }
  session_undefined=$(nm -u "$session_archive")
  if grep -Eq ' U oa_(controller_|motion_plan_|manifest_)' <<<"$session_undefined"; then
    printf '%s\n' 'Production ROS session bypasses OpenArm::Runtime' >&2
    return 1
  fi
  if ! grep -q ' U oa_runtime_create' <<<"$session_undefined"; then
    printf '%s\n' 'Production ROS session does not consume OpenArm::Runtime' >&2
    return 1
  fi

  if ((run_tests)); then
    ros_test_listing=$(ctest --test-dir "$ros_build/openarm_ik_ros" -N)
    printf '%s\n' "$ros_test_listing"
    registered_ros_tests=$(awk '/Total Tests:/ {print $3}' <<<"$ros_test_listing")
    if [[ "$registered_ros_tests" != 15 ]]; then
      printf 'Expected 15 openarm_ik_ros tests, found %s\n' \
        "${registered_ros_tests:-none}" >&2
      return 1
    fi
  fi

  ros_request_after=$(openarm_build_state_requested_digest ros "$build_type" \
    "$run_tests" "$coverage_mode" "$install_prefix") || return 1
  [[ "$ros_request_after" == "$ros_request" ]] || {
    printf 'Requested ROS toolchain changed during the build\n' >&2
    return 1
  }
  openarm_build_state_publish_completed "$ros_build" "$ros_request" \
    openarm_control_msgs openarm_description openarm_ik_ros || return 1

  openarm_write_launch_stamp "$root_dir" "$output_root" "$description_dir" \
    "$input_fingerprint" "$build_type" "$run_tests"

  printf 'OpenArm build complete. Source %s/setup.bash\n' "$install_prefix"
}

openarm_run_with_locks \
  "$output_root" "$native_build" "$install_prefix" -- \
  openarm_build_all_body "$root_dir" "$output_root" "$build_type" \
  "$run_tests" "$clean" "$jobs" "$input_fingerprint"
