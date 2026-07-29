#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_root=
install_prefix=
build_type=Release
run_tests=0

usage() {
  cat <<'EOF'
Usage: scripts/build_native.sh --build-root PATH --install-prefix PATH [OPTIONS]

Build and install CAN, model, commission, transport, and control in dependency
order. Build and install paths must be supplied explicitly.

Options:
  --build-type TYPE  CMake build type (default: Release)
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

description_dir="$root_dir/upstream/openarm_description"
if [[ ! -f "$description_dir/package.xml" ]]; then
  printf 'Missing pinned upstream. Run %s/scripts/fetch_upstreams.sh first.\n' "$root_dir" >&2
  exit 1
fi

xacro_executable=/opt/ros/lyrical/bin/xacro
xacro_pythonpath=/opt/ros/lyrical/lib/python3.14/site-packages
[[ -x "$xacro_executable" ]] || {
  printf 'Missing ROS xacro executable: %s\n' "$xacro_executable" >&2
  exit 1
}
[[ -d "$xacro_pythonpath" ]] || {
  printf 'Missing ROS xacro Python package path: %s\n' "$xacro_pythonpath" >&2
  exit 1
}

mkdir -p "$build_root" "$install_prefix"
tests_flag=OFF
if ((run_tests)); then
  tests_flag=ON
fi

configure_build_test_install() {
  local component=$1
  shift
  local component_build="$build_root/$component"
  cmake -S "$root_dir/$component" -B "$component_build" \
    -DCMAKE_BUILD_TYPE="$build_type" \
    -DCMAKE_INSTALL_PREFIX="$install_prefix" \
    -DCMAKE_PREFIX_PATH="$install_prefix" \
    "$@"
  cmake --build "$component_build" --parallel
  if ((run_tests)); then
    ctest --test-dir "$component_build" --output-on-failure --no-tests=error
  fi
  cmake --install "$component_build"
}

configure_build_test_install can \
  -DBUILD_TESTING="$tests_flag" \
  -DOA_CAN_ENABLE_SANITIZERS=OFF

configure_build_test_install model \
  -DOA_MODEL_BUILD_TESTS="$tests_flag" \
  -DOA_MODEL_SANITIZERS=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOA_DESCRIPTION_ROOT="$description_dir" \
  -DOA_XACRO_EXECUTABLE="$xacro_executable" \
  -DOA_XACRO_PYTHONPATH="$xacro_pythonpath" \
  -DOA_XACRO_AMENT_PREFIX=/opt/ros/lyrical

configure_build_test_install commission \
  -DBUILD_TESTING="$tests_flag" \
  -DOA_COMMISSION_ENABLE_SANITIZERS=OFF

configure_build_test_install transport \
  -DBUILD_TESTING="$tests_flag" \
  -DOA_TRANSPORT_ENABLE_SANITIZERS=OFF \
  -DOA_TRANSPORT_ENABLE_THREAD_SANITIZER=OFF \
  -DOA_TRANSPORT_BUILD_VCAN_SMOKE=OFF

configure_build_test_install control \
  -DBUILD_TESTING="$tests_flag" \
  -DOA_CONTROL_BUILD_TESTS="$tests_flag" \
  -DOA_CONTROL_SANITIZERS=OFF \
  -DOA_CONTROL_TSAN=OFF
