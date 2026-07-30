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

Build and install CAN, model, commission, transport, control, and runtime in dependency
order. Build and install paths must be supplied explicitly. Reusing a build
root with a different install prefix deterministically refreshes dependencies.

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

assert_cache_value() {
  local cache_file=$1
  local key=$2
  local expected=$3
  local line value=
  while IFS= read -r line; do
    case "$line" in
      "$key":*=*)
        value=${line#*=}
        break
        ;;
    esac
  done < "$cache_file"
  if [[ "$value" != "$expected" ]]; then
    printf 'Unexpected %s in %s: expected %s, found %s\n' \
      "$key" "$cache_file" "$expected" "${value:-unset}" >&2
    exit 1
  fi
}

configure_build_test_install() {
  local component=$1
  shift
  local component_build="$build_root/$component"
  cmake -E remove_directory "$component_build"
  cmake -S "$root_dir/$component" -B "$component_build" \
    -DCMAKE_BUILD_TYPE="$build_type" \
    -DCMAKE_INSTALL_PREFIX="$install_prefix" \
    -DCMAKE_PREFIX_PATH="$install_prefix" \
    "$@"
  assert_cache_value "$component_build/CMakeCache.txt" \
    CMAKE_INSTALL_PREFIX "$install_prefix"
  assert_cache_value "$component_build/CMakeCache.txt" \
    CMAKE_PREFIX_PATH "$install_prefix"
  case "$component" in
    transport)
      assert_cache_value "$component_build/CMakeCache.txt" \
        OpenArmCan_DIR "$install_prefix/lib/cmake/OpenArmCan"
      ;;
    control)
      assert_cache_value "$component_build/CMakeCache.txt" \
        openarm_model_DIR "$install_prefix/lib/cmake/openarm_model"
      ;;
    runtime)
      assert_cache_value "$component_build/CMakeCache.txt" \
        openarm_control_DIR "$install_prefix/lib/cmake/openarm_control"
      ;;
  esac
  cmake --build "$component_build" --parallel
  if ((run_tests)); then
    ctest --test-dir "$component_build" --output-on-failure --no-tests=error
  fi
  cmake --install "$component_build"
}

configure_build_test_install can \
  -DBUILD_TESTING="$tests_flag" \
  -DOA_CAN_ENABLE_SANITIZERS=OFF

model_test_args=()
if ((run_tests)); then
  model_test_args=(-DPython3_EXECUTABLE=/usr/bin/python3)
fi

configure_build_test_install model \
  -DOA_MODEL_BUILD_TESTS="$tests_flag" \
  -DOA_MODEL_SANITIZERS=OFF \
  "${model_test_args[@]}" \
  -DOA_DESCRIPTION_ROOT="$description_dir" \
  -DOA_XACRO_EXECUTABLE="$xacro_executable" \
  -DOA_XACRO_PYTHONPATH="$xacro_pythonpath" \
  -DOA_XACRO_AMENT_PREFIX=/opt/ros/lyrical

configure_build_test_install commission \
  -DBUILD_TESTING="$tests_flag" \
  -DOA_COMMISSION_ENABLE_SANITIZERS=OFF

configure_build_test_install transport \
  -DBUILD_TESTING="$tests_flag" \
  -DOpenArmCan_DIR="$install_prefix/lib/cmake/OpenArmCan" \
  -DOA_TRANSPORT_ENABLE_SANITIZERS=OFF \
  -DOA_TRANSPORT_ENABLE_THREAD_SANITIZER=OFF \
  -DOA_TRANSPORT_BUILD_VCAN_SMOKE=OFF

configure_build_test_install control \
  -DBUILD_TESTING="$tests_flag" \
  -Dopenarm_model_DIR="$install_prefix/lib/cmake/openarm_model" \
  -DOA_CONTROL_BUILD_TESTS="$tests_flag" \
  -DOA_CONTROL_SANITIZERS=OFF \
  -DOA_CONTROL_TSAN=OFF

configure_build_test_install runtime \
  -DBUILD_TESTING="$tests_flag" \
  -Dopenarm_model_DIR="$install_prefix/lib/cmake/openarm_model" \
  -Dopenarm_commission_DIR="$install_prefix/lib/cmake/openarm_commission" \
  -Dopenarm_control_DIR="$install_prefix/lib/cmake/openarm_control" \
  -DOA_RUNTIME_ENABLE_SANITIZERS=OFF \
  -DOA_RUNTIME_ENABLE_THREAD_SANITIZER=OFF

control_symbols=$(nm -gC --defined-only \
  "$install_prefix/lib/libopenarm_control.a")
commission_symbols=$(nm -gC --defined-only \
  "$install_prefix/lib/libopenarm_commission.a")
transport_symbols=$(nm -gC --defined-only \
  "$install_prefix/lib/libopenarm_transport.a")
runtime_symbols=$(nm -gC --defined-only \
  "$install_prefix/lib/libopenarm_runtime.a")
runtime_references=$(nm -u "$install_prefix/lib/libopenarm_runtime.a")
if [[ "$control_symbols" == *oa_control_test_* ]]; then
  printf 'Installed control archive exposes test-only symbols\n' >&2
  exit 1
fi
if [[ "$commission_symbols" == *openarm::commission::test::* ]]; then
  printf 'Installed commission archive exposes test-only symbols\n' >&2
  exit 1
fi
if [[ "$transport_symbols" == *' T oa_can_'* ]]; then
  printf 'Installed transport archive embeds CAN implementation symbols\n' >&2
  exit 1
fi
if [[ "$runtime_symbols" == *'_test_'* ]]; then
  printf 'Installed runtime archive exposes test-only symbols\n' >&2
  exit 1
fi
if [[ "$runtime_references" == *'oa_can_'* ||
      "$runtime_references" == *'oa_transport_'* ]]; then
  printf 'Installed runtime archive reaches CAN codec or transport symbols\n' >&2
  exit 1
fi

if ((run_tests)); then
  control_config="$install_prefix/lib/cmake/openarm_control/openarm_controlConfig.cmake"
  model_header=$(<"$install_prefix/include/openarm_model.h")
  control_header=$(<"$install_prefix/include/openarm_control.h")
  if [[ -f "$control_config" ]] &&
     [[ "$model_header" == *OPENARM_DISABLE_LEGACY_GENERIC_STATUS* ]] &&
     [[ "$control_header" == *OPENARM_DISABLE_LEGACY_GENERIC_STATUS* ]]; then
    consumer_build="$build_root/installed_native_consumer"
    cmake -E remove_directory "$consumer_build"
    cmake -S "$root_dir/tests/installed_native_consumer" -B "$consumer_build" \
      -DCMAKE_BUILD_TYPE="$build_type" \
      -DCMAKE_PREFIX_PATH="$install_prefix" \
      -DOpenArmCan_DIR="$install_prefix/lib/cmake/OpenArmCan" \
      -Dopenarm_model_DIR="$install_prefix/lib/cmake/openarm_model" \
      -Dopenarm_commission_DIR="$install_prefix/lib/cmake/openarm_commission" \
      -DOpenArmTransport_DIR="$install_prefix/lib/cmake/OpenArmTransport" \
      -Dopenarm_control_DIR="$install_prefix/lib/cmake/openarm_control" \
      -Dopenarm_runtime_DIR="$install_prefix/lib/cmake/openarm_runtime"
    assert_cache_value "$consumer_build/CMakeCache.txt" \
      CMAKE_PREFIX_PATH "$install_prefix"
    assert_cache_value "$consumer_build/CMakeCache.txt" \
      OpenArmCan_DIR "$install_prefix/lib/cmake/OpenArmCan"
    assert_cache_value "$consumer_build/CMakeCache.txt" \
      openarm_model_DIR "$install_prefix/lib/cmake/openarm_model"
    assert_cache_value "$consumer_build/CMakeCache.txt" \
      openarm_commission_DIR "$install_prefix/lib/cmake/openarm_commission"
    assert_cache_value "$consumer_build/CMakeCache.txt" \
      OpenArmTransport_DIR "$install_prefix/lib/cmake/OpenArmTransport"
    assert_cache_value "$consumer_build/CMakeCache.txt" \
      openarm_control_DIR "$install_prefix/lib/cmake/openarm_control"
    assert_cache_value "$consumer_build/CMakeCache.txt" \
      openarm_runtime_DIR "$install_prefix/lib/cmake/openarm_runtime"
    cmake --build "$consumer_build" --parallel
    "$consumer_build/openarm_installed_c11"
    "$consumer_build/openarm_installed_cxx17"
    "$consumer_build/openarm_runtime_only_c11"
    "$consumer_build/openarm_runtime_only_cxx17"
    if ldd "$consumer_build/openarm_runtime_only_c11" | \
         grep -Eiq 'python|openarm_(can|transport)'; then
      printf 'Runtime-only installed consumer has a forbidden runtime dependency\n' >&2
      exit 1
    fi
  else
    printf '%s\n' \
      'Installed all-header consumers deferred until control export/status integration'
  fi
fi
