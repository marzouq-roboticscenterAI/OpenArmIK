#!/usr/bin/env bash
# Native build implementation. This file is sourced; it does not run a body.

openarm_build_native_assert_cache_value() {
  local cache_file=$1 key=$2 expected=$3 line value=
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
    return 1
  fi
}

openarm_build_native_configure_component() {
  local component=$1
  shift
  local component_build="$build_root/$component"
  if ((!reuse_build_trees)); then
    cmake -E remove_directory "$component_build"
  fi
  cmake -S "$root_dir/$component" -B "$component_build" \
    -DCMAKE_BUILD_TYPE="$build_type" \
    -DCMAKE_INSTALL_PREFIX="$install_prefix" \
    -DCMAKE_PREFIX_PATH="$install_prefix" \
    "$@"
  openarm_build_native_assert_cache_value "$component_build/CMakeCache.txt" \
    CMAKE_INSTALL_PREFIX "$install_prefix"
  openarm_build_native_assert_cache_value "$component_build/CMakeCache.txt" \
    CMAKE_PREFIX_PATH "$install_prefix"
  case "$component" in
    transport)
      openarm_build_native_assert_cache_value "$component_build/CMakeCache.txt" \
        OpenArmCan_DIR "$install_prefix/lib/cmake/OpenArmCan"
      ;;
    control)
      openarm_build_native_assert_cache_value "$component_build/CMakeCache.txt" \
        openarm_model_DIR "$install_prefix/lib/cmake/openarm_model"
      ;;
    runtime)
      openarm_build_native_assert_cache_value "$component_build/CMakeCache.txt" \
        openarm_control_DIR "$install_prefix/lib/cmake/openarm_control"
      ;;
  esac
  cmake --build "$component_build" --parallel "$jobs"
  if ((run_tests)); then
    ctest --test-dir "$component_build" --output-on-failure --no-tests=error
  fi
  cmake --install "$component_build"
}

openarm_build_native_body() {
  local root_dir=$1 build_root=$2 install_prefix=$3 build_type=$4
  local run_tests=$5 reuse_build_trees=$6 jobs=$7
  local description_dir="$root_dir/upstream/openarm_description"
  local xacro_executable=/opt/ros/lyrical/bin/xacro
  local xacro_pythonpath=/opt/ros/lyrical/lib/python3.14/site-packages
  local tests_flag=OFF control_symbols commission_symbols transport_symbols
  local runtime_symbols runtime_references control_config model_header control_header
  local consumer_build
  local -a model_test_args=()

  mkdir -p "$build_root" "$install_prefix"
  if ((run_tests)); then
    tests_flag=ON
  fi

  openarm_build_native_configure_component can \
    -DBUILD_TESTING="$tests_flag" \
    -DOA_CAN_ENABLE_SANITIZERS=OFF

  if ((run_tests)); then
    model_test_args=(-DPython3_EXECUTABLE=/usr/bin/python3)
  fi
  openarm_build_native_configure_component model \
    -DOA_MODEL_BUILD_TESTS="$tests_flag" \
    -DOA_MODEL_SANITIZERS=OFF \
    "${model_test_args[@]}" \
    -DOA_DESCRIPTION_ROOT="$description_dir" \
    -DOA_XACRO_EXECUTABLE="$xacro_executable" \
    -DOA_XACRO_PYTHONPATH="$xacro_pythonpath" \
    -DOA_XACRO_AMENT_PREFIX=/opt/ros/lyrical

  openarm_build_native_configure_component commission \
    -DBUILD_TESTING="$tests_flag" \
    -DOA_COMMISSION_ENABLE_SANITIZERS=OFF

  openarm_build_native_configure_component transport \
    -DBUILD_TESTING="$tests_flag" \
    -DOpenArmCan_DIR="$install_prefix/lib/cmake/OpenArmCan" \
    -DOA_TRANSPORT_ENABLE_SANITIZERS=OFF \
    -DOA_TRANSPORT_ENABLE_THREAD_SANITIZER=OFF \
    -DOA_TRANSPORT_BUILD_VCAN_SMOKE=OFF

  openarm_build_native_configure_component control \
    -DBUILD_TESTING="$tests_flag" \
    -Dopenarm_model_DIR="$install_prefix/lib/cmake/openarm_model" \
    -DOA_CONTROL_BUILD_TESTS="$tests_flag" \
    -DOA_CONTROL_SANITIZERS=OFF \
    -DOA_CONTROL_TSAN=OFF

  openarm_build_native_configure_component runtime \
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
    return 1
  fi
  if [[ "$commission_symbols" == *openarm::commission::test::* ]]; then
    printf 'Installed commission archive exposes test-only symbols\n' >&2
    return 1
  fi
  if [[ "$transport_symbols" == *' T oa_can_'* ]]; then
    printf 'Installed transport archive embeds CAN implementation symbols\n' >&2
    return 1
  fi
  if [[ "$runtime_symbols" == *'_test_'* ]]; then
    printf 'Installed runtime archive exposes test-only symbols\n' >&2
    return 1
  fi
  if [[ "$runtime_references" == *'oa_can_'* ||
        "$runtime_references" == *'oa_transport_'* ]]; then
    printf 'Installed runtime archive reaches CAN codec or transport symbols\n' >&2
    return 1
  fi

  if ((run_tests)); then
    control_config="$install_prefix/lib/cmake/openarm_control/openarm_controlConfig.cmake"
    model_header=$(<"$install_prefix/include/openarm_model.h")
    control_header=$(<"$install_prefix/include/openarm_control.h")
    if [[ -f "$control_config" ]] &&
       [[ "$model_header" == *OPENARM_DISABLE_LEGACY_GENERIC_STATUS* ]] &&
       [[ "$control_header" == *OPENARM_DISABLE_LEGACY_GENERIC_STATUS* ]]; then
      consumer_build="$build_root/installed_native_consumer"
      if ((!reuse_build_trees)); then
        cmake -E remove_directory "$consumer_build"
      fi
      cmake -S "$root_dir/tests/installed_native_consumer" -B "$consumer_build" \
        -DCMAKE_BUILD_TYPE="$build_type" \
        -DCMAKE_PREFIX_PATH="$install_prefix" \
        -DOpenArmCan_DIR="$install_prefix/lib/cmake/OpenArmCan" \
        -Dopenarm_model_DIR="$install_prefix/lib/cmake/openarm_model" \
        -Dopenarm_commission_DIR="$install_prefix/lib/cmake/openarm_commission" \
        -DOpenArmTransport_DIR="$install_prefix/lib/cmake/OpenArmTransport" \
        -Dopenarm_control_DIR="$install_prefix/lib/cmake/openarm_control" \
        -Dopenarm_runtime_DIR="$install_prefix/lib/cmake/openarm_runtime"
      openarm_build_native_assert_cache_value "$consumer_build/CMakeCache.txt" \
        CMAKE_PREFIX_PATH "$install_prefix"
      openarm_build_native_assert_cache_value "$consumer_build/CMakeCache.txt" \
        OpenArmCan_DIR "$install_prefix/lib/cmake/OpenArmCan"
      openarm_build_native_assert_cache_value "$consumer_build/CMakeCache.txt" \
        openarm_model_DIR "$install_prefix/lib/cmake/openarm_model"
      openarm_build_native_assert_cache_value "$consumer_build/CMakeCache.txt" \
        openarm_commission_DIR "$install_prefix/lib/cmake/openarm_commission"
      openarm_build_native_assert_cache_value "$consumer_build/CMakeCache.txt" \
        OpenArmTransport_DIR "$install_prefix/lib/cmake/OpenArmTransport"
      openarm_build_native_assert_cache_value "$consumer_build/CMakeCache.txt" \
        openarm_control_DIR "$install_prefix/lib/cmake/openarm_control"
      openarm_build_native_assert_cache_value "$consumer_build/CMakeCache.txt" \
        openarm_runtime_DIR "$install_prefix/lib/cmake/openarm_runtime"
      cmake --build "$consumer_build" --parallel "$jobs"
      "$consumer_build/openarm_installed_c11"
      "$consumer_build/openarm_installed_cxx17"
      "$consumer_build/openarm_runtime_only_c11"
      "$consumer_build/openarm_runtime_only_cxx17"
      if ldd "$consumer_build/openarm_runtime_only_c11" |
           grep -Eiq 'python|openarm_(can|transport)'; then
        printf 'Runtime-only installed consumer has a forbidden runtime dependency\n' >&2
        return 1
      fi
    else
      printf '%s\n' \
        'Installed all-header consumers deferred until control export/status integration'
    fi
  fi
}
