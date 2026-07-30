#!/usr/bin/env bash
# Transactional CMake cache provenance. This file is sourced.

OPENARM_BUILD_STATE_FILE=.openarm-build-state-v1
OPENARM_INSTALL_STATE_FILE=.openarm-install-root-v1

openarm_build_state_record_command() {
  local label=$1 specification=$2 spec_hash token path real hash count=0
  local -a tokens=()
  spec_hash=$(printf '%s' "$specification" | sha256sum) || return 1
  mapfile -d '' -t tokens < <(/usr/bin/python3 -c \
    'import shlex,sys; [sys.stdout.buffer.write(x.encode()+b"\0") for x in shlex.split(sys.argv[1])]' \
    "$specification")
  ((${#tokens[@]})) || return 1
  printf 'request\0%s\0%s\0' "$label" "${spec_hash%% *}"
  for token in "${tokens[@]}"; do
    [[ "$token" != -* ]] || continue
    if [[ "$token" == */* ]]; then
      [[ -e "$token" ]] || continue
      path=$token
    else
      path=$(command -v -- "$token" 2>/dev/null) || continue
    fi
    real=$(realpath -e -- "$path") || return 1
    [[ -f "$real" && -x "$real" ]] || continue
    hash=$(sha256sum -- "$real") || return 1
    printf 'request_file\0%s\0%s\0%s\0' "$label" "$real" "${hash%% *}"
    ((count += 1))
  done
  ((count > 0))
}

openarm_build_state_record_cmake_launcher() {
  local record_kind=$1 label=$2 specification=$3 argv0 path real hash
  printf '%s_spec\0%s\0%s\0' "$record_kind" "$label" "$specification"
  if [[ -z "$specification" ]]; then
    printf '%s_file\0%s\0none\0' "$record_kind" "$label"
    return 0
  fi
  # CMake launcher values are semicolon lists. Only the first element is the
  # executable; the remaining elements are fixed arguments already bound by
  # the raw specification above. Never interpret the value as shell input.
  [[ "$specification" != *$'\n'* && "$specification" != *$'\r'* &&
     "$specification" != *'\\'* && "$specification" != *'$<'* ]] || return 1
  argv0=${specification%%;*}
  [[ -n "$argv0" && "$argv0" != -* && "$argv0" != *[[:space:]]* ]] || return 1
  if [[ "$argv0" == */* ]]; then
    [[ "$argv0" == /* && -e "$argv0" ]] || return 1
    path=$argv0
  else
    path=$(command -v -- "$argv0" 2>/dev/null) || return 1
  fi
  real=$(realpath -e -- "$path") || return 1
  [[ -f "$real" && -x "$real" ]] || return 1
  hash=$(sha256sum -- "$real") || return 1
  printf '%s_file\0%s\0%s\0%s\0' "$record_kind" "$label" "$real" \
    "${hash%% *}"
}

openarm_build_state_requested_digest() {
  local scope=$1 build_type=$2 testing=$3 coverage=$4 install_prefix=$5
  local manifest generator backend toolchain toolchain_real hash key value
  manifest=$(mktemp "${TMPDIR:-/tmp}/openarmik-build-request.XXXXXX") || return 1
  (
    printf 'OPENARM_BUILD_REQUEST_V1\0%s\0%s\0%s\0%s\0%s\0' \
      "$scope" "$build_type" "$testing" "$coverage" "$install_prefix"
    openarm_build_state_record_command CMAKE "$(command -v cmake)" || exit 1
    openarm_build_state_record_command PYTHON /usr/bin/python3 || exit 1
    openarm_build_state_record_command CC "${CC:-cc}" || exit 1
    openarm_build_state_record_command CXX "${CXX:-c++}" || exit 1
    openarm_build_state_record_command AR "${AR:-ar}" || exit 1
    openarm_build_state_record_command RANLIB "${RANLIB:-ranlib}" || exit 1
    openarm_build_state_record_command LD "${LD:-ld}" || exit 1
    openarm_build_state_record_command NM "${NM:-nm}" || exit 1
    openarm_build_state_record_command STRIP "${STRIP:-strip}" || exit 1
    openarm_build_state_record_command OBJCOPY "${OBJCOPY:-objcopy}" || exit 1
    generator=${CMAKE_GENERATOR:-Unix Makefiles}
    if [[ -n ${CMAKE_MAKE_PROGRAM:-} ]]; then
      backend=$CMAKE_MAKE_PROGRAM
    elif [[ "$generator" == *Ninja* ]]; then
      backend=ninja
    elif [[ "$generator" == *Makefiles* ]]; then
      backend=make
    else
      exit 1
    fi
    openarm_build_state_record_command CMAKE_MAKE_PROGRAM "$backend" || exit 1
    printf 'generator\0%s\0' "$generator"
    toolchain=${CMAKE_TOOLCHAIN_FILE:-}
    if [[ -n "$toolchain" ]]; then
      [[ "$toolchain" == /* && -f "$toolchain" ]] || exit 1
      toolchain_real=$(realpath -e -- "$toolchain") || exit 1
      hash=$(sha256sum -- "$toolchain_real") || exit 1
      printf 'toolchain\0%s\0%s\0' "$toolchain_real" "${hash%% *}"
    else
      printf 'toolchain\0none\0'
    fi
    for key in CFLAGS CXXFLAGS CPPFLAGS LDFLAGS CMAKE_GENERATOR_PLATFORM \
      CMAKE_GENERATOR_TOOLSET CMAKE_GENERATOR_INSTANCE \
      Python3_EXECUTABLE; do
      value=${!key-}
      hash=$(printf '%s' "$value" | sha256sum) || exit 1
      printf 'option\0%s\0%s\0' "$key" "${hash%% *}"
    done
    for key in CMAKE_C_COMPILER_LAUNCHER CMAKE_CXX_COMPILER_LAUNCHER \
      CMAKE_C_LINKER_LAUNCHER CMAKE_CXX_LINKER_LAUNCHER; do
      value=${!key-}
      openarm_build_state_record_cmake_launcher request_launcher "$key" \
        "$value" || exit 1
    done
  ) > "$manifest" || { rm -f -- "$manifest"; return 1; }
  sha256sum -- "$manifest" | awk '{print $1}'
  rm -f -- "$manifest"
}

openarm_build_state_cache_get() {
  local cache=$1 wanted=$2 required=$3 line value= count=0
  while IFS= read -r line || [[ -n "$line" ]]; do
    case "$line" in
      "$wanted":*=*)
        ((count += 1))
        value=${line#*=}
        ;;
    esac
  done < "$cache"
  ((count <= 1)) || {
    printf 'Duplicate CMake cache key %s in %s\n' "$wanted" "$cache" >&2
    return 1
  }
  if ((required && count != 1)); then
    printf 'Missing CMake cache key %s in %s\n' "$wanted" "$cache" >&2
    return 1
  fi
  OPENARM_CACHE_PRESENT=$count
  OPENARM_CACHE_VALUE=$value
}

openarm_build_state_actual_digest() {
  local build_root=$1 manifest cache component component_dir key real hash found
  local cxx_present root_real
  shift
  local -a components=("$@")
  local -a required_values=(
    CMAKE_PROJECT_NAME CMAKE_GENERATOR CMAKE_GENERATOR_INSTANCE
    CMAKE_GENERATOR_PLATFORM CMAKE_GENERATOR_TOOLSET CMAKE_BUILD_TYPE
    CMAKE_INSTALL_PREFIX CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG
    CMAKE_C_FLAGS_RELEASE CMAKE_C_FLAGS_RELWITHDEBINFO CMAKE_C_FLAGS_MINSIZEREL
    CMAKE_EXE_LINKER_FLAGS CMAKE_EXE_LINKER_FLAGS_DEBUG
    CMAKE_EXE_LINKER_FLAGS_RELEASE CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO
    CMAKE_EXE_LINKER_FLAGS_MINSIZEREL CMAKE_SHARED_LINKER_FLAGS
    CMAKE_SHARED_LINKER_FLAGS_DEBUG CMAKE_SHARED_LINKER_FLAGS_RELEASE
    CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL
    CMAKE_MODULE_LINKER_FLAGS CMAKE_MODULE_LINKER_FLAGS_DEBUG
    CMAKE_MODULE_LINKER_FLAGS_RELEASE CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO
    CMAKE_MODULE_LINKER_FLAGS_MINSIZEREL CMAKE_STATIC_LINKER_FLAGS
    CMAKE_STATIC_LINKER_FLAGS_DEBUG CMAKE_STATIC_LINKER_FLAGS_RELEASE
    CMAKE_STATIC_LINKER_FLAGS_RELWITHDEBINFO CMAKE_STATIC_LINKER_FLAGS_MINSIZEREL)
  local -a optional_values=(
    CMAKE_C_COMPILER_ARG1 CMAKE_CXX_COMPILER_ARG1 CMAKE_TOOLCHAIN_FILE
    CMAKE_PREFIX_PATH CMAKE_SYSROOT CMAKE_SYSROOT_COMPILE CMAKE_SYSROOT_LINK
    CMAKE_C_COMPILER_TARGET CMAKE_CXX_COMPILER_TARGET
    CMAKE_C_COMPILER_EXTERNAL_TOOLCHAIN CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN
    CMAKE_FIND_ROOT_PATH BUILD_TESTING OA_MODEL_BUILD_TESTS
    OA_CONTROL_BUILD_TESTS Python_EXECUTABLE CMAKE_C_COMPILER_LAUNCHER
    CMAKE_CXX_COMPILER_LAUNCHER CMAKE_C_LINKER_LAUNCHER
    CMAKE_CXX_LINKER_LAUNCHER)
  local -a required_tools=(CMAKE_C_COMPILER CMAKE_MAKE_PROGRAM CMAKE_AR
    CMAKE_RANLIB CMAKE_LINKER CMAKE_NM CMAKE_STRIP CMAKE_OBJCOPY)
  local -a optional_tools=(CMAKE_CXX_COMPILER CMAKE_C_COMPILER_AR
    CMAKE_C_COMPILER_RANLIB CMAKE_CXX_COMPILER_AR CMAKE_CXX_COMPILER_RANLIB)
  local -a optional_launchers=(CMAKE_C_COMPILER_LAUNCHER
    CMAKE_CXX_COMPILER_LAUNCHER CMAKE_C_LINKER_LAUNCHER
    CMAKE_CXX_LINKER_LAUNCHER)
  [[ -d "$build_root" && ! -L "$build_root" ]] || return 1
  root_real=$(realpath -e -- "$build_root") || return 1
  [[ "$root_real" == "$build_root" ]] || return 1
  manifest=$(mktemp "${TMPDIR:-/tmp}/openarmik-cache-state.XXXXXX") || return 1
  (
    printf 'OPENARM_ACTUAL_CMAKE_V1\0'
    for component in "${components[@]}"; do
      [[ "$component" =~ ^[A-Za-z0-9_+-]+$ ]] || exit 1
      component_dir="$root_real/$component"
      [[ -d "$component_dir" && ! -L "$component_dir" ]] || exit 1
      [[ $(realpath -e -- "$component_dir") == "$component_dir" ]] || exit 1
      cache="$component_dir/CMakeCache.txt"
      [[ -f "$cache" && ! -L "$cache" ]] || exit 1
      [[ $(realpath -e -- "$cache") == "$cache" ]] || exit 1
      printf 'component\0%s\0' "$component"
      for key in "${required_values[@]}"; do
        openarm_build_state_cache_get "$cache" "$key" 1 || exit 1
        printf 'value\0%s\0%s\0' "$key" "$OPENARM_CACHE_VALUE"
      done
      openarm_build_state_cache_get "$cache" CMAKE_CXX_COMPILER 0 || exit 1
      cxx_present=$OPENARM_CACHE_PRESENT
      if ((cxx_present)); then
        for key in CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE \
          CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_CXX_FLAGS_MINSIZEREL; do
          openarm_build_state_cache_get "$cache" "$key" 1 || exit 1
          printf 'value\0%s\0%s\0' "$key" "$OPENARM_CACHE_VALUE"
        done
      fi
      for key in "${optional_values[@]}"; do
        openarm_build_state_cache_get "$cache" "$key" 0 || exit 1
        printf 'optional\0%s\0%s\0%s\0' "$key" "$OPENARM_CACHE_PRESENT" \
          "$OPENARM_CACHE_VALUE"
      done
      for key in "${required_tools[@]}"; do
        openarm_build_state_cache_get "$cache" "$key" 1 || exit 1
        real=$(realpath -e -- "$OPENARM_CACHE_VALUE") || exit 1
        [[ -f "$real" && -x "$real" ]] || exit 1
        hash=$(sha256sum -- "$real") || exit 1
        printf 'tool\0%s\0%s\0%s\0' "$key" "$real" "${hash%% *}"
      done
      for key in "${optional_tools[@]}"; do
        openarm_build_state_cache_get "$cache" "$key" 0 || exit 1
        if ((OPENARM_CACHE_PRESENT)); then
          real=$(realpath -e -- "$OPENARM_CACHE_VALUE") || exit 1
          [[ -f "$real" && -x "$real" ]] || exit 1
          hash=$(sha256sum -- "$real") || exit 1
          printf 'tool\0%s\0%s\0%s\0' "$key" "$real" "${hash%% *}"
        else
          printf 'tool_missing\0%s\0' "$key"
        fi
      done
      for key in "${optional_launchers[@]}"; do
        openarm_build_state_cache_get "$cache" "$key" 0 || exit 1
        if ((OPENARM_CACHE_PRESENT)); then
          openarm_build_state_record_cmake_launcher actual_launcher "$key" \
            "$OPENARM_CACHE_VALUE" || exit 1
        else
          printf 'actual_launcher_missing\0%s\0' "$key"
        fi
      done
      if [[ "$component" == openarm_control_msgs ||
            "$component" == openarm_description ||
            "$component" == openarm_ik_ros ]]; then
        openarm_build_state_cache_get "$cache" Python3_EXECUTABLE 1 || exit 1
      else
        openarm_build_state_cache_get "$cache" Python3_EXECUTABLE 0 || exit 1
      fi
      if ((OPENARM_CACHE_PRESENT)); then
        real=$(realpath -e -- "$OPENARM_CACHE_VALUE") || exit 1
        [[ -f "$real" && -x "$real" ]] || exit 1
        hash=$(sha256sum -- "$real") || exit 1
        printf 'python\0%s\0%s\0' "$real" "${hash%% *}"
      else
        printf 'python_missing\0'
      fi
      openarm_build_state_cache_get "$cache" CMAKE_TOOLCHAIN_FILE 0 || exit 1
      if ((OPENARM_CACHE_PRESENT)) && [[ -n "$OPENARM_CACHE_VALUE" ]]; then
        real=$(realpath -e -- "$OPENARM_CACHE_VALUE") || exit 1
        [[ -f "$real" ]] || exit 1
        hash=$(sha256sum -- "$real") || exit 1
        printf 'toolchain_file\0%s\0%s\0' "$real" "${hash%% *}"
      fi
    done
    while IFS= read -r -d '' found; do
      component=${found#"$root_real"/}; component=${component%/CMakeCache.txt}
      [[ " ${components[*]} " == *" $component "* ]] || exit 1
    done < <(find -P "$root_real" -mindepth 2 -maxdepth 2 -name CMakeCache.txt -print0)
  ) > "$manifest" || { rm -f -- "$manifest"; return 1; }
  sha256sum -- "$manifest" | awk '{print $1}'
  rm -f -- "$manifest"
}

openarm_build_state_read_completed() {
  local build_root=$1 expected_request=${2:-} state="$1/$OPENARM_BUILD_STATE_FILE"
  local actual line
  shift 2
  local -a components=("$@") lines=()
  [[ -f "$state" && ! -L "$state" ]] || return 1
  mapfile -t lines < "$state"
  ((${#lines[@]} == 4)) || return 1
  [[ "${lines[0]}" == OPENARM_BUILD_STATE_V1 &&
     "${lines[1]}" == 'phase completed' &&
     "${lines[2]}" =~ ^request\ [0-9a-f]{64}$ &&
     "${lines[3]}" =~ ^actual\ [0-9a-f]{64}$ ]] || return 1
  [[ -z "$expected_request" || "${lines[2]#request }" == "$expected_request" ]] || return 1
  actual=$(openarm_build_state_actual_digest "$build_root" "${components[@]}") || return 1
  [[ "${lines[3]#actual }" == "$actual" ]] || return 1
  printf '%s\n' "${lines[2]#request } ${lines[3]#actual }"
}

openarm_build_state_write() {
  local build_root=$1 phase=$2 request=$3 actual=${4:-} temporary
  mkdir -p -- "$build_root"
  temporary=$(mktemp "$build_root/.openarm-build-state.XXXXXX") || return 1
  {
    printf 'OPENARM_BUILD_STATE_V1\nphase %s\nrequest %s\n' "$phase" "$request"
    [[ "$phase" != completed ]] || printf 'actual %s\n' "$actual"
  } > "$temporary" || { rm -f -- "$temporary"; return 1; }
  chmod 0600 -- "$temporary"
  mv -fT -- "$temporary" "$build_root/$OPENARM_BUILD_STATE_FILE"
}

openarm_build_state_publish_completed() {
  local build_root=$1 request=$2 actual
  shift 2
  actual=$(openarm_build_state_actual_digest "$build_root" "$@") || return 1
  openarm_build_state_write "$build_root" completed "$request" "$actual"
}

openarm_build_state_validate_output_root() {
  local root_dir=$1 output_root=$2 root_real parent base parent_real
  root_real=$(realpath -e -- "$root_dir") || return 1
  parent=$(dirname -- "$output_root"); base=$(basename -- "$output_root")
  parent_real=$(realpath -e -- "$parent") || return 2
  [[ "$output_root" == /* && -d "$parent_real" && ! -L "$parent" &&
     "$output_root" == "$parent_real/$base" && "$base" != . &&
     "$base" != .. && "$base" != .git ]] || return 2
  case "$output_root" in
    "$root_real/ros2_ws") ;;
    "$root_real"|"$root_real"/*|/*/.git|/*/.git/*) return 2 ;;
    *) case "$root_real" in "$output_root"/*) return 2 ;; esac ;;
  esac
  [[ ! -e "$output_root" && ! -L "$output_root" ]] ||
    [[ -d "$output_root" && ! -L "$output_root" &&
       $(realpath -e -- "$output_root") == "$output_root" ]]
}

openarm_build_state_remove_output_child() {
  local root_dir=$1 output_root=$2 child=$3 target
  openarm_build_state_validate_output_root "$root_dir" "$output_root" || return 2
  case "$child" in native_build|build|log|install) ;;
    *) return 2 ;;
  esac
  target="$output_root/$child"
  if [[ -e "$target" || -L "$target" ]]; then
    [[ -d "$target" && ! -L "$target" &&
       $(realpath -e -- "$target") == "$target" ]] || return 2
  fi
  rm -rf --one-file-system -- "$target" || return 1
  [[ ! -e "$target" && ! -L "$target" ]]
}

openarm_build_state_remove_owned_tree() {
  local root_dir=$1 target=$2 marker=$3 root_real parent base parent_real entries
  root_real=$(realpath -e -- "$root_dir") || return 1
  parent=$(dirname -- "$target"); base=$(basename -- "$target")
  parent_real=$(realpath -e -- "$parent") || return 2
  [[ "$target" == /* && -d "$parent_real" && ! -L "$parent" &&
     "$target" == "$parent_real/$base" && "$base" != . && "$base" != .. &&
     "$base" != .git && "$marker" =~ ^\.openarm-[A-Za-z0-9._-]+$ ]] || return 2
  case "$target" in
    /|"${HOME:-/nonexistent}"|"$root_real"|"$root_real"/*|/*/.git|/*/.git/*)
      return 2 ;;
    *) case "$root_real" in "$target"/*) return 2 ;; esac ;;
  esac
  if [[ -e "$target" || -L "$target" ]]; then
    [[ -d "$target" && ! -L "$target" &&
       $(realpath -e -- "$target") == "$target" ]] || return 2
    if [[ -e "$target/$marker" || -L "$target/$marker" ]]; then
      [[ -f "$target/$marker" && ! -L "$target/$marker" ]] || return 2
    else
      entries=$(find -P "$target" -mindepth 1 -maxdepth 1 -print -quit) || return 1
      [[ -z "$entries" ]] || return 2
    fi
  fi
  rm -rf --one-file-system -- "$target" || return 1
  [[ ! -e "$target" && ! -L "$target" ]]
}
