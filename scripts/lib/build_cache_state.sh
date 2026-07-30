#!/usr/bin/env bash
# Transactional CMake cache provenance. This file is sourced.

OPENARM_BUILD_STATE_FILE=.openarm-build-state-v1

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
  local build_root=$1 manifest cache component key real hash found cxx_present
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
    OA_CONTROL_BUILD_TESTS Python_EXECUTABLE)
  local -a required_tools=(CMAKE_C_COMPILER CMAKE_MAKE_PROGRAM CMAKE_AR
    CMAKE_RANLIB CMAKE_LINKER CMAKE_NM CMAKE_STRIP CMAKE_OBJCOPY)
  local -a optional_tools=(CMAKE_CXX_COMPILER CMAKE_C_COMPILER_AR
    CMAKE_C_COMPILER_RANLIB CMAKE_CXX_COMPILER_AR CMAKE_CXX_COMPILER_RANLIB)
  [[ -d "$build_root" && ! -L "$build_root" ]] || return 1
  manifest=$(mktemp "${TMPDIR:-/tmp}/openarmik-cache-state.XXXXXX") || return 1
  (
    printf 'OPENARM_ACTUAL_CMAKE_V1\0'
    for component in "${components[@]}"; do
      cache="$build_root/$component/CMakeCache.txt"
      [[ -f "$cache" && ! -L "$cache" ]] || exit 1
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
      component=${found#"$build_root"/}; component=${component%/CMakeCache.txt}
      [[ " ${components[*]} " == *" $component "* ]] || exit 1
    done < <(find "$build_root" -mindepth 2 -maxdepth 2 -name CMakeCache.txt -print0)
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

openarm_build_state_remove_tree() {
  local root_dir=$1 target=$2 resolved root_real protected
  resolved=$(realpath -m -- "$target") || return 1
  root_real=$(realpath -e -- "$root_dir") || return 1
  case "$resolved" in
    /|/etc|/home|/opt|/root|/tmp|/usr|/var|"$root_real"|"$(dirname "$root_real")"|"${HOME:-/nonexistent}")
      printf 'Refusing unsafe cache cleanup: %s\n' "$resolved" >&2; return 2 ;;
  esac
  for protected in can model commission transport control runtime scripts tests \
      upstream ros2_ws/src; do
    protected="$root_real/$protected"
    case "$resolved" in "$protected"|"$protected"/*)
      printf 'Refusing source cleanup: %s\n' "$resolved" >&2; return 2 ;; esac
  done
  rm -rf --one-file-system -- "$resolved"
  [[ ! -e "$resolved" && ! -L "$resolved" ]]
}
