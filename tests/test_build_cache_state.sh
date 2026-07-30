#!/usr/bin/env bash
set -euo pipefail
root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root_dir/scripts/lib/build_cache_state.sh"
work_root=$(mktemp -d "${TMPDIR:-/tmp}/openarmik-cache-state-test.XXXXXX")
cleanup() { rm -rf -- "$work_root"; }
trap cleanup EXIT

source_dir="$work_root/source"
build_root="$work_root/build"
install_root="$work_root/install"
mkdir -p "$source_dir"
printf 'int main(void) { return 0; }\n' > "$source_dir/main.c"
printf 'int helper() { return 0; }\n' > "$source_dir/helper.cpp"
cat > "$source_dir/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(openarm_cache_probe LANGUAGES C CXX)
include(CTest)
add_executable(cache_probe main.c helper.cpp)
install(TARGETS cache_probe RUNTIME DESTINATION bin)
EOF

wrapper_a="$work_root/cc-a"
wrapper_b="$work_root/cc-b"
printf '#!/usr/bin/env bash\nexec /usr/bin/cc "$@"\n' > "$wrapper_a"
printf '#!/usr/bin/env bash\nexec /usr/bin/cc "$@"\n' > "$wrapper_b"
chmod +x "$wrapper_a" "$wrapper_b"
toolchain_a="$work_root/toolchain-a.cmake"
toolchain_b="$work_root/toolchain-b.cmake"
printf 'set(OPENARM_CACHE_PROBE one CACHE STRING "")\n' > "$toolchain_a"
printf 'set(OPENARM_CACHE_PROBE two CACHE STRING "")\n' > "$toolchain_b"

run_transaction() {
  local request request_after effective_reuse=0
  request=$(openarm_build_state_requested_digest probe Release 1 0 "$install_root")
  if openarm_build_state_read_completed "$build_root" "$request" probe \
      >/dev/null 2>&1; then
    effective_reuse=1
  else
    openarm_build_state_remove_tree "$root_dir" "$build_root"
  fi
  openarm_build_state_remove_tree "$root_dir" "$install_root"
  openarm_build_state_write "$build_root" pending "$request"
  local -a configure_args=(-S "$source_dir" -B "$build_root/probe"
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
    -DCMAKE_INSTALL_PREFIX="$install_root")
  cmake "${configure_args[@]}" >/dev/null
  if [[ ${OPENARM_CACHE_TEST_FAIL:-0} == 1 ]]; then
    return 9
  fi
  cmake --build "$build_root/probe" --parallel 1 >/dev/null
  cmake --install "$build_root/probe" >/dev/null
  request_after=$(openarm_build_state_requested_digest probe Release 1 0 \
    "$install_root")
  [[ "$request_after" == "$request" ]]
  openarm_build_state_publish_completed "$build_root" "$request" probe
  LAST_REUSED=$effective_reuse
}

CC="$wrapper_a" CXX=/usr/bin/c++ CFLAGS=-DREQUEST_ONE=1 \
  CXXFLAGS=-DREQUEST_CXX=1 LDFLAGS=-Wl,--as-needed run_transaction
[[ "$LAST_REUSED" == 0 ]]
touch "$build_root/probe/reuse-marker"
CC="$wrapper_a" CXX=/usr/bin/c++ CFLAGS=-DREQUEST_ONE=1 \
  CXXFLAGS=-DREQUEST_CXX=1 LDFLAGS=-Wl,--as-needed run_transaction
[[ "$LAST_REUSED" == 1 && -e "$build_root/probe/reuse-marker" ]]

# Changed compiler request cannot be silently ignored by an old cache.
CC="$wrapper_b" CXX=/usr/bin/c++ CFLAGS=-DREQUEST_ONE=1 \
  CXXFLAGS=-DREQUEST_CXX=1 LDFLAGS=-Wl,--as-needed run_transaction
[[ "$LAST_REUSED" == 0 && ! -e "$build_root/probe/reuse-marker" ]]
openarm_build_state_cache_get "$build_root/probe/CMakeCache.txt" CMAKE_C_COMPILER 1
[[ $(realpath -e "$OPENARM_CACHE_VALUE") == $(realpath -e "$wrapper_b") ]]

# Toolchain path and initialization flags each force a fresh configure.
touch "$build_root/probe/toolchain-marker"
CC="$wrapper_b" CXX=/usr/bin/c++ CMAKE_TOOLCHAIN_FILE="$toolchain_a" \
  CFLAGS=-DTOOLCHAIN_A=1 CXXFLAGS=-DREQUEST_CXX=1 LDFLAGS=-Wl,--as-needed \
  run_transaction
[[ "$LAST_REUSED" == 0 && ! -e "$build_root/probe/toolchain-marker" ]]
openarm_build_state_cache_get "$build_root/probe/CMakeCache.txt" CMAKE_TOOLCHAIN_FILE 1
[[ $(realpath -e "$OPENARM_CACHE_VALUE") == $(realpath -e "$toolchain_a") ]]
CC="$wrapper_b" CXX=/usr/bin/c++ CMAKE_TOOLCHAIN_FILE="$toolchain_b" \
  CFLAGS=-DTOOLCHAIN_B=1 CXXFLAGS=-DREQUEST_CXX=2 LDFLAGS=-Wl,--no-undefined \
  run_transaction
[[ "$LAST_REUSED" == 0 ]]
openarm_build_state_cache_get "$build_root/probe/CMakeCache.txt" CMAKE_C_FLAGS 1
[[ "$OPENARM_CACHE_VALUE" == -DTOOLCHAIN_B=1 ]]
openarm_build_state_cache_get "$build_root/probe/CMakeCache.txt" CMAKE_CXX_FLAGS 1
[[ "$OPENARM_CACHE_VALUE" == -DREQUEST_CXX=2 ]]

# In-place wrapper mutation, inert cache tamper, and interrupted pending state
# all invalidate reuse. A failed transaction never publishes completion.
printf '#!/usr/bin/env bash\nexec /usr/bin/cc -DIN_PLACE_CHANGED=1 "$@"\n' > "$wrapper_b"
touch "$build_root/probe/in-place-marker"
CC="$wrapper_b" CXX=/usr/bin/c++ CMAKE_TOOLCHAIN_FILE="$toolchain_b" \
  CFLAGS=-DTOOLCHAIN_B=1 CXXFLAGS=-DREQUEST_CXX=2 LDFLAGS=-Wl,--no-undefined \
  run_transaction
[[ "$LAST_REUSED" == 0 && ! -e "$build_root/probe/in-place-marker" ]]
printf 'CMAKE_C_FLAGS:STRING=-DTAMPERED\n' >> "$build_root/probe/CMakeCache.txt"
touch "$build_root/probe/tamper-marker"
CC="$wrapper_b" CXX=/usr/bin/c++ CMAKE_TOOLCHAIN_FILE="$toolchain_b" \
  CFLAGS=-DTOOLCHAIN_B=1 CXXFLAGS=-DREQUEST_CXX=2 LDFLAGS=-Wl,--no-undefined \
  run_transaction
[[ "$LAST_REUSED" == 0 && ! -e "$build_root/probe/tamper-marker" ]]

request=$(CC="$wrapper_b" CXX=/usr/bin/c++ CMAKE_TOOLCHAIN_FILE="$toolchain_b" \
  CFLAGS=-DTOOLCHAIN_B=1 CXXFLAGS=-DREQUEST_CXX=2 LDFLAGS=-Wl,--no-undefined \
  openarm_build_state_requested_digest probe Release 1 0 "$install_root")
openarm_build_state_write "$build_root" pending "$request"
touch "$build_root/probe/interrupted-marker"
CC="$wrapper_b" CXX=/usr/bin/c++ CMAKE_TOOLCHAIN_FILE="$toolchain_b" \
  CFLAGS=-DTOOLCHAIN_B=1 CXXFLAGS=-DREQUEST_CXX=2 LDFLAGS=-Wl,--no-undefined \
  run_transaction
[[ "$LAST_REUSED" == 0 && ! -e "$build_root/probe/interrupted-marker" ]]

set +e
OPENARM_CACHE_TEST_FAIL=1 CC="$wrapper_b" CXX=/usr/bin/c++ \
  CMAKE_TOOLCHAIN_FILE="$toolchain_a" CFLAGS=-DFAIL=1 \
  CXXFLAGS=-DFAIL=1 LDFLAGS=-Wl,--as-needed run_transaction
failure_status=$?
set -e
[[ "$failure_status" == 9 ]]
grep -Fxq 'phase pending' "$build_root/$OPENARM_BUILD_STATE_FILE"
! grep -q '^actual ' "$build_root/$OPENARM_BUILD_STATE_FILE"

printf '%s\n' 'Build cache-state transaction regression passed'
