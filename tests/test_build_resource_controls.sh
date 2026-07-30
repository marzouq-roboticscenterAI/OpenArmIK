#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
work_root=$(mktemp -d "${TMPDIR:-/tmp}/openarmik-build-controls.XXXXXX")
cleanup() { rm -rf -- "$work_root"; }
trap cleanup EXIT

runtime_dir="$work_root/runtime"
lock_dir="$runtime_dir/openarmik-build-locks-$UID"
fixture_description="$work_root/openarm_description"
fake_bin="$work_root/bin"
command_log="$work_root/commands.log"
mkdir -p -m 700 "$runtime_dir" "$fixture_description" "$fake_bin"
: > "$fixture_description/package.xml"
# A partial pre-existing upstream checkout must never be populated or removed by
# this test; all fixtures live below the one disposable work root.
preexisting_upstream="$work_root/preexisting-upstream/openarm_description"
mkdir -p "$preexisting_upstream"
touch "$preexisting_upstream/package.xml-missing-sentinel"

lock_file_for() {
  local resource=$1 digest
  digest=$(printf '%s\0' "$resource" | sha256sum | awk '{print $1}')
  printf '%s/%s.lock\n' "$lock_dir" "$digest"
}

# Real tools cover the generator, one bounded build, and CTest. Production
# command construction is tested separately with narrow shims below.
real_fixture="$work_root/real-cmake"
mkdir -p "$real_fixture"
cat > "$real_fixture/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(openarmik_lock_fixture C)
enable_testing()
add_executable(lock_fixture main.c)
add_test(NAME lock_fixture COMMAND lock_fixture)
EOF
cat > "$real_fixture/main.c" <<'EOF'
int main(void) { return 0; }
EOF
cmake -S "$real_fixture" -B "$real_fixture/build"
cmake --build "$real_fixture/build" --parallel 1
ctest --test-dir "$real_fixture/build" --output-on-failure

cat > "$fake_bin/cmake" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf 'cmake' >> "$OPENARM_BUILD_TEST_LOG"
printf ' <%s>' "$@" >> "$OPENARM_BUILD_TEST_LOG"
printf '\n' >> "$OPENARM_BUILD_TEST_LOG"
case "$1" in
  -E) [[ "$2" == remove_directory ]]; rm -rf -- "$3" ;;
  -S)
    build_dir=; declare -A definitions=()
    while (($#)); do
      case "$1" in
        -B) build_dir=$2; shift 2; continue ;;
        -D*) definition=${1#-D}; definitions[${definition%%=*}]=${definition#*=} ;;
      esac
      shift
    done
    mkdir -p "$build_dir"; : > "$build_dir/CMakeCache.txt"
    for key in "${!definitions[@]}"; do
      printf '%s:STRING=%s\n' "$key" "${definitions[$key]}" >> "$build_dir/CMakeCache.txt"
    done
    ;;
  --build) ;;
  --install)
    prefix=$(awk -F= '/^CMAKE_INSTALL_PREFIX:/ {print $2; exit}' "$2/CMakeCache.txt")
    mkdir -p "$prefix/lib"
    touch "$prefix/lib/libopenarm_control.a" "$prefix/lib/libopenarm_commission.a" \
      "$prefix/lib/libopenarm_transport.a" "$prefix/lib/libopenarm_runtime.a"
    ;;
  *) exit 1 ;;
esac
EOF
cat > "$fake_bin/colcon" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf 'colcon MAKEFLAGS=%s CMAKE_BUILD_PARALLEL_LEVEL=%s' "${MAKEFLAGS:-}" "${CMAKE_BUILD_PARALLEL_LEVEL:-}" >> "$OPENARM_BUILD_TEST_LOG"
printf ' <%s>' "$@" >> "$OPENARM_BUILD_TEST_LOG"; printf '\n' >> "$OPENARM_BUILD_TEST_LOG"
build_base=
while (($#)); do case "$1" in --build-base) build_base=$2; shift 2; continue;; esac; shift; done
mkdir -p "$build_base/openarm_ik_ros"; : > "$build_base/openarm_ik_ros/libopenarm_virtual_control_session.a"
EOF
cat > "$fake_bin/nm" <<'EOF'
#!/usr/bin/env bash
[[ "$1" != -u ]] || printf '                 U oa_runtime_create\n'
EOF
chmod +x "$fake_bin/cmake" "$fake_bin/colcon" "$fake_bin/nm"

run_top() {
  PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
    OPENARM_DESCRIPTION_DIR="$fixture_description" XDG_RUNTIME_DIR="$runtime_dir" \
    OPENARM_BUILD_JOBS=2 "$root_dir/scripts/build.sh" "$@"
}
run_native() {
  PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
    OPENARM_DESCRIPTION_DIR="$fixture_description" XDG_RUNTIME_DIR="$runtime_dir" \
    OPENARM_BUILD_JOBS=2 "$root_dir/scripts/build_native.sh" "$@"
}

output_root="$work_root/output"
mkdir -p "$output_root/native_build"; touch "$output_root/native_build/invalid-jobs-marker"
if OPENARM_BUILD_JOBS=0 OPENARM_DESCRIPTION_DIR="$fixture_description" \
  "$root_dir/scripts/build.sh" --output-root "$output_root" >/dev/null 2>&1; then exit 1; fi
[[ -e "$output_root/native_build/invalid-jobs-marker" ]]
if "$root_dir/scripts/build_native.sh" --jobs 1x >/dev/null 2>&1; then exit 1; fi
if "$root_dir/scripts/build.sh" --jobs '' >/dev/null 2>&1; then exit 1; fi

: > "$command_log"
run_top --output-root "$output_root" --jobs 2
grep -Fq '<--parallel> <2>' "$command_log"
grep -Fq 'colcon MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2' "$command_log"
grep -Fq '<--executor> <sequential>' "$command_log"
touch "$output_root/native_build/can/incremental-marker"
: > "$command_log"
run_top --incremental --output-root "$output_root" --jobs 2
[[ -e "$output_root/native_build/can/incremental-marker" ]]
! grep -Fq '<-E> <remove_directory>' "$command_log"
! grep -Fq '<--cmake-clean-cache>' "$command_log"

# Public sentinels are forged; real flock contention must still win.
mkdir -p "$lock_dir"
top_lock=$(lock_file_for "$output_root")
flock "$top_lock" sleep 10 & holder=$!; sleep 0.1
set +e
OPENARM_BUILD_LOCK_HELD=1 OPENARM_NATIVE_BUILD_LOCK_HELD=1 \
  PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
  OPENARM_DESCRIPTION_DIR="$fixture_description" XDG_RUNTIME_DIR="$runtime_dir" \
  "$root_dir/scripts/build.sh" --output-root "$output_root" >"$work_root/top-lock.out" 2>&1
top_status=$?
set -e
kill "$holder" 2>/dev/null || true; wait "$holder" 2>/dev/null || true
[[ "$top_status" == 3 ]]; grep -Fq 'already being built' "$work_root/top-lock.out"

build_a="$work_root/a/build"; build_b="$work_root/b/build"; shared_prefix="$work_root/shared-install"
mkdir -p "$(dirname "$build_a")" "$(dirname "$build_b")"
build_lock=$(lock_file_for "$build_a")
flock "$build_lock" sleep 10 & holder=$!; sleep 0.1
set +e
OPENARM_NATIVE_BUILD_LOCK_HELD=1 run_native --build-root "$work_root/a/../a/build" \
  --install-prefix "$work_root/a/install" --jobs 1 >"$work_root/native-same.out" 2>&1
same_status=$?
set -e
kill "$holder" 2>/dev/null || true; wait "$holder" 2>/dev/null || true
[[ "$same_status" == 3 ]]; grep -Fq 'Native build root is already being built' "$work_root/native-same.out"

prefix_lock=$(lock_file_for "$shared_prefix")
flock "$prefix_lock" sleep 10 & holder=$!; sleep 0.1
set +e
OPENARM_NATIVE_BUILD_LOCK_HELD=1 run_native --build-root "$build_b" --install-prefix "$shared_prefix" --jobs 1 >"$work_root/prefix-lock.out" 2>&1
prefix_status=$?
set -e
kill "$holder" 2>/dev/null || true; wait "$holder" 2>/dev/null || true
[[ "$prefix_status" == 3 ]]; grep -Fq 'Native build root is already being built' "$work_root/prefix-lock.out"

# Sibling roots and prefixes are independent; no held resource lock blocks B.
run_native --build-root "$build_b" --install-prefix "$work_root/b/install" --jobs 1

# Browser is deliberately unmanaged and must not inherit the GUI lock FD.
grep -Fq '"$browser_command" "$url" 9>&-' "$root_dir/scripts/launch_web_portal.sh"
exec 9>"$work_root/browser-lock"
bash -c '[[ ! -e /proc/self/fd/9 ]]' 9>&-
exec 9>&-
grep -Fq 'lock_file="$runtime_dir/openarmik-gui-$UID.lock"' \
  "$root_dir/scripts/launch_web_portal.sh"
grep -Fq 'lock_file="$runtime_dir/openarmik-gui-$UID.lock"' \
  "$root_dir/scripts/launch_rviz.sh"
[[ -e "$preexisting_upstream/package.xml-missing-sentinel" ]]

printf '%s\n' 'Build resource-control regression passed (real CMake/CTest plus argument and flock coverage)'
