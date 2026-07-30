#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
work_root=$(mktemp -d "${TMPDIR:-/tmp}/openarmik-build-controls.XXXXXX")
description_dir="$root_dir/upstream/openarm_description"
created_description=0
if [[ ! -f "$description_dir/package.xml" ]]; then
  mkdir -p "$description_dir"
  : > "$description_dir/package.xml"
  created_description=1
fi
cleanup() {
  rm -rf -- "$work_root"
  if ((created_description)); then
    rm -rf -- "$description_dir"
    rmdir "$root_dir/upstream" 2>/dev/null || true
  fi
}
trap cleanup EXIT
output_root="$work_root/output"
fake_bin="$work_root/bin"
command_log="$work_root/commands.log"
mkdir -p "$fake_bin"

cat > "$fake_bin/cmake" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf 'cmake' >> "$OPENARM_BUILD_TEST_LOG"
printf ' <%s>' "$@" >> "$OPENARM_BUILD_TEST_LOG"
printf '\n' >> "$OPENARM_BUILD_TEST_LOG"
case "$1" in
  -E)
    [[ "$2" == remove_directory ]]
    rm -rf -- "$3"
    ;;
  -S)
    build_dir=
    declare -A definitions=()
    while (($#)); do
      case "$1" in
        -B) build_dir=$2; shift 2; continue ;;
        -D*) definition=${1#-D}; definitions[${definition%%=*}]=${definition#*=} ;;
      esac
      shift
    done
    mkdir -p "$build_dir"
    : > "$build_dir/CMakeCache.txt"
    for key in "${!definitions[@]}"; do
      printf '%s:STRING=%s\n' "$key" "${definitions[$key]}" >> "$build_dir/CMakeCache.txt"
    done
    ;;
  --build)
    ;;
  --install)
    build_dir=$2
    prefix=$(awk -F= '/^CMAKE_INSTALL_PREFIX:/ {print $2; exit}' "$build_dir/CMakeCache.txt")
    mkdir -p "$prefix/lib"
    touch "$prefix/lib/libopenarm_control.a" "$prefix/lib/libopenarm_commission.a" \
      "$prefix/lib/libopenarm_transport.a" "$prefix/lib/libopenarm_runtime.a"
    ;;
  *)
    printf 'Unexpected fake cmake invocation: %s\n' "$*" >&2
    exit 1
    ;;
esac
EOF
cat > "$fake_bin/colcon" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf 'colcon MAKEFLAGS=%s CMAKE_BUILD_PARALLEL_LEVEL=%s' "${MAKEFLAGS:-}" "${CMAKE_BUILD_PARALLEL_LEVEL:-}" >> "$OPENARM_BUILD_TEST_LOG"
printf ' <%s>' "$@" >> "$OPENARM_BUILD_TEST_LOG"
printf '\n' >> "$OPENARM_BUILD_TEST_LOG"
build_base=
while (($#)); do
  case "$1" in
    --build-base) build_base=$2; shift 2; continue ;;
  esac
  shift
done
mkdir -p "$build_base/openarm_ik_ros"
: > "$build_base/openarm_ik_ros/libopenarm_virtual_control_session.a"
EOF
cat > "$fake_bin/nm" <<'EOF'
#!/usr/bin/env bash
if [[ "$1" == -u ]]; then
  printf '                 U oa_runtime_create\n'
fi
exit 0
EOF
chmod +x "$fake_bin/cmake" "$fake_bin/colcon" "$fake_bin/nm"

run_build() {
  PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
    OPENARM_BUILD_JOBS=2 "$root_dir/scripts/build.sh" --output-root "$output_root" "$@"
}

mkdir -p "$output_root/native_build"
touch "$output_root/native_build/invalid-jobs-marker"
if OPENARM_BUILD_JOBS=0 "$root_dir/scripts/build.sh" --output-root "$output_root" >/dev/null 2>&1; then
  printf '%s\n' 'zero jobs unexpectedly succeeded' >&2
  exit 1
fi
[[ -e "$output_root/native_build/invalid-jobs-marker" ]]
if "$root_dir/scripts/build_native.sh" --jobs 1x >/dev/null 2>&1; then
  printf '%s\n' 'invalid native job count unexpectedly succeeded' >&2
  exit 1
fi
if "$root_dir/scripts/build.sh" --jobs '' >/dev/null 2>&1; then
  printf '%s\n' 'empty explicit job count unexpectedly succeeded' >&2
  exit 1
fi

: > "$command_log"
run_build --jobs 2
grep -Fq 'cmake <--build> ' "$command_log"
grep -Fq '<--parallel> <2>' "$command_log"
grep -Fq 'colcon MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2' "$command_log"
grep -Fq '<--executor> <sequential>' "$command_log"
grep -Fq '<--cmake-clean-cache>' "$command_log"

touch "$output_root/native_build/can/incremental-marker"
: > "$command_log"
run_build --incremental --jobs 2
[[ -e "$output_root/native_build/can/incremental-marker" ]]
if grep -Fq '<-E> <remove_directory>' "$command_log"; then
  printf '%s\n' 'incremental native build removed a component tree' >&2
  exit 1
fi
if grep -Fq '<--cmake-clean-cache>' "$command_log"; then
  printf '%s\n' 'incremental ROS build cleaned its CMake cache' >&2
  exit 1
fi

flock "$output_root/.openarmik-build.lock" sleep 10 &
lock_holder=$!
sleep 0.1
ln -s "$output_root" "$work_root/output-alias"
set +e
PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
  OPENARM_BUILD_JOBS=2 "$root_dir/scripts/build.sh" \
  --output-root "$work_root/output-alias" \
  >"$work_root/lock.out" 2>&1
lock_status=$?
set -e
kill "$lock_holder" 2>/dev/null || true
wait "$lock_holder" 2>/dev/null || true
[[ "$lock_status" == 3 ]]
grep -Fq 'already being built' "$work_root/lock.out"

flock "$output_root/.openarmik-build.lock" sleep 10 &
native_lock_holder=$!
sleep 0.1
set +e
PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
  OPENARM_BUILD_JOBS=2 "$root_dir/scripts/build_native.sh" \
  --build-root "$output_root/native_build" \
  --install-prefix "$output_root/install" \
  --jobs 2 >"$work_root/native-lock.out" 2>&1
native_lock_status=$?
set -e
kill "$native_lock_holder" 2>/dev/null || true
wait "$native_lock_holder" 2>/dev/null || true
[[ "$native_lock_status" == 3 ]]
grep -Fq 'Native build root is already being built' "$work_root/native-lock.out"

grep -Fq 'exec "$root_dir/scripts/launch_web_portal.sh" --build --firefox "$@"' \
  "$root_dir/run.sh"
grep -Fq 'lock_file="$runtime_dir/openarmik-gui-$UID.lock"' \
  "$root_dir/scripts/launch_web_portal.sh"
grep -Fq 'lock_file="$runtime_dir/openarmik-gui-$UID.lock"' \
  "$root_dir/scripts/launch_rviz.sh"

runtime_dir="$work_root/runtime"
mkdir -m 700 "$runtime_dir"
gui_lock="$runtime_dir/openarmik-gui-$UID.lock"
flock "$gui_lock" sleep 10 &
gui_lock_holder=$!
sleep 0.1
set +e
XDG_RUNTIME_DIR="$runtime_dir" "$root_dir/scripts/launch_rviz.sh" \
  >"$work_root/rviz-lock.out" 2>&1
rviz_lock_status=$?
DISPLAY=:99 XDG_RUNTIME_DIR="$runtime_dir" \
  "$root_dir/scripts/launch_web_portal.sh" --no-browser \
  >"$work_root/portal-lock.out" 2>&1
portal_lock_status=$?
set -e
kill "$gui_lock_holder" 2>/dev/null || true
wait "$gui_lock_holder" 2>/dev/null || true
[[ "$rviz_lock_status" == 3 ]]
[[ "$portal_lock_status" == 3 ]]
grep -Fq 'An OpenArm GUI is already running' "$work_root/rviz-lock.out"
grep -Fq 'An OpenArm GUI is already running' "$work_root/portal-lock.out"
printf '%s\n' 'Build resource-control regression passed'
