#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
work_root=$(mktemp -d "${TMPDIR:-/tmp}/openarmik-build-controls.XXXXXX")
cleanup() { rm -rf -- "$work_root"; }
trap cleanup EXIT

runtime_dir="$work_root/runtime"
lock_dir="$runtime_dir/openarmik-build-locks-$UID"
mkdir -p -m 700 "$runtime_dir"

lock_file_for() {
  local resource=$1 digest
  digest=$(printf '%s\0' "$resource" | sha256sum | awk '{print $1}')
  printf '%s/%s.lock\n' "$lock_dir" "$digest"
}

wait_for_file() {
  local file=$1 attempt
  for attempt in {1..250}; do
    [[ -e "$file" ]] && return 0
    sleep 0.02
  done
  printf 'Timed out waiting for %s\n' "$file" >&2
  return 1
}

# Real tools cover one generated, compiled, one-job target and a real CTest.
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

# The mutation callback and a real grandchild must share one private process
# group, and neither may inherit either lock descriptor.
source "$root_dir/scripts/build_lock.sh"

# Lock directories are created atomically and accepted only after owner, type,
# symlink, and exact-mode validation. Every fixture remains below work_root.
security_root="$work_root/lock-security"
mkdir -m 700 "$security_root"
secure_base="$security_root/secure-base"
mkdir -m 700 "$secure_base"
secure_candidate=$(openarm_prepare_lock_dir "$secure_base" "$EUID")
[[ "$secure_candidate" == "$secure_base/openarmik-build-locks-$UID" ]]
[[ $(stat -c '%a' "$secure_candidate") == 700 ]]
[[ $(openarm_prepare_lock_dir "$secure_base" "$EUID") == "$secure_candidate" ]]

# The same object is attacker-owned relative to a different expected euid.
set +e
openarm_prepare_lock_dir "$secure_base" "$((EUID + 1))" \
  > "$work_root/attacker-owned.out" 2>&1
attacker_owned_status=$?
set -e
[[ "$attacker_owned_status" == 2 ]]

world_base="$security_root/world-base"
mkdir -m 700 "$world_base"
mkdir -m 777 "$world_base/openarmik-build-locks-$UID"
set +e
openarm_prepare_lock_dir "$world_base" "$EUID" \
  > "$work_root/world-writable.out" 2>&1
world_status=$?
set -e
[[ "$world_status" == 2 ]]

# A permissive existing directory is rejected; no chmod repair can be masked.
masked_base="$security_root/masked-base"
mkdir -m 700 "$masked_base"
mkdir -m 755 "$masked_base/openarmik-build-locks-$UID"
masked_bin="$security_root/masked-bin"
mkdir "$masked_bin"
cat > "$masked_bin/chmod" <<'EOF'
#!/usr/bin/env bash
touch "$OPENARM_MASKED_CHMOD_MARKER"
exit 0
EOF
chmod +x "$masked_bin/chmod"
set +e
PATH="$masked_bin:$PATH" OPENARM_MASKED_CHMOD_MARKER="$work_root/chmod-called" \
  openarm_prepare_lock_dir "$masked_base" "$EUID" \
  > "$work_root/masked-mode.out" 2>&1
masked_status=$?
set -e
[[ "$masked_status" == 2 && ! -e "$work_root/chmod-called" ]]

# Deterministically replace the child with a symlink at mkdir time. Validation
# must reject it before any lock-file open can touch the victim directory.
race_base="$security_root/race-base"
victim_dir="$security_root/victim"
race_bin="$security_root/race-bin"
mkdir -m 700 "$race_base" "$victim_dir" "$race_bin"
printf '%s\n' 'do-not-truncate' > "$victim_dir/sentinel"
cat > "$race_bin/mkdir" <<'EOF'
#!/usr/bin/env bash
candidate=${!#}
ln -s -- "$OPENARM_RACE_TARGET" "$candidate"
exit 1
EOF
chmod +x "$race_bin/mkdir"
set +e
PATH="$race_bin:$PATH" OPENARM_RACE_TARGET="$victim_dir" \
  openarm_prepare_lock_dir "$race_base" "$EUID" \
  > "$work_root/symlink-race.out" 2>&1
race_status=$?
set -e
[[ "$race_status" == 2 ]]
[[ "$(<"$victim_dir/sentinel")" == do-not-truncate ]]
[[ $(find "$victim_dir" -mindepth 1 -maxdepth 1 | wc -l) == 1 ]]
race_mutation="$work_root/race-mutation"
race_callback() { touch "$race_mutation"; }
set +e
XDG_RUNTIME_DIR="$race_base" \
  openarm_run_with_locks "$work_root/race-resource" -- race_callback \
  > "$work_root/race-public.out" 2>&1
race_public_status=$?
set -e
[[ "$race_public_status" == 2 && ! -e "$race_mutation" ]]
[[ "$(<"$victim_dir/sentinel")" == do-not-truncate ]]
[[ $(find "$victim_dir" -mindepth 1 -maxdepth 1 | wc -l) == 1 ]]

insecure_xdg="$security_root/insecure-xdg"
mkdir -m 777 "$insecure_xdg"
[[ $(XDG_RUNTIME_DIR="$insecure_xdg" openarm_choose_lock_base) == /tmp ]]
symlink_xdg="$security_root/symlink-xdg"
ln -s "$secure_base" "$symlink_xdg"
[[ $(XDG_RUNTIME_DIR="$symlink_xdg" openarm_choose_lock_base) == /tmp ]]
[[ $(XDG_RUNTIME_DIR="$secure_base" openarm_choose_lock_base) == "$secure_base" ]]

fd_resource_a=$(realpath -m -- "$work_root/fd-resource-a")
fd_resource_b=$(realpath -m -- "$work_root/fd-resource-b")
fd_lock_a=$(lock_file_for "$fd_resource_a")
fd_lock_b=$(lock_file_for "$fd_resource_b")
reentrant_driver="$work_root/reentrant-driver.sh"
cat > "$reentrant_driver" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
source "$1"
marker=$2
resource=$3
mutate() { touch "$marker"; }
openarm_run_with_locks "$resource" -- mutate
EOF
chmod +x "$reentrant_driver"

assert_no_build_lock_fds() {
  local fd target process_pid=$BASHPID
  for fd in /proc/"$process_pid"/fd/*; do
    target=$(readlink -f "$fd" 2>/dev/null || true)
    [[ "$target" != "$fd_lock_a" && "$target" != "$fd_lock_b" ]] || {
      printf 'Build lock leaked into callback: %s\n' "$target" >&2
      return 1
    }
  done
}

fd_callback() {
  local child status
  assert_no_build_lock_fds
  printf '%s %s\n' "$BASHPID" \
    "$(ps -o pgid= -p "$BASHPID" | tr -d ' ')" > "$work_root/fd-pgids"
  (
    assert_no_build_lock_fds
    printf '%s %s\n' "$BASHPID" \
      "$(ps -o pgid= -p "$BASHPID" | tr -d ' ')" >> "$work_root/fd-pgids"
  )
  set +e
  XDG_RUNTIME_DIR="$runtime_dir" "$reentrant_driver" \
    "$root_dir/scripts/build_lock.sh" "$work_root/reentrant-mutation" \
    "$fd_resource_a" > "$work_root/reentrant.out" 2>&1
  status=$?
  set -e
  printf '%s\n' "$status" > "$work_root/reentrant.status"
  touch "$work_root/fd-ready"
  while [[ ! -e "$work_root/fd-release" ]]; do sleep 0.02; done
}

XDG_RUNTIME_DIR="$runtime_dir" \
  openarm_run_with_locks "$fd_resource_a" "$fd_resource_b" -- fd_callback &
fd_supervisor=$!
wait_for_file "$work_root/fd-ready"
read -r callback_pid callback_pgid < "$work_root/fd-pgids"
read -r grandchild_pid grandchild_pgid < <(sed -n '2p' "$work_root/fd-pgids")
[[ "$callback_pid" == "$callback_pgid" ]]
[[ "$grandchild_pgid" == "$callback_pgid" ]]
[[ "$(<"$work_root/reentrant.status")" == 3 ]]
[[ ! -e "$work_root/reentrant-mutation" ]]
grep -Fq 'already being built' "$work_root/reentrant.out"
touch "$work_root/fd-release"
wait "$fd_supervisor"

post_marker="$work_root/post-release-mutation"
post_release() { touch "$post_marker"; }
XDG_RUNTIME_DIR="$runtime_dir" \
  openarm_run_with_locks "$fd_resource_a" -- post_release
[[ -e "$post_marker" ]]

saved_hup=$(trap -p HUP)
saved_monitor=0
[[ $- == *m* ]] && saved_monitor=1
trap ':' HUP
custom_hup=$(trap -p HUP)
exact_failure() { return 19; }
set +e
XDG_RUNTIME_DIR="$runtime_dir" \
  openarm_run_with_locks "$work_root/exact-status" -- exact_failure
exact_status=$?
set -e
[[ "$exact_status" == 19 ]]
[[ "$(trap -p HUP)" == "$custom_hup" ]]
if ((saved_monitor)); then [[ $- == *m* ]]; else [[ $- != *m* ]]; fi
if [[ -n "$saved_hup" ]]; then eval "$saved_hup"; else trap - HUP; fi

# A signal sent only to the supervisor must be forwarded exactly, preserve the
# conventional status, empty the owned group, and release the lock immediately.
for signal_case in HUP:129 INT:130 TERM:143; do
  signal=${signal_case%%:*}
  expected_status=${signal_case#*:}
  signal_resource=$(realpath -m -- "$work_root/signal-$signal")
  signal_callback() {
    local callback_signal=$1
    printf '%s %s\n' "$BASHPID" \
      "$(ps -o pgid= -p "$BASHPID" | tr -d ' ')" \
      > "$work_root/$callback_signal.pids"
    (sleep 30) &
    printf '%s %s\n' "$!" \
      "$(ps -o pgid= -p "$!" | tr -d ' ')" \
      >> "$work_root/$callback_signal.pids"
    touch "$work_root/$callback_signal.ready"
    wait
  }
  XDG_RUNTIME_DIR="$runtime_dir" \
    openarm_run_with_locks "$signal_resource" -- signal_callback "$signal" &
  signal_supervisor=$!
  wait_for_file "$work_root/$signal.ready"
  kill -s "$signal" "$signal_supervisor"
  set +e
  wait "$signal_supervisor"
  signal_status=$?
  set -e
  [[ "$signal_status" == "$expected_status" ]]
  while read -r member member_pgid; do
    ! kill -0 "$member" 2>/dev/null
    ! kill -0 -- "-$member_pgid" 2>/dev/null
  done < "$work_root/$signal.pids"
  signal_reacquired="$work_root/$signal.reacquired"
  reacquire() { touch "$signal_reacquired"; }
  XDG_RUNTIME_DIR="$runtime_dir" \
    openarm_run_with_locks "$signal_resource" -- reacquire
  [[ -e "$signal_reacquired" ]]
done

# A callback that deliberately ignores TERM is escalated after the bounded
# grace period, still only against its owned process group.
stubborn_resource=$(realpath -m -- "$work_root/signal-stubborn")
stubborn_callback() {
  trap '' TERM
  (trap '' TERM; while :; do sleep 1; done) &
  printf '%s %s\n' "$BASHPID" \
    "$(ps -o pgid= -p "$BASHPID" | tr -d ' ')" > "$work_root/stubborn.pids"
  printf '%s %s\n' "$!" \
    "$(ps -o pgid= -p "$!" | tr -d ' ')" >> "$work_root/stubborn.pids"
  touch "$work_root/stubborn.ready"
  while :; do sleep 1; done
}
XDG_RUNTIME_DIR="$runtime_dir" \
  openarm_run_with_locks "$stubborn_resource" -- stubborn_callback &
stubborn_supervisor=$!
wait_for_file "$work_root/stubborn.ready"
kill -TERM "$stubborn_supervisor"
set +e
wait "$stubborn_supervisor"
stubborn_status=$?
set -e
[[ "$stubborn_status" == 143 ]]
while read -r member member_pgid; do
  ! kill -0 "$member" 2>/dev/null
  ! kill -0 -- "-$member_pgid" 2>/dev/null
done < "$work_root/stubborn.pids"

# Production command construction is exercised only in a copied miniature
# repository with pinned description identity and narrow argument shims.
mini_repo="$work_root/mini-repo"
mkdir -p "$mini_repo/scripts/lib" "$mini_repo/upstream/openarm_description" \
  "$mini_repo/ros2_ws/src"
cp "$root_dir/scripts/build.sh" "$mini_repo/scripts/build.sh"
cp "$root_dir/scripts/build_native.sh" "$mini_repo/scripts/build_native.sh"
cp "$root_dir/scripts/build_lock.sh" "$mini_repo/scripts/build_lock.sh"
cp "$root_dir/scripts/lib/build_native_body.sh" \
  "$mini_repo/scripts/lib/build_native_body.sh"
touch "$mini_repo/upstream/openarm_description/package.xml"
for component in can model commission transport control runtime; do
  mkdir -p "$mini_repo/$component"
done

fake_bin="$work_root/bin"
command_log="$work_root/commands.log"
mkdir -p "$fake_bin"
: > "$command_log"
cat > "$fake_bin/cmake" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf 'cmake' >> "$OPENARM_BUILD_TEST_LOG"
printf ' <%s>' "$@" >> "$OPENARM_BUILD_TEST_LOG"
printf '\n' >> "$OPENARM_BUILD_TEST_LOG"
if [[ -n ${OPENARM_BUILD_TEST_GATE:-} ]] &&
   mkdir "$OPENARM_BUILD_TEST_GATE.claim" 2>/dev/null; then
  touch "$OPENARM_BUILD_TEST_GATE.ready"
  while [[ ! -e "$OPENARM_BUILD_TEST_GATE.release" ]]; do sleep 0.02; done
fi
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
      printf '%s:STRING=%s\n' "$key" "${definitions[$key]}" \
        >> "$build_dir/CMakeCache.txt"
    done
    ;;
  --build) ;;
  --install)
    prefix=$(awk -F= '/^CMAKE_INSTALL_PREFIX:/ {print $2; exit}' \
      "$2/CMakeCache.txt")
    mkdir -p "$prefix/lib"
    touch "$prefix/lib/libopenarm_control.a" \
      "$prefix/lib/libopenarm_commission.a" \
      "$prefix/lib/libopenarm_transport.a" \
      "$prefix/lib/libopenarm_runtime.a"
    ;;
  *) exit 1 ;;
esac
EOF
cat > "$fake_bin/colcon" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf 'colcon MAKEFLAGS=%s CMAKE_BUILD_PARALLEL_LEVEL=%s' \
  "${MAKEFLAGS:-}" "${CMAKE_BUILD_PARALLEL_LEVEL:-}" >> "$OPENARM_BUILD_TEST_LOG"
printf ' <%s>' "$@" >> "$OPENARM_BUILD_TEST_LOG"; printf '\n' >> "$OPENARM_BUILD_TEST_LOG"
build_base=
while (($#)); do
  case "$1" in --build-base) build_base=$2; shift 2; continue ;; esac
  shift
done
mkdir -p "$build_base/openarm_ik_ros"
: > "$build_base/openarm_ik_ros/libopenarm_virtual_control_session.a"
EOF
cat > "$fake_bin/nm" <<'EOF'
#!/usr/bin/env bash
if [[ "$1" == -u && "$2" == *libopenarm_virtual_control_session.a ]]; then
  printf '                 U oa_runtime_create\n'
fi
EOF
chmod +x "$fake_bin/cmake" "$fake_bin/colcon" "$fake_bin/nm"

decoy_description="$work_root/decoy-description"
mkdir -p "$decoy_description"
touch "$decoy_description/package.xml"
run_top() {
  PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
    OPENARM_DESCRIPTION_DIR="$decoy_description" XDG_RUNTIME_DIR="$runtime_dir" \
    OPENARM_BUILD_JOBS=2 "$mini_repo/scripts/build.sh" "$@"
}
run_native() {
  PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
    OPENARM_DESCRIPTION_DIR="$decoy_description" XDG_RUNTIME_DIR="$runtime_dir" \
    OPENARM_BUILD_JOBS=2 "$mini_repo/scripts/build_native.sh" "$@"
}

output_root="$work_root/output"
mkdir -p "$output_root/native_build"
touch "$output_root/native_build/non-destructive-sentinel"
if OPENARM_BUILD_JOBS=0 "$mini_repo/scripts/build.sh" \
  --output-root "$output_root" >/dev/null 2>&1; then exit 1; fi
[[ -e "$output_root/native_build/non-destructive-sentinel" ]]
if "$mini_repo/scripts/build_native.sh" --locked-body x >/dev/null 2>&1; then exit 1; fi
if "$mini_repo/scripts/build.sh" --locked-body x >/dev/null 2>&1; then exit 1; fi
if "$mini_repo/scripts/build_native.sh" --jobs 1x >/dev/null 2>&1; then exit 1; fi
if "$mini_repo/scripts/build.sh" --jobs '' >/dev/null 2>&1; then exit 1; fi

: > "$command_log"
run_top --output-root "$output_root" --jobs 2
[[ $(grep -c '^cmake <-S>' "$command_log") == 6 ]]
[[ $(grep -c '^colcon ' "$command_log") == 1 ]]
grep -Fq '<--parallel> <2>' "$command_log"
grep -Fq 'colcon MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2' "$command_log"
grep -Fq '<--executor> <sequential>' "$command_log"
grep -Fq "<-DOA_DESCRIPTION_ROOT=$mini_repo/upstream/openarm_description>" \
  "$command_log"
grep -Fq "<--base-paths> <$mini_repo/upstream/openarm_description>" \
  "$command_log"
! grep -Fq "$decoy_description" "$command_log"

touch "$output_root/native_build/can/incremental-marker"
: > "$command_log"
run_top --incremental --output-root "$output_root" --jobs 2
[[ -e "$output_root/native_build/can/incremental-marker" ]]
! grep -Fq '<-E> <remove_directory>' "$command_log"
! grep -Fq '<--cmake-clean-cache>' "$command_log"

# A gated top-level body owns the output, native, and install resources while
# directly composing exactly one source-only native sequence.
gate="$work_root/top-gate"
: > "$command_log"
OPENARM_BUILD_TEST_GATE="$gate" run_top --output-root "$output_root" --jobs 1 \
  > "$work_root/gated-top.out" 2>&1 &
top_supervisor=$!
wait_for_file "$gate.ready"
set +e
run_native --build-root "$output_root/native_build" \
  --install-prefix "$work_root/other-install" --jobs 1 \
  > "$work_root/top-native-root.out" 2>&1
root_status=$?
run_native --build-root "$work_root/other-native" \
  --install-prefix "$output_root/install" --jobs 1 \
  > "$work_root/top-install.out" 2>&1
install_status=$?
set -e
[[ "$root_status" == 3 && "$install_status" == 3 ]]
touch "$gate.release"
wait "$top_supervisor"
[[ $(grep -c '^cmake <-S>' "$command_log") == 6 ]]
[[ $(grep -c '^colcon ' "$command_log") == 1 ]]

# Canonical aliases and shared prefixes contend; independent sibling resources
# proceed. Legacy ambient variables have no effect on ownership.
build_a=$(realpath -m -- "$work_root/a/build")
build_b=$(realpath -m -- "$work_root/b/build")
shared_prefix=$(realpath -m -- "$work_root/shared-install")
mkdir -p "$(dirname "$build_a")" "$(dirname "$build_b")" "$lock_dir"
flock "$(lock_file_for "$build_a")" sleep 30 & holder=$!
sleep 0.05
set +e
OPENARM_BUILD_LOCK_HELD=1 OPENARM_NATIVE_BUILD_LOCK_HELD=1 \
  run_native --build-root "$work_root/a/../a/build" \
  --install-prefix "$work_root/a/install" --jobs 1 \
  > "$work_root/native-alias.out" 2>&1
alias_status=$?
set -e
kill "$holder"; wait "$holder" 2>/dev/null || true
[[ "$alias_status" == 3 ]]

flock "$(lock_file_for "$shared_prefix")" sleep 30 & holder=$!
sleep 0.05
set +e
run_native --build-root "$build_b" --install-prefix "$shared_prefix" --jobs 1 \
  > "$work_root/shared-prefix.out" 2>&1
prefix_status=$?
set -e
kill "$holder"; wait "$holder" 2>/dev/null || true
[[ "$prefix_status" == 3 ]]
run_native --build-root "$build_b" --install-prefix "$work_root/b/install" --jobs 1

# A valid decoy cannot replace a missing pinned checkout, and validation occurs
# before a shim observes any mutation command.
missing_repo="$work_root/missing-repo"
mkdir -p "$missing_repo/scripts/lib"
cp "$mini_repo/scripts/build.sh" "$missing_repo/scripts/build.sh"
cp "$mini_repo/scripts/build_native.sh" "$missing_repo/scripts/build_native.sh"
cp "$mini_repo/scripts/build_lock.sh" "$missing_repo/scripts/build_lock.sh"
cp "$mini_repo/scripts/lib/build_native_body.sh" \
  "$missing_repo/scripts/lib/build_native_body.sh"
: > "$command_log"
set +e
PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
  OPENARM_DESCRIPTION_DIR="$decoy_description" \
  "$missing_repo/scripts/build_native.sh" --build-root "$work_root/missing-native" \
  --install-prefix "$work_root/missing-install" --jobs 1 >/dev/null 2>&1
missing_native_status=$?
PATH="$fake_bin:$PATH" OPENARM_BUILD_TEST_LOG="$command_log" \
  OPENARM_DESCRIPTION_DIR="$decoy_description" \
  "$missing_repo/scripts/build.sh" --output-root "$work_root/missing-output" \
  --jobs 1 >/dev/null 2>&1
missing_top_status=$?
set -e
[[ "$missing_native_status" == 1 && "$missing_top_status" == 1 ]]
[[ ! -s "$command_log" ]]
! rg -n 'OPENARM_DESCRIPTION_DIR|OPENARM_BUILD_LOCK_FDS|LOCK_HELD|locked-body' \
  "$root_dir/scripts"

# Browser is intentionally unmanaged and closes GUI descriptor 9.
grep -Fq '"$browser_command" "$url" 9>&-' \
  "$root_dir/scripts/launch_web_portal.sh"
exec 9>"$work_root/browser-lock"
bash -c '[[ ! -e /proc/self/fd/9 ]]' 9>&-
exec 9>&-
grep -Fq 'lock_file="$runtime_dir/openarmik-gui-$UID.lock"' \
  "$root_dir/scripts/launch_web_portal.sh"
grep -Fq 'lock_file="$runtime_dir/openarmik-gui-$UID.lock"' \
  "$root_dir/scripts/launch_rviz.sh"

printf '%s\n' \
  'Build resource-control regression passed (supervisor, pinned fixture, real CMake/CTest)'
