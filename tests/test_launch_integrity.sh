#!/usr/bin/env bash
set -euo pipefail
root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root_dir/scripts/lib/description_pin.sh"
source "$root_dir/scripts/build_lock.sh"
source "$root_dir/scripts/lib/launch_integrity.sh"
work_root=$(mktemp -d "${TMPDIR:-/tmp}/openarm-launch-integrity.XXXXXX")
cleanup() { rm -rf -- "$work_root"; }
trap cleanup EXIT
export XDG_RUNTIME_DIR="$work_root"

fixture_root="$work_root/source"
description="$fixture_root/upstream/openarm_description"
output="$work_root/output"
mkdir -p "$fixture_root/scripts" "$description" "$output/build/openarm_ik_ros" \
  "$output/install/openarm_ik_ros/lib/openarm_ik_ros" "$output/install/lib" \
  "$output/install/openarm_ik_ros/share/openarm_ik_ros/launch" \
  "$output/install/openarm_ik_ros/share/openarm_ik_ros/rviz" \
  "$output/install/openarm_ik_ros/share/openarm_ik_ros/web" \
  "$output/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh"
mkdir -p "$fixture_root/can" "$fixture_root/model" "$fixture_root/commission" \
  "$fixture_root/transport" "$fixture_root/control" "$fixture_root/runtime" \
  "$fixture_root/ros2_ws/src" "$fixture_root/tests"
git -C "$fixture_root" init -q
git -C "$fixture_root" config user.name fixture
git -C "$fixture_root" config user.email fixture@example.invalid
printf 'tracked source\n' > "$fixture_root/scripts/input.sh"
printf 'upstream/\n*.ignored\n' > "$fixture_root/.gitignore"
git -C "$fixture_root" add .
git -C "$fixture_root" commit -qm source

git -C "$description" init -q
git -C "$description" config user.name fixture
git -C "$description" config user.email fixture@example.invalid
printf '<package/>\n' > "$description/package.xml"
git -C "$description" add package.xml
git -C "$description" commit -qm description
git -C "$description" remote add origin https://example.invalid/description.git
git -C "$description" checkout -q --detach
fixture_commit=$(git -C "$description" rev-parse HEAD)
fixture_tree=$(git -C "$description" rev-parse 'HEAD^{tree}')
openarm_validate_description_pin() {
  openarm_validate_description_repository "$1" "$fixture_commit" \
    https://example.invalid/description.git "$fixture_tree" 1 none
}

session_source="$work_root/session.c"
printf '%s\n' \
  'extern void oa_runtime_create(void);' \
  'extern void oa_runtime_get_capabilities(void);' \
  'void session(void) { oa_runtime_create(); oa_runtime_get_capabilities(); }' \
  > "$session_source"
cc -c "$session_source" -o "$work_root/session.o"
ar rcs "$output/build/openarm_ik_ros/libopenarm_virtual_control_session.a" \
  "$work_root/session.o"
printf 'void runtime_local(void) {}\n' | cc -x c -c - -o "$work_root/runtime.o"
ar rcs "$output/install/lib/libopenarm_runtime.a" "$work_root/runtime.o"
for executable in openarm_ik_ros_node openarm_portal close_rviz_window; do
  printf '#!/usr/bin/env bash\nexit 0\n' > "$output/install/openarm_ik_ros/lib/openarm_ik_ros/$executable"
  chmod +x "$output/install/openarm_ik_ros/lib/openarm_ik_ros/$executable"
done
printf '# setup\n' > "$output/install/setup.bash"
printf '# local setup\n' > "$output/install/local_setup.bash"
printf '# generated setup helper\n' > "$output/install/_local_setup_util_sh.py"
mkdir -p "$output/install/share/ament_index/resource_index/packages"
printf '%s\n' "$output/install/openarm_ik_ros" > \
  "$output/install/share/ament_index/resource_index/packages/openarm_ik_ros"
printf '# launch\n' > "$output/install/openarm_ik_ros/share/openarm_ik_ros/launch/openarm_ik_rviz.launch.py"
printf '# rviz\n' > "$output/install/openarm_ik_ros/share/openarm_ik_ros/rviz/openarm_ik.rviz"
for asset in portal.css portal.js viewer.js; do
  printf 'viewer asset %s\n' "$asset" > "$output/install/openarm_ik_ros/share/openarm_ik_ros/web/$asset"
done
printf '{"schema":1}\n' > "$output/install/openarm_ik_ros/share/openarm_ik_ros/viewer/manifest.json"
printf '<robot/>\n' > "$output/install/openarm_ik_ros/share/openarm_ik_ros/viewer/stage_a.urdf"
for mesh in body_link0_symp link0_symp link1_symp link2_symp link3_symp link4_symp link5_symp link6_symp link7_symp hand finger; do
  printf 'mesh %s\n' "$mesh" > "$output/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/$mesh.stl"
done

# A launch stamp requires completed, verifiable native and ROS CMake selections.
cache_source="$work_root/cache-source"
cache_template="$work_root/cache-template"
mkdir -p "$cache_source"
printf '%s\n' 'cmake_minimum_required(VERSION 3.16)' \
  'project(openarm_launch_cache_fixture LANGUAGES C CXX)' \
  'include(CTest)' \
  > "$cache_source/CMakeLists.txt"
cmake -S "$cache_source" -B "$cache_template" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$output/install" -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 >/dev/null
for component in can model commission transport control runtime; do
  mkdir -p "$output/native_build/$component"
  cp "$cache_template/CMakeCache.txt" "$output/native_build/$component/CMakeCache.txt"
done
for package in openarm_control_msgs openarm_description openarm_ik_ros; do
  mkdir -p "$output/build/$package"
  cp "$cache_template/CMakeCache.txt" "$output/build/$package/CMakeCache.txt"
done
native_request=$(openarm_build_state_requested_digest native Release 0 0 \
  "$output/install")
ros_request=$(openarm_build_state_requested_digest ros Release 0 0 \
  "$output/install")
openarm_build_state_write "$output/native_build" pending "$native_request"
openarm_build_state_publish_completed "$output/native_build" "$native_request" \
  can model commission transport control runtime
openarm_build_state_write "$output/build" pending "$ros_request"
openarm_build_state_publish_completed "$output/build" "$ros_request" \
  openarm_control_msgs openarm_description openarm_ik_ros

fingerprint=$(openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release)
openarm_write_launch_stamp "$fixture_root" "$output" "$description" "$fingerprint" Release
openarm_assert_current_launch_tree "$fixture_root" "$output" Release
printf 'CMAKE_C_FLAGS:STRING=-DCACHE_TAMPER\n' >> \
  "$output/native_build/can/CMakeCache.txt"
if openarm_assert_current_launch_tree "$fixture_root" "$output" Release \
    >/dev/null 2>&1; then
  printf 'Tampered effective CMake cache unexpectedly matched stamp\n' >&2
  exit 1
fi
cp "$cache_template/CMakeCache.txt" "$output/native_build/can/CMakeCache.txt"
openarm_assert_current_launch_tree "$fixture_root" "$output" Release
printf '# tampered hook\n' >> "$output/install/local_setup.bash"
if openarm_assert_current_launch_tree "$fixture_root" "$output" Release >/dev/null 2>&1; then
  printf 'Tampered setup helper unexpectedly matched stamp\n' >&2
  exit 1
fi
sed -i '$d' "$output/install/local_setup.bash"

copied_output="$work_root/copied-output"
cp -a "$output" "$copied_output"
if openarm_assert_current_launch_tree "$fixture_root" "$copied_output" Release \
    >/dev/null 2>&1; then
  printf 'Copied output tree unexpectedly replayed its stamp\n' >&2
  exit 1
fi

baseline_fingerprint=$fingerprint
printf 'untracked launch input\n' > "$fixture_root/ros2_ws/src/untracked.launch.py"
changed_fingerprint=$(openarm_compute_launch_source_fingerprint \
  "$fixture_root" "$description" Release 0)
[[ "$changed_fingerprint" != "$baseline_fingerprint" ]]
rm "$fixture_root/ros2_ws/src/untracked.launch.py"
printf 'ignored input\n' > "$fixture_root/runtime/cache.ignored"
changed_fingerprint=$(openarm_compute_launch_source_fingerprint \
  "$fixture_root" "$description" Release 0)
[[ "$changed_fingerprint" != "$baseline_fingerprint" ]]
rm "$fixture_root/runtime/cache.ignored"
changed_fingerprint=$(OPENARM_IK_ROS_COVERAGE=1 \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
[[ "$changed_fingerprint" != "$baseline_fingerprint" ]]
tool_bin="$work_root/tool-bin"
mkdir "$tool_bin"
printf '#!/usr/bin/env bash\nexec /usr/bin/cmake "$@"\n' > "$tool_bin/cmake"
chmod +x "$tool_bin/cmake"
changed_fingerprint=$(PATH="$tool_bin:$PATH" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
[[ "$changed_fingerprint" != "$baseline_fingerprint" ]]

# Effective wrapper, compiler, toolchain, linker, and generator-backend bytes
# are inputs even when their command/path spellings stay constant.
compiler_wrapper="$work_root/compiler-wrapper"
printf '#!/usr/bin/env bash\nexec /usr/bin/cc "$@"\n' > "$compiler_wrapper"
chmod +x "$compiler_wrapper"
wrapper_before=$(CC="$compiler_wrapper /usr/bin/cc -DONE=1" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
printf '#!/usr/bin/env bash\nexec /usr/bin/cc -DWRAPPER_CHANGED=1 "$@"\n' \
  > "$compiler_wrapper"
wrapper_after=$(CC="$compiler_wrapper /usr/bin/cc -DONE=1" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
[[ "$wrapper_before" != "$wrapper_after" ]]

cmake_compiler_launcher="$work_root/cmake-compiler-launcher"
printf '#!/usr/bin/env bash\nexec "$@"\n' > "$cmake_compiler_launcher"
chmod +x "$cmake_compiler_launcher"
launcher_before=$(CMAKE_C_COMPILER_LAUNCHER="$cmake_compiler_launcher;--one" \
  CMAKE_CXX_COMPILER_LAUNCHER="$cmake_compiler_launcher;--one" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" \
    Release 0)
launcher_args_after=$( \
  CMAKE_C_COMPILER_LAUNCHER="$cmake_compiler_launcher;--two" \
  CMAKE_CXX_COMPILER_LAUNCHER="$cmake_compiler_launcher;--two" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" \
    Release 0)
[[ "$launcher_before" != "$launcher_args_after" ]]
printf '#!/usr/bin/env bash\n: launcher-bytes-changed\nexec "$@"\n' \
  > "$cmake_compiler_launcher"
launcher_bytes_after=$( \
  CMAKE_C_COMPILER_LAUNCHER="$cmake_compiler_launcher;--one" \
  CMAKE_CXX_COMPILER_LAUNCHER="$cmake_compiler_launcher;--one" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" \
    Release 0)
[[ "$launcher_before" != "$launcher_bytes_after" ]]
linker_launcher_before=$( \
  CMAKE_C_LINKER_LAUNCHER="$cmake_compiler_launcher;--link-one" \
  CMAKE_CXX_LINKER_LAUNCHER="$cmake_compiler_launcher;--link-one" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" \
    Release 0)
linker_launcher_after=$( \
  CMAKE_C_LINKER_LAUNCHER="$cmake_compiler_launcher;--link-two" \
  CMAKE_CXX_LINKER_LAUNCHER="$cmake_compiler_launcher;--link-two" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" \
    Release 0)
[[ "$linker_launcher_before" != "$linker_launcher_after" ]]
if CMAKE_C_COMPILER_LAUNCHER='ambiguous launcher;--arg' \
    openarm_compute_launch_source_fingerprint "$fixture_root" "$description" \
      Release 0 >/dev/null 2>&1; then
  printf 'Ambiguous launcher executable unexpectedly accepted\n' >&2
  exit 1
fi

cxx_wrapper="$work_root/cxx-wrapper"
printf '#!/usr/bin/env bash\nexec /usr/bin/c++ "$@"\n' > "$cxx_wrapper"
chmod +x "$cxx_wrapper"
cxx_before=$(CXX="$cxx_wrapper /usr/bin/c++" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
printf '#!/usr/bin/env bash\nexec /usr/bin/c++ -DCXX_CHANGED=1 "$@"\n' > "$cxx_wrapper"
cxx_after=$(CXX="$cxx_wrapper /usr/bin/c++" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
[[ "$cxx_before" != "$cxx_after" ]]

toolchain_file="$work_root/toolchain.cmake"
printf 'set(CMAKE_SYSTEM_NAME Linux)\n' > "$toolchain_file"
toolchain_before=$(CMAKE_TOOLCHAIN_FILE="$toolchain_file" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
printf 'set(CMAKE_SYSTEM_NAME Generic)\n' > "$toolchain_file"
toolchain_after=$(CMAKE_TOOLCHAIN_FILE="$toolchain_file" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
[[ "$toolchain_before" != "$toolchain_after" ]]
if CMAKE_TOOLCHAIN_FILE=relative-toolchain.cmake \
    openarm_compute_launch_source_fingerprint "$fixture_root" "$description" \
      Release 0 >/dev/null 2>&1; then
  printf 'Relative/missing toolchain file unexpectedly accepted\n' >&2
  exit 1
fi

backend_wrapper="$work_root/backend-wrapper"
printf '#!/usr/bin/env bash\nexec /usr/bin/make "$@"\n' > "$backend_wrapper"
chmod +x "$backend_wrapper"
backend_before=$(CMAKE_MAKE_PROGRAM="$backend_wrapper" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
printf '#!/usr/bin/env bash\nexec /usr/bin/make --no-print-directory "$@"\n' \
  > "$backend_wrapper"
backend_after=$(CMAKE_MAKE_PROGRAM="$backend_wrapper" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
[[ "$backend_before" != "$backend_after" ]]

linker_wrapper="$work_root/linker-wrapper"
printf '#!/usr/bin/env bash\nexec /usr/bin/ld "$@"\n' > "$linker_wrapper"
chmod +x "$linker_wrapper"
linker_before=$(LD="$linker_wrapper" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
printf '#!/usr/bin/env bash\nexec /usr/bin/ld --warn-common "$@"\n' > "$linker_wrapper"
linker_after=$(LD="$linker_wrapper" \
  openarm_compute_launch_source_fingerprint "$fixture_root" "$description" Release 0)
[[ "$linker_before" != "$linker_after" ]]

# Symlink closure rejects broken/escaping targets and binds internal target
# metadata/content without following arbitrary directory links.
printf 'internal source target\n' > "$fixture_root/scripts/link-target.txt"
ln -s link-target.txt "$fixture_root/scripts/link-input.txt"
source_link_before=$(openarm_compute_launch_source_fingerprint \
  "$fixture_root" "$description" Release 0)
printf 'changed internal source target\n' > "$fixture_root/scripts/link-target.txt"
source_link_after=$(openarm_compute_launch_source_fingerprint \
  "$fixture_root" "$description" Release 0)
[[ "$source_link_before" != "$source_link_after" ]]
rm "$fixture_root/scripts/link-input.txt" "$fixture_root/scripts/link-target.txt"
ln -s "$work_root/toolchain.cmake" "$fixture_root/scripts/escaping-link"
if openarm_compute_launch_source_fingerprint "$fixture_root" "$description" \
    Release 0 >/dev/null 2>&1; then
  printf 'Escaping source symlink unexpectedly accepted\n' >&2
  exit 1
fi
rm "$fixture_root/scripts/escaping-link"
ln -s missing-target "$fixture_root/scripts/broken-link"
if openarm_compute_launch_source_fingerprint "$fixture_root" "$description" \
    Release 0 >/dev/null 2>&1; then
  printf 'Broken source symlink unexpectedly accepted\n' >&2
  exit 1
fi
rm "$fixture_root/scripts/broken-link"

printf 'install target\n' > "$output/install/internal-target"
ln -s internal-target "$output/install/internal-link"
install_link_before=$(openarm_compute_install_manifest_digest "$output")
printf 'changed install target\n' > "$output/install/internal-target"
install_link_after=$(openarm_compute_install_manifest_digest "$output")
[[ "$install_link_before" != "$install_link_after" ]]
rm "$output/install/internal-link" "$output/install/internal-target"
mkdir "$output/install/internal-directory"
printf 'directory target content\n' > "$output/install/internal-directory/content"
ln -s internal-directory "$output/install/internal-directory-link"
directory_link_before=$(openarm_compute_install_manifest_digest "$output")
printf 'changed directory target content\n' > "$output/install/internal-directory/content"
directory_link_after=$(openarm_compute_install_manifest_digest "$output")
[[ "$directory_link_before" != "$directory_link_after" ]]
rm "$output/install/internal-directory-link"
rm -r "$output/install/internal-directory"
ln -s "$work_root/toolchain.cmake" "$output/install/escaping-link"
if openarm_compute_install_manifest_digest "$output" >/dev/null 2>&1; then
  printf 'Escaping install symlink unexpectedly accepted\n' >&2
  exit 1
fi
rm "$output/install/escaping-link"
ln -s missing-target "$output/install/broken-link"
if openarm_compute_install_manifest_digest "$output" >/dev/null 2>&1; then
  printf 'Broken install symlink unexpectedly accepted\n' >&2
  exit 1
fi
rm "$output/install/broken-link"
mkfifo "$output/install/unexpected-fifo"
if openarm_compute_install_manifest_digest "$output" >/dev/null 2>&1; then
  printf 'Special install file unexpectedly accepted\n' >&2
  exit 1
fi
rm "$output/install/unexpected-fifo"

# A shared launch lease excludes mutation for the entire observed lifetime.
XDG_RUNTIME_DIR="$work_root" openarm_acquire_shared_locks \
  "$output" "$output/native_build" "$output/install"
blocked_mutation() { touch "$work_root/unexpected-mutation"; }
set +e
XDG_RUNTIME_DIR="$work_root" openarm_run_with_locks \
  "$(realpath -m -- "$output")" "$(realpath -m -- "$output/native_build")" \
  "$(realpath -m -- "$output/install")" -- blocked_mutation >/dev/null 2>&1
blocked_status=$?
set -e
[[ "$blocked_status" == 3 && ! -e "$work_root/unexpected-mutation" ]]
openarm_close_shared_lock_fds

exclusive_hold() {
  touch "$work_root/exclusive-ready"
  while [[ ! -e "$work_root/exclusive-release" ]]; do sleep 0.02; done
}
XDG_RUNTIME_DIR="$work_root" openarm_run_with_locks \
  "$(realpath -m -- "$output")" "$(realpath -m -- "$output/native_build")" \
  "$(realpath -m -- "$output/install")" -- exclusive_hold &
exclusive_pid=$!
for attempt in {1..200}; do
  [[ -e "$work_root/exclusive-ready" ]] && break
  sleep 0.01
done
[[ -e "$work_root/exclusive-ready" ]]
(
  XDG_RUNTIME_DIR="$work_root" \
    openarm_ensure_current_launch_tree "$fixture_root" "$output" never ''
  touch "$work_root/shared-finished"
) &
shared_pid=$!
sleep 0.1
[[ ! -e "$work_root/shared-finished" ]]
touch "$work_root/exclusive-release"
wait "$exclusive_pid"
wait "$shared_pid"
[[ -e "$work_root/shared-finished" ]]

rm -f "$work_root/exclusive-ready" "$work_root/exclusive-release"
exclusive_mutate() {
  printf '# raced mutation\n' >> \
    "$output/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_portal"
  touch "$work_root/exclusive-ready"
  while [[ ! -e "$work_root/exclusive-release" ]]; do sleep 0.02; done
}
XDG_RUNTIME_DIR="$work_root" openarm_run_with_locks \
  "$(realpath -m -- "$output")" "$(realpath -m -- "$output/native_build")" \
  "$(realpath -m -- "$output/install")" -- exclusive_mutate &
exclusive_pid=$!
for attempt in {1..200}; do
  [[ -e "$work_root/exclusive-ready" ]] && break
  sleep 0.01
done
set +e
XDG_RUNTIME_DIR="$work_root" \
  openarm_ensure_current_launch_tree "$fixture_root" "$output" never '' &
shared_pid=$!
sleep 0.1
touch "$work_root/exclusive-release"
wait "$exclusive_pid"
wait "$shared_pid"
raced_status=$?
set -e
[[ "$raced_status" == 1 ]]
sed -i '$d' "$output/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_portal"
openarm_close_shared_lock_fds

printf 'dirty current source\n' >> "$fixture_root/scripts/input.sh"
if openarm_assert_current_launch_tree "$fixture_root" "$output" Release >/dev/null 2>&1; then
  printf 'Dirty source unexpectedly matched stamp\n' >&2
  exit 1
fi
git -C "$fixture_root" restore scripts/input.sh
printf '# changed\n' >> "$output/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_portal"
if openarm_assert_current_launch_tree "$fixture_root" "$output" Release >/dev/null 2>&1; then
  printf 'Changed artifact unexpectedly matched stamp\n' >&2
  exit 1
fi
sed -i '$d' "$output/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_portal"

rm -f "$output/$OPENARM_LAUNCH_STAMP_NAME"
if openarm_assert_current_launch_tree "$fixture_root" "$output" Release >/dev/null 2>&1; then
  printf 'Missing stamp unexpectedly accepted\n' >&2
  exit 1
fi
if openarm_write_launch_stamp "$fixture_root" "$output" "$description" deadbeef Release \
    >/dev/null 2>&1; then
  printf 'Changed-during-build fingerprint unexpectedly stamped\n' >&2
  exit 1
fi
[[ ! -e "$output/$OPENARM_LAUNCH_STAMP_NAME" ]]

printf 'extern void oa_controller_create(void); void bad(void){oa_controller_create();}\n' |
  cc -x c -c - -o "$work_root/bad-session.o"
ar rcs "$output/build/openarm_ik_ros/libopenarm_virtual_control_session.a" \
  "$work_root/bad-session.o"
if openarm_assert_launch_authority "$output" >/dev/null 2>&1; then
  printf 'Direct controller session unexpectedly accepted\n' >&2
  exit 1
fi
ar rcs "$output/build/openarm_ik_ros/libopenarm_virtual_control_session.a" \
  "$work_root/session.o"
printf 'extern void oa_can_open(void); void bad(void){oa_can_open();}\n' |
  cc -x c -c - -o "$work_root/bad-runtime.o"
ar rcs "$output/install/lib/libopenarm_runtime.a" "$work_root/bad-runtime.o"
if openarm_assert_launch_authority "$output" >/dev/null 2>&1; then
  printf 'CAN-capable Runtime unexpectedly accepted\n' >&2
  exit 1
fi

# The ensure helper's default path always requests one incremental build and
# preserves the caller's bounded job choice. No real build or launch runs here.
shim_root="$work_root/shim-root"
mkdir -p "$shim_root/scripts"
printf '#!/usr/bin/env bash\nprintf "%%s\\n" "$*" >> "$OPENARM_ENSURE_LOG"\n' \
  > "$shim_root/scripts/build.sh"
chmod +x "$shim_root/scripts/build.sh"
openarm_assert_current_launch_tree() { printf 'assert %s\n' "$2" >> "$OPENARM_ENSURE_LOG"; }
export OPENARM_ENSURE_LOG="$work_root/ensure.log"
: > "$OPENARM_ENSURE_LOG"
openarm_ensure_current_launch_tree "$shim_root" "$work_root/shim-output" auto 1
[[ $(wc -l < "$OPENARM_ENSURE_LOG") == 2 ]]
grep -Fx -- '--incremental --output-root '"$work_root/shim-output"' --jobs 1' \
  "$OPENARM_ENSURE_LOG"
: > "$OPENARM_ENSURE_LOG"
openarm_ensure_current_launch_tree "$shim_root" "$work_root/shim-output" never ''
[[ $(wc -l < "$OPENARM_ENSURE_LOG") == 1 ]]

# The main wrapper honors the effective last build-mode option. Default and
# --build perform one build; explicit final --no-build performs none.
run_fixture="$work_root/run-fixture"
mkdir -p "$run_fixture/scripts" "$run_fixture/bin"
cp "$root_dir/run.sh" "$run_fixture/run.sh"
for command in install_all_dependencies.sh build.sh launch_web_portal.sh; do
  printf '#!/usr/bin/env bash\nprintf "%s %%s\\n" "$*" >> "$OPENARM_RUN_LOG"\n' \
    "$command" > "$run_fixture/scripts/$command"
  chmod +x "$run_fixture/scripts/$command"
done
printf '#!/usr/bin/env bash\nexit 0\n' > "$run_fixture/bin/firefox"
chmod +x "$run_fixture/bin/firefox"
export OPENARM_RUN_LOG="$work_root/run.log"
: > "$OPENARM_RUN_LOG"
PATH="$run_fixture/bin:$PATH" "$run_fixture/run.sh" --no-build
[[ $(grep -c '^build.sh ' "$OPENARM_RUN_LOG" || true) == 0 ]]
: > "$OPENARM_RUN_LOG"
PATH="$run_fixture/bin:$PATH" "$run_fixture/run.sh"
[[ $(grep -c '^build.sh ' "$OPENARM_RUN_LOG") == 1 ]]
: > "$OPENARM_RUN_LOG"
PATH="$run_fixture/bin:$PATH" "$run_fixture/run.sh" --no-build --build
[[ $(grep -c '^build.sh ' "$OPENARM_RUN_LOG") == 1 ]]
: > "$OPENARM_RUN_LOG"
PATH="$run_fixture/bin:$PATH" "$run_fixture/run.sh" --build --no-build
[[ $(grep -c '^build.sh ' "$OPENARM_RUN_LOG" || true) == 0 ]]

! grep -q 'OPENARM_PORTAL_BINARY' "$root_dir/scripts/launch_web_portal.sh"

printf '%s\n' 'Launch freshness and authority regression passed'
