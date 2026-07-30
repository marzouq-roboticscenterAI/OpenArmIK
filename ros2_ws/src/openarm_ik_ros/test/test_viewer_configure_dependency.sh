#!/usr/bin/env bash
set -euo pipefail

module= license= description= cmake=
while (($#)); do
  case "$1" in
    --module) module=$2 ;;
    --license) license=$2 ;;
    --description) description=$2 ;;
    --cmake) cmake=$2 ;;
    *) printf 'unknown configure-dependency argument: %s\n' "$1" >&2; exit 2 ;;
  esac
  shift 2
done
for required in module license description cmake; do
  [[ -n ${!required} ]] || { printf 'missing --%s\n' "$required" >&2; exit 2; }
done

work_root=$(mktemp -d "${TMPDIR:-/tmp}/openarm-viewer-configure.XXXXXX")
cleanup() { rm -rf -- "$work_root"; }
trap cleanup EXIT
source_dir="$work_root/source"
build_dir="$work_root/build"
mkdir -p "$source_dir/licenses"
cp -- "$license" "$source_dir/licenses/openarm_description-LICENSE.txt"
printf '<robot name="fixture"/>\n' > "$source_dir/stage.urdf"
cat > "$source_dir/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(openarm_viewer_configure_fixture NONE)
include("${VIEWER_MODULE}")
openarm_configure_viewer_assets(
  "${DESCRIPTION_SHARE}" "${CMAKE_CURRENT_SOURCE_DIR}/stage.urdf"
  "${CMAKE_CURRENT_BINARY_DIR}/viewer")
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/viewer-assets.stamp"
  COMMAND "${CMAKE_COMMAND}" -E touch
    "${CMAKE_CURRENT_BINARY_DIR}/viewer-assets.stamp"
  DEPENDS "${OPENARM_VIEWER_STAGE_A_URDF}" "${OPENARM_VIEWER_MANIFEST}")
add_custom_target(viewer_fixture ALL
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/viewer-assets.stamp")
EOF

"$cmake" -S "$source_dir" -B "$build_dir" \
  "-DVIEWER_MODULE=$module" "-DDESCRIPTION_SHARE=$description" >/dev/null
"$cmake" --build "$build_dir" --parallel 1 >/dev/null
stamp="$build_dir/viewer-assets.stamp"
[[ -f "$stamp" ]]

fixture_license="$source_dir/licenses/openarm_description-LICENSE.txt"
original_size=$(stat -c '%s' "$fixture_license")
original_hash=$(sha256sum "$fixture_license" | awk '{print $1}')
rm "$stamp"
sleep 1
printf 'X' | dd of="$fixture_license" bs=1 seek=0 count=1 conv=notrunc status=none
[[ $(stat -c '%s' "$fixture_license") == "$original_size" ]]
[[ $(sha256sum "$fixture_license" | awk '{print $1}') != "$original_hash" ]]

set +e
"$cmake" --build "$build_dir" --parallel 1 \
  >"$work_root/incremental.log" 2>&1
incremental_status=$?
set -e
[[ "$incremental_status" != 0 ]] || {
  printf 'same-size license mutation did not force cached reconfiguration\n' >&2
  exit 1
}
grep -Fq 'The pinned openarm_description Apache-2.0 license changed' \
  "$work_root/incremental.log"
[[ ! -e "$stamp" ]] || {
  printf 'viewer asset stamp was emitted after failed license pin validation\n' >&2
  exit 1
}
printf 'viewer configure dependency gate: same-size license mutation failed closed\n'
