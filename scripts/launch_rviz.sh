#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root_dir/scripts/lib/description_pin.sh"
source "$root_dir/scripts/build_lock.sh"
source "$root_dir/scripts/lib/launch_integrity.sh"
output_root="$root_dir/ros2_ws"
build_mode=auto
jobs=
launcher_arguments=()

usage() {
  cat <<'EOF'
Usage: scripts/launch_rviz.sh [OPTIONS] [ROS_LAUNCH_ARGUMENTS]

Incrementally build the current virtual stack, then launch it with stock RViz.

Options:
  --output-root PATH  Build/install root (default: ros2_ws)
  --build             Force the default incremental build policy
  --no-build          Require a current stamped and gated install without building
  --jobs JOBS         Maximum concurrent build jobs (default: build default)
  -h, --help          Show this help
EOF
}

while (($#)); do
  case "$1" in
    --output-root)
      (($# >= 2)) || { printf '%s requires a path\n' "$1" >&2; exit 2; }
      output_root=$2
      shift 2
      ;;
    --build) build_mode=always; shift ;;
    --no-build) build_mode=never; shift ;;
    --jobs)
      (($# >= 2)) && [[ -n "$2" ]] || {
        printf '%s requires a positive integer\n' "$1" >&2
        exit 2
      }
      jobs=$2
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    --)
      shift
      launcher_arguments+=("$@")
      break
      ;;
    *) launcher_arguments+=("$1"); shift ;;
  esac
done
[[ -z "$jobs" || "$jobs" =~ ^[1-9][0-9]*$ ]] || {
  printf 'Jobs must be a positive integer: %s\n' "$jobs" >&2
  exit 2
}
if [[ "$output_root" != /* ]]; then
  output_root="$PWD/$output_root"
fi
output_root=$(realpath -m -- "$output_root")
if [[ "$output_root" == *:* || "$output_root" == *\;* ]]; then
  printf 'Output roots containing : or ; are unsupported: %s\n' "$output_root" >&2
  exit 2
fi

runtime_dir=${XDG_RUNTIME_DIR:-}
if [[ -z "$runtime_dir" || ! -d "$runtime_dir" || ! -w "$runtime_dir" ]]; then
  runtime_dir="/tmp/openarmik-runtime-$UID"
  if [[ -L "$runtime_dir" ]]; then
    printf 'Refusing symlinked fallback runtime directory: %s\n' "$runtime_dir" >&2
    exit 1
  fi
  if [[ -e "$runtime_dir" && ! -O "$runtime_dir" ]]; then
    printf 'Fallback runtime directory is owned by another user: %s\n' \
      "$runtime_dir" >&2
    exit 1
  fi
  mkdir -m 700 -p -- "$runtime_dir"
  chmod 700 -- "$runtime_dir"
fi
lock_file="$runtime_dir/openarmik-gui-$UID.lock"
exec 9>"$lock_file"
if ! flock -n 9; then
  printf 'An OpenArm GUI is already running; close it or press Ctrl+C in its terminal.\n' >&2
  exit 3
fi
openarm_ensure_current_launch_tree \
  "$root_dir" "$output_root" "$build_mode" "$jobs"
for variable in SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA SNAP_INSTANCE_KEY SNAP_LIBRARY_PATH SNAP_NAME SNAP_REAL_HOME SNAP_REEXEC SNAP_REVISION SNAP_USER_COMMON SNAP_USER_DATA GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_MODULE_DIR QT_PLUGIN_PATH QT_QPA_PLATFORMTHEME XDG_DATA_DIRS; do
  unset "$variable" || true
done
source /opt/ros/lyrical/setup.bash
source "$output_root/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
package_prefix=$(ros2 pkg prefix openarm_ik_ros 2>/dev/null || true)
[[ -n "$package_prefix" ]] || {
  printf '%s\n' 'The installed openarm_ik_ros package was not found.' >&2
  exit 1
}
package_prefix=$(realpath -e -- "$package_prefix")
expected_package_prefix=$(realpath -e -- "$output_root/install/openarm_ik_ros")
[[ "$package_prefix" == "$expected_package_prefix" ]] || {
  printf 'Sourced package prefix escaped the audited output root: %s\n' \
    "$package_prefix" >&2
  exit 1
}

# RViz/Ogre on this Wayland desktop requires XWayland/GLX.  Its Ogre render
# window can flicker after a HiDPI resize when Qt uses devicePixelRatio=2, so
# keep scaling off for this process only.  XWayland still reports 192 DPI and
# preserves readable fonts without the unstable double-scaled render target.
export QT_QPA_PLATFORM=xcb
export QT_XCB_GL_INTEGRATION=xcb_glx
export QT_ENABLE_HIGHDPI_SCALING=0
export QT_SCREEN_SCALE_FACTORS=1
unset QT_SCALE_FACTOR QT_AUTO_SCREEN_SCALE_FACTOR || true
renderer=${OPENARM_RVIZ_RENDERER:-auto}
if [[ "$renderer" == "auto" ]]; then
  # Hardware GLX presentation flickers during live resize through XWayland on
  # this HiDPI hybrid-GPU laptop.  The small OpenArm scene runs smoothly in
  # llvmpipe and remains stable while resizing.  Keep GPU acceleration for a
  # native X11 session or when explicitly requested.
  if [[ ${XDG_SESSION_TYPE:-} == "wayland" ]]; then
    renderer=software
  elif [[ -e /dev/nvidia0 ]] && command -v nvidia-smi >/dev/null 2>&1 && \
      nvidia-smi >/dev/null 2>&1; then
    renderer=nvidia
  else
    renderer=integrated
  fi
fi
case "$renderer" in
  nvidia)
    unset LIBGL_ALWAYS_SOFTWARE || true
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __VK_LAYER_NV_optimus=NVIDIA_only
    export __GL_SYNC_TO_VBLANK=1
    export __GL_GSYNC_ALLOWED=0
    export __GL_VRR_ALLOWED=0
    printf 'OpenArm RViz renderer: NVIDIA PRIME offload (XWayland/GLX)\n'
    ;;
  integrated)
    unset LIBGL_ALWAYS_SOFTWARE || true
    unset __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME __VK_LAYER_NV_optimus __GL_SYNC_TO_VBLANK __GL_GSYNC_ALLOWED __GL_VRR_ALLOWED || true
    printf 'OpenArm RViz renderer: integrated GPU (XWayland/GLX)\n'
    ;;
  software)
    unset __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME __VK_LAYER_NV_optimus __GL_SYNC_TO_VBLANK __GL_GSYNC_ALLOWED __GL_VRR_ALLOWED || true
    export LIBGL_ALWAYS_SOFTWARE=1
    printf 'OpenArm RViz renderer: Mesa software rasterizer (XWayland/GLX)\n'
    ;;
  *)
    printf 'OPENARM_RVIZ_RENDERER must be auto, nvidia, integrated, or software\n' >&2
    exit 2
    ;;
esac

rviz_enabled=1
launch_arguments=()
for argument in "${launcher_arguments[@]}"; do
  if [[ "$argument" == rviz:=* ]]; then
    rviz_value=${argument#rviz:=}
    case "${rviz_value,,}" in
      true|1) rviz_enabled=1 ;;
      false|0) rviz_enabled=0 ;;
      *)
        printf 'rviz must be true, false, 1, or 0 (received %s)\n' "$rviz_value" >&2
        exit 2
        ;;
    esac
  else
    launch_arguments+=("$argument")
  fi
done

if (( ! rviz_enabled )); then
  exec ros2 launch openarm_ik_ros openarm_ik_rviz.launch.py \
    rviz:=false "${launch_arguments[@]}"
fi

share_dir="$package_prefix/share/openarm_ik_ros"
close_helper="$package_prefix/lib/openarm_ik_ros/close_rviz_window"
if [[ ! -x "$close_helper" ]]; then
  printf 'Missing %s; run %s/scripts/build.sh first.\n' "$close_helper" "$root_dir" >&2
  exit 2
fi
core_pid=
rviz_pid=
shutting_down=0

process_is_running() {
  local pid=$1 state
  kill -0 "$pid" 2>/dev/null || return 1
  state=$(ps -o stat= -p "$pid" 2>/dev/null) || return 1
  [[ "$state" != Z* ]]
}

wait_for_exit() {
  local pid=$1 attempts=${2:-50}
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    process_is_running "$pid" || return 0
    sleep 0.1
  done
  return 1
}

shutdown() {
  local status=${1:-0}
  if ((shutting_down)); then
    return
  fi
  shutting_down=1
  trap - EXIT HUP INT TERM

  if [[ -n "$rviz_pid" ]] && process_is_running "$rviz_pid"; then
    if ! "$close_helper" "$rviz_pid" --timeout 3; then
      kill -TERM -- "-$rviz_pid" 2>/dev/null || true
    fi
    if ! wait_for_exit "$rviz_pid" 30; then
      kill -KILL -- "-$rviz_pid" 2>/dev/null || true
    fi
  fi
  [[ -z "$rviz_pid" ]] || wait "$rviz_pid" 2>/dev/null || true

  if [[ -n "$core_pid" ]] && process_is_running "$core_pid"; then
    kill -INT -- "-$core_pid" 2>/dev/null || true
    if ! wait_for_exit "$core_pid" 50; then
      kill -TERM -- "-$core_pid" 2>/dev/null || true
    fi
    if ! wait_for_exit "$core_pid" 30; then
      kill -KILL -- "-$core_pid" 2>/dev/null || true
    fi
  fi
  [[ -z "$core_pid" ]] || wait "$core_pid" 2>/dev/null || true
  exit "$status"
}

trap 'printf "\nStopping OpenArm RViz...\n"; shutdown 130' INT
trap 'shutdown 129' HUP
trap 'shutdown 143' TERM
trap 'shutdown $?' EXIT

# Keep RViz out of the ROS launcher's signal path.  Closing it through the
# window manager avoids the RViz/Ogre SIGINT teardown crash seen on this host.
setsid ros2 launch openarm_ik_ros openarm_ik_rviz.launch.py \
  rviz:=false "${launch_arguments[@]}" &
core_pid=$!
setsid rviz2 -d "$share_dir/rviz/openarm_ik.rviz" --ros-args -r __node:=rviz2 &
rviz_pid=$!

set +e
wait -n "$core_pid" "$rviz_pid"
status=$?
set -e
shutdown "$status"
