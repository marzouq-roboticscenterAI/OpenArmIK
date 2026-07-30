#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
output_root="$root_dir/ros2_ws"
port=8080
open_browser=1
browser_command=xdg-open
build_mode=auto
renderer=${OPENARM_RVIZ_RENDERER:-auto}

usage() {
  cat <<'EOF'
Usage: scripts/launch_web_portal.sh [OPTIONS]

Build if necessary, then launch the virtual OpenArm controller, stock RViz,
and the compiled loopback-only web portal. Ctrl+C shuts down the portal,
RViz, and ROS processes in that order.

Options:
  --port PORT        Loopback HTTP port (default: 8080)
  --output-root PATH Build/install root (default: ros2_ws)
  --build            Force an incremental build before launch
  --no-build         Never build; fail if installed products are missing
  --no-browser       Print the URL without opening the browser
  --firefox          Open the portal specifically with Firefox
  --renderer MODE    auto, software, integrated, or nvidia
  -h, --help         Show this help

This launcher is virtual-only. It does not open SocketCAN or control physical
motors. Portal motion remains disabled unless the installed controller reports
a verified collision scene containing both arms and the central support pole.
EOF
}

while (($#)); do
  case "$1" in
    --port)
      (($# >= 2)) || { printf '%s requires a value\n' "$1" >&2; exit 2; }
      port=$2
      shift 2
      ;;
    --output-root)
      (($# >= 2)) || { printf '%s requires a path\n' "$1" >&2; exit 2; }
      output_root=$2
      shift 2
      ;;
    --build)
      build_mode=always
      shift
      ;;
    --no-build)
      build_mode=never
      shift
      ;;
    --no-browser)
      open_browser=0
      shift
      ;;
    --firefox)
      browser_command=firefox
      shift
      ;;
    --renderer)
      (($# >= 2)) || { printf '%s requires a value\n' "$1" >&2; exit 2; }
      renderer=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ "$port" =~ ^[0-9]+$ ]] && ((port >= 1024 && port <= 65535)) || {
  printf 'Port must be an integer from 1024 through 65535: %s\n' "$port" >&2
  exit 2
}
case "$renderer" in
  auto|software|integrated|nvidia) ;;
  *)
    printf 'Renderer must be auto, software, integrated, or nvidia: %s\n' \
      "$renderer" >&2
    exit 2
    ;;
esac

if [[ "$output_root" != /* ]]; then
  output_root="$PWD/$output_root"
fi
output_root=$(realpath -m -- "$output_root")
if [[ "$output_root" == *:* || "$output_root" == *\;* ]]; then
  printf 'Output roots containing : or ; are unsupported: %s\n' \
    "$output_root" >&2
  exit 2
fi

[[ -n ${DISPLAY:-} ]] || {
  printf '%s\n' 'DISPLAY is unset. Run this from the logged-in graphical session.' >&2
  exit 1
}
[[ -r /opt/ros/lyrical/setup.bash ]] || {
  printf '%s\n' 'ROS 2 Lyrical is missing; run scripts/install_all_dependencies.sh.' >&2
  exit 1
}

runtime_dir=${XDG_RUNTIME_DIR:-}
if [[ -z "$runtime_dir" || ! -d "$runtime_dir" || ! -w "$runtime_dir" ]]; then
  runtime_dir="/tmp/openarmik-runtime-$UID"
  if [[ -L "$runtime_dir" ]]; then
    printf 'Refusing symlinked fallback runtime directory: %s\n' \
      "$runtime_dir" >&2
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
  printf '%s\n' 'An OpenArm GUI is already running for this user.' >&2
  exit 3
fi

if ss -H -ltn "sport = :$port" 2>/dev/null | grep -q .; then
  printf 'Port 127.0.0.1:%s is already in use.\n' "$port" >&2
  exit 3
fi

install_setup="$output_root/install/setup.bash"

build_stack() {
  "$root_dir/scripts/build.sh" --incremental --output-root "$output_root"
}

if [[ "$build_mode" == always ]]; then
  build_stack
elif [[ ! -r "$install_setup" && "$build_mode" == auto ]]; then
  build_stack
fi

[[ -r "$install_setup" ]] || {
  printf 'Missing install setup: %s\n' "$install_setup" >&2
  printf 'Run: %s/scripts/build.sh --tests --output-root %q\n' \
    "$root_dir" "$output_root" >&2
  exit 1
}

set +u
source /opt/ros/lyrical/setup.bash
source "$install_setup"
set -u

package_prefix=$(ros2 pkg prefix openarm_ik_ros 2>/dev/null || true)
[[ -n "$package_prefix" ]] || {
  printf '%s\n' 'The installed openarm_ik_ros package was not found.' >&2
  exit 1
}
share_dir="$package_prefix/share/openarm_ik_ros"
close_helper="$package_prefix/lib/openarm_ik_ros/close_rviz_window"
portal_binary=${OPENARM_PORTAL_BINARY:-"$package_prefix/lib/openarm_ik_ros/openarm_portal"}
rviz_package_prefix=$(ros2 pkg prefix rviz2 2>/dev/null || true)
[[ -n "$rviz_package_prefix" ]] || {
  printf '%s\n' 'The installed stock rviz2 package was not found.' >&2
  exit 1
}
rviz_command=$(command -v rviz2 2>/dev/null || true)
rviz_executable=$(realpath -e -- "$rviz_command" 2>/dev/null || true)
[[ -x "$rviz_executable" ]] || {
  printf '%s\n' 'The stock RViz executable is missing from PATH.' >&2
  exit 1
}
case "$rviz_executable" in
  "$rviz_package_prefix"/*) ;;
  *)
    printf 'RViz executable is outside its ROS package prefix: %s\n' "$rviz_executable" >&2
    exit 1
    ;;
esac

if [[ ! -x "$portal_binary" && "$build_mode" == auto ]]; then
  build_stack
  set +u
  source "$install_setup"
  set -u
fi

[[ -x "$portal_binary" ]] || {
  printf 'Missing compiled portal executable: %s\n' "$portal_binary" >&2
  printf '%s\n' 'The portal target must be integrated before this demo can run.' >&2
  printf 'Build with: %s/scripts/build.sh --tests --output-root %q\n' \
    "$root_dir" "$output_root" >&2
  exit 1
}
[[ -x "$close_helper" ]] || {
  printf 'Missing RViz close helper: %s\n' "$close_helper" >&2
  exit 1
}
[[ -r "$share_dir/rviz/openarm_ik.rviz" ]] || {
  printf 'Missing RViz configuration under: %s\n' "$share_dir" >&2
  exit 1
}

for variable in SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA SNAP_INSTANCE_KEY SNAP_LIBRARY_PATH SNAP_NAME SNAP_REAL_HOME SNAP_REEXEC SNAP_REVISION SNAP_USER_COMMON SNAP_USER_DATA GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_MODULE_DIR QT_PLUGIN_PATH QT_QPA_PLATFORMTHEME; do
  unset "$variable" || true
done
export QT_QPA_PLATFORM=xcb
export QT_XCB_GL_INTEGRATION=xcb_glx
export QT_ENABLE_HIGHDPI_SCALING=0
export QT_SCREEN_SCALE_FACTORS=1
unset QT_SCALE_FACTOR QT_AUTO_SCREEN_SCALE_FACTOR || true

if [[ "$renderer" == auto ]]; then
  if [[ ${XDG_SESSION_TYPE:-} == wayland ]]; then
    renderer=software
  elif [[ -e /dev/nvidia0 ]] && command -v nvidia-smi >/dev/null 2>&1 && \
      nvidia-smi >/dev/null 2>&1; then
    renderer=nvidia
  else
    renderer=integrated
  fi
fi
case "$renderer" in
  software)
    export LIBGL_ALWAYS_SOFTWARE=1
    unset __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME __VK_LAYER_NV_optimus || true
    ;;
  integrated)
    unset LIBGL_ALWAYS_SOFTWARE __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME __VK_LAYER_NV_optimus || true
    ;;
  nvidia)
    unset LIBGL_ALWAYS_SOFTWARE || true
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __VK_LAYER_NV_optimus=NVIDIA_only
    ;;
esac
printf 'OpenArm portal RViz renderer: %s\n' "$renderer"

core_pid=
rviz_pid=
portal_pid=
shutting_down=0

process_is_running() {
  local pid=$1 state
  kill -0 "$pid" 2>/dev/null || return 1
  state=$(ps -o stat= -p "$pid" 2>/dev/null) || return 1
  [[ "$state" != Z* ]]
}

wait_for_exit() {
  local pid=$1 attempts=${2:-50} attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    process_is_running "$pid" || return 0
    sleep 0.1
  done
  return 1
}

stop_group() {
  local signal=$1 pid=$2
  [[ -n "$pid" ]] || return 0
  process_is_running "$pid" || return 0
  kill -"$signal" -- "-$pid" 2>/dev/null || true
}

shutdown() {
  local status=${1:-0}
  if ((shutting_down)); then
    return
  fi
  shutting_down=1
  trap - EXIT HUP INT TERM

  if [[ -n "$portal_pid" ]] && process_is_running "$portal_pid"; then
    stop_group TERM "$portal_pid"
    wait_for_exit "$portal_pid" 50 || stop_group KILL "$portal_pid"
  fi
  [[ -z "$portal_pid" ]] || wait "$portal_pid" 2>/dev/null || true

  if [[ -n "$rviz_pid" ]] && process_is_running "$rviz_pid"; then
    "$close_helper" "$rviz_pid" --timeout 3 || stop_group TERM "$rviz_pid"
    wait_for_exit "$rviz_pid" 30 || stop_group KILL "$rviz_pid"
  fi
  [[ -z "$rviz_pid" ]] || wait "$rviz_pid" 2>/dev/null || true

  if [[ -n "$core_pid" ]] && process_is_running "$core_pid"; then
    stop_group INT "$core_pid"
    wait_for_exit "$core_pid" 50 || stop_group TERM "$core_pid"
    wait_for_exit "$core_pid" 30 || stop_group KILL "$core_pid"
  fi
  [[ -z "$core_pid" ]] || wait "$core_pid" 2>/dev/null || true
  exit "$status"
}

trap 'printf "\nStopping OpenArm portal demo...\n"; shutdown 130' INT
trap 'shutdown 129' HUP
trap 'shutdown 143' TERM
trap 'shutdown $?' EXIT

setsid ros2 launch openarm_ik_ros openarm_ik_rviz.launch.py rviz:=false &
core_pid=$!

setsid "$rviz_executable" -d "$share_dir/rviz/openarm_ik.rviz" \
  --ros-args -r __node:=rviz2 &
rviz_pid=$!

for ((attempt = 0; attempt < 100; attempt++)); do
  process_is_running "$rviz_pid" || {
    printf '%s\n' 'RViz exited before the portal could start.' >&2
    exit 1
  }
  if [[ -r "/proc/$rviz_pid/stat" ]]; then
    rviz_start_ticks=$(awk '{print $22}' "/proc/$rviz_pid/stat")
    [[ "$rviz_start_ticks" =~ ^[0-9]+$ ]] && break
  fi
  sleep 0.1
done
[[ ${rviz_start_ticks:-} =~ ^[0-9]+$ ]] || {
  printf '%s\n' 'Could not establish the RViz process identity.' >&2
  exit 1
}

setsid "$portal_binary" --rviz-pid "$rviz_pid" \
  --rviz-start-ticks "$rviz_start_ticks" \
  --rviz-executable "$rviz_executable" --port "$port" &
portal_pid=$!

url="http://127.0.0.1:$port/"
healthy=0
for ((attempt = 0; attempt < 200; attempt++)); do
  process_is_running "$portal_pid" || {
    printf '%s\n' 'The portal exited before becoming healthy.' >&2
    exit 1
  }
  if curl --fail --silent --show-error --max-time 1 \
      "http://127.0.0.1:$port/api/health" >/dev/null 2>&1; then
    healthy=1
    break
  fi
  sleep 0.1
done
((healthy)) || {
  printf '%s\n' 'The portal did not become healthy within 20 seconds.' >&2
  exit 1
}

printf '\nOpenArm virtual portal: %s\n' "$url"
printf '%s\n' 'Press Ctrl+C here to stop the portal, RViz, and ROS.'
if ((open_browser)); then
  command -v "$browser_command" >/dev/null 2>&1 || {
    printf 'Browser executable not found: %s\n' "$browser_command" >&2
    exit 1
  }
  "$browser_command" "$url" 9>&- >/dev/null 2>&1 || \
    printf 'Could not open a browser automatically; visit %s\n' "$url" >&2
fi

set +e
wait -n "$core_pid" "$rviz_pid" "$portal_pid"
status=$?
set -e
shutdown "$status"
