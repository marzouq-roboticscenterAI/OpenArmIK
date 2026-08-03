#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root_dir/scripts/lib/description_pin.sh"
source "$root_dir/scripts/build_lock.sh"
source "$root_dir/scripts/lib/launch_integrity.sh"
source "$root_dir/scripts/lib/rviz_env.sh"
output_root="$root_dir/ros2_ws"
port=8080
open_browser=1
browser_command=xdg-open
build_mode=auto
jobs=
show_rviz=1

usage() {
  cat <<'EOF'
Usage: scripts/launch_web_portal.sh [OPTIONS]

Incrementally build the current virtual OpenArm controller, then launch the
compiled loopback-only web portal. Ctrl+C shuts down the portal and ROS.

Options:
  --port PORT        Loopback HTTP port (default: 8080)
  --output-root PATH Build/install root (default: ros2_ws)
  --build            Force an incremental build before launch
  --no-build         Never build; require a current stamped and gated install
  --jobs JOBS        Maximum concurrent build jobs (default: build default)
  --no-browser       Print the URL without opening the browser
  --firefox          Open the portal specifically with Firefox
  --no-rviz          Do not open the RViz 3D window
  -h, --help         Show this help

The 3D view is a real RViz window loaded with a panel-free layout, so it shows
the render view only. Use scripts/launch_rviz.sh for the full RViz engineering
layout with the Displays and Views panels.

This launcher is virtual-only. It does not open SocketCAN or control physical
motors. Portal motion uses a limited sampled nominal virtual prefilter and
central keepout, and the controller additionally re-proves that keepout from
measured feedback on every control cycle. Certified collision checking remains
unavailable (collision_checked=false); this is not physical collision
certification or a verified scene.
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
    --jobs)
      (($# >= 2)) && [[ -n "$2" ]] || {
        printf '%s requires a positive integer\n' "$1" >&2
        exit 2
      }
      jobs=$2
      shift 2
      ;;
    --no-browser)
      open_browser=0
      shift
      ;;
    --no-rviz)
      show_rviz=0
      shift
      ;;
    --firefox)
      browser_command=firefox
      shift
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
[[ -z "$jobs" || "$jobs" =~ ^[1-9][0-9]*$ ]] || {
  printf 'Jobs must be a positive integer: %s\n' "$jobs" >&2
  exit 2
}

if [[ "$output_root" != /* ]]; then
  output_root="$PWD/$output_root"
fi
output_root=$(realpath -m -- "$output_root")
if [[ "$output_root" == *:* || "$output_root" == *\;* ]]; then
  printf 'Output roots containing : or ; are unsupported: %s\n' \
    "$output_root" >&2
  exit 2
fi

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
openarm_ensure_current_launch_tree \
  "$root_dir" "$output_root" "$build_mode" "$jobs"

[[ -r "$install_setup" ]] || {
  printf 'Missing install setup: %s\n' "$install_setup" >&2
  printf 'Run: %s/scripts/build.sh --tests --output-root %q\n' \
    "$root_dir" "$output_root" >&2
  exit 1
}

set +u
openarm_sanitize_snap_environment
source /opt/ros/lyrical/setup.bash
source "$install_setup"
set -u
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
portal_binary="$package_prefix/lib/openarm_ik_ros/openarm_portal"
share_dir="$package_prefix/share/openarm_ik_ros"
close_helper="$package_prefix/lib/openarm_ik_ros/close_rviz_window"
if ((show_rviz)); then
  [[ -x "$close_helper" ]] || {
    printf 'Missing %s; run %s/scripts/build.sh first.\n' "$close_helper" \
      "$root_dir" >&2
    exit 2
  }
  [[ -r "$share_dir/rviz/openarm_bare.rviz" ]] || {
    printf 'Missing panel-free RViz layout in %s.\n' "$share_dir" >&2
    exit 2
  }
  openarm_configure_rviz_environment || exit $?
fi

[[ -x "$portal_binary" ]] || {
  printf 'Missing compiled portal executable: %s\n' "$portal_binary" >&2
  printf '%s\n' 'The portal target must be integrated before this demo can run.' >&2
  printf 'Build with: %s/scripts/build.sh --tests --output-root %q\n' \
    "$root_dir" "$output_root" >&2
  exit 1
}
core_pid=
portal_pid=
rviz_pid=
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

  if [[ -n "$rviz_pid" ]] && process_is_running "$rviz_pid"; then
    if ! "$close_helper" "$rviz_pid" --timeout 3; then
      stop_group TERM "$rviz_pid"
    fi
    wait_for_exit "$rviz_pid" 30 || stop_group KILL "$rviz_pid"
  fi
  [[ -z "$rviz_pid" ]] || wait "$rviz_pid" 2>/dev/null || true

  if [[ -n "$portal_pid" ]] && process_is_running "$portal_pid"; then
    stop_group TERM "$portal_pid"
    wait_for_exit "$portal_pid" 50 || stop_group KILL "$portal_pid"
  fi
  [[ -z "$portal_pid" ]] || wait "$portal_pid" 2>/dev/null || true

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

setsid ros2 launch openarm_ik_ros openarm_ik_rviz.launch.xml rviz:=false &
core_pid=$!

setsid "$portal_binary" --port "$port" &
portal_pid=$!

if ((show_rviz)); then
  # The 3D view is stock RViz rather than a browser canvas, loaded with a
  # panel-free layout so only the render view is shown.  Keep it out of the ROS
  # launcher's signal path: closing it through the window manager avoids the
  # RViz/Ogre SIGINT teardown crash seen on this host.
  setsid rviz2 -d "$share_dir/rviz/openarm_bare.rviz" \
    --ros-args -r __node:=rviz2_portal &
  rviz_pid=$!
fi

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
printf '%s\n' 'Press Ctrl+C here to stop the portal and ROS.'
if ((open_browser)); then
  command -v "$browser_command" >/dev/null 2>&1 || {
    printf 'Browser executable not found: %s\n' "$browser_command" >&2
    exit 1
  }
  (
    exec 9>&-
    openarm_close_shared_lock_fds
    exec "$browser_command" "$url"
  ) >/dev/null 2>&1 || \
      printf 'Could not open a browser automatically; visit %s\n' "$url" >&2
fi

set +e
wait -n "$core_pid" "$portal_pid" ${rviz_pid:+"$rviz_pid"}
status=$?
set -e
shutdown "$status"
