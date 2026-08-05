#!/usr/bin/env bash
# Hand-guided joint range calibration.
#
#   ./calib.sh                    # can0 and can1
#   ./calib.sh can1 can0          # swap which interface is listed first
#
# Opens a GUI. Select an arm and a motor, press Start, move that joint by hand
# from one hard stop to the other, press Stop. Repeat for all sixteen motors,
# then press Save JSON.
#
# The motors are NEVER powered. The arms stay limp, so you are moving dead
# weight rather than fighting a servo, and the only frame that goes on the bus
# is a DaMiao status query. Nothing here can enable, zero, or move a motor.
#
# No sudo. Bringing the CAN interfaces up needs root and is done below by
# setup_can_interfaces.sh, which elevates itself; everything that talks to the
# motors runs unprivileged.
#
# Output: ~/.openarm_calibration.json, holding for each motor the full recorded
# path rather than just the endpoints, plus the derived extent and path length.
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
interfaces=("${1:-can0}" "${2:-can1}")

for argument in "$@"; do
  case "$argument" in
    -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
  esac
done

if [[ ${EUID} -eq 0 ]]; then
  printf 'Do not run this as root; it does not need it.\n' >&2
  exit 1
fi

for interface in "${interfaces[@]}"; do
  if ! ip link show "$interface" >/dev/null 2>&1; then
    printf 'Interface %s does not exist. Plug in the CAN adapter.\n' "$interface" >&2
    exit 1
  fi
done

down=()
for interface in "${interfaces[@]}"; do
  [[ "$(ip -br link show "$interface" | awk '{print $2}')" == UP ]] || down+=("$interface")
done
if ((${#down[@]})); then
  printf 'CAN interfaces are down: %s\n' "${down[*]}"
  printf 'Bringing them up (this needs sudo)...\n\n'
  "$root_dir/scripts/setup_can_interfaces.sh"
  printf '\n'
fi

# Snap puts /snap/core20 libraries ahead of the system ones, which makes a
# GTK binary die with "undefined symbol: __libc_pthread_init" from the wrong
# libpthread. The repo already carries the sanitizer used for RViz. It must run
# BEFORE sourcing ROS, because it clears XDG_DATA_DIRS.
# shellcheck disable=SC1091
source "$root_dir/scripts/lib/rviz_env.sh"
openarm_sanitize_snap_environment

"$root_dir/scripts/build.sh" --incremental --output-root "$root_dir/ros2_ws"

# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash >/dev/null 2>&1 || true
# shellcheck disable=SC1091
source "$root_dir/ros2_ws/install/setup.bash" >/dev/null 2>&1 || true

binary="$root_dir/ros2_ws/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_calibrate_gui"
[[ -x "$binary" ]] || {
  printf 'Calibration binary not found at %s\n' "$binary" >&2
  exit 1
}

printf '\n'
printf 'Hand calibration: the motors stay UNPOWERED throughout.\n'
printf 'For each motor: Start, move the joint through its full range, Stop.\n'
printf 'Then Save JSON. Output goes to ~/.openarm_calibration.json\n\n'

exec "$binary" "${interfaces[@]}"
