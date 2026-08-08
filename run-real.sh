#!/usr/bin/env bash
# Launch the OpenArm portal against the physically connected arms.
#
# The counterpart to run.sh, which drives the simulated controller. This one
# attaches to real hardware. Startup is passive: it mirrors encoder feedback
# without enabling motion until the operator presses Connect and enable motors.
# Once armed, portal targets and demos command the physical J1..J7 axes. J8 is
# the calibrated gripper axis and can be opened, closed, or torque-limited with
# the MoveGripper action/C CLI while its encoder remains under the watchdog.
#
# It is also passive on startup. No CAN socket is opened and no frame is sent
# until you press Connect in the portal.
#
# This script does NOT need sudo, and refuses to run as root. Opening the CAN
# interfaces is the only privileged step and it is separate:
#
#   sudo bash scripts/setup_can_interfaces.sh
#   ./run-real.sh
#
# Keeping them apart means nothing that talks to the arms ever runs with root.
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
interfaces=(can0 can1)

for argument in "$@"; do
  case "$argument" in
    -h|--help)
      sed -n '2,20p' "$0"
      printf '\nOptions are passed through to scripts/launch_web_portal.sh:\n\n'
      exec "$root_dir/scripts/launch_web_portal.sh" --help
      ;;
  esac
done

if [[ ${EUID} -eq 0 ]]; then
  printf 'Do not run this as root. Bring the interfaces up separately:\n'    >&2
  printf '  sudo bash %s/scripts/setup_can_interfaces.sh\n  ./run-real.sh\n' \
    "$root_dir" >&2
  exit 1
fi

# Refuse to start on interfaces that are not up. Failing here names the problem;
# starting anyway would surface as an empty sweep and look like dead motors.
missing=()
down=()
for interface in "${interfaces[@]}"; do
  if ! ip link show "$interface" >/dev/null 2>&1; then
    missing+=("$interface")
  elif [[ "$(ip -br link show "$interface" | awk '{print $2}')" != UP ]]; then
    down+=("$interface")
  fi
done
if ((${#missing[@]})); then
  printf 'These CAN interfaces do not exist: %s\n' "${missing[*]}" >&2
  printf 'Plug in the DaMiao USB-to-CAN adapter and check: ip -br link show\n' >&2
  exit 1
fi
if ((${#down[@]})); then
  printf 'CAN interfaces are down: %s\n' "${down[*]}" >&2
  printf 'Bring them up in a separate terminal, then rerun this command:\n' >&2
  printf '  bash %s/scripts/setup_can_interfaces.sh\n' "$root_dir" >&2
  exit 1
fi

output_root="$root_dir/ros2_ws"
jobs=
build_mode=auto
arguments=("$@")
for ((index = 0; index < ${#arguments[@]}; index++)); do
  case "${arguments[index]}" in
    --output-root)
      ((index + 1 < ${#arguments[@]})) || {
        printf '%s requires a path\n' --output-root >&2
        exit 2
      }
      output_root=${arguments[index + 1]}
      ((index += 1))
      ;;
    --jobs)
      ((index + 1 < ${#arguments[@]})) || {
        printf '%s requires a value\n' --jobs >&2
        exit 2
      }
      jobs=${arguments[index + 1]}
      ((index += 1))
      ;;
    --build)
      build_mode=always
      ;;
    --no-build)
      build_mode=never
      ;;
  esac
done

"$root_dir/scripts/install_all_dependencies.sh" --verify >/dev/null

build_arguments=(--incremental --output-root "$output_root")
[[ -z "$jobs" ]] || build_arguments+=(--jobs "$jobs")
if [[ "$build_mode" != never ]]; then
  "$root_dir/scripts/build.sh" "${build_arguments[@]}"
fi

printf '\n'
printf 'Real-arm mode: startup is PASSIVE; every motor remains disabled until\n'
printf 'you explicitly press "Connect and enable motors" in the portal.\n'
printf 'Interfaces up: %s\n' "${interfaces[*]}"
printf 'Saved per-joint calibration is preserved; connecting does not redefine\n'
printf 'neutral. Disconnect, E-stop, Ctrl+C, faults, and feedback timeout all\n'
printf 'disable every motor. Keep the hardware stop within reach.\n\n'

exec "$root_dir/scripts/launch_web_portal.sh" --real --firefox "$@" --no-build
