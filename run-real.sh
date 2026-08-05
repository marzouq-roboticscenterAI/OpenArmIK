#!/usr/bin/env bash
# Launch the OpenArm portal against the physically connected arms.
#
# The counterpart to run.sh, which drives the simulated controller. This one
# attaches to real hardware, and the difference that matters is what it can do
# to it: nothing. The node behind this script is read-only. It polls DaMiao
# motor status, decodes the feedback, and publishes /joint_states so RViz
# mirrors the arms. It has no path to enable, zero, or move a motor.
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
  # Bring them up rather than refusing. The helper re-execs itself under sudo,
  # so the password prompt appears here and the rest of this script, and
  # everything that talks to the arms, still runs unprivileged.
  printf 'CAN interfaces are down: %s\n' "${down[*]}"
  printf 'Bringing them up (this needs sudo)...\n\n'
  "$root_dir/scripts/setup_can_interfaces.sh"
  printf '\n'
  for interface in "${interfaces[@]}"; do
    if [[ "$(ip -br link show "$interface" | awk '{print $2}')" != UP ]]; then
      printf 'Interface %s is still down; cannot continue.\n' "$interface" >&2
      exit 1
    fi
  done
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
command -v firefox >/dev/null 2>&1 || {
  printf '%s\n' 'Firefox is not installed or is not on PATH.' >&2
  exit 1
}

build_arguments=(--incremental --output-root "$output_root")
[[ -z "$jobs" ]] || build_arguments+=(--jobs "$jobs")
if [[ "$build_mode" != never ]]; then
  "$root_dir/scripts/build.sh" "${build_arguments[@]}"
fi

printf '\n'
printf 'Real-arm mode: READ ONLY. It polls motor status and mirrors the pose;\n'
printf 'it cannot enable, zero, or move a motor.\n'
printf 'Interfaces up: %s\n' "${interfaces[*]}"
printf 'Connecting to the motors automatically and taking the current pose as\n'
printf 'neutral. If the arms are not at rest, press Clear zero then Connect.\n\n'

exec "$root_dir/scripts/launch_web_portal.sh" --real --firefox "$@" --no-build
