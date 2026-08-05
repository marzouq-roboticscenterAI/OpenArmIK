#!/usr/bin/env bash
# Write the current physical pose into each motor as its mechanical zero.
#
#   ./scripts/set_zero.sh --interface can0 --dry-run   # show what would be sent
#   ./scripts/set_zero.sh --interface can0             # calibrate one arm
#   ./scripts/set_zero.sh --both                       # calibrate can0 and can1
#
# No sudo. The CAN interfaces need root to bring up, which
# scripts/setup_can_interfaces.sh does; talking to the motors afterwards does
# not.
#
# WHAT THIS IS FOR
#
# The motors are not mounted such that their encoder zero coincides with the
# URDF zero pose, so a resting arm renders lifted and joint 4 reports about
# -0.94 rad against a URDF range starting at 0. This is the fix upstream uses:
# hold the arm at the neutral pose and tell each motor "here is zero".
#
# READ THIS BEFORE RUNNING
#
# This writes to non-volatile memory inside each motor. It is PERSISTENT across
# power cycles, it applies to every program that talks to these motors and not
# just this repository, and it OVERWRITES the existing calibration, which is not
# recoverable from here. The pre-calibration readings are saved to a log so
# there is at least a record of what was replaced.
#
# Get the pose right first. Whatever position the arm is physically in when
# this runs becomes the permanent definition of zero for every joint.
#
# The sequence per motor is upstream's, from
# openarm_can/setup/cli/commands/zero_position_commands.cpp:
#   Disable (0xFD) -> Set Zero (0xFE) -> Disable (0xFD)
# Disable brackets the write so no motor is left energized. No MIT/motion frame
# is ever sent, so nothing here commands a position, velocity or torque.
set -euo pipefail

interfaces=()
motors=(01 02 03 04 05 06 07 08)
dry_run=0
assume_yes=0
log_file=${OPENARM_SET_ZERO_LOG:-$HOME/.openarm_set_zero.log}

while (($#)); do
  case "$1" in
    --interface) interfaces+=("${2:?--interface requires a name}"); shift 2 ;;
    --both)      interfaces=(can0 can1); shift ;;
    --motors)    IFS=, read -r -a motors <<< "${2:?--motors requires a list}"; shift 2 ;;
    --dry-run)   dry_run=1; shift ;;
    --yes)       assume_yes=1; shift ;;
    -h|--help)   sed -n '2,38p' "$0"; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
((${#interfaces[@]})) || interfaces=(can0)

command -v cansend >/dev/null 2>&1 || {
  printf 'cansend is not installed. Install can-utils.\n' >&2
  exit 1
}
for interface in "${interfaces[@]}"; do
  if ! ip link show "$interface" >/dev/null 2>&1; then
    printf 'Interface %s does not exist.\n' "$interface" >&2
    exit 1
  fi
  if [[ "$(ip -br link show "$interface" | awk '{print $2}')" != UP ]]; then
    printf 'Interface %s is down. Run: ./scripts/setup_can_interfaces.sh\n' "$interface" >&2
    exit 1
  fi
done

# cansend wants exactly three hex characters for a standard CAN ID. A bare "08"
# makes it print usage and send nothing, which is indistinguishable from a motor
# ignoring the frame -- so every ID is built here.
frame_id() { printf '%03X' "$((16#$1))"; }

# Read positions without energizing anything: refresh-status is a query.
read_positions() {
  local interface=$1 motor raw dump
  dump=$(mktemp)
  timeout 4 candump -t d "$interface" > "$dump" 2>&1 &
  local dump_pid=$!
  sleep 0.3
  for motor in "${motors[@]}"; do
    cansend "$interface" "7FF#${motor}00CC0000000000" 2>/dev/null || true
    sleep 0.04
  done
  sleep 0.3
  kill "$dump_pid" 2>/dev/null || true
  wait "$dump_pid" 2>/dev/null || true
  local out=""
  for motor in "${motors[@]}"; do
    raw=$(awk -v want="$(printf '%03X' $((16#$motor + 16)))" \
      '$3 == want {print $6 $7; exit}' "$dump" 2>/dev/null || true)
    if [[ -n "$raw" ]]; then
      out+="$motor=$(awk -v d="$((16#$raw))" \
        'BEGIN{printf "%+.4f", (d/65535)*25 - 12.5}') "
    else
      out+="$motor=no-reply "
    fi
  done
  rm -f "$dump"
  printf '%s' "$out"
}

printf 'Interfaces : %s\n' "${interfaces[*]}"
printf 'Motors     : %s\n' "${motors[*]}"
printf 'Log        : %s\n\n' "$log_file"

printf 'Current positions (these are what will become zero):\n'
declare -A before
for interface in "${interfaces[@]}"; do
  before[$interface]=$(read_positions "$interface")
  printf '  %-6s %s\n' "$interface" "${before[$interface]}"
done

if ((dry_run)); then
  printf '\nDry run. The following would be sent, and nothing was written:\n'
  for interface in "${interfaces[@]}"; do
    for motor in "${motors[@]}"; do
      printf '  cansend %s %s#FFFFFFFFFFFFFFFD   (disable)\n' "$interface" "$(frame_id "$motor")"
      printf '  cansend %s %s#FFFFFFFFFFFFFFFE   (SET ZERO, persistent)\n' \
        "$interface" "$(frame_id "$motor")"
      printf '  cansend %s %s#FFFFFFFFFFFFFFFD   (disable)\n' "$interface" "$(frame_id "$motor")"
    done
  done
  exit 0
fi

printf '\n'
printf 'This PERMANENTLY overwrites the mechanical zero of %d motor(s) on %s.\n' \
  "$((${#motors[@]} * ${#interfaces[@]}))" "${interfaces[*]}"
printf 'The pose above becomes the definition of zero. It survives power cycles\n'
printf 'and affects every program that uses these motors. The old calibration\n'
printf 'cannot be recovered.\n\n'
printf 'Confirm the arms are physically in the neutral pose before continuing.\n\n'

if ((!assume_yes)); then
  read -r -p 'Type SET ZERO to proceed: ' reply
  [[ "$reply" == "SET ZERO" ]] || { printf 'Aborted; nothing was written.\n'; exit 0; }
fi

{
  printf '=== %s ===\n' "$(date -Is)"
  for interface in "${interfaces[@]}"; do
    printf 'before %s: %s\n' "$interface" "${before[$interface]}"
  done
} >> "$log_file"

for interface in "${interfaces[@]}"; do
  printf '\nCalibrating %s\n' "$interface"
  for motor in "${motors[@]}"; do
    id=$(frame_id "$motor")
    cansend "$interface" "${id}#FFFFFFFFFFFFFFFD"   # disable
    sleep 0.1
    cansend "$interface" "${id}#FFFFFFFFFFFFFFFE"   # set zero
    sleep 0.1
    cansend "$interface" "${id}#FFFFFFFFFFFFFFFD"   # disable again
    sleep 0.1
    printf '  motor 0x%s zeroed\n' "$motor"
  done
done

printf '\nPositions after (every joint should now read close to 0):\n'
for interface in "${interfaces[@]}"; do
  after=$(read_positions "$interface")
  printf '  %-6s %s\n' "$interface" "$after"
  printf 'after  %s: %s\n' "$interface" "$after" >> "$log_file"
done

printf '\nDone. If a joint did not go to ~0, that motor did not accept the write;\n'
printf 're-run for that motor with --motors.\n'
printf '\nNow that the motors carry the zero themselves, the software offset is\n'
printf 'redundant. Launch with capture_zero_on_connect:=false, or press Clear\n'
printf 'zero in the portal, so the two do not stack.\n'
