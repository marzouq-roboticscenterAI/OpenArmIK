#!/usr/bin/env bash
# Briefly enable the motors on one CAN bus so their LEDs turn green, then
# disable them again. Used to confirm by eye which physical arm is on which
# interface.
#
#   ./scripts/blink_arm_leds.sh                  # gripper only on can1, 5 s
#   ./scripts/blink_arm_leds.sh --all            # all 8 motors on can1
#   ./scripts/blink_arm_leds.sh --interface can0 --all --seconds 3
#   ./scripts/blink_arm_leds.sh --yes            # skip the confirmation prompt
#
# READ THIS BEFORE RUNNING.
#
# Enabling a DaMiao motor energizes it. A motor whose saved gains are non-zero
# will start holding a position target the instant it is enabled, and if that
# target is far from where the arm is now, the joint SNAPS toward it. That is a
# real hazard on a 7-DOF arm. Keep clear of the arm and keep a hand near the
# power switch.
#
# This is why the default is the gripper alone: it is the lowest-torque
# actuator on the arm, and one green LED identifies the arm just as well as
# eight do. Use --all only once the single-motor test has shown the arm stays
# put.
#
# The script sends exactly two kinds of frame, enable (0xFC) and disable
# (0xFD). It never sends an MIT/motion command, so no position, velocity or
# torque target is ever transmitted. Whatever the motors do while enabled comes
# from their own saved configuration, not from this script.
#
# Disable is sent from an EXIT trap, so the motors are released even if you
# Ctrl+C partway through or a cansend fails.
set -euo pipefail

interface=can1
seconds=5
motors=(08)
assume_yes=0

while (($#)); do
  case "$1" in
    --interface) interface=${2:?--interface requires a name}; shift 2 ;;
    --seconds)   seconds=${2:?--seconds requires a number}; shift 2 ;;
    --all)       motors=(01 02 03 04 05 06 07 08); shift ;;
    --motors)    IFS=, read -r -a motors <<< "${2:?--motors requires a list}"; shift 2 ;;
    --yes)       assume_yes=1; shift ;;
    -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done

command -v cansend >/dev/null 2>&1 || {
  printf 'cansend is not installed. Install can-utils.\n' >&2
  exit 1
}
if ! ip link show "$interface" >/dev/null 2>&1; then
  printf 'Interface %s does not exist.\n' "$interface" >&2
  exit 1
fi
if [[ "$(ip -br link show "$interface" | awk '{print $2}')" != UP ]]; then
  printf 'Interface %s is down. Run: sudo bash scripts/setup_can_interfaces.sh\n' \
    "$interface" >&2
  exit 1
fi

# cansend demands exactly 3 hex characters for a standard CAN ID. Passing a
# bare "08" makes it print its usage and send nothing, which looks identical to
# a motor that ignored the frame -- so build every ID through here.
frame_id() {
  printf '%03X' "$((16#$1))"
}

# Read one motor's position without energizing it: a refresh-status query,
# which is read-only. Used before and after so any movement is visible.
read_positions() {
  local label=$1 motor raw
  local dump=/tmp/openarm_blink_$$.txt
  timeout 3 candump -t d "$interface" > "$dump" 2>&1 &
  local dump_pid=$!
  sleep 0.3
  for motor in "${motors[@]}"; do
    cansend "$interface" "7FF#${motor}00CC0000000000" 2>/dev/null || true
    sleep 0.05
  done
  sleep 0.3
  kill "$dump_pid" 2>/dev/null || true
  wait "$dump_pid" 2>/dev/null || true
  printf '  %s: ' "$label"
  for motor in "${motors[@]}"; do
    # Reply ID is send+0x10; position is the 16-bit field at data[1..2],
    # spanning +/-12.5 rad.
    raw=$(awk -v want="$(printf '%03X' $((16#$motor + 16)))" \
      '$3 == want {print $6 $7; exit}' "$dump" 2>/dev/null || true)
    if [[ -n "$raw" ]]; then
      # Hex to decimal in bash, because mawk has no strtonum; awk then does the
      # floating-point scaling, which bash cannot.
      printf '%s=%s ' "$motor" \
        "$(awk -v d="$((16#$raw))" 'BEGIN{printf "%+.3f", (d/65535)*25 - 12.5}')"
    else
      printf '%s=?? ' "$motor"
    fi
  done
  printf '\n'
  rm -f "$dump"
}

released=0
release_motors() {
  ((released)) && return 0
  released=1
  printf '\nDisabling motors...\n'
  local motor attempt
  # Twice, in case a frame is lost. Disable is idempotent.
  for attempt in 1 2; do
    for motor in "${motors[@]}"; do
      cansend "$interface" "$(frame_id "$motor")#FFFFFFFFFFFFFFFD" || true
      sleep 0.02
    done
  done
  printf 'Motors disabled. LEDs should be red again.\n'
}
trap release_motors EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf 'Interface : %s\n' "$interface"
printf 'Motors    : %s\n' "${motors[*]}"
printf 'Hold time : %s s\n\n' "$seconds"
if ((${#motors[@]} > 1)); then
  printf 'WARNING: enabling %d motors at once. A joint can snap toward a saved\n' \
    "${#motors[@]}"
  printf 'position target the moment it is energized. Stand clear of the arm.\n\n'
fi

if ((!assume_yes)); then
  read -r -p 'Type yes to enable: ' reply
  [[ "$reply" == yes ]] || { printf 'Aborted; nothing was sent.\n'; exit 0; }
fi

printf '\nPositions before:\n'
read_positions before

printf '\nEnabling...\n'
for motor in "${motors[@]}"; do
  cansend "$interface" "$(frame_id "$motor")#FFFFFFFFFFFFFFFC"
  printf '  motor 0x%s enabled\n' "$motor"
  sleep 0.05
done

printf '\nLEDs should be GREEN now. Look at the arms.\n'
for ((remaining = seconds; remaining > 0; remaining--)); do
  printf '\r  releasing in %2d s ' "$remaining"
  sleep 1
done
printf '\r                      \r'

printf 'Positions after:\n'
read_positions after
printf '\nIf the two rows differ by more than a few thousandths, a joint moved\n'
printf 'while energized. Investigate before enabling anything again.\n'

# release_motors runs from the EXIT trap.
