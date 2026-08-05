#!/usr/bin/env bash
# Bring up the two OpenArm CAN-FD interfaces. Requires root.
#
#   sudo ./scripts/setup_can_interfaces.sh            # bring up
#   sudo ./scripts/setup_can_interfaces.sh --down     # take down
#   ./scripts/setup_can_interfaces.sh --status        # inspect, no root needed
#
# This is the only privileged step. run-real.sh runs entirely unprivileged and
# refuses to start unless the interfaces are already up, so nothing that talks
# to the arms ever runs as root.
#
# Bitrates match the OpenArm DaMiao default: 1 Mbit/s arbitration with 5 Mbit/s
# CAN-FD data. Override with OPENARM_CAN_BITRATE / OPENARM_CAN_DBITRATE if the
# motors on your arms were commissioned differently; a mismatch shows up as
# bus-off or zero received frames rather than wrong data.
set -euo pipefail

interfaces=(can0 can1)
bitrate=${OPENARM_CAN_BITRATE:-1000000}
data_bitrate=${OPENARM_CAN_DBITRATE:-5000000}
# Queue depth. The default of 10 drops frames when seven motors answer at once.
txqueuelen=${OPENARM_CAN_TXQUEUELEN:-1000}
action=up

for argument in "$@"; do
  case "$argument" in
    --down) action=down ;;
    --status) action=status ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$argument" >&2; exit 2 ;;
  esac
done

show_status() {
  local interface
  for interface in "${interfaces[@]}"; do
    if ! ip link show "$interface" >/dev/null 2>&1; then
      printf '%-6s ABSENT   (no such interface; is the adapter plugged in?)\n' "$interface"
      continue
    fi
    local state bitrate_now
    state=$(ip -br link show "$interface" | awk '{print $2}')
    bitrate_now=$(ip -d link show "$interface" | awk '/bitrate/ {print $2; exit}')
    printf '%-6s %-8s bitrate=%s\n' "$interface" "$state" "${bitrate_now:-unset}"
  done
}

if [[ "$action" == status ]]; then
  show_status
  exit 0
fi

if [[ ${EUID} -ne 0 ]]; then
  printf 'This script must run as root: sudo %s %s\n' "$0" "$*" >&2
  exit 1
fi

for interface in "${interfaces[@]}"; do
  if ! ip link show "$interface" >/dev/null 2>&1; then
    printf 'Interface %s does not exist. Plug in the CAN adapter and retry.\n' \
      "$interface" >&2
    exit 1
  fi
done

if [[ "$action" == down ]]; then
  for interface in "${interfaces[@]}"; do
    ip link set "$interface" down 2>/dev/null || true
    printf 'took %s down\n' "$interface"
  done
  exit 0
fi

# Not every adapter supports every attribute. The gs_usb bridge on the OpenArm
# kit rejects restart-ms outright ("Device doesn't support restart from Bus
# Off"), so each attribute combination is attempted in order of preference and
# the first that the driver accepts wins. Attempting and falling back is the
# only reliable test; there is no capability flag to read beforehand.
configure_interface() {
  local interface=$1
  local -a attempts=(
    "bitrate $bitrate dbitrate $data_bitrate fd on restart-ms 100|CAN-FD, auto-restart"
    "bitrate $bitrate dbitrate $data_bitrate fd on|CAN-FD, no auto-restart"
    "bitrate $bitrate restart-ms 100|classic CAN, auto-restart"
    "bitrate $bitrate|classic CAN, no auto-restart"
  )
  local attempt args label
  for attempt in "${attempts[@]}"; do
    args=${attempt%%|*}
    label=${attempt##*|}
    # shellcheck disable=SC2086 -- args is a deliberately word-split argument list.
    if ip link set "$interface" type can $args 2>/dev/null; then
      printf 'configured %s: %s bit/s arbitration, %s\n' "$interface" "$bitrate" "$label"
      return 0
    fi
  done
  # Re-run the simplest form without silencing it so the driver says why.
  printf 'Failed to configure %s. The driver reported:\n' "$interface" >&2
  ip link set "$interface" type can bitrate "$bitrate" >&2
  return 1
}

for interface in "${interfaces[@]}"; do
  # Always down first: configuring a running interface fails, and a half
  # configured link is worse than a closed one.
  ip link set "$interface" down 2>/dev/null || true
  configure_interface "$interface"
  # A queue depth of 10 drops frames when seven motors answer at once. Some
  # drivers refuse to change it; that is a throughput risk, not a correctness
  # one, so warn rather than abort.
  ip link set "$interface" txqueuelen "$txqueuelen" 2>/dev/null ||
    printf 'warning: %s kept its default txqueuelen\n' "$interface" >&2
  ip link set "$interface" up
done

printf '\n'
show_status
printf '\nInterfaces are up. Now run, without sudo:\n  ./run-real.sh\n'
