#!/usr/bin/env bash
# Launch the DREAD live hardstop calibration wizard against this repository's
# OpenArm URDF.
#
#   ./dread.sh            # RViz + the wizard for both arms
#   ./dread.sh left       # one side only
#   ./dread.sh --rviz     # RViz side alone, wizard started by hand elsewhere
#
# This is third-party code, collected in dread/, adapted only where it named
# packages that do not exist here. It is used in preference to the calibration
# written in this repository because that one could not work: it read encoder
# positions from unpowered motors, and a disabled DaMiao reports a FROZEN
# encoder value. That is why a 180 degree sweep recorded as 13 degrees. DREAD's
# README documents the same failure and the same signature.
#
# What DREAD does differently, and why it works: it compliant-enables each motor
# at strictly zero torque (kp = kd = tau = 0). The motor is powered, so its
# position observer runs and streams true live angles, but commands nothing, so
# the arm stays limp and back-drivable by hand.
#
# THE MOTORS ARE POWERED. The arm should stay limp because the gains are zero,
# but this is not the read-only mode used elsewhere in this repository: a motor
# is energized. Keep clear, and keep the hardware e-stop within reach.
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
side=both
rviz_only=0
wizard_only=0

for argument in "$@"; do
  case "$argument" in
    left|right|both) side=$argument ;;
    --rviz)   rviz_only=1 ;;
    --wizard) wizard_only=1 ;;
    -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$argument" >&2; exit 2 ;;
  esac
done

if [[ ${EUID} -eq 0 ]]; then
  printf 'Do not run this as root.\n' >&2
  exit 1
fi

for interface in can0 can1; do
  ip link show "$interface" >/dev/null 2>&1 || {
    printf 'Interface %s does not exist. Plug in the CAN adapter.\n' "$interface" >&2
    exit 1
  }
  if [[ "$(ip -br link show "$interface" | awk '{print $2}')" != UP ]]; then
    printf 'Bringing %s up (needs sudo)...\n' "$interface"
    bash "$root_dir/scripts/setup_can_interfaces.sh"
    break
  fi
done

# Nothing else may own the bus while the wizard drives it. Two publishers on
# /joint_states also makes RViz show a blend of two sources.
for pattern in "openarm_real[_]observer" "openarm[_]portal"; do
  pids=$(pgrep -f "$pattern" 2>/dev/null || true)
  if [[ -n "$pids" ]]; then
    printf 'Stopping %s so it does not contend for the CAN bus.\n' "${pattern//[\[\]]/}"
    # shellcheck disable=SC2086
    kill $pids 2>/dev/null || true
  fi
done

# Snap's libraries shadow the system ones and break GTK/Qt binaries launched
# from here; the repo already carries the sanitizer used for RViz. Must run
# before sourcing ROS, since it clears XDG_DATA_DIRS.
# shellcheck disable=SC1091
source "$root_dir/scripts/lib/rviz_env.sh"
openarm_sanitize_snap_environment
openarm_configure_rviz_environment || true

# shellcheck disable=SC1091
set +u
source /opt/ros/lyrical/setup.bash
source "$root_dir/ros2_ws/install/setup.bash"
set -u

# The wizard is a plain Python package; it is run as a module rather than
# through `ros2 run`, so it does not need to be built into the ROS workspace.
export PYTHONPATH="$root_dir/dread:${PYTHONPATH:-}"

# The operator confirmed this installed pair by moving each physical arm while
# the read-only observer streamed encoder deltas: can0 is robot-right and can1
# is robot-left. The dual-channel DM-USB2FDCAN exposes one USB identity for both
# netdevs, so sysfs cannot rediscover this fact. Keep the commissioned assignment
# explicit while allowing an operator override after rewiring.
export M1_CAN_RIGHT="${OPENARM_RIGHT_CAN_INTERFACE:-can0}"
export M1_CAN_LEFT="${OPENARM_LEFT_CAN_INTERFACE:-can1}"

# The wizard reads ~/.config/m1/motor_map.yaml and refuses to start without it.
# Seed it from the current OpenArm v1.0 map. The collected M1/Ranger maps use a
# different rotary gripper and must never seed this prismatic 0..44 mm URDF.
config_dir="$HOME/.config/m1"
if [[ ! -f "$config_dir/motor_map.yaml" ]]; then
  mkdir -p "$config_dir"
  cp "$root_dir/dread/config/motor_map.openarm_v10.yaml" "$config_dir/motor_map.yaml"
  printf 'Seeded %s from the current OpenArm v1.0 URDF map.\n\n' \
    "$config_dir/motor_map.yaml"
fi
export M1_MOTOR_MAP="$config_dir/motor_map.yaml"
export M1_REPO_MAP="$root_dir/dread/config/motor_map.openarm_v10.yaml"
export M1_BACKUP_ROOT="$config_dir/calibration-backups"

urdf="$root_dir/ros2_ws/install/openarm_ik_ros/share/openarm_ik_ros/urdf/openarm_v10_bimanual.urdf"
[[ -r "$urdf" ]] || {
  printf 'URDF not found at %s\nRun ./run-real.sh once to build it.\n' "$urdf" >&2
  exit 1
}

start_rviz() {
  printf 'Starting robot_state_publisher + RViz on the OpenArm URDF...\n'
  ros2 launch "$root_dir/dread/launch/live_calibration.launch.py" \
    "urdf_path:=$urdf" &
  rviz_pid=$!
}

if ((wizard_only)); then
  exec python3 -m m1_can_tools.calibrate_live "$side"
fi

start_rviz
trap 'kill "${rviz_pid:-0}" 2>/dev/null || true' EXIT INT TERM

if ((rviz_only)); then
  printf '\nRViz is up. Run the wizard in another terminal:\n'
  printf '  ./dread.sh --wizard %s\n\n' "$side"
  wait "$rviz_pid"
  exit 0
fi

sleep 4
printf '\n'
printf 'THE MOTORS WILL BE POWERED at zero gains. The arm should stay limp and\n'
printf 'hand-movable. Keep the hardware e-stop within reach.\n\n'
python3 -m m1_can_tools.calibrate_live "$side"
