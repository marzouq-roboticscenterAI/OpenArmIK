#!/usr/bin/env bash
set -eo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
description_dir="$root_dir/upstream/openarm_description"
if [[ ! -f "$description_dir/package.xml" ]]; then
  printf 'Missing pinned upstream. Run %s/scripts/fetch_upstreams.sh first.\n' "$root_dir" >&2
  exit 1
fi
source /opt/ros/lyrical/setup.bash
set -u
colcon --log-base "$root_dir/ros2_ws/log" build \
  --base-paths "$description_dir" "$root_dir/ros2_ws/src" \
  --packages-select openarm_description openarm_ik_ros \
  --build-base "$root_dir/ros2_ws/build" \
  --install-base "$root_dir/ros2_ws/install" \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
