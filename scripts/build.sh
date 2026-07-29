#!/usr/bin/env bash
set -eo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
description_dir="$root_dir/upstream/openarm_description"
if [[ ! -f "$description_dir/package.xml" ]]; then
  printf 'Missing pinned upstream. Run %s/scripts/fetch_upstreams.sh first.\n' "$root_dir" >&2
  exit 1
fi
cmake -S "$root_dir/model" -B "$root_dir/ros2_ws/model_build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DOA_MODEL_BUILD_TESTS=OFF \
  -DCMAKE_INSTALL_PREFIX="$root_dir/ros2_ws/install"
cmake --build "$root_dir/ros2_ws/model_build" --parallel
cmake --install "$root_dir/ros2_ws/model_build"
source /opt/ros/lyrical/setup.bash
set -u
export CMAKE_PREFIX_PATH="$root_dir/ros2_ws/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
coverage_args=("-DOPENARM_IK_ROS_COVERAGE=OFF")
if [[ "${OPENARM_IK_ROS_COVERAGE:-0}" == "1" ]]; then
  coverage_args=("-DOPENARM_IK_ROS_COVERAGE=ON")
fi
colcon --log-base "$root_dir/ros2_ws/log" build \
  --base-paths "$description_dir" "$root_dir/ros2_ws/src" \
  --packages-select openarm_description openarm_ik_ros \
  --build-base "$root_dir/ros2_ws/build" \
  --install-base "$root_dir/ros2_ws/install" \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 "${coverage_args[@]}"
