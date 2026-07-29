#!/usr/bin/env bash
set -eo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
OPENARM_IK_ROS_COVERAGE=1 "$root_dir/scripts/build.sh" --tests
source /opt/ros/lyrical/setup.bash
source "$root_dir/ros2_ws/install/setup.bash"
set -u
colcon --log-base "$root_dir/ros2_ws/log_coverage" test \
  --build-base "$root_dir/ros2_ws/build" \
  --install-base "$root_dir/ros2_ws/install" \
  --packages-select openarm_ik_ros \
  --event-handlers console_direct+
coverage_dir="$root_dir/ros2_ws/coverage"
mkdir -p "$coverage_dir"
{
  cd "$coverage_dir"
  for source in paired_transaction.cpp openarm_ik_ros_node.cpp; do
    target=openarm_ik_transaction
    if [[ "$source" == "openarm_ik_ros_node.cpp" ]]; then
      target=openarm_ik_ros_node
    fi
    gcov -b -c "$root_dir/ros2_ws/build/openarm_ik_ros/CMakeFiles/$target.dir/src/$source.gcda"
  done
} | tee "$coverage_dir/openarm_ik_ros.txt"
colcon test-result --test-result-base "$root_dir/ros2_ws/build" --verbose
