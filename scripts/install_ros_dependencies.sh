#!/usr/bin/env bash
set -euo pipefail

packages=(
  build-essential
  libx11-dev
  ros-dev-tools
  ros-lyrical-ament-cmake
  ros-lyrical-ament-cmake-gtest
  ros-lyrical-rclcpp
  ros-lyrical-sensor-msgs
  ros-lyrical-diagnostic-msgs
  ros-lyrical-geometry-msgs
  ros-lyrical-robot-state-publisher
  ros-lyrical-rviz2
  ros-lyrical-xacro
  strace
)

if [[ "${1:-}" != "--apply" ]]; then
  printf 'Review only; no packages were installed. To apply:\n'
  printf '  sudo apt install'
  printf ' %q' "${packages[@]}"
  printf '\n'
  exit 0
fi
sudo apt install "${packages[@]}"
