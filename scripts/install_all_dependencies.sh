#!/usr/bin/env bash
set -euo pipefail

apply=0
assume_yes=0
update_index=1
verify_only=0

usage() {
  cat <<'EOF'
Usage: scripts/install_all_dependencies.sh [OPTIONS]

Install the build, ROS 2 Lyrical, RViz, collision, and local web-portal
dependencies for OpenArmIK. The default is a review-only dry run.

Options:
  --apply        Run apt-get through sudo
  --verify       Verify installed packages without using sudo
  --yes          Pass --assume-yes to apt-get (requires --apply)
  --skip-update  Do not run apt-get update before installation
  -h, --help     Show this help

Examples:
  ./scripts/install_all_dependencies.sh
  ./scripts/install_all_dependencies.sh --apply
  ./scripts/install_all_dependencies.sh --apply --yes
EOF
}

while (($#)); do
  case "$1" in
    --apply)
      apply=1
      ;;
    --verify)
      verify_only=1
      ;;
    --yes)
      assume_yes=1
      ;;
    --skip-update)
      update_index=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if ((assume_yes && !apply)); then
  printf '%s\n' '--yes requires --apply' >&2
  exit 2
fi
if ((verify_only && apply)); then
  printf '%s\n' '--verify and --apply are mutually exclusive' >&2
  exit 2
fi

if [[ $(uname -m) != x86_64 ]]; then
  printf 'Unsupported architecture: %s (expected x86_64)\n' "$(uname -m)" >&2
  exit 1
fi

if [[ ! -r /etc/os-release ]]; then
  printf '%s\n' 'Cannot identify the operating system: /etc/os-release is missing.' >&2
  exit 1
fi
. /etc/os-release
if [[ ${ID:-} != ubuntu ]]; then
  printf 'Unsupported distribution: %s (expected Ubuntu)\n' "${ID:-unknown}" >&2
  exit 1
fi

packages=(
  build-essential
  cmake
  ninja-build
  pkg-config
  git
  curl
  jq
  firefox
  cppcheck
  lcov
  procps
  util-linux
  iproute2
  can-utils
  strace
  x11-utils
  xdg-utils
  libeigen3-dev
  libboost-dev
  libfcl-dev
  libssl-dev
  libx11-dev
  libxdamage-dev
  libxext-dev
  libxfixes-dev
  libxrender-dev
  libgl1-mesa-dri
  libglx-mesa0
  mesa-utils
  ros-dev-tools
  python3-colcon-common-extensions
  ros-lyrical-ament-cmake
  ros-lyrical-ament-cmake-gtest
  ros-lyrical-action-msgs
  ros-lyrical-builtin-interfaces
  ros-lyrical-diagnostic-msgs
  ros-lyrical-geometry-msgs
  ros-lyrical-launch-ros
  ros-lyrical-rclcpp
  ros-lyrical-rclcpp-action
  ros-lyrical-robot-state-publisher
  ros-lyrical-rosidl-default-generators
  ros-lyrical-rosidl-default-runtime
  ros-lyrical-rviz2
  ros-lyrical-sensor-msgs
  ros-lyrical-std-msgs
  ros-lyrical-std-srvs
  ros-lyrical-tf2-geometry-msgs
  ros-lyrical-tf2-ros
  ros-lyrical-unique-identifier-msgs
  ros-lyrical-xacro
)

missing_metadata=()
for package in "${packages[@]}"; do
  if ! apt-cache show "$package" >/dev/null 2>&1; then
    missing_metadata+=("$package")
  fi
done
if ((${#missing_metadata[@]})); then
  printf 'These packages are not visible to apt:\n' >&2
  printf '  %s\n' "${missing_metadata[@]}" >&2
  printf '%s\n' 'Check that the Ubuntu and ROS 2 Lyrical repositories are enabled.' >&2
  exit 1
fi

printf 'OpenArmIK dependency plan for Ubuntu %s (%s):\n' \
  "${VERSION_ID:-unknown}" "$(uname -m)"
printf '  %s\n' "${packages[@]}"

verify_installed() {
  local package
  local -a failed=()
  for package in "${packages[@]}"; do
    if ! dpkg-query -W -f='${db:Status-Abbrev}' "$package" 2>/dev/null | \
        grep -q '^ii '; then
      failed+=("$package")
    fi
  done
  if ((${#failed[@]})); then
    printf 'Installation verification failed for:\n' >&2
    printf '  %s\n' "${failed[@]}" >&2
    return 1
  fi
  [[ -r /opt/ros/lyrical/setup.bash ]] || {
    printf '%s\n' 'Packages are installed, but the ROS setup file is missing.' >&2
    return 1
  }
  command -v geckodriver >/dev/null 2>&1 || {
    printf '%s\n' 'Browser fidelity tests require geckodriver (the Firefox snap provides it on this platform).' >&2
    return 1
  }
  geckodriver --version >/dev/null 2>&1 || {
    printf '%s\n' 'geckodriver is present but cannot execute.' >&2
    return 1
  }
  printf '\nAll OpenArmIK system dependencies are installed.\n'
}

if ((verify_only)); then
  verify_installed
  exit
fi

if ((!apply)); then
  printf '\nReview only; no packages were installed. To apply:\n'
  printf '  %q --apply\n' "$0"
  exit 0
fi

command -v sudo >/dev/null 2>&1 || {
  printf '%s\n' 'sudo is required but was not found.' >&2
  exit 1
}

sudo -v
if ((update_index)); then
  sudo apt-get update
fi

apt_options=(install --no-install-recommends)
if ((assume_yes)); then
  apt_options+=(--assume-yes)
fi
sudo apt-get "${apt_options[@]}" "${packages[@]}"

verify_installed
printf 'Next: %s/scripts/build.sh --tests\n' \
  "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
