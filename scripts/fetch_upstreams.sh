#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
upstream_dir="$root_dir/upstream"
mkdir -p "$upstream_dir"

fetch_pinned() {
  local name=$1
  local url=$2
  local revision=$3
  local directory="$upstream_dir/$name"
  if [[ ! -d "$directory/.git" ]]; then
    git clone "$url" "$directory"
  fi
  if [[ "$(git -C "$directory" remote get-url origin)" != "$url" ]]; then
    printf 'refusing unexpected origin for %s\n' "$directory" >&2
    exit 1
  fi
  git -C "$directory" fetch --quiet origin "$revision"
  git -C "$directory" checkout --quiet --detach "$revision"
  [[ "$(git -C "$directory" rev-parse HEAD)" == "$revision" ]]
}

fetch_pinned openarm_description https://github.com/enactic/openarm_description.git \
  6c7b720f1ba48e8bafa3a3dc752c45f397b42221
fetch_pinned openarm_can https://github.com/enactic/openarm_can.git \
  c32ecd31da267967f0c913c2118c843177d88b91
fetch_pinned openarm_ros2 https://github.com/enactic/openarm_ros2.git \
  4e837e1d0dae692ff67b560b69d8d281d7a8d4ed
fetch_pinned openarm https://github.com/enactic/openarm.git \
  990fda921c82ae9d12b00f23e449793a9a313afd
fetch_pinned openarm_hardware https://github.com/enactic/openarm_hardware.git \
  12c07510c09b2c10b7dfe48010dae5c05cbe887f
fetch_pinned openarm_teleop https://github.com/enactic/openarm_teleop.git \
  eb2d49338bf70ace95282ea724903849397b7811
fetch_pinned openarm_isaac_lab https://github.com/enactic/openarm_isaac_lab.git \
  bad82e23716e6941c2de78ccb978f57c78b37734
fetch_pinned openarm_mujoco https://github.com/enactic/openarm_mujoco.git \
  8955afb54e4adfb59a236e2b4d15192b7a02865c
fetch_pinned openarm_dataset https://github.com/enactic/openarm_dataset.git \
  2da1062f524a5b240e61e1031f637d765076569d
fetch_pinned dora-openarm https://github.com/enactic/dora-openarm.git \
  d988dd24537d670f07b6d6e85e0cdd25f0b05b82
