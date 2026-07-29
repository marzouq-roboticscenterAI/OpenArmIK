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
