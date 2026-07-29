#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

"$root_dir/scripts/install_all_dependencies.sh" --verify >/dev/null
command -v firefox >/dev/null 2>&1 || {
  printf '%s\n' 'Firefox is not installed or is not on PATH.' >&2
  exit 1
}

exec "$root_dir/scripts/launch_web_portal.sh" --build --firefox "$@"
