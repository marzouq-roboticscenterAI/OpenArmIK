#!/usr/bin/env bash
set -eo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
for variable in SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA SNAP_INSTANCE_KEY SNAP_LIBRARY_PATH SNAP_NAME SNAP_REAL_HOME SNAP_REEXEC SNAP_REVISION SNAP_USER_COMMON SNAP_USER_DATA GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_MODULE_DIR QT_PLUGIN_PATH QT_QPA_PLATFORMTHEME XDG_DATA_DIRS; do
  unset "$variable" || true
done
source /opt/ros/lyrical/setup.bash
source "$root_dir/ros2_ws/install/setup.bash"
set -u
exec ros2 launch openarm_ik_ros openarm_ik_rviz.launch.py "$@"
