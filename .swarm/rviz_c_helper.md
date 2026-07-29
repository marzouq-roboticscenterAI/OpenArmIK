# RViz C++ lifecycle helper

Date: 2026-07-29
Branch: `impl/rviz-c-helper`

## Result

The Python/ctypes runtime helper was replaced by the C++17
`close_rviz_window` executable in `openarm_ik_ros`. It uses Xlib to find the
X11/XWayland top-level window whose `_NET_WM_PID` matches RViz, sends the same
`WM_PROTOCOLS` / `WM_DELETE_WINDOW` client message, and waits on the same
process-exit-or-zombie condition. The timeout is monotonic, polled at no more
than 50 ms, and limited to 0 through 3600 seconds.

The ROS package installs the helper to
`lib/openarm_ik_ros/close_rviz_window`. The launcher resolves that installed
libexec path through the package prefix and fails with a rebuild instruction if
it is missing. `libx11-dev` is declared in both `package.xml` and the explicit
dependency installer. The removed `scripts/close_rviz_window.py` is no longer
used.

## Verification

- Package build passed with GCC 15.2 and `-Wall -Wextra -Wpedantic -Werror`.
- Package tests passed: 8 CTest drivers, 13 test cases, 0 errors, 0 failures,
  0 skipped. This includes compiled-helper help, invalid-PID, and bounded-timeout
  checks plus all existing ROS adapter tests.
- `bash -n scripts/launch_rviz.sh`, `xmllint --noout package.xml`, and
  `git diff --check` passed.
- A final live XWayland RViz launch used the installed compiled helper. Ctrl+C
  returned exit 130 in 0.62 seconds; RViz closed through its window manager and
  both `openarm_ik_ros_node` and `robot_state_publisher` reported clean exits.
- Final process inspection found no RViz, ROS launch, adapter, or state-publisher
  residue.

The four finger-inertia RViz diagnostics appeared as expected from the pinned
upstream URDF and are unrelated to lifecycle shutdown.
