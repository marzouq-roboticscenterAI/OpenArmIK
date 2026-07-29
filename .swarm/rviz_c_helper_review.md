# RViz native close-helper review

## Verdict

**CLEAN.** No Critical, Important, or Minor finding in commit `33af0118`
relative to `1bfaf90`.

## Evidence

- Reviewed the complete diff and callers. The Xlib `ClientMessage` fields and
  format-32 `XGetWindowProperty` decoding are ABI-correct on this Linux LP64
  target; `_NET_CLIENT_LIST`, `_NET_WM_PID`, `WM_PROTOCOLS`, and
  `WM_DELETE_WINDOW` are used consistently with the prior working helper.
- PID parsing is bounded to positive `pid_t` values. `/proc/PID/stat` parsing
  uses the final `") "` delimiter, avoiding spaces and parentheses in `comm`,
  and treats zombies as exited. Polling uses one monotonic bounded deadline.
- The executable installs to `lib/openarm_ik_ros/close_rviz_window`, exactly the
  path resolved from `ros2 pkg prefix` by the launcher. `X11::X11` links the
  expected `libX11.so.6`; the declared `libx11-dev` rosdep key and explicit
  installer package supply headers and runtime linkage.
- The launcher validates the installed helper before starting processes. A
  helper failure still falls back to TERM and then KILL of the isolated RViz
  process group; Ctrl+C status 130 and core-process shutdown ordering are
  preserved. `rviz:=false` bypasses the helper as intended.
- The removed Python/ctypes script has no remaining caller. The close path is a
  native executable with no Python dependency; unrelated ROS launch/test Python
  dependencies remain accurately scoped.
- Fresh checks passed: `bash -n`, `git diff --check`, installed-path/prefix and
  ELF dependency inspection, helper argument/error behavior, and all 8 package
  CTest drivers after sourcing ROS/workspace setup.

The three new CTest cases cover CLI validation rather than the X11 event path.
That is not a release blocker here because the commit report includes a live
XWayland/RViz Ctrl+C run and the native behavior is a direct, audited translation
of the previously verified helper; a future hermetic X-server integration test
would improve regression coverage.
