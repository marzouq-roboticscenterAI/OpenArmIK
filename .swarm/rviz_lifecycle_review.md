# RViz Lifecycle Review

CLEAN

The corrected parser consumes all `rviz:=` arguments, applies last-value-wins,
accepts ROS-compatible case-insensitive `true`/`false` and `1`/`0`, rejects
invalid values, and never forwards the setting into the core launch. Thus the
enabled path starts exactly one RViz in its own session/process group, while the
disabled path starts none.

Rechecked the complete diff, Bash traps/process groups/escalation, lock
lifetime, argument arrays, Xlib ctypes ABI and WM_DELETE_WINDOW message,
renderer/scaling environment, README claims, and RViz view configuration.
Static checks passed: `bash -n`, Python AST compilation, and `git diff --check`.
