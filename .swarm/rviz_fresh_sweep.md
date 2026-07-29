# Fresh RViz launcher sweep

Date: 2026-07-29 (America/Los_Angeles)

## Verdict

**CLEAN.** No Critical, Important, or Minor finding remains in the RViz
launcher, GUI settings, shutdown helper, documentation, or ROS-launch
integration covered by this sweep.

The previously identified `rviz` launch-argument bug is fixed in the current
diff.  The wrapper now consumes every `rviz:=...` assignment, applies ROS's
last-value-wins ordering, accepts case-insensitive `true`/`false` and `1`/`0`,
rejects invalid or empty values, and never forwards a caller-supplied `rviz`
assignment to the core launch.  A final false value starts no RViz; a final true
value starts exactly one wrapper-managed RViz in its own process group.

## Cross-checks

- **ROS integration:** Both the headless and managed paths force
  `rviz:=false` for the core launch after removing all caller `rviz` arguments.
  Other launch arguments retain their original order and shell-safe boundaries.
- **Renderer selection:** On the observed host, Qt 6.10.2 RViz links Ogre
  1.12.10, GLX, and X11.  Forcing `QT_QPA_PLATFORM=xcb` and GLX is consistent
  with native Qt Wayland failing Ogre's GLX path.  Selecting Mesa software for
  `XDG_SESSION_TYPE=wayland` is a conservative, host-validated default because
  NVIDIA and integrated hardware GLX flicker during active resize while
  software rendering was empirically stable.  Explicit `nvidia` and
  `integrated` overrides preserve opt-in acceleration.
- **HiDPI environment:** Qt documents both
  `QT_ENABLE_HIGHDPI_SCALING=0` and `QT_SCREEN_SCALE_FACTORS=1`.  With the xcb
  backend they constrain device-pixel scaling and therefore the Ogre render
  target.  Their process-level use is consistent with the empirical fix and is
  appropriately documented as host-specific behavior.
- **Shutdown path:** RViz and the ROS launch run in separate sessions/process
  groups.  The helper finds the RViz X11/XWayland top-level window by
  `_NET_WM_PID`, sends `WM_DELETE_WINDOW`, waits for process exit, and the shell
  then stops the ROS group with bounded TERM/KILL fallbacks.  This matches the
  clean Ctrl+C result already observed.  The ctypes layouts and Xlib signatures
  are appropriate on this x86-64 host.
- **Locking:** The per-user `flock` is acquired before launch and inherited for
  the complete managed or headless lifetime, preventing concurrent wrapper
  instances from duplicating joint-state/TF authorities.
- **RViz settings and docs:** The Orbit view is well-formed, targets `world`,
  and has plausible framing for the bimanual model.  The README accurately
  describes the host-scoped software default, hardware overrides, HiDPI
  workaround, clean shutdown ordering, and single-instance behavior.
- **Static hygiene:** Fresh `git diff --check`, `bash -n scripts/launch_rviz.sh`,
  and Python AST parsing of the helper and ROS launch
  file passed.  The helper is executable, and the source RViz config matches the
  installed workspace copy.

## Scope

Read-only inspection covered `SKILL.md`, `.swarm/ledger.md`, the existing final
reports, the complete current uncommitted diff, the untracked shutdown helper,
README text, RViz config, ROS launch file, package/install declarations, and the
installed ROS 2 Lyrical/Qt argument and scaling semantics.  No build or GUI
process was run.  The only write was this report.
