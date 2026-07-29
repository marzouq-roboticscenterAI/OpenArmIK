# Compiled local OpenArm portal implementation

Date: 2026-07-29 (America/Los_Angeles)
Branch: `feat/portal-server`
Baseline: `527d25d`
Status: **DONE_WITH_CONCERNS**

## Implemented contract

- Added the installed C++17 `openarm_portal` executable expected by
  `scripts/launch_web_portal.sh`. The existing root `run.sh` remains the single
  build-and-launch command and requests Firefox through that launcher.
- The HTTP acceptor constructs only an IPv4 `127.0.0.1` endpoint. It has no
  non-loopback mode, shell execution, external assets, Python portal runtime,
  CAN, transport, commission, persistence, or physical-control dependency.
- The responsive embedded page displays controls on the left and JPEG snapshots
  of actual launcher-owned stock RViz pixels on the right. Capture recursively
  identifies exactly one mapped top-level X11 window by `WM_STATE` and exact
  `_NET_WM_PID`, requires the `/proc/PID/exe` basename `rviz2`, revalidates
  `/proc/PID/stat` start ticks and window uniqueness before every frame, uses
  XComposite redirected pixmaps, bounds
  dimensions, traps X11 errors around resize/destroy races, and encodes with
  libjpeg.
- State comes only from `/joint_states`. All fourteen canonical public model
  names and finite positions are required, both producer stamp and receipt must
  be newer than 500 ms, and current controller diagnostics must report virtual,
  physical unauthorized, exact expected/fresh masks, zero fault masks, and
  collision unchecked. Both TCPs are recomputed with public `oa_fk`. The two
  XYZ forms are initially populated from that measured state.
- A Left or Right request snapshots both arms once, guards the selected target,
  and sends one `MovePairedTcp` goal with the other target set to its freshest
  measured TCP. Browser values are never used for the opposite target. A state
  generation is rechecked after guard evaluation and before send, rejecting an
  intervening state update. Action feedback displays measured progress; the
  terminal text displays the controller's `collision_checked` result.
- The pure nominal guard validates finite targets and measured joint/model
  bounds, uses measured seeds and public `oa_ik_position_v2`/`oa_fk` at 17
  sampled Cartesian waypoints, requires branch continuity, checks conservative
  left/right link and tool capsules, and checks both arms against a central
  nominal body/pole cylinder plus 25 mm clearance. Both sides are re-solved at
  each sample to mirror paired-plan behavior. Any IK/FK/numeric/bounds/geometry
  uncertainty rejects the request; inputs are never clamped.
- No recommended target is exposed. The forms use current measured TCPs, so no
  preset is represented as guard-approved before it is evaluated against the
  live measured seed.
- The page and mutation/result responses explicitly say the guard is sampled
  nominal virtual protection, not physical certification, and that the current
  controller still reports `collision_checked=false`. There is no speed slider.
- “Auto Calibrate — simulation verification only” performs only fresh-state and
  public-FK verification and returns the fixed truth that no physical
  calibration occurred. “Request software stop (not a hardwired E-stop)”
  cancels an active or acceptance-pending portal action and is explicitly not
  safety-rated.
- Mutations are exact POST routes with a 512-byte parser limit, exact
  `application/json`, strict finite-number schema, exact same-origin Host and
  Origin, and a random per-process 256-bit CSRF token delivered only in the
  embedded page. Responses disable caching; the page has CSP, nosniff,
  no-referrer, and frame-ancestor denial.
- The server uses a fixed four-worker HTTP pool with at most eight admitted
  requests, one accept thread, and one ROS
  executor thread. Capture is serialized, while stop requests can bypass an
  in-progress JPEG or guard request. SIGINT/SIGTERM immediately latch motion
  rejection, stop acceptance, request cancellation before draining bounded
  workers, wait up to two seconds for a terminal result, then shut ROS down and
  join every bounded thread.

## Verification evidence

- The complete native stack and three ROS packages compiled under
  `/tmp/openarmik-portal-server-build`, and CMake installed
  `install/openarm_ik_ros/lib/openarm_ik_ros/openarm_portal`. A second fresh
  package build from an empty path, `/tmp/openarmik-portal-server-clean-package`,
  compiled every package target after the final review fixes.
- `openarm_portal` compiled with `-Wall -Wextra -Wpedantic -Werror` and links
  X11, XComposite, libjpeg, ROS, the action interfaces, and the public model.
  `ldd` showed no CAN, transport, commission, or Python library.
- An escalated headless package run used loopback DDS and ptrace solely for the
  existing no-CAN syscall test: **10/10 passed in 70.79 seconds**. After the
  final review fixes, the fresh-package aggregate passed 9/10; one unchanged
  paired virtual-session case hit a scheduler-sensitive native deadline
  (`advance_failed status=5`). Its exact single case then passed in 20.705 s,
  and the complete 13-case virtual-session executable passed on immediate rerun
  in 24.69 s. The other final aggregate tests—portal core, no-CAN linkage and
  syscalls, generated URDF, invalid configuration, live ROS contract, active
  SIGINT, and RViz close helpers—all passed.
- `test_portal_core` deterministically covers strict JSON rejection, Host /
  Origin / CSRF / content-type / length policy, nonfinite/unreachable guard
  rejection, an accepted public-FK/IK stationary regression pose with at least
  25 mm nominal clearance, exact PID/start-ticks identity, and JSON escaping.
- `git diff --check` passed after the final source changes.
- No GUI, RViz, Firefox, browser, portal process, screenshot, X display, CAN
  interface, commissioning, hardware, or physical command was started.

## Limitations and release concerns

- XComposite capture of the live Ogre render child, occlusion independence,
  resize/minimize recovery, frame cadence, and Firefox presentation require an
  explicit acceptance run in the logged-in graphical session. They were not
  and must not be inferred from this headless build.
- The nominal guard is deliberately not a continuous collision certificate.
  Its capsule and central-cylinder proxies are conservative product policy,
  not surveyed installation geometry or a calibrated full-mesh scene. It may
  reject useful paths, and it cannot authorize physical motion.
- The underlying paired controller reports collision unchecked. The portal
  guard does not change that controller truth, and the virtual adapter has no
  hardware backend.
- The software stop is an action cancellation path. Browser scheduling, HTTP,
  the portal process, ROS, and controller serialization remain failure paths;
  it cannot replace a hardwired E-stop.
- The paired controller independently solves both TCP targets. The portal
  truthfully promises only that the opposite target is populated from its
  freshest measured TCP; it does not claim a joint-posture freeze.
