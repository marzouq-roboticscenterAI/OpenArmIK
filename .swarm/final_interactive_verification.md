# Final independent interactive verification — CLEAN

Initially verified at `bd06a301095243325bbe4cf6fc3e94cdf702653f`, then
reverified from the rebuilt production tree at
`8fc4b16ef30c7034c6ba13a0c1e149abdd658314` on 2026-07-30. I did not edit
production, stage files, or `transport/tests/test_transport.cpp`, and did not
open CAN or transmit to physical hardware. Existing/shared worktree changes
were preserved.

## Previously Important finding — resolved

At `bd06a30`, a successful production launch imported the installed Python
launch file and created:

`ros2_ws/install/openarm_ik_ros/share/openarm_ik_ros/launch/__pycache__/openarm_ik_rviz.launch.cpython-314.pyc`

and the next `--no-build` launch failed integrity validation. Commit `8fc4b16`
exports `PYTHONDONTWRITEBYTECODE=1` before the ROS launch subprocess and adds
fixture coverage for repeated installed-launch imports and launch gates.

The rebuilt `run_tests=0` tree closes the finding with real production
evidence:

- Baseline launch-stamp SHA-256:
  `83d8ba0fb01688fd7263d44023e2cdd1efa667855c5c8a1ca4fa44b3efa91cc2`.
- Baseline complete install-manifest digest:
  `39a41ce9c8c24a690b9abd7214fdccc58d641055d21ff13d8b377fb54ae5c378`.
- Baseline installed launch-tree Python cache count: zero.
- Two consecutive real commands using ports 38548 and 38549 both passed the
  production `--no-build` integrity gate, reached healthy loopback portal
  service, and shut down through Ctrl+C with status 130 and clean ROS child
  exits.
- After launch one and again after launch two, the stamp hash and install
  manifest exactly matched the baselines above. The installed launch tree still
  contained no `__pycache__`, `*.pyc`, or `*.pyo` entry.
- A direct post-run `openarm_assert_current_launch_tree` passed. No verifier
  child, port-38548/38549 listener, or RViz process remained.

The production self-mutation release blocker is therefore resolved.

## Green evidence

- `git diff --check 658174a..HEAD` passed. The integrated range contains the
  portal viewer, nine presets per arm, viewer integrity generation, Firefox
  oracle, and launcher/integrity changes expected from the feature.
- A fresh current-source, test-enabled isolated ROS build succeeded under
  `build/final-review-verification`. `ctest -N` registered exactly 15 tests,
  numbered 1 through 15.
- Fresh focused CTest: `test_virtual_control_session` passed in 161.64 s and
  `test_portal_core` passed in 0.97 s. The former includes
  `AllPortalTargetsCompleteFromFreshMeasuredFeedback`; the latter includes the
  exact 10 x 10 x 2 x 9 = 1,800 endpoint/cross-state guard matrix and its
  asserted minimum clearance of at least 0.0265278 m.
- Fresh Firefox 153 browser/security test passed in 6.21 s when run with the
  real user-session runtime directory. It parsed the URDF, proved resident
  served bytes after on-disk mutation, kept authenticated stop responsive with
  eight slow static clients (10 ms) and sixteen partial bodies (13 ms),
  reclaimed FDs/threads, completed the actual WebGL transform/camera oracle,
  reported 23 instances, 11 buffers, 1,798,416 GPU bytes, and zero camera
  requests, shut down partial-body intake in 114 ms, and rejected the mutated
  copied prefix by SHA-256 on restart. Initial verifier attempts using an
  isolated private `XDG_RUNTIME_DIR` could not create Firefox WebGL2; restoring
  `/run/user/1000` produced the clean checked-in-test pass.
- `test_visualization_urdf` and `test_viewer_configure_dependency` passed. The
  dependency test proves a same-size pinned-license mutation forces
  reconfiguration and fails closed.
- `tests/test_launch_integrity.sh` passed (`Launch freshness and authority
  regression passed`), and `tests/test_build_resource_controls.sh` passed
  (`supervisor, pinned fixture, real CMake/CTest`). The launch-integrity test was
  rerun at `8fc4b16` and passed its new no-bytecode/repeated-gate regression.
- The installed Runtime V1 header equals its frozen header. The installed
  `libopenarm_runtime.a` passed the checked-in ABI-symbol script with exactly 50
  unique `oa_runtime_*` text symbols versus 50 expected. The freshly built
  VirtualControlSession archive has Runtime-only undefined API references and
  no direct `oa_controller_*`, `oa_motion_plan_*`, or `oa_manifest_*` bypass.
- Direct log evidence from the first production run is in
  `/home/signalprocessing-dev/.ros/log/2026-07-30-15-40-29-586164-Legion-Pro-5-16ADR10-2807465/launch.log`.
  It started only `robot_state_publisher` and `openarm_ik_ros_node` (no RViz
  child), recorded SIGINT (`signum=2`) in both node logs, and reports both
  processes finished cleanly at 15:44:18. This is historical evidence from the
  pre-fix run; the two post-fix cycles below supersede its failed follow-up gate.
- Reverification logs for the two fixed production launches are under
  `build/final-review-verification/ros-log/2026-07-30-16-02-01-603811-Legion-Pro-5-16ADR10-2918244/launch.log`
  and
  `build/final-review-verification/ros-log/2026-07-30-16-03-03-012940-Legion-Pro-5-16ADR10-2935732/launch.log`.
  Each records only robot-state-publisher and the virtual IK node as ROS
  children, then reports both finished cleanly on SIGINT.

## Stationary normal viewer

With no pointer input, motion request, or measurement injection, each real
portal launch returned advancing view-state sequences while all 14 measured
joint positions remained exactly unchanged at
`0.000066758220798490697` rad. On the second launch, for example, sequences
1417 and 1616 had byte-for-byte equal `position_rad` arrays and both were fresh.
The viewer camera begins at fixed yaw/pitch/distance constants; yaw and pitch
are modified only inside active pointer-move handling, distance only by pinch,
wheel, or reset. `requestAnimationFrame(draw)` redraws the same pose/camera and
does not increment any orbit angle. Normal portal display is therefore
stationary absent input. The prior constant rotation was the explicit
FPS-verifier measured-state injection, which is closed and was not part of
normal startup.

## Visible Firefox measurement interpretation

The reported headed measurement is internally consistent:

- `3601 * 1000 / 60007.02 = 60.009645538 FPS` (reported 60.0096 FPS), with a
  mean interval of 16.66399 ms.
- p99 17.14 ms and maximum 17.18 ms are plausible and consistent with that
  near-60 Hz `requestAnimationFrame` cadence.
- The recorded backing size was 1920 x 1080 and page visibility was `visible`.

This demonstrates foreground JavaScript/WebGL draw/rAF cadence, not measured
physical panel scanout or a compositor-present guarantee. Normal startup is
stationary. The separately observed continuous orbit came from the verifier's
external measured-state injection; that injection is closed and no such
process remains.

## Verdict

**CLEAN.** The viewer, presets, safety matrix, Firefox/security behavior,
Runtime authority/ABI, resource controls, stationary normal behavior, repeated
production launch integrity, and Ctrl+C shutdown are green. The earlier
installed Python-bytecode self-mutation is resolved by `8fc4b16` and the rebuilt
tree passed two consecutive real launch/stop cycles without stamp, manifest, or
installed-launch-tree drift.
