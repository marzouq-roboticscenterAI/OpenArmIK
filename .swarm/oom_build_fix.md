# Bounded build-memory fix

Date: 2026-07-29

## Delivered

- `scripts/build.sh` now resolves a positive `OPENARM_BUILD_JOBS` / `--jobs`
  budget (default `2`), runs colcon with the sequential executor, and supplies
  `MAKEFLAGS=-jN` plus `CMAKE_BUILD_PARALLEL_LEVEL=N` to suppress colcon's
  CPU-count Make arguments.
- Native builds, prefix-reuse rebuilds, and installed-consumer CMake helpers use
  the same explicit bounded job count. Native CTest execution inherits a bounded
  `CTEST_PARALLEL_LEVEL`.
- The launcher's existing `--build` behavior is retained. Its incremental call
  now preserves and reconfigures compatible native trees through
  `--reuse-build-trees`, and no longer asks colcon to clean its cache.
- Canonical mutable build resources are locked through inherited, validated file
  descriptors rather than caller-controlled environment flags. Top-level builds
  lock their output root, native build root, and install prefix before cleanup;
  direct native builds lock their build root and install prefix in deterministic
  order. This coordinates normal top-level/native calls while allowing fully
  independent roots and prefixes to proceed.
- Both GUI launchers now use the same guarded per-user runtime lock and handle
  `HUP` through their existing exact-PID, process-group shutdown routines.
  Browser ownership and motion safety behavior were not changed.

## Verification

- `bash -n` passed for every changed shell script.
- `./tests/test_build_resource_controls.sh` passed. It uses only an isolated
  temporary fixture, runs a real one-job CMake build and CTest, and uses narrow
  shims only to inspect the top-level CMake/colcon argument contract. It covers
  forged lock sentinels, canonical build-root and install-prefix contention,
  independent sibling roots, incremental reuse, and browser lock-FD closure.
- Both recursive CMake installed-consumer helpers rejected an invalid
  `OPENARM_BUILD_JOBS` before their project-specific inputs were needed.
- Fresh real build passed with `OPENARM_BUILD_JOBS=1` against disposable output
  `/tmp/openarmik-oom-lowmem-reuse.7Vzev8`; colcon event logs report successful
  sequential completion of `openarm_control_msgs`, `openarm_description`, and
  `openarm_ik_ros`. The immediate incremental rebuild completed successfully;
  its colcon event log shows only up-to-date install work. The lock was released
  after completion and `openarm_portal` was installed.
- After integrating `main` at `4d129a6`, a fresh one-job top-level build passed
  against `/tmp/openarmik-oom-ros-merged.pwnQgU`, including the production ROS
  runtime-authority archive check. The two-prefix native reuse regression also
  passed with `OPENARM_BUILD_JOBS=1`.
- Follow-up lock hardening passed `./tests/test_build_resource_controls.sh` with
  a real temporary CMake/CTest fixture and real `flock` contention. It verifies
  that forged legacy lock-sentinel variables cannot bypass either public entry
  point, aliases of one native build root contend, differing roots sharing an
  install prefix contend, and independent roots/prefixes proceed. The test
  creates and removes only its `mktemp` tree. A fresh actual two-prefix native
  reuse test also passed at one job after the FD-validated lock runner change.

No full sanitizer or broad test matrix was run. Build locks are per canonical
mutable resource, not a host-wide mutex for unrelated output roots and install
prefixes.
