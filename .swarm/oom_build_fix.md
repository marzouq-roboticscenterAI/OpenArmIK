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
- A nonblocking canonical-output-root lock now wraps the entire top-level build,
  including cleanup. Direct native builds use the same identity when they share
  that native build root. The lock is held by `flock --close`, so build
  descendants cannot accidentally retain it after the launcher script exits.
- Both GUI launchers now use the same guarded per-user runtime lock and handle
  `HUP` through their existing exact-PID, process-group shutdown routines.
  Browser ownership and motion safety behavior were not changed.

## Verification

- `bash -n` passed for every changed shell script.
- `./tests/test_build_resource_controls.sh` passed. It uses shims (no sanitizer
  suite) to verify invalid job values preserve outputs, native `--parallel N`,
  sequential colcon with `MAKEFLAGS=-j2`, incremental tree/cache reuse,
  same-root locking through a symlink alias, the shared GUI lock without
  starting GUI processes, and the retained `run.sh --build` ordering.
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

No full sanitizer or broad test matrix was run.
