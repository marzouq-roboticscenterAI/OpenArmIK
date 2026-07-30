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
- Canonical mutable build resources are locked by a waiting supervisor. Its
  private callback process group closes every lock descriptor before cleanup,
  CMake, CTest, colcon, or test executables run, while the supervisor retains
  ownership until that group is empty. Top-level builds lock their output root,
  native build root, and install prefix before cleanup; direct native builds
  always lock their build root and install prefix in caller order. There is no
  public internal-body mode or environment lock record.
- The top-level callback sources the native implementation directly, performing
  one native sequence while the same supervisor owns the complete
  native-plus-ROS transaction. Both public scripts accept only the repository's
  fetched `upstream/openarm_description`; the fixture-only ambient override was
  removed.
- Both GUI launchers now use the same guarded per-user runtime lock and handle
  `HUP` through their existing exact-PID, process-group shutdown routines.
  Browser ownership and motion safety behavior were not changed.

## Verification

- `bash -n` passed for every changed shell script.
- `./tests/test_build_resource_controls.sh` passed. It uses only an isolated
  temporary fixture, runs a real one-job CMake build and CTest, and uses narrow
  shims only to inspect the top-level CMake/colcon argument contract. Real
  callback/grandchild checks prove one owned process group and no inherited lock
  descriptors. Recursive requests contend, aliases and shared prefixes contend,
  independent siblings proceed, and HUP/INT/TERM return their conventional
  statuses with no surviving group before immediate reacquisition.
- Copied miniature repositories prove the pinned description is used even with
  a valid decoy in the environment, and that both public scripts fail before
  mutation when the pinned checkout is absent. The regression also covers
  top/native composition, incremental reuse, non-destructive early validation,
  and unmanaged-browser lock-FD closure.
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
- The supervisor forwards and, after a bounded grace interval, escalates only
  against its exact callback PGID. It reaps the leader, retains locks until the
  group is empty, and restores caller signal traps and monitor mode. The test
  creates and removes only its `mktemp` tree.
- Lock-directory setup now accepts an XDG runtime base only after explicit
  owner/type/mode checks, otherwise validates the root-owned sticky `/tmp`
  base. Its fixed per-user child is created atomically with mode 0700 and is
  revalidated without a chmod repair before any lock file is opened. Fixture
  tests cover owner mismatch, permissive modes, insecure XDG fallback, and a
  deterministic mkdir-time symlink race without touching the victim directory.
- Top-level read-only validation again checks the xacro executable and Python
  package directory before acquiring locks or performing clean mutations.

No full sanitizer or broad test matrix was run. Build locks are per canonical
mutable resource, not a host-wide mutex for unrelated output roots and install
prefixes.
