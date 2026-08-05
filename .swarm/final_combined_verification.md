# Final combined verification

- Verdict: **CLEAN**
- Target: `53bfd809a7bd81c753fcdd354c1412339b9d2f1a`
- Completed: `2026-07-29T20:05:54-07:00`
- Fresh output root: `/tmp/openarmik-final-verification.bdAKMg`
- Resource policy: `OPENARM_BUILD_JOBS=1`,
  `CMAKE_BUILD_PARALLEL_LEVEL=1`, `MAKEFLAGS=-j1`,
  `CTEST_PARALLEL_LEVEL=1`; top-level colcon used its sequential executor.
- Safety: no GUI, browser, RViz, physical CAN, or transmit path was run.

## Baseline and preservation

`git status --short` before verification contained only the explicitly preserved
paths:

```text
 M transport/tests/test_transport.cpp
?? .swarm/oom_build_audit.md
?? .swarm/oom_design_review.md
```

Those paths were not changed or removed. The final report itself is the only
additional untracked path. No production file or commit was modified.

## Gates

1. **Fresh-output top-level build: PASS.** On an empty `mktemp -d` output root,
   `./scripts/build.sh --incremental --tests --jobs 1 --output-root ...` built
   and installed CAN, model, commission, transport, control, runtime, the
   installed native consumers, `openarm_control_msgs`, the pinned
   `openarm_description`, and `openarm_ik_ros`. Colcon completed all three ROS
   packages sequentially with `BUILD_TESTING=ON` and registered exactly 13 ROS
   CTests.

2. **Native tests: PASS (16/16).** CAN 1/1, model 4/4, commission 2/2,
   transport 3/3, control 4/4, and runtime 2/2 all passed. The runtime suite,
   including `physical_observation_is_fail_closed`, passed.

3. **Installed consumers: PASS (4/4).** The installed C11 and C++17 all-header
   consumers and the C11 and C++17 Runtime-only consumers all executed with
   status 0. The physical-backend Runtime-only C11 consumer specifically
   verified that physical register-query capability is absent,
   `oa_runtime_inventory_query()` returns `OA_RUNTIME_EUNSUPPORTED` while
   clearing the output pointer, and physical configuration apply returns
   `OA_RUNTIME_EUNSUPPORTED`.

4. **Physical-query syscall audit: PASS.** The Runtime-only physical consumer
   was run under `strace` for `socket`, send/receive message, and send/receive
   datagram syscalls. The trace file had zero lines. No physical CAN interface
   or TX action was used.

5. **Export/symbol/no-CAN audits: PASS.** All six installed archives and CMake
   package consumers resolved. The installed runtime exposed no test-only
   symbols and had no undefined `oa_can_*`, `oa_transport_*`, `socket`, `send`,
   or `recv` references. The installed transport did not define embedded
   `oa_can_*` implementation symbols. The ROS session archive had an undefined
   `oa_runtime_create` reference and no direct `oa_controller_*`,
   `oa_motion_plan_*`, or `oa_manifest_*` bypass references. Runtime-only `ldd`
   output had no Python, CAN, or transport dependency.

6. **Bounded-memory/resource controls: PASS.** With the one-job outer
   environment, `tests/test_build_resource_controls.sh` passed its supervisor,
   pinned-fixture, real-CMake/CTest, lock noninheritance/contention, cleanup,
   incremental reuse, and job-propagation checks.

7. **ROS full CTest: PASS (13/13).** The freshly installed current runtime and
   action-message overlays were sourced. Direct CTest ran sequentially with
   `ROS_DOMAIN_ID=93` and isolated
   `ROS_LOG_DIR=/tmp/openarmik-final-verification.bdAKMg/ros_logs/full_ctest`.
   All 13 tests passed in 97.80 seconds, including ROS contract, lifecycle,
   active SIGINT, and no-CAN linkage.

8. **Focused ROS session: PASS (16/16).** The distinct
   `test_virtual_control_session` GTest binary listed and passed all 16 tests in
   29.086 seconds under a separate log directory/domain.

9. **Safe launcher/build/dependency paths: PASS.** `./run.sh --help` and
   `./scripts/build.sh --help` both returned 0 without launching a GUI or
   browser. The default review-only `./scripts/install_all_dependencies.sh`
   returned 0 and explicitly installed nothing.

10. **Syntax/diff/process/memory: PASS.** `bash -n` passed for `run.sh`,
    `scripts/*.sh`, `scripts/lib/*.sh`, and `tests/*.sh`; `git diff --check`
    returned 0. No colcon, CMake, CTest, make/ninja, ROS, RViz, OpenArm node,
    portal, CLI, or robot-state-publisher process remained. Final available
    memory was 8.2 GiB with only 944 KiB swap in use.

