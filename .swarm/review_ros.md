# Final independent ROS adapter re-review

Reviewed cumulative `impl/ros-adapter` through `1246e7f` against `main`, the original user goal, `.swarm/ros_compatibility.md`, both design critiques, and every finding from the prior review. No implementation was modified, no CAN interface or robot hardware was touched, and no `sudo` command was run.

## Verdict: CLEAN

No Critical, Important, or Minor findings remain in the reviewed scope. The four finger-inertia messages are inherited from the exact pinned upstream URDF, do not prevent rendering, and are now explicitly documented.

## Prior-finding closure

- **Expiry overflow and boundaries:** `request_expiry_ms` is validated in `[1, 60000]` before safe conversion. The processor independently rejects invalid durations and uses ordered positive timestamp subtraction, avoiding signed overflow. Unit tests cover exact stale/future boundaries, zero/negative timestamps, invalid clocks, and `INT64_MAX`; ROS tests reject `0`, `-1`, `60001`, and `INT64_MAX`. Under UBSan, the old reproducer now cleanly rejects with `std::invalid_argument`; `60000` starts normally.
- **One-shot helper:** the CLI waits for target-subscriber discovery, stamps after discovery, publishes once, and waits for a diagnostic acknowledgement. Live invocations produced exactly sequence 1 for success and sequence 2 for the following failure—no duplicate transactions.
- **Redundant-DOF contract:** the immutable `continuity-v1` policy is documented, named in every diagnostic, and exposes its last-committed seed/posture rule plus tolerance, damping, step, posture weight, and iteration settings. Orientation-free/no-collision limitations remain explicit in README, logs, and diagnostics.
- **Failure diagnostics:** `committed` and `achieved_available` are explicit. Residual/TCP fields are emitted only for a committed pair; an unreachable-left/reachable-right request returned `committed=false`, `achieved_available=false`, both solver statuses, and no fake residual or transform fields. The prior coherent JointState remained unchanged.
- **Integration and coverage:** committed tests now launch the real headless ROS graph, verify exact JointState order, sole JointState/TF publishers, QoS connectivity, paired success/failure retention, diagnostic fields, and TCP TF accuracy. A reproducible gcov path exists. Fresh authored line coverage measured 97.87% for `paired_transaction.cpp` and 99.13% for `openarm_ik_ros_node.cpp`; gcov evaluated 100% and 91.35% of their branch sites respectively (69.57% and 49.73% taken at least once). The sole uncovered node line was the uncaught invalid-configuration throw, which is exercised in subprocess tests but cannot flush gcov after abort.
- **Installable model/CMake portability:** `model/` installs `openarm_model::openarm_model`, public headers, frozen URDF, license/notice, config, version, and export files without source-tree paths. `cmake --find-package` found the installed package, and a clean ROS build consumed only the installed model prefix and installed `openarm_description`; the ROS package no longer compiles or tests against monorepo-relative model/upstream paths.
- **Header, dependency, and Snap cleanup:** the ROS package no longer installs an unusable private header; `ament_index_python` and ROS integration-test dependencies are declared; the RViz wrapper clears the remaining Snap-owned GTK/GDK/XDG paths. The desktop launch emitted no prior GTK/canberra warning.
- **Pinned inertia caveat:** README now records the four canonical finger-inertia errors and correctly limits their impact to RViz's equivalent inertia boxes. Meshes, RobotModel, and TF load normally.

## Verification evidence

- **Strict model regression:** clean Debug GCC 15.2 build with `-Wall -Wextra -Wpedantic -Werror`, ASan+UBSan, system Python, and deterministic pinned-xacro inputs passed all 4 tests: C model, ABI-v1 canary, independent Python/URDF reference, and generator byte determinism. Install completed successfully.
- **Clean ROS 2 Lyrical:** from empty temporary build/install directories, the installed standalone model, pinned `openarm_description`, and `openarm_ik_ros` configured and built. All 5 registered adapter tests passed: 5 C++ transaction cases, generated URDF/mesh validation, ldd/strace isolation, real headless ROS contract, and invalid expiry parameters.
- **ROS sanitizers:** a separate ASan+UBSan Lyrical build passed the same 5 registered tests with leak and UB halting enabled.
- **Coverage:** a separate clean gcov-instrumented build passed all 5 tests and generated reports for both authored adapter translation units. Authored line coverage was 92/94 and 114/115 respectively.
- **Live transaction/TF:** the one-shot helper committed `(0.20, 0.30, 0.85)` and `(0.20, -0.30, 0.85)` with approximately `6.06e-8 m` residuals. `tf2_echo` reported the left TCP at `[0.200, 0.300, 0.850]`; the automated contract independently checked both TCPs against diagnostic matrices within `1e-6 m`. A subsequent unreachable-left request was sequence 2, rejected atomically, and left all 16 JointState positions unchanged.
- **Exact state/TF ownership:** JointState order remained left joints 1-7, right joints 1-7, then the two independent `finger_joint1` drivers; mimic joints were omitted. The graph had one `/joint_states` publisher (adapter) and one `/tf` publisher (`robot_state_publisher`).
- **No physical access:** sourced `ldd` contained no OpenArm CAN/SocketCAN library. A fresh two-second socket `strace` contained DDS/route sockets but no `AF_CAN`, `PF_CAN`, or `CAN_RAW`.
- **Desktop RViz:** RViz, adapter, and robot-state-publisher remained alive; RViz reported OpenGL 4.6 / GLSL 4.6, loaded the RobotModel without mesh/material/package errors, and all processes exited cleanly on SIGINT. Only the four documented pinned finger-inertia errors appeared.
- **Workspace state:** `impl/ros-adapter` remained clean at `1246e7f` after review.
